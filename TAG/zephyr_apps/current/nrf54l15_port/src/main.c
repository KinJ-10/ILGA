/* main.c - BLE sample + BMI270 streaming (Notify)
 *
 * This file is created by merging:
 * - BLE base (マージ用_BLE.txt)
 * - BMI270 polling sample (マージ用_BMI.txt)
 *
 * What this does:
 * - Starts BLE advertising (based on the BLE sample)
 * - Keeps the sample services (BAS/HRS/CTS/IAS + Vendor service)
 * - Adds one combined BMI270 NOTIFY characteristic:
 *     version(uint8) + seq(uint32 LE) + accel(3 x int16 LE, mg)
 *     + gyro(3 x int32 LE, mdps)
 * - Polls BMI270 on an absolute 10 ms schedule and queues BLE transmission
 *   to a separate thread so Bluetooth backpressure cannot stop acquisition.
 *
 * Notes:
 * - DEVICE_DT_GET_ONE(bosch_bmi270) will compile-time fail if BMI270 node
 *   is not present in Devicetree. To avoid that, this code guards it with
 *   DT_HAS_COMPAT_STATUS_OKAY(bosch_bmi270).
 * - Make sure your overlay enables BMI270 under &arduino_i2c and i2c1 buffer.
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>

#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/kernel.h>

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/devicetree.h>

#include <zephyr/settings/settings.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/bluetooth/services/cts.h>
#include <zephyr/bluetooth/services/hrs.h>
#include <zephyr/bluetooth/services/ias.h>

/* ============================================================
 * BMI270 helper (from マージ用_BMI.txt)
 * ==========================================================*/

/* ============================================================
 * BLE base (from マージ用_BLE.txt)
 * ==========================================================*/

/* Custom Service Variables */
#define BT_UUID_CUSTOM_SERVICE_VAL \
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)

static const struct bt_uuid_128 vnd_uuid = BT_UUID_INIT_128(
	BT_UUID_CUSTOM_SERVICE_VAL);

static const struct bt_uuid_128 vnd_enc_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1));

static const struct bt_uuid_128 vnd_auth_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef2));

/* Legacy READ-only characteristics. */
static const struct bt_uuid_128 bmi_acc_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef11));

static const struct bt_uuid_128 bmi_gyr_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef12));

/* Combined BMI270 stream UUID. */
static const struct bt_uuid_128 bmi_stream_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef13));

#define BMI_STREAM_VERSION 1U
#define BMI_STREAM_PAYLOAD_SIZE 23U
#define BMI_BLE_QUEUE_CAPACITY 64U

#define VND_MAX_LEN 20
#define BT_HR_HEARTRATE_DEFAULT_MIN 90U
#define BT_HR_HEARTRATE_DEFAULT_MAX 160U

static uint8_t vnd_value[VND_MAX_LEN + 1] = {'V', 'e', 'n', 'd', 'o', 'r'};
static uint8_t vnd_auth_value[VND_MAX_LEN + 1] = {'V', 'e', 'n', 'd', 'o', 'r'};
static uint8_t vnd_wwr_value[VND_MAX_LEN + 1] = {'V', 'e', 'n', 'd', 'o', 'r'};

/* Last BMI data (for READ) */
static uint8_t bmi_acc_last[10]; /* seq u32 + 3x i16 = 10 bytes */
static uint8_t bmi_gyr_last[16]; /* seq u32 + 3x i32 = 16 bytes */
static uint8_t bmi_stream_last[BMI_STREAM_PAYLOAD_SIZE];

struct bmi_sample
{
	uint32_t seq;
	int16_t ax_mg;
	int16_t ay_mg;
	int16_t az_mg;
	int32_t gx_mdps;
	int32_t gy_mdps;
	int32_t gz_mdps;
};

K_MSGQ_DEFINE(bmi_ble_queue, sizeof(struct bmi_sample),
		 BMI_BLE_QUEUE_CAPACITY, 4);
K_MUTEX_DEFINE(current_conn_mutex);

static atomic_t bmi_stream_notify_enabled;
static atomic_t bmi_stream_connected;
static atomic_t bmi_stream_notify_last_ret = ATOMIC_INIT(INT32_MIN);

#define ADV_RESTART_DELAY_MS 200

/* attribute pointers (set at runtime with bt_gatt_find_by_uuid) */
static const struct bt_gatt_attr *bmi_acc_attr;
static const struct bt_gatt_attr *bmi_gyr_attr;
static const struct bt_gatt_attr *bmi_stream_attr;

/* keep current connection */
static struct bt_conn *current_conn;

static ssize_t read_vnd(struct bt_conn *conn, const struct bt_gatt_attr *attr,
						void *buf, uint16_t len, uint16_t offset)
{
	const char *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
							 strlen(value));
}

static ssize_t write_vnd(struct bt_conn *conn, const struct bt_gatt_attr *attr,
						 const void *buf, uint16_t len, uint16_t offset,
						 uint8_t flags)
{
	uint8_t *value = attr->user_data;

	if (offset + len > VND_MAX_LEN)
	{
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);
	value[offset + len] = 0;

	return len;
}

/* READ handlers for BMI characteristics */
static ssize_t read_bmi_acc(struct bt_conn *conn, const struct bt_gatt_attr *attr,
							void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset,
							 bmi_acc_last, sizeof(bmi_acc_last));
}

static ssize_t read_bmi_gyr(struct bt_conn *conn, const struct bt_gatt_attr *attr,
							void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset,
							 bmi_gyr_last, sizeof(bmi_gyr_last));
}

static ssize_t read_bmi_stream(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				       void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 bmi_stream_last, sizeof(bmi_stream_last));
}

static void bmi_stream_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	bool enabled = (value == BT_GATT_CCC_NOTIFY);

	atomic_clear(&bmi_stream_notify_enabled);
	k_msgq_purge(&bmi_ble_queue);
	atomic_set(&bmi_stream_notify_last_ret, INT32_MIN);
	if (enabled)
	{
		atomic_set(&bmi_stream_notify_enabled, 1);
	}
	printk("BMI stream notify %s\n", enabled ? "ENABLED" : "DISABLED");
}

/* Vendor service indication simulation (original) */
static uint8_t simulate_vnd;
static uint8_t indicating;
static struct bt_gatt_indicate_params ind_params;

static void vnd_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	simulate_vnd = (value == BT_GATT_CCC_INDICATE) ? 1 : 0;
}

static void indicate_cb(struct bt_conn *conn,
						struct bt_gatt_indicate_params *params, uint8_t err)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(params);
	printk("Indication %s\n", err != 0U ? "fail" : "success");
}

static void indicate_destroy(struct bt_gatt_indicate_params *params)
{
	ARG_UNUSED(params);
	printk("Indication complete\n");
	indicating = 0U;
}

#define VND_LONG_MAX_LEN 74
static uint8_t vnd_long_value[VND_LONG_MAX_LEN + 1] = {
	'V',
	'e',
	'n',
	'd',
	'o',
	'r',
	' ',
	'd',
	'a',
	't',
	'a',
	'1',
	'V',
	'e',
	'n',
	'd',
	'o',
	'r',
	' ',
	'd',
	'a',
	't',
	'a',
	'2',
	'V',
	'e',
	'n',
	'd',
	'o',
	'r',
	' ',
	'd',
	'a',
	't',
	'a',
	'3',
	'V',
	'e',
	'n',
	'd',
	'o',
	'r',
	' ',
	'd',
	'a',
	't',
	'a',
	'4',
	'V',
	'e',
	'n',
	'd',
	'o',
	'r',
	' ',
	'd',
	'a',
	't',
	'a',
	'5',
	'V',
	'e',
	'n',
	'd',
	'o',
	'r',
	' ',
	'd',
	'a',
	't',
	'a',
	'6',
	'.',
	' ',
};

static ssize_t write_long_vnd(struct bt_conn *conn,
							  const struct bt_gatt_attr *attr, const void *buf,
							  uint16_t len, uint16_t offset, uint8_t flags)
{
	uint8_t *value = attr->user_data;

	if (flags & BT_GATT_WRITE_FLAG_PREPARE)
	{
		return 0;
	}

	if (offset + len > VND_LONG_MAX_LEN)
	{
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);
	value[offset + len] = 0;

	return len;
}

static const struct bt_uuid_128 vnd_long_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef3));

static struct bt_gatt_cep vnd_long_cep = {
	.properties = BT_GATT_CEP_RELIABLE_WRITE,
};

static int signed_value;

static ssize_t read_signed(struct bt_conn *conn, const struct bt_gatt_attr *attr,
						   void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset,
							 &signed_value, sizeof(signed_value));
}

static ssize_t write_signed(struct bt_conn *conn, const struct bt_gatt_attr *attr,
							const void *buf, uint16_t len, uint16_t offset,
							uint8_t flags)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(flags);

	if (offset + len > sizeof(signed_value))
	{
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy((uint8_t *)&signed_value + offset, buf, len);

	return len;
}

static const struct bt_uuid_128 vnd_signed_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x13345678, 0x1234, 0x5678, 0x1334, 0x56789abcdef3));

static const struct bt_uuid_128 vnd_write_cmd_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef4));

static ssize_t write_without_rsp_vnd(struct bt_conn *conn,
									 const struct bt_gatt_attr *attr,
									 const void *buf, uint16_t len, uint16_t offset,
									 uint8_t flags)
{
	uint8_t *value = attr->user_data;

	if (!(flags & BT_GATT_WRITE_FLAG_CMD))
	{
		return BT_GATT_ERR(BT_ATT_ERR_WRITE_REQ_REJECTED);
	}

	if (offset + len > VND_MAX_LEN)
	{
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);
	value[offset + len] = 0;

	return len;
}

/* Vendor Primary Service Declaration (extended with BMI characteristics) */
BT_GATT_SERVICE_DEFINE(vnd_svc,
					   BT_GATT_PRIMARY_SERVICE(&vnd_uuid),

					   /* Original characteristic */
					   BT_GATT_CHARACTERISTIC(&vnd_enc_uuid.uuid,
											  BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_INDICATE,
											  BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT,
											  read_vnd, write_vnd, vnd_value),
					   BT_GATT_CCC(vnd_ccc_cfg_changed,
								   BT_GATT_PERM_READ | BT_GATT_PERM_WRITE_ENCRYPT),

					   BT_GATT_CHARACTERISTIC(&vnd_auth_uuid.uuid,
											  BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
											  BT_GATT_PERM_READ_AUTHEN | BT_GATT_PERM_WRITE_AUTHEN,
											  read_vnd, write_vnd, vnd_auth_value),

					   /* Legacy ACC/GYR READ values retained for GATT compatibility. */
					   BT_GATT_CHARACTERISTIC(&bmi_acc_uuid.uuid,
										  BT_GATT_CHRC_READ,
										  BT_GATT_PERM_READ,
										  read_bmi_acc, NULL, bmi_acc_last),

					   BT_GATT_CHARACTERISTIC(&bmi_gyr_uuid.uuid,
										  BT_GATT_CHRC_READ,
										  BT_GATT_PERM_READ,
										  read_bmi_gyr, NULL, bmi_gyr_last),

					   /* Versioned combined ACC/GYR notification. */
					   BT_GATT_CHARACTERISTIC(&bmi_stream_uuid.uuid,
										  BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
										  BT_GATT_PERM_READ,
										  read_bmi_stream, NULL, bmi_stream_last),
					   BT_GATT_CCC(bmi_stream_ccc_cfg_changed,
								   BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

					   /* Long/reliable write */
					   BT_GATT_CHARACTERISTIC(&vnd_long_uuid.uuid,
											  BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_EXT_PROP,
											  BT_GATT_PERM_READ | BT_GATT_PERM_WRITE | BT_GATT_PERM_PREPARE_WRITE,
											  read_vnd, write_long_vnd, &vnd_long_value),
					   BT_GATT_CEP(&vnd_long_cep),

					   BT_GATT_CHARACTERISTIC(&vnd_signed_uuid.uuid,
											  BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_AUTH,
											  BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
											  read_signed, write_signed, &signed_value),

					   BT_GATT_CHARACTERISTIC(&vnd_write_cmd_uuid.uuid,
											  BT_GATT_CHRC_WRITE_WITHOUT_RESP,
											  BT_GATT_PERM_WRITE,
											  NULL, write_without_rsp_vnd, &vnd_wwr_value), );

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
				  BT_UUID_16_ENCODE(BT_UUID_HRS_VAL),
				  BT_UUID_16_ENCODE(BT_UUID_BAS_VAL),
				  BT_UUID_16_ENCODE(BT_UUID_CTS_VAL)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_CUSTOM_SERVICE_VAL),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
			sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static void mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
	ARG_UNUSED(conn);
	printk("Updated MTU: TX: %d RX: %d bytes\n", tx, rx);
}

static struct bt_gatt_cb gatt_callbacks = {
	.att_mtu_updated = mtu_updated};

static const char *advertising_restart_reason = "unknown";

static const struct bt_le_conn_param preferred_conn_params = {
	.interval_min = 6,  /* 7.5 ms */
	.interval_max = 12, /* 15 ms */
	.latency = 0,
	.timeout = 400, /* 4 s */
};

static void advertising_restart_work_handler(struct k_work *work);

static K_WORK_DELAYABLE_DEFINE(advertising_restart_work,
						 advertising_restart_work_handler);

static int start_advertising(const char *reason)
{
	int err;

	printk("Advertising start requested (%s)\n", reason);

	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad),
							  sd, ARRAY_SIZE(sd));
	if (err && err != -EALREADY)
	{
		printk("Advertising failed (%s, err=%d)\n", reason, err);
		return err;
	}

	printk("Advertising started (%s)\n", reason);
	return 0;
}

static void schedule_advertising_restart(const char *reason)
{
	advertising_restart_reason = reason;
	k_work_reschedule(&advertising_restart_work,
					  K_MSEC(ADV_RESTART_DELAY_MS));
	printk("Advertising restart scheduled (%s, %d ms)\n", reason,
		   ADV_RESTART_DELAY_MS);
}

static void advertising_restart_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	start_advertising(advertising_restart_reason);
}

static void print_connection_diagnostics(struct bt_conn *conn, const char *reason)
{
	struct bt_conn_info info;
	int err = bt_conn_get_info(conn, &info);

	if (err)
	{
		printk("BLE_DIAG info err=%d reason=%s\n", err, reason);
		return;
	}

	printk("BLE_DIAG link reason=%s interval_us=%u latency=%u timeout_10ms=%u\n",
		   reason, (unsigned int)info.le.interval_us,
		   (unsigned int)info.le.latency, (unsigned int)info.le.timeout);

#if defined(CONFIG_BT_USER_PHY_UPDATE)
	if (info.le.phy)
	{
		printk("BLE_DIAG phy reason=%s tx=%u rx=%u\n", reason,
			   (unsigned int)info.le.phy->tx_phy,
			   (unsigned int)info.le.phy->rx_phy);
	}
#endif

#if defined(CONFIG_BT_USER_DATA_LEN_UPDATE)
	if (info.le.data_len)
	{
		printk("BLE_DIAG data_len reason=%s tx=%u/%uus rx=%u/%uus\n", reason,
			   (unsigned int)info.le.data_len->tx_max_len,
			   (unsigned int)info.le.data_len->tx_max_time,
			   (unsigned int)info.le.data_len->rx_max_len,
			   (unsigned int)info.le.data_len->rx_max_time);
	}
#endif
}

static void le_param_updated(struct bt_conn *conn, uint16_t interval,
				 uint16_t latency, uint16_t timeout)
{
	ARG_UNUSED(conn);
	printk("BLE_DIAG param_updated interval_us=%u latency=%u timeout_10ms=%u\n",
		   (unsigned int)BT_CONN_INTERVAL_TO_US(interval),
		   (unsigned int)latency, (unsigned int)timeout);
}

#if defined(CONFIG_BT_USER_PHY_UPDATE)
static void le_phy_updated(struct bt_conn *conn, struct bt_conn_le_phy_info *param)
{
	ARG_UNUSED(conn);
	printk("BLE_DIAG phy_updated tx=%u rx=%u\n",
		   (unsigned int)param->tx_phy, (unsigned int)param->rx_phy);
}
#endif

#if defined(CONFIG_BT_USER_DATA_LEN_UPDATE)
static void le_data_len_updated(struct bt_conn *conn,
					struct bt_conn_le_data_len_info *info)
{
	ARG_UNUSED(conn);
	printk("BLE_DIAG data_len_updated tx=%u/%uus rx=%u/%uus\n",
		   (unsigned int)info->tx_max_len, (unsigned int)info->tx_max_time,
		   (unsigned int)info->rx_max_len, (unsigned int)info->rx_max_time);
}
#endif

static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN] = "(unknown)";

	if (conn)
	{
		bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	}

	if (err)
	{
		printk("Connection failed (%s), err 0x%02x %s\n", addr, err,
			   bt_hci_err_to_str(err));
		schedule_advertising_restart("connect-failed");
		return;
	}

	printk("Connected: %s\n", addr);
	k_work_cancel_delayable(&advertising_restart_work);

	k_mutex_lock(&current_conn_mutex, K_FOREVER);
	if (current_conn)
	{
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
	current_conn = bt_conn_ref(conn);
	k_mutex_unlock(&current_conn_mutex);
	atomic_set(&bmi_stream_connected, 1);
	print_connection_diagnostics(conn, "connected");

	int update_err = bt_conn_le_param_update(conn, &preferred_conn_params);
	printk("BLE_DIAG param_request interval_us=7500-15000 latency=0 "
	       "timeout_10ms=400 ret=%d\n", update_err);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN] = "(unknown)";

	if (conn)
	{
		bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	}

	printk("Disconnected: %s, reason 0x%02x %s\n", addr, reason,
			   bt_hci_err_to_str(reason));

	atomic_clear(&bmi_stream_connected);
	atomic_clear(&bmi_stream_notify_enabled);
	k_msgq_purge(&bmi_ble_queue);

	k_mutex_lock(&current_conn_mutex, K_FOREVER);
	if (current_conn)
	{
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
	k_mutex_unlock(&current_conn_mutex);
	atomic_set(&bmi_stream_notify_last_ret, INT32_MIN);
	printk("BMI stream notify DISABLED (disconnect)\n");

	schedule_advertising_restart("disconnect");
}

static void alert_stop(void) { printk("Alert stopped\n"); }
static void alert_start(void) { printk("Mild alert started\n"); }
static void alert_high_start(void) { printk("High alert started\n"); }

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.le_param_updated = le_param_updated,
#if defined(CONFIG_BT_USER_PHY_UPDATE)
	.le_phy_updated = le_phy_updated,
#endif
#if defined(CONFIG_BT_USER_DATA_LEN_UPDATE)
	.le_data_len_updated = le_data_len_updated,
#endif
};

BT_IAS_CB_DEFINE(ias_callbacks) = {
	.no_alert = alert_stop,
	.mild_alert = alert_start,
	.high_alert = alert_high_start,
};

static void bt_ready(void)
{
	printk("Bluetooth initialized\n");

	if (IS_ENABLED(CONFIG_SETTINGS))
	{
		settings_load();
	}

	start_advertising("startup");
}

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Passkey for %s: %06u\n", addr, passkey);
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Pairing cancelled: %s\n", addr);
}

static struct bt_conn_auth_cb auth_cb_display = {
	.passkey_display = auth_passkey_display,
	.passkey_entry = NULL,
	.cancel = auth_cancel,
};

static void bas_notify(void)
{
	uint8_t battery_level = bt_bas_get_battery_level();

	battery_level--;
	if (!battery_level)
	{
		battery_level = 100U;
	}
	bt_bas_set_battery_level(battery_level);
}

static uint8_t bt_heartrate = BT_HR_HEARTRATE_DEFAULT_MIN;

static void hrs_notify(void)
{
	bt_heartrate++;
	if (bt_heartrate == BT_HR_HEARTRATE_DEFAULT_MAX)
	{
		bt_heartrate = BT_HR_HEARTRATE_DEFAULT_MIN;
	}
	bt_hrs_notify(bt_heartrate);
}

/* CTS helper */
static int64_t unix_ms_ref;
static bool cts_notification_enabled;

void bt_cts_notification_changed(bool enabled)
{
	cts_notification_enabled = enabled;
}

int bt_cts_cts_time_write(struct bt_cts_time_format *cts_time)
{
	int err;
	int64_t unix_ms;

	if (IS_ENABLED(CONFIG_BT_CTS_HELPER_API))
	{
		err = bt_cts_time_to_unix_ms(cts_time, &unix_ms);
		if (err)
		{
			return err;
		}
	}
	else
	{
		return -ENOTSUP;
	}

	unix_ms_ref = unix_ms - k_uptime_get();
	return 0;
}

int bt_cts_fill_current_cts_time(struct bt_cts_time_format *cts_time)
{
	int64_t unix_ms = unix_ms_ref + k_uptime_get();

	if (IS_ENABLED(CONFIG_BT_CTS_HELPER_API))
	{
		return bt_cts_time_from_unix_ms(cts_time, unix_ms);
	}
	else
	{
		return -ENOTSUP;
	}
}

const struct bt_cts_cb cts_cb = {
	.notification_changed = bt_cts_notification_changed,
	.cts_time_write = bt_cts_cts_time_write,
	.fill_current_cts_time = bt_cts_fill_current_cts_time,
};

static int bt_hrs_ctrl_point_write(uint8_t request)
{
	printk("HRS Control point request: %d\n", request);
	if (request != BT_HRS_CONTROL_POINT_RESET_ENERGY_EXPANDED_REQ)
	{
		return -ENOTSUP;
	}

	bt_heartrate = BT_HR_HEARTRATE_DEFAULT_MIN;
	return 0;
}

static struct bt_hrs_cb hrs_cb = {
	.ctrl_point_write = bt_hrs_ctrl_point_write,
};

/* ============================================================
 * BMI270 thread
 * ==========================================================*/

#define BMI_THREAD_STACK_SIZE 1024
#define BMI_THREAD_PRIORITY 5
#define BMI_BLE_THREAD_STACK_SIZE 1024
#define BMI_BLE_THREAD_PRIORITY 6
#define BMI_DIAG_THREAD_STACK_SIZE 1024
#define BMI_DIAG_THREAD_PRIORITY 7
#define BMI_SAMPLE_PERIOD_MS 10
#define BMI_DIAG_INTERVAL_MS 5000
#define BMI_ZERO_ACCEL_LIMIT 10U
#define BMI_RECOVERY_MAX_ATTEMPTS 2U
#define BMI_RECOVERY_SETTLE_MS 50U
#define BMI_ACCEL_RANGE_G 4
#define BMI_GYRO_RANGE_DPS 1000
#define BMI270_REG_CHIP_ID 0x00
#define BMI270_REG_ERROR 0x02
#define BMI270_REG_STATUS 0x03
#define BMI270_REG_INTERNAL_STATUS 0x21
#define BMI270_REG_ACC_CONF 0x40
#define BMI270_REG_ACC_RANGE 0x41
#define BMI270_REG_GYR_CONF 0x42
#define BMI270_REG_GYR_RANGE 0x43
#define BMI270_REG_PWR_CTRL 0x7D
#define BMI270_ODR_MASK 0x0F
#define BMI270_ACC_RANGE_MASK 0x03
#define BMI270_GYR_RANGE_MASK 0x07
#define BMI270_EXPECTED_ACC_RANGE_4G 0x01
#define BMI270_EXPECTED_GYR_RANGE_1000DPS 0x01
#define BMI270_INTERNAL_STATUS_MASK 0x0F
#define BMI270_INTERNAL_STATUS_INIT_OK 0x01
#define BMI270_PWR_CTRL_ACC_GYR_MASK 0x06
#define BMI270_EXPECTED_CHIP_ID 0x24

#if DT_HAS_COMPAT_STATUS_OKAY(bosch_bmi270)
#define BMI270_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(bosch_bmi270)
static const struct device *const bmi = DEVICE_DT_GET_ONE(bosch_bmi270);
static const struct i2c_dt_spec bmi_i2c = I2C_DT_SPEC_GET(BMI270_NODE);
#else
#warning "No BMI270 instance found in devicetree (compatible = \"bosch,bmi270\")"
#define bmi NULL
#endif

struct bmi_timing_metrics
{
	uint64_t period_total_us;
	uint64_t fetch_total_us;
	uint64_t process_total_us;
	uint64_t active_total_us;
	uint64_t notify_total_us;
	uint32_t period_max_us;
	uint32_t fetch_max_us;
	uint32_t process_max_us;
	uint32_t active_max_us;
	uint32_t notify_max_us;
	uint32_t deadline_late_max_us;
	uint32_t periods;
	uint32_t fetch_calls;
	uint32_t samples;
	uint32_t fetch_errors;
	uint32_t invalid_samples;
	uint32_t recovery_attempts;
	uint32_t recovery_errors;
	uint32_t deadline_overruns;
	uint32_t skipped_periods;
	uint32_t queue_depth_max;
	uint32_t queue_dropped;
	uint32_t notify_ok;
	uint32_t notify_errors;
};

static struct bmi_timing_metrics bmi_metrics;
static struct k_spinlock bmi_metrics_lock;

static uint32_t cycles_since_us(uint64_t start_cycles)
{
	return (uint32_t)k_cyc_to_us_floor64(k_cycle_get_64() - start_cycles);
}

static void timing_add(uint64_t *total_us, uint32_t *max_us, uint32_t elapsed_us)
{
	*total_us += elapsed_us;
	if (elapsed_us > *max_us)
	{
		*max_us = elapsed_us;
	}
}

static uint32_t timing_average(uint64_t total_us, uint32_t count)
{
	return count ? (uint32_t)(total_us / count) : 0U;
}

static void bmi_metrics_record_acquisition(uint32_t period_us, bool period_valid,
					   uint32_t fetch_us, uint32_t process_us,
					   uint32_t active_us, bool fetch_success,
					   bool data_valid)
{
	k_spinlock_key_t key = k_spin_lock(&bmi_metrics_lock);

	if (period_valid)
	{
		timing_add(&bmi_metrics.period_total_us, &bmi_metrics.period_max_us,
			   period_us);
		bmi_metrics.periods++;
	}
	timing_add(&bmi_metrics.fetch_total_us, &bmi_metrics.fetch_max_us, fetch_us);
	bmi_metrics.fetch_calls++;
	if (fetch_success && data_valid)
	{
		timing_add(&bmi_metrics.process_total_us, &bmi_metrics.process_max_us,
			   process_us);
		timing_add(&bmi_metrics.active_total_us, &bmi_metrics.active_max_us,
			   active_us);
		bmi_metrics.samples++;
	}
	else if (!fetch_success)
	{
		bmi_metrics.fetch_errors++;
	}
	else
	{
		bmi_metrics.invalid_samples++;
	}

	k_spin_unlock(&bmi_metrics_lock, key);
}

static void bmi_metrics_record_recovery(int ret)
{
	k_spinlock_key_t key = k_spin_lock(&bmi_metrics_lock);

	bmi_metrics.recovery_attempts++;
	if (ret != 0)
	{
		bmi_metrics.recovery_errors++;
	}

	k_spin_unlock(&bmi_metrics_lock, key);
}

static void bmi_metrics_record_overrun(uint32_t late_us, uint32_t skipped)
{
	k_spinlock_key_t key = k_spin_lock(&bmi_metrics_lock);

	bmi_metrics.deadline_overruns++;
	bmi_metrics.skipped_periods += skipped;
	if (late_us > bmi_metrics.deadline_late_max_us)
	{
		bmi_metrics.deadline_late_max_us = late_us;
	}

	k_spin_unlock(&bmi_metrics_lock, key);
}

static void bmi_metrics_record_queue_depth(uint32_t depth)
{
	k_spinlock_key_t key = k_spin_lock(&bmi_metrics_lock);

	if (depth > bmi_metrics.queue_depth_max)
	{
		bmi_metrics.queue_depth_max = depth;
	}

	k_spin_unlock(&bmi_metrics_lock, key);
}

static void bmi_metrics_record_queue_drop(void)
{
	k_spinlock_key_t key = k_spin_lock(&bmi_metrics_lock);

	bmi_metrics.queue_dropped++;

	k_spin_unlock(&bmi_metrics_lock, key);
}

static void bmi_metrics_record_notify(uint32_t elapsed_us, int ret)
{
	k_spinlock_key_t key = k_spin_lock(&bmi_metrics_lock);

	timing_add(&bmi_metrics.notify_total_us, &bmi_metrics.notify_max_us,
		   elapsed_us);
	if (ret == 0)
	{
		bmi_metrics.notify_ok++;
	}
	else
	{
		bmi_metrics.notify_errors++;
	}

	k_spin_unlock(&bmi_metrics_lock, key);
}

static struct bmi_timing_metrics bmi_metrics_take_snapshot(void)
{
	struct bmi_timing_metrics snapshot;
	k_spinlock_key_t key = k_spin_lock(&bmi_metrics_lock);

	snapshot = bmi_metrics;
	memset(&bmi_metrics, 0, sizeof(bmi_metrics));

	k_spin_unlock(&bmi_metrics_lock, key);
	return snapshot;
}

#if DT_HAS_COMPAT_STATUS_OKAY(bosch_bmi270)
struct bmi_register_state
{
	uint8_t chip_id;
	uint8_t error;
	uint8_t status;
	uint8_t internal_status;
	uint8_t acc_conf;
	uint8_t acc_range;
	uint8_t gyr_conf;
	uint8_t gyr_range;
	uint8_t pwr_ctrl;
};

static int bmi_read_register_state(struct bmi_register_state *state)
{
	int ret = i2c_reg_read_byte_dt(&bmi_i2c, BMI270_REG_CHIP_ID,
					   &state->chip_id);

	if (ret == 0)
	{
		ret = i2c_reg_read_byte_dt(&bmi_i2c, BMI270_REG_ERROR,
						   &state->error);
	}
	if (ret == 0)
	{
		ret = i2c_reg_read_byte_dt(&bmi_i2c, BMI270_REG_STATUS,
						   &state->status);
	}
	if (ret == 0)
	{
		ret = i2c_reg_read_byte_dt(&bmi_i2c, BMI270_REG_INTERNAL_STATUS,
						   &state->internal_status);
	}
	if (ret == 0)
	{
		ret = i2c_reg_read_byte_dt(&bmi_i2c, BMI270_REG_ACC_CONF,
						   &state->acc_conf);
	}
	if (ret == 0)
	{
		ret = i2c_reg_read_byte_dt(&bmi_i2c, BMI270_REG_ACC_RANGE,
						   &state->acc_range);
	}
	if (ret == 0)
	{
		ret = i2c_reg_read_byte_dt(&bmi_i2c, BMI270_REG_GYR_CONF,
						   &state->gyr_conf);
	}
	if (ret == 0)
	{
		ret = i2c_reg_read_byte_dt(&bmi_i2c, BMI270_REG_GYR_RANGE,
						   &state->gyr_range);
	}
	if (ret == 0)
	{
		ret = i2c_reg_read_byte_dt(&bmi_i2c, BMI270_REG_PWR_CTRL,
						   &state->pwr_ctrl);
	}

	return ret;
}

static int bmi_diag_print_registers(const char *reason,
					struct bmi_register_state *state_out)
{
	struct bmi_register_state state;
	int ret = bmi_read_register_state(&state);

	if (ret != 0)
	{
		printk("BMI_DIAG register_read reason=%s err=%d\n", reason, ret);
		return ret;
	}

	printk("BMI_DIAG registers reason=%s chip_id=0x%02x err=0x%02x "
		   "status=0x%02x internal=0x%02x pwr_ctrl=0x%02x "
		   "acc_range_req_g=%u acc_range=0x%02x acc_range_expected=0x%02x "
		   "acc_conf=0x%02x acc_odr=0x%x "
		   "gyr_range_req_dps=%u gyr_range=0x%02x gyr_range_expected=0x%02x "
		   "gyr_conf=0x%02x gyr_odr=0x%x\n",
		   reason, state.chip_id, state.error, state.status,
		   state.internal_status, state.pwr_ctrl,
		   BMI_ACCEL_RANGE_G,
		   state.acc_range & BMI270_ACC_RANGE_MASK,
		   BMI270_EXPECTED_ACC_RANGE_4G,
		   state.acc_conf, state.acc_conf & BMI270_ODR_MASK,
		   BMI_GYRO_RANGE_DPS,
		   state.gyr_range & BMI270_GYR_RANGE_MASK,
		   BMI270_EXPECTED_GYR_RANGE_1000DPS,
		   state.gyr_conf, state.gyr_conf & BMI270_ODR_MASK);

	if (state_out != NULL)
	{
		*state_out = state;
	}

	return 0;
}

static int bmi_set_attr_checked(enum sensor_channel channel,
					enum sensor_attribute attribute,
					const struct sensor_value *value,
					const char *label)
{
	int ret = sensor_attr_set(bmi, channel, attribute, value);

	printk("BMI config %s ret=%d\n", label, ret);
	return ret;
}

static int bmi_configure_sensor(const char *reason)
{
	struct sensor_value full_scale = {0};
	struct sensor_value sampling_freq = {0};
	struct sensor_value oversampling = {0};
	int ret;

	printk("BMI configuration start reason=%s\n", reason);

	/* ACC: +/-4G, 100 Hz, normal averaging. */
	full_scale.val1 = BMI_ACCEL_RANGE_G;
	oversampling.val1 = 1;
	sampling_freq.val1 = 100;

	ret = bmi_set_attr_checked(SENSOR_CHAN_ACCEL_XYZ,
					   SENSOR_ATTR_FULL_SCALE, &full_scale,
					   "accel FULL_SCALE");
	if (ret != 0)
	{
		return ret;
	}
	ret = bmi_set_attr_checked(SENSOR_CHAN_ACCEL_XYZ,
					   SENSOR_ATTR_OVERSAMPLING, &oversampling,
					   "accel OVERSAMPLING");
	if (ret != 0)
	{
		return ret;
	}
	ret = bmi_set_attr_checked(SENSOR_CHAN_ACCEL_XYZ,
					   SENSOR_ATTR_SAMPLING_FREQUENCY, &sampling_freq,
					   "accel SAMPLING_FREQ");
	if (ret != 0)
	{
		return ret;
	}

	/* GYR: +/-1000 dps, 100 Hz, normal averaging. */
	full_scale.val1 = BMI_GYRO_RANGE_DPS;
	ret = bmi_set_attr_checked(SENSOR_CHAN_GYRO_XYZ,
					   SENSOR_ATTR_FULL_SCALE, &full_scale,
					   "gyro FULL_SCALE");
	if (ret != 0)
	{
		return ret;
	}
	ret = bmi_set_attr_checked(SENSOR_CHAN_GYRO_XYZ,
					   SENSOR_ATTR_OVERSAMPLING, &oversampling,
					   "gyro OVERSAMPLING");
	if (ret != 0)
	{
		return ret;
	}
	ret = bmi_set_attr_checked(SENSOR_CHAN_GYRO_XYZ,
					   SENSOR_ATTR_SAMPLING_FREQUENCY, &sampling_freq,
					   "gyro SAMPLING_FREQ");
	if (ret != 0)
	{
		return ret;
	}

	return bmi_diag_print_registers(reason, NULL);
}

static int bmi_recover_zero_data(void)
{
	struct bmi_register_state state;
	int ret = bmi_diag_print_registers("zero_data", &state);

	if (ret != 0)
	{
		return ret;
	}
	if (state.chip_id != BMI270_EXPECTED_CHIP_ID ||
	    (state.internal_status & BMI270_INTERNAL_STATUS_MASK) !=
		    BMI270_INTERNAL_STATUS_INIT_OK)
	{
		printk("BMI recovery requires driver reinitialization\n");
		return -EIO;
	}
	if ((state.pwr_ctrl & BMI270_PWR_CTRL_ACC_GYR_MASK) !=
	    BMI270_PWR_CTRL_ACC_GYR_MASK)
	{
		printk("BMI recovery detected disabled sensor power bits\n");
	}

	ret = bmi_configure_sensor("zero_data_reconfigure");
	if (ret == 0)
	{
		k_sleep(K_MSEC(BMI_RECOVERY_SETTLE_MS));
	}

	return ret;
}
#endif

static void pack_and_store_acc(uint32_t seq, int16_t ax_mg, int16_t ay_mg, int16_t az_mg)
{
	sys_put_le32(seq, &bmi_acc_last[0]);
	sys_put_le16((uint16_t)ax_mg, &bmi_acc_last[4]);
	sys_put_le16((uint16_t)ay_mg, &bmi_acc_last[6]);
	sys_put_le16((uint16_t)az_mg, &bmi_acc_last[8]);
}

static void pack_and_store_gyr(uint32_t seq, int32_t gx_mdps, int32_t gy_mdps, int32_t gz_mdps)
{
	sys_put_le32(seq, &bmi_gyr_last[0]);
	sys_put_le32((uint32_t)gx_mdps, &bmi_gyr_last[4]);
	sys_put_le32((uint32_t)gy_mdps, &bmi_gyr_last[8]);
	sys_put_le32((uint32_t)gz_mdps, &bmi_gyr_last[12]);
}

/*
 * Wire format is exactly 23 bytes with no C struct padding:
 *   [0] version (1)
 *   [1..4] seq (u32 LE)
 *   [5..10] ax/ay/az (3 x i16 LE, mg)
 *   [11..22] gx/gy/gz (3 x i32 LE, mdps)
 */
static void pack_bmi_stream(const struct bmi_sample *sample, uint8_t *payload)
{
	payload[0] = BMI_STREAM_VERSION;
	sys_put_le32(sample->seq, &payload[1]);
	sys_put_le16((uint16_t)sample->ax_mg, &payload[5]);
	sys_put_le16((uint16_t)sample->ay_mg, &payload[7]);
	sys_put_le16((uint16_t)sample->az_mg, &payload[9]);
	sys_put_le32((uint32_t)sample->gx_mdps, &payload[11]);
	sys_put_le32((uint32_t)sample->gy_mdps, &payload[15]);
	sys_put_le32((uint32_t)sample->gz_mdps, &payload[19]);
}

static void bmi_ble_enqueue(const struct bmi_sample *sample)
{
	if (!atomic_get(&bmi_stream_connected) ||
	    !atomic_get(&bmi_stream_notify_enabled))
	{
		return;
	}

	int ret = k_msgq_put(&bmi_ble_queue, sample, K_NO_WAIT);
	if (ret != 0)
	{
		struct bmi_sample oldest;

		/* Preserve the newest sensor data: discard one oldest queued sample. */
		if (k_msgq_get(&bmi_ble_queue, &oldest, K_NO_WAIT) == 0)
		{
			bmi_metrics_record_queue_drop();
		}

		ret = k_msgq_put(&bmi_ble_queue, sample, K_NO_WAIT);
		if (ret != 0)
		{
			/* A concurrent purge/disconnect can still make this sample unsendable. */
			bmi_metrics_record_queue_drop();
			return;
		}
	}

	bmi_metrics_record_queue_depth(k_msgq_num_used_get(&bmi_ble_queue));
}

static void bmi_ble_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1)
	{
		struct bmi_sample sample;
		uint8_t payload[BMI_STREAM_PAYLOAD_SIZE];

		if (k_msgq_get(&bmi_ble_queue, &sample, K_FOREVER) != 0)
		{
			continue;
		}
		if (!atomic_get(&bmi_stream_connected) ||
		    !atomic_get(&bmi_stream_notify_enabled) || !bmi_stream_attr)
		{
			continue;
		}

		k_mutex_lock(&current_conn_mutex, K_FOREVER);
		struct bt_conn *conn = current_conn ? bt_conn_ref(current_conn) : NULL;
		k_mutex_unlock(&current_conn_mutex);
		if (!conn)
		{
			continue;
		}

		pack_bmi_stream(&sample, payload);
		uint64_t notify_start_cycles = k_cycle_get_64();
		int notify_ret = bt_gatt_notify(conn, bmi_stream_attr,
						 payload, sizeof(payload));
		uint32_t notify_us = cycles_since_us(notify_start_cycles);
		bt_conn_unref(conn);

		bmi_metrics_record_notify(notify_us, notify_ret);
		if (notify_ret != atomic_get(&bmi_stream_notify_last_ret))
		{
			if (notify_ret == 0)
			{
				printk("BMI stream notify active (%u bytes)\n",
				       (unsigned int)sizeof(payload));
			}
			else
			{
				printk("BMI stream notify err=%d\n", notify_ret);
			}
			atomic_set(&bmi_stream_notify_last_ret, notify_ret);
		}
	}
}

static void bmi_diag_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	int64_t window_start_ms = k_uptime_get();

	while (1)
	{
		k_sleep(K_MSEC(BMI_DIAG_INTERVAL_MS));

		int64_t now_ms = k_uptime_get();
		int64_t elapsed_ms = now_ms - window_start_ms;
		window_start_ms = now_ms;
		struct bmi_timing_metrics metrics = bmi_metrics_take_snapshot();
		uint32_t queue_depth = k_msgq_num_used_get(&bmi_ble_queue);
		uint32_t rate_millihz = elapsed_ms > 0
			? (uint32_t)(((uint64_t)metrics.samples * 1000000U) /
				     (uint64_t)elapsed_ms)
			: 0U;
		uint32_t notify_calls = metrics.notify_ok + metrics.notify_errors;

		printk("BMI_DIAG window_ms=%lld samples=%u rate_millihz=%u fetch_err=%u "
		       "invalid=%u recovery=%u recovery_err=%u\n",
		       elapsed_ms, metrics.samples, rate_millihz, metrics.fetch_errors,
		       metrics.invalid_samples, metrics.recovery_attempts,
		       metrics.recovery_errors);
		printk("BMI_DIAG timing_us period_avg=%u period_max=%u fetch_avg=%u "
		       "fetch_max=%u process_avg=%u process_max=%u active_avg=%u "
		       "active_max=%u\n",
		       timing_average(metrics.period_total_us, metrics.periods),
		       metrics.period_max_us,
		       timing_average(metrics.fetch_total_us, metrics.fetch_calls),
		       metrics.fetch_max_us,
		       timing_average(metrics.process_total_us, metrics.samples),
		       metrics.process_max_us,
		       timing_average(metrics.active_total_us, metrics.samples),
		       metrics.active_max_us);
		printk("BMI_DIAG schedule overrun=%u skipped=%u late_max_us=%u\n",
		       metrics.deadline_overruns, metrics.skipped_periods,
		       metrics.deadline_late_max_us);
		printk("BMI_DIAG ble queue_depth=%u queue_max=%u dropped=%u "
		       "notify_ok=%u notify_err=%u notify_avg_us=%u notify_max_us=%u\n",
		       queue_depth, metrics.queue_depth_max, metrics.queue_dropped,
		       metrics.notify_ok, metrics.notify_errors,
		       timing_average(metrics.notify_total_us, notify_calls),
		       metrics.notify_max_us);
	}
}

static void bmi_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

#if !DT_HAS_COMPAT_STATUS_OKAY(bosch_bmi270)
	printk("BMI270 not present in devicetree, BMI thread exiting.\n");
	return;
#else
	struct sensor_value acc[3];
	struct sensor_value gyr[3];

	uint32_t seq = 0;
	uint32_t zero_accel_streak = 0;
	uint32_t recovery_attempts = 0;
	int ret;

	printk("BMI270 dev ptr = %p, name = %s\n", bmi, bmi->name);

	if (!device_is_ready(bmi))
	{
		printk("BMI270 device not ready: %s\n", bmi->name);
		return;
	}

	printk("BMI270 is ready\n");
	ret = bmi_configure_sensor("startup");
	if (ret != 0)
	{
		printk("BMI startup configuration failed ret=%d; rebooting\n", ret);
		k_sleep(K_MSEC(BMI_RECOVERY_SETTLE_MS));
		sys_reboot(SYS_REBOOT_COLD);
		return;
	}

	const int64_t period_ticks = k_ms_to_ticks_ceil64(BMI_SAMPLE_PERIOD_MS);
	int64_t next_release_ticks = k_uptime_ticks();
	uint64_t previous_sample_start_cycles = 0U;

	while (1)
	{
		int64_t now_ticks = k_uptime_ticks();
		if (now_ticks < next_release_ticks)
		{
			k_sleep(K_TIMEOUT_ABS_TICKS(next_release_ticks));
		}

		uint64_t sample_start_cycles = k_cycle_get_64();
		uint32_t period_us = previous_sample_start_cycles
			? (uint32_t)k_cyc_to_us_floor64(sample_start_cycles -
							       previous_sample_start_cycles)
			: 0U;
		bool period_valid = previous_sample_start_cycles != 0U;
		previous_sample_start_cycles = sample_start_cycles;
		uint64_t stage_start_cycles = k_cycle_get_64();
		int err = sensor_sample_fetch(bmi);
		uint32_t fetch_us = cycles_since_us(stage_start_cycles);
		if (err)
		{
			printk("sensor_sample_fetch err=%d\n", err);
			bmi_metrics_record_acquisition(period_us, period_valid, fetch_us,
						       0U, cycles_since_us(sample_start_cycles),
						       false, false);
			goto schedule_next_sample;
		}

		stage_start_cycles = k_cycle_get_64();
		int acc_err = sensor_channel_get(bmi, SENSOR_CHAN_ACCEL_XYZ, acc);
		int gyr_err = sensor_channel_get(bmi, SENSOR_CHAN_GYRO_XYZ, gyr);
		if (acc_err != 0 || gyr_err != 0)
		{
			printk("sensor_channel_get err acc=%d gyr=%d\n", acc_err, gyr_err);
			bmi_metrics_record_acquisition(period_us, period_valid, fetch_us,
						       cycles_since_us(stage_start_cycles),
						       cycles_since_us(sample_start_cycles),
						       true, false);
			goto schedule_next_sample;
		}

		int32_t ax_mg32 = sensor_ms2_to_mg(&acc[0]);
		int32_t ay_mg32 = sensor_ms2_to_mg(&acc[1]);
		int32_t az_mg32 = sensor_ms2_to_mg(&acc[2]);

		int32_t gx_mdps = sensor_rad_to_10udegrees(&gyr[0]) / 100;
		int32_t gy_mdps = sensor_rad_to_10udegrees(&gyr[1]) / 100;
		int32_t gz_mdps = sensor_rad_to_10udegrees(&gyr[2]) / 100;

		if (ax_mg32 == 0 && ay_mg32 == 0 && az_mg32 == 0)
		{
			zero_accel_streak++;
			bmi_metrics_record_acquisition(period_us, period_valid, fetch_us,
						       cycles_since_us(stage_start_cycles),
						       cycles_since_us(sample_start_cycles),
						       true, false);

			if (zero_accel_streak >= BMI_ZERO_ACCEL_LIMIT)
			{
				recovery_attempts++;
				printk("BMI zero acceleration detected count=%u recovery=%u\n",
				       zero_accel_streak, recovery_attempts);
				ret = bmi_recover_zero_data();
				bmi_metrics_record_recovery(ret);
				zero_accel_streak = 0;

				if (ret != 0 ||
				    recovery_attempts >= BMI_RECOVERY_MAX_ATTEMPTS)
				{
					printk("BMI recovery failed ret=%d attempts=%u; rebooting\n",
					       ret, recovery_attempts);
					k_sleep(K_MSEC(BMI_RECOVERY_SETTLE_MS));
					sys_reboot(SYS_REBOOT_COLD);
					return;
				}
			}

			goto schedule_next_sample;
		}

		zero_accel_streak = 0;
		recovery_attempts = 0;

		/* clamp accel to int16 range */
		if (ax_mg32 > INT16_MAX)
			ax_mg32 = INT16_MAX;
		if (ax_mg32 < INT16_MIN)
			ax_mg32 = INT16_MIN;
		if (ay_mg32 > INT16_MAX)
			ay_mg32 = INT16_MAX;
		if (ay_mg32 < INT16_MIN)
			ay_mg32 = INT16_MIN;
		if (az_mg32 > INT16_MAX)
			az_mg32 = INT16_MAX;
		if (az_mg32 < INT16_MIN)
			az_mg32 = INT16_MIN;

		int16_t ax_mg = (int16_t)ax_mg32;
		int16_t ay_mg = (int16_t)ay_mg32;
		int16_t az_mg = (int16_t)az_mg32;

		struct bmi_sample sample = {
			.seq = seq,
			.ax_mg = ax_mg,
			.ay_mg = ay_mg,
			.az_mg = az_mg,
			.gx_mdps = gx_mdps,
			.gy_mdps = gy_mdps,
			.gz_mdps = gz_mdps,
		};

		/* Store legacy READ values and the combined READ/notify payload. */
		pack_and_store_acc(seq, ax_mg, ay_mg, az_mg);
		pack_and_store_gyr(seq, gx_mdps, gy_mdps, gz_mdps);
		pack_bmi_stream(&sample, bmi_stream_last);
		uint32_t process_us = cycles_since_us(stage_start_cycles);
		uint32_t active_us = cycles_since_us(sample_start_cycles);

		bmi_metrics_record_acquisition(period_us, period_valid, fetch_us,
						   process_us, active_us, true, true);
		bmi_ble_enqueue(&sample);
		seq++;

schedule_next_sample:
		next_release_ticks += period_ticks;
		now_ticks = k_uptime_ticks();
		if (now_ticks > next_release_ticks)
		{
			int64_t late_ticks = now_ticks - next_release_ticks;
			uint32_t skipped = (uint32_t)(late_ticks / period_ticks) + 1U;
			uint32_t late_us = (uint32_t)k_ticks_to_us_floor64(late_ticks);

			bmi_metrics_record_overrun(late_us, skipped);
			next_release_ticks += (int64_t)skipped * period_ticks;
		}
	}
#endif
}

K_THREAD_DEFINE(bmi_thread_id,
				BMI_THREAD_STACK_SIZE,
				bmi_thread,
				NULL, NULL, NULL,
				BMI_THREAD_PRIORITY, 0, 0);

K_THREAD_DEFINE(bmi_ble_thread_id,
				BMI_BLE_THREAD_STACK_SIZE,
				bmi_ble_thread,
				NULL, NULL, NULL,
				BMI_BLE_THREAD_PRIORITY, 0, 0);

K_THREAD_DEFINE(bmi_diag_thread_id,
				BMI_DIAG_THREAD_STACK_SIZE,
				bmi_diag_thread,
				NULL, NULL, NULL,
				BMI_DIAG_THREAD_PRIORITY, 0, 0);

/* ============================================================
 * main()
 * ==========================================================*/

int main(void)
{
	struct bt_gatt_attr *vnd_ind_attr;
	char str[BT_UUID_STR_LEN];
	int err;

	err = bt_enable(NULL);
	if (err)
	{
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	bt_cts_init(&cts_cb);
	bt_hrs_cb_register(&hrs_cb);

	bt_gatt_cb_register(&gatt_callbacks);
	bt_conn_auth_cb_register(&auth_cb_display);

	/* original indication demo */
	vnd_ind_attr = bt_gatt_find_by_uuid(vnd_svc.attrs, vnd_svc.attr_count,
										&vnd_enc_uuid.uuid);
	bt_uuid_to_str(&vnd_enc_uuid.uuid, str, sizeof(str));
	printk("Indicate VND attr %p (UUID %s)\n", vnd_ind_attr, str);

	/* find BMI characteristic value attributes */
	bmi_acc_attr = bt_gatt_find_by_uuid(vnd_svc.attrs, vnd_svc.attr_count, &bmi_acc_uuid.uuid);
	bmi_gyr_attr = bt_gatt_find_by_uuid(vnd_svc.attrs, vnd_svc.attr_count, &bmi_gyr_uuid.uuid);
	bmi_stream_attr = bt_gatt_find_by_uuid(vnd_svc.attrs, vnd_svc.attr_count,
					       &bmi_stream_uuid.uuid);

	bt_uuid_to_str(&bmi_acc_uuid.uuid, str, sizeof(str));
	printk("BMI ACC attr %p (UUID %s)\n", bmi_acc_attr, str);
	bt_uuid_to_str(&bmi_gyr_uuid.uuid, str, sizeof(str));
	printk("BMI GYR attr %p (UUID %s)\n", bmi_gyr_attr, str);
	bt_uuid_to_str(&bmi_stream_uuid.uuid, str, sizeof(str));
	printk("BMI stream attr %p (UUID %s, version=%u, payload=%u bytes)\n",
	       bmi_stream_attr, str, BMI_STREAM_VERSION, BMI_STREAM_PAYLOAD_SIZE);

	/* Start advertising only after all stream attributes are ready. */
	bt_ready();

	/* keep original demo periodic notifications (1Hz) */
	while (1)
	{
		k_sleep(K_SECONDS(1));

		if (cts_notification_enabled)
		{
			bt_cts_send_notification(BT_CTS_UPDATE_REASON_MANUAL);
		}
		hrs_notify();
		bas_notify();

		if (simulate_vnd && vnd_ind_attr)
		{
			if (indicating)
			{
				continue;
			}

			ind_params.attr = vnd_ind_attr;
			ind_params.func = indicate_cb;
			ind_params.destroy = indicate_destroy;
			ind_params.data = &indicating;
			ind_params.len = sizeof(indicating);

			if (bt_gatt_indicate(NULL, &ind_params) == 0)
			{
				indicating = 1U;
			}
		}
	}

	return 0;
}
