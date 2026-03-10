/* main.c - BLE sample + BMI270 streaming (Notify)
 *
 * This file is created by merging:
 * - BLE base (マージ用_BLE.txt)
 * - BMI270 polling sample (マージ用_BMI.txt)
 *
 * What this does:
 * - Starts BLE advertising (based on the BLE sample)
 * - Keeps the sample services (BAS/HRS/CTS/IAS + Vendor service)
 * - Adds two NOTIFY characteristics to the Vendor service:
 *     * Accel (mg):  seq(uint32) + ax/ay/az(int16)
 *     * Gyro (mdps): seq(uint32) + gx/gy/gz(int32)
 * - Polls BMI270 at ~100Hz, prints CSV to UART, and notifies when enabled.
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
#include <zephyr/kernel.h>

#include <zephyr/device.h>
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

/* sensor_value → milli-unit integer (mg or mdps) */
static inline int32_t sv_to_milli(const struct sensor_value *v)
{
	/* val2 is in 1e-6 units */
	return (int32_t)(v->val1 * 1000 + v->val2 / 1000);
}

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

/* ---- Added: BMI270 streaming characteristics (NOTIFY) ---- */
/* Accel notify char UUID */
static const struct bt_uuid_128 bmi_acc_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef11));

/* Gyro notify char UUID */
static const struct bt_uuid_128 bmi_gyr_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef12));

#define VND_MAX_LEN 20
#define BT_HR_HEARTRATE_DEFAULT_MIN 90U
#define BT_HR_HEARTRATE_DEFAULT_MAX 160U

static uint8_t vnd_value[VND_MAX_LEN + 1] = {'V', 'e', 'n', 'd', 'o', 'r'};
static uint8_t vnd_auth_value[VND_MAX_LEN + 1] = {'V', 'e', 'n', 'd', 'o', 'r'};
static uint8_t vnd_wwr_value[VND_MAX_LEN + 1] = {'V', 'e', 'n', 'd', 'o', 'r'};

/* Last BMI data (for READ) */
static uint8_t bmi_acc_last[10]; /* seq u32 + 3x i16 = 10 bytes */
static uint8_t bmi_gyr_last[16]; /* seq u32 + 3x i32 = 16 bytes */

/* notification enable flags */
static bool bmi_acc_notify_enabled;
static bool bmi_gyr_notify_enabled;

/* attribute pointers (set at runtime with bt_gatt_find_by_uuid) */
static const struct bt_gatt_attr *bmi_acc_attr;
static const struct bt_gatt_attr *bmi_gyr_attr;

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

/* CCC callbacks (enable notify flags) */
static void bmi_acc_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	bmi_acc_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
	printk("BMI ACC notify %s\n", bmi_acc_notify_enabled ? "ENABLED" : "DISABLED");
}

static void bmi_gyr_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	bmi_gyr_notify_enabled = (value == BT_GATT_CCC_NOTIFY);
	printk("BMI GYR notify %s\n", bmi_gyr_notify_enabled ? "ENABLED" : "DISABLED");
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

					   /* ACC notify */
					   BT_GATT_CHARACTERISTIC(&bmi_acc_uuid.uuid,
											  BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
											  BT_GATT_PERM_READ,
											  read_bmi_acc, NULL, bmi_acc_last),
					   BT_GATT_CCC(bmi_acc_ccc_cfg_changed,
								   BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

					   /* GYR notify */
					   BT_GATT_CHARACTERISTIC(&bmi_gyr_uuid.uuid,
											  BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
											  BT_GATT_PERM_READ,
											  read_bmi_gyr, NULL, bmi_gyr_last),
					   BT_GATT_CCC(bmi_gyr_ccc_cfg_changed,
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

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err)
	{
		printk("Connection failed, err 0x%02x %s\n", err,
			   bt_hci_err_to_str(err));
		return;
	}

	printk("Connected\n");

	if (current_conn)
	{
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
	current_conn = bt_conn_ref(conn);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	ARG_UNUSED(conn);

	printk("Disconnected, reason 0x%02x %s\n", reason,
		   bt_hci_err_to_str(reason));

	if (current_conn)
	{
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
}

static void alert_stop(void) { printk("Alert stopped\n"); }
static void alert_start(void) { printk("Mild alert started\n"); }
static void alert_high_start(void) { printk("High alert started\n"); }

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

BT_IAS_CB_DEFINE(ias_callbacks) = {
	.no_alert = alert_stop,
	.mild_alert = alert_start,
	.high_alert = alert_high_start,
};

static void bt_ready(void)
{
	int err;

	printk("Bluetooth initialized\n");

	if (IS_ENABLED(CONFIG_SETTINGS))
	{
		settings_load();
	}

	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad),
						  sd, ARRAY_SIZE(sd));
	if (err)
	{
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
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

#if DT_HAS_COMPAT_STATUS_OKAY(bosch_bmi270)
static const struct device *const bmi = DEVICE_DT_GET_ONE(bosch_bmi270);
#else
#warning "No BMI270 instance found in devicetree (compatible = \"bosch,bmi270\")"
#define bmi NULL
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

	struct sensor_value full_scale;
	struct sensor_value sampling_freq;
	struct sensor_value oversampling;

	uint32_t seq = 0;
	int ret;

	printk("BMI270 dev ptr = %p, name = %s\n", bmi, bmi->name);

	if (!device_is_ready(bmi))
	{
		printk("BMI270 device not ready: %s\n", bmi->name);
		return;
	}

	printk("BMI270 is ready\n");

	/* ACC: ±2G, 100Hz */
	full_scale.val1 = 2;
	full_scale.val2 = 0;
	oversampling.val1 = 1;
	oversampling.val2 = 0;
	sampling_freq.val1 = 100;
	sampling_freq.val2 = 0;

	ret = sensor_attr_set(bmi, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &full_scale);
	printk("attr accel FULL_SCALE ret=%d\n", ret);
	ret = sensor_attr_set(bmi, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_OVERSAMPLING, &oversampling);
	printk("attr accel OVERSAMPLING ret=%d\n", ret);
	ret = sensor_attr_set(bmi, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &sampling_freq);
	printk("attr accel SAMPLING_FREQ ret=%d\n", ret);

	/* GYR: ±500 dps, 100Hz */
	full_scale.val1 = 500;
	full_scale.val2 = 0;
	oversampling.val1 = 1;
	oversampling.val2 = 0;
	sampling_freq.val1 = 100;
	sampling_freq.val2 = 0;

	ret = sensor_attr_set(bmi, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_FULL_SCALE, &full_scale);
	printk("attr gyro FULL_SCALE ret=%d\n", ret);
	ret = sensor_attr_set(bmi, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_OVERSAMPLING, &oversampling);
	printk("attr gyro OVERSAMPLING ret=%d\n", ret);
	ret = sensor_attr_set(bmi, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &sampling_freq);
	printk("attr gyro SAMPLING_FREQ ret=%d\n", ret);

	while (1)
	{
		int err = sensor_sample_fetch(bmi);
		if (err)
		{
			printk("sensor_sample_fetch err=%d\n", err);
			k_msleep(10);
			continue;
		}

		sensor_channel_get(bmi, SENSOR_CHAN_ACCEL_XYZ, acc);
		sensor_channel_get(bmi, SENSOR_CHAN_GYRO_XYZ, gyr);

		int32_t ax_mg32 = sv_to_milli(&acc[0]);
		int32_t ay_mg32 = sv_to_milli(&acc[1]);
		int32_t az_mg32 = sv_to_milli(&acc[2]);

		int32_t gx_mdps = sv_to_milli(&gyr[0]);
		int32_t gy_mdps = sv_to_milli(&gyr[1]);
		int32_t gz_mdps = sv_to_milli(&gyr[2]);

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

		/* store for READ */
		pack_and_store_acc(seq, ax_mg, ay_mg, az_mg);
		pack_and_store_gyr(seq, gx_mdps, gy_mdps, gz_mdps);

		/* UART CSV line (same format as your Python viewer expects) */
		printk("%u,%d,%d,%d,%d,%d,%d\n",
			   seq,
			   (int)ax_mg, (int)ay_mg, (int)az_mg,
			   (int)gx_mdps, (int)gy_mdps, (int)gz_mdps);

		/* BLE notify */
		if (current_conn)
		{
			if (bmi_acc_notify_enabled && bmi_acc_attr)
			{
				(void)bt_gatt_notify(current_conn, bmi_acc_attr,
									 bmi_acc_last, sizeof(bmi_acc_last));
			}
			if (bmi_gyr_notify_enabled && bmi_gyr_attr)
			{
				(void)bt_gatt_notify(current_conn, bmi_gyr_attr,
									 bmi_gyr_last, sizeof(bmi_gyr_last));
			}
		}

		seq++;
		k_msleep(10); /* ~100Hz */
	}
#endif
}

K_THREAD_DEFINE(bmi_thread_id,
				BMI_THREAD_STACK_SIZE,
				bmi_thread,
				NULL, NULL, NULL,
				BMI_THREAD_PRIORITY, 0, 0);

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

	bt_ready();
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

	bt_uuid_to_str(&bmi_acc_uuid.uuid, str, sizeof(str));
	printk("BMI ACC attr %p (UUID %s)\n", bmi_acc_attr, str);
	bt_uuid_to_str(&bmi_gyr_uuid.uuid, str, sizeof(str));
	printk("BMI GYR attr %p (UUID %s)\n", bmi_gyr_attr, str);

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
