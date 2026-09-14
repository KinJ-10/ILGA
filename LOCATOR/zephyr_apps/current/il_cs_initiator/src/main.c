/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <math.h>
#include <stdarg.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/cs.h>
#include <zephyr/bluetooth/att.h>
#include <zephyr/bluetooth/gatt.h>
#include "distance_estimation.h"
#include "common.h"

#define CS_CONFIG_ID     0
#define NUM_MODE_0_STEPS 1
#define IL_CS_CHANNEL_START 26u
#define IL_CS_NORMAL_CHANNEL_COUNT 36u

#if defined(CONFIG_IL_CS_RAW_DIAGNOSTICS)
#define IL_CS_RAW_VERSION 2
#define IL_CS_STEP_MODE_0 0u
#define IL_CS_PEER_META_MAGIC 0x31435349u /* "ICS1" in little endian */
#define IL_CS_PEER_FLAG_VALID BIT(0)
#define IL_CS_PEER_FLAG_OVERFLOW BIT(1)
#define IL_CS_PEER_FLAG_NO_DATA BIT(2)
#define IL_CS_RAW_RECORD_MAX_LEN 192
#define IL_CS_RAW_LINE_MAX_LEN   (IL_CS_RAW_RECORD_MAX_LEN + 12)

struct il_cs_peer_meta {
	uint32_t magic;
	uint8_t version;
	uint8_t flags;
	uint8_t config_id;
	uint16_t start_acl_conn_event;
	uint16_t procedure_counter;
	uint16_t frequency_compensation;
	int8_t reference_power_level;
	uint8_t procedure_done_status;
	uint8_t subevent_done_status;
	uint8_t procedure_abort_reason;
	uint8_t subevent_abort_reason;
	uint8_t num_antenna_paths;
	uint8_t num_steps_reported;
	uint8_t abort_step;
	uint16_t step_data_len;
} __packed;

BUILD_ASSERT(sizeof(struct il_cs_peer_meta) <= 32,
	     "Peer metadata must fit in one ATT write");

struct il_cs_local_meta {
	uint8_t config_id;
	uint16_t start_acl_conn_event;
	uint16_t procedure_counter;
	uint16_t frequency_compensation;
	int8_t reference_power_level;
	uint8_t procedure_done_status;
	uint8_t subevent_done_status;
	uint8_t procedure_abort_reason;
	uint8_t subevent_abort_reason;
	uint8_t num_antenna_paths;
	uint8_t num_steps_reported;
	uint8_t abort_step;
	uint16_t step_data_len;
	uint8_t transport_flags;
};

static K_MUTEX_DEFINE(raw_uart_mutex);
static const struct device *const raw_uart = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
#endif

static K_SEM_DEFINE(sem_acl_encryption_enabled, 0, 1);
static K_SEM_DEFINE(sem_remote_capabilities_obtained, 0, 1);
static K_SEM_DEFINE(sem_config_created, 0, 1);
static K_SEM_DEFINE(sem_cs_security_enabled, 0, 1);
static K_SEM_DEFINE(sem_procedure_done, 0, 1);
static K_SEM_DEFINE(sem_connected, 0, 1);
static K_SEM_DEFINE(sem_data_received, 0, 1);

static ssize_t on_attr_write_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				const void *buf, uint16_t len, uint16_t offset, uint8_t flags);
static struct bt_conn *connection;
static uint8_t n_ap;
static uint8_t latest_num_steps_reported;
static uint16_t latest_procedure_counter = UINT16_MAX;
static uint16_t latest_step_data_len;
static uint8_t latest_local_steps[STEP_DATA_BUF_LEN];
static uint8_t latest_peer_steps[STEP_DATA_BUF_LEN];
#if defined(CONFIG_IL_CS_RAW_DIAGNOSTICS)
static struct il_cs_local_meta latest_local_meta;
static struct il_cs_peer_meta latest_peer_meta;
static struct il_cs_peer_meta pending_peer_meta;
static bool pending_peer_meta_valid;
static bool latest_peer_data_received;
static uint32_t raw_record_sequence;
#endif

static struct bt_gatt_attr gatt_attributes[] = {
	BT_GATT_PRIMARY_SERVICE(&step_data_svc_uuid),
	BT_GATT_CHARACTERISTIC(&step_data_char_uuid.uuid, BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_WRITE | BT_GATT_PERM_PREPARE_WRITE, NULL,
			       on_attr_write_cb, NULL),
};
static struct bt_gatt_service step_data_gatt_service = BT_GATT_SERVICE(gatt_attributes);
static const char sample_str[] = "IL-CS-TAG";

static ssize_t on_attr_write_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	if (flags & BT_GATT_WRITE_FLAG_PREPARE) {
		return 0;
	}

	if (offset) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

#if defined(CONFIG_IL_CS_RAW_DIAGNOSTICS)
	if (len == sizeof(struct il_cs_peer_meta)) {
		const struct il_cs_peer_meta *meta = buf;

		if (meta->magic != IL_CS_PEER_META_MAGIC || meta->version != IL_CS_RAW_VERSION) {
			return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
		}

		pending_peer_meta = *meta;
		pending_peer_meta_valid = true;
		return len;
	}
#endif

	if (len != sizeof(latest_local_steps)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	if (flags & BT_GATT_WRITE_FLAG_EXECUTE) {
		uint8_t *data = (uint8_t *)buf;

		memset(latest_peer_steps, 0, sizeof(latest_peer_steps));
		memcpy(latest_peer_steps, &data[offset], len);
#if defined(CONFIG_IL_CS_RAW_DIAGNOSTICS)
		latest_peer_data_received = true;
		if (pending_peer_meta_valid) {
			latest_peer_meta = pending_peer_meta;
		} else {
			memset(&latest_peer_meta, 0, sizeof(latest_peer_meta));
			latest_peer_meta.flags = IL_CS_PEER_FLAG_NO_DATA;
		}
		pending_peer_meta_valid = false;
#endif
		k_sem_give(&sem_data_received);
	}

	return len;
}

static void subevent_result_cb(struct bt_conn *conn, struct bt_conn_le_cs_subevent_result *result)
{
	latest_num_steps_reported = result->header.num_steps_reported;
	n_ap = result->header.num_antenna_paths;

	if (result->header.procedure_counter == latest_procedure_counter) {
		printk("The sample does not handle CS procedures with multiple CS subevents.\n");
		latest_procedure_counter = result->header.procedure_counter;
		return;
	}
	latest_procedure_counter = result->header.procedure_counter;
	latest_step_data_len = 0;
	memset(latest_local_steps, 0, sizeof(latest_local_steps));
#if defined(CONFIG_IL_CS_RAW_DIAGNOSTICS)
	latest_local_meta = (struct il_cs_local_meta){
		.config_id = result->header.config_id,
		.start_acl_conn_event = result->header.start_acl_conn_event,
		.procedure_counter = result->header.procedure_counter,
		.frequency_compensation = result->header.frequency_compensation,
		.reference_power_level = result->header.reference_power_level,
		.procedure_done_status = result->header.procedure_done_status,
		.subevent_done_status = result->header.subevent_done_status,
		.procedure_abort_reason = result->header.procedure_abort_reason,
		.subevent_abort_reason = result->header.subevent_abort_reason,
		.num_antenna_paths = result->header.num_antenna_paths,
		.num_steps_reported = result->header.num_steps_reported,
		.abort_step = result->header.abort_step,
		.step_data_len = result->step_data_buf ? result->step_data_buf->len : 0,
		.transport_flags = IL_CS_PEER_FLAG_NO_DATA,
	};
	latest_peer_data_received = false;
#endif

	if (result->step_data_buf) {
		if (result->step_data_buf->len <= STEP_DATA_BUF_LEN) {
			memcpy(latest_local_steps, result->step_data_buf->data,
			       result->step_data_buf->len);
			latest_step_data_len = result->step_data_buf->len;
#if defined(CONFIG_IL_CS_RAW_DIAGNOSTICS)
			latest_local_meta.transport_flags = IL_CS_PEER_FLAG_VALID;
#endif
		} else {
			printk("Not enough memory to store step data. (%d > %d)\n",
			       result->step_data_buf->len, STEP_DATA_BUF_LEN);
			latest_num_steps_reported = 0;
#if defined(CONFIG_IL_CS_RAW_DIAGNOSTICS)
			latest_local_meta.transport_flags = IL_CS_PEER_FLAG_OVERFLOW;
#endif
		}
	}

	if (result->header.procedure_done_status != BT_CONN_LE_CS_PROCEDURE_INCOMPLETE) {
		k_sem_give(&sem_procedure_done);
	}
}

#if defined(CONFIG_IL_CS_RAW_DIAGNOSTICS)
struct il_cs_emit_context {
	uint16_t procedure_counter;
	uint16_t step_index;
	uint8_t num_antenna_paths;
	char side;
};

static uint32_t raw_record_checksum(const char *data, size_t len)
{
	uint32_t hash = 2166136261u;

	for (size_t i = 0; i < len; i++) {
		hash ^= (uint8_t)data[i];
		hash *= 16777619u;
	}

	return hash;
}

static void raw_uart_write_line(const char *line, size_t len)
{
	k_mutex_lock(&raw_uart_mutex, K_FOREVER);
	for (size_t i = 0; i < len; i++) {
		uart_poll_out(raw_uart, line[i]);
	}
	k_mutex_unlock(&raw_uart_mutex);
}

static void emit_raw_record(const char *format, ...)
{
	char record[IL_CS_RAW_RECORD_MAX_LEN];
	char line[IL_CS_RAW_LINE_MAX_LEN];
	int prefix_len = snprintk(record, sizeof(record), "ILCS2,%u,", raw_record_sequence);
	size_t remaining;
	va_list args;
	int payload_len;
	int line_len;

	if (prefix_len < 0 || (size_t)prefix_len >= sizeof(record)) {
		printk("ILCS raw record prefix formatting overflow\n");
		raw_record_sequence++;
		return;
	}

	remaining = sizeof(record) - (size_t)prefix_len;
	va_start(args, format);
	payload_len = vsnprintk(&record[prefix_len], remaining, format, args);
	va_end(args);

	if (payload_len < 0 || (size_t)payload_len >= remaining) {
		printk("ILCS raw record formatting overflow\n");
		raw_record_sequence++;
		return;
	}

	uint32_t checksum = raw_record_checksum(record, prefix_len + payload_len);

	line_len = snprintk(line, sizeof(line), "%s,%08x\r\n", record, checksum);
	if (line_len < 0 || (size_t)line_len >= sizeof(line)) {
		printk("ILCS raw line formatting overflow\n");
		raw_record_sequence++;
		return;
	}

	raw_uart_write_line(line, (size_t)line_len);
	raw_record_sequence++;
#if CONFIG_IL_CS_RAW_RECORD_PACING_MS > 0
	k_msleep(CONFIG_IL_CS_RAW_RECORD_PACING_MS);
#endif
}

static void emit_raw_error(uint16_t procedure_counter, char side, uint16_t step_index,
			   const char *reason)
{
	emit_raw_record("E,%u,%c,%u,%s", procedure_counter, side, step_index, reason);
}

static bool emit_raw_step(struct bt_le_cs_subevent_step *step, void *user_data)
{
	struct il_cs_emit_context *context = user_data;
	uint16_t step_index = context->step_index++;

	switch (step->mode) {
	case IL_CS_STEP_MODE_0:
		if (context->side == 'L') {
			if (step->data_len < sizeof(struct bt_hci_le_cs_step_data_mode_0_initiator)) {
				emit_raw_error(context->procedure_counter, context->side, step_index,
					       "MALFORMED_MODE0");
				return true;
			}
			const struct bt_hci_le_cs_step_data_mode_0_initiator *data =
				(const void *)step->data;

			emit_raw_record("0,%u,%c,%u,%u,%u,%u,%d,%u,%u",
			       context->procedure_counter, context->side, step_index, step->channel,
			       data->packet_quality_aa_check, data->packet_quality_bit_errors,
			       (int8_t)data->packet_rssi, data->packet_antenna,
			       data->measured_freq_offset);
		} else {
			if (step->data_len < sizeof(struct bt_hci_le_cs_step_data_mode_0_reflector)) {
				emit_raw_error(context->procedure_counter, context->side, step_index,
					       "MALFORMED_MODE0");
				return true;
			}
			const struct bt_hci_le_cs_step_data_mode_0_reflector *data =
				(const void *)step->data;

			emit_raw_record("0,%u,%c,%u,%u,%u,%u,%d,%u,NA",
			       context->procedure_counter, context->side, step_index, step->channel,
			       data->packet_quality_aa_check, data->packet_quality_bit_errors,
			       (int8_t)data->packet_rssi, data->packet_antenna);
		}
		break;
	case BT_HCI_OP_LE_CS_MAIN_MODE_1: {
		if (step->data_len < sizeof(struct bt_hci_le_cs_step_data_mode_1)) {
			emit_raw_error(context->procedure_counter, context->side, step_index,
				       "MALFORMED_MODE1");
			return true;
		}
		const struct bt_hci_le_cs_step_data_mode_1 *data = (const void *)step->data;
		int16_t timing = context->side == 'L' ? data->toa_tod_initiator
						       : data->tod_toa_reflector;

		emit_raw_record("1,%u,%c,%u,%u,%u,%u,%u,%d,%d,%u",
		       context->procedure_counter, context->side, step_index, step->channel,
		       data->packet_quality_aa_check, data->packet_quality_bit_errors,
		       data->packet_nadm, (int8_t)data->packet_rssi, timing,
		       data->packet_antenna);
		break;
	}
	case BT_HCI_OP_LE_CS_MAIN_MODE_2: {
		const struct bt_hci_le_cs_step_data_mode_2 *data = (const void *)step->data;
		uint8_t tone_count = context->num_antenna_paths + 1;
		size_t required_len = sizeof(*data) + tone_count * sizeof(data->tone_info[0]);

		if (step->data_len < required_len) {
			emit_raw_error(context->procedure_counter, context->side, step_index,
				       "MALFORMED_MODE2");
			return true;
		}

		for (uint8_t tone_index = 0; tone_index < tone_count; tone_index++) {
			if (data->tone_info[tone_index].extension_indicator !=
			    BT_HCI_LE_CS_NOT_TONE_EXT_SLOT) {
				continue;
			}

			struct bt_le_cs_iq_sample iq =
				bt_le_cs_parse_pct(data->tone_info[tone_index].phase_correction_term);

			emit_raw_record("2,%u,%c,%u,%u,%u,%u,%d,%d,%u,%u",
			       context->procedure_counter, context->side, step_index, step->channel,
			       tone_index, data->antenna_permutation_index, iq.i, iq.q,
			       data->tone_info[tone_index].quality_indicator,
			       data->tone_info[tone_index].extension_indicator);
		}
		break;
	}
	default:
		emit_raw_error(context->procedure_counter, context->side, step_index,
			       "UNSUPPORTED_MODE");
		break;
	}

	return true;
}

static void emit_raw_header(uint16_t procedure_counter, char side, uint8_t config_id,
			    uint16_t start_acl_conn_event, uint16_t frequency_compensation,
			    int8_t reference_power_level, uint8_t procedure_done_status,
			    uint8_t subevent_done_status, uint8_t procedure_abort_reason,
			    uint8_t subevent_abort_reason, uint8_t num_antenna_paths,
			    uint8_t num_steps_reported, uint8_t abort_step, uint16_t step_data_len,
			    uint8_t transport_flags)
{
	emit_raw_record("H,%u,%c,%u,%u,%u,%d,%u,%u,%u,%u,%u,%u,%u,%u,%u",
	       procedure_counter, side, config_id, start_acl_conn_event,
	       frequency_compensation, reference_power_level, procedure_done_status,
	       subevent_done_status, procedure_abort_reason, subevent_abort_reason,
	       num_antenna_paths, num_steps_reported, abort_step, step_data_len,
	       transport_flags);
}

static void emit_raw_procedure(void)
{
	uint16_t procedure_counter = latest_local_meta.procedure_counter;
	bool local_valid = (latest_local_meta.transport_flags & IL_CS_PEER_FLAG_VALID) != 0;
	bool peer_valid = latest_peer_data_received &&
			  (latest_peer_meta.flags & IL_CS_PEER_FLAG_VALID) != 0 &&
			  latest_peer_meta.step_data_len <= STEP_DATA_BUF_LEN;
	bool counters_match = latest_peer_meta.procedure_counter == procedure_counter;

	emit_raw_header(procedure_counter, 'L', latest_local_meta.config_id,
			latest_local_meta.start_acl_conn_event,
			latest_local_meta.frequency_compensation,
			latest_local_meta.reference_power_level,
			latest_local_meta.procedure_done_status,
			latest_local_meta.subevent_done_status,
			latest_local_meta.procedure_abort_reason,
			latest_local_meta.subevent_abort_reason,
			latest_local_meta.num_antenna_paths,
			latest_local_meta.num_steps_reported, latest_local_meta.abort_step,
			latest_local_meta.step_data_len, latest_local_meta.transport_flags);
	emit_raw_header(procedure_counter, 'P', latest_peer_meta.config_id,
			latest_peer_meta.start_acl_conn_event,
			latest_peer_meta.frequency_compensation,
			latest_peer_meta.reference_power_level,
			latest_peer_meta.procedure_done_status,
			latest_peer_meta.subevent_done_status,
			latest_peer_meta.procedure_abort_reason,
			latest_peer_meta.subevent_abort_reason,
			latest_peer_meta.num_antenna_paths,
			latest_peer_meta.num_steps_reported, latest_peer_meta.abort_step,
			latest_peer_meta.step_data_len, latest_peer_meta.flags);

	if (!local_valid) {
		emit_raw_error(procedure_counter, 'L', UINT16_MAX, "LOCAL_DATA_INVALID");
	}
	if (!peer_valid) {
		emit_raw_error(procedure_counter, 'P', UINT16_MAX, "PEER_DATA_INVALID");
	}
	if (!counters_match) {
		emit_raw_error(procedure_counter, 'P', UINT16_MAX, "PROCEDURE_COUNTER_MISMATCH");
	}

	if (local_valid && peer_valid && counters_match) {
		struct net_buf_simple buf;
		struct il_cs_emit_context context = {
			.procedure_counter = procedure_counter,
			.num_antenna_paths = latest_local_meta.num_antenna_paths,
			.side = 'L',
		};

		net_buf_simple_init_with_data(&buf, latest_local_steps, latest_step_data_len);
		bt_le_cs_step_data_parse(&buf, emit_raw_step, &context);

		context.step_index = 0;
		context.num_antenna_paths = latest_peer_meta.num_antenna_paths;
		context.side = 'P';
		net_buf_simple_init_with_data(&buf, latest_peer_steps,
					      latest_peer_meta.step_data_len);
		bt_le_cs_step_data_parse(&buf, emit_raw_step, &context);
	}

	emit_raw_record("Z,%u", procedure_counter);
}
#endif

static void mtu_exchange_cb(struct bt_conn *conn, uint8_t err,
			    struct bt_gatt_exchange_params *params)
{
	printk("MTU exchange %s (%u)\n", err == 0U ? "success" : "failed", bt_gatt_get_mtu(conn));
}

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	(void)bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	printk("Connected to %s (err 0x%02X)\n", addr, err);

	__ASSERT(connection == conn, "Unexpected connected callback");

	if (err) {
		bt_conn_unref(conn);
		connection = NULL;
	}

	static struct bt_gatt_exchange_params mtu_exchange_params = {.func = mtu_exchange_cb};

	err = bt_gatt_exchange_mtu(connection, &mtu_exchange_params);
	if (err) {
		printk("%s: MTU exchange failed (err %d)\n", __func__, err);
	}

	k_sem_give(&sem_connected);
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason 0x%02X)\n", reason);

	bt_conn_unref(conn);
	connection = NULL;
}

static void security_changed_cb(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
	if (err) {
		printk("Encryption failed. (err %d)\n", err);
	} else {
		printk("Security changed to level %d.\n", level);
	}

	k_sem_give(&sem_acl_encryption_enabled);
}

static void remote_capabilities_cb(struct bt_conn *conn,
				   uint8_t status,
				   struct bt_conn_le_cs_capabilities *params)
{
	ARG_UNUSED(params);

	if (status == BT_HCI_ERR_SUCCESS) {
		printk("CS capability exchange completed.\n");
		k_sem_give(&sem_remote_capabilities_obtained);
	} else {
		printk("CS capability exchange failed. (HCI status 0x%02x)\n", status);
	}
}

static void config_create_cb(struct bt_conn *conn,
			     uint8_t status,
			     struct bt_conn_le_cs_config *config)
{
	if (status == BT_HCI_ERR_SUCCESS) {
		printk("CS config creation complete. ID: %d\n", config->id);
		k_sem_give(&sem_config_created);
	} else {
		printk("CS config creation failed. (HCI status 0x%02x)\n", status);
	}
}

static void security_enable_cb(struct bt_conn *conn, uint8_t status)
{
	if (status == BT_HCI_ERR_SUCCESS) {
		printk("CS security enabled.\n");
		k_sem_give(&sem_cs_security_enabled);
	} else {
		printk("CS security enable failed. (HCI status 0x%02x)\n", status);
	}
}

static void procedure_enable_cb(struct bt_conn *conn,
				 uint8_t status,
				 struct bt_conn_le_cs_procedure_enable_complete *params)
{
	if (status == BT_HCI_ERR_SUCCESS) {
		if (params->state == 1) {
			printk("CS procedures enabled.\n");
		} else {
			printk("CS procedures disabled.\n");
		}
	} else {
		printk("CS procedures enable failed. (HCI status 0x%02x)\n", status);
	}
}

static bool data_cb(struct bt_data *data, void *user_data)
{
	char *name = user_data;
	uint8_t len;

	switch (data->type) {
	case BT_DATA_NAME_SHORTENED:
	case BT_DATA_NAME_COMPLETE:
		len = MIN(data->data_len, NAME_LEN - 1);
		memcpy(name, data->data, len);
		name[len] = '\0';
		return false;
	default:
		return true;
	}
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	char addr_str[BT_ADDR_LE_STR_LEN];
	char name[NAME_LEN] = {};
	int err;

	if (connection) {
		return;
	}

	/* We're only interested in connectable events */
	if (type != BT_GAP_ADV_TYPE_ADV_IND && type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
		return;
	}

	bt_data_parse(ad, data_cb, name);

	if (strcmp(name, sample_str)) {
		return;
	}

	if (bt_le_scan_stop()) {
		return;
	}

	printk("Found device with name %s, connecting...\n", name);

	/* 30 ms connection interval x 33 events gives a nominal 0.99 s CS cadence. */
	err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, BT_LE_CONN_PARAM(24, 24, 0, 400),
				&connection);
	if (err) {
		printk("Create conn to %s failed (%u)\n", addr_str, err);
	}
}

struct bt_le_cs_create_config_params get_cs_config_params(void)
{
	/* Create config parameters that result in a limited number of CS steps per CS procedure.
	 * This is achieved by setting channel_map_repetition = 1 and only enabling a limited set
	 * of channels in the channel_map. This is required due to a limitation in the sample:
	 *	The reflector uses a single GATT write operation to transfer all of the step data
	 *	from one CS procedure to the initiator. This limits the amount of CS step data per
	 *	CS procedure that will allow the sample to function to 512 bytes.
	 * This method for limiting the amount of CS steps per CS procedure is chosen because it
	 * will result in the same number of CS steps regardless of controller capabilities.
	 * Other possible methods such as limiting the max_procedure_len in the procedure params
	 * would yield different number of CS steps per CS procedure depending on the
	 * timing parameters supported by the local and peer controller.
	 */
	struct bt_le_cs_create_config_params config_params = {
		.id = CS_CONFIG_ID,
		.mode = BT_CONN_LE_CS_MAIN_MODE_2_SUB_MODE_1,
		.min_main_mode_steps = 2,
		.max_main_mode_steps = 10,
		.main_mode_repetition = 0,
		.mode_0_steps = NUM_MODE_0_STEPS,
		.role = BT_CONN_LE_CS_ROLE_INITIATOR,
		.rtt_type = BT_CONN_LE_CS_RTT_TYPE_AA_ONLY,
		.cs_sync_phy = BT_CONN_LE_CS_SYNC_1M_PHY,
		.channel_map_repetition = 1,
		.channel_selection_type = BT_CONN_LE_CS_CHSEL_TYPE_3B,
		.ch3c_shape = BT_CONN_LE_CS_CH3C_SHAPE_HAT,
		.ch3c_jump = 2,
	};

	memset(config_params.channel_map, 0, 10);
	/* Enable consecutive CS channels.
	 * Start at i = 26 since channels 23, 24 and 25 are disallowed by spec.
	 * The upstream sample comment intends 32 channels, while its loop enables
	 * 36. Keep the existing 36-channel normal profile, but use the configured
	 * 32-channel raw profile to stay below the single-write 512-byte limit.
	 */
	uint8_t channel_count = IL_CS_NORMAL_CHANNEL_COUNT;

#if defined(CONFIG_IL_CS_RAW_DIAGNOSTICS)
	channel_count = CONFIG_IL_CS_RAW_CHANNEL_COUNT;
#endif

	for (uint8_t i = IL_CS_CHANNEL_START; i < IL_CS_CHANNEL_START + channel_count; i++) {
		BT_LE_CS_CHANNEL_BIT_SET_VAL(config_params.channel_map, i, 1);
	}

	return config_params;
}

BT_CONN_CB_DEFINE(conn_cb) = {
	.connected = connected_cb,
	.disconnected = disconnected_cb,
	.security_changed = security_changed_cb,
	.le_cs_read_remote_capabilities_complete = remote_capabilities_cb,
	.le_cs_config_complete = config_create_cb,
	.le_cs_security_enable_complete = security_enable_cb,
	.le_cs_procedure_enable_complete = procedure_enable_cb,
	.le_cs_subevent_data_available = subevent_result_cb,
};

int main(void)
{
	int err;

	printk("Starting ILGA Channel Sounding Phase 0 (LOCATOR/Initiator)\n");
#if defined(CONFIG_IL_CS_RAW_DIAGNOSTICS)
	if (!device_is_ready(raw_uart)) {
		printk("ILCS raw UART device is not ready\n");
		return 0;
	}
	printk("ILCS2 raw diagnostics: %u channels, %u ms record pacing, direct UART\n",
	       CONFIG_IL_CS_RAW_CHANNEL_COUNT, CONFIG_IL_CS_RAW_RECORD_PACING_MS);
#endif

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	err = bt_gatt_service_register(&step_data_gatt_service);
	if (err) {
		printk("bt_gatt_service_register() returned err %d\n", err);
		return 0;
	}

	err = bt_le_scan_start(BT_LE_SCAN_ACTIVE_CONTINUOUS, device_found);
	if (err) {
		printk("Scanning failed to start (err %d)\n", err);
		return 0;
	}

	k_sem_take(&sem_connected, K_FOREVER);

	const struct bt_le_cs_set_default_settings_param default_settings = {
		.enable_initiator_role = true,
		.enable_reflector_role = false,
		.cs_sync_antenna_selection = BT_LE_CS_ANTENNA_SELECTION_OPT_REPETITIVE,
		.max_tx_power = BT_HCI_OP_LE_CS_MAX_MAX_TX_POWER,
	};

	err = bt_le_cs_set_default_settings(connection, &default_settings);
	if (err) {
		printk("Failed to configure default CS settings (err %d)\n", err);
	}

	err = bt_conn_set_security(connection, BT_SECURITY_L2);
	if (err) {
		printk("Failed to encrypt connection (err %d)\n", err);
		return 0;
	}

	k_sem_take(&sem_acl_encryption_enabled, K_FOREVER);

	err = bt_le_cs_read_remote_supported_capabilities(connection);
	if (err) {
		printk("Failed to exchange CS capabilities (err %d)\n", err);
		return 0;
	}

	k_sem_take(&sem_remote_capabilities_obtained, K_FOREVER);

	struct bt_le_cs_create_config_params config_params = get_cs_config_params();

	err = bt_le_cs_create_config(connection, &config_params,
				     BT_LE_CS_CREATE_CONFIG_CONTEXT_LOCAL_AND_REMOTE);

	if (err) {
		printk("Failed to create CS config (err %d)\n", err);
		return 0;
	}

	k_sem_take(&sem_config_created, K_FOREVER);

	err = bt_le_cs_security_enable(connection);
	if (err) {
		printk("Failed to start CS Security (err %d)\n", err);
		return 0;
	}

	k_sem_take(&sem_cs_security_enabled, K_FOREVER);
	const struct bt_le_cs_set_procedure_parameters_param procedure_params = {
		.config_id = CS_CONFIG_ID,
		.max_procedure_len = 0xffff,
		.min_procedure_interval = 33,
		.max_procedure_interval = 33,
		.max_procedure_count = 0,
		/* Use a relatively large subevent_len to make sure the CS procedure
		 * will terminate due to running out of unused channels in the channel map
		 * (and channel map repetitions) in the first CS subevent of the CS procedure.
		 * This will limit the number of CS subevents per CS procedure to 1, which is
		 * required by the design of the sample.
		 */
		.min_subevent_len = 50000,
		.max_subevent_len = 50000,
		.tone_antenna_config_selection = BT_LE_CS_TONE_ANTENNA_CONFIGURATION_A1_B1,
		.phy = BT_LE_CS_PROCEDURE_PHY_1M,
		.tx_power_delta = 0x80,
		.preferred_peer_antenna = BT_LE_CS_PROCEDURE_PREFERRED_PEER_ANTENNA_1,
		.snr_control_initiator = BT_LE_CS_SNR_CONTROL_NOT_USED,
		.snr_control_reflector = BT_LE_CS_SNR_CONTROL_NOT_USED,
	};

	err = bt_le_cs_set_procedure_parameters(connection, &procedure_params);
	if (err) {
		printk("Failed to set procedure parameters (err %d)\n", err);
		return 0;
	}

	struct bt_le_cs_procedure_enable_param params = {
		.config_id = CS_CONFIG_ID,
		.enable = 1,
	};

	err = bt_le_cs_procedure_enable(connection, &params);
	if (err) {
		printk("Failed to enable CS procedures (err %d)\n", err);
		return 0;
	}

	while (true) {
		k_sem_take(&sem_procedure_done, K_FOREVER);
		k_sem_take(&sem_data_received, K_FOREVER);

#if defined(CONFIG_IL_CS_RAW_DIAGNOSTICS)
		if ((latest_local_meta.procedure_counter % CONFIG_IL_CS_RAW_EVERY_N) == 0) {
			emit_raw_procedure();
		}

		if (latest_local_meta.transport_flags != IL_CS_PEER_FLAG_VALID ||
		    latest_peer_meta.flags != IL_CS_PEER_FLAG_VALID ||
		    latest_peer_meta.procedure_counter != latest_local_meta.procedure_counter ||
		    latest_peer_meta.step_data_len > STEP_DATA_BUF_LEN) {
			printk("A reliable distance estimate could not be computed.\n");
			continue;
		}

		estimate_distance(latest_local_steps, latest_step_data_len, latest_peer_steps,
				  latest_peer_meta.step_data_len, n_ap,
				  BT_CONN_LE_CS_ROLE_INITIATOR);
#else
		if (latest_step_data_len <
		    NUM_MODE_0_STEPS *
			    (sizeof(struct bt_hci_le_cs_step_data_mode_0_initiator) -
			     sizeof(struct bt_hci_le_cs_step_data_mode_0_reflector))) {
			printk("A reliable distance estimate could not be computed.\n");
			continue;
		}

		estimate_distance(
			latest_local_steps, latest_step_data_len, latest_peer_steps,
			latest_step_data_len -
				NUM_MODE_0_STEPS *
					(sizeof(struct bt_hci_le_cs_step_data_mode_0_initiator) -
					 sizeof(struct bt_hci_le_cs_step_data_mode_0_reflector)),
			n_ap, BT_CONN_LE_CS_ROLE_INITIATOR);
#endif
	}

	return 0;
}
