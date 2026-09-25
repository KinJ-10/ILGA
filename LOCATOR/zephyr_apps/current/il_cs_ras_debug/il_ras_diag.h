/* ILGA diagnostic extension for NCS v3.2.3 RAS Initiator. */
#ifndef IL_RAS_DIAG_H_
#define IL_RAS_DIAG_H_

#include <math.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>

BUILD_ASSERT(CONFIG_BT_RAS_MAX_ANTENNA_PATHS == 1,
	     "The ILRAS1 capture format currently supports one antenna path");

#define IL_RAS_DIAG_CHANNELS 75
#define IL_RAS_DIAG_MIN_INTERVAL_MS 2000u
#define IL_RAS_DIAG_UNKNOWN_QUALITY 255u

struct il_ras_diag_capture {
	cs_de_report_t report;
	uint8_t local_quality[IL_RAS_DIAG_CHANNELS];
	uint8_t peer_quality[IL_RAS_DIAG_CHANNELS];
	uint32_t sample_id;
	uint32_t ranging_counter;
	uint32_t missed_samples;
	cs_de_quality_t quality;
};

static uint8_t il_ras_diag_local_quality[IL_RAS_DIAG_CHANNELS];
static uint8_t il_ras_diag_peer_quality[IL_RAS_DIAG_CHANNELS];
static struct il_ras_diag_capture il_ras_diag_capture;
static atomic_t il_ras_diag_busy;
static uint32_t il_ras_diag_seen;
static uint32_t il_ras_diag_missed;
static uint32_t il_ras_diag_last_capture_ms;
static struct k_work_q il_ras_diag_queue;
K_THREAD_STACK_DEFINE(il_ras_diag_stack, 2048);

static bool il_ras_diag_ranging_header(struct ras_ranging_header *header, void *user_data)
{
	ARG_UNUSED(header);
	ARG_UNUSED(user_data);
	return true;
}

static bool il_ras_diag_step(struct bt_le_cs_subevent_step *local_step,
			     struct bt_le_cs_subevent_step *peer_step, void *user_data)
{
	ARG_UNUSED(user_data);
	if (local_step->channel < 2 || local_step->channel >= 77 ||
	    local_step->channel != peer_step->channel || local_step->mode != peer_step->mode) {
		return true;
	}

	const struct bt_hci_le_cs_step_data_tone_info *local_tones;
	const struct bt_hci_le_cs_step_data_tone_info *peer_tones;
	if (local_step->mode == BT_HCI_OP_LE_CS_MAIN_MODE_2) {
		if (local_step->data_len < sizeof(struct bt_hci_le_cs_step_data_mode_2) +
					 sizeof(*local_tones) ||
		    peer_step->data_len < sizeof(struct bt_hci_le_cs_step_data_mode_2) +
					 sizeof(*peer_tones)) {
			return true;
		}
		local_tones = ((struct bt_hci_le_cs_step_data_mode_2 *)local_step->data)->tone_info;
		peer_tones = ((struct bt_hci_le_cs_step_data_mode_2 *)peer_step->data)->tone_info;
	} else if (local_step->mode == BT_HCI_OP_LE_CS_MAIN_MODE_3) {
		if (local_step->data_len < sizeof(struct bt_hci_le_cs_step_data_mode_3) +
					 sizeof(*local_tones) ||
		    peer_step->data_len < sizeof(struct bt_hci_le_cs_step_data_mode_3) +
					 sizeof(*peer_tones)) {
			return true;
		}
		local_tones = ((struct bt_hci_le_cs_step_data_mode_3 *)local_step->data)->tone_info;
		peer_tones = ((struct bt_hci_le_cs_step_data_mode_3 *)peer_step->data)->tone_info;
	} else {
		return true;
	}

	uint8_t index = local_step->channel - 2;
	il_ras_diag_local_quality[index] = local_tones[0].quality_indicator;
	il_ras_diag_peer_quality[index] = peer_tones[0].quality_indicator;
	return true;
}

static void il_ras_diag_collect(struct net_buf_simple *local, struct net_buf_simple *peer,
				struct bt_conn_le_cs_config *config)
{
	/* The RAS parser advances buffer cursors, so use copies of the descriptors. */
	struct net_buf_simple local_view = *local;
	struct net_buf_simple peer_view = *peer;

	memset(il_ras_diag_local_quality, IL_RAS_DIAG_UNKNOWN_QUALITY,
	       sizeof(il_ras_diag_local_quality));
	memset(il_ras_diag_peer_quality, IL_RAS_DIAG_UNKNOWN_QUALITY,
	       sizeof(il_ras_diag_peer_quality));
	bt_ras_rreq_rd_subevent_data_parse(&peer_view, &local_view, config->role,
					   il_ras_diag_ranging_header, NULL,
					   il_ras_diag_step, NULL);
}

static int32_t il_ras_diag_fixed(float value, float scale)
{
	if (!isfinite(value) || value * scale > INT32_MAX - 1 ||
	    value * scale < INT32_MIN + 1) {
		return INT32_MIN;
	}
	return (int32_t)lroundf(value * scale);
}

static uint32_t il_ras_diag_hash_line(uint32_t hash, const char *line)
{
	for (const char *p = line; *p; p++) {
		hash = (hash ^ (uint8_t)*p) * 16777619u;
	}
	return (hash ^ (uint8_t)'\n') * 16777619u;
}

static uint32_t il_ras_diag_print_line(uint32_t hash, const char *line)
{
	printk("%s\n", line);
	return il_ras_diag_hash_line(hash, line);
}

static void il_ras_diag_worker(struct k_work *work)
{
	ARG_UNUSED(work);
	const struct il_ras_diag_capture *sample = &il_ras_diag_capture;
	const cs_de_iq_tones_t *iq = &sample->report.iq_tones[0];
	const cs_de_dist_estimates_t *distance = &sample->report.distance_estimates[0];
	char line[160];
	uint32_t hash = 2166136261u;

	snprintk(line, sizeof(line),
		 "ILRAS1,S,%u,%u,%u,%u,%u,%d,%u,%d,%d,%d,%d,%u,%u",
		 sample->sample_id, sample->ranging_counter, sample->report.n_ap,
		 sample->quality, sample->report.tone_quality[0],
		 sample->report.rtt_accumulated_half_ns, sample->report.rtt_count,
		 il_ras_diag_fixed(distance->ifft, 1000.0f),
		 il_ras_diag_fixed(distance->phase_slope, 1000.0f),
		 il_ras_diag_fixed(distance->rtt, 1000.0f),
		 il_ras_diag_fixed(distance->best, 1000.0f), sample->missed_samples,
		 BT_HCI_LE_CS_TONE_QUALITY_HIGH);
	hash = il_ras_diag_print_line(hash, line);
	for (uint8_t i = 0; i < IL_RAS_DIAG_CHANNELS; i++) {
		snprintk(line, sizeof(line),
			 "ILRAS1,T,%u,%u,%u,%u,%d,%d,%d,%d",
			 sample->sample_id, i + 2, sample->local_quality[i],
			 sample->peer_quality[i],
			 il_ras_diag_fixed(iq->i_local[i], 1000000.0f),
			 il_ras_diag_fixed(iq->q_local[i], 1000000.0f),
			 il_ras_diag_fixed(iq->i_remote[i], 1000000.0f),
			 il_ras_diag_fixed(iq->q_remote[i], 1000000.0f));
		hash = il_ras_diag_print_line(hash, line);
	}
	printk("ILRAS1,E,%u,75,%08x\n", sample->sample_id, hash);
	atomic_set(&il_ras_diag_busy, 0);
}

K_WORK_DEFINE(il_ras_diag_work, il_ras_diag_worker);

static void il_ras_diag_init(void)
{
	k_work_queue_start(&il_ras_diag_queue, il_ras_diag_stack,
			   K_THREAD_STACK_SIZEOF(il_ras_diag_stack), 10, NULL);
}

static void il_ras_diag_enqueue(uint32_t ranging_counter, const cs_de_report_t *report,
				cs_de_quality_t quality)
{
	il_ras_diag_seen++;
	uint32_t now = k_uptime_get_32();
	if (now - il_ras_diag_last_capture_ms < IL_RAS_DIAG_MIN_INTERVAL_MS) {
		return;
	}
	if (!atomic_cas(&il_ras_diag_busy, 0, 1)) {
		il_ras_diag_missed++;
		return;
	}
	il_ras_diag_capture.report = *report;
	memcpy(il_ras_diag_capture.local_quality, il_ras_diag_local_quality,
	       sizeof(il_ras_diag_local_quality));
	memcpy(il_ras_diag_capture.peer_quality, il_ras_diag_peer_quality,
	       sizeof(il_ras_diag_peer_quality));
	il_ras_diag_capture.sample_id = il_ras_diag_seen;
	il_ras_diag_capture.ranging_counter = ranging_counter;
	il_ras_diag_capture.missed_samples = il_ras_diag_missed;
	il_ras_diag_capture.quality = quality;
	if (k_work_submit_to_queue(&il_ras_diag_queue, &il_ras_diag_work) < 0) {
		atomic_set(&il_ras_diag_busy, 0);
		il_ras_diag_missed++;
	} else {
		il_ras_diag_last_capture_ms = now;
	}
}

#define IL_RAS_DIAG_INIT() il_ras_diag_init()
#define IL_RAS_DIAG_COLLECT(local, peer, config) il_ras_diag_collect(local, peer, config)
#define IL_RAS_DIAG_ENQUEUE(counter, report, quality) \
	il_ras_diag_enqueue(counter, report, quality)

#endif /* IL_RAS_DIAG_H_ */
