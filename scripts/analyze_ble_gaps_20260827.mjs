#!/usr/bin/env node

import fs from "node:fs/promises";
import { existsSync, mkdirSync, writeFileSync } from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const scriptDir = path.dirname(fileURLToPath(import.meta.url));
const repoRoot = path.resolve(scriptDir, "..");
const inputDir = path.join(repoRoot, "logs", "ble");
const outputDir = path.join(
  repoRoot,
  "analysis_out",
  "ble_gap_investigation_20260827",
);

const trials = [
  ["Kin", "walk1", "20260827_ga_phase1a_Kin_right_walk1.csv"],
  ["Kin", "walk2", "20260827_ga_phase1a_Kin_right_walk2.csv"],
  ["Kin", "walk3", "20260827_ga_phase1a_Kin_right_walk3.csv"],
  ["Kin", "stop30s", "20260827_ga_phase1a_Kin_stop30s_right.csv"],
  ["doi", "walk1", "20260827_ga_phase1a_doi_right_walk1.csv"],
  ["doi", "walk2", "20260827_ga_phase1a_doi_right_walk2.csv"],
  ["doi", "walk3", "20260827_ga_phase1a_doi_right_walk3.csv"],
  ["doi", "stop30s", "20260827_ga_phase1a_doi_stop30s_right.csv"],
];

const activityThreshold = {
  gyroMdps: 30_000,
  accelDeviationMg: 150,
  joinGapSec: 0.5,
};

function percentile(values, fraction) {
  if (values.length === 0) return 0;
  const ordered = [...values].sort((a, b) => a - b);
  const position = (ordered.length - 1) * fraction;
  const lower = Math.floor(position);
  const upper = Math.ceil(position);
  return ordered[lower] + (ordered[upper] - ordered[lower]) * (position - lower);
}

function parseCsv(text) {
  const lines = text.trim().split(/\r?\n/);
  const header = lines[0].split(",");
  return lines.slice(1).map((line) => {
    const values = line.split(",").map(Number);
    return Object.fromEntries(header.map((name, index) => [name, values[index]]));
  });
}

function csvCell(value) {
  if (value === null || value === undefined) return "";
  const text = String(value);
  return /[",\r\n]/.test(text) ? `"${text.replaceAll('"', '""')}"` : text;
}

function toCsv(columns, rows) {
  return [
    columns.join(","),
    ...rows.map((row) => columns.map((column) => csvCell(row[column])).join(",")),
  ].join("\n") + "\n";
}

function getActivityGroups(rows) {
  const active = rows.filter(
    (row) =>
      row.gyro_norm_mdps >= activityThreshold.gyroMdps ||
      row.accel_deviation_mg >= activityThreshold.accelDeviationMg,
  );
  const groups = [];
  for (const row of active) {
    let group = groups.at(-1);
    if (!group || row.elapsed_s - group.end_s > activityThreshold.joinGapSec) {
      group = {
        start_s: row.elapsed_s,
        end_s: row.elapsed_s,
        active_samples: 0,
        max_gyro_mdps: 0,
        max_accel_deviation_mg: 0,
      };
      groups.push(group);
    }
    group.end_s = row.elapsed_s;
    group.active_samples += 1;
    group.max_gyro_mdps = Math.max(group.max_gyro_mdps, row.gyro_norm_mdps);
    group.max_accel_deviation_mg = Math.max(
      group.max_accel_deviation_mg,
      row.accel_deviation_mg,
    );
  }
  return groups;
}

function phaseForEvent(condition, elapsedS, mainGroup, secondaryGroup) {
  if (condition === "stop30s") return "stop";
  if (!mainGroup) return "unclassified";
  if (elapsedS < mainGroup.start_s) return "pre_walk_stillness";
  if (elapsedS <= mainGroup.end_s) {
    return elapsedS >= mainGroup.end_s - 0.5 ? "terminal_movement" : "main_walk";
  }
  if (elapsedS <= mainGroup.end_s + 0.25) return "terminal_movement";
  if (secondaryGroup && elapsedS >= secondaryGroup.start_s - 0.5) {
    return elapsedS < secondaryGroup.start_s
      ? "post_trial_movement_onset"
      : "post_trial_movement";
  }
  return "post_walk_stillness";
}

function rangeText(values, digits = 2) {
  if (values.length === 0) return "n/a";
  return `${Math.min(...values).toFixed(digits)}–${Math.max(...values).toFixed(digits)}`;
}

const gapRows = [];
const summaries = [];

for (const [participant, condition, filename] of trials) {
  const sourcePath = path.join(inputDir, filename);
  const rows = parseCsv(await fs.readFile(sourcePath, "utf8"));
  rows.forEach((row) => {
    row.elapsed_s = row.rx_elapsed_ns / 1_000_000_000;
    row.gyro_norm_mdps = Math.hypot(row.gx_mdps, row.gy_mdps, row.gz_mdps);
    row.accel_deviation_mg = Math.abs(
      Math.hypot(row.ax_mg, row.ay_mg, row.az_mg) - 1000,
    );
  });

  const clusterSize = Array(rows.length).fill(1);
  for (let start = 0; start < rows.length; ) {
    let end = start + 1;
    while (
      end < rows.length &&
      rows[end].rx_monotonic_ns === rows[start].rx_monotonic_ns
    ) {
      end += 1;
    }
    for (let index = start; index < end; index += 1) {
      clusterSize[index] = end - start;
    }
    start = end;
  }

  const interArrivalNs = [];
  let duplicateCount = 0;
  let staleCount = 0;
  let missingTotal = 0;
  let episode = 0;
  let previousGapTime = -Infinity;
  const activityGroups = getActivityGroups(rows);
  const mainGroup = condition === "stop30s" ? null : activityGroups[0] ?? null;
  const secondaryGroup = condition === "stop30s" ? null : activityGroups[1] ?? null;
  const trialGapRows = [];

  for (let index = 1; index < rows.length; index += 1) {
    const previous = rows[index - 1];
    const current = rows[index];
    interArrivalNs.push(current.rx_monotonic_ns - previous.rx_monotonic_ns);
    const delta = (current.seq - previous.seq) >>> 0;
    if (delta === 0) {
      duplicateCount += 1;
      continue;
    }
    if (delta >= 0x80000000) {
      staleCount += 1;
      continue;
    }
    if (delta <= 1) continue;

    const missing = delta - 1;
    missingTotal += missing;
    if (current.elapsed_s - previousGapTime > 0.75) episode += 1;
    previousGapTime = current.elapsed_s;
    const local = rows.slice(Math.max(0, index - 50), Math.min(rows.length, index + 51));
    const phase = phaseForEvent(condition, current.elapsed_s, mainGroup, secondaryGroup);
    const event = {
      trial_id: `${participant}_${condition}`,
      participant,
      condition,
      source_file: filename,
      gap_episode: episode,
      seq_before: previous.seq,
      missing_seq_start: (previous.seq + 1) >>> 0,
      missing_seq_end: (current.seq - 1) >>> 0,
      seq_after: current.seq,
      consecutive_missing_length: missing,
      previous_elapsed_s: previous.elapsed_s.toFixed(6),
      detected_elapsed_s: current.elapsed_s.toFixed(6),
      rx_interval_prev_ms: (
        (current.rx_monotonic_ns - previous.rx_monotonic_ns) /
        1_000_000
      ).toFixed(3),
      rx_interval_next_ms:
        index + 1 < rows.length
          ? (
              (rows[index + 1].rx_monotonic_ns - current.rx_monotonic_ns) /
              1_000_000
            ).toFixed(3)
          : "",
      same_timestamp_cluster_size: clusterSize[index],
      same_timestamp_concentrated: clusterSize[index] > 1 ? 1 : 0,
      local_gyro_median_mdps: Math.round(
        percentile(local.map((row) => row.gyro_norm_mdps), 0.5),
      ),
      local_gyro_max_mdps: Math.round(
        Math.max(...local.map((row) => row.gyro_norm_mdps)),
      ),
      local_accel_deviation_median_mg: Math.round(
        percentile(local.map((row) => row.accel_deviation_mg), 0.5),
      ),
      local_accel_deviation_max_mg: Math.round(
        Math.max(...local.map((row) => row.accel_deviation_mg)),
      ),
      phase_classification: phase,
      phase_basis: "signal-derived; gyro>=30000 mdps or |acc_norm-1000|>=150 mg",
    };
    trialGapRows.push(event);
    gapRows.push(event);
  }

  const durationS = (rows.at(-1).rx_elapsed_ns - rows[0].rx_elapsed_ns) / 1e9;
  const zeroIntervals = interArrivalNs.filter((value) => value === 0).length;
  const phaseCounts = Object.entries(
    trialGapRows.reduce((counts, row) => {
      counts[row.phase_classification] = (counts[row.phase_classification] ?? 0) + 1;
      return counts;
    }, {}),
  )
    .map(([phase, count]) => `${phase}:${count}`)
    .join(";");
  const missingAtSameTimestamp = trialGapRows
    .filter((row) => row.same_timestamp_concentrated === 1)
    .reduce((total, row) => total + row.consecutive_missing_length, 0);

  summaries.push({
    trial_id: `${participant}_${condition}`,
    participant,
    condition,
    source_file: filename,
    accepted_rows: rows.length,
    first_seq: rows[0].seq,
    last_seq: rows.at(-1).seq,
    expected_seq_count: rows.length + missingTotal,
    duration_s: durationS.toFixed(6),
    rx_timestamp_effective_hz:
      durationS > 0 ? ((rows.length - 1) / durationS).toFixed(6) : "0",
    missing_seq_total: missingTotal,
    missing_rate_percent: (
      (100 * missingTotal) /
      (rows.length + missingTotal)
    ).toFixed(4),
    gap_event_count: trialGapRows.length,
    gap_episode_count: episode,
    duplicate_count: duplicateCount,
    stale_count: staleCount,
    inter_arrival_median_ms: (percentile(interArrivalNs, 0.5) / 1e6).toFixed(3),
    inter_arrival_p95_ms: (percentile(interArrivalNs, 0.95) / 1e6).toFixed(3),
    inter_arrival_max_ms: (Math.max(...interArrivalNs) / 1e6).toFixed(3),
    zero_interval_count: zeroIntervals,
    zero_interval_percent: ((100 * zeroIntervals) / interArrivalNs.length).toFixed(3),
    max_same_timestamp_cluster_size: Math.max(...clusterSize),
    gap_events_same_timestamp: trialGapRows.filter(
      (row) => row.same_timestamp_concentrated === 1,
    ).length,
    missing_samples_same_timestamp_events: missingAtSameTimestamp,
    long_interval_ge_100ms_count: interArrivalNs.filter((value) => value >= 100_000_000)
      .length,
    main_activity_start_s: mainGroup ? mainGroup.start_s.toFixed(3) : "",
    main_activity_end_s: mainGroup ? mainGroup.end_s.toFixed(3) : "",
    secondary_activity_start_s: secondaryGroup ? secondaryGroup.start_s.toFixed(3) : "",
    secondary_activity_end_s: secondaryGroup ? secondaryGroup.end_s.toFixed(3) : "",
    gap_phase_counts: phaseCounts,
  });
}

const gapColumns = [
  "trial_id", "participant", "condition", "source_file", "gap_episode",
  "seq_before", "missing_seq_start", "missing_seq_end", "seq_after",
  "consecutive_missing_length", "previous_elapsed_s", "detected_elapsed_s",
  "rx_interval_prev_ms", "rx_interval_next_ms", "same_timestamp_cluster_size",
  "same_timestamp_concentrated", "local_gyro_median_mdps", "local_gyro_max_mdps",
  "local_accel_deviation_median_mg", "local_accel_deviation_max_mg",
  "phase_classification", "phase_basis",
];
const summaryColumns = Object.keys(summaries[0]);

const gapTrials = summaries.filter((row) => row.missing_seq_total > 0);
const cleanTrials = summaries.filter((row) => row.missing_seq_total === 0);
const sameTimestampGapEvents = gapRows.filter(
  (row) => row.same_timestamp_concentrated === 1,
).length;
const phaseTotals = gapRows.reduce((counts, row) => {
  counts[row.phase_classification] = (counts[row.phase_classification] ?? 0) + 1;
  return counts;
}, {});

const report = `# BLE gap investigation — 2026-08-27 GA-Phase1A

## Scope and method

- Scope: Kin/doi walk1, walk2, walk3 and stop30s (8 CSV files). The two 4g_test files are excluded.
- Gap definition: a forward seq delta greater than 1. Each row in \`gap_events.csv\` is one contiguous missing-seq run.
- Receive timing uses PC-side \`rx_monotonic_ns\`; it is not sensor acquisition time.
- Activity boundaries are signal-derived with gyro norm >= ${activityThreshold.gyroMdps} mdps or |acceleration norm - 1000 mg| >= ${activityThreshold.accelDeviationMg} mg, joining active points separated by <= ${activityThreshold.joinGapSec} s.
- Phase labels are approximate because the CSV contains no operator event markers. In particular, three synchronization stomps cannot always be separated from the following walk.

## Trial summary

| Trial | Rows | Missing seq | Gap events / episodes | Timestamp Hz | median / p95 / max rx interval ms | zero-interval % | Signal-derived gap phases |
|---|---:|---:|---:|---:|---:|---:|---|
${summaries.map((row) => `| ${row.trial_id} | ${row.accepted_rows} | ${row.missing_seq_total} | ${row.gap_event_count} / ${row.gap_episode_count} | ${Number(row.rx_timestamp_effective_hz).toFixed(3)} | ${row.inter_arrival_median_ms} / ${row.inter_arrival_p95_ms} / ${row.inter_arrival_max_ms} | ${row.zero_interval_percent} | ${row.gap_phase_counts || "none"} |`).join("\n")}

## Facts supported directly by the CSVs

1. Missing seq occurs only in Kin walk1 (${gapTrials.find((r) => r.trial_id === "Kin_walk1").missing_seq_total}), Kin walk2 (${gapTrials.find((r) => r.trial_id === "Kin_walk2").missing_seq_total}), Kin walk3 (${gapTrials.find((r) => r.trial_id === "Kin_walk3").missing_seq_total}) and doi walk2 (${gapTrials.find((r) => r.trial_id === "doi_walk2").missing_seq_total}). Kin stop30s and doi walk1/walk3/stop30s have zero missing seq.
2. No event is classified in the middle of the main walking interval. Kin walk3 has terminal-movement events at the detected end of its main activity; the remaining events are post-walk stillness, post-trial movement onset, or post-trial movement. Phase totals: ${Object.entries(phaseTotals).map(([name, count]) => `${name}=${count}`).join(", ")}.
3. Gap trials have zero-interval percentages ${rangeText(gapTrials.map((r) => Number(r.zero_interval_percent)), 3)}%, versus ${rangeText(cleanTrials.map((r) => Number(r.zero_interval_percent)), 3)}% for gap-free trials. Gap-trial p95 is ${rangeText(gapTrials.map((r) => Number(r.inter_arrival_p95_ms)))} ms; gap-free p95 is ${rangeText(cleanTrials.map((r) => Number(r.inter_arrival_p95_ms)))} ms.
4. Gap-trial maximum receive intervals are ${rangeText(gapTrials.map((r) => Number(r.inter_arrival_max_ms)), 0)} ms, versus ${rangeText(cleanTrials.map((r) => Number(r.inter_arrival_max_ms)), 0)} ms for gap-free trials.
5. ${sameTimestampGapEvents}/${gapRows.length} gap events are observed in a repeated-timestamp cluster. All trials, including gap-free trials, contain repeated timestamps and dominant 0/15–16 ms intervals, showing that the Windows timestamp is quantized near the timer tick and cannot resolve true 10 ms callback spacing.
6. Duplicate and stale seq counts are zero in all eight files.

## Interpretation: fact versus inference

| Candidate | What the data supports | What the data cannot establish | Assessment |
|---|---|---|---|
| CSV writer | The same writer path produced both clean and gapped trials. Timestamp is captured before decode/print/write. | Handler duration, filesystem latency and queue occupancy were not logged. A delayed handler could still contribute to upstream queue pressure. | Not demonstrated; cannot be excluded. |
| Windows timer resolution | Exact repeated timestamps and 15–16 ms quantization occur in all trials. | Quantized timestamps cannot distinguish callbacks delivered microseconds apart from callbacks delivered within one coarse tick. | Confirmed measurement limitation, not by itself a cause of seq loss. |
| BLE notification batching / host stack buffering | Gap trials show more zero intervals, 32 ms p95, large same-timestamp clusters and 453–1266 ms maximum intervals. | The CSV has no HCI timestamps or queue-depth counters, so batching layer and drop point are unknown. | Strongly consistent with delayed/batched host delivery around receiver stalls. |
| Wireless link | Seq loss could arise if radio throughput temporarily falls below generation rate. | No RSSI, PHY, connection interval, retransmission or packet-error data exists. Gaps mostly occur after main walking rather than during strongest body motion. | Possible, but not supported strongly enough to identify RF as the cause. |
| OS scheduling / other applications | Long receive stalls followed by concentrated callbacks are compatible with a delayed Windows/Bleak event loop. | CPU load, power mode, process scheduling and competing BLE-client activity were not recorded. | Plausible leading host-side explanation; requires controlled A/B measurement. |

The CSVs identify receiver-observed loss and timing concentration, but they do not uniquely identify the layer that dropped notifications. The host-side explanation is an inference, not a confirmed root cause.

## Conditions to record in the next measurement

1. Keep the PC-TAG distance fixed and record it in metres; use the same TAG orientation and body side for all trials.
2. Record whether the PC is on AC power, its Windows power mode, and whether the lid/display state changes.
3. Close other BLE clients and record any high-load applications that remain open. Do not run a second Viewer instance.
4. Record whether each trial reuses the BLE connection or disconnects/reconnects. Prefer a deliberate disconnect/reconnect between trials and note the outcome.
5. Record trial start/end wall-clock time, Trial name, participant, three synchronization stomps, stillness duration, 10 m walk, actual step count, starting foot and end condition.
6. Keep the receiver console and CSV path on the same local disk. For diagnosis, compare a normal capture with a no-CSV run whose console output is redirected, while keeping every other condition fixed; the current CLI still emits per-sample output, so this is not a pure writer-only experiment.
7. If gaps recur, collect Windows CPU/load and Bluetooth adapter identity. A later instrumented build should add callback/queue depth and HCI-level timestamps before assigning cause to the writer, host stack or RF link.

## Phase classification caveat

Phase counts are derived from inertial activity windows, not manually annotated protocol markers. They are useful for separating the main activity window from later motion, but the exact boundary between terminal operation and subsequent movement should be confirmed against operator notes.
`;

const outputs = [
  { path: path.join(outputDir, "gap_events.csv"), text: toCsv(gapColumns, gapRows) },
  { path: path.join(outputDir, "trial_summary.csv"), text: toCsv(summaryColumns, summaries) },
  { path: path.join(outputDir, "report.md"), text: report },
];

for (const output of outputs) {
  if (existsSync(output.path)) {
    throw new Error(`Refusing to overwrite existing output: ${output.path}`);
  }
}

mkdirSync(outputDir, { recursive: false });
for (const output of outputs) {
  writeFileSync(output.path, output.text, "utf8");
  console.log(`Wrote ${output.path}`);
}
console.log(`Gap events: ${gapRows.length}; trial summaries: ${summaries.length}`);
