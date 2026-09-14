#!/usr/bin/env python3
"""Parse ILCS1/ILCS2 UART records and recalculate basic CS diagnostics."""

from __future__ import annotations

import argparse
import csv
import math
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any, Iterable


SPEED_OF_LIGHT_M_PER_S = 299_792_458.0
SPEED_OF_LIGHT_NM_PER_S = SPEED_OF_LIGHT_M_PER_S / 1_000_000_000.0
TIME_NOT_AVAILABLE = -32768
RSSI_NOT_AVAILABLE = 127
GOOD_TONE_QUALITIES = {0, 1}
PEER_FLAG_VALID = 1
PROCEDURE_COUNTER_SPACE = 0x10000


class RawDataError(ValueError):
    """Raised when an input is not an ILCS raw capture."""


def _new_procedure() -> dict[str, Any]:
    return {
        "boot_index": 0,
        "procedure_counter": None,
        "headers": {},
        "mode0": {"L": [], "P": []},
        "rtt": {"L": [], "P": []},
        "pbr": {"L": [], "P": []},
        "errors": [],
        "parse_errors": [],
        "ended": False,
    }


def _get_procedure(
    procedures: dict[int, dict[str, Any]], boot_index: int, procedure_counter: int
) -> dict[str, Any]:
    key = boot_index * PROCEDURE_COUNTER_SPACE + procedure_counter
    procedure = procedures[key]
    procedure["boot_index"] = boot_index
    procedure["procedure_counter"] = procedure_counter
    return procedure


def _int(value: str) -> int:
    return int(value, 0)


def fnv1a32(value: str) -> int:
    checksum = 2_166_136_261
    for byte in value.encode("utf-8"):
        checksum ^= byte
        checksum = (checksum * 16_777_619) & 0xFFFFFFFF
    return checksum


def _require_range(name: str, value: int, minimum: int, maximum: int) -> None:
    if not minimum <= value <= maximum:
        raise ValueError(f"{name} out of range: {value}")


def _require_member(name: str, value: Any, allowed: set[Any]) -> None:
    if value not in allowed:
        raise ValueError(f"{name} has invalid value: {value}")


def _parse_error(
    parse_errors: list[dict[str, Any]],
    procedures: dict[int, dict[str, Any]],
    *,
    boot_index: int,
    line_number: int,
    host_timestamp: str,
    schema: str,
    record_sequence: int | None,
    procedure_counter: int | None,
    line: str,
    reason: str,
) -> None:
    error = {
        "boot_index": boot_index,
        "line_number": line_number,
        "host_timestamp": host_timestamp,
        "schema": schema,
        "record_sequence": record_sequence,
        "procedure_counter": procedure_counter,
        "line": line,
        "reason": reason,
    }
    parse_errors.append(error)
    if procedure_counter is not None:
        _get_procedure(procedures, boot_index, procedure_counter)["parse_errors"].append(reason)


def parse_raw_log(path: Path) -> tuple[dict[int, dict[str, Any]], list[dict[str, Any]]]:
    procedures: dict[int, dict[str, Any]] = defaultdict(_new_procedure)
    parse_errors: list[dict[str, Any]] = []
    raw_record_count = 0
    previous_record_sequence: int | None = None
    boot_index = 0

    with path.open("r", encoding="utf-8", errors="replace", newline="") as stream:
        for line_number, raw_line in enumerate(stream, 1):
            line_terminated = raw_line.endswith(("\r", "\n"))
            line = raw_line.rstrip("\r\n")
            markers = [position for prefix in ("ILCS2,", "ILCS1,") if (position := line.find(prefix)) >= 0]
            if not markers:
                continue
            marker = min(markers)
            host_timestamp = line[:marker].rstrip("\t ")
            wire_line = line[marker:]
            wire_fields = next(csv.reader([wire_line]))
            schema = wire_fields[0] if wire_fields else "UNKNOWN"
            raw_record_count += 1
            record_sequence: int | None = None
            procedure_counter: int | None = None

            procedure_index = 3 if schema == "ILCS2" else 2
            if len(wire_fields) > procedure_index:
                try:
                    procedure_counter = _int(wire_fields[procedure_index])
                except ValueError:
                    pass

            try:
                if schema == "ILCS2":
                    if len(wire_fields) < 5:
                        raise ValueError("ILCS2 record is too short")
                    record_sequence = _int(wire_fields[1])
                    _require_range("record_sequence", record_sequence, 0, 0xFFFFFFFF)
                    if len(wire_fields[-1]) != 8:
                        raise ValueError("CHECKSUM_FIELD_INVALID")
                    supplied_checksum = int(wire_fields[-1], 16)
                    checksum_input = ",".join(wire_fields[:-1])
                    expected_checksum = fnv1a32(checksum_input)
                    if supplied_checksum != expected_checksum:
                        raise ValueError(
                            f"CHECKSUM_MISMATCH expected={expected_checksum:08x} "
                            f"actual={supplied_checksum:08x}"
                        )
                    fields = [wire_fields[0], *wire_fields[2:-1]]
                elif schema == "ILCS1":
                    fields = wire_fields
                else:
                    raise ValueError(f"unsupported schema: {schema}")

                record_type = fields[1]
                procedure_counter = _int(fields[2])
                _require_range("procedure_counter", procedure_counter, 0, 0xFFFF)

                if record_sequence is not None:
                    if (
                        record_sequence == 0
                        and previous_record_sequence is not None
                        and previous_record_sequence != 0xFFFFFFFF
                    ):
                        boot_index += 1
                        previous_record_sequence = None
                    if previous_record_sequence is not None:
                        expected_sequence = (previous_record_sequence + 1) & 0xFFFFFFFF
                        if record_sequence != expected_sequence:
                            reason = (
                                f"RECORD_SEQUENCE_GAP expected={expected_sequence} "
                                f"actual={record_sequence}"
                            )
                            _parse_error(
                                parse_errors,
                                procedures,
                                boot_index=boot_index,
                                line_number=line_number,
                                host_timestamp=host_timestamp,
                                schema=schema,
                                record_sequence=record_sequence,
                                procedure_counter=procedure_counter,
                                line=wire_line,
                                reason=reason,
                            )
                    previous_record_sequence = record_sequence

                procedure = _get_procedure(procedures, boot_index, procedure_counter)

                if record_type == "H" and len(fields) == 17:
                    side = fields[3]
                    header = {
                        "boot_index": boot_index,
                        "host_timestamp": host_timestamp,
                        "record_sequence": record_sequence,
                        "procedure_counter": procedure_counter,
                        "side": side,
                        "config_id": _int(fields[4]),
                        "start_acl_conn_event": _int(fields[5]),
                        "frequency_compensation": _int(fields[6]),
                        "reference_power_level": _int(fields[7]),
                        "procedure_done_status": _int(fields[8]),
                        "subevent_done_status": _int(fields[9]),
                        "procedure_abort_reason": _int(fields[10]),
                        "subevent_abort_reason": _int(fields[11]),
                        "num_antenna_paths": _int(fields[12]),
                        "num_steps_reported": _int(fields[13]),
                        "abort_step": _int(fields[14]),
                        "step_data_len": _int(fields[15]),
                        "transport_flags": _int(fields[16]),
                    }
                    _require_member("side", side, {"L", "P"})
                    _require_range("config_id", header["config_id"], 0, 3)
                    _require_range("start_acl_conn_event", header["start_acl_conn_event"], 0, 0xFFFF)
                    _require_range("frequency_compensation", header["frequency_compensation"], 0, 0xFFFF)
                    _require_range("reference_power_level", header["reference_power_level"], -127, 127)
                    _require_member("procedure_done_status", header["procedure_done_status"], {0, 1, 15})
                    _require_member("subevent_done_status", header["subevent_done_status"], {0, 1, 15})
                    _require_member("procedure_abort_reason", header["procedure_abort_reason"], {0, 1, 2, 3, 15})
                    _require_member("subevent_abort_reason", header["subevent_abort_reason"], {0, 1, 2, 3, 15})
                    _require_range("num_antenna_paths", header["num_antenna_paths"], 0, 4)
                    _require_range("num_steps_reported", header["num_steps_reported"], 0, 255)
                    _require_range("abort_step", header["abort_step"], 0, 255)
                    _require_range("step_data_len", header["step_data_len"], 0, 0xFFFF)
                    if header["transport_flags"] & ~0x07:
                        raise ValueError(f"transport_flags has unknown bits: {header['transport_flags']}")
                    procedure["headers"][side] = header
                elif record_type == "0" and len(fields) == 11:
                    side = fields[3]
                    row = {
                            "boot_index": boot_index,
                            "host_timestamp": host_timestamp,
                            "record_sequence": record_sequence,
                            "procedure_counter": procedure_counter,
                            "side": side,
                            "step_index": _int(fields[4]),
                            "channel": _int(fields[5]),
                            "aa_quality": _int(fields[6]),
                            "bit_errors": _int(fields[7]),
                            "rssi_dbm": _int(fields[8]),
                            "antenna": _int(fields[9]),
                            "measured_freq_offset": None
                            if fields[10] == "NA"
                            else _int(fields[10]),
                    }
                    _require_member("side", side, {"L", "P"})
                    _require_range("step_index", row["step_index"], 0, 0xFFFF)
                    _require_range("channel", row["channel"], 0, 78)
                    _require_member("aa_quality", row["aa_quality"], {0, 1, 2})
                    _require_range("bit_errors", row["bit_errors"], 0, 15)
                    _require_range("rssi_dbm", row["rssi_dbm"], -128, 127)
                    procedure["mode0"][side].append(row)
                elif record_type == "1" and len(fields) == 12:
                    side = fields[3]
                    row = {
                            "boot_index": boot_index,
                            "host_timestamp": host_timestamp,
                            "record_sequence": record_sequence,
                            "procedure_counter": procedure_counter,
                            "side": side,
                            "step_index": _int(fields[4]),
                            "channel": _int(fields[5]),
                            "aa_quality": _int(fields[6]),
                            "bit_errors": _int(fields[7]),
                            "nadm": _int(fields[8]),
                            "rssi_dbm": _int(fields[9]),
                            "timing": _int(fields[10]),
                            "antenna": _int(fields[11]),
                    }
                    _require_member("side", side, {"L", "P"})
                    _require_range("step_index", row["step_index"], 0, 0xFFFF)
                    _require_range("channel", row["channel"], 0, 78)
                    _require_member("aa_quality", row["aa_quality"], {0, 1, 2})
                    _require_range("bit_errors", row["bit_errors"], 0, 15)
                    _require_member("nadm", row["nadm"], {0, 1, 2, 3, 4, 5, 6, 255})
                    _require_range("rssi_dbm", row["rssi_dbm"], -128, 127)
                    _require_range("timing", row["timing"], -32768, 32767)
                    procedure["rtt"][side].append(row)
                elif record_type == "2" and len(fields) == 12:
                    side = fields[3]
                    row = {
                            "boot_index": boot_index,
                            "host_timestamp": host_timestamp,
                            "record_sequence": record_sequence,
                            "procedure_counter": procedure_counter,
                            "side": side,
                            "step_index": _int(fields[4]),
                            "channel": _int(fields[5]),
                            "tone_index": _int(fields[6]),
                            "permutation": _int(fields[7]),
                            "i": _int(fields[8]),
                            "q": _int(fields[9]),
                            "quality": _int(fields[10]),
                            "extension": _int(fields[11]),
                    }
                    _require_member("side", side, {"L", "P"})
                    _require_range("step_index", row["step_index"], 0, 0xFFFF)
                    _require_range("channel", row["channel"], 0, 78)
                    _require_range("tone_index", row["tone_index"], 0, 4)
                    _require_range("permutation", row["permutation"], 0, 23)
                    _require_range("i", row["i"], -2048, 2047)
                    _require_range("q", row["q"], -2048, 2047)
                    _require_range("quality", row["quality"], 0, 3)
                    _require_range("extension", row["extension"], 0, 2)
                    procedure["pbr"][side].append(row)
                elif record_type == "E" and len(fields) == 6:
                    _require_member("side", fields[3], {"L", "P"})
                    step_index = _int(fields[4])
                    _require_range("step_index", step_index, 0, 0xFFFF)
                    procedure["errors"].append(
                        {
                            "boot_index": boot_index,
                            "host_timestamp": host_timestamp,
                            "record_sequence": record_sequence,
                            "procedure_counter": procedure_counter,
                            "side": fields[3],
                            "step_index": step_index,
                            "reason": fields[5],
                        }
                    )
                elif record_type == "Z" and len(fields) == 3:
                    procedure["ended"] = True
                else:
                    raise ValueError(f"unsupported type or field count: {record_type}")
            except (IndexError, KeyError, ValueError) as exc:
                reason = "TRUNCATED_AT_CAPTURE_END" if not line_terminated else str(exc)
                _parse_error(
                    parse_errors,
                    procedures,
                    boot_index=boot_index,
                    line_number=line_number,
                    host_timestamp=host_timestamp,
                    schema=schema,
                    record_sequence=record_sequence,
                    procedure_counter=procedure_counter,
                    line=wire_line,
                    reason=reason,
                )

    if raw_record_count == 0:
        raise RawDataError(
            "No ILCS1/ILCS2 raw records found. Legacy A-B-A logs contain only final distance "
            "values and cannot be used for channel-level analysis."
        )
    return dict(procedures), parse_errors


def unwrap_phases(phases: Iterable[float]) -> list[float]:
    values = list(phases)
    if not values:
        return []
    result = [values[0]]
    offset = 0.0
    for previous, current in zip(values, values[1:]):
        difference = current - previous
        if difference > math.pi:
            offset -= 2.0 * math.pi
        elif difference < -math.pi:
            offset += 2.0 * math.pi
        result.append(current + offset)
    return result


def linear_regression(x_values: list[float], y_values: list[float]) -> tuple[float, float]:
    if len(x_values) != len(y_values) or len(x_values) < 2:
        raise ValueError("regression requires at least two paired samples")
    x_mean = sum(x_values) / len(x_values)
    y_mean = sum(y_values) / len(y_values)
    denominator = sum((x - x_mean) ** 2 for x in x_values)
    if denominator == 0:
        raise ValueError("regression frequencies have zero variance")
    slope = sum((x - x_mean) * (y - y_mean) for x, y in zip(x_values, y_values)) / denominator
    return y_mean - slope * x_mean, slope


def _join_reasons(reasons: Iterable[str]) -> str:
    return ";".join(sorted(set(reason for reason in reasons if reason)))


def _pbr_key(row: dict[str, Any]) -> tuple[int, int, int]:
    return row["step_index"], row["channel"], row["tone_index"]


def analyze_pbr(
    boot_index: int, procedure_counter: int, procedure: dict[str, Any]
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    local = {_pbr_key(row): row for row in procedure["pbr"]["L"]}
    peer = {_pbr_key(row): row for row in procedure["pbr"]["P"]}
    rows: list[dict[str, Any]] = []

    for key in sorted(set(local) | set(peer)):
        local_row = local.get(key)
        peer_row = peer.get(key)
        reasons: list[str] = []
        if local_row is None:
            reasons.append("MISSING_LOCAL")
        if peer_row is None:
            reasons.append("MISSING_PEER")
        if local_row and local_row["extension"] != 0:
            reasons.append("LOCAL_EXTENSION_SLOT")
        if peer_row and peer_row["extension"] != 0:
            reasons.append("PEER_EXTENSION_SLOT")
        if local_row and local_row["quality"] not in GOOD_TONE_QUALITIES:
            reasons.append("LOCAL_TONE_QUALITY")
        if peer_row and peer_row["quality"] not in GOOD_TONE_QUALITIES:
            reasons.append("PEER_TONE_QUALITY")

        wrapped = None
        if local_row and peer_row:
            combined_i = local_row["i"] * peer_row["i"] - local_row["q"] * peer_row["q"]
            combined_q = local_row["i"] * peer_row["q"] + local_row["q"] * peer_row["i"]
            if combined_i == 0 and combined_q == 0:
                reasons.append("ZERO_COMPLEX_PRODUCT")
            else:
                wrapped = math.atan2(combined_q, combined_i)

        rows.append(
            {
                "boot_index": boot_index,
                "procedure_counter": procedure_counter,
                "step_index": key[0],
                "channel": key[1],
                "tone_index": key[2],
                "frequency_mhz": 2402 + key[1],
                "local_record_sequence": local_row["record_sequence"] if local_row else None,
                "peer_record_sequence": peer_row["record_sequence"] if peer_row else None,
                "local_i": local_row["i"] if local_row else None,
                "local_q": local_row["q"] if local_row else None,
                "peer_i": peer_row["i"] if peer_row else None,
                "peer_q": peer_row["q"] if peer_row else None,
                "local_quality": local_row["quality"] if local_row else None,
                "peer_quality": peer_row["quality"] if peer_row else None,
                "local_extension": local_row["extension"] if local_row else None,
                "peer_extension": peer_row["extension"] if peer_row else None,
                "wrapped_phase_rad": wrapped,
                "unwrapped_phase_rad": None,
                "residual_rad": None,
                "valid": not reasons,
                "failure_reason": _join_reasons(reasons),
            }
        )

    valid_rows = sorted(
        (row for row in rows if row["valid"]), key=lambda row: row["frequency_mhz"]
    )
    distance = slope = residual_rms = None
    if len(valid_rows) >= 2:
        unwrapped = unwrap_phases(row["wrapped_phase_rad"] for row in valid_rows)
        frequencies = [float(row["frequency_mhz"]) for row in valid_rows]
        intercept, slope = linear_regression(frequencies, unwrapped)
        residuals = []
        for row, phase in zip(valid_rows, unwrapped):
            residual = phase - (intercept + slope * row["frequency_mhz"])
            row["unwrapped_phase_rad"] = phase
            row["residual_rad"] = residual
            residuals.append(residual)
        residual_rms = math.sqrt(sum(value * value for value in residuals) / len(residuals))
        distance = -slope * SPEED_OF_LIGHT_M_PER_S / (4.0 * math.pi) / 1_000_000.0

    return rows, {
        "pbr_pairs": len(rows),
        "pbr_valid": len(valid_rows),
        "pbr_missing_or_failed": len(rows) - len(valid_rows),
        "pbr_distance_m": distance,
        "pbr_slope_rad_per_mhz": slope,
        "pbr_residual_rms_rad": residual_rms,
    }


def analyze_rtt(
    boot_index: int, procedure_counter: int, procedure: dict[str, Any]
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    local = sorted(procedure["rtt"]["L"], key=lambda row: row["step_index"])
    peer = sorted(procedure["rtt"]["P"], key=lambda row: row["step_index"])
    rows: list[dict[str, Any]] = []
    nordic_mean = 0.0
    valid_tof: list[float] = []

    for pair_index in range(max(len(local), len(peer))):
        local_row = local[pair_index] if pair_index < len(local) else None
        peer_row = peer[pair_index] if pair_index < len(peer) else None
        reasons: list[str] = []
        if local_row is None:
            reasons.append("MISSING_LOCAL")
        if peer_row is None:
            reasons.append("MISSING_PEER")
        if local_row and peer_row and (
            local_row["step_index"], local_row["channel"]
        ) != (peer_row["step_index"], peer_row["channel"]):
            reasons.append("STEP_CHANNEL_MISMATCH")
        for side_name, row in (("LOCAL", local_row), ("PEER", peer_row)):
            if row and row["aa_quality"] != 0:
                reasons.append(f"{side_name}_AA_QUALITY")
            if row and row["rssi_dbm"] == RSSI_NOT_AVAILABLE:
                reasons.append(f"{side_name}_RSSI_UNAVAILABLE")
            if row and row["timing"] == TIME_NOT_AVAILABLE:
                reasons.append(f"{side_name}_TIMING_UNAVAILABLE")

        tof = None
        if local_row and peer_row and not reasons:
            tof = (local_row["timing"] - peer_row["timing"]) / 2.0
            valid_tof.append(tof)
            # Reproduce NCS v3.2.3 distance_estimation.c, including its use of
            # the all-sample index in the cumulative moving average.
            nordic_mean += (tof - nordic_mean) / (pair_index + 1)

        rows.append(
            {
                "boot_index": boot_index,
                "procedure_counter": procedure_counter,
                "pair_index": pair_index,
                "local_step_index": local_row["step_index"] if local_row else None,
                "peer_step_index": peer_row["step_index"] if peer_row else None,
                "local_channel": local_row["channel"] if local_row else None,
                "peer_channel": peer_row["channel"] if peer_row else None,
                "local_record_sequence": local_row["record_sequence"] if local_row else None,
                "peer_record_sequence": peer_row["record_sequence"] if peer_row else None,
                "local_aa_quality": local_row["aa_quality"] if local_row else None,
                "peer_aa_quality": peer_row["aa_quality"] if peer_row else None,
                "local_bit_errors": local_row["bit_errors"] if local_row else None,
                "peer_bit_errors": peer_row["bit_errors"] if peer_row else None,
                "local_nadm": local_row["nadm"] if local_row else None,
                "peer_nadm": peer_row["nadm"] if peer_row else None,
                "local_rssi_dbm": local_row["rssi_dbm"] if local_row else None,
                "peer_rssi_dbm": peer_row["rssi_dbm"] if peer_row else None,
                "toa_tod_initiator": local_row["timing"] if local_row else None,
                "tod_toa_reflector": peer_row["timing"] if peer_row else None,
                "tof_intermediate": tof,
                "valid": not reasons,
                "failure_reason": _join_reasons(reasons),
            }
        )

    nordic_distance = nordic_mean / 2.0 * SPEED_OF_LIGHT_NM_PER_S if valid_tof else None
    valid_mean_distance = (
        sum(valid_tof) / len(valid_tof) / 2.0 * SPEED_OF_LIGHT_NM_PER_S
        if valid_tof
        else None
    )
    return rows, {
        "rtt_pairs": len(rows),
        "rtt_valid": len(valid_tof),
        "rtt_missing_or_failed": len(rows) - len(valid_tof),
        "rtt_ncs_v3_2_3_distance_m": nordic_distance,
        "rtt_valid_mean_distance_m": valid_mean_distance,
    }


def analyze_procedures(procedures: dict[int, dict[str, Any]]) -> tuple[list[dict[str, Any]], list[dict[str, Any]], list[dict[str, Any]]]:
    pbr_rows: list[dict[str, Any]] = []
    rtt_rows: list[dict[str, Any]] = []
    summaries: list[dict[str, Any]] = []

    for procedure_key in sorted(procedures):
        procedure = procedures[procedure_key]
        boot_index = procedure["boot_index"]
        procedure_counter = procedure["procedure_counter"]
        procedure_pbr, pbr_summary = analyze_pbr(boot_index, procedure_counter, procedure)
        procedure_rtt, rtt_summary = analyze_rtt(boot_index, procedure_counter, procedure)
        pbr_rows.extend(procedure_pbr)
        rtt_rows.extend(procedure_rtt)
        local_header = procedure["headers"].get("L")
        peer_header = procedure["headers"].get("P")
        reasons = [error["reason"] for error in procedure["errors"]]
        reasons.extend(procedure["parse_errors"])
        if local_header is None:
            reasons.append("MISSING_LOCAL_HEADER")
        if peer_header is None:
            reasons.append("MISSING_PEER_HEADER")
        if local_header and not (local_header["transport_flags"] & PEER_FLAG_VALID):
            reasons.append("LOCAL_TRANSPORT_INVALID")
        if peer_header and not (peer_header["transport_flags"] & PEER_FLAG_VALID):
            reasons.append("PEER_TRANSPORT_INVALID")
        if local_header and peer_header and local_header["procedure_counter"] != peer_header["procedure_counter"]:
            reasons.append("PROCEDURE_COUNTER_MISMATCH")
        if not procedure["ended"]:
            reasons.append("MISSING_END_RECORD")

        summaries.append(
            {
                "boot_index": boot_index,
                "procedure_counter": procedure_counter,
                "local_header_present": local_header is not None,
                "peer_header_present": peer_header is not None,
                "ended": procedure["ended"],
                "failure_reason": _join_reasons(reasons),
                **pbr_summary,
                **rtt_summary,
            }
        )
    return pbr_rows, rtt_rows, summaries


def _write_csv(path: Path, rows: list[dict[str, Any]], fieldnames: list[str]) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def write_analysis(output_dir: Path, procedures: dict[int, dict[str, Any]], parse_errors: list[dict[str, Any]]) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    pbr_rows, rtt_rows, summaries = analyze_procedures(procedures)
    headers = [header for procedure in procedures.values() for header in procedure["headers"].values()]
    mode0_rows = [row for procedure in procedures.values() for side in ("L", "P") for row in procedure["mode0"][side]]
    errors = [error for procedure in procedures.values() for error in procedure["errors"]]

    _write_csv(output_dir / "procedures.csv", headers, [
        "boot_index", "host_timestamp", "record_sequence", "procedure_counter", "side", "config_id", "start_acl_conn_event",
        "frequency_compensation", "reference_power_level", "procedure_done_status",
        "subevent_done_status", "procedure_abort_reason", "subevent_abort_reason",
        "num_antenna_paths", "num_steps_reported", "abort_step", "step_data_len",
        "transport_flags",
    ])
    _write_csv(output_dir / "mode0_samples.csv", mode0_rows, [
        "boot_index", "host_timestamp", "record_sequence", "procedure_counter", "side", "step_index", "channel",
        "aa_quality", "bit_errors", "rssi_dbm", "antenna", "measured_freq_offset",
    ])
    _write_csv(output_dir / "pbr_samples.csv", pbr_rows, [
        "boot_index", "procedure_counter", "step_index", "channel", "tone_index", "frequency_mhz",
        "local_record_sequence", "peer_record_sequence",
        "local_i", "local_q", "peer_i", "peer_q", "local_quality", "peer_quality",
        "local_extension", "peer_extension", "wrapped_phase_rad", "unwrapped_phase_rad",
        "residual_rad", "valid", "failure_reason",
    ])
    _write_csv(output_dir / "rtt_samples.csv", rtt_rows, [
        "boot_index", "procedure_counter", "pair_index", "local_step_index", "peer_step_index",
        "local_channel", "peer_channel", "local_record_sequence", "peer_record_sequence",
        "local_aa_quality", "peer_aa_quality",
        "local_bit_errors", "peer_bit_errors", "local_nadm", "peer_nadm",
        "local_rssi_dbm", "peer_rssi_dbm", "toa_tod_initiator", "tod_toa_reflector",
        "tof_intermediate", "valid", "failure_reason",
    ])
    _write_csv(output_dir / "procedure_summary.csv", summaries, [
        "boot_index", "procedure_counter", "local_header_present", "peer_header_present", "ended",
        "failure_reason", "pbr_pairs", "pbr_valid", "pbr_missing_or_failed",
        "pbr_distance_m", "pbr_slope_rad_per_mhz", "pbr_residual_rms_rad", "rtt_pairs",
        "rtt_valid", "rtt_missing_or_failed", "rtt_ncs_v3_2_3_distance_m",
        "rtt_valid_mean_distance_m",
    ])
    _write_csv(output_dir / "firmware_errors.csv", errors, [
        "boot_index", "host_timestamp", "record_sequence", "procedure_counter", "side", "step_index", "reason",
    ])
    _write_csv(output_dir / "parse_errors.csv", parse_errors, [
        "boot_index", "line_number", "host_timestamp", "schema", "record_sequence",
        "procedure_counter", "line", "reason",
    ])


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path, help="timestamped Initiator UART log")
    parser.add_argument("--output-dir", type=Path, required=True, help="new analysis directory")
    args = parser.parse_args(argv)

    if args.output_dir.exists() and (
        not args.output_dir.is_dir() or any(args.output_dir.iterdir())
    ):
        parser.error(f"output path must be an empty directory: {args.output_dir}")
    try:
        procedures, parse_errors = parse_raw_log(args.log)
    except (OSError, RawDataError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1
    write_analysis(args.output_dir, procedures, parse_errors)
    print(f"Parsed {len(procedures)} procedures into {args.output_dir}")
    if parse_errors:
        print(f"WARNING: {len(parse_errors)} malformed raw records; see parse_errors.csv")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
