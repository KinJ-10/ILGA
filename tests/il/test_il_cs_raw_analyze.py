#!/usr/bin/env python3

from __future__ import annotations

import math
import sys
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from il_cs_raw_analyze import (  # noqa: E402
    RawDataError,
    analyze_procedures,
    fnv1a32,
    parse_raw_log,
    unwrap_phases,
    write_analysis,
)


FIXTURE = Path(__file__).parent / "fixtures" / "il_cs_raw_synthetic.log"


def ilcs2_record(sequence: int, payload: str) -> str:
    body = f"ILCS2,{sequence},{payload}"
    return f"{body},{fnv1a32(body):08x}\n"


class IlCsRawAnalyzeTests(unittest.TestCase):
    def test_unwrap_crosses_negative_pi_boundary(self) -> None:
        actual = unwrap_phases([-3.0, 3.0, 2.7])
        expected = [-3.0, 3.0 - 2.0 * math.pi, 2.7 - 2.0 * math.pi]
        for value, wanted in zip(actual, expected):
            self.assertAlmostEqual(value, wanted, places=12)

    def test_fixture_parser_and_recalculation(self) -> None:
        procedures, parse_errors = parse_raw_log(FIXTURE)
        self.assertEqual(parse_errors, [])
        self.assertEqual(set(procedures), {7})
        self.assertTrue(procedures[7]["ended"])
        self.assertEqual(procedures[7]["headers"]["P"]["step_data_len"], 98)

        pbr_rows, rtt_rows, summaries = analyze_procedures(procedures)
        self.assertEqual(len(pbr_rows), 3)
        self.assertEqual(len(rtt_rows), 2)
        summary = summaries[0]
        self.assertEqual(summary["pbr_valid"], 3)
        self.assertLess(summary["pbr_slope_rad_per_mhz"], 0.0)
        self.assertGreater(summary["pbr_distance_m"], 0.0)
        self.assertLess(pbr_rows[1]["unwrapped_phase_rad"], pbr_rows[0]["unwrapped_phase_rad"])

        expected_rtt = 10.5 * 0.299792458
        self.assertAlmostEqual(summary["rtt_ncs_v3_2_3_distance_m"], expected_rtt, places=9)
        self.assertAlmostEqual(summary["rtt_valid_mean_distance_m"], expected_rtt, places=9)

    def test_writes_all_csv_outputs(self) -> None:
        procedures, parse_errors = parse_raw_log(FIXTURE)
        with tempfile.TemporaryDirectory() as temp_dir:
            output_dir = Path(temp_dir)
            write_analysis(output_dir, procedures, parse_errors)
            expected = {
                "procedures.csv",
                "mode0_samples.csv",
                "pbr_samples.csv",
                "rtt_samples.csv",
                "procedure_summary.csv",
                "firmware_errors.csv",
                "parse_errors.csv",
            }
            self.assertEqual({path.name for path in output_dir.iterdir()}, expected)

    def test_missing_peer_pbr_sample_keeps_reason(self) -> None:
        procedures, _ = parse_raw_log(FIXTURE)
        procedures[7]["pbr"]["P"].pop()
        pbr_rows, _, summaries = analyze_procedures(procedures)
        missing = [row for row in pbr_rows if row["failure_reason"] == "MISSING_PEER"]
        self.assertEqual(len(missing), 1)
        self.assertEqual(summaries[0]["pbr_missing_or_failed"], 1)

    def test_ilcs2_checksum_and_sequence_gap(self) -> None:
        capture = "".join(
            [
                ilcs2_record(10, "H,7,L,0,100,0,-40,0,0,0,0,1,1,255,12,1"),
                ilcs2_record(12, "H,7,P,0,100,49152,-42,0,0,0,0,1,1,255,10,1"),
                ilcs2_record(13, "Z,7"),
            ]
        )
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "ilcs2.log"
            path.write_text(capture)
            procedures, parse_errors = parse_raw_log(path)

        self.assertTrue(procedures[7]["ended"])
        self.assertEqual(procedures[7]["headers"]["L"]["record_sequence"], 10)
        self.assertEqual(len(parse_errors), 1)
        self.assertIn("RECORD_SEQUENCE_GAP", parse_errors[0]["reason"])

    def test_ilcs2_checksum_corruption_is_rejected(self) -> None:
        valid = ilcs2_record(20, "2,8,L,1,26,0,0,100,-50,0,0")
        corrupted = valid.replace(",100,-50,", ",101,-50,")
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "corrupt.log"
            path.write_text(corrupted)
            procedures, parse_errors = parse_raw_log(path)

        self.assertEqual(procedures[8]["pbr"]["L"], [])
        self.assertIn("CHECKSUM_MISMATCH", parse_errors[0]["reason"])

    def test_ilcs2_crlf_framing_is_accepted(self) -> None:
        capture = "".join(
            [
                ilcs2_record(0, "H,3,L,0,100,0,-40,0,0,0,0,1,1,255,12,1"),
                ilcs2_record(1, "Z,3"),
            ]
        ).replace("\n", "\r\n")
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "crlf.log"
            path.write_bytes(capture.encode("ascii"))
            procedures, parse_errors = parse_raw_log(path)

        self.assertEqual(parse_errors, [])
        self.assertTrue(procedures[3]["ended"])

    def test_ilcs2_sequence_zero_starts_new_boot_without_gap(self) -> None:
        capture = "".join(
            [
                ilcs2_record(100, "Z,12"),
                ilcs2_record(0, "H,0,L,0,100,0,-40,0,0,0,0,1,1,255,12,1"),
                ilcs2_record(1, "H,0,P,0,100,49152,-42,0,0,0,0,1,1,255,10,1"),
                ilcs2_record(2, "Z,0"),
            ]
        )
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "two_boots.log"
            path.write_text(capture)
            procedures, parse_errors = parse_raw_log(path)

        self.assertEqual(parse_errors, [])
        self.assertEqual(set(procedures), {12, 0x10000})
        self.assertEqual(procedures[12]["boot_index"], 0)
        self.assertEqual(procedures[0x10000]["boot_index"], 1)
        self.assertTrue(procedures[0x10000]["ended"])

    def test_partial_final_line_and_semantic_error_are_distinct(self) -> None:
        capture = (
            "ILCS1,2,9,L,1,26,0,0,100,-50,9,0\n"
            "ILCS1,2,9,L,2,27"
        )
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "partial.log"
            path.write_text(capture)
            _, parse_errors = parse_raw_log(path)

        self.assertIn("quality out of range", parse_errors[0]["reason"])
        self.assertEqual(parse_errors[1]["reason"], "TRUNCATED_AT_CAPTURE_END")

    def test_legacy_final_distance_log_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            legacy = Path(temp_dir) / "legacy.log"
            legacy.write_text("Estimated distance to reflector:\n- Phase-Based Ranging method: 2.3 meters\n")
            with self.assertRaisesRegex(RawDataError, "Legacy A-B-A logs"):
                parse_raw_log(legacy)


if __name__ == "__main__":
    unittest.main()
