#!/usr/bin/env python3

from __future__ import annotations

import csv
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "apps" / "commands" / "benchmarks" / "gnss_smartphone_quality_report.py"
SPEC = importlib.util.spec_from_file_location("gnss_smartphone_quality_report", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
REPORT = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = REPORT
SPEC.loader.exec_module(REPORT)


DEVICE_FIELDS = (
    "MessageType",
    "utcTimeMillis",
    "HardwareClockDiscontinuityCount",
    "Svid",
    "AccumulatedDeltaRangeState",
    "SignalType",
    "ReceivedSvTimeUncertaintyNanos",
    "Cn0DbHz",
    "RawPseudorangeMeters",
    "RawPseudorangeUncertaintyMeters",
    "ArrivalTimeNanosSinceGpsEpoch",
)


def device_row(timestamp: int, cn0: float, raw_uncertainty: float, sv_uncertainty: int, adr: int) -> dict[str, str]:
    return {
        "MessageType": "Raw",
        "utcTimeMillis": str(timestamp),
        "HardwareClockDiscontinuityCount": "3",
        "Svid": "3",
        "AccumulatedDeltaRangeState": str(adr),
        "SignalType": "GPS_L1_CA",
        "ReceivedSvTimeUncertaintyNanos": str(sv_uncertainty),
        "Cn0DbHz": str(cn0),
        "RawPseudorangeMeters": "22000000",
        "RawPseudorangeUncertaintyMeters": str(raw_uncertainty),
        "ArrivalTimeNanosSinceGpsEpoch": "1378148416000000000",
    }


class SmartphoneQualityReportTests(unittest.TestCase):
    def test_fixed_buckets_join_errors_without_changing_the_input_contract(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_smartphone_quality_report_") as temp_dir:
            root = Path(temp_dir)
            device = root / "device_gnss.csv"
            with device.open("w", encoding="utf-8", newline="") as handle:
                writer = csv.DictWriter(handle, fieldnames=DEVICE_FIELDS)
                writer.writeheader()
                writer.writerow(device_row(1000, 30.0, 4.0, 10, 1))
                writer.writerow({**device_row(1000, 28.0, 5.0, 12, 1), "SignalType": "GAL_E1_C_P"})
                writer.writerow(device_row(2000, 40.0, 8.0, 40, 16))
                writer.writerow({**device_row(2000, 39.0, 9.0, 42, 16), "SignalType": "<unmapped>"})
                writer.writerow(device_row(4000, 46.0, 12.0, 100, 21))

            adapter_path = root / "adapter.json"
            adapter_path.write_text(
                json.dumps(
                    {
                        "schema_version": "smartphone-gnss-adapter.v1",
                        "inputs": {"device_gnss": {"sha256": hashlib.sha256(device.read_bytes()).hexdigest()}},
                        "observations": {
                            "explicitly_skipped_epochs": 0,
                            "epochs": 3,
                            "input_epochs": 3,
                            "first_utc_time_millis": 1000,
                            "last_utc_time_millis": 4000,
                        },
                    }
                ),
                encoding="utf-8",
            )
            matches_path = root / "matches.csv"
            with matches_path.open("w", encoding="utf-8", newline="") as handle:
                writer = csv.DictWriter(
                    handle,
                    fieldnames=[
                        "solution_unix_time_millis",
                        "truth_unix_time_millis",
                        "horizontal_error_m",
                        "vertical_error_m",
                    ],
                )
                writer.writeheader()
                writer.writerows(
                    [
                        {"solution_unix_time_millis": 1000, "truth_unix_time_millis": 1000, "horizontal_error_m": 1.0, "vertical_error_m": 2.0},
                        {"solution_unix_time_millis": 4000, "truth_unix_time_millis": 4000, "horizontal_error_m": 3.0, "vertical_error_m": -4.0},
                    ]
                )
            signoff_path = root / "signoff.json"
            signoff_path.write_text(
                json.dumps(
                    {
                        "schema_version": "smartphone-gnss-signoff.v1",
                        "decision": "pass",
                        "metrics": {
                            "selected_epochs": 3,
                            "solution_epochs": 2,
                            "truth_matched_epochs": 2,
                            "solution_availability_ratio": 2 / 3,
                            "truth_scored_coverage_ratio": 2 / 3,
                            "horizontal_median_m": 2.0,
                            "horizontal_p95_m": 2.9,
                            "vertical_p95_abs_m": 3.9,
                            "max_solution_gap_s": 3.0,
                        },
                    }
                ),
                encoding="utf-8",
            )

            report, segments, buckets = REPORT.build_report(
                device,
                adapter_path,
                signoff_path,
                matches_path,
                segment_seconds=1.5,
            )

            self.assertEqual(report["schema_version"], "smartphone-gnss-quality-report.v1")
            self.assertEqual(report["global_quality"]["selected_rows"], 5)
            self.assertEqual(report["global_quality"]["solver_rows"], 3)
            self.assertEqual(report["global_quality"]["unsupported_signal_rows"], 2)
            self.assertEqual(report["global_quality"]["matched_solution_epochs"], 2)
            self.assertEqual(len(segments), 2)
            self.assertIn("median_cn0_dbhz", {row["dimension"] for row in buckets})
            self.assertIn("adr_state", {row["dimension"] for row in buckets})
            self.assertTrue(report["analysis_contract"]["bucket_boundaries_are_fixed_before_error_join"])

    def test_rejects_device_hash_mismatch(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_smartphone_quality_report_") as temp_dir:
            root = Path(temp_dir)
            device = root / "device.csv"
            device.write_text("not-a-device\n", encoding="utf-8")
            adapter = root / "adapter.json"
            adapter.write_text(
                json.dumps(
                    {
                        "schema_version": "smartphone-gnss-adapter.v1",
                        "inputs": {"device_gnss": {"sha256": "0" * 64}},
                    }
                ),
                encoding="utf-8",
            )
            with self.assertRaisesRegex(ValueError, "device GNSS hash"):
                REPORT.build_report(device, adapter, root / "signoff.json", root / "matches.csv")


if __name__ == "__main__":
    unittest.main()
