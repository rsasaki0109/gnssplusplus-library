from __future__ import annotations

import csv
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase31_quality_anchor_structural.py"
SPEC = importlib.util.spec_from_file_location("phase31_quality_anchor_structural", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class Phase31QualityAnchorStructuralTests(unittest.TestCase):
    def test_raw_epoch_keys_collapse_satellite_rows_and_exclude_warmup(self) -> None:
        with tempfile.TemporaryDirectory(prefix="phase31-raw-") as temporary:
            path = Path(temporary) / "device_gnss.csv"
            with path.open("w", encoding="utf-8", newline="") as handle:
                writer = csv.writer(handle, lineterminator="\n")
                writer.writerow(["utcTimeMillis", "Svid", "PseudorangeMeters"])
                writer.writerows([[1000, 1, 2], [1000, 2, 3], [2000, 1, 2], [3000, 1, 2]])
            self.assertEqual(MODULE.read_raw_epoch_keys(path), [1000, 2000, 3000])

    def test_raw_epoch_keys_reject_time_reversal(self) -> None:
        with tempfile.TemporaryDirectory(prefix="phase31-raw-reversal-") as temporary:
            path = Path(temporary) / "device_gnss.csv"
            path.write_text(
                "utcTimeMillis\n1000\n2000\n1500\n",
                encoding="utf-8",
            )
            with self.assertRaises(MODULE.Phase31Error):
                MODULE.read_raw_epoch_keys(path)

    def test_speed_report_keeps_physical_bound_explicit(self) -> None:
        rows = [(1000, 37.0, -122.0), (2000, 37.0001, -122.0)]
        report = MODULE.speed_report(rows)
        self.assertTrue(report["finite"])
        self.assertEqual(report["transition_count"], 1)
        self.assertEqual(report["over_70_mps_count"], 0)

        spike = MODULE.speed_report([(1000, 37.0, -122.0), (2000, 37.01, -122.0)])
        self.assertGreater(spike["over_70_mps_count"], 0)

    def test_summary_contract_requires_anchor_and_unchanged_graph(self) -> None:
        summary = {
            "dataset_id": "route/phone",
            "truth_used": False,
            "production_default_changed": False,
            "native_quality_anchor": True,
            "native_pdc_state_bridge": False,
            "graph": {
                "converged": True,
                "initial_cost": 10.0,
                "final_cost": 1.0,
                "factors": 12,
                "values": 3,
                "imu_intervals": 2,
                "iterations": 3,
            },
            "epochs": {
                "problem": 3,
                "output": 3,
                "pseudorange_factors": 4,
                "tdcp_factors_built": 2,
            },
            "quality_anchor_initialization": {
                "enabled": True,
                "selected": True,
                "truth_free": True,
                "graph_model_changed": False,
                "fallback_epochs": 0,
                "eligible_candidates": 3,
            },
            "raw_utc_key_contract": {"unresolved_epochs": 0, "interpolated_epochs": 0},
            "imu_initialization": {},
        }
        projection = MODULE.validate_summary(summary, "route/phone")
        self.assertEqual(projection["quality_anchor"]["selected"], True)
        summary["quality_anchor_initialization"]["graph_model_changed"] = True
        with self.assertRaises(MODULE.Phase31Error):
            MODULE.validate_summary(summary, "route/phone")


if __name__ == "__main__":
    unittest.main()
