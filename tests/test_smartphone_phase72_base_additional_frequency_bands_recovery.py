"""Focused tests for the Phase72 evaluator-integrity recovery."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase72_base_additional_frequency_bands_recovery.py"
SPEC = importlib.util.spec_from_file_location("phase72_base_recovery", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def control_summary(route: str) -> dict:
    return {
        "dataset_id": route,
        "truth_used": False,
        "production_default_changed": False,
        "native_quality_anchor": True,
        "native_pdc_imu_tdcp_no_bridge": True,
        "native_pdc_state_bridge": False,
        "graph": {"converged": True},
        "epochs": {"problem": 3, "output": 3, "pseudorange_factors": 10},
        "raw_utc_key_contract": {"unresolved_epochs": 0, "target_epochs": 2, "exact_solution_epochs": 2},
        "tdcp_contract": {"factors_built": 2, "factors_inserted": 2, "nonfinite_residuals": 0},
    }


class Phase72RecoveryTests(unittest.TestCase):
    def test_artifact_report_has_disjoint_metadata_and_payload(self) -> None:
        route = MODULE.ROUTES[0]
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            submission = root / "submission.csv"
            summary = root / "summary.json"
            submission.write_text(
                "phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n"
                f"{route},1,37.0,-122.0\n",
                encoding="utf-8",
            )
            summary.write_text(json.dumps(control_summary(route)), encoding="utf-8")
            report = MODULE.artifact_report(submission, summary, route, False)
            summary_sha = MODULE.sha256(summary)
        self.assertIn("summary_artifact", report)
        self.assertIn("summary_payload", report)
        self.assertNotIn("summary", report)
        self.assertEqual(report["summary_artifact"]["sha256"], summary_sha)
        self.assertEqual(report["summary_payload"]["dataset_id"], route)

    def test_run_case_return_shape_cannot_recreate_collision(self) -> None:
        route = MODULE.ROUTES[0]

        def fake_run(command, **_kwargs):
            out_path = Path(command[command.index("--out") + 1])
            summary_path = Path(command[command.index("--summary-json") + 1])
            if not out_path.is_absolute():
                out_path = ROOT / out_path
            if not summary_path.is_absolute():
                summary_path = ROOT / summary_path
            out_path.parent.mkdir(parents=True, exist_ok=True)
            summary_path.parent.mkdir(parents=True, exist_ok=True)
            out_path.write_text(
                "phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n"
                f"{route},1,37.0,-122.0\n",
                encoding="utf-8",
            )
            summary_path.write_text(json.dumps(control_summary(route)), encoding="utf-8")
            return SimpleNamespace(returncode=0, stdout="")

        with tempfile.TemporaryDirectory() as directory, mock.patch.object(MODULE.subprocess, "run", side_effect=fake_run):
            report = MODULE.run_case(Path(directory), route, "control", 1, False)
        self.assertIn("summary_artifact", report)
        self.assertIn("summary_payload", report)
        self.assertNotIn("summary", report)
        self.assertEqual(report["summary_payload"]["dataset_id"], route)

    def test_failure_path_catches_unexpected_exception(self) -> None:
        source = MODULE_PATH.read_text(encoding="utf-8")
        self.assertIn("except Exception as exc:", source)
        self.assertIn("structural_failure.json", source)

    def test_freeze_pins_new_root_and_unchanged_gates(self) -> None:
        freeze = json.loads((ROOT / "docs/use_cases/records/smartphone_r5_phase72_base_additional_frequency_bands_recovery_freeze_v1.json").read_text(encoding="utf-8"))
        self.assertEqual(freeze["evaluator_recovery_contract"]["summary_artifact_key"], "summary_artifact")
        self.assertEqual(freeze["evaluator_recovery_contract"]["summary_payload_key"], "summary_payload")
        self.assertFalse(freeze["authority"]["phase71_partial_output_reuse"])
        self.assertEqual(freeze["gates"]["prediction_domain_coverage_exact"], 1.0)
        self.assertEqual(freeze["gates"]["candidate_matched_factor_fraction_each_route_min"], 0.8)
        self.assertEqual(freeze["gates"]["candidate_finite_correction_fraction_among_matched_each_route_min"], 0.99)


if __name__ == "__main__":
    unittest.main()
