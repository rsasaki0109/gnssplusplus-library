#!/usr/bin/env python3
"""Focused raw-free tests for the Phase67 artifact-schema recovery."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase67_phase66_structural_integrity_recovery.py"
FREEZE_PATH = ROOT / "docs/use_cases/records/smartphone_r5_phase67_phase66_structural_integrity_recovery_freeze_v1.json"
MANIFEST_PATH = ROOT / "docs/use_cases/records/smartphone_r5_phase67_phase66_structural_integrity_recovery_manifest_v1.json"


def load_runner():
    spec = importlib.util.spec_from_file_location("phase67_structural_recovery", RUNNER_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load Phase67 runner")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Phase67ArtifactSchemaTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.runner = load_runner()

    def test_run_case_collision_shape_is_normalized(self):
        # This mirrors the actual P65.run_case() return: summary is parsed
        # native JSON, not artifact metadata. normalize_case puts it under
        # native_summary and obtains file metadata from the run directory.
        run_dir = ROOT / "output" / "phase67-test-fixture"
        run_dir.mkdir(parents=True, exist_ok=True)
        try:
            (run_dir / "submission.csv").write_text(
                "phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n"
                "2021-03-16-18-59-us-ca-mtv-a/pixel5,1,37.0,-122.0\n",
                encoding="utf-8",
            )
            (run_dir / "summary.json").write_text("{}\n", encoding="utf-8")
            raw_report = {
                "variant": "control",
                "run": 1,
                "candidate": False,
                "return_code": 0,
                "summary": {"schema_version": "native-summary"},
                "speed": {"over_70_mps_count": 0},
            }
            normalized = self.runner.normalize_case(
                raw_report, run_dir, "2021-03-16-18-59-us-ca-mtv-a/pixel5"
            )
            self.assertEqual(normalized["native_summary"]["schema_version"], "native-summary")
            self.assertEqual(normalized["summary_artifact"]["sha256"], self.runner.sha256(run_dir / "summary.json"))
            self.assertEqual(normalized["submission_artifact"]["rows"], 1)
            self.assertNotIn("sha256", normalized["native_summary"])
        finally:
            (run_dir / "submission.csv").unlink(missing_ok=True)
            (run_dir / "summary.json").unlink(missing_ok=True)
            run_dir.rmdir()

    def test_direct_control_artifacts_compare_to_nested_reference(self):
        actual = {
            "submission_artifact": {"bytes": 17, "sha256": "submission-digest"},
            "summary_artifact": {"bytes": 23, "sha256": "summary-digest"},
        }
        reference = {
            "submission": {"bytes": 17, "sha256": "submission-digest"},
            "summary": {"bytes": 23, "sha256": "summary-digest"},
        }
        self.runner.compare_control_artifacts(actual, reference, "fixture")
        mismatch = {**reference, "summary": {**reference["summary"], "sha256": "wrong"}}
        with self.assertRaises(self.runner.Phase67Error):
            self.runner.compare_control_artifacts(actual, mismatch, "fixture")

    def test_candidate_repeat_direct_artifacts_and_telemetry(self):
        first = {
            "submission_artifact": {"sha256": "s", "bytes": 1},
            "summary_artifact": {"sha256": "m", "bytes": 1},
            "base": {"finite_correction_fraction": 1.0},
        }
        second = json.loads(json.dumps(first))
        self.runner.compare_candidate_repeats(first, second, "fixture")
        second["summary_artifact"]["sha256"] = "different"
        with self.assertRaises(self.runner.Phase67Error):
            self.runner.compare_candidate_repeats(first, second, "fixture")

    def test_manifest_freeze_and_route_pins(self):
        freeze = json.loads(FREEZE_PATH.read_text(encoding="utf-8"))
        manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
        self.assertEqual(manifest["freeze"]["sha256"], self.runner.FREEZE_SHA256)
        self.assertEqual(manifest["evaluator"]["sha256"], "21f905e1fc876de0d998153dd23f12ca45db7c51ffb6778bf161c99fe1accc47")
        self.assertEqual(manifest["routes"], list(self.runner.ROUTES))
        self.assertEqual(freeze["structural_matrix"]["native_solver_invocations"], 12)
        self.assertEqual(freeze["structural_matrix"]["phase65_partial_output_reuse"], False)
        self.assertEqual(freeze["structural_matrix"]["phase66_partial_output_reuse"], False)
        self.assertEqual(manifest["read_accounting"]["truth_reads"], 0)
        self.assertIs(manifest["read_accounting"]["phase65_partial_output_reuse"], False)
        self.assertIs(manifest["read_accounting"]["phase66_partial_output_reuse"], False)
        for route in self.runner.ROUTES:
            expected = self.runner.P65.BASE_INPUT_HASHES[route]
            actual = manifest["base_inputs"][route]
            self.assertEqual(actual["sha256"], expected["sha256"])
            self.assertEqual(actual["bytes"], expected["bytes"])

    def test_no_prior_partial_output_path_is_matrix_input(self):
        source = RUNNER_PATH.read_text(encoding="utf-8")
        self.assertIn("direct_artifacts", source)
        self.assertIn("phase65_partial_output_reuse", source)
        self.assertIn("phase66_partial_output_reuse", source)
        self.assertNotIn("phase65-native-base-pseudorange-compensation-v1/candidate", source)
        self.assertNotIn("phase66-phase65-structural-integrity-recovery-v1", source)


if __name__ == "__main__":
    unittest.main()
