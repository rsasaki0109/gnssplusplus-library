#!/usr/bin/env python3
"""Tests for the MADOCA release-matrix runner."""

from __future__ import annotations

import importlib.util
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


ROOT_DIR = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT_DIR / "scripts" / "ci" / "run_madoca_release_matrix.py"
SPEC = importlib.util.spec_from_file_location("run_madoca_release_matrix", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
matrix = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = matrix
SPEC.loader.exec_module(matrix)


class MadocaReleaseMatrixTest(unittest.TestCase):
    def test_hourly_paths_cover_requested_window_and_streams(self) -> None:
        paths = matrix.l6_paths(Path("/data"), 2, (200, 201), ionosphere=True)
        self.assertEqual(
            [path.name for path in paths],
            [
                "2025091A.200.l6",
                "2025091A.201.l6",
                "2025091B.200.l6",
                "2025091B.201.l6",
            ],
        )
        self.assertEqual(matrix.end_time(1), "2025/04/01 00:59:30")
        self.assertEqual(matrix.end_time(24), "2025/04/01 23:59:30")
        with self.assertRaises(ValueError):
            matrix.hourly_letters(25)

    def test_bridge_and_native_commands_select_profile_contracts(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            root = Path(temp_dir)
            output = root / "out.pos"
            summary = root / "summary.json"
            station = matrix.STATIONS["mizu"]
            bridge = matrix.bridge_command(
                Path("gnss_ppp"), root, station, "pppar-ion", 2, output, summary
            )
            native = matrix.native_command(
                Path("gnss_ppp"), root, station, "pppar-ion", 2, output, summary
            )

        self.assertEqual(bridge.count("--madocalib-l6"), 4)
        self.assertEqual(bridge.count("--madocalib-mdciono"), 4)
        self.assertIn("--madocalib-profile", bridge)
        self.assertEqual(native.count("--madoca-l6"), 4)
        self.assertEqual(native.count("--madoca-l6d"), 4)
        self.assertIn("--ar-method", native)
        self.assertIn("local-enu", native)

        ppp_native = matrix.native_command(
            Path("gnss_ppp"),
            Path("/data"),
            matrix.STATIONS["alic"],
            "ppp",
            1,
            Path("out.pos"),
            Path("summary.json"),
        )
        self.assertIn("--static", ppp_native)
        self.assertNotIn("--kinematic", ppp_native)
        self.assertNotIn("--enable-ar", ppp_native)
        self.assertNotIn("--madoca-l6d", ppp_native)

        pppar_native = matrix.native_command(
            Path("gnss_ppp"),
            Path("/data"),
            matrix.STATIONS["alic"],
            "pppar",
            1,
            Path("out.pos"),
            Path("summary.json"),
        )
        self.assertIn("--kinematic", pppar_native)
        self.assertNotIn("--static", pppar_native)

    def test_baseline_evaluation_checks_status_trajectory_and_runtime(self) -> None:
        measured = {
            "matched_epochs": 118,
            "base_only_epochs": 0,
            "candidate_only_epochs": 0,
            "base_status_counts": {"1": 108, "6": 10},
            "candidate_status_counts": {"5": 10, "6": 108},
            "status_pair_counts": {"1->6": 108, "6->5": 10},
            "native_row_counts": {
                "madoca_l6d_constraint_epochs": 1,
                "madoca_l6d_constraint_rows": 9,
                "madoca_l6d_constraint_position_gate_epochs": 117,
            },
            "wrong_fixes": 0,
            "missed_fixes": 0,
            "delta_rms_3d_m": 0.1,
            "delta_max_3d_m": 0.5,
            "boundary_max_delta_3d_m": 0.3,
            "native_runtime_s": 2.0,
            "bridge_runtime_s": 3.0,
        }
        baseline = dict(measured)
        baseline["matched_epochs"] = 100
        baseline["native_runtime_s"] = 4.0
        self.assertEqual(matrix.evaluate_baseline(measured, baseline), [])

        regressed = dict(measured)
        regressed["wrong_fixes"] = 1
        regressed["delta_max_3d_m"] = 0.6
        regressed["native_runtime_s"] = 4.1
        failures = matrix.evaluate_baseline(regressed, baseline)
        self.assertTrue(any("wrong_fixes" in failure for failure in failures))
        self.assertTrue(any("delta_max_3d_m" in failure for failure in failures))
        self.assertTrue(any("native_runtime_s" in failure for failure in failures))

    def test_measured_baseline_adds_declared_margins(self) -> None:
        case = {
            "matched_epochs": 118,
            "base_only_epochs": 0,
            "candidate_only_epochs": 0,
            "base_status_counts": {"1": 108, "6": 10},
            "candidate_status_counts": {"5": 10, "6": 108},
            "status_pair_counts": {"1->6": 108, "6->5": 10},
            "native_row_counts": {
                "madoca_l6d_constraint_epochs": 1,
                "madoca_l6d_constraint_rows": 9,
                "madoca_l6d_constraint_position_gate_epochs": 117,
            },
            "wrong_fixes": 0,
            "missed_fixes": 0,
            "delta_rms_3d_m": 0.1,
            "delta_max_3d_m": 0.5,
            "boundary_max_delta_3d_m": None,
            "native_runtime_s": 2.0,
            "bridge_runtime_s": 3.0,
        }
        baseline = matrix.measured_baseline(case)
        self.assertAlmostEqual(baseline["delta_rms_3d_m"], 0.11)
        self.assertAlmostEqual(baseline["delta_max_3d_m"], 0.525)
        self.assertIsNone(baseline["boundary_max_delta_3d_m"])
        self.assertEqual(baseline["native_runtime_s"], 4.0)
        self.assertNotIn("bridge_runtime_s", baseline)

    def test_github_summary_exposes_schemas_case_and_commands_artifact(self) -> None:
        payload = {
            "schema_version": matrix.SCHEMA_VERSION,
            "command_schema_version": matrix.COMMAND_SCHEMA_VERSION,
            "baseline_schema_version": matrix.BASELINE_SCHEMA_VERSION,
            "status": "passed",
            "cases": [
                {
                    "key": "mizu.pppar.1h",
                    "matched_epochs": 118,
                    "wrong_fixes": 0,
                    "missed_fixes": 0,
                    "delta_rms_3d_m": 0.1,
                    "delta_max_3d_m": 0.5,
                    "native_runtime_s": 2.0,
                    "artifact_dir": "output/matrix/mizu.pppar.1h",
                }
            ],
        }
        with tempfile.TemporaryDirectory() as temp_dir:
            summary_path = Path(temp_dir) / "step-summary.md"
            with mock.patch.dict(
                os.environ, {"GITHUB_STEP_SUMMARY": str(summary_path)}
            ):
                matrix.append_github_summary(payload)
            summary = summary_path.read_text(encoding="utf-8")

        self.assertIn(matrix.SCHEMA_VERSION, summary)
        self.assertIn(matrix.COMMAND_SCHEMA_VERSION, summary)
        self.assertIn(matrix.BASELINE_SCHEMA_VERSION, summary)
        self.assertIn("mizu.pppar.1h", summary)
        self.assertIn(
            "output/matrix/mizu.pppar.1h/commands.json",
            summary,
        )


if __name__ == "__main__":
    unittest.main()
