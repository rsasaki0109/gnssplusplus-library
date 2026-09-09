"""Focused truth-free contract tests for the Phase85 C0/D matrix."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase85_source_clock_c0d_structural.py"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase85_source_clock_c0d_structural_manifest_v1.json"
EXPECTED_PHASE84_SHA256 = "c20d8849257cf10afdcea9e57d7299ed595c2c045a1b555241a40430e077cca4"

_SPEC = importlib.util.spec_from_file_location("phase85_source_clock_c0d_structural", RUNNER_PATH)
if _SPEC is None or _SPEC.loader is None:  # pragma: no cover
    raise RuntimeError(f"failed to load runner: {RUNNER_PATH}")
RUNNER = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(RUNNER)


class Phase85SourceClockC0DStructuralTests(unittest.TestCase):
    def test_freeze_chain_and_manifest_are_pinned_without_raw_matrix(self) -> None:
        freeze = RUNNER.verify_freeze()
        self.assertEqual(freeze["phase"], 84)
        self.assertEqual(hashlib.sha256(RUNNER.PHASE84_FREEZE.read_bytes()).hexdigest(), EXPECTED_PHASE84_SHA256)
        manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
        self.assertEqual(manifest["status"], "frozen-before-phase85-v2-raw-read")
        self.assertEqual(manifest["candidate"]["controls"], 0)
        self.assertEqual(manifest["matrix"]["native_invocations"], 8)
        self.assertEqual(manifest["matrix"]["truth_reads"], 0)
        self.assertEqual(manifest["matrix"]["precomputed_coordinate_reads"], 0)
        self.assertFalse(manifest["output_policy"]["accuracy_scoring"])

    def test_candidate_command_is_phase80_exact_plus_one_flag(self) -> None:
        route = RUNNER.ROUTES[0]
        run_dir = Path("output/smartphone-r5/phase85-source-clock-c0d-structural-v1") / route / "candidate" / "run1"
        command = RUNNER.native_command(route, run_dir)
        self.assertEqual(command[-1], RUNNER.CLOCK_FLAG)
        self.assertEqual(command.count(RUNNER.CLOCK_FLAG), 1)
        self.assertEqual(command.count(RUNNER.DIRECT_FLAG), 1)
        self.assertEqual(command.count(RUNNER.NO_BRIDGE_FLAG), 1)
        self.assertEqual(command.count("--native-signal-bias-states"), 1)
        self.assertNotIn(RUNNER.LEGACY_QUALITY_FLAG, command)
        self.assertNotIn(RUNNER.PRESERVE_BANDS_FLAG, command)
        self.assertEqual(command[:-1], RUNNER.P80.native_command(route, run_dir))

    def test_clock_telemetry_equation_units_sigma_and_edge_accounting(self) -> None:
        telemetry = {
            "clock_c0d_enabled": True,
            "clock_c0d_factor_count": 1,
            "clock_c0d_clock_jump_skips": 0,
            "clock_c0d_gap_skips": 1,
            "clock_c0d_invalid_dt_skips": 0,
            "clock_c0d_phone_exclusion_skips": 0,
            "clock_c0d_dt_min_s": 1.0,
            "clock_c0d_dt_max_s": 2.0,
            "clock_c0d_equation": "(c2-c1)-((d1+d2)*dt/(2*C_LIGHT))",
            "clock_c0d_jacobian_order": ["c1", "c2", "d1", "d2"],
            "clock_c0d_jacobian": "[-1,+1,-dt/(2*C_LIGHT),-dt/(2*C_LIGHT)]",
            "clock_c0d_units": {"clock": "seconds", "drift": "metres_per_second", "dt": "seconds", "residual": "seconds", "sigma": "seconds"},
            "speed_of_light_mps": 299792458.0,
            "clock_c0d_sigma_seconds": 0.1 / 299792458.0,
            "clock_jump_noise": "Inf (active C0 factor omitted)",
            "parity_scope": "C0/D active-row parity; not full seven-vector",
            "legacy_scalar_clock_between_factor_count": 0,
        }
        summary = {"epochs": {"output": 3}, "native_source_clock_c0d_factor": telemetry}
        self.assertEqual(RUNNER._validate_clock_telemetry(summary, RUNNER.ROUTES[0], 2)["factor_count"], 1)
        bad = json.loads(json.dumps(summary))
        bad["native_source_clock_c0d_factor"]["legacy_scalar_clock_between_factor_count"] = 1
        with self.assertRaises(RUNNER.Phase85StructuralError):
            RUNNER._validate_clock_telemetry(bad, RUNNER.ROUTES[0], 2)

    def test_nonempty_output_root_is_refused_before_any_native_case(self) -> None:
        with tempfile.TemporaryDirectory(prefix="phase85-structural-guard-") as directory:
            output = Path(directory) / "out"
            output.mkdir()
            (output / "sentinel").write_text("must remain", encoding="utf-8")
            with self.assertRaises(RUNNER.Phase85StructuralError):
                RUNNER.run_matrix(output)
            self.assertEqual((output / "sentinel").read_text(encoding="utf-8"), "must remain")

    def test_gnss_first_initializer_does_not_carry_c0d_into_non_pose3_path(self) -> None:
        source = (ROOT / "apps/native/gnss_fgo_imu_no_base.cpp").read_text(encoding="utf-8")
        self.assertIn("gnss_first_config.use_native_source_clock_c0d_factor = false;", source)
        self.assertIn("gnss_first_config.native_source_clock_c0d_phone.clear();", source)


if __name__ == "__main__":
    unittest.main()
