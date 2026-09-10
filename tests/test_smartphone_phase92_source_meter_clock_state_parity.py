"""Focused pre-raw tests for the sealed Phase92 Luna Max execution plan.

These tests inspect only source/docs/planned command strings.  They do not
open, hash, or execute any raw GNSS, IMU, navigation, truth, or coordinate
artifact.
"""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase92_source_meter_clock_state_parity.py"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase92_source_meter_clock_state_parity_execution_freeze_v1.json"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase92_source_meter_clock_state_parity_execution_manifest_v1.json"

_SPEC = importlib.util.spec_from_file_location("phase92_source_meter_pre_raw", RUNNER_PATH)
if _SPEC is None or _SPEC.loader is None:  # pragma: no cover
    raise RuntimeError(f"failed to load evaluator: {RUNNER_PATH}")
RUNNER = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(RUNNER)


def _manifest() -> dict:
    return json.loads(MANIFEST.read_text(encoding="utf-8"))


class Phase92PreRawTests(unittest.TestCase):
    def test_sealed_freeze_and_manifest_are_pre_raw(self) -> None:
        self.assertEqual(hashlib.sha256(FREEZE.read_bytes()).hexdigest(), RUNNER.EXECUTION_FREEZE_SHA256)
        freeze = RUNNER.verify_implementation_freeze()
        manifest = RUNNER.verify_manifest()
        self.assertEqual(freeze["status"], "frozen-before-phase92-raw-execution")
        self.assertEqual(manifest["status"], "sealed-before-phase92-raw-execution")
        self.assertFalse(freeze["decision_boundary"]["execution_authorized"])
        self.assertFalse(manifest["execution_authorization"]["raw_execution_authorized"])
        self.assertEqual(manifest["matrix"]["candidate_runs_total"], 4)
        self.assertEqual(manifest["matrix"]["control_runs_per_route"], 0)

    def test_exactly_four_inherited_routes_one_run_each(self) -> None:
        manifest = _manifest()
        routes = manifest["routes"]
        self.assertEqual([item["dataset_id"] for item in routes], list(RUNNER.ROUTES))
        self.assertEqual([item["runs"] for item in routes], [1, 1, 1, 1])
        self.assertEqual([item["domain_rows"] for item in routes], [RUNNER.DOMAIN_ROWS[r] for r in RUNNER.ROUTES])
        self.assertTrue(all(item["diagnostic_only"] for item in routes))

    def test_raw_input_set_is_gnss_imu_nav_only_and_unread(self) -> None:
        manifest = _manifest()
        for item in manifest["routes"]:
            raw_inputs = item["raw_inputs"]
            self.assertEqual(tuple(raw_inputs), RUNNER.RAW_INPUT_NAMES)
            self.assertEqual(set(raw_inputs), {"device_gnss.csv", "device_imu.csv", "brdc.nav"})
            for name, pin in raw_inputs.items():
                self.assertIsNone(pin["sha256"])
                self.assertFalse(pin["sha256_available"])
                self.assertFalse(pin["read_at_manifest_creation"])
                self.assertEqual(Path(pin["path"]).name, name)
                self.assertTrue(pin["path"].startswith(f"raw/phase92/{item['dataset_id']}/"))

    def test_commands_have_exact_candidate_dependencies_and_no_forbidden_lane(self) -> None:
        manifest = _manifest()
        for item in manifest["routes"]:
            command = item["command"]
            self.assertEqual(command[0], "build/apps/gnss_fgo_imu_no_base")
            for flag in (
                RUNNER.RAW_UTC_FLAG,
                *RUNNER.COMPATIBILITY_FLAGS,
                RUNNER.NO_BRIDGE_FLAG,
                RUNNER.DIRECT_FLAG,
                RUNNER.CLOCK_FLAG,
                RUNNER.ACTIVE_FLAG,
                RUNNER.RAW_DRIFT_FLAG,
                RUNNER.SELECTOR,
            ):
                self.assertEqual(command.count(flag), 1, flag)
            for flag in RUNNER.FORBIDDEN_FLAGS:
                self.assertNotIn(flag, command)
            self.assertNotIn("--native-gnss-first-velocity-only-handoff", command)
            self.assertNotIn("--native-direct-doppler-wls-handoff", command)
            self.assertEqual(command[command.index("--dataset-id") + 1], item["dataset_id"])
            self.assertEqual(command[command.index("--android-gnss") + 1], item["raw_inputs"]["device_gnss.csv"]["path"])
            self.assertEqual(command[command.index("--android-imu") + 1], item["raw_inputs"]["device_imu.csv"]["path"])
            self.assertEqual(command[command.index("--nav") + 1], item["raw_inputs"]["brdc.nav"]["path"])

    def test_meter_units_and_retained_key_propagation_are_pinned_in_source(self) -> None:
        app = (ROOT / "apps/native/gnss_fgo_imu_no_base.cpp").read_text(encoding="utf-8")
        backend = (ROOT / "src/algorithms/fgo_gtsam_backend.cpp").read_text(encoding="utf-8")
        internal = (ROOT / "src/algorithms/fgo_gtsam_internal.hpp").read_text(encoding="utf-8")
        initializer = (ROOT / "include/libgnss++/algorithms/source_clock_c0d_initializer.hpp").read_text(encoding="utf-8")
        builder = (ROOT / "src/algorithms/fgo_problems.cpp").read_text(encoding="utf-8")
        parser = (ROOT / "src/io/android_raw_gnss.cpp").read_text(encoding="utf-8")
        config = (ROOT / "include/libgnss++/algorithms/fgo_config.hpp").read_text(encoding="utf-8")
        self.assertIn(RUNNER.SELECTOR, app)
        self.assertIn("use_native_source_clock_c0d_meter_state_parity", config)
        self.assertIn("clockStateScale", internal)
        self.assertIn("clockStateTerm", internal)
        self.assertIn("publicClockSeconds", backend)
        self.assertIn("if (use_pose3 && !use_native_source_clock_c0d_meter_state)", backend)
        self.assertIn("graph.emplace_shared<gtsam::CarrierPhaseFactor>", backend)
        self.assertIn("validateRetainedRawDAlignment", initializer)
        self.assertIn("raw_source_index", parser)
        self.assertIn("raw_utc_time_millis", parser)
        self.assertIn("seed.raw_source_index = epoch.raw_source_index", builder)
        self.assertIn("seed.raw_utc_time_millis = epoch.raw_utc_time_millis", builder)
        self.assertIn("raw_drift_mps.reserve(android_gnss.observations.epochs.size())", app)
        self.assertIn("(C2-C1)-((D1+D2)*dt/2)", app)
        self.assertIn("[-1,+1,-dt/2,-dt/2]", app)

    def test_pre_raw_evaluator_contains_no_process_launch_or_raw_read_path(self) -> None:
        source = RUNNER_PATH.read_text(encoding="utf-8")
        self.assertNotIn("import subprocess", source)
        self.assertNotIn("subprocess.", source)
        self.assertNotIn("Popen(", source)
        self.assertNotIn("os.system", source)
        self.assertNotIn("read_raw", source)
        self.assertIsNone(_manifest()["pre_raw_evaluator"]["raw_execution_entrypoint"])

    def test_forbidden_artifacts_and_accuracy_are_absent_from_plan(self) -> None:
        manifest = _manifest()
        planned = json.dumps(
            {
                "routes": manifest["routes"],
                "matrix": manifest["matrix"],
            }
        ).lower()
        for term in ("ground_truth", ".mat", "kaggle", "validation_holdout", "phase82", "base.rinex"):
            self.assertNotIn(term, planned)
        self.assertFalse(manifest["candidate"]["accuracy_scoring"])
        self.assertFalse(manifest["candidate"]["route_score_selection"])
        self.assertFalse(manifest["execution_authorization"]["accuracy_or_submission_release"])
        self.assertFalse(manifest["execution_authorization"]["raw_execution_authorized"])
        accounting = manifest["read_accounting_at_manifest_creation"]
        self.assertEqual(accounting["raw_device_gnss_reads"], 0)
        self.assertEqual(accounting["raw_device_imu_reads"], 0)
        self.assertEqual(accounting["broadcast_navigation_reads"], 0)
        self.assertEqual(accounting["native_solver_invocations"], 0)

    def test_pre_raw_report_is_explicitly_zero_read(self) -> None:
        report = RUNNER.verify_pre_raw()
        self.assertEqual(report["status"], "pre-raw-verified")
        self.assertFalse(report["raw_execution_authorized"])
        self.assertEqual(report["raw_reads"], 0)
        self.assertEqual(report["native_solver_invocations"], 0)
        self.assertFalse(report["accuracy_scored"])


if __name__ == "__main__":
    unittest.main()
