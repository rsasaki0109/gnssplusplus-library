"""Focused pre-raw tests for the sealed Phase93 Luna Max plan.

Only source text, sealed records, and planned command strings are inspected.
No raw GNSS, IMU, navigation, truth, MAT, or coordinate artifact is opened,
hashed, or executed by these tests.
"""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = ROOT / (
    "apps/commands/benchmarks/"
    "gnss_smartphone_phase93_source_staging_clock_state_handoff.py"
)
FREEZE = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase93_source_staging_clock_state_handoff_execution_freeze_v1.json"
)
MANIFEST = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase93_source_staging_clock_state_handoff_execution_manifest_v1.json"
)

_SPEC = importlib.util.spec_from_file_location("phase93_source_staging_pre_raw", RUNNER_PATH)
if _SPEC is None or _SPEC.loader is None:  # pragma: no cover
    raise RuntimeError(f"failed to load evaluator: {RUNNER_PATH}")
RUNNER = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(RUNNER)


def _manifest() -> dict:
    return json.loads(MANIFEST.read_text(encoding="utf-8"))


class Phase93PreRawTests(unittest.TestCase):
    def test_authoritative_freeze_and_manifest_are_pre_raw(self) -> None:
        self.assertEqual(
            hashlib.sha256(FREEZE.read_bytes()).hexdigest(),
            RUNNER.EXECUTION_FREEZE_SHA256,
        )
        freeze = RUNNER.verify_execution_freeze()
        manifest = RUNNER.verify_manifest()
        self.assertEqual(freeze["status"], "frozen-before-phase93-raw-execution")
        self.assertEqual(manifest["status"], "sealed-before-phase93-raw-execution")
        self.assertFalse(freeze["execution_boundary"]["raw_execution_authorized"])
        self.assertFalse(manifest["execution_authorization"]["raw_execution_authorized"])
        self.assertEqual(
            freeze["implementation"]["parent_phase93_freeze_commit"],
            RUNNER.PHASE93_SOURCE_FREEZE_COMMIT,
        )

    def test_exactly_four_inherited_routes_one_run_each(self) -> None:
        manifest = _manifest()
        routes = manifest["routes"]
        self.assertEqual([item["dataset_id"] for item in routes], list(RUNNER.ROUTES))
        self.assertEqual([item["runs"] for item in routes], [1, 1, 1, 1])
        self.assertEqual(
            [item["domain_rows"] for item in routes],
            [RUNNER.DOMAIN_ROWS[route] for route in RUNNER.ROUTES],
        )
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
                self.assertTrue(pin["path"].startswith(f"{RUNNER.RAW_ROOT}{item['dataset_id']}/"))

    def test_commands_are_exact_candidate_lane_and_no_legacy_raw_d(self) -> None:
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
                RUNNER.METER_FLAG,
                RUNNER.ACTIVE_FLAG,
                RUNNER.SELECTOR,
            ):
                self.assertEqual(command.count(flag), 1, flag)
            self.assertNotIn(RUNNER.RAW_DRIFT_FLAG, command)
            for flag in RUNNER.FORBIDDEN_FLAGS:
                self.assertNotIn(flag, command)
            self.assertEqual(command[command.index("--dataset-id") + 1], item["dataset_id"])
            self.assertEqual(
                command[command.index("--android-gnss") + 1],
                item["raw_inputs"]["device_gnss.csv"]["path"],
            )
            self.assertEqual(
                command[command.index("--android-imu") + 1],
                item["raw_inputs"]["device_imu.csv"]["path"],
            )
            self.assertEqual(
                command[command.index("--nav") + 1],
                item["raw_inputs"]["brdc.nav"]["path"],
            )

    def test_structural_gates_require_active_progress_and_full_handoff(self) -> None:
        manifest = _manifest()
        self.assertTrue(manifest["structural_gates"]["required_before_release"])
        for stage in ("gnss_first", "main"):
            gates = manifest["structural_gates"][stage]
            self.assertEqual(gates["accepted_outer_iterations_min_exclusive"], 0)
            self.assertTrue(gates["strict_final_cost_lt_initial_cost"])
            self.assertTrue(gates["finite_optimized_C_and_D_output"])
        handoff = manifest["structural_gates"]["handoff"]
        self.assertEqual(
            handoff["retained_key_fields"],
            ["raw_source_index", "raw_utc_time_millis", "GNSS_week_tow"],
        )
        self.assertTrue(handoff["one_to_one"])
        self.assertTrue(handoff["source_order_preserved"])
        self.assertTrue(handoff["reject_missing_duplicate_reordered_or_mismatched_keys"])
        self.assertTrue(handoff["reject_nearest_interpolated_padded_truncated_mapping"])
        self.assertTrue(handoff["reject_raw_zero_hold_wls_or_inferred_D"])
        self.assertTrue(manifest["structural_gates"]["conditioning_telemetry"]["required"])

    def test_meter_contract_and_optimized_d_are_pinned_in_source(self) -> None:
        RUNNER._require_source_terms()
        app = (ROOT / "apps/native/gnss_fgo_imu_no_base.cpp").read_text(encoding="utf-8")
        result = (ROOT / "include/libgnss++/algorithms/fgo.hpp").read_text(encoding="utf-8")
        config = (ROOT / "include/libgnss++/algorithms/fgo_config.hpp").read_text(encoding="utf-8")
        self.assertIn("epoch_clock_drift_mps", result)
        self.assertIn("= false;", config[config.index("use_native_source_clock_c0d_gnss_first_meter_state_handoff"):config.index("use_native_source_clock_c0d_gnss_first_meter_state_handoff") + 100])
        self.assertIn("(C2-C1)-((D1+D2)*dt/2)", app)
        self.assertIn("[-1,+1,-dt/2,-dt/2]", app)
        self.assertIn("SPEED_OF_LIGHT", app)

    def test_full_cpp_qualification_has_no_phase93_failure(self) -> None:
        evidence = _manifest()["qualification_evidence"]["full_cpp_suite"]
        self.assertEqual(evidence["registered_tests"], 1101)
        self.assertEqual(evidence["passed"], 1043)
        self.assertEqual(evidence["skipped"], 58)
        self.assertEqual(evidence["failed"], 0)
        self.assertEqual(evidence["phase93_related_tests_passed"], 2)
        self.assertEqual(evidence["phase93_related_tests_failed"], 0)
        self.assertFalse(_manifest()["qualification_evidence"]["skip_classification"]["phase93_caused"])

    def test_pre_raw_evaluator_has_no_process_launch_or_raw_read_path(self) -> None:
        source = RUNNER_PATH.read_text(encoding="utf-8")
        self.assertNotIn("import subprocess", source)
        self.assertNotIn("subprocess.", source)
        self.assertNotIn("Popen(", source)
        self.assertNotIn("os.system", source)
        self.assertNotIn("read_raw", source)
        self.assertIsNone(_manifest()["pre_raw_evaluator"]["raw_execution_entrypoint"])

    def test_forbidden_artifacts_accuracy_and_reads_are_absent(self) -> None:
        manifest = _manifest()
        planned = json.dumps({"routes": manifest["routes"], "matrix": manifest["matrix"]}).lower()
        for term in ("ground_truth", ".mat", "kaggle", "validation_holdout", "base.rinex"):
            self.assertNotIn(term, planned)
        self.assertFalse(manifest["candidate"]["accuracy_scoring"])
        self.assertFalse(manifest["candidate"]["route_score_selection"])
        self.assertFalse(manifest["execution_authorization"]["accuracy_or_submission_release"])
        accounting = manifest["read_accounting_at_manifest_creation"]
        for key in (
            "raw_device_gnss_reads",
            "raw_device_imu_reads",
            "broadcast_navigation_reads",
            "base_rinex_reads",
            "truth_reads",
            "mat_reads_or_generated",
            "precomputed_coordinate_reads",
            "native_solver_invocations",
        ):
            self.assertEqual(accounting[key], 0)

    def test_pre_raw_report_is_explicitly_zero_read(self) -> None:
        report = RUNNER.verify_pre_raw()
        self.assertEqual(report["status"], "pre-raw-verified")
        self.assertFalse(report["raw_execution_authorized"])
        self.assertEqual(report["raw_reads"], 0)
        self.assertEqual(report["native_solver_invocations"], 0)
        self.assertFalse(report["accuracy_scored"])


if __name__ == "__main__":
    unittest.main()
