"""Focused tests for the Phase95 raw-path-only correction.

These tests inspect source, the path-availability freeze, and synthetic
command records. They do not launch the native solver and never open or hash
raw GNSS/IMU/navigation/truth/MAT/coordinate/base/Kaggle files.
"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
FREEZE = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase95_raw_input_path_availability_freeze_v1.json"
)
EVALUATOR_PATH = ROOT / (
    "apps/commands/benchmarks/"
    "gnss_smartphone_phase95_raw_input_path_corrected.py"
)
WRAPPER_PATH = ROOT / (
    "apps/commands/benchmarks/"
    "gnss_smartphone_phase95_raw_input_path_corrected_execute.py"
)

ROUTE = "2021-03-16-18-59-us-ca-mtv-a/pixel5"
RAW_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav")


def _load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Phase95RawPathCorrectedTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        cls.wrapper_source = WRAPPER_PATH.read_text(encoding="utf-8")
        cls.evaluator_source = EVALUATOR_PATH.read_text(encoding="utf-8")
        cls.wrapper = _load(WRAPPER_PATH, "phase95_path_wrapper_test")
        cls.evaluator = _load(EVALUATOR_PATH, "phase95_path_evaluator_test")

    def test_freeze_is_one_default_off_path_only_candidate(self) -> None:
        candidate = self.freeze["candidate"]
        self.assertTrue(candidate["exactly_one"])
        self.assertEqual(
            candidate["id"],
            "phase95_phase94_wrapper_inherit_phase91_raw_paths_and_materialize_exact_route_inputs",
        )
        self.assertEqual(candidate["status"], "frozen-not-implemented")
        self.assertTrue(candidate["default_off"])
        self.assertFalse(candidate["raw_execution_authorized"])
        self.assertFalse(candidate["native_algorithm_change"])
        self.assertFalse(candidate["solver_filter_lm_change"])

    def test_wrapper_uses_inherited_manifest_without_copy_or_transform(self) -> None:
        for term in (
            "PHASE91_MANIFEST",
            "load_inherited_raw_inputs",
            "path.stat().st_size",
            "materialize_command",
            "materialized[index] = pin[\"path\"]",
            "cwd=ROOT",
            "raw_content_copied_or_transformed",
        ):
            self.assertIn(term, self.wrapper_source)
        for forbidden in ("shutil.copy", "copyfile", "copy2", "transform_raw"):
            self.assertNotIn(forbidden, self.wrapper_source)

    def test_synthetic_materialization_changes_only_three_raw_arguments(self) -> None:
        command = [
            "build/apps/gnss_fgo_imu_no_base",
            "--dataset-id", ROUTE,
            "--android-gnss", f"raw/phase93/{ROUTE}/device_gnss.csv",
            "--android-imu", f"raw/phase93/{ROUTE}/device_imu.csv",
            "--nav", f"raw/phase93/{ROUTE}/brdc.nav",
            "--all-epochs",
            "--android-raw-utc-keys",
            "--android-raw-clock-only",
            "--android-utc-wall-clock-fallback",
            "--native-pdc-imu-tdcp-no-bridge",
            "--native-source-direct-observable-quality",
            "--native-source-clock-c0d-factor",
            "--native-source-clock-c0d-meter-state-parity",
            "--native-source-clock-c0d-active-solve-diagnostic",
            "--native-source-clock-c0d-gnss-first-meter-state-handoff",
            "--native-source-clock-c0d-phase94-stage-diagnostics",
            "--out", f"output/smartphone-r5/phase95-raw-input-path-corrected-v1/{ROUTE.replace('/', '__')}/withheld_solution_output.csv",
            "--summary-json", f"output/smartphone-r5/phase95-raw-input-path-corrected-v1/{ROUTE.replace('/', '__')}/summary.json",
        ]
        record = {
            "dataset_id": ROUTE,
            "raw_inputs": {
                name: {
                    "path": f"raw/phase93/{ROUTE}/{name}",
                    "sha256": None,
                    "read_at_manifest_creation": False,
                }
                for name in RAW_NAMES
            },
            "planned_output": {
                "summary": f"output/smartphone-r5/phase95-raw-input-path-corrected-v1/{ROUTE.replace('/', '__')}/summary.json",
                "withheld_solution_output": f"output/smartphone-r5/phase95-raw-input-path-corrected-v1/{ROUTE.replace('/', '__')}/withheld_solution_output.csv",
            },
            "command": command,
        }
        actual = {
            "device_gnss.csv": {"path": "output/smartphone-r5/phase25-raw-clock-eval-v1/raw/2021-03-16-18-59-us-ca-mtv-a/pixel5/device_gnss.csv"},
            "device_imu.csv": {"path": "output/smartphone-r5/phase25-raw-clock-eval-v1/raw/2021-03-16-18-59-us-ca-mtv-a/pixel5/device_imu.csv"},
            "brdc.nav": {"path": "output/smartphone-r5/phase25-raw-clock-eval-v1/raw/2021-03-16-18-59-us-ca-mtv-a/pixel5/brdc.nav"},
        }
        materialized = self.wrapper.materialize_command(record, actual, self.evaluator)
        for flag, name in (("--android-gnss", "device_gnss.csv"), ("--android-imu", "device_imu.csv"), ("--nav", "brdc.nav")):
            self.assertEqual(materialized[materialized.index(flag) + 1], actual[name]["path"])
        self.assertEqual(materialized[materialized.index("--dataset-id") + 1], ROUTE)
        self.assertEqual(materialized[materialized.index("--out") + 1], record["planned_output"]["withheld_solution_output"])
        self.assertEqual(materialized[materialized.index("--summary-json") + 1], record["planned_output"]["summary"])
        changed = {index for index, (before, after) in enumerate(zip(command, materialized)) if before != after}
        self.assertEqual(len(changed), 3)

    def test_pre_raw_evaluator_has_no_launch_or_raw_read_path(self) -> None:
        for forbidden in ("subprocess", "os.system", "Popen"):
            self.assertNotIn(forbidden, self.evaluator_source)
        self.assertIn("raw_execution_authorized", self.evaluator_source)
        self.assertIn("raw_reads", self.evaluator_source)
        self.assertFalse(self.freeze["execution_boundary"]["raw_execution_authorized"])


if __name__ == "__main__":
    unittest.main()
