"""Launch-free Phase136 authorized-wrapper argv/path boundary tests.

Only tracked source and sealed Phase135 metadata are read.  These tests never
materialize raw inputs, launch the native process, read solution rows or
truth, or access MAT/PDC/precomputed/Kaggle artifacts.
"""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
AUTHORIZED = ROOT / (
    "apps/commands/benchmarks/"
    "gnss_smartphone_phase135_official_affine_structural_authorized_execute.py"
)
NATIVE_APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"
RESULT = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase135_official_affine_structural_raw_result_v1.json"
)


def _load_authorized_wrapper():
    spec = importlib.util.spec_from_file_location(
        "phase136_authorized_wrapper", AUTHORIZED
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("unable to load authorized Phase135 wrapper")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


WRAPPER = _load_authorized_wrapper()
ROUTE = WRAPPER.STATIC.ROUTES[0]


def _safe_command() -> list[str]:
    paths = {
        "device_gnss.csv": Path("/synthetic/raw/device_gnss.csv"),
        "device_imu.csv": Path("/synthetic/raw/device_imu.csv"),
        "brdc.nav": Path("/synthetic/raw/brdc.nav"),
        "base.obs": Path("/synthetic/base/base.obs"),
    }
    command = WRAPPER.command_for(ROUTE, paths, Path("/synthetic/opaque"))
    command[
        command.index("--native-base-rinex-sha256") + 1
    ] = "a" * 64
    return command


class Phase136WrapperForbiddenTokenBoundaryTests(unittest.TestCase):
    def test_required_native_pdc_algorithm_option_is_accepted_once(self) -> None:
        command = _safe_command()
        WRAPPER.verify_command_raw_only(command, ROUTE)
        self.assertEqual(
            command.count("--native-pdc-imu-tdcp-no-bridge"), 1
        )
        self.assertIn(
            "--native-pdc-imu-tdcp-no-bridge", WRAPPER.STATIC.REQUIRED_RECIPE_FLAGS
        )

    def test_option_positions_and_counts_are_exact(self) -> None:
        command = _safe_command()
        template_options = [
            token for token in WRAPPER.STATIC.command_template(ROUTE)
            if token.startswith("--")
        ]
        actual_options = [token for token in command if token.startswith("--")]
        self.assertEqual(actual_options, template_options)
        WRAPPER.verify_command_raw_only(command, ROUTE)

        duplicate = _safe_command()
        duplicate[duplicate.index("--native-source-direct-observable-quality")] = (
            "--native-pdc-imu-tdcp-no-bridge"
        )
        with self.assertRaises(WRAPPER.AuthorizationError):
            WRAPPER.verify_command_raw_only(duplicate, ROUTE)

    def test_pdc_path_and_pdc_value_remain_forbidden(self) -> None:
        pdc_path = _safe_command()
        pdc_path[pdc_path.index("--android-gnss") + 1] = (
            "/synthetic/pdc/device_gnss.csv"
        )
        with self.assertRaises(WRAPPER.AuthorizationError):
            WRAPPER.verify_command_raw_only(pdc_path, ROUTE)

        pdc_value = _safe_command()
        pdc_value[pdc_value.index("--dataset-id") + 1] = "pdc-derived-route"
        with self.assertRaises(WRAPPER.AuthorizationError):
            WRAPPER.verify_command_raw_only(pdc_value, ROUTE)

        pdc_output_path = _safe_command()
        pdc_output_path[pdc_output_path.index("--out") + 1] = (
            "/synthetic/pdc/opaque_solution_output.csv"
        )
        with self.assertRaises(WRAPPER.AuthorizationError):
            WRAPPER.verify_command_raw_only(pdc_output_path, ROUTE)

    def test_unknown_selector_and_off_selector_fail_closed(self) -> None:
        unknown = _safe_command()
        unknown[unknown.index("--native-source-direct-observable-quality")] = (
            "--unknown-selector"
        )
        with self.assertRaises(WRAPPER.AuthorizationError):
            WRAPPER.verify_command_raw_only(unknown, ROUTE)

        forbidden = _safe_command()
        forbidden[forbidden.index("--native-source-direct-observable-quality")] = (
            WRAPPER.STATIC.OFF_SELECTORS[0]
        )
        with self.assertRaises(WRAPPER.AuthorizationError):
            WRAPPER.verify_command_raw_only(forbidden, ROUTE)

    def test_other_forbidden_values_are_not_option_exempt(self) -> None:
        for forbidden_value in (
            "/synthetic/truth/reference.csv",
            "/synthetic/ground_truth/reference.csv",
            "/synthetic/precomputed/seed.csv",
            "/synthetic/measurements.mat",
            "/synthetic/kaggle/input.csv",
            "/synthetic/token/input.csv",
        ):
            command = _safe_command()
            command[command.index("--android-imu") + 1] = forbidden_value
            with self.subTest(forbidden_value=forbidden_value):
                with self.assertRaises(WRAPPER.AuthorizationError):
                    WRAPPER.verify_command_raw_only(command, ROUTE)

    def test_native_cli_owns_the_required_option(self) -> None:
        source = NATIVE_APP.read_text(encoding="utf-8")
        option = "--native-pdc-imu-tdcp-no-bridge"
        self.assertIn(option, source)
        self.assertIn("options.native_pdc_imu_tdcp_no_bridge = true", source)
        self.assertIn("native_pdc_imu_tdcp_no_bridge", source)

    def test_previous_sealed_result_is_not_reclassified_or_rerun(self) -> None:
        sealed = json.loads(RESULT.read_text(encoding="utf-8"))
        self.assertEqual(sealed["status"], "sealed-fail-closed-wrapper-preflight")
        self.assertEqual(sealed["routes"][0]["native_solver_invocations"], 0)
        self.assertEqual(sealed["routes"][1]["native_solver_invocations"], 0)
        self.assertEqual(sealed["read_accounting"]["truth_reads"], 0)
        self.assertEqual(sealed["read_accounting"]["solution_coordinate_reads"], 0)
        self.assertFalse(sealed["policy"]["pdc_used"])
        self.assertIn("No rerun is authorized", sealed["next_action"])


if __name__ == "__main__":
    unittest.main()
