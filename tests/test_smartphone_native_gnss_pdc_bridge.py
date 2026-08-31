#!/usr/bin/env python3
"""Regression tests for the dedicated native raw Android PDC boundary.

The tests exercise argument and path authorization before any raw/nav bytes
are opened.  They intentionally do not materialize a result MAT, ground
truth, official sample, or test member.
"""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
APP_SOURCE = ROOT / "apps" / "native" / "gnss_pos_vel_pdc.cpp"
APP = ROOT / "build" / "apps" / "gnss_pos_vel_pdc"
FGO_SOURCE = ROOT / "apps" / "native" / "gnss_smartphone_fgo.cpp"
FGO_APP = ROOT / "build" / "apps" / "gnss_smartphone_fgo"
BENCHMARKS = ROOT / "apps" / "commands" / "benchmarks"
if str(BENCHMARKS) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS))
import gnss_smartphone_native_gnss_pdc as native_pdc  # noqa: E402


class NativeGnssPdcBridgeTests(unittest.TestCase):
    def test_source_declares_raw_native_spp_contract(self) -> None:
        source = APP_SOURCE.read_text(encoding="utf-8")
        for token in (
            "--android-raw",
            "--keyed-out-csv",
            "device_wls_seed_consumed",
            "native_spp_seed_epochs",
            "loadAndroidRawGnssCsv",
            "forbiddenMatPath",
            "--upstream-residual-snr",
            "exobs_residuals.m",
            "obserrmodel.m",
        ):
            self.assertIn(token, source)
        self.assertIn("optional WlsPosition* columns", source)
        self.assertIn("SPP implementation", source)
        self.assertIn("forbiddenNativeInputPath", source)

    def test_companion_native_fgo_entrypoint_has_mat_guard(self) -> None:
        source = FGO_SOURCE.read_text(encoding="utf-8")
        self.assertIn("lowered.find(\".mat\")", source)
        self.assertIn("forbiddenPath(options.output)", source)

    def test_orchestrator_is_raw_nav_only_and_rejects_mat_before_open(self) -> None:
        source = (BENCHMARKS / "gnss_smartphone_native_gnss_pdc.py").read_text(
            encoding="utf-8"
        )
        for token in (
            "--android-raw",
            "--nav",
            "truth-free-artifacts-sealed",
            "run_manifest.sha256",
            "python_coordinate_or_rinex_stage",
        ):
            self.assertIn(token, source)
        with tempfile.TemporaryDirectory(prefix="native_gnss_pdc_mat_guard_") as directory:
            root = Path(directory)
            poison = root / "result_gnss.mat"
            poison.write_bytes(b"must not be opened")
            with self.assertRaises(native_pdc.NativeGnssPdcError):
                native_pdc.preflight(
                    poison,
                    root / "brdc.nav",
                    APP,
                )

    @unittest.skipUnless(APP.is_file(), "Release gnss_pos_vel_pdc is not built")
    def test_raw_argument_contract_fails_closed_before_input_open(self) -> None:
        with tempfile.TemporaryDirectory(prefix="native_gnss_pdc_contract_") as directory:
            root = Path(directory)
            raw = root / "device_gnss.csv"
            nav = root / "brdc.nav"
            raw.write_text("poisoned raw fixture\n", encoding="ascii")
            nav.write_text("poisoned nav fixture\n", encoding="ascii")

            cases = (
                (
                    "forbidden raw result MAT",
                    root / "result_gnss.mat",
                    nav,
                    "--keyed-out-csv",
                    root / "out.csv",
                ),
                (
                    "forbidden nav sample",
                    raw,
                    root / "sample_submission.csv",
                    "--keyed-out-csv",
                    root / "out.csv",
                ),
            )
            for label, raw_path, nav_path, output_flag, output_path in cases:
                with self.subTest(label=label):
                    completed = subprocess.run(
                        [
                            str(APP),
                            "--android-raw",
                            str(raw_path),
                            "--nav",
                            str(nav_path),
                            output_flag,
                            str(output_path),
                            "--trip-id",
                            "fixture/phone",
                        ],
                        cwd=ROOT,
                        env={
                            **os.environ,
                            "LD_LIBRARY_PATH": ":".join(
                                value
                                for value in (
                                    "/home/sasaki/.local/lib",
                                    "/opt/ros/jazzy/lib",
                                    os.environ.get("LD_LIBRARY_PATH", ""),
                                )
                                if value
                            ),
                        },
                        check=False,
                        capture_output=True,
                        text=True,
                    )
                    self.assertEqual(completed.returncode, 2)
                    self.assertFalse(output_path.exists())

            missing_key = subprocess.run(
                [
                    str(APP),
                    "--android-raw",
                    str(raw),
                    "--nav",
                    str(nav),
                ],
                cwd=ROOT,
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(missing_key.returncode, 2)

            both_inputs = subprocess.run(
                [
                    str(APP),
                    "--obs",
                    str(raw),
                    "--android-raw",
                    str(raw),
                    "--nav",
                    str(nav),
                    "--keyed-out-csv",
                    str(root / "both.csv"),
                ],
                cwd=ROOT,
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(both_inputs.returncode, 2)

            seed_forbidden = subprocess.run(
                [
                    str(APP),
                    "--android-raw",
                    str(raw),
                    "--nav",
                    str(nav),
                    "--seed-pos",
                    str(root / "seed.pos"),
                    "--keyed-out-csv",
                    str(root / "seed.csv"),
                ],
                cwd=ROOT,
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(seed_forbidden.returncode, 2)

            mat_output = subprocess.run(
                [
                    str(APP),
                    "--android-raw",
                    str(raw),
                    "--nav",
                    str(nav),
                    "--keyed-out-csv",
                    str(root / "native_output.mat"),
                ],
                cwd=ROOT,
                check=False,
                capture_output=True,
                text=True,
            )
            self.assertEqual(mat_output.returncode, 2)

    @unittest.skipUnless(FGO_APP.is_file(), "Release gnss_smartphone_fgo is not built")
    def test_native_fgo_rejects_mat_inputs_and_outputs_before_open(self) -> None:
        with tempfile.TemporaryDirectory(prefix="native_fgo_mat_contract_") as directory:
            root = Path(directory)
            valid_gnss = root / "device_gnss.csv"
            valid_imu = root / "device_imu.csv"
            valid_nav = root / "brdc.nav"
            valid_gnss.write_text("poisoned raw fixture\n", encoding="ascii")
            valid_imu.write_text("poisoned imu fixture\n", encoding="ascii")
            valid_nav.write_text("poisoned nav fixture\n", encoding="ascii")

            for name, gnss, imu, nav, output, summary in (
                (
                    "MAT GNSS",
                    root / "phone_data.mat",
                    valid_imu,
                    valid_nav,
                    root / "out.csv",
                    root / "summary.json",
                ),
                (
                    "MAT IMU",
                    valid_gnss,
                    root / "device_imu.mat",
                    valid_nav,
                    root / "out.csv",
                    root / "summary.json",
                ),
                (
                    "MAT navigation",
                    valid_gnss,
                    valid_imu,
                    root / "brdc.mat",
                    root / "out.csv",
                    root / "summary.json",
                ),
                (
                    "MAT output",
                    valid_gnss,
                    valid_imu,
                    valid_nav,
                    root / "out.mat",
                    root / "summary.json",
                ),
                (
                    "MAT summary",
                    valid_gnss,
                    valid_imu,
                    valid_nav,
                    root / "out.csv",
                    root / "summary.mat",
                ),
            ):
                with self.subTest(name=name):
                    completed = subprocess.run(
                        [
                            str(FGO_APP),
                            "--device-gnss",
                            str(gnss),
                            "--device-imu",
                            str(imu),
                            "--nav",
                            str(nav),
                            "--out",
                            str(output),
                            "--summary-json",
                            str(summary),
                        ],
                        cwd=ROOT,
                        env={
                            **os.environ,
                            "LD_LIBRARY_PATH": ":".join(
                                value
                                for value in (
                                    "/home/sasaki/.local/lib",
                                    "/opt/ros/jazzy/lib",
                                    os.environ.get("LD_LIBRARY_PATH", ""),
                                )
                                if value
                            ),
                        },
                        check=False,
                        capture_output=True,
                        text=True,
                    )
                    self.assertEqual(completed.returncode, 2)
                    self.assertFalse(output.exists())
                    self.assertFalse(summary.exists())


if __name__ == "__main__":
    unittest.main()
