"""Regression tests for gnss_fuse's concise help and TOML config surface."""

from __future__ import annotations

import os
import subprocess
import tempfile
import unittest
from pathlib import Path


class GnssFuseCliTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        binary_dir = Path(os.environ["GNSSPP_BINARY_DIR"])
        executable_name = "gnss_fuse.exe" if os.name == "nt" else "gnss_fuse"
        candidates = [
            binary_dir / "apps" / executable_name,
            *(binary_dir / "apps" / config / executable_name
              for config in ("Release", "RelWithDebInfo", "Debug")),
        ]
        cls.executable = next(
            (candidate for candidate in candidates if candidate.is_file()),
            None,
        )
        if cls.executable is None:
            rendered = ", ".join(str(candidate) for candidate in candidates)
            raise RuntimeError(f"gnss_fuse executable not found; checked: {rendered}")

    @classmethod
    def run_fuse(cls, *options: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [str(cls.executable), *options],
            check=False,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
        )

    def test_default_help_keeps_the_supported_path_concise(self) -> None:
        result = self.run_fuse("--help")

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("--data-dir <dir>", result.stdout)
        self.assertIn("--config <path>", result.stdout)
        self.assertIn("--navi776-tc", result.stdout)
        self.assertIn("--help-advanced", result.stdout)
        self.assertNotIn("--tc-doppler-sigma", result.stdout)
        self.assertNotIn("--rtk-ins-prior-inflation", result.stdout)
        self.assertLess(len(result.stdout), 5000)

    def test_config_supplies_defaults_and_cli_always_overrides_them(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_fuse_config_") as temp_dir:
            config_path = Path(temp_dir) / "fuse.toml"
            config_path.write_text(
                "\n".join(
                    [
                        "[gnss_fuse]",
                        'gnss_pos = "missing-solution.pos"',
                        'imu = "missing-imu.csv"',
                        "lever_arm = [0.31, 0.0, 0.55]",
                        "navi776_tc = false",
                        "zupt_gnss_speed_gate = false",
                        "base_interp = true",
                        "max_epochs = -1",
                    ]
                ),
                encoding="utf-8",
            )

            result = self.run_fuse(
                "--max-epochs",
                "0",
                "--config",
                str(config_path),
            )

        self.assertEqual(result.returncode, 1)
        self.assertIn("failed to load IMU CSV", result.stderr)
        self.assertNotIn("--max-epochs must be non-negative", result.stderr)

    def test_unknown_config_key_is_rejected_by_the_existing_option_parser(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_fuse_config_") as temp_dir:
            config_path = Path(temp_dir) / "fuse.toml"
            config_path.write_text(
                "[gnss_fuse]\nthis_option_does_not_exist = 1\n",
                encoding="utf-8",
            )

            result = self.run_fuse("--config", str(config_path))

        self.assertEqual(result.returncode, 1)
        self.assertIn("unknown or incomplete argument: --this-option-does-not-exist",
                      result.stderr)

    def test_advanced_help_retains_research_option_discoverability(self) -> None:
        result = self.run_fuse("--help-advanced")

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("--navi776-tc", result.stdout)
        self.assertIn("--zupt-gnss-speed-gate", result.stdout)
        self.assertIn("--tc-doppler-sigma", result.stdout)
        self.assertIn("--rtk-ins-prior-inflation", result.stdout)
        self.assertIn("--help-advanced", result.stdout)

    def test_zupt_speed_threshold_rejects_non_finite_or_negative_values(self) -> None:
        for invalid in ("-1", "nan"):
            result = self.run_fuse(
                "--imu", "missing-imu.csv",
                "--gnss-pos", "missing-solution.pos",
                "--zupt-gnss-speed-threshold-mps", invalid,
            )
            self.assertEqual(result.returncode, 1)
            self.assertIn(
                "--zupt-gnss-speed-threshold-mps must be finite and non-negative",
                result.stderr,
            )

    def test_velocity_reanchor_options_reject_invalid_values(self) -> None:
        result = self.run_fuse(
            "--imu", "missing-imu.csv",
            "--gnss-pos", "missing-solution.pos",
            "--max-consecutive-velocity-gate-rejections", "-1",
        )
        self.assertEqual(result.returncode, 1)
        self.assertIn(
            "--max-consecutive-velocity-gate-rejections must be non-negative",
            result.stderr,
        )

        for invalid in ("-1", "nan"):
            result = self.run_fuse(
                "--imu", "missing-imu.csv",
                "--gnss-pos", "missing-solution.pos",
                "--max-gnss-velocity-reanchor-mps", invalid,
            )
            self.assertEqual(result.returncode, 1)
            self.assertIn(
                "--max-gnss-velocity-reanchor-mps must be finite and non-negative",
                result.stderr,
            )

    def test_rtk_hold_and_fix_satellite_options_mirror_gnss_solve(self) -> None:
        result = self.run_fuse("--help-advanced")
        self.assertEqual(result.returncode, 0)
        self.assertIn("--hold-kalman-update", result.stdout)
        self.assertIn("--min-fix-satellites <k>", result.stdout)

        result = self.run_fuse(
            "--imu", "missing-imu.csv",
            "--gnss-pos", "missing-solution.pos",
            "--min-fix-satellites", "-1",
        )
        self.assertEqual(result.returncode, 1)
        self.assertIn("--min-fix-satellites must be >= 0", result.stderr)

        result = self.run_fuse(
            "--max-epochs", "0",
            "--imu", "missing-imu.csv",
            "--gnss-pos", "missing-solution.pos",
            "--hold-kalman-update",
            "--hold-kalman-update-validated-only",
            "--min-fix-satellites", "4",
        )
        self.assertEqual(result.returncode, 1)
        self.assertIn("failed to load IMU CSV", result.stderr)

    def test_validated_only_kalman_hold_requires_the_kalman_hold(self) -> None:
        result = self.run_fuse("--help-advanced")
        self.assertIn("--hold-kalman-update-validated-only", result.stdout)

        result = self.run_fuse(
            "--imu", "missing-imu.csv",
            "--gnss-pos", "missing-solution.pos",
            "--hold-kalman-update-validated-only",
        )
        self.assertEqual(result.returncode, 1)
        self.assertIn(
            "--hold-kalman-update-validated-only requires --hold-kalman-update",
            result.stderr,
        )

    def test_consider_before_heading_latch_is_an_advanced_opt_in(self) -> None:
        result = self.run_fuse("--help")
        self.assertNotIn("--consider-before-heading-latch", result.stdout)
        result = self.run_fuse("--help-advanced")
        self.assertIn("--consider-before-heading-latch", result.stdout)

        # Parsed as a plain flag, so the run reaches the input loader.
        result = self.run_fuse(
            "--imu", "missing-imu.csv",
            "--gnss-pos", "missing-solution.pos",
            "--consider-before-heading-latch",
        )
        self.assertEqual(result.returncode, 1)
        self.assertIn("failed to load IMU CSV", result.stderr)

        # A TOML true maps to the flag; false is rejected like other default-off flags.
        with tempfile.TemporaryDirectory(prefix="gnss_fuse_config_") as temp_dir:
            config_path = Path(temp_dir) / "fuse.toml"
            config_path.write_text(
                "[gnss_fuse]\n"
                'gnss_pos = "missing-solution.pos"\n'
                'imu = "missing-imu.csv"\n'
                "consider_before_heading_latch = true\n",
                encoding="utf-8",
            )
            result = self.run_fuse("--config", str(config_path))
            self.assertEqual(result.returncode, 1)
            self.assertIn("failed to load IMU CSV", result.stderr)

            config_path.write_text(
                "[gnss_fuse]\nconsider_before_heading_latch = false\n",
                encoding="utf-8",
            )
            result = self.run_fuse("--config", str(config_path))
            self.assertEqual(result.returncode, 1)
            self.assertIn("boolean false is not supported", result.stderr)

    def test_independent_output_filter_comes_with_navi776_tc_and_has_an_opt_out(self) -> None:
        result = self.run_fuse("--help")
        self.assertNotIn("--independent-output-filter", result.stdout)
        result = self.run_fuse("--help-advanced")
        self.assertIn("--independent-output-filter", result.stdout)
        self.assertIn("--no-independent-output-filter", result.stdout)
        self.assertIn("and --independent-output-filter with --base", result.stdout)

        # The explicit flag still needs the epoch SPP velocity of the RTK path.
        result = self.run_fuse(
            "--imu", "missing-imu.csv",
            "--gnss-pos", "missing-solution.pos",
            "--independent-output-filter",
        )
        self.assertEqual(result.returncode, 1)
        self.assertIn("--independent-output-filter requires --base", result.stderr)

        rtk_inputs = ("--imu", "missing-imu.csv", "--rover", "missing-rover.obs",
                      "--base", "missing-base.obs", "--nav", "missing.nav")
        result = self.run_fuse(*rtk_inputs, "--independent-output-filter",
                               "--library-fix-integrity-gate")
        self.assertEqual(result.returncode, 1)
        self.assertIn(
            "--independent-output-filter cannot be combined with --library-fix-integrity-gate",
            result.stderr,
        )

        # The preset only asks for it: without --base, or with the integrity
        # gate, it stays off instead of failing, so those runs still reach the
        # input loader.
        for arguments in (
            ("--imu", "missing-imu.csv", "--gnss-pos", "missing-solution.pos", "--navi776-tc"),
            (*rtk_inputs, "--navi776-tc", "--library-fix-integrity-gate"),
            (*rtk_inputs, "--navi776-tc"),
            (*rtk_inputs, "--no-independent-output-filter", "--navi776-tc"),
            (*rtk_inputs, "--independent-output-filter", "--no-independent-output-filter",
             "--library-fix-integrity-gate"),
        ):
            result = self.run_fuse(*arguments)
            self.assertEqual(result.returncode, 1, msg=arguments)
            self.assertIn("failed to load", result.stderr, msg=arguments)
            self.assertNotIn("independent-output-filter", result.stderr, msg=arguments)

        with tempfile.TemporaryDirectory(prefix="gnss_fuse_config_") as temp_dir:
            config_path = Path(temp_dir) / "fuse.toml"
            for value in ("true", "false"):
                config_path.write_text(
                    "[gnss_fuse]\n"
                    'imu = "missing-imu.csv"\n'
                    'rover = "missing-rover.obs"\n'
                    'base = "missing-base.obs"\n'
                    'nav = "missing.nav"\n'
                    "navi776_tc = true\n"
                    f"independent_output_filter = {value}\n",
                    encoding="utf-8",
                )
                result = self.run_fuse("--config", str(config_path))
                self.assertEqual(result.returncode, 1, msg=value)
                self.assertNotIn("unknown or incomplete argument", result.stderr, msg=value)
                self.assertNotIn("boolean false is not supported", result.stderr, msg=value)
                self.assertNotIn("requires --base", result.stderr, msg=value)

            # A config false is overridden by the command line, as for every option.
            config_path.write_text(
                "[gnss_fuse]\n"
                'imu = "missing-imu.csv"\n'
                'gnss_pos = "missing-solution.pos"\n'
                "independent_output_filter = false\n",
                encoding="utf-8",
            )
            result = self.run_fuse("--config", str(config_path), "--independent-output-filter")
            self.assertEqual(result.returncode, 1)
            self.assertIn("--independent-output-filter requires --base", result.stderr)

    def test_heading_latch_direction_regression_is_an_advanced_opt_in(self) -> None:
        result = self.run_fuse("--help")
        self.assertNotIn("--heading-latch-direction-regression", result.stdout)
        result = self.run_fuse("--help-advanced")
        self.assertIn("--heading-latch-direction-regression", result.stdout)

        # A plain flag in every mode: the run reaches the input loader.
        for extra in ((), ("--navi776-tc",), ("--independent-output-filter",)):
            result = self.run_fuse(
                "--imu", "missing-imu.csv",
                "--rover", "missing-rover.obs", "--base", "missing-base.obs", "--nav", "missing.nav",
                "--heading-latch-direction-regression", *extra,
            )
            self.assertEqual(result.returncode, 1, msg=extra)
            self.assertIn("failed to load IMU CSV", result.stderr, msg=extra)
        result = self.run_fuse(
            "--imu", "missing-imu.csv", "--gnss-pos", "missing-solution.pos",
            "--heading-latch-direction-regression",
        )
        self.assertEqual(result.returncode, 1)
        self.assertIn("failed to load IMU CSV", result.stderr)

        with tempfile.TemporaryDirectory(prefix="gnss_fuse_config_") as temp_dir:
            config_path = Path(temp_dir) / "fuse.toml"
            config_path.write_text(
                "[gnss_fuse]\n"
                'gnss_pos = "missing-solution.pos"\n'
                'imu = "missing-imu.csv"\n'
                "heading_latch_direction_regression = true\n",
                encoding="utf-8",
            )
            result = self.run_fuse("--config", str(config_path))
            self.assertEqual(result.returncode, 1)
            self.assertIn("failed to load IMU CSV", result.stderr)

            config_path.write_text(
                "[gnss_fuse]\nheading_latch_direction_regression = false\n",
                encoding="utf-8",
            )
            result = self.run_fuse("--config", str(config_path))
            self.assertEqual(result.returncode, 1)
            self.assertIn("boolean false is not supported", result.stderr)

if __name__ == "__main__":
    unittest.main()
