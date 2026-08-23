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
                        "lever_arm = [0.31, 0.0, -0.55]",
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


if __name__ == "__main__":
    unittest.main()
