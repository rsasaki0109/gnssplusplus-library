"""Regression tests for gnss_solve's concise help and TOML config surface."""

from __future__ import annotations

import os
import subprocess
import tempfile
import unittest
from pathlib import Path


class GnssSolveCliTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        binary_dir = Path(os.environ["GNSSPP_BINARY_DIR"])
        executable_name = "gnss_solve.exe" if os.name == "nt" else "gnss_solve"
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
            raise RuntimeError(f"gnss_solve executable not found; checked: {rendered}")

    @classmethod
    def run_solve(cls, *options: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [str(cls.executable), *options],
            check=False,
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
        )

    def test_default_help_keeps_the_supported_path_concise(self) -> None:
        result = self.run_solve("--help")

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("--data-dir <dir>", result.stdout)
        self.assertIn("--config <path>", result.stdout)
        self.assertIn("--preset <name>", result.stdout)
        self.assertIn("--help-advanced", result.stdout)
        self.assertNotIn("--rtk-adaptive-noise-alpha-phase", result.stdout)
        self.assertNotIn("--fixed-bridge-burst-max-residual", result.stdout)
        self.assertNotIn("--multisd-fgo-shadow-csv", result.stdout)
        self.assertLess(len(result.stdout), 5000)

    def test_advanced_help_retains_research_option_discoverability(self) -> None:
        result = self.run_solve("--help-advanced")

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("--config <path>", result.stdout)
        self.assertIn("--rtk-adaptive-noise-alpha-phase", result.stdout)
        self.assertIn("--fixed-bridge-burst-max-residual", result.stdout)
        self.assertIn("--multisd-fgo-shadow-csv", result.stdout)
        self.assertIn("--multisd-fgo-shadow-window", result.stdout)
        self.assertIn(
            "--multisd-fgo-shadow-max-seed-separation", result.stdout
        )
        self.assertIn(
            "--multisd-fgo-shadow-validation-history", result.stdout
        )
        self.assertIn(
            "--multisd-fgo-shadow-min-carrier-fraction", result.stdout
        )
        self.assertIn(
            "--multisd-fgo-shadow-min-fixed-ambiguities", result.stdout
        )
        self.assertIn(
            "--multisd-fgo-shadow-holdout-satellites", result.stdout
        )
        self.assertIn(
            "--multisd-fgo-shadow-constellation-par", result.stdout
        )
        self.assertIn(
            "--multisd-fgo-shadow-variance-ranked-par", result.stdout
        )
        self.assertIn(
            "--multisd-fgo-shadow-interleave-constellation-par", result.stdout
        )
        self.assertIn(
            "--multisd-fgo-shadow-quality-ranked-par", result.stdout
        )
        self.assertIn(
            "--multisd-fgo-shadow-candidate-ratio", result.stdout
        )
        self.assertIn(
            "--multisd-fgo-shadow-candidate-groups", result.stdout
        )
        self.assertIn(
            "--multisd-fgo-shadow-fallback-consensus-groups", result.stdout
        )
        self.assertIn(
            "--multisd-fgo-shadow-fallback-consensus-separation", result.stdout
        )
        self.assertIn(
            "--multisd-fgo-shadow-fallback-max-seed-separation", result.stdout
        )
        self.assertIn("--multisd-fgo-shadow-min-bsr", result.stdout)
        self.assertIn("--multisd-fgo-shadow-max-adop", result.stdout)

    def test_config_supplies_defaults_and_cli_always_overrides_them(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_solve_config_") as temp_dir:
            config_path = Path(temp_dir) / "solve.toml"
            config_path.write_text(
                "\n".join(
                    [
                        "[gnss_solve]",
                        'data_dir = "missing-run"',
                        "preset = 'low-cost'",
                        "rtk_snr_weighting = true",
                        "arfilter = false",
                        "max_epochs = -1",
                    ]
                ),
                encoding="utf-8",
            )

            result = self.run_solve(
                "--max-epochs",
                "0",
                "--config",
                str(config_path),
            )

        self.assertEqual(result.returncode, 1)
        self.assertIn("cannot open rover observation file", result.stderr)
        self.assertNotIn("--max-epochs must be >= 0", result.stderr)

    def test_unknown_config_key_is_rejected_by_the_existing_option_parser(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_solve_config_") as temp_dir:
            config_path = Path(temp_dir) / "solve.toml"
            config_path.write_text(
                "[gnss_solve]\nthis_option_does_not_exist = 1\n",
                encoding="utf-8",
            )

            result = self.run_solve("--config", str(config_path))

        self.assertEqual(result.returncode, 1)
        self.assertIn("unknown or incomplete argument: --this-option-does-not-exist",
                      result.stderr)

    def test_multisd_candidate_controls_reject_unsafe_ranges(self) -> None:
        ratio = self.run_solve(
            "--data-dir", "missing",
            "--multisd-fgo-shadow-candidate-ratio", "0.99"
        )
        groups = self.run_solve(
            "--data-dir", "missing",
            "--multisd-fgo-shadow-candidate-groups", "33"
        )
        consensus = self.run_solve(
            "--data-dir", "missing",
            "--multisd-fgo-shadow-fallback-consensus-groups", "2"
        )
        bsr = self.run_solve(
            "--data-dir", "missing",
            "--multisd-fgo-shadow-min-bsr", "1.01"
        )
        adop = self.run_solve(
            "--data-dir", "missing",
            "--multisd-fgo-shadow-max-adop", "-0.01"
        )

        self.assertEqual(ratio.returncode, 1)
        self.assertIn("candidate-ratio must be >= 1", ratio.stderr)
        self.assertEqual(groups.returncode, 1)
        self.assertIn("candidate-groups must be in [1, 32]", groups.stderr)
        self.assertEqual(consensus.returncode, 1)
        self.assertIn(
            "fallback-consensus-separation must be > 0", consensus.stderr
        )
        self.assertEqual(bsr.returncode, 1)
        self.assertIn("min-bsr must be in [0, 1]", bsr.stderr)
        self.assertEqual(adop.returncode, 1)
        self.assertIn("max-adop must be >= 0", adop.stderr)


if __name__ == "__main__":
    unittest.main()
