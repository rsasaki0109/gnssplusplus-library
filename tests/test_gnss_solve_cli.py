"""Regression tests for gnss_solve's concise help and TOML config surface."""

from __future__ import annotations

import os
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parents[1]


def ppc_run_dir(city: str, run: int) -> Path | None:
    """PPC run directory (rover.obs, base.obs, base.nav), or None when the dataset is absent."""
    candidates = []
    if os.environ.get("GNSSPP_PPC_DATASET_DIR"):
        candidates.append(Path(os.environ["GNSSPP_PPC_DATASET_DIR"]))
    candidates.append(ROOT_DIR / "data" / "PPC-Dataset")
    for root in candidates:
        run_dir = root / city / f"run{run}"
        if all((run_dir / name).is_file() for name in ("rover.obs", "base.obs", "base.nav")):
            return run_dir
    return None


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
        self.assertLess(len(result.stdout), 5000)

    def test_advanced_help_retains_research_option_discoverability(self) -> None:
        result = self.run_solve("--help-advanced")

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("--config <path>", result.stdout)
        self.assertIn("--rtk-adaptive-noise-alpha-phase", result.stdout)
        self.assertIn("--fixed-bridge-burst-max-residual", result.stdout)

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

    def test_pos_fix_diagnostics_is_advanced_help_and_requires_pos_format(self) -> None:
        advanced = self.run_solve("--help-advanced")
        self.assertEqual(advanced.returncode, 0, msg=advanced.stderr)
        self.assertIn("--pos-fix-diagnostics", advanced.stdout)

        result = self.run_solve(
            "--rover", "rover.obs", "--base", "base.obs", "--nav", "base.nav",
            "--pos-fix-diagnostics", "--format", "llh",
        )
        self.assertEqual(result.returncode, 1)
        self.assertIn("--pos-fix-diagnostics requires --format pos", result.stderr)

    @staticmethod
    def read_pos(path: Path) -> tuple[list[str], list[list[str]]]:
        header: list[str] = []
        rows: list[list[str]] = []
        for line in path.read_text(encoding="ascii").splitlines():
            if line.startswith("%"):
                if "GPS_Week" in line:
                    header = line.lstrip("% ").split()
            elif line.strip():
                rows.append(line.split())
        return header, rows

    def test_pos_fix_diagnostics_columns_on_ppc_run(self) -> None:
        run_dir = ppc_run_dir("tokyo", 1)
        if run_dir is None:
            self.skipTest("PPC tokyo run1 not available (data/PPC-Dataset or GNSSPP_PPC_DATASET_DIR)")
        with tempfile.TemporaryDirectory(prefix="gnss_solve_fix_diag_") as temp_dir:
            temp = Path(temp_dir)
            outputs = {}
            for name, extra in (
                ("off", []),
                ("diag", ["--pos-fix-diagnostics"]),
                ("unc", ["--pos-uncertainty"]),
                ("unc_diag", ["--pos-uncertainty", "--pos-fix-diagnostics"]),
            ):
                out = temp / f"{name}.pos"
                result = self.run_solve(
                    "--rover", str(run_dir / "rover.obs"),
                    "--base", str(run_dir / "base.obs"),
                    "--nav", str(run_dir / "base.nav"),
                    "--mode", "kinematic", "--preset", "low-cost",
                    "--max-epochs", "600", "--no-kml", "--out", str(out), *extra,
                )
                self.assertEqual(result.returncode, 0, msg=result.stderr)
                outputs[name] = (self.read_pos(out), result.stdout)

            (off_header, off_rows), _ = outputs["off"]
            (diag_header, diag_rows), diag_stdout = outputs["diag"]
            (unc_header, unc_rows), _ = outputs["unc"]
            (both_header, both_rows), _ = outputs["unc_diag"]
            self.assertIn("fix diagnostics:", diag_stdout)

            # The flag only appends two columns, after the uncertainty columns.
            self.assertEqual(diag_header, off_header + ["FloatFixH(m)", "FloatFixU(m)"])
            self.assertEqual(both_header, unc_header + ["FloatFixH(m)", "FloatFixU(m)"])
            self.assertEqual(len(diag_rows), len(off_rows))
            self.assertEqual(len(both_rows), len(unc_rows))
            for base_rows, extended in ((off_rows, diag_rows), (unc_rows, both_rows)):
                for plain, extra in zip(base_rows, extended):
                    self.assertEqual(plain, extra[:-2])

            status_index = off_header.index("Status")
            fixed_status = "4"
            finite_fixed = 0
            for row in diag_rows:
                h_value, u_value = row[-2], row[-1]
                if row[status_index] != fixed_status:
                    self.assertEqual((h_value, u_value), ("nan", "nan"))
                    continue
                if h_value == "nan":
                    self.assertEqual(u_value, "nan")
                    continue
                finite_fixed += 1
                self.assertGreaterEqual(float(h_value), 0.0)
                self.assertGreaterEqual(float(u_value), 0.0)
                self.assertLess(float(h_value), 100.0)
            self.assertGreater(finite_fixed, 0)


if __name__ == "__main__":
    unittest.main()
