"""Regression tests for gnss_fgo's preset option resolution.

`--preset real-data-fixed` on the Eigen batch path (no --imu) reports
per-epoch conditional LAMBDA fixes instead of the batch label that marks
every epoch FIXED (docs/ppc_rtk_vs_gnss_imu_fusion.md, carrier-only Eigen
caveat); an explicit --[no-]epoch-lambda-fixed-output keeps the caller's
choice. The resolved options are read back from --summary-json.
"""

from __future__ import annotations

import json
import os
import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FIXTURE = ROOT / "tests" / "fixtures" / "realtime_stress"


def find_executable(name: str) -> Path:
    binary_dir = Path(os.environ["GNSSPP_BINARY_DIR"])
    executable = f"{name}.exe" if os.name == "nt" else name
    candidates = [
        binary_dir / "apps" / executable,
        *(binary_dir / "apps" / config / executable for config in ("Release", "RelWithDebInfo", "Debug")),
    ]
    found = next((c for c in candidates if c.is_file()), None)
    if found is None:
        raise RuntimeError(f"{name} executable not found; checked: {', '.join(map(str, candidates))}")
    return found


class GnssFgoCliTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.fgo = find_executable("gnss_fgo")
        convert = find_executable("gnss_convert")
        cls.temp = tempfile.TemporaryDirectory(prefix="gnss_fgo_cli_")
        cls.dir = Path(cls.temp.name)
        for name, nav in (("rover", False), ("base", True)):
            args = [str(convert), "--input", str(FIXTURE / f"{name}.rtcm"), "--format", "rtcm",
                    "--gps-week", "2324", "--obs-out", str(cls.dir / f"{name}.obs"), "--quiet"]
            if nav:
                args += ["--nav-out", str(cls.dir / "base.nav")]
            subprocess.run(args, check=True, capture_output=True)

    @classmethod
    def tearDownClass(cls) -> None:
        cls.temp.cleanup()

    def run_fgo(self, *options: str) -> dict:
        summary = self.dir / "summary.json"
        summary.unlink(missing_ok=True)
        result = subprocess.run(
            [str(self.fgo), "--obs", str(self.dir / "rover.obs"), "--base", str(self.dir / "base.obs"),
             "--nav", str(self.dir / "base.nav"), "--out", str(self.dir / "out.pos"),
             "--summary-json", str(summary), "--max-epochs", "20", "--quiet", *options],
            check=False, capture_output=True, text=True,
        )
        self.assertEqual(result.returncode, 0, msg=result.stderr)
        return json.loads(summary.read_text(encoding="utf-8"))

    def resolved(self, summary: dict) -> tuple[bool, bool]:
        options = summary.get("options", summary)
        return options["use_epoch_lambda_fixed_output"], options["report_batch_fix_as_float"]

    def test_real_data_fixed_reports_per_epoch_fixes_on_the_eigen_path(self) -> None:
        self.assertEqual(self.resolved(self.run_fgo("--preset", "real-data-fixed")), (True, True))

    def test_explicit_no_epoch_output_keeps_the_batch_labels(self) -> None:
        summary = self.run_fgo("--preset", "real-data-fixed", "--no-epoch-lambda-fixed-output")
        self.assertEqual(self.resolved(summary), (False, False))

    def test_explicit_epoch_output_is_kept_without_forcing_the_relabel(self) -> None:
        summary = self.run_fgo("--preset", "real-data-fixed", "--epoch-lambda-fixed-output")
        self.assertEqual(self.resolved(summary), (True, False))

    def test_other_presets_are_unchanged(self) -> None:
        self.assertEqual(self.resolved(self.run_fgo("--preset", "real-data-float")), (False, False))
        self.assertEqual(self.resolved(self.run_fgo()), (False, False))


if __name__ == "__main__":
    unittest.main()
