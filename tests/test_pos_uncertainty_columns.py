#!/usr/bin/env python3
"""Existing Python .pos readers must tolerate the `--pos-uncertainty` and `--pos-fix-diagnostics` columns."""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path
import sys
import tempfile
import unittest


ROOT_DIR = Path(__file__).resolve().parents[1]
COMMANDS_DIR = ROOT_DIR / "apps" / "commands"
for entry in (
    ROOT_DIR / "scripts",
    COMMANDS_DIR,
    COMMANDS_DIR / "diagnostics",
    COMMANDS_DIR / "positioning",
):
    sys.path.insert(0, str(entry))

import generate_driving_comparison as comparison  # noqa: E402
import gnss_mode_compare  # noqa: E402
import gnss_rtk_kinematic_signoff  # noqa: E402

def _load_artifacts():
    # artifacts.py is pure Python; load it directly so the test does not need
    # the compiled libgnsspp extension.
    spec = importlib.util.spec_from_file_location(
        "libgnsspp_artifacts_for_test", ROOT_DIR / "python" / "libgnsspp" / "artifacts.py"
    )
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module  # dataclasses need the module registered
    spec.loader.exec_module(module)
    return module


artifacts = _load_artifacts()

LEGACY_HEADER = (
    "% GPS_Week GPS_TOW X(m) Y(m) Z(m) Lat(deg) Lon(deg) Height(m) Status NumSat PDOP Ratio "
    "Baseline(m) RTKIter RTKObs RTKPhaseObs RTKCodeObs RTKOutliers RTKPrefitRMS(m) "
    "RTKPrefitMax(m) RTKPostSuppressRMS(m) RTKPostSuppressMax(m) RTKUpdateNIS "
    "RTKUpdateNISPerObs RTKUpdateNISRejected"
)
UNCERTAINTY_HEADER = " sdn(m) sde(m) sdu(m) sdne(m) sdeu(m) sdun(m) DiffAge(s)"
FIX_DIAGNOSTIC_HEADER = " FloatFixH(m) FloatFixU(m)"
LEGACY_ROW = (
    "2324 187874.600 -3960753.0205 3349128.2293 3699284.1213 35.678352935 139.782852400 "
    "41.0166 4 26 2.00 37.9 1579.5928 2 58 29 29 7 3.4127 18.0081 0.5938 2.9486 43.4283 "
    "0.7896 0"
)


class PosUncertaintyColumnsReaderTest(unittest.TestCase):
    def write_pos(self, directory: Path, with_columns: bool, nan_tail: bool = False,
                  fix_diagnostics: bool = False) -> Path:
        path = directory / ("unc.pos" if with_columns else "legacy.pos")
        header = LEGACY_HEADER + (UNCERTAINTY_HEADER if with_columns else "")
        row = LEGACY_ROW
        if with_columns:
            row += " nan nan nan nan nan nan nan" if nan_tail else (
                " 0.01000 0.02000 0.03000 -0.00700 0.00000 0.00000 0.600"
            )
        if fix_diagnostics:
            path = directory / ("fixdiag_unc.pos" if with_columns else "fixdiag.pos")
            header += FIX_DIAGNOSTIC_HEADER
            row += " nan nan" if nan_tail else " 0.12345 0.04321"
        path.write_text(f"% LibGNSS++ Position Solution\n{header}\n{row}\n", encoding="ascii")
        return path

    def test_python_readers_ignore_appended_columns(self) -> None:
        # (uncertainty columns, --pos-fix-diagnostics columns, nan values)
        for with_columns, fix_diagnostics, nan_tail in (
            (True, False, False),
            (True, False, True),
            (False, True, False),
            (False, True, True),
            (True, True, False),
            (True, True, True),
        ):
            with self.subTest(with_columns=with_columns, fix_diagnostics=fix_diagnostics,
                              nan_tail=nan_tail), \
                    tempfile.TemporaryDirectory(prefix="gnss_pos_unc_") as tmp:
                legacy = self.write_pos(Path(tmp), with_columns=False)
                extended = self.write_pos(Path(tmp), with_columns=with_columns,
                                          nan_tail=nan_tail, fix_diagnostics=fix_diagnostics)

                old_rows = artifacts.load_pos(legacy)
                new_rows = artifacts.load_pos(extended)
                self.assertEqual(old_rows[0].ecef_m, new_rows[0].ecef_m)
                self.assertEqual((old_rows[0].status, old_rows[0].satellites, old_rows[0].ratio),
                                 (new_rows[0].status, new_rows[0].satellites, new_rows[0].ratio))

                old_mc = gnss_mode_compare.read_pos_records(legacy)
                new_mc = gnss_mode_compare.read_pos_records(extended)
                self.assertEqual(old_mc, new_mc)

                old_dc = comparison.read_libgnss_pos(legacy)[0]
                new_dc = comparison.read_libgnss_pos(extended)[0]
                self.assertEqual(old_dc.status, new_dc.status)
                self.assertEqual(old_dc.rtk_update_rejected_by_innovation_gate,
                                 new_dc.rtk_update_rejected_by_innovation_gate)
                self.assertTrue(math.isclose(old_dc.tow, new_dc.tow))

                self.assertEqual(
                    gnss_rtk_kinematic_signoff.read_pos_records(legacy),
                    gnss_rtk_kinematic_signoff.read_pos_records(extended),
                )


if __name__ == "__main__":
    unittest.main()
