#!/usr/bin/env python3
"""Tests for CLAS SD no-ambiguity comparison helpers."""

from __future__ import annotations

import contextlib
import io
import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "analysis" / "ppp_clas_sd_noamb_compare.py"

spec = importlib.util.spec_from_file_location("ppp_clas_sd_noamb_compare", SCRIPT_PATH)
assert spec is not None and spec.loader is not None
compare = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = compare
spec.loader.exec_module(compare)


class PppClasSdNoambCompareTest(unittest.TestCase):
    def test_parses_legacy_model_components_with_measrow_metadata(self) -> None:
        measrow_text = """tow=10.000000 row=0 sat=G12 ref=G05 is_phase=1 freq=0 z=1.25 R=4
tow=10.000000 row=1 sat=G12 ref=G05 is_phase=0 freq=0 z=99 R=9
tow=10.000000 row=2 sat=G05 ref=G12 is_phase=1 freq=1 z=-2.5 R=16
"""
        model_text = """tow=10.000000 sat=G12 is_phase=1 freq=0 obs=12 rho=20 dts=3 rel=0.1 sagnac=0.2 trop_d=0.3 trop_w=0.4 trop_grid=0.5 iono_grid=0.6 iono_state=0.7 cpc=0.8 pb=0.9 pcomp=1.0 wu=1.1 pco_r=0.1 pco_s=0.2 pcv_r=0.3 pcv_s=0.4 amb=2.0 sum=10.75 y=1.25
tow=10.000000 sat=G12 is_phase=0 freq=0 obs=20 y=99
tow=10.000000 sat=G05 is_phase=1 freq=1 obs=30 rho=40 dts=5 rel=0.2 sagnac=0.3 trop_d=0.4 trop_w=0.5 trop_grid=0.6 iono_grid=0.7 iono_state=0.8 cpc=0.9 pb=1.0 pcomp=1.1 wu=1.2 amb=-4.0 sum=32.5 y=-2.5
"""
        with tempfile.TemporaryDirectory(prefix="clas_sd_noamb_compare_test_") as temp_dir:
            measrow_path = Path(temp_dir) / "measrow.dump"
            model_path = Path(temp_dir) / "model.dump"
            measrow_path.write_text(measrow_text, encoding="utf-8")
            model_path.write_text(model_text, encoding="utf-8")

            rows, receiver_positions = compare.parse_claslib_legacy_model_comp(
                model_path,
                measrow_path,
            )

        self.assertEqual(receiver_positions, {})
        self.assertEqual(len(rows), 2)
        row0 = rows[(10000, "G05", "G12", 0)]
        self.assertAlmostEqual(row0.y_m, 1.25)
        self.assertAlmostEqual(row0.amb_m, 2.0)
        self.assertAlmostEqual(row0.noamb_m, 3.25)
        self.assertAlmostEqual(row0.trop_model_m, 0.7)
        self.assertAlmostEqual(row0.iono_state_term_m, 0.7)
        self.assertAlmostEqual(row0.phase_bias_m, 0.9)
        self.assertAlmostEqual(row0.pco_pcv_m, 1.0)
        self.assertTrue(row0.components_are_sd)
        row1 = rows[(10000, "G12", "G05", 1)]
        self.assertAlmostEqual(row1.y_m, -2.5)
        self.assertAlmostEqual(row1.amb_m, -4.0)
        self.assertAlmostEqual(row1.noamb_m, -6.5)

    def test_main_reports_regular_claslib_pos_solution_deltas(self) -> None:
        with tempfile.TemporaryDirectory(prefix="clas_sd_noamb_compare_test_") as temp_dir:
            temp_path = Path(temp_dir)
            ddres_path = temp_path / "lib.ddres"
            model_path = temp_path / "model.dump"
            lib_pos_path = temp_path / "lib.pos"
            claslib_pos_path = temp_path / "claslib.pos"
            ddres_path.write_text(
                """strict_clas_ddres_dump=1
time_tow=10.000000
receiver_x_m=1.0
receiver_y_m=2.0
receiver_z_m=3.0
""",
                encoding="utf-8",
            )
            model_path.write_text("", encoding="utf-8")
            lib_pos_path.write_text(
                "2324 10.000000 1.0 2.0 3.0\n",
                encoding="utf-8",
            )
            claslib_pos_path.write_text(
                "2324 10.000000 2.0 2.0 3.0\n",
                encoding="utf-8",
            )

            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                rc = compare.main(
                    [
                        "--libgnss-ddres",
                        str(ddres_path),
                        "--claslib-model-comp",
                        str(model_path),
                        "--libgnss-pos",
                        str(lib_pos_path),
                        "--claslib-pos",
                        str(claslib_pos_path),
                    ]
                )

        self.assertEqual(rc, 0)
        text = output.getvalue()
        self.assertIn("lib prior vs CLASLIB solution:", text)
        self.assertIn("lib solution vs CLASLIB solution:", text)
        self.assertIn("matched_positions=1", text)
        self.assertIn("position_rms_delta_m=1", text)


if __name__ == "__main__":
    unittest.main()
