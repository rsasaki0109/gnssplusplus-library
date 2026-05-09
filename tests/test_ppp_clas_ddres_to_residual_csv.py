#!/usr/bin/env python3
"""Tests for CLAS DD residual CSV conversion."""

from __future__ import annotations

import csv
import importlib.util
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "analysis" / "ppp_clas_ddres_to_residual_csv.py"

spec = importlib.util.spec_from_file_location("ppp_clas_ddres_to_residual_csv", SCRIPT_PATH)
assert spec is not None and spec.loader is not None
converter = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = converter
spec.loader.exec_module(converter)


class PppClasDdresToResidualCsvTest(unittest.TestCase):
    def test_converts_libgnss_ddres_sd_phase_rows(self) -> None:
        text = """strict_clas_ddres_dump=1
time_week=2360
time_tow=123.000000000000
[sd_phase]
k=0 ref=G05 sat=G12 freq_index=0 freq_group=0 sd_residual_m=1.25 variance_m2=4
k=1 ref=G05 sat=G19 freq_index=1 freq_group=1 sd_residual_m=-2.5 variance_m2=9
"""
        with tempfile.TemporaryDirectory(prefix="clas_ddres_convert_test_") as temp_dir:
            source = Path(temp_dir) / "native.dump"
            out = Path(temp_dir) / "native.csv"
            source.write_text(text, encoding="utf-8")

            rows = converter.parse_libgnss_ddres(source)
            self.assertEqual(len(rows), 2)
            converter.write_residual_csv(rows, out, canonicalize_pairs=False)

            with out.open(newline="", encoding="utf-8") as handle:
                csv_rows = list(csv.DictReader(handle))
            self.assertEqual(csv_rows[0]["week"], "2360")
            self.assertEqual(csv_rows[0]["tow"], "123.000")
            self.assertEqual(csv_rows[0]["sat"], "G05>G12")
            self.assertEqual(csv_rows[0]["frequency_index"], "0")
            self.assertEqual(csv_rows[0]["residual_m"], "1.25")
            self.assertEqual(csv_rows[1]["sat"], "G05>G19")
            self.assertEqual(csv_rows[1]["residual_m"], "-2.5")

    def test_converts_claslib_model_comp_phase_rows(self) -> None:
        text = """noise
[CLAS-MODEL-COMP] type=phase tow=123.000 ref=G05 sat=G12 freq_group=0 y=-0.75 obs=10.25 amb_m=1.5
[CLAS-MODEL-COMP] type=code tow=123.000 ref=G05 sat=G12 freq_group=0 y=99
[CLAS-MODEL-COMP] type=phase week=2361 tow=124.000 ref=G12 sat=G05 freq_group=1 y=2.5 obs=9.0
"""
        with tempfile.TemporaryDirectory(prefix="clas_model_convert_test_") as temp_dir:
            source = Path(temp_dir) / "bridge.stat"
            out = Path(temp_dir) / "bridge.csv"
            source.write_text(text, encoding="utf-8")

            rows = converter.parse_claslib_model_comp(source, default_week=2360)
            self.assertEqual(len(rows), 2)
            converter.write_residual_csv(rows, out, canonicalize_pairs=True)

            with out.open(newline="", encoding="utf-8") as handle:
                csv_rows = list(csv.DictReader(handle))
            self.assertEqual(csv_rows[0]["week"], "2360")
            self.assertEqual(csv_rows[0]["sat"], "G05>G12")
            self.assertEqual(csv_rows[0]["residual_m"], "-0.75")
            self.assertEqual(csv_rows[0]["predicted_m"], "11")
            self.assertEqual(csv_rows[1]["week"], "2361")
            self.assertEqual(csv_rows[1]["sat"], "G05>G12")
            self.assertEqual(csv_rows[1]["residual_m"], "-2.5")

    def test_converts_measrow_dump_phase_rows(self) -> None:
        text = """tow=123.000 row=0 sat=G12 ref=G05 is_phase=1 freq=0 z=1.25 R=4
tow=123.000 row=1 sat=G12 ref=G05 is_phase=0 freq=0 z=99 R=9
tow=124.000 row=2 sat=G05 ref=G12 is_phase=1 freq=1 z=-2.5 R=16
tow=125.000 row=3 sat=G06 ref=none is_phase=1 freq=0 z=3 R=1
"""
        with tempfile.TemporaryDirectory(prefix="clas_measrow_convert_test_") as temp_dir:
            source = Path(temp_dir) / "measrow.dump"
            out = Path(temp_dir) / "measrow.csv"
            source.write_text(text, encoding="utf-8")

            rows = converter.parse_measrow_dump(source, default_week=2360)
            self.assertEqual(len(rows), 2)
            converter.write_residual_csv(rows, out, canonicalize_pairs=True)

            with out.open(newline="", encoding="utf-8") as handle:
                csv_rows = list(csv.DictReader(handle))
            self.assertEqual(csv_rows[0]["week"], "2360")
            self.assertEqual(csv_rows[0]["tow"], "123.000")
            self.assertEqual(csv_rows[0]["sat"], "G05>G12")
            self.assertEqual(csv_rows[0]["frequency_index"], "0")
            self.assertEqual(csv_rows[0]["residual_m"], "1.25")
            self.assertEqual(csv_rows[1]["sat"], "G05>G12")
            self.assertEqual(csv_rows[1]["frequency_index"], "1")
            self.assertEqual(csv_rows[1]["residual_m"], "2.5")

    def test_cli_writes_residual_csv(self) -> None:
        text = """[CLAS-MODEL-COMP] type=phase tow=1.000 ref=G01 sat=G02 freq_group=0 y=0.125
"""
        with tempfile.TemporaryDirectory(prefix="clas_ddres_convert_cli_test_") as temp_dir:
            source = Path(temp_dir) / "bridge.stat"
            out = Path(temp_dir) / "bridge.csv"
            source.write_text(text, encoding="utf-8")

            result = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT_PATH),
                    "--claslib-model-comp",
                    str(source),
                    "--week",
                    "2360",
                    "--out",
                    str(out),
                ],
                check=False,
                capture_output=True,
                text=True,
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("wrote 1 residual rows", result.stdout)
            with out.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(rows[0]["week"], "2360")
            self.assertEqual(rows[0]["sat"], "G01>G02")
            self.assertEqual(rows[0]["residual_m"], "0.125")


if __name__ == "__main__":
    unittest.main()
