#!/usr/bin/env python3

import importlib.util
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "analysis" / "madoca_glonass_phase_audit.py"
spec = importlib.util.spec_from_file_location("madoca_glonass_phase_audit", SCRIPT)
module = importlib.util.module_from_spec(spec)
assert spec and spec.loader
spec.loader.exec_module(module)


class MadocaGlonassPhaseAuditTest(unittest.TestCase):
    def test_reports_demeaned_variation_and_elevation_shape(self) -> None:
        rows = [
            {"record": "row", "tow": "0", "sat": "R09", "system": "GLO", "obs_type": "phase", "residual_m": "-1", "elevation_deg": "10"},
            {"record": "row", "tow": "30", "sat": "R09", "system": "GLO", "obs_type": "phase", "residual_m": "0", "elevation_deg": "30"},
            {"record": "row", "tow": "60", "sat": "R09", "system": "GLO", "obs_type": "phase", "residual_m": "1", "elevation_deg": "50"},
            {"record": "row", "tow": "60", "sat": "G01", "system": "GPS", "obs_type": "phase", "residual_m": "9", "elevation_deg": "50"},
        ]
        report = module.audit_rows(rows)
        sat = report["satellites"]["R09"]
        self.assertEqual(report["glonass_phase_rows"], 3)
        self.assertAlmostEqual(sat["demeaned_rms_m"], (2 / 3) ** 0.5)
        self.assertAlmostEqual(sat["residual_elevation_correlation"], 1.0)
        self.assertEqual(sat["residual_span_m"], 2.0)

    def test_cli_rejects_shifted_legacy_header(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            source = Path(temp_dir) / "shadow.csv"
            output = Path(temp_dir) / "report.json"
            source.write_text("record,tow,sat,system,residual_m,elevation_deg\n", encoding="utf-8")
            result = subprocess.run([sys.executable, str(SCRIPT), str(source), "--json-out", str(output)], capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("obs_type", json.loads(output.read_text(encoding="utf-8"))["failures"][0])


if __name__ == "__main__":
    unittest.main()
