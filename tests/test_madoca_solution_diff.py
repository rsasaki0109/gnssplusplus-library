#!/usr/bin/env python3
"""Tests for MADOCA solution .pos comparison helpers."""

from __future__ import annotations

import csv
import importlib.util
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "analysis" / "madoca_solution_diff.py"

spec = importlib.util.spec_from_file_location("madoca_solution_diff", SCRIPT_PATH)
assert spec is not None and spec.loader is not None
solution_diff = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = solution_diff
spec.loader.exec_module(solution_diff)


class MadocaSolutionDiffTest(unittest.TestCase):
    def test_reads_libgnss_and_madocalib_pos_formats(self) -> None:
        lib_epoch = solution_diff.parse_solution_line(
            "2360 172800.000000 -3857171.270216 3108692.758967 4004040.095338 "
            "39.135150 141.132872 116.9 5 42 999.9"
        )
        bridge_epoch = solution_diff.parse_solution_line(
            "2025/04/01 00:00:00.000   39.135150713  141.132872006   117.1574   1  30"
        )

        self.assertIsNotNone(lib_epoch)
        self.assertIsNotNone(bridge_epoch)
        assert lib_epoch is not None
        assert bridge_epoch is not None
        self.assertEqual(lib_epoch.week, 2360)
        self.assertAlmostEqual(lib_epoch.tow, 172800.0)
        self.assertEqual(lib_epoch.status, 5)
        self.assertEqual(bridge_epoch.week, 2360)
        self.assertAlmostEqual(bridge_epoch.tow, 172800.0)
        self.assertEqual(bridge_epoch.status, 1)

    def test_build_report_and_writes_match_csv(self) -> None:
        with tempfile.TemporaryDirectory(prefix="madoca_solution_diff_test_") as temp_dir:
            root = Path(temp_dir)
            reference = solution_diff.make_reference_frame((1000.0, 2000.0, 3000.0))
            base_pos = root / "base.pos"
            candidate_pos = root / "candidate.pos"
            report_path = root / "report.json"
            match_path = root / "matches.csv"
            base_pos.write_text(
                "% base\n"
                "2360 172800.000000 1000.000 2000.000 3000.000 0.0 0.0 0.0 1 10 1.0\n"
                "2360 172830.000000 1001.000 2000.000 3000.000 0.0 0.0 0.0 1 11 1.0\n",
                encoding="utf-8",
            )
            candidate_pos.write_text(
                "% candidate\n"
                "2360 172800.000000 1000.000 2001.000 3000.000 0.0 0.0 0.0 5 12 1.0\n"
                "2360 172860.000000 1000.000 2000.000 3000.000 0.0 0.0 0.0 5 12 1.0\n",
                encoding="utf-8",
            )

            result = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT_PATH),
                    str(base_pos),
                    str(candidate_pos),
                    "--reference-ecef",
                    "1000",
                    "2000",
                    "3000",
                    "--base-label",
                    "bridge",
                    "--candidate-label",
                    "native",
                    "--threshold-3d",
                    "0.5",
                    "--json-out",
                    str(report_path),
                    "--match-csv",
                    str(match_path),
                ],
                check=False,
                capture_output=True,
                text=True,
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("comparison:", result.stdout)
            self.assertIn("events:", result.stdout)
            report = json.loads(report_path.read_text(encoding="utf-8"))
            self.assertEqual(report["comparison"]["matched_epochs"], 1)
            self.assertEqual(report["base_only_epochs"], 1)
            self.assertEqual(report["candidate_only_epochs"], 1)
            self.assertEqual(report["base_label"], "bridge")
            self.assertEqual(report["candidate_label"], "native")
            self.assertEqual(report["event_thresholds_3d_m"], [0.5])
            self.assertEqual(report["events"]["max_delta_3d"]["tow"], 172800.0)
            crossing = report["events"]["first_delta_3d_over_thresholds"][0]
            self.assertEqual(crossing["threshold_m"], 0.5)
            self.assertIsNotNone(crossing["event"])

            with match_path.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["tow"], "172800.000")
            self.assertGreater(float(rows[0]["delta_3d_m"]), 0.0)


if __name__ == "__main__":
    unittest.main()
