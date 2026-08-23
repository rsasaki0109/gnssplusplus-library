#!/usr/bin/env python3

from __future__ import annotations

import csv
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
GNSS = ROOT / "apps" / "gnss.py"
GPS_EPOCH_UNIX_SECONDS = 315_964_800


class SmartphoneGnssSignoffTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory(prefix="gnss_smartphone_signoff_")
        self.root = Path(self.temp.name)
        self.position = self.root / "solution.pos"
        self.truth = self.root / "ground_truth.csv"
        self.adapter = self.root / "adapter.json"
        self.solver = self.root / "solver.json"
        self.output = self.root / "output"
        week = 2263
        tows = (100.0, 101.0)
        self.position.write_text(
            "% fixture\n"
            + "\n".join(
                f"{week} {tow:.3f} 0 0 0 37.000000 -122.000000 10.0 1 7 2.0"
                for tow in tows
            )
            + "\n",
            encoding="ascii",
        )
        with self.truth.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                (
                    "MessageType",
                    "Provider",
                    "LatitudeDegrees",
                    "LongitudeDegrees",
                    "AltitudeMeters",
                    "UnixTimeMillis",
                )
            )
            for tow in tows:
                unix_ms = int((GPS_EPOCH_UNIX_SECONDS + week * 604800 + tow - 18) * 1000)
                writer.writerow(("Fix", "GT", 37.00001, -122.0, 10.5, unix_ms))
        self.adapter.write_text(
            json.dumps(
                {
                    "schema_version": "smartphone-gnss-adapter.v1",
                    "observations": {"epochs": 2},
                }
            ),
            encoding="utf-8",
        )
        self.solver.write_text(json.dumps({"valid_solutions": 2}), encoding="utf-8")

    def tearDown(self) -> None:
        self.temp.cleanup()

    def run_signoff(self, *gates: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            (
                sys.executable,
                str(GNSS),
                "smartphone-gnss-signoff",
                "--position",
                str(self.position),
                "--ground-truth",
                str(self.truth),
                "--adapter-summary",
                str(self.adapter),
                "--solver-summary",
                str(self.solver),
                "--output-dir",
                str(self.output),
                *gates,
            ),
            cwd=ROOT,
            text=True,
            capture_output=True,
            check=False,
        )

    def test_passes_explicit_truth_and_availability_gates(self) -> None:
        result = self.run_signoff(
            "--require-availability-min",
            "1.0",
            "--require-truth-coverage-min",
            "1.0",
            "--require-horizontal-p95-max",
            "2.0",
            "--require-vertical-p95-max",
            "1.0",
            "--require-max-gap-s",
            "1.0",
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        payload = json.loads((self.output / "signoff_summary.json").read_text(encoding="utf-8"))
        self.assertEqual(payload["decision"], "pass")
        self.assertEqual(payload["metrics"]["truth_matched_epochs"], 2)
        self.assertTrue(all(gate["passed"] for gate in payload["gates"]))

    def test_failed_gate_returns_nonzero_and_keeps_report(self) -> None:
        result = self.run_signoff("--require-horizontal-p95-max", "0.1")
        self.assertNotEqual(result.returncode, 0)
        payload = json.loads((self.output / "signoff_summary.json").read_text(encoding="utf-8"))
        self.assertEqual(payload["decision"], "fail")
        self.assertFalse(payload["gates"][0]["passed"])


if __name__ == "__main__":
    unittest.main()
