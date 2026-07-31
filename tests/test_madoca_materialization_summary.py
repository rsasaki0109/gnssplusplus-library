#!/usr/bin/env python3
"""Tests for MADOCA materialization snapshot summaries."""

from __future__ import annotations

import importlib.util
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "analysis" / "madoca_materialization_summary.py"

spec = importlib.util.spec_from_file_location("madoca_materialization_summary", SCRIPT_PATH)
assert spec is not None and spec.loader is not None
summary_script = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = summary_script
spec.loader.exec_module(summary_script)


HEADER = (
    "schema_version,sat,system,prn,week,tow,orbit_frame,orbit_valid,clock_valid,"
    "code_bias_valid,phase_bias_valid,orbit_week,orbit_tow,clock_week,clock_tow,"
    "iode,ssr_orbit_iod,ssr_clock_iod,orbit_radial_m,orbit_along_m,orbit_cross_m,"
    "clock_m,code_bias_count,code_biases_m,phase_bias_count,phase_biases_m,"
    "phase_bias_discnt\n"
)


def snapshot_row(
    *,
    sat: str,
    system: str,
    prn: int,
    tow: float,
    code_bias_count: int = 1,
    code_biases: str = "2:0.1",
    phase_bias_count: int = 1,
    phase_biases: str = "8:0.2",
    discnt: str = "8:3",
) -> str:
    return (
        f"madoca_materialization_snapshot.v1,{sat},{system},{prn},2360,{tow},rac,"
        f"1,1,{1 if code_bias_count else 0},{1 if phase_bias_count else 0},"
        "2360,100.0,2360,105.0,33,11,11,0.1,0.2,0.3,0.4,"
        f"{code_bias_count},{code_biases},{phase_bias_count},{phase_biases},{discnt}\n"
    )


class MadocaMaterializationSummaryTest(unittest.TestCase):
    def test_summarize_rows_reports_systems_bias_ids_and_time_range(self) -> None:
        rows = summary_script.materialization_diff.read_csv_rows(
            self.write_snapshot(
                snapshot_row(sat="G01", system="GPS", prn=1, tow=100.0),
                snapshot_row(
                    sat="J02",
                    system="QZS",
                    prn=194,
                    tow=105.0,
                    code_bias_count=2,
                    code_biases="2:0.1;9:-0.2",
                    phase_bias_count=0,
                    phase_biases="",
                    discnt="",
                ),
            )
        )

        payload = summary_script.summarize_rows(rows)

        self.assertEqual(payload["schema"], "madoca_materialization_summary.v1")
        self.assertEqual(payload["status"], "passed")
        self.assertEqual(payload["rows"], 2)
        self.assertEqual(payload["systems"], {"GPS": 1, "QZS": 1})
        self.assertEqual(payload["bias_identity"]["code_bias_ids"], {"2": 2, "9": 1})
        self.assertEqual(payload["bias_identity"]["phase_bias_ids"], {"8": 1})
        self.assertEqual(payload["time_range"]["start_tow"], 100.0)
        self.assertEqual(payload["time_range"]["end_tow"], 105.0)

    def test_summarize_rows_detects_count_mismatch_and_duplicate_keys(self) -> None:
        path = self.write_snapshot(
            snapshot_row(sat="G01", system="GPS", prn=1, tow=100.0, code_bias_count=2, code_biases="2:0.1"),
            snapshot_row(sat="G01", system="GPS", prn=1, tow=100.0, phase_biases="", discnt="8:3"),
        )
        rows = summary_script.materialization_diff.read_csv_rows(path)

        payload = summary_script.summarize_rows(rows)

        self.assertEqual(payload["status"], "failed")
        self.assertEqual(payload["row_key"]["duplicate_groups"], 1)
        self.assertEqual(payload["row_key"]["max_duplicate_occurrences"], 2)
        self.assertEqual(payload["issue_counts"]["code_bias_count_mismatch"], 1)
        self.assertEqual(payload["issue_counts"]["phase_bias_count_mismatch"], 1)
        self.assertEqual(payload["issue_counts"]["phase_discnt_without_phase_bias"], 1)

    def test_cli_writes_json_and_enforces_requirements(self) -> None:
        with tempfile.TemporaryDirectory(prefix="madoca_materialization_summary_cli_") as temp_dir:
            output = Path(temp_dir) / "summary.json"
            snapshot = self.write_snapshot(
                snapshot_row(sat="G01", system="GPS", prn=1, tow=100.0),
                root=Path(temp_dir),
            )

            result = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT_PATH),
                    str(snapshot),
                    "--json-out",
                    str(output),
                    "--require-rows-min",
                    "2",
                    "--require-system",
                    "QZS",
                ],
                cwd=ROOT_DIR,
                text=True,
                capture_output=True,
                check=False,
            )

            self.assertNotEqual(result.returncode, 0)
            payload = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual(payload["status"], "failed")
            self.assertIn("rows 1 < required minimum 2", payload["failures"])
            self.assertIn("required system QZS is absent", payload["failures"])

    def test_cli_passes_clean_snapshot_with_fail_on_issue(self) -> None:
        with tempfile.TemporaryDirectory(prefix="madoca_materialization_summary_pass_") as temp_dir:
            output = Path(temp_dir) / "summary.json"
            snapshot = self.write_snapshot(
                snapshot_row(sat="G01", system="GPS", prn=1, tow=100.0),
                root=Path(temp_dir),
            )

            result = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT_PATH),
                    str(snapshot),
                    "--json-out",
                    str(output),
                    "--require-rows-min",
                    "1",
                    "--require-code-bias-id",
                    "2",
                    "--fail-on-issue",
                ],
                cwd=ROOT_DIR,
                text=True,
                capture_output=True,
                check=False,
            )

            self.assertEqual(result.returncode, 0, result.stderr)
            payload = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual(payload["status"], "passed")
            self.assertEqual(payload["failures"], [])

    def write_snapshot(self, *rows: str, root: Path | None = None) -> Path:
        if root is None:
            self.addCleanup(lambda: None)
            temp_dir = tempfile.TemporaryDirectory(prefix="madoca_materialization_summary_")
            self.addCleanup(temp_dir.cleanup)
            root = Path(temp_dir.name)
        path = root / "snapshot.csv"
        path.write_text(HEADER + "".join(rows), encoding="ascii")
        return path


if __name__ == "__main__":
    unittest.main()
