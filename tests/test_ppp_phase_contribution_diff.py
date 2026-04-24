#!/usr/bin/env python3
"""Tests for PPP phase contribution comparison helpers."""

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
SCRIPT_PATH = ROOT_DIR / "scripts" / "analysis" / "ppp_phase_contribution_diff.py"

spec = importlib.util.spec_from_file_location("ppp_phase_contribution_diff", SCRIPT_PATH)
assert spec is not None and spec.loader is not None
phase_diff = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = phase_diff
spec.loader.exec_module(phase_diff)


FIELDNAMES = [
    "week",
    "tow",
    "iteration",
    "sat",
    "row_type",
    "residual_m",
    "primary_signal",
    "secondary_signal",
    "primary_observation_code",
    "secondary_observation_code",
    "frequency_index",
    "ionosphere_coefficient",
    "position_update_contribution_x_m",
    "position_update_contribution_y_m",
    "position_update_contribution_z_m",
    "position_update_contribution_3d_m",
    "receiver_clock_update_contribution_m",
    "ionosphere_update_contribution_m",
    "ambiguity_update_contribution_m",
    "innovation_inverse_phase_coupling_abs_1_per_m2",
    "receiver_clock_kalman_gain",
    "position_x_kalman_gain",
]


def contribution_row(
    *,
    sat: str,
    tow: str = "10.000",
    iteration: str = "0",
    row_type: str = "phase",
    frequency_index: str = "0",
    x: str = "0",
    y: str = "0",
    z: str = "0",
    position_3d: str = "0",
    clock: str = "0",
    ionosphere: str = "0",
    ambiguity: str = "0",
    residual: str = "0",
    inverse_phase_coupling: str = "0",
    clock_gain: str = "0",
    position_x_gain: str = "0",
) -> dict[str, str]:
    return {
        "week": "2360",
        "tow": tow,
        "iteration": iteration,
        "sat": sat,
        "row_type": row_type,
        "residual_m": residual,
        "primary_signal": "0",
        "secondary_signal": "2",
        "primary_observation_code": "C1C",
        "secondary_observation_code": "C2W",
        "frequency_index": frequency_index,
        "ionosphere_coefficient": "1.0",
        "position_update_contribution_x_m": x,
        "position_update_contribution_y_m": y,
        "position_update_contribution_z_m": z,
        "position_update_contribution_3d_m": position_3d,
        "receiver_clock_update_contribution_m": clock,
        "ionosphere_update_contribution_m": ionosphere,
        "ambiguity_update_contribution_m": ambiguity,
        "innovation_inverse_phase_coupling_abs_1_per_m2": inverse_phase_coupling,
        "receiver_clock_kalman_gain": clock_gain,
        "position_x_kalman_gain": position_x_gain,
    }


def write_csv(path: Path, rows: list[dict[str, str]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerows(rows)


class PppPhaseContributionDiffTest(unittest.TestCase):
    def test_summarizes_windowed_accepted_phase_contribution_deltas(self) -> None:
        base_rows = [
            contribution_row(sat="G01", x="1", position_3d="1", clock="0.1", residual="2"),
            contribution_row(sat="E11", row_type="code", x="10"),
            contribution_row(sat="G02", iteration="1", x="10"),
        ]
        candidate_rows = [
            contribution_row(sat="G01", x="1.5", position_3d="1.5", clock="0.2", residual="3"),
            contribution_row(
                sat="C30",
                frequency_index="1",
                x="-0.5",
                position_3d="0.5",
                ionosphere="0.4",
                ambiguity="1.2",
                residual="-4",
            ),
        ]

        report = phase_diff.summarize_contribution_diff(
            base_rows,
            candidate_rows,
            windows=[("early", 0.0, 20.0)],
            iteration=0,
            top=2,
        )

        window = report["windows"][0]
        self.assertEqual(window["totals"]["base"]["rows"], 1)
        self.assertEqual(window["totals"]["candidate"]["rows"], 2)
        self.assertEqual(window["totals"]["delta"]["rows"], 1)
        self.assertAlmostEqual(
            window["totals"]["delta"]["position_update_3d_abs_sum_m"],
            1.0,
        )
        self.assertAlmostEqual(window["totals"]["delta"]["ambiguity_update_sum_m"], 1.2)
        self.assertEqual(window["top_identity_deltas"][0]["identity"]["sat"], "C30")
        self.assertEqual(
            window["top_identity_deltas"][0]["identity"]["frequency_index"],
            1,
        )

    def test_summarizes_code_row_contribution_deltas(self) -> None:
        base_rows = [
            contribution_row(sat="G01", row_type="phase", x="9", position_3d="9"),
            contribution_row(sat="G01", row_type="code", x="1", position_3d="1", clock="2"),
        ]
        candidate_rows = [
            contribution_row(sat="G01", row_type="code", x="3", position_3d="3", clock="5"),
            contribution_row(sat="G02", row_type="code", x="4", position_3d="4", ionosphere="7"),
        ]

        report = phase_diff.summarize_contribution_diff(
            base_rows,
            candidate_rows,
            row_type="code",
            windows=[("early", 0.0, 20.0)],
            iteration=0,
            top=2,
        )

        window = report["windows"][0]
        self.assertEqual(report["row_type"], "code")
        self.assertEqual(window["totals"]["base"]["rows"], 1)
        self.assertEqual(window["totals"]["candidate"]["rows"], 2)
        self.assertAlmostEqual(window["totals"]["delta"]["position_update_sum_x_m"], 6.0)
        self.assertAlmostEqual(window["totals"]["delta"]["receiver_clock_update_sum_m"], 3.0)
        self.assertAlmostEqual(window["totals"]["delta"]["ionosphere_update_sum_m"], 7.0)

    def test_summarizes_optional_coupling_and_gain_columns(self) -> None:
        base_rows = [
            contribution_row(
                sat="G01",
                row_type="code",
                inverse_phase_coupling="1.5",
                clock_gain="-2",
                position_x_gain="3",
            ),
        ]
        candidate_rows = [
            contribution_row(
                sat="G01",
                row_type="code",
                inverse_phase_coupling="2.0",
                clock_gain="4",
                position_x_gain="-5",
            ),
        ]

        report = phase_diff.summarize_contribution_diff(
            base_rows,
            candidate_rows,
            row_type="code",
            windows=[("early", 0.0, 20.0)],
            iteration=0,
            top=1,
        )

        delta = report["windows"][0]["totals"]["delta"]
        self.assertAlmostEqual(delta["innovation_inverse_phase_coupling_abs_sum_1_per_m2"], 0.5)
        self.assertAlmostEqual(delta["receiver_clock_kalman_gain_abs_sum"], 2.0)
        self.assertAlmostEqual(delta["position_kalman_gain_abs_sum"], 2.0)

    def test_cli_writes_json_report(self) -> None:
        with tempfile.TemporaryDirectory(prefix="ppp_phase_contribution_diff_test_") as temp_dir:
            temp_root = Path(temp_dir)
            base_path = temp_root / "base.csv"
            candidate_path = temp_root / "candidate.csv"
            report_path = temp_root / "report.json"
            write_csv(base_path, [contribution_row(sat="G01", row_type="code", x="1", position_3d="1")])
            write_csv(candidate_path, [contribution_row(sat="G01", row_type="code", x="2", position_3d="2")])

            result = subprocess.run(
                [
                    sys.executable,
                    str(SCRIPT_PATH),
                    str(base_path),
                    str(candidate_path),
                    "--window",
                    "early:0:20",
                    "--iteration",
                    "0",
                    "--row-type",
                    "code",
                    "--json-out",
                    str(report_path),
                ],
                check=False,
                capture_output=True,
                text=True,
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("ppp_contribution_diff:", result.stdout)
            report = json.loads(report_path.read_text(encoding="utf-8"))
            self.assertEqual(report["row_type"], "code")
            self.assertEqual(report["windows"][0]["totals"]["delta"]["rows"], 0)
            self.assertAlmostEqual(
                report["windows"][0]["totals"]["delta"]["position_update_3d_abs_sum_m"],
                1.0,
            )


if __name__ == "__main__":
    unittest.main()
