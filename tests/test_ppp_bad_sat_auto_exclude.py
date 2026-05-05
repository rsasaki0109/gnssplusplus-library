#!/usr/bin/env python3
"""Tests for the bad-satellite auto-exclusion helper."""

from __future__ import annotations

import csv
import importlib.util
import io
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "analysis" / "ppp_bad_sat_auto_exclude.py"

spec = importlib.util.spec_from_file_location("ppp_bad_sat_auto_exclude", SCRIPT_PATH)
assert spec is not None and spec.loader is not None
auto_exclude = importlib.util.module_from_spec(spec)
spec.loader.exec_module(auto_exclude)


FIELDNAMES = [
    "week",
    "tow",
    "iteration",
    "row_index",
    "sat",
    "row_type",
    "residual_m",
    "frequency_index",
    "phase_accepted",
]


def make_row(**overrides: object) -> dict:
    row = {
        "week": "2360",
        "tow": "173000",
        "iteration": "1",
        "row_index": "0",
        "sat": "G05",
        "row_type": "phase",
        "residual_m": "0.001",
        "frequency_index": "0",
        "phase_accepted": "1",
    }
    row.update({k: str(v) for k, v in overrides.items()})
    return row


class CollectPairsTests(unittest.TestCase):
    def test_flags_pairs_above_threshold(self) -> None:
        rows = [
            # Clean satellite (residual 1 mm) at a range of tows.
            make_row(tow="174000", sat="G01", residual_m="0.001"),
            make_row(tow="174030", sat="G01", residual_m="0.001"),
            # Bad satellite (residual 35 mm) on both frequencies.
            make_row(tow="174000", sat="G26", residual_m="0.035", frequency_index="0"),
            make_row(tow="174030", sat="G26", residual_m="0.035", frequency_index="0"),
            make_row(tow="174000", sat="G26", residual_m="0.022", frequency_index="1"),
            make_row(tow="174030", sat="G26", residual_m="0.022", frequency_index="1"),
        ]
        flagged = auto_exclude.collect_pairs(
            rows,
            tail_seconds=900.0,
            threshold_m=0.015,
            min_samples=2,
        )
        self.assertIn(("G26", 0), flagged)
        self.assertIn(("G26", 1), flagged)
        self.assertNotIn(("G01", 0), flagged)

    def test_min_samples_gate_suppresses_thin_pairs(self) -> None:
        rows = [
            make_row(tow="174000", sat="G26", residual_m="0.040"),
            # Only one sample — must not survive the ``min_samples=3`` gate.
        ]
        flagged = auto_exclude.collect_pairs(
            rows,
            tail_seconds=900.0,
            threshold_m=0.015,
            min_samples=3,
        )
        self.assertEqual(flagged, {})

    def test_excludes_banned_systems(self) -> None:
        rows = [
            make_row(tow="174000", sat="C07", residual_m="0.100"),
            make_row(tow="174030", sat="C07", residual_m="0.100"),
        ]
        flagged = auto_exclude.collect_pairs(
            rows,
            tail_seconds=900.0,
            threshold_m=0.015,
            min_samples=2,
            exclude_systems=["C"],
        )
        self.assertEqual(flagged, {})

    def test_ignores_rejected_phase_rows(self) -> None:
        rows = [
            make_row(tow="174000", sat="G26", residual_m="0.040", phase_accepted="0"),
            make_row(tow="174030", sat="G26", residual_m="0.040", phase_accepted="0"),
        ]
        flagged = auto_exclude.collect_pairs(
            rows,
            tail_seconds=900.0,
            threshold_m=0.015,
            min_samples=2,
        )
        self.assertEqual(flagged, {})

    def test_ignores_code_rows(self) -> None:
        rows = [
            make_row(tow="174000", sat="G26", residual_m="0.400", row_type="code"),
            make_row(tow="174030", sat="G26", residual_m="0.400", row_type="code"),
        ]
        flagged = auto_exclude.collect_pairs(
            rows,
            tail_seconds=900.0,
            threshold_m=0.015,
            min_samples=2,
        )
        self.assertEqual(flagged, {})

    def test_samples_only_final_iteration(self) -> None:
        # Early iteration is noisy (40 mm), final iteration clean (1 mm).
        rows = [
            make_row(tow="174000", sat="G26", iteration="0", residual_m="0.040"),
            make_row(tow="174000", sat="G26", iteration="1", residual_m="0.001"),
            make_row(tow="174030", sat="G26", iteration="0", residual_m="0.040"),
            make_row(tow="174030", sat="G26", iteration="1", residual_m="0.001"),
        ]
        flagged = auto_exclude.collect_pairs(
            rows,
            tail_seconds=900.0,
            threshold_m=0.015,
            min_samples=2,
        )
        self.assertEqual(flagged, {})


class SelectByCnsatelliteTests(unittest.TestCase):
    def test_keeps_sibling_frequencies_of_top_satellites(self) -> None:
        flagged = {
            ("J02", 0): {"abs_mean_m": 0.038, "mean_m": 0.038, "samples": 30.0},
            ("J02", 1): {"abs_mean_m": 0.034, "mean_m": -0.034, "samples": 30.0},
            ("G02", 0): {"abs_mean_m": 0.022, "mean_m": 0.022, "samples": 30.0},
            ("G27", 0): {"abs_mean_m": 0.022, "mean_m": -0.022, "samples": 30.0},
        }
        # With ``max_satellites=1`` only J02 should survive, but both freqs.
        kept = auto_exclude.select_pairs_by_satellite(flagged, max_satellites=1)
        self.assertEqual(sorted(kept.keys()), [("J02", 0), ("J02", 1)])

    def test_zero_max_is_passthrough(self) -> None:
        flagged = {
            ("J02", 0): {"abs_mean_m": 0.038, "mean_m": 0.038, "samples": 30.0},
            ("G02", 0): {"abs_mean_m": 0.022, "mean_m": 0.022, "samples": 30.0},
        }
        kept = auto_exclude.select_pairs_by_satellite(flagged, max_satellites=0)
        self.assertEqual(kept, flagged)


class FormatAndCliTests(unittest.TestCase):
    def test_format_pair_csv_is_sorted(self) -> None:
        flagged = {
            ("J03", 0): {"abs_mean_m": 0.033, "mean_m": 0.033, "samples": 30.0},
            ("G16", 1): {"abs_mean_m": 0.015, "mean_m": 0.015, "samples": 30.0},
            ("G16", 0): {"abs_mean_m": 0.035, "mean_m": -0.035, "samples": 30.0},
        }
        self.assertEqual(
            auto_exclude.format_pair_csv(flagged),
            "G16:0,G16:1,J03:0",
        )

    def test_cli_round_trip(self) -> None:
        rows = [
            make_row(tow="174000", sat="J02", frequency_index="0", residual_m="0.038"),
            make_row(tow="174030", sat="J02", frequency_index="0", residual_m="0.038"),
            make_row(tow="174000", sat="J02", frequency_index="1", residual_m="-0.034"),
            make_row(tow="174030", sat="J02", frequency_index="1", residual_m="-0.034"),
            make_row(tow="174000", sat="G02", frequency_index="0", residual_m="0.008"),
            make_row(tow="174030", sat="G02", frequency_index="0", residual_m="0.008"),
        ]
        with tempfile.NamedTemporaryFile(
            mode="w", suffix=".csv", delete=False, newline=""
        ) as handle:
            writer = csv.DictWriter(handle, fieldnames=FIELDNAMES)
            writer.writeheader()
            for row in rows:
                writer.writerow(row)
            path = Path(handle.name)
        try:
            result = subprocess.run(
                [sys.executable, str(SCRIPT_PATH), str(path), "--min-samples", "2"],
                text=True,
                capture_output=True,
                check=False,
            )
            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertEqual(result.stdout.strip(), "J02:0,J02:1")
        finally:
            path.unlink()


if __name__ == "__main__":
    unittest.main()
