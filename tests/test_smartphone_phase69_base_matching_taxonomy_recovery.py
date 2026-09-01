#!/usr/bin/env python3
"""Input-free fixtures for the Phase69 RINEX3 framing recovery."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
EVALUATOR = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase69_base_matching_taxonomy_recovery.py"


def load_evaluator():
    spec = importlib.util.spec_from_file_location("phase69_taxonomy_recovery", EVALUATOR)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load Phase69 evaluator")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def header_line(body: str, label: str) -> str:
    return body[:60].ljust(60) + label


def observation(value: float) -> str:
    return f"{value:14.3f}  "


def epoch(second: int, satellites: int = 1) -> str:
    return f"> 2021 03 16 18 59 {second:02d}.0000000  0  {satellites:1d}"


class Phase69RinexFramingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_evaluator()

    def header(self, count: int, types: str) -> str:
        return "\n".join(
            (
                header_line("     3.03           OBSERVATION DATA    M (MIXED)", "RINEX VERSION / TYPE"),
                header_line(f"G  {count:1d}   {types}", "SYS / # / OBS TYPES"),
                header_line("", "END OF HEADER"),
            )
        )

    def test_long_single_line_record_does_not_consume_next_epoch_marker(self):
        m = self.module
        payload = (
            self.header(4, "C1C L1C C5Q L5Q")
            + "\n"
            + epoch(0)
            + "\n"
            + "G01" + observation(20_000_000.0) + observation(1_000.0) + observation(21_000_000.0) + observation(2_000.0) + " " * 24
            + "\n"
            + epoch(1)
            + "\n"
            + "G01" + observation(20_000_001.0) + observation(1_001.0) + observation(21_000_001.0) + observation(2_001.0) + " " * 24
            + "\n"
        ).encode("ascii")
        index = m.parse_rinex(payload)
        self.assertEqual(index.selected_rows, 4)
        self.assertEqual(index.time_min_s, index.time_max_s - 1.0)
        self.assertEqual(len(index.exact_streams[("GPS", 1, "GPS_L1CA")]), 2)

    def test_standard_continuation_record_is_consumed_exactly(self):
        m = self.module
        payload = (
            self.header(6, "C1C L1C C5Q L5Q C2W L2W")
            + "\n"
            + epoch(0)
            + "\n"
            + "G01" + observation(20_000_000.0) + observation(1_000.0) + observation(21_000_000.0) + observation(2_000.0)
            + "\n"
            + "   " + observation(20_000_002.0) + observation(3_000.0)
            + "\n"
        ).encode("ascii")
        index = m.parse_rinex(payload)
        # The taxonomy indexes finite code observables only; phase and
        # lock-strength fields remain part of the physical record but are not
        # pseudorange matching streams.
        self.assertEqual(index.base_rows, 3)
        self.assertEqual(index.finite_code_rows, 3)
        self.assertEqual(index.selected_rows, 2)
        self.assertEqual(index.signal_census["GPS_L1CA"], 1)
        self.assertEqual(index.signal_census["GPS_L5"], 1)

    def test_premature_epoch_marker_is_rejected_for_standard_record(self):
        m = self.module
        with self.assertRaises(m.Phase69Error):
            m._record_lines(["G01" + observation(20_000_000.0), epoch(1)], 0, 6)

    def test_proxy_scope_is_explicit(self):
        source = EVALUATOR.read_text(encoding="utf-8")
        self.assertIn("not asserted equal to the native adopted FGO", source)
        self.assertIn("phase68_partial_output_reuse", source)


if __name__ == "__main__":
    unittest.main()
