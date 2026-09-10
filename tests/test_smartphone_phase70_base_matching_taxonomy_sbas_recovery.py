#!/usr/bin/env python3
"""Input-free Phase70 tests for native RINEX system recognition."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
EVALUATOR = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase70_base_matching_taxonomy_sbas_recovery.py"


def load_evaluator():
    spec = importlib.util.spec_from_file_location("phase70_sbas_recovery", EVALUATOR)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load Phase70 evaluator")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def header_line(body: str, label: str) -> str:
    return body[:60].ljust(60) + label


def observation(value: float) -> str:
    return f"{value:14.3f}  "


def epoch(satellites: int = 2) -> str:
    return f"> 2021 03 16 18 59 00.0000000  0  {satellites:1d}"


class Phase70SbasRecoveryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_evaluator()

    def test_source_system_s_is_consumed_but_never_indexed(self):
        m = self.module
        header = "\n".join(
            (
                header_line("     3.03           OBSERVATION DATA    M (MIXED)", "RINEX VERSION / TYPE"),
                header_line("G  4   C1C L1C C5Q L5Q", "SYS / # / OBS TYPES"),
                header_line("S  2   C1C C5X", "SYS / # / OBS TYPES"),
                header_line("", "END OF HEADER"),
            )
        )
        g_record = "G01" + observation(20_000_000.0) + observation(1_000.0) + observation(21_000_000.0) + observation(2_000.0) + " " * 24
        s_record = "S31" + observation(20_000_010.0) + observation(20_000_011.0) + " " * 24
        index = m.parse_rinex((header + "\n" + epoch() + "\n" + g_record + "\n" + s_record + "\n").encode("ascii"))
        self.assertEqual(index.unselected_system_record_count, 1)
        self.assertEqual(index.unselected_system_census, {"SBAS": 1})
        self.assertNotIn(("SBAS", 31), index.satellite_streams)
        self.assertEqual(index.selected_rows, 2)
        self.assertIn(("GPS", 1), index.satellite_streams)

    def test_sbas_fixture_preserves_physical_record_boundary(self):
        m = self.module
        record = "S31" + observation(20_000_010.0) + observation(20_000_011.0) + " " * 24
        lines = [record, "> 2021 03 16 18 59 01.0000000  0  0"]
        physical, next_cursor = m.P69._record_lines(lines, 0, 2)
        self.assertEqual(next_cursor, 1)
        self.assertEqual(physical, [record])
        self.assertTrue(lines[next_cursor].startswith(">"))

    def test_proxy_disclaimer_and_prior_phase_policy_remain_explicit(self):
        source = EVALUATOR.read_text(encoding="utf-8")
        self.assertIn("not asserted equal to the native adopted FGO", source)
        self.assertIn("phase69_partial_output_reuse", source)
        self.assertIn("phase68_partial_output_reuse", source)


if __name__ == "__main__":
    unittest.main()
