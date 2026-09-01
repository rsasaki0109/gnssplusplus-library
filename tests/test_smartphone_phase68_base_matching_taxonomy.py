#!/usr/bin/env python3
"""Focused, input-free tests for the Phase68 matching taxonomy."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
EVALUATOR = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase68_base_matching_taxonomy.py"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase68_base_matching_taxonomy_freeze_v1.json"


def load_evaluator():
    spec = importlib.util.spec_from_file_location("phase68_matching_taxonomy", EVALUATOR)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load Phase68 evaluator")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def header_line(body: str, label: str) -> str:
    return body[:60].ljust(60) + label


class Phase68MatchingTaxonomyTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.module = load_evaluator()

    def test_rinex3_observation_type_continuation_keeps_active_system(self):
        lines = [
            header_line("     3.03           OBSERVATION DATA    M (MIXED)", "RINEX VERSION / TYPE"),
            header_line("G  6   C1C L1C C5Q L5Q", "SYS / # / OBS TYPES"),
            header_line("       C2W L2W", "SYS / # / OBS TYPES"),
            header_line("", "END OF HEADER"),
        ]
        _, version, systems, _, _ = self.module.parse_header(lines)
        self.assertEqual(version, 3.03)
        self.assertEqual(systems["G"], ["C1C", "L1C", "C5Q", "L5Q", "C2W", "L2W"])
        self.assertNotIn("", systems)

    def test_classification_is_exhaustive_and_precedence_is_frozen(self):
        m = self.module
        raw = m.RawReport(
            adopted_rows=[
                m.RawAdoptedRow(100.5, 1, "GPS", 1, "GPS_L1CA", "GPS_L1", 1575.42e6),
                m.RawAdoptedRow(100.5, 2, "GPS", 2, "GPS_L1CA", "GPS_L1", 1575.42e6),
                m.RawAdoptedRow(99.0, 3, "GPS", 3, "GPS_L5", "GPS_L5", 1176.45e6),
                m.RawAdoptedRow(100.5, 4, "Galileo", 4, "GAL_E5A", "GAL_E5A", 1176.45e6),
                m.RawAdoptedRow(100.5, 5, "GLONASS", 5, "GLO_L1CA", "GLO_L1_CH0", 1602.0e6),
            ]
        )
        base = m.BaseIndex(
            exact_streams={
                ("GPS", 1, "GPS_L1CA"): [100.0, 101.0],
                ("GPS", 3, "GPS_L5"): [100.0, 101.0],
            },
            frequency_streams={
                ("GPS", 1, "GPS_L1"): [100.0, 101.0],
                ("GPS", 2, "GPS_L1"): [100.0, 101.0],
                ("GPS", 3, "GPS_L5"): [100.0, 101.0],
                ("Galileo", 4, "GAL_E1"): [100.0, 101.0],
            },
            satellite_streams={
                ("GPS", 1): {"GPS_L1"},
                ("GPS", 2): {"GPS_L1"},
                ("GPS", 3): {"GPS_L5"},
                ("Galileo", 4): {"GAL_E1"},
            },
        )
        classification = m.classify_rows(raw, base)
        self.assertEqual(
            classification["counts"],
            {
                "exact_signal_in_domain": 1,
                "same_frequency_variant_in_domain": 1,
                "out_of_domain_time": 1,
                "missing_frequency_stream": 1,
                "missing_satellite_stream": 1,
            },
        )
        self.assertEqual(classification["adopted_rows"], 5)
        self.assertTrue(classification["sum_check"])

    def test_duplicate_canonical_frequency_is_report_only(self):
        m = self.module
        index = m.BaseIndex()
        entries = [
            ("GPS", 7, "C1C", m.SignalInfo("GPS", "GPS_L1CA", "GPS_L1", 1), 20_000_000.0),
            ("GPS", 7, "C1W", m.SignalInfo("GPS", "GPS_L1P", "GPS_L1", 1), 20_000_001.0),
            ("GPS", 7, "C5Q", m.SignalInfo("GPS", "GPS_L5", "GPS_L5", 5), 21_000_000.0),
        ]
        m.add_base_epoch(index, 100.0, entries)
        self.assertEqual(index.duplicate_epoch_frequency_events, 1)
        self.assertEqual(index.duplicate_epoch_frequency_rows, 1)
        self.assertEqual(index.selected_rows, 2)
        self.assertEqual(len(index.frequency_streams), 2)

    def test_freeze_declares_proxy_not_native_factor_equality(self):
        freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        self.assertIn("MessageType=Raw", freeze["source_matching_contract"]["raw_adopted_proxy"])
        self.assertFalse(freeze["read_contract"]["solver_invocations"])
        source = EVALUATOR.read_text(encoding="utf-8")
        self.assertIn("not asserted", source)
        self.assertIn("native adopted FGO", source)


if __name__ == "__main__":
    unittest.main()
