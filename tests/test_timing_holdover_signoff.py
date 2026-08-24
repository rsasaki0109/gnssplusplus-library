"""Tests for R9 clock comparison and simulated holdover."""

from __future__ import annotations

import csv
from pathlib import Path
import sys
import tempfile
import unittest


ROOT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT_DIR / "apps/commands"))
sys.path.insert(0, str(ROOT_DIR / "apps/commands/benchmarks"))

import gnss_timing_holdover_signoff as signoff  # noqa: E402


class TimingHoldoverSignoffTest(unittest.TestCase):
    def test_calendar_key_matches_known_gps_week_tow(self) -> None:
        self.assertEqual(signoff.calendar_gps_key(2024, 1, 1, 0, 0, 0.0), signoff.gps_key(2295, 86400.0))

    def test_lock_coverage_uses_the_processed_clock_window(self) -> None:
        clock = {
            index * 3000: {"bias_s": index * 1e-9}
            for index in range(12)
        }
        reference = {
            index * 3000: {"bias_s": index * 1e-9, "sigma_s": 1e-11}
            for index in range(20)
        }
        result = signoff.analyse(clock, reference)
        self.assertEqual(result["reference_epochs"], 12)
        self.assertEqual(result["lock_coverage_pct"], 100.0)

    def test_reference_reader_requires_named_station(self) -> None:
        with tempfile.TemporaryDirectory(prefix="r9_clk_") as temp_dir:
            path = Path(temp_dir) / "sample.clk"
            path.write_text("AR TEST 2024 01 01 00 00  0.000000  2  1.0e-8  1.0e-11\n", encoding="ascii")
            self.assertEqual(len(signoff.read_reference_clk(path, "TEST")), 1)
            with self.assertRaisesRegex(ValueError, "absent"):
                signoff.read_reference_clk(path, "NONE")

    def test_candidate_thresholds_keep_service_no_go(self) -> None:
        result = {
            "lock_coverage_pct": 100.0,
            "lock_phase_error": {"p95_abs_ns": 10.1},
            "maximum_lock_step_ns": 3.2,
            "simulated_holdover": {
                str(horizon): {"p95_abs_error_ns": 10.0 + index}
                for index, horizon in enumerate(signoff.HORIZONS_S)
            },
        }
        profile = signoff.build_candidate_profile(result)
        self.assertEqual(profile["maximum_lock_phase_p95_abs_ns"], 16)
        self.assertTrue(profile["service_decision"].startswith("no_go"))
        self.assertIn("no physical PPS", profile["claim_boundary"])

    def test_enforcement_rejects_missing_holdover_population(self) -> None:
        result = {
            "lock_coverage_pct": 100.0,
            "lock_phase_error": {"p95_abs_ns": 1.0},
            "maximum_lock_step_ns": 1.0,
            "simulated_holdover": {},
        }
        profile = {
            "minimum_lock_coverage_pct": 95.0,
            "maximum_lock_phase_p95_abs_ns": 2.0,
            "maximum_lock_step_ns": 2.0,
            "maximum_holdover_p95_abs_ns": {"300": 5.0},
        }
        self.assertIn("holdover_300s_population_missing", signoff.enforce(result, profile))

    def test_native_spp_exposes_clock_csv_without_changing_pos_schema(self) -> None:
        source = (ROOT_DIR / "apps/native/gnss_spp.cpp").read_text(encoding="utf-8")
        self.assertIn("--clock-csv <clock.csv>", source)
        self.assertIn("receiver_clock_bias_s", source)
        position_writer = (ROOT_DIR / "src/core/solution.cpp").read_text(encoding="utf-8")
        self.assertNotIn("receiver_clock_bias_s", position_writer)


if __name__ == "__main__":
    unittest.main()
