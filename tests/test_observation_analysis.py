"""Independent clock and carrier witnesses for Python observation diagnostics."""
from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("observation_diagnostics", ROOT / "python/libgnsspp/observations.py")
diagnostics = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(diagnostics)


def solution(tow, *, week=2324, valid=True):
    return SimpleNamespace(time=SimpleNamespace(week=week, tow=tow), position_ecef_m=(6378137.0, 0, 0), is_valid=lambda: valid)


def measurement(satellite="G01", *, phase=100.0, doppler=-3.0, code=20200100.0,
                group=1, weight=1.0, frequency=diagnostics.C, signal=0, tracking="L1C", lli=0):
    return SimpleNamespace(satellite_id=satellite, signal_id=signal, carrier_observation_type=tracking,
                           carrier_frequency_hz=frequency, clock_group=group, snr=40.0, elevation=0.5,
                           carrier_phase=phase, doppler=doppler, corrected_pseudorange=code,
                           satellite_ecef=(26578137.0, 0, 0), weight=weight, variance=1.0 / weight,
                           satellite_velocity=(0, 3000.0, 0), satellite_clock_drift=1e-12,
                           ionosphere_free=False, loss_of_lock_indicator=lli, source_loss_of_lock=False)


class ObservationAnalysisTest(unittest.TestCase):
    def test_returning_tracking_code_cannot_reuse_an_old_arc(self):
        rows, _ = diagnostics.analyze_epochs([
            (solution(1), [measurement()]),
            (solution(1.2), [measurement(phase=1000, tracking="L1W")]),
            (solution(1.4), [measurement(phase=9000, tracking="L1C")]),
            (solution(1.6), [measurement(phase=math.nan, tracking="")]),
            (solution(1.8), [measurement(phase=20000, tracking="L1C")])])
        self.assertFalse(any(row["slip_suspect"] for row in rows))
        self.assertIn("arc_start", rows[2]["reason_codes"])
        self.assertIn("arc_start", rows[4]["reason_codes"])

    def test_multiple_signals_do_not_invent_independent_clock_witnesses(self):
        first = [measurement(f"G{i:02}", signal=signal) for i in range(1, 3) for signal in (0, 1)]
        second = [measurement(f"G{i:02}", phase=123, signal=signal) for i in range(1, 3) for signal in (0, 1)]
        _, summary = diagnostics.analyze_epochs([(solution(1), first), (solution(2), second)])
        self.assertEqual(summary["clock_step_candidate_rows"], 0)
        self.assertEqual(summary["slip_suspect_rows"], 4)

    def test_nonfinite_optional_motion_and_elevation_are_json_null(self):
        row = measurement()
        row.satellite_velocity = (math.nan, 0, 0)
        row.elevation = math.inf
        rows, summary = diagnostics.analyze_epochs([(solution(1), [row])])
        self.assertIsNone(rows[0]["satellite_speed_mps"])
        self.assertIsNone(rows[0]["elevation_deg"])
        json.dumps({"rows": rows, "summary": summary}, allow_nan=False)

    def test_weighted_clock_removal_keeps_constellation_offsets_separate(self):
        rows = [measurement("G01", code=20200100, weight=1), measurement("G02", code=20200104, weight=3),
                measurement("E01", code=20200200, group=4), measurement("E02", code=20200202, group=4)]
        records, summary = diagnostics.analyze_epochs([(solution(1), rows)])
        self.assertEqual([row["clock_removed_code_residual_m"] for row in records], [-3, 1, -1, 1])
        self.assertEqual([row["fitted_clock_bias_m"] for row in records], [103, 103, 201, 201])
        self.assertEqual(summary["valid_solution_epochs"], 1)
        json.dumps({"records": records, "summary": summary}, allow_nan=False)

    def test_rinex_doppler_sign_and_single_satellite_jump(self):
        first = [measurement(f"G{i:02}") for i in range(1, 5)]
        second = [measurement(f"G{i:02}", phase=123 if i == 1 else 103) for i in range(1, 5)]
        rows, summary = diagnostics.analyze_epochs([(solution(1), first), (solution(2), second)])
        self.assertEqual([row["phase_doppler_raw_cycles"] for row in rows[4:]], [20, 0, 0, 0])
        self.assertEqual([row["slip_suspect"] for row in rows[4:]], [True, False, False, False])
        self.assertEqual(summary["slip_suspect_rows"], 1)
        self.assertEqual(summary["clock_step_candidate_rows"], 0)

    def test_common_clock_step_in_metres_across_different_frequencies(self):
        first = [measurement(f"G{i:02}", frequency=diagnostics.C if i <= 2 else diagnostics.C / 2) for i in range(1, 5)]
        second = [measurement(f"G{i:02}", phase=123 if i <= 2 else 113,
                              frequency=diagnostics.C if i <= 2 else diagnostics.C / 2) for i in range(1, 5)]
        rows, summary = diagnostics.analyze_epochs([(solution(1), first), (solution(2), second)])
        self.assertEqual(summary["clock_step_candidate_rows"], 4)
        self.assertEqual(summary["slip_suspect_rows"], 0)
        self.assertTrue(all(row["phase_doppler_adjusted_cycles"] == 0 for row in rows[4:]))
        self.assertTrue(all(row["common_clock_step_m"] == 20 for row in rows[4:]))

    def test_sparse_common_jump_is_not_silently_exonerated(self):
        first = [measurement(f"G{i:02}") for i in range(1, 4)]
        second = [measurement(f"G{i:02}", phase=123) for i in range(1, 4)]
        _, summary = diagnostics.analyze_epochs([(solution(1), first), (solution(2), second)])
        self.assertEqual(summary["slip_suspect_rows"], 3)
        self.assertEqual(summary["clock_step_candidate_rows"], 0)

    def test_lock_flag_is_visible_even_with_small_phase_residual(self):
        rows, summary = diagnostics.analyze_epochs([(solution(1), [measurement()]),
                                                   (solution(2), [measurement(phase=103, lli=1)])])
        self.assertTrue(rows[-1]["slip_suspect"])
        self.assertIn("source_loss_of_lock", rows[-1]["reason_codes"])
        self.assertEqual(summary["reason_counts"]["source_loss_of_lock"], 1)

    def test_missing_carrier_and_doppler_do_not_manufacture_slip_evidence(self):
        rows, summary = diagnostics.analyze_epochs([
            (solution(1), [measurement()]), (solution(2), [measurement(phase=math.nan)]),
            (solution(3), [measurement(phase=106)]), (solution(4), [measurement(phase=109, doppler=math.nan)])])
        self.assertFalse(any(row["slip_suspect"] for row in rows))
        self.assertIn("arc_start", rows[2]["reason_codes"])
        self.assertIn("doppler_unavailable", rows[3]["reason_codes"])
        self.assertEqual(summary["phase_doppler_assessed_rows"], 0)
        json.dumps(rows, allow_nan=False)

    def test_gap_or_tracking_change_resets_the_carrier_comparison(self):
        rows, _ = diagnostics.analyze_epochs([(solution(1), [measurement()]),
                                             (solution(5), [measurement(phase=500)]),
                                             (solution(6), [measurement(phase=900, tracking="L1W")])])
        self.assertIn("gap_reset", rows[1]["reason_codes"])
        self.assertIn("arc_start", rows[2]["reason_codes"])
        self.assertFalse(any(row["slip_suspect"] for row in rows))

    def test_invalid_solution_does_not_claim_code_residual_accuracy(self):
        rows, summary = diagnostics.analyze_epochs([(solution(1, valid=False), [measurement("G01"), measurement("G02")])])
        self.assertTrue(all(row["clock_removed_code_residual_m"] is None for row in rows))
        self.assertEqual(summary["valid_solution_epochs"], 0)

    def test_single_clock_row_has_no_identifiable_satellite_residual(self):
        rows, _ = diagnostics.analyze_epochs([(solution(1), [measurement()])])
        self.assertIsNone(rows[0]["clock_removed_code_residual_m"])
        self.assertIn("clock_group_too_small", rows[0]["reason_codes"])

    def test_time_rollover_works_but_duplicate_epoch_or_row_is_rejected(self):
        rows, _ = diagnostics.analyze_epochs([(solution(604799), [measurement()]),
                                             (solution(0, week=2325), [measurement(phase=103)])])
        self.assertEqual(rows[1]["phase_doppler_raw_cycles"], 0)
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            diagnostics.analyze_epochs([(solution(1), [measurement()]), (solution(1), [measurement()])])
        with self.assertRaisesRegex(ValueError, "duplicate"):
            diagnostics.analyze_epochs([(solution(1), [measurement(), measurement()])])

    def test_invalid_settings_and_empty_analysis_fail(self):
        for args in ({"max_gap_s": 0}, {"slip_threshold_cycles": math.nan}, {"min_clock_witnesses": 3}):
            with self.assertRaises(ValueError):
                diagnostics.analyze_epochs([], **args)
        with self.assertRaisesRegex(ValueError, "no corrected"):
            diagnostics.analyze_epochs([])


if __name__ == "__main__":
    unittest.main()
