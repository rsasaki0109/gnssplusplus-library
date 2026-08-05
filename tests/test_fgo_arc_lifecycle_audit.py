#!/usr/bin/env python3

import importlib.util
import unittest
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "scripts" / "analyze_fgo_arc_lifecycle.py"
SPEC = importlib.util.spec_from_file_location("arc_lifecycle", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
AUDIT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AUDIT)


class FgoArcLifecycleAuditTest(unittest.TestCase):
    def test_reference_change_and_arc_churn_are_deterministic(self) -> None:
        candidates = {
            1000: [
                {"ambiguity_index": "10", "system": "1", "signal": "0", "reference_satellite": "G01"},
                {"ambiguity_index": "11", "system": "1", "signal": "0", "reference_satellite": "G01"},
            ],
            1200: [
                {"ambiguity_index": "12", "system": "1", "signal": "0", "reference_satellite": "G02"},
                {"ambiguity_index": "13", "system": "1", "signal": "0", "reference_satellite": "G02"},
            ],
        }
        lifecycle = AUDIT.candidate_lifecycle([1000, 1200], candidates)
        self.assertEqual(lifecycle[1]["reference_changes"], 1)
        self.assertEqual(lifecycle[1]["new_ambiguity_fraction"], 1.0)
        self.assertEqual(lifecycle[1]["ambiguity_churn_fraction"], 1.0)
        self.assertEqual(lifecycle[1]["minimum_arc_age_epochs"], 1)

    def test_exposure_never_uses_future_events(self) -> None:
        events = [set(), set(), {"clock_jump"}]
        self.assertFalse(AUDIT.exposure(events, 1, "clock_jump", 5))
        self.assertTrue(AUDIT.exposure(events, 2, "clock_jump", 0))

    def test_exposure_does_not_cross_epoch_gap(self) -> None:
        events = [{"clock_jump"}, set()]
        self.assertFalse(
            AUDIT.exposure(events, 1, "clock_jump", 5, [1000, 1400])
        )

    def test_epoch_gap_starts_fresh_arc_age(self) -> None:
        candidate = {
            "ambiguity_index": "10",
            "system": "1",
            "signal": "0",
            "reference_satellite": "G01",
        }
        lifecycle = AUDIT.candidate_lifecycle(
            [1000, 1200, 1600],
            {1000: [candidate], 1200: [candidate], 1600: [candidate]},
        )
        self.assertEqual(lifecycle[1]["minimum_arc_age_epochs"], 2)
        self.assertEqual(lifecycle[2]["minimum_arc_age_epochs"], 1)
        self.assertEqual(lifecycle[2]["reference_changes"], 0)
        self.assertEqual(lifecycle[2]["ambiguity_churn_fraction"], 0.0)

    def test_signed_common_clock_delta_detects_negative_jump(self) -> None:
        row = {
            "status": "FIXED",
            "e_err_m": "0",
            "n_err_m": "0",
            "u_err_m": "0",
            "clock_jump": "0",
            "clock_common_delta_m": "-299792.458",
            "clock_common_delta_satellites": "6",
        }
        lifecycle = [{
            "reference_changes": 0,
            "ambiguity_churn_fraction": 0.0,
            "eligible_ambiguities": 4,
            "minimum_arc_age_epochs": 3,
        }]
        _, events, _, _ = AUDIT.label_and_events([row], lifecycle)
        self.assertIn("clock_jump", events[0])

    def test_frozen_gate_requires_selectivity_and_support(self) -> None:
        size = 240
        wrong_fix = [False] * size
        correct_fix = [False] * size
        events = [set() for _ in range(size)]
        for index in range(20, 120, 5):
            wrong_fix[index] = True
            events[index].add("reference_change")
        for index in range(120, 240):
            correct_fix[index] = True
        summary = AUDIT.score(
            ["reference_change"], events, wrong_fix, correct_fix
        )
        self.assertEqual(summary["support"]["wrong_fix_onsets"], 20)
        self.assertTrue(summary["events"]["reference_change"]["gate_passed"])

    def test_markdown_handles_missing_wrong_onset_support(self) -> None:
        summary = AUDIT.score(["clock_jump"], [set()], [False], [True])
        rendered = AUDIT.render_markdown(summary)
        self.assertIn("n/a", rendered)
        self.assertIn("FAIL", rendered)


if __name__ == "__main__":
    unittest.main()
