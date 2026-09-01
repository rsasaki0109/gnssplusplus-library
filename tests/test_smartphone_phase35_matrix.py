from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase35_matrix.py"
SPEC = importlib.util.spec_from_file_location("phase35_matrix", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class Phase35MatrixTests(unittest.TestCase):
    def test_frozen_matrix_has_anchor_control_and_three_explicit_variants(self) -> None:
        self.assertEqual(
            tuple(MODULE.MATRIX_FLAGS),
            (
                "control_quality_anchor_base",
                "A_anchor_signal_iono_hatch",
                "B_A_plus_stop",
                "C_A_plus_position_offset",
            ),
        )
        self.assertIn("--native-quality-anchor", MODULE.MATRIX_FLAGS["control_quality_anchor_base"])
        self.assertNotIn("--native-signal-bias-states", MODULE.MATRIX_FLAGS["control_quality_anchor_base"])

    def test_A_contains_only_predeclared_fixed_elements(self) -> None:
        flags = MODULE.MATRIX_FLAGS["A_anchor_signal_iono_hatch"]
        for expected in (
            "--native-quality-anchor",
            "--native-signal-bias-states",
            "--native-residual-ionosphere",
            "--native-carrier-code-leveling",
            "--native-carrier-code-innovation-reset",
            "--native-carrier-code-gal-e1-e5a",
        ):
            self.assertIn(expected, flags)
        self.assertNotIn("--native-upstream-stop-constraints", flags)
        self.assertNotIn("--native-upstream-position-offset", flags)

    def test_B_and_C_differ_from_A_by_one_existing_flag(self) -> None:
        a = set(MODULE.MATRIX_FLAGS["A_anchor_signal_iono_hatch"])
        b = set(MODULE.MATRIX_FLAGS["B_A_plus_stop"])
        c = set(MODULE.MATRIX_FLAGS["C_A_plus_position_offset"])
        self.assertEqual(b - a, {"--native-upstream-stop-constraints"})
        self.assertEqual(c - a, {"--native-upstream-position-offset"})
        self.assertEqual(a - b, set())
        self.assertEqual(a - c, set())

    def test_cohort_is_route_disjoint_and_truth_free_before_structural_runs(self) -> None:
        freeze = MODULE.verify_freeze()
        cohort = freeze["cohort"]
        train = set(cohort["train_development_reuse"])
        validation = cohort["validation_reuse_after_train_gate"]
        holdout = cohort["future_holdout"]
        self.assertTrue(train.isdisjoint({validation, holdout}))
        self.assertEqual(freeze["truth_policy"]["phase35_truth_open_count_before_structural"], 0)
        self.assertFalse(freeze["truth_policy"]["mat_read_or_generated"])

    def test_all_inputs_are_raw_only(self) -> None:
        self.assertEqual(MODULE.RAW_NAMES, {"gnss": "device_gnss.csv", "imu": "device_imu.csv", "nav": "brdc.nav"})
        self.assertNotIn("ground_truth.csv", MODULE.RAW_NAMES.values())
        self.assertNotIn(".mat", " ".join(MODULE.RAW_NAMES.values()).lower())


if __name__ == "__main__":
    unittest.main()
