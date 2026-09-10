from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase34_quality_anchor_validation_eval.py"
SPEC = importlib.util.spec_from_file_location("phase34_quality_anchor_validation_eval", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def metric(value: float, *, p50: float = 1.0, p95: float = 2.0, over_70: int = 0) -> dict:
    return {
        "kaggle_diagnostic_score_variants_m": {
            key: value for key in MODULE.DIAGNOSTIC_KEYS
        },
        "horizontal_wgs84_m": {"p50_m": p50, "p95_m": p95},
        "availability_ratio": 1.0,
        "truth_coverage_ratio": 1.0,
        "continuity": {"over_70_mps_count": over_70},
    }


class Phase34QualityAnchorValidationEvalTests(unittest.TestCase):
    def test_all_four_strict_improvement_is_required(self) -> None:
        gate = MODULE._strict_horizontal_compare(metric(1.0), metric(2.0, p50=2.0, p95=3.0, over_70=3))
        self.assertTrue(gate["passed"])
        self.assertEqual(set(gate["strictly_improved_variants"]), set(MODULE.DIAGNOSTIC_KEYS))
        self.assertTrue(gate["control_continuity_is_diagnostic_only"])

    def test_equal_variant_fails_strict_gate(self) -> None:
        control = metric(2.0, p50=2.0, p95=3.0)
        candidate = metric(1.0, p50=1.0, p95=2.0)
        candidate["kaggle_diagnostic_score_variants_m"][MODULE.DIAGNOSTIC_KEYS[0]] = 2.0
        gate = MODULE._strict_horizontal_compare(candidate, control)
        self.assertFalse(gate["passed"])
        self.assertIn("not_all_four_diagnostics_strictly_improved", gate["failures"])

    def test_control_continuity_does_not_veto_safe_candidate(self) -> None:
        gate = MODULE._strict_horizontal_compare(
            metric(1.0, p50=1.0, p95=2.0, over_70=0),
            metric(2.0, p50=2.0, p95=3.0, over_70=3),
        )
        self.assertTrue(gate["passed"])
        self.assertNotIn("control_continuity_regression", gate["failures"])

    def test_candidate_continuity_is_mandatory(self) -> None:
        gate = MODULE._strict_horizontal_compare(metric(1.0, over_70=1), metric(2.0, p50=2.0, p95=3.0))
        self.assertFalse(gate["passed"])
        self.assertIn("candidate_continuity_not_safe", gate["failures"])

    def test_horizontal_regression_and_coverage_fail_closed(self) -> None:
        candidate = metric(1.0, p50=3.0, p95=4.0)
        candidate["availability_ratio"] = 0.9
        candidate["truth_coverage_ratio"] = 0.9
        gate = MODULE._strict_horizontal_compare(candidate, metric(2.0, p50=2.0, p95=3.0))
        self.assertFalse(gate["passed"])
        self.assertIn("h_p50_m_regression", gate["failures"])
        self.assertIn("h_p95_m_regression", gate["failures"])
        self.assertIn("availability_ratio_regression", gate["failures"])
        self.assertIn("truth_coverage_ratio_regression", gate["failures"])


if __name__ == "__main__":
    unittest.main()
