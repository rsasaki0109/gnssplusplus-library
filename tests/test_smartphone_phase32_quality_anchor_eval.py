from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase32_quality_anchor_eval.py"
SPEC = importlib.util.spec_from_file_location("phase32_quality_anchor_eval", SCRIPT)
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
        "vertical_p95_abs_m": None,
    }


class Phase32QualityAnchorEvaluationTests(unittest.TestCase):
    def test_vertical_unavailable_is_informational_for_route_gate(self) -> None:
        gate = MODULE._horizontal_compare(metric(1.0), metric(2.0))
        self.assertTrue(gate["passed"])
        self.assertEqual(
            gate["vertical"]["status"],
            "informational-unavailable-four-column-prediction",
        )

    def test_route_gate_keeps_availability_and_continuity_non_regressed(self) -> None:
        gate = MODULE._horizontal_compare(
            metric(1.0, p50=1.0, p95=2.0, over_70=1),
            metric(2.0, p50=1.1, p95=2.1, over_70=1),
        )
        self.assertTrue(gate["passed"])
        rejected = MODULE._horizontal_compare(
            metric(1.0, over_70=2), metric(2.0, over_70=1)
        )
        self.assertFalse(rejected["passed"])
        self.assertIn("continuity_regression", rejected["failures"])

    def test_macro_requires_strict_improvement_in_every_variant(self) -> None:
        control = MODULE._aggregate([metric(2.0), metric(2.0)])
        candidate = MODULE._aggregate([metric(1.0), metric(1.0)])
        self.assertTrue(MODULE._aggregate_compare(candidate, control)["passed"])

        equal_one_variant = MODULE._aggregate([metric(1.0), metric(1.0)])
        equal_one_variant["mean_kaggle_diagnostic_score_variants_m"][
            MODULE.DIAGNOSTIC_KEYS[0]
        ] = control["mean_kaggle_diagnostic_score_variants_m"][MODULE.DIAGNOSTIC_KEYS[0]]
        gate = MODULE._aggregate_compare(equal_one_variant, control)
        self.assertFalse(gate["passed"])
        self.assertTrue(
            any(name.endswith("_not_strictly_improved") for name in gate["failures"])
        )


if __name__ == "__main__":
    unittest.main()
