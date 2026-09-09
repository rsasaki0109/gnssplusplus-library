from __future__ import annotations

from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands"))
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_native_fgo_convergence_selector_eval as SELECTOR  # noqa: E402


class NativeFgoConvergenceSelectorTests(unittest.TestCase):
    def test_candidate_is_selected_only_for_complete_converged_contract(self) -> None:
        summary = {
            "converged": True,
            "iterations": 29,
            "pseudorange_factors": 10,
            "tdcp_factors": 9,
            "motion_factors": 8,
            "single_difference_doppler_factors": 0,
            "single_difference_tdcp_factors": 0,
            "carrier_phase_factors": 0,
            "double_difference_pseudorange_factors": 0,
            "double_difference_carrier_factors": 0,
        }
        trace = {"converged_rows": 1, "monotonic_non_increasing": True}
        keys = {"exact_key_coverage": True, "finite_coordinates": True}
        continuity = {"physical_continuity_passed": True}
        reasons = SELECTOR.candidate_rejection_reasons(summary, trace, keys, continuity)
        self.assertEqual(reasons, [])
        self.assertEqual(SELECTOR.choose_lane(reasons), "candidate50")

    def test_candidate_failure_falls_back_to_baseline(self) -> None:
        summary = {
            "converged": False,
            "iterations": 50,
            "pseudorange_factors": 10,
            "tdcp_factors": 9,
            "motion_factors": 8,
            "single_difference_doppler_factors": 0,
            "single_difference_tdcp_factors": 0,
            "carrier_phase_factors": 0,
            "double_difference_pseudorange_factors": 0,
            "double_difference_carrier_factors": 0,
        }
        reasons = SELECTOR.candidate_rejection_reasons(
            summary,
            {"converged_rows": 0, "monotonic_non_increasing": False},
            {"exact_key_coverage": True, "finite_coordinates": True},
            {"physical_continuity_passed": True},
        )
        self.assertIn("candidate_summary_not_converged", reasons)
        self.assertIn("candidate_did_not_terminate_before_max_iterations", reasons)
        self.assertIn("candidate_trace_has_no_convergence_row", reasons)
        self.assertIn("candidate_cost_not_monotonic_non_increasing", reasons)
        self.assertEqual(SELECTOR.choose_lane(reasons), "baseline8")

    def test_cost_trace_requires_ordered_finite_monotonic_costs(self) -> None:
        header = "global_iteration,cost,converged\n"
        with tempfile.TemporaryDirectory(prefix="convergence_selector_trace_") as raw:
            path = Path(raw) / "trace.csv"
            path.write_text(header + "0,10,0\n1,9,1\n", encoding="ascii")
            stats = SELECTOR.cost_trace_stats(path, 50)
            self.assertTrue(stats["monotonic_non_increasing"])
            self.assertEqual(stats["converged_rows"], 1)

            path.write_text(header + "1,9,0\n0,10,1\n", encoding="ascii")
            with self.assertRaises(SELECTOR.SelectorError):
                SELECTOR.cost_trace_stats(path, 50)

            path.write_text(header + "0,10,0\n1,11,1\n", encoding="ascii")
            self.assertFalse(SELECTOR.cost_trace_stats(path, 50)["monotonic_non_increasing"])

    def test_atomic_copy_preserves_exact_baseline_bytes(self) -> None:
        with tempfile.TemporaryDirectory(prefix="convergence_selector_copy_") as raw:
            root = Path(raw)
            source = root / "baseline.pos"
            target = root / "selected" / "fgo.pos"
            source.write_bytes(b"finite,baseline,bytes\n")
            selected = SELECTOR.atomic_copy(source, target)
            self.assertEqual(target.read_bytes(), source.read_bytes())
            self.assertEqual(selected["sha256"], SELECTOR.sha256(source))


if __name__ == "__main__":
    unittest.main()
