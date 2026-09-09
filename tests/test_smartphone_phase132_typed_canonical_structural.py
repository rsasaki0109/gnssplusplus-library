"""Launch-free Phase132 structural-contract tests.

Every inventory and summary is synthetic in-memory data.  No raw payload,
solution row, truth, MAT, PDC, precomputed coordinate, native solver, or
Kaggle artifact is opened or launched.
"""

from __future__ import annotations

from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps/commands/benchmarks"))
import gnss_smartphone_phase132_typed_canonical_structural as contract  # noqa: E402


def passing_inventory(route: str) -> dict:
    rover = {
        "input_rows": 9, "certified_rows": 6, "explicit_local_miss_rows": 3,
        "all_rows_classified": True, "no_raw_uncorrected": True,
        "no_zero_correction": True, "no_fallback": True,
        "no_extrapolation": True, "finite_certified_wavelength": True,
        "canonical_key_accounted": True, "literal_provenance_retained": True,
        "canonical": {
            "all_rows_accounted": True,
            "same_physical_family_aliases_only": True,
            "different_family_rejected": True, "unknown_band_explicit_miss": True,
            "literal_tracking_code_in_join": False,
            "original_signal_provenance_retained": True,
            "fcn_certification_fail_closed": True,
        },
    }
    base = {
        "input_rows": 14, "certified_rows": 10, "explicit_local_miss_rows": 4,
        "all_rows_classified": True, "no_raw_uncorrected": True,
        "no_zero_correction": True, "no_fallback": True,
        "no_extrapolation": True, "finite_certified_wavelength": True,
        "canonical_key_accounted": True, "literal_provenance_retained": True,
        "canonical": {
            "all_rows_accounted": True,
            "same_physical_family_aliases_only": True,
            "different_family_rejected": True, "unknown_band_explicit_miss": True,
            "literal_tracking_code_in_join": False,
            "original_signal_provenance_retained": True,
            "fcn_certification_fail_closed": True,
        },
    }
    return {
        "route": route, "stage": "post-independent-authorization-pre-solver",
        "solver_invocations": 0, "solver_may_start": True,
        "rover": rover, "base": base,
        "shared_ledger": {
            "side_local_conservation": True, "canonical_key_conservation": True,
            "same_physical_family_aliases_only": True,
            "different_physical_band_rejected": True,
            "literal_tracking_code_in_join": False,
            "original_signal_provenance_retained": True,
            "certified_glonass_fcn_only": True,
            "ambiguous_multi_code_fail_closed": True,
            "duplicate_conflict_fail_closed": True,
            "retained_rover_exact_key_support": True,
            "unmatched_rover_explicit_factor_miss": True,
            "unused_base_rows_accounted": True,
            "whole_ledger_equality_required": False,
            "cross_side_count_equality_required": False,
            "no_raw_uncorrected": True, "no_zero_correction": True,
            "no_fallback": True, "no_extrapolation": True,
        },
        "base_streams": {
            "finite_samples": 8, "used_rows": 5, "unused_rows": 3,
            "unused_streams": 1, "all_samples_accounted": True,
            "duplicate_nonmonotonic_rejected": True,
            "canonical_key_streams_accounted": True,
        },
        "correction": {
            "factor_input_rows": 8, "retained_corrected_rows": 5,
            "missing_exact_stream": 2, "out_of_domain": 1, "nonfinite": 0,
            "support": {"exact_endpoint_rows": 1,
                        "adjacent_two_point_bracket_rows": 3,
                        "exact_interior_sample_rows": 1},
            "retained_factors_have_exact_key_support": True,
            "one_support_result_per_retained_factor": True,
            "unused_base_rows_accounted": True, "application_passes": 1,
            "exactly_once": True, "no_duplicate_application": True,
            "no_raw_fallback": True, "no_zero_fallback": True,
            "no_nearest_fill": True, "no_endpoint_hold": True,
            "no_extrapolation": True, "finite_corrected_rows": True,
            "canonical_key_mode": True, "factor_topology_changed": False,
        },
        "phase132": {
            "enabled": True, "python_preflight_started": True,
            "python_preflight_executed": True, "python_preflight_call_count": 1,
            "native_selector_forwarded": False, "native_command_constructed": False,
            "native_binary_invocation_attempted": False,
            "native_resolver_executed": False, "native_resolver_call_count": 0,
            "native_resolver_evidence_source": "not-launched",
            "source_complete_a_b_c": True, "canonical_key_count": 2,
        },
    }


def passing_summary(route: str) -> dict:
    return {
        "route": route, "solution_content_read": False,
        "fallback_used": False, "rerun_count": 0,
        "gnss_first": {"accepted_iterations": 2, "initial_cost": 10.0,
                       "final_cost": 7.0},
        "main": {"accepted_iterations": 3, "initial_cost": 9.0,
                 "final_cost": 6.0, "solver_branch": "MULTIFRONTAL_QR"},
        "phase132": {
            "enabled": True, "python_preflight_executed": True,
            "native_selector_forwarded": True, "native_command_constructed": True,
            "native_binary_invocation_attempted": True,
            "native_resolver_executed": True,
            "native_resolver_call_count": 5,
            "native_resolver_evidence_source": "native-summary",
            "literal_tracking_code_in_join": False,
            "original_signal_provenance_retained": True,
            "side_local_conservation": True, "canonical_key_conservation": True,
            "retained_rover_exact_key_support": True,
            "unused_base_rows_accounted": True, "no_fallback": True,
            "factor_topology_changed": False,
        },
        "gates": {"canonical_join": True, "factor_support": True,
                  "finite_coverage": True},
        "opaque_solution": {"sha256": "a" * 64, "rows": 10},
    }


class Phase132TypedCanonicalStructuralTests(unittest.TestCase):
    def test_aliases_share_physical_family_but_not_band(self) -> None:
        ca = contract.canonicalize_typed("GLONASS", 7, "L1", -4)
        cp = contract.canonicalize_typed("GLONASS", 7, "L1", -4)
        self.assertTrue(ca["accepted"])
        self.assertEqual(ca["key"], cp["key"])
        self.assertNotEqual(ca["key"], contract.canonicalize_typed(
            "GLONASS", 7, "L5", -4)["key"])
        self.assertEqual(contract.typed_family("GLONASS", "GLO_G1_CA"), "L1")
        self.assertEqual(contract.typed_family("GLONASS", "GLO_L1P"), "L1")
        self.assertEqual(contract.rinex_family("GLONASS", "C1C"), "L1")
        self.assertIsNone(contract.rinex_family("GLONASS", "C2P"))

    def test_fcn_missing_and_bounds_fail_closed(self) -> None:
        self.assertFalse(contract.canonicalize_typed("GLONASS", 7, "L1")["accepted"])
        self.assertFalse(contract.canonicalize_typed("GLONASS", 7, "L1", -8)["accepted"])
        self.assertFalse(contract.canonicalize_typed("GLONASS", 7, "L1", 7)["accepted"])
        self.assertTrue(contract.canonicalize_typed("GLONASS", 7, "L1", -7)["accepted"])
        self.assertTrue(contract.canonicalize_typed("GLONASS", 7, "L1", 6)["accepted"])

    def test_inventory_validator_accepts_preflight_before_native(self) -> None:
        record = contract.verify_inventory_record(contract.ROUTES[0],
                                                  passing_inventory(contract.ROUTES[0]))
        self.assertTrue(record["inventory_passed"])
        self.assertEqual(record["solver_invocations"], 0)

    def test_summary_validator_requires_native_reach_and_strict_progress(self) -> None:
        record = contract.verify_structural_summary(contract.ROUTES[0],
                                                    passing_summary(contract.ROUTES[0]))
        self.assertTrue(record["structural_gate_passed"])
        self.assertEqual(record["accepted_iterations"]["main"], 3)

    def test_preflight_failure_cannot_claim_native_reach(self) -> None:
        record = passing_inventory(contract.ROUTES[0])
        record["phase132"]["python_preflight_executed"] = False
        with self.assertRaises(contract.Phase132ContractError):
            contract.verify_inventory_record(contract.ROUTES[0], record)

    def test_placeholder_command_is_exact_and_selector_isolated(self) -> None:
        command = contract.command_template(contract.ROUTES[0])
        contract.validate_command(contract.ROUTES[0], command)
        self.assertEqual(command.count(contract.PHASE131_SELECTOR), 1)
        self.assertEqual(command.count(contract.PHASE117_SELECTOR), 0)
        self.assertEqual(command.count(contract.PHASE120_SELECTOR), 0)
        self.assertEqual(command.count(contract.ADDITIONAL_SELECTOR), 0)


if __name__ == "__main__":
    unittest.main()
