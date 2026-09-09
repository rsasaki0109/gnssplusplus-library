"""Launch-free tests for the Phase131 canonical correction contract.

All inventory and structural examples are synthetic in-memory dictionaries.
No GNSS/IMU/navigation/base payload, solver output, truth, MAT, PDC, or
Kaggle artifact is opened by this test module.
"""

from __future__ import annotations

import json
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps/commands/benchmarks"))
import gnss_smartphone_phase131_canonical_correction_structural as contract  # noqa: E402


FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase131_canonical_correction_structural_contract_freeze_v1.json"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase131_canonical_correction_structural_manifest_v1.json"
VALIDATOR = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase131_canonical_correction_structural.py"
RUNNER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase131_canonical_correction_structural_execute.py"


def passing_inventory(route: str) -> dict:
    """A deliberately unequal side/canonical ledger with alias rows."""
    rover = {
        "input_rows": 9,
        "certified_rows": 6,
        "explicit_local_miss_rows": 3,
        "header_primary_certified_rows": 2,
        "broadcast_geph_certified_rows": 4,
        "reason_counts": {"glonass-fcn-missing": 1, "unknown-band": 2},
        "all_rows_classified": True,
        "no_raw_uncorrected": True,
        "no_zero_correction": True,
        "no_fallback": True,
        "no_extrapolation": True,
        "finite_certified_wavelength": True,
        "canonical_key_accounted": True,
        "literal_provenance_retained": True,
        "canonical": {
            "input_rows": 9,
            "certified_rows": 6,
            "explicit_local_miss_rows": 3,
            "all_rows_accounted": True,
            "same_physical_family_aliases_only": True,
            "different_family_rejected": True,
            "unknown_band_explicit_miss": True,
            "literal_tracking_code_in_join": False,
            "original_signal_provenance_retained": True,
            "fcn_certification_fail_closed": True,
        },
    }
    base = {
        "input_rows": 14,
        "certified_rows": 10,
        "explicit_local_miss_rows": 4,
        "header_primary_certified_rows": 4,
        "broadcast_geph_certified_rows": 6,
        "reason_counts": {"query-time-gap": 4},
        "all_rows_classified": True,
        "no_raw_uncorrected": True,
        "no_zero_correction": True,
        "no_fallback": True,
        "no_extrapolation": True,
        "finite_certified_wavelength": True,
        "canonical_key_accounted": True,
        "literal_provenance_retained": True,
        "canonical": {
            "input_rows": 14,
            "certified_rows": 10,
            "explicit_local_miss_rows": 4,
            "all_rows_accounted": True,
            "same_physical_family_aliases_only": True,
            "different_family_rejected": True,
            "unknown_band_explicit_miss": True,
            "literal_tracking_code_in_join": False,
            "original_signal_provenance_retained": True,
            "fcn_certification_fail_closed": True,
        },
    }
    return {
        "route": route,
        "stage": "post-independent-authorization-pre-solver",
        "solver_invocations": 0,
        "solver_may_start": True,
        "rover": rover,
        "base": base,
        "shared_ledger": {
            "side_local_conservation": True,
            "canonical_key_conservation": True,
            "same_physical_family_aliases_only": True,
            "different_physical_band_rejected": True,
            "literal_tracking_code_in_join": False,
            "original_signal_provenance_retained": True,
            "certified_glonass_fcn_only": True,
            "ambiguous_multi_code_fail_closed": True,
            "duplicate_conflict_fail_closed": True,
            "retained_rover_exact_key_support": True,
            "exact_endpoint_or_adjacent_two_point_bracket": True,
            "unmatched_rover_explicit_factor_miss": True,
            "unused_base_rows_accounted": True,
            "whole_ledger_equality_required": False,
            "cross_side_count_equality_required": False,
            "no_raw_uncorrected": True,
            "no_zero_correction": True,
            "no_fallback": True,
            "no_extrapolation": True,
        },
        "base_streams": {
            "finite_samples": 8,
            "used_rows": 5,
            "unused_rows": 3,
            "unused_streams": 1,
            "all_samples_accounted": True,
            "duplicate_nonmonotonic_rejected": True,
            "canonical_key_streams_accounted": True,
        },
        "correction": {
            "factor_input_rows": 8,
            "retained_corrected_rows": 5,
            "missing_exact_stream": 2,
            "out_of_domain": 1,
            "nonfinite": 0,
            "support": {
                "exact_endpoint_rows": 1,
                "adjacent_two_point_bracket_rows": 3,
                "exact_interior_sample_rows": 1,
            },
            "retained_factors_have_exact_key_support": True,
            "one_support_result_per_retained_factor": True,
            "unused_base_rows_accounted": True,
            "application_passes": 1,
            "exactly_once": True,
            "no_duplicate_application": True,
            "no_raw_fallback": True,
            "no_zero_fallback": True,
            "no_nearest_fill": True,
            "no_endpoint_hold": True,
            "no_extrapolation": True,
            "finite_corrected_rows": True,
            "canonical_key_mode": True,
            "factor_topology_changed": False,
        },
        "phase131": {
            "enabled": True,
            "canonical_key": "GNSSSystem,PRN,physical-frequency-family[,certified-GLO-FCN]",
            "side_local_conservation": True,
            "canonical_key_conservation": True,
            "literal_tracking_code_in_join": False,
            "original_signal_provenance_retained": True,
            "retained_rover_exact_key_support": True,
            "unmatched_rover_explicit_factor_miss": True,
            "unused_base_rows_accounted": True,
            "no_raw_uncorrected": True,
            "no_zero_correction": True,
            "no_fallback": True,
            "no_extrapolation": True,
            "source_complete_a_b_c": True,
        },
    }


def passing_summary(route: str) -> dict:
    return {
        "route": route,
        "solution_content_read": False,
        "fallback_used": False,
        "rerun_count": 0,
        "gnss_first": {"accepted_iterations": 2, "initial_cost": 10.0, "final_cost": 7.0},
        "main": {"accepted_iterations": 3, "initial_cost": 9.0, "final_cost": 6.0,
                 "solver_branch": "MULTIFRONTAL_QR"},
        "phase131": {
            "enabled": True,
            "canonical_key": "GNSSSystem,PRN,physical-frequency-family[,certified-GLO-FCN]",
            "side_local_conservation": True,
            "canonical_key_conservation": True,
            "literal_tracking_code_in_join": False,
            "original_signal_provenance_retained": True,
            "retained_rover_exact_key_support": True,
            "unused_base_rows_accounted": True,
            "no_fallback": True,
            "factor_topology_changed": False,
        },
        "gates": {"canonical_join": True, "factor_support": True,
                  "finite_coverage": True},
        "opaque_solution": {"sha256": "a" * 64, "rows": 10},
    }


class Phase131CanonicalStructuralTests(unittest.TestCase):
    def test_aliases_collapse_only_by_physical_family(self) -> None:
        ca = contract.canonicalize_typed("GLONASS", 7, "L1", -4)
        p = contract.canonicalize_typed("GLONASS", 7, "L1", -4)
        self.assertEqual(ca["key"], p["key"])
        self.assertNotEqual(ca["key"], contract.canonicalize_typed(
            "GLONASS", 7, "L5", -4)["key"])
        self.assertEqual(contract.typed_family("GLONASS", "GLO_G1_CA"), "L1")
        self.assertEqual(contract.typed_family("GLONASS", "GLO_G1C"), "L1")
        self.assertEqual(contract.rinex_family("GLONASS", "C1C"), "L1")
        self.assertEqual(contract.rinex_family("GLONASS", "C1P"), "L1")
        self.assertIsNone(contract.rinex_family("GLONASS", "C2P"))

    def test_certified_glonass_fcn_is_required_and_bounded(self) -> None:
        self.assertFalse(contract.canonicalize_typed(
            "GLONASS", 7, "L1")["accepted"])
        self.assertFalse(contract.canonicalize_typed(
            "GLONASS", 7, "L1", -8)["accepted"])
        self.assertTrue(contract.canonicalize_typed(
            "GLONASS", 7, "L1", -7)["accepted"])
        self.assertTrue(contract.canonicalize_typed(
            "GLONASS", 7, "L1", 6)["accepted"])
        self.assertFalse(contract.canonicalize_typed(
            "GPS", 1, "L1", -4)["accepted"])

    def test_same_key_alias_and_divergent_conflict(self) -> None:
        rows = [
            {"system": "GLONASS", "prn": 7, "family": "L1",
             "certified_fcn": -4, "original_signal": "GLO_L1CA",
             "literal_tracking_code": "C1C", "value_digest": "same"},
            {"system": "GLONASS", "prn": 7, "family": "L1",
             "certified_fcn": -4, "original_signal": "GLO_L1P",
             "literal_tracking_code": "C1P", "value_digest": "same"},
        ]
        result = contract.verify_canonical_streams(rows)
        self.assertEqual(result["canonical_distinct_keys"], 1)
        self.assertEqual(result["same_family_alias_rows"], 1)
        rows[1]["value_digest"] = "different"
        with self.assertRaises(contract.Phase131ContractError):
            contract.verify_canonical_streams(rows)

    def test_unknown_band_and_invalid_provenance_are_explicit_misses(self) -> None:
        for row in (
            {"system": "GPS", "prn": 1, "family": "L2",
             "original_signal": "GPS_L2C", "literal_tracking_code": "C2C"},
            {"system": "GLONASS", "prn": 7, "family": "L1",
             "original_signal": "GLO_L1CA", "literal_tracking_code": "C1C"},
        ):
            item = contract.canonicalize_row(row)
            self.assertFalse(item["accepted"])
            self.assertIsNotNone(item["reason"])

    def test_side_and_shared_ledger_allow_unused_base_rows(self) -> None:
        report = contract.verify_inventory_record(
            contract.ROUTES[0], passing_inventory(contract.ROUTES[0]))
        self.assertTrue(report["inventory_passed"])
        self.assertEqual(report["base_unused_rows"], 3)
        self.assertEqual(report["rover_rows"], 9)
        self.assertEqual(report["base_rows"], 14)

    def test_duplicate_stream_and_unsupported_support_fail_closed(self) -> None:
        with self.assertRaises(contract.Phase131ContractError):
            contract.validate_stream_samples(
                [{"time": 10.0, "value": 1.0}, {"time": 10.0, "value": 2.0}])
        with self.assertRaises(contract.Phase131ContractError):
            contract.validate_stream_samples(
                [{"time": 20.0, "value": 1.0}, {"time": 10.0, "value": 2.0}])
        support = contract.classify_support(
            [{"time": 10.0, "value": 100.0},
             {"time": 20.0, "value": 120.0}], 15.0)
        self.assertTrue(support["supported"])
        self.assertEqual(support["support_kind"], "adjacent_two_point_bracket")
        self.assertEqual(support["value"], 110.0)
        self.assertFalse(contract.classify_support(
            [{"time": 10.0, "value": 1.0}, {"time": 20.0, "value": 2.0}],
            9.0)["supported"])

    def test_summary_requires_progress_decrease_and_opaque_solution(self) -> None:
        report = contract.verify_structural_summary(
            contract.ROUTES[0], passing_summary(contract.ROUTES[0]))
        self.assertTrue(report["structural_gate_passed"])
        summary = passing_summary(contract.ROUTES[0])
        summary["main"]["final_cost"] = 9.0
        with self.assertRaises(contract.Phase131ContractError):
            contract.verify_structural_summary(contract.ROUTES[0], summary)

    def test_command_has_exact_composition_and_placeholders(self) -> None:
        command = contract.command_template(contract.ROUTES[0])
        for flag in (contract.PHASE126_SELECTOR, contract.PHASE127_SELECTOR,
                     contract.PHASE128_SELECTOR, contract.PHASE129_SELECTOR,
                     contract.PHASE130_SELECTOR, contract.PHASE131_SELECTOR,
                     contract.PHASE118_SELECTOR):
            self.assertEqual(command.count(flag), 1)
        for flag in (contract.PHASE117_SELECTOR, contract.PHASE120_SELECTOR,
                     contract.ADDITIONAL_SELECTOR):
            self.assertEqual(command.count(flag), 0)
        self.assertIn("__PHASE131_RAW_DEVICE_GNSS__", command)
        self.assertIn("__PHASE131_RAW_BASE_RINEX__", command)

    def test_launch_free_sources_do_not_launch_or_probe_payloads(self) -> None:
        combined = (VALIDATOR.read_text(encoding="utf-8") +
                    RUNNER.read_text(encoding="utf-8"))
        for token in ("subprocess", "Popen", "os.system", "os.popen",
                      "check_output"):
            self.assertNotIn(token, combined)
        self.assertIn("verify_pre_raw_static", combined)

    def test_freeze_and_manifest_are_pinned(self) -> None:
        self.assertTrue(FREEZE.is_file())
        self.assertTrue(MANIFEST.is_file())
        freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        self.assertEqual(freeze["phase"], 131)
        self.assertEqual(freeze["decision"]["candidate_id"], contract.CANDIDATE_ID)
        self.assertTrue(freeze["decision"]["implementation_authorized"])
        self.assertFalse(freeze["decision"]["raw_materialization_authorized"])
        self.assertEqual(contract.verify_freeze()["phase"], 131)
        self.assertEqual(contract.verify_manifest()["phase"], 131)

    def test_pre_raw_accounting_is_zero(self) -> None:
        result = contract.verify_pre_raw_static()
        self.assertEqual(result["read_accounting"]["raw_phone_gnss_reads"], 0)
        self.assertEqual(result["read_accounting"]["raw_base_rinex_payload_reads"], 0)
        self.assertEqual(result["read_accounting"]["native_solver_invocations"], 0)
        self.assertFalse(result["raw_execution_authorized"])


if __name__ == "__main__":
    unittest.main()
