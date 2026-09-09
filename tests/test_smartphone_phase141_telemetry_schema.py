"""Focused, launch-free tests for the Phase141 native telemetry contract.

The fixtures below are in-memory native-summary-shaped objects.  They do not
open a route payload, solution, truth, MAT/PDC artifact, or start a solver.
"""

from __future__ import annotations

import copy
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = (
    ROOT
    / "apps/commands/benchmarks/"
    / "gnss_smartphone_phase141_telemetry_schema_structural.py"
)
SPEC = importlib.util.spec_from_file_location("phase141_runner", RUNNER_PATH)
assert SPEC is not None and SPEC.loader is not None
RUNNER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RUNNER)


def _equation() -> dict:
    return {
        "semantic_id": RUNNER.EQUATION_ID,
        "representation": "full-assignment",
        "display_expression": RUNNER.EQUATION_DISPLAY,
        "ast": {
            "kind": "assign",
            "lhs": "tdcp_phase138",
            "rhs": {
                "kind": "sub",
                "left": "tdcp_native",
                "right": {
                    "kind": "sub",
                    "left": "rho_current_initial",
                    "right": "rho_previous_initial",
                },
            },
        },
        "token_tuple": list(RUNNER.EQUATION_FULL_TOKENS),
        "rhs_only_token_tuple": list(RUNNER.EQUATION_RHS_TOKENS),
        "whitespace_normalization_only": True,
    }


def _family(rows: int, name: str) -> dict:
    return {
        "admitted_rows": rows,
        "affine_factors_inserted": rows,
        "key_order_exact": True,
        "finite_values": True,
        "source_geometry_same_path": True,
        "source_report": name,
    }


def complete_summary(route: str) -> dict:
    selectors = {
        "phase135_official_affine_measurement_family": True,
        "phase138_affine_tdcp_anchor_range_constant": True,
        "phase118_official_tdcp_huber_k": True,
        "phase117_dynamic_tdcp_sigma": False,
        "phase120_official_tdcp_resl_atmosphere_cancellation": False,
        "phase126_raw_base_source_complete": False,
        "phase127_glonass_channel_provenance": False,
        "phase128_glonass_provenance_parser_admission": False,
        "phase129_glonass_local_miss_mask": False,
        "phase130_shared_ledger_key_local_support": False,
        "phase131_canonical_correction_band_key": False,
        "phase107_raw_base_compensation": True,
        "phase107_raw_base_source_miss_mask": True,
        "phase107_preserve_additional_frequency_bands": False,
        "phase141_telemetry_schema": True,
    }
    phase135 = {
        "enabled": True,
        "configuration_valid": True,
        "transactional": True,
        "fixed_initial_geometry": True,
        "finite_jacobians": True,
        "single_sagnac_representation": True,
        "los_convention": "-e=(receiver-satellite)/range",
        "geometry_rows": 3,
        "sagnac_evaluations": 3,
        "pseudorange": _family(3, "FGOProblem.pseudorange_factors"),
        "doppler": _family(3, "FGOProblem.undifferenced_doppler_factors"),
        "ordinary_tdcp": _family(3, "FGOProblem.tdcp_factors"),
        "legacy_factor_counts": {
            "pseudorange": 0,
            "doppler": 0,
            "ordinary_tdcp": 0,
        },
        "pose3_x_bridge": {"count": 3, "keys_exact": True},
    }
    phase138 = {
        "enabled": True,
        "phase135_dependency_satisfied": True,
        "configuration_valid": True,
        "transactional": True,
        "adjusted_exactly_once": True,
        "factor_count_unchanged": True,
        "same_endpoint_epoch_and_satellite_state": True,
        "same_satellite_state": True,
        "finite_adjusted_measurements": True,
        "no_raw_or_zero_fallback": True,
        "phase118_atmosphere_sigma_huber_unchanged": True,
        "single_sagnac_representation": True,
        "measurement_equation": RUNNER.EQUATION_DISPLAY,
        "geometry_representation": "RTKLIB-geodist-single-Sagnac-fixed-initial-endpoints",
        "range_constants_validated": 3,
        "tdcp_measurements_adjusted": 3,
        "affine_tdcp_factor_count": 3,
        "adjustment_application_passes": 1,
        "legacy_tdcp_factor_count": 0,
        "equation": _equation(),
    }
    telemetry = {
        "schema_version": RUNNER.NATIVE_SCHEMA,
        "enabled": True,
        "sync_count": 1,
        "authority": {
            "gnss_first": "ImuBuildReport+GNSS-first-FGOResult.diagnostics",
            "main": "FGOResult.diagnostics+FGOProblem",
            "raw_base": "BasePseudorangeCompensationReport",
            "offset": "UpstreamPositionOffsetReport",
            "output": "native-output-boundary",
            "wrapper_read_accounting": "independent-wrapper-observation",
        },
        "selectors": selectors,
        "equation": _equation(),
        "phase135": phase135,
        "phase138": phase138,
        "raw_base": {
            "phase107_recipe": True,
            "applied_exactly_once": True,
            "source_miss_conservation": True,
            "no_raw_or_zero_fallback": True,
        },
        "clock": {
            "c_units": "metres",
            "d_units": "metres/second",
            "c7_mapping_exact": True,
            "d_full_finite_exact_alignment": True,
            "c_epoch_count": 3,
            "c_finite_count": 3,
            "d_epoch_count": 3,
            "d_finite_count": 3,
            "c7_dimension": 7,
            "c7_state_count": 3,
            "c7_handoff_count": 3,
        },
        "solver": {
            "linear_solver": "MULTIFRONTAL_QR",
            "elimination": "EliminateQR",
            "gnss_first": {
                "attempted": True,
                "accepted_iterations": 2,
                "initial_cost": 10.0,
                "final_cost": 5.0,
                "costs_finite": True,
                "strict_cost_decrease": True,
                "terminal_branch": "native-gnss-first",
                "no_fallback": True,
            },
            "main": {
                "attempted": True,
                "accepted_iterations": 2,
                "initial_cost": 20.0,
                "final_cost": 8.0,
                "costs_finite": True,
                "strict_cost_decrease": True,
                "terminal_branch": "converged",
                "no_fallback": True,
            },
        },
        "factor_counts": {
            "pseudorange": 3,
            "doppler": 3,
            "ordinary_tdcp": 3,
            "legacy_pseudorange": 0,
            "legacy_doppler": 0,
            "legacy_tdcp": 0,
        },
        "bridge": {
            "pose3_x_count": 3,
            "pose3_x_keys_exact": True,
            "phase131_sync_count": 0,
        },
        "offset": {"enabled": True, "applied": True, "application_passes": 1},
        "output": {
            "finite": True,
            "earth_valid": True,
            "expected_epoch_coverage": True,
            "opaque_solution_seal": True,
            "coordinate_rows_interpreted": False,
            "pixel5_offset_applications": 1,
        },
        "policy": {
            "truth_used": False,
            "coordinate_rows_interpreted": False,
            "solution_publication_authorized": False,
            "fallback": False,
            "rerun": False,
        },
    }
    return {
        "dataset_id": route,
        "truth_used": False,
        "no_base_contract": True,
        "native_phase141_telemetry_schema": True,
        "phase141_telemetry": telemetry,
    }


class Phase141TelemetrySchemaTests(unittest.TestCase):
    def test_complete_real_shaped_summary_has_no_missing_fields(self) -> None:
        for route in RUNNER.ROUTES:
            RUNNER.validate_native_summary(route, complete_summary(route))

    def test_generic_iterations_cannot_replace_accepted_iterations(self) -> None:
        summary = complete_summary(RUNNER.ROUTES[0])
        stage = summary["phase141_telemetry"]["solver"]["main"]
        del stage["accepted_iterations"]
        stage["iterations"] = 4
        with self.assertRaises(RUNNER.Phase141ContractError):
            RUNNER.validate_native_summary(RUNNER.ROUTES[0], summary)

    def test_missing_or_misnamed_group_rejects(self) -> None:
        for bad_key in ("phase138", "solver"):
            summary = complete_summary(RUNNER.ROUTES[0])
            telemetry = summary["phase141_telemetry"]
            telemetry[bad_key + "_misnamed"] = telemetry.pop(bad_key)
            with self.subTest(bad_key=bad_key), self.assertRaises(
                RUNNER.Phase141ContractError
            ):
                RUNNER.validate_native_summary(RUNNER.ROUTES[0], summary)

    def test_equation_ast_operator_and_lhs_are_source_locked(self) -> None:
        for mutate in (
            lambda equation: equation["ast"]["rhs"].__setitem__("kind", "add"),
            lambda equation: equation["token_tuple"].__setitem__(2, "ADD"),
            lambda equation: equation["ast"].__setitem__("lhs", "tdcp_wrong"),
        ):
            summary = complete_summary(RUNNER.ROUTES[0])
            mutate(summary["phase141_telemetry"]["equation"])
            with self.assertRaises(RUNNER.Phase141ContractError):
                RUNNER.validate_native_summary(RUNNER.ROUTES[0], summary)

    def test_duplicate_sync_and_solution_content_reject(self) -> None:
        summary = complete_summary(RUNNER.ROUTES[0])
        summary["phase141_telemetry"]["sync_count"] = 2
        with self.assertRaises(RUNNER.Phase141ContractError):
            RUNNER.validate_native_summary(RUNNER.ROUTES[0], summary)

        summary = complete_summary(RUNNER.ROUTES[0])
        summary["phase141_telemetry"]["output"]["coordinate_rows"] = []
        with self.assertRaises(RUNNER.Phase141ContractError):
            RUNNER.validate_native_summary(RUNNER.ROUTES[0], summary)

    def test_duplicate_json_keys_fail_closed(self) -> None:
        with tempfile.NamedTemporaryFile("w", suffix=".json", encoding="utf-8") as handle:
            handle.write('{"schema_version": "a", "schema_version": "b"}')
            handle.flush()
            with self.assertRaises(RUNNER.Phase141ContractError):
                RUNNER.read_json(Path(handle.name), "duplicate fixture")

    def test_selector_isolation_rejects_phase117(self) -> None:
        summary = complete_summary(RUNNER.ROUTES[0])
        summary["phase141_telemetry"]["selectors"]["phase117_dynamic_tdcp_sigma"] = True
        with self.assertRaises(RUNNER.Phase141ContractError):
            RUNNER.validate_native_summary(RUNNER.ROUTES[0], summary)

    def test_disabled_phase131_cannot_claim_a_bridge_sync(self) -> None:
        summary = complete_summary(RUNNER.ROUTES[0])
        summary["phase141_telemetry"]["bridge"]["phase131_sync_count"] = 1
        with self.assertRaises(RUNNER.Phase141ContractError):
            RUNNER.validate_native_summary(RUNNER.ROUTES[0], summary)

    def test_equation_representation_is_explicit(self) -> None:
        summary = complete_summary(RUNNER.ROUTES[0])
        summary["phase141_telemetry"]["equation"]["representation"] = (
            "rhs-only-native-diagnostic"
        )
        RUNNER.validate_native_summary(RUNNER.ROUTES[0], summary)


if __name__ == "__main__":
    unittest.main()
