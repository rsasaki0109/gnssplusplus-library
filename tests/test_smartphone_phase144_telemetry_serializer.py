"""Focused launch-free tests for the Phase144 native serializer contract.

All native-summary fixtures are synthetic in-memory/JSON bytes.  These tests
do not open raw GNSS/IMU/navigation/base payloads, solution rows, truth, MAT,
PDC, or Kaggle artifacts and never start a solver.
"""

from __future__ import annotations

import copy
import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = ROOT / (
    "apps/commands/benchmarks/"
    "gnss_smartphone_phase144_telemetry_serializer_structural.py"
)
SPEC = importlib.util.spec_from_file_location("phase144_runner", RUNNER_PATH)
assert SPEC is not None and SPEC.loader is not None
RUNNER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RUNNER)


def _family(rows: int, source: str) -> dict:
    return {
        "admitted_rows": rows,
        "affine_factors_inserted": rows,
        "key_order_exact": True,
        "finite_values": True,
        "source_geometry_same_path": True,
        "source_report": source,
    }


def _termination(stage: str) -> dict:
    return {
        "selector_enabled": True,
        "stage": stage,
        "configured_max_iterations": 1000,
        "effective_max_iterations": 1000,
        "attempted": True,
        "attempted_outer_iterations": 3,
        "accepted_outer_iterations": 2,
        "rejected_outer_iterations": 1,
        "total_inner_lambda_attempts": 3,
        "initial_cost": 20.0,
        "final_cost": 8.0,
        "costs_finite": True,
        "strict_cost_decrease": True,
        "termination_branch": "converged",
        "relative_error_tolerance": 1e-5,
        "absolute_error_tolerance": 1e-5,
        "error_tolerance": 1e-5,
        "initial_lambda": 1e-5,
        "final_lambda": 1e-6,
        "maximum_lambda": 1e5,
        "lambda_factor": 10.0,
        "lambda_lower_bound": 1e-9,
        "lambda_upper_bound": 1e9,
        "min_model_fidelity": 1e-3,
        "diagonal_damping": True,
        "use_fixed_lambda_factor": True,
        "linear_solver": "MULTIFRONTAL_QR",
        "elimination": "EliminateQR",
        "ordering_type": "COLAMD",
        "explicit_ordering_present": False,
        "no_fallback": True,
        "termination_trace_complete": True,
        "configuration_valid": True,
        "configuration_failure": "",
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
        "phase143_official_main_lm_termination_budget": True,
        "phase144_telemetry_schema": True,
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
        "equation": RUNNER.equation_object(),
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
        "equation": RUNNER.equation_object(),
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
        "native_phase144_telemetry_schema": True,
        "phase144_telemetry": telemetry,
        "phase138_affine_tdcp_anchor_range_constant": {
            "measurement_equation": RUNNER.EQUATION_DISPLAY,
            "equation": RUNNER.equation_object(),
        },
        "phase143_termination": {
            "schema_version": "smartphone-r5-native-fgo-phase143-termination.v1",
            "authority": "FGOResult.diagnostics.native_phase143_termination",
            "main": _termination("main"),
            "gnss_first": _termination("gnss_first"),
            "no_solution_or_accuracy_fields": True,
        },
    }


NATIVE_SUMMARY_FIXTURES = {
    "mtv-a": '{"gnss_first":{"failure":"stage","failure":"handoff"},"phase144_telemetry":{"items":[{"failure":1,"failure":2}]}}',
    "lax-t": '{"gnss_first":{"nested":[{"failure":0,"failure":1}],"failure":"stage","failure":"handoff"},"phase144_telemetry":{"items":[{"failure":1,"failure":2}]}}',
}


class Phase144TelemetrySerializerTests(unittest.TestCase):
    def test_complete_native_summary_has_no_missing_fields(self) -> None:
        for route in RUNNER.ROUTES:
            RUNNER.validate_native_summary(route, complete_summary(route))

    def test_recursive_duplicate_paths_cover_every_summary_fixture(self) -> None:
        for name, text in NATIVE_SUMMARY_FIXTURES.items():
            with self.subTest(fixture=name):
                paths = RUNNER.duplicate_json_paths(text)
                self.assertEqual(paths.count("/gnss_first/failure"), 1)
                self.assertIn("/phase144_telemetry/items/0/failure", paths)

    def test_duplicate_json_read_fails_closed(self) -> None:
        with tempfile.NamedTemporaryFile("w", suffix=".json", encoding="utf-8") as handle:
            handle.write('{"gnss_first":{"failure":1,"failure":2}}')
            handle.flush()
            with self.assertRaises(RUNNER.Phase144ContractError):
                RUNNER.read_json(Path(handle.name), "duplicate native summary")

    def test_missing_and_misnamed_fields_fail_closed(self) -> None:
        for key in ("phase144_telemetry", "phase143_termination"):
            summary = complete_summary(RUNNER.ROUTES[0])
            summary[key + "_misnamed"] = summary.pop(key)
            with self.subTest(key=key), self.assertRaises(RUNNER.Phase144ContractError):
                RUNNER.validate_native_summary(RUNNER.ROUTES[0], summary)

    def test_wrong_equation_display_ast_tokens_and_representation_fail(self) -> None:
        mutations = (
            lambda equation: equation.__setitem__("display_expression", "tdcp_native"),
            lambda equation: equation["ast"]["rhs"].__setitem__("kind", "add"),
            lambda equation: equation["token_tuple"].__setitem__(2, "ADD"),
            lambda equation: equation.__setitem__("representation", "rhs-only-native-diagnostic"),
        )
        for mutate in mutations:
            summary = complete_summary(RUNNER.ROUTES[0])
            mutate(summary["phase144_telemetry"]["equation"])
            with self.assertRaises(RUNNER.Phase144ContractError):
                RUNNER.validate_native_summary(RUNNER.ROUTES[0], summary)

    def test_nonfinite_cost_and_generic_iterations_cannot_replace_authority(self) -> None:
        summary = complete_summary(RUNNER.ROUTES[0])
        summary["phase144_telemetry"]["solver"]["main"]["initial_cost"] = float("nan")
        with self.assertRaises(RUNNER.Phase144ContractError):
            RUNNER.validate_native_summary(RUNNER.ROUTES[0], summary)

        summary = complete_summary(RUNNER.ROUTES[0])
        stage = summary["phase144_telemetry"]["solver"]["main"]
        del stage["accepted_iterations"]
        stage["iterations"] = 12
        with self.assertRaises(RUNNER.Phase144ContractError):
            RUNNER.validate_native_summary(RUNNER.ROUTES[0], summary)

    def test_selector_isolation_and_authority_duplication_fail_closed(self) -> None:
        summary = complete_summary(RUNNER.ROUTES[0])
        summary["phase144_telemetry"]["selectors"]["phase117_dynamic_tdcp_sigma"] = True
        with self.assertRaises(RUNNER.Phase144ContractError):
            RUNNER.validate_native_summary(RUNNER.ROUTES[0], summary)

        summary = complete_summary(RUNNER.ROUTES[0])
        summary["phase144_telemetry"]["sync_count"] = 2
        with self.assertRaises(RUNNER.Phase144ContractError):
            RUNNER.validate_native_summary(RUNNER.ROUTES[0], summary)

    def test_solution_rows_and_duplicate_top_level_failure_are_not_accepted(self) -> None:
        summary = complete_summary(RUNNER.ROUTES[0])
        summary["phase144_telemetry"]["output"]["solution_rows"] = []
        with self.assertRaises(RUNNER.Phase144ContractError):
            RUNNER.validate_native_summary(RUNNER.ROUTES[0], summary)

        duplicate = '{"failure":"native","failure":"handoff"}'
        self.assertEqual(RUNNER.duplicate_json_paths(duplicate), ["/failure"])

    def test_source_has_opt_in_serializer_and_no_solver_execution_hook(self) -> None:
        source = RUNNER.NATIVE_SOURCE.read_text(encoding="utf-8")
        self.assertIn("--native-phase144-telemetry-schema", source)
        self.assertIn("phase144_telemetry", source)
        self.assertIn(RUNNER.EQUATION_DISPLAY, source)
        self.assertIn("handoff_failure", source)


if __name__ == "__main__":
    unittest.main()
