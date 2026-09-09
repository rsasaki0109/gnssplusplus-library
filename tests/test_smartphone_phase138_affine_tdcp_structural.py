"""Launch-free tests for the Phase138 structural contract.

Only tracked contract/source files and an in-memory synthetic summary are
used.  No raw payload, solution coordinate, truth, MAT/PDC, or native solver
is opened or launched.
"""

from __future__ import annotations

import copy
import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = ROOT / (
    "apps/commands/benchmarks/"
    "gnss_smartphone_phase138_affine_tdcp_structural.py"
)


def _load_runner():
    spec = importlib.util.spec_from_file_location(
        "phase138_affine_tdcp_structural_runner", RUNNER_PATH
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("unable to load Phase138 structural runner")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


RUNNER = _load_runner()


def _family(count: int = 2) -> dict:
    return {
        "admitted_rows": count,
        "affine_factors_inserted": count,
        "key_order_exact": True,
        "finite_values": True,
        "source_geometry_same_path": True,
    }


def _summary(route: str) -> dict:
    tdcp_count = 2
    return {
        "route": route,
        "selectors": {
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
        },
        "phase135": {
            "enabled": True,
            "configuration_valid": True,
            "transactional": True,
            "fixed_initial_geometry": True,
            "finite_jacobians": True,
            "single_sagnac_representation": True,
            "los_convention": "-e=(receiver-satellite)/range",
            "sagnac_evaluations": 7,
            "geometry_rows": 7,
            "pseudorange": _family(3),
            "doppler": _family(3),
            "ordinary_tdcp": _family(tdcp_count),
            "legacy_factor_counts": {
                "pseudorange": 0,
                "doppler": 0,
                "ordinary_tdcp": 0,
            },
            "pose3_x_bridge": {"count": 3, "keys_exact": True},
        },
        "phase138": {
            "enabled": True,
            "phase135_dependency_satisfied": True,
            "configuration_valid": True,
            "measurement_equation": (
                "tdcp_phase138 = tdcp_native - "
                "(rho_current_initial - rho_previous_initial)"
            ),
            "geometry_representation": (
                "RTKLIB-geodist-single-Sagnac-fixed-initial-endpoints"
            ),
            "range_constants_validated": tdcp_count,
            "tdcp_measurements_adjusted": tdcp_count,
            "affine_tdcp_factor_count": tdcp_count,
            "adjustment_application_passes": 1,
            "adjusted_exactly_once": True,
            "factor_count_unchanged": True,
            "same_endpoint_epoch_and_satellite_state": True,
            "same_satellite_state": True,
            "finite_adjusted_measurements": True,
            "phase118_atmosphere_sigma_huber_unchanged": True,
            "single_sagnac_representation": True,
            "no_raw_or_zero_fallback": True,
            "transactional": True,
            "legacy_tdcp_factor_count": 0,
        },
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
        },
        "solver": {
            "linear_solver": "MULTIFRONTAL_QR",
            "elimination": "EliminateQR",
            "gnss_first": {
                "accepted_iterations": 1,
                "initial_cost": 10.0,
                "final_cost": 5.0,
                "no_fallback": True,
            },
            "main": {
                "accepted_iterations": 2,
                "initial_cost": 20.0,
                "final_cost": 10.0,
                "no_fallback": True,
            },
        },
        "output": {
            "finite": True,
            "earth_valid": True,
            "expected_epoch_coverage": True,
            "pixel5_offset_applications": 1,
            "opaque_solution_seal": True,
        },
        "read_accounting": {
            "raw_phone_gnss_reads": 43259,
            "raw_phone_imu_reads": 86400,
            "broadcast_navigation_reads": 400,
            "raw_base_rinex_reads": 900,
            "truth_reads": 0,
            "solution_coordinate_reads": 0,
            "mat_pdc_precomputed_coordinate_reads": 0,
            "solver_invocations": 1,
            "accuracy_calculations": 0,
            "kaggle_access": 0,
        },
        "fallback": False,
        "rerun": False,
    }


class Phase138AffineTdcpStructuralTests(unittest.TestCase):
    def test_launch_free_static_pins_and_zero_reads(self) -> None:
        result = RUNNER.launch_free_validation()
        self.assertEqual(result["status"], "launch-free-qualified")
        self.assertEqual(result["phase"], 138)
        self.assertEqual(result["routes"], list(RUNNER.ROUTES))
        self.assertEqual(result["raw_execution_authorized"], False)
        self.assertEqual(result["solver_invocations"], 0)
        self.assertEqual(result["truth_reads"], 0)
        self.assertEqual(result["solution_coordinate_reads"], 0)
        self.assertEqual(result["mat_pdc_precomputed_coordinate_reads"], 0)
        self.assertEqual(result["kaggle_access"], 0)

    def test_placeholder_commands_are_exact_in_fixed_route_order(self) -> None:
        manifest = RUNNER.read_json(RUNNER.MANIFEST, "manifest")
        self.assertEqual(manifest["routes"], list(RUNNER.ROUTES))
        self.assertEqual(list(manifest["command_snapshots"]), list(RUNNER.ROUTES))
        for route in RUNNER.ROUTES:
            command = manifest["command_snapshots"][route]
            RUNNER.validate_command(route, command)
            self.assertEqual(command, RUNNER.command_template(route))

    def test_synthetic_phase138_summary_passes_for_both_routes(self) -> None:
        for route in RUNNER.ROUTES:
            RUNNER.validate_structural_summary(route, _summary(route))

    def test_range_count_endpoint_and_exactly_once_fail_closed(self) -> None:
        bad = _summary(RUNNER.ROUTES[0])
        bad["phase138"]["range_constants_validated"] = 1
        with self.assertRaises(RUNNER.Phase138ContractError):
            RUNNER.validate_structural_summary(RUNNER.ROUTES[0], bad)

        bad = _summary(RUNNER.ROUTES[0])
        bad["phase138"]["same_satellite_state"] = False
        with self.assertRaises(RUNNER.Phase138ContractError):
            RUNNER.validate_structural_summary(RUNNER.ROUTES[0], bad)

        bad = _summary(RUNNER.ROUTES[0])
        bad["phase138"]["adjustment_application_passes"] = 2
        with self.assertRaises(RUNNER.Phase138ContractError):
            RUNNER.validate_structural_summary(RUNNER.ROUTES[0], bad)

    def test_family_legacy_read_and_recipe_violations_fail_closed(self) -> None:
        bad = _summary(RUNNER.ROUTES[0])
        bad["phase135"]["legacy_factor_counts"]["ordinary_tdcp"] = 1
        with self.assertRaises(RUNNER.Phase138ContractError):
            RUNNER.validate_structural_summary(RUNNER.ROUTES[0], bad)

        bad = _summary(RUNNER.ROUTES[0])
        bad["read_accounting"]["truth_reads"] = 1
        with self.assertRaises(RUNNER.Phase138ContractError):
            RUNNER.validate_structural_summary(RUNNER.ROUTES[0], bad)

        bad = copy.deepcopy(_summary(RUNNER.ROUTES[0]))
        bad["selectors"]["phase120_official_tdcp_resl_atmosphere_cancellation"] = True
        with self.assertRaises(RUNNER.Phase138ContractError):
            RUNNER.validate_structural_summary(RUNNER.ROUTES[0], bad)


if __name__ == "__main__":
    unittest.main()
