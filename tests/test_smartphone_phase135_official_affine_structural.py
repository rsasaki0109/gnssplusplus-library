"""Launch-free Phase135 official affine structural contract tests.

The tests exercise only the static freeze, placeholder command snapshots, and
an in-memory synthetic structural summary.  They never materialize raw data,
launch the native solver, read solution rows or truth, or access MAT/PDC/
precomputed/Kaggle artifacts.
"""

from __future__ import annotations

import copy
import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = ROOT / (
    "apps/commands/benchmarks/"
    "gnss_smartphone_phase135_official_affine_structural.py"
)


def _load_runner():
    spec = importlib.util.spec_from_file_location(
        "phase135_structural_runner", RUNNER_PATH
    )
    if spec is None or spec.loader is None:
        raise RuntimeError("unable to load Phase135 structural runner")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


RUNNER = _load_runner()


def _summary(route: str) -> dict:
    """Return a complete opaque post-run summary for contract tests."""
    family = {
        "admitted_rows": 2,
        "affine_factors_inserted": 2,
        "key_order_exact": True,
        "finite_values": True,
    }
    doppler = {
        "official_rate_residual": True,
        "receiver_velocity_included": True,
        "explicit_sagnac_included": True,
        "source_provenance_complete": True,
        "source_provenance_finite": True,
        "los_convention": "-e=(receiver-satellite)/range",
        **family,
    }
    return {
        "route": route,
        "selectors": {
            "phase135_official_affine_measurement_family": True,
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
            "los_convention": "-e=(receiver-satellite)/range",
            "single_sagnac_representation": True,
            "sagnac_evaluations": 5,
            "geometry_rows": 5,
            "doppler": doppler,
            "pseudorange": dict(family),
            "ordinary_tdcp": dict(family),
            "legacy_factor_counts": {
                "pseudorange": 0,
                "doppler": 0,
                "ordinary_tdcp": 0,
            },
            "pose3_x_bridge": {"count": 2, "keys_exact": True},
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
            "raw_phone_gnss_reads": 0,
            "raw_phone_imu_reads": 0,
            "broadcast_nav_reads": 0,
            "raw_base_rinex_reads": 0,
            "truth_reads": 0,
            "solution_coordinate_reads": 0,
            "mat_pdc_precomputed_coordinate_reads": 0,
            "solver_invocations": 0,
            "accuracy_calculations": 0,
            "kaggle_access": 0,
        },
        "fallback": False,
        "rerun": False,
    }


class Phase135OfficialAffineStructuralTests(unittest.TestCase):
    def test_launch_free_static_pins_and_read_accounting(self) -> None:
        result = RUNNER.launch_free_validation()
        self.assertEqual(result["status"], "launch-free-qualified")
        self.assertEqual(result["routes"], list(RUNNER.ROUTES))
        self.assertEqual(result["solver_invocations"], 0)
        self.assertEqual(result["truth_reads"], 0)
        self.assertEqual(result["solution_coordinate_reads"], 0)
        self.assertEqual(result["mat_pdc_precomputed_reads"], 0)
        self.assertEqual(result["kaggle_access"], 0)

    def test_placeholder_commands_are_exactly_two_routes_in_order(self) -> None:
        manifest = RUNNER.read_json(RUNNER.MANIFEST, "manifest")
        self.assertEqual(manifest["routes"], list(RUNNER.ROUTES))
        for route in RUNNER.ROUTES:
            command = manifest["command_snapshots"][route]
            RUNNER.validate_command(route, command)
            self.assertEqual(command, RUNNER.command_template(route))

    def test_synthetic_structural_summary_passes_for_each_route(self) -> None:
        for route in RUNNER.ROUTES:
            RUNNER.validate_structural_summary(route, _summary(route))

    def test_affine_measurement_and_doppler_provenance_fail_closed(self) -> None:
        bad = _summary(RUNNER.ROUTES[0])
        bad["phase135"]["doppler"]["los_convention"] = (
            "e=(satellite-receiver)/range"
        )
        with self.assertRaises(RUNNER.Phase135ContractError):
            RUNNER.validate_structural_summary(RUNNER.ROUTES[0], bad)

        bad = _summary(RUNNER.ROUTES[0])
        bad["phase135"]["ordinary_tdcp"]["affine_factors_inserted"] = 1
        with self.assertRaises(RUNNER.Phase135ContractError):
            RUNNER.validate_structural_summary(RUNNER.ROUTES[0], bad)

    def test_selector_legacy_and_read_policy_fail_closed(self) -> None:
        bad = _summary(RUNNER.ROUTES[0])
        bad["selectors"]["phase131_canonical_correction_band_key"] = True
        with self.assertRaises(RUNNER.Phase135ContractError):
            RUNNER.validate_structural_summary(RUNNER.ROUTES[0], bad)

        bad = _summary(RUNNER.ROUTES[0])
        bad["read_accounting"]["truth_reads"] = 1
        with self.assertRaises(RUNNER.Phase135ContractError):
            RUNNER.validate_structural_summary(RUNNER.ROUTES[0], bad)

        bad = _summary(RUNNER.ROUTES[0])
        bad["solver"]["main"]["final_cost"] = 20.0
        with self.assertRaises(RUNNER.Phase135ContractError):
            RUNNER.validate_structural_summary(RUNNER.ROUTES[0], bad)


if __name__ == "__main__":
    unittest.main()
