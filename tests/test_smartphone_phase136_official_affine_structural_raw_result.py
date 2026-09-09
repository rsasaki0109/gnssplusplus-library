"""Launch-free checks for the sealed Phase136 structural raw result.

The test reads only the tracked result artifact.  It does not read raw input,
native output, solution rows, truth, MAT/PDC/precomputed data, or Kaggle.
"""

from __future__ import annotations

import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
RESULT = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase136_official_affine_structural_raw_result_v1.json"
)


class Phase136OfficialAffineStructuralRawResultTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.result = json.loads(RESULT.read_text(encoding="utf-8"))

    def test_seal_identity_and_route_order(self) -> None:
        self.assertEqual(
            self.result["schema_version"],
            "smartphone-r5-phase136-official-affine-structural-raw-result.v1",
        )
        self.assertEqual(self.result["phase"], 136)
        self.assertEqual(self.result["recipe_phase"], 135)
        self.assertEqual(self.result["status"], "sealed-structural-raw-result-no-accuracy")
        self.assertEqual(self.result["authorization_commit"], "445f53b44b186c111a926f4e9b5daf88ef2ffd07")
        self.assertEqual(self.result["route_order"], ["MTV-A", "LAX-T"])
        self.assertEqual(self.result["runs_per_route"], 1)
        self.assertFalse(self.result["policy"]["solution_coordinate_interpretation"])

    def test_each_route_has_one_raw_solver_and_complete_structural_progress(self) -> None:
        expected = {
            "MTV-A": (2159, 43259, 20748, 31269, 536, 12),
            "LAX-T": (1466, 30664, 8942, 14012, 581, 12),
        }
        for route, (epochs, p_count, d_count, tdcp_count, stage_iters, main_iters) in expected.items():
            with self.subTest(route=route):
                item = self.result["route_results"][route]
                self.assertEqual(item["return_code"], 0)
                self.assertEqual(item["solver_invocations"], 1)
                self.assertEqual(item["input_read_counts"], {
                    "device_gnss.csv": 1,
                    "device_imu.csv": 1,
                    "brdc.nav": 1,
                    "base.obs": 1,
                })
                stage = item["gnss_first"]
                main = item["main"]
                family = item["affine_families"]
                self.assertEqual(stage["epochs"], epochs)
                self.assertEqual(stage["iterations"], stage_iters)
                self.assertGreater(stage["accepted_outer_iterations"], 0)
                self.assertTrue(stage["strict_cost_decrease"])
                self.assertLess(stage["final_cost"], stage["initial_cost"])
                self.assertEqual(main["iterations"], main_iters)
                self.assertGreater(main["accepted_outer_iterations"], 0)
                self.assertTrue(main["strict_cost_decrease"])
                self.assertLess(main["final_cost"], main["initial_cost"])
                self.assertEqual(main["linear_solver"], "MULTIFRONTAL_QR")
                self.assertEqual(main["elimination"], "EliminateQR")
                self.assertEqual(family["pseudorange_admitted_or_inserted"], p_count)
                self.assertEqual(family["doppler_admitted_or_inserted"], d_count)
                self.assertEqual(family["ordinary_tdcp_admitted_or_inserted"], tdcp_count)
                self.assertEqual(family["pose3_x_bridge"], epochs)
                self.assertTrue(family["configuration_valid"])
                self.assertTrue(family["single_sagnac_representation"])
                self.assertTrue(family["key_order_exact"])
                self.assertTrue(family["finite_jacobians_and_source_geometry"])

    def test_c7d_base_tdcp_and_output_gates(self) -> None:
        for route, item in self.result["route_results"].items():
            with self.subTest(route=route):
                stage = item["gnss_first"]
                c7d = item["c7_d_and_units"]
                base = item["raw_base"]
                tdcp = item["tdcp"]
                output = item["output"]
                self.assertEqual(stage["c7_dimension"], 7)
                self.assertEqual(stage["c_epoch_count"], stage["epochs"])
                self.assertEqual(stage["d_epoch_count"], stage["epochs"])
                self.assertEqual(stage["c_nonfinite_components"], 0)
                self.assertEqual(stage["d_nonfinite_count"], 0)
                self.assertTrue(stage["c_export_valid"])
                self.assertTrue(stage["d_export_valid"])
                self.assertTrue(stage["exact_epoch_alignment"])
                self.assertEqual(c7d["c_units"], "metres")
                self.assertEqual(c7d["d_units"], "metres_per_second")
                self.assertEqual(c7d["c0d_sigma_m"], 0.1)
                self.assertEqual(c7d["global_isb_state_count"], 0)
                self.assertTrue(base["applied"] and base["built"])
                self.assertEqual(base["correction_application_pass_count"], 1)
                self.assertTrue(base["correction_applied_exactly_once"])
                self.assertEqual(base["adopted_rows_corrected"], base["in_domain_rows"])
                self.assertEqual(base["adopted_rows_corrected"], base["finite_correction_rows"])
                self.assertTrue(base["no_extrapolation_or_endpoint_hold"])
                self.assertTrue(tdcp["enabled"])
                self.assertEqual(tdcp["factors_built"], tdcp["factors_inserted"])
                self.assertEqual(tdcp["factors_inserted"], tdcp["finite_residuals"])
                self.assertEqual(tdcp["nonfinite_residuals"], 0)
                self.assertEqual(tdcp["fixed_sigma_m"], 0.03)
                self.assertEqual(tdcp["official_huber_k"], 0.5)
                self.assertFalse(tdcp["base_or_double_difference_factors"])
                self.assertTrue(output["finite_coordinates"])
                self.assertTrue(output["atomic_publish"])
                self.assertTrue(output["expected_epoch_coverage"])
                self.assertEqual(output["output_epochs"], stage["epochs"])
                self.assertEqual(output["pixel5_offset_applications"], 1)

    def test_legacy_and_policy_accounting_are_fail_closed(self) -> None:
        for item in self.result["route_results"].values():
            legacy = item["legacy_family_evidence"]
            self.assertEqual(legacy["double_difference_carrier_factors"], 0)
            self.assertEqual(legacy["double_difference_pseudorange_factors"], 0)
            self.assertEqual(legacy["receiver_signal_bias_states"], 0)
            self.assertFalse(legacy["base_or_double_difference_tdcp"])
            self.assertFalse(legacy["standalone_carrier_ambiguity_factors"])
            self.assertFalse(legacy["native_pdc_state_bridge"])
            selectors = item["selectors"]
            self.assertTrue(selectors["native_pdc_imu_tdcp_no_bridge"])
            self.assertTrue(selectors["phase135_official_affine_measurement_family"])
            self.assertTrue(selectors["phase118_official_tdcp_huber_k"])
            self.assertTrue(selectors["phase107_raw_base_compensation"])
            for key in (
                "phase117_dynamic_tdcp_sigma",
                "phase120_official_tdcp_resl_atmosphere_cancellation",
                "phase126_134_compound",
                "additional_frequency_bands",
                "native_pdc_state_bridge",
                "native_direct_wls_ephemeral_c7d_main_seed",
            ):
                self.assertFalse(selectors[key])
        for key in (
            "truth_used",
            "mat_used",
            "pdc_used",
            "precomputed_coordinates_used",
            "accuracy_evaluation",
            "kaggle_access",
            "solution_publication",
            "solution_coordinate_interpretation",
            "fallback",
            "rerun",
            "repair",
        ):
            self.assertFalse(self.result["policy"][key])
        accounting = self.result["read_accounting"]
        self.assertEqual(accounting["raw_phone_gnss_reads"], 2)
        self.assertEqual(accounting["raw_phone_imu_reads"], 2)
        self.assertEqual(accounting["broadcast_navigation_reads"], 2)
        self.assertEqual(accounting["raw_base_rinex_reads"], 2)
        self.assertEqual(accounting["native_solver_invocations"], 2)
        for key in (
            "truth_reads",
            "solution_coordinate_reads",
            "accuracy_calculations",
            "mat_pdc_precomputed_coordinate_reads",
            "kaggle_or_token_access",
            "reruns_fallbacks_repairs_sweeps",
        ):
            self.assertEqual(accounting[key], 0)
        self.assertIn("no truth/accuracy evaluation", self.result["next_action"])


if __name__ == "__main__":
    unittest.main()
