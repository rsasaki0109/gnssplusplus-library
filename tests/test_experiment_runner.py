#!/usr/bin/env python3
"""Regression tests for experiment-driven PPP-AR tooling."""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
EXPERIMENTS_DIR = ROOT_DIR / "experiments" / "ppp_ar"
sys.path.insert(0, str(EXPERIMENTS_DIR))

import run_experiments as experiments  # noqa: E402


class PPPArExperimentsTest(unittest.TestCase):
    def test_load_suite_config_supports_single_case_compatibility(self) -> None:
        suite = experiments.load_suite_config(EXPERIMENTS_DIR / "input.example.toml")
        self.assertEqual(suite.label, "PPP-AR experiment suite")
        self.assertEqual(len(suite.cases), 1)
        self.assertEqual(suite.cases[0].case_key, "default")

    def test_load_input_config_uses_rinex_approx_position_when_reference_missing(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnsspp_ppp_ar_cfg_") as temp_dir:
            temp_path = Path(temp_dir)
            obs_path = temp_path / "sample.obs"
            obs_path.write_text(
                "\n".join(
                    [
                        "     3.02           OBSERVATION DATA    G                   RINEX VERSION / TYPE",
                        "  1234567.0000  2345678.0000  3456789.0000                  APPROX POSITION XYZ",
                        "                                                            END OF HEADER",
                    ]
                )
                + "\n",
                encoding="ascii",
            )
            nav_path = temp_path / "sample.nav"
            nav_path.write_text("", encoding="ascii")
            config_path = temp_path / "input.toml"
            config_path.write_text(
                "\n".join(
                    [
                        "[clas_ppp_experiment]",
                        f"obs = \"{obs_path}\"",
                        f"nav = \"{nav_path}\"",
                        "max_epochs = 10",
                        "strategies = [\"iflc_float_baseline\"]",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            loaded = experiments.load_input_config(config_path)

            self.assertEqual(loaded.reference_ecef, (1234567.0, 2345678.0, 3456789.0))

    def test_load_suite_config_assigns_case_specific_output_dirs(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnsspp_ppp_ar_suite_") as temp_dir:
            temp_path = Path(temp_dir)
            suite_path = temp_path / "suite.toml"
            suite_path.write_text(
                "\n".join(
                    [
                        "[clas_ppp_suite]",
                        'output_dir = "output/ppp_ar_experiments"',
                        'strategies = ["iflc_float_baseline"]',
                        "",
                        "[[clas_ppp_suite.cases]]",
                        'case_key = "first"',
                        'obs = "first.obs"',
                        'nav = "first.nav"',
                        'reference_ecef = [1.0, 2.0, 3.0]',
                        "",
                        "[[clas_ppp_suite.cases]]",
                        'case_key = "second"',
                        'obs = "second.obs"',
                        'nav = "second.nav"',
                        'reference_ecef = [4.0, 5.0, 6.0]',
                        "",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            suite = experiments.load_suite_config(suite_path)

            self.assertEqual(suite.cases[0].output_dir, temp_path / "output" / "ppp_ar_experiments" / "first")
            self.assertEqual(suite.cases[1].output_dir, temp_path / "output" / "ppp_ar_experiments" / "second")

    def test_load_strategies_exposes_selector_experiment_arms(self) -> None:
        strategies = experiments.load_strategies(EXPERIMENTS_DIR / "strategies.toml")
        self.assertEqual(
            sorted(strategies.keys()),
            [
                "iflc_float_baseline",
                "osr_ar_all_base_satellite_extend_pipeline",
                "osr_ar_balanced_pipeline",
                "osr_ar_base_only_code_bias_composition_pipeline",
                "osr_ar_base_only_phase_bias_composition_pipeline",
                "osr_ar_base_plus_network_code_bias_composition_pipeline",
                "osr_ar_base_plus_network_phase_bias_composition_pipeline",
                "osr_ar_clock_bound_atmos_phase_bias_pipeline",
                "osr_ar_clock_bound_phase_bias_pipeline",
                "osr_ar_clock_reference_time_pipeline",
                "osr_ar_close_30s_code_bias_bank_pipeline",
                "osr_ar_close_30s_phase_bias_bank_pipeline",
                "osr_ar_combined_priority_subtype_merge_pipeline",
                "osr_ar_compensation_only_value_pipeline",
                "osr_ar_coupled_code_phase_pipeline",
                "osr_ar_freshness_pipeline",
                "osr_ar_gridded_priority_subtype_merge_pipeline",
                "osr_ar_guarded_pipeline",
                "osr_ar_hybrid_pipeline",
                "osr_ar_indexed_only_residual_sampling_pipeline",
                "osr_ar_latest_preceding_code_bias_bank_pipeline",
                "osr_ar_latest_preceding_phase_bias_bank_pipeline",
                "osr_ar_mean_only_residual_sampling_pipeline",
                "osr_ar_message_reset_phase_bias_merge_pipeline",
                "osr_ar_network_locked_atmos_merge_pipeline",
                "osr_ar_network_row_driven_pipeline",
                "osr_ar_no_carry_atmos_merge_pipeline",
                "osr_ar_no_phase_bias_pipeline",
                "osr_ar_observation_reference_time_pipeline",
                "osr_ar_offset_only_subtype12_value_pipeline",
                "osr_ar_orbit_and_clock_source_pipeline",
                "osr_ar_orbit_clock_bias_pipeline",
                "osr_ar_orbit_clock_only_pipeline",
                "osr_ar_orbit_or_clock_source_pipeline",
                "osr_ar_phase_bias_only_value_pipeline",
                "osr_ar_pipeline",
                "osr_ar_planar_subtype12_value_pipeline",
                "osr_ar_polynomial_only_value_pipeline",
                "osr_ar_raw_phase_bias_pipeline",
                "osr_ar_repair_only_pipeline",
                "osr_ar_residual_only_value_pipeline",
                "osr_ar_row_first_value_second_pipeline",
                "osr_ar_same_30s_code_bias_bank_pipeline",
                "osr_ar_same_30s_phase_bias_bank_pipeline",
                "osr_ar_selected_mask_prune_phase_bias_merge_pipeline",
                "osr_ar_selected_satellite_base_extend_pipeline",
                "osr_ar_sis_continuity_only_pipeline",
                "osr_ar_strict_pipeline",
                "osr_ar_subtype5_priority_phase_bias_source_pipeline",
                "osr_ar_subtype6_priority_phase_bias_source_pipeline",
                "osr_float_all_base_satellite_extend_pipeline",
                "osr_float_balanced_pipeline",
                "osr_float_base_only_code_bias_composition_pipeline",
                "osr_float_base_only_phase_bias_composition_pipeline",
                "osr_float_base_plus_network_code_bias_composition_pipeline",
                "osr_float_base_plus_network_phase_bias_composition_pipeline",
                "osr_float_clock_bound_atmos_phase_bias_pipeline",
                "osr_float_clock_bound_phase_bias_pipeline",
                "osr_float_clock_reference_time_pipeline",
                "osr_float_close_30s_code_bias_bank_pipeline",
                "osr_float_close_30s_phase_bias_bank_pipeline",
                "osr_float_combined_priority_subtype_merge_pipeline",
                "osr_float_compensation_only_value_pipeline",
                "osr_float_coupled_code_phase_pipeline",
                "osr_float_freshness_pipeline",
                "osr_float_gridded_priority_subtype_merge_pipeline",
                "osr_float_guarded_pipeline",
                "osr_float_hybrid_pipeline",
                "osr_float_indexed_only_residual_sampling_pipeline",
                "osr_float_latest_preceding_code_bias_bank_pipeline",
                "osr_float_latest_preceding_phase_bias_bank_pipeline",
                "osr_float_mean_only_residual_sampling_pipeline",
                "osr_float_message_reset_phase_bias_merge_pipeline",
                "osr_float_network_locked_atmos_merge_pipeline",
                "osr_float_network_row_driven_pipeline",
                "osr_float_no_carry_atmos_merge_pipeline",
                "osr_float_no_phase_bias_pipeline",
                "osr_float_observation_reference_time_pipeline",
                "osr_float_offset_only_subtype12_value_pipeline",
                "osr_float_orbit_and_clock_source_pipeline",
                "osr_float_orbit_clock_bias_pipeline",
                "osr_float_orbit_clock_only_pipeline",
                "osr_float_orbit_or_clock_source_pipeline",
                "osr_float_phase_bias_only_value_pipeline",
                "osr_float_pipeline",
                "osr_float_planar_subtype12_value_pipeline",
                "osr_float_polynomial_only_value_pipeline",
                "osr_float_raw_phase_bias_pipeline",
                "osr_float_repair_only_pipeline",
                "osr_float_residual_only_value_pipeline",
                "osr_float_row_first_value_second_pipeline",
                "osr_float_same_30s_code_bias_bank_pipeline",
                "osr_float_same_30s_phase_bias_bank_pipeline",
                "osr_float_selected_mask_prune_phase_bias_merge_pipeline",
                "osr_float_selected_satellite_base_extend_pipeline",
                "osr_float_sis_continuity_only_pipeline",
                "osr_float_strict_pipeline",
                "osr_float_subtype5_priority_phase_bias_source_pipeline",
                "osr_float_subtype6_priority_phase_bias_source_pipeline",
            ],
        )
        self.assertEqual(strategies["osr_ar_pipeline"].design_style, "pipeline+solver")

    def test_ssr_variant_cache_key_distinguishes_code_bias_bank_policy(self) -> None:
        strategies = experiments.load_strategies(EXPERIMENTS_DIR / "strategies.toml")
        same_30s = experiments.ssr_variant_cache_key(
            strategies["osr_float_same_30s_code_bias_bank_pipeline"]
        )
        preceding = experiments.ssr_variant_cache_key(
            strategies["osr_float_latest_preceding_code_bias_bank_pipeline"]
        )
        self.assertNotEqual(same_30s, preceding)

    def test_ssr_variant_cache_key_distinguishes_bias_row_materialization_policy(self) -> None:
        strategies = experiments.load_strategies(EXPERIMENTS_DIR / "strategies.toml")
        selected = experiments.ssr_variant_cache_key(
            strategies["osr_float_selected_satellite_base_extend_pipeline"]
        )
        all_sat = experiments.ssr_variant_cache_key(
            strategies["osr_float_all_base_satellite_extend_pipeline"]
        )
        self.assertNotEqual(selected, all_sat)

    def test_ssr_variant_cache_key_distinguishes_row_construction_policy(self) -> None:
        strategies = experiments.load_strategies(EXPERIMENTS_DIR / "strategies.toml")
        coupled = experiments.ssr_variant_cache_key(
            strategies["osr_float_coupled_code_phase_pipeline"]
        )
        row_first = experiments.ssr_variant_cache_key(
            strategies["osr_float_row_first_value_second_pipeline"]
        )
        network = experiments.ssr_variant_cache_key(
            strategies["osr_float_network_row_driven_pipeline"]
        )
        strict = experiments.ssr_variant_cache_key(
            strategies["osr_float_strict_pipeline"]
        )
        self.assertNotEqual(coupled, row_first)
        self.assertNotEqual(coupled, network)
        self.assertNotEqual(row_first, network)
        self.assertNotEqual(coupled, strict)

    def test_new_row_construction_strategies_load_correctly(self) -> None:
        strategies = experiments.load_strategies(EXPERIMENTS_DIR / "strategies.toml")
        self.assertIn("osr_float_coupled_code_phase_pipeline", strategies)
        self.assertIn("osr_float_row_first_value_second_pipeline", strategies)
        self.assertIn("osr_float_network_row_driven_pipeline", strategies)
        self.assertIn("osr_ar_coupled_code_phase_pipeline", strategies)
        self.assertIn("osr_ar_row_first_value_second_pipeline", strategies)
        self.assertIn("osr_ar_network_row_driven_pipeline", strategies)
        coupled = strategies["osr_float_coupled_code_phase_pipeline"]
        self.assertEqual(coupled.compact_row_construction_policy, "coupled-code-phase")
        row_first = strategies["osr_float_row_first_value_second_pipeline"]
        self.assertEqual(row_first.compact_row_construction_policy, "row-first-value-second")
        network = strategies["osr_float_network_row_driven_pipeline"]
        self.assertEqual(network.compact_row_construction_policy, "network-row-driven")

    def test_structure_metrics_distinguish_monolith_and_pipeline(self) -> None:
        strategies = experiments.load_strategies(EXPERIMENTS_DIR / "strategies.toml")
        baseline = experiments.compute_structure_metrics(strategies["iflc_float_baseline"])
        pipeline = experiments.compute_structure_metrics(strategies["osr_ar_pipeline"])
        self.assertLess(baseline["module_count"], pipeline["module_count"])
        self.assertGreater(pipeline["extensibility_score"], baseline["extensibility_score"])

    def test_summarize_solution_computes_error_percentiles(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnsspp_ppp_ar_pos_") as temp_dir:
            pos_path = Path(temp_dir) / "solution.pos"
            pos_path.write_text(
                "\n".join(
                    [
                        "% week tow x y z q ns",
                        "2200 345600.0 10.0 0.0 0.0 0 0 0 6 8",
                        "2200 345601.0 11.0 0.0 0.0 0 0 0 5 8",
                        "2200 345602.0 13.0 0.0 0.0 0 0 0 6 8",
                    ]
                )
                + "\n",
                encoding="ascii",
            )

            summary = experiments.summarize_solution(pos_path, (10.0, 0.0, 0.0))

            self.assertEqual(summary["epochs"], 3)
            self.assertEqual(summary["fixed_epochs"], 2)
            self.assertAlmostEqual(summary["mean_3d_error_m"], 1.333333, places=6)
            self.assertAlmostEqual(summary["p95_3d_error_m"], 2.8, places=6)

    def test_render_markdown_lists_all_results(self) -> None:
        input_config = experiments.ExperimentInput(
            case_key="cl_static",
            dataset_label="CLAS static",
            obs=Path("/tmp/0627239Q.obs"),
            nav=Path("/tmp/sept_2019239.nav"),
            qzss_l6=Path("/tmp/2019239Q.l6"),
            ssr_csv=None,
            qzss_gps_week=2068,
            reference_ecef=(-1.0, 2.0, 3.0),
            max_epochs=60,
            output_dir=Path("/tmp/ppp_ar_experiments"),
            mode="static",
            estimate_troposphere=True,
            strategies=("iflc_float_baseline", "osr_float_pipeline", "osr_ar_pipeline"),
        )
        results = [
            {
                "strategy": "iflc_float_baseline",
                "label": "IFLC Float Baseline",
                "design_style": "monolithic",
                "status": "baseline",
                "ppp_solution_rate_pct": 100.0,
                "ppp_fixed_solutions": 0,
                "mean_3d_error_m": 3.9,
                "p95_3d_error_m": 4.2,
                "wall_time_s": 1.2,
                "readability_rating": "medium",
                "extensibility_rating": "low",
                "module_count": 1,
                "implementation_loc": 1200,
                "max_file_loc": 1200,
                "command_arg_count": 0,
                "claslib_reference": "control",
                "promotion_stage": "stable_control",
                "promotion_gate_pass": False,
                "promotion_rationale": "Stable control arm used to judge candidate strategies.",
                "comparison_to_baseline": {
                    "mean_3d_error_m_delta": 0.0,
                    "p95_3d_error_m_delta": 0.0,
                    "fixed_solution_delta": 0,
                    "solution_rate_pct_delta": 0.0,
                },
            },
            {
                "strategy": "osr_float_pipeline",
                "label": "OSR Float Pipeline",
                "design_style": "pipeline",
                "status": "candidate",
                "ppp_solution_rate_pct": 100.0,
                "ppp_fixed_solutions": 0,
                "mean_3d_error_m": 3.1,
                "p95_3d_error_m": 3.4,
                "wall_time_s": 1.4,
                "readability_rating": "medium",
                "extensibility_rating": "high",
                "module_count": 3,
                "implementation_loc": 2300,
                "max_file_loc": 900,
                "command_arg_count": 2,
                "claslib_reference": "osr",
                "promotion_stage": "trial_candidate",
                "promotion_gate_pass": True,
                "promotion_rationale": "Beats the stable control on the shared accuracy gates and is ready for wider trials.",
                "comparison_to_baseline": {
                    "mean_3d_error_m_delta": 0.8,
                    "p95_3d_error_m_delta": 0.8,
                    "fixed_solution_delta": 0,
                    "solution_rate_pct_delta": 0.0,
                },
            },
        ]
        suite = experiments.ExperimentSuite(label="CLAS suite", cases=(input_config,))
        case_payloads = [
            {
                "case_key": "cl_static",
                "dataset_label": input_config.dataset_label,
                "obs": input_config.obs.name,
                "nav": input_config.nav.name,
                "qzss_l6": input_config.qzss_l6.name,
                "ssr_csv": "expanded_ssr.csv",
                "max_epochs": input_config.max_epochs,
                "mode": input_config.mode,
                "estimate_troposphere": input_config.estimate_troposphere,
                "results": results,
            }
        ]
        suite_summary = experiments.summarize_suite_results(case_payloads)

        markdown = experiments.render_markdown(
            suite,
            EXPERIMENTS_DIR / "strategies.toml",
            case_payloads,
            suite_summary,
        )

        self.assertIn("# PPP-AR Experiments", markdown)
        self.assertIn("IFLC Float Baseline", markdown)
        self.assertIn("OSR Float Pipeline", markdown)
        self.assertIn("Suite Summary", markdown)
        self.assertIn("Case Results", markdown)
        self.assertIn("Promotion Gate", markdown)
        self.assertIn("trial_candidate", markdown)

    def test_render_markdown_accepts_relative_strategy_path(self) -> None:
        input_config = experiments.ExperimentInput(
            case_key="cl_static",
            dataset_label="CLAS case",
            obs=Path("/tmp/0627239Q.obs"),
            nav=Path("/tmp/sept_2019239.nav"),
            qzss_l6=Path("/tmp/2019239Q.l6"),
            ssr_csv=None,
            qzss_gps_week=2068,
            reference_ecef=(-3957240.1233, 3310370.8778, 3737527.7041),
            output_dir=Path("/tmp/out"),
            max_epochs=120,
            mode="static",
            estimate_troposphere=True,
            strategies=("iflc_float_baseline",),
        )
        results = [
            {
                "strategy": "iflc_float_baseline",
                "label": "IFLC Float Baseline",
                "status": "baseline",
                "design_style": "monolithic",
                "ppp_solution_rate_pct": 99.2,
                "ppp_fixed_solutions": 0,
                "mean_3d_error_m": 4.54,
                "p95_3d_error_m": 4.64,
                "wall_time_s": 1.2,
                "readability_score": 12.0,
                "extensibility_score": 14.0,
                "readability_rating": "low",
                "extensibility_rating": "low",
                "module_count": 1,
                "implementation_loc": 1200,
                "max_file_loc": 1200,
                "command_arg_count": 0,
                "claslib_reference": "control",
                "promotion_stage": "stable_control",
                "promotion_gate_pass": False,
                "promotion_rationale": "Control arm.",
                "comparison_to_baseline": {},
            },
        ]
        suite = experiments.ExperimentSuite(label="CLAS suite", cases=(input_config,))
        case_payloads = [
            {
                "case_key": "cl_static",
                "dataset_label": input_config.dataset_label,
                "obs": input_config.obs.name,
                "nav": input_config.nav.name,
                "qzss_l6": input_config.qzss_l6.name,
                "ssr_csv": "expanded_ssr.csv",
                "max_epochs": input_config.max_epochs,
                "mode": input_config.mode,
                "estimate_troposphere": input_config.estimate_troposphere,
                "results": results,
            }
        ]
        suite_summary = experiments.summarize_suite_results(case_payloads)

        markdown = experiments.render_markdown(
            suite,
            Path("experiments/ppp_ar/strategies.toml"),
            case_payloads,
            suite_summary,
        )

        self.assertIn("experiments/ppp_ar/strategies.toml", markdown)

    def test_evaluate_promotion_readiness_marks_solver_arm_as_candidate(self) -> None:
        results = [
            {
                "strategy": "iflc_float_baseline",
                "label": "IFLC Float Baseline",
                "status": "baseline",
                "design_style": "monolithic",
                "ppp_solution_rate_pct": 99.2,
                "ppp_fixed_solutions": 0,
                "mean_3d_error_m": 4.54,
                "p95_3d_error_m": 4.64,
                "readability_score": 3.7,
                "extensibility_score": 4.8,
            },
            {
                "strategy": "osr_ar_pipeline",
                "label": "OSR AR Pipeline",
                "status": "candidate",
                "design_style": "pipeline+solver",
                "ppp_solution_rate_pct": 100.0,
                "ppp_fixed_solutions": 2,
                "mean_3d_error_m": 3.99,
                "p95_3d_error_m": 4.15,
                "readability_score": 63.2,
                "extensibility_score": 74.5,
            },
        ]

        evaluated = experiments.evaluate_promotion_readiness(results)
        solver_arm = evaluated[1]

        self.assertEqual(solver_arm["promotion_stage"], "promotion_candidate")
        self.assertTrue(solver_arm["promotion_gate_pass"])
        self.assertEqual(solver_arm["comparison_to_baseline"]["fixed_solution_delta"], 2)

    def test_summarize_suite_results_requires_repeated_wins(self) -> None:
        case_payloads = [
            {
                "case_key": "a",
                "results": [
                    {
                        "strategy": "iflc_float_baseline",
                        "label": "IFLC Float Baseline",
                        "status": "baseline",
                        "design_style": "monolithic",
                        "ppp_solution_rate_pct": 99.0,
                        "ppp_fixed_solutions": 0,
                        "mean_3d_error_m": 4.5,
                        "p95_3d_error_m": 4.8,
                        "wall_time_s": 5.0,
                        "readability_score": 10.0,
                        "extensibility_score": 12.0,
                        "readability_rating": "low",
                        "extensibility_rating": "low",
                        "promotion_gate_pass": False,
                        "comparison_to_baseline": {},
                    },
                    {
                        "strategy": "osr_ar_pipeline",
                        "label": "OSR AR Pipeline",
                        "status": "candidate",
                        "design_style": "pipeline+solver",
                        "ppp_solution_rate_pct": 100.0,
                        "ppp_fixed_solutions": 2,
                        "mean_3d_error_m": 4.0,
                        "p95_3d_error_m": 4.1,
                        "wall_time_s": 6.0,
                        "readability_score": 60.0,
                        "extensibility_score": 70.0,
                        "readability_rating": "medium",
                        "extensibility_rating": "medium",
                        "promotion_gate_pass": True,
                        "comparison_to_baseline": {
                            "solution_rate_pct_delta": 1.0,
                            "mean_3d_error_m_delta": 0.5,
                            "p95_3d_error_m_delta": 0.7,
                            "fixed_solution_delta": 2,
                        },
                    },
                ],
            },
            {
                "case_key": "b",
                "results": [
                    {
                        "strategy": "iflc_float_baseline",
                        "label": "IFLC Float Baseline",
                        "status": "baseline",
                        "design_style": "monolithic",
                        "ppp_solution_rate_pct": 99.1,
                        "ppp_fixed_solutions": 0,
                        "mean_3d_error_m": 4.4,
                        "p95_3d_error_m": 4.6,
                        "wall_time_s": 4.8,
                        "readability_score": 10.0,
                        "extensibility_score": 12.0,
                        "readability_rating": "low",
                        "extensibility_rating": "low",
                        "promotion_gate_pass": False,
                        "comparison_to_baseline": {},
                    },
                    {
                        "strategy": "osr_ar_pipeline",
                        "label": "OSR AR Pipeline",
                        "status": "candidate",
                        "design_style": "pipeline+solver",
                        "ppp_solution_rate_pct": 99.0,
                        "ppp_fixed_solutions": 0,
                        "mean_3d_error_m": 4.3,
                        "p95_3d_error_m": 4.5,
                        "wall_time_s": 6.1,
                        "readability_score": 60.0,
                        "extensibility_score": 70.0,
                        "readability_rating": "medium",
                        "extensibility_rating": "medium",
                        "promotion_gate_pass": False,
                        "comparison_to_baseline": {
                            "solution_rate_pct_delta": -0.1,
                            "mean_3d_error_m_delta": 0.1,
                            "p95_3d_error_m_delta": 0.1,
                            "fixed_solution_delta": 0,
                        },
                    },
                ],
            },
        ]

        summary = experiments.summarize_suite_results(case_payloads)
        solver_arm = next(item for item in summary if item["strategy"] == "osr_ar_pipeline")

        self.assertEqual(solver_arm["cases_run"], 2)
        self.assertEqual(solver_arm["cases_passed"], 1)
        self.assertEqual(solver_arm["promotion_stage"], "extended_trial")


if __name__ == "__main__":
    unittest.main()
