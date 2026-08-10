"""Benchmark comparison and report test cases."""

from ._support import *  # noqa: F401,F403

__all__ = [
    "DrivingComparisonHelpersTest",
    "ScorecardHelpersTest",
    "ScorecardRenderTest",
    "SegmentedBenchmarkTest",
]


class DrivingComparisonHelpersTest(unittest.TestCase):
    @staticmethod
    def matched_epoch(tow: float, horiz_error_m: float, up_m: float, status: int) -> comparison.MatchedEpoch:
        return comparison.MatchedEpoch(
            tow=tow,
            traj_east_m=tow,
            traj_north_m=0.0,
            traj_up_m=0.0,
            east_m=horiz_error_m,
            north_m=0.0,
            up_m=up_m,
            horiz_error_m=horiz_error_m,
            status=status,
        )

    def test_pair_epochs_builds_unique_common_epoch_pairs(self) -> None:
        lib = [
            self.matched_epoch(0.0, 0.7, 0.4, 4),
            self.matched_epoch(1.0, 0.8, 0.3, 3),
            self.matched_epoch(2.0, 0.9, 0.2, 3),
            self.matched_epoch(4.0, 1.0, 0.1, 1),
        ]
        rtklib = [
            self.matched_epoch(1.0, 0.6, 0.5, 1),
            self.matched_epoch(2.0, 0.7, 0.6, 2),
            self.matched_epoch(3.0, 0.8, 0.7, 5),
        ]

        pairs = comparison.pair_epochs(lib, rtklib, tolerance_s=0.11)

        self.assertEqual([round(pair.tow, 1) for pair in pairs], [1.0, 2.0])

        lib_common, rt_common = comparison.summarize_common_epochs(
            pairs,
            lib_fixed_status=4,
            rtklib_fixed_status=1,
        )
        self.assertEqual(lib_common["epochs"], 2)
        self.assertEqual(rt_common["epochs"], 2)
        self.assertAlmostEqual(lib_common["median_h_m"], 0.85)
        self.assertAlmostEqual(rt_common["median_h_m"], 0.65)

    def test_ppc_coverage_quality_groups_status_and_bad_segments(self) -> None:
        matches = [
            self.matched_epoch(0.0, 0.1, 0.1, 4),
            self.matched_epoch(1.0, 10.0, 1.0, 3),
            self.matched_epoch(2.0, 20.0, 2.0, 3),
            self.matched_epoch(2.5, 30.0, 3.0, 1),
            self.matched_epoch(3.0, 0.1, 0.1, 4),
        ]

        status_rows = ppc_coverage_quality.summarize_by_status(matches, reference_count=5)
        status_by_name = {row["status"]: row for row in status_rows}
        self.assertEqual(status_by_name["FIXED"]["epochs"], 2)
        self.assertEqual(status_by_name["FLOAT"]["epochs"], 2)
        self.assertEqual(status_by_name["SPP"]["epochs"], 1)
        self.assertEqual(status_by_name["FIXED"]["ppc_score_3d_50cm_epochs"], 2)

        global_p95, contribution = ppc_coverage_quality.p95_contribution_by_status(matches)
        self.assertGreater(global_p95, 20.0)
        self.assertEqual(contribution[0]["status"], "SPP")
        self.assertEqual(contribution[0]["epochs"], 1)

        segments = ppc_coverage_quality.bad_segments(
            matches,
            threshold_m=15.0,
            max_gap_s=0.6,
        )
        self.assertEqual(len(segments), 1)
        self.assertEqual(segments[0]["epochs"], 2)
        self.assertEqual(segments[0]["statuses"], ["FLOAT", "SPP"])
        self.assertEqual(segments[0]["status_counts"], {"FLOAT": 1, "SPP": 1})
        self.assertEqual(segments[0]["dominant_status"], "FLOAT")
        self.assertEqual(segments[0]["previous_fixed_tow_s"], 0.0)
        self.assertEqual(segments[0]["next_fixed_tow_s"], 3.0)
        self.assertEqual(segments[0]["fixed_anchor_gap_s"], 3.0)
        self.assertEqual(segments[0]["fixed_anchor_distance_m"], 3.0)
        self.assertEqual(segments[0]["fixed_anchor_speed_mps"], 1.0)
        self.assertEqual(segments[0]["fixed_anchor_bridge_residual_max_m"], 0.0)

        reference = [
            comparison.ReferenceEpoch(2300, 0.0, 0.0, 0.0, 0.0, np.array([0.0, 0.0, 0.0])),
            comparison.ReferenceEpoch(2300, 1.0, 0.0, 0.0, 0.0, np.array([10.0, 0.0, 0.0])),
            comparison.ReferenceEpoch(2300, 2.0, 0.0, 0.0, 0.0, np.array([20.0, 0.0, 0.0])),
            comparison.ReferenceEpoch(2300, 3.0, 0.0, 0.0, 0.0, np.array([40.0, 0.0, 0.0])),
        ]
        lib_solution = [
            comparison.SolutionEpoch(
                2300,
                1.0,
                0.0,
                0.0,
                0.0,
                np.array([10.2, 0.0, 0.0]),
                3,
                12,
                4.0,
                100.0,
                2,
                14,
                7,
                7,
                0,
                0.25,
                4.0,
                0.25,
                4.0,
                56.0,
                4.0,
                0,
            ),
            comparison.SolutionEpoch(
                2300,
                2.0,
                0.0,
                0.0,
                0.0,
                np.array([21.0, 0.0, 0.0]),
                3,
                12,
                12.0,
                101.0,
                2,
                16,
                8,
                8,
                1,
                2.5,
                31.0,
                1.2,
                20.0,
                128.0,
                8.0,
                1,
            ),
        ]
        rtklib_solution = [
            comparison.SolutionEpoch(2300, 1.0, 0.0, 0.0, 0.0, np.array([11.0, 0.0, 0.0]), 2, 12),
            comparison.SolutionEpoch(2300, 2.0, 0.0, 0.0, 0.0, np.array([20.1, 0.0, 0.0]), 1, 12),
        ]
        spp_solution = [
            comparison.SolutionEpoch(2300, 1.0, 0.0, 0.0, 0.0, np.array([10.3, 0.0, 0.0]), 1, 12),
            comparison.SolutionEpoch(2300, 2.0, 0.0, 0.0, 0.0, np.array([20.1, 0.0, 0.0]), 1, 12),
        ]
        lib_records = ppc_metrics.ppc_official_segment_records(reference, lib_solution, 0.25)
        rtklib_records = ppc_metrics.ppc_official_segment_records(reference, rtklib_solution, 0.25)
        spp_records = ppc_metrics.ppc_official_segment_records(reference, spp_solution, 0.25)

        loss_by_state = {
            row["score_state"]: row
            for row in ppc_coverage_quality.official_loss_by_state(lib_records)
        }
        self.assertEqual(loss_by_state["scored"]["distance_m"], 10.0)
        self.assertEqual(loss_by_state["high_error"]["distance_m"], 10.0)
        self.assertEqual(loss_by_state["no_solution"]["distance_m"], 20.0)

        high_error_by_status = {
            row["status"]: row
            for row in ppc_coverage_quality.official_loss_by_status(
                lib_records,
                ppc_coverage_quality.status_name,
                ("high_error",),
            )
        }
        self.assertEqual(high_error_by_status["FLOAT"]["distance_m"], 10.0)
        self.assertEqual(high_error_by_status["FLOAT"]["median_ratio"], 12.0)
        self.assertEqual(high_error_by_status["FLOAT"]["ratio_ge_10_distance_m"], 10.0)
        self.assertEqual(high_error_by_status["FLOAT"]["median_rtk_update_observations"], 16.0)
        self.assertEqual(high_error_by_status["FLOAT"]["median_rtk_prefit_rms_m"], 2.5)
        self.assertEqual(high_error_by_status["FLOAT"]["p95_rtk_prefit_max_m"], 31.0)
        self.assertEqual(high_error_by_status["FLOAT"]["median_rtk_update_nis_per_obs"], 8.0)
        self.assertEqual(high_error_by_status["FLOAT"]["rtk_update_nis_rejected_segments"], 1)
        self.assertEqual(high_error_by_status["FLOAT"]["rtk_update_nis_rejected_distance_m"], 10.0)
        rtk_diagnostics_by_state = {
            (row["status"], row["score_state"]): row
            for row in ppc_coverage_quality.official_rtk_update_diagnostics_by_state(
                lib_records,
                ppc_coverage_quality.status_name,
            )
        }
        self.assertEqual(rtk_diagnostics_by_state[("FLOAT", "scored")]["distance_m"], 10.0)
        self.assertEqual(rtk_diagnostics_by_state[("FLOAT", "high_error")]["distance_m"], 10.0)
        self.assertEqual(
            rtk_diagnostics_by_state[("FLOAT", "scored")]["median_rtk_prefit_rms_m"],
            0.25,
        )
        self.assertEqual(
            rtk_diagnostics_by_state[("FLOAT", "high_error")]["median_rtk_prefit_rms_m"],
            2.5,
        )
        self.assertEqual(
            rtk_diagnostics_by_state[("FLOAT", "high_error")][
                "median_rtk_update_nis_per_obs"
            ],
            8.0,
        )
        unscored_by_status = {
            row["status"]: row
            for row in ppc_coverage_quality.official_loss_by_status(
                lib_records,
                ppc_coverage_quality.status_name,
            )
        }
        self.assertEqual(unscored_by_status["NO_SOLUTION"]["distance_m"], 20.0)

        official_segments = ppc_coverage_quality.official_loss_segments(
            lib_records,
            ppc_coverage_quality.status_name,
        )
        self.assertEqual(len(official_segments), 1)
        self.assertEqual(official_segments[0]["distance_m"], 30.0)
        self.assertEqual(official_segments[0]["dominant_score_state"], "no_solution")
        self.assertEqual(official_segments[0]["status_counts"], {"NO_SOLUTION": 1, "FLOAT": 1})

        combined = ppc_coverage_quality.official_combined_records(lib_records, rtklib_records)
        delta_by_bucket = {
            row["bucket"]: row
            for row in ppc_coverage_quality.official_delta_by_bucket(combined)
        }
        self.assertEqual(delta_by_bucket["gnssplusplus_gain"]["score_delta_pct"], 25.0)
        self.assertEqual(delta_by_bucket["rtklib_gain"]["score_delta_pct"], -25.0)
        best_of_score = ppc_coverage_quality.official_best_of_lib_rtklib_score(
            ppc_metrics.ppc_official_distance_score(reference, lib_solution, 0.25),
            combined,
        )
        self.assertEqual(best_of_score["score_distance_m"], 20.0)
        self.assertEqual(best_of_score["score_pct"], 50.0)
        self.assertEqual(best_of_score["rtklib_additional_distance_m"], 10.0)
        self.assertEqual(best_of_score["remaining_unscored_distance_m"], 20.0)
        float_spp_sweep = ppc_coverage_quality.official_float_spp_divergence_sweep(
            lib_records,
            spp_records,
            lib_solution,
            spp_solution,
            [0.5],
        )
        self.assertEqual(float_spp_sweep[0]["threshold_m"], 0.5)
        self.assertEqual(float_spp_sweep[0]["score_distance_m"], 20.0)
        self.assertEqual(float_spp_sweep[0]["score_delta_distance_m"], 10.0)
        self.assertEqual(float_spp_sweep[0]["recovered_distance_m"], 10.0)
        self.assertEqual(combined[1]["lib_ratio"], 12.0)
        self.assertEqual(combined[1]["lib_baseline_m"], 101.0)
        self.assertEqual(combined[1]["lib_rtk_update_observations"], 16)
        self.assertEqual(combined[1]["lib_rtk_update_prefit_residual_max_m"], 31.0)
        self.assertEqual(
            combined[1]["lib_rtk_update_normalized_innovation_squared_per_observation"],
            8.0,
        )
        self.assertEqual(combined[1]["lib_rtk_update_rejected_by_innovation_gate"], 1)


class ScorecardHelpersTest(unittest.TestCase):
    def test_ratio_text_handles_zero_baseline(self) -> None:
        self.assertEqual(scorecard.ratio_text(0.0, 0.0), "1.0x")
        self.assertEqual(scorecard.ratio_text(5.0, 0.0), "inf")
        self.assertEqual(scorecard.ratio_text(6.0, 3.0), "2.0x")

    def test_improvement_text_handles_zero_baseline(self) -> None:
        self.assertEqual(scorecard.improvement_text(1.0, 0.0), "n/a")
        self.assertEqual(scorecard.improvement_text(1.0, 4.0), "75%")

    def test_ppc_scorecard_loads_coverage_matrix_summary(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_scorecard_summary_") as temp_dir:
            summary_json = Path(temp_dir) / "summary.json"
            summary_json.write_text(
                json.dumps(
                    {
                        "runs": [
                            {
                                "key": "tokyo_run1",
                                "metrics": {
                                    "positioning_rate_pct": 86.2,
                                    "fix_rate_pct": 48.6,
                                    "ppc_official_score_pct": 42.0,
                                },
                                "rtklib": {
                                    "positioning_rate_pct": 66.3,
                                    "fix_rate_pct": 30.5,
                                    "ppc_official_score_pct": 21.0,
                                },
                                "delta_vs_rtklib": {
                                    "positioning_rate_pct": 19.9,
                                    "ppc_official_score_pct": 21.0,
                                    "ppc_score_3d_50cm_ref_pct": 35.6,
                                    "p95_h_m": -6.97,
                                },
                            }
                        ]
                    }
                ),
                encoding="utf-8",
            )

            runs = ppc_rtk_scorecard.runs_from_summary(summary_json)

            self.assertEqual(len(runs), 1)
            self.assertEqual(runs[0].label, "Tokyo r1")
            self.assertEqual(runs[0].positioning_delta_pct, 19.9)
            self.assertEqual(runs[0].lib_fix_pct - runs[0].rtklib_fix_pct, 18.1)
            self.assertEqual(runs[0].official_score_delta_pct, 21.0)


class ScorecardRenderTest(unittest.TestCase):
    def write_reference_csv(
        self,
        path: Path,
        rows: list[tuple[int, float, float, float, float]],
    ) -> None:
        with path.open("w", newline="", encoding="ascii") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                [
                    "gps_tow_s",
                    "gps_week",
                    "lat_deg",
                    "lon_deg",
                    "height_m",
                    "ecef_x_m",
                    "ecef_y_m",
                    "ecef_z_m",
                ]
            )
            for week, tow, lat, lon, height in rows:
                ecef = comparison.llh_to_ecef(lat, lon, height)
                writer.writerow(
                    [tow, week, lat, lon, height, ecef[0], ecef[1], ecef[2]]
                )

    def write_lib_pos(
        self,
        path: Path,
        rows: list[tuple[int, float, float, float, float, int]],
    ) -> None:
        with path.open("w", encoding="ascii") as handle:
            handle.write("% synthetic libgnss++ solution\n")
            for week, tow, lat, lon, height, status in rows:
                ecef = comparison.llh_to_ecef(lat, lon, height)
                handle.write(
                    f"{week} {tow:.1f} {ecef[0]:.6f} {ecef[1]:.6f} {ecef[2]:.6f} "
                    f"{lat:.9f} {lon:.9f} {height:.4f} {status} 10 1.0\n"
                )

    def write_rtklib_pos(
        self,
        path: Path,
        rows: list[tuple[int, float, float, float, float, int]],
    ) -> None:
        with path.open("w", encoding="ascii") as handle:
            handle.write("% synthetic rtklib solution\n")
            for week, tow, lat, lon, height, status in rows:
                handle.write(
                    f"{week} {tow:.1f} {lat:.9f} {lon:.9f} {height:.4f} {status} 10\n"
                )

    def test_scorecard_main_renders_png_with_zero_fix_baseline(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_scorecard_test_") as temp_dir:
            temp_root = Path(temp_dir)
            reference_csv = temp_root / "reference.csv"
            lib_pos = temp_root / "lib.pos"
            rtklib_pos = temp_root / "rtklib.pos"
            output_png = temp_root / "scorecard.png"

            rows = [
                (2000, 0.0, 35.0000000, 139.0000000, 10.0),
                (2000, 1.0, 35.0000100, 139.0000100, 10.2),
                (2000, 2.0, 35.0000200, 139.0000200, 10.4),
            ]
            self.write_reference_csv(reference_csv, rows)
            self.write_lib_pos(
                lib_pos,
                [
                    (2000, 0.0, 35.0000000, 139.0000000, 10.0, 4),
                    (2000, 1.0, 35.0000102, 139.0000102, 10.3, 4),
                    (2000, 2.0, 35.0000201, 139.0000201, 10.5, 4),
                ],
            )
            self.write_rtklib_pos(
                rtklib_pos,
                [
                    (2000, 0.0, 35.0000000, 139.0000000, 10.0, 5),
                    (2000, 1.0, 35.0000100, 139.0000100, 10.2, 5),
                    (2000, 2.0, 35.0000200, 139.0000200, 10.4, 5),
                ],
            )

            argv = [
                "generate_odaiba_scorecard.py",
                "--lib-pos",
                str(lib_pos),
                "--rtklib-pos",
                str(rtklib_pos),
                "--reference-csv",
                str(reference_csv),
                "--output",
                str(output_png),
                "--title",
                "Synthetic Odaiba",
            ]
            with mock.patch.object(sys, "argv", argv):
                with mock.patch.dict(os.environ, {"MPLBACKEND": "Agg"}, clear=False):
                    scorecard.main()

            self.assertTrue(output_png.exists())
            self.assertGreater(output_png.stat().st_size, 0)

    def test_social_card_main_renders_png(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_social_card_test_") as temp_dir:
            temp_root = Path(temp_dir)
            reference_csv = temp_root / "reference.csv"
            lib_pos = temp_root / "lib.pos"
            rtklib_pos = temp_root / "rtklib.pos"
            output_png = temp_root / "social_card.png"

            rows = [
                (2000, 0.0, 35.0000000, 139.0000000, 10.0),
                (2000, 1.0, 35.0000100, 139.0000100, 10.2),
                (2000, 2.0, 35.0000200, 139.0000200, 10.4),
            ]
            self.write_reference_csv(reference_csv, rows)
            self.write_lib_pos(
                lib_pos,
                [
                    (2000, 0.0, 35.0000000, 139.0000000, 10.0, 4),
                    (2000, 1.0, 35.0000102, 139.0000102, 10.3, 4),
                    (2000, 2.0, 35.0000201, 139.0000201, 10.5, 4),
                ],
            )
            self.write_rtklib_pos(
                rtklib_pos,
                [
                    (2000, 0.0, 35.0000000, 139.0000000, 10.0, 1),
                    (2000, 1.0, 35.0000100, 139.0000100, 10.2, 1),
                    (2000, 2.0, 35.0000200, 139.0000200, 10.4, 1),
                ],
            )

            argv = [
                "generate_odaiba_social_card.py",
                "--lib-pos",
                str(lib_pos),
                "--rtklib-pos",
                str(rtklib_pos),
                "--reference-csv",
                str(reference_csv),
                "--output",
                str(output_png),
                "--title",
                "Synthetic Odaiba",
            ]
            with mock.patch.object(sys, "argv", argv):
                with mock.patch.dict(os.environ, {"MPLBACKEND": "Agg"}, clear=False):
                    social_card.main()

            self.assertTrue(output_png.exists())
            self.assertGreater(output_png.stat().st_size, 0)
            try:
                from PIL import Image

                with Image.open(output_png) as image:
                    self.assertEqual(image.size, (1200, 630))
            except ModuleNotFoundError:
                pass

    def test_feature_overview_card_main_renders_png(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_feature_card_test_") as temp_dir:
            output_png = Path(temp_dir) / "feature_overview.png"

            argv = [
                "generate_feature_overview_card.py",
                "--output",
                str(output_png),
            ]
            with mock.patch.object(sys, "argv", argv):
                with mock.patch.dict(os.environ, {"MPLBACKEND": "Agg"}, clear=False):
                    feature_overview.main()

            self.assertTrue(output_png.exists())
            self.assertGreater(output_png.stat().st_size, 0)
            try:
                from PIL import Image

                with Image.open(output_png) as image:
                    self.assertEqual(image.size, (2240, 1376))
            except ModuleNotFoundError:
                pass

    def test_ppc_rtk_scorecard_main_renders_png(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_scorecard_test_") as temp_dir:
            output_png = Path(temp_dir) / "ppc_rtk_scorecard.png"

            argv = [
                "generate_ppc_rtk_scorecard.py",
                "--output",
                str(output_png),
            ]
            with mock.patch.object(sys, "argv", argv):
                with mock.patch.dict(os.environ, {"MPLBACKEND": "Agg"}, clear=False):
                    exit_code = ppc_rtk_scorecard.main()

            self.assertEqual(exit_code, 0)
            self.assertTrue(output_png.exists())
            self.assertGreater(output_png.stat().st_size, 0)
            try:
                from PIL import Image

                with Image.open(output_png) as image:
                    self.assertEqual(image.size, (1400, 750))
            except ModuleNotFoundError:
                pass

    def test_ppc_tail_cleanup_scorecard_main_renders_png(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_tail_scorecard_test_") as temp_dir:
            temp_root = Path(temp_dir)
            baseline_summary = temp_root / "baseline.json"
            cleanup_summary = temp_root / "cleanup.json"
            output_png = temp_root / "ppc_tail_cleanup_scorecard.png"

            def run_record(key: str, pos: float, official: float, p95: float, max_h: float, rejected: int) -> dict[str, object]:
                return {
                    "key": key,
                    "metrics": {
                        "positioning_rate_pct": pos,
                        "fix_rate_pct": 50.0,
                        "ppc_official_score_pct": official,
                        "p95_h_m": p95,
                        "max_h_m": max_h,
                    },
                    "delta_vs_rtklib": {
                        "p95_h_m": p95 - 30.0,
                    },
                    "guards": {
                        "nonfix_drift_guard": {"rejected_epochs": rejected},
                        "fixed_bridge_burst_guard": {"rejected_epochs": 2},
                    },
                }

            baseline_summary.write_text(
                json.dumps(
                    {
                        "runs": [
                            run_record("tokyo_run1", 90.0, 35.0, 34.0, 52.0, 0),
                            run_record("nagoya_run1", 88.0, 49.0, 12.0, 18.0, 0),
                        ]
                    }
                ),
                encoding="utf-8",
            )
            cleanup_summary.write_text(
                json.dumps(
                    {
                        "ratio": 2.4,
                        "nonfix_drift_max_residual": 4.0,
                        "fixed_bridge_burst_max_residual": 20.0,
                        "runs": [
                            run_record("tokyo_run1", 87.6, 34.9, 26.6, 47.3, 337),
                            run_record("nagoya_run1", 87.7, 48.9, 11.0, 16.5, 5),
                        ],
                    }
                ),
                encoding="utf-8",
            )

            argv = [
                "generate_ppc_tail_cleanup_scorecard.py",
                "--baseline-summary-json",
                str(baseline_summary),
                "--cleanup-summary-json",
                str(cleanup_summary),
                "--output",
                str(output_png),
            ]
            with mock.patch.object(sys, "argv", argv):
                with mock.patch.dict(os.environ, {"MPLBACKEND": "Agg"}, clear=False):
                    exit_code = ppc_tail_cleanup_scorecard.main()

            self.assertEqual(exit_code, 0)
            self.assertTrue(output_png.exists())
            self.assertGreater(output_png.stat().st_size, 0)
            try:
                from PIL import Image

                with Image.open(output_png) as image:
                    self.assertEqual(image.size, (1400, 760))
            except ModuleNotFoundError:
                pass

    def test_ppc_selector_validation_scorecard_main_renders_png(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_selector_scorecard_test_") as temp_dir:
            temp_root = Path(temp_dir)
            net_summary = temp_root / "net.json"
            robust_summary = temp_root / "robust.json"
            output_png = temp_root / "ppc_selector_validation_scorecard.png"

            def summary(net_m: float, precision_pct: float, nonnegative: int) -> dict[str, object]:
                return {
                    "aggregates": {
                        "fold_count": 2,
                        "holdout_selected_score_delta_distance_m": net_m,
                        "holdout_selector_vs_candidate_all_delta_m": net_m + 20.0,
                        "holdout_distance_precision_pct": precision_pct,
                        "nonnegative_holdout_runs": nonnegative,
                        "min_holdout_delta_m": min(1.0, net_m),
                        "holdout_selected_loss_distance_m": -2.0,
                    },
                    "folds": [
                        {
                            "holdout_run": "tokyo_run1",
                            "holdout_selected_score_delta_distance_m": net_m - 1.0,
                            "holdout_selected_loss_distance_m": -2.0,
                            "holdout_selected_segments": 3,
                        },
                        {
                            "holdout_run": "nagoya_run1",
                            "holdout_selected_score_delta_distance_m": 1.0,
                            "holdout_selected_loss_distance_m": 0.0,
                            "holdout_selected_segments": 1,
                        },
                    ],
                }

            net_summary.write_text(json.dumps(summary(10.0, 80.0, 2)), encoding="utf-8")
            robust_summary.write_text(json.dumps(summary(18.0, 95.0, 2)), encoding="utf-8")

            argv = [
                "generate_ppc_selector_validation_scorecard.py",
                "--summary",
                f"net={net_summary}",
                "--summary",
                f"robust={robust_summary}",
                "--best-label",
                "robust",
                "--output",
                str(output_png),
            ]
            with mock.patch.object(sys, "argv", argv):
                with mock.patch.dict(os.environ, {"MPLBACKEND": "Agg"}, clear=False):
                    exit_code = ppc_selector_scorecard.main()

            self.assertEqual(exit_code, 0)
            self.assertTrue(output_png.exists())
            self.assertGreater(output_png.stat().st_size, 0)
            try:
                from PIL import Image

                with Image.open(output_png) as image:
                    self.assertEqual(image.size, (1400, 760))
            except ModuleNotFoundError:
                pass

    def test_ppc_rtk_trajectory_main_renders_png(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_trajectory_test_") as temp_dir:
            temp_root = Path(temp_dir)
            reference_csv = temp_root / "reference.csv"
            lib_pos = temp_root / "lib.pos"
            rtklib_pos = temp_root / "rtklib.pos"
            output_png = temp_root / "ppc_rtk_trajectory.png"

            rows = [
                (2000, 0.0, 35.0000000, 139.0000000, 10.0),
                (2000, 1.0, 35.0000100, 139.0000100, 10.2),
                (2000, 2.0, 35.0000200, 139.0000200, 10.4),
                (2000, 3.0, 35.0000300, 139.0000300, 10.6),
            ]
            self.write_reference_csv(reference_csv, rows)
            self.write_lib_pos(
                lib_pos,
                [
                    (2000, 0.0, 35.0000000, 139.0000000, 10.0, 4),
                    (2000, 1.0, 35.0000101, 139.0000101, 10.2, 4),
                    (2000, 2.0, 35.0000202, 139.0000202, 10.5, 3),
                    (2000, 3.0, 35.0000300, 139.0000300, 10.6, 4),
                ],
            )
            self.write_rtklib_pos(
                rtklib_pos,
                [
                    (2000, 0.0, 35.0000000, 139.0000000, 10.0, 1),
                    (2000, 1.0, 35.0000110, 139.0000110, 10.5, 2),
                    (2000, 2.0, 35.0000200, 139.0000200, 10.4, 5),
                    (2000, 3.0, 35.0000300, 139.0000300, 10.6, 1),
                ],
            )

            argv = [
                "generate_ppc_rtk_trajectory.py",
                "--lib-pos",
                str(lib_pos),
                "--rtklib-pos",
                str(rtklib_pos),
                "--reference-csv",
                str(reference_csv),
                "--output",
                str(output_png),
                "--title",
                "Synthetic PPC trajectory",
            ]
            with mock.patch.object(sys, "argv", argv):
                with mock.patch.dict(os.environ, {"MPLBACKEND": "Agg"}, clear=False):
                    exit_code = ppc_rtk_trajectory.main()

            self.assertEqual(exit_code, 0)
            self.assertTrue(output_png.exists())
            self.assertGreater(output_png.stat().st_size, 0)

            official_png = temp_root / "ppc_rtk_trajectory_official.png"
            bad_segments_json = temp_root / "coverage_quality.json"
            bad_segments_json.write_text(
                json.dumps(
                    {
                        "bad_segments": [
                            {
                                "start_tow_s": 1.0,
                                "end_tow_s": 2.0,
                                "epochs": 2,
                                "max_h_m": 3.2,
                            }
                        ]
                    }
                ),
                encoding="utf-8",
            )
            argv = [
                "generate_ppc_rtk_trajectory.py",
                "--lib-pos",
                str(lib_pos),
                "--rtklib-pos",
                str(rtklib_pos),
                "--reference-csv",
                str(reference_csv),
                "--output",
                str(official_png),
                "--title",
                "Synthetic PPC official trajectory",
                "--color-mode",
                "official",
                "--bad-segments-json",
                str(bad_segments_json),
            ]
            with mock.patch.object(sys, "argv", argv):
                with mock.patch.dict(os.environ, {"MPLBACKEND": "Agg"}, clear=False):
                    exit_code = ppc_rtk_trajectory.main()

            self.assertEqual(exit_code, 0)
            self.assertTrue(official_png.exists())
            self.assertGreater(official_png.stat().st_size, 0)

    def test_architecture_diagram_main_renders_png(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_architecture_card_test_") as temp_dir:
            output_png = Path(temp_dir) / "architecture.png"

            argv = [
                "generate_architecture_diagram.py",
                "--output",
                str(output_png),
            ]
            with mock.patch.object(sys, "argv", argv):
                with mock.patch.dict(os.environ, {"MPLBACKEND": "Agg"}, clear=False):
                    architecture_diagram.main()

            self.assertTrue(output_png.exists())
            self.assertGreater(output_png.stat().st_size, 0)
            try:
                from PIL import Image

                with Image.open(output_png) as image:
                    self.assertEqual(image.size, (2240, 1408))
            except ModuleNotFoundError:
                pass


class SegmentedBenchmarkTest(unittest.TestCase):
    def write_reference_csv(self, path: Path, tows: list[float]) -> None:
        with path.open("w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(["gps_tow_s", "lat_deg", "lon_deg", "height_m"])
            for tow in tows:
                writer.writerow([tow, 0.0, 0.0, 0.0])

    def test_segmented_merge_uses_raw_child_solves(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_benchmark_test_") as temp_dir:
            temp_root = Path(temp_dir)
            reference_csv = temp_root / "reference.csv"
            self.write_reference_csv(reference_csv, [0.0, 1.0, 2.0, 3.0, 4.0])

            args = argparse.Namespace(
                reference_csv=reference_csv,
                segment_epochs=2,
                warmup_epochs=1,
                jobs=1,
                rover=temp_root / "rover.obs",
                base=temp_root / "base.obs",
                nav=temp_root / "base.nav",
                lib_pos=temp_root / "merged.pos",
                lib_kml=temp_root / "merged.kml",
                mode="kinematic",
                glonass_ar="autocal",
                scorecard_title="Test Title",
            )
            dispatcher = temp_root / "dispatcher.py"
            dispatcher.write_text("#!/usr/bin/env python3\n", encoding="ascii")

            solve_commands: list[list[str]] = []
            pos2kml_commands: list[list[str]] = []

            def fake_subprocess_run(command: list[str], check: bool) -> None:
                self.assertTrue(check)
                solve_commands.append(command)
                out_path = Path(command[command.index("--out") + 1])
                skip_epochs = int(command[command.index("--skip-epochs") + 1])
                max_epochs = int(command[command.index("--max-epochs") + 1])
                end_epoch = min(skip_epochs + max_epochs, 5)
                with out_path.open("w", encoding="ascii") as handle:
                    handle.write("% synthetic segment\n")
                    for tow in range(skip_epochs, end_epoch):
                        handle.write(
                            f"2000 {float(tow):.1f} 0 0 0 0 0 0 2 8 1.0\n"
                        )

            def fake_run_command(command: list[str]) -> None:
                pos2kml_commands.append(command)

            with mock.patch.object(benchmark.subprocess, "run", side_effect=fake_subprocess_run):
                with mock.patch.object(benchmark, "run_command", side_effect=fake_run_command):
                    benchmark.run_segmented_lib_solve(args, dispatcher)

            self.assertEqual(len(solve_commands), 3)
            for command in solve_commands:
                self.assertIn("--no-kinematic-post-filter", command)

            self.assertEqual(len(pos2kml_commands), 1)
            self.assertEqual(pos2kml_commands[0][1], str(dispatcher))
            self.assertEqual(pos2kml_commands[0][2], "pos2kml")

            merged_lines = [
                line.strip()
                for line in args.lib_pos.read_text(encoding="ascii").splitlines()
                if line.strip() and not line.startswith("%")
            ]
            self.assertEqual(len(merged_lines), 5)
            merged_tows = [float(line.split()[1]) for line in merged_lines]
            self.assertEqual(merged_tows, [0.0, 1.0, 2.0, 3.0, 4.0])

    def make_benchmark_args(self, temp_root: Path, **overrides: object) -> argparse.Namespace:
        paths = {
            "rover": temp_root / "rover.obs",
            "base": temp_root / "base.obs",
            "nav": temp_root / "base.nav",
            "reference_csv": temp_root / "reference.csv",
            "rtklib_config": temp_root / "rtklib.conf",
            "rtklib_bin": temp_root / "rnx2rtkp",
            "malib_config": temp_root / "malib.conf",
            "malib_bin": None,
            "lib_pos": temp_root / "lib.pos",
            "lib_kml": temp_root / "lib.kml",
            "rtklib_pos": temp_root / "rtklib.pos",
            "malib_pos": temp_root / "malib.pos",
            "comparison_png": temp_root / "comparison.png",
            "scorecard_png": temp_root / "scorecard.png",
            "social_card_png": temp_root / "social_card.png",
            "summary_json": temp_root / "summary.json",
        }
        for key, path in paths.items():
            if isinstance(path, Path):
                if key == "malib_pos":
                    continue
                path.write_text("synthetic\n", encoding="ascii")
        defaults: dict[str, object] = {
            **paths,
            "comparison_title": "Comparison",
            "scorecard_title": "Scorecard",
            "social_card_title": "Social Card",
            "require_all_epochs_min": 0,
            "require_common_epoch_pairs_min": 0,
            "require_lib_all_p95_h_max": None,
            "require_lib_common_median_h_max": None,
            "require_lib_common_p95_h_max": None,
            "mode": "kinematic",
            "glonass_ar": "autocal",
            "skip_epochs": 0,
            "max_epochs": -1,
            "segment_epochs": 0,
            "warmup_epochs": 300,
            "jobs": 4,
        }
        defaults.update(overrides)
        return argparse.Namespace(**defaults)

    def test_main_partial_window_skips_segmented_and_downstream_pipeline(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_benchmark_main_") as temp_dir:
            temp_root = Path(temp_dir)
            args = self.make_benchmark_args(
                temp_root,
                skip_epochs=12,
                max_epochs=34,
                segment_epochs=100,
            )

            commands: list[list[str]] = []
            with mock.patch.object(benchmark, "parse_args", return_value=args):
                with mock.patch.object(benchmark, "run_segmented_lib_solve") as segmented:
                    with mock.patch.object(
                        benchmark, "run_command", side_effect=commands.append
                    ):
                        exit_code = benchmark.main()

            self.assertEqual(exit_code, 0)
            segmented.assert_not_called()
            self.assertEqual(len(commands), 1)
            solve_command = commands[0]
            self.assertEqual(solve_command[2], "solve")
            self.assertIn("--no-kml", solve_command)
            self.assertIn("--skip-epochs", solve_command)
            self.assertIn("--max-epochs", solve_command)

    def test_main_full_segmented_run_uses_segment_solver_then_reports(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_benchmark_main_") as temp_dir:
            temp_root = Path(temp_dir)
            args = self.make_benchmark_args(
                temp_root,
                segment_epochs=500,
                warmup_epochs=50,
                jobs=2,
            )

            commands: list[list[str]] = []
            with mock.patch.object(benchmark, "parse_args", return_value=args):
                with mock.patch.object(benchmark, "run_segmented_lib_solve") as segmented:
                    with mock.patch.object(benchmark, "write_summary_json") as summary_writer:
                        with mock.patch.object(
                            benchmark, "enforce_summary_requirements"
                        ) as summary_checks:
                            with mock.patch.object(
                                benchmark, "run_command", side_effect=commands.append
                            ):
                                exit_code = benchmark.main()

            self.assertEqual(exit_code, 0)
            segmented.assert_called_once()
            summary_writer.assert_called_once_with(args)
            summary_checks.assert_called_once_with(summary_writer.return_value, args)
            self.assertEqual(len(commands), 4)
            self.assertEqual(commands[0][0], str(args.rtklib_bin))
            self.assertEqual(commands[1][2], "driving-compare")
            self.assertEqual(commands[2][2], "scorecard")
            self.assertEqual(commands[3][2], "social-card")

    def test_write_summary_json_exports_all_and_common_epoch_metrics(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_benchmark_summary_") as temp_dir:
            temp_root = Path(temp_dir)
            reference_csv = temp_root / "custom_reference.csv"
            lib_pos = temp_root / "custom_lib.pos"
            rtklib_pos = temp_root / "custom_rtklib.pos"
            malib_pos = temp_root / "custom_malib.pos"
            summary_json = temp_root / "custom_summary.json"

            rows = [
                (2000, 0.0, 35.0, 139.0, 10.0),
                (2000, 1.0, 35.00001, 139.00001, 10.1),
                (2000, 2.0, 35.00002, 139.00002, 10.2),
            ]
            ScorecardRenderTest().write_reference_csv(reference_csv, rows)
            ScorecardRenderTest().write_lib_pos(
                lib_pos,
                [
                    (2000, 0.0, 35.0, 139.0, 10.0, 4),
                    (2000, 1.0, 35.0000101, 139.0000101, 10.15, 3),
                    (2000, 2.0, 35.0000202, 139.0000202, 10.25, 4),
                ],
            )
            ScorecardRenderTest().write_rtklib_pos(
                rtklib_pos,
                [
                    (2000, 0.0, 35.0, 139.0, 10.0, 1),
                    (2000, 1.0, 35.0000102, 139.0000102, 10.12, 2),
                    (2000, 2.0, 35.0000203, 139.0000203, 10.22, 1),
                ],
            )
            ScorecardRenderTest().write_rtklib_pos(
                malib_pos,
                [
                    (2000, 0.0, 35.0, 139.0, 10.0, 1),
                    (2000, 1.0, 35.0000100, 139.0000100, 10.10, 1),
                    (2000, 2.0, 35.0000201, 139.0000201, 10.20, 2),
                ],
            )

            args = self.make_benchmark_args(
                temp_root,
                reference_csv=reference_csv,
                lib_pos=lib_pos,
                rtklib_pos=rtklib_pos,
                malib_pos=malib_pos,
                summary_json=summary_json,
            )

            returned_payload = benchmark.write_summary_json(args)

            payload = json.loads(summary_json.read_text(encoding="utf-8"))
            self.assertEqual(payload["dataset"], "UrbanNav Tokyo Odaiba")
            self.assertEqual(payload["common_epoch_pairs"], 3)
            self.assertEqual(payload["libgnss_all_epochs"]["epochs"], 3)
            self.assertEqual(payload["rtklib_all_epochs"]["epochs"], 3)
            self.assertEqual(payload["malib_all_epochs"]["epochs"], 3)
            self.assertIn("median_h_m", payload["libgnss_common_epochs"])
            self.assertIn("p95_h_m", payload["rtklib_common_epochs"])
            self.assertIn("median_h_m", payload["malib_common_epochs"])
            self.assertEqual(payload["malib_common_epoch_pairs"], 3)
            self.assertEqual(payload, returned_payload)

    def test_main_full_run_invokes_optional_malib_pipeline(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_benchmark_malib_") as temp_dir:
            temp_root = Path(temp_dir)
            malib_bin = temp_root / "malib_rnx2rtkp"
            malib_bin.write_text("synthetic\n", encoding="ascii")
            args = self.make_benchmark_args(temp_root, malib_bin=malib_bin)

            commands: list[list[str]] = []
            with mock.patch.object(benchmark, "parse_args", return_value=args):
                with mock.patch.object(benchmark, "write_summary_json") as summary_writer:
                    with mock.patch.object(
                        benchmark, "enforce_summary_requirements"
                    ) as summary_checks:
                        with mock.patch.object(
                            benchmark, "run_command", side_effect=commands.append
                        ):
                            exit_code = benchmark.main()

            self.assertEqual(exit_code, 0)
            summary_writer.assert_called_once_with(args)
            summary_checks.assert_called_once_with(summary_writer.return_value, args)
            self.assertEqual(len(commands), 6)
            self.assertEqual(commands[0][2], "solve")
            self.assertEqual(commands[1][0], str(args.rtklib_bin))
            self.assertEqual(commands[2][0], str(args.malib_bin))
            self.assertEqual(commands[3][2], "driving-compare")
            self.assertEqual(commands[4][2], "scorecard")
            self.assertEqual(commands[5][2], "social-card")

    def test_enforce_summary_requirements_passes_and_fails(self) -> None:
        payload = {
            "common_epoch_pairs": 8123,
            "libgnss_all_epochs": {"epochs": 11637, "p95_h_m": 7.583936},
            "libgnss_common_epochs": {"median_h_m": 0.733387, "p95_h_m": 5.941091},
        }
        passing_args = argparse.Namespace(
            require_all_epochs_min=11000,
            require_common_epoch_pairs_min=8000,
            require_lib_all_p95_h_max=8.0,
            require_lib_common_median_h_max=0.8,
            require_lib_common_p95_h_max=6.5,
        )
        benchmark.enforce_summary_requirements(payload, passing_args)

        failing_args = argparse.Namespace(
            require_all_epochs_min=12000,
            require_common_epoch_pairs_min=9000,
            require_lib_all_p95_h_max=7.0,
            require_lib_common_median_h_max=0.7,
            require_lib_common_p95_h_max=5.0,
        )
        with self.assertRaises(SystemExit) as context:
            benchmark.enforce_summary_requirements(payload, failing_args)

        message = str(context.exception)
        self.assertIn("all-epoch matched count", message)
        self.assertIn("common epoch pairs", message)
        self.assertIn("all-epoch p95_h", message)
        self.assertIn("common-epoch median_h", message)
        self.assertIn("common-epoch p95_h", message)
