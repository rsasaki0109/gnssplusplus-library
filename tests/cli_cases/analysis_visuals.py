"""CLI regression cases for the AnalysisVisualsCases domain."""

from ._support import *  # noqa: F401,F403

class AnalysisVisualsCases:
    def test_spp_cli_processes_real_static_sample(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_spp_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            output_path = temp_root / "spp.pos"
            summary_path = temp_root / "spp_summary.json"

            result = self.run_gnss(
                "spp",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--out",
                str(output_path),
                "--summary-json",
                str(summary_path),
                "--robust-weighting",
                "--robust-threshold-sigma",
                "2.5",
                "--robust-min-weight",
                "0.1",
                "--adaptive-robust-weighting",
                "--adaptive-robust-activation-threshold-sigma",
                "3.0",
                "--adaptive-robust-min-tail-measurements",
                "2",
                "--adaptive-robust-min-tail-fraction",
                "0.08",
                "--max-position-jump-rate-mps",
                "50",
                "--max-position-jump-min-m",
                "20",
                "--max-epochs",
                "10",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("Processed epochs:", result.stdout)
            self.assertIn("Valid solutions:", result.stdout)
            self.assertTrue(output_path.exists())
            self.assertTrue(summary_path.exists())
            self.assertIn("LibGNSS++ Position Solution", output_path.read_text(encoding="ascii"))
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertTrue(payload["config"]["robust_weighting"])
            self.assertEqual(payload["config"]["robust_threshold_sigma"], 2.5)
            self.assertEqual(payload["config"]["robust_min_weight"], 0.1)
            self.assertTrue(payload["config"]["adaptive_robust_weighting"])
            self.assertEqual(payload["config"]["adaptive_robust_activation_threshold_sigma"], 3.0)
            self.assertEqual(payload["config"]["adaptive_robust_min_tail_measurements"], 2)
            self.assertEqual(payload["config"]["adaptive_robust_min_tail_fraction"], 0.08)
            self.assertTrue(payload["config"]["position_jump_gate"])
            self.assertEqual(payload["config"]["max_position_jump_rate_mps"], 50.0)
            self.assertEqual(payload["config"]["max_position_jump_min_m"], 20.0)
            self.assertIn("robust_weighted_measurements", payload["spp_qc"])
            self.assertIn("adaptive_robust_activations", payload["spp_qc"])
            self.assertIn("adaptive_robust_tail_measurements", payload["spp_qc"])
            self.assertIn("position_jump_gate_rejections", payload["spp_qc"])
    def test_visibility_cli_writes_csv_and_summary_for_static_data(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_visibility_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            csv_path = temp_root / "visibility.csv"
            summary_path = temp_root / "visibility.json"

            result = self.run_gnss(
                "visibility",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--csv",
                str(csv_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "5",
                "--min-elevation-deg",
                "5",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("Visibility summary:", result.stdout)
            self.assertIn("rows written:", result.stdout)
            self.assertTrue(csv_path.exists())
            self.assertTrue(summary_path.exists())

            csv_lines = csv_path.read_text(encoding="utf-8").splitlines()
            self.assertGreater(len(csv_lines), 1)
            self.assertEqual(
                csv_lines[0],
                "epoch_index,week,tow,satellite,system,signal,azimuth_deg,elevation_deg,snr_dbhz,"
                "has_pseudorange,has_carrier_phase,has_doppler",
            )
            first_row = csv_lines[1].split(",")
            self.assertEqual(len(first_row), 12)
            self.assertTrue(first_row[3].startswith(("G", "R", "E", "C", "J", "S", "I")))

            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["epochs_processed"], 5)
            self.assertGreater(payload["rows_written"], 0)
            self.assertGreater(payload["unique_satellites"], 0)
            self.assertIn("GPS", payload["rows_per_system"])
    def test_visibility_plot_generates_png_from_visibility_csv(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_visibility_plot_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            csv_path = temp_root / "visibility.csv"
            png_path = temp_root / "visibility.png"
            csv_path.write_text(
                "\n".join(
                    [
                        "epoch_index,week,tow,satellite,system,signal,azimuth_deg,elevation_deg,snr_dbhz,has_pseudorange,has_carrier_phase,has_doppler",
                        "1,2200,100.0,G01,GPS,GPS_L1CA,45.0,30.0,42.0,1,1,0",
                        "1,2200,100.0,G02,GPS,GPS_L1CA,120.0,55.0,47.0,1,1,0",
                        "2,2200,130.0,E11,Galileo,GAL_E1,250.0,20.0,38.0,1,1,0",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            result = self.run_gnss("visibility-plot", str(csv_path), str(png_path))

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("Saved:", result.stdout)
            self.assertTrue(png_path.exists())
            self.assertGreater(png_path.stat().st_size, 0)
    def test_stats_reports_solution_status_breakdown(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_stats_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            pos_path = temp_root / "stats.pos"
            write_libgnss_pos(
                pos_path,
                [
                    (2200, 345600.0, 35.0, 139.0, 10.0, 4, 10, 1.0),
                    (2200, 345630.0, 35.000001, 139.000001, 10.1, 3, 9, 1.3),
                    (2200, 345660.0, 35.000002, 139.000002, 10.2, 1, 8, 1.8),
                    (2200, 345690.0, 35.0000005, 139.0000005, 10.0, 4, 11, 0.9),
                ],
            )

            result = self.run_gnss("stats", str(pos_path))

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("総エポック数", result.stdout)
            self.assertIn("Fix解", result.stdout)
            self.assertIn("Float解", result.stdout)
            self.assertIn("SPP解", result.stdout)
    def test_compare_generates_png_for_synthetic_solutions(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_compare_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            lib_path = temp_root / "compare.pos"
            rtklib_path = temp_root / "compare_rtklib.pos"
            output_png = temp_root / "compare_comparison.png"
            write_libgnss_pos(
                lib_path,
                [
                    (2200, 345600.0, 35.0, 139.0, 10.0, 4, 10, 1.0),
                    (2200, 345630.0, 35.000001, 139.000002, 10.0, 4, 10, 1.0),
                    (2200, 345660.0, 35.000002, 139.000004, 10.1, 3, 9, 1.2),
                ],
            )
            write_rtklib_pos(
                rtklib_path,
                [
                    (2200, 345600.0, 35.0, 139.0, 10.0, 1, 10),
                    (2200, 345630.0, 35.0000012, 139.0000021, 10.1, 1, 10),
                    (2200, 345660.0, 35.0000024, 139.0000040, 10.2, 2, 9),
                ],
            )

            result = self.run_gnss("compare", str(lib_path), str(rtklib_path))

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("LibGNSS++:", result.stdout)
            self.assertIn("RTKLIB   :", result.stdout)
            self.assertTrue(output_png.exists())
    def test_plot_generates_single_and_comparison_pngs(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_plot_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            lib_path = temp_root / "plot.pos"
            rtklib_path = temp_root / "plot_rtklib.pos"
            write_libgnss_pos(
                lib_path,
                [
                    (2200, 345600.0, 35.0, 139.0, 10.0, 4, 10, 1.0),
                    (2200, 345630.0, 35.000001, 139.000001, 10.0, 3, 9, 1.2),
                    (2200, 345660.0, 35.000002, 139.000002, 10.1, 1, 8, 1.6),
                ],
            )
            write_rtklib_pos(
                rtklib_path,
                [
                    (2200, 345600.0, 35.0, 139.0, 10.0, 1, 10),
                    (2200, 345630.0, 35.0000015, 139.0000010, 10.0, 2, 9),
                    (2200, 345660.0, 35.0000025, 139.0000021, 10.2, 5, 8),
                ],
            )

            result = self.run_gnss("plot", str(lib_path), str(rtklib_path))

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertTrue((temp_root / "plot_plot.png").exists())
            self.assertTrue((temp_root / "plot_vs_rtklib.png").exists())
    def test_trackplot_generates_png(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_trackplot_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            lib_path = temp_root / "track.pos"
            rtklib_path = temp_root / "track_rtklib.pos"
            output_png = temp_root / "track.png"
            write_libgnss_pos(
                lib_path,
                [
                    (2200, 345600.0, 35.0, 139.0, 10.0, 4, 10, 1.0),
                    (2200, 345630.0, 35.000001, 139.000001, 10.1, 3, 9, 1.2),
                    (2200, 345660.0, 35.000002, 139.000002, 10.2, 1, 8, 1.6),
                ],
            )
            write_rtklib_pos(
                rtklib_path,
                [
                    (2200, 345600.0, 35.0, 139.0, 10.0, 1, 10),
                    (2200, 345630.0, 35.0000011, 139.0000011, 10.0, 2, 9),
                    (2200, 345660.0, 35.0000022, 139.0000021, 10.1, 4, 8),
                ],
            )

            result = self.run_gnss("trackplot", str(lib_path), str(rtklib_path), str(output_png))

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("Saved:", result.stdout)
            self.assertTrue(output_png.exists())
    def test_rtklib2pos_converts_solution_file(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_rtklib2pos_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "input_rtklib.pos"
            output_path = temp_root / "converted.pos"
            input_path.write_text(
                "\n".join(
                    [
                        "% synthetic rtklib solution",
                        "2026/03/27 00:00:00.000 35.000000000 139.000000000 10.0000 1 10",
                        "2026/03/27 00:00:30.000 35.000001000 139.000001000 10.1000 2 9",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            result = self.run_gnss("rtklib2pos", str(input_path), str(output_path))

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("Converted 2 position solutions", result.stdout)
            exported = output_path.read_text(encoding="utf-8")
            self.assertIn("Converted from RTKLIB POS format", exported)
            self.assertIn(" 4 10 0.0", exported)
            self.assertIn(" 3 9 0.0", exported)
    def test_pos2kml_exports_track_and_sample_points(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_pos2kml_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "track.pos"
            output_path = temp_root / "track.kml"
            write_libgnss_pos(
                input_path,
                [
                    (2200, 345600.0, 35.0, 139.0, 10.0, 4, 10, 1.0),
                    (2200, 345630.0, 35.000001, 139.000001, 10.1, 3, 9, 1.2),
                    (2200, 345660.0, 35.000002, 139.000002, 10.2, 1, 8, 1.6),
                ],
            )

            result = self.run_gnss(
                "pos2kml",
                str(input_path),
                str(output_path),
                "--sample-points",
                "1",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("Saved:", result.stdout)
            self.assertIn("Epochs: 3", result.stdout)
            exported = output_path.read_text(encoding="utf-8")
            self.assertIn("<LineString>", exported)
            self.assertIn("FIXED", exported)
    def test_driving_compare_generates_pngs_from_synthetic_inputs(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_driving_compare_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            lib_path = temp_root / "lib.pos"
            rtklib_path = temp_root / "rtklib.pos"
            reference_csv = temp_root / "reference.csv"
            output_png = temp_root / "comparison.png"
            rtklib_2d = temp_root / "rtklib_2d.png"
            lib_2d = temp_root / "lib_2d.png"
            reference_rows = [
                (2200, 345600.0, 35.0, 139.0, 10.0),
                (2200, 345630.0, 35.000001, 139.000001, 10.0),
                (2200, 345660.0, 35.000002, 139.000002, 10.0),
                (2200, 345690.0, 35.000003, 139.000003, 10.0),
            ]
            write_reference_csv(reference_csv, reference_rows)
            write_libgnss_pos(
                lib_path,
                [
                    (2200, 345600.0, 35.0, 139.0, 10.0, 4, 10, 1.0),
                    (2200, 345630.0, 35.0000011, 139.0000011, 10.0, 3, 9, 1.2),
                    (2200, 345660.0, 35.0000020, 139.0000021, 10.1, 1, 8, 1.5),
                    (2200, 345690.0, 35.0000030, 139.0000030, 10.0, 4, 10, 1.0),
                ],
            )
            write_rtklib_pos(
                rtklib_path,
                [
                    (2200, 345600.0, 35.0, 139.0, 10.0, 1, 10),
                    (2200, 345630.0, 35.0000012, 139.0000010, 10.0, 2, 9),
                    (2200, 345660.0, 35.0000022, 139.0000022, 10.0, 4, 8),
                    (2200, 345690.0, 35.0000031, 139.0000031, 10.0, 1, 10),
                ],
            )

            result = self.run_gnss(
                "driving-compare",
                "--lib-pos",
                str(lib_path),
                "--rtklib-pos",
                str(rtklib_path),
                "--reference-csv",
                str(reference_csv),
                "--output",
                str(output_png),
                "--rtklib-2d-output",
                str(rtklib_2d),
                "--lib-2d-output",
                str(lib_2d),
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertTrue(output_png.exists())
            self.assertTrue(rtklib_2d.exists())
            self.assertTrue(lib_2d.exists())
    def test_scorecard_and_social_card_generate_pngs_from_synthetic_inputs(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_cards_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            lib_path = temp_root / "lib.pos"
            rtklib_path = temp_root / "rtklib.pos"
            reference_csv = temp_root / "reference.csv"
            scorecard_png = temp_root / "scorecard.png"
            social_png = temp_root / "social.png"
            reference_rows = [
                (2200, 345600.0, 35.0, 139.0, 10.0),
                (2200, 345630.0, 35.000001, 139.000001, 10.0),
                (2200, 345660.0, 35.000002, 139.000002, 10.0),
            ]
            write_reference_csv(reference_csv, reference_rows)
            write_libgnss_pos(
                lib_path,
                [
                    (2200, 345600.0, 35.0, 139.0, 10.0, 4, 10, 1.0),
                    (2200, 345630.0, 35.0000011, 139.0000010, 10.0, 4, 10, 1.0),
                    (2200, 345660.0, 35.0000021, 139.0000020, 10.0, 4, 10, 1.0),
                ],
            )
            write_rtklib_pos(
                rtklib_path,
                [
                    (2200, 345600.0, 35.0, 139.0, 10.0, 1, 10),
                    (2200, 345630.0, 35.0000014, 139.0000014, 10.0, 1, 10),
                    (2200, 345660.0, 35.0000026, 139.0000026, 10.0, 1, 10),
                ],
            )

            scorecard_result = self.run_gnss(
                "scorecard",
                "--lib-pos",
                str(lib_path),
                "--rtklib-pos",
                str(rtklib_path),
                "--reference-csv",
                str(reference_csv),
                "--output",
                str(scorecard_png),
            )
            self.assertEqual(scorecard_result.returncode, 0, msg=scorecard_result.stderr)
            self.assertTrue(scorecard_png.exists())

            social_result = self.run_gnss(
                "social-card",
                "--lib-pos",
                str(lib_path),
                "--rtklib-pos",
                str(rtklib_path),
                "--reference-csv",
                str(reference_csv),
                "--output",
                str(social_png),
            )
            self.assertEqual(social_result.returncode, 0, msg=social_result.stderr)
            self.assertIn("Saved:", social_result.stdout)
            self.assertTrue(social_png.exists())
