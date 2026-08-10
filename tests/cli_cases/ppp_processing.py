"""CLI regression cases for the PPPProcessingCases domain."""

from ._support import *  # noqa: F401,F403

class PPPProcessingCases:
    def test_ppp_cli_processes_synthetic_precise_products(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_test_") as temp_dir:
            temp_root = Path(temp_dir)
            obs_path, sp3_path, clk_path, true_position = build_synthetic_ppp_inputs(temp_root)
            out_path = temp_root / "ppp_solution.pos"

            result = self.run_gnss(
                "ppp",
                "--static",
                "--obs",
                str(obs_path),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--no-estimate-troposphere",
                "--out",
                str(out_path),
                "--max-epochs",
                "8",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("PPP summary:", result.stdout)
            self.assertIn("PPP float solutions: 8", result.stdout)
            self.assertIn("PPP fixed solutions: 0", result.stdout)
            self.assertIn("fallback solutions: 0", result.stdout)
            self.assertIn("mode: static", result.stdout)

            records = self.read_pos_records(out_path)
            self.assertEqual(len(records), 8)
            self.assertTrue(all(record["status"] == 5 for record in records))
            self.assertTrue(all(record["satellites"] >= 6 for record in records))

            last_record = records[-1]
            error = math.sqrt(
                (last_record["x"] - true_position[0]) ** 2
                + (last_record["y"] - true_position[1]) ** 2
                + (last_record["z"] - true_position[2]) ** 2
            )
            self.assertLess(error, 1.0)
    def test_ppp_cli_accepts_ionex_and_dcb_products(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_bias_products_test_") as temp_dir:
            temp_root = Path(temp_dir)
            obs_path, sp3_path, clk_path, _true_position = build_synthetic_ppp_inputs(temp_root)
            ionex_path = temp_root / "synthetic.ionex"
            dcb_path = temp_root / "synthetic.bsx"
            out_path = temp_root / "ppp_with_bias_products.pos"
            summary_path = temp_root / "ppp_with_bias_products.json"
            ionex_path.write_text(build_synthetic_ionex_text(), encoding="ascii")
            dcb_path.write_text(build_synthetic_dcb_text(), encoding="ascii")

            result = self.run_gnss(
                "ppp",
                "--static",
                "--obs",
                str(obs_path),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--ionex",
                str(ionex_path),
                "--dcb",
                str(dcb_path),
                "--no-estimate-troposphere",
                "--out",
                str(out_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "4",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("ionex maps: 2", result.stdout)
            self.assertIn("dcb entries: 4", result.stdout)
            self.assertIn("ionex corrections: 0", result.stdout)
            self.assertNotIn("dcb corrections: 0", result.stdout)
            self.assertTrue(out_path.exists())
            self.assertTrue(summary_path.exists())
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertTrue(payload["ionex_loaded"])
            self.assertTrue(payload["dcb_loaded"])
            self.assertEqual(payload["ionex_corrections"], 0)
            self.assertGreater(payload["dcb_corrections"], 0)
            self.assertEqual(payload["valid_solutions"], 4)
            records = self.read_pos_records(out_path)
            self.assertEqual(len(records), 4)
    def test_ppp_cli_supports_receiver_antex_offsets(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_antex_test_") as temp_dir:
            temp_root = Path(temp_dir)
            obs_path, sp3_path, clk_path, true_position = build_synthetic_ppp_inputs(
                temp_root,
                include_antenna_header=True,
            )
            antex_path = temp_root / "receiver.atx"
            antex_path.write_text(build_synthetic_receiver_antex_text(), encoding="ascii")
            base_out_path = temp_root / "ppp_base.pos"
            antex_out_path = temp_root / "ppp_antex.pos"

            base_result = self.run_gnss(
                "ppp",
                "--static",
                "--obs",
                str(obs_path),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--no-estimate-troposphere",
                "--out",
                str(base_out_path),
                "--max-epochs",
                "8",
            )
            antex_result = self.run_gnss(
                "ppp",
                "--static",
                "--obs",
                str(obs_path),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--antex",
                str(antex_path),
                "--no-estimate-troposphere",
                "--out",
                str(antex_out_path),
                "--max-epochs",
                "8",
            )

            self.assertEqual(base_result.returncode, 0, msg=base_result.stderr)
            self.assertEqual(antex_result.returncode, 0, msg=antex_result.stderr)
            self.assertIn("PPP float solutions: 8", antex_result.stdout)

            base_records = self.read_pos_records(base_out_path)
            antex_records = self.read_pos_records(antex_out_path)
            self.assertEqual(len(base_records), 8)
            self.assertEqual(len(antex_records), 8)
            self.assertTrue(all(record["status"] == 5 for record in base_records))
            self.assertTrue(all(record["status"] == 5 for record in antex_records))

            base_last = base_records[-1]
            antex_last = antex_records[-1]
            base_error = math.sqrt(
                (base_last["x"] - true_position[0]) ** 2
                + (base_last["y"] - true_position[1]) ** 2
                + (base_last["z"] - true_position[2]) ** 2
            )
            antex_error = math.sqrt(
                (antex_last["x"] - true_position[0]) ** 2
                + (antex_last["y"] - true_position[1]) ** 2
                + (antex_last["z"] - true_position[2]) ** 2
            )
            solution_delta = math.sqrt(
                (antex_last["x"] - base_last["x"]) ** 2
                + (antex_last["y"] - base_last["y"]) ** 2
                + (antex_last["z"] - base_last["z"]) ** 2
            )

            self.assertLess(base_error, 1.5)
            self.assertLess(antex_error, 1.7)
            self.assertGreater(solution_delta, 1e-4)
            self.assertLess(solution_delta, 1.0)
    def test_ppp_cli_supports_receiver_antex_type_override(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_antex_override_test_") as temp_dir:
            temp_root = Path(temp_dir)
            obs_path, sp3_path, clk_path, true_position = build_synthetic_ppp_inputs(temp_root)
            antex_path = temp_root / "receiver.atx"
            antex_path.write_text(
                build_synthetic_receiver_antex_text().replace("\n", "\r\n"),
                encoding="ascii",
            )
            base_out_path = temp_root / "ppp_base.pos"
            antex_out_path = temp_root / "ppp_antex_override.pos"
            summary_path = temp_root / "ppp_antex_override_summary.json"

            base_result = self.run_gnss(
                "ppp",
                "--static",
                "--obs",
                str(obs_path),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--no-estimate-troposphere",
                "--out",
                str(base_out_path),
                "--max-epochs",
                "8",
            )
            antex_result = self.run_gnss(
                "ppp",
                "--static",
                "--obs",
                str(obs_path),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--antex",
                str(antex_path),
                "--receiver-antenna-type",
                "TEST-ANT",
                "--no-estimate-troposphere",
                "--out",
                str(antex_out_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "8",
            )

            self.assertEqual(base_result.returncode, 0, msg=base_result.stderr)
            self.assertEqual(antex_result.returncode, 0, msg=antex_result.stderr)
            self.assertIn("PPP float solutions: 8", antex_result.stdout)

            base_records = self.read_pos_records(base_out_path)
            antex_records = self.read_pos_records(antex_out_path)
            self.assertEqual(len(base_records), 8)
            self.assertEqual(len(antex_records), 8)
            base_last = base_records[-1]
            antex_last = antex_records[-1]
            antex_error = math.sqrt(
                (antex_last["x"] - true_position[0]) ** 2
                + (antex_last["y"] - true_position[1]) ** 2
                + (antex_last["z"] - true_position[2]) ** 2
            )
            solution_delta = math.sqrt(
                (antex_last["x"] - base_last["x"]) ** 2
                + (antex_last["y"] - base_last["y"]) ** 2
                + (antex_last["z"] - base_last["z"]) ** 2
            )
            payload = json.loads(summary_path.read_text(encoding="utf-8"))

            self.assertLess(antex_error, 1.7)
            self.assertGreater(solution_delta, 1e-4)
            self.assertLess(solution_delta, 1.0)
            self.assertEqual(payload["receiver_antenna_type"], "TEST-ANT")
            self.assertEqual(payload["receiver_antenna_type_source"], "cli")
            self.assertEqual(payload["antex"], str(antex_path))
    def test_ppp_cli_supports_ocean_loading_coefficients(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_blq_test_") as temp_dir:
            temp_root = Path(temp_dir)
            obs_path, sp3_path, clk_path, true_position = build_synthetic_ppp_inputs(temp_root)
            blq_path = temp_root / "site.blq"
            blq_path.write_text(build_synthetic_blq_text(), encoding="ascii")
            base_out_path = temp_root / "ppp_base.pos"
            blq_out_path = temp_root / "ppp_blq.pos"

            base_result = self.run_gnss(
                "ppp",
                "--static",
                "--obs",
                str(obs_path),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--no-estimate-troposphere",
                "--out",
                str(base_out_path),
                "--max-epochs",
                "8",
            )
            blq_result = self.run_gnss(
                "ppp",
                "--static",
                "--obs",
                str(obs_path),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--blq",
                str(blq_path),
                "--no-estimate-troposphere",
                "--out",
                str(blq_out_path),
                "--max-epochs",
                "8",
            )

            self.assertEqual(base_result.returncode, 0, msg=base_result.stderr)
            self.assertEqual(blq_result.returncode, 0, msg=blq_result.stderr)
            base_records = self.read_pos_records(base_out_path)
            blq_records = self.read_pos_records(blq_out_path)
            self.assertEqual(len(base_records), 8)
            self.assertEqual(len(blq_records), 8)

            base_last = base_records[-1]
            blq_last = blq_records[-1]
            base_error = math.sqrt(
                (base_last["x"] - true_position[0]) ** 2
                + (base_last["y"] - true_position[1]) ** 2
                + (base_last["z"] - true_position[2]) ** 2
            )
            blq_error = math.sqrt(
                (blq_last["x"] - true_position[0]) ** 2
                + (blq_last["y"] - true_position[1]) ** 2
                + (blq_last["z"] - true_position[2]) ** 2
            )
            solution_delta = math.sqrt(
                (blq_last["x"] - base_last["x"]) ** 2
                + (blq_last["y"] - base_last["y"]) ** 2
                + (blq_last["z"] - base_last["z"]) ** 2
            )

            self.assertLess(base_error, 1.5)
            self.assertLess(blq_error, 1.5)
            self.assertGreater(solution_delta, 1e-5)
            self.assertLess(solution_delta, 0.5)
    def test_ppp_cli_supports_kinematic_mode(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_kinematic_test_") as temp_dir:
            temp_root = Path(temp_dir)
            obs_path, sp3_path, clk_path, _ = build_synthetic_ppp_inputs(temp_root)
            out_path = temp_root / "ppp_kinematic_solution.pos"

            result = self.run_gnss(
                "ppp",
                "--kinematic",
                "--obs",
                str(obs_path),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--no-estimate-troposphere",
                "--out",
                str(out_path),
                "--max-epochs",
                "4",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("mode: kinematic", result.stdout)
            self.assertIn("PPP float solutions: 4", result.stdout)
            records = self.read_pos_records(out_path)
            self.assertEqual(len(records), 4)
            self.assertTrue(all(record["status"] == 5 for record in records))
    def test_ppp_cli_can_enable_ambiguity_resolution(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_ar_test_") as temp_dir:
            temp_root = Path(temp_dir)
            obs_path, sp3_path, clk_path, _ = build_synthetic_ppp_inputs(temp_root)
            out_path = temp_root / "ppp_ar_solution.pos"
            summary_path = temp_root / "ppp_ar_summary.json"

            result = self.run_gnss(
                "ppp",
                "--kinematic",
                "--enable-ar",
                "--ar-method",
                "per-freq",
                "--convergence-min-epochs",
                "4",
                "--convergence-policy",
                "local-enu",
                "--convergence-threshold-horizontal",
                "10.0",
                "--convergence-threshold-vertical",
                "20.0",
                "--ar-ratio-threshold",
                "2.0",
                "--obs",
                str(obs_path),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--no-estimate-troposphere",
                "--out",
                str(out_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "8",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("ambiguity resolution: on", result.stdout)
            summary = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(summary["ar_method"], "per-freq")
            self.assertTrue(summary["estimate_ionosphere"])
            self.assertFalse(summary["use_ionosphere_free"])
            self.assertEqual(summary["phase_measurement_min_lock_count"], 1)
            self.assertEqual(summary["convergence_policy"], "local-enu")
            self.assertEqual(
                summary["convergence_horizontal_position_deviation_threshold_m"],
                10.0,
            )
            self.assertEqual(
                summary["convergence_vertical_position_deviation_threshold_m"],
                20.0,
            )
            self.assertIn("convergence_max_horizontal_position_deviation_m", summary)
            self.assertIn("convergence_max_vertical_position_deviation_m", summary)
            self.assertIn("ar_stage_last", summary)
            self.assertIn("ar_per_frequency_attempts", summary)
            self.assertIn("PPP fixed solutions:", result.stdout)
            records = self.read_pos_records(out_path)
            self.assertEqual(len(records), 8)
            self.assertTrue(all(record["status"] in (5, 6) for record in records))
    def test_nav_products_cli_generates_sp3_and_clk_from_static_sample(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_nav_products_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            sp3_path = temp_root / "static_products.sp3"
            clk_path = temp_root / "static_products.clk"

            result = self.run_gnss(
                "nav-products",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--sp3-out",
                str(sp3_path),
                "--clk-out",
                str(clk_path),
                "--max-epochs",
                "4",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("epochs written: 4", result.stdout)
            self.assertTrue(sp3_path.exists())
            self.assertTrue(clk_path.exists())

            sp3_text = sp3_path.read_text(encoding="ascii")
            clk_text = clk_path.read_text(encoding="ascii")
            self.assertIn("*  2005 04 02 00 00 00.00000000", sp3_text)
            self.assertIn("PG03", sp3_text)
            self.assertIn("PG07", sp3_text)
            self.assertIn("AS G03 2005 04 02 00 00 00.00000000", clk_text)
            self.assertIn("AS G07 2005 04 02 00 00 00.00000000", clk_text)
    def test_ppp_cli_processes_real_static_sample_with_generated_products(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_real_static_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            sp3_path = temp_root / "static_products.sp3"
            clk_path = temp_root / "static_products.clk"
            output_path = temp_root / "ppp_real_static.pos"

            nav_products = self.run_gnss(
                "nav-products",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--sp3-out",
                str(sp3_path),
                "--clk-out",
                str(clk_path),
                "--max-epochs",
                "120",
            )
            self.assertEqual(nav_products.returncode, 0, msg=nav_products.stderr)

            result = self.run_gnss(
                "ppp",
                "--static",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--out",
                str(output_path),
                "--max-epochs",
                "120",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertTrue(output_path.exists())
            self.assertIn("valid solutions: 120", result.stdout)
            self.assertIn("PPP float solutions: 120", result.stdout)
            self.assertIn("PPP fixed solutions: 0", result.stdout)
            self.assertIn("fallback solutions: 0", result.stdout)
            self.assertIn("PPP solution rate (%): 100", result.stdout)
    def test_ppp_cli_runs_real_static_slice_with_generated_products_and_ar_enabled(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_real_static_ar_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            sp3_path = temp_root / "static_products.sp3"
            clk_path = temp_root / "static_products.clk"
            output_path = temp_root / "ppp_real_static_ar.pos"

            nav_products = self.run_gnss(
                "nav-products",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--sp3-out",
                str(sp3_path),
                "--clk-out",
                str(clk_path),
                "--max-epochs",
                "20",
            )
            self.assertEqual(nav_products.returncode, 0, msg=nav_products.stderr)

            result = self.run_gnss(
                "ppp",
                "--static",
                "--enable-ar",
                "--convergence-min-epochs",
                "4",
                "--ar-ratio-threshold",
                "1.5",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--out",
                str(output_path),
                "--max-epochs",
                "20",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertTrue(output_path.exists())
            self.assertIn("valid solutions: 20", result.stdout)
            self.assertIn("ambiguity resolution: on", result.stdout)
            self.assertIn("PPP fixed solutions:", result.stdout)
            self.assertIn("PPP solution rate (%):", result.stdout)
    def test_ppp_cli_accepts_ssr_corrections_csv(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_ssr_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            ssr_path = temp_root / "corrections.csv"
            output_path = temp_root / "ppp_ssr.pos"
            ssr_path.write_text(
                "\n".join(
                    [
                        "# week,tow,sat,dx,dy,dz,dclock_m",
                        "1316,518400.0,G03,0.0,0.0,0.0,0.0",
                        "1316,518430.0,G03,0.0,0.0,0.0,0.0",
                        "1316,518460.0,G03,0.0,0.0,0.0,0.0",
                    ]
                )
                + "\n",
                encoding="ascii",
            )

            result = self.run_gnss(
                "ppp",
                "--static",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--ssr",
                str(ssr_path),
                "--out",
                str(output_path),
                "--max-epochs",
                "3",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertTrue(output_path.exists())
            self.assertIn("SSR corrections: on", result.stdout)
            self.assertIn("valid solutions: 3", result.stdout)
    def test_ppp_cli_rejects_madoca_materialization_dump_without_l6(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_madoca_materialization_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            obs_path, sp3_path, _, _ = build_synthetic_ppp_inputs(temp_root)
            output_path = temp_root / "ppp.pos"
            dump_path = temp_root / "madoca_materialization.csv"

            result = self.run_gnss(
                "ppp",
                "--static",
                "--obs",
                str(obs_path),
                "--sp3",
                str(sp3_path),
                "--madoca-materialization-dump",
                str(dump_path),
                "--out",
                str(output_path),
                "--max-epochs",
                "1",
            )

            self.assertNotEqual(result.returncode, 0)
            self.assertIn("--madoca-materialization-dump requires --madoca-l6", result.stderr)
            self.assertFalse(dump_path.exists())
    def test_ppp_cli_reports_applied_atmospheric_corrections_from_sampled_ssr(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_ssr_atmos_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            obs_path, sp3_path, clk_path, ssr_path, true_position = build_synthetic_ppp_inputs_with_atmos(
                temp_root
            )
            baseline_path = temp_root / "ppp_ssr_atmos_baseline.pos"
            output_path = temp_root / "ppp_ssr_atmos.pos"

            baseline_result = self.run_gnss(
                "ppp",
                "--kinematic",
                "--obs",
                str(obs_path),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--no-estimate-troposphere",
                "--out",
                str(baseline_path),
                "--max-epochs",
                "4",
            )

            result = self.run_gnss(
                "ppp",
                "--kinematic",
                "--obs",
                str(obs_path),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--ssr",
                str(ssr_path),
                "--no-estimate-troposphere",
                "--out",
                str(output_path),
                "--max-epochs",
                "4",
            )

            self.assertEqual(baseline_result.returncode, 0, msg=baseline_result.stderr)
            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertTrue(baseline_path.exists())
            self.assertTrue(output_path.exists())
            self.assertIn("SSR corrections: on", result.stdout)
            self.assertIn("atmospheric trop corrections:", result.stdout)
            self.assertIn("atmospheric ionosphere corrections:", result.stdout)
            trop_line = next(
                line for line in result.stdout.splitlines() if "atmospheric trop corrections:" in line
            )
            iono_line = next(
                line
                for line in result.stdout.splitlines()
                if "atmospheric ionosphere corrections:" in line
            )
            self.assertGreater(int(trop_line.rsplit(":", 1)[1].strip()), 0)
            self.assertGreater(int(iono_line.rsplit(":", 1)[1].strip()), 0)

            baseline_records = self.read_pos_records(baseline_path)
            records = self.read_pos_records(output_path)
            self.assertEqual(len(baseline_records), 4)
            self.assertEqual(len(records), 4)
            baseline_last = baseline_records[-1]
            last_record = records[-1]
            baseline_error = math.sqrt(
                (baseline_last["x"] - true_position[0]) ** 2
                + (baseline_last["y"] - true_position[1]) ** 2
                + (baseline_last["z"] - true_position[2]) ** 2
            )
            error = math.sqrt(
                (last_record["x"] - true_position[0]) ** 2
                + (last_record["y"] - true_position[1]) ** 2
                + (last_record["z"] - true_position[2]) ** 2
            )
            self.assertLess(error, baseline_error)
    def test_ppp_cli_writes_clas_osr_application_mode_to_summary_json(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_clas_osr_application_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            obs_path, sp3_path, clk_path, ssr_path, _ = build_synthetic_ppp_inputs_with_atmos(
                temp_root
            )
            output_path = temp_root / "ppp_clas_osr_application.pos"
            summary_path = temp_root / "ppp_clas_osr_application_summary.json"

            result = self.run_gnss(
                "ppp",
                "--kinematic",
                "--obs",
                str(obs_path),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--ssr",
                str(ssr_path),
                "--clas-osr-application",
                "orbit-clock-only",
                "--clas-phase-continuity",
                "no-phase-bias",
                "--no-estimate-troposphere",
                "--summary-json",
                str(summary_path),
                "--out",
                str(output_path),
                "--max-epochs",
                "4",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertTrue(summary_path.exists())
            summary = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(summary["clas_epoch_policy"], "strict-osr")
            self.assertEqual(summary["clas_osr_application"], "orbit-clock-only")
            self.assertEqual(summary["clas_phase_continuity"], "no-phase-bias")
            self.assertEqual(summary["clas_ssr_timing"], "lag-tolerant")
    def test_ppp_cli_writes_intermediate_phase_continuity_mode_to_summary_json(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_clas_phase_mode_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            obs_path, sp3_path, clk_path, ssr_path, _ = build_synthetic_ppp_inputs_with_atmos(
                temp_root
            )
            output_path = temp_root / "ppp_clas_phase_mode.pos"
            summary_path = temp_root / "ppp_clas_phase_mode_summary.json"

            result = self.run_gnss(
                "ppp",
                "--kinematic",
                "--obs",
                str(obs_path),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--ssr",
                str(ssr_path),
                "--clas-phase-continuity",
                "sis-continuity-only",
                "--no-estimate-troposphere",
                "--summary-json",
                str(summary_path),
                "--out",
                str(output_path),
                "--max-epochs",
                "4",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            summary = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(summary["clas_phase_continuity"], "sis-continuity-only")
    def test_ppp_cli_writes_phase_bias_value_mode_to_summary_json(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_clas_phase_value_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            obs_path, sp3_path, clk_path, ssr_path, _ = build_synthetic_ppp_inputs_with_atmos(
                temp_root
            )
            output_path = temp_root / "ppp_clas_phase_value.pos"
            summary_path = temp_root / "ppp_clas_phase_value_summary.json"

            result = self.run_gnss(
                "ppp",
                "--kinematic",
                "--obs",
                str(obs_path),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--ssr",
                str(ssr_path),
                "--clas-phase-bias-values",
                "phase-bias-only",
                "--no-estimate-troposphere",
                "--summary-json",
                str(summary_path),
                "--out",
                str(output_path),
                "--max-epochs",
                "4",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            summary = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(summary["clas_phase_bias_values"], "phase-bias-only")
    def test_ppp_cli_writes_phase_bias_reference_time_mode_to_summary_json(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_clas_phase_ref_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            obs_path, sp3_path, clk_path, ssr_path, _ = build_synthetic_ppp_inputs_with_atmos(
                temp_root
            )
            output_path = temp_root / "ppp_clas_phase_ref.pos"
            summary_path = temp_root / "ppp_clas_phase_ref_summary.json"

            result = self.run_gnss(
                "ppp",
                "--kinematic",
                "--obs",
                str(obs_path),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--ssr",
                str(ssr_path),
                "--clas-phase-bias-reference-time",
                "clock-reference",
                "--no-estimate-troposphere",
                "--summary-json",
                str(summary_path),
                "--out",
                str(output_path),
                "--max-epochs",
                "4",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            summary = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(summary["clas_phase_bias_reference_time"], "clock-reference")
    def test_ppp_cli_writes_clas_expanded_values_mode_to_summary_json(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_clas_value_mode_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            obs_path, sp3_path, clk_path, ssr_path, _ = build_synthetic_ppp_inputs_with_atmos(
                temp_root
            )
            output_path = temp_root / "ppp_clas_value_mode.pos"
            summary_path = temp_root / "ppp_clas_value_mode_summary.json"

            result = self.run_gnss(
                "ppp",
                "--kinematic",
                "--obs",
                str(obs_path),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--ssr",
                str(ssr_path),
                "--clas-expanded-values",
                "residual-only",
                "--no-estimate-troposphere",
                "--summary-json",
                str(summary_path),
                "--out",
                str(output_path),
                "--max-epochs",
                "4",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            summary = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(summary["clas_expanded_values"], "residual-only")
            self.assertEqual(summary["clas_epoch_policy"], "strict-osr")
    def test_ppp_cli_writes_clas_residual_sampling_mode_to_summary_json(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_clas_residual_sampling_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            obs_path, sp3_path, clk_path, ssr_path, _ = build_synthetic_ppp_inputs_with_atmos(
                temp_root
            )
            output_path = temp_root / "ppp_clas_residual_sampling.pos"
            summary_path = temp_root / "ppp_clas_residual_sampling_summary.json"

            result = self.run_gnss(
                "ppp",
                "--kinematic",
                "--obs",
                str(obs_path),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--ssr",
                str(ssr_path),
                "--clas-residual-sampling",
                "mean-only",
                "--no-estimate-troposphere",
                "--summary-json",
                str(summary_path),
                "--out",
                str(output_path),
                "--max-epochs",
                "4",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            summary = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(summary["clas_residual_sampling"], "mean-only")
            self.assertEqual(summary["clas_epoch_policy"], "strict-osr")
    def test_ppp_cli_writes_clas_subtype12_values_mode_to_summary_json(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_clas_subtype12_values_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            obs_path, sp3_path, clk_path, ssr_path, _ = build_synthetic_ppp_inputs_with_atmos(
                temp_root
            )
            output_path = temp_root / "ppp_clas_subtype12_values.pos"
            summary_path = temp_root / "ppp_clas_subtype12_values_summary.json"

            result = self.run_gnss(
                "ppp",
                "--kinematic",
                "--obs",
                str(obs_path),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--ssr",
                str(ssr_path),
                "--clas-subtype12-values",
                "planar",
                "--no-estimate-troposphere",
                "--summary-json",
                str(summary_path),
                "--out",
                str(output_path),
                "--max-epochs",
                "4",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            summary = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(summary["clas_subtype12_values"], "planar")
            self.assertEqual(summary["clas_epoch_policy"], "strict-osr")
    def test_ppp_cli_applies_grid_polynomial_ionosphere_from_sampled_ssr(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_ssr_grid_poly_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            obs_path, sp3_path, clk_path, ssr_path, true_position = (
                build_synthetic_ppp_inputs_with_grid_polynomial_atmos(temp_root)
            )
            baseline_path = temp_root / "ppp_ssr_grid_poly_baseline.pos"
            output_path = temp_root / "ppp_ssr_grid_poly.pos"

            baseline_result = self.run_gnss(
                "ppp",
                "--kinematic",
                "--obs",
                str(obs_path),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--no-estimate-troposphere",
                "--out",
                str(baseline_path),
                "--max-epochs",
                "4",
            )

            result = self.run_gnss(
                "ppp",
                "--kinematic",
                "--obs",
                str(obs_path),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--ssr",
                str(ssr_path),
                "--no-estimate-troposphere",
                "--out",
                str(output_path),
                "--max-epochs",
                "4",
            )

            self.assertEqual(baseline_result.returncode, 0, msg=baseline_result.stderr)
            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertTrue(baseline_path.exists())
            self.assertTrue(output_path.exists())
            self.assertIn("SSR corrections: on", result.stdout)
            self.assertIn("atmospheric ionosphere corrections:", result.stdout)

            iono_count_line = next(
                line
                for line in result.stdout.splitlines()
                if "atmospheric ionosphere corrections:" in line
            )
            self.assertGreater(int(iono_count_line.rsplit(":", 1)[1].strip()), 0)

            baseline_records = self.read_pos_records(baseline_path)
            records = self.read_pos_records(output_path)
            self.assertEqual(len(baseline_records), 4)
            self.assertEqual(len(records), 4)
            baseline_last = baseline_records[-1]
            last_record = records[-1]
            baseline_error = math.sqrt(
                (baseline_last["x"] - true_position[0]) ** 2
                + (baseline_last["y"] - true_position[1]) ** 2
                + (baseline_last["z"] - true_position[2]) ** 2
            )
            error = math.sqrt(
                (last_record["x"] - true_position[0]) ** 2
                + (last_record["y"] - true_position[1]) ** 2
                + (last_record["z"] - true_position[2]) ** 2
            )
            self.assertLess(error, baseline_error)
    def test_ppp_cli_accepts_rtcm_ssr_corrections(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_ssr_rtcm_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            ssr_path = temp_root / "corrections.rtcm3"
            output_path = temp_root / "ppp_ssr_rtcm.pos"
            ssr_path.write_bytes(
                build_rtcm1060(3, 518400) +
                build_rtcm1059(3, 518400) +
                build_rtcm1062(3, 518400)
            )

            result = self.run_gnss(
                "ppp",
                "--static",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--ssr-rtcm",
                str(ssr_path),
                "--out",
                str(output_path),
                "--max-epochs",
                "3",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertTrue(output_path.exists())
            self.assertIn("SSR corrections: on", result.stdout)
            self.assertIn("valid solutions: 3", result.stdout)
    def test_ppp_cli_detects_cycle_slip_from_geometry_free(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_cycle_slip_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            obs_path, sp3_path, clk_path, _ = build_synthetic_ppp_inputs_with_cycle_slip(temp_root)
            output_path = temp_root / "ppp_cycle_slip.pos"

            result = self.run_gnss(
                "ppp",
                "--kinematic",
                "--obs",
                str(obs_path),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--no-estimate-troposphere",
                "--out",
                str(output_path),
                "--max-epochs",
                "4",
                extra_env={"GNSS_PPP_DEBUG": "1"},
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertTrue(output_path.exists())
            self.assertIn("valid solutions: 4", result.stdout)
            self.assertIn("cycle slip reset G01", result.stderr)
            self.assertIn("reason=geometry-free+melbourne-wubbena", result.stderr)
    def test_ppp_cli_accepts_ntrip_rtcm_ssr_corrections(self) -> None:
        payload = build_rtcm1060(3, 518400) + build_rtcm1059(3, 518400) + build_rtcm1062(3, 518400)

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.bind(("127.0.0.1", 0))
            server.listen(1)
            server.settimeout(2.0)
            port = server.getsockname()[1]

            def caster() -> None:
                conn, _ = server.accept()
                with conn:
                    conn.recv(1024)
                    conn.sendall(b"ICY 200 OK\r\nNtrip-Version: Ntrip/2.0\r\n\r\n")
                    conn.sendall(payload)

            thread = threading.Thread(target=caster)
            thread.start()
            try:
                with tempfile.TemporaryDirectory(prefix="gnss_ppp_ssr_ntrip_cli_") as temp_dir:
                    temp_root = Path(temp_dir)
                    output_path = temp_root / "ppp_ssr_ntrip.pos"

                    result = self.run_gnss(
                        "ppp",
                        "--static",
                        "--obs",
                        str(ROOT_DIR / "data/rover_static.obs"),
                        "--nav",
                        str(ROOT_DIR / "data/navigation_static.nav"),
                        "--ssr-rtcm",
                        f"ntrip://127.0.0.1:{port}/MOUNT1",
                        "--out",
                        str(output_path),
                        "--max-epochs",
                        "3",
                    )
                    self.assertEqual(result.returncode, 0, msg=result.stderr)
                    self.assertTrue(output_path.exists())
                    self.assertIn("SSR corrections: on", result.stdout)
                    self.assertIn("valid solutions: 3", result.stdout)
            finally:
                thread.join()
