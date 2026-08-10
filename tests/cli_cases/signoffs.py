"""CLI regression cases for the SignoffCases domain."""

from ._support import *  # noqa: F401,F403

class SignoffCases:
    def test_solve_odaiba_slice_uses_glonass_and_beidou_in_rtk(self) -> None:
        rover = ROOT_DIR / "data/driving/Tokyo_Data/Odaiba/rover_trimble.obs"
        base = ROOT_DIR / "data/driving/Tokyo_Data/Odaiba/base_trimble.obs"
        nav = ROOT_DIR / "data/driving/Tokyo_Data/Odaiba/base.nav"
        reference_csv = ROOT_DIR / "data/driving/Tokyo_Data/Odaiba/reference.csv"

        self.assertTrue(rover.exists())
        self.assertTrue(base.exists())
        self.assertTrue(nav.exists())
        self.assertTrue(reference_csv.exists())

        with tempfile.TemporaryDirectory(prefix="gnss_solve_odaiba_test_") as temp_dir:
            temp_root = Path(temp_dir)
            cases = {
                "default": [],
                "no_glonass": ["--no-glonass"],
                "no_beidou": ["--no-beidou"],
                "gps_gal_qzss_only": ["--no-glonass", "--no-beidou"],
            }
            case_records: dict[str, list[dict[str, float | int]]] = {}

            for name, extra_args in cases.items():
                output_path = temp_root / f"{name}.pos"
                result = self.run_gnss(
                    "solve",
                    "--rover",
                    str(rover),
                    "--base",
                    str(base),
                    "--nav",
                    str(nav),
                    "--out",
                    str(output_path),
                    "--no-kml",
                    "--skip-epochs",
                    "0",
                    "--max-epochs",
                    "5",
                    *extra_args,
                )

                self.assertEqual(result.returncode, 0, msg=result.stderr)
                self.assertIn("valid solutions: 5", result.stdout)
                self.assertTrue(output_path.exists())

                records = self.read_pos_records(output_path)
                self.assertEqual(len(records), 5)
                self.assertTrue(all(int(record["status"]) > 0 for record in records))
                case_records[name] = records

            def average_satellites(name: str) -> float:
                records = case_records[name]
                return sum(int(record["satellites"]) for record in records) / len(records)

            default_avg = average_satellites("default")
            no_glonass_avg = average_satellites("no_glonass")
            no_beidou_avg = average_satellites("no_beidou")
            gps_gal_qzss_avg = average_satellites("gps_gal_qzss_only")

            self.assertGreater(default_avg, no_glonass_avg)
            self.assertGreater(default_avg, no_beidou_avg)
            self.assertGreater(no_glonass_avg, gps_gal_qzss_avg)
            self.assertGreater(no_beidou_avg, gps_gal_qzss_avg)

            reference = driving_comparison.read_reference_csv(reference_csv)
            default_epochs = driving_comparison.read_libgnss_pos(temp_root / "default.pos")
            default_matched = driving_comparison.match_to_reference(default_epochs, reference, 0.11)
            default_summary = driving_comparison.summarize(
                default_matched,
                fixed_status=4,
                label="odaiba default slice",
            )

            self.assertEqual(default_summary["epochs"], 5)
            self.assertLess(default_summary["p95_h_m"], 1.0)
            self.assertLess(default_summary["max_h_m"], 1.0)
    def test_solve_short_baseline_cli_reaches_fixed_solution(self) -> None:
        rover = ROOT_DIR / "data/short_baseline/TSK200JPN_R_20240010000_01D_30S_MO.rnx"
        base = ROOT_DIR / "data/short_baseline/TSKB00JPN_R_20240010000_01D_30S_MO.rnx"
        nav = ROOT_DIR / "data/short_baseline/BRDC00IGS_R_20240010000_01D_MN.rnx"

        self.assertTrue(rover.exists())
        self.assertTrue(base.exists())
        self.assertTrue(nav.exists())

        with tempfile.TemporaryDirectory(prefix="gnss_solve_short_baseline_test_") as temp_dir:
            output_path = Path(temp_dir) / "short_baseline.pos"
            result = self.run_gnss(
                "solve",
                "--rover",
                str(rover),
                "--base",
                str(base),
                "--nav",
                str(nav),
                "--out",
                str(output_path),
                "--no-kml",
                "--max-epochs",
                "10",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("valid solutions: 10", result.stdout)
            self.assertIn("fixed solutions:", result.stdout)
            self.assertTrue(output_path.exists())

            records = self.read_pos_records(output_path)
            self.assertEqual(len(records), 10)
            self.assertGreaterEqual(
                sum(int(record["status"]) == 4 for record in records),
                1,
            )
            self.assertGreaterEqual(
                sum(int(record["satellites"]) for record in records) / len(records),
                10.0,
            )
    def test_solve_cli_supports_estimated_iono_mode(self) -> None:
        rover = ROOT_DIR / "data/rover_kinematic.obs"
        base = ROOT_DIR / "data/base_kinematic.obs"
        nav = ROOT_DIR / "data/navigation_kinematic.nav"

        self.assertTrue(rover.exists())
        self.assertTrue(base.exists())
        self.assertTrue(nav.exists())

        with tempfile.TemporaryDirectory(prefix="gnss_solve_iono_est_") as temp_dir:
            output_path = Path(temp_dir) / "iono_est.pos"
            result = self.run_gnss(
                "solve",
                "--rover",
                str(rover),
                "--base",
                str(base),
                "--nav",
                str(nav),
                "--out",
                str(output_path),
                "--no-kml",
                "--mode",
                "kinematic",
                "--iono",
                "est",
                "--max-epochs",
                "5",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("iono: est", result.stdout)
            self.assertIn("valid solutions: 5", result.stdout)
            self.assertTrue(output_path.exists())

            records = self.read_pos_records(output_path)
            self.assertEqual(len(records), 5)
            self.assertTrue(all(int(record["status"]) > 0 for record in records))
    def test_solve_cli_supports_moving_base_mode(self) -> None:
        rover = ROOT_DIR / "data/rover_kinematic.obs"
        base = ROOT_DIR / "data/base_kinematic.obs"
        nav = ROOT_DIR / "data/navigation_kinematic.nav"

        self.assertTrue(rover.exists())
        self.assertTrue(base.exists())
        self.assertTrue(nav.exists())

        with tempfile.TemporaryDirectory(prefix="gnss_solve_moving_base_") as temp_dir:
            output_path = Path(temp_dir) / "moving_base.pos"
            result = self.run_gnss(
                "solve",
                "--rover",
                str(rover),
                "--base",
                str(base),
                "--nav",
                str(nav),
                "--out",
                str(output_path),
                "--no-kml",
                "--mode",
                "moving-base",
                "--max-epochs",
                "5",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("mode: moving-base", result.stdout)
            self.assertIn("valid solutions: 5", result.stdout)
            self.assertTrue(output_path.exists())
    def test_short_baseline_signoff_cli_writes_summary_and_passes_thresholds(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_short_baseline_signoff_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            output_path = temp_root / "short_baseline.pos"
            summary_path = temp_root / "short_baseline_summary.json"

            result = self.run_gnss(
                "short-baseline-signoff",
                "--out",
                str(output_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "120",
                "--require-fix-rate-min",
                "95",
                "--require-mean-error-max",
                "0.15",
                "--require-max-error-max",
                "0.60",
                "--require-mean-sats-min",
                "14",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("Finished short-baseline sign-off.", result.stdout)
            self.assertTrue(output_path.exists())
            self.assertTrue(summary_path.exists())

            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["dataset"], "Tsukuba short_baseline")
            self.assertEqual(payload["epochs"], 120)
            self.assertGreaterEqual(payload["fix_rate_pct"], 95.0)
            self.assertLessEqual(payload["mean_position_error_m"], 0.15)
            self.assertLessEqual(payload["max_position_error_m"], 0.60)
            self.assertGreaterEqual(payload["mean_satellites"], 14.0)
    def test_rtk_kinematic_signoff_cli_writes_summary_and_passes_thresholds(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_rtk_kinematic_signoff_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            output_path = temp_root / "rtk_kinematic.pos"
            summary_path = temp_root / "rtk_kinematic_summary.json"

            result = self.run_gnss(
                "rtk-kinematic-signoff",
                "--out",
                str(output_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "120",
                "--require-valid-epochs-min",
                "120",
                "--require-fix-rate-min",
                "95",
                "--require-mean-error-max",
                "3.0",
                "--require-max-error-max",
                "3.0",
                "--require-mean-sats-min",
                "25",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("Finished RTK kinematic sign-off.", result.stdout)
            self.assertTrue(output_path.exists())
            self.assertTrue(summary_path.exists())

            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["dataset"], "sample mixed-GNSS kinematic RTK")
            self.assertEqual(payload["epochs"], 120)
            self.assertGreaterEqual(payload["fix_rate_pct"], 95.0)
            self.assertLessEqual(payload["mean_position_error_m"], 3.0)
            self.assertLessEqual(payload["max_position_error_m"], 3.0)
            self.assertGreaterEqual(payload["mean_satellites"], 25.0)
    def test_ppp_static_signoff_cli_writes_summary_and_passes_thresholds(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_static_signoff_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            output_path = temp_root / "ppp_static.pos"
            summary_path = temp_root / "ppp_static_summary.json"

            result = self.run_gnss(
                "ppp-static-signoff",
                "--out",
                str(output_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "120",
                "--require-valid-epochs-min",
                "120",
                "--require-mean-error-max",
                "1.5",
                "--require-max-error-max",
                "1.5",
                "--require-mean-sats-min",
                "6.0",
                "--require-ppp-solution-rate-min",
                "100.0",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("Finished PPP static sign-off.", result.stdout)
            self.assertTrue(output_path.exists())
            self.assertTrue(summary_path.exists())

            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["dataset"], "sample static PPP")
            self.assertEqual(payload["epochs"], 120)
            self.assertEqual(payload["fallback_epochs"], 0)
            self.assertLessEqual(payload["mean_position_error_m"], 1.5)
            self.assertLessEqual(payload["max_position_error_m"], 1.5)
            self.assertGreaterEqual(payload["mean_satellites"], 6.0)
            self.assertEqual(payload["ppp_solution_rate_pct"], 100.0)
    def test_ppp_static_signoff_cli_supports_real_data_ar_signoff(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_static_signoff_ar_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            output_path = temp_root / "ppp_static_ar.pos"
            summary_path = temp_root / "ppp_static_ar_summary.json"

            result = self.run_gnss(
                "ppp-static-signoff",
                "--out",
                str(output_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "120",
                "--enable-ar",
                "--generate-products",
                "--ar-ratio-threshold",
                "1.5",
                "--require-valid-epochs-min",
                "120",
                "--require-mean-error-max",
                "5.0",
                "--require-max-error-max",
                "6.0",
                "--require-ppp-solution-rate-min",
                "100.0",
                "--require-ppp-fixed-epochs-min",
                "1",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("Finished PPP static sign-off.", result.stdout)
            self.assertTrue(output_path.exists())
            self.assertTrue(summary_path.exists())

            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["dataset"], "sample static PPP")
            self.assertEqual(payload["epochs"], 120)
            self.assertEqual(payload["fallback_epochs"], 0)
            self.assertLessEqual(payload["mean_position_error_m"], 5.0)
            self.assertLessEqual(payload["max_position_error_m"], 6.0)
            self.assertGreaterEqual(payload["ppp_fixed_epochs"], 1)
            self.assertEqual(payload["ppp_solution_rate_pct"], 100.0)
            self.assertTrue(payload["ambiguity_resolution_enabled"])
            self.assertEqual(payload["ar_ratio_threshold"], 1.5)
    def test_ppp_static_signoff_cli_can_fetch_precise_products(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_static_signoff_fetch_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            generated_sp3 = temp_root / "generated.sp3"
            generated_clk = temp_root / "generated.clk"
            products_dir = temp_root / "products"
            products_dir.mkdir()
            output_path = temp_root / "ppp_static_fetch.pos"
            summary_path = temp_root / "ppp_static_fetch_summary.json"

            nav_products = self.run_gnss(
                "nav-products",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--sp3-out",
                str(generated_sp3),
                "--clk-out",
                str(generated_clk),
                "--max-epochs",
                "30",
            )
            self.assertEqual(nav_products.returncode, 0, msg=nav_products.stderr)

            with gzip.open(products_dir / "2024002.sp3.gz", "wb") as stream:
                stream.write(generated_sp3.read_bytes())
            with gzip.open(products_dir / "2024002.clk.gz", "wb") as stream:
                stream.write(generated_clk.read_bytes())
            with gzip.open(products_dir / "2024002.ionex.gz", "wb") as stream:
                stream.write(build_synthetic_ionex_text().encode("ascii"))
            with gzip.open(products_dir / "2024002.bsx.gz", "wb") as stream:
                stream.write(build_synthetic_dcb_text().encode("ascii"))

            result = self.run_gnss(
                "ppp-static-signoff",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--out",
                str(output_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "30",
                "--fetch-products",
                "--product-date",
                "2024-01-02",
                "--product",
                f"sp3={products_dir / '{yyyy}{doy}.sp3.gz'}",
                "--product",
                f"clk={products_dir / '{yyyy}{doy}.clk.gz'}",
                "--product",
                f"ionex={products_dir / '{yyyy}{doy}.ionex.gz'}",
                "--product",
                f"dcb={products_dir / '{yyyy}{doy}.bsx.gz'}",
                "--require-valid-epochs-min",
                "30",
                "--require-ppp-solution-rate-min",
                "100",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertTrue(payload["fetch_products"])
            self.assertEqual(payload["fetched_product_date"], "2024-01-02")
            self.assertEqual(set(payload["fetched_products"]), {"sp3", "clk", "ionex", "dcb"})
            self.assertTrue(str(payload["ionex"]).endswith("2024002.ionex"))
            self.assertTrue(str(payload["dcb"]).endswith("2024002.bsx"))
            self.assertGreaterEqual(payload["ionex_corrections"], 0)
            self.assertGreaterEqual(payload["dcb_corrections"], 0)
            self.assertIn("ppp_run_summary", payload)
    def test_ppp_kinematic_signoff_cli_writes_summary_and_passes_thresholds(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_kinematic_signoff_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            output_path = temp_root / "ppp_kinematic.pos"
            reference_path = temp_root / "ppp_kinematic_reference.pos"
            summary_path = temp_root / "ppp_kinematic_summary.json"

            result = self.run_gnss(
                "ppp-kinematic-signoff",
                "--out",
                str(output_path),
                "--reference-pos",
                str(reference_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "60",
                "--require-common-epoch-pairs-min",
                "60",
                "--require-reference-fix-rate-min",
                "90",
                "--require-converged",
                "--require-convergence-time-max",
                "300",
                "--require-mean-sats-min",
                "18",
                "--require-mean-error-max",
                "7",
                "--require-p95-error-max",
                "7",
                "--require-max-error-max",
                "7",
                "--require-ppp-solution-rate-min",
                "100",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("Finished PPP kinematic sign-off.", result.stdout)
            self.assertTrue(output_path.exists())
            self.assertTrue(reference_path.exists())
            self.assertTrue(summary_path.exists())

            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["dataset"], "sample kinematic PPP")
            self.assertEqual(payload["epochs"], 60)
            self.assertEqual(payload["common_epoch_pairs"], 60)
            self.assertTrue(payload["ppp_converged"])
            self.assertLessEqual(payload["ppp_convergence_time_s"], 300.0)
            self.assertEqual(payload["fallback_epochs"], 0)
            self.assertGreaterEqual(payload["reference_fix_rate_pct"], 90.0)
            self.assertLessEqual(payload["mean_position_error_m"], 7.0)
            self.assertLessEqual(payload["p95_position_error_m"], 7.0)
            self.assertLessEqual(payload["max_position_error_m"], 7.0)
    def test_ppp_kinematic_signoff_cli_can_fetch_precise_products(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_kinematic_signoff_fetch_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            generated_sp3 = temp_root / "generated.sp3"
            generated_clk = temp_root / "generated.clk"
            products_dir = temp_root / "products"
            products_dir.mkdir()
            output_path = temp_root / "ppp_kinematic_fetch.pos"
            reference_path = temp_root / "ppp_kinematic_fetch_reference.pos"
            summary_path = temp_root / "ppp_kinematic_fetch_summary.json"

            nav_products = self.run_gnss(
                "nav-products",
                "--obs",
                str(ROOT_DIR / "data/rover_kinematic.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_kinematic.nav"),
                "--sp3-out",
                str(generated_sp3),
                "--clk-out",
                str(generated_clk),
                "--max-epochs",
                "20",
            )
            self.assertEqual(nav_products.returncode, 0, msg=nav_products.stderr)

            with gzip.open(products_dir / "2024002.sp3.gz", "wb") as stream:
                stream.write(generated_sp3.read_bytes())
            with gzip.open(products_dir / "2024002.clk.gz", "wb") as stream:
                stream.write(generated_clk.read_bytes())
            with gzip.open(products_dir / "2024002.ionex.gz", "wb") as stream:
                stream.write(build_synthetic_ionex_text().encode("ascii"))
            with gzip.open(products_dir / "2024002.bsx.gz", "wb") as stream:
                stream.write(build_synthetic_dcb_text().encode("ascii"))

            result = self.run_gnss(
                "ppp-kinematic-signoff",
                "--obs",
                str(ROOT_DIR / "data/rover_kinematic.obs"),
                "--base",
                str(ROOT_DIR / "data/base_kinematic.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_kinematic.nav"),
                "--out",
                str(output_path),
                "--reference-pos",
                str(reference_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "20",
                "--fetch-products",
                "--product-date",
                "2024-01-02",
                "--product",
                f"sp3={products_dir / '{yyyy}{doy}.sp3.gz'}",
                "--product",
                f"clk={products_dir / '{yyyy}{doy}.clk.gz'}",
                "--product",
                f"ionex={products_dir / '{yyyy}{doy}.ionex.gz'}",
                "--product",
                f"dcb={products_dir / '{yyyy}{doy}.bsx.gz'}",
                "--require-common-epoch-pairs-min",
                "20",
                "--require-ppp-solution-rate-min",
                "100",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertTrue(payload["fetch_products"])
            self.assertEqual(payload["fetched_product_date"], "2024-01-02")
            self.assertEqual(set(payload["fetched_products"]), {"sp3", "clk", "ionex", "dcb"})
            self.assertTrue(str(payload["ionex"]).endswith("2024002.ionex"))
            self.assertTrue(str(payload["dcb"]).endswith("2024002.bsx"))
            self.assertGreaterEqual(payload["ionex_corrections"], 0)
            self.assertGreaterEqual(payload["dcb_corrections"], 0)
            self.assertIn("ppp_run_summary", payload)
            self.assertGreaterEqual(payload["mean_satellites"], 18.0)
            self.assertEqual(payload["ppp_solution_rate_pct"], 100.0)
    def test_ppp_products_signoff_cli_runs_static_profile_with_local_templates(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_products_signoff_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            generated_sp3 = temp_root / "generated.sp3"
            generated_clk = temp_root / "generated.clk"
            products_dir = temp_root / "products"
            products_dir.mkdir()
            output_path = temp_root / "ppp_products.pos"
            summary_path = temp_root / "ppp_products_summary.json"
            malib_pos = temp_root / "malib_static.pos"

            nav_products = self.run_gnss(
                "nav-products",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--sp3-out",
                str(generated_sp3),
                "--clk-out",
                str(generated_clk),
                "--max-epochs",
                "20",
            )
            self.assertEqual(nav_products.returncode, 0, msg=nav_products.stderr)

            with gzip.open(products_dir / "2024002.sp3.gz", "wb") as stream:
                stream.write(generated_sp3.read_bytes())
            with gzip.open(products_dir / "2024002.clk.gz", "wb") as stream:
                stream.write(generated_clk.read_bytes())
            with gzip.open(products_dir / "2024002.ionex.gz", "wb") as stream:
                stream.write(build_synthetic_ionex_text().encode("ascii"))
            with gzip.open(products_dir / "2024002.bsx.gz", "wb") as stream:
                stream.write(build_synthetic_dcb_text().encode("ascii"))

            malib_pos.write_text(
                "\n".join(
                    [
                        "% synthetic MALIB static",
                        "2024/01/02 00:00:00.000 1.0 2.0 3.0 5 7",
                        "2024/01/02 00:00:30.000 1.0 2.0 3.0 5 7",
                    ]
                )
                + "\n",
                encoding="ascii",
            )

            result = self.run_gnss(
                "ppp-products-signoff",
                "--profile",
                "static",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--out",
                str(output_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "20",
                "--product-date",
                "2024-01-02",
                "--product",
                f"sp3={products_dir / '{yyyy}{doy}.sp3.gz'}",
                "--product",
                f"clk={products_dir / '{yyyy}{doy}.clk.gz'}",
                "--product",
                f"ionex={products_dir / '{yyyy}{doy}.ionex.gz'}",
                "--product",
                f"dcb={products_dir / '{yyyy}{doy}.bsx.gz'}",
                "--malib-pos",
                str(malib_pos),
                "--use-existing-malib",
                "--require-valid-epochs-min",
                "20",
                "--require-ppp-solution-rate-min",
                "100",
                "--require-lib-mean-error-vs-malib-max-delta",
                "100.0",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["products_signoff_profile"], "static")
            self.assertEqual(payload["product_presets"], [])
            self.assertEqual(len(payload["product_specs"]), 4)
            self.assertTrue(payload["fetch_products"])
            self.assertEqual(payload["fetched_product_date"], "2024-01-02")
            self.assertEqual(payload["comparison_target"], "MALIB")
            self.assertIn("comparison_status", payload)
            self.assertTrue(str(payload["malib_solution_pos"]).endswith("malib_static.pos"))

            failing_result = self.run_gnss(
                "ppp-products-signoff",
                "--profile",
                "static",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--out",
                str(output_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "20",
                "--product-date",
                "2024-01-02",
                "--product",
                f"sp3={products_dir / '{yyyy}{doy}.sp3.gz'}",
                "--product",
                f"clk={products_dir / '{yyyy}{doy}.clk.gz'}",
                "--product",
                f"ionex={products_dir / '{yyyy}{doy}.ionex.gz'}",
                "--product",
                f"dcb={products_dir / '{yyyy}{doy}.bsx.gz'}",
                "--malib-pos",
                str(malib_pos),
                "--use-existing-malib",
                "--require-lib-mean-error-vs-malib-max-delta",
                "-1000000000.0",
            )
            self.assertNotEqual(failing_result.returncode, 0)
            self.assertIn("PPP products comparison checks failed", failing_result.stderr)
    def test_ppp_products_signoff_cli_runs_ppc_profile_with_existing_solution(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_products_signoff_ppc_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            run_dir = temp_root / "tokyo" / "run1"
            run_dir.mkdir(parents=True)
            products_dir = temp_root / "products"
            products_dir.mkdir()
            output_path = temp_root / "ppp_ppc_products.pos"
            summary_path = temp_root / "ppp_ppc_products_summary.json"
            ppp_run_summary_path = temp_root / "ppp_run_summary.json"
            malib_pos = temp_root / "malib_ppc.pos"
            reference_csv = run_dir / "reference.csv"

            (run_dir / "rover.obs").write_bytes((ROOT_DIR / "data/rover_static.obs").read_bytes())

            nav_products = self.run_gnss(
                "nav-products",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--sp3-out",
                str(temp_root / "generated.sp3"),
                "--clk-out",
                str(temp_root / "generated.clk"),
                "--max-epochs",
                "20",
            )
            self.assertEqual(nav_products.returncode, 0, msg=nav_products.stderr)

            with gzip.open(products_dir / "2024002.sp3.gz", "wb") as stream:
                stream.write((temp_root / "generated.sp3").read_bytes())
            with gzip.open(products_dir / "2024002.clk.gz", "wb") as stream:
                stream.write((temp_root / "generated.clk").read_bytes())
            with gzip.open(products_dir / "2024002.ionex.gz", "wb") as stream:
                stream.write(build_synthetic_ionex_text().encode("ascii"))
            with gzip.open(products_dir / "2024002.bsx.gz", "wb") as stream:
                stream.write(build_synthetic_dcb_text().encode("ascii"))

            reference_rows = [
                (2300, 1000.0, 35.1000000, 139.1000000, 42.0),
                (2300, 1000.2, 35.1000100, 139.1000200, 42.2),
                (2300, 1000.4, 35.1000200, 139.1000400, 42.4),
            ]
            reference_lines = ["gps_week,gps_tow_s,lat_deg,lon_deg,height_m"]
            for week, tow, lat, lon, height in reference_rows:
                reference_lines.append(f"{week},{tow:.3f},{lat:.7f},{lon:.7f},{height:.3f}")
            reference_csv.write_text("\n".join(reference_lines) + "\n", encoding="ascii")

            with output_path.open("w", encoding="ascii") as handle:
                handle.write("% synthetic ppc ppp solution\n")
                for week, tow, lat, lon, height, status, satellites in (
                    (2300, 1000.0, 35.1000002, 139.1000001, 42.1, 6, 12),
                    (2300, 1000.2, 35.1000101, 139.1000201, 42.3, 6, 13),
                    (2300, 1000.4, 35.1000202, 139.1000402, 42.5, 5, 11),
                ):
                    ecef = driving_comparison.llh_to_ecef(lat, lon, height)
                    handle.write(
                        f"{week} {tow:.3f} {ecef[0]:.6f} {ecef[1]:.6f} {ecef[2]:.6f} "
                        f"{lat:.9f} {lon:.9f} {height:.4f} {status} {satellites} 1.0\n"
                    )

            ppp_run_summary_path.write_text(
                json.dumps(
                    {
                        "converged": True,
                        "convergence_time_s": 180.0,
                        "solution_rate_pct": 100.0,
                        "ionex_corrections": 3,
                        "ionex_meters": 0.42,
                        "dcb_corrections": 3,
                        "dcb_meters": 0.03,
                    }
                ),
                encoding="utf-8",
            )

            write_rtklib_pos(
                malib_pos,
                [
                    (2300, 1000.0, 35.1000006, 139.1000005, 42.4, 6, 12),
                    (2300, 1000.2, 35.1000106, 139.1000206, 42.6, 6, 13),
                    (2300, 1000.4, 35.1000206, 139.1000405, 42.8, 5, 11),
                ],
            )

            result = self.run_gnss(
                "ppp-products-signoff",
                "--profile",
                "ppc",
                "--run-dir",
                str(run_dir),
                "--reference-csv",
                str(reference_csv),
                "--out",
                str(output_path),
                "--summary-json",
                str(summary_path),
                "--use-existing-solution",
                "--ppp-run-summary-json",
                str(ppp_run_summary_path),
                "--product-date",
                "2024-01-02",
                "--product",
                f"sp3={products_dir / '{yyyy}{doy}.sp3.gz'}",
                "--product",
                f"clk={products_dir / '{yyyy}{doy}.clk.gz'}",
                "--product",
                f"ionex={products_dir / '{yyyy}{doy}.ionex.gz'}",
                "--product",
                f"dcb={products_dir / '{yyyy}{doy}.bsx.gz'}",
                "--malib-pos",
                str(malib_pos),
                "--require-valid-epochs-min",
                "3",
                "--require-matched-epochs-min",
                "3",
                "--require-ppp-solution-rate-min",
                "100",
                "--require-converged",
                "--require-ionex-corrections-min",
                "1",
                "--require-dcb-corrections-min",
                "1",
                "--require-lib-mean-error-vs-malib-max-delta",
                "100.0",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["products_signoff_profile"], "ppc")
            self.assertEqual(payload["dataset"], "PPC-Dataset tokyo run1")
            self.assertEqual(payload["product_presets"], [])
            self.assertTrue(payload["fetch_products"])
            self.assertEqual(payload["fetched_product_date"], "2024-01-02")
            self.assertEqual(payload["ppp_solution_rate_pct"], 100.0)
            self.assertTrue(payload["ppp_converged"])
            self.assertEqual(payload["ionex_corrections"], 3)
            self.assertEqual(payload["dcb_corrections"], 3)
            self.assertEqual(payload["comparison_target"], "MALIB")
            self.assertIn("comparison_status", payload)
            self.assertEqual(payload["common_epoch_pairs"], 3)
            self.assertTrue(str(payload["comparison_csv"]).endswith("ppp_ppc_products_comparison.csv"))
            self.assertTrue(str(payload["comparison_png"]).endswith("ppp_ppc_products_comparison.png"))
            self.assertTrue(Path(payload["comparison_csv"]).exists())
            self.assertTrue(Path(payload["comparison_png"]).exists())
            self.assertTrue(str(payload["reference_csv"]).endswith("reference.csv"))
            self.assertTrue(str(payload["run_dir"]).endswith("tokyo/run1"))
    def test_ppp_products_signoff_cli_reads_ppc_config_toml(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_products_signoff_ppc_toml_") as temp_dir:
            temp_root = Path(temp_dir)
            run_dir = temp_root / "tokyo" / "run1"
            run_dir.mkdir(parents=True)
            products_dir = temp_root / "products"
            products_dir.mkdir()
            output_path = temp_root / "ppp_ppc_products.pos"
            summary_path = temp_root / "ppp_ppc_products_summary.json"
            ppp_run_summary_path = temp_root / "ppp_run_summary.json"
            malib_pos = temp_root / "malib_ppc.pos"
            reference_csv = run_dir / "reference.csv"
            config_toml = temp_root / "ppp_products.toml"

            (run_dir / "rover.obs").write_bytes((ROOT_DIR / "data/rover_static.obs").read_bytes())
            (run_dir / "base.nav").write_bytes((ROOT_DIR / "data/navigation_static.nav").read_bytes())

            nav_products = self.run_gnss(
                "nav-products",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--sp3-out",
                str(temp_root / "generated.sp3"),
                "--clk-out",
                str(temp_root / "generated.clk"),
                "--max-epochs",
                "20",
            )
            self.assertEqual(nav_products.returncode, 0, msg=nav_products.stderr)

            with gzip.open(products_dir / "2024002.sp3.gz", "wb") as stream:
                stream.write((temp_root / "generated.sp3").read_bytes())
            with gzip.open(products_dir / "2024002.clk.gz", "wb") as stream:
                stream.write((temp_root / "generated.clk").read_bytes())
            with gzip.open(products_dir / "2024002.ionex.gz", "wb") as stream:
                stream.write(build_synthetic_ionex_text().encode("ascii"))
            with gzip.open(products_dir / "2024002.bsx.gz", "wb") as stream:
                stream.write(build_synthetic_dcb_text().encode("ascii"))

            reference_rows = [
                (2300, 1000.0, 35.1000000, 139.1000000, 42.0),
                (2300, 1000.2, 35.1000100, 139.1000200, 42.2),
                (2300, 1000.4, 35.1000200, 139.1000400, 42.4),
            ]
            reference_lines = ["gps_week,gps_tow_s,lat_deg,lon_deg,height_m"]
            for week, tow, lat, lon, height in reference_rows:
                reference_lines.append(f"{week},{tow:.3f},{lat:.7f},{lon:.7f},{height:.3f}")
            reference_csv.write_text("\n".join(reference_lines) + "\n", encoding="ascii")

            with output_path.open("w", encoding="ascii") as handle:
                handle.write("% synthetic ppc ppp solution\n")
                for week, tow, lat, lon, height, status, satellites in (
                    (2300, 1000.0, 35.1000002, 139.1000001, 42.1, 6, 12),
                    (2300, 1000.2, 35.1000101, 139.1000201, 42.3, 6, 13),
                    (2300, 1000.4, 35.1000202, 139.1000402, 42.5, 5, 11),
                ):
                    ecef = driving_comparison.llh_to_ecef(lat, lon, height)
                    handle.write(
                        f"{week} {tow:.3f} {ecef[0]:.6f} {ecef[1]:.6f} {ecef[2]:.6f} "
                        f"{lat:.9f} {lon:.9f} {height:.4f} {status} {satellites} 1.0\n"
                    )

            ppp_run_summary_path.write_text(
                json.dumps(
                    {
                        "converged": True,
                        "convergence_time_s": 180.0,
                        "solution_rate_pct": 100.0,
                        "ionex_corrections": 3,
                        "ionex_meters": 0.42,
                        "dcb_corrections": 3,
                        "dcb_meters": 0.03,
                    }
                ),
                encoding="utf-8",
            )

            write_rtklib_pos(
                malib_pos,
                [
                    (2300, 1000.0, 35.1000006, 139.1000005, 42.4, 6, 12),
                    (2300, 1000.2, 35.1000106, 139.1000206, 42.6, 6, 13),
                    (2300, 1000.4, 35.1000206, 139.1000405, 42.8, 5, 11),
                ],
            )

            product_specs = [
                f"sp3={products_dir / '{yyyy}{doy}.sp3.gz'}",
                f"clk={products_dir / '{yyyy}{doy}.clk.gz'}",
                f"ionex={products_dir / '{yyyy}{doy}.ionex.gz'}",
                f"dcb={products_dir / '{yyyy}{doy}.bsx.gz'}",
            ]
            config_lines = [
                "[ppp_products_signoff]",
                'profile = "ppc"',
                f'run_dir = "{run_dir}"',
                f'reference_csv = "{reference_csv}"',
                f'out = "{output_path}"',
                f'summary_json = "{summary_path}"',
                "use_existing_solution = true",
                f'ppp_run_summary_json = "{ppp_run_summary_path}"',
                'product_date = "2024-01-02"',
                "product = [",
            ]
            config_lines.extend(f'  "{spec}",' for spec in product_specs)
            config_lines.extend(
                [
                    "]",
                    f'malib_pos = "{malib_pos}"',
                    "use_existing_malib = true",
                    "require_valid_epochs_min = 3",
                    "require_matched_epochs_min = 3",
                    "require_ppp_solution_rate_min = 100.0",
                    "require_converged = true",
                    "require_ionex_corrections_min = 1",
                    "require_dcb_corrections_min = 1",
                    "require_lib_mean_error_vs_malib_max_delta = 100.0",
                ]
            )
            config_toml.write_text("\n".join(config_lines) + "\n", encoding="utf-8")

            result = self.run_gnss(
                "ppp-products-signoff",
                "--config-toml",
                str(config_toml),
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["products_signoff_profile"], "ppc")
            self.assertEqual(payload["comparison_target"], "MALIB")
            self.assertEqual(payload["common_epoch_pairs"], 3)
            self.assertTrue(Path(payload["comparison_csv"]).exists())
            self.assertTrue(Path(payload["comparison_png"]).exists())
    def test_ppp_products_signoff_cli_runs_ppc_profile_with_malib_bin(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_products_signoff_ppc_malib_bin_") as temp_dir:
            temp_root = Path(temp_dir)
            run_dir = temp_root / "tokyo" / "run1"
            run_dir.mkdir(parents=True)
            products_dir = temp_root / "products"
            products_dir.mkdir()
            output_path = temp_root / "ppp_ppc_products.pos"
            summary_path = temp_root / "ppp_ppc_products_summary.json"
            ppp_run_summary_path = temp_root / "ppp_run_summary.json"
            malib_pos = temp_root / "malib_ppc.pos"
            malib_bin = temp_root / "fake_malib.py"
            malib_config = temp_root / "malib.conf"
            reference_csv = run_dir / "reference.csv"

            (run_dir / "rover.obs").write_bytes((ROOT_DIR / "data/rover_static.obs").read_bytes())
            (run_dir / "base.nav").write_bytes((ROOT_DIR / "data/navigation_static.nav").read_bytes())
            malib_config.write_text("# synthetic malib config\n", encoding="utf-8")

            nav_products = self.run_gnss(
                "nav-products",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--sp3-out",
                str(temp_root / "generated.sp3"),
                "--clk-out",
                str(temp_root / "generated.clk"),
                "--max-epochs",
                "20",
            )
            self.assertEqual(nav_products.returncode, 0, msg=nav_products.stderr)

            with gzip.open(products_dir / "2024002.sp3.gz", "wb") as stream:
                stream.write((temp_root / "generated.sp3").read_bytes())
            with gzip.open(products_dir / "2024002.clk.gz", "wb") as stream:
                stream.write((temp_root / "generated.clk").read_bytes())
            with gzip.open(products_dir / "2024002.ionex.gz", "wb") as stream:
                stream.write(build_synthetic_ionex_text().encode("ascii"))
            with gzip.open(products_dir / "2024002.bsx.gz", "wb") as stream:
                stream.write(build_synthetic_dcb_text().encode("ascii"))

            reference_rows = [
                (2300, 1000.0, 35.1000000, 139.1000000, 42.0),
                (2300, 1000.2, 35.1000100, 139.1000200, 42.2),
                (2300, 1000.4, 35.1000200, 139.1000400, 42.4),
            ]
            reference_lines = ["gps_week,gps_tow_s,lat_deg,lon_deg,height_m"]
            for week, tow, lat, lon, height in reference_rows:
                reference_lines.append(f"{week},{tow:.3f},{lat:.7f},{lon:.7f},{height:.3f}")
            reference_csv.write_text("\n".join(reference_lines) + "\n", encoding="ascii")

            with output_path.open("w", encoding="ascii") as handle:
                handle.write("% synthetic ppc ppp solution\n")
                for week, tow, lat, lon, height, status, satellites in (
                    (2300, 1000.0, 35.1000002, 139.1000001, 42.1, 6, 12),
                    (2300, 1000.2, 35.1000101, 139.1000201, 42.3, 6, 13),
                    (2300, 1000.4, 35.1000202, 139.1000402, 42.5, 5, 11),
                ):
                    ecef = driving_comparison.llh_to_ecef(lat, lon, height)
                    handle.write(
                        f"{week} {tow:.3f} {ecef[0]:.6f} {ecef[1]:.6f} {ecef[2]:.6f} "
                        f"{lat:.9f} {lon:.9f} {height:.4f} {status} {satellites} 1.0\n"
                    )

            ppp_run_summary_path.write_text(
                json.dumps(
                    {
                        "converged": True,
                        "convergence_time_s": 180.0,
                        "solution_rate_pct": 100.0,
                        "ionex_corrections": 3,
                        "ionex_meters": 0.42,
                        "dcb_corrections": 3,
                        "dcb_meters": 0.03,
                    }
                ),
                encoding="utf-8",
            )

            malib_bin.write_text(
                "\n".join(
                    [
                        "#!/usr/bin/env python3",
                        "from datetime import datetime, timedelta",
                        "import pathlib, sys",
                        "out = pathlib.Path(sys.argv[sys.argv.index('-o') + 1])",
                        "start = datetime.strptime(",
                        "    sys.argv[sys.argv.index('-ts') + 1] + ' ' + sys.argv[sys.argv.index('-ts') + 2],",
                        "    '%Y/%m/%d %H:%M:%S',",
                        ")",
                        "rows = []",
                        "for offset, sats in ((0.0, 12), (0.2, 13), (0.4, 11)):",
                        "    stamp = start + timedelta(seconds=offset)",
                        "    rows.append(f\"{stamp.strftime('%Y/%m/%d %H:%M:%S.%f')[:-3]} 1.0 2.0 3.0 5 {sats}\")",
                        "out.write_text(",
                        "    '% synthetic MALIB ppc\\n' + '\\n'.join(rows) + '\\n',",
                        "    encoding='ascii',",
                        ")",
                        "print('synthetic malib ok')",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )
            malib_bin.chmod(0o755)

            result = self.run_gnss(
                "ppp-products-signoff",
                "--profile",
                "ppc",
                "--run-dir",
                str(run_dir),
                "--reference-csv",
                str(reference_csv),
                "--out",
                str(output_path),
                "--summary-json",
                str(summary_path),
                "--use-existing-solution",
                "--ppp-run-summary-json",
                str(ppp_run_summary_path),
                "--product-date",
                "2024-01-02",
                "--product",
                f"sp3={products_dir / '{yyyy}{doy}.sp3.gz'}",
                "--product",
                f"clk={products_dir / '{yyyy}{doy}.clk.gz'}",
                "--product",
                f"ionex={products_dir / '{yyyy}{doy}.ionex.gz'}",
                "--product",
                f"dcb={products_dir / '{yyyy}{doy}.bsx.gz'}",
                "--malib-bin",
                str(malib_bin),
                "--malib-config",
                str(malib_config),
                "--malib-pos",
                str(malib_pos),
                "--require-valid-epochs-min",
                "3",
                "--require-matched-epochs-min",
                "3",
                "--require-converged",
                "--require-common-epoch-pairs-min",
                "3",
                "--require-lib-mean-error-vs-malib-max-delta",
                "100.0",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["comparison_target"], "MALIB")
            self.assertEqual(payload["common_epoch_pairs"], 3)
            self.assertTrue(Path(payload["malib_solution_pos"]).exists())
            self.assertTrue(Path(payload["comparison_csv"]).exists())
            self.assertTrue(Path(payload["comparison_png"]).exists())
    def test_ppp_products_signoff_cli_runs_kinematic_profile_with_malib_bin(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_products_signoff_kinematic_malib_bin_") as temp_dir:
            temp_root = Path(temp_dir)
            generated_sp3 = temp_root / "generated.sp3"
            generated_clk = temp_root / "generated.clk"
            products_dir = temp_root / "products"
            products_dir.mkdir()
            output_path = temp_root / "ppp_kinematic_products.pos"
            reference_path = temp_root / "ppp_kinematic_products_reference.pos"
            summary_path = temp_root / "ppp_kinematic_products_summary.json"
            malib_pos = temp_root / "malib_kinematic.pos"
            malib_bin = temp_root / "fake_malib_kinematic.py"
            malib_config = temp_root / "malib_kinematic.conf"

            malib_config.write_text("# synthetic malib kinematic config\n", encoding="utf-8")

            nav_products = self.run_gnss(
                "nav-products",
                "--obs",
                str(ROOT_DIR / "data/rover_kinematic.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_kinematic.nav"),
                "--sp3-out",
                str(generated_sp3),
                "--clk-out",
                str(generated_clk),
                "--max-epochs",
                "20",
            )
            self.assertEqual(nav_products.returncode, 0, msg=nav_products.stderr)

            with gzip.open(products_dir / "2024002.sp3.gz", "wb") as stream:
                stream.write(generated_sp3.read_bytes())
            with gzip.open(products_dir / "2024002.clk.gz", "wb") as stream:
                stream.write(generated_clk.read_bytes())
            with gzip.open(products_dir / "2024002.ionex.gz", "wb") as stream:
                stream.write(build_synthetic_ionex_text().encode("ascii"))
            with gzip.open(products_dir / "2024002.bsx.gz", "wb") as stream:
                stream.write(build_synthetic_dcb_text().encode("ascii"))

            malib_bin.write_text(
                "\n".join(
                    [
                        "#!/usr/bin/env python3",
                        "from datetime import datetime, timedelta",
                        "import pathlib, sys",
                        "out = pathlib.Path(sys.argv[sys.argv.index('-o') + 1])",
                        "start = datetime.strptime(",
                        "    sys.argv[sys.argv.index('-ts') + 1] + ' ' + sys.argv[sys.argv.index('-ts') + 2],",
                        "    '%Y/%m/%d %H:%M:%S.%f',",
                        ")",
                        "end = datetime.strptime(",
                        "    sys.argv[sys.argv.index('-te') + 1] + ' ' + sys.argv[sys.argv.index('-te') + 2],",
                        "    '%Y/%m/%d %H:%M:%S.%f',",
                        ")",
                        "rows = ['% synthetic MALIB kinematic']",
                        "stamp = start",
                        "index = 0",
                        "while stamp <= end:",
                        "    rows.append(",
                        "        f\"{stamp.strftime('%Y/%m/%d %H:%M:%S.%f')[:-3]} 2077971.{1200 + index:04d} -5684020.{7100 + index:04d} 2006789.{6250 + index:04d} 5 18\"",
                        "    )",
                        "    stamp += timedelta(seconds=15)",
                        "    index += 1",
                        "out.write_text('\\n'.join(rows) + '\\n', encoding='ascii')",
                        "print('synthetic kinematic malib ok')",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )
            malib_bin.chmod(0o755)

            result = self.run_gnss(
                "ppp-products-signoff",
                "--profile",
                "kinematic",
                "--obs",
                str(ROOT_DIR / "data/rover_kinematic.obs"),
                "--base",
                str(ROOT_DIR / "data/base_kinematic.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_kinematic.nav"),
                "--out",
                str(output_path),
                "--reference-pos",
                str(reference_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "20",
                "--product-date",
                "2024-01-02",
                "--product",
                f"sp3={products_dir / '{yyyy}{doy}.sp3.gz'}",
                "--product",
                f"clk={products_dir / '{yyyy}{doy}.clk.gz'}",
                "--product",
                f"ionex={products_dir / '{yyyy}{doy}.ionex.gz'}",
                "--product",
                f"dcb={products_dir / '{yyyy}{doy}.bsx.gz'}",
                "--malib-bin",
                str(malib_bin),
                "--malib-config",
                str(malib_config),
                "--malib-pos",
                str(malib_pos),
                "--require-common-epoch-pairs-min",
                "20",
                "--require-ppp-solution-rate-min",
                "100",
                "--require-lib-mean-error-vs-malib-max-delta",
                "1000000.0",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["products_signoff_profile"], "kinematic")
            self.assertEqual(payload["comparison_target"], "MALIB")
            self.assertEqual(payload["reference_common_epoch_pairs"], 20)
            self.assertEqual(payload["common_epoch_pairs"], 20)
            self.assertTrue(Path(payload["reference_pos"]).exists())
            self.assertTrue(Path(payload["malib_solution_pos"]).exists())
            self.assertTrue(Path(payload["comparison_csv"]).exists())
            self.assertTrue(Path(payload["comparison_png"]).exists())
            self.assertIn("comparison_status", payload)
    def test_ppc_demo_cli_summarizes_existing_solution_against_reference_csv(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_demo_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            run_dir = temp_root / "tokyo" / "run1"
            run_dir.mkdir(parents=True)
            solution_path = temp_root / "ppc_demo.pos"
            rtklib_path = temp_root / "ppc_demo_rtklib.pos"
            commercial_path = temp_root / "ppc_commercial_receiver.csv"
            commercial_matches = temp_root / "ppc_commercial_matches.csv"
            summary_path = temp_root / "ppc_demo_summary.json"
            reference_csv = run_dir / "reference.csv"

            reference_rows = [
                (2300, 1000.0, 35.1000000, 139.1000000, 42.0),
                (2300, 1000.2, 35.1000100, 139.1000200, 42.2),
                (2300, 1000.4, 35.1000200, 139.1000400, 42.4),
            ]
            reference_lines = [
                "gps_week,gps_tow_s,lat_deg,lon_deg,height_m,roll_deg,pitch_deg,yaw_deg"
            ]
            for week, tow, lat, lon, height in reference_rows:
                reference_lines.append(
                    f"{week},{tow:.3f},{lat:.7f},{lon:.7f},{height:.3f},0.0,0.0,0.0"
                )
            reference_csv.write_text("\n".join(reference_lines) + "\n", encoding="ascii")

            with solution_path.open("w", encoding="ascii") as handle:
                handle.write("% synthetic ppc demo solution\n")
                for week, tow, lat, lon, height, status, satellites in (
                    (2300, 1000.0, 35.1000002, 139.1000001, 42.1, 4, 12),
                    (2300, 1000.2, 35.1000099, 139.1000202, 42.3, 4, 13),
                    (2300, 1000.4, 35.1000197, 139.1000398, 42.5, 3, 11),
                ):
                    ecef = driving_comparison.llh_to_ecef(lat, lon, height)
                    handle.write(
                        f"{week} {tow:.3f} {ecef[0]:.6f} {ecef[1]:.6f} {ecef[2]:.6f} "
                        f"{lat:.9f} {lon:.9f} {height:.4f} {status} {satellites} 1.0\n"
                    )
            write_rtklib_pos(
                rtklib_path,
                [
                    (2300, 1000.0, 35.1000004, 139.1000003, 42.2, 1, 12),
                    (2300, 1000.2, 35.1000103, 139.1000205, 42.4, 1, 13),
                    (2300, 1000.4, 35.1000205, 139.1000404, 42.6, 2, 11),
                ],
            )
            commercial_path.write_text(
                "\n".join(
                    [
                        "gps_week,gps_tow_s,lat_deg,lon_deg,height_m,solution_status,num_satellites",
                        "2300,1000.0,35.1000001,139.1000001,42.1,rtk_fixed,14",
                        "2300,1000.2,35.1000101,139.1000201,42.2,rtk_fixed,14",
                        "2300,1000.4,35.1000201,139.1000401,42.4,rtk_float,13",
                    ]
                )
                + "\n",
                encoding="ascii",
            )

            result = self.run_gnss(
                "ppc-demo",
                "--run-dir",
                str(run_dir),
                "--solver",
                "rtk",
                "--use-existing-solution",
                "--solver-wall-time-s",
                "0.5",
                "--out",
                str(solution_path),
                "--rtklib-pos",
                str(rtklib_path),
                "--use-existing-rtklib-solution",
                "--rtklib-solver-wall-time-s",
                "0.1",
                "--commercial-pos",
                str(commercial_path),
                "--commercial-label",
                "survey_receiver",
                "--commercial-matched-csv",
                str(commercial_matches),
                "--commercial-solver-wall-time-s",
                "0.2",
                "--summary-json",
                str(summary_path),
                "--require-valid-epochs-min",
                "3",
                "--require-matched-epochs-min",
                "3",
                "--require-fix-rate-min",
                "60",
                "--require-median-h-max",
                "0.2",
                "--require-p95-h-max",
                "0.2",
                "--require-max-h-max",
                "0.2",
                "--require-p95-up-max",
                "0.2",
                "--require-mean-sats-min",
                "11",
                "--require-solver-wall-time-max",
                "1.0",
                "--require-realtime-factor-min",
                "0.5",
                "--require-effective-epoch-rate-min",
                "5.0",
                "--require-lib-fix-rate-vs-rtklib-min-delta",
                "0.0",
                "--require-lib-median-h-vs-rtklib-max-delta",
                "0.0",
                "--require-lib-p95-h-vs-rtklib-max-delta",
                "0.0",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("Finished PPC-Dataset demo.", result.stdout)
            self.assertTrue(summary_path.exists())

            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["dataset"], "PPC-Dataset tokyo run1")
            self.assertEqual(payload["solver"], "rtk")
            provenance = payload["receiver_observation_provenance"]
            self.assertEqual(provenance["vehicle_receiver"], "Septentrio mosaic-X5")
            self.assertEqual(provenance["reference_station_receiver"], "Trimble Alloy")
            self.assertFalse(provenance["receiver_engine_solution_available"])
            self.assertEqual(payload["valid_epochs"], 3)
            self.assertEqual(payload["matched_epochs"], 3)
            self.assertEqual(payload["fixed_epochs"], 2)
            self.assertGreaterEqual(payload["fix_rate_pct"], 60.0)
            self.assertLessEqual(payload["p95_h_m"], 0.2)
            self.assertLessEqual(payload["p95_abs_up_m"], 0.2)
            self.assertGreaterEqual(payload["mean_satellites"], 11.0)
            self.assertEqual(payload["solver_wall_time_s"], 0.5)
            self.assertEqual(payload["solution_span_s"], 0.4)
            self.assertEqual(payload["realtime_factor"], 0.8)
            self.assertEqual(payload["effective_epoch_rate_hz"], 6.0)
            self.assertIn("rtklib", payload)
            self.assertEqual(payload["rtklib"]["matched_epochs"], 3)
            self.assertEqual(payload["rtklib"]["solver_wall_time_s"], 0.1)
            self.assertAlmostEqual(payload["rtklib"]["realtime_factor"], 4.0, places=5)
            self.assertIn("delta_vs_rtklib", payload)
            self.assertIn("commercial_receiver", payload)
            self.assertEqual(payload["commercial_receiver"]["label"], "survey_receiver")
            self.assertEqual(payload["commercial_receiver"]["matched_epochs"], 3)
            self.assertEqual(payload["commercial_receiver"]["fixed_epochs"], 2)
            self.assertEqual(payload["commercial_receiver"]["matched_csv"], str(commercial_matches))
            self.assertTrue(commercial_matches.exists())
            self.assertIn("delta_vs_commercial_receiver", payload)
            self.assertIn("rtklib performance: wall=0.1 s", result.stdout)
            self.assertIn("commercial_receiver:", result.stdout)
            self.assertIn("performance: wall=0.5 s, span=0.4 s, rtf=0.8, rate=6.0 Hz", result.stdout)
    def test_ppc_demo_cli_summarizes_existing_spp_solution_against_reference_csv(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_demo_spp_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            run_dir = temp_root / "tokyo" / "run1"
            run_dir.mkdir(parents=True)
            solution_path = temp_root / "ppc_demo_spp.pos"
            summary_path = temp_root / "ppc_demo_spp_summary.json"
            spp_run_summary = temp_root / "gnss_spp_summary.json"
            reference_csv = run_dir / "reference.csv"

            reference_csv.write_text(
                "\n".join(
                    [
                        "gps_week,gps_tow_s,lat_deg,lon_deg,height_m",
                        "2300,1000.000,35.1000000,139.1000000,42.000",
                        "2300,1000.200,35.1000100,139.1000200,42.200",
                        "2300,1000.400,35.1000200,139.1000400,42.400",
                    ]
                )
                + "\n",
                encoding="ascii",
            )
            with solution_path.open("w", encoding="ascii") as handle:
                handle.write("% synthetic ppc spp solution\n")
                for week, tow, lat, lon, height, status, satellites in (
                    (2300, 1000.0, 35.1000001, 139.1000001, 42.1, 1, 12),
                    (2300, 1000.2, 35.1000101, 139.1000201, 42.3, 1, 13),
                    (2300, 1000.4, 35.1000202, 139.1000402, 42.5, 0, 11),
                ):
                    ecef = driving_comparison.llh_to_ecef(lat, lon, height)
                    handle.write(
                        f"{week} {tow:.3f} {ecef[0]:.6f} {ecef[1]:.6f} {ecef[2]:.6f} "
                        f"{lat:.9f} {lon:.9f} {height:.4f} {status} {satellites} 1.0\n"
                    )
            spp_run_summary.write_text(
                json.dumps(
                    {
                        "availability_rate": 1.0,
                        "mean_residual_rms_m": 1.25,
                        "spp_qc": {
                            "outlier_rejections": 4,
                            "raim_fde_rejections": 1,
                            "position_jump_gate_rejections": 3,
                            "robust_weighted_measurements": 2,
                            "adaptive_robust_activations": 7,
                            "adaptive_robust_tail_measurements": 9,
                            "min_robust_weight_factor": 0.25,
                            "precise_orbit_clock_measurements": 2,
                            "ssr_orbit_clock_corrections": 3,
                            "ssr_code_bias_corrections": 5,
                            "ionex_corrections": 6,
                            "ionex_meters": 0.7,
                            "dcb_corrections": 8,
                            "dcb_meters": 0.9,
                        },
                    }
                ),
                encoding="utf-8",
            )

            result = self.run_gnss(
                "ppc-demo",
                "--run-dir",
                str(run_dir),
                "--solver",
                "spp",
                "--use-existing-solution",
                "--solver-wall-time-s",
                "0.5",
                "--out",
                str(solution_path),
                "--spp-summary-json",
                str(spp_run_summary),
                "--summary-json",
                str(summary_path),
                "--require-valid-epochs-min",
                "3",
                "--require-matched-epochs-min",
                "3",
                "--require-fix-rate-min",
                "60",
                "--require-median-h-max",
                "0.2",
                "--require-p95-up-max",
                "0.2",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["solver"], "spp")
            self.assertEqual(payload["fixed_epochs"], 2)
            self.assertEqual(payload["spp_solution_rate_pct"], 100.0)
            self.assertEqual(payload["spp_mean_residual_rms_m"], 1.25)
            self.assertEqual(payload["spp_outlier_rejections"], 4)
            self.assertEqual(payload["spp_raim_fde_rejections"], 1)
            self.assertEqual(payload["spp_position_jump_gate_rejections"], 3)
            self.assertEqual(payload["spp_robust_weighted_measurements"], 2)
            self.assertEqual(payload["spp_adaptive_robust_activations"], 7)
            self.assertEqual(payload["spp_adaptive_robust_tail_measurements"], 9)
            self.assertEqual(payload["spp_min_robust_weight_factor"], 0.25)
            self.assertEqual(payload["spp_precise_orbit_clock_measurements"], 2)
            self.assertEqual(payload["spp_ssr_orbit_clock_corrections"], 3)
            self.assertEqual(payload["spp_ssr_code_bias_corrections"], 5)
            self.assertEqual(payload["ionex_corrections"], 6)
            self.assertEqual(payload["dcb_corrections"], 8)
    def test_ppc_spp_jump_sweep_scores_posthoc_gate_candidates(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_spp_jump_sweep_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            reference_csv = temp_root / "reference.csv"
            solution_path = temp_root / "spp.pos"
            summary_path = temp_root / "sweep.json"
            csv_path = temp_root / "sweep.csv"
            filtered_path = temp_root / "spp_filtered.pos"
            policy_summary_path = temp_root / "sweep_policy.json"
            policy_csv_path = temp_root / "sweep_policy.csv"
            policy_filtered_path = temp_root / "spp_policy_filtered.pos"

            reference_rows = [
                (2300, 1000.0, 35.1000000, 139.1000000, 42.0),
                (2300, 1000.2, 35.1000010, 139.1000010, 42.0),
                (2300, 1000.4, 35.1000020, 139.1000020, 42.0),
                (2300, 1000.6, 35.1000030, 139.1000030, 42.0),
            ]
            reference_lines = ["gps_week,gps_tow_s,lat_deg,lon_deg,height_m"]
            for week, tow, lat, lon, height in reference_rows:
                reference_lines.append(f"{week},{tow:.3f},{lat:.7f},{lon:.7f},{height:.3f}")
            reference_csv.write_text("\n".join(reference_lines) + "\n", encoding="ascii")

            write_libgnss_pos(
                solution_path,
                [
                    (2300, 1000.0, 35.1000000, 139.1000000, 42.0, 1, 12, 1.0),
                    (2300, 1000.2, 35.1010000, 139.1010000, 42.0, 1, 12, 1.0),
                    (2300, 1000.4, 35.1000020, 139.1000020, 42.0, 1, 12, 1.0),
                    (2300, 1000.6, 35.1000030, 139.1000030, 42.0, 1, 12, 1.0),
                ],
            )

            result = self.run_gnss(
                "ppc-spp-jump-sweep",
                "--reference-csv",
                str(reference_csv),
                "--pos",
                str(solution_path),
                "--rates-mps",
                "50",
                "--min-jumps-m",
                "20",
                "--summary-json",
                str(summary_path),
                "--csv",
                str(csv_path),
                "--filtered-pos-out",
                str(filtered_path),
                "--filtered-rate-mps",
                "50",
                "--filtered-min-jump-m",
                "20",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("PPC SPP jump-gate sweep", result.stdout)
            self.assertIn("filtered_pos:", result.stdout)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["candidate_count"], 2)
            gated = [
                row for row in payload["results"]
                if row["max_position_jump_rate_mps"] == 50.0
            ][0]
            baseline = [
                row for row in payload["results"]
                if row["max_position_jump_rate_mps"] is None
            ][0]
            self.assertEqual(gated["jump_gate_rejected_epochs"], 1)
            self.assertEqual(gated["valid_epochs"], 3)
            self.assertLess(gated["max_h_m"], baseline["max_h_m"])
            self.assertTrue(csv_path.exists())
            self.assertEqual(payload["filtered_pos"]["path"], str(filtered_path))
            filtered_epochs = driving_comparison.read_libgnss_pos(filtered_path)
            self.assertEqual(len(filtered_epochs), 3)
            self.assertEqual([epoch.tow for epoch in filtered_epochs], [1000.0, 1000.4, 1000.6])

            policy_result = self.run_gnss(
                "ppc-spp-jump-sweep",
                "--reference-csv",
                str(reference_csv),
                "--pos",
                str(solution_path),
                "--rates-mps",
                "50",
                "--min-jumps-m",
                "20",
                "--summary-json",
                str(policy_summary_path),
                "--csv",
                str(policy_csv_path),
                "--max-positioning-drop-pct",
                "10",
                "--filtered-pos-out",
                str(policy_filtered_path),
            )

            self.assertEqual(policy_result.returncode, 0, msg=policy_result.stderr)
            self.assertIn("policy_best:", policy_result.stdout)
            policy_payload = json.loads(policy_summary_path.read_text(encoding="utf-8"))
            self.assertTrue(policy_payload["policy"]["enabled"])
            self.assertEqual(policy_payload["policy"]["candidate_count"], 1)
            self.assertIsNone(
                policy_payload["policy"]["best_by_p95_h_m"]["max_position_jump_rate_mps"]
            )
            self.assertEqual(
                policy_payload["policy"]["best_by_p95_h_m"]["positioning_drop_pct"],
                0.0,
            )
            policy_filtered_epochs = driving_comparison.read_libgnss_pos(policy_filtered_path)
            self.assertEqual(len(policy_filtered_epochs), 4)
    def test_ppc_spp_policy_report_summarizes_sweep_jsons(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_spp_policy_report_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            sweep_a_path = temp_root / "sweep_a.json"
            sweep_b_path = temp_root / "sweep_b.json"
            summary_path = temp_root / "policy_report.json"
            csv_path = temp_root / "policy_report.csv"
            failing_summary_path = temp_root / "policy_report_fail.json"
            failing_csv_path = temp_root / "policy_report_fail.csv"

            def write_sweep(
                path: Path,
                *,
                baseline_positioning: float,
                baseline_p95: float,
                policy_positioning: float,
                policy_p95: float,
            ) -> None:
                baseline = {
                    "max_position_jump_rate_mps": None,
                    "max_position_jump_min_m": None,
                    "valid_epochs": 100,
                    "matched_epochs": 100,
                    "positioning_rate_pct": baseline_positioning,
                    "median_h_m": 2.0,
                    "p95_h_m": baseline_p95,
                    "max_h_m": 20.0,
                    "p95_abs_up_m": 9.0,
                    "jump_gate_rejected_epochs": 0,
                    "jump_gate_rejected_groups": 0,
                    "bridge_inserted_epochs": 0,
                    "positioning_drop_pct": 0.0,
                }
                policy = {
                    "max_position_jump_rate_mps": 50.0,
                    "max_position_jump_min_m": 30.0,
                    "valid_epochs": 99,
                    "matched_epochs": 99,
                    "positioning_rate_pct": policy_positioning,
                    "median_h_m": 1.5,
                    "p95_h_m": policy_p95,
                    "max_h_m": 15.0,
                    "p95_abs_up_m": 7.0,
                    "jump_gate_rejected_epochs": 5,
                    "jump_gate_rejected_groups": 2,
                    "bridge_inserted_epochs": 1,
                    "positioning_drop_pct": baseline_positioning - policy_positioning,
                }
                path.write_text(
                    json.dumps(
                        {
                            "reference_csv": "reference.csv",
                            "pos": "spp.pos",
                            "bridge_max_gap_s": 5.0,
                            "bridge_max_anchor_speed_mps": 10.0,
                            "results": [baseline, policy],
                            "policy": {
                                "enabled": True,
                                "max_positioning_drop_pct": 1.0,
                                "min_positioning_rate_pct": None,
                                "candidate_count": 1,
                                "best_by_p95_h_m": policy,
                            },
                        },
                        indent=2,
                    )
                    + "\n",
                    encoding="ascii",
                )

            write_sweep(
                sweep_a_path,
                baseline_positioning=100.0,
                baseline_p95=10.0,
                policy_positioning=99.0,
                policy_p95=8.0,
            )
            write_sweep(
                sweep_b_path,
                baseline_positioning=98.5,
                baseline_p95=6.0,
                policy_positioning=98.0,
                policy_p95=6.0,
            )

            result = self.run_gnss(
                "ppc-spp-policy-report",
                "--sweep",
                f"run_a={sweep_a_path}",
                "--sweep",
                f"run_b={sweep_b_path}",
                "--summary-json",
                str(summary_path),
                "--csv",
                str(csv_path),
                "--max-p95-delta-m",
                "0",
                "--max-positioning-drop-pct",
                "1.0",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("PPC SPP policy report", result.stdout)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertTrue(payload["checks"]["passed"])
            self.assertEqual(payload["sweep_count"], 2)
            run_a = [row for row in payload["runs"] if row["label"] == "run_a"][0]
            self.assertEqual(run_a["selected_rate_mps"], 50.0)
            self.assertEqual(run_a["selected_min_jump_m"], 30.0)
            self.assertEqual(run_a["valid_epoch_delta"], -1)
            self.assertEqual(run_a["positioning_drop_pct"], 1.0)
            self.assertEqual(run_a["p95_h_delta_m"], -2.0)
            self.assertEqual(run_a["bridge_inserted_epochs"], 1)

            csv_rows = list(csv.DictReader(csv_path.open(encoding="utf-8")))
            self.assertEqual(len(csv_rows), 2)
            self.assertEqual(csv_rows[0]["label"], "run_a")
            self.assertEqual(float(csv_rows[0]["p95_h_delta_m"]), -2.0)

            failing_result = self.run_gnss(
                "ppc-spp-policy-report",
                "--sweep",
                f"run_a={sweep_a_path}",
                "--summary-json",
                str(failing_summary_path),
                "--csv",
                str(failing_csv_path),
                "--max-positioning-drop-pct",
                "0.5",
            )

            self.assertEqual(failing_result.returncode, 2, msg=failing_result.stderr)
            self.assertIn("check failures", failing_result.stderr)
            failing_payload = json.loads(failing_summary_path.read_text(encoding="utf-8"))
            self.assertFalse(failing_payload["checks"]["passed"])
    def test_ppc_spp_policy_suite_runs_sweeps_report_and_compare(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_spp_policy_suite_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            output_dir = temp_root / "suite"
            suite_summary_path = output_dir / "synthetic_suite_summary.json"

            run_args: list[str] = []
            direct_csv_run_arg: str | None = None
            manifest_runs: list[dict[str, str]] = []
            for label, lon_offset in (("run_a", 0.0), ("run_b", 0.001)):
                run_dir = temp_root / label
                run_dir.mkdir()
                reference_csv = run_dir / "reference.csv"
                solution_path = run_dir / "spp.pos"
                baseline_path = run_dir / "baseline.pos"
                reference_rows = [
                    (2300, 1000.0, 35.1000000, 139.1000000 + lon_offset, 42.0),
                    (2300, 1000.2, 35.1000010, 139.1000010 + lon_offset, 42.0),
                    (2300, 1000.4, 35.1000020, 139.1000020 + lon_offset, 42.0),
                    (2300, 1000.6, 35.1000030, 139.1000030 + lon_offset, 42.0),
                ]
                reference_lines = ["gps_week,gps_tow_s,lat_deg,lon_deg,height_m"]
                for week, tow, lat, lon, height in reference_rows:
                    reference_lines.append(f"{week},{tow:.3f},{lat:.7f},{lon:.7f},{height:.3f}")
                reference_csv.write_text("\n".join(reference_lines) + "\n", encoding="ascii")
                if label == "run_a":
                    named_reference_csv = run_dir / "ppc_reference.csv"
                    named_reference_csv.write_text(reference_csv.read_text(encoding="ascii"), encoding="ascii")
                    direct_csv_run_arg = f"run_csv={named_reference_csv},{solution_path},{baseline_path}"

                write_libgnss_pos(
                    solution_path,
                    [
                        (2300, 1000.0, 35.1000000, 139.1000000 + lon_offset, 42.0, 1, 12, 1.0),
                        (2300, 1000.2, 35.1010000, 139.1010000 + lon_offset, 42.0, 1, 12, 1.0),
                        (2300, 1000.4, 35.1000020, 139.1000020 + lon_offset, 42.0, 1, 12, 1.0),
                        (2300, 1000.6, 35.1000030, 139.1000030 + lon_offset, 42.0, 1, 12, 1.0),
                    ],
                )
                write_libgnss_pos(
                    baseline_path,
                    [
                        (2300, 1000.0, 35.1000000, 139.1000000 + lon_offset, 42.0, 1, 12, 1.0),
                        (2300, 1000.2, 35.1012000, 139.1012000 + lon_offset, 42.0, 1, 12, 1.0),
                        (2300, 1000.4, 35.1000020, 139.1000020 + lon_offset, 42.0, 1, 12, 1.0),
                        (2300, 1000.6, 35.1000030, 139.1000030 + lon_offset, 42.0, 1, 12, 1.0),
                    ],
                )
                run_args.extend(
                    [
                        "--run",
                        f"{label}={run_dir},{solution_path},{baseline_path}",
                    ]
                )
                manifest_runs.append(
                    {
                        "label": label,
                        "run_dir": str(run_dir),
                        "input_pos": str(solution_path),
                        "baseline_pos": str(baseline_path),
                    }
                )

            result = self.run_gnss(
                "ppc-spp-policy-suite",
                *run_args,
                "--output-dir",
                str(output_dir),
                "--prefix",
                "synthetic",
                "--rates-mps",
                "50",
                "--min-jumps-m",
                "20",
                "--max-positioning-drop-pct",
                "30",
                "--max-p95-delta-m",
                "0",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("PPC SPP policy suite", result.stdout)
            payload = json.loads(suite_summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["run_count"], 2)
            self.assertFalse(payload["dry_run"])
            self.assertTrue(payload["checks"]["passed"])
            self.assertEqual(payload["checks"]["dry_run"], False)
            self.assertLessEqual(payload["worst_p95_h_delta_m"], 0.0)
            self.assertEqual(payload["worst_positioning_drop_pct"], 25.0)
            self.assertEqual(len(payload["policy_runs"]), 2)
            self.assertEqual(payload["policy_runs"][0]["selected_rate_mps"], 50.0)
            self.assertTrue(Path(payload["policy_report_json"]).exists())
            policy_payload = json.loads(Path(payload["policy_report_json"]).read_text(encoding="utf-8"))
            self.assertTrue(policy_payload["checks"]["passed"])
            self.assertEqual(policy_payload["sweep_count"], 2)
            for record in payload["runs"]:
                self.assertTrue(Path(record["sweep_json"]).exists())
                self.assertTrue(Path(record["sweep_csv"]).exists())
                self.assertTrue(Path(record["filtered_pos"]).exists())
                self.assertTrue(Path(record["compare_summary_json"]).exists())
                self.assertTrue(Path(record["compare_csv"]).exists())
                self.assertTrue(Path(record["compare_matched_csv"]).exists())
                compare_png = Path(record["compare_png"])
                self.assertTrue(compare_png.exists())
                self.assertGreater(compare_png.stat().st_size, 0)

            self.assertIsNotNone(direct_csv_run_arg)
            direct_csv_output_dir = temp_root / "direct_csv_suite"
            direct_csv_result = self.run_gnss(
                "ppc-spp-policy-suite",
                "--run",
                direct_csv_run_arg,
                "--output-dir",
                str(direct_csv_output_dir),
                "--prefix",
                "direct_csv",
                "--rates-mps",
                "50",
                "--min-jumps-m",
                "20",
                "--dry-run",
            )

            self.assertEqual(direct_csv_result.returncode, 0, msg=direct_csv_result.stderr)
            direct_csv_payload = json.loads(
                (direct_csv_output_dir / "direct_csv_suite_summary.json").read_text(
                    encoding="utf-8"
                )
            )
            self.assertEqual(
                direct_csv_payload["runs"][0]["reference_csv"],
                direct_csv_run_arg.split("=", 1)[1].split(",", 1)[0],
            )

            manifest_output_dir = temp_root / "manifest_suite"
            manifest_path = temp_root / "policy_suite_manifest.json"
            manifest_path.write_text(
                json.dumps(
                    {
                        "output_dir": str(manifest_output_dir),
                        "prefix": "manifested",
                        "rates_mps": "50",
                        "min_jumps_m": "20",
                        "max_positioning_drop_pct": 30.0,
                        "max_p95_delta_m": 0.0,
                        "skip_compare": True,
                        "runs": manifest_runs[:1],
                    }
                ),
                encoding="utf-8",
            )

            manifest_result = self.run_gnss(
                "ppc-spp-policy-suite",
                "--manifest-json",
                str(manifest_path),
            )

            self.assertEqual(manifest_result.returncode, 0, msg=manifest_result.stderr)
            manifest_summary_path = manifest_output_dir / "manifested_suite_summary.json"
            manifest_payload = json.loads(manifest_summary_path.read_text(encoding="utf-8"))
            self.assertEqual(manifest_payload["manifest_json"], str(manifest_path))
            self.assertEqual(manifest_payload["output_dir"], str(manifest_output_dir))
            self.assertEqual(manifest_payload["prefix"], "manifested")
            self.assertEqual(manifest_payload["rates_mps"], "50")
            self.assertEqual(manifest_payload["min_jumps_m"], "20")
            self.assertEqual(manifest_payload["max_positioning_drop_pct"], 30.0)
            self.assertEqual(manifest_payload["max_p95_delta_m"], 0.0)
            self.assertEqual(manifest_payload["run_count"], 1)
            self.assertTrue(manifest_payload["checks"]["passed"])
            self.assertLessEqual(manifest_payload["worst_p95_h_delta_m"], 0.0)
            self.assertEqual(manifest_payload["worst_positioning_drop_pct"], 25.0)
            self.assertNotIn("compare_summary_json", manifest_payload["runs"][0])
            self.assertTrue(Path(manifest_payload["policy_report_json"]).exists())

            dry_run_output_dir = temp_root / "dry_run_suite"
            dry_run_result = self.run_gnss(
                "ppc-spp-policy-suite",
                "--manifest-json",
                str(manifest_path),
                "--output-dir",
                str(dry_run_output_dir),
                "--prefix",
                "dry_run",
                "--dry-run",
            )

            self.assertEqual(dry_run_result.returncode, 0, msg=dry_run_result.stderr)
            self.assertIn("dry_run: true", dry_run_result.stdout)
            dry_run_summary_path = dry_run_output_dir / "dry_run_suite_summary.json"
            dry_run_payload = json.loads(dry_run_summary_path.read_text(encoding="utf-8"))
            self.assertTrue(dry_run_payload["dry_run"])
            self.assertIsNone(dry_run_payload["checks"]["passed"])
            self.assertTrue(dry_run_payload["checks"]["dry_run"])
            self.assertIsNone(dry_run_payload["worst_p95_h_delta_m"])
            self.assertEqual(dry_run_payload["policy_runs"], [])
            self.assertEqual(dry_run_payload["run_count"], 1)
            self.assertEqual(len(dry_run_payload["commands"]), 2)
            self.assertFalse(Path(dry_run_payload["runs"][0]["sweep_json"]).exists())
            self.assertFalse(Path(dry_run_payload["policy_report_json"]).exists())
    def test_ppc_spp_compare_generates_summary_csv_and_png(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_spp_compare_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            reference_csv = temp_root / "reference.csv"
            baseline_path = temp_root / "baseline.pos"
            improved_path = temp_root / "improved.pos"
            summary_path = temp_root / "compare.json"
            csv_path = temp_root / "compare.csv"
            matched_csv = temp_root / "compare_matches.csv"
            png_path = temp_root / "compare.png"

            reference_rows = [
                (2300, 1000.0, 35.1000000, 139.1000000, 42.0),
                (2300, 1000.2, 35.1000010, 139.1000010, 42.0),
                (2300, 1000.4, 35.1000020, 139.1000020, 42.0),
                (2300, 1000.6, 35.1000030, 139.1000030, 42.0),
            ]
            reference_lines = ["gps_week,gps_tow_s,lat_deg,lon_deg,height_m"]
            for week, tow, lat, lon, height in reference_rows:
                reference_lines.append(f"{week},{tow:.3f},{lat:.7f},{lon:.7f},{height:.3f}")
            reference_csv.write_text("\n".join(reference_lines) + "\n", encoding="ascii")

            write_libgnss_pos(
                baseline_path,
                [
                    (2300, 1000.0, 35.1000000, 139.1000000, 42.0, 1, 12, 1.0),
                    (2300, 1000.2, 35.1010000, 139.1010000, 42.0, 1, 12, 1.0),
                    (2300, 1000.4, 35.1000020, 139.1000020, 42.0, 1, 12, 1.0),
                    (2300, 1000.6, 35.1000030, 139.1000030, 42.0, 1, 12, 1.0),
                ],
            )
            write_libgnss_pos(
                improved_path,
                [
                    (2300, 1000.0, 35.1000000, 139.1000000, 42.0, 1, 12, 1.0),
                    (2300, 1000.2, 35.1000010, 139.1000010, 42.0, 1, 12, 1.0),
                    (2300, 1000.4, 35.1000020, 139.1000020, 42.0, 1, 12, 1.0),
                    (2300, 1000.6, 35.1000030, 139.1000030, 42.0, 1, 12, 1.0),
                ],
            )

            result = self.run_gnss(
                "ppc-spp-compare",
                "--reference-csv",
                str(reference_csv),
                "--solution",
                f"baseline={baseline_path}",
                "--solution",
                f"improved={improved_path}",
                "--summary-json",
                str(summary_path),
                "--csv",
                str(csv_path),
                "--matched-csv",
                str(matched_csv),
                "--png",
                str(png_path),
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("PPC SPP comparison", result.stdout)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["baseline_label"], "baseline")
            self.assertEqual(len(payload["solutions"]), 2)
            improved = [
                row for row in payload["solutions"]
                if row["label"] == "improved"
            ][0]
            self.assertLess(improved["delta_vs_baseline"]["p95_h_m"], 0.0)
            self.assertTrue(csv_path.exists())
            self.assertTrue(matched_csv.exists())
            self.assertTrue(png_path.exists())
            self.assertGreater(png_path.stat().st_size, 0)
    def test_ppc_demo_cli_summarizes_commercial_rover_existing_solution(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_demo_commercial_rover_") as temp_dir:
            temp_root = Path(temp_dir)
            run_dir = temp_root / "tokyo" / "run1"
            run_dir.mkdir(parents=True)
            solution_path = temp_root / "ppc_demo.pos"
            commercial_solution = temp_root / "trimble_net_r9.pos"
            commercial_rover = run_dir / "rover_trimble.obs"
            commercial_base = run_dir / "base_trimble.obs"
            commercial_nav = run_dir / "base.nav"
            commercial_matches = temp_root / "trimble_net_r9_matches.csv"
            summary_path = temp_root / "ppc_demo_summary.json"
            reference_csv = run_dir / "reference.csv"

            reference_csv.write_text(
                "\n".join(
                    [
                        "gps_week,gps_tow_s,lat_deg,lon_deg,height_m,roll_deg,pitch_deg,yaw_deg",
                        "2300,1000.000,35.1000000,139.1000000,42.000,0.0,0.0,0.0",
                        "2300,1000.200,35.1000100,139.1000200,42.200,0.0,0.0,0.0",
                        "2300,1000.400,35.1000200,139.1000400,42.400,0.0,0.0,0.0",
                    ]
                )
                + "\n",
                encoding="ascii",
            )
            for path in (commercial_rover, commercial_base, commercial_nav):
                path.write_text("# synthetic rinex placeholder\n", encoding="ascii")

            with solution_path.open("w", encoding="ascii") as handle:
                handle.write("% synthetic ppc demo solution\n")
                for week, tow, lat, lon, height, status, satellites in (
                    (2300, 1000.0, 35.1000002, 139.1000001, 42.1, 4, 12),
                    (2300, 1000.2, 35.1000099, 139.1000202, 42.3, 4, 13),
                    (2300, 1000.4, 35.1000197, 139.1000398, 42.5, 3, 11),
                ):
                    ecef = driving_comparison.llh_to_ecef(lat, lon, height)
                    handle.write(
                        f"{week} {tow:.3f} {ecef[0]:.6f} {ecef[1]:.6f} {ecef[2]:.6f} "
                        f"{lat:.9f} {lon:.9f} {height:.4f} {status} {satellites} 1.0\n"
                    )
            with commercial_solution.open("w", encoding="ascii") as handle:
                handle.write("% synthetic commercial receiver observation solution\n")
                for week, tow, lat, lon, height, status, satellites in (
                    (2300, 1000.0, 35.1000001, 139.1000001, 42.1, 4, 18),
                    (2300, 1000.2, 35.1000101, 139.1000201, 42.2, 4, 19),
                    (2300, 1000.4, 35.1000201, 139.1000401, 42.4, 4, 18),
                ):
                    ecef = driving_comparison.llh_to_ecef(lat, lon, height)
                    handle.write(
                        f"{week} {tow:.3f} {ecef[0]:.6f} {ecef[1]:.6f} {ecef[2]:.6f} "
                        f"{lat:.9f} {lon:.9f} {height:.4f} {status} {satellites} 1.0\n"
                    )

            result = self.run_gnss(
                "ppc-demo",
                "--run-dir",
                str(run_dir),
                "--solver",
                "rtk",
                "--use-existing-solution",
                "--solver-wall-time-s",
                "0.5",
                "--out",
                str(solution_path),
                "--commercial-rover",
                str(commercial_rover),
                "--commercial-base",
                str(commercial_base),
                "--commercial-nav",
                str(commercial_nav),
                "--commercial-out",
                str(commercial_solution),
                "--use-existing-commercial-solution",
                "--commercial-label",
                "trimble_net_r9",
                "--commercial-matched-csv",
                str(commercial_matches),
                "--commercial-solver-wall-time-s",
                "0.3",
                "--summary-json",
                str(summary_path),
                "--require-valid-epochs-min",
                "3",
                "--require-matched-epochs-min",
                "3",
                "--require-fix-rate-min",
                "60",
                "--require-median-h-max",
                "0.2",
                "--require-p95-h-max",
                "0.2",
                "--require-max-h-max",
                "0.2",
                "--require-p95-up-max",
                "0.2",
                "--require-mean-sats-min",
                "11",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            commercial = payload["commercial_receiver"]
            self.assertEqual(commercial["label"], "trimble_net_r9")
            self.assertEqual(commercial["source"], "libgnss_solved_receiver_observations")
            self.assertEqual(commercial["solution_pos"], str(commercial_solution))
            self.assertEqual(commercial["rover"], str(commercial_rover))
            self.assertEqual(commercial["base"], str(commercial_base))
            self.assertEqual(commercial["nav"], str(commercial_nav))
            self.assertFalse(commercial["generated_solution"])
            self.assertEqual(commercial["matched_epochs"], 3)
            self.assertTrue(commercial_matches.exists())
            self.assertIn("delta_vs_commercial_receiver", payload)
            self.assertIn("commercial_receiver:", result.stdout)
    def test_ppc_rtk_signoff_cli_wraps_profile_with_existing_solutions(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_rtk_signoff_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            run_dir = temp_root / "tokyo" / "run1"
            run_dir.mkdir(parents=True)
            solution_path = temp_root / "ppc_signoff.pos"
            rtklib_path = temp_root / "ppc_signoff_rtklib.pos"
            commercial_path = temp_root / "ppc_signoff_commercial.csv"
            commercial_matches = temp_root / "ppc_signoff_commercial_matches.csv"
            summary_path = temp_root / "ppc_signoff_summary.json"
            reference_csv = run_dir / "reference.csv"

            reference_rows = [
                (2300, 1000.0, 35.1000000, 139.1000000, 42.0),
                (2300, 1000.2, 35.1000100, 139.1000200, 42.2),
                (2300, 1000.4, 35.1000200, 139.1000400, 42.4),
            ]
            reference_lines = [
                "gps_week,gps_tow_s,lat_deg,lon_deg,height_m,roll_deg,pitch_deg,yaw_deg"
            ]
            for week, tow, lat, lon, height in reference_rows:
                reference_lines.append(
                    f"{week},{tow:.3f},{lat:.7f},{lon:.7f},{height:.3f},0.0,0.0,0.0"
                )
            reference_csv.write_text("\n".join(reference_lines) + "\n", encoding="ascii")

            with solution_path.open("w", encoding="ascii") as handle:
                handle.write("% synthetic ppc signoff solution\n")
                for week, tow, lat, lon, height, status, satellites in (
                    (2300, 1000.0, 35.1000002, 139.1000001, 42.1, 4, 12),
                    (2300, 1000.2, 35.1000099, 139.1000202, 42.3, 4, 13),
                    (2300, 1000.4, 35.1000197, 139.1000398, 42.5, 3, 11),
                ):
                    ecef = driving_comparison.llh_to_ecef(lat, lon, height)
                    handle.write(
                        f"{week} {tow:.3f} {ecef[0]:.6f} {ecef[1]:.6f} {ecef[2]:.6f} "
                        f"{lat:.9f} {lon:.9f} {height:.4f} {status} {satellites} 1.0\n"
                    )
            write_rtklib_pos(
                rtklib_path,
                [
                    (2300, 1000.0, 35.1000004, 139.1000003, 42.2, 1, 12),
                    (2300, 1000.2, 35.1000103, 139.1000205, 42.4, 1, 13),
                    (2300, 1000.4, 35.1000205, 139.1000404, 42.6, 2, 11),
                ],
            )
            commercial_path.write_text(
                "\n".join(
                    [
                        "gps_week,gps_tow_s,lat_deg,lon_deg,height_m,solution_status,num_satellites",
                        "2300,1000.0,35.1000001,139.1000001,42.1,rtk_fixed,14",
                        "2300,1000.2,35.1000101,139.1000201,42.2,rtk_fixed,14",
                        "2300,1000.4,35.1000201,139.1000401,42.4,rtk_float,13",
                    ]
                )
                + "\n",
                encoding="ascii",
            )

            result = self.run_gnss(
                "ppc-rtk-signoff",
                "--run-dir",
                str(run_dir),
                "--city",
                "tokyo",
                "--use-existing-solution",
                "--solver-wall-time-s",
                "0.5",
                "--out",
                str(solution_path),
                "--rtklib-pos",
                str(rtklib_path),
                "--use-existing-rtklib-solution",
                "--rtklib-solver-wall-time-s",
                "0.1",
                "--commercial-pos",
                str(commercial_path),
                "--commercial-matched-csv",
                str(commercial_matches),
                "--commercial-solver-wall-time-s",
                "0.2",
                "--summary-json",
                str(summary_path),
                "--require-valid-epochs-min",
                "3",
                "--require-matched-epochs-min",
                "3",
                "--require-fix-rate-min",
                "60",
                "--require-median-h-max",
                "0.2",
                "--require-p95-h-max",
                "0.2",
                "--require-max-h-max",
                "0.2",
                "--require-p95-up-max",
                "0.2",
                "--require-mean-sats-min",
                "11",
                "--require-solver-wall-time-max",
                "1.0",
                "--require-realtime-factor-min",
                "0.5",
                "--require-effective-epoch-rate-min",
                "5.0",
                "--require-lib-fix-rate-vs-rtklib-min-delta",
                "0.0",
                "--require-lib-median-h-vs-rtklib-max-delta",
                "0.0",
                "--require-lib-p95-h-vs-rtklib-max-delta",
                "0.0",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("Finished PPC RTK sign-off.", result.stdout)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["signoff_profile"], "ppc-rtk-tokyo")
            self.assertTrue(payload["rtklib_comparison_enabled"])
            self.assertIn("signoff_thresholds", payload)
            self.assertEqual(payload["signoff_thresholds"]["require_fix_rate_min"], 60.0)
            self.assertIn("delta_vs_rtklib", payload)
            self.assertTrue(payload["commercial_receiver_comparison_enabled"])
            self.assertIn("commercial_receiver", payload)
            self.assertTrue(commercial_matches.exists())
    def test_ppc_rtk_signoff_cli_reads_config_toml(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_rtk_signoff_toml_") as temp_dir:
            temp_root = Path(temp_dir)
            run_dir = temp_root / "tokyo" / "run1"
            run_dir.mkdir(parents=True)
            solution_path = temp_root / "ppc_signoff.pos"
            rtklib_path = temp_root / "ppc_signoff_rtklib.pos"
            summary_path = temp_root / "ppc_signoff_summary.json"
            config_toml = temp_root / "ppc_signoff.toml"
            reference_csv = run_dir / "reference.csv"

            reference_rows = [
                (2300, 1000.0, 35.1000000, 139.1000000, 42.0),
                (2300, 1000.2, 35.1000100, 139.1000200, 42.2),
                (2300, 1000.4, 35.1000200, 139.1000400, 42.4),
            ]
            reference_lines = [
                "gps_week,gps_tow_s,lat_deg,lon_deg,height_m,roll_deg,pitch_deg,yaw_deg"
            ]
            for week, tow, lat, lon, height in reference_rows:
                reference_lines.append(
                    f"{week},{tow:.3f},{lat:.7f},{lon:.7f},{height:.3f},0.0,0.0,0.0"
                )
            reference_csv.write_text("\n".join(reference_lines) + "\n", encoding="ascii")

            with solution_path.open("w", encoding="ascii") as handle:
                handle.write("% synthetic ppc signoff solution\n")
                for week, tow, lat, lon, height, status, satellites in (
                    (2300, 1000.0, 35.1000002, 139.1000001, 42.1, 4, 12),
                    (2300, 1000.2, 35.1000099, 139.1000202, 42.3, 4, 13),
                    (2300, 1000.4, 35.1000197, 139.1000398, 42.5, 3, 11),
                ):
                    ecef = driving_comparison.llh_to_ecef(lat, lon, height)
                    handle.write(
                        f"{week} {tow:.3f} {ecef[0]:.6f} {ecef[1]:.6f} {ecef[2]:.6f} "
                        f"{lat:.9f} {lon:.9f} {height:.4f} {status} {satellites} 1.0\n"
                    )
            write_rtklib_pos(
                rtklib_path,
                [
                    (2300, 1000.0, 35.1000004, 139.1000003, 42.2, 1, 12),
                    (2300, 1000.2, 35.1000103, 139.1000205, 42.4, 1, 13),
                    (2300, 1000.4, 35.1000205, 139.1000404, 42.6, 2, 11),
                ],
            )
            config_toml.write_text(
                "\n".join(
                    [
                        "[ppc_rtk_signoff]",
                        f'run_dir = "{run_dir}"',
                        'city = "tokyo"',
                        f'out = "{solution_path}"',
                        f'summary_json = "{summary_path}"',
                        f'rtklib_pos = "{rtklib_path}"',
                        "use_existing_solution = true",
                        "use_existing_rtklib_solution = true",
                        "solver_wall_time_s = 0.5",
                        "rtklib_solver_wall_time_s = 0.1",
                        "require_valid_epochs_min = 3",
                        "require_matched_epochs_min = 3",
                        "require_fix_rate_min = 60.0",
                        "require_median_h_max = 0.2",
                        "require_p95_h_max = 0.2",
                        "require_max_h_max = 0.2",
                        "require_p95_up_max = 0.2",
                        "require_mean_sats_min = 11.0",
                        "require_solver_wall_time_max = 1.0",
                        "require_realtime_factor_min = 0.5",
                        "require_effective_epoch_rate_min = 5.0",
                        "require_lib_fix_rate_vs_rtklib_min_delta = 0.0",
                        "require_lib_median_h_vs_rtklib_max_delta = 0.0",
                        "require_lib_p95_h_vs_rtklib_max_delta = 0.0",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            result = self.run_gnss(
                "ppc-rtk-signoff",
                "--config-toml",
                str(config_toml),
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["signoff_profile"], "ppc-rtk-tokyo")
            self.assertTrue(payload["rtklib_comparison_enabled"])
            self.assertEqual(payload["signoff_thresholds"]["require_fix_rate_min"], 60.0)
    def test_ppc_demo_cli_tokyo_rtk_realdataset_signoff_if_present(self) -> None:
        dataset_root = ppc_dataset_root()
        run_dir = dataset_root / "tokyo" / "run1"
        required_files = (
            run_dir / "rover.obs",
            run_dir / "base.obs",
            run_dir / "base.nav",
            run_dir / "reference.csv",
        )
        if not all(path.exists() for path in required_files):
            self.skipTest("PPC-Dataset tokyo/run1 is not available")

        with tempfile.TemporaryDirectory(prefix="gnss_ppc_tokyo_rtk_") as temp_dir:
            temp_root = Path(temp_dir)
            solution_path = temp_root / "tokyo_run1_rtk.pos"
            summary_path = temp_root / "tokyo_run1_rtk_summary.json"

            result = self.run_gnss(
                "ppc-demo",
                "--run-dir",
                str(run_dir),
                "--solver",
                "rtk",
                "--max-epochs",
                "120",
                "--out",
                str(solution_path),
                "--summary-json",
                str(summary_path),
                "--require-valid-epochs-min",
                "100",
                "--require-matched-epochs-min",
                "100",
                "--require-fix-rate-min",
                "90",
                "--require-median-h-max",
                "0.10",
                "--require-p95-h-max",
                "0.20",
                "--require-max-h-max",
                "0.50",
                "--require-p95-up-max",
                "0.60",
                "--require-mean-sats-min",
                "18",
                "--require-solver-wall-time-max",
                "10.0",
                "--require-realtime-factor-min",
                "1.0",
                "--require-effective-epoch-rate-min",
                "5.0",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("Finished PPC-Dataset demo.", result.stdout)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["dataset"], "PPC-Dataset tokyo run1")
            self.assertEqual(payload["solver"], "rtk")
            self.assertGreaterEqual(payload["valid_epochs"], 100)
            self.assertGreaterEqual(payload["matched_epochs"], 100)
            self.assertGreaterEqual(payload["fix_rate_pct"], 90.0)
            self.assertLessEqual(payload["median_h_m"], 0.10)
            self.assertLessEqual(payload["p95_h_m"], 0.20)
            self.assertLessEqual(payload["max_h_m"], 0.50)
            self.assertLessEqual(payload["p95_abs_up_m"], 0.60)
            self.assertGreaterEqual(payload["mean_satellites"], 18.0)
            self.assertLessEqual(payload["solver_wall_time_s"], 10.0)
            self.assertGreaterEqual(payload["realtime_factor"], 1.0)
            self.assertGreaterEqual(payload["effective_epoch_rate_hz"], 5.0)
    def test_ppc_demo_cli_tokyo_rtk_realdataset_tracks_rtklib_if_present(self) -> None:
        dataset_root = ppc_dataset_root()
        run_dir = dataset_root / "tokyo" / "run1"
        rtklib_bin = rtklib_binary()
        required_files = (
            run_dir / "rover.obs",
            run_dir / "base.obs",
            run_dir / "base.nav",
            run_dir / "reference.csv",
        )
        if not all(path.exists() for path in required_files):
            self.skipTest("PPC-Dataset tokyo/run1 is not available")
        if not rtklib_bin.exists():
            self.skipTest("RTKLIB rnx2rtkp is not available")

        with tempfile.TemporaryDirectory(prefix="gnss_ppc_tokyo_rtk_rtklib_") as temp_dir:
            temp_root = Path(temp_dir)
            solution_path = temp_root / "tokyo_run1_rtk.pos"
            summary_path = temp_root / "tokyo_run1_rtk_summary.json"

            result = self.run_gnss(
                "ppc-demo",
                "--run-dir",
                str(run_dir),
                "--solver",
                "rtk",
                "--max-epochs",
                "120",
                "--out",
                str(solution_path),
                "--summary-json",
                str(summary_path),
                "--rtklib-bin",
                str(rtklib_bin),
                "--require-valid-epochs-min",
                "100",
                "--require-matched-epochs-min",
                "100",
                "--require-realtime-factor-min",
                "1.0",
                "--require-lib-fix-rate-vs-rtklib-min-delta",
                "0.0",
                "--require-lib-median-h-vs-rtklib-max-delta",
                "0.01",
                "--require-lib-p95-h-vs-rtklib-max-delta",
                "0.02",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertIn("rtklib", payload)
            self.assertIn("delta_vs_rtklib", payload)
            self.assertGreaterEqual(payload["realtime_factor"], 1.0)
            self.assertGreaterEqual(
                payload["delta_vs_rtklib"]["fix_rate_pct"],
                0.0,
            )
            self.assertLessEqual(
                payload["delta_vs_rtklib"]["median_h_m"],
                0.01,
            )
            self.assertLessEqual(
                payload["delta_vs_rtklib"]["p95_h_m"],
                0.02,
            )
    def test_ppc_rtk_signoff_cli_tokyo_realdataset_if_present(self) -> None:
        dataset_root = ppc_dataset_root()
        run_dir = dataset_root / "tokyo" / "run1"
        rtklib_bin = rtklib_binary()
        required_files = (
            run_dir / "rover.obs",
            run_dir / "base.obs",
            run_dir / "base.nav",
            run_dir / "reference.csv",
        )
        if not all(path.exists() for path in required_files):
            self.skipTest("PPC-Dataset tokyo/run1 is not available")
        if not rtklib_bin.exists():
            self.skipTest("RTKLIB rnx2rtkp is not available")

        with tempfile.TemporaryDirectory(prefix="gnss_ppc_tokyo_signoff_") as temp_dir:
            temp_root = Path(temp_dir)
            summary_path = temp_root / "tokyo_run1_signoff_summary.json"
            result = self.run_gnss(
                "ppc-rtk-signoff",
                "--dataset-root",
                str(dataset_root),
                "--city",
                "tokyo",
                "--max-epochs",
                "120",
                "--summary-json",
                str(summary_path),
                "--rtklib-bin",
                str(rtklib_bin),
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["signoff_profile"], "ppc-rtk-tokyo")
            self.assertTrue(payload["rtklib_comparison_enabled"])
            self.assertEqual(payload["tuning_profile"]["preset"], "low-cost")
            self.assertTrue(payload["tuning_profile"]["arfilter"])
            self.assertGreaterEqual(payload["fix_rate_pct"], 95.0)
            self.assertGreaterEqual(payload["delta_vs_rtklib"]["fix_rate_pct"], 0.0)
            self.assertLessEqual(payload["delta_vs_rtklib"]["median_h_m"], 0.01)
            self.assertLessEqual(payload["delta_vs_rtklib"]["p95_h_m"], 0.02)
    def test_ppc_rtk_signoff_cli_nagoya_realdataset_if_present(self) -> None:
        dataset_root = ppc_dataset_root()
        run_dir = dataset_root / "nagoya" / "run1"
        required_files = (
            run_dir / "rover.obs",
            run_dir / "base.obs",
            run_dir / "base.nav",
            run_dir / "reference.csv",
        )
        if not all(path.exists() for path in required_files):
            self.skipTest("PPC-Dataset nagoya/run1 is not available")

        with tempfile.TemporaryDirectory(prefix="gnss_ppc_nagoya_signoff_") as temp_dir:
            temp_root = Path(temp_dir)
            summary_path = temp_root / "nagoya_run1_signoff_summary.json"
            result = self.run_gnss(
                "ppc-rtk-signoff",
                "--dataset-root",
                str(dataset_root),
                "--city",
                "nagoya",
                "--max-epochs",
                "120",
                "--summary-json",
                str(summary_path),
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["signoff_profile"], "ppc-rtk-nagoya")
            self.assertFalse(payload["rtklib_comparison_enabled"])
            self.assertEqual(payload["tuning_profile"]["preset"], "low-cost")
            self.assertFalse(payload["tuning_profile"]["arfilter"])
            self.assertGreaterEqual(payload["fix_rate_pct"], 95.0)
            self.assertLessEqual(payload["median_h_m"], 0.12)
            self.assertLessEqual(payload["p95_h_m"], 0.12)
