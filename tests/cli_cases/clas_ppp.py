"""CLI regression cases for the CLASPPPCases domain."""

from ._support import *  # noqa: F401,F403

class CLASPPPCases:
    def test_clas_ppp_cli_writes_summary_for_named_profile(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_ppp_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            ssr_path = temp_root / "corrections.rtcm3"
            output_path = temp_root / "clas_ppp.pos"
            summary_path = temp_root / "clas_ppp_summary.json"
            ssr_path.write_bytes(
                build_rtcm1060(3, 518400) +
                build_rtcm1059(3, 518400) +
                build_rtcm1062(3, 518400)
            )

            result = self.run_gnss(
                "clas-ppp",
                "--profile",
                "madoca",
                "--static",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--ssr-rtcm",
                str(ssr_path),
                "--out",
                str(output_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "3",
                "--require-valid-epochs-min",
                "3",
                "--require-ppp-solution-rate-min",
                "100",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertTrue(output_path.exists())
            self.assertTrue(summary_path.exists())
            self.assertIn("Finished CLAS/MADOCA PPP run.", result.stdout)
            self.assertIn("profile: madoca", result.stdout)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["dataset"], "CLAS/MADOCA PPP")
            self.assertEqual(payload["correction_profile"], "madoca")
            self.assertEqual(payload["ssr_transport"], "file")
            self.assertEqual(payload["epochs"], 3)
            self.assertEqual(payload["ppp_solution_rate_pct"], 100.0)
            self.assertFalse(payload["clas_osr_filter_enabled"])
            self.assertNotIn("--clas-osr", result.stdout)
    def test_clas_ppp_cli_accepts_compact_sampled_corrections(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_ppp_compact_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            compact_path = temp_root / "corrections.compact.csv"
            output_path = temp_root / "clas_ppp_compact.pos"
            summary_path = temp_root / "clas_ppp_compact_summary.json"
            compact_path.write_text(
                "\n".join(
                    [
                        "# week,tow,system,prn,dx,dy,dz,dclock_m,high_rate_clock_m",
                        "1316,518400.0,G,3,0.0,0.0,0.0,0.0,0.025",
                        "1316,518430.0,G,3,0.0,0.0,0.0,0.0,0.025",
                        "1316,518460.0,G,3,0.0,0.0,0.0,0.0,0.025",
                    ]
                )
                + "\n",
                encoding="ascii",
            )

            result = self.run_gnss(
                "clas-ppp",
                "--profile",
                "clas",
                "--static",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--compact-ssr",
                str(compact_path),
                "--out",
                str(output_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "3",
                "--require-valid-epochs-min",
                "3",
                "--require-ppp-solution-rate-min",
                "100",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertTrue(output_path.exists())
            self.assertTrue(summary_path.exists())
            self.assertIn("Finished CLAS/MADOCA PPP run.", result.stdout)
            self.assertIn("encoding: compact", result.stdout)
            self.assertIn("expanded compact corrections:", result.stdout)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["dataset"], "CLAS/MADOCA PPP")
            self.assertEqual(payload["correction_profile"], "clas")
            self.assertEqual(payload["ssr_transport"], "file")
            self.assertEqual(payload["correction_encoding"], "compact")
            self.assertEqual(payload["epochs"], 3)
            self.assertEqual(payload["ppp_solution_rate_pct"], 100.0)
            self.assertTrue(payload["clas_osr_filter_enabled"])
            self.assertIn("--clas-osr", result.stdout)
    def test_clas_ppp_cli_accepts_direct_qzss_l6_corrections(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_ppp_qzss_l6_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            l6_path = temp_root / "corrections_l6.bin"
            output_path = temp_root / "clas_ppp_l6.pos"
            summary_path = temp_root / "clas_ppp_l6_summary.json"
            l6_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_combined_message(tow_delta=0, iod=3, prn=3, dclock_m=0.025),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518430, iod=3, prn=3, sync=True),
                        build_qzss_cssr_combined_message(tow_delta=30, iod=3, prn=3, dclock_m=0.025),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518460, iod=3, prn=3, sync=True),
                        build_qzss_cssr_combined_message(tow_delta=60, iod=3, prn=3, dclock_m=0.025),
                    ]
                )
            )

            result = self.run_gnss(
                "clas-ppp",
                "--profile",
                "clas",
                "--static",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--qzss-l6",
                str(l6_path),
                "--qzss-gps-week",
                "1316",
                "--compact-phase-bias-merge-policy",
                "message-reset",
                "--compact-phase-bias-source-policy",
                "subtype5-priority",
                "--out",
                str(output_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "3",
                "--require-valid-epochs-min",
                "3",
                "--require-ppp-solution-rate-min",
                "100",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertTrue(output_path.exists())
            self.assertTrue(summary_path.exists())
            self.assertIn("Finished CLAS/MADOCA PPP run.", result.stdout)
            self.assertIn("encoding: qzss_l6", result.stdout)
            self.assertIn("decoded qzss l6 corrections:", result.stdout)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["dataset"], "CLAS/MADOCA PPP")
            self.assertEqual(payload["correction_profile"], "clas")
            self.assertEqual(payload["ssr_transport"], "file")
            self.assertEqual(payload["correction_encoding"], "qzss_l6")
            self.assertEqual(payload["epochs"], 3)
            self.assertEqual(payload["ppp_solution_rate_pct"], 100.0)
            self.assertTrue(payload["clas_osr_filter_enabled"])
            self.assertIn("--clas-osr", result.stdout)
    def test_clas_ppp_cli_accepts_direct_qzss_l6_orbit_clock_corrections(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_ppp_qzss_l6_oc_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            l6_path = temp_root / "corrections_l6_oc.bin"
            output_path = temp_root / "clas_ppp_l6_oc.pos"
            summary_path = temp_root / "clas_ppp_l6_oc_summary.json"
            l6_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_orbit_message(tow_delta=0, iod=3, dx=0.0, dy=0.0, dz=0.0, sync=True),
                        build_qzss_cssr_clock_message(tow_delta=0, iod=3, dclock_m=0.025, sync=False),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518430, iod=3, prn=3, sync=True),
                        build_qzss_cssr_orbit_message(tow_delta=30, iod=3, dx=0.0, dy=0.0, dz=0.0, sync=True),
                        build_qzss_cssr_clock_message(tow_delta=30, iod=3, dclock_m=0.025, sync=False),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518460, iod=3, prn=3, sync=True),
                        build_qzss_cssr_orbit_message(tow_delta=60, iod=3, dx=0.0, dy=0.0, dz=0.0, sync=True),
                        build_qzss_cssr_clock_message(tow_delta=60, iod=3, dclock_m=0.025, sync=False),
                    ]
                )
            )

            result = self.run_gnss(
                "clas-ppp",
                "--profile",
                "clas",
                "--static",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--qzss-l6",
                str(l6_path),
                "--qzss-gps-week",
                "1316",
                "--compact-phase-bias-composition-policy",
                "base-plus-network",
                "--compact-phase-bias-bank-policy",
                "same-30s-bank",
                "--out",
                str(output_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "3",
                "--require-valid-epochs-min",
                "3",
                "--require-ppp-solution-rate-min",
                "100",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertTrue(output_path.exists())
            self.assertTrue(summary_path.exists())
            self.assertIn("decoded qzss l6 corrections:", result.stdout)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["correction_encoding"], "qzss_l6")
            self.assertEqual(payload["compact_phase_bias_composition_policy"], "base-plus-network")
            self.assertEqual(payload["compact_phase_bias_bank_policy"], "same-30s-bank")
            self.assertEqual(payload["ppp_solution_rate_pct"], 100.0)
    def test_clas_ppp_cli_accepts_direct_qzss_l6_code_bias_and_ura(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_ppp_qzss_l6_cbias_ura_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            l6_path = temp_root / "corrections_l6_cbias_ura.bin"
            output_path = temp_root / "clas_ppp_l6_cbias_ura.pos"
            summary_path = temp_root / "clas_ppp_l6_cbias_ura_summary.json"
            l6_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_code_bias_message(tow_delta=0, iod=3, bias_m=-0.12, sync=True),
                        build_qzss_cssr_ura_message(tow_delta=0, iod=3, ura_index=9, sync=True),
                        build_qzss_cssr_combined_message(tow_delta=0, iod=3, prn=3, dclock_m=0.025, sync=False),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518430, iod=3, prn=3, sync=True),
                        build_qzss_cssr_code_bias_message(tow_delta=30, iod=3, bias_m=-0.12, sync=True),
                        build_qzss_cssr_ura_message(tow_delta=30, iod=3, ura_index=9, sync=True),
                        build_qzss_cssr_combined_message(tow_delta=30, iod=3, prn=3, dclock_m=0.025, sync=False),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518460, iod=3, prn=3, sync=True),
                        build_qzss_cssr_code_bias_message(tow_delta=60, iod=3, bias_m=-0.12, sync=True),
                        build_qzss_cssr_ura_message(tow_delta=60, iod=3, ura_index=9, sync=True),
                        build_qzss_cssr_combined_message(tow_delta=60, iod=3, prn=3, dclock_m=0.025, sync=False),
                    ]
                )
            )

            result = self.run_gnss(
                "clas-ppp",
                "--profile",
                "clas",
                "--static",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--qzss-l6",
                str(l6_path),
                "--qzss-gps-week",
                "1316",
                "--compact-phase-bias-merge-policy",
                "message-reset",
                "--compact-phase-bias-source-policy",
                "subtype5-priority",
                "--out",
                str(output_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "3",
                "--require-valid-epochs-min",
                "3",
                "--require-ppp-solution-rate-min",
                "100",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertTrue(output_path.exists())
            self.assertTrue(summary_path.exists())
            self.assertIn("decoded qzss l6 corrections:", result.stdout)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["correction_encoding"], "qzss_l6")
            self.assertEqual(payload["compact_phase_bias_merge_policy"], "message-reset")
            self.assertEqual(payload["compact_phase_bias_source_policy"], "subtype5-priority")
            self.assertEqual(payload["ppp_solution_rate_pct"], 100.0)
    def test_clas_ppp_cli_accepts_direct_qzss_l6_with_atmos_inventory(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_ppp_qzss_l6_atmos_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            l6_path = temp_root / "corrections_l6_atmos.bin"
            output_path = temp_root / "clas_ppp_l6_atmos.pos"
            summary_path = temp_root / "clas_ppp_l6_atmos_summary.json"
            l6_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_atmos_message(tow_delta=0, iod=3, sync=True),
                        build_qzss_cssr_combined_message(tow_delta=0, iod=3, prn=3, dclock_m=0.025, sync=False),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518430, iod=3, prn=3, sync=True),
                        build_qzss_cssr_atmos_message(tow_delta=30, iod=3, sync=True),
                        build_qzss_cssr_combined_message(tow_delta=30, iod=3, prn=3, dclock_m=0.025, sync=False),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518460, iod=3, prn=3, sync=True),
                        build_qzss_cssr_atmos_message(tow_delta=60, iod=3, sync=True),
                        build_qzss_cssr_combined_message(tow_delta=60, iod=3, prn=3, dclock_m=0.025, sync=False),
                    ]
                )
            )

            result = self.run_gnss(
                "clas-ppp",
                "--profile",
                "clas",
                "--static",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--qzss-l6",
                str(l6_path),
                "--qzss-gps-week",
                "1316",
                "--out",
                str(output_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "3",
                "--require-valid-epochs-min",
                "3",
                "--require-ppp-solution-rate-min",
                "100",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertTrue(output_path.exists())
            self.assertTrue(summary_path.exists())
            self.assertIn("decoded qzss l6 corrections:", result.stdout)
            self.assertIn("atmos_messages=3", result.stdout)
            self.assertIn("atmos_rows=3", result.stdout)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["correction_encoding"], "qzss_l6")
            self.assertEqual(payload["ppp_solution_rate_pct"], 100.0)
            self.assertEqual(payload["atmos_messages"], 3)
            self.assertEqual(payload["atmos_rows"], 3)
    def test_clas_ppp_cli_accepts_direct_qzss_l6_with_stec_inventory(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_ppp_qzss_l6_stec_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            l6_path = temp_root / "corrections_l6_stec.bin"
            output_path = temp_root / "clas_ppp_l6_stec.pos"
            summary_path = temp_root / "clas_ppp_l6_stec_summary.json"
            l6_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_stec_message(
                            tow_delta=0,
                            iod=3,
                            sync=True,
                            network_id=7,
                            stec_type=3,
                            selected_satellites=1,
                            stec_quality=17,
                            stec_c00_tecu=1.5,
                            stec_c01_tecu_per_deg=0.12,
                            stec_c10_tecu_per_deg=-0.10,
                            stec_c11_tecu_per_deg2=0.06,
                            stec_c02_tecu_per_deg2=0.025,
                            stec_c20_tecu_per_deg2=-0.015,
                        ),
                        build_qzss_cssr_combined_message(
                            tow_delta=0, iod=3, prn=3, dclock_m=0.025, sync=False
                        ),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518430, iod=3, prn=3, sync=True),
                        build_qzss_cssr_stec_message(
                            tow_delta=30,
                            iod=3,
                            sync=True,
                            network_id=7,
                            stec_type=3,
                            selected_satellites=1,
                            stec_quality=17,
                            stec_c00_tecu=1.5,
                            stec_c01_tecu_per_deg=0.12,
                            stec_c10_tecu_per_deg=-0.10,
                            stec_c11_tecu_per_deg2=0.06,
                            stec_c02_tecu_per_deg2=0.025,
                            stec_c20_tecu_per_deg2=-0.015,
                        ),
                        build_qzss_cssr_combined_message(
                            tow_delta=30, iod=3, prn=3, dclock_m=0.025, sync=False
                        ),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518460, iod=3, prn=3, sync=True),
                        build_qzss_cssr_stec_message(
                            tow_delta=60,
                            iod=3,
                            sync=True,
                            network_id=7,
                            stec_type=3,
                            selected_satellites=1,
                            stec_quality=17,
                            stec_c00_tecu=1.5,
                            stec_c01_tecu_per_deg=0.12,
                            stec_c10_tecu_per_deg=-0.10,
                            stec_c11_tecu_per_deg2=0.06,
                            stec_c02_tecu_per_deg2=0.025,
                            stec_c20_tecu_per_deg2=-0.015,
                        ),
                        build_qzss_cssr_combined_message(
                            tow_delta=60, iod=3, prn=3, dclock_m=0.025, sync=False
                        ),
                    ]
                )
            )

            result = self.run_gnss(
                "clas-ppp",
                "--profile",
                "clas",
                "--static",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--qzss-l6",
                str(l6_path),
                "--qzss-gps-week",
                "1316",
                "--out",
                str(output_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "3",
                "--require-valid-epochs-min",
                "3",
                "--require-ppp-solution-rate-min",
                "100",
                "--require-atmos-messages-min",
                "3",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertTrue(output_path.exists())
            self.assertTrue(summary_path.exists())
            self.assertIn("decoded qzss l6 corrections:", result.stdout)
            self.assertIn("atmos_messages=3", result.stdout)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["correction_encoding"], "qzss_l6")
            self.assertEqual(payload["ppp_solution_rate_pct"], 100.0)
            self.assertEqual(payload["atmos_messages"], 3)
    def test_clas_ppp_cli_reports_applied_atmospheric_corrections_for_direct_qzss_l6(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_ppp_qzss_l6_atmos_apply_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            obs_path, sp3_path, clk_path, _, _ = build_synthetic_ppp_inputs_with_atmos(temp_root)
            l6_path = temp_root / "corrections_l6_atmos_apply.bin"
            output_path = temp_root / "clas_ppp_l6_atmos_apply.pos"
            summary_path = temp_root / "clas_ppp_l6_atmos_apply_summary.json"
            l6_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=356400, iod=3, prn=1, sync=True),
                        build_qzss_cssr_atmos_message(
                            tow_delta=0,
                            iod=3,
                            sync=True,
                            selected_satellites=1,
                            trop_avail=3,
                            stec_avail=3,
                            trop_quality=9,
                            trop_type=0,
                            trop_t00_m=0.8,
                            stec_quality=17,
                            stec_type=0,
                            stec_c00_tecu=12.5,
                        ),
                        build_qzss_cssr_combined_message(tow_delta=0, iod=3, prn=1, dclock_m=0.0, sync=False),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=356430, iod=3, prn=1, sync=True),
                        build_qzss_cssr_atmos_message(
                            tow_delta=30,
                            iod=3,
                            sync=True,
                            selected_satellites=1,
                            trop_avail=3,
                            stec_avail=3,
                            trop_quality=9,
                            trop_type=0,
                            trop_t00_m=0.8,
                            stec_quality=17,
                            stec_type=0,
                            stec_c00_tecu=12.5,
                        ),
                        build_qzss_cssr_combined_message(tow_delta=30, iod=3, prn=1, dclock_m=0.0, sync=False),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=356460, iod=3, prn=1, sync=True),
                        build_qzss_cssr_atmos_message(
                            tow_delta=60,
                            iod=3,
                            sync=True,
                            selected_satellites=1,
                            trop_avail=3,
                            stec_avail=3,
                            trop_quality=9,
                            trop_type=0,
                            trop_t00_m=0.8,
                            stec_quality=17,
                            stec_type=0,
                            stec_c00_tecu=12.5,
                        ),
                        build_qzss_cssr_combined_message(tow_delta=60, iod=3, prn=1, dclock_m=0.0, sync=False),
                    ]
                )
            )

            result = self.run_gnss(
                "clas-ppp",
                "--profile",
                "clas",
                "--kinematic",
                "--obs",
                str(obs_path),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--qzss-l6",
                str(l6_path),
                "--qzss-gps-week",
                "2411",
                "--out",
                str(output_path),
                "--summary-json",
                str(summary_path),
                "--no-estimate-troposphere",
                "--max-epochs",
                "3",
                "--require-valid-epochs-min",
                "3",
                "--require-ppp-solution-rate-min",
                "100",
                "--require-atmos-messages-min",
                "3",
                "--require-ppp-atmos-trop-corrections-min",
                "1",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertTrue(output_path.exists())
            self.assertTrue(summary_path.exists())
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["correction_encoding"], "qzss_l6")
            self.assertEqual(payload["atmos_messages"], 3)
            self.assertGreaterEqual(payload["ppp_atmospheric_trop_corrections"], 1)
            self.assertGreaterEqual(payload["ppp_atmospheric_ionosphere_corrections"], 1)
            self.assertGreater(payload["ppp_atmospheric_trop_meters"], 0.0)
    def test_clas_ppp_cli_uses_nearest_clas_grid_residuals_for_direct_qzss_l6(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_ppp_qzss_l6_grid_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            obs_path, sp3_path, clk_path, _, _ = build_synthetic_ppp_inputs_with_atmos(temp_root)
            l6_path = temp_root / "corrections_l6_grid.bin"
            output_path = temp_root / "clas_ppp_l6_grid.pos"
            summary_path = temp_root / "clas_ppp_l6_grid_summary.json"
            # Synthetic PPP input is centered near Tokyo (35.0N, 139.0E), so CLAS network 7
            # grid 11 (34.77N, 139.37E) is the nearest reference grid in the official grid table.
            network_id = 7
            grid_count = 22
            nearest_grid_index = 10  # network 7 grid no. 11 -> zero-based index
            trop_residuals = build_single_grid_residuals(grid_count, nearest_grid_index, 0.5)
            stec_residuals = build_single_grid_residuals(grid_count, nearest_grid_index, 12.48)
            l6_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=356400, iod=3, prn=1, sync=True),
                        build_qzss_cssr_atmos_message(
                            tow_delta=0,
                            iod=3,
                            sync=True,
                            network_id=network_id,
                            grid_count=grid_count,
                            selected_satellites=1,
                            trop_avail=2,
                            stec_avail=2,
                            trop_residual_size=1,
                            trop_offset_m=0.3,
                            trop_residuals_m=trop_residuals,
                            stec_residual_size=3,
                            stec_residuals_tecu=stec_residuals,
                        ),
                        build_qzss_cssr_combined_message(
                            tow_delta=0,
                            iod=3,
                            prn=1,
                            network_id=network_id,
                            dclock_m=0.0,
                            sync=False,
                        ),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=356430, iod=3, prn=1, sync=True),
                        build_qzss_cssr_atmos_message(
                            tow_delta=30,
                            iod=3,
                            sync=True,
                            network_id=network_id,
                            grid_count=grid_count,
                            selected_satellites=1,
                            trop_avail=2,
                            stec_avail=2,
                            trop_residual_size=1,
                            trop_offset_m=0.3,
                            trop_residuals_m=trop_residuals,
                            stec_residual_size=3,
                            stec_residuals_tecu=stec_residuals,
                        ),
                        build_qzss_cssr_combined_message(
                            tow_delta=30,
                            iod=3,
                            prn=1,
                            network_id=network_id,
                            dclock_m=0.0,
                            sync=False,
                        ),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=356460, iod=3, prn=1, sync=True),
                        build_qzss_cssr_atmos_message(
                            tow_delta=60,
                            iod=3,
                            sync=True,
                            network_id=network_id,
                            grid_count=grid_count,
                            selected_satellites=1,
                            trop_avail=2,
                            stec_avail=2,
                            trop_residual_size=1,
                            trop_offset_m=0.3,
                            trop_residuals_m=trop_residuals,
                            stec_residual_size=3,
                            stec_residuals_tecu=stec_residuals,
                        ),
                        build_qzss_cssr_combined_message(
                            tow_delta=60,
                            iod=3,
                            prn=1,
                            network_id=network_id,
                            dclock_m=0.0,
                            sync=False,
                        ),
                    ]
                )
            )

            result = self.run_gnss(
                "clas-ppp",
                "--profile",
                "clas",
                "--kinematic",
                "--obs",
                str(obs_path),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--qzss-l6",
                str(l6_path),
                "--qzss-gps-week",
                "2411",
                "--out",
                str(output_path),
                "--summary-json",
                str(summary_path),
                "--no-estimate-troposphere",
                "--max-epochs",
                "3",
                "--require-valid-epochs-min",
                "3",
                "--require-ppp-solution-rate-min",
                "100",
                "--require-atmos-messages-min",
                "3",
                "--require-ppp-atmos-trop-corrections-min",
                "1",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["correction_encoding"], "qzss_l6")
            self.assertEqual(payload["atmos_messages"], 3)
            self.assertGreater(payload["ppp_atmospheric_trop_meters"], 1.5)
    def test_clas_ppp_cli_accepts_direct_qzss_l6_gridded_corrections(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_ppp_qzss_l6_gridded_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            obs_path, sp3_path, clk_path, _, _ = build_synthetic_ppp_inputs_with_atmos(temp_root)
            l6_path = temp_root / "corrections_l6_gridded.bin"
            output_path = temp_root / "clas_ppp_l6_gridded.pos"
            summary_path = temp_root / "clas_ppp_l6_gridded_summary.json"
            network_id = 7
            grid_count = 22
            nearest_grid_index = 10
            trop_hs_residuals = build_single_grid_residuals(grid_count, nearest_grid_index, 0.24)
            trop_wet_residuals = build_single_grid_residuals(grid_count, nearest_grid_index, -0.06)
            stec_residuals = build_single_grid_residuals(grid_count, nearest_grid_index, 12.48)
            l6_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=356400, iod=3, prn=1, sync=True),
                        build_qzss_cssr_gridded_message(
                            tow_delta=0,
                            iod=3,
                            sync=True,
                            network_id=network_id,
                            trop_type=1,
                            trop_quality=9,
                            grid_count=grid_count,
                            selected_satellites=1,
                            trop_hs_residuals_m=trop_hs_residuals,
                            trop_wet_residuals_m=trop_wet_residuals,
                            stec_residuals_tecu=stec_residuals,
                        ),
                        build_qzss_cssr_combined_message(
                            tow_delta=0,
                            iod=3,
                            prn=1,
                            network_id=network_id,
                            dclock_m=0.0,
                            sync=False,
                        ),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=356430, iod=3, prn=1, sync=True),
                        build_qzss_cssr_gridded_message(
                            tow_delta=30,
                            iod=3,
                            sync=True,
                            network_id=network_id,
                            trop_type=1,
                            trop_quality=9,
                            grid_count=grid_count,
                            selected_satellites=1,
                            trop_hs_residuals_m=trop_hs_residuals,
                            trop_wet_residuals_m=trop_wet_residuals,
                            stec_residuals_tecu=stec_residuals,
                        ),
                        build_qzss_cssr_combined_message(
                            tow_delta=30,
                            iod=3,
                            prn=1,
                            network_id=network_id,
                            dclock_m=0.0,
                            sync=False,
                        ),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=356460, iod=3, prn=1, sync=True),
                        build_qzss_cssr_gridded_message(
                            tow_delta=60,
                            iod=3,
                            sync=True,
                            network_id=network_id,
                            trop_type=1,
                            trop_quality=9,
                            grid_count=grid_count,
                            selected_satellites=1,
                            trop_hs_residuals_m=trop_hs_residuals,
                            trop_wet_residuals_m=trop_wet_residuals,
                            stec_residuals_tecu=stec_residuals,
                        ),
                        build_qzss_cssr_combined_message(
                            tow_delta=60,
                            iod=3,
                            prn=1,
                            network_id=network_id,
                            dclock_m=0.0,
                            sync=False,
                        ),
                    ]
                )
            )

            result = self.run_gnss(
                "clas-ppp",
                "--profile",
                "clas",
                "--kinematic",
                "--obs",
                str(obs_path),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--sp3",
                str(sp3_path),
                "--clk",
                str(clk_path),
                "--qzss-l6",
                str(l6_path),
                "--qzss-gps-week",
                "2411",
                "--out",
                str(output_path),
                "--summary-json",
                str(summary_path),
                "--no-estimate-troposphere",
                "--max-epochs",
                "3",
                "--require-valid-epochs-min",
                "3",
                "--require-ppp-solution-rate-min",
                "100",
                "--require-atmos-messages-min",
                "3",
                "--require-ppp-atmos-trop-corrections-min",
                "1",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["correction_encoding"], "qzss_l6")
            self.assertEqual(payload["atmos_messages"], 3)
            self.assertGreaterEqual(payload["ppp_atmospheric_trop_corrections"], 1)
            self.assertGreater(payload["ppp_atmospheric_trop_meters"], 1.0)
    def test_clas_ppp_cli_accepts_direct_qzss_l6_code_phase_bias(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_ppp_qzss_l6_code_phase_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            l6_path = temp_root / "corrections_l6_code_phase.bin"
            output_path = temp_root / "clas_ppp_l6_code_phase.pos"
            summary_path = temp_root / "clas_ppp_l6_code_phase_summary.json"
            l6_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_code_phase_bias_message(
                            tow_delta=0, iod=3, code_bias_m=-0.12, phase_bias_m=0.015, sync=True
                        ),
                        build_qzss_cssr_combined_message(tow_delta=0, iod=3, prn=3, dclock_m=0.025, sync=False),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518430, iod=3, prn=3, sync=True),
                        build_qzss_cssr_code_phase_bias_message(
                            tow_delta=30, iod=3, code_bias_m=-0.12, phase_bias_m=0.015, sync=True
                        ),
                        build_qzss_cssr_combined_message(tow_delta=30, iod=3, prn=3, dclock_m=0.025, sync=False),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518460, iod=3, prn=3, sync=True),
                        build_qzss_cssr_code_phase_bias_message(
                            tow_delta=60, iod=3, code_bias_m=-0.12, phase_bias_m=0.015, sync=True
                        ),
                        build_qzss_cssr_combined_message(tow_delta=60, iod=3, prn=3, dclock_m=0.025, sync=False),
                    ]
                )
            )

            result = self.run_gnss(
                "clas-ppp",
                "--profile",
                "clas",
                "--static",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--qzss-l6",
                str(l6_path),
                "--qzss-gps-week",
                "1316",
                "--out",
                str(output_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "3",
                "--require-valid-epochs-min",
                "3",
                "--require-ppp-solution-rate-min",
                "100",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertTrue(output_path.exists())
            self.assertTrue(summary_path.exists())
            self.assertIn("decoded qzss l6 corrections:", result.stdout)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["correction_encoding"], "qzss_l6")
            self.assertEqual(payload["ppp_solution_rate_pct"], 100.0)
    def test_clas_ppp_cli_accepts_direct_qzss_l6_code_bias_composition_policy(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_ppp_qzss_l6_code_bias_comp_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            l6_path = temp_root / "corrections_l6_code_bias_comp.bin"
            output_path = temp_root / "clas_ppp_l6_code_bias_comp.pos"
            summary_path = temp_root / "clas_ppp_l6_code_bias_comp_summary.json"
            l6_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_code_bias_message(tow_delta=0, iod=3, bias_m=-0.12, sync=True),
                        build_qzss_cssr_code_phase_bias_message(
                            tow_delta=0,
                            iod=3,
                            code_bias_m=0.04,
                            sync=True,
                            network_bias=True,
                            code_bias_exists=True,
                            phase_bias_exists=False,
                            selected_mask=0b1,
                        ),
                        build_qzss_cssr_combined_message(tow_delta=0, iod=3, prn=3, dclock_m=0.025, sync=False),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518430, iod=3, prn=3, sync=True),
                        build_qzss_cssr_code_bias_message(tow_delta=30, iod=3, bias_m=-0.12, sync=True),
                        build_qzss_cssr_code_phase_bias_message(
                            tow_delta=30,
                            iod=3,
                            code_bias_m=0.04,
                            sync=True,
                            network_bias=True,
                            code_bias_exists=True,
                            phase_bias_exists=False,
                            selected_mask=0b1,
                        ),
                        build_qzss_cssr_combined_message(tow_delta=30, iod=3, prn=3, dclock_m=0.025, sync=False),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518460, iod=3, prn=3, sync=True),
                        build_qzss_cssr_code_bias_message(tow_delta=60, iod=3, bias_m=-0.12, sync=True),
                        build_qzss_cssr_code_phase_bias_message(
                            tow_delta=60,
                            iod=3,
                            code_bias_m=0.04,
                            sync=True,
                            network_bias=True,
                            code_bias_exists=True,
                            phase_bias_exists=False,
                            selected_mask=0b1,
                        ),
                        build_qzss_cssr_combined_message(tow_delta=60, iod=3, prn=3, dclock_m=0.025, sync=False),
                    ]
                )
            )

            result = self.run_gnss(
                "clas-ppp",
                "--profile",
                "clas",
                "--static",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--qzss-l6",
                str(l6_path),
                "--qzss-gps-week",
                "1316",
                "--compact-code-bias-composition-policy",
                "base-plus-network",
                "--out",
                str(output_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "3",
                "--require-valid-epochs-min",
                "3",
                "--require-ppp-solution-rate-min",
                "100",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertTrue(output_path.exists())
            self.assertTrue(summary_path.exists())
            self.assertIn("decoded qzss l6 corrections:", result.stdout)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["correction_encoding"], "qzss_l6")
            self.assertEqual(payload["compact_code_bias_composition_policy"], "base-plus-network")
            self.assertEqual(payload["ppp_solution_rate_pct"], 100.0)
    def test_clas_ppp_cli_accepts_direct_qzss_l6_code_bias_bank_policy(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_ppp_qzss_l6_code_bias_bank_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            l6_path = temp_root / "corrections_l6_code_bias_bank.bin"
            output_path = temp_root / "clas_ppp_l6_code_bias_bank.pos"
            summary_path = temp_root / "clas_ppp_l6_code_bias_bank_summary.json"
            l6_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518370, iod=3, prn=3, sync=True),
                        build_qzss_cssr_code_bias_message(tow_delta=0, iod=3, bias_m=-0.12, sync=False),
                        build_qzss_cssr_combined_message(tow_delta=0, iod=3, prn=3, dclock_m=0.025, sync=False),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_code_phase_bias_message(
                            tow_delta=0,
                            iod=3,
                            code_bias_m=0.04,
                            sync=True,
                            network_bias=True,
                            code_bias_exists=True,
                            phase_bias_exists=False,
                            selected_mask=0b1,
                        ),
                        build_qzss_cssr_combined_message(tow_delta=0, iod=3, prn=3, dclock_m=0.025, sync=False),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518430, iod=3, prn=3, sync=True),
                        build_qzss_cssr_code_phase_bias_message(
                            tow_delta=30,
                            iod=3,
                            code_bias_m=0.04,
                            sync=True,
                            network_bias=True,
                            code_bias_exists=True,
                            phase_bias_exists=False,
                            selected_mask=0b1,
                        ),
                        build_qzss_cssr_combined_message(tow_delta=30, iod=3, prn=3, dclock_m=0.025, sync=False),
                    ]
                )
            )

            result = self.run_gnss(
                "clas-ppp",
                "--profile",
                "clas",
                "--static",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--qzss-l6",
                str(l6_path),
                "--qzss-gps-week",
                "1316",
                "--compact-code-bias-composition-policy",
                "base-plus-network",
                "--compact-code-bias-bank-policy",
                "latest-preceding-bank",
                "--out",
                str(output_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "3",
                "--require-valid-epochs-min",
                "3",
                "--require-ppp-solution-rate-min",
                "100",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["compact_code_bias_composition_policy"], "base-plus-network")
            self.assertEqual(payload["compact_code_bias_bank_policy"], "latest-preceding-bank")
            self.assertEqual(payload["ppp_solution_rate_pct"], 100.0)
    def test_clas_ppp_cli_accepts_direct_qzss_l6_bias_row_materialization(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_ppp_qzss_l6_bias_rows_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            l6_path = temp_root / "corrections_l6_bias_rows.bin"
            output_path = temp_root / "clas_ppp_l6_bias_rows.pos"
            summary_path = temp_root / "clas_ppp_l6_bias_rows_summary.json"
            l6_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True, sigmask=0xC000),
                        build_qzss_cssr_code_phase_bias_message(
                            tow_delta=0,
                            iod=3,
                            sync=True,
                            network_bias=False,
                            code_bias_exists=True,
                            phase_bias_exists=True,
                            code_biases_m=(-0.12, -0.08),
                            phase_biases_m=(0.015, 0.025),
                            entries_per_selected_satellite=2,
                        ),
                        build_qzss_cssr_combined_message(tow_delta=0, iod=3, prn=3, dclock_m=0.025, sync=False),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518430, iod=3, prn=3, sync=True, sigmask=0x8000),
                        build_qzss_cssr_code_phase_bias_message(
                            tow_delta=30,
                            iod=3,
                            sync=True,
                            network_bias=True,
                            code_bias_exists=True,
                            phase_bias_exists=True,
                            code_biases_m=(0.04,),
                            phase_biases_m=(0.045,),
                            entries_per_selected_satellite=1,
                        ),
                        build_qzss_cssr_combined_message(tow_delta=30, iod=3, prn=3, dclock_m=0.025, sync=False),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518460, iod=3, prn=3, sync=True, sigmask=0x8000),
                        build_qzss_cssr_code_phase_bias_message(
                            tow_delta=60,
                            iod=3,
                            sync=True,
                            network_bias=True,
                            code_bias_exists=True,
                            phase_bias_exists=True,
                            code_biases_m=(0.04,),
                            phase_biases_m=(0.045,),
                            entries_per_selected_satellite=1,
                        ),
                        build_qzss_cssr_combined_message(tow_delta=60, iod=3, prn=3, dclock_m=0.025, sync=False),
                    ]
                )
            )

            result = self.run_gnss(
                "clas-ppp",
                "--profile",
                "clas",
                "--static",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--qzss-l6",
                str(l6_path),
                "--qzss-gps-week",
                "1316",
                "--compact-bias-row-materialization",
                "selected-satellite-base-extend",
                "--compact-code-bias-bank-policy",
                "same-30s-bank",
                "--compact-phase-bias-bank-policy",
                "same-30s-bank",
                "--out",
                str(output_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "3",
                "--require-valid-epochs-min",
                "3",
                "--require-ppp-solution-rate-min",
                "100",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["compact_bias_row_materialization"], "selected-satellite-base-extend")
            self.assertEqual(payload["compact_code_bias_bank_policy"], "same-30s-bank")
            self.assertEqual(payload["compact_phase_bias_bank_policy"], "same-30s-bank")
            self.assertEqual(payload["ppp_solution_rate_pct"], 100.0)
    def test_clas_ppp_cli_accepts_direct_qzss_l6_row_construction_policy(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_ppp_qzss_l6_row_constr_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            l6_path = temp_root / "corrections_l6_row_constr.bin"
            output_path = temp_root / "clas_ppp_l6_row_constr.pos"
            summary_path = temp_root / "clas_ppp_l6_row_constr_summary.json"
            l6_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True, sigmask=0xC000),
                        build_qzss_cssr_code_phase_bias_message(
                            tow_delta=0,
                            iod=3,
                            sync=True,
                            network_bias=False,
                            code_bias_exists=True,
                            phase_bias_exists=True,
                            code_biases_m=(-0.12, -0.08),
                            phase_biases_m=(0.015, 0.025),
                            entries_per_selected_satellite=2,
                        ),
                        build_qzss_cssr_combined_message(tow_delta=0, iod=3, prn=3, dclock_m=0.025, sync=False),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518430, iod=3, prn=3, sync=True, sigmask=0x8000),
                        build_qzss_cssr_code_phase_bias_message(
                            tow_delta=30,
                            iod=3,
                            sync=True,
                            network_bias=True,
                            code_bias_exists=True,
                            phase_bias_exists=True,
                            code_biases_m=(0.04,),
                            phase_biases_m=(0.045,),
                            entries_per_selected_satellite=1,
                        ),
                        build_qzss_cssr_combined_message(tow_delta=30, iod=3, prn=3, dclock_m=0.025, sync=False),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518460, iod=3, prn=3, sync=True, sigmask=0x8000),
                        build_qzss_cssr_code_phase_bias_message(
                            tow_delta=60,
                            iod=3,
                            sync=True,
                            network_bias=True,
                            code_bias_exists=True,
                            phase_bias_exists=True,
                            code_biases_m=(0.04,),
                            phase_biases_m=(0.045,),
                            entries_per_selected_satellite=1,
                        ),
                        build_qzss_cssr_combined_message(tow_delta=60, iod=3, prn=3, dclock_m=0.025, sync=False),
                    ]
                )
            )

            result = self.run_gnss(
                "clas-ppp",
                "--profile",
                "clas",
                "--static",
                "--obs",
                str(ROOT_DIR / "data/rover_static.obs"),
                "--nav",
                str(ROOT_DIR / "data/navigation_static.nav"),
                "--qzss-l6",
                str(l6_path),
                "--qzss-gps-week",
                "1316",
                "--compact-row-construction-policy",
                "coupled-code-phase",
                "--out",
                str(output_path),
                "--summary-json",
                str(summary_path),
                "--max-epochs",
                "3",
                "--require-valid-epochs-min",
                "3",
                "--require-ppp-solution-rate-min",
                "100",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["compact_row_construction_policy"], "coupled-code-phase")
            self.assertEqual(payload["ppp_solution_rate_pct"], 100.0)
