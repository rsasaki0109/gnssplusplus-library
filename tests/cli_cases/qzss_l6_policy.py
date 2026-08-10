"""CLI regression cases for the QZSSL6PolicyCases domain."""

from ._support import *  # noqa: F401,F403

class QZSSL6PolicyCases:
    def test_qzss_l6_info_extracts_atmos_inventory(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_atmos_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_atmos.bin"
            messages_path = temp_root / "session_atmos_messages.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_atmos_message(
                            tow_delta=0,
                            iod=3,
                            sync=False,
                            network_id=1,
                            trop_avail=3,
                            stec_avail=3,
                            grid_count=1,
                            selected_satellites=1,
                        ),
                    ]
                )
            )

            result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--limit",
                "5",
                "--extract-compact-messages",
                str(messages_path),
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("cssr_message: subframe=1 index=2 subtype=12 tow=518400", result.stdout)
            self.assertIn("detail=network=1 grids=1 trop=3 stec=3 sats=1", result.stdout)
            messages_csv = messages_path.read_text(encoding="ascii")
            self.assertIn("1,2,4073,12,518400,1.0,0,3,", messages_csv)
            self.assertIn("network=1 grids=1 trop=3 stec=3 sats=1", messages_csv)
    def test_qzss_l6_info_extracts_stec_inventory(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_stec_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_stec.bin"
            messages_path = temp_root / "session_stec_messages.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_stec_message(
                            tow_delta=0,
                            iod=3,
                            sync=False,
                            network_id=7,
                            stec_type=3,
                            selected_satellites=1,
                        ),
                    ]
                )
            )

            result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--limit",
                "5",
                "--extract-compact-messages",
                str(messages_path),
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("cssr_message: subframe=1 index=2 subtype=8 tow=518400", result.stdout)
            self.assertIn("detail=network=7 stec_type=3 sats=1", result.stdout)
            messages_csv = messages_path.read_text(encoding="ascii")
            self.assertIn("1,2,4073,8,518400,1.0,0,3,", messages_csv)
            self.assertIn("network=7 stec_type=3 sats=1", messages_csv)
    def test_qzss_l6_info_extracts_service_info_packets(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_service_info_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_service_info.bin"
            messages_path = temp_root / "session_service_messages.csv"
            packets_path = temp_root / "session_service_packets.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_service_info_message(
                            sync=True,
                            info_counter=0,
                            payload_bytes=b"\x01\x02\x03\x04\x05",
                        ),
                        build_qzss_cssr_service_info_message(
                            sync=False,
                            info_counter=1,
                            payload_bytes=b"\x06\x07\x08\x09\x0a",
                        ),
                    ]
                )
            )

            result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-messages",
                str(messages_path),
                "--extract-service-info",
                str(packets_path),
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("cssr_message: subframe=1 index=1 subtype=10 tow=0", result.stdout)
            self.assertIn("cssr_message: subframe=1 index=2 subtype=10 tow=0", result.stdout)
            self.assertIn("extracted: service_info_packets=1", result.stdout)
            messages_csv = messages_path.read_text(encoding="ascii")
            self.assertIn(",10,0,0.0,1,0,", messages_csv)
            self.assertIn(",10,0,0.0,0,1,", messages_csv)
            packets_csv = packets_path.read_text(encoding="ascii")
            self.assertIn("packet_index,first_subframe_index,last_subframe_index,chunk_count,total_bits,packet_hex", packets_csv)
            self.assertIn("1,1,1,2,80,0102030405060708090a", packets_csv)
    def test_qzss_l6_info_exports_atmos_metadata_on_compact_corrections(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_atmos_corr_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_atmos_corr.bin"
            corrections_path = temp_root / "session_atmos_corr.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_atmos_message(
                            tow_delta=0,
                            iod=3,
                            sync=True,
                            network_id=1,
                            trop_avail=3,
                            stec_avail=3,
                            grid_count=1,
                            trop_quality=9,
                            trop_type=2,
                            trop_t00_m=0.12,
                            trop_t01_m_per_deg=0.01,
                            trop_t10_m_per_deg=-0.02,
                            trop_t11_m_per_deg2=0.003,
                            trop_residual_size=0,
                            trop_offset_m=0.08,
                            trop_residuals_m=(0.02,),
                            stec_quality=17,
                            stec_type=3,
                            stec_c00_tecu=1.5,
                            stec_c01_tecu_per_deg=0.12,
                            stec_c10_tecu_per_deg=-0.10,
                            stec_c11_tecu_per_deg2=0.06,
                            stec_c02_tecu_per_deg2=0.025,
                            stec_c20_tecu_per_deg2=-0.015,
                            stec_residual_size=2,
                            stec_residuals_tecu=(0.32,),
                        ),
                        build_qzss_cssr_combined_message(
                            tow_delta=0, iod=3, prn=3, dclock_m=0.025, sync=False
                        ),
                    ]
                )
            )

            result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(corrections_path),
                "--gps-week",
                "2200",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            corrections_csv = corrections_path.read_text(encoding="ascii")
            self.assertIn("atmos_network_id=1", corrections_csv)
            self.assertIn("atmos_trop_avail=3", corrections_csv)
            self.assertIn("atmos_stec_avail=3", corrections_csv)
            self.assertIn("atmos_grid_count=1", corrections_csv)
            self.assertIn("atmos_trop_quality=9", corrections_csv)
            self.assertIn("atmos_trop_type=2", corrections_csv)
            self.assertIn("atmos_trop_t00_m=0.120000", corrections_csv)
            self.assertIn("atmos_trop_t01_m_per_deg=0.010000", corrections_csv)
            self.assertIn("atmos_trop_t10_m_per_deg=-0.020000", corrections_csv)
            self.assertIn("atmos_trop_t11_m_per_deg2=0.003000", corrections_csv)
            self.assertIn("atmos_trop_offset_m=0.080000", corrections_csv)
            self.assertIn("atmos_trop_residuals_m=0.020000", corrections_csv)
            self.assertIn("atmos_stec_quality:G03=17", corrections_csv)
            self.assertIn("atmos_stec_type:G03=3", corrections_csv)
            self.assertIn("atmos_stec_c00_tecu:G03=1.500000", corrections_csv)
            self.assertIn("atmos_stec_c01_tecu_per_deg:G03=0.120000", corrections_csv)
            self.assertIn("atmos_stec_c10_tecu_per_deg:G03=-0.100000", corrections_csv)
            self.assertIn("atmos_stec_c11_tecu_per_deg2:G03=0.060000", corrections_csv)
            self.assertIn("atmos_stec_c02_tecu_per_deg2:G03=0.025000", corrections_csv)
            self.assertIn("atmos_stec_c20_tecu_per_deg2:G03=-0.015000", corrections_csv)
            self.assertIn("atmos_stec_residual_size:G03=2", corrections_csv)
            self.assertIn("atmos_stec_residuals_tecu:G03=0.320000", corrections_csv)
    def test_qzss_l6_info_exports_stec_metadata_on_compact_corrections(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_stec_corr_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_stec_corr.bin"
            corrections_path = temp_root / "session_stec_corr.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_stec_message(
                            tow_delta=0,
                            iod=3,
                            sync=True,
                            network_id=7,
                            stec_quality=17,
                            stec_type=3,
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
            )

            result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(corrections_path),
                "--gps-week",
                "2200",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            corrections_csv = corrections_path.read_text(encoding="ascii")
            self.assertIn("atmos_network_id=7", corrections_csv)
            self.assertIn("atmos_trop_avail=0", corrections_csv)
            self.assertIn("atmos_stec_avail=1", corrections_csv)
            self.assertIn("atmos_grid_count=0", corrections_csv)
            self.assertIn("atmos_stec_satellites=G03", corrections_csv)
            self.assertIn("atmos_stec_satellites:7=G03", corrections_csv)
            self.assertIn("atmos_stec_quality:G03=17", corrections_csv)
            self.assertIn("atmos_stec_type:G03=3", corrections_csv)
            self.assertIn("atmos_stec_c00_tecu:G03=1.500000", corrections_csv)
            self.assertIn("atmos_stec_c01_tecu_per_deg:G03=0.120000", corrections_csv)
            self.assertIn("atmos_stec_c10_tecu_per_deg:G03=-0.100000", corrections_csv)
            self.assertIn("atmos_stec_c11_tecu_per_deg2:G03=0.060000", corrections_csv)
            self.assertIn("atmos_stec_c02_tecu_per_deg2:G03=0.025000", corrections_csv)
            self.assertIn("atmos_stec_c20_tecu_per_deg2:G03=-0.015000", corrections_csv)
    def test_qzss_l6_info_exports_gridded_metadata_on_compact_corrections(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_gridded_corr_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_gridded_corr.bin"
            corrections_path = temp_root / "session_gridded_corr.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_gridded_message(
                            tow_delta=0,
                            iod=3,
                            sync=True,
                            network_id=7,
                            trop_type=1,
                            trop_quality=9,
                            grid_count=2,
                            selected_satellites=1,
                            trop_hs_residuals_m=(0.100, 0.200),
                            trop_wet_residuals_m=(-0.020, 0.032),
                            stec_residuals_tecu=(0.40, -0.28),
                        ),
                        build_qzss_cssr_combined_message(
                            tow_delta=0, iod=3, prn=3, dclock_m=0.025, sync=False
                        ),
                    ]
                )
            )

            result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(corrections_path),
                "--gps-week",
                "2200",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            corrections_csv = corrections_path.read_text(encoding="ascii")
            self.assertIn("atmos_network_id=7", corrections_csv)
            self.assertIn("atmos_trop_type=1", corrections_csv)
            self.assertIn("atmos_trop_quality=9", corrections_csv)
            self.assertIn("atmos_grid_count=2", corrections_csv)
            self.assertIn("atmos_trop_hs_residuals_m=0.100000;0.200000", corrections_csv)
            self.assertIn("atmos_trop_wet_residuals_m=-0.020000;0.032000", corrections_csv)
            self.assertIn("atmos_stec_residual_range=0", corrections_csv)
            self.assertIn("atmos_stec_residuals_tecu:G03=0.400000;-0.280000", corrections_csv)
    def test_qzss_l6_info_extracts_code_phase_bias_corrections(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_pbias_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_pbias.bin"
            messages_path = temp_root / "session_pbias_messages.csv"
            corrections_path = temp_root / "session_pbias_corrections.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_code_phase_bias_message(
                            tow_delta=0, iod=3, code_bias_m=-0.12, phase_bias_m=0.015, sync=True
                        ),
                        build_qzss_cssr_combined_message(
                            tow_delta=0, iod=3, prn=3, dclock_m=0.025, sync=False
                        ),
                    ]
                )
            )

            result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-messages",
                str(messages_path),
                "--extract-compact-corrections",
                str(corrections_path),
                "--gps-week",
                "2200",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("cssr_message: subframe=1 index=2 subtype=6 tow=518400", result.stdout)
            self.assertIn("mapped_code=1 mapped_phase=1", result.stdout)
            corrections_csv = corrections_path.read_text(encoding="ascii")
            self.assertIn("cbias:2=-0.120000", corrections_csv)
            self.assertIn("pbias:2=0.015000", corrections_csv)
            messages_csv = messages_path.read_text(encoding="ascii")
            self.assertIn(",6,518400,", messages_csv)
    def test_qzss_l6_info_extracts_phase_bias_only_corrections(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_phase_bias_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_phase_bias.bin"
            messages_path = temp_root / "session_phase_bias_messages.csv"
            corrections_path = temp_root / "session_phase_bias_corrections.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_phase_bias_message(
                            tow_delta=0, iod=3, phase_bias_m=0.015, sync=True
                        ),
                        build_qzss_cssr_combined_message(
                            tow_delta=0, iod=3, prn=3, dclock_m=0.025, sync=False
                        ),
                    ]
                )
            )

            result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-messages",
                str(messages_path),
                "--extract-compact-corrections",
                str(corrections_path),
                "--gps-week",
                "2200",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("cssr_message: subframe=1 index=2 subtype=5 tow=518400", result.stdout)
            self.assertIn("mapped_phase=1", result.stdout)
            corrections_csv = corrections_path.read_text(encoding="ascii")
            self.assertIn("pbias:2=0.015000", corrections_csv)
            messages_csv = messages_path.read_text(encoding="ascii")
            self.assertIn(",5,518400,", messages_csv)
    def test_qzss_l6_info_compact_flush_policy_drops_phase_bias_only_rows_without_orbit_or_clock(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_flush_phase_only_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_phase_bias_only.bin"
            corrections_path = temp_root / "session_phase_bias_only_corrections.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_phase_bias_message(
                            tow_delta=0, iod=3, phase_bias_m=0.015, sync=False
                        ),
                    ]
                )
            )

            result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(corrections_path),
                "--gps-week",
                "2200",
                "--compact-flush-policy",
                "orbit-or-clock-only",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            corrections_csv = corrections_path.read_text(encoding="ascii")
            self.assertEqual(
                corrections_csv.strip(),
                "# week,tow,system,prn,dx,dy,dz,dclock_m,high_rate_clock_m,clock_network_id[,ura_sigma_m=<m>][,cbias:<id>=<m>...][,pbias:<id>=<m>...][,bias_network_id=<n>][,atmos_<name>=<value>...]",
            )
    def test_qzss_l6_info_compact_flush_policy_can_require_both_orbit_and_clock(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_flush_orbit_clock_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_orbit_only.bin"
            orbit_or_clock_path = temp_root / "orbit_or_clock.csv"
            orbit_and_clock_path = temp_root / "orbit_and_clock.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_orbit_message(
                            tow_delta=0, iod=3, dx=0.16, dy=0.32, dz=-0.32, sync=False
                        ),
                    ]
                )
            )

            orbit_or_clock = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(orbit_or_clock_path),
                "--gps-week",
                "2200",
                "--compact-flush-policy",
                "orbit-or-clock-only",
            )
            self.assertEqual(orbit_or_clock.returncode, 0, msg=orbit_or_clock.stderr)
            orbit_or_clock_csv = orbit_or_clock_path.read_text(encoding="ascii")
            self.assertIn("0.160000", orbit_or_clock_csv)

            orbit_and_clock = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(orbit_and_clock_path),
                "--gps-week",
                "2200",
                "--compact-flush-policy",
                "orbit-and-clock-only",
            )
            self.assertEqual(orbit_and_clock.returncode, 0, msg=orbit_and_clock.stderr)
            orbit_and_clock_csv = orbit_and_clock_path.read_text(encoding="ascii")
            self.assertEqual(
                orbit_and_clock_csv.strip(),
                "# week,tow,system,prn,dx,dy,dz,dclock_m,high_rate_clock_m,clock_network_id[,ura_sigma_m=<m>][,cbias:<id>=<m>...][,pbias:<id>=<m>...][,bias_network_id=<n>][,atmos_<name>=<value>...]",
            )
    def test_qzss_l6_info_compact_atmos_merge_policy_no_carry_drops_stec_coefficients_between_epochs(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_no_carry_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_no_carry.bin"
            default_path = temp_root / "default.csv"
            no_carry_path = temp_root / "no_carry.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_stec_message(
                            tow_delta=0,
                            iod=3,
                            sync=True,
                            network_id=7,
                            selected_satellites=1,
                            stec_type=0,
                            stec_c00_tecu=1.5,
                        ),
                        build_qzss_cssr_combined_message(
                            tow_delta=0,
                            iod=3,
                            prn=3,
                            dclock_m=0.025,
                            sync=False,
                        ),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518430, iod=3, prn=3, sync=True),
                        build_qzss_cssr_combined_message(
                            tow_delta=30,
                            iod=3,
                            prn=3,
                            dclock_m=0.025,
                            sync=False,
                        ),
                    ]
                )
            )

            default_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(default_path),
                "--gps-week",
                "2200",
            )
            self.assertEqual(default_result.returncode, 0, msg=default_result.stderr)
            default_second_row = next(
                line
                for line in default_path.read_text(encoding="ascii").splitlines()
                if line.startswith("2200,518430.000")
            )
            self.assertIn("atmos_stec_c00_tecu:G03=1.500000", default_second_row)

            no_carry_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(no_carry_path),
                "--gps-week",
                "2200",
                "--compact-atmos-merge-policy",
                "no-carry",
            )
            self.assertEqual(no_carry_result.returncode, 0, msg=no_carry_result.stderr)
            no_carry_second_row = next(
                line
                for line in no_carry_path.read_text(encoding="ascii").splitlines()
                if line.startswith("2200,518430.000")
            )
            self.assertNotIn("atmos_stec_c00_tecu:G03=1.500000", no_carry_second_row)
    def test_qzss_l6_info_compact_atmos_merge_policy_network_locked_resets_carried_stec_coefficients(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_network_locked_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_network_locked.bin"
            default_path = temp_root / "default.csv"
            network_locked_path = temp_root / "network_locked.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_stec_message(
                            tow_delta=0,
                            iod=3,
                            sync=True,
                            network_id=7,
                            selected_satellites=1,
                            stec_type=0,
                            stec_c00_tecu=1.5,
                        ),
                        build_qzss_cssr_combined_message(
                            tow_delta=0,
                            iod=3,
                            prn=3,
                            dclock_m=0.025,
                            sync=False,
                        ),
                    ]
                )
                + build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518430, iod=3, prn=3, sync=True),
                        build_qzss_cssr_gridded_message(
                            tow_delta=30,
                            iod=3,
                            sync=True,
                            network_id=9,
                            selected_satellites=1,
                        ),
                        build_qzss_cssr_combined_message(
                            tow_delta=30,
                            iod=3,
                            prn=3,
                            dclock_m=0.025,
                            sync=False,
                        ),
                    ]
                )
            )

            default_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(default_path),
                "--gps-week",
                "2200",
            )
            self.assertEqual(default_result.returncode, 0, msg=default_result.stderr)
            default_second_row = next(
                line
                for line in default_path.read_text(encoding="ascii").splitlines()
                if line.startswith("2200,518430.000")
            )
            self.assertIn("atmos_network_id=9", default_second_row)
            self.assertIn("atmos_stec_c00_tecu:G03=1.500000", default_second_row)

            network_locked_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(network_locked_path),
                "--gps-week",
                "2200",
                "--compact-atmos-merge-policy",
                "network-locked-stec-coeff-carry",
            )
            self.assertEqual(network_locked_result.returncode, 0, msg=network_locked_result.stderr)
            network_locked_second_row = next(
                line
                for line in network_locked_path.read_text(encoding="ascii").splitlines()
                if line.startswith("2200,518430.000")
            )
            self.assertIn("atmos_network_id=9", network_locked_second_row)
            self.assertNotIn("atmos_stec_c00_tecu:G03=1.500000", network_locked_second_row)
    def test_qzss_l6_info_compact_atmos_subtype_merge_policy_gridded_priority_drops_stec_polynomials(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_gridded_priority_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_gridded_priority.bin"
            default_path = temp_root / "default.csv"
            priority_path = temp_root / "gridded_priority.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_stec_message(
                            tow_delta=0,
                            iod=3,
                            sync=True,
                            network_id=7,
                            selected_satellites=1,
                            stec_type=0,
                            stec_c00_tecu=1.5,
                        ),
                        build_qzss_cssr_gridded_message(
                            tow_delta=0,
                            iod=3,
                            sync=True,
                            network_id=7,
                            selected_satellites=1,
                            stec_residuals_tecu=(0.24,),
                        ),
                        build_qzss_cssr_combined_message(
                            tow_delta=0,
                            iod=3,
                            prn=3,
                            dclock_m=0.025,
                            sync=False,
                        ),
                    ]
                )
            )

            default_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(default_path),
                "--gps-week",
                "2200",
            )
            self.assertEqual(default_result.returncode, 0, msg=default_result.stderr)
            default_row = next(
                line
                for line in default_path.read_text(encoding="ascii").splitlines()
                if line.startswith("2200,518400.000")
            )
            self.assertIn("atmos_stec_c00_tecu:G03=1.500000", default_row)
            self.assertIn("atmos_stec_residuals_tecu:G03=0.240000", default_row)

            priority_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(priority_path),
                "--gps-week",
                "2200",
                "--compact-atmos-subtype-merge-policy",
                "gridded-priority",
            )
            self.assertEqual(priority_result.returncode, 0, msg=priority_result.stderr)
            priority_row = next(
                line
                for line in priority_path.read_text(encoding="ascii").splitlines()
                if line.startswith("2200,518400.000")
            )
            self.assertNotIn("atmos_stec_c00_tecu:G03=1.500000", priority_row)
            self.assertIn("atmos_stec_residuals_tecu:G03=0.240000", priority_row)
    def test_qzss_l6_info_compact_atmos_subtype_merge_policy_combined_priority_replaces_partial_families(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_combined_priority_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_combined_priority.bin"
            default_path = temp_root / "default.csv"
            priority_path = temp_root / "combined_priority.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_stec_message(
                            tow_delta=0,
                            iod=3,
                            sync=True,
                            network_id=7,
                            selected_satellites=1,
                            stec_type=0,
                            stec_c00_tecu=1.5,
                        ),
                        build_qzss_cssr_gridded_message(
                            tow_delta=0,
                            iod=3,
                            sync=True,
                            network_id=7,
                            selected_satellites=1,
                            stec_residuals_tecu=(0.24,),
                        ),
                        build_qzss_cssr_atmos_message(
                            tow_delta=0,
                            iod=3,
                            sync=True,
                            network_id=7,
                            trop_avail=0,
                            stec_avail=1,
                            selected_satellites=1,
                            stec_type=0,
                            stec_c00_tecu=2.5,
                        ),
                        build_qzss_cssr_combined_message(
                            tow_delta=0,
                            iod=3,
                            prn=3,
                            dclock_m=0.025,
                            sync=False,
                        ),
                    ]
                )
            )

            default_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(default_path),
                "--gps-week",
                "2200",
            )
            self.assertEqual(default_result.returncode, 0, msg=default_result.stderr)
            default_row = next(
                line
                for line in default_path.read_text(encoding="ascii").splitlines()
                if line.startswith("2200,518400.000")
            )
            self.assertIn("atmos_stec_c00_tecu:G03=2.500000", default_row)
            self.assertIn("atmos_stec_residuals_tecu:G03=0.240000", default_row)

            priority_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(priority_path),
                "--gps-week",
                "2200",
                "--compact-atmos-subtype-merge-policy",
                "combined-priority",
            )
            self.assertEqual(priority_result.returncode, 0, msg=priority_result.stderr)
            priority_row = next(
                line
                for line in priority_path.read_text(encoding="ascii").splitlines()
                if line.startswith("2200,518400.000")
            )
            self.assertIn("atmos_stec_c00_tecu:G03=2.500000", priority_row)
            self.assertNotIn("atmos_stec_residuals_tecu:G03=0.240000", priority_row)
    def test_qzss_l6_info_compact_phase_bias_merge_policy_message_reset_drops_stale_satellite_biases(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_phase_bias_reset_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_phase_bias_reset.bin"
            default_path = temp_root / "default.csv"
            reset_path = temp_root / "message_reset.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prns=(3, 4), sync=True),
                        build_qzss_cssr_phase_bias_message(
                            tow_delta=0,
                            iod=3,
                            sync=True,
                            satellite_count=2,
                            phase_biases_m=(0.015, 0.025),
                        ),
                        build_qzss_cssr_code_phase_bias_message(
                            tow_delta=0,
                            iod=3,
                            sync=False,
                            network_bias=True,
                            code_bias_exists=False,
                            phase_bias_exists=True,
                            satellite_count=2,
                            selected_mask=0b01,
                            phase_biases_m=(0.045,),
                        ),
                    ]
                )
            )

            default_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(default_path),
                "--gps-week",
                "2200",
            )
            self.assertEqual(default_result.returncode, 0, msg=default_result.stderr)
            default_csv = default_path.read_text(encoding="ascii")
            self.assertIn("2200,518400.000,G,3", default_csv)
            self.assertIn("pbias:2=0.015000", default_csv)

            reset_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(reset_path),
                "--gps-week",
                "2200",
                "--compact-phase-bias-merge-policy",
                "message-reset",
            )
            self.assertEqual(reset_result.returncode, 0, msg=reset_result.stderr)
            reset_csv = reset_path.read_text(encoding="ascii")
            self.assertNotIn("2200,518400.000,G,3", reset_csv)
            self.assertIn("2200,518400.000,G,4", reset_csv)
            self.assertIn("pbias:2=0.045000", reset_csv)
    def test_qzss_l6_info_preserves_exact_rtklib_phase_bias_signal_identity(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_exact_pbias_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_exact_pbias.bin"
            output_path = temp_root / "exact_pbias.csv"
            # GPS CSSR slots 0, 8 and 10 are respectively RTKLIB CODE_L1C,
            # CODE_L2X and CODE_L2W.  CODE_L2L (17) is deliberately absent;
            # collapsing the cells to RTCM bands must not invent it.
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(
                            tow=518400,
                            iod=3,
                            prn=4,
                            sigmask=0x80A0,
                            sync=True,
                        ),
                        build_qzss_cssr_phase_bias_message(
                            tow_delta=0,
                            iod=3,
                            phase_biases_m=(0.101, 0.202, 0.303),
                            entry_count=3,
                            sync=False,
                        ),
                    ]
                )
            )

            result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(output_path),
                "--gps-week",
                "2200",
            )
            self.assertEqual(result.returncode, 0, msg=result.stderr)
            row = next(
                line
                for line in output_path.read_text(encoding="ascii").splitlines()
                if line.startswith("2200,518400.000,G,4")
            )
            self.assertIn("pbias_code:1=0.101000", row)
            self.assertIn("pbias_code:18=0.202000", row)
            self.assertIn("pbias_code:20=0.303000", row)
            self.assertNotIn("pbias_code:17=", row)
    def test_qzss_l6_info_compact_phase_bias_merge_policy_selected_mask_prune_drops_unselected_satellites(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_phase_bias_prune_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_phase_bias_prune.bin"
            default_path = temp_root / "default.csv"
            prune_path = temp_root / "selected_mask_prune.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prns=(3, 4), sync=True),
                        build_qzss_cssr_code_phase_bias_message(
                            tow_delta=0,
                            iod=3,
                            sync=True,
                            network_bias=True,
                            code_bias_exists=False,
                            phase_bias_exists=True,
                            satellite_count=2,
                            selected_mask=0b10,
                            phase_biases_m=(0.015,),
                        ),
                        build_qzss_cssr_code_phase_bias_message(
                            tow_delta=0,
                            iod=3,
                            sync=False,
                            network_bias=True,
                            code_bias_exists=True,
                            phase_bias_exists=False,
                            satellite_count=2,
                            selected_mask=0b01,
                            code_biases_m=(-0.12,),
                        ),
                    ]
                )
            )

            default_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(default_path),
                "--gps-week",
                "2200",
            )
            self.assertEqual(default_result.returncode, 0, msg=default_result.stderr)
            default_csv = default_path.read_text(encoding="ascii")
            self.assertIn("2200,518400.000,G,3", default_csv)
            self.assertIn("pbias:2=0.015000", default_csv)

            prune_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(prune_path),
                "--gps-week",
                "2200",
                "--compact-phase-bias-merge-policy",
                "selected-mask-prune",
            )
            self.assertEqual(prune_result.returncode, 0, msg=prune_result.stderr)
            prune_csv = prune_path.read_text(encoding="ascii")
            prune_sat3_row = next(
                line
                for line in prune_csv.splitlines()
                if line.startswith("2200,518400.000,G,3")
            )
            self.assertIn("2200,518400.000,G,4", prune_csv)
            self.assertNotIn("pbias:2=", prune_sat3_row)
            self.assertNotIn("pbias:2=0.015000", prune_csv)
    def test_qzss_l6_info_compact_phase_bias_source_policy_subtype5_priority_keeps_dedicated_phase_bias(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_phase_bias_src5_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_phase_bias_src5.bin"
            default_path = temp_root / "default.csv"
            priority_path = temp_root / "subtype5_priority.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_phase_bias_message(
                            tow_delta=0,
                            iod=3,
                            sync=True,
                            phase_bias_m=0.015,
                        ),
                        build_qzss_cssr_code_phase_bias_message(
                            tow_delta=0,
                            iod=3,
                            sync=False,
                            code_bias_exists=False,
                            phase_bias_exists=True,
                            phase_bias_m=0.045,
                        ),
                    ]
                )
            )

            default_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(default_path),
                "--gps-week",
                "2200",
            )
            self.assertEqual(default_result.returncode, 0, msg=default_result.stderr)
            self.assertIn("pbias:2=0.045000", default_path.read_text(encoding="ascii"))

            priority_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(priority_path),
                "--gps-week",
                "2200",
                "--compact-phase-bias-source-policy",
                "subtype5-priority",
            )
            self.assertEqual(priority_result.returncode, 0, msg=priority_result.stderr)
            priority_csv = priority_path.read_text(encoding="ascii")
            self.assertIn("pbias:2=0.015000", priority_csv)
            self.assertNotIn("pbias:2=0.045000", priority_csv)
    def test_qzss_l6_info_compact_phase_bias_source_policy_subtype6_priority_keeps_code_phase_bias(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_phase_bias_src6_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_phase_bias_src6.bin"
            default_path = temp_root / "default.csv"
            priority_path = temp_root / "subtype6_priority.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_code_phase_bias_message(
                            tow_delta=0,
                            iod=3,
                            sync=True,
                            code_bias_exists=False,
                            phase_bias_exists=True,
                            phase_bias_m=0.045,
                        ),
                        build_qzss_cssr_phase_bias_message(
                            tow_delta=0,
                            iod=3,
                            sync=False,
                            phase_bias_m=0.015,
                        ),
                    ]
                )
            )

            default_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(default_path),
                "--gps-week",
                "2200",
            )
            self.assertEqual(default_result.returncode, 0, msg=default_result.stderr)
            self.assertIn("pbias:2=0.015000", default_path.read_text(encoding="ascii"))

            priority_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(priority_path),
                "--gps-week",
                "2200",
                "--compact-phase-bias-source-policy",
                "subtype6-priority",
            )
            self.assertEqual(priority_result.returncode, 0, msg=priority_result.stderr)
            priority_csv = priority_path.read_text(encoding="ascii")
            self.assertIn("pbias:2=0.045000", priority_csv)
            self.assertNotIn("pbias:2=0.015000", priority_csv)
    def test_qzss_l6_info_compact_phase_bias_composition_policy_base_plus_network_adds_base_row(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_phase_bias_comp_add_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_phase_bias_comp_add.bin"
            default_path = temp_root / "default.csv"
            composed_path = temp_root / "base_plus_network.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_phase_bias_message(
                            tow_delta=0,
                            iod=3,
                            sync=True,
                            phase_bias_m=0.015,
                        ),
                        build_qzss_cssr_code_phase_bias_message(
                            tow_delta=0,
                            iod=3,
                            sync=False,
                            network_bias=True,
                            code_bias_exists=False,
                            phase_bias_exists=True,
                            selected_mask=0b1,
                            phase_bias_m=0.045,
                        ),
                    ]
                )
            )

            default_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(default_path),
                "--gps-week",
                "2200",
            )
            self.assertEqual(default_result.returncode, 0, msg=default_result.stderr)
            self.assertIn("pbias:2=0.045000", default_path.read_text(encoding="ascii"))

            composed_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(composed_path),
                "--gps-week",
                "2200",
                "--compact-phase-bias-composition-policy",
                "base-plus-network",
            )
            self.assertEqual(composed_result.returncode, 0, msg=composed_result.stderr)
            self.assertIn("pbias:2=0.060000", composed_path.read_text(encoding="ascii"))
    def test_qzss_l6_info_compact_phase_bias_composition_policy_base_only_if_present_keeps_base_row(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_phase_bias_comp_base_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_phase_bias_comp_base.bin"
            direct_path = temp_root / "direct.csv"
            base_only_path = temp_root / "base_only.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_phase_bias_message(
                            tow_delta=0,
                            iod=3,
                            sync=True,
                            phase_bias_m=0.015,
                        ),
                        build_qzss_cssr_code_phase_bias_message(
                            tow_delta=0,
                            iod=3,
                            sync=False,
                            network_bias=True,
                            code_bias_exists=False,
                            phase_bias_exists=True,
                            selected_mask=0b1,
                            phase_bias_m=0.045,
                        ),
                    ]
                )
            )

            direct_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(direct_path),
                "--gps-week",
                "2200",
            )
            self.assertEqual(direct_result.returncode, 0, msg=direct_result.stderr)
            self.assertIn("pbias:2=0.045000", direct_path.read_text(encoding="ascii"))

            base_only_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(base_only_path),
                "--gps-week",
                "2200",
                "--compact-phase-bias-composition-policy",
                "base-only-if-present",
            )
            self.assertEqual(base_only_result.returncode, 0, msg=base_only_result.stderr)
            base_only_csv = base_only_path.read_text(encoding="ascii")
            self.assertIn("pbias:2=0.015000", base_only_csv)
            self.assertNotIn("pbias:2=0.045000", base_only_csv)
    def test_qzss_l6_info_compact_phase_bias_bank_policy_same_30s_bank_uses_prior_base_bank(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_phase_bias_bank_30s_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_phase_bias_bank_30s.bin"
            default_path = temp_root / "pending_epoch.csv"
            anchored_path = temp_root / "same_30s.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_phase_bias_message(
                            tow_delta=0,
                            iod=3,
                            sync=False,
                            phase_bias_m=0.015,
                        ),
                        build_qzss_cssr_mask_message(tow=518412, iod=3, prn=3, sync=True),
                        build_qzss_cssr_code_phase_bias_message(
                            tow_delta=0,
                            iod=3,
                            sync=False,
                            network_bias=True,
                            code_bias_exists=False,
                            phase_bias_exists=True,
                            selected_mask=0b1,
                            phase_bias_m=0.045,
                        ),
                    ]
                )
            )

            default_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(default_path),
                "--gps-week",
                "2200",
                "--compact-phase-bias-composition-policy",
                "base-plus-network",
            )
            self.assertEqual(default_result.returncode, 0, msg=default_result.stderr)
            default_csv = default_path.read_text(encoding="ascii")
            self.assertIn("pbias:2=0.045000", default_csv)
            self.assertNotIn("pbias:2=0.060000", default_csv)

            anchored_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(anchored_path),
                "--gps-week",
                "2200",
                "--compact-phase-bias-composition-policy",
                "base-plus-network",
                "--compact-phase-bias-bank-policy",
                "same-30s-bank",
            )
            self.assertEqual(anchored_result.returncode, 0, msg=anchored_result.stderr)
            self.assertIn("pbias:2=0.060000", anchored_path.read_text(encoding="ascii"))
    def test_qzss_l6_info_compact_phase_bias_bank_policy_latest_preceding_bank_uses_prior_anchor(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_phase_bias_bank_prev_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_phase_bias_bank_prev.bin"
            same_30s_path = temp_root / "same_30s.csv"
            preceding_path = temp_root / "latest_preceding.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518370, iod=3, prn=3, sync=True),
                        build_qzss_cssr_phase_bias_message(
                            tow_delta=0,
                            iod=3,
                            sync=False,
                            phase_bias_m=0.015,
                        ),
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_code_phase_bias_message(
                            tow_delta=0,
                            iod=3,
                            sync=False,
                            network_bias=True,
                            code_bias_exists=False,
                            phase_bias_exists=True,
                            selected_mask=0b1,
                            phase_bias_m=0.045,
                        ),
                    ]
                )
            )

            same_30s_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(same_30s_path),
                "--gps-week",
                "2200",
                "--compact-phase-bias-composition-policy",
                "base-plus-network",
                "--compact-phase-bias-bank-policy",
                "same-30s-bank",
            )
            self.assertEqual(same_30s_result.returncode, 0, msg=same_30s_result.stderr)
            same_30s_csv = same_30s_path.read_text(encoding="ascii")
            self.assertIn("pbias:2=0.045000", same_30s_csv)
            self.assertNotIn("pbias:2=0.060000", same_30s_csv)

            preceding_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(preceding_path),
                "--gps-week",
                "2200",
                "--compact-phase-bias-composition-policy",
                "base-plus-network",
                "--compact-phase-bias-bank-policy",
                "latest-preceding-bank",
            )
            self.assertEqual(preceding_result.returncode, 0, msg=preceding_result.stderr)
            self.assertIn("pbias:2=0.060000", preceding_path.read_text(encoding="ascii"))
    def test_qzss_l6_info_compact_phase_bias_bank_policy_close_30s_bank_rejects_older_anchor(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_phase_bias_bank_close30_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_phase_bias_bank_close30.bin"
            close_30s_path = temp_root / "close_30s.csv"
            preceding_path = temp_root / "latest_preceding.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518339, iod=3, prn=3, sync=True),
                        build_qzss_cssr_phase_bias_message(
                            tow_delta=0,
                            iod=3,
                            sync=False,
                            phase_bias_m=0.015,
                        ),
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_code_phase_bias_message(
                            tow_delta=0,
                            iod=3,
                            sync=False,
                            network_bias=True,
                            code_bias_exists=False,
                            phase_bias_exists=True,
                            selected_mask=0b1,
                            phase_bias_m=0.045,
                        ),
                    ]
                )
            )

            close_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(close_30s_path),
                "--gps-week",
                "2200",
                "--compact-phase-bias-composition-policy",
                "base-plus-network",
                "--compact-phase-bias-bank-policy",
                "close-30s-bank",
            )
            self.assertEqual(close_result.returncode, 0, msg=close_result.stderr)
            close_30s_csv = close_30s_path.read_text(encoding="ascii")
            self.assertIn("pbias:2=0.045000", close_30s_csv)
            self.assertNotIn("pbias:2=0.060000", close_30s_csv)

            preceding_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(preceding_path),
                "--gps-week",
                "2200",
                "--compact-phase-bias-composition-policy",
                "base-plus-network",
                "--compact-phase-bias-bank-policy",
                "latest-preceding-bank",
            )
            self.assertEqual(preceding_result.returncode, 0, msg=preceding_result.stderr)
            self.assertIn("pbias:2=0.060000", preceding_path.read_text(encoding="ascii"))
    def test_qzss_l6_info_compact_bias_row_materialization_selected_satellite_extends_missing_phase_bias_rows(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_phase_bias_row_extend_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_phase_bias_row_extend.bin"
            strict_path = temp_root / "strict.csv"
            extend_path = temp_root / "selected_satellite_extend.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(
                            tow=518400,
                            iod=3,
                            prn=3,
                            sync=True,
                            sigmask=0xC000,
                        ),
                        build_qzss_cssr_phase_bias_message(
                            tow_delta=0,
                            iod=3,
                            phase_biases_m=(0.015, 0.025),
                            entry_count=2,
                            sync=False,
                        ),
                        build_qzss_cssr_mask_message(
                            tow=518412,
                            iod=3,
                            prn=3,
                            sync=True,
                            sigmask=0x8000,
                        ),
                        build_qzss_cssr_code_phase_bias_message(
                            tow_delta=0,
                            iod=3,
                            sync=False,
                            network_bias=True,
                            code_bias_exists=False,
                            phase_bias_exists=True,
                            selected_mask=0b1,
                            phase_biases_m=(0.045,),
                            entries_per_selected_satellite=1,
                        ),
                    ]
                )
            )

            strict_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(strict_path),
                "--gps-week",
                "2200",
                "--compact-phase-bias-bank-policy",
                "same-30s-bank",
            )
            self.assertEqual(strict_result.returncode, 0, msg=strict_result.stderr)
            strict_row = next(
                line
                for line in strict_path.read_text(encoding="ascii").splitlines()
                if line.startswith("2200,518400.000") and "bias_network_id=1" in line
            )
            self.assertIn("pbias:2=0.045000", strict_row)
            self.assertNotIn("pbias:3=", strict_row)

            extend_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(extend_path),
                "--gps-week",
                "2200",
                "--compact-phase-bias-bank-policy",
                "same-30s-bank",
                "--compact-bias-row-materialization",
                "selected-satellite-base-extend",
            )
            self.assertEqual(extend_result.returncode, 0, msg=extend_result.stderr)
            extend_row = next(
                line
                for line in extend_path.read_text(encoding="ascii").splitlines()
                if line.startswith("2200,518400.000") and "bias_network_id=1" in line
            )
            self.assertIn("pbias:2=0.045000", extend_row)
            self.assertIn("pbias:3=0.025000", extend_row)
