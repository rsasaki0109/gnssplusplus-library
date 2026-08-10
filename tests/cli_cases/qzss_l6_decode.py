"""CLI regression cases for the QZSSL6DecodeCases domain."""

from ._support import *  # noqa: F401,F403

class QZSSL6DecodeCases:
    def test_qzss_l6_info_decodes_frames_and_extracts_csv(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_test_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6.bin"
            extract_path = temp_root / "session_l6.csv"
            input_path.write_bytes(
                build_qzss_l6_frame(
                    prn=199,
                    facility_id=0,
                    subframe_start=True,
                    data_part=b"CLAS-L6-START",
                )
                + build_qzss_l6_frame(
                    prn=200,
                    facility_id=2,
                    subframe_start=False,
                    alert=True,
                    data_part=b"CLAS-L6-NEXT",
                )
            )

            result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--show-preview",
                "--extract-data-parts",
                str(extract_path),
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("l6_frame: index=1 prn=199 vendor=5 facility=Hitachi-Ota", result.stdout)
            self.assertIn("subframe_start=1", result.stdout)
            self.assertIn("preview=CLAS-L6-START", result.stdout)
            self.assertIn("l6_frame: index=2 prn=200 vendor=5 facility=Kobe", result.stdout)
            self.assertIn("alert=1", result.stdout)
            self.assertIn("extracted: frames=2", result.stdout)
            self.assertIn(
                "summary: frames=2 valid=2 clas_vendor=2 subframe_starts=1 alerts=1 subframes=0 prns=199,200",
                result.stdout,
            )

            extracted = extract_path.read_text(encoding="ascii")
            self.assertIn("frame_index,prn,vendor_id,facility_id,reserved_bits,subframe_start,alert,data_part_bits,data_part_hex,rs_parity_hex", extracted)
            self.assertIn("1,199,5,0,0,1,0,1695,", extracted)
            self.assertIn("2,200,5,2,0,0,1,1695,", extracted)
    def test_qzss_l6_info_assembles_five_part_subframe(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_subframe_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_subframe.bin"
            subframe_path = temp_root / "session_l6_subframes.csv"
            input_path.write_bytes(
                b"".join(
                    build_qzss_l6_frame(
                        prn=199,
                        facility_id=0,
                        subframe_start=index == 0,
                        data_part=f"SUBFRAME-{index}".encode("ascii"),
                    )
                    for index in range(5)
                )
            )

            result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--limit",
                "5",
                "--extract-subframes",
                str(subframe_path),
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("l6_subframe: index=1 prn=199 vendor=5 facility=Hitachi-Ota frames=5", result.stdout)
            self.assertIn("assembled: subframes=1", result.stdout)
            self.assertIn(
                "summary: frames=5 valid=5 clas_vendor=5 subframe_starts=1 alerts=0 subframes=1 prns=199",
                result.stdout,
            )

            exported = subframe_path.read_text(encoding="ascii")
            self.assertIn(
                "subframe_index,prn,vendor_id,facility_id,frame_count,alert_frames,data_bits,first_frame_index,last_frame_index,subframe_hex",
                exported,
            )
            self.assertIn("1,199,5,0,5,0,8475,1,5,", exported)
    def test_qzss_l6_info_extracts_compact_cssr_messages_and_corrections(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_compact_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_compact.bin"
            messages_path = temp_root / "session_compact_messages.csv"
            corrections_path = temp_root / "session_compact_corrections.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_combined_message(
                            tow_delta=0,
                            iod=3,
                            prn=3,
                            sync=False,
                            network_id=1,
                            dclock_m=0.025,
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
                "--gps-week",
                "1316",
                "--extract-compact-messages",
                str(messages_path),
                "--extract-compact-corrections",
                str(corrections_path),
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("cssr_message: subframe=1 index=1 subtype=1 tow=518400", result.stdout)
            self.assertIn("cssr_message: subframe=1 index=2 subtype=11 tow=518400", result.stdout)
            self.assertIn("decoded: compact_messages=2", result.stdout)
            self.assertIn("extracted: compact_corrections=1", result.stdout)
            messages_csv = messages_path.read_text(encoding="ascii")
            self.assertIn(
                "subframe_index,message_index,ctype,subtype,tow,udi_seconds,sync,iod,message_bits,correction_count,detail",
                messages_csv,
            )
            self.assertIn("1,1,4073,1,518400,1.0,1,3,", messages_csv)
            self.assertIn("1,2,4073,11,518400,1.0,0,3,", messages_csv)
            corrections_csv = corrections_path.read_text(encoding="ascii")
            self.assertIn("# week,tow,system,prn,dx,dy,dz,dclock_m,high_rate_clock_m,clock_network_id", corrections_csv)
            self.assertIn("1316,518400.000,G,3,0.000000,0.000000,0.000000,0.025600,0.000000,1", corrections_csv)
    def test_qzss_l6_info_extracts_separate_orbit_clock_corrections(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_orbit_clock_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_orbit_clock.bin"
            corrections_path = temp_root / "session_orbit_clock_corrections.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_orbit_message(
                            tow_delta=0,
                            iod=3,
                            dx=0.016,
                            dy=0.0128,
                            dz=-0.0128,
                            sync=True,
                        ),
                        build_qzss_cssr_clock_message(tow_delta=0, iod=3, dclock_m=0.0256, sync=False),
                    ]
                )
            )

            result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--limit",
                "5",
                "--gps-week",
                "1316",
                "--extract-compact-corrections",
                str(corrections_path),
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("cssr_message: subframe=1 index=2 subtype=2 tow=518400", result.stdout)
            self.assertIn("cssr_message: subframe=1 index=3 subtype=3 tow=518400", result.stdout)
            corrections_csv = corrections_path.read_text(encoding="ascii")
            self.assertIn("1316,518400.000,G,3,0.016000,0.012800,-0.012800,0.025600,0.000000", corrections_csv)
    def test_qzss_l6_info_suppresses_st11_network_orbit_in_pending_orbit(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_st11_net_orbit_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_st11_net_orbit.bin"
            corrections_path = temp_root / "session_st11_net_orbit_corrections.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_orbit_message(
                            tow_delta=0,
                            iod=3,
                            dx=0.016,
                            dy=0.0128,
                            dz=-0.0128,
                            sync=True,
                        ),
                        build_qzss_cssr_clock_message(tow_delta=0, iod=3, dclock_m=0.0256, sync=False),
                        build_qzss_cssr_combined_message(
                            tow_delta=25,
                            iod=3,
                            prn=3,
                            sync=False,
                            network_id=1,
                            dx=-0.2512,
                            dy=-0.8320,
                            dz=0.0,
                            dclock_m=0.0512,
                        ),
                    ]
                )
            )

            result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--limit",
                "6",
                "--gps-week",
                "1316",
                "--extract-compact-corrections",
                str(corrections_path),
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            corrections_csv = corrections_path.read_text(encoding="ascii")
            self.assertIn(
                "1316,518400.000,G,3,0.016000,0.012800,-0.012800,0.025600,0.000000,0",
                corrections_csv,
            )
            # Network ST11 clock is retained and tagged; network orbit stays suppressed.
            self.assertIn("518425.000", corrections_csv)
            self.assertIn("0.051200", corrections_csv)
            self.assertNotIn("-0.251200", corrections_csv)
            self.assertNotIn("-0.832000", corrections_csv)
    def test_qzss_l6_info_extracts_code_bias_and_ura_corrections(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_cbias_ura_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_cbias_ura.bin"
            corrections_path = temp_root / "session_cbias_ura_corrections.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_code_bias_message(
                            tow_delta=0,
                            iod=3,
                            bias_m=-0.12,
                            sync=True,
                        ),
                        build_qzss_cssr_ura_message(
                            tow_delta=0,
                            iod=3,
                            ura_index=9,
                            sync=True,
                        ),
                        build_qzss_cssr_combined_message(
                            tow_delta=0,
                            iod=3,
                            prn=3,
                            dclock_m=0.0256,
                            sync=False,
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
                "--gps-week",
                "1316",
                "--extract-compact-corrections",
                str(corrections_path),
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("cssr_message: subframe=1 index=2 subtype=4 tow=518400", result.stdout)
            self.assertIn("cssr_message: subframe=1 index=3 subtype=7 tow=518400", result.stdout)
            corrections_csv = corrections_path.read_text(encoding="ascii")
            self.assertIn("ura_sigma_m=0.002750", corrections_csv)
            self.assertIn("cbias:2=-0.120000", corrections_csv)
    def test_qzss_l6_info_extracts_legacy_s_system_code_bias_corrections(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_s_cbias_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_s_cbias.bin"
            corrections_path = temp_root / "session_s_cbias_corrections.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(
                            tow=518400,
                            iod=3,
                            prn=120,
                            sync=True,
                            sigmask=0x0800,
                            system_id=4,
                            prn_base=120,
                        ),
                        build_qzss_cssr_code_bias_message(
                            tow_delta=0,
                            iod=3,
                            bias_m=-0.12,
                            sync=True,
                        ),
                        build_qzss_cssr_combined_message(
                            tow_delta=0,
                            iod=3,
                            dclock_m=0.0256,
                            sync=False,
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
                "--gps-week",
                "1316",
                "--extract-compact-corrections",
                str(corrections_path),
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            corrections_csv = corrections_path.read_text(encoding="ascii")
            self.assertIn("1316,518400.000,S,120", corrections_csv)
            self.assertIn("cbias:8=-0.120000", corrections_csv)
    def test_qzss_l6_info_compact_code_bias_composition_policy_base_plus_network_adds_base_row(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_code_bias_comp_add_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_code_bias_comp_add.bin"
            direct_path = temp_root / "direct.csv"
            composed_path = temp_root / "base_plus_network.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_code_bias_message(
                            tow_delta=0,
                            iod=3,
                            bias_m=-0.12,
                            sync=True,
                        ),
                        build_qzss_cssr_code_phase_bias_message(
                            tow_delta=0,
                            iod=3,
                            sync=False,
                            network_bias=True,
                            code_bias_exists=True,
                            phase_bias_exists=False,
                            selected_mask=0b1,
                            code_bias_m=0.04,
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
            self.assertIn("cbias:2=0.040000", direct_path.read_text(encoding="ascii"))

            composed_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(composed_path),
                "--gps-week",
                "2200",
                "--compact-code-bias-composition-policy",
                "base-plus-network",
            )
            self.assertEqual(composed_result.returncode, 0, msg=composed_result.stderr)
            self.assertIn("cbias:2=-0.080000", composed_path.read_text(encoding="ascii"))
    def test_qzss_l6_info_compact_code_bias_composition_policy_base_only_if_present_keeps_base_row(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_code_bias_comp_base_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_code_bias_comp_base.bin"
            direct_path = temp_root / "direct.csv"
            base_only_path = temp_root / "base_only.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_code_bias_message(
                            tow_delta=0,
                            iod=3,
                            bias_m=-0.12,
                            sync=True,
                        ),
                        build_qzss_cssr_code_phase_bias_message(
                            tow_delta=0,
                            iod=3,
                            sync=False,
                            network_bias=True,
                            code_bias_exists=True,
                            phase_bias_exists=False,
                            selected_mask=0b1,
                            code_bias_m=0.04,
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
            self.assertIn("cbias:2=0.040000", direct_path.read_text(encoding="ascii"))

            base_only_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(base_only_path),
                "--gps-week",
                "2200",
                "--compact-code-bias-composition-policy",
                "base-only-if-present",
            )
            self.assertEqual(base_only_result.returncode, 0, msg=base_only_result.stderr)
            base_only_csv = base_only_path.read_text(encoding="ascii")
            self.assertIn("cbias:2=-0.120000", base_only_csv)
            self.assertNotIn("cbias:2=0.040000", base_only_csv)
    def test_qzss_l6_info_compact_code_bias_bank_policy_same_30s_bank_uses_prior_base_bank(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_code_bias_bank_30s_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_code_bias_bank_30s.bin"
            default_path = temp_root / "pending_epoch.csv"
            anchored_path = temp_root / "same_30s.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_code_bias_message(
                            tow_delta=0,
                            iod=3,
                            bias_m=-0.12,
                            sync=False,
                        ),
                        build_qzss_cssr_mask_message(tow=518412, iod=3, prn=3, sync=True),
                        build_qzss_cssr_code_phase_bias_message(
                            tow_delta=0,
                            iod=3,
                            sync=False,
                            network_bias=True,
                            code_bias_exists=True,
                            phase_bias_exists=False,
                            selected_mask=0b1,
                            code_bias_m=0.04,
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
                "--compact-code-bias-composition-policy",
                "base-plus-network",
            )
            self.assertEqual(default_result.returncode, 0, msg=default_result.stderr)
            default_csv = default_path.read_text(encoding="ascii")
            self.assertIn("cbias:2=0.040000", default_csv)
            self.assertNotIn("cbias:2=-0.080000", default_csv)

            anchored_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(anchored_path),
                "--gps-week",
                "2200",
                "--compact-code-bias-composition-policy",
                "base-plus-network",
                "--compact-code-bias-bank-policy",
                "same-30s-bank",
            )
            self.assertEqual(anchored_result.returncode, 0, msg=anchored_result.stderr)
            self.assertIn("cbias:2=-0.080000", anchored_path.read_text(encoding="ascii"))
    def test_qzss_l6_info_compact_code_bias_bank_policy_latest_preceding_bank_uses_prior_anchor(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_code_bias_bank_prev_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_code_bias_bank_prev.bin"
            same_30s_path = temp_root / "same_30s.csv"
            preceding_path = temp_root / "latest_preceding.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518370, iod=3, prn=3, sync=True),
                        build_qzss_cssr_code_bias_message(
                            tow_delta=0,
                            iod=3,
                            bias_m=-0.12,
                            sync=False,
                        ),
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_code_phase_bias_message(
                            tow_delta=0,
                            iod=3,
                            sync=False,
                            network_bias=True,
                            code_bias_exists=True,
                            phase_bias_exists=False,
                            selected_mask=0b1,
                            code_bias_m=0.04,
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
                "--compact-code-bias-composition-policy",
                "base-plus-network",
                "--compact-code-bias-bank-policy",
                "same-30s-bank",
            )
            self.assertEqual(same_30s_result.returncode, 0, msg=same_30s_result.stderr)
            same_30s_csv = same_30s_path.read_text(encoding="ascii")
            self.assertIn("cbias:2=0.040000", same_30s_csv)
            self.assertNotIn("cbias:2=-0.080000", same_30s_csv)

            preceding_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(preceding_path),
                "--gps-week",
                "2200",
                "--compact-code-bias-composition-policy",
                "base-plus-network",
                "--compact-code-bias-bank-policy",
                "latest-preceding-bank",
            )
            self.assertEqual(preceding_result.returncode, 0, msg=preceding_result.stderr)
            self.assertIn("cbias:2=-0.080000", preceding_path.read_text(encoding="ascii"))
    def test_qzss_l6_info_compact_code_bias_bank_policy_delayed_15s_uses_half_window_boundary(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_code_bias_bank_delay15_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_code_bias_bank_delay15.bin"
            delayed_path = temp_root / "delayed_15s.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_code_bias_message(
                            tow_delta=0,
                            iod=3,
                            bias_m=-0.12,
                            sync=True,
                        ),
                        build_qzss_cssr_code_bias_message(
                            tow_delta=30,
                            iod=3,
                            bias_m=0.06,
                            sync=True,
                        ),
                        build_qzss_cssr_code_phase_bias_message(
                            tow_delta=44,
                            iod=3,
                            sync=False,
                            network_bias=True,
                            code_bias_exists=True,
                            phase_bias_exists=False,
                            selected_mask=0b1,
                            code_bias_m=0.04,
                        ),
                        build_qzss_cssr_code_phase_bias_message(
                            tow_delta=45,
                            iod=3,
                            sync=False,
                            network_bias=True,
                            code_bias_exists=True,
                            phase_bias_exists=False,
                            selected_mask=0b1,
                            code_bias_m=0.04,
                        ),
                    ]
                )
            )

            delayed_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(delayed_path),
                "--gps-week",
                "2200",
                "--compact-code-bias-composition-policy",
                "base-plus-network",
                "--compact-code-bias-bank-policy",
                "delayed-15s-bank",
            )
            self.assertEqual(delayed_result.returncode, 0, msg=delayed_result.stderr)
            delayed_rows = delayed_path.read_text(encoding="ascii").splitlines()
            row_14s = [line for line in delayed_rows if line.startswith("2200,518444.000,G,3")]
            row_15s = [line for line in delayed_rows if line.startswith("2200,518445.000,G,3")]
            self.assertEqual(len(row_14s), 1)
            self.assertEqual(len(row_15s), 2)
            self.assertIn("cbias:2=-0.080000", row_14s[0])
            self.assertNotIn("cbias:2=0.100000", row_14s[0])
            row_15s_base = [line for line in row_15s if "bias_network_id=" not in line]
            row_15s_network = [line for line in row_15s if "bias_network_id=" in line]
            self.assertEqual(len(row_15s_base), 1)
            self.assertEqual(len(row_15s_network), 1)
            self.assertIn("cbias:2=0.060000", row_15s_base[0])
            self.assertIn("cbias:2=0.100000", row_15s_network[0])
            self.assertNotIn("cbias:2=-0.080000", row_15s_network[0])
    def test_qzss_l6_info_delayed_15s_code_bias_bank_emits_base_refresh_row(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_code_bias_bank_refresh_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_code_bias_bank_refresh.bin"
            default_path = temp_root / "pending_epoch.csv"
            delayed_path = temp_root / "delayed_15s.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_code_bias_message(
                            tow_delta=30,
                            iod=3,
                            bias_m=0.06,
                            sync=True,
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
            self.assertNotIn("518445.000", default_path.read_text(encoding="ascii"))

            delayed_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(delayed_path),
                "--gps-week",
                "2200",
                "--compact-code-bias-bank-policy",
                "delayed-15s-bank",
            )
            self.assertEqual(delayed_result.returncode, 0, msg=delayed_result.stderr)
            delayed_rows = delayed_path.read_text(encoding="ascii").splitlines()
            refresh_rows = [line for line in delayed_rows if line.startswith("2200,518445.000,G,3")]
            self.assertEqual(len(refresh_rows), 1)
            self.assertIn("cbias:2=0.060000", refresh_rows[0])
            self.assertNotIn("bias_network_id=", refresh_rows[0])
    def test_qzss_l6_info_delayed_15s_code_bias_bank_refreshes_network_rows(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_code_bias_network_refresh_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_code_bias_network_refresh.bin"
            delayed_path = temp_root / "delayed_15s.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_code_bias_message(
                            tow_delta=0,
                            iod=3,
                            bias_m=-0.12,
                            sync=True,
                        ),
                        build_qzss_cssr_code_bias_message(
                            tow_delta=30,
                            iod=3,
                            bias_m=0.06,
                            sync=True,
                        ),
                        build_qzss_cssr_code_phase_bias_message(
                            tow_delta=30,
                            iod=3,
                            sync=False,
                            network_bias=True,
                            code_bias_exists=False,
                            phase_bias_exists=True,
                            selected_mask=0b1,
                            phase_bias_m=0.02,
                        ),
                    ]
                )
            )

            delayed_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(delayed_path),
                "--gps-week",
                "2200",
                "--compact-code-bias-composition-policy",
                "base-only-if-present",
                "--compact-code-bias-bank-policy",
                "delayed-15s-bank",
                "--compact-bias-row-materialization",
                "selected-satellite-base-extend",
            )
            self.assertEqual(delayed_result.returncode, 0, msg=delayed_result.stderr)
            delayed_rows = delayed_path.read_text(encoding="ascii").splitlines()
            network_rows = [
                line
                for line in delayed_rows
                if line.startswith("2200,518445.000,G,3") and "bias_network_id=1" in line
            ]
            self.assertEqual(len(network_rows), 1)
            self.assertIn("cbias:2=0.060000", network_rows[0])
    def test_qzss_l6_info_compact_code_bias_base_only_uses_prior_anchor(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_code_bias_base_only_prev_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_code_bias_base_only_prev.bin"
            direct_path = temp_root / "direct.csv"
            base_only_path = temp_root / "base_only.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518370, iod=3, prn=3, sync=True),
                        build_qzss_cssr_code_bias_message(
                            tow_delta=0,
                            iod=3,
                            bias_m=-0.12,
                            sync=False,
                        ),
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_code_phase_bias_message(
                            tow_delta=0,
                            iod=3,
                            sync=False,
                            network_bias=True,
                            code_bias_exists=True,
                            phase_bias_exists=False,
                            selected_mask=0b1,
                            code_bias_m=0.04,
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
            direct_csv = direct_path.read_text(encoding="ascii")
            direct_network_rows = [
                line
                for line in direct_csv.splitlines()
                if line.startswith("2200,518400.000,G,3")
            ]
            self.assertEqual(len(direct_network_rows), 1)
            self.assertIn("cbias:2=0.040000", direct_network_rows[0])
            self.assertNotIn("cbias:2=-0.120000", direct_network_rows[0])

            base_only_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(base_only_path),
                "--gps-week",
                "2200",
                "--compact-code-bias-composition-policy",
                "base-only-if-present",
                "--compact-code-bias-bank-policy",
                "latest-preceding-bank",
            )
            self.assertEqual(base_only_result.returncode, 0, msg=base_only_result.stderr)
            base_only_csv = base_only_path.read_text(encoding="ascii")
            base_only_network_rows = [
                line
                for line in base_only_csv.splitlines()
                if line.startswith("2200,518400.000,G,3")
            ]
            self.assertEqual(len(base_only_network_rows), 1)
            self.assertIn("cbias:2=-0.120000", base_only_network_rows[0])
            self.assertNotIn("cbias:2=0.040000", base_only_network_rows[0])
    def test_qzss_l6_info_compact_code_bias_bank_policy_close_30s_bank_rejects_older_anchor(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_code_bias_bank_close30_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_code_bias_bank_close30.bin"
            close_30s_path = temp_root / "close_30s.csv"
            preceding_path = temp_root / "latest_preceding.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(tow=518339, iod=3, prn=3, sync=True),
                        build_qzss_cssr_code_bias_message(
                            tow_delta=0,
                            iod=3,
                            bias_m=-0.12,
                            sync=False,
                        ),
                        build_qzss_cssr_mask_message(tow=518400, iod=3, prn=3, sync=True),
                        build_qzss_cssr_code_phase_bias_message(
                            tow_delta=0,
                            iod=3,
                            sync=False,
                            network_bias=True,
                            code_bias_exists=True,
                            phase_bias_exists=False,
                            selected_mask=0b1,
                            code_bias_m=0.04,
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
                "--compact-code-bias-composition-policy",
                "base-plus-network",
                "--compact-code-bias-bank-policy",
                "close-30s-bank",
            )
            self.assertEqual(close_result.returncode, 0, msg=close_result.stderr)
            close_30s_csv = close_30s_path.read_text(encoding="ascii")
            self.assertIn("cbias:2=0.040000", close_30s_csv)
            self.assertNotIn("cbias:2=-0.080000", close_30s_csv)

            preceding_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(preceding_path),
                "--gps-week",
                "2200",
                "--compact-code-bias-composition-policy",
                "base-plus-network",
                "--compact-code-bias-bank-policy",
                "latest-preceding-bank",
            )
            self.assertEqual(preceding_result.returncode, 0, msg=preceding_result.stderr)
            self.assertIn("cbias:2=-0.080000", preceding_path.read_text(encoding="ascii"))
    def test_qzss_l6_info_compact_bias_row_materialization_selected_satellite_extends_missing_code_bias_rows(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_code_bias_row_extend_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_code_bias_row_extend.bin"
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
                        build_qzss_cssr_code_bias_message(
                            tow_delta=0,
                            iod=3,
                            biases_m=(-0.12, -0.08),
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
                            code_bias_exists=True,
                            phase_bias_exists=False,
                            selected_mask=0b1,
                            code_biases_m=(0.04,),
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
                "--compact-code-bias-bank-policy",
                "same-30s-bank",
            )
            self.assertEqual(strict_result.returncode, 0, msg=strict_result.stderr)
            strict_row = next(
                line
                for line in strict_path.read_text(encoding="ascii").splitlines()
                if line.startswith("2200,518400.000") and "bias_network_id=1" in line
            )
            self.assertIn("cbias:2=0.040000", strict_row)
            self.assertNotIn("cbias:3=", strict_row)

            extend_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(extend_path),
                "--gps-week",
                "2200",
                "--compact-code-bias-bank-policy",
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
            self.assertIn("cbias:2=0.040000", extend_row)
            self.assertIn("cbias:3=-0.080000", extend_row)
    def test_qzss_l6_info_compact_bias_row_materialization_extends_phase_only_network_code_bias_rows(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_phase_only_code_bias_row_extend_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_phase_only_code_bias_row_extend.bin"
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
                            sigmask=0x8000,
                        ),
                        build_qzss_cssr_code_bias_message(
                            tow_delta=0,
                            iod=3,
                            bias_m=-0.12,
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
                            phase_bias_m=0.045,
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
                "--compact-code-bias-bank-policy",
                "same-30s-bank",
            )
            self.assertEqual(strict_result.returncode, 0, msg=strict_result.stderr)
            strict_row = next(
                line
                for line in strict_path.read_text(encoding="ascii").splitlines()
                if line.startswith("2200,518400.000") and "bias_network_id=1" in line
            )
            self.assertIn("pbias:2=0.045000", strict_row)
            self.assertNotIn("cbias:2=", strict_row)

            extend_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(extend_path),
                "--gps-week",
                "2200",
                "--compact-code-bias-bank-policy",
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
            self.assertIn("cbias:2=-0.120000", extend_row)
    def test_qzss_l6_info_compact_bias_row_materialization_all_base_satellite_extends_unselected_code_bias_rows(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_code_bias_mask_extend_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_code_bias_mask_extend.bin"
            selected_path = temp_root / "selected_satellite_extend.csv"
            all_sat_path = temp_root / "all_base_satellite_extend.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(
                            tow=518400,
                            iod=3,
                            prns=(3, 4),
                            sync=True,
                            sigmask=0x8000,
                        ),
                        build_qzss_cssr_code_bias_message(
                            tow_delta=0,
                            iod=3,
                            biases_m=(-0.12, -0.22),
                            entry_count=2,
                            sync=False,
                        ),
                        build_qzss_cssr_mask_message(
                            tow=518412,
                            iod=3,
                            prns=(3, 4),
                            sync=True,
                            sigmask=0x8000,
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
                            code_biases_m=(0.04,),
                            entries_per_selected_satellite=1,
                        ),
                    ]
                )
            )

            selected_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(selected_path),
                "--gps-week",
                "2200",
                "--compact-code-bias-bank-policy",
                "same-30s-bank",
                "--compact-bias-row-materialization",
                "selected-satellite-base-extend",
            )
            self.assertEqual(selected_result.returncode, 0, msg=selected_result.stderr)
            selected_csv = selected_path.read_text(encoding="ascii")
            self.assertNotIn("2200,518400.000,G,3,0.000000,0.000000,0.000000,0.000000,0.000000,cbias:2=-0.120000,bias_network_id=1", selected_csv)

            all_sat_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(all_sat_path),
                "--gps-week",
                "2200",
                "--compact-code-bias-bank-policy",
                "same-30s-bank",
                "--compact-bias-row-materialization",
                "all-base-satellite-extend",
            )
            self.assertEqual(all_sat_result.returncode, 0, msg=all_sat_result.stderr)
            all_sat_csv = all_sat_path.read_text(encoding="ascii")
            self.assertIn("2200,518400.000,G,3", all_sat_csv)
            sat3_row = next(
                line
                for line in all_sat_csv.splitlines()
                if line.startswith("2200,518400.000,G,3") and "bias_network_id=1" in line
            )
            self.assertIn("cbias:2=-0.120000", sat3_row)
    def test_qzss_l6_info_compact_row_construction_coupled_code_phase_brings_forward_missing_phase_bias(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_coupled_cp_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_coupled_cp.bin"
            independent_path = temp_root / "independent.csv"
            coupled_path = temp_root / "coupled.csv"
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
                        build_qzss_cssr_mask_message(
                            tow=518430,
                            iod=3,
                            prn=3,
                            sync=True,
                            sigmask=0x8000,
                        ),
                        build_qzss_cssr_code_phase_bias_message(
                            tow_delta=30,
                            iod=3,
                            sync=True,
                            network_bias=True,
                            code_bias_exists=True,
                            phase_bias_exists=False,
                            selected_mask=0b1,
                            code_biases_m=(0.04,),
                            entries_per_selected_satellite=1,
                        ),
                        build_qzss_cssr_combined_message(tow_delta=30, iod=3, prn=3, dclock_m=0.025, sync=False),
                    ]
                )
            )

            independent_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(independent_path),
                "--gps-week",
                "2200",
            )
            self.assertEqual(independent_result.returncode, 0, msg=independent_result.stderr)
            independent_rows = [
                line
                for line in independent_path.read_text(encoding="ascii").splitlines()
                if line.startswith("2200,518430.000") and "cbias:" in line
            ]
            self.assertTrue(len(independent_rows) > 0, "should have correction row at tow 518430")
            independent_row = independent_rows[0]
            self.assertIn("cbias:2=0.040000", independent_row)
            self.assertNotIn("pbias:", independent_row, "phase bias should be absent under independent policy")

            coupled_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(coupled_path),
                "--gps-week",
                "2200",
                "--compact-row-construction-policy",
                "coupled-code-phase",
                "--compact-phase-bias-bank-policy",
                "latest-preceding-bank",
            )
            self.assertEqual(coupled_result.returncode, 0, msg=coupled_result.stderr)
            coupled_rows = [
                line
                for line in coupled_path.read_text(encoding="ascii").splitlines()
                if line.startswith("2200,518430.000") and "cbias:" in line
            ]
            self.assertTrue(len(coupled_rows) > 0, "should have correction row at tow 518430")
            coupled_row = coupled_rows[0]
            self.assertIn("cbias:2=0.040000", coupled_row)
            self.assertIn("pbias:2=0.015000", coupled_row, "phase bias should be brought forward under coupled policy")
    def test_qzss_l6_info_compact_row_construction_network_row_driven_suppresses_base_only_rows(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_qzss_l6_net_row_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_l6_net_row.bin"
            independent_path = temp_root / "independent.csv"
            network_path = temp_root / "network.csv"
            input_path.write_bytes(
                build_qzss_l6_subframe_stream(
                    [
                        build_qzss_cssr_mask_message(
                            tow=518400,
                            iod=3,
                            prns=(3, 7),
                            sync=True,
                            sigmask=0xC000,
                        ),
                        build_qzss_cssr_code_bias_message(
                            tow_delta=0,
                            iod=3,
                            biases_m=(-0.12, -0.08, -0.10, -0.06),
                            entry_count=4,
                            sync=True,
                        ),
                        build_qzss_cssr_code_phase_bias_message(
                            tow_delta=0,
                            iod=3,
                            sync=True,
                            network_bias=True,
                            code_bias_exists=True,
                            phase_bias_exists=True,
                            selected_mask=0b01,
                            satellite_count=2,
                            code_biases_m=(0.04, 0.06),
                            phase_biases_m=(0.015, 0.025),
                            entries_per_selected_satellite=2,
                        ),
                        build_qzss_cssr_combined_message(tow_delta=0, iod=3, prn=3, dclock_m=0.025, sync=False),
                    ]
                )
            )

            independent_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(independent_path),
                "--gps-week",
                "2200",
            )
            self.assertEqual(independent_result.returncode, 0, msg=independent_result.stderr)
            independent_text = independent_path.read_text(encoding="ascii")
            independent_sat3_rows = [
                line for line in independent_text.splitlines()
                if line.startswith("2200,518400.000") and ",G,3," in line
            ]
            self.assertTrue(len(independent_sat3_rows) > 0, "G03 should appear under independent policy")
            self.assertIn("cbias:", independent_sat3_rows[0], "G03 should have code bias under independent policy")

            network_result = self.run_gnss(
                "qzss-l6-info",
                "--input",
                str(input_path),
                "--extract-compact-corrections",
                str(network_path),
                "--gps-week",
                "2200",
                "--compact-row-construction-policy",
                "network-row-driven",
            )
            self.assertEqual(network_result.returncode, 0, msg=network_result.stderr)
            network_text = network_path.read_text(encoding="ascii")
            network_sat3_rows = [
                line for line in network_text.splitlines()
                if line.startswith("2200,518400.000") and ",G,3," in line
            ]
            network_sat7_rows = [
                line for line in network_text.splitlines()
                if line.startswith("2200,518400.000") and ",G,7," in line
            ]
            if network_sat3_rows:
                self.assertNotIn("cbias:", network_sat3_rows[0], "G03 code bias should be suppressed under network-row-driven")
            self.assertTrue(len(network_sat7_rows) > 0, "G07 should appear under network-row-driven policy")
            self.assertIn("cbias:", network_sat7_rows[0], "G07 should have code bias under network-row-driven policy")
