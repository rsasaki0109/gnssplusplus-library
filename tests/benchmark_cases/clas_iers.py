"""CLAS, IERS, and correction helper benchmark test cases."""

from ._support import *  # noqa: F401,F403

__all__ = [
    "ClasMrtklibV051RunnerTest",
    "IersMultisiteBenchHelpersTest",
    "IersAtmTidalLoadingMultisiteBenchHelpersTest",
    "VmfAtlHelpersTest",
    "ClasCompactHelpersTest",
]


class ClasMrtklibV051RunnerTest(unittest.TestCase):
    def test_dynamic_profile_keeps_fixture_identity_and_dynamics_flag(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnsspp_clas_v051_") as temp_dir:
            root = Path(temp_dir)
            for name in ("gnss_ppp", "rover.obs", "base.nav"):
                (root / name).write_text("fixture\n", encoding="ascii")
            ssr = root / "expanded.csv"
            ssr.write_text("# fixture\n", encoding="ascii")
            config = {
                "runner": {
                    "gnss_ppp": "gnss_ppp",
                    "env": {"PARITY_GATE": "1"},
                },
                "profiles": {
                    "dynamic": {
                        "obs": "rover.obs",
                        "nav": "base.nav",
                        "ssr": "expanded.csv",
                        "ssr_md5": clas_v051_runner.file_md5(ssr),
                        "out": "result.pos",
                        "env": ["PARITY_GATE"],
                        "args": ["--kinematic", "--use-dynamics-model"],
                    }
                },
            }
            with mock.patch.object(clas_v051_runner, "ROOT_DIR", root):
                command, env, out = clas_v051_runner.profile_command(config, "dynamic")

            self.assertIn("--use-dynamics-model", command)
            self.assertEqual(env["PARITY_GATE"], "1")
            self.assertEqual(out, root / "result.pos")


class IersMultisiteBenchHelpersTest(unittest.TestCase):
    def test_resolve_config_path_uses_site_config_directory(self) -> None:
        config_dir = Path("/tmp/gnsspp/site-config")

        self.assertEqual(
            iers_multisite_bench.resolve_config_path(
                "data/igs_2026105/PERT00AUS.rnx", config_dir),
            config_dir / "data/igs_2026105/PERT00AUS.rnx",
        )
        self.assertEqual(
            iers_multisite_bench.resolve_config_path(
                "/var/tmp/PERT00AUS.rnx", config_dir),
            Path("/var/tmp/PERT00AUS.rnx"),
        )

    def test_resolve_config_path_falls_back_to_parent_relative_path(
        self,
    ) -> None:
        with tempfile.TemporaryDirectory(prefix="gnsspp_multisite_config_") as temp_dir:
            root = Path(temp_dir)
            config_dir = root / "data" / "igs_2026105"
            obs = root / "data" / "igs_2026105" / "PERT00AUS.rnx"
            obs.parent.mkdir(parents=True)
            obs.touch()

            self.assertEqual(
                iers_multisite_bench.resolve_config_path(
                    "data/igs_2026105/PERT00AUS.rnx", config_dir),
                obs,
            )

    def test_resolve_config_path_falls_back_to_source_root(self) -> None:
        self.assertEqual(
            iers_multisite_bench.resolve_config_path(
                "test_data/iers/tskb_synth.atl", Path("/tmp")),
            ROOT_DIR / "test_data" / "iers" / "tskb_synth.atl",
        )


class IersAtmTidalLoadingMultisiteBenchHelpersTest(unittest.TestCase):
    def test_smoke_example_config_references_tracked_synthetic_atl_fixture(self) -> None:
        config_path = ROOT_DIR / "configs" / "benchmarks" / "iers_atl_multisite_smoke.example.json"
        config = json.loads(config_path.read_text(encoding="utf-8"))

        self.assertEqual(config["sites"][0]["name"], "TSKB")
        atl_path = ROOT_DIR / config["sites"][0]["atm_tidal_loading"]
        self.assertTrue(atl_path.exists())
        self.assertIn("S1", atl_path.read_text(encoding="ascii"))

    def test_vmf_example_config_references_tracked_real_atl_fixtures(self) -> None:
        config_path = ROOT_DIR / "configs" / "benchmarks" / "iers_atl_multisite_vmf.example.json"
        config = json.loads(config_path.read_text(encoding="utf-8"))

        self.assertEqual([site["name"] for site in config["sites"]], ["PERT", "TSKB"])
        for site in config["sites"]:
            atl_path = ROOT_DIR / site["atm_tidal_loading"]
            text = atl_path.read_text(encoding="ascii")
            self.assertTrue(atl_path.exists())
            self.assertIn(site["name"], text)
            self.assertIn("Source: https://vmf.geo.tuwien.ac.at", text)

    def test_resolve_config_paths_resolves_common_and_site_paths(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnsspp_atl_multisite_") as temp_dir:
            root = Path(temp_dir)
            config_dir = root / "configs"
            config_dir.mkdir()
            nav = root / "data" / "BRDC.rnx"
            obs = root / "data" / "PERT00AUS.rnx"
            atl = root / "data" / "PERT.atl"
            nav.parent.mkdir()
            for path in (nav, obs, atl):
                path.touch()
            config = {
                "common": {"nav": "data/BRDC.rnx"},
                "sites": [
                    {
                        "name": "PERT",
                        "obs": "data/PERT00AUS.rnx",
                        "atm_tidal_loading": "data/PERT.atl",
                    }
                ],
            }

            iers_atl_multisite_bench.resolve_config_paths(config, config_dir)

            self.assertEqual(config["common"]["nav"], nav)
            self.assertEqual(config["sites"][0]["obs"], obs)
            self.assertEqual(config["sites"][0]["atm_tidal_loading"], atl)

    def test_site_value_prefers_site_over_common(self) -> None:
        site = {"atm_tidal_loading": Path("site.atl")}
        common = {"atm_tidal_loading": Path("common.atl")}

        self.assertEqual(
            iers_atl_multisite_bench.site_value(
                site, common, "atm_tidal_loading"),
            Path("site.atl"),
        )
        self.assertEqual(
            iers_atl_multisite_bench.site_value(site, common, "nav"),
            None,
        )

    def test_resolve_config_path_falls_back_to_source_root(self) -> None:
        self.assertEqual(
            iers_atl_multisite_bench.resolve_config_path(
                "test_data/iers/tskb_synth.atl", Path("/tmp")),
            ROOT_DIR / "test_data" / "iers" / "tskb_synth.atl",
        )


class VmfAtlHelpersTest(unittest.TestCase):
    def sample_vmf_text(self) -> str:
        values = " ".join(str(float(i)) for i in range(1, 19))
        return f"PERT {values}\n"

    def test_amplitude_phase_matches_cos_sin_form(self) -> None:
        amplitude_m, phase_deg = vmf_atl.amplitude_phase_m(3.0, 4.0)

        self.assertAlmostEqual(amplitude_m, 0.005)
        self.assertAlmostEqual(phase_deg, 53.13010235415598)

    def test_convert_row_flips_east_north_to_west_south(self) -> None:
        # radial S1=(3,4), S2=(0,5); east S1=(1,0), S2=(0,2);
        # north S1=(0,1), S2=(2,0). All values are VMF mm cos/sin.
        values = [
            3.0, 4.0, 0.0, 5.0, 0.0, 0.0,
            1.0, 0.0, 0.0, 2.0, 0.0, 0.0,
            0.0, 1.0, 2.0, 0.0, 0.0, 0.0,
        ]

        text = vmf_atl.convert_row("PERT", values)

        self.assertIn("PERT", text)
        self.assertIn("S1  0.00500000  0.00100000  0.00100000", text)
        self.assertIn("S2  0.00500000  0.00200000  0.00200000", text)

    def test_parse_vmf_rows_uppercases_station_names(self) -> None:
        rows = vmf_atl.parse_vmf_rows("pert " + " ".join(["1"] * 18))

        self.assertIn("PERT", rows)
        self.assertEqual(len(rows["PERT"]), 18)

    def test_main_writes_local_source_station_file(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnsspp_vmf_atl_") as temp_dir:
            temp_root = Path(temp_dir)
            source = temp_root / "vmf.dat"
            out_dir = temp_root / "out"
            source.write_text(self.sample_vmf_text(), encoding="ascii")

            with mock.patch.object(
                sys,
                "argv",
                [
                    "gnss_vmf_atl.py",
                    "--source", str(source),
                    "--station", "pert",
                    "--output-dir", str(out_dir),
                    "--suffix", "unit",
                ],
            ):
                rc = vmf_atl.main()

            output = out_dir / "pert_unit.atl"
            self.assertEqual(rc, 0)
            self.assertTrue(output.exists())
            self.assertIn("PERT", output.read_text(encoding="ascii"))

    def test_main_reports_missing_station(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnsspp_vmf_atl_missing_") as temp_dir:
            temp_root = Path(temp_dir)
            source = temp_root / "vmf.dat"
            source.write_text(self.sample_vmf_text(), encoding="ascii")

            with mock.patch.object(
                sys,
                "argv",
                [
                    "gnss_vmf_atl.py",
                    "--source", str(source),
                    "--station", "NOPE",
                    "--output-dir", str(temp_root / "out"),
                ],
            ):
                rc = vmf_atl.main()

            self.assertEqual(rc, 1)


class ClasCompactHelpersTest(unittest.TestCase):
    def test_expand_compact_ssr_text_merges_high_rate_clock_and_system_tokens(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_compact_") as temp_dir:
            output_csv = Path(temp_dir) / "expanded.csv"
            payload = clas_ppp.expand_compact_ssr_text(
                "\n".join(
                    [
                        "# week,tow,system,prn,dx,dy,dz,dclock_m,high_rate_clock_m",
                        "2200,345600.0,G,3,0.1,0.2,0.3,0.4,0.05",
                        "2200,345600.0,QZSS,3,0.0,0.0,0.0,0.1",
                    ]
                ),
                output_csv,
            )

            self.assertEqual(payload["rows_written"], 2)
            self.assertEqual(payload["systems"], ["G", "J"])
            lines = output_csv.read_text(encoding="ascii").splitlines()
            self.assertEqual(lines[0], "# week,tow,sat,dx,dy,dz,dclock_m")
            self.assertIn("2200,345600.000,G03,0.100000,0.200000,0.300000,0.450000", lines[1])
            self.assertIn("2200,345600.000,J03,0.000000,0.000000,0.000000,0.100000", lines[2])

    def test_expand_compact_ssr_text_preserves_optional_ura_code_and_phase_bias_tokens(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_compact_meta_") as temp_dir:
            output_csv = Path(temp_dir) / "expanded.csv"
            clas_ppp.expand_compact_ssr_text(
                "\n".join(
                    [
                        "# week,tow,system,prn,dx,dy,dz,dclock_m,high_rate_clock_m,ura_sigma_m=<m>,cbias:<id>=<m>,pbias:<id>=<m>",
                        "2200,345600.0,G,3,0.1,0.2,0.3,0.4,0.05,ura_sigma_m=0.002750,cbias:2=-0.120000,pbias:2=0.015000",
                    ]
                ),
                output_csv,
            )

            lines = output_csv.read_text(encoding="ascii").splitlines()
            self.assertEqual(lines[0], "# week,tow,sat,dx,dy,dz,dclock_m")
            self.assertEqual(
                lines[1],
                "2200,345600.000,G03,0.100000,0.200000,0.300000,0.450000,ura_sigma_m=0.002750,cbias:2=-0.120000,pbias:2=0.015000",
            )

    def test_expand_compact_ssr_text_preserves_atmos_metadata_tokens(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_compact_atmos_") as temp_dir:
            output_csv = Path(temp_dir) / "expanded.csv"
            clas_ppp.expand_compact_ssr_text(
                "\n".join(
                    [
                        "# week,tow,system,prn,dx,dy,dz,dclock_m,atmos_<name>=<value>",
                        "2200,345600.0,G,3,0.1,0.2,0.3,0.4,atmos_network_id=1,atmos_trop_avail=3,atmos_stec_avail=3",
                    ]
                ),
                output_csv,
            )

            lines = output_csv.read_text(encoding="ascii").splitlines()
            self.assertEqual(
                lines[1],
                "2200,345600.000,G03,0.100000,0.200000,0.300000,0.400000,atmos_network_id=1,atmos_trop_avail=3,atmos_stec_avail=3",
            )

    def test_expand_compact_ssr_text_normalizes_legacy_qzss_and_iode(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_compact_qzss_") as temp_dir:
            output_csv = Path(temp_dir) / "expanded.csv"
            clas_ppp.expand_compact_ssr_text(
                "2200,345600.0,S,121,0.1,0.2,0.3,0.4,0.0,0,"
                "orbit_iode=32,atmos_stec_c00_tecu:S121=12.5,"
                "atmos_stec_satellites=G03;S121;S122",
                output_csv,
            )

            self.assertEqual(
                output_csv.read_text(encoding="ascii").splitlines()[1],
                "2200,345600.000,J02,0.100000,0.200000,0.300000,0.400000,"
                "orbit_iode=32,atmos_stec_c00_tecu:J02=12.5,"
                "atmos_stec_satellites=G03;J02;J03",
            )

    def test_parse_ppp_summary_counts_extracts_atmospheric_lines(self) -> None:
        parsed = clas_ppp._parse_ppp_summary_counts(
            "\n".join(
                [
                    "PPP summary:",
                    "  valid solutions: 4",
                    "  atmospheric trop corrections: 12",
                    "  atmospheric trop meters: 5.500000",
                    "  atmospheric ionosphere corrections: 8",
                    "  atmospheric ionosphere meters: 3.250000",
                ]
            )
        )
        self.assertEqual(parsed["ppp_atmospheric_trop_corrections"], 12)
        self.assertEqual(parsed["ppp_atmospheric_ionosphere_corrections"], 8)
        self.assertAlmostEqual(float(parsed["ppp_atmospheric_trop_meters"]), 5.5)
        self.assertAlmostEqual(float(parsed["ppp_atmospheric_ionosphere_meters"]), 3.25)

    def test_build_summary_payload_marks_clas_osr_profile(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_summary_") as temp_dir:
            temp_root = Path(temp_dir)
            pos_path = temp_root / "solution.pos"
            pos_path.write_text(
                "\n".join(
                    [
                        "% synthetic solution",
                        "2200 345600.000 1.0 2.0 3.0 0.0 0.0 0.0 5 8",
                        "2200 345630.000 1.0 2.0 3.0 0.0 0.0 0.0 6 9",
                    ]
                )
                + "\n",
                encoding="ascii",
            )

            common_args = {
                "obs": Path("rover.obs"),
                "nav": Path("nav.rnx"),
                "sp3": None,
                "clk": None,
                "antex": None,
                "receiver_antenna_type": None,
                "ssr_rtcm": None,
                "compact_ssr": "corrections.compact.csv",
                "qzss_l6": None,
                "out": pos_path,
                "summary_json": None,
                "enable_ar": False,
                "ar_ratio_threshold": 3.0,
                "kinematic": False,
                "compact_atmos_merge_policy": "stec-coeff-carry",
                "compact_atmos_subtype_merge_policy": "union",
                "compact_phase_bias_merge_policy": "latest-union",
                "compact_phase_bias_source_policy": "arrival-order",
                "compact_code_bias_composition_policy": "direct-values",
                "compact_code_bias_bank_policy": "pending-epoch",
                "compact_phase_bias_composition_policy": "direct-values",
                "compact_phase_bias_bank_policy": "pending-epoch",
                "compact_bias_row_materialization": "overlap-only",
                "compact_row_construction_policy": "independent",
            }

            clas_payload = clas_ppp.build_summary_payload(
                argparse.Namespace(profile="clas", **common_args)
            )
            madoca_payload = clas_ppp.build_summary_payload(
                argparse.Namespace(profile="madoca", **common_args)
            )

            self.assertTrue(clas_payload["clas_osr_filter_enabled"])
            self.assertFalse(madoca_payload["clas_osr_filter_enabled"])
            self.assertIsNone(clas_payload["antex"])
            self.assertIsNone(madoca_payload["antex"])
            self.assertIsNone(clas_payload["receiver_antenna_type"])
            self.assertIsNone(madoca_payload["receiver_antenna_type"])

    def test_main_forwards_antex_to_ppp_and_records_summary(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_antex_forward_") as temp_dir:
            temp_root = Path(temp_dir)
            obs_path = temp_root / "rover.obs"
            nav_path = temp_root / "nav.rnx"
            compact_path = temp_root / "corrections.compact.csv"
            antex_path = temp_root / "receiver.atx"
            output_path = temp_root / "solution.pos"
            summary_path = temp_root / "summary.json"
            obs_path.write_text("synthetic obs placeholder\n", encoding="ascii")
            nav_path.write_text("synthetic nav placeholder\n", encoding="ascii")
            compact_path.write_text(
                "\n".join(
                    [
                        "# week,tow,system,prn,dx,dy,dz,dclock_m",
                        "2200,345600.0,G,3,0.0,0.0,0.0,0.0",
                    ]
                )
                + "\n",
                encoding="ascii",
            )
            antex_path.write_text("synthetic antex placeholder\n", encoding="ascii")
            commands: list[list[str]] = []

            def fake_run_command(command: list[str]) -> mock.Mock:
                commands.append(command)
                output_path.write_text(
                    "% synthetic solution\n"
                    "2200 345600.000 1.0 2.0 3.0 0.0 0.0 0.0 5 8\n",
                    encoding="ascii",
                )
                return mock.Mock(stdout="PPP summary:\n  valid solutions: 1\n")

            argv = [
                "gnss_clas_ppp.py",
                "--profile",
                "clas",
                "--obs",
                str(obs_path),
                "--nav",
                str(nav_path),
                "--compact-ssr",
                str(compact_path),
                "--antex",
                str(antex_path),
                "--receiver-antenna-type",
                "TEST-ANT",
                "--clas-atmos-selection",
                "freshness-first",
                "--clas-atmos-stale-after-seconds",
                "12.5",
                "--out",
                str(output_path),
                "--summary-json",
                str(summary_path),
            ]
            with (
                mock.patch.object(sys, "argv", argv),
                mock.patch.object(clas_ppp, "resolve_gnss_command", return_value=["gnss"]),
                mock.patch.object(clas_ppp, "run_command", side_effect=fake_run_command),
            ):
                self.assertEqual(clas_ppp.main(), 0)

            self.assertEqual(len(commands), 1)
            command = commands[0]
            self.assertIn("--antex", command)
            self.assertEqual(command[command.index("--antex") + 1], str(antex_path))
            self.assertIn("--receiver-antenna-type", command)
            self.assertEqual(command[command.index("--receiver-antenna-type") + 1], "TEST-ANT")
            self.assertIn("--clas-atmos-selection", command)
            self.assertEqual(command[command.index("--clas-atmos-selection") + 1], "freshness-first")
            self.assertIn("--clas-atmos-stale-after-seconds", command)
            self.assertEqual(command[command.index("--clas-atmos-stale-after-seconds") + 1], "12.5")
            self.assertIn("--clas-osr", command)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["antex"], str(antex_path))
            self.assertEqual(payload["receiver_antenna_type"], "TEST-ANT")
