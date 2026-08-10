"""CLI regression cases for the DiagnosticsDataCases domain."""

from ._support import *  # noqa: F401,F403

class DiagnosticsDataCases:
    def test_doctor_cli_emits_json_readiness_report(self) -> None:
        result = self.run_gnss("doctor", "--json")
        self.assertEqual(result.returncode, 0, msg=result.stderr)
        payload = json.loads(result.stdout)
        self.assertIn("checks", payload)
        self.assertIn("next_commands", payload)
        check_names = {item["name"] for item in payload["checks"]}
        self.assertIn("repository root", check_names)
        self.assertIn("gnss_solve", check_names)
        self.assertIn("robotics_quickstart.md", check_names)
    def test_ros2_doctor_cli_emits_field_debug_report(self) -> None:
        result = self.run_gnss(
            "ros2-doctor",
            "--json",
            "--device",
            "/dev/gnsspp-missing-test-device",
        )
        self.assertEqual(result.returncode, 0, msg=result.stderr)
        payload = json.loads(result.stdout)
        self.assertIn("checks", payload)
        self.assertIn("commands", payload)
        self.assertIn("launch", payload["commands"])
        self.assertIn("record", payload["commands"])
        self.assertIn("topic_list", payload["commands"])
        self.assertIn("device:=/dev/gnsspp-missing-test-device", payload["commands"]["launch"])
        check_names = {item["name"] for item in payload["checks"]}
        self.assertIn("serial device", check_names)
        self.assertIn("driver node binary", check_names)
        self.assertIn("launch file", check_names)
    def test_ros2_bag_doctor_cli_reports_topics_rates_and_replayability(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ros2_bag_doctor_") as temp_dir:
            bag_dir = Path(temp_dir) / "bag"
            summary_json = Path(temp_dir) / "ros2_bag_doctor_summary.json"
            build_synthetic_sqlite_rosbag(bag_dir)

            result = self.run_gnss(
                "ros2-bag-doctor",
                "--bag",
                str(bag_dir),
                "--json",
                "--summary-json",
                str(summary_json),
                "--gap-factor",
                "1.0",
                "--gap-min-s",
                "0.5",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(result.stdout)
            self.assertEqual(payload["status"], "ready")
            self.assertTrue(payload["replayable_raw_binary"])
            self.assertEqual(payload["message_count"], 9)
            self.assertEqual(payload["topic_status"]["raw_binary"]["status"], "ok")
            self.assertEqual(payload["topic_status"]["raw"]["status"], "ok")
            self.assertEqual(payload["topic_status"]["fix"]["status"], "ok")
            self.assertEqual(payload["topic_status"]["fix"]["gap_count"], 1)
            self.assertIn("decode", payload["commands"])
            self.assertIn("gnss_bag_processor_node", payload["commands"]["decode"])
            self.assertTrue(summary_json.exists())
            written_summary = json.loads(summary_json.read_text(encoding="utf-8"))
            self.assertEqual(written_summary["topic_count"], 3)
    def test_ros2_bag_doctor_reads_mcap_metadata_without_sqlite(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ros2_bag_doctor_mcap_") as temp_dir:
            bag_dir = Path(temp_dir) / "bag"
            summary_json = Path(temp_dir) / "ros2_bag_doctor_mcap_summary.json"
            build_synthetic_mcap_metadata_rosbag(bag_dir)

            result = self.run_gnss(
                "ros2-bag-doctor",
                "--bag",
                str(bag_dir),
                "--json",
                "--summary-json",
                str(summary_json),
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(result.stdout)
            self.assertEqual(payload["status"], "partial-metadata")
            self.assertEqual(payload["diagnostic_depth"], "metadata")
            self.assertEqual(payload["storage_identifier"], "mcap")
            self.assertTrue(payload["replayable_raw_binary"])
            self.assertEqual(payload["message_count"], 7)
            self.assertEqual(payload["topic_count"], 3)
            self.assertEqual(payload["duration_s"], 4.0)
            self.assertEqual(payload["topic_status"]["raw_binary"]["status"], "ok")
            self.assertEqual(payload["topic_status"]["raw_binary"]["source"], "metadata")
            self.assertIsNone(payload["topic_status"]["fix"]["mean_rate_hz"])
            self.assertIsNone(payload["topic_status"]["fix"]["gap_count"])
            checks = json.dumps(payload["checks"])
            reader_unavailable = "mcap reader unavailable" in checks
            reader_failed = "read failed" in checks
            self.assertTrue(reader_unavailable or reader_failed)
            if reader_unavailable:
                self.assertIn("Install the Python `mcap` package", checks)
            self.assertTrue(summary_json.exists())

            strict_result = self.run_gnss(
                "ros2-bag-doctor",
                "--bag",
                str(bag_dir),
                "--json",
                "--strict",
            )
            self.assertNotEqual(strict_result.returncode, 0)
            self.assertEqual(json.loads(strict_result.stdout)["status"], "partial-metadata")
    def test_ros2_bag_doctor_reads_mcap_messages_when_reader_available(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ros2_bag_doctor_mcap_reader_") as temp_dir:
            temp_root = Path(temp_dir)
            bag_dir = temp_root / "bag"
            fake_package_root = temp_root / "fake_python"
            build_synthetic_mcap_metadata_rosbag(bag_dir)
            write_fake_mcap_messages(bag_dir)
            write_fake_mcap_reader_package(fake_package_root)

            result = self.run_gnss(
                "ros2-bag-doctor",
                "--bag",
                str(bag_dir),
                "--json",
                "--strict",
                "--gap-factor",
                "1.0",
                "--gap-min-s",
                "0.5",
                extra_env={"PYTHONPATH": str(fake_package_root)},
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(result.stdout)
            self.assertEqual(payload["status"], "ready")
            self.assertEqual(payload["diagnostic_depth"], "message-data")
            self.assertEqual(payload["message_source"], "mcap")
            self.assertEqual(payload["storage_identifier"], "mcap")
            self.assertEqual(payload["message_count"], 7)
            self.assertEqual(payload["topic_count"], 3)
            self.assertEqual(payload["duration_s"], 3.0)
            self.assertEqual(payload["topic_status"]["raw_binary"]["source"], "mcap")
            self.assertEqual(payload["topic_status"]["fix"]["gap_count"], 1)
            self.assertIn("rates and timestamp gaps inspected from MCAP messages", json.dumps(payload["checks"]))
    def test_ros2_bag_doctor_strict_fails_without_raw_binary(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ros2_bag_doctor_missing_") as temp_dir:
            bag_dir = Path(temp_dir) / "bag"
            build_synthetic_sqlite_rosbag(bag_dir, include_raw_binary=False)

            result = self.run_gnss(
                "ros2-bag-doctor",
                "--bag",
                str(bag_dir),
                "--json",
                "--strict",
            )

            self.assertNotEqual(result.returncode, 0)
            payload = json.loads(result.stdout)
            self.assertEqual(payload["status"], "partial")
            self.assertFalse(payload["replayable_raw_binary"])
            self.assertEqual(payload["topic_status"]["raw_binary"]["status"], "missing")
    def test_field_report_aggregates_doctors_bag_and_robotics_summaries(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_field_report_") as temp_dir:
            temp_root = Path(temp_dir)
            output_dir = temp_root / "output"
            bag_summary = output_dir / "ros2_bag_doctor_summary.json"
            robotics_summary = output_dir / "robotics_smoke" / "realtime.json"
            report_md = output_dir / "field_report.md"
            report_json = output_dir / "field_report.json"

            (temp_root / "apps").mkdir(parents=True)
            (temp_root / "apps" / "gnss.py").write_text("# synthetic dispatcher\n", encoding="utf-8")
            (temp_root / "CMakeLists.txt").write_text("cmake_minimum_required(VERSION 3.16)\n", encoding="utf-8")
            (temp_root / "docs").mkdir()
            for name in ("robotics_quickstart.md", "research_quickstart.md", "dataset_gallery.md"):
                (temp_root / "docs" / name).write_text("# synthetic\n", encoding="utf-8")
            (temp_root / "ros2" / "gnss_raw_driver" / "launch").mkdir(parents=True)
            (temp_root / "ros2" / "gnss_raw_driver" / "launch" / "gnss_raw_driver.launch.py").write_text(
                "# synthetic launch\n",
                encoding="utf-8",
            )
            (temp_root / "ros2" / "gnss_raw_driver" / "msg").mkdir(parents=True)
            (temp_root / "ros2" / "gnss_raw_driver" / "msg" / "GnssRawEpoch.msg").write_text(
                "builtin_interfaces/Time stamp\n",
                encoding="utf-8",
            )
            (temp_root / "ros2" / "gnss_raw_driver" / "msg" / "GnssRawObservation.msg").write_text(
                "float64 pseudorange_m\n",
                encoding="utf-8",
            )

            bag_summary.parent.mkdir(parents=True)
            bag_summary.write_text(
                json.dumps(
                    {
                        "tool": "ros2-bag-doctor",
                        "bag": str(temp_root / "field_bag"),
                        "status": "ready",
                        "replayable_raw_binary": True,
                        "message_count": 9,
                        "topic_count": 3,
                        "duration_s": 3.0,
                        "topic_status": {
                            "raw_binary": {"status": "ok", "message_count": 3},
                            "fix": {"status": "ok", "message_count": 3},
                        },
                        "topics": [
                            {"name": "/gnss/raw_binary", "message_count": 3, "gap_count": 0},
                            {"name": "/gnss/fix", "message_count": 3, "gap_count": 1},
                        ],
                        "commands": {"decode": "ros2 run gnss_raw_driver gnss_bag_processor_node"},
                    }
                ),
                encoding="utf-8",
            )
            robotics_summary.parent.mkdir(parents=True)
            robotics_summary.write_text(
                json.dumps(
                    {
                        "robotics_smoke_status": "passed",
                        "robotics_smoke_profile": "realtime",
                        "dataset": "synthetic",
                        "matched_epochs": 200,
                        "fix_rate_pct": 98.0,
                        "positioning_rate_pct": 1.67,
                        "realtime_factor": 1.25,
                        "effective_epoch_rate_hz": 6.0,
                        "solver_wall_time_s": 32.0,
                    }
                ),
                encoding="utf-8",
            )

            result = self.run_gnss(
                "field-report",
                "--root",
                str(temp_root),
                "--out",
                str(report_md),
                "--json-out",
                str(report_json),
                "--device",
                "/dev/gnsspp-missing-test-device",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertTrue(report_md.exists())
            self.assertTrue(report_json.exists())
            markdown = report_md.read_text(encoding="utf-8")
            self.assertIn("libgnss++ Field Report", markdown)
            self.assertIn("ROS2 Bag Diagnostics", markdown)
            self.assertIn("Robotics Realtime Smoke", markdown)
            self.assertIn("ros2_bag_doctor_summary.json", markdown)
            self.assertIn("robotics_smoke/realtime.json", markdown)
            payload = json.loads(report_json.read_text(encoding="utf-8"))
            self.assertEqual(payload["tool"], "field-report")
            self.assertEqual(len(payload["ros2_bags"]), 1)
            self.assertEqual(len(payload["robotics_smoke"]), 1)
            self.assertTrue(payload["ros2_bags"][0]["replayable_raw_binary"])
            self.assertIn("python3 apps/gnss.py web", "\n".join(payload["next_actions"]))
    def test_robotics_smoke_dry_run_includes_realtime_gates(self) -> None:
        result = self.run_gnss(
            "robotics-smoke",
            "--dry-run",
            "--max-epochs",
            "5",
            "--realtime-factor-min",
            "1.0",
            "--effective-epoch-rate-min",
            "5.0",
        )
        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("ppc-rtk-signoff", result.stdout)
        self.assertIn("--require-realtime-factor-min 1.0", result.stdout)
        self.assertIn("--require-effective-epoch-rate-min 5.0", result.stdout)
        self.assertIn("profile: realtime", result.stdout)
        self.assertIn("max_epochs: 5", result.stdout)

        quick = self.run_gnss("robotics-smoke", "--dry-run", "--profile", "quick")
        self.assertEqual(quick.returncode, 0, msg=quick.stderr)
        self.assertIn("profile: quick", quick.stdout)
        self.assertIn("max_epochs: 50", quick.stdout)
        self.assertIn("--require-solver-wall-time-max 120.0", quick.stdout)
    def test_rinex_info_reports_observation_header_and_epoch_count(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_rinex_info_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            obs_path, _, _, _ = build_synthetic_ppp_inputs(temp_root)

            result = self.run_gnss("rinex-info", "--count-records", str(obs_path))

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("type: observation", result.stdout)
            self.assertIn("marker: TESTMARK", result.stdout)
            self.assertIn("epoch count: 8", result.stdout)
            self.assertIn("total observation records: 96", result.stdout)
    def test_fetch_products_cli_copies_and_decompresses_local_template(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_fetch_products_local_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            source_dir = temp_root / "source"
            source_dir.mkdir()
            compressed_path = source_dir / "2024002.sp3.gz"
            with gzip.open(compressed_path, "wb") as stream:
                stream.write(b"TEST SP3\n")

            cache_dir = temp_root / "cache"
            summary_path = temp_root / "products.json"
            product_template = str(source_dir / "{yyyy}{doy}.sp3.gz")

            result = self.run_gnss(
                "fetch-products",
                "--date",
                "2024-01-02",
                "--product",
                f"sp3={product_template}",
                "--cache-dir",
                str(cache_dir),
                "--summary-json",
                str(summary_path),
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(result.stdout)
            self.assertEqual(payload["status_counts"], {"copied": 1})
            self.assertEqual(set(payload["products"]), {"sp3"})
            destination = Path(payload["products"]["sp3"])
            self.assertTrue(destination.exists())
            self.assertEqual(destination.read_bytes(), b"TEST SP3\n")
            self.assertFalse(destination.name.endswith(".gz"))
            self.assertEqual(json.loads(summary_path.read_text(encoding="utf-8"))["products"]["sp3"], str(destination))

            cached = self.run_gnss(
                "fetch-products",
                "--date",
                "2024-01-02",
                "--product",
                f"sp3={product_template}",
                "--cache-dir",
                str(cache_dir),
            )
            self.assertEqual(cached.returncode, 0, msg=cached.stderr)
            cached_payload = json.loads(cached.stdout)
            self.assertEqual(cached_payload["results"][0]["status"], "cached")
    def test_fetch_products_cli_downloads_http_template(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_fetch_products_http_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            source_dir = temp_root / "source"
            source_dir.mkdir()
            compressed_path = source_dir / "24002.clk.gz"
            with gzip.open(compressed_path, "wb") as stream:
                stream.write(b"TEST CLK\n")

            server, thread, port = start_static_http_server(source_dir)
            try:
                cache_dir = temp_root / "cache"
                summary_path = temp_root / "products.json"
                result = self.run_gnss(
                    "fetch-products",
                    "--date",
                    "2024-01-02",
                    "--product",
                    f"clk=http://127.0.0.1:{port}/{{yy}}{{doy}}.clk.gz",
                    "--cache-dir",
                    str(cache_dir),
                    "--summary-json",
                    str(summary_path),
                )
            finally:
                server.shutdown()
                thread.join(timeout=2.0)
                server.server_close()

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(result.stdout)
            self.assertEqual(payload["status_counts"], {"downloaded": 1})
            destination = Path(payload["products"]["clk"])
            self.assertTrue(destination.exists())
            self.assertEqual(destination.read_bytes(), b"TEST CLK\n")
            self.assertEqual(payload["results"][0]["resolved_source"], f"http://127.0.0.1:{port}/24002.clk.gz")
            self.assertEqual(json.loads(summary_path.read_text(encoding="utf-8"))["products"]["clk"], str(destination))
    def test_fetch_products_cli_lists_presets_and_supports_dry_run(self) -> None:
        presets = self.run_gnss("fetch-products", "--list-presets")
        self.assertEqual(presets.returncode, 0, msg=presets.stderr)
        self.assertIn("igs-final", presets.stdout)
        self.assertIn("ionex", presets.stdout)
        self.assertIn("dcb", presets.stdout)
        self.assertIn("brdc-nav", presets.stdout)

        with tempfile.TemporaryDirectory(prefix="gnss_fetch_products_dry_run_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            cache_dir = temp_root / "cache"
            summary_path = temp_root / "products.json"
            dry_run = self.run_gnss(
                "fetch-products",
                "--date",
                "2024-01-02",
                "--preset",
                "igs-final",
                "--preset",
                "ionex",
                "--preset",
                "dcb",
                "--cache-dir",
                str(cache_dir),
                "--summary-json",
                str(summary_path),
                "--dry-run",
            )
            self.assertEqual(dry_run.returncode, 0, msg=dry_run.stderr)
            payload = json.loads(dry_run.stdout)
            self.assertTrue(payload["dry_run"])
            self.assertEqual(payload["presets"], ["igs-final", "ionex", "dcb"])
            self.assertEqual(set(payload["products"]), {"sp3", "clk", "ionex", "dcb"})
            self.assertEqual(payload["status_counts"], {"dry-run": 4})
            self.assertIn("COD0OPSFIN_20240020000_01D_05M_ORB.SP3.gz", payload["results"][0]["resolved_source"])
            self.assertIn("CAS0MGXRAP_20240020000_01D_01D_DCB.BSX.gz", json.dumps(payload))
            self.assertEqual(
                json.loads(summary_path.read_text(encoding="utf-8"))["products"]["ionex"],
                payload["products"]["ionex"],
            )
    def test_ionex_info_reports_header_and_map_counts(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ionex_info_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            ionex_path = temp_root / "sample.ionex"
            summary_path = temp_root / "sample_ionex.json"
            ionex_path.write_text(
                "\n".join(
                    [
                        "     1.0           I                   G                   IONEX VERSION / TYPE",
                        "  2024     1     2     0     0     0                        EPOCH OF FIRST MAP",
                        "  2024     1     2     1     0     0                        EPOCH OF LAST MAP",
                        "    3600                                                      INTERVAL",
                        "       2                                                    # OF MAPS IN FILE",
                        "       2                                                    MAP DIMENSION",
                        "  6371.0                                                    BASE RADIUS",
                        "     10.0                                                   ELEVATION CUTOFF",
                        "COSZ                                                        MAPPING FUNCTION",
                        "      -1                                                    EXPONENT",
                        "   -87.5    87.5     2.5                                    LAT1 / LAT2 / DLAT",
                        "  -180.0   180.0     5.0                                    LON1 / LON2 / DLON",
                        "   450.0   450.0     0.0                                    HGT1 / HGT2 / DHGT",
                        "G01     1.234     0.100                                     PRN / BIAS / RMS",
                        "                                                            END OF HEADER",
                        "     1                                                      START OF TEC MAP",
                        "  2024     1     2     0     0     0                        EPOCH OF CURRENT MAP",
                        "     1                                                      END OF TEC MAP",
                        "     2                                                      START OF TEC MAP",
                        "  2024     1     2     1     0     0                        EPOCH OF CURRENT MAP",
                        "     2                                                      END OF TEC MAP",
                        "     1                                                      START OF RMS MAP",
                        "     1                                                      END OF RMS MAP",
                    ]
                )
                + "\n",
                encoding="ascii",
            )

            result = self.run_gnss(
                "ionex-info",
                "--input",
                str(ionex_path),
                "--summary-json",
                str(summary_path),
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("format: IONEX", result.stdout)
            self.assertIn("maps: 2", result.stdout)
            self.assertIn("rms maps: 1", result.stdout)
            self.assertIn("aux dcb entries: 1", result.stdout)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["map_count"], 2)
            self.assertEqual(payload["rms_map_count"], 1)
            self.assertEqual(payload["aux_dcb_count"], 1)
            self.assertEqual(payload["system"], "G")
    def test_dcb_info_reports_bias_sinex_entries(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_dcb_info_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            dcb_path = temp_root / "sample.BSX"
            summary_path = temp_root / "sample_dcb.json"
            dcb_path.write_text(
                "\n".join(
                    [
                        "%=BIA 1.00 TEST TEST 2024:002:00000 TEST",
                        "+BIAS/SOLUTION",
                        "*BIAS SVN PRN STATION OBS1 OBS2 BEGIN END UNIT EST STDDEV",
                        " DSB G01 C1C C2W 2024:002:00000 2024:003:00000 ns 1.234 0.100",
                        " OSB E11 C1C C5Q 2024:002:00000 2024:003:00000 ns 0.321 0.050",
                        "-BIAS/SOLUTION",
                    ]
                )
                + "\n",
                encoding="ascii",
            )

            result = self.run_gnss(
                "dcb-info",
                "--input",
                str(dcb_path),
                "--summary-json",
                str(summary_path),
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("format: SINEX_BIAS", result.stdout)
            self.assertIn("entries: 2", result.stdout)
            self.assertIn("bias types: DSB, OSB", result.stdout)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["entry_count"], 2)
            self.assertEqual(payload["bias_types"], ["DSB", "OSB"])
            self.assertEqual(payload["systems"], ["E", "G"])
