"""CLI regression cases for the StreamProtocolCases domain."""

import ast

from ._support import *  # noqa: F401,F403


def _sys_path_access_lines(source: str, *, filename: str = "<string>") -> list[int]:
    tree = ast.parse(source, filename=filename)
    nodes = tuple(ast.walk(tree))
    sys_names = {
        imported.asname or imported.name
        for node in nodes
        if isinstance(node, ast.Import)
        for imported in node.names
        if imported.name == "sys"
    }
    return sorted(
        {
            node.lineno
            for node in nodes
            if (
                isinstance(node, ast.Attribute)
                and node.attr == "path"
                and isinstance(node.value, ast.Name)
                and node.value.id in sys_names
            )
            or (
                isinstance(node, ast.ImportFrom)
                and node.module == "sys"
                and any(imported.name == "path" for imported in node.names)
            )
        }
    )


class StreamProtocolCases:
    def test_sys_path_access_detector_covers_aliases_and_targets(self) -> None:
        cases = (
            ("module alias", "import sys as s\ns.path.insert(0, 'support')\n", [2]),
            ("from-import alias", "from sys import path as p\np.append('support')\n", [1]),
            ("assigned path alias", "import sys\np = sys.path\np.append('support')\n", [2]),
            ("recursive targets", "import sys\nx, sys.path = values\ndel (y, sys.path)\n", [2, 3]),
            (
                "non-path sys attributes",
                (
                    "import sys as s\n"
                    "data = s.stdin.read()\n"
                    "s.stderr.write(data)\n"
                    "command = [s.executable]\n"
                ),
                [],
            ),
        )

        for name, source, expected_lines in cases:
            with self.subTest(name=name):
                self.assertEqual(_sys_path_access_lines(source), expected_lines)

    def test_receiver_commands_do_not_access_sys_path(self) -> None:
        receiver_dir = ROOT_DIR / "apps" / "commands" / "receivers"
        accesses: list[str] = []

        for source_path in sorted(receiver_dir.glob("*.py")):
            lines = _sys_path_access_lines(
                source_path.read_text(encoding="utf-8"),
                filename=str(source_path),
            )
            accesses.extend(
                f"{source_path.relative_to(ROOT_DIR)}:{line}" for line in lines
            )

        self.assertEqual(
            accesses,
            [],
            "Receiver command sources must not access sys.path: " + ", ".join(accesses),
        )

    def test_stream_relays_rtcm_frames(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_stream_test_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "input.rtcm3"
            output_path = temp_root / "relay.rtcm3"
            frame = build_rtcm1005(3875000.125, 332500.25, 5025000.75)
            input_path.write_bytes(frame)

            result = self.run_gnss(
                "stream",
                "--input",
                str(input_path),
                "--output",
                str(output_path),
                "--limit",
                "1",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("Reference Station ARP", result.stdout)
            self.assertIn("summary: messages=1", result.stdout)
            self.assertEqual(output_path.read_bytes(), frame)
    @unittest.skipIf(os.name == "nt", "serial PTY test is POSIX-only")
    def test_stream_reads_rtcm_frame_from_serial_device(self) -> None:
        frame = build_rtcm1005(14.0, 28.0, 42.0)
        last_result: subprocess.CompletedProcess[str] | None = None

        for _ in range(3):
            master_fd, slave_fd = pty.openpty()
            try:
                slave_path = os.ttyname(slave_fd)
            finally:
                os.close(slave_fd)

            def writer() -> None:
                write_pty_payload(
                    master_fd,
                    [frame, frame],
                    initial_delay_s=0.10,
                    between_delay_s=0.05,
                    final_delay_s=0.20,
                )

            thread = threading.Thread(target=writer)
            thread.start()
            try:
                result = self.run_gnss(
                    "stream",
                    "--input",
                    f"serial://{slave_path}?baud=115200",
                    "--limit",
                    "1",
                )
            finally:
                thread.join()

            last_result = result
            if "Reference Station ARP" in result.stdout and "summary: messages=1" in result.stdout:
                break

        assert last_result is not None
        self.assertEqual(last_result.returncode, 0, msg=last_result.stderr)
        self.assertIn("Reference Station ARP", last_result.stdout)
        self.assertIn("summary: messages=1", last_result.stdout)
    @unittest.skipIf(os.name == "nt", "TCP source test is POSIX-only in the current build")
    def test_stream_reads_rtcm_frame_from_tcp_source(self) -> None:
        frame = build_rtcm1005(14.5, 29.0, 43.5)

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.bind(("127.0.0.1", 0))
            server.listen(1)
            server.settimeout(2.0)
            port = server.getsockname()[1]

            def writer() -> None:
                conn, _ = server.accept()
                with conn:
                    conn.sendall(frame)

            thread = threading.Thread(target=writer)
            thread.start()
            try:
                result = self.run_gnss(
                    "stream",
                    "--input",
                    f"tcp://127.0.0.1:{port}",
                    "--limit",
                    "1",
                )
            finally:
                thread.join()

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("Reference Station ARP", result.stdout)
        self.assertIn("summary: messages=1", result.stdout)
    @unittest.skipIf(os.name == "nt", "serial PTY test is POSIX-only")
    def test_stream_relays_rtcm_frame_to_serial_device(self) -> None:
        master_fd, slave_fd = pty.openpty()
        try:
            slave_path = os.ttyname(slave_fd)
        finally:
            os.close(slave_fd)

        frame = build_rtcm1005(15.0, 30.0, 45.0)

        with tempfile.TemporaryDirectory(prefix="gnss_stream_serial_out_test_") as temp_dir:
            input_path = Path(temp_dir) / "input.rtcm3"
            input_path.write_bytes(frame)
            received = bytearray()

            def reader() -> None:
                deadline = time.time() + 2.0
                try:
                    while len(received) < len(frame) and time.time() < deadline:
                        try:
                            chunk = os.read(master_fd, 1024)
                        except OSError as exc:
                            if exc.errno == 5:
                                time.sleep(0.02)
                                continue
                            raise
                        if not chunk:
                            time.sleep(0.02)
                            continue
                        received.extend(chunk)
                finally:
                    os.close(master_fd)

            thread = threading.Thread(target=reader)
            thread.start()
            try:
                result = self.run_gnss(
                    "stream",
                    "--input",
                    str(input_path),
                    "--output",
                    f"serial://{slave_path}?baud=115200",
                    "--limit",
                    "1",
                    "--quiet",
                )
            finally:
                thread.join()

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("summary: messages=1", result.stdout)
            self.assertEqual(bytes(received), frame)
    @unittest.skipIf(os.name == "nt", "TCP sink relay is POSIX-only in the current build")
    def test_stream_relays_rtcm_frame_to_tcp_sink(self) -> None:
        frame = build_rtcm1005(16.0, 32.0, 48.0)

        with tempfile.TemporaryDirectory(prefix="gnss_stream_tcp_out_test_") as temp_dir:
            input_path = Path(temp_dir) / "input.rtcm3"
            input_path.write_bytes(frame)

            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
                server.bind(("127.0.0.1", 0))
                server.listen(1)
                server.settimeout(2.0)
                port = server.getsockname()[1]
                received = bytearray()

                def reader() -> None:
                    conn, _ = server.accept()
                    with conn:
                        conn.settimeout(2.0)
                        while len(received) < len(frame):
                            chunk = conn.recv(1024)
                            if not chunk:
                                break
                            received.extend(chunk)

                thread = threading.Thread(target=reader)
                thread.start()
                try:
                    result = self.run_gnss(
                        "stream",
                        "--input",
                        str(input_path),
                        "--output",
                        f"tcp://127.0.0.1:{port}",
                        "--limit",
                        "1",
                        "--quiet",
                    )
                finally:
                    thread.join()

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("summary: messages=1", result.stdout)
            self.assertEqual(bytes(received), frame)
    def test_ubx_info_decodes_and_exports_rawx(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ubx_test_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session.ubx"
            output_path = temp_root / "session.obs"
            input_path.write_bytes(
                build_nav_pvt_message() + build_sfrbx_message() + build_mixed_rawx_message()
            )

            result = self.run_gnss(
                "ubx-info",
                "--input",
                str(input_path),
                "--decode-nav",
                "--decode-observations",
                "--obs-rinex-out",
                str(output_path),
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("UBX-NAV-PVT", result.stdout)
            self.assertIn("UBX-RXM-SFRBX", result.stdout)
            self.assertIn("UBX-RXM-RAWX", result.stdout)
            self.assertIn("nav: fix_type=3", result.stdout)
            self.assertIn("subframe: system=GPS sv=12 words=3 kind=GPS_LNAV frame_id=5", result.stdout)
            self.assertIn("obs: week=2200", result.stdout)
            self.assertIn("summary: processed_messages=3", result.stdout)
            self.assertIn("sfrbx=1", result.stdout)
            self.assertTrue(output_path.exists())
            self.assertGreater(output_path.stat().st_size, 0)
            exported = output_path.read_text(encoding="ascii")
            self.assertIn("RINEX VERSION / TYPE", exported)
            self.assertIn("G12", exported)
            self.assertIn("E05", exported)
            self.assertIn("R07", exported)
            self.assertIn("C19", exported)
            self.assertIn("J03", exported)
    def test_convert_converts_ubx_into_observation_rinex(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_convert_test_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session.ubx"
            output_path = temp_root / "converted.obs"
            input_path.write_bytes(build_nav_pvt_message() + build_mixed_rawx_message())

            result = self.run_gnss(
                "convert",
                "--format",
                "ubx",
                "--input",
                str(input_path),
                "--obs-out",
                str(output_path),
                "--quiet",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("summary: processed_messages=2", result.stdout)
            self.assertIn("exported_obs_epochs=1", result.stdout)
            self.assertTrue(output_path.exists())
            exported = output_path.read_text(encoding="ascii")
            self.assertIn("RINEX VERSION / TYPE", exported)
            self.assertIn("G12", exported)
            self.assertIn("E05", exported)
            self.assertIn("R07", exported)
            self.assertIn("C19", exported)
            self.assertIn("J03", exported)
    def test_nmea_info_decodes_gga_and_rmc_from_file(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_nmea_test_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session.nmea"
            input_path.write_text(
                build_nmea_gga_sentence() + build_nmea_rmc_sentence(),
                encoding="ascii",
            )

            result = self.run_gnss(
                "nmea-info",
                "--input",
                str(input_path),
                "--decode-gga",
                "--decode-rmc",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("gga: time=123519", result.stdout)
            self.assertIn("quality=4", result.stdout)
            self.assertIn("rmc: time=123520", result.stdout)
            self.assertIn("status=A", result.stdout)
            self.assertIn(
                "summary: sentences=2 valid=2 gga=1 rmc=1 valid_positions=2 checksum_errors=0",
                result.stdout,
            )
    def test_novatel_info_decodes_bestpos_and_bestvel_from_file(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_novatel_test_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session.log"
            input_path.write_text(
                build_novatel_bestpos_record() + build_novatel_bestvel_record(),
                encoding="ascii",
            )

            result = self.run_gnss(
                "novatel-info",
                "--input",
                str(input_path),
                "--decode-bestpos",
                "--decode-bestvel",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("bestpos: week=2200 tow=345600.000", result.stdout)
            self.assertIn("type=NARROW_INT", result.stdout)
            self.assertIn("bestvel: week=2200 tow=345600.000", result.stdout)
            self.assertIn("speed_mps=5.500", result.stdout)
            self.assertIn(
                "summary: records=2 valid=2 bestpos=1 bestvel=1 valid_positions=1 checksum_errors=0",
                result.stdout,
            )
    def test_novatel_info_decodes_binary_bestpos_and_bestvel_from_file(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_novatel_binary_test_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session.bin"
            input_path.write_bytes(
                build_novatel_bestpos_binary_record() + build_novatel_bestvel_binary_record()
            )

            result = self.run_gnss(
                "novatel-info",
                "--input",
                str(input_path),
                "--decode-bestpos",
                "--decode-bestvel",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("bestpos: week=2200 tow=345600.000", result.stdout)
            self.assertIn("type=NARROW_INT", result.stdout)
            self.assertIn("bestvel: week=2200 tow=345600.000", result.stdout)
            self.assertIn("type=DOPPLER_VELOCITY", result.stdout)
            self.assertIn(
                "summary: records=2 valid=2 bestpos=1 bestvel=1 valid_positions=1 checksum_errors=0",
                result.stdout,
            )
    def test_sbp_info_decodes_gps_time_pos_llh_and_vel_ned_from_file(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_sbp_test_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session.sbp"
            input_path.write_bytes(
                build_sbp_gps_time_frame() + build_sbp_pos_llh_frame() + build_sbp_vel_ned_frame()
            )

            result = self.run_gnss(
                "sbp-info",
                "--input",
                str(input_path),
                "--decode-time",
                "--decode-pos",
                "--decode-vel",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("gps_time: sender=66 week=2200 tow_ms=345600123", result.stdout)
            self.assertIn("ns_residual=-250", result.stdout)
            self.assertIn("pos_llh: sender=66 tow_ms=345600123", result.stdout)
            self.assertIn("lat=35.1234567", result.stdout)
            self.assertIn("height=42.100m", result.stdout)
            self.assertIn("vel_ned: sender=66 tow_ms=345600123", result.stdout)
            self.assertIn("n=1.250 e=-0.500 d=0.125mps", result.stdout)
            self.assertIn(
                "summary: frames=3 valid=3 gps_time=1 pos_llh=1 vel_ned=1 valid_positions=1 crc_errors=0",
                result.stdout,
            )
    def test_sbf_info_decodes_pvt_lband_and_p2pp_from_file(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_sbf_test_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session.sbf"
            input_path.write_bytes(
                build_sbf_pvt_geodetic_frame()
                + build_sbf_lband_tracker_frame()
                + build_sbf_p2pp_status_frame()
            )

            result = self.run_gnss(
                "sbf-info",
                "--input",
                str(input_path),
                "--decode-pvt",
                "--decode-lband",
                "--decode-p2pp",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("pvt_geodetic: week=2200 tow_ms=345600123 mode=PPP", result.stdout)
            self.assertIn("lat=35.1234567", result.stdout)
            self.assertIn("lband_tracker: week=2200 tow_ms=345600123", result.stdout)
            self.assertIn("locked=1", result.stdout)
            self.assertIn("service=42", result.stdout)
            self.assertIn("p2pp_status: week=2200 tow_ms=345600123", result.stdout)
            self.assertIn("status=Connected", result.stdout)
            self.assertIn(
                "summary: frames=3 valid=3 pvt_geodetic=1 lband_tracker=1 p2pp_status=1 valid_positions=1 crc_errors=0",
                result.stdout,
            )
    def test_trimble_info_decodes_time_llh_and_velocity_from_file(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_trimble_test_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session.gsof"
            input_path.write_bytes(
                build_gsof_genout_packet(
                    [
                        build_gsof_time_record(),
                        build_gsof_llh_record(),
                        build_gsof_velocity_record(),
                    ]
                )
            )

            result = self.run_gnss(
                "trimble-info",
                "--input",
                str(input_path),
                "--decode-time",
                "--decode-llh",
                "--decode-vel",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("time: week=2200 tow=345600.123 svs=18", result.stdout)
            self.assertIn("llh: lat=35.1234567 lon=139.9876543 height=42.100m", result.stdout)
            self.assertIn("velocity: flags=0x03 horiz=1.250mps heading=90.00deg vertical=-0.125mps", result.stdout)
            self.assertIn(
                "summary: packets=1 valid=1 time=1 llh=1 velocity=1 valid_positions=1 checksum_errors=0",
                result.stdout,
            )
    def test_skytraq_info_decodes_epoch_rawx_and_ack_from_file(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_skytraq_test_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session.stq"
            input_path.write_bytes(
                build_skytraq_epoch_message()
                + build_skytraq_rawx_message()
                + build_skytraq_ack_message()
            )

            result = self.run_gnss(
                "skytraq-info",
                "--input",
                str(input_path),
                "--decode-epoch",
                "--decode-raw",
                "--decode-ack",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("epoch: iod=7 week=2200 tow=345600.123", result.stdout)
            self.assertIn("rawx: version=1 iod=7 week=2200 tow=345600.123 period=1.000 nsat=14", result.stdout)
            self.assertIn("ack: msg=0x1E", result.stdout)
            self.assertIn(
                "summary: frames=3 valid=3 epoch=1 raw=0 rawx=1 ack=1 nack=0 checksum_errors=0",
                result.stdout,
            )
    def test_binex_info_decodes_metadata_nav_and_proto_from_file(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_binex_test_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session.bnx"
            input_path.write_bytes(
                build_binex_metadata_frame() + build_binex_nav_frame() + build_binex_proto_frame()
            )

            result = self.run_gnss(
                "binex-info",
                "--input",
                str(input_path),
                "--decode-metadata",
                "--decode-nav",
                "--decode-proto",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("metadata: subrecord=0x08 payload_bytes=4", result.stdout)
            self.assertIn("nav: subrecord=0x06 payload_bytes=5", result.stdout)
            self.assertIn("proto: subrecord=0x05 payload_bytes=4", result.stdout)
            self.assertIn(
                "summary: frames=3 valid=3 metadata=1 nav=1 proto=1 checksum_errors=0",
                result.stdout,
            )
    @unittest.skipIf(os.name == "nt", "serial PTY test is POSIX-only")
    def test_ubx_info_reads_mixed_rawx_from_serial_device(self) -> None:
        payload = build_nav_pvt_message() + build_mixed_rawx_message()
        last_result: subprocess.CompletedProcess[str] | None = None
        last_output_path: Path | None = None

        for _ in range(3):
            master_fd, slave_fd = pty.openpty()
            try:
                slave_path = os.ttyname(slave_fd)
            finally:
                os.close(slave_fd)

            def writer() -> None:
                write_pty_payload(
                    master_fd,
                    [payload, payload],
                    initial_delay_s=0.10,
                    between_delay_s=0.05,
                    final_delay_s=0.20,
                )

            with tempfile.TemporaryDirectory(prefix="gnss_ubx_serial_test_") as temp_dir:
                output_path = Path(temp_dir) / "serial.obs"
                last_output_path = output_path
                thread = threading.Thread(target=writer)
                thread.start()
                try:
                    result = self.run_gnss(
                        "ubx-info",
                        "--input",
                        f"serial://{slave_path}?baud=115200",
                        "--decode-observations",
                        "--obs-rinex-out",
                        str(output_path),
                        "--limit",
                        "4",
                    )
                finally:
                    thread.join()

                last_result = result
                if "UBX-RXM-RAWX" in result.stdout and "summary: processed_messages=" in result.stdout:
                    exported = output_path.read_text(encoding="ascii")
                    self.assertIn("G12", exported)
                    self.assertIn("E05", exported)
                    self.assertIn("R07", exported)
                    self.assertIn("C19", exported)
                    self.assertIn("J03", exported)
                    return

        self.assertIsNotNone(last_result)
        self.assertEqual(last_result.returncode, 0, msg=last_result.stderr)
        self.assertIsNotNone(last_output_path)
        self.assertIn("UBX-RXM-RAWX", last_result.stdout)
    @unittest.skipIf(os.name == "nt", "serial PTY test is POSIX-only")
    def test_nmea_info_reads_sentences_from_serial_device(self) -> None:
        master_fd, slave_fd = pty.openpty()
        try:
            slave_path = os.ttyname(slave_fd)
        finally:
            os.close(slave_fd)

        payload = (build_nmea_gga_sentence() + build_nmea_rmc_sentence()).encode("ascii")

        def writer() -> None:
            time.sleep(0.05)
            os.write(master_fd, payload)
            time.sleep(0.1)
            os.close(master_fd)

        thread = threading.Thread(target=writer)
        thread.start()
        try:
            result = self.run_gnss(
                "nmea-info",
                "--input",
                f"serial://{slave_path}?baud=9600",
                "--limit",
                "2",
            )
        finally:
            thread.join()

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("gga: time=123519", result.stdout)
        self.assertIn("rmc: time=123520", result.stdout)
        self.assertIn(
            "summary: sentences=2 valid=2 gga=1 rmc=1 valid_positions=2 checksum_errors=0",
            result.stdout,
        )
    @unittest.skipIf(os.name == "nt", "serial PTY test is POSIX-only")
    def test_novatel_info_reads_records_from_serial_device(self) -> None:
        master_fd, slave_fd = pty.openpty()
        try:
            slave_path = os.ttyname(slave_fd)
        finally:
            os.close(slave_fd)

        payload = (build_novatel_bestpos_record() + build_novatel_bestvel_record()).encode("ascii")

        def writer() -> None:
            time.sleep(0.20)
            os.write(master_fd, payload)
            time.sleep(0.1)
            os.close(master_fd)

        thread = threading.Thread(target=writer)
        thread.start()
        try:
            result = self.run_gnss(
                "novatel-info",
                "--input",
                f"serial://{slave_path}?baud=115200",
                "--limit",
                "2",
            )
        finally:
            thread.join()

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("bestpos: week=2200 tow=345600.000", result.stdout)
        self.assertIn("bestvel: week=2200 tow=345600.000", result.stdout)
        self.assertIn(
            "summary: records=2 valid=2 bestpos=1 bestvel=1 valid_positions=1 checksum_errors=0",
            result.stdout,
        )
    @unittest.skipIf(os.name == "nt", "serial PTY test is POSIX-only")
    def test_novatel_info_reads_binary_records_from_serial_device(self) -> None:
        master_fd, slave_fd = pty.openpty()
        try:
            slave_path = os.ttyname(slave_fd)
        finally:
            os.close(slave_fd)

        payload = build_novatel_bestpos_binary_record() + build_novatel_bestvel_binary_record()

        def writer() -> None:
            time.sleep(0.20)
            os.write(master_fd, payload)
            time.sleep(0.1)
            os.close(master_fd)

        thread = threading.Thread(target=writer)
        thread.start()
        try:
            result = self.run_gnss(
                "novatel-info",
                "--input",
                f"serial://{slave_path}?baud=115200",
                "--limit",
                "2",
            )
        finally:
            thread.join()

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("bestpos: week=2200 tow=345600.000", result.stdout)
        self.assertIn("bestvel: week=2200 tow=345600.000", result.stdout)
        self.assertIn(
            "summary: records=2 valid=2 bestpos=1 bestvel=1 valid_positions=1 checksum_errors=0",
            result.stdout,
        )
    @unittest.skipIf(os.name == "nt", "serial PTY test is POSIX-only")
    def test_sbp_info_reads_records_from_serial_device(self) -> None:
        master_fd, slave_fd = pty.openpty()
        try:
            slave_path = os.ttyname(slave_fd)
        finally:
            os.close(slave_fd)

        payload = build_sbp_gps_time_frame() + build_sbp_pos_llh_frame() + build_sbp_vel_ned_frame()

        def writer() -> None:
            time.sleep(0.20)
            os.write(master_fd, payload)
            time.sleep(0.1)
            os.close(master_fd)

        thread = threading.Thread(target=writer)
        thread.start()
        try:
            result = self.run_gnss(
                "sbp-info",
                "--input",
                f"serial://{slave_path}?baud=115200",
                "--limit",
                "3",
            )
        finally:
            thread.join()

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("gps_time: sender=66 week=2200 tow_ms=345600123", result.stdout)
        self.assertIn("pos_llh: sender=66 tow_ms=345600123", result.stdout)
        self.assertIn("vel_ned: sender=66 tow_ms=345600123", result.stdout)
        self.assertIn(
            "summary: frames=3 valid=3 gps_time=1 pos_llh=1 vel_ned=1 valid_positions=1 crc_errors=0",
            result.stdout,
        )
    @unittest.skipIf(os.name == "nt", "serial PTY test is POSIX-only")
    def test_sbf_info_reads_records_from_serial_device(self) -> None:
        master_fd, slave_fd = pty.openpty()
        try:
            slave_path = os.ttyname(slave_fd)
        finally:
            os.close(slave_fd)

        payload = (
            build_sbf_pvt_geodetic_frame()
            + build_sbf_lband_tracker_frame()
            + build_sbf_p2pp_status_frame()
        )

        def writer() -> None:
            time.sleep(0.20)
            os.write(master_fd, payload)
            time.sleep(0.1)
            os.close(master_fd)

        thread = threading.Thread(target=writer)
        thread.start()
        try:
            result = self.run_gnss(
                "sbf-info",
                "--input",
                f"serial://{slave_path}?baud=115200",
                "--limit",
                "3",
            )
        finally:
            thread.join()

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("pvt_geodetic: week=2200 tow_ms=345600123 mode=PPP", result.stdout)
        self.assertIn("lband_tracker: week=2200 tow_ms=345600123", result.stdout)
        self.assertIn("p2pp_status: week=2200 tow_ms=345600123", result.stdout)
        self.assertIn(
            "summary: frames=3 valid=3 pvt_geodetic=1 lband_tracker=1 p2pp_status=1 valid_positions=1 crc_errors=0",
            result.stdout,
        )
    @unittest.skipIf(os.name == "nt", "serial PTY test is POSIX-only")
    def test_trimble_info_reads_records_from_serial_device(self) -> None:
        master_fd, slave_fd = pty.openpty()
        try:
            slave_path = os.ttyname(slave_fd)
        finally:
            os.close(slave_fd)

        payload = build_gsof_genout_packet(
            [
                build_gsof_time_record(),
                build_gsof_llh_record(),
                build_gsof_velocity_record(),
            ]
        )

        def writer() -> None:
            time.sleep(0.20)
            os.write(master_fd, payload)
            time.sleep(0.1)
            os.close(master_fd)

        thread = threading.Thread(target=writer)
        thread.start()
        try:
            result = self.run_gnss(
                "trimble-info",
                "--input",
                f"serial://{slave_path}?baud=115200",
                "--limit",
                "3",
            )
        finally:
            thread.join()

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("time: week=2200 tow=345600.123", result.stdout)
        self.assertIn("llh: lat=35.1234567 lon=139.9876543 height=42.100m", result.stdout)
        self.assertIn("velocity: flags=0x03 horiz=1.250mps heading=90.00deg vertical=-0.125mps", result.stdout)
        self.assertIn(
            "summary: packets=1 valid=1 time=1 llh=1 velocity=1 valid_positions=1 checksum_errors=0",
            result.stdout,
        )
    @unittest.skipIf(os.name == "nt", "serial PTY test is POSIX-only")
    def test_skytraq_info_reads_epoch_and_nack_from_serial_device(self) -> None:
        master_fd, slave_fd = pty.openpty()
        try:
            slave_path = os.ttyname(slave_fd)
        finally:
            os.close(slave_fd)

        payload = build_skytraq_epoch_message() + build_skytraq_nack_message()

        def writer() -> None:
            time.sleep(0.20)
            os.write(master_fd, payload)
            time.sleep(0.1)
            os.close(master_fd)

        thread = threading.Thread(target=writer)
        thread.start()
        try:
            result = self.run_gnss(
                "skytraq-info",
                "--input",
                f"serial://{slave_path}?baud=115200",
                "--limit",
                "2",
            )
        finally:
            thread.join()

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("epoch: iod=7 week=2200 tow=345600.123", result.stdout)
        self.assertIn("nack: msg=0x09", result.stdout)
        self.assertIn(
            "summary: frames=2 valid=2 epoch=1 raw=0 rawx=0 ack=0 nack=1 checksum_errors=0",
            result.stdout,
        )
    @unittest.skipIf(os.name == "nt", "serial PTY test is POSIX-only")
    def test_binex_info_reads_records_from_serial_device(self) -> None:
        master_fd, slave_fd = pty.openpty()
        try:
            slave_path = os.ttyname(slave_fd)
        finally:
            os.close(slave_fd)

        payload = build_binex_metadata_frame() + build_binex_proto_frame()

        def writer() -> None:
            time.sleep(0.20)
            os.write(master_fd, payload)
            time.sleep(0.1)
            os.close(master_fd)

        thread = threading.Thread(target=writer)
        thread.start()
        try:
            result = self.run_gnss(
                "binex-info",
                "--input",
                f"serial://{slave_path}?baud=115200",
                "--decode-metadata",
                "--decode-proto",
                "--limit",
                "2",
            )
        finally:
            thread.join()

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("metadata: subrecord=0x08 payload_bytes=4", result.stdout)
        self.assertIn("proto: subrecord=0x05 payload_bytes=4", result.stdout)
        self.assertIn(
            "summary: frames=2 valid=2 metadata=1 nav=0 proto=1 checksum_errors=0",
            result.stdout,
        )
    @unittest.skipIf(os.name == "nt", "serial PTY test is POSIX-only")
    def test_qzss_l6_info_reads_frames_from_serial_device(self) -> None:
        payload = (
            build_qzss_l6_frame(
                prn=199,
                facility_id=1,
                subframe_start=True,
                data_part=b"L6-SERIAL-ONE",
            )
            + build_qzss_l6_frame(
                prn=201,
                facility_id=3,
                subframe_start=False,
                data_part=b"L6-SERIAL-TWO",
            )
        )
        last_result: subprocess.CompletedProcess[str] | None = None
        for _ in range(3):
            master_fd, slave_fd = pty.openpty()
            try:
                slave_path = os.ttyname(slave_fd)
            finally:
                os.close(slave_fd)

            def writer() -> None:
                write_pty_payload(
                    master_fd,
                    [payload],
                    initial_delay_s=0.20,
                    between_delay_s=0.0,
                    final_delay_s=0.1,
                )

            thread = threading.Thread(target=writer)
            thread.start()
            try:
                result = self.run_gnss(
                    "qzss-l6-info",
                    "--input",
                    f"serial://{slave_path}?baud=115200",
                    "--limit",
                    "2",
                    "--show-preview",
                )
            finally:
                thread.join()
            last_result = result
            if (
                result.returncode == 0
                and "l6_frame: index=1 prn=199 vendor=5 facility=Hitachi-Ota" in result.stdout
            ):
                break

        assert last_result is not None
        self.assertEqual(last_result.returncode, 0, msg=last_result.stderr)
        self.assertIn("l6_frame: index=1 prn=199 vendor=5 facility=Hitachi-Ota", last_result.stdout)
        self.assertIn("preview=L6-SERIAL-ONE", last_result.stdout)
        self.assertIn("l6_frame: index=2 prn=201 vendor=5 facility=Kobe", last_result.stdout)
        self.assertIn(
            "summary: frames=2 valid=2 clas_vendor=2 subframe_starts=1 alerts=0 subframes=0 prns=199,201",
            last_result.stdout,
        )
    @unittest.skipIf(os.name == "nt", "serial PTY test is POSIX-only")
    def test_convert_reads_mixed_rawx_from_serial_device(self) -> None:
        payload = build_nav_pvt_message() + build_mixed_rawx_message()
        last_result: subprocess.CompletedProcess[str] | None = None
        for _ in range(3):
            master_fd, slave_fd = pty.openpty()
            try:
                slave_path = os.ttyname(slave_fd)
            finally:
                os.close(slave_fd)

            def writer() -> None:
                write_pty_payload(
                    master_fd,
                    [payload, payload],
                    initial_delay_s=0.10,
                    between_delay_s=0.05,
                    final_delay_s=0.20,
                )

            with tempfile.TemporaryDirectory(prefix="gnss_convert_serial_test_") as temp_dir:
                output_path = Path(temp_dir) / "serial_converted.obs"
                thread = threading.Thread(target=writer)
                thread.start()
                try:
                    result = self.run_gnss(
                        "convert",
                        "--format",
                        "ubx",
                        "--input",
                        f"serial://{slave_path}?baud=115200",
                        "--obs-out",
                        str(output_path),
                        "--limit",
                        "4",
                        "--quiet",
                    )
                finally:
                    thread.join()

                last_result = result
                if "summary: processed_messages=" in result.stdout and output_path.exists():
                    exported = output_path.read_text(encoding="ascii")
                    if all(token in exported for token in ("G12", "E05", "R07", "C19", "J03")):
                        self.assertEqual(result.returncode, 0, msg=result.stderr)
                        return

        self.assertIsNotNone(last_result)
        self.assertEqual(last_result.returncode, 0, msg=last_result.stderr)
        self.assertIn("summary: processed_messages=", last_result.stdout)
    def test_convert_exports_sfrbx_csv(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_convert_sfrbx_test_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session.ubx"
            output_path = temp_root / "session_sfrbx.csv"
            input_path.write_bytes(build_sfrbx_message())

            result = self.run_gnss(
                "convert",
                "--format",
                "ubx",
                "--input",
                str(input_path),
                "--sfrbx-out",
                str(output_path),
                "--quiet",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("summary: processed_messages=1", result.stdout)
            self.assertIn("exported_sfrbx_messages=1", result.stdout)
            exported = output_path.read_text(encoding="ascii")
            self.assertIn(
                "system,sv_id,frequency_id,channel,version,frame_kind,frame_id,page_id,word_count,words_hex",
                exported,
            )
            self.assertIn("GPS,12,0,1,2,GPS_LNAV,5,,3,8B0000AA;00000500;CAFEBABE", exported)
    def test_convert_exports_gps_nav_from_ubx_sfrbx(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_convert_nav_test_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_nav.ubx"
            output_path = temp_root / "session.nav"
            input_path.write_bytes(
                build_gps_lnav_sfrbx_message(1)
                + build_gps_lnav_sfrbx_message(2)
                + build_gps_lnav_sfrbx_message(3)
            )

            result = self.run_gnss(
                "convert",
                "--format",
                "ubx",
                "--input",
                str(input_path),
                "--nav-out",
                str(output_path),
                "--quiet",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("summary: processed_messages=3", result.stdout)
            self.assertIn("exported_nav_messages=1", result.stdout)
            exported = output_path.read_text(encoding="ascii")
            self.assertIn("RINEX VERSION / TYPE", exported)
            self.assertIn("G12", exported)
            self.assertIn("3.456000000000D+05", exported)
            self.assertIn("5.153795890808D+03", exported)
    def test_convert_exports_glonass_nav_from_ubx_sfrbx(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_convert_glo_nav_test_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_glo_nav.ubx"
            output_path = temp_root / "session_glo.nav"
            input_path.write_bytes(
                build_rawx_message()
                + build_glonass_sfrbx_message(1)
                + build_glonass_sfrbx_message(2)
                + build_glonass_sfrbx_message(3)
                + build_glonass_sfrbx_message(4)
            )

            result = self.run_gnss(
                "convert",
                "--format",
                "ubx",
                "--input",
                str(input_path),
                "--nav-out",
                str(output_path),
                "--quiet",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("summary: processed_messages=5", result.stdout)
            self.assertIn("exported_nav_messages=1", result.stdout)

            exported_lines = output_path.read_text(encoding="ascii").splitlines()
            record_index = next(index for index, line in enumerate(exported_lines) if line.startswith("R07"))
            self.assertEqual(len(exported_lines) - record_index, 4)
            self.assertTrue(exported_lines[record_index + 2].rstrip().endswith(" 1.000000000000D+00"))
            self.assertTrue(exported_lines[record_index + 3].rstrip().endswith(" 9.000000000000D+00"))
    def test_convert_exports_beidou_d1_nav_from_ubx_sfrbx(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_convert_bds_nav_test_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_bds_nav.ubx"
            output_path = temp_root / "session_bds.nav"
            input_path.write_bytes(
                build_beidou_d1_sfrbx_message(1)
                + build_beidou_d1_sfrbx_message(2)
                + build_beidou_d1_sfrbx_message(3)
            )

            result = self.run_gnss(
                "convert",
                "--format",
                "ubx",
                "--input",
                str(input_path),
                "--nav-out",
                str(output_path),
                "--quiet",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("summary: processed_messages=3", result.stdout)
            self.assertIn("exported_nav_messages=1", result.stdout)

            exported = output_path.read_text(encoding="ascii")
            self.assertIn("C12", exported)
            self.assertIn("5.282625600815D+03", exported)
            self.assertIn("2.300000000000D-08", exported)
    def test_convert_exports_beidou_d2_nav_from_ubx_sfrbx(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_convert_bds_d2_nav_test_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_bds_d2_nav.ubx"
            output_path = temp_root / "session_bds_d2.nav"
            input_path.write_bytes(
                build_beidou_d2_sfrbx_message(1)
                + build_beidou_d2_sfrbx_message(3)
                + build_beidou_d2_sfrbx_message(4)
                + build_beidou_d2_sfrbx_message(5)
                + build_beidou_d2_sfrbx_message(6)
                + build_beidou_d2_sfrbx_message(7)
                + build_beidou_d2_sfrbx_message(8)
                + build_beidou_d2_sfrbx_message(9)
                + build_beidou_d2_sfrbx_message(10)
            )

            result = self.run_gnss(
                "convert",
                "--format",
                "ubx",
                "--input",
                str(input_path),
                "--nav-out",
                str(output_path),
                "--quiet",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("summary: processed_messages=9", result.stdout)
            self.assertIn("exported_nav_messages=1", result.stdout)

            exported = output_path.read_text(encoding="ascii")
            self.assertIn("C03", exported)
            self.assertIn("5.282625600815D+03", exported)
            self.assertIn("2.300000000000D-08", exported)
    def test_convert_exports_galileo_nav_from_ubx_sfrbx(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_convert_gal_nav_test_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session_gal_nav.ubx"
            output_path = temp_root / "session_gal.nav"
            input_path.write_bytes(
                b"".join(build_galileo_inav_sfrbx_message(word_type) for word_type in range(6))
            )

            result = self.run_gnss(
                "convert",
                "--format",
                "ubx",
                "--input",
                str(input_path),
                "--nav-out",
                str(output_path),
                "--quiet",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("summary: processed_messages=6", result.stdout)
            self.assertIn("exported_nav_messages=1", result.stdout)

            exported = output_path.read_text(encoding="ascii")
            self.assertIn("E05", exported)
            self.assertIn("5.440588203430D+03", exported)
            self.assertIn("1.094304025173D-08", exported)
