#!/usr/bin/env python3
"""CLI regression tests for the dispatcher-backed native tools."""

from __future__ import annotations

import os
import json
import struct
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
DISPATCHER = ROOT_DIR / "apps" / "gnss.py"


def crc24q(data: bytes) -> int:
    table = (
        0x000000, 0x864CFB, 0x8AD50D, 0x0C99F6, 0x93E6E1, 0x15AA1A, 0x1933EC, 0x9F7F17,
        0xA18139, 0x27CDC2, 0x2B5434, 0xAD18CF, 0x3267D8, 0xB42B23, 0xB8B2D5, 0x3EFE2E,
        0xC54E89, 0x430272, 0x4F9B84, 0xC9D77F, 0x56A868, 0xD0E493, 0xDC7D65, 0x5A319E,
        0x64CFB0, 0xE2834B, 0xEE1ABD, 0x685646, 0xF72951, 0x7165AA, 0x7DFC5C, 0xFBB0A7,
        0x0CD1E9, 0x8A9D12, 0x8604E4, 0x00481F, 0x9F3708, 0x197BF3, 0x15E205, 0x93AEFE,
        0xAD50D0, 0x2B1C2B, 0x2785DD, 0xA1C926, 0x3EB631, 0xB8FACA, 0xB4633C, 0x322FC7,
        0xC99F60, 0x4FD39B, 0x434A6D, 0xC50696, 0x5A7981, 0xDC357A, 0xD0AC8C, 0x56E077,
        0x681E59, 0xEE52A2, 0xE2CB54, 0x6487AF, 0xFBF8B8, 0x7DB443, 0x712DB5, 0xF7614E,
        0x19A3D2, 0x9FEF29, 0x9376DF, 0x153A24, 0x8A4533, 0x0C09C8, 0x00903E, 0x86DCC5,
        0xB822EB, 0x3E6E10, 0x32F7E6, 0xB4BB1D, 0x2BC40A, 0xAD88F1, 0xA11107, 0x275DFC,
        0xDCED5B, 0x5AA1A0, 0x563856, 0xD074AD, 0x4F0BBA, 0xC94741, 0xC5DEB7, 0x43924C,
        0x7D6C62, 0xFB2099, 0xF7B96F, 0x71F594, 0xEE8A83, 0x68C678, 0x645F8E, 0xE21375,
        0x15723B, 0x933EC0, 0x9FA736, 0x19EBCD, 0x8694DA, 0x00D821, 0x0C41D7, 0x8A0D2C,
        0xB4F302, 0x32BFF9, 0x3E260F, 0xB86AF4, 0x2715E3, 0xA15918, 0xADC0EE, 0x2B8C15,
        0xD03CB2, 0x567049, 0x5AE9BF, 0xDCA544, 0x43DA53, 0xC596A8, 0xC90F5E, 0x4F43A5,
        0x71BD8B, 0xF7F170, 0xFB6886, 0x7D247D, 0xE25B6A, 0x641791, 0x688E67, 0xEEC29C,
        0x3347A4, 0xB50B5F, 0xB992A9, 0x3FDE52, 0xA0A145, 0x26EDBE, 0x2A7448, 0xAC38B3,
        0x92C69D, 0x148A66, 0x181390, 0x9E5F6B, 0x01207C, 0x876C87, 0x8BF571, 0x0DB98A,
        0xF6092D, 0x7045D6, 0x7CDC20, 0xFA90DB, 0x65EFCC, 0xE3A337, 0xEF3AC1, 0x69763A,
        0x578814, 0xD1C4EF, 0xDD5D19, 0x5B11E2, 0xC46EF5, 0x42220E, 0x4EBBF8, 0xC8F703,
        0x3F964D, 0xB9DAB6, 0xB54340, 0x330FBB, 0xAC70AC, 0x2A3C57, 0x26A5A1, 0xA0E95A,
        0x9E1774, 0x185B8F, 0x14C279, 0x928E82, 0x0DF195, 0x8BBD6E, 0x872498, 0x016863,
        0xFAD8C4, 0x7C943F, 0x700DC9, 0xF64132, 0x693E25, 0xEF72DE, 0xE3EB28, 0x65A7D3,
        0x5B59FD, 0xDD1506, 0xD18CF0, 0x57C00B, 0xC8BF1C, 0x4EF3E7, 0x426A11, 0xC426EA,
        0x2AE476, 0xACA88D, 0xA0317B, 0x267D80, 0xB90297, 0x3F4E6C, 0x33D79A, 0xB59B61,
        0x8B654F, 0x0D29B4, 0x01B042, 0x87FCB9, 0x1883AE, 0x9ECF55, 0x9256A3, 0x141A58,
        0xEFAAFF, 0x69E604, 0x657FF2, 0xE33309, 0x7C4C1E, 0xFA00E5, 0xF69913, 0x70D5E8,
        0x4E2BC6, 0xC8673D, 0xC4FECB, 0x42B230, 0xDDCD27, 0x5B81DC, 0x57182A, 0xD154D1,
        0x26359F, 0xA07964, 0xACE092, 0x2AAC69, 0xB5D37E, 0x339F85, 0x3F0673, 0xB94A88,
        0x87B4A6, 0x01F85D, 0x0D61AB, 0x8B2D50, 0x145247, 0x921EBC, 0x9E874A, 0x18CBB1,
        0xE37B16, 0x6537ED, 0x69AE1B, 0xEFE2E0, 0x709DF7, 0xF6D10C, 0xFA48FA, 0x7C0401,
        0x42FA2F, 0xC4B6D4, 0xC82F22, 0x4E63D9, 0xD11CCE, 0x575035, 0x5BC9C3, 0xDD8538,
    )
    crc = 0
    for byte in data:
        index = ((crc >> 16) ^ byte) & 0xFF
        crc = ((crc << 8) ^ table[index]) & 0xFFFFFFFF
    return crc & 0xFFFFFF


def set_unsigned_bits(payload: bytearray, pos: int, length: int, value: int) -> None:
    for i in range(length):
        bit_index = pos + length - 1 - i
        byte_index = bit_index // 8
        bit_in_byte = 7 - (bit_index % 8)
        mask = 1 << bit_in_byte
        if (value >> i) & 1:
            payload[byte_index] |= mask
        else:
            payload[byte_index] &= ~mask & 0xFF


def set_signed_bits(payload: bytearray, pos: int, length: int, value: int) -> None:
    masked = value & ((1 << length) - 1)
    set_unsigned_bits(payload, pos, length, masked)


def build_rtcm1005(x_m: float, y_m: float, z_m: float) -> bytes:
    payload = bytearray(19)
    bit = 0
    set_unsigned_bits(payload, bit, 12, 1005)
    bit += 12
    set_unsigned_bits(payload, bit, 12, 42)
    bit += 12
    set_unsigned_bits(payload, bit, 6, 0)
    bit += 6
    for _ in range(4):
        set_unsigned_bits(payload, bit, 1, 1)
        bit += 1
    set_signed_bits(payload, bit, 38, round(x_m * 10000.0))
    bit += 38
    set_unsigned_bits(payload, bit, 1, 0)
    bit += 1
    set_unsigned_bits(payload, bit, 1, 0)
    bit += 1
    set_signed_bits(payload, bit, 38, round(y_m * 10000.0))
    bit += 38
    set_unsigned_bits(payload, bit, 2, 0)
    bit += 2
    set_signed_bits(payload, bit, 38, round(z_m * 10000.0))

    frame = bytearray([0xD3, 0x00, len(payload)])
    frame.extend(payload)
    crc = crc24q(frame)
    frame.extend(((crc >> 16) & 0xFF, (crc >> 8) & 0xFF, crc & 0xFF))
    return bytes(frame)


def build_ubx_message(message_class: int, message_id: int, payload: bytes) -> bytes:
    message = bytearray([0xB5, 0x62, message_class, message_id])
    message.extend(struct.pack("<H", len(payload)))
    message.extend(payload)
    ck_a = 0
    ck_b = 0
    for byte in message[2:]:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    message.extend((ck_a, ck_b))
    return bytes(message)


def build_nav_pvt_message() -> bytes:
    payload = bytearray(92)
    struct.pack_into("<I", payload, 0, 345600000)
    payload[20] = 3
    payload[21] = 0x01 | 0x02 | (0x02 << 6)
    payload[23] = 18
    struct.pack_into("<i", payload, 24, 1391234567)
    struct.pack_into("<i", payload, 28, 356543210)
    struct.pack_into("<i", payload, 32, 12345)
    struct.pack_into("<I", payload, 40, 1500)
    struct.pack_into("<I", payload, 44, 2300)
    return build_ubx_message(0x01, 0x07, payload)


def build_rawx_message() -> bytes:
    payload = bytearray()
    payload.extend(struct.pack("<d", 345600.125))
    payload.extend(struct.pack("<H", 2200))
    payload.extend(bytes([18, 1, 0x01, 0x01, 0x00, 0x00]))

    payload.extend(struct.pack("<d", 20200000.25))
    payload.extend(struct.pack("<d", 110000.5))
    payload.extend(struct.pack("<f", -1234.5))
    payload.extend(bytes([0, 12, 0, 0]))
    payload.extend(struct.pack("<H", 500))
    payload.extend(bytes([45, 0, 0, 0, 0x03, 0]))

    return build_ubx_message(0x02, 0x15, payload)


class CLIToolsTest(unittest.TestCase):
    def run_gnss(self, *args: str) -> subprocess.CompletedProcess[str]:
        env = os.environ.copy()
        env["PYTHONUNBUFFERED"] = "1"
        return subprocess.run(
            [sys.executable, str(DISPATCHER), *args],
            cwd=ROOT_DIR,
            env=env,
            text=True,
            capture_output=True,
            check=False,
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

    def test_ubx_info_decodes_and_exports_rawx(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ubx_test_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session.ubx"
            output_path = temp_root / "session.obs"
            input_path.write_bytes(build_nav_pvt_message() + build_rawx_message())

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
            self.assertIn("UBX-RXM-RAWX", result.stdout)
            self.assertIn("nav: fix_type=3", result.stdout)
            self.assertIn("obs: week=2200", result.stdout)
            self.assertIn("summary: processed_messages=2", result.stdout)
            self.assertTrue(output_path.exists())
            self.assertGreater(output_path.stat().st_size, 0)
            exported = output_path.read_text(encoding="ascii")
            self.assertIn("RINEX VERSION / TYPE", exported)
            self.assertIn("G12", exported)

    def test_convert_converts_ubx_into_observation_rinex(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_convert_test_") as temp_dir:
            temp_root = Path(temp_dir)
            input_path = temp_root / "session.ubx"
            output_path = temp_root / "converted.obs"
            input_path.write_bytes(build_nav_pvt_message() + build_rawx_message())

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

    def test_replay_solves_bundled_rinex_sequence(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_replay_test_") as temp_dir:
            temp_root = Path(temp_dir)
            output_path = temp_root / "replay.pos"

            result = self.run_gnss(
                "replay",
                "--rover-rinex",
                str(ROOT_DIR / "data" / "rover_kinematic.obs"),
                "--base-rinex",
                str(ROOT_DIR / "data" / "base_kinematic.obs"),
                "--nav-rinex",
                str(ROOT_DIR / "data" / "navigation_kinematic.nav"),
                "--out",
                str(output_path),
                "--max-epochs",
                "20",
                "--quiet",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("summary: aligned_epochs=", result.stdout)
            self.assertIn("written_solutions=", result.stdout)
            self.assertTrue(output_path.exists())
            exported = output_path.read_text(encoding="ascii")
            self.assertIn("LibGNSS++ Position Solution", exported)
            solution_lines = [
                line for line in exported.splitlines()
                if line and not line.startswith("%")
            ]
            self.assertGreater(len(solution_lines), 0)

    def test_live_command_help_is_available(self) -> None:
        result = self.run_gnss("live", "--help")
        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("Usage:", result.stdout)
        self.assertIn("--rover-rtcm", result.stdout)
        self.assertIn("--base-hold-seconds", result.stdout)

    def test_rcv_dry_run_and_status_snapshot(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_rcv_test_") as temp_dir:
            temp_root = Path(temp_dir)
            config_path = temp_root / "receiver.conf"
            status_path = temp_root / "receiver_status.json"
            output_path = temp_root / "receiver.pos"
            config_path.write_text(
                "\n".join(
                    [
                        f"rover_rtcm={temp_root / 'rover.rtcm3'}",
                        f"base_rtcm={temp_root / 'base.rtcm3'}",
                        f"out={output_path}",
                        "max_epochs=1",
                        "base_hold_seconds=0.5",
                        "quiet=true",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            dry_run = self.run_gnss("rcv", "--config", str(config_path), "--dry-run")
            self.assertEqual(dry_run.returncode, 0, msg=dry_run.stderr)
            resolved = json.loads(dry_run.stdout)
            self.assertEqual(resolved["config"]["max_epochs"], "1")
            self.assertIn("--base-hold-seconds", resolved["command"])

            config_path.write_text(
                "\n".join(
                    [
                        f"rover_rtcm={temp_root / 'missing_rover.rtcm3'}",
                        f"base_rtcm={temp_root / 'missing_base.rtcm3'}",
                        f"out={output_path}",
                        "max_epochs=1",
                        "quiet=true",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )
            run_result = self.run_gnss(
                "rcv",
                "--config",
                str(config_path),
                "--status-out",
                str(status_path),
            )
            self.assertNotEqual(run_result.returncode, 0)
            self.assertTrue(status_path.exists())
            status = json.loads(status_path.read_text(encoding="utf-8"))
            self.assertEqual(status["state"], "failed")
            self.assertIn("command", status)
            self.assertEqual(status["returncode"], run_result.returncode)

    def test_rcv_start_and_status_report_failed_background_run(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_rcv_start_test_") as temp_dir:
            temp_root = Path(temp_dir)
            config_path = temp_root / "receiver.conf"
            status_path = temp_root / "receiver_status.json"
            log_path = temp_root / "receiver.log"
            output_path = temp_root / "receiver.pos"
            config_path.write_text(
                "\n".join(
                    [
                        f"rover_rtcm={temp_root / 'missing_rover.rtcm3'}",
                        f"base_rtcm={temp_root / 'missing_base.rtcm3'}",
                        f"out={output_path}",
                        "max_epochs=1",
                        "quiet=true",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            start_result = self.run_gnss(
                "rcv",
                "start",
                "--config",
                str(config_path),
                "--status-out",
                str(status_path),
                "--log-out",
                str(log_path),
            )
            self.assertEqual(start_result.returncode, 0, msg=start_result.stderr)
            launched = json.loads(start_result.stdout)
            self.assertEqual(launched["state"], "starting")
            self.assertEqual(launched["status_path"], str(status_path))
            self.assertEqual(launched["log_path"], str(log_path))

            status_result = self.run_gnss(
                "rcv",
                "status",
                "--status-out",
                str(status_path),
                "--wait-seconds",
                "5",
            )
            self.assertEqual(status_result.returncode, 0, msg=status_result.stderr)
            status = json.loads(status_result.stdout)
            self.assertEqual(status["state"], "failed")
            self.assertFalse(status["pid_running"])
            self.assertEqual(status["log_path"], str(log_path))
            self.assertGreaterEqual(status["uptime_seconds"], 0.0)
            self.assertTrue(log_path.exists())

    def test_rcv_stop_is_idempotent_when_receiver_is_not_running(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_rcv_stop_test_") as temp_dir:
            temp_root = Path(temp_dir)
            status_path = temp_root / "receiver_status.json"
            status_path.write_text(
                json.dumps(
                    {
                        "state": "completed",
                        "pid": 999999,
                        "config_path": str(temp_root / "receiver.conf"),
                    }
                )
                + "\n",
                encoding="utf-8",
            )

            result = self.run_gnss("rcv", "stop", "--status-out", str(status_path))
            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("receiver not running", result.stdout)

    def test_rcv_restart_without_existing_status_behaves_like_start(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_rcv_restart_test_") as temp_dir:
            temp_root = Path(temp_dir)
            config_path = temp_root / "receiver.conf"
            status_path = temp_root / "receiver_status.json"
            output_path = temp_root / "receiver.pos"
            config_path.write_text(
                "\n".join(
                    [
                        f"rover_rtcm={temp_root / 'missing_rover.rtcm3'}",
                        f"base_rtcm={temp_root / 'missing_base.rtcm3'}",
                        f"out={output_path}",
                        "max_epochs=1",
                        "quiet=true",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            restart_result = self.run_gnss(
                "rcv",
                "restart",
                "--config",
                str(config_path),
                "--status-out",
                str(status_path),
                "--wait-seconds",
                "0.1",
            )
            self.assertEqual(restart_result.returncode, 0, msg=restart_result.stderr)
            launched = json.loads(restart_result.stdout)
            self.assertEqual(launched["state"], "starting")
            self.assertEqual(launched["status_path"], str(status_path))


if __name__ == "__main__":
    unittest.main()
