#!/usr/bin/env python3
"""Tests for `gnss realtime-stress`: framing/time-shift helpers, fault injection, and a smoke run.

The smoke run replays the tracked 40 s recorded fixture (tests/fixtures/realtime_stress, PPC Tokyo
run1, GPS-only RTCM3) at 20x over local TCP/NTRIP against the built gnss_live, injects every fault
kind and checks that gnss_live survives where it should and that each fault is detected/reported.
"""

from __future__ import annotations

import json
import math
import os
import sys
import tempfile
import time
import unittest
from pathlib import Path

ROOT_DIR = Path(__file__).resolve().parents[1]
COMMANDS_DIR = ROOT_DIR / "apps" / "commands"
sys.path.insert(0, str(COMMANDS_DIR))
sys.path.insert(0, str(COMMANDS_DIR / "benchmarks"))

from support import gnss_rtstress as rt  # noqa: E402
import gnss_realtime_stress as cli  # noqa: E402

FIXTURE = ROOT_DIR / "tests" / "fixtures" / "realtime_stress"


def build_dir() -> str | None:
    env = os.environ.get("GNSSPP_BUILD_DIR")
    if env:
        return env
    default = ROOT_DIR / "build"
    return str(default) if default.exists() else None


def have_live() -> bool:
    try:
        rt.find_binary("gnss_live", build_dir())
        return True
    except SystemExit:
        return False


def live_has_flag(flag: str) -> bool:
    return have_live() and rt.binary_supports(rt.find_binary("gnss_live", build_dir()), flag)


class FramingTests(unittest.TestCase):
    def setUp(self) -> None:
        self.rover = rt.StreamPlan.load("rover", FIXTURE / "rover.rtcm", FIXTURE / "rover.idx")
        self.base = rt.StreamPlan.load("base", FIXTURE / "base.rtcm", FIXTURE / "base.idx")

    def test_index_matches_stream(self) -> None:
        self.assertEqual(len(self.rover.epochs), 201)
        self.assertEqual(len(self.base.epochs), 41)
        self.assertAlmostEqual(self.rover.span, 40.0)
        total = sum(len(rt.split_frames(self.rover.chunk(i))) for i in range(len(self.rover.epochs)))
        self.assertEqual(total, 201)  # one MSM7 observation message per epoch
        for plan in (self.rover, self.base):
            for i in range(len(plan.epochs)):
                chunk = plan.chunk(i)
                frames = rt.split_frames(chunk)
                self.assertEqual(sum(len(f) for f in frames), len(chunk))
                for frame in frames:
                    self.assertEqual(rt.crc24q(frame[:-3]), int.from_bytes(frame[-3:], "big"))

    def test_base_carries_station_and_ephemeris(self) -> None:
        types = {rt.frame_type(f) for i in range(len(self.base.epochs)) for f in rt.split_frames(self.base.chunk(i))}
        self.assertIn(1005, types)
        self.assertIn(1019, types)
        self.assertIn(1077, types)

    def test_time_shift_is_consistent(self) -> None:
        for plan in (self.rover, self.base):
            for i in (0, 7, len(plan.epochs) - 1):
                for frame in rt.split_frames(plan.chunk(i)):
                    self.assertEqual(rt.shift_gps_frame_time(frame, 0), frame)
                    shifted = rt.shift_gps_frame_time(frame, 304)
                    self.assertEqual(len(shifted), len(frame))
                    self.assertEqual(rt.crc24q(shifted[:-3]), int.from_bytes(shifted[-3:], "big"))
                    self.assertEqual(rt.shift_gps_frame_time(shifted, -304), frame)
                    kind = rt.frame_type(frame)
                    if 1071 <= kind <= 1077:
                        before = rt._get_bits(frame, 24 + 24, 30)
                        after = rt._get_bits(shifted, 24 + 24, 30)
                        self.assertEqual(after - before, 304 * 1000)
                    elif kind == 1019:
                        toe_b, toe_a = rt._get_bits(frame, 24 + 288, 16), rt._get_bits(shifted, 24 + 288, 16)
                        self.assertEqual(toe_a - toe_b, 304 // 16)
                        self.assertNotEqual(rt._get_bits(frame, 24 + 320, 32), rt._get_bits(shifted, 24 + 320, 32))
                    else:
                        self.assertEqual(shifted, frame)

    def test_loop_period_is_a_multiple_of_16(self) -> None:
        period = rt.loop_period([self.rover, self.base], gap=60.0)
        self.assertEqual(period % 16, 0)
        self.assertGreater(period, 40.0 + 60.0)


class FaultTests(unittest.TestCase):
    def test_parse(self) -> None:
        f = rt.Fault.parse("corrupt_base@60+30=0.05")
        self.assertEqual((f.kind, f.at, f.dur, f.value), ("corrupt_base", 60.0, 30.0, 0.05))
        f = rt.Fault.parse("base_outage@120+10")
        self.assertEqual((f.kind, f.at, f.dur, f.value), ("base_outage", 120.0, 10.0, 0.0))
        with self.assertRaises(ValueError):
            rt.Fault.parse("nonsense@1+1")

    def test_server_fault_transformations(self) -> None:
        plan = rt.StreamPlan.load("base", FIXTURE / "base.rtcm", FIXTURE / "base.idx")
        clock = rt.StreamClock(1.0)
        faults = [rt.Fault("base_outage", 10.0, 5.0), rt.Fault("base_latency", 20.0, 0.0, 2.0),
                  rt.Fault("corrupt_base", 30.0, 0.0, 1.0), rt.Fault("base_stall", 5.0, 3.0)]
        srv = rt.StreamServer("base", plan, clock, faults)
        try:
            chunk = plan.chunk(12)
            meta: dict = {}
            payload, delay = srv._apply_faults(12.0, chunk, meta)
            self.assertIsNone(payload)
            self.assertEqual(meta["fault"], "base_outage")
            payload, delay = srv._apply_faults(1.0, chunk, {})
            self.assertEqual((payload, delay), (chunk, 0.0))
            payload, delay = srv._apply_faults(6.0, chunk, {})
            self.assertAlmostEqual(delay, 2.0)  # held until the stall window closes at t=8
            payload, delay = srv._apply_faults(21.0, chunk, {})
            self.assertAlmostEqual(delay, 2.0)
            meta = {}
            payload, _ = srv._apply_faults(31.0, chunk, meta)
            self.assertNotEqual(payload, chunk)
            self.assertEqual(len(payload), len(chunk))
            bad = [f for f in rt.split_frames(payload) if rt.crc24q(f[:-3]) != int.from_bytes(f[-3:], "big")]
            self.assertEqual(len(bad), meta["corrupted_frames"])
            self.assertGreaterEqual(len(bad), 1)
        finally:
            srv.stop()

    def test_blackhole_parse(self) -> None:
        f = rt.Fault.parse("base_blackhole@100")
        self.assertEqual((f.kind, f.at, f.dur, f.value), ("base_blackhole", 100.0, 0.0, 0.0))
        self.assertIn("rover_blackhole", rt.FAULT_KINDS)
        with self.assertRaises(ValueError):
            rt.Fault.parse("rover_blackhole@100+10")  # open ended: dur must be 0

    def test_blackhole_keeps_the_connection_open_and_silent(self) -> None:
        import socket
        plan = rt.StreamPlan.load("base", FIXTURE / "base.rtcm", FIXTURE / "base.idx")
        clock = rt.StreamClock(1.0)
        srv = rt.StreamServer("base", plan, clock, [rt.Fault("base_blackhole", 10.0)])

        def event(stream_t: float) -> rt._Event:
            meta = {"loop": 0, "index": 0, "tow": 0.0, "stream_t": stream_t, "sched_wall": 0.0, "bytes": 3}
            return rt._Event(0.0, 0, b"abc", meta)

        server_end, client_end = socket.socketpair()
        client_end.settimeout(0.2)
        try:
            srv._client, srv._client_since = server_end, -math.inf
            self.assertFalse(srv._is_blackholed(9.0))
            self.assertTrue(srv._is_blackholed(10.0))
            srv._send(event(9.0))
            self.assertEqual(client_end.recv(16), b"abc")
            srv._send(event(11.0))
            with self.assertRaises(socket.timeout):
                client_end.recv(16)  # silent, but not closed (a close would return b"")
            self.assertEqual(srv.counters["epochs_blackholed"], 1)
            self.assertEqual(srv.log[-1]["dropped_reason"], "blackhole")
            # a connection made after the fault is served normally
            srv._client_since = 12.0
            self.assertFalse(srv._is_blackholed(13.0))
            srv._send(event(13.0))
            self.assertEqual(client_end.recv(16), b"abc")
            # replacing a silenced connection holds it open instead of closing it
            srv._client_since = -math.inf
            clock.t0 = time.monotonic() - 20.0
            clock.started.set()
            srv._drop_client(keep_if_blackholed=True)
            self.assertEqual(srv.counters["connections_blackholed"], 1)
            with self.assertRaises(socket.timeout):
                client_end.recv(16)
            srv._close_held()
            self.assertEqual(client_end.recv(16), b"")
        finally:
            srv.stop()
            client_end.close()
            server_end.close()

    def test_stats_helpers(self) -> None:
        self.assertAlmostEqual(rt.percentile([1.0, 2.0, 3.0, 4.0], 0.5), 2.5)
        self.assertIsNone(rt.percentile([], 0.5))
        slope = rt.linear_slope([0.0, 1.0, 2.0, 3.0], [1.0, 3.0, 5.0, 7.0])
        self.assertAlmostEqual(slope, 2.0)
        stats = rt.error_stats([(0.0, 0.0, 0.0, 4), (3.0, 4.0, 0.0, 3)])
        self.assertAlmostEqual(stats["fix_rate"], 0.5)
        self.assertAlmostEqual(stats["h_max"], 5.0)
        a = {1000: (1.0, 2.0, 3.0, 4, 8), 2000: (1.0, 2.0, 3.0, 3, 8), 3000: (0.0, 0.0, 0.0, 1, 5)}
        b = {1000: (1.0, 2.0, 3.01, 4, 8), 2000: (1.0, 2.0, 5.0, 4, 8), 4000: (0.0, 0.0, 0.0, 1, 5)}
        cmp_ = rt.compare_to_batch(a, b)
        self.assertEqual(cmp_["common_epochs"], 2)
        self.assertEqual(cmp_["live_only_epochs"], 1)
        self.assertEqual(cmp_["batch_only_epochs"], 1)
        self.assertAlmostEqual(cmp_["status_agreement"], 0.5)
        self.assertEqual(cmp_["pos_diff_both_fixed"]["count"], 1)

    def test_reference_enu_error(self) -> None:
        ref = rt.Reference(FIXTURE / "reference.csv")
        rx, ry, rz, lat, lon = ref.points[int(round(187470.0 * 10))]
        err = ref.enu_error(187470.0, rx, ry, rz)
        self.assertEqual(err, (0.0, 0.0, 0.0))
        # 1 m in the local up direction
        up = (math.cos(lat) * math.cos(lon), math.cos(lat) * math.sin(lon), math.sin(lat))
        e, n, u = ref.enu_error(187470.0, rx + up[0], ry + up[1], rz + up[2])
        self.assertAlmostEqual(u, 1.0, places=6)
        self.assertAlmostEqual(e, 0.0, places=6)
        self.assertAlmostEqual(n, 0.0, places=6)


@unittest.skipUnless(have_live(), "gnss_live is not built")
class PrepareEphSystemsTests(unittest.TestCase):
    """`prepare --eph-systems` reaches the base encode only, and its default leaves the command unchanged."""

    def _run_prepare(self, extra: list[str]) -> tuple[list[list[str]], dict]:
        import subprocess
        from unittest import mock

        commands: list[list[str]] = []

        def fake_run(cmd, **kwargs):
            commands.append(list(cmd))
            summary = {"epochs_written": 3}
            return subprocess.CompletedProcess(cmd, 0, stdout=json.dumps(summary) + "\n", stderr="")

        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp) / "run"
            run_dir.mkdir()
            for name in ("rover.obs", "base.obs", "base.nav"):
                (run_dir / name).write_text("x")
            out_dir = Path(tmp) / "out"
            args = cli.build_parser().parse_args(
                ["prepare", "--run-dir", str(run_dir), "--out-dir", str(out_dir), *extra])
            with mock.patch.object(cli, "find_binary", return_value="gnss_rtcm_encode"), \
                    mock.patch.object(cli.subprocess, "run", side_effect=fake_run), \
                    mock.patch("builtins.print"):
                self.assertEqual(cli.prepare(args), 0)
            manifest = json.loads((out_dir / "manifest.json").read_text())
            commands = [[token.replace(tmp, "<tmp>") for token in cmd] for cmd in commands]
        return commands, manifest

    def test_default_leaves_the_command_line_unchanged(self) -> None:
        commands, manifest = self._run_prepare(["--systems", "G,R,E,J,C"])
        self.assertEqual(len(commands), 2)
        for cmd in commands:
            self.assertNotIn("--eph-systems", cmd)
        self.assertNotIn("eph_systems", manifest)

    def test_eph_systems_goes_to_the_base_encode_only(self) -> None:
        commands, manifest = self._run_prepare(["--systems", "G,R,E,J,C", "--eph-systems", "G,R,E,J,C"])
        rover, base = commands
        self.assertNotIn("--eph-systems", rover)
        self.assertEqual(base[base.index("--eph-systems") + 1], "G,R,E,J,C")
        self.assertEqual(manifest["eph_systems"], "G,R,E,J,C")
        # everything else on the base command line is what the default run builds
        default_base = self._run_prepare(["--systems", "G,R,E,J,C"])[0][1]
        stripped = list(base)
        i = stripped.index("--eph-systems")
        del stripped[i:i + 2]
        self.assertEqual(stripped, default_base)


class EncoderEphSystemsOptionTests(unittest.TestCase):
    """`gnss_rtcm_encode --eph-systems` argument handling (needs a built gnss_rtcm_encode)."""

    def setUp(self) -> None:
        try:
            self.encode = rt.find_binary("gnss_rtcm_encode", build_dir())
        except SystemExit:
            self.skipTest("gnss_rtcm_encode is not built")

    def _run(self, *args: str):
        import subprocess

        return subprocess.run([self.encode, "--obs", "missing.obs", "--out", "unused.rtcm", *args],
                              capture_output=True, text=True, timeout=30)

    def test_rejects_unknown_system_letter(self) -> None:
        done = self._run("--eph-systems", "G,X")
        self.assertEqual(done.returncode, 1)
        self.assertIn("unsupported --eph-systems entry: X", done.stderr)

    def test_rejects_empty_list(self) -> None:
        done = self._run("--eph-systems", ",")
        self.assertEqual(done.returncode, 1)
        self.assertIn("--eph-systems needs at least one", done.stderr)

    def test_help_lists_the_option(self) -> None:
        import subprocess

        done = subprocess.run([self.encode, "--help"], capture_output=True, text=True, timeout=30)
        self.assertIn("--eph-systems", done.stdout + done.stderr)


class SmokeSuiteTests(unittest.TestCase):
    """Runs the smoke preset (every fault kind) against the real gnss_live at 20x on the fixture."""

    @classmethod
    def setUpClass(cls) -> None:
        cls.tmp = tempfile.TemporaryDirectory(prefix="gnss_rt_smoke_")
        out = Path(cls.tmp.name) / "suite"
        argv = ["--bin-dir", build_dir() or "", "suite", "--prepared", str(FIXTURE), "--out-dir", str(out),
                "--speed", "20", "--preset", "smoke", "--jobs", "4"]
        cls.rc = cli.main(argv)
        cls.summary = json.loads((out / "suite_summary.json").read_text())["scenarios"]
        cls.results = {p.name: json.loads((p / "result.json").read_text()) for p in out.iterdir() if p.is_dir()}

    @classmethod
    def tearDownClass(cls) -> None:
        cls.tmp.cleanup()

    def test_baseline_survives_and_matches_input(self) -> None:
        base = self.results["baseline"]
        self.assertTrue(base["survived_to_end"])
        self.assertEqual(base["input"]["rover_epochs_sent"], 201)
        self.assertGreaterEqual(base["output_epochs"], 185)
        self.assertEqual(base["restarts"], 0)
        self.assertGreater(base["accuracy"]["all"]["fix_rate"], 0.8)
        self.assertLess(base["accuracy"]["all"]["d3_p50"], 0.5)
        self.assertLess(base["latency_s"]["max"], 1.0)
        self.assertGreater(base["resources"]["rss_peak_mb"], 1.0)

    def test_every_injected_fault_is_detected(self) -> None:
        for name, row in self.summary.items():
            if name == "baseline":
                continue
            self.assertTrue(row["faults"], name)
            self.assertTrue(row["faults_detected_all"], f"{name}: {json.dumps(row['faults'], indent=1)}")

    def test_stream_faults_do_not_kill_gnss_live(self) -> None:
        for name in ("base_outage", "base_outage_long", "base_stall", "base_latency", "base_jitter",
                     "corrupt_base", "corrupt_rover", "combined"):
            r = self.results[name]
            self.assertTrue(r["survived_to_end"], f"{name}: {r['process']}")
            self.assertGreater(r["output_epochs"], 100, name)

    def test_corruption_is_counted_by_the_decoder(self) -> None:
        self.assertGreater(self.results["corrupt_base"]["live_totals"]["base_decoder_errors"], 0)
        self.assertGreater(self.results["corrupt_rover"]["live_totals"]["rover_decoder_errors"], 0)
        self.assertGreater(self.results["corrupt_base"]["input"]["base_counters"]["frames_corrupted"], 0)

    def test_outage_drops_epochs_and_jitter_reorders(self) -> None:
        base = self.results["baseline"]["output_epochs"]
        self.assertLess(self.results["base_outage_long"]["output_epochs"], base)
        self.assertGreater(self.results["base_jitter"]["input"]["base_counters"]["epochs_reordered"], 0)
        self.assertGreater(self.results["base_latency"]["latency_s"]["p50"], self.results["baseline"]["latency_s"]["p50"])

    def test_link_loss_ends_gnss_live_unless_it_reconnects(self) -> None:
        for name in ("rover_disconnect", "server_restart"):
            r = self.results[name]
            self.assertFalse(r["survived_to_end"], name)
            self.assertTrue(r["faults"][0]["detected"], name)
        supervised = self.results["rover_disconnect_supervised"]
        self.assertEqual(supervised["restarts"], 1)
        self.assertGreater(supervised["output_epochs"], self.results["rover_disconnect"]["output_epochs"])
        for name in ("rover_disconnect_reconnect", "server_restart_reconnect"):
            r = self.results[name]
            self.assertTrue(r["survived_to_end"], f"{name}: {r['process']}")
            self.assertIn("client_reconnected", r["faults"][0]["evidence"])
            self.assertIsNotNone(r["faults"][0]["recovery_float_s"])

    def test_ntrip_transport_replays_the_same_epochs(self) -> None:
        out = Path(self.tmp.name) / "ntrip"
        rc = cli.main(["--bin-dir", build_dir() or "", "suite", "--prepared", str(FIXTURE), "--out-dir", str(out),
                       "--speed", "20", "--preset", "faults_ntrip", "--only", "baseline"])
        self.assertEqual(rc, 0)
        ntrip = json.loads((out / "baseline" / "result.json").read_text())
        self.assertTrue(ntrip["survived_to_end"], ntrip["process"])
        self.assertEqual(ntrip["output_epochs"], self.results["baseline"]["output_epochs"])
        self.assertEqual(ntrip["input"]["rover_counters"]["connections"], 1)

    def _run_blackhole(self, name: str, fault: str, watchdog: bool) -> dict:
        out = Path(self.tmp.name) / name
        argv = ["--bin-dir", build_dir() or "", "run", "--prepared", str(FIXTURE), "--out-dir", str(out),
                "--speed", "20", "--name", name, "--fault", fault, "--allow-failure"]
        if watchdog:
            # 300 ms of wall time is 6 stream seconds at 20x
            argv += ["--reconnect", "--live-arg=--stall-timeout-ms", "--live-arg=300"]
        self.assertEqual(cli.main(argv), 0)
        return json.loads((out / "result.json").read_text())

    @unittest.skipUnless(live_has_flag("--stall-timeout-ms"), "gnss_live has no --stall-timeout-ms")
    def test_blackhole_hangs_without_the_watchdog_and_recovers_with_it(self) -> None:
        for target in ("base", "rover"):
            fault = f"{target}_blackhole@10"
            plain = self._run_blackhole(f"{target}_blackhole_plain", fault, watchdog=False)
            self.assertIsNone(plain["faults"][0]["first_solution_after_s"], plain["faults"][0])
            self.assertGreater(plain["input"][f"{target}_counters"]["epochs_blackholed"], 0)
            guarded = self._run_blackhole(f"{target}_blackhole_watchdog", fault, watchdog=True)
            rep = guarded["faults"][0]
            self.assertTrue(guarded["survived_to_end"], guarded["process"])
            self.assertTrue(rep["detected"], rep)
            self.assertGreaterEqual(guarded["live_totals"][f"{target}_stalls"], 1)
            self.assertGreaterEqual(guarded["input"][f"{target}_counters"]["connections"], 2)
            self.assertIsNotNone(rep["first_solution_after_s"])
            self.assertLess(rep["first_solution_after_s"], 20.0)
            self.assertGreater(guarded["output_epochs"], plain["output_epochs"])

    def test_suite_exit_code_and_files(self) -> None:
        self.assertEqual(self.rc, 0)
        run_dir = Path(self.tmp.name) / "suite" / "baseline"
        for name in ("result.json", "epochs.csv", "resources.csv", "server_rover.csv", "server_base.csv",
                     "solution.0.pos", "gnss_live.0.log"):
            self.assertTrue((run_dir / name).exists(), name)


if __name__ == "__main__":
    unittest.main()
