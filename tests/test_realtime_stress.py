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

    def test_suite_exit_code_and_files(self) -> None:
        self.assertEqual(self.rc, 0)
        run_dir = Path(self.tmp.name) / "suite" / "baseline"
        for name in ("result.json", "epochs.csv", "resources.csv", "server_rover.csv", "server_base.csv",
                     "solution.0.pos", "gnss_live.0.log"):
            self.assertTrue((run_dir / name).exists(), name)


if __name__ == "__main__":
    unittest.main()
