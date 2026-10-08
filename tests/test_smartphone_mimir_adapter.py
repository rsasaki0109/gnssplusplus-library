#!/usr/bin/env python3
"""Fail-closed tests for the Mimir (Nantes) Raw.csv/PSR.csv adapter and the
barometer evaluation helpers.  Pure Python; no dataset access."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
BENCH = ROOT / "apps" / "commands" / "benchmarks"
sys.path.insert(0, str(BENCH))

import gnss_smartphone_baro_eval as ev  # noqa: E402
import gnss_smartphone_mimir_adapter as mimir  # noqa: E402

GNSS = ROOT / "apps" / "gnss.py"
C = 299_792_458.0
WEEK_NS = 604_800 * 10**9
FULL_BIAS = -1_394_444_862_161_999_342  # real Pixel 7 magnitude (week 2305)


def raw_line(
    *,
    time_nanos: int,
    utc_ms: int,
    svid: int = 6,
    state: int = 16431,
    constellation: int = 1,
    carrier: str = "1.57542003E9",
    travel_s: float = 0.075,
    rx_unc: float = 19.0,
    clock_count: int = 71,
) -> list[str]:
    arrival_ns = time_nanos - FULL_BIAS  # BiasNanos = 0, TimeOffsetNanos = 0
    tow_ns = arrival_ns % WEEK_NS
    tx_ns = tow_ns - int(travel_s * 1e9)
    values = dict.fromkeys(mimir.MIMIR_RAW_FIELDS, "")
    values.update(
        {
            "Raw": "Raw",
            "utcTimeMillis": str(utc_ms),
            "TimeNanos": str(time_nanos),
            "FullBiasNanos": str(FULL_BIAS),
            "BiasNanos": "0.0",
            "HardwareClockDiscontinuityCount": str(clock_count),
            "Svid": str(svid),
            "TimeOffsetNanos": "0.0",
            "State": str(state),
            "ReceivedSvTimeNanos": str(tx_ns),
            "ReceivedSvTimeUncertaintyNanos": str(rx_unc),
            "Cn0DbHz": "35.5",
            "PseudorangeRateMetersPerSecond": "288.1",
            "AccumulatedDeltaRangeState": "16",
            "AccumulatedDeltaRangeMeters": "103.1",
            "CarrierFrequencyHz": carrier,
            "ConstellationType": str(constellation),
        }
    )
    return [values[f] for f in mimir.MIMIR_RAW_FIELDS]


class MimirUnitTests(unittest.TestCase):
    def test_state_usable_rules(self) -> None:
        self.assertTrue(mimir.state_usable("1", 16431))      # code lock + TOW decoded + known
        self.assertTrue(mimir.state_usable("1", 16423))      # TOW_KNOWN, no decode
        self.assertFalse(mimir.state_usable("1", 17))        # msec ambiguous
        self.assertFalse(mimir.state_usable("1", 16384))     # TOW known but no code lock
        self.assertFalse(mimir.state_usable("1", 16396))     # no code lock
        self.assertTrue(mimir.state_usable("6", 85026))      # E1BC lock + TOW_KNOWN
        self.assertFalse(mimir.state_usable("6", 1074))      # E1 lock but TOW unknown
        self.assertFalse(mimir.state_usable("5", 16431))     # BeiDou not mapped

    def test_signal_classification_tolerance(self) -> None:
        self.assertEqual(mimir.classify_signal("1", 1_575_420_030.0), "GPS_L1_CA")
        self.assertEqual(mimir.classify_signal("6", 1_575_420_030.0), "GAL_E1_C_P")
        self.assertIsNone(mimir.classify_signal("1", 1_176_450_000.0))   # L5
        self.assertIsNone(mimir.classify_signal("1", 1_575_500_000.0))   # 80 kHz off
        self.assertIsNone(mimir.classify_signal("3", 1_602_000_000.0))   # GLONASS
        self.assertIsNone(mimir.classify_signal("1", None))

    def test_pseudorange_matches_travel_time_and_wraps_week(self) -> None:
        row = dict(zip(mimir.MIMIR_RAW_FIELDS, raw_line(time_nanos=1_105_000_000, utc_ms=1)))
        pr, arrival = mimir.pseudorange_from_row(row, 1)
        self.assertAlmostEqual(pr, 0.075 * C, delta=0.5)
        self.assertAlmostEqual(arrival, 1_105_000_000 - FULL_BIAS, delta=512.0)
        # Receiver time just after the week boundary, transmit time just before.
        base = {k: "0" for k in mimir.MIMIR_RAW_FIELDS}
        base.update({"TimeNanos": "0", "FullBiasNanos": str(-(WEEK_NS + 10_000_000)),
                     "BiasNanos": "0", "TimeOffsetNanos": "0",
                     "ReceivedSvTimeNanos": str(WEEK_NS - 65_000_000)})
        pr2, _ = mimir.pseudorange_from_row(base, 1)
        self.assertAlmostEqual(pr2 / C, 0.075, places=6)

    def test_implausible_travel_time_dropped(self) -> None:
        row = dict(zip(mimir.MIMIR_RAW_FIELDS, raw_line(time_nanos=1_105_000_000, utc_ms=1,
                                                       travel_s=0.5)))
        self.assertEqual(mimir.pseudorange_from_row(row, 1), (None, None))

    def test_wrong_field_count_fails_closed(self) -> None:
        with self.assertRaises(SystemExit):
            mimir.parse_raw_line(["Raw", "1", "2"], 1)
        with self.assertRaises(SystemExit):
            mimir.parse_raw_line(["Nope"] + [""] * (len(mimir.MIMIR_RAW_FIELDS) - 1), 1)

    def test_psr_reader_rejects_malformed_and_drops_out_of_range(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / "PSR.csv"
            p.write_text("PSR,1000,5,1010.5,3\nPSR,1100,6,50.0,3\nPSR,1200,7,1010.6,3\n")
            rows, counts = mimir.read_psr(p)
            self.assertEqual(len(rows), 2)
            self.assertEqual(counts["dropped_out_of_range"], 1)
            p.write_text("PSR,1000,5,1010.5\n")
            with self.assertRaises(SystemExit):
                mimir.read_psr(p)
            p.write_text("PSR,1000,5,1010.5,3\nPSR,900,6,1010.5,3\n")
            with self.assertRaises(SystemExit):  # time moved backwards
                mimir.read_psr(p)
            p.write_text("")
            with self.assertRaises(SystemExit):
                mimir.read_psr(p)


class MimirCliTests(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)

    def tearDown(self) -> None:
        self.tmp.cleanup()

    def write_raw(self, rows: list[list[str]]) -> Path:
        path = self.root / "Raw.csv"
        with path.open("w", newline="") as handle:
            csv.writer(handle, lineterminator="\n").writerows(rows)
        return path

    def run_adapter(self, raw: Path, extra: tuple[str, ...] = ()) -> subprocess.CompletedProcess[str]:
        psr = self.root / "PSR.csv"
        if not psr.exists():
            psr.write_text("PSR,1710409645964,1,1010.74,3\nPSR,1710409647001,2,1010.70,3\n")
        return subprocess.run(
            [sys.executable, str(GNSS), "smartphone-mimir-adapter", "--raw", str(raw),
             "--psr", str(psr), "--output-dir", str(self.root / "out"),
             "--dataset-id", "t", "--source-url", "u", "--source-terms", "CC-BY-4.0",
             "--approx-llh", "47.2177,-1.5426,59.5", *extra],
            capture_output=True, text=True, check=False,
        )

    def epoch(self, k: int, **kw) -> list[list[str]]:
        t = 1_105_000_000 + k * 1_000_000_000
        utc = 1_710_409_647_704 + k * 1000
        return [raw_line(time_nanos=t, utc_ms=utc, svid=s, **kw) for s in (6, 11, 12, 24)]

    def test_valid_run_accounts_for_every_row(self) -> None:
        rows = []
        for k in range(3):
            rows += self.epoch(k)
        rows.append(raw_line(time_nanos=1_105_000_000 + 2_000_000_000, utc_ms=1_710_409_649_704,
                             svid=3, state=17))                       # ambiguous -> state_not_usable
        rows.append(raw_line(time_nanos=1_105_000_000 + 2_000_000_000, utc_ms=1_710_409_649_704,
                             svid=4, carrier="1.17645E9"))             # L5 -> unsupported
        # keep rows of an epoch contiguous
        rows.sort(key=lambda r: int(r[mimir.MIMIR_RAW_FIELDS.index("TimeNanos")]))
        result = self.run_adapter(self.write_raw(rows))
        self.assertEqual(result.returncode, 0, result.stderr)
        summary = json.loads((self.root / "out" / "summary.json").read_text())
        obs = summary["observations"]
        self.assertEqual(obs["rows"], len(rows))
        self.assertEqual(sum(obs["row_dispositions"].values()), len(rows))
        self.assertEqual(obs["row_dispositions"]["used"], 12)
        self.assertEqual(obs["row_dispositions"]["state_not_usable"], 1)
        self.assertEqual(obs["row_dispositions"]["unsupported_signal"], 1)
        self.assertEqual(obs["epochs_with_usable_rows"], 3)
        rinex = (self.root / "out" / "rover.obs").read_text()
        self.assertEqual(sum(1 for l in rinex.splitlines() if l.startswith(">")), 3)
        self.assertIn("G06", rinex)
        # lossless: every source row is preserved verbatim + disposition
        with (self.root / "out" / "observations.csv").open() as handle:
            reader = list(csv.reader(handle))
        self.assertEqual(len(reader) - 1, len(rows))
        self.assertEqual(reader[1][: len(mimir.MIMIR_RAW_FIELDS)], rows[0])
        # barometer file is GPST and hashes are recorded
        baro = list((self.root / "out" / "baro.csv").read_text().splitlines())
        self.assertEqual(baro[0].split(",")[:3], ["gps_week", "gps_tow_s", "pressure_hpa"])
        self.assertEqual(len(baro), 3)
        self.assertEqual(len(summary["artifact_sha256"]["rinex"]), 64)

    def test_malformed_raw_publishes_nothing(self) -> None:
        rows = self.epoch(0) + [["Raw", "1", "2"]]
        result = self.run_adapter(self.write_raw(rows))
        self.assertNotEqual(result.returncode, 0)
        self.assertFalse((self.root / "out" / "summary.json").exists())
        self.assertFalse((self.root / "out" / "rover.obs").exists())

    def test_backwards_time_and_clock_discontinuity_fail(self) -> None:
        rows = self.epoch(1) + self.epoch(0)
        self.assertNotEqual(self.run_adapter(self.write_raw(rows)).returncode, 0)
        rows = self.epoch(0) + self.epoch(1, clock_count=70)
        self.assertNotEqual(self.run_adapter(self.write_raw(rows)).returncode, 0)

    def test_galileo_requires_navigation_and_missing_inputs_fail(self) -> None:
        raw = self.write_raw(self.epoch(0))
        self.assertNotEqual(self.run_adapter(raw, ("--enable-galileo-e1",)).returncode, 0)
        self.assertNotEqual(
            self.run_adapter(raw, ("--broadcast-nav", str(self.root / "x.rnx"))).returncode, 0
        )
        self.assertNotEqual(self.run_adapter(self.root / "missing.csv").returncode, 0)

    def test_no_usable_rows_fails(self) -> None:
        rows = self.epoch(0, state=17)
        self.assertNotEqual(self.run_adapter(self.write_raw(rows)).returncode, 0)

    def test_inconsistent_phone_clock_fails(self) -> None:
        rows = self.epoch(0)
        rows += [raw_line(time_nanos=1_105_000_000 + k * 10**9,
                          utc_ms=1_710_409_647_704 + k * 1000 + (30_000 if k % 2 else 0), svid=6)
                 for k in range(1, 40)]
        self.assertNotEqual(self.run_adapter(self.write_raw(rows)).returncode, 0)


class BaroEvalTests(unittest.TestCase):
    def reference(self) -> ev.Reference:
        rows = [(1000.0 + i / 60.0, 47.0 + 1e-6 * i, -1.5, 50.0 + (10.0 if i > 600 else 0.0))
                for i in range(1200)]
        return ev.Reference(rows, lag_s=0.0)

    def test_horizontal_and_vertical_error_alignment(self) -> None:
        ref = self.reference()
        sol = []
        for k in range(1, 18):
            tow = 1000.0 + k
            r = ref.at(tow)
            lat = 47.0 + 1e-6 * (tow - 1000.0) * 60.0
            sol.append((tow, lat + 5.0 / (ev.EARTH_RADIUS_M * math.pi / 180.0), -1.5,
                        r[2] + 3.0, 8))
        result = ev.score(sol, ref, [1000.0 + k for k in range(1, 18)])
        self.assertAlmostEqual(result["horizontal_m"]["rmse"], 5.0, places=1)
        self.assertAlmostEqual(result["vertical_raw_m"]["mean"], 3.0, places=6)
        self.assertAlmostEqual(result["vertical_demeaned_m"]["rmse"], 0.0, places=6)
        self.assertEqual(result["availability"], 1.0)

    def test_lag_shifts_reference_time(self) -> None:
        rows = [(1000.0 + i / 60.0, 47.0 + 1e-6 * i, -1.5, 50.0) for i in range(1200)]
        a = ev.Reference(rows, lag_s=0.0).at(1005.0)
        b = ev.Reference(rows, lag_s=2.0).at(1005.0)
        c = ev.Reference(rows, lag_s=0.0).at(1007.0)
        self.assertEqual(b, c)
        self.assertNotEqual(a, b)

    def test_outside_reference_span_is_not_scored(self) -> None:
        ref = self.reference()
        self.assertIsNone(ref.at(900.0))
        self.assertIsNone(ref.at(1100.0))

    def test_pooled_summary_counts_and_availability(self) -> None:
        run = [{"tow": float(i), "dh": 2.0, "dv_raw": 1.0, "ref_alt": 50.0,
                "h": 51.0, "x": 0.0, "y": float(i)} for i in range(10)]
        out = ev.summarize([run, run], window_epochs=[20, 10], common_offsets=[0.0, 0.0],
                           start_alts=[50.0, 50.0], level_split_m=3.0)
        self.assertEqual(out["matched_epochs"], 20)
        self.assertAlmostEqual(out["availability"], 20 / 30)
        self.assertEqual(out["horizontal_m"]["rmse"], 2.0)


if __name__ == "__main__":
    unittest.main()
