#!/usr/bin/env python3
"""Multi-signal tests for the Mimir adapter: frequency -> signal mapping,
constellation time bases, rejection accounting and the OFF (legacy) contract.
Pure Python; no dataset access."""

from __future__ import annotations

import csv
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
BENCH = ROOT / "apps" / "commands" / "benchmarks"
sys.path.insert(0, str(BENCH))
sys.path.insert(0, str(ROOT / "tests"))

import gnss_smartphone_mimir_adapter as mimir  # noqa: E402
import gnss_smartphone_mimir_signals as sigs  # noqa: E402
from test_smartphone_mimir_adapter import FULL_BIAS, WEEK_NS, raw_line  # noqa: E402

GNSS = ROOT / "apps" / "gnss.py"
DAY_NS = 86_400 * 10**9


class SignalMappingTests(unittest.TestCase):
    def test_gps_galileo_l1_and_l5_by_constellation(self) -> None:
        for con, name_l1, name_l5 in (("1", "GPS_L1_CA", "GPS_L5_Q"), ("6", "GAL_E1_C_P", "GAL_E5A_Q")):
            spec, fcn, reason = sigs.classify_multi(con, 1_575_420_030.0)  # Android +30 Hz
            self.assertEqual((spec.name, fcn, reason), (name_l1, None, ""))
            spec, _, _ = sigs.classify_multi(con, 1_176_450_050.0)
            self.assertEqual(spec.name, name_l5)
        self.assertEqual(sigs.SIGNAL_BY_NAME["GPS_L5_Q"].obs_codes, ("C5Q", "L5Q", "D5Q", "S5Q"))
        self.assertEqual(sigs.SIGNAL_BY_NAME["GAL_E5A_Q"].obs_codes[0], "C5Q")

    def test_beidou_bands_use_rinex3_codes(self) -> None:
        self.assertEqual(sigs.classify_multi("5", 1_561_097_980.0)[0].obs_codes[0], "C2I")  # B1I
        self.assertEqual(sigs.classify_multi("5", 1_176_450_000.0)[0].name, "BDS_B2A")
        self.assertEqual(sigs.classify_multi("5", 1_575_420_000.0)[0].name, "BDS_B1C")

    def test_qzss_l1_l5(self) -> None:
        self.assertEqual(sigs.classify_multi("4", 1_575_420_000.0)[0].name, "QZS_L1_CA")
        self.assertEqual(sigs.classify_multi("4", 1_176_450_000.0)[0].name, "QZS_L5")
        spec = sigs.SIGNAL_BY_NAME["QZS_L1_CA"]
        self.assertEqual(sigs.prn_from_svid(spec, 194), 2)  # Android QZSS Svid 193..202
        self.assertIsNone(sigs.prn_from_svid(spec, 2))

    def test_glonass_fdma_channel_from_carrier(self) -> None:
        for k in range(-7, 7):
            carrier = 1_602_000_000.0 + k * 562_500.0 + 40.0
            spec, fcn, reason = sigs.classify_multi("3", carrier)
            self.assertEqual((spec.name, fcn, reason), ("GLO_G1_CA", k, ""))
        # beyond the channel plan or off the FDMA grid is a rejected frequency
        self.assertEqual(sigs.classify_multi("3", 1_602_000_000.0 + 7 * 562_500.0)[2],
                         "unsupported_frequency")
        self.assertEqual(sigs.classify_multi("3", 1_602_000_000.0 + 100_000.0)[2],
                         "unsupported_frequency")

    def test_rejections_carry_a_reason(self) -> None:
        self.assertEqual(sigs.classify_multi("7", 1_176_450_000.0), (None, None, "unsupported_constellation"))
        self.assertEqual(sigs.classify_multi("2", 1_575_420_000.0)[2], "unsupported_constellation")
        self.assertEqual(sigs.classify_multi("1", 1_227_600_000.0)[2], "unsupported_frequency")  # L2
        self.assertEqual(sigs.classify_multi("1", 1_575_500_000.0)[2], "unsupported_frequency")  # 80 kHz off
        self.assertEqual(sigs.classify_multi("1", None)[2], "unsupported_frequency")
        self.assertEqual(sigs.classify_multi("6", 1_207_140_000.0)[2], "unsupported_frequency")  # E5b

    def test_svid_ranges(self) -> None:
        gps = sigs.SIGNAL_BY_NAME["GPS_L1_CA"]
        self.assertEqual(sigs.prn_from_svid(gps, 32), 32)
        self.assertIsNone(sigs.prn_from_svid(gps, 33))
        self.assertIsNone(sigs.prn_from_svid(gps, 0))
        self.assertIsNone(sigs.prn_from_svid(sigs.SIGNAL_BY_NAME["GLO_G1_CA"], 93))

    def test_state_rules_per_constellation(self) -> None:
        self.assertTrue(sigs.state_usable_multi("1", 16431))
        self.assertFalse(sigs.state_usable_multi("1", 17))      # ms ambiguous
        self.assertTrue(sigs.state_usable_multi("5", 81967))    # BDS code lock + TOW known
        self.assertFalse(sigs.state_usable_multi("5", 16384))   # no code lock
        self.assertTrue(sigs.state_usable_multi("3", 32995))    # GLO code lock + TOD known/decoded
        self.assertFalse(sigs.state_usable_multi("3", 32768))   # TOD known but no code lock
        self.assertFalse(sigs.state_usable_multi("3", 16431))   # GPS-style TOW is not GLONASS TOD
        self.assertTrue(sigs.state_usable_multi("6", 85026))
        self.assertFalse(sigs.state_usable_multi("7", 16431))   # NavIC not mapped

    def test_time_bases(self) -> None:
        arrival = 380_867 * 10**9 + 123_456_789  # TOW ns
        whole = (2305 * WEEK_NS) + arrival
        travel = 0.0765
        tx_gps = arrival - int(travel * 1e9)
        self.assertAlmostEqual(sigs.travel_time_s("1", whole, 0.0, tx_gps), travel, places=6)
        self.assertAlmostEqual(sigs.travel_time_s("6", whole, 0.0, tx_gps), travel, places=6)
        # BeiDou transmits in BDT = GPST - 14 s
        tx_bdt = arrival - 14 * 10**9 - int(travel * 1e9)
        self.assertAlmostEqual(sigs.travel_time_s("5", whole, 0.0, tx_bdt), travel, places=6)
        self.assertGreater(abs(sigs.travel_time_s("5", whole, 0.0, tx_gps)), 1.0)  # wrong base is caught
        # GLONASS time of day = UTC + 3 h = GPST - 18 s + 3 h
        glo_rx = (arrival - 18 * 10**9 + 3 * 3600 * 10**9) % DAY_NS
        tx_glo = (glo_rx - int(travel * 1e9)) % DAY_NS
        self.assertAlmostEqual(sigs.travel_time_s("3", whole, 0.0, tx_glo), travel, places=6)
        # day roll-over between transmit (20 ms before GLONASS midnight) and receive
        tow = 3 * DAY_NS - (3 * 3600 - 18) * 10**9 + 56_000_000  # GLONASS time of day = 56 ms
        self.assertAlmostEqual(
            sigs.travel_time_s("3", 2305 * WEEK_NS + tow, 0.0, DAY_NS - 20_000_000), 0.076, places=6
        )

    def test_rinex_headers(self) -> None:
        specs = tuple(sigs.SIGNAL_BY_NAME[n] for n in ("GPS_L1_CA", "GPS_L5_Q", "GLO_G1_CA"))
        lines = dict((c[0], c) for c, _ in sigs.rinex_obs_type_lines(specs))
        self.assertEqual(lines["G"].split(), ["G", "8", "C1C", "L1C", "D1C", "S1C", "C5Q", "L5Q", "D5Q", "S5Q"])
        self.assertEqual(lines["R"].split(), ["R", "4", "C1C", "L1C", "D1C", "S1C"])
        slot = sigs.glonass_slot_frq_lines({6: -4, 7: 5})
        self.assertEqual(slot[0][0].rstrip(), "  2 R06 -4 R07  5")
        self.assertEqual(slot[0][1], "GLONASS SLOT / FRQ #")
        nine = sigs.glonass_slot_frq_lines({s: 0 for s in range(1, 10)})
        self.assertEqual(len(nine), 2)  # 8 per header line


def nav_text(entries: list[tuple[str, int, int]]) -> str:
    """Minimal mixed RINEX 3 nav with record head lines (``(sys, prn, fcn)``)."""

    out = ["     3.04           N: GNSS NAV DATA    M: MIXED            RINEX VERSION / TYPE",
           "                                                            END OF HEADER"]
    for system, prn, fcn in entries:
        out.append(f"{system}{prn:02d} 2024 03 14 09 45 00" + " " * 57)
        if system == "R":
            out.append(" " * 80)
            out.append(" " * 61 + f"{float(fcn):19.12E}".replace("E", "E"))
            out.append(" " * 80)
        else:
            out.extend([" " * 80] * 7)
    return "\n".join(out) + "\n"


class MultiCliTests(unittest.TestCase):
    CLEAN = (1_105_000_000, 1_710_409_647_704)

    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)

    def tearDown(self) -> None:
        self.tmp.cleanup()

    def field(self, row: list[str], name: str, value: str) -> list[str]:
        row = list(row)
        row[mimir.MIMIR_RAW_FIELDS.index(name)] = value
        return row

    def glonass_row(self, k: int, **kw) -> list[str]:
        t, utc = self.CLEAN
        row = raw_line(time_nanos=t, utc_ms=utc, svid=kw.pop("svid", 7), constellation=3,
                       carrier=repr(1_602_000_000.0 + k * 562_500.0), state=32995, **kw)
        # re-time the transmit stamp to GLONASS time of day
        arrival = t - FULL_BIAS
        tow = arrival % WEEK_NS
        rx = (tow - 18 * 10**9 + 3 * 3600 * 10**9) % DAY_NS
        return self.field(row, "ReceivedSvTimeNanos", str(rx - 75_000_000))

    def bds_row(self, carrier: str, svid: int = 23) -> list[str]:
        t, utc = self.CLEAN
        row = raw_line(time_nanos=t, utc_ms=utc, svid=svid, constellation=5, carrier=carrier,
                       state=81967)
        arrival = t - FULL_BIAS
        tx = arrival % WEEK_NS - 14 * 10**9 - 75_000_000
        return self.field(row, "ReceivedSvTimeNanos", str(tx))

    def write(self, rows: list[list[str]], nav: str | None = None) -> tuple[Path, Path]:
        raw = self.root / "Raw.csv"
        with raw.open("w", newline="") as handle:
            csv.writer(handle, lineterminator="\n").writerows(rows)
        nav_path = self.root / "brdc.rnx"
        nav_path.write_text(nav if nav is not None else nav_text(
            [("G", 6, 0), ("G", 11, 0), ("E", 4, 0), ("C", 23, 0), ("R", 7, 5)]))
        return raw, nav_path

    def run_adapter(self, raw: Path, *extra: str) -> subprocess.CompletedProcess[str]:
        psr = self.root / "PSR.csv"
        psr.write_text("PSR,1710409645964,1,1010.74,3\nPSR,1710409647001,2,1010.70,3\n")
        return subprocess.run(
            [sys.executable, str(GNSS), "smartphone-mimir-adapter", "--raw", str(raw),
             "--psr", str(psr), "--output-dir", str(self.root / "out"), "--dataset-id", "t",
             "--source-url", "u", "--source-terms", "CC-BY-4.0",
             "--approx-llh", "47.2177,-1.5426,59.5", *extra],
            capture_output=True, text=True, check=False,
        )

    def epoch_rows(self) -> list[list[str]]:
        t, utc = self.CLEAN
        gps_l1 = raw_line(time_nanos=t, utc_ms=utc, svid=6)
        gps_l5 = raw_line(time_nanos=t, utc_ms=utc, svid=6, carrier="1.17645E9")
        gal = raw_line(time_nanos=t, utc_ms=utc, svid=4, constellation=6)
        return [
            gps_l1, gps_l5, gal,
            self.glonass_row(5),
            self.bds_row("1.56109798E9"),
            self.bds_row("1.17645E9"),
            raw_line(time_nanos=t, utc_ms=utc, svid=3, constellation=7, carrier="1.17645E9"),  # NavIC
            raw_line(time_nanos=t, utc_ms=utc, svid=9, carrier="1.2276E9"),                    # GPS L2
            raw_line(time_nanos=t, utc_ms=utc, svid=99),                                       # bad svid
            raw_line(time_nanos=t, utc_ms=utc, svid=20),                                       # no nav
            self.glonass_row(3, svid=9),                                                       # slot absent from nav
            raw_line(time_nanos=t, utc_ms=utc, svid=11, state=17),                              # ms ambiguous
        ]

    def test_multi_accounts_every_row_with_a_reason(self) -> None:
        rows = self.epoch_rows()
        raw, nav = self.write(rows)
        result = self.run_adapter(raw, "--signal-set", "multi", "--broadcast-nav", str(nav))
        self.assertEqual(result.returncode, 0, result.stderr)
        summary = json.loads((self.root / "out" / "summary.json").read_text())
        obs = summary["observations"]
        d = obs["row_dispositions"]
        self.assertEqual(obs["rows"], len(rows))
        self.assertEqual(sum(d.values()), len(rows))
        self.assertEqual(d["used"], 6)
        self.assertEqual(d["unsupported_constellation"], 1)
        self.assertEqual(d["unsupported_frequency"], 1)
        self.assertEqual(d["invalid_svid"], 1)
        self.assertEqual(d["no_navigation"], 2)
        self.assertEqual(d["state_not_usable"], 1)
        self.assertEqual(obs["rejected_rows_detail"]["unsupported_constellation"], {"NavIC@1176.45MHz": 1})
        self.assertEqual(obs["rejected_rows_detail"]["unsupported_frequency"], {"GPS@1227.60MHz": 1})
        self.assertEqual(
            obs["used_signal_rows"],
            {"BDS_B1I": 1, "BDS_B2A": 1, "GAL_E1_C_P": 1, "GLO_G1_CA": 1, "GPS_L1_CA": 1, "GPS_L5_Q": 1},
        )
        self.assertEqual(summary["schema_version"], mimir.SCHEMA_VERSION_MULTI)
        self.assertEqual(summary["navigation"]["glonass_channels_in_rinex_header"], {"7": 5})
        # every source row is preserved with its disposition
        with (self.root / "out" / "observations.csv").open() as handle:
            table = list(csv.DictReader(handle))
        self.assertEqual(len(table), len(rows))
        self.assertEqual(sum(1 for r in table if r["disposition"] == "used"), 6)
        # one RINEX line per satellite, GPS L1+L5 share G06
        rinex = (self.root / "out" / "rover.obs").read_text().splitlines()
        g06 = [l for l in rinex if l.startswith("G06")]
        self.assertEqual(len(g06), 1)
        self.assertEqual(len(g06[0]), 3 + 16 * 8)
        self.assertTrue(g06[0][3:19].strip() and g06[0][3 + 16 * 4:3 + 16 * 5].strip())
        self.assertTrue(any(l.startswith("C23") for l in rinex))
        self.assertTrue(any(l.startswith("R07") for l in rinex))
        self.assertIn("GLONASS SLOT / FRQ #", "\n".join(rinex))
        # pseudoranges are physical for every constellation time base (75 ms)
        for line in rinex:
            if line[:3] in ("G06", "E04", "C23", "R07"):
                self.assertAlmostEqual(float(line[3:17]) / 299_792_458.0, 0.075, delta=1e-4, msg=line[:20])

    def test_glonass_channel_conflict_with_navigation_is_rejected(self) -> None:
        rows = [self.glonass_row(5), raw_line(time_nanos=self.CLEAN[0], utc_ms=self.CLEAN[1], svid=6)]
        raw, nav = self.write(rows, nav_text([("G", 6, 0), ("R", 7, -2)]))  # nav says k=-2, carrier says +5
        result = self.run_adapter(raw, "--signal-set", "multi", "--broadcast-nav", str(nav))
        self.assertEqual(result.returncode, 0, result.stderr)
        obs = json.loads((self.root / "out" / "summary.json").read_text())["observations"]
        self.assertEqual(obs["row_dispositions"]["glonass_fcn_conflict"], 1)
        self.assertEqual(obs["row_dispositions"]["used"], 1)

    def test_enable_signals_subset_is_accounted(self) -> None:
        rows = self.epoch_rows()
        raw, nav = self.write(rows)
        result = self.run_adapter(raw, "--signal-set", "multi", "--broadcast-nav", str(nav),
                                  "--enable-signals", "GPS_L1_CA")
        self.assertEqual(result.returncode, 0, result.stderr)
        obs = json.loads((self.root / "out" / "summary.json").read_text())["observations"]
        self.assertEqual(obs["row_dispositions"]["used"], 1)
        self.assertEqual(obs["row_dispositions"]["signal_not_enabled"], 6)
        self.assertEqual(sum(obs["row_dispositions"].values()), len(rows))

    def test_option_combinations_fail_closed(self) -> None:
        raw, nav = self.write(self.epoch_rows())
        self.assertNotEqual(self.run_adapter(raw, "--signal-set", "multi").returncode, 0)  # no nav
        self.assertNotEqual(self.run_adapter(raw, "--signal-set", "multi", "--broadcast-nav", str(nav),
                                             "--enable-galileo-e1").returncode, 0)
        self.assertNotEqual(self.run_adapter(raw, "--signal-set", "multi", "--broadcast-nav", str(nav),
                                             "--enable-signals", "GPS_L1_CA,NOPE").returncode, 0)
        self.assertNotEqual(self.run_adapter(raw, "--enable-signals", "GPS_L1_CA").returncode, 0)
        self.assertNotEqual(self.run_adapter(raw, "--hatch-window-s", "30").returncode, 0)
        self.assertFalse((self.root / "out" / "summary.json").exists())

    def test_legacy_default_ignores_other_constellations_unchanged(self) -> None:
        """OFF contract: no --signal-set => the original L1/E1-only behaviour."""
        rows = [r for r in self.epoch_rows() if r[mimir.MIMIR_RAW_FIELDS.index("Svid")] != "99"]
        raw, _ = self.write(rows)  # legacy still fails closed on an out-of-range GPS PRN
        result = self.run_adapter(raw)
        self.assertEqual(result.returncode, 0, result.stderr)
        summary = json.loads((self.root / "out" / "summary.json").read_text())
        self.assertEqual(summary["schema_version"], mimir.SCHEMA_VERSION)
        d = summary["observations"]["row_dispositions"]
        self.assertEqual(set(d), set(mimir.DISPOSITIONS))
        self.assertEqual(d["used"], 2)  # GPS L1 C/A of satellites 6 and 20 (no nav check in legacy)
        self.assertNotIn("rejected_rows_detail", summary["observations"])
        rinex = (self.root / "out" / "rover.obs").read_text()
        self.assertIn("G    4 C1C L1C D1C S1C", rinex)
        self.assertNotIn("C5Q", rinex)

    def test_hatch_applies_to_galileo_e1_only(self) -> None:
        def build(hatch: bool) -> list[str]:
            rows = []
            for k, travel in enumerate((0.0750, 0.0752)):
                t, utc = self.CLEAN[0] + k * 10**9, self.CLEAN[1] + k * 1000
                for svid, con in ((6, 1), (4, 6)):
                    r = raw_line(time_nanos=t, utc_ms=utc, svid=svid, constellation=con,
                                 travel_s=travel)
                    r = self.field(r, "AccumulatedDeltaRangeState", "1")
                    r = self.field(r, "AccumulatedDeltaRangeMeters", "103.1")
                    rows.append(r)
            raw, nav = self.write(rows)
            out = self.root / ("out_h" if hatch else "out_n")
            psr = self.root / "PSR.csv"
            psr.write_text("PSR,1710409645964,1,1010.74,3\nPSR,1710409647001,2,1010.70,3\n")
            cmd = [sys.executable, str(GNSS), "smartphone-mimir-adapter", "--raw", str(raw),
                   "--psr", str(psr), "--output-dir", str(out), "--dataset-id", "t",
                   "--source-url", "u", "--source-terms", "x", "--approx-llh", "47.2,-1.5,59.5",
                   "--signal-set", "multi", "--broadcast-nav", str(nav)]
            if hatch:
                cmd += ["--hatch-window-s", "10"]
            res = subprocess.run(cmd, capture_output=True, text=True, check=False)
            self.assertEqual(res.returncode, 0, res.stderr)
            return (out / "rover.obs").read_text().splitlines()

        plain, smooth = build(False), build(True)
        g_plain = [l for l in plain if l.startswith("G06")]
        g_smooth = [l for l in smooth if l.startswith("G06")]
        e_plain = [l for l in plain if l.startswith("E04")]
        e_smooth = [l for l in smooth if l.startswith("E04")]
        self.assertEqual(g_plain, g_smooth)          # GPS never smoothed
        self.assertEqual(e_plain[0], e_smooth[0])    # arc start emits the raw code
        self.assertNotEqual(e_plain[1], e_smooth[1])  # second epoch is smoothed


if __name__ == "__main__":
    unittest.main()
