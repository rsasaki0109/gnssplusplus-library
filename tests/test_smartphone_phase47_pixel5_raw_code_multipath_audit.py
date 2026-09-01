"""Focused raw-only contract tests for the Phase47 code/multipath audit."""

from __future__ import annotations

from decimal import Decimal
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
BENCHMARKS = ROOT / "apps" / "commands" / "benchmarks"
sys.path.insert(0, str(BENCHMARKS))

import gnss_smartphone_phase47_pixel5_raw_code_multipath_audit as audit  # noqa: E402


HEADER = [
    "MessageType", "utcTimeMillis", "TimeNanos", "FullBiasNanos", "BiasNanos",
    "ReceivedSvTimeNanos", "Svid", "ConstellationType", "SignalType", "CarrierFrequencyHz",
    "HardwareClockDiscontinuityCount", "TimeOffsetNanos", "State", "MultipathIndicator",
    "AccumulatedDeltaRangeState", "AccumulatedDeltaRangeMeters", "ReceivedSvTimeUncertaintyNanos",
    "Cn0DbHz", "PseudorangeRateMetersPerSecond", "PseudorangeRateUncertaintyMetersPerSecond",
]


def _csv(rows: list[list[str]]) -> bytes:
    return (",".join(HEADER) + "\n" + "\n".join(",".join(row) for row in rows) + "\n").encode()


def _row(
    utc: int, time_ns: int, full_bias: int, sv_time: int, svid: int, signal: str,
    frequency: int, *, adr: str = "100", adr_state: str = "1", state: str = "17417",
    multipath: str = "0", cn0: str = "35", hcdc: str = "1",
) -> list[str]:
    values = {
        "MessageType": "Raw", "utcTimeMillis": str(utc), "TimeNanos": str(time_ns),
        "FullBiasNanos": str(full_bias), "BiasNanos": "0", "ReceivedSvTimeNanos": str(sv_time),
        "Svid": str(svid), "ConstellationType": "1", "SignalType": signal,
        "CarrierFrequencyHz": str(frequency), "HardwareClockDiscontinuityCount": hcdc,
        "TimeOffsetNanos": "0", "State": state, "MultipathIndicator": multipath,
        "AccumulatedDeltaRangeState": adr_state, "AccumulatedDeltaRangeMeters": adr,
        "ReceivedSvTimeUncertaintyNanos": "4", "Cn0DbHz": cn0,
        "PseudorangeRateMetersPerSecond": "-1", "PseudorangeRateUncertaintyMetersPerSecond": "0.2",
    }
    return [values[name] for name in HEADER]


class Phase47RawCodeMultipathAuditTest(unittest.TestCase):
    def test_freeze_is_pre_read_and_pairs_are_sealed(self) -> None:
        freeze = audit._verify_freeze()
        self.assertEqual(freeze["status"], "frozen-before-phase47-raw-read")
        self.assertEqual(tuple(freeze["cohort"]["route_order"]), audit.ROUTES)
        self.assertEqual(freeze["input_policy"]["truth_read_count"], 0)
        self.assertEqual(freeze["signal_pairs"]["minimum_dual_frequency_satellites_per_route"], 3)
        self.assertEqual(freeze["numeric_gates"]["flagged_vs_clean"]["flagged_p95_min_m"], 2.0)
        self.assertEqual(freeze["numeric_gates"]["flagged_vs_clean"]["flagged_to_clean_p95_ratio_min"], 1.5)
        self.assertFalse(freeze["pre_read_assertions"]["raw_payload_opened"])

    def test_raw_pseudorange_uses_integer_segment_base_and_bds_offset(self) -> None:
        row = audit.RawObservation(
            row_number=2, utc_ms=1000, time_ns=1_000_000_000_000, full_bias_ns=0,
            bias_ns=Decimal("0.25"), time_offset_ns=Decimal("1.5"),
            received_sv_time_ns=Decimal("900000000000"), hcdc=0, state=None,
            multipath=None, adr_state=None, adr_m=None, sv_time_uncertainty_ns=None,
            cn0_dbhz=None, pseudorange_rate_mps=None, pseudorange_rate_uncertainty_mps=None,
            drift_nsps=None, bias_uncertainty_ns=None, drift_uncertainty_nsps=None,
            time_uncertainty_ns=None, svid=1, system="BEIDOU", signal="BDS_B1I",
            carrier_hz=Decimal("1561098000"),
        )
        result = audit._raw_pseudorange(row, 0)
        self.assertIsNotNone(result)
        expected_delta = (Decimal("1000") - Decimal("0.25e-9") - Decimal("1.5e-9") - Decimal("900") - Decimal("14"))
        self.assertAlmostEqual(result, float(expected_delta * Decimal(str(audit.SPEED_OF_LIGHT_MPS))), places=5)

    def test_parse_pair_cancels_common_code_and_reports_iono_combination(self) -> None:
        # The transmit times are selected to make finite positive ranges from
        # a 1-second receiver TOW.  Each signal receives the same common range.
        time_ns = 1_000_000_000
        base = 0
        sv_time = 900_000_000
        rows = [
            _row(1000, time_ns, base, sv_time, 1, "GPS_L1_CA", 1575420000),
            _row(1000, time_ns, base, sv_time, 1, "GPS_L5_Q", 1176450000),
            _row(1000, time_ns, base, sv_time, 2, "GPS_L1_CA", 1575420000),
            _row(1000, time_ns, base, sv_time, 2, "GPS_L5_Q", 1176450000),
        ]
        epochs, _ = audit._parse_raw_payload(_csv(rows))
        audit._assign_segments(epochs)
        pairs, _, metadata = audit._pair_rows(epochs)
        self.assertEqual(len(pairs), 2)
        self.assertEqual(metadata["pair_counts"]["GPS_L1CA-GPS_L5"]["epochs"], 2)
        self.assertAlmostEqual(pairs[0]["code_difference_m"], 0.0, places=9)
        self.assertTrue(math_is_finite(pairs[0]["frequency_squared_ionosphere_m"]))

    def test_cmc_resets_on_cycle_slip_and_time_gap(self) -> None:
        rows = [
            _row(1000, 1_000_000_000, 0, 900_000_000, 1, "GPS_L1_CA", 1575420000),
            _row(2000, 2_000_000_000, 0, 1_900_000_000, 1, "GPS_L1_CA", 1575420000),
            _row(4000, 4_000_000_000, 0, 3_900_000_000, 1, "GPS_L1_CA", 1575420000, adr_state="3"),
        ]
        epochs, _ = audit._parse_raw_payload(_csv(rows))
        audit._assign_segments(epochs)
        cmc, events = audit._cmc_rows(epochs)
        self.assertEqual(len(cmc), 2)
        self.assertTrue(any(event["kind"] == "cmc_cycle_slip_arc_reset" for event in events))
        self.assertTrue(any(event["kind"] == "cmc_gap_arc_reset" for event in events))

    def test_existing_mask_buckets_are_raw_only(self) -> None:
        rows = [
            _row(1000, 1_000_000_000, 0, 900_000_000, 1, "GPS_L1_CA", 1575420000, multipath="1", cn0="10"),
            _row(1000, 1_000_000_000, 0, 900_000_000, 1, "GPS_L5_Q", 1176450000, multipath="1", cn0="10"),
        ]
        epochs, _ = audit._parse_raw_payload(_csv(rows))
        audit._assign_segments(epochs)
        pairs, _, _ = audit._pair_rows(epochs)
        self.assertEqual(len(pairs), 1)
        self.assertFalse(pairs[0]["clean"])
        self.assertIn("multipath", pairs[0]["mask_reasons"])
        self.assertIn("low_cnr", pairs[0]["mask_reasons"])

    def test_static_contract_and_source_are_solver_free(self) -> None:
        freeze = audit._verify_freeze()
        contract = audit._static_signal_contract(freeze)
        self.assertTrue(contract["all_declared_pair_signals_adopted_by_current_fgo"])
        source = Path(audit.__file__).read_text(encoding="utf-8")
        self.assertNotIn("subprocess", source)
        self.assertNotIn("zipfile", source)
        self.assertNotIn("run_native", source)

    def test_one_read_hash_accounting_uses_same_buffer(self) -> None:
        with tempfile.TemporaryDirectory(prefix="phase47-") as directory:
            path = Path(directory) / "device_gnss.csv"
            path.write_bytes(b"raw")
            payload, digest = audit._read_bytes_once(path, "synthetic raw")
        self.assertEqual(payload, b"raw")
        self.assertEqual(digest, audit._sha256_bytes(payload))


def math_is_finite(value: float) -> bool:
    return value == value and abs(value) != float("inf")


if __name__ == "__main__":
    unittest.main()
