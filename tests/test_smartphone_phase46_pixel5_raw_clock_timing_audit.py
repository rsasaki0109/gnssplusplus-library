"""Focused contract tests for the Phase46 raw Android clock audit."""

from __future__ import annotations

from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
BENCHMARKS = ROOT / "apps" / "commands" / "benchmarks"
sys.path.insert(0, str(BENCHMARKS))

import gnss_smartphone_phase46_pixel5_raw_clock_timing_audit as audit  # noqa: E402


HEADER = [
    "MessageType",
    "utcTimeMillis",
    "TimeNanos",
    "FullBiasNanos",
    "BiasNanos",
    "HardwareClockDiscontinuityCount",
    "DriftNanosPerSecond",
    "BiasUncertaintyNanos",
    "TimeUncertaintyNanos",
    "TimeOffsetNanos",
    "ReceivedSvTimeNanos",
    "ConstellationType",
    "Svid",
    "SignalType",
]


def _csv(rows: list[list[str]]) -> bytes:
    return (",").join(HEADER).encode() + b"\n" + b"\n".join(
        (",").join(row).encode() for row in rows
    ) + b"\n"


class SmartphonePhase46Pixel5RawClockTimingAuditTest(unittest.TestCase):
    def test_v1_freeze_is_pre_raw_and_has_closed_gates(self) -> None:
        freeze = audit._verify_freeze()
        self.assertEqual(freeze["status"], "frozen-before-phase46-raw-read")
        self.assertEqual(tuple(freeze["exact_raw_inputs"]), audit.ROUTES)
        self.assertEqual(freeze["read_contract"]["raw_device_gnss_read_limit_per_route"], 1)
        self.assertEqual(freeze["read_contract"]["truth_read_count_required"], 0)
        self.assertEqual(freeze["numeric_gates"]["detrended_utc_gps_residual_ms"]["max_abs_required"], 2.0)
        self.assertEqual(freeze["numeric_gates"]["linear_drift_ppm"]["max_abs_required"], 1000.0)
        self.assertEqual(freeze["numeric_gates"]["uncaptured_tow_noncommon_mode_m"]["max_required"], 10.0)

    def test_raw_clock_equation_uses_fixed_gps_utc_offset(self) -> None:
        utc_ms = 1_600_000_000_000
        hardware_gps_ns = (utc_ms - audit.GPS_EPOCH_UNIX_MS + 18_000) * 1_000_000
        time_ns = 1_500_000_000_000_000_000
        full_bias_ns = time_ns - hardware_gps_ns
        row = [
            "Raw", str(utc_ms), str(time_ns), str(full_bias_ns), "0", "4",
            "0", "1", "2", "0", "123", "1", "7", "L1",
        ]
        epochs, metadata = audit._parse_raw_payload(_csv([row]))
        self.assertEqual(metadata["raw_rows"], 1)
        self.assertEqual(len(epochs), 1)
        self.assertAlmostEqual(epochs[0].first.utc_gps_residual_ms, 0.0)

    def test_parse_groups_satellites_by_first_seen_utc_key(self) -> None:
        rows = [
            ["Raw", "1000", "100000000000", "0", "0", "1", "", "", "", "10", "", "1", "1", "L1"],
            ["Raw", "1000", "100000000000", "0", "0", "1", "", "", "", "11", "", "1", "2", "L1"],
            ["Raw", "2000", "101000000000", "0", "0", "1", "", "", "", "10", "", "1", "1", "L1"],
        ]
        epochs, metadata = audit._parse_raw_payload(_csv(rows))
        self.assertEqual([epoch.key_ms for epoch in epochs], [1000, 2000])
        self.assertEqual([len(epoch.rows) for epoch in epochs], [2, 1])
        self.assertEqual(metadata["repeated_epoch_key_count"], 0)

    def test_segment_boundary_captures_hcdc_and_time_gap(self) -> None:
        rows = [
            ["Raw", "1000", "100000000000", "-1000000000000000000", "0", "1", "", "", "", "0", "", "1", "1", "L1"],
            ["Raw", "2000", "101000000000", "-1000000000000000000", "0", "1", "", "", "", "0", "", "1", "1", "L1"],
            ["Raw", "5000", "103000000000", "-1000000000000000000", "0", "2", "", "", "", "0", "", "1", "1", "L1"],
        ]
        epochs, _ = audit._parse_raw_payload(_csv(rows))
        assignments, events = audit._segment_and_events(epochs)
        self.assertEqual([item["segment"] for item in assignments], [0, 0, 1])
        self.assertTrue(any(event["kind"] == "hardware_clock_discontinuity" for event in events))
        self.assertTrue(any(event["kind"] == "time_nanos_gap_gt_1s" for event in events))

    def test_uncaptured_full_bias_change_is_reported(self) -> None:
        rows = [
            ["Raw", "1000", "100000000000", "-1000000000000000000", "0", "1", "", "", "", "0", "", "1", "1", "L1"],
            ["Raw", "1000", "100000000000", "-1000000000000000001", "0", "1", "", "", "", "0", "", "1", "2", "L1"],
        ]
        epochs, _ = audit._parse_raw_payload(_csv(rows))
        _, events = audit._segment_and_events(epochs)
        event = next(event for event in events if event["kind"] == "full_bias_change")
        self.assertFalse(event["captured_by_explicit_boundary"])

    def test_float64_risk_is_measured_while_integer_difference_is_exact(self) -> None:
        row = audit.RawRow(
            row_number=2,
            utc_ms=1000,
            time_ns=10**18 + 123,
            full_bias_ns=-10**18 + 456,
            bias_ns=audit.Decimal("0.25"),
            hcdc=1,
            drift_nsps=None,
            bias_uncertainty_ns=None,
            drift_uncertainty_nsps=None,
            time_uncertainty_ns=None,
            time_offset_ns=None,
            received_sv_time_ns=None,
            state="",
            constellation="1",
            svid="1",
            signal="L1",
            carrier_hz=None,
        )
        self.assertEqual(row.integer_hardware_ns, 2 * 10**18 - 333)
        float_value = float(row.time_ns) - float(row.full_bias_ns) - float(row.bias_ns)
        self.assertGreater(abs(float_value - float(row.exact_hardware_gps_ns)), 0.0)

    def test_affine_fit_reports_ppm_scale_without_nan(self) -> None:
        slope, intercept, residuals = audit._fit_affine([0.0, 1000.0, 2000.0], [1.0, 1.001, 1.002])
        self.assertAlmostEqual(slope, 1e-6)
        self.assertAlmostEqual(intercept, 1.0)
        self.assertEqual(len(residuals), 3)
        self.assertTrue(all(abs(value) < 1e-12 for value in residuals))

    def test_read_bytes_once_hashes_one_payload(self) -> None:
        with tempfile.TemporaryDirectory(prefix="phase46-") as directory:
            path = Path(directory) / "device_gnss.csv"
            path.write_bytes(b"raw")
            payload, digest = audit._read_bytes_once(path, "synthetic raw")
        self.assertEqual(payload, b"raw")
        self.assertEqual(digest, audit._sha256_bytes(b"raw"))

    def test_source_has_no_solver_or_archive_execution(self) -> None:
        source = Path(audit.__file__).read_text(encoding="utf-8")
        self.assertNotIn("subprocess", source)
        self.assertNotIn("zipfile", source)
        self.assertNotIn("Popen", source)
        self.assertNotIn("run_native", source)


if __name__ == "__main__":
    unittest.main()
