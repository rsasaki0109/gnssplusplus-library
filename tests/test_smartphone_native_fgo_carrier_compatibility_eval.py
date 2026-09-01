from __future__ import annotations

import csv
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands"))
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_native_fgo_carrier_compatibility_eval as CARRIER  # noqa: E402


RAW_FIELDS = [
    "utcTimeMillis",
    "Svid",
    "AccumulatedDeltaRangeState",
    "AccumulatedDeltaRangeMeters",
    "PseudorangeRateMetersPerSecond",
    "HardwareClockDiscontinuityCount",
    "SignalType",
]


def _rinex_satellite(value: str = "123.000") -> str:
    fields = [value, "4.000", "-1.000", "35.000"]
    return "G01" + "".join(f"{field:>16}" for field in fields) + "\n"


def _rinex(epochs: list[tuple[int, str]]) -> str:
    lines = [
        "     3.04           OBSERVATION DATA    M                   RINEX VERSION / TYPE\n",
        "                                                            END OF HEADER\n",
    ]
    for second, satellite in epochs:
        lines.append(f"> 2024 01 01 00 00 {second:02d}.0000000  0  1\n")
        lines.append(satellite)
    return "".join(lines)


def _write_raw(path: Path, rows: list[dict[str, object]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=RAW_FIELDS)
        writer.writeheader()
        writer.writerows(rows)


class CarrierCompatibilityContractTests(unittest.TestCase):
    def test_boundary_marks_invalid_and_next_valid_without_touching_other_fields(self) -> None:
        with tempfile.TemporaryDirectory(prefix="carrier-compat-") as directory:
            root = Path(directory)
            raw = root / "device_gnss.csv"
            source = root / "source.obs"
            target = root / "sanitized.obs"
            _write_raw(raw, [
                {"utcTimeMillis": 1000, "Svid": 1, "AccumulatedDeltaRangeState": 1, "AccumulatedDeltaRangeMeters": 10, "PseudorangeRateMetersPerSecond": 1, "HardwareClockDiscontinuityCount": 0, "SignalType": "GPS_L1_CA"},
                {"utcTimeMillis": 2000, "Svid": 1, "AccumulatedDeltaRangeState": 16, "AccumulatedDeltaRangeMeters": 0, "PseudorangeRateMetersPerSecond": 1, "HardwareClockDiscontinuityCount": 0, "SignalType": "GPS_L1_CA"},
                {"utcTimeMillis": 3000, "Svid": 1, "AccumulatedDeltaRangeState": 1, "AccumulatedDeltaRangeMeters": 12, "PseudorangeRateMetersPerSecond": 1, "HardwareClockDiscontinuityCount": 0, "SignalType": "GPS_L1_CA"},
            ])
            first = _rinex_satellite("123.000")
            second = _rinex_satellite("456.000")
            source.write_text(_rinex([(0, first), (1, second), (2, second)]), encoding="ascii")
            result = CARRIER.sanitize_rinex_carrier_boundaries(raw, source, target)
            self.assertEqual(result["invalid_rows"], 1)
            self.assertGreaterEqual(result["sanitized_rows"], 2)
            output = target.read_text(encoding="ascii").splitlines()
            satellite_lines = [line for line in output if line.startswith("G01")]
            self.assertEqual(len(satellite_lines), 3)
            self.assertEqual(satellite_lines[0][3:19], first[3:19])
            self.assertEqual(satellite_lines[0][19:35], first[19:35])
            expected_blank_with_lli = (" " * 14) + "1 "
            self.assertEqual(satellite_lines[1][19:35], expected_blank_with_lli)
            self.assertEqual(satellite_lines[2][19:35], expected_blank_with_lli)
            self.assertEqual(satellite_lines[1][33], "1")
            self.assertEqual(satellite_lines[2][33], "1")

    def test_reset_and_gap_are_explicit_boundaries(self) -> None:
        with tempfile.TemporaryDirectory(prefix="carrier-compat-") as directory:
            root = Path(directory)
            raw = root / "device_gnss.csv"
            source = root / "source.obs"
            target = root / "sanitized.obs"
            _write_raw(raw, [
                {"utcTimeMillis": 1000, "Svid": 1, "AccumulatedDeltaRangeState": 1, "AccumulatedDeltaRangeMeters": 10, "PseudorangeRateMetersPerSecond": 1, "HardwareClockDiscontinuityCount": 0, "SignalType": "GPS_L1_CA"},
                {"utcTimeMillis": 2000, "Svid": 1, "AccumulatedDeltaRangeState": 3, "AccumulatedDeltaRangeMeters": 11, "PseudorangeRateMetersPerSecond": 1, "HardwareClockDiscontinuityCount": 0, "SignalType": "GPS_L1_CA"},
                {"utcTimeMillis": 4000, "Svid": 1, "AccumulatedDeltaRangeState": 1, "AccumulatedDeltaRangeMeters": 13, "PseudorangeRateMetersPerSecond": 1, "HardwareClockDiscontinuityCount": 0, "SignalType": "GPS_L1_CA"},
            ])
            source.write_text(_rinex([(0, _rinex_satellite()), (1, _rinex_satellite()), (4, _rinex_satellite())]), encoding="ascii")
            result = CARRIER.sanitize_rinex_carrier_boundaries(raw, source, target)
            self.assertGreaterEqual(result["boundary_reason_counts"].get("adr-reset", 0), 1)
            self.assertGreaterEqual(result["boundary_reason_counts"].get("gap-over-1500ms", 0), 1)

    def test_epoch_and_key_mismatch_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory(prefix="carrier-compat-") as directory:
            root = Path(directory)
            raw = root / "device_gnss.csv"
            source = root / "source.obs"
            _write_raw(raw, [{"utcTimeMillis": 1000, "Svid": 1, "AccumulatedDeltaRangeState": 1, "AccumulatedDeltaRangeMeters": 10, "PseudorangeRateMetersPerSecond": 1, "HardwareClockDiscontinuityCount": 0, "SignalType": "GPS_L1_CA"}])
            source.write_text(_rinex([(0, _rinex_satellite()), (1, _rinex_satellite())]), encoding="ascii")
            with self.assertRaises(CARRIER.CarrierCompatibilityError):
                CARRIER.sanitize_rinex_carrier_boundaries(raw, source, root / "out.obs")

    def test_physical_bound_is_derived_from_sigma_huber_and_elevation(self) -> None:
        self.assertAlmostEqual(CARRIER.POSTFIT_RMS_BOUND_M, 0.2303508, places=5)
        self.assertAlmostEqual(CARRIER.PREFIT_RATE_RMS_BOUND_MPS, 2 * CARRIER.POSTFIT_RMS_BOUND_M, places=8)

    def test_command_contract_freezes_baseline_and_float_candidate(self) -> None:
        baseline = CARRIER.baseline_command(Path("obs"), Path("nav"), Path("seed"), Path("out"))
        candidate = CARRIER.carrier_float_command(Path("obs"), Path("nav"), Path("seed"), Path("out"))
        CARRIER.validate_command(baseline, candidate=False)
        CARRIER.validate_command(candidate, candidate=True)
        self.assertFalse(CARRIER.command_contract(baseline)["carrier_phase_enabled"])
        self.assertTrue(CARRIER.command_contract(candidate)["carrier_phase_enabled"])
        self.assertNotIn("--base", candidate)
        self.assertNotIn("--fix-ambiguities", candidate)
        self.assertNotIn("--sd-doppler-factors", candidate)
        self.assertEqual(CARRIER.command_contract(candidate)["max_iterations"], 50)

    def test_candidate_gate_rejects_nonconvergence_rms_and_continuity(self) -> None:
        summary = {"converged": False, "iterations": 50, "max_iterations": 50, "carrier_phase_residual_rms_m": 1.0, "ambiguity_condition_proxy": 2.0}
        trace = {"monotonic_non_increasing": False, "converged_rows": 0}
        carrier = {"valid_rows": 10, "max_arc_duration_s": 20, "prefit_rate": {"rms_mps": 1.0}}
        position = {"above_speed_bound": 1}
        passed, reasons, _ = CARRIER._candidate_gate(summary, trace, carrier, position)
        self.assertFalse(passed)
        self.assertIn("summary_not_converged", reasons)
        self.assertIn("cost_trace_not_monotonic_or_converged", reasons)
        self.assertIn("prefit_rate_rms_exceeds_physical_bound", reasons)
        self.assertIn("postfit_carrier_rms_exceeds_physical_bound", reasons)
        self.assertIn("physical_continuity_violation", reasons)

    def test_baseline_fallback_copy_is_byte_exact(self) -> None:
        with tempfile.TemporaryDirectory(prefix="carrier-compat-") as directory:
            root = Path(directory)
            source = root / "baseline.pos"
            target = root / "selected.pos"
            payload = b"binary-safe\n0.123\n"
            source.write_bytes(payload)
            CARRIER._copy_atomic(source, target)
            self.assertEqual(target.read_bytes(), payload)
            self.assertEqual(CARRIER.sha256(source), CARRIER.sha256(target))


if __name__ == "__main__":
    unittest.main()
