"""Focused truth-free contract tests for the Phase60 ISB audit."""

from __future__ import annotations

from decimal import Decimal
from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
BENCHMARKS = ROOT / "apps" / "commands" / "benchmarks"
if str(BENCHMARKS) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS))

import gnss_smartphone_phase60_pixel5_intersignal_bias_audit as audit  # noqa: E402


class SmartphonePhase60InterSignalBiasAuditTest(unittest.TestCase):
    def test_freeze_is_sealed_before_raw_read(self) -> None:
        freeze = audit._verify_freeze()
        self.assertEqual(freeze["status"], "frozen-before-phase60-raw-read")
        self.assertEqual(tuple(freeze["cohort"]["route_order"]), audit.ROUTES)
        self.assertEqual(freeze["input_policy"]["raw_device_gnss_read_count_total"], 4)
        self.assertEqual(freeze["input_policy"]["truth_read_count"], 0)
        self.assertFalse(freeze["pre_read_assertions"]["phase60_raw_device_gnss_opened"])
        self.assertEqual(
            freeze["source_semantics"]["full_inter_signal_bias"]["sign_equation"],
            "corrected pseudorange = raw pseudorange - FullInterSignalBiasNanos",
        )
        self.assertIn("Full plus Satellite", freeze["source_semantics"]["composition_rule"])

    def test_android_sign_and_nanoseconds_to_metres(self) -> None:
        one_ns_m = 299792458.0 / 1_000_000_000.0
        self.assertAlmostEqual(audit._bias_m(Decimal("1")), one_ns_m, places=15)
        self.assertAlmostEqual(
            audit._corrected_pseudorange_m(100.0, Decimal("1")),
            100.0 - one_ns_m,
            places=12,
        )
        self.assertIsNone(audit._bias_m(Decimal("NaN")))
        self.assertIsNone(audit._corrected_pseudorange_m(100.0, None))

    def test_full_includes_satellite_and_plus_is_not_a_candidate(self) -> None:
        raw = 20.0
        full = Decimal("2")
        satellite = Decimal("0.5")
        full_only = audit._corrected_pseudorange_m(raw, full)
        satellite_only = audit._corrected_pseudorange_m(raw, satellite)
        both_wrong = raw - (audit._bias_m(full) or 0.0) - (audit._bias_m(satellite) or 0.0)
        self.assertAlmostEqual(full_only or 0.0, raw - 2.0 * 299792458.0e-9, places=12)
        self.assertAlmostEqual(satellite_only or 0.0, raw - 0.5 * 299792458.0e-9, places=12)
        self.assertNotAlmostEqual(both_wrong, full_only or 0.0, places=12)
        self.assertNotEqual(full_only, satellite_only)

    def test_phase25_proxy_uses_receiver_minus_transmit_clock(self) -> None:
        # 20 ns of receive-minus-transmit time is exactly c*20 ns.
        pseudorange = audit._raw_pseudorange_m(
            time_ns=20_000_000_000,
            base_full_bias_ns=0,
            bias_ns=Decimal("0"),
            time_offset_ns=Decimal("0"),
            received_sv_time_ns=19_999_999_980,
            constellation=6,
        )
        self.assertAlmostEqual(pseudorange or 0.0, 299792458.0 * 20e-9, places=9)

    def test_signal_groups_are_fixed_and_frequency_checked(self) -> None:
        self.assertEqual(
            audit._signal_group(6, "GAL_E1", Decimal("1575420000")),
            "GAL_E1",
        )
        self.assertEqual(
            audit._signal_group(1, "GPS_L5", Decimal("1176450000")),
            "GPS_L5",
        )
        self.assertIsNone(
            audit._signal_group(6, "GAL_E1", Decimal("1176450000")),
        )

    def test_source_plumbing_is_static_and_raw_fields_are_not_currently_consumed(self) -> None:
        freeze = audit._verify_freeze()
        source = audit._source_audit(freeze)
        self.assertFalse(source["adapter"]["parses_full_intersignal_bias"])
        self.assertFalse(source["adapter"]["parses_satellite_intersignal_bias"])
        self.assertFalse(source["observation"]["retains_full_intersignal_bias"])
        self.assertFalse(source["observation"]["retains_satellite_intersignal_bias"])
        self.assertFalse(source["fgo"]["consumes_raw_full_intersignal_bias"])
        self.assertFalse(source["fgo"]["consumes_raw_satellite_intersignal_bias"])
        self.assertTrue(source["fgo"]["existing_signal_bias_state_path_present"])

    def test_source_has_no_solver_or_forbidden_payload_reader(self) -> None:
        source = Path(audit.__file__).read_text(encoding="utf-8")
        self.assertNotIn("subprocess", source)
        self.assertNotIn("truth_maps", source)
        self.assertNotIn("nav.calculate", source)


if __name__ == "__main__":
    unittest.main()
