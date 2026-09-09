from __future__ import annotations

import csv
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands"))
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_native_fgo_float_carrier_eval as CARRIER  # noqa: E402


class FloatCarrierContractTests(unittest.TestCase):
    def setUp(self) -> None:
        self.obs = Path("/tmp/rover.obs")
        self.nav = Path("/tmp/brdc.nav")
        self.seed = Path("/tmp/seed.pos")
        self.out = Path("/tmp/fgo-float-carrier")

    def test_candidate_command_is_no_base_float_and_adr_valid(self) -> None:
        command = CARRIER.float_carrier_fgo_command(
            self.obs, self.nav, self.seed, self.out, max_epochs=30
        )
        CARRIER.validate_command(command, candidate=True)
        contract = CARRIER.command_contract(command)
        self.assertTrue(contract["carrier_phase_enabled"])
        self.assertTrue(contract["reject_rover_carrier_lli"])
        self.assertEqual(contract["violations"], [])
        self.assertNotIn("--base", command)
        self.assertNotIn("--fix-ambiguities", command)
        self.assertNotIn("--sd-doppler-factors", command)
        self.assertNotIn("--sd-tdcp-factors", command)

    def test_baseline_command_does_not_enable_carrier(self) -> None:
        command = CARRIER.baseline_fgo_command(self.obs, self.nav, self.seed, self.out)
        CARRIER.validate_command(command, candidate=False)
        self.assertFalse(CARRIER.command_contract(command)["carrier_phase_enabled"])

    def test_forbidden_command_flags_fail_closed(self) -> None:
        command = CARRIER.float_carrier_fgo_command(self.obs, self.nav, self.seed, self.out)
        command.extend(("--fix-ambiguities", "--sd-doppler-factors"))
        with self.assertRaises(CARRIER.FloatCarrierError):
            CARRIER.validate_command(command, candidate=True)

    def test_arc_boundaries_are_deterministic(self) -> None:
        first = {"constellation": "G", "prn": 7, "signal": "GPS_L1_CA", "timestamp_s": 1.0}
        current = {**first, "timestamp_s": 2.0}
        self.assertEqual(CARRIER.arc_break_reason(None, first), "initial")
        self.assertEqual(CARRIER.arc_break_reason(first, current), "continue")
        self.assertEqual(
            CARRIER.arc_break_reason(first, {**current, "timestamp_s": 4.1}), "gap"
        )
        self.assertEqual(
            CARRIER.arc_break_reason(first, {**current, "loss_of_lock": True}),
            "loss-of-lock-or-invalid-adr",
        )
        self.assertEqual(
            CARRIER.arc_break_reason(first, {**current, "clock_discontinuity": True}),
            "hardware-clock-discontinuity",
        )
        self.assertEqual(
            CARRIER.arc_break_reason(first, {**current, "signal": "GAL_E1_C_P"}),
            "identity-change",
        )
        self.assertEqual(
            CARRIER.arc_break_reason(first, {**current, "timestamp_s": 0.0}),
            "nonpositive-time",
        )

    def _good_summary(self) -> dict[str, object]:
        return {
            "backend": "eigen",
            "preset": "default",
            "input_epochs": 30,
            "optimized_epochs": 30,
            "valid_solutions": 30,
            "pseudorange_factors": 300,
            "tdcp_factors": 270,
            "tdcp_factors_inserted": 270,
            "motion_factors": 29,
            "carrier_phase_factors": 190,
            "ambiguity_states": 40,
            "double_difference_pseudorange_factors": 0,
            "double_difference_carrier_factors": 0,
            "single_difference_doppler_factors": 0,
            "single_difference_tdcp_factors": 0,
            "ambiguity_between_factors": 0,
            "fixed_ambiguities": 0,
            "fixed_solution": False,
            "iterations": 8,
            "max_iterations": 8,
            "initial_cost": 100.0,
            "final_cost": 20.0,
            "converged": True,
            "graph_factors": 800,
            "graph_values": 200,
            "residual_rms_m": 2.0,
            "carrier_phase_residual_rms_m": 0.02,
        }

    def test_float_summary_requires_carrier_and_forbids_fixing(self) -> None:
        result = CARRIER.validate_float_carrier_summary(self._good_summary())
        self.assertEqual(result["carrier_phase_factors"], 190)
        self.assertEqual(result["ambiguity_states"], 40)
        bad = self._good_summary()
        bad["fixed_solution"] = True
        with self.assertRaises(CARRIER.FloatCarrierError):
            CARRIER.validate_float_carrier_summary(bad)
        bad = self._good_summary()
        bad["double_difference_carrier_factors"] = 1
        with self.assertRaises(CARRIER.FloatCarrierError):
            CARRIER.validate_float_carrier_summary(bad)
        bad = self._good_summary()
        bad["carrier_phase_factors"] = 0
        with self.assertRaises(CARRIER.FloatCarrierError):
            CARRIER.validate_float_carrier_summary(bad)

    def test_clock_counter_transition_is_fail_closed(self) -> None:
        fields = ["HardwareClockDiscontinuityCount", "Svid"]
        with tempfile.TemporaryDirectory(prefix="float-carrier-clock-") as raw:
            path = Path(raw) / "observations.csv"
            with path.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=fields)
                writer.writeheader()
                writer.writerow({fields[0]: "0", fields[1]: "1"})
                writer.writerow({fields[0]: "0", fields[1]: "1"})
                writer.writerow({fields[0]: "1", fields[1]: "1"})
            report = CARRIER.inspect_clock_discontinuities(path)
            self.assertEqual(report["transition_rows"], 1)
            self.assertFalse(report["candidate_safe"])

    def test_clock_counter_decrease_is_invalid(self) -> None:
        fields = ["HardwareClockDiscontinuityCount"]
        with tempfile.TemporaryDirectory(prefix="float-carrier-clock-") as raw:
            path = Path(raw) / "observations.csv"
            with path.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=fields)
                writer.writeheader()
                writer.writerow({fields[0]: "2"})
                writer.writerow({fields[0]: "1"})
            with self.assertRaises(CARRIER.FloatCarrierError):
                CARRIER.inspect_clock_discontinuities(path)


if __name__ == "__main__":
    unittest.main()
