"""Launch-free Phase133 selector-boundary tests.

All route records are synthetic.  The tests do not open raw GNSS/IMU/nav or
base files, solution rows, truth, MAT/PDC/precomputed-coordinate artifacts,
and do not launch a native solver or a fake child process.
"""

from __future__ import annotations

from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps/commands/benchmarks"))
import gnss_smartphone_phase133_runner_native_selector_boundary as contract  # noqa: E402
import gnss_smartphone_phase132_typed_canonical_structural as phase132  # noqa: E402


class Phase133RunnerNativeSelectorBoundaryTests(unittest.TestCase):
    def test_audit_freeze_and_native_source_are_pinned(self) -> None:
        freeze = contract.verify_freeze()
        self.assertEqual(freeze["decision"]["candidate_count"], 1)
        evidence = contract.native_source_cross_check()
        self.assertFalse(evidence["phase130_present"])
        self.assertFalse(evidence["real_binary_launched"])

    def test_native_help_cross_check_accepts_synthetic_help(self) -> None:
        help_text = "Usage: gnss_fgo_imu_no_base\n" + "\n".join(contract.NATIVE_SELECTORS)
        result = contract.native_help_cross_check(help_text)
        self.assertEqual(result["required_native_selectors"], list(contract.NATIVE_SELECTORS))
        self.assertFalse(result["phase130_present"])
        with self.assertRaises(contract.Phase133ContractError):
            contract.native_help_cross_check(help_text + "\n" + contract.PHASE130_SELECTOR)

    def test_selector_ownership_is_exact(self) -> None:
        ownership = contract.selector_ownership()
        self.assertEqual(ownership["runner_only"], [contract.PHASE130_SELECTOR])
        self.assertEqual(ownership["native_exactly_once"], list(contract.NATIVE_SELECTORS))
        self.assertEqual(ownership["off"], list(contract.OFF_SELECTORS))
        self.assertNotIn(contract.PHASE130_SELECTOR, ownership["native_exactly_once"])

    def test_fixed_command_snapshot_has_native_flags_once_and_phase130_zero(self) -> None:
        for route in contract.ROUTES:
            command = contract.command_template(route)
            contract.validate_command(route, command)
            for selector in contract.NATIVE_SELECTORS:
                self.assertEqual(command.count(selector), 1)
            for selector in contract.RUNNER_ONLY_SELECTORS + contract.OFF_SELECTORS:
                self.assertEqual(command.count(selector), 0)

    def test_phase130_is_the_only_removed_control_token(self) -> None:
        old = phase132.command_template(contract.ROUTES[0])
        new = contract.command_template(contract.ROUTES[0])
        old_controls = [token for token in old if token.startswith("--")]
        new_controls = [token for token in new if token.startswith("--")]
        self.assertEqual(
            [token for token in old_controls if token != contract.PHASE130_SELECTOR],
            new_controls,
        )
        self.assertEqual(old_controls.count(contract.PHASE130_SELECTOR), 1)
        self.assertEqual(new_controls.count(contract.PHASE130_SELECTOR), 0)

    def test_synthetic_fake_binary_rejects_historical_unknown_option(self) -> None:
        fixed = contract.command_template(contract.ROUTES[0])
        historical = list(fixed)
        insert_at = historical.index(contract.PHASE131_SELECTOR)
        historical.insert(insert_at, contract.PHASE130_SELECTOR)
        supported = contract.fake_native_supported_options()
        self.assertEqual(
            contract.fake_binary_unknown_options(historical, supported),
            [contract.PHASE130_SELECTOR],
        )
        self.assertEqual(contract.fake_binary_unknown_options(fixed, supported), [])

    def test_evidence_requires_preflight_and_distinguishes_failure(self) -> None:
        failure = {
            "phase133": {
                "typed_preflight_call_count": 1,
                "old_literal_phase130_preflight_call_count": 0,
                "native_command_constructed": False,
                "native_selector_forwarded": False,
                "native_binary_invocation_attempted": False,
                "native_resolver_executed": False,
                "native_command_construction_count": 0,
                "native_solver_invocations": 0,
            }
        }
        result = contract.verify_execution_evidence(
            contract.ROUTES[0], failure, preflight_passed=False
        )
        self.assertTrue(result["fail_closed"])

        passing = {
            "phase133": {
                "typed_preflight_call_count": 1,
                "old_literal_phase130_preflight_call_count": 0,
                "native_command_constructed": True,
                "native_selector_forwarded": True,
                "native_binary_invocation_attempted": True,
                "native_resolver_executed": True,
                "native_resolver_call_count": 3,
                "native_command_construction_count": 1,
                "native_selector_forwarding_count": 1,
                "native_solver_invocations": 1,
            }
        }
        result = contract.verify_execution_evidence(
            contract.ROUTES[1], passing, preflight_passed=True
        )
        self.assertFalse(result["fail_closed"])

    def test_evidence_cannot_promote_old_phase130_counters(self) -> None:
        evidence = {
            "phase133": {
                "typed_preflight_call_count": 1,
                "old_literal_phase130_preflight_call_count": 2,
                "native_command_constructed": False,
                "native_selector_forwarded": False,
                "native_binary_invocation_attempted": False,
                "native_resolver_executed": False,
                "native_command_construction_count": 0,
                "native_solver_invocations": 0,
            }
        }
        with self.assertRaises(contract.Phase133ContractError):
            contract.verify_execution_evidence(
                contract.ROUTES[0], evidence, preflight_passed=False
            )

    def test_zero_read_accounting_is_explicit(self) -> None:
        accounting = contract.zero_read_accounting()
        self.assertEqual(accounting["raw_phone_gnss_reads"], 0)
        self.assertEqual(accounting["native_solver_invocations"], 0)
        self.assertEqual(accounting["truth_reads"], 0)
        self.assertEqual(accounting["mat_pdc_precomputed_coordinate_reads"], 0)
        self.assertEqual(accounting["kaggle_or_token_access"], 0)

    def test_launch_free_cli_only_verifies_freeze(self) -> None:
        self.assertEqual(contract.main(["--verify-freeze"]), 0)


if __name__ == "__main__":
    unittest.main()
