"""Launch-free tests for the Phase132 Phase131 runner preflight boundary.

These tests use synthetic dictionaries only.  They do not open GNSS, IMU,
navigation, RINEX, solution, truth, MAT, PDC, or precomputed-coordinate
payloads and never launch the native process.
"""

from __future__ import annotations

from pathlib import Path
import sys
import unittest
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps/commands/benchmarks"))
import gnss_smartphone_phase131_canonical_correction_structural_authorized_execute as runner  # noqa: E402


class Phase132RunnerPreflightTests(unittest.TestCase):
    def test_alias_and_rinex_band_share_typed_key_without_literal_join(self) -> None:
        android = runner.canonicalize_runner_signal(
            "GLO_G1_CA", ("R", 7), -4)
        shorthand = runner.canonicalize_runner_signal(
            "L1", ("R", 7), -4)
        rinex = runner.canonicalize_runner_rinex(
            "C1C", ("R", 7), -4)
        rinex_p = runner.canonicalize_runner_rinex(
            "C1P", ("R", 7), -4)
        self.assertTrue(android["accepted"])
        self.assertTrue(shorthand["accepted"])
        self.assertTrue(rinex["accepted"])
        self.assertTrue(rinex_p["accepted"])
        self.assertEqual(android["key_text"], rinex["key_text"])
        self.assertEqual(rinex["key_text"], rinex_p["key_text"])
        self.assertEqual(shorthand["key_text"], android["key_text"])
        self.assertEqual(android["original_signal"], "GLO_G1_CA")
        self.assertEqual(rinex["literal_tracking_code"], "C1C")

    def test_fcn_is_required_bounded_and_never_inferred(self) -> None:
        self.assertFalse(runner.canonicalize_runner_signal(
            "GLO_G1_CA", ("R", 7), None)["accepted"])
        self.assertFalse(runner.canonicalize_runner_signal(
            "GLO_G1_CA", ("R", 7), -8)["accepted"])
        self.assertFalse(runner.canonicalize_runner_signal(
            "GLO_G1_CA", ("R", 7), 7)["accepted"])
        self.assertTrue(runner.canonicalize_runner_signal(
            "GLO_G1_CA", ("R", 7), -7)["accepted"])
        self.assertTrue(runner.canonicalize_runner_signal(
            "GLO_G1_CA", ("R", 7), 6)["accepted"])
        self.assertFalse(runner.canonicalize_runner_rinex(
            "C2P", ("R", 7), -4)["accepted"])

    def test_preflight_replaces_phase130_builder_and_records_python_evidence(self) -> None:
        route = runner.ROUTES[0]
        phone = {
            "ok": True,
            "rows": 1,
            "selected_rows": 1,
            "glonass_rows": 1,
            "invalid_rows": 0,
            "invalid_reasons": {},
            "typed_satellite_keys": True,
            "native_gpst_query_times": True,
            "carrier_frequency_used_as_fcn": False,
            "failure": None,
            "glonass": [{"satellite": ("R", 7), "query_gpst": 100.0,
                         "signal": "GLO_G1_CA"}],
        }
        imu = {"ok": True, "rows": 1, "failure": None}
        nav = {
            "ok": True,
            "records_seen": 1,
            "accepted_records": 1,
            "rejected_records": 0,
            "reject_counts": {},
            "satellite_count": 1,
            "canonical_field_positions": True,
            "fcn_field": "data[10]",
            "fcn_encoded_gt_128_subtract_256": True,
            "native_gpst_query_and_toe": True,
            "failure": None,
            "by_sat": {("R", 7): [{"toe_gpst": 100.0, "fcn": -4}]},
        }
        base_rows = [
            {"satellite": ("R", 7), "query_gpst": 100.0, "row_index": 0},
        ]
        base = {
            "ok": True,
            "bytes": 1,
            "header": {"status": "absent", "entries": {},
                        "entry_count": 0, "label_lines": 0,
                        "malformed_entries": 0, "conflict_entries": 0},
            "observations": base_rows,
            "observation_codes": {"R": ["C1C", "C1P"]},
            "mapping_failures": [],
            "earth_valid_reference": True,
            "antenna_semantics_proven": True,
            "glonass_rows": 1,
            "failure": None,
        }

        def resolve(satellite, query, nav_by_sat, header):
            del satellite, query, nav_by_sat, header
            return {"ok": True, "fcn": -4, "source": "broadcast-geph",
                    "age_s": 0.0}

        with patch.object(runner.p130, "build_inventory",
                          side_effect=AssertionError("Phase130 builder reused")), \
             patch.object(runner.p128, "inventory_phone_gnss",
                          return_value=phone), \
             patch.object(runner.p128, "inventory_imu", return_value=imu), \
             patch.object(runner.p128, "parse_nav_geph", return_value=nav), \
             patch.object(runner.p128, "parse_base_rinex", return_value=base), \
             patch.object(runner.p128, "resolve_query", side_effect=resolve), \
             patch.object(runner, "exact_base_observations",
                          return_value=base_rows):
            inventory = runner.build_inventory(
                route,
                {"device_gnss.csv": b"synthetic",
                 "device_imu.csv": b"synthetic",
                 "brdc.nav": b"synthetic",
                 "base.obs": b"synthetic"},
                {},
            )

        self.assertTrue(inventory["ok"])
        self.assertEqual(inventory["phase131"]["python_preflight_call_count"], 1)
        self.assertTrue(inventory["phase131"]["python_preflight_executed"])
        self.assertFalse(inventory["phase131"]["native_selector_forwarded"])
        self.assertEqual(inventory["phase131"]["canonical_key_count"], 1)
        self.assertEqual(inventory["phase131"]["canonical_key_examples"],
                         ["GLONASS:7:L1:fcn=-4"])
        self.assertEqual(inventory["rover"]["certified_rows"], 1)
        self.assertEqual(inventory["correction"]["retained_corrected_rows"], 1)

    def test_preflight_failure_records_solver_zero_and_no_selector_forwarding(self) -> None:
        record = runner.inventory_failure_record(
            runner.ROUTES[0], "synthetic canonical support failure", {"base.obs": 1})
        self.assertFalse(record["ok"])
        self.assertEqual(record["solver_invocations"], 0)
        self.assertFalse(record["solver_may_start"])
        phase = record["phase131"]
        self.assertTrue(phase["python_preflight_started"])
        self.assertFalse(phase["python_preflight_executed"])
        self.assertFalse(phase["native_selector_forwarded"])
        self.assertFalse(phase["native_command_constructed"])
        self.assertFalse(phase["native_binary_invocation_attempted"])
        self.assertEqual(phase["native_resolver_call_count"], 0)

    def test_command_construction_has_one_selector_and_no_old_payload_gate(self) -> None:
        auth = {
            "raw_inputs": {
                "device_gnss.csv": {"path": "raw/device_gnss.csv"},
                "device_imu.csv": {"path": "raw/device_imu.csv"},
                "brdc.nav": {"path": "raw/brdc.nav"},
            },
            "base_input": {"path": "base/base.obs", "sha256": "b" * 64},
        }
        command = runner.command_for(runner.ROUTES[0], auth)
        self.assertEqual(command.count(runner.contract.PHASE131_SELECTOR), 1)
        self.assertIn("raw/device_gnss.csv", command)
        self.assertIn("raw/device_imu.csv", command)
        self.assertIn("raw/brdc.nav", command)
        self.assertNotIn(runner.contract.PHASE117_SELECTOR, command)
        self.assertNotIn(runner.contract.PHASE120_SELECTOR, command)

    def test_native_summary_evidence_distinguishes_selector_from_resolver_rows(self) -> None:
        inventory = {
            "ok": True,
            "shared_ledger": {
                "side_local_conservation": True,
                "canonical_key_conservation": True,
                "unused_base_rows_accounted": True,
            },
            "phase131": {
                "python_preflight_executed": True,
                "native_selector_forwarded": True,
                "native_command_constructed": True,
                "native_binary_invocation_attempted": True,
            },
        }
        native = {
            "phase131_canonical_correction_band_key": {
                "enabled": True,
                "configuration_valid": True,
                "canonical_rows": 3,
                "canonical_rejected_rows": 2,
                "canonical_selected_streams": 1,
                "literal_tracking_code_in_join": False,
                "factor_topology_changed": False,
                "raw_or_zero_correction_fallback": False,
            },
            "selected_solver_branch": "MULTIFRONTAL_QR",
        }
        normalized, telemetry = runner.normalize_native(
            runner.ROUTES[0], native, {"opened": False}, 0, inventory)
        evidence = telemetry["execution_evidence"]
        self.assertTrue(evidence["native_selector_forwarded"])
        self.assertTrue(evidence["native_resolver_executed"])
        self.assertEqual(evidence["native_resolver_call_count"], 5)
        self.assertEqual(normalized["phase131"]["native_resolver_call_count"], 5)
        self.assertEqual(evidence["native_resolver_call_count_semantics"].startswith(
            "canonical rows plus rejected"), True)


if __name__ == "__main__":
    unittest.main()
