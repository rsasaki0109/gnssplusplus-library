"""Launch-free Phase134 native-summary bridge tests.

Only synthetic summaries and temporary metadata files are used.  No raw
GNSS/IMU/nav/base payload, solution rows, truth, MAT/PDC/precomputed
coordinates, Kaggle resource, or native process is opened or launched.
"""

from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps/commands/benchmarks"))
import gnss_smartphone_phase131_canonical_correction_structural_authorized_execute as implementation  # noqa: E402
import gnss_smartphone_phase133_runner_native_selector_boundary_authorized_execute as wrapper  # noqa: E402


class Phase134NativeSummaryBridgeTests(unittest.TestCase):
    def _native_summary(self, *, resolver_count: int = 7) -> dict[str, object]:
        return {
            "status": "imu-combined-factor",
            "truth_used": False,
            "production_default_changed": False,
            "native_phase117_tdcp_snr_type_sigma": False,
            "native_phase120_official_tdcp_resl_atmosphere_cancellation": False,
            "native_direct_wls_ephemeral_c7d_main_seed_enabled": False,
            "selected_solver_branch": "MULTIFRONTAL_QR",
            "output_contract": {"finite_coordinates": True},
            "phase131_canonical_correction_band_key": {
                "enabled": True,
                "configuration_valid": True,
                "canonical_rows": 5,
                "canonical_rejected_rows": 2,
                "canonicalization_attempt_rows": 7,
                "resolver_call_count": resolver_count,
                "canonical_selected_streams": 3,
                "literal_tracking_code_in_join": False,
                "raw_or_zero_correction_fallback": False,
                "diagnostics_bridge": {
                    "source": "BasePseudorangeCompensationReport",
                    "synchronization_count": 1,
                    "exactly_once": True,
                },
                "correction_conservation": {
                    "source_miss_mask_enabled": True,
                    "canonical_key_mode": True,
                    "original_adopted_pseudorange_rows": 12,
                    "retained_finite_pc_pseudorange_rows": 10,
                    "dropped_missing_exact_stream_rows": 1,
                    "dropped_out_of_domain_rows": 1,
                    "correction_applied_exactly_once": True,
                },
            },
        }

    def test_native_bridge_fields_drive_resolver_and_conservation_view(self) -> None:
        native = self._native_summary()
        normalized, telemetry = implementation.normalize_native(
            "2021-03-16-18-59-us-ca-mtv-a/pixel5",
            native,
            {"opened": False},
            0,
            {
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
            },
        )
        self.assertTrue(normalized["gates"]["phase131_native_admission"])
        self.assertEqual(normalized["phase131"]["native_resolver_call_count"], 7)
        self.assertTrue(normalized["phase131"]["diagnostics_bridge_exactly_once"])
        self.assertEqual(
            normalized["phase131"]["correction_conservation"][
                "retained_finite_pc_pseudorange_rows"],
            10,
        )
        self.assertEqual(
            telemetry["execution_evidence"]["native_resolver_call_count_source"],
            "native-summary-resolver_call_count",
        )

    def test_inconsistent_explicit_resolver_count_fails_admission(self) -> None:
        native = self._native_summary(resolver_count=8)
        normalized, telemetry = implementation.normalize_native(
            "2021-03-16-18-59-us-ca-mtv-a/pixel5",
            native,
            {"opened": False},
            0,
            {"ok": True, "shared_ledger": {}, "phase131": {}},
        )
        self.assertFalse(normalized["gates"]["phase131_native_admission"])
        self.assertFalse(
            telemetry["execution_evidence"]["native_resolver_call_count_consistent"]
        )

    def test_native_summary_is_byte_exact_and_distinct_from_normalized_view(self) -> None:
        raw_bytes = b'{\n  "native": true,\n  "phase131": {"canonical_rows": 4}\n}\n'
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            native_path = root / "native_summary.json"
            normalized_path = root / "structural_summary.json"
            normalized_path.write_text(json.dumps({"normalized": True}), encoding="utf-8")
            native_path_input = root / "native_summary_input.json"
            native_path_input.write_bytes(raw_bytes)

            artifact = wrapper.preserve_native_summary(
                native_path_input, native_path)

            self.assertTrue(artifact["byte_exact"])
            self.assertEqual(native_path.read_bytes(), raw_bytes)
            self.assertNotEqual(native_path.resolve(), normalized_path.resolve())
            self.assertEqual(artifact["bytes"], len(raw_bytes))
            with self.assertRaises(wrapper.Phase133ExecutionError):
                wrapper.preserve_native_summary(native_path_input, native_path)

    def test_execution_command_uses_separate_native_summary_path(self) -> None:
        auth = {
            "raw_inputs": {
                "device_gnss.csv": {"path": "raw/device_gnss.csv"},
                "device_imu.csv": {"path": "raw/device_imu.csv"},
                "brdc.nav": {"path": "raw/brdc.nav"},
            },
            "base_input": {"path": "base/base.obs", "sha256": "a" * 64},
        }
        native_path = ROOT / "output/synthetic/native_summary.json"
        command = wrapper.command_for(
            "2021-03-16-18-59-us-ca-mtv-a/pixel5", auth, native_path)
        summary_argument = command[command.index("--summary-json") + 1]
        self.assertEqual(summary_argument, str(native_path))
        self.assertNotEqual(summary_argument.rsplit("/", 1)[-1],
                            "structural_summary.json")

    def test_zero_summary_bridge_does_not_fabricate_positive_rows(self) -> None:
        native = self._native_summary()
        p131 = native["phase131_canonical_correction_band_key"]
        p131["enabled"] = False
        p131["canonical_rows"] = 0
        p131["canonical_rejected_rows"] = 0
        p131["canonicalization_attempt_rows"] = 0
        p131["resolver_call_count"] = 0
        p131["canonical_selected_streams"] = 0
        p131["diagnostics_bridge"]["exactly_once"] = False
        p131["diagnostics_bridge"]["synchronization_count"] = 0
        normalized, _ = implementation.normalize_native(
            "2021-03-16-18-59-us-ca-mtv-a/pixel5",
            native,
            {"opened": False},
            0,
            {"ok": True, "shared_ledger": {}, "phase131": {}},
        )
        self.assertFalse(normalized["gates"]["phase131_native_admission"])
        self.assertEqual(normalized["phase131"]["native_resolver_call_count"], 0)


if __name__ == "__main__":
    unittest.main()
