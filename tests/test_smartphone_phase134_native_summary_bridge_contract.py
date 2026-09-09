"""Launch-free Phase134 summary-bridge contract tests.

All summaries are synthetic in-memory objects.  Temporary files, when used,
are ordinary non-payload metadata fixtures; no raw input, solution, truth,
MAT/PDC/precomputed coordinate file, or native process is opened.
"""

from __future__ import annotations

import copy
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps/commands/benchmarks"))
import gnss_smartphone_phase134_native_summary_bridge as contract  # noqa: E402


class Phase134NativeSummaryBridgeContractTests(unittest.TestCase):
    def _native_summary(self) -> dict[str, object]:
        canonical = {
            "enabled": True,
            "configuration_valid": True,
            "configuration_failure": "",
            "canonical_rows": 5,
            "canonical_rejected_rows": 2,
            "unknown_band_rows": 1,
            "canonical_key_conflicts": 0,
            "canonical_duplicate_rows": 0,
            "canonical_streams": 4,
            "canonical_selected_streams": 3,
            "canonical_merged_streams": 1,
            "failure_counts": {"unknown-band": 1, "missing-fcn": 1},
        }
        base = {
            "phase131_canonical_correction_band_key": True,
            "phase131_configuration_valid": True,
            "phase131_configuration_failure": "",
            "phase131_canonical_rows": 5,
            "phase131_canonical_rejected_rows": 2,
            "phase131_unknown_band_rows": 1,
            "phase131_canonical_key_conflicts": 0,
            "phase131_canonical_duplicate_rows": 0,
            "phase131_canonical_streams": 4,
            "phase131_canonical_selected_streams": 3,
            "phase131_canonical_merged_streams": 1,
            "phase131_failure_counts": {"unknown-band": 1, "missing-fcn": 1},
            "source_miss_mask_canonical_key_mode": True,
            "source_miss_mask_matching_key": "(GNSSSystem,PRN,physical-frequency-family[,certified-GLO-FCN])",
            "matched_factor_rows": 8,
            "finite_correction_rows_among_matched": 8,
            "source_model_build_count": 1,
            "correction_application_pass_count": 1,
            "applied": True,
            "correction_applied_exactly_once": True,
            "duplicate_correction_rejected": False,
        }
        mask = {
            "enabled": True,
            "canonical_key_mode": True,
            "matching_key": base["source_miss_mask_matching_key"],
            "original_adopted_pseudorange_rows": 10,
            "retained_finite_pc_pseudorange_rows": 8,
            "dropped_missing_exact_stream_rows": 1,
            "dropped_out_of_domain_rows": 1,
            "dropped_nonfinite_correction_rows": 0,
            "corrected_rows": 8,
            "pseudorange_factor_count_consistent": True,
            "signal_count_consistent": True,
        }
        conservation = {
            "source_miss_mask_enabled": True,
            "canonical_key_mode": True,
            "matching_key": base["source_miss_mask_matching_key"],
            "original_adopted_pseudorange_rows": 10,
            "retained_finite_pc_pseudorange_rows": 8,
            "dropped_missing_exact_stream_rows": 1,
            "dropped_out_of_domain_rows": 1,
            "dropped_nonfinite_correction_rows": 0,
            "matched_factor_rows": 8,
            "finite_correction_rows_among_matched": 8,
            "source_model_build_count": 1,
            "correction_application_pass_count": 1,
            "corrected_rows": 8,
            "pseudorange_factor_count_consistent": True,
            "signal_count_consistent": True,
            "applied": True,
            "correction_applied_exactly_once": True,
            "duplicate_correction_rejected": False,
        }
        p131 = dict(canonical)
        p131.update({
            "canonicalization_attempt_rows": 7,
            "resolver_call_count": 7,
            "correction_conservation": conservation,
            "diagnostics_bridge": {
                "source": "BasePseudorangeCompensationReport",
                "synchronization_count": 1,
                "exactly_once": True,
            },
        })
        return {
            "status": "imu-combined-factor",
            "selected_solver_branch": "MULTIFRONTAL_QR",
            "native_base_pseudorange_compensation": base,
            "native_base_pseudorange_source_miss_mask": mask,
            "phase131_canonical_correction_band_key": p131,
        }

    def test_freeze_and_static_pins_are_valid(self) -> None:
        freeze = contract.verify_freeze()
        self.assertEqual(freeze["candidate"]["count"], 1)
        self.assertTrue(freeze["bridge_contract"]["native_bytes_hash_immutable"])

    def test_command_snapshot_is_exact_and_summary_paths_are_separate(self) -> None:
        for route in contract.ROUTES:
            command = contract.command_template(route)
            contract.validate_command(route, command)
            self.assertEqual(command.count(contract.PHASE130_SELECTOR), 0)
            self.assertEqual(command.count(contract.PHASE131_SELECTOR), 1)
            summary = command[command.index("--summary-json") + 1]
            self.assertTrue(summary.endswith("/native_summary.json"))
            self.assertNotIn("structural_summary.json", summary)

    def test_nonzero_bridge_is_an_exact_base_copy(self) -> None:
        evidence = contract.validate_bridge_summary(self._native_summary())
        self.assertTrue(evidence["bridge_exactly_once"])
        self.assertTrue(evidence["base_top_level_exact_copy"])
        self.assertEqual(evidence["resolver_call_count"], 7)

    def test_corrupt_counter_or_resolver_is_rejected(self) -> None:
        native = self._native_summary()
        native["phase131_canonical_correction_band_key"]["canonical_rows"] = 6
        with self.assertRaises(contract.Phase134ContractError):
            contract.validate_bridge_summary(native)

        native = self._native_summary()
        native["phase131_canonical_correction_band_key"]["resolver_call_count"] = 8
        with self.assertRaises(contract.Phase134ContractError):
            contract.validate_bridge_summary(native)

    def test_corrupt_conservation_or_double_sync_is_rejected(self) -> None:
        native = self._native_summary()
        native["phase131_canonical_correction_band_key"]["correction_conservation"][
            "corrected_rows"] = 7
        with self.assertRaises(contract.Phase134ContractError):
            contract.validate_bridge_summary(native)

        native = self._native_summary()
        native["phase131_canonical_correction_band_key"]["diagnostics_bridge"][
            "synchronization_count"] = 2
        with self.assertRaises(contract.Phase134ContractError):
            contract.validate_bridge_summary(native)

    def test_zero_disabled_bridge_does_not_fabricate_positive_admission(self) -> None:
        native = self._native_summary()
        p131 = native["phase131_canonical_correction_band_key"]
        p131["enabled"] = False
        p131["configuration_valid"] = False
        p131["canonical_rows"] = 0
        p131["canonical_rejected_rows"] = 0
        p131["canonicalization_attempt_rows"] = 0
        p131["resolver_call_count"] = 0
        p131["canonical_selected_streams"] = 0
        for field in ("canonical_key_conflicts", "canonical_duplicate_rows",
                      "canonical_streams", "canonical_merged_streams",
                      "unknown_band_rows"):
            p131[field] = 0
        p131["failure_counts"] = {}
        conservation = p131["correction_conservation"]
        for field in ("original_adopted_pseudorange_rows",
                      "retained_finite_pc_pseudorange_rows",
                      "dropped_missing_exact_stream_rows",
                      "dropped_out_of_domain_rows",
                      "dropped_nonfinite_correction_rows",
                      "matched_factor_rows",
                      "finite_correction_rows_among_matched",
                      "source_model_build_count",
                      "correction_application_pass_count",
                      "corrected_rows"):
            conservation[field] = 0
        for field in ("source_miss_mask_enabled", "canonical_key_mode",
                      "pseudorange_factor_count_consistent",
                      "signal_count_consistent", "applied",
                      "correction_applied_exactly_once",
                      "duplicate_correction_rejected"):
            conservation[field] = False
        p131["diagnostics_bridge"]["exactly_once"] = False
        p131["diagnostics_bridge"]["synchronization_count"] = 0
        base = native["native_base_pseudorange_compensation"]
        base["phase131_canonical_correction_band_key"] = False
        base["phase131_configuration_valid"] = False
        base["phase131_unknown_band_rows"] = 0
        base["phase131_canonical_key_conflicts"] = 0
        base["phase131_canonical_duplicate_rows"] = 0
        base["phase131_canonical_streams"] = 0
        base["phase131_canonical_merged_streams"] = 0
        base["phase131_failure_counts"] = {}
        base["phase131_canonical_rows"] = 0
        base["phase131_canonical_rejected_rows"] = 0
        base["phase131_canonical_selected_streams"] = 0
        base["source_miss_mask_canonical_key_mode"] = False
        base["source_miss_mask_matching_key"] = "(satellite,signal)"
        base["matched_factor_rows"] = 0
        base["finite_correction_rows_among_matched"] = 0
        base["source_model_build_count"] = 0
        base["correction_application_pass_count"] = 0
        base["applied"] = False
        base["correction_applied_exactly_once"] = False
        base["duplicate_correction_rejected"] = False
        mask = native["native_base_pseudorange_source_miss_mask"]
        mask["enabled"] = False
        mask["canonical_key_mode"] = False
        mask["matching_key"] = "(satellite,signal)"
        mask["original_adopted_pseudorange_rows"] = 0
        mask["retained_finite_pc_pseudorange_rows"] = 0
        mask["corrected_rows"] = 0
        mask["pseudorange_factor_count_consistent"] = False
        mask["signal_count_consistent"] = False
        self.assertEqual(
            native["phase131_canonical_correction_band_key"]["resolver_call_count"], 0)
        with self.assertRaises(contract.Phase134ContractError):
            # The disabled/default shape has no valid bridge admission.  It
            # must remain zero/fail-closed rather than being promoted.
            contract.validate_bridge_summary(native, require_positive=False,
                                             require_applied=False)

    def test_summary_views_require_immutable_native_and_distinct_normalized(self) -> None:
        native = {
            "path": "route/native_summary.json",
            "bytes": 32,
            "sha256": "a" * 64,
            "source": "native-process",
            "byte_exact": True,
            "overwritten": False,
        }
        normalized = {
            "path": "route/structural_summary.json",
            "source": "wrapper-normalized-view",
            "source_native_path": "route/native_summary.json",
        }
        result = contract.validate_summary_views(native, normalized)
        self.assertFalse(result["native_summary_overwrite"])
        bad = dict(normalized)
        bad["path"] = native["path"]
        with self.assertRaises(contract.Phase134ContractError):
            contract.validate_summary_views(native, bad)

    def test_synthetic_execution_evidence_is_launch_free(self) -> None:
        result = contract.verify_synthetic_execution_evidence({
            "phase134": {
                "bridge_sync_count": 1,
                "native_summary_overwrite": False,
                "normalized_summary_distinct": True,
                "resolver_call_count": 3,
                "native_solver_invocations": 0,
            }
        }, contract.ROUTES[0])
        self.assertTrue(result["launch_free"])

    def test_pre_raw_accounting_schema_is_explicit(self) -> None:
        accounting = contract.zero_read_accounting()
        self.assertEqual(accounting["raw_phone_gnss_reads"], 0)
        self.assertEqual(accounting["native_solver_invocations"], 0)
        self.assertEqual(accounting["truth_reads"], 0)
        self.assertEqual(accounting["mat_pdc_precomputed_coordinate_reads"], 0)


if __name__ == "__main__":
    unittest.main()
