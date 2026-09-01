"""Focused, non-raw tests for the Phase56 evidence-map/dedup audit."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest


ROOT = Path(__file__).resolve().parents[1]
BENCHMARKS = ROOT / "apps" / "commands" / "benchmarks"
sys.path.insert(0, str(BENCHMARKS))

import gnss_smartphone_phase56_bias_uncertainty_dedup as audit  # noqa: E402


class Phase56BiasUncertaintyDedupTest(unittest.TestCase):
    def test_freeze_is_pre_read_and_dedup_policy_is_explicit(self) -> None:
        freeze = audit._verify_freeze()
        self.assertEqual(freeze["status"], "frozen-before-phase56-new-raw-read")
        self.assertEqual(freeze["scope"]["new_raw_reads"], 0)
        self.assertEqual(freeze["scope"]["truth_or_metric_payload_inputs"], 0)
        self.assertEqual(
            freeze["phase46_evidence_map"]["bias_uncertainty_source"]["field"],
            "BiasUncertaintyNanos",
        )
        self.assertEqual(
            freeze["next_nonduplicate_factor"]["field"],
            "PseudorangeRateUncertaintyMetersPerSecond",
        )

    def test_source_contract_proves_rate_uncertainty_is_not_currently_consumed(self) -> None:
        freeze = audit._verify_freeze()
        evidence = audit._source_evidence(freeze)
        self.assertTrue(evidence["adapter"]["parses_pseudorange_rate_mps"])
        self.assertFalse(evidence["adapter"]["parses_pseudorange_rate_uncertainty_mps"])
        self.assertTrue(evidence["adapter"]["parses_bias_uncertainty_nanos"])
        self.assertFalse(evidence["observation"]["retains_pseudorange_rate_uncertainty_mps"])
        self.assertFalse(evidence["fgo"]["consumes_raw_rate_uncertainty"])
        self.assertTrue(evidence["fgo"]["uses_fixed_undifferenced_doppler_sigma"])
        self.assertTrue(evidence["fgo"]["uses_fixed_single_difference_doppler_sigma"])
        self.assertTrue(evidence["fgo"]["uses_snr_derived_upstream_doppler_sigma"])
        self.assertFalse(evidence["cli"]["raw_rate_uncertainty_option"])

    def test_sealed_artifact_read_accounting_is_one_each(self) -> None:
        freeze = audit._verify_freeze()
        sealed, reads = audit._verify_artifacts(freeze)
        self.assertEqual(set(reads), {
            "phase46_freeze",
            "phase46_evaluator_manifest",
            "phase46_result_record",
            "phase46_output_manifest",
            "phase55_policy_result",
            "phase46_output_result",
            "phase46_output_routes",
            "phase46_output_events",
        })
        self.assertTrue(all(count == 1 for count in reads.values()))
        route_evidence = sealed["phase46_bias_uncertainty_route_evidence"]
        self.assertEqual(
            set(route_evidence["count_by_route"]), set(audit.ROUTES)
        )
        self.assertTrue(route_evidence["all_bias_uncertainty_populated"])
        self.assertTrue(route_evidence["same_epoch_base_clock_spread_zero"])
        self.assertTrue(route_evidence["same_epoch_signal_spread_zero"])
        self.assertTrue(route_evidence["noncommon_tow_effect_zero"])

    def test_evaluator_is_static_and_forbidden_inputs_are_not_invoked(self) -> None:
        source = Path(audit.__file__).read_text(encoding="utf-8")
        self.assertNotIn("subprocess", source)
        self.assertNotIn("truth_maps", source)
        self.assertNotIn("read_csv", source)
        self.assertNotIn("open_truth", source)


if __name__ == "__main__":
    unittest.main()
