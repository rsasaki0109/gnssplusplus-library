"""Tests for R8 empirical protection-envelope geofence decisions."""

from __future__ import annotations

from pathlib import Path
import sys
import unittest


ROOT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT_DIR / "apps/commands"))
sys.path.insert(0, str(ROOT_DIR / "apps/commands/benchmarks"))

import gnss_integrity_geofence_signoff as signoff  # noqa: E402
import gnss_integrity_geofence_workflow as workflow  # noqa: E402


class IntegrityGeofenceSignoffTest(unittest.TestCase):
    def test_age_population_fails_closed_after_sixty_seconds(self) -> None:
        self.assertEqual(signoff.age_bin(0.0, True), "fixed")
        self.assertEqual(signoff.age_bin(5.0, False), "bridge_0_5s")
        self.assertEqual(signoff.age_bin(15.0, False), "bridge_5_15s")
        self.assertEqual(signoff.age_bin(60.0, False), "bridge_30_60s")
        self.assertIsNone(signoff.age_bin(60.1, False))
        self.assertIsNone(signoff.age_bin(None, False))

    def test_development_envelopes_cover_maxima_and_do_not_shrink_with_age(self) -> None:
        errors = {
            "fixed": [0.1, 2.0],
            "bridge_0_5s": [10.0],
            "bridge_5_15s": [5.0],
            "bridge_15_30s": [20.0],
            "bridge_30_60s": [4.0],
        }
        profile = signoff.candidate_profile(errors, 500.0)
        envelopes = profile["empirical_protection_envelopes_m"]
        bridge = [envelopes[name] for name, _, _ in signoff.AGE_BINS[1:]]
        self.assertEqual(bridge, sorted(bridge))
        self.assertGreater(envelopes["fixed"], max(errors["fixed"]))
        self.assertIn("not a certified integrity", profile["claim_boundary"])

    def test_scoring_counts_unknown_separately_and_rejects_misleading(self) -> None:
        rows = [
            {"decision": "inside", "truth_decision": "inside", "misleading_decision": False,
             "population": "fixed", "horizontal_error_m": 1.0, "envelope_exceeded": False,
             "unknown_reason": None},
            {"decision": "outside", "truth_decision": "inside", "misleading_decision": True,
             "population": "fixed", "horizontal_error_m": 3.0, "envelope_exceeded": True,
             "unknown_reason": None},
            {"decision": "unknown", "truth_decision": "outside", "misleading_decision": False,
             "population": None, "horizontal_error_m": None, "envelope_exceeded": False,
             "unknown_reason": "unqualified_or_over_60s_since_fix"},
        ]
        gates = {
            "maximum_misleading_decisions": 0,
            "maximum_envelope_exceedance_pct": 0.1,
            "minimum_decisive_availability_pct": 50.0,
        }
        result = signoff.score_rows(rows, gates)
        self.assertEqual(result["gate"]["status"], "failed")
        self.assertEqual(result["populations"]["unknown"], 1)
        self.assertIn("misleading_decision_count_above_maximum", result["gate"]["failures"])

    def test_surface_distance_is_zero_and_symmetric(self) -> None:
        self.assertEqual(signoff.surface_distance_m(35.0, 139.0, 35.0, 139.0), 0.0)
        forward = signoff.surface_distance_m(35.0, 139.0, 35.001, 139.002)
        reverse = signoff.surface_distance_m(35.001, 139.002, 35.0, 139.0)
        self.assertAlmostEqual(forward, reverse, places=9)

    def test_frozen_profile_is_sealed_and_never_claims_certified_integrity(self) -> None:
        import json

        profile = json.loads(
            (ROOT_DIR / "configs/benchmarks/integrity_geofence_r8.json").read_text(encoding="utf-8")
        )
        self.assertEqual(profile["release_state"], "sealed")
        self.assertIn("not certified integrity", profile["claim_boundary"])
        self.assertEqual(profile["maximum_qualified_age_s"], 60.0)

    def test_opened_workflow_manifest_refuses_holdout_rerun(self) -> None:
        import tempfile

        with tempfile.TemporaryDirectory(prefix="r8_opened_") as temp_dir:
            root = Path(temp_dir); output = root / "output"; output.mkdir()
            (output / "workflow_manifest.json").write_text("{}\n", encoding="utf-8")
            args = workflow.parse_args([
                "--phase", "holdout", "--mode", "full", "--data-dir", str(root / "data"),
                "--output-dir", str(output),
            ])
            self.assertEqual(workflow.run(args), 2)


if __name__ == "__main__":
    unittest.main()
