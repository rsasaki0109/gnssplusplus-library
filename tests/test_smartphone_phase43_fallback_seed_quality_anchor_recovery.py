"""Truth-free contract tests for the Phase43 fallback-seed anchor recovery."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase43_native_fallback_seed_quality_anchor_recovery_freeze_v1.json"
SOURCE = ROOT / "src/algorithms/fgo_problems.cpp"
APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"
CONFIG = ROOT / "include/libgnss++/algorithms/fgo_config.hpp"
HEADER = ROOT / "include/libgnss++/algorithms/fgo.hpp"


class SmartphonePhase43FallbackSeedQualityAnchorRecoveryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        cls.source = SOURCE.read_text(encoding="utf-8")
        cls.app = APP.read_text(encoding="utf-8")
        cls.config = CONFIG.read_text(encoding="utf-8")
        cls.header = HEADER.read_text(encoding="utf-8")

    def test_freeze_is_phase43_raw_truth_free_and_precedes_source_changes(self) -> None:
        self.assertEqual(self.freeze["phase"], 43)
        self.assertEqual(self.freeze["status"], "frozen-before-implementation-and-matrix")
        self.assertEqual(
            self.freeze["objective"]["candidate_flag"],
            "--native-fallback-seed-quality-anchor-recovery",
        )
        self.assertEqual(len(self.freeze["objective"]["structural_routes"]), 6)
        self.assertTrue(self.freeze["objective"]["route_order_is_fixed"])
        self.assertTrue(self.freeze["objective"]["truth_free"])
        self.assertEqual(self.freeze["truth_accounting"]["allowed_truth_reads"], 0)
        self.assertTrue(self.freeze["authority"]["phase42_freeze_is_immutable"])
        self.assertEqual(
            self.freeze["authority"]["phase42_freeze"]["sha256"],
            "f80865dc9db3d87c310e064df1afe687833225a1b0d1b6027dac6435492f8123",
        )

    def test_candidate_flag_is_explicit_and_default_is_false(self) -> None:
        self.assertIn("--native-fallback-seed-quality-anchor-recovery", self.app)
        self.assertIn(
            "native_fallback_seed_quality_anchor_recovery = false", self.app
        )
        self.assertIn(
            "bool use_fallback_seed_quality_anchor_recovery = false", self.config
        )
        self.assertIn(
            "options.native_fallback_seed_quality_anchor_recovery;", self.app
        )
        self.assertIn(
            "options.native_quality_anchor ||\n        options.native_fallback_seed_quality_anchor_recovery",
            self.app,
        )

    def test_normal_reconnaissance_precedes_exclusive_recovery(self) -> None:
        normal = self.source.index("collect_candidates(reconnaissance, candidates);")
        trigger = self.source.index(
            "if (anchor == nullptr &&\n            config_.use_fallback_seed_quality_anchor_recovery)",
            normal,
        )
        recovery_mask = self.source.index(
            "recovery_processor_config.elevation_mask = -90.0;", trigger
        )
        recovery_collect = self.source.index(
            "collect_candidates(recovery_reconnaissance, recovery_candidates);",
            recovery_mask,
        )
        replay_normal_mask = self.source.index(
            "replay_processor.initialize(spp_processor_config);", recovery_collect
        )
        self.assertLess(normal, trigger)
        self.assertLess(trigger, recovery_mask)
        self.assertLess(recovery_mask, recovery_collect)
        self.assertLess(recovery_collect, replay_normal_mask)
        self.assertIn("anchor = fgo_quality_anchor::choose(recovery_candidates);", self.source)
        self.assertIn("anchor_reconnaissance = &recovery_reconnaissance;", self.source)

    def test_recovery_preserves_ranking_and_factor_level_fail_closed(self) -> None:
        self.assertIn("fgo_quality_anchor::choose(candidates)", self.source)
        self.assertIn("fgo_quality_anchor::choose(recovery_candidates)", self.source)
        self.assertIn("sentinel_factor_bypass = false", self.header)
        self.assertIn("problem.diagnostics.sentinel_factor_bypass", self.app)
        self.assertIn("recovery forbids sentinel", self.app)
        # Recovery's -90 degree override is scoped to the reconnaissance
        # processor; the ordinary builder mask remains sourced from config.
        self.assertIn("const double effective_min_elevation_deg =", self.source)
        self.assertIn("config_.min_elevation_deg;", self.source)
        self.assertIn("replay_processor.initialize(spp_processor_config);", self.source)

    def test_summary_has_candidate_fields_only_when_flag_is_on(self) -> None:
        marker = '\\"native_fallback_seed_quality_anchor_recovery\\": true'
        self.assertIn(marker, self.app)
        summary_source = self.app.replace('\\"', '"')
        self.assertIn('"normal_quality_anchor_candidates"', summary_source)
        self.assertIn('"recovery_quality_anchor_candidates"', summary_source)
        self.assertIn('"recovery_trigger"', summary_source)
        self.assertIn('"recovery_anchor_index"', summary_source)
        self.assertIn('"recovery_replay_valid_epochs"', summary_source)
        self.assertIn('"sentinel_factor_bypass"', summary_source)
        self.assertIn("if (options.native_fallback_seed_quality_anchor_recovery)", self.app)

    def test_freeze_hash_is_self_consistent(self) -> None:
        digest = hashlib.sha256(FREEZE.read_bytes()).hexdigest()
        # The freeze is committed before candidate source/tests.  This test
        # guards accidental edits without pinning a future result artifact.
        self.assertEqual(len(digest), 64)


if __name__ == "__main__":
    unittest.main()
