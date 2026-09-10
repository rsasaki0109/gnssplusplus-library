#!/usr/bin/env python3
"""Truth-free contract/fixture tests for the v3 heading-optional lane."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_v3_heading_optional_freeze.json"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_v3_heading_optional_freeze_manifest.json"
SOURCE = ROOT / "apps/native/gnss_fgo_imu_v3_heading_optional.cpp"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def heading_decision(gravity_passed: bool, valid_windows: int, path_m: float,
                     span_s: float, scatter_deg: float) -> str:
    """Small executable specification of the frozen, truth-free gate."""
    if not gravity_passed:
        return "exact-v1-fallback"
    if (valid_windows >= 3 and path_m >= 2.0 and span_s >= 2.0 and
            scatter_deg <= 45.0):
        return "multi-epoch-gnss-course"
    return "broad-yaw-prior-joint-estimation"


class SmartphoneNativeFgoV3HeadingOptionalTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        cls.manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
        cls.source = SOURCE.read_text(encoding="utf-8")

    def test_selection_and_hashes_are_sealed_before_payload(self) -> None:
        self.assertEqual(self.freeze["status"], "frozen-before-v3-payload-materialization")
        self.assertFalse(self.freeze["truth_policy"]["selected_payload_materialized_before_this_record"])
        self.assertEqual(self.freeze["truth_policy"]["selected_truth_open_count_before_this_record"], 0)
        self.assertEqual(
            self.freeze["split_inventory"]["selected_train_smoke_confirmation"],
            [
                "2021-07-19-20-49-us-ca-mtv-a/pixel5",
                "2023-09-05-23-07-us-ca-routen/pixel5",
            ],
        )
        source_hash = sha256(SOURCE)
        self.assertEqual(source_hash, self.manifest["v3_source_sha256"])
        self.assertEqual(source_hash, self.freeze["v3_candidate"]["source_hashes"]["apps/native/gnss_fgo_imu_v3_heading_optional.cpp"])
        self.assertEqual(sha256(ROOT / "apps/native/gnss_fgo_imu_v21.cpp"),
                         self.freeze["v3_candidate"]["source_hashes"]["apps/native/gnss_fgo_imu_v21.cpp_dependency"])

    def test_stationary_and_weak_yaw_do_not_abort(self) -> None:
        self.assertEqual(heading_decision(True, 0, 0.0, 0.0, 0.0),
                         "broad-yaw-prior-joint-estimation")
        self.assertEqual(heading_decision(True, 1, 1.0, 1.0, 5.0),
                         "broad-yaw-prior-joint-estimation")
        self.assertIn("broad-yaw-prior-joint-estimation", self.source)
        self.assertIn("yaw_prior_sigma_rad = kV3BroadYawSigmaRad", self.source)

    def test_moving_course_and_gravity_fallback(self) -> None:
        self.assertEqual(heading_decision(True, 3, 12.0, 10.0, 10.0),
                         "multi-epoch-gnss-course")
        self.assertEqual(heading_decision(False, 3, 12.0, 10.0, 10.0),
                         "exact-v1-fallback")
        self.assertIn("tryAlignHeading", self.source)
        self.assertIn("gravity_passed", self.source)
        self.assertIn("exact v1 fallback", self.source)

    def test_gauge_and_forbidden_factor_contract(self) -> None:
        gauge = self.freeze["heading_optional_contract"]["weak_yaw_gauge"]
        self.assertTrue(gauge["prior_is_finite"])
        self.assertAlmostEqual(gauge["yaw_prior_sigma_rad"], 3.141592653589793)
        self.assertTrue(gauge["covariance_off_diagonal_reset"])
        self.assertEqual(
            self.freeze["v3_candidate"]["forbidden_graph_classes"],
            [
                "double-difference pseudorange",
                "double-difference carrier",
                "single-difference Doppler",
                "single-difference TDCP",
            ],
        )

    def test_taroz_initialization_evidence_is_pinned(self) -> None:
        upstream = self.freeze["v3_candidate"]["upstream_initialization_audit"]
        self.assertEqual(upstream["commit"], "29923f9f370f09ebc00f96d8cca375007a18e7d5")
        self.assertEqual(upstream["fgo_gnss_imu_m_sha256"],
                         "c70090ccb8b27fc8ac7fd2929e2f995a14cfe7f089bb1fef067370e22051c3e3")
        self.assertEqual(upstream["parameters_m_sha256"],
                         "518925e9c75c7a14fceb5cd99432fe883311e0d64c72b94396744ac765120f52")
        self.assertIn("0.5 m/s threshold", " ".join(upstream["evidence"]))


if __name__ == "__main__":
    unittest.main()
