#!/usr/bin/env python3
"""Regression checks for the published GSDC 2023 reproducibility audit."""

from __future__ import annotations

import json
import re
import unittest
from pathlib import Path
from urllib.parse import urlparse


ROOT_DIR = Path(__file__).resolve().parents[1]
RECORD_PATH = (
    ROOT_DIR
    / "docs"
    / "use_cases"
    / "records"
    / "smartphone_r5_gsdc2023_published_solution_reproducibility_audit.json"
)
MANIFEST_PATH = RECORD_PATH.with_name(
    "smartphone_r5_gsdc2023_published_solution_reproducibility_manifest.json"
)
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
COMMIT_RE = re.compile(r"^[0-9a-f]{40}$")
ALLOWED_HOSTS = {
    "github.com",
    "www.github.com",
    "ion.org",
    "www.ion.org",
    "kaggle.com",
    "www.kaggle.com",
    "taroz.net",
    "www.taroz.net",
}


class PublishedSolutionReproducibilityAuditTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.payload = json.loads(RECORD_PATH.read_text(encoding="utf-8"))

    def test_record_is_sealed_without_external_mutation(self) -> None:
        self.assertEqual(
            self.payload["schema_version"],
            "smartphone-r5-gsdc2023-published-solution-reproducibility-audit.v1",
        )
        self.assertEqual(self.payload["status"], "blocked-at-matlab-mex-wrapper")
        policy = self.payload["policy"]
        self.assertFalse(policy["external_mutation"])
        self.assertFalse(policy["kaggle_submission"])
        self.assertFalse(policy["token_access"])
        self.assertFalse(policy["leaderboard_scores_used_for_tuning"])
        self.assertFalse(policy["fresh_validation_opened"])
        self.assertFalse(policy["future_holdout_opened"])
        self.assertFalse(policy["production_default_changed"])
        self.assertFalse(policy["source_copied_into_production"])

    def test_primary_urls_are_allowlisted(self) -> None:
        urls = [item["url"] for item in self.payload["primary_source_evidence"]]
        urls.append(self.payload["published_repository"]["url"])
        urls.extend(item["url"] for item in self.payload["dependency_repositories"])
        urls.append(self.payload["public_artifacts"]["preprocessed_dataset"]["source_url_as_published"])
        urls.append(self.payload["public_artifacts"]["preprocessed_dataset"]["redirected_url_observed"])
        urls.append(self.payload["public_artifacts"]["repository_sample"]["source_url"])
        for url in urls:
            parsed = urlparse(url)
            self.assertIn(parsed.scheme, {"http", "https"}, url)
            self.assertIn(parsed.hostname, ALLOWED_HOSTS, url)

    def test_repository_and_artifact_hashes_are_well_formed(self) -> None:
        repo = self.payload["published_repository"]
        self.assertRegex(repo["commit"], COMMIT_RE)
        self.assertRegex(repo["git_tree_listing_sha256"], SHA256_RE)
        self.assertRegex(repo["license_sha256"], SHA256_RE)
        self.assertRegex(repo["readme_sha256"], SHA256_RE)
        self.assertRegex(repo["public_output_sample"]["sha256"], SHA256_RE)
        self.assertEqual(repo["public_output_sample"]["data_rows"], 71936)

        for dependency in self.payload["dependency_repositories"]:
            self.assertRegex(dependency["commit"], COMMIT_RE)
            self.assertRegex(dependency["git_tree_listing_sha256"], SHA256_RE)
            self.assertRegex(dependency["license_sha256"], SHA256_RE)
            self.assertIn(
                dependency["license"],
                {"MIT", "BSD 3-Clause", "Simplified BSD; repository also contains LICENSE.BSD"},
            )
            self.assertTrue(dependency["retrievable_without_credentials"])

        artifact = self.payload["public_artifacts"]["preprocessed_dataset"]
        self.assertEqual(artifact["content_length_bytes"], 2761355999)
        self.assertEqual(artifact["local_size_bytes"], artifact["content_length_bytes"])
        self.assertRegex(artifact["local_sha256"], SHA256_RE)
        for item in self.payload["route_materialization"]["files"].values():
            self.assertRegex(item["sha256"], SHA256_RE)

    def test_entry_points_and_blocker_are_explicit(self) -> None:
        entry_points = {item["path"]: item for item in self.payload["published_repository"]["entry_points"]}
        self.assertEqual(entry_points["run_fgo.m"]["hardcoded_dataset"], "test")
        self.assertEqual(entry_points["run_fgo.m"]["sha256"], "02150849653b2e5c80609e971dbef41b010569de5dcc304782235b7fc96b9065")
        execution = self.payload["execution_evidence"]
        self.assertEqual(execution["gtsam_gnss_without_gtwrap"]["return_code"], 1)
        self.assertEqual(execution["gtsam_gnss_with_gtwrap"]["return_code"], 1)
        self.assertIn("FindMatlab", execution["gtsam_gnss_with_gtwrap"]["first_blocker"])
        self.assertEqual(execution["taroz_matlab_entrypoint"]["return_code"], 127)
        self.assertEqual(execution["local_native_factor_smoke"]["return_code"], 0)
        self.assertTrue(execution["local_native_factor_smoke"]["status"].endswith("not-taroz-reproduction"))

    def test_truth_boundary_and_attribution_are_preserved(self) -> None:
        truth = self.payload["truth_and_leakage"]
        self.assertEqual(truth["fresh_validation_payload_materialized"], 0)
        self.assertEqual(truth["fresh_validation_truth_open_count"], 0)
        self.assertEqual(truth["future_holdout_payload_materialized"], 0)
        self.assertEqual(truth["future_holdout_truth_open_count"], 0)
        self.assertFalse(truth["old_holdout_used_for_selection"])
        search = self.payload["exact_first_and_third_place_repository_search"]
        self.assertIsNone(search["first_place_public_repository_linked_by_primary_sources"])
        self.assertIsNone(search["third_place_public_repository_linked_by_primary_sources"])
        self.assertIn("Taro Suzuki", search["taroz_attribution"])

    def test_audit_contains_no_token_material(self) -> None:
        serialized = json.dumps(self.payload, sort_keys=True)
        for marker in ("KAGGLE_API_TOKEN", "KAGGLE_KEY", "Bearer ", "access_token="):
            self.assertNotIn(marker, serialized)

    def test_separate_manifest_pins_record_hash(self) -> None:
        manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
        self.assertEqual(manifest["record_path"], str(RECORD_PATH.relative_to(ROOT_DIR)))
        self.assertEqual(
            manifest["record_sha256"],
            __import__("hashlib").sha256(RECORD_PATH.read_bytes()).hexdigest(),
        )
        self.assertEqual(manifest["published_repository"]["commit"], self.payload["published_repository"]["commit"])
        self.assertFalse(manifest["contract"]["external_mutation"])
        self.assertFalse(manifest["contract"]["token_access"])


if __name__ == "__main__":
    unittest.main()
