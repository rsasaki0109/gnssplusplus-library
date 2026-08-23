#!/usr/bin/env python3

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
import zipfile


ROOT = Path(__file__).resolve().parents[1]
COMMANDS = ROOT / "apps" / "commands"
sys.path.insert(0, str(COMMANDS))
SPEC = importlib.util.spec_from_file_location(
    "smartphone_workflow",
    COMMANDS / "benchmarks" / "gnss_smartphone_gnss_workflow.py",
)
assert SPEC is not None and SPEC.loader is not None
WORKFLOW = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(WORKFLOW)


def digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


class SmartphoneGnssWorkflowTests(unittest.TestCase):
    def test_materializes_only_frozen_hash_verified_members(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_smartphone_workflow_") as temp_dir:
            root = Path(temp_dir)
            archive_path = root / "dataset.zip"
            route = "route-a"
            phone = "pixel-test"
            payloads = {
                "device_gnss": b"device-data\n",
                "ground_truth": b"truth-data\n",
                "broadcast_nav": b"nav-data\n",
            }
            members = {
                "device_gnss": f"dataset_2023/train/{route}/{phone}/device_gnss.csv",
                "ground_truth": f"dataset_2023/train/{route}/{phone}/ground_truth.csv",
                "broadcast_nav": f"dataset_2023/train/{route}/brdc.nav",
            }
            with zipfile.ZipFile(archive_path, "w") as archive:
                for key, member in members.items():
                    archive.writestr(member, payloads[key])
                archive.writestr("dataset_2023/train/unselected/secret.txt", b"not extracted")
            profile = {
                "schema_version": "smartphone-r5-profile.v1",
                "archive": {"sha256": WORKFLOW.sha256(archive_path)},
                "datasets": {
                    "development": {
                        "id": f"{route}/{phone}",
                        "device_gnss_sha256": digest(payloads["device_gnss"]),
                        "ground_truth_sha256": digest(payloads["ground_truth"]),
                        "broadcast_nav_sha256": digest(payloads["broadcast_nav"]),
                    }
                },
            }
            destination = root / "inputs"
            paths = WORKFLOW.materialize_inputs(
                archive_path, profile, "development", destination
            )
            self.assertEqual(paths["device_gnss"].read_bytes(), payloads["device_gnss"])
            self.assertEqual(paths["ground_truth"].read_bytes(), payloads["ground_truth"])
            self.assertEqual(paths["broadcast_nav"].read_bytes(), payloads["broadcast_nav"])
            self.assertEqual({path.name for path in destination.iterdir()}, {
                "device_gnss.csv", "ground_truth.csv", "brdc.nav"
            })

    def test_tracked_profile_preserves_holdout_and_negative_variant(self) -> None:
        profile = json.loads(
            (ROOT / "configs/benchmarks/smartphone_r5_gsdc2023.json").read_text(
                encoding="utf-8"
            )
        )
        self.assertTrue(profile["datasets"]["holdout"]["sealed_before_threshold_freeze"])
        self.assertEqual(profile["holdout_run1"]["pre_holdout_commit"], "98b15f4")
        self.assertTrue(profile["holdout_run1"]["all_frozen_gates_passed"])
        self.assertEqual(
            profile["precise_product_development_lane"]["status"],
            "evaluated-not-promoted",
        )
        self.assertEqual(profile["thresholds"]["horizontal_p95_max"], 25.0)


if __name__ == "__main__":
    unittest.main()
