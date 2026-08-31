#!/usr/bin/env python3

from __future__ import annotations

import copy
import hashlib
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest import mock
import zipfile


ROOT = Path(__file__).resolve().parents[1]
COMMANDS = ROOT / "apps" / "commands"
BENCHMARKS = COMMANDS / "benchmarks"
import sys

sys.path.insert(0, str(COMMANDS))
sys.path.insert(0, str(BENCHMARKS))
SPEC = importlib.util.spec_from_file_location(
    "smartphone_generalization",
    BENCHMARKS / "gnss_smartphone_generalization.py",
)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


FIXED_IDS = MODULE.FIXED_CANDIDATE_IDS


def _digest(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _profile(archive_sha: str) -> dict:
    return {
        "schema_version": "smartphone-r5-profile.v1",
        "profile_id": "fixture",
        "archive": {"sha256": archive_sha, "url": "fixture", "source_terms": "fixture"},
        "datasets": {
            "development": {
                "id": "development-route/pixel7pro",
                "device_model": "pixel7pro",
                "skip_epochs": 1,
            },
            "holdout": {
                "id": "sealed-route/pixel7pro",
                "device_model": "pixel7pro",
            },
        },
    }


class SmartphoneGeneralizationTests(unittest.TestCase):
    def _fixture_archive(self, root: Path) -> tuple[Path, dict[str, dict[str, bytes]]]:
        archive_path = root / "dataset.zip"
        payloads: dict[str, dict[str, bytes]] = {}
        with zipfile.ZipFile(archive_path, "w") as archive:
            for dataset_id in FIXED_IDS:
                route, phone = dataset_id.split("/", 1)
                values = {
                    "device_gnss.csv": b"MessageType,utcTimeMillis\nRaw,1000\nRaw,2000\n",
                    "device_imu.csv": b"MessageType,utcTimeMillis\nUncalGyro,1000\n",
                    "ground_truth.csv": b"UnixTimeMillis,LatitudeDegrees,LongitudeDegrees,AltitudeMeters\n1000,1,2,3\n",
                }
                payloads[dataset_id] = values
                for name, data in values.items():
                    archive.writestr(
                        f"dataset_2023/train/{route}/{phone}/{name}", data
                    )
                archive.writestr(f"dataset_2023/train/{route}/brdc.nav", b"nav\n")
            archive.writestr("dataset_2023/train/unselected/secret.txt", b"secret")
        return archive_path, payloads

    def test_inventory_uses_central_directory_without_opening_members(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_generalization_") as temp_dir:
            archive_path, _ = self._fixture_archive(Path(temp_dir))
            original_open = zipfile.ZipFile.open
            with mock.patch.object(
                zipfile.ZipFile,
                "open",
                side_effect=AssertionError("inventory opened a member"),
            ):
                inventory = MODULE.inventory_archive(archive_path)
            self.assertTrue(inventory["archive"]["central_directory_only"])
            self.assertFalse(inventory["archive"]["member_content_read"])
            self.assertEqual(inventory["train"]["route_phone_count"], 3)
            self.assertEqual(original_open, zipfile.ZipFile.open)

    def test_selection_excludes_profile_routes_and_requires_unique_nav(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_generalization_") as temp_dir:
            archive_path, _ = self._fixture_archive(Path(temp_dir))
            inventory = MODULE.inventory_archive(archive_path)
            profile = _profile("unused")
            selected = MODULE.select_candidates(inventory, profile)
            self.assertEqual([row["dataset_id"] for row in selected], list(FIXED_IDS))
            self.assertEqual(len({row["route"] for row in selected}), 3)
            self.assertEqual(len({row["phone"] for row in selected}), 3)

    def test_materialization_hashes_only_selected_members(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_generalization_") as temp_dir:
            root = Path(temp_dir)
            archive_path, _ = self._fixture_archive(root)
            archive_sha = MODULE._sha256(archive_path)
            base = _profile(archive_sha)
            inventory = MODULE.inventory_archive(archive_path)
            candidate = MODULE.select_candidates(inventory, base)[0]
            member_hashes = MODULE._discover_member_hashes(
                archive_path, MODULE._member_names(candidate["route"], candidate["phone"])
            )
            candidate_profile = MODULE._candidate_profile(
                base, candidate, archive_sha, member_hashes
            )
            output = root / "out"
            materialized = MODULE.materialize_candidate(
                archive_path,
                candidate_profile,
                candidate,
                archive_sha,
                output,
                member_hashes,
            )
            inputs = materialized["inputs"]
            self.assertEqual(
                {path.name for path in inputs.iterdir()},
                {"device_gnss.csv", "device_imu.csv", "ground_truth.csv", "brdc.nav"},
            )
            self.assertEqual(
                MODULE._sha256(inputs / "device_imu.csv"),
                member_hashes["device_imu"]["sha256"],
            )
            self.assertFalse((output / "routes" / "sealed-route").exists())

    def test_score_reports_per_phone_p50_p95_and_diagnostics(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_generalization_") as temp_dir:
            root = Path(temp_dir)
            # week=2200, TOW=100000 maps to this UTC millisecond with the
            # smoother's fixed 18-second leap-second contract.
            timestamp = int(315964800000 + 2200 * 604800000 + 100000000 - 18000)
            device = root / "device.csv"
            device.write_text(
                "MessageType,utcTimeMillis\n"
                f"Raw,{timestamp - 1000}\n"
                f"Raw,{timestamp}\n",
                encoding="utf-8",
            )
            position = root / "position.pos"
            position.write_text(
                "% test\n"
                f"2200 100000.000000 6378137 0 0 0 0 0 1 5 1 0 0 0\n",
                encoding="utf-8",
            )
            truth = root / "truth.csv"
            truth.write_text(
                "UnixTimeMillis,LatitudeDegrees,LongitudeDegrees,AltitudeMeters\n"
                f"{timestamp},0,0,0\n",
                encoding="utf-8",
            )
            metrics = MODULE._score_position(position, device, truth, 1)
            self.assertEqual(metrics["device_epochs"], 1)
            self.assertEqual(metrics["position_epochs"], 1)
            self.assertEqual(metrics["truth_matched_epochs"], 1)
            self.assertIn("wgs84_vincenty__linear_n_minus_1", metrics["phone_score_variants_m"])
            self.assertIsNone(metrics["official_primary_score_m"])

    def test_canonicalization_rewrites_only_week_tow_for_one_ms_rounding(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_generalization_") as temp_dir:
            root = Path(temp_dir)
            device = root / "device.csv"
            # The second key deliberately uses the common Android ...999
            # representation while the POS line is rounded to the next ms.
            timestamp = int(315964800000 + 2200 * 604800000 + 100000000 - 18000)
            device.write_text(
                "MessageType,utcTimeMillis\n"
                f"Raw,{timestamp - 1000}\n"
                f"Raw,{timestamp - 1}\n",
                encoding="utf-8",
            )
            raw = root / "raw.pos"
            raw.write_text(
                "% fixture\n"
                "2200 100000.000000 1 2 3 0 0 0 1 5 1 0 0 0\n",
                encoding="ascii",
            )
            canonical = root / "canonical.pos"
            result = MODULE._canonicalize_position_timestamps(
                raw, device, canonical, 1
            )
            self.assertEqual(result["mapped_device_epochs"], 1)
            self.assertGreaterEqual(result["changed_timestamp_rows"], 1)
            self.assertIn("2200 99999.999", canonical.read_text(encoding="ascii"))


if __name__ == "__main__":
    unittest.main()
