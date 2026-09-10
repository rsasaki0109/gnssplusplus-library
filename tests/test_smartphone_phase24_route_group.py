from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock
import zipfile
import zlib
import sys


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase24_route_group.py"
SPEC = importlib.util.spec_from_file_location("phase24_route_group", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def _digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class Phase24RouteGroupTests(unittest.TestCase):
    def _freeze(self, archive_sha: str, metadata: dict[str, dict[str, object]]) -> dict[str, object]:
        train = [
            {
                "dataset_id": "2021-01-01-00-00-us-ca-a/pixel5",
                "route": "2021-01-01-00-00-us-ca-a",
                "phone": "pixel5",
            },
            {
                "dataset_id": "2021-01-02-00-00-us-ca-b/pixel4",
                "route": "2021-01-02-00-00-us-ca-b",
                "phone": "pixel4",
            },
        ]
        return {
            "schema_version": "smartphone-r5-phase24-route-group-split-freeze.v1",
            "scope": {
                "archive_sha256": archive_sha,
                "mat_policy": "No .mat member is fetched, opened, generated, or used.",
            },
            "frozen_split": {
                "train_calibration": train,
                "fresh_validation": {"dataset_id": "2021-01-03-00-00-us-ca-c/pixel5"},
                "future_holdout": {"dataset_id": "2021-01-04-00-00-us-ca-d/pixel5"},
            },
            "selected_central_directory_metadata": metadata,
        }

    def _fixture_archive(self, root: Path) -> tuple[Path, dict[str, dict[str, object]]]:
        archive_path = root / "dataset.zip"
        metadata: dict[str, dict[str, object]] = {}
        payloads = {
            "2021-01-01-00-00-us-ca-a/pixel5": b"gnss-a\n",
            "2021-01-02-00-00-us-ca-b/pixel4": b"gnss-b\n",
        }
        with zipfile.ZipFile(archive_path, "w", compression=zipfile.ZIP_DEFLATED) as archive:
            for dataset_id, gnss in payloads.items():
                route, phone = dataset_id.split("/", 1)
                members = {
                    "device_gnss": (f"dataset_2023/train/{route}/{phone}/device_gnss.csv", gnss),
                    "device_imu": (f"dataset_2023/train/{route}/{phone}/device_imu.csv", b"imu\n"),
                    "broadcast_nav": (f"dataset_2023/train/{route}/brdc.nav", b"nav\n"),
                }
                metadata[dataset_id] = {}
                for key, (name, data) in members.items():
                    archive.writestr(name, data)
                    metadata[dataset_id][key] = {
                        "name": name,
                        "file_size": len(data),
                    "crc32_hex": f"{zlib.crc32(data) & 0xffffffff:08x}",
                    }
            # Metadata-only presence of a forbidden member must not cause an open.
            archive.writestr("dataset_2023/train/forbidden/result_gnss.mat", b"poison")
        return archive_path, metadata

    def test_load_freeze_rejects_role_route_reuse(self) -> None:
        with tempfile.TemporaryDirectory(prefix="phase24-freeze-") as temp:
            path = Path(temp) / "freeze.json"
            payload = self._freeze("0" * 64, {})
            payload["frozen_split"]["fresh_validation"] = {  # type: ignore[index]
                "dataset_id": "2021-01-02-00-00-us-ca-b/pixel5"
            }
            path.write_text(json.dumps(payload), encoding="utf-8")
            with self.assertRaises(MODULE.Phase24Error):
                MODULE.load_freeze(path)

    def test_materialize_opens_only_allowlisted_raw_members(self) -> None:
        with tempfile.TemporaryDirectory(prefix="phase24-materialize-") as temp:
            root = Path(temp)
            archive_path, metadata = self._fixture_archive(root)
            archive_sha = _digest(archive_path)
            freeze = self._freeze(archive_sha, metadata)
            freeze_path = root / "freeze.json"
            freeze_path.write_text(json.dumps(freeze), encoding="utf-8")
            output = root / "out"
            original_open = zipfile.ZipFile.open

            test_case = self

            def guarded_open(archive, name, *args, **kwargs):  # type: ignore[no-untyped-def]
                member_name = name.filename if isinstance(name, zipfile.ZipInfo) else name
                test_case.assertFalse(str(member_name).lower().endswith(".mat"))
                return original_open(archive, name, *args, **kwargs)

            with mock.patch.object(zipfile.ZipFile, "open", new=guarded_open):
                report = MODULE.materialize_train(freeze_path, archive_path, output)
            self.assertEqual(report["truth_open_count"], 0)
            self.assertFalse(report["mat_member_opened"])
            for dataset_id in metadata:
                route, phone = dataset_id.split("/", 1)
                input_dir = output / "raw" / route / phone
                self.assertEqual(
                    {path.name for path in input_dir.iterdir()},
                    {"device_gnss.csv", "device_imu.csv", "brdc.nav", "materialization_manifest.json"},
                )
                manifest = json.loads((input_dir / "materialization_manifest.json").read_text())
                self.assertEqual(manifest["members_opened"], ["broadcast_nav", "device_gnss", "device_imu"])
                self.assertFalse(manifest["truth_materialized"])
                self.assertEqual(manifest["truth_open_count"], 0)

    def test_holdout_is_fail_closed_before_archive_access(self) -> None:
        with tempfile.TemporaryDirectory(prefix="phase24-holdout-") as temp:
            root = Path(temp)
            freeze = self._freeze("0" * 64, {})
            freeze_path = root / "freeze.json"
            freeze_path.write_text(json.dumps(freeze), encoding="utf-8")
            with mock.patch.object(zipfile.ZipFile, "open", side_effect=AssertionError("archive opened")):
                with self.assertRaises(MODULE.Phase24Error):
                    MODULE.selected_routes(freeze, "holdout")

    def test_candidate_matrix_is_fixed_and_raw_only(self) -> None:
        self.assertEqual(
            MODULE.CANDIDATE_FLAGS["phase12_control"],
            (
                "--native-pdc-imu-tdcp",
                "--native-signal-bias-states",
                "--native-residual-ionosphere",
            ),
        )
        self.assertNotIn(".mat", " ".join(sum(MODULE.CANDIDATE_FLAGS.values(), ())).lower())
        self.assertEqual(
            MODULE.RAW_CLOCK_FLAGS,
            ("--android-raw-clock-only", "--android-utc-wall-clock-fallback"),
        )


if __name__ == "__main__":
    unittest.main()
