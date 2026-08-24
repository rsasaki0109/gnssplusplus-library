from __future__ import annotations

import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
DISPATCHER = ROOT / "apps" / "gnss.py"


class UavMarsAcquireTests(unittest.TestCase):
    def run_cli(self, *args: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [sys.executable, str(DISPATCHER), "uav-mars-acquire", *args],
            cwd=ROOT,
            text=True,
            capture_output=True,
            check=False,
        )

    @staticmethod
    def write_profile(
        path: Path, bag: bytes, *, frozen: bool = True, container: str = "ros1_bag"
    ) -> None:
        digest = hashlib.sha256(bag).hexdigest() if frozen else None
        payload = {
            "schema_version": "uav-r6-profile.v1",
            "datasets": {
                "development": {
                    "id": "fixture-flight",
                    "filename": "fixture.bag",
                    "container": container,
                    "expected_bytes": len(bag),
                    "sha256": digest,
                    "urls": ["https://invalid.example/fixture.bag"],
                    "source_page": "https://invalid.example/dataset",
                    "official_file_id": "fixture-id",
                    "licence_status": "fixture",
                    "redistribution": "fixture only",
                }
            },
            "frame_contract": {"vehicle_nhc": False},
            "time_contract": {"comparison_time": "UTC"},
        }
        path.write_text(json.dumps(payload), encoding="utf-8")

    def test_validates_frozen_existing_bag_and_writes_summary(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            work = Path(directory)
            bag = b"#ROSBAG V2.0\n" + bytes(range(64))
            bag_path = work / "fixture.bag"
            bag_path.write_bytes(bag)
            profile = work / "profile.json"
            self.write_profile(profile, bag)
            output = work / "output"
            result = self.run_cli(
                "--profile", str(profile),
                "--role", "development",
                "--input", str(bag_path),
                "--output-dir", str(output),
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            summary = json.loads((output / "acquire_summary.json").read_text())
            self.assertEqual(summary["schema_version"], "uav-mars-acquire.v1")
            self.assertEqual(summary["bag"]["sha256"], hashlib.sha256(bag).hexdigest())
            self.assertFalse(summary["frame_contract"]["vehicle_nhc"])

    def test_rejects_quota_page_even_when_size_and_hash_are_frozen(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            work = Path(directory)
            page = b"<html>Google Drive - Quota exceeded</html>" * 2
            input_path = work / "quota.bag"
            input_path.write_bytes(page)
            profile = work / "profile.json"
            self.write_profile(profile, page)
            result = self.run_cli(
                "--profile", str(profile),
                "--role", "development",
                "--input", str(input_path),
                "--output-dir", str(work / "output"),
            )
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("quota/login/error pages are rejected", result.stderr)

    def test_validates_frozen_mcap_container(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            work = Path(directory)
            data = b"\x89MCAP0\r\n" + bytes(range(64))
            input_path = work / "fixture.mcap"
            input_path.write_bytes(data)
            profile = work / "profile.json"
            self.write_profile(profile, data, container="mcap")
            result = self.run_cli(
                "--profile", str(profile), "--role", "development",
                "--input", str(input_path), "--output-dir", str(work / "output"),
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            summary = json.loads((work / "output" / "acquire_summary.json").read_text())
            self.assertEqual(summary["bag"]["container"], "mcap")

    def test_rejects_unfrozen_dataset_before_network_access(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            work = Path(directory)
            bag = b"#ROSBAG V2.0\n" + bytes(range(64))
            input_path = work / "fixture.bag"
            input_path.write_bytes(bag)
            profile = work / "profile.json"
            self.write_profile(profile, bag, frozen=False)
            result = self.run_cli(
                "--profile", str(profile),
                "--role", "development",
                "--input", str(input_path),
                "--output-dir", str(work / "output"),
            )
            self.assertNotEqual(result.returncode, 0)
            self.assertIn("not frozen: missing sha256", result.stderr)


if __name__ == "__main__":
    unittest.main()
