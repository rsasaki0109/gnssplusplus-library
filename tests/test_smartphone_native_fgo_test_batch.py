from __future__ import annotations

import csv
import hashlib
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands"))
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_native_fgo_test_batch as BATCH  # noqa: E402


class NativeFgoTestBatchTests(unittest.TestCase):
    def _sample(self) -> dict[str, object]:
        return {
            "rows": [
                {
                    "trip_id": "route-a/pixel5",
                    "timestamp": 1000,
                    "timestamp_text": "1000",
                    "latitude": 34.0,
                    "longitude": -120.0,
                },
                {
                    "trip_id": "route-a/pixel5",
                    "timestamp": 2000,
                    "timestamp_text": "2000",
                    "latitude": 34.0,
                    "longitude": -120.0,
                },
            ]
        }

    def test_submission_preserves_exact_key_order_and_records_fallback(self) -> None:
        output, contract = BATCH._submission_bytes(
            self._sample(),
            {("route-a/pixel5", 1000): (35.0, -121.0)},
        )
        self.assertIn(b"tripId,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n", output)
        self.assertEqual(contract["exact_position_count"], 1)
        self.assertEqual(contract["official_sample_coordinate_fallback_count"], 1)
        with tempfile.TemporaryDirectory(prefix="native_fgo_test_batch_") as raw:
            path = Path(raw) / "submission.csv"
            path.write_bytes(output)
            verification = BATCH._verify_submission(path, self._sample())
        self.assertTrue(verification["exact_sample_order"])
        self.assertEqual(verification["row_count"], 2)

    def test_submission_rejects_key_order_change(self) -> None:
        sample = self._sample()
        output, _ = BATCH._submission_bytes(
            sample,
            {
                ("route-a/pixel5", 1000): (35.0, -121.0),
                ("route-a/pixel5", 2000): (35.1, -121.1),
            },
        )
        lines = output.decode("utf-8").splitlines()
        lines[1], lines[2] = lines[2], lines[1]
        with tempfile.TemporaryDirectory(prefix="native_fgo_test_batch_") as raw:
            path = Path(raw) / "submission.csv"
            path.write_text("\n".join(lines) + "\n", encoding="utf-8")
            with self.assertRaises(BATCH.TestBatchError):
                BATCH._verify_submission(path, sample)

    def test_cache_key_is_deterministic_and_content_addressed(self) -> None:
        record = {
            "dataset_id": "route-a/pixel5",
            "route": "route-a",
            "phone": "pixel5",
            "central_directory_files": {"device_gnss.csv": {"name": "d", "file_size": 1}},
            "central_directory_broadcast_nav": {"name": "n", "file_size": 2},
        }
        with tempfile.TemporaryDirectory(prefix="native_fgo_test_batch_") as raw:
            root = Path(raw)
            fgo = root / "fgo"
            spp = root / "spp"
            fgo.write_bytes(b"fgo")
            spp.write_bytes(b"spp")
            first = BATCH._cache_key(record, "archive", "inventory", "authorization", fgo, spp)
            second = BATCH._cache_key(record, "archive", "inventory", "authorization", fgo, spp)
        self.assertEqual(first, second)
        self.assertEqual(len(first), hashlib.sha256(b"").digest_size * 2)

    def test_source_hash_value_accepts_manifest_object_only(self) -> None:
        self.assertEqual(BATCH._source_hash_value({"sha256": "abc"}), "abc")
        self.assertEqual(BATCH._source_hash_value("abc"), "abc")
        self.assertIsNone(BATCH._source_hash_value({"hash": "abc"}))

    def test_official_header_constant_is_exact(self) -> None:
        self.assertEqual(
            BATCH.SAMPLE_FIELDS,
            ("tripId", "UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees"),
        )
        self.assertEqual(BATCH.FGO_RECIPE_HASH, "4633bfd3a86cf34ebd86820ed59ee7192b3cbf23fc75ce9e72fc1f2c88fb39f6")


if __name__ == "__main__":
    unittest.main()
