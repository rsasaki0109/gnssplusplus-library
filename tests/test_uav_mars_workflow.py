from __future__ import annotations

import hashlib
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "apps" / "commands" / "benchmarks" / "gnss_uav_mars_workflow.py"
sys.path.insert(0, str(ROOT / "apps" / "commands"))
SPEC = importlib.util.spec_from_file_location("gnss_uav_mars_workflow", MODULE_PATH)
assert SPEC and SPEC.loader
WORKFLOW = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(WORKFLOW)


class UavMarsWorkflowTests(unittest.TestCase):
    def test_navigation_product_requires_frozen_bytes_and_hash(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "brdc.rnx"
            path.write_bytes(b"frozen navigation")
            profile = {
                "navigation_products": {
                    "development": {
                        "uncompressed_bytes": path.stat().st_size,
                        "uncompressed_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                        "date_utc": "2023-10-24",
                        "provider": "fixture",
                    }
                }
            }
            result = WORKFLOW.validate_navigation(path, profile, "development")
            self.assertEqual(result["sha256"], hashlib.sha256(path.read_bytes()).hexdigest())

    def test_navigation_product_rejects_hash_mismatch(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "brdc.rnx"
            path.write_bytes(b"changed navigation")
            profile = {
                "navigation_products": {
                    "holdout": {
                        "uncompressed_bytes": path.stat().st_size,
                        "uncompressed_sha256": "0" * 64,
                    }
                }
            }
            with self.assertRaises(SystemExit) as raised:
                WORKFLOW.validate_navigation(path, profile, "holdout")
            self.assertIn("SHA-256", str(raised.exception))


if __name__ == "__main__":
    unittest.main()
