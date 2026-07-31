#!/usr/bin/env python3
"""Tests for the dashboard artifact manifest CI contract validator."""

from __future__ import annotations

import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
VALIDATOR_PATH = ROOT_DIR / "scripts" / "ci" / "validate_artifact_manifest_contract.py"
PNG_BYTES = b"\x89PNG\r\n\x1a\nsynthetic\n"


def load_validator_module():
    spec = importlib.util.spec_from_file_location("validate_artifact_manifest_contract", VALIDATOR_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed to load {VALIDATOR_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


validator = load_validator_module()


class ArtifactManifestContractValidatorTest(unittest.TestCase):
    def make_fixture(self, root: Path) -> Path:
        output = root / "output"
        output.mkdir()
        summary = output / "visibility_static_summary.json"
        csv_path = output / "visibility_static.csv"
        png_path = output / "visibility_static.png"
        manifest = output / "artifact_manifest.json"
        summary.write_text(
            json.dumps(
                {
                    "epochs_processed": 1,
                    "rows_written": 1,
                    "csv": "output/visibility_static.csv",
                    "png": "output/visibility_static.png",
                }
            )
            + "\n",
            encoding="utf-8",
        )
        csv_path.write_text("satellite,system\nG01,GPS\n", encoding="utf-8")
        png_path.write_bytes(PNG_BYTES)
        manifest.write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "contract": "artifact_manifest.v1",
                    "root": str(root.resolve()),
                    "generated_at_utc": "2026-06-20T00:00:00+00:00",
                    "bundle_count": 1,
                    "bundles": [
                        {
                            "category": "visibility",
                            "label": "visibility_static_summary.json",
                            "summary_json": "output/visibility_static_summary.json",
                            "status": "available",
                            "headline": "1 rows / 1 sats",
                            "artifacts": {
                                "summary": "output/visibility_static_summary.json",
                                "csv": "output/visibility_static.csv",
                                "png": "output/visibility_static.png",
                                "external": "https://example.com/report",
                                "runs": [{"csv": "output/visibility_static.csv"}],
                            },
                            "metrics": {"rows_written": 1},
                        }
                    ],
                },
                indent=2,
                sort_keys=True,
            )
            + "\n",
            encoding="utf-8",
        )
        return manifest

    def load_payload(self, manifest: Path) -> dict[str, object]:
        return json.loads(manifest.read_text(encoding="utf-8"))

    def test_valid_manifest_passes(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_artifact_contract_") as temp_dir:
            manifest = self.make_fixture(Path(temp_dir))
            validator.validate_manifest(self.load_payload(manifest))

    def test_missing_linked_artifact_fails(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_artifact_contract_") as temp_dir:
            root = Path(temp_dir)
            manifest = self.make_fixture(root)
            (root / "output" / "visibility_static.csv").unlink()

            with self.assertRaisesRegex(AssertionError, "missing artifact file"):
                validator.validate_manifest(self.load_payload(manifest))

    def test_path_escape_fails(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_artifact_contract_") as temp_dir:
            manifest = self.make_fixture(Path(temp_dir))
            payload = self.load_payload(manifest)
            bundle = payload["bundles"][0]
            bundle["artifacts"]["csv"] = "../outside.csv"

            with self.assertRaisesRegex(AssertionError, "escapes manifest root"):
                validator.validate_manifest(payload)

    def test_invalid_png_signature_fails(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_artifact_contract_") as temp_dir:
            root = Path(temp_dir)
            manifest = self.make_fixture(root)
            (root / "output" / "visibility_static.png").write_bytes(b"not-a-png")

            with self.assertRaisesRegex(AssertionError, "PNG signature is invalid"):
                validator.validate_manifest(self.load_payload(manifest))

    def test_bundle_count_mismatch_fails(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_artifact_contract_") as temp_dir:
            manifest = self.make_fixture(Path(temp_dir))
            payload = self.load_payload(manifest)
            payload["bundle_count"] = 2

            with self.assertRaisesRegex(AssertionError, "bundle_count"):
                validator.validate_manifest(payload)


if __name__ == "__main__":
    unittest.main()
