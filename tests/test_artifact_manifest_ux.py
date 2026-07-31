#!/usr/bin/env python3
"""Lightweight artifact-manifest user-experience smoke tests."""

from __future__ import annotations

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
GNSS_CLI = ROOT_DIR / "apps" / "gnss.py"


class ArtifactManifestUxTest(unittest.TestCase):
    def run_gnss(self, *args: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [sys.executable, str(GNSS_CLI), *args],
            cwd=ROOT_DIR,
            check=False,
            capture_output=True,
            text=True,
        )

    def assert_no_traceback(self, result: subprocess.CompletedProcess[str]) -> None:
        combined = result.stdout + result.stderr
        self.assertNotIn("Traceback (most recent call last)", combined)
        self.assertNotIn("ModuleNotFoundError", combined)

    def test_help_uses_dispatcher_name(self) -> None:
        result = self.run_gnss("artifact-manifest", "--help")

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assert_no_traceback(result)
        self.assertIn("usage: gnss artifact-manifest", result.stdout)
        self.assertIn("--root", result.stdout)
        self.assertIn("--output", result.stdout)

    def test_manifest_builds_web_ready_bundle_index(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_artifact_manifest_") as temp_dir:
            root = Path(temp_dir)
            output = root / "output"
            output.mkdir()
            ppc_solution = output / "ppc_tokyo_run1.pos"
            ppc_reference = output / "ppc_tokyo_run1_reference.pos"
            visibility_csv = output / "visibility_static.csv"
            visibility_png = output / "visibility_static.png"
            ppc_solution.write_text("% synthetic solution\n", encoding="ascii")
            ppc_reference.write_text("% synthetic reference\n", encoding="ascii")
            visibility_csv.write_text("satellite,system\nG01,GPS\n", encoding="ascii")
            visibility_png.write_bytes(b"not-a-real-png")

            (output / "ppc_tokyo_run1_summary.json").write_text(
                json.dumps(
                    {
                        "matched_epochs": 200,
                        "fix_rate_pct": 98.0,
                        "median_h_m": 0.044,
                        "p95_h_m": 0.114,
                        "solver_wall_time_s": 35.0,
                        "realtime_factor": 1.13,
                        "effective_epoch_rate_hz": 5.7,
                        "solution_pos": str(ppc_solution),
                        "reference_pos": str(ppc_reference.relative_to(root)),
                    }
                ),
                encoding="utf-8",
            )
            (output / "visibility_static_summary.json").write_text(
                json.dumps(
                    {
                        "epochs_processed": 30,
                        "epochs_with_rows": 30,
                        "rows_written": 120,
                        "unique_satellites": 12,
                        "mean_satellites_per_epoch": 4.0,
                        "mean_elevation_deg": 42.5,
                        "csv": str(visibility_csv),
                        "png": str(visibility_png.relative_to(root)),
                    }
                ),
                encoding="utf-8",
            )
            (output / "ppc_broken_summary.json").write_text("{not json", encoding="utf-8")
            manifest_path = output / "artifact_manifest.json"

            result = self.run_gnss(
                "artifact-manifest",
                "--root",
                str(root),
                "--output",
                str(manifest_path),
                "--quiet",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assert_no_traceback(result)
            self.assertEqual(result.stdout, "")
            self.assertEqual(result.stderr, "")
            payload = json.loads(manifest_path.read_text(encoding="utf-8"))

        self.assertEqual(payload["schema_version"], 1)
        self.assertEqual(payload["contract"], "artifact_manifest.v1")
        self.assertEqual(payload["bundle_count"], 2)
        self.assertEqual(payload["root"], str(root.resolve()))
        bundles = {bundle["category"]: bundle for bundle in payload["bundles"]}
        self.assertEqual(set(bundles), {"ppc", "visibility"})

        ppc = bundles["ppc"]
        self.assertEqual(ppc["status"], "excellent")
        self.assertEqual(ppc["runtime_status"], "realtime")
        self.assertEqual(ppc["artifacts"]["solution"], "output/ppc_tokyo_run1.pos")
        self.assertEqual(ppc["artifacts"]["reference"], "output/ppc_tokyo_run1_reference.pos")
        self.assertEqual(ppc["metrics"]["matched_epochs"], 200)
        self.assertIn("200 matched", ppc["headline"])
        self.assertNotIn(str(root), json.dumps(ppc))

        visibility = bundles["visibility"]
        self.assertEqual(visibility["status"], "available")
        self.assertEqual(visibility["artifacts"]["csv"], "output/visibility_static.csv")
        self.assertEqual(visibility["artifacts"]["png"], "output/visibility_static.png")
        self.assertEqual(visibility["metrics"]["unique_satellites"], 12)


if __name__ == "__main__":
    unittest.main()
