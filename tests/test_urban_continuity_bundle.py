"""Contract tests for the one-command urban continuity artifact bundle."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock


ROOT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT_DIR / "apps" / "commands" / "benchmarks"))

import gnss_urban_continuity_bundle as bundle  # noqa: E402


class UrbanContinuityBundleTest(unittest.TestCase):
    def test_frozen_command_contains_contract_and_gates(self) -> None:
        with tempfile.TemporaryDirectory(prefix="urban_bundle_commands_") as temp_dir:
            root = Path(temp_dir)
            commands = bundle.build_step_commands(
                root / "data",
                root / "data" / "reference.csv",
                root / "bundle",
                [0.31, 0.0, -0.55],
                "low-cost",
                True,
                25.0,
                3,
                20.0,
                0,
            )

            fuse = commands["fuse"]
            self.assertIn("--lever-arm", fuse)
            self.assertIn("0.31,0,-0.55", fuse)
            self.assertIn("--zupt", fuse)
            self.assertIn("--no-nhc", fuse)
            self.assertIn("--max-velocity-nis", fuse)
            self.assertIn("25", fuse)
            self.assertIn("--max-consecutive-velocity-gate-rejections", fuse)
            self.assertIn("3", fuse)
            self.assertIn("--max-gnss-velocity-reanchor-mps", fuse)
            self.assertIn("20", fuse)
            self.assertIn("--max-position-nis", fuse)
            self.assertIn("--max-consecutive-gate-rejections", fuse)

            score = commands["score"]
            for option in (
                "--require-fused-bridge-coverage-min",
                "--require-max-bridge-error-max",
                "--require-max-reacquisition-jump-max",
                "--require-fixed-p95-regression-max",
                "--require-fused-availability-at-least-rtk",
                "--require-no-nonfinite",
            ):
                self.assertIn(option, score)
            self.assertEqual(commands["png"][2:], ["trackplot", str(root / "bundle" / "fused.pos")])

    def test_failed_run_keeps_manifest_and_log(self) -> None:
        with tempfile.TemporaryDirectory(prefix="urban_bundle_failure_") as temp_dir:
            root = Path(temp_dir)
            data_dir = root / "data"
            data_dir.mkdir()
            (data_dir / "reference.csv").write_text("reference\n", encoding="utf-8")
            output_dir = root / "bundle"
            args = bundle.parse_args(
                ["--data-dir", str(data_dir), "--output-dir", str(output_dir)]
            )

            def fail_step(name: str, argv: list[str], log_path: Path) -> dict[str, object]:
                return {"name": name, "argv": argv, "exit_status": 7, "status": "failed"}

            with mock.patch.object(bundle, "run_step", side_effect=fail_step):
                result = bundle.run_bundle(args)

            self.assertEqual(result, 1)
            manifest_path = output_dir / "manifest.json"
            self.assertTrue(manifest_path.is_file())
            self.assertTrue((output_dir / "bundle.log").is_file())
            payload = json.loads(manifest_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["status"], "failed")
            self.assertEqual(payload["exit_status"], 1)
            self.assertEqual(payload["steps"][0]["exit_status"], 7)
            self.assertEqual([step["status"] for step in payload["steps"]], [
                "failed", "skipped", "skipped", "skipped"
            ])

    def test_success_records_hashes_score_and_age(self) -> None:
        with tempfile.TemporaryDirectory(prefix="urban_bundle_success_") as temp_dir:
            root = Path(temp_dir)
            data_dir = root / "data"
            data_dir.mkdir()
            (data_dir / "rover.obs").write_text("rover\n", encoding="utf-8")
            (data_dir / "reference.csv").write_text("reference\n", encoding="utf-8")
            output_dir = root / "bundle"
            args = bundle.parse_args(
                ["--data-dir", str(data_dir), "--output-dir", str(output_dir)]
            )

            def fake_step(name: str, argv: list[str], log_path: Path) -> dict[str, object]:
                if name == "fuse":
                    Path(argv[argv.index("--rtk-pos-out") + 1]).write_text("rtk\n", encoding="utf-8")
                    Path(argv[argv.index("--out") + 1]).write_text("fused\n", encoding="utf-8")
                elif name == "kml":
                    Path(argv[4]).write_text("<kml/>\n", encoding="utf-8")
                elif name == "png":
                    fused_path = Path(argv[3])
                    fused_path.with_name(fused_path.stem + "_trajectory.png").write_bytes(
                        b"PNG fixture"
                    )
                elif name == "score":
                    summary_path = Path(argv[argv.index("--summary-json") + 1])
                    segments_path = Path(argv[argv.index("--segments-csv") + 1])
                    summary_path.write_text(
                        json.dumps(
                            {
                                "aggregate": {
                                    "fused_availability_pct": 99.7,
                                    "rtk_solution_availability_pct": 99.1,
                                },
                                "gate": {"status": "passed", "failures": []},
                                "segments": [{"propagation_age_s": 87.2}],
                            }
                        ),
                        encoding="utf-8",
                    )
                    segments_path.write_text("propagation_age_s\n87.2\n", encoding="utf-8")
                return {"name": name, "argv": argv, "exit_status": 0, "status": "passed"}

            with mock.patch.object(bundle, "run_step", side_effect=fake_step):
                result = bundle.run_bundle(args)

            self.assertEqual(result, 0)
            manifest_path = output_dir / "manifest.json"
            payload = json.loads(manifest_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["status"], "passed")
            self.assertEqual(payload["score"]["gate"]["status"], "passed")
            self.assertEqual(payload["validated_max_propagation_age_s"], 87.2)
            self.assertTrue(payload["operational_max_bridge_age_exceeded"])
            self.assertEqual(len(payload["steps"]), 4)
            for name in ("rtk_pos", "fused_pos", "kml", "png", "score_summary", "segments", "log"):
                record = payload["artifacts"][name]
                self.assertTrue(record["exists"], name)
                self.assertGreater(record["bytes"], 0, name)
                self.assertEqual(len(record["sha256"]), 64, name)

            fused_path = output_dir / "fused.pos"
            self.assertEqual(
                payload["artifacts"]["fused_pos"]["sha256"],
                hashlib.sha256(fused_path.read_bytes()).hexdigest(),
            )
            self.assertIn(payload["binary"]["version_available"], (True, False))
            self.assertEqual(payload["manifest_path"], str(manifest_path))
            self.assertEqual(payload["dataset_provenance"]["dataset"], "PPC-Dataset")
            self.assertIn("upstream", payload["dataset_provenance"])
            self.assertIn("license", payload["dataset_provenance"])
            self.assertIn("git_head_available", payload["software_revision"])
            self.assertIn("changed_files", payload["software_revision"])
            dispatcher_argv = payload["invocation"]["dispatcher_argv"]
            self.assertIn("urban-continuity-bundle", dispatcher_argv)
            self.assertIn("--data-dir", dispatcher_argv)


if __name__ == "__main__":
    unittest.main()
