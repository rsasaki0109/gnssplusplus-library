"""Provenance and fail-closed checks for raw-input native PPC regeneration."""
from __future__ import annotations

import copy
import json
import os
from pathlib import Path
import subprocess
import sys
import tarfile
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps/commands"))
sys.path.insert(0, str(ROOT / "apps/commands/benchmarks"))
import gnss_ppc_native_replay as replay


class NativeReplayTest(unittest.TestCase):
    def test_output_refuses_prior_results_without_deleting_them(self):
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory)
            (out / "solution.pos").write_text("old solution")
            with self.assertRaisesRegex(ValueError, "new or empty"):
                replay.prepare_output(out)
            self.assertEqual((out / "solution.pos").read_text(), "old solution")

    def test_build_cache_cannot_claim_another_source_tree(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "CMakeCache.txt").write_text("CMAKE_HOME_DIRECTORY:INTERNAL=/another/tree\n")
            with self.assertRaisesRegex(ValueError, "this source tree"):
                replay.read_cache(root, ROOT)

    def test_snapshot_output_must_be_ignored_or_external(self):
        replay.check_output_location(ROOT / "output/native-replay", ROOT)
        replay.check_output_location(ROOT.parent / "external-native-replay", ROOT)
        with self.assertRaisesRegex(ValueError, "Git-ignored"):
            replay.check_output_location(ROOT / "source-contaminating-results", ROOT)

    def test_archive_reconstructs_dirty_and_untracked_source(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory) / "repo"
            root.mkdir()
            def git(*args):
                subprocess.run(["git", "-C", str(root), *args], check=True, capture_output=True)
            git("init")
            git("config", "user.name", "Replay test")
            git("config", "user.email", "replay@example.invalid")
            (root / "source.cpp").write_text("original")
            (root / "deleted.cpp").write_text("original")
            (root / ".gitignore").write_text("output/\n")
            git("add", ".")
            git("commit", "-m", "fixture")
            (root / "source.cpp").write_text("edited source")
            (root / "deleted.cpp").unlink()
            (root / "new.cpp").write_text("new source")
            (root / "output").mkdir()
            (root / "output/ignored.cpp").write_text("not build input")
            snapshot = replay.source_snapshot(root)
            self.assertIsNone(snapshot["files"]["deleted.cpp"])
            archive = Path(directory) / "source.tar.gz"
            replay.archive_source(root, snapshot, archive)
            with tarfile.open(archive) as handle:
                self.assertEqual(handle.extractfile("source.cpp").read(), b"edited source")
                self.assertEqual(handle.extractfile("new.cpp").read(), b"new source")
                self.assertNotIn("deleted.cpp", handle.getnames())
                self.assertNotIn("output/ignored.cpp", handle.getnames())
            old_digest = snapshot["contents_sha256"]
            (root / "new.cpp").write_text("later edit")
            self.assertNotEqual(replay.source_snapshot(root)["contents_sha256"], old_digest)

    def test_native_commands_do_not_read_truth_and_preserve_both_fusion_streams(self):
        binaries = {"gnss_solve": Path("/build/gnss_solve"), "gnss_fuse": Path("/build/gnss_fuse")}
        commands = replay.solver_commands(binaries, Path("/data/tokyo/run1"), "tokyo",
                                           Path("/out"), ["rtk", "fusion"], -1)
        for _, command, _ in commands:
            self.assertFalse(any("reference" in arg for arg in command))
            self.assertNotIn("--max-epochs", command)
        self.assertEqual([stream for stream, _ in commands[1][2]], ["fused", "coupled_rtk"])
        self.assertIn("--navi776-tc", commands[1][1])

    def test_data_digest_catches_status_or_position_change_not_output_header(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "solution.pos"
            path.write_text("% file A\n2324  1.0 1 2 3 4\n")
            first = replay.solution_data_sha256(path)
            path.write_text("% file B\n2324 1.0 1 2 3 4\n")
            self.assertEqual(replay.solution_data_sha256(path), first)
            path.write_text("2324 1.0 1 2 3 3\n")
            self.assertNotEqual(replay.solution_data_sha256(path), first)
            path.write_text("% header only\n")
            with self.assertRaisesRegex(ValueError, "no data"):
                replay.solution_data_sha256(path)

    def test_repeated_run_requires_matching_provenance_and_every_stream(self):
        baseline = {"state": "passed", "evaluation": "full", "max_epochs": -1,
                    "runs": ["tokyo/run1"], "paths": ["rtk"],
                    "source": {"contents_sha256": "source"}, "inputs": ["input"],
                    "binaries": {"solve": "binary"}, "runtime_libraries": [],
                    "results": [{"run": "tokyo/run1", "stream": "rtk", "solution_data_sha256": "data"}]}
        candidate = copy.deepcopy(baseline)
        self.assertTrue(replay.compare_reports(baseline, candidate)["passed"])
        candidate["results"][0]["solution_data_sha256"] = "changed"
        self.assertFalse(replay.compare_reports(baseline, candidate)["passed"])
        candidate["binaries"]["solve"] = "new binary"
        with self.assertRaisesRegex(ValueError, "binaries"):
            replay.compare_reports(baseline, candidate)
        candidate = copy.deepcopy(baseline)
        candidate["results"] = []
        with self.assertRaisesRegex(ValueError, "population"):
            replay.compare_reports(baseline, candidate)

    def test_failed_child_is_recorded_and_not_reported_as_pass(self):
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory)
            report = {"steps": []}
            with self.assertRaisesRegex(ValueError, "command failed"):
                replay.run_step([sys.executable, "-c", "raise SystemExit(7)"],
                                out / "solver.log", os.environ.copy(), report, out / "manifest.json")
            saved = json.loads((out / "manifest.json").read_text())
            self.assertEqual(saved["steps"][0]["exit_code"], 7)
            self.assertEqual(saved["steps"][0]["state"], "failed")

    def test_missing_executable_does_not_leave_a_phantom_running_step(self):
        with tempfile.TemporaryDirectory() as directory:
            out = Path(directory)
            with self.assertRaises(OSError):
                replay.run_step([str(out / "missing-executable")], out / "solver.log",
                                os.environ.copy(), {"steps": []}, out / "manifest.json")
            saved = json.loads((out / "manifest.json").read_text())
            self.assertEqual(saved["steps"][0]["state"], "failed")

    def test_default_is_full_and_duplicate_or_zero_cap_is_rejected(self):
        args = ["--dataset-root", "data", "--build-dir", "build", "--output-dir", "out"]
        self.assertEqual(replay.parse_args(args).max_epochs, -1)
        for extra in (["--max-epochs", "0"], ["--runs", "tokyo/run1", "tokyo/run1"]):
            with patch("sys.stderr"), self.assertRaises(SystemExit):
                replay.parse_args([*args, *extra])


if __name__ == "__main__":
    unittest.main()
