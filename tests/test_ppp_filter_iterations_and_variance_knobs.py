#!/usr/bin/env python3
"""Tests for --filter-iterations / --initial-*-variance CLI knobs on gnss_ppp."""

from __future__ import annotations

import os
import subprocess
import sys
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
DISPATCHER = ROOT_DIR / "apps" / "gnss.py"


def run_gnss(*args: str) -> subprocess.CompletedProcess[str]:
    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    return subprocess.run(
        [sys.executable, str(DISPATCHER), *args],
        cwd=ROOT_DIR,
        env=env,
        text=True,
        capture_output=True,
        check=False,
    )


class FilterIterationsKnobTests(unittest.TestCase):
    def test_accepts_positive_value(self) -> None:
        result = run_gnss(
            "ppp",
            "--obs", "/nonexistent.obs",
            "--nav", "/nonexistent.nav",
            "--out", "/tmp/unused.pos",
            "--filter-iterations", "8",
        )
        # Missing obs fails later; argument validation must not reject the knob.
        self.assertNotIn("--filter-iterations must be non-negative", result.stderr)

    def test_rejects_negative_value(self) -> None:
        result = run_gnss(
            "ppp",
            "--obs", "/nonexistent.obs",
            "--nav", "/nonexistent.nav",
            "--out", "/tmp/unused.pos",
            "--filter-iterations", "-1",
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("--filter-iterations must be non-negative", result.stderr)


class InitialVarianceKnobTests(unittest.TestCase):
    def test_accepts_positive_iono_variance(self) -> None:
        result = run_gnss(
            "ppp",
            "--obs", "/nonexistent.obs",
            "--nav", "/nonexistent.nav",
            "--out", "/tmp/unused.pos",
            "--initial-ionosphere-variance", "3600",
        )
        self.assertNotIn("--initial-ionosphere-variance must be positive", result.stderr)

    def test_rejects_zero_iono_variance(self) -> None:
        result = run_gnss(
            "ppp",
            "--obs", "/nonexistent.obs",
            "--nav", "/nonexistent.nav",
            "--out", "/tmp/unused.pos",
            "--initial-ionosphere-variance", "0",
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("--initial-ionosphere-variance must be positive", result.stderr)

    def test_accepts_positive_trop_variance(self) -> None:
        result = run_gnss(
            "ppp",
            "--obs", "/nonexistent.obs",
            "--nav", "/nonexistent.nav",
            "--out", "/tmp/unused.pos",
            "--initial-troposphere-variance", "0.05",
        )
        self.assertNotIn("--initial-troposphere-variance must be positive", result.stderr)

    def test_rejects_zero_trop_variance(self) -> None:
        result = run_gnss(
            "ppp",
            "--obs", "/nonexistent.obs",
            "--nav", "/nonexistent.nav",
            "--out", "/tmp/unused.pos",
            "--initial-troposphere-variance", "0",
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("--initial-troposphere-variance must be positive", result.stderr)


if __name__ == "__main__":
    unittest.main()
