#!/usr/bin/env python3
"""Tests for --exclude-known-bad-madoca-sats CLI preset on gnss_ppp."""

from __future__ import annotations

import os
import subprocess
import sys
import tempfile
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


class ExcludeKnownBadMadocaPresetTests(unittest.TestCase):
    def test_help_lists_preset_flag(self) -> None:
        result = run_gnss("ppp", "--help")
        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("--exclude-known-bad-madoca-sats", result.stdout)
        self.assertIn("mizu-2025-04", result.stdout)

    def test_rejects_unknown_preset(self) -> None:
        result = run_gnss(
            "ppp",
            "--obs", "/nonexistent.obs",
            "--nav", "/nonexistent.nav",
            "--out", "/tmp/unused.pos",
            "--exclude-known-bad-madoca-sats", "not-a-real-preset",
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("unknown --exclude-known-bad-madoca-sats preset", result.stderr)
        self.assertIn("mizu-2025-04", result.stderr)

    def test_accepts_known_preset(self) -> None:
        # The parser must accept the documented preset without error. Missing
        # observation files fail later in main(), but argument validation
        # (where preset expansion happens) must succeed first.
        result = run_gnss(
            "ppp",
            "--obs", "/nonexistent.obs",
            "--nav", "/nonexistent.nav",
            "--out", "/tmp/unused.pos",
            "--exclude-known-bad-madoca-sats", "mizu-2025-04",
        )
        # Expect non-zero from missing obs file, but NOT a preset-related
        # argument-validation error.
        self.assertNotIn(
            "unknown --exclude-known-bad-madoca-sats preset",
            result.stderr,
        )
        self.assertNotIn("mizu-2025-04'", result.stderr)


if __name__ == "__main__":
    unittest.main()
