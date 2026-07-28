"""Regression tests for the concise and advanced gnss_fuse help surfaces."""

from __future__ import annotations

import os
import subprocess
import unittest
from pathlib import Path


class GnssFuseCliTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        binary_dir = Path(os.environ["GNSSPP_BINARY_DIR"])
        executable_name = "gnss_fuse.exe" if os.name == "nt" else "gnss_fuse"
        candidates = [
            binary_dir / "apps" / executable_name,
            *(binary_dir / "apps" / config / executable_name
              for config in ("Release", "RelWithDebInfo", "Debug")),
        ]
        cls.executable = next(
            (candidate for candidate in candidates if candidate.is_file()),
            None,
        )
        if cls.executable is None:
            rendered = ", ".join(str(candidate) for candidate in candidates)
            raise RuntimeError(f"gnss_fuse executable not found; checked: {rendered}")

    @classmethod
    def run_fuse(cls, option: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [str(cls.executable), option],
            check=False,
            capture_output=True,
            text=True,
        )

    def test_default_help_keeps_the_supported_path_concise(self) -> None:
        result = self.run_fuse("--help")

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("--data-dir <dir>", result.stdout)
        self.assertIn("--navi776-tc", result.stdout)
        self.assertIn("--help-advanced", result.stdout)
        self.assertNotIn("--tc-doppler-sigma", result.stdout)
        self.assertNotIn("--rtk-ins-prior-inflation", result.stdout)
        self.assertLess(len(result.stdout), 5000)

    def test_advanced_help_retains_research_option_discoverability(self) -> None:
        result = self.run_fuse("--help-advanced")

        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("--navi776-tc", result.stdout)
        self.assertIn("--tc-doppler-sigma", result.stdout)
        self.assertIn("--rtk-ins-prior-inflation", result.stdout)
        self.assertIn("--help-advanced", result.stdout)


if __name__ == "__main__":
    unittest.main()
