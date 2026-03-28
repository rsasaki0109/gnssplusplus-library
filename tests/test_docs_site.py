#!/usr/bin/env python3
import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
ARCHITECTURE_PNG = ROOT_DIR / "docs" / "libgnsspp_architecture.png"


class DocsSiteTest(unittest.TestCase):
    def test_mkdocs_builds_strict_site(self) -> None:
        mkdocs = shutil.which("mkdocs")
        self.assertIsNotNone(mkdocs, "mkdocs command is required for docs build")
        site_dir = Path(tempfile.mkdtemp(prefix="gnsspp_mkdocs_site_"))
        try:
            subprocess.run(
                [
                    shutil.which("python3") or "python3",
                    str(ROOT_DIR / "scripts" / "generate_architecture_diagram.py"),
                    "--output",
                    str(ARCHITECTURE_PNG),
                ],
                cwd=ROOT_DIR,
                check=True,
                env={**os.environ, "MPLBACKEND": "Agg"},
            )
            self.assertTrue(ARCHITECTURE_PNG.exists())
            subprocess.run(
                [
                    mkdocs,
                    "build",
                    "--strict",
                    "--site-dir",
                    str(site_dir),
                ],
                cwd=ROOT_DIR,
                check=True,
            )
            self.assertTrue((site_dir / "index.html").exists())
            self.assertTrue((site_dir / "architecture" / "index.html").exists() or (site_dir / "architecture.html").exists())
            self.assertTrue((site_dir / "libgnsspp_architecture.png").exists())
        finally:
            shutil.rmtree(site_dir, ignore_errors=True)


if __name__ == "__main__":
    unittest.main()
