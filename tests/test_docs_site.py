#!/usr/bin/env python3
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]


class DocsSiteTest(unittest.TestCase):
    def test_mkdocs_builds_strict_site(self) -> None:
        mkdocs = shutil.which("mkdocs")
        self.assertIsNotNone(mkdocs, "mkdocs command is required for docs build")
        site_dir = Path(tempfile.mkdtemp(prefix="gnsspp_mkdocs_site_"))
        try:
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
        finally:
            shutil.rmtree(site_dir, ignore_errors=True)


if __name__ == "__main__":
    unittest.main()
