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
    def _doc_exists(self, site_dir: Path, relative_stem: str) -> bool:
        return (
            (site_dir / relative_stem / "index.html").exists()
            or (site_dir / f"{relative_stem}.html").exists()
        )

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
            self.assertTrue(self._doc_exists(site_dir, "architecture"))
            self.assertTrue((site_dir / "libgnsspp_architecture.png").exists())
            self.assertTrue(self._doc_exists(site_dir, "track_a_benchmark_2018"))
            self.assertTrue(self._doc_exists(site_dir, "ssr2obs_dump_implementation"))
            self.assertTrue(self._doc_exists(site_dir, "references"))
            self.assertTrue(self._doc_exists(site_dir, "references/madocalib-gap"))
            self.assertTrue(self._doc_exists(site_dir, "references/claslib-gap"))
            self.assertTrue(self._doc_exists(site_dir, "references/roadmap"))
            self.assertTrue(self._doc_exists(site_dir, "clas"))
            self.assertTrue(self._doc_exists(site_dir, "clas_quickstart"))
            self.assertTrue(self._doc_exists(site_dir, "clas_compact_ssr_policies"))
            self.assertTrue(self._doc_exists(site_dir, "clas_parity_artifacts"))
            self.assertTrue(self._doc_exists(site_dir, "clas_debug_playbook"))
            self.assertTrue(self._doc_exists(site_dir, "clas_parity_blockers"))
            self.assertTrue((site_dir / "javascripts" / "mermaid-init.js").exists())
            clas_html = None
            for candidate in (
                site_dir / "clas" / "index.html",
                site_dir / "clas.html",
            ):
                if candidate.exists():
                    clas_html = candidate.read_text(encoding="utf-8")
                    break
            self.assertIsNotNone(clas_html, "missing rendered CLAS overview page")
            self.assertIn("mermaid.min.js", clas_html)
            self.assertIn("class=\"mermaid\"", clas_html)
        finally:
            shutil.rmtree(site_dir, ignore_errors=True)


if __name__ == "__main__":
    unittest.main()
