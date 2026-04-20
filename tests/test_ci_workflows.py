#!/usr/bin/env python3
"""Regression checks for GitHub Actions workflows."""

from __future__ import annotations

import re
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]


class WorkflowRegressionTest(unittest.TestCase):
    def test_ci_workflow_covers_develop_prs_and_uploads_failure_logs(self) -> None:
        workflow = (ROOT_DIR / ".github" / "workflows" / "ci.yml").read_text(encoding="utf-8")
        self.assertRegex(workflow, r"pull_request:\s*\n\s*branches:\s*\[\s*main,\s*develop\s*\]")
        self.assertIn("workflow_dispatch:", workflow)
        self.assertIn("group: ci-${{ github.workflow }}-${{ github.ref }}", workflow)
        self.assertIn("Upload CTest logs", workflow)
        self.assertIn("if: failure()", workflow)
        self.assertIn("build/Testing/**/*", workflow)

    def test_ci_workflow_runs_clasnat_parity_against_checked_out_claslib(self) -> None:
        workflow = (ROOT_DIR / ".github" / "workflows" / "ci.yml").read_text(encoding="utf-8")
        self.assertIn("clasnat-parity:", workflow)
        self.assertIn("repository: QZSS-Strategy-Office/claslib", workflow)
        self.assertIn("path: external/claslib", workflow)
        self.assertIn("-DCLASLIB_PARITY_LINK=ON", workflow)
        self.assertIn('-DCLASLIB_ROOT_DIR="${GITHUB_WORKSPACE}/external/claslib"', workflow)
        self.assertIn("--gtest_filter='*ClasnatParity*'", workflow)

    def test_nightly_workflow_runs_long_clas_regression_on_schedule_and_manual(self) -> None:
        workflow = (ROOT_DIR / ".github" / "workflows" / "nightly.yml").read_text(encoding="utf-8")
        self.assertRegex(workflow, r"schedule:\s*\n\s*-\s*cron:\s*\"0 17 \* \* \*\"")
        self.assertIn("workflow_dispatch:", workflow)
        self.assertIn("timeout-minutes: 30", workflow)
        self.assertIn("--target gnss_ppp", workflow)
        self.assertIn(
            "python3 -m pytest tests/test_cli_tools.py::CLIToolsTest::test_clas_ppp_regression_cm_accuracy -q",
            workflow,
        )
        self.assertIn("Upload CLAS regression artifacts", workflow)
        self.assertIn("if: failure()", workflow)

    def test_docs_workflow_builds_on_pr_and_deploys_only_from_develop_push(self) -> None:
        workflow = (ROOT_DIR / ".github" / "workflows" / "docs.yml").read_text(encoding="utf-8")
        self.assertRegex(workflow, r"pull_request:\s*\n\s*branches:\s*\[\s*main,\s*develop\s*\]")
        self.assertIn("group: docs-${{ github.workflow }}-${{ github.ref }}", workflow)
        self.assertIn("if: github.event_name == 'push' && github.ref == 'refs/heads/develop'", workflow)
        self.assertIn("python -m mkdocs build --strict", workflow)

    def test_docker_workflow_validates_prs_without_pushing(self) -> None:
        workflow = (ROOT_DIR / ".github" / "workflows" / "docker.yml").read_text(encoding="utf-8")
        self.assertRegex(workflow, r"pull_request:\s*\n\s*branches:\s*\[\s*main,\s*develop\s*\]")
        self.assertIn("if: github.event_name != 'pull_request'", workflow)
        self.assertIn("push: ${{ github.event_name != 'pull_request' }}", workflow)
        self.assertIn("cache-from: type=gha", workflow)
        self.assertIn("cache-to: type=gha,mode=max", workflow)


if __name__ == "__main__":
    unittest.main()
