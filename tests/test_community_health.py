#!/usr/bin/env python3
"""Pure-stdlib regression checks for the public community onboarding surface."""

from __future__ import annotations

import re
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
ISSUE_TEMPLATE_DIR = ROOT_DIR / ".github" / "ISSUE_TEMPLATE"
ALLOWED_LABELS = {
    "bug",
    "documentation",
    "enhancement",
    "good first issue",
    "help wanted",
    "question",
}


def read(relative_path: str) -> str:
    return (ROOT_DIR / relative_path).read_text(encoding="utf-8")


def form_labels(relative_path: str) -> set[str]:
    text = read(relative_path)
    match = re.search(r"^labels:\s*\[(.*?)\]\s*$", text, re.MULTILINE)
    if match is None:
        return set()
    return {
        item.strip().strip("\"'")
        for item in match.group(1).split(",")
        if item.strip()
    }


class CommunityHealthTest(unittest.TestCase):
    def test_required_community_files_exist(self) -> None:
        for relative_path in (
            "CODE_OF_CONDUCT.md",
            "SECURITY.md",
            "SUPPORT.md",
            ".github/ISSUE_TEMPLATE/feature_request.yml",
            ".github/ISSUE_TEMPLATE/question.yml",
            "docs/community.md",
        ):
            with self.subTest(path=relative_path):
                self.assertTrue((ROOT_DIR / relative_path).is_file())

    def test_issue_forms_use_only_existing_labels_and_no_stale_triage_label(self) -> None:
        forms = sorted(ISSUE_TEMPLATE_DIR.glob("*.yml"))
        self.assertTrue(forms)
        for path in forms:
            text = path.read_text(encoding="utf-8")
            with self.subTest(path=path.name):
                self.assertNotIn("needs-triage", text)
                self.assertTrue(form_labels(str(path.relative_to(ROOT_DIR))) <= ALLOWED_LABELS)
        self.assertEqual(form_labels(".github/ISSUE_TEMPLATE/bug_report.yml"), {"bug"})
        self.assertEqual(form_labels(".github/ISSUE_TEMPLATE/development_slice.yml"), {"help wanted"})
        self.assertEqual(form_labels(".github/ISSUE_TEMPLATE/feature_request.yml"), {"enhancement"})
        self.assertEqual(form_labels(".github/ISSUE_TEMPLATE/question.yml"), {"question"})

    def test_issue_config_disables_blank_issues_and_routes_support_docs_security(self) -> None:
        config = read(".github/ISSUE_TEMPLATE/config.yml")
        self.assertIn("blank_issues_enabled: false", config)
        self.assertIn("name: Support and questions", config)
        self.assertIn("name: Documentation and quick starts", config)
        self.assertIn("name: Private security report", config)
        self.assertIn("issues/new?template=question.yml", config)
        self.assertIn("rsasaki0109.github.io/gnssplusplus-library/", config)
        self.assertIn("security/advisories/new", config)
        self.assertNotIn("discussions", config.lower())

    def test_forms_have_structured_user_value_and_data_fields(self) -> None:
        feature = read(".github/ISSUE_TEMPLATE/feature_request.yml")
        question = read(".github/ISSUE_TEMPLATE/question.yml")
        for field in ("user_problem", "smallest_value", "acceptance", "data_needs"):
            with self.subTest(form="feature", field=field):
                self.assertIn(f"id: {field}", feature)
        for field in ("goal", "context", "tried", "expected", "data_needs"):
            with self.subTest(form="question", field=field):
                self.assertIn(f"id: {field}", question)

    def test_policy_routing_and_enforcement_contracts(self) -> None:
        conduct = read("CODE_OF_CONDUCT.md")
        security = read("SECURITY.md")
        support = read("SUPPORT.md")
        self.assertIn("rsasaki0109@gmail.com", conduct)
        self.assertIn("github.com/contact/report-abuse", conduct)
        self.assertIn("Scope", conduct)
        self.assertIn("enforcement", conduct.lower())
        self.assertIn("v0.2.x", security)
        self.assertIn("security/advisories/new", security)
        self.assertIn("best-effort", security)
        self.assertIn("no SLA", security)
        for token in ("RINEX", "RTCM", "UBX", "Python bindings"):
            with self.subTest(token=token):
                self.assertIn(token, security)
        for token in ("question.yml", "bug_report.yml", "feature_request.yml", "SECURITY.md"):
            with self.subTest(token=token):
                self.assertIn(token, support)

    def test_first_contribution_links_and_three_lanes_are_visible(self) -> None:
        community = read("docs/community.md")
        contributing = read("CONTRIBUTING.md")
        docs_index = read("docs/index.md")
        mkdocs = read("mkdocs.yml")
        readme = read("README.md")
        self.assertIn("15-minute first contribution", community)
        for token in (
            "Docs-only",
            "Python tool or test",
            "C++",
            "good first issue",
            "help wanted",
            "python3 -m mkdocs build --strict",
            "python3 tests/test_cli_ux.py",
            "cmake -S . -B build",
        ):
            with self.subTest(token=token):
                self.assertIn(token, community)
        self.assertIn("docs/community.md", contributing)
        self.assertIn("15-minute first contribution", contributing)
        self.assertIn("Community onboarding", docs_index)
        self.assertIn("Community Onboarding: community.md", mkdocs)
        self.assertIn("docs/community.md", readme)
        self.assertNotIn("discussions", community.lower())


if __name__ == "__main__":
    unittest.main()
