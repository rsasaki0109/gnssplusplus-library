#!/usr/bin/env python3
"""Network-independent tests for the GitHub growth snapshot collector."""

from __future__ import annotations

import contextlib
import copy
import importlib.util
import io
import os
import tempfile
import unittest
from pathlib import Path
from unittest import mock
from urllib.error import HTTPError


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = ROOT_DIR / "scripts" / "metrics" / "capture_growth_snapshot.py"
SPEC = importlib.util.spec_from_file_location("capture_growth_snapshot", SCRIPT_PATH)
assert SPEC is not None and SPEC.loader is not None
growth = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(growth)


class FakeClient:
    def __init__(self, payloads: dict[str, object]) -> None:
        self.payloads = payloads
        self.calls: list[str] = []

    def get(self, endpoint: str) -> object:
        self.calls.append(endpoint)
        return copy.deepcopy(self.payloads[endpoint])


def fixture_client(stars: int = 183, asset_downloads: int = 1) -> FakeClient:
    repo = "/repos/rsasaki0109/gnssplusplus-library"
    return FakeClient(
        {
            repo: {
                "stargazers_count": stars,
                "forks_count": 21,
                "watchers_count": 183,
                "subscribers_count": 7,
                "html_url": "https://github.com/rsasaki0109/gnssplusplus-library",
            },
            f"{repo}/traffic/views": {
                "count": 42,
                "uniques": 12,
                "views": [{"timestamp": "2026-08-12T00:00:00Z", "count": 4, "uniques": 2}],
            },
            f"{repo}/traffic/clones": {
                "count": 24,
                "uniques": 8,
                "clones": [{"timestamp": "2026-08-12T00:00:00Z", "count": 6, "uniques": 3}],
            },
            f"{repo}/traffic/popular/referrers": [
                {"referrer": "github.com", "count": 10, "uniques": 5},
                {"referrer": "Google", "count": 4, "uniques": 3},
            ],
            f"{repo}/traffic/popular/paths": [
                {"path": "/rsasaki0109/gnssplusplus-library", "title": "repo", "count": 11, "uniques": 6}
            ],
            f"{repo}/releases?per_page=100": [
                {
                    "tag_name": "v0.2.0",
                    "name": "libgnss++ v0.2.0",
                    "published_at": "2026-08-01T00:00:00Z",
                    "html_url": "https://github.com/rsasaki0109/gnssplusplus-library/releases/tag/v0.2.0",
                    "assets": [
                        {
                            "name": "libgnsspp.tar.gz",
                            "download_count": asset_downloads,
                            "size": 123,
                            "browser_download_url": "https://example.invalid/asset",
                        }
                    ],
                }
            ],
        }
    )


class GrowthSnapshotTest(unittest.TestCase):
    def test_endpoint_aggregation_and_deterministic_timestamp(self) -> None:
        client = fixture_client()
        snapshot = growth.capture_snapshot(
            "rsasaki0109/gnssplusplus-library",
            client,
            captured_at="2026-08-12T09:00:00+09:00",
        )
        self.assertEqual(snapshot["schema_version"], "growth_snapshot.v1")
        self.assertEqual(snapshot["captured_at"], "2026-08-12T00:00:00Z")
        self.assertEqual(snapshot["public"]["stars"], 183)
        self.assertEqual(snapshot["public"]["forks"], 21)
        self.assertEqual(snapshot["public"]["watchers"], 7)
        self.assertEqual(snapshot["public"]["subscribers"], 7)
        self.assertEqual(snapshot["traffic"]["views"]["count"], 42)
        self.assertEqual(snapshot["traffic"]["clones"]["uniques"], 8)
        self.assertEqual(snapshot["traffic"]["clones"]["daily"][0]["count"], 6)
        self.assertEqual(snapshot["traffic"]["referrers"][0]["referrer"], "github.com")
        self.assertEqual(snapshot["releases"][0]["asset_downloads"], 1)
        self.assertEqual(len(client.calls), 6)

    def test_markdown_and_comparison_math(self) -> None:
        current = growth.capture_snapshot(
            "rsasaki0109/gnssplusplus-library",
            fixture_client(stars=183, asset_downloads=3),
            captured_at="2026-08-12T00:00:00Z",
        )
        baseline = growth.capture_snapshot(
            "rsasaki0109/gnssplusplus-library",
            fixture_client(stars=180, asset_downloads=1),
            captured_at="2026-08-10T00:00:00Z",
        )
        baseline["traffic"]["views"]["count"] = 17
        baseline["traffic"]["views"]["uniques"] = 5
        current["comparison"] = growth.compare_snapshots(current, baseline)
        self.assertEqual(current["comparison"]["stars_delta"], 3)
        self.assertEqual(current["comparison"]["gap_to_200"], 17)
        self.assertEqual(current["comparison"]["traffic"]["views"]["count"]["delta"], 25)
        self.assertEqual(current["comparison"]["release_assets"]["delta"], 2)
        markdown = growth.render_markdown(current)
        self.assertIn("Stars: `183`", markdown)
        self.assertIn("Gap to 200 stars: `17`", markdown)
        self.assertIn("rolling 14-day", markdown)
        self.assertIn("v0.2.0", markdown)

    def test_missing_token_error_is_clear_and_redacted(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "snapshot.json"
            stderr = io.StringIO()
            with mock.patch.dict(os.environ, {}, clear=True), contextlib.redirect_stderr(stderr):
                result = growth.main(
                    ["--repo", "owner/name", "--output", str(output)]
                )
            self.assertEqual(result, 2)
            self.assertIn("GH_TOKEN", stderr.getvalue())
            self.assertNotIn("ghs_", stderr.getvalue())
            self.assertFalse(output.exists())

    def test_http_auth_error_does_not_expose_token(self) -> None:
        secret = "ghs_example_secret"
        error = HTTPError("https://api.github.com/repos/owner/name", 401, "unauthorized", {}, None)
        with mock.patch.object(growth.urllib.request, "urlopen", side_effect=error):
            with self.assertRaises(growth.SnapshotError) as raised:
                growth.GitHubClient(secret).get("/repos/owner/name")
        self.assertIn("401", str(raised.exception))
        self.assertNotIn(secret, str(raised.exception))

    def test_invalid_repo_and_output_collision_are_rejected(self) -> None:
        with self.assertRaises(growth.SnapshotError):
            growth.validate_repo("not-an-owner-name")
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "snapshot.json"
            output.write_text("{}", encoding="utf-8")
            with self.assertRaises(growth.SnapshotError):
                growth.validate_output_paths(output, None, output)

    def test_symlink_output_is_rejected_when_supported(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            target = root / "target.json"
            target.write_text("{}", encoding="utf-8")
            link = root / "link.json"
            try:
                link.symlink_to(target)
            except (OSError, NotImplementedError):
                self.skipTest("symlinks unavailable")
            with self.assertRaises(growth.SnapshotError):
                growth.validate_output_paths(link, None, None)


if __name__ == "__main__":
    unittest.main()
