#!/usr/bin/env python3
"""Browser-free gnss web HTTP user-experience smoke tests."""

from __future__ import annotations

import argparse
import json
import sys
import tempfile
import threading
import unittest
from http.server import ThreadingHTTPServer
from pathlib import Path
from urllib import error, request


ROOT_DIR = Path(__file__).resolve().parents[1]
APPS_DIR = ROOT_DIR / "apps"

if str(APPS_DIR) not in sys.path:
    sys.path.insert(0, str(APPS_DIR))

import gnss_web  # noqa: E402


def make_args(root: Path, manifest_path: Path) -> argparse.Namespace:
    return argparse.Namespace(
        root=root,
        lib_pos=None,
        rtklib_pos=None,
        odaiba_summary=None,
        rcv_status=None,
        ppc_summary_glob="output/ppc_*_summary.json",
        live_summary_glob="output/live*_summary.json",
        robotics_summary_glob="output/robotics_smoke*/**/*.json",
        ros2_bag_summary_glob="output/ros2_bag*_summary.json",
        field_report_glob="output/field_report*.json",
        visibility_summary_glob="output/visibility*_summary.json",
        moving_base_summary_glob="output/*moving_base_summary.json",
        ppp_products_summary_glob="output/*ppp*_products*_summary.json",
        artifact_manifest=manifest_path,
        docs_url=gnss_web.DOCS_SITE_URL,
    )


def fetch_text(url: str) -> tuple[int, str, str]:
    try:
        with request.urlopen(url, timeout=5.0) as response:
            return (
                response.status,
                response.headers.get("Content-Type", ""),
                response.read().decode("utf-8"),
            )
    except error.HTTPError as exc:
        return (
            exc.code,
            exc.headers.get("Content-Type", ""),
            exc.read().decode("utf-8"),
        )


def fetch_json(url: str) -> tuple[int, str, dict[str, object]]:
    status, content_type, body = fetch_text(url)
    return status, content_type, json.loads(body)


class WebHttpUxTest(unittest.TestCase):
    def test_web_http_entrypoints_are_stable_without_browser(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_web_http_") as temp_dir:
            root = Path(temp_dir)
            output = root / "output"
            output.mkdir()
            artifact_path = output / "synthetic.txt"
            artifact_path.write_text("synthetic artifact\n", encoding="utf-8")
            manifest_path = output / "artifact_manifest.json"
            manifest_path.write_text(
                json.dumps(
                    {
                        "root": str(root),
                        "bundle_count": 1,
                        "bundles": [
                            {
                                "category": "ppc",
                                "label": "ppc_tokyo_run1_summary.json",
                                "summary_json": "output/ppc_tokyo_run1_summary.json",
                                "status": "excellent",
                                "headline": "200 matched / 98.0 fix / 0.114 p95 H",
                                "artifacts": {
                                    "summary": "output/ppc_tokyo_run1_summary.json",
                                    "text": "output/synthetic.txt",
                                },
                                "metrics": {"matched_epochs": 200},
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )

            server = ThreadingHTTPServer(
                ("127.0.0.1", 0),
                gnss_web.make_handler(make_args(root, manifest_path)),
            )
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            try:
                bound_port = int(server.server_address[1])
                base_url = f"http://127.0.0.1:{bound_port}"

                status, content_type, html = fetch_text(f"{base_url}/")
                self.assertEqual(status, 200)
                self.assertIn("text/html", content_type)
                self.assertIn("libgnss++ local web UI", html)
                self.assertIn("/api/overview", html)

                status, content_type, health = fetch_json(f"{base_url}/api/health")
                self.assertEqual(status, 200)
                self.assertIn("application/json", content_type)
                self.assertIs(health["ok"], True)

                status, _, overview = fetch_json(f"{base_url}/api/overview")
                self.assertEqual(status, 200)
                self.assertEqual(overview["artifacts"]["artifact_manifest"], "output/artifact_manifest.json")
                self.assertEqual(len(overview["artifact_manifest"]), 1)
                bundle = overview["artifact_manifest"][0]
                self.assertEqual(bundle["category"], "ppc")
                self.assertEqual(bundle["artifacts"]["text"], "output/synthetic.txt")

                status, content_type, artifact = fetch_text(
                    f"{base_url}/artifact?path=output/synthetic.txt"
                )
                self.assertEqual(status, 200)
                self.assertIn("text/plain", content_type)
                self.assertEqual(artifact, "synthetic artifact\n")

                status, _, missing_path = fetch_json(f"{base_url}/artifact")
                self.assertEqual(status, 400)
                self.assertEqual(missing_path["error"], "missing artifact path")

                status, _, escaped_path = fetch_json(f"{base_url}/artifact?path=../outside.txt")
                self.assertEqual(status, 400)
                self.assertEqual(escaped_path["error"], "artifact path escapes artifact root")
            finally:
                server.shutdown()
                server.server_close()
                thread.join(timeout=5)


if __name__ == "__main__":
    unittest.main()
