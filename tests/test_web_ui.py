#!/usr/bin/env python3
"""Playwright smoke tests for the local gnss web UI."""

from __future__ import annotations

import json
import socket
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

try:
    from playwright.sync_api import Error as PlaywrightError
    from playwright.sync_api import sync_playwright
except ImportError as exc:  # pragma: no cover - exercised through skip path
    sync_playwright = None
    PlaywrightError = RuntimeError
    PLAYWRIGHT_IMPORT_ERROR = exc
else:
    PLAYWRIGHT_IMPORT_ERROR = None


ROOT_DIR = Path(__file__).resolve().parents[1]
DISPATCHER = ROOT_DIR / "apps" / "gnss.py"


def find_free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def wait_for_file(path: Path, timeout_s: float = 5.0) -> str:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if path.exists():
            return path.read_text(encoding="utf-8").strip()
        time.sleep(0.05)
    raise TimeoutError(f"timed out waiting for {path}")


@unittest.skipIf(sync_playwright is None, f"Playwright import unavailable: {PLAYWRIGHT_IMPORT_ERROR}")
class WebUISmokeTest(unittest.TestCase):
    def test_web_ui_renders_overview_status_and_ppc_rows(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_web_ui_") as temp_dir:
            temp_root = Path(temp_dir)
            lib_pos = temp_root / "lib.pos"
            rtklib_pos = temp_root / "rtklib.pos"
            summary_json = temp_root / "odaiba_summary.json"
            status_json = temp_root / "receiver.status.json"
            ppc_summary = temp_root / "output" / "ppc_tokyo_run1_rtk_summary.json"
            port_file = temp_root / "port.txt"

            ppc_summary.parent.mkdir(parents=True, exist_ok=True)
            lib_pos.write_text(
                "\n".join(
                    [
                        "% synthetic libgnss++",
                        "2200 100.0 1.0 2.0 3.0 35.000000000 139.000000000 10.0 4 12 3.5",
                        "2200 101.0 2.0 3.0 4.0 35.000001000 139.000001000 10.1 3 11 2.1",
                    ]
                )
                + "\n",
                encoding="ascii",
            )
            rtklib_pos.write_text(
                "\n".join(
                    [
                        "% synthetic rtklib",
                        "2200 100.0 35.000000000 139.000000000 10.0 1 12",
                        "2200 101.0 35.000001000 139.000001000 10.1 2 11",
                    ]
                )
                + "\n",
                encoding="ascii",
            )
            summary_json.write_text(
                json.dumps(
                    {
                        "all_epochs": {
                            "libgnsspp": {"epochs": 11637, "fix_rate_pct": 8.11},
                            "rtklib": {"epochs": 8241, "fix_rate_pct": 7.22},
                        },
                        "common_epochs": {
                            "libgnsspp": {"median_h_m": 0.733387, "p95_h_m": 5.941091},
                            "rtklib": {"median_h_m": 0.703880, "p95_h_m": 27.673014},
                        },
                    }
                ),
                encoding="utf-8",
            )
            status_json.write_text(
                json.dumps(
                    {
                        "state": "running",
                        "pid": 4321,
                        "pid_running": True,
                        "uptime_seconds": 18.5,
                        "restart_count": 2,
                    }
                ),
                encoding="utf-8",
            )
            ppc_summary.write_text(
                json.dumps(
                    {
                        "matched_epochs": 120,
                        "fix_rate_pct": 96.67,
                        "median_h_m": 0.108,
                        "p95_h_m": 0.110,
                        "realtime_factor": 1.23,
                    }
                ),
                encoding="utf-8",
            )

            port = find_free_port()
            process = subprocess.Popen(
                [
                    sys.executable,
                    str(DISPATCHER),
                    "web",
                    "--host",
                    "127.0.0.1",
                    "--port",
                    str(port),
                    "--port-file",
                    str(port_file),
                    "--root",
                    str(temp_root),
                    "--lib-pos",
                    str(lib_pos),
                    "--rtklib-pos",
                    str(rtklib_pos),
                    "--odaiba-summary",
                    str(summary_json),
                    "--rcv-status",
                    str(status_json),
                ],
                cwd=ROOT_DIR,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            try:
                bound_port = int(wait_for_file(port_file))
                with sync_playwright() as playwright:
                    try:
                        browser = playwright.chromium.launch(headless=True)
                    except PlaywrightError as exc:  # pragma: no cover - exercised through skip path
                        self.skipTest(f"Chromium unavailable for Playwright: {exc}")
                    page = browser.new_page()
                    page.goto(f"http://127.0.0.1:{bound_port}/", wait_until="networkidle")

                    self.assertEqual(page.locator("h1").text_content(), "libgnss++ local web UI")
                    self.assertTrue(page.locator("#artifact-chips").text_content())
                    self.assertIn("11637", page.locator("#odaiba-metrics").text_content())
                    self.assertIn("running", page.locator("#receiver-metrics").text_content())
                    self.assertIn('"restart_count": 2', page.locator("#receiver-json").text_content())
                    self.assertEqual(page.locator("canvas").count(), 2)
                    self.assertIn("FIXED", page.locator("#status-legend").text_content())
                    self.assertIn("ppc_tokyo_run1_rtk_summary.json", page.locator("#ppc-table tbody").text_content())
                    self.assertIn("96.67%", page.locator("#ppc-table tbody").text_content())
                    browser.close()
            finally:
                process.terminate()
                try:
                    process.communicate(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.communicate(timeout=5)


if __name__ == "__main__":
    unittest.main()
