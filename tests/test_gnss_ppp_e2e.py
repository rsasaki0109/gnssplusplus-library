#!/usr/bin/env python3
"""Short gnss_ppp end-to-end regression tests (repo data/rover_static.obs)."""

from __future__ import annotations

import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
OBS_STATIC = ROOT_DIR / "data" / "rover_static.obs"
NAV_STATIC = ROOT_DIR / "data" / "navigation_static.nav"


def _resolve_gnss_ppp_bin() -> Path:
    for candidate in (ROOT_DIR / "build" / "gnss_ppp", ROOT_DIR / "build" / "apps" / "gnss_ppp"):
        if candidate.is_file():
            return candidate
    return ROOT_DIR / "build" / "gnss_ppp"


PPP_BIN = _resolve_gnss_ppp_bin()


def _data_ready() -> bool:
    return OBS_STATIC.is_file() and NAV_STATIC.is_file() and PPP_BIN.is_file()


def _parse_pos_body_lines(path: Path) -> list[list[str]]:
    rows: list[list[str]] = []
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        line = line.strip()
        if not line or line.startswith("%"):
            continue
        rows.append(line.split())
    return rows


def _assert_pos_contract(path: Path, min_epochs: int) -> None:
    rows = _parse_pos_body_lines(path)
    assert len(rows) >= min_epochs, f"expected >= {min_epochs} epochs, got {len(rows)}"
    for tokens in rows:
        assert len(tokens) >= 11, tokens
        week = int(tokens[0])
        assert week > 0, f"invalid GPS week in output: {tokens!r}"
        tow = float(tokens[1])
        assert tow >= 0.0, f"negative TOW: {tokens}"
        status = int(tokens[8])
        assert 0 <= status <= 10, f"status out of range: {status}"


class GnssPppE2ETest(unittest.TestCase):
    @unittest.skipUnless(_data_ready(), "gnss_ppp or static RINEX test data missing")
    def test_ppp_float_short_run(self) -> None:
        fd, out_str = tempfile.mkstemp(prefix="gnsspp_e2e_", suffix=".pos")
        os.close(fd)
        out = Path(out_str)
        try:
            r = subprocess.run(
                [
                    str(PPP_BIN),
                    "--obs",
                    str(OBS_STATIC),
                    "--nav",
                    str(NAV_STATIC),
                    "--out",
                    str(out),
                    "--static",
                    "--max-epochs",
                    "30",
                    "--quiet",
                ],
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(r.returncode, 0, msg=r.stderr + r.stdout)
            _assert_pos_contract(out, min_epochs=25)
        finally:
            out.unlink(missing_ok=True)

    @unittest.skipUnless(_data_ready(), "gnss_ppp or static RINEX test data missing")
    def test_ppp_ar_wlnl_short_run_exits_cleanly(self) -> None:
        """AR may be skipped (no SSR); we only require a valid .pos time series."""
        fd, out_str = tempfile.mkstemp(prefix="gnsspp_e2e_ar_", suffix=".pos")
        os.close(fd)
        out = Path(out_str)
        try:
            r = subprocess.run(
                [
                    str(PPP_BIN),
                    "--obs",
                    str(OBS_STATIC),
                    "--nav",
                    str(NAV_STATIC),
                    "--out",
                    str(out),
                    "--static",
                    "--estimate-troposphere",
                    "--enable-ar",
                    "--ar-method",
                    "dd-wlnl",
                    "--ar-ratio-threshold",
                    "2.0",
                    "--max-epochs",
                    "30",
                    "--quiet",
                ],
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(r.returncode, 0, msg=r.stderr + r.stdout)
            _assert_pos_contract(out, min_epochs=25)
        finally:
            out.unlink(missing_ok=True)


class GnssPppHelpTest(unittest.TestCase):
    @unittest.skipUnless(PPP_BIN.is_file(), f"gnss_ppp not built at {PPP_BIN}")
    def test_help_documents_clas_hybrid_default(self) -> None:
        r = subprocess.run(
            [str(PPP_BIN), "--help"],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(r.returncode, 0, msg=r.stderr)
        self.assertIn("hybrid-standard-ppp", r.stdout)
        self.assertIn("--clas-osr", r.stdout)
        self.assertIn("--claslib-parity", r.stdout)


if __name__ == "__main__":
    unittest.main()
