#!/usr/bin/env python3
"""Regression tests for tools/compare_ppp_solutions.py and gnss_ppp PPP options."""

from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile
import unittest
from datetime import datetime, timezone
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
COMPARE_SCRIPT = ROOT_DIR / "tools" / "compare_ppp_solutions.py"
PYTHON = Path(sys.executable)


def _resolve_gnss_ppp_bin() -> Path:
    for candidate in (ROOT_DIR / "build" / "gnss_ppp", ROOT_DIR / "build" / "apps" / "gnss_ppp"):
        if candidate.is_file():
            return candidate
    return ROOT_DIR / "build" / "gnss_ppp"


PPP_BIN = _resolve_gnss_ppp_bin()


def _write_minimal_pos(path: Path, rows: list[tuple]) -> None:
    """rows: (week, tow, x, y, z, lat_deg, lon_deg, h, status, sats, pdop)"""
    lines = [
        "% LibGNSS++ Position Solution",
        "% Format: pos",
        "% Columns: GPS_Week GPS_TOW X(m) Y(m) Z(m) Lat(deg) Lon(deg) Height(m) Status Satellites PDOP",
    ]
    for r in rows:
        lines.append(
            f"{r[0]} {r[1]:.6f} {r[2]:.6f} {r[3]:.6f} {r[4]:.6f} "
            f"{r[5]:.6f} {r[6]:.6f} {r[7]:.6f} {r[8]} {r[9]} {r[10]:.6f}"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


class ComparePppToolTest(unittest.TestCase):
    def test_self_reference_trajectory_zero_rms(self) -> None:
        fd, pos_str = tempfile.mkstemp(prefix="gnsspp_cmp_", suffix=".pos")
        os.close(fd)
        pos = Path(pos_str)
        try:
            _write_minimal_pos(
                pos,
                [
                    (2068, 100.0, -3957237.0, 3310369.0, 3737530.0, 36.1, 140.0, 70.0, 5, 12, 2.0),
                    (2068, 101.0, -3957237.1, 3310369.1, 3737530.1, 36.1, 140.0, 70.0, 5, 12, 2.0),
                    (2068, 102.0, -3957237.2, 3310369.2, 3737530.2, 36.1, 140.0, 70.0, 5, 12, 2.0),
                ],
            )
            jfd, jpath = tempfile.mkstemp(prefix="gnsspp_cmp_", suffix=".json")
            os.close(jfd)
            out_json = Path(jpath)
            try:
                r = subprocess.run(
                    [
                        str(PYTHON),
                        str(COMPARE_SCRIPT),
                        "--candidate",
                        str(pos),
                        "--reference",
                        str(pos),
                        "--last-n",
                        "10",
                        "--json-out",
                        str(out_json),
                    ],
                    capture_output=True,
                    text=True,
                    check=False,
                )
                self.assertEqual(r.returncode, 0, msg=r.stderr + r.stdout)
                data = json.loads(out_json.read_text(encoding="utf-8"))
                err = data["error_vs_reference"]
                self.assertEqual(err["epochs_used"], 3)
                self.assertAlmostEqual(err["rms_3d_m"], 0.0, places=9)
                self.assertAlmostEqual(err["rms_h_m"], 0.0, places=9)
            finally:
                Path(out_json).unlink(missing_ok=True)
        finally:
            pos.unlink(missing_ok=True)

    def test_reference_ecef_stats_json(self) -> None:
        fd, pos_str = tempfile.mkstemp(prefix="gnsspp_cmp_", suffix=".pos")
        os.close(fd)
        pos = Path(pos_str)
        try:
            _write_minimal_pos(
                pos,
                [
                    (2068, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5, 8, 2.0),
                    (2068, 101.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 5, 8, 2.0),
                ],
            )
            jfd, jpath = tempfile.mkstemp(prefix="gnsspp_cmp_", suffix=".json")
            os.close(jfd)
            out_json = Path(jpath)
            try:
                r = subprocess.run(
                    [
                        str(PYTHON),
                        str(COMPARE_SCRIPT),
                        "--candidate",
                        str(pos),
                        "--ecef-x",
                        "0",
                        "--ecef-y",
                        "0",
                        "--ecef-z",
                        "0",
                        "--last-n",
                        "10",
                        "--json-out",
                        str(out_json),
                    ],
                    capture_output=True,
                    text=True,
                    check=False,
                )
                self.assertEqual(r.returncode, 0, msg=r.stderr + r.stdout)
                data = json.loads(out_json.read_text(encoding="utf-8"))
                self.assertIn("candidate_summary", data)
                self.assertEqual(data["candidate_summary"]["valid_epochs"], 2)
            finally:
                Path(out_json).unlink(missing_ok=True)
        finally:
            pos.unlink(missing_ok=True)

    def test_rtklib_xyz_reference_trajectory_aligns_with_libgnss(self) -> None:
        """CLASLIB/rnx2rtkp ECEF .pos (date + XYZ) pairs with LibGNSS week/tow .pos."""
        gps0 = datetime(1980, 1, 6, 0, 0, 0, tzinfo=timezone.utc).timestamp()

        def week_tow_to_date_time_str(week: int, tow: float) -> tuple[str, str]:
            ts = gps0 + week * 604800.0 + tow
            dt = datetime.fromtimestamp(ts, tz=timezone.utc)
            return dt.strftime("%Y/%m/%d"), dt.strftime("%H:%M:%S.%f")

        fd, lg_str = tempfile.mkstemp(prefix="gnsspp_lg_", suffix=".pos")
        os.close(fd)
        fd, rt_str = tempfile.mkstemp(prefix="gnsspp_rt_", suffix=".pos")
        os.close(fd)
        lg_path, rt_path = Path(lg_str), Path(rt_str)
        try:
            d1, t1 = week_tow_to_date_time_str(2068, 100.0)
            d2, t2 = week_tow_to_date_time_str(2068, 101.0)
            _write_minimal_pos(
                lg_path,
                [
                    (2068, 100.0, -3957237.0, 3310369.0, 3737530.0, 36.1, 140.0, 70.0, 5, 12, 2.0),
                    (2068, 101.0, -3957237.1, 3310369.1, 3737530.1, 36.1, 140.0, 70.0, 5, 12, 2.0),
                ],
            )
            rt_path.write_text(
                "\n".join(
                    [
                        "% RTKLIB pos (minimal test)",
                        f"{d1} {t1} -3957237.000000 3310369.000000 3737530.000000 2 12 0 0 0 0 2.000000",
                        f"{d2} {t2} -3957237.100000 3310369.100000 3737530.100000 2 12 0 0 0 0 2.000000",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )
            jfd, jpath = tempfile.mkstemp(prefix="gnsspp_cmp_", suffix=".json")
            os.close(jfd)
            out_json = Path(jpath)
            try:
                r = subprocess.run(
                    [
                        str(PYTHON),
                        str(COMPARE_SCRIPT),
                        "--candidate",
                        str(lg_path),
                        "--reference",
                        str(rt_path),
                        "--reference-format",
                        "rtklib-ecef",
                        "--last-n",
                        "10",
                        "--json-out",
                        str(out_json),
                    ],
                    capture_output=True,
                    text=True,
                    check=False,
                )
                self.assertEqual(r.returncode, 0, msg=r.stderr + r.stdout)
                data = json.loads(out_json.read_text(encoding="utf-8"))
                self.assertEqual(data.get("reference_format"), "rtklib-ecef")
                err = data["error_vs_reference"]
                self.assertEqual(err.get("epochs_used"), 2)
                self.assertAlmostEqual(err["rms_3d_m"], 0.0, places=6)
            finally:
                out_json.unlink(missing_ok=True)
        finally:
            lg_path.unlink(missing_ok=True)
            rt_path.unlink(missing_ok=True)

    @unittest.skipUnless(PPP_BIN.is_file(), f"gnss_ppp not built at {PPP_BIN}")
    def test_gnss_ppp_help_lists_ar_method(self) -> None:
        r = subprocess.run(
            [str(PPP_BIN), "--help"],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(r.returncode, 0, msg=r.stderr)
        self.assertIn("--ar-method", r.stdout)


if __name__ == "__main__":
    unittest.main()
