#!/usr/bin/env python3
"""Tests for `gnss uncertainty-eval` (truth-based honesty of reported sigma)."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import random
import subprocess
import sys
import tempfile
import unittest


ROOT_DIR = Path(__file__).resolve().parents[1]
COMMANDS_DIR = ROOT_DIR / "apps" / "commands"
DIAG_DIR = COMMANDS_DIR / "diagnostics"
sys.path.insert(0, str(COMMANDS_DIR))
sys.path.insert(0, str(DIAG_DIR))
sys.path.insert(0, str(ROOT_DIR / "scripts"))
SUBPROCESS_ENV = {
    **os.environ,
    "PYTHONPATH": os.pathsep.join(
        filter(None, (str(COMMANDS_DIR), str(ROOT_DIR / "scripts"), os.environ.get("PYTHONPATH", "")))
    ),
}

import gnss_uncertainty_eval as ue  # noqa: E402
import generate_driving_comparison as comparison  # noqa: E402

LAT_DEG, LON_DEG, HEIGHT_M = 35.0, 139.0, 40.0
BASE_COLUMNS = (
    "GPS_Week GPS_TOW X(m) Y(m) Z(m) Lat(deg) Lon(deg) Height(m) Status NumSat PDOP Ratio Baseline(m)"
)
UNC_COLUMNS = "sdn(m) sde(m) sdu(m) sdne(m) sdeu(m) sdun(m) DiffAge(s)"


def enu_to_ecef_delta(e: float, n: float, u: float) -> tuple[float, float, float]:
    lat, lon = math.radians(LAT_DEG), math.radians(LON_DEG)
    sl, cl, so, co = math.sin(lat), math.cos(lat), math.sin(lon), math.cos(lon)
    return (
        -so * e - sl * co * n + cl * co * u,
        co * e - sl * so * n + cl * so * u,
        cl * n + sl * u,
    )


def synthetic_samples(count: int, error_scale: float, seed: int = 7, sigma_axis: float = 0.05,
                      sigma_u: float = 0.08):
    """Errors ~ error_scale * N(0, sigma) per axis, reported sigma stays (sigma_axis, sigma_u)."""
    rng = random.Random(seed)
    e_h, e_u, s_h, s_v = [], [], [], []
    for _ in range(count):
        east = rng.gauss(0.0, sigma_axis * error_scale)
        north = rng.gauss(0.0, sigma_axis * error_scale)
        up = rng.gauss(0.0, sigma_u * error_scale)
        e_h.append(math.hypot(east, north))
        e_u.append(abs(up))
        s_h.append(math.hypot(sigma_axis, sigma_axis))
        s_v.append(sigma_u)
    return e_h, e_u, s_h, s_v


class GaussianHonestyTest(unittest.TestCase):
    def test_gaussian_errors_match_gaussian_exceedance(self) -> None:
        summary = ue.summarize_samples(*synthetic_samples(40000, 1.0))
        for k, tol_h, tol_v in ((1, 0.015, 0.015), (2, 0.006, 0.008), (3, 0.002, 0.003)):
            ex = summary["exceedance"][str(k)]
            self.assertAlmostEqual(ex["H"]["fraction"], math.exp(-k * k), delta=tol_h)
            self.assertAlmostEqual(ex["V"]["fraction"], math.erfc(k / math.sqrt(2)), delta=tol_v)
            self.assertAlmostEqual(ex["H"]["gaussian"], math.exp(-k * k))
        self.assertEqual(summary["exceedance"]["5"]["V"]["count"], 0)
        nz = summary["normalized_error"]
        self.assertAlmostEqual(nz["H"]["median"], math.sqrt(math.log(2.0)), delta=0.03)
        self.assertAlmostEqual(nz["V"]["median"], 0.6745, delta=0.03)
        self.assertAlmostEqual(nz["V"]["p95"], 1.96, delta=0.1)
        self.assertAlmostEqual(nz["V"]["gaussian_p99"], 2.576, delta=0.002)
        self.assertEqual(summary["verdict"]["H"], "consistent")
        self.assertEqual(summary["verdict"]["V"], "consistent")
        # K*sigma proxy at 5.33 should essentially never be violated by Gaussian errors.
        self.assertEqual(summary["stanford"]["H"]["regions"]["misleading"], 0)
        self.assertEqual(summary["stanford"]["V"]["regions"]["misleading"], 0)

    def test_overconfident_case_is_flagged(self) -> None:
        summary = ue.summarize_samples(*synthetic_samples(5000, 4.0))
        self.assertEqual(summary["verdict"]["H"], "overconfident")
        self.assertEqual(summary["verdict"]["V"], "overconfident")
        self.assertGreater(summary["exceedance"]["3"]["H"]["fraction"], 0.5)
        self.assertGreater(summary["stanford"]["V"]["regions"]["misleading"], 500)

    def test_over_conservative_case_is_flagged(self) -> None:
        summary = ue.summarize_samples(*synthetic_samples(5000, 0.2))
        self.assertEqual(summary["verdict"]["H"], "over_conservative")
        self.assertEqual(summary["verdict"]["V"], "over_conservative")

    def test_heavy_tail_with_good_core_is_flagged(self) -> None:
        e_h, e_u, s_h, s_v = synthetic_samples(5000, 1.0)
        for i in range(0, 5000, 50):  # 2% gross errors, e.g. wrong fixes
            e_h[i] = 20.0 * s_h[i]
            e_u[i] = 20.0 * s_v[i]
        summary = ue.summarize_samples(e_h, e_u, s_h, s_v)
        self.assertEqual(summary["verdict"]["H"], "overconfident")
        self.assertEqual(summary["verdict"]["V"], "overconfident")

    def test_insufficient_data(self) -> None:
        summary = ue.summarize_samples(*synthetic_samples(5, 1.0))
        self.assertEqual(summary["verdict"]["H"], "insufficient_data")

    def test_stanford_regions_with_alert_limit(self) -> None:
        # sigma = 0.1 -> bound = 0.533 at K=5.33
        errors = [0.1, 0.5, 0.6, 3.0]
        bounds = [0.533] * 4
        regions = ue.stanford_regions(errors, bounds, None)
        self.assertEqual(regions, {"nominal": 2, "misleading": 2, "unavailable": 0})
        regions = ue.stanford_regions(errors, bounds, 0.5)
        self.assertEqual(regions["unavailable"], 4)  # bound 0.533 > AL 0.5
        regions = ue.stanford_regions(errors, bounds, 1.0)
        self.assertEqual(
            regions, {"nominal": 2, "misleading": 2, "unavailable": 0, "hazardously_misleading": 1}
        )

    def test_stanford_bins_sum_to_n(self) -> None:
        e_h, e_u, s_h, s_v = synthetic_samples(1000, 2.0)
        summary = ue.summarize_samples(e_h, e_u, s_h, s_v)
        for axis in ("H", "V"):
            counts = summary["stanford"][axis]["counts_bound_by_error"]
            self.assertEqual(sum(sum(row) for row in counts), 1000)
        self.assertIn("NOT a certified", summary["stanford"]["label"])


class PosFileTest(unittest.TestCase):
    def write_files(self, directory: Path, rows, with_unc: bool = True):
        ref_lines = [
            "GPS TOW (s),GPS Week,Latitude (deg),Longitude (deg),Ellipsoid Height (m),ECEF X (m),ECEF Y (m),ECEF Z (m)"
        ]
        ref_ecef = comparison.llh_to_ecef(LAT_DEG, LON_DEG, HEIGHT_M)
        pos_lines = ["% LibGNSS++ Position Solution\n"]
        pos_lines.append(f"% {BASE_COLUMNS}{' ' + UNC_COLUMNS if with_unc else ''}\n")
        for tow, status, (east, north, up), (sdn, sde, sdu, age) in rows:
            ref_lines.append(
                f"{tow:.3f}, 2300, {LAT_DEG}, {LON_DEG}, {HEIGHT_M}, "
                f"{ref_ecef[0]:.4f}, {ref_ecef[1]:.4f}, {ref_ecef[2]:.4f}"
            )
            dx, dy, dz = enu_to_ecef_delta(east, north, up)
            line = (
                f"2300 {tow:.3f} {ref_ecef[0] + dx:.4f} {ref_ecef[1] + dy:.4f} {ref_ecef[2] + dz:.4f} "
                f"{LAT_DEG:.9f} {LON_DEG:.9f} {HEIGHT_M:.4f} {status} 12 1.50 3.0 100.0"
            )
            if with_unc:
                fmt = lambda v: "nan" if v is None else f"{v:.5f}"  # noqa: E731
                line += f" {fmt(sdn)} {fmt(sde)} {fmt(sdu)} 0.00000 0.00000 0.00000 {fmt(age)}"
            pos_lines.append(line + "\n")
        pos = directory / "run.pos"
        ref = directory / "reference.csv"
        pos.write_text("".join(pos_lines), encoding="ascii")
        ref.write_text("\n".join(ref_lines) + "\n", encoding="ascii")
        return pos, ref

    def test_reader_is_header_driven_and_nan_safe(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_unc_reader_") as tmp:
            pos, _ = self.write_files(
                Path(tmp),
                [
                    (100.0, 4, (0.01, 0.0, 0.0), (0.01, 0.02, 0.03, 0.0)),
                    (101.0, 3, (0.0, 0.0, 0.0), (None, None, None, None)),
                ],
            )
            rows = ue.read_pos_uncertainty(pos)
            self.assertEqual(len(rows), 2)
            self.assertAlmostEqual(rows[0]["sdn(m)"], 0.01)
            self.assertAlmostEqual(rows[0]["sdu(m)"], 0.03)
            self.assertTrue(math.isnan(rows[1]["sde(m)"]))

    def test_missing_columns_is_an_error(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_unc_nocols_") as tmp:
            pos, _ = self.write_files(
                Path(tmp), [(100.0, 4, (0.0, 0.0, 0.0), (0, 0, 0, 0))], with_unc=False
            )
            with self.assertRaises(SystemExit) as raised:
                ue.read_pos_uncertainty(pos)
            self.assertIn("--pos-uncertainty", str(raised.exception))

    def test_cli_end_to_end_json_csv_and_no_matplotlib_needed(self) -> None:
        rng = random.Random(3)
        rows = []
        for i in range(400):
            fixed = i % 2 == 0
            sd = 0.02 if fixed else 0.5
            rows.append((
                1000.0 + i * 0.2,
                4 if fixed else 3,
                (rng.gauss(0, sd), rng.gauss(0, sd), rng.gauss(0, sd * 1.5)),
                (sd, sd, sd * 1.5, 0.0),
            ))
        # one NaN-sigma epoch must be excluded, one wrong fix must be counted misleading
        rows.append((2000.0, 3, (0.0, 0.0, 0.0), (None, None, None, None)))
        rows.append((2001.0, 4, (3.0, 0.0, 0.0), (0.02, 0.02, 0.03, 0.0)))
        with tempfile.TemporaryDirectory(prefix="gnss_unc_cli_") as tmp:
            tmp_path = Path(tmp)
            pos, ref = self.write_files(tmp_path, rows)
            out_json = tmp_path / "out.json"
            out_csv = tmp_path / "epochs.csv"
            out_bins = tmp_path / "bins.csv"
            result = subprocess.run(
                [
                    sys.executable, str(DIAG_DIR / "gnss_uncertainty_eval.py"),
                    "--pos", str(pos), "--reference-csv", str(ref),
                    "--out-json", str(out_json), "--out-csv", str(out_csv),
                    "--out-stanford-csv", str(out_bins), "--out-png", str(tmp_path / "x.png"),
                    "--alert-limit-h", "1.0",
                ],
                check=True, capture_output=True, text=True, env=SUBPROCESS_ENV,
            )
            self.assertIn("NOT a certified", result.stdout)
            payload = json.loads(out_json.read_text(encoding="utf-8"))
            self.assertEqual(payload["counts"]["pos_rows"], 402)
            self.assertEqual(payload["counts"]["no_sigma"], 1)
            fixed = payload["groups"]["FIXED"]
            self.assertEqual(fixed["n"], 201)
            self.assertEqual(payload["groups"]["FLOAT"]["n"], 200)
            self.assertEqual(payload["groups"]["ALL"]["n"], 401)
            self.assertEqual(fixed["stanford"]["H"]["regions"]["misleading"], 1)
            self.assertEqual(fixed["stanford"]["H"]["regions"]["hazardously_misleading"], 1)
            # a single wrong fix in 201 epochs is a misleading epoch, not yet a distribution verdict
            self.assertEqual(fixed["verdict"]["H"], "consistent")
            self.assertEqual(fixed["exceedance"]["5"]["H"]["count"], 1)
            self.assertEqual(payload["groups"]["FLOAT"]["verdict"]["H"], "consistent")
            self.assertEqual(len(out_csv.read_text().splitlines()), 403)  # header + all matched epochs
            self.assertIn("FIXED,H,5.33", out_bins.read_text())
            if not out_json.with_name("x.png").exists():
                self.assertIn("matplotlib", result.stderr)

    def test_dispatcher_registers_command(self) -> None:
        result = subprocess.run(
            [sys.executable, str(ROOT_DIR / "apps" / "gnss.py"), "uncertainty-eval", "--help"],
            capture_output=True, text=True, env=SUBPROCESS_ENV,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("NOT a certified", " ".join(result.stdout.split()))


if __name__ == "__main__":
    unittest.main()
