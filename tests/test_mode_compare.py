#!/usr/bin/env python3
"""Tests for the SPP/FGO/RTK mode comparison runner."""

from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT_DIR = Path(__file__).resolve().parents[1]
APPS_DIR = ROOT_DIR / "apps"
sys.path.insert(0, str(APPS_DIR))

import gnss_mode_compare  # noqa: E402


class ModeCompareTest(unittest.TestCase):
    def write_pos(self, path: Path, rows: list[tuple[float, float, int, int, float]]) -> None:
        lines = [
            "% LibGNSS++ Position Solution\n",
            "% Columns: GPS_Week GPS_TOW X(m) Y(m) Z(m) Lat(deg) Lon(deg) Height(m) Status Satellites PDOP Ratio\n",
        ]
        for tow, x, status, satellites, ratio in rows:
            lines.append(
                f"2200 {tow:.3f} {x:.4f} 2.0000 3.0000 "
                f"35.000000000 139.000000000 10.0000 {status} {satellites} 1.20 {ratio:.1f}\n"
            )
        path.write_text("".join(lines), encoding="ascii")

    def write_reference_csv(self, path: Path) -> None:
        path.write_text(
            "\n".join(
                [
                    "GPS TOW (s),GPS Week,Latitude (deg),Longitude (deg),Ellipsoid Height (m),ECEF X (m),ECEF Y (m),ECEF Z (m),Roll (deg),Pitch (deg),Heading (deg),East Velocity (m/s),North Velocity (m/s),Up Velocity (m/s)",
                    "10.000, 2200, 0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 0, 0, 0, 0, 0, 0",
                    "11.000, 2200, 0.0, 0.0, 0.0, 2.0, 2.3, 3.4, 0, 0, 0, 0, 0, 0",
                    "",
                ]
            ),
            encoding="ascii",
        )

    def test_summarizes_pos_records_and_pairwise_deltas(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_mode_compare_pos_test_") as temp_dir:
            temp_root = Path(temp_dir)
            spp_path = temp_root / "spp.pos"
            fgo_path = temp_root / "fgo.pos"
            self.write_pos(
                spp_path,
                [
                    (10.000, 1.0, 1, 7, 0.0),
                    (11.000, 2.0, 1, 8, 0.0),
                ],
            )
            self.write_pos(
                fgo_path,
                [
                    (10.004, 1.5, 3, 9, 2.5),
                    (11.004, 3.0, 4, 10, 4.0),
                ],
            )

            spp_records = gnss_mode_compare.read_pos_records(spp_path)
            fgo_records = gnss_mode_compare.read_pos_records(fgo_path)
            spp_summary = gnss_mode_compare.summarize_records(spp_records)
            pair_summary = gnss_mode_compare.summarize_pairwise(
                "spp",
                spp_records,
                "fgo",
                fgo_records,
                tolerance_s=0.01,
            )

            self.assertEqual(spp_summary["epochs"], 2)
            self.assertEqual(spp_summary["spp_epochs"], 2)
            self.assertEqual(spp_summary["status_counts"], {"SPP": 2})
            self.assertEqual(pair_summary["common_epochs"], 2)
            self.assertAlmostEqual(float(pair_summary["mean_3d_delta_m"]), 0.75)
            self.assertAlmostEqual(float(pair_summary["max_3d_delta_m"]), 1.0)

    def test_run_plan_reports_solver_wall_time(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_mode_compare_wall_time_test_") as temp_dir:
            temp_root = Path(temp_dir)
            plan = gnss_mode_compare.ModePlan(
                name="noop",
                pos_path=temp_root / "noop.pos",
                summary_path=None,
                command=[sys.executable, "-c", "print('ok')"],
            )

            completed, solver_wall_time_s = gnss_mode_compare.run_plan(plan, keep_going=False)

            self.assertEqual(completed.returncode, 0)
            self.assertGreater(solver_wall_time_s, 0.0)

    def test_dry_run_writes_planned_auto_mode_summary(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_mode_compare_dry_run_test_") as temp_dir:
            temp_root = Path(temp_dir)
            obs_path = temp_root / "rover.obs"
            nav_path = temp_root / "nav.rnx"
            base_path = temp_root / "base.obs"
            for path in (obs_path, nav_path, base_path):
                path.write_text("", encoding="ascii")
            summary_path = temp_root / "summary.json"

            result = subprocess.run(
                [
                    sys.executable,
                    str(APPS_DIR / "gnss_mode_compare.py"),
                    "--obs",
                    str(obs_path),
                    "--nav",
                    str(nav_path),
                    "--base",
                    str(base_path),
                    "--out-dir",
                    str(temp_root / "out"),
                    "--summary-json",
                    str(summary_path),
                    "--fgo-preset",
                    "taroz-pc",
                    "--fgo-skip-epochs",
                    "7",
                    "--max-epochs",
                    "5",
                    "--dry-run",
                ],
                cwd=ROOT_DIR,
                text=True,
                capture_output=True,
                check=False,
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(set(payload["modes"]), {"spp", "fgo", "rtk"})
            self.assertEqual(payload["modes"]["spp"]["status"], "planned")
            self.assertEqual(payload["fgo_skip_epochs"], 7)
            self.assertIsNone(payload["modes"]["spp"]["solver_wall_time_s"])
            self.assertIn("fgo", payload["modes"]["fgo"]["command"])
            self.assertIn("--summary-json", payload["modes"]["fgo"]["command"])
            self.assertIn("--preset", payload["modes"]["fgo"]["command"])
            self.assertIn("taroz-pc", payload["modes"]["fgo"]["command"])
            self.assertIn("--backend", payload["modes"]["fgo"]["command"])
            self.assertIn("gtsam-pc", payload["modes"]["fgo"]["command"])
            self.assertIn("--skip-epochs", payload["modes"]["fgo"]["command"])
            self.assertIn("7", payload["modes"]["fgo"]["command"])
            self.assertIn("--base", payload["modes"]["fgo"]["command"])
            self.assertIn(str(base_path), payload["modes"]["fgo"]["command"])
            self.assertIn("solve", payload["modes"]["rtk"]["command"])
            self.assertIn("--no-kml", payload["modes"]["rtk"]["command"])

    def test_existing_pos_inputs_do_not_require_rinex_inputs(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_mode_compare_external_pos_test_") as temp_dir:
            temp_root = Path(temp_dir)
            spp_path = temp_root / "existing_spp.pos"
            fgo_path = temp_root / "existing_fgo.pos"
            fgo_summary_path = temp_root / "existing_fgo_summary.json"
            summary_path = temp_root / "summary.json"
            self.write_pos(
                spp_path,
                [
                    (10.000, 1.0, 1, 7, 0.0),
                    (11.000, 2.0, 1, 8, 0.0),
                ],
            )
            self.write_pos(
                fgo_path,
                [
                    (10.000, 1.2, 3, 9, 2.5),
                    (11.000, 2.4, 4, 10, 4.0),
                ],
            )
            fgo_summary_path.write_text(
                json.dumps({"fixed_solutions": 1, "float_solutions": 1}) + "\n",
                encoding="utf-8",
            )

            result = subprocess.run(
                [
                    sys.executable,
                    str(APPS_DIR / "gnss_mode_compare.py"),
                    "--modes",
                    "spp,fgo",
                    "--spp-pos",
                    str(spp_path),
                    "--fgo-pos",
                    str(fgo_path),
                    "--fgo-summary-json",
                    str(fgo_summary_path),
                    "--summary-json",
                    str(summary_path),
                    "--require-mode-epochs-min",
                    "spp=2",
                    "--require-mode-fix-rate-min",
                    "fgo=50",
                    "--require-pair-common-min",
                    "spp:fgo=2",
                    "--require-pair-p95-3d-max",
                    "spp:fgo=0.4",
                ],
                cwd=ROOT_DIR,
                text=True,
                capture_output=True,
                check=False,
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertIsNone(payload["obs"])
            self.assertIsNone(payload["nav"])
            self.assertEqual(payload["modes"]["spp"]["status"], "external")
            self.assertEqual(payload["modes"]["fgo"]["status"], "external")
            self.assertEqual(payload["modes"]["fgo"]["native_summary"]["fixed_solutions"], 1)
            self.assertEqual(payload["modes"]["fgo"]["summary"]["fixed_epochs"], 1)
            self.assertEqual(payload["pairwise"][0]["common_epochs"], 2)
            self.assertTrue(payload["requirements"]["passed"])
            self.assertEqual(payload["requirements"]["failures"], [])

    def test_reference_csv_adds_per_mode_reference_metrics(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_mode_compare_reference_test_") as temp_dir:
            temp_root = Path(temp_dir)
            spp_path = temp_root / "existing_spp.pos"
            fgo_path = temp_root / "existing_fgo.pos"
            reference_path = temp_root / "reference.csv"
            summary_path = temp_root / "summary.json"
            self.write_pos(
                spp_path,
                [
                    (10.000, 1.0, 1, 7, 0.0),
                    (11.000, 2.0, 1, 8, 0.0),
                ],
            )
            self.write_pos(
                fgo_path,
                [
                    (10.000, 1.0, 3, 9, 2.5),
                    (11.000, 2.0, 4, 10, 4.0),
                ],
            )
            self.write_reference_csv(reference_path)

            result = subprocess.run(
                [
                    sys.executable,
                    str(APPS_DIR / "gnss_mode_compare.py"),
                    "--modes",
                    "spp,fgo",
                    "--spp-pos",
                    str(spp_path),
                    "--fgo-pos",
                    str(fgo_path),
                    "--reference-csv",
                    str(reference_path),
                    "--summary-json",
                    str(summary_path),
                ],
                cwd=ROOT_DIR,
                text=True,
                capture_output=True,
                check=False,
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("ref_mean_h", result.stdout)
            self.assertIn("ref_3d50_matched=100.00%", result.stdout)
            self.assertIn("ref_3d50_ref=100.00%", result.stdout)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["reference_csv"], str(reference_path))
            self.assertEqual(payload["reference_match_tolerance_s"], 0.25)
            spp_reference = payload["modes"]["spp"]["reference_summary"]
            fgo_reference = payload["modes"]["fgo"]["reference_summary"]
            self.assertEqual(spp_reference["matched_epochs"], 2)
            self.assertAlmostEqual(float(spp_reference["mean_h_m"]), 0.25)
            self.assertAlmostEqual(float(spp_reference["p95_h_m"]), 0.475)
            self.assertEqual(spp_reference["ppc_score_3d_50cm_matched_pct"], 100.0)
            self.assertEqual(fgo_reference["matched_epochs"], 2)

    def test_requirement_failures_return_nonzero_and_stay_in_summary(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_mode_compare_requirements_test_") as temp_dir:
            temp_root = Path(temp_dir)
            spp_path = temp_root / "spp.pos"
            fgo_path = temp_root / "fgo.pos"
            summary_path = temp_root / "summary.json"
            self.write_pos(
                spp_path,
                [
                    (10.000, 1.0, 1, 7, 0.0),
                    (11.000, 2.0, 1, 8, 0.0),
                ],
            )
            self.write_pos(
                fgo_path,
                [
                    (10.000, 1.2, 3, 9, 2.5),
                    (11.000, 2.4, 4, 10, 4.0),
                ],
            )

            result = subprocess.run(
                [
                    sys.executable,
                    str(APPS_DIR / "gnss_mode_compare.py"),
                    "--modes",
                    "spp,fgo",
                    "--spp-pos",
                    str(spp_path),
                    "--fgo-pos",
                    str(fgo_path),
                    "--summary-json",
                    str(summary_path),
                    "--require-mode-fix-rate-min",
                    "fgo=75",
                    "--require-pair-p95-3d-max",
                    "spp:fgo=0.1",
                ],
                cwd=ROOT_DIR,
                text=True,
                capture_output=True,
                check=False,
            )

            self.assertEqual(result.returncode, 2)
            self.assertIn("Requirement checks failed", result.stderr)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertFalse(payload["requirements"]["passed"])
            self.assertEqual(len(payload["requirements"]["failures"]), 2)
            self.assertIn("fgo=75", payload["requirements"]["failures"][0])


if __name__ == "__main__":
    unittest.main()
