#!/usr/bin/env python3

import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = ROOT_DIR / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import generate_ppc_clas_scorecard as scorecard  # noqa: E402
import generate_ppc_clas_full_comparison as full_comparison  # noqa: E402


class PpcClasLeverArmTest(unittest.TestCase):
    def test_level_north_facing_body_axes_map_from_frd_to_enu(self) -> None:
        rotation = scorecard.body_to_enu_rotation_matrix(0.0, 0.0, 0.0)
        np.testing.assert_allclose(rotation @ np.array([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0])
        np.testing.assert_allclose(rotation @ np.array([0.0, 1.0, 0.0]), [1.0, 0.0, 0.0])
        np.testing.assert_allclose(rotation @ np.array([0.0, 0.0, 1.0]), [0.0, 0.0, -1.0])

    def test_level_east_facing_forward_axis_maps_to_east(self) -> None:
        rotation = scorecard.body_to_enu_rotation_matrix(0.0, 0.0, 90.0)
        np.testing.assert_allclose(
            rotation @ np.array([1.0, 0.0, 0.0]), [1.0, 0.0, 0.0], atol=1e-12
        )

    def test_rotation_preserves_lever_arm_norm(self) -> None:
        rotation = scorecard.body_to_enu_rotation_matrix(-1.4, 3.2, 312.7)
        np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-12)
        for city in ("tokyo", "nagoya"):
            lever = scorecard.lever_arm_for_city(city)
            self.assertAlmostEqual(np.linalg.norm(rotation @ lever), np.linalg.norm(lever))

    def test_lever_arm_requires_known_city(self) -> None:
        with self.assertRaisesRegex(ValueError, "Unknown PPC city"):
            scorecard.lever_arm_for_city("tokoyo")
        with self.assertRaisesRegex(ValueError, "city is required"):
            scorecard.read_reference_csv(Path("unused.csv"))

    def test_reference_reader_applies_city_specific_vehicle_to_antenna_arm(self) -> None:
        header = (
            "GPS TOW (s),GPS Week,Latitude (deg),Longitude (deg),"
            "Ellipsoid Height (m),ECEF X (m),ECEF Y (m),ECEF Z (m),"
            "Roll (deg),Pitch (deg),Heading (deg)\n"
        )
        row = "100.0,2300,0.0,0.0,0.0,6378137.0,0.0,0.0,0.0,0.0,0.0\n"
        with tempfile.TemporaryDirectory() as temp_dir:
            reference_csv = Path(temp_dir) / "reference.csv"
            reference_csv.write_text(header + row, encoding="utf-8")
            tokyo = scorecard.read_reference_csv(
                reference_csv, apply_lever_arm=True, city="tokyo"
            )[0]
            nagoya = scorecard.read_reference_csv(
                reference_csv, apply_lever_arm=True, city="nagoya"
            )[0]

        np.testing.assert_allclose(
            tokyo.ecef - np.array([6378137.0, 0.0, 0.0]),
            [0.55, 0.0, 0.31],
            atol=1e-9,
        )
        np.testing.assert_allclose(
            nagoya.ecef - np.array([6378137.0, 0.0, 0.0]),
            [1.216, -0.670, 0.593],
            atol=1e-9,
        )

    def test_full_comparison_markdown_records_reference_difference(self) -> None:
        runs = []
        for key, _label in full_comparison.RUNS:
            runs.append(
                {
                    "metrics": {
                        "interval_coverage_pct": 100.0,
                        "epoch_coverage_pct": 99.9,
                        "fix_pct": 12.5,
                        "rms2d_fixed_m": 0.25,
                        "p68_fixed_m": 0.20,
                        "rms2d_all_m": 4.0,
                        "rms2d_float_m": 1.5,
                        "rms2d_single_m": 8.0,
                        "max_fixed_m": 0.75,
                        "fixed_over_3m": 0,
                        "ttff_30_s": None,
                    },
                    "mrtklib_v0_4_2": scorecard.MRTKLIB_TARGETS[key],
                }
            )
        aggregate = {
            "fix_pct": 0.0,
            "rms2d_fixed_m": None,
            "p68_fixed_m": None,
            "rms2d_all_m": 4.0,
            "rms2d_float_m": 1.5,
            "rms2d_single_m": 8.0,
            "max_fixed_m": None,
            "fixed_over_3m": 0,
        }
        with tempfile.TemporaryDirectory() as temp_dir:
            output = Path(temp_dir) / "table.md"
            full_comparison.write_markdown_table(runs, aggregate, output)
            markdown = output.read_text(encoding="utf-8")

        self.assertEqual(markdown.count("| Tokyo "), 3)
        self.assertEqual(markdown.count("| Nagoya "), 3)
        self.assertIn("100.000% / 99.900%", markdown)
        self.assertIn("vehicle truth transformed to the antenna phase center", markdown)
        self.assertIn("unmodified PPC reference point", markdown)
        self.assertIn(">3 m FIX", markdown)
        self.assertIn("FLOAT RMS2D", markdown)
        self.assertIn("SINGLE RMS2D", markdown)
        self.assertIn("—", markdown)
        rows = [line for line in markdown.splitlines() if line.startswith("|")]
        expected_columns = rows[0].count("|")
        self.assertTrue(all(line.count("|") == expected_columns for line in rows))

    def test_write_report_with_only_parity_config_does_not_raise(self) -> None:
        run_key = "tokyo_run2"
        metrics = {
            "matched_epochs": 100,
            "fixed_epochs": 20,
            "fix_pct": 20.0,
            "rms2d_fixed_m": 0.5,
            "sigma2d_fixed_m": 0.3,
            "rms2d_all_m": 1.2,
            "ttff_s": 30.0,
            "mean_satellites": 9.0,
            "status_counts": {6: 20, 5: 80},
            "notes": [],
        }
        run_results = [
            {
                "run_key": run_key,
                "gps_week": 2300,
                "gps_tow_start": 100.0,
                "gps_tow_end": 200.0,
                "utc_start": "2024-01-01 00:00:00",
                "utc_end": "2024-01-01 00:01:40",
                "doy": 1,
                "year": 2024,
                "l6_slots": ["slot0"],
                "ssr_rows": 42,
                "configs": {"parity": metrics},
            }
        ]
        with tempfile.TemporaryDirectory() as temp_dir:
            report_path = Path(temp_dir) / "report.md"
            scorecard.write_report(
                report_path,
                l6_source="cache",
                csv_recipe="recipe",
                command_template="gnss_ppp --ssr foo.csv",
                run_results=run_results,
                work_dir=Path(temp_dir),
                l6_cache=Path(temp_dir) / "l6_cache",
            )
            markdown = report_path.read_text(encoding="utf-8")

        self.assertIn("## Scorecard — parity stack", markdown)
        self.assertNotIn("## Scorecard — default (no env gates)", markdown)
        self.assertIn(run_key, markdown)

    def test_full_coverage_rejects_missing_middle_epochs(self) -> None:
        with self.assertRaisesRegex(SystemExit, "matched epochs 50 of 100"):
            full_comparison.validate_coverage(
                "tokyo_run1",
                reference_duration=100.0,
                matched_duration=100.0,
                matched_epochs=50,
                observation_epochs=100,
            )

        interval, epochs = full_comparison.validate_coverage(
            "tokyo_run1",
            reference_duration=100.0,
            matched_duration=99.0,
            matched_epochs=99,
            observation_epochs=100,
        )
        self.assertEqual(interval, 0.99)
        self.assertEqual(epochs, 0.99)

    def test_min_epoch_coverage_override_does_not_relax_interval_coverage(self) -> None:
        # Epoch coverage below the default threshold but above an explicit
        # override should pass, while interval coverage still uses the
        # module-level MIN_INTERVAL_COVERAGE regardless of the override.
        interval, epochs = full_comparison.validate_coverage(
            "tokyo_run1",
            reference_duration=100.0,
            matched_duration=100.0,
            matched_epochs=94,
            observation_epochs=100,
            min_epoch_coverage=0.93,
        )
        self.assertEqual(interval, 1.0)
        self.assertEqual(epochs, 0.94)

        with self.assertRaisesRegex(SystemExit, "matched duration 90.0s"):
            full_comparison.validate_coverage(
                "tokyo_run1",
                reference_duration=100.0,
                matched_duration=90.0,
                matched_epochs=94,
                observation_epochs=100,
                min_epoch_coverage=0.93,
            )


if __name__ == "__main__":
    unittest.main()
