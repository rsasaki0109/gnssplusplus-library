#!/usr/bin/env python3

import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
SCRIPTS_DIR = ROOT_DIR / "scripts"
PPC_SCRIPTS_DIR = SCRIPTS_DIR / "experiments" / "ppc"
for script_dir in (SCRIPTS_DIR, PPC_SCRIPTS_DIR):
    if str(script_dir) not in sys.path:
        sys.path.insert(0, str(script_dir))

import generate_ppc_clas_scorecard as scorecard  # noqa: E402
import generate_ppc_clas_full_comparison as full_comparison  # noqa: E402
import run_ppc_clas_candidate as candidate_runner  # noqa: E402


class PpcClasLeverArmTest(unittest.TestCase):
    def test_candidate_artifact_hash_is_reproducible(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            artifact = Path(temp_dir) / "artifact.bin"
            artifact.write_bytes(b"abc")
            expected = (
                "BA7816BF8F01CFEA414140DE5DAE2223"
                "B00361A396177A9CB410FF61F20015AD"
            )
            self.assertEqual(candidate_runner.sha256_file(artifact), expected)
            self.assertEqual(full_comparison.sha256_file(artifact), expected)

    def test_candidate_manifest_environment_is_merged_into_full_report(self) -> None:
        manifest = {
            "parity_environment": {
                "GNSS_PPP_CLAS_AR_HELD_MIN_DD_ROWS": "4",
                "GNSS_PPP_CLAS_AR_HELD_MAX_PUBLICATION_STREAK": "5",
            }
        }
        merged = full_comparison.merged_parity_environment(manifest)
        self.assertEqual(
            merged["GNSS_PPP_CLAS_BASE_CLOCK_PARITY"],
            scorecard.PARITY_ENV["GNSS_PPP_CLAS_BASE_CLOCK_PARITY"],
        )
        self.assertEqual(merged["GNSS_PPP_CLAS_AR_HELD_MIN_DD_ROWS"], "4")
        self.assertEqual(
            merged["GNSS_PPP_CLAS_AR_HELD_MAX_PUBLICATION_STREAK"], "5"
        )

    def test_candidate_manifest_provenance_omits_machine_local_paths(self) -> None:
        manifest = {
            "dataset_root": r"E:\private\PPC-Dataset",
            "ssr_root": r"E:\private\ssr",
            "gnss_ppp": r"C:\Users\person\build\gnss_ppp.exe",
            "gnss_ppp_sha256": "ABC123",
            "source": {"revision": "deadbeef", "worktree_dirty": True},
            "held_min_dd_rows": 4,
            "held_max_publication_streak": 5,
            "parity_environment": {"GNSS_PPP_CLAS_AR_HELD_MIN_DD_ROWS": "4"},
            "runs": {
                "tokyo_run1": {
                    "command": [r"C:\Users\person\build\gnss_ppp.exe"],
                    "pos": r"E:\private\tokyo_run1.pos",
                    "log": r"E:\private\tokyo_run1.log",
                    "pos_size_bytes": 1234,
                    "pos_sha256": "DEF456",
                }
            },
        }

        portable = full_comparison.portable_candidate_manifest(manifest)

        self.assertEqual(portable["gnss_ppp_sha256"], "ABC123")
        self.assertEqual(
            portable["runs"]["tokyo_run1"],
            {"pos_size_bytes": 1234, "pos_sha256": "DEF456"},
        )
        serialized = str(portable)
        self.assertNotIn(r"C:\Users", serialized)
        self.assertNotIn(r"E:\private", serialized)

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
                    "key": key,
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
        signoff = full_comparison.evaluate_mrtklib_signoff(runs)
        with tempfile.TemporaryDirectory() as temp_dir:
            output = Path(temp_dir) / "table.md"
            full_comparison.write_markdown_table(
                runs, aggregate, signoff, output
            )
            markdown = output.read_text(encoding="utf-8")

        self.assertEqual(markdown.count("| Tokyo "), 6)
        self.assertEqual(markdown.count("| Nagoya "), 6)
        self.assertIn("100.000% / 99.900%", markdown)
        self.assertIn("raw PPC reference point", markdown)
        self.assertIn("directly comparable", markdown)
        self.assertIn(">3 m FIX", markdown)
        self.assertIn("FLOAT RMS2D", markdown)
        self.assertIn("SINGLE RMS2D", markdown)
        self.assertIn("—", markdown)

        with tempfile.TemporaryDirectory() as temp_dir:
            output = Path(temp_dir) / "table.md"
            full_comparison.write_markdown_table(
                runs, aggregate, signoff, output, apply_lever_arm=True
            )
            markdown = output.read_text(encoding="utf-8")

        self.assertIn("vehicle truth transformed to the antenna phase center", markdown)
        self.assertIn("unmodified PPC reference point", markdown)
        table_sections = markdown.split("## MRTKLIB v0.4.2 sign-off", 1)
        self.assertEqual(len(table_sections), 2)
        for section in table_sections:
            rows = [
                line for line in section.splitlines() if line.startswith("|")
            ]
            expected_columns = rows[0].count("|")
            self.assertTrue(
                all(line.count("|") == expected_columns for line in rows)
            )

    def test_mrtklib_signoff_requires_full_coverage(self) -> None:
        metrics = {
            "interval_coverage_pct": 100.0,
            "epoch_coverage_pct": 99.0,
            "fix_pct": 10.0,
            "rms2d_fixed_m": 0.2,
            "fixed_over_3m": 0,
            "p68_fixed_m": 0.2,
            "ttff_30_s": 1.0,
        }
        target = {
            "fix_pct": 9.0,
            "rms2d_m": 0.3,
            "sigma2d_m": 0.3,
            "ttff_s": 2.0,
        }
        run = {
            "key": "probe",
            "metrics": metrics,
            "mrtklib_v0_4_2": target,
        }

        verdict = full_comparison.evaluate_mrtklib_signoff([run])
        self.assertTrue(verdict["hard_pass"])

        metrics["epoch_coverage_pct"] = 98.999
        verdict = full_comparison.evaluate_mrtklib_signoff([run])
        self.assertFalse(verdict["hard_pass"])
        self.assertFalse(
            verdict["runs"]["probe"]["hard_gates"][
                "epoch_coverage_at_least_99pct"
            ]
        )

        hard_failures = {
            "interval_coverage_at_least_99pct": (
                "interval_coverage_pct", 98.999
            ),
            "fix_rate_at_least_mrtklib": ("fix_pct", 8.999),
            "fix_rms2d_at_most_mrtklib": ("rms2d_fixed_m", 0.301),
            "fixed_over_3m_is_zero": ("fixed_over_3m", 1),
        }
        for gate, (field, failing_value) in hard_failures.items():
            with self.subTest(gate=gate):
                probe_metrics = {
                    **metrics,
                    "interval_coverage_pct": 100.0,
                    "epoch_coverage_pct": 100.0,
                    field: failing_value,
                }
                run["metrics"] = probe_metrics
                verdict = full_comparison.evaluate_mrtklib_signoff([run])
                self.assertFalse(verdict["hard_pass"])
                self.assertFalse(
                    verdict["runs"]["probe"]["hard_gates"][gate]
                )

        for gate, field, failing_value in (
            ("fix_p68_at_most_mrtklib", "p68_fixed_m", 0.301),
            ("ttff_at_most_mrtklib", "ttff_30_s", 2.001),
        ):
            with self.subTest(gate=gate):
                run["metrics"] = {
                    **metrics,
                    "interval_coverage_pct": 100.0,
                    "epoch_coverage_pct": 100.0,
                    field: failing_value,
                }
                verdict = full_comparison.evaluate_mrtklib_signoff([run])
                self.assertTrue(verdict["hard_pass"])
                self.assertFalse(verdict["soft_pass"])
                self.assertFalse(
                    verdict["runs"]["probe"]["soft_gates"][gate]
                )

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
