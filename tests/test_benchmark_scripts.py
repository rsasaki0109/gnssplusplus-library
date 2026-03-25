#!/usr/bin/env python3
"""Regression tests for benchmark and reporting scripts."""

from __future__ import annotations

import argparse
import csv
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


ROOT_DIR = Path(__file__).resolve().parents[1]
APPS_DIR = ROOT_DIR / "apps"
SCRIPTS_DIR = ROOT_DIR / "scripts"

sys.path.insert(0, str(APPS_DIR))
sys.path.insert(0, str(SCRIPTS_DIR))

import gnss_odaiba_benchmark as benchmark  # noqa: E402
import generate_driving_comparison as comparison  # noqa: E402
import generate_odaiba_scorecard as scorecard  # noqa: E402


class ScorecardHelpersTest(unittest.TestCase):
    def test_ratio_text_handles_zero_baseline(self) -> None:
        self.assertEqual(scorecard.ratio_text(0.0, 0.0), "1.0x")
        self.assertEqual(scorecard.ratio_text(5.0, 0.0), "inf")
        self.assertEqual(scorecard.ratio_text(6.0, 3.0), "2.0x")

    def test_improvement_text_handles_zero_baseline(self) -> None:
        self.assertEqual(scorecard.improvement_text(1.0, 0.0), "n/a")
        self.assertEqual(scorecard.improvement_text(1.0, 4.0), "75%")


class DrivingComparisonHelpersTest(unittest.TestCase):
    @staticmethod
    def matched_epoch(tow: float, horiz_error_m: float, up_m: float, status: int) -> comparison.MatchedEpoch:
        return comparison.MatchedEpoch(
            tow=tow,
            traj_east_m=tow,
            traj_north_m=0.0,
            traj_up_m=0.0,
            east_m=horiz_error_m,
            north_m=0.0,
            up_m=up_m,
            horiz_error_m=horiz_error_m,
            status=status,
        )

    def test_pair_epochs_builds_unique_common_epoch_pairs(self) -> None:
        lib = [
            self.matched_epoch(0.0, 0.7, 0.4, 4),
            self.matched_epoch(1.0, 0.8, 0.3, 3),
            self.matched_epoch(2.0, 0.9, 0.2, 3),
            self.matched_epoch(4.0, 1.0, 0.1, 1),
        ]
        rtklib = [
            self.matched_epoch(1.0, 0.6, 0.5, 1),
            self.matched_epoch(2.0, 0.7, 0.6, 2),
            self.matched_epoch(3.0, 0.8, 0.7, 5),
        ]

        pairs = comparison.pair_epochs(lib, rtklib, tolerance_s=0.11)

        self.assertEqual([round(pair.tow, 1) for pair in pairs], [1.0, 2.0])

        lib_common, rt_common = comparison.summarize_common_epochs(
            pairs,
            lib_fixed_status=4,
            rtklib_fixed_status=1,
        )
        self.assertEqual(lib_common["epochs"], 2)
        self.assertEqual(rt_common["epochs"], 2)
        self.assertAlmostEqual(lib_common["median_h_m"], 0.85)
        self.assertAlmostEqual(rt_common["median_h_m"], 0.65)


class ScorecardRenderTest(unittest.TestCase):
    def write_reference_csv(
        self,
        path: Path,
        rows: list[tuple[int, float, float, float, float]],
    ) -> None:
        with path.open("w", newline="", encoding="ascii") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                [
                    "gps_tow_s",
                    "gps_week",
                    "lat_deg",
                    "lon_deg",
                    "height_m",
                    "ecef_x_m",
                    "ecef_y_m",
                    "ecef_z_m",
                ]
            )
            for week, tow, lat, lon, height in rows:
                ecef = comparison.llh_to_ecef(lat, lon, height)
                writer.writerow(
                    [tow, week, lat, lon, height, ecef[0], ecef[1], ecef[2]]
                )

    def write_lib_pos(
        self,
        path: Path,
        rows: list[tuple[int, float, float, float, float, int]],
    ) -> None:
        with path.open("w", encoding="ascii") as handle:
            handle.write("% synthetic libgnss++ solution\n")
            for week, tow, lat, lon, height, status in rows:
                ecef = comparison.llh_to_ecef(lat, lon, height)
                handle.write(
                    f"{week} {tow:.1f} {ecef[0]:.6f} {ecef[1]:.6f} {ecef[2]:.6f} "
                    f"{lat:.9f} {lon:.9f} {height:.4f} {status} 10 1.0\n"
                )

    def write_rtklib_pos(
        self,
        path: Path,
        rows: list[tuple[int, float, float, float, float, int]],
    ) -> None:
        with path.open("w", encoding="ascii") as handle:
            handle.write("% synthetic rtklib solution\n")
            for week, tow, lat, lon, height, status in rows:
                handle.write(
                    f"{week} {tow:.1f} {lat:.9f} {lon:.9f} {height:.4f} {status} 10\n"
                )

    def test_scorecard_main_renders_png_with_zero_fix_baseline(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_scorecard_test_") as temp_dir:
            temp_root = Path(temp_dir)
            reference_csv = temp_root / "reference.csv"
            lib_pos = temp_root / "lib.pos"
            rtklib_pos = temp_root / "rtklib.pos"
            output_png = temp_root / "scorecard.png"

            rows = [
                (2000, 0.0, 35.0000000, 139.0000000, 10.0),
                (2000, 1.0, 35.0000100, 139.0000100, 10.2),
                (2000, 2.0, 35.0000200, 139.0000200, 10.4),
            ]
            self.write_reference_csv(reference_csv, rows)
            self.write_lib_pos(
                lib_pos,
                [
                    (2000, 0.0, 35.0000000, 139.0000000, 10.0, 4),
                    (2000, 1.0, 35.0000102, 139.0000102, 10.3, 4),
                    (2000, 2.0, 35.0000201, 139.0000201, 10.5, 4),
                ],
            )
            self.write_rtklib_pos(
                rtklib_pos,
                [
                    (2000, 0.0, 35.0000000, 139.0000000, 10.0, 5),
                    (2000, 1.0, 35.0000100, 139.0000100, 10.2, 5),
                    (2000, 2.0, 35.0000200, 139.0000200, 10.4, 5),
                ],
            )

            argv = [
                "generate_odaiba_scorecard.py",
                "--lib-pos",
                str(lib_pos),
                "--rtklib-pos",
                str(rtklib_pos),
                "--reference-csv",
                str(reference_csv),
                "--output",
                str(output_png),
                "--title",
                "Synthetic Odaiba",
            ]
            with mock.patch.object(sys, "argv", argv):
                with mock.patch.dict(os.environ, {"MPLBACKEND": "Agg"}, clear=False):
                    scorecard.main()

            self.assertTrue(output_png.exists())
            self.assertGreater(output_png.stat().st_size, 0)


class SegmentedBenchmarkTest(unittest.TestCase):
    def write_reference_csv(self, path: Path, tows: list[float]) -> None:
        with path.open("w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(["gps_tow_s", "lat_deg", "lon_deg", "height_m"])
            for tow in tows:
                writer.writerow([tow, 0.0, 0.0, 0.0])

    def test_segmented_merge_uses_raw_child_solves(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_benchmark_test_") as temp_dir:
            temp_root = Path(temp_dir)
            reference_csv = temp_root / "reference.csv"
            self.write_reference_csv(reference_csv, [0.0, 1.0, 2.0, 3.0, 4.0])

            args = argparse.Namespace(
                reference_csv=reference_csv,
                segment_epochs=2,
                warmup_epochs=1,
                jobs=1,
                rover=temp_root / "rover.obs",
                base=temp_root / "base.obs",
                nav=temp_root / "base.nav",
                lib_pos=temp_root / "merged.pos",
                lib_kml=temp_root / "merged.kml",
                mode="kinematic",
                glonass_ar="autocal",
                scorecard_title="Test Title",
            )
            dispatcher = temp_root / "dispatcher.py"
            dispatcher.write_text("#!/usr/bin/env python3\n", encoding="ascii")

            solve_commands: list[list[str]] = []
            pos2kml_commands: list[list[str]] = []

            def fake_subprocess_run(command: list[str], check: bool) -> None:
                self.assertTrue(check)
                solve_commands.append(command)
                out_path = Path(command[command.index("--out") + 1])
                skip_epochs = int(command[command.index("--skip-epochs") + 1])
                max_epochs = int(command[command.index("--max-epochs") + 1])
                end_epoch = min(skip_epochs + max_epochs, 5)
                with out_path.open("w", encoding="ascii") as handle:
                    handle.write("% synthetic segment\n")
                    for tow in range(skip_epochs, end_epoch):
                        handle.write(
                            f"2000 {float(tow):.1f} 0 0 0 0 0 0 2 8 1.0\n"
                        )

            def fake_run_command(command: list[str]) -> None:
                pos2kml_commands.append(command)

            with mock.patch.object(benchmark.subprocess, "run", side_effect=fake_subprocess_run):
                with mock.patch.object(benchmark, "run_command", side_effect=fake_run_command):
                    benchmark.run_segmented_lib_solve(args, dispatcher)

            self.assertEqual(len(solve_commands), 3)
            for command in solve_commands:
                self.assertIn("--no-kinematic-post-filter", command)

            self.assertEqual(len(pos2kml_commands), 1)
            self.assertEqual(pos2kml_commands[0][1], str(dispatcher))
            self.assertEqual(pos2kml_commands[0][2], "pos2kml")

            merged_lines = [
                line.strip()
                for line in args.lib_pos.read_text(encoding="ascii").splitlines()
                if line.strip() and not line.startswith("%")
            ]
            self.assertEqual(len(merged_lines), 5)
            merged_tows = [float(line.split()[1]) for line in merged_lines]
            self.assertEqual(merged_tows, [0.0, 1.0, 2.0, 3.0, 4.0])

    def make_benchmark_args(self, temp_root: Path, **overrides: object) -> argparse.Namespace:
        paths = {
            "rover": temp_root / "rover.obs",
            "base": temp_root / "base.obs",
            "nav": temp_root / "base.nav",
            "reference_csv": temp_root / "reference.csv",
            "rtklib_config": temp_root / "rtklib.conf",
            "rtklib_bin": temp_root / "rnx2rtkp",
            "lib_pos": temp_root / "lib.pos",
            "lib_kml": temp_root / "lib.kml",
            "rtklib_pos": temp_root / "rtklib.pos",
            "comparison_png": temp_root / "comparison.png",
            "scorecard_png": temp_root / "scorecard.png",
        }
        for path in paths.values():
            path.write_text("synthetic\n", encoding="ascii")
        defaults: dict[str, object] = {
            **paths,
            "comparison_title": "Comparison",
            "scorecard_title": "Scorecard",
            "mode": "kinematic",
            "glonass_ar": "autocal",
            "skip_epochs": 0,
            "max_epochs": -1,
            "segment_epochs": 0,
            "warmup_epochs": 300,
            "jobs": 4,
        }
        defaults.update(overrides)
        return argparse.Namespace(**defaults)

    def test_main_partial_window_skips_segmented_and_downstream_pipeline(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_benchmark_main_") as temp_dir:
            temp_root = Path(temp_dir)
            args = self.make_benchmark_args(
                temp_root,
                skip_epochs=12,
                max_epochs=34,
                segment_epochs=100,
            )

            commands: list[list[str]] = []
            with mock.patch.object(benchmark, "parse_args", return_value=args):
                with mock.patch.object(benchmark, "run_segmented_lib_solve") as segmented:
                    with mock.patch.object(
                        benchmark, "run_command", side_effect=commands.append
                    ):
                        exit_code = benchmark.main()

            self.assertEqual(exit_code, 0)
            segmented.assert_not_called()
            self.assertEqual(len(commands), 1)
            solve_command = commands[0]
            self.assertEqual(solve_command[2], "solve")
            self.assertIn("--no-kml", solve_command)
            self.assertIn("--skip-epochs", solve_command)
            self.assertIn("--max-epochs", solve_command)

    def test_main_full_segmented_run_uses_segment_solver_then_reports(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_benchmark_main_") as temp_dir:
            temp_root = Path(temp_dir)
            args = self.make_benchmark_args(
                temp_root,
                segment_epochs=500,
                warmup_epochs=50,
                jobs=2,
            )

            commands: list[list[str]] = []
            with mock.patch.object(benchmark, "parse_args", return_value=args):
                with mock.patch.object(benchmark, "run_segmented_lib_solve") as segmented:
                    with mock.patch.object(
                        benchmark, "run_command", side_effect=commands.append
                    ):
                        exit_code = benchmark.main()

            self.assertEqual(exit_code, 0)
            segmented.assert_called_once()
            self.assertEqual(len(commands), 3)
            self.assertEqual(commands[0][0], str(args.rtklib_bin))
            self.assertEqual(commands[1][2], "driving-compare")
            self.assertEqual(commands[2][2], "scorecard")


if __name__ == "__main__":
    unittest.main()
