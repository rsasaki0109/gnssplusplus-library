#!/usr/bin/env python3
"""Regression tests for benchmark and reporting scripts."""

from __future__ import annotations

import argparse
import csv
from datetime import timedelta
import json
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
import gnss_clas_ppp as clas_ppp  # noqa: E402
import gnss_live_signoff as live_signoff  # noqa: E402
import gnss_ppc_demo as ppc_demo  # noqa: E402
import gnss_ppc_rtk_signoff as ppc_rtk_signoff  # noqa: E402
import gnss_ppp_kinematic_signoff as ppp_kinematic_signoff  # noqa: E402
import gnss_ppp_static_signoff as ppp_static_signoff  # noqa: E402
import gnss_short_baseline_signoff as short_signoff  # noqa: E402
import generate_driving_comparison as comparison  # noqa: E402
import generate_feature_overview_card as feature_overview  # noqa: E402
import generate_odaiba_scorecard as scorecard  # noqa: E402
import generate_odaiba_social_card as social_card  # noqa: E402


class ScorecardHelpersTest(unittest.TestCase):
    def test_ratio_text_handles_zero_baseline(self) -> None:
        self.assertEqual(scorecard.ratio_text(0.0, 0.0), "1.0x")
        self.assertEqual(scorecard.ratio_text(5.0, 0.0), "inf")
        self.assertEqual(scorecard.ratio_text(6.0, 3.0), "2.0x")

    def test_improvement_text_handles_zero_baseline(self) -> None:
        self.assertEqual(scorecard.improvement_text(1.0, 0.0), "n/a")
        self.assertEqual(scorecard.improvement_text(1.0, 4.0), "75%")


class ClasCompactHelpersTest(unittest.TestCase):
    def test_expand_compact_ssr_text_merges_high_rate_clock_and_system_tokens(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_compact_") as temp_dir:
            output_csv = Path(temp_dir) / "expanded.csv"
            payload = clas_ppp.expand_compact_ssr_text(
                "\n".join(
                    [
                        "# week,tow,system,prn,dx,dy,dz,dclock_m,high_rate_clock_m",
                        "2200,345600.0,G,3,0.1,0.2,0.3,0.4,0.05",
                        "2200,345600.0,QZSS,3,0.0,0.0,0.0,0.1",
                    ]
                ),
                output_csv,
            )

            self.assertEqual(payload["rows_written"], 2)
            self.assertEqual(payload["systems"], ["G", "J"])
            lines = output_csv.read_text(encoding="ascii").splitlines()
            self.assertEqual(lines[0], "# week,tow,sat,dx,dy,dz,dclock_m")
            self.assertIn("2200,345600.000,G03,0.100000,0.200000,0.300000,0.450000", lines[1])
            self.assertIn("2200,345600.000,J03,0.000000,0.000000,0.000000,0.100000", lines[2])

    def test_expand_compact_ssr_text_preserves_optional_ura_code_and_phase_bias_tokens(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_compact_meta_") as temp_dir:
            output_csv = Path(temp_dir) / "expanded.csv"
            clas_ppp.expand_compact_ssr_text(
                "\n".join(
                    [
                        "# week,tow,system,prn,dx,dy,dz,dclock_m,high_rate_clock_m,ura_sigma_m=<m>,cbias:<id>=<m>,pbias:<id>=<m>",
                        "2200,345600.0,G,3,0.1,0.2,0.3,0.4,0.05,ura_sigma_m=0.002750,cbias:2=-0.120000,pbias:2=0.015000",
                    ]
                ),
                output_csv,
            )

            lines = output_csv.read_text(encoding="ascii").splitlines()
            self.assertEqual(lines[0], "# week,tow,sat,dx,dy,dz,dclock_m")
            self.assertEqual(
                lines[1],
                "2200,345600.000,G03,0.100000,0.200000,0.300000,0.450000,ura_sigma_m=0.002750,cbias:2=-0.120000,pbias:2=0.015000",
            )

    def test_expand_compact_ssr_text_preserves_atmos_metadata_tokens(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_compact_atmos_") as temp_dir:
            output_csv = Path(temp_dir) / "expanded.csv"
            clas_ppp.expand_compact_ssr_text(
                "\n".join(
                    [
                        "# week,tow,system,prn,dx,dy,dz,dclock_m,atmos_<name>=<value>",
                        "2200,345600.0,G,3,0.1,0.2,0.3,0.4,atmos_network_id=1,atmos_trop_avail=3,atmos_stec_avail=3",
                    ]
                ),
                output_csv,
            )

            lines = output_csv.read_text(encoding="ascii").splitlines()
            self.assertEqual(
                lines[1],
                "2200,345600.000,G03,0.100000,0.200000,0.300000,0.400000,atmos_network_id=1,atmos_trop_avail=3,atmos_stec_avail=3",
            )

    def test_parse_ppp_summary_counts_extracts_atmospheric_lines(self) -> None:
        parsed = clas_ppp._parse_ppp_summary_counts(
            "\n".join(
                [
                    "PPP summary:",
                    "  valid solutions: 4",
                    "  atmospheric trop corrections: 12",
                    "  atmospheric trop meters: 5.500000",
                    "  atmospheric ionosphere corrections: 8",
                    "  atmospheric ionosphere meters: 3.250000",
                ]
            )
        )
        self.assertEqual(parsed["ppp_atmospheric_trop_corrections"], 12)
        self.assertEqual(parsed["ppp_atmospheric_ionosphere_corrections"], 8)
        self.assertAlmostEqual(float(parsed["ppp_atmospheric_trop_meters"]), 5.5)
        self.assertAlmostEqual(float(parsed["ppp_atmospheric_ionosphere_meters"]), 3.25)


class PPCRTKSignoffHelpersTest(unittest.TestCase):
    def test_selected_thresholds_keep_rtklib_gates_only_when_enabled(self) -> None:
        args = argparse.Namespace(**{name: None for name in ppc_rtk_signoff.REQUIREMENT_NAMES})

        tokyo_without_rtklib = ppc_rtk_signoff.selected_thresholds(args, "tokyo", False)
        self.assertNotIn("require_lib_fix_rate_vs_rtklib_min_delta", tokyo_without_rtklib)
        self.assertEqual(tokyo_without_rtklib["require_fix_rate_min"], 95.0)

        nagoya_with_rtklib = ppc_rtk_signoff.selected_thresholds(args, "nagoya", True)
        self.assertIn("require_lib_fix_rate_vs_rtklib_min_delta", nagoya_with_rtklib)
        self.assertEqual(nagoya_with_rtklib["require_max_h_max"], 0.60)

    def test_build_ppc_demo_command_includes_profile_thresholds_and_rtklib_flags(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_rtk_signoff_unit_") as temp_dir:
            temp_root = Path(temp_dir)
            args = argparse.Namespace(
                max_epochs=120,
                match_tolerance_s=0.25,
                use_existing_solution=True,
                solver_wall_time_s=1.5,
                rtklib_bin=temp_root / "rnx2rtkp",
                rtklib_config=temp_root / "rtklib.conf",
                rtklib_pos=temp_root / "rtklib.pos",
                use_existing_rtklib_solution=True,
                rtklib_solver_wall_time_s=0.8,
            )
            run_dir = temp_root / "tokyo" / "run1"
            out = temp_root / "solution.pos"
            summary_json = temp_root / "summary.json"
            thresholds = {
                "require_fix_rate_min": 95.0,
                "require_lib_fix_rate_vs_rtklib_min_delta": 0.0,
            }

            command = ppc_rtk_signoff.build_ppc_demo_command(
                args, run_dir, out, summary_json, thresholds
            )

            self.assertEqual(command[:3], [sys.executable, str(ROOT_DIR / "apps" / "gnss.py"), "ppc-demo"])
            self.assertIn("--use-existing-solution", command)
            self.assertIn("--rtklib-bin", command)
            self.assertIn(str(args.rtklib_bin), command)
            self.assertIn("--use-existing-rtklib-solution", command)
            self.assertIn("--require-fix-rate-min", command)
            self.assertIn("95.0", command)
            self.assertIn("--require-lib-fix-rate-vs-rtklib-min-delta", command)


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

    def test_social_card_main_renders_png(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_social_card_test_") as temp_dir:
            temp_root = Path(temp_dir)
            reference_csv = temp_root / "reference.csv"
            lib_pos = temp_root / "lib.pos"
            rtklib_pos = temp_root / "rtklib.pos"
            output_png = temp_root / "social_card.png"

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
                    (2000, 0.0, 35.0000000, 139.0000000, 10.0, 1),
                    (2000, 1.0, 35.0000100, 139.0000100, 10.2, 1),
                    (2000, 2.0, 35.0000200, 139.0000200, 10.4, 1),
                ],
            )

            argv = [
                "generate_odaiba_social_card.py",
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
                    social_card.main()

            self.assertTrue(output_png.exists())
            self.assertGreater(output_png.stat().st_size, 0)
            try:
                from PIL import Image

                with Image.open(output_png) as image:
                    self.assertEqual(image.size, (1200, 630))
            except ModuleNotFoundError:
                pass

    def test_feature_overview_card_main_renders_png(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_feature_card_test_") as temp_dir:
            output_png = Path(temp_dir) / "feature_overview.png"

            argv = [
                "generate_feature_overview_card.py",
                "--output",
                str(output_png),
            ]
            with mock.patch.object(sys, "argv", argv):
                with mock.patch.dict(os.environ, {"MPLBACKEND": "Agg"}, clear=False):
                    feature_overview.main()

            self.assertTrue(output_png.exists())
            self.assertGreater(output_png.stat().st_size, 0)
            try:
                from PIL import Image

                with Image.open(output_png) as image:
                    self.assertEqual(image.size, (2240, 1376))
            except ModuleNotFoundError:
                pass


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
            "malib_config": temp_root / "malib.conf",
            "malib_bin": None,
            "lib_pos": temp_root / "lib.pos",
            "lib_kml": temp_root / "lib.kml",
            "rtklib_pos": temp_root / "rtklib.pos",
            "malib_pos": temp_root / "malib.pos",
            "comparison_png": temp_root / "comparison.png",
            "scorecard_png": temp_root / "scorecard.png",
            "social_card_png": temp_root / "social_card.png",
            "summary_json": temp_root / "summary.json",
        }
        for key, path in paths.items():
            if isinstance(path, Path):
                if key == "malib_pos":
                    continue
                path.write_text("synthetic\n", encoding="ascii")
        defaults: dict[str, object] = {
            **paths,
            "comparison_title": "Comparison",
            "scorecard_title": "Scorecard",
            "social_card_title": "Social Card",
            "require_all_epochs_min": 0,
            "require_common_epoch_pairs_min": 0,
            "require_lib_all_p95_h_max": None,
            "require_lib_common_median_h_max": None,
            "require_lib_common_p95_h_max": None,
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
                    with mock.patch.object(benchmark, "write_summary_json") as summary_writer:
                        with mock.patch.object(
                            benchmark, "enforce_summary_requirements"
                        ) as summary_checks:
                            with mock.patch.object(
                                benchmark, "run_command", side_effect=commands.append
                            ):
                                exit_code = benchmark.main()

            self.assertEqual(exit_code, 0)
            segmented.assert_called_once()
            summary_writer.assert_called_once_with(args)
            summary_checks.assert_called_once_with(summary_writer.return_value, args)
            self.assertEqual(len(commands), 4)
            self.assertEqual(commands[0][0], str(args.rtklib_bin))
            self.assertEqual(commands[1][2], "driving-compare")
            self.assertEqual(commands[2][2], "scorecard")
            self.assertEqual(commands[3][2], "social-card")

    def test_write_summary_json_exports_all_and_common_epoch_metrics(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_benchmark_summary_") as temp_dir:
            temp_root = Path(temp_dir)
            reference_csv = temp_root / "custom_reference.csv"
            lib_pos = temp_root / "custom_lib.pos"
            rtklib_pos = temp_root / "custom_rtklib.pos"
            malib_pos = temp_root / "custom_malib.pos"
            summary_json = temp_root / "custom_summary.json"

            rows = [
                (2000, 0.0, 35.0, 139.0, 10.0),
                (2000, 1.0, 35.00001, 139.00001, 10.1),
                (2000, 2.0, 35.00002, 139.00002, 10.2),
            ]
            ScorecardRenderTest().write_reference_csv(reference_csv, rows)
            ScorecardRenderTest().write_lib_pos(
                lib_pos,
                [
                    (2000, 0.0, 35.0, 139.0, 10.0, 4),
                    (2000, 1.0, 35.0000101, 139.0000101, 10.15, 3),
                    (2000, 2.0, 35.0000202, 139.0000202, 10.25, 4),
                ],
            )
            ScorecardRenderTest().write_rtklib_pos(
                rtklib_pos,
                [
                    (2000, 0.0, 35.0, 139.0, 10.0, 1),
                    (2000, 1.0, 35.0000102, 139.0000102, 10.12, 2),
                    (2000, 2.0, 35.0000203, 139.0000203, 10.22, 1),
                ],
            )
            ScorecardRenderTest().write_rtklib_pos(
                malib_pos,
                [
                    (2000, 0.0, 35.0, 139.0, 10.0, 1),
                    (2000, 1.0, 35.0000100, 139.0000100, 10.10, 1),
                    (2000, 2.0, 35.0000201, 139.0000201, 10.20, 2),
                ],
            )

            args = self.make_benchmark_args(
                temp_root,
                reference_csv=reference_csv,
                lib_pos=lib_pos,
                rtklib_pos=rtklib_pos,
                malib_pos=malib_pos,
                summary_json=summary_json,
            )

            returned_payload = benchmark.write_summary_json(args)

            payload = json.loads(summary_json.read_text(encoding="utf-8"))
            self.assertEqual(payload["dataset"], "UrbanNav Tokyo Odaiba")
            self.assertEqual(payload["common_epoch_pairs"], 3)
            self.assertEqual(payload["libgnss_all_epochs"]["epochs"], 3)
            self.assertEqual(payload["rtklib_all_epochs"]["epochs"], 3)
            self.assertEqual(payload["malib_all_epochs"]["epochs"], 3)
            self.assertIn("median_h_m", payload["libgnss_common_epochs"])
            self.assertIn("p95_h_m", payload["rtklib_common_epochs"])
            self.assertIn("median_h_m", payload["malib_common_epochs"])
            self.assertEqual(payload["malib_common_epoch_pairs"], 3)
            self.assertEqual(payload, returned_payload)

    def test_main_full_run_invokes_optional_malib_pipeline(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_benchmark_malib_") as temp_dir:
            temp_root = Path(temp_dir)
            malib_bin = temp_root / "malib_rnx2rtkp"
            malib_bin.write_text("synthetic\n", encoding="ascii")
            args = self.make_benchmark_args(temp_root, malib_bin=malib_bin)

            commands: list[list[str]] = []
            with mock.patch.object(benchmark, "parse_args", return_value=args):
                with mock.patch.object(benchmark, "write_summary_json") as summary_writer:
                    with mock.patch.object(
                        benchmark, "enforce_summary_requirements"
                    ) as summary_checks:
                        with mock.patch.object(
                            benchmark, "run_command", side_effect=commands.append
                        ):
                            exit_code = benchmark.main()

            self.assertEqual(exit_code, 0)
            summary_writer.assert_called_once_with(args)
            summary_checks.assert_called_once_with(summary_writer.return_value, args)
            self.assertEqual(len(commands), 6)
            self.assertEqual(commands[0][2], "solve")
            self.assertEqual(commands[1][0], str(args.rtklib_bin))
            self.assertEqual(commands[2][0], str(args.malib_bin))
            self.assertEqual(commands[3][2], "driving-compare")
            self.assertEqual(commands[4][2], "scorecard")
            self.assertEqual(commands[5][2], "social-card")

    def test_enforce_summary_requirements_passes_and_fails(self) -> None:
        payload = {
            "common_epoch_pairs": 8123,
            "libgnss_all_epochs": {"epochs": 11637, "p95_h_m": 7.583936},
            "libgnss_common_epochs": {"median_h_m": 0.733387, "p95_h_m": 5.941091},
        }
        passing_args = argparse.Namespace(
            require_all_epochs_min=11000,
            require_common_epoch_pairs_min=8000,
            require_lib_all_p95_h_max=8.0,
            require_lib_common_median_h_max=0.8,
            require_lib_common_p95_h_max=6.5,
        )
        benchmark.enforce_summary_requirements(payload, passing_args)

        failing_args = argparse.Namespace(
            require_all_epochs_min=12000,
            require_common_epoch_pairs_min=9000,
            require_lib_all_p95_h_max=7.0,
            require_lib_common_median_h_max=0.7,
            require_lib_common_p95_h_max=5.0,
        )
        with self.assertRaises(SystemExit) as context:
            benchmark.enforce_summary_requirements(payload, failing_args)

        message = str(context.exception)
        self.assertIn("all-epoch matched count", message)
        self.assertIn("common epoch pairs", message)
        self.assertIn("all-epoch p95_h", message)
        self.assertIn("common-epoch median_h", message)
        self.assertIn("common-epoch p95_h", message)


class ShortBaselineSignoffTest(unittest.TestCase):
    def write_rinex_header(self, path: Path, position: tuple[float, float, float]) -> None:
        with path.open("w", encoding="ascii") as handle:
            handle.write("     3.02           OBSERVATION DATA    M                   RINEX VERSION / TYPE\n")
            handle.write(
                f"{position[0]:14.4f}{position[1]:14.4f}{position[2]:14.4f}"
                "                  APPROX POSITION XYZ\n"
            )
            handle.write("                                                            END OF HEADER\n")

    def write_pos(
        self,
        path: Path,
        records: list[tuple[int, float, tuple[float, float, float], int, int]],
    ) -> None:
        with path.open("w", encoding="ascii") as handle:
            handle.write("% synthetic short-baseline signoff\n")
            for week, tow, position, status, satellites in records:
                handle.write(
                    f"{week} {tow:.1f} {position[0]:.4f} {position[1]:.4f} {position[2]:.4f} "
                    f"35.0 139.0 10.0 {status} {satellites} 1.0\n"
                )

    def test_build_summary_payload_and_requirements(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_short_signoff_") as temp_dir:
            temp_root = Path(temp_dir)
            rover = temp_root / "rover.rnx"
            base = temp_root / "base.rnx"
            nav = temp_root / "nav.rnx"
            out = temp_root / "solution.pos"
            summary_json = temp_root / "summary.json"
            rover_position = (-3957184.1109, 3310231.7255, 3737703.9594)
            base_position = (-3957199.2400, 3310199.6680, 3737711.7080)

            self.write_rinex_header(rover, rover_position)
            self.write_rinex_header(base, base_position)
            nav.write_text("synthetic\n", encoding="ascii")
            self.write_pos(
                out,
                [
                    (2000, 0.0, rover_position, 4, 15),
                    (2000, 1.0, (-3957184.0109, 3310231.7255, 3737703.9594), 4, 14),
                    (2000, 2.0, (-3957184.2109, 3310231.7255, 3737703.9594), 3, 14),
                ],
            )

            args = argparse.Namespace(
                rover=rover,
                base=base,
                nav=nav,
                out=out,
                summary_json=summary_json,
                require_fix_rate_min=60.0,
                require_mean_error_max=0.2,
                require_max_error_max=0.25,
                require_mean_sats_min=14.0,
            )

            payload = short_signoff.build_summary_payload(args)

            self.assertEqual(payload["dataset"], "Tsukuba short_baseline")
            self.assertEqual(payload["epochs"], 3)
            self.assertEqual(payload["fixed_epochs"], 2)
            self.assertAlmostEqual(payload["fix_rate_pct"], 66.666667, places=5)
            self.assertAlmostEqual(payload["mean_position_error_m"], 0.066667, places=6)
            self.assertAlmostEqual(payload["max_position_error_m"], 0.1, places=6)
            self.assertAlmostEqual(payload["mean_satellites"], 14.333333, places=5)
            self.assertTrue(summary_json.exists())

            short_signoff.enforce_summary_requirements(payload, args)

            failing_args = argparse.Namespace(
                require_fix_rate_min=90.0,
                require_mean_error_max=0.05,
                require_max_error_max=0.05,
                require_mean_sats_min=15.0,
            )
            with self.assertRaises(SystemExit) as context:
                short_signoff.enforce_summary_requirements(payload, failing_args)

            message = str(context.exception)
            self.assertIn("fix rate", message)
            self.assertIn("mean position error", message)
            self.assertIn("max position error", message)
        self.assertIn("mean satellites", message)


class PPPStaticSignoffTest(unittest.TestCase):
    def write_rinex_header(self, path: Path, position: tuple[float, float, float]) -> None:
        with path.open("w", encoding="ascii") as handle:
            handle.write("     2.10           OBSERVATION DATA    G                   RINEX VERSION / TYPE\n")
            handle.write(
                f"{position[0]:14.4f}{position[1]:14.4f}{position[2]:14.4f}"
                "                  APPROX POSITION XYZ\n"
            )
            handle.write("                                                            END OF HEADER\n")

    def write_pos(
        self,
        path: Path,
        records: list[tuple[int, float, tuple[float, float, float], int, int]],
    ) -> None:
        with path.open("w", encoding="ascii") as handle:
            handle.write("% synthetic ppp static signoff\n")
            for week, tow, position, status, satellites in records:
                handle.write(
                    f"{week} {tow:.1f} {position[0]:.4f} {position[1]:.4f} {position[2]:.4f} "
                    f"35.0 139.0 10.0 {status} {satellites} 1.0\n"
                )

    def test_build_summary_payload_and_requirements(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_static_signoff_") as temp_dir:
            temp_root = Path(temp_dir)
            obs = temp_root / "rover.obs"
            nav = temp_root / "nav.rnx"
            out = temp_root / "solution.pos"
            summary_json = temp_root / "summary.json"
            rover_position = (-3978242.4348, 3382841.1715, 3649902.7667)

            self.write_rinex_header(obs, rover_position)
            nav.write_text("synthetic nav\n", encoding="ascii")
            self.write_pos(
                out,
                [
                    (1316, 518400.0, rover_position, 5, 9),
                    (1316, 518430.0, (-3978243.2000, 3382840.6000, 3649902.4000), 5, 8),
                    (1316, 518460.0, (-3978243.7000, 3382840.2000, 3649902.1000), 1, 9),
                ],
            )

            args = argparse.Namespace(
                obs=obs,
                nav=nav,
                sp3=None,
                clk=None,
                enable_ar=False,
                ar_ratio_threshold=3.0,
                generate_products=False,
                out=out,
                summary_json=summary_json,
                require_valid_epochs_min=3,
                require_mean_error_max=2.0,
                require_max_error_max=2.5,
                require_mean_sats_min=8.0,
                require_ppp_solution_rate_min=60.0,
                require_ppp_fixed_epochs_min=None,
                malib_pos=None,
            )

            payload = ppp_static_signoff.build_summary_payload(args)
            ppp_static_signoff.enforce_summary_requirements(payload, args)

            self.assertEqual(payload["dataset"], "sample static PPP")
            self.assertEqual(payload["epochs"], 3)
            self.assertEqual(payload["ppp_float_epochs"], 2)
            self.assertEqual(payload["ppp_fixed_epochs"], 0)
            self.assertEqual(payload["fallback_epochs"], 1)
            self.assertGreaterEqual(payload["ppp_solution_rate_pct"], 60.0)
            self.assertLessEqual(payload["mean_position_error_m"], 2.0)
            self.assertLessEqual(payload["max_position_error_m"], 2.5)
            self.assertGreaterEqual(payload["mean_satellites"], 8.0)

            failing_args = argparse.Namespace(
                require_valid_epochs_min=4,
                require_mean_error_max=0.1,
                require_max_error_max=0.2,
                require_mean_sats_min=10.0,
                require_ppp_solution_rate_min=90.0,
                require_ppp_fixed_epochs_min=1,
            )
            with self.assertRaises(SystemExit) as context:
                ppp_static_signoff.enforce_summary_requirements(payload, failing_args)

            message = str(context.exception)
            self.assertIn("valid epochs", message)
            self.assertIn("mean position error", message)
            self.assertIn("max position error", message)
            self.assertIn("mean satellites", message)
            self.assertIn("PPP solution rate", message)
            self.assertIn("PPP fixed epochs", message)

    def test_build_summary_payload_with_malib_sidecar(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_static_signoff_malib_") as temp_dir:
            temp_root = Path(temp_dir)
            obs = temp_root / "rover.obs"
            nav = temp_root / "nav.rnx"
            out = temp_root / "solution.pos"
            malib_pos = temp_root / "malib.pos"
            summary_json = temp_root / "summary.json"
            rover_position = (-3978242.4348, 3382841.1715, 3649902.7667)

            self.write_rinex_header(obs, rover_position)
            nav.write_text("synthetic nav\n", encoding="ascii")
            self.write_pos(
                out,
                [
                    (1316, 518400.0, rover_position, 5, 9),
                    (1316, 518430.0, rover_position, 5, 8),
                ],
            )
            malib_pos.write_text(
                "\n".join(
                    [
                        "% synthetic malib xyz",
                        "2005/04/02 00:00:00.000 -3978242.4348 3382841.1715 3649902.7667 6 8",
                        "2005/04/02 00:00:30.000 -3978243.4348 3382841.1715 3649902.7667 6 7",
                    ]
                )
                + "\n",
                encoding="ascii",
            )

            args = argparse.Namespace(
                obs=obs,
                nav=nav,
                sp3=None,
                clk=None,
                enable_ar=False,
                ar_ratio_threshold=3.0,
                generate_products=False,
                out=out,
                malib_pos=malib_pos,
                summary_json=summary_json,
                require_valid_epochs_min=None,
                require_mean_error_max=None,
                require_max_error_max=None,
                require_mean_sats_min=None,
                require_ppp_solution_rate_min=None,
                require_ppp_fixed_epochs_min=None,
            )

            payload = ppp_static_signoff.build_summary_payload(args)
            self.assertEqual(payload["malib_epochs"], 2)
            self.assertAlmostEqual(payload["malib_mean_position_error_m"], 0.5)
            self.assertIn("libgnss_minus_malib_mean_error_m", payload)


class PPPKinematicSignoffTest(unittest.TestCase):
    def write_pos(
        self,
        path: Path,
        records: list[tuple[int, float, tuple[float, float, float], int, int]],
    ) -> None:
        with path.open("w", encoding="ascii") as handle:
            handle.write("% synthetic ppp kinematic signoff\n")
            for week, tow, position, status, satellites in records:
                handle.write(
                    f"{week} {tow:.1f} {position[0]:.4f} {position[1]:.4f} {position[2]:.4f} "
                    f"35.0 139.0 10.0 {status} {satellites} 1.0\n"
                )

    def test_build_summary_payload_and_requirements(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_kinematic_signoff_") as temp_dir:
            temp_root = Path(temp_dir)
            obs = temp_root / "rover.obs"
            base = temp_root / "base.obs"
            nav = temp_root / "nav.rnx"
            out = temp_root / "ppp_solution.pos"
            reference_pos = temp_root / "reference.pos"
            summary_json = temp_root / "summary.json"

            obs.write_text("synthetic obs\n", encoding="ascii")
            base.write_text("synthetic base\n", encoding="ascii")
            nav.write_text("synthetic nav\n", encoding="ascii")
            self.write_pos(
                out,
                [
                    (1316, 518400.0, (-3978242.0, 3382841.0, 3649903.0), 5, 20),
                    (1316, 518430.0, (-3978248.0, 3382839.0, 3649900.0), 5, 21),
                    (1316, 518460.0, (-3978255.0, 3382834.0, 3649898.0), 1, 19),
                ],
            )
            self.write_pos(
                reference_pos,
                [
                    (1316, 518400.0, (-3978240.0, 3382840.0, 3649902.0), 4, 20),
                    (1316, 518430.0, (-3978244.0, 3382840.0, 3649900.0), 4, 21),
                    (1316, 518460.0, (-3978250.0, 3382835.0, 3649897.0), 3, 19),
                ],
            )

            args = argparse.Namespace(
                obs=obs,
                base=base,
                nav=nav,
                out=out,
                reference_pos=reference_pos,
                summary_json=summary_json,
                require_common_epoch_pairs_min=3,
                require_reference_fix_rate_min=60.0,
                require_mean_error_max=6.0,
                require_p95_error_max=8.0,
                require_max_error_max=8.0,
                require_mean_sats_min=19.0,
                require_ppp_solution_rate_min=60.0,
                malib_pos=None,
            )

            payload = ppp_kinematic_signoff.build_summary_payload(args)
            ppp_kinematic_signoff.enforce_summary_requirements(payload, args)

            self.assertEqual(payload["dataset"], "sample kinematic PPP")
            self.assertEqual(payload["epochs"], 3)
            self.assertEqual(payload["common_epoch_pairs"], 3)
            self.assertEqual(payload["ppp_float_epochs"], 2)
            self.assertEqual(payload["ppp_fixed_epochs"], 0)
            self.assertEqual(payload["fallback_epochs"], 1)
            self.assertGreaterEqual(payload["reference_fix_rate_pct"], 60.0)
            self.assertLessEqual(payload["mean_position_error_m"], 6.0)
            self.assertLessEqual(payload["p95_position_error_m"], 8.0)
            self.assertLessEqual(payload["max_position_error_m"], 8.0)
            self.assertGreaterEqual(payload["mean_satellites"], 19.0)
            self.assertGreaterEqual(payload["ppp_solution_rate_pct"], 60.0)

            failing_args = argparse.Namespace(
                require_common_epoch_pairs_min=4,
                require_reference_fix_rate_min=90.0,
                require_mean_error_max=1.0,
                require_p95_error_max=2.0,
                require_max_error_max=3.0,
                require_mean_sats_min=25.0,
                require_ppp_solution_rate_min=90.0,
            )
            with self.assertRaises(SystemExit) as context:
                ppp_kinematic_signoff.enforce_summary_requirements(payload, failing_args)

            message = str(context.exception)
            self.assertIn("common epoch pairs", message)
            self.assertIn("reference fix rate", message)
            self.assertIn("mean position error", message)
            self.assertIn("p95 position error", message)
            self.assertIn("max position error", message)
            self.assertIn("mean satellites", message)
            self.assertIn("PPP solution rate", message)

    def test_build_summary_payload_with_malib_sidecar(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_kinematic_signoff_malib_") as temp_dir:
            temp_root = Path(temp_dir)
            obs = temp_root / "rover.obs"
            base = temp_root / "base.obs"
            nav = temp_root / "nav.rnx"
            out = temp_root / "ppp_solution.pos"
            reference_pos = temp_root / "reference.pos"
            malib_pos = temp_root / "malib.pos"
            summary_json = temp_root / "summary.json"

            obs.write_text("synthetic obs\n", encoding="ascii")
            base.write_text("synthetic base\n", encoding="ascii")
            nav.write_text("synthetic nav\n", encoding="ascii")
            self.write_pos(
                out,
                [
                    (1316, 518400.0, (-3978242.0, 3382841.0, 3649903.0), 5, 20),
                    (1316, 518430.0, (-3978248.0, 3382839.0, 3649900.0), 5, 21),
                ],
            )
            self.write_pos(
                reference_pos,
                [
                    (1316, 518400.0, (-3978240.0, 3382840.0, 3649902.0), 4, 20),
                    (1316, 518430.0, (-3978244.0, 3382840.0, 3649900.0), 4, 21),
                ],
            )
            malib_pos.write_text(
                "\n".join(
                    [
                        "% synthetic malib xyz",
                        "2005/04/02 00:00:00.000 -3978240.5000 3382840.0000 3649902.0000 6 7",
                        "2005/04/02 00:00:30.000 -3978244.5000 3382840.0000 3649900.0000 6 6",
                    ]
                )
                + "\n",
                encoding="ascii",
            )

            args = argparse.Namespace(
                obs=obs,
                base=base,
                nav=nav,
                out=out,
                reference_pos=reference_pos,
                malib_pos=malib_pos,
                summary_json=summary_json,
                require_common_epoch_pairs_min=None,
                require_reference_fix_rate_min=None,
                require_mean_error_max=None,
                require_p95_error_max=None,
                require_max_error_max=None,
                require_mean_sats_min=None,
                require_ppp_solution_rate_min=None,
            )

            payload = ppp_kinematic_signoff.build_summary_payload(args)
            self.assertEqual(payload["malib_common_epoch_pairs"], 2)
            self.assertAlmostEqual(payload["malib_mean_position_error_m"], 0.5)
            self.assertIn("libgnss_minus_malib_p95_error_m", payload)


class LiveSignoffTest(unittest.TestCase):
    def test_build_summary_payload_and_requirements(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_live_signoff_") as temp_dir:
            temp_root = Path(temp_dir)
            summary_json = temp_root / "summary.json"
            args = argparse.Namespace(
                out=temp_root / "live.pos",
                summary_json=summary_json,
                log_out=temp_root / "live.log",
                use_existing_log=None,
                require_termination="completed",
                require_aligned_epochs_min=3,
                require_written_solutions_min=3,
                require_fixed_solutions_min=1,
                require_rover_decoder_errors_max=0,
                require_base_decoder_errors_max=0,
                require_realtime_factor_min=1.0,
                require_effective_epoch_rate_min=10.0,
                require_solver_wall_time_max=2.0,
            )
            summary_line = (
                "summary: termination=completed aligned_epochs=3 written_solutions=3 "
                "fixed_solutions=1 rover_decoder_errors=0 base_decoder_errors=0 "
                "solver_wall_time_s=0.250000 solution_span_s=1.000000 "
                "realtime_factor=4.000000 effective_epoch_rate_hz=12.000000"
            )

            payload = live_signoff.build_summary_payload(
                args,
                summary_line,
                "stdout text\n" + summary_line + "\n",
                "",
                0,
            )
            self.assertEqual(payload["metrics"]["termination"], "completed")
            self.assertEqual(payload["metrics"]["written_solutions"], 3)
            self.assertTrue(summary_json.exists())
            live_signoff.enforce_requirements(payload, args)

            failing_args = argparse.Namespace(
                **{
                    **vars(args),
                    "require_realtime_factor_min": 5.0,
                    "require_written_solutions_min": 4,
                }
            )
            with self.assertRaises(SystemExit) as ctx:
                live_signoff.enforce_requirements(payload, failing_args)

            self.assertIn("realtime_factor", str(ctx.exception))
            self.assertIn("written_solutions", str(ctx.exception))


class PPCDemoTest(unittest.TestCase):
    def write_reference_csv(
        self,
        path: Path,
        rows: list[tuple[int, float, float, float, float]],
    ) -> None:
        with path.open("w", newline="", encoding="ascii") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                [
                    "gps_week",
                    "gps_tow_s",
                    "lat_deg",
                    "lon_deg",
                    "height_m",
                    "roll_deg",
                    "pitch_deg",
                    "yaw_deg",
                ]
            )
            for week, tow, lat, lon, height in rows:
                writer.writerow([week, f"{tow:.3f}", lat, lon, height, 0.0, 0.0, 0.0])

    def write_pos(
        self,
        path: Path,
        rows: list[tuple[int, float, float, float, float, int, int]],
    ) -> None:
        with path.open("w", encoding="ascii") as handle:
            handle.write("% synthetic ppc demo solution\n")
            for week, tow, lat, lon, height, status, satellites in rows:
                ecef = comparison.llh_to_ecef(lat, lon, height)
                handle.write(
                    f"{week} {tow:.3f} {ecef[0]:.6f} {ecef[1]:.6f} {ecef[2]:.6f} "
                    f"{lat:.9f} {lon:.9f} {height:.4f} {status} {satellites} 1.0\n"
                )

    def write_rtklib_pos(
        self,
        path: Path,
        rows: list[tuple[int, float, float, float, float, int]],
    ) -> None:
        with path.open("w", encoding="ascii") as handle:
            handle.write("% synthetic rtklib solution\n")
            for week, tow, lat, lon, height, quality in rows:
                stamp = ppc_demo.GPS_EPOCH + timedelta(weeks=week, seconds=tow)
                handle.write(
                    f"{stamp:%Y/%m/%d %H:%M:%S.%f}"[:-3]
                    + f" {lat:.9f} {lon:.9f} {height:.4f} {quality} 0 0 0 0 0 0\n"
                )

    def test_build_summary_payload_and_requirements(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_demo_") as temp_dir:
            temp_root = Path(temp_dir)
            run_dir = temp_root / "tokyo" / "run1"
            run_dir.mkdir(parents=True)
            rover = run_dir / "rover.obs"
            base = run_dir / "base.obs"
            nav = run_dir / "base.nav"
            reference_csv = run_dir / "reference.csv"
            out = temp_root / "ppc_demo.pos"
            rtklib_pos = temp_root / "ppc_demo_rtklib.pos"
            summary_json = temp_root / "ppc_demo_summary.json"

            rover.write_text("synthetic rover\n", encoding="ascii")
            base.write_text("synthetic base\n", encoding="ascii")
            nav.write_text("synthetic nav\n", encoding="ascii")
            self.write_reference_csv(
                reference_csv,
                [
                    (2300, 1000.0, 35.1000000, 139.1000000, 42.0),
                    (2300, 1000.2, 35.1000100, 139.1000200, 42.2),
                    (2300, 1000.4, 35.1000200, 139.1000400, 42.4),
                ],
            )
            self.write_pos(
                out,
                [
                    (2300, 1000.0, 35.1000002, 139.1000001, 42.1, 4, 12),
                    (2300, 1000.2, 35.1000099, 139.1000202, 42.3, 4, 13),
                    (2300, 1000.4, 35.1000197, 139.1000398, 42.5, 3, 11),
                ],
            )
            self.write_rtklib_pos(
                rtklib_pos,
                [
                    (2300, 1000.0, 35.1000004, 139.1000003, 42.2, 1),
                    (2300, 1000.2, 35.1000103, 139.1000205, 42.4, 1),
                    (2300, 1000.4, 35.1000205, 139.1000404, 42.6, 2),
                ],
            )

            args = argparse.Namespace(
                dataset_root=None,
                city="tokyo",
                run="run1",
                run_dir=run_dir,
                solver="rtk",
                rover=rover,
                base=base,
                nav=nav,
                reference_csv=reference_csv,
                out=out,
                summary_json=summary_json,
                rtklib_pos=rtklib_pos,
                rtklib_bin=None,
                rtklib_config=None,
                use_existing_rtklib_solution=True,
                rtklib_solver_wall_time_s=0.1,
                max_epochs=120,
                match_tolerance_s=0.25,
                use_existing_solution=True,
                solver_wall_time_s=0.5,
                sp3=None,
                clk=None,
                antex=None,
                blq=None,
                enable_ar=False,
                low_dynamics=False,
                require_valid_epochs_min=3,
                require_matched_epochs_min=3,
                require_fix_rate_min=60.0,
                require_median_h_max=0.2,
                require_p95_h_max=0.2,
                require_max_h_max=0.2,
                require_p95_up_max=0.2,
                require_mean_sats_min=11.0,
                require_solver_wall_time_max=1.0,
                require_realtime_factor_min=0.5,
                require_effective_epoch_rate_min=5.0,
                require_lib_fix_rate_vs_rtklib_min_delta=0.0,
                require_lib_median_h_vs_rtklib_max_delta=0.0,
                require_lib_p95_h_vs_rtklib_max_delta=0.0,
                _dataset_city="tokyo",
                _dataset_run="run1",
            )

            payload = ppc_demo.build_summary_payload(
                args,
                run_dir,
                rover,
                base,
                nav,
                reference_csv,
                out,
                summary_json,
                solver_wall_time_s=args.solver_wall_time_s,
            )
            ppc_demo.enforce_summary_requirements(payload, args)

            self.assertEqual(payload["dataset"], "PPC-Dataset tokyo run1")
            self.assertEqual(payload["solver"], "rtk")
            self.assertEqual(payload["valid_epochs"], 3)
            self.assertEqual(payload["matched_epochs"], 3)
            self.assertEqual(payload["fixed_epochs"], 2)
            self.assertGreaterEqual(payload["fix_rate_pct"], 60.0)
            self.assertLessEqual(payload["median_h_m"], 0.2)
            self.assertLessEqual(payload["p95_h_m"], 0.2)
            self.assertLessEqual(payload["max_h_m"], 0.2)
            self.assertLessEqual(payload["p95_abs_up_m"], 0.2)
            self.assertGreaterEqual(payload["mean_satellites"], 11.0)
            self.assertEqual(payload["solver_wall_time_s"], 0.5)
            self.assertEqual(payload["solution_span_s"], 0.4)
            self.assertEqual(payload["realtime_factor"], 0.8)
            self.assertEqual(payload["effective_epoch_rate_hz"], 6.0)
            self.assertIn("rtklib", payload)
            self.assertEqual(payload["rtklib"]["matched_epochs"], 3)
            self.assertEqual(payload["rtklib"]["solver_wall_time_s"], 0.1)
            self.assertAlmostEqual(payload["rtklib"]["realtime_factor"], 4.0, places=5)
            self.assertIn("delta_vs_rtklib", payload)
            self.assertTrue(summary_json.exists())

            failing_args = argparse.Namespace(
                require_valid_epochs_min=4,
                require_matched_epochs_min=4,
                require_fix_rate_min=90.0,
                require_median_h_max=0.01,
                require_p95_h_max=0.01,
                require_max_h_max=0.01,
                require_p95_up_max=0.01,
                require_mean_sats_min=20.0,
                require_solver_wall_time_max=0.1,
                require_realtime_factor_min=2.0,
                require_effective_epoch_rate_min=10.0,
                require_lib_fix_rate_vs_rtklib_min_delta=10.0,
                require_lib_median_h_vs_rtklib_max_delta=-0.01,
                require_lib_p95_h_vs_rtklib_max_delta=-0.01,
            )
            with self.assertRaises(SystemExit) as context:
                ppc_demo.enforce_summary_requirements(payload, failing_args)

            message = str(context.exception)
            self.assertIn("valid epochs", message)
            self.assertIn("matched epochs", message)
            self.assertIn("fix rate", message)
            self.assertIn("median horizontal error", message)
            self.assertIn("p95 horizontal error", message)
            self.assertIn("max horizontal error", message)
            self.assertIn("p95 absolute up error", message)
            self.assertIn("mean satellites", message)
            self.assertIn("solver wall time", message)
            self.assertIn("realtime factor", message)
            self.assertIn("effective epoch rate", message)
            self.assertIn("RTKLIB", message)

    def test_run_rtklib_solver_executes_binary_path(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_rtklib_bin_") as temp_dir:
            temp_root = Path(temp_dir)
            run_dir = temp_root / "nagoya" / "run1"
            run_dir.mkdir(parents=True)
            rover = run_dir / "rover.obs"
            base = run_dir / "base.obs"
            nav = run_dir / "base.nav"
            reference_csv = run_dir / "reference.csv"
            rtklib_pos = temp_root / "rtklib.pos"
            config_path = temp_root / "rtklib.conf"
            fake_rtklib = temp_root / "fake_rnx2rtkp.py"

            rover.write_text("synthetic rover\n", encoding="ascii")
            base.write_text("synthetic base\n", encoding="ascii")
            nav.write_text("synthetic nav\n", encoding="ascii")
            config_path.write_text("pos1-navsys        =1\n", encoding="ascii")
            self.write_reference_csv(
                reference_csv,
                [
                    (2300, 1000.0, 35.1000000, 139.1000000, 42.0),
                    (2300, 1000.2, 35.1000100, 139.1000200, 42.2),
                ],
            )
            fake_rtklib.write_text(
                """#!/usr/bin/env python3
import sys
from pathlib import Path

args = sys.argv[1:]
out = Path(args[args.index("-o") + 1])
out.write_text(
    "% synthetic rtklib solution\\n"
    "2024/02/18 00:16:22.000 35.100000000 139.100000000 42.0000 1 0 0 0 0 0 0\\n"
    "2024/02/18 00:16:22.200 35.100010000 139.100020000 42.2000 1 0 0 0 0 0 0\\n",
    encoding="ascii",
)
""",
                encoding="utf-8",
            )
            fake_rtklib.chmod(0o755)

            args = argparse.Namespace(
                solver="rtk",
                rtklib_bin=fake_rtklib,
                rtklib_config=config_path,
                max_epochs=120,
            )

            elapsed = ppc_demo.run_rtklib_solver(
                args,
                rover,
                base,
                nav,
                ppc_demo.read_flexible_reference_csv(reference_csv),
                rtklib_pos,
            )

            self.assertGreaterEqual(elapsed, 0.0)
            self.assertTrue(rtklib_pos.exists())
            contents = rtklib_pos.read_text(encoding="ascii")
            self.assertIn("synthetic rtklib solution", contents)


if __name__ == "__main__":
    unittest.main()
