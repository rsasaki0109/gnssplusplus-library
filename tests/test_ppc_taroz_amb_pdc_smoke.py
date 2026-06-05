#!/usr/bin/env python3
"""Tests for the PPC taroz ambiguity PDC smoke harness."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest


ROOT_DIR = Path(__file__).resolve().parents[1]
APPS_DIR = ROOT_DIR / "apps"
sys.path.insert(0, str(APPS_DIR))

import gnss_ppc_taroz_amb_pdc_smoke  # noqa: E402

FULL_SEED_SUMMARY = (
    ROOT_DIR / "output/dogfood/ppc_taroz_amb_pdc_smoke_200_seed_current/summary.json"
)
SHIFTED_NAGOYA_RUN3_SEED_SUMMARY = (
    ROOT_DIR
    / "output/dogfood/ppc_taroz_amb_pdc_nagoya_run3_shifted_seed_current/summary.json"
)
NAGOYA_RUN2_1000_SEED_SUMMARY = (
    ROOT_DIR
    / "output/dogfood/ppc_taroz_amb_pdc_nagoya_run2_1000_seed_current/summary.json"
)
NAGOYA_RUN3_1000_SEED_SUMMARY = (
    ROOT_DIR
    / "output/dogfood/ppc_taroz_amb_pdc_nagoya_run3_1000_seed_current/summary.json"
)
TOKYO_RUN2_1000_SEED_SUMMARY = (
    ROOT_DIR
    / "output/dogfood/ppc_taroz_amb_pdc_tokyo_run2_1000_seed_current/summary.json"
)


def read_seed_pos_by_tow(path: Path) -> dict[int, dict[str, float]]:
    rows: dict[int, dict[str, float]] = {}
    with path.open(encoding="ascii", errors="ignore") as handle:
        for line in handle:
            if not line.strip() or line.startswith("%"):
                continue
            parts = line.split()
            if len(parts) < 5:
                raise ValueError(f"unexpected seed POS row width in {path}: {line.rstrip()}")
            rows[round(float(parts[1]) * 1000)] = {
                "gps_week": float(parts[0]),
                "x_m": float(parts[2]),
                "y_m": float(parts[3]),
                "z_m": float(parts[4]),
            }
    return rows


def read_solution_pos_by_tow(path: Path) -> dict[tuple[int, int], dict[str, float | int]]:
    rows: dict[tuple[int, int], dict[str, float | int]] = {}
    with path.open(encoding="ascii", errors="ignore") as handle:
        for line in handle:
            if not line.strip() or line.startswith("%"):
                continue
            parts = line.split()
            if len(parts) < 14:
                raise ValueError(f"unexpected solution POS row width in {path}: {line.rstrip()}")
            key = (int(float(parts[0])), round(float(parts[1]) * 1000))
            rows[key] = {
                "x_m": float(parts[2]),
                "y_m": float(parts[3]),
                "z_m": float(parts[4]),
                "status": int(float(parts[8])),
                "satellites": int(float(parts[9])),
                "ratio": float(parts[11]),
                "fixed_ambiguities": int(float(parts[12])),
                "iterations": int(float(parts[13])),
            }
    return rows


def delimited_count(value: str) -> int:
    return len([part for part in value.split(";") if part])


class PpcTarozAmbPdcSmokeTest(unittest.TestCase):
    def require_summary(self, path: Path) -> dict[str, object]:
        if not path.exists():
            self.skipTest(f"missing optional PPC taroz dogfood summary: {path}")
        return json.loads(path.read_text(encoding="utf-8"))

    def assert_seed_state_matches_epoch_debug(self, run_payload: dict[str, object]) -> None:
        outputs = run_payload["outputs"]
        epoch_path = Path(outputs["epoch_debug"])
        seed_path = Path(outputs["seed_pos"])
        if not epoch_path.exists() or not seed_path.exists():
            self.skipTest(f"missing optional PPC taroz seed artifact(s): {epoch_path}, {seed_path}")

        with epoch_path.open(newline="") as handle:
            epoch_rows = list(csv.DictReader(handle))
        seed_rows = read_seed_pos_by_tow(seed_path)
        native = run_payload["native_summary"]

        self.assertEqual(len(epoch_rows), native["optimized_epochs"])
        self.assertEqual(len(epoch_rows), native["seed_matched_epochs"])
        self.assertEqual(native["seed_interpolated_epochs"], 0)

        tows = [round(float(row["gps_tow"]) * 1000) for row in epoch_rows]
        self.assertEqual(len(tows), len(set(tows)))
        self.assertTrue(all(current < following for current, following in zip(tows, tows[1:])))
        self.assertTrue(set(tows).issubset(seed_rows))

        for row in epoch_rows:
            seed = seed_rows[round(float(row["gps_tow"]) * 1000)]
            self.assertEqual(int(seed["gps_week"]), int(row["gps_week"]))
            self.assertAlmostEqual(seed["x_m"], float(row["seed_position_x_m"]), places=6)
            self.assertAlmostEqual(seed["y_m"], float(row["seed_position_y_m"]), places=6)
            self.assertAlmostEqual(seed["z_m"], float(row["seed_position_z_m"]), places=6)
            position = (
                float(row["position_x_m"]),
                float(row["position_y_m"]),
                float(row["position_z_m"]),
            )
            seed_position = (
                float(row["seed_position_x_m"]),
                float(row["seed_position_y_m"]),
                float(row["seed_position_z_m"]),
            )
            self.assertAlmostEqual(
                math.dist(position, seed_position),
                float(row["seed_position_divergence_m"]),
                places=5,
            )

    def assert_solution_artifacts_match_summary(self, run_payload: dict[str, object]) -> None:
        outputs = run_payload["outputs"]
        epoch_path = Path(outputs["epoch_debug"])
        pos_path = Path(outputs["pos"])
        if not epoch_path.exists() or not pos_path.exists():
            self.skipTest(
                f"missing optional PPC taroz solution artifact(s): {epoch_path}, {pos_path}"
            )

        with epoch_path.open(newline="") as handle:
            epoch_rows = list(csv.DictReader(handle))
        pos_rows = read_solution_pos_by_tow(pos_path)
        native = run_payload["native_summary"]
        reference = run_payload["reference_summary"]

        self.assertEqual(len(epoch_rows), native["optimized_epochs"])
        self.assertEqual(len(pos_rows), native["optimized_epochs"])

        epoch_status_counts: dict[str, int] = {}
        pos_status_counts: dict[str, int] = {}
        for pos in pos_rows.values():
            status_key = str(pos["status"])
            pos_status_counts[status_key] = pos_status_counts.get(status_key, 0) + 1

        dd_carrier_candidates = 0
        lambda_attempts = 0
        lambda_candidates = 0
        fixed_ambiguities = 0
        for row in epoch_rows:
            key = (int(float(row["gps_week"])), round(float(row["gps_tow"]) * 1000))
            self.assertIn(key, pos_rows)
            pos = pos_rows[key]
            status = int(float(row["status"]))
            epoch_status_counts[str(status)] = epoch_status_counts.get(str(status), 0) + 1
            candidate_count = int(float(row["ambiguity_candidates"]))
            dd_carrier_candidates += candidate_count
            if candidate_count > 5:
                lambda_attempts += 1
                lambda_candidates += candidate_count
            fixed_ambiguities += int(float(row["num_fixed_ambiguities"]))

            self.assertEqual(status, pos["status"])
            self.assertEqual(delimited_count(row["dd_satellites"]), pos["satellites"])
            self.assertAlmostEqual(float(row["position_x_m"]), pos["x_m"], places=6)
            self.assertAlmostEqual(float(row["position_y_m"]), pos["y_m"], places=6)
            self.assertAlmostEqual(float(row["position_z_m"]), pos["z_m"], places=6)
            self.assertAlmostEqual(float(row["ratio"]), pos["ratio"], places=6)
            self.assertEqual(int(float(row["num_fixed_ambiguities"])), pos["fixed_ambiguities"])
            self.assertEqual(pos["iterations"], native["iterations"])

        self.assertEqual(epoch_status_counts, reference["position_status_counts"])
        self.assertEqual(pos_status_counts, reference["position_status_counts"])
        self.assertEqual(pos_status_counts.get("4", 0), native["fixed_solutions"])
        self.assertEqual(pos_status_counts.get("3", 0), native["float_solutions"])
        self.assertEqual(
            pos_status_counts.get("3", 0) + pos_status_counts.get("4", 0),
            native["valid_solutions"],
        )
        self.assertEqual(dd_carrier_candidates, native["double_difference_carrier_factors"])
        self.assertEqual(lambda_attempts, native["lambda_ambiguity_attempts"])
        self.assertEqual(lambda_candidates, native["lambda_ambiguity_candidates"])
        self.assertEqual(fixed_ambiguities, native["lambda_ambiguity_used_candidates"])

    def assert_lambda_debug_matches_epoch_debug(self, run_payload: dict[str, object]) -> None:
        outputs = run_payload["outputs"]
        if "lambda_debug" not in outputs:
            self.skipTest("missing optional PPC taroz lambda debug output path")
        epoch_path = Path(outputs["epoch_debug"])
        lambda_path = Path(outputs["lambda_debug"])
        if not epoch_path.exists() or not lambda_path.exists():
            self.skipTest(
                f"missing optional PPC taroz lambda artifact(s): {epoch_path}, {lambda_path}"
            )

        with epoch_path.open(newline="") as handle:
            epoch_rows = list(csv.DictReader(handle))
        with lambda_path.open(newline="") as handle:
            lambda_rows = list(csv.DictReader(handle))
        native = run_payload["native_summary"]

        attempted_epochs: dict[tuple[int, int], dict[str, str]] = {}
        for row in epoch_rows:
            candidate_count = int(float(row["ambiguity_candidates"]))
            if candidate_count > 5:
                key = (int(float(row["gps_week"])), round(float(row["gps_tow"]) * 1000))
                attempted_epochs[key] = row

        matrix_rows_by_epoch: dict[tuple[int, int], int] = {}
        diagonal_by_epoch: dict[tuple[int, int], dict[str, str]] = {}
        for row in lambda_rows:
            key = (int(float(row["gps_week"])), round(float(row["gps_tow"]) * 1000))
            matrix_rows_by_epoch[key] = matrix_rows_by_epoch.get(key, 0) + 1
            if int(float(row["row"])) == 0 and int(float(row["col"])) == 0:
                diagonal_by_epoch[key] = row

        self.assertEqual(set(diagonal_by_epoch), set(attempted_epochs))
        self.assertEqual(len(diagonal_by_epoch), native["lambda_ambiguity_attempts"])
        self.assertEqual(
            len(lambda_rows),
            sum(
                int(float(row["ambiguity_candidates"])) ** 2
                for row in attempted_epochs.values()
            ),
        )

        lambda_candidates = 0
        fixed_candidates = 0
        fixed_epochs = 0
        for key, epoch_row in attempted_epochs.items():
            lambda_row = diagonal_by_epoch[key]
            candidate_count = int(float(epoch_row["ambiguity_candidates"]))
            fixed_epoch = int(float(lambda_row["fixed_epoch"])) == 1
            lambda_candidates += candidate_count
            if fixed_epoch:
                fixed_epochs += 1
                fixed_candidates += candidate_count

            self.assertEqual(int(float(lambda_row["candidate_count"])), candidate_count)
            self.assertEqual(matrix_rows_by_epoch[key], candidate_count * candidate_count)
            self.assertEqual(fixed_epoch, int(float(epoch_row["status"])) == 4)
            self.assertAlmostEqual(float(lambda_row["ratio"]), float(epoch_row["ratio"]), places=6)

        self.assertEqual(lambda_candidates, native["lambda_ambiguity_candidates"])
        self.assertEqual(fixed_candidates, native["lambda_ambiguity_used_candidates"])
        self.assertEqual(fixed_epochs, native["fixed_solutions"])

    def assert_cost_trace_matches_summary(self, run_payload: dict[str, object]) -> None:
        outputs = run_payload["outputs"]
        if "cost_trace" not in outputs:
            self.skipTest("missing optional PPC taroz cost trace output path")
        cost_path = Path(outputs["cost_trace"])
        if not cost_path.exists():
            self.skipTest(f"missing optional PPC taroz cost trace artifact: {cost_path}")

        with cost_path.open(newline="") as handle:
            rows = list(csv.DictReader(handle))
        native = run_payload["native_summary"]

        self.assertGreater(len(rows), 0)
        self.assertEqual(rows[0]["phase"], "float")
        self.assertEqual(int(rows[0]["local_iteration"]), 0)
        self.assertEqual(int(rows[0]["global_iteration"]), 0)
        self.assertAlmostEqual(float(rows[0]["cost"]), float(native["initial_cost"]), places=5)
        self.assertAlmostEqual(float(rows[-1]["cost"]), float(native["final_cost"]), places=5)
        phases = {row["phase"] for row in rows}
        self.assertTrue(phases <= {"float", "fixed"})
        self.assertTrue(all(math.isfinite(float(row["cost"])) for row in rows))
        self.assertTrue(all(float(row["cost"]) >= 0.0 for row in rows))
        global_iterations = [int(row["global_iteration"]) for row in rows]
        self.assertEqual(global_iterations, sorted(global_iterations))
        self.assertEqual(len(global_iterations), len(set(global_iterations)))
        self.assertLessEqual(len(rows), int(native["iterations"]) + len(phases))

    def make_run(self, root: Path, spec: str) -> Path:
        site, run_name = spec.split("/")
        run_dir = root / site / run_name
        run_dir.mkdir(parents=True)
        for name in ("rover.obs", "base.obs", "base.nav", "reference.csv"):
            (run_dir / name).write_text("", encoding="ascii")
        return run_dir

    def test_discovers_nested_dataset_root(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_discover_test_") as temp_dir:
            outer_root = Path(temp_dir)
            dataset_root = outer_root / "PPC-Dataset"
            self.make_run(dataset_root, "nagoya/run1")
            self.make_run(dataset_root, "tokyo/run2")

            runs = gnss_ppc_taroz_amb_pdc_smoke.discover_runs(outer_root)

            self.assertEqual([run.spec for run in runs], ["nagoya/run1", "tokyo/run2"])

    def test_dry_run_writes_planned_commands(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_smoke_dry_run_test_") as temp_dir:
            temp_root = Path(temp_dir)
            dataset_root = temp_root / "PPC-Dataset"
            self.make_run(dataset_root, "nagoya/run1")
            summary_json = temp_root / "summary.json"

            result = gnss_ppc_taroz_amb_pdc_smoke.main(
                [
                    "--dataset-root",
                    str(temp_root),
                    "--run",
                    "nagoya/run1",
                    "--summary-json",
                    str(summary_json),
                    "--fgo-bin",
                    str(temp_root / "gnss_fgo"),
                    "--spp-bin",
                    str(temp_root / "gnss_spp"),
                    "--skip-epochs",
                    "7",
                    "--generate-spp-seed",
                    "--dry-run",
                ]
            )

            self.assertEqual(result, 0)
            payload = json.loads(summary_json.read_text(encoding="utf-8"))
            command = payload["planned_commands"]["nagoya/run1"]
            seed_command = payload["planned_seed_commands"]["nagoya/run1"]
            self.assertEqual(payload["status"], "dry-run")
            self.assertEqual(payload["skip_epochs"], 7)
            self.assertTrue(payload["generate_spp_seed"])
            self.assertIn("--max-epochs", seed_command)
            self.assertIn("27", seed_command)
            self.assertIn("--skip-epochs", command)
            self.assertIn("7", command)
            self.assertIn("--seed-pos", command)
            self.assertIn("--preset", command)
            self.assertIn("taroz-amb-pdc", command)
            self.assertIn("--lambda-debug-csv", command)
            self.assertIn("--cost-trace-csv", command)
            self.assertIn("--max-epochs", command)
            self.assertIn("20", command)

    def test_taroz_matlab_data_dir_requires_generated_seed(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_smoke_dry_run_test_") as temp_dir:
            temp_root = Path(temp_dir)
            dataset_root = temp_root / "PPC-Dataset"
            self.make_run(dataset_root, "nagoya/run1")

            with self.assertRaises(SystemExit) as raised:
                gnss_ppc_taroz_amb_pdc_smoke.main(
                    [
                        "--dataset-root",
                        str(dataset_root),
                        "--run",
                        "nagoya/run1",
                        "--taroz-matlab-data-dir",
                        str(temp_root / "taroz_data"),
                        "--dry-run",
                    ]
                )

            self.assertIn(
                "--taroz-matlab-data-dir requires --generate-spp-seed",
                str(raised.exception),
            )

    def test_prepare_taroz_matlab_data_dir_writes_seed_and_base_position(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_taroz_data_test_") as temp_dir:
            temp_root = Path(temp_dir)
            run_dir = self.make_run(temp_root / "PPC-Dataset", "nagoya/run1")
            (run_dir / "base.obs").write_text(
                "     3.04           OBSERVATION DATA    M                   RINEX VERSION / TYPE\n"
                " -3817680.7301  3562839.5234  3650159.2418                  APPROX POSITION XYZ\n"
                "                                                            END OF HEADER\n",
                encoding="ascii",
            )
            seed_pos = temp_root / "seed.pos"
            seed_pos.write_text(
                "% seed\n"
                "2325 553800.000000 -3811467.244148 3566808.551089 "
                "3652669.169515 0 0 0 1 27 0 0 0 0\n",
                encoding="ascii",
            )
            out_dir = temp_root / "taroz_data"
            run = gnss_ppc_taroz_amb_pdc_smoke.PpcRun(
                "nagoya",
                "run1",
                run_dir,
            )

            payload = gnss_ppc_taroz_amb_pdc_smoke.prepare_taroz_matlab_data_dir(
                run,
                seed_pos,
                out_dir,
            )

            self.assertEqual(payload["seed_rows"], 1)
            self.assertEqual(payload["base_position_llh_deg_m"][0], 35.13470947)
            self.assertTrue((out_dir / "rover_1Hz.obs").exists())
            self.assertTrue((out_dir / "base_position.txt").exists())
            seed_text = (out_dir / "rover_1Hz_spp.pos").read_text(encoding="ascii")
            self.assertIn("2024/08/03 09:50:00.000", seed_text)
            self.assertIn("35.1627", seed_text)

    def test_taroz_matlab_seed_export_must_cover_skipped_window(self) -> None:
        class Args:
            taroz_matlab_data_dir = Path("taroz_data")
            skip_epochs = 400
            max_epochs = 120

        self.assertEqual(
            gnss_ppc_taroz_amb_pdc_smoke.validate_taroz_matlab_data(
                Args(),
                {"seed_rows": 520},
            ),
            [],
        )
        self.assertEqual(
            gnss_ppc_taroz_amb_pdc_smoke.validate_taroz_matlab_data(
                Args(),
                {"seed_rows": 120},
            ),
            ["taroz MATLAB seed_rows 120 != skip_epochs + max_epochs 520"],
        )

    def test_taroz_matlab_seed_export_allows_full_run_without_fixed_row_count(self) -> None:
        class Args:
            taroz_matlab_data_dir = Path("taroz_data")
            skip_epochs = 400
            max_epochs = 0

        self.assertEqual(
            gnss_ppc_taroz_amb_pdc_smoke.validate_taroz_matlab_data(
                Args(),
                {"seed_rows": 120},
            ),
            [],
        )

    def test_generated_seed_audit_accepts_shifted_window_coverage(self) -> None:
        class Args:
            generate_spp_seed = True
            skip_epochs = 2
            max_epochs = 3

        with tempfile.TemporaryDirectory(prefix="gnss_ppc_seed_audit_test_") as temp_dir:
            temp_root = Path(temp_dir)
            seed_pos = temp_root / "seed.pos"
            epoch_debug = temp_root / "epoch_debug.csv"
            seed_pos.write_text(
                "% seed\n"
                "2325 10.000000 1.0 2.0 3.0 0 0 0 1 12\n"
                "2325 10.200000 2.0 3.0 4.0 0 0 0 1 12\n"
                "2325 10.400000 3.0 4.0 5.0 0 0 0 1 12\n"
                "2325 10.600000 4.0 5.0 6.0 0 0 0 1 12\n"
                "2325 10.800000 5.0 6.0 7.0 0 0 0 1 12\n",
                encoding="ascii",
            )
            epoch_debug.write_text(
                "gps_week,gps_tow\n"
                "2325,10.400000\n"
                "2325,10.600000\n"
                "2325,10.800000\n",
                encoding="utf-8",
            )

            audit, failures = gnss_ppc_taroz_amb_pdc_smoke.audit_generated_seed_artifacts(
                Args(),
                seed_pos,
                epoch_debug,
            )

            self.assertEqual(failures, [])
            self.assertEqual(audit["seed_rows"], 5)
            self.assertEqual(audit["optimized_epoch_rows"], 3)
            self.assertEqual(audit["expected_seed_rows"], 5)
            self.assertEqual(audit["missing_optimized_seed_epochs"], 0)
            self.assertTrue(audit["window_sequence_matches"])

    def test_generated_seed_audit_flags_missing_and_shifted_window_seed(self) -> None:
        class Args:
            generate_spp_seed = True
            skip_epochs = 2
            max_epochs = 3

        with tempfile.TemporaryDirectory(prefix="gnss_ppc_seed_audit_test_") as temp_dir:
            temp_root = Path(temp_dir)
            seed_pos = temp_root / "seed.pos"
            epoch_debug = temp_root / "epoch_debug.csv"
            seed_pos.write_text(
                "% seed shifted by one row and missing 10.400\n"
                "2325 10.000000 1.0 2.0 3.0 0 0 0 1 12\n"
                "2325 10.200000 2.0 3.0 4.0 0 0 0 1 12\n"
                "2325 10.600000 4.0 5.0 6.0 0 0 0 1 12\n"
                "2325 10.800000 5.0 6.0 7.0 0 0 0 1 12\n"
                "2325 11.000000 6.0 7.0 8.0 0 0 0 1 12\n",
                encoding="ascii",
            )
            epoch_debug.write_text(
                "gps_week,gps_tow\n"
                "2325,10.400000\n"
                "2325,10.600000\n"
                "2325,10.800000\n",
                encoding="utf-8",
            )

            audit, failures = gnss_ppc_taroz_amb_pdc_smoke.audit_generated_seed_artifacts(
                Args(),
                seed_pos,
                epoch_debug,
            )

            self.assertEqual(audit["missing_optimized_seed_epochs"], 1)
            self.assertFalse(audit["window_sequence_matches"])
            self.assertIn(
                "generated seed POS is missing 1 optimized epoch(s), "
                "first missing 2325:10.400",
                failures,
            )
            self.assertIn(
                "generated seed window does not match optimized epoch sequence "
                "at skip_epochs 2: seed 2325:10.600 != epoch 2325:10.400",
                failures,
            )

    def test_generated_seed_audit_flags_row_count_drift(self) -> None:
        class Args:
            generate_spp_seed = True
            skip_epochs = 2
            max_epochs = 3

        with tempfile.TemporaryDirectory(prefix="gnss_ppc_seed_audit_test_") as temp_dir:
            temp_root = Path(temp_dir)
            seed_pos = temp_root / "seed.pos"
            epoch_debug = temp_root / "epoch_debug.csv"
            seed_pos.write_text(
                "% seed too short\n"
                "2325 10.400000 3.0 4.0 5.0 0 0 0 1 12\n"
                "2325 10.600000 4.0 5.0 6.0 0 0 0 1 12\n"
                "2325 10.800000 5.0 6.0 7.0 0 0 0 1 12\n",
                encoding="ascii",
            )
            epoch_debug.write_text(
                "gps_week,gps_tow\n"
                "2325,10.400000\n"
                "2325,10.600000\n"
                "2325,10.800000\n",
                encoding="utf-8",
            )

            _, failures = gnss_ppc_taroz_amb_pdc_smoke.audit_generated_seed_artifacts(
                Args(),
                seed_pos,
                epoch_debug,
            )

            self.assertIn(
                "generated seed rows 3 != skip_epochs + max_epochs 5",
                failures,
            )

    def test_generated_seed_audit_flags_duplicate_epochs(self) -> None:
        class Args:
            generate_spp_seed = True
            skip_epochs = 0
            max_epochs = 3

        with tempfile.TemporaryDirectory(prefix="gnss_ppc_seed_audit_test_") as temp_dir:
            temp_root = Path(temp_dir)
            seed_pos = temp_root / "seed.pos"
            epoch_debug = temp_root / "epoch_debug.csv"
            seed_pos.write_text(
                "% seed duplicate\n"
                "2325 10.000000 1.0 2.0 3.0 0 0 0 1 12\n"
                "2325 10.200000 2.0 3.0 4.0 0 0 0 1 12\n"
                "2325 10.200000 9.0 9.0 9.0 0 0 0 1 12\n",
                encoding="ascii",
            )
            epoch_debug.write_text(
                "gps_week,gps_tow\n"
                "2325,10.000000\n"
                "2325,10.200000\n"
                "2325,10.200000\n",
                encoding="utf-8",
            )

            audit, failures = gnss_ppc_taroz_amb_pdc_smoke.audit_generated_seed_artifacts(
                Args(),
                seed_pos,
                epoch_debug,
            )

            self.assertEqual(audit["missing_optimized_seed_epochs"], 0)
            self.assertIsNone(audit["window_sequence_matches"])
            self.assertIn(
                "generated seed POS has duplicate epoch(s): 2325:10.200",
                failures,
            )
            self.assertIn(
                "epoch debug CSV has duplicate epoch(s): 2325:10.200",
                failures,
            )

    def test_generated_seed_audit_flags_unsorted_epochs(self) -> None:
        class Args:
            generate_spp_seed = True
            skip_epochs = 0
            max_epochs = 3

        with tempfile.TemporaryDirectory(prefix="gnss_ppc_seed_audit_test_") as temp_dir:
            temp_root = Path(temp_dir)
            seed_pos = temp_root / "seed.pos"
            epoch_debug = temp_root / "epoch_debug.csv"
            seed_pos.write_text(
                "% seed unsorted\n"
                "2325 10.200000 2.0 3.0 4.0 0 0 0 1 12\n"
                "2325 10.000000 1.0 2.0 3.0 0 0 0 1 12\n"
                "2325 10.400000 3.0 4.0 5.0 0 0 0 1 12\n",
                encoding="ascii",
            )
            epoch_debug.write_text(
                "gps_week,gps_tow\n"
                "2325,10.200000\n"
                "2325,10.000000\n"
                "2325,10.400000\n",
                encoding="utf-8",
            )

            audit, failures = gnss_ppc_taroz_amb_pdc_smoke.audit_generated_seed_artifacts(
                Args(),
                seed_pos,
                epoch_debug,
            )

            self.assertTrue(audit["window_sequence_matches"])
            self.assertIn("generated seed POS epochs are not strictly sorted", failures)
            self.assertIn("epoch debug CSV epochs are not strictly sorted", failures)

    def test_generated_seed_audit_skips_fixed_row_count_for_full_run(self) -> None:
        class Args:
            generate_spp_seed = True
            skip_epochs = 400
            max_epochs = 0

        with tempfile.TemporaryDirectory(prefix="gnss_ppc_seed_audit_test_") as temp_dir:
            temp_root = Path(temp_dir)
            seed_pos = temp_root / "seed.pos"
            epoch_debug = temp_root / "epoch_debug.csv"
            seed_pos.write_text(
                "% full run seed, no fixed row-count expectation\n"
                "2325 10.000000 1.0 2.0 3.0 0 0 0 1 12\n"
                "2325 10.200000 2.0 3.0 4.0 0 0 0 1 12\n",
                encoding="ascii",
            )
            epoch_debug.write_text(
                "gps_week,gps_tow\n"
                "2325,10.000000\n"
                "2325,10.200000\n",
                encoding="utf-8",
            )

            audit, failures = gnss_ppc_taroz_amb_pdc_smoke.audit_generated_seed_artifacts(
                Args(),
                seed_pos,
                epoch_debug,
            )

            self.assertEqual(failures, [])
            self.assertIsNone(audit["expected_seed_rows"])
            self.assertIsNone(audit["window_sequence_matches"])

    def test_native_summary_validation_flags_low_candidate_run(self) -> None:
        class Args:
            generate_spp_seed = False
            max_epochs = 20
            require_valid_min = 1
            require_dd_carrier_min = 1
            require_lambda_attempts_min = 1
            require_fix_rate_min = 0.0
            allow_not_converged = False

        failures = gnss_ppc_taroz_amb_pdc_smoke.validate_native_summary(
            Args(),
            {
                "preset": "taroz-amb-pdc",
                "backend": "eigen",
                "optimized_epochs": 20,
                "valid_solutions": 0,
                "double_difference_carrier_factors": 0,
                "lambda_ambiguity_attempts": 0,
                "fix_rate_percent": 0.0,
                "converged": True,
            },
        )

        self.assertEqual(
            failures,
            [
                "valid_solutions 0 < 1",
                "double_difference_carrier_factors 0 < 1",
                "lambda_ambiguity_attempts 0 < 1",
            ],
        )

    def test_native_summary_validation_accepts_generated_seed_match(self) -> None:
        class Args:
            generate_spp_seed = True
            max_epochs = 20
            require_valid_min = 1
            require_dd_carrier_min = 1
            require_lambda_attempts_min = 1
            require_fix_rate_min = 0.0
            allow_not_converged = False

        failures = gnss_ppc_taroz_amb_pdc_smoke.validate_native_summary(
            Args(),
            {
                "preset": "taroz-amb-pdc",
                "backend": "eigen",
                "optimized_epochs": 20,
                "seed_pos": "out/run/spp_seed.pos",
                "seed_matched_epochs": 20,
                "seed_interpolated_epochs": 0,
                "valid_solutions": 20,
                "double_difference_carrier_factors": 120,
                "lambda_ambiguity_attempts": 20,
                "fix_rate_percent": 100.0,
                "converged": True,
            },
        )

        self.assertEqual(failures, [])

    def test_native_summary_validation_flags_generated_seed_drift(self) -> None:
        class Args:
            generate_spp_seed = True
            max_epochs = 20
            require_valid_min = 1
            require_dd_carrier_min = 1
            require_lambda_attempts_min = 1
            require_fix_rate_min = 0.0
            allow_not_converged = False

        failures = gnss_ppc_taroz_amb_pdc_smoke.validate_native_summary(
            Args(),
            {
                "preset": "taroz-amb-pdc",
                "backend": "eigen",
                "optimized_epochs": 20,
                "seed_pos": "",
                "seed_matched_epochs": 19,
                "seed_interpolated_epochs": 1,
                "valid_solutions": 20,
                "double_difference_carrier_factors": 120,
                "lambda_ambiguity_attempts": 20,
                "fix_rate_percent": 100.0,
                "converged": True,
            },
        )

        self.assertEqual(
            failures,
            [
                "generated SPP seed path is missing from summary",
                "seed_matched_epochs 19 != 20",
                "seed_interpolated_epochs 1 != 0",
            ],
        )

    def test_reference_summary_keeps_no_solution_out_of_valid_error(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_reference_summary_test_") as temp_dir:
            temp_root = Path(temp_dir)
            pos_path = temp_root / "solution.pos"
            reference_csv = temp_root / "reference.csv"
            pos_path.write_text(
                "\n".join(
                    [
                        "% synthetic solution",
                        "2325 10.000000 1.0 0.0 0.0 0 0 0 3 8",
                        "2325 10.200000 1000.0 0.0 0.0 0 0 0 0 0",
                    ]
                )
                + "\n",
                encoding="ascii",
            )
            reference_csv.write_text(
                "GPS Week,GPS TOW (s),ECEF X (m),ECEF Y (m),ECEF Z (m)\n"
                "2325,10.000000,0.0,0.0,0.0\n"
                "2325,10.200000,0.0,0.0,0.0\n",
                encoding="ascii",
            )

            summary = gnss_ppc_taroz_amb_pdc_smoke.summarize_reference_errors(
                pos_path,
                reference_csv,
                0.05,
            )

            self.assertEqual(summary["matched_epochs"], 1)
            self.assertEqual(summary["p95_3d_error_m"], 1.0)
            self.assertEqual(summary["tail_counts_3d_error_m"]["gt_0_5_m"], 1)
            self.assertEqual(summary["tail_counts_3d_error_m"]["gt_1_m"], 0)
            self.assertEqual(summary["worst_epoch"]["status"], 3)
            self.assertEqual(summary["no_solution"]["matched_epochs"], 1)
            self.assertEqual(
                summary["no_solution"]["tail_counts_3d_error_m"]["gt_100_m"],
                1,
            )
            self.assertEqual(summary["all_output"]["max_3d_error_m"], 1000.0)
            self.assertEqual(summary["all_output"]["worst_epoch"]["status"], 0)

    def test_reference_summary_validation_flags_large_valid_p95(self) -> None:
        class Args:
            require_valid_p95_3d_max = 2.0
            require_valid_max_3d_max = 0.0
            require_fixed_p95_3d_max = 0.5

        failures = gnss_ppc_taroz_amb_pdc_smoke.validate_reference_summary(
            Args(),
            {
                "p95_3d_error_m": 3.25,
                "max_3d_error_m": 10.0,
                "fixed": {"p95_3d_error_m": 0.75},
            },
        )

        self.assertEqual(
            failures,
            [
                "valid p95_3d_error_m 3.250000 > 2.0",
                "fixed p95_3d_error_m 0.750000 > 0.5",
            ],
        )

    def test_optional_200_epoch_generated_seed_summary_contract(self) -> None:
        payload = self.require_summary(FULL_SEED_SUMMARY)

        expected_runs = {
            "nagoya/run1",
            "nagoya/run2",
            "nagoya/run3",
            "tokyo/run1",
            "tokyo/run2",
            "tokyo/run3",
        }
        self.assertEqual(payload["status"], "ok")
        self.assertEqual(payload["failures"], [])
        self.assertEqual(payload["max_epochs"], 200)
        self.assertEqual(payload["skip_epochs"], 0)
        self.assertTrue(payload["generate_spp_seed"])
        self.assertEqual(set(payload["runs"]), expected_runs)

        for run_name, run_payload in payload["runs"].items():
            native = run_payload["native_summary"]
            reference = run_payload["reference_summary"]
            self.assertEqual(run_payload["status"], "ok", run_name)
            self.assertEqual(run_payload["failures"], [], run_name)
            self.assertEqual(native["preset"], "taroz-amb-pdc", run_name)
            self.assertEqual(native["backend"], "eigen", run_name)
            self.assertTrue(native["converged"], run_name)
            self.assertEqual(native["optimized_epochs"], 200, run_name)
            self.assertEqual(native["valid_solutions"], 200, run_name)
            self.assertEqual(native["seed_matched_epochs"], 200, run_name)
            self.assertEqual(native["seed_interpolated_epochs"], 0, run_name)
            self.assertEqual(native["lambda_ambiguity_attempts"], 200, run_name)
            self.assertGreater(native["double_difference_carrier_factors"], 0, run_name)
            self.assertEqual(reference["matched_epochs"], 200, run_name)
            self.assertLessEqual(reference["p95_3d_error_m"], 2.0, run_name)
            self.assertLessEqual(reference["max_3d_error_m"], 2.0, run_name)

        nagoya_run3 = payload["runs"]["nagoya/run3"]["native_summary"]
        self.assertEqual(nagoya_run3["fixed_solutions"], 24)
        self.assertEqual(nagoya_run3["float_solutions"], 176)
        self.assertLessEqual(nagoya_run3["fix_rate_percent"], 15.0)

    def test_optional_nagoya_run3_shifted_window_summary_contract(self) -> None:
        full_payload = self.require_summary(FULL_SEED_SUMMARY)
        shifted_payload = self.require_summary(SHIFTED_NAGOYA_RUN3_SEED_SUMMARY)

        self.assertEqual(shifted_payload["status"], "ok")
        self.assertEqual(shifted_payload["failures"], [])
        self.assertEqual(shifted_payload["max_epochs"], 200)
        self.assertEqual(shifted_payload["skip_epochs"], 400)
        self.assertTrue(shifted_payload["generate_spp_seed"])
        self.assertEqual(set(shifted_payload["runs"]), {"nagoya/run3"})

        initial_native = full_payload["runs"]["nagoya/run3"]["native_summary"]
        shifted_run = shifted_payload["runs"]["nagoya/run3"]
        shifted_native = shifted_run["native_summary"]
        shifted_reference = shifted_run["reference_summary"]

        self.assertEqual(shifted_run["status"], "ok")
        self.assertEqual(shifted_run["failures"], [])
        self.assertEqual(shifted_native["optimized_epochs"], 200)
        self.assertEqual(shifted_native["valid_solutions"], 200)
        self.assertEqual(shifted_native["seed_matched_epochs"], 200)
        self.assertEqual(shifted_native["seed_interpolated_epochs"], 0)
        self.assertEqual(shifted_native["lambda_ambiguity_attempts"], 200)
        self.assertEqual(shifted_native["fixed_solutions"], 194)
        self.assertEqual(shifted_native["float_solutions"], 6)
        self.assertGreater(shifted_native["fixed_solutions"], initial_native["fixed_solutions"])
        self.assertEqual(shifted_reference["position_status_counts"], {"3": 6, "4": 194})
        self.assertEqual(shifted_reference["matched_epochs"], 200)
        self.assertLessEqual(shifted_reference["p95_3d_error_m"], 0.2)
        self.assertLessEqual(shifted_reference["max_3d_error_m"], 1.3)

    def test_optional_generated_seed_state_matches_epoch_debug(self) -> None:
        payload = self.require_summary(FULL_SEED_SUMMARY)
        shifted_payload = self.require_summary(SHIFTED_NAGOYA_RUN3_SEED_SUMMARY)

        for run_payload in payload["runs"].values():
            self.assert_seed_state_matches_epoch_debug(run_payload)
        self.assert_seed_state_matches_epoch_debug(shifted_payload["runs"]["nagoya/run3"])

    def test_optional_generated_seed_solution_artifacts_match_summary(self) -> None:
        payload = self.require_summary(FULL_SEED_SUMMARY)
        shifted_payload = self.require_summary(SHIFTED_NAGOYA_RUN3_SEED_SUMMARY)

        for run_payload in payload["runs"].values():
            self.assert_solution_artifacts_match_summary(run_payload)
        self.assert_solution_artifacts_match_summary(shifted_payload["runs"]["nagoya/run3"])

    def test_optional_nagoya_run2_1000_epoch_summary_contract(self) -> None:
        payload = self.require_summary(NAGOYA_RUN2_1000_SEED_SUMMARY)

        self.assertEqual(payload["status"], "ok")
        self.assertEqual(payload["failures"], [])
        self.assertEqual(payload["max_epochs"], 1000)
        self.assertEqual(payload["skip_epochs"], 0)
        self.assertTrue(payload["generate_spp_seed"])
        self.assertEqual(set(payload["runs"]), {"nagoya/run2"})

        run_payload = payload["runs"]["nagoya/run2"]
        native = run_payload["native_summary"]
        reference = run_payload["reference_summary"]
        audit = run_payload["generated_seed_audit"]
        self.assertEqual(run_payload["status"], "ok")
        self.assertEqual(run_payload["failures"], [])
        self.assertTrue(native["converged"])
        self.assertEqual(native["optimized_epochs"], 1000)
        self.assertEqual(native["valid_solutions"], 1000)
        self.assertEqual(native["fixed_solutions"], 777)
        self.assertEqual(native["float_solutions"], 223)
        self.assertEqual(native["seed_matched_epochs"], 1000)
        self.assertEqual(native["seed_interpolated_epochs"], 0)
        self.assertEqual(native["lambda_ambiguity_attempts"], 1000)
        self.assertEqual(native["lambda_ambiguity_candidates"], 18741)
        self.assertEqual(native["lambda_ambiguity_used_candidates"], 14389)
        self.assertEqual(native["double_difference_carrier_factors"], 18741)
        self.assertEqual(native["float_rejected_seed_position_divergence"], 0)
        self.assertEqual(native["float_rejected_position_jump"], 0)
        self.assertEqual(reference["position_status_counts"], {"3": 223, "4": 777})
        self.assertEqual(reference["matched_epochs"], 1000)
        self.assertLessEqual(reference["p95_3d_error_m"], 0.25)
        self.assertLessEqual(reference["max_3d_error_m"], 0.33)
        self.assertEqual(reference["tail_counts_3d_error_m"]["gt_0_5_m"], 0)
        self.assertEqual(reference["fixed"]["matched_epochs"], 777)
        self.assertLessEqual(reference["fixed"]["p95_3d_error_m"], 0.22)
        self.assertLessEqual(reference["fixed"]["max_3d_error_m"], 0.23)
        self.assertEqual(reference["fixed"]["tail_counts_3d_error_m"]["gt_0_5_m"], 0)
        self.assertEqual(reference["float"]["matched_epochs"], 223)
        self.assertLessEqual(reference["float"]["p95_3d_error_m"], 0.27)
        self.assertLessEqual(reference["float"]["max_3d_error_m"], 0.33)
        self.assertEqual(reference["float"]["tail_counts_3d_error_m"]["gt_0_5_m"], 0)
        self.assertEqual(reference["no_solution"]["matched_epochs"], 0)
        self.assertEqual(reference["all_output"]["matched_epochs"], 1000)
        self.assertEqual(reference["worst_epoch"]["gps_tow"], 555825.8)
        self.assertEqual(reference["worst_epoch"]["status"], 3)
        self.assertEqual(audit["seed_rows"], 1000)
        self.assertEqual(audit["expected_seed_rows"], 1000)
        self.assertEqual(audit["missing_optimized_seed_epochs"], 0)
        self.assertTrue(audit["window_sequence_matches"])
        self.assert_solution_artifacts_match_summary(run_payload)

    def test_optional_nagoya_run2_1000_epoch_lambda_debug_contract(self) -> None:
        payload = self.require_summary(NAGOYA_RUN2_1000_SEED_SUMMARY)

        self.assert_lambda_debug_matches_epoch_debug(payload["runs"]["nagoya/run2"])

    def test_optional_nagoya_run2_1000_epoch_cost_trace_contract(self) -> None:
        payload = self.require_summary(NAGOYA_RUN2_1000_SEED_SUMMARY)

        self.assert_cost_trace_matches_summary(payload["runs"]["nagoya/run2"])

    def test_optional_nagoya_run3_1000_epoch_summary_contract(self) -> None:
        payload = self.require_summary(NAGOYA_RUN3_1000_SEED_SUMMARY)

        self.assertEqual(payload["status"], "ok")
        self.assertEqual(payload["failures"], [])
        self.assertEqual(payload["max_epochs"], 1000)
        self.assertEqual(payload["skip_epochs"], 0)
        self.assertTrue(payload["generate_spp_seed"])
        self.assertEqual(set(payload["runs"]), {"nagoya/run3"})

        run_payload = payload["runs"]["nagoya/run3"]
        native = run_payload["native_summary"]
        reference = run_payload["reference_summary"]
        self.assertEqual(run_payload["status"], "ok")
        self.assertEqual(run_payload["failures"], [])
        self.assertEqual(native["optimized_epochs"], 1000)
        self.assertEqual(native["seed_matched_epochs"], 1000)
        self.assertEqual(native["seed_interpolated_epochs"], 0)
        self.assertEqual(native["valid_solutions"], 956)
        self.assertEqual(native["fixed_solutions"], 582)
        self.assertEqual(native["float_solutions"], 374)
        self.assertEqual(native["lambda_ambiguity_attempts"], 956)
        self.assertEqual(native["float_rejected_seed_position_divergence"], 0)
        self.assertEqual(native["float_rejected_position_jump"], 0)
        self.assertEqual(reference["position_status_counts"], {"0": 44, "3": 374, "4": 582})
        self.assertEqual(reference["matched_epochs"], 956)
        self.assertLessEqual(reference["p95_3d_error_m"], 1.1)
        self.assertLessEqual(reference["fixed"]["p95_3d_error_m"], 0.2)
        self.assertLessEqual(reference["float"]["p95_3d_error_m"], 1.2)
        self.assertEqual(reference["fixed"]["tail_counts_3d_error_m"]["gt_100_m"], 0)
        self.assertEqual(reference["float"]["tail_counts_3d_error_m"]["gt_2_m"], 0)
        self.assertEqual(reference["worst_epoch"]["gps_tow"], 553834.4)
        self.assertEqual(reference["worst_epoch"]["status"], 3)
        self.assertEqual(reference["no_solution"]["matched_epochs"], 44)
        self.assertEqual(reference["no_solution"]["tail_counts_3d_error_m"]["gt_100_m"], 0)
        self.assertEqual(reference["no_solution"]["tail_counts_3d_error_m"]["gt_10_m"], 0)
        self.assertEqual(reference["no_solution"]["tail_counts_3d_error_m"]["gt_2_m"], 0)
        self.assertEqual(reference["no_solution"]["tail_counts_3d_error_m"]["gt_1_m"], 7)
        self.assertEqual(reference["no_solution"]["worst_epoch"]["status"], 0)
        self.assertEqual(reference["no_solution"]["worst_epoch"]["gps_tow"], 553865.4)
        self.assertEqual(reference["all_output"]["worst_epoch"]["status"], 0)
        self.assertEqual(reference["all_output"]["worst_epoch"]["gps_tow"], 553865.4)
        self.assert_solution_artifacts_match_summary(run_payload)

    def test_optional_nagoya_run3_1000_epoch_lambda_debug_contract(self) -> None:
        payload = self.require_summary(NAGOYA_RUN3_1000_SEED_SUMMARY)

        self.assert_lambda_debug_matches_epoch_debug(payload["runs"]["nagoya/run3"])

    def test_optional_nagoya_run3_1000_epoch_cost_trace_contract(self) -> None:
        payload = self.require_summary(NAGOYA_RUN3_1000_SEED_SUMMARY)

        self.assert_cost_trace_matches_summary(payload["runs"]["nagoya/run3"])

    def test_optional_tokyo_run2_1000_epoch_summary_contract(self) -> None:
        payload = self.require_summary(TOKYO_RUN2_1000_SEED_SUMMARY)

        self.assertEqual(payload["status"], "ok")
        self.assertEqual(payload["failures"], [])
        self.assertEqual(payload["max_epochs"], 1000)
        self.assertEqual(payload["skip_epochs"], 0)
        self.assertTrue(payload["generate_spp_seed"])
        self.assertEqual(set(payload["runs"]), {"tokyo/run2"})

        run_payload = payload["runs"]["tokyo/run2"]
        native = run_payload["native_summary"]
        reference = run_payload["reference_summary"]
        audit = run_payload["generated_seed_audit"]
        self.assertEqual(run_payload["status"], "ok")
        self.assertEqual(run_payload["failures"], [])
        self.assertTrue(native["converged"])
        self.assertEqual(native["optimized_epochs"], 1000)
        self.assertEqual(native["valid_solutions"], 1000)
        self.assertEqual(native["fixed_solutions"], 868)
        self.assertEqual(native["float_solutions"], 132)
        self.assertEqual(native["seed_matched_epochs"], 1000)
        self.assertEqual(native["seed_interpolated_epochs"], 0)
        self.assertEqual(native["lambda_ambiguity_attempts"], 1000)
        self.assertEqual(native["lambda_ambiguity_candidates"], 17675)
        self.assertEqual(native["lambda_ambiguity_used_candidates"], 15960)
        self.assertEqual(native["double_difference_carrier_factors"], 17675)
        self.assertEqual(native["float_rejected_seed_position_divergence"], 0)
        self.assertEqual(native["float_rejected_position_jump"], 0)
        self.assertEqual(reference["position_status_counts"], {"3": 132, "4": 868})
        self.assertEqual(reference["matched_epochs"], 1000)
        self.assertLessEqual(reference["p95_3d_error_m"], 1.6)
        self.assertLessEqual(reference["max_3d_error_m"], 1.7)
        self.assertEqual(reference["fixed"]["matched_epochs"], 868)
        self.assertLessEqual(reference["fixed"]["p95_3d_error_m"], 0.1)
        self.assertLessEqual(reference["fixed"]["max_3d_error_m"], 0.18)
        self.assertEqual(reference["fixed"]["tail_counts_3d_error_m"]["gt_0_5_m"], 0)
        self.assertEqual(reference["float"]["matched_epochs"], 132)
        self.assertLessEqual(reference["float"]["p95_3d_error_m"], 1.7)
        self.assertEqual(reference["float"]["tail_counts_3d_error_m"]["gt_2_m"], 0)
        self.assertEqual(reference["no_solution"]["matched_epochs"], 0)
        self.assertEqual(reference["all_output"]["matched_epochs"], 1000)
        self.assertEqual(reference["worst_epoch"]["gps_tow"], 177121.0)
        self.assertEqual(reference["worst_epoch"]["status"], 3)
        self.assertEqual(audit["seed_rows"], 1000)
        self.assertEqual(audit["expected_seed_rows"], 1000)
        self.assertEqual(audit["missing_optimized_seed_epochs"], 0)
        self.assertTrue(audit["window_sequence_matches"])
        self.assert_solution_artifacts_match_summary(run_payload)

    def test_optional_tokyo_run2_1000_epoch_lambda_debug_contract(self) -> None:
        payload = self.require_summary(TOKYO_RUN2_1000_SEED_SUMMARY)

        self.assert_lambda_debug_matches_epoch_debug(payload["runs"]["tokyo/run2"])

    def test_optional_tokyo_run2_1000_epoch_cost_trace_contract(self) -> None:
        payload = self.require_summary(TOKYO_RUN2_1000_SEED_SUMMARY)

        self.assert_cost_trace_matches_summary(payload["runs"]["tokyo/run2"])


if __name__ == "__main__":
    unittest.main()
