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
NAGOYA_RUN3_1000_SEED_SUMMARY = (
    ROOT_DIR
    / "output/dogfood/ppc_taroz_amb_pdc_nagoya_run3_1000_seed_current/summary.json"
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
            self.assertIn("--max-epochs", command)
            self.assertIn("20", command)

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
            self.assertEqual(summary["no_solution"]["matched_epochs"], 1)
            self.assertEqual(summary["all_output"]["max_3d_error_m"], 1000.0)

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
        self.assertEqual(nagoya_run3["fixed_solutions"], 18)
        self.assertEqual(nagoya_run3["float_solutions"], 182)
        self.assertLessEqual(nagoya_run3["fix_rate_percent"], 10.0)

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
        self.assertGreater(shifted_native["fixed_solutions"], initial_native["fixed_solutions"])
        self.assertGreaterEqual(shifted_native["fixed_solutions"], 160)
        self.assertLessEqual(shifted_native["float_solutions"], 40)
        self.assertEqual(shifted_reference["matched_epochs"], 200)
        self.assertLessEqual(shifted_reference["p95_3d_error_m"], 2.0)
        self.assertLessEqual(shifted_reference["max_3d_error_m"], 2.0)

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
        self.assertEqual(native["valid_solutions"], 783)
        self.assertEqual(native["fixed_solutions"], 406)
        self.assertEqual(native["float_solutions"], 377)
        self.assertEqual(native["lambda_ambiguity_attempts"], 949)
        self.assertEqual(native["float_rejected_seed_position_divergence"], 36)
        self.assertEqual(native["float_rejected_position_jump"], 130)
        self.assertEqual(reference["position_status_counts"], {"0": 217, "3": 377, "4": 406})
        self.assertEqual(reference["matched_epochs"], 783)
        self.assertLessEqual(reference["p95_3d_error_m"], 1.1)
        self.assertLessEqual(reference["fixed"]["p95_3d_error_m"], 0.2)
        self.assertLessEqual(reference["float"]["p95_3d_error_m"], 1.2)
        self.assertEqual(reference["no_solution"]["matched_epochs"], 217)
        self.assert_solution_artifacts_match_summary(run_payload)

    def test_optional_nagoya_run3_1000_epoch_lambda_debug_contract(self) -> None:
        payload = self.require_summary(NAGOYA_RUN3_1000_SEED_SUMMARY)

        self.assert_lambda_debug_matches_epoch_debug(payload["runs"]["nagoya/run3"])


if __name__ == "__main__":
    unittest.main()
