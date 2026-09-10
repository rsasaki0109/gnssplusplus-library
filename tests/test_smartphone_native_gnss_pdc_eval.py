#!/usr/bin/env python3
"""Tests for the fixed timestamp-recovery evaluator."""

from __future__ import annotations

import csv
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = (
    ROOT
    / "apps"
    / "commands"
    / "benchmarks"
    / "gnss_smartphone_native_gnss_pdc_eval.py"
)
SPEC = importlib.util.spec_from_file_location(
    "gnss_smartphone_native_gnss_pdc_eval", MODULE_PATH
)
assert SPEC is not None and SPEC.loader is not None
EVALUATOR = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = EVALUATOR
SPEC.loader.exec_module(EVALUATOR)


FIELDS = ["phone", "UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees"]


def write_csv(path: Path, fields: list[str], rows: list[dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class SmartphoneNativeGnssPdcEvalTests(unittest.TestCase):
    def test_fixed_999ms_alignment_is_one_to_one_and_scores_four_variants(self) -> None:
        with tempfile.TemporaryDirectory(prefix="native_pdc_eval_") as temp_dir:
            root = Path(temp_dir)
            phone = "fixture/phone"
            candidate = root / "keyed.csv"
            write_csv(
                candidate,
                FIELDS,
                [
                    {"phone": phone, "UnixTimeMillis": 1000, "LatitudeDegrees": 35.0, "LongitudeDegrees": 139.0},
                    {"phone": phone, "UnixTimeMillis": 2000, "LatitudeDegrees": 35.0, "LongitudeDegrees": 139.0},
                    {"phone": phone, "UnixTimeMillis": 3000, "LatitudeDegrees": 35.0, "LongitudeDegrees": 139.0},
                    {"phone": phone, "UnixTimeMillis": 4000, "LatitudeDegrees": 35.0, "LongitudeDegrees": 139.0},
                ],
            )
            run_manifest = root / "run_manifest.json"
            run_manifest.write_text(
                json.dumps(
                    {
                        "schema_version": "smartphone-r5-native-gnss-pdc-run-manifest.v1",
                        "status": "truth-free-artifacts-sealed",
                        "truth_free": True,
                        "trip_id": phone,
                        "forbidden_input_policy": {
                            "mat_paths_rejected_before_open": True,
                            "result_coordinates_read": False,
                            "ground_truth_read": False,
                            "sample_coordinates_read": False,
                            "v5_output_read": False,
                            "python_coordinate_or_rinex_stage": False,
                            "base_or_double_difference_factors": False,
                        },
                        "artifacts": {
                            "keyed.csv": {"sha256": sha256(candidate)},
                        },
                        "summary": {"pseudorange_factors": 3},
                    }
                ),
                encoding="utf-8",
            )
            recovery = root / "recovery.json"
            recovery.write_text(
                json.dumps(
                    {
                        "schema_version": EVALUATOR.RECOVERY_SCHEMA_VERSION,
                        "status": "authorized-evaluation-recovery",
                        "candidate": {
                            "route_device": phone,
                            "keyed_output_sha256": sha256(candidate),
                            "run_manifest_sha256": sha256(run_manifest),
                        },
                        "contract": {
                            "candidate_unchanged": True,
                            "parameter_selection": False,
                        },
                        "exact_failure": "fixed fixture failure",
                    }
                ),
                encoding="utf-8",
            )
            truth = root / "ground_truth.csv"
            write_csv(
                truth,
                FIELDS,
                [
                    {"phone": phone, "UnixTimeMillis": 1999, "LatitudeDegrees": 35.0, "LongitudeDegrees": 139.0},
                    {"phone": phone, "UnixTimeMillis": 2999, "LatitudeDegrees": 35.0, "LongitudeDegrees": 139.0},
                    {"phone": phone, "UnixTimeMillis": 3999, "LatitudeDegrees": 35.0, "LongitudeDegrees": 139.0},
                ],
            )
            report = EVALUATOR.evaluate(
                candidate,
                run_manifest,
                truth,
                recovery,
                root / "score.json",
                phone,
            )
            self.assertEqual(report["timestamp_alignment"]["matched_rows"], 3)
            self.assertEqual(report["timestamp_alignment"]["unmatched_prediction_rows"], 1)
            self.assertEqual(report["timestamp_alignment"]["delta_ms"]["max"], 999)
            self.assertEqual(report["metrics"]["phone_count_scored"], 1)
            self.assertEqual(len(report["metrics"]["score_variants_m"]), 4)
            self.assertEqual(report["truth"]["read_method"], "single-open-bytes-parse-and-hash")

    def test_nearest_tie_fails_closed(self) -> None:
        rows = [
            EVALUATOR.metric.CoordinateRow("fixture/phone", 1500, 35.0, 139.0, 2),
        ]
        truth = [
            EVALUATOR.metric.CoordinateRow("fixture/phone", 1000, 35.0, 139.0, 2),
            EVALUATOR.metric.CoordinateRow("fixture/phone", 2000, 35.0, 139.0, 3),
        ]
        with self.assertRaisesRegex(ValueError, "ambiguous nearest"):
            EVALUATOR._nearest_one_to_one(rows, truth)

    def test_mat_path_rejected_before_open(self) -> None:
        with tempfile.TemporaryDirectory(prefix="native_pdc_eval_mat_guard_") as temp_dir:
            poison = Path(temp_dir) / "result_gnss.mat"
            poison.write_bytes(b"must not be opened")
            with self.assertRaisesRegex(ValueError, "never opened"):
                EVALUATOR.evaluate(
                    poison,
                    Path(temp_dir) / "manifest.json",
                    Path(temp_dir) / "ground_truth.csv",
                    Path(temp_dir) / "recovery.json",
                    Path(temp_dir) / "score.json",
                    "fixture/phone",
                )


if __name__ == "__main__":
    unittest.main()
