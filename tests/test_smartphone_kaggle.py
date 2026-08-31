#!/usr/bin/env python3

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
MODULE_PATH = ROOT / "apps" / "commands" / "benchmarks" / "gnss_smartphone_kaggle.py"
SPEC = importlib.util.spec_from_file_location("gnss_smartphone_kaggle", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
KAGGLE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = KAGGLE
SPEC.loader.exec_module(KAGGLE)


SUBMISSION_FIELDS = [
    "phone",
    "UnixTimeMillis",
    "LatitudeDegrees",
    "LongitudeDegrees",
]
TRUTH_FIELDS = [
    "phone",
    "UnixTimeMillis",
    "LatitudeDegrees",
    "LongitudeDegrees",
]


def write_rows(path: Path, fields: list[str], rows: list[dict[str, object]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


class SmartphoneKaggleTests(unittest.TestCase):
    def test_pinned_matlab_score_fixture_matches_hand_and_numpy_reference(self) -> None:
        fixture_path = ROOT / "tests" / "fixtures" / "smartphone_kaggle_metric_parity.json"
        fixture = json.loads(fixture_path.read_text(encoding="utf-8"))
        radius_m = fixture["haversine_radius_m"]
        for pair in fixture["haversine_pairs"]:
            prediction = pair["prediction"]
            truth = pair["truth"]
            actual = KAGGLE._haversine_horizontal_distance_m(
                prediction[0],
                prediction[1],
                truth[0],
                truth[1],
                radius_m=radius_m,
            )
            self.assertAlmostEqual(actual, pair["expected_distance_m"], places=10)

        values = fixture["percentile_values_m"]
        self.assertAlmostEqual(
            KAGGLE._percentile_linear_n_minus_1(values, 0.50),
            fixture["expected_linear_percentile_m"]["0.50"],
        )
        self.assertAlmostEqual(
            KAGGLE._percentile_linear_n_minus_1(values, 0.95),
            fixture["expected_linear_percentile_m"]["0.95"],
        )
        self.assertAlmostEqual(
            (
                KAGGLE._percentile_linear_n_minus_1(values, 0.50)
                + KAGGLE._percentile_linear_n_minus_1(values, 0.95)
            )
            / 2.0,
            fixture["expected_phone_score_m"],
        )

        try:
            import numpy as np
        except ImportError:
            self.skipTest("NumPy is unavailable for the independent parity witness")
        try:
            numpy_p50 = float(np.percentile(values, 50.0, method="linear"))
            numpy_p95 = float(np.percentile(values, 95.0, method="linear"))
        except TypeError:
            numpy_p50 = float(np.percentile(values, 50.0, interpolation="linear"))
            numpy_p95 = float(np.percentile(values, 95.0, interpolation="linear"))
        self.assertAlmostEqual(numpy_p50, fixture["expected_linear_percentile_m"]["0.50"])
        self.assertAlmostEqual(numpy_p95, fixture["expected_linear_percentile_m"]["0.95"])

    def test_generator_is_truth_free_and_uses_exact_floor_keys(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_smartphone_kaggle_") as temp_dir:
            root = Path(temp_dir)
            position = root / "solution.pos"
            device = root / "device_gnss.csv"
            first = KAGGLE._position_timestamp("2200", "100.125", 18, 1)
            second = KAGGLE._position_timestamp("2200", "101.125", 18, 2)
            position.write_text(
                "\n".join(
                    [
                        "% GPS solution",
                        f"2200 100.125 0 0 0 35.0 139.0 0 1 8",
                        f"2200 101.125 0 0 0 35.1 139.1 0 1 8",
                    ]
                )
                + "\n",
                encoding="ascii",
            )
            write_rows(
                device,
                ["MessageType", "utcTimeMillis", "Svid"],
                [
                    {"MessageType": "Raw", "utcTimeMillis": first, "Svid": 1},
                    {"MessageType": "Raw", "utcTimeMillis": first, "Svid": 2},
                    {"MessageType": "Raw", "utcTimeMillis": second, "Svid": 1},
                ],
            )
            output = root / "submission.csv"
            manifest_path = root / "submission.manifest.json"
            manifest = KAGGLE.generate_submission(
                position,
                output,
                "fixture-phone",
                device_gnss_path=device,
                manifest_path=manifest_path,
            )

            with output.open(encoding="utf-8", newline="") as handle:
                reader = csv.reader(handle)
                self.assertEqual(next(reader), SUBMISSION_FIELDS)
                rows = list(reader)
            self.assertEqual([int(row[1]) for row in rows], [first, second])
            self.assertEqual(manifest["compatibility"], KAGGLE.COMPATIBILITY)
            self.assertFalse(manifest["generator_contract"]["truth_used"])
            self.assertNotIn("ground_truth", manifest["inputs"])
            self.assertEqual(
                manifest["artifacts"]["submission"]["sha256"],
                hashlib.sha256(output.read_bytes()).hexdigest(),
            )

    def test_metric_scores_each_phone_with_declared_linear_percentiles(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_smartphone_kaggle_") as temp_dir:
            root = Path(temp_dir)
            truth = root / "ground_truth.csv"
            submission = root / "submission.csv"
            report_path = root / "report.json"
            truth_rows: list[dict[str, object]] = []
            submission_rows: list[dict[str, object]] = []
            offsets = {"phone-a": [0.0, 0.00001, 0.00002], "phone-b": [0.0, 0.00003, 0.00004]}
            for phone, phone_offsets in offsets.items():
                for index, offset in enumerate(phone_offsets):
                    timestamp = 1000 + index * 1000
                    truth_rows.append(
                        {
                            "phone": phone,
                            "UnixTimeMillis": timestamp,
                            "LatitudeDegrees": 35.0,
                            "LongitudeDegrees": 139.0,
                        }
                    )
                    submission_rows.append(
                        {
                            "phone": phone,
                            "UnixTimeMillis": timestamp,
                            "LatitudeDegrees": 35.0 + offset,
                            "LongitudeDegrees": 139.0,
                        }
                    )
            write_rows(truth, TRUTH_FIELDS, truth_rows)
            write_rows(submission, SUBMISSION_FIELDS, submission_rows)

            report = KAGGLE.evaluate_submission(submission, truth, report_path)

            self.assertEqual(report["compatibility"], KAGGLE.COMPATIBILITY)
            self.assertTrue(report["key_validation"]["complete_key_match"])
            self.assertEqual(report["metrics"]["phone_count_scored"], 2)
            self.assertEqual(report["metrics"]["coverage_ratio"], 1.0)
            self.assertIsNone(report["metrics"]["kaggle_metric_m"])
            self.assertIsNone(report["metrics"]["primary_score_m"])
            self.assertEqual(
                report["metrics"]["primary_score_status"],
                KAGGLE.PRIMARY_SCORE_STATUS,
            )
            self.assertFalse(
                report["metric_contract"]["official_public_source"][
                    "distance_formula_published"
                ]
            )
            self.assertFalse(
                report["metric_contract"]["official_public_source"][
                    "percentile_interpolation_published"
                ]
            )
            self.assertEqual(
                report["metric_contract"]["distance"]["variants"][
                    "haversine_sphere"
                ]["earth_radius_m"],
                KAGGLE.HAVERSINE_EARTH_RADIUS_M,
            )
            self.assertIsNone(
                report["metric_contract"]["distance"]["primary_variant"]
            )
            distance_functions = {
                "wgs84_vincenty": KAGGLE._wgs84_horizontal_distance_m,
                "haversine_sphere": KAGGLE._haversine_horizontal_distance_m,
            }
            percentile_functions = {
                "linear_n_minus_1": KAGGLE._percentile_linear_n_minus_1,
                "nearest_rank_ceiling": KAGGLE._percentile_nearest_rank_ceiling,
            }
            expected_phone_scores: dict[str, list[float]] = {
                KAGGLE._score_variant_id(distance_variant, percentile_variant): []
                for distance_variant in distance_functions
                for percentile_variant in percentile_functions
            }
            for phone, phone_offsets in offsets.items():
                for distance_variant, distance_function in distance_functions.items():
                    distances = [
                        distance_function(35.0 + offset, 139.0, 35.0, 139.0)
                        for offset in phone_offsets
                    ]
                    for percentile_variant, percentile_function in percentile_functions.items():
                        expected_p50 = percentile_function(distances, 0.50)
                        expected_p95 = percentile_function(distances, 0.95)
                        variant_id = KAGGLE._score_variant_id(
                            distance_variant, percentile_variant
                        )
                        expected_phone_score = (expected_p50 + expected_p95) / 2.0
                        expected_phone_scores[variant_id].append(expected_phone_score)
                        actual = report["phones"][phone]["score_variants"][variant_id]
                        self.assertAlmostEqual(actual["p50_m"], expected_p50)
                        self.assertAlmostEqual(actual["p95_m"], expected_p95)
                        self.assertAlmostEqual(
                            actual["phone_score_m"], expected_phone_score
                        )
            for variant_id, phone_scores in expected_phone_scores.items():
                self.assertAlmostEqual(
                    report["metrics"]["score_variants_m"][variant_id],
                    sum(phone_scores) / len(phone_scores),
                )
            self.assertIsNone(report["metric_contract"]["percentile"]["primary_variant"])
            self.assertEqual(
                report["metric_contract"]["percentile"]["variants"][
                    "linear_n_minus_1"
                ]["method"],
                "linear interpolation at rank (n - 1) * q",
            )
            self.assertEqual(
                report["inputs"]["submission"]["sha256"],
                hashlib.sha256(submission.read_bytes()).hexdigest(),
            )
            self.assertEqual(
                json.loads(report_path.read_text(encoding="utf-8"))["metrics"],
                report["metrics"],
            )

    def test_haversine_radius_is_explicit_and_nearest_rank_is_declared(self) -> None:
        expected = 2.0 * KAGGLE.HAVERSINE_EARTH_RADIUS_M * (
            3.141592653589793 / 360.0
        )
        self.assertAlmostEqual(
            KAGGLE._haversine_horizontal_distance_m(0.0, 0.0, 0.0, 1.0),
            expected,
        )
        self.assertEqual(
            KAGGLE._percentile_nearest_rank_ceiling([1.0, 2.0, 3.0, 4.0], 0.95),
            4.0,
        )
        self.assertAlmostEqual(
            KAGGLE._percentile_linear_n_minus_1([1.0, 2.0, 3.0, 4.0], 0.95),
            3.85,
        )

    def test_fail_closed_for_duplicate_extra_missing_and_nonfinite_keys(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_smartphone_kaggle_") as temp_dir:
            root = Path(temp_dir)
            truth = root / "ground_truth.csv"
            write_rows(
                truth,
                TRUTH_FIELDS,
                [
                    {"phone": "phone-a", "UnixTimeMillis": 1000, "LatitudeDegrees": 0, "LongitudeDegrees": 0},
                    {"phone": "phone-a", "UnixTimeMillis": 2000, "LatitudeDegrees": 0, "LongitudeDegrees": 0},
                ],
            )

            duplicate = root / "duplicate.csv"
            write_rows(
                duplicate,
                SUBMISSION_FIELDS,
                [
                    {"phone": "phone-a", "UnixTimeMillis": 1000, "LatitudeDegrees": 0, "LongitudeDegrees": 0},
                    {"phone": "phone-a", "UnixTimeMillis": 1000, "LatitudeDegrees": 0, "LongitudeDegrees": 0},
                ],
            )
            with self.assertRaisesRegex(ValueError, "duplicate submission key"):
                KAGGLE.evaluate_submission(duplicate, truth, root / "duplicate.json")

            extra = root / "extra.csv"
            write_rows(
                extra,
                SUBMISSION_FIELDS,
                [
                    {"phone": "phone-a", "UnixTimeMillis": 1000, "LatitudeDegrees": 0, "LongitudeDegrees": 0},
                    {"phone": "phone-a", "UnixTimeMillis": 2000, "LatitudeDegrees": 0, "LongitudeDegrees": 0},
                    {"phone": "phone-a", "UnixTimeMillis": 3000, "LatitudeDegrees": 0, "LongitudeDegrees": 0},
                ],
            )
            with self.assertRaisesRegex(ValueError, "keys absent from ground truth"):
                KAGGLE.evaluate_submission(extra, truth, root / "extra.json")

            missing = root / "missing.csv"
            write_rows(
                missing,
                SUBMISSION_FIELDS,
                [{"phone": "phone-a", "UnixTimeMillis": 1000, "LatitudeDegrees": 0, "LongitudeDegrees": 0}],
            )
            with self.assertRaisesRegex(ValueError, "missing 1 ground-truth keys"):
                KAGGLE.evaluate_submission(missing, truth, root / "missing.json")
            sparse_report = KAGGLE.evaluate_submission(
                missing, truth, root / "sparse.json", allow_gaps=True
            )
            self.assertEqual(sparse_report["key_validation"]["missing_truth_keys"], 1)
            self.assertFalse(sparse_report["key_validation"]["complete_key_match"])
            self.assertIsNone(sparse_report["metrics"]["primary_score_m"])

            nonfinite = root / "nonfinite.csv"
            write_rows(
                nonfinite,
                SUBMISSION_FIELDS,
                [
                    {"phone": "phone-a", "UnixTimeMillis": 1000, "LatitudeDegrees": "nan", "LongitudeDegrees": 0},
                    {"phone": "phone-a", "UnixTimeMillis": 2000, "LatitudeDegrees": 0, "LongitudeDegrees": 0},
                ],
            )
            with self.assertRaisesRegex(ValueError, "must be finite"):
                KAGGLE.evaluate_submission(nonfinite, truth, root / "nonfinite.json")


if __name__ == "__main__":
    unittest.main()
