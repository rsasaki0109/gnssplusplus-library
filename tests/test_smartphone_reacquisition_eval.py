from __future__ import annotations

import tempfile
import unittest
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands"))
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_reacquisition_eval as EVAL  # noqa: E402
import gnss_smartphone_trajectory_smoother as SMOOTHER  # noqa: E402


class SmartphoneReacquisitionEvalTests(unittest.TestCase):
    def test_fixed_roles_exclude_sealed_holdout(self) -> None:
        self.assertEqual(
            EVAL.TRAIN_IDS,
            (
                "2021-01-04-21-50-us-ca-e1highway280driveroutea/mi8",
                "2022-08-04-20-07-us-ca-sjc-q/pixel5",
            ),
        )
        with self.assertRaises(EVAL.ReacquisitionError):
            EVAL._route_inputs(
                EVAL.HOLDOUT_ID,
                Path("/tmp/unused-generalization"),
                main_position=Path("/tmp/unused.pos"),
                main_device=Path("/tmp/unused.csv"),
                main_truth=Path("/tmp/unused-truth.csv"),
            )

    def test_aggregate_gate_and_candidate_rank_inputs(self) -> None:
        reference = {
            "mean_availability_ratio": 1.0,
            "mean_horizontal_wgs84_p50_m": 5.0,
            "mean_horizontal_wgs84_p95_m": 20.0,
            "mean_vertical_p95_abs_m": 10.0,
            "mean_kaggle_diagnostic_score_variants_m": {
                key: 10.0 for key in EVAL._DIAGNOSTIC_KEYS
            },
        }
        candidate = {
            **reference,
            "mean_horizontal_wgs84_p50_m": 4.0,
            "mean_horizontal_wgs84_p95_m": 8.0,
            "mean_vertical_p95_abs_m": 9.0,
            "mean_kaggle_diagnostic_score_variants_m": {
                key: 8.0 for key in EVAL._DIAGNOSTIC_KEYS
            },
        }
        passed, failures = EVAL._aggregate_non_regression(candidate, reference)
        self.assertTrue(passed)
        self.assertEqual(failures, [])

    def test_diagnostics_records_rejected_prediction_drift(self) -> None:
        week = 2200
        base = SMOOTHER._wgs84_geodetic_to_ecef(
            SMOOTHER.math.radians(35.0), SMOOTHER.math.radians(139.0), 40.0
        )
        epochs = [
            SMOOTHER._position_timestamp(str(week), f"{100 + i}.0", 18, i + 1)
            for i in range(5)
        ]
        positions = []
        baseline_rows = []
        for index, timestamp in enumerate(epochs):
            raw_ecef = base + SMOOTHER.np.array((float(index), 0.0, 0.0))
            raw_lat, raw_lon, raw_height = SMOOTHER._wgs84_ecef_to_geodetic(raw_ecef)
            positions.append(
                SMOOTHER.PositionRow(
                    week,
                    100.0 + index,
                    timestamp,
                    raw_ecef,
                    SMOOTHER.math.degrees(raw_lat),
                    SMOOTHER.math.degrees(raw_lon),
                    raw_height,
                    1,
                    10,
                    2.0,
                    0.0,
                    0,
                    3,
                    index + 1,
                )
            )
            drift_ecef = raw_ecef + (
                SMOOTHER.np.array((100.0, 0.0, 0.0)) if index in (2, 3) else SMOOTHER.np.zeros(3)
            )
            lat, lon, height = SMOOTHER._wgs84_ecef_to_geodetic(drift_ecef)
            baseline_rows.append(
                SMOOTHER.SmoothedRow(
                    timestamp_ms=timestamp,
                    week=week,
                    tow=100.0 + index,
                    ecef=drift_ecef,
                    latitude=SMOOTHER.math.degrees(lat),
                    longitude=SMOOTHER.math.degrees(lon),
                    height=height,
                    status=SMOOTHER.PROPAGATED_STATUS if index in (2, 3) else 1,
                    satellites=0 if index in (2, 3) else 10,
                    pdop=0.0 if index in (2, 3) else 2.0,
                    ratio=0.0,
                    fixed_ambiguities=0,
                    iterations=0,
                    source="outlier_rejected" if index in (2, 3) else "measured",
                    segment_id=0,
                    measurement_used=index not in (2, 3),
                    outlier_rejected=index in (2, 3),
                    innovation_sigma=6.0 if index in (2, 3) else 0.0,
                    position_sigma_m=1.0,
                )
            )
        with tempfile.TemporaryDirectory(prefix="reacquisition_eval_") as temp_dir:
            report = EVAL._diagnostics(
                Path(temp_dir),
                "fixture/phone",
                positions,
                epochs,
                baseline_rows,
                {timestamp: (35.0, 139.0, 40.0) for timestamp in epochs},
            )
            self.assertTrue(report["drift_evidence"]["prediction_drift_supported"])
            self.assertEqual(report["population"]["maximum_reject_run_epochs"], 2)
            self.assertTrue(
                (Path(temp_dir) / "routes" / "fixture__phone" / "trajectory_diagnostics.csv").is_file()
            )


if __name__ == "__main__":
    unittest.main()
