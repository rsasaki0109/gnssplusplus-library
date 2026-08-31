from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands"))
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_segment_stability as STABILITY  # noqa: E402
import gnss_smartphone_trajectory_smoother as SMOOTHER  # noqa: E402


class SmartphoneSegmentStabilityTests(unittest.TestCase):
    def _position(self, timestamp: int, offset: float) -> SMOOTHER.PositionRow:
        ecef = SMOOTHER._wgs84_geodetic_to_ecef(
            math.radians(35.0), math.radians(139.0), 40.0
        ) + np.array((offset, 0.0, 0.0))
        latitude, longitude, height = SMOOTHER._wgs84_ecef_to_geodetic(ecef)
        return SMOOTHER.PositionRow(
            week=2200,
            tow=100.0 + offset,
            timestamp_ms=timestamp,
            ecef=ecef,
            latitude=math.degrees(latitude),
            longitude=math.degrees(longitude),
            height=height,
            status=1,
            satellites=10,
            pdop=2.0,
            ratio=0.0,
            fixed_ambiguities=0,
            iterations=3,
            source_line=int(offset) + 1,
        )

    def _baseline(
        self,
        positions: list[SMOOTHER.PositionRow],
        rejected: tuple[bool, ...],
        measured: tuple[bool, ...] | None = None,
    ) -> SMOOTHER.SmoothingResult:
        measured = measured or tuple(not value for value in rejected)
        rows = []
        for index, (position, is_rejected, is_measured) in enumerate(
            zip(positions, rejected, measured)
        ):
            rows.append(
                SMOOTHER.SmoothedRow(
                    timestamp_ms=position.timestamp_ms,
                    week=position.week,
                    tow=position.tow,
                    ecef=position.ecef.copy(),
                    latitude=position.latitude,
                    longitude=position.longitude,
                    height=position.height,
                    status=7 if not is_measured else position.status,
                    satellites=0 if not is_measured else position.satellites,
                    pdop=0.0 if not is_measured else position.pdop,
                    ratio=0.0,
                    fixed_ambiguities=0,
                    iterations=0 if not is_measured else position.iterations,
                    source="outlier_rejected" if is_rejected else "measured",
                    segment_id=0,
                    measurement_used=is_measured,
                    outlier_rejected=is_rejected,
                    innovation_sigma=6.0 if is_rejected else 1.0,
                    position_sigma_m=1.0,
                )
            )
        return SMOOTHER.SmoothingResult(
            rows=rows,
            origin_ecef=positions[0].ecef.copy(),
            origin_latitude=positions[0].latitude,
            origin_longitude=positions[0].longitude,
            origin_height=positions[0].height,
            selected_device_epochs=len(rows),
            measured_epochs=sum(measured),
            synthesized_epochs=len(rows) - sum(measured),
            outlier_rejections=sum(rejected),
            segment_count=1,
            max_input_gap_s=1.0,
            max_position_gap_s=1.0,
            reset_indices=(),
            numerical_fallbacks=0,
            elapsed_s=0.01,
            max_consecutive_rejects=sum(rejected),
            max_prediction_duration_s=2.0,
        )

    def test_unstable_segment_falls_back_to_all_raw_coordinates(self) -> None:
        positions = [
            self._position(1694113198000 + 1000 * index, float(index))
            for index in range(4)
        ]
        baseline = self._baseline(positions, (False, True, True, False))
        result, report = STABILITY.apply_segment_stability(
            baseline,
            positions,
            [position.timestamp_ms for position in positions],
            max_consecutive_rejects=1,
            max_prediction_duration_s=1.0,
        )
        self.assertEqual(report["population"]["unstable_segment_count"], 1)
        self.assertEqual(report["population"]["fallback_epochs"], 4)
        self.assertEqual({row.source for row in result.rows}, {"raw_fallback"})
        for output, raw in zip(result.rows, positions):
            np.testing.assert_array_equal(output.ecef, raw.ecef)

    def test_stable_segment_retains_existing_rts_rows(self) -> None:
        positions = [
            self._position(1694113198000 + 1000 * index, float(index))
            for index in range(4)
        ]
        baseline = self._baseline(positions, (False, True, True, False))
        result, report = STABILITY.apply_segment_stability(
            baseline,
            positions,
            [position.timestamp_ms for position in positions],
            max_consecutive_rejects=2,
            max_prediction_duration_s=2.0,
        )
        self.assertEqual(report["population"]["stable_segment_count"], 1)
        self.assertEqual(report["population"]["fallback_epochs"], 0)
        self.assertEqual(result.rows, baseline.rows)

    def test_missing_device_key_uses_explicit_bracketed_raw_interpolation(self) -> None:
        raw_positions = [
            self._position(1694113198000, 0.0),
            self._position(1694113200000, 2.0),
        ]
        device_epochs = [1694113198000, 1694113199000, 1694113200000]
        middle = self._position(1694113199000, 1.0)
        baseline_positions = [raw_positions[0], middle, raw_positions[1]]
        baseline = self._baseline(
            baseline_positions,
            (False, True, True),
            (True, False, False),
        )
        result, report = STABILITY.apply_segment_stability(
            baseline,
            raw_positions,
            device_epochs,
            max_consecutive_rejects=1,
            max_prediction_duration_s=1.0,
        )
        self.assertEqual(report["population"]["raw_interpolated_epochs"], 1)
        self.assertEqual(result.rows[1].source, "raw_interpolated")
        self.assertFalse(result.rows[1].measurement_used)
        np.testing.assert_allclose(result.rows[1].ecef, middle.ecef, atol=1e-9)


if __name__ == "__main__":
    unittest.main()
