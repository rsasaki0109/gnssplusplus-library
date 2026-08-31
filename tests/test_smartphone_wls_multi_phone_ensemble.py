from __future__ import annotations

import json
from pathlib import Path
import sys
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands"))
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_wls_multi_phone_ensemble_eval as ENSEMBLE  # noqa: E402
import gnss_smartphone_trajectory_smoother as SMOOTHER  # noqa: E402


class SmartphoneWlsMultiPhoneEnsembleTests(unittest.TestCase):
    def test_selection_record_is_payload_free_and_methods_are_fixed(self) -> None:
        path = (
            ROOT
            / "docs"
            / "use_cases"
            / "records"
            / "smartphone_r5_wls_multi_phone_ensemble_selection.json"
        )
        selection = json.loads(path.read_text(encoding="utf-8"))
        self.assertEqual(
            selection["status"], "selection-frozen-before-member-content-read"
        )
        self.assertFalse(selection["archive"]["member_content_read"])
        self.assertFalse(selection["archive"]["truth_opened"])
        self.assertEqual(
            tuple(method["id"] for method in selection["candidate_method_contract"]["methods"]),
            ENSEMBLE.METHOD_IDS,
        )
        self.assertTrue(selection["next_holdout"]["materialization_forbidden"])
        self.assertTrue(selection["next_holdout"]["truth_open_forbidden"])

    def test_profile_exposes_development_only_lane_without_changing_production(self) -> None:
        profile = json.loads(
            (ROOT / "configs" / "benchmarks" / "smartphone_r5_gsdc2023.json").read_text(
                encoding="utf-8"
            )
        )
        lane = profile["development_only_submission_lane"]["multi_phone_ensemble"]
        self.assertEqual(lane["selected_method"], "coordinate_wise_ecef_median")
        self.assertTrue(lane["truth_free_runtime"])
        self.assertFalse(lane["production_default"])
        self.assertEqual(lane["next_holdout"], "2022-10-06-21-51-us-ca-mtv-n")

    def test_fixed_fusion_methods_are_finite(self) -> None:
        values = [
            np.array([10.0, 20.0, 30.0]),
            np.array([11.0, 19.0, 31.0]),
            np.array([10.5, 20.5, 29.5]),
        ]
        median = ENSEMBLE._fuse(values, "coordinate_wise_ecef_median")
        geometric = ENSEMBLE._fuse(values, "geometric_median_ecef")
        trimmed = ENSEMBLE._fuse(values, "trimmed_mean_ecef")
        self.assertTrue(np.isfinite(median).all())
        self.assertTrue(np.isfinite(geometric).all())
        self.assertTrue(np.isfinite(trimmed).all())
        np.testing.assert_allclose(median, [10.5, 20.0, 30.0])
        np.testing.assert_allclose(trimmed, np.mean(values, axis=0))

    def test_alignment_reports_missing_phone_and_spread(self) -> None:
        epoch_ms = 1609459200000  # 2021-01-01T00:00:00Z, representable GPS epoch

        def row(timestamp: int, x: float) -> SMOOTHER.PositionRow:
            return SMOOTHER.PositionRow(
                week=2200,
                tow=float(timestamp) / 1000.0,
                timestamp_ms=timestamp,
                ecef=np.array([x, 0.0, 6370000.0]),
                latitude=0.0,
                longitude=0.0,
                height=0.0,
                status=1,
                satellites=4,
                pdop=0.0,
                ratio=0.0,
                fixed_ambiguities=0,
                iterations=0,
                source_line=1,
            )

        positions, stats = ENSEMBLE._fused_positions(
            {
                "a": [row(epoch_ms, 1.0), row(epoch_ms + 1000, 2.0)],
                "b": [row(epoch_ms + 1, 3.0)],
            },
            "coordinate_wise_ecef_median",
        )
        self.assertEqual(len(positions["a"]), 2)
        self.assertEqual(len(positions["b"]), 1)
        self.assertEqual(stats["alignment_tolerance_ms"], 1)
        self.assertEqual(stats["phone_count_histogram"]["2"], 2)
        self.assertEqual(stats["phone_count_histogram"]["1"], 1)
        self.assertEqual(stats["missing_phone_observations"], 1)
        self.assertEqual(stats["max_abs_alignment_offset_ms"], 1.0)
        self.assertGreater(stats["spread_ecef_m"]["max_m"], 0.0)

    def test_alignment_tolerance_v1_4_boundary_and_deterministic_tie(self) -> None:
        self.assertEqual(ENSEMBLE.ALIGNMENT_TOLERANCE_MS, 1)
        self.assertEqual(ENSEMBLE.V1_4_ALIGNMENT_TOLERANCE_MS, 10)
        epoch_ms = 1609459200000

        def row(timestamp: int, x: float) -> SMOOTHER.PositionRow:
            return SMOOTHER.PositionRow(
                week=2200,
                tow=float(timestamp) / 1000.0,
                timestamp_ms=timestamp,
                ecef=np.array([x, 0.0, 6370000.0]),
                latitude=0.0,
                longitude=0.0,
                height=0.0,
                status=1,
                satellites=4,
                pdop=0.0,
                ratio=0.0,
                fixed_ambiguities=0,
                iterations=0,
                source_line=1,
            )

        positions = [row(epoch_ms - 10, 1.0), row(epoch_ms + 10, 2.0)]
        nearest = ENSEMBLE._nearest_position(
            positions,
            [value.timestamp_ms for value in positions],
            epoch_ms,
            ENSEMBLE.V1_4_ALIGNMENT_TOLERANCE_MS,
        )
        self.assertIsNotNone(nearest)
        self.assertEqual(nearest[0].timestamp_ms, epoch_ms - 10)
        self.assertEqual(nearest[1], 10)
        self.assertIsNone(
            ENSEMBLE._nearest_position(
                [row(epoch_ms + 11, 3.0)],
                [epoch_ms + 11],
                epoch_ms,
                ENSEMBLE.V1_4_ALIGNMENT_TOLERANCE_MS,
            )
        )

    def test_sparse_alignment_recovers_target_missing_from_peer_and_records_source(self) -> None:
        epoch_ms = 1609459200000

        def row(timestamp: int, x: float) -> SMOOTHER.PositionRow:
            return SMOOTHER.PositionRow(
                week=2200,
                tow=float(timestamp) / 1000.0,
                timestamp_ms=timestamp,
                ecef=np.array([x, 0.0, 6370000.0]),
                latitude=0.0,
                longitude=0.0,
                height=0.0,
                status=1,
                satellites=4,
                pdop=0.0,
                ratio=0.0,
                fixed_ambiguities=0,
                iterations=0,
                source_line=1,
            )

        positions, stats = ENSEMBLE._fused_positions(
            {
                "a": [row(epoch_ms + 1000, 1.0)],
                "b": [row(epoch_ms, 3.0), row(epoch_ms + 1000, 3.0)],
            },
            "coordinate_wise_ecef_median",
            timestamp_keys_by_phone={
                "a": [epoch_ms, epoch_ms + 1000],
                "b": [epoch_ms, epoch_ms + 1000],
            },
        )
        self.assertEqual(len(positions["a"]), 2)
        self.assertEqual(len(positions["b"]), 2)
        sparse_rows = [
            entry
            for entry in stats["epoch_manifest"]
            if entry["target_phone"] == "a" and entry["timestamp_ms"] == epoch_ms
        ]
        self.assertEqual(len(sparse_rows), 1)
        self.assertEqual(sparse_rows[0]["source"], "peer_wls_sparse_recovery")
        self.assertFalse(sparse_rows[0]["target_wls_available"])
        self.assertEqual(sparse_rows[0]["used_phones"], ["b"])
        self.assertTrue(sparse_rows[0]["fallback"])

    def test_sparse_alignment_fails_closed_when_all_phones_are_missing(self) -> None:
        with self.assertRaises(ENSEMBLE.EnsembleError):
            ENSEMBLE._fused_positions(
                {"a": [], "b": []},
                "coordinate_wise_ecef_median",
                timestamp_keys_by_phone={"a": [1609459200000], "b": [1609459200000]},
            )

    def test_sparse_alignment_accepts_one_phone_gap_when_peer_covers_timestamp(self) -> None:
        epoch_ms = 1609459200000

        def row(timestamp: int, x: float) -> SMOOTHER.PositionRow:
            return SMOOTHER.PositionRow(
                week=2200,
                tow=float(timestamp) / 1000.0,
                timestamp_ms=timestamp,
                ecef=np.array([x, 0.0, 6370000.0]),
                latitude=0.0,
                longitude=0.0,
                height=0.0,
                status=1,
                satellites=4,
                pdop=0.0,
                ratio=0.0,
                fixed_ambiguities=0,
                iterations=0,
                source_line=1,
            )

        middle = epoch_ms + 2_001
        positions, stats = ENSEMBLE._fused_positions(
            {
                "a": [row(epoch_ms, 1.0), row(epoch_ms + 4_002, 1.0)],
                "b": [row(epoch_ms, 3.0), row(middle, 3.0), row(epoch_ms + 4_002, 3.0)],
            },
            "coordinate_wise_ecef_median",
            timestamp_keys_by_phone={
                "a": [epoch_ms, middle, epoch_ms + 4_002],
                "b": [epoch_ms, middle, epoch_ms + 4_002],
            },
        )
        self.assertEqual(len(positions["a"]), 3)
        self.assertEqual(len(positions["b"]), 3)
        recovered = [
            entry
            for entry in stats["epoch_manifest"]
            if entry["target_phone"] == "a" and entry["timestamp_ms"] == middle
        ]
        self.assertEqual(recovered[0]["source"], "peer_wls_sparse_recovery")
        self.assertFalse(recovered[0]["target_wls_available"])
        self.assertEqual(stats["covered_target_epoch_count"], stats["target_epoch_count"])
        self.assertEqual(stats["unresolved_target_epoch_count"], 0)
        self.assertTrue(stats["all_target_timestamps_covered_by_source"])
        self.assertFalse(stats["extrapolation_used"])

    def test_sparse_alignment_fails_when_all_phones_share_the_gap(self) -> None:
        epoch_ms = 1609459200000
        middle = epoch_ms + 2_001

        def row(timestamp: int, x: float) -> SMOOTHER.PositionRow:
            return SMOOTHER.PositionRow(
                week=2200,
                tow=float(timestamp) / 1000.0,
                timestamp_ms=timestamp,
                ecef=np.array([x, 0.0, 6370000.0]),
                latitude=0.0,
                longitude=0.0,
                height=0.0,
                status=1,
                satellites=4,
                pdop=0.0,
                ratio=0.0,
                fixed_ambiguities=0,
                iterations=0,
                source_line=1,
            )

        with self.assertRaises(ENSEMBLE.EnsembleError):
            ENSEMBLE._fused_positions(
                {
                    "a": [row(epoch_ms, 1.0), row(epoch_ms + 4_002, 1.0)],
                    "b": [row(epoch_ms, 3.0), row(epoch_ms + 4_002, 3.0)],
                },
                "coordinate_wise_ecef_median",
                timestamp_keys_by_phone={
                    "a": [epoch_ms, middle, epoch_ms + 4_002],
                    "b": [epoch_ms, middle, epoch_ms + 4_002],
                },
            )

    def test_sparse_alignment_fails_when_peer_is_outside_tolerance(self) -> None:
        epoch_ms = 1609459200000

        def row(timestamp: int) -> SMOOTHER.PositionRow:
            return SMOOTHER.PositionRow(
                week=2200,
                tow=float(timestamp) / 1000.0,
                timestamp_ms=timestamp,
                ecef=np.array([1.0, 0.0, 6370000.0]),
                latitude=0.0,
                longitude=0.0,
                height=0.0,
                status=1,
                satellites=4,
                pdop=0.0,
                ratio=0.0,
                fixed_ambiguities=0,
                iterations=0,
                source_line=1,
            )

        with self.assertRaises(ENSEMBLE.EnsembleError):
            ENSEMBLE._fused_positions(
                {"a": [], "b": [row(epoch_ms + 2)]},
                "coordinate_wise_ecef_median",
                timestamp_keys_by_phone={"a": [epoch_ms], "b": [epoch_ms]},
            )

    def test_train_truth_vehicle_audit_rejects_large_divergence(self) -> None:
        first = {1000: (37.0, -122.0, 10.0), 2000: (37.0, -122.0, 10.0)}
        second = {1000: (37.0, -122.0, 10.0), 2000: (37.0, -122.0, 10.0)}
        thresholds = {
            "minimum_pairwise_coverage_ratio": 0.95,
            "maximum_pairwise_horizontal_p95_m": 50.0,
            "maximum_pairwise_vertical_p95_m": 25.0,
        }
        self.assertTrue(
            ENSEMBLE._truth_vehicle_audit({"a": first, "b": second}, thresholds)["passed"]
        )
        shifted = {1433: (37.0, -122.0, 10.0), 2433: (37.0, -122.0, 10.0)}
        shifted_audit = ENSEMBLE._truth_vehicle_audit({"a": first, "b": shifted}, thresholds)
        self.assertTrue(shifted_audit["passed"])
        self.assertEqual(shifted_audit["pairs"][0]["estimated_constant_time_offset_ms"], 433)
        second[2000] = (37.01, -122.0, 10.0)
        with self.assertRaises(ENSEMBLE.EnsembleError):
            ENSEMBLE._truth_vehicle_audit({"a": first, "b": second}, thresholds)


if __name__ == "__main__":
    unittest.main()
