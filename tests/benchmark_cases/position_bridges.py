"""Position and FGO bridge benchmark test cases."""

from ._support import *  # noqa: F401,F403

__all__ = [
    "FixedAnchorBridgeTest",
    "FloatHorizontalSmootherTest",
    "VelocityFixedAnchorBridgeTest",
    "CandidateQualityPositionSelectorTest",
    "FgoPositionConsensusTest",
    "FgoShadowAuthorityTest",
    "KfFgoAlignmentTest",
]


class FixedAnchorBridgeTest(unittest.TestCase):
    @staticmethod
    def epoch(
        tow: float,
        x_m: float,
        status: int,
        ratio: float = 0.0,
    ) -> comparison.SolutionEpoch:
        return comparison.SolutionEpoch(
            week=2300,
            tow=tow,
            lat_deg=0.0,
            lon_deg=0.0,
            height_m=0.0,
            ecef=np.asarray([x_m, 0.0, 0.0]),
            status=status,
            num_satellites=12,
            ratio=ratio,
            rtk_update_post_suppression_residual_rms_m=0.2,
            rtk_update_normalized_innovation_squared_per_observation=0.5,
        )

    def test_replaces_only_nonfixed_epochs_between_trusted_fixed_anchors(self) -> None:
        epochs = [
            self.epoch(0.0, 0.0, 4, 10.0),
            self.epoch(1.0, 50.0, 3),
            self.epoch(2.0, 20.0, 4, 10.0),
        ]
        args = argparse.Namespace(
            nominal_interval_s=1.0,
            max_anchor_gap_s=3.0,
            anchor_min_ratio=2.0,
            anchor_min_satellites=0,
            anchor_max_post_rms_m=0.8,
            anchor_max_nis_per_observation=5.0,
            replace_nonfixed=True,
            interpolation="linear",
        )

        bridged, summary = fixed_anchor_bridge.bridge_epochs(epochs, args)

        self.assertEqual([epoch.status for epoch in bridged], [4, 3, 4])
        np.testing.assert_allclose(bridged[1].ecef, [10.0, 0.0, 0.0])
        np.testing.assert_allclose(bridged[0].ecef, epochs[0].ecef)
        np.testing.assert_allclose(bridged[2].ecef, epochs[2].ecef)
        self.assertEqual(summary["replaced_nonfixed_epochs"], 1)
        self.assertEqual(summary["filled_missing_epochs"], 0)
        self.assertFalse(summary["reference_truth_used"])

    def test_gap_fill_does_not_replace_existing_nonfixed_without_opt_in(self) -> None:
        epochs = [
            self.epoch(0.0, 0.0, 4, 10.0),
            self.epoch(1.0, 50.0, 3),
            self.epoch(3.0, 30.0, 4, 10.0),
        ]
        args = argparse.Namespace(
            nominal_interval_s=1.0,
            max_anchor_gap_s=3.0,
            anchor_min_ratio=2.0,
            anchor_min_satellites=0,
            anchor_max_post_rms_m=0.8,
            anchor_max_nis_per_observation=5.0,
            replace_nonfixed=False,
            interpolation="linear",
        )

        bridged, summary = fixed_anchor_bridge.bridge_epochs(epochs, args)

        self.assertEqual([epoch.tow for epoch in bridged], [0.0, 1.0, 2.0, 3.0])
        np.testing.assert_allclose(bridged[1].ecef, epochs[1].ecef)
        np.testing.assert_allclose(bridged[2].ecef, [20.0, 0.0, 0.0])
        self.assertEqual(summary["replaced_nonfixed_epochs"], 0)
        self.assertEqual(summary["filled_missing_epochs"], 1)

    def test_fill_between_all_positions_bridges_short_float_dropout(self) -> None:
        epochs = [
            self.epoch(0.0, 0.0, 3),
            self.epoch(2.0, 20.0, 3),
        ]
        args = argparse.Namespace(
            nominal_interval_s=1.0,
            max_anchor_gap_s=2.0,
            anchor_min_ratio=2.0,
            anchor_min_satellites=0,
            anchor_max_post_rms_m=0.8,
            anchor_max_nis_per_observation=5.0,
            replace_nonfixed=False,
            fill_between_all_positions=True,
            interpolation="linear",
        )

        bridged, summary = fixed_anchor_bridge.bridge_epochs(epochs, args)

        self.assertEqual([epoch.tow for epoch in bridged], [0.0, 1.0, 2.0])
        np.testing.assert_allclose(bridged[1].ecef, [10.0, 0.0, 0.0])
        self.assertEqual(bridged[1].status, 3)
        self.assertEqual(summary["filled_missing_epochs"], 1)
        self.assertTrue(summary["fill_between_all_positions"])

    def test_no_fill_missing_preserves_epoch_grid_while_replacing_float(self) -> None:
        epochs = [
            self.epoch(0.0, 0.0, 4, 10.0),
            self.epoch(1.0, 50.0, 3),
            self.epoch(3.0, 30.0, 4, 10.0),
        ]
        args = argparse.Namespace(
            nominal_interval_s=1.0,
            max_anchor_gap_s=3.0,
            anchor_min_ratio=2.0,
            anchor_min_satellites=0,
            anchor_max_post_rms_m=0.8,
            anchor_max_nis_per_observation=5.0,
            replace_nonfixed=True,
            no_fill_missing=True,
            interpolation="linear",
        )

        bridged, summary = fixed_anchor_bridge.bridge_epochs(epochs, args)

        self.assertEqual([epoch.tow for epoch in bridged], [0.0, 1.0, 3.0])
        np.testing.assert_allclose(bridged[1].ecef, [10.0, 0.0, 0.0])
        self.assertEqual(summary["replaced_nonfixed_epochs"], 1)
        self.assertEqual(summary["filled_missing_epochs"], 0)
        self.assertFalse(summary["fill_missing"])

    def test_preserve_up_component_removes_local_vertical_correction(self) -> None:
        existing = self.epoch(1.0, 6378137.0, 3)
        candidate = self.epoch(1.0, 6378147.0, 3)

        horizontal = fixed_anchor_bridge.preserve_up_component(existing, candidate)

        np.testing.assert_allclose(horizontal.ecef, existing.ecef, atol=1e-6)

    def test_preserve_horizontal_component_applies_only_local_up_correction(self) -> None:
        existing = dataclasses.replace(
            self.epoch(1.0, 6378137.0, 3),
            ecef=np.asarray([6378137.0, 5.0, 0.0]),
        )
        candidate = dataclasses.replace(
            self.epoch(1.0, 6378147.0, 3),
            ecef=np.asarray([6378147.0, 10.0, 0.0]),
        )

        vertical = fixed_anchor_bridge.preserve_horizontal_component(existing, candidate)

        np.testing.assert_allclose(vertical.ecef, [6378147.0, 5.0, 0.0], atol=1e-5)

    def test_replacement_displacement_gate_preserves_far_float(self) -> None:
        epochs = [
            self.epoch(0.0, 0.0, 4, 10.0),
            self.epoch(1.0, 50.0, 3),
            self.epoch(2.0, 20.0, 4, 10.0),
        ]
        args = argparse.Namespace(
            nominal_interval_s=1.0,
            max_anchor_gap_s=3.0,
            anchor_min_ratio=2.0,
            anchor_min_satellites=0,
            anchor_max_post_rms_m=0.8,
            anchor_max_nis_per_observation=5.0,
            replace_nonfixed=True,
            no_fill_missing=True,
            preserve_existing_up=False,
            max_replacement_displacement_m=5.0,
            interpolation="linear",
        )

        bridged, summary = fixed_anchor_bridge.bridge_epochs(epochs, args)

        np.testing.assert_allclose(bridged[1].ecef, epochs[1].ecef)
        self.assertEqual(summary["replaced_nonfixed_epochs"], 0)
        self.assertEqual(summary["displacement_rejected_epochs"], 1)

    def test_hermite_outer_uses_neighboring_fixed_anchor_velocities(self) -> None:
        epochs = [
            self.epoch(-1.0, -10.0, 4, 10.0),
            self.epoch(0.0, 0.0, 4, 10.0),
            self.epoch(1.0, 50.0, 3),
            self.epoch(2.0, 20.0, 4, 10.0),
            self.epoch(3.0, 40.0, 4, 10.0),
        ]
        args = argparse.Namespace(
            nominal_interval_s=1.0,
            max_anchor_gap_s=3.0,
            anchor_min_ratio=2.0,
            anchor_min_satellites=0,
            anchor_max_post_rms_m=0.8,
            anchor_max_nis_per_observation=5.0,
            replace_nonfixed=True,
            interpolation="hermite-outer",
        )

        bridged, summary = fixed_anchor_bridge.bridge_epochs(epochs, args)
        midpoint = next(epoch for epoch in bridged if epoch.tow == 1.0)

        np.testing.assert_allclose(midpoint.ecef, [7.5, 0.0, 0.0])
        self.assertEqual(midpoint.status, 3)
        self.assertEqual(summary["interpolation"], "hermite-outer")

    def test_hermite_horizontal_removes_up_component_from_hermite_delta(self) -> None:
        left = self.epoch(0.0, 6378137.0, 4, 10.0)
        right = self.epoch(2.0, 6378157.0, 4, 10.0)

        full = fixed_anchor_bridge.interpolated_epoch(
            left,
            right,
            1.0,
            np.asarray([0.0, 0.0, 0.0]),
            np.asarray([20.0, 0.0, 0.0]),
        )
        horizontal = fixed_anchor_bridge.interpolated_epoch(
            left,
            right,
            1.0,
            np.asarray([0.0, 0.0, 0.0]),
            np.asarray([20.0, 0.0, 0.0]),
            hermite_horizontal_only=True,
        )

        self.assertNotAlmostEqual(float(full.ecef[0]), 6378147.0)
        self.assertAlmostEqual(float(horizontal.ecef[0]), 6378147.0, places=6)


class FloatHorizontalSmootherTest(unittest.TestCase):
    def test_preserves_fixed_status_and_up_while_smoothing_float_outlier(self) -> None:
        origin = comparison.llh_to_ecef(35.0, 139.0, 50.0)
        seed = comparison.SolutionEpoch(
            week=2300,
            tow=0.0,
            lat_deg=35.0,
            lon_deg=139.0,
            height_m=50.0,
            ecef=origin,
            status=4,
            num_satellites=12,
            ratio=10.0,
        )
        rotation = float_horizontal_smoother.ecef_to_enu_rotation(seed)
        epochs = []
        for index in range(7):
            enu = np.asarray([float(index), 4.0 if index == 3 else 0.0, 0.0])
            ecef = origin + rotation.transpose() @ enu
            lat, lon, height = ppc_metrics.llh_from_ecef(*ecef)
            epochs.append(
                comparison.SolutionEpoch(
                    week=2300,
                    tow=float(index),
                    lat_deg=lat,
                    lon_deg=lon,
                    height_m=height,
                    ecef=ecef,
                    status=4 if index in (0, 6) else 3,
                    num_satellites=12,
                    ratio=10.0 if index in (0, 6) else 1.0,
                )
            )

        smoothed, summary = float_horizontal_smoother.smooth_float_horizontal(
            epochs, window_epochs=5, degree=2, max_displacement_m=5.0
        )

        np.testing.assert_allclose(smoothed[0].ecef, epochs[0].ecef)
        np.testing.assert_allclose(smoothed[-1].ecef, epochs[-1].ecef)
        self.assertEqual([epoch.status for epoch in smoothed], [4, 3, 3, 3, 3, 3, 4])
        correction_enu = rotation @ (smoothed[3].ecef - epochs[3].ecef)
        self.assertLess(abs(float(correction_enu[2])), 1e-8)
        self.assertGreater(abs(float(correction_enu[1])), 1.0)
        self.assertEqual(summary["preserved_fixed_epochs"], 2)
        self.assertFalse(summary["reference_truth_used"])


class VelocityFixedAnchorBridgeTest(unittest.TestCase):
    def test_overlay_applies_only_candidate_horizontal_delta(self) -> None:
        origin_ecef = comparison.llh_to_ecef(35.0, 139.0, 50.0)
        base = comparison.SolutionEpoch(
            2300, 0.0, 35.0, 139.0, 50.0, origin_ecef, 4, 12, ratio=10.0
        )
        rotation = velocity_fixed_anchor_bridge.ecef_to_enu_rotation(base)

        def moved(enu: list[float], status: int) -> comparison.SolutionEpoch:
            ecef = origin_ecef + rotation.transpose() @ np.asarray(enu)
            lat, lon, height = ppc_metrics.llh_from_ecef(*ecef)
            return dataclasses.replace(
                base,
                ecef=ecef,
                lat_deg=lat,
                lon_deg=lon,
                height_m=height,
                status=status,
            )

        combined, replaced = velocity_fixed_anchor_bridge.overlay_changed_horizontal(
            [base], [moved([1.0, 0.0, 0.0], 3)], [moved([0.0, 0.0, 2.0], 3)]
        )

        combined_enu = rotation @ (combined[0].ecef - origin_ecef)
        np.testing.assert_allclose(combined_enu, [1.0, 0.0, 2.0], atol=1e-5)
        self.assertEqual(combined[0].status, 3)
        self.assertEqual(replaced, 1)

    def test_velocity_bridge_can_require_large_candidate_correction(self) -> None:
        origin_ecef = comparison.llh_to_ecef(35.0, 139.0, 50.0)
        seed = comparison.SolutionEpoch(
            2300, 0.0, 35.0, 139.0, 50.0, origin_ecef, 4, 12, ratio=10.0
        )
        rotation = velocity_fixed_anchor_bridge.ecef_to_enu_rotation(seed)
        epochs = []
        for index, north_m in enumerate((0.0, 2.0, 0.0)):
            ecef = origin_ecef + rotation.transpose() @ np.asarray(
                [float(index), north_m, 0.0]
            )
            lat, lon, height = ppc_metrics.llh_from_ecef(*ecef)
            epochs.append(
                comparison.SolutionEpoch(
                    2300,
                    float(index),
                    lat,
                    lon,
                    height,
                    ecef,
                    4 if index in (0, 2) else 3,
                    12,
                    ratio=10.0 if index in (0, 2) else 0.0,
                )
            )

        bridged, summary = velocity_fixed_anchor_bridge.bridge_velocity_spans(
            epochs,
            np.arange(3, dtype=float),
            np.tile(np.asarray([1.0, 0.0]), (3, 1)),
            blend=1.0,
            max_span_s=3.0,
            max_candidate_correction_m=3.0,
            max_velocity_sample_gap_s=1.1,
            min_candidate_correction_m=2.5,
        )

        np.testing.assert_allclose(bridged[1].ecef, epochs[1].ecef)
        self.assertEqual(summary["accepted_spans"], 0)
        self.assertEqual(summary["minimum_correction_rejected_spans"], 1)

    def test_velocity_gap_fill_emits_float_regular_grid_without_truth(self) -> None:
        origin_ecef = comparison.llh_to_ecef(35.0, 139.0, 50.0)
        seed = comparison.SolutionEpoch(
            2300, 0.0, 35.0, 139.0, 50.0, origin_ecef, 4, 12, ratio=10.0
        )
        rotation = velocity_fixed_anchor_bridge.ecef_to_enu_rotation(seed)
        end_ecef = origin_ecef + rotation.transpose() @ np.asarray([4.0, 0.0, 0.0])
        end_lat, end_lon, end_height = ppc_metrics.llh_from_ecef(*end_ecef)
        epochs = [
            seed,
            comparison.SolutionEpoch(
                2300,
                4.0,
                end_lat,
                end_lon,
                end_height,
                end_ecef,
                4,
                10,
                ratio=8.0,
            ),
        ]

        filled, summary = velocity_fixed_anchor_bridge.fill_missing_velocity_epochs(
            epochs,
            np.arange(5, dtype=float),
            np.tile(np.asarray([1.0, 0.0]), (5, 1)),
            epoch_interval_s=1.0,
            max_fill_span_s=5.0,
            max_candidate_correction_m=1.0,
            max_velocity_sample_gap_s=1.1,
        )

        self.assertEqual([epoch.tow for epoch in filled], [0.0, 1.0, 2.0, 3.0, 4.0])
        self.assertEqual([epoch.status for epoch in filled], [4, 3, 3, 3, 4])
        self.assertEqual(summary["filled_missing_epochs"], 3)
        self.assertEqual(summary["accepted_fill_spans"], 1)
        for index in (1, 2, 3):
            enu = rotation @ (filled[index].ecef - origin_ecef)
            np.testing.assert_allclose(enu, [float(index), 0.0, 0.0], atol=1e-6)
            self.assertEqual(filled[index].ratio, 0.0)
            self.assertIsNone(filled[index].rtk_iterations)

    def test_velocity_bridge_preserves_fixed_up_and_status(self) -> None:
        origin_ecef = comparison.llh_to_ecef(35.0, 139.0, 50.0)
        seed = comparison.SolutionEpoch(
            2300, 0.0, 35.0, 139.0, 50.0, origin_ecef, 4, 12, ratio=10.0
        )
        rotation = velocity_fixed_anchor_bridge.ecef_to_enu_rotation(seed)
        epochs = []
        for index, north_m in enumerate((0.0, 2.0, 3.0, 2.0, 0.0)):
            ecef = origin_ecef + rotation.transpose() @ np.asarray(
                [float(index), north_m, 0.0]
            )
            lat, lon, height = ppc_metrics.llh_from_ecef(*ecef)
            epochs.append(
                comparison.SolutionEpoch(
                    2300,
                    float(index),
                    lat,
                    lon,
                    height,
                    ecef,
                    4 if index in (0, 4) else 3,
                    12,
                    ratio=10.0 if index in (0, 4) else 0.0,
                )
            )

        bridged, summary = velocity_fixed_anchor_bridge.bridge_velocity_spans(
            epochs,
            np.arange(5, dtype=float),
            np.tile(np.asarray([1.0, 0.0]), (5, 1)),
            blend=1.0,
            max_span_s=5.0,
            max_candidate_correction_m=4.0,
            max_velocity_sample_gap_s=1.1,
        )

        self.assertEqual([epoch.status for epoch in bridged], [4, 3, 3, 3, 4])
        np.testing.assert_allclose(bridged[0].ecef, epochs[0].ecef)
        np.testing.assert_allclose(bridged[-1].ecef, epochs[-1].ecef)
        for index in (1, 2, 3):
            lat = np.radians(epochs[index].lat_deg)
            lon = np.radians(epochs[index].lon_deg)
            local_up = np.asarray(
                [np.cos(lat) * np.cos(lon), np.cos(lat) * np.sin(lon), np.sin(lat)]
            )
            correction_ecef = bridged[index].ecef - epochs[index].ecef
            self.assertAlmostEqual(float(np.dot(correction_ecef, local_up)), 0.0, places=7)
            corrected_enu = rotation @ (bridged[index].ecef - origin_ecef)
            self.assertAlmostEqual(float(corrected_enu[1]), 0.0, places=6)
        self.assertEqual(summary["accepted_spans"], 1)
        self.assertEqual(summary["replaced_float_epochs"], 3)
        self.assertFalse(summary["reference_truth_used"])


class CandidateQualityPositionSelectorTest(unittest.TestCase):
    def test_can_gate_on_baseline_wrong_basin_telemetry_and_replace_status(self) -> None:
        origin = comparison.llh_to_ecef(35.0, 139.0, 50.0)
        candidate_ecef = origin + np.asarray([1.0, 0.0, 0.0])
        lat, lon, height = ppc_metrics.llh_from_ecef(*candidate_ecef)
        baseline = [
            comparison.SolutionEpoch(
                2300,
                0.0,
                35.0,
                139.0,
                50.0,
                origin,
                4,
                20,
                ratio=30.0,
                rtk_update_suppressed_outliers=45,
                rtk_update_prefit_residual_rms_m=8.1,
            )
        ]
        candidate = [
            comparison.SolutionEpoch(
                2300, 0.0, lat, lon, height, candidate_ecef, 3, 0, ratio=0.0
            )
        ]
        args = argparse.Namespace(
            match_tolerance_s=0.11,
            candidate_status=3,
            candidate_min_ratio=0.0,
            candidate_min_satellites=0,
            candidate_max_post_rms_m=0.0,
            candidate_max_nis_per_observation=0.0,
            baseline_min_prefit_rms_m=8.0,
            baseline_min_outliers=45,
            min_position_separation_m=0.5,
            max_position_separation_m=0.0,
            replace_status=True,
        )

        selected, summary = candidate_quality_selector.select_candidate_positions(
            baseline, candidate, args
        )

        np.testing.assert_allclose(selected[0].ecef, candidate_ecef)
        self.assertEqual(selected[0].status, 3)

    def test_can_select_low_confidence_consensus_escape_and_force_float(self) -> None:
        origin = comparison.llh_to_ecef(35.0, 139.0, 50.0)
        candidate_ecef = origin + np.asarray([6.0, 0.0, 0.0])
        lat, lon, height = ppc_metrics.llh_from_ecef(*candidate_ecef)
        baseline = comparison.SolutionEpoch(
            2300, 100.0, 35.0, 139.0, 50.0, origin, 4, 19, ratio=6.0
        )
        candidate = comparison.SolutionEpoch(
            2300, 100.0, lat, lon, height, candidate_ecef, 4, 19, ratio=2.0
        )
        args = argparse.Namespace(
            match_tolerance_s=0.11,
            candidate_status=4,
            candidate_min_ratio=2.0,
            candidate_max_ratio=2.0,
            candidate_min_satellites=12,
            candidate_max_post_rms_m=0.0,
            candidate_max_nis_per_observation=0.0,
            baseline_min_prefit_rms_m=0.0,
            baseline_min_outliers=0,
            baseline_min_satellites=19,
            replace_status=False,
            replacement_status=3,
            min_position_separation_m=5.0,
            max_position_separation_m=0.0,
        )

        selected, summary = candidate_quality_selector.select_candidate_positions(
            [baseline], [candidate], args
        )

        self.assertEqual(summary["selected_candidate_positions"], 1)
        self.assertEqual(summary["candidate_max_ratio"], 2.0)
        self.assertEqual(summary["baseline_min_satellites"], 19)
        self.assertEqual(summary["replacement_status"], 3)
        self.assertEqual(selected[0].status, 3)
        np.testing.assert_allclose(selected[0].ecef, candidate.ecef)
        self.assertFalse(summary["preserved_baseline_status"])
        self.assertTrue(summary["preserved_baseline_telemetry"])

    def test_selects_position_only_when_all_quality_gates_pass(self) -> None:
        origin = comparison.llh_to_ecef(35.0, 139.0, 50.0)
        seed = comparison.SolutionEpoch(2300, 0.0, 35.0, 139.0, 50.0, origin, 3, 10)
        rotation = velocity_fixed_anchor_bridge.ecef_to_enu_rotation(seed)

        def epoch(
            index: int,
            east_m: float,
            status: int,
            ratio: float,
        ) -> comparison.SolutionEpoch:
            ecef = origin + rotation.transpose() @ np.asarray([east_m, 0.0, 0.0])
            lat, lon, height = ppc_metrics.llh_from_ecef(*ecef)
            return comparison.SolutionEpoch(
                week=2300,
                tow=float(index),
                lat_deg=lat,
                lon_deg=lon,
                height_m=height,
                ecef=ecef,
                status=status,
                num_satellites=12,
                ratio=ratio,
                rtk_update_post_suppression_residual_rms_m=0.5,
                rtk_update_normalized_innovation_squared_per_observation=1.0,
            )

        baseline = [epoch(0, 0.0, 3, 0.0), epoch(1, 1.0, 4, 10.0), epoch(2, 2.0, 3, 0.0)]
        candidate = [epoch(0, 1.0, 4, 3.0), epoch(1, 2.0, 4, 1.0), epoch(2, 2.1, 4, 3.0)]
        args = argparse.Namespace(
            match_tolerance_s=0.11,
            candidate_status=4,
            candidate_min_ratio=2.0,
            candidate_min_satellites=12,
            candidate_max_post_rms_m=1.0,
            candidate_max_nis_per_observation=5.0,
            min_position_separation_m=0.5,
            max_position_separation_m=0.0,
        )

        selected, summary = candidate_quality_selector.select_candidate_positions(
            baseline, candidate, args
        )

        np.testing.assert_allclose(selected[0].ecef, candidate[0].ecef)
        np.testing.assert_allclose(selected[1].ecef, baseline[1].ecef)
        np.testing.assert_allclose(selected[2].ecef, baseline[2].ecef)
        self.assertEqual([epoch.status for epoch in selected], [3, 4, 3])
        self.assertEqual(summary["selected_candidate_positions"], 1)
        self.assertFalse(summary["reference_truth_used"])


class FgoPositionConsensusTest(unittest.TestCase):
    def test_two_fixed_shadows_replace_position_without_changing_status(self) -> None:
        origin = comparison.llh_to_ecef(35.0, 139.0, 50.0)
        primary_ecef = origin + np.asarray([1.0, 0.0, 0.0])
        lat, lon, height = ppc_metrics.llh_from_ecef(*primary_ecef)
        primary = [
            comparison.SolutionEpoch(
                week=2300,
                tow=1.0,
                lat_deg=lat,
                lon_deg=lon,
                height_m=height,
                ecef=primary_ecef,
                status=4,
                num_satellites=12,
                ratio=25.0,
            )
        ]
        tracks = [
            {
                1.0: fgo_position_consensus.TrackEpoch(
                    sample=ppc_integrity_consensus.ShadowEpoch(
                        tow=1.0,
                        ecef=origin + np.asarray([offset, 0.0, 0.0]),
                        status="FIXED",
                        gdop=2.0,
                        ddpr_rms_m=1.0,
                        nsat=10,
                    ),
                    age_epochs=10,
                )
            }
            for offset in (0.0, 0.1)
        ]
        args = argparse.Namespace(
            min_independent_shadows=2,
            shadow_agreement_aperture_m=0.25,
            primary_separation_min_m=0.5,
            fresh_shadow_max_age_epochs=100,
            candidate_max_prediction_error_m=0.0,
            shadow_max_gdop=0.0,
            shadow_max_ddpr_rms_m=0.0,
            shadow_min_satellites=0,
        )

        output, summary, ledger = fgo_position_consensus.apply_position_consensus(
            primary, tracks, args
        )

        np.testing.assert_allclose(output[0].ecef, origin + np.asarray([0.05, 0.0, 0.0]))
        self.assertEqual(output[0].status, 4)
        self.assertEqual(output[0].ratio, 25.0)
        self.assertEqual(summary["positions_replaced"], 1)
        self.assertEqual(summary["statuses_changed"], 0)
        self.assertFalse(summary["runtime_truth_used"])
        self.assertEqual(len(ledger), 1)

        args.fresh_shadow_max_age_epochs = 5
        stale_output, stale_summary, _ = (
            fgo_position_consensus.apply_position_consensus(primary, tracks, args)
        )
        np.testing.assert_array_equal(stale_output[0].ecef, primary_ecef)
        self.assertEqual(stale_summary["positions_replaced"], 0)

    def test_equal_size_disagreeing_shadow_clusters_are_ambiguous(self) -> None:
        samples = [
            (
                index,
                fgo_position_consensus.TrackEpoch(
                    sample=ppc_integrity_consensus.ShadowEpoch(
                        tow=1.0,
                        ecef=np.asarray([offset, 0.0, 0.0]),
                        status="FIXED",
                        gdop=2.0,
                        ddpr_rms_m=1.0,
                        nsat=10,
                    ),
                    age_epochs=10,
                ),
            )
            for index, offset in enumerate((0.0, 0.1, 10.0, 10.1))
        ]

        cluster, _, ambiguous = fgo_position_consensus.select_unique_consensus(
            samples, 2, 0.25
        )

        self.assertIsNone(cluster)
        self.assertTrue(ambiguous)

    def test_prediction_gate_rejects_a_consensus_position_jump(self) -> None:
        origin = comparison.llh_to_ecef(35.0, 139.0, 50.0)
        lat, lon, height = ppc_metrics.llh_from_ecef(*origin)
        primary = [
            comparison.SolutionEpoch(
                week=2300,
                tow=float(index),
                lat_deg=lat,
                lon_deg=lon,
                height_m=height,
                ecef=origin.copy(),
                status=4,
                num_satellites=12,
            )
            for index in range(3)
        ]
        jump = origin + np.asarray([10.0, 0.0, 0.0])
        tracks = [
            {
                2.0: fgo_position_consensus.TrackEpoch(
                    sample=ppc_integrity_consensus.ShadowEpoch(
                        tow=2.0,
                        ecef=jump + np.asarray([offset, 0.0, 0.0]),
                        status="FIXED",
                        gdop=2.0,
                        ddpr_rms_m=1.0,
                        nsat=10,
                    ),
                    age_epochs=10,
                )
            }
            for offset in (0.0, 0.1)
        ]
        args = argparse.Namespace(
            min_independent_shadows=2,
            shadow_agreement_aperture_m=0.25,
            primary_separation_min_m=0.5,
            fresh_shadow_max_age_epochs=100,
            candidate_max_prediction_error_m=2.0,
            shadow_max_gdop=0.0,
            shadow_max_ddpr_rms_m=0.0,
            shadow_min_satellites=0,
        )

        output, summary, _ = fgo_position_consensus.apply_position_consensus(
            primary, tracks, args
        )

        np.testing.assert_array_equal(output[-1].ecef, origin)
        self.assertEqual(summary["candidate_prediction_rejections"], 1)
        self.assertEqual(summary["positions_replaced"], 0)


class FgoShadowAuthorityTest(unittest.TestCase):
    def test_report_separates_recovery_from_unsafe_replacement(self) -> None:
        origin = comparison.llh_to_ecef(35.0, 139.0, 50.0)
        rotation = kf_fgo_alignment.ecef_to_enu_rotation(origin)
        correct = origin.copy()
        wrong = origin + rotation.transpose() @ np.asarray([1.0, 0.0, 0.0])
        primary_positions = [wrong, correct, correct, wrong]
        shadow_positions = [correct, wrong, correct, wrong]

        with tempfile.TemporaryDirectory(prefix="fgo_shadow_authority_") as temp_dir:
            root = Path(temp_dir)
            primary_path = root / "primary.pos"
            shadow_path = root / "shadow.csv"
            reference_path = root / "reference.csv"
            output_path = root / "report.json"
            kf_fgo_alignment.write_pos(
                primary_path,
                [
                    {
                        "week": 2300,
                        "tow": float(index),
                        "ecef": ecef,
                        "status": 4,
                        "nsat": 12,
                        "pdop": 1.0,
                        "ratio": 10.0,
                        "baseline": 1.0,
                    }
                    for index, ecef in enumerate(primary_positions)
                ],
            )
            with reference_path.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.writer(handle)
                writer.writerow(["tow", "week", "lat", "lon", "height", "x", "y", "z"])
                for index in range(4):
                    writer.writerow([index, 2300, 35.0, 139.0, 50.0, *origin])
            with shadow_path.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.writer(handle)
                writer.writerow(
                    [
                        "tow",
                        "status",
                        "x_ecef_m",
                        "y_ecef_m",
                        "z_ecef_m",
                        "gdop",
                        "ddpr_rms_m",
                        "nsat",
                    ]
                )
                for index, ecef in enumerate(shadow_positions):
                    writer.writerow([index, "FIXED", *ecef, 2.0, 1.0, 12])
            args = argparse.Namespace(
                primary_pos=primary_path,
                shadow_csv=[shadow_path],
                reference_csv=reference_path,
                output_json=output_path,
                match_tolerance_s=0.11,
                wrong_fix_threshold_m=0.5,
                shadow_max_gdop=4.0,
                shadow_max_ddpr_rms_m=40.0,
                shadow_min_satellites=8,
                aperture_m=[1.1],
            )

            report = fgo_shadow_authority.build_report(args)

        self.assertTrue(report["reference_truth_used"])
        self.assertEqual(report["matched_primary_fixed_epochs"], 4)
        self.assertEqual(report["healthy"]["recoveries"], 1)
        self.assertEqual(report["healthy"]["unsafe_replacements"], 1)
        self.assertEqual(report["healthy"]["both_correct"], 1)
        self.assertEqual(report["healthy"]["both_wrong"], 1)
        self.assertEqual(
            report["healthy_by_separation_aperture_m"]["1.1"]["selected_epochs"],
            4,
        )


class KfFgoAlignmentTest(unittest.TestCase):
    def test_causal_alignment_uses_only_solver_positions(self) -> None:
        origin = comparison.llh_to_ecef(35.0, 139.0, 50.0)
        rotation = kf_fgo_alignment.ecef_to_enu_rotation(origin)
        offset_enu = np.asarray([0.4, -0.2, 0.3])
        offset_ecef = rotation.transpose() @ offset_enu
        kf_rows = [
            {
                "week": 2300,
                "tow": float(index),
                "ecef": origin + offset_ecef,
                "status": 4,
                "nsat": 12,
                "pdop": 1.0,
                "ratio": 25.0,
                "baseline": 1.0,
            }
            for index in range(10)
        ]
        fgo_rows = [
            {
                "tow": float(index),
                "ecef": origin.copy(),
                "status": 3,
                "nsat": 12,
                "ratio": 0.0,
            }
            for index in range(10)
        ]
        args = argparse.Namespace(
            match_tolerance_s=0.11,
            horizontal_ratio_min=20.0,
            horizontal_window=5,
            horizontal_gate_m=0.5,
            vertical_ratio_min=3.0,
            vertical_window=5,
            vertical_gate_m=0.75,
            kf_fallback=False,
            kf_gap_fill_only=False,
            replace_kf_nonfixed=False,
        )

        fused, summary = kf_fgo_alignment.fuse(kf_rows, fgo_rows, args)

        np.testing.assert_allclose(fused[-1]["ecef"], origin + offset_ecef, atol=1e-5)
        self.assertFalse(summary["reference_truth_used"])
        self.assertEqual(summary["horizontal_updates_accepted"], 10)
        self.assertEqual(summary["vertical_updates_accepted"], 10)

    def test_fixed_alignment_is_explicit_and_truth_free(self) -> None:
        origin = comparison.llh_to_ecef(35.0, 139.0, 50.0)
        rotation = kf_fgo_alignment.ecef_to_enu_rotation(origin)
        offset_ecef = rotation.transpose() @ np.asarray([0.4, -0.2, 0.3])
        kf_rows = [
            {
                "week": 2300,
                "tow": float(index),
                "ecef": origin + offset_ecef,
                "status": 4,
                "nsat": 12,
                "pdop": 1.0,
                "ratio": 25.0,
                "baseline": 1.0,
            }
            for index in range(3)
        ]
        fgo_rows = [
            {
                "tow": float(index),
                "ecef": origin.copy(),
                "status": 4,
                "nsat": 12,
                "ratio": 5.0,
            }
            for index in range(3)
        ]
        args = argparse.Namespace(
            match_tolerance_s=0.11,
            horizontal_ratio_min=20.0,
            horizontal_window=5,
            horizontal_gate_m=0.5,
            vertical_ratio_min=3.0,
            vertical_window=5,
            vertical_gate_m=0.75,
            kf_fallback=False,
            kf_gap_fill_only=False,
            replace_kf_nonfixed=False,
            align_fgo_fixed=True,
        )

        fused, summary = kf_fgo_alignment.fuse(kf_rows, fgo_rows, args)

        np.testing.assert_allclose(fused[-1]["ecef"], origin + offset_ecef, atol=1e-5)
        self.assertTrue(summary["align_fgo_fixed"])
        self.assertFalse(summary["reference_truth_used"])

    def test_gap_fill_mode_preserves_available_kf_epochs(self) -> None:
        origin = comparison.llh_to_ecef(35.0, 139.0, 50.0)
        rotation = kf_fgo_alignment.ecef_to_enu_rotation(origin)
        offset_ecef = rotation.transpose() @ np.asarray([0.2, 0.1, 0.1])
        kf_rows = [
            {
                "week": 2300,
                "tow": tow,
                "ecef": origin + offset_ecef,
                "status": 4,
                "nsat": 12,
                "pdop": 1.0,
                "ratio": 25.0,
                "baseline": 1.0,
            }
            for tow in (0.0, 2.0)
        ]
        fgo_rows = [
            {"tow": tow, "ecef": origin.copy(), "status": 4, "nsat": 10, "ratio": 5.0}
            for tow in (0.0, 1.0, 2.0)
        ]
        args = argparse.Namespace(
            match_tolerance_s=0.11,
            horizontal_ratio_min=20.0,
            horizontal_window=5,
            horizontal_gate_m=0.5,
            vertical_ratio_min=3.0,
            vertical_window=5,
            vertical_gate_m=0.75,
            kf_fallback=True,
            kf_gap_fill_only=True,
            replace_kf_nonfixed=False,
            min_window_alignment_updates=0,
        )

        fused, summary = kf_fgo_alignment.fuse(kf_rows, fgo_rows, args)

        self.assertEqual([float(row["tow"]) for row in fused], [0.0, 1.0, 2.0])
        np.testing.assert_array_equal(fused[0]["ecef"], kf_rows[0]["ecef"])
        np.testing.assert_array_equal(fused[2]["ecef"], kf_rows[1]["ecef"])
        self.assertEqual(int(fused[1]["status"]), 3)
        self.assertTrue(summary["kf_gap_fill_only"])

    def test_multiple_gap_windows_merge_in_cli_order_without_duplicates(self) -> None:
        origin = comparison.llh_to_ecef(35.0, 139.0, 50.0)
        kf_rows = [
            {
                "week": 2300,
                "tow": 0.0,
                "ecef": origin.copy(),
                "status": 4,
                "nsat": 12,
                "pdop": 1.0,
                "ratio": 25.0,
                "baseline": 1.0,
            }
        ]
        first = [
            {"tow": tow, "ecef": origin.copy(), "status": 3, "nsat": 10, "ratio": 0.0}
            for tow in (0.0, 1.0)
        ]
        second = [
            {"tow": tow, "ecef": origin.copy(), "status": 3, "nsat": 10, "ratio": 0.0}
            for tow in (1.0, 2.0)
        ]
        args = argparse.Namespace(
            match_tolerance_s=0.11,
            horizontal_ratio_min=3.0,
            horizontal_window=10,
            horizontal_gate_m=5.0,
            vertical_ratio_min=3.0,
            vertical_window=5,
            vertical_gate_m=2.0,
            kf_fallback=True,
            kf_gap_fill_only=True,
            replace_kf_nonfixed=False,
        )

        fused, summary = kf_fgo_alignment.fuse_windows(kf_rows, [first, second], args)

        self.assertEqual([float(row["tow"]) for row in fused], [0.0, 1.0, 2.0])
        self.assertEqual(summary["fgo_window_count"], 2)
        self.assertEqual(summary["output_epochs"], 3)

    def test_replace_nonfixed_preserves_kf_fix_and_uses_fgo_for_float(self) -> None:
        origin = comparison.llh_to_ecef(35.0, 139.0, 50.0)
        kf_rows = [
            {
                "week": 2300,
                "tow": tow,
                "ecef": origin + np.asarray([10.0 if status == 4 else 20.0, 0.0, 0.0]),
                "status": status,
                "nsat": 12,
                "pdop": 1.0,
                "ratio": 25.0 if status == 4 else 0.0,
                "baseline": 1.0,
            }
            for tow, status in ((0.0, 4), (1.0, 3))
        ]
        fgo_rows = [
            {"tow": tow, "ecef": origin.copy(), "status": 3, "nsat": 10, "ratio": 0.0}
            for tow in (0.0, 1.0)
        ]
        args = argparse.Namespace(
            match_tolerance_s=0.11,
            horizontal_ratio_min=3.0,
            horizontal_window=10,
            horizontal_gate_m=5.0,
            vertical_ratio_min=3.0,
            vertical_window=5,
            vertical_gate_m=2.0,
            kf_fallback=True,
            kf_gap_fill_only=True,
            replace_kf_nonfixed=True,
            min_window_alignment_updates=0,
        )

        fused, summary = kf_fgo_alignment.fuse(kf_rows, fgo_rows, args)

        self.assertEqual(len(fused), 2)
        self.assertEqual(len({(int(row["week"]), float(row["tow"])) for row in fused}), 2)
        np.testing.assert_array_equal(fused[0]["ecef"], kf_rows[0]["ecef"])
        np.testing.assert_allclose(fused[1]["ecef"], kf_rows[0]["ecef"], atol=1e-6)
        self.assertFalse(np.array_equal(fused[1]["ecef"], kf_rows[1]["ecef"]))
        self.assertEqual(int(fused[0]["status"]), 4)
        self.assertEqual(int(fused[1]["status"]), 3)
        self.assertTrue(summary["replace_kf_nonfixed"])

    def test_replace_nis_rejected_fix_uses_aligned_fgo_float(self) -> None:
        origin = comparison.llh_to_ecef(35.0, 139.0, 50.0)
        trusted_ecef = origin + np.asarray([10.0, 0.0, 0.0])
        rejected_ecef = origin + np.asarray([20.0, 0.0, 0.0])
        kf_rows = [
            {
                "week": 2300,
                "tow": tow,
                "ecef": ecef,
                "status": 4,
                "nsat": 12,
                "pdop": 1.0,
                "ratio": 25.0,
                "baseline": 1.0,
                "nis_per_observation": nis,
            }
            for tow, ecef, nis in (
                (0.0, trusted_ecef, 1.0),
                (1.0, rejected_ecef, 10.0),
            )
        ]
        fgo_rows = [
            {"tow": tow, "ecef": origin.copy(), "status": 3, "nsat": 10, "ratio": 0.0}
            for tow in (0.0, 1.0)
        ]
        args = argparse.Namespace(
            match_tolerance_s=0.11,
            horizontal_ratio_min=3.0,
            horizontal_window=10,
            horizontal_gate_m=5.0,
            vertical_ratio_min=3.0,
            vertical_window=5,
            vertical_gate_m=2.0,
            alignment_kf_max_nis_per_observation=5.0,
            kf_fallback=True,
            kf_gap_fill_only=True,
            replace_kf_nonfixed=True,
            replace_kf_nis_rejected=True,
            min_window_alignment_updates=0,
        )

        fused, summary = kf_fgo_alignment.fuse(kf_rows, fgo_rows, args)

        np.testing.assert_array_equal(fused[0]["ecef"], trusted_ecef)
        np.testing.assert_allclose(fused[1]["ecef"], trusted_ecef, atol=1e-6)
        self.assertEqual(int(fused[0]["status"]), 4)
        self.assertEqual(int(fused[1]["status"]), 3)
        self.assertEqual(summary["kf_telemetry_rejections"], 1)
        self.assertEqual(summary["kf_nis_rejected_replacements"], 1)
        self.assertTrue(summary["replace_kf_nis_rejected"])

    def test_replacement_nis_and_fgo_quality_gates_are_independent(self) -> None:
        origin = comparison.llh_to_ecef(35.0, 139.0, 50.0)
        kf_rows = [
            {
                "week": 2300,
                "tow": tow,
                "ecef": origin + np.asarray([10.0 + tow, 0.0, 0.0]),
                "status": 4,
                "nsat": 12,
                "pdop": 1.0,
                "ratio": 25.0,
                "baseline": 1.0,
                "nis_per_observation": 10.0,
            }
            for tow in (0.0, 1.0)
        ]
        fgo_rows = [
            {
                "tow": 0.0,
                "ecef": origin.copy(),
                "status": 3,
                "nsat": 10,
                "ratio": 0.0,
                "ddpr_rms_m": 2.0,
                "gdop": 1.0,
            },
            {
                "tow": 1.0,
                "ecef": origin.copy(),
                "status": 3,
                "nsat": 10,
                "ratio": 0.0,
                "ddpr_rms_m": 10.0,
                "gdop": 2.0,
            },
        ]
        args = argparse.Namespace(
            match_tolerance_s=0.11,
            horizontal_ratio_min=3.0,
            horizontal_window=10,
            horizontal_gate_m=5.0,
            vertical_ratio_min=3.0,
            vertical_window=5,
            vertical_gate_m=2.0,
            alignment_kf_max_nis_per_observation=0.0,
            replace_kf_nis_threshold=5.0,
            replace_kf_nis_min_fgo_ddpr_rms_m=6.0,
            replace_kf_nis_min_fgo_gdop=1.5,
            kf_fallback=True,
            kf_gap_fill_only=True,
            replace_kf_nonfixed=True,
            replace_kf_nis_rejected=True,
            min_window_alignment_updates=0,
        )

        fused, summary = kf_fgo_alignment.fuse(kf_rows, fgo_rows, args)

        np.testing.assert_array_equal(fused[0]["ecef"], kf_rows[0]["ecef"])
        self.assertEqual(int(fused[0]["status"]), 4)
        self.assertEqual(int(fused[1]["status"]), 3)
        self.assertEqual(summary["horizontal_updates_accepted"], 2)
        self.assertEqual(summary["kf_telemetry_rejections"], 0)
        self.assertEqual(summary["kf_nis_rejected_replacements"], 1)

    def test_multi_window_replace_nonfixed_reports_actual_merge_policy(self) -> None:
        origin = comparison.llh_to_ecef(35.0, 139.0, 50.0)
        kf_rows = [
            {
                "week": 2300,
                "tow": 0.0,
                "ecef": origin.copy(),
                "status": 4,
                "nsat": 12,
                "pdop": 1.0,
                "ratio": 25.0,
                "baseline": 1.0,
            }
        ]
        window = [{"tow": 0.0, "ecef": origin.copy(), "status": 3, "nsat": 10, "ratio": 0.0}]
        args = argparse.Namespace(
            match_tolerance_s=0.11,
            horizontal_ratio_min=3.0,
            horizontal_window=10,
            horizontal_gate_m=5.0,
            vertical_ratio_min=3.0,
            vertical_window=5,
            vertical_gate_m=2.0,
            kf_fallback=True,
            kf_gap_fill_only=True,
            replace_kf_nonfixed=True,
            min_window_alignment_updates=0,
        )

        _, summary = kf_fgo_alignment.fuse_windows(kf_rows, [window, window], args)

        self.assertEqual(
            summary["merge_policy"],
            "KF FIXED preserved; FGO windows in CLI order replace non-FIXED/missing; "
            "first solution wins per week/tow",
        )

    def test_multi_window_alignment_update_gate_rejects_unanchored_window(self) -> None:
        origin = comparison.llh_to_ecef(35.0, 139.0, 50.0)
        kf_rows = [
            {
                "week": 2300,
                "tow": 0.0,
                "ecef": origin.copy(),
                "status": 3,
                "nsat": 12,
                "pdop": 1.0,
                "ratio": 0.0,
                "baseline": 1.0,
            }
        ]
        window = [{"tow": 0.0, "ecef": origin.copy(), "status": 3, "nsat": 10, "ratio": 0.0}]
        args = argparse.Namespace(
            match_tolerance_s=0.11,
            horizontal_ratio_min=3.0,
            horizontal_window=10,
            horizontal_gate_m=5.0,
            vertical_ratio_min=3.0,
            vertical_window=5,
            vertical_gate_m=2.0,
            kf_fallback=True,
            kf_gap_fill_only=True,
            replace_kf_nonfixed=True,
            min_window_alignment_updates=1,
        )

        fused, summary = kf_fgo_alignment.fuse_windows(kf_rows, [window, window], args)

        self.assertEqual(len(fused), 1)
        self.assertEqual(summary["fgo_windows_accepted"], 0)
        self.assertFalse(summary["windows"][0]["window_accepted"])

    def test_alignment_age_gate_stops_unanchored_gap_fill(self) -> None:
        origin = comparison.llh_to_ecef(35.0, 139.0, 50.0)
        kf_rows = [
            {
                "week": 2300,
                "tow": 0.0,
                "ecef": origin.copy(),
                "status": 4,
                "nsat": 12,
                "pdop": 1.0,
                "ratio": 25.0,
                "baseline": 1.0,
            }
        ]
        fgo_rows = [
            {"tow": tow, "ecef": origin.copy(), "status": 3, "nsat": 10, "ratio": 0.0}
            for tow in (0.0, 1.0, 2.0)
        ]
        args = argparse.Namespace(
            match_tolerance_s=0.11,
            horizontal_ratio_min=3.0,
            horizontal_window=10,
            horizontal_gate_m=5.0,
            vertical_ratio_min=3.0,
            vertical_window=5,
            vertical_gate_m=2.0,
            kf_fallback=True,
            kf_gap_fill_only=True,
            replace_kf_nonfixed=True,
            max_alignment_age_epochs=1,
        )

        fused, summary = kf_fgo_alignment.fuse(kf_rows, fgo_rows, args)

        self.assertEqual([float(row["tow"]) for row in fused], [0.0, 1.0])
        self.assertEqual(summary["stale_fgo_rejections"], 1)

    def test_alignment_initialization_gate_rejects_pre_anchor_gap_fill(self) -> None:
        origin = comparison.llh_to_ecef(35.0, 139.0, 50.0)
        kf_rows = [
            {
                "week": 2300,
                "tow": 1.0,
                "ecef": origin.copy(),
                "status": 4,
                "nsat": 12,
                "pdop": 1.0,
                "ratio": 25.0,
                "baseline": 1.0,
            }
        ]
        fgo_rows = [
            {"tow": tow, "ecef": origin.copy(), "status": 3, "nsat": 10, "ratio": 0.0}
            for tow in (0.0, 1.0)
        ]
        args = argparse.Namespace(
            match_tolerance_s=0.11,
            horizontal_ratio_min=3.0,
            horizontal_window=10,
            horizontal_gate_m=5.0,
            vertical_ratio_min=3.0,
            vertical_window=5,
            vertical_gate_m=2.0,
            kf_fallback=True,
            kf_gap_fill_only=True,
            replace_kf_nonfixed=True,
            max_alignment_age_epochs=0,
            require_alignment_initialized=True,
        )

        fused, summary = kf_fgo_alignment.fuse(kf_rows, fgo_rows, args)

        self.assertEqual([float(row["tow"]) for row in fused], [1.0])
        self.assertEqual(summary["uninitialized_alignment_rejections"], 1)
        self.assertEqual(summary["stale_fgo_rejections"], 0)
        self.assertTrue(summary["require_alignment_initialized"])

    def test_alignment_trend_causally_extrapolates_fgo_drift(self) -> None:
        origin = comparison.llh_to_ecef(35.0, 139.0, 50.0)
        rotation = kf_fgo_alignment.ecef_to_enu_rotation(origin)
        kf_rows = [
            {
                "week": 2300,
                "tow": float(index),
                "ecef": origin.copy(),
                "status": 4,
                "nsat": 12,
                "pdop": 1.0,
                "ratio": 25.0,
                "baseline": 1.0,
            }
            for index in range(6)
        ]
        fgo_rows = [
            {
                "tow": float(index),
                "ecef": origin + rotation.transpose() @ np.asarray([float(index), 0.0, 0.0]),
                "status": 3,
                "nsat": 10,
                "ratio": 0.0,
                "ddpr_rms_m": 5.0,
                "gdop": 1.5,
            }
            for index in range(9)
        ]
        args = argparse.Namespace(
            match_tolerance_s=0.11,
            horizontal_ratio_min=3.0,
            horizontal_window=10,
            horizontal_gate_m=10.0,
            vertical_ratio_min=3.0,
            vertical_window=5,
            vertical_gate_m=2.0,
            kf_fallback=False,
            kf_gap_fill_only=True,
            replace_kf_nonfixed=True,
            max_alignment_age_epochs=0,
            alignment_trend_window=6,
            alignment_trend_max_rate_mps=2.0,
            alignment_trend_max_ddpr_rms_m=10.0,
        )

        fused, summary = kf_fgo_alignment.fuse(kf_rows, fgo_rows, args)

        np.testing.assert_allclose(fused[-1]["ecef"], origin, atol=1e-4)
        self.assertGreater(summary["trend_prediction_epochs"], 0)

        args.alignment_trend_max_ddpr_rms_m = 1.0
        median_only, gated_summary = kf_fgo_alignment.fuse(kf_rows, fgo_rows, args)
        self.assertGreater(float(np.linalg.norm(median_only[-1]["ecef"] - origin)), 1.0)
        self.assertEqual(gated_summary["trend_prediction_epochs"], 0)

        args.alignment_trend_max_ddpr_rms_m = 10.0
        args.alignment_trend_max_gdop = 1.0
        gdop_gated, gdop_summary = kf_fgo_alignment.fuse(kf_rows, fgo_rows, args)
        self.assertGreater(float(np.linalg.norm(gdop_gated[-1]["ecef"] - origin)), 1.0)
        self.assertEqual(gdop_summary["trend_prediction_epochs"], 0)
        self.assertEqual(gdop_summary["alignment_trend_max_gdop"], 1.0)

        args.alignment_trend_max_gdop = 0.0
        args.alignment_kf_max_nis_per_observation = 1.0
        _, missing_nis_summary = kf_fgo_alignment.fuse(kf_rows, fgo_rows, args)
        self.assertEqual(missing_nis_summary["horizontal_updates_accepted"], 0)
        self.assertEqual(missing_nis_summary["vertical_updates_accepted"], 0)
        self.assertGreater(missing_nis_summary["kf_telemetry_rejections"], 0)

        for row in kf_rows:
            row["nis_per_observation"] = 0.5
        nis_accepted, nis_summary = kf_fgo_alignment.fuse(kf_rows, fgo_rows, args)
        np.testing.assert_allclose(nis_accepted[-1]["ecef"], origin, atol=1e-4)
        self.assertGreater(nis_summary["trend_prediction_epochs"], 0)

    def test_alignment_trend_dropout_rate_bridges_good_ddpr_gdop_loss(self) -> None:
        origin = comparison.llh_to_ecef(35.0, 139.0, 50.0)
        rotation = kf_fgo_alignment.ecef_to_enu_rotation(origin)
        kf_rows = [
            {
                "week": 2300,
                "tow": float(index),
                "ecef": origin.copy(),
                "status": 4,
                "nsat": 12,
                "pdop": 1.0,
                "ratio": 25.0,
                "baseline": 1.0,
            }
            for index in range(6)
        ]
        fgo_rows = [
            {
                "tow": float(index),
                "ecef": origin
                + rotation.transpose()
                @ np.asarray([float(index), 0.0, 0.0]),
                "status": 3,
                "nsat": 10,
                "ratio": 0.0,
                "ddpr_rms_m": 1.0,
                "gdop": 1.5 if index < 6 else np.inf,
            }
            for index in range(9)
        ]
        args = argparse.Namespace(
            match_tolerance_s=0.11,
            horizontal_ratio_min=3.0,
            horizontal_window=10,
            horizontal_gate_m=10.0,
            vertical_ratio_min=3.0,
            vertical_window=5,
            vertical_gate_m=2.0,
            kf_fallback=False,
            kf_gap_fill_only=True,
            replace_kf_nonfixed=True,
            max_alignment_age_epochs=0,
            alignment_trend_window=6,
            alignment_trend_max_rate_mps=0.1,
            alignment_trend_max_ddpr_rms_m=10.0,
        )

        slow, _ = kf_fgo_alignment.fuse(kf_rows, fgo_rows, args)
        slow_error = float(np.linalg.norm(slow[-1]["ecef"] - origin))

        args.alignment_trend_dropout_max_ddpr_rms_m = 1.1
        args.alignment_trend_dropout_max_rate_mps = 2.0
        bridged, summary = kf_fgo_alignment.fuse(kf_rows, fgo_rows, args)
        bridged_error = float(np.linalg.norm(bridged[-1]["ecef"] - origin))

        self.assertGreater(slow_error, 1.0)
        self.assertLess(bridged_error, 1e-4)
        self.assertLess(bridged_error, slow_error)
        self.assertEqual(summary["alignment_trend_dropout_max_ddpr_rms_m"], 1.1)
        self.assertEqual(summary["alignment_trend_dropout_max_rate_mps"], 2.0)

    def test_alignment_pos_round_trip_preserves_kf_rtk_telemetry(self) -> None:
        row = {
            "week": 2300,
            "tow": 1.0,
            "ecef": comparison.llh_to_ecef(35.0, 139.0, 50.0),
            "status": 4,
            "nsat": 12,
            "pdop": 1.0,
            "ratio": 25.0,
            "baseline": 1.0,
            "rtk_fields": ["2", "20", "10", "10", "1", "0.5", "2.0", "0.4", "1.5", "8.0", "0.4", "0"],
        }
        with tempfile.TemporaryDirectory(prefix="kf_fgo_telemetry_") as temp_dir:
            path = Path(temp_dir) / "aligned.pos"
            kf_fgo_alignment.write_pos(path, [row])
            parsed = kf_fgo_alignment.read_kf(path)

        self.assertEqual(parsed[0]["rtk_fields"], row["rtk_fields"])
        self.assertEqual(parsed[0]["nis_per_observation"], 0.4)

    def test_alignment_reader_accepts_pos_without_optional_telemetry(self) -> None:
        with tempfile.TemporaryDirectory(prefix="kf_fgo_short_pos_") as temp_dir:
            path = Path(temp_dir) / "short.pos"
            path.write_text(
                "% GPS_Week GPS_TOW X Y Z Lat Lon Height Status NumSat PDOP\n"
                "2300 1.000 1.0 2.0 3.0 0.0 0.0 0.0 3 0 2.0\n",
                encoding="ascii",
            )

            parsed = kf_fgo_alignment.read_kf(path)

        self.assertEqual(parsed[0]["ratio"], 0.0)
        self.assertEqual(parsed[0]["baseline"], 0.0)
        self.assertIsNone(parsed[0]["rtk_fields"])
        self.assertTrue(np.isnan(parsed[0]["nis_per_observation"]))

    def test_ppc_matrix_pos_specs_require_each_run_once(self) -> None:
        specs = [
            f"{city}/{run}=output/{city}_{run}.pos"
            for city, run in fgo_ppc_matrix.RUNS
        ]

        selected = fgo_ppc_matrix.parse_pos_specs(specs)

        self.assertEqual(set(selected), set(fgo_ppc_matrix.RUNS))
        with self.assertRaises(SystemExit):
            fgo_ppc_matrix.parse_pos_specs(specs[:-1])

    def test_libgnss_pos_parser_keeps_ratio_and_baseline_telemetry(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_pos_ratio_parse_") as temp_dir:
            pos_path = Path(temp_dir) / "solution.pos"
            pos_path.write_text(
                "\n".join(
                    [
                        "% GPS_Week GPS_TOW X Y Z Lat Lon Height Status NumSat PDOP Ratio Baseline",
                        "2300 1.000 10.0 0.0 0.0 0.0 0.0 0.0 4 12 2.0 "
                        "17.5 9400.25 2 16 8 8 1 2.5 31.0 1.2 20.0 64.0 4.0 1",
                    ]
                )
                + "\n",
                encoding="ascii",
            )

            epochs = comparison.read_libgnss_pos(pos_path)

            self.assertEqual(len(epochs), 1)
            self.assertEqual(epochs[0].ratio, 17.5)
            self.assertEqual(epochs[0].baseline_m, 9400.25)
            self.assertEqual(epochs[0].rtk_iterations, 2)
            self.assertEqual(epochs[0].rtk_update_observations, 16)
            self.assertEqual(epochs[0].rtk_update_phase_observations, 8)
            self.assertEqual(epochs[0].rtk_update_code_observations, 8)
            self.assertEqual(epochs[0].rtk_update_suppressed_outliers, 1)
            self.assertEqual(epochs[0].rtk_update_prefit_residual_rms_m, 2.5)
            self.assertEqual(epochs[0].rtk_update_prefit_residual_max_m, 31.0)
            self.assertEqual(epochs[0].rtk_update_post_suppression_residual_rms_m, 1.2)
            self.assertEqual(epochs[0].rtk_update_post_suppression_residual_max_m, 20.0)
            self.assertEqual(epochs[0].rtk_update_normalized_innovation_squared, 64.0)
            self.assertEqual(
                epochs[0].rtk_update_normalized_innovation_squared_per_observation,
                4.0,
            )
            self.assertEqual(epochs[0].rtk_update_rejected_by_innovation_gate, 1)

    def test_rtklib_pos_parser_keeps_ratio_telemetry(self) -> None:
        with tempfile.TemporaryDirectory(prefix="rtklib_pos_ratio_parse_") as temp_dir:
            pos_path = Path(temp_dir) / "rtklib.pos"
            pos_path.write_text(
                "\n".join(
                    [
                        "%  GPST latitude(deg) longitude(deg) height(m) Q ns sdn sde sdu sdne sdeu sdun age ratio",
                        "2024/07/20 10:22:00.000 35.165452362 136.881445510 "
                        "41.1780 1 7 0.0049 0.0044 0.0135 0.0026 0.0030 "
                        "0.0031 0.00 8.5",
                    ]
                )
                + "\n",
                encoding="ascii",
            )

            epochs = comparison.read_rtklib_pos(pos_path)

            self.assertEqual(len(epochs), 1)
            self.assertEqual(epochs[0].ratio, 8.5)
            self.assertIsNone(epochs[0].baseline_m)

    def test_official_distance_score_weights_reference_distance(self) -> None:
        reference = [
            comparison.ReferenceEpoch(2300, 0.0, 0.0, 0.0, 0.0, (0.0, 0.0, 0.0)),
            comparison.ReferenceEpoch(2300, 1.0, 0.0, 0.0, 0.0, (10.0, 0.0, 0.0)),
            comparison.ReferenceEpoch(2300, 2.0, 0.0, 0.0, 0.0, (20.0, 0.0, 0.0)),
            comparison.ReferenceEpoch(2300, 3.0, 0.0, 0.0, 0.0, (40.0, 0.0, 0.0)),
        ]
        solution = [
            comparison.SolutionEpoch(2300, 1.0, 0.0, 0.0, 0.0, (10.2, 0.0, 0.0), 4, 12),
            comparison.SolutionEpoch(2300, 2.0, 0.0, 0.0, 0.0, (21.0, 0.0, 0.0), 4, 12),
        ]

        score = ppc_metrics.ppc_official_distance_score(reference, solution, 0.25)

        self.assertEqual(score["ppc_official_total_distance_m"], 40.0)
        self.assertEqual(score["ppc_official_matched_distance_m"], 20.0)
        self.assertEqual(score["ppc_official_score_distance_m"], 10.0)
        self.assertEqual(score["ppc_official_score_pct"], 25.0)

        records = ppc_metrics.ppc_official_segment_records(reference, solution, 0.25)
        self.assertEqual([record["score_state"] for record in records], ["scored", "high_error", "no_solution"])
        self.assertEqual(records[0]["status"], 4)
        self.assertEqual(records[1]["status"], 4)
        self.assertIsNone(records[2]["status"])
