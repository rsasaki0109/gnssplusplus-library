#!/usr/bin/env python3
"""Source-contract tests for the opt-in Phase194 IMU noise parity lane."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"
NOISE = ROOT / "include/libgnss++/algorithms/native_utc_fallback_imu_noise.hpp"


class Phase194SourceUtcFallbackNoiseTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = APP.read_text(encoding="utf-8")
        cls.noise = NOISE.read_text(encoding="utf-8")

    def test_selector_is_default_off_and_narrowly_gated(self):
        self.assertIn("bool native_phase194_source_utc_fallback_imu_noise = false;", self.app)
        self.assertIn("phase171_imu_main || !phase171_ecef_doppler", self.app)
        self.assertIn("!options.android_utc_wall_clock_fallback", self.app)
        self.assertIn("phoneFromDatasetId(options.dataset_id) != \"pixel5\"", self.app)

    def test_actual_loader_fallback_controls_policy_application(self):
        self.assertIn("report.android_load.utc_wall_clock_fallback_applied", self.app)
        self.assertIn("native_utc_fallback_imu_noise::apply", self.app)
        self.assertIn("options.native_phase194_source_utc_fallback_imu_noise", self.app)
        self.assertIn("android_config.imu_sync_coefficient = kUpstreamImuSyncCoefficient", self.app)

    def test_summary_reports_chosen_branch_and_component_covariances(self):
        for token in (
            'imu_measurement_noise',
            'source_branch',
            'measurement_sync_coefficient',
            'accel_noise_sigma',
            'gyro_noise_sigma',
            'accel_covariance_diagonal',
            'gyro_covariance_diagonal',
            'bias_random_walk_unchanged',
            'integration_noise_unchanged',
        ):
            self.assertIn(token, self.app)

    def test_noise_policy_changes_only_white_noise_densities(self):
        self.assertIn("kSourceFallbackAccelNoiseSigma = 0.05", self.noise)
        self.assertIn("kSourceFallbackGyroNoiseSigma = 0.001", self.noise)
        self.assertIn("noise.accel_noise_sigma = selection.accel_noise_sigma", self.noise)
        self.assertIn("noise.gyro_noise_sigma = selection.gyro_noise_sigma", self.noise)
        self.assertNotIn("noise.accel_bias_rw_sigma", self.noise)
        self.assertNotIn("noise.gyro_bias_rw_sigma", self.noise)
        self.assertNotIn("noise.integration_sigma", self.noise)


if __name__ == "__main__":
    unittest.main()
