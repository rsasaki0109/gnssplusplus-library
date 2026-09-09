#!/usr/bin/env python3
"""Source-level CLI composition checks for the Phase187 stop selector.

These checks intentionally use no Android inputs.  The app's parser has a
large raw-input admission surface, so this focused contract test catches the
legacy Phase93 guard that used to reject the Phase171+stop composition before
any payload was opened.
"""

from pathlib import Path
import unittest


APP = Path(__file__).resolve().parents[1] / "apps/native/gnss_fgo_imu_no_base.cpp"


class Phase187UpstreamStopCliTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = APP.read_text(encoding="utf-8")

    def test_phase171_exempts_stop_from_legacy_phase93_recipe_guard(self):
        start = self.source.index(
            "if (options.native_source_clock_c0d_gnss_first_meter_state_handoff)"
        )
        end = self.source.index(
            "if (options.native_source_clock_c0d_epoch_vector_parity)", start
        )
        guard = self.source[start:end]
        self.assertIn(
            "(options.native_upstream_stop_constraints && !phase171_imu_main)",
            guard,
        )
        self.assertNotIn(
            "\n            options.native_upstream_stop_constraints ||", guard
        )

    def test_phase171_stages_raw_p_and_retains_main_stop_selector(self):
        phase171 = self.source.index("} else if (phase171_imu_main) {")
        stage_end = self.source.index(
            "gnss_first_config.use_velocity_motion_factors = phase171_imu_main",
            phase171,
        )
        stage = self.source[phase171:stage_end]
        self.assertIn("gnss_first_config.use_imu = false;", stage)
        self.assertIn(
            "gnss_first_config.use_native_raw_p_no_doppler_graph =\n"
            "                !phase171_ecef_doppler;",
            stage,
        )
        self.assertIn(
            "gnss_first_config.use_native_raw_p_ecef_doppler_gnss_first =\n"
            "                phase171_ecef_doppler;",
            stage,
        )
        # The stage is raw-P only; the stop selector is consumed by the later
        # Pose3+IMU main config, not by a second serialized or raw-D lane.
        self.assertIn("gnss_first_config.use_native_phase171_raw_p_no_doppler_imu_main =\n                false;", stage)
        main = self.source[self.source.index(
            "if (options.native_upstream_stop_constraints)"
        ):]
        self.assertIn("config.use_upstream_stop_constraints = true;", main)


if __name__ == "__main__":
    unittest.main()
