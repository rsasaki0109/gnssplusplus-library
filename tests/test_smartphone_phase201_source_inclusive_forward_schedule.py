"""Static contract tests for the opt-in Phase201 IMU schedule lane."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
APP = (ROOT / "apps/native/gnss_fgo_imu_no_base.cpp").read_text(encoding="utf-8")
CONFIG = (ROOT / "include/libgnss++/algorithms/fgo_config.hpp").read_text(
    encoding="utf-8"
)
BACKEND = (ROOT / "src/algorithms/fgo_gtsam_backend.cpp").read_text(
    encoding="utf-8"
)
INTERNAL = (ROOT / "src/algorithms/fgo_gtsam_internal.hpp").read_text(
    encoding="utf-8"
)


class Phase201SourceInclusiveForwardScheduleTest(unittest.TestCase):
    def test_selector_is_default_off_and_cli_is_narrowly_gated(self):
        self.assertIn(
            "bool native_phase201_source_inclusive_forward_imu_schedule = false;",
            APP,
        )
        self.assertIn(
            "--native-phase201-source-inclusive-forward-imu-schedule", APP
        )
        self.assertIn("!phase171_imu_main || !phase171_ecef_doppler", APP)
        self.assertIn("!options.android_utc_wall_clock_fallback", APP)
        self.assertIn('phoneFromDatasetId(options.dataset_id) != "pixel5"', APP)
        self.assertIn(
            "use_native_phase201_source_inclusive_forward_imu_schedule =\n"
            "                false;",
            APP,
        )

    def test_backend_guard_and_config_wiring_are_dedicated(self):
        self.assertIn(
            "use_native_phase201_source_inclusive_forward_imu_schedule", CONFIG
        )
        self.assertIn("const bool phase201_requested", BACKEND)
        self.assertIn("!phase171_requested", BACKEND)
        self.assertIn("!use_imu", BACKEND)
        self.assertIn('config.native_source_clock_c0d_phone != "pixel5"', BACKEND)
        self.assertIn(
            "config.use_native_phase201_source_inclusive_forward_imu_schedule =",
            APP,
        )

    def test_new_branch_preserves_legacy_loop_and_fails_closed(self):
        branch = "if (phase201_requested)"
        self.assertIn(branch, BACKEND)
        branch_start = BACKEND.index(branch)
        legacy_start = BACKEND.index(
            "while (sample_cursor < imu_samples.size()", branch_start
        )
        self.assertGreater(legacy_start, branch_start)
        self.assertIn("makeSourceInclusiveForwardImuSchedule", BACKEND)
        self.assertIn("return result;", BACKEND[branch_start:legacy_start])
        self.assertIn("native_phase201_configuration_failure", BACKEND)
        self.assertIn("native_phase201_duration_error_max_abs_s", APP)

    def test_helper_uses_exact_inclusive_forward_mapped_times(self):
        self.assertIn("sample_offset_s < 0.0 || sample_offset_s > interval_s", INTERNAL)
        self.assertIn("samples[i + 1].time - samples[i].time", INTERNAL)
        self.assertIn("samples[i].time - samples[i - 1].time", INTERNAL)
        self.assertIn("next_sample_index", INTERNAL)
        self.assertIn("validate_all_samples", INTERNAL)


if __name__ == "__main__":
    unittest.main()
