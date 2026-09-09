"""Static contract tests for the opt-in Phase197 UTC fallback offset lane."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
APP = (ROOT / "apps/native/gnss_fgo_imu_no_base.cpp").read_text(encoding="utf-8")
IMU_HPP = (ROOT / "include/libgnss++/io/imu.hpp").read_text(encoding="utf-8")
IMU_CPP = (ROOT / "src/io/imu.cpp").read_text(encoding="utf-8")
OFFSET_HPP = (ROOT / "include/libgnss++/algorithms/native_utc_fallback_imu_offset.hpp").read_text(encoding="utf-8")


class Phase197UtcFallbackOffsetTest(unittest.TestCase):
    def test_selector_is_default_off_and_pixel5_fallback_gated(self):
        self.assertIn("bool native_phase197_source_utc_fallback_imu_offset = false;", APP)
        self.assertIn("--native-phase197-source-utc-fallback-imu-offset", APP)
        self.assertIn("!phase171_imu_main || !phase171_ecef_doppler", APP)
        self.assertIn("!options.android_utc_wall_clock_fallback", APP)
        self.assertIn('phoneFromDatasetId(options.dataset_id) != "pixel5"', APP)

    def test_loader_changes_mapped_time_not_pairing_or_anchors(self):
        self.assertIn("apply_utc_wall_clock_fallback_offset", IMU_HPP)
        self.assertIn("utc_wall_clock_fallback_offset_ms", IMU_HPP)
        self.assertIn("mapped_utc_time_ms += offset_ms", IMU_CPP)
        self.assertIn("gpsNanosAtUtc(mapped_utc_time_ms)", IMU_CPP)
        self.assertIn("sample.pairing_time_ns = sample.utc_time_ms * kMillisToNanos", IMU_CPP)
        self.assertIn("if (use_gnss_anchor)", IMU_CPP)
        self.assertIn("else if (use_utc_wall_clock_fallback)", IMU_CPP)
        self.assertIn("kSourceUtcWallClockOffsetMs = -20", OFFSET_HPP)

    def test_underflow_and_actual_branch_are_reported(self):
        self.assertIn("underflows timestamp", IMU_CPP)
        self.assertIn("utc_wall_clock_fallback_offset_applied", IMU_HPP)
        self.assertIn('imu_utc_fallback_offset', APP)
        self.assertIn('configured_offset_ms', APP)
        self.assertIn('effective_offset_ms', APP)
        self.assertIn('pairing_clock_unchanged', APP)
        self.assertIn('raw_utc_keys_unchanged', APP)
        self.assertIn("report.android_load.utc_wall_clock_fallback_offset_applied", APP)

    def test_phase194_noise_remains_separate(self):
        self.assertIn("phase194_source_utc_fallback_imu_noise", APP)
        self.assertIn("phase197_source_utc_fallback_imu_offset", APP)
        self.assertIn("native_utc_fallback_imu_noise::apply", APP)
        self.assertIn("native_utc_fallback_imu_offset::select", APP)


if __name__ == "__main__":
    unittest.main()
