"""Launch-free regression for Phase171 raw-P staging handoff admission."""

from __future__ import annotations

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
APP = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"
HELPER = ROOT / "include/libgnss++/algorithms/native_raw_p_ecef_doppler_staging.hpp"


class Phase178StagingHandoffTests(unittest.TestCase):
    def test_raw_p_staging_does_not_select_legacy_raw_d_handoff_guard(self) -> None:
        source = APP.read_text(encoding="utf-8")
        branch = source.index("} else if (phase171_imu_main) {")
        processor = source.index(
            "const libgnss::FGOProcessor gnss_first_processor(gnss_first_config);",
            branch,
        )
        staging = source[branch:processor]
        self.assertIn(
            "gnss_first_config.use_native_raw_p_no_doppler_graph =\n"
            "                !phase171_ecef_doppler;",
            staging,
        )
        self.assertIn(
            "gnss_first_config.use_native_raw_p_ecef_doppler_gnss_first =\n"
            "                phase171_ecef_doppler;",
            staging,
        )
        self.assertIn(
            "gnss_first_config.use_native_source_clock_c0d_gnss_first_meter_state_handoff =\n"
            "                false;",
            staging,
        )
        self.assertNotIn(
            "gnss_first_config.use_native_source_clock_c0d_gnss_first_meter_state_handoff =\n"
            "                true;",
            staging,
        )
        self.assertIn(
            "gnss_first_config.use_native_source_clock_c0d_raw_drift_d_initializer =\n"
            "                false;",
            staging,
        )

    def test_main_keeps_source_clock_handoff_for_optimized_imu_graph(self) -> None:
        source = APP.read_text(encoding="utf-8")
        main_start = source.index(
            "if (options.native_phase171_raw_p_no_doppler_imu_main) {"
        )
        staging_start = source.index(
            "std::vector<libgnss::raw_p_seed::RawPNoDopplerSeed> retained_seeds",
            main_start,
        )
        main_config = source[main_start:staging_start]
        self.assertIn(
            "config.use_native_source_clock_c0d_gnss_first_meter_state_handoff = true;",
            main_config,
        )
        self.assertIn(
            "config.use_native_source_clock_c0d_raw_drift_d_initializer = false;",
            main_config,
        )

    def test_staging_identity_uses_raw_keys_and_strict_time(self) -> None:
        source = APP.read_text(encoding="utf-8")
        helper = HELPER.read_text(encoding="utf-8")
        self.assertIn(
            "using RawEpochIdentity = std::pair<std::size_t, std::int64_t>;",
            helper,
        )
        self.assertIn(
            "std::map<RawEpochIdentity, std::size_t>", helper
        )
        self.assertIn(
            "source_clock_c0d::strictEpochTimeEqual(", helper
        )
        self.assertIn(
            "source_clock_c0d::validEpochTime(", helper
        )
        # A rounded-TOW bin is not an exact raw-row identity and can alias
        # two rows inside GNSSTime::operator=='s 1 us tolerance.
        self.assertNotIn("llround", helper)
        self.assertIn(
            "gnss_first_problem.undifferenced_doppler_factors =\n"
            "                std::move(staged_doppler);",
            source,
        )


if __name__ == "__main__":
    unittest.main()
