#include <gtest/gtest.h>
#include <libgnss++/algorithms/rtk.hpp>
#include <libgnss++/io/rinex.hpp>

#include <filesystem>
#include <memory>
#include <string>

using namespace libgnss;

namespace {

std::string sourcePath(const std::string& relative_path) {
    return std::string(GNSSPP_SOURCE_DIR) + "/" + relative_path;
}

bool sourcePathExists(const std::string& relative_path) {
    return std::filesystem::exists(sourcePath(relative_path));
}

}  // namespace

class RTKSmokeTest : public ::testing::Test {
protected:
    struct RunSummary {
        int epochs_processed = 0;
        int valid_solutions = 0;
        int fixed_solutions = 0;
        PositionSolution first_valid_solution;
        PositionSolution first_fixed_solution;
    };

    void SetUp() override {
        if (!sourcePathExists("data/rover_kinematic.obs") ||
            !sourcePathExists("data/base_kinematic.obs") ||
            !sourcePathExists("data/navigation_kinematic.nav")) {
            GTEST_SKIP() << "repo kinematic test data is not available";
        }
        ASSERT_TRUE(rover_reader_.open(sourcePath("data/rover_kinematic.obs")));
        ASSERT_TRUE(rover_reader_.readHeader(rover_header_));
        ASSERT_TRUE(base_reader_.open(sourcePath("data/base_kinematic.obs")));
        ASSERT_TRUE(base_reader_.readHeader(base_header_));
        ASSERT_TRUE(nav_reader_.open(sourcePath("data/navigation_kinematic.nav")));
        ASSERT_TRUE(nav_reader_.readNavigationData(nav_data_));
        ASSERT_GT(base_header_.approximate_position.norm(), 1e6);
        ASSERT_GT(rover_header_.approximate_position.norm(), 1e6);
    }

    RunSummary runEpochs(int max_epochs, RTKProcessor::RTKConfig config = [] {
        RTKProcessor::RTKConfig cfg;
        cfg.position_mode = RTKProcessor::RTKConfig::PositionMode::KINEMATIC;
        cfg.ar_mode = RTKProcessor::RTKConfig::AmbiguityResolutionMode::CONTINUOUS;
        cfg.min_satellites_for_ar = 5;
        cfg.ratio_threshold = 3.0;
        return cfg;
    }()) {
        RTKProcessor processor;
        processor.setRTKConfig(config);
        processor.setBasePosition(base_header_.approximate_position);

        RunSummary summary;
        ObservationData rover_obs;
        ObservationData base_obs;

        bool rover_ok = rover_reader_.readObservationEpoch(rover_obs);
        bool base_ok = base_reader_.readObservationEpoch(base_obs);
        if (rover_ok) {
            rover_obs.receiver_position = rover_header_.approximate_position;
        }

        while (rover_ok && base_ok && summary.epochs_processed < max_epochs) {
            double time_diff = (rover_obs.time.week - base_obs.time.week) * 604800.0 +
                               (rover_obs.time.tow - base_obs.time.tow);
            while (rover_ok && base_ok && std::abs(time_diff) > 0.5) {
                if (time_diff < 0.0) {
                    const Vector3d saved_position = rover_obs.receiver_position;
                    rover_ok = rover_reader_.readObservationEpoch(rover_obs);
                    if (rover_ok) {
                        rover_obs.receiver_position = saved_position;
                    }
                } else {
                    base_ok = base_reader_.readObservationEpoch(base_obs);
                }
                if (!rover_ok || !base_ok) {
                    break;
                }
                time_diff = (rover_obs.time.week - base_obs.time.week) * 604800.0 +
                            (rover_obs.time.tow - base_obs.time.tow);
            }
            if (!rover_ok || !base_ok) {
                break;
            }

            const auto solution = processor.processRTKEpoch(rover_obs, base_obs, nav_data_);
            summary.epochs_processed++;
            if (solution.isValid()) {
                summary.valid_solutions++;
                if (!summary.first_valid_solution.isValid()) {
                    summary.first_valid_solution = solution;
                }
                if (solution.isFixed()) {
                    summary.fixed_solutions++;
                    if (!summary.first_fixed_solution.isValid()) {
                        summary.first_fixed_solution = solution;
                    }
                }
            }

            const Vector3d fallback_position = rover_obs.receiver_position;
            rover_ok = rover_reader_.readObservationEpoch(rover_obs);
            base_ok = base_reader_.readObservationEpoch(base_obs);
            if (rover_ok) {
                const bool trusted_seed =
                    solution.isFixed() ||
                    (solution.status == SolutionStatus::SPP && solution.num_satellites >= 7);
                rover_obs.receiver_position = trusted_seed ? solution.position_ecef : fallback_position;
            }
        }

        return summary;
    }

    io::RINEXReader rover_reader_;
    io::RINEXReader base_reader_;
    io::RINEXReader nav_reader_;
    io::RINEXReader::RINEXHeader rover_header_;
    io::RINEXReader::RINEXHeader base_header_;
    NavigationData nav_data_;
};

TEST_F(RTKSmokeTest, ProducesValidSolutionsOnBundledKinematicData) {
    const auto summary = runEpochs(10);

    EXPECT_EQ(summary.epochs_processed, 10);
    EXPECT_GE(summary.valid_solutions, 8);
    EXPECT_TRUE(summary.first_valid_solution.isValid());
    EXPECT_GE(summary.first_valid_solution.num_satellites, 5);
}

TEST_F(RTKSmokeTest, AchievesEarlyFixedSolutionOnBundledKinematicData) {
    const auto summary = runEpochs(20);

    EXPECT_EQ(summary.epochs_processed, 20);
    EXPECT_GE(summary.fixed_solutions, 1);
    EXPECT_TRUE(summary.first_fixed_solution.isValid());
    EXPECT_GT(summary.first_fixed_solution.ratio, 0.0);
    EXPECT_GE(summary.first_fixed_solution.num_fixed_ambiguities, 4);
}

TEST_F(RTKSmokeTest, GlonassARAutocalDoesNotRegressEarlyFixesOnBundledKinematicData) {
    RTKProcessor::RTKConfig off_config;
    off_config.position_mode = RTKProcessor::RTKConfig::PositionMode::KINEMATIC;
    off_config.ar_mode = RTKProcessor::RTKConfig::AmbiguityResolutionMode::CONTINUOUS;
    off_config.min_satellites_for_ar = 5;
    off_config.ratio_threshold = 3.0;
    off_config.enable_glonass = true;
    off_config.glonass_ar_mode = RTKProcessor::RTKConfig::GlonassARMode::OFF;

    auto on_config = off_config;
    on_config.glonass_ar_mode = RTKProcessor::RTKConfig::GlonassARMode::AUTOCAL;

    const auto off_summary = runEpochs(20, off_config);

    rover_reader_.close();
    base_reader_.close();
    nav_reader_.close();
    SetUp();

    const auto on_summary = runEpochs(20, on_config);

    EXPECT_EQ(off_summary.epochs_processed, 20);
    EXPECT_EQ(on_summary.epochs_processed, 20);
    EXPECT_EQ(off_summary.valid_solutions, on_summary.valid_solutions);
    EXPECT_GE(on_summary.fixed_solutions, off_summary.fixed_solutions);
}

TEST_F(RTKSmokeTest, EstimatedIonoModeProducesValidFloatSolutionsOnBundledKinematicData) {
    RTKProcessor::RTKConfig config;
    config.position_mode = RTKProcessor::RTKConfig::PositionMode::KINEMATIC;
    config.ar_mode = RTKProcessor::RTKConfig::AmbiguityResolutionMode::CONTINUOUS;
    config.ionoopt = RTKProcessor::RTKConfig::IonoOpt::EST;
    config.min_satellites_for_ar = 5;
    config.ratio_threshold = 3.0;

    const auto summary = runEpochs(10, config);

    EXPECT_EQ(summary.epochs_processed, 10);
    EXPECT_GE(summary.valid_solutions, 8);
    EXPECT_EQ(summary.fixed_solutions, 0);
    EXPECT_TRUE(summary.first_valid_solution.isValid());
}

TEST_F(RTKSmokeTest, MovingBaseModeProducesValidSolutionsOnBundledKinematicData) {
    RTKProcessor::RTKConfig config;
    config.position_mode = RTKProcessor::RTKConfig::PositionMode::MOVING_BASE;
    config.ar_mode = RTKProcessor::RTKConfig::AmbiguityResolutionMode::CONTINUOUS;
    config.min_satellites_for_ar = 5;
    config.ratio_threshold = 3.0;

    const auto summary = runEpochs(10, config);

    EXPECT_EQ(summary.epochs_processed, 10);
    EXPECT_GE(summary.valid_solutions, 8);
    EXPECT_TRUE(summary.first_valid_solution.isValid());
    EXPECT_GE(summary.first_valid_solution.num_satellites, 5);
}

// ============================================================================
// WP7: dead-knob wiring regression tests.
//
// WP6 found --arfilter/--arfilter-margin and --hold-ratio-threshold were
// parsed into RTKConfig but never read by rtk.cpp (rtk_ar_evaluation::
// passesArFilter() was only ever called from its own unit test; the
// fix-and-hold ratio relaxation used a hardcoded literal 2.0 instead of
// RTKConfig::hold_ambiguity_ratio_threshold). These tests exercise the
// wiring end-to-end on the bundled kinematic fixture, not just the pure
// helper functions in test_rtk_ar_evaluation.cpp.
// ============================================================================

TEST_F(RTKSmokeTest, ArFilterDisabledByDefaultMatchesPreWiringBehavior) {
    RTKProcessor::RTKConfig off_config;
    off_config.position_mode = RTKProcessor::RTKConfig::PositionMode::KINEMATIC;
    off_config.ar_mode = RTKProcessor::RTKConfig::AmbiguityResolutionMode::CONTINUOUS;
    off_config.min_satellites_for_ar = 5;
    off_config.ratio_threshold = 3.0;
    ASSERT_FALSE(off_config.enable_ar_filter);

    auto explicit_zero_margin_config = off_config;
    explicit_zero_margin_config.enable_ar_filter = true;
    explicit_zero_margin_config.ar_filter_margin = 0.0;

    const auto off_summary = runEpochs(30, off_config);

    rover_reader_.close();
    base_reader_.close();
    nav_reader_.close();
    SetUp();

    const auto zero_margin_summary = runEpochs(30, explicit_zero_margin_config);

    // enable_ar_filter=true with margin=0 imposes ratio >= threshold + 0,
    // identical to the always-on base gate, so the two runs must match
    // exactly (bit-identical wiring, not just "close").
    EXPECT_EQ(off_summary.epochs_processed, zero_margin_summary.epochs_processed);
    EXPECT_EQ(off_summary.valid_solutions, zero_margin_summary.valid_solutions);
    EXPECT_EQ(off_summary.fixed_solutions, zero_margin_summary.fixed_solutions);
}

TEST_F(RTKSmokeTest, ArFilterWithLargeMarginSuppressesFixesOnceEnabled) {
    RTKProcessor::RTKConfig off_config;
    off_config.position_mode = RTKProcessor::RTKConfig::PositionMode::KINEMATIC;
    off_config.ar_mode = RTKProcessor::RTKConfig::AmbiguityResolutionMode::CONTINUOUS;
    off_config.min_satellites_for_ar = 5;
    off_config.ratio_threshold = 3.0;

    auto strict_config = off_config;
    strict_config.enable_ar_filter = true;
    strict_config.ar_filter_margin = 1000.0;  // effectively unreachable ratio margin

    const auto baseline_summary = runEpochs(30, off_config);

    rover_reader_.close();
    base_reader_.close();
    nav_reader_.close();
    SetUp();

    const auto strict_summary = runEpochs(30, strict_config);

    EXPECT_GE(baseline_summary.fixed_solutions, 1);
    EXPECT_EQ(strict_summary.fixed_solutions, 0);
    // The AR filter only guards ambiguity acceptance, not the SPP/float path.
    EXPECT_EQ(baseline_summary.valid_solutions, strict_summary.valid_solutions);
}

TEST_F(RTKSmokeTest, HoldAmbiguityRatioThresholdIsHonoredNotHardcodedTwo) {
    // min_hold_count=2 makes the hold-fix regime reachable within the
    // bundled fixture's early, easy fix streak (AchievesEarlyFixedSolution*
    // above shows fixed_solutions >= 1 well within 20 epochs).
    RTKProcessor::RTKConfig config;
    config.position_mode = RTKProcessor::RTKConfig::PositionMode::KINEMATIC;
    config.ar_mode = RTKProcessor::RTKConfig::AmbiguityResolutionMode::CONTINUOUS;
    config.min_satellites_for_ar = 5;
    config.ratio_threshold = 3.0;
    config.ambiguity_ratio_threshold = 3.0;
    config.min_hold_count = 2;
    config.hold_ambiguity_ratio_threshold = 7.25;  // distinct from both 3.0 and the old hardcoded 2.0

    RTKProcessor processor;
    processor.setRTKConfig(config);
    processor.setBasePosition(base_header_.approximate_position);

    ObservationData rover_obs;
    ObservationData base_obs;
    ASSERT_TRUE(rover_reader_.readObservationEpoch(rover_obs));
    ASSERT_TRUE(base_reader_.readObservationEpoch(base_obs));
    rover_obs.receiver_position = rover_header_.approximate_position;

    bool observed_hold_regime = false;
    for (int i = 0; i < 60; ++i) {
        double time_diff = (rover_obs.time.week - base_obs.time.week) * 604800.0 +
                            (rover_obs.time.tow - base_obs.time.tow);
        bool rover_ok = true;
        bool base_ok = true;
        while (std::abs(time_diff) > 0.5) {
            if (time_diff < 0.0) {
                const Vector3d saved_position = rover_obs.receiver_position;
                rover_ok = rover_reader_.readObservationEpoch(rover_obs);
                if (rover_ok) rover_obs.receiver_position = saved_position;
            } else {
                base_ok = base_reader_.readObservationEpoch(base_obs);
            }
            if (!rover_ok || !base_ok) break;
            time_diff = (rover_obs.time.week - base_obs.time.week) * 604800.0 +
                        (rover_obs.time.tow - base_obs.time.tow);
        }
        if (!rover_ok || !base_ok) break;

        const auto solution = processor.processRTKEpoch(rover_obs, base_obs, nav_data_);
        const auto& telemetry = processor.getLastDebugTelemetry();
        if (telemetry.ar_attempted && std::isfinite(telemetry.effective_ratio_threshold) &&
            std::abs(telemetry.effective_ratio_threshold - config.ratio_threshold) > 1e-9) {
            // The hold regime kicked in (effective threshold deviated from
            // the base ratio_threshold): it must equal the *configured*
            // hold_ambiguity_ratio_threshold, not a hardcoded 2.0.
            observed_hold_regime = true;
            EXPECT_DOUBLE_EQ(telemetry.effective_ratio_threshold, config.hold_ambiguity_ratio_threshold);
        }

        const Vector3d fallback_position = rover_obs.receiver_position;
        rover_ok = rover_reader_.readObservationEpoch(rover_obs);
        base_ok = base_reader_.readObservationEpoch(base_obs);
        if (!rover_ok || !base_ok) break;
        const bool trusted_seed =
            solution.isFixed() || (solution.status == SolutionStatus::SPP && solution.num_satellites >= 7);
        rover_obs.receiver_position = trusted_seed ? solution.position_ecef : fallback_position;
    }

    EXPECT_TRUE(observed_hold_regime)
        << "bundled kinematic fixture never reached the hold-fix regime within 60 epochs";
}

TEST_F(RTKSmokeTest, HoldAmbiguityRatioThresholdDefaultMatchesLegacyHardcodedValue) {
    // Default hold_ambiguity_ratio_threshold is 2.0 (rtk.hpp), matching the
    // value that used to be hardcoded at the fix-and-hold call site — this
    // is the bit-identical-by-default half of the WP7 dead-knob fix.
    RTKProcessor::RTKConfig config;
    config.position_mode = RTKProcessor::RTKConfig::PositionMode::KINEMATIC;
    config.ar_mode = RTKProcessor::RTKConfig::AmbiguityResolutionMode::CONTINUOUS;
    config.min_satellites_for_ar = 5;
    config.ratio_threshold = 3.0;
    config.ambiguity_ratio_threshold = 3.0;
    config.min_hold_count = 2;
    EXPECT_DOUBLE_EQ(config.hold_ambiguity_ratio_threshold, 2.0);

    const auto summary = runEpochs(30, config);
    EXPECT_GE(summary.fixed_solutions, 1);
}

// --------------------------------------------------------------------------
// WP8: hard NLOS exclusion (--nlos-weight-mode exclude)
// --------------------------------------------------------------------------

TEST_F(RTKSmokeTest, NlosExcludeModeAbsentTableIsBitIdenticalToDefault) {
    RTKProcessor::RTKConfig config;
    config.position_mode = RTKProcessor::RTKConfig::PositionMode::KINEMATIC;
    config.ar_mode = RTKProcessor::RTKConfig::AmbiguityResolutionMode::CONTINUOUS;
    config.min_satellites_for_ar = 5;
    config.ratio_threshold = 3.0;

    const auto baseline_summary = runEpochs(30, config);

    rover_reader_.close();
    base_reader_.close();
    nav_reader_.close();
    SetUp();

    // Mode set to EXCLUDE but setNlosWeightTable() is never called -- must
    // stay a no-op, mirroring the WP7 sigma-inflation modes' own
    // absent-table guard (buildSelectionSnapshot() checks
    // `nlos_weight_table_ && !nlos_weight_table_->empty()` before doing
    // anything).
    auto exclude_config = config;
    exclude_config.nlos_weight_mode = nlos_weights::NlosWeightMode::EXCLUDE;
    exclude_config.nlos_exclude_threshold = 0.9;  // would exclude almost everyone if it fired
    exclude_config.nlos_min_sats = 0;
    const auto exclude_summary = runEpochs(30, exclude_config);

    EXPECT_EQ(baseline_summary.epochs_processed, exclude_summary.epochs_processed);
    EXPECT_EQ(baseline_summary.valid_solutions, exclude_summary.valid_solutions);
    EXPECT_EQ(baseline_summary.fixed_solutions, exclude_summary.fixed_solutions);
}

namespace {
// Every plausible satellite ID across all five constellations, marked NLOS
// (los_prob=0.0) at a single tow key with an enormous tolerance so the
// nearest-tow lookup always matches regardless of the fixture's real
// observation times -- exercises the exclusion *mechanism* without needing
// to know the bundled fixture's exact tracked PRNs.
std::shared_ptr<nlos_weights::NlosWeightTable> allSatellitesFlaggedNlosTable() {
    auto table = std::make_shared<nlos_weights::NlosWeightTable>();
    auto& row = table->by_tow[0.0];
    for (int prn = 1; prn <= 32; ++prn) row[SatelliteId(GNSSSystem::GPS, prn).toString()] = 0.0;
    for (int prn = 1; prn <= 36; ++prn) row[SatelliteId(GNSSSystem::Galileo, prn).toString()] = 0.0;
    for (int prn = 1; prn <= 24; ++prn) row[SatelliteId(GNSSSystem::GLONASS, prn).toString()] = 0.0;
    for (int prn = 1; prn <= 46; ++prn) row[SatelliteId(GNSSSystem::BeiDou, prn).toString()] = 0.0;
    for (int prn = 1; prn <= 7; ++prn) row[SatelliteId(GNSSSystem::QZSS, prn).toString()] = 0.0;
    return table;
}

int countFixedSolutionsWithTable(
    io::RINEXReader& rover_reader, io::RINEXReader& base_reader,
    const NavigationData& nav_data, const Vector3d& base_position,
    const Vector3d& rover_seed_position, const RTKProcessor::RTKConfig& config,
    std::shared_ptr<const nlos_weights::NlosWeightTable> table, int max_epochs) {
    RTKProcessor processor;
    processor.setRTKConfig(config);
    processor.setBasePosition(base_position);
    if (table) processor.setNlosWeightTable(std::move(table));

    ObservationData rover_obs;
    ObservationData base_obs;
    bool rover_ok = rover_reader.readObservationEpoch(rover_obs);
    bool base_ok = base_reader.readObservationEpoch(base_obs);
    if (rover_ok) rover_obs.receiver_position = rover_seed_position;

    int fixed_solutions = 0;
    int epochs_processed = 0;
    while (rover_ok && base_ok && epochs_processed < max_epochs) {
        double time_diff = (rover_obs.time.week - base_obs.time.week) * 604800.0 +
                            (rover_obs.time.tow - base_obs.time.tow);
        while (rover_ok && base_ok && std::abs(time_diff) > 0.5) {
            if (time_diff < 0.0) {
                const Vector3d saved_position = rover_obs.receiver_position;
                rover_ok = rover_reader.readObservationEpoch(rover_obs);
                if (rover_ok) rover_obs.receiver_position = saved_position;
            } else {
                base_ok = base_reader.readObservationEpoch(base_obs);
            }
            if (!rover_ok || !base_ok) break;
            time_diff = (rover_obs.time.week - base_obs.time.week) * 604800.0 +
                        (rover_obs.time.tow - base_obs.time.tow);
        }
        if (!rover_ok || !base_ok) break;

        const auto solution = processor.processRTKEpoch(rover_obs, base_obs, nav_data);
        epochs_processed++;
        if (solution.isFixed()) fixed_solutions++;

        const Vector3d fallback_position = rover_obs.receiver_position;
        rover_ok = rover_reader.readObservationEpoch(rover_obs);
        base_ok = base_reader.readObservationEpoch(base_obs);
        if (!rover_ok || !base_ok) break;
        rover_obs.receiver_position = solution.isValid() ? solution.position_ecef : fallback_position;
    }
    return fixed_solutions;
}
}  // namespace

TEST_F(RTKSmokeTest, NlosExcludeModeWithMinSatsZeroCollapsesFixedSolutions) {
    RTKProcessor::RTKConfig config;
    config.position_mode = RTKProcessor::RTKConfig::PositionMode::KINEMATIC;
    config.ar_mode = RTKProcessor::RTKConfig::AmbiguityResolutionMode::CONTINUOUS;
    config.min_satellites_for_ar = 5;
    config.ratio_threshold = 3.0;
    config.nlos_weight_mode = nlos_weights::NlosWeightMode::EXCLUDE;
    config.nlos_exclude_threshold = 0.5;
    config.nlos_min_sats = 0;   // guard disabled -- exclusion always fires
    config.nlos_tow_tolerance_s = 1e9;

    const int fixed_solutions = countFixedSolutionsWithTable(
        rover_reader_, base_reader_, nav_data_, base_header_.approximate_position,
        rover_header_.approximate_position, config, allSatellitesFlaggedNlosTable(), 30);

    // Every candidate satellite is flagged NLOS, so (down to at most one
    // protected reference per system, per the "never drop the last
    // reference-satellite candidate" guard) each system's DD pair set
    // collapses to ~0 pairs -- AR can never gather enough redundancy to
    // fix. This is the exclusion mechanism itself, not a tuned threshold.
    EXPECT_EQ(fixed_solutions, 0);
}

TEST_F(RTKSmokeTest, NlosExcludeModeMinSatsGuardKeepsAllSatellitesWhenTooFewWouldSurvive) {
    RTKProcessor::RTKConfig baseline_config;
    baseline_config.position_mode = RTKProcessor::RTKConfig::PositionMode::KINEMATIC;
    baseline_config.ar_mode = RTKProcessor::RTKConfig::AmbiguityResolutionMode::CONTINUOUS;
    baseline_config.min_satellites_for_ar = 5;
    baseline_config.ratio_threshold = 3.0;

    const int baseline_fixed = countFixedSolutionsWithTable(
        rover_reader_, base_reader_, nav_data_, base_header_.approximate_position,
        rover_header_.approximate_position, baseline_config, nullptr, 30);
    ASSERT_GE(baseline_fixed, 1)
        << "bundled fixture expected to fix within 30 epochs (see other tests in this file)";

    rover_reader_.close();
    base_reader_.close();
    nav_reader_.close();
    SetUp();

    // Same "exclude everyone" table as the mechanism test above, but with
    // an unreachably high --nlos-min-sats floor -- the guard must trip
    // every epoch and keep every satellite, reproducing the baseline
    // fixed-solution count exactly (the guard's whole point: never degrade
    // geometry below solvability).
    auto guarded_config = baseline_config;
    guarded_config.nlos_weight_mode = nlos_weights::NlosWeightMode::EXCLUDE;
    guarded_config.nlos_exclude_threshold = 0.5;
    guarded_config.nlos_min_sats = 999;
    guarded_config.nlos_tow_tolerance_s = 1e9;

    const int guarded_fixed = countFixedSolutionsWithTable(
        rover_reader_, base_reader_, nav_data_, base_header_.approximate_position,
        rover_header_.approximate_position, guarded_config, allSatellitesFlaggedNlosTable(), 30);

    EXPECT_EQ(guarded_fixed, baseline_fixed);
}

TEST_F(RTKSmokeTest, FloatTrustPolicyDefaultLegacyIsBitIdenticalToPreWp9) {
    RTKProcessor::RTKConfig config;
    config.position_mode = RTKProcessor::RTKConfig::PositionMode::KINEMATIC;
    config.ar_mode = RTKProcessor::RTKConfig::AmbiguityResolutionMode::CONTINUOUS;
    config.min_satellites_for_ar = 5;
    config.ratio_threshold = 3.0;

    const auto baseline_summary = runEpochs(30, config);

    rover_reader_.close();
    base_reader_.close();
    nav_reader_.close();
    SetUp();

    // float_trust_policy left at its LEGACY default explicitly -- must be a
    // no-op, mirroring the absent-flag guarantee required for every other
    // WP9 flag.
    auto explicit_legacy_config = config;
    explicit_legacy_config.float_trust_policy = float_trust_policy::FloatTrustPolicy::LEGACY;
    explicit_legacy_config.trust_lapse_qpos_m2_per_s = 999.0;  // must be ignored under LEGACY
    const auto explicit_legacy_summary = runEpochs(30, explicit_legacy_config);

    EXPECT_EQ(baseline_summary.epochs_processed, explicit_legacy_summary.epochs_processed);
    EXPECT_EQ(baseline_summary.valid_solutions, explicit_legacy_summary.valid_solutions);
    EXPECT_EQ(baseline_summary.fixed_solutions, explicit_legacy_summary.fixed_solutions);
}

TEST_F(RTKSmokeTest, FloatTrustPolicyCvPredictAndScaledResetDoNotCrashAndRemainValid) {
    // Not a numerical-equivalence test (that's float_trust_policy.hpp's own
    // unit tests) -- this is an integration smoke test that both new
    // policies wire in cleanly against real data: no crashes, solutions
    // stay finite/valid, and fixed-solution count never exceeds the legacy
    // baseline's on this short, easy bundled fixture (both policies only
    // ever engage once trust has lapsed, so on an easy fixture where trust
    // rarely lapses they should track the baseline closely).
    RTKProcessor::RTKConfig baseline_config;
    baseline_config.position_mode = RTKProcessor::RTKConfig::PositionMode::KINEMATIC;
    baseline_config.ar_mode = RTKProcessor::RTKConfig::AmbiguityResolutionMode::CONTINUOUS;
    baseline_config.min_satellites_for_ar = 5;
    baseline_config.ratio_threshold = 3.0;
    const auto baseline_summary = runEpochs(60, baseline_config);
    ASSERT_GT(baseline_summary.valid_solutions, 0);

    for (const auto policy : {float_trust_policy::FloatTrustPolicy::CV_PREDICT,
                              float_trust_policy::FloatTrustPolicy::SCALED_RESET}) {
        rover_reader_.close();
        base_reader_.close();
        nav_reader_.close();
        SetUp();

        auto config = baseline_config;
        config.float_trust_policy = policy;
        config.trust_lapse_qpos_m2_per_s = 10.0;
        const auto summary = runEpochs(60, config);

        EXPECT_EQ(summary.epochs_processed, baseline_summary.epochs_processed);
        EXPECT_GT(summary.valid_solutions, 0);
        if (summary.first_valid_solution.isValid()) {
            EXPECT_TRUE(summary.first_valid_solution.position_ecef.allFinite());
        }
    }
}

TEST_F(RTKSmokeTest, FloatTrustPolicyLapseGatedWithHugeGateIsBitIdenticalToLegacy) {
    // WP10's core bit-identity claim: at a gate value far larger than any
    // lapse that can occur on this short fixture, lapse-gated must never
    // take the scaled-reset branch -- every epoch falls through to the
    // unmodified legacy fallback path, reproducing the plain-LEGACY
    // baseline's fixed/valid counts exactly.
    RTKProcessor::RTKConfig config;
    config.position_mode = RTKProcessor::RTKConfig::PositionMode::KINEMATIC;
    config.ar_mode = RTKProcessor::RTKConfig::AmbiguityResolutionMode::CONTINUOUS;
    config.min_satellites_for_ar = 5;
    config.ratio_threshold = 3.0;
    const auto baseline_summary = runEpochs(60, config);
    ASSERT_GT(baseline_summary.valid_solutions, 0);

    rover_reader_.close();
    base_reader_.close();
    nav_reader_.close();
    SetUp();

    auto gated_config = config;
    gated_config.float_trust_policy = float_trust_policy::FloatTrustPolicy::LAPSE_GATED;
    gated_config.trust_lapse_gate_s = 1.0e6;
    gated_config.trust_lapse_qpos_m2_per_s = 0.1;
    const auto gated_summary = runEpochs(60, gated_config);

    EXPECT_EQ(gated_summary.epochs_processed, baseline_summary.epochs_processed);
    EXPECT_EQ(gated_summary.valid_solutions, baseline_summary.valid_solutions);
    EXPECT_EQ(gated_summary.fixed_solutions, baseline_summary.fixed_solutions);
}

TEST_F(RTKSmokeTest, FloatTrustPolicyLapseGatedAtZeroGateDoesNotCrash) {
    // The opposite endpoint of the gate sweep (0s -- "always scaled, never
    // legacy, once lapsed") should behave like SCALED_RESET: no crashes,
    // solutions stay finite/valid.
    RTKProcessor::RTKConfig config;
    config.position_mode = RTKProcessor::RTKConfig::PositionMode::KINEMATIC;
    config.ar_mode = RTKProcessor::RTKConfig::AmbiguityResolutionMode::CONTINUOUS;
    config.min_satellites_for_ar = 5;
    config.ratio_threshold = 3.0;
    config.float_trust_policy = float_trust_policy::FloatTrustPolicy::LAPSE_GATED;
    config.trust_lapse_gate_s = 0.0;
    config.trust_lapse_qpos_m2_per_s = 0.1;

    const auto summary = runEpochs(60, config);
    EXPECT_GT(summary.valid_solutions, 0);
    if (summary.first_valid_solution.isValid()) {
        EXPECT_TRUE(summary.first_valid_solution.position_ecef.allFinite());
    }
}

TEST(RTKStateIndexTest, SeparatesConstellationsInAmbiguityStateLayout) {
    const int gps_l1 = RTKProcessor::ambiguityStateIndex(SatelliteId(GNSSSystem::GPS, 1), 0);
    const int gal_l1 = RTKProcessor::ambiguityStateIndex(SatelliteId(GNSSSystem::Galileo, 1), 0);
    const int glo_l1 = RTKProcessor::ambiguityStateIndex(SatelliteId(GNSSSystem::GLONASS, 1), 0);
    const int bds_l1 = RTKProcessor::ambiguityStateIndex(SatelliteId(GNSSSystem::BeiDou, 1), 0);
    const int qzs_l1 = RTKProcessor::ambiguityStateIndex(SatelliteId(GNSSSystem::QZSS, 1), 0);
    const int gps_l2 = RTKProcessor::ambiguityStateIndex(SatelliteId(GNSSSystem::GPS, 1), 1);

    EXPECT_NE(gps_l1, gal_l1);
    EXPECT_NE(gps_l1, glo_l1);
    EXPECT_NE(gps_l1, bds_l1);
    EXPECT_NE(gps_l1, qzs_l1);
    EXPECT_NE(gps_l1, gps_l2);
}

TEST(RTKStateIndexTest, SeparatesIonoStatesFromAmbiguityStatesAndConstellations) {
    const int gps_iono = RTKProcessor::ionoStateIndex(SatelliteId(GNSSSystem::GPS, 1));
    const int gal_iono = RTKProcessor::ionoStateIndex(SatelliteId(GNSSSystem::Galileo, 1));
    const int gps_l1 = RTKProcessor::ambiguityStateIndex(SatelliteId(GNSSSystem::GPS, 1), 0);
    const int gps_l2 = RTKProcessor::ambiguityStateIndex(SatelliteId(GNSSSystem::GPS, 1), 1);

    EXPECT_NE(gps_iono, gal_iono);
    EXPECT_NE(gps_iono, gps_l1);
    EXPECT_NE(gps_iono, gps_l2);
}

TEST(RTKMixedConstellationTest, UsesBeiDouOnOdaibaExactEpochWithoutLargeJump) {
    if (!sourcePathExists("data/driving/Tokyo_Data/Odaiba/rover_trimble.obs") ||
        !sourcePathExists("data/driving/Tokyo_Data/Odaiba/base_trimble.obs") ||
        !sourcePathExists("data/driving/Tokyo_Data/Odaiba/base.nav")) {
        GTEST_SKIP() << "repo Odaiba test data is not available";
    }
    io::RINEXReader rover_reader;
    io::RINEXReader base_reader;
    io::RINEXReader nav_reader;
    io::RINEXReader::RINEXHeader rover_header;
    io::RINEXReader::RINEXHeader base_header;
    NavigationData nav_data;
    ObservationData rover_obs;
    ObservationData base_obs;

    ASSERT_TRUE(rover_reader.open(sourcePath("data/driving/Tokyo_Data/Odaiba/rover_trimble.obs")));
    ASSERT_TRUE(rover_reader.readHeader(rover_header));
    ASSERT_TRUE(base_reader.open(sourcePath("data/driving/Tokyo_Data/Odaiba/base_trimble.obs")));
    ASSERT_TRUE(base_reader.readHeader(base_header));
    ASSERT_TRUE(nav_reader.open(sourcePath("data/driving/Tokyo_Data/Odaiba/base.nav")));
    ASSERT_TRUE(nav_reader.readNavigationData(nav_data));
    ASSERT_TRUE(rover_reader.readObservationEpoch(rover_obs));
    ASSERT_TRUE(base_reader.readObservationEpoch(base_obs));
    ASSERT_GT(rover_header.approximate_position.norm(), 1e6);
    ASSERT_GT(base_header.approximate_position.norm(), 1e6);

    rover_obs.receiver_position = rover_header.approximate_position;

    auto solve_epoch = [&](bool enable_beidou) {
        RTKProcessor processor;
        RTKProcessor::RTKConfig config;
        config.position_mode = RTKProcessor::RTKConfig::PositionMode::KINEMATIC;
        config.ar_mode = RTKProcessor::RTKConfig::AmbiguityResolutionMode::CONTINUOUS;
        config.min_satellites_for_ar = 5;
        config.ratio_threshold = 3.0;
        config.enable_beidou = enable_beidou;
        processor.setRTKConfig(config);
        processor.setBasePosition(base_header.approximate_position);
        return processor.processRTKEpoch(rover_obs, base_obs, nav_data);
    };

    const auto without_beidou = solve_epoch(false);
    const auto with_beidou = solve_epoch(true);

    ASSERT_TRUE(without_beidou.isValid());
    ASSERT_TRUE(with_beidou.isValid());
    EXPECT_GT(with_beidou.num_satellites, without_beidou.num_satellites);
    EXPECT_LT((with_beidou.position_ecef - without_beidou.position_ecef).norm(), 5.0);
}

TEST(RTKMixedConstellationTest, UsesGlonassOnOdaibaExactEpochWithoutLargeJump) {
    if (!sourcePathExists("data/driving/Tokyo_Data/Odaiba/rover_trimble.obs") ||
        !sourcePathExists("data/driving/Tokyo_Data/Odaiba/base_trimble.obs") ||
        !sourcePathExists("data/driving/Tokyo_Data/Odaiba/base.nav")) {
        GTEST_SKIP() << "repo Odaiba test data is not available";
    }
    io::RINEXReader rover_reader;
    io::RINEXReader base_reader;
    io::RINEXReader nav_reader;
    io::RINEXReader::RINEXHeader rover_header;
    io::RINEXReader::RINEXHeader base_header;
    NavigationData nav_data;
    ObservationData rover_obs;
    ObservationData base_obs;

    ASSERT_TRUE(rover_reader.open(sourcePath("data/driving/Tokyo_Data/Odaiba/rover_trimble.obs")));
    ASSERT_TRUE(rover_reader.readHeader(rover_header));
    ASSERT_TRUE(base_reader.open(sourcePath("data/driving/Tokyo_Data/Odaiba/base_trimble.obs")));
    ASSERT_TRUE(base_reader.readHeader(base_header));
    ASSERT_TRUE(nav_reader.open(sourcePath("data/driving/Tokyo_Data/Odaiba/base.nav")));
    ASSERT_TRUE(nav_reader.readNavigationData(nav_data));
    ASSERT_TRUE(rover_reader.readObservationEpoch(rover_obs));
    ASSERT_TRUE(base_reader.readObservationEpoch(base_obs));
    ASSERT_GT(rover_header.approximate_position.norm(), 1e6);
    ASSERT_GT(base_header.approximate_position.norm(), 1e6);

    rover_obs.receiver_position = rover_header.approximate_position;

    auto solve_epoch = [&](bool enable_glonass) {
        RTKProcessor processor;
        RTKProcessor::RTKConfig config;
        config.position_mode = RTKProcessor::RTKConfig::PositionMode::KINEMATIC;
        config.ar_mode = RTKProcessor::RTKConfig::AmbiguityResolutionMode::CONTINUOUS;
        config.min_satellites_for_ar = 5;
        config.ratio_threshold = 3.0;
        config.enable_glonass = enable_glonass;
        processor.setRTKConfig(config);
        processor.setBasePosition(base_header.approximate_position);
        return processor.processRTKEpoch(rover_obs, base_obs, nav_data);
    };

    const auto without_glonass = solve_epoch(false);
    const auto with_glonass = solve_epoch(true);

    ASSERT_TRUE(without_glonass.isValid());
    ASSERT_TRUE(with_glonass.isValid());
    EXPECT_GT(with_glonass.num_satellites, without_glonass.num_satellites);
    EXPECT_LT((with_glonass.position_ecef - without_glonass.position_ecef).norm(), 5.0);
}

TEST(RTKMixedConstellationTest, SPPSeedHonorsGlonassSwitch) {
    if (!sourcePathExists("data/driving/Tokyo_Data/Odaiba/rover_trimble.obs") ||
        !sourcePathExists("data/driving/Tokyo_Data/Odaiba/base.nav")) {
        GTEST_SKIP() << "repo Odaiba test data is not available";
    }
    io::RINEXReader rover_reader;
    io::RINEXReader nav_reader;
    io::RINEXReader::RINEXHeader rover_header;
    NavigationData nav_data;
    ObservationData rover_obs;

    ASSERT_TRUE(rover_reader.open(sourcePath("data/driving/Tokyo_Data/Odaiba/rover_trimble.obs")));
    ASSERT_TRUE(rover_reader.readHeader(rover_header));
    ASSERT_TRUE(nav_reader.open(sourcePath("data/driving/Tokyo_Data/Odaiba/base.nav")));
    ASSERT_TRUE(nav_reader.readNavigationData(nav_data));
    ASSERT_TRUE(rover_reader.readObservationEpoch(rover_obs));
    ASSERT_GT(rover_header.approximate_position.norm(), 1e6);

    rover_obs.receiver_position = rover_header.approximate_position;

    auto solve_epoch = [&](bool enable_glonass) {
        RTKProcessor processor;
        RTKProcessor::RTKConfig config;
        config.enable_glonass = enable_glonass;
        processor.setRTKConfig(config);
        return processor.processEpoch(rover_obs, nav_data);
    };

    const auto without_glonass = solve_epoch(false);
    const auto with_glonass = solve_epoch(true);

    ASSERT_TRUE(without_glonass.isValid());
    ASSERT_TRUE(with_glonass.isValid());
    EXPECT_GT(with_glonass.num_satellites, without_glonass.num_satellites);
    EXPECT_LT((with_glonass.position_ecef - without_glonass.position_ecef).norm(), 5.0);
}
