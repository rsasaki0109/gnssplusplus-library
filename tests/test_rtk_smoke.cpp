#include <gtest/gtest.h>
#include <libgnss++/algorithms/rtk.hpp>
#include <libgnss++/io/rinex.hpp>

#include <filesystem>
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
