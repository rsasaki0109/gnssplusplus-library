#include <gtest/gtest.h>
#include <libgnss++/algorithms/rtk.hpp>
#include <libgnss++/io/rinex.hpp>

#include <string>

using namespace libgnss;

namespace {

std::string sourcePath(const std::string& relative_path) {
    return std::string(GNSSPP_SOURCE_DIR) + "/" + relative_path;
}

}  // namespace

class RTKSmokeTest : public ::testing::Test {
protected:
    struct RunSummary {
        int epochs_processed = 0;
        int valid_solutions = 0;
        int fixed_solutions = 0;
        PositionSolution first_valid_solution;
    };

    void SetUp() override {
        ASSERT_TRUE(rover_reader_.open(sourcePath("data/rover_kinematic.obs")));
        ASSERT_TRUE(rover_reader_.readHeader(rover_header_));
        ASSERT_TRUE(base_reader_.open(sourcePath("data/base_kinematic.obs")));
        ASSERT_TRUE(base_reader_.readHeader(base_header_));
        ASSERT_TRUE(nav_reader_.open(sourcePath("data/navigation_kinematic.nav")));
        ASSERT_TRUE(nav_reader_.readNavigationData(nav_data_));
        ASSERT_GT(base_header_.approximate_position.norm(), 1e6);
        ASSERT_GT(rover_header_.approximate_position.norm(), 1e6);
    }

    RunSummary runEpochs(int max_epochs) {
        RTKProcessor processor;
        RTKProcessor::RTKConfig config;
        config.position_mode = RTKProcessor::RTKConfig::PositionMode::KINEMATIC;
        config.ar_mode = RTKProcessor::RTKConfig::AmbiguityResolutionMode::CONTINUOUS;
        config.min_satellites_for_ar = 5;
        config.ratio_threshold = 3.0;
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
                }
            }

            const Vector3d fallback_position = rover_obs.receiver_position;
            rover_ok = rover_reader_.readObservationEpoch(rover_obs);
            base_ok = base_reader_.readObservationEpoch(base_obs);
            if (rover_ok) {
                rover_obs.receiver_position =
                    solution.isValid() ? solution.position_ecef : fallback_position;
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
}
