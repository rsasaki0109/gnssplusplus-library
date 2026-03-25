#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <libgnss++/algorithms/spp.hpp>
#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/observation.hpp>
#include <libgnss++/core/processor.hpp>
#include <libgnss++/core/solution.hpp>
#include <libgnss++/io/rinex.hpp>

#define private public
#include <libgnss++/algorithms/rtk.hpp>
#undef private

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

using namespace libgnss;

namespace {

std::string sourcePath(const std::string& relative_path) {
    return std::string(GNSSPP_SOURCE_DIR) + "/" + relative_path;
}

class RTKLegacyCompatibilityTest : public ::testing::Test {
protected:
    void SetUp() override {
        ASSERT_TRUE(rover_reader_.open(sourcePath("data/rover_kinematic.obs")));
        ASSERT_TRUE(rover_reader_.readHeader(rover_header_));
        ASSERT_TRUE(base_reader_.open(sourcePath("data/base_kinematic.obs")));
        ASSERT_TRUE(base_reader_.readHeader(base_header_));
        ASSERT_TRUE(nav_reader_.open(sourcePath("data/navigation_kinematic.nav")));
        ASSERT_TRUE(nav_reader_.readNavigationData(nav_data_));
        ASSERT_GT(rover_header_.approximate_position.norm(), 1e6);
        ASSERT_GT(base_header_.approximate_position.norm(), 1e6);
        ASSERT_TRUE(readAlignedEpoch(rover_obs_, base_obs_));

        rover_obs_.receiver_position = rover_header_.approximate_position;

        config_.position_mode = RTKProcessor::RTKConfig::PositionMode::KINEMATIC;
        config_.ar_mode = RTKProcessor::RTKConfig::AmbiguityResolutionMode::CONTINUOUS;
        config_.min_satellites_for_ar = 5;
        config_.ratio_threshold = 3.0;
        processor_.setRTKConfig(config_);
        processor_.setBasePosition(base_header_.approximate_position);
    }

    bool readAlignedEpoch(ObservationData& rover_obs, ObservationData& base_obs) {
        bool rover_ok = rover_reader_.readObservationEpoch(rover_obs);
        bool base_ok = base_reader_.readObservationEpoch(base_obs);
        while (rover_ok && base_ok) {
            const double time_diff =
                (rover_obs.time.week - base_obs.time.week) * 604800.0 +
                (rover_obs.time.tow - base_obs.time.tow);
            if (std::abs(time_diff) <= 0.5) {
                return true;
            }
            if (time_diff < 0.0) {
                rover_ok = rover_reader_.readObservationEpoch(rover_obs);
            } else {
                base_ok = base_reader_.readObservationEpoch(base_obs);
            }
        }
        return false;
    }

    RTKProcessor processor_;
    RTKProcessor::RTKConfig config_;
    io::RINEXReader rover_reader_;
    io::RINEXReader base_reader_;
    io::RINEXReader nav_reader_;
    io::RINEXReader::RINEXHeader rover_header_;
    io::RINEXReader::RINEXHeader base_header_;
    NavigationData nav_data_;
    ObservationData rover_obs_;
    ObservationData base_obs_;
};

TEST_F(RTKLegacyCompatibilityTest, FormsDoubleDifferencesAndLinearizedHelpersStayConsistent) {
    const auto measurements = processor_.formDoubleDifferences(rover_obs_, base_obs_, nav_data_);

    ASSERT_GE(measurements.size(), 4u);
    EXPECT_TRUE(processor_.filter_initialized_);
    EXPECT_FALSE(processor_.current_sat_data_.empty());

    for (const auto& measurement : measurements) {
        EXPECT_TRUE(measurement.valid);
        EXPECT_TRUE(std::isfinite(measurement.pseudorange_dd));
        EXPECT_TRUE(std::isfinite(measurement.carrier_phase_dd));
        EXPECT_TRUE(std::isfinite(measurement.geometric_range));
        EXPECT_TRUE(measurement.unit_vector.array().isFinite().all());
        EXPECT_GT(measurement.variance, 0.0);
    }

    const Vector3d baseline = processor_.calculateBaseline();
    EXPECT_TRUE(baseline.array().isFinite().all());

    const VectorXd residuals = processor_.calculateResiduals(measurements, baseline);
    ASSERT_EQ(residuals.size(), static_cast<int>(measurements.size()));
    EXPECT_TRUE(residuals.array().isFinite().all());

    const MatrixXd H =
        processor_.formMeasurementMatrix(measurements, nav_data_, rover_obs_.time);
    ASSERT_EQ(H.rows(), static_cast<int>(measurements.size()));
    ASSERT_EQ(H.cols(), 3);
    EXPECT_TRUE(H.array().isFinite().all());

    const MatrixXd W = processor_.calculateMeasurementWeights(measurements);
    ASSERT_EQ(W.rows(), static_cast<int>(measurements.size()));
    ASSERT_EQ(W.cols(), static_cast<int>(measurements.size()));
    for (int i = 0; i < W.rows(); ++i) {
        EXPECT_GT(W(i, i), 0.0);
        for (int j = 0; j < W.cols(); ++j) {
            if (i == j) {
                continue;
            }
            EXPECT_DOUBLE_EQ(W(i, j), 0.0);
        }
    }

    EXPECT_TRUE(processor_.hasSufficientSatellites(measurements));
}

TEST(RTKLegacyCompatibilityStandaloneTest, LambdaCompatibilityProducesValidatedIntegerFix) {
    RTKProcessor processor;
    VectorXd float_ambiguities(3);
    float_ambiguities << 1.02, -2.97, 5.01;
    MatrixXd covariance = MatrixXd::Identity(3, 3) * 0.01;

    const auto result = processor.solveLAMBDA(float_ambiguities, covariance);

    ASSERT_TRUE(result.success);
    ASSERT_EQ(result.fixed_ambiguities.size(), float_ambiguities.size());
    EXPECT_GT(result.ratio, 0.0);
    for (int i = 0; i < result.fixed_ambiguities.size(); ++i) {
        EXPECT_NEAR(result.fixed_ambiguities(i),
                    std::round(result.fixed_ambiguities(i)),
                    1e-9);
    }

    EXPECT_TRUE(processor.validateAmbiguityResolution(result.fixed_ambiguities,
                                                      float_ambiguities,
                                                      covariance,
                                                      3.5));
    EXPECT_FALSE(processor.validateAmbiguityResolution(result.fixed_ambiguities,
                                                       float_ambiguities,
                                                       covariance,
                                                       0.5));
}

TEST_F(RTKLegacyCompatibilityTest, AppliesFixedAmbiguitiesAndPublishesFixedBaseline) {
    const auto measurements = processor_.formDoubleDifferences(rover_obs_, base_obs_, nav_data_);

    ASSERT_GE(measurements.size(), 4u);
    ASSERT_TRUE(processor_.filter_initialized_);

    std::vector<SatelliteId> satellites;
    satellites.reserve(processor_.current_sat_data_.size());
    for (const auto& [satellite, sat_data] : processor_.current_sat_data_) {
        if (sat_data.has_l1 || sat_data.has_l2) {
            satellites.push_back(satellite);
        }
    }
    std::sort(satellites.begin(), satellites.end());

    int n1_count = 0;
    int n2_count = 0;
    for (const auto& satellite : satellites) {
        if (processor_.filter_state_.n1_indices.count(satellite) > 0) {
            ++n1_count;
        }
        if (processor_.filter_state_.n2_indices.count(satellite) > 0) {
            ++n2_count;
        }
    }

    ASSERT_GT(n1_count, 0);
    ASSERT_GT(n2_count, 0);

    VectorXd fixed_n1(n1_count);
    VectorXd fixed_n2(n2_count);
    for (int i = 0; i < n1_count; ++i) {
        fixed_n1(i) = 100.0 + i;
    }
    for (int i = 0; i < n2_count; ++i) {
        fixed_n2(i) = 200.0 + i;
    }

    ASSERT_TRUE(processor_.applyFixedAmbiguities(fixed_n1, fixed_n2, processor_.current_sat_data_));

    int n1_index = 0;
    int n2_index = 0;
    for (const auto& satellite : satellites) {
        const auto n1_it = processor_.filter_state_.n1_indices.find(satellite);
        if (n1_it != processor_.filter_state_.n1_indices.end()) {
            EXPECT_DOUBLE_EQ(processor_.filter_state_.state(n1_it->second), fixed_n1(n1_index));
            ++n1_index;
        }
        const auto n2_it = processor_.filter_state_.n2_indices.find(satellite);
        if (n2_it != processor_.filter_state_.n2_indices.end()) {
            EXPECT_DOUBLE_EQ(processor_.filter_state_.state(n2_it->second), fixed_n2(n2_index));
            ++n2_index;
        }
    }

    processor_.fixed_baseline_ = Vector3d(1.0, 2.0, 3.0);
    processor_.has_fixed_solution_ = true;
    processor_.solvePositionWithAmbiguities(processor_.current_sat_data_);

    EXPECT_TRUE(processor_.filter_state_.state.head<3>().isApprox(processor_.fixed_baseline_, 1e-12));
}

}  // namespace
