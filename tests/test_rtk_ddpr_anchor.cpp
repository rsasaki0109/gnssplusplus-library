#include <gtest/gtest.h>

#include <array>
#include <limits>
#include <vector>

#include <libgnss++/algorithms/rtk_ddpr_anchor.hpp>
#include <libgnss++/core/coordinates.hpp>

namespace {

using libgnss::rtk_ddpr_anchor::Config;
using libgnss::rtk_ddpr_anchor::Observation;
using libgnss::rtk_ddpr_anchor::Status;

std::vector<Observation> makeObservations(const Eigen::Vector3d& rover,
                                          const Eigen::Vector3d& base) {
    constexpr double radius = 26560000.0;
    const std::array<Eigen::Vector3d, 7> satellites{
        Eigen::Vector3d{radius, 0.0, 0.0},
        Eigen::Vector3d{0.0, radius, 0.0},
        Eigen::Vector3d{0.0, 0.0, radius},
        Eigen::Vector3d{-radius, 0.4 * radius, 0.2 * radius},
        Eigen::Vector3d{0.3 * radius, -radius, 0.5 * radius},
        Eigen::Vector3d{-0.2 * radius, -0.4 * radius, radius},
        Eigen::Vector3d{0.7 * radius, 0.6 * radius, -0.5 * radius},
    };
    std::vector<Observation> observations;
    for (std::size_t i = 1; i < satellites.size(); ++i) {
        Observation observation;
        observation.reference_satellite_rover_ecef = satellites[0];
        observation.target_satellite_rover_ecef = satellites[i];
        observation.reference_satellite_base_ecef = satellites[0];
        observation.target_satellite_base_ecef = satellites[i];
        observation.dd_pseudorange_m =
            (libgnss::geodist(satellites[0], rover) -
             libgnss::geodist(satellites[0], base)) -
            (libgnss::geodist(satellites[i], rover) -
             libgnss::geodist(satellites[i], base));
        observations.push_back(observation);
    }
    return observations;
}

TEST(RTKDdPrAnchorTest, RecoversPositionFromCodeDoubleDifferences) {
    const Eigen::Vector3d base{-3961800.0, 3349000.0, 3698300.0};
    const Eigen::Vector3d truth = base + Eigen::Vector3d{120.0, -80.0, 35.0};
    const auto result = libgnss::rtk_ddpr_anchor::solve(
        makeObservations(truth, base), base, truth + Eigen::Vector3d{20.0, -15.0, 10.0});
    ASSERT_TRUE(result.valid);
    EXPECT_EQ(result.status, Status::ACCEPTED);
    EXPECT_LT((result.position_ecef - truth).norm(), 1e-3);
    EXPECT_EQ(result.observations_used, 6u);
    EXPECT_TRUE(result.covariance_ecef.allFinite());
}

TEST(RTKDdPrAnchorTest, RejectsSingleGrossOutlierWithFde) {
    const Eigen::Vector3d base{-3961800.0, 3349000.0, 3698300.0};
    const Eigen::Vector3d truth = base + Eigen::Vector3d{80.0, 50.0, -20.0};
    auto observations = makeObservations(truth, base);
    observations[2].dd_pseudorange_m += 100.0;
    Config config;
    config.fde_threshold_m = 5.0;
    const auto result = libgnss::rtk_ddpr_anchor::solve(
        observations, base, truth + Eigen::Vector3d{10.0, 10.0, 10.0}, config);
    ASSERT_TRUE(result.valid);
    EXPECT_EQ(result.observations_rejected, 1u);
    EXPECT_LT((result.position_ecef - truth).norm(), 1e-2);
}

TEST(RTKDdPrAnchorTest, RejectsInsufficientAndNonFiniteInputs) {
    const Eigen::Vector3d base{1.0, 2.0, 3.0};
    auto result = libgnss::rtk_ddpr_anchor::solve({}, base, base);
    EXPECT_EQ(result.status, Status::INSUFFICIENT_OBSERVATIONS);
    Eigen::Vector3d invalid = base;
    invalid.x() = std::numeric_limits<double>::quiet_NaN();
    result = libgnss::rtk_ddpr_anchor::solve(std::vector<Observation>(4), base, invalid);
    EXPECT_EQ(result.status, Status::INVALID_INPUT);
}

TEST(RTKDdPrAnchorTest, RejectsSingularGeometry) {
    const Eigen::Vector3d base{6378137.0, 0.0, 0.0};
    std::vector<Observation> observations(4);
    for (auto& observation : observations) {
        observation.reference_satellite_rover_ecef = {26560000.0, 0.0, 0.0};
        observation.target_satellite_rover_ecef = {27000000.0, 0.0, 0.0};
        observation.reference_satellite_base_ecef = observation.reference_satellite_rover_ecef;
        observation.target_satellite_base_ecef = observation.target_satellite_rover_ecef;
    }
    const auto result = libgnss::rtk_ddpr_anchor::solve(observations, base, base);
    EXPECT_EQ(result.status, Status::SINGULAR_GEOMETRY);
}

}  // namespace
