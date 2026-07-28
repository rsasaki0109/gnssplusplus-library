#include <gtest/gtest.h>

#include <libgnss++/algorithms/rtk_float_stabilizer.hpp>

#include <deque>

namespace libgnss::rtk_float_stabilizer {
namespace {

TEST(RTKFloatStabilizerTest, PredictsFromRecentFixedTrajectory) {
    std::deque<FixedAnchor> anchors;
    const Eigen::Vector3d origin(1.0e6, 2.0e6, 3.0e6);
    const Eigen::Vector3d velocity(4.0, -2.0, 0.5);
    for (int i = 0; i <= 100; ++i) {
        const double time_s = 1000.0 + 0.2 * i;
        anchors.push_back(
            {time_s, origin + velocity * (time_s - 1000.0)});
    }

    const Eigen::Vector3d float_position =
        origin + Eigen::Vector3d(100.0, 100.0, 100.0);
    const auto predicted = predict(
        anchors, 1025.0, float_position, 25.0);

    ASSERT_TRUE(predicted.has_value());
    EXPECT_NEAR(
        (*predicted - (origin + velocity * 25.0)).norm(),
        0.0,
        1e-8);
}

TEST(RTKFloatStabilizerTest, LeavesHealthyFloatUnchanged) {
    std::deque<FixedAnchor> anchors = {
        {100.0, Eigen::Vector3d::Zero()},
        {101.0, Eigen::Vector3d(1.0, 0.0, 0.0)},
    };

    EXPECT_FALSE(predict(
        anchors,
        102.0,
        Eigen::Vector3d(2.1, 0.0, 0.0),
        25.0).has_value());
    EXPECT_FALSE(predict(
        anchors,
        102.0,
        Eigen::Vector3d(20.0, 0.0, 0.0),
        10.0).has_value());
}

TEST(RTKFloatStabilizerTest, RejectsStaleAnchor) {
    std::deque<FixedAnchor> anchors = {
        {100.0, Eigen::Vector3d::Zero()},
        {101.0, Eigen::Vector3d(1.0, 0.0, 0.0)},
    };

    EXPECT_FALSE(predict(
        anchors,
        117.0,
        Eigen::Vector3d(100.0, 0.0, 0.0),
        25.0).has_value());
}

TEST(RTKFloatStabilizerTest, RejectsCurvedFixedTrajectory) {
    std::deque<FixedAnchor> anchors;
    for (int i = 0; i <= 20; ++i) {
        const double angle = 0.1 * i;
        anchors.push_back(
            {1000.0 + i,
             Eigen::Vector3d(
                 20.0 * std::cos(angle),
                 20.0 * std::sin(angle),
                 0.0)});
    }

    EXPECT_FALSE(predict(
        anchors,
        1021.0,
        Eigen::Vector3d(100.0, 100.0, 0.0),
        25.0).has_value());
}

TEST(RTKFloatStabilizerTest, DoesNotArmOutsideShortBaselineRegion) {
    EXPECT_TRUE(shouldArm(999.0, 1000.0));
    EXPECT_FALSE(shouldArm(9000.0, 1000.0));
}

}  // namespace
}  // namespace libgnss::rtk_float_stabilizer
