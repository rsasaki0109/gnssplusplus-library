#include <gtest/gtest.h>

#include <limits>

#include <libgnss++/algorithms/safe_float_continuity.hpp>

namespace {

TEST(SafeFloatContinuityTest, DisabledFailsClosed) {
    libgnss::safe_float_continuity::Config config;
    EXPECT_FALSE(
        libgnss::safe_float_continuity::propagate(
            config,
            Eigen::Vector3d(1.0, 2.0, 3.0),
            1.0,
            Eigen::Vector3d(4.0, 0.0, 0.0),
            0.2)
            .valid);
}

TEST(SafeFloatContinuityTest, PropagatesTrustedAnchorWithBoundedVelocity) {
    libgnss::safe_float_continuity::Config config;
    config.enabled = true;
    const auto result =
        libgnss::safe_float_continuity::propagate(
            config,
            Eigen::Vector3d(100.0, 200.0, 300.0),
            2.0,
            Eigen::Vector3d(5.0, -1.0, 0.5),
            1.0);
    ASSERT_TRUE(result.valid);
    EXPECT_TRUE(
        result.position_ecef.isApprox(
            Eigen::Vector3d(110.0, 198.0, 301.0)));
    EXPECT_DOUBLE_EQ(result.position_variance_m2, 61.0);
}

TEST(SafeFloatContinuityTest, StaleOrImplausibleInputsFailClosed) {
    libgnss::safe_float_continuity::Config config;
    config.enabled = true;
    EXPECT_FALSE(
        libgnss::safe_float_continuity::propagate(
            config,
            Eigen::Vector3d::Zero(),
            6.1,
            Eigen::Vector3d::Zero(),
            0.0)
            .valid);
    EXPECT_FALSE(
        libgnss::safe_float_continuity::propagate(
            config,
            Eigen::Vector3d::Zero(),
            1.0,
            Eigen::Vector3d(81.0, 0.0, 0.0),
            0.0)
            .valid);
    EXPECT_FALSE(
        libgnss::safe_float_continuity::propagate(
            config,
            Eigen::Vector3d::Zero(),
            1.0,
            Eigen::Vector3d(
                std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0),
            0.0)
            .valid);
}

}  // namespace
