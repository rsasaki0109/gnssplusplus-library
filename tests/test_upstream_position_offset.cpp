#include <gtest/gtest.h>

#include <libgnss++/algorithms/upstream_position_offset.hpp>

#include <Eigen/Geometry>

#include <cmath>
#include <limits>

TEST(UpstreamPositionOffset, MatchesPinnedPhoneBranches) {
    libgnss::upstream_position_offset::PhoneOffset offset;
    ASSERT_TRUE(libgnss::upstream_position_offset::phoneOffset(
        "sm-g988", offset));
    EXPECT_DOUBLE_EQ(offset.offset_rl_m, 0.20);
    EXPECT_DOUBLE_EQ(offset.offset_ud_m, -0.05);

    ASSERT_TRUE(libgnss::upstream_position_offset::phoneOffset(
        "pixel7pro", offset));
    EXPECT_DOUBLE_EQ(offset.offset_rl_m, -0.10);
    EXPECT_DOUBLE_EQ(offset.offset_ud_m, -0.20);
    ASSERT_TRUE(libgnss::upstream_position_offset::phoneOffset(
        "samsung-a52", offset));
    EXPECT_DOUBLE_EQ(offset.offset_rl_m, 0.30);
    EXPECT_DOUBLE_EQ(offset.offset_ud_m, -0.25);
}

TEST(UpstreamPositionOffset, UnknownPhoneFailsClosed) {
    libgnss::upstream_position_offset::PhoneOffset offset;
    EXPECT_FALSE(libgnss::upstream_position_offset::phoneOffset(
        "unknown-phone", offset));
    EXPECT_FALSE(libgnss::upstream_position_offset::offsetFromRpy(
                     "unknown-phone", Eigen::Vector3d::Zero())
                     .ok);
}

TEST(UpstreamPositionOffset, UsesRxRyRzAfterYawPiShift) {
    const Eigen::Vector3d rpy(0.0, 0.0, 0.0);
    const Eigen::Matrix3d matrix =
        libgnss::upstream_position_offset::eul2rotm(rpy);
    const Eigen::Matrix3d expected =
        Eigen::AngleAxisd(-std::acos(-1.0), Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
    EXPECT_TRUE(matrix.isApprox(expected, 1.0e-15));

    const auto result = libgnss::upstream_position_offset::offsetFromRpy(
        "pixel7pro", rpy);
    ASSERT_TRUE(result.ok);
    EXPECT_TRUE(result.offset_enu_m.isApprox(Eigen::Vector3d(0.20, 0.10, 0.0),
                                             1.0e-15));
}

TEST(UpstreamPositionOffset, PreservesOffsetNormAndRejectsNonFiniteRpy) {
    const Eigen::Vector3d rpy(0.31, -0.42, 1.17);
    const auto result = libgnss::upstream_position_offset::offsetFromRpy(
        "pixel7pro", rpy);
    ASSERT_TRUE(result.ok);
    EXPECT_NEAR(result.offset_enu_m.norm(),
                std::hypot(0.20, 0.10), 1.0e-14);

    Eigen::Vector3d invalid = rpy;
    invalid.x() = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(libgnss::upstream_position_offset::offsetFromRpy(
                     "pixel7pro", invalid)
                     .ok);
}
