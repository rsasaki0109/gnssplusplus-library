#include <gtest/gtest.h>
#include <libgnss++/algorithms/native_nhc_gate.hpp>

namespace {
using namespace libgnss;
std::vector<ImuSample> straight() {
    std::vector<ImuSample> s(5);
    for (std::size_t i = 0; i < s.size(); ++i) {
        s[i].time.week = 2200;
        s[i].time.tow = 100. + i * .25;
    }
    return s;
}
NativeNhcGateResult gate(const std::vector<ImuSample>& s, double speed = 20.) {
    GNSSTime t0, t1;
    t0.week = t1.week = 2200;
    t0.tow = 100.; t1.tow = 101.;
    return nativeNhcGate(s, 0, s.size(), t0, t1,
                         Eigen::Vector3d::Zero(), speed, 2., .2, .25);
}
TEST(NativeNhcGate, StraightAndSpeedBoundary) {
    EXPECT_TRUE(gate(straight()).admitted);
    EXPECT_TRUE(gate(straight(), 2.).admitted);
    EXPECT_FALSE(gate(straight(), 1.99).admitted);
}
TEST(NativeNhcGate, MissingCoverageAndOrderingRejected) {
    EXPECT_FALSE(gate({}).supported);
    auto s = straight(); s.erase(s.begin() + 2);
    EXPECT_FALSE(gate(s).supported);
    s = straight(); s.erase(s.begin(), s.begin() + 2);
    EXPECT_FALSE(gate(s).supported);
    s = straight(); s.resize(3);
    EXPECT_FALSE(gate(s).supported);
    s = straight(); s[2].time = s[1].time;
    EXPECT_FALSE(gate(s).supported);
}
TEST(NativeNhcGate, ReversalAndRollCannotCancel) {
    auto s = straight();
    s[1].gyro_raw_radps.z() = .4;
    s[2].gyro_raw_radps.z() = -.4;
    EXPECT_TRUE(gate(s).supported);
    EXPECT_FALSE(gate(s).admitted);
    EXPECT_DOUBLE_EQ(gate(s).peak_angular_speed_radps, .4);
    s = straight(); s[2].gyro_raw_radps.x() = .4;
    EXPECT_FALSE(gate(s).admitted);
}
TEST(NativeNhcGate, NonfiniteRejected) {
    auto s = straight();
    s[2].gyro_raw_radps.z() = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(gate(s).supported);
    EXPECT_FALSE(gate(straight(), std::numeric_limits<double>::infinity()).supported);
}
}
