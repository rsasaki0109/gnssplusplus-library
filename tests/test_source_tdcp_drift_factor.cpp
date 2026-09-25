#include <gtest/gtest.h>
#include <limits>
#include <libgnss++/algorithms/smartphone_temporal_recipe.hpp>
#ifdef GNSSPP_HAS_GTSAM
#include <libgnss++/algorithms/source_tdcp_drift_factor.hpp>
#include <gtsam/base/numericalDerivative.h>
#ifdef GNSSPP_TEST_UPSTREAM_TDCP
#include <TDCPFactor_XXDD.h>
#endif
#endif

using namespace libgnss::smartphone_temporal;

TEST(SmartphoneTemporalRecipe, PublishedPhoneFamilies) {
    EXPECT_FALSE(forPhone("sm-a205u").clock_between);
    EXPECT_FALSE(forPhone("sm-a505u").clock_between);
    EXPECT_FALSE(forPhone("samsunga325g").clock_between);
    EXPECT_TRUE(forPhone("sm-a217m").clock_between);
    EXPECT_EQ(forPhone("pixel5").carrier_clock, CarrierClock::BiasDifference);
    EXPECT_EQ(forPhone("sm-a325f").carrier_clock, CarrierClock::Disabled);
    EXPECT_EQ(forPhone("samsunga32").carrier_clock, CarrierClock::Disabled);
    EXPECT_EQ(forPhone("sm-a600t").carrier_clock, CarrierClock::IntegratedDrift);
    EXPECT_DOUBLE_EQ(forPhone("sm-a205u").carrier_offset_m, 1.117);
    EXPECT_DOUBLE_EQ(forPhone("samsunga325g").carrier_offset_m, 0.0);
}

TEST(SmartphoneTemporalRecipe, RequiresExpectedClockEdgeFamily) {
    EXPECT_TRUE(clockEdgeCountMatches(true, "sm-a205u", 0));
    EXPECT_FALSE(clockEdgeCountMatches(true, "sm-a205u", 10));
    EXPECT_FALSE(clockEdgeCountMatches(false, "sm-a205u", 0));
    EXPECT_FALSE(clockEdgeCountMatches(true, "pixel5", 0));
    EXPECT_TRUE(clockEdgeCountMatches(true, "pixel5", 10));
    EXPECT_FALSE(clockEdgeCountMatches(true, "", 0));
}

TEST(SmartphoneTemporalRecipe, StrictUtcMotionBoundaryIndependentOfCarrierModel) {
    EXPECT_TRUE(motionEdgeEligible(0.999));
    EXPECT_TRUE(motionEdgeEligible(1.499));
    for (double dt : {0.0, -1.0, 1.5, 2.0, 45.0,
                      std::numeric_limits<double>::infinity(),
                      std::numeric_limits<double>::quiet_NaN()})
        EXPECT_FALSE(motionEdgeEligible(dt));
}

#ifdef GNSSPP_HAS_GTSAM
TEST(SourceDriftTdcp, ReferenceResidualAndJacobians) {
    const gtsam::Vector3 los(0.6, 0.0, 0.8);
    const gtsam::Point3 a1(10, 20, 30), a2(13, 24, 32);
    const gtsam::Point3 x1 = a1 + gtsam::Vector3(1, 2, 3);
    const gtsam::Point3 x2 = a2 + gtsam::Vector3(4, -1, 2);
    const gtsam::Vector d1 = gtsam::Vector::Constant(1, 2.0);
    const gtsam::Vector d2 = gtsam::Vector::Constant(1, -0.5);
    const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1);
    for (double dt : {0.25, 1.0, 1.4, 2.0}) {
        DriftTdcpPointFactor f(1, 2, 3, 4, los, a1, a2, 0.4, dt, noise);
        gtsam::Matrix h1, h2, hd1, hd2;
        auto e = f.evaluateError(x1, x2, d1, d2, h1, h2, hd1, hd2);
        EXPECT_NEAR(e[0], 0.6 + 0.75 * dt, 1e-12);
        const auto fn = [&](const gtsam::Point3& p) {
            return f.evaluateError(p, x2, d1, d2);
        };
        EXPECT_LT((h1 - gtsam::numericalDerivative11<gtsam::Vector,
                   gtsam::Point3>(fn, x1)).norm(), 1e-8);
        EXPECT_LT((h1 + h2).norm(), 1e-12);
        EXPECT_DOUBLE_EQ(hd1(0, 0), dt / 2);
        EXPECT_DOUBLE_EQ(hd2(0, 0), dt / 2);
#ifdef GNSSPP_TEST_UPSTREAM_TDCP
        gtsam_gnss::TDCPFactor_XXDD reference(1, 2, 3, 4,
            los, 0.4, dt, a1, a2, noise);
        gtsam::Matrix r1, r2, rd1, rd2;
        auto re = reference.evaluateError(x1, x2, d1, d2,
                                          r1, r2, rd1, rd2);
        EXPECT_LT((e - re).norm(), 1e-12);
        EXPECT_LT((h1 - r1).norm(), 1e-12);
        EXPECT_LT((h2 - r2).norm(), 1e-12);
        EXPECT_LT((hd1 - rd1).norm(), 1e-12);
        EXPECT_LT((hd2 - rd2).norm(), 1e-12);
#endif
    }
}

TEST(SourceDriftTdcp, PoseJacobiansAndInvalidState) {
    const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1);
    const gtsam::Vector3 los(0.6, 0, 0.8);
    const gtsam::Pose3 p1(gtsam::Rot3::RzRyRx(0.2, -0.1, 0.4),
                         gtsam::Point3(10, 20, 30));
    const gtsam::Pose3 p2(gtsam::Rot3::RzRyRx(-0.3, 0.2, 0.1),
                         gtsam::Point3(15, 22, 31));
    const gtsam::Vector d = gtsam::Vector::Constant(1, 2);
    const gtsam::gnss::LeverArm arm(gtsam::Point3(0.1, -0.2, 0.3),
        gtsam::Pose3(gtsam::Rot3::RzRyRx(0.4, -0.2, 0.3),
                     gtsam::Point3(100, 200, 300)));
    DriftTdcpPoseFactor f(1, 2, 3, 4, los, arm.antennaPosition(p1),
                         arm.antennaPosition(p2), 1.117, 1, noise, arm);
    gtsam::Matrix h1, h2, hd1, hd2;
    const auto e = f.evaluateError(p1, p2, d, d, h1, h2, hd1, hd2);
    EXPECT_NEAR(e[0], 2 - 1.117, 1e-12);
    const auto fn1 = [&](const gtsam::Pose3& p) {
        return f.evaluateError(p, p2, d, d);
    };
    const auto fn2 = [&](const gtsam::Pose3& p) {
        return f.evaluateError(p1, p, d, d);
    };
    EXPECT_LT((h1 - gtsam::numericalDerivative11<gtsam::Vector,
               gtsam::Pose3>(fn1, p1)).norm(), 1e-8);
    EXPECT_LT((h2 - gtsam::numericalDerivative11<gtsam::Vector,
               gtsam::Pose3>(fn2, p2)).norm(), 1e-8);
    EXPECT_THROW(f.evaluateError(p1, p2, gtsam::Vector::Zero(7), d),
                 std::invalid_argument);
    EXPECT_THROW((DriftTdcpPoseFactor(1, 2, 3, 4, los, p1.translation(),
                 p2.translation(), 0, 0, noise)), std::invalid_argument);
}
#endif
