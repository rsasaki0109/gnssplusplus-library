#include <gtest/gtest.h>
#include "fgo_gtsam_internal.hpp"
#include <ClockFactor_CCDD.h>
#include <MotionFactor_XXVV.h>
#include <DopplerFactor_VD.h>
#include <PseudorangeFactor_XC.h>
#include <TDCPFactor_XXCC.h>

namespace native = libgnss::fgo_gtsam_internal;
namespace {
void equal(const gtsam::Matrix& actual, const gtsam::Matrix& reference,
           double tolerance = 1e-12) {
    ASSERT_EQ(actual.rows(), reference.rows());
    ASSERT_EQ(actual.cols(), reference.cols());
    ASSERT_TRUE(actual.allFinite());
    ASSERT_TRUE(reference.allFinite());
    EXPECT_LE((actual-reference).cwiseAbs().maxCoeff(), tolerance);
}
}

TEST(PinnedGsdcFactors, ClockVectorResidualAndFourJacobians) {
    const auto noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(7) << 0.1,0,0,0,0,0,0).finished());
    for (double dt : {0.001, 0.999, 1.0, 1.499, 4.0}) {
        native::SourceClockVectorC0DFactor actual(1,2,3,4,dt,noise);
        gtsam_gnss::ClockFactor_CCDD reference(1,2,3,4,dt,noise);
        for (double scale : {1.0, 1000000.0, -1000000.0}) {
            const gtsam::Vector c1 = (gtsam::Vector(7) << scale,2,-3,4,5,-6,7).finished();
            const gtsam::Vector c2 = c1 + gtsam::Vector::LinSpaced(7,-0.5,1.5);
            const gtsam::Vector d1 = gtsam::Vector::Constant(1,-120.25);
            const gtsam::Vector d2 = gtsam::Vector::Constant(1,10.5);
            gtsam::Matrix a,b,c,d,ra,rb,rc,rd;
            const gtsam::Vector error = actual.evaluateError(c1,c2,d1,d2,&a,&b,&c,&d);
            const gtsam::Vector expected = reference.evaluateError(c1,c2,d1,d2,&ra,&rb,&rc,&rd);
            equal(error,expected); equal(a,ra); equal(b,rb); equal(c,rc); equal(d,rd);
        }
    }
}

TEST(PinnedGsdcFactors, MotionResidualAndFourJacobians) {
    const auto noise = gtsam::noiseModel::Isotropic::Sigma(3,0.1);
    for (double dt : {0.001, 0.999, 1.0, 1.499, 4.0}) {
        native::MotionFactorXXVV actual(1,2,3,4,dt,noise);
        gtsam_gnss::MotionFactor_XXVV reference(1,2,3,4,dt,noise);
        for (double shift : {0.0, 6378137.0, -6378137.0}) {
            const gtsam::Vector3 x1(shift,20,-30), x2(shift+12.5,21.5,-29);
            const gtsam::Vector3 v1(1,2,3), v2(3,-2,5);
            gtsam::Matrix a,b,c,d,ra,rb,rc,rd;
            const gtsam::Vector error = actual.evaluateError(x1,x2,v1,v2,&a,&b,&c,&d);
            const gtsam::Vector expected = reference.evaluateError(x1,x2,v1,v2,&ra,&rb,&rc,&rd);
            equal(error,expected); equal(a,ra); equal(b,rb); equal(c,rc); equal(d,rd);
        }
    }
}

TEST(PinnedGsdcFactors, DopplerResidualAfterExplicitLinearizationOriginConversion) {
    const auto noise = gtsam::noiseModel::Isotropic::Sigma(1,0.5);
    for (double scale : {0.0, 1.0, -20.0, 50.0}) {
        const gtsam::Vector3 los = gtsam::Vector3(0.2,-0.3,0.7).normalized();
        const gtsam::Vector3 origin(scale,-2*scale,0.5*scale);
        const gtsam::Vector3 velocity = origin + gtsam::Vector3(1.5,-3.2,0.9);
        for (double residual : {-300.0,0.0,250.0}) {
            const gtsam::Vector drift = gtsam::Vector::Constant(1,120.25);
            // Source takes a residual at origin; native takes the corresponding
            // absolute projected measurement. Compare their actual evaluators.
            native::UndifferencedDopplerVelocityFactorSourceClockEcef actual(
                1,2,los,residual+los.dot(origin),noise);
            gtsam_gnss::DopplerFactor_VD reference(1,2,los,residual,origin,noise);
            gtsam::Matrix a,b,ra,rb;
            const gtsam::Vector error = actual.evaluateError(velocity,drift,&a,&b);
            const gtsam::Vector expected = reference.evaluateError(velocity,drift,&ra,&rb);
            equal(error,expected,1e-10); equal(a,ra); equal(b,rb);
        }
    }
}

TEST(PinnedGsdcFactors, OptionalAffinePseudorangeAllSevenClockComponents) {
    const auto noise = gtsam::noiseModel::Isotropic::Sigma(1,3.0);
    const gtsam::Vector3 los = gtsam::Vector3(0.2,-0.3,0.7).normalized();
    const gtsam::Vector3 origin(-2700000,-4300000,3800000);
    const gtsam::Vector clock = (gtsam::Vector(7) << 100,2,-3,4,5,-6,7).finished();
    for (int component=0;component<7;++component) {
        native::Phase135PseudorangeAffinePointFactor actual(1,2,los,17,component,origin,noise);
        gtsam_gnss::PseudorangeFactor_XC reference(1,2,los,17,component,origin,noise);
        for (double displacement : {0.0,1.0,100.0}) {
            const gtsam::Vector3 point=origin+gtsam::Vector3(displacement,-2*displacement,0.5*displacement);
            gtsam::Matrix a,b,ra,rb;
            const gtsam::Vector error=actual.evaluateError(point,clock,&a,&b);
            const gtsam::Vector expected=reference.evaluateError(point,clock,&ra,&rb);
            equal(error,expected);equal(a,ra);equal(b,rb);
        }
    }
}

TEST(PinnedGsdcFactors, OptionalAffineTdcpResidualAndFourJacobians) {
    const auto noise = gtsam::noiseModel::Isotropic::Sigma(1,0.1);
    const gtsam::Vector3 los=gtsam::Vector3(0.2,-0.3,0.7).normalized();
    const gtsam::Vector3 origin1(-2700000,-4300000,3800000),origin2=origin1+gtsam::Vector3(10,20,3);
    const gtsam::Vector c1=(gtsam::Vector(7)<<100,2,-3,4,5,-6,7).finished();
    const gtsam::Vector c2=c1+gtsam::Vector::LinSpaced(7,0.5,100.0);
    for (double displacement : {0.0,1.0,100.0}) {
        const gtsam::Vector3 x1=origin1+gtsam::Vector3(displacement,-2*displacement,0.5*displacement);
        const gtsam::Vector3 x2=origin2+gtsam::Vector3(-2*displacement,displacement,0.1*displacement);
        native::Phase135TdcpAffinePointFactor actual(1,2,3,4,los,0.7,origin1,origin2,noise);
        gtsam_gnss::TDCPFactor_XXCC reference(1,2,3,4,los,0.7,origin1,origin2,noise);
        gtsam::Matrix a,b,c,d,ra,rb,rc,rd;
        const gtsam::Vector error=actual.evaluateError(x1,x2,c1,c2,&a,&b,&c,&d);
        const gtsam::Vector expected=reference.evaluateError(x1,x2,c1,c2,&ra,&rb,&rc,&rd);
        equal(error,expected);equal(a,ra);equal(b,rb);equal(c,rc);equal(d,rd);
    }
}

TEST(PinnedGsdcFactors, NonlinearPseudorangeMatchesAnchorButDiffersAwayFromIt) {
    const auto noise=gtsam::noiseModel::Isotropic::Sigma(1,3.0);
    const gtsam::Vector3 satellite(20200000,14000000,21000000);
    const gtsam::Vector3 origin(-2700000,-4300000,3800000);
    const double range=(origin-satellite).norm();
    const gtsam::Vector3 los=(origin-satellite)/range;
    const gtsam::Vector clock=gtsam::Vector::Zero(7);
    native::PseudorangeFactorSourceClock actual(1,2,range+17,satellite,0,noise);
    gtsam_gnss::PseudorangeFactor_XC reference(1,2,los,17,0,origin,noise);
    gtsam::Matrix a,b,ra,rb;
    equal(actual.evaluateError(origin,clock,&a,&b),reference.evaluateError(origin,clock,&ra,&rb),1e-8);
    equal(a,ra);equal(b,rb);
    const gtsam::Vector3 moved=origin+1000*los.cross(gtsam::Vector3::UnitZ()).normalized();
    const double difference=(actual.evaluateError(moved,clock)-reference.evaluateError(moved,clock)).norm();
    EXPECT_GT(difference,0.001);
    EXPECT_LT(difference,0.1);
    std::cout << "nonlinear_p_vs_source_at_1000m_transverse_shift_m=" << std::setprecision(17) << difference << '\n';
}
