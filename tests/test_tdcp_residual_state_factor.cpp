#include <gtest/gtest.h>
#ifdef GNSSPP_HAS_GTSAM
#include <libgnss++/algorithms/tdcp_residual_state_factor.hpp>
#include <libgnss++/algorithms/tdcp_ionosphere_endpoints_factor.hpp>
#include <libgnss++/algorithms/code_ionosphere_state_factor.hpp>
#include "../src/algorithms/fgo_gtsam_internal.hpp"

namespace {
using libgnss::tdcp_frequency::ResidualStateFactor;
using libgnss::fgo_gtsam_internal::TimeDifferencedCarrierFactorSourceClockPoint;

std::shared_ptr<TimeDifferencedCarrierFactorSourceClockPoint> original() {
    auto gaussian = gtsam::noiseModel::Isotropic::Sigma(1, .002);
    auto robust = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(4.), gaussian);
    return std::make_shared<TimeDifferencedCarrierFactorSourceClockPoint>(
        1, 2, 3, 4, gtsam::Point3(20., 30., 40.),
        gtsam::Point3(21., 31., 41.), .4, robust);
}
gtsam::Values fixture() {
    gtsam::Values values;
    values.insert(1, gtsam::Point3(1., 2., 3.));
    values.insert(3, gtsam::Point3(1.1, 2.1, 3.1));
    values.insert(2, gtsam::Vector(gtsam::Vector::Zero(7)));
    values.insert(4, gtsam::Vector(gtsam::Vector::Constant(7, .1)));
    values.insert(5, 0.0);
    return values;
}

TEST(TdcpResidualStateFactor, ZeroStatePreservesNativeC7ResidualNoiseAndJacobians) {
    auto base = original();
    ResidualStateFactor factor(base, 5, 1.8);
    auto values = fixture();
    std::vector<gtsam::Matrix> before(4), after(5);
    const auto expected = base->unwhitenedError(values, before);
    const auto actual = factor.unwhitenedError(values, after);
    EXPECT_EQ(factor.noiseModel(), base->noiseModel());
    EXPECT_DOUBLE_EQ(actual[0], expected[0]);
    EXPECT_DOUBLE_EQ(factor.error(values), base->error(values));
    ASSERT_EQ(after.size(), 5);
    for (size_t i=0; i<4; ++i) EXPECT_EQ((after[i]-before[i]).norm(), 0.0);
    EXPECT_DOUBLE_EQ(after[4](0,0), -1.8);
}

TEST(TdcpResidualStateFactor, NuisanceDerivativeAndClone) {
    ResidualStateFactor factor(original(), 5, 1.8);
    auto values = fixture();
    const double baseline = factor.unwhitenedError(values)[0];
    values.update(5, .03);
    EXPECT_NEAR(factor.unwhitenedError(values)[0], baseline-.054, 1e-14);
    auto plus = values, minus = values;
    plus.update(5, .030001); minus.update(5, .029999);
    EXPECT_NEAR((factor.unwhitenedError(plus)[0]-factor.unwhitenedError(minus)[0])/2e-6,
                -1.8, 1e-8);
    EXPECT_DOUBLE_EQ(factor.clone()->error(values), factor.error(values));
    EXPECT_TRUE(factor.equals(*factor.clone()));
    EXPECT_FALSE(factor.equals(ResidualStateFactor(original(), 5, 1.)));
    EXPECT_NO_THROW(factor.linearize(values));
}

TEST(TdcpResidualStateFactor, RejectsInvalidKeysCoefficientsAndStates) {
    EXPECT_THROW(ResidualStateFactor(original(), 1, 1.), std::invalid_argument);
    EXPECT_THROW(ResidualStateFactor(nullptr, 5, 1.), std::invalid_argument);
    EXPECT_THROW(ResidualStateFactor(original(), 5, 0.), std::invalid_argument);
    auto values = fixture();
    values.update(5, std::numeric_limits<double>::quiet_NaN());
    EXPECT_THROW(ResidualStateFactor(original(), 5, 1.).unwhitenedError(values),
                 std::invalid_argument);
}

TEST(TdcpResidualStateFactor, Pose3ArmJacobianMatchesNumericalRetraction) {
    using libgnss::fgo_gtsam_internal::TimeDifferencedCarrierFactorSourceClockArm;
    const auto noise = original()->noiseModel();
    auto base = std::make_shared<TimeDifferencedCarrierFactorSourceClockArm>(
        1, 2, 3, 4, gtsam::Point3(20.,30.,40.), gtsam::Point3(21.,31.,41.),
        .4, gtsam::gnss::LeverArm(gtsam::Point3(.2,-.1,.3)), noise);
    ResidualStateFactor factor(base, 5, 1.8);
    auto values = fixture();
    values.erase(1); values.erase(3);
    values.insert(1, gtsam::Pose3(gtsam::Rot3::RzRyRx(.1,.2,.3), {1.,2.,3.}));
    values.insert(3, gtsam::Pose3(gtsam::Rot3::RzRyRx(.2,.1,.4), {1.1,2.1,3.1}));
    EXPECT_DOUBLE_EQ(factor.error(values), base->error(values));
    values.update(5, .03);
    std::vector<gtsam::Matrix> jacobians(5);
    factor.unwhitenedError(values, jacobians);
    for (const auto key : {1,3}) {
        const auto pose = values.at<gtsam::Pose3>(key);
        for (int axis=0; axis<6; ++axis) {
            gtsam::Vector6 delta = gtsam::Vector6::Zero(); delta[axis]=1e-5;
            auto plus=values, minus=values;
            plus.update(key, pose.retract(delta)); minus.update(key, pose.retract(-delta));
            const double numeric=(factor.unwhitenedError(plus)[0]-
                                  factor.unwhitenedError(minus)[0])/2e-5;
            EXPECT_NEAR(jacobians[key==1 ? 0 : 2](0,axis), numeric, 1e-8);
        }
    }
}

TEST(TdcpResidualStateFactor, PairedNativeGraphRecoversSharedResidualChange) {
    auto initial = fixture();
    const double slant_change=.03;
    const double geometry_clock = original()->unwhitenedError(initial)[0]+.4;
    gtsam::NonlinearFactorGraph graph;
    for (const double alpha : {1.0, std::pow(1575.42/1176.45,2)}) {
        auto measurement = std::make_shared<TimeDifferencedCarrierFactorSourceClockPoint>(
            1,2,3,4,gtsam::Point3(20.,30.,40.),gtsam::Point3(21.,31.,41.),
            geometry_clock-alpha*slant_change, original()->noiseModel());
        graph.emplace_shared<ResidualStateFactor>(measurement,5,alpha);
    }
    for (auto key : {1,3}) graph.add(gtsam::PriorFactor<gtsam::Point3>(
        key, initial.at<gtsam::Point3>(key), gtsam::noiseModel::Isotropic::Sigma(3,1e-5)));
    for (auto key : {2,4}) graph.add(gtsam::PriorFactor<gtsam::Vector>(
        key, initial.at<gtsam::Vector>(key), gtsam::noiseModel::Isotropic::Sigma(7,1e-5)));
    // Synthetic prior only, not a recommended smartphone model setting.
    graph.add(gtsam::PriorFactor<double>(5,0.,gtsam::noiseModel::Isotropic::Sigma(1,1.)));
    ASSERT_EQ(graph.size(),7);
    const double before=graph.error(initial);
    const auto optimized=gtsam::LevenbergMarquardtOptimizer(graph,initial).optimize();
    EXPECT_NEAR(optimized.at<double>(5),slant_change,1e-6);
    EXPECT_LT(graph.error(optimized),before);
}
TEST(JointCodeTdcpIonosphere, SharedEndpointStatesRecoverSyntheticDelays) {
    using libgnss::fgo_gtsam_internal::PseudorangeFactorSourceClock;
    auto initial=fixture(); initial.insert(6,0.);
    const double ip=.4, ic=.7;
    const gtsam::Point3 sp(20.,30.,40.),sc(21.,31.,41.);
    const double gp=(sp-initial.at<gtsam::Point3>(1)).norm();
    const double gc=(sc-initial.at<gtsam::Point3>(3)).norm()+.1;
    const double tdcp_geometry=original()->unwhitenedError(initial)[0]+.4;
    const auto code_noise=gtsam::noiseModel::Isotropic::Sigma(1,.1);
    gtsam::NonlinearFactorGraph graph;
    for(double scale:{1.,std::pow(1575.42/1176.45,2)}) {
        const double ap=1.2*scale,ac=1.5*scale;
        auto p=std::make_shared<PseudorangeFactorSourceClock>(1,2,gp+ap*ip,sp,0,code_noise);
        auto c=std::make_shared<PseudorangeFactorSourceClock>(3,4,gc+ac*ic,sc,0,code_noise);
        graph.emplace_shared<libgnss::code_ionosphere::ResidualStateFactor>(p,5,ap);
        graph.emplace_shared<libgnss::code_ionosphere::ResidualStateFactor>(c,6,ac);
        auto carrier=std::make_shared<TimeDifferencedCarrierFactorSourceClockPoint>(
            1,2,3,4,sp,sc,tdcp_geometry+ap*ip-ac*ic,original()->noiseModel());
        graph.emplace_shared<libgnss::code_ionosphere::TdcpEndpointsFactor>(carrier,5,6,ap,ac);
    }
    // Pin geometry/clocks to isolate coupling and sign, NOT full observability.
    for(auto key:{1,3}) graph.addPrior(key,initial.at<gtsam::Point3>(key),
        gtsam::noiseModel::Isotropic::Sigma(3,1e-5));
    for(auto key:{2,4}) graph.addPrior(key,initial.at<gtsam::Vector>(key),
        gtsam::noiseModel::Isotropic::Sigma(7,1e-5));
    ASSERT_EQ(graph.size(),10); // six wrapped observations; no duplicated originals
    auto truth=initial; truth.update(5,ip); truth.update(6,ic);
    EXPECT_LT(graph.error(truth),1e-18);
    const auto result=gtsam::LevenbergMarquardtOptimizer(graph,initial).optimize();
    EXPECT_NEAR(result.at<double>(5),ip,1e-5);
    EXPECT_NEAR(result.at<double>(6),ic,1e-5);
    EXPECT_LT(graph.error(result),graph.error(initial));
}
TEST(TdcpIonosphereEndpoints, ZeroPreservesActualC7AndBothEndpointDerivatives) {
    const auto base=original();
    libgnss::code_ionosphere::TdcpEndpointsFactor factor(base,5,6,1.2,1.5);
    auto x=fixture(); x.insert(6,0.);
    std::vector<gtsam::Matrix> a(4),b(6);
    EXPECT_LT((factor.unwhitenedError(x,b)-base->unwhitenedError(x,a)).norm(),1e-14);
    EXPECT_DOUBLE_EQ(factor.error(x),base->error(x));
    for(size_t i=0;i<4;++i) EXPECT_LT((a[i]-b[i]).norm(),1e-14);
    EXPECT_DOUBLE_EQ(b[4](0,0),1.2);
    EXPECT_DOUBLE_EQ(b[5](0,0),-1.5);
    x.update(5,2.); x.update(6,2.);
    // Constant vertical residual does not cancel when mapping changes.
    EXPECT_NEAR((factor.unwhitenedError(x)-base->unwhitenedError(x))[0],-.6,1e-12);
    for(const auto key:{5,6}) {
        x.update(key,2.+1e-5); const double plus=factor.unwhitenedError(x)[0];
        x.update(key,2.-1e-5); const double minus=factor.unwhitenedError(x)[0];
        EXPECT_NEAR((plus-minus)/2e-5,key==5?1.2:-1.5,1e-8);
        x.update(key,2.);
    }
    EXPECT_TRUE(factor.equals(*factor.clone()));
    EXPECT_THROW(libgnss::code_ionosphere::TdcpEndpointsFactor(base,5,5,1.,1.),std::invalid_argument);
    EXPECT_THROW(libgnss::code_ionosphere::TdcpEndpointsFactor(base,1,6,1.,1.),std::invalid_argument);
    EXPECT_THROW(libgnss::code_ionosphere::TdcpEndpointsFactor(base,5,6,0.,1.),std::invalid_argument);
}
}  // namespace
#endif
