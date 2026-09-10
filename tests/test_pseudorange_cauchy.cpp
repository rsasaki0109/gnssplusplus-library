#include <gtest/gtest.h>
#include "../src/algorithms/fgo_pseudorange_cauchy.hpp"
#include <limits>
#include <libgnss++/algorithms/fgo.hpp>

TEST(PseudorangeCauchyTest, WhitenedResidualLossAndWeightHaveExpectedScale) {
    const auto noise=libgnss::fgo_gtsam_internal::makePseudorangeCauchyNoise(2,4);
    const auto robust=std::dynamic_pointer_cast<gtsam::noiseModel::Robust>(noise);
    ASSERT_TRUE(robust);
    for (double whitened : {0.,1.,4.,40.}) {
        EXPECT_NEAR(robust->robust()->loss(whitened),8*std::log1p(whitened*whitened/16),1e-12);
        EXPECT_NEAR(robust->robust()->weight(whitened),1/(1+whitened*whitened/16),1e-12);
    }
    gtsam::Vector residual(1);residual<<8;
    EXPECT_NEAR(robust->noise()->whiten(residual)[0],4,1e-12);
}

TEST(PseudorangeCauchyTest, InvalidNoiseParametersFailClosed) {
    for (double bad : {0.,-1.,std::numeric_limits<double>::infinity(),
                       std::numeric_limits<double>::quiet_NaN()}) {
        EXPECT_THROW(libgnss::fgo_gtsam_internal::makePseudorangeCauchyNoise(bad,4),std::invalid_argument);
        EXPECT_THROW(libgnss::fgo_gtsam_internal::makePseudorangeCauchyNoise(2,bad),std::invalid_argument);
    }
}

TEST(PseudorangeCauchyTest, UnsupportedSolverConfigurationsFailBeforeEmptyProblem) {
    using namespace libgnss;
    FGOProcessor::FGOConfig supported;
    EXPECT_FALSE(supported.use_main_pseudorange_cauchy_loss);
    supported.use_main_pseudorange_cauchy_loss=true;
    supported.backend=FGOBackend::GTSAM;
    supported.use_imu=true;
    supported.use_native_phase171_raw_p_no_doppler_imu_main=true;
    supported.use_fixed_lag_smoother=false;
    supported.use_robust_loss=true;
    for (int variant=0;variant<5;++variant) {
        auto config=supported;
        if (variant==0) config.backend=FGOBackend::Eigen;
        if (variant==1) config.use_imu=false;
        if (variant==2) config.use_native_phase171_raw_p_no_doppler_imu_main=false;
        if (variant==3) config.use_fixed_lag_smoother=true;
        if (variant==4) config.use_robust_loss=false;
        try {
            (void)FGOProcessor(config).optimizeProblem({});
            FAIL()<<"Unsupported Cauchy config accepted: "<<variant;
        } catch (const std::invalid_argument& error) {
            EXPECT_STREQ(error.what(),"Main P Cauchy requires robust Phase171 batch GTSAM IMU");
        }
    }
}
