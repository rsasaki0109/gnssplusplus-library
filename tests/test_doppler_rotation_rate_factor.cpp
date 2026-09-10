#include <gtest/gtest.h>
#include <libgnss++/algorithms/doppler_rotation_rate_factor.hpp>
#include <gtsam/nonlinear/Values.h>

TEST(DopplerRotationRateFactor, PreservesClockNoiseAndNonunitVelocityJacobian) {
    using namespace libgnss;
    const Vector3d p(15600000.,20100000.,14000000.), r(6371000.,0.,0.);
    const Vector3d v(-1800.,2200.,1300.), receiver_velocity(2.,30.,-1.);
    const auto model = doppler_rotation_rate::make(p,v,r);
    ASSERT_TRUE(model);
    const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, .2);
    for (double drift : {-300., 0., 300.}) {
        const double satellite_clock = .03;
        const double observed = model->satellite_mps +
            model->receiver_velocity_jacobian.dot(receiver_velocity) + drift - satellite_clock;
        doppler_rotation_rate::EcefFactor factor(1,2,p,v,r,observed,satellite_clock,noise);
        gtsam::Vector clock = gtsam::Vector::Constant(1,drift);
        gtsam::Matrix Hv,Hc;
        EXPECT_NEAR(factor.evaluateError(receiver_velocity,clock,&Hv,&Hc)(0),0.,1e-10);
        EXPECT_EQ(factor.noiseModel(),noise);
        EXPECT_DOUBLE_EQ(Hc(0,0),1.);
        EXPECT_GT(std::abs(Hv.norm()-1.),1e-8);
        for (int axis=0;axis<3;++axis) {
            Vector3d plus=receiver_velocity,minus=receiver_velocity;
            plus(axis)+=.01; minus(axis)-=.01;
            EXPECT_NEAR((factor.evaluateError(plus,clock)(0)-
                factor.evaluateError(minus,clock)(0))/.02,Hv(0,axis),1e-10);
        }
        clock(0)+=.4;
        gtsam::Values values;
        values.insert(1,receiver_velocity); values.insert(2,clock);
        EXPECT_NEAR(factor.error(values),2.,1e-9);
        EXPECT_THROW(factor.evaluateError(receiver_velocity,gtsam::Vector::Zero(2)),std::invalid_argument);
    }
}
