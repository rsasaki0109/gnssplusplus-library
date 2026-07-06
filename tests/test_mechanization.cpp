#include <gtest/gtest.h>

#include <libgnss++/fusion/mechanization.hpp>

#include <cmath>

namespace libgnss {
namespace {

constexpr double kGravity = 9.80665;
const Eigen::Vector3d kGravityEnu(0.0, 0.0, -kGravity);

TEST(MechanizationTest, ConstantVelocityLevelFlightIntegratesLinearly) {
    // Level attitude, constant ENU velocity (5, 0, 0) m/s, zero rotation.
    // Closed-form: specific force must exactly cancel gravity every step
    // (f_body = R^T * (-gravity_enu)), so velocity/attitude stay fixed and
    // position advances linearly.
    NominalState state;
    state.attitude_body_to_enu = Eigen::Quaterniond::Identity();
    state.velocity_enu = Eigen::Vector3d(5.0, 0.0, 0.0);
    state.position_enu = Eigen::Vector3d::Zero();

    ImuSample sample;
    sample.accel_raw = state.attitude_body_to_enu.conjugate() * (-kGravityEnu);
    sample.gyro_raw_radps.setZero();

    constexpr double dt = 0.1;
    constexpr int steps = 100;  // 10 s total
    for (int i = 0; i < steps; ++i) {
        sample.time = state.time + dt;
        state = mechanization::propagate(state, sample, dt, kGravityEnu);
    }

    EXPECT_TRUE(state.velocity_enu.isApprox(Eigen::Vector3d(5.0, 0.0, 0.0), 1e-9));
    EXPECT_TRUE(state.position_enu.isApprox(Eigen::Vector3d(50.0, 0.0, 0.0), 1e-6));
    EXPECT_TRUE(state.attitude_body_to_enu.isApprox(Eigen::Quaterniond::Identity(), 1e-9));
}

TEST(MechanizationTest, ConstantTurnRateIntegratesYawExactly) {
    // Body spinning in place about its own Up axis at a constant rate;
    // omega_body = [0, 0, turn_rate] is independent of velocity/position/
    // accel feed, so the closed-form yaw after time T is exactly
    // turn_rate * T regardless of any (small) mid-point-vs-start attitude
    // discrepancy in the specific-force rotation.
    const double turn_rate = 0.2;  // rad/s
    NominalState state;
    state.attitude_body_to_enu = Eigen::Quaterniond::Identity();
    state.velocity_enu.setZero();
    state.position_enu.setZero();

    ImuSample sample;
    sample.gyro_raw_radps = Eigen::Vector3d(0.0, 0.0, turn_rate);

    constexpr double dt = 0.01;
    constexpr int steps = 500;  // 5 s total
    for (int i = 0; i < steps; ++i) {
        // Feed the specific force that would exactly cancel gravity at the
        // *start*-of-step attitude (a first-order approximation of the
        // mid-point value mechanization::propagate actually uses).
        sample.accel_raw = state.attitude_body_to_enu.conjugate() * (-kGravityEnu);
        sample.time = state.time + dt;
        state = mechanization::propagate(state, sample, dt, kGravityEnu);
    }

    const double total_time = steps * dt;
    const Eigen::Quaterniond expected_attitude(
        Eigen::AngleAxisd(turn_rate * total_time, Eigen::Vector3d::UnitZ()));
    EXPECT_TRUE(state.attitude_body_to_enu.isApprox(expected_attitude, 1e-9));
    // Velocity/position should stay close to the stationary starting point;
    // small residual drift is expected from the start-vs-mid-point
    // specific-force approximation used to feed this synthetic test.
    EXPECT_LT(state.velocity_enu.norm(), 1e-2);
    EXPECT_LT(state.position_enu.norm(), 1e-2);
}

TEST(MechanizationTest, ZeroDtIsNoOp) {
    NominalState state;
    state.velocity_enu = Eigen::Vector3d(1.0, 2.0, 3.0);
    state.position_enu = Eigen::Vector3d(10.0, 20.0, 30.0);

    ImuSample sample;
    sample.accel_raw = Eigen::Vector3d(0.1, 0.2, 0.3);
    sample.gyro_raw_radps = Eigen::Vector3d(0.01, 0.02, 0.03);

    const NominalState propagated = mechanization::propagate(state, sample, 0.0, kGravityEnu);
    EXPECT_TRUE(propagated.velocity_enu.isApprox(state.velocity_enu, 1e-12));
    EXPECT_TRUE(propagated.position_enu.isApprox(state.position_enu, 1e-12));
}

}  // namespace
}  // namespace libgnss
