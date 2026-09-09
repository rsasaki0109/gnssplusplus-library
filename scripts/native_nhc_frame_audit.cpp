// Synthetic frame sensitivity of the existing factor; no route/truth inputs.
#include "../src/algorithms/fgo_gtsam_internal.hpp"
#include <iostream>
int main() {
    using namespace libgnss::fgo_gtsam_internal;
    NonHolonomicFactor factor(0, 1, gtsam::noiseModel::Isotropic::Sigma(2, 1.));
    const gtsam::Vector3 velocity(20., 0., 0.);
    for (double degrees : {0., 1., 5.}) {
        const double yaw = degrees * std::acos(-1.) / 180.;
        const gtsam::Pose3 pose(gtsam::Rot3::Rz(yaw), gtsam::Point3::Zero());
        const auto residual = factor.evaluateError(pose, velocity, nullptr, nullptr);
        if (std::abs(residual(0) + 20.*std::sin(yaw)) > 1e-12 ||
            std::abs(residual(1)) > 1e-12) return 1;
        std::cout << "yaw_error_deg=" << degrees << " lateral_residual_mps="
                  << residual(0) << " normalized_at_sigma_0_3="
                  << std::abs(residual(0))/0.3 << '\n';
    }
    // Rotation covariance: a genuine vehicle turn does not itself imply
    // lateral slip. Rotate both pose and forward velocity together.
    const auto turning_rotation = gtsam::Rot3::Rz(0.4);
    const auto turning_residual = factor.evaluateError(
        gtsam::Pose3(turning_rotation, gtsam::Point3::Zero()),
        turning_rotation.rotate(velocity), nullptr, nullptr);
    if (turning_residual.norm() > 1e-12) return 2;

    // Audit the existing fixed-lag gate, without changing its behavior.
    // Empty input and opposite turns currently both report zero mean yaw.
    const libgnss::Vector3d zero = libgnss::Vector3d::Zero();
    const std::vector<libgnss::ImuSample> empty;
    const auto missing = imuWindowStats(empty, 0, 0, zero, zero);
    std::vector<libgnss::ImuSample> reversal(2);
    for (auto& sample : reversal) {
        sample.accel_raw = zero;
        sample.gyro_raw_radps = zero;
    }
    reversal[0].gyro_raw_radps.z() = 0.4;
    reversal[1].gyro_raw_radps.z() = -0.4;
    const auto changing = imuWindowStats(reversal, 0, 2, zero, zero);
    if (missing.n != 0 || missing.yaw_rate_abs != 0. ||
        changing.n != 2 || changing.yaw_rate_abs != 0. ||
        std::abs(changing.gyro_median - 0.4) > 1e-12) return 3;
    std::cout << "turning_no_slip_residual_norm=" << turning_residual.norm()
              << " empty_window_yaw=" << missing.yaw_rate_abs
              << " reversal_mean_yaw=" << changing.yaw_rate_abs
              << " reversal_gyro_norm=" << changing.gyro_median << '\n';

    // Joint pose/velocity solve, not a fixed-attitude pseudo observation.
    // Synthetic frame test only: a modest prior yaw error should decrease
    // while the tightly observed forward velocity remains stable.
    gtsam::NonlinearFactorGraph graph;
    const gtsam::Pose3 biased_pose(gtsam::Rot3::Rz(0.05), gtsam::Point3::Zero());
    graph.addPrior<gtsam::Pose3>(0, biased_pose,
        gtsam::noiseModel::Isotropic::Sigma(6, 0.1));
    graph.addPrior<gtsam::Vector3>(1, velocity,
        gtsam::noiseModel::Isotropic::Sigma(3, 0.001));
    graph.emplace_shared<NonHolonomicFactor>(0, 1,
        gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(1.345),
            gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(0.3, 0.2))));
    gtsam::Values initial;
    initial.insert(0, biased_pose);
    initial.insert(1, velocity);
    const auto optimized = gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
    const auto fitted_velocity = optimized.at<gtsam::Vector3>(1);
    const double fitted_yaw = optimized.at<gtsam::Pose3>(0).rotation().yaw();
    if (!(graph.error(optimized) < graph.error(initial)) ||
        !std::isfinite(fitted_yaw) || std::abs(fitted_yaw) >= 0.005 ||
        (fitted_velocity - velocity).norm() >= 0.001) return 4;
    std::cout << "joint_solve_yaw_before_rad=0.05 after_rad=" << fitted_yaw
              << " velocity_change_mps=" << (fitted_velocity - velocity).norm()
              << '\n';
}
