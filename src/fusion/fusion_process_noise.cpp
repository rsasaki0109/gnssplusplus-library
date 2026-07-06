#include <libgnss++/fusion/fusion_process_noise.hpp>

#include <libgnss++/fusion/attitude.hpp>

namespace libgnss {
namespace fusion_process_noise {

namespace {

Eigen::Matrix<double, 15, 15> continuousTransition(const NominalState& state,
                                                    const Eigen::Vector3d& specific_force_body,
                                                    const Eigen::Vector3d& angular_rate_body) {
    Eigen::Matrix<double, 15, 15> fc = Eigen::Matrix<double, 15, 15>::Zero();
    const Eigen::Matrix3d rotation_body_to_enu = state.attitude_body_to_enu.toRotationMatrix();

    // d(dp)/dt = dv
    fc.block<3, 3>(fusion_index::POSITION, fusion_index::VELOCITY) = Eigen::Matrix3d::Identity();
    // d(dv)/dt = -R*[f_b]x*dtheta - R*dba (+ noise)
    fc.block<3, 3>(fusion_index::VELOCITY, fusion_index::ATTITUDE) =
        -rotation_body_to_enu * attitude::skew(specific_force_body);
    fc.block<3, 3>(fusion_index::VELOCITY, fusion_index::ACCEL_BIAS) = -rotation_body_to_enu;
    // d(dtheta)/dt = -[omega_b]x*dtheta - dbg (+ noise)
    fc.block<3, 3>(fusion_index::ATTITUDE, fusion_index::ATTITUDE) =
        -attitude::skew(angular_rate_body);
    fc.block<3, 3>(fusion_index::ATTITUDE, fusion_index::GYRO_BIAS) = -Eigen::Matrix3d::Identity();
    // d(dba)/dt, d(dbg)/dt: pure random walk, no deterministic term.

    return fc;
}

}  // namespace

Eigen::Matrix<double, 15, 15> transitionMatrix(const NominalState& state,
                                               const Eigen::Vector3d& specific_force_body,
                                               const Eigen::Vector3d& angular_rate_body,
                                               double dt) {
    const Eigen::Matrix<double, 15, 15> fc =
        continuousTransition(state, specific_force_body, angular_rate_body);
    const Eigen::Matrix<double, 15, 15> fc_dt = fc * dt;
    return Eigen::Matrix<double, 15, 15>::Identity() + fc_dt + 0.5 * fc_dt * fc_dt;
}

Eigen::Matrix<double, 15, 15> processNoiseCovariance(const Eigen::Matrix<double, 15, 15>& phi,
                                                     const ProcessNoiseConfig& cfg,
                                                     double dt) {
    Eigen::Matrix<double, 15, 15> g_qc_gt = Eigen::Matrix<double, 15, 15>::Zero();
    g_qc_gt.block<3, 3>(fusion_index::VELOCITY, fusion_index::VELOCITY) =
        (cfg.accel_noise_density * cfg.accel_noise_density) * Eigen::Matrix3d::Identity();
    g_qc_gt.block<3, 3>(fusion_index::ATTITUDE, fusion_index::ATTITUDE) =
        (cfg.gyro_noise_density * cfg.gyro_noise_density) * Eigen::Matrix3d::Identity();
    g_qc_gt.block<3, 3>(fusion_index::ACCEL_BIAS, fusion_index::ACCEL_BIAS) =
        (cfg.accel_bias_random_walk * cfg.accel_bias_random_walk) * Eigen::Matrix3d::Identity();
    g_qc_gt.block<3, 3>(fusion_index::GYRO_BIAS, fusion_index::GYRO_BIAS) =
        (cfg.gyro_bias_random_walk * cfg.gyro_bias_random_walk) * Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 15, 15> qd = phi * g_qc_gt * phi.transpose() * dt;
    qd = 0.5 * (qd + qd.transpose());
    return qd;
}

}  // namespace fusion_process_noise
}  // namespace libgnss
