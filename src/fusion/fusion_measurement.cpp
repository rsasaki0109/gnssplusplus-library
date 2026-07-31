#include <libgnss++/fusion/fusion_measurement.hpp>

#include <libgnss++/fusion/attitude.hpp>

namespace libgnss {
namespace fusion_measurement {

FusionMeasurementSystem buildGnssPositionUpdate(const FusionState& state,
                                                const Eigen::Vector3d& antenna_position_enu,
                                                const Eigen::Matrix3d& position_covariance_enu,
                                                const Eigen::Vector3d& lever_arm_body) {
    const Eigen::Matrix3d rotation = state.nominal.attitude_body_to_enu.toRotationMatrix();
    const Eigen::Vector3d predicted = state.nominal.position_enu + rotation * lever_arm_body;

    FusionMeasurementSystem system;
    system.design_matrix = Eigen::MatrixXd::Zero(3, fusion_index::SIZE);
    system.design_matrix.block<3, 3>(0, fusion_index::POSITION) = Eigen::Matrix3d::Identity();
    system.design_matrix.block<3, 3>(0, fusion_index::ATTITUDE) =
        -rotation * attitude::skew(lever_arm_body);
    system.residuals = antenna_position_enu - predicted;
    system.covariance = position_covariance_enu;
    return system;
}

FusionMeasurementSystem buildGnssVelocityUpdate(const FusionState& state,
                                                const Eigen::Vector3d& antenna_velocity_enu,
                                                const Eigen::Matrix3d& velocity_covariance_enu,
                                                const Eigen::Vector3d& lever_arm_body,
                                                const Eigen::Vector3d& angular_rate_body) {
    const Eigen::Matrix3d rotation = state.nominal.attitude_body_to_enu.toRotationMatrix();
    const Eigen::Vector3d lever_velocity_body = angular_rate_body.cross(lever_arm_body);
    const Eigen::Vector3d predicted =
        state.nominal.velocity_enu + rotation * lever_velocity_body;

    // Note: angular_rate_body is an opaque, already bias-corrected input
    // supplied by the caller (see header doc) rather than being recomputed
    // here from state.nominal.gyro_bias, so this measurement model has no
    // algebraic dependency on the gyro-bias error state dbg -- H's
    // GYRO_BIAS columns are correctly all-zero (perturbing dbg alone does
    // not change h() as actually implemented; a finite-difference Jacobian
    // check against this exact function confirms it).
    FusionMeasurementSystem system;
    system.design_matrix = Eigen::MatrixXd::Zero(3, fusion_index::SIZE);
    system.design_matrix.block<3, 3>(0, fusion_index::VELOCITY) = Eigen::Matrix3d::Identity();
    system.design_matrix.block<3, 3>(0, fusion_index::ATTITUDE) =
        -rotation * attitude::skew(lever_velocity_body);
    system.residuals = antenna_velocity_enu - predicted;
    system.covariance = velocity_covariance_enu;
    return system;
}

FusionMeasurementSystem buildZuptUpdate(const FusionState& state, double sigma_mps) {
    FusionMeasurementSystem system;
    system.design_matrix = Eigen::MatrixXd::Zero(3, fusion_index::SIZE);
    system.design_matrix.block<3, 3>(0, fusion_index::VELOCITY) = Eigen::Matrix3d::Identity();
    system.residuals = -state.nominal.velocity_enu;
    system.covariance = (sigma_mps * sigma_mps) * Eigen::Matrix3d::Identity();
    return system;
}

FusionMeasurementSystem buildNhcUpdate(const FusionState& state,
                                       const Eigen::Vector3d& lever_arm_body,
                                       const Eigen::Vector3d& angular_rate_body,
                                       double sigma_lateral_mps, double sigma_vertical_mps) {
    const Eigen::Matrix3d rotation = state.nominal.attitude_body_to_enu.toRotationMatrix();
    const Eigen::Vector3d velocity_body_nominal =
        rotation.transpose() * state.nominal.velocity_enu + angular_rate_body.cross(lever_arm_body);

    // Full 3x3 Jacobian blocks of v_body = R^T*v_enu + (omega x lever) w.r.t.
    // [dv, dtheta]; only the lateral (row 1) and vertical (row 2) rows are
    // kept below (NHC does not constrain the forward component).
    // angular_rate_body is an opaque, already bias-corrected input (see
    // buildGnssVelocityUpdate's comment above) -- no GYRO_BIAS column here
    // for the same reason.
    const Eigen::Matrix3d d_dv = rotation.transpose();
    const Eigen::Matrix3d d_dtheta =
        attitude::skew(rotation.transpose() * state.nominal.velocity_enu);

    FusionMeasurementSystem system;
    system.design_matrix = Eigen::MatrixXd::Zero(2, fusion_index::SIZE);
    system.design_matrix.block<1, 3>(0, fusion_index::VELOCITY) = d_dv.row(1);
    system.design_matrix.block<1, 3>(1, fusion_index::VELOCITY) = d_dv.row(2);
    system.design_matrix.block<1, 3>(0, fusion_index::ATTITUDE) = d_dtheta.row(1);
    system.design_matrix.block<1, 3>(1, fusion_index::ATTITUDE) = d_dtheta.row(2);

    system.residuals = Eigen::VectorXd(2);
    system.residuals(0) = -velocity_body_nominal(1);  // lateral (Left)
    system.residuals(1) = -velocity_body_nominal(2);  // vertical (Up)

    system.covariance = Eigen::MatrixXd::Zero(2, 2);
    system.covariance(0, 0) = sigma_lateral_mps * sigma_lateral_mps;
    system.covariance(1, 1) = sigma_vertical_mps * sigma_vertical_mps;
    return system;
}

}  // namespace fusion_measurement
}  // namespace libgnss
