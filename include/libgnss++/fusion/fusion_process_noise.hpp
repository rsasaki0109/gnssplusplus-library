#pragma once

#include <Eigen/Dense>

#include <libgnss++/fusion/fusion_types.hpp>

namespace libgnss {

/**
 * @brief Continuous-time IMU noise spectral densities and bias random-walk
 * rates.
 *
 * Defaults match the reference's tactical-grade IMU preset
 * (reference_notes.md 7 table) so a dataset tuned against both tools doesn't
 * need noise numbers re-derived from scratch.
 */
struct ProcessNoiseConfig {
    double accel_noise_density = 2.84e-4;      ///< m/s^2/sqrt(Hz)
    double gyro_noise_density = 4.01e-5;       ///< rad/s/sqrt(Hz)
    double accel_bias_random_walk = 3.14e-4;   ///< m/s^2/sqrt(Hz) (bias random-walk rate)
    double gyro_bias_random_walk = 9.70e-6;    ///< rad/s/sqrt(Hz) (bias random-walk rate)
};

namespace fusion_process_noise {

/**
 * @brief Discrete error-state transition matrix Phi = I + Fc*dt + (Fc*dt)^2/2.
 *
 * Fc is the continuous-time error-state dynamics matrix: skew-based coupling
 * of specific force into d(dv)/dt via d(dtheta), angular rate into
 * d(dtheta)/dt, and direct -R / -I coupling of the accel/gyro biases (biases
 * themselves are modeled as pure random walks, i.e. zero deterministic
 * dynamics beyond the noise term captured in processNoiseCovariance()).
 *
 * @param state                Current nominal state (uses attitude_body_to_enu)
 * @param specific_force_body  Bias-corrected accel measurement, body frame (m/s^2)
 * @param angular_rate_body    Bias-corrected gyro measurement, body frame (rad/s)
 * @param dt                   Time step, seconds
 * @return                     15x15 discrete transition matrix Phi
 */
Eigen::Matrix<double, 15, 15> transitionMatrix(const NominalState& state,
                                               const Eigen::Vector3d& specific_force_body,
                                               const Eigen::Vector3d& angular_rate_body,
                                               double dt);

/**
 * @brief Discretized process noise covariance Qd ~= Phi (G Qc G^T) Phi^T dt.
 *
 * G Qc G^T is independent of the current attitude: the accel/gyro noise
 * terms enter the velocity/attitude error derivatives through -R / -I
 * couplings, but because the continuous-time noise is isotropic
 * (sigma^2 * I3 in the body frame) and R is a rotation matrix,
 * R (sigma^2 I3) R^T = sigma^2 I3 regardless of R -- so the noise-injection
 * block is the same constant block-diagonal matrix in every frame, and
 * transitionMatrix()'s Phi alone carries the necessary rotation/coupling
 * information into the discretized result.
 *
 * @param phi  Discrete transition matrix from transitionMatrix()
 * @param cfg  Continuous-time noise spectral densities
 * @param dt   Time step, seconds
 * @return     15x15 discretized process noise covariance Qd
 */
Eigen::Matrix<double, 15, 15> processNoiseCovariance(const Eigen::Matrix<double, 15, 15>& phi,
                                                     const ProcessNoiseConfig& cfg,
                                                     double dt);

}  // namespace fusion_process_noise
}  // namespace libgnss
