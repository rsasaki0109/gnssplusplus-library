#include <libgnss++/fusion/mechanization.hpp>

#include <libgnss++/fusion/attitude.hpp>

namespace libgnss {
namespace mechanization {

NominalState propagate(const NominalState& prev, const ImuSample& sample_body_flu,
                       double dt, const Eigen::Vector3d& gravity_enu) {
    NominalState next = prev;
    next.time = prev.time + dt;
    if (dt <= 0.0) {
        return next;
    }

    // Bias-correct the raw body-frame accel/gyro.
    const Eigen::Vector3d specific_force_body = sample_body_flu.accel_raw - prev.accel_bias;
    const Eigen::Vector3d angular_rate_body = sample_body_flu.gyro_raw_radps - prev.gyro_bias;
    const Eigen::Vector3d dtheta = angular_rate_body * dt;

    // Mid-point attitude: half the rotation increment applied to the
    // starting attitude, used only to rotate the specific force -- the full
    // increment is applied separately to advance the nominal attitude.
    const Eigen::Quaterniond dq_half = attitude::smallAngleQuaternion(0.5 * dtheta);
    const Eigen::Quaterniond q_mid = (prev.attitude_body_to_enu * dq_half).normalized();

    const Eigen::Quaterniond dq_full = attitude::smallAngleQuaternion(dtheta);
    next.attitude_body_to_enu = (prev.attitude_body_to_enu * dq_full).normalized();

    const Eigen::Vector3d specific_force_enu = q_mid * specific_force_body;
    const Eigen::Vector3d accel_enu = specific_force_enu + gravity_enu;

    // Trapezoidal position update (average of start/end velocity), velocity
    // updated with the mid-point-rotated specific force.
    next.velocity_enu = prev.velocity_enu + accel_enu * dt;
    next.position_enu =
        prev.position_enu + prev.velocity_enu * dt + 0.5 * accel_enu * dt * dt;

    // Biases evolve only through the error-state random walk (process
    // noise), not through deterministic mechanization.
    next.accel_bias = prev.accel_bias;
    next.gyro_bias = prev.gyro_bias;

    return next;
}

}  // namespace mechanization
}  // namespace libgnss
