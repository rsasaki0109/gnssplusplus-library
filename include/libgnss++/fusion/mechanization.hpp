#pragma once

#include <Eigen/Dense>

#include <libgnss++/fusion/fusion_types.hpp>
#include <libgnss++/io/imu.hpp>

namespace libgnss {
namespace mechanization {

/**
 * @brief Strapdown INS mechanization: propagate the nominal state forward by
 * one IMU sample interval.
 *
 * Same physics as reference_notes.md 3.1's mid-point strapdown scheme:
 * bias-correct the raw accel/gyro -> mid-point quaternion increment -> rotate
 * specific force into ENU using the mid-point attitude -> trapezoidal
 * position update. A pure function of its inputs, unit-testable in isolation
 * (no processor object involved), exactly like kalman.hpp's free functions.
 *
 * @param prev            Nominal state at the start of the interval
 * @param sample_body_flu IMU sample already rotated into body FLU (via
 *                        ImuAxisConvention::apply()) -- NOT raw sensor axes
 * @param dt              Time step, seconds (sample_body_flu.time - prev.time)
 * @param gravity_enu     Local ENU gravity vector, e.g. (0, 0, -9.80665)
 * @return                Nominal state at the end of the interval
 */
NominalState propagate(const NominalState& prev, const ImuSample& sample_body_flu,
                       double dt, const Eigen::Vector3d& gravity_enu);

}  // namespace mechanization
}  // namespace libgnss
