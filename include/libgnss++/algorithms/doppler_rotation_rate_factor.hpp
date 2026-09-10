#pragma once

#include <libgnss++/algorithms/doppler_rotation_rate.hpp>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <stdexcept>

namespace libgnss::doppler_rotation_rate {

// Research-only ECEF factor, not wired into production graph selection.
// Observed rate and satellite clock drift are both m/s. Keep measurement
// noise in its original units: do not normalize the velocity Jacobian or
// multiply the observation equation by the geometric denominator.
class EcefFactor final : public gtsam::NoiseModelFactorN<gtsam::Vector3, gtsam::Vector> {
    Model geometry_;
    double receiver_only_mps_;
public:
    using Base = gtsam::NoiseModelFactorN<gtsam::Vector3, gtsam::Vector>;
    using Base::evaluateError;

    EcefFactor(gtsam::Key velocity, gtsam::Key clock,
               const Vector3d& rotated_satellite, const Vector3d& rotated_velocity,
               const Vector3d& receiver, double observed_mps,
               double satellite_clock_drift_mps, const gtsam::SharedNoiseModel& noise)
        : Base(noise, velocity, clock), geometry_{}, receiver_only_mps_(0.) {
        const auto geometry = make(rotated_satellite, rotated_velocity, receiver);
        if (!geometry || !noise || noise->dim() != 1 || velocity == clock ||
            !std::isfinite(observed_mps) || !std::isfinite(satellite_clock_drift_mps))
            throw std::invalid_argument("Invalid rotation-rate Doppler factor");
        geometry_ = *geometry;
        receiver_only_mps_ = observed_mps - geometry_.satellite_mps + satellite_clock_drift_mps;
        if (!std::isfinite(receiver_only_mps_))
            throw std::invalid_argument("Nonfinite rotation-rate Doppler observation");
    }

    gtsam::Vector evaluateError(const gtsam::Vector3& velocity,
                               const gtsam::Vector& clock,
                               gtsam::OptionalMatrixType H_velocity,
                               gtsam::OptionalMatrixType H_clock) const override {
        if (!velocity.allFinite() || clock.size() != 1 || !clock.allFinite())
            throw std::invalid_argument("Invalid rotation-rate Doppler state");
        if (H_velocity) *H_velocity = geometry_.receiver_velocity_jacobian.transpose();
        if (H_clock) *H_clock = gtsam::Matrix::Ones(1, 1);
        return gtsam::Vector::Constant(1,
            geometry_.receiver_velocity_jacobian.dot(velocity) + clock(0) - receiver_only_mps_);
    }
};
}  // namespace libgnss::doppler_rotation_rate
