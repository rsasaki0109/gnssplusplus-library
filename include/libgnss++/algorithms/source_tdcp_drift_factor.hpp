#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/GnssCommon.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>
#include <cmath>
#include <stdexcept>
#include <type_traits>

namespace libgnss::smartphone_temporal {
// Affine TDCP_XXDD equation from taroz/gtsam_gnss revision e679b72.
// The anchor and range-change subtraction belong to graph construction.
// D states are one-element vectors in m/s; measurement and offset are in m.
// Pose3 maps to the antenna in ECEF using the same arm as other GNSS rows.
template <class Position>
class DriftTdcpFactor
    : public gtsam::NoiseModelFactorN<Position, Position,
                                      gtsam::Vector, gtsam::Vector> {
    using Base = gtsam::NoiseModelFactorN<Position, Position,
                                         gtsam::Vector, gtsam::Vector>;
    gtsam::Vector3 los_;
    gtsam::Point3 anchor1_, anchor2_;
    double measurement_, half_dt_;
    gtsam::gnss::LeverArm arm_;

 public:
    using Base::evaluateError;
    DriftTdcpFactor(gtsam::Key x1, gtsam::Key x2,
                    gtsam::Key d1, gtsam::Key d2,
                    const gtsam::Vector3& los,
                    const gtsam::Point3& anchor1,
                    const gtsam::Point3& anchor2,
                    double measurement_m, double dt_s,
                    const gtsam::SharedNoiseModel& noise,
                    const gtsam::gnss::LeverArm& arm = {})
        : Base(noise, x1, x2, d1, d2), los_(los),
          anchor1_(anchor1), anchor2_(anchor2),
          measurement_(measurement_m), half_dt_(0.5 * dt_s), arm_(arm) {
        if (!los.allFinite() || std::abs(los.norm() - 1.0) > 1e-6 ||
            !anchor1.allFinite() || !anchor2.allFinite() ||
            !std::isfinite(measurement_m) ||
            !std::isfinite(dt_s) || dt_s <= 0.0) {
            throw std::invalid_argument("Invalid affine drift TDCP geometry or time");
        }
    }

    gtsam::Vector evaluateError(
        const Position& x1, const Position& x2,
        const gtsam::Vector& d1, const gtsam::Vector& d2,
        gtsam::OptionalMatrixType H1, gtsam::OptionalMatrixType H2,
        gtsam::OptionalMatrixType HD1,
        gtsam::OptionalMatrixType HD2) const override {
        if (d1.size() != 1 || d2.size() != 1 ||
            !d1.allFinite() || !d2.allFinite()) {
            throw std::invalid_argument("Drift TDCP requires finite scalar-vector m/s states");
        }
        gtsam::Point3 p1, p2;
        if constexpr (std::is_same_v<Position, gtsam::Pose3>) {
            gtsam::gnss::LeverArm::PoseFrame f1, f2;
            p1 = arm_.antennaPosition(x1, H1 ? &f1 : nullptr);
            p2 = arm_.antennaPosition(x2, H2 ? &f2 : nullptr);
            if (H1) *H1 = arm_.antennaPoseJacobian(-los_.transpose(), f1);
            if (H2) *H2 = arm_.antennaPoseJacobian(los_.transpose(), f2);
        } else {
            static_assert(std::is_same_v<Position, gtsam::Point3>);
            p1 = x1;
            p2 = x2;
            if (H1) *H1 = -los_.transpose();
            if (H2) *H2 = los_.transpose();
        }
        if (HD1) *HD1 = gtsam::Matrix::Constant(1, 1, half_dt_);
        if (HD2) *HD2 = gtsam::Matrix::Constant(1, 1, half_dt_);
        return gtsam::Vector::Constant(1,
            los_.dot((p2 - anchor2_) - (p1 - anchor1_)) +
            half_dt_ * (d1[0] + d2[0]) - measurement_);
    }
};
using DriftTdcpPointFactor = DriftTdcpFactor<gtsam::Point3>;
using DriftTdcpPoseFactor = DriftTdcpFactor<gtsam::Pose3>;
}  // namespace libgnss::smartphone_temporal
