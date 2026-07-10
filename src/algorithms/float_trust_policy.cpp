#include "libgnss++/algorithms/float_trust_policy.hpp"

#include <algorithm>
#include <cmath>

namespace libgnss {
namespace float_trust_policy {

bool hasTrustLapsed(bool has_last_trusted, bool trust_refreshed_last_epoch) {
    return !has_last_trusted || !trust_refreshed_last_epoch;
}

double growPositionVarianceCvPredict(double previous_var_m2,
                                      double qpos_m2_per_s,
                                      double dt_s,
                                      double legacy_var_m2) {
    const double safe_legacy = std::isfinite(legacy_var_m2) ? std::max(legacy_var_m2, 0.0) : 900.0;
    const double safe_prev = std::isfinite(previous_var_m2)
        ? std::max(previous_var_m2, 0.0)
        : safe_legacy;
    const double safe_qpos = std::isfinite(qpos_m2_per_s) ? std::max(qpos_m2_per_s, 0.0) : 0.0;
    const double safe_dt = std::isfinite(dt_s) ? std::max(dt_s, 0.0) : 0.0;
    const double grown = safe_prev + safe_qpos * safe_dt;
    return std::min(grown, safe_legacy);
}

double scaledResetPositionVariance(double base_var_m2,
                                    double qpos_m2_per_s,
                                    double dt_since_trust_s,
                                    double legacy_var_m2) {
    const double safe_legacy = std::isfinite(legacy_var_m2) ? std::max(legacy_var_m2, 0.0) : 900.0;
    const double safe_base = std::isfinite(base_var_m2) ? std::max(base_var_m2, 0.0) : 0.0;
    const double safe_qpos = std::isfinite(qpos_m2_per_s) ? std::max(qpos_m2_per_s, 0.0) : 0.0;
    const double safe_dt = std::isfinite(dt_since_trust_s) ? std::max(dt_since_trust_s, 0.0) : 0.0;
    const double grown = safe_base + safe_qpos * safe_dt * safe_dt;
    return std::min(grown, safe_legacy);
}

Eigen::Vector3d estimateVelocityFromTrustedDeltas(const Eigen::Vector3d& newer_position,
                                                   const Eigen::Vector3d& older_position,
                                                   double dt_s,
                                                   double max_dt_s) {
    if (!std::isfinite(dt_s) || dt_s <= 0.0 || dt_s > max_dt_s ||
        !newer_position.allFinite() || !older_position.allFinite()) {
        return Eigen::Vector3d::Zero();
    }
    return (newer_position - older_position) / dt_s;
}

Eigen::Vector3d predictPositionConstantVelocity(const Eigen::Vector3d& previous_position,
                                                 const Eigen::Vector3d& velocity_mps,
                                                 double dt_s) {
    if (!previous_position.allFinite() || !velocity_mps.allFinite() || !std::isfinite(dt_s)) {
        return previous_position;
    }
    return previous_position + velocity_mps * dt_s;
}

bool lapseGateExceeded(double dt_since_trust_s, double gate_s) {
    const double safe_dt = std::isfinite(dt_since_trust_s) ? std::max(dt_since_trust_s, 0.0) : 0.0;
    const double safe_gate = std::isfinite(gate_s) ? std::max(gate_s, 0.0) : 0.0;
    return safe_dt >= safe_gate;
}

}  // namespace float_trust_policy
}  // namespace libgnss
