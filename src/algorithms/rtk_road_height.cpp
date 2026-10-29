#include <libgnss++/algorithms/rtk.hpp>
#include <libgnss++/core/coordinates.hpp>

#include <cmath>

#include "rtk_internal.hpp"

namespace libgnss {

using namespace rtk_internal;

namespace {

Vector3d upVector(double lat, double lon) {
    return Vector3d(std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat));
}

}  // namespace

void RTKProcessor::applyRoadHeightPrior(bool position_block_only) {
    debug_telemetry_.road_height_applied = false;
    const double sigma = rtk_config_.road_height_sigma_m;
    if (!(sigma > 0.0) || !std::isfinite(sigma) || !road_height_map_ || road_height_map_->empty() ||
        !isDynamicPositionMode(rtk_config_) || isMovingBasePositionMode(rtk_config_) ||
        filter_state_.state.size() < 3) {
        return;
    }
    const Vector3d rover = base_position_ + filter_state_.state.head<3>();
    const auto road =
        road_height::lookupRoadSurface(*road_height_map_, rover, rtk_config_.road_height_radius_m);
    if (!road) return;

    double lat = 0.0, lon = 0.0, height = 0.0;
    ecef2geodetic(rover, lat, lon, height);
    const double innovation = road->height_m + rtk_config_.road_height_offset_m - height;
    if (!std::isfinite(innovation)) return;

    // A horizontal error moves the looked-up road height by gradient . d_en.
    const Vector3d east(-std::sin(lon), std::cos(lon), 0.0);
    const Vector3d north(-std::sin(lat) * std::cos(lon), -std::sin(lat) * std::sin(lon),
                         std::cos(lat));
    const Vector3d slope_ecef = road->gradient.x() * east + road->gradient.y() * north;
    const Matrix3d p_pos = filter_state_.covariance.topLeftCorner<3, 3>();
    const double variance = sigma * sigma + slope_ecef.dot(p_pos * slope_ecef);

    // h(x) is the ellipsoidal height; its gradient is the local up vector.
    const Vector3d up = upVector(lat, lon);
    if (position_block_only) {
        const Vector3d ph = p_pos * up;
        const double s = up.dot(ph) + variance;
        if (!(s > 0.0) || !std::isfinite(s)) return;
        filter_state_.state.head<3>() += ph / s * innovation;
        filter_state_.covariance.topLeftCorner<3, 3>() -= ph * ph.transpose() / s;
    } else {
        const int n = static_cast<int>(filter_state_.state.size());
        VectorXd h_row = VectorXd::Zero(n);
        h_row.head<3>() = up;
        const VectorXd ph = filter_state_.covariance * h_row;
        const double s = h_row.dot(ph) + variance;
        if (!(s > 0.0) || !std::isfinite(s)) return;
        const VectorXd gain = ph / s;
        filter_state_.state += gain * innovation;
        filter_state_.covariance -= gain * ph.transpose();
        filter_state_.covariance =
            (filter_state_.covariance + filter_state_.covariance.transpose()) / 2.0;
    }

    debug_telemetry_.road_height_applied = true;
    debug_telemetry_.road_height_innovation_m = innovation;
    debug_telemetry_.road_height_sigma_m = std::sqrt(variance);
}

}  // namespace libgnss
