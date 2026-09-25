#pragma once

#include <libgnss++/algorithms/native_gnss_imu_initial_handoff.hpp>
#include <libgnss++/algorithms/upstream_position_offset.hpp>
#include <libgnss++/fusion/fusion_initialization.hpp>

namespace libgnss::native_imu_refinement {

struct PositionOffsetHandoff {
    Handoff states;
    std::vector<Vector3d> displacement_enu_m;
};

namespace stage_offset_detail {
inline PositionOffsetHandoff apply(Handoff states,
                                  const std::vector<Vector3d>& rpy,
                                  std::string_view phone, double latitude,
                                  double longitude) {
    if (states.observations.empty() || rpy.size() != states.observations.size())
        throw std::invalid_argument("Stage position offset requires complete attitudes");
    PositionOffsetHandoff out{std::move(states), {}};
    for (std::size_t i = 0; i < rpy.size(); ++i) {
        const auto offset = upstream_position_offset::offsetFromRpy(phone, rpy[i]);
        if (!offset.ok)
            throw std::invalid_argument("Stage position offset has invalid phone or attitude");
        auto& position = out.states.observations[i].receiver_position;
        position += enu2ecef(offset.offset_enu_m, latitude, longitude);
        if (!position.allFinite() || position.norm() < 1e6 || position.norm() > 1e8)
            throw std::invalid_argument("Stage position offset produced invalid position");
        out.displacement_enu_m.push_back(offset.offset_enu_m);
    }
    return out;
}
}  // namespace stage_offset_detail

// The pinned MATLAB scripts save a corrected position at each stage. Apply
// that correction to a fresh validated handoff, never to the original result.
// Velocities, C7/D, body attitudes and raw observables are unchanged. Final
// output correction is separate and must operate on the next optimized result.
inline PositionOffsetHandoff fromResultWithPositionOffset(
    const std::vector<ObservationData>& raw,
    const FGOProcessor::FGOProblem& source,
    const FGOProcessor::FGOResult& result, std::string_view phone) {
    return stage_offset_detail::apply(fromResult(raw, source, result),
        result.epoch_attitude_rpy_rad, phone,
        source.imu.nav_origin_lat_rad, source.imu.nav_origin_lon_rad);
}

inline PositionOffsetHandoff fromGnssResultWithPositionOffset(
    const std::vector<ObservationData>& raw,
    const FGOProcessor::FGOProblem& initialized_imu,
    const FGOProcessor::FGOResult& gnss, std::string_view phone) {
    auto states = fromGnssResult(raw, initialized_imu, gnss);
    std::vector<Vector3d> velocities;
    for (const auto& velocity : states.velocity_ecef_mps)
        velocities.push_back(ecef2enu(velocity,
            initialized_imu.imu.nav_origin_lat_rad,
            initialized_imu.imu.nav_origin_lon_rad));
    fusion_initialization::VelocityHeadingConfig config;
    config.nearest_fill_interior = true;
    const auto heading = fusion_initialization::velocityToRpy(velocities, config);
    if (!heading.ok || heading.linear_fill_count != 0)
        throw std::invalid_argument("GNSS stage position offset has no velocity heading");
    return stage_offset_detail::apply(std::move(states), heading.rpy_rad, phone,
        initialized_imu.imu.nav_origin_lat_rad,
        initialized_imu.imu.nav_origin_lon_rad);
}
}  // namespace libgnss::native_imu_refinement
