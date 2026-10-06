#pragma once

#include <libgnss++/fusion/online_rtk_imu.hpp>
#include <iomanip>
#include <ostream>

namespace libgnss {
// Append-only online CSV schema v2. The original 27 columns are unchanged.
inline constexpr const char* kOnlinePvaCsvHeader =
    "rover_week,rover_tow,received_week,received_tow,input_age_s,exact_base,imu_consumed,reset_generation,"
    "fusion_initialized,heading_converged,gnss_position_updated,tight_update,fusion_age_s,processing_ms,reason,"
    "rtk_status,rtk_week,rtk_tow,rtk_x_m,rtk_y_m,rtk_z_m,fused_status,fused_week,fused_tow,fused_x_m,fused_y_m,fused_z_m,"
    "rtk_has_velocity,rtk_vx_mps,rtk_vy_mps,rtk_vz_mps,fused_has_velocity,fused_vx_mps,fused_vy_mps,fused_vz_mps,"
    "attitude_available,heading_aligned,attitude_week,attitude_tow,qw,qx,qy,qz,roll_deg,pitch_deg,heading_deg,"
    "ecef_to_enu_00,ecef_to_enu_01,ecef_to_enu_02,ecef_to_enu_10,ecef_to_enu_11,ecef_to_enu_12,"
    "ecef_to_enu_20,ecef_to_enu_21,ecef_to_enu_22";

inline void writeOnlinePvaCsv(std::ostream& stream, const GNSSTime& rover,
                              const OnlineRtkImuProcessor::Output& row) {
    stream << std::setprecision(17) << rover.week << ',' << rover.tow << ','
        << row.received_at.week << ',' << row.received_at.tow << ',' << row.input_age_s << ','
        << row.exact_base_available << ',' << row.imu_consumed << ',' << row.reset_generation << ','
        << row.fusion_initialized << ',' << row.heading_converged << ',' << row.gnss_position_updated << ','
        << row.tight_time_update_supplied << ',' << row.fusion_age_s << ',' << row.processing_ms << ',' << row.reason;
    for (const auto* solution : {&row.rtk, &row.fused}) {
        stream << ',' << static_cast<int>(solution->status) << ',' << solution->time.week << ',' << solution->time.tow;
        if (solution->isValid()) stream << ',' << solution->position_ecef.x() << ','
            << solution->position_ecef.y() << ',' << solution->position_ecef.z();
        else stream << ",nan,nan,nan";
    }
    for (const auto* solution : {&row.rtk, &row.fused}) {
        const bool valid = solution->isValid() && solution->has_velocity && solution->velocity_ecef.allFinite();
        stream << ',' << valid;
        if (valid) stream << ',' << solution->velocity_ecef.x() << ',' << solution->velocity_ecef.y()
                          << ',' << solution->velocity_ecef.z();
        else stream << ",nan,nan,nan";
    }
    stream << ',' << row.attitude_available << ',' << row.heading_aligned << ','
        << row.attitude_time.week << ',' << row.attitude_time.tow << ','
        << row.attitude_body_to_enu.w() << ',' << row.attitude_body_to_enu.x() << ','
        << row.attitude_body_to_enu.y() << ',' << row.attitude_body_to_enu.z() << ','
        << row.rpy_frd_ned_deg.x() << ',' << row.rpy_frd_ned_deg.y() << ',' << row.rpy_frd_ned_deg.z();
    for (int r = 0; r < 3; ++r) for (int c = 0; c < 3; ++c)
        stream << ',' << row.ecef_to_attitude_enu(r, c);
    stream << '\n';
}
} // namespace libgnss
