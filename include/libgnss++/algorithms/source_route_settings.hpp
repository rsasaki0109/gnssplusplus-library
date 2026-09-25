#pragma once
#include <string>

namespace libgnss::smartphone_temporal {
struct SourceRouteSettings {
    bool valid = false;
    double pseudorange_huber = 0, doppler_huber = 0, tdcp_huber = 0;
    double elevation_deg = 0, velocity_motion_sigma_m = 0;
};

// Pinned taroz parameters.m: explicit setting.Type/L5, with Pixel4 Doppler
// and Mi8 motion exceptions. No route-name or accuracy-based classification.
inline SourceRouteSettings sourceRouteSettings(
    const std::string& type, const std::string& phone, int l5) {
    if ((type != "Highway" && type != "Street" && type != "Mix") ||
        phone.empty() || (l5 != 0 && l5 != 1)) return {};
    SourceRouteSettings r;
    r.valid = true;
    const bool urban = type == "Street" || type == "Mix";
    r.pseudorange_huber = urban ? 0.1 : 0.2;
    r.doppler_huber = phone.find("pixel4") != std::string::npos ? 0.2 : urban ? 0.4 : 0.8;
    r.tdcp_huber = urban ? 0.2 : 0.5;
    r.elevation_deg = l5 ? 5.0 : 10.0;
    r.velocity_motion_sigma_m = phone == "mi8" || phone == "xiaomimi8" ? 0.1
                              : type == "Street" ? 0.05 : 0.01;
    return r;
}
}  // namespace libgnss::smartphone_temporal
