#pragma once

#include <string>
#include <cstddef>
#include <cmath>

namespace libgnss::smartphone_temporal {
enum class CarrierClock { BiasDifference, IntegratedDrift, Disabled };
struct Recipe {
    bool clock_between = true;
    CarrierClock carrier_clock = CarrierClock::BiasDifference;
    double carrier_offset_m = 0.0;
};
// Shared by every phone, independently of the TDCP clock model. Source
// fgo_gnss.m uses UTC dt and a strict time_diff_th=1.5 s motion/clock gate.
inline bool motionEdgeEligible(double utc_dt_s) {
    return std::isfinite(utc_dt_s) && utc_dt_s > 0.0 && utc_dt_s < 1.5;
}
// taroz/gsdc2023 fgo_gnss.m and fgo_gnss_imu.m, revision 29923f9.
// This describes the published model; it does not authorize incomplete
// state coverage or alter the raw clock-discontinuity checks.
inline Recipe forPhone(const std::string& phone) {
    Recipe r;
    r.clock_between = phone != "sm-a205u" && phone != "sm-a505u" &&
                      phone != "samsunga325g";
    if (phone == "sm-a325f" || phone == "samsunga32") {
        r.carrier_clock = CarrierClock::Disabled;
    } else if (phone == "sm-a205u" || phone == "sm-a217m" ||
               phone == "sm-a505g" || phone == "sm-a600t" ||
               phone == "sm-a505u" || phone == "samsunga325g") {
        r.carrier_clock = CarrierClock::IntegratedDrift;
        r.carrier_offset_m = phone == "samsunga325g" ? 0.0 : 1.117;
    }
    return r;
}
// Validate the selected graph, rather than requiring a clock edge from
// phones whose published model intentionally omits every such edge.
inline bool clockEdgeCountMatches(bool native_phone_recipe,
                                  const std::string& phone,
                                  std::size_t count) {
    if (native_phone_recipe && !phone.empty() && !forPhone(phone).clock_between)
        return count == 0;
    return count > 0;
}
}  // namespace libgnss::smartphone_temporal
