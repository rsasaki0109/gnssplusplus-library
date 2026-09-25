#pragma once

#include <optional>
#include <string_view>

namespace libgnss::source_phone_imu_noise {

struct Selection {
    double accel_noise_sigma;
    double gyro_noise_sigma;
    double sync_coefficient;
    const char* source;
};

// taroz/gsdc2023 29923f9: parameters.m and functions/imuprocessing.m.
// Preserve the literal sm-g325f spelling and branch order of that source.
inline std::optional<Selection> select(std::string_view phone, bool utc_fallback) {
    const auto has = [phone](std::string_view part) {
        return phone.find(part) != std::string_view::npos;
    };
    double accel = 0.05;
    double gyro = 0.001;
    const char* source = nullptr;
    if (has("pixel")) {
        source = "source-phone-pixel";
    } else if (has("sm-s908") || has("sm-g988") || has("sm-g325f") || has("samsun")) {
        source = "source-phone-explicit-samsung";
    } else if (has("sm-a217m")) {
        accel = 0.1;
        gyro = 0.005;
        source = "source-phone-sm-a217m";
    } else if (has("sm")) {
        accel = 0.1;
        source = "source-phone-other-sm";
    } else if (has("mi")) {
        source = "source-phone-mi";
    } else {
        return std::nullopt;
    }
    const double coefficient = utc_fallback ? 1.0 : 0.5;
    return Selection{accel * coefficient, gyro * coefficient, coefficient, source};
}

}  // namespace libgnss::source_phone_imu_noise
