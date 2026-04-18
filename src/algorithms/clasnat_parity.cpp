#include <libgnss++/algorithms/clasnat_parity.hpp>
#include <libgnss++/algorithms/tidal.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/models/troposphere.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <ctime>

namespace libgnss::clasnat_parity {
namespace {

constexpr double kDegreesToRadians = M_PI / 180.0;
constexpr double kRadiansToDegrees = 180.0 / M_PI;
constexpr double kIonosphereHeightM = 350000.0;

double dot3(const double a[3], const double b[3]) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

double norm3(const double a[3]) {
    return std::sqrt(dot3(a, a));
}

bool unit3(const double a[3], double out[3]) {
    const double n = norm3(a);
    if (n <= 0.0) {
        return false;
    }
    for (int i = 0; i < 3; ++i) {
        out[i] = a[i] / n;
    }
    return true;
}

void cross3(const double a[3], const double b[3], double out[3]) {
    out[0] = a[1] * b[2] - a[2] * b[1];
    out[1] = a[2] * b[0] - a[0] * b[2];
    out[2] = a[0] * b[1] - a[1] * b[0];
}

double interpPcvVariation(const std::array<double, kParityPcvGridSize>& var,
                          double zenith_deg) {
    const double a = zenith_deg / 5.0;
    const int i = static_cast<int>(a);
    if (i < 0) {
        return var[0];
    }
    if (i >= kParityPcvGridSize - 1) {
        return var[kParityPcvGridSize - 1];
    }
    return var[static_cast<size_t>(i)] * (1.0 - a + i) +
           var[static_cast<size_t>(i + 1)] * (a - i);
}

int dayOfYearFromTime(const GNSSTime& time) {
    const auto system_time = time.toSystemTime();
    const std::time_t utc_seconds = std::chrono::system_clock::to_time_t(system_time);
    std::tm utc_tm{};
    gmtime_r(&utc_seconds, &utc_tm);
    return utc_tm.tm_yday + 1;
}

double embeddedGeoidHeightJapan(double latitude_rad, double longitude_rad) {
    constexpr int lon_min_deg = 120;
    constexpr int lon_max_deg = 150;
    constexpr int lat_min_deg = 20;
    constexpr int lat_max_deg = 50;
    constexpr double geoid[lon_max_deg - lon_min_deg + 1][lat_max_deg - lat_min_deg + 1] = {
#include "clas_embedded_geoid_japan.inc"
    };

    const double latitude_deg = latitude_rad * kRadiansToDegrees;
    const double longitude_deg = longitude_rad * kRadiansToDegrees;
    if (longitude_deg < lon_min_deg || longitude_deg > lon_max_deg ||
        latitude_deg < lat_min_deg || latitude_deg > lat_max_deg) {
        return 0.0;
    }

    const double lon_offset = longitude_deg - lon_min_deg;
    const double lat_offset = latitude_deg - lat_min_deg;
    const int lon0 = std::clamp(static_cast<int>(std::floor(lon_offset)),
                                0,
                                lon_max_deg - lon_min_deg);
    const int lat0 = std::clamp(static_cast<int>(std::floor(lat_offset)),
                                0,
                                lat_max_deg - lat_min_deg);
    const int lon1 = std::min(lon0 + 1, lon_max_deg - lon_min_deg);
    const int lat1 = std::min(lat0 + 1, lat_max_deg - lat_min_deg);
    const double a = lon_offset - lon0;
    const double b = lat_offset - lat0;

    return geoid[lon0][lat0] * (1.0 - a) * (1.0 - b) +
           geoid[lon1][lat0] * a * (1.0 - b) +
           geoid[lon0][lat1] * (1.0 - a) * b +
           geoid[lon1][lat1] * a * b;
}

double interpMetCoefficient(const std::array<double, 5>& values, double latitude_deg) {
    const int i = static_cast<int>(latitude_deg / 15.0);
    if (i < 1) {
        return values[0];
    }
    if (i > 4) {
        return values[4];
    }
    return values[static_cast<size_t>(i - 1)] * (1.0 - latitude_deg / 15.0 + i) +
           values[static_cast<size_t>(i)] * (latitude_deg / 15.0 - i);
}

bool standardVerticalTroposphereDelays(const GNSSTime& time,
                                       const double pos[3],
                                       double& hydrostatic_m,
                                       double& wet_m) {
    constexpr std::array<double, 5> mean_pressure = {
        1013.25, 1017.25, 1015.75, 1011.75, 1013.00};
    constexpr std::array<double, 5> mean_temperature = {
        299.65, 294.15, 283.15, 272.15, 263.65};
    constexpr std::array<double, 5> mean_water_pressure = {
        26.31, 21.79, 11.66, 6.78, 4.11};
    constexpr std::array<double, 5> mean_lapse_rate = {
        6.30e-3, 6.05e-3, 5.58e-3, 5.39e-3, 4.53e-3};
    constexpr std::array<double, 5> mean_water_vapor_rate = {
        2.77, 3.15, 2.57, 1.81, 1.55};
    constexpr std::array<double, 5> amp_pressure = {
        0.00, -3.75, -2.25, -1.75, -0.50};
    constexpr std::array<double, 5> amp_temperature = {
        0.00, 7.00, 11.00, 15.00, 14.50};
    constexpr std::array<double, 5> amp_water_pressure = {
        0.00, 8.85, 7.24, 5.36, 3.39};
    constexpr std::array<double, 5> amp_lapse_rate = {
        0.00e-3, 0.25e-3, 0.32e-3, 0.81e-3, 0.62e-3};
    constexpr std::array<double, 5> amp_water_vapor_rate = {
        0.00, 0.33, 0.46, 0.74, 0.30};
    constexpr double gravity = 9.80665;
    constexpr double gas_constant_dry = 287.0537625;

    const double latitude_deg = pos[0] * kRadiansToDegrees;
    if (latitude_deg < 0.0 || latitude_deg > 75.0) {
        return false;
    }

    const double doy = static_cast<double>(dayOfYearFromTime(time));
    const double seasonal = std::cos(2.0 * M_PI * (doy - 28.0) / 365.25);
    auto seasonalValues = [&](const std::array<double, 5>& mean_values,
                              const std::array<double, 5>& amp_values) {
        std::array<double, 5> values = {};
        for (size_t i = 0; i < values.size(); ++i) {
            values[i] = mean_values[i] - amp_values[i] * seasonal;
        }
        return values;
    };

    const double pressure0 =
        interpMetCoefficient(seasonalValues(mean_pressure, amp_pressure), latitude_deg);
    const double temperature0 =
        interpMetCoefficient(seasonalValues(mean_temperature, amp_temperature), latitude_deg);
    const double water_pressure0 =
        interpMetCoefficient(seasonalValues(mean_water_pressure, amp_water_pressure), latitude_deg);
    const double lapse_rate =
        interpMetCoefficient(seasonalValues(mean_lapse_rate, amp_lapse_rate), latitude_deg);
    const double water_vapor_rate =
        interpMetCoefficient(seasonalValues(mean_water_vapor_rate, amp_water_vapor_rate),
                             latitude_deg);

    const double geoid_height_m = embeddedGeoidHeightJapan(pos[0], pos[1]);
    const double orthometric_height_m = pos[2] - geoid_height_m;
    const double height_scale = 1.0 - lapse_rate * orthometric_height_m / temperature0;
    const double temperature = temperature0 - lapse_rate * orthometric_height_m;
    if (temperature <= 0.0 || height_scale <= 0.0 || lapse_rate <= 0.0) {
        return false;
    }

    const double pressure =
        pressure0 * std::pow(height_scale, gravity / (gas_constant_dry * lapse_rate));
    double water_pressure =
        water_pressure0 *
        std::pow(height_scale,
                 (water_vapor_rate + 1.0) * gravity / (gas_constant_dry * lapse_rate));

    const double celsius = temperature - 273.15;
    const double saturation =
        6.11 * std::pow(temperature / 273.15, -5.3) *
        std::exp(25.2 * celsius / temperature);
    if (water_pressure > saturation) {
        water_pressure = saturation;
    }

    hydrostatic_m =
        2.2768 * pressure * 0.001 /
        (1.0 - 0.00266 * std::cos(2.0 * pos[0]) - 2.8e-7 * pos[2]);
    wet_m = 2.2768 * (1255.0 / temperature + 0.05) * water_pressure * 0.001;
    return std::isfinite(hydrostatic_m) && std::isfinite(wet_m) &&
           hydrostatic_m > 0.0 && wet_m > 0.0;
}

tidal::EarthRotationParameters oneEntryErp(const GNSSTime& time, const double erpv[5]) {
    tidal::EarthRotationParameters erp;
    tidal::EarthRotationParameters::Entry entry;
    entry.mjd = tidal::julianDateFromTime(time) - 2400000.5;
    entry.xp_rad = erpv[0];
    entry.yp_rad = erpv[1];
    entry.ut1_utc_s = erpv[2];
    entry.lod_s_per_day = erpv[3];
    entry.xpr_rad_per_day = erpv[4];
    entry.ypr_rad_per_day = 0.0;
    erp.entries.push_back(entry);
    return erp;
}

}  // namespace

bool tidedispAvailable() {
    return true;
}

bool windupcorrAvailable() {
    return true;
}

bool antmodelAvailable() {
    return true;
}

bool ionmapfAvailable() {
    return true;
}

bool prectropAvailable() {
    return true;
}

bool satposSsrAvailable() {
    return false;
}

bool corrmeasAvailable() {
    return false;
}

void tidedisp(const GNSSTime& gpst,
              const double rr[3],
              const double erpv[5],
              double disp_out[3]) {
    const Vector3d receiver_position(rr[0], rr[1], rr[2]);
    const auto erp = oneEntryErp(gpst, erpv);
    const auto components = tidal::calculateTideDisplacement(
        receiver_position, gpst, &erp, nullptr, nullptr, true, false, true);
    const Vector3d disp = components.total();
    for (int i = 0; i < 3; ++i) {
        disp_out[i] = disp(i);
    }
}

void windupcorr(const GNSSTime& /*time*/,
                const double rs[6],
                const double rr[3],
                double& phw_io) {
    double ek_vec[3] = {};
    for (int i = 0; i < 3; ++i) {
        ek_vec[i] = rr[i] - rs[i];
    }
    double ek[3] = {};
    if (!unit3(ek_vec, ek)) {
        return;
    }

    double ezs_vec[3] = {-rs[0], -rs[1], -rs[2]};
    double ezs[3] = {};
    if (!unit3(ezs_vec, ezs)) {
        return;
    }

    const double omega[3] = {0.0, 0.0, constants::OMEGA_E};
    double omega_cross_r[3] = {};
    cross3(omega, rs, omega_cross_r);
    double ess_vec[3] = {
        rs[3] + omega_cross_r[0],
        rs[4] + omega_cross_r[1],
        rs[5] + omega_cross_r[2],
    };
    double ess[3] = {};
    if (!unit3(ess_vec, ess)) {
        return;
    }

    double tmp[3] = {};
    double eys[3] = {};
    cross3(ezs, ess, tmp);
    if (!unit3(tmp, eys)) {
        return;
    }
    double exs[3] = {};
    cross3(eys, ezs, exs);

    double lat = 0.0;
    double lon = 0.0;
    double h = 0.0;
    ecef2geodetic(Vector3d(rr[0], rr[1], rr[2]), lat, lon, h);
    (void)h;
    const double sin_lat = std::sin(lat);
    const double cos_lat = std::cos(lat);
    const double sin_lon = std::sin(lon);
    const double cos_lon = std::cos(lon);
    const double exr[3] = {-sin_lon, cos_lon, 0.0};
    const double eyr[3] = {-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat};

    double eks[3] = {};
    double ekr[3] = {};
    cross3(ek, eys, eks);
    cross3(ek, eyr, ekr);
    double ds[3] = {};
    double dr[3] = {};
    for (int i = 0; i < 3; ++i) {
        ds[i] = exs[i] - ek[i] * dot3(ek, exs) - eks[i];
        dr[i] = exr[i] - ek[i] * dot3(ek, exr) + ekr[i];
    }
    const double denom = norm3(ds) * norm3(dr);
    if (denom <= 0.0) {
        return;
    }
    double cosp = dot3(ds, dr) / denom;
    cosp = std::clamp(cosp, -1.0, 1.0);
    double ph = std::acos(cosp) / (2.0 * M_PI);
    double drs[3] = {};
    cross3(ds, dr, drs);
    if (dot3(ek, drs) < 0.0) {
        ph = -ph;
    }
    phw_io = ph + std::floor(phw_io - ph + 0.5);
}

void antmodel(const ReceiverPcvModel& pcv,
              const double del[3],
              const double azel[2],
              int opt,
              double dant[kParityMaxFreq]) {
    const double cos_el = std::cos(azel[1]);
    const double e[3] = {
        std::sin(azel[0]) * cos_el,
        std::cos(azel[0]) * cos_el,
        std::sin(azel[1]),
    };
    for (int f = 0; f < kParityMaxFreq; ++f) {
        double off[3] = {};
        for (int j = 0; j < 3; ++j) {
            off[j] = pcv.offsets_m[static_cast<size_t>(f)][static_cast<size_t>(j)] + del[j];
        }
        const double pcv_m =
            opt ? interpPcvVariation(
                      pcv.variations_m[static_cast<size_t>(f)],
                      90.0 - azel[1] * kRadiansToDegrees)
                : 0.0;
        dant[f] = -dot3(off, e) + pcv_m;
    }
}

double ionmapf(const double pos[3], const double azel[2]) {
    if (pos[2] >= kIonosphereHeightM) {
        return 1.0;
    }
    const double ratio =
        (constants::WGS84_A + pos[2]) / (constants::WGS84_A + kIonosphereHeightM);
    return 1.0 /
           std::cos(std::asin(ratio * std::sin(M_PI / 2.0 - azel[1])));
}

double prectrop(const GNSSTime& time,
                const double pos[3],
                const double azel[2],
                double zwd,
                double ztd) {
    double hydrostatic_m = 0.0;
    double wet_m = 0.0;
    if (!standardVerticalTroposphereDelays(time, pos, hydrostatic_m, wet_m)) {
        return 0.0;
    }
    const double dry_mapping =
        models::niellHydrostaticMapping(pos[0], pos[2], azel[1], dayOfYearFromTime(time));
    const double wet_mapping = models::niellWetMapping(pos[0], azel[1]);
    return dry_mapping * hydrostatic_m * ztd + wet_mapping * wet_m * zwd;
}

bool satpos_ssr(const GNSSTime& /*teph*/,
                const GNSSTime& /*time*/,
                int /*sat*/,
                SatposSsrOutput& /*out*/) {
    return false;
}

bool corrmeas(CorrmeasOutput& /*out*/) {
    return false;
}

}  // namespace libgnss::clasnat_parity
