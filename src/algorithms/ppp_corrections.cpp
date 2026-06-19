#include "ppp_internal.hpp"

#include <libgnss++/algorithms/ppp_bias_identity.hpp>
#include <libgnss++/algorithms/ppp_atmosphere.hpp>
#include <libgnss++/algorithms/ppp_osr.hpp>
#include <libgnss++/algorithms/ppp_utils.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/signals.hpp>
#include <libgnss++/iers/earth_rotation.hpp>
#include <libgnss++/iers/ephemeris.hpp>
#include <libgnss++/iers/tides.hpp>
#include <libgnss++/models/troposphere.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace libgnss {

using namespace ppp_internal;

namespace {

constexpr double kDegreesToRadians = M_PI / 180.0;
constexpr double kRtkLibMinElevationRadians = 5.0 * kDegreesToRadians;
constexpr double kRtkLibGlonassErrorFactor = 1.5;
constexpr double kRtkLibSbasErrorFactor = 3.0;
constexpr double kRtkLibGpsL5ErrorFactor = 10.0;
constexpr double kMadocaSsrMaxAgeSeconds = 60.0;
constexpr std::array<double, 11> kOceanLoadingPeriodsSeconds = {
    12.4206012 * 3600.0,   // M2
    12.0 * 3600.0,         // S2
    12.65834751 * 3600.0,  // N2
    11.967234 * 3600.0,    // K2
    23.93447213 * 3600.0,  // K1
    25.81933871 * 3600.0,  // O1
    24.065889 * 3600.0,    // P1
    26.8683567 * 3600.0,   // Q1
    327.8599387 * 3600.0,  // Mf
    661.3111655 * 3600.0,  // Mm
    4382.905 * 3600.0      // Ssa
};

bool madocaSsrReferenceIsCurrent(const GNSSTime& epoch, const GNSSTime& reference) {
    if (reference.week == 0) {
        return false;
    }
    const double age = epoch - reference;
    return age >= -1e-6 && age <= kMadocaSsrMaxAgeSeconds;
}

bool madocaSsrCorrectionIsCoherent(const SSRCorrectionStatus& status,
                                   const GNSSTime& epoch) {
    if (!status.orbit_valid || !status.clock_valid) {
        return false;
    }
    if (!madocaSsrReferenceIsCurrent(epoch, status.orbit_reference_time) ||
        !madocaSsrReferenceIsCurrent(epoch, status.clock_reference_time)) {
        return false;
    }
    if (status.ssr_orbit_iod >= 0 && status.ssr_clock_iod >= 0 &&
        status.ssr_orbit_iod != status.ssr_clock_iod) {
        return false;
    }
    return true;
}

int dayOfYearFromTime(const GNSSTime& time) {
    const auto system_time = time.toSystemTime();
    const std::time_t utc_seconds = std::chrono::system_clock::to_time_t(system_time);
    std::tm utc_tm{};
    gmtime_r(&utc_seconds, &utc_tm);
    return utc_tm.tm_yday + 1;
}

double julianDateFromTime(const GNSSTime& time) {
    // GPS epoch (1980-01-06 00:00:00) is JD 2444244.5.
    return 2444244.5 + static_cast<double>(time.week) * 7.0 + time.tow / 86400.0;
}

double wrapRadians(double angle_rad) {
    const double wrapped = std::fmod(angle_rad, 2.0 * M_PI);
    return wrapped < 0.0 ? wrapped + 2.0 * M_PI : wrapped;
}

double greenwichMeanSiderealTime(const GNSSTime& time) {
    const double jd = julianDateFromTime(time);
    const double t = (jd - 2451545.0) / 36525.0;
    const double gmst_deg =
        280.46061837 + 360.98564736629 * (jd - 2451545.0) +
        0.000387933 * t * t - (t * t * t) / 38710000.0;
    return wrapRadians(gmst_deg * kDegreesToRadians);
}

Vector3d rotateEciToEcef(const Vector3d& eci, double gmst_rad) {
    const double cos_gmst = std::cos(gmst_rad);
    const double sin_gmst = std::sin(gmst_rad);
    return Vector3d(
        cos_gmst * eci.x() + sin_gmst * eci.y(),
        -sin_gmst * eci.x() + cos_gmst * eci.y(),
        eci.z());
}

Vector3d approximateSunPositionEcef(const GNSSTime& time) {
    constexpr double kAstronomicalUnitMeters = 149597870700.0;
    const double days_since_j2000 = julianDateFromTime(time) - 2451545.0;
    const double mean_longitude =
        wrapRadians((280.460 + 0.9856474 * days_since_j2000) * kDegreesToRadians);
    const double mean_anomaly =
        wrapRadians((357.528 + 0.9856003 * days_since_j2000) * kDegreesToRadians);
    const double ecliptic_longitude =
        wrapRadians(mean_longitude +
                    (1.915 * std::sin(mean_anomaly) +
                     0.020 * std::sin(2.0 * mean_anomaly)) * kDegreesToRadians);
    const double obliquity =
        (23.439 - 0.0000004 * days_since_j2000) * kDegreesToRadians;
    const double radius_au =
        1.00014 - 0.01671 * std::cos(mean_anomaly) -
        0.00014 * std::cos(2.0 * mean_anomaly);
    const double radius_m = radius_au * kAstronomicalUnitMeters;
    const Vector3d eci(
        radius_m * std::cos(ecliptic_longitude),
        radius_m * std::cos(obliquity) * std::sin(ecliptic_longitude),
        radius_m * std::sin(obliquity) * std::sin(ecliptic_longitude));
    return rotateEciToEcef(eci, greenwichMeanSiderealTime(time));
}

Vector3d approximateMoonPositionEcef(const GNSSTime& time) {
    constexpr double kKilometersToMeters = 1000.0;
    const double days_since_j2000 = julianDateFromTime(time) - 2451545.0;
    const double mean_longitude =
        wrapRadians((218.316 + 13.176396 * days_since_j2000) * kDegreesToRadians);
    const double mean_anomaly_moon =
        wrapRadians((134.963 + 13.064993 * days_since_j2000) * kDegreesToRadians);
    const double mean_anomaly_sun =
        wrapRadians((357.529 + 0.98560028 * days_since_j2000) * kDegreesToRadians);
    const double elongation =
        wrapRadians((297.850 + 12.190749 * days_since_j2000) * kDegreesToRadians);
    const double argument_of_latitude =
        wrapRadians((93.272 + 13.229350 * days_since_j2000) * kDegreesToRadians);

    const double ecliptic_longitude =
        wrapRadians(mean_longitude +
                    (6.289 * std::sin(mean_anomaly_moon) +
                     1.274 * std::sin(2.0 * elongation - mean_anomaly_moon) +
                     0.658 * std::sin(2.0 * elongation) +
                     0.214 * std::sin(2.0 * mean_anomaly_moon) -
                     0.186 * std::sin(mean_anomaly_sun)) *
                        kDegreesToRadians);
    const double ecliptic_latitude =
        (5.128 * std::sin(argument_of_latitude) +
         0.280 * std::sin(mean_anomaly_moon + argument_of_latitude) +
         0.277 * std::sin(mean_anomaly_moon - argument_of_latitude) +
         0.173 * std::sin(2.0 * elongation - argument_of_latitude)) *
        kDegreesToRadians;
    const double radius_m =
        (385001.0 - 20905.0 * std::cos(mean_anomaly_moon) -
         3699.0 * std::cos(2.0 * elongation - mean_anomaly_moon) -
         2956.0 * std::cos(2.0 * elongation) -
         570.0 * std::cos(2.0 * mean_anomaly_moon)) *
        kKilometersToMeters;
    const double obliquity =
        (23.439 - 0.0000004 * days_since_j2000) * kDegreesToRadians;
    const double cos_lat = std::cos(ecliptic_latitude);
    const Vector3d eci(
        radius_m * cos_lat * std::cos(ecliptic_longitude),
        radius_m *
            (std::cos(obliquity) * cos_lat * std::sin(ecliptic_longitude) -
             std::sin(obliquity) * std::sin(ecliptic_latitude)),
        radius_m *
            (std::sin(obliquity) * cos_lat * std::sin(ecliptic_longitude) +
             std::cos(obliquity) * std::sin(ecliptic_latitude)));
    return rotateEciToEcef(eci, greenwichMeanSiderealTime(time));
}

Vector3d surfaceUpVector(const Vector3d& receiver_position) {
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double height_m = 0.0;
    ecef2geodetic(receiver_position, latitude_rad, longitude_rad, height_m);
    return Vector3d(
               std::cos(latitude_rad) * std::cos(longitude_rad),
               std::cos(latitude_rad) * std::sin(longitude_rad),
               std::sin(latitude_rad))
        .normalized();
}

Vector3d bodyTideDisplacement(const Vector3d& receiver_position,
                              const Vector3d& body_position_ecef,
                              double body_gm_m3_s2) {
    constexpr double kEarthGM = 3.986004418e14;
    constexpr double kLoveH2 = 0.6078;
    constexpr double kLoveL2 = 0.0847;

    const double body_distance_m = body_position_ecef.norm();
    if (!std::isfinite(body_distance_m) || body_distance_m <= 1.0) {
        return Vector3d::Zero();
    }

    const Vector3d up = surfaceUpVector(receiver_position);
    const Vector3d body_dir = body_position_ecef / body_distance_m;
    const double projection = std::clamp(up.dot(body_dir), -1.0, 1.0);
    const Vector3d tangential = body_dir - projection * up;
    const double scale =
        body_gm_m3_s2 / kEarthGM *
        std::pow(constants::WGS84_A / body_distance_m, 3.0) *
        constants::WGS84_A;
    return scale * (kLoveH2 * (1.5 * projection * projection - 0.5) * up +
                    3.0 * kLoveL2 * projection * tangential);
}

double modeledZenithTroposphereDelayMetersImpl(const Vector3d& receiver_position,
                                               const GNSSTime& time) {
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double height_m = 0.0;
    ecef2geodetic(receiver_position, latitude_rad, longitude_rad, height_m);
    const auto delay =
        models::estimateZenithTroposphereClimatology(latitude_rad, height_m, dayOfYearFromTime(time));
    const double total_delay_m = delay.totalDelayMeters();
    return std::isfinite(total_delay_m) && total_delay_m > 0.0 ?
        total_delay_m :
        kDefaultZenithDelayMeters;
}

double modeledTroposphereDelayMeters(const Vector3d& receiver_position,
                                     double elevation,
                                     const GNSSTime& time) {
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double height_m = 0.0;
    ecef2geodetic(receiver_position, latitude_rad, longitude_rad, height_m);
    const double modeled_delay = models::modeledTroposphereDelayClimatology(
        latitude_rad,
        height_m,
        elevation,
        dayOfYearFromTime(time));
    return std::isfinite(modeled_delay) && modeled_delay > 0.0 ?
        modeled_delay :
        kDefaultZenithDelayMeters;
}

// Look up an SSR bias (code or phase) keyed by RTCM SSR signal id.
// MADOCA transmits a single GPS L2 bias under the C2W code (id 9), but the
// receiver may track C2L (id 8). Direct .find() then misses on C2L/C2W
// disagreement and silently returns 0. Mirror RTKLIB's mcssr_sel_biascode():
// for GPS L2 (id 8/9) fall back to the sibling id so we always pick up the
// transmitted L2 bias regardless of which L2 signal the receiver tracked.
inline double ssrBiasLookup(const std::map<uint8_t, double>& m,
                            GNSSSystem system,
                            uint8_t id,
                            bool madoca_bias_identity) {
    const auto it = m.find(id);
    if (it != m.end()) {
        return it->second;
    }
    if (!madoca_bias_identity && system == GNSSSystem::GPS && (id == 8U || id == 9U)) {
        const auto sibling = m.find(id == 8U ? 9U : 8U);
        if (sibling != m.end()) {
            return sibling->second;
        }
    }
    return 0.0;
}

double observationCodeBiasMeters(GNSSSystem system,
                                 SignalType primary_signal,
                                 SignalType secondary_signal,
                                 bool use_ionosphere_free,
                                 const std::map<uint8_t, double>& code_bias_m,
                                 double coeff_primary,
                                 double coeff_secondary,
                                 const std::string& primary_observation_type,
                                 const std::string& secondary_observation_type,
                                 bool madoca_bias_identity) {
    const uint8_t primary_id = algorithms::ppp_bias_identity::madocaBiasIdentityIdForObservation(
        system, primary_signal, primary_observation_type, madoca_bias_identity);
    if (primary_id == 0U) {
        return 0.0;
    }
    const double primary_bias =
        ssrBiasLookup(code_bias_m, system, primary_id, madoca_bias_identity);
    if (!use_ionosphere_free || secondary_signal == SignalType::SIGNAL_TYPE_COUNT) {
        return primary_bias;
    }

    const uint8_t secondary_id = algorithms::ppp_bias_identity::madocaBiasIdentityIdForObservation(
        system, secondary_signal, secondary_observation_type, madoca_bias_identity);
    if (secondary_id == 0U) {
        return coeff_primary * primary_bias;
    }
    const double secondary_bias =
        ssrBiasLookup(code_bias_m, system, secondary_id, madoca_bias_identity);
    return coeff_primary * primary_bias + coeff_secondary * secondary_bias;
}

double observationPhaseBiasMeters(GNSSSystem system,
                                  SignalType primary_signal,
                                  SignalType secondary_signal,
                                  bool use_ionosphere_free,
                                  const std::map<uint8_t, double>& phase_bias_m,
                                  double coeff_primary,
                                  double coeff_secondary,
                                  const std::string& primary_observation_type,
                                  const std::string& secondary_observation_type,
                                  bool madoca_bias_identity) {
    const uint8_t primary_id = algorithms::ppp_bias_identity::madocaBiasIdentityIdForObservation(
        system, primary_signal, primary_observation_type, madoca_bias_identity);
    if (primary_id == 0U) {
        return 0.0;
    }
    const double primary_bias =
        ssrBiasLookup(phase_bias_m, system, primary_id, madoca_bias_identity);
    if (!use_ionosphere_free || secondary_signal == SignalType::SIGNAL_TYPE_COUNT) {
        return primary_bias;
    }

    const uint8_t secondary_id = algorithms::ppp_bias_identity::madocaBiasIdentityIdForObservation(
        system, secondary_signal, secondary_observation_type, madoca_bias_identity);
    if (secondary_id == 0U) {
        return coeff_primary * primary_bias;
    }
    const double secondary_bias =
        ssrBiasLookup(phase_bias_m, system, secondary_id, madoca_bias_identity);
    return coeff_primary * primary_bias + coeff_secondary * secondary_bias;
}

bool ssrBiasPresent(const std::map<uint8_t, double>& bias_m,
                    GNSSSystem system,
                    SignalType signal,
                    const std::string& observation_type,
                    bool madoca_bias_identity) {
    const uint8_t id = algorithms::ppp_bias_identity::madocaBiasIdentityIdForObservation(
        system, signal, observation_type, madoca_bias_identity);
    if (id == 0U) {
        return false;
    }
    if (bias_m.find(id) != bias_m.end()) {
        return true;
    }
    if (!madoca_bias_identity && system == GNSSSystem::GPS && (id == 8U || id == 9U)) {
        return bias_m.find(id == 8U ? 9U : 8U) != bias_m.end();
    }
    return false;
}

bool madocaSsrMeasurementBiasesAreCoherent(
    const SatelliteId& satellite,
    SignalType primary_signal,
    SignalType secondary_signal,
    const std::string& primary_observation_type,
    const std::string& secondary_observation_type,
    bool has_l2,
    bool has_carrier_phase,
    bool has_carrier_phase_l2,
    const std::map<uint8_t, double>& code_bias_m,
    const std::map<uint8_t, double>& phase_bias_m,
    bool use_ionosphere_free,
    bool madoca_bias_identity) {
    if (!ssrBiasPresent(
            code_bias_m,
            satellite.system,
            primary_signal,
            primary_observation_type,
            madoca_bias_identity)) {
        return false;
    }
    const bool needs_secondary =
        secondary_signal != SignalType::SIGNAL_TYPE_COUNT &&
        (use_ionosphere_free || has_l2);
    if (needs_secondary &&
        !ssrBiasPresent(
            code_bias_m,
            satellite.system,
            secondary_signal,
            secondary_observation_type,
            madoca_bias_identity)) {
        return false;
    }
    if (satellite.system == GNSSSystem::GLONASS) {
        return true;
    }
    if (has_carrier_phase &&
        !ssrBiasPresent(
            phase_bias_m,
            satellite.system,
            primary_signal,
            primary_observation_type,
            madoca_bias_identity)) {
        return false;
    }
    if (needs_secondary &&
        (has_carrier_phase || has_carrier_phase_l2) &&
        !ssrBiasPresent(
            phase_bias_m,
            satellite.system,
            secondary_signal,
            secondary_observation_type,
            madoca_bias_identity)) {
        return false;
    }
    return true;
}

std::string biasObservationCode(SignalType signal) {
    switch (signal) {
        case SignalType::GPS_L1CA: return "C1C";
        case SignalType::GPS_L1P: return "C1P";
        case SignalType::GPS_L2P: return "C2P";
        case SignalType::GPS_L2C: return "C2W";
        case SignalType::GPS_L5: return "C5Q";
        case SignalType::GLO_L1CA: return "C1C";
        case SignalType::GLO_L1P: return "C1P";
        case SignalType::GLO_L2CA: return "C2C";
        case SignalType::GLO_L2P: return "C2P";
        case SignalType::GAL_E1: return "C1C";
        case SignalType::GAL_E5A: return "C5Q";
        case SignalType::GAL_E5B: return "C7Q";
        case SignalType::GAL_E6: return "C6C";
        case SignalType::BDS_B1I: return "C2I";
        case SignalType::BDS_B2I: return "C7I";
        case SignalType::BDS_B3I: return "C6I";
        case SignalType::BDS_B1C: return "C1C";
        case SignalType::BDS_B2A: return "C5Q";
        case SignalType::QZS_L1CA: return "C1C";
        case SignalType::QZS_L2C: return "C2L";
        case SignalType::QZS_L5: return "C5Q";
        default: return "";
    }
}

double dcbEntryBiasMeters(const DCBEntry& entry) {
    std::string unit = trimCopy(entry.unit);
    std::transform(unit.begin(), unit.end(), unit.begin(), [](unsigned char ch) {
        return static_cast<char>(std::toupper(ch));
    });
    if (unit == "NS") {
        return entry.bias * constants::SPEED_OF_LIGHT * 1e-9;
    }
    if (unit == "M" || unit == "METER" || unit == "METERS") {
        return entry.bias;
    }
    return std::numeric_limits<double>::quiet_NaN();
}

bool findOsbBiasMeters(const DCBProducts& dcb_products,
                       const SatelliteId& satellite,
                       const std::string& observation_code,
                       double& bias_m) {
    for (const auto& entry : dcb_products.entries) {
        if (!entry.valid || !(entry.satellite == satellite) || entry.bias_type != "OSB") {
            continue;
        }
        if (entry.observation_1 != observation_code && entry.observation_2 != observation_code) {
            continue;
        }
        const double converted = dcbEntryBiasMeters(entry);
        if (!std::isfinite(converted)) {
            continue;
        }
        bias_m = converted;
        return true;
    }
    return false;
}

double observationDcbBiasMeters(const DCBProducts& dcb_products,
                                const SatelliteId& satellite,
                                SignalType primary_signal,
                                SignalType secondary_signal,
                                bool use_ionosphere_free,
                                double coeff_primary,
                                double coeff_secondary) {
    const std::string primary_code = biasObservationCode(primary_signal);
    if (primary_code.empty()) {
        return 0.0;
    }

    double primary_bias_m = 0.0;
    const bool have_primary = findOsbBiasMeters(dcb_products, satellite, primary_code, primary_bias_m);
    if (!use_ionosphere_free || secondary_signal == SignalType::SIGNAL_TYPE_COUNT) {
        return have_primary ? primary_bias_m : 0.0;
    }

    const std::string secondary_code = biasObservationCode(secondary_signal);
    if (secondary_code.empty()) {
        return have_primary ? coeff_primary * primary_bias_m : 0.0;
    }

    double secondary_bias_m = 0.0;
    const bool have_secondary =
        findOsbBiasMeters(dcb_products, satellite, secondary_code, secondary_bias_m);
    if (!have_primary && !have_secondary) {
        return 0.0;
    }
    return coeff_primary * (have_primary ? primary_bias_m : 0.0) +
           coeff_secondary * (have_secondary ? secondary_bias_m : 0.0);
}

bool ionexPiercePointAndMapping(const IONEXProducts& ionex_products,
                                const Vector3d& receiver_position,
                                double azimuth_rad,
                                double elevation_rad,
                                double& ipp_lat_deg,
                                double& ipp_lon_deg,
                                double& mapping_factor) {
    if (elevation_rad <= 0.0) {
        return false;
    }
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double height_m = 0.0;
    ecef2geodetic(receiver_position, latitude_rad, longitude_rad, height_m);

    const double earth_radius_m =
        ionex_products.base_radius_km > 0.0 ?
            ionex_products.base_radius_km * 1000.0 :
            constants::WGS84_A;
    const double shell_height_m =
        !ionex_products.height_grid.empty() ?
            ionex_products.height_grid.front() * 1000.0 :
            450000.0;
    if (earth_radius_m <= 0.0 || shell_height_m < 0.0) {
        return false;
    }

    const double ratio = earth_radius_m / (earth_radius_m + shell_height_m);
    const double cos_elevation = std::cos(elevation_rad);
    const double argument = std::clamp(ratio * cos_elevation, -1.0, 1.0);
    const double psi = M_PI_2 - elevation_rad - std::asin(argument);
    const double sin_lat = std::sin(latitude_rad);
    const double cos_lat = std::cos(latitude_rad);
    const double sin_psi = std::sin(psi);
    const double cos_psi = std::cos(psi);

    const double ipp_lat_rad = std::asin(
        std::clamp(sin_lat * cos_psi + cos_lat * sin_psi * std::cos(azimuth_rad), -1.0, 1.0));
    const double ipp_lon_rad = longitude_rad + std::atan2(
        sin_psi * std::sin(azimuth_rad),
        cos_lat * cos_psi - sin_lat * sin_psi * std::cos(azimuth_rad));

    const double mapping_argument = ratio * cos_elevation;
    mapping_factor = 1.0 / std::sqrt(std::max(1e-12, 1.0 - mapping_argument * mapping_argument));
    ipp_lat_deg = ipp_lat_rad / kDegreesToRadians;
    ipp_lon_deg = ipp_lon_rad / kDegreesToRadians;
    while (ipp_lon_deg > 180.0) {
        ipp_lon_deg -= 360.0;
    }
    while (ipp_lon_deg < -180.0) {
        ipp_lon_deg += 360.0;
    }
    return std::isfinite(mapping_factor);
}

double rtklibSystemErrorFactor(GNSSSystem system) {
    switch (system) {
        case GNSSSystem::GLONASS:
            return kRtkLibGlonassErrorFactor;
        case GNSSSystem::SBAS:
            return kRtkLibSbasErrorFactor;
        default:
            return 1.0;
    }
}

bool usesGpsL5ErrorFactor(SignalType signal) {
    return signal == SignalType::GPS_L5 || signal == SignalType::QZS_L5;
}

}  // namespace

double PPPProcessor::modeledZenithTroposphereDelayMeters(
    const Vector3d& receiver_position,
    const GNSSTime& time) {
    return modeledZenithTroposphereDelayMetersImpl(receiver_position, time);
}

bool PPPProcessor::hasEnoughCoherentSsrObservations(const ObservationData& obs,
                                                    const NavigationData& nav) {
    if (!require_coherent_ssr_ || !ssr_products_loaded_) {
        return true;
    }

    const auto if_obs = formIonosphereFree(obs, nav);
    const bool madoca_bias_identity =
        require_coherent_ssr_ && env_overrides_.madoca_bias_identity;
    size_t coherent_count = 0;
    for (const auto& observation : if_obs) {
        if (!observation.valid) {
            continue;
        }
        Vector3d orbit_correction = Vector3d::Zero();
        double clock_correction_m = 0.0;
        std::map<uint8_t, double> code_bias_m;
        std::map<uint8_t, double> phase_bias_m;
        SSRCorrectionStatus status;
        const bool ssr_ok = ssr_products_.interpolateCorrection(
            observation.satellite,
            obs.time,
            orbit_correction,
            clock_correction_m,
            nullptr,
            &code_bias_m,
            &phase_bias_m,
            nullptr,
            nullptr,
            nullptr,
            nullptr,
            0,
            nullptr,
            nullptr,
            &status,
            false);
        if (env_overrides_.madoca_galileo_gate &&
            observation.satellite.system == GNSSSystem::Galileo &&
            !nav.hasMadocaGalileoEphemeris(
                observation.satellite, obs.time, status.orbit_iode)) {
            continue;
        }
        if (ssr_ok && madocaSsrCorrectionIsCoherent(status, obs.time) &&
            madocaSsrMeasurementBiasesAreCoherent(
                observation.satellite,
                observation.primary_signal,
                observation.secondary_signal,
                observation.primary_observation_type,
                observation.secondary_observation_type,
                observation.has_l2,
                observation.has_carrier_phase,
                observation.has_carrier_phase_l2,
                code_bias_m,
                phase_bias_m,
                ppp_config_.use_ionosphere_free,
                madoca_bias_identity)) {
            ++coherent_count;
        }
    }
    return coherent_count >= static_cast<size_t>(config_.min_satellites);
}

Vector3d PPPProcessor::calculateReceiverAntennaOffsetEcef(
    const Vector3d& receiver_marker_position,
    const IonosphereFreeObs& observation) const {
    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double height_m = 0.0;
    ecef2geodetic(receiver_marker_position, latitude_rad, longitude_rad, height_m);

    Vector3d offset_enu = ppp_config_.receiver_antenna_delta_enu;
    if (receiver_antex_loaded_ && !ppp_config_.receiver_antenna_type.empty()) {
        const auto antenna_it =
            receiver_antex_offsets_.find(normalizeAntennaType(ppp_config_.receiver_antenna_type));
        if (antenna_it != receiver_antex_offsets_.end()) {
            auto signal_offset = [&](SignalType signal) -> Vector3d {
                const auto it = antenna_it->second.find(signal);
                return it != antenna_it->second.end() ? it->second : Vector3d::Zero();
            };
            if (!ppp_config_.use_ionosphere_free ||
                observation.secondary_signal == SignalType::SIGNAL_TYPE_COUNT) {
                offset_enu += signal_offset(observation.primary_signal);
            } else {
                offset_enu +=
                    observation.primary_code_bias_coeff * signal_offset(observation.primary_signal) +
                    observation.secondary_code_bias_coeff *
                        signal_offset(observation.secondary_signal);
            }
        }
    }

    return enu2ecef(offset_enu, latitude_rad, longitude_rad);
}

double PPPProcessor::receiverAntennaPcvMeters(SignalType signal,
                                              double elevation_rad) const {
    if (!receiver_antex_loaded_ || ppp_config_.receiver_antenna_type.empty()) {
        return 0.0;
    }
    const auto ant_it =
        receiver_antex_pcv_.find(normalizeAntennaType(ppp_config_.receiver_antenna_type));
    if (ant_it == receiver_antex_pcv_.end()) {
        return 0.0;
    }
    const auto sig_it = ant_it->second.find(signal);
    if (sig_it == ant_it->second.end()) {
        return 0.0;
    }
    const ReceiverPcvGrid& grid = sig_it->second;
    if (grid.noazi_m.empty() || grid.dzen_deg <= 0.0) {
        return 0.0;
    }
    // RTKLIB interpvar(): linear interpolation of the NOAZI grid at the zenith
    // angle, clamped at both ends.
    const double zenith_deg = 90.0 - elevation_rad / kDegreesToRadians;
    const double node = (zenith_deg - grid.zen1_deg) / grid.dzen_deg;
    const int last = static_cast<int>(grid.noazi_m.size()) - 1;
    if (node <= 0.0) return grid.noazi_m.front();
    if (node >= last) return grid.noazi_m.back();
    const int i0 = static_cast<int>(std::floor(node));
    const double frac = node - i0;
    return grid.noazi_m[i0] * (1.0 - frac) + grid.noazi_m[i0 + 1] * frac;
}

double PPPProcessor::clasReceiverAntennaRangeCorrectionMeters(
    SignalType signal,
    double azimuth_rad,
    double elevation_rad) const {
    if (!receiver_antex_loaded_ || ppp_config_.receiver_antenna_type.empty() ||
        signal == SignalType::SIGNAL_TYPE_COUNT) {
        return 0.0;
    }
    const auto ant_it =
        receiver_antex_offsets_.find(normalizeAntennaType(ppp_config_.receiver_antenna_type));
    if (ant_it == receiver_antex_offsets_.end()) {
        return 0.0;
    }
    Vector3d antenna_offset_neu = Vector3d::Zero();
    const auto sig_it = ant_it->second.find(signal);
    if (sig_it != ant_it->second.end()) {
        antenna_offset_neu = sig_it->second;
    }
    return clasReceiverAntennaCorrectionMeters(
        ppp_config_.receiver_antenna_delta_enu,
        antenna_offset_neu,
        receiverAntennaPcvMeters(signal, elevation_rad),
        azimuth_rad,
        elevation_rad);
}

void PPPProcessor::materializeClasReceiverAntennaCorrections(
    std::vector<OSRCorrection>& osr_corrections) const {
    if (!env_overrides_.clas_rx_antenna || !receiver_antex_loaded_ ||
        ppp_config_.receiver_antenna_type.empty()) {
        return;
    }
    for (auto& osr : osr_corrections) {
        if (!osr.valid) {
            continue;
        }
        const int frequency_count = std::min(osr.num_frequencies, OSR_MAX_FREQ);
        for (int f = 0; f < frequency_count; ++f) {
            setClasOsrReceiverAntennaCorrection(
                osr,
                f,
                clasReceiverAntennaRangeCorrectionMeters(
                    osr.signals[f],
                    osr.azimuth,
                    osr.elevation));
        }
    }
}

void PPPProcessor::applyPreciseCorrections(std::vector<IonosphereFreeObs>& observations,
                                           const NavigationData& nav,
                                           const GNSSTime& time) {
    const Vector3d receiver_marker_position = applyGeophysicalCorrections(
        filter_state_.state.segment(filter_state_.pos_index, 3), time);
    const double elevation_mask = config_.elevation_mask * kDegreesToRadians;

    // Pre-fetch epoch-wide atmosphere tokens from any satellite that has them.
    // CLAS atmosphere corrections are network-wide, not per-satellite, so we
    // retrieve them once and share across all observations in this epoch.
    std::map<std::string, std::string> epoch_atmos_tokens;
    if (ssr_products_loaded_) {
        std::vector<SatelliteId> epoch_satellites;
        epoch_satellites.reserve(observations.size());
        for (const auto& obs_scan : observations) {
            if (obs_scan.valid) {
                epoch_satellites.push_back(obs_scan.satellite);
            }
        }
        epoch_atmos_tokens =
            selectClasEpochAtmosTokens(
                ssr_products_, epoch_satellites, time, receiver_marker_position, ppp_config_);
    }
    const int preferred_network_id = preferredClasNetworkId(epoch_atmos_tokens);
    const bool madoca_bias_identity =
        require_coherent_ssr_ && env_overrides_.madoca_bias_identity;
    const bool capture_shadow_metadata =
        !env_overrides_.madoca_postfit_shadow_path.empty();

    // Sun position is needed for the satellite body frame in phase wind-up.
    // We compute it once per epoch and reuse for every satellite.
    const Vector3d sun_position_ecef = approximateSunPositionEcef(time);

    for (auto& observation : observations) {
        double deferred_variance_pr = 0.0;
        double deferred_variance_cp = 0.0;
        bool applied_ssr_code_bias = false;
        bool applied_ssr_iono = false;
        const Vector3d receiver_antenna_offset_ecef =
            calculateReceiverAntennaOffsetEcef(receiver_marker_position, observation);
        const Vector3d receiver_position = receiver_marker_position + receiver_antenna_offset_ecef;
        const Ephemeris* eph = nav.getEphemeris(observation.satellite, time);
        Vector3d sat_position = Vector3d::Zero();
        Vector3d sat_velocity = Vector3d::Zero();
        double sat_clock_bias = 0.0;
        double sat_clock_drift = 0.0;

        bool have_precise = false;
        if (precise_products_loaded_) {
            have_precise = precise_products_.interpolateOrbitClock(
                observation.satellite,
                time,
                sat_position,
                sat_velocity,
                sat_clock_bias,
                sat_clock_drift);
            if (have_precise) {
                // Light-travel-time correction: the SP3 query above samples
                // the orbit at reception time, but the signal left the sat at
                // emission_time = reception − τ. Re-querying the precise
                // products at emission_time fails at SP3 file boundaries
                // (extrapolation back across the day boundary falls back to
                // the single-sample branch and zeros the velocity), so use a
                // first-order Taylor expansion with the sat_velocity returned
                // by the bracket: sat_position(em) ≈ sat_position(rcv) −
                // sat_velocity × τ. For sub-second travel times this is
                // accurate to mm. The downstream geodist() handles the Sagnac
                // (frame-rotation) term.
                const double initial_distance =
                    (sat_position - receiver_position).norm();
                if (std::isfinite(initial_distance) && initial_distance > 0.0) {
                    const double travel_time =
                        initial_distance / constants::SPEED_OF_LIGHT;
                    sat_position -= sat_velocity * travel_time;
                    sat_clock_bias -= sat_clock_drift * travel_time;
                }
            }
        }

        if (!have_precise) {
            // Select the broadcast ephemeris whose IODE matches the SSR orbit
            // correction's reference IODE (RTKLIB ephpos(...,ssr->iode,...)).
            // Without this, near an IODE transition native picks the nearest-
            // time ephemeris while the SSR delta references the previous one,
            // leaving a per-satellite orbit error of ~metres that the filter
            // absorbs into a constant ~+0.36 m East position bias.
            int ssr_orbit_iode = -1;
            SSRCorrectionStatus ssr_status;
            bool ssr_status_ok = false;
            if (ssr_products_loaded_) {
                Vector3d iode_orbit;
                double iode_clock = 0.0;
                ssr_status_ok = ssr_products_.interpolateCorrection(
                    observation.satellite, time, iode_orbit, iode_clock,
                    nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr,
                    preferred_network_id, &ssr_orbit_iode, nullptr, &ssr_status,
                    !require_coherent_ssr_);
            }
            // Require a valid SSR orbit correction (orbit_iode>=0) when SSR
            // products are loaded. MADOCA/CLAS SSR augments broadcast orbits; a
            // satellite missing its orbit delta (e.g. MIZU E12, for which the
            // MADOCALIB bridge logs "no ssr orbit correction E12" and drops the
            // satellite outright) would otherwise run on the raw broadcast orbit
            // -- metres off -- and, because the est-stec (iono, ambiguity) split
            // freezes within the first few epochs, lock in a constant position
            // bias that never recovers (E12 alone accounts for MIZU's +0.24 m
            // East offset versus the bridge). Mirror the bridge by dropping such
            // satellites upstream. Env-gated: GNSS_PPP_REQUIRE_SSR_ORBIT (unset
            // or 0 = legacy behaviour, keep the satellite on its broadcast orbit).
            if (require_coherent_ssr_ && ssr_products_loaded_ &&
                (!ssr_status_ok ||
                 !madocaSsrCorrectionIsCoherent(ssr_status, time))) {
                observation.valid = false;
                continue;
            }
            if (env_overrides_.madoca_galileo_gate &&
                require_coherent_ssr_ &&
                observation.satellite.system == GNSSSystem::Galileo &&
                !nav.hasMadocaGalileoEphemeris(
                    observation.satellite, time, ssr_status.orbit_iode)) {
                observation.valid = false;
                continue;
            }
            if (env_overrides_.require_ssr_orbit &&
                ssr_products_loaded_ && ssr_orbit_iode < 0) {
                observation.valid = false;
                continue;
            }
            // First pass: compute satellite state at reception time
            if (!nav.calculateSatelliteState(
                    observation.satellite,
                    time,
                    sat_position,
                    sat_velocity,
                    sat_clock_bias,
                    sat_clock_drift,
                    ssr_orbit_iode)) {
                observation.valid = false;
                continue;
            }
            // Light travel time correction: re-evaluate the orbit at the true
            // GPS emission time. t_obs - pr/c gives the satellite-clock time of
            // transmission; subtracting the satellite clock bias yields the
            // true GPS emission time the broadcast orbit is parameterised in
            // (RTKLIB satposs convention: time = t_obs - pr/c - dts). Using
            // +dts here evaluated the orbit at t_emit + 2*dts, shifting the
            // satellite ~v*2*dts along-track and biasing native MADOCA PPP
            // (both IFLC and est-stec) by ~+0.36 m East.
            if (observation.pseudorange_if > 0.0) {
                const double travel_time = observation.pseudorange_if / constants::SPEED_OF_LIGHT;
                const GNSSTime emission_time = time - travel_time - sat_clock_bias;
                if (!nav.calculateSatelliteState(
                        observation.satellite,
                        emission_time,
                        sat_position,
                        sat_velocity,
                        sat_clock_bias,
                        sat_clock_drift,
                        ssr_orbit_iode)) {
                    observation.valid = false;
                    continue;
                }
            }
        }

        if (ssr_products_loaded_) {
            Vector3d orbit_correction_ecef = Vector3d::Zero();
            double clock_correction_m = 0.0;
            double ura_sigma_m = 0.0;
            std::map<uint8_t, double> code_bias_m;
            std::map<uint8_t, double> phase_bias_m;
            std::map<std::string, std::string> atmos_tokens;
            SSRCorrectionStatus ssr_status;
            const bool ssr_ok = ssr_products_.interpolateCorrection(
                    observation.satellite,
                    time,
                    orbit_correction_ecef,
                    clock_correction_m,
                    &ura_sigma_m,
                    &code_bias_m,
                    &phase_bias_m,
                    &atmos_tokens,
                    nullptr,
                    nullptr,
                    nullptr,
                    preferred_network_id,
                    nullptr,
                    nullptr,
                    &ssr_status,
                    !require_coherent_ssr_);
            if (ssr_ok && capture_shadow_metadata) {
                if (ssr_status.orbit_reference_time.week != 0) {
                    observation.ssr_orbit_age_s =
                        time - ssr_status.orbit_reference_time;
                }
                if (ssr_status.clock_reference_time.week != 0) {
                    observation.ssr_clock_age_s =
                        time - ssr_status.clock_reference_time;
                }
                observation.ssr_orbit_iod = ssr_status.ssr_orbit_iod;
                observation.ssr_clock_iod = ssr_status.ssr_clock_iod;
                observation.ssr_orbit_iode = ssr_status.orbit_iode;
            }
            // Fall back to epoch-wide atmosphere tokens when per-satellite
            // atmos are empty (CLAS broadcasts network-wide corrections).
            if (atmos_tokens.empty() && !epoch_atmos_tokens.empty()) {
                atmos_tokens = epoch_atmos_tokens;
            }
            if (require_coherent_ssr_ &&
                (!ssr_ok ||
                 !madocaSsrCorrectionIsCoherent(ssr_status, time) ||
                 !madocaSsrMeasurementBiasesAreCoherent(
                     observation.satellite,
                     observation.primary_signal,
                     observation.secondary_signal,
                     observation.primary_observation_type,
                     observation.secondary_observation_type,
                     observation.has_l2,
                     observation.has_carrier_phase,
                     observation.has_carrier_phase_l2,
                     code_bias_m,
                     phase_bias_m,
                     ppp_config_.use_ionosphere_free,
                     madoca_bias_identity))) {
                observation.valid = false;
                continue;
            }
            if (ssr_ok) {
                if (!have_precise) {
                    if (ssr_products_.orbitCorrectionsAreRac()) {
                        orbit_correction_ecef =
                            ssrRacToEcef(sat_position, sat_velocity, orbit_correction_ecef);
                    }
                    sat_position += orbit_correction_ecef;
                    sat_clock_bias += clock_correction_m / constants::SPEED_OF_LIGHT;
                }
                const double code_bias = observationCodeBiasMeters(
                    observation.satellite.system,
                    observation.primary_signal,
                    observation.secondary_signal,
                    ppp_config_.use_ionosphere_free,
                    code_bias_m,
                    observation.primary_code_bias_coeff,
                    observation.secondary_code_bias_coeff,
                    observation.primary_observation_type,
                    observation.secondary_observation_type,
                    madoca_bias_identity);
                // MADOCALIB adds SSR biases to the observables (ppp.c:432-451).
                // The MADOCA L6 path follows that convention by default;
                // GNSS_PPP_MADOCA_BIAS_SUBTRACT restores the legacy subtraction
                // for diagnostics. Non-MADOCA SSR paths keep subtracting.
                const double ssr_bias_sign =
                    (require_coherent_ssr_ && !env_overrides_.madoca_bias_subtract)
                        ? +1.0
                        : -1.0;
                observation.pseudorange_if += ssr_bias_sign * code_bias;
                observation.pseudorange_code_bias_m = code_bias;
                applied_ssr_code_bias = std::abs(code_bias) > 0.0;
                if (observation.has_carrier_phase && !phase_bias_m.empty()) {
                    const double phase_bias = observationPhaseBiasMeters(
                        observation.satellite.system,
                        observation.primary_signal,
                        observation.secondary_signal,
                        ppp_config_.use_ionosphere_free,
                        phase_bias_m,
                        observation.primary_code_bias_coeff,
                        observation.secondary_code_bias_coeff,
                        observation.primary_observation_type,
                        observation.secondary_observation_type,
                        madoca_bias_identity);
                    observation.carrier_phase_if += ssr_bias_sign * phase_bias;
                    observation.carrier_phase_bias_m = phase_bias;
                }
                // Per-frequency (est-stec) biases: apply the L1 and L2 satellite
                // code/phase biases to their own observables separately so the
                // per-frequency ambiguities carry the integer-recoverable bias.
                if (!ppp_config_.use_ionosphere_free) {
                    const double cb_l1 = observationCodeBiasMeters(
                        observation.satellite.system, observation.primary_signal,
                        SignalType::SIGNAL_TYPE_COUNT, false, code_bias_m, 1.0, 0.0,
                        observation.primary_observation_type,
                        std::string(),
                        madoca_bias_identity);
                    observation.pseudorange_l1 += ssr_bias_sign * cb_l1;
                    observation.code_bias_l1_m = cb_l1;
                    if (observation.has_l2) {
                        const double cb_l2 = observationCodeBiasMeters(
                            observation.satellite.system, observation.secondary_signal,
                            SignalType::SIGNAL_TYPE_COUNT, false, code_bias_m, 1.0, 0.0,
                            observation.secondary_observation_type,
                            std::string(),
                            madoca_bias_identity);
                        observation.pseudorange_l2 += ssr_bias_sign * cb_l2;
                        observation.code_bias_l2_m = cb_l2;
                    }
                    // MADOCALIB ppp.c:451 ADDS the SSR phase bias to the carrier
                    // phase (L[i]+=pb); applying it with the opposite sign pushes
                    // the per-frequency ambiguity away from its integer. Non-MADOCA
                    // paths keep the legacy subtraction unless GNSS_PPP_PB_ADD is
                    // set while their convention is validated.
                    const double pb_sign = env_overrides_.pb_add ? +1.0 : -1.0;
                    const double phase_bias_sign =
                        (require_coherent_ssr_ && !env_overrides_.madoca_bias_subtract)
                            ? +1.0
                            : pb_sign;
                    if (!phase_bias_m.empty() && !env_overrides_.no_phase_bias) {
                        if (observation.has_carrier_phase) {
                            const double pb_l1 = observationPhaseBiasMeters(
                                observation.satellite.system, observation.primary_signal,
                                SignalType::SIGNAL_TYPE_COUNT, false, phase_bias_m, 1.0, 0.0,
                                observation.primary_observation_type,
                                std::string(),
                                madoca_bias_identity);
                            observation.carrier_phase_l1 += phase_bias_sign * pb_l1;
                            observation.phase_bias_l1_m = pb_l1;
                            if (env_overrides_.pb_apply_dump) {
                                std::cerr << "[PB-APPLY] " << observation.satellite.toString()
                                          << " pb_l1=" << pb_l1 << "\n";
                            }
                        }
                        if (observation.has_carrier_phase_l2) {
                            const double pb_l2 = observationPhaseBiasMeters(
                                observation.satellite.system, observation.secondary_signal,
                                SignalType::SIGNAL_TYPE_COUNT, false, phase_bias_m, 1.0, 0.0,
                                observation.secondary_observation_type,
                                std::string(),
                                madoca_bias_identity);
                            observation.carrier_phase_l2 += phase_bias_sign * pb_l2;
                            observation.phase_bias_l2_m = pb_l2;
                        }
                    }
                    // Re-derive the ionosphere seed from bias-corrected codes.
                    if (observation.has_l2 && observation.pseudorange_l1 != 0.0 &&
                        observation.freq_l1 > 0.0 && observation.freq_l2 > 0.0) {
                        const double ratio = observation.freq_l1 / observation.freq_l2;
                        const double denom = 1.0 - ratio * ratio;
                        if (std::abs(denom) > 1e-6) {
                            observation.iono_init_m =
                                (observation.pseudorange_l1 - observation.pseudorange_l2) / denom;
                            observation.has_iono_init = true;
                        }
                    }
                }
                if (ura_sigma_m > 0.0 && std::isfinite(ura_sigma_m)) {
                    const double ura_variance = ura_sigma_m * ura_sigma_m;
                    deferred_variance_pr += ura_variance;
                    deferred_variance_cp += ura_variance;
                }
                const auto ssr_geometry = nav.calculateGeometry(receiver_position, sat_position);
                const double trop_correction_m =
                    ppp_atmosphere::atmosphericTroposphereCorrectionMeters(
                        atmos_tokens,
                        receiver_position,
                        time,
                        ssr_geometry.elevation,
                        ppp_config_.clas_expanded_value_construction_policy,
                        ppp_config_.clas_subtype12_value_construction_policy,
                        ppp_config_.clas_expanded_residual_sampling_policy);
                if (std::isfinite(trop_correction_m) && std::abs(trop_correction_m) > 0.0) {
                    observation.pseudorange_if -= trop_correction_m;
                    if (observation.has_carrier_phase) {
                        observation.carrier_phase_if -= trop_correction_m;
                    }
                    observation.atmospheric_trop_correction_m = trop_correction_m;
                    ++last_applied_atmos_trop_corrections_;
                    last_applied_atmos_trop_m_ += std::abs(trop_correction_m);
                }
                const double stec_tecu =
                    ppp_atmosphere::atmosphericStecTecu(
                        atmos_tokens,
                        observation.satellite,
                        receiver_position,
                        ppp_config_.clas_expanded_value_construction_policy,
                        ppp_config_.clas_subtype12_value_construction_policy,
                        ppp_config_.clas_expanded_residual_sampling_policy);
                if (pppDebugEnabled()) {
                    std::cerr << "[PPP-STEC] " << observation.satellite.toString()
                              << " stec_tecu=" << stec_tecu
                              << " atmos_tokens_size=" << atmos_tokens.size()
                              << " use_iflc=" << ppp_config_.use_ionosphere_free
                              << "\n";
                }
                const double ionosphere_correction_m = ppp_atmosphere::observationIonosphereDelayMeters(
                    eph,
                    observation.primary_signal,
                    observation.secondary_signal,
                    ppp_config_.use_ionosphere_free,
                    stec_tecu,
                    observation.primary_code_bias_coeff,
                    observation.secondary_code_bias_coeff);
                if (ppp_config_.estimate_ionosphere && std::isfinite(stec_tecu) &&
                    std::abs(stec_tecu) > 0.0) {
                    // In ionosphere estimation mode, inject STEC as a tight constraint
                    // on the ionosphere state instead of correcting observations.
                    const auto iono_it = filter_state_.ionosphere_indices.find(observation.satellite);
                    if (iono_it != filter_state_.ionosphere_indices.end()) {
                        const double iono_delay_m = ppp_atmosphere::ionosphereDelayMetersFromTecu(
                            observation.primary_signal, eph, stec_tecu);
                        if (std::isfinite(iono_delay_m)) {
                            const int idx = iono_it->second;
                            const double innovation = iono_delay_m - filter_state_.state(idx);
                            const double stec_sigma = 0.5;  // 50cm STEC constraint
                            const double S = filter_state_.covariance(idx, idx) + stec_sigma * stec_sigma;
                            if (S > 0.0) {
                                VectorXd K = filter_state_.covariance.col(idx) / S;
                                filter_state_.state += K * innovation;
                                filter_state_.covariance -= K * filter_state_.covariance.row(idx);
                            }
                        }
                    }
                    ++last_applied_atmos_iono_corrections_;
                    last_applied_atmos_iono_m_ += std::abs(ionosphere_correction_m);
                    applied_ssr_iono = true;
                } else if (std::isfinite(ionosphere_correction_m) &&
                    std::abs(ionosphere_correction_m) > 0.0) {
                    // Direct observation correction (non-estimation mode)
                    observation.pseudorange_if -= ionosphere_correction_m;
                    if (observation.has_carrier_phase) {
                        observation.carrier_phase_if += ionosphere_correction_m;
                    }
                    observation.atmospheric_iono_correction_m = ionosphere_correction_m;
                    ++last_applied_atmos_iono_corrections_;
                    last_applied_atmos_iono_m_ += std::abs(ionosphere_correction_m);
                    applied_ssr_iono = true;
                }
            }
        }

        // Satellite antenna PCO from ANTEX. The IGS final products since
        // the 2017 convention switch publish SP3 / CLK at the iono-free
        // combination antenna phase centre (so no shift is needed by
        // default). For SP3 sources known to report centre of mass,
        // setting apply_satellite_antenna_pco rotates the body-frame PCO
        // (constructed via the yaw-steering attitude) into ECEF and
        // shifts sat_position so geodist() returns the antenna range.
        if (satellite_antex_loaded_ && ppp_config_.apply_satellite_antenna_pco) {
            Vector3d pco_body_if = Vector3d::Zero();
            bool have_pco = false;
            for (const auto& entry : satellite_antex_offsets_) {
                if (!(entry.satellite == observation.satellite)) continue;
                if (entry.valid_from.week != 0 && time < entry.valid_from) continue;
                if (entry.valid_until.week != 0 && entry.valid_until < time) continue;
                const auto primary_it = entry.body_offsets_m.find(observation.primary_signal);
                if (primary_it == entry.body_offsets_m.end()) break;
                if (ppp_config_.use_ionosphere_free &&
                    observation.secondary_signal != SignalType::SIGNAL_TYPE_COUNT) {
                    const auto secondary_it =
                        entry.body_offsets_m.find(observation.secondary_signal);
                    if (secondary_it == entry.body_offsets_m.end()) break;
                    pco_body_if =
                        observation.primary_code_bias_coeff * primary_it->second +
                        observation.secondary_code_bias_coeff * secondary_it->second;
                } else {
                    pco_body_if = primary_it->second;
                }
                have_pco = true;
                break;
            }
            if (have_pco && sat_position.norm() > 1.0) {
                // Yaw-steering body frame: z toward Earth, y orthogonal to
                // the spacecraft–Sun plane, x toward the Sun side.
                const Vector3d e_z_sat = -sat_position.normalized();
                const Vector3d sun_los = sun_position_ecef - sat_position;
                if (sun_los.norm() > 1.0) {
                    Vector3d e_y_sat = e_z_sat.cross(sun_los).normalized();
                    if (e_y_sat.allFinite() && e_y_sat.norm() > 0.5) {
                        const Vector3d e_x_sat = e_y_sat.cross(e_z_sat);
                        const Vector3d pco_ecef =
                            pco_body_if.x() * e_x_sat +
                            pco_body_if.y() * e_y_sat +
                            pco_body_if.z() * e_z_sat;
                        sat_position += pco_ecef;
                    }
                }
            }
        }

        const auto geometry = nav.calculateGeometry(receiver_position, sat_position);
        if (!std::isfinite(geometry.distance) || geometry.elevation < elevation_mask) {
            observation.valid = false;
            continue;
        }

        if (!applied_ssr_code_bias && dcb_products_loaded_) {
            const double dcb_bias_m = observationDcbBiasMeters(
                dcb_products_,
                observation.satellite,
                observation.primary_signal,
                observation.secondary_signal,
                ppp_config_.use_ionosphere_free,
                observation.primary_code_bias_coeff,
                observation.secondary_code_bias_coeff);
            if (std::isfinite(dcb_bias_m) && std::abs(dcb_bias_m) > 0.0) {
                observation.pseudorange_if -= dcb_bias_m;
                observation.pseudorange_code_bias_m += dcb_bias_m;
                ++last_applied_dcb_corrections_;
                last_applied_dcb_m_ += std::abs(dcb_bias_m);
            }
        }

        // The dual-frequency ionosphere-free combination already cancels the
        // first-order ionospheric delay analytically, so subtracting an
        // IONEX-derived correction on top of an IF observable can only
        // introduce numerical-precision and TEC-mapping noise (a few cm
        // routinely observed at TSKB/GRAZ). Skip the subtraction in IF mode
        // unless the filter is estimating a per-satellite STEC state, in
        // which case the IONEX value is injected as a tight constraint
        // earlier in this loop rather than subtracted from observations.
        const bool ionex_applies_to_observation =
            !ppp_config_.use_ionosphere_free ||
            observation.secondary_signal == SignalType::SIGNAL_TYPE_COUNT ||
            ppp_config_.estimate_ionosphere;
        if (!applied_ssr_iono && ionex_products_loaded_ && ionex_applies_to_observation) {
            double ipp_lat_deg = 0.0;
            double ipp_lon_deg = 0.0;
            double mapping_factor = 0.0;
            double vertical_tecu = 0.0;
            if (ionexPiercePointAndMapping(
                    ionex_products_,
                    receiver_position,
                    geometry.azimuth,
                    geometry.elevation,
                    ipp_lat_deg,
                    ipp_lon_deg,
                    mapping_factor) &&
                ionex_products_.interpolateTecu(time, ipp_lat_deg, ipp_lon_deg, vertical_tecu, nullptr)) {
                const double stec_tecu = mapping_factor * vertical_tecu;
                const double ionosphere_correction_m = ppp_atmosphere::observationIonosphereDelayMeters(
                    eph,
                    observation.primary_signal,
                    observation.secondary_signal,
                    ppp_config_.use_ionosphere_free,
                    stec_tecu,
                    observation.primary_code_bias_coeff,
                    observation.secondary_code_bias_coeff);
                if (std::isfinite(ionosphere_correction_m) &&
                    std::abs(ionosphere_correction_m) > 0.0) {
                    observation.pseudorange_if -= ionosphere_correction_m;
                    if (observation.has_carrier_phase) {
                        observation.carrier_phase_if += ionosphere_correction_m;
                    }
                    observation.atmospheric_iono_correction_m += ionosphere_correction_m;
                    ++last_applied_atmos_iono_corrections_;
                    last_applied_atmos_iono_m_ += std::abs(ionosphere_correction_m);
                    ++last_applied_ionex_corrections_;
                    last_applied_ionex_m_ += std::abs(ionosphere_correction_m);
                }
            }
        }

        observation.satellite_position = sat_position;
        observation.satellite_velocity = sat_velocity;
        observation.satellite_clock_bias = sat_clock_bias;
        observation.satellite_clock_drift = sat_clock_drift;
        observation.elevation = geometry.elevation;
        observation.azimuth = geometry.azimuth;

        // Env-gated satpos dump for native-vs-bridge orbit/clock parity diff.
        // Emits the finalized satellite ECEF (broadcast + SSR orbit, evaluated
        // at emission time, APC frame — matching MADOCALIB satposs rs[]) so an
        // external script can match by epoch+sat and project the delta onto the
        // line of sight. clk in metres (sat_clock_bias * c).
        if (env_overrides_.satpos_dump) {
            std::cerr << "[SATPOS] tow=" << std::fixed << std::setprecision(3) << time.tow
                      << " sat=" << observation.satellite.toString()
                      << std::setprecision(4)
                      << " x=" << sat_position.x()
                      << " y=" << sat_position.y()
                      << " z=" << sat_position.z()
                      << " clk=" << (sat_clock_bias * constants::SPEED_OF_LIGHT)
                      << " el=" << std::setprecision(2) << geometry.elevation
                      << "\n";
        }

        observation.trop_mapping =
            calculateMappingFunction(receiver_position, geometry.elevation, time);
        // Per-frequency (est-stec): also compute the wet mapping and the a priori
        // zenith hydrostatic delay so the measurement model can map the estimated
        // wet residual with the correct (wet) mapping instead of hydrostatic.
        if (!ppp_config_.use_ionosphere_free && ppp_config_.estimate_ionosphere &&
            !ppp_config_.use_clas_osr_filter) {
            double lat = 0.0, lon = 0.0, hgt = 0.0;
            ecef2geodetic(receiver_position, lat, lon, hgt);
            observation.trop_mapping_wet =
                models::niellWetMapping(lat, geometry.elevation);
            observation.trop_zhd_m =
                models::estimateZenithTroposphereClimatology(lat, hgt, dayOfYearFromTime(time))
                    .hydrostatic_delay_m;
        }
        observation.modeled_trop_delay_m =
            modeledTroposphereDelayMeters(receiver_position, geometry.elevation, time);
        if (ppp_config_.use_rtklib_measurement_variance && !precise_products_loaded_) {
            observation.variance_pr = measurementVariance(observation, false);
            observation.variance_cp = measurementVariance(observation, true);
        }
        observation.variance_pr = safeVariance(observation.variance_pr + deferred_variance_pr, 1e-6);
        observation.variance_cp = safeVariance(observation.variance_cp + deferred_variance_cp, 1e-8);
        // Per-frequency (est-stec) code de-weighting: the uncombined L1/L2 code
        // carries a systematic (multipath / residual code-bias) error that, when
        // trusted as tightly as the elevation model implies, biases the converged
        // position (observed E+0.37 m at MIZU). The IFLC path implicitly de-weights
        // code via the 3x ionosphere-free noise amplification; mirror that here so
        // phase drives the converged solution. Env-overridable for tuning.
        if (!ppp_config_.use_ionosphere_free && ppp_config_.estimate_ionosphere &&
            !ppp_config_.use_clas_osr_filter) {
            observation.variance_pr *= env_overrides_.pf_code_var_scale;
        }
        observation.receiver_position = receiver_position;
        if (geometry.distance > 0.0) {
            const Vector3d los_unit = (sat_position - receiver_position) / geometry.distance;
            observation.antenna_pco_m = los_unit.dot(receiver_antenna_offset_ecef);
        } else {
            observation.antenna_pco_m = 0.0;
        }

        // Per-frequency receiver antenna model (est-stec). MADOCALIB applies a
        // per-frequency receiver PCO/PCV correction (posopt2=on) but no
        // satellite PCO/PCV (brdc+ssrapc -> opt=0; satantpcv() is disabled for
        // madoca-ppp). The shared receiver_position already carries the L1 PCO
        // via geodist(), so here we add only the residual per-frequency terms:
        //   L1 row: + PCV_L1(zen)
        //   L2 row: -los.(PCO_L2 - PCO_L1) + PCV_L2(zen)
        // Gated to the est-stec path; the IFLC path keeps its IF-combined PCO
        // position shift bit-identical (corrections stay 0).
        observation.rx_ant_corr_l1_m = 0.0;
        observation.rx_ant_corr_l2_m = 0.0;
        if (env_overrides_.pf_rx_antenna && receiver_antex_loaded_ &&
            !ppp_config_.use_ionosphere_free && ppp_config_.estimate_ionosphere &&
            !ppp_config_.use_clas_osr_filter && geometry.distance > 0.0 &&
            !ppp_config_.receiver_antenna_type.empty()) {
            const auto ant_it = receiver_antex_offsets_.find(
                normalizeAntennaType(ppp_config_.receiver_antenna_type));
            if (ant_it != receiver_antex_offsets_.end()) {
                double rx_lat = 0.0, rx_lon = 0.0, rx_h = 0.0;
                ecef2geodetic(receiver_marker_position, rx_lat, rx_lon, rx_h);
                const Vector3d los_unit =
                    (sat_position - receiver_position) / geometry.distance;
                observation.rx_ant_corr_l1_m =
                    receiverAntennaPcvMeters(observation.primary_signal, observation.elevation);
                if (observation.secondary_signal != SignalType::SIGNAL_TYPE_COUNT) {
                    const auto l1_it = ant_it->second.find(observation.primary_signal);
                    const auto l2_it = ant_it->second.find(observation.secondary_signal);
                    double pco_delta_m = 0.0;
                    if (l1_it != ant_it->second.end() && l2_it != ant_it->second.end()) {
                        const Vector3d delta_ecef =
                            enu2ecef(l2_it->second - l1_it->second, rx_lat, rx_lon);
                        pco_delta_m = -los_unit.dot(delta_ecef);
                    }
                    observation.rx_ant_corr_l2_m =
                        pco_delta_m +
                        receiverAntennaPcvMeters(observation.secondary_signal,
                                                 observation.elevation);
                }
            }
        }

        // Phase wind-up correction (Wu et al. 1993). Same cycle count for L1
        // and L2 — convert via the IF combination wavelength c/(f1+f2),
        // which is c1*λ1 + c2*λ2 with the standard IFLC coefficients.
        if (observation.has_carrier_phase) {
            const double previous_windup =
                windup_cache_.count(observation.satellite) ?
                    windup_cache_[observation.satellite] : 0.0;
            const double new_windup = ppp_utils::calculatePhaseWindup(
                receiver_position, sat_position, sun_position_ecef, previous_windup);
            windup_cache_[observation.satellite] = new_windup;

            double windup_wavelength_m = 0.0;
            if (ppp_config_.use_ionosphere_free &&
                observation.secondary_signal != SignalType::SIGNAL_TYPE_COUNT) {
                const double f1 = signalFrequencyHz(observation.primary_signal, eph);
                const double f2 = signalFrequencyHz(observation.secondary_signal, eph);
                if (f1 > 0.0 && f2 > 0.0 && f1 + f2 > 0.0) {
                    windup_wavelength_m = constants::SPEED_OF_LIGHT / (f1 + f2);
                }
            } else {
                windup_wavelength_m = signalWavelengthMeters(observation.primary_signal, eph);
            }
            if (windup_wavelength_m > 0.0 && std::isfinite(new_windup)) {
                const double windup_correction_m = new_windup * windup_wavelength_m;
                observation.carrier_phase_if -= windup_correction_m;
            }
            // Per-frequency windup: the phase windup is geometric (in cycles),
            // so each frequency's correction scales with its own wavelength.
            if (!ppp_config_.use_ionosphere_free && std::isfinite(new_windup)) {
                if (observation.has_carrier_phase && observation.wavelength_l1 > 0.0) {
                    observation.carrier_phase_l1 -= new_windup * observation.wavelength_l1;
                }
                if (observation.has_carrier_phase_l2 && observation.wavelength_l2 > 0.0) {
                    observation.carrier_phase_l2 -= new_windup * observation.wavelength_l2;
                }
            }
        }

        // In CLAS-PPP mode (estimate_ionosphere), reject satellites without
        // both SSR orbit/clock and ionosphere corrections — matching CLASLIB's
        // corrmeas() which returns 0 when STEC correction fails.
        observation.valid = true;
    }
}

double PPPProcessor::measurementVariance(const IonosphereFreeObs& observation,
                                         bool carrier_phase) const {
    const double sin_elevation = std::sin(std::max(observation.elevation, kRtkLibMinElevationRadians));
    double factor = rtklibSystemErrorFactor(observation.satellite.system);
    if (usesGpsL5ErrorFactor(observation.primary_signal) ||
        usesGpsL5ErrorFactor(observation.secondary_signal)) {
        factor *= kRtkLibGpsL5ErrorFactor;
    }
    if (ppp_config_.use_ionosphere_free) {
        factor *= 3.0;
    }
    if (!carrier_phase) {
        const double ratio =
            observation.secondary_signal == SignalType::SIGNAL_TYPE_COUNT ?
                ppp_config_.code_phase_error_ratio_l1 :
                ppp_config_.code_phase_error_ratio_l2;
        factor *= ratio > 0.0 ? ratio : ppp_config_.code_phase_error_ratio_l1;
    }
    return safeVariance(
        std::pow(factor * ppp_config_.rtklib_phase_error_m, 2.0) +
            std::pow(factor * ppp_config_.rtklib_phase_error_elevation_m / sin_elevation, 2.0),
        carrier_phase ? 1e-8 : 1e-6);
}

double PPPProcessor::calculateTroposphericDelay(const Vector3d& receiver_pos,
                                                const Vector3d& satellite_pos,
                                                const GNSSTime& time,
                                                double zenith_delay) const {
    double lat = 0.0;
    double lon = 0.0;
    double h = 0.0;
    ecef2geodetic(receiver_pos, lat, lon, h);
    const Vector3d los_enu = ecef2enu(satellite_pos - receiver_pos, lat, lon);
    const double horizontal = std::hypot(los_enu.x(), los_enu.y());
    const double elevation = std::atan2(los_enu.z(), horizontal);
    if (!ppp_config_.estimate_troposphere) {
        return modeledTroposphereDelayMeters(receiver_pos, elevation, time);
    }
    const double mapping = calculateMappingFunction(receiver_pos, elevation, time);
    return mapping * zenith_delay;
}

double PPPProcessor::calculateMappingFunction(const Vector3d& receiver_pos,
                                              double elevation,
                                              const GNSSTime& time) const {
    double lat = 0.0;
    double lon = 0.0;
    double h = 0.0;
    ecef2geodetic(receiver_pos, lat, lon, h);
    const double hydrostatic =
        models::niellHydrostaticMapping(lat, h, elevation, dayOfYearFromTime(time));
    // Hydrostatic (dry) mapping only — the KF tracks a single zenith delay
    // and the hydrostatic component dominates (2.3m vs 0.25m wet).
    return hydrostatic;
}

Vector3d PPPProcessor::applyGeophysicalCorrections(const Vector3d& position,
                                                   const GNSSTime& time) const {
    Vector3d corrected = position;
    // GNSS_PPP_NO_SOLID_TIDE: opt-in env knob to completely skip solid-earth
    // tide displacement. Diagnostic only — used to bisect the MADOCA est-stec
    // MIZU 7.5x gap vs bridge (cm-level systematic differences across the
    // three solid-tide implementations: legacy Step-1-Love, IERS Step-1+2,
    // and MADOCALIB Step-1+K1-only). Default OFF (bit-identical).
    if (ppp_config_.apply_solid_earth_tides && !env_overrides_.no_solid_tide) {
        corrected += calculateSolidEarthTides(position, time);
    }
    if (ppp_config_.apply_ocean_loading) {
        corrected += calculateOceanLoading(position, time);
    }
    if (ppp_config_.use_iers_pole_tide) {
        corrected += calculatePoleTide(position, time);
    }
    if (ppp_config_.use_iers_atm_tidal_loading) {
        corrected += calculateAtmosphericTidalLoading(position, time);
    }
    return corrected;
}

Vector3d PPPProcessor::calculateSolidEarthTides(const Vector3d& position,
                                                const GNSSTime& time) const {
    if (!position.allFinite() || position.norm() < constants::WGS84_A * 0.5) {
        return Vector3d::Zero();
    }

    if (ppp_config_.use_iers_solid_tide) {
        // IERS Conventions 2010 §7.1.1 (Dehant) Step-1 + Step-2 model
        // via the libgnss::iers wrapper. Sun and Moon are supplied in
        // ICRS — see the FRAME NOTE in libgnss++/iers/tides.hpp for
        // why this is the correct frame for the IERS routine despite
        // its public documentation calling it "ECEF".
        const double mjd_utc = libgnss::iers::gnssTimeToMjdUtc(time);
        const Vector3d sun_icrs  = libgnss::iers::sunPositionIcrs(mjd_utc);
        const Vector3d moon_icrs = libgnss::iers::moonPositionIcrs(mjd_utc);
        return libgnss::iers::solidEarthTideDisplacement(
            mjd_utc, position, sun_icrs, moon_icrs);
    }

    // Default path: simplified Step-1-only Love-number body-tide
    // approximation kept for behavioral compatibility while the
    // IERS path stays opt-in pending truth-bench validation. See
    // docs/iers-integration-plan.md.
    constexpr double kSunGM  = 1.32712440018e20;
    constexpr double kMoonGM = 4.902801e12;
    const Vector3d sun_position  = approximateSunPositionEcef(time);
    const Vector3d moon_position = approximateMoonPositionEcef(time);
    return bodyTideDisplacement(position, sun_position, kSunGM) +
           bodyTideDisplacement(position, moon_position, kMoonGM);
}

Vector3d PPPProcessor::calculateOceanLoading(const Vector3d& position,
                                             const GNSSTime& time) const {
    if (!ocean_loading_loaded_ || !position.allFinite() ||
        position.norm() < constants::WGS84_A * 0.5) {
        return Vector3d::Zero();
    }

    double up_m = 0.0;
    double west_m = 0.0;
    double south_m = 0.0;

    if (ppp_config_.use_iers_ocean_loading) {
        // IERS Conventions 2010 §7.1.2 HARDISP path: spline-interpolated
        // admittance over 342 reference harmonics, with proper
        // astronomical (Doodson) phase reference. The wrapper takes a
        // BLQ in the libgnss::iers struct shape; we copy the existing
        // PPPProcessor cache (same 11-constituent BLQ block) into it.
        libgnss::iers::OceanLoadingBlq blq;
        for (std::size_t i = 0; i < blq.radial_amplitudes_m.size(); ++i) {
            blq.radial_amplitudes_m[i] =
                ocean_loading_coefficients_.up_amplitudes_m[i];
            blq.west_amplitudes_m[i] =
                ocean_loading_coefficients_.west_amplitudes_m[i];
            blq.south_amplitudes_m[i] =
                ocean_loading_coefficients_.south_amplitudes_m[i];
            blq.radial_phases_deg[i] =
                ocean_loading_coefficients_.up_phases_deg[i];
            blq.west_phases_deg[i] =
                ocean_loading_coefficients_.west_phases_deg[i];
            blq.south_phases_deg[i] =
                ocean_loading_coefficients_.south_phases_deg[i];
        }
        const double mjd_utc = libgnss::iers::gnssTimeToMjdUtc(time);
        const Vector3d disp_local =
            libgnss::iers::oceanLoadingDisplacement(mjd_utc, blq);
        up_m    = disp_local.x();
        west_m  = disp_local.y();
        south_m = disp_local.z();
    } else {
        // Legacy direct-sum path (kept as the default for now). Note
        // that this uses Unix epoch as the phase reference rather than
        // the proper IERS Doodson astronomical argument, so it is best
        // viewed as a placeholder; the IERS HARDISP path above is the
        // physically correct routine.
        const auto system_time = time.toSystemTime();
        const double seconds_since_unix =
            std::chrono::duration<double>(system_time.time_since_epoch()).count();
        for (size_t i = 0; i < kOceanLoadingPeriodsSeconds.size(); ++i) {
            const double period_seconds = kOceanLoadingPeriodsSeconds[i];
            if (period_seconds <= 0.0) {
                continue;
            }
            const double angle_rad = 2.0 * M_PI * seconds_since_unix / period_seconds;
            up_m += ocean_loading_coefficients_.up_amplitudes_m[i] *
                std::cos(angle_rad - ocean_loading_coefficients_.up_phases_deg[i] * kDegreesToRadians);
            west_m += ocean_loading_coefficients_.west_amplitudes_m[i] *
                std::cos(angle_rad - ocean_loading_coefficients_.west_phases_deg[i] * kDegreesToRadians);
            south_m += ocean_loading_coefficients_.south_amplitudes_m[i] *
                std::cos(angle_rad - ocean_loading_coefficients_.south_phases_deg[i] * kDegreesToRadians);
        }
    }

    double latitude_rad = 0.0;
    double longitude_rad = 0.0;
    double height_m = 0.0;
    ecef2geodetic(position, latitude_rad, longitude_rad, height_m);
    const Vector3d enu_offset(-west_m, -south_m, up_m);
    return enu2ecef(enu_offset, latitude_rad, longitude_rad);
}

Vector3d PPPProcessor::calculatePoleTide(const Vector3d& position,
                                         const GNSSTime& time) const {
    if (!eop_table_ || !position.allFinite() ||
        position.norm() < constants::WGS84_A * 0.5) {
        return Vector3d::Zero();
    }
    const double mjd_utc = libgnss::iers::gnssTimeToMjdUtc(time);
    libgnss::iers::EarthOrientationParams eop;
    try {
        // getEarthOrientationParams applies the sub-daily correction
        // when ppp_config_.use_iers_sub_daily_eop is enabled — pole
        // tide automatically picks up the higher-frequency CIP wobble
        // when both flags are on.
        eop = getEarthOrientationParams(time);
    } catch (const std::out_of_range&) {
        return Vector3d::Zero();
    }
    return libgnss::iers::poleTideDisplacement(mjd_utc, position, eop);
}

Vector3d PPPProcessor::calculateAtmosphericTidalLoading(
    const Vector3d& position, const GNSSTime& time) const {
    if (!atm_tidal_loading_loaded_ || !position.allFinite() ||
        position.norm() < constants::WGS84_A * 0.5) {
        return Vector3d::Zero();
    }
    const double mjd_utc = libgnss::iers::gnssTimeToMjdUtc(time);
    return libgnss::iers::atmosphericTidalLoadingDisplacement(
        mjd_utc, position, atm_tidal_loading_coefficients_);
}

}  // namespace libgnss
