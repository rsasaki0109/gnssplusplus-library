#include <libgnss++/core/navigation.hpp>
#include <libgnss++/algorithms/ppp_env_overrides.hpp>
#include <algorithm>
#include <cmath>
#include <cctype>
#include <ctime>
#include <iostream>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "navigation_internal.hpp"

namespace libgnss {

using namespace navigation_internal;

bool Ephemeris::calculateSatelliteState(const GNSSTime& time,
                                       Vector3d& pos,
                                       Vector3d& vel,
                                       double& clock_bias,
                                       double& clock_drift,
                                       bool use_mrtklib_galileo_mu) const {
    if (satellite.system == GNSSSystem::GLONASS) {
        return computeGlonassState(*this, time, pos, vel, clock_bias, clock_drift);
    }
    if (satellite.system == GNSSSystem::SBAS) {
        return computeSbasState(*this, time, pos, vel, clock_bias, clock_drift);
    }

    if (!computeBroadcastState(*this, time, pos, clock_bias, clock_drift,
                               use_mrtklib_galileo_mu)) {
        return false;
    }

    Vector3d pos_forward;
    Vector3d pos_backward;
    double clk_forward = 0.0;
    double clk_backward = 0.0;
    double drift_dummy = 0.0;
    const double dt = 0.5;

    if (computeBroadcastState(*this, time + dt, pos_forward, clk_forward,
                              drift_dummy, use_mrtklib_galileo_mu) &&
        computeBroadcastState(*this, time - dt, pos_backward, clk_backward,
                              drift_dummy, use_mrtklib_galileo_mu)) {
        vel = (pos_forward - pos_backward) / (2.0 * dt);
        clock_drift = (clk_forward - clk_backward) / (2.0 * dt);
    } else {
        vel.setZero();
    }

    return true;
}

bool Ephemeris::isValid(const GNSSTime& time) const {
    if (!valid) return false;
    if (satellite.system == GNSSSystem::GLONASS) {
        return std::abs(time - toe) <= 1800.0;
    }
    if (satellite.system == GNSSSystem::SBAS) {
        // RTKLIB seleph uses MAXDTOE (7200 s) for SBAS GEO ephemerides.
        return std::abs(time - toe) <= 7200.0;
    }
    double age = std::abs(time - toe);
    return age <= 14400.0; // 4 hours (RTKLIB MAXDTOE=7200 but relaxed for sparse nav data)
}

double Ephemeris::getAge(const GNSSTime& time) const {
    return std::abs(time - toe);
}

void NavigationData::addEphemeris(const Ephemeris& eph) {
    ephemeris_data[eph.satellite].push_back(eph);
    satellite_state_cache_.clear();
    
    // Sort by time of ephemeris
    auto& eph_list = ephemeris_data[eph.satellite];
    std::sort(eph_list.begin(), eph_list.end(),
              [](const Ephemeris& a, const Ephemeris& b) {
                  return a.toe < b.toe;
              });
}

namespace {
// RTKLIB seleph() applies two Galileo-specific selection rules that native
// otherwise lacks (it picks purely the age-closest valid record):
//   1. Data-source filter: with the default selection (eph_sel[SYS_GAL]=0) keep
//      only I/NAV records, recognised by bit 9 of the "data sources" word
//      (af0-af2/Toc referenced to E5b,E1). BRDM-style merged nav files carry
//      both I/NAV (513/516/517) and F/NAV (258) per satellite; without this
//      native lands on F/NAV ~half the time (clock-reference mismatch).
//   2. No-future rule: skip any record whose toe is at or after the query time
//      (RTKLIB `timediff(eph->toe,time)>=0.0`), so the most-recent already-in-
//      effect ephemeris is used rather than the age-closest (which near a 600 s
//      toe boundary is the next, future record — a ~0.5 m along-track shift).
// Gated to Galileo so the data-source word — which means other things for
// GPS/QZSS/BDS — is ignored elsewhere. Env override for A/B (default ON).
//
// DEFAULT OFF (opt-in via GNSS_PPP_GAL_INAV=1): although this matches RTKLIB
// seleph and makes the Galileo clock reference consistent with the bridge
// (per-sat clk std 0.25 m -> 0.005 m), enabling it REGRESSES position on the
// MADOCA parity datasets (MIZU 3D 0.315 -> 0.469, ALIC 0.390 -> 1.087): the
// age-nearest F/NAV-mixing the native estimator currently does happens to
// compensate for an estimator-layer Galileo bias, so correcting only the input
// removes the compensation. It also does NOT close the ~0.5 m tangential
// Galileo orbit gap (the toe rule made no difference), whose root cause is the
// IODE-match interacting with the I/NAV filter (separate, unsolved, ~5 cm LOS
// impact). Kept as an opt-in for when the estimator layer is addressed.
inline bool galileoSkip(const SatelliteId& sat, const Ephemeris& eph,
                        const GNSSTime& time) {
    if (!pppEnvOverrides().gal_inav || sat.system != GNSSSystem::Galileo) {
        return false;
    }
    constexpr int kGalInavClockBit = 1 << 9;
    if (!(eph.data_source_code & kGalInavClockBit)) {
        return true;  // not I/NAV
    }
    if (eph.toe - time >= 0.0) {
        return true;  // future ephemeris
    }
    return false;
}

// RTKLIB/MADOCALIB seleph() treats the Compact SSR BeiDou IODE as an
// eight-second toe index rather than comparing it with the RINEX AODE field.
// Other broadcast constellations compare the ephemeris IODE directly.
inline bool ephemerisMatchesSsrIode(const SatelliteId& sat,
                                    const Ephemeris& eph,
                                    int desired_iode) {
    if (sat.system == GNSSSystem::BeiDou) {
        constexpr int kBeiDouIodeCycleSeconds = 2048;
        const int toe_seconds = static_cast<int>(eph.toes);
        const int correction_toe_seconds = desired_iode * 8;
        return toe_seconds % kBeiDouIodeCycleSeconds ==
               correction_toe_seconds % kBeiDouIodeCycleSeconds;
    }
    return static_cast<int>(eph.iode) == desired_iode;
}
}  // namespace

const Ephemeris* NavigationData::getEphemeris(const SatelliteId& sat, const GNSSTime& time) const {
    auto it = ephemeris_data.find(sat);
    if (it == ephemeris_data.end()) {
        return nullptr;
    }
    
    const Ephemeris* best_eph = nullptr;
    double min_age = 1e9;

    for (const auto& eph : it->second) {
        if (galileoSkip(sat, eph, time)) {
            continue;
        }
        if (eph.isValid(time)) {
            double age = eph.getAge(time);
            if (age < min_age) {
                min_age = age;
                best_eph = &eph;
            }
        }
    }

    return best_eph;
}

const Ephemeris* NavigationData::getEphemeris(const SatelliteId& sat,
                                              const GNSSTime& time,
                                              int desired_iode) const {
    if (desired_iode < 0) {
        return getEphemeris(sat, time);
    }
    auto it = ephemeris_data.find(sat);
    if (it == ephemeris_data.end()) {
        return nullptr;
    }
    // Require a valid ephemeris whose IODE matches the SSR orbit correction's
    // reference IODE (RTKLIB/MADOCALIB seleph() behaviour). Applying an SSR
    // delta to a different broadcast orbit can introduce metre-level errors.
    // Among matches pick the freshest (smallest age); do not fall back to an
    // IODE-agnostic ephemeris when the referenced record is unavailable.
    const Ephemeris* best_match = nullptr;
    double min_age = 1e9;
    for (const auto& eph : it->second) {
        if (galileoSkip(sat, eph, time)) {
            continue;
        }
        if (!ephemerisMatchesSsrIode(sat, eph, desired_iode)) {
            continue;
        }
        if (!eph.isValid(time)) {
            continue;
        }
        const double age = eph.getAge(time);
        if (age < min_age) {
            min_age = age;
            best_match = &eph;
        }
    }
    return best_match;
}

std::vector<Ephemeris> NavigationData::getEphemeris(
    const SatelliteId& sat) const {
    const auto it = ephemeris_data.find(sat);
    return it == ephemeris_data.end() ? std::vector<Ephemeris>{} : it->second;
}

bool NavigationData::hasMadocaGalileoEphemeris(const SatelliteId& sat,
                                               const GNSSTime& time,
                                               int desired_iode) const {
    if (sat.system != GNSSSystem::Galileo) {
        return true;
    }
    auto it = ephemeris_data.find(sat);
    if (it == ephemeris_data.end()) {
        return false;
    }
    constexpr int kGalInavClockBit = 1 << 9;
    for (const auto& eph : it->second) {
        if (desired_iode >= 0 &&
            !ephemerisMatchesSsrIode(sat, eph, desired_iode)) {
            continue;
        }
        if (!(eph.data_source_code & kGalInavClockBit)) {
            continue;
        }
        if (eph.toe - time >= 0.0) {
            continue;
        }
        if (!eph.isValid(time)) {
            continue;
        }
        return true;
    }
    return false;
}

bool NavigationData::calculateSatelliteState(const SatelliteId& sat,
                                           const GNSSTime& time,
                                           Vector3d& position,
                                           Vector3d& velocity,
                                           double& clock_bias,
                                           double& clock_drift) const {
    const SatelliteStateCacheKey cache_key{sat, time, -1};
    const auto cache_it = satellite_state_cache_.find(cache_key);
    if (cache_it != satellite_state_cache_.end()) {
        const SatelliteStateCacheValue& cached = cache_it->second;
        position = cached.position;
        velocity = cached.velocity;
        clock_bias = cached.clock_bias;
        clock_drift = cached.clock_drift;
        return cached.valid;
    }

    SatelliteStateCacheValue computed;
    const Ephemeris* eph = getEphemeris(sat, time);
    if (eph) {
        computed.valid =
            eph->calculateSatelliteState(
                time,
                computed.position,
                computed.velocity,
                computed.clock_bias,
                computed.clock_drift);
    }
    satellite_state_cache_.emplace(cache_key, computed);
    position = computed.position;
    velocity = computed.velocity;
    clock_bias = computed.clock_bias;
    clock_drift = computed.clock_drift;
    return computed.valid;
}

bool NavigationData::calculateSatelliteState(const SatelliteId& sat,
                                           const GNSSTime& time,
                                           Vector3d& position,
                                           Vector3d& velocity,
                                           double& clock_bias,
                                           double& clock_drift,
                                           int desired_iode) const {
    const SatelliteStateCacheKey cache_key{sat, time, desired_iode};
    const auto cache_it = satellite_state_cache_.find(cache_key);
    if (cache_it != satellite_state_cache_.end()) {
        const SatelliteStateCacheValue& cached = cache_it->second;
        position = cached.position;
        velocity = cached.velocity;
        clock_bias = cached.clock_bias;
        clock_drift = cached.clock_drift;
        return cached.valid;
    }

    SatelliteStateCacheValue computed;
    const Ephemeris* eph = getEphemeris(sat, time, desired_iode);
    if (eph) {
        computed.valid =
            eph->calculateSatelliteState(
                time,
                computed.position,
                computed.velocity,
                computed.clock_bias,
                computed.clock_drift);
    }
    satellite_state_cache_.emplace(cache_key, computed);
    position = computed.position;
    velocity = computed.velocity;
    clock_bias = computed.clock_bias;
    clock_drift = computed.clock_drift;
    return computed.valid;
}

std::map<SatelliteId, Vector3d> NavigationData::calculateSatellitePositions(
    const std::vector<SatelliteId>& satellites,
    const GNSSTime& time) const {
    std::map<SatelliteId, Vector3d> positions;
    
    for (const auto& sat : satellites) {
        Vector3d pos, vel;
        double clock_bias, clock_drift;
        if (calculateSatelliteState(sat, time, pos, vel, clock_bias, clock_drift)) {
            positions[sat] = pos;
        }
    }
    
    return positions;
}

NavigationData::SatelliteGeometry NavigationData::calculateGeometry(
    const Vector3d& receiver_pos,
    const Vector3d& satellite_pos) const {
    SatelliteGeometry geom;
    
    Vector3d los = satellite_pos - receiver_pos;
    geom.distance = los.norm();
    
    double lat = 0.0;
    double lon = 0.0;
    double height = 0.0;
    ecefToGeodetic(receiver_pos, lat, lon, height);

    const double sin_lat = std::sin(lat);
    const double cos_lat = std::cos(lat);
    const double sin_lon = std::sin(lon);
    const double cos_lon = std::cos(lon);

    Vector3d east(-sin_lon, cos_lon, 0.0);
    Vector3d north(-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat);
    Vector3d up(cos_lat * cos_lon, cos_lat * sin_lon, sin_lat);

    Vector3d los_local(los.dot(east), los.dot(north), los.dot(up));
    
    geom.elevation = std::atan2(los_local(2), std::sqrt(los_local(0)*los_local(0) + los_local(1)*los_local(1)));
    geom.azimuth = std::atan2(los_local(0), los_local(1));
    
    if (geom.azimuth < 0) {
        geom.azimuth += 2.0 * M_PI;
    }
    
    return geom;
}

bool NavigationData::hasEphemeris(const SatelliteId& sat, const GNSSTime& time) const {
    return getEphemeris(sat, time) != nullptr;
}

std::vector<SatelliteId> NavigationData::getAvailableSatellites(const GNSSTime& time) const {
    std::vector<SatelliteId> satellites;
    
    for (const auto& pair : ephemeris_data) {
        if (hasEphemeris(pair.first, time)) {
            satellites.push_back(pair.first);
        }
    }
    
    return satellites;
}

NavigationData::NavigationData() {
    clear();
}

void NavigationData::clear() {
    ephemeris_data.clear();
    satellite_state_cache_.clear();
}

bool NavigationData::isEmpty() const {
    return ephemeris_data.empty();
}

double IonosphereModel::calculateDelay(const GNSSTime& time,
                                     const GeodeticCoord& user_pos,
                                     const Vector3d& sat_pos,
                                     double frequency) const {
    if (!valid) return 0.0;
    
    // Simplified Klobuchar model
    double delay = 0.0;
    
    // Would implement full ionospheric delay calculation here
    // For now, return a simple frequency-dependent delay
    double f1 = constants::GPS_L1_FREQ;
    delay = 5.0 * (f1 * f1) / (frequency * frequency); // meters
    
    return delay;
}

double TroposphereModel::calculateDelay(const GeodeticCoord& user_pos,
                                      double elevation,
                                      const GNSSTime& time) const {
    // Simplified Saastamoinen model
    double height = user_pos.height;
    double lat = user_pos.latitude;
    
    // Pressure at sea level
    double P0 = 1013.25; // mbar
    
    // Pressure at user height
    double P = P0 * std::pow(1.0 - 2.26e-5 * height, 5.225);
    
    // Temperature
    double T = 288.15 - 6.5e-3 * height; // K
    
    // Water vapor pressure (simplified)
    double e = 6.108 * std::exp(17.15 * T / (234.7 + T));
    
    // Zenith delays
    double Zd = 0.002277 * (P + (1255.0/T + 0.05) * e);
    
    // Mapping function (simplified)
    double mapping = 1.0 / std::sin(elevation);
    if (mapping > 10.0) mapping = 10.0;
    
    return Zd * mapping;
}
} // namespace libgnss
