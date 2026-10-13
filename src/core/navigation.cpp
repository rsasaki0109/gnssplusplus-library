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

namespace {
// Three-way compare of doubles with a defined place for NaN (last), so the
// ordering below is a strict weak ordering for every input.
inline int compareDouble(double a, double b) {
    const bool a_nan = std::isnan(a);
    const bool b_nan = std::isnan(b);
    if (a_nan || b_nan) {
        return a_nan == b_nan ? 0 : (a_nan ? 1 : -1);
    }
    return a < b ? -1 : (a > b ? 1 : 0);
}

inline int compareTime(const GNSSTime& a, const GNSSTime& b) {
    if (a.week != b.week) {
        return a.week < b.week ? -1 : 1;
    }
    return compareDouble(a.tow, b.tow);
}

template <typename T>
inline int compareValue(const T& a, const T& b) {
    return a < b ? -1 : (b < a ? 1 : 0);
}

// Galileo only: I/NAV records (data-source bit 9, clock referenced to E5b/E1)
// rank before records without it (F/NAV). Other systems all rank equal because
// the data-source word means other things there.
inline int ephemerisSourceRank(const Ephemeris& eph) {
    if (eph.satellite.system != GNSSSystem::Galileo) {
        return 0;
    }
    return (eph.data_source_code & galileo_data_source::kClockE5bE1) ? 0 : 1;
}

// Total order over the records of one satellite, independent of insertion
// order. Primary key is toe (as before); the remaining keys only break ties,
// and run through every field that affects position/clock so that two records
// compare equal only when they are interchangeable.
int compareEphemerisOrder(const Ephemeris& a, const Ephemeris& b) {
    int c;
#define LIBGNSS_EPH_KEY(expr) \
    if ((c = (expr)) != 0) return c
    LIBGNSS_EPH_KEY(compareTime(a.toe, b.toe));
    LIBGNSS_EPH_KEY(compareValue(ephemerisSourceRank(a), ephemerisSourceRank(b)));
    LIBGNSS_EPH_KEY(compareValue(a.data_source_code, b.data_source_code));
    LIBGNSS_EPH_KEY(compareValue(a.iode, b.iode));
    LIBGNSS_EPH_KEY(compareValue(a.iodc, b.iodc));
    LIBGNSS_EPH_KEY(compareTime(a.toc, b.toc));
    LIBGNSS_EPH_KEY(compareDouble(a.af0, b.af0));
    LIBGNSS_EPH_KEY(compareDouble(a.af1, b.af1));
    LIBGNSS_EPH_KEY(compareDouble(a.af2, b.af2));
    // Remaining fields: only reached for records identical in every key above.
    LIBGNSS_EPH_KEY(compareTime(a.tof, b.tof));
    LIBGNSS_EPH_KEY(compareDouble(a.toes, b.toes));
    LIBGNSS_EPH_KEY(compareValue(a.week, b.week));
    LIBGNSS_EPH_KEY(compareValue(a.health, b.health));
    LIBGNSS_EPH_KEY(compareDouble(a.sv_health, b.sv_health));
    LIBGNSS_EPH_KEY(compareDouble(a.sv_accuracy, b.sv_accuracy));
    LIBGNSS_EPH_KEY(compareValue(a.ura, b.ura));
    LIBGNSS_EPH_KEY(compareDouble(a.sqrt_a, b.sqrt_a));
    LIBGNSS_EPH_KEY(compareDouble(a.e, b.e));
    LIBGNSS_EPH_KEY(compareDouble(a.i0, b.i0));
    LIBGNSS_EPH_KEY(compareDouble(a.omega0, b.omega0));
    LIBGNSS_EPH_KEY(compareDouble(a.omega, b.omega));
    LIBGNSS_EPH_KEY(compareDouble(a.m0, b.m0));
    LIBGNSS_EPH_KEY(compareDouble(a.delta_n, b.delta_n));
    LIBGNSS_EPH_KEY(compareDouble(a.idot, b.idot));
    LIBGNSS_EPH_KEY(compareDouble(a.i_dot, b.i_dot));
    LIBGNSS_EPH_KEY(compareDouble(a.omega_dot, b.omega_dot));
    LIBGNSS_EPH_KEY(compareDouble(a.cuc, b.cuc));
    LIBGNSS_EPH_KEY(compareDouble(a.cus, b.cus));
    LIBGNSS_EPH_KEY(compareDouble(a.crc, b.crc));
    LIBGNSS_EPH_KEY(compareDouble(a.crs, b.crs));
    LIBGNSS_EPH_KEY(compareDouble(a.cic, b.cic));
    LIBGNSS_EPH_KEY(compareDouble(a.cis, b.cis));
    LIBGNSS_EPH_KEY(compareDouble(a.tgd, b.tgd));
    LIBGNSS_EPH_KEY(compareDouble(a.tgd_secondary, b.tgd_secondary));
    LIBGNSS_EPH_KEY(compareDouble(a.glonass_taun, b.glonass_taun));
    LIBGNSS_EPH_KEY(compareDouble(a.glonass_gamn, b.glonass_gamn));
    for (int i = 0; i < 3; ++i) {
        LIBGNSS_EPH_KEY(compareDouble(a.glonass_position[i], b.glonass_position[i]));
        LIBGNSS_EPH_KEY(compareDouble(a.glonass_velocity[i], b.glonass_velocity[i]));
        LIBGNSS_EPH_KEY(compareDouble(a.glonass_acceleration[i], b.glonass_acceleration[i]));
    }
    LIBGNSS_EPH_KEY(compareValue(a.glonass_frequency_channel, b.glonass_frequency_channel));
    LIBGNSS_EPH_KEY(compareValue(a.glonass_frequency_channel_present,
                                 b.glonass_frequency_channel_present));
    LIBGNSS_EPH_KEY(compareValue(a.glonass_canonical_geph_data_valid,
                                 b.glonass_canonical_geph_data_valid));
    LIBGNSS_EPH_KEY(compareValue(a.glonass_canonical_geph_reject_reason,
                                 b.glonass_canonical_geph_reject_reason));
    LIBGNSS_EPH_KEY(compareValue(a.glonass_age, b.glonass_age));
    LIBGNSS_EPH_KEY(compareValue(static_cast<int>(a.navigation_message_type),
                                 static_cast<int>(b.navigation_message_type)));
    LIBGNSS_EPH_KEY(compareValue(a.valid, b.valid));
    LIBGNSS_EPH_KEY(compareValue(a.glonass_cdma_data.has_value(),
                                 b.glonass_cdma_data.has_value()));
    if (a.glonass_cdma_data && b.glonass_cdma_data) {
        LIBGNSS_EPH_KEY(compareDouble(a.glonass_cdma_data->beta, b.glonass_cdma_data->beta));
        LIBGNSS_EPH_KEY(compareValue(a.glonass_cdma_data->data_validity,
                                     b.glonass_cdma_data->data_validity));
        LIBGNSS_EPH_KEY(compareValue(a.glonass_cdma_data->satellite_type,
                                     b.glonass_cdma_data->satellite_type));
        LIBGNSS_EPH_KEY(compareValue(a.glonass_cdma_data->source_flags,
                                     b.glonass_cdma_data->source_flags));
    }
#undef LIBGNSS_EPH_KEY
    return 0;
}
}  // namespace

void NavigationData::addEphemeris(const Ephemeris& eph) {
    ephemeris_data[eph.satellite].push_back(eph);
    satellite_state_cache_.clear();
    ++revision_;

    // Keep each satellite's records in a total order that does not depend on
    // insertion order (see compareEphemerisOrder): toe first, then, for
    // Galileo, I/NAV before F/NAV, then the remaining tie-break keys.
    // getEphemeris() takes the first of several equal-age candidates, so the
    // order decides which copy of a same-toe record is used.
    auto& eph_list = ephemeris_data[eph.satellite];
    std::stable_sort(eph_list.begin(), eph_list.end(),
                     [](const Ephemeris& a, const Ephemeris& b) {
                         return compareEphemerisOrder(a, b) < 0;
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
//
// Tie-break among same-toe copies (default selection): merged nav files and
// multi-stream RTCM carry an F/NAV and an I/NAV copy of each Galileo record
// with the same toe, whose af0 differs by decimetres. getEphemeris() takes the
// first of several equal-age candidates, so that choice used to depend on
// insertion order (std::sort on toe alone is unstable, and RINEX load, merge,
// RTCM decode and copy all insert in different orders). addEphemeris() now keeps
// every satellite's list in the total order of compareEphemerisOrder(): toe,
// then (Galileo only) I/NAV before F/NAV, then data_source_code, iode, iodc,
// toc, af0, af1, af2 and the remaining fields. Among equal-toe Galileo copies
// the I/NAV one therefore always wins when it exists (the RTKLIB default
// preference), and a lone F/NAV record is still used: nothing is filtered.
// Non-Galileo ties are likewise resolved by the same fixed order, not by
// insertion order.
//
// Separately, NavigationData::setGalileoEphemerisSource(INavOnly) applies only
// the data-source filter (no toe rule). It is set by SSR paths whose clock
// corrections reference the I/NAV clock (Galileo HAS via RTCM SSR), where the
// SSR IODnav already pins the record in time.
constexpr int kGalInavClockBit = 1 << 9;

inline bool galileoSkip(const SatelliteId& sat, const Ephemeris& eph,
                        const GNSSTime& time, bool inav_only) {
    if (sat.system != GNSSSystem::Galileo) {
        return false;
    }
    if (!pppEnvOverrides().gal_inav) {
        return inav_only && !(eph.data_source_code & kGalInavClockBit);
    }
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
        if (galileoSkip(sat, eph, time, galileo_ephemeris_source_ ==
                                          GalileoEphemerisSource::INavOnly)) {
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
        if (galileoSkip(sat, eph, time, galileo_ephemeris_source_ ==
                                          GalileoEphemerisSource::INavOnly)) {
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
    if (satellite_state_cache_limit_ != 0 && satellite_state_cache_.size() >= satellite_state_cache_limit_) {
        satellite_state_cache_.clear();
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
    if (satellite_state_cache_limit_ != 0 && satellite_state_cache_.size() >= satellite_state_cache_limit_) {
        satellite_state_cache_.clear();
    }
    satellite_state_cache_.emplace(cache_key, computed);
    position = computed.position;
    velocity = computed.velocity;
    clock_bias = computed.clock_bias;
    clock_drift = computed.clock_drift;
    return computed.valid;
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

NavigationData::NavigationData() = default;

void NavigationData::setGalileoEphemerisSource(GalileoEphemerisSource source) {
    if (source == galileo_ephemeris_source_) {
        return;
    }
    galileo_ephemeris_source_ = source;
    satellite_state_cache_.clear();
    ++revision_;
}

bool NavigationData::addEphemerisIfNew(const Ephemeris& eph) {
    const auto it = ephemeris_data.find(eph.satellite);
    if (it != ephemeris_data.end()) {
        // Galileo I/NAV and F/NAV share IODnav but differ in clock reference
        // (data-source bits 9 / 8); the remaining source bits vary between
        // RINEX writers for the same broadcast record.
        constexpr int kGalClockSourceMask = (1 << 8) | (1 << 9);
        const bool galileo = eph.satellite.system == GNSSSystem::Galileo;
        for (const auto& existing : it->second) {
            if (existing.iode == eph.iode &&
                std::abs(existing.toe - eph.toe) < 1e-3 &&
                std::abs(existing.toc - eph.toc) < 1e-3 &&
                (!galileo || (existing.data_source_code & kGalClockSourceMask) ==
                                 (eph.data_source_code & kGalClockSourceMask))) {
                return false;
            }
        }
    }
    addEphemeris(eph);
    return true;
}

bool NavigationData::removeEphemeris(const Ephemeris& eph) {
    const auto it = ephemeris_data.find(eph.satellite);
    if (it == ephemeris_data.end()) {
        return false;
    }
    auto& list = it->second;
    // compareEphemerisOrder() runs through every field except the satellite (one list per
    // satellite) and glonass_ln, so equality here means "the same record in every field".
    const auto match = std::find_if(list.begin(), list.end(), [&eph](const Ephemeris& stored) {
        return stored.satellite == eph.satellite && stored.glonass_ln == eph.glonass_ln &&
               compareEphemerisOrder(stored, eph) == 0;
    });
    if (match == list.end()) {
        return false;
    }
    list.erase(match);
    if (list.empty()) {
        ephemeris_data.erase(it);
    }
    satellite_state_cache_.clear();
    ++revision_;
    return true;
}

void NavigationData::clear() {
    ephemeris_data.clear();
    satellite_state_cache_.clear();
    ++revision_;
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
