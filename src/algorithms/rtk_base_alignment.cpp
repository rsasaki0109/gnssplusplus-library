#include <libgnss++/algorithms/rtk_base_alignment.hpp>

#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/models/troposphere.hpp>

#include <cmath>
#include <map>

namespace libgnss {
namespace rtk_base_alignment {
namespace {
constexpr double kAgeUpperToleranceS = 1e-6;

struct ModeledRangePair {
    bool valid = false;
    double base_m = 0.0;
    double target_m = 0.0;
};

double signalFrequencyHz(const SatelliteId& satellite, SignalType signal,
                         const GNSSTime& time, const NavigationData& nav) {
    // Mirrors apps/native/rtk_base_epoch_align.hpp signalFrequencyHz().
    const Ephemeris* eph = nav.getEphemeris(satellite, time);
    switch (signal) {
        case SignalType::GPS_L1CA:
        case SignalType::QZS_L1CA:
            return constants::GPS_L1_FREQ;
        case SignalType::GPS_L2C:
        case SignalType::QZS_L2C:
            return constants::GPS_L2_FREQ;
        case SignalType::GPS_L5:
        case SignalType::QZS_L5:
            return constants::GPS_L5_FREQ;
        case SignalType::GLO_L1CA:
        case SignalType::GLO_L1P:
            if (eph && eph->satellite.system == GNSSSystem::GLONASS)
                return constants::GLO_L1_BASE_FREQ +
                    eph->glonass_frequency_channel * constants::GLO_L1_STEP_FREQ;
            return constants::GLO_L1_BASE_FREQ;
        case SignalType::GLO_L2CA:
        case SignalType::GLO_L2P:
            if (eph && eph->satellite.system == GNSSSystem::GLONASS)
                return constants::GLO_L2_BASE_FREQ +
                    eph->glonass_frequency_channel * constants::GLO_L2_STEP_FREQ;
            return constants::GLO_L2_BASE_FREQ;
        case SignalType::GAL_E1:
            return constants::GAL_E1_FREQ;
        case SignalType::GAL_E5A:
            return constants::GAL_E5A_FREQ;
        case SignalType::GAL_E5B:
            return constants::GAL_E5B_FREQ;
        case SignalType::GAL_E6:
            return constants::GAL_E6_FREQ;
        case SignalType::BDS_B1I:
            return constants::BDS_B1I_FREQ;
        case SignalType::BDS_B2I:
            return constants::BDS_B2I_FREQ;
        case SignalType::BDS_B3I:
            return constants::BDS_B3I_FREQ;
        case SignalType::BDS_B1C:
            return constants::BDS_B1C_FREQ;
        case SignalType::BDS_B2A:
            return constants::BDS_B2A_FREQ;
        default:
            return 0.0;
    }
}
} // namespace

double signalWavelength(const SatelliteId& satellite, SignalType signal,
                        const GNSSTime& time, const NavigationData& nav) {
    const double frequency = signalFrequencyHz(satellite, signal, time, nav);
    return frequency <= 0.0 ? 0.0 : constants::SPEED_OF_LIGHT / frequency;
}

bool calculateModeledBaseRange(const SatelliteId& satellite, const GNSSTime& time,
                               double approx_pseudorange, const Vector3d& base_position,
                               const NavigationData& nav, double& modeled_range) {
    // Mirrors apps/native/rtk_base_epoch_align.hpp calculateModeledBaseRange().
    Vector3d sat_pos, sat_vel;
    double sat_clk = 0.0, sat_clk_drift = 0.0;
    const double travel_time = approx_pseudorange > 1.0
        ? approx_pseudorange / constants::SPEED_OF_LIGHT : 0.075;
    GNSSTime tx_time = time - travel_time;
    if (!nav.calculateSatelliteState(satellite, tx_time, sat_pos, sat_vel, sat_clk, sat_clk_drift))
        return false;
    tx_time = tx_time - sat_clk;
    if (!nav.calculateSatelliteState(satellite, tx_time, sat_pos, sat_vel, sat_clk, sat_clk_drift))
        return false;
    const auto geom = nav.calculateGeometry(base_position, sat_pos);
    if (geom.elevation <= kMinModeledElevationRad) return false;
    modeled_range = geodist(sat_pos, base_position) +
        models::tropDelaySaastamoinen(base_position, geom.elevation);
    return std::isfinite(modeled_range);
}

bool holdBaseEpoch(const ObservationData& base, const GNSSTime& target_time,
                   const Vector3d& base_position, const NavigationData& nav,
                   double max_age_s, ObservationData& held) {
    const double age = target_time - base.time;
    if (!std::isfinite(age) || !std::isfinite(max_age_s) || age < 0.0 ||
        age > max_age_s + kAgeUpperToleranceS)
        return false;

    held = ObservationData(target_time);
    held.receiver_position = base.receiver_position;
    held.receiver_clock_bias = base.receiver_clock_bias;

    // Modeled ranges are per satellite, evaluated once from the first
    // pseudorange-bearing signal (same caching rule as interpolateBaseEpoch).
    std::map<SatelliteId, ModeledRangePair> cache;
    for (const auto& observation : base.observations) {
        if (!observation.has_pseudorange) continue;
        const double wavelength = signalWavelength(
            observation.satellite, observation.signal, target_time, nav);
        if (wavelength <= 0.0) continue;

        auto found = cache.find(observation.satellite);
        if (found == cache.end()) {
            ModeledRangePair pair;
            pair.valid =
                calculateModeledBaseRange(observation.satellite, base.time, observation.pseudorange,
                                          base_position, nav, pair.base_m) &&
                calculateModeledBaseRange(observation.satellite, target_time, observation.pseudorange,
                                          base_position, nav, pair.target_m);
            found = cache.emplace(observation.satellite, pair).first;
        }
        const ModeledRangePair& modeled = found->second;
        if (!modeled.valid) continue;

        Observation out = observation;
        out.pseudorange = modeled.target_m + (observation.pseudorange - modeled.base_m);
        out.has_pseudorange = std::isfinite(out.pseudorange);
        if (observation.has_carrier_phase && (observation.lli & 0x01) == 0 &&
            !observation.loss_of_lock) {
            out.carrier_phase = (modeled.target_m +
                (observation.carrier_phase * wavelength - modeled.base_m)) / wavelength;
            out.has_carrier_phase = std::isfinite(out.carrier_phase);
        } else {
            out.carrier_phase = 0.0;
            out.has_carrier_phase = false;
        }
        if (out.has_pseudorange) held.addObservation(out);
    }
    return !held.observations.empty();
}

} // namespace rtk_base_alignment
} // namespace libgnss
