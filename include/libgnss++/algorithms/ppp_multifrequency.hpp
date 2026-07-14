#pragma once

#include <libgnss++/core/types.hpp>

#include <utility>
#include <vector>

namespace libgnss::algorithms::ppp_multifrequency {

// MADOCALIB rtkcmn.c obsdef_* frequency-index order. Only indices 2 and 3
// become the L3/L4 states introduced by the native est-STEC path.
inline bool isMadocalibBeiDou2(const SatelliteId& satellite) {
    return satellite.system == GNSSSystem::BeiDou && satellite.prn < 19;
}

inline std::pair<GNSSSystem, int> receiverFrequencyBiasKey(
    const SatelliteId& satellite,
    int frequency_ordinal) {
    // MADOCALIB allocates I3/I4 for every NC(opt) clock group, including
    // separate BDS-2 and BDS-3 entries. Keep the public state key compact by
    // reserving +10 for the legacy BDS-2 group.
    const int group_ordinal = isMadocalibBeiDou2(satellite)
        ? frequency_ordinal + 10
        : frequency_ordinal;
    return {satellite.system, group_ordinal};
}

inline std::vector<SignalType> signalsForFrequencyOrdinal(
                                                          const SatelliteId& satellite,
                                                          int ordinal) {
    switch (satellite.system) {
        case GNSSSystem::GPS:
            if (ordinal == 2) return {SignalType::GPS_L5};
            break;
        case GNSSSystem::Galileo:
            if (ordinal == 2) return {SignalType::GAL_E5B};
            if (ordinal == 3) return {SignalType::GAL_E6};
            break;
        case GNSSSystem::BeiDou:
            if (ordinal == 2) {
                return isMadocalibBeiDou2(satellite)
                    ? std::vector<SignalType>{SignalType::BDS_B2I}
                    : std::vector<SignalType>{SignalType::BDS_B2A};
            }
            break;
        case GNSSSystem::QZSS:
            if (ordinal == 2) return {SignalType::QZS_L2C};
            break;
        default:
            break;
    }
    return {};
}

inline double ionosphereScale(double primary_frequency_hz,
                              double frequency_hz) {
    if (!(primary_frequency_hz > 0.0) || !(frequency_hz > 0.0)) {
        return 0.0;
    }
    const double ratio = primary_frequency_hz / frequency_hz;
    return ratio * ratio;
}

inline double ambiguitySeedMeters(double carrier_phase_m,
                                  double pseudorange_m,
                                  double ionosphere_l1_m,
                                  double primary_frequency_hz,
                                  double frequency_hz) {
    return carrier_phase_m - pseudorange_m +
           2.0 * ionosphere_l1_m *
               ionosphereScale(primary_frequency_hz, frequency_hz);
}

inline double ambiguityDifferenceCycles(double first_ambiguity_m,
                                        double first_wavelength_m,
                                        double second_ambiguity_m,
                                        double second_wavelength_m) {
    if (!(first_wavelength_m > 0.0) || !(second_wavelength_m > 0.0)) {
        return 0.0;
    }
    return first_ambiguity_m / first_wavelength_m -
           second_ambiguity_m / second_wavelength_m;
}

inline double extraWideLaneDoubleDifferenceCycles(
    double reference_l2_ambiguity_m,
    double reference_l2_wavelength_m,
    double reference_extra_ambiguity_m,
    double reference_extra_wavelength_m,
    double satellite_l2_ambiguity_m,
    double satellite_l2_wavelength_m,
    double satellite_extra_ambiguity_m,
    double satellite_extra_wavelength_m) {
    return ambiguityDifferenceCycles(
               reference_l2_ambiguity_m, reference_l2_wavelength_m,
               reference_extra_ambiguity_m, reference_extra_wavelength_m) -
           ambiguityDifferenceCycles(
               satellite_l2_ambiguity_m, satellite_l2_wavelength_m,
               satellite_extra_ambiguity_m, satellite_extra_wavelength_m);
}

}  // namespace libgnss::algorithms::ppp_multifrequency
