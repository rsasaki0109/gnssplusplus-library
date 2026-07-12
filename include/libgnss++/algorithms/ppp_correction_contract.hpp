#pragma once

#include <array>
#include <cstddef>

namespace libgnss::algorithms::ppp_correction_contract {

// Stable ordering contract for corrections materialized by
// PPPProcessor::applyPreciseCorrections(). Keep this list synchronized with
// that function: changing the order is a measurement-model change and must be
// reviewed against the MADOCA/CLAS parity gates.
enum class Stage {
    ReceiverGeophysics,
    SatelliteOrbitClock,
    SsrMeasurementBias,
    SsrAtmosphere,
    SatelliteAntenna,
    DcbFallback,
    IonexFallback,
    ReceiverAntenna,
    PhaseWindup,
};

inline constexpr std::array<Stage, 9> kApplicationOrder = {
    Stage::ReceiverGeophysics,
    Stage::SatelliteOrbitClock,
    Stage::SsrMeasurementBias,
    Stage::SsrAtmosphere,
    Stage::SatelliteAntenna,
    Stage::DcbFallback,
    Stage::IonexFallback,
    Stage::ReceiverAntenna,
    Stage::PhaseWindup,
};

constexpr std::size_t orderOf(Stage stage) {
    for (std::size_t index = 0; index < kApplicationOrder.size(); ++index) {
        if (kApplicationOrder[index] == stage) {
            return index;
        }
    }
    return kApplicationOrder.size();
}

constexpr const char* name(Stage stage) {
    switch (stage) {
        case Stage::ReceiverGeophysics: return "receiver_geophysics";
        case Stage::SatelliteOrbitClock: return "satellite_orbit_clock";
        case Stage::SsrMeasurementBias: return "ssr_measurement_bias";
        case Stage::SsrAtmosphere: return "ssr_atmosphere";
        case Stage::SatelliteAntenna: return "satellite_antenna";
        case Stage::DcbFallback: return "dcb_fallback";
        case Stage::IonexFallback: return "ionex_fallback";
        case Stage::ReceiverAntenna: return "receiver_antenna";
        case Stage::PhaseWindup: return "phase_windup";
    }
    return "unknown";
}

// MADOCA follows MADOCALIB ppp.c and adds SSR code/phase biases to the
// observables. Other SSR inputs retain libgnss++'s historical subtraction.
constexpr double ssrMeasurementBiasSign(bool madoca_convention) {
    return madoca_convention ? 1.0 : -1.0;
}

// Neutral atmosphere delays are removed from code and phase alike, while the
// first-order ionosphere is dispersive: remove it from code, add it to phase.
constexpr double measurementCorrectionSign(bool carrier_phase, bool dispersive) {
    return carrier_phase && dispersive ? 1.0 : -1.0;
}

}  // namespace libgnss::algorithms::ppp_correction_contract
