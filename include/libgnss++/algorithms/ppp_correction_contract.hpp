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

// MADOCALIB initializes per-frequency ambiguity states with corrected carrier
// phase/code observations but computes the ionosphere seed from the original
// raw P1/P2 pair. Preserve that seed for coherent MADOCA SSR processing.
constexpr bool rederiveIonosphereSeedAfterSsrBias(bool madoca_convention) {
    return !madoca_convention;
}

// MADOCALIB udbias_ppp() deliberately calls corr_meas() with phw=0 while the
// ordinary measurement residual uses the accumulated phase windup. Match that
// initialization convention only for coherent MADOCA processing.
constexpr bool excludePhaseWindupFromAmbiguitySeed(bool madoca_convention) {
    return madoca_convention;
}

// MADOCALIB satpos_ssr() replaces the broadcast eccentric-anomaly
// approximation with RTKLIB's state-vector form after ephpos() has formed a
// 1 ms forward-difference velocity. Harmonic orbit terms make the two forms
// differ by centimetres for real ephemerides.
constexpr double madocalibSsrBroadcastClock(double polynomial_clock_s,
                                            double position_velocity_dot_m2_s,
                                            double speed_of_light_m_s) {
    return polynomial_clock_s -
        2.0 * position_velocity_dot_m2_s /
            (speed_of_light_m_s * speed_of_light_m_s);
}

// The first coherent-MADOCA N1 solution is published only after two
// consecutive successful ambiguity attempts. Once an N1 epoch has been
// committed, normal single-epoch reacquisition applies.
constexpr bool deferFirstMadocaN1Fix(bool madoca_convention,
                                     std::size_t committed_n1_epochs,
                                     bool confirmation_pending) {
    return madoca_convention &&
        committed_n1_epochs == 0 &&
        !confirmation_pending;
}

// MADOCALIB's PPP receiver geometry uses its legacy Love-number solid-earth
// tide model. Keep the IERS model selectable for every other PPP profile.
constexpr bool useIersSolidEarthTide(bool madoca_convention,
                                     bool configured_iers_model) {
    return configured_iers_model && !madoca_convention;
}

// Neutral atmosphere delays are removed from code and phase alike, while the
// first-order ionosphere is dispersive: remove it from code, add it to phase.
constexpr double measurementCorrectionSign(bool carrier_phase, bool dispersive) {
    return carrier_phase && dispersive ? 1.0 : -1.0;
}

}  // namespace libgnss::algorithms::ppp_correction_contract
