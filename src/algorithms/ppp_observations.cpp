#include "ppp_internal.hpp"

#include <libgnss++/algorithms/ppp_utils.hpp>
#include <libgnss++/core/signal_policy.hpp>
#include <libgnss++/core/signals.hpp>

#include <cmath>
#include <vector>

namespace libgnss {

using namespace ppp_internal;

namespace {

std::vector<SignalType> secondarySignalsForObservation(const SatelliteId& sat,
                                                       bool prefer_qzss_l5) {
    if (prefer_qzss_l5 && sat.system == GNSSSystem::QZSS) {
        return {SignalType::QZS_L5, SignalType::QZS_L2C};
    }
    return secondarySignals(sat.system);
}

}  // namespace

std::vector<PPPProcessor::IonosphereFreeObs> PPPProcessor::formIonosphereFree(
    const ObservationData& obs,
    const NavigationData& nav) {
    std::vector<IonosphereFreeObs> combined;
    combined.reserve(obs.getSatellites().size());
    const bool capture_shadow_metadata =
        !env_overrides_.madoca_postfit_shadow_path.empty();

    for (const auto& sat : obs.getSatellites()) {
        // Keep the native MADOCA parity path on systems whose L6 SSR handling is
        // coherent with the MADOCALIB bridge for this loader.
        if (require_coherent_ssr_ && sat.system == GNSSSystem::BeiDou) {
            continue;
        }
        if (require_coherent_ssr_ && sat.system == GNSSSystem::GLONASS &&
            !env_overrides_.madoca_glonass) {
            continue;
        }

        const Observation* primary = findObservationForSignals(obs, sat, primarySignals(sat.system));
        if (primary == nullptr) {
            continue;
        }

        // BeiDou GEO satellites (C01-C05, C59-C63) need the special geostationary
        // orbit propagation the broadcast model does not yet implement; their
        // computed ranges are off by hundreds of metres to tens of kilometres
        // (e.g. C59 ~270 km on the MIZU sample), which destabilises the filter.
        // SPP already gates them (spp.cpp validateObservations); mirror that here.
        if (signal_policy::isBeiDouGeoSatellite(sat)) {
            continue;
        }

        const Ephemeris* eph = nav.getEphemeris(sat, obs.time);

        // Drop satellites broadcasting an unhealthy status, mirroring RTKLIB
        // satexclude(): QZSS masks its LEX (L6) signal-health bit (bit 0); any
        // other set health bit, or any nonzero health on the remaining systems,
        // excludes the satellite from the PPP measurement update. The MIZU BRDC
        // nav carries unhealthy R11/R26/R27, E14/E18 and several BDS GEO/IGSO
        // whose stale broadcast orbits otherwise drive the filter kilometers off.
        if (eph != nullptr) {
            int sv_health = static_cast<int>(eph->health);
            if (sat.system == GNSSSystem::QZSS) {
                sv_health &= 0xFE;
            }
            if (sv_health != 0) {
                continue;
            }
        }

        IonosphereFreeObs entry;
        entry.satellite = sat;
        if (capture_shadow_metadata && eph != nullptr &&
            sat.system == GNSSSystem::GLONASS) {
            entry.glonass_frequency_channel = eph->glonass_frequency_channel;
        }

        if (!ppp_config_.use_ionosphere_free) {
            const double primary_frequency_hz = signalFrequencyHz(primary->signal, eph);
            entry.pseudorange_if = primary->pseudorange;
            entry.primary_signal = primary->signal;
            entry.primary_observation_type = primary->exactBiasObservationType();
            entry.primary_code_bias_coeff = 1.0;
            entry.secondary_code_bias_coeff = 0.0;
            if (capture_shadow_metadata) {
                entry.primary_frequency_hz = primary_frequency_hz;
            }
            // Per-frequency L1 raw observable (biases applied later in
            // applyPreciseCorrections).
            entry.pseudorange_l1 = primary->pseudorange;
            entry.freq_l1 = primary_frequency_hz;
            entry.variance_pr_l1 = safeVariance(
                ppp_config_.pseudorange_sigma * ppp_config_.pseudorange_sigma, 1e-6);
            entry.variance_cp_l1 = safeVariance(
                ppp_config_.carrier_phase_sigma * ppp_config_.carrier_phase_sigma, 1e-8);
            if (primary->has_carrier_phase) {
                const double wavelength = signalWavelengthMeters(primary->signal, eph);
                if (wavelength > 0.0) {
                    entry.carrier_phase_if = primary->carrier_phase * wavelength;
                    entry.ambiguity_scale_m = wavelength;
                    entry.has_carrier_phase = true;
                    entry.carrier_phase_l1 = primary->carrier_phase * wavelength;
                    entry.wavelength_l1 = wavelength;
                }
            }
            entry.variance_pr = safeVariance(
                ppp_config_.pseudorange_sigma * ppp_config_.pseudorange_sigma, 1e-6);
            entry.variance_cp = safeVariance(
                ppp_config_.carrier_phase_sigma * ppp_config_.carrier_phase_sigma, 1e-8);
            entry.valid = true;
            // Fall through to SSR correction application below.
            // But skip IFLC formation — keep L1-only entry.
        }

        const bool prefer_qzss_l5 =
            require_coherent_ssr_ && env_overrides_.madoca_qzss_l5;
        const Observation* secondary = findObservationForSignals(
            obs, sat, secondarySignalsForObservation(sat, prefer_qzss_l5));

        // Per-frequency mode: carry BOTH L1 and L2 raw observables.
        // SSR corrections (orbit/clock/bias/iono) still applied below.
        if (!ppp_config_.use_ionosphere_free) {
            if (secondary != nullptr) {
                entry.secondary_signal = secondary->signal;
                entry.secondary_observation_type = secondary->exactBiasObservationType();
                const double f1 = signalFrequencyHz(primary->signal, eph);
                const double f2 = signalFrequencyHz(secondary->signal, eph);
                if (capture_shadow_metadata) {
                    entry.primary_frequency_hz = f1;
                    entry.secondary_frequency_hz = f2;
                }
                if (f1 > 0.0 && f2 > 0.0) {
                    entry.primary_code_bias_coeff = 1.0;
                    entry.secondary_code_bias_coeff = 0.0;
                    entry.freq_l2 = f2;
                    entry.has_l2 = true;
                    if (secondary->has_pseudorange &&
                        std::isfinite(secondary->pseudorange)) {
                        entry.pseudorange_l2 = secondary->pseudorange;
                        entry.variance_pr_l2 = safeVariance(
                            ppp_config_.pseudorange_sigma *
                                ppp_config_.pseudorange_sigma,
                            1e-6);
                        // Geometry-free ionosphere seed in L1-equivalent meters:
                        // (P1 - P2) / (1 - (f1/f2)^2). Positive STEC -> positive.
                        if (entry.pseudorange_l1 != 0.0) {
                            const double ratio = (f1 / f2);
                            const double denom = 1.0 - ratio * ratio;
                            if (std::abs(denom) > 1e-6) {
                                entry.iono_init_m =
                                    (entry.pseudorange_l1 - entry.pseudorange_l2) /
                                    denom;
                                entry.has_iono_init = true;
                            }
                        }
                    }
                    if (secondary->has_carrier_phase) {
                        const double lambda2 =
                            signalWavelengthMeters(secondary->signal, eph);
                        if (lambda2 > 0.0) {
                            entry.carrier_phase_l2 =
                                secondary->carrier_phase * lambda2;
                            entry.wavelength_l2 = lambda2;
                            entry.has_carrier_phase_l2 = true;
                            entry.variance_cp_l2 = safeVariance(
                                ppp_config_.carrier_phase_sigma *
                                    ppp_config_.carrier_phase_sigma,
                                1e-8);
                        }
                    }
                }
            }
            entry.valid = true;
            combined.push_back(entry);
            continue;
        }

        if (secondary == nullptr) {
            entry.pseudorange_if = primary->pseudorange;
            entry.primary_signal = primary->signal;
            entry.primary_observation_type = primary->exactBiasObservationType();
            entry.primary_code_bias_coeff = 1.0;
            entry.secondary_code_bias_coeff = 0.0;
            if (capture_shadow_metadata) {
                entry.primary_frequency_hz = signalFrequencyHz(primary->signal, eph);
            }
            if (primary->has_carrier_phase) {
                const double wavelength = signalWavelengthMeters(primary->signal, eph);
                if (wavelength > 0.0) {
                    entry.carrier_phase_if = primary->carrier_phase * wavelength;
                    entry.ambiguity_scale_m = wavelength;
                    entry.has_carrier_phase = true;
                }
            }
            entry.variance_pr = safeVariance(
                ppp_config_.pseudorange_sigma * ppp_config_.pseudorange_sigma, 1e-6);
            entry.variance_cp = safeVariance(
                ppp_config_.carrier_phase_sigma * ppp_config_.carrier_phase_sigma, 1e-8);
            entry.valid = true;
            combined.push_back(entry);
            continue;
        }

        const double f1 = signalFrequencyHz(primary->signal, eph);
        const double f2 = signalFrequencyHz(secondary->signal, eph);
        if (f1 <= 0.0 || f2 <= 0.0 || std::abs(f1 - f2) < 1.0) {
            continue;
        }

        const auto coefficients = ppp_utils::getIonosphereFreeCoefficients(f1, f2);
        entry.pseudorange_if =
            coefficients.first * primary->pseudorange + coefficients.second * secondary->pseudorange;
        entry.primary_signal = primary->signal;
        entry.secondary_signal = secondary->signal;
        entry.primary_observation_type = primary->exactBiasObservationType();
        entry.secondary_observation_type = secondary->exactBiasObservationType();
        entry.primary_code_bias_coeff = coefficients.first;
        entry.secondary_code_bias_coeff = coefficients.second;
        if (capture_shadow_metadata) {
            entry.primary_frequency_hz = f1;
            entry.secondary_frequency_hz = f2;
        }
        entry.pseudorange_code_bias_m = 0.0;
        entry.variance_pr = safeVariance(
            (coefficients.first * coefficients.first +
             coefficients.second * coefficients.second) *
                ppp_config_.pseudorange_sigma * ppp_config_.pseudorange_sigma,
            1e-6);

        if (primary->has_carrier_phase && secondary->has_carrier_phase) {
            const double lambda1 = signalWavelengthMeters(primary->signal, eph);
            const double lambda2 = signalWavelengthMeters(secondary->signal, eph);
            if (lambda1 > 0.0 && lambda2 > 0.0) {
                const double l1_m = primary->carrier_phase * lambda1;
                const double l2_m = secondary->carrier_phase * lambda2;
                entry.carrier_phase_if = coefficients.first * l1_m + coefficients.second * l2_m;
                entry.ambiguity_scale_m = lambda1;
                entry.variance_cp = safeVariance(
                    (coefficients.first * coefficients.first +
                     coefficients.second * coefficients.second) *
                        ppp_config_.carrier_phase_sigma * ppp_config_.carrier_phase_sigma,
                    1e-8);
                entry.has_carrier_phase = true;
            }
        }

        entry.valid = std::isfinite(entry.pseudorange_if);
        combined.push_back(entry);
    }

    return combined;
}

}  // namespace libgnss
