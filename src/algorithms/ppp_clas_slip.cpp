#include <libgnss++/algorithms/ppp_clas.hpp>

#include <libgnss++/algorithms/ppp_ar.hpp>
#include <libgnss++/algorithms/ppp_bias_identity.hpp>
#include <libgnss++/algorithms/ppp_env_overrides.hpp>
#include <libgnss++/algorithms/ppp_osr.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>

#include "ppp_internal.hpp"
#include "ppp_clas_diagnostics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include "ppp_clas_internal.hpp"

namespace libgnss::ppp_clas {
using namespace internal;

ClasSlipDetectionStats detectClasCycleSlips(
    const ObservationData& obs,
    const std::vector<OSRCorrection>& osr_corrections,
    const ppp_shared::PPPConfig& config,
    double dt_seconds,
    ppp_shared::PPPState& filter_state,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    std::map<SatelliteId, CLASDispersionCompensationInfo>& dispersion_compensation,
    std::map<SatelliteId, CLASPhaseBiasRepairInfo>& phase_bias_repair,
    const AmbiguityResetFunction& ambiguity_reset_function,
    double ambiguity_reset_variance,
    bool debug_enabled) {
    ClasSlipDetectionStats stats;
    if (!config.kinematic_mode || !config.enable_cycle_slip_detection) {
        return stats;
    }

    const double threshold_scale = clasSlipThresholdScale(dt_seconds);
    const double gf_threshold_m = std::max(
        config.cycle_slip_threshold, kMinimumGeometryFreeSlipThresholdMeters) *
        threshold_scale;
    const double mw_threshold_cycles = kMinimumMwSlipThresholdCycles * threshold_scale;
    const bool mrtklib_float_parity =
        config.clas_mrtklib_float_parity && config.kinematic_mode &&
        !config.low_dynamics_mode && config.use_clas_osr_filter &&
        config.use_dynamics_model;
    const bool outage_gap = dt_seconds > kClasOutageGapResetS;
    // Dynamics mode: a receiver-wide data outage (bridge/tunnel) almost
    // certainly broke carrier lock on every satellite, and the GF/MW
    // detectors cannot see it (their history is cleared by the outage
    // branch below, so the first epoch back has nothing to difference
    // against). RTKLIB handles this via the per-satellite outage counter
    // (udbias: outage > maxout resets the bias); mirror that by treating
    // the gap itself as a slip on every satellite. White-noise mode keeps
    // its historical behavior (position/clock are re-anchored to SPP each
    // epoch there, which bounds the damage of a stale ambiguity until the
    // residual gates catch it).
    const bool outage_resets_ambiguity =
        outage_gap && config.use_dynamics_model && !config.low_dynamics_mode;

    // MRTKLIB udbias_ppp() increments ssat[].outc[f] for every extant
    // ambiguity before looking at the current observations. ppp_rtk_pos()
    // clears it only for phase rows that survive the post-fit residual test.
    // With benchmark/clas.toml maxout=1, one rejected/missing epoch therefore
    // makes the next epoch start at outc=2 and resets lock to -minlock.
    if (mrtklib_float_parity) {
        for (auto& [_, ambiguity] : ambiguity_states) {
            ambiguity.outage_count = std::min(ambiguity.outage_count + 1, 1000000);
        }
    }

    for (const auto& osr : osr_corrections) {
        if (!osr.valid) {
            continue;
        }
        const Observation* l1_raw = mrtklib_float_parity
            ? findMrtklibParityRawObservation(obs, osr, 0)
            : findOsrFrequencyObservation(obs, osr, 0);
        const Observation* l2_raw = mrtklib_float_parity
            ? findMrtklibParityRawObservation(obs, osr, 1)
            : findOsrFrequencyObservation(obs, osr, 1);
        const auto raw_frequency_usable = [](const Observation* raw) {
            return raw != nullptr && raw->valid && raw->has_carrier_phase &&
                   raw->has_pseudorange;
        };
        const bool l1_raw_usable = raw_frequency_usable(l1_raw);
        const bool l2_raw_usable = raw_frequency_usable(l2_raw);
        // MRTKLIB detslp_ll() runs before the per-frequency outage reset and
        // treats either of the two low LLI bits as a slip. Preserve that LLI
        // when the other frequency is missing: the outage branch below must
        // not return before resetting the still-observed slipping frequency.
        const bool l1_lli_slip =
            mrtklib_float_parity && l1_raw_usable && (l1_raw->lli & 0x03U) != 0;
        const bool l2_lli_slip =
            mrtklib_float_parity && l2_raw_usable && (l2_raw->lli & 0x03U) != 0;
        auto& ambiguity = ambiguity_states[osr.satellite];
        bool per_sat_outage = false;
        bool l1_outage_overflow = false;
        bool l2_outage_overflow = false;
        const SatelliteId l2_ambiguity_satellite(
            osr.satellite.system,
            static_cast<uint8_t>(std::min(
                255, static_cast<int>(osr.satellite.prn) + 100)));
        // MRTKLIB udbias_ppp() increments outc[f] for every extant ambiguity
        // (mirrored at the top of this function), then clears it back to 0
        // for every satellite whose phase row survives that epoch's
        // post-fit residual test (mrtk_ppp_rtk.c ~2342: "if (!vsat[f])
        // continue; ...outc[f]=0;"). Our post-fit acceptance path
        // (updateObservedAmbiguities()) mirrors that clear, but only for
        // rows that actually reach DD/measurement construction; a satellite
        // that silently drops out of that build for one epoch (reference
        // reselection etc., not a slip or a genuine residual-test
        // rejection) never gets the chance, and because the overflow branch
        // below deliberately *preserves* -- never zeroes -- an already
        // elevated count (matching MRTKLIB's own increment-before-test
        // ordering), the counter is then parked above maxout permanently:
        // "outage_sat" re-fires every subsequent epoch even though the
        // satellite continues to be observed and corrected (G15 at
        // nagoya/run3 tow 553825-553830: fires every 0.2s epoch once
        // tripped, though raw obs + OSR corrections are valid throughout).
        // Clear the counter directly from this epoch's raw observation
        // validity -- the same underlying signal MRTKLIB's vsat[f]
        // ultimately reduces to for a satellite that was never rejected --
        // so a continuously observed satellite cannot get stuck above
        // maxout. Kill switch: GNSS_PPP_CLAS_OUTAGE_RESET_PARITY=0 restores
        // the unconditional-increment-only legacy behavior exactly.
        if (mrtklib_float_parity && !outage_gap &&
            pppEnvOverrides().clas_outage_reset_parity) {
            if (l1_raw_usable) {
                ambiguity.outage_count = 0;
            }
            if (l2_raw_usable) {
                ambiguity_states[l2_ambiguity_satellite].outage_count = 0;
            }
        }
        int l1_outage_count = ambiguity.outage_count;
        int l2_outage_count = 0;
        const auto l2_ambiguity_it = ambiguity_states.find(l2_ambiguity_satellite);
        if (l2_ambiguity_it != ambiguity_states.end()) {
            l2_outage_count = l2_ambiguity_it->second.outage_count;
        }
        if (mrtklib_float_parity && !outage_gap) {
            constexpr int kMrtklibMaxOut = 1;
            // clas_osr_zdres() does not create Galileo filter rows and uses
            // only QZSS frequency slot zero.  Their absent ambiguity states
            // therefore cannot overflow outc[] or set pbreset[] in CLASLIB.
            // Native retains the extra observations for OSR diagnostics and
            // dispersion compensation, so gate the outage lifecycle to the
            // subset that the literal filter actually tracks.
            const bool tracks_l1_filter_state =
                osr.satellite.system != GNSSSystem::Galileo;
            const bool tracks_l2_filter_state =
                tracks_l1_filter_state &&
                osr.satellite.system != GNSSSystem::QZSS;
            l1_outage_overflow =
                tracks_l1_filter_state &&
                l1_outage_count > kMrtklibMaxOut;
            l2_outage_overflow =
                tracks_l2_filter_state &&
                l2_outage_count > kMrtklibMaxOut;
            per_sat_outage = l1_outage_overflow || l2_outage_overflow;
            if (per_sat_outage) {
                ++stats.per_sat_outage_resets;
            }
        } else if (config.use_clas_osr_filter && !outage_gap &&
                   ambiguity.last_time.week > 0 && dt_seconds > 0.0) {
            const double sat_gap_s = obs.time - ambiguity.last_time;
            if (sat_gap_s >
                kClasPerSatOutageEpochs * dt_seconds + 0.5 * dt_seconds) {
                per_sat_outage = true;
                ++stats.per_sat_outage_resets;
            }
        }

        // detslp_ll() precedes udbias_ppp()'s dual-frequency checks.  A valid
        // LLI on the one frequency that remains available must therefore
        // reset that frequency even when neither outage counter has crossed
        // maxout yet (G04 at tokyo/run2 TOW 177084.4 is L1C-only with
        // LLI=1).  Keep the complete-pair case in the combined GF/LLI path
        // below, where both CLAS ambiguity states are reset together.
        const bool incomplete_pair =
            osr.num_frequencies < 2 || !l1_raw_usable || !l2_raw_usable;
        const bool incomplete_pair_lli =
            (l1_lli_slip || l2_lli_slip) && incomplete_pair;
        // udion() follows udbias_ppp() and resets the ionosphere state when
        // any currently selected frequency has exceeded maxout.  This is
        // independent of whether the complete pair continues into GF slip
        // detection below.
        const bool selected_frequency_outage =
            (l1_outage_overflow && l1_raw_usable) ||
            (l2_outage_overflow && osr.num_frequencies > 1 &&
             l2_raw_usable);
        const auto iono_it =
            filter_state.ionosphere_indices.find(osr.satellite);
        if (mrtklib_float_parity && selected_frequency_outage &&
            iono_it != filter_state.ionosphere_indices.end() &&
            iono_it->second >= 0 &&
            iono_it->second < filter_state.total_states) {
            constexpr double kMrtklibInitialIonoStateM = 1e-6;
            constexpr double kMrtklibInitialIonoVariance = 0.01 * 0.01;
            const int iono_index = iono_it->second;
            filter_state.state(iono_index) = kMrtklibInitialIonoStateM;
            filter_state.covariance.row(iono_index).setZero();
            filter_state.covariance.col(iono_index).setZero();
            filter_state.covariance(iono_index, iono_index) =
                kMrtklibInitialIonoVariance;
            filter_state.adaptive_ionosphere_process_noise[osr.satellite] =
                0.0;
        }
        if (mrtklib_float_parity &&
            (per_sat_outage || incomplete_pair_lli)) {
            // udbias_ppp() performs this frequency-specific outage reset
            // before it asks whether both L1 and L2 are currently usable.
            // Keeping the reset behind the dual-frequency slip detector
            // leaves a returning L1-only satellite (for example G11 at
            // tokyo/run2 TOW 177091.4) anchored to its stale ambiguity.
            const auto reset_frequency = [&](
                const SatelliteId& ambiguity_satellite,
                SignalType signal,
                int outage_count) {
                if (ambiguity_reset_function) {
                    ambiguity_reset_function(ambiguity_satellite, signal);
                }
                constexpr int kMrtklibMinLock = 5;
                auto& reset_ambiguity = ambiguity_states[ambiguity_satellite];
                reset_ambiguity.lock_count = -kMrtklibMinLock;
                reset_ambiguity.outage_count = outage_count;
            };
            // With a complete L1/L2 pair, MRTKLIB's geometry-free detector
            // runs immediately after detslp_ll().  On a returning frequency
            // the LLI epoch therefore marks both frequency slips (G09 at
            // tokyo/run2 TOW 177090.8 publishes slip=1/lock=-4 on L1 and L2).
            // Keep a genuinely incomplete pair frequency-specific.
            const bool complete_pair_lli =
                !incomplete_pair && (l1_lli_slip || l2_lli_slip);
            const bool reset_l1 =
                l1_outage_overflow || l1_lli_slip || complete_pair_lli;
            const bool reset_l2 =
                l2_outage_overflow || l2_lli_slip || complete_pair_lli;
            if (reset_l1) {
                reset_frequency(osr.satellite,
                                l1_lli_slip ? l1_raw->signal
                                            : SignalType::SIGNAL_TYPE_COUNT,
                                l1_outage_count);
            }
            if (reset_l2) {
                reset_frequency(l2_ambiguity_satellite,
                                l2_lli_slip ? l2_raw->signal
                                            : SignalType::SIGNAL_TYPE_COUNT,
                                l2_outage_count);
            }
            if (l1_lli_slip || l2_lli_slip) {
                ++stats.lli_count;
                dispersion_compensation[osr.satellite].slip = {true, true};
                auto repair_it = phase_bias_repair.find(osr.satellite);
                if (repair_it != phase_bias_repair.end()) {
                    resetClasPhaseBiasRepair(repair_it->second);
                }
            }
            ++stats.total_resets;
            stats.reset_satellites.insert(osr.satellite);
            if (debug_enabled) {
                std::cerr << "[CLAS-SLIP] " << osr.satellite.toString()
                          << " tow=" << obs.time.tow
                          << " reason="
                          << (per_sat_outage ? "outage_sat" : "")
                          << (per_sat_outage &&
                                      (l1_lli_slip || l2_lli_slip)
                                  ? "+"
                                  : "")
                          << ((l1_lli_slip || l2_lli_slip) ? "lli" : "")
                          << " freq="
                          << (reset_l1 && reset_l2
                                  ? "L1+L2"
                                  : (reset_l1 ? "L1" : "L2"))
                          << " dt=" << dt_seconds << "\n";
            }
            continue;
        }

        if (incomplete_pair) {
            continue;
        }

        const double f1 = osr.frequencies[0];
        const double f2 = osr.frequencies[1];
        if (f1 <= 0.0 || f2 <= 0.0 || std::abs(f1 - f2) < 1e6) {
            continue;
        }

        // MRTKLIB detslp_gf() / gfmeas_L1L2() operates on the raw carrier
        // phases.  Applying the broadcast phase biases here turns a CSSR
        // phase-bias update into an apparent geometry-free jump and resets
        // otherwise continuous ambiguities (G04 at tow 177067.4 in
        // tokyo/run2).  Biases belong to the corrected measurement model,
        // not to receiver cycle-slip detection.
        const double l1_m = l1_raw->carrier_phase * osr.wavelengths[0] -
            (mrtklib_float_parity ? 0.0 : osr.phase_bias_m[0]);
        const double l2_m = l2_raw->carrier_phase * osr.wavelengths[1] -
            (mrtklib_float_parity ? 0.0 : osr.phase_bias_m[1]);
        const double gf_m = l1_m - l2_m;
        // RTKLIB's GPS frequency-2 observation slot follows its RINEX code
        // priority (the tokyo receiver supplies L2W when usable).  If L2W is
        // absent, gfmeas_L1L2() sees obs->L[1] == 0 and leaves the saved GF
        // value untouched; it does not fall through to the simultaneously
        // recorded L2L signal.  Native's generic secondary-signal selector
        // does make that L2L fallback, so gate only the parity slip detector
        // to the literal RTKLIB slot identity.
        const bool mrtklib_gf_pair_usable = true;
        const double p1 = l1_raw->pseudorange - osr.code_bias_m[0];
        const double p2 = l2_raw->pseudorange - osr.code_bias_m[1];
        const double mw_m = (f1 * l1_m - f2 * l2_m) / (f1 - f2) -
                              (f1 * p1 + f2 * p2) / (f1 + f2);
        const double lambda_wl = constants::SPEED_OF_LIGHT / std::abs(f1 - f2);
        const double mw_cycles = mw_m / lambda_wl;

        bool lli_slip = l1_raw->loss_of_lock || l2_raw->loss_of_lock;
        bool gf_slip = false;
        bool mw_slip = false;
        bool code_change_slip = false;

        // MRTKLIB detslp_code(): changing the tracked observation code on a
        // frequency invalidates that frequency's phase-bias state.  CLAS uses
        // two frequencies and resets both together, matching the existing
        // real/pseudo-satellite ambiguity layout.  The first observed code
        // only seeds history and is not a slip.
        if (mrtklib_float_parity) {
            const std::array<SignalType, 2> current_signals{
                l1_raw->signal, l2_raw->signal};
            const std::array<std::string, 2> current_carrier_types{
                l1_raw->carrier_phase_observation_type,
                l2_raw->carrier_phase_observation_type};
            for (int frequency = 0; frequency < 2; ++frequency) {
                if (ambiguity.has_last_observation_signal[frequency] &&
                    (ambiguity.last_observation_signals[frequency] !=
                         current_signals[frequency] ||
                     ambiguity.last_carrier_observation_types[frequency] !=
                         current_carrier_types[frequency])) {
                    code_change_slip = true;
                }
                ambiguity.last_observation_signals[frequency] =
                    current_signals[frequency];
                ambiguity.last_carrier_observation_types[frequency] =
                    current_carrier_types[frequency];
                ambiguity.has_last_observation_signal[frequency] = true;
            }
        }

        // MRTKLIB per-satellite outage reset (mrtk_ppp_rtk.c:865-875,
        // clas.toml out_count = 1): a satellite whose observations were
        // missing (or whose measurement update failed) for more than maxout
        // consecutive epochs gets its ambiguity fully reset and lock
        // restarted. Without this a satellite blocked for a few seconds
        // returns with its stale ambiguity still AR-eligible -- the GF/MW
        // detectors compare against seconds-old history and can miss the
        // integer break, producing self-consistent wrong fixes. The global
        // outage_gap branch below only covers receiver-wide gaps.
        if (outage_gap) {
            ++stats.outage_resets;
            ambiguity.has_last_geometry_free = false;
            ambiguity.has_last_melbourne_wubbena = false;
            clearClasWlnlMwState(ambiguity);
        } else {
            if (mrtklib_gf_pair_usable &&
                ambiguity.has_last_geometry_free &&
                std::isfinite(gf_m) &&
                std::abs(gf_m - ambiguity.last_geometry_free_m) > gf_threshold_m) {
                gf_slip = true;
            }
            // MW-mean slip is a native-only detector. MRTKLIB detslp_gf uses
            // LLI + geometry-free phase here and does not reset ambiguities
            // from a Melbourne-Wubbena running-mean excursion.
            const bool wl_mean_stable =
                ambiguity.mw_count >= config.wl_min_averaging_epochs &&
                std::isfinite(ambiguity.mw_mean_cycles) &&
                std::abs(ambiguity.mw_mean_cycles -
                         std::round(ambiguity.mw_mean_cycles)) < 0.25;
            if (!mrtklib_float_parity &&
                !ambiguity.wl_is_fixed && !wl_mean_stable) {
                if (ambiguity.mw_count >= 3 &&
                    std::isfinite(mw_cycles) &&
                    std::abs(mw_cycles - ambiguity.mw_mean_cycles) >
                        mw_threshold_cycles) {
                    mw_slip = true;
                } else if (ambiguity.mw_count < 3 &&
                           ambiguity.has_last_melbourne_wubbena &&
                           std::isfinite(mw_cycles) &&
                           std::abs(mw_m - ambiguity.last_melbourne_wubbena_m) >
                               kMwSlipFallbackThresholdMeters * threshold_scale) {
                    mw_slip = true;
                }
            }
        }

        if (mrtklib_gf_pair_usable && std::isfinite(gf_m)) {
            ambiguity.last_geometry_free_m = gf_m;
            ambiguity.has_last_geometry_free = true;
        }
        if (std::isfinite(mw_m)) {
            ambiguity.last_melbourne_wubbena_m = mw_m;
            ambiguity.has_last_melbourne_wubbena = true;
        }

        const bool combined_phase_reset =
            lli_slip || gf_slip || mw_slip || code_change_slip ||
            outage_resets_ambiguity;
        if (!lli_slip && !gf_slip && !mw_slip && !code_change_slip &&
            !outage_resets_ambiguity && !per_sat_outage) {
            continue;
        }

        if (lli_slip) {
            ++stats.lli_count;
        }
        if (gf_slip) {
            ++stats.gf_count;
        }
        if (mw_slip) {
            ++stats.mw_count;
        }
        if (code_change_slip) {
            ++stats.code_change_count;
        }
        ++stats.total_resets;
        stats.reset_satellites.insert(osr.satellite);

        ambiguity.needs_reinitialization = true;
        ambiguity.has_last_slip_time = true;
        ambiguity.last_slip_time = obs.time;
        clearClasWlnlMwState(ambiguity);
        ambiguity.has_last_geometry_free = false;
        ambiguity.has_last_melbourne_wubbena = false;

        // MRTKLIB compensatedisp() latches ssat.slip, which is set only by
        // the receiver cycle-slip detectors.  An outage resets the ambiguity
        // and lock in udbias_ppp(), but it does not set comp_slip.  Keeping
        // those lifecycles separate is essential: treating an ordinary
        // post-fit rejection/outage as a carrier slip suppresses valid
        // measurement-based dispersion compensation until the next STEC
        // bank.
        if (lli_slip || gf_slip || mw_slip || code_change_slip) {
            dispersion_compensation[osr.satellite].slip = {true, true};
        }
        auto repair_it = phase_bias_repair.find(osr.satellite);
        if (repair_it != phase_bias_repair.end()) {
            resetClasPhaseBiasRepair(repair_it->second);
        }

        if (ambiguity_reset_function) {
            const uint8_t l2_prn = static_cast<uint8_t>(
                std::min(255, static_cast<int>(osr.satellite.prn) + 100));
            const SatelliteId l2_satellite(osr.satellite.system, l2_prn);
            ambiguity_reset_function(osr.satellite, l1_raw->signal);
            ambiguity_reset_function(l2_satellite, l2_raw->signal);
            if (mrtklib_float_parity) {
                // udbias_ppp(): a slip/outage reset starts at -minlock;
                // subsequent accepted samples increment it and ddmat admits
                // the ambiguity only after lock becomes positive.
                constexpr int kMrtklibMinLock = 5;
                ambiguity_states[osr.satellite].lock_count = -kMrtklibMinLock;
                ambiguity_states[l2_satellite].lock_count = -kMrtklibMinLock;
                // resetAmbiguity() value-initializes the bookkeeping, but
                // MRTKLIB retains each frequency's current counter until a
                // valid post-fit phase row clears it.
                ambiguity_states[osr.satellite].outage_count = l1_outage_count;
                ambiguity_states[l2_satellite].outage_count = l2_outage_count;
            }
        }

        if (debug_enabled) {
            std::string reason;
            if (lli_slip) {
                reason += "lli";
            }
            if (gf_slip) {
                if (!reason.empty()) {
                    reason += "+";
                }
                reason += "gf";
            }
            if (mw_slip) {
                if (!reason.empty()) {
                    reason += "+";
                }
                reason += "mw";
            }
            if (code_change_slip) {
                if (!reason.empty()) {
                    reason += "+";
                }
                reason += "code";
            }
            if (outage_resets_ambiguity) {
                if (!reason.empty()) {
                    reason += "+";
                }
                reason += "outage";
            }
            if (per_sat_outage) {
                if (!reason.empty()) {
                    reason += "+";
                }
                reason += "outage_sat";
            }
            std::cerr << "[CLAS-SLIP] " << osr.satellite.toString()
                      << " tow=" << obs.time.tow
                      << " reason=" << reason
                      << " dt=" << dt_seconds
                      << " gf_m=" << gf_m
                      << " mw_cyc=" << mw_cycles
                      << "\n";
        }
    }

    if (stats.total_resets > 0 && !mrtklib_float_parity) {
        syncSlipState(
            obs,
            filter_state,
            ambiguity_states,
            dispersion_compensation,
            phase_bias_repair,
            ambiguity_reset_variance,
            true);
    }

    return stats;
}

void syncSlipState(
    const ObservationData& obs,
    ppp_shared::PPPState& filter_state,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    std::map<SatelliteId, CLASDispersionCompensationInfo>& dispersion_compensation,
    std::map<SatelliteId, CLASPhaseBiasRepairInfo>& phase_bias_repair,
    double ambiguity_reset_variance,
    bool mark_dispersion_slip) {
    for (const auto& satellite : obs.getSatellites()) {
        const auto slip_it = ambiguity_states.find(satellite);
        if (slip_it == ambiguity_states.end() || !slip_it->second.needs_reinitialization) {
            continue;
        }

        const SatelliteId l2_satellite(
            satellite.system,
            static_cast<uint8_t>(std::min(255, satellite.prn + 100)));
        auto reset_ambiguity = [&](const SatelliteId& ambiguity_satellite) {
            auto& ambiguity = ambiguity_states[ambiguity_satellite];
            // detectClasCycleSlips() has already applied MRTKLIB's
            // lock=-minlock and retained its overflowed outc. This secondary
            // state/covariance synchronization must not turn a just-reset
            // ambiguity back into an immediately AR-eligible lock=0 state.
            const int reset_lock_count = ambiguity.lock_count;
            const int reset_outage_count = ambiguity.outage_count;
            ambiguity = ppp_shared::PPPAmbiguityInfo{};
            ambiguity.needs_reinitialization = true;
            if (reset_lock_count < 0) {
                ambiguity.lock_count = reset_lock_count;
            }
            ambiguity.outage_count = reset_outage_count;

            const auto ambiguity_index_it =
                filter_state.ambiguity_indices.find(ambiguity_satellite);
            if (ambiguity_index_it == filter_state.ambiguity_indices.end()) {
                return;
            }
            const int ambiguity_index = ambiguity_index_it->second;
            if (ambiguity_index < 0 || ambiguity_index >= filter_state.total_states) {
                return;
            }
            filter_state.state(ambiguity_index) = 0.0;
            filter_state.covariance.row(ambiguity_index).setZero();
            filter_state.covariance.col(ambiguity_index).setZero();
            filter_state.covariance(ambiguity_index, ambiguity_index) =
                ambiguity_reset_variance;
        };

        if (filter_state.ambiguity_indices.find(l2_satellite) !=
            filter_state.ambiguity_indices.end()) {
            reset_ambiguity(l2_satellite);
        }

        if (mark_dispersion_slip) {
            dispersion_compensation[satellite].slip = {true, true};
        }
        auto repair_it = phase_bias_repair.find(satellite);
        if (repair_it != phase_bias_repair.end()) {
            repair_it->second.reference_time = GNSSTime();
            repair_it->second.last_continuity_m = {0.0, 0.0, 0.0};
            repair_it->second.offset_cycles = {0.0, 0.0, 0.0};
            repair_it->second.pending_state_shift_cycles = {0.0, 0.0, 0.0};
            repair_it->second.has_last = {false, false, false};
        }
    }
}


void markSlipCompensationFromAmbiguities(
    const ObservationData& obs,
    const std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    std::map<SatelliteId, CLASDispersionCompensationInfo>& dispersion_compensation) {
    for (const auto& satellite : obs.getSatellites()) {
        auto& compensation = dispersion_compensation[satellite];
        const auto l1_ambiguity_it = ambiguity_states.find(satellite);
        if (l1_ambiguity_it != ambiguity_states.end() &&
            l1_ambiguity_it->second.needs_reinitialization) {
            compensation.slip[0] = true;
        }
        const SatelliteId l2_satellite(
            satellite.system,
            static_cast<uint8_t>(std::min(255, satellite.prn + 100)));
        const auto l2_ambiguity_it = ambiguity_states.find(l2_satellite);
        if (l2_ambiguity_it != ambiguity_states.end() &&
            l2_ambiguity_it->second.needs_reinitialization) {
            compensation.slip[1] = true;
        }
    }
}

}  // namespace libgnss::ppp_clas
