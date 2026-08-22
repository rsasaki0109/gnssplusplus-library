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

bool mrtklibPostfitRowsValid(
    const ppp_shared::PPPState& filter_state,
    const std::vector<MeasurementRow>& measurements,
    const std::vector<MeasurementRow>& prefit_linearization_measurements,
    std::set<SatelliteId>* accepted_l1_phase_satellites = nullptr,
    std::set<SatelliteId>* accepted_phase_ambiguities = nullptr,
    bool debug_enabled = false) {
    if (measurements.empty()) {
        return false;
    }
    const int nobs = static_cast<int>(measurements.size());
    MatrixXd H = MatrixXd::Zero(nobs, filter_state.total_states);
    VectorXd residuals = VectorXd::Zero(nobs);
    MatrixXd R = MatrixXd::Zero(nobs, nobs);
    const bool reuse_prefit_linearization =
        prefit_linearization_measurements.size() == measurements.size();
    for (int i = 0; i < nobs; ++i) {
        const auto& row = measurements[static_cast<size_t>(i)];
        // ppp_rtk_pos() recomputes postfit DD residuals with H=NULL, then
        // residual_test(stage=1) deliberately reuses the H produced by the
        // prefit ddres() call. Only v and R are postfit quantities.
        H.row(i) = reuse_prefit_linearization
            ? prefit_linearization_measurements[static_cast<size_t>(i)].H
            : row.H;
        residuals(i) = row.residual;
        R(i, i) = row.variance;
    }
    for (int i = 0; i < nobs; ++i) {
        const auto& row_i = measurements[static_cast<size_t>(i)];
        if (row_i.dd_covariance_block < 0) {
            continue;
        }
        for (int j = i + 1; j < nobs; ++j) {
            const auto& row_j = measurements[static_cast<size_t>(j)];
            if (row_j.dd_covariance_block == row_i.dd_covariance_block) {
                R(i, j) = row_i.reference_variance;
                R(j, i) = row_i.reference_variance;
            }
        }
    }
    const MatrixXd Q = H * filter_state.covariance * H.transpose() + R;
    int total_phase_rows = 0;
    int accepted_phase_rows = 0;
    double normalized_sum = 0.0;
    std::set<int> pair_rejected_rows;
    std::map<SatelliteId, std::array<int, 2>> phase_pair_rows;
    for (int i = 0; i < nobs; ++i) {
        const auto& row = measurements[static_cast<size_t>(i)];
        if (!row.is_phase || row.freq_index < 0 || row.freq_index > 1) {
            continue;
        }
        auto result = phase_pair_rows.try_emplace(
            row.satellite, std::array<int, 2>{-1, -1});
        result.first->second[static_cast<size_t>(row.freq_index)] = i;
    }
    constexpr double kF1F2 = 1575.42e6 / 1227.60e6;
    constexpr double kGamma = kF1F2 * kF1F2;
    for (const auto& [satellite, rows] : phase_pair_rows) {
        (void)satellite;
        if (rows[0] < 0 || rows[1] < 0) {
            continue;
        }
        const double v0 = residuals(rows[0]);
        const double v1 = residuals(rows[1]);
        const double dispersive = kF1F2 * (v0 - v1) / (1.0 - kGamma);
        const double nondispersive =
            (kGamma * v0 - v1) / (kGamma - 1.0);
        const double variance = std::max(Q(rows[0], rows[0]), Q(rows[1], rows[1]));
        if (variance > 0.0 &&
            (dispersive * dispersive > 9.0 * variance ||
             nondispersive * nondispersive > 9.0 * variance)) {
            if (debug_enabled) {
                std::cerr << "[CLAS-POSTFIT-PAIR] sat="
                          << satellite.toString() << " v0=" << v0
                          << " v1=" << v1 << " Smax=" << variance
                          << " dispersive=" << dispersive
                          << " nondispersive=" << nondispersive
                          << " rejected=1\n";
            }
            pair_rejected_rows.insert(rows[0]);
            pair_rejected_rows.insert(rows[1]);
        }
    }
    const auto ambiguity_satellite = [](const SatelliteId& satellite,
                                        int freq_index) {
        return SatelliteId(
            satellite.system,
            static_cast<uint8_t>(std::min(
                255, static_cast<int>(satellite.prn) +
                         (freq_index == 0 ? 0 : 100))));
    };
    // ddres() marks both ends of every phase DD as valid before residual_test().
    // residual_test() then clears only sat2 (the non-reference satellite) when
    // either the paired-frequency or individual innovation gate rejects a row.
    // Preserve that asymmetric vsat lifecycle: the reference satellite must not
    // acquire an outage merely because one of its DD targets was rejected.
    for (const auto& row : measurements) {
        if (!row.is_phase || row.freq_index < 0) {
            continue;
        }
        if (accepted_l1_phase_satellites != nullptr && row.freq_index == 0) {
            accepted_l1_phase_satellites->insert(row.reference_satellite);
            accepted_l1_phase_satellites->insert(row.satellite);
        }
        if (accepted_phase_ambiguities != nullptr) {
            accepted_phase_ambiguities->insert(
                ambiguity_satellite(row.reference_satellite, row.freq_index));
            accepted_phase_ambiguities->insert(
                ambiguity_satellite(row.satellite, row.freq_index));
        }
    }
    for (int i = 0; i < nobs; ++i) {
        const auto& row = measurements[static_cast<size_t>(i)];
        if (!row.is_phase || row.freq_index < 0) {
            continue;
        }
        ++total_phase_rows;
        double threshold = 2.0;
        if (row.ambiguity_fresh) {
            threshold *= 10.0;
        }
        if (pair_rejected_rows.count(i) == 0 && Q(i, i) > 0.0 &&
            residuals(i) * residuals(i) <
                threshold * threshold * Q(i, i)) {
            ++accepted_phase_rows;
            normalized_sum += residuals(i) * residuals(i) / Q(i, i);
        } else {
            if (accepted_l1_phase_satellites != nullptr &&
                row.freq_index == 0) {
                accepted_l1_phase_satellites->erase(row.satellite);
            }
            if (accepted_phase_ambiguities != nullptr) {
                accepted_phase_ambiguities->erase(
                    ambiguity_satellite(row.satellite, row.freq_index));
            }
            if (debug_enabled && row.freq_index == 0) {
                std::cerr << "[CLAS-POSTFIT-GATE] sat="
                          << row.satellite.toString() << " v=" << residuals(i)
                          << " S=" << Q(i, i) << " threshold=" << threshold
                          << " pair_rejected="
                          << (pair_rejected_rows.count(i) != 0) << "\n";
            }
        }
    }
    // MRTKLIB residual_test(): when phase rows do not exceed the nine PVA
    // states, postfit succeeds only if at least half survive the innovation
    // gate. With more rows it records a normalized chi-square; the outer
    // reference loop retries only on the sentinel value 100.
    if (accepted_phase_rows <= 9) {
        return total_phase_rows > 0 &&
               2 * accepted_phase_rows >= total_phase_rows;
    }
    const double limit = claslibChiSquare001ForDof(accepted_phase_rows - 9);
    return limit > 0.0 && normalized_sum / limit < 100.0;
}

EpochUpdateResult runEpochMeasurementUpdate(
    const ObservationData& obs,
    const CLASEpochContext& epoch_context,
    ppp_shared::PPPState& filter_state,
    const ppp_shared::PPPConfig& config,
    const PositionSolution& seed_solution,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const TropMappingFunction& trop_mapping_function,
    const AmbiguityResetFunction& ambiguity_reset_function,
    const AmbiguityIndexFunction& ambiguity_index_function,
    bool debug_enabled) {
    EpochUpdateResult result;
    const Vector3d state_position_before_update =
        filter_state.state.segment(filter_state.pos_index, 3);
    const Vector3d receiver_geometry_offset =
        epoch_context.receiver_position - state_position_before_update;
    auto measurement_build_result = buildEpochMeasurements(
        obs,
        epoch_context.osr_corrections,
        filter_state,
        config,
        epoch_context.receiver_position,
        epoch_context.receiver_clock_m,
        epoch_context.trop_zenith_m,
        epoch_context.epoch_atmos_tokens,
        trop_mapping_function,
        ambiguity_reset_function,
        debug_enabled);

    if (measurement_build_result.measurements.size() < 5) {
        return result;
    }

    const ppp_shared::PPPState pre_update_state = filter_state;
    std::set<SatelliteId> postfit_accepted_phase_ambiguities;
    result.update_stats = applyMeasurementUpdate(
        filter_state, measurement_build_result.measurements, config, &seed_solution);
    if (!result.update_stats.updated) {
        return result;
    }
    if (clasMrtklibFloatParity(config)) {
        std::set<SatelliteId> accepted_l1_phase_satellites;
        ppp_shared::PPPState postfit_probe_state = filter_state;
        const Vector3d postfit_position =
            filter_state.state.segment(filter_state.pos_index, 3) +
            receiver_geometry_offset;
        const double postfit_trop =
            filter_state.trop_index >= 0 &&
                    filter_state.trop_index < filter_state.total_states
                ? filter_state.state(filter_state.trop_index)
                : epoch_context.trop_zenith_m;
        const auto postfit_build = buildEpochMeasurements(
            obs,
            epoch_context.osr_corrections,
            postfit_probe_state,
            config,
            postfit_position,
            epoch_context.receiver_clock_m,
            postfit_trop,
            epoch_context.epoch_atmos_tokens,
            trop_mapping_function,
            AmbiguityResetFunction{},
            debug_enabled,
            0);
        if (!mrtklibPostfitRowsValid(
                filter_state, postfit_build.measurements,
                measurement_build_result.measurements,
                &accepted_l1_phase_satellites,
                &postfit_accepted_phase_ambiguities,
                debug_enabled)) {
            filter_state = pre_update_state;
            auto retry_build = buildEpochMeasurements(
                obs,
                epoch_context.osr_corrections,
                filter_state,
                config,
                epoch_context.receiver_position,
                epoch_context.receiver_clock_m,
                epoch_context.trop_zenith_m,
                epoch_context.epoch_atmos_tokens,
                trop_mapping_function,
                AmbiguityResetFunction{},
                debug_enabled,
                1);
            result.update_stats = applyMeasurementUpdate(
                filter_state, retry_build.measurements, config, &seed_solution);
            bool retry_valid = result.update_stats.updated;
            if (retry_valid) {
                accepted_l1_phase_satellites.clear();
                postfit_accepted_phase_ambiguities.clear();
                ppp_shared::PPPState retry_probe_state = filter_state;
                const Vector3d retry_position =
                    filter_state.state.segment(filter_state.pos_index, 3) +
                    receiver_geometry_offset;
                const double retry_trop =
                    filter_state.trop_index >= 0 &&
                            filter_state.trop_index < filter_state.total_states
                        ? filter_state.state(filter_state.trop_index)
                        : epoch_context.trop_zenith_m;
                const auto retry_postfit = buildEpochMeasurements(
                    obs,
                    epoch_context.osr_corrections,
                    retry_probe_state,
                    config,
                    retry_position,
                    epoch_context.receiver_clock_m,
                    retry_trop,
                    epoch_context.epoch_atmos_tokens,
                    trop_mapping_function,
                    AmbiguityResetFunction{},
                    debug_enabled,
                    1);
                retry_valid = mrtklibPostfitRowsValid(
                    filter_state, retry_postfit.measurements,
                    retry_build.measurements,
                    &accepted_l1_phase_satellites,
                    &postfit_accepted_phase_ambiguities,
                    debug_enabled);
            }
            if (!retry_valid) {
                filter_state = pre_update_state;
                result.update_stats = {};
                if (debug_enabled) {
                    std::cerr << "[CLAS-FILTER2] both references rejected; rollback\n";
                }
                return result;
            }
            measurement_build_result = std::move(retry_build);
            if (debug_enabled) {
                std::cerr << "[CLAS-FILTER2] second reference accepted\n";
            }
        }
        if (accepted_l1_phase_satellites.size() < 4) {
            filter_state = pre_update_state;
            result.updated = false;
            result.insufficient_valid_satellites = true;
            if (debug_enabled) {
                std::cerr << "[CLAS-FILTER2] lack of valid satellites ns="
                          << accepted_l1_phase_satellites.size()
                          << " sats=";
                for (const auto& satellite : accepted_l1_phase_satellites) {
                    std::cerr << satellite.toString() << ' ';
                }
                std::cerr << "\n";
            }
            return result;
        }

        // MRTKLIB filter2_() records Qp=(K*v)(K*v)' and, after a successful
        // float update, adapts each observed ionosphere Q diagonal using the
        // CLASLIB paper/config coefficients: forget=0.3, gain=3.0. udion()
        // clamps the result on the next epoch, not here.
        constexpr double kIonoForgetting = 0.3;
        constexpr double kIonoAdaptiveGainSquared = 3.0 * 3.0;
        const auto observed_satellite_list = obs.getSatellites();
        const std::set<SatelliteId> observed_satellites(
            observed_satellite_list.begin(), observed_satellite_list.end());
        for (const auto& satellite : observed_satellites) {
            const auto iono_it = filter_state.ionosphere_indices.find(satellite);
            if (iono_it == filter_state.ionosphere_indices.end() ||
                iono_it->second < 0 ||
                iono_it->second >= result.update_stats.dx.size()) {
                continue;
            }
            double& process_noise =
                filter_state.adaptive_ionosphere_process_noise[satellite];
            const double iono_update = result.update_stats.dx(iono_it->second);
            process_noise =
                kIonoForgetting * process_noise +
                (1.0 - kIonoForgetting) * kIonoAdaptiveGainSquared *
                    iono_update * iono_update;
        }
    }
    dumpClasCodeRows(
        "post",
        obs,
        epoch_context.osr_corrections,
        filter_state,
        config,
        trop_mapping_function);
    dumpClasPhaseRows(
        "post",
        obs,
        epoch_context.osr_corrections,
        filter_state,
        config,
        trop_mapping_function);

    updateObservedAmbiguities(
        obs.time,
        [&]() {
            if (!clasMrtklibFloatParity(config)) {
                return measurement_build_result.observed_ambiguities;
            }
            std::vector<AmbiguityObservation> accepted;
            accepted.reserve(measurement_build_result.observed_ambiguities.size());
            for (const auto& ambiguity_obs :
                 measurement_build_result.observed_ambiguities) {
                if (postfit_accepted_phase_ambiguities.count(
                        ambiguity_obs.ambiguity_satellite) != 0) {
                    accepted.push_back(ambiguity_obs);
                }
            }
            return accepted;
        }(),
        filter_state,
        ambiguity_states,
        ambiguity_index_function);
    result.updated = true;
    return result;
}

void updateObservedAmbiguities(
    const GNSSTime& time,
    const std::vector<AmbiguityObservation>& observed_ambiguities,
    const ppp_shared::PPPState& filter_state,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const AmbiguityIndexFunction& ambiguity_index_function) {
    for (const auto& ambiguity_obs : observed_ambiguities) {
        auto& ambiguity = ambiguity_states[ambiguity_obs.ambiguity_satellite];
        ambiguity.last_phase = ambiguity_obs.carrier_phase_cycles;
        ambiguity.last_time = time;
        ambiguity.lock_count += 1;
        ambiguity.outage_count = 0;
        ambiguity.quality_indicator = ambiguity_obs.snr;
        ambiguity.ambiguity_scale_m = ambiguity_obs.wavelength_m;
        ambiguity.needs_reinitialization = false;
        const int ambiguity_index =
            ambiguity_index_function(ambiguity_obs.ambiguity_satellite);
        if (ambiguity_index >= 0 && ambiguity_index < filter_state.total_states) {
            ambiguity.float_value = filter_state.state(ambiguity_index);
        }
    }
}

FixValidationStats validateFixedSolution(
    const ObservationData& obs,
    const std::vector<OSRCorrection>& osr_corrections,
    const ppp_shared::PPPState& filter_state,
    const ppp_shared::PPPConfig& config,
    const TropMappingFunction& trop_mapping_function,
    const AmbiguityIndexFunction& ambiguity_index_function,
    bool debug_enabled,
    const FixValidationOptions& options) {
    FixValidationStats stats;
    double phase_sum_sq = 0.0;
    double code_sum_sq = 0.0;
    double phase_chi_sq = 0.0;
    bool pair_validation_ok = true;
    SatelliteId worst_reference_satellite;
    SatelliteId worst_satellite;
    int worst_freq_group = -1;
    double worst_dd_residual_m = 0.0;
    double worst_dd_sigma = 0.0;
    SatelliteId worst_pair_satellite;
    double worst_pair_dispersive = 0.0;
    double worst_pair_nondispersive = 0.0;
    double worst_pair_sigma = 0.0;

    std::vector<PhaseResidualInfo> phase_residuals;
    const bool mrtklib_parity = clasMrtklibFloatParity(config);

    const Vector3d receiver_position =
        filter_state.state.segment(filter_state.pos_index, 3);
    const double trop_zenith = filter_state.state(filter_state.trop_index);
    for (const auto& osr : osr_corrections) {
        if (!osr.valid) {
            continue;
        }
        // The float ddres path is the source of the stage-2 fixed residual
        // set as well.  On this v0.5.1 CLAS dataset Galileo never acquires a
        // valid phase row, and QZSS contributes L1 only.
        if (mrtklib_parity &&
            osr.satellite.system == GNSSSystem::Galileo) {
            continue;
        }

        const double geo = geodist(osr.satellite_position, receiver_position);
        const double sat_clk_m =
            constants::SPEED_OF_LIGHT * osr.satellite_clock_bias_s;
        const double receiver_clock_m =
            receiverClockBiasMeters(filter_state, osr.satellite);
        const double trop_mapping =
            trop_mapping_function(receiver_position, osr.elevation, obs.time);
        const double trop_modeled = trop_mapping * trop_zenith;

        std::array<const Observation*, OSR_MAX_FREQ> raw_observations{};
        for (int f = 0; f < osr.num_frequencies; ++f) {
            raw_observations[static_cast<size_t>(f)] =
                findOsrFrequencyObservation(obs, osr, f);
        }

        for (int f = 0; f < osr.num_frequencies; ++f) {
            if (mrtklib_parity &&
                osr.satellite.system == GNSSSystem::QZSS && f > 0) {
                continue;
            }
            const Observation* raw = raw_observations[static_cast<size_t>(f)];
            if (raw == nullptr || !raw->valid) {
                continue;
            }
            const auto applied_corrections = selectAppliedOsrCorrections(
                osr, f, config.clas_correction_application_policy);

            const auto iono_it = filter_state.ionosphere_indices.find(osr.satellite);
            const double iono_scale =
                (config.estimate_ionosphere &&
                 iono_it != filter_state.ionosphere_indices.end() &&
                 iono_it->second >= 0 &&
                 iono_it->second < filter_state.total_states &&
                 osr.frequencies[f] > 0.0 &&
                 osr.wavelengths[0] > 0.0)
                    ? std::pow(osr.wavelengths[f] / osr.wavelengths[0], 2)
                    : 0.0;
            const double iono_state_m =
                iono_scale > 0.0 ? filter_state.state(iono_it->second) : 0.0;

            const double el_weight = elevationWeight(osr.elevation);
            // Mirror the buildEpochMeasurements() phase model: the applied
            // troposphere term depends on the correction application policy.
            const bool claslib_amb_datum_phase =
                suppressesClasAmbDatumPhaseTrop(
                    config.clas_correction_application_policy);
            const bool residual_amb_datum_phase_trop =
                usesResidualClasAmbDatumPhaseTrop(
                    config.clas_correction_application_policy);
            const double phase_trop_modeled = mrtklib_parity
                ? 0.0
                : (claslib_amb_datum_phase
                      ? 0.0
                      : (residual_amb_datum_phase_trop
                            ? trop_modeled - osr.trop_correction_m
                            : trop_modeled));
            const double phase_predicted =
                geo - sat_clk_m + receiver_clock_m + phase_trop_modeled;

            if (raw->has_pseudorange && std::isfinite(raw->pseudorange)) {
                const bool claslib_code_prc = usesClaslibCodePrcRows(config);
                const double code_trop_modeled =
                    claslib_code_prc ? 0.0 : trop_modeled;
                const double code_predicted =
                    geo - sat_clk_m + receiver_clock_m + code_trop_modeled;
                const double residual =
                    (raw->pseudorange - applied_corrections.pseudorange_correction_m) -
                    (code_predicted + iono_scale * iono_state_m);
                code_sum_sq += residual * residual;
                ++stats.code_rows;
            }

            if (!raw->has_carrier_phase || !std::isfinite(raw->carrier_phase) ||
                (mrtklib_parity &&
                 (!raw->has_pseudorange ||
                  !std::isfinite(raw->pseudorange)))) {
                continue;
            }

            const uint8_t ambiguity_prn = f == 0 ? osr.satellite.prn :
                static_cast<uint8_t>(std::min(255, osr.satellite.prn + 100));
            const SatelliteId ambiguity_satellite(osr.satellite.system, ambiguity_prn);
            const int ambiguity_index = ambiguity_index_function(ambiguity_satellite);
            if (ambiguity_index < 0 || ambiguity_index >= filter_state.total_states) {
                continue;
            }

            const double carrier_phase_m = raw->carrier_phase * osr.wavelengths[f];
            const double residual =
                (carrier_phase_m - applied_corrections.carrier_phase_correction_m) -
                (phase_predicted - iono_scale * iono_state_m +
                 filter_state.state(ambiguity_index));
            const double variance =
                clasPhaseVariance(config, osr.elevation, f, osr.frequencies[f]);
            PhaseResidualInfo info;
            info.ambiguity_satellite = ambiguity_satellite;
            info.real_satellite = osr.satellite;
            info.residual_m = residual;
            info.variance_m2 = variance;
            info.frequency_hz = osr.frequencies[f];
            info.wavelength_m = osr.wavelengths[f];
            info.elevation_rad = osr.elevation;
            const double range = std::max(geo, 1.0);
            info.unit_vector =
                (receiver_position - osr.satellite_position) / range;
            info.trop_mapping = trop_mapping;
            info.iono_index =
                iono_scale > 0.0 ? iono_it->second : -1;
            info.iono_scale = iono_scale;
            info.ambiguity_index = ambiguity_index;
            phase_residuals.push_back(std::move(info));
        }
    }

    // MRTKLIB innovation-variance basis: h_dd' P h_dd for one DD phase row
    // (reference r minus satellite s within the same system group; receiver
    // clock cancels). h entries: position (u_r - u_s), troposphere zenith
    // (m_r - m_s), per-satellite ionosphere (-mu_r at r, +mu_s at s; phase
    // sign), ambiguity states (+1 at r, -1 at s; states are in meters).
    const auto dd_state_variance =
        [&](const PhaseResidualInfo& r, const PhaseResidualInfo& s) -> double {
        const MatrixXd& P = *options.innovation_covariance;
        const int n = static_cast<int>(P.rows());
        std::array<std::pair<int, double>, 9> h{};
        size_t k = 0;
        for (int axis = 0; axis < 3; ++axis) {
            h[k++] = {filter_state.pos_index + axis,
                      r.unit_vector(axis) - s.unit_vector(axis)};
        }
        h[k++] = {filter_state.trop_index, r.trop_mapping - s.trop_mapping};
        if (r.iono_index >= 0 && r.iono_index < n) {
            h[k++] = {r.iono_index, -r.iono_scale};
        }
        if (s.iono_index >= 0 && s.iono_index < n) {
            h[k++] = {s.iono_index, s.iono_scale};
        }
        if (r.ambiguity_index >= 0 && r.ambiguity_index < n) {
            h[k++] = {r.ambiguity_index, 1.0};
        }
        if (s.ambiguity_index >= 0 && s.ambiguity_index < n) {
            h[k++] = {s.ambiguity_index, -1.0};
        }
        double quad = 0.0;
        for (size_t i = 0; i < k; ++i) {
            for (size_t j = 0; j < k; ++j) {
                quad += h[i].second * P(h[i].first, h[j].first) * h[j].second;
            }
        }
        return std::max(quad, 0.0);
    };

    std::map<std::pair<GNSSSystem, int>, std::vector<PhaseResidualInfo>> dd_groups;
    for (const auto& phase_residual : phase_residuals) {
        dd_groups[ppp_ar::ambiguityDdGroup(phase_residual.ambiguity_satellite)]
            .push_back(phase_residual);
    }

    std::map<SatelliteId, PhasePairInfo> phase_pairs;
    for (auto& [group, residuals] : dd_groups) {
        std::sort(residuals.begin(), residuals.end(),
                  [mrtklib_parity](const PhaseResidualInfo& lhs,
                                   const PhaseResidualInfo& rhs) {
                      if (mrtklib_parity &&
                          lhs.elevation_rad != rhs.elevation_rad) {
                          // ddres() selects the highest-elevation valid
                          // satellite as the reference independently for each
                          // system/frequency block.
                          return lhs.elevation_rad > rhs.elevation_rad;
                      }
                      if (lhs.real_satellite.system != rhs.real_satellite.system) {
                          return static_cast<int>(lhs.real_satellite.system) <
                                 static_cast<int>(rhs.real_satellite.system);
                      }
                      return lhs.real_satellite.prn < rhs.real_satellite.prn;
                  });
        if (residuals.size() < 2) {
            continue;
        }

        const auto& reference = residuals.front();
        const size_t freq_slot = static_cast<size_t>(group.second);
        for (size_t index = 1; index < residuals.size(); ++index) {
            const auto& residual = residuals[index];
            const double dd_residual = reference.residual_m - residual.residual_m;
            double dd_variance = reference.variance_m2 + residual.variance_m2;
            if (options.innovation_covariance != nullptr &&
                options.innovation_covariance->rows() ==
                    filter_state.total_states) {
                // MRTKLIB filter2_ basis (mrtk_ppp_rtk.c:1125): the residual
                // gate and chi-square normalize by the innovation covariance
                // Q = H'*P*H + R, not by R alone.
                dd_variance += dd_state_variance(reference, residual);
            }
            const double sigma = std::sqrt(std::max(dd_variance, 1e-12));

            // MRTKLIB residual_test() (mrtk_ppp_rtk.c:1040-1058): individual
            // residuals beyond the rejionno1 sigma gate are excluded from the
            // chi-square sum instead of failing the whole validation. D2 rule
            // (mrtk_ppp_rtk.c:1046-1050): the gate (not the chi-square) is
            // inflated 10x for rows whose phase-bias state is still at its
            // initialization variance.
            double gate_scale = 1.0;
            if (options.innovation_covariance != nullptr &&
                options.innovation_covariance->rows() ==
                    filter_state.total_states) {
                const MatrixXd& P_gate = *options.innovation_covariance;
                const double init_var = 1e4;  // MRTKLIB SQR(std[0]=100)
                for (const int amb_idx :
                     {reference.ambiguity_index, residual.ambiguity_index}) {
                    if (amb_idx >= 0 && amb_idx < P_gate.rows() &&
                        std::abs(P_gate(amb_idx, amb_idx) - init_var) <
                            1e-3 * init_var) {
                        gate_scale = 10.0;
                    }
                }
            }
            if (options.outlier_sigma_gate > 0.0 &&
                dd_residual * dd_residual >
                    options.outlier_sigma_gate * options.outlier_sigma_gate *
                        gate_scale * gate_scale * dd_variance) {
                ++stats.phase_outlier_rows;
                continue;
            }

            phase_sum_sq += dd_residual * dd_residual;
            phase_chi_sq += dd_residual * dd_residual /
                            std::max(dd_variance, 1e-12);
            ++stats.phase_rows;
            stats.max_phase_sigma =
                std::max(stats.max_phase_sigma, std::abs(dd_residual) / sigma);
            if (sigma > 0.0 &&
                std::abs(dd_residual) / sigma >=
                    std::abs(worst_dd_residual_m) /
                        std::max(worst_dd_sigma, 1e-12)) {
                worst_reference_satellite = reference.real_satellite;
                worst_satellite = residual.real_satellite;
                worst_freq_group = static_cast<int>(freq_slot);
                worst_dd_residual_m = dd_residual;
                worst_dd_sigma = sigma;
            }

            if (freq_slot < 2) {
                auto& pair_info = phase_pairs[residual.real_satellite];
                pair_info.residual_m[freq_slot] = dd_residual;
                pair_info.variance_m2[freq_slot] = dd_variance;
                pair_info.frequency_hz[freq_slot] = residual.frequency_hz;
                pair_info.wavelength_m[freq_slot] = residual.wavelength_m;
            }
        }
    }

    for (const auto& [satellite, pair_info] : phase_pairs) {
        const bool has_l1 =
            std::isfinite(pair_info.residual_m[0]) &&
            std::isfinite(pair_info.variance_m2[0]) &&
            pair_info.frequency_hz[0] > 0.0 &&
            pair_info.wavelength_m[0] > 0.0;
        const bool has_l2 =
            std::isfinite(pair_info.residual_m[1]) &&
            std::isfinite(pair_info.variance_m2[1]) &&
            pair_info.frequency_hz[1] > 0.0 &&
            pair_info.wavelength_m[1] > 0.0;
        if (!has_l1 || !has_l2) {
            continue;
        }

        const double gamma =
            std::pow(pair_info.wavelength_m[1] / pair_info.wavelength_m[0], 2);
        const double denom = 1.0 - gamma;
        if (std::abs(denom) <= 1e-9) {
            continue;
        }

        const double max_variance =
            std::max(pair_info.variance_m2[0], pair_info.variance_m2[1]);
        const double dispersive =
            (pair_info.frequency_hz[0] / pair_info.frequency_hz[1]) *
            (pair_info.residual_m[0] - pair_info.residual_m[1]) / denom;
        const double nondispersive =
            (gamma * pair_info.residual_m[0] - pair_info.residual_m[1]) /
            (gamma - 1.0);
        worst_pair_satellite = satellite;
        worst_pair_dispersive = dispersive;
        worst_pair_nondispersive = nondispersive;
        worst_pair_sigma = std::sqrt(std::max(max_variance, 1e-12));
        const double pair_validation_scale = mrtklib_parity ? 9.0 : 64.0;
        if (dispersive * dispersive > pair_validation_scale * max_variance ||
            nondispersive * nondispersive > pair_validation_scale * max_variance) {
            pair_validation_ok = false;
            break;
        }
    }

    const int position_dof =
        (config.kinematic_mode || config.use_dynamics_model) ? 9 : 3;
    if (stats.phase_rows > 0) {
        stats.phase_rms = std::sqrt(phase_sum_sq / stats.phase_rows);
    }
    if (stats.code_rows > 0) {
        stats.code_rms = std::sqrt(code_sum_sq / stats.code_rows);
    }
    if (stats.phase_rows > position_dof) {
        const double chi_square_limit =
            claslibChiSquare001ForDof(stats.phase_rows - position_dof);
        if (chi_square_limit > 0.0) {
            stats.phase_chisq = phase_chi_sq / chi_square_limit;
        }
    } else if (options.mrtklib_chisq_fallback) {
        // MRTKLIB residual_test() fallback (mrtk_ppp_rtk.c:1080-1082): with
        // too few rows for a chi-square test, pass when at least half of the
        // carrier residuals survived the outlier gate.
        const int total_rows = stats.phase_rows + stats.phase_outlier_rows;
        stats.phase_chisq =
            (total_rows <= 0 ||
             static_cast<double>(stats.phase_rows) / total_rows < 0.5)
                ? 100.0
                : 0.0;
    }

    constexpr double kMaxPhaseSigma = 4.0;
    constexpr double kMaxPhaseChisq = 5.0;
    stats.accepted =
        stats.phase_rows > position_dof &&
        stats.max_phase_sigma < kMaxPhaseSigma &&
        pair_validation_ok &&
        stats.phase_chisq < kMaxPhaseChisq;
    if (debug_enabled && !stats.accepted) {
        std::cerr << "[CLAS-FIX-DBG] worst_dd ref="
                  << worst_reference_satellite.toString()
                  << " sat=" << worst_satellite.toString()
                  << " freq=" << worst_freq_group
                  << " resid=" << worst_dd_residual_m
                  << " sigma=" << worst_dd_sigma
                  << " pair_sat=" << worst_pair_satellite.toString()
                  << " dispersive=" << worst_pair_dispersive
                  << " nondisp=" << worst_pair_nondispersive
                  << " pair_sigma=" << worst_pair_sigma
                  << "\n";
    }
    return stats;
}

AmbiguityResolutionResult resolveAndValidateAmbiguities(
    ppp_shared::PPPState& filter_state,
    std::map<SatelliteId, ppp_shared::PPPAmbiguityInfo>& ambiguity_states,
    const ResolveAmbiguitiesFunction& resolve_ambiguities,
    const ValidateFixedSolutionFunction& validate_fixed_solution,
    bool debug_enabled) {
    AmbiguityResolutionResult result;
    if (!resolve_ambiguities) {
        return result;
    }

    const ppp_shared::PPPState pre_fix_state = filter_state;
    const auto pre_fix_ambiguities = ambiguity_states;

    result.attempted = true;
    if (!resolve_ambiguities()) {
        return result;
    }

    if (validate_fixed_solution) {
        result.validation_stats = validate_fixed_solution();
        result.accepted = result.validation_stats.accepted;
    } else {
        result.accepted = true;
    }

    if (!result.accepted) {
        filter_state = pre_fix_state;
        ambiguity_states = pre_fix_ambiguities;
        result.rejected_after_fix = true;
        if (debug_enabled) {
            std::cerr << "[CLAS-FIX] reject: phase_rows="
                      << result.validation_stats.phase_rows
                      << " phase_rms=" << result.validation_stats.phase_rms
                      << " phase_chisq=" << result.validation_stats.phase_chisq
                      << " max_phase_sigma=" << result.validation_stats.max_phase_sigma
                      << "\n";
        }
        return result;
    }

    if (debug_enabled) {
        std::cerr << "[CLAS-FIX] accept: phase_rows="
                  << result.validation_stats.phase_rows
                  << " phase_rms=" << result.validation_stats.phase_rms
                  << " phase_chisq=" << result.validation_stats.phase_chisq
                  << " max_phase_sigma=" << result.validation_stats.max_phase_sigma
                  << "\n";
    }

    return result;
}

void logUpdateSummary(
    const KalmanUpdateStats& update_stats,
    size_t satellite_count) {
    double code_rms = 0.0;
    double phase_rms = 0.0;
    int n_code = 0;
    int n_phase = 0;
    constexpr double kCodePhaseVarianceBoundary = 0.5;
    for (int i = 0; i < update_stats.nobs; ++i) {
        if (update_stats.variances(i) > kCodePhaseVarianceBoundary) {
            code_rms += update_stats.residuals(i) * update_stats.residuals(i);
            ++n_code;
        } else {
            phase_rms += update_stats.residuals(i) * update_stats.residuals(i);
            ++n_phase;
        }
    }
    if (n_code > 0) {
        code_rms = std::sqrt(code_rms / n_code);
    }
    if (n_phase > 0) {
        phase_rms = std::sqrt(phase_rms / n_phase);
    }
    std::cerr << "[CLAS-PPP] rows=" << update_stats.nobs
              << " sats=" << satellite_count
              << " pos_delta=" << update_stats.dx.head(3).norm()
              << " code_rms=" << code_rms
              << " phase_rms=" << phase_rms
              << "\n";
}

}  // namespace libgnss::ppp_clas
