#include <libgnss++/algorithms/rtk.hpp>
#include <libgnss++/algorithms/rtk_ar_evaluation.hpp>
#include <libgnss++/algorithms/rtk_ar_selection.hpp>
#include <libgnss++/algorithms/disjoint_satellite_fix_evidence.hpp>
#include <libgnss++/algorithms/fix_failure_budget.hpp>
#include <libgnss++/algorithms/lambda.hpp>
#include <libgnss++/algorithms/rtk_cp_pr_gate.hpp>
#include <libgnss++/algorithms/rtk_ddpr_anchor.hpp>
#include <libgnss++/algorithms/rtk_measurement.hpp>
#include <libgnss++/algorithms/rtk_selection.hpp>
#include <libgnss++/algorithms/rtk_ins_time_update.hpp>
#include <libgnss++/algorithms/rtk_tdcp_diagnostics.hpp>
#include <libgnss++/algorithms/rtk_update.hpp>
#include <libgnss++/algorithms/spp_velocity.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/signal_policy.hpp>
#include <libgnss++/core/signals.hpp>
#include <libgnss++/models/troposphere.hpp>
#include <iostream>
#include <iterator>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <set>

#include "rtk_internal.hpp"

namespace libgnss {

using namespace rtk_internal;

std::vector<rtk_measurement::MeasurementBlock> RTKProcessor::buildMeasurementBlocks(
    const std::map<SatelliteId, SatelliteData>& sat_data) const {
    const Vector3d rover_pos = base_position_ + filter_state_.state.head<3>();
    const bool estimate_iono = usesEstimatedIono(rtk_config_);
    const auto selection_snapshot = buildSelectionSnapshot(sat_data);
    std::vector<rtk_measurement::MeasurementBlock> blocks;

    // navi.776 B2: rover-only between-satellite SD Doppler rows. Skipped
    // until the first INS position/velocity time update has initialized the
    // velocity covariance -- before that the rows would carry residuals with
    // no active velocity columns and pollute NIS.
    const bool adaptive_noise_active = adaptiveNoiseActiveThisEpoch();
    const bool doppler_baseline_gate_passes =
        rtk_config_.doppler_row_max_baseline_m <= 0.0 ||
        !std::isfinite(rtk_config_.doppler_row_max_baseline_m) ||
        filter_state_.state.size() < 3 ||
        !(filter_state_.state.head<3>().norm() >
          rtk_config_.doppler_row_max_baseline_m);
    const bool build_doppler_rows =
        rtk_config_.enable_doppler_measurement_rows &&
        doppler_baseline_gate_passes &&
        rtk_config_.enable_velocity_states &&
        filter_state_.state.size() >= VELOCITY_STATE_INDEX + VELOCITY_STATES &&
        filter_state_.covariance(VELOCITY_STATE_INDEX, VELOCITY_STATE_INDEX) > 0.0;
    const Vector3d velocity_estimate =
        build_doppler_rows ? Vector3d(filter_state_.state.segment<3>(VELOCITY_STATE_INDEX))
                           : Vector3d::Zero();
    // Predicted range rate at the current velocity estimate (RTKLIB resdop
    // form): e.(v_sat - v_rx) + Sagnac rate term - c * sat clock drift. The
    // receiver clock drift term is omitted -- it cancels exactly in the
    // between-satellite difference.
    auto predicted_range_rate_mps = [&](const SatelliteData& s) -> double {
        const Vector3d e = (s.sat_pos - rover_pos).normalized();
        const double sagnac_rate =
            constants::OMEGA_E / constants::SPEED_OF_LIGHT *
            (s.sat_vel.y() * rover_pos.x() + s.sat_pos.y() * velocity_estimate.x() -
             s.sat_vel.x() * rover_pos.y() - s.sat_pos.x() * velocity_estimate.y());
        return (s.sat_vel - velocity_estimate).dot(e) + sagnac_rate -
               constants::SPEED_OF_LIGHT * s.sat_clock_drift;
    };

    // WP7: NLOS/multipath sigma inflation. Returns 1.0 (no-op) whenever the
    // feature is off or no table/entry is available, so this is bit-identical
    // to pre-WP7 behavior by construction when nlos_weight_mode == OFF.
    const bool nlos_weighting_active =
        rtk_config_.nlos_weight_mode != nlos_weights::NlosWeightMode::OFF &&
        nlos_weight_table_ && !nlos_weight_table_->empty();
    auto nlos_variance_factor = [&](const SatelliteId& sat) -> double {
        if (!nlos_weighting_active) return 1.0;
        const double los_prob = nlos_weights::lookupLosProb(
            *nlos_weight_table_, current_epoch_time_.tow, sat.toString(),
            rtk_config_.nlos_tow_tolerance_s);
        return nlos_weights::nlosVarianceInflationFactor(
            los_prob,
            rtk_config_.nlos_weight_mode,
            rtk_config_.nlos_continuous_los_prob_floor,
            rtk_config_.nlos_two_tier_los_threshold,
            rtk_config_.nlos_two_tier_sigma_inflation);
    };

    for (GNSSSystem system : kRTKSupportedSystems) {
        if (!isEnabledRTKSystem(rtk_config_, system)) continue;
        SatelliteId ref_sat;
        const SatelliteId* forced_ref = nullptr;
        if (rtk_config_.cmc_aware_reference_selection) {
            const auto forced_it = cmc_aware_ref_by_system_.find(system);
            if (forced_it != cmc_aware_ref_by_system_.end()) forced_ref = &forced_it->second;
        }
        if (forced_ref != nullptr) {
            ref_sat = *forced_ref;
        } else if (!rtk_selection::selectSystemReferenceSatellite(selection_snapshot, system, 0,
                                                                    ref_sat)) {
            continue;
        }

        auto ref_it = sat_data.find(ref_sat);
        if (ref_it == sat_data.end()) continue;
        const auto& ref_sd = ref_it->second;
        const auto system_pairs = rtk_selection::buildDoubleDifferencePairsForSystem(
            selection_snapshot,
            system,
            0,
            requiresMatchedCarrierWavelength(rtk_config_, system),
            forced_ref);
        const double rr_ref = geodist_range(ref_sd.sat_pos, rover_pos) +
                              tropModel(rover_pos, ref_sd.elevation);
        const double br_ref = geodist_range(ref_sd.sat_pos_base, base_position_) +
                              tropModel(base_position_, ref_sd.base_elevation);
        const Vector3d los_ref = (ref_sd.sat_pos - rover_pos).normalized();
        auto signal_snr_dbhz = [](const SatelliteData& sd, int freq) {
            if (freq == 0) return combinedSnrDbHz(sd.rover_l1_snr, sd.base_l1_snr);
            if (freq == 1) return combinedSnrDbHz(sd.rover_l2_snr, sd.base_l2_snr);
            return combinedSnrDbHz(sd.rover_l5_snr, sd.base_l5_snr);  // Phase 18 Step 4
        };
        // Phase 18 Step 4: per-frequency accessors (parallel to updateBias). freq=2 returns L5.
        auto freq_wavelength_local = [](const SatelliteData& sd, int freq) -> double {
            if (freq == 0) return sd.l1_wavelength;
            if (freq == 1) return sd.l2_wavelength;
            return sd.l5_wavelength;
        };
        auto freq_frequency_hz_local = [](const SatelliteData& sd, int freq) -> double {
            if (freq == 0) return sd.l1_frequency_hz;
            if (freq == 1) return sd.l2_frequency_hz;
            return sd.l5_frequency_hz;
        };
        auto freq_phase_diff_local = [](const SatelliteData& sd, int freq) -> double {
            if (freq == 0) return sd.rover_l1_phase - sd.base_l1_phase;
            if (freq == 1) return sd.rover_l2_phase - sd.base_l2_phase;
            return sd.rover_l5_phase - sd.base_l5_phase;
        };
        auto freq_code_diff_local = [](const SatelliteData& sd, int freq) -> double {
            if (freq == 0) return sd.rover_l1_code - sd.base_l1_code;
            if (freq == 1) return sd.rover_l2_code - sd.base_l2_code;
            return sd.rover_l5_code - sd.base_l5_code;
        };
        auto freq_rover_doppler_local = [](const SatelliteData& sd, int freq) -> double {
            if (freq == 0) return sd.rover_l1_doppler;
            if (freq == 1) return sd.rover_l2_doppler;
            return sd.rover_l5_doppler;
        };
        auto freq_has_doppler_local = [](const SatelliteData& sd, int freq) -> bool {
            if (freq == 0) return sd.has_l1_doppler;
            if (freq == 1) return sd.has_l2_doppler;
            return sd.has_l5_doppler;
        };
        // Elevation-law Doppler variance, same 1/sin^2(el) family as varerr.
        auto doppler_variance = [&](double elevation) -> double {
            double sin_el = std::sin(elevation);
            if (sin_el < 0.1) sin_el = 0.1;
            const double sigma = rtk_config_.doppler_row_sigma_mps;
            return sigma * sigma * (1.0 + 1.0 / (sin_el * sin_el));
        };

        auto append_frequency_blocks = [&](int freq) {
            rtk_measurement::MeasurementBlock phase_block;
            phase_block.kind = rtk_measurement::MeasurementKind::PHASE;
            phase_block.frequency_index = freq;
            rtk_measurement::MeasurementBlock code_block;
            code_block.kind = rtk_measurement::MeasurementKind::CODE;
            code_block.frequency_index = freq;
            rtk_measurement::MeasurementBlock doppler_block;
            doppler_block.kind = rtk_measurement::MeasurementKind::DOPPLER;
            doppler_block.frequency_index = freq;
            const bool ref_doppler_ok =
                build_doppler_rows && freq_has_doppler_local(ref_sd, freq) &&
                ref_sd.has_sat_velocity &&
                freq_frequency_hz_local(ref_sd, freq) > 0.0;
            // Between-satellite SD reference terms: measured range rate
            // (RINEX sign: rr = -D * c / f) and prediction at current v_hat.
            const double rr_obs_ref =
                ref_doppler_ok
                    ? -freq_rover_doppler_local(ref_sd, freq) *
                          (constants::SPEED_OF_LIGHT / freq_frequency_hz_local(ref_sd, freq))
                    : 0.0;
            const double rr_pred_ref = ref_doppler_ok ? predicted_range_rate_mps(ref_sd) : 0.0;
            const Vector3d e_ref_doppler =
                ref_doppler_ok ? Vector3d((ref_sd.sat_pos - rover_pos).normalized())
                               : Vector3d::Zero();
            const double ref_doppler_variance =
                ref_doppler_ok ? doppler_variance(ref_sd.elevation) : 0.0;
            const auto& ref_indices = (freq == 0) ? filter_state_.n1_indices :
                                      (freq == 1) ? filter_state_.n2_indices :
                                                    filter_state_.n5_indices;
            auto ref_state_it = ref_indices.find(ref_sat);
            if (ref_state_it == ref_indices.end()) {
                blocks.push_back(std::move(phase_block));
                blocks.push_back(std::move(code_block));
                return;
            }

            const double ref_wavelength = freq_wavelength_local(ref_sd, freq);
            if (ref_wavelength <= 0.0) {
                blocks.push_back(std::move(phase_block));
                blocks.push_back(std::move(code_block));
                return;
            }
            const int ref_iono_idx =
                estimate_iono ? II(ref_sat) : -1;
            const double ref_iono_scale =
                estimate_iono
                    ? ionoFrequencyScale(
                          freq,
                          ref_sd.l1_frequency_hz,
                          freq_frequency_hz_local(ref_sd, freq))
                    : 0.0;
            if (estimate_iono && filter_state_.covariance(ref_iono_idx, ref_iono_idx) <= 0.0) {
                blocks.push_back(std::move(phase_block));
                blocks.push_back(std::move(code_block));
                return;
            }
            const double ref_snr = signal_snr_dbhz(ref_sd, freq);
            const double ref_nlos_factor = nlos_variance_factor(ref_sat);
            const double ref_phase_variance = varerr(ref_sd.elevation, true, ref_snr) * ref_nlos_factor;
            const double ref_code_variance = varerr(ref_sd.elevation, false, ref_snr) * ref_nlos_factor;

            for (const auto& pair : system_pairs) {
                if (pair.freq != freq) continue;
                const auto sat_it = sat_data.find(pair.sat);
                if (sat_it == sat_data.end()) continue;
                const auto& sat = pair.sat;
                const auto& sd = sat_it->second;
                const auto& sat_indices = (freq == 0) ? filter_state_.n1_indices :
                                          (freq == 1) ? filter_state_.n2_indices :
                                                        filter_state_.n5_indices;
                auto sat_state_it = sat_indices.find(sat);
                if (sat_state_it == sat_indices.end()) continue;

                const double sat_wavelength = freq_wavelength_local(sd, freq);
                if (sat_wavelength <= 0.0) continue;
                const double sat_snr = signal_snr_dbhz(sd, freq);
                const double sat_nlos_factor = nlos_variance_factor(sat);
                double sat_phase_variance = varerr(sd.elevation, true, sat_snr) * sat_nlos_factor;
                double sat_code_variance = varerr(sd.elevation, false, sat_snr) * sat_nlos_factor;
                // navi.776 A2: replace the satellite-side model variance with
                // the innovation-adapted one. The reference-side variance
                // stays at the model value so the DD block structure
                // (ref_var*11' + diag) keeps its known correlated part.
                const int adaptive_key = freq * MAXSAT + satelliteSlot(sat);
                const double sat_phase_model_variance = sat_phase_variance;
                const double sat_code_model_variance = sat_code_variance;
                if (adaptive_noise_active) {
                    const auto adaptive_config =
                        adaptiveNoiseConfig(sd.satellite);
                    sat_phase_variance = adaptive_noise_tracker_.adaptedVariance(
                        adaptive_key, rtk_measurement::MeasurementKind::PHASE,
                        sat_phase_model_variance, adaptive_config);
                    if (!rtk_config_.adaptive_noise_phase_only) {
                        sat_code_variance = adaptive_noise_tracker_.adaptedVariance(
                            adaptive_key, rtk_measurement::MeasurementKind::CODE,
                            sat_code_model_variance, adaptive_config);
                    }
                }
                const int sat_iono_idx = estimate_iono ? II(sat) : -1;
                const double sat_iono_scale =
                    estimate_iono
                        ? ionoFrequencyScale(
                              freq,
                              sd.l1_frequency_hz,
                              freq_frequency_hz_local(sd, freq))
                        : 0.0;
                if (estimate_iono &&
                    filter_state_.covariance(sat_iono_idx, sat_iono_idx) <= 0.0) {
                    continue;
                }

                const double rr = geodist_range(sd.sat_pos, rover_pos) + tropModel(rover_pos, sd.elevation);
                const double br = geodist_range(sd.sat_pos_base, base_position_) +
                                  tropModel(base_position_, sd.base_elevation);
                const double geom_dd = (rr_ref - br_ref) - (rr - br);
                const Vector3d dd_los = -los_ref + (sd.sat_pos - rover_pos).normalized();
                const double ref_iono_state =
                    estimate_iono ? filter_state_.state(ref_iono_idx) : 0.0;
                const double sat_iono_state =
                    estimate_iono ? filter_state_.state(sat_iono_idx) : 0.0;
                const double ref_phase = freq_phase_diff_local(ref_sd, freq);
                const double sat_phase = freq_phase_diff_local(sd, freq);
                const double ref_code = freq_code_diff_local(ref_sd, freq);
                const double sat_code = freq_code_diff_local(sd, freq);
                // Glonass autocal/ICB only meaningful on L1/L2 (no GLO L5 path); freq==2 → false.
                const bool autocal_glonass =
                    usesGlonassAutocal(rtk_config_) &&
                    ref_sd.satellite.system == GNSSSystem::GLONASS &&
                    sd.satellite.system == GNSSSystem::GLONASS &&
                    freq < GLO_HWBIAS_STATES;
                const double df_mhz =
                    autocal_glonass
                        ? (freq_frequency_hz_local(ref_sd, freq) -
                           freq_frequency_hz_local(sd, freq)) / 1e6
                        : 0.0;
                const double glonass_icb =
                    glonassInterChannelBiasMeters(
                        rtk_config_,
                        ref_sd.satellite.system,
                        sd.satellite.system,
                        freq_frequency_hz_local(ref_sd, freq),
                        freq_frequency_hz_local(sd, freq),
                        freq);
                const double phase_iono_term =
                    estimate_iono ?
                        (-ref_iono_scale * ref_iono_state + sat_iono_scale * sat_iono_state) :
                        0.0;
                const double code_iono_term =
                    estimate_iono ?
                        (ref_iono_scale * ref_iono_state - sat_iono_scale * sat_iono_state) :
                        0.0;

                rtk_measurement::MeasurementRow phase_row;
                const double amb_term =
                    ref_wavelength * filter_state_.state(ref_state_it->second) -
                    sat_wavelength * filter_state_.state(sat_state_it->second);
                phase_row.residual = ref_phase * ref_wavelength - sat_phase * sat_wavelength -
                                     geom_dd - amb_term - glonass_icb - phase_iono_term;
                if (autocal_glonass) {
                    phase_row.residual -= df_mhz * filter_state_.state(IL(freq));
                    phase_row.state_coefficients.push_back({IL(freq), df_mhz});
                }
                phase_row.state_coefficients.push_back({ref_state_it->second, ref_wavelength});
                phase_row.state_coefficients.push_back({sat_state_it->second, -sat_wavelength});
                if (estimate_iono) {
                    phase_row.state_coefficients.push_back({ref_iono_idx, -ref_iono_scale});
                    phase_row.state_coefficients.push_back({sat_iono_idx, sat_iono_scale});
                }
                phase_row.baseline_coefficients = dd_los;
                phase_row.reference_variance = ref_phase_variance;
                phase_row.satellite_variance = sat_phase_variance;
                phase_row.adaptive_key = adaptive_key;
                phase_row.adaptive_model_variance = sat_phase_model_variance;
                phase_block.rows.push_back(std::move(phase_row));

                rtk_measurement::MeasurementRow code_row;
                code_row.residual = (ref_code - sat_code) - geom_dd - code_iono_term;
                if (estimate_iono) {
                    code_row.state_coefficients.push_back({ref_iono_idx, ref_iono_scale});
                    code_row.state_coefficients.push_back({sat_iono_idx, -sat_iono_scale});
                }
                code_row.baseline_coefficients = dd_los;
                code_row.reference_variance = ref_code_variance;
                code_row.satellite_variance = sat_code_variance;
                code_row.adaptive_key = adaptive_key;
                code_row.adaptive_model_variance = sat_code_model_variance;
                code_block.rows.push_back(std::move(code_row));

                // navi.776 B2: SD Doppler row for this satellite pair.
                if (ref_doppler_ok && freq_has_doppler_local(sd, freq) &&
                    sd.has_sat_velocity &&
                    freq_frequency_hz_local(sd, freq) > 0.0) {
                    const double rr_obs_sat =
                        -freq_rover_doppler_local(sd, freq) *
                        (constants::SPEED_OF_LIGHT / freq_frequency_hz_local(sd, freq));
                    const double rr_pred_sat = predicted_range_rate_mps(sd);
                    const Vector3d e_sat = (sd.sat_pos - rover_pos).normalized();

                    rtk_measurement::MeasurementRow doppler_row;
                    doppler_row.residual =
                        (rr_obs_sat - rr_pred_sat) - (rr_obs_ref - rr_pred_ref);
                    // d(h)/d(v_rx) = e_ref - e_sat (h = rr_sat - rr_ref,
                    // d(rr)/d(v_rx) = -e). Position/ambiguity/iono columns
                    // stay zero: baseline_coefficients untouched.
                    for (int axis = 0; axis < 3; ++axis) {
                        doppler_row.state_coefficients.push_back(
                            {VELOCITY_STATE_INDEX + axis,
                             e_ref_doppler(axis) - e_sat(axis)});
                    }
                    doppler_row.reference_variance = ref_doppler_variance;
                    double sat_doppler_variance = doppler_variance(sd.elevation);
                    const double sat_doppler_model_variance = sat_doppler_variance;
                    if (adaptive_noise_active) {
                        sat_doppler_variance = adaptive_noise_tracker_.adaptedVariance(
                            adaptive_key, rtk_measurement::MeasurementKind::DOPPLER,
                            sat_doppler_model_variance, adaptiveNoiseConfig(sd.satellite));
                    }
                    doppler_row.satellite_variance = sat_doppler_variance;
                    doppler_row.adaptive_key = adaptive_key;
                    doppler_row.adaptive_model_variance = sat_doppler_model_variance;
                    doppler_block.rows.push_back(std::move(doppler_row));
                }
            }
            blocks.push_back(std::move(phase_block));
            blocks.push_back(std::move(code_block));
            if (!doppler_block.rows.empty()) {
                blocks.push_back(std::move(doppler_block));
            }
        };

        append_frequency_blocks(0);
        append_frequency_blocks(1);
        if (rtk_config_.enable_l5) {
            append_frequency_blocks(2);  // Phase 18 Step 4: L5 phase + code DD measurement rows
        }
    }

    return blocks;
}

bool RTKProcessor::updateFilter(const std::map<SatelliteId, SatelliteData>& sat_data) {
    if (sat_data.size() < 4) return false;

    const auto blocks = buildMeasurementBlocks(sat_data);
    const auto measurement_diagnostics = rtk_measurement::summarizeMeasurementBlocks(blocks);
    auto measurement_system = rtk_measurement::assembleMeasurementSystem(
        blocks, filter_state_.state.size());
    // navi.776 B2: Doppler rows live in the m/s domain -- give them their
    // own outlier threshold instead of the metre-domain scalar.
    debug_telemetry_.float_update_doppler_observation_count =
        measurement_diagnostics.doppler_observation_count;
    if (measurement_diagnostics.doppler_observation_count > 0) {
        measurement_system.row_outlier_thresholds.assign(
            static_cast<size_t>(measurement_system.residuals.size()), 0.0);
        double doppler_residual_sum_sq = 0.0;
        int row_index = 0;
        for (const auto& block : blocks) {
            for (const auto& row : block.rows) {
                if (block.kind == rtk_measurement::MeasurementKind::DOPPLER) {
                    measurement_system.row_outlier_thresholds[static_cast<size_t>(row_index)] =
                        rtk_config_.doppler_row_outlier_threshold_mps;
                    doppler_residual_sum_sq += row.residual * row.residual;
                }
                ++row_index;
            }
        }
        debug_telemetry_.doppler_row_residual_rms_mps = std::sqrt(
            doppler_residual_sum_sq /
            static_cast<double>(measurement_diagnostics.doppler_observation_count));
    }
    std::vector<bool> force_active;
    if (rtk_config_.enable_velocity_states &&
        filter_state_.state.size() >= VELOCITY_STATE_INDEX + VELOCITY_STATES) {
        force_active.assign(filter_state_.state.size(), false);
        for (int i = 0; i < VELOCITY_STATES; ++i) {
            force_active[VELOCITY_STATE_INDEX + i] = true;
        }
    }
    // navi.776 C: position before the measurement update, for the Kalman
    // position-correction statistic driving the offline time-offset search.
    const Vector3d position_before_update = filter_state_.state.head<3>();
    // Evaluated on the prior state, before the update moves the baseline.
    const bool adaptive_noise_active = adaptiveNoiseActiveThisEpoch();
    const bool nis_gates_disabled =
        !(std::isfinite(rtk_config_.max_update_nis_per_observation) &&
          rtk_config_.max_update_nis_per_observation > 0.0) &&
        !(std::isfinite(rtk_config_.max_fixed_update_nis_per_observation) &&
          rtk_config_.max_fixed_update_nis_per_observation > 0.0);

    const double update_outlier_threshold =
        rtk_config_.outlier_threshold > 0.0
            ? rtk_config_.outlier_threshold
            : 30.0;
    bool adaptive_tracker_updated = false;
    auto feed_adaptive_tracker =
        [&](const std::vector<rtk_measurement::MeasurementBlock>& update_blocks,
            const rtk_measurement::MeasurementSystem& update_system,
            const rtk_update::FilterUpdateResult& result) {
            if (!adaptive_noise_active || !result.ok ||
                result.rejected_by_innovation_gate ||
                result.row_innovations.size() == 0 ||
                result.row_hph_diagonal.size() !=
                    result.row_innovations.size()) {
                return;
            }
            const double tow = current_epoch_time_.tow;
            int row_index = 0;
            for (const auto& block : update_blocks) {
                for (const auto& row : block.rows) {
                    if (row_index >= result.row_innovations.size()) break;
                    const bool suppressed =
                        update_system.design_matrix.row(row_index).isZero(0.0);
                    if (row.adaptive_key >= 0 && !suppressed) {
                        // phase-only mode never learns code rows: the code
                        // variance stays at its model value, so feeding it
                        // would build stale code memory that a later
                        // phase-only epoch cannot use. The innovation index
                        // still advances for every row regardless.
                        const bool code_row =
                            block.kind == rtk_measurement::MeasurementKind::CODE;
                        if (!(rtk_config_.adaptive_noise_phase_only && code_row)) {
                            // adaptive_key = freq*MAXSAT + satelliteSlot(sat);
                            // recover the satellite so per-system alpha
                            // applies during the update too.
                            const SatelliteId row_satellite =
                                satelliteFromSlot(row.adaptive_key % MAXSAT);
                            adaptive_noise_tracker_.update(
                                row.adaptive_key,
                                block.kind,
                                result.row_innovations(row_index),
                                result.row_hph_diagonal(row_index),
                                row.reference_variance,
                                row.adaptive_model_variance,
                                tow,
                                adaptiveNoiseConfig(row_satellite));
                        }
                    }
                    ++row_index;
                }
            }
            adaptive_tracker_updated = true;
        };

    rtk_update::FilterUpdateResult update_result;
    const bool use_sequential_doppler_update =
        rtk_config_.sequential_doppler_update &&
        measurement_diagnostics.doppler_observation_count >= 3 &&
        nis_gates_disabled;
    if (use_sequential_doppler_update) {
        std::vector<rtk_measurement::MeasurementBlock> position_blocks;
        std::vector<rtk_measurement::MeasurementBlock> doppler_blocks;
        position_blocks.reserve(blocks.size());
        doppler_blocks.reserve(blocks.size());
        for (const auto& block : blocks) {
            if (block.kind == rtk_measurement::MeasurementKind::DOPPLER) {
                doppler_blocks.push_back(block);
            } else {
                position_blocks.push_back(block);
            }
        }
        auto position_system = rtk_measurement::assembleMeasurementSystem(
            position_blocks, filter_state_.state.size());
        auto doppler_system = rtk_measurement::assembleMeasurementSystem(
            doppler_blocks, filter_state_.state.size());
        doppler_system.row_outlier_thresholds.assign(
            static_cast<size_t>(doppler_system.residuals.size()),
            rtk_config_.doppler_row_outlier_threshold_mps);

        const VectorXd state_before_sequential = filter_state_.state;
        const MatrixXd covariance_before_sequential = filter_state_.covariance;
        auto position_result = rtk_update::applyMeasurementUpdate(
            filter_state_.state,
            filter_state_.covariance,
            position_system,
            update_outlier_threshold,
            6,
            0.0,
            force_active,
            adaptive_noise_active,
            rtk_config_.reuse_kalman_factorization_for_nis,
            rtk_config_.student_t_front_end);
        rtk_update::FilterUpdateResult doppler_result;
        if (position_result.ok) {
            const VectorXd state_correction =
                filter_state_.state - state_before_sequential;
            doppler_system.residuals -=
                doppler_system.design_matrix * state_correction;
            doppler_result = rtk_update::applyMeasurementUpdate(
                filter_state_.state,
                filter_state_.covariance,
                doppler_system,
                update_outlier_threshold,
                3,
                0.0,
                force_active,
                adaptive_noise_active,
                rtk_config_.reuse_kalman_factorization_for_nis,
                rtk_config_.student_t_front_end);
        }
        update_result.ok = position_result.ok && doppler_result.ok;
        if (!update_result.ok) {
            filter_state_.state = state_before_sequential;
            filter_state_.covariance = covariance_before_sequential;
        }
        update_result.rejected_by_innovation_gate =
            position_result.rejected_by_innovation_gate ||
            doppler_result.rejected_by_innovation_gate;
        update_result.observation_count =
            position_result.observation_count + doppler_result.observation_count;
        update_result.innovation_observation_count =
            position_result.innovation_observation_count +
            doppler_result.innovation_observation_count;
        update_result.suppressed_outliers =
            position_result.suppressed_outliers +
            doppler_result.suppressed_outliers;
        update_result.prefit_residual_rms_m =
            measurement_diagnostics.residual_rms_m;
        update_result.prefit_residual_max_abs_m =
            measurement_diagnostics.residual_max_abs_m;
        const double post_sum_sq =
            position_system.residuals.squaredNorm() +
            doppler_system.residuals.squaredNorm();
        if (update_result.observation_count > 0) {
            update_result.post_suppression_residual_rms_m =
                std::sqrt(post_sum_sq /
                          static_cast<double>(update_result.observation_count));
        }
        update_result.post_suppression_residual_max_abs_m =
            std::max(position_system.residuals.cwiseAbs().maxCoeff(),
                     doppler_system.residuals.cwiseAbs().maxCoeff());
        update_result.normalized_innovation_squared =
            position_result.normalized_innovation_squared +
            doppler_result.normalized_innovation_squared;
        if (update_result.innovation_observation_count > 0) {
            update_result.normalized_innovation_squared_per_observation =
                update_result.normalized_innovation_squared /
                static_cast<double>(update_result.innovation_observation_count);
        }
        if (update_result.ok) {
            feed_adaptive_tracker(
                position_blocks, position_system, position_result);
            feed_adaptive_tracker(
                doppler_blocks, doppler_system, doppler_result);
        }
    } else {
        update_result = rtk_update::applyMeasurementUpdate(
            filter_state_.state,
            filter_state_.covariance,
            measurement_system,
            update_outlier_threshold,
            6,
            rtk_config_.max_update_nis_per_observation,
            force_active,
            adaptive_noise_active,
            rtk_config_.reuse_kalman_factorization_for_nis &&
                nis_gates_disabled,
            rtk_config_.student_t_front_end);
        feed_adaptive_tracker(blocks, measurement_system, update_result);
    }

    if (update_result.ok) {
        const double correction_norm_m =
            (filter_state_.state.head<3>() - position_before_update).norm();
        debug_telemetry_.position_correction_norm_m = correction_norm_m;
        if (ins_time_update_applied_last_epoch_) {
            ++position_correction_count_;
            position_correction_sum_sq_m2_ += correction_norm_m * correction_norm_m;
        }
    }

    // navi.776 A2: feed this epoch's innovations back into the adaptive
    // noise tracker (consumed by the NEXT epoch's buildMeasurementBlocks —
    // the paper's R_{k+1} recursion). Never learn from rejected or failed
    // updates, and skip rows zeroed by suppressOutlierRows.
    if (adaptive_tracker_updated) {
        const double tow = current_epoch_time_.tow;
        adaptive_noise_tracker_.pruneStale(tow, rtk_config_.adaptive_noise_reset_gap_s);
        debug_telemetry_.adaptive_noise_tracked_entries =
            static_cast<int>(adaptive_noise_tracker_.size());
        debug_telemetry_.adaptive_noise_mean_phase_scale =
            adaptive_noise_tracker_.meanVarianceScale(
                rtk_measurement::MeasurementKind::PHASE);
        debug_telemetry_.adaptive_noise_mean_code_scale =
            adaptive_noise_tracker_.meanVarianceScale(
                rtk_measurement::MeasurementKind::CODE);
    }
    current_update_diagnostics_.observation_count = update_result.observation_count;
    current_update_diagnostics_.phase_observation_count =
        measurement_diagnostics.phase_observation_count;
    current_update_diagnostics_.code_observation_count =
        measurement_diagnostics.code_observation_count;
    current_update_diagnostics_.suppressed_outliers += update_result.suppressed_outliers;
    current_update_diagnostics_.prefit_residual_rms_m = update_result.prefit_residual_rms_m;
    current_update_diagnostics_.prefit_residual_max_m =
        std::max(current_update_diagnostics_.prefit_residual_max_m,
                 update_result.prefit_residual_max_abs_m);
    current_update_diagnostics_.post_suppression_residual_rms_m =
        update_result.post_suppression_residual_rms_m;
    current_update_diagnostics_.post_suppression_residual_max_m =
        std::max(current_update_diagnostics_.post_suppression_residual_max_m,
                 update_result.post_suppression_residual_max_abs_m);
    current_update_diagnostics_.normalized_innovation_squared =
        update_result.normalized_innovation_squared;
    current_update_diagnostics_.normalized_innovation_squared_per_observation =
        update_result.normalized_innovation_squared_per_observation;
    current_update_diagnostics_.rejected_by_innovation_gate =
        update_result.rejected_by_innovation_gate;

    // WP8 canyon forensics: mirror into the public per-epoch debug
    // telemetry (see EpochDebugTelemetry's float_update_* fields).
    debug_telemetry_.float_update_observation_count = update_result.observation_count;
    debug_telemetry_.float_update_prefit_residual_rms_m = update_result.prefit_residual_rms_m;
    debug_telemetry_.float_update_post_suppression_residual_rms_m =
        update_result.post_suppression_residual_rms_m;
    debug_telemetry_.float_update_nis_per_observation =
        update_result.normalized_innovation_squared_per_observation;
    debug_telemetry_.float_update_suppressed_outliers = update_result.suppressed_outliers;
    debug_telemetry_.float_update_student_t_downweighted_rows =
        update_result.student_t.downweighted_rows;
    if (update_result.student_t.applied) {
        debug_telemetry_.float_update_student_t_minimum_weight =
            update_result.student_t.minimum_weight;
        debug_telemetry_.float_update_student_t_mean_weight =
            update_result.student_t.mean_weight;
    }
    debug_telemetry_.float_position_covariance_trace_m2 =
        filter_state_.covariance.rows() >= 3 && filter_state_.covariance.cols() >= 3
            ? filter_state_.covariance(0, 0) + filter_state_.covariance(1, 1) +
                  filter_state_.covariance(2, 2)
            : std::numeric_limits<double>::quiet_NaN();

    return update_result.ok;
}

// ============================================================
// Resolve ambiguities: SD->DD transform + LAMBDA
// ============================================================

}  // namespace libgnss
