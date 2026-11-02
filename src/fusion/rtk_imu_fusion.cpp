#include <libgnss++/fusion/rtk_imu_fusion.hpp>

#include <algorithm>
#include <cmath>
#include <utility>

#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/fusion/attitude.hpp>

namespace libgnss {

namespace {

Eigen::Matrix3d ecefToEnuRotation(double lat, double lon) {
    Eigen::Matrix3d rotation;
    rotation.col(0) = libgnss::ecef2enu(Eigen::Vector3d::UnitX(), lat, lon);
    rotation.col(1) = libgnss::ecef2enu(Eigen::Vector3d::UnitY(), lat, lon);
    rotation.col(2) = libgnss::ecef2enu(Eigen::Vector3d::UnitZ(), lat, lon);
    return rotation;
}

// --independent-output-filter: the loose ESKF behind --out. It never feeds
// the RTK filter or the closed-loop INS, so the RTK stream is unchanged.
LooseCouplingProcessor::Config makeOutputFilterConfig(
    const RtkImuFusionProcessor::Config& config) {
    libgnss::LooseCouplingProcessor::Config output_filter_config = config.fusion_config;
    output_filter_config.consider_attitude_and_biases_before_heading_latch = true;
    return output_filter_config;
}

ImuPreintegrationConfig makePreintegrationConfig(const RtkImuFusionProcessor::Config& config) {
    libgnss::ImuPreintegrationConfig tc_preintegration_config;
    tc_preintegration_config.process_noise = config.fusion_config.process_noise;
    tc_preintegration_config.max_sample_gap_s = config.tc_ins_max_sample_gap_s;
    return tc_preintegration_config;
}

TightCouplingProcessor::Config makeClosedLoopConfig(const RtkImuFusionProcessor::Config& config) {
    libgnss::TightCouplingProcessor::Config tc_closed_loop_config;
    tc_closed_loop_config.process_noise = config.fusion_config.process_noise;
    tc_closed_loop_config.lever_arm_body = config.fusion_config.lever_arm_body;
    tc_closed_loop_config.max_sample_gap_s = config.tc_ins_max_sample_gap_s;
    tc_closed_loop_config.zupt_enable = config.fusion_config.zupt_enable;
    tc_closed_loop_config.zupt_sigma_mps = config.fusion_config.zupt_sigma_mps;
    tc_closed_loop_config.zupt_max_accel_std = config.fusion_config.zupt_max_accel_std;
    tc_closed_loop_config.zupt_max_gyro_std = config.fusion_config.zupt_max_gyro_std;
    tc_closed_loop_config.zupt_max_gyro_median = config.fusion_config.zupt_max_gyro_median;
    tc_closed_loop_config.nhc_enable = config.fusion_config.nhc_enable;
    tc_closed_loop_config.nhc_sigma_lateral_mps = config.fusion_config.nhc_sigma_lateral_mps;
    tc_closed_loop_config.nhc_sigma_vertical_mps = config.fusion_config.nhc_sigma_vertical_mps;
    tc_closed_loop_config.velocity_state_output_enable = config.tc_velocity_states;
    return tc_closed_loop_config;
}

}  // namespace

RtkImuFusionProcessor::RtkImuFusionProcessor(const Config& config,
                                             RTKProcessor::RTKConfig rtk_config,
                                             const Eigen::Vector3d& base_position_ecef)
    : config_(config),
      fusion_processor_(config.fusion_config),
      output_filter_(makeOutputFilterConfig(config)),
      tc_preintegrator_(makePreintegrationConfig(config)),
      tc_closed_loop_processor_(makeClosedLoopConfig(config)) {
    // Phase 1 GNSS/IMU coupling: no-op unless --rtk-ins-prior was passed
    // (RTKConfig::use_external_position_prior defaults false, so this line
    // is a no-op for every existing caller/preset).
    rtk_config.use_external_position_prior = config_.rtk_ins_prior;
    rtk_config.use_external_position_time_update =
        config_.tc_ins_time_update || config_.tc_closed_loop;
    rtk_config.enable_velocity_states = config_.tc_velocity_states;
    rtk_config.enable_tdcp_diagnostics = config_.tc_tdcp_diagnostics;
    rtk_config.enable_cp_pr_fixed_gate = config_.tc_closed_loop;
    rtk_processor_.setRTKConfig(rtk_config);
    rtk_processor_.setBasePosition(base_position_ecef);
}

void RtkImuFusionProcessor::processImuSample(const ImuSample& sample) {
    fusion_processor_.processImuSample(sample);
    if (config_.independent_output_filter) output_filter_.processImuSample(sample);
    if (config_.tc_ins_time_update && tc_preintegrator_.initialized()) {
        const auto status = tc_preintegrator_.integrate(sample);
        if (status != libgnss::PreintegrationStatus::ACCEPTED &&
            status != libgnss::PreintegrationStatus::INTERVAL_INVALID) {
            // Count the root failure once. Later samples in the same
            // latched-invalid interval report INTERVAL_INVALID.
            ++counters_.tc_invalid_interval_count;
        }
    }
    if (config_.tc_closed_loop) {
        tc_closed_loop_processor_.processImuSample(sample);
    }
}

RtkImuFusionProcessor::EpochResult RtkImuFusionProcessor::processEpoch(
    const ObservationData& rover_obs,
    const ObservationData& aligned_base_obs,
    const NavigationData& nav_data) {
    // M1: convert this RTK-to-RTK preintegrated antenna displacement and
    // its process noise from the anchor ENU frame to ECEF. The anchor's
    // nominal body position is -R*lever, so predicted p+R*lever is the
    // antenna displacement itself, with no loose-ESKF absolute position
    // entering the update.
    if (config_.tc_ins_time_update && tc_preintegrator_.valid()) {
        const auto tc_result = tc_preintegrator_.result();
        const Eigen::Matrix3d body_to_enu =
            tc_result.predicted_state.attitude_body_to_enu.toRotationMatrix();
        const Eigen::Vector3d antenna_delta_enu =
            tc_result.predicted_state.position_enu +
            body_to_enu * config_.fusion_config.lever_arm_body;
        Eigen::Matrix<double, 3, libgnss::fusion_index::SIZE> antenna_jacobian =
            Eigen::Matrix<double, 3, libgnss::fusion_index::SIZE>::Zero();
        antenna_jacobian.block<3, 3>(0, libgnss::fusion_index::POSITION) =
            Eigen::Matrix3d::Identity();
        antenna_jacobian.block<3, 3>(0, libgnss::fusion_index::ATTITUDE) =
            -body_to_enu * libgnss::attitude::skew(config_.fusion_config.lever_arm_body);
        const Eigen::Matrix3d antenna_process_noise_enu =
            antenna_jacobian * tc_result.process_noise * antenna_jacobian.transpose();
        const Eigen::Matrix3d ecef_to_enu =
            ecefToEnuRotation(tc_anchor_lat_rad_, tc_anchor_lon_rad_);
        const Eigen::Matrix3d enu_to_ecef = ecef_to_enu.transpose();
        const Eigen::Vector3d antenna_delta_ecef = enu_to_ecef * antenna_delta_enu;
        const Eigen::Matrix3d antenna_process_noise_ecef =
            enu_to_ecef * antenna_process_noise_enu * ecef_to_enu;
        if (antenna_delta_ecef.allFinite() && antenna_process_noise_ecef.allFinite()) {
            rtk_processor_.setExternalPositionTimeUpdate(
                antenna_delta_ecef, antenna_process_noise_ecef);
            ++counters_.tc_update_supplied_count;
        } else {
            ++counters_.tc_invalid_interval_count;
            tc_preintegrator_.clear();
        }
    }
    if (config_.tc_closed_loop) {
        const auto update = tc_closed_loop_processor_.prepareTimeUpdate();
        if (update.valid) {
            if (config_.tc_velocity_states) {
                rtk_processor_.setExternalPositionVelocityTimeUpdate(
                    update.antenna_delta_ecef, update.antenna_velocity_ecef,
                    update.position_velocity_process_noise_ecef,
                    update.velocity_covariance_ecef);
            } else {
                rtk_processor_.setExternalPositionTimeUpdate(
                    update.antenna_delta_ecef, update.process_noise_ecef);
            }
        }
    }

    // Phase 1 GNSS/IMU coupling (docs/design.md): after IMU mechanization
    // has been propagated up to this epoch's time above, hand the ESKF's
    // predicted antenna ECEF position to RTKProcessor as this epoch's
    // KINEMATIC prior, BEFORE processRTKEpoch() runs its internal
    // resetPositionToSPP() time update. Gated on the fusion filter being
    // initialized, ECEF-anchored, and heading-converged (isHeadingConverged()
    // requires both a latch AND that recent GNSS-velocity innovations still
    // corroborate it -- see LooseCouplingProcessor doc comment) so an
    // unaligned or drifting ESKF never gets to seed RTK's position states.
    // predictedAntennaPositionEcef() itself already returns false pre-
    // origin-anchor, so isOriginSet() is a redundant but cheap belt-and-
    // suspenders check. RTKProcessor::resetPositionToSPP() consumes
    // (clears) any prior on every call, so simply not calling
    // setExternalPositionPrior() this epoch (rather than an explicit
    // clearExternalPositionPrior()) is sufficient to fall back to the
    // legacy SPP reseed.
    // Phase 1b redesign (docs/imu_fusion.md, docs/decisions.md): v1 fed
    // the prior into every eligible epoch and regressed at all tested
    // inflations, because the ESKF's predicted mean carries a persistent
    // multi-meter bias that trips RTKProcessor's magnitude-based
    // jump/reacquisition gates on otherwise-healthy epochs RTK would
    // have solved fine on its own. Two additional gates (see gnss_fuse's
    // FuseOptions rtk_ins_prior_spp_gate/rtk_ins_prior_max_nis doc
    // comment for the full rationale) now restrict injection to epochs
    // that are BOTH plausibly SPP-degraded (the bridges/tunnels/urban-
    // canyon case the prior exists for) AND backed by a currently
    // healthy ESKF (recent velocity innovations still corroborate the
    // predicted state) -- --rtk-ins-prior-always disables both to
    // reproduce v1 exactly for comparison.
    if (config_.rtk_ins_prior && fusion_processor_.isInitialized() &&
        fusion_processor_.isOriginSet() && fusion_processor_.isHeadingConverged()) {
        ++counters_.ins_prior_eligible_epochs;
        const int current_nsat = static_cast<int>(rover_obs.getNumSatellites());
        const double current_nis = fusion_processor_.getVelocityNisEma();
        if (config_.record_ins_prior_signals) {
            counters_.ins_prior_eligible_nsats.push_back(current_nsat);
            counters_.ins_prior_eligible_nis.push_back(current_nis);
        }
        const bool spp_degraded = !config_.rtk_ins_prior_spp_gate ||
            current_nsat < config_.rtk_ins_prior_min_sats;
        const bool eskf_healthy = config_.rtk_ins_prior_max_nis <= 0.0 ||
            current_nis <= config_.rtk_ins_prior_max_nis;
        if (spp_degraded && eskf_healthy) {
            Eigen::Vector3d predicted_pos;
            Eigen::Matrix3d predicted_cov;
            if (fusion_processor_.predictedAntennaPositionEcef(predicted_pos, predicted_cov) &&
                predicted_pos.allFinite() && predicted_cov.allFinite()) {
                rtk_processor_.setExternalPositionPrior(
                    predicted_pos, predicted_cov * config_.rtk_ins_prior_inflation);
                ++counters_.ins_prior_fire_count;
            }
        }
    }

    auto pos_solution = rtk_processor_.processRTKEpoch(
        rover_obs, aligned_base_obs, nav_data);
    if (config_.tc_closed_loop) {
        const auto& telemetry = rtk_processor_.getLastDebugTelemetry();
        counters_.tc_cp_pr_evaluated_count += telemetry.cp_pr_gate_evaluated ? 1 : 0;
        counters_.tc_cp_pr_rejected_count += telemetry.cp_pr_gate_rejected ? 1 : 0;
        counters_.tc_cp_pr_escalated_count += telemetry.cp_pr_gate_escalated ? 1 : 0;
        counters_.tc_ddpr_anchor_count += telemetry.ddpr_anchor_valid ? 1 : 0;
    }
    if (config_.tc_tdcp_diagnostics) {
        const auto& telemetry = rtk_processor_.getLastDebugTelemetry();
        counters_.tc_tdcp_candidate_count += telemetry.tdcp_candidate_count;
        counters_.tc_tdcp_residual_count += telemetry.tdcp_residual_count;
        counters_.tc_tdcp_missing_previous_count += telemetry.tdcp_rejected_missing_previous;
        counters_.tc_tdcp_gap_count += telemetry.tdcp_rejected_gap;
        counters_.tc_tdcp_loss_of_lock_count += telemetry.tdcp_rejected_loss_of_lock;
        counters_.tc_tdcp_invalid_count += telemetry.tdcp_rejected_invalid;
        if (telemetry.tdcp_residual_count > 0 &&
            std::isfinite(telemetry.tdcp_residual_rms_m)) {
            counters_.tc_tdcp_residual_sum_squares += telemetry.tdcp_residual_count *
                telemetry.tdcp_residual_rms_m * telemetry.tdcp_residual_rms_m;
            counters_.tc_tdcp_residual_max_abs_m = std::max(
                counters_.tc_tdcp_residual_max_abs_m, telemetry.tdcp_residual_max_abs_m);
        }
    }
    const bool rtk_valid = pos_solution.isValid();
    if (rtk_valid) {
        auto fusion_solution = pos_solution;
        if (config_.independent_output_filter) {
            // The epoch SPP Doppler velocity, not the RTK velocity state
            // (docs/imu_fusion.md, "Independent output filter").
            libgnss::PositionSolution output_input = fusion_solution;
            const auto& spp = rtk_processor_.currentSpp();
            output_input.has_velocity = spp.has_velocity && spp.velocity_ecef.allFinite() &&
                                        spp.velocity_covariance.allFinite();
            if (output_input.has_velocity) {
                output_input.velocity_ecef = spp.velocity_ecef;
                output_input.velocity_covariance = spp.velocity_covariance;
            }
            output_filter_.processGnssSolution(output_input);
        }
        fusion_processor_.processGnssSolution(fusion_solution);
        pos_solution.velocity_ecef = fusion_solution.velocity_ecef;
        pos_solution.velocity_covariance = fusion_solution.velocity_covariance;
        pos_solution.has_velocity = fusion_solution.has_velocity;
        if (config_.tightly_coupled_dd_imu && fusion_processor_.isOriginSet()) {
            const auto propagated_solution = fusion_processor_.toPositionSolution();
            const auto rotation = fusion_processor_.ecefToLocalEnuRotation();
            auto dd_rows = rtk_processor_.formTightlyCoupledObservations(
                rover_obs, aligned_base_obs, nav_data,
                propagated_solution.position_ecef, rotation,
                fusion_processor_.state().nominal.attitude_body_to_enu);
            if (config_.tightly_coupled_dd_code_only) {
                for (auto& row : dd_rows) row.carrier_variance_m2 = 0.0;
            }
            if (!dd_rows.empty()) {
                const auto dd_result = fusion_processor_.processTightlyCoupledDD(
                    dd_rows, &pos_solution);
                ++counters_.tight_dd_epochs;
                counters_.tight_dd_rows += static_cast<int>(dd_rows.size());
                if (std::isfinite(dd_result.update.nis_per_observation)) {
                    ++counters_.tight_dd_nis_samples;
                    counters_.tight_dd_nis_sum += dd_result.update.nis_per_observation;
                    counters_.tight_dd_nis_max = std::max(
                        counters_.tight_dd_nis_max,
                        dd_result.update.nis_per_observation);
                }
                if (dd_result.update.ok) {
                    ++counters_.tight_dd_accepted;
                }
                if (dd_result.update.rejected_by_innovation_gate) {
                    ++counters_.tight_dd_rejected;
                }
                if (dd_result.update.carrier_fallback_used) {
                    ++counters_.tight_dd_carrier_fallbacks;
                }
                if (dd_result.partial_ar.fixed) {
                    ++counters_.tight_dd_partial_ar_epochs;
                    counters_.tight_dd_fixed_ambiguities += dd_result.partial_ar.fixed_count;
                }
                if (dd_result.sse_partial_ar.available) {
                    ++counters_.tight_dd_sse_par_available_epochs;
                }
                if (dd_result.sse_partial_ar.subsets_evaluated > 0) {
                    ++counters_.tight_dd_sse_par_eligible_epochs;
                }
                counters_.tight_dd_sse_par_subsets_evaluated +=
                    dd_result.sse_partial_ar.subsets_evaluated;
                counters_.tight_dd_sse_par_ratio_passed_subsets +=
                    dd_result.sse_partial_ar.ratio_passed_subsets;
                counters_.tight_dd_sse_par_separation_rejected_subsets +=
                    dd_result.sse_partial_ar
                        .separation_rejected_subsets;
                if (dd_result.sse_partial_ar.passed) {
                    ++counters_.tight_dd_sse_par_passed_epochs;
                    counters_.tight_dd_sse_par_fixed_ambiguities +=
                        dd_result.sse_partial_ar.fixed_count;
                }
                if (dd_result.reset_action !=
                    libgnss::dd_imu_bridge::SoftResetAction::REJECTED) {
                    ++counters_.tight_dd_soft_resets;
                }
            }
        }
        if (config_.tc_ins_time_update) {
            bool anchored = false;
            Eigen::Vector3d float_position_ecef;
            Eigen::Matrix3d float_position_covariance_ecef;
            if (pos_solution.has_velocity && pos_solution.velocity_ecef.allFinite() &&
                fusion_processor_.isInitialized() && fusion_processor_.isOriginSet() &&
                fusion_processor_.isHeadingConverged() &&
                rtk_processor_.getFloatPosteriorPosition(
                    float_position_ecef, float_position_covariance_ecef)) {
                double anchor_height_m = 0.0;
                libgnss::ecef2geodetic(float_position_ecef, tc_anchor_lat_rad_,
                                       tc_anchor_lon_rad_, anchor_height_m);
                const Eigen::Matrix3d ecef_to_enu =
                    ecefToEnuRotation(tc_anchor_lat_rad_, tc_anchor_lon_rad_);
                libgnss::NominalState anchor_state = fusion_processor_.state().nominal;
                anchor_state.time = rover_obs.time;
                anchor_state.velocity_enu = ecef_to_enu * pos_solution.velocity_ecef;
                anchor_state.position_enu =
                    -anchor_state.attitude_body_to_enu.toRotationMatrix() *
                    config_.fusion_config.lever_arm_body;
                anchored = tc_preintegrator_.reset(anchor_state) ==
                    libgnss::PreintegrationStatus::ACCEPTED;
                if (anchored) {
                    ++counters_.tc_anchor_count;
                } else {
                    ++counters_.tc_invalid_interval_count;
                }
            }
            if (!anchored) {
                tc_preintegrator_.clear();
            }
        }
        if (config_.tc_closed_loop) {
            bool anchored = false;
            Eigen::Vector3d anchor_position_ecef;
            Eigen::Matrix3d anchor_position_covariance_ecef;
            const bool needs_bootstrap = !tc_closed_loop_processor_.initialized();
            const bool bootstrap_ready = !needs_bootstrap ||
                (fusion_processor_.isInitialized() && fusion_processor_.isOriginSet() &&
                 fusion_processor_.isHeadingConverged());
            bool have_anchor = false;
            const auto& telemetry = rtk_processor_.getLastDebugTelemetry();
            if (telemetry.ddpr_anchor_valid) {
                libgnss::GNSSTime anchor_time;
                have_anchor = rtk_processor_.getLastDdPrAnchor(
                    anchor_position_ecef, anchor_position_covariance_ecef, anchor_time);
            }
            if (!have_anchor) {
                have_anchor = rtk_processor_.getFloatPosteriorPosition(
                    anchor_position_ecef, anchor_position_covariance_ecef);
            }
            if (bootstrap_ready && have_anchor && pos_solution.has_velocity &&
                pos_solution.velocity_ecef.allFinite() &&
                pos_solution.velocity_covariance.allFinite()) {
                const libgnss::FusionState* bootstrap =
                    needs_bootstrap ? &fusion_processor_.state() : nullptr;
                anchored = tc_closed_loop_processor_.reanchor(
                    anchor_position_ecef, anchor_position_covariance_ecef,
                    pos_solution.velocity_ecef, pos_solution.velocity_covariance,
                    rover_obs.time, bootstrap);
            }
            if (!anchored) tc_closed_loop_processor_.invalidateInterval();
        }
    } else {
        if (config_.tc_ins_time_update) tc_preintegrator_.clear();
        if (config_.tc_closed_loop) tc_closed_loop_processor_.invalidateInterval();
    }

    EpochResult result;
    result.rtk_valid = rtk_valid;
    {
        const auto& output_source =
            config_.independent_output_filter ? output_filter_ : fusion_processor_;
        if (output_source.isOriginSet()) {
            result.has_output = true;
            result.output = output_source.toAntennaPositionSolution();
            result.attitude_time = output_source.state().nominal.time;
            result.attitude_body_to_enu = output_source.state().nominal.attitude_body_to_enu;
        }
    }
    result.rtk = std::move(pos_solution);
    return result;
}

}  // namespace libgnss
