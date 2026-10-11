#include <libgnss++/algorithms/rtk.hpp>
#include <libgnss++/algorithms/rtk_carrier_prior.hpp>
#include <libgnss++/algorithms/rtk_ins_time_update.hpp>

#include <algorithm>
#include <cmath>
#include <map>
#include <vector>

#include "rtk_internal.hpp"

namespace libgnss {

using namespace rtk_internal;

namespace {

// Constants of the carrier-propagated prior that are not part of the pure
// helper module (see rtk_carrier_prior.hpp for the TDCP / monitor ones).

/// Largest epoch gap bridged by a TDCP difference. TDCP needs the same
/// carrier arc in both epochs; 1.5 s covers 1 Hz data with one missed epoch.
constexpr double kMaxTdcpGapS = 1.5;
/// Unmodelled carrier-rate error of the TDCP velocity [m/s]: multipath /
/// diffraction of a blocked-sky signal moves the carrier phase by well under a
/// tenth of a cycle over several seconds => ~1 cm/s. Enters the position
/// process noise as (sigma_v * dt)^2, so the prior forgets at the rate its own
/// error budget allows and does not become infinitely stiff.
constexpr double kCarrierRateSigmaMps = 0.01;
/// Constant-velocity bridge over TDCP outages (fewer than 5 continuous
/// satellites). A road vehicle accelerates/brakes below ~0.3 g, so the velocity
/// is good to 3 m/s^2 * tau; 0.5 s bounds the extrapolation error to ~0.4 m.
constexpr double kBridgeMaxS = 0.5;
constexpr double kBridgeAccelSigmaMps2 = 3.0;
/// A posterior whose prefit code/carrier RMS is below this is consistent
/// enough to start a chain from: a third of the filter's 3 m row outlier gate
/// (open-sky epochs on UrbanNav/PPC run at 0.3-0.5 m).
constexpr double kHealthyPrefitRmsM = 1.0;
/// A measurement update that moves the position farther than the 3 m outlier
/// gate from the carrier prior contradicts the prior; release and do not
/// restart until a healthy epoch has been seen.
constexpr double kContradictionShiftM = 3.0;

double absoluteTimeS(const GNSSTime& t) {
    return static_cast<double>(t.week) * 604800.0 + t.tow;
}

}  // namespace

void RTKProcessor::captureCarrierPriorSnapshot() {
    auto& rt = carrier_prior_;
    // The state is only a posterior when the previous epoch's update ran
    // (finishCarrierPriorEpoch set `posterior_valid`).
    rt.snapshot_valid = false;
    if (!rt.posterior_valid) return;
    rt.posterior_valid = false;
    if (filter_state_.state.size() < BASE_STATES ||
        filter_state_.covariance.rows() != filter_state_.state.size() ||
        !filter_state_.state.head<3>().allFinite()) {
        return;
    }
    rt.snapshot_baseline = filter_state_.state.head<3>();
    rt.snapshot_rows = filter_state_.covariance.topRows<3>();
    rt.snapshot_valid = rt.snapshot_rows.allFinite();
}

void RTKProcessor::applyCarrierPrior(
    const ObservationData& rover_obs,
    const std::map<SatelliteId, SatelliteData>& sat_data) {
    auto& rt = carrier_prior_;
    auto& stats = carrier_prior_stats_;
    ++stats.epochs_evaluated;
    rt.engaged_this_epoch = false;
    const bool snapshot_valid = rt.snapshot_valid;
    rt.snapshot_valid = false;
    const bool previous_healthy = rt.previous_healthy;
    rt.previous_healthy = false;
    const double now_s = absoluteTimeS(rover_obs.time);

    // Whatever happens below, this epoch becomes the TDCP reference.
    const auto remember = [&]() {
        rt.previous_sat_data = sat_data;
        rt.previous_time = rover_obs.time;
        rt.has_previous = true;
    };
    const auto unavailable = [&]() {
        rtk_carrier_prior::MonitorInput input;
        input.time_s = now_s;
        input.previous_epoch_healthy = previous_healthy;
        rt.monitor.update(input);
        stats.engaged_last_epoch = false;
        remember();
    };

    if (!isDynamicPositionMode(rtk_config_) || isMovingBasePositionMode(rtk_config_) ||
        rtk_config_.use_external_position_prior ||
        rtk_config_.use_external_position_time_update || !snapshot_valid ||
        !rt.has_previous) {
        unavailable();
        return;
    }
    const double dt = rover_obs.time - rt.previous_time;
    if (!std::isfinite(dt) || dt <= 0.0 || dt > kMaxTdcpGapS) {
        rt.has_velocity = false;
        unavailable();
        return;
    }

    const Vector3d x0 = base_position_ + rt.snapshot_baseline;
    const auto geometry = [&](const SatelliteData& s) {
        const double rover_range =
            geodist_range(s.sat_pos, x0) + tropModel(x0, s.elevation);
        const double base_range = geodist_range(s.sat_pos_base, base_position_) +
                                  tropModel(base_position_, s.base_elevation);
        return rover_range - base_range;
    };

    // --- TDCP displacement over (previous epoch, this epoch] ---
    rtk_carrier_prior::TdcpConfig tdcp_config;
    std::vector<rtk_carrier_prior::TdcpRow> rows;
    for (const auto& [sat, sd] : sat_data) {
        const auto previous_it = rt.previous_sat_data.find(sat);
        if (previous_it == rt.previous_sat_data.end()) continue;
        const SatelliteData& pd = previous_it->second;
        if (!sd.has_l1 || !pd.has_l1 || sd.l1_wavelength <= 0.0 ||
            pd.l1_wavelength <= 0.0 || sd.rover_l1_phase == 0.0 ||
            sd.base_l1_phase == 0.0 || pd.rover_l1_phase == 0.0 ||
            pd.base_l1_phase == 0.0 || sd.elevation <= 0.0) {
            continue;
        }
        // A cycle-slip flag on either receiver ends the carrier arc.
        if ((sd.l1_lli & 0x01) != 0 || (sd.rover_l1_lli & 0x01) != 0 ||
            (sd.base_l1_lli & 0x01) != 0) {
            continue;
        }
        const double phase_now =
            (sd.rover_l1_phase - sd.base_l1_phase) * sd.l1_wavelength;
        const double phase_previous =
            (pd.rover_l1_phase - pd.base_l1_phase) * pd.l1_wavelength;
        rtk_carrier_prior::TdcpRow row;
        row.delta_m = (phase_now - phase_previous) - (geometry(sd) - geometry(pd));
        row.los = (sd.sat_pos - x0).normalized();
        row.sigma_m = tdcp_config.sigma_zenith_m /
                      std::max(std::sin(sd.elevation), tdcp_config.min_sin_elevation);
        rows.push_back(row);
    }
    const auto tdcp = rtk_carrier_prior::solveTdcpDisplacement(rows, tdcp_config);

    Vector3d delta = Vector3d::Zero();
    Matrix3d process_noise = Matrix3d::Zero();
    bool prior_available = false;
    bool bridged = false;
    if (tdcp.ok) {
        ++stats.tdcp_valid_epochs;
        delta = tdcp.displacement_ecef;
        process_noise = tdcp.covariance +
                        (kCarrierRateSigmaMps * dt) * (kCarrierRateSigmaMps * dt) *
                            Matrix3d::Identity();
        rt.velocity_ecef = delta / dt;
        rt.velocity_time = rover_obs.time;
        rt.has_velocity = true;
        prior_available = true;
    } else {
        ++stats.tdcp_invalid_epochs;
        if (rt.has_velocity) {
            const double age = rover_obs.time - rt.velocity_time;
            if (std::isfinite(age) && age >= 0.0 && age <= kBridgeMaxS) {
                delta = rt.velocity_ecef * dt;
                const double sigma = kBridgeAccelSigmaMps2 * dt * std::max(age, dt);
                process_noise = (sigma * sigma +
                                 (kCarrierRateSigmaMps * dt) * (kCarrierRateSigmaMps * dt)) *
                                Matrix3d::Identity();
                prior_available = true;
                bridged = true;
                ++stats.bridged_epochs;
            }
        }
    }

    // --- code consistency against the carrier-propagated position ---
    int inconsistent_rows = 0;
    if (prior_available) {
        const Vector3d x_prior = x0 + delta;
        const double threshold =
            rtk_config_.outlier_threshold > 0.0 ? rtk_config_.outlier_threshold : 3.0;
        struct SystemResiduals {
            std::vector<double> residual_m;
            std::vector<double> elevation_rad;
        };
        std::map<GNSSSystem, SystemResiduals> by_system;
        for (const auto& [sat, sd] : sat_data) {
            // GLONASS FDMA inter-channel code biases between receivers reach
            // metres and are not part of this test.
            if (sat.system == GNSSSystem::GLONASS) continue;
            if (!sd.has_l1 || sd.rover_l1_code == 0.0 || sd.base_l1_code == 0.0) continue;
            const double rover_range =
                geodist_range(sd.sat_pos, x_prior) + tropModel(x_prior, sd.elevation);
            const double base_range = geodist_range(sd.sat_pos_base, base_position_) +
                                      tropModel(base_position_, sd.base_elevation);
            auto& entry = by_system[sat.system];
            entry.residual_m.push_back(
                (sd.rover_l1_code - sd.base_l1_code) - (rover_range - base_range));
            entry.elevation_rad.push_back(sd.elevation);
        }
        const rtk_carrier_prior::MonitorConfig monitor_config;
        for (const auto& [system, entry] : by_system) {
            // The receiver-clock (and inter-system bias) term is removed by the
            // per-system median; three satellites are the minimum for one.
            if (entry.residual_m.size() < 3) continue;
            inconsistent_rows += rtk_carrier_prior::evaluateCodeConsistency(
                                     entry.residual_m, entry.elevation_rad, threshold,
                                     monitor_config.min_elevation_rad)
                                     .inconsistent_rows;
        }
    }

    rtk_carrier_prior::MonitorInput input;
    input.time_s = now_s;
    input.prior_available = prior_available;
    input.prior_bridged = bridged;
    input.previous_epoch_healthy = previous_healthy;
    input.inconsistent_rows = inconsistent_rows;
    const bool was_engaged = rt.monitor.engaged();
    const bool engage = rt.monitor.update(input);
    remember();
    if (!engage) {
        stats.engaged_last_epoch = false;
        return;
    }

    // Restore the previous posterior (position and its cross-covariances),
    // then advance it with the TDCP displacement. The ambiguity states are
    // untouched, so every satellite keeps its carrier-defined ambiguity and a
    // newly (re)initialised one is pulled to the prior by the phase rows.
    const int n = static_cast<int>(filter_state_.state.size());
    if (rt.snapshot_rows.rows() != 3 || rt.snapshot_rows.cols() != n) {
        rt.monitor.reportContradiction();
        stats.engaged_last_epoch = false;
        return;
    }
    VectorXd state = filter_state_.state;
    MatrixXd covariance = filter_state_.covariance;
    state.head<3>() = rt.snapshot_baseline;
    covariance.topRows<3>() = rt.snapshot_rows;
    covariance.leftCols<3>() = rt.snapshot_rows.transpose();
    for (int j = 3; j < n; ++j) {
        // Ambiguities reset since the snapshot carry no position correlation.
        if (state(j) == 0.0) {
            covariance.block(0, j, 3, 1).setZero();
            covariance.block(j, 0, 1, 3).setZero();
        }
    }
    if (!rtk_ins_time_update::apply(state, covariance, delta, process_noise, 0.0)) {
        rt.monitor.reportContradiction();
        stats.engaged_last_epoch = false;
        return;
    }
    filter_state_.state = std::move(state);
    filter_state_.covariance = std::move(covariance);
    rt.prior_position_baseline = filter_state_.state.head<3>();
    rt.engaged_this_epoch = true;
    ++stats.engaged_epochs;
    if (!was_engaged) ++stats.engagements;
    stats.engaged_last_epoch = true;
}

bool RTKProcessor::initializeAmbiguityFromCarrierPrior(
    const SatelliteId& sat, int freq,
    const std::map<SatelliteId, SatelliteData>& sat_data) {
    if (usesEstimatedIono(rtk_config_) || freq < 0 || freq > 2) return false;
    auto& indices = (freq == 0)   ? filter_state_.n1_indices
                    : (freq == 1) ? filter_state_.n2_indices
                                  : filter_state_.n5_indices;
    const auto has_signal = [freq](const SatelliteData& sd) {
        return freq == 0 ? sd.has_l1 : (freq == 1 ? sd.has_l2 : sd.has_l5);
    };
    const auto wavelength = [freq](const SatelliteData& sd) {
        return freq == 0 ? sd.l1_wavelength : (freq == 1 ? sd.l2_wavelength : sd.l5_wavelength);
    };
    const auto sd_phase_cycles = [freq](const SatelliteData& sd) {
        if (freq == 0) return sd.rover_l1_phase - sd.base_l1_phase;
        if (freq == 1) return sd.rover_l2_phase - sd.base_l2_phase;
        return sd.rover_l5_phase - sd.base_l5_phase;
    };
    const auto sat_it = sat_data.find(sat);
    if (sat_it == sat_data.end() || !has_signal(sat_it->second) ||
        wavelength(sat_it->second) <= 0.0) {
        return false;
    }
    const SatelliteData& sd = sat_it->second;
    const int n = static_cast<int>(filter_state_.state.size());
    const int idx = IB(sat, freq);

    // Anchor: the tightest existing ambiguity of the same system and frequency
    // (sigma <= 10 cycles; a fresh code seed carries 30 cycles and is no anchor).
    constexpr double kMaxAnchorVariance = 100.0;
    int anchor_idx = -1;
    const SatelliteData* anchor = nullptr;
    double anchor_variance = kMaxAnchorVariance;
    for (const auto& [asat, aidx] : indices) {
        if (asat == sat || asat.system != sat.system || aidx == idx) continue;
        if (aidx < 0 || aidx >= n || filter_state_.state(aidx) == 0.0) continue;
        const auto a_it = sat_data.find(asat);
        if (a_it == sat_data.end() || !has_signal(a_it->second) ||
            wavelength(a_it->second) <= 0.0) {
            continue;
        }
        const double variance = filter_state_.covariance(aidx, aidx);
        if (!(variance > 0.0) || variance >= anchor_variance) continue;
        anchor_variance = variance;
        anchor_idx = aidx;
        anchor = &a_it->second;
    }
    if (anchor == nullptr) return false;

    const Vector3d rover_pos = base_position_ + filter_state_.state.head<3>();
    const auto geometry = [&](const SatelliteData& s) {
        return geodist_range(s.sat_pos, rover_pos) + tropModel(rover_pos, s.elevation) -
               (geodist_range(s.sat_pos_base, base_position_) +
                tropModel(base_position_, s.base_elevation));
    };
    const double lambda_i = wavelength(sd);
    const double lambda_j = wavelength(*anchor);
    const double a = lambda_j / lambda_i;
    const double new_state =
        (lambda_j * filter_state_.state(anchor_idx) -
         (sd_phase_cycles(*anchor) * lambda_j - sd_phase_cycles(sd) * lambda_i) +
         (geometry(*anchor) - geometry(sd))) /
        lambda_i;
    if (!std::isfinite(new_state) || new_state == 0.0) return false;

    // s_i = a * s_j + f(x): covariance of the augmented state.
    const Vector3d u_i = (sd.sat_pos - rover_pos).normalized();
    const Vector3d u_j = (anchor->sat_pos - rover_pos).normalized();
    const Vector3d g = (u_i - u_j) / lambda_i;
    VectorXd row = a * filter_state_.covariance.row(anchor_idx).transpose() +
                   filter_state_.covariance.topRows(3).transpose() * g;
    // Carrier-phase noise plus rover-base atmospheric differential (~2 cm).
    constexpr double kAtmosphereFloorCycles2 = 0.01;
    const double variance = a * row(anchor_idx) + g.dot(row.head<3>()) + kAtmosphereFloorCycles2;
    if (!row.allFinite() || !(variance > 0.0)) return false;
    row(idx) = variance;
    filter_state_.state(idx) = new_state;
    filter_state_.covariance.row(idx) = row.transpose();
    filter_state_.covariance.col(idx) = row;
    indices[sat] = idx;
    ++carrier_prior_stats_.phase_initialised_ambiguities;
    return true;
}

void RTKProcessor::finishCarrierPriorEpoch(bool filter_ok) {
    auto& rt = carrier_prior_;
    rt.posterior_valid = filter_ok;
    rt.previous_healthy =
        filter_ok && std::isfinite(current_update_diagnostics_.prefit_residual_rms_m) &&
        current_update_diagnostics_.prefit_residual_rms_m <= kHealthyPrefitRmsM;
    if (rt.engaged_this_epoch && filter_ok &&
        (filter_state_.state.head<3>() - rt.prior_position_baseline).norm() >
            kContradictionShiftM) {
        rt.monitor.reportContradiction();
        ++carrier_prior_stats_.contradictions;
        rt.previous_healthy = false;
    }
}

}  // namespace libgnss
