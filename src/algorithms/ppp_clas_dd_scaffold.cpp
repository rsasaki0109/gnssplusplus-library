// DdFilterScaffold: DD float-filter snapshot/predict/hold scaffolding.
// Split out of the former monolithic ppp_clas_dd.cpp.

#include <libgnss++/algorithms/ppp_clas_dd.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <set>
#include <sstream>
#include <tuple>
#include <utility>

#include <libgnss++/algorithms/lambda.hpp>
#include <libgnss++/algorithms/ppp_osr.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>

#include "ppp_clas_dd_internal.hpp"

namespace libgnss::ppp_clas_dd {
using namespace internal;

void DdFilterScaffold::ensureSnapshotStorage(
    const StateLayout& layout,
    bool preserve_existing) {
    const int nx = layout.nx();
    if (!has_snapshot_ || snapshot_.state.size() != nx) {
        snapshot_.state = VectorXd::Zero(nx);
        snapshot_.covariance = MatrixXd::Zero(nx, nx);
    } else if (!preserve_existing) {
        snapshot_.state.setZero();
        snapshot_.covariance.setZero();
    }
    snapshot_.layout = layout;
    has_snapshot_ = true;
}

void DdFilterScaffold::resetStateElement(
    int index,
    double value,
    double variance) {
    if (index < 0 || index >= snapshot_.state.size()) {
        return;
    }
    snapshot_.state(index) = value;
    snapshot_.covariance.row(index).setZero();
    snapshot_.covariance.col(index).setZero();
    snapshot_.covariance(index, index) = variance;
}

void DdFilterScaffold::initializeFromNativeFloat(
    const GNSSTime& time,
    const StateLayout& layout,
    const ppp_shared::PPPState& native_state,
    const PositionSolution& native_float_solution,
    const ppp_shared::PPPConfig& config,
    const std::vector<OSRCorrection>& osr_corrections,
    const std::map<std::string, std::string>& epoch_atmos) {
    ensureSnapshotStorage(layout, false);
    adaptive_position_process_noise_ecef_.setZero();
    snapshot_.time = time;
    snapshot_.seeded_from_native_float = true;
    snapshot_.native_total_states = native_state.total_states;
    snapshot_.has_atmosphere_network_id =
        readAtmosphereNetworkId(epoch_atmos, snapshot_.atmosphere_network_id);

    const Vector3d seed_position =
        native_float_solution.position_ecef.allFinite()
            ? native_float_solution.position_ecef
            : native_state.state.segment(native_state.pos_index, 3);
    const double initial_position_variance =
        config.clas_mrtklib_float_parity
            ? kClaslibInitialPositionVarianceM2
            : kDdInitialPositionVarianceM2;
    for (int axis = 0; axis < 3 && axis < layout.np(); ++axis) {
        resetStateElement(axis, seed_position(axis), initial_position_variance);
    }

    if (layout.options.dynamics && native_state.state.size() >= native_state.pos_index + 9) {
        for (int axis = 3; axis < 9; ++axis) {
            const double variance = axis < 6 ? 1.0 : 1.0;
            resetStateElement(axis, native_state.state(native_state.pos_index + axis), variance);
        }
    }

    const int trop_index = layout.troposphereIndex();
    if (trop_index >= 0 && config.estimate_troposphere) {
        resetStateElement(
            trop_index,
            kClaslibInitialZwdM,
            kClaslibStdTropM * kClaslibStdTropM);
    }

    updateObservedStateBookkeeping(
        ObservationData(time), osr_corrections, config, 0.0);
}

void DdFilterScaffold::predictState(
    const ObservationData& obs,
    const std::vector<OSRCorrection>& osr_corrections,
    const ppp_shared::PPPConfig& config,
    double dt) {
    if (!has_snapshot_) {
        return;
    }
    const double abs_dt = std::abs(dt);
    const StateLayout& layout = snapshot_.layout;
    if (layout.options.dynamics && layout.np() >= 9) {
        for (int axis = 0; axis < 3; ++axis) {
            snapshot_.state(axis) +=
                snapshot_.state(axis + 3) * dt +
                0.5 * snapshot_.state(axis + 6) * dt * dt;
            snapshot_.state(axis + 3) += snapshot_.state(axis + 6) * dt;
        }
    }

    // CLASLIB udpos_ppp() models estimated ionosphere as a first-order
    // Gauss-Markov process.  With stats-tconstiono=10 s it scales every iono
    // state by exp(-dt/10), its P cross-covariances once, and the iono-iono
    // block twice before udion() adds adaptive process noise.
    if (config.clas_mrtklib_float_parity &&
        layout.options.ionosphere_mode == IonosphereMode::EstimateAdaptive &&
        abs_dt > 0.0) {
        const double ki = std::exp(-abs_dt / kClaslibIonoTimeConstantS);
        for (int satno = 1; satno <= layout.options.max_satellites; ++satno) {
            const int index = layout.ionosphereIndex(satno);
            if (index < 0 || index >= snapshot_.state.size()) {
                continue;
            }
            snapshot_.state(index) *= ki;
            snapshot_.covariance.row(index) *= ki;
            snapshot_.covariance.col(index) *= ki;
        }
    }

    // With pos2-prnadpt=on CLASLIB propagates the adaptive Q matrix formed
    // from the previous measurement-update correction.  It starts at zero;
    // stats-prnposith/v are only used when adaptive PVA noise is disabled.
    if (config.clas_mrtklib_float_parity &&
        adaptive_position_process_noise_ecef_.allFinite()) {
        snapshot_.covariance.block<3, 3>(0, 0) +=
            adaptive_position_process_noise_ecef_ * abs_dt;
    } else {
        const double position_q =
            std::max(0.0, config.process_noise_position) * abs_dt;
        for (int axis = 0; axis < 3; ++axis) {
            snapshot_.covariance(axis, axis) += position_q;
        }
    }

    const int trop_index = layout.troposphereIndex();
    if (trop_index >= 0 && snapshot_.state(trop_index) == 0.0) {
        resetStateElement(
            trop_index,
            kClaslibInitialZwdM,
            kClaslibStdTropM * kClaslibStdTropM);
    } else if (trop_index >= 0) {
        snapshot_.covariance(trop_index, trop_index) +=
            kClaslibPrnTropM * kClaslibPrnTropM * abs_dt;
    }

    updateObservedStateBookkeeping(obs, osr_corrections, config, abs_dt);
}

void DdFilterScaffold::updateObservedStateBookkeeping(
    const ObservationData& obs,
    const std::vector<OSRCorrection>& osr_corrections,
    const ppp_shared::PPPConfig& config,
    double dt) {
    if (!has_snapshot_) {
        return;
    }

    const StateLayout& layout = snapshot_.layout;
    std::map<int, std::vector<std::pair<int, double>>> ambiguity_biases_m;

    for (auto& [_, outage] : ionosphere_outage_by_satno_) {
        ++outage;
    }
    for (auto& [_, outage] : ambiguity_outage_by_satno_freq_) {
        ++outage;
    }

    for (const auto& osr : osr_corrections) {
        if (!osr.valid) {
            continue;
        }
        const int satno = claslibSatelliteNumber(osr.satellite);
        if (!layout.validSatelliteNumber(satno)) {
            continue;
        }
        ionosphere_outage_by_satno_[satno] = 0;

        const int iono_index = layout.ionosphereIndex(satno);
        if (config.estimate_ionosphere && iono_index >= 0) {
            if (snapshot_.state(iono_index) == 0.0 ||
                snapshot_.covariance(iono_index, iono_index) <= 0.0) {
                resetStateElement(
                    iono_index,
                    1e-6,
                    kClaslibStdIonoM * kClaslibStdIonoM);
                ionosphere_process_noise_by_satno_[satno] = 0.0;
            } else if (dt > 0.0) {
                const double elevation = osr.elevation;
                double qi = ionosphere_process_noise_by_satno_[satno];
                if (layout.options.ionosphere_mode == IonosphereMode::EstimateAdaptive) {
                    if (qi == 0.0) {
                        const double fact = std::cos(elevation);
                        qi = kClaslibPrnIonoM * kClaslibPrnIonoM * fact * fact;
                    } else {
                        qi = std::clamp(
                            qi,
                            kClaslibPrnIonoM * kClaslibPrnIonoM,
                            kClaslibPrnIonoMaxM * kClaslibPrnIonoMaxM);
                    }
                } else {
                    const double fact = std::cos(elevation);
                    qi = kClaslibPrnIonoM * kClaslibPrnIonoM * fact * fact;
                }
                ionosphere_process_noise_by_satno_[satno] = qi;
                snapshot_.covariance(iono_index, iono_index) += qi * dt;
            }
        }

        for (int f = 0; f < osr.num_frequencies && f < layout.nf(); ++f) {
            if (osr.wavelengths[f] <= 0.0) {
                continue;
            }
            const int frequency_index =
                claslibFrequencyIndex(osr.satellite, osr.signals[f], f);
            if (frequency_index < 0 || frequency_index >= layout.nf()) {
                continue;
            }
            const Observation* raw = findOsrFrequencyObservation(obs, osr, f);
            if (raw == nullptr || !raw->valid || !raw->has_carrier_phase ||
                !raw->has_pseudorange || !std::isfinite(raw->carrier_phase) ||
                !std::isfinite(raw->pseudorange)) {
                continue;
            }
            const auto key = std::make_pair(satno, frequency_index);
            ambiguity_outage_by_satno_freq_[key] = 0;
            if (ambiguity_lock_by_satno_freq_.find(key) ==
                ambiguity_lock_by_satno_freq_.end()) {
                ambiguity_lock_by_satno_freq_[key] = -kClaslibMinLockCount;
            }

            const int ambiguity_index = layout.ambiguityIndex(satno, frequency_index);
            if (ambiguity_index < 0) {
                continue;
            }
            if (raw->loss_of_lock) {
                resetStateElement(ambiguity_index, 0.0, 0.0);
                ambiguity_lock_by_satno_freq_[key] = -kClaslibMinLockCount;
            } else if (snapshot_.covariance(ambiguity_index, ambiguity_index) > 0.0 &&
                       dt > 0.0) {
                snapshot_.covariance(ambiguity_index, ambiguity_index) +=
                    kClaslibPrnBiasCycles * kClaslibPrnBiasCycles * dt;
            }

            const double bias_m =
                raw->carrier_phase * osr.wavelengths[f] - raw->pseudorange;
            if (std::isfinite(bias_m)) {
                ambiguity_biases_m[frequency_index].push_back(
                    {ambiguity_index, bias_m});
            }
        }
    }

    for (const auto& [satno, outage] : ionosphere_outage_by_satno_) {
        if (outage <= kClaslibMaxOutage) {
            continue;
        }
        const int iono_index = layout.ionosphereIndex(satno);
        if (iono_index >= 0) {
            resetStateElement(iono_index, 0.0, 0.0);
        }
    }
    for (const auto& [key, outage] : ambiguity_outage_by_satno_freq_) {
        if (outage <= kClaslibMaxOutage) {
            continue;
        }
        const int ambiguity_index = layout.ambiguityIndex(key.first, key.second);
        if (ambiguity_index >= 0) {
            resetStateElement(ambiguity_index, 0.0, 0.0);
            ambiguity_lock_by_satno_freq_[key] = -kClaslibMinLockCount;
        }
    }

    for (const auto& [frequency, biases] : ambiguity_biases_m) {
        double common_bias_m = 0.0;
        int common_count = 0;
        for (const auto& [ambiguity_index, bias_m] : biases) {
            if (snapshot_.covariance(ambiguity_index, ambiguity_index) > 0.0 &&
                snapshot_.state(ambiguity_index) != 0.0) {
                double wavelength = 0.0;
                for (const auto& osr : osr_corrections) {
                    const int satno = claslibSatelliteNumber(osr.satellite);
                    if (layout.ambiguityIndex(satno, frequency) == ambiguity_index) {
                        wavelength =
                            osrWavelengthForClaslibFrequency(osr, frequency);
                        break;
                    }
                }
                if (wavelength > 0.0) {
                    common_bias_m += bias_m - snapshot_.state(ambiguity_index) * wavelength;
                    ++common_count;
                }
            }
        }
        if (common_count > 0) {
            common_bias_m /= static_cast<double>(common_count);
        }
        for (const auto& [ambiguity_index, bias_m] : biases) {
            if (snapshot_.state(ambiguity_index) != 0.0 &&
                snapshot_.covariance(ambiguity_index, ambiguity_index) > 0.0) {
                continue;
            }
            double wavelength = 0.0;
            for (const auto& osr : osr_corrections) {
                const int satno = claslibSatelliteNumber(osr.satellite);
                if (layout.ambiguityIndex(satno, frequency) == ambiguity_index) {
                    wavelength =
                        osrWavelengthForClaslibFrequency(osr, frequency);
                    break;
                }
            }
            if (wavelength <= 0.0) {
                continue;
            }
            double cycles = (bias_m - common_bias_m) / wavelength;
            if (cycles == 0.0) {
                cycles = 1e-6;
            }
            resetStateElement(
                ambiguity_index,
                cycles,
                kClaslibStdBiasCycles * kClaslibStdBiasCycles);
            for (const auto& osr : osr_corrections) {
                const int satno = claslibSatelliteNumber(osr.satellite);
                if (layout.ambiguityIndex(satno, frequency) == ambiguity_index) {
                    ambiguity_lock_by_satno_freq_[{satno, frequency}] =
                        -kClaslibMinLockCount;
                    break;
                }
            }
        }
    }
}

PositionSolution DdFilterScaffold::fallbackSolution(
    const PositionSolution& native_float_solution,
    const std::string& reason,
    int observed_satellites) {
    ++total_fallback_epochs_;
    last_diagnostics_.updated = false;
    last_diagnostics_.fallback_reason = reason;
    PositionSolution solution = native_float_solution;
    solution.status = SolutionStatus::PPP_FLOAT;
    solution.ratio = 0.0;
    solution.num_fixed_ambiguities = 0;
    solution.num_satellites = observed_satellites;
    solution.iterations = 0;
    return solution;
}

PositionSolution DdFilterScaffold::publishSolution(
    const PositionSolution& native_float_solution,
    const rtk_measurement::MeasurementDiagnostics& measurement_diagnostics,
    int observed_satellites,
    const StateSnapshot& source_snapshot,
    bool fixed) const {
    PositionSolution solution = native_float_solution;
    solution.position_ecef = source_snapshot.state.segment(0, 3);
    if (source_snapshot.covariance.rows() >= 3 &&
        source_snapshot.covariance.cols() >= 3) {
        solution.position_covariance =
            source_snapshot.covariance.block<3, 3>(0, 0);
    }
    solution.status = fixed ? SolutionStatus::PPP_FIXED : SolutionStatus::PPP_FLOAT;
    solution.ratio = last_diagnostics_.lambda_ratio;
    solution.num_fixed_ambiguities =
        fixed ? last_diagnostics_.lambda_ambiguities : 0;
    solution.num_satellites = observed_satellites;
    solution.iterations = 1;
    solution.residual_rms = measurement_diagnostics.residual_rms_m;
    solution.rtk_update_observations = measurement_diagnostics.observation_count;
    solution.rtk_update_phase_observations =
        measurement_diagnostics.phase_observation_count;
    solution.rtk_update_code_observations =
        measurement_diagnostics.code_observation_count;
    solution.rtk_update_suppressed_outliers =
        last_diagnostics_.filter_update.suppressed_outliers;
    solution.rtk_update_prefit_residual_rms_m =
        last_diagnostics_.filter_update.prefit_residual_rms_m;
    solution.rtk_update_prefit_residual_max_m =
        last_diagnostics_.filter_update.prefit_residual_max_abs_m;
    solution.rtk_update_post_suppression_residual_rms_m =
        last_diagnostics_.filter_update.post_suppression_residual_rms_m;
    solution.rtk_update_post_suppression_residual_max_m =
        last_diagnostics_.filter_update.post_suppression_residual_max_abs_m;
    solution.rtk_update_normalized_innovation_squared =
        last_diagnostics_.filter_update.normalized_innovation_squared;
    solution.rtk_update_normalized_innovation_squared_per_observation =
        last_diagnostics_.filter_update.normalized_innovation_squared_per_observation;
    solution.rtk_update_rejected_by_innovation_gate =
        last_diagnostics_.filter_update.rejected_by_innovation_gate ? 1 : 0;
    return solution;
}

int DdFilterScaffold::countReferenceChanges(
    const DdMeasurementBuildResult& build) const {
    int changes = 0;
    for (const auto& group : build.reference_groups_detail) {
        if (!group.is_phase) {
            continue;
        }
        const auto key = std::make_tuple(
            group.system_group, group.frequency_index, group.is_phase);
        const auto it = reference_by_group_.find(key);
        if (it != reference_by_group_.end() &&
            !(it->second == group.reference_satellite)) {
            ++changes;
        }
    }
    return changes;
}

void DdFilterScaffold::rememberReferenceGroups(
    const DdMeasurementBuildResult& build) {
    for (const auto& group : build.reference_groups_detail) {
        if (!group.is_phase) {
            continue;
        }
        reference_by_group_[std::make_tuple(
            group.system_group, group.frequency_index, group.is_phase)] =
            group.reference_satellite;
    }
}

void DdFilterScaffold::updateAmbiguityLockCounts(
    const std::vector<DdRow>& rows) {
    std::set<std::pair<int, int>> valid_phase_states;
    for (const auto& row : rows) {
        if (!row.is_phase) {
            continue;
        }
        const int ref_satno = claslibSatelliteNumber(row.reference_satellite);
        const int target_satno = claslibSatelliteNumber(row.target_satellite);
        if (snapshot_.layout.validSatelliteNumber(ref_satno)) {
            valid_phase_states.insert({ref_satno, row.frequency_index});
        }
        if (snapshot_.layout.validSatelliteNumber(target_satno)) {
            valid_phase_states.insert({target_satno, row.frequency_index});
        }
    }
    for (const auto& key : valid_phase_states) {
        auto& lock = ambiguity_lock_by_satno_freq_[key];
        if (lock < 0) {
            ++lock;
        } else {
            ++lock;
        }
    }
}

double DdFilterScaffold::computePdop(
    const std::vector<DdRow>& rows,
    double min_elevation_rad) const {
    std::vector<Vector3d> geometry_rows;
    auto collect = [&](bool phase_rows) {
        geometry_rows.clear();
        std::set<std::tuple<int, int, int>> seen;
        for (const auto& row : rows) {
            if (row.is_phase != phase_rows ||
                row.frequency_index != 0 ||
                row.reference_elevation_rad < min_elevation_rad ||
                row.target_elevation_rad < min_elevation_rad) {
                continue;
            }
            const int ref_satno = claslibSatelliteNumber(row.reference_satellite);
            const int target_satno = claslibSatelliteNumber(row.target_satellite);
            const auto key = std::make_tuple(row.system_group, ref_satno, target_satno);
            if (!seen.insert(key).second) {
                continue;
            }
            geometry_rows.push_back(row.position_coefficients);
        }
    };

    collect(false);
    if (geometry_rows.size() < 3) {
        collect(true);
    }
    if (geometry_rows.size() < 3) {
        return 1000.0;
    }

    MatrixXd h = MatrixXd::Zero(static_cast<int>(geometry_rows.size()), 3);
    for (int row = 0; row < static_cast<int>(geometry_rows.size()); ++row) {
        h.row(row) = geometry_rows[static_cast<size_t>(row)].transpose();
    }
    const Matrix3d normal = h.transpose() * h;
    Eigen::LDLT<Matrix3d> ldlt(normal);
    if (ldlt.info() != Eigen::Success) {
        return 1000.0;
    }
    const Matrix3d q = ldlt.solve(Matrix3d::Identity());
    if (!q.allFinite()) {
        return 1000.0;
    }
    return std::sqrt(std::max(0.0, q.trace()));
}

bool DdFilterScaffold::applyHoldAmbiguity(const std::vector<DdRow>& rows) {
    last_diagnostics_.hold_applied = false;
    last_diagnostics_.hold_rows = 0;
    last_diagnostics_.hold_reject_reason.clear();
    if (validated_hold_differences_.empty() ||
        validated_hold_ambiguities_.size() !=
            static_cast<int>(validated_hold_differences_.size())) {
        last_diagnostics_.hold_reject_reason = "hold_no_validated_ambiguities";
        return false;
    }

    const int nx = snapshot_.layout.nx();
    std::vector<int> selected;
    selected.reserve(validated_hold_differences_.size());
    for (int i = 0; i < static_cast<int>(validated_hold_differences_.size()); ++i) {
        const auto& difference = validated_hold_differences_[static_cast<size_t>(i)];
        const auto row_it = std::find_if(
            rows.begin(),
            rows.end(),
            [&](const DdRow& row) {
                if (!row.is_phase ||
                    row.reference_elevation_rad < kClaslibHoldElevationMaskRad ||
                    row.target_elevation_rad < kClaslibHoldElevationMaskRad) {
                    return false;
                }
                const int ref_satno = claslibSatelliteNumber(row.reference_satellite);
                const int target_satno = claslibSatelliteNumber(row.target_satellite);
                return snapshot_.layout.ambiguityIndex(ref_satno, row.frequency_index) ==
                           difference.reference_state_index &&
                       snapshot_.layout.ambiguityIndex(target_satno, row.frequency_index) ==
                           difference.satellite_state_index;
            });
        if (row_it != rows.end()) {
            selected.push_back(i);
        }
    }

    if (selected.empty()) {
        last_diagnostics_.hold_reject_reason = "hold_no_rows";
        return false;
    }

    rtk_measurement::MeasurementSystem system;
    system.design_matrix = MatrixXd::Zero(static_cast<int>(selected.size()), nx);
    system.residuals = VectorXd::Zero(static_cast<int>(selected.size()));
    system.covariance = MatrixXd::Zero(
        static_cast<int>(selected.size()), static_cast<int>(selected.size()));

    for (int row = 0; row < static_cast<int>(selected.size()); ++row) {
        const int source_index = selected[static_cast<size_t>(row)];
        const auto& difference =
            validated_hold_differences_[static_cast<size_t>(source_index)];
        const double fixed_dd = validated_hold_ambiguities_(source_index);
        const double float_dd =
            snapshot_.state(difference.reference_state_index) -
            snapshot_.state(difference.satellite_state_index);
        system.residuals(row) = fixed_dd - float_dd;
        system.design_matrix(row, difference.reference_state_index) = 1.0;
        system.design_matrix(row, difference.satellite_state_index) = -1.0;
        system.covariance(row, row) = kClaslibVarHoldAmbCycles2;
    }

    auto hold_update = rtk_update::applyMeasurementUpdate(
        snapshot_.state,
        snapshot_.covariance,
        system,
        std::numeric_limits<double>::infinity(),
        1);
    if (!hold_update.ok) {
        last_diagnostics_.hold_reject_reason = "hold_update";
        return false;
    }

    last_diagnostics_.hold_applied = true;
    last_diagnostics_.hold_rows = static_cast<int>(selected.size());
    ++total_hold_applied_epochs_;
    return true;
}

void DdFilterScaffold::appendDiagnosticsCsv(
    const GNSSTime& time,
    bool fixed) const {
    const char* path = clasDdDiagnosticsPath();
    if (path == nullptr || path[0] == '\0') {
        return;
    }
    const bool needs_header = !std::ifstream(path).good();
    std::ofstream output(path, std::ios::app);
    if (!output.is_open()) {
        return;
    }
    output.precision(17);
    if (needs_header) {
        output
            << "week,tow,updated,fixed,lambda_attempted,lambda_accepted,"
            << "lambda_ambiguities,lambda_ratio,reject_reason,postfit_rms_m,"
            << "postfit_max_m,postfit_phase_rms_m,postfit_phase_max_m,"
            << "postfit_chi_ratio,ar_pdop,hold_pdop,reference_change_groups,"
            << "consecutive_fix_count,hold_applied,hold_rows,hold_reject_reason,"
            << "row_summary,reference_summary,postfit_worst_group,"
            << "postfit_worst_frequency,postfit_worst_pair,"
            << "postfit_worst_phase_residual_m,"
            << "float_x_m,float_y_m,float_z_m,"
            << "fixed_x_m,fixed_y_m,fixed_z_m,fixed_shift_m\n";
    }
    const std::string reject_reason =
        !last_diagnostics_.fixed_reject_reason.empty()
            ? last_diagnostics_.fixed_reject_reason
            : (!last_diagnostics_.lambda_reject_reason.empty()
                   ? last_diagnostics_.lambda_reject_reason
                   : last_diagnostics_.fallback_reason);
    output << time.week << ',' << time.tow << ','
           << (last_diagnostics_.updated ? 1 : 0) << ','
           << (fixed ? 1 : 0) << ','
           << (last_diagnostics_.lambda_attempted ? 1 : 0) << ','
           << (last_diagnostics_.lambda_accepted ? 1 : 0) << ','
           << last_diagnostics_.lambda_ambiguities << ','
           << last_diagnostics_.lambda_ratio << ','
           << reject_reason << ','
           << last_diagnostics_.fixed_postfit.residual_rms_m << ','
           << last_diagnostics_.fixed_postfit.residual_max_abs_m << ','
           << last_diagnostics_.fixed_postfit.phase_residual_rms_m << ','
           << last_diagnostics_.fixed_postfit.phase_residual_max_abs_m << ','
           << last_diagnostics_.fixed_postfit.chi_square_ratio << ','
           << last_diagnostics_.ar_pdop << ','
           << last_diagnostics_.hold_pdop << ','
           << last_diagnostics_.reference_change_groups << ','
           << last_diagnostics_.consecutive_fix_count << ','
           << (last_diagnostics_.hold_applied ? 1 : 0) << ','
           << last_diagnostics_.hold_rows << ','
           << last_diagnostics_.hold_reject_reason << ','
           << last_diagnostics_.row_summary << ','
           << last_diagnostics_.reference_summary << ','
           << last_diagnostics_.fixed_postfit.worst_phase_system_group << ','
           << last_diagnostics_.fixed_postfit.worst_phase_frequency_index << ','
           << last_diagnostics_.fixed_postfit.worst_phase_pair << ','
           << last_diagnostics_.fixed_postfit.worst_phase_residual_m << ',';
    const bool have_float_position = snapshot_.state.size() >= 3;
    const bool have_fixed_position = fixed_snapshot_.state.size() >= 3;
    Vector3d float_position =
        Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    Vector3d fixed_position =
        Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    if (have_float_position) {
        float_position = snapshot_.state.head<3>();
    }
    if (have_fixed_position) {
        fixed_position = fixed_snapshot_.state.head<3>();
    }
    output << float_position.x() << ',' << float_position.y() << ','
           << float_position.z() << ',' << fixed_position.x() << ','
           << fixed_position.y() << ',' << fixed_position.z() << ','
           << ((have_float_position && have_fixed_position)
                   ? (fixed_position - float_position).norm()
                   : std::numeric_limits<double>::quiet_NaN())
           << '\n';
}

void DdFilterScaffold::appendStateDumpCsv(
    const GNSSTime& time,
    const std::vector<OSRCorrection>& osr_corrections,
    bool fixed) const {
    const auto& path = pppEnvOverrides().clas_dd_state_dump_path;
    if (path.empty() || snapshot_.state.size() != snapshot_.layout.nx()) {
        return;
    }
    const bool needs_header = !std::ifstream(path).good();
    std::ofstream output(path, std::ios::app);
    if (!output.is_open()) {
        return;
    }
    output.precision(17);
    if (needs_header) {
        output
            << "schema,week,tow,solution_state,state_kind,state_key,state_index,"
            << "float_value,fixed_value,variance,process_noise\n";
    }
    const double nan = std::numeric_limits<double>::quiet_NaN();
    auto append = [&](const char* kind,
                      const std::string& key,
                      int index,
                      double process_noise) {
        if (index < 0 || index >= snapshot_.state.size() ||
            index >= snapshot_.covariance.rows()) {
            return;
        }
        const double fixed_value =
            fixed && index < fixed_snapshot_.state.size()
                ? fixed_snapshot_.state(index)
                : nan;
        output << "clas_dd_filter_state.v1," << time.week << ',' << time.tow
               << ',' << (fixed ? "fix" : "float") << ',' << kind << ','
               << key << ',' << index << ',' << snapshot_.state(index) << ','
               << fixed_value << ',' << snapshot_.covariance(index, index)
               << ',' << process_noise << '\n';
    };
    static constexpr std::array<const char*, 3> kAxes{{"x", "y", "z"}};
    for (int axis = 0; axis < 3; ++axis) {
        append(
            "position",
            kAxes[static_cast<size_t>(axis)],
            axis,
            adaptive_position_process_noise_ecef_(axis, axis));
    }
    std::set<int> dumped_satellites;
    for (const auto& osr : osr_corrections) {
        const int satno = claslibSatelliteNumber(osr.satellite);
        if (!dumped_satellites.insert(satno).second) {
            continue;
        }
        const int index = snapshot_.layout.ionosphereIndex(satno);
        // CLASLIB's oracle dump walks the raw epoch observations and emits an
        // already-created ionosphere state even when corrmeas() rejects the
        // current correction row.  Mirror that diagnostic surface: osr.valid
        // controls estimator admission, not whether the live state is visible.
        if (index < 0 || index >= snapshot_.state.size() ||
            snapshot_.state(index) == 0.0) {
            continue;
        }
        const auto q_it = ionosphere_process_noise_by_satno_.find(satno);
        append(
            "ionosphere",
            osr.satellite.toString(),
            index,
            q_it == ionosphere_process_noise_by_satno_.end() ? 0.0 : q_it->second);
    }
}

bool DdFilterScaffold::conditionFixedSnapshot(
    const ObservationData& obs,
    const std::vector<OSRCorrection>& osr_corrections,
    const std::vector<DdRow>& rows,
    const ppp_shared::PPPConfig& config,
    const TropMappingFunction& trop_mapping_function,
    const Vector3d& receiver_geometry_displacement,
    int reference_change_groups,
    double ar_pdop) {
    struct LambdaAttempt {
        bool attempted = false;
        bool lambda_success = false;
        bool accepted = false;
        int nb = 0;
        double ratio = 0.0;
        std::string reject_reason;
        std::vector<rtk_measurement::AmbiguityDifference> differences;
        rtk_measurement::AmbiguityTransform transform;
        MatrixXd q_amb;
        VectorXd fixed_ambiguities;
    };

    has_fixed_snapshot_ = false;
    last_diagnostics_.lambda_attempted = false;
    last_diagnostics_.lambda_accepted = false;
    last_diagnostics_.lambda_ambiguities = 0;
    last_diagnostics_.lambda_ratio = 0.0;
    last_diagnostics_.lambda_required_ratio = config.ar_ratio_threshold;
    last_diagnostics_.lambda_reject_reason.clear();
    last_diagnostics_.fixed_postfit_accepted = false;
    last_diagnostics_.fixed_reject_reason.clear();
    last_diagnostics_.ar_pdop = ar_pdop;
    last_diagnostics_.reference_change_groups = reference_change_groups;

    const StateLayout& layout = snapshot_.layout;
    const int na = layout.nr();
    if (!config.enable_ambiguity_resolution) {
        last_diagnostics_.lambda_reject_reason = "ar_disabled";
        return false;
    }
    if (snapshot_.state.size() != layout.nx() ||
        snapshot_.covariance.rows() != layout.nx() ||
        snapshot_.covariance.cols() != layout.nx() ||
        na <= 0 || na > layout.nx()) {
        last_diagnostics_.lambda_reject_reason = "invalid_snapshot";
        return false;
    }

    const auto candidates = collectDdAmbiguityCandidates(
        rows,
        layout,
        snapshot_.state,
        snapshot_.covariance,
        ambiguity_lock_by_satno_freq_);
    last_diagnostics_.lambda_ambiguities = static_cast<int>(candidates.size());
    const int min_ambiguities = std::max(2, config.min_satellites_for_ar);

    auto evaluate = [&](const std::vector<int>& indices) {
        LambdaAttempt attempt;
        attempt.nb = static_cast<int>(indices.size());
        if (attempt.nb < min_ambiguities) {
            attempt.reject_reason = "insufficient_ambiguities";
            return attempt;
        }

        attempt.differences.reserve(indices.size());
        for (const int index : indices) {
            if (index < 0 || index >= static_cast<int>(candidates.size())) {
                continue;
            }
            attempt.differences.push_back(
                candidates[static_cast<size_t>(index)].difference);
        }
        attempt.nb = static_cast<int>(attempt.differences.size());
        if (attempt.nb < min_ambiguities) {
            attempt.reject_reason = "insufficient_ambiguities";
            return attempt;
        }

        attempt.transform = rtk_measurement::buildAmbiguityTransform(
            snapshot_.state, snapshot_.covariance, na, attempt.differences);
        attempt.q_amb = 0.5 *
            (attempt.transform.ambiguity_covariance +
             attempt.transform.ambiguity_covariance.transpose());
        if (!attempt.transform.dd_float.allFinite() ||
            !attempt.q_amb.allFinite()) {
            attempt.reject_reason = "nonfinite_ambiguity_system";
            return attempt;
        }

        attempt.attempted = true;
        if (!lambdaSearch(
                attempt.transform.dd_float,
                attempt.q_amb,
                attempt.fixed_ambiguities,
                attempt.ratio)) {
            attempt.reject_reason = "lambda_search";
            return attempt;
        }
        attempt.lambda_success = true;
        if (std::isfinite(attempt.ratio)) {
            attempt.ratio = std::min(attempt.ratio, 999.9);
        } else {
            attempt.ratio = 0.0;
        }
        const double required_ratio = claslibRatioThreshold(attempt.nb);
        if (attempt.ratio < required_ratio) {
            attempt.reject_reason = "ratio";
            return attempt;
        }
        attempt.accepted = true;
        return attempt;
    };

    std::vector<int> full_indices;
    full_indices.reserve(candidates.size());
    for (int i = 0; i < static_cast<int>(candidates.size()); ++i) {
        full_indices.push_back(i);
    }

    LambdaAttempt best_attempt;
    auto remember = [&](LambdaAttempt&& attempt) {
        if (!attempt.attempted) {
            if (best_attempt.reject_reason.empty()) {
                best_attempt.reject_reason = attempt.reject_reason;
                best_attempt.nb = std::max(best_attempt.nb, attempt.nb);
            }
            return;
        }
        if (!best_attempt.attempted ||
            (attempt.accepted && !best_attempt.accepted) ||
            (attempt.accepted && best_attempt.accepted &&
             std::tie(attempt.nb, attempt.ratio) >
                 std::tie(best_attempt.nb, best_attempt.ratio)) ||
            (!attempt.accepted && !best_attempt.accepted &&
             attempt.ratio > best_attempt.ratio)) {
            best_attempt = std::move(attempt);
        }
    };

    remember(evaluate(full_indices));

    // CLASLIB PAR (ppprtk.c): after a failed full LAMBDA search, test
    // excluding each elevation-sorted satellite on all frequencies.  If no
    // trial fixes, retain the exclusion that produced the highest ratio and
    // repeat, up to armaxdelsat=5.  This differs materially from trying one
    // frequency family in isolation.
    if (!best_attempt.accepted) {
        std::map<SatelliteId, double> target_elevations;
        for (const auto& candidate : candidates) {
            auto [it, inserted] = target_elevations.emplace(
                candidate.target_satellite, candidate.target_elevation_rad);
            if (!inserted) {
                it->second = std::max(it->second, candidate.target_elevation_rad);
            }
        }
        std::vector<std::pair<SatelliteId, double>> ordered_targets(
            target_elevations.begin(), target_elevations.end());
        std::sort(
            ordered_targets.begin(), ordered_targets.end(),
            [](const auto& lhs, const auto& rhs) {
                if (lhs.second != rhs.second) {
                    return lhs.second < rhs.second;
                }
                return lhs.first < rhs.first;
            });

        std::set<SatelliteId> excluded_targets;
        constexpr int kClaslibMaxDeletedSatellites = 5;
        for (int deletion = 0;
             deletion < kClaslibMaxDeletedSatellites && !best_attempt.accepted;
             ++deletion) {
            LambdaAttempt best_trial;
            SatelliteId best_trial_satellite;
            bool have_best_trial_satellite = false;
            for (const auto& [trial_satellite, elevation] : ordered_targets) {
                (void)elevation;
                if (excluded_targets.count(trial_satellite) != 0) {
                    continue;
                }
                std::vector<int> trial_indices;
                trial_indices.reserve(candidates.size());
                for (int index = 0; index < static_cast<int>(candidates.size()); ++index) {
                    const SatelliteId target =
                        candidates[static_cast<size_t>(index)].target_satellite;
                    if (target == trial_satellite ||
                        excluded_targets.count(target) != 0) {
                        continue;
                    }
                    trial_indices.push_back(index);
                }
                auto trial = evaluate(trial_indices);
                if (trial.accepted) {
                    best_attempt = std::move(trial);
                    break;
                }
                if (trial.attempted &&
                    (!best_trial.attempted || trial.ratio > best_trial.ratio)) {
                    best_trial = std::move(trial);
                    best_trial_satellite = trial_satellite;
                    have_best_trial_satellite = true;
                }
            }
            if (best_attempt.accepted) {
                break;
            }
            if (!have_best_trial_satellite) {
                break;
            }
            excluded_targets.insert(best_trial_satellite);
            remember(std::move(best_trial));
        }
    }

    last_diagnostics_.lambda_attempted = best_attempt.attempted;
    last_diagnostics_.lambda_ambiguities = best_attempt.nb;
    last_diagnostics_.lambda_ratio = best_attempt.ratio;
    last_diagnostics_.lambda_required_ratio =
        claslibRatioThreshold(best_attempt.nb);
    last_diagnostics_.lambda_reject_reason = best_attempt.reject_reason;

    if (!best_attempt.attempted) {
        last_diagnostics_.lambda_reject_reason = "insufficient_ambiguities";
        return false;
    }
    lambda_ratio_sum_ += best_attempt.ratio;
    ++lambda_ratio_count_;
    if (!best_attempt.accepted) {
        ++total_lambda_rejected_epochs_;
        if (best_attempt.reject_reason == "ratio") {
            ++fixed_reject_ratio_epochs_;
        }
        consecutive_validated_fixes_ = 0;
        last_diagnostics_.consecutive_fix_count = 0;
        return false;
    }

    auto reject_fixed = [&](const std::string& reason) {
        last_diagnostics_.lambda_accepted = false;
        last_diagnostics_.fixed_reject_reason = reason;
        last_diagnostics_.lambda_reject_reason = reason;
        ++total_lambda_rejected_epochs_;
        consecutive_validated_fixes_ = 0;
        last_diagnostics_.consecutive_fix_count = 0;
        if (reason == "pdop") {
            ++fixed_reject_pdop_epochs_;
        } else if (reason == "min_fix") {
            ++fixed_reject_minfix_epochs_;
        } else if (reason == "ref_change") {
            ++fixed_reject_refchange_epochs_;
        } else {
            ++fixed_reject_postfit_epochs_;
        }
        validated_hold_differences_.clear();
        validated_hold_ambiguities_.resize(0);
        has_fixed_snapshot_ = false;
        return false;
    };

    if (!std::isfinite(ar_pdop) || ar_pdop > kClaslibMaxPdopAr) {
        return reject_fixed("pdop");
    }
    Eigen::LDLT<MatrixXd> ldlt(best_attempt.q_amb);
    if (ldlt.info() != Eigen::Success) {
        ++total_lambda_rejected_epochs_;
        last_diagnostics_.lambda_reject_reason = "q_amb_ldlt";
        consecutive_validated_fixes_ = 0;
        last_diagnostics_.consecutive_fix_count = 0;
        return false;
    }

    const VectorXd dd_residual =
        best_attempt.transform.dd_float - best_attempt.fixed_ambiguities;
    const VectorXd db = ldlt.solve(dd_residual);
    if (!db.allFinite()) {
        ++total_lambda_rejected_epochs_;
        last_diagnostics_.lambda_reject_reason = "q_amb_solve";
        consecutive_validated_fixes_ = 0;
        last_diagnostics_.consecutive_fix_count = 0;
        return false;
    }

    fixed_snapshot_ = snapshot_;
    fixed_snapshot_.state.head(na) -=
        best_attempt.transform.head_ambiguity_covariance * db;

    const MatrixXd q_ab_solved =
        ldlt.solve(
            best_attempt.transform.head_ambiguity_covariance.transpose());
    MatrixXd fixed_head_cov =
        snapshot_.covariance.topLeftCorner(na, na) -
        best_attempt.transform.head_ambiguity_covariance * q_ab_solved;
    fixed_head_cov = 0.5 * (fixed_head_cov + fixed_head_cov.transpose());
    fixed_snapshot_.covariance.topLeftCorner(na, na) = fixed_head_cov;
    fixed_snapshot_.covariance.topRightCorner(na, layout.nx() - na).setZero();
    fixed_snapshot_.covariance.bottomLeftCorner(layout.nx() - na, na).setZero();

    for (int k = 0; k < best_attempt.nb; ++k) {
        const int ref_index =
            best_attempt.differences[static_cast<size_t>(k)].reference_state_index;
        const int target_index =
            best_attempt.differences[static_cast<size_t>(k)].satellite_state_index;
        if (ref_index < 0 || target_index < 0 ||
            ref_index >= fixed_snapshot_.state.size() ||
            target_index >= fixed_snapshot_.state.size()) {
            continue;
        }
        fixed_snapshot_.state(target_index) =
            fixed_snapshot_.state(ref_index) - best_attempt.fixed_ambiguities(k);
        fixed_snapshot_.covariance.row(target_index).setZero();
        fixed_snapshot_.covariance.col(target_index).setZero();
        fixed_snapshot_.covariance(target_index, target_index) = 1e-6;
    }

    const auto fixed_postfit_build = buildDdMeasurementSystem(
        obs,
        osr_corrections,
        fixed_snapshot_.layout,
        fixed_snapshot_.state,
        config,
        trop_mapping_function,
        receiver_geometry_displacement);
    last_diagnostics_.fixed_postfit =
        validateDdPostfitResiduals(
            fixed_postfit_build,
            fixed_snapshot_.layout,
            fixed_snapshot_.covariance);
    if (!last_diagnostics_.fixed_postfit.accepted) {
        return reject_fixed(last_diagnostics_.fixed_postfit.reject_reason);
    }

    const int next_fix_count = consecutive_validated_fixes_ + 1;
    if (next_fix_count < kClaslibMinFixCount) {
        return reject_fixed("min_fix");
    }

    has_fixed_snapshot_ = true;
    last_diagnostics_.fixed_postfit_accepted = true;
    last_diagnostics_.lambda_accepted = true;
    last_diagnostics_.lambda_reject_reason.clear();
    last_diagnostics_.fixed_reject_reason.clear();
    validated_hold_differences_ = best_attempt.differences;
    validated_hold_ambiguities_ = best_attempt.fixed_ambiguities;
    consecutive_validated_fixes_ = next_fix_count;
    last_diagnostics_.consecutive_fix_count = consecutive_validated_fixes_;
    ++total_lambda_accepted_epochs_;
    return true;
}

PositionSolution DdFilterScaffold::processFloatUpdate(
    const ObservationData& obs,
    const CLASEpochContext& epoch_context,
    const ppp_shared::PPPState& native_state,
    const PositionSolution& native_float_solution,
    const ppp_shared::PPPConfig& config,
    const TropMappingFunction& trop_mapping_function) {
    last_diagnostics_ = DdUpdateDiagnostics{};
    const auto& osr_corrections = epoch_context.osr_corrections;
    const auto& epoch_atmos = epoch_context.epoch_atmos_tokens;
    const StateLayout layout{layoutOptionsFromConfig(config, osr_corrections)};
    const int observed_satellites = countObservedSatellites(osr_corrections);

    const bool needs_seed =
        !has_snapshot_ || snapshot_.state.size() != layout.nx() ||
        snapshot_.layout.nx() != layout.nx();
    if (needs_seed) {
        initializeFromNativeFloat(
            obs.time,
            layout,
            native_state,
            native_float_solution,
            config,
            osr_corrections,
            epoch_atmos);
    } else {
        snapshot_.layout = layout;
    }

    snapshot_.observed_satellites.clear();
    snapshot_.observed_satellites.reserve(osr_corrections.size());
    for (const auto& osr : osr_corrections) {
        if (osr.valid) {
            snapshot_.observed_satellites.push_back(osr.satellite);
        }
    }
    snapshot_.native_total_states = native_state.total_states;

    int network_id = -1;
    const bool has_network_id = readAtmosphereNetworkId(epoch_atmos, network_id);
    if (has_network_id) {
        if (snapshot_.has_atmosphere_network_id &&
            snapshot_.atmosphere_network_id != network_id) {
            for (int satno = 1; satno <= layout.options.max_satellites; ++satno) {
                const int iono_index = layout.ionosphereIndex(satno);
                if (iono_index >= 0) {
                    resetStateElement(iono_index, 0.0, 0.0);
                }
            }
            ionosphere_process_noise_by_satno_.clear();
            ionosphere_outage_by_satno_.clear();
        }
        snapshot_.atmosphere_network_id = network_id;
        snapshot_.has_atmosphere_network_id = true;
    } else {
        snapshot_.has_atmosphere_network_id = false;
    }

    const double dt = needs_seed ? 1.0 : std::max(obs.time - snapshot_.time, 0.001);
    if (std::abs(dt) > 30.0) {
        initializeFromNativeFloat(
            obs.time,
            layout,
            native_state,
            native_float_solution,
            config,
            osr_corrections,
            epoch_atmos);
        updateObservedStateBookkeeping(obs, osr_corrections, config, 0.0);
    } else {
        predictState(obs, osr_corrections, config, dt);
    }
    snapshot_.time = obs.time;

    const auto measurement_build = buildDdMeasurementSystem(
        obs,
        osr_corrections,
        snapshot_.layout,
        snapshot_.state,
        config,
        trop_mapping_function,
        epoch_context.receiver_tide_displacement);
    appendDdMeasurementRowsCsv(
        pppEnvOverrides().clas_dd_row_dump_path,
        obs.time,
        measurement_build);
    last_diagnostics_.measurement_rows =
        static_cast<int>(measurement_build.rows.size());
    last_diagnostics_.phase_rows = measurement_build.phase_rows;
    last_diagnostics_.code_rows = measurement_build.code_rows;
    last_diagnostics_.reference_groups = measurement_build.reference_groups;
    last_diagnostics_.row_summary =
        summarizeRowsByGroupFrequency(measurement_build.rows);
    last_diagnostics_.reference_summary =
        summarizeReferenceGroups(measurement_build.reference_groups_detail);

    rtk_measurement::MeasurementDiagnostics measurement_diagnostics;
    measurement_diagnostics.observation_count =
        static_cast<int>(measurement_build.rows.size());
    measurement_diagnostics.phase_observation_count = measurement_build.phase_rows;
    measurement_diagnostics.code_observation_count = measurement_build.code_rows;
    double residual_sum_sq = 0.0;
    for (const auto& row : measurement_build.rows) {
        residual_sum_sq += row.residual_m * row.residual_m;
        measurement_diagnostics.residual_max_abs_m =
            std::max(measurement_diagnostics.residual_max_abs_m, std::abs(row.residual_m));
    }
    if (measurement_diagnostics.observation_count > 0) {
        measurement_diagnostics.residual_rms_m =
            std::sqrt(residual_sum_sq /
                      static_cast<double>(measurement_diagnostics.observation_count));
    }

    if (measurement_build.rows.size() < 4) {
        PositionSolution solution = fallbackSolution(
            native_float_solution, "insufficient_dd_rows", observed_satellites);
        solution.time = obs.time;
        appendDiagnosticsCsv(obs.time, false);
        return solution;
    }

    std::vector<DdRow> accepted_rows;
    VectorXd accepted_state_delta = VectorXd::Zero(snapshot_.layout.nx());
    int accepted_reference_changes = 0;
    auto apply_update = [&](DdMeasurementBuildResult& build) {
        const VectorXd prior_state = snapshot_.state;
        auto measurement_system = build.measurement_system;
        last_diagnostics_.filter_update = rtk_update::applyMeasurementUpdate(
            snapshot_.state,
            snapshot_.covariance,
            measurement_system,
            std::numeric_limits<double>::infinity(),
            4);
        if (!last_diagnostics_.filter_update.ok) {
            return false;
        }
        const auto postfit_build = buildDdMeasurementSystem(
            obs,
            osr_corrections,
            snapshot_.layout,
            snapshot_.state,
            config,
            trop_mapping_function,
            epoch_context.receiver_tide_displacement);
        if (!postfitRowsAccepted(postfit_build)) {
            return false;
        }
        accepted_rows = build.rows;
        accepted_reference_changes = countReferenceChanges(build);
        last_diagnostics_.reference_change_groups = accepted_reference_changes;
        last_diagnostics_.ar_pdop = computePdop(
            accepted_rows, kClaslibArElevationMaskRad);
        last_diagnostics_.hold_pdop = computePdop(
            accepted_rows, kClaslibHoldElevationMaskRad);
        rememberReferenceGroups(build);
        accepted_state_delta = snapshot_.state - prior_state;
        return true;
    };

    const VectorXd predicted_state = snapshot_.state;
    const MatrixXd predicted_covariance = snapshot_.covariance;
    auto update_build = measurement_build;
    bool update_accepted = apply_update(update_build);
    if (!update_accepted) {
        snapshot_.state = predicted_state;
        snapshot_.covariance = predicted_covariance;

        initializeFromNativeFloat(
            obs.time,
            layout,
            native_state,
            native_float_solution,
            config,
            osr_corrections,
            epoch_atmos);
        updateObservedStateBookkeeping(obs, osr_corrections, config, 0.0);
        auto retry_build = buildDdMeasurementSystem(
            obs,
            osr_corrections,
            snapshot_.layout,
            snapshot_.state,
            config,
            trop_mapping_function,
            epoch_context.receiver_tide_displacement);
        last_diagnostics_.measurement_rows =
            static_cast<int>(retry_build.rows.size());
        last_diagnostics_.phase_rows = retry_build.phase_rows;
        last_diagnostics_.code_rows = retry_build.code_rows;
        last_diagnostics_.reference_groups = retry_build.reference_groups;
        last_diagnostics_.row_summary =
            summarizeRowsByGroupFrequency(retry_build.rows);
        last_diagnostics_.reference_summary =
            summarizeReferenceGroups(retry_build.reference_groups_detail);
        if (retry_build.rows.size() < 4 || !apply_update(retry_build)) {
            PositionSolution solution = fallbackSolution(
                native_float_solution,
                last_diagnostics_.filter_update.ok
                    ? "postfit_dd_residual"
                    : "kalman_update",
                observed_satellites);
            solution.time = obs.time;
            appendDiagnosticsCsv(obs.time, false);
            return solution;
        }
    }

    if (config.clas_mrtklib_float_parity && accepted_state_delta.size() >= 3) {
        const Vector3d position_delta = accepted_state_delta.head<3>();
        adaptive_position_process_noise_ecef_ =
            kClaslibForgetPosition * adaptive_position_process_noise_ecef_ +
            (1.0 - kClaslibForgetPosition) *
                kClaslibAfGainPosition * kClaslibAfGainPosition *
                (position_delta * position_delta.transpose());
    }
    if (snapshot_.layout.options.ionosphere_mode == IonosphereMode::EstimateAdaptive) {
        for (const auto& osr : osr_corrections) {
            if (!osr.valid) {
                continue;
            }
            const int satno = claslibSatelliteNumber(osr.satellite);
            const int iono_index = snapshot_.layout.ionosphereIndex(satno);
            if (iono_index < 0 ||
                iono_index >= snapshot_.covariance.rows()) {
                continue;
            }
            const double old_q = ionosphere_process_noise_by_satno_[satno];
            const double state_delta =
                iono_index < accepted_state_delta.size()
                    ? accepted_state_delta(iono_index)
                    : 0.0;
            ionosphere_process_noise_by_satno_[satno] =
                kClaslibForgetIono * old_q +
                (1.0 - kClaslibForgetIono) *
                    kClaslibAfGainIono * kClaslibAfGainIono *
                    state_delta * state_delta;
        }
    }

    ++total_updated_epochs_;
    last_diagnostics_.updated = true;
    updateAmbiguityLockCounts(accepted_rows);
    if (accepted_reference_changes > 0) {
        validated_hold_differences_.clear();
        validated_hold_ambiguities_.resize(0);
    }
    const bool fixed = conditionFixedSnapshot(
        obs,
        osr_corrections,
        accepted_rows,
        config,
        trop_mapping_function,
        epoch_context.receiver_tide_displacement,
        accepted_reference_changes,
        last_diagnostics_.ar_pdop);
    if (fixed) {
        const bool hold_postfit_ok =
            last_diagnostics_.fixed_postfit.chi_square_ratio <
            kClaslibHoldChiSquareGate;
        if (!hold_postfit_ok) {
            last_diagnostics_.hold_reject_reason = "hold_postfit";
        } else if (!std::isfinite(last_diagnostics_.hold_pdop) ||
                   last_diagnostics_.hold_pdop > kClaslibMaxPdopHold) {
            last_diagnostics_.hold_reject_reason = "hold_pdop";
        } else if (consecutive_validated_fixes_ < kClaslibMinFixCount) {
            last_diagnostics_.hold_reject_reason = "hold_min_fix";
        } else {
            applyHoldAmbiguity(accepted_rows);
        }
    }
    appendStateDumpCsv(obs.time, osr_corrections, fixed);
    PositionSolution solution = publishSolution(
        native_float_solution,
        measurement_diagnostics,
        observed_satellites,
        fixed ? fixed_snapshot_ : snapshot_,
        fixed);
    solution.time = obs.time;
    appendDiagnosticsCsv(obs.time, fixed);
    return solution;
}

PositionSolution DdFilterScaffold::processFloatPassthrough(
    const GNSSTime& time,
    const ppp_shared::PPPState& native_state,
    const PositionSolution& native_float_solution,
    const ppp_shared::PPPConfig& config,
    const std::vector<OSRCorrection>& osr_corrections,
    const std::map<std::string, std::string>& epoch_atmos) {
    const StateLayout layout{layoutOptionsFromConfig(config, osr_corrections)};
    ensureSnapshotStorage(layout, false);

    snapshot_.time = time;
    snapshot_.observed_satellites.clear();
    snapshot_.observed_satellites.reserve(osr_corrections.size());
    snapshot_.seeded_from_native_float = true;
    snapshot_.native_total_states = native_state.total_states;
    snapshot_.has_atmosphere_network_id =
        readAtmosphereNetworkId(epoch_atmos, snapshot_.atmosphere_network_id);

    if (native_state.state.size() >= native_state.pos_index + 3) {
        snapshot_.state.segment(0, 3) =
            native_state.state.segment(native_state.pos_index, 3);
    }
    if (native_state.covariance.rows() >= native_state.pos_index + 3 &&
        native_state.covariance.cols() >= native_state.pos_index + 3) {
        snapshot_.covariance.block(0, 0, 3, 3) =
            native_state.covariance.block(native_state.pos_index, native_state.pos_index, 3, 3);
    }

    const int trop_index = layout.troposphereIndex();
    if (trop_index >= 0 &&
        native_state.trop_index >= 0 &&
        native_state.state.size() > native_state.trop_index) {
        snapshot_.state(trop_index) = native_state.state(native_state.trop_index);
        if (native_state.covariance.rows() > native_state.trop_index &&
            native_state.covariance.cols() > native_state.trop_index) {
            snapshot_.covariance(trop_index, trop_index) =
                native_state.covariance(native_state.trop_index, native_state.trop_index);
        }
    }

    for (const auto& [satellite, native_index] : native_state.ionosphere_indices) {
        const int satno = claslibSatelliteNumber(satellite);
        const int dd_index = layout.ionosphereIndex(satno);
        if (dd_index < 0 || native_index < 0 ||
            native_state.state.size() <= native_index) {
            continue;
        }
        snapshot_.state(dd_index) = native_state.state(native_index);
        if (native_state.covariance.rows() > native_index &&
            native_state.covariance.cols() > native_index) {
            snapshot_.covariance(dd_index, dd_index) =
                native_state.covariance(native_index, native_index);
        }
    }

    for (const auto& osr : osr_corrections) {
        if (osr.valid) {
            snapshot_.observed_satellites.push_back(osr.satellite);
        }
    }

    PositionSolution solution = native_float_solution;
    solution.time = time;
    solution.status = SolutionStatus::PPP_FLOAT;
    solution.ratio = 0.0;
    solution.num_fixed_ambiguities = 0;
    solution.num_satellites = static_cast<int>(snapshot_.observed_satellites.size());
    return solution;
}

void DdFilterScaffold::reset() {
    snapshot_ = StateSnapshot{};
    fixed_snapshot_ = StateSnapshot{};
    adaptive_position_process_noise_ecef_.setZero();
    ionosphere_outage_by_satno_.clear();
    ionosphere_process_noise_by_satno_.clear();
    ambiguity_outage_by_satno_freq_.clear();
    ambiguity_lock_by_satno_freq_.clear();
    reference_by_group_.clear();
    validated_hold_differences_.clear();
    validated_hold_ambiguities_.resize(0);
    last_diagnostics_ = DdUpdateDiagnostics{};
    total_updated_epochs_ = 0;
    total_fallback_epochs_ = 0;
    total_lambda_accepted_epochs_ = 0;
    total_lambda_rejected_epochs_ = 0;
    fixed_reject_ratio_epochs_ = 0;
    fixed_reject_postfit_epochs_ = 0;
    fixed_reject_pdop_epochs_ = 0;
    fixed_reject_minfix_epochs_ = 0;
    fixed_reject_refchange_epochs_ = 0;
    total_hold_applied_epochs_ = 0;
    consecutive_validated_fixes_ = 0;
    lambda_ratio_sum_ = 0.0;
    lambda_ratio_count_ = 0;
    has_snapshot_ = false;
    has_fixed_snapshot_ = false;
}
}  // namespace libgnss::ppp_clas_dd
