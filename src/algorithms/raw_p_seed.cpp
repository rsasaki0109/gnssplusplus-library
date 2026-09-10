#include <libgnss++/algorithms/raw_p_seed.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace libgnss {
namespace raw_p_seed {

int c7ClockComponentFor(GNSSSystem system, SignalType signal) {
    // Exact source order from sysfreq2sigtype.m / Phase101.  Keep this
    // mapping at the raw ingress boundary as well as the graph boundary so
    // adapter admission and factor insertion cannot silently diverge.
    switch (system) {
        case GNSSSystem::GPS:
            if (signal == SignalType::GPS_L1CA ||
                signal == SignalType::GPS_L1P) return 0;
            if (signal == SignalType::GPS_L5) return 4;
            return -1;
        case GNSSSystem::GLONASS:
            if (signal == SignalType::GLO_L1CA ||
                signal == SignalType::GLO_L1P) return 1;
            return -1;
        case GNSSSystem::Galileo:
            if (signal == SignalType::GAL_E1) return 2;
            if (signal == SignalType::GAL_E5A) return 5;
            return -1;
        case GNSSSystem::BeiDou:
            if (signal == SignalType::BDS_B1I ||
                signal == SignalType::BDS_B1C) return 3;
            if (signal == SignalType::BDS_B2A) return 6;
            return -1;
        default:
            // QZSS is folded into native SPP's GPS clock group, but the
            // source C7 selector has no QZSS signal slot.  Reject it rather
            // than claiming it is a GPS-L1 measurement.
            return -1;
    }
}

namespace {

constexpr int kPositionClockUnknowns = 4;
constexpr double kTimeEqualityToleranceS = 1e-6;

constexpr GNSSSystem kKnownClockGroups[] = {
    GNSSSystem::GPS,     GNSSSystem::GLONASS, GNSSSystem::Galileo,
    GNSSSystem::BeiDou,  GNSSSystem::QZSS,    GNSSSystem::NavIC,
};

bool finiteTime(const GNSSTime& time) {
    return std::isfinite(time.tow);
}

GNSSSystem rawPseudorangeClockGroup(GNSSSystem system) {
    switch (system) {
        case GNSSSystem::GPS:
        case GNSSSystem::QZSS:
            return GNSSSystem::GPS;
        case GNSSSystem::GLONASS:
        case GNSSSystem::Galileo:
        case GNSSSystem::BeiDou:
        case GNSSSystem::NavIC:
            return system;
        default:
            return GNSSSystem::UNKNOWN;
    }
}

bool isSupportedClockGroup(GNSSSystem group) {
    switch (group) {
        case GNSSSystem::GPS:
        case GNSSSystem::GLONASS:
        case GNSSSystem::Galileo:
        case GNSSSystem::BeiDou:
        case GNSSSystem::QZSS:
            return true;
        default:
            // NavIC is a known enum value, but native SPP currently has no
            // corrected-measurement/system-column path for it.
            return false;
    }
}

bool usesSeparateClockBias(GNSSSystem group) {
    return group != GNSSSystem::UNKNOWN && group != GNSSSystem::GPS;
}

GNSSSystem selectReferenceClockGroup(
    const std::map<GNSSSystem, int>& group_counts) {
    const auto gps_it = group_counts.find(GNSSSystem::GPS);
    if (gps_it != group_counts.end() && gps_it->second > 0) {
        return GNSSSystem::GPS;
    }

    GNSSSystem best_group = GNSSSystem::UNKNOWN;
    int best_count = -1;
    // std::map iteration order intentionally matches the native SPP helper;
    // equal-count ties therefore retain the enum ordering used there.
    for (const auto& [group, count] : group_counts) {
        if (count > best_count) {
            best_group = group;
            best_count = count;
        }
    }
    return best_group;
}

std::vector<GNSSSystem> separateClockGroups(
    const std::map<GNSSSystem, int>& group_counts,
    GNSSSystem reference_group,
    bool model_intersystem_bias) {
    std::vector<GNSSSystem> groups;
    if (!model_intersystem_bias) {
        return groups;
    }
    for (const auto& [group, count] : group_counts) {
        if (count > 0 && group != reference_group &&
            usesSeparateClockBias(group)) {
            groups.push_back(group);
        }
    }
    return groups;
}

std::size_t countRawPseudorangeSatellites(
    const ObservationData& epoch,
    std::size_t& rows,
    std::set<GNSSSystem>& clock_groups) {
    std::set<SatelliteId> satellites;
    clock_groups.clear();
    rows = 0;
    for (const auto& observation : epoch.observations) {
        if (!observation.valid || !observation.has_pseudorange ||
            !std::isfinite(observation.pseudorange) ||
            observation.pseudorange <= 0.0) {
            continue;
        }
        ++rows;
        satellites.insert(observation.satellite);
        clock_groups.insert(rawPseudorangeClockGroup(observation.satellite.system));
    }
    return satellites.size();
}

ObservationData rawPOnlyCopy(const ObservationData& input) {
    ObservationData output = input;
    for (auto& observation : output.observations) {
        // The native SPP implementation's position/clock solve is
        // pseudorange-only.  Clear all range-rate availability on the private
        // copy as a hard contract so a future SPP change cannot make this
        // preparatory stage depend on Doppler-derived velocity.
        observation.doppler = 0.0;
        observation.has_doppler = false;
        observation.pseudorange_rate_mps = 0.0;
        observation.has_pseudorange_rate_mps = false;
        observation.source_carrier_frequency_hz = 0.0;
        observation.has_source_carrier_frequency_hz = false;
    }
    output.receiver_clock_drift_mps =
        std::numeric_limits<double>::quiet_NaN();
    return output;
}

bool bootstrapNativePosition(const ObservationData& input_epoch,
                             const NavigationData& nav,
                             const Config& config,
                             Vector3d& position_ecef) {
    ProcessorConfig bootstrap_processor_config = config.processor_config;
    bootstrap_processor_config.mode = PositioningMode::SPP;
    // This is a private first-pass gate only.  The caller's configured mask
    // is restored for the ordinary SPP/preprocessEpoch pass below.
    bootstrap_processor_config.elevation_mask = -90.0;

    SPPProcessor::SPPConfig bootstrap_spp_config = config.spp_config;
    // A non-negative SPP override would otherwise supersede the private
    // processor-level bootstrap gate.  The ordinary config is not modified.
    bootstrap_spp_config.elevation_mask_override_deg = -1.0;

    SPPProcessor bootstrap_spp(bootstrap_spp_config);
    if (!bootstrap_spp.initialize(bootstrap_processor_config)) {
        return false;
    }

    ObservationData bootstrap_epoch = rawPOnlyCopy(input_epoch);
    // Do not let the application reconnaissance point (or any caller-provided
    // location) bypass the native cold-start path.  This is a same-run raw-P
    // anchor, not an imported receiver coordinate.
    bootstrap_epoch.receiver_position = Vector3d::Zero();
    const PositionSolution bootstrap_solution =
        bootstrap_spp.processEpoch(bootstrap_epoch, nav);
    // Native solvePositionLS performs its weighted rank checks before marking
    // an SPP solution valid.  Retain an explicit finite/ECEF-domain check here
    // because PositionSolution::isValid() alone does not inspect coordinates.
    if (!bootstrap_solution.isValid() ||
        bootstrap_solution.num_satellites < 4 ||
        !bootstrap_solution.position_ecef.allFinite() ||
        !std::isfinite(bootstrap_solution.receiver_clock_bias) ||
        !std::isfinite(bootstrap_solution.position_ecef.norm()) ||
        bootstrap_solution.position_ecef.norm() <= 1.0e6) {
        return false;
    }

    position_ecef = bootstrap_solution.position_ecef;
    return true;
}

void clearVelocities(Result& result) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (auto& epoch : result.epochs) {
        epoch.velocity_ecef_mps.setConstant(nan);
        epoch.has_velocity = false;
    }
}

void refreshEpochCounts(Result& result) {
    result.evaluated_epoch_count = 0;
    result.accepted_epoch_count = 0;
    result.rejected_epoch_count = 0;
    for (const auto& epoch : result.epochs) {
        if (epoch.status == EpochStatus::NotEvaluated) {
            continue;
        }
        ++result.evaluated_epoch_count;
        if (epoch.status == EpochStatus::Accepted) {
            ++result.accepted_epoch_count;
        } else {
            ++result.rejected_epoch_count;
        }
    }
}

void fail(Result& result,
          EpochStatus status,
          const std::string& reason,
          std::size_t epoch_index = std::numeric_limits<std::size_t>::max(),
          bool preserve_first_failure = false) {
    result.ok = false;
    if (!preserve_first_failure ||
        result.failure_status == EpochStatus::NotEvaluated) {
        result.failure_status = status;
        result.failure_reason = reason;
    }
    if (epoch_index < result.epochs.size()) {
        result.epochs[epoch_index].status = status;
        result.epochs[epoch_index].reason = reason;
    }
    clearVelocities(result);
    refreshEpochCounts(result);
}

}  // namespace

GeometryRank assessGeometryRank(const std::vector<Vector3d>& satellite_positions,
                                const Vector3d& receiver_position_ecef) {
    if (satellite_positions.size() <
        static_cast<std::size_t>(kPositionClockUnknowns)) {
        GeometryRank report;
        report.status = RankStatus::InsufficientRows;
        report.rows = satellite_positions.size();
        return report;
    }
    const std::vector<GNSSSystem> shared_clock_groups(
        satellite_positions.size(), GNSSSystem::GPS);
    return assessGeometryRank(satellite_positions,
                              receiver_position_ecef,
                              shared_clock_groups,
                              GNSSSystem::GPS,
                              false);
}

GeometryRank assessGeometryRank(
    const std::vector<Vector3d>& satellite_positions,
    const Vector3d& receiver_position_ecef,
    const std::vector<GNSSSystem>& clock_groups,
    GNSSSystem reference_clock_group,
    bool model_intersystem_bias) {
    return assessGeometryRank(satellite_positions,
                              receiver_position_ecef,
                              clock_groups,
                              reference_clock_group,
                              model_intersystem_bias,
                              {});
}

GeometryRank assessGeometryRank(
    const std::vector<Vector3d>& satellite_positions,
    const Vector3d& receiver_position_ecef,
    const std::vector<GNSSSystem>& clock_groups,
    GNSSSystem reference_clock_group,
    bool model_intersystem_bias,
    const std::vector<double>& row_weights) {
    GeometryRank report;
    if (satellite_positions.size() != clock_groups.size()) {
        report.status = RankStatus::NumericallyInvalid;
        return report;
    }
    if (!row_weights.empty() &&
        row_weights.size() != satellite_positions.size()) {
        report.status = RankStatus::NumericallyInvalid;
        return report;
    }
    if (satellite_positions.empty()) {
        report.status = RankStatus::InsufficientRows;
        return report;
    }

    std::map<GNSSSystem, int> group_counts;
    for (const auto group : clock_groups) {
        if (!isSupportedClockGroup(group)) {
            report.status = RankStatus::NumericallyInvalid;
            return report;
        }
        ++group_counts[group];
    }
    if (reference_clock_group == GNSSSystem::UNKNOWN) {
        reference_clock_group = selectReferenceClockGroup(group_counts);
    }
    const auto reference_it = group_counts.find(reference_clock_group);
    if (reference_it == group_counts.end() || reference_it->second <= 0) {
        report.status = RankStatus::NumericallyInvalid;
        return report;
    }
    const auto bias_groups = separateClockGroups(
        group_counts, reference_clock_group, model_intersystem_bias);
    report.required_rank = kPositionClockUnknowns +
                           static_cast<int>(bias_groups.size());

    if (satellite_positions.size() <
        static_cast<std::size_t>(report.required_rank)) {
        report.status = RankStatus::InsufficientRows;
        report.rows = satellite_positions.size();
        return report;
    }

    if (!receiver_position_ecef.allFinite()) {
        report.status = RankStatus::NumericallyInvalid;
        return report;
    }
    const Vector3d& receiver = receiver_position_ecef;
    Eigen::MatrixXd geometry(
        static_cast<Eigen::Index>(satellite_positions.size()),
        report.required_rank);
    std::map<GNSSSystem, int> bias_columns;
    for (std::size_t i = 0; i < bias_groups.size(); ++i) {
        bias_columns[bias_groups[i]] =
            kPositionClockUnknowns + static_cast<int>(i);
    }
    Eigen::Index valid_rows = 0;
    bool invalid_row = false;
    for (std::size_t row = 0; row < satellite_positions.size(); ++row) {
        const auto& satellite_position = satellite_positions[row];
        if (!satellite_position.allFinite()) {
            invalid_row = true;
            continue;
        }
        const Vector3d delta = satellite_position - receiver;
        const double range = delta.norm();
        if (!std::isfinite(range) || range <= 0.0) {
            invalid_row = true;
            continue;
        }
        const Vector3d los = delta / range;
        if (!los.allFinite()) {
            invalid_row = true;
            continue;
        }
        const double row_weight = row_weights.empty()
                                       ? 1.0
                                       : row_weights[row];
        if (!std::isfinite(row_weight) || row_weight <= 0.0) {
            invalid_row = true;
            continue;
        }
        const double sqrt_weight = std::sqrt(row_weight);
        // Eigen::MatrixXd does not initialize coefficients.  Every row must
        // start at zero so that only the clock-group column for this row is
        // populated below; otherwise stale/uninitialized bias columns can
        // manufacture rank and make this preflight disagree with native H.
        geometry.row(valid_rows).setZero();
        geometry(valid_rows, 0) = -los.x() * sqrt_weight;
        geometry(valid_rows, 1) = -los.y() * sqrt_weight;
        geometry(valid_rows, 2) = -los.z() * sqrt_weight;
        geometry(valid_rows, 3) = sqrt_weight;
        const auto bias_column = bias_columns.find(clock_groups[row]);
        if (bias_column != bias_columns.end()) {
            geometry(valid_rows, bias_column->second) = sqrt_weight;
        }
        ++valid_rows;
    }

    report.rows = static_cast<std::size_t>(valid_rows);
    if (invalid_row) {
        report.status = RankStatus::NumericallyInvalid;
        return report;
    }
    if (valid_rows < report.required_rank) {
        report.status = RankStatus::InsufficientRows;
        return report;
    }

    geometry.conservativeResize(valid_rows, report.required_rank);
    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(geometry);
    report.rank = qr.rank();
    if (report.rank < 0) {
        report.status = RankStatus::NumericallyInvalid;
        report.rank = 0;
    } else if (report.rank < report.required_rank) {
        report.status = RankStatus::RankDeficient;
    } else {
        report.status = RankStatus::FullRank;
    }
    return report;
}

std::map<GNSSSystem, int> correctedClockGroupCounts(
    const std::vector<SPPProcessor::CorrectedMeasurement>& measurements,
    bool& unsupported_group) {
    std::map<GNSSSystem, int> counts;
    unsupported_group = false;
    for (const auto& measurement : measurements) {
        if (!isSupportedClockGroup(measurement.clock_group)) {
            unsupported_group = true;
            continue;
        }
        ++counts[measurement.clock_group];
    }
    return counts;
}

bool selectNativeUsedMeasurements(
    const std::vector<SPPProcessor::CorrectedMeasurement>& corrected_measurements,
    const PositionSolution& solution,
    std::vector<SPPProcessor::CorrectedMeasurement>& native_measurements) {
    native_measurements.clear();
    if (corrected_measurements.empty()) {
        return true;
    }
    // Native SPP fills this exact identity ledger from the final (possibly
    // QC-reduced) measurement set.  A non-empty corrected set without it is
    // ambiguous: selecting every row would silently reintroduce rows rejected
    // by outlier/FDE QC.  Matching signal and source row indexes as well as
    // satellite prevents a repeated satellite observation from being mapped
    // to the wrong design row.
    if (solution.spp_used_measurements.empty() ||
        solution.num_satellites !=
            static_cast<int>(solution.spp_used_measurements.size()) ||
        solution.satellites_used.size() !=
            solution.spp_used_measurements.size()) {
        return false;
    }

    std::vector<bool> used(corrected_measurements.size(), false);
    native_measurements.reserve(solution.spp_used_measurements.size());
    for (const auto& identity : solution.spp_used_measurements) {
        std::size_t match_index = corrected_measurements.size();
        for (std::size_t i = 0; i < corrected_measurements.size(); ++i) {
            if (used[i]) continue;
            const auto& measurement = corrected_measurements[i];
            if (measurement.identity.satellite == identity.satellite &&
                measurement.identity.signal == identity.signal &&
                measurement.identity.input_row_index ==
                    identity.input_row_index &&
                measurement.identity.secondary_input_row_index ==
                    identity.secondary_input_row_index &&
                measurement.identity.ionosphere_free ==
                    identity.ionosphere_free &&
                measurement.identity.clock_group == identity.clock_group &&
                std::isfinite(identity.weight) && identity.weight > 0.0) {
                match_index = i;
                break;
            }
        }
        if (match_index == corrected_measurements.size()) {
            return false;
        }
        used[match_index] = true;
        auto native_measurement = corrected_measurements[match_index];
        // Keep the native solve's design-column group and base weight as the
        // authority.  The matched corrected row supplies the same physical
        // row/geometry, while this assignment prevents a future serializer
        // or preprocessing path from silently substituting a different
        // clock column or weighting.
        native_measurement.clock_group = identity.clock_group;
        native_measurement.weight = identity.weight;
        native_measurement.variance = 1.0 / identity.weight;
        native_measurement.identity = identity;
        native_measurements.push_back(std::move(native_measurement));
    }
    return native_measurements.size() == solution.spp_used_measurements.size();
}

bool populateClockGroupReport(
    EpochSeed& epoch,
    const std::vector<SPPProcessor::CorrectedMeasurement>& measurements,
    const PositionSolution& solution,
    const std::map<GNSSSystem, double>& native_biases,
    bool model_intersystem_bias) {
    bool unsupported_group = false;
    const auto counts = correctedClockGroupCounts(measurements, unsupported_group);
    epoch.corrected_clock_groups = counts.size();
    epoch.reference_clock_group = selectReferenceClockGroup(counts);
    epoch.clock_group_biases.clear();
    epoch.clock_group_biases.reserve(std::size(kKnownClockGroups));

    bool all_estimates_available = !unsupported_group &&
                                   epoch.reference_clock_group != GNSSSystem::UNKNOWN;
    for (const auto group : kKnownClockGroups) {
        ClockGroupBias report;
        report.group = group;
        const auto count_it = counts.find(group);
        report.observed = count_it != counts.end() && count_it->second > 0;
        report.corrected_rows = report.observed
                                    ? static_cast<std::size_t>(count_it->second)
                                    : 0U;
        report.is_reference = report.observed &&
                              group == epoch.reference_clock_group;
        if (report.is_reference) {
            report.bias_m = solution.receiver_clock_bias;
            report.estimate_available = std::isfinite(report.bias_m);
        } else if (report.observed && model_intersystem_bias &&
                   usesSeparateClockBias(group)) {
            const auto bias_it = native_biases.find(group);
            if (bias_it != native_biases.end() && std::isfinite(bias_it->second)) {
                report.bias_m = bias_it->second;
                report.estimate_available = true;
            }
        }
        if (report.observed && !report.estimate_available &&
            (report.is_reference || model_intersystem_bias)) {
            all_estimates_available = false;
        }
        epoch.clock_group_biases.push_back(report);
    }
    return all_estimates_available;
}

void copyPreprocessDiagnostics(
    EpochSeed& epoch,
    const SPPProcessor::PreprocessDiagnostics& diagnostics) {
    epoch.preprocessing_diagnostics_available = diagnostics.available;
    epoch.preprocessing_input_rows = diagnostics.input_rows;
    epoch.preprocessing_accepted_rows = diagnostics.accepted_rows;
    epoch.preprocessing_rejected_rows = diagnostics.rejected_rows;
    epoch.preprocessing_reason_counts.clear();
    epoch.preprocessing_reason_counts.reserve(diagnostics.reason_counts.size());
    for (const auto& [reason, count] : diagnostics.reason_counts) {
        epoch.preprocessing_reason_counts.push_back({reason, count});
    }
    epoch.preprocessing_rows.clear();
    epoch.preprocessing_rows.reserve(diagnostics.rows.size());
    for (const auto& row : diagnostics.rows) {
        PreprocessRowDiagnostic copy;
        copy.input_row_index = row.input_row_index;
        copy.system = row.system;
        copy.accepted = row.accepted;
        copy.reason = row.reason;
        epoch.preprocessing_rows.push_back(std::move(copy));
    }
}

Result solve(const std::vector<ObservationData>& input_epochs,
             const NavigationData& nav,
             const Config& config) {
    Result result;
    result.endpoint_policy = config.endpoint_policy;
    result.collect_all_epochs_for_diagnostics =
        config.collect_all_epochs_for_diagnostics;
    result.velocity_diagnostics_disabled =
        config.collect_all_epochs_for_diagnostics && config.derive_velocity;
    result.input_epoch_count = input_epochs.size();
    result.epochs.reserve(input_epochs.size());
    const bool collect_all = config.collect_all_epochs_for_diagnostics;

    if (input_epochs.empty()) {
        fail(result, EpochStatus::InsufficientEpochs, "empty-input");
        return result;
    }
    if (!std::isfinite(config.max_gap_s) || config.max_gap_s <= 0.0 ||
        config.min_pseudorange_satellites <
            static_cast<std::size_t>(kPositionClockUnknowns) ||
        config.spp_config.max_iterations <= 0) {
        fail(result, EpochStatus::SolverRejected,
             "invalid-raw-p-seed-configuration");
        return result;
    }
    if (config.derive_velocity && input_epochs.size() < 2U) {
        EpochSeed epoch;
        epoch.input_epoch_index = 0;
        epoch.time = input_epochs.front().time;
        epoch.raw_source_index = input_epochs.front().raw_source_index;
        epoch.raw_utc_time_millis = input_epochs.front().raw_utc_time_millis;
        result.epochs.push_back(std::move(epoch));
        fail(result, EpochStatus::InsufficientEpochs,
             "velocity-requires-at-least-two-epochs", 0);
        return result;
    }

    std::vector<bool> timestamp_valid(input_epochs.size(), true);
    for (std::size_t i = 0; i < input_epochs.size(); ++i) {
        EpochSeed epoch;
        epoch.input_epoch_index = i;
        epoch.time = input_epochs[i].time;
        epoch.raw_source_index = input_epochs[i].raw_source_index;
        epoch.raw_utc_time_millis = input_epochs[i].raw_utc_time_millis;
        result.epochs.push_back(std::move(epoch));

        if (!finiteTime(input_epochs[i].time)) {
            timestamp_valid[i] = false;
            fail(result,
                 EpochStatus::NonfiniteTime,
                 "nonfinite-epoch-time",
                 i,
                 collect_all);
            if (!collect_all) return result;
            continue;
        }
        if (i == 0U) {
            continue;
        }
        const double dt = input_epochs[i].time - input_epochs[i - 1U].time;
        if (!std::isfinite(dt)) {
            timestamp_valid[i] = false;
            fail(result,
                 EpochStatus::NonfiniteTime,
                 "nonfinite-epoch-delta",
                 i,
                 collect_all);
            if (!collect_all) return result;
            continue;
        }
        if (std::abs(dt) <= kTimeEqualityToleranceS) {
            timestamp_valid[i] = false;
            fail(result,
                 EpochStatus::DuplicateTime,
                 "duplicate-epoch-time",
                 i,
                 collect_all);
            if (!collect_all) return result;
            continue;
        }
        if (dt < 0.0) {
            timestamp_valid[i] = false;
            fail(result, EpochStatus::NonmonotonicTime,
                 "nonmonotonic-epoch-time",
                 i,
                 collect_all);
            if (!collect_all) return result;
            continue;
        }
        if (dt > config.max_gap_s) {
            timestamp_valid[i] = false;
            fail(result, EpochStatus::TimeGap,
                 "epoch-time-gap-exceeds-limit",
                 i,
                 collect_all);
            if (!collect_all) return result;
            continue;
        }
    }

    ProcessorConfig processor_config = config.processor_config;
    processor_config.mode = PositioningMode::SPP;
    SPPProcessor spp(config.spp_config);
    if (!spp.initialize(processor_config)) {
        fail(result, EpochStatus::SolverRejected,
             "native-spp-initialization-failed");
        return result;
    }

    std::vector<Vector3d> positions;
    positions.reserve(input_epochs.size());
    bool bootstrap_position_ready = false;
    Vector3d bootstrap_position = Vector3d::Zero();
    for (std::size_t i = 0; i < input_epochs.size(); ++i) {
        if (!timestamp_valid[i]) {
            continue;
        }

        SPPProcessor independent_spp(config.spp_config);
        SPPProcessor* epoch_spp = &spp;
        if (collect_all) {
            if (!independent_spp.initialize(processor_config)) {
                fail(result,
                     EpochStatus::SolverRejected,
                     "native-spp-initialization-failed",
                     i,
                     true);
                continue;
            }
            epoch_spp = &independent_spp;
        }
        const auto reject_epoch = [&](EpochStatus status,
                                      const char* reason) {
            fail(result, status, reason, i, collect_all);
            return collect_all;
        };

        std::size_t raw_pseudorange_rows = 0;
        std::set<GNSSSystem> clock_groups;
        const std::size_t raw_pseudorange_satellites =
            countRawPseudorangeSatellites(input_epochs[i], raw_pseudorange_rows,
                                          clock_groups);
        result.epochs[i].raw_pseudorange_rows = raw_pseudorange_rows;
        result.epochs[i].raw_pseudorange_satellites =
            raw_pseudorange_satellites;
        result.epochs[i].raw_clock_groups = clock_groups.size();
        if (raw_pseudorange_satellites < config.min_pseudorange_satellites) {
            if (!reject_epoch(EpochStatus::InsufficientPseudorange,
                              "insufficient-pseudorange-satellites")) {
                return result;
            }
            continue;
        }
        bool unsupported_raw_group = false;
        for (const auto group : clock_groups) {
            if (!isSupportedClockGroup(group)) {
                unsupported_raw_group = true;
                break;
            }
        }
        if (unsupported_raw_group) {
            if (!reject_epoch(EpochStatus::UnsupportedClockGroups,
                              "unknown-or-unsupported-raw-clock-group")) {
                return result;
            }
            continue;
        }

        const bool bootstrap_ready_for_epoch = collect_all
                                                   ? false
                                                   : bootstrap_position_ready;
        const Vector3d bootstrap_position_for_epoch =
            collect_all ? Vector3d::Zero() : bootstrap_position;
        bool current_bootstrap_ready = bootstrap_ready_for_epoch;
        Vector3d current_bootstrap_position = bootstrap_position_for_epoch;

        ObservationData spp_input = rawPOnlyCopy(input_epochs[i]);
        if (config.bootstrap_position_before_elevation &&
            !current_bootstrap_ready) {
            if (!bootstrapNativePosition(input_epochs[i], nav, config,
                                          current_bootstrap_position)) {
                if (!reject_epoch(EpochStatus::SolverRejected,
                                  "native-raw-p-bootstrap-rejected")) {
                    return result;
                }
                continue;
            }
            current_bootstrap_ready = true;
            spp_input.receiver_position = current_bootstrap_position;
            if (!collect_all) {
                bootstrap_position_ready = current_bootstrap_ready;
                bootstrap_position = current_bootstrap_position;
            }
        }

        SPPProcessor::PreprocessDiagnostics preprocess_diagnostics;
        const auto [solution, corrected_measurements] =
            epoch_spp->preprocessEpoch(spp_input, nav, &preprocess_diagnostics);
        copyPreprocessDiagnostics(result.epochs[i], preprocess_diagnostics);
        auto& processed = result.epochs[i];
        processed.native_spp_status_available = true;
        processed.native_spp_status = solution.status;
        processed.satellites_used = solution.num_satellites;
        processed.iterations = solution.iterations;
        processed.degrees_of_freedom = solution.spp_degrees_of_freedom;
        processed.gdop = solution.gdop;
        processed.pdop = solution.pdop;
        processed.residual_rms_m = solution.residual_rms;
        processed.max_abs_residual_m = solution.spp_max_abs_residual_m;
        processed.converged = solution.isValid() && solution.iterations > 0 &&
                              solution.iterations < config.spp_config.max_iterations;
        result.epochs[i].corrected_pseudorange_rows =
            corrected_measurements.size();
        std::vector<SPPProcessor::CorrectedMeasurement> native_measurements;
        if (solution.isValid()) {
            if (!selectNativeUsedMeasurements(corrected_measurements,
                                              solution,
                                              native_measurements)) {
                if (!reject_epoch(
                        EpochStatus::SolverRejected,
                        "native-used-measurement-provenance-unavailable")) {
                    return result;
                }
                continue;
            }
        } else {
            // Preserve the native solver failure reason below.  There is no
            // authoritative used-row ledger to select until a valid native
            // solution exists, so rank diagnostics are limited to the rows
            // returned by preprocessing.
            native_measurements = corrected_measurements;
        }
        processed.native_used_pseudorange_rows = native_measurements.size();
        std::vector<Vector3d> satellite_positions;
        std::vector<GNSSSystem> corrected_clock_groups;
        std::vector<double> corrected_weights;
        satellite_positions.reserve(native_measurements.size());
        corrected_clock_groups.reserve(native_measurements.size());
        corrected_weights.reserve(native_measurements.size());
        bool unsupported_corrected_group = false;
        for (const auto& measurement : native_measurements) {
            if (!isSupportedClockGroup(measurement.clock_group)) {
                unsupported_corrected_group = true;
                continue;
            }
            satellite_positions.emplace_back(measurement.satellite_ecef[0],
                                              measurement.satellite_ecef[1],
                                              measurement.satellite_ecef[2]);
            corrected_clock_groups.push_back(measurement.clock_group);
            corrected_weights.push_back(measurement.weight);
        }
        if (unsupported_corrected_group) {
            if (!reject_epoch(EpochStatus::UnsupportedClockGroups,
                              "unknown-or-unsupported-corrected-clock-group")) {
                return result;
            }
            continue;
        }

        const Vector3d rank_receiver =
            solution.isValid() && solution.position_ecef.allFinite()
                ? solution.position_ecef
                : input_epochs[i].receiver_position;
        const auto corrected_group_counts =
            correctedClockGroupCounts(native_measurements,
                                      unsupported_corrected_group);
        const GNSSSystem reference_clock_group =
            selectReferenceClockGroup(corrected_group_counts);
        processed.corrected_clock_groups = corrected_group_counts.size();
        processed.reference_clock_group = reference_clock_group;
        result.epochs[i].geometry_rank =
            assessGeometryRank(satellite_positions,
                               rank_receiver,
                               corrected_clock_groups,
                               reference_clock_group,
                               config.spp_config.model_intersystem_bias,
                               corrected_weights);
        if (result.epochs[i].geometry_rank.status == RankStatus::NumericallyInvalid) {
            if (!reject_epoch(EpochStatus::NonfiniteGeometry,
                              "nonfinite-pseudorange-satellite-geometry")) {
                return result;
            }
            continue;
        }
        if (result.epochs[i].geometry_rank.status ==
                RankStatus::InsufficientRows ||
            native_measurements.size() <
                static_cast<std::size_t>(result.epochs[i].geometry_rank.required_rank)) {
            if (!reject_epoch(EpochStatus::InsufficientGeometry,
                              "insufficient-corrected-pseudorange-geometry")) {
                return result;
            }
            continue;
        }
        if (result.epochs[i].geometry_rank.status == RankStatus::RankDeficient) {
            if (!reject_epoch(EpochStatus::RankDeficient,
                              "pseudorange-geometry-rank-deficient")) {
                return result;
            }
            continue;
        }
        if (!solution.isValid()) {
            if (!reject_epoch(EpochStatus::SolverRejected,
                              "native-spp-rejected-epoch")) {
                return result;
            }
            continue;
        }
        if (!solution.position_ecef.allFinite() ||
            !std::isfinite(solution.receiver_clock_bias)) {
            if (!reject_epoch(EpochStatus::NonfiniteSolution,
                              "nonfinite-spp-position-or-clock")) {
                return result;
            }
            continue;
        }

        if (!populateClockGroupReport(processed,
                                      native_measurements,
                                      solution,
                                      epoch_spp->getSystemBiases(),
                                      config.spp_config.model_intersystem_bias)) {
            if (!reject_epoch(EpochStatus::MissingClockBiasEstimate,
                              "missing-native-clock-group-estimate")) {
                return result;
            }
            continue;
        }

        auto& accepted = processed;
        accepted.status = EpochStatus::Accepted;
        accepted.position_ecef = solution.position_ecef;
        accepted.receiver_clock_bias_m = solution.receiver_clock_bias;
        accepted.reason = accepted.converged ? "accepted" : "iteration-limit";
        positions.push_back(solution.position_ecef);
    }

    if (config.derive_velocity && !collect_all) {
        if (config.endpoint_policy !=
            VelocityEndpointPolicy::OneSidedEndpointsCenteredInterior) {
            fail(result, EpochStatus::NonfiniteVelocity,
                 "unsupported-velocity-endpoint-policy");
            return result;
        }
        for (std::size_t i = 0; i < positions.size(); ++i) {
            const std::size_t previous = i == 0U ? 0U : i - 1U;
            const std::size_t next = i + 1U < positions.size() ? i + 1U : i;
            const double dt = input_epochs[next].time - input_epochs[previous].time;
            if (!std::isfinite(dt) || dt <= 0.0 ||
                !positions[previous].allFinite() || !positions[next].allFinite()) {
                fail(result, EpochStatus::NonfiniteVelocity,
                     "invalid-velocity-difference-interval", i);
                return result;
            }
            const Vector3d velocity =
                (positions[next] - positions[previous]) / dt;
            if (!velocity.allFinite()) {
                fail(result, EpochStatus::NonfiniteVelocity,
                     "nonfinite-derived-velocity", i);
                return result;
            }
            result.epochs[i].velocity_ecef_mps = velocity;
            result.epochs[i].has_velocity = true;
        }
    }

    if (collect_all) {
        refreshEpochCounts(result);
        if (result.rejected_epoch_count == 0U &&
            result.evaluated_epoch_count == result.input_epoch_count) {
            result.ok = true;
            result.failure_status = EpochStatus::Accepted;
            result.failure_reason.clear();
        }
        return result;
    }

    result.ok = true;
    result.failure_status = EpochStatus::Accepted;
    result.failure_reason.clear();
    refreshEpochCounts(result);
    return result;
}

namespace {

void adapterFailure(RawPNoDopplerSeedAdapterResult& result,
                    SeedAdapterStatus status,
                    const std::string& reason,
                    std::size_t seed_index =
                        std::numeric_limits<std::size_t>::max()) {
    result.ok = false;
    if (result.status == SeedAdapterStatus::NotEvaluated) {
        result.status = status;
        result.failure_reason = reason;
    }
    if (seed_index < result.seeds.size()) {
        result.seeds[seed_index].status = status;
        result.seeds[seed_index].reason = reason;
    }
}

bool exactEpochIdentity(const ObservationData& input,
                        const EpochSeed& raw_seed,
                        std::size_t expected_index) {
    return raw_seed.input_epoch_index == expected_index &&
           raw_seed.time == input.time &&
           raw_seed.raw_source_index == input.raw_source_index &&
           raw_seed.raw_utc_time_millis == input.raw_utc_time_millis;
}

}  // namespace

RawPNoDopplerSeedAdapterResult adaptSameRunNoDopplerSeeds(
    const std::vector<ObservationData>& input_epochs,
    const Result& raw_p_result) {
    RawPNoDopplerSeedAdapterResult result;
    result.input_epoch_count = input_epochs.size();

    if (input_epochs.size() != raw_p_result.epochs.size()) {
        adapterFailure(result,
                       SeedAdapterStatus::InputSizeMismatch,
                       "raw-p-result-input-epoch-count-mismatch");
        return result;
    }
    result.seeds.resize(input_epochs.size());
    if (!raw_p_result.ok) {
        adapterFailure(result,
                       SeedAdapterStatus::RawPResultRejected,
                       "raw-p-result-not-accepted");
    }

    bool c7_mapping_supported = true;
    bool all_drift_available = true;
    for (std::size_t i = 0; i < input_epochs.size(); ++i) {
        const auto& input = input_epochs[i];
        const auto& raw_seed = raw_p_result.epochs[i];
        auto& seed = result.seeds[i];
        seed.epoch_index = i;
        seed.time = input.time;
        seed.raw_source_index = input.raw_source_index;
        seed.raw_utc_time_millis = input.raw_utc_time_millis;
        seed.reference_clock_group = raw_seed.reference_clock_group;

        if (!exactEpochIdentity(input, raw_seed, i)) {
            adapterFailure(result,
                           SeedAdapterStatus::EpochIdentityMismatch,
                           "raw-p-result-epoch-identity-mismatch",
                           i);
            continue;
        }
        if (raw_seed.status != EpochStatus::Accepted) {
            adapterFailure(result,
                           SeedAdapterStatus::RawPResultRejected,
                           "raw-p-epoch-not-accepted",
                           i);
            continue;
        }
        if (!raw_seed.position_ecef.allFinite() ||
            !std::isfinite(raw_seed.receiver_clock_bias_m)) {
            adapterFailure(result,
                           SeedAdapterStatus::NonfinitePosition,
                           "raw-p-position-or-clock-not-finite",
                           i);
            continue;
        }
        if (!raw_seed.has_velocity) {
            adapterFailure(result,
                           SeedAdapterStatus::MissingVelocity,
                           "same-run-raw-p-velocity-unavailable",
                           i);
            continue;
        }
        if (!raw_seed.velocity_ecef_mps.allFinite()) {
            adapterFailure(result,
                           SeedAdapterStatus::NonfiniteVelocity,
                           "same-run-raw-p-velocity-not-finite",
                           i);
            continue;
        }

        seed.position_ecef = raw_seed.position_ecef;
        seed.velocity_ecef_mps = raw_seed.velocity_ecef_mps;
        seed.clock_bias_m = raw_seed.receiver_clock_bias_m;
        seed.has_position = true;
        seed.has_velocity = true;
        seed.has_clock = true;

        // This is deliberately an exact same-vector lookup.  There is no
        // nearest-time, retained-index, finite-difference, or zero fallback
        // for D: raw_p_seed's private solve cleared Doppler, so only the
        // original raw epoch can supply this field.
        if (std::isnan(input.receiver_clock_drift_mps)) {
            adapterFailure(result,
                           SeedAdapterStatus::MissingRawClockDrift,
                           "same-run-raw-clock-drift-missing",
                           i);
            all_drift_available = false;
            continue;
        }
        if (!std::isfinite(input.receiver_clock_drift_mps)) {
            adapterFailure(result,
                           SeedAdapterStatus::NonfiniteRawClockDrift,
                           "same-run-raw-clock-drift-nonfinite",
                           i);
            all_drift_available = false;
            continue;
        }
        seed.clock_rate_mps = input.receiver_clock_drift_mps;
        seed.has_clock_rate = true;

        // Native SPP's raw P clock grouping is constellation-level, while the
        // graph's C7 vector is frequency-slot-specific.  Validate the exact
        // source mapping for every retained P row, but do not mistake a
        // supported non-GPS row for a measured ISB: only the native GPS
        // reference is copied into C[0].  Missing C[1..6] values stay
        // unavailable; the graph may initialize those nuisance variables at
        // its documented numerical gauge, then estimate them from P rows.
        seed.clock_bias_components_m.fill(
            std::numeric_limits<double>::quiet_NaN());
        seed.clock_bias_component_available.fill(false);
        for (const auto& observation : input.observations) {
            if (!observation.valid || !observation.has_pseudorange ||
                !std::isfinite(observation.pseudorange) ||
                observation.pseudorange <= 0.0) {
                continue;
            }
            if (c7ClockComponentFor(observation.satellite.system,
                                    observation.signal) < 0) {
                ++seed.c7_unsupported_pseudorange_rows;
            } else {
                ++seed.c7_supported_pseudorange_rows;
            }
        }
        if (seed.reference_clock_group != GNSSSystem::GPS) {
            c7_mapping_supported = false;
            if (result.graph_disabled_reason.empty()) {
                result.graph_disabled_reason =
                    "native-reference-clock-is-not-proven-C7-C0";
            }
        } else {
            seed.clock_bias_components_m[0] = seed.clock_bias_m;
            seed.clock_bias_component_available[0] = true;
        }
        seed.c7_clock_mapping_supported =
            seed.reference_clock_group == GNSSSystem::GPS;
        if (seed.status == SeedAdapterStatus::NotEvaluated) {
            seed.status = SeedAdapterStatus::Accepted;
            seed.reason = "accepted";
            ++result.accepted_epoch_count;
        }
    }

    // A raw-P result must be complete before any handoff can be considered;
    // collect-all diagnostics remain useful metadata but are not a seed set.
    if (!raw_p_result.ok || result.accepted_epoch_count != input_epochs.size()) {
        result.ok = false;
        if (result.status == SeedAdapterStatus::NotEvaluated) {
            result.status = SeedAdapterStatus::RawPResultRejected;
            result.failure_reason = "raw-p-seed-set-is-not-complete";
        }
    } else if (!all_drift_available) {
        result.ok = false;
    } else {
        result.ok = true;
        result.status = SeedAdapterStatus::Accepted;
    }
    result.rejected_epoch_count =
        result.input_epoch_count - result.accepted_epoch_count;
    result.graph_compatible = result.ok && c7_mapping_supported &&
                              result.accepted_epoch_count == result.input_epoch_count;
    if (!result.graph_compatible && result.graph_disabled_reason.empty()) {
        result.graph_disabled_reason =
            result.ok ? "raw-P-handoff-is-not-C7-compatible"
                      : "raw-P-handoff-validation-failed";
    }
    return result;
}

const char* rankStatusName(RankStatus status) {
    switch (status) {
        case RankStatus::NotEvaluated: return "not-evaluated";
        case RankStatus::InsufficientRows: return "insufficient-rows";
        case RankStatus::FullRank: return "full-rank";
        case RankStatus::RankDeficient: return "rank-deficient";
        case RankStatus::NumericallyInvalid: return "numerically-invalid";
    }
    return "unknown";
}

const char* epochStatusName(EpochStatus status) {
    switch (status) {
        case EpochStatus::NotEvaluated: return "not-evaluated";
        case EpochStatus::Accepted: return "accepted";
        case EpochStatus::NonfiniteTime: return "nonfinite-time";
        case EpochStatus::DuplicateTime: return "duplicate-time";
        case EpochStatus::NonmonotonicTime: return "nonmonotonic-time";
        case EpochStatus::TimeGap: return "time-gap";
        case EpochStatus::InsufficientEpochs: return "insufficient-epochs";
        case EpochStatus::UnsupportedClockGroups: return "unsupported-clock-groups";
        case EpochStatus::InsufficientPseudorange: return "insufficient-pseudorange";
        case EpochStatus::InsufficientGeometry: return "insufficient-geometry";
        case EpochStatus::NonfiniteGeometry: return "nonfinite-geometry";
        case EpochStatus::RankDeficient: return "rank-deficient";
        case EpochStatus::NonfiniteSolution: return "nonfinite-solution";
        case EpochStatus::MissingClockBiasEstimate:
            return "missing-clock-bias-estimate";
        case EpochStatus::SolverRejected: return "solver-rejected";
        case EpochStatus::NonfiniteVelocity: return "nonfinite-velocity";
    }
    return "unknown";
}

const char* nativeSppStatusName(SolutionStatus status) {
    switch (status) {
        case SolutionStatus::NONE: return "none";
        case SolutionStatus::SPP: return "spp";
        case SolutionStatus::DGPS: return "dgps";
        case SolutionStatus::FLOAT: return "float";
        case SolutionStatus::FIXED: return "fixed";
        case SolutionStatus::PPP_FLOAT: return "ppp_float";
        case SolutionStatus::PPP_FIXED: return "ppp_fixed";
        case SolutionStatus::PROPAGATED: return "propagated";
    }
    return "unknown";
}

const char* velocityEndpointPolicyName(VelocityEndpointPolicy policy) {
    switch (policy) {
        case VelocityEndpointPolicy::OneSidedEndpointsCenteredInterior:
            return "one-sided-endpoints-centered-interior";
    }
    return "unknown";
}

const char* clockGroupName(GNSSSystem group) {
    switch (group) {
        case GNSSSystem::GPS: return "gps";
        case GNSSSystem::GLONASS: return "glonass";
        case GNSSSystem::Galileo: return "galileo";
        case GNSSSystem::BeiDou: return "beidou";
        case GNSSSystem::QZSS: return "qzss";
        case GNSSSystem::NavIC: return "navic";
        case GNSSSystem::SBAS: return "sbas";
        case GNSSSystem::UNKNOWN: return "unknown";
    }
    return "unknown";
}

const char* seedAdapterStatusName(SeedAdapterStatus status) {
    switch (status) {
        case SeedAdapterStatus::NotEvaluated: return "not-evaluated";
        case SeedAdapterStatus::Accepted: return "accepted";
        case SeedAdapterStatus::InputSizeMismatch:
            return "input-size-mismatch";
        case SeedAdapterStatus::EpochIdentityMismatch:
            return "epoch-identity-mismatch";
        case SeedAdapterStatus::RawPResultRejected:
            return "raw-p-result-rejected";
        case SeedAdapterStatus::NonfinitePosition:
            return "nonfinite-position";
        case SeedAdapterStatus::MissingVelocity:
            return "missing-velocity";
        case SeedAdapterStatus::NonfiniteVelocity:
            return "nonfinite-velocity";
        case SeedAdapterStatus::UnsupportedReferenceClockGroup:
            return "unsupported-reference-clock-group";
        case SeedAdapterStatus::UnsupportedC7ClockMapping:
            return "unsupported-c7-clock-mapping";
        case SeedAdapterStatus::MissingRawClockDrift:
            return "missing-raw-clock-drift";
        case SeedAdapterStatus::NonfiniteRawClockDrift:
            return "nonfinite-raw-clock-drift";
    }
    return "unknown";
}

}  // namespace raw_p_seed
}  // namespace libgnss
