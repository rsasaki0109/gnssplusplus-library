// Opt-in v2.1 smartphone FGO candidate.
//
// The v2.0 no-base entry point intentionally exercised only pseudorange plus
// CombinedImuFactor.  This wrapper reuses its frozen MAT/Android frame and
// attitude plumbing, but keeps the native v1 ordinary TDCP and position/clock
// motion factors enabled.  It is a separate binary so the v2.0/v5 artifacts
// and production defaults remain byte-for-byte untouched.

// Reuse the v2.0 loader/IMU contract without exporting or duplicating its
// internal implementation.  Rename its entry point, then provide this
// candidate's own orchestration below.
#define main gnss_fgo_imu_no_base_v2_disabled_main
#include "gnss_fgo_imu_no_base.cpp"
#undef main

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace {

struct OffsetPoint {
    libgnss::GNSSTime time;
    double speed_mps = 0.0;
};

struct OffsetCandidate {
    double offset_s = 0.0;
    double score = -std::numeric_limits<double>::infinity();
    std::size_t pairs = 0;
};

struct OffsetReport {
    double selected_offset_s = 0.0;
    double best_score = 0.0;
    double runner_up_score = 0.0;
    std::size_t pairs = 0;
    std::size_t finite_imu_samples = 0;
    bool confident = false;
    bool applied = false;
    std::string reason = "not evaluated";
};

std::vector<OffsetPoint> buildGnssSpeedSeries(
    const libgnss::FGOProcessor::FGOProblem& problem) {
    std::vector<OffsetPoint> speeds;
    if (problem.epochs.size() < 2) return speeds;
    speeds.reserve(problem.epochs.size() - 1);
    for (std::size_t i = 1; i < problem.epochs.size(); ++i) {
        const double dt = problem.epochs[i].time - problem.epochs[i - 1].time;
        if (dt <= 1e-6) continue;
        const double speed =
            (problem.epochs[i].position_ecef - problem.epochs[i - 1].position_ecef).norm() /
            dt;
        if (std::isfinite(speed)) {
            speeds.push_back({problem.epochs[i].time, speed});
        }
    }
    return speeds;
}

double imuMotionActivity(const libgnss::ImuSample& sample) {
    if (!sample.accel_raw.allFinite() || !sample.gyro_raw_radps.allFinite()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    constexpr double kGravity = 9.80665;
    const double accel_dynamic = std::abs(sample.accel_raw.norm() - kGravity);
    const double gyro_activity = 2.0 * sample.gyro_raw_radps.norm();
    const double activity = std::hypot(accel_dynamic, gyro_activity);
    return std::isfinite(activity) ? activity
                                   : std::numeric_limits<double>::quiet_NaN();
}

double positivePearson(const std::vector<double>& x,
                       const std::vector<double>& y) {
    if (x.size() != y.size() || x.size() < 2) return -1.0;
    double x_mean = 0.0;
    double y_mean = 0.0;
    for (std::size_t i = 0; i < x.size(); ++i) {
        x_mean += x[i];
        y_mean += y[i];
    }
    x_mean /= static_cast<double>(x.size());
    y_mean /= static_cast<double>(y.size());
    double numerator = 0.0;
    double x_energy = 0.0;
    double y_energy = 0.0;
    for (std::size_t i = 0; i < x.size(); ++i) {
        const double xd = x[i] - x_mean;
        const double yd = y[i] - y_mean;
        numerator += xd * yd;
        x_energy += xd * xd;
        y_energy += yd * yd;
    }
    if (x_energy <= 1e-12 || y_energy <= 1e-12) return -1.0;
    const double score = numerator / std::sqrt(x_energy * y_energy);
    return std::isfinite(score) ? score : -1.0;
}

double activityAt(const std::vector<libgnss::ImuSample>& samples,
                  const libgnss::GNSSTime& target,
                  double half_window_s,
                  std::size_t& finite_count) {
    const libgnss::GNSSTime lower = target - half_window_s;
    const libgnss::GNSSTime upper = target + half_window_s;
    const auto begin = std::lower_bound(
        samples.begin(), samples.end(), lower,
        [](const libgnss::ImuSample& sample, const libgnss::GNSSTime& time) {
            return sample.time < time;
        });
    double sum = 0.0;
    std::size_t count = 0;
    for (auto it = begin; it != samples.end() && !(upper < it->time); ++it) {
        const double activity = imuMotionActivity(*it);
        if (!std::isfinite(activity)) continue;
        sum += activity;
        ++count;
    }
    finite_count += count;
    return count > 0 ? sum / static_cast<double>(count)
                     : std::numeric_limits<double>::quiet_NaN();
}

// Estimate only an observable clock offset.  The returned offset has the
// explicit convention aligned_imu_time = raw_imu_time + offset_s.  The grid,
// window, and confidence gate are frozen in the v2.1 record; a weak or flat
// correlation intentionally selects zero rather than inventing a correction.
OffsetReport estimateOffset(
    const libgnss::FGOProcessor::FGOProblem& problem,
    const std::vector<libgnss::ImuSample>& samples) {
    constexpr double kGridMinS = -0.10;
    constexpr double kGridMaxS = 0.10;
    constexpr double kGridStepS = 0.01;
    constexpr double kHalfWindowS = 0.10;
    constexpr std::size_t kMinPairs = 20;
    constexpr double kMinScore = 0.40;
    constexpr double kMinMargin = 0.05;

    OffsetReport report;
    report.finite_imu_samples = samples.size();
    const auto speeds = buildGnssSpeedSeries(problem);
    if (speeds.size() < kMinPairs || samples.empty()) {
        report.reason = "insufficient truth-free GNSS-speed or IMU samples";
        return report;
    }

    std::vector<OffsetCandidate> candidates;
    for (int grid = -10; grid <= 10; ++grid) {
        const double offset = static_cast<double>(grid) * kGridStepS;
        std::vector<double> gnss;
        std::vector<double> activity;
        gnss.reserve(speeds.size());
        activity.reserve(speeds.size());
        std::size_t finite_count = 0;
        for (const auto& point : speeds) {
            // The target is the GNSS epoch expressed in the aligned IMU clock.
            const double a = activityAt(samples, point.time + offset,
                                        kHalfWindowS, finite_count);
            if (!std::isfinite(a)) continue;
            gnss.push_back(point.speed_mps);
            activity.push_back(a);
        }
        const double score = positivePearson(gnss, activity);
        candidates.push_back({offset, score, gnss.size()});
    }
    std::sort(candidates.begin(), candidates.end(),
              [](const OffsetCandidate& lhs, const OffsetCandidate& rhs) {
                  if (lhs.score != rhs.score) return lhs.score > rhs.score;
                  const double lhs_abs = std::abs(lhs.offset_s);
                  const double rhs_abs = std::abs(rhs.offset_s);
                  if (lhs_abs != rhs_abs) return lhs_abs < rhs_abs;
                  return lhs.offset_s < rhs.offset_s;
              });
    if (candidates.empty()) {
        report.reason = "offset grid produced no candidates";
        return report;
    }
    const OffsetCandidate& best = candidates.front();
    const double runner_up = candidates.size() > 1 ? candidates[1].score : -1.0;
    report.selected_offset_s = best.offset_s;
    report.best_score = best.score;
    report.runner_up_score = std::max(0.0, runner_up);
    report.pairs = best.pairs;
    const bool not_edge = std::abs(best.offset_s) < kGridMaxS - 1e-9;
    report.confident = best.pairs >= kMinPairs &&
                       best.score >= kMinScore &&
                       best.score - runner_up >= kMinMargin && not_edge;
    if (report.confident) {
        report.reason = "observable GNSS-speed/IMU-activity correlation passed frozen gate";
    } else if (!not_edge) {
        report.reason = "best correlation is at the physical grid edge; zero-offset fail-closed";
    } else if (best.pairs < kMinPairs) {
        report.reason = "best candidate has too few finite pairs; zero-offset fail-closed";
    } else if (best.score < kMinScore) {
        report.reason = "best correlation is below the frozen confidence threshold; zero-offset fail-closed";
    } else {
        report.reason = "best-versus-runner-up margin is below the frozen confidence threshold; zero-offset fail-closed";
    }
    return report;
}

std::string makeV21Summary(const Options& options,
                           const libgnss::FGOProcessor::FGOProblem& problem,
                           const libgnss::FGOProcessor::FGOResult& result,
                           const ImuBuildReport& imu_report,
                           bool fallback,
                           const OffsetReport& offset_report) {
    std::string summary = makeSummary(options, problem, result, imu_report, fallback);
    const std::string old_schema = "smartphone-r5-native-fgo-v2-mat-no-base-run.v1";
    const std::string new_schema = "smartphone-r5-native-fgo-v2-1-preserve-v1-run.v1";
    const std::size_t schema_pos = summary.find(old_schema);
    if (schema_pos != std::string::npos) {
        summary.replace(schema_pos, old_schema.size(), new_schema);
    }
    std::ostringstream extra;
    extra << std::setprecision(17);
    extra << ",\n  \"v2_1_graph_contract\": {\n"
          << "    \"ordinary_tdcp_factors_built\": " << problem.tdcp_factors.size() << ",\n"
          << "    \"ordinary_tdcp_factors_inserted\": "
          << result.diagnostics.tdcp_factors_inserted << ",\n"
          << "    \"position_clock_motion_pairs\": "
          << result.diagnostics.motion_factors << ",\n"
          << "    \"combined_imu_factors\": " << result.diagnostics.imu_intervals << ",\n"
          << "    \"double_difference_pseudorange_factors\": "
          << problem.double_difference_pseudorange_factors.size() << ",\n"
          << "    \"double_difference_carrier_factors\": "
          << problem.double_difference_carrier_factors.size() << ",\n"
          << "    \"single_difference_doppler_factors\": "
          << problem.single_difference_doppler_factors.size() << ",\n"
          << "    \"single_difference_tdcp_factors\": "
          << problem.single_difference_tdcp_factors.size() << ",\n"
          << "    \"hubert_loss_preserved\": true,\n"
          << "    \"v1_recipe_preserved\": true\n"
          << "  },\n"
          << "  \"observable_offset_estimator\": {\n"
          << "    \"selected_offset_seconds\": " << offset_report.selected_offset_s << ",\n"
          << "    \"best_score\": " << offset_report.best_score << ",\n"
          << "    \"runner_up_score\": " << offset_report.runner_up_score << ",\n"
          << "    \"pairs\": " << offset_report.pairs << ",\n"
          << "    \"finite_imu_samples\": " << offset_report.finite_imu_samples << ",\n"
          << "    \"confident\": " << (offset_report.confident ? "true" : "false") << ",\n"
          << "    \"applied\": " << (offset_report.applied ? "true" : "false") << ",\n"
          << "    \"reason\": ";
    writeJsonString(extra, offset_report.reason);
    extra << "\n  }";
    const std::size_t close_pos = summary.rfind("\n}");
    if (close_pos == std::string::npos) return summary;
    summary.insert(close_pos, extra.str());
    return summary;
}

}  // namespace

int main(int argc, char** argv) {
    Options options;
    if (!parseArguments(argc, argv, options)) return 2;

    libgnss::io::RINEXReader obs_reader;
    if (!obs_reader.open(options.obs_path)) {
        std::cerr << "failed to open observation file\n";
        return 1;
    }
    libgnss::io::RINEXReader::RINEXHeader obs_header;
    if (!obs_reader.readHeader(obs_header)) {
        std::cerr << "failed to read observation header\n";
        return 1;
    }
    libgnss::io::RINEXReader nav_reader;
    if (!nav_reader.open(options.nav_path)) {
        std::cerr << "failed to open navigation file\n";
        return 1;
    }
    libgnss::NavigationData nav;
    if (!nav_reader.readNavigationData(nav)) {
        std::cerr << "failed to read navigation file\n";
        return 1;
    }

    std::vector<libgnss::ObservationData> epochs;
    libgnss::ObservationData epoch;
    int observation_index = 0;
    while (epochs.size() < static_cast<std::size_t>(options.max_epochs) &&
           obs_reader.readObservationEpoch(epoch)) {
        if (observation_index++ < options.skip_epochs) continue;
        if (obs_header.approximate_position.norm() > 1.0e6) {
            epoch.receiver_position = obs_header.approximate_position;
        }
        epochs.push_back(epoch);
    }
    if (epochs.size() < 2) {
        std::cerr << "fewer than two observation epochs\n";
        return 1;
    }

    libgnss::FGOProcessor::FGOConfig config;
    config.backend = libgnss::FGOBackend::GTSAM;
    // These values are the native v1 smartphone recipe, written explicitly so
    // this candidate cannot inherit the v2.0 no-base-only defaults by accident.
    config.max_iterations = 8;
    config.pseudorange_sigma_m = 3.0;
    config.pseudorange_elevation_sigma_power = 1.0;
    config.motion_sigma_m = 50.0;
    config.clock_motion_sigma_m = 300.0;
    config.tdcp_sigma_m = 0.03;
    config.pseudorange_huber_threshold_sigma = 4.0;
    config.tdcp_huber_threshold_sigma = 4.0;
    config.max_tdcp_gap_s = 2.0;
    config.min_elevation_deg = 10.0;
    config.min_snr_dbhz = 0.0;
    config.min_satellites_per_epoch = 4;
    config.use_ionosphere_model = true;
    config.use_troposphere_model = true;
    config.use_inter_system_biases = true;
    config.use_pose3_state = true;
    config.use_imu = true;
    config.use_double_difference_factors = false;
    config.use_pseudorange_factors = true;
    config.use_tdcp_factors = true;
    config.use_motion_factors = true;
    config.use_position_motion_factors = true;
    config.use_clock_motion_factors = true;
    config.use_carrier_phase_factors = false;
    config.use_single_difference_doppler_factors = false;
    config.use_single_difference_tdcp_factors = false;
    config.use_velocity_states = false;
    config.use_velocity_motion_factors = false;
    config.use_ambiguity_between_factors = false;
    config.use_robust_loss = true;
    config.pose3_lever_arm_body_m = libgnss::Vector3d::Zero();

    const libgnss::FGOProcessor processor(config);
    libgnss::FGOProcessor::FGOProblem problem =
        processor.buildPseudorangeProblem(epochs, nav);
    if (problem.epochs.size() < 2 || problem.pseudorange_factors.empty() ||
        problem.tdcp_factors.empty()) {
        std::cerr << "v2.1 problem lacks pseudorange or ordinary TDCP factors\n";
        return 1;
    }
    if (!problem.double_difference_pseudorange_factors.empty() ||
        !problem.double_difference_carrier_factors.empty()) {
        std::cerr << "v2.1 no-base contract violated by problem builder\n";
        return 1;
    }

    ImuBuildReport imu_report;
    if (!buildImuInput(options.imu_path, problem, imu_report)) {
        std::cerr << "IMU initialization failed: " << imu_report.failure << "\n";
        return 1;
    }
    const std::vector<libgnss::ImuSample> raw_samples = problem.imu.samples_body_flu;
    OffsetReport offset_report = estimateOffset(problem, raw_samples);
    if (offset_report.confident) {
        for (auto& sample : problem.imu.samples_body_flu) {
            sample.time = sample.time + offset_report.selected_offset_s;
        }
        std::sort(problem.imu.samples_body_flu.begin(),
                  problem.imu.samples_body_flu.end(),
                  [](const libgnss::ImuSample& lhs, const libgnss::ImuSample& rhs) {
                      return lhs.time < rhs.time;
                  });
        offset_report.applied = true;
    }

    libgnss::FGOProcessor::FGOResult result = processor.optimizeProblem(problem);
    bool fallback = false;
    if (result.solution.isEmpty() || !result.diagnostics.converged ||
        result.diagnostics.imu_intervals == 0 ||
        result.diagnostics.tdcp_factors_inserted != problem.tdcp_factors.size()) {
        // Fallback is the exact v1 graph, not the v2.0 pseudorange-only graph:
        // retain ordinary TDCP/motion and simply disable the IMU state chain.
        fallback = true;
        problem.imu.valid = false;
        libgnss::FGOProcessor::FGOConfig fallback_config = config;
        fallback_config.use_imu = false;
        fallback_config.use_pose3_state = false;
        const libgnss::FGOProcessor fallback_processor(fallback_config);
        result = fallback_processor.optimizeProblem(problem);
    }
    if (result.solution.isEmpty() || !result.diagnostics.converged) {
        std::cerr << "v2.1 FGO produced no finite converged output\n";
        return 1;
    }
    if (!problem.double_difference_pseudorange_factors.empty() ||
        !problem.double_difference_carrier_factors.empty() ||
        problem.single_difference_doppler_factors.size() != 0 ||
        problem.single_difference_tdcp_factors.size() != 0) {
        std::cerr << "v2.1 forbidden factor class was present\n";
        return 1;
    }

    std::ostringstream csv;
    csv << "phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n";
    std::size_t finite_rows = 0;
    for (const auto& solution : result.solution.solutions) {
        double lat = solution.position_geodetic.latitude;
        double lon = solution.position_geodetic.longitude;
        if (!std::isfinite(lat) || !std::isfinite(lon)) {
            double height = 0.0;
            libgnss::ecef2geodetic(solution.position_ecef, lat, lon, height);
        }
        const double timestamp = unixMillis(solution.time);
        if (!std::isfinite(timestamp) || !std::isfinite(lat) || !std::isfinite(lon) ||
            std::abs(lat) > kPi / 2.0 || std::abs(lon) > kPi) {
            std::cerr << "non-finite or out-of-range v2.1 output row\n";
            return 1;
        }
        csv << options.dataset_id << ',' << static_cast<long long>(timestamp) << ','
            << std::fixed << std::setprecision(10) << lat * kRadToDeg << ','
            << lon * kRadToDeg << '\n';
        ++finite_rows;
    }
    if (finite_rows == 0 || !atomicWrite(options.out_path, csv.str())) {
        std::cerr << "failed to atomically publish v2.1 output\n";
        return 1;
    }
    const std::string summary = makeV21Summary(
        options, problem, result, imu_report, fallback, offset_report);
    if (!atomicWrite(options.summary_path, summary)) {
        std::cerr << "failed to atomically publish v2.1 summary\n";
        return 1;
    }
    std::cout << "native v2.1 preserve-v1 FGO: epochs=" << problem.epochs.size()
              << " output=" << finite_rows
              << " pseudorange=" << problem.pseudorange_factors.size()
              << " tdcp=" << problem.tdcp_factors.size()
              << " tdcp_inserted=" << result.diagnostics.tdcp_factors_inserted
              << " motion_pairs=" << result.diagnostics.motion_factors
              << " combined_imu=" << result.diagnostics.imu_intervals
              << " offset_s=" << offset_report.selected_offset_s
              << " offset_applied=" << (offset_report.applied ? "yes" : "no")
              << " fallback=" << (fallback ? "yes" : "no") << '\n';
    return 0;
}
