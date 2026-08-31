// Opt-in v3 smartphone FGO candidate: heading-optional IMU initialization.
//
// This translation unit deliberately includes the frozen v2.0 no-base loader
// only for the Android frame, atomic output, and observation plumbing.  The
// v2.1 binary/source is not included or modified.  The graph recipe below is
// the v2.1 preserve-v1 recipe (undifferenced pseudorange, ordinary TDCP,
// position/clock motion, Huber loss, and CombinedImuFactor) with one change:
// a weak GNSS course is not an initialization error.  Static gravity fixes
// roll/pitch; a sufficiently long, multi-epoch course fixes yaw; otherwise a
// finite broad yaw prior leaves yaw to the GNSS/TDCP/motion/IMU graph.

#define main gnss_fgo_imu_no_base_v2_disabled_main
#include "gnss_fgo_imu_no_base.cpp"
#undef main

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <exception>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr double kV3BroadYawSigmaRad = kPi;
constexpr std::size_t kV3CourseWindowEpochs = 5;
constexpr std::size_t kV3CourseMinimumWindows = 3;
constexpr double kV3CourseMinimumSpeedMps = 0.5;
constexpr double kV3CourseMaximumSpeedMps = 50.0;
constexpr double kV3CourseMaximumVerticalSpeedMps = 3.0;
constexpr double kV3CourseMaximumScatterDeg = 45.0;
constexpr double kV3CourseMinimumPathM = 2.0;
constexpr double kV3CourseMinimumSpanS = 2.0;

struct V3HeadingReport {
    bool gravity_passed = false;
    bool course_observable = false;
    bool heading_latched = false;
    bool covariance_finite = false;
    std::string yaw_mode = "not-initialized";
    std::string failure_reason;
    std::size_t course_windows = 0;
    std::size_t course_valid_windows = 0;
    double course_path_m = 0.0;
    double course_span_s = 0.0;
    double course_scatter_deg = 0.0;
    double course_mean_speed_mps = 0.0;
    double yaw_prior_sigma_rad = kV3BroadYawSigmaRad;
    std::size_t loaded_samples = 0;
    std::size_t stationary_samples = 0;
    double gravity_mean_norm = 0.0;
    double gravity_norm_std = 0.0;
};

struct V3CourseSample {
    std::size_t begin_epoch = 0;
    std::size_t end_epoch = 0;
    libgnss::Vector3d velocity_enu = libgnss::Vector3d::Zero();
    double speed_mps = 0.0;
};

// The v2.1 truth-free offset contract is copied by value under new names so
// the frozen v2.1 source remains byte-identical.  A flat/edge correlation is
// intentionally fail-closed to zero offset.
struct V3OffsetPoint {
    libgnss::GNSSTime time;
    double speed_mps = 0.0;
};

struct V3OffsetCandidate {
    double offset_s = 0.0;
    double score = -std::numeric_limits<double>::infinity();
    std::size_t pairs = 0;
};

struct V3OffsetReport {
    double selected_offset_s = 0.0;
    double best_score = 0.0;
    double runner_up_score = 0.0;
    std::size_t pairs = 0;
    std::size_t finite_imu_samples = 0;
    bool confident = false;
    bool applied = false;
    std::string reason = "not evaluated";
};

std::vector<V3OffsetPoint> v3BuildGnssSpeedSeries(
    const libgnss::FGOProcessor::FGOProblem& problem) {
    std::vector<V3OffsetPoint> speeds;
    if (problem.epochs.size() < 2) return speeds;
    speeds.reserve(problem.epochs.size() - 1);
    for (std::size_t i = 1; i < problem.epochs.size(); ++i) {
        const double dt = problem.epochs[i].time - problem.epochs[i - 1].time;
        if (dt <= 1e-6) continue;
        const double speed =
            (problem.epochs[i].position_ecef - problem.epochs[i - 1].position_ecef).norm() / dt;
        if (std::isfinite(speed)) speeds.push_back({problem.epochs[i].time, speed});
    }
    return speeds;
}

double v3ImuActivity(const libgnss::ImuSample& sample) {
    if (!sample.accel_raw.allFinite() || !sample.gyro_raw_radps.allFinite()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const double accel_dynamic = std::abs(sample.accel_raw.norm() - kGravity);
    const double gyro_activity = 2.0 * sample.gyro_raw_radps.norm();
    const double activity = std::hypot(accel_dynamic, gyro_activity);
    return std::isfinite(activity) ? activity : std::numeric_limits<double>::quiet_NaN();
}

double v3PositivePearson(const std::vector<double>& x,
                         const std::vector<double>& y) {
    if (x.size() != y.size() || x.size() < 2) return -1.0;
    double x_mean = 0.0;
    double y_mean = 0.0;
    for (std::size_t i = 0; i < x.size(); ++i) {
        x_mean += x[i];
        y_mean += y[i];
    }
    x_mean /= static_cast<double>(x.size());
    y_mean /= static_cast<double>(x.size());
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

double v3ActivityAt(const std::vector<libgnss::ImuSample>& samples,
                    const libgnss::GNSSTime& target,
                    double half_window_s) {
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
        const double activity = v3ImuActivity(*it);
        if (!std::isfinite(activity)) continue;
        sum += activity;
        ++count;
    }
    return count > 0 ? sum / static_cast<double>(count)
                     : std::numeric_limits<double>::quiet_NaN();
}

V3OffsetReport v3EstimateOffset(
    const libgnss::FGOProcessor::FGOProblem& problem,
    const std::vector<libgnss::ImuSample>& samples) {
    constexpr double kGridMinS = -0.10;
    constexpr double kGridMaxS = 0.10;
    constexpr double kGridStepS = 0.01;
    constexpr double kHalfWindowS = 0.10;
    constexpr std::size_t kMinPairs = 20;
    constexpr double kMinScore = 0.40;
    constexpr double kMinMargin = 0.05;

    V3OffsetReport report;
    report.finite_imu_samples = samples.size();
    const auto speeds = v3BuildGnssSpeedSeries(problem);
    if (speeds.size() < kMinPairs || samples.empty()) {
        report.reason = "insufficient truth-free GNSS-speed or IMU samples";
        return report;
    }
    std::vector<V3OffsetCandidate> candidates;
    for (int grid = -10; grid <= 10; ++grid) {
        const double offset = static_cast<double>(grid) * kGridStepS;
        std::vector<double> gnss;
        std::vector<double> activity;
        for (const auto& point : speeds) {
            const double a = v3ActivityAt(samples, point.time + offset, kHalfWindowS);
            if (!std::isfinite(a)) continue;
            gnss.push_back(point.speed_mps);
            activity.push_back(a);
        }
        candidates.push_back({offset, v3PositivePearson(gnss, activity), gnss.size()});
    }
    std::sort(candidates.begin(), candidates.end(),
              [](const V3OffsetCandidate& lhs, const V3OffsetCandidate& rhs) {
                  if (lhs.score != rhs.score) return lhs.score > rhs.score;
                  if (std::abs(lhs.offset_s) != std::abs(rhs.offset_s)) {
                      return std::abs(lhs.offset_s) < std::abs(rhs.offset_s);
                  }
                  return lhs.offset_s < rhs.offset_s;
              });
    if (candidates.empty()) {
        report.reason = "offset grid produced no candidates";
        return report;
    }
    const auto& best = candidates.front();
    const double runner_up = candidates.size() > 1 ? candidates[1].score : -1.0;
    report.selected_offset_s = best.offset_s;
    report.best_score = best.score;
    report.runner_up_score = std::max(0.0, runner_up);
    report.pairs = best.pairs;
    const bool not_edge = std::abs(best.offset_s) < kGridMaxS - 1e-9;
    report.confident = best.pairs >= kMinPairs && best.score >= kMinScore &&
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

bool v3FiniteSolution(const libgnss::FGOProcessor::FGOResult& result) {
    if (result.solution.isEmpty() || !std::isfinite(result.diagnostics.initial_cost) ||
        !std::isfinite(result.diagnostics.final_cost)) {
        return false;
    }
    for (const auto& solution : result.solution.solutions) {
        if (!solution.position_ecef.allFinite() ||
            !std::isfinite(solution.position_geodetic.latitude) ||
            !std::isfinite(solution.position_geodetic.longitude) ||
            !std::isfinite(solution.position_geodetic.height)) {
            return false;
        }
    }
    return true;
}

bool buildHeadingOptionalImuInput(const std::string& path,
                                  libgnss::FGOProcessor::FGOProblem& problem,
                                  V3HeadingReport& report) {
    if (problem.epochs.size() < 2) {
        report.failure_reason = "fewer than two GNSS epochs";
        return false;
    }
    libgnss::ImuSeries series;
    const auto load = libgnss::loadImuCsv(path, series);
    if (!load.ok || series.isEmpty()) {
        report.failure_reason = load.error.empty() ? "empty IMU series" : load.error;
        return false;
    }
    series.sortByTime();
    const Eigen::Matrix3d mounting = tarozMountingRotation();
    std::vector<libgnss::ImuSample> samples;
    samples.reserve(series.samples.size());
    for (auto sample : series.samples) {
        sample.accel_raw = mounting * sample.accel_raw;
        sample.gyro_raw_radps = mounting * sample.gyro_raw_radps;
        if (!sample.accel_raw.allFinite() || !sample.gyro_raw_radps.allFinite()) {
            report.failure_reason = "non-finite IMU sample";
            return false;
        }
        samples.push_back(sample);
    }
    if (samples.size() < kStationarySamples) {
        report.failure_reason = "IMU stream shorter than frozen leveling window";
        return false;
    }
    report.loaded_samples = samples.size();
    const std::size_t stationary_count = std::min(kStationarySamples, samples.size());
    std::vector<libgnss::ImuSample> stationary(samples.begin(),
                                                samples.begin() + stationary_count);
    Eigen::Vector3d accel_sum = Eigen::Vector3d::Zero();
    std::vector<double> norms;
    norms.reserve(stationary.size());
    for (const auto& sample : stationary) {
        accel_sum += sample.accel_raw;
        norms.push_back(sample.accel_raw.norm());
    }
    const double count = static_cast<double>(stationary.size());
    const Eigen::Vector3d accel_mean = accel_sum / count;
    const double mean_norm = accel_mean.norm();
    double variance = 0.0;
    for (double norm : norms) variance += (norm - mean_norm) * (norm - mean_norm);
    const double norm_std = std::sqrt(variance / count);
    report.stationary_samples = stationary.size();
    report.gravity_mean_norm = mean_norm;
    report.gravity_norm_std = norm_std;
    if (!std::isfinite(mean_norm) || !std::isfinite(norm_std) ||
        mean_norm < kGravityNormMin || mean_norm > kGravityNormMax ||
        norm_std > kGravityNormStdMax) {
        report.failure_reason = "leveling window is not stationary/low-dynamics under frozen gravity gate";
        return false;
    }
    report.gravity_passed = true;

    const libgnss::Vector3d origin_ecef = problem.epochs.front().position_ecef;
    double lat = 0.0;
    double lon = 0.0;
    double height = 0.0;
    libgnss::ecef2geodetic(origin_ecef, lat, lon, height);
    if (!origin_ecef.allFinite() || !std::isfinite(lat) || !std::isfinite(lon)) {
        report.failure_reason = "invalid GNSS nav origin";
        return false;
    }

    const libgnss::NominalState aligned = libgnss::fusion_initialization::alignStatic(
        stationary, libgnss::Vector3d::Zero(), kGravity);
    libgnss::FusionState state;
    state.nominal = aligned;
    state.covariance.setIdentity();

    std::vector<V3CourseSample> course_samples;
    for (std::size_t i = 0; i + kV3CourseWindowEpochs < problem.epochs.size(); ++i) {
        const std::size_t j = i + kV3CourseWindowEpochs;
        const double dt = problem.epochs[j].time - problem.epochs[i].time;
        ++report.course_windows;
        if (dt <= 1e-3) continue;
        const Eigen::Vector3d p0 = libgnss::ecef2enu(
            problem.epochs[i].position_ecef - origin_ecef, lat, lon);
        const Eigen::Vector3d p1 = libgnss::ecef2enu(
            problem.epochs[j].position_ecef - origin_ecef, lat, lon);
        const libgnss::Vector3d velocity = (p1 - p0) / dt;
        const double speed = std::hypot(velocity.x(), velocity.y());
        if (!velocity.allFinite() || !std::isfinite(speed) ||
            speed < kV3CourseMinimumSpeedMps || speed > kV3CourseMaximumSpeedMps ||
            std::abs(velocity.z()) > kV3CourseMaximumVerticalSpeedMps) {
            continue;
        }
        course_samples.push_back({i, j, velocity, speed});
    }
    report.course_valid_windows = course_samples.size();

    libgnss::Vector3d initial_velocity = libgnss::Vector3d::Zero();
    const double first_dt = problem.epochs[1].time - problem.epochs[0].time;
    if (first_dt > 1e-3) {
        const Eigen::Vector3d p0 = libgnss::ecef2enu(
            problem.epochs[0].position_ecef - origin_ecef, lat, lon);
        const Eigen::Vector3d p1 = libgnss::ecef2enu(
            problem.epochs[1].position_ecef - origin_ecef, lat, lon);
        const Eigen::Vector3d velocity = (p1 - p0) / first_dt;
        if (velocity.allFinite()) initial_velocity = velocity;
    }

    libgnss::Vector3d course_sum = libgnss::Vector3d::Zero();
    double unit_east = 0.0;
    double unit_north = 0.0;
    double path_m = 0.0;
    for (const auto& sample : course_samples) {
        course_sum += sample.velocity_enu;
        unit_east += sample.velocity_enu.x() / sample.speed_mps;
        unit_north += sample.velocity_enu.y() / sample.speed_mps;
        path_m += sample.speed_mps *
                  (problem.epochs[sample.end_epoch].time - problem.epochs[sample.begin_epoch].time);
    }
    report.course_path_m = path_m;
    if (!course_samples.empty()) {
        report.course_mean_speed_mps = course_sum.head<2>().norm() /
                                       static_cast<double>(course_samples.size());
        const double resultant = std::hypot(unit_east, unit_north) /
                                 static_cast<double>(course_samples.size());
        const double clamped = std::min(1.0, std::max(1e-12, resultant));
        report.course_scatter_deg = std::sqrt(std::max(0.0, -2.0 * std::log(clamped))) * kRadToDeg;
        report.course_span_s = problem.epochs[course_samples.back().end_epoch].time -
                               problem.epochs[course_samples.front().begin_epoch].time;
    }
    const bool enough_course =
        course_samples.size() >= kV3CourseMinimumWindows &&
        report.course_path_m >= kV3CourseMinimumPathM &&
        report.course_span_s >= kV3CourseMinimumSpanS &&
        std::isfinite(report.course_scatter_deg) &&
        report.course_scatter_deg <= kV3CourseMaximumScatterDeg;
    if (enough_course) {
        const libgnss::Vector3d course = course_sum /
                                         static_cast<double>(course_samples.size());
        if (libgnss::fusion_initialization::tryAlignHeading(state, course, 0.5, 5.0)) {
            report.course_observable = true;
            report.heading_latched = true;
            report.yaw_mode = "multi-epoch-gnss-course";
            report.yaw_prior_sigma_rad = 5.0 / kRadToDeg;
            initial_velocity = course;
        }
    }
    if (!report.course_observable) {
        // Yaw is a finite gauge anchor, not an abort condition.  Roll/pitch
        // remain gravity-aligned; GNSS position/TDCP/motion and the IMU chain
        // can jointly move yaw away from this broad initial prior.
        report.yaw_mode = "broad-yaw-prior-joint-estimation";
        report.yaw_prior_sigma_rad = kV3BroadYawSigmaRad;
        report.failure_reason = course_samples.empty()
                                    ? "insufficient multi-epoch course samples; broad yaw prior"
                                    : "course baseline/scatter gate not met; broad yaw prior";
        state.covariance.row(libgnss::fusion_index::ATTITUDE + 2).setZero();
        state.covariance.col(libgnss::fusion_index::ATTITUDE + 2).setZero();
        state.covariance(libgnss::fusion_index::ATTITUDE + 2,
                         libgnss::fusion_index::ATTITUDE + 2) =
            kV3BroadYawSigmaRad * kV3BroadYawSigmaRad;
    }
    report.covariance_finite = state.covariance.allFinite() &&
                               state.nominal.attitude_body_to_enu.coeffs().allFinite() &&
                               initial_velocity.allFinite();
    if (!report.covariance_finite) {
        report.failure_reason = "non-finite heading-optional initial covariance/state";
        return false;
    }

    auto& imu = problem.imu;
    imu.valid = true;
    imu.nav_origin_ecef = origin_ecef;
    imu.nav_origin_lat_rad = lat;
    imu.nav_origin_lon_rad = lon;
    imu.samples_body_flu = std::move(samples);
    imu.init_attitude_body_to_nav = state.nominal.attitude_body_to_enu.toRotationMatrix();
    imu.init_velocity_nav = initial_velocity;
    imu.init_accel_bias = aligned.accel_bias;
    imu.init_gyro_bias = aligned.gyro_bias;
    imu.noise.gravity_mps2 = kGravity;
    imu.noise.accel_noise_sigma = 0.025;
    imu.noise.gyro_noise_sigma = 0.0005;
    imu.noise.accel_bias_rw_sigma = 0.00025;
    imu.noise.gyro_bias_rw_sigma = 0.0000005;
    imu.noise.integration_sigma = 0.05;
    imu.init_attitude_sigma_roll_pitch_rad = 0.05;
    imu.init_attitude_sigma_yaw_rad = report.yaw_prior_sigma_rad;
    imu.init_velocity_sigma_mps = 0.5;
    imu.init_accel_bias_sigma = 0.1;
    imu.init_gyro_bias_sigma = 0.01;
    return true;
}

std::string v3Summary(const Options& options,
                      const libgnss::FGOProcessor::FGOProblem& problem,
                      const libgnss::FGOProcessor::FGOResult& result,
                      const V3HeadingReport& heading,
                      bool fallback,
                      const V3OffsetReport& offset,
                      const std::string& detail_path) {
    ImuBuildReport compatibility;
    compatibility.ok = heading.gravity_passed;
    compatibility.heading_latched = heading.heading_latched;
    compatibility.loaded_samples = heading.loaded_samples;
    compatibility.stationary_samples = heading.stationary_samples;
    compatibility.gravity_mean_norm = heading.gravity_mean_norm;
    compatibility.gravity_norm_std = heading.gravity_norm_std;
    compatibility.heading_windows = static_cast<int>(heading.course_valid_windows);
    compatibility.failure = heading.failure_reason;
    std::string summary = makeSummary(options, problem, result, compatibility, fallback);
    const std::string old_schema = "smartphone-r5-native-fgo-v2-1-preserve-v1-run.v1";
    const std::string old_schema_v2 = "smartphone-r5-native-fgo-v2-mat-no-base-run.v1";
    const std::string new_schema = "smartphone-r5-native-fgo-v3-heading-optional-run.v1";
    std::size_t schema_pos = summary.find(old_schema);
    if (schema_pos == std::string::npos) schema_pos = summary.find(old_schema_v2);
    if (schema_pos != std::string::npos) {
        const std::size_t old_size = summary.compare(schema_pos, old_schema.size(), old_schema) == 0
                                         ? old_schema.size()
                                         : old_schema_v2.size();
        summary.replace(schema_pos, old_size, new_schema);
    }
    std::ostringstream extra;
    extra << std::setprecision(17);
    extra << ",\n  \"v3_heading_optional_initialization\": {\n"
          << "    \"gravity_passed\": " << (heading.gravity_passed ? "true" : "false") << ",\n"
          << "    \"course_observable\": " << (heading.course_observable ? "true" : "false") << ",\n"
          << "    \"heading_latched\": " << (heading.heading_latched ? "true" : "false") << ",\n"
          << "    \"yaw_mode\": ";
    writeJsonString(extra, heading.yaw_mode);
    extra << ",\n    \"yaw_prior_sigma_rad\": " << heading.yaw_prior_sigma_rad << ",\n"
          << "    \"course_windows\": " << heading.course_windows << ",\n"
          << "    \"course_valid_windows\": " << heading.course_valid_windows << ",\n"
          << "    \"course_path_m\": " << heading.course_path_m << ",\n"
          << "    \"course_span_s\": " << heading.course_span_s << ",\n"
          << "    \"course_scatter_deg\": " << heading.course_scatter_deg << ",\n"
          << "    \"course_mean_speed_mps\": " << heading.course_mean_speed_mps << ",\n"
          << "    \"covariance_finite\": " << (heading.covariance_finite ? "true" : "false") << ",\n"
          << "    \"failure_reason\": ";
    writeJsonString(extra, heading.failure_reason);
    extra << "\n  },\n  \"observable_offset_estimator\": {\n"
          << "    \"selected_offset_seconds\": " << offset.selected_offset_s << ",\n"
          << "    \"best_score\": " << offset.best_score << ",\n"
          << "    \"runner_up_score\": " << offset.runner_up_score << ",\n"
          << "    \"pairs\": " << offset.pairs << ",\n"
          << "    \"finite_imu_samples\": " << offset.finite_imu_samples << ",\n"
          << "    \"confident\": " << (offset.confident ? "true" : "false") << ",\n"
          << "    \"applied\": " << (offset.applied ? "true" : "false") << ",\n"
          << "    \"reason\": ";
    writeJsonString(extra, offset.reason);
    extra << "\n  },\n  \"fallback_contract\": {\n"
          << "    \"solver_or_innovation_failure_falls_back_to_exact_v1\": true,\n"
          << "    \"fallback_selected\": " << (fallback ? "true" : "false") << ",\n"
          << "    \"production_default_changed\": false\n"
          << "  },\n  \"detail_artifact\": ";
    writeJsonString(extra, detail_path);
    extra << "\n";
    const std::size_t close_pos = summary.rfind("\n}");
    if (close_pos == std::string::npos) return summary;
    summary.insert(close_pos, extra.str());
    return summary;
}

std::string v3Detail(const libgnss::FGOProcessor::FGOResult& result) {
    std::ostringstream out;
    out << std::setprecision(17) << "{\n  \"schema_version\": \"smartphone-r5-native-fgo-v3-detail.v1\",\n"
        << "  \"truth_used\": false,\n  \"rows\": [\n";
    for (std::size_t i = 0; i < result.solution.solutions.size(); ++i) {
        const auto& solution = result.solution.solutions[i];
        out << "    {\"timestamp_unix_millis\": " << static_cast<long long>(unixMillis(solution.time))
            << ", \"ecef_m\": [" << solution.position_ecef.x() << ", "
            << solution.position_ecef.y() << ", " << solution.position_ecef.z() << "],\n"
            << "     \"latitude_deg\": " << solution.position_geodetic.latitude * kRadToDeg
            << ", \"longitude_deg\": " << solution.position_geodetic.longitude * kRadToDeg
            << ", \"height_m\": " << solution.position_geodetic.height << "}";
        if (i + 1 != result.solution.solutions.size()) out << ',';
        out << '\n';
    }
    out << "  ]\n}\n";
    return out.str();
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
    libgnss::FGOProcessor::FGOProblem problem = processor.buildPseudorangeProblem(epochs, nav);
    if (problem.epochs.size() < 2 || problem.pseudorange_factors.empty() ||
        problem.tdcp_factors.empty()) {
        std::cerr << "v3 problem lacks pseudorange or ordinary TDCP factors\n";
        return 1;
    }
    if (!problem.double_difference_pseudorange_factors.empty() ||
        !problem.double_difference_carrier_factors.empty()) {
        std::cerr << "v3 no-base contract violated by problem builder\n";
        return 1;
    }

    V3HeadingReport heading_report;
    V3OffsetReport offset_report;
    bool use_imu = buildHeadingOptionalImuInput(options.imu_path, problem, heading_report);
    bool fallback = false;
    libgnss::FGOProcessor::FGOResult result;
    if (use_imu) {
        const std::vector<libgnss::ImuSample> raw_samples = problem.imu.samples_body_flu;
        offset_report = v3EstimateOffset(problem, raw_samples);
        if (offset_report.confident) {
            for (auto& sample : problem.imu.samples_body_flu) {
                sample.time = sample.time + offset_report.selected_offset_s;
            }
            std::sort(problem.imu.samples_body_flu.begin(), problem.imu.samples_body_flu.end(),
                      [](const libgnss::ImuSample& lhs, const libgnss::ImuSample& rhs) {
                          return lhs.time < rhs.time;
                      });
            offset_report.applied = true;
        }
        try {
            result = processor.optimizeProblem(problem);
        } catch (const std::exception& error) {
            heading_report.failure_reason = std::string("IMU solver exception; exact v1 fallback: ") + error.what();
        } catch (...) {
            heading_report.failure_reason = "IMU solver exception; exact v1 fallback";
        }
        if (!v3FiniteSolution(result) || !result.diagnostics.converged ||
            result.diagnostics.imu_intervals == 0 ||
            result.diagnostics.tdcp_factors_inserted != problem.tdcp_factors.size()) {
            use_imu = false;
            fallback = true;
        }
    } else {
        fallback = true;
    }
    if (!use_imu) {
        fallback = true;
        problem.imu.valid = false;
        libgnss::FGOProcessor::FGOConfig fallback_config = config;
        fallback_config.use_imu = false;
        fallback_config.use_pose3_state = false;
        const libgnss::FGOProcessor fallback_processor(fallback_config);
        try {
            result = fallback_processor.optimizeProblem(problem);
        } catch (const std::exception& error) {
            std::cerr << "exact v1 fallback exception: " << error.what() << "\n";
            return 1;
        } catch (...) {
            std::cerr << "exact v1 fallback exception\n";
            return 1;
        }
    }
    if (!v3FiniteSolution(result) || !result.diagnostics.converged) {
        std::cerr << "v3 FGO and exact v1 fallback produced no finite converged output\n";
        return 1;
    }
    if (!problem.double_difference_pseudorange_factors.empty() ||
        !problem.double_difference_carrier_factors.empty() ||
        problem.single_difference_doppler_factors.size() != 0 ||
        problem.single_difference_tdcp_factors.size() != 0) {
        std::cerr << "v3 forbidden factor class was present\n";
        return 1;
    }

    std::ostringstream csv;
    csv << "phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n";
    std::size_t finite_rows = 0;
    for (const auto& solution : result.solution.solutions) {
        double lat_out = solution.position_geodetic.latitude;
        double lon_out = solution.position_geodetic.longitude;
        if (!std::isfinite(lat_out) || !std::isfinite(lon_out)) {
            double height_out = 0.0;
            libgnss::ecef2geodetic(solution.position_ecef, lat_out, lon_out, height_out);
        }
        const double timestamp = unixMillis(solution.time);
        if (!std::isfinite(timestamp) || !std::isfinite(lat_out) || !std::isfinite(lon_out) ||
            std::abs(lat_out) > kPi / 2.0 || std::abs(lon_out) > kPi) {
            std::cerr << "non-finite or out-of-range v3 output row\n";
            return 1;
        }
        csv << options.dataset_id << ',' << static_cast<long long>(timestamp) << ','
            << std::fixed << std::setprecision(10) << lat_out * kRadToDeg << ','
            << lon_out * kRadToDeg << '\n';
        ++finite_rows;
    }
    const std::string detail_path = options.summary_path + ".detail.json";
    if (finite_rows == 0 || !atomicWrite(options.out_path, csv.str()) ||
        !atomicWrite(detail_path, v3Detail(result))) {
        std::cerr << "failed to atomically publish v3 output/detail\n";
        return 1;
    }
    const std::string summary = v3Summary(options, problem, result, heading_report, fallback,
                                          offset_report, detail_path);
    if (!atomicWrite(options.summary_path, summary)) {
        std::cerr << "failed to atomically publish v3 summary\n";
        return 1;
    }
    std::cout << "native v3 heading-optional FGO: epochs=" << problem.epochs.size()
              << " output=" << finite_rows
              << " pseudorange=" << problem.pseudorange_factors.size()
              << " tdcp=" << problem.tdcp_factors.size()
              << " tdcp_inserted=" << result.diagnostics.tdcp_factors_inserted
              << " motion_pairs=" << result.diagnostics.motion_factors
              << " combined_imu=" << result.diagnostics.imu_intervals
              << " yaw_mode=" << heading_report.yaw_mode
              << " fallback=" << (fallback ? "yes" : "no") << '\n';
    return 0;
}
