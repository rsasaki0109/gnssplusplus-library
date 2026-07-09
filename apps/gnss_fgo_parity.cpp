// Real-data Eigen-vs-GTSAM FGO parity harness (Phase 1 validation).
//
// Loads a real tokyo/nagoya PPC run (rover.obs + base.obs + base.nav) exactly
// the way apps/gnss_fgo.cpp does, materializes ONE production FGOProblem via
// FGOProcessor::buildDoubleDifferenceProblem(), then runs that identical
// problem through BOTH FGOProcessor backends (FGOBackend::Eigen and
// FGOBackend::GTSAM) and reports:
//   (a) per-epoch ECEF float-solution delta (max / RMS),
//   (b) fixed-solution / fix agreement with LAMBDA enabled on both backends.
//
// This is an app (not a unit test) because it needs the real dataset files.
// It only builds when the library was configured with GTSAM (otherwise the
// GTSAM backend silently falls back to Eigen and the comparison is trivial).

#include <libgnss++/algorithms/fgo.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/observation.hpp>
#include <libgnss++/fusion/fusion_initialization.hpp>
#include <libgnss++/io/imu.hpp>
#include <libgnss++/io/rinex.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

struct Args {
    std::string rover_path;
    std::string base_path;
    std::string nav_path;
    int max_epochs = 0;      // 0 = all
    int max_iters = 0;       // 0 = use preset default
    bool no_pr_factors = false;  // pure double-difference path (clock-free)
    bool no_robust = false;      // disable Huber robust loss (convexity check)
    bool float_only = false;     // skip the LAMBDA/Marginals fix comparison
    std::string imu_path;        // milestone 2b: imu.csv -> IMU-coupled Pose3 run
    std::string ref_path;        // optional reference.csv for attitude sanity
    double fixed_lag_s = 0.0;    // milestone 2c: >0 enables the fixed-lag smoother
    bool use_nhc = false;        // milestone 2d
    bool use_zupt = false;       // milestone 2d
};

Args parseArgs(int argc, char** argv) {
    Args args;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--rover" && i + 1 < argc) {
            args.rover_path = argv[++i];
        } else if (a == "--base" && i + 1 < argc) {
            args.base_path = argv[++i];
        } else if (a == "--nav" && i + 1 < argc) {
            args.nav_path = argv[++i];
        } else if (a == "--max-epochs" && i + 1 < argc) {
            args.max_epochs = std::stoi(argv[++i]);
        } else if (a == "--max-iters" && i + 1 < argc) {
            args.max_iters = std::stoi(argv[++i]);
        } else if (a == "--no-pr") {
            args.no_pr_factors = true;
        } else if (a == "--no-robust") {
            args.no_robust = true;
        } else if (a == "--float-only") {
            args.float_only = true;
        } else if (a == "--imu" && i + 1 < argc) {
            args.imu_path = argv[++i];
        } else if (a == "--ref" && i + 1 < argc) {
            args.ref_path = argv[++i];
        } else if (a == "--fixed-lag" && i + 1 < argc) {
            args.fixed_lag_s = std::stod(argv[++i]);
        } else if (a == "--nhc") {
            args.use_nhc = true;
        } else if (a == "--zupt") {
            args.use_zupt = true;
        } else {
            std::cerr << "Unknown/incomplete arg: " << a << "\n";
            std::exit(2);
        }
    }
    if (args.rover_path.empty() || args.base_path.empty() || args.nav_path.empty()) {
        std::cerr << "Usage: gnss_fgo_parity --rover <rover.obs> --base <base.obs> "
                     "--nav <base.nav> [--max-epochs N]\n";
        std::exit(2);
    }
    return args;
}

// Mirrors apps/gnss_fgo.cpp's "real-data-float" preset for the DD RTK path so
// the FGOProblem we build matches a realistic production configuration.
libgnss::FGOProcessor::FGOConfig makeRealDataDdConfig() {
    libgnss::FGOProcessor::FGOConfig config;
    config.max_iterations = 12;
    config.pseudorange_sigma_m = 4.0;
    config.motion_sigma_m = 100.0;
    config.clock_motion_sigma_m = 600.0;
    config.tdcp_sigma_m = 0.05;
    config.carrier_phase_sigma_m = 0.02;
    config.double_difference_pseudorange_sigma_m = 1.0;
    config.double_difference_carrier_sigma_m = 0.02;
    config.ambiguity_prior_sigma_m = 2000.0;
    config.pseudorange_huber_threshold_sigma = 3.0;
    config.carrier_phase_huber_threshold_sigma = 3.0;
    config.tdcp_huber_threshold_sigma = 3.0;
    config.max_tdcp_gap_s = 1.5;
    config.min_elevation_deg = 15.0;
    config.use_robust_loss = true;
    config.use_double_difference_factors = true;
    config.use_pseudorange_factors = true;
    config.use_carrier_phase_factors = false;
    config.use_tdcp_factors = true;
    config.use_ambiguity_priors = true;
    config.fix_ambiguities = false;
    // Per-epoch LAMBDA on this urban multipath data tops out around ratio ~2.5;
    // the default 3.0 gate fixes nothing on EITHER backend's per-epoch path
    // (Eigen only "fixes" via its lenient global nearest-integer snap). Use 2.0
    // so both backends' per-epoch LAMBDA actually engage and can be compared.
    config.lambda_ratio_threshold = 2.0;
    config.use_inter_system_biases = true;
    return config;
}

std::vector<libgnss::ObservationData> loadEpochs(const std::string& path,
                                                 int max_epochs,
                                                 const libgnss::Vector3d& fixed_position,
                                                 bool assign_fixed_position) {
    libgnss::io::RINEXReader reader;
    std::vector<libgnss::ObservationData> epochs;
    if (!reader.open(path)) {
        std::cerr << "Error: cannot open " << path << "\n";
        std::exit(1);
    }
    libgnss::io::RINEXReader::RINEXHeader header;
    if (!reader.readHeader(header)) {
        std::cerr << "Error: cannot read header " << path << "\n";
        std::exit(1);
    }
    libgnss::ObservationData epoch;
    while (reader.readObservationEpoch(epoch)) {
        if (assign_fixed_position) {
            epoch.receiver_position = fixed_position;
        } else if (header.approximate_position.norm() > 0.0) {
            epoch.receiver_position = header.approximate_position;
        }
        epochs.push_back(epoch);
        if (max_epochs > 0 && static_cast<int>(epochs.size()) >= max_epochs) {
            break;
        }
    }
    return epochs;
}

struct FloatParity {
    std::size_t compared_epochs = 0;
    double max_delta_m = 0.0;
    double rms_delta_m = 0.0;
    std::size_t nonfinite_eigen = 0;
    std::size_t nonfinite_gtsam = 0;
    std::size_t status_disagreements = 0;
};

const char* statusName(libgnss::SolutionStatus s) {
    switch (s) {
        case libgnss::SolutionStatus::NONE: return "NONE";
        case libgnss::SolutionStatus::SPP: return "SPP";
        case libgnss::SolutionStatus::FLOAT: return "FLOAT";
        case libgnss::SolutionStatus::FIXED: return "FIXED";
        default: return "?";
    }
}

FloatParity compareFloat(const libgnss::FGOProcessor::FGOResult& e,
                         const libgnss::FGOProcessor::FGOResult& g) {
    FloatParity fp;
    const std::size_t n = std::min(e.solution.solutions.size(), g.solution.solutions.size());
    double sum_sq = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const auto& es = e.solution.solutions[i];
        const auto& gs = g.solution.solutions[i];
        if (es.status != gs.status) {
            ++fp.status_disagreements;
        }
        const bool ef = es.position_ecef.allFinite();
        const bool gf = gs.position_ecef.allFinite();
        if (!ef) ++fp.nonfinite_eigen;
        if (!gf) ++fp.nonfinite_gtsam;
        // Only compare epochs where BOTH produced a usable (non-NONE) finite fix.
        if (!ef || !gf || es.status == libgnss::SolutionStatus::NONE ||
            gs.status == libgnss::SolutionStatus::NONE) {
            continue;
        }
        const double d = (es.position_ecef - gs.position_ecef).norm();
        fp.max_delta_m = std::max(fp.max_delta_m, d);
        sum_sq += d * d;
        ++fp.compared_epochs;
    }
    if (fp.compared_epochs > 0) {
        fp.rms_delta_m = std::sqrt(sum_sq / static_cast<double>(fp.compared_epochs));
    }
    return fp;
}

std::size_t countFixedEpochs(const libgnss::FGOProcessor::FGOResult& r) {
    std::size_t n = 0;
    for (const auto& s : r.solution.solutions) {
        if (s.status == libgnss::SolutionStatus::FIXED) {
            ++n;
        }
    }
    return n;
}

// Tokyo IMU->antenna lever arm, body FLU (docs/imu_fusion.md, project memory);
// used only when the caller asks for the milestone-2a Pose3 GTSAM path.
constexpr double kTokyoLeverArmX = 0.31;
constexpr double kTokyoLeverArmY = 0.0;
constexpr double kTokyoLeverArmZ = 0.55;

libgnss::FGOProcessor::FGOResult run(const libgnss::FGOProcessor::FGOProblem& problem,
                                     libgnss::FGOProcessor::FGOConfig config,
                                     libgnss::FGOBackend backend,
                                     bool use_lambda,
                                     double& seconds,
                                     bool use_pose3 = false,
                                     bool use_imu = false,
                                     double fixed_lag_s = 0.0,
                                     bool use_nhc = false,
                                     bool use_zupt = false) {
    config.backend = backend;
    config.use_lambda_ambiguity_fix = use_lambda;
    config.fix_ambiguities = use_lambda;
    // Drive BOTH backends through their per-epoch LAMBDA path (fixed-position
    // output) so the fix comparison is apples-to-apples: the native backend's
    // per-epoch fixing and the GTSAM backend's per-epoch fixing both mark
    // epochs FIXED and snap positions to the fixed ambiguities.
    config.use_epoch_lambda_fixed_output = use_lambda;
    config.collect_lambda_debug = false;
    if (use_pose3) {
        // Milestone 2a (docs/gtsam_backend_design.md): key the GTSAM rover
        // state as a body Pose3 + lever arm instead of a bare Point3.
        config.use_pose3_state = true;
        config.pose3_lever_arm_body_m =
            libgnss::Vector3d(kTokyoLeverArmX, kTokyoLeverArmY, kTokyoLeverArmZ);
    }
    // Milestone 2b: enable IMU tight coupling (the problem must already carry
    // a valid ImuInput; the backend checks problem.imu.valid).
    config.use_imu = use_imu;
    // Milestone 2c: enable the incremental fixed-lag smoother.
    if (fixed_lag_s > 0.0) {
        config.use_fixed_lag_smoother = true;
        config.fixed_lag_smoother_lag_s = fixed_lag_s;
    }
    // Milestone 2d: NHC / ZUPT pseudo-measurements.
    config.use_nhc = use_nhc;
    config.use_zupt = use_zupt;
    libgnss::FGOProcessor processor(config);
    const auto t0 = std::chrono::high_resolution_clock::now();
    auto result = processor.optimizeProblem(problem);
    const auto t1 = std::chrono::high_resolution_clock::now();
    seconds = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - t0).count();
    return result;
}

// Per-epoch ECEF delta between two results restricted to epochs both mark FIXED.
struct FixedDelta {
    std::size_t both_fixed = 0;
    double max_delta_m = 0.0;
    double rms_delta_m = 0.0;
};

FixedDelta compareFixedPositions(const libgnss::FGOProcessor::FGOResult& e,
                                 const libgnss::FGOProcessor::FGOResult& g) {
    FixedDelta fd;
    const std::size_t n = std::min(e.solution.solutions.size(), g.solution.solutions.size());
    double sum_sq = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const auto& es = e.solution.solutions[i];
        const auto& gs = g.solution.solutions[i];
        if (es.status != libgnss::SolutionStatus::FIXED ||
            gs.status != libgnss::SolutionStatus::FIXED ||
            !es.position_ecef.allFinite() || !gs.position_ecef.allFinite()) {
            continue;
        }
        const double d = (es.position_ecef - gs.position_ecef).norm();
        fd.max_delta_m = std::max(fd.max_delta_m, d);
        sum_sq += d * d;
        ++fd.both_fixed;
    }
    if (fd.both_fixed > 0) {
        fd.rms_delta_m = std::sqrt(sum_sq / static_cast<double>(fd.both_fixed));
    }
    return fd;
}

// --- Milestone 2b IMU plumbing (harness side) ---
//
// Loads imu.csv (already-FLU raw axes, gyro converted to rad/s by loadImuCsv),
// runs the SAME Stage-1 ESKF alignment used by `gnss fuse` to get the initial
// body->ENU attitude + gyro/accel bias, latches heading off the first GNSS
// motion, and packages everything into FGOProblem::ImuInput. Keeping this in
// the harness (rather than the backend) means the GTSAM backend depends only
// on the packaged ImuInput, not on the fusion/ alignment module.
struct RefRow {
    libgnss::GNSSTime time;
    double roll_deg = 0.0, pitch_deg = 0.0, heading_deg = 0.0;
    double ve = 0.0, vn = 0.0, vu = 0.0;
    libgnss::Vector3d ecef = libgnss::Vector3d::Zero();
};

std::vector<RefRow> loadReference(const std::string& path) {
    std::vector<RefRow> rows;
    std::ifstream in(path);
    if (!in) return rows;
    std::string line;
    std::getline(in, line);  // header
    while (std::getline(in, line)) {
        std::stringstream ss(line);
        std::string cell;
        std::vector<double> v;
        while (std::getline(ss, cell, ',')) {
            try { v.push_back(std::stod(cell)); } catch (...) { v.push_back(0.0); }
        }
        if (v.size() < 14) continue;
        RefRow r;
        r.time = libgnss::GNSSTime(static_cast<int>(v[1]), v[0]);
        r.ecef = libgnss::Vector3d(v[5], v[6], v[7]);
        r.roll_deg = v[8]; r.pitch_deg = v[9]; r.heading_deg = v[10];
        r.ve = v[11]; r.vn = v[12]; r.vu = v[13];
        rows.push_back(r);
    }
    return rows;
}

// Horizontal (E/N) position error of a result vs reference.csv, split by
// FLOAT vs FIXED solution status. Matches epochs to reference rows by time.
struct HorizError {
    std::size_t n_float = 0, n_fixed = 0;
    double float_rms = 0.0, float_max = 0.0;
    double fixed_rms = 0.0, fixed_max = 0.0;
    double frac_all_under_50cm = 0.0;  ///< fraction of all (float+fixed) epochs with horiz<0.5 m
};

HorizError horizontalErrorVsRef(const libgnss::FGOProcessor::FGOResult& r,
                                const std::vector<RefRow>& ref) {
    HorizError he;
    if (ref.empty()) return he;
    double lat0 = 0.0, lon0 = 0.0, h0 = 0.0;
    libgnss::ecef2geodetic(ref.front().ecef, lat0, lon0, h0);
    double fsum = 0.0, xsum = 0.0;
    std::size_t ri = 0;  // reference cursor (both streams time-sorted)
    std::size_t n_all = 0, n_all_under50 = 0;
    for (const auto& s : r.solution.solutions) {
        if (s.status == libgnss::SolutionStatus::NONE || !s.position_ecef.allFinite()) continue;
        // Advance reference cursor to the nearest row in time.
        while (ri + 1 < ref.size() &&
               std::abs(ref[ri + 1].time - s.time) < std::abs(ref[ri].time - s.time)) {
            ++ri;
        }
        if (std::abs(ref[ri].time - s.time) > 0.11) continue;
        const libgnss::Vector3d enu =
            libgnss::ecef2enu(s.position_ecef - ref[ri].ecef, lat0, lon0);
        const double horiz = std::hypot(enu.x(), enu.y());
        ++n_all;
        if (horiz < 0.5) ++n_all_under50;
        if (s.status == libgnss::SolutionStatus::FIXED) {
            xsum += horiz * horiz;
            he.fixed_max = std::max(he.fixed_max, horiz);
            ++he.n_fixed;
        } else {
            fsum += horiz * horiz;
            he.float_max = std::max(he.float_max, horiz);
            ++he.n_float;
        }
    }
    if (he.n_float > 0) he.float_rms = std::sqrt(fsum / double(he.n_float));
    if (he.n_fixed > 0) he.fixed_rms = std::sqrt(xsum / double(he.n_fixed));
    if (n_all > 0) he.frac_all_under_50cm = double(n_all_under50) / double(n_all);
    return he;
}

// Populate problem.imu from an imu.csv + the already-built FGOProblem epochs.
// Returns false (and leaves problem.imu.valid=false) if anything is missing.
bool buildImuInput(const std::string& imu_path,
                   libgnss::FGOProcessor::FGOProblem& problem) {
    if (problem.epochs.size() < 2) return false;
    libgnss::ImuSeries imu_series;
    const auto load = libgnss::loadImuCsv(imu_path, imu_series);
    if (!load.ok || imu_series.isEmpty()) {
        std::cerr << "Error: cannot load IMU csv " << imu_path
                  << " (" << load.error << ")\n";
        return false;
    }
    // tokyo imu.csv is already body FLU (identity axis convention, same default
    // as gnss_fuse.cpp) with gyro converted to rad/s at load time.
    const libgnss::ImuAxisConvention axis;  // identity
    for (auto& s : imu_series.samples) {
        s.accel_raw = axis.apply(s.accel_raw);
        s.gyro_raw_radps = axis.apply(s.gyro_raw_radps);
    }

    auto& imu = problem.imu;
    imu.samples_body_flu = imu_series.samples;

    // Nav (ENU) origin: first epoch antenna ECEF -> geodetic lat/lon.
    const libgnss::Vector3d origin_ecef = problem.epochs.front().position_ecef;
    double lat = 0.0, lon = 0.0, h = 0.0;
    libgnss::ecef2geodetic(origin_ecef, lat, lon, h);
    imu.nav_origin_ecef = origin_ecef;
    imu.nav_origin_lat_rad = lat;
    imu.nav_origin_lon_rad = lon;

    // Stage-1 static leveling, reusing fusion_initialization::alignStatic.
    // The tokyo run starts stationary (reference velocity ~0 for the first
    // seconds) and the IMU stream begins essentially concurrent with the first
    // GNSS epoch, so the leveling window is the first ~2.5 s of IMU samples at
    // or after the first epoch time (samples strictly "before" the first epoch
    // do not exist). A short leading gyro/accel-quiet check guards against
    // starting mid-motion.
    const libgnss::GNSSTime t_first = problem.epochs.front().time;
    std::vector<libgnss::ImuSample> stationary;
    for (const auto& s : imu.samples_body_flu) {
        if (s.time < t_first) continue;
        if (stationary.size() >= 250) break;  // ~2.5 s at 100 Hz
        stationary.push_back(s);
    }
    const libgnss::NominalState aligned = libgnss::fusion_initialization::alignStatic(
        stationary, libgnss::Vector3d::Zero(), imu.noise.gravity_mps2);

    // Heading latch: first epoch whose GNSS ENU speed exceeds threshold sets
    // yaw = course (mirrors fusion_initialization::tryAlignHeading). Compute
    // ENU velocity by finite difference of consecutive antenna positions.
    libgnss::FusionState fstate;
    fstate.nominal = aligned;
    libgnss::Vector3d init_vel_enu = libgnss::Vector3d::Zero();
    bool heading_latched = false;
    for (std::size_t i = 0; i + 1 < problem.epochs.size(); ++i) {
        const double dt = problem.epochs[i + 1].time - problem.epochs[i].time;
        if (dt <= 1e-3) continue;
        const libgnss::Vector3d enu0 = libgnss::ecef2enu(
            problem.epochs[i].position_ecef - origin_ecef, lat, lon);
        const libgnss::Vector3d enu1 = libgnss::ecef2enu(
            problem.epochs[i + 1].position_ecef - origin_ecef, lat, lon);
        const libgnss::Vector3d vel = (enu1 - enu0) / dt;
        if (!heading_latched) {
            if (libgnss::fusion_initialization::tryAlignHeading(fstate, vel, 1.0, 5.0)) {
                heading_latched = true;
                init_vel_enu = vel;
            }
        }
    }
    // If never moved fast enough in the batch, fall back to first-interval vel.
    if (!heading_latched && problem.epochs.size() >= 2) {
        const double dt = problem.epochs[1].time - problem.epochs[0].time;
        if (dt > 1e-3) {
            init_vel_enu = libgnss::ecef2enu(
                problem.epochs[1].position_ecef - origin_ecef, lat, lon) / dt -
                libgnss::ecef2enu(problem.epochs[0].position_ecef - origin_ecef, lat, lon) / dt;
        }
    }

    imu.init_attitude_body_to_nav = fstate.nominal.attitude_body_to_enu.toRotationMatrix();
    imu.init_velocity_nav = init_vel_enu;
    imu.init_accel_bias = aligned.accel_bias;
    imu.init_gyro_bias = aligned.gyro_bias;
    // Low-cost MEMS-ish noise (order of the tokyo low-cost preset); 2b is a
    // wiring/sanity gate, tuning is 2c/2e.
    // Tighter than the initial 2b sanity setting so the IMU actually bridges
    // weak-GNSS urban stretches (still conservative MEMS-grade; deep tuning is
    // 2e). Loose IMU let the float wander to metres in canyon; tightening pulls
    // it back toward the IMU-predicted trajectory between good fixes.
    imu.noise.accel_noise_sigma = 2.0e-2;
    imu.noise.gyro_noise_sigma = 2.0e-3;
    imu.noise.accel_bias_rw_sigma = 3.0e-3;
    imu.noise.gyro_bias_rw_sigma = 3.0e-4;
    imu.noise.integration_sigma = 1.0e-3;
    imu.init_attitude_sigma_roll_pitch_rad = 0.05;
    imu.init_attitude_sigma_yaw_rad = heading_latched ? 0.15 : 3.0;
    imu.init_velocity_sigma_mps = 0.5;
    imu.init_accel_bias_sigma = 0.1;
    imu.init_gyro_bias_sigma = 0.01;
    imu.valid = true;

    {
        // Epoch/IMU cadence sanity (helps diagnose preintegration dt issues).
        const double dt_epoch = problem.epochs.size() >= 2
                                    ? (problem.epochs[1].time - problem.epochs[0].time)
                                    : 0.0;
        double dt_imu = 0.0;
        for (std::size_t k = 1; k < imu.samples_body_flu.size() && k < 5; ++k) {
            dt_imu = imu.samples_body_flu[k].time - imu.samples_body_flu[k - 1].time;
        }
        std::cout << "IMU cadence: epoch dt=" << dt_epoch << " s, imu dt~=" << dt_imu
                  << " s; epoch0.tow=" << problem.epochs[0].time.tow
                  << " imu0.tow=" << imu.samples_body_flu[0].time.tow
                  << " (weeks e" << problem.epochs[0].time.week << "/i"
                  << imu.samples_body_flu[0].time.week << ")\n";
    }
    std::cout << "IMU: loaded " << imu.samples_body_flu.size() << " samples, stationary_window="
              << stationary.size() << ", heading_latched=" << (heading_latched ? "yes" : "no")
              << "\n  init attitude(body->ENU) roll/pitch from leveling; init_vel_enu=["
              << init_vel_enu.transpose() << "] m/s\n"
              << "  init_gyro_bias=[" << aligned.gyro_bias.transpose() << "] rad/s"
              << " init_accel_bias=[" << aligned.accel_bias.transpose() << "] m/s^2\n";
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    const Args args = parseArgs(argc, argv);

    // Navigation.
    libgnss::io::RINEXReader nav_reader;
    if (!nav_reader.open(args.nav_path)) {
        std::cerr << "Error: cannot open nav " << args.nav_path << "\n";
        return 1;
    }
    libgnss::NavigationData nav;
    if (!nav_reader.readNavigationData(nav)) {
        std::cerr << "Error: cannot read nav " << args.nav_path << "\n";
        return 1;
    }

    // Base station position from its RINEX header (mirrors gnss_fgo.cpp).
    libgnss::io::RINEXReader base_header_reader;
    if (!base_header_reader.open(args.base_path)) {
        std::cerr << "Error: cannot open base " << args.base_path << "\n";
        return 1;
    }
    libgnss::io::RINEXReader::RINEXHeader base_header;
    if (!base_header_reader.readHeader(base_header)) {
        std::cerr << "Error: cannot read base header " << args.base_path << "\n";
        return 1;
    }
    if (base_header.approximate_position.norm() <= 1e6) {
        std::cerr << "Error: base approximate position unavailable in " << args.base_path << "\n";
        return 1;
    }
    const libgnss::Vector3d base_position = base_header.approximate_position;

    const std::vector<libgnss::ObservationData> rover_epochs =
        loadEpochs(args.rover_path, args.max_epochs, libgnss::Vector3d::Zero(), false);
    // Base is read fully (its epochs are matched/interpolated to rover time
    // inside buildDoubleDifferenceProblem); no rover-side epoch cap on base.
    const std::vector<libgnss::ObservationData> base_epochs =
        loadEpochs(args.base_path, 0, base_position, true);

    std::cout << "Dataset:\n"
              << "  rover=" << args.rover_path << " (" << rover_epochs.size() << " epochs"
              << (args.max_epochs > 0 ? " capped" : "") << ")\n"
              << "  base=" << args.base_path << " (" << base_epochs.size() << " epochs)\n"
              << "  nav=" << args.nav_path << "\n"
              << "  base_pos_ecef=[" << base_position.transpose() << "]\n";

    libgnss::FGOProcessor::FGOConfig config = makeRealDataDdConfig();
    if (args.max_iters > 0) {
        config.max_iterations = args.max_iters;
    }
    if (args.no_robust) {
        config.use_robust_loss = false;
    }
    if (args.no_pr_factors) {
        // Pure DD path: no undifferenced pseudorange factors, so there are no
        // receiver-clock / inter-system-bias states at all -- this isolates the
        // clock-free double-difference RTK path (the Phase-1 target) from the
        // undifferenced-pseudorange modeling differences between backends.
        config.use_pseudorange_factors = false;
    }
    const libgnss::FGOProcessor builder(config);
    const libgnss::FGOProcessor::FGOProblem problem =
        builder.buildDoubleDifferenceProblem(rover_epochs, base_epochs, nav, base_position);

    std::cout << "FGOProblem: epochs=" << problem.epochs.size()
              << " dd_pr_factors=" << problem.double_difference_pseudorange_factors.size()
              << " dd_cp_factors=" << problem.double_difference_carrier_factors.size()
              << " ambiguity_states=" << problem.ambiguity_states.size()
              << " pr_factors=" << problem.pseudorange_factors.size() << "\n";

    if (problem.epochs.empty() ||
        (problem.double_difference_carrier_factors.empty() &&
         problem.double_difference_pseudorange_factors.empty())) {
        std::cerr << "Error: no DD factors built; cannot compare backends.\n";
        return 1;
    }

    // --- (a4) MILESTONE 2c: IncrementalFixedLagSmoother at scale. Runs FIRST
    // and returns, so the full run does NOT pay for the heavy batch (a)/(a2)/(b)
    // DD comparisons (whose full-batch gtsam::Marginals is exactly what 2c
    // avoids). Requires --imu + --fixed-lag S. ---
    if (args.fixed_lag_s > 0.0 && !args.imu_path.empty()) {
        libgnss::FGOProcessor::FGOProblem problem_imu = problem;
        if (!buildImuInput(args.imu_path, problem_imu)) {
            std::cerr << "Error: could not build IMU input for fixed-lag run.\n";
            return 1;
        }
        std::vector<RefRow> ref_rows;
        if (!args.ref_path.empty()) ref_rows = loadReference(args.ref_path);

        double t_fl = 0.0;
        const auto fl = run(problem_imu, config, libgnss::FGOBackend::GTSAM, true, t_fl,
                            /*use_pose3=*/true, /*use_imu=*/true, args.fixed_lag_s,
                            args.use_nhc, args.use_zupt);
        std::size_t nonfinite = 0, none_epochs = 0;
        for (const auto& s : fl.solution.solutions) {
            if (s.status == libgnss::SolutionStatus::NONE) ++none_epochs;
            else if (!s.position_ecef.allFinite()) ++nonfinite;
        }
        const std::size_t fl_fixed = countFixedEpochs(fl);
        const std::size_t ne = fl.solution.solutions.size();
        const HorizError he = horizontalErrorVsRef(fl, ref_rows);

        std::cout << "\n=== (a4) MILESTONE 2c: IncrementalFixedLagSmoother (full-scale) ===\n"
                  << "  lag=" << args.fixed_lag_s << " s, epochs=" << ne
                  << ", smoother_updates=" << fl.diagnostics.smoother_updates
                  << ", peak_window_vars=" << fl.diagnostics.smoother_max_window_vars << "\n"
                  << "  wall_clock=" << t_fl << " s ("
                  << (ne > 0 ? t_fl / double(ne) * 1e3 : 0.0)
                  << " ms/epoch), nonfinite=" << nonfinite << ", NONE_epochs=" << none_epochs << "\n"
                  << "  per-epoch LAMBDA: attempts=" << fl.diagnostics.lambda_ambiguity_attempts
                  << ", fixed_epochs=" << fl_fixed << "/" << ne << " ("
                  << (ne > 0 ? 100.0 * double(fl_fixed) / double(ne) : 0.0)
                  << "% fix-rate), best_ratio=" << fl.diagnostics.lambda_ambiguity_ratio << "\n"
                  << "  NHC/ZUPT: nhc=" << (args.use_nhc ? "on" : "off")
                  << " (applied " << fl.diagnostics.nhc_epochs << " epochs), zupt="
                  << (args.use_zupt ? "on" : "off") << " (applied " << fl.diagnostics.zupt_epochs
                  << " epochs)\n";
        if (!ref_rows.empty()) {
            std::cout << "  horizontal error vs reference.csv:\n"
                      << "    FLOAT: n=" << he.n_float << " rms=" << he.float_rms
                      << " m max=" << he.float_max << " m\n"
                      << "    FIXED: n=" << he.n_fixed << " rms=" << he.fixed_rms
                      << " m max=" << he.fixed_max << " m\n"
                      << "    ALL (float+fixed) <50cm rate=" << (100.0 * he.frac_all_under_50cm)
                      << "%\n"
                      << "    (inuex35 truth target run1: FixRMS 0.815 m / fix 49.5% / <50cm 56.7%)\n";
            // Stationary-epoch velocity: should collapse to ~0 with ZUPT.
            if (!fl.epoch_velocity_nav_mps.empty()) {
                double vsum = 0.0, vmax = 0.0;
                std::size_t vn = 0, ri = 0;
                for (std::size_t i = 0; i < ne && i < fl.epoch_velocity_nav_mps.size(); ++i) {
                    const auto& sol = fl.solution.solutions[i];
                    while (ri + 1 < ref_rows.size() &&
                           std::abs(ref_rows[ri + 1].time - sol.time) <
                               std::abs(ref_rows[ri].time - sol.time)) {
                        ++ri;
                    }
                    if (std::abs(ref_rows[ri].time - sol.time) > 0.11) continue;
                    const double ref_speed = std::hypot(ref_rows[ri].ve, ref_rows[ri].vn);
                    if (ref_speed > 0.1) continue;  // reference-stationary epochs only
                    const auto& v = fl.epoch_velocity_nav_mps[i];
                    const double sp = v.norm();
                    vsum += sp * sp;
                    vmax = std::max(vmax, sp);
                    ++vn;
                }
                if (vn > 0) {
                    std::cout << "    stationary-epoch |velocity| (ref speed<0.1): rms="
                              << std::sqrt(vsum / double(vn)) << " m/s max=" << vmax
                              << " m/s (n=" << vn << ")\n";
                }
            }
        }
        // Consistency vs 2b batch on the overlap (only when small enough that
        // the batch path is safe). Apples-to-apples: both float-only.
        if (ne <= 600) {
            double t_b = 0.0, t_flf = 0.0;
            const auto batch = run(problem_imu, config, libgnss::FGOBackend::GTSAM, false, t_b,
                                   /*use_pose3=*/true, /*use_imu=*/true);
            const auto fl_float = run(problem_imu, config, libgnss::FGOBackend::GTSAM, false, t_flf,
                                      /*use_pose3=*/true, /*use_imu=*/true, args.fixed_lag_s);
            const FloatParity cons = compareFloat(batch, fl_float);
            const HorizError he_batch = horizontalErrorVsRef(batch, ref_rows);
            const HorizError he_flf = horizontalErrorVsRef(fl_float, ref_rows);
            std::cout << "  consistency vs 2b batch (float-vs-float, over " << cons.compared_epochs
                      << " epochs): max=" << cons.max_delta_m << " m rms=" << cons.rms_delta_m << " m\n"
                      << "    batch-float horiz err vs ref:    rms=" << he_batch.float_rms << " m\n"
                      << "    smoother-float horiz err vs ref: rms=" << he_flf.float_rms << " m\n";
        }
        const bool go_2c = fl.diagnostics.smoother_updates == ne && nonfinite == 0 &&
                           fl.diagnostics.lambda_ambiguity_attempts > 0;
        std::cout << "\nRESULT (milestone 2c): fixed-lag full-scale run "
                  << (go_2c ? "GO" : "NO-GO") << " (epochs=" << ne
                  << ", peak_window_vars=" << fl.diagnostics.smoother_max_window_vars
                  << ", fix-rate=" << (ne > 0 ? 100.0 * double(fl_fixed) / double(ne) : 0.0)
                  << "%, wall=" << t_fl << " s)\n";
        std::cout.flush();
        return 0;
    }

    // --- (a) Float parity: LAMBDA off on both backends ---
    double t_ef = 0.0, t_gf = 0.0;
    const auto eigen_float = run(problem, config, libgnss::FGOBackend::Eigen, false, t_ef);
    const auto gtsam_float = run(problem, config, libgnss::FGOBackend::GTSAM, false, t_gf);
    const FloatParity fp = compareFloat(eigen_float, gtsam_float);

    std::cout << "\n=== (a) FLOAT parity (LAMBDA off) ===\n"
              << "  eigen: " << t_ef << " s, final_cost=" << eigen_float.diagnostics.final_cost
              << ", dd_cp_rms=" << eigen_float.diagnostics.double_difference_carrier_residual_rms_m
              << " m\n"
              << "  gtsam: " << t_gf << " s, iters=" << gtsam_float.diagnostics.iterations
              << ", final_cost=" << gtsam_float.diagnostics.final_cost
              << ", dd_cp_rms=" << gtsam_float.diagnostics.double_difference_carrier_residual_rms_m
              << " m, graph_values=" << gtsam_float.diagnostics.graph_values
              << " graph_factors=" << gtsam_float.diagnostics.graph_factors << "\n"
              << "  compared_epochs=" << fp.compared_epochs << "\n"
              << "  max ECEF delta = " << fp.max_delta_m << " m\n"
              << "  rms ECEF delta = " << fp.rms_delta_m << " m\n"
              << "  status_disagreements=" << fp.status_disagreements
              << " nonfinite(eigen=" << fp.nonfinite_eigen << ", gtsam=" << fp.nonfinite_gtsam
              << ")\n";
    std::cout.flush();

    // --- (a2) Milestone 2a: Pose3-GTSAM (lever-arm '...FactorArm' DD factors)
    // vs Point3-GTSAM antenna position parity, both with LAMBDA off. Attitude
    // is unobservable without IMU (2b), so only the recovered ANTENNA
    // position is validated here; the 2a success gate is Pose3 vs Point3
    // sub-cm agreement (docs/gtsam_backend_design.md, milestone 2a). ---
    double t_gp3 = 0.0;
    const auto gtsam_pose3_float =
        run(problem, config, libgnss::FGOBackend::GTSAM, false, t_gp3, /*use_pose3=*/true);
    const FloatParity fp_pose3_vs_point3 = compareFloat(gtsam_float, gtsam_pose3_float);
    const FloatParity fp_pose3_vs_eigen = compareFloat(eigen_float, gtsam_pose3_float);

    std::cout << "\n=== (a2) MILESTONE 2a: Pose3-GTSAM vs Point3-GTSAM antenna position parity ===\n"
              << "  lever_arm_body_flu=[" << kTokyoLeverArmX << ", " << kTokyoLeverArmY << ", "
              << kTokyoLeverArmZ << "] m\n"
              << "  pose3-gtsam: " << t_gp3 << " s, iters=" << gtsam_pose3_float.diagnostics.iterations
              << ", final_cost=" << gtsam_pose3_float.diagnostics.final_cost
              << ", dd_cp_rms=" << gtsam_pose3_float.diagnostics.double_difference_carrier_residual_rms_m
              << " m\n"
              << "  compared_epochs=" << fp_pose3_vs_point3.compared_epochs << "\n"
              << "  Pose3 vs Point3 (both GTSAM) max ECEF delta = " << fp_pose3_vs_point3.max_delta_m
              << " m\n"
              << "  Pose3 vs Point3 (both GTSAM) rms ECEF delta = " << fp_pose3_vs_point3.rms_delta_m
              << " m\n"
              << "  Pose3 (GTSAM) vs Eigen      max ECEF delta = " << fp_pose3_vs_eigen.max_delta_m
              << " m\n"
              << "  Pose3 (GTSAM) vs Eigen      rms ECEF delta = " << fp_pose3_vs_eigen.rms_delta_m
              << " m\n"
              << "  status_disagreements=" << fp_pose3_vs_point3.status_disagreements
              << " nonfinite(point3=" << fp_pose3_vs_point3.nonfinite_eigen
              << ", pose3=" << fp_pose3_vs_point3.nonfinite_gtsam << ")\n";
    std::cout.flush();

    const bool go_2a = fp_pose3_vs_point3.compared_epochs > 0 && fp_pose3_vs_point3.max_delta_m < 0.01;
    std::cout << "\nRESULT (milestone 2a): Pose3-vs-Point3 antenna-position parity "
              << (go_2a ? "GO" : "NO-GO") << " (max=" << fp_pose3_vs_point3.max_delta_m
              << " m rms=" << fp_pose3_vs_point3.rms_delta_m << " m over "
              << fp_pose3_vs_point3.compared_epochs << " epochs)\n";
    std::cout.flush();

    // --- IMU-coupled paths (2b batch or 2c fixed-lag). Attach IMU once. ---
    if (!args.imu_path.empty()) {
        libgnss::FGOProcessor::FGOProblem problem_imu = problem;
        if (buildImuInput(args.imu_path, problem_imu)) {
            std::vector<RefRow> ref_rows;
            if (!args.ref_path.empty()) ref_rows = loadReference(args.ref_path);

            // ============================================================
            // (a3) MILESTONE 2b: IMU-coupled batch Pose3 (CombinedImuFactor).
            // (Fixed-lag 2c is handled earlier, before section (a).)
            // ============================================================
            double t_imu = 0.0;
            const auto gtsam_imu =
                run(problem_imu, config, libgnss::FGOBackend::GTSAM, false, t_imu,
                    /*use_pose3=*/true, /*use_imu=*/true);
            const FloatParity fp_imu_vs_pose3 = compareFloat(gtsam_pose3_float, gtsam_imu);
            const FloatParity fp_imu_vs_eigen = compareFloat(eigen_float, gtsam_imu);

            // Divergence / NaN check.
            std::size_t nonfinite = 0;
            for (const auto& s : gtsam_imu.solution.solutions) {
                if (!s.position_ecef.allFinite()) ++nonfinite;
            }

            std::cout << "\n=== (a3) MILESTONE 2b: IMU-coupled Pose3 (CombinedImuFactor) ===\n"
                      << "  imu: " << t_imu << " s, iters=" << gtsam_imu.diagnostics.iterations
                      << ", imu_intervals=" << gtsam_imu.diagnostics.imu_intervals
                      << ", initial_cost=" << gtsam_imu.diagnostics.initial_cost
                      << ", final_cost=" << gtsam_imu.diagnostics.final_cost
                      << ", graph_values=" << gtsam_imu.diagnostics.graph_values
                      << ", graph_factors=" << gtsam_imu.diagnostics.graph_factors << "\n"
                      << "  solve_ok=" << (gtsam_imu.diagnostics.converged ? "yes" : "no")
                      << ", nonfinite_epochs=" << nonfinite << "\n"
                      << "  IMU-Pose3 vs 2a-Pose3 (no IMU) antenna delta: max="
                      << fp_imu_vs_pose3.max_delta_m << " m rms=" << fp_imu_vs_pose3.rms_delta_m
                      << " m (over " << fp_imu_vs_pose3.compared_epochs << " epochs)\n"
                      << "  IMU-Pose3 vs Eigen antenna delta:            max="
                      << fp_imu_vs_eigen.max_delta_m << " m rms=" << fp_imu_vs_eigen.rms_delta_m
                      << " m\n";

            // Attitude observability + sanity (vs reference.csv if provided).
            const std::vector<RefRow>& ref = ref_rows;
            auto refAt = [&](const libgnss::GNSSTime& t) -> const RefRow* {
                const RefRow* best = nullptr;
                double best_dt = 0.25;
                for (const auto& r : ref) {
                    const double d = std::abs(r.time - t);
                    if (d < best_dt) { best_dt = d; best = &r; }
                }
                return best;
            };
            if (!gtsam_imu.epoch_attitude_rpy_deg.empty()) {
                const std::size_t ne = gtsam_imu.epoch_attitude_rpy_deg.size();
                const std::size_t samples[3] = {std::size_t(0), ne / 2, ne - 1};
                std::cout << "  estimated attitude [roll pitch heading] deg / velocity ENU (m/s):\n";
                for (std::size_t k = 0; k < 3; ++k) {
                    const std::size_t i = samples[k];
                    const auto& a = gtsam_imu.epoch_attitude_rpy_deg[i];
                    const auto& v = gtsam_imu.epoch_velocity_nav_mps[i];
                    std::cout << "    epoch " << i << ": [" << a.transpose() << "]  v=["
                              << v.transpose() << "]";
                    const RefRow* r = refAt(gtsam_imu.solution.solutions[i].time);
                    if (r) {
                        std::cout << "  ref[roll pitch heading]=[" << r->roll_deg << " "
                                  << r->pitch_deg << " " << r->heading_deg << "] ref_v=["
                                  << r->ve << " " << r->vn << " " << r->vu << "]";
                    }
                    std::cout << "\n";
                }
                // Heading RMS error vs reference over moving epochs (speed>1 m/s).
                if (!ref.empty()) {
                    double sum_sq = 0.0; std::size_t cnt = 0;
                    for (std::size_t i = 0; i < ne; ++i) {
                        const auto& v = gtsam_imu.epoch_velocity_nav_mps[i];
                        if (std::hypot(v.x(), v.y()) < 1.0) continue;
                        const RefRow* r = refAt(gtsam_imu.solution.solutions[i].time);
                        if (!r) continue;
                        double dh = gtsam_imu.epoch_attitude_rpy_deg[i].z() - r->heading_deg;
                        while (dh > 180.0) dh -= 360.0;
                        while (dh < -180.0) dh += 360.0;
                        sum_sq += dh * dh; ++cnt;
                    }
                    if (cnt > 0) {
                        std::cout << "  heading RMS error vs reference (moving epochs, n=" << cnt
                                  << ") = " << std::sqrt(sum_sq / double(cnt)) << " deg\n";
                    }
                }
            }
            // Confirm per-epoch LAMBDA (gtsam::Marginals over the now
            // IMU-augmented graph, with the 2a rotation pin removed) still
            // factorizes -- the 2b requirement that Marginals succeeds.
            double t_imu_fix = 0.0;
            const auto gtsam_imu_fix =
                run(problem_imu, config, libgnss::FGOBackend::GTSAM, true, t_imu_fix,
                    /*use_pose3=*/true, /*use_imu=*/true);
            const std::size_t imu_fixed_epochs = countFixedEpochs(gtsam_imu_fix);
            const bool marginals_ok = gtsam_imu_fix.diagnostics.lambda_ambiguity_attempts > 0;
            std::cout << "  IMU + per-epoch LAMBDA: marginals_ok=" << (marginals_ok ? "yes" : "no")
                      << " attempts=" << gtsam_imu_fix.diagnostics.lambda_ambiguity_attempts
                      << " fixed_epochs=" << imu_fixed_epochs << "/"
                      << gtsam_imu_fix.solution.solutions.size()
                      << " lambda_ratio=" << gtsam_imu_fix.diagnostics.lambda_ambiguity_ratio
                      << " (" << t_imu_fix << " s)\n";

            const bool go_2b = gtsam_imu.diagnostics.converged && nonfinite == 0 &&
                               gtsam_imu.diagnostics.imu_intervals > 0 &&
                               fp_imu_vs_pose3.compared_epochs > 0 && marginals_ok;
            std::cout << "\nRESULT (milestone 2b): IMU-coupled solve "
                      << (go_2b ? "GO" : "NO-GO") << " (imu_intervals="
                      << gtsam_imu.diagnostics.imu_intervals << ", nonfinite=" << nonfinite
                      << ", IMU-vs-2a max delta=" << fp_imu_vs_pose3.max_delta_m << " m)\n";
            std::cout.flush();
        }
    }

    if (args.float_only) {
        const bool go = fp.compared_epochs > 0 && fp.max_delta_m < 0.05;
        std::cout << "\nRESULT (float-only): float-parity " << (go ? "GO" : "NO-GO")
                  << " (max=" << fp.max_delta_m << " m rms=" << fp.rms_delta_m << " m over "
                  << fp.compared_epochs << " epochs)\n";
        std::cout.flush();
        return 0;
    }

    // --- (b) Fix agreement: LAMBDA on both backends ---
    double t_ex = 0.0, t_gx = 0.0;
    const auto eigen_fix = run(problem, config, libgnss::FGOBackend::Eigen, true, t_ex);
    const auto gtsam_fix = run(problem, config, libgnss::FGOBackend::GTSAM, true, t_gx);

    const std::size_t e_fixed_epochs = countFixedEpochs(eigen_fix);
    const std::size_t g_fixed_epochs = countFixedEpochs(gtsam_fix);
    // Per-epoch fix-status disagreement (one FIXED, the other not).
    std::size_t fix_disagreements = 0;
    const std::size_t nfx =
        std::min(eigen_fix.solution.solutions.size(), gtsam_fix.solution.solutions.size());
    for (std::size_t i = 0; i < nfx; ++i) {
        const bool ef = eigen_fix.solution.solutions[i].status == libgnss::SolutionStatus::FIXED;
        const bool gf = gtsam_fix.solution.solutions[i].status == libgnss::SolutionStatus::FIXED;
        if (ef != gf) {
            ++fix_disagreements;
        }
    }

    const FixedDelta fd = compareFixedPositions(eigen_fix, gtsam_fix);

    std::cout << "\n=== (b) FIXED / LAMBDA agreement (per-epoch LAMBDA on both) ===\n"
              << "  eigen: fixed_solution=" << (eigen_fix.diagnostics.fixed_solution ? "1" : "0")
              << " fixed_ambiguities=" << eigen_fix.diagnostics.fixed_ambiguities
              << " lambda_ratio=" << eigen_fix.diagnostics.lambda_ambiguity_ratio
              << " fixed_epochs=" << e_fixed_epochs << "/" << eigen_fix.solution.solutions.size()
              << " (" << t_ex << " s)\n"
              << "  gtsam: fixed_solution=" << (gtsam_fix.diagnostics.fixed_solution ? "1" : "0")
              << " fixed_ambiguities=" << gtsam_fix.diagnostics.fixed_ambiguities
              << " lambda_ratio=" << gtsam_fix.diagnostics.lambda_ambiguity_ratio
              << " lambda_candidates=" << gtsam_fix.diagnostics.lambda_ambiguity_candidates
              << " fixed_epochs=" << g_fixed_epochs << "/" << gtsam_fix.solution.solutions.size()
              << " (" << t_gx << " s)\n"
              << "  per-epoch fix-status disagreements=" << fix_disagreements << "\n"
              << "  both-fixed epochs=" << fd.both_fixed
              << ", fixed-position delta: max=" << fd.max_delta_m << " m rms=" << fd.rms_delta_m
              << " m\n";

    // --- (b2) Milestone 2a: does per-epoch LAMBDA still work on the Pose3
    // path? gtsam::Marginals needs the graph Hessian to be non-singular;
    // the Pose3RotationPrior gauge pin (fgo_gtsam_backend.cpp) exists
    // specifically so Cholesky succeeds here. ---
    double t_gp3x = 0.0;
    const auto gtsam_pose3_fix =
        run(problem, config, libgnss::FGOBackend::GTSAM, true, t_gp3x, /*use_pose3=*/true);
    const std::size_t gp3_fixed_epochs = countFixedEpochs(gtsam_pose3_fix);
    const FixedDelta fd_pose3 = compareFixedPositions(gtsam_fix, gtsam_pose3_fix);
    std::size_t fix_disagreements_pose3 = 0;
    const std::size_t nfx3 =
        std::min(gtsam_fix.solution.solutions.size(), gtsam_pose3_fix.solution.solutions.size());
    for (std::size_t i = 0; i < nfx3; ++i) {
        const bool gf = gtsam_fix.solution.solutions[i].status == libgnss::SolutionStatus::FIXED;
        const bool pf = gtsam_pose3_fix.solution.solutions[i].status == libgnss::SolutionStatus::FIXED;
        if (gf != pf) {
            ++fix_disagreements_pose3;
        }
    }
    std::cout << "\n=== (b2) MILESTONE 2a: Pose3-GTSAM per-epoch LAMBDA (Marginals/Cholesky) ===\n"
              << "  pose3-gtsam: fixed_solution=" << (gtsam_pose3_fix.diagnostics.fixed_solution ? "1" : "0")
              << " fixed_ambiguities=" << gtsam_pose3_fix.diagnostics.fixed_ambiguities
              << " lambda_ratio=" << gtsam_pose3_fix.diagnostics.lambda_ambiguity_ratio
              << " fixed_epochs=" << gp3_fixed_epochs << "/" << gtsam_pose3_fix.solution.solutions.size()
              << " (" << t_gp3x << " s)\n"
              << "  per-epoch fix-status disagreements vs Point3-GTSAM=" << fix_disagreements_pose3 << "\n"
              << "  both-fixed epochs=" << fd_pose3.both_fixed
              << ", fixed-position delta vs Point3-GTSAM: max=" << fd_pose3.max_delta_m
              << " m rms=" << fd_pose3.rms_delta_m << " m\n";

    // Go/No-go on the float parity (the Phase-1 gate).
    const bool go = fp.compared_epochs > 0 && fp.max_delta_m < 0.05 /* 5 cm */;
    std::cout << "\nRESULT: float-parity " << (go ? "GO" : "NO-GO")
              << " (max=" << fp.max_delta_m << " m over " << fp.compared_epochs << " epochs)\n";
    return 0;
}
