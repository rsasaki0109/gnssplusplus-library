#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <libgnss++/algorithms/rtk.hpp>
#include <libgnss++/algorithms/spp.hpp>
#include <libgnss++/core/solution.hpp>
#include <libgnss++/fusion/fusion_processor.hpp>
#include <libgnss++/io/imu.hpp>
#include <libgnss++/io/rinex.hpp>

#include "rtk_base_epoch_align.hpp"

namespace {

constexpr double kExactTimeToleranceSeconds = libgnss_apps::kExactTimeToleranceSeconds;

struct FuseOptions {
    std::string data_dir;
    std::string rover_path;
    std::string base_path;
    std::string nav_path;
    std::string imu_path;
    std::string out_path = "output/fused_solution.pos";
    std::string kml_path;
    bool write_kml = false;
    std::string attitude_csv_path;  ///< optional roll/pitch/yaw debug export (validation-only)

    // RTK tuning (only consulted when --base is provided; see
    // docs/design.md 5 -- "same defaults/presets" as the `solve` command).
    std::string rtk_preset;
    bool base_position_override = false;
    Eigen::Vector3d base_position_ecef = Eigen::Vector3d::Zero();
    double max_baseline_length_m = 20000.0;
    double ratio_threshold = 3.0;
    bool ratio_threshold_set = false;
    double elevation_mask_deg = 15.0;
    bool enable_base_interpolation = true;
    // Off by default: see runRtkFusion's doc comment. RTKProcessor/SPPProcessor
    // now populate has_velocity from a real Doppler-derived least squares
    // solve (spp_velocity::solveVelocity/solveVelocityFromObservations, see
    // docs/design.md), which is preferred and applies whenever an epoch has
    // >= 4 satellites with usable Doppler. This finite-difference-of-FIXED-
    // positions fallback remains available (guarded on !has_velocity, so it
    // only fires when Doppler velocity was unavailable that epoch) but stays
    // opt-in: validation on PPC tokyo/run1 showed it can amplify a single
    // low-cost-preset wrong-fix into a bad heading/attitude latch
    // (fusion_initialization::tryAlignHeading is a one-shot, ungated
    // correction).
    bool derive_velocity_from_fixed = false;
    bool tightly_coupled_dd_imu = false;
    bool tightly_coupled_dd_code_only = false;

    libgnss::LooseCouplingProcessor::Config fusion_config;
    int max_epochs = 0;
    bool quiet = false;
    bool verbose = false;
};

std::string requireValue(const std::string& arg, int& i, int argc, char* argv[]) {
    if (i + 1 >= argc) {
        throw std::invalid_argument("missing value for " + arg);
    }
    return argv[++i];
}

Eigen::Vector3d parseVector3(const std::string& text, const std::string& arg) {
    std::stringstream stream(text);
    std::string component;
    std::vector<double> values;
    while (std::getline(stream, component, ',')) {
        values.push_back(std::stod(component));
    }
    if (values.size() != 3) {
        throw std::invalid_argument(arg + " requires exactly 3 comma-separated values, got: " + text);
    }
    return Eigen::Vector3d(values[0], values[1], values[2]);
}

void applyImuGradePreset(const std::string& grade, libgnss::ProcessNoiseConfig& cfg) {
    // Continuous-time spectral-density presets, matching
    // reference_notes.md 7's IMU_PRESETS table (same names/values as the
    // reference so a dataset tuned against both tools shares noise numbers).
    if (grade == "tactical") {
        cfg.accel_noise_density = 2.84e-4;
        cfg.gyro_noise_density = 4.01e-5;
        cfg.accel_bias_random_walk = 3.14e-4;
        cfg.gyro_bias_random_walk = 9.70e-6;
    } else if (grade == "consumer") {
        cfg.accel_noise_density = 2.0e-3;
        cfg.gyro_noise_density = 4.0e-4;
        cfg.accel_bias_random_walk = 1.0e-3;
        cfg.gyro_bias_random_walk = 5.0e-5;
    } else if (grade == "industrial") {
        cfg.accel_noise_density = 6.0e-4;
        cfg.gyro_noise_density = 1.0e-4;
        cfg.accel_bias_random_walk = 5.0e-4;
        cfg.gyro_bias_random_walk = 2.0e-5;
    } else if (grade == "nav_grade") {
        cfg.accel_noise_density = 5.7e-5;
        cfg.gyro_noise_density = 3.5e-6;
        cfg.accel_bias_random_walk = 2.0e-5;
        cfg.gyro_bias_random_walk = 5.0e-7;
    } else {
        throw std::invalid_argument("unknown --imu-grade: " + grade +
                                    " (expected tactical|consumer|industrial|nav_grade)");
    }
}

void printUsage(const char* program_name) {
    std::cout
        << "Usage: " << program_name
        << " --rover <rover.obs> --nav <nav.rnx> --imu <imu.csv> [--base <base.obs>] --out <fused.pos>\n"
        << "Loosely-coupled GNSS/IMU fusion (Stage 1, docs/design.md): a 15-state\n"
        << "error-state EKF mechanizes 100 Hz IMU samples and opportunistically\n"
        << "corrects with a position solution computed in-process from --rover/--nav\n"
        << "(SPP), or from --rover/--base/--nav (RTK, same RTKProcessor pipeline the\n"
        << "`solve` command uses) when --base is supplied.\n\n"
        << "Options:\n"
        << "  --data-dir <dir>             Use <dir>/rover.obs, base.obs, base.nav, imu.csv as\n"
        << "                                defaults for --rover/--base/--nav/--imu\n"
        << "  --rover <rover.obs>          RINEX observation file (required)\n"
        << "  --base <base.obs>            RINEX base observation file. When given, gnss_fuse\n"
        << "                                runs the RTKProcessor pipeline (same as `solve`) and\n"
        << "                                feeds its per-epoch PositionSolution to the fusion\n"
        << "                                filter instead of an SPP-only solution.\n"
        << "  --nav <nav.rnx>              RINEX navigation file (required)\n"
        << "  --imu <imu.csv>              PPC-Dataset-style IMU CSV file (required)\n"
        << "  --out <fused.pos>            Output position file (default: output/fused_solution.pos)\n"
        << "  --kml <fused.kml>            Optional KML trajectory output\n"
        << "  --attitude-csv <path>        Optional roll/pitch/yaw (deg) debug CSV, one row per\n"
        << "                                fused epoch (validation against reference.csv attitude)\n"
        << "  --lever-arm x,y,z            IMU -> antenna lever arm, body FLU, meters (default 0,0,0)\n"
        << "  --zupt / --no-zupt           Enable/disable zero-velocity updates (default: enabled)\n"
        << "  --nhc / --no-nhc             Enable/disable the non-holonomic constraint (default: disabled)\n"
        << "  --imu-grade <grade>          tactical|consumer|industrial|nav_grade (default: tactical)\n"
        << "  --align-vel-thresh <mps>     Minimum GNSS speed to attempt heading alignment (default: 1.0)\n"
        << "  --align-static-window-s <s>  Stationary window duration for coarse leveling (default: 2.0)\n"
        << "  --align-heading-window-s <s> Trailing window of GNSS-course samples considered for a\n"
        << "                                heading latch (default: 2.0)\n"
        << "  --align-heading-min-samples <n>\n"
        << "                                Minimum consistent above-threshold GNSS-course samples\n"
        << "                                (within the window) required before latching heading;\n"
        << "                                a single noisy epoch can no longer latch alone (default: 3)\n"
        << "  --align-heading-max-scatter-deg <deg>\n"
        << "                                Max circular-standard-deviation of buffered course samples\n"
        << "                                allowed for a latch (default: 10.0)\n"
        << "  --heading-recovery-nis <v>   If the velocity-update NIS-per-observation EMA stays above\n"
        << "                                this for --heading-recovery-min-bad-epochs updates, yaw\n"
        << "                                covariance is re-inflated so ordinary updates can pull it\n"
        << "                                back (<=0 disables). Default: 30.0\n"
        << "  --heading-recovery-min-bad-epochs <n>\n"
        << "                                Consecutive bad-NIS velocity updates required to trigger\n"
        << "                                heading recovery. Default: 0 (disabled -- PPC nagoya/run1\n"
        << "                                validation found this fires on routine turning dynamics\n"
        << "                                too, given a nonzero lever arm, and net hurts accuracy;\n"
        << "                                opt in and tune for your own dataset if desired)\n"
        << "  --heading-recovery-cooldown-epochs <n>\n"
        << "                                Velocity-update epochs to wait after a recovery fires\n"
        << "                                before it can fire again. Default: 20\n"
        << "  --max-position-nis <v>       Reject a GNSS position update if its normalized\n"
        << "                                innovation squared per observation exceeds this (guards\n"
        << "                                against RTK float-reconvergence jumps corrupting attitude\n"
        << "                                via the lever-arm coupling; <=0 disables). Default: 500.0\n"
        << "  --max-velocity-nis <v>       Same gate for the GNSS velocity update. Default: 500.0\n"
        << "  --max-consecutive-gate-rejections <n>\n"
        << "                                Force-accept a channel's update after this many\n"
        << "                                consecutive NIS-gate rejections, to escape a lockout\n"
        << "                                spiral where the drifted state makes every later fix\n"
        << "                                look like an outlier too (<=0 disables). Default: 30\n"
        << "  --preset <survey|low-cost|moving-base|odaiba>\n"
        << "                                RTK tuning preset, only used with --base (default: none)\n"
        << "  --ratio <value>              RTK ambiguity ratio threshold (default: 3.0)\n"
        << "  --max-baseline-m <v>         Max RTK baseline length in meters (default: 20000)\n"
        << "  --elevation-mask-deg <v>     RTK elevation mask in degrees (default: 15)\n"
        << "  --base-ecef <x> <y> <z>      Override base ECEF position in meters\n"
        << "  --no-base-interp             Require exact rover/base epoch alignment (RTK path)\n"
        << "  --derive-velocity-from-fixed Fallback: derive GNSS velocity from consecutive FIXED\n"
        << "                                position differences (RTK path only) when an epoch's\n"
        << "                                Doppler-derived velocity solve (the default source of\n"
        << "                                has_velocity) is unavailable. Experimental -- see\n"
        << "                                docs/design.md notes on wrong-fix sensitivity. Default: off.\n"
        << "  --tight-dd-imu               Opt-in DD + IMU ESKF update. Code DD is committed; carrier DD\n"
        << "                                and ambiguity/partial-AR updates are shadow-gated by default\n"
        << "  --tight-dd-carrier-experimental\n"
        << "                                Commit carrier/ambiguity updates that pass the innovation\n"
        << "                                gate (research ablation; real-data validation found instability)\n"
        << "  --tight-dd-code-only         Diagnostic ablation: disable DD carrier/PAR rows\n"
        << "                                (requires --base; default: off)\n"
        << "  --max-epochs <n>             Stop after n GNSS observation epochs (0 = no limit)\n"
        << "  --verbose                    Print periodic per-epoch progress\n"
        << "  --quiet                      Suppress run summary\n"
        << "  -h, --help                   Show this help\n";
}

// Converts the fusion filter's body(FLU)->ENU attitude quaternion into
// roll/pitch/yaw (degrees), using the same aerospace 3-2-1 (yaw-pitch-roll)
// Euler decomposition reference.csv's Applanix POS LV roll/pitch/heading
// columns follow (body FRD, world NED). FLU/ENU -> FRD/NED is a pure
// axis-sign/permutation change (no physical rotation), so R_frd_ned =
// P * R_flu_enu * D, with D = diag(1,-1,-1) (FLU->FRD: negate left/up) and
// P mapping ENU->NED ((N,E,D) = (y_enu, x_enu, -z_enu)). This is validation
// tooling only (docs/design.md's reference.csv attitude truth, task B), not
// part of the library's public state representation.
struct EulerAnglesDeg {
    double roll_deg = 0.0;
    double pitch_deg = 0.0;
    double yaw_deg = 0.0;
};

EulerAnglesDeg fluEnuQuaternionToNedEulerDeg(const Eigen::Quaterniond& attitude_body_to_enu) {
    const Eigen::Matrix3d r_flu_enu = attitude_body_to_enu.toRotationMatrix();
    Eigen::Matrix3d d = Eigen::Matrix3d::Identity();
    d(1, 1) = -1.0;
    d(2, 2) = -1.0;
    Eigen::Matrix3d p;
    p << 0.0, 1.0, 0.0,
         1.0, 0.0, 0.0,
         0.0, 0.0, -1.0;
    const Eigen::Matrix3d r_frd_ned = p * r_flu_enu * d;

    EulerAnglesDeg angles;
    angles.yaw_deg = std::atan2(r_frd_ned(1, 0), r_frd_ned(0, 0)) * 180.0 / M_PI;
    if (angles.yaw_deg < 0.0) angles.yaw_deg += 360.0;
    angles.pitch_deg =
        std::atan2(-r_frd_ned(2, 0), std::sqrt(r_frd_ned(2, 1) * r_frd_ned(2, 1) +
                                               r_frd_ned(2, 2) * r_frd_ned(2, 2))) *
        180.0 / M_PI;
    angles.roll_deg = std::atan2(r_frd_ned(2, 1), r_frd_ned(2, 2)) * 180.0 / M_PI;
    return angles;
}

class AttitudeCsvWriter {
public:
    bool open(const std::string& path) {
        if (path.empty()) {
            return true;
        }
        file_.open(path);
        if (!file_.is_open()) {
            return false;
        }
        file_ << "gps_week,tow,roll_deg,pitch_deg,yaw_deg\n";
        return true;
    }

    void write(const libgnss::GNSSTime& time, const Eigen::Quaterniond& attitude_body_to_enu) {
        if (!file_.is_open()) {
            return;
        }
        const auto angles = fluEnuQuaternionToNedEulerDeg(attitude_body_to_enu);
        file_ << time.week << "," << std::fixed << std::setprecision(3) << time.tow << ","
              << std::setprecision(4) << angles.roll_deg << "," << angles.pitch_deg << ","
              << angles.yaw_deg << "\n";
    }

private:
    std::ofstream file_;
};

[[noreturn]] void argumentError(const std::string& message, const char* program_name) {
    std::cerr << "Argument error: " << message << "\n\n";
    printUsage(program_name);
    std::exit(1);
}

FuseOptions parseArguments(int argc, char* argv[]) {
    FuseOptions options;
    // LooseCouplingProcessor::Config's own library-wide default for both of
    // these is 0.0 (no innovation gating at all -- see
    // fusion_update::applyDenseUpdate, threshold<=0 disables the check).
    // That default was never exercised against real RTK-outage/reacquisition
    // behavior before this app could supply a GNSS velocity (docs/design.md
    // Doppler-velocity task): validating on PPC tokyo/run1 surfaced that an
    // RTK float solution reconverging after an outage can report a
    // deceptively tight position_covariance while its actual position is
    // tens of meters off (a known RTK/float-ambiguity failure mode already
    // guarded against *inside* RTKProcessor's own float-vs-trusted jump
    // gates -- rtk_validation.hpp -- but those guards don't reach this
    // fusion-filter consumption path). Applied ungated, that single bad
    // epoch's residual gets projected through the position update's
    // lever-arm/attitude Jacobian and can tip the whole attitude estimate
    // over (observed: roll/pitch diverging for the remainder of the run).
    // Set conservative-but-real default gates here (this app's own policy,
    // not a change to the library default) rather than shipping ungated by
    // default; both remain overridable via --max-position-nis/--max-velocity-nis.
    //
    // The threshold and LooseCouplingProcessor::Config::
    // max_consecutive_gate_rejections (the lockout-spiral escape hatch, see
    // fusion_processor.hpp) interact non-monotonically -- a tight threshold
    // rejects more often, so the escape hatch engages more often too, each
    // time risking forcing through a genuinely bad update. Empirically swept
    // on PPC tokyo/run1 (docs/design.md validation): 500 paired with a
    // patience of 30 consecutive rejections gave the best result of the
    // configurations tried (H-RMSE ~9 m, roll/pitch RMSE ~1 deg, vs. e.g.
    // 20/5 which spirals into tens of km of drift, or 500/5 which recovers
    // position but leaves roll RMSE ~17 deg from more frequent forced
    // updates). Treat these as a reasonable starting point for a low-cost
    // receiver + this preset, not a universally optimal constant -- both are
    // exposed via --max-position-nis/--max-velocity-nis/
    // --max-consecutive-gate-rejections for retuning on other datasets.
    options.fusion_config.max_position_update_nis_per_observation = 500.0;
    options.fusion_config.max_velocity_update_nis_per_observation = 500.0;
    options.fusion_config.max_consecutive_gate_rejections = 30;
    std::string imu_grade = "tactical";
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            std::exit(0);
        } else if (arg == "--data-dir") {
            options.data_dir = requireValue(arg, i, argc, argv);
        } else if (arg == "--rover") {
            options.rover_path = requireValue(arg, i, argc, argv);
        } else if (arg == "--base") {
            options.base_path = requireValue(arg, i, argc, argv);
        } else if (arg == "--nav") {
            options.nav_path = requireValue(arg, i, argc, argv);
        } else if (arg == "--imu") {
            options.imu_path = requireValue(arg, i, argc, argv);
        } else if (arg == "--out") {
            options.out_path = requireValue(arg, i, argc, argv);
        } else if (arg == "--kml") {
            options.kml_path = requireValue(arg, i, argc, argv);
            options.write_kml = true;
        } else if (arg == "--attitude-csv") {
            options.attitude_csv_path = requireValue(arg, i, argc, argv);
        } else if (arg == "--lever-arm") {
            options.fusion_config.lever_arm_body =
                parseVector3(requireValue(arg, i, argc, argv), arg);
        } else if (arg == "--zupt") {
            options.fusion_config.zupt_enable = true;
        } else if (arg == "--no-zupt") {
            options.fusion_config.zupt_enable = false;
        } else if (arg == "--nhc") {
            options.fusion_config.nhc_enable = true;
        } else if (arg == "--no-nhc") {
            options.fusion_config.nhc_enable = false;
        } else if (arg == "--imu-grade") {
            imu_grade = requireValue(arg, i, argc, argv);
        } else if (arg == "--align-vel-thresh") {
            options.fusion_config.align_velocity_threshold_mps =
                std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--align-static-window-s") {
            options.fusion_config.align_static_window_s =
                std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--align-heading-window-s") {
            options.fusion_config.align_heading_window_s =
                std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--align-heading-min-samples") {
            options.fusion_config.align_heading_min_samples =
                std::stoi(requireValue(arg, i, argc, argv));
        } else if (arg == "--align-heading-max-scatter-deg") {
            options.fusion_config.align_heading_max_course_scatter_deg =
                std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--heading-recovery-nis") {
            options.fusion_config.heading_recovery_nis_threshold =
                std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--heading-recovery-min-bad-epochs") {
            options.fusion_config.heading_recovery_min_bad_epochs =
                std::stoi(requireValue(arg, i, argc, argv));
        } else if (arg == "--heading-recovery-cooldown-epochs") {
            options.fusion_config.heading_recovery_cooldown_epochs =
                std::stoi(requireValue(arg, i, argc, argv));
        } else if (arg == "--max-position-nis") {
            options.fusion_config.max_position_update_nis_per_observation =
                std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--max-velocity-nis") {
            options.fusion_config.max_velocity_update_nis_per_observation =
                std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--max-consecutive-gate-rejections") {
            options.fusion_config.max_consecutive_gate_rejections =
                std::stoi(requireValue(arg, i, argc, argv));
        } else if (arg == "--preset") {
            options.rtk_preset = requireValue(arg, i, argc, argv);
        } else if (arg == "--ratio") {
            options.ratio_threshold = std::stod(requireValue(arg, i, argc, argv));
            options.ratio_threshold_set = true;
        } else if (arg == "--max-baseline-m") {
            options.max_baseline_length_m = std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--elevation-mask-deg") {
            options.elevation_mask_deg = std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--base-ecef") {
            if (i + 3 >= argc) {
                throw std::invalid_argument("--base-ecef requires 3 values");
            }
            options.base_position_ecef =
                Eigen::Vector3d(std::stod(argv[i + 1]), std::stod(argv[i + 2]), std::stod(argv[i + 3]));
            i += 3;
            options.base_position_override = true;
        } else if (arg == "--no-base-interp") {
            options.enable_base_interpolation = false;
        } else if (arg == "--derive-velocity-from-fixed") {
            options.derive_velocity_from_fixed = true;
        } else if (arg == "--tight-dd-imu") {
            options.tightly_coupled_dd_imu = true;
        } else if (arg == "--tight-dd-carrier-experimental") {
            options.tightly_coupled_dd_imu = true;
            options.fusion_config.tight_dd_commit_carrier_updates = true;
        } else if (arg == "--tight-dd-code-only") {
            options.tightly_coupled_dd_imu = true;
            options.tightly_coupled_dd_code_only = true;
        } else if (arg == "--max-epochs") {
            options.max_epochs = std::stoi(requireValue(arg, i, argc, argv));
        } else if (arg == "--verbose") {
            options.verbose = true;
        } else if (arg == "--quiet") {
            options.quiet = true;
        } else {
            argumentError("unknown or incomplete argument: " + arg, argv[0]);
        }
    }

    applyImuGradePreset(imu_grade, options.fusion_config.process_noise);

    if (!options.data_dir.empty()) {
        if (options.rover_path.empty()) options.rover_path = options.data_dir + "/rover.obs";
        if (options.base_path.empty()) options.base_path = options.data_dir + "/base.obs";
        if (options.nav_path.empty()) options.nav_path = options.data_dir + "/base.nav";
        if (options.imu_path.empty()) options.imu_path = options.data_dir + "/imu.csv";
    }

    if (options.rover_path.empty()) argumentError("--rover is required", argv[0]);
    if (options.nav_path.empty()) argumentError("--nav is required", argv[0]);
    if (options.imu_path.empty()) argumentError("--imu is required", argv[0]);
    if (options.tightly_coupled_dd_imu && options.base_path.empty()) {
        argumentError("--tight-dd-imu requires --base", argv[0]);
    }
    if (options.max_epochs < 0) argumentError("--max-epochs must be non-negative", argv[0]);

    return options;
}

// SPP-only path (no --base): mirrors the original gnss_fuse behavior, used
// as a fallback when no base station is available.
int runSppFusion(const FuseOptions& options, libgnss::ImuSeries& imu_series,
                 const libgnss::ImuAxisConvention& axis_convention,
                 const libgnss::NavigationData& nav_data) {
    libgnss::io::RINEXReader obs_reader;
    if (!obs_reader.open(options.rover_path)) {
        std::cerr << "Error: failed to open observation file: " << options.rover_path << "\n";
        return 1;
    }
    libgnss::io::RINEXReader::RINEXHeader obs_header;
    if (!obs_reader.readHeader(obs_header)) {
        std::cerr << "Error: failed to read observation header: " << options.rover_path << "\n";
        return 1;
    }

    libgnss::ProcessorConfig spp_processor_config;
    spp_processor_config.mode = libgnss::PositioningMode::SPP;
    libgnss::SPPProcessor spp_processor;
    if (!spp_processor.initialize(spp_processor_config)) {
        std::cerr << "Error: failed to initialize SPP processor\n";
        return 1;
    }

    libgnss::LooseCouplingProcessor fusion_processor(options.fusion_config);
    libgnss::Solution fused_solution;
    AttitudeCsvWriter attitude_writer;
    if (!attitude_writer.open(options.attitude_csv_path)) {
        std::cerr << "Error: failed to open attitude CSV: " << options.attitude_csv_path << "\n";
        return 1;
    }

    size_t imu_cursor = 0;
    int processed_epochs = 0;
    int valid_solutions = 0;

    libgnss::ObservationData obs;
    while (obs_reader.readObservationEpoch(obs)) {
        if (options.max_epochs > 0 && processed_epochs >= options.max_epochs) {
            break;
        }
        if (obs_header.approximate_position.norm() > 0.0) {
            obs.receiver_position = obs_header.approximate_position;
        }

        while (imu_cursor < imu_series.samples.size() &&
               imu_series.samples[imu_cursor].time <= obs.time) {
            libgnss::ImuSample sample = imu_series.samples[imu_cursor];
            sample.accel_raw = axis_convention.apply(sample.accel_raw);
            sample.gyro_raw_radps = axis_convention.apply(sample.gyro_raw_radps);
            fusion_processor.processImuSample(sample);
            ++imu_cursor;
        }

        const auto spp_solution = spp_processor.processEpoch(obs, nav_data);
        if (spp_solution.isValid()) {
            fusion_processor.processGnssSolution(spp_solution);
            ++valid_solutions;
        }

        if (fusion_processor.isOriginSet()) {
            fused_solution.addSolution(fusion_processor.toPositionSolution());
            attitude_writer.write(fusion_processor.state().nominal.time,
                                  fusion_processor.state().nominal.attitude_body_to_enu);
        }
        ++processed_epochs;
    }

    if (!fused_solution.writeToFile(options.out_path)) {
        std::cerr << "Error: failed to write fused solution file: " << options.out_path << "\n";
        return 1;
    }
    if (options.write_kml && !fused_solution.writeKML(options.kml_path)) {
        std::cerr << "Error: failed to write KML file: " << options.kml_path << "\n";
        return 1;
    }

    if (!options.quiet) {
        std::cout << "GNSS half: SPP (no --base provided)\n";
        std::cout << "IMU samples loaded: " << imu_series.samples.size() << "\n";
        std::cout << "GNSS epochs processed: " << processed_epochs << "\n";
        std::cout << "Valid SPP solutions: " << valid_solutions << "\n";
        std::cout << "Fused epochs written: " << fused_solution.size() << "\n";
        std::cout << "Fusion initialized: " << (fusion_processor.isInitialized() ? "yes" : "no") << "\n";
        // isHeadingAligned() only means "a latch has happened at some point"
        // -- it used to print "yes" even on runs where the latch was badly
        // wrong and never recovered (PPC nagoya/run1 investigation).
        // isHeadingConverged() is the real health signal: aligned AND the
        // recent velocity-innovation NIS still corroborates that latch.
        std::cout << "Heading aligned: " << (fusion_processor.isHeadingAligned() ? "yes" : "no") << "\n";
        std::cout << "Heading converged: " << (fusion_processor.isHeadingConverged() ? "yes" : "no") << "\n";
        std::cout << "Output: " << options.out_path << "\n";
        if (options.write_kml) {
            std::cout << "KML: " << options.kml_path << "\n";
        }
    }
    return 0;
}

// RTK path (--base provided): drives libgnss::RTKProcessor over the
// rover/base RINEX pair using the exact same epoch-alignment helpers
// (libgnss_apps::interpolateBaseEpoch et al.) apps/gnss_solve.cpp uses, so
// the GNSS half of `gnss fuse` is the same RTK pipeline as `gnss solve`
// rather than a re-implementation (docs/design.md 5). Solution quality
// flows into the fusion filter for free: RTKProcessor already populates
// PositionSolution::position_covariance (small for FIXED, larger for
// FLOAT), and LooseCouplingProcessor::processGnssSolution() consumes that
// covariance directly (fusion_processor.cpp), so a FIXED epoch pulls the
// filter harder than a FLOAT/degraded epoch with no extra plumbing needed.
int runRtkFusion(const FuseOptions& options, libgnss::ImuSeries& imu_series,
                 const libgnss::ImuAxisConvention& axis_convention,
                 const libgnss::NavigationData& nav_data) {
    libgnss::io::RINEXReader rover_reader;
    if (!rover_reader.open(options.rover_path)) {
        std::cerr << "Error: cannot open rover observation file: " << options.rover_path << "\n";
        return 1;
    }
    libgnss::io::RINEXReader::RINEXHeader rover_header;
    if (!rover_reader.readHeader(rover_header)) {
        std::cerr << "Error: failed to read rover observation header\n";
        return 1;
    }

    libgnss::io::RINEXReader base_reader;
    if (!base_reader.open(options.base_path)) {
        std::cerr << "Error: cannot open base observation file: " << options.base_path << "\n";
        return 1;
    }
    libgnss::io::RINEXReader::RINEXHeader base_header;
    if (!base_reader.readHeader(base_header)) {
        std::cerr << "Error: failed to read base observation header\n";
        return 1;
    }

    Eigen::Vector3d base_position = Eigen::Vector3d::Zero();
    if (options.base_position_override) {
        base_position = options.base_position_ecef;
    } else if (base_header.approximate_position.norm() > 0.0) {
        base_position = base_header.approximate_position;
    } else {
        std::cerr << "Error: base position unavailable. Use --base-ecef to override.\n";
        return 1;
    }

    libgnss::RTKProcessor rtk_processor;
    libgnss::RTKProcessor::RTKConfig rtk_config;
    rtk_config.max_baseline_length = options.max_baseline_length_m;
    rtk_config.ar_mode = libgnss::RTKProcessor::RTKConfig::AmbiguityResolutionMode::CONTINUOUS;
    if (options.ratio_threshold_set) {
        rtk_config.ratio_threshold = options.ratio_threshold;
        rtk_config.ambiguity_ratio_threshold = options.ratio_threshold;
    }
    rtk_config.elevation_mask = options.elevation_mask_deg * M_PI / 180.0;
    if (!libgnss_apps::applyRtkConfigPreset(options.rtk_preset, rtk_config)) {
        std::cerr << "Error: unsupported --preset value: " << options.rtk_preset << "\n";
        return 1;
    }
    // A non-default --ratio always wins over whatever the preset set, since
    // it was requested explicitly after preset resolution would have run.
    if (options.ratio_threshold_set) {
        rtk_config.ratio_threshold = options.ratio_threshold;
        rtk_config.ambiguity_ratio_threshold = options.ratio_threshold;
    }
    rtk_processor.setRTKConfig(rtk_config);
    rtk_processor.setBasePosition(base_position);

    libgnss::LooseCouplingProcessor fusion_processor(options.fusion_config);
    libgnss::Solution fused_solution;
    AttitudeCsvWriter attitude_writer;
    if (!attitude_writer.open(options.attitude_csv_path)) {
        std::cerr << "Error: failed to open attitude CSV: " << options.attitude_csv_path << "\n";
        return 1;
    }

    libgnss::ObservationData rover_obs;
    libgnss::ObservationData base_obs;
    libgnss::ObservationData previous_base_obs;
    bool has_previous_base = false;
    bool rover_ok = rover_reader.readObservationEpoch(rover_obs);
    bool base_ok = base_reader.readObservationEpoch(base_obs);
    if (!rover_ok) {
        std::cerr << "Error: no rover epochs available\n";
        return 1;
    }
    if (!base_ok) {
        std::cerr << "Error: no base epochs available\n";
        return 1;
    }

    if (rover_header.approximate_position.norm() > 0.0) {
        rover_obs.receiver_position = rover_header.approximate_position;
    } else {
        rover_obs.receiver_position = base_position + Eigen::Vector3d(3000.0, 0.0, 0.0);
    }

    size_t imu_cursor = 0;
    int processed_epochs = 0;
    int valid_solutions = 0;
    int fixed_solutions = 0;
    int exact_base_epochs = 0;
    int interpolated_base_epochs = 0;
    int skipped_rover_epochs = 0;
    int derived_velocity_updates = 0;
    int tight_dd_epochs = 0;
    int tight_dd_accepted = 0;
    int tight_dd_rejected = 0;
    int tight_dd_carrier_fallbacks = 0;
    int tight_dd_rows = 0;
    int tight_dd_partial_ar_epochs = 0;
    int tight_dd_fixed_ambiguities = 0;
    int tight_dd_soft_resets = 0;
    int tight_dd_nis_samples = 0;
    double tight_dd_nis_sum = 0.0;
    double tight_dd_nis_max = 0.0;

    // RTKProcessor::processRTKEpoch() now populates PositionSolution::
    // has_velocity from a real Doppler-derived least squares solve
    // (spp_velocity::solveVelocityFromObservations(), run automatically on
    // every valid solution that doesn't already carry one) whenever an
    // epoch has >= 4 satellites with usable Doppler -- so
    // LooseCouplingProcessor::processGnssSolution()'s velocity update and
    // GNSS-course heading alignment (fusion_initialization::
    // tryAlignHeading(), gated on solution.has_velocity) fire on their own
    // now, without any help from this app. The block below is a secondary,
    // opt-in (--derive-velocity-from-fixed) fallback for epochs where
    // Doppler is unavailable (e.g. a receiver/RINEX stream missing D
    // observables): derive a coarse velocity from consecutive FIXED epochs'
    // position difference instead (finite-difference of two cm-level FIXED
    // positions a fraction of a second apart is itself a reasonably precise
    // velocity estimate), propagating each position's covariance through
    // the difference (Var(v) = (Var(p1)+Var(p0))/dt^2). Only FIXED-to-FIXED
    // pairs are used (not FLOAT/SPP) so an ambiguity reconvergence jump
    // never gets misread as a real velocity.
    // Deliberately tight: the "low-cost" preset's raw (unguarded --
    // apps/gnss_solve.cpp's own nonfix-drift/float-bridge/jump guards are
    // solve-specific output polish this app does not replicate, see the
    // runRtkFusion doc comment above) FIXED stream occasionally contains a
    // wrong-fix epoch. A wrong-fix differenced against its FIXED neighbor
    // produces a bogus velocity that is otherwise indistinguishable from a
    // real one; requiring back-to-back epochs (no skipped/FLOAT epoch in
    // between) and a plausible car speed bounds the damage a single
    // wrong-fix can do to fusion_initialization::tryAlignHeading() (a
    // one-shot, ungated latch -- see fusion_initialization.cpp) and to the
    // velocity update.
    libgnss::PositionSolution previous_fixed_solution;
    bool have_previous_fixed_solution = false;
    constexpr double kMaxVelocityDerivationGapSeconds = 0.25;
    constexpr double kMaxPlausibleSpeedMps = 60.0;
    constexpr double kMinDerivedVelocityRatio = 2.0;

    while (rover_ok) {
        if (options.max_epochs > 0 && processed_epochs >= options.max_epochs) {
            break;
        }

        libgnss::ObservationData lower_base_obs = previous_base_obs;
        bool have_lower_base = has_previous_base;

        while (base_ok &&
               libgnss_apps::timeDiffSeconds(base_obs.time, rover_obs.time) < -kExactTimeToleranceSeconds) {
            lower_base_obs = base_obs;
            have_lower_base = true;
            previous_base_obs = base_obs;
            has_previous_base = true;

            libgnss::ObservationData next_base_obs;
            base_ok = base_reader.readObservationEpoch(next_base_obs);
            if (base_ok) {
                base_obs = std::move(next_base_obs);
            }
        }

        libgnss::ObservationData aligned_base_obs;
        const double exact_dt = base_ok
            ? std::abs(libgnss_apps::timeDiffSeconds(base_obs.time, rover_obs.time))
            : std::numeric_limits<double>::infinity();
        bool have_aligned_base = false;

        if (base_ok && exact_dt <= kExactTimeToleranceSeconds) {
            aligned_base_obs = base_obs;
            exact_base_epochs++;
            have_aligned_base = true;
        } else if (options.enable_base_interpolation && base_ok && have_lower_base &&
                   libgnss_apps::timeDiffSeconds(rover_obs.time, lower_base_obs.time) >=
                       -kExactTimeToleranceSeconds &&
                   libgnss_apps::timeDiffSeconds(base_obs.time, rover_obs.time) >=
                       -kExactTimeToleranceSeconds &&
                   libgnss_apps::interpolateBaseEpoch(lower_base_obs, base_obs, rover_obs.time,
                                                      base_position, nav_data, aligned_base_obs)) {
            interpolated_base_epochs++;
            have_aligned_base = true;
        }

        if (!have_aligned_base) {
            skipped_rover_epochs++;
            const Eigen::Vector3d saved_rover_pos = rover_obs.receiver_position;
            rover_ok = rover_reader.readObservationEpoch(rover_obs);
            if (rover_ok) {
                rover_obs.receiver_position = saved_rover_pos;
            }
            processed_epochs++;
            continue;
        }

        while (imu_cursor < imu_series.samples.size() &&
               imu_series.samples[imu_cursor].time <= rover_obs.time) {
            libgnss::ImuSample sample = imu_series.samples[imu_cursor];
            sample.accel_raw = axis_convention.apply(sample.accel_raw);
            sample.gyro_raw_radps = axis_convention.apply(sample.gyro_raw_radps);
            fusion_processor.processImuSample(sample);
            ++imu_cursor;
        }

        auto pos_solution = rtk_processor.processRTKEpoch(rover_obs, aligned_base_obs, nav_data);
        if (pos_solution.isValid()) {
            if (options.derive_velocity_from_fixed && !pos_solution.has_velocity &&
                pos_solution.isFixed() && have_previous_fixed_solution &&
                pos_solution.ratio >= kMinDerivedVelocityRatio &&
                previous_fixed_solution.ratio >= kMinDerivedVelocityRatio) {
                const double dt = pos_solution.time - previous_fixed_solution.time;
                if (dt > 0.0 && dt <= kMaxVelocityDerivationGapSeconds) {
                    const Eigen::Vector3d candidate_velocity =
                        (pos_solution.position_ecef - previous_fixed_solution.position_ecef) / dt;
                    if (candidate_velocity.norm() <= kMaxPlausibleSpeedMps) {
                        pos_solution.velocity_ecef = candidate_velocity;
                        pos_solution.velocity_covariance =
                            (pos_solution.position_covariance + previous_fixed_solution.position_covariance) /
                            (dt * dt);
                        pos_solution.has_velocity = true;
                        ++derived_velocity_updates;
                    }
                }
            }
            if (pos_solution.isFixed()) {
                previous_fixed_solution = pos_solution;
                have_previous_fixed_solution = true;
            }

            fusion_processor.processGnssSolution(pos_solution);
            if (options.tightly_coupled_dd_imu && fusion_processor.isOriginSet()) {
                const auto propagated_solution = fusion_processor.toPositionSolution();
                const auto rotation = fusion_processor.ecefToLocalEnuRotation();
                auto dd_rows = rtk_processor.formTightlyCoupledObservations(
                    rover_obs, aligned_base_obs, nav_data,
                    propagated_solution.position_ecef, rotation,
                    fusion_processor.state().nominal.attitude_body_to_enu);
                if (options.tightly_coupled_dd_code_only) {
                    for (auto& row : dd_rows) row.carrier_variance_m2 = 0.0;
                }
                if (!dd_rows.empty()) {
                    const auto dd_result = fusion_processor.processTightlyCoupledDD(
                        dd_rows, &pos_solution);
                    ++tight_dd_epochs;
                    tight_dd_rows += static_cast<int>(dd_rows.size());
                    if (std::isfinite(dd_result.update.nis_per_observation)) {
                        ++tight_dd_nis_samples;
                        tight_dd_nis_sum += dd_result.update.nis_per_observation;
                        tight_dd_nis_max = std::max(
                            tight_dd_nis_max,
                            dd_result.update.nis_per_observation);
                    }
                    if (dd_result.update.ok) {
                        ++tight_dd_accepted;
                    }
                    if (dd_result.update.rejected_by_innovation_gate) {
                        ++tight_dd_rejected;
                    }
                    if (dd_result.update.carrier_fallback_used) {
                        ++tight_dd_carrier_fallbacks;
                    }
                    if (dd_result.partial_ar.fixed) {
                        ++tight_dd_partial_ar_epochs;
                        tight_dd_fixed_ambiguities += dd_result.partial_ar.fixed_count;
                    }
                    if (dd_result.reset_action !=
                        libgnss::dd_imu_bridge::SoftResetAction::REJECTED) {
                        ++tight_dd_soft_resets;
                    }
                }
            }
            ++valid_solutions;
            if (pos_solution.isFixed()) {
                ++fixed_solutions;
            }
        }

        if (fusion_processor.isOriginSet()) {
            fused_solution.addSolution(fusion_processor.toPositionSolution());
            attitude_writer.write(fusion_processor.state().nominal.time,
                                  fusion_processor.state().nominal.attitude_body_to_enu);
        }

        if (options.verbose && (valid_solutions <= 5 || valid_solutions % 200 == 0)) {
            std::cout << "epoch " << processed_epochs << " tow=" << std::fixed
                      << std::setprecision(3) << pos_solution.time.tow
                      << " rtk_status=" << static_cast<int>(pos_solution.status)
                      << " sats=" << pos_solution.num_satellites
                      << " ratio=" << std::setprecision(2) << pos_solution.ratio << "\n";
        }

        const Eigen::Vector3d saved_rover_pos = rover_obs.receiver_position;
        rover_ok = rover_reader.readObservationEpoch(rover_obs);
        if (rover_ok) {
            if (pos_solution.isFixed()) {
                rover_obs.receiver_position = pos_solution.position_ecef;
            } else {
                rover_obs.receiver_position = saved_rover_pos;
            }
        }
        processed_epochs++;
    }

    if (!fused_solution.writeToFile(options.out_path)) {
        std::cerr << "Error: failed to write fused solution file: " << options.out_path << "\n";
        return 1;
    }
    if (options.write_kml && !fused_solution.writeKML(options.kml_path)) {
        std::cerr << "Error: failed to write KML file: " << options.kml_path << "\n";
        return 1;
    }

    if (!options.quiet) {
        std::cout << "GNSS half: RTK (--base " << options.base_path << ")\n";
        if (!options.rtk_preset.empty()) {
            std::cout << "RTK preset: " << options.rtk_preset << "\n";
        }
        std::cout << "IMU samples loaded: " << imu_series.samples.size() << "\n";
        std::cout << "Rover epochs processed: " << processed_epochs << "\n";
        std::cout << "Valid RTK solutions: " << valid_solutions << "\n";
        std::cout << "Fixed RTK solutions: " << fixed_solutions;
        if (valid_solutions > 0) {
            std::cout << " (" << std::fixed << std::setprecision(2)
                      << (100.0 * fixed_solutions / valid_solutions) << "%)";
        }
        std::cout << "\n";
        std::cout << "Exact base epochs: " << exact_base_epochs << "\n";
        std::cout << "Interpolated base epochs: " << interpolated_base_epochs << "\n";
        std::cout << "Skipped rover epochs (no aligned base): " << skipped_rover_epochs << "\n";
        std::cout << "Derived FIXED-to-FIXED velocity updates: " << derived_velocity_updates << "\n";
        if (options.tightly_coupled_dd_imu) {
            std::cout << "Tight DD/IMU epochs: " << tight_dd_epochs
                      << " (accepted=" << tight_dd_accepted
                      << ", innovation_rejected=" << tight_dd_rejected << ")\n";
            std::cout << "Tight DD rows: " << tight_dd_rows << "\n";
            std::cout << "Carrier-to-code fallbacks: "
                      << tight_dd_carrier_fallbacks << "\n";
            std::cout << "Tight DD NIS/observation mean/max: "
                      << (tight_dd_nis_samples > 0
                              ? tight_dd_nis_sum / tight_dd_nis_samples
                              : 0.0)
                      << "/" << tight_dd_nis_max << "\n";
            std::cout << "Partial-AR epochs/fixed ambiguities: "
                      << tight_dd_partial_ar_epochs << "/"
                      << tight_dd_fixed_ambiguities << "\n";
            std::cout << "Innovation-gated soft resets: " << tight_dd_soft_resets << "\n";
        }
        std::cout << "Fused epochs written: " << fused_solution.size() << "\n";
        std::cout << "Fusion initialized: " << (fusion_processor.isInitialized() ? "yes" : "no") << "\n";
        // isHeadingAligned() only means "a latch has happened at some point"
        // -- it used to print "yes" even on runs where the latch was badly
        // wrong and never recovered (PPC nagoya/run1 investigation).
        // isHeadingConverged() is the real health signal: aligned AND the
        // recent velocity-innovation NIS still corroborates that latch.
        std::cout << "Heading aligned: " << (fusion_processor.isHeadingAligned() ? "yes" : "no") << "\n";
        std::cout << "Heading converged: " << (fusion_processor.isHeadingConverged() ? "yes" : "no") << "\n";
        std::cout << "Output: " << options.out_path << "\n";
        if (options.write_kml) {
            std::cout << "KML: " << options.kml_path << "\n";
        }
    }
    return 0;
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        const FuseOptions options = parseArguments(argc, argv);

        // Load IMU samples up front (this is the whole file: for the
        // PPC-Dataset's tokyo/run1, ~239k samples / ~2.4 MB in memory).
        libgnss::ImuSeries imu_series;
        const auto imu_result = libgnss::loadImuCsv(options.imu_path, imu_series);
        if (!imu_result.ok) {
            std::cerr << "Error: failed to load IMU CSV: " << imu_result.error << "\n";
            return 1;
        }
        imu_series.sortByTime();

        // Default axis convention is identity (raw sensor axes already FLU),
        // matching the PPC-Dataset default per docs/design.md 1.1.1. A
        // future CLI revision could expose --forward-axis/--up-axis/etc. if
        // a non-PPC, non-FLU dataset needs it.
        const libgnss::ImuAxisConvention axis_convention;

        libgnss::io::RINEXReader nav_reader;
        if (!nav_reader.open(options.nav_path)) {
            std::cerr << "Error: failed to open navigation file: " << options.nav_path << "\n";
            return 1;
        }
        libgnss::NavigationData nav_data;
        if (!nav_reader.readNavigationData(nav_data)) {
            std::cerr << "Error: failed to read navigation data: " << options.nav_path << "\n";
            return 1;
        }

        if (!options.base_path.empty()) {
            return runRtkFusion(options, imu_series, axis_convention, nav_data);
        }
        return runSppFusion(options, imu_series, axis_convention, nav_data);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
