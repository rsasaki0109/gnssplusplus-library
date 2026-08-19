#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <libgnss++/algorithms/rtk.hpp>
#include <libgnss++/algorithms/spp.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/solution.hpp>
#include <libgnss++/fusion/attitude.hpp>
#include <libgnss++/fusion/fusion_processor.hpp>
#include <libgnss++/fusion/preintegration.hpp>
#include <libgnss++/fusion/tight_coupling_processor.hpp>
#include <libgnss++/io/imu.hpp>
#include <libgnss++/io/rinex.hpp>

#include "cli_toml_config.hpp"
#include "rtk_base_epoch_align.hpp"

namespace {

constexpr double kExactTimeToleranceSeconds = libgnss_apps::kExactTimeToleranceSeconds;

Eigen::Matrix3d ecefToEnuRotation(double lat, double lon) {
    Eigen::Matrix3d rotation;
    rotation.col(0) = libgnss::ecef2enu(Eigen::Vector3d::UnitX(), lat, lon);
    rotation.col(1) = libgnss::ecef2enu(Eigen::Vector3d::UnitY(), lat, lon);
    rotation.col(2) = libgnss::ecef2enu(Eigen::Vector3d::UnitZ(), lat, lon);
    return rotation;
}

enum class DisjointPartitionScheme {
    GRJ_EC,
    GEJ_RC,
    GCJ_RE,
    GJ_ERC,
};

struct FuseOptions {
    std::string data_dir;
    std::string rover_path;
    std::string base_path;
    std::string nav_path;
    std::string gnss_pos_path;
    std::string imu_path;
    bool rtklibexplorer_imu_format = false;
    bool imu_mount_rotation_enabled = false;
    Eigen::Matrix3d imu_raw_to_body_flu = Eigen::Matrix3d::Identity();
    std::string out_path = "output/fused_solution.pos";
    std::string kml_path;
    bool write_kml = false;
    std::string attitude_csv_path;  ///< optional roll/pitch/yaw debug export (validation-only)
    std::string sse_par_csv_path;
    std::string library_fix_integrity_csv_path;

    // RTK tuning (only consulted when --base is provided; see
    // docs/design.md 5 -- "same defaults/presets" as the `solve` command).
    std::string rtk_preset;
    bool base_position_override = false;
    Eigen::Vector3d base_position_ecef = Eigen::Vector3d::Zero();
    double max_baseline_length_m = 20000.0;
    double ratio_threshold = 3.0;
    bool ratio_threshold_set = false;
    libgnss::RTKProcessor::RTKConfig::ARPolicy ar_policy =
        libgnss::RTKProcessor::RTKConfig::ARPolicy::EXTENDED;
    libgnss::RTKProcessor::RTKConfig::GlonassARMode glonass_ar_mode =
        libgnss::RTKProcessor::RTKConfig::GlonassARMode::OFF;
    double max_position_jump_m = 5.0;
    double rtk_update_outlier_threshold = 0.0;
    bool prefer_trusted_seed = false;
    std::string rover_seed_pos_path;
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
    bool integrity_fixed_velocity = false;
    bool tightly_coupled_dd_imu = false;
    bool tightly_coupled_dd_code_only = false;
    bool library_fix_integrity_gate = false;
    double integrity_anchor_max_age_s = 2.0;
    DisjointPartitionScheme disjoint_partition_scheme =
        DisjointPartitionScheme::GRJ_EC;
    bool disjoint_partition_ensemble = false;
    bool integrity_disjoint_evaluate_all = false;
    bool integrity_student_t_front_end = false;
    bool integrity_student_t_all_measurements = false;
    bool integrity_laplacian_all_measurements = false;
    bool integrity_huber_all_measurements = false;
    double integrity_student_t_degrees_of_freedom = 4.0;
    std::string integrity_disjoint_heavy_tail_model = "inherit";
    double integrity_heavy_tail_activation_sigma = 2.5;
    double integrity_validator_runtime_budget_ms = 0.0;
    double integrity_max_statistical_separation_m = 1.0;
    int integrity_disjoint_acquisition_streak = 3;
    double integrity_disjoint_correction_jump_m = 0.02;
    bool integrity_disjoint_selected_pair_ratio = false;
    bool integrity_causal_arc_readiness_shadow = false;
    bool integrity_causal_arc_smoothed_search = false;
    int integrity_causal_arc_smoothed_max_pairs = 0;
    bool integrity_causal_arc_promotion = false;
    bool integrity_satellite_par_consensus_shadow = false;
    bool integrity_satellite_par_consensus_promotion = false;
    bool integrity_src_par_consensus_shadow = false;
    bool integrity_src_par_consensus_promotion = false;
    bool integrity_inertial_referenced_consensus_shadow = false;
    bool integrity_inertial_referenced_consensus_promotion = false;
    bool integrity_satellite_par_quality_diverse = false;
    bool integrity_satellite_par_persistent_subset = false;
    bool integrity_disjoint_satellite_par = false;
    int integrity_satellite_par_max_drop_steps = 8;
    bool integrity_wide_lane_front_end = false;
    bool integrity_l1_l2_wlnl_cascade = false;
    bool integrity_l2_l5_wlnl_cascade = false;
    bool integrity_multifrequency_consensus_shadow = false;
    bool integrity_multifrequency_consensus_promotion = false;

    // Phase 1 GNSS/IMU coupling (docs/design.md): feed the ESKF's INS-
    // mechanization-predicted antenna ECEF position back into RTKProcessor
    // as a KINEMATIC-epoch prior, in place of the SPP-only reseed, when the
    // fusion filter is healthy/aligned. Opt-in, off by default -- see
    // runRtkFusion()'s use of these two options for the exact wiring and
    // health gating.
    bool rtk_ins_prior = false;
    // Multiplier applied to the ESKF's own predicted-position covariance
    // before handing it to RTKProcessor as var_pos. Conservative (>> 1) by
    // default: the prior only needs to be tight enough to beat the legacy
    // 900 m^2 kinematic-reset variance during a genuine SPP-degraded
    // stretch (bridges/tunnels/urban canyon), while staying soft enough
    // that a biased ESKF prediction can't dominate the DD phase/code
    // measurement update and entrench itself via the loose-coupling
    // feedback loop (ESKF <- RTK solution <- ESKF prior <- ...).
    double rtk_ins_prior_inflation = 25.0;

    // Phase 1b redesign (docs/imu_fusion.md): Phase 1 v1 injected the prior
    // on every eligible (initialized/anchored/heading-converged) epoch and
    // that regressed at every tested inflation on PPC tokyo/run1 -- the
    // ESKF's predicted mean carries a persistent multi-meter bias that trips
    // RTKProcessor's own magnitude-based jump/reacquisition gates each time
    // it seeds a healthy epoch that RTK itself would have solved just fine.
    // Two additional gates, both on by default whenever --rtk-ins-prior is
    // given, restrict injection to the degraded-GNSS epochs the prior
    // actually exists for (bridges/tunnels/urban canyon):
    //
    // 1. SPP-degraded gate (rtk_ins_prior_spp_gate): only inject when the
    //    current epoch's raw rover observation satellite count is below
    //    rtk_ins_prior_min_sats. This is deliberately the raw pre-solve
    //    count from ObservationData::getNumSatellites(), not a lagged
    //    previous-epoch RTK/SPP solution status: at the point in the epoch
    //    loop where the prior must be handed to RTKProcessor (before
    //    processRTKEpoch() runs its internal resetPositionToSPP() time
    //    update), no current-epoch SPP/RTK quality figure exists yet without
    //    duplicating the solve. Raw satellite count is available immediately,
    //    with zero lag, and is exactly the signal that collapses during a
    //    genuine sky-blockage event. Note this raw count (unique satellites
    //    with ANY observation in the epoch, pre elevation-mask/QC) runs much
    //    higher than PositionSolution::num_satellites (the post-QC count
    //    used-in-solution, mean ~17-18 on this dataset): on PPC tokyo/run1
    //    the epochs eligible for injection (isInitialized/isOriginSet/
    //    isHeadingConverged) have a raw-count median of 32 and p10 of ~24 --
    //    calibrated empirically (docs/imu_fusion.md-style validation, see
    //    this feature's gate verification) by sweeping the threshold and
    //    picking the value that fires on a small (~1-5%) fraction of
    //    eligible epochs rather than 0% or a large fraction.
    // 2. ESKF health gate (rtk_ins_prior_max_nis): only inject when the
    //    ESKF's own velocity-update NIS-per-observation EMA (the same signal
    //    backing isHeadingConverged(), see LooseCouplingProcessor::
    //    getVelocityNisEma()) is below this threshold, i.e. the predicted
    //    mean is currently being corroborated by GNSS velocity innovations
    //    and is less likely to carry the kind of bias v1 tripped over.
    //    <= 0 disables this gate. Deliberately tighter than
    //    isHeadingConverged()'s own heading_recovery_nis_threshold (30.0 by
    //    default): that threshold answers "is yaw still trustworthy", this
    //    one answers "is the filter tight enough right now to seed a
    //    position", a stricter bar.
    //
    // --rtk-ins-prior-always restores exact v1 behavior (unconditional
    // injection on every isInitialized/isOriginSet/isHeadingConverged epoch)
    // for A/B comparison, by disabling both gates at once.
    bool rtk_ins_prior_spp_gate = true;
    int rtk_ins_prior_min_sats = 20;
    double rtk_ins_prior_max_nis = 3.0;

    // M1 RTK-hosted tight coupling: mechanize an incremental antenna
    // displacement from RTK's FLOAT posterior and use it in place of the
    // per-epoch SPP position wipe. Independent of the Phase 1 absolute
    // position prior above; the two modes are mutually exclusive.
    bool tc_ins_time_update = false;
    bool tc_closed_loop = false;
    bool tc_trusted_reanchor = false;
    int tc_trusted_reanchor_max_epochs = 0;
    bool tc_velocity_states = false;
    // navi.776 B2: SD Doppler rows making the M4 velocity states directly
    // observable. Requires --tc-velocity-states.
    bool tc_doppler_rows = false;
    bool tc_reuse_update_factorization = false;
    bool tc_sequential_doppler_update = false;
    double tc_doppler_sigma_mps = 0.2;
    double tc_doppler_max_baseline_m = 0.0;
    // navi.776 C: constant GNSS-IMU time offset (s), applied to all IMU
    // timestamps right after load. Positive = IMU later.
    double imu_time_offset_s = 0.0;
    int imu_time_offset_score_start_epoch = 0;
    bool tc_tdcp_diagnostics = false;
    double tc_ins_position_q_floor_m2 = 25.0;
    double tc_ins_max_sample_gap_s = 0.1;
    bool tc_cp_pr_gate = false;
    double tc_cp_pr_threshold_m = 10.0;
    int tc_cp_pr_min_pairs = 4;
    int tc_cp_pr_max_bad_pairs = 1;
    int tc_cp_pr_escalation_epochs = 2;
    double tc_ddpr_fde_threshold_m = 10.0;
    int tc_ddpr_max_fde_removals = 3;

    // RTK tuning passthroughs mirroring apps/gnss_solve.cpp's flags of the
    // same name, added so gnss_fuse's underlying RTKProcessor config can be
    // made to match a `gnss solve` run exactly (docs/design.md Phase 1 gate
    // 2 verification: scoring the pre-fusion RTK stream against the
    // gnss_solve baseline requires the same RTK tuning on both sides).
    // Preset-set-then-explicit-override semantics only matter for
    // enable_ar_filter (the only field below any preset also touches --
    // see rtk_base_epoch_align.hpp's applyRtkConfigPreset()); the SNR/
    // subset-AR fields are never touched by a preset, so they can just be
    // assigned unconditionally.
    bool arfilter_override = false;
    bool arfilter_value = false;
    bool rtk_snr_weighting = false;
    double rtk_snr_reference_dbhz = 45.0;
    double rtk_snr_max_variance_scale = 25.0;
    int max_subset_ar_drop_steps = -1;  // < 0 leaves RTKConfig's own default (6)

    // navi.776 A2: innovation-based adaptive measurement variance, mirroring
    // gnss_solve's --rtk-adaptive-noise family. Off by default.
    bool rtk_adaptive_noise = false;
    double rtk_adaptive_noise_alpha_phase = 0.9;
    double rtk_adaptive_noise_alpha_code = 0.5;
    double rtk_adaptive_noise_min_scale = 0.25;
    double rtk_adaptive_noise_max_scale = 25.0;
    double rtk_adaptive_noise_max_baseline_m = 0.0;
    bool rtk_adaptive_noise_phase_only = false;
    bool rtk_adaptive_noise_per_system_alpha = false;

    // Phase 2a: CMC-aware DD reference-satellite selection with hysteresis
    // (RTKConfig::cmc_aware_reference_selection), mirroring gnss_solve's
    // --cmc-ref family so gnss_fuse's underlying RTKProcessor can be tuned
    // identically. Off by default; see rtk.hpp's doc comment for the
    // algorithm.
    bool cmc_aware_reference_selection = false;
    double cmc_ref_level_m = 0.75;
    int cmc_ref_switch_epochs = 3;
    double cmc_ref_return_min_elev_deg = 5.0;
    double cmc_ref_switch_max_elev_drop_deg = 10.0;
    double cmc_ref_switch_min_elev_deg = 30.0;

    // Opt-in: also write the per-epoch RTK (pre-fusion) PositionSolution
    // stream to its own .pos file, in the same libgnss::Solution format
    // `gnss solve` writes, so the existing PPC scoring pipeline can score
    // the RTK solution in isolation from the ESKF's own smoothing/accuracy
    // characteristics (docs/design.md Phase 1 gate 2). Empty (default): off.
    std::string rtk_pos_out;

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

Eigen::Matrix3d rtklibExplorerRawToBodyFlu(const Eigen::Vector3d& rpy_deg) {
    constexpr double kDegreesToRadians = M_PI / 180.0;
    const Eigen::Vector3d rpy = rpy_deg * kDegreesToRadians;
    const double sphi = std::sin(rpy.x());
    const double cphi = std::cos(rpy.x());
    const double stheta = std::sin(rpy.y());
    const double ctheta = std::cos(rpy.y());
    const double spsi = std::sin(rpy.z());
    const double cpsi = std::cos(rpy.z());

    // Exact Euler_to_CTM convention used by upstream GNSS_IMU.py: raw
    // sensor axes -> body FRD. The diagonal then converts FRD to FLU.
    Eigen::Matrix3d raw_to_body_frd;
    raw_to_body_frd
        << ctheta * cpsi, ctheta * spsi, -stheta,
        -cphi * spsi + sphi * stheta * cpsi,
        cphi * cpsi + sphi * stheta * spsi, sphi * ctheta,
        sphi * spsi + cphi * stheta * cpsi,
        -sphi * cpsi + cphi * stheta * spsi, cphi * ctheta;
    return Eigen::Vector3d(1.0, -1.0, -1.0).asDiagonal() * raw_to_body_frd;
}

void transformImuSample(libgnss::ImuSample& sample, const FuseOptions& options,
                        const libgnss::ImuAxisConvention& axis_convention) {
    if (options.imu_mount_rotation_enabled) {
        sample.accel_raw = options.imu_raw_to_body_flu * sample.accel_raw;
        sample.gyro_raw_radps =
            options.imu_raw_to_body_flu * sample.gyro_raw_radps;
        return;
    }
    sample.accel_raw = axis_convention.apply(sample.accel_raw);
    sample.gyro_raw_radps = axis_convention.apply(sample.gyro_raw_radps);
}

bool belongsToDisjointPartitionA(
    libgnss::GNSSSystem system,
    DisjointPartitionScheme scheme) {
    using libgnss::GNSSSystem;
    switch (scheme) {
        case DisjointPartitionScheme::GRJ_EC:
            return system == GNSSSystem::GPS ||
                   system == GNSSSystem::GLONASS ||
                   system == GNSSSystem::QZSS;
        case DisjointPartitionScheme::GEJ_RC:
            return system == GNSSSystem::GPS ||
                   system == GNSSSystem::Galileo ||
                   system == GNSSSystem::QZSS;
        case DisjointPartitionScheme::GCJ_RE:
            return system == GNSSSystem::GPS ||
                   system == GNSSSystem::BeiDou ||
                   system == GNSSSystem::QZSS;
        case DisjointPartitionScheme::GJ_ERC:
            return system == GNSSSystem::GPS ||
                   system == GNSSSystem::QZSS;
    }
    return false;
}

bool belongsToDisjointPartitionB(
    libgnss::GNSSSystem system,
    DisjointPartitionScheme scheme) {
    return !belongsToDisjointPartitionA(system, scheme) &&
           system != libgnss::GNSSSystem::UNKNOWN &&
           system != libgnss::GNSSSystem::SBAS &&
           system != libgnss::GNSSSystem::NavIC;
}

libgnss::ObservationData filterDisjointPartition(
    const libgnss::ObservationData& input,
    bool partition_a,
    DisjointPartitionScheme scheme) {
    libgnss::ObservationData filtered(input.time);
    filtered.receiver_position = input.receiver_position;
    filtered.receiver_clock_bias = input.receiver_clock_bias;
    filtered.observations.reserve(input.observations.size());
    for (const auto& observation : input.observations) {
        const bool keep =
            partition_a
                ? belongsToDisjointPartitionA(
                      observation.satellite.system, scheme)
                : belongsToDisjointPartitionB(
                      observation.satellite.system, scheme);
        if (keep) {
            filtered.observations.push_back(observation);
        }
    }
    return filtered;
}

bool observationInputsAreDisjoint(
    const libgnss::ObservationData& partition_a,
    const libgnss::ObservationData& partition_b) {
    std::set<libgnss::SatelliteId> satellites_a;
    for (const auto& observation : partition_a.observations) {
        satellites_a.insert(observation.satellite);
    }
    for (const auto& observation : partition_b.observations) {
        if (satellites_a.count(observation.satellite) != 0) {
            return false;
        }
    }
    return true;
}

double roundedTowKey(double tow) {
    return std::round(tow * 10.0) / 10.0;
}

std::map<double, Eigen::Vector3d> loadSeedPositions(
    const std::string& path) {
    std::ifstream input(path);
    if (!input.is_open()) {
        throw std::runtime_error(
            "failed to open rover seed .pos: " + path);
    }
    std::map<double, Eigen::Vector3d> output;
    std::string line;
    while (std::getline(input, line)) {
        if (line.empty() || line[0] == '%') continue;
        std::istringstream stream(line);
        double week = 0.0;
        double tow = 0.0;
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        if (!(stream >> week >> tow >> position.x() >>
              position.y() >> position.z())) {
            continue;
        }
        if (std::isfinite(tow) && position.allFinite() &&
            position.norm() > 1e6) {
            output[roundedTowKey(tow)] = position;
        }
    }
    if (output.empty()) {
        throw std::runtime_error(
            "rover seed .pos contained no usable ECEF rows: " +
            path);
    }
    return output;
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
        << " --data-dir <run-dir> [--navi776-tc] [options]\n"
        << "       " << program_name
        << " --rover <rover.obs> --nav <nav.rnx> --imu <imu.csv>"
           " [--base <base.obs>] [options]\n\n"
        << "GNSS/IMU fusion with in-process SPP or RTK positioning.\n"
        << "Use --data-dir when a run directory contains rover.obs, base.obs,\n"
        << "base.nav, and imu.csv. Existing advanced flags remain supported.\n\n"
        << "Inputs:\n"
        << "  --data-dir <dir>        Load the standard run files from one directory\n"
        << "  --rover <path>          Rover RINEX observation file\n"
        << "  --base <path>           Base RINEX observation file; enables RTK\n"
        << "  --nav <path>            RINEX navigation file\n"
        << "  --imu <path>            IMU CSV file\n"
        << "  --gnss-pos <path>       Use an existing position/velocity solution\n\n"
        << "Common options:\n"
        << "  --config <path>         Load flat TOML defaults; CLI options override them\n"
        << "  --navi776-tc            Validated short-baseline tight-coupling preset\n"
        << "  --lever-arm x,y,z       IMU-to-antenna lever arm, body FLU, meters\n"
        << "  --preset <name>         RTK preset: survey|low-cost|moving-base|odaiba\n"
        << "  --ratio <value>         RTK ambiguity ratio threshold (default: 3.0)\n"
        << "  --max-epochs <n>        Stop after n GNSS epochs (0 = no limit)\n"
        << "  --out <path>            Fused solution output"
           " (default: output/fused_solution.pos)\n"
        << "  --rtk-pos-out <path>    Optional pre-fusion RTK solution output\n"
        << "  --kml <path>            Optional KML trajectory output\n"
        << "  --verbose               Print periodic progress\n"
        << "  --quiet                 Suppress the run summary\n\n"
        << "Help:\n"
        << "  -h, --help              Show this everyday-use help\n"
        << "  --help-advanced         Show every tuning, experiment, and diagnostic option\n\n"
        << "Recommended PPC invocation:\n"
        << "  " << program_name
        << " --data-dir <run-dir> --lever-arm x,y,z --preset low-cost"
           " --navi776-tc --rtk-pos-out output/rtk.pos\n";
}

void printAdvancedUsage(const char* program_name) {
    std::cout
        << "Usage: " << program_name
        << " --rover <rover.obs> --nav <nav.rnx> --imu <imu.csv> [--base <base.obs>] --out <fused.pos>\n"
        << "Loosely-coupled GNSS/IMU fusion (Stage 1, docs/design.md): a 15-state\n"
        << "error-state EKF mechanizes 100 Hz IMU samples and opportunistically\n"
        << "corrects with a position solution computed in-process from --rover/--nav\n"
        << "(SPP), or from --rover/--base/--nav (RTK, same RTKProcessor pipeline the\n"
        << "`solve` command uses) when --base is supplied.\n\n"
        << "Options:\n"
        << "  --config <path>              Load flat TOML defaults from [gnss_fuse]. CLI\n"
        << "                                options always override config values\n"
        << "  --data-dir <dir>             Use <dir>/rover.obs, base.obs, base.nav, imu.csv as\n"
        << "                                defaults for --rover/--base/--nav/--imu\n"
        << "  --rover <rover.obs>          RINEX observation file (required)\n"
        << "  --gnss-pos <solution.pos>    Read a LibGNSS++/RTKLIB position+velocity log\n"
        << "                                directly instead of solving --rover/--nav\n"
        << "  --base <base.obs>            RINEX base observation file. When given, gnss_fuse\n"
        << "                                runs the RTKProcessor pipeline (same as `solve`) and\n"
        << "                                feeds its per-epoch PositionSolution to the fusion\n"
        << "                                filter instead of an SPP-only solution.\n"
        << "  --nav <nav.rnx>              RINEX navigation file (required)\n"
        << "  --imu <imu.csv>              IMU CSV file (required)\n"
        << "  --imu-format <format>        ppc (default) or rtklibexplorer-sf. The latter\n"
        << "                                converts GPST Unix seconds, accel g, gyro rad/s\n"
        << "  --imu-misalignment-rpy-deg r,p,y\n"
        << "                                Upstream Euler_to_CTM raw -> body-FRD rotation,\n"
        << "                                followed by FRD -> FLU conversion\n"
        << "  --out <fused.pos>            Output position file (default: output/fused_solution.pos)\n"
        << "  --kml <fused.kml>            Optional KML trajectory output\n"
        << "  --attitude-csv <path>        Optional roll/pitch/yaw (deg) debug CSV, one row per\n"
        << "                                fused epoch (validation against reference.csv attitude)\n"
        << "  --tight-dd-sse-csv <path>    Optional truth-free per-epoch SSE-PAR audit CSV\n"
        << "  --library-fix-integrity-gate Require primary FFRT plus an independent\n"
        << "                                L1/L5 or causal pre-update INS source before\n"
        << "                                emitting library Status=4 (default: off)\n"
        << "  --library-fix-integrity-csv <path>\n"
        << "                                Optional truth-free per-epoch gate audit CSV\n"
        << "  --integrity-anchor-max-age-s <s>\n"
        << "                                Maximum causal age of an independently accepted\n"
        << "                                FIX anchor for INS separation (default: 2)\n"
        << "  --integrity-disjoint-partition <grj-ec|gej-rc|gcj-re|gj-erc>\n"
        << "                                Fixed non-overlapping GNSS-system split\n"
        << "                                used by independent RTK validators\n"
        << "  --integrity-disjoint-ensemble Evaluate all four fixed splits and\n"
        << "                                select the available split with the\n"
        << "                                smallest truth-free A/B separation\n"
        << "  --integrity-disjoint-evaluate-all\n"
        << "                                Do not early-stop after a hard A/B match\n"
        << "  --integrity-student-t-front-end\n"
        << "                                Enable code-only Student-t covariance\n"
        << "                                inflation before FLOAT updates; it has\n"
        << "                                no direct integer/FIX authority\n"
        << "  --integrity-student-t-all-measurements\n"
        << "                                Apply Student-t inflation to code and\n"
        << "                                carrier rows (implies front-end enable)\n"
        << "  --integrity-student-t-degrees-of-freedom <nu>\n"
        << "                                Student-t tail parameter (default: 4)\n"
        << "  --integrity-laplacian-all-measurements\n"
        << "                                Use an L1/Laplacian IRLS front-end on\n"
        << "                                code and carrier rows\n"
        << "  --integrity-huber-all-measurements\n"
        << "                                Use a continuous Huber IRLS front-end on\n"
        << "                                code and carrier rows\n"
        << "  --integrity-heavy-tail-activation-sigma <sigma>\n"
        << "                                Residual threshold for heavy-tail\n"
        << "                                covariance inflation (default: 2.5)\n"
        << "  --integrity-disjoint-heavy-tail-model <inherit|student-t|laplacian|huber>\n"
        << "                                Optional robust model override for\n"
        << "                                disjoint validators (default: inherit)\n"
        << "  --integrity-validator-runtime-budget-ms <ms>\n"
        << "                                Stop before another fallback validator\n"
        << "                                pair once this epoch budget is reached\n"
        << "  --integrity-max-statistical-separation-m <m>\n"
        << "                                Gross three-solution bound used before\n"
        << "                                causal consensus (default: 1.0)\n"
        << "  --integrity-disjoint-acquisition-streak <n>\n"
        << "                                Consecutive stable consensus epochs\n"
        << "                                required for declaration (default: 3)\n"
        << "  --integrity-disjoint-correction-jump-m <m>\n"
        << "                                Maximum correction change within that\n"
        << "                                streak (default: 0.02)\n"
        << "  --integrity-disjoint-selected-pair-ratio\n"
        << "                                Apply the unchanged absolute ratio gate\n"
        << "                                to both estimators in the selected pair\n"
        << "  --integrity-causal-arc-readiness-shadow\n"
        << "                                Audit reference-generation-aware float\n"
        << "                                ambiguity arc readiness; no FIX authority\n"
        << "  --integrity-causal-arc-smoothed-search-shadow\n"
        << "                                Search ready arcs with their causal float\n"
        << "                                means/covariance; no FIX authority\n"
        << "  --integrity-causal-arc-smoothed-max-pairs <n>\n"
        << "                                Covariance-only PAR cap for smoothed\n"
        << "                                search (0=all, minimum 12)\n"
        << "  --integrity-causal-arc-promotion\n"
        << "                                Promote only a declared arc/disjoint\n"
        << "                                causal consensus (implies shadow)\n"
        << "  --integrity-satellite-par-consensus-shadow\n"
        << "                                Audit FFRT satellite-PAR against both\n"
        << "                                disjoint validators; no FIX authority\n"
        << "  --integrity-satellite-par-consensus-promotion\n"
        << "                                Promote only declared PAR/disjoint\n"
        << "                                causal consensus (implies shadow)\n"
        << "  --integrity-src-par-consensus-shadow\n"
        << "                                Audit covariance-scale-16 SRC-PAR\n"
        << "                                against both disjoint validators\n"
        << "  --integrity-src-par-consensus-promotion\n"
        << "                                Promote only declared SRC/disjoint\n"
        << "                                causal consensus (implies shadow)\n"
        << "  --integrity-inertial-referenced-consensus-shadow\n"
        << "                                Audit primary FFRT candidate against\n"
        << "                                prior-anchor causal INS over 3 epochs\n"
        << "  --integrity-inertial-referenced-consensus-promotion\n"
        << "                                Promote only declared FFRT/INS causal\n"
        << "                                consensus (implies shadow)\n"
        << "  --integrity-satellite-par-quality-diverse\n"
        << "                                Rank PAR drop subsets by covariance,\n"
        << "                                elevation, SNR, residual, and azimuth\n"
        << "  --integrity-satellite-par-persistent-subset-shadow\n"
        << "                                Re-evaluate the prior accepted satellite\n"
        << "                                IDs first with fresh LAMBDA/FFRT\n"
        << "  --integrity-disjoint-satellite-par-shadow\n"
        << "                                Allow each non-overlapping validator to\n"
        << "                                use its own fresh FFRT PAR fallback\n"
        << "  --integrity-satellite-par-max-drop-steps <n>\n"
        << "                                Maximum satellite subsets tried by main\n"
        << "                                and partition PAR (default: 8)\n"
        << "  --integrity-wide-lane-front-end\n"
        << "                                Enable L1/L2 Melbourne-Wubbena wide-lane\n"
        << "                                conditioning before main integer search\n"
        << "  --integrity-l1-l2-wlnl-cascade-shadow\n"
        << "                                Run the two-stage FFRT WL/NL cascade on\n"
        << "                                L1/L2 instead of sparse L1/L5\n"
        << "  --integrity-l2-l5-wlnl-cascade-shadow\n"
        << "                                Add the long-wide-lane L2/L5 two-stage\n"
        << "                                FFRT cascade as the same fault family\n"
        << "  --integrity-multifrequency-consensus-shadow\n"
        << "                                Audit dual WL/NL candidates against both\n"
        << "                                disjoint validators; no FIX authority\n"
        << "  --integrity-multifrequency-consensus-promotion\n"
        << "                                Promote only declared WL/NL/disjoint\n"
        << "                                causal consensus (implies shadow)\n"
        << "  --ar-policy <extended|demo5-continuous>\n"
        << "  --glonass-ar <off|on|autocal>\n"
        << "  --max-pos-jump <m>           RTK absolute position jump bound\n"
        << "  --rtk-update-outlier-threshold <sigma>\n"
        << "  --prefer-trusted-seed        Prefer trusted/current rover seed\n"
        << "  --rover-seed-pos <file>      Per-epoch ECEF seed .pos\n"
        << "  --integrity-fixed-velocity   Feed causal velocity from consecutive\n"
        << "                                independently accepted FIX positions to INS\n"
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
        << "  --rtk-ins-prior               Phase 1 GNSS/IMU coupling (--base only): feed the\n"
        << "                                ESKF's INS-predicted antenna ECEF position into\n"
        << "                                RTKProcessor as a KINEMATIC-epoch prior (replacing the\n"
        << "                                SPP-only reseed) whenever the fusion filter is\n"
        << "                                initialized, origin-anchored, and heading-converged.\n"
        << "                                Falls back to the legacy SPP reseed on any epoch\n"
        << "                                without a healthy prior (e.g. before alignment, or an\n"
        << "                                IMU gap). Default: off.\n"
        << "  --rtk-ins-prior-inflation <f> Multiplier applied to the ESKF's predicted-position\n"
        << "                                covariance before it seeds RTKProcessor's position\n"
        << "                                states (guards against the ESKF<-RTK<-ESKF feedback\n"
        << "                                loop entrenching a biased prediction). Only used with\n"
        << "                                --rtk-ins-prior. Default: 25.0\n"
        << "  --rtk-ins-prior-spp-gate      Only inject the prior on epochs where the raw rover\n"
        << "                                satellite count is below --rtk-ins-prior-min-sats\n"
        << "                                (the degraded-GNSS case the prior exists for). On by\n"
        << "                                default whenever --rtk-ins-prior is given; this flag is\n"
        << "                                an explicit no-op restating the default. See\n"
        << "                                --rtk-ins-prior-always to disable it.\n"
        << "  --rtk-ins-prior-min-sats <n>  Raw satellite count threshold for the SPP-degraded gate\n"
        << "                                (pre-QC ObservationData::getNumSatellites(), not the\n"
        << "                                post-QC PositionSolution::num_satellites -- runs much\n"
        << "                                higher, calibrate per-dataset). Default: 20\n"
        << "  --rtk-ins-prior-max-nis <f>   Only inject the prior when the ESKF's velocity-update\n"
        << "                                NIS-per-observation EMA is at or below this (the\n"
        << "                                filter's own health signal, see\n"
        << "                                LooseCouplingProcessor::getVelocityNisEma()). <=0\n"
        << "                                disables this gate. Default: 3.0\n"
        << "  --rtk-ins-prior-always        Restore Phase 1 v1 behavior: inject on every\n"
        << "                                isInitialized/isOriginSet/isHeadingConverged epoch,\n"
        << "                                disabling both the SPP-degraded gate and the NIS health\n"
        << "                                gate (for A/B comparison against the gated default).\n"
        << "  --tc-ins-time-update          M1 RTK-hosted tight coupling (--base only): replace the\n"
        << "                                kinematic SPP position wipe with an incremental IMU\n"
        << "                                antenna displacement anchored at RTK's FLOAT posterior.\n"
        << "                                Preserves position/ambiguity cross-covariance. Mutually\n"
        << "                                exclusive with --rtk-ins-prior. Default: off.\n"
        << "  --tc-ins-position-q-floor <m2>\n"
        << "                                Diagonal position process-noise floor added per applied\n"
        << "                                INS interval (default: 25 m^2).\n"
        << "  --tc-ins-max-sample-gap <s>  Invalidate an IMU interval containing a larger sample\n"
        << "                                gap and fall back to legacy RTK reseeding (default: 0.1 s).\n"
        << "  --tc-closed-loop             M3 TightCouplingProcessor path. Owns IMU intervals,\n"
        << "                                re-anchoring, ZUPT/NHC, and enables the M2 gate.\n"
        << "                                Mutually exclusive with --tc-ins-time-update. Default: off.\n"
        << "  --tc-trusted-reanchor        With --tc-closed-loop, re-anchor only after a FIXED RTK\n"
        << "                                epoch; bridge FLOAT/SPP epochs by continuing the IMU\n"
        << "                                prediction instead of copying their posterior. Default: off.\n"
        << "  --tc-trusted-reanchor-max-epochs <n>\n"
        << "                                Maximum consecutive FLOAT/SPP epochs bridged before\n"
        << "                                re-anchoring to the next available posterior (0 = no\n"
        << "                                limit). Requires --tc-trusted-reanchor. Default: 0.\n"
        << "  --tc-doppler-rows            navi.776: rover-only between-satellite SD Doppler\n"
        << "                                observation rows (velocity observability). Requires\n"
        << "                                --tc-velocity-states. Default: off.\n"
        << "  --tc-reuse-update-factorization\n"
        << "                                Reuse the Kalman LU for NIS/row diagnostics when\n"
        << "                                NIS gates are disabled. Default: off.\n"
        << "  --tc-sequential-doppler-update\n"
        << "                                Apply phase/code and Doppler blocks sequentially\n"
        << "                                to reduce dense KF cost. Default: off.\n"
        << "  --tc-doppler-sigma <mps>     SD Doppler row sigma in m/s (default: 0.2)\n"
        << "  --tc-doppler-max-baseline <m>\n"
        << "                                Build Doppler rows only while the float baseline is\n"
        << "                                at or below this many meters (default: 0 = no gate)\n"
        << "  --navi776-tc                 Enable the validated navi.776 short-baseline combo:\n"
        << "                                closed loop + velocity states + Doppler rows\n"
        << "                                (sigma 0.5, 1000 m gate) + gated adaptive noise\n"
        << "                                (1000 m). Flags given after this one override it.\n"
        << "  --imu-time-offset <s>        navi.776: constant GNSS-IMU time offset applied to\n"
        << "                                all IMU timestamps at load (positive = IMU later).\n"
        << "                                Default: 0.0 (exact no-op).\n"
        << "  --imu-time-offset-score-start-epoch <n>\n"
        << "                                With --gnss-pos, exclude the first n GNSS epochs\n"
        << "                                from J while retaining them as filter warm-up\n"
        << "  --tc-velocity-states         M4 ECEF velocity states appended after legacy RTK state.\n"
        << "                                Requires --tc-closed-loop. Default: off.\n"
        << "  --tc-tdcp-diagnostics        M5 measurement-neutral SD-TDCP vs Doppler diagnostics.\n"
        << "                                Requires --tc-velocity-states. Default: off.\n"
        << "  --tc-cp-pr-gate              M2 fixed-candidate CP-vs-PR gate (--base only). Vetoes\n"
        << "                                inconsistent integers before FIX feedback. Default: off.\n"
        << "  --tc-cp-pr-threshold <m>     Per-pair absolute innovation threshold (default: 10).\n"
        << "  --tc-cp-pr-min-pairs <n>     Minimum non-GLONASS DD pairs to evaluate (default: 4).\n"
        << "  --tc-cp-pr-max-bad-pairs <n> Allowed above-threshold pairs (default: 1).\n"
        << "  --tc-cp-pr-escalation <n>    Consecutive vetoes before DDPR-LS anchor (default: 2).\n"
        << "  --tc-ddpr-fde-threshold <m>  DDPR-LS outlier threshold (default: 10).\n"
        << "  --tc-ddpr-max-fde-removals <n> Maximum DDPR-LS row removals (default: 3).\n"
        << "  --arfilter / --no-arfilter   Force RTK subset-AR filter margin on/off, overriding\n"
        << "                                whatever --preset set (mirrors `gnss solve`'s flag).\n"
        << "  --rtk-snr-weighting          Inflate RTK observation variance for low-SNR links\n"
        << "                                (mirrors `gnss solve`; default: off)\n"
        << "  --rtk-snr-reference-dbhz <v> No inflation at/above this SNR (default: 45.0)\n"
        << "  --rtk-snr-max-variance-scale <v>\n"
        << "                                Clamp low-SNR variance inflation (default: 25.0)\n"
        << "  --max-subset-ar-drop-steps <n>\n"
        << "                                Max worst-variance DD pairs dropped during subset AR\n"
        << "                                (mirrors `gnss solve`; default: RTKConfig's own, 6)\n"
        << "  --rtk-pos-out <path>          Also write the per-epoch RTK (pre-fusion) solution\n"
        << "                                stream to <path>, in the same format `gnss solve`\n"
        << "                                writes -- lets the PPC scoring pipeline evaluate the\n"
        << "                                RTK solution alone, independent of the ESKF's own\n"
        << "                                smoothing (--base only; default: off)\n"
        << "  --cmc-ref                    Phase 2a: CMC-aware DD reference-satellite selection\n"
        << "                                with hysteresis (mirrors `gnss solve`; default: off)\n"
        << "  --cmc-ref-level <m>           CMC suspect-classification threshold in meters\n"
        << "                                (default: 0.75). No effect without --cmc-ref\n"
        << "  --cmc-ref-switch-epochs <k>   Consecutive suspect/non-suspect epochs required\n"
        << "                                before switching away/back (default: 3)\n"
        << "  --cmc-ref-return-min-elev <deg>\n"
        << "                                Elevation margin (degrees) required before a switch-\n"
        << "                                back is considered (default: 5.0)\n"
        << "  --cmc-ref-switch-max-elev-drop <deg>\n"
        << "                                Elevation-quality gate on switch-away only: only\n"
        << "                                switch if the replacement is within this many\n"
        << "                                degrees below the suspect reference (default: 10.0)\n"
        << "  --cmc-ref-switch-min-elev <deg>\n"
        << "                                Companion absolute floor for the switch-away\n"
        << "                                replacement's elevation (default: 30.0)\n"
        << "  --verbose                    Print periodic per-epoch progress\n"
        << "  --quiet                      Suppress run summary\n"
        << "  -h, --help                   Show concise everyday-use help\n"
        << "  --help-advanced              Show this complete option reference\n";
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

class SSEPartialARCsvWriter {
public:
    bool open(const std::string& path) {
        if (path.empty()) {
            return true;
        }
        file_.open(path);
        if (!file_.is_open()) {
            return false;
        }
        file_ << "gps_week,tow,available,attempted,subsets_evaluated,"
                 "ratio_passed_subsets,separation_rejected_subsets,passed,"
                 "fixed_count,dropped_count,ratio,bsr_qscale16,"
                 "ffrt_min_ratio,sse_statistic_per_dof,"
                 "position_separation_m,candidate_ecef_x,candidate_ecef_y,"
                 "candidate_ecef_z\n";
        return true;
    }

    void write(
        const libgnss::GNSSTime& time,
        const libgnss::dd_imu_bridge::SSEPartialARResult& result,
        const Eigen::Vector3d& candidate_ecef) {
        if (!file_.is_open()) {
            return;
        }
        file_ << time.week << "," << std::fixed << std::setprecision(3)
              << time.tow << "," << result.available << ","
              << result.attempted << "," << result.subsets_evaluated
              << "," << result.ratio_passed_subsets << ","
              << result.separation_rejected_subsets << ","
              << result.passed << "," << result.fixed_count << ","
              << result.dropped_count << "," << std::setprecision(9)
              << result.ratio << ","
              << result.bootstrapped_success_rate << ","
              << result.ffrt_minimum_ratio << ","
              << result.statistic_per_dof << ","
              << result.position_separation_m << ",";
        if (result.passed && candidate_ecef.allFinite()) {
            file_ << std::setprecision(4) << candidate_ecef.x() << ","
                  << candidate_ecef.y() << "," << candidate_ecef.z();
        } else {
            file_ << ",,";
        }
        file_ << "\n";
    }

private:
    std::ofstream file_;
};

class LibraryFixIntegrityCsvWriter {
public:
    bool open(const std::string& path) {
        if (path.empty()) return true;
        file_.open(path);
        if (!file_.is_open()) return false;
        file_ << "gps_week,tow,library_status,primary_fixed_applied,"
                 "independent_families,joint_failure_probability,"
                 "failure_budget_passed,inertial_available,"
                 "primary_ffrt_passed,primary_pair_count,"
                 "primary_full_ratio,"
                 "causal_arc_total_pairs,causal_arc_ready_pairs,"
                 "causal_arc_subset_pair_count,"
                 "causal_arc_resets,"
                 "causal_arc_subset_ratio,"
                 "causal_arc_subset_bsr,"
                 "causal_arc_subset_variance_min,"
                 "causal_arc_subset_variance_max,"
                 "causal_arc_subset_ffrt_min_ratio,"
                 "causal_arc_subset_ffrt_passed,"
                 "causal_arc_subset_primary_separation_m,"
                 "causal_arc_subset_second_position_delta_m,"
                 "causal_arc_subset_partition_a_separation_m,"
                 "causal_arc_subset_partition_b_separation_m,"
                 "causal_arc_disjoint_evidence_passed,"
                 "causal_arc_consensus_declared_fixed,"
                 "causal_arc_consensus_state,"
                 "causal_arc_consensus_acquisition_streak,"
                 "causal_arc_candidate_ecef_x,"
                 "causal_arc_candidate_ecef_y,"
                 "causal_arc_candidate_ecef_z,"
                 "satellite_par_ffrt_passed,"
                 "satellite_par_subset_size,"
                 "satellite_par_persistent_subset_attempted,"
                 "satellite_par_persistent_subset_used,"
                 "satellite_par_ratio,"
                 "satellite_par_disjoint_evidence_passed,"
                 "satellite_par_consensus_declared_fixed,"
                 "satellite_par_consensus_state,"
                 "satellite_par_consensus_acquisition_streak,"
                 "satellite_par_candidate_ecef_x,"
                 "satellite_par_candidate_ecef_y,"
                 "satellite_par_candidate_ecef_z,"
                 "src_par_ffrt_passed,"
                 "src_par_subset_size,"
                 "src_par_ratio,"
                 "src_par_second_position_delta_m,"
                 "src_par_disjoint_evidence_passed,"
                 "src_par_partition_a_separation_m,"
                 "src_par_partition_b_separation_m,"
                 "src_par_consensus_declared_fixed,"
                 "src_par_consensus_state,"
                 "src_par_consensus_acquisition_streak,"
                 "src_par_candidate_ecef_x,"
                 "src_par_candidate_ecef_y,"
                 "src_par_candidate_ecef_z,"
                 "float_update_nis_per_observation,"
                 "float_update_prefit_residual_rms_m,"
                 "safe_shadow_declared_fixed,"
                 "inertial_healthy_anchor,inertial_passed,"
                 "inertial_time_error_s,inertial_position_delta_m,"
                 "inertial_nis_per_dimension,"
                 "inertial_referenced_consensus_declared_fixed,"
                 "inertial_referenced_consensus_state,"
                 "inertial_referenced_consensus_acquisition_streak,"
                 "inertial_referenced_correction_x,"
                 "inertial_referenced_correction_y,"
                 "inertial_referenced_correction_z,"
                 "disjoint_available,"
                 "disjoint_inputs_verified,disjoint_a_ffrt_passed,"
                 "disjoint_b_ffrt_passed,disjoint_passed,"
                 "disjoint_partition_separation_m,"
                 "disjoint_a_primary_separation_m,"
                 "disjoint_b_primary_separation_m,"
                 "disjoint_hard_separation_passed,"
                 "disjoint_statistical_separation_passed,"
                 "disjoint_partition_nis_per_dimension,"
                 "disjoint_a_primary_nis_per_dimension,"
                 "disjoint_b_primary_nis_per_dimension,"
                 "disjoint_consensus_declared_fixed,"
                 "disjoint_consensus_state,"
                 "disjoint_consensus_acquisition_streak,"
                 "disjoint_consensus_selected_pair,"
                 "disjoint_consensus_selected_pair_min_ratio,"
                 "multifrequency_wl_ffrt_passed,"
                 "multifrequency_nl_ffrt_passed,"
                 "multifrequency_candidate_pair_count,"
                 "multifrequency_primary_separation_m,"
                 "multifrequency_disjoint_evidence_passed,"
                 "multifrequency_consensus_declared_fixed,"
                 "multifrequency_consensus_state,"
                 "multifrequency_consensus_acquisition_streak,"
                 "multifrequency_candidate_ecef_x,"
                 "multifrequency_candidate_ecef_y,"
                 "multifrequency_candidate_ecef_z,"
                 "l1_l2_multifrequency_wl_ffrt_passed,"
                 "l1_l2_multifrequency_nl_ffrt_passed,"
                 "l1_l2_multifrequency_candidate_pair_count,"
                 "l1_l2_multifrequency_primary_separation_m,"
                 "l1_l2_multifrequency_disjoint_evidence_passed,"
                 "l1_l2_multifrequency_consensus_declared_fixed,"
                 "l1_l2_multifrequency_consensus_state,"
                 "l1_l2_multifrequency_consensus_acquisition_streak,"
                 "l1_l2_multifrequency_candidate_ecef_x,"
                 "l1_l2_multifrequency_candidate_ecef_y,"
                 "l1_l2_multifrequency_candidate_ecef_z,"
                 "l2_l5_multifrequency_wl_ffrt_passed,"
                 "l2_l5_multifrequency_nl_ffrt_passed,"
                 "l2_l5_multifrequency_pair_count,"
                 "l2_l5_multifrequency_wl_ratio,"
                 "l2_l5_multifrequency_wl_ffrt_min_ratio,"
                 "l2_l5_multifrequency_mw_disagreements,"
                 "l2_l5_multifrequency_nl_ratio,"
                 "l2_l5_multifrequency_nl_ffrt_min_ratio,"
                 "l2_l5_multifrequency_candidate_pair_count,"
                 "l2_l5_multifrequency_candidate_ecef_x,"
                 "l2_l5_multifrequency_candidate_ecef_y,"
                 "l2_l5_multifrequency_candidate_ecef_z,"
                 "quality_gate_passed,"
                 "quality_gate_promoted,"
                 "quality_gate_disjoint_consensus_promoted,"
                 "quality_gate_causal_arc_promoted,"
                 "quality_gate_satellite_par_promoted,"
                 "quality_gate_src_par_promoted,"
                 "quality_gate_inertial_referenced_promoted,"
                 "quality_gate_multifrequency_promoted,"
                 "quality_gate_raw_partition_conflict,"
                 "quality_gate_demoted,"
                 "disjoint_validators_evaluated,"
                 "processing_runtime_ms\n";
        return true;
    }

    void write(
        const libgnss::PositionSolution& solution,
        const libgnss::RTKProcessor::EpochDebugTelemetry& telemetry,
        int disjoint_validators_evaluated,
        double processing_runtime_ms) {
        if (!file_.is_open()) return;
        auto number = [&](double value) {
            if (std::isfinite(value)) file_ << std::setprecision(12) << value;
        };
        const Eigen::Vector3d multifrequency_candidate(
            telemetry.lambda_l1_l5_wlnl_shadow_best_ecef_x,
            telemetry.lambda_l1_l5_wlnl_shadow_best_ecef_y,
            telemetry.lambda_l1_l5_wlnl_shadow_best_ecef_z);
        const Eigen::Vector3d l1_l2_multifrequency_candidate(
            telemetry.lambda_l1_l2_wlnl_shadow_best_ecef_x,
            telemetry.lambda_l1_l2_wlnl_shadow_best_ecef_y,
            telemetry.lambda_l1_l2_wlnl_shadow_best_ecef_z);
        const Eigen::Vector3d l2_l5_multifrequency_candidate(
            telemetry.lambda_l2_l5_wlnl_shadow_best_ecef_x,
            telemetry.lambda_l2_l5_wlnl_shadow_best_ecef_y,
            telemetry.lambda_l2_l5_wlnl_shadow_best_ecef_z);
        const Eigen::Vector3d primary_candidate(
            telemetry.lambda_shadow_best_ecef_x,
            telemetry.lambda_shadow_best_ecef_y,
            telemetry.lambda_shadow_best_ecef_z);
        const Eigen::Vector3d causal_arc_candidate(
            telemetry.lambda_causal_arc_subset_best_ecef_x,
            telemetry.lambda_causal_arc_subset_best_ecef_y,
            telemetry.lambda_causal_arc_subset_best_ecef_z);
        file_ << solution.time.week << "," << std::fixed
              << std::setprecision(3) << solution.time.tow << ","
              << static_cast<int>(solution.status) << ","
              << telemetry.final_fixed_applied << ","
              << telemetry.safe_fix_shadow_independent_source_families
              << ",";
        number(telemetry.safe_fix_shadow_joint_failure_probability);
        file_ << "," << telemetry.safe_fix_shadow_failure_budget_passed
              << "," << telemetry.inertial_fix_evidence_available
              << "," << telemetry.lambda_shadow_ffrt_passed
              << "," << telemetry.pair_count << ",";
        number(telemetry.full_ratio);
        file_ << ","
              << telemetry.lambda_causal_arc_total_pairs
              << ","
              << telemetry.lambda_causal_arc_ready_pairs
              << ","
              << telemetry.lambda_causal_arc_subset_pair_count
              << ","
              << telemetry.lambda_causal_arc_resets
              << ",";
        number(telemetry.lambda_causal_arc_subset_ratio);
        file_ << ",";
        number(telemetry.lambda_causal_arc_subset_bsr);
        file_ << ",";
        number(telemetry.lambda_causal_arc_subset_variance_min);
        file_ << ",";
        number(telemetry.lambda_causal_arc_subset_variance_max);
        file_ << ",";
        number(
            telemetry
                .lambda_causal_arc_subset_ffrt_min_ratio);
        file_ << ","
              << telemetry.lambda_causal_arc_subset_ffrt_passed
              << ",";
        number(
            causal_arc_candidate.allFinite() &&
                    primary_candidate.allFinite()
                ? (causal_arc_candidate - primary_candidate).norm()
                : std::numeric_limits<double>::quiet_NaN());
        file_ << ",";
        number(
            telemetry
                .lambda_causal_arc_subset_second_position_delta_m);
        file_ << ",";
        number(
            telemetry
                .lambda_causal_arc_subset_partition_a_separation_m);
        file_ << ",";
        number(
            telemetry
                .lambda_causal_arc_subset_partition_b_separation_m);
        file_ << ","
              << telemetry.causal_arc_disjoint_evidence_passed
              << ","
              << telemetry.causal_arc_consensus_declared_fixed
              << ","
              << telemetry.causal_arc_consensus_state
              << ","
              << telemetry
                     .causal_arc_consensus_acquisition_streak
              << ",";
        number(causal_arc_candidate.x());
        file_ << ",";
        number(causal_arc_candidate.y());
        file_ << ",";
        number(causal_arc_candidate.z());
        file_ << ",";
        file_ << telemetry.lambda_satellite_par_shadow_ffrt_passed
              << ","
              << telemetry.lambda_satellite_par_shadow_subset_size
              << ","
              << telemetry
                     .lambda_satellite_par_persistent_subset_attempted
              << ","
              << telemetry
                     .lambda_satellite_par_persistent_subset_used
              << ",";
        number(telemetry.lambda_satellite_par_shadow_ratio);
        file_ << ","
              << telemetry.satellite_par_disjoint_evidence_passed
              << ","
              << telemetry.satellite_par_consensus_declared_fixed
              << ","
              << telemetry.satellite_par_consensus_state
              << ","
              << telemetry
                     .satellite_par_consensus_acquisition_streak
              << ",";
        number(telemetry.lambda_satellite_par_shadow_best_ecef_x);
        file_ << ",";
        number(telemetry.lambda_satellite_par_shadow_best_ecef_y);
        file_ << ",";
        number(telemetry.lambda_satellite_par_shadow_best_ecef_z);
        file_ << ",";
        file_ << telemetry.lambda_src_par_shadow_ffrt_passed
              << ","
              << telemetry.lambda_src_par_shadow_subset_size
              << ",";
        number(telemetry.lambda_src_par_shadow_ratio);
        file_ << ",";
        number(
            telemetry.lambda_src_par_shadow_second_position_delta_m);
        file_ << ","
              << telemetry.src_par_disjoint_evidence_passed
              << ",";
        number(telemetry.src_par_partition_a_separation_m);
        file_ << ",";
        number(telemetry.src_par_partition_b_separation_m);
        file_ << ","
              << telemetry.src_par_consensus_declared_fixed
              << ","
              << telemetry.src_par_consensus_state
              << ","
              << telemetry.src_par_consensus_acquisition_streak
              << ",";
        number(telemetry.lambda_src_par_shadow_best_ecef_x);
        file_ << ",";
        number(telemetry.lambda_src_par_shadow_best_ecef_y);
        file_ << ",";
        number(telemetry.lambda_src_par_shadow_best_ecef_z);
        file_ << ",";
        number(telemetry.float_update_nis_per_observation);
        file_ << ",";
        number(telemetry.float_update_prefit_residual_rms_m);
        file_ << "," << telemetry.safe_fix_shadow_declared_fixed
              << "," << telemetry.inertial_fix_evidence_healthy_anchor
              << "," << telemetry.inertial_fix_evidence_passed << ",";
        number(telemetry.inertial_fix_evidence_time_error_s);
        file_ << ",";
        number(telemetry.inertial_fix_evidence_position_delta_m);
        file_ << ",";
        number(telemetry.inertial_fix_evidence_nis_per_dimension);
        file_ << ","
              << telemetry
                     .inertial_referenced_consensus_declared_fixed
              << ","
              << telemetry.inertial_referenced_consensus_state
              << ","
              << telemetry
                     .inertial_referenced_consensus_acquisition_streak
              << ",";
        number(telemetry.inertial_referenced_correction_x);
        file_ << ",";
        number(telemetry.inertial_referenced_correction_y);
        file_ << ",";
        number(telemetry.inertial_referenced_correction_z);
        file_ << ","
              << telemetry.disjoint_satellite_fix_evidence_available
              << ","
              << telemetry.disjoint_satellite_fix_inputs_verified
              << ","
              << telemetry
                     .disjoint_satellite_fix_partition_a_ffrt_passed
              << ","
              << telemetry
                     .disjoint_satellite_fix_partition_b_ffrt_passed
              << ","
              << telemetry.disjoint_satellite_fix_evidence_passed
              << ",";
        number(
            telemetry
                .disjoint_satellite_fix_partition_separation_m);
        file_ << ",";
        number(
            telemetry
                .disjoint_satellite_fix_partition_a_primary_separation_m);
        file_ << ",";
        number(
            telemetry
                .disjoint_satellite_fix_partition_b_primary_separation_m);
        file_ << ","
              << telemetry
                     .disjoint_satellite_fix_hard_separation_passed
              << ","
              << telemetry
                     .disjoint_satellite_fix_statistical_separation_passed
              << ",";
        number(
            telemetry
                .disjoint_satellite_fix_partition_nis_per_dimension);
        file_ << ",";
        number(
            telemetry
                .disjoint_satellite_fix_partition_a_primary_nis_per_dimension);
        file_ << ",";
        number(
            telemetry
                .disjoint_satellite_fix_partition_b_primary_nis_per_dimension);
        file_ << ","
              << telemetry.disjoint_consensus_declared_fixed
              << "," << telemetry.disjoint_consensus_state
              << ","
              << telemetry.disjoint_consensus_acquisition_streak
              << ","
              << telemetry.disjoint_consensus_selected_pair
              << ",";
        number(
            telemetry
                .disjoint_consensus_selected_pair_min_ratio);
        file_
              << ","
              << telemetry
                     .lambda_l1_l5_wlnl_shadow_wl_ffrt_passed
              << ","
              << telemetry
                     .lambda_l1_l5_wlnl_shadow_nl_ffrt_passed
              << ","
              << telemetry
                     .lambda_l1_l5_wlnl_shadow_candidate_pair_count
              << ",";
        number(
            multifrequency_candidate.allFinite() &&
                    primary_candidate.allFinite()
                ? (multifrequency_candidate -
                   primary_candidate)
                      .norm()
                : std::numeric_limits<double>::quiet_NaN());
        file_ << ","
              << telemetry.multifrequency_disjoint_evidence_passed
              << ","
              << telemetry.multifrequency_consensus_declared_fixed
              << ","
              << telemetry.multifrequency_consensus_state
              << ","
              << telemetry
                     .multifrequency_consensus_acquisition_streak
              << ",";
        number(multifrequency_candidate.x());
        file_ << ",";
        number(multifrequency_candidate.y());
        file_ << ",";
        number(multifrequency_candidate.z());
        file_
              << ","
              << telemetry
                     .lambda_l1_l2_wlnl_shadow_wl_ffrt_passed
              << ","
              << telemetry
                     .lambda_l1_l2_wlnl_shadow_nl_ffrt_passed
              << ","
              << telemetry
                     .lambda_l1_l2_wlnl_shadow_candidate_pair_count
              << ",";
        number(
            l1_l2_multifrequency_candidate.allFinite() &&
                    primary_candidate.allFinite()
                ? (l1_l2_multifrequency_candidate -
                   primary_candidate)
                      .norm()
                : std::numeric_limits<double>::quiet_NaN());
        file_ << ","
              << telemetry
                     .l1_l2_multifrequency_disjoint_evidence_passed
              << ","
              << telemetry
                     .l1_l2_multifrequency_consensus_declared_fixed
              << ","
              << telemetry
                     .l1_l2_multifrequency_consensus_state
              << ","
              << telemetry
                     .l1_l2_multifrequency_consensus_acquisition_streak
              << ",";
        number(l1_l2_multifrequency_candidate.x());
        file_ << ",";
        number(l1_l2_multifrequency_candidate.y());
        file_ << ",";
        number(l1_l2_multifrequency_candidate.z());
        file_
              << ","
              << telemetry
                     .lambda_l2_l5_wlnl_shadow_wl_ffrt_passed
              << ","
              << telemetry
                     .lambda_l2_l5_wlnl_shadow_nl_ffrt_passed
              << ","
              << telemetry.lambda_l2_l5_wlnl_shadow_pair_count
              << ",";
        number(telemetry.lambda_l2_l5_wlnl_shadow_wl_ratio);
        file_ << ",";
        number(
            telemetry.lambda_l2_l5_wlnl_shadow_wl_ffrt_min_ratio);
        file_ << ","
              << telemetry
                     .lambda_l2_l5_wlnl_shadow_mw_disagreements
              << ",";
        number(telemetry.lambda_l2_l5_wlnl_shadow_nl_ratio);
        file_ << ",";
        number(
            telemetry.lambda_l2_l5_wlnl_shadow_nl_ffrt_min_ratio);
        file_ << ","
              << telemetry
                     .lambda_l2_l5_wlnl_shadow_candidate_pair_count
              << ",";
        number(l2_l5_multifrequency_candidate.x());
        file_ << ",";
        number(l2_l5_multifrequency_candidate.y());
        file_ << ",";
        number(l2_l5_multifrequency_candidate.z());
        file_ << "," << telemetry.library_fixed_quality_gate_passed
              << ","
              << telemetry.library_fixed_quality_gate_promoted
              << ","
              << telemetry
                     .library_fixed_quality_gate_disjoint_consensus_promoted
              << ","
              << telemetry
                     .library_fixed_quality_gate_causal_arc_promoted
              << ","
              << telemetry
                     .library_fixed_quality_gate_satellite_par_promoted
              << ","
              << telemetry.library_fixed_quality_gate_src_par_promoted
              << ","
              << telemetry
                     .library_fixed_quality_gate_inertial_referenced_promoted
              << ","
              << telemetry
                     .library_fixed_quality_gate_multifrequency_promoted
              << ","
              << telemetry
                     .library_fixed_quality_gate_raw_partition_conflict
              << "," << telemetry.library_fixed_quality_gate_demoted
              << "," << disjoint_validators_evaluated
              << ",";
        number(processing_runtime_ms);
        file_ << "\n";
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
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            std::exit(0);
        }
        if (arg == "--help-advanced") {
            printAdvancedUsage(argv[0]);
            std::exit(0);
        }
    }

    const libgnss_apps::TomlCliSchema schema{
        "gnss_fuse",
        {
            {"--base-interp", ""},
        },
        {
            {"--arfilter", "--no-arfilter"},
            {"--base-interp", "--no-base-interp"},
            {"--nhc", "--no-nhc"},
            {"--navi776-tc", ""},
            {"--zupt", "--no-zupt"},
        },
        {
            {"--base-ecef", libgnss_apps::TomlArrayStyle::SEPARATE_ARGUMENTS},
            {"--imu-misalignment-rpy-deg", libgnss_apps::TomlArrayStyle::COMMA_JOINED},
            {"--lever-arm", libgnss_apps::TomlArrayStyle::COMMA_JOINED},
        },
    };
    std::vector<std::string> expanded_arguments =
        libgnss_apps::expandTomlConfigArguments(argc, argv, schema);
    std::vector<char*> expanded_argv;
    expanded_argv.reserve(expanded_arguments.size());
    for (auto& argument : expanded_arguments) {
        expanded_argv.push_back(argument.data());
    }
    argc = static_cast<int>(expanded_argv.size());
    argv = expanded_argv.data();

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
        } else if (arg == "--help-advanced") {
            printAdvancedUsage(argv[0]);
            std::exit(0);
        } else if (arg == "--data-dir") {
            options.data_dir = requireValue(arg, i, argc, argv);
        } else if (arg == "--rover") {
            options.rover_path = requireValue(arg, i, argc, argv);
        } else if (arg == "--base") {
            options.base_path = requireValue(arg, i, argc, argv);
        } else if (arg == "--nav") {
            options.nav_path = requireValue(arg, i, argc, argv);
        } else if (arg == "--gnss-pos") {
            options.gnss_pos_path = requireValue(arg, i, argc, argv);
        } else if (arg == "--imu") {
            options.imu_path = requireValue(arg, i, argc, argv);
        } else if (arg == "--imu-format") {
            const std::string format = requireValue(arg, i, argc, argv);
            if (format == "ppc") {
                options.rtklibexplorer_imu_format = false;
            } else if (format == "rtklibexplorer-sf") {
                options.rtklibexplorer_imu_format = true;
            } else {
                argumentError("--imu-format must be ppc or rtklibexplorer-sf",
                              argv[0]);
            }
        } else if (arg == "--imu-misalignment-rpy-deg") {
            options.imu_raw_to_body_flu = rtklibExplorerRawToBodyFlu(
                parseVector3(requireValue(arg, i, argc, argv), arg));
            options.imu_mount_rotation_enabled = true;
        } else if (arg == "--out") {
            options.out_path = requireValue(arg, i, argc, argv);
        } else if (arg == "--kml") {
            options.kml_path = requireValue(arg, i, argc, argv);
            options.write_kml = true;
        } else if (arg == "--attitude-csv") {
            options.attitude_csv_path = requireValue(arg, i, argc, argv);
        } else if (arg == "--tight-dd-sse-csv") {
            options.sse_par_csv_path = requireValue(arg, i, argc, argv);
        } else if (arg == "--library-fix-integrity-gate") {
            options.library_fix_integrity_gate = true;
        } else if (arg == "--library-fix-integrity-csv") {
            options.library_fix_integrity_csv_path =
                requireValue(arg, i, argc, argv);
        } else if (arg == "--integrity-anchor-max-age-s") {
            options.integrity_anchor_max_age_s =
                std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--integrity-disjoint-partition") {
            const auto value = requireValue(arg, i, argc, argv);
            if (value == "grj-ec") {
                options.disjoint_partition_scheme =
                    DisjointPartitionScheme::GRJ_EC;
            } else if (value == "gej-rc") {
                options.disjoint_partition_scheme =
                    DisjointPartitionScheme::GEJ_RC;
            } else if (value == "gcj-re") {
                options.disjoint_partition_scheme =
                    DisjointPartitionScheme::GCJ_RE;
            } else if (value == "gj-erc") {
                options.disjoint_partition_scheme =
                    DisjointPartitionScheme::GJ_ERC;
            } else {
                argumentError(
                    "--integrity-disjoint-partition must be "
                    "grj-ec, gej-rc, gcj-re, or gj-erc",
                    argv[0]);
            }
        } else if (arg == "--integrity-disjoint-ensemble") {
            options.disjoint_partition_ensemble = true;
        } else if (arg == "--integrity-disjoint-evaluate-all") {
            options.integrity_disjoint_evaluate_all = true;
        } else if (arg == "--integrity-student-t-front-end") {
            options.integrity_student_t_front_end = true;
        } else if (arg ==
                   "--integrity-student-t-all-measurements") {
            options.integrity_student_t_front_end = true;
            options.integrity_student_t_all_measurements = true;
        } else if (arg ==
                   "--integrity-student-t-degrees-of-freedom") {
            options.integrity_student_t_degrees_of_freedom =
                std::stod(requireValue(arg, i, argc, argv));
            if (options.integrity_student_t_degrees_of_freedom <= 0.0) {
                argumentError(
                    "--integrity-student-t-degrees-of-freedom must be > 0",
                    argv[0]);
            }
        } else if (arg ==
                   "--integrity-laplacian-all-measurements") {
            options.integrity_student_t_front_end = true;
            options.integrity_student_t_all_measurements = true;
            options.integrity_laplacian_all_measurements = true;
        } else if (arg ==
                   "--integrity-huber-all-measurements") {
            options.integrity_student_t_front_end = true;
            options.integrity_student_t_all_measurements = true;
            options.integrity_huber_all_measurements = true;
        } else if (arg ==
                   "--integrity-heavy-tail-activation-sigma") {
            options.integrity_heavy_tail_activation_sigma =
                std::stod(requireValue(arg, i, argc, argv));
            if (options.integrity_heavy_tail_activation_sigma < 0.0) {
                argumentError(
                    "--integrity-heavy-tail-activation-sigma must be >= 0",
                    argv[0]);
            }
        } else if (arg ==
                   "--integrity-disjoint-heavy-tail-model") {
            options.integrity_disjoint_heavy_tail_model =
                requireValue(arg, i, argc, argv);
            const auto& model =
                options.integrity_disjoint_heavy_tail_model;
            if (model != "inherit" && model != "student-t" &&
                model != "laplacian" && model != "huber") {
                argumentError(
                    "--integrity-disjoint-heavy-tail-model must be "
                    "inherit, student-t, laplacian, or huber",
                    argv[0]);
            }
        } else if (arg ==
                   "--integrity-validator-runtime-budget-ms") {
            options.integrity_validator_runtime_budget_ms =
                std::stod(requireValue(arg, i, argc, argv));
            if (options.integrity_validator_runtime_budget_ms < 0.0) {
                argumentError(
                    "--integrity-validator-runtime-budget-ms must be >= 0",
                    argv[0]);
            }
        } else if (arg ==
                   "--integrity-max-statistical-separation-m") {
            options.integrity_max_statistical_separation_m =
                std::stod(requireValue(arg, i, argc, argv));
            if (options.integrity_max_statistical_separation_m <= 0.0) {
                argumentError(
                    "--integrity-max-statistical-separation-m must be > 0",
                    argv[0]);
            }
        } else if (arg ==
                   "--integrity-disjoint-acquisition-streak") {
            options.integrity_disjoint_acquisition_streak =
                std::stoi(requireValue(arg, i, argc, argv));
            if (options.integrity_disjoint_acquisition_streak < 1) {
                argumentError(
                    "--integrity-disjoint-acquisition-streak must be >= 1",
                    argv[0]);
            }
        } else if (arg ==
                   "--integrity-disjoint-correction-jump-m") {
            options.integrity_disjoint_correction_jump_m =
                std::stod(requireValue(arg, i, argc, argv));
            if (options.integrity_disjoint_correction_jump_m <= 0.0) {
                argumentError(
                    "--integrity-disjoint-correction-jump-m must be > 0",
                    argv[0]);
            }
        } else if (arg ==
                   "--integrity-disjoint-selected-pair-ratio") {
            options.integrity_disjoint_selected_pair_ratio = true;
        } else if (arg ==
                   "--integrity-causal-arc-readiness-shadow") {
            options.integrity_causal_arc_readiness_shadow = true;
        } else if (arg ==
                   "--integrity-causal-arc-smoothed-search-shadow") {
            options.integrity_causal_arc_readiness_shadow = true;
            options.integrity_causal_arc_smoothed_search = true;
        } else if (arg ==
                   "--integrity-causal-arc-smoothed-max-pairs") {
            options.integrity_causal_arc_smoothed_max_pairs =
                std::stoi(requireValue(arg, i, argc, argv));
            if (options.integrity_causal_arc_smoothed_max_pairs != 0 &&
                options.integrity_causal_arc_smoothed_max_pairs < 12) {
                argumentError(
                    "--integrity-causal-arc-smoothed-max-pairs must be 0 or >= 12",
                    argv[0]);
            }
        } else if (arg ==
                   "--integrity-causal-arc-promotion") {
            options.integrity_causal_arc_readiness_shadow = true;
            options.integrity_causal_arc_promotion = true;
        } else if (arg ==
                   "--integrity-satellite-par-consensus-shadow") {
            options.integrity_satellite_par_consensus_shadow = true;
        } else if (arg ==
                   "--integrity-satellite-par-consensus-promotion") {
            options.integrity_satellite_par_consensus_shadow = true;
            options.integrity_satellite_par_consensus_promotion = true;
        } else if (arg ==
                   "--integrity-src-par-consensus-shadow") {
            options.integrity_src_par_consensus_shadow = true;
        } else if (arg ==
                   "--integrity-src-par-consensus-promotion") {
            options.integrity_src_par_consensus_shadow = true;
            options.integrity_src_par_consensus_promotion = true;
        } else if (arg ==
                   "--integrity-inertial-referenced-consensus-shadow") {
            options.integrity_inertial_referenced_consensus_shadow = true;
        } else if (arg ==
                   "--integrity-inertial-referenced-consensus-promotion") {
            options.integrity_inertial_referenced_consensus_shadow = true;
            options.integrity_inertial_referenced_consensus_promotion =
                true;
        } else if (arg ==
                   "--integrity-satellite-par-quality-diverse") {
            options.integrity_satellite_par_quality_diverse = true;
        } else if (arg ==
                   "--integrity-satellite-par-persistent-subset-shadow") {
            options.integrity_satellite_par_persistent_subset = true;
        } else if (arg ==
                   "--integrity-disjoint-satellite-par-shadow") {
            options.integrity_disjoint_satellite_par = true;
        } else if (arg ==
                   "--integrity-satellite-par-max-drop-steps") {
            options.integrity_satellite_par_max_drop_steps =
                std::stoi(requireValue(arg, i, argc, argv));
            if (options.integrity_satellite_par_max_drop_steps < 1 ||
                options.integrity_satellite_par_max_drop_steps > 32) {
                argumentError(
                    "--integrity-satellite-par-max-drop-steps must be in [1, 32]",
                    argv[0]);
            }
        } else if (arg ==
                   "--integrity-wide-lane-front-end") {
            options.integrity_wide_lane_front_end = true;
        } else if (arg ==
                   "--integrity-l1-l2-wlnl-cascade-shadow") {
            options.integrity_l1_l2_wlnl_cascade = true;
        } else if (arg ==
                   "--integrity-l2-l5-wlnl-cascade-shadow") {
            options.integrity_l2_l5_wlnl_cascade = true;
        } else if (arg ==
                   "--integrity-multifrequency-consensus-shadow") {
            options.integrity_multifrequency_consensus_shadow = true;
        } else if (arg ==
                   "--integrity-multifrequency-consensus-promotion") {
            options.integrity_multifrequency_consensus_shadow = true;
            options.integrity_multifrequency_consensus_promotion = true;
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
        } else if (arg == "--ar-policy") {
            const auto value = requireValue(arg, i, argc, argv);
            if (value == "extended") {
                options.ar_policy =
                    libgnss::RTKProcessor::RTKConfig::ARPolicy::
                        EXTENDED;
            } else if (value == "demo5-continuous") {
                options.ar_policy =
                    libgnss::RTKProcessor::RTKConfig::ARPolicy::
                        DEMO5_CONTINUOUS;
            } else {
                argumentError(
                    "unsupported --ar-policy value: " + value,
                    argv[0]);
            }
        } else if (arg == "--glonass-ar") {
            const auto value = requireValue(arg, i, argc, argv);
            if (value == "off") {
                options.glonass_ar_mode =
                    libgnss::RTKProcessor::RTKConfig::
                        GlonassARMode::OFF;
            } else if (value == "on") {
                options.glonass_ar_mode =
                    libgnss::RTKProcessor::RTKConfig::
                        GlonassARMode::ON;
            } else if (value == "autocal") {
                options.glonass_ar_mode =
                    libgnss::RTKProcessor::RTKConfig::
                        GlonassARMode::AUTOCAL;
            } else {
                argumentError(
                    "unsupported --glonass-ar value: " + value,
                    argv[0]);
            }
        } else if (arg == "--max-pos-jump") {
            options.max_position_jump_m =
                std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--rtk-update-outlier-threshold") {
            options.rtk_update_outlier_threshold =
                std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--prefer-trusted-seed") {
            options.prefer_trusted_seed = true;
        } else if (arg == "--rover-seed-pos") {
            options.rover_seed_pos_path =
                requireValue(arg, i, argc, argv);
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
        } else if (arg == "--integrity-fixed-velocity") {
            options.integrity_fixed_velocity = true;
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
        } else if (arg == "--rtk-ins-prior") {
            options.rtk_ins_prior = true;
        } else if (arg == "--rtk-ins-prior-inflation") {
            options.rtk_ins_prior_inflation = std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--rtk-ins-prior-spp-gate") {
            options.rtk_ins_prior_spp_gate = true;
        } else if (arg == "--rtk-ins-prior-min-sats") {
            options.rtk_ins_prior_min_sats = std::stoi(requireValue(arg, i, argc, argv));
        } else if (arg == "--rtk-ins-prior-max-nis") {
            options.rtk_ins_prior_max_nis = std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--rtk-ins-prior-always") {
            options.rtk_ins_prior_spp_gate = false;
            options.rtk_ins_prior_max_nis = 0.0;
        } else if (arg == "--tc-ins-time-update") {
            options.tc_ins_time_update = true;
        } else if (arg == "--navi776-tc") {
            // Validated navi.776 short-baseline combo (docs/navi776_techniques.md,
            // combined-configuration table): M3 closed loop + M4 velocity
            // states + SD Doppler rows (sigma 0.5, gated 1000 m) + gated
            // innovation-adaptive measurement noise. Positional: flags given
            // AFTER this one override its settings.
            options.tc_closed_loop = true;
            options.tc_velocity_states = true;
            options.tc_doppler_rows = true;
            options.tc_reuse_update_factorization = true;
            options.tc_sequential_doppler_update = true;
            options.tc_doppler_sigma_mps = 0.5;
            options.tc_doppler_max_baseline_m = 1000.0;
            options.rtk_adaptive_noise = true;
            options.rtk_adaptive_noise_max_baseline_m = 1000.0;
        } else if (arg == "--tc-closed-loop") {
            options.tc_closed_loop = true;
        } else if (arg == "--tc-trusted-reanchor") {
            options.tc_trusted_reanchor = true;
        } else if (arg == "--tc-trusted-reanchor-max-epochs") {
            options.tc_trusted_reanchor_max_epochs =
                std::stoi(requireValue(arg, i, argc, argv));
        } else if (arg == "--tc-velocity-states") {
            options.tc_velocity_states = true;
        } else if (arg == "--tc-doppler-rows") {
            options.tc_doppler_rows = true;
        } else if (arg == "--tc-reuse-update-factorization") {
            options.tc_reuse_update_factorization = true;
        } else if (arg == "--tc-sequential-doppler-update") {
            options.tc_sequential_doppler_update = true;
        } else if (arg == "--tc-doppler-sigma") {
            options.tc_doppler_sigma_mps = std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--tc-doppler-max-baseline") {
            options.tc_doppler_max_baseline_m = std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--imu-time-offset") {
            options.imu_time_offset_s = std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--imu-time-offset-score-start-epoch") {
            options.imu_time_offset_score_start_epoch =
                std::stoi(requireValue(arg, i, argc, argv));
        } else if (arg == "--tc-tdcp-diagnostics") {
            options.tc_tdcp_diagnostics = true;
        } else if (arg == "--tc-ins-position-q-floor") {
            options.tc_ins_position_q_floor_m2 = std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--tc-ins-max-sample-gap") {
            options.tc_ins_max_sample_gap_s = std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--tc-cp-pr-gate") {
            options.tc_cp_pr_gate = true;
        } else if (arg == "--tc-cp-pr-threshold") {
            options.tc_cp_pr_threshold_m = std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--tc-cp-pr-min-pairs") {
            options.tc_cp_pr_min_pairs = std::stoi(requireValue(arg, i, argc, argv));
        } else if (arg == "--tc-cp-pr-max-bad-pairs") {
            options.tc_cp_pr_max_bad_pairs = std::stoi(requireValue(arg, i, argc, argv));
        } else if (arg == "--tc-cp-pr-escalation") {
            options.tc_cp_pr_escalation_epochs = std::stoi(requireValue(arg, i, argc, argv));
        } else if (arg == "--tc-ddpr-fde-threshold") {
            options.tc_ddpr_fde_threshold_m = std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--tc-ddpr-max-fde-removals") {
            options.tc_ddpr_max_fde_removals = std::stoi(requireValue(arg, i, argc, argv));
        } else if (arg == "--arfilter") {
            options.arfilter_override = true;
            options.arfilter_value = true;
        } else if (arg == "--no-arfilter") {
            options.arfilter_override = true;
            options.arfilter_value = false;
        } else if (arg == "--rtk-snr-weighting") {
            options.rtk_snr_weighting = true;
        } else if (arg == "--rtk-snr-reference-dbhz") {
            options.rtk_snr_reference_dbhz = std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--rtk-snr-max-variance-scale") {
            options.rtk_snr_max_variance_scale = std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--max-subset-ar-drop-steps") {
            options.max_subset_ar_drop_steps = std::stoi(requireValue(arg, i, argc, argv));
        } else if (arg == "--rtk-adaptive-noise") {
            options.rtk_adaptive_noise = true;
        } else if (arg == "--rtk-adaptive-noise-alpha-phase") {
            options.rtk_adaptive_noise_alpha_phase = std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--rtk-adaptive-noise-alpha-code") {
            options.rtk_adaptive_noise_alpha_code = std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--rtk-adaptive-noise-min-scale") {
            options.rtk_adaptive_noise_min_scale = std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--rtk-adaptive-noise-max-scale") {
            options.rtk_adaptive_noise_max_scale = std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--rtk-adaptive-noise-max-baseline") {
            options.rtk_adaptive_noise_max_baseline_m = std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--rtk-adaptive-noise-phase-only") {
            options.rtk_adaptive_noise_phase_only = true;
        } else if (arg == "--rtk-adaptive-noise-per-system-alpha") {
            options.rtk_adaptive_noise_per_system_alpha = true;
        } else if (arg == "--rtk-pos-out") {
            options.rtk_pos_out = requireValue(arg, i, argc, argv);
        } else if (arg == "--cmc-ref") {
            options.cmc_aware_reference_selection = true;
        } else if (arg == "--cmc-ref-level") {
            options.cmc_ref_level_m = std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--cmc-ref-switch-epochs") {
            options.cmc_ref_switch_epochs = std::stoi(requireValue(arg, i, argc, argv));
        } else if (arg == "--cmc-ref-return-min-elev") {
            options.cmc_ref_return_min_elev_deg = std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--cmc-ref-switch-max-elev-drop") {
            options.cmc_ref_switch_max_elev_drop_deg = std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--cmc-ref-switch-min-elev") {
            options.cmc_ref_switch_min_elev_deg = std::stod(requireValue(arg, i, argc, argv));
        } else {
            argumentError("unknown or incomplete argument: " + arg, argv[0]);
        }
    }

    applyImuGradePreset(imu_grade, options.fusion_config.process_noise);

    if (!options.data_dir.empty()) {
        if (options.gnss_pos_path.empty()) {
            if (options.rover_path.empty()) options.rover_path = options.data_dir + "/rover.obs";
            if (options.base_path.empty()) options.base_path = options.data_dir + "/base.obs";
            if (options.nav_path.empty()) options.nav_path = options.data_dir + "/base.nav";
        }
        if (options.imu_path.empty()) options.imu_path = options.data_dir + "/imu.csv";
    }

    if (options.gnss_pos_path.empty() && options.rover_path.empty()) {
        argumentError("--rover is required unless --gnss-pos is supplied", argv[0]);
    }
    if (options.gnss_pos_path.empty() && options.nav_path.empty()) {
        argumentError("--nav is required unless --gnss-pos is supplied", argv[0]);
    }
    if (options.imu_path.empty()) argumentError("--imu is required", argv[0]);
    if (!options.gnss_pos_path.empty() &&
        (!options.rover_path.empty() || !options.base_path.empty() ||
         !options.nav_path.empty())) {
        argumentError("--gnss-pos is mutually exclusive with --rover/--base/--nav",
                      argv[0]);
    }
    if (options.tightly_coupled_dd_imu && options.base_path.empty()) {
        argumentError("--tight-dd-imu requires --base", argv[0]);
    }
    if (options.library_fix_integrity_gate &&
        options.base_path.empty()) {
        argumentError(
            "--library-fix-integrity-gate requires --base",
            argv[0]);
    }
    if (options.tightly_coupled_dd_imu ||
        options.library_fix_integrity_gate) {
        options.fusion_config.position_updates_require_fixed = true;
    }
    if (!std::isfinite(options.integrity_anchor_max_age_s) ||
        options.integrity_anchor_max_age_s <= 0.0) {
        argumentError(
            "--integrity-anchor-max-age-s must be finite and positive",
            argv[0]);
    }
    options.fusion_config.tight_dd_sse_fixed_anchor_max_age_s =
        options.integrity_anchor_max_age_s;
    if (options.max_epochs < 0) argumentError("--max-epochs must be non-negative", argv[0]);
    if (options.imu_time_offset_score_start_epoch < 0) {
        argumentError("--imu-time-offset-score-start-epoch must be non-negative",
                      argv[0]);
    }
    if (options.rtk_ins_prior && options.tc_ins_time_update) {
        argumentError("--rtk-ins-prior and --tc-ins-time-update are mutually exclusive", argv[0]);
    }
    if (options.tc_closed_loop && (options.tc_ins_time_update || options.rtk_ins_prior)) {
        argumentError("--tc-closed-loop is mutually exclusive with legacy INS prior/time-update modes",
                      argv[0]);
    }
    if (options.tc_doppler_rows && !options.tc_velocity_states) {
        argumentError("--tc-doppler-rows requires --tc-velocity-states", argv[0]);
    }
    if (options.tc_velocity_states && !options.tc_closed_loop) {
        argumentError("--tc-velocity-states requires --tc-closed-loop", argv[0]);
    }
    if (options.tc_trusted_reanchor && !options.tc_closed_loop) {
        argumentError("--tc-trusted-reanchor requires --tc-closed-loop", argv[0]);
    }
    if (options.tc_trusted_reanchor_max_epochs < 0) {
        argumentError("--tc-trusted-reanchor-max-epochs must be non-negative", argv[0]);
    }
    if (options.tc_trusted_reanchor_max_epochs > 0 && !options.tc_trusted_reanchor) {
        argumentError("--tc-trusted-reanchor-max-epochs requires --tc-trusted-reanchor",
                      argv[0]);
    }
    if (options.tc_tdcp_diagnostics && !options.tc_velocity_states) {
        argumentError("--tc-tdcp-diagnostics requires --tc-velocity-states", argv[0]);
    }
    if (!std::isfinite(options.tc_ins_position_q_floor_m2) ||
        options.tc_ins_position_q_floor_m2 < 0.0) {
        argumentError("--tc-ins-position-q-floor must be finite and >= 0", argv[0]);
    }
    if (!std::isfinite(options.tc_ins_max_sample_gap_s) ||
        options.tc_ins_max_sample_gap_s <= 0.0) {
        argumentError("--tc-ins-max-sample-gap must be finite and > 0", argv[0]);
    }
    if (!std::isfinite(options.tc_cp_pr_threshold_m) || options.tc_cp_pr_threshold_m <= 0.0) {
        argumentError("--tc-cp-pr-threshold must be finite and > 0", argv[0]);
    }
    if (options.tc_cp_pr_min_pairs < 1 || options.tc_cp_pr_max_bad_pairs < 0 ||
        options.tc_cp_pr_escalation_epochs < 1) {
        argumentError("CP-vs-PR counts must be positive (max bad pairs may be zero)", argv[0]);
    }
    if (!std::isfinite(options.tc_ddpr_fde_threshold_m) ||
        options.tc_ddpr_fde_threshold_m < 0.0 || options.tc_ddpr_max_fde_removals < 0) {
        argumentError("DDPR FDE settings must be finite and non-negative", argv[0]);
    }
    if (options.cmc_ref_switch_epochs < 1) {
        argumentError("--cmc-ref-switch-epochs must be >= 1", argv[0]);
    }
    if (options.cmc_ref_return_min_elev_deg < 0.0) {
        argumentError("--cmc-ref-return-min-elev must be >= 0", argv[0]);
    }
    if (options.cmc_ref_switch_max_elev_drop_deg < 0.0) {
        argumentError("--cmc-ref-switch-max-elev-drop must be >= 0", argv[0]);
    }
    if (options.cmc_ref_switch_min_elev_deg < 0.0) {
        argumentError("--cmc-ref-switch-min-elev must be >= 0", argv[0]);
    }

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
            transformImuSample(sample, options, axis_convention);
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

// Precomputed-solution path (--gnss-pos): drives the same Stage-1 ESKF from
// a LibGNSS++ or RTKLIB .pos stream. This is useful when raw observations or
// base data are unavailable but an external GNSS position+velocity solution
// and raw IMU are available (for example rtklibexplorer/GNSS_IMU).
int runPositionSolutionFusion(
    const FuseOptions& options, libgnss::ImuSeries& imu_series,
    const libgnss::ImuAxisConvention& axis_convention) {
    libgnss::Solution gnss_solution;
    if (!gnss_solution.loadFromFile(options.gnss_pos_path)) {
        std::cerr << "Error: failed to load GNSS solution file: "
                  << options.gnss_pos_path << "\n";
        return 1;
    }
    gnss_solution.sortByTime();

    libgnss::LooseCouplingProcessor fusion_processor(options.fusion_config);
    libgnss::Solution fused_solution;
    AttitudeCsvWriter attitude_writer;
    if (!attitude_writer.open(options.attitude_csv_path)) {
        std::cerr << "Error: failed to open attitude CSV: "
                  << options.attitude_csv_path << "\n";
        return 1;
    }

    size_t imu_cursor = 0;
    int processed_epochs = 0;
    int valid_solutions = 0;
    int correction_epochs = 0;
    double correction_sum_squares = 0.0;

    for (const auto& solution : gnss_solution.solutions) {
        if (options.max_epochs > 0 && processed_epochs >= options.max_epochs) {
            break;
        }
        while (imu_cursor < imu_series.samples.size() &&
               imu_series.samples[imu_cursor].time <= solution.time) {
            libgnss::ImuSample sample = imu_series.samples[imu_cursor];
            transformImuSample(sample, options, axis_convention);
            fusion_processor.processImuSample(sample);
            ++imu_cursor;
        }

        const bool can_measure_correction =
            solution.isValid() && fusion_processor.isInitialized() &&
            fusion_processor.isOriginSet() &&
            processed_epochs >= options.imu_time_offset_score_start_epoch;
        if (solution.isValid()) {
            fusion_processor.processGnssSolution(solution);
            ++valid_solutions;
        }
        if (can_measure_correction &&
            fusion_processor.lastGnssPositionUpdateApplied()) {
            const double correction_sq =
                fusion_processor.lastGnssPositionCorrectionEnu().squaredNorm();
            if (std::isfinite(correction_sq)) {
                correction_sum_squares += correction_sq;
                ++correction_epochs;
            }
        }

        if (fusion_processor.isOriginSet()) {
            fused_solution.addSolution(fusion_processor.toPositionSolution());
            attitude_writer.write(
                fusion_processor.state().nominal.time,
                fusion_processor.state().nominal.attitude_body_to_enu);
        }
        ++processed_epochs;
    }

    if (!fused_solution.writeToFile(options.out_path)) {
        std::cerr << "Error: failed to write fused solution file: "
                  << options.out_path << "\n";
        return 1;
    }
    if (options.write_kml && !fused_solution.writeKML(options.kml_path)) {
        std::cerr << "Error: failed to write KML file: " << options.kml_path
                  << "\n";
        return 1;
    }

    const double score =
        correction_epochs > 0
            ? correction_sum_squares / static_cast<double>(correction_epochs)
            : std::numeric_limits<double>::quiet_NaN();
    std::cout << "imu_time_offset_score:"
              << " offset_s=" << options.imu_time_offset_s
              << " J_mean_sq_correction_m2=" << score
              << " correction_epochs=" << correction_epochs
              << " applied_updates=" << correction_epochs << "\n";

    if (!options.quiet) {
        std::cout << "GNSS half: precomputed position solution\n"
                  << "IMU samples loaded: " << imu_series.samples.size() << "\n"
                  << "GNSS epochs processed: " << processed_epochs << "\n"
                  << "Valid GNSS solutions: " << valid_solutions << "\n"
                  << "Fused epochs written: " << fused_solution.size() << "\n"
                  << "Fusion initialized: "
                  << (fusion_processor.isInitialized() ? "yes" : "no") << "\n"
                  << "Output: " << options.out_path << "\n";
    }
    return 0;
}

// RTK path (--base provided): drives libgnss::RTKProcessor over the
// rover/base RINEX pair using the exact same epoch-alignment helpers
// (libgnss_apps::interpolateBaseEpoch et al.) apps/native/gnss_solve.cpp uses, so
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
    rtk_config.ar_policy = options.ar_policy;
    rtk_config.glonass_ar_mode = options.glonass_ar_mode;
    rtk_config.max_position_jump_m =
        options.max_position_jump_m;
    rtk_config.prefer_trusted_position_seed =
        options.prefer_trusted_seed;
    rtk_config.prefer_rover_position_seed =
        options.prefer_trusted_seed ||
        !options.rover_seed_pos_path.empty();
    if (options.rtk_update_outlier_threshold > 0.0) {
        rtk_config.outlier_threshold =
            options.rtk_update_outlier_threshold;
    }
    if (options.ratio_threshold_set) {
        rtk_config.ratio_threshold = options.ratio_threshold;
        rtk_config.ambiguity_ratio_threshold = options.ratio_threshold;
    }
    rtk_config.elevation_mask = options.elevation_mask_deg * M_PI / 180.0;
    // Phase 1 GNSS/IMU coupling: no-op unless --rtk-ins-prior was passed
    // (RTKConfig::use_external_position_prior defaults false, so this line
    // is a no-op for every existing caller/preset).
    rtk_config.use_external_position_prior = options.rtk_ins_prior;
    rtk_config.use_external_position_time_update =
        options.tc_ins_time_update || options.tc_closed_loop;
    rtk_config.enable_velocity_states = options.tc_velocity_states;
    rtk_config.enable_fixed_anchor_float_stabilization =
        options.tc_doppler_rows &&
        options.rtk_adaptive_noise &&
        options.tc_doppler_max_baseline_m == 1000.0 &&
        options.rtk_adaptive_noise_max_baseline_m == 1000.0;
    rtk_config.enable_doppler_measurement_rows = options.tc_doppler_rows;
    rtk_config.reuse_kalman_factorization_for_nis =
        options.tc_reuse_update_factorization;
    rtk_config.sequential_doppler_update =
        options.tc_sequential_doppler_update;
    rtk_config.doppler_row_sigma_mps = options.tc_doppler_sigma_mps;
    rtk_config.doppler_row_max_baseline_m = options.tc_doppler_max_baseline_m;
    rtk_config.enable_tdcp_diagnostics = options.tc_tdcp_diagnostics;
    rtk_config.ins_time_update_position_q_floor_m2 = options.tc_ins_position_q_floor_m2;
    rtk_config.enable_cp_pr_fixed_gate = options.tc_cp_pr_gate || options.tc_closed_loop;
    rtk_config.cp_pr_fixed_gate_threshold_m = options.tc_cp_pr_threshold_m;
    rtk_config.cp_pr_fixed_gate_min_pairs = options.tc_cp_pr_min_pairs;
    rtk_config.cp_pr_fixed_gate_max_bad_pairs = options.tc_cp_pr_max_bad_pairs;
    rtk_config.cp_pr_fixed_gate_escalation_epochs = options.tc_cp_pr_escalation_epochs;
    rtk_config.ddpr_anchor_fde_threshold_m = options.tc_ddpr_fde_threshold_m;
    rtk_config.ddpr_anchor_max_fde_removals = options.tc_ddpr_max_fde_removals;
    // RTK tuning passthroughs (see FuseOptions doc comment): never touched
    // by any preset, so safe to assign unconditionally before the preset
    // call runs.
    rtk_config.enable_snr_weighting = options.rtk_snr_weighting;
    rtk_config.snr_reference_dbhz = options.rtk_snr_reference_dbhz;
    rtk_config.snr_max_variance_scale = options.rtk_snr_max_variance_scale;
    rtk_config.enable_adaptive_measurement_noise = options.rtk_adaptive_noise;
    rtk_config.adaptive_noise_alpha_phase = options.rtk_adaptive_noise_alpha_phase;
    rtk_config.adaptive_noise_alpha_code = options.rtk_adaptive_noise_alpha_code;
    rtk_config.adaptive_noise_min_variance_scale = options.rtk_adaptive_noise_min_scale;
    rtk_config.adaptive_noise_max_variance_scale = options.rtk_adaptive_noise_max_scale;
    rtk_config.adaptive_noise_max_baseline_m = options.rtk_adaptive_noise_max_baseline_m;
    rtk_config.adaptive_noise_phase_only = options.rtk_adaptive_noise_phase_only;
    rtk_config.adaptive_noise_per_system_alpha = options.rtk_adaptive_noise_per_system_alpha;
    if (options.max_subset_ar_drop_steps >= 0) {
        rtk_config.max_subset_drop_steps_for_ar = options.max_subset_ar_drop_steps;
    }
    rtk_config.cmc_aware_reference_selection = options.cmc_aware_reference_selection;
    rtk_config.cmc_ref_level_m = options.cmc_ref_level_m;
    rtk_config.cmc_ref_switch_epochs = options.cmc_ref_switch_epochs;
    rtk_config.cmc_ref_return_min_elev_deg = options.cmc_ref_return_min_elev_deg;
    rtk_config.cmc_ref_switch_max_elev_drop_deg = options.cmc_ref_switch_max_elev_drop_deg;
    rtk_config.cmc_ref_switch_min_elev_deg = options.cmc_ref_switch_min_elev_deg;
    if (!libgnss_apps::applyRtkConfigPreset(options.rtk_preset, rtk_config)) {
        std::cerr << "Error: unsupported --preset value: " << options.rtk_preset << "\n";
        return 1;
    }
    // --arfilter/--no-arfilter always win over whatever the preset set,
    // since they were requested explicitly after preset resolution would
    // have run (mirrors apps/gnss_solve.cpp's has_ar_filter_override).
    if (options.arfilter_override) {
        rtk_config.enable_ar_filter = options.arfilter_value;
    }
    // A non-default --ratio always wins over whatever the preset set, since
    // it was requested explicitly after preset resolution would have run.
    if (options.ratio_threshold_set) {
        rtk_config.ratio_threshold = options.ratio_threshold;
        rtk_config.ambiguity_ratio_threshold = options.ratio_threshold;
    }
    if (options.library_fix_integrity_gate) {
        rtk_config.lambda_candidate_shadow_count = 2;
        rtk_config.lambda_satellite_par_shadow_max_drop_steps =
            options.integrity_satellite_par_max_drop_steps;
        rtk_config.lambda_l1_l5_wlnl_shadow = true;
        rtk_config.lambda_l1_l5_wlnl_causal_arc_smoothing = true;
        rtk_config.safe_fix_shadow_state_machine.enabled = true;
        rtk_config.safe_fix_shadow_state_machine
            .require_independent_failure_budget = true;
        rtk_config.safe_fix_shadow_state_machine
            .acquisition_streak_epochs = 3;
        rtk_config.safe_fix_shadow_state_machine
            .maximum_acquisition_correction_jump_m = 0.02;
        rtk_config.safe_fix_shadow_state_machine.maximum_hold_epochs = 0;
        rtk_config.safe_fix_shadow_state_machine.minimum_absolute_ratio =
            1.4;
        rtk_config.safe_fix_shadow_state_machine
            .maximum_independent_consensus_delta_m = 0.10;
        rtk_config.safe_fix_shadow_state_machine
            .allow_strong_instant_acquisition = true;
        rtk_config.safe_fix_shadow_state_machine
            .allow_change_point_acquisition = true;
        rtk_config.safe_fix_shadow_minimum_pairs = 12;
        rtk_config.safe_fix_shadow_maximum_second_position_delta_m =
            0.25;
        rtk_config.library_fixed_quality_gate.enabled = true;
        rtk_config.library_fixed_quality_gate
            .require_independent_failure_budget = true;
        rtk_config.library_fixed_quality_gate
            .allow_safe_shadow_promotion = true;
        rtk_config.library_fixed_quality_gate
            .allow_failure_budget_candidate_promotion = false;
        rtk_config.disjoint_consensus_state_machine =
            rtk_config.safe_fix_shadow_state_machine;
        rtk_config.disjoint_consensus_state_machine
            .maximum_independent_consensus_delta_m = 1.0;
        rtk_config.disjoint_consensus_state_machine
            .acquisition_streak_epochs =
            options.integrity_disjoint_acquisition_streak;
        rtk_config.disjoint_consensus_state_machine
            .maximum_acquisition_correction_jump_m =
            options.integrity_disjoint_correction_jump_m;
        rtk_config.disjoint_consensus_state_machine
            .allow_strong_instant_acquisition = false;
        rtk_config.disjoint_consensus_state_machine
            .allow_change_point_acquisition = false;
        rtk_config.disjoint_consensus_use_selected_pair_ratio =
            options.integrity_disjoint_selected_pair_ratio;
        rtk_config.lambda_causal_arc_readiness_shadow =
            options.integrity_causal_arc_readiness_shadow;
        rtk_config.lambda_causal_arc_smoothed_search =
            options.integrity_causal_arc_smoothed_search;
        rtk_config.lambda_causal_arc_smoothed_max_pairs =
            options.integrity_causal_arc_smoothed_max_pairs;
        rtk_config.causal_arc_consensus_state_machine =
            rtk_config.disjoint_consensus_state_machine;
        rtk_config.causal_arc_consensus_state_machine.enabled =
            options.integrity_causal_arc_readiness_shadow;
        rtk_config.causal_arc_consensus_promotion =
            options.integrity_causal_arc_promotion;
        rtk_config.satellite_par_consensus_state_machine =
            rtk_config.disjoint_consensus_state_machine;
        rtk_config.satellite_par_consensus_state_machine.enabled =
            options.integrity_satellite_par_consensus_shadow;
        rtk_config.satellite_par_consensus_promotion =
            options.integrity_satellite_par_consensus_promotion;
        rtk_config.lambda_src_par_shadow_success_rate =
            options.integrity_src_par_consensus_shadow ? 0.999 : 0.0;
        rtk_config.lambda_src_par_shadow_covariance_scale = 16.0;
        rtk_config.src_par_consensus_state_machine =
            rtk_config.disjoint_consensus_state_machine;
        rtk_config.src_par_consensus_state_machine.enabled =
            options.integrity_src_par_consensus_shadow;
        // SRC-PAR already applies a dimension/BSR-specific FFRT at a
        // 1e-3 fixed-failure-rate target. Do not stack the unrelated
        // legacy absolute-ratio 1.4 threshold on top of that calibrated
        // test; all temporal and solution-separation gates remain active.
        rtk_config.src_par_consensus_state_machine.minimum_absolute_ratio =
            0.0;
        rtk_config.src_par_consensus_promotion =
            options.integrity_src_par_consensus_promotion;
        rtk_config.inertial_referenced_consensus_state_machine =
            rtk_config.disjoint_consensus_state_machine;
        rtk_config.inertial_referenced_consensus_state_machine.enabled =
            options.integrity_inertial_referenced_consensus_shadow;
        rtk_config.inertial_referenced_consensus_promotion =
            options.integrity_inertial_referenced_consensus_promotion;
        rtk_config.lambda_satellite_par_shadow_quality_diverse =
            options.integrity_satellite_par_quality_diverse;
        rtk_config.lambda_satellite_par_persistent_subset =
            options.integrity_satellite_par_persistent_subset;
        rtk_config.enable_wide_lane_ar =
            options.integrity_wide_lane_front_end;
        rtk_config.lambda_l1_l2_wlnl_shadow =
            options.integrity_l1_l2_wlnl_cascade;
        rtk_config.lambda_l2_l5_wlnl_shadow =
            options.integrity_l2_l5_wlnl_cascade;
        rtk_config.multifrequency_consensus_state_machine =
            rtk_config.disjoint_consensus_state_machine;
        rtk_config.multifrequency_consensus_state_machine.enabled =
            options.integrity_multifrequency_consensus_shadow;
        rtk_config.multifrequency_consensus_promotion =
            options.integrity_multifrequency_consensus_promotion;
        rtk_config
            .disjoint_satellite_fix_max_statistical_separation_m =
            options.integrity_max_statistical_separation_m;
        rtk_config.student_t_front_end.enabled =
            options.integrity_student_t_front_end;
        rtk_config.student_t_front_end.code_only =
            !options.integrity_student_t_all_measurements;
        rtk_config.student_t_front_end.degrees_of_freedom =
            options.integrity_student_t_degrees_of_freedom;
        rtk_config.student_t_front_end.activation_threshold_sigma =
            options.integrity_heavy_tail_activation_sigma;
        if (options.integrity_laplacian_all_measurements) {
            rtk_config.student_t_front_end.weight_model =
                libgnss::rtk_update::HeavyTailWeightModel::
                    LAPLACIAN;
        } else if (options.integrity_huber_all_measurements) {
            rtk_config.student_t_front_end.weight_model =
                libgnss::rtk_update::HeavyTailWeightModel::
                    HUBER;
        }
    }
    rtk_processor.setRTKConfig(rtk_config);
    rtk_processor.setBasePosition(base_position);
    struct DisjointValidator {
        DisjointPartitionScheme scheme;
        std::unique_ptr<libgnss::RTKProcessor> partition_a;
        std::unique_ptr<libgnss::RTKProcessor> partition_b;
    };
    std::vector<DisjointValidator> disjoint_validators;
    if (options.library_fix_integrity_gate) {
        std::vector<DisjointPartitionScheme> schemes{
            options.disjoint_partition_scheme};
        if (options.disjoint_partition_ensemble) {
            const std::vector<DisjointPartitionScheme> remaining{
                DisjointPartitionScheme::GRJ_EC,
                DisjointPartitionScheme::GEJ_RC,
                DisjointPartitionScheme::GCJ_RE,
                DisjointPartitionScheme::GJ_ERC,
            };
            for (const auto scheme : remaining) {
                if (scheme != options.disjoint_partition_scheme) {
                    schemes.push_back(scheme);
                }
            }
        }
        for (const auto scheme : schemes) {
            auto disjoint_config = rtk_config;
            if (options.integrity_disjoint_heavy_tail_model !=
                "inherit") {
                disjoint_config.student_t_front_end.enabled = true;
                disjoint_config.student_t_front_end.code_only = false;
                if (options.integrity_disjoint_heavy_tail_model ==
                    "laplacian") {
                    disjoint_config.student_t_front_end.weight_model =
                        libgnss::rtk_update::HeavyTailWeightModel::
                            LAPLACIAN;
                } else if (
                    options.integrity_disjoint_heavy_tail_model ==
                    "huber") {
                    disjoint_config.student_t_front_end.weight_model =
                        libgnss::rtk_update::HeavyTailWeightModel::
                            HUBER;
                } else {
                    disjoint_config.student_t_front_end.weight_model =
                        libgnss::rtk_update::HeavyTailWeightModel::
                            STUDENT_T;
                }
            }
            disjoint_config.library_fixed_quality_gate.enabled = false;
            disjoint_config.library_fixed_quality_gate
                .require_independent_failure_budget = false;
            disjoint_config.safe_fix_shadow_state_machine.enabled = false;
            disjoint_config.lambda_satellite_par_shadow_max_drop_steps =
                options.integrity_disjoint_satellite_par
                    ? options.integrity_satellite_par_max_drop_steps
                    : 0;
            disjoint_config
                .lambda_satellite_par_only_after_full_ffrt_failure =
                options.integrity_disjoint_satellite_par;
            disjoint_config.enable_wide_lane_ar = false;
            disjoint_config.lambda_l1_l5_wlnl_shadow = false;
            disjoint_config.lambda_l1_l2_wlnl_shadow = false;
            disjoint_config.lambda_l1_l5_wlnl_causal_arc_smoothing =
                false;
            disjoint_config.lambda_causal_arc_readiness_shadow = false;
            disjoint_config.lambda_candidate_shadow_count = 2;
            disjoint_config.prefer_trusted_position_seed = false;
            disjoint_config.prefer_rover_position_seed = false;
            // Every partition must produce a fresh FFRT decision; it must
            // never gain authority from held integers.
            disjoint_config.min_hold_count =
                std::numeric_limits<int>::max();
            if (!belongsToDisjointPartitionA(
                    libgnss::GNSSSystem::GLONASS, scheme)) {
                disjoint_config.enable_glonass = false;
                disjoint_config.glonass_ar_mode =
                    libgnss::RTKProcessor::RTKConfig::
                        GlonassARMode::OFF;
            }
            auto partition_b_config = disjoint_config;
            if (belongsToDisjointPartitionB(
                    libgnss::GNSSSystem::GLONASS, scheme)) {
                partition_b_config.enable_glonass =
                    rtk_config.enable_glonass;
                partition_b_config.glonass_ar_mode =
                    rtk_config.glonass_ar_mode;
            } else {
                partition_b_config.enable_glonass = false;
                partition_b_config.glonass_ar_mode =
                    libgnss::RTKProcessor::RTKConfig::
                        GlonassARMode::OFF;
            }
            DisjointValidator validator{
                scheme,
                std::make_unique<libgnss::RTKProcessor>(),
                std::make_unique<libgnss::RTKProcessor>()};
            validator.partition_a->setRTKConfig(disjoint_config);
            validator.partition_a->setBasePosition(base_position);
            validator.partition_b->setRTKConfig(partition_b_config);
            validator.partition_b->setBasePosition(base_position);
            disjoint_validators.push_back(std::move(validator));
        }
    }
    const auto rover_seed_positions =
        options.rover_seed_pos_path.empty()
            ? std::map<double, Eigen::Vector3d>{}
            : loadSeedPositions(options.rover_seed_pos_path);

    libgnss::LooseCouplingProcessor fusion_processor(options.fusion_config);
    libgnss::ImuPreintegrationConfig tc_preintegration_config;
    tc_preintegration_config.process_noise = options.fusion_config.process_noise;
    tc_preintegration_config.max_sample_gap_s = options.tc_ins_max_sample_gap_s;
    libgnss::ImuPreintegrator tc_preintegrator(tc_preintegration_config);
    libgnss::TightCouplingProcessor::Config tc_closed_loop_config;
    tc_closed_loop_config.process_noise = options.fusion_config.process_noise;
    tc_closed_loop_config.lever_arm_body = options.fusion_config.lever_arm_body;
    tc_closed_loop_config.max_sample_gap_s = options.tc_ins_max_sample_gap_s;
    tc_closed_loop_config.zupt_enable = options.fusion_config.zupt_enable;
    tc_closed_loop_config.zupt_sigma_mps = options.fusion_config.zupt_sigma_mps;
    tc_closed_loop_config.zupt_max_accel_std = options.fusion_config.zupt_max_accel_std;
    tc_closed_loop_config.zupt_max_gyro_std = options.fusion_config.zupt_max_gyro_std;
    tc_closed_loop_config.zupt_max_gyro_median = options.fusion_config.zupt_max_gyro_median;
    tc_closed_loop_config.nhc_enable = options.fusion_config.nhc_enable;
    tc_closed_loop_config.nhc_sigma_lateral_mps = options.fusion_config.nhc_sigma_lateral_mps;
    tc_closed_loop_config.nhc_sigma_vertical_mps = options.fusion_config.nhc_sigma_vertical_mps;
    tc_closed_loop_config.velocity_state_output_enable = options.tc_velocity_states;
    libgnss::TightCouplingProcessor tc_closed_loop_processor(tc_closed_loop_config);
    double tc_anchor_lat_rad = 0.0;
    double tc_anchor_lon_rad = 0.0;
    int tc_update_supplied_count = 0;
    int tc_anchor_count = 0;
    int tc_invalid_interval_count = 0;
    int tc_trusted_reanchor_fallback_count = 0;
    int tc_consecutive_prediction_continuations = 0;
    int tc_cp_pr_evaluated_count = 0;
    int tc_cp_pr_rejected_count = 0;
    int tc_cp_pr_escalated_count = 0;
    int tc_ddpr_anchor_count = 0;
    std::size_t tc_tdcp_candidate_count = 0;
    std::size_t tc_tdcp_residual_count = 0;
    std::size_t tc_tdcp_missing_previous_count = 0;
    std::size_t tc_tdcp_gap_count = 0;
    std::size_t tc_tdcp_loss_of_lock_count = 0;
    std::size_t tc_tdcp_invalid_count = 0;
    double tc_tdcp_residual_sum_squares = 0.0;
    double tc_tdcp_residual_max_abs_m = 0.0;
    libgnss::Solution fused_solution;
    // Opt-in (--rtk-pos-out): the raw per-epoch RTK PositionSolution stream,
    // before it ever reaches the fusion filter -- see FuseOptions doc
    // comment. Left empty/unused unless options.rtk_pos_out is set.
    libgnss::Solution rtk_solution_log;
    AttitudeCsvWriter attitude_writer;
    if (!attitude_writer.open(options.attitude_csv_path)) {
        std::cerr << "Error: failed to open attitude CSV: " << options.attitude_csv_path << "\n";
        return 1;
    }
    SSEPartialARCsvWriter sse_par_writer;
    if (!sse_par_writer.open(options.sse_par_csv_path)) {
        std::cerr << "Error: failed to open SSE-PAR CSV: "
                  << options.sse_par_csv_path << "\n";
        return 1;
    }
    LibraryFixIntegrityCsvWriter integrity_writer;
    if (!integrity_writer.open(
            options.library_fix_integrity_csv_path)) {
        std::cerr << "Error: failed to open library FIX integrity CSV: "
                  << options.library_fix_integrity_csv_path << "\n";
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
    int tight_dd_sse_par_available_epochs = 0;
    int tight_dd_sse_par_eligible_epochs = 0;
    int tight_dd_sse_par_passed_epochs = 0;
    int tight_dd_sse_par_fixed_ambiguities = 0;
    int tight_dd_sse_par_subsets_evaluated = 0;
    int tight_dd_sse_par_ratio_passed_subsets = 0;
    int tight_dd_sse_par_separation_rejected_subsets = 0;
    int tight_dd_soft_resets = 0;
    int tight_dd_nis_samples = 0;
    double tight_dd_nis_sum = 0.0;
    double tight_dd_nis_max = 0.0;
    // Phase 1b (docs/imu_fusion.md): how often the gated INS position prior
    // actually fired vs. how many epochs were even eligible (fusion filter
    // initialized/anchored/heading-converged) to fire on, absent the new
    // gates. With the gates doing their job, ins_prior_fire_count should be
    // a small fraction of ins_prior_eligible_epochs -- most eligible epochs
    // have a healthy SPP fix and should fall back to the legacy reseed.
    int ins_prior_eligible_epochs = 0;
    int ins_prior_fire_count = 0;
    // Gate-tuning diagnostics only (not printed unless --verbose): per
    // eligible epoch, the two raw signals the gates threshold on, so a
    // --verbose run's tail summary can show their distribution and calibrate
    // rtk_ins_prior_min_sats / rtk_ins_prior_max_nis against a real dataset
    // without needing custom instrumentation each time.
    std::vector<int> ins_prior_eligible_nsats;
    std::vector<double> ins_prior_eligible_nis;

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
    // apps/native/gnss_solve.cpp's own nonfix-drift/float-bridge/jump guards are
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
        if (!rover_seed_positions.empty()) {
            const auto seed = rover_seed_positions.find(
                roundedTowKey(rover_obs.time.tow));
            if (seed != rover_seed_positions.end()) {
                rover_obs.receiver_position = seed->second;
            }
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

        const auto epoch_processing_started =
            std::chrono::steady_clock::now();
        while (imu_cursor < imu_series.samples.size() &&
               imu_series.samples[imu_cursor].time <= rover_obs.time) {
            libgnss::ImuSample sample = imu_series.samples[imu_cursor];
            transformImuSample(sample, options, axis_convention);
            fusion_processor.processImuSample(sample);
            if (options.tc_ins_time_update && tc_preintegrator.initialized()) {
                const auto status = tc_preintegrator.integrate(sample);
                if (status != libgnss::PreintegrationStatus::ACCEPTED &&
                    status != libgnss::PreintegrationStatus::INTERVAL_INVALID) {
                    // Count the root failure once. Later samples in the same
                    // latched-invalid interval report INTERVAL_INVALID.
                    ++tc_invalid_interval_count;
                }
            }
            if (options.tc_closed_loop) {
                tc_closed_loop_processor.processImuSample(sample);
            }
            ++imu_cursor;
        }

        // M1: convert this RTK-to-RTK preintegrated antenna displacement and
        // its process noise from the anchor ENU frame to ECEF. The anchor's
        // nominal body position is -R*lever, so predicted p+R*lever is the
        // antenna displacement itself, with no loose-ESKF absolute position
        // entering the update.
        if (options.tc_ins_time_update && tc_preintegrator.valid()) {
            const auto tc_result = tc_preintegrator.result();
            const Eigen::Matrix3d body_to_enu =
                tc_result.predicted_state.attitude_body_to_enu.toRotationMatrix();
            const Eigen::Vector3d antenna_delta_enu =
                tc_result.predicted_state.position_enu +
                body_to_enu * options.fusion_config.lever_arm_body;
            Eigen::Matrix<double, 3, libgnss::fusion_index::SIZE> antenna_jacobian =
                Eigen::Matrix<double, 3, libgnss::fusion_index::SIZE>::Zero();
            antenna_jacobian.block<3, 3>(0, libgnss::fusion_index::POSITION) =
                Eigen::Matrix3d::Identity();
            antenna_jacobian.block<3, 3>(0, libgnss::fusion_index::ATTITUDE) =
                -body_to_enu * libgnss::attitude::skew(options.fusion_config.lever_arm_body);
            const Eigen::Matrix3d antenna_process_noise_enu =
                antenna_jacobian * tc_result.process_noise * antenna_jacobian.transpose();
            const Eigen::Matrix3d ecef_to_enu =
                ecefToEnuRotation(tc_anchor_lat_rad, tc_anchor_lon_rad);
            const Eigen::Matrix3d enu_to_ecef = ecef_to_enu.transpose();
            const Eigen::Vector3d antenna_delta_ecef = enu_to_ecef * antenna_delta_enu;
            const Eigen::Matrix3d antenna_process_noise_ecef =
                enu_to_ecef * antenna_process_noise_enu * ecef_to_enu;
            if (antenna_delta_ecef.allFinite() && antenna_process_noise_ecef.allFinite()) {
                rtk_processor.setExternalPositionTimeUpdate(
                    antenna_delta_ecef, antenna_process_noise_ecef);
                ++tc_update_supplied_count;
            } else {
                ++tc_invalid_interval_count;
                tc_preintegrator.clear();
            }
        }
        if (options.tc_closed_loop) {
            const auto update = tc_closed_loop_processor.prepareTimeUpdate();
            if (update.valid) {
                if (options.tc_velocity_states) {
                    rtk_processor.setExternalPositionVelocityTimeUpdate(
                        update.antenna_delta_ecef, update.antenna_velocity_ecef,
                        update.position_velocity_process_noise_ecef,
                        update.velocity_covariance_ecef);
                } else {
                    rtk_processor.setExternalPositionTimeUpdate(
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
        // have solved fine on its own. Two additional gates (see
        // FuseOptions's rtk_ins_prior_spp_gate/rtk_ins_prior_max_nis doc
        // comment for the full rationale) now restrict injection to epochs
        // that are BOTH plausibly SPP-degraded (the bridges/tunnels/urban-
        // canyon case the prior exists for) AND backed by a currently
        // healthy ESKF (recent velocity innovations still corroborate the
        // predicted state) -- --rtk-ins-prior-always disables both to
        // reproduce v1 exactly for comparison.
        if (options.rtk_ins_prior && fusion_processor.isInitialized() &&
            fusion_processor.isOriginSet() && fusion_processor.isHeadingConverged()) {
            ++ins_prior_eligible_epochs;
            const int current_nsat = static_cast<int>(rover_obs.getNumSatellites());
            const double current_nis = fusion_processor.getVelocityNisEma();
            if (options.verbose) {
                ins_prior_eligible_nsats.push_back(current_nsat);
                ins_prior_eligible_nis.push_back(current_nis);
            }
            const bool spp_degraded = !options.rtk_ins_prior_spp_gate ||
                current_nsat < options.rtk_ins_prior_min_sats;
            const bool eskf_healthy = options.rtk_ins_prior_max_nis <= 0.0 ||
                current_nis <= options.rtk_ins_prior_max_nis;
            if (spp_degraded && eskf_healthy) {
                Eigen::Vector3d predicted_pos;
                Eigen::Matrix3d predicted_cov;
                if (fusion_processor.predictedAntennaPositionEcef(predicted_pos, predicted_cov) &&
                    predicted_pos.allFinite() && predicted_cov.allFinite()) {
                    rtk_processor.setExternalPositionPrior(
                        predicted_pos, predicted_cov * options.rtk_ins_prior_inflation);
                    ++ins_prior_fire_count;
                }
            }
        }

        if (options.library_fix_integrity_gate &&
            fusion_processor.isInitialized() &&
            fusion_processor.isOriginSet()) {
            const auto prediction =
                fusion_processor.toPositionSolution();
            libgnss::RTKProcessor::ExternalInertialFixEvidence
                inertial_evidence;
            inertial_evidence.available =
                prediction.position_ecef.allFinite() &&
                prediction.position_covariance.allFinite();
            inertial_evidence.healthy_independent_anchor =
                fusion_processor.hasHealthyFixedAnchor();
            inertial_evidence.time = prediction.time;
            inertial_evidence.position_ecef =
                prediction.position_ecef;
            inertial_evidence.position_covariance_ecef =
                prediction.position_covariance;
            rtk_processor.setExternalInertialFixEvidence(
                inertial_evidence);
        }

        int disjoint_validators_evaluated = 0;
        if (options.library_fix_integrity_gate) {
            libgnss::RTKProcessor::
                ExternalDisjointSatelliteFixEvidence
                    selected_evidence;
            double selected_partition_separation =
                std::numeric_limits<double>::infinity();
            bool selected_evidence_eligible = false;
            bool selected_ratio_qualified = false;
            for (auto& validator : disjoint_validators) {
                if (disjoint_validators_evaluated > 0 &&
                    options.integrity_validator_runtime_budget_ms > 0.0) {
                    const double elapsed_ms =
                        std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() -
                            epoch_processing_started)
                            .count();
                    if (elapsed_ms >=
                        options.integrity_validator_runtime_budget_ms) {
                        break;
                    }
                }
                const auto rover_partition_a =
                    filterDisjointPartition(
                        rover_obs, true, validator.scheme);
                const auto base_partition_a =
                    filterDisjointPartition(
                        aligned_base_obs, true, validator.scheme);
                const auto rover_partition_b =
                    filterDisjointPartition(
                        rover_obs, false, validator.scheme);
                const auto base_partition_b =
                    filterDisjointPartition(
                        aligned_base_obs, false, validator.scheme);
                const auto partition_a_solution =
                    validator.partition_a->processRTKEpoch(
                        rover_partition_a, base_partition_a, nav_data);
                const auto partition_b_solution =
                    validator.partition_b->processRTKEpoch(
                        rover_partition_b, base_partition_b, nav_data);
                ++disjoint_validators_evaluated;
                const auto& partition_a_telemetry =
                    validator.partition_a->getLastDebugTelemetry();
                const auto& partition_b_telemetry =
                    validator.partition_b->getLastDebugTelemetry();
                libgnss::RTKProcessor::
                    ExternalDisjointSatelliteFixEvidence evidence;
                evidence.available = true;
                evidence.inputs_verified_disjoint =
                    observationInputsAreDisjoint(
                        rover_partition_a, rover_partition_b) &&
                    observationInputsAreDisjoint(
                        base_partition_a, base_partition_b);
                const Eigen::Vector3d partition_a_full_candidate(
                        partition_a_telemetry
                            .lambda_shadow_best_ecef_x,
                        partition_a_telemetry
                            .lambda_shadow_best_ecef_y,
                        partition_a_telemetry
                            .lambda_shadow_best_ecef_z);
                const Eigen::Vector3d partition_b_full_candidate(
                        partition_b_telemetry
                            .lambda_shadow_best_ecef_x,
                        partition_b_telemetry
                            .lambda_shadow_best_ecef_y,
                        partition_b_telemetry
                            .lambda_shadow_best_ecef_z);
                const Eigen::Vector3d partition_a_par_candidate(
                        partition_a_telemetry
                            .lambda_satellite_par_shadow_best_ecef_x,
                        partition_a_telemetry
                            .lambda_satellite_par_shadow_best_ecef_y,
                        partition_a_telemetry
                            .lambda_satellite_par_shadow_best_ecef_z);
                const Eigen::Vector3d partition_b_par_candidate(
                        partition_b_telemetry
                            .lambda_satellite_par_shadow_best_ecef_x,
                        partition_b_telemetry
                            .lambda_satellite_par_shadow_best_ecef_y,
                        partition_b_telemetry
                            .lambda_satellite_par_shadow_best_ecef_z);
                const bool partition_a_full_passed =
                    partition_a_telemetry.lambda_shadow_ffrt_passed &&
                    partition_a_full_candidate.allFinite();
                const bool partition_b_full_passed =
                    partition_b_telemetry.lambda_shadow_ffrt_passed &&
                    partition_b_full_candidate.allFinite();
                const bool partition_a_par_passed =
                    options.integrity_disjoint_satellite_par &&
                    partition_a_telemetry
                        .lambda_satellite_par_shadow_ffrt_passed &&
                    partition_a_par_candidate.allFinite();
                const bool partition_b_par_passed =
                    options.integrity_disjoint_satellite_par &&
                    partition_b_telemetry
                        .lambda_satellite_par_shadow_ffrt_passed &&
                    partition_b_par_candidate.allFinite();
                evidence.partition_a_ffrt_passed =
                    partition_a_full_passed ||
                    partition_a_par_passed;
                evidence.partition_b_ffrt_passed =
                    partition_b_full_passed ||
                    partition_b_par_passed;
                evidence.partition_a_ratio =
                    partition_a_full_passed
                        ? partition_a_telemetry.full_ratio
                        : partition_a_telemetry
                              .lambda_satellite_par_shadow_ratio;
                evidence.partition_b_ratio =
                    partition_b_full_passed
                        ? partition_b_telemetry.full_ratio
                        : partition_b_telemetry
                              .lambda_satellite_par_shadow_ratio;
                evidence.partition_a_candidate_ecef =
                    partition_a_full_passed
                        ? partition_a_full_candidate
                        : partition_a_par_candidate;
                evidence.partition_b_candidate_ecef =
                    partition_b_full_passed
                        ? partition_b_full_candidate
                        : partition_b_par_candidate;
                evidence.partition_a_covariance_ecef =
                    partition_a_solution.position_covariance;
                evidence.partition_b_covariance_ecef =
                    partition_b_solution.position_covariance;
                const double separation =
                    (evidence.partition_a_candidate_ecef -
                     evidence.partition_b_candidate_ecef)
                        .norm();
                const bool eligible =
                    evidence.inputs_verified_disjoint &&
                    evidence.partition_a_ffrt_passed &&
                    evidence.partition_b_ffrt_passed &&
                    std::isfinite(separation);
                const bool ratio_qualified =
                    std::isfinite(evidence.partition_a_ratio) &&
                    evidence.partition_a_ratio >=
                        rtk_config.disjoint_consensus_state_machine
                            .minimum_absolute_ratio &&
                    std::isfinite(evidence.partition_b_ratio) &&
                    evidence.partition_b_ratio >=
                        rtk_config.disjoint_consensus_state_machine
                            .minimum_absolute_ratio;
                const bool prefer_ratio_qualified =
                    options.integrity_disjoint_selected_pair_ratio &&
                    ratio_qualified &&
                    !selected_ratio_qualified;
                const bool same_ratio_class =
                    !options.integrity_disjoint_selected_pair_ratio ||
                    ratio_qualified == selected_ratio_qualified;
                if ((!selected_evidence.available) ||
                    (eligible && !selected_evidence_eligible) ||
                    (eligible && selected_evidence_eligible &&
                     prefer_ratio_qualified) ||
                    (eligible && selected_evidence_eligible &&
                     same_ratio_class &&
                     separation < selected_partition_separation)) {
                    selected_evidence = evidence;
                    selected_evidence_eligible = eligible;
                    selected_ratio_qualified =
                        eligible && ratio_qualified;
                    selected_partition_separation =
                        eligible
                            ? separation
                            : std::numeric_limits<double>::
                                  infinity();
                }
                // A physically tight A/B agreement already satisfies the
                // strongest partition-only separation condition. Preserve
                // the configured split priority and avoid running further
                // validators unless this split is unavailable or only
                // covariance-normalized agreement is possible.
                if (!options.integrity_disjoint_evaluate_all &&
                    eligible &&
                    (!options.integrity_disjoint_selected_pair_ratio ||
                     ratio_qualified) &&
                    separation <=
                        rtk_config
                            .disjoint_satellite_fix_max_partition_separation_m) {
                    break;
                }
            }
            rtk_processor.setExternalDisjointSatelliteFixEvidence(
                selected_evidence);
        }

        auto pos_solution = rtk_processor.processRTKEpoch(
            rover_obs, aligned_base_obs, nav_data);
        if (options.library_fix_integrity_gate) {
            const double processing_runtime_ms =
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() -
                    epoch_processing_started)
                    .count();
            integrity_writer.write(
                pos_solution,
                rtk_processor.getLastDebugTelemetry(),
                disjoint_validators_evaluated,
                processing_runtime_ms);
        }
        if (options.tc_cp_pr_gate || options.tc_closed_loop) {
            const auto& telemetry = rtk_processor.getLastDebugTelemetry();
            tc_cp_pr_evaluated_count += telemetry.cp_pr_gate_evaluated ? 1 : 0;
            tc_cp_pr_rejected_count += telemetry.cp_pr_gate_rejected ? 1 : 0;
            tc_cp_pr_escalated_count += telemetry.cp_pr_gate_escalated ? 1 : 0;
            tc_ddpr_anchor_count += telemetry.ddpr_anchor_valid ? 1 : 0;
        }
        if (options.tc_tdcp_diagnostics) {
            const auto& telemetry = rtk_processor.getLastDebugTelemetry();
            tc_tdcp_candidate_count += telemetry.tdcp_candidate_count;
            tc_tdcp_residual_count += telemetry.tdcp_residual_count;
            tc_tdcp_missing_previous_count += telemetry.tdcp_rejected_missing_previous;
            tc_tdcp_gap_count += telemetry.tdcp_rejected_gap;
            tc_tdcp_loss_of_lock_count += telemetry.tdcp_rejected_loss_of_lock;
            tc_tdcp_invalid_count += telemetry.tdcp_rejected_invalid;
            if (telemetry.tdcp_residual_count > 0 &&
                std::isfinite(telemetry.tdcp_residual_rms_m)) {
                tc_tdcp_residual_sum_squares += telemetry.tdcp_residual_count *
                    telemetry.tdcp_residual_rms_m * telemetry.tdcp_residual_rms_m;
                tc_tdcp_residual_max_abs_m = std::max(
                    tc_tdcp_residual_max_abs_m, telemetry.tdcp_residual_max_abs_m);
            }
        }
        if (pos_solution.isValid()) {
            auto fusion_solution = pos_solution;
            const auto& epoch_telemetry =
                rtk_processor.getLastDebugTelemetry();
            const bool causal_candidate_promoted =
                epoch_telemetry
                    .library_fixed_quality_gate_causal_arc_promoted ||
                epoch_telemetry
                    .library_fixed_quality_gate_satellite_par_promoted ||
                epoch_telemetry
                    .library_fixed_quality_gate_src_par_promoted ||
                epoch_telemetry
                    .library_fixed_quality_gate_inertial_referenced_promoted ||
                epoch_telemetry
                    .library_fixed_quality_gate_multifrequency_promoted;
            if (causal_candidate_promoted) {
                const Eigen::Vector3d original_ecef(
                    epoch_telemetry
                        .library_fixed_quality_gate_original_ecef_x,
                    epoch_telemetry
                        .library_fixed_quality_gate_original_ecef_y,
                    epoch_telemetry
                        .library_fixed_quality_gate_original_ecef_z);
                if (original_ecef.allFinite()) {
                    fusion_solution.position_ecef = original_ecef;
                    fusion_solution.position_geodetic =
                        libgnss::spp_utils::ecefToGeodetic(
                            original_ecef);
                }
                fusion_solution.status =
                    static_cast<libgnss::SolutionStatus>(
                        epoch_telemetry
                            .library_fixed_quality_gate_original_status);
                fusion_solution.ratio =
                    epoch_telemetry
                        .library_fixed_quality_gate_original_ratio;
            }
            const bool use_fixed_velocity =
                (options.derive_velocity_from_fixed &&
                 !fusion_solution.has_velocity) ||
                (options.library_fix_integrity_gate &&
                 options.integrity_fixed_velocity);
            if (use_fixed_velocity &&
                fusion_solution.isFixed() && have_previous_fixed_solution &&
                fusion_solution.ratio >= kMinDerivedVelocityRatio &&
                previous_fixed_solution.ratio >= kMinDerivedVelocityRatio) {
                const double dt =
                    fusion_solution.time -
                    previous_fixed_solution.time;
                if (dt > 0.0 && dt <= kMaxVelocityDerivationGapSeconds) {
                    const Eigen::Vector3d candidate_velocity =
                        (fusion_solution.position_ecef -
                         previous_fixed_solution.position_ecef) /
                        dt;
                    if (candidate_velocity.norm() <= kMaxPlausibleSpeedMps) {
                        fusion_solution.velocity_ecef =
                            candidate_velocity;
                        fusion_solution.velocity_covariance =
                            (fusion_solution.position_covariance +
                             previous_fixed_solution.position_covariance) /
                            (dt * dt);
                        fusion_solution.has_velocity = true;
                        ++derived_velocity_updates;
                    }
                }
            }

            fusion_processor.processGnssSolution(fusion_solution);
            if (fusion_solution.isFixed()) {
                previous_fixed_solution = fusion_solution;
                have_previous_fixed_solution = true;
            }
            if (!causal_candidate_promoted) {
                pos_solution.velocity_ecef =
                    fusion_solution.velocity_ecef;
                pos_solution.velocity_covariance =
                    fusion_solution.velocity_covariance;
                pos_solution.has_velocity =
                    fusion_solution.has_velocity;
            }
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
                    if (dd_result.sse_partial_ar.available) {
                        ++tight_dd_sse_par_available_epochs;
                    }
                    if (dd_result.sse_partial_ar.subsets_evaluated > 0) {
                        ++tight_dd_sse_par_eligible_epochs;
                    }
                    tight_dd_sse_par_subsets_evaluated +=
                        dd_result.sse_partial_ar.subsets_evaluated;
                    tight_dd_sse_par_ratio_passed_subsets +=
                        dd_result.sse_partial_ar.ratio_passed_subsets;
                    tight_dd_sse_par_separation_rejected_subsets +=
                        dd_result.sse_partial_ar
                            .separation_rejected_subsets;
                    if (dd_result.sse_partial_ar.passed) {
                        ++tight_dd_sse_par_passed_epochs;
                        tight_dd_sse_par_fixed_ambiguities +=
                            dd_result.sse_partial_ar.fixed_count;
                    }
                    Eigen::Vector3d sse_candidate_ecef =
                        Eigen::Vector3d::Constant(
                            std::numeric_limits<double>::quiet_NaN());
                    if (dd_result.sse_partial_ar.passed) {
                        const auto fused_now =
                            fusion_processor.toPositionSolution();
                        sse_candidate_ecef =
                            fused_now.position_ecef +
                            fusion_processor
                                .ecefToLocalEnuRotation()
                                .transpose() *
                                (dd_result.sse_partial_ar
                                     .fixed_position_enu -
                                 fusion_processor.state()
                                     .nominal.position_enu);
                    }
                    sse_par_writer.write(
                        pos_solution.time,
                        dd_result.sse_partial_ar,
                        sse_candidate_ecef);
                    if (dd_result.reset_action !=
                        libgnss::dd_imu_bridge::SoftResetAction::REJECTED) {
                        ++tight_dd_soft_resets;
                    }
                }
            }
            if (options.tc_ins_time_update) {
                bool anchored = false;
                Eigen::Vector3d float_position_ecef;
                Eigen::Matrix3d float_position_covariance_ecef;
                if (pos_solution.has_velocity && pos_solution.velocity_ecef.allFinite() &&
                    fusion_processor.isInitialized() && fusion_processor.isOriginSet() &&
                    fusion_processor.isHeadingConverged() &&
                    rtk_processor.getFloatPosteriorPosition(
                        float_position_ecef, float_position_covariance_ecef)) {
                    double anchor_height_m = 0.0;
                    libgnss::ecef2geodetic(float_position_ecef, tc_anchor_lat_rad,
                                           tc_anchor_lon_rad, anchor_height_m);
                    const Eigen::Matrix3d ecef_to_enu =
                        ecefToEnuRotation(tc_anchor_lat_rad, tc_anchor_lon_rad);
                    libgnss::NominalState anchor_state = fusion_processor.state().nominal;
                    anchor_state.time = rover_obs.time;
                    anchor_state.velocity_enu = ecef_to_enu * pos_solution.velocity_ecef;
                    anchor_state.position_enu =
                        -anchor_state.attitude_body_to_enu.toRotationMatrix() *
                        options.fusion_config.lever_arm_body;
                    anchored = tc_preintegrator.reset(anchor_state) ==
                        libgnss::PreintegrationStatus::ACCEPTED;
                    if (anchored) {
                        ++tc_anchor_count;
                    } else {
                        ++tc_invalid_interval_count;
                    }
                }
                if (!anchored) {
                    tc_preintegrator.clear();
                }
            }
            if (options.tc_closed_loop) {
                bool anchored = false;
                Eigen::Vector3d anchor_position_ecef;
                Eigen::Matrix3d anchor_position_covariance_ecef;
                const bool needs_bootstrap = !tc_closed_loop_processor.initialized();
                const bool bootstrap_ready = !needs_bootstrap ||
                    (fusion_processor.isInitialized() && fusion_processor.isOriginSet() &&
                     fusion_processor.isHeadingConverged());
                const bool continuation_limit_reached = options.tc_trusted_reanchor &&
                    options.tc_trusted_reanchor_max_epochs > 0 &&
                    tc_consecutive_prediction_continuations >=
                        options.tc_trusted_reanchor_max_epochs;
                const bool trusted_reanchor = !options.tc_trusted_reanchor ||
                    pos_solution.isFixed() || continuation_limit_reached;
                bool have_anchor = false;
                const auto& telemetry = rtk_processor.getLastDebugTelemetry();
                if (telemetry.ddpr_anchor_valid) {
                    libgnss::GNSSTime anchor_time;
                    have_anchor = rtk_processor.getLastDdPrAnchor(
                        anchor_position_ecef, anchor_position_covariance_ecef, anchor_time);
                }
                if (!have_anchor) {
                    have_anchor = rtk_processor.getFloatPosteriorPosition(
                        anchor_position_ecef, anchor_position_covariance_ecef);
                }
                if (trusted_reanchor && bootstrap_ready && have_anchor && pos_solution.has_velocity &&
                    pos_solution.velocity_ecef.allFinite() &&
                    pos_solution.velocity_covariance.allFinite()) {
                    const libgnss::FusionState* bootstrap =
                        needs_bootstrap ? &fusion_processor.state() : nullptr;
                    anchored = tc_closed_loop_processor.reanchor(
                        anchor_position_ecef, anchor_position_covariance_ecef,
                        pos_solution.velocity_ecef, pos_solution.velocity_covariance,
                        rover_obs.time, bootstrap);
                    if (anchored) {
                        if (continuation_limit_reached && !pos_solution.isFixed()) {
                            ++tc_trusted_reanchor_fallback_count;
                        }
                        tc_consecutive_prediction_continuations = 0;
                    }
                }
                if (!anchored) {
                    const bool continued = options.tc_trusted_reanchor && !needs_bootstrap &&
                        !continuation_limit_reached &&
                        tc_closed_loop_processor.continueFromPrediction();
                    if (continued) ++tc_consecutive_prediction_continuations;
                    if (!continued) {
                        tc_closed_loop_processor.invalidateInterval();
                    }
                }
            }
            ++valid_solutions;
            if (pos_solution.isFixed()) {
                ++fixed_solutions;
            }

            // Opt-in (--rtk-pos-out): log the RTK solution exactly as
            // processRTKEpoch() returned it, before fusion ever touches it,
            // for scoring the RTK stream in isolation (see FuseOptions doc
            // comment / gnss_solve.cpp's own solution logging for parity).
            if (!options.rtk_pos_out.empty()) {
                rtk_solution_log.addSolution(pos_solution);
            }
        } else {
            if (options.tc_ins_time_update) tc_preintegrator.clear();
            if (options.tc_closed_loop) {
                const bool continuation_limit_reached = options.tc_trusted_reanchor &&
                    options.tc_trusted_reanchor_max_epochs > 0 &&
                    tc_consecutive_prediction_continuations >=
                        options.tc_trusted_reanchor_max_epochs;
                const bool continued = options.tc_trusted_reanchor &&
                    !continuation_limit_reached &&
                    tc_closed_loop_processor.continueFromPrediction();
                if (continued) ++tc_consecutive_prediction_continuations;
                if (!continued) tc_closed_loop_processor.invalidateInterval();
            }
        }

        if (options.library_fix_integrity_gate &&
            pos_solution.isValid()) {
            // The authoritative integrity product must preserve the library
            // solution denominator even during the IMU alignment prefix.
            fused_solution.addSolution(pos_solution);
        } else if (fusion_processor.isOriginSet()) {
            fused_solution.addSolution(
                fusion_processor.toPositionSolution());
        }
        if (fusion_processor.isOriginSet()) {
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

    if (!options.rtk_pos_out.empty() && !rtk_solution_log.writeToFile(options.rtk_pos_out)) {
        std::cerr << "Error: failed to write RTK solution file: " << options.rtk_pos_out << "\n";
        return 1;
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
        std::cout << "RTK INS position prior: " << (options.rtk_ins_prior ? "on" : "off");
        if (options.rtk_ins_prior) {
            std::cout << " (inflation=" << options.rtk_ins_prior_inflation
                      << ", spp_gate=" << (options.rtk_ins_prior_spp_gate ? "on" : "off")
                      << (options.rtk_ins_prior_spp_gate
                              ? (" min_sats=" + std::to_string(options.rtk_ins_prior_min_sats))
                              : "")
                      << ", max_nis="
                      << (options.rtk_ins_prior_max_nis <= 0.0
                              ? std::string("off")
                              : std::to_string(options.rtk_ins_prior_max_nis))
                      << ")";
        }
        std::cout << "\n";
        if (options.rtk_ins_prior) {
            std::cout << "INS prior eligible epochs (initialized/anchored/heading-converged): "
                      << ins_prior_eligible_epochs << "\n";
            std::cout << "INS prior fired (actually injected): " << ins_prior_fire_count;
            if (ins_prior_eligible_epochs > 0) {
                std::cout << " (" << std::fixed << std::setprecision(2)
                          << (100.0 * ins_prior_fire_count / ins_prior_eligible_epochs)
                          << "% of eligible epochs)";
            }
            std::cout << "\n";
            if (options.verbose && !ins_prior_eligible_nsats.empty()) {
                auto percentile = [](std::vector<double> values, double p) {
                    std::sort(values.begin(), values.end());
                    size_t idx = static_cast<size_t>(p * (values.size() - 1));
                    return values[idx];
                };
                std::vector<double> nsats_as_double(ins_prior_eligible_nsats.begin(),
                                                     ins_prior_eligible_nsats.end());
                std::cout << "  eligible-epoch nsat distribution: min="
                          << percentile(nsats_as_double, 0.0) << " p10=" << percentile(nsats_as_double, 0.10)
                          << " p25=" << percentile(nsats_as_double, 0.25)
                          << " median=" << percentile(nsats_as_double, 0.50)
                          << " p75=" << percentile(nsats_as_double, 0.75)
                          << " max=" << percentile(nsats_as_double, 1.0) << "\n";
                std::cout << "  eligible-epoch velocity-NIS-EMA distribution: min="
                          << percentile(ins_prior_eligible_nis, 0.0)
                          << " p50=" << percentile(ins_prior_eligible_nis, 0.50)
                          << " p90=" << percentile(ins_prior_eligible_nis, 0.90)
                          << " max=" << percentile(ins_prior_eligible_nis, 1.0) << "\n";
            }
        }
        std::cout << "RTK INS time update: "
                  << (options.tc_ins_time_update ? "on" : "off");
        if (options.tc_ins_time_update) {
            const auto tc_diagnostics = rtk_processor.getInsTimeUpdateDiagnostics();
            std::cout << " (q_floor_m2=" << options.tc_ins_position_q_floor_m2
                      << ", max_sample_gap_s=" << options.tc_ins_max_sample_gap_s << ")\n"
                      << "  anchors=" << tc_anchor_count
                      << " supplied=" << tc_update_supplied_count
                      << " applied=" << tc_diagnostics.applied_count
                      << " rejected=" << tc_diagnostics.rejected_count
                      << " invalid_intervals=" << tc_invalid_interval_count;
        }
        std::cout << "\n";
        std::cout << "RTK tight-coupling closed loop: "
                  << (options.tc_closed_loop ? "on" : "off");
        if (options.tc_closed_loop) {
            const auto diagnostics = tc_closed_loop_processor.diagnostics();
            std::cout << " (trusted_reanchor="
                      << (options.tc_trusted_reanchor ? "on" : "off")
                      << ", max_bridge_epochs="
                      << options.tc_trusted_reanchor_max_epochs << ")\n  anchors="
                      << diagnostics.anchors
                      << " supplied=" << diagnostics.supplied_updates
                      << " continued=" << diagnostics.prediction_continuations
                      << " fallback_reanchors=" << tc_trusted_reanchor_fallback_count
                      << " invalid_intervals=" << diagnostics.invalid_intervals
                      << " zupt_updates=" << diagnostics.zupt_updates
                      << " nhc_updates=" << diagnostics.nhc_updates;
        }
        std::cout << "\n";
        std::cout << "RTK velocity states: "
                  << (options.tc_velocity_states ? "on" : "off") << "\n";
        std::cout << "RTK TDCP diagnostics: "
                  << (options.tc_tdcp_diagnostics ? "on" : "off");
        if (options.tc_tdcp_diagnostics) {
            const double rms_m = tc_tdcp_residual_count > 0
                ? std::sqrt(tc_tdcp_residual_sum_squares / tc_tdcp_residual_count)
                : std::numeric_limits<double>::quiet_NaN();
            std::cout << "\n  candidates=" << tc_tdcp_candidate_count
                      << " residuals=" << tc_tdcp_residual_count
                      << " missing_previous=" << tc_tdcp_missing_previous_count
                      << " gap=" << tc_tdcp_gap_count
                      << " loss_of_lock=" << tc_tdcp_loss_of_lock_count
                      << " invalid=" << tc_tdcp_invalid_count
                      << " rms_m=" << rms_m
                      << " max_abs_m=" << tc_tdcp_residual_max_abs_m;
        }
        std::cout << "\n";
        const bool cp_pr_gate_active = options.tc_cp_pr_gate || options.tc_closed_loop;
        std::cout << "RTK CP-vs-PR fixed gate: " << (cp_pr_gate_active ? "on" : "off");
        if (cp_pr_gate_active) {
            std::cout << " (threshold_m=" << options.tc_cp_pr_threshold_m
                      << ", min_pairs=" << options.tc_cp_pr_min_pairs
                      << ", max_bad_pairs=" << options.tc_cp_pr_max_bad_pairs
                      << ", escalation_epochs=" << options.tc_cp_pr_escalation_epochs << ")\n"
                      << "  evaluated=" << tc_cp_pr_evaluated_count
                      << " rejected=" << tc_cp_pr_rejected_count
                      << " escalated=" << tc_cp_pr_escalated_count
                      << " ddpr_anchors=" << tc_ddpr_anchor_count;
        }
        std::cout << "\n";
        if (!options.rtk_pos_out.empty()) {
            std::cout << "RTK solution stream written: " << options.rtk_pos_out << "\n";
        }
        {
            // navi.776 C: J(dt) line for scripts/search_imu_time_offset.py.
            // correction_epochs counts only INS-time-update epochs; the
            // script must reject candidates whose applied-update coverage is
            // too low for J to be meaningful.
            const auto correction_stats = rtk_processor.getPositionCorrectionStats();
            const auto tc_diag = rtk_processor.getInsTimeUpdateDiagnostics();
            std::cout << "imu_time_offset_score:"
                      << " offset_s=" << options.imu_time_offset_s
                      << " J_mean_sq_correction_m2=" << correction_stats.mean_square_m2
                      << " correction_epochs=" << correction_stats.count
                      << " applied_updates=" << tc_diag.applied_count << "\n";
        }
        if (options.cmc_aware_reference_selection) {
            const auto cmc_ref_diag = rtk_processor.getCmcReferenceDiagnostics();
            std::cout << "CMC-aware reference selection: enabled"
                      << " suspect_epochs=" << cmc_ref_diag.suspect_epoch_count
                      << " switches=" << cmc_ref_diag.switch_count << "\n";
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
            std::cout << "SSE-PAR available/passed/fixed ambiguities: "
                      << tight_dd_sse_par_available_epochs << "/"
                      << tight_dd_sse_par_passed_epochs << "/"
                      << tight_dd_sse_par_fixed_ambiguities << "\n";
            std::cout << "SSE-PAR eligible/subsets/ratio-pass/SSE-reject: "
                      << tight_dd_sse_par_eligible_epochs << "/"
                      << tight_dd_sse_par_subsets_evaluated << "/"
                      << tight_dd_sse_par_ratio_passed_subsets << "/"
                      << tight_dd_sse_par_separation_rejected_subsets
                      << "\n";
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
        const auto imu_result =
            options.rtklibexplorer_imu_format
                ? libgnss::loadRtklibExplorerImuCsv(options.imu_path, imu_series)
                : libgnss::loadImuCsv(options.imu_path, imu_series);
        if (!imu_result.ok) {
            std::cerr << "Error: failed to load IMU CSV: " << imu_result.error << "\n";
            return 1;
        }
        imu_series.sortByTime();
        // navi.776 C: constant GNSS-IMU time offset, applied before the
        // axis remap and monotone cursor ever see a sample. 0.0 (default)
        // is a guarded exact no-op.
        if (options.imu_time_offset_s != 0.0) {
            imu_series.shiftTime(options.imu_time_offset_s);
            std::cout << "IMU time offset applied: " << options.imu_time_offset_s
                      << " s\n";
        }

        // Default axis convention is identity (raw sensor axes already FLU),
        // matching the PPC-Dataset default per docs/design.md 1.1.1. A
        // future CLI revision could expose --forward-axis/--up-axis/etc. if
        // a non-PPC, non-FLU dataset needs it.
        const libgnss::ImuAxisConvention axis_convention;

        if (!options.gnss_pos_path.empty()) {
            return runPositionSolutionFusion(options, imu_series,
                                             axis_convention);
        }

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
