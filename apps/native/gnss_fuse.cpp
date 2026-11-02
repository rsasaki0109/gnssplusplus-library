#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <libgnss++/algorithms/rtk.hpp>
#include <libgnss++/algorithms/spp.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/solution.hpp>
#include <libgnss++/fusion/fusion_processor.hpp>
#include <libgnss++/fusion/rtk_imu_fusion.hpp>
#include <libgnss++/io/imu.hpp>
#include <libgnss++/io/rinex.hpp>

#include "cli_toml_config.hpp"
#include "rtk_base_epoch_align.hpp"

namespace {

constexpr double kExactTimeToleranceSeconds = libgnss_apps::kExactTimeToleranceSeconds;

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
    double elevation_mask_deg = 15.0;
    bool enable_base_interpolation = true;
    bool tightly_coupled_dd_imu = false;
    bool tightly_coupled_dd_code_only = false;
    bool integrity_student_t_front_end = false;
    bool integrity_student_t_all_measurements = false;
    bool integrity_laplacian_all_measurements = false;
    bool integrity_huber_all_measurements = false;
    double integrity_student_t_degrees_of_freedom = 4.0;
    double integrity_heavy_tail_activation_sigma = 2.5;

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
    // Mirror gnss_solve's --hold-kalman-update and --min-fix-satellites; no
    // preset touches them. Off by default.
    bool hold_kalman_update = false;
    bool hold_kalman_update_validated_only = false;
    int min_fix_satellites = 0;
    // --independent-output-filter: write --out and the attitude stream from a
    // second loose ESKF that takes the RTK position and the epoch SPP Doppler
    // velocity (consider update before the heading latch). The RTK filter and
    // the closed-loop INS bootstrap keep using the main loose ESKF. Resolved
    // after parsing: an explicit --independent-output-filter or
    // --no-independent-output-filter (the last one given wins) decides;
    // otherwise --navi776-tc turns it on when --base is given (frozen
    // SmartPNT-POS holdout, docs/smartpnt_fuse_output_holdout_results.md).
    // Off otherwise.
    bool independent_output_filter = false;
    std::optional<bool> independent_output_filter_choice;
    bool navi776_tc = false;
    std::string independent_output_filter_reason;

    // navi.776 A2: innovation-based adaptive measurement variance, mirroring
    // gnss_solve's --rtk-adaptive-noise family. Off by default.
    bool rtk_adaptive_noise = false;
    // Extend the causal fixed-anchor stabilizer to the SPP fallback output.
    bool rtk_single_stabilizer = false;
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
        << "  --ar-policy <extended|demo5-continuous>\n"
        << "  --glonass-ar <off|on|autocal>\n"
        << "  --max-pos-jump <m>           RTK absolute position jump bound\n"
        << "  --rtk-update-outlier-threshold <sigma>\n"
        << "  --lever-arm x,y,z            IMU -> antenna lever arm, body FLU, meters (default 0,0,0)\n"
        << "  --zupt / --no-zupt           Enable/disable zero-velocity updates (default: enabled)\n"
        << "  --zupt-gnss-speed-gate / --no-zupt-gnss-speed-gate\n"
        << "                                Latch suppression after last-known GNSS\n"
        << "                                speed exceeds 0.5 m/s (default: enabled)\n"
        << "  --zupt-gnss-speed-threshold-mps <mps>\n"
        << "                                Moving-speed threshold for the ZUPT gate\n"
        << "                                (default: 0.5)\n"
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
        << "  --consider-before-heading-latch\n"
        << "                                Until the heading is latched, GNSS, ZUPT and NHC updates\n"
        << "                                leave the attitude and IMU biases unchanged (Schmidt\n"
        << "                                consider update), so the unknown yaw cannot corrupt\n"
        << "                                them; while it stands still it re-aligns the gyro bias\n"
        << "                                and tilt over the stand-still instead. Changes the RTK\n"
        << "                                stream with --tc-closed-loop or --navi776-tc. Default: off\n"
        << "  --independent-output-filter  Write --out (and --attitude-csv) from a second loose\n"
        << "                                ESKF that takes the RTK position and the epoch SPP\n"
        << "                                Doppler velocity, with the consider update before the\n"
        << "                                heading latch. The RTK stream is unchanged. --base\n"
        << "                                only; on with --navi776-tc (with --base), off otherwise\n"
        << "  --no-independent-output-filter\n"
        << "                                Keep --out from the main loose ESKF under --navi776-tc,\n"
        << "                                wherever it is given. Of the two explicit flags, the\n"
        << "                                last one wins\n"
        << "  --max-position-nis <v>       Reject a GNSS position update if its normalized\n"
        << "                                innovation squared per observation exceeds this (guards\n"
        << "                                against RTK float-reconvergence jumps corrupting attitude\n"
        << "                                via the lever-arm coupling; <=0 disables). Default: 500.0\n"
        << "  --max-velocity-nis <v>       Same gate for the GNSS velocity update. Default: 500.0\n"
        << "  --max-consecutive-velocity-gate-rejections <n>\n"
        << "                                After this many consecutive velocity-gate\n"
        << "                                rejections, allow a bounded Doppler velocity-only\n"
        << "                                re-anchor (<=0 disables). Default: 3\n"
        << "  --max-gnss-velocity-reanchor-mps <mps>\n"
        << "                                Maximum velocity correction accepted by the\n"
        << "                                velocity-only re-anchor. Default: 20.0\n"
        << "  --max-consecutive-gate-rejections <n>\n"
        << "                                After this many consecutive position-gate rejections,\n"
        << "                                allow a trusted FIXED position-only re-anchor; FLOAT,\n"
        << "                                velocity, attitude, and bias updates are never forced\n"
        << "                                through (<=0 disables). Default: 30\n"
        << "  --preset <survey|low-cost|moving-base|odaiba>\n"
        << "                                RTK tuning preset, only used with --base (default: none)\n"
        << "  --ratio <value>              RTK ambiguity ratio threshold (default: 3.0)\n"
        << "  --max-baseline-m <v>         Max RTK baseline length in meters (default: 20000)\n"
        << "  --elevation-mask-deg <v>     RTK elevation mask in degrees (default: 15)\n"
        << "  --base-ecef <x> <y> <z>      Override base ECEF position in meters\n"
        << "  --no-base-interp             Require exact rover/base epoch alignment (RTK path)\n"
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
        << "                                (1000 m), and --independent-output-filter with --base.\n"
        << "                                Flags given after this one override it.\n"
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
        << "  --tc-cp-pr-threshold <m>     M2 fixed-candidate CP-vs-PR gate (on with --tc-closed-loop):\n"
        << "                                per-pair absolute innovation threshold (default: 10).\n"
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
        << "  --hold-kalman-update         Apply held DD integers as a Kalman pseudo-observation\n"
        << "                                instead of overwriting the ambiguities\n"
        << "                                (mirrors `gnss solve`; default: off)\n"
        << "  --hold-kalman-update-validated-only\n"
        << "                                With --hold-kalman-update, apply it only after a\n"
        << "                                validated AR fix, not after the hold-fix fallback\n"
        << "                                (mirrors `gnss solve`; default: off)\n"
        << "  --min-fix-satellites <k>     Min non-reference satellites in an ambiguity-resolution\n"
        << "                                FIX (mirrors `gnss solve`; default: 0 = off)\n"
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
            {"--independent-output-filter", "--no-independent-output-filter"},
            {"--nhc", "--no-nhc"},
            {"--navi776-tc", ""},
            {"--zupt", "--no-zupt"},
            {"--zupt-gnss-speed-gate", "--no-zupt-gnss-speed-gate"},
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
    // A gated update is never retried ungated. Once the position channel has
    // rejected this many consecutive epochs, a FIXED solution may perform a
    // position-only re-anchor; velocity, attitude, and bias states remain
    // protected by their ordinary NIS gate.
    options.fusion_config.max_position_update_nis_per_observation = 500.0;
    options.fusion_config.max_velocity_update_nis_per_observation = 500.0;
    options.fusion_config.max_consecutive_gate_rejections = 30;
    options.fusion_config.max_consecutive_velocity_gate_rejections = 3;
    options.fusion_config.max_gnss_velocity_reanchor_mps = 20.0;
    std::string imu_grade = "tactical";
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        // MSVC counts every `else if` as one more nesting level and rejects
        // chains deeper than 128 (C1061), so the option chain is split in
        // two: the first half `continue`s once it has consumed `arg`.
        bool matched = true;
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
        } else if (arg == "--lever-arm") {
            options.fusion_config.lever_arm_body =
                parseVector3(requireValue(arg, i, argc, argv), arg);
        } else if (arg == "--zupt") {
            options.fusion_config.zupt_enable = true;
        } else if (arg == "--no-zupt") {
            options.fusion_config.zupt_enable = false;
        } else if (arg == "--zupt-gnss-speed-gate") {
            options.fusion_config.zupt_gnss_speed_gate_enable = true;
        } else if (arg == "--no-zupt-gnss-speed-gate") {
            options.fusion_config.zupt_gnss_speed_gate_enable = false;
        } else if (arg == "--zupt-gnss-speed-threshold-mps") {
            options.fusion_config.zupt_gnss_speed_gate_threshold_mps =
                std::stod(requireValue(arg, i, argc, argv));
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
        } else if (arg == "--consider-before-heading-latch") {
            options.fusion_config.consider_attitude_and_biases_before_heading_latch = true;
        } else if (arg == "--independent-output-filter") {
            options.independent_output_filter_choice = true;
        } else if (arg == "--no-independent-output-filter") {
            options.independent_output_filter_choice = false;
        } else {
            matched = false;
        }
        if (matched) {
            continue;
        }

        if (arg == "--max-position-nis") {
            options.fusion_config.max_position_update_nis_per_observation =
                std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--max-velocity-nis") {
            options.fusion_config.max_velocity_update_nis_per_observation =
                std::stod(requireValue(arg, i, argc, argv));
        } else if (arg == "--max-consecutive-velocity-gate-rejections") {
            options.fusion_config.max_consecutive_velocity_gate_rejections =
                std::stoi(requireValue(arg, i, argc, argv));
        } else if (arg == "--max-gnss-velocity-reanchor-mps") {
            options.fusion_config.max_gnss_velocity_reanchor_mps =
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
            // Also requests the independent output filter, resolved after
            // parsing (needs --base); --no-independent-output-filter wins
            // whatever its position.
            options.navi776_tc = true;
        } else if (arg == "--tc-closed-loop") {
            options.tc_closed_loop = true;
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
        } else if (arg == "--hold-kalman-update") {
            options.hold_kalman_update = true;
        } else if (arg == "--hold-kalman-update-validated-only") {
            options.hold_kalman_update_validated_only = true;
        } else if (arg == "--min-fix-satellites") {
            options.min_fix_satellites = std::stoi(requireValue(arg, i, argc, argv));
            if (options.min_fix_satellites < 0) {
                argumentError("--min-fix-satellites must be >= 0", argv[0]);
            }
        } else if (arg == "--rtk-adaptive-noise") {
            options.rtk_adaptive_noise = true;
        } else if (arg == "--rtk-single-stabilizer") {
            options.rtk_single_stabilizer = true;
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

    if (options.independent_output_filter_choice.has_value()) {
        options.independent_output_filter = *options.independent_output_filter_choice;
        if (!options.independent_output_filter && options.navi776_tc) {
            options.independent_output_filter_reason = "--no-independent-output-filter";
        }
    } else if (options.navi776_tc) {
        if (options.base_path.empty()) {
            options.independent_output_filter_reason = "--navi776-tc without --base";
        } else {
            options.independent_output_filter = true;
            options.independent_output_filter_reason = "--navi776-tc";
        }
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
    if (options.hold_kalman_update_validated_only && !options.hold_kalman_update) {
        argumentError("--hold-kalman-update-validated-only requires --hold-kalman-update",
                      argv[0]);
    }
    if (options.independent_output_filter && options.base_path.empty()) {
        argumentError("--independent-output-filter requires --base", argv[0]);
    }
    if (options.tightly_coupled_dd_imu) {
        options.fusion_config.position_updates_require_fixed = true;
    }
    if (!std::isfinite(options.fusion_config.zupt_gnss_speed_gate_threshold_mps) ||
        options.fusion_config.zupt_gnss_speed_gate_threshold_mps < 0.0) {
        argumentError(
            "--zupt-gnss-speed-threshold-mps must be finite and non-negative",
            argv[0]);
    }
    if (options.fusion_config.max_consecutive_velocity_gate_rejections < 0) {
        argumentError(
            "--max-consecutive-velocity-gate-rejections must be non-negative",
            argv[0]);
    }
    if (!std::isfinite(options.fusion_config.max_gnss_velocity_reanchor_mps) ||
        options.fusion_config.max_gnss_velocity_reanchor_mps < 0.0) {
        argumentError(
            "--max-gnss-velocity-reanchor-mps must be finite and non-negative",
            argv[0]);
    }
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
// Run-summary line for a run where --navi776-tc asked for the independent
// output filter but it stayed off (no --base, or the explicit opt-out).
void printIndependentOutputFilterOffReason(const FuseOptions& options) {
    if (!options.independent_output_filter &&
        !options.independent_output_filter_reason.empty()) {
        std::cout << "Independent output filter: off ("
                  << options.independent_output_filter_reason << ")\n";
    }
}

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
            fused_solution.addSolution(fusion_processor.toAntennaPositionSolution());
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
        printIndependentOutputFilterOffReason(options);
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
            fused_solution.addSolution(fusion_processor.toAntennaPositionSolution());
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
                  << (fusion_processor.isInitialized() ? "yes" : "no") << "\n";
        printIndependentOutputFilterOffReason(options);
        std::cout << "Output: " << options.out_path << "\n";
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

    libgnss::RTKProcessor::RTKConfig rtk_config;
    rtk_config.max_baseline_length = options.max_baseline_length_m;
    rtk_config.ar_mode = libgnss::RTKProcessor::RTKConfig::AmbiguityResolutionMode::CONTINUOUS;
    rtk_config.ar_policy = options.ar_policy;
    rtk_config.glonass_ar_mode = options.glonass_ar_mode;
    rtk_config.max_position_jump_m =
        options.max_position_jump_m;
    if (options.rtk_update_outlier_threshold > 0.0) {
        rtk_config.outlier_threshold =
            options.rtk_update_outlier_threshold;
    }
    if (options.ratio_threshold_set) {
        rtk_config.ratio_threshold = options.ratio_threshold;
        rtk_config.ambiguity_ratio_threshold = options.ratio_threshold;
    }
    rtk_config.elevation_mask = options.elevation_mask_deg * M_PI / 180.0;
    rtk_config.enable_fixed_anchor_float_stabilization =
        options.tc_doppler_rows &&
        options.rtk_adaptive_noise &&
        options.tc_doppler_max_baseline_m == 1000.0 &&
        options.rtk_adaptive_noise_max_baseline_m == 1000.0;
    rtk_config.enable_fixed_anchor_single_stabilization =
        options.rtk_single_stabilizer;
    rtk_config.enable_doppler_measurement_rows = options.tc_doppler_rows;
    rtk_config.reuse_kalman_factorization_for_nis =
        options.tc_reuse_update_factorization;
    rtk_config.sequential_doppler_update =
        options.tc_sequential_doppler_update;
    rtk_config.doppler_row_sigma_mps = options.tc_doppler_sigma_mps;
    rtk_config.doppler_row_max_baseline_m = options.tc_doppler_max_baseline_m;
    rtk_config.ins_time_update_position_q_floor_m2 = options.tc_ins_position_q_floor_m2;
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
    rtk_config.hold_ambiguity_kalman_update = options.hold_kalman_update;
    rtk_config.hold_ambiguity_kalman_update_validated_only =
        options.hold_kalman_update_validated_only;
    rtk_config.min_fix_satellites = options.min_fix_satellites;
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
    // The heavy-tail (Student-t / Huber / Laplacian) front-end: a
    // measurement-domain robust weight on the primary RTK update
    // (docs/ppc_rtk_vs_gnss_imu_fusion.md).
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
    libgnss::RtkImuFusionProcessor::Config rtk_imu_config;
    rtk_imu_config.fusion_config = options.fusion_config;
    rtk_imu_config.independent_output_filter = options.independent_output_filter;
    rtk_imu_config.rtk_ins_prior = options.rtk_ins_prior;
    rtk_imu_config.rtk_ins_prior_inflation = options.rtk_ins_prior_inflation;
    rtk_imu_config.rtk_ins_prior_spp_gate = options.rtk_ins_prior_spp_gate;
    rtk_imu_config.rtk_ins_prior_min_sats = options.rtk_ins_prior_min_sats;
    rtk_imu_config.rtk_ins_prior_max_nis = options.rtk_ins_prior_max_nis;
    rtk_imu_config.record_ins_prior_signals = options.verbose;
    rtk_imu_config.tc_ins_time_update = options.tc_ins_time_update;
    rtk_imu_config.tc_closed_loop = options.tc_closed_loop;
    rtk_imu_config.tc_velocity_states = options.tc_velocity_states;
    rtk_imu_config.tc_tdcp_diagnostics = options.tc_tdcp_diagnostics;
    rtk_imu_config.tc_ins_max_sample_gap_s = options.tc_ins_max_sample_gap_s;
    rtk_imu_config.tightly_coupled_dd_imu = options.tightly_coupled_dd_imu;
    rtk_imu_config.tightly_coupled_dd_code_only = options.tightly_coupled_dd_code_only;
    libgnss::RtkImuFusionProcessor rtk_imu(rtk_imu_config, rtk_config, base_position);
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
            transformImuSample(sample, options, axis_convention);
            rtk_imu.processImuSample(sample);
            ++imu_cursor;
        }

        auto epoch = rtk_imu.processEpoch(rover_obs, aligned_base_obs, nav_data);
        const auto& pos_solution = epoch.rtk;
        if (epoch.rtk_valid) {
            ++valid_solutions;
            if (pos_solution.isFixed()) {
                ++fixed_solutions;
            }

            // Opt-in (--rtk-pos-out): log the RTK solution stream
            // (RtkImuFusionProcessor::EpochResult::rtk) for scoring the RTK
            // stream in isolation (see FuseOptions doc comment /
            // gnss_solve.cpp's own solution logging for parity).
            if (!options.rtk_pos_out.empty()) {
                rtk_solution_log.addSolution(pos_solution);
            }
        }

        if (epoch.has_output) {
            fused_solution.addSolution(epoch.output);
            attitude_writer.write(epoch.attitude_time, epoch.attitude_body_to_enu);
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
        const auto& counters = rtk_imu.counters();
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
                      << counters.ins_prior_eligible_epochs << "\n";
            std::cout << "INS prior fired (actually injected): " << counters.ins_prior_fire_count;
            if (counters.ins_prior_eligible_epochs > 0) {
                std::cout << " (" << std::fixed << std::setprecision(2)
                          << (100.0 * counters.ins_prior_fire_count / counters.ins_prior_eligible_epochs)
                          << "% of eligible epochs)";
            }
            std::cout << "\n";
            if (options.verbose && !counters.ins_prior_eligible_nsats.empty()) {
                auto percentile = [](std::vector<double> values, double p) {
                    std::sort(values.begin(), values.end());
                    size_t idx = static_cast<size_t>(p * (values.size() - 1));
                    return values[idx];
                };
                std::vector<double> nsats_as_double(counters.ins_prior_eligible_nsats.begin(),
                                                     counters.ins_prior_eligible_nsats.end());
                std::cout << "  eligible-epoch nsat distribution: min="
                          << percentile(nsats_as_double, 0.0) << " p10=" << percentile(nsats_as_double, 0.10)
                          << " p25=" << percentile(nsats_as_double, 0.25)
                          << " median=" << percentile(nsats_as_double, 0.50)
                          << " p75=" << percentile(nsats_as_double, 0.75)
                          << " max=" << percentile(nsats_as_double, 1.0) << "\n";
                std::cout << "  eligible-epoch velocity-NIS-EMA distribution: min="
                          << percentile(counters.ins_prior_eligible_nis, 0.0)
                          << " p50=" << percentile(counters.ins_prior_eligible_nis, 0.50)
                          << " p90=" << percentile(counters.ins_prior_eligible_nis, 0.90)
                          << " max=" << percentile(counters.ins_prior_eligible_nis, 1.0) << "\n";
            }
        }
        std::cout << "RTK INS time update: "
                  << (options.tc_ins_time_update ? "on" : "off");
        if (options.tc_ins_time_update) {
            const auto tc_diagnostics = rtk_imu.rtk().getInsTimeUpdateDiagnostics();
            std::cout << " (q_floor_m2=" << options.tc_ins_position_q_floor_m2
                      << ", max_sample_gap_s=" << options.tc_ins_max_sample_gap_s << ")\n"
                      << "  anchors=" << counters.tc_anchor_count
                      << " supplied=" << counters.tc_update_supplied_count
                      << " applied=" << tc_diagnostics.applied_count
                      << " rejected=" << tc_diagnostics.rejected_count
                      << " invalid_intervals=" << counters.tc_invalid_interval_count;
        }
        std::cout << "\n";
        std::cout << "RTK tight-coupling closed loop: "
                  << (options.tc_closed_loop ? "on" : "off");
        if (options.tc_closed_loop) {
            const auto diagnostics = rtk_imu.closedLoop().diagnostics();
            std::cout << "\n  anchors="
                      << diagnostics.anchors
                      << " supplied=" << diagnostics.supplied_updates
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
            const double rms_m = counters.tc_tdcp_residual_count > 0
                ? std::sqrt(counters.tc_tdcp_residual_sum_squares / counters.tc_tdcp_residual_count)
                : std::numeric_limits<double>::quiet_NaN();
            std::cout << "\n  candidates=" << counters.tc_tdcp_candidate_count
                      << " residuals=" << counters.tc_tdcp_residual_count
                      << " missing_previous=" << counters.tc_tdcp_missing_previous_count
                      << " gap=" << counters.tc_tdcp_gap_count
                      << " loss_of_lock=" << counters.tc_tdcp_loss_of_lock_count
                      << " invalid=" << counters.tc_tdcp_invalid_count
                      << " rms_m=" << rms_m
                      << " max_abs_m=" << counters.tc_tdcp_residual_max_abs_m;
        }
        std::cout << "\n";
        const bool cp_pr_gate_active = options.tc_closed_loop;
        std::cout << "RTK CP-vs-PR fixed gate: " << (cp_pr_gate_active ? "on" : "off");
        if (cp_pr_gate_active) {
            std::cout << " (threshold_m=" << options.tc_cp_pr_threshold_m
                      << ", min_pairs=" << options.tc_cp_pr_min_pairs
                      << ", max_bad_pairs=" << options.tc_cp_pr_max_bad_pairs
                      << ", escalation_epochs=" << options.tc_cp_pr_escalation_epochs << ")\n"
                      << "  evaluated=" << counters.tc_cp_pr_evaluated_count
                      << " rejected=" << counters.tc_cp_pr_rejected_count
                      << " escalated=" << counters.tc_cp_pr_escalated_count
                      << " ddpr_anchors=" << counters.tc_ddpr_anchor_count;
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
            const auto correction_stats = rtk_imu.rtk().getPositionCorrectionStats();
            const auto tc_diag = rtk_imu.rtk().getInsTimeUpdateDiagnostics();
            std::cout << "imu_time_offset_score:"
                      << " offset_s=" << options.imu_time_offset_s
                      << " J_mean_sq_correction_m2=" << correction_stats.mean_square_m2
                      << " correction_epochs=" << correction_stats.count
                      << " applied_updates=" << tc_diag.applied_count << "\n";
        }
        if (options.cmc_aware_reference_selection) {
            const auto cmc_ref_diag = rtk_imu.rtk().getCmcReferenceDiagnostics();
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
        if (options.tightly_coupled_dd_imu) {
            std::cout << "Tight DD/IMU epochs: " << counters.tight_dd_epochs
                      << " (accepted=" << counters.tight_dd_accepted
                      << ", innovation_rejected=" << counters.tight_dd_rejected << ")\n";
            std::cout << "Tight DD rows: " << counters.tight_dd_rows << "\n";
            std::cout << "Carrier-to-code fallbacks: "
                      << counters.tight_dd_carrier_fallbacks << "\n";
            std::cout << "Tight DD NIS/observation mean/max: "
                      << (counters.tight_dd_nis_samples > 0
                              ? counters.tight_dd_nis_sum / counters.tight_dd_nis_samples
                              : 0.0)
                      << "/" << counters.tight_dd_nis_max << "\n";
            std::cout << "Partial-AR epochs/fixed ambiguities: "
                      << counters.tight_dd_partial_ar_epochs << "/"
                      << counters.tight_dd_fixed_ambiguities << "\n";
            std::cout << "SSE-PAR available/passed/fixed ambiguities: "
                      << counters.tight_dd_sse_par_available_epochs << "/"
                      << counters.tight_dd_sse_par_passed_epochs << "/"
                      << counters.tight_dd_sse_par_fixed_ambiguities << "\n";
            std::cout << "SSE-PAR eligible/subsets/ratio-pass/SSE-reject: "
                      << counters.tight_dd_sse_par_eligible_epochs << "/"
                      << counters.tight_dd_sse_par_subsets_evaluated << "/"
                      << counters.tight_dd_sse_par_ratio_passed_subsets << "/"
                      << counters.tight_dd_sse_par_separation_rejected_subsets
                      << "\n";
            std::cout << "Innovation-gated soft resets: " << counters.tight_dd_soft_resets << "\n";
        }
        std::cout << "Fused epochs written: " << fused_solution.size() << "\n";
        std::cout << "Fusion initialized: " << (rtk_imu.fusion().isInitialized() ? "yes" : "no") << "\n";
        // isHeadingAligned() only means "a latch has happened at some point"
        // -- it used to print "yes" even on runs where the latch was badly
        // wrong and never recovered (PPC nagoya/run1 investigation).
        // isHeadingConverged() is the real health signal: aligned AND the
        // recent velocity-innovation NIS still corroborates that latch.
        std::cout << "Heading aligned: " << (rtk_imu.fusion().isHeadingAligned() ? "yes" : "no") << "\n";
        std::cout << "Heading converged: " << (rtk_imu.fusion().isHeadingConverged() ? "yes" : "no") << "\n";
        if (options.independent_output_filter) {
            std::cout << "Independent output filter: on (heading aligned: "
                      << (rtk_imu.outputFilter().isHeadingAligned() ? "yes" : "no")
                      << ", converged: " << (rtk_imu.outputFilter().isHeadingConverged() ? "yes" : "no")
                      << ")\n";
        } else {
            printIndependentOutputFilterOffReason(options);
        }
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
