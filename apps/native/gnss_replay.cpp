#include <Eigen/Dense>

#include <cmath>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include <libgnss++/algorithms/rtk.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/io/rinex.hpp>
#include <libgnss++/io/observation_epoch_assembler.hpp>
#include <libgnss++/io/rtcm.hpp>
#include <libgnss++/io/rtcm_stream.hpp>
#include <libgnss++/io/solution_writer.hpp>
#include <libgnss++/io/ubx.hpp>

#include "navigation_merge.hpp"
#include "rtcm_frame_common.hpp"

namespace {

using libgnss_apps::buildRtcmFrame;
using libgnss_apps::mergeNavigationData;

constexpr double kExactTimeToleranceSeconds = 1e-3;

enum class ModeChoice {
    KINEMATIC,
    STATIC,
    MOVING_BASE
};

enum class GlonassARChoice {
    OFF,
    ON,
    AUTOCAL
};

enum class RTKTuningPreset {
    NONE,
    SURVEY,
    LOW_COST,
    MOVING_BASE,
    URBAN
};

struct ReplayConfig {
    std::string rover_rinex_path;
    std::string rover_ubx_path;
    std::string base_rinex_path;
    std::string base_ubx_path;
    std::string base_rtcm_path;
    std::string nav_rinex_path;
    std::string output_pos_path = "output/replay_solution.pos";
    libgnss::io::SolutionWriter::Format output_format = libgnss::io::SolutionWriter::Format::POS;
    bool pos_uncertainty = false;
    bool pos_fix_diagnostics = false;
    ModeChoice mode = ModeChoice::KINEMATIC;
    bool verbose = false;
    bool quiet = false;
    int max_epochs = -1;
    size_t rtcm_message_limit = 0;
    // Merge per-constellation MSM messages of one epoch (RTCM multiple-message bit).
    bool assemble_rtcm_epochs = true;
    bool base_position_override = false;
    Eigen::Vector3d base_position_ecef = Eigen::Vector3d::Zero();
    double ratio_threshold = 3.0;
    bool enable_ar_filter = false;
    bool has_ar_filter_override = false;
    double ar_filter_margin = 0.25;
    int min_satellites_for_ar = 5;
    int min_hold_count = 5;
    double hold_ratio_threshold = 2.0;
    bool hold_fix_unchanged_set = false;
    int hold_fix_min_matched_pairs = 4;
    int min_fix_satellites = 0;
    double min_full_ratio_for_subset_ar = 0.0;
    double max_fixed_anchor_age_s = 0.0;
    double max_fixed_doppler_consensus_m = 0.0;
    double max_fixed_doppler_consensus_age_s = 0.0;
    double low_ratio_guard_threshold = 0.0;
    int low_ratio_min_fixed_ambiguities = 0;
    bool fix_history_motion_compensation = false;
    bool max_position_jump_motion_compensation = false;
    double max_position_jump_min_m = 0.0;
    double max_position_jump_rate_mps = 0.0;
    double max_float_prefit_residual_rms_m = 0.0;
    double max_float_prefit_residual_max_m = 0.0;
    int max_float_prefit_residual_reset_streak = 3;
    double elevation_mask_deg = 15.0;
    bool enable_glonass = true;
    bool enable_beidou = true;
    GlonassARChoice glonass_ar = GlonassARChoice::OFF;
    RTKTuningPreset preset = RTKTuningPreset::NONE;
    bool ratio_threshold_set = false;
    bool ar_filter_margin_set = false;
    bool min_satellites_for_ar_set = false;
    bool min_hold_count_set = false;
    bool hold_ratio_threshold_set = false;
    bool min_full_ratio_for_subset_ar_set = false;
    bool max_fixed_anchor_age_s_set = false;
    bool max_fixed_doppler_consensus_m_set = false;
    bool max_fixed_doppler_consensus_age_s_set = false;
    bool low_ratio_guard_threshold_set = false;
    bool low_ratio_min_fixed_ambiguities_set = false;
    bool max_position_jump_min_m_set = false;
    bool max_position_jump_rate_mps_set = false;
    bool max_float_prefit_residual_rms_m_set = false;
    bool max_float_prefit_residual_max_m_set = false;
    bool max_float_prefit_residual_reset_streak_set = false;
};

double timeDiffSeconds(const libgnss::GNSSTime& a, const libgnss::GNSSTime& b) {
    return a - b;
}

std::string outputFormatString(libgnss::io::SolutionWriter::Format format) {
    switch (format) {
        case libgnss::io::SolutionWriter::Format::POS:
            return "pos";
        case libgnss::io::SolutionWriter::Format::LLH:
            return "llh";
        case libgnss::io::SolutionWriter::Format::XYZ:
            return "xyz";
    }
    return "pos";
}

void printUsage(const char* argv0) {
    std::cout
        << "Usage: " << argv0 << " [options]\n"
        << "  Rover input (choose one)\n"
        << "    --rover-rinex <file>      Rover observation RINEX\n"
        << "    --rover-ubx <file>        Rover UBX file with NAV-PVT / RXM-RAWX\n"
        << "  Base input (choose one)\n"
        << "    --base-rinex <file>       Base observation RINEX\n"
        << "    --base-ubx <file>         Base UBX file with NAV-PVT / RXM-RAWX\n"
        << "    --base-rtcm <path|url>    Base RTCM file or ntrip:// source\n"
        << "  Navigation\n"
        << "    --nav-rinex <file>        Broadcast navigation RINEX (optional if RTCM carries nav)\n"
        << "  Output\n"
        << "    --out <file>              Output solution file (default: output/replay_solution.pos)\n"
        << "    --format <pos|llh|xyz>    Output format (default: pos)\n"
        << "    --pos-uncertainty         Append sdn sde sdu sdne sdeu sdun (m) and DiffAge(s) to .pos rows\n"
        << "    --pos-fix-diagnostics     Append FloatFixH(m) FloatFixU(m) to .pos rows (nan unless FIXED; diagnostic only)\n"
        << "  Solver\n"
        << "    --mode <kinematic|static|moving-base> Replay mode (default: kinematic)\n"
        << "    --ratio <value>           Ambiguity ratio threshold (default: 3.0)\n"
        << "    --preset <survey|low-cost|moving-base|urban>\n"
        << "                              Apply a named RTK tuning preset\n"
        << "    --arfilter                Require extra ratio margin for subset AR fixes\n"
        << "    --no-arfilter             Disable subset AR filter margin even if a preset enables it\n"
        << "    --arfilter-margin <v>     Extra ratio margin for --arfilter (default: 0.25)\n"
        << "    --min-ar-sats <n>         Minimum satellites for AR (default: 5)\n"
        << "    --min-hold-count <n>      Consecutive fixes before hold ambiguity is allowed (default: 5)\n"
        << "    --hold-ratio-threshold <v> Ratio threshold used while hold ambiguity is active (default: 2.0)\n"
        << "    --hold-fix-unchanged-set  Reject the hold-fix fallback when a DD pair has no held integer (default: off)\n"
        << "    --hold-fix-min-matched-pairs <n> Min held DD pairs for the hold-fix fallback (default: 4)\n"
        << "    --min-fix-satellites <k>  Min non-reference satellites in an ambiguity-resolution FIX (default: 0 = off)\n"
        << "    --min-full-ratio-for-subset-ar <v> Min full-constellation LAMBDA ratio to accept a subset-AR fix (0=off)\n"
        << "    --max-fixed-anchor-age <s> Expire a FIX anchor after s seconds (0=never)\n"
        << "    --max-fixed-doppler-consensus <m> Reject a FIX more than m from the Doppler track (0=off)\n"
        << "    --max-fixed-doppler-consensus-age <s> Doppler track stops vetoing s seconds after its last FIX (0=never)\n"
        << "    --low-ratio-guard-threshold <v> Below this ratio a FIX needs the minimum fixed ambiguities (0=off)\n"
        << "    --low-ratio-min-fixed-ambiguities <n> Minimum fixed ambiguities for that guard (0=off)\n"
        << "    --fix-history-motion-compensation Allow for rover motion in the FIX history check\n"
        << "    --max-position-jump-motion-compensation Allow for rover motion in the max position jump check\n"
        << "    --max-pos-jump-min <m>    Floor of the motion-aware fixed-jump gate [m] (0=static gate)\n"
        << "    --max-pos-jump-rate <m/s> Per-second growth of the fixed-jump gate; needed for a moving rover (0=off)\n"
        << "    --max-float-prefit-rms <m> Float prefit DD residual RMS that triggers divergence reset (0=off)\n"
        << "    --max-float-prefit-max <m> Float prefit DD residual max that triggers divergence reset (0=off)\n"
        << "    --max-float-prefit-reset-streak <n> Consecutive high-residual epochs before reset (default: 3)\n"
        << "    --elevation-mask-deg <v>  Elevation mask in degrees (default: 15)\n"
        << "    --no-glonass              Disable GLONASS carrier processing\n"
        << "    --no-beidou               Disable BeiDou carrier processing\n"
        << "    --glonass-ar <off|on|autocal>\n"
        << "  Replay\n"
        << "    --max-epochs <n>          Stop after n rover epochs\n"
        << "    --rtcm-message-limit <n>  Stop RTCM ingest after n messages (0 = all)\n"
        << "    --no-rtcm-epoch-assembly  Treat every base RTCM observation message as its own epoch\n"
        << "                              (default: merge the per-constellation MSM messages of one epoch)\n"
        << "    --base-ecef <x> <y> <z>   Override base ECEF position\n"
        << "    --quiet                   Suppress per-epoch prints\n"
        << "    --verbose                 Print per-epoch solve details\n"
        << "    -h, --help                Show this help\n";
}

[[noreturn]] void argumentError(const std::string& message, const char* argv0) {
    std::cerr << "Argument error: " << message << "\n\n";
    printUsage(argv0);
    std::exit(1);
}

ModeChoice parseMode(const std::string& value, const char* argv0) {
    if (value == "kinematic") return ModeChoice::KINEMATIC;
    if (value == "static") return ModeChoice::STATIC;
    if (value == "moving-base") return ModeChoice::MOVING_BASE;
    argumentError("unsupported --mode value: " + value, argv0);
}

GlonassARChoice parseGlonassARChoice(const std::string& value, const char* argv0) {
    if (value == "off") return GlonassARChoice::OFF;
    if (value == "on") return GlonassARChoice::ON;
    if (value == "autocal") return GlonassARChoice::AUTOCAL;
    argumentError("unsupported --glonass-ar value: " + value, argv0);
}

RTKTuningPreset parseRTKTuningPreset(const std::string& value, const char* argv0) {
    if (value == "survey") return RTKTuningPreset::SURVEY;
    if (value == "low-cost") return RTKTuningPreset::LOW_COST;
    if (value == "moving-base") return RTKTuningPreset::MOVING_BASE;
    if (value == "urban") return RTKTuningPreset::URBAN;
    argumentError("unsupported --preset value: " + value, argv0);
}

void applyRTKTuningPreset(ReplayConfig& config) {
    switch (config.preset) {
        case RTKTuningPreset::NONE:
            return;
        case RTKTuningPreset::SURVEY:
            if (!config.ratio_threshold_set) config.ratio_threshold = 3.0;
            if (!config.has_ar_filter_override) config.enable_ar_filter = false;
            if (!config.ar_filter_margin_set) config.ar_filter_margin = 0.25;
            if (!config.min_satellites_for_ar_set) config.min_satellites_for_ar = 5;
            if (!config.min_hold_count_set) config.min_hold_count = 5;
            if (!config.hold_ratio_threshold_set) config.hold_ratio_threshold = 2.0;
            return;
        case RTKTuningPreset::LOW_COST:
            if (!config.ratio_threshold_set) config.ratio_threshold = 3.0;
            if (!config.has_ar_filter_override) config.enable_ar_filter = true;
            if (!config.ar_filter_margin_set) config.ar_filter_margin = 0.35;
            if (!config.min_satellites_for_ar_set) config.min_satellites_for_ar = 6;
            if (!config.min_hold_count_set) config.min_hold_count = 8;
            if (!config.hold_ratio_threshold_set) config.hold_ratio_threshold = 2.5;
            // Motion-aware kinematic gates (PR #177/#178): a moving low-cost rover
            // outruns the static 5 m jump gate, starving fixes; loose prefit-residual
            // reset recovers diverged floats. Mirrors gnss_solve LOW_COST defaults.
            if (!config.max_position_jump_rate_mps_set) config.max_position_jump_rate_mps = 30.0;
            if (!config.max_position_jump_min_m_set) config.max_position_jump_min_m = 5.0;
            if (!config.min_full_ratio_for_subset_ar_set) config.min_full_ratio_for_subset_ar = 1.5;
            if (!config.max_float_prefit_residual_rms_m_set) config.max_float_prefit_residual_rms_m = 4.0;
            if (!config.max_float_prefit_residual_max_m_set) config.max_float_prefit_residual_max_m = 10.0;
            if (!config.max_float_prefit_residual_reset_streak_set) config.max_float_prefit_residual_reset_streak = 5;
            return;
        case RTKTuningPreset::URBAN:
            // Same as gnss_solve --preset urban: the CDME fix-recovery recipe of
            // docs/rtk_doppler_consensus_age.md on top of the default profile.
            if (!config.max_fixed_anchor_age_s_set) config.max_fixed_anchor_age_s = 5.0;
            if (!config.max_fixed_doppler_consensus_m_set) config.max_fixed_doppler_consensus_m = 3.0;
            if (!config.max_fixed_doppler_consensus_age_s_set) config.max_fixed_doppler_consensus_age_s = 5.0;
            if (!config.low_ratio_guard_threshold_set) config.low_ratio_guard_threshold = 1e9;
            if (!config.low_ratio_min_fixed_ambiguities_set) config.low_ratio_min_fixed_ambiguities = 10;
            config.fix_history_motion_compensation = true;
            config.max_position_jump_motion_compensation = true;
            return;
        case RTKTuningPreset::MOVING_BASE:
            if (!config.ratio_threshold_set) config.ratio_threshold = 2.8;
            if (!config.has_ar_filter_override) config.enable_ar_filter = true;
            if (!config.ar_filter_margin_set) config.ar_filter_margin = 0.20;
            if (!config.min_satellites_for_ar_set) config.min_satellites_for_ar = 6;
            if (!config.min_hold_count_set) config.min_hold_count = 8;
            if (!config.hold_ratio_threshold_set) config.hold_ratio_threshold = 2.4;
            return;
    }
}

libgnss::io::SolutionWriter::Format parseOutputFormat(const std::string& value, const char* argv0) {
    if (value == "pos") return libgnss::io::SolutionWriter::Format::POS;
    if (value == "llh") return libgnss::io::SolutionWriter::Format::LLH;
    if (value == "xyz") return libgnss::io::SolutionWriter::Format::XYZ;
    argumentError("unsupported --format value: " + value, argv0);
}

const char* modeChoiceString(ModeChoice mode) {
    switch (mode) {
        case ModeChoice::KINEMATIC:
            return "kinematic";
        case ModeChoice::STATIC:
            return "static";
        case ModeChoice::MOVING_BASE:
            return "moving-base";
    }
    return "kinematic";
}

ReplayConfig parseArguments(int argc, char** argv) {
    ReplayConfig config;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            std::exit(0);
        } else if (arg == "--rover-rinex" && i + 1 < argc) {
            config.rover_rinex_path = argv[++i];
        } else if (arg == "--rover-ubx" && i + 1 < argc) {
            config.rover_ubx_path = argv[++i];
        } else if (arg == "--base-rinex" && i + 1 < argc) {
            config.base_rinex_path = argv[++i];
        } else if (arg == "--base-ubx" && i + 1 < argc) {
            config.base_ubx_path = argv[++i];
        } else if (arg == "--base-rtcm" && i + 1 < argc) {
            config.base_rtcm_path = argv[++i];
        } else if (arg == "--nav-rinex" && i + 1 < argc) {
            config.nav_rinex_path = argv[++i];
        } else if (arg == "--out" && i + 1 < argc) {
            config.output_pos_path = argv[++i];
        } else if (arg == "--format" && i + 1 < argc) {
            config.output_format = parseOutputFormat(argv[++i], argv[0]);
        } else if (arg == "--pos-uncertainty") {
            config.pos_uncertainty = true;
        } else if (arg == "--pos-fix-diagnostics") {
            config.pos_fix_diagnostics = true;
        } else if (arg == "--mode" && i + 1 < argc) {
            config.mode = parseMode(argv[++i], argv[0]);
        } else if (arg == "--ratio" && i + 1 < argc) {
            config.ratio_threshold = std::stod(argv[++i]);
            config.ratio_threshold_set = true;
        } else if (arg == "--preset" && i + 1 < argc) {
            config.preset = parseRTKTuningPreset(argv[++i], argv[0]);
        } else if (arg == "--arfilter") {
            config.enable_ar_filter = true;
            config.has_ar_filter_override = true;
        } else if (arg == "--no-arfilter") {
            config.enable_ar_filter = false;
            config.has_ar_filter_override = true;
        } else if (arg == "--arfilter-margin" && i + 1 < argc) {
            config.ar_filter_margin = std::stod(argv[++i]);
            config.ar_filter_margin_set = true;
        } else if (arg == "--min-ar-sats" && i + 1 < argc) {
            config.min_satellites_for_ar = std::stoi(argv[++i]);
            config.min_satellites_for_ar_set = true;
        } else if (arg == "--min-hold-count" && i + 1 < argc) {
            config.min_hold_count = std::stoi(argv[++i]);
            config.min_hold_count_set = true;
        } else if (arg == "--hold-ratio-threshold" && i + 1 < argc) {
            config.hold_ratio_threshold = std::stod(argv[++i]);
            config.hold_ratio_threshold_set = true;
        } else if (arg == "--hold-fix-unchanged-set") {
            config.hold_fix_unchanged_set = true;
        } else if (arg == "--hold-fix-min-matched-pairs" && i + 1 < argc) {
            config.hold_fix_min_matched_pairs = std::stoi(argv[++i]);
        } else if (arg == "--min-fix-satellites" && i + 1 < argc) {
            config.min_fix_satellites = std::stoi(argv[++i]);
        } else if (arg == "--min-full-ratio-for-subset-ar" && i + 1 < argc) {
            config.min_full_ratio_for_subset_ar = std::stod(argv[++i]);
            config.min_full_ratio_for_subset_ar_set = true;
        } else if (arg == "--max-fixed-anchor-age" && i + 1 < argc) {
            config.max_fixed_anchor_age_s = std::stod(argv[++i]);
            config.max_fixed_anchor_age_s_set = true;
        } else if (arg == "--max-fixed-doppler-consensus" && i + 1 < argc) {
            config.max_fixed_doppler_consensus_m = std::stod(argv[++i]);
            config.max_fixed_doppler_consensus_m_set = true;
        } else if (arg == "--max-fixed-doppler-consensus-age" && i + 1 < argc) {
            config.max_fixed_doppler_consensus_age_s = std::stod(argv[++i]);
            config.max_fixed_doppler_consensus_age_s_set = true;
        } else if (arg == "--low-ratio-guard-threshold" && i + 1 < argc) {
            config.low_ratio_guard_threshold = std::stod(argv[++i]);
            config.low_ratio_guard_threshold_set = true;
        } else if (arg == "--low-ratio-min-fixed-ambiguities" && i + 1 < argc) {
            config.low_ratio_min_fixed_ambiguities = std::stoi(argv[++i]);
            config.low_ratio_min_fixed_ambiguities_set = true;
        } else if (arg == "--fix-history-motion-compensation") {
            config.fix_history_motion_compensation = true;
        } else if (arg == "--max-position-jump-motion-compensation") {
            config.max_position_jump_motion_compensation = true;
        } else if (arg == "--max-pos-jump-min" && i + 1 < argc) {
            config.max_position_jump_min_m = std::stod(argv[++i]);
            config.max_position_jump_min_m_set = true;
        } else if (arg == "--max-pos-jump-rate" && i + 1 < argc) {
            config.max_position_jump_rate_mps = std::stod(argv[++i]);
            config.max_position_jump_rate_mps_set = true;
        } else if (arg == "--max-float-prefit-rms" && i + 1 < argc) {
            config.max_float_prefit_residual_rms_m = std::stod(argv[++i]);
            config.max_float_prefit_residual_rms_m_set = true;
        } else if (arg == "--max-float-prefit-max" && i + 1 < argc) {
            config.max_float_prefit_residual_max_m = std::stod(argv[++i]);
            config.max_float_prefit_residual_max_m_set = true;
        } else if (arg == "--max-float-prefit-reset-streak" && i + 1 < argc) {
            config.max_float_prefit_residual_reset_streak = std::stoi(argv[++i]);
            config.max_float_prefit_residual_reset_streak_set = true;
        } else if (arg == "--elevation-mask-deg" && i + 1 < argc) {
            config.elevation_mask_deg = std::stod(argv[++i]);
        } else if (arg == "--no-glonass") {
            config.enable_glonass = false;
        } else if (arg == "--no-beidou") {
            config.enable_beidou = false;
        } else if (arg == "--glonass-ar" && i + 1 < argc) {
            config.glonass_ar = parseGlonassARChoice(argv[++i], argv[0]);
        } else if (arg == "--max-epochs" && i + 1 < argc) {
            config.max_epochs = std::stoi(argv[++i]);
        } else if (arg == "--no-rtcm-epoch-assembly") {
            config.assemble_rtcm_epochs = false;
        } else if (arg == "--rtcm-message-limit" && i + 1 < argc) {
            config.rtcm_message_limit = static_cast<size_t>(std::stoull(argv[++i]));
        } else if (arg == "--base-ecef" && i + 3 < argc) {
            // Do not increment i multiple times in constructor arguments: C++
            // does not guarantee their evaluation order, and MSVC builds read
            // the supplied XYZ reversed.
            const double base_x = std::stod(argv[++i]);
            const double base_y = std::stod(argv[++i]);
            const double base_z = std::stod(argv[++i]);
            config.base_position_ecef = Eigen::Vector3d(base_x, base_y, base_z);
            config.base_position_override = true;
        } else if (arg == "--quiet") {
            config.quiet = true;
        } else if (arg == "--verbose") {
            config.verbose = true;
        } else {
            argumentError("unknown or incomplete argument: " + arg, argv[0]);
        }
    }

    applyRTKTuningPreset(config);

    const bool has_rover_rinex = !config.rover_rinex_path.empty();
    const bool has_rover_ubx = !config.rover_ubx_path.empty();
    const bool has_base_rinex = !config.base_rinex_path.empty();
    const bool has_base_ubx = !config.base_ubx_path.empty();
    const bool has_base_rtcm = !config.base_rtcm_path.empty();
    if (has_rover_rinex == has_rover_ubx) {
        argumentError("choose exactly one of --rover-rinex or --rover-ubx", argv[0]);
    }
    if (static_cast<int>(has_base_rinex) + static_cast<int>(has_base_ubx) + static_cast<int>(has_base_rtcm) != 1) {
        argumentError("choose exactly one of --base-rinex, --base-ubx, or --base-rtcm", argv[0]);
    }
    if (!has_base_rtcm && config.nav_rinex_path.empty()) {
        argumentError("--nav-rinex is required when base input is not RTCM", argv[0]);
    }
    if (config.max_epochs == 0) {
        argumentError("--max-epochs must be != 0", argv[0]);
    }
    if (config.min_satellites_for_ar < 4) {
        argumentError("--min-ar-sats must be >= 4", argv[0]);
    }
    if (config.ar_filter_margin < 0.0) {
        argumentError("--arfilter-margin must be >= 0", argv[0]);
    }
    if (config.min_hold_count < 0) {
        argumentError("--min-hold-count must be >= 0", argv[0]);
    }
    if (config.hold_ratio_threshold <= 0.0) {
        argumentError("--hold-ratio-threshold must be > 0", argv[0]);
    }
    if (config.hold_fix_min_matched_pairs < 4) {
        argumentError("--hold-fix-min-matched-pairs must be >= 4", argv[0]);
    }
    if (config.min_fix_satellites < 0) {
        argumentError("--min-fix-satellites must be >= 0", argv[0]);
    }
    if (config.max_position_jump_min_m < 0.0) {
        argumentError("--max-pos-jump-min must be >= 0", argv[0]);
    }
    if (config.max_position_jump_rate_mps < 0.0) {
        argumentError("--max-pos-jump-rate must be >= 0", argv[0]);
    }
    if (config.max_fixed_anchor_age_s < 0.0) {
        argumentError("--max-fixed-anchor-age must be >= 0", argv[0]);
    }
    if (config.max_fixed_doppler_consensus_m < 0.0) {
        argumentError("--max-fixed-doppler-consensus must be >= 0", argv[0]);
    }
    if (config.max_fixed_doppler_consensus_age_s < 0.0) {
        argumentError("--max-fixed-doppler-consensus-age must be >= 0", argv[0]);
    }
    if (config.low_ratio_guard_threshold < 0.0) {
        argumentError("--low-ratio-guard-threshold must be >= 0", argv[0]);
    }
    if (config.low_ratio_min_fixed_ambiguities < 0) {
        argumentError("--low-ratio-min-fixed-ambiguities must be >= 0", argv[0]);
    }
    if (config.max_float_prefit_residual_rms_m < 0.0) {
        argumentError("--max-float-prefit-rms must be >= 0", argv[0]);
    }
    if (config.max_float_prefit_residual_max_m < 0.0) {
        argumentError("--max-float-prefit-max must be >= 0", argv[0]);
    }
    if (config.max_float_prefit_residual_reset_streak < 1) {
        argumentError("--max-float-prefit-reset-streak must be >= 1", argv[0]);
    }
    if (config.pos_uncertainty &&
        config.output_format != libgnss::io::SolutionWriter::Format::POS) {
        argumentError("--pos-uncertainty requires --format pos", argv[0]);
    }
    if (config.pos_fix_diagnostics &&
        config.output_format != libgnss::io::SolutionWriter::Format::POS) {
        argumentError("--pos-fix-diagnostics requires --format pos", argv[0]);
    }
    if (!config.base_rtcm_path.empty() &&
        config.base_rtcm_path.rfind("ntrip://", 0) == 0 &&
        config.rtcm_message_limit == 0) {
        argumentError("use --rtcm-message-limit with NTRIP replay sources to bound ingest", argv[0]);
    }
    return config;
}

bool loadRoverRinex(const std::string& path,
                    int max_epochs,
                    std::vector<libgnss::ObservationData>& epochs,
                    libgnss::io::RINEXReader::RINEXHeader& header) {
    libgnss::io::RINEXReader reader;
    if (!reader.open(path) || !reader.readHeader(header)) {
        return false;
    }

    libgnss::ObservationData epoch;
    while ((max_epochs < 0 || static_cast<int>(epochs.size()) < max_epochs) &&
           reader.readObservationEpoch(epoch)) {
        if (header.approximate_position.norm() > 0.0) {
            epoch.receiver_position = header.approximate_position;
        }
        epochs.push_back(epoch);
    }
    reader.close();
    return !epochs.empty();
}

bool loadRoverUbx(const std::string& path,
                  int max_epochs,
                  std::vector<libgnss::ObservationData>& epochs) {
    std::ifstream input(path, std::ios::binary);
    if (!input.is_open()) {
        return false;
    }
    const std::vector<uint8_t> buffer((std::istreambuf_iterator<char>(input)),
                                      std::istreambuf_iterator<char>());
    input.close();

    libgnss::io::UBXDecoder decoder;
    const auto messages = decoder.decode(buffer.data(), buffer.size());
    for (const auto& message : messages) {
        if (max_epochs >= 0 && static_cast<int>(epochs.size()) >= max_epochs) {
            break;
        }
        libgnss::io::UBXNavPVT nav_pvt;
        (void)decoder.decodeNavPVT(message, nav_pvt);
        libgnss::ObservationData obs_data;
        if (decoder.decodeRawx(message, obs_data)) {
            epochs.push_back(obs_data);
        }
    }
    return !epochs.empty();
}

bool loadBaseRinex(const std::string& obs_path,
                   const std::string& nav_path,
                   std::vector<libgnss::ObservationData>& base_epochs,
                   libgnss::NavigationData& nav_data,
                   Eigen::Vector3d& base_position,
                   bool& have_base_position) {
    libgnss::io::RINEXReader base_reader;
    libgnss::io::RINEXReader nav_reader;
    libgnss::io::RINEXReader::RINEXHeader base_header;

    if (!base_reader.open(obs_path) || !base_reader.readHeader(base_header)) {
        return false;
    }
    if (!nav_reader.open(nav_path) || !nav_reader.readNavigationData(nav_data)) {
        return false;
    }

    libgnss::ObservationData epoch;
    while (base_reader.readObservationEpoch(epoch)) {
        base_epochs.push_back(epoch);
    }
    base_reader.close();
    nav_reader.close();

    if (base_header.approximate_position.norm() > 0.0) {
        base_position = base_header.approximate_position;
        have_base_position = true;
    }
    return !base_epochs.empty();
}

bool loadNavigationRinex(const std::string& path, libgnss::NavigationData& nav_data);

bool loadBaseUbx(const std::string& ubx_path,
                 const std::string& nav_path,
                 std::vector<libgnss::ObservationData>& base_epochs,
                 libgnss::NavigationData& nav_data,
                 Eigen::Vector3d& base_position,
                 bool& have_base_position) {
    if (!loadNavigationRinex(nav_path, nav_data)) {
        return false;
    }

    std::ifstream input(ubx_path, std::ios::binary);
    if (!input.is_open()) {
        return false;
    }
    const std::vector<uint8_t> buffer((std::istreambuf_iterator<char>(input)),
                                      std::istreambuf_iterator<char>());
    input.close();

    libgnss::io::UBXDecoder decoder;
    const auto messages = decoder.decode(buffer.data(), buffer.size());
    for (const auto& message : messages) {
        libgnss::io::UBXNavPVT nav_pvt;
        (void)decoder.decodeNavPVT(message, nav_pvt);
        libgnss::ObservationData obs_data;
        if (decoder.decodeRawx(message, obs_data)) {
            if (!have_base_position && obs_data.receiver_position.norm() > 1e6) {
                base_position = obs_data.receiver_position;
                have_base_position = true;
            }
            base_epochs.push_back(obs_data);
        }
    }
    return !base_epochs.empty();
}

// Decodes the base RTCM stream with RTCMStreamDecoder so TOW-only MSM parts get a full GPST week
// (and GLONASS epochs are dated from the stream, not the host clock), exactly as in gnss_live.
// `seed` is the latest GPS toe of the --nav-rinex data when one was given; otherwise the decoder
// seeds itself from the first in-stream GPS/Galileo/BeiDou/QZSS ephemeris.
bool loadBaseRtcm(const std::string& source,
                  size_t message_limit,
                  bool assemble_epochs,
                  const std::optional<libgnss::GNSSTime>& seed,
                  std::vector<libgnss::ObservationData>& base_epochs,
                  libgnss::NavigationData& nav_data,
                  Eigen::Vector3d& base_position,
                  bool& have_base_position) {
    libgnss::io::RTCMReader reader;
    if (!reader.open(source)) {
        return false;
    }

    libgnss::io::RTCMStreamDecoder decoder;
    if (seed) {
        decoder.setReferenceTime(*seed);
    }
    libgnss::io::ObservationEpochAssembler assembler;
    // First reference the decoder had; epochs decoded before it existed (head of a stream with
    // neither a navigation RINEX nor an early ephemeris) are still TOW-only (week 0) and are
    // resolved against it after the pass, with the same nearest-week rule.
    std::optional<libgnss::GNSSTime> first_reference = decoder.referenceTime();
    size_t message_count = 0;
    libgnss::io::RTCMMessage raw_message;
    std::vector<libgnss::io::RTCMStreamDecoder::Event> events;
    while ((message_limit == 0 || message_count < message_limit) && reader.readMessage(raw_message)) {
        ++message_count;
        const auto frame = buildRtcmFrame(raw_message);
        decoder.pushFrame(frame.data(), frame.size(), events);
        for (auto& event : events) {
            if (event.has_observation) {
                if (assemble_epochs) {
                    for (auto& epoch : assembler.push(event.observation, event.observation_more_follows)) {
                        base_epochs.push_back(std::move(epoch));
                    }
                } else {
                    base_epochs.push_back(event.observation);
                }
            } else if (event.has_navigation) {
                mergeNavigationData(nav_data, event.navigation);
            }
        }
        if (!first_reference) {
            first_reference = decoder.referenceTime();
        }
    }
    // A pending (partially assembled) epoch at end of stream is still an epoch.
    for (auto& epoch : assembler.flush()) {
        base_epochs.push_back(std::move(epoch));
    }
    if (first_reference) {
        for (auto& epoch : base_epochs) {
            if (epoch.time.week == 0) {
                epoch.time = libgnss::io::RTCMStreamDecoder::resolveTowOnlyTime(epoch.time, *first_reference);
            }
        }
    }
    if (decoder.hasStationPosition()) {
        base_position = decoder.getStationPosition();
        have_base_position = true;
    }
    reader.close();
    return !base_epochs.empty();
}

bool loadNavigationRinex(const std::string& path, libgnss::NavigationData& nav_data) {
    libgnss::io::RINEXReader reader;
    if (!reader.open(path)) {
        return false;
    }
    libgnss::NavigationData loaded;
    if (!reader.readNavigationData(loaded)) {
        reader.close();
        return false;
    }
    reader.close();
    mergeNavigationData(nav_data, loaded);
    return true;
}

size_t runReplay(const ReplayConfig& config) {
    std::vector<libgnss::ObservationData> rover_epochs;
    std::vector<libgnss::ObservationData> base_epochs;
    libgnss::NavigationData nav_data;
    Eigen::Vector3d base_position = Eigen::Vector3d::Zero();
    bool have_base_position = false;
    libgnss::io::RINEXReader::RINEXHeader rover_header;

    if (!config.rover_rinex_path.empty()) {
        if (!loadRoverRinex(config.rover_rinex_path, config.max_epochs, rover_epochs, rover_header)) {
            throw std::runtime_error("failed to load rover RINEX observations");
        }
    } else if (!loadRoverUbx(config.rover_ubx_path, config.max_epochs, rover_epochs)) {
        throw std::runtime_error("failed to load rover UBX observations");
    }

    if (!config.base_rinex_path.empty()) {
        if (!loadBaseRinex(config.base_rinex_path,
                           config.nav_rinex_path,
                           base_epochs,
                           nav_data,
                           base_position,
                           have_base_position)) {
            throw std::runtime_error("failed to load base RINEX observations/navigation");
        }
    } else if (!config.base_ubx_path.empty()) {
        if (!loadBaseUbx(config.base_ubx_path,
                         config.nav_rinex_path,
                         base_epochs,
                         nav_data,
                         base_position,
                         have_base_position)) {
            throw std::runtime_error("failed to load base UBX observations/navigation");
        }
    } else {
        // The navigation RINEX is read first (into its own container) only to seed the stream's
        // GPST reference; it is merged after the in-stream ephemerides, as before.
        libgnss::NavigationData supplemental_nav;
        std::optional<libgnss::GNSSTime> stream_reference_seed;
        if (!config.nav_rinex_path.empty()) {
            if (!loadNavigationRinex(config.nav_rinex_path, supplemental_nav)) {
                throw std::runtime_error("failed to load supplemental navigation RINEX");
            }
            stream_reference_seed = libgnss::io::RTCMStreamDecoder::referenceFromNavigation(supplemental_nav);
        }
        if (!loadBaseRtcm(config.base_rtcm_path,
                          config.rtcm_message_limit,
                          config.assemble_rtcm_epochs,
                          stream_reference_seed,
                          base_epochs,
                          nav_data,
                          base_position,
                          have_base_position)) {
            throw std::runtime_error("failed to load base RTCM observations");
        }
        mergeNavigationData(nav_data, supplemental_nav);
    }

    if (nav_data.ephemeris_data.empty()) {
        throw std::runtime_error("no navigation data available");
    }

    if (config.base_position_override) {
        base_position = config.base_position_ecef;
        have_base_position = true;
    }
    if (!have_base_position) {
        throw std::runtime_error("base position unavailable; use --base-ecef or a source with base coordinates");
    }

    libgnss::RTKProcessor rtk;
    libgnss::RTKProcessor::RTKConfig rtk_config;
    rtk_config.position_mode =
        config.mode == ModeChoice::STATIC
            ? libgnss::RTKProcessor::RTKConfig::PositionMode::STATIC
            : (config.mode == ModeChoice::MOVING_BASE
                   ? libgnss::RTKProcessor::RTKConfig::PositionMode::MOVING_BASE
                   : libgnss::RTKProcessor::RTKConfig::PositionMode::KINEMATIC);
    rtk_config.ratio_threshold = config.ratio_threshold;
    rtk_config.ambiguity_ratio_threshold = config.ratio_threshold;
    rtk_config.hold_ambiguity_ratio_threshold = config.hold_ratio_threshold;
    rtk_config.enable_ar_filter = config.enable_ar_filter;
    rtk_config.ar_filter_margin = config.ar_filter_margin;
    rtk_config.min_satellites_for_ar = config.min_satellites_for_ar;
    rtk_config.min_hold_count = config.min_hold_count;
    rtk_config.hold_fix_unchanged_set_only = config.hold_fix_unchanged_set;
    rtk_config.hold_fix_min_matched_pairs = config.hold_fix_min_matched_pairs;
    rtk_config.min_fix_satellites = config.min_fix_satellites;
    rtk_config.min_full_ratio_for_subset_ar = config.min_full_ratio_for_subset_ar;
    rtk_config.max_fixed_anchor_age_s = config.max_fixed_anchor_age_s;
    rtk_config.max_fixed_doppler_consensus_m = config.max_fixed_doppler_consensus_m;
    rtk_config.max_fixed_doppler_consensus_age_s = config.max_fixed_doppler_consensus_age_s;
    rtk_config.low_ratio_guard_threshold = config.low_ratio_guard_threshold;
    rtk_config.low_ratio_min_fixed_ambiguities = config.low_ratio_min_fixed_ambiguities;
    rtk_config.fix_history_motion_compensation = config.fix_history_motion_compensation;
    rtk_config.max_position_jump_motion_compensation =
        config.max_position_jump_motion_compensation;
    rtk_config.max_position_jump_min_m = config.max_position_jump_min_m;
    rtk_config.max_position_jump_rate_mps = config.max_position_jump_rate_mps;
    rtk_config.max_float_prefit_residual_rms_m = config.max_float_prefit_residual_rms_m;
    rtk_config.max_float_prefit_residual_max_m = config.max_float_prefit_residual_max_m;
    rtk_config.max_float_prefit_residual_reset_streak =
        config.max_float_prefit_residual_reset_streak;
    rtk_config.elevation_mask = config.elevation_mask_deg * M_PI / 180.0;
    rtk_config.enable_glonass = config.enable_glonass;
    rtk_config.enable_beidou = config.enable_beidou;
    rtk_config.glonass_ar_mode =
        config.glonass_ar == GlonassARChoice::AUTOCAL
            ? libgnss::RTKProcessor::RTKConfig::GlonassARMode::AUTOCAL
            : (config.glonass_ar == GlonassARChoice::ON
                ? libgnss::RTKProcessor::RTKConfig::GlonassARMode::ON
                : libgnss::RTKProcessor::RTKConfig::GlonassARMode::OFF);
    rtk.setRTKConfig(rtk_config);
    rtk.setBasePosition(base_position);

    libgnss::io::SolutionWriter writer;
    if (!writer.open(config.output_pos_path, config.output_format, config.pos_uncertainty,
                     config.pos_fix_diagnostics)) {
        throw std::runtime_error("failed to open output file: " + config.output_pos_path);
    }

    size_t aligned_epochs = 0;
    size_t written_solutions = 0;
    size_t fixed_solutions = 0;
    size_t skipped_rover_epochs = 0;
    size_t base_index = 0;
    Eigen::Vector3d rover_seed = Eigen::Vector3d::Zero();
    if (!rover_epochs.empty() && rover_epochs.front().receiver_position.norm() > 0.0) {
        rover_seed = rover_epochs.front().receiver_position;
    } else if (rover_header.approximate_position.norm() > 0.0) {
        rover_seed = rover_header.approximate_position;
    } else {
        rover_seed = base_position + Eigen::Vector3d(3000.0, 0.0, 0.0);
    }

    for (size_t rover_index = 0; rover_index < rover_epochs.size(); ++rover_index) {
        auto rover_obs = rover_epochs[rover_index];
        if (rover_obs.receiver_position.norm() == 0.0) {
            rover_obs.receiver_position = rover_seed;
        } else {
            rover_seed = rover_obs.receiver_position;
        }

        while (base_index < base_epochs.size() &&
               timeDiffSeconds(base_epochs[base_index].time, rover_obs.time) < -kExactTimeToleranceSeconds) {
            ++base_index;
        }

        if (base_index >= base_epochs.size()) {
            break;
        }

        const double dt = std::abs(timeDiffSeconds(base_epochs[base_index].time, rover_obs.time));
        if (dt > kExactTimeToleranceSeconds) {
            ++skipped_rover_epochs;
            continue;
        }

        const auto solution = rtk.processRTKEpoch(rover_obs, base_epochs[base_index], nav_data);
        ++aligned_epochs;

        if (!solution.isValid()) {
            continue;
        }

        // Replay only uses exactly aligned base epochs.
        writer.writeEpoch(solution, timeDiffSeconds(rover_obs.time, base_epochs[base_index].time));
        ++written_solutions;
        if (solution.isFixed()) {
            ++fixed_solutions;
        }

        if (config.verbose && !config.quiet) {
            std::cout << "epoch " << aligned_epochs
                      << " tow=" << std::fixed << std::setprecision(3) << solution.time.tow
                      << " status=" << static_cast<int>(solution.status)
                      << " sats=" << solution.num_satellites
                      << " ratio=" << std::setprecision(2) << solution.ratio << "\n";
        }
    }

    writer.close();

    if (!config.quiet) {
        std::cout << "summary: rover_epochs=" << rover_epochs.size()
                  << " base_epochs=" << base_epochs.size()
                  << " mode=" << modeChoiceString(config.mode)
                  << " aligned_epochs=" << aligned_epochs
                  << " skipped_rover_epochs=" << skipped_rover_epochs
                  << " written_solutions=" << written_solutions
                  << " fixed_solutions=" << fixed_solutions
                  << " out=" << config.output_pos_path
                  << " format=" << outputFormatString(config.output_format)
                  << "\n";
    } else {
        std::cout << "summary: aligned_epochs=" << aligned_epochs
                  << " mode=" << modeChoiceString(config.mode)
                  << " written_solutions=" << written_solutions
                  << " fixed_solutions=" << fixed_solutions << "\n";
    }
    return written_solutions;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const ReplayConfig config = parseArguments(argc, argv);
        const size_t written_solutions = runReplay(config);
        return written_solutions == 0 ? 1 : 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
