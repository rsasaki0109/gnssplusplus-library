#include <Eigen/Dense>
#include <cmath>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include <libgnss++/algorithms/rtk.hpp>
#include <libgnss++/algorithms/rtk_validation.hpp>
#include <libgnss++/algorithms/spp.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/gnss.hpp>
#include <libgnss++/io/rinex.hpp>
#include <libgnss++/io/solution_writer.hpp>
#include <libgnss++/models/troposphere.hpp>

namespace {

constexpr double kExactTimeToleranceSeconds = 1e-6;
constexpr double kDefaultFloatVsSppGuardMeters = 30.0;
constexpr double kDefaultNonFixedJumpGuardMeters = 8.0;
constexpr double kDefaultVerticalStepGuardMeters = 1.0;
constexpr double kDefaultNonFixDriftGuardMaxAnchorGapSeconds = 120.0;
constexpr double kDefaultNonFixDriftGuardMaxAnchorSpeedMps = 1.0;
constexpr double kDefaultNonFixDriftGuardMaxResidualMeters = 30.0;
constexpr double kDefaultNonFixDriftGuardMinHorizontalResidualMeters = 0.0;
constexpr int kDefaultNonFixDriftGuardMinSegmentEpochs = 20;
constexpr int kDefaultNonFixDriftGuardMaxSegmentEpochs = 0;
constexpr double kDefaultSppHeightStepGuardMinMeters = 30.0;
constexpr double kDefaultSppHeightStepGuardMaxRateMps = 4.0;
constexpr double kDefaultFloatBridgeTailGuardMaxAnchorGapSeconds = 120.0;
constexpr double kDefaultFloatBridgeTailGuardMinAnchorSpeedMps = 0.4;
constexpr double kDefaultFloatBridgeTailGuardMaxAnchorSpeedMps = 1.0;
constexpr double kDefaultFloatBridgeTailGuardMaxResidualMeters = 12.0;
constexpr int kDefaultFloatBridgeTailGuardMinSegmentEpochs = 20;
constexpr double kDefaultFixedBridgeBurstGuardMaxAnchorGapSeconds = 30.0;
constexpr double kDefaultFixedBridgeBurstGuardMinBoundaryGapSeconds = 1.0;
constexpr double kDefaultFixedBridgeBurstGuardMaxResidualMeters = 20.0;
constexpr int kDefaultFixedBridgeBurstGuardMaxSegmentEpochs = 12;

enum class ModeChoice {
    AUTO,
    KINEMATIC,
    STATIC,
    MOVING_BASE
};

enum class IonoChoice {
    AUTO,
    OFF,
    IFLC,
    EST
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
    ODAIBA
};

struct SolveConfig {
    std::string data_dir;
    std::string rover_obs_path;
    std::string base_obs_path;
    std::string nav_path;
    std::string output_pos_path = "output/rtk_solution.pos";
    std::string output_kml_path = "output/rtk_solution.kml";
    bool write_kml = true;
    bool enable_base_interpolation = true;
    bool verbose = false;
    bool base_position_override = false;
    Eigen::Vector3d base_position_ecef = Eigen::Vector3d::Zero();
    ModeChoice mode = ModeChoice::AUTO;
    IonoChoice iono = IonoChoice::AUTO;
    libgnss::io::SolutionWriter::Format output_format = libgnss::io::SolutionWriter::Format::POS;
    double max_baseline_length_m = 20000.0;
    double ratio_threshold = 3.0;
    bool enable_ar_filter = false;
    bool has_ar_filter_override = false;
    double ar_filter_margin = 0.25;
    int min_satellites_for_ar = 5;
    int min_subset_pairs_for_ar = 4;
    int min_subset_sats_for_ar = 0;
    int min_subset_systems_for_ar = 0;
    int min_subset_frequencies_for_ar = 0;
    int min_subset_dual_frequency_sats_for_ar = 0;
    double min_full_ratio_for_subset_ar = 0.0;
    int min_hold_count = 5;
    double hold_ratio_threshold = 2.0;
    double elevation_mask_deg = 15.0;
    double process_noise_position = 1e-4;
    double process_noise_ambiguity = 1e-8;
    double process_noise_iono = 1e-4;
    double carrier_phase_sigma = 0.002;
    bool process_noise_position_set = false;
    bool process_noise_ambiguity_set = false;
    bool process_noise_iono_set = false;
    bool carrier_phase_sigma_set = false;
    bool enable_glonass = true;
    bool enable_beidou = true;
    GlonassARChoice glonass_ar = GlonassARChoice::OFF;
    double glonass_icb_l1_m_per_mhz = 0.0;
    double glonass_icb_l2_m_per_mhz = 0.0;
    int skip_epochs = 0;
    int max_epochs = -1;
    std::string debug_epoch_log_path;
    bool enable_kinematic_post_filter = true;
    bool enable_nonfix_drift_guard = true;
    double nonfix_drift_guard_max_anchor_gap_s = kDefaultNonFixDriftGuardMaxAnchorGapSeconds;
    double nonfix_drift_guard_max_anchor_speed_mps = kDefaultNonFixDriftGuardMaxAnchorSpeedMps;
    double nonfix_drift_guard_max_residual_m = kDefaultNonFixDriftGuardMaxResidualMeters;
    double nonfix_drift_guard_min_horizontal_residual_m =
        kDefaultNonFixDriftGuardMinHorizontalResidualMeters;
    int nonfix_drift_guard_min_segment_epochs = kDefaultNonFixDriftGuardMinSegmentEpochs;
    int nonfix_drift_guard_max_segment_epochs = kDefaultNonFixDriftGuardMaxSegmentEpochs;
    bool enable_spp_height_step_guard = true;
    double spp_height_step_guard_min_m = kDefaultSppHeightStepGuardMinMeters;
    double spp_height_step_guard_max_rate_mps = kDefaultSppHeightStepGuardMaxRateMps;
    bool enable_float_bridge_tail_guard = true;
    double float_bridge_tail_guard_max_anchor_gap_s = kDefaultFloatBridgeTailGuardMaxAnchorGapSeconds;
    double float_bridge_tail_guard_min_anchor_speed_mps = kDefaultFloatBridgeTailGuardMinAnchorSpeedMps;
    double float_bridge_tail_guard_max_anchor_speed_mps = kDefaultFloatBridgeTailGuardMaxAnchorSpeedMps;
    double float_bridge_tail_guard_max_residual_m = kDefaultFloatBridgeTailGuardMaxResidualMeters;
    int float_bridge_tail_guard_min_segment_epochs = kDefaultFloatBridgeTailGuardMinSegmentEpochs;
    bool enable_fixed_bridge_burst_guard = false;
    double fixed_bridge_burst_guard_max_anchor_gap_s = kDefaultFixedBridgeBurstGuardMaxAnchorGapSeconds;
    double fixed_bridge_burst_guard_min_boundary_gap_s = kDefaultFixedBridgeBurstGuardMinBoundaryGapSeconds;
    double fixed_bridge_burst_guard_max_residual_m = kDefaultFixedBridgeBurstGuardMaxResidualMeters;
    int fixed_bridge_burst_guard_max_segment_epochs = kDefaultFixedBridgeBurstGuardMaxSegmentEpochs;
    RTKTuningPreset preset = RTKTuningPreset::NONE;
    bool ratio_threshold_set = false;
    bool ar_filter_margin_set = false;
    bool min_satellites_for_ar_set = false;
    bool min_subset_sats_for_ar_set = false;
    bool min_subset_systems_for_ar_set = false;
    bool min_subset_frequencies_for_ar_set = false;
    bool min_subset_dual_frequency_sats_for_ar_set = false;
    bool min_hold_count_set = false;
    bool hold_ratio_threshold_set = false;
    libgnss::RTKProcessor::RTKConfig::ARPolicy ar_policy =
        libgnss::RTKProcessor::RTKConfig::ARPolicy::EXTENDED;
    double max_hold_divergence_m = 0.0;
    double max_position_jump_m = 5.0;
    double max_position_jump_min_m = 0.0;
    double max_position_jump_rate_mps = 0.0;
    double max_float_spp_divergence_m = 0.0;
    double max_float_prefit_residual_rms_m = 0.0;
    double max_float_prefit_residual_max_m = 0.0;
    int max_float_prefit_residual_reset_streak = 3;
    double min_float_prefit_residual_trusted_jump_m = 0.0;
    double max_update_nis_per_observation = 0.0;
    int max_consecutive_float_for_reset = 0;
    int max_consecutive_nonfix_for_reset = 0;
    double max_postfix_residual_rms = 0.0;
    bool enable_wide_lane_ar = false;
    bool wide_lane_ar_set = false;
    double wide_lane_acceptance_threshold = 0.25;
    bool wide_lane_acceptance_threshold_set = false;
};

double timeDiffSeconds(const libgnss::GNSSTime& a, const libgnss::GNSSTime& b) {
    return a - b;
}

struct ObservationKey {
    libgnss::SatelliteId satellite;
    libgnss::SignalType signal;

    bool operator<(const ObservationKey& other) const {
        if (satellite < other.satellite) return true;
        if (other.satellite < satellite) return false;
        return signal < other.signal;
    }
};

std::map<ObservationKey, const libgnss::Observation*> indexObservations(
    const libgnss::ObservationData& epoch) {
    std::map<ObservationKey, const libgnss::Observation*> indexed;
    for (const auto& obs : epoch.observations) {
        indexed[{obs.satellite, obs.signal}] = &obs;
    }
    return indexed;
}

double signalFrequencyHz(const libgnss::SatelliteId& satellite,
                         libgnss::SignalType signal,
                         const libgnss::GNSSTime& time,
                         const libgnss::NavigationData& nav) {
    const libgnss::Ephemeris* eph = nav.getEphemeris(satellite, time);
    switch (signal) {
        case libgnss::SignalType::GPS_L1CA:
        case libgnss::SignalType::QZS_L1CA:
            return libgnss::constants::GPS_L1_FREQ;
        case libgnss::SignalType::GPS_L2C:
        case libgnss::SignalType::QZS_L2C:
            return libgnss::constants::GPS_L2_FREQ;
        case libgnss::SignalType::GPS_L5:
        case libgnss::SignalType::QZS_L5:
            return libgnss::constants::GPS_L5_FREQ;
        case libgnss::SignalType::GLO_L1CA:
        case libgnss::SignalType::GLO_L1P:
            if (eph && eph->satellite.system == libgnss::GNSSSystem::GLONASS) {
                return libgnss::constants::GLO_L1_BASE_FREQ +
                    eph->glonass_frequency_channel * libgnss::constants::GLO_L1_STEP_FREQ;
            }
            return libgnss::constants::GLO_L1_BASE_FREQ;
        case libgnss::SignalType::GLO_L2CA:
        case libgnss::SignalType::GLO_L2P:
            if (eph && eph->satellite.system == libgnss::GNSSSystem::GLONASS) {
                return libgnss::constants::GLO_L2_BASE_FREQ +
                    eph->glonass_frequency_channel * libgnss::constants::GLO_L2_STEP_FREQ;
            }
            return libgnss::constants::GLO_L2_BASE_FREQ;
        case libgnss::SignalType::GAL_E1:
            return libgnss::constants::GAL_E1_FREQ;
        case libgnss::SignalType::GAL_E5A:
            return libgnss::constants::GAL_E5A_FREQ;
        case libgnss::SignalType::GAL_E5B:
            return libgnss::constants::GAL_E5B_FREQ;
        case libgnss::SignalType::GAL_E6:
            return libgnss::constants::GAL_E6_FREQ;
        case libgnss::SignalType::BDS_B1I:
            return libgnss::constants::BDS_B1I_FREQ;
        case libgnss::SignalType::BDS_B2I:
            return libgnss::constants::BDS_B2I_FREQ;
        case libgnss::SignalType::BDS_B3I:
            return libgnss::constants::BDS_B3I_FREQ;
        case libgnss::SignalType::BDS_B1C:
            return libgnss::constants::BDS_B1C_FREQ;
        case libgnss::SignalType::BDS_B2A:
            return libgnss::constants::BDS_B2A_FREQ;
        default:
            return 0.0;
    }
}

double signalWavelength(const libgnss::SatelliteId& satellite,
                        libgnss::SignalType signal,
                        const libgnss::GNSSTime& time,
                        const libgnss::NavigationData& nav) {
    const double frequency = signalFrequencyHz(satellite, signal, time, nav);
    if (frequency <= 0.0) {
        return 0.0;
    }
    return libgnss::constants::SPEED_OF_LIGHT / frequency;
}

double signalWavelength(libgnss::SignalType signal) = delete;

bool calculateModeledBaseRange(const libgnss::SatelliteId& satellite,
                               const libgnss::GNSSTime& time,
                               double approx_pseudorange,
                               const libgnss::Vector3d& base_position,
                               const libgnss::NavigationData& nav,
                               double& modeled_range) {
    libgnss::Vector3d sat_pos;
    libgnss::Vector3d sat_vel;
    double sat_clk = 0.0;
    double sat_clk_drift = 0.0;

    const double travel_time = approx_pseudorange > 1.0
        ? approx_pseudorange / libgnss::constants::SPEED_OF_LIGHT
        : 0.075;
    libgnss::GNSSTime tx_time = time - travel_time;
    if (!nav.calculateSatelliteState(satellite, tx_time, sat_pos, sat_vel, sat_clk, sat_clk_drift)) {
        return false;
    }

    tx_time = tx_time - sat_clk;
    if (!nav.calculateSatelliteState(satellite, tx_time, sat_pos, sat_vel, sat_clk, sat_clk_drift)) {
        return false;
    }

    const auto geom = nav.calculateGeometry(base_position, sat_pos);
    if (geom.elevation <= 0.05) {
        return false;
    }

    modeled_range = libgnss::geodist(sat_pos, base_position) +
        libgnss::models::tropDelaySaastamoinen(base_position, geom.elevation);
    return std::isfinite(modeled_range);
}

bool interpolateBaseEpoch(const libgnss::ObservationData& before,
                          const libgnss::ObservationData& after,
                          const libgnss::GNSSTime& target_time,
                          const libgnss::Vector3d& base_position,
                          const libgnss::NavigationData& nav,
                          libgnss::ObservationData& interpolated_epoch) {
    constexpr double kMaxInterpolationGapSeconds = 2.0;
    const double total_dt = timeDiffSeconds(after.time, before.time);
    if (!std::isfinite(total_dt) || total_dt <= 1e-6 || total_dt > kMaxInterpolationGapSeconds) {
        return false;
    }

    const double alpha = timeDiffSeconds(target_time, before.time) / total_dt;
    if (!std::isfinite(alpha) || alpha < -1e-6 || alpha > 1.0 + 1e-6) {
        return false;
    }

    interpolated_epoch = libgnss::ObservationData(target_time);
    interpolated_epoch.receiver_position =
        before.receiver_position.norm() > 0.0 ? before.receiver_position : after.receiver_position;
    interpolated_epoch.receiver_clock_bias =
        (1.0 - alpha) * before.receiver_clock_bias + alpha * after.receiver_clock_bias;

    const auto before_obs = indexObservations(before);
    const auto after_obs = indexObservations(after);
    struct ModeledRangeTriple {
        bool valid = false;
        double before_m = 0.0;
        double after_m = 0.0;
        double target_m = 0.0;
    };
    std::map<libgnss::SatelliteId, ModeledRangeTriple> modeled_range_cache;

    for (const auto& [key, obs_before_ptr] : before_obs) {
        const auto after_it = after_obs.find(key);
        if (after_it == after_obs.end()) continue;

        const auto& obs_before = *obs_before_ptr;
        const auto& obs_after = *after_it->second;
        const double wavelength = signalWavelength(key.satellite, key.signal, target_time, nav);
        if (wavelength <= 0.0 || !obs_before.has_pseudorange || !obs_after.has_pseudorange) {
            continue;
        }

        auto& modeled = modeled_range_cache[key.satellite];
        if (!modeled.valid && modeled.before_m == 0.0 && modeled.after_m == 0.0 &&
            modeled.target_m == 0.0) {
            if (!calculateModeledBaseRange(key.satellite, before.time, obs_before.pseudorange,
                                           base_position, nav, modeled.before_m) ||
                !calculateModeledBaseRange(key.satellite, after.time, obs_after.pseudorange,
                                           base_position, nav, modeled.after_m)) {
                modeled.before_m = std::numeric_limits<double>::quiet_NaN();
                continue;
            }
            modeled.target_m = modeled.before_m + alpha * (modeled.after_m - modeled.before_m);
            modeled.valid = true;
        }
        if (!modeled.valid) {
            continue;
        }

        libgnss::Observation obs(key.satellite, key.signal);
        obs.valid = obs_before.valid && obs_after.valid;
        obs.lli = alpha < 0.5 ? obs_before.lli : obs_after.lli;
        obs.code = alpha < 0.5 ? obs_before.code : obs_after.code;
        obs.signal_strength = std::max(obs_before.signal_strength, obs_after.signal_strength);
        obs.snr = (1.0 - alpha) * obs_before.snr + alpha * obs_after.snr;
        obs.loss_of_lock = obs_before.loss_of_lock || obs_after.loss_of_lock;

        const double code_residual_before = obs_before.pseudorange - modeled.before_m;
        const double code_residual_after = obs_after.pseudorange - modeled.after_m;
        obs.pseudorange = modeled.target_m +
            code_residual_before + alpha * (code_residual_after - code_residual_before);
        obs.has_pseudorange = std::isfinite(obs.pseudorange);

        if (obs_before.has_carrier_phase && obs_after.has_carrier_phase &&
            (obs_before.lli & 0x01) == 0 && (obs_after.lli & 0x01) == 0 && !obs.loss_of_lock) {
            const double phase_residual_before = obs_before.carrier_phase * wavelength - modeled.before_m;
            const double phase_residual_after = obs_after.carrier_phase * wavelength - modeled.after_m;
            const double phase_residual_target =
                phase_residual_before + alpha * (phase_residual_after - phase_residual_before);
            obs.carrier_phase = (modeled.target_m + phase_residual_target) / wavelength;
            obs.has_carrier_phase = std::isfinite(obs.carrier_phase);
        }

        if (obs_before.has_doppler && obs_after.has_doppler) {
            obs.doppler = obs_before.doppler + alpha * (obs_after.doppler - obs_before.doppler);
            obs.has_doppler = true;
        }

        if (obs.has_pseudorange || obs.has_carrier_phase || obs.has_doppler) {
            interpolated_epoch.addObservation(obs);
        }
    }

    return !interpolated_epoch.observations.empty();
}

std::string modeChoiceString(ModeChoice mode) {
    switch (mode) {
        case ModeChoice::AUTO:
            return "auto";
        case ModeChoice::KINEMATIC:
            return "kinematic";
        case ModeChoice::STATIC:
            return "static";
        case ModeChoice::MOVING_BASE:
            return "moving-base";
    }
    return "unknown";
}

std::string ionoChoiceString(IonoChoice iono) {
    switch (iono) {
        case IonoChoice::AUTO:
            return "auto";
        case IonoChoice::OFF:
            return "off";
        case IonoChoice::IFLC:
            return "iflc";
        case IonoChoice::EST:
            return "est";
    }
    return "unknown";
}

std::string glonassARChoiceString(GlonassARChoice choice) {
    switch (choice) {
        case GlonassARChoice::OFF:
            return "off";
        case GlonassARChoice::ON:
            return "on";
        case GlonassARChoice::AUTOCAL:
            return "autocal";
    }
    return "off";
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

class EpochDebugWriter {
public:
    bool open(const std::string& path) {
        if (path.empty()) {
            return true;
        }
        file_.open(path);
        if (!file_.is_open()) {
            return false;
        }
        file_ << "gps_week,tow,status,num_sats,ratio,baseline_m,"
              << "ar_attempted,input_pair_count,pair_count,max_ambiguity_variance,"
              << "effective_ratio_threshold,min_subset_pair_count,min_full_ratio_for_subset_ar,"
              << "subset_candidates_evaluated,subset_candidates_rejected_by_full_ratio,"
              << "subset_candidates_rejected_by_diversity,wide_lane_total,wide_lane_fixed,"
              << "wide_lane_rejected,wide_lane_min_distance,wide_lane_max_distance,"
              << "full_lambda_solved,full_ratio,selected_fixed,selected_ratio,"
              << "selected_pair_count,selected_distinct_sats,selected_distinct_systems,"
              << "selected_distinct_frequencies,selected_dual_frequency_sats,"
              << "selected_fixed_ambiguities,selected_used_subset,"
              << "used_wlnl_fallback,validation_attempted,validation_passed,"
              << "postfix_residual_rms,fixed_float_jump_m,post_validation_rejected,"
              << "final_fixed_applied,reject_reason,ar_skip_reason\n";
        return true;
    }

    void write(const libgnss::PositionSolution& solution,
               const libgnss::RTKProcessor::EpochDebugTelemetry& telemetry) {
        if (!file_.is_open()) {
            return;
        }
        file_ << solution.time.week << ","
              << std::fixed << std::setprecision(3) << solution.time.tow << ","
              << static_cast<int>(solution.status) << ","
              << solution.num_satellites << ",";
        writeNumber(solution.ratio);
        file_ << ",";
        writeNumber(solution.baseline_length);
        file_ << ","
              << telemetry.ar_attempted << ","
              << telemetry.input_pair_count << ","
              << telemetry.pair_count << ",";
        writeNumber(telemetry.max_ambiguity_variance);
        file_ << ",";
        writeNumber(telemetry.effective_ratio_threshold);
        file_ << ","
              << telemetry.min_subset_pair_count << ","
              << telemetry.min_full_ratio_for_subset_ar << ","
              << telemetry.subset_candidates_evaluated << ","
              << telemetry.subset_candidates_rejected_by_full_ratio << ","
              << telemetry.subset_candidates_rejected_by_diversity << ","
              << telemetry.wide_lane_total << ","
              << telemetry.wide_lane_fixed << ","
              << telemetry.wide_lane_rejected << ",";
        writeNumber(telemetry.wide_lane_min_distance);
        file_ << ",";
        writeNumber(telemetry.wide_lane_max_distance);
        file_ << ","
              << telemetry.full_lambda_solved << ",";
        writeNumber(telemetry.full_ratio);
        file_ << ","
              << telemetry.selected_fixed << ",";
        writeNumber(telemetry.selected_ratio);
        file_ << ","
              << telemetry.selected_pair_count << ","
              << telemetry.selected_distinct_sats << ","
              << telemetry.selected_distinct_systems << ","
              << telemetry.selected_distinct_frequencies << ","
              << telemetry.selected_dual_frequency_sats << ","
              << telemetry.selected_fixed_ambiguities << ","
              << telemetry.selected_used_subset << ","
              << telemetry.used_wlnl_fallback << ","
              << telemetry.validation_attempted << ","
              << telemetry.validation_passed << ",";
        writeNumber(telemetry.postfix_residual_rms);
        file_ << ",";
        writeNumber(telemetry.fixed_float_jump_m);
        file_ << ","
              << telemetry.post_validation_rejected << ","
              << telemetry.final_fixed_applied << ","
              << telemetry.reject_reason << ","
              << libgnss::RTKProcessor::arSkipReasonToString(telemetry.ar_skip_reason) << "\n";
        file_.flush();
    }

private:
    void writeNumber(double value) {
        if (!std::isfinite(value)) {
            return;
        }
        file_ << std::setprecision(6) << value;
    }

    std::ofstream file_;
};

void printUsage(const char* program_name) {
    std::cout
        << "Usage: " << program_name << " [options]\n"
        << "  --data-dir <dir>           Use <dir>/rover.obs, base.obs, navigation.nav\n"
        << "  --rover <file>             Rover RINEX observation file\n"
        << "  --base <file>              Base RINEX observation file\n"
        << "  --nav <file>               Navigation RINEX file\n"
        << "  --out <file>               Output solution file (default: output/rtk_solution.pos)\n"
        << "  --kml <file>               Write KML output (default: output/rtk_solution.kml)\n"
        << "  --no-kml                   Disable KML output\n"
        << "  --format <pos|llh|xyz>     Output text format (default: pos)\n"
        << "  --mode <auto|kinematic|static|moving-base>\n"
        << "                             Position mode (default: auto)\n"
        << "  --iono <auto|off|iflc|est> Ionosphere option (default: auto)\n"
        << "  --ratio <value>            Ambiguity ratio threshold (default: 3.0)\n"
        << "  --preset <survey|low-cost|moving-base|odaiba>\n"
        << "                             Apply a named RTK tuning preset\n"
        << "  --arfilter                 Require extra ratio margin for subset AR fixes\n"
        << "  --no-arfilter              Disable subset AR filter margin even if a preset enables it\n"
        << "  --arfilter-margin <v>      Extra ratio margin for --arfilter (default: 0.25)\n"
        << "  --min-ar-sats <n>          Minimum satellites for AR (default: 5)\n"
        << "  --min-subset-ar-pairs <n>  Minimum DD pairs for subset AR (default: 4)\n"
        << "  --min-subset-ar-sats <n>   Minimum distinct satellites for subset AR\n"
        << "                             (default: 0, disabled)\n"
        << "  --min-subset-ar-systems <n>\n"
        << "                             Minimum distinct constellations for subset AR\n"
        << "                             (default: 0, disabled)\n"
        << "  --min-subset-ar-freqs <n>  Minimum distinct frequencies for subset AR\n"
        << "                             (default: 0, disabled)\n"
        << "  --min-subset-ar-dual-freq-sats <n>\n"
        << "                             Minimum dual-frequency satellites for subset AR\n"
        << "                             (default: 0, disabled)\n"
        << "  --min-full-ratio-for-subset-ar <v>\n"
        << "                             Require full-set LAMBDA ratio before subset AR\n"
        << "                             (default: 0, disabled)\n"
        << "  --min-hold-count <n>       Consecutive fixes before hold ambiguity is allowed (default: 5)\n"
        << "  --hold-ratio-threshold <v> Ratio threshold used while hold ambiguity is active (default: 2.0)\n"
        << "  --elevation-mask-deg <v>   Elevation mask in degrees (default: 15)\n"
        << "  --process-noise-position <v>\n"
        << "                             KF process noise for position state, m^2/s (default: 1e-4)\n"
        << "  --process-noise-ambiguity <v>\n"
        << "                             KF process noise for DD ambiguity state, cycles^2/s (default: 1e-8)\n"
        << "  --process-noise-iono <v>   KF process noise for DD ionosphere state, m^2/s (default: 1e-4)\n"
        << "  --carrier-phase-sigma <v>  Carrier phase observation sigma in m (default: 0.002)\n"
        << "  --no-glonass               Disable GLONASS in RTK carrier processing\n"
        << "  --no-beidou                Disable BeiDou in RTK carrier processing\n"
        << "  --glonass-ar <off|on|autocal> GLONASS ambiguity resolution mode (default: off)\n"
        << "  --glonass-icb-l1 <m/MHz>   GLONASS L1 inter-channel bias slope (default: 0)\n"
        << "  --glonass-icb-l2 <m/MHz>   GLONASS L2 inter-channel bias slope (default: 0)\n"
        << "  --ar-policy <extended|demo5-continuous>\n"
        << "                             AR policy gate (default: extended)\n"
        << "                             demo5-continuous disables relaxed-hold-ratio,\n"
        << "                             subset/partial AR fallback, hold-fix fallback,\n"
        << "                             and Q regularization (raw Q passed to LAMBDA)\n"
        << "  --max-hold-div <v>         Max hold fix divergence from float in meters\n"
        << "                             (default: 0, disabled)\n"
        << "  --max-pos-jump <v>         Max AR fix jump from last fixed pos in meters\n"
        << "                             (default: 5.0; pass 0 to disable, additional to history check)\n"
        << "  --max-pos-jump-min <v>     Min adaptive AR fix jump in meters (default: 0)\n"
        << "  --max-pos-jump-rate <v>    Max adaptive AR fix jump rate in m/s\n"
        << "                             (default: 0, disabled)\n"
        << "  --max-float-spp-div <v>    Max FLOAT divergence from same-epoch SPP in meters\n"
        << "                             (default: 0, disabled)\n"
        << "  --max-float-prefit-rms <v> Max FLOAT prefit DD residual RMS before state reset\n"
        << "                             (default: 0, disabled)\n"
        << "  --max-float-prefit-max <v> Max FLOAT prefit DD residual magnitude before state reset\n"
        << "                             (default: 0, disabled)\n"
        << "  --max-float-prefit-reset-streak <n>\n"
        << "                             Consecutive high-residual FLOAT epochs before reset (default: 3)\n"
        << "  --min-float-prefit-trusted-jump <v>\n"
        << "                             Require high-residual FLOAT to be this far from last trusted pos\n"
        << "                             before reset (default: 0, disabled)\n"
        << "  --max-update-nis-per-obs <v>\n"
        << "                             Reject DD Kalman update when NIS/active observation exceeds v\n"
        << "                             (default: 0, disabled)\n"
        << "  --max-postfix-rms <v>      Max L1 post-fix phase residual RMS in meters\n"
        << "                             (default: 0, disabled)\n"
        << "  --enable-wide-lane-ar      Enable MW wide-lane AR pre-step (default: off)\n"
        << "  --no-wide-lane-ar          Disable MW wide-lane AR, overriding presets\n"
        << "  --wide-lane-threshold <v>  WL float->int threshold in cycles (default: 0.25)\n"
        << "  --max-consec-float-reset <n>\n"
        << "                             Reset ambiguity state after n consecutive float epochs\n"
        << "                             (default: 0, disabled; e.g. 10 for aggressive urban reconvergence)\n"
        << "  --max-consec-nonfix-reset <n>\n"
        << "                             Reset ambiguity state after n consecutive non-FIX epochs\n"
        << "                             including FLOAT/SPP/no-solution (default: 0, disabled)\n"
        << "  --no-nonfix-drift-guard   Disable low-speed non-FIX segment drift rejection\n"
        << "  --nonfix-drift-max-anchor-gap <s>\n"
        << "                             Max FIX-to-FIX gap for non-FIX drift guard (default: 120)\n"
        << "  --nonfix-drift-max-anchor-speed <m/s>\n"
        << "                             Max FIX-anchor speed treated as stationary (default: 1.0)\n"
        << "  --nonfix-drift-max-residual <m>\n"
        << "                             Reject non-FIX epochs farther than this from FIX-anchor bridge\n"
        << "                             in low-speed segments (default: 30)\n"
        << "  --nonfix-drift-min-horizontal-residual <m>\n"
        << "                             Require this horizontal bridge residual before rejecting\n"
        << "                             a non-FIX epoch (default: 0, disabled)\n"
        << "  --nonfix-drift-min-segment-epochs <n>\n"
        << "                             Minimum bounded non-FIX segment length to inspect (default: 20)\n"
        << "  --nonfix-drift-max-segment-epochs <n>\n"
        << "                             Maximum bounded non-FIX segment length to inspect\n"
        << "                             (default: 0, disabled)\n"
        << "  --no-spp-height-step-guard\n"
        << "                             Disable SPP vertical spike rejection\n"
        << "  --spp-height-step-min <m>  Minimum SPP height jump rejected (default: 30)\n"
        << "  --spp-height-step-rate <m/s>\n"
        << "                             Rate-scaled SPP height jump limit (default: 4)\n"
        << "  --float-bridge-tail-guard Enable slow FLOAT bridge-tail rejection (default: on)\n"
        << "  --no-float-bridge-tail-guard\n"
        << "                             Disable slow FLOAT bridge-tail rejection\n"
        << "  --float-bridge-tail-max-anchor-gap <s>\n"
        << "                             Max FIX-to-FIX gap for FLOAT bridge-tail guard (default: 120)\n"
        << "  --float-bridge-tail-min-anchor-speed <m/s>\n"
        << "                             Min horizontal FIX-anchor speed for FLOAT bridge-tail guard (default: 0.4)\n"
        << "  --float-bridge-tail-max-anchor-speed <m/s>\n"
        << "                             Max horizontal FIX-anchor speed for FLOAT bridge-tail guard (default: 1.0)\n"
        << "  --float-bridge-tail-max-residual <m>\n"
        << "                             Reject FLOAT epochs farther than this from FIX-anchor bridge\n"
        << "                             in slow bounded segments (default: 12)\n"
        << "  --float-bridge-tail-min-segment-epochs <n>\n"
        << "                             Minimum bounded FLOAT-tail segment length to inspect (default: 20)\n"
        << "  --fixed-bridge-burst-guard\n"
        << "                             Enable short FIX burst bridge-residual rejection (default: off)\n"
        << "  --no-fixed-bridge-burst-guard\n"
        << "                             Disable short FIX burst bridge-residual rejection\n"
        << "  --fixed-bridge-burst-max-anchor-gap <s>\n"
        << "                             Max FIX-anchor gap for fixed-burst guard (default: 30)\n"
        << "  --fixed-bridge-burst-min-boundary-gap <s>\n"
        << "                             Min gap around a short FIX burst (default: 1)\n"
        << "  --fixed-bridge-burst-max-residual <m>\n"
        << "                             Reject burst FIX epochs farther than this from FIX bridge (default: 20)\n"
        << "  --fixed-bridge-burst-max-segment-epochs <n>\n"
        << "                             Max short FIX segment length to inspect (default: 12)\n"
        << "  --max-baseline-m <v>       Max baseline length in meters (default: 20000)\n"
        << "  --base-ecef <x> <y> <z>    Override base ECEF position in meters\n"
        << "  --skip-epochs <n>          Skip the first n rover epochs before solving\n"
        << "  --max-epochs <n>           Stop after n rover epochs\n"
        << "  --debug-epoch-log <csv>    Write per-epoch AR/debug telemetry CSV\n"
        << "  --no-kinematic-post-filter Disable the kinematic output post-filter\n"
        << "  --no-base-interp           Require exact rover/base epoch alignment\n"
        << "  --verbose                  Print per-epoch progress summary\n"
        << "  -h, --help                 Show this help\n";
}

[[noreturn]] void argumentError(const std::string& message, const char* program_name) {
    std::cerr << "Argument error: " << message << "\n\n";
    printUsage(program_name);
    std::exit(1);
}

ModeChoice parseModeChoice(const std::string& value, const char* program_name) {
    if (value == "auto") return ModeChoice::AUTO;
    if (value == "kinematic") return ModeChoice::KINEMATIC;
    if (value == "static") return ModeChoice::STATIC;
    if (value == "moving-base") return ModeChoice::MOVING_BASE;
    argumentError("unsupported --mode value: " + value, program_name);
}

IonoChoice parseIonoChoice(const std::string& value, const char* program_name) {
    if (value == "auto") return IonoChoice::AUTO;
    if (value == "off") return IonoChoice::OFF;
    if (value == "iflc") return IonoChoice::IFLC;
    if (value == "est") return IonoChoice::EST;
    argumentError("unsupported --iono value: " + value, program_name);
}

GlonassARChoice parseGlonassARChoice(const std::string& value, const char* program_name) {
    if (value == "off") return GlonassARChoice::OFF;
    if (value == "on") return GlonassARChoice::ON;
    if (value == "autocal") return GlonassARChoice::AUTOCAL;
    argumentError("unsupported --glonass-ar value: " + value, program_name);
}

RTKTuningPreset parseRTKTuningPreset(const std::string& value, const char* program_name) {
    if (value == "survey") return RTKTuningPreset::SURVEY;
    if (value == "low-cost") return RTKTuningPreset::LOW_COST;
    if (value == "moving-base") return RTKTuningPreset::MOVING_BASE;
    if (value == "odaiba") return RTKTuningPreset::ODAIBA;
    argumentError("unsupported --preset value: " + value, program_name);
}

void applyRTKTuningPreset(SolveConfig& config) {
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
            return;
        case RTKTuningPreset::MOVING_BASE:
            if (!config.ratio_threshold_set) config.ratio_threshold = 2.8;
            if (!config.has_ar_filter_override) config.enable_ar_filter = true;
            if (!config.ar_filter_margin_set) config.ar_filter_margin = 0.20;
            if (!config.min_satellites_for_ar_set) config.min_satellites_for_ar = 6;
            if (!config.min_hold_count_set) config.min_hold_count = 8;
            if (!config.hold_ratio_threshold_set) config.hold_ratio_threshold = 2.4;
            return;
        case RTKTuningPreset::ODAIBA:
            if (!config.ratio_threshold_set) config.ratio_threshold = 3.0;
            if (!config.has_ar_filter_override) config.enable_ar_filter = true;
            if (!config.ar_filter_margin_set) config.ar_filter_margin = 0.35;
            if (!config.min_satellites_for_ar_set) config.min_satellites_for_ar = 6;
            if (!config.min_hold_count_set) config.min_hold_count = 8;
            if (!config.hold_ratio_threshold_set) config.hold_ratio_threshold = 2.5;
            if (!config.wide_lane_ar_set) config.enable_wide_lane_ar = true;
            if (!config.wide_lane_acceptance_threshold_set) {
                config.wide_lane_acceptance_threshold = 0.12;
            }
            return;
    }
}

libgnss::io::SolutionWriter::Format parseOutputFormat(const std::string& value,
                                                      const char* program_name) {
    if (value == "pos") return libgnss::io::SolutionWriter::Format::POS;
    if (value == "llh") return libgnss::io::SolutionWriter::Format::LLH;
    if (value == "xyz") return libgnss::io::SolutionWriter::Format::XYZ;
    argumentError("unsupported --format value: " + value, program_name);
}

SolveConfig parseArguments(int argc, char* argv[]) {
    SolveConfig config;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];

        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            std::exit(0);
        }
        if (arg == "--data-dir" && i + 1 < argc) {
            config.data_dir = argv[++i];
        } else if (arg == "--rover" && i + 1 < argc) {
            config.rover_obs_path = argv[++i];
        } else if (arg == "--base" && i + 1 < argc) {
            config.base_obs_path = argv[++i];
        } else if (arg == "--nav" && i + 1 < argc) {
            config.nav_path = argv[++i];
        } else if (arg == "--out" && i + 1 < argc) {
            config.output_pos_path = argv[++i];
        } else if (arg == "--kml" && i + 1 < argc) {
            config.output_kml_path = argv[++i];
            config.write_kml = true;
        } else if (arg == "--no-kml") {
            config.write_kml = false;
        } else if (arg == "--format" && i + 1 < argc) {
            config.output_format = parseOutputFormat(argv[++i], argv[0]);
        } else if (arg == "--mode" && i + 1 < argc) {
            config.mode = parseModeChoice(argv[++i], argv[0]);
        } else if (arg == "--iono" && i + 1 < argc) {
            config.iono = parseIonoChoice(argv[++i], argv[0]);
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
        } else if (arg == "--min-subset-ar-pairs" && i + 1 < argc) {
            config.min_subset_pairs_for_ar = std::stoi(argv[++i]);
        } else if (arg == "--min-subset-ar-sats" && i + 1 < argc) {
            config.min_subset_sats_for_ar = std::stoi(argv[++i]);
            config.min_subset_sats_for_ar_set = true;
        } else if (arg == "--min-subset-ar-systems" && i + 1 < argc) {
            config.min_subset_systems_for_ar = std::stoi(argv[++i]);
            config.min_subset_systems_for_ar_set = true;
        } else if (arg == "--min-subset-ar-freqs" && i + 1 < argc) {
            config.min_subset_frequencies_for_ar = std::stoi(argv[++i]);
            config.min_subset_frequencies_for_ar_set = true;
        } else if (arg == "--min-subset-ar-dual-freq-sats" && i + 1 < argc) {
            config.min_subset_dual_frequency_sats_for_ar = std::stoi(argv[++i]);
            config.min_subset_dual_frequency_sats_for_ar_set = true;
        } else if (arg == "--min-full-ratio-for-subset-ar" && i + 1 < argc) {
            config.min_full_ratio_for_subset_ar = std::stod(argv[++i]);
        } else if (arg == "--min-hold-count" && i + 1 < argc) {
            config.min_hold_count = std::stoi(argv[++i]);
            config.min_hold_count_set = true;
        } else if (arg == "--hold-ratio-threshold" && i + 1 < argc) {
            config.hold_ratio_threshold = std::stod(argv[++i]);
            config.hold_ratio_threshold_set = true;
        } else if (arg == "--elevation-mask-deg" && i + 1 < argc) {
            config.elevation_mask_deg = std::stod(argv[++i]);
        } else if (arg == "--process-noise-position" && i + 1 < argc) {
            config.process_noise_position = std::stod(argv[++i]);
            config.process_noise_position_set = true;
        } else if (arg == "--process-noise-ambiguity" && i + 1 < argc) {
            config.process_noise_ambiguity = std::stod(argv[++i]);
            config.process_noise_ambiguity_set = true;
        } else if (arg == "--process-noise-iono" && i + 1 < argc) {
            config.process_noise_iono = std::stod(argv[++i]);
            config.process_noise_iono_set = true;
        } else if (arg == "--carrier-phase-sigma" && i + 1 < argc) {
            config.carrier_phase_sigma = std::stod(argv[++i]);
            config.carrier_phase_sigma_set = true;
        } else if (arg == "--no-glonass") {
            config.enable_glonass = false;
        } else if (arg == "--no-beidou") {
            config.enable_beidou = false;
        } else if (arg == "--glonass-ar" && i + 1 < argc) {
            config.glonass_ar = parseGlonassARChoice(argv[++i], argv[0]);
        } else if (arg == "--glonass-icb-l1" && i + 1 < argc) {
            config.glonass_icb_l1_m_per_mhz = std::stod(argv[++i]);
        } else if (arg == "--glonass-icb-l2" && i + 1 < argc) {
            config.glonass_icb_l2_m_per_mhz = std::stod(argv[++i]);
        } else if (arg == "--ar-policy" && i + 1 < argc) {
            const std::string policy_str = argv[++i];
            if (policy_str == "extended") {
                config.ar_policy = libgnss::RTKProcessor::RTKConfig::ARPolicy::EXTENDED;
            } else if (policy_str == "demo5-continuous") {
                config.ar_policy = libgnss::RTKProcessor::RTKConfig::ARPolicy::DEMO5_CONTINUOUS;
            } else {
                argumentError("unsupported --ar-policy value: " + policy_str, argv[0]);
            }
        } else if (arg == "--max-hold-div" && i + 1 < argc) {
            config.max_hold_divergence_m = std::stod(argv[++i]);
        } else if (arg == "--max-pos-jump" && i + 1 < argc) {
            config.max_position_jump_m = std::stod(argv[++i]);
        } else if (arg == "--max-pos-jump-min" && i + 1 < argc) {
            config.max_position_jump_min_m = std::stod(argv[++i]);
        } else if (arg == "--max-pos-jump-rate" && i + 1 < argc) {
            config.max_position_jump_rate_mps = std::stod(argv[++i]);
        } else if (arg == "--max-float-spp-div" && i + 1 < argc) {
            config.max_float_spp_divergence_m = std::stod(argv[++i]);
        } else if (arg == "--max-float-prefit-rms" && i + 1 < argc) {
            config.max_float_prefit_residual_rms_m = std::stod(argv[++i]);
        } else if (arg == "--max-float-prefit-max" && i + 1 < argc) {
            config.max_float_prefit_residual_max_m = std::stod(argv[++i]);
        } else if (arg == "--max-float-prefit-reset-streak" && i + 1 < argc) {
            config.max_float_prefit_residual_reset_streak = std::stoi(argv[++i]);
        } else if (arg == "--min-float-prefit-trusted-jump" && i + 1 < argc) {
            config.min_float_prefit_residual_trusted_jump_m = std::stod(argv[++i]);
        } else if (arg == "--max-update-nis-per-obs" && i + 1 < argc) {
            config.max_update_nis_per_observation = std::stod(argv[++i]);
        } else if (arg == "--max-postfix-rms" && i + 1 < argc) {
            config.max_postfix_residual_rms = std::stod(argv[++i]);
        } else if (arg == "--enable-wide-lane-ar") {
            config.enable_wide_lane_ar = true;
            config.wide_lane_ar_set = true;
        } else if (arg == "--no-wide-lane-ar") {
            config.enable_wide_lane_ar = false;
            config.wide_lane_ar_set = true;
        } else if (arg == "--wide-lane-threshold" && i + 1 < argc) {
            config.wide_lane_acceptance_threshold = std::stod(argv[++i]);
            config.wide_lane_acceptance_threshold_set = true;
        } else if (arg == "--max-consec-float-reset" && i + 1 < argc) {
            config.max_consecutive_float_for_reset = std::stoi(argv[++i]);
        } else if (arg == "--max-consec-nonfix-reset" && i + 1 < argc) {
            config.max_consecutive_nonfix_for_reset = std::stoi(argv[++i]);
        } else if (arg == "--no-nonfix-drift-guard") {
            config.enable_nonfix_drift_guard = false;
        } else if (arg == "--nonfix-drift-max-anchor-gap" && i + 1 < argc) {
            config.nonfix_drift_guard_max_anchor_gap_s = std::stod(argv[++i]);
        } else if (arg == "--nonfix-drift-max-anchor-speed" && i + 1 < argc) {
            config.nonfix_drift_guard_max_anchor_speed_mps = std::stod(argv[++i]);
        } else if (arg == "--nonfix-drift-max-residual" && i + 1 < argc) {
            config.nonfix_drift_guard_max_residual_m = std::stod(argv[++i]);
        } else if (arg == "--nonfix-drift-min-horizontal-residual" && i + 1 < argc) {
            config.nonfix_drift_guard_min_horizontal_residual_m = std::stod(argv[++i]);
        } else if (arg == "--nonfix-drift-min-segment-epochs" && i + 1 < argc) {
            config.nonfix_drift_guard_min_segment_epochs = std::stoi(argv[++i]);
        } else if (arg == "--nonfix-drift-max-segment-epochs" && i + 1 < argc) {
            config.nonfix_drift_guard_max_segment_epochs = std::stoi(argv[++i]);
        } else if (arg == "--no-spp-height-step-guard") {
            config.enable_spp_height_step_guard = false;
        } else if (arg == "--spp-height-step-min" && i + 1 < argc) {
            config.spp_height_step_guard_min_m = std::stod(argv[++i]);
        } else if (arg == "--spp-height-step-rate" && i + 1 < argc) {
            config.spp_height_step_guard_max_rate_mps = std::stod(argv[++i]);
        } else if (arg == "--float-bridge-tail-guard") {
            config.enable_float_bridge_tail_guard = true;
        } else if (arg == "--no-float-bridge-tail-guard") {
            config.enable_float_bridge_tail_guard = false;
        } else if (arg == "--float-bridge-tail-max-anchor-gap" && i + 1 < argc) {
            config.float_bridge_tail_guard_max_anchor_gap_s = std::stod(argv[++i]);
        } else if (arg == "--float-bridge-tail-min-anchor-speed" && i + 1 < argc) {
            config.float_bridge_tail_guard_min_anchor_speed_mps = std::stod(argv[++i]);
        } else if (arg == "--float-bridge-tail-max-anchor-speed" && i + 1 < argc) {
            config.float_bridge_tail_guard_max_anchor_speed_mps = std::stod(argv[++i]);
        } else if (arg == "--float-bridge-tail-max-residual" && i + 1 < argc) {
            config.float_bridge_tail_guard_max_residual_m = std::stod(argv[++i]);
        } else if (arg == "--float-bridge-tail-min-segment-epochs" && i + 1 < argc) {
            config.float_bridge_tail_guard_min_segment_epochs = std::stoi(argv[++i]);
        } else if (arg == "--fixed-bridge-burst-guard") {
            config.enable_fixed_bridge_burst_guard = true;
        } else if (arg == "--no-fixed-bridge-burst-guard") {
            config.enable_fixed_bridge_burst_guard = false;
        } else if (arg == "--fixed-bridge-burst-max-anchor-gap" && i + 1 < argc) {
            config.fixed_bridge_burst_guard_max_anchor_gap_s = std::stod(argv[++i]);
        } else if (arg == "--fixed-bridge-burst-min-boundary-gap" && i + 1 < argc) {
            config.fixed_bridge_burst_guard_min_boundary_gap_s = std::stod(argv[++i]);
        } else if (arg == "--fixed-bridge-burst-max-residual" && i + 1 < argc) {
            config.fixed_bridge_burst_guard_max_residual_m = std::stod(argv[++i]);
        } else if (arg == "--fixed-bridge-burst-max-segment-epochs" && i + 1 < argc) {
            config.fixed_bridge_burst_guard_max_segment_epochs = std::stoi(argv[++i]);
        } else if (arg == "--max-baseline-m" && i + 1 < argc) {
            config.max_baseline_length_m = std::stod(argv[++i]);
        } else if (arg == "--base-ecef" && i + 3 < argc) {
            config.base_position_ecef =
                Eigen::Vector3d(std::stod(argv[++i]), std::stod(argv[++i]), std::stod(argv[++i]));
            config.base_position_override = true;
        } else if (arg == "--skip-epochs" && i + 1 < argc) {
            config.skip_epochs = std::stoi(argv[++i]);
        } else if (arg == "--max-epochs" && i + 1 < argc) {
            config.max_epochs = std::stoi(argv[++i]);
        } else if (arg == "--debug-epoch-log" && i + 1 < argc) {
            config.debug_epoch_log_path = argv[++i];
        } else if (arg == "--no-kinematic-post-filter") {
            config.enable_kinematic_post_filter = false;
        } else if (arg == "--no-base-interp") {
            config.enable_base_interpolation = false;
        } else if (arg == "--verbose") {
            config.verbose = true;
        } else {
            argumentError("unknown or incomplete argument: " + arg, argv[0]);
        }
    }

    if (!config.data_dir.empty()) {
        if (config.rover_obs_path.empty()) config.rover_obs_path = config.data_dir + "/rover.obs";
        if (config.base_obs_path.empty()) config.base_obs_path = config.data_dir + "/base.obs";
        if (config.nav_path.empty()) config.nav_path = config.data_dir + "/navigation.nav";
    }

    applyRTKTuningPreset(config);

    if (config.rover_obs_path.empty() || config.base_obs_path.empty() || config.nav_path.empty()) {
        argumentError("provide --data-dir or all of --rover, --base, and --nav", argv[0]);
    }
    if (config.min_satellites_for_ar < 4) {
        argumentError("--min-ar-sats must be >= 4", argv[0]);
    }
    if (config.min_subset_pairs_for_ar < 4) {
        argumentError("--min-subset-ar-pairs must be >= 4", argv[0]);
    }
    if (config.min_subset_sats_for_ar < 0) {
        argumentError("--min-subset-ar-sats must be >= 0", argv[0]);
    }
    if (config.min_subset_systems_for_ar < 0) {
        argumentError("--min-subset-ar-systems must be >= 0", argv[0]);
    }
    if (config.min_subset_frequencies_for_ar < 0) {
        argumentError("--min-subset-ar-freqs must be >= 0", argv[0]);
    }
    if (config.min_subset_dual_frequency_sats_for_ar < 0) {
        argumentError("--min-subset-ar-dual-freq-sats must be >= 0", argv[0]);
    }
    if (config.min_full_ratio_for_subset_ar < 0.0) {
        argumentError("--min-full-ratio-for-subset-ar must be >= 0", argv[0]);
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
    if (config.elevation_mask_deg < 0.0 || config.elevation_mask_deg >= 90.0) {
        argumentError("--elevation-mask-deg must be in [0, 90)", argv[0]);
    }
    if (config.max_baseline_length_m <= 0.0) {
        argumentError("--max-baseline-m must be > 0", argv[0]);
    }
    if (config.max_position_jump_m < 0.0) {
        argumentError("--max-pos-jump must be >= 0", argv[0]);
    }
    if (config.max_position_jump_min_m < 0.0) {
        argumentError("--max-pos-jump-min must be >= 0", argv[0]);
    }
    if (config.max_position_jump_rate_mps < 0.0) {
        argumentError("--max-pos-jump-rate must be >= 0", argv[0]);
    }
    if (config.max_float_spp_divergence_m < 0.0) {
        argumentError("--max-float-spp-div must be >= 0", argv[0]);
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
    if (config.min_float_prefit_residual_trusted_jump_m < 0.0) {
        argumentError("--min-float-prefit-trusted-jump must be >= 0", argv[0]);
    }
    if (config.max_update_nis_per_observation < 0.0) {
        argumentError("--max-update-nis-per-obs must be >= 0", argv[0]);
    }
    if (config.nonfix_drift_guard_max_anchor_gap_s <= 0.0) {
        argumentError("--nonfix-drift-max-anchor-gap must be > 0", argv[0]);
    }
    if (config.nonfix_drift_guard_max_anchor_speed_mps < 0.0) {
        argumentError("--nonfix-drift-max-anchor-speed must be >= 0", argv[0]);
    }
    if (config.nonfix_drift_guard_max_residual_m <= 0.0) {
        argumentError("--nonfix-drift-max-residual must be > 0", argv[0]);
    }
    if (config.nonfix_drift_guard_min_horizontal_residual_m < 0.0) {
        argumentError("--nonfix-drift-min-horizontal-residual must be >= 0", argv[0]);
    }
    if (config.nonfix_drift_guard_min_segment_epochs < 1) {
        argumentError("--nonfix-drift-min-segment-epochs must be >= 1", argv[0]);
    }
    if (config.nonfix_drift_guard_max_segment_epochs < 0) {
        argumentError("--nonfix-drift-max-segment-epochs must be >= 0", argv[0]);
    }
    if (config.spp_height_step_guard_min_m <= 0.0) {
        argumentError("--spp-height-step-min must be > 0", argv[0]);
    }
    if (config.spp_height_step_guard_max_rate_mps < 0.0) {
        argumentError("--spp-height-step-rate must be >= 0", argv[0]);
    }
    if (config.float_bridge_tail_guard_max_anchor_gap_s <= 0.0) {
        argumentError("--float-bridge-tail-max-anchor-gap must be > 0", argv[0]);
    }
    if (config.float_bridge_tail_guard_min_anchor_speed_mps < 0.0) {
        argumentError("--float-bridge-tail-min-anchor-speed must be >= 0", argv[0]);
    }
    if (config.float_bridge_tail_guard_max_anchor_speed_mps <
        config.float_bridge_tail_guard_min_anchor_speed_mps) {
        argumentError("--float-bridge-tail-max-anchor-speed must be >= min anchor speed", argv[0]);
    }
    if (config.float_bridge_tail_guard_max_residual_m <= 0.0) {
        argumentError("--float-bridge-tail-max-residual must be > 0", argv[0]);
    }
    if (config.float_bridge_tail_guard_min_segment_epochs < 1) {
        argumentError("--float-bridge-tail-min-segment-epochs must be >= 1", argv[0]);
    }
    if (config.fixed_bridge_burst_guard_max_anchor_gap_s <= 0.0) {
        argumentError("--fixed-bridge-burst-max-anchor-gap must be > 0", argv[0]);
    }
    if (config.fixed_bridge_burst_guard_min_boundary_gap_s < 0.0) {
        argumentError("--fixed-bridge-burst-min-boundary-gap must be >= 0", argv[0]);
    }
    if (config.fixed_bridge_burst_guard_max_residual_m <= 0.0) {
        argumentError("--fixed-bridge-burst-max-residual must be > 0", argv[0]);
    }
    if (config.fixed_bridge_burst_guard_max_segment_epochs < 1) {
        argumentError("--fixed-bridge-burst-max-segment-epochs must be >= 1", argv[0]);
    }
    if (config.skip_epochs < 0) {
        argumentError("--skip-epochs must be >= 0", argv[0]);
    }

    return config;
}

bool pathLooksStatic(const std::string& text) {
    return text.find("short_baseline") != std::string::npos ||
        text.find("static") != std::string::npos;
}

bool pathLooksShortBaseline(const std::string& text) {
    return text.find("short_baseline") != std::string::npos;
}

libgnss::RTKProcessor::RTKConfig::PositionMode resolvePositionMode(const SolveConfig& config) {
    if (config.mode == ModeChoice::KINEMATIC) {
        return libgnss::RTKProcessor::RTKConfig::PositionMode::KINEMATIC;
    }
    if (config.mode == ModeChoice::STATIC) {
        return libgnss::RTKProcessor::RTKConfig::PositionMode::STATIC;
    }
    if (config.mode == ModeChoice::MOVING_BASE) {
        return libgnss::RTKProcessor::RTKConfig::PositionMode::MOVING_BASE;
    }

    const std::string hint = config.data_dir + " " + config.rover_obs_path + " " + config.base_obs_path;
    if (pathLooksStatic(hint)) {
        return libgnss::RTKProcessor::RTKConfig::PositionMode::STATIC;
    }
    return libgnss::RTKProcessor::RTKConfig::PositionMode::KINEMATIC;
}

libgnss::RTKProcessor::RTKConfig::IonoOpt resolveIonoOpt(const SolveConfig& config,
                                                         libgnss::RTKProcessor::RTKConfig::PositionMode mode) {
    if (config.iono == IonoChoice::OFF) {
        return libgnss::RTKProcessor::RTKConfig::IonoOpt::OFF;
    }
    if (config.iono == IonoChoice::IFLC) {
        return libgnss::RTKProcessor::RTKConfig::IonoOpt::IFLC;
    }
    if (config.iono == IonoChoice::EST) {
        return libgnss::RTKProcessor::RTKConfig::IonoOpt::EST;
    }

    const std::string hint = config.data_dir + " " + config.rover_obs_path + " " + config.base_obs_path;
    const bool short_baseline = pathLooksShortBaseline(hint);
    if (mode == libgnss::RTKProcessor::RTKConfig::PositionMode::STATIC && !short_baseline) {
        return libgnss::RTKProcessor::RTKConfig::IonoOpt::IFLC;
    }
    return libgnss::RTKProcessor::RTKConfig::IonoOpt::OFF;
}

std::string positionModeString(libgnss::RTKProcessor::RTKConfig::PositionMode mode) {
    switch (mode) {
        case libgnss::RTKProcessor::RTKConfig::PositionMode::STATIC:
            return "static";
        case libgnss::RTKProcessor::RTKConfig::PositionMode::KINEMATIC:
            return "kinematic";
        case libgnss::RTKProcessor::RTKConfig::PositionMode::MOVING_BASE:
            return "moving-base";
    }
    return "kinematic";
}

std::string ionoOptString(libgnss::RTKProcessor::RTKConfig::IonoOpt iono) {
    switch (iono) {
        case libgnss::RTKProcessor::RTKConfig::IonoOpt::OFF:
            return "off";
        case libgnss::RTKProcessor::RTKConfig::IonoOpt::IFLC:
            return "iflc";
        case libgnss::RTKProcessor::RTKConfig::IonoOpt::EST:
            return "est";
    }
    return "off";
}

bool writeSolutions(const SolveConfig& config, const libgnss::Solution& solution) {
    libgnss::io::SolutionWriter writer;
    if (!writer.open(config.output_pos_path, config.output_format)) {
        std::cerr << "Error: failed to open output file: " << config.output_pos_path << std::endl;
        return false;
    }

    for (const auto& epoch_solution : solution.solutions) {
        writer.writeEpoch(epoch_solution);
    }
    writer.close();

    if (config.write_kml && !solution.writeKML(config.output_kml_path)) {
        std::cerr << "Error: failed to write KML output: " << config.output_kml_path << std::endl;
        return false;
    }
    return true;
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        const SolveConfig config = parseArguments(argc, argv);

        libgnss::RTKProcessor rtk_processor;
        libgnss::SPPProcessor spp_processor;
        libgnss::RTKProcessor::RTKConfig rtk_config;
        rtk_config.max_baseline_length = config.max_baseline_length_m;
        rtk_config.ar_mode = libgnss::RTKProcessor::RTKConfig::AmbiguityResolutionMode::CONTINUOUS;
        rtk_config.ratio_threshold = config.ratio_threshold;
        rtk_config.ambiguity_ratio_threshold = config.ratio_threshold;
        rtk_config.hold_ambiguity_ratio_threshold = config.hold_ratio_threshold;
        rtk_config.enable_ar_filter = config.enable_ar_filter;
        rtk_config.ar_filter_margin = config.ar_filter_margin;
        rtk_config.min_satellites_for_ar = config.min_satellites_for_ar;
        rtk_config.min_subset_pairs_for_ar = config.min_subset_pairs_for_ar;
        rtk_config.min_subset_sats_for_ar = config.min_subset_sats_for_ar;
        rtk_config.min_subset_systems_for_ar = config.min_subset_systems_for_ar;
        rtk_config.min_subset_frequencies_for_ar = config.min_subset_frequencies_for_ar;
        rtk_config.min_subset_dual_frequency_sats_for_ar =
            config.min_subset_dual_frequency_sats_for_ar;
        rtk_config.min_full_ratio_for_subset_ar = config.min_full_ratio_for_subset_ar;
        rtk_config.min_hold_count = config.min_hold_count;
        rtk_config.elevation_mask = config.elevation_mask_deg * M_PI / 180.0;
        if (config.process_noise_position_set) {
            rtk_config.process_noise_position = config.process_noise_position;
        }
        if (config.process_noise_ambiguity_set) {
            rtk_config.process_noise_ambiguity = config.process_noise_ambiguity;
        }
        if (config.process_noise_iono_set) {
            rtk_config.process_noise_iono = config.process_noise_iono;
        }
        if (config.carrier_phase_sigma_set) {
            rtk_config.carrier_phase_sigma = config.carrier_phase_sigma;
        }
        rtk_config.position_mode = resolvePositionMode(config);
        rtk_config.ionoopt = resolveIonoOpt(config, rtk_config.position_mode);
        rtk_config.enable_glonass = config.enable_glonass;
        rtk_config.enable_beidou = config.enable_beidou;
        rtk_config.glonass_ar_mode =
            config.glonass_ar == GlonassARChoice::AUTOCAL
                ? libgnss::RTKProcessor::RTKConfig::GlonassARMode::AUTOCAL
                : (config.glonass_ar == GlonassARChoice::ON
                       ? libgnss::RTKProcessor::RTKConfig::GlonassARMode::ON
                       : libgnss::RTKProcessor::RTKConfig::GlonassARMode::OFF);
        rtk_config.glonass_icb_l1_m_per_mhz = config.glonass_icb_l1_m_per_mhz;
        rtk_config.glonass_icb_l2_m_per_mhz = config.glonass_icb_l2_m_per_mhz;
        rtk_config.ar_policy = config.ar_policy;
        rtk_config.max_hold_divergence_m = config.max_hold_divergence_m;
        rtk_config.max_position_jump_m = config.max_position_jump_m;
        rtk_config.max_position_jump_min_m = config.max_position_jump_min_m;
        rtk_config.max_position_jump_rate_mps = config.max_position_jump_rate_mps;
        rtk_config.max_float_spp_divergence_m = config.max_float_spp_divergence_m;
        rtk_config.max_float_prefit_residual_rms_m =
            config.max_float_prefit_residual_rms_m;
        rtk_config.max_float_prefit_residual_max_m =
            config.max_float_prefit_residual_max_m;
        rtk_config.max_float_prefit_residual_reset_streak =
            config.max_float_prefit_residual_reset_streak;
        rtk_config.min_float_prefit_residual_trusted_jump_m =
            config.min_float_prefit_residual_trusted_jump_m;
        rtk_config.max_update_nis_per_observation = config.max_update_nis_per_observation;
        rtk_config.max_consecutive_float_for_reset = config.max_consecutive_float_for_reset;
        rtk_config.max_consecutive_nonfix_for_reset = config.max_consecutive_nonfix_for_reset;
        rtk_config.max_postfix_residual_rms = config.max_postfix_residual_rms;
        rtk_config.enable_wide_lane_ar = config.enable_wide_lane_ar;
        rtk_config.wide_lane_acceptance_threshold = config.wide_lane_acceptance_threshold;
        rtk_processor.setRTKConfig(rtk_config);
        libgnss::SPPProcessor::SPPConfig spp_config;
        spp_config.use_multi_constellation = true;
        spp_config.enable_glonass = config.enable_glonass;
        spp_config.enable_beidou = config.enable_beidou;
        spp_processor.setSPPConfig(spp_config);
        EpochDebugWriter debug_writer;
        if (!debug_writer.open(config.debug_epoch_log_path)) {
            std::cerr << "Error: failed to open debug epoch log: "
                      << config.debug_epoch_log_path << std::endl;
            return 1;
        }

        std::cout << "libgnss++ post-process solver" << std::endl;
        std::cout << "  rover: " << config.rover_obs_path << std::endl;
        std::cout << "  base: " << config.base_obs_path << std::endl;
        std::cout << "  nav: " << config.nav_path << std::endl;
        std::cout << "  out: " << config.output_pos_path << " (" << outputFormatString(config.output_format)
                  << ")" << std::endl;
        if (config.write_kml) {
            std::cout << "  kml: " << config.output_kml_path << std::endl;
        }
        std::cout << "  mode: " << positionModeString(rtk_config.position_mode)
                  << " (requested " << modeChoiceString(config.mode) << ")" << std::endl;
        std::cout << "  iono: " << ionoOptString(rtk_config.ionoopt)
                  << " (requested " << ionoChoiceString(config.iono) << ")" << std::endl;
        std::cout << "  carrier constellations: GLONASS "
                  << (rtk_config.enable_glonass ? "on" : "off")
                  << ", BeiDou " << (rtk_config.enable_beidou ? "on" : "off") << std::endl;
        std::cout << "  GLONASS AR: " << glonassARChoiceString(config.glonass_ar)
                  << " (L1 ICB " << config.glonass_icb_l1_m_per_mhz
                  << " m/MHz, L2 ICB " << config.glonass_icb_l2_m_per_mhz << " m/MHz)"
                  << std::endl;
        std::cout << "  base interpolation: "
                  << (config.enable_base_interpolation ? "enabled" : "disabled") << std::endl;

        libgnss::io::RINEXReader rover_reader;
        libgnss::io::RINEXReader base_reader;
        libgnss::io::RINEXReader nav_reader;

        if (!rover_reader.open(config.rover_obs_path)) {
            std::cerr << "Error: cannot open rover observation file" << std::endl;
            return 1;
        }
        libgnss::io::RINEXReader::RINEXHeader rover_header;
        if (!rover_reader.readHeader(rover_header)) {
            std::cerr << "Error: failed to read rover observation header" << std::endl;
            return 1;
        }

        if (!base_reader.open(config.base_obs_path)) {
            std::cerr << "Error: cannot open base observation file" << std::endl;
            return 1;
        }
        libgnss::io::RINEXReader::RINEXHeader base_header;
        if (!base_reader.readHeader(base_header)) {
            std::cerr << "Error: failed to read base observation header" << std::endl;
            return 1;
        }

        if (!nav_reader.open(config.nav_path)) {
            std::cerr << "Error: cannot open navigation file" << std::endl;
            return 1;
        }
        libgnss::NavigationData nav_data;
        if (!nav_reader.readNavigationData(nav_data)) {
            std::cerr << "Error: failed to read navigation data" << std::endl;
            return 1;
        }

        Eigen::Vector3d base_position = Eigen::Vector3d::Zero();
        if (config.base_position_override) {
            base_position = config.base_position_ecef;
            if (base_header.approximate_position.norm() > 0.0) {
                const double override_delta =
                    (config.base_position_ecef - base_header.approximate_position).norm();
                if (override_delta > 1000.0) {
                    std::cerr << "Warning: --base-ecef differs from RINEX header by "
                              << override_delta << " m; this may severely degrade RTK alignment."
                              << std::endl;
                }
            }
        } else if (base_header.approximate_position.norm() > 0.0) {
            base_position = base_header.approximate_position;
        } else {
            std::cerr << "Error: base position unavailable. Use --base-ecef to override." << std::endl;
            return 1;
        }
        rtk_processor.setBasePosition(base_position);

        libgnss::Solution solution;
        libgnss::ObservationData rover_obs;
        libgnss::ObservationData base_obs;
        libgnss::ObservationData previous_base_obs;
        bool has_previous_base = false;
        bool rover_ok = rover_reader.readObservationEpoch(rover_obs);
        bool base_ok = base_reader.readObservationEpoch(base_obs);

        if (!rover_ok) {
            std::cerr << "Error: no rover epochs available" << std::endl;
            return 1;
        }
        if (!base_ok) {
            std::cerr << "Error: no base epochs available" << std::endl;
            return 1;
        }

        if (rover_header.approximate_position.norm() > 0.0) {
            rover_obs.receiver_position = rover_header.approximate_position;
        } else {
            rover_obs.receiver_position = base_position + Eigen::Vector3d(3000.0, 0.0, 0.0);
        }

        int skipped_initial_epochs = 0;
        while (rover_ok && skipped_initial_epochs < config.skip_epochs) {
            const Eigen::Vector3d saved_rover_pos = rover_obs.receiver_position;
            rover_ok = rover_reader.readObservationEpoch(rover_obs);
            if (rover_ok) {
                rover_obs.receiver_position = saved_rover_pos;
            }
            skipped_initial_epochs++;
        }
        if (!rover_ok) {
            std::cerr << "Error: skip-epochs exhausted the rover observation stream" << std::endl;
            return 1;
        }

        int processed_rover_epochs = 0;
        int valid_solution_count = 0;
        int fixed_solution_count = 0;
        int exact_base_epochs = 0;
        int interpolated_base_epochs = 0;
        int skipped_rover_epochs = 0;
        int nonfix_drift_guard_inspected_segments = 0;
        int nonfix_drift_guard_rejected_segments = 0;
        int nonfix_drift_guard_rejected_epochs = 0;
        int spp_height_step_guard_rejected_epochs = 0;
        int float_bridge_tail_guard_inspected_segments = 0;
        int float_bridge_tail_guard_rejected_segments = 0;
        int float_bridge_tail_guard_rejected_epochs = 0;
        int fixed_bridge_burst_guard_inspected_segments = 0;
        int fixed_bridge_burst_guard_rejected_segments = 0;
        int fixed_bridge_burst_guard_rejected_epochs = 0;
        libgnss::PositionSolution last_fixed_output;
        bool have_last_fixed_output = false;

        while (rover_ok) {
            if (config.max_epochs > 0 && processed_rover_epochs >= config.max_epochs) {
                break;
            }

            libgnss::ObservationData lower_base_obs = previous_base_obs;
            bool have_lower_base = has_previous_base;

            while (base_ok && timeDiffSeconds(base_obs.time, rover_obs.time) < -kExactTimeToleranceSeconds) {
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
            const double exact_dt = base_ok ? std::abs(timeDiffSeconds(base_obs.time, rover_obs.time))
                                            : std::numeric_limits<double>::infinity();
            bool have_aligned_base = false;
            bool used_interpolated_base = false;

            if (base_ok && exact_dt <= kExactTimeToleranceSeconds) {
                aligned_base_obs = base_obs;
                exact_base_epochs++;
                have_aligned_base = true;
            } else if (config.enable_base_interpolation && base_ok && have_lower_base &&
                       timeDiffSeconds(rover_obs.time, lower_base_obs.time) >= -kExactTimeToleranceSeconds &&
                       timeDiffSeconds(base_obs.time, rover_obs.time) >= -kExactTimeToleranceSeconds &&
                       interpolateBaseEpoch(lower_base_obs, base_obs, rover_obs.time,
                                            base_position, nav_data, aligned_base_obs)) {
                interpolated_base_epochs++;
                have_aligned_base = true;
                used_interpolated_base = true;
            }

            if (!have_aligned_base) {
                skipped_rover_epochs++;
                const Eigen::Vector3d saved_rover_pos = rover_obs.receiver_position;
                rover_ok = rover_reader.readObservationEpoch(rover_obs);
                if (rover_ok) {
                    rover_obs.receiver_position = saved_rover_pos;
                }
                processed_rover_epochs++;
                continue;
            }

            auto pos_solution = rtk_processor.processRTKEpoch(rover_obs, aligned_base_obs, nav_data);
            const libgnss::PositionSolution* last_output = solution.getLastSolution();
            const bool have_last_output = last_output != nullptr && last_output->isValid();
            const auto jump_from_last_output = [&](const libgnss::PositionSolution& candidate) {
                if (!have_last_output || !candidate.isValid()) {
                    return std::numeric_limits<double>::infinity();
                }
                return (candidate.position_ecef - last_output->position_ecef).norm();
            };
            const auto max_nonfixed_jump = [&]() {
                if (!have_last_output) {
                    return kDefaultNonFixedJumpGuardMeters;
                }
                double dt = pos_solution.time - last_output->time;
                if (!std::isfinite(dt) || dt <= 0.0) {
                    dt = 1.0;
                }
                return std::max(kDefaultNonFixedJumpGuardMeters, 25.0 * dt);
            };
            if (used_interpolated_base && pos_solution.status == libgnss::SolutionStatus::FLOAT) {
                auto spp_solution = spp_processor.processEpoch(rover_obs, nav_data);
                if (spp_solution.isValid()) {
                    const double float_vs_spp = (pos_solution.position_ecef - spp_solution.position_ecef).norm();
                    if (std::isfinite(float_vs_spp) && float_vs_spp > kDefaultFloatVsSppGuardMeters) {
                        spp_solution.status = libgnss::SolutionStatus::SPP;
                        const double float_jump = jump_from_last_output(pos_solution);
                        const double spp_jump = jump_from_last_output(spp_solution);
                        const double jump_guard = max_nonfixed_jump();
                        const bool spp_is_plausible =
                            std::isfinite(spp_jump) &&
                            spp_jump <= jump_guard &&
                            (!std::isfinite(float_jump) || spp_jump + 3.0 < float_jump);
                        if (spp_is_plausible) {
                            pos_solution = spp_solution;
                        }
                    }
                }
            }

            if (pos_solution.isValid() &&
                pos_solution.status != libgnss::SolutionStatus::FIXED) {
                const double candidate_jump = jump_from_last_output(pos_solution);
                if (std::isfinite(candidate_jump) && candidate_jump > max_nonfixed_jump()) {
                    pos_solution = libgnss::PositionSolution{};
                    pos_solution.time = rover_obs.time;
                    pos_solution.status = libgnss::SolutionStatus::NONE;
                }
            }

            if (pos_solution.isValid() &&
                pos_solution.status != libgnss::SolutionStatus::FIXED &&
                have_last_fixed_output) {
                double dt_since_fixed = pos_solution.time - last_fixed_output.time;
                if (std::isfinite(dt_since_fixed) && dt_since_fixed > 0.0 && dt_since_fixed <= 5.0) {
                    const double drift_from_fixed =
                        (pos_solution.position_ecef - last_fixed_output.position_ecef).norm();
                    const double height_from_fixed = std::abs(
                        pos_solution.position_geodetic.height -
                        last_fixed_output.position_geodetic.height);
                    const double max_fixed_drift = std::max(12.0, 15.0 * dt_since_fixed);
                    const double max_height_drift = std::max(6.0, 3.0 * dt_since_fixed);
                    if (drift_from_fixed > max_fixed_drift ||
                        height_from_fixed > max_height_drift) {
                        pos_solution = libgnss::PositionSolution{};
                        pos_solution.time = rover_obs.time;
                        pos_solution.status = libgnss::SolutionStatus::NONE;
                    }
                }
            }

            debug_writer.write(pos_solution, rtk_processor.getLastDebugTelemetry());

            if (pos_solution.isValid()) {
                solution.addSolution(pos_solution);
                valid_solution_count++;
                if (pos_solution.isFixed()) {
                    fixed_solution_count++;
                    last_fixed_output = pos_solution;
                    have_last_fixed_output = true;
                }

                if (config.verbose && (valid_solution_count <= 5 || valid_solution_count % 100 == 0)) {
                    std::cout << "epoch " << valid_solution_count
                              << " tow=" << std::fixed << std::setprecision(3) << pos_solution.time.tow
                              << " status=" << static_cast<int>(pos_solution.status)
                              << " sats=" << pos_solution.num_satellites
                              << " ratio=" << std::setprecision(2) << pos_solution.ratio
                              << std::endl;
                }
            }

            const Eigen::Vector3d saved_rover_pos = rover_obs.receiver_position;
            rover_ok = rover_reader.readObservationEpoch(rover_obs);
            if (rover_ok) {
                const bool trusted_spp_seed =
                    pos_solution.status == libgnss::SolutionStatus::SPP &&
                    pos_solution.num_satellites >= 7;
                if (pos_solution.isFixed() || trusted_spp_seed) {
                    rover_obs.receiver_position = pos_solution.position_ecef;
                } else {
                    rover_obs.receiver_position = saved_rover_pos;
                }
            }
            processed_rover_epochs++;
        }

        if (config.enable_nonfix_drift_guard &&
            rtk_config.position_mode != libgnss::RTKProcessor::RTKConfig::PositionMode::STATIC &&
            !solution.isEmpty()) {
            libgnss::rtk_validation::NonFixedDriftGuardConfig guard_config;
            guard_config.max_anchor_gap_s = config.nonfix_drift_guard_max_anchor_gap_s;
            guard_config.max_anchor_speed_mps = config.nonfix_drift_guard_max_anchor_speed_mps;
            guard_config.max_residual_m = config.nonfix_drift_guard_max_residual_m;
            guard_config.min_horizontal_residual_m =
                config.nonfix_drift_guard_min_horizontal_residual_m;
            guard_config.min_segment_epochs = config.nonfix_drift_guard_min_segment_epochs;
            guard_config.max_segment_epochs = config.nonfix_drift_guard_max_segment_epochs;
            auto guard_result = libgnss::rtk_validation::filterNonFixedStationaryDrift(
                solution.solutions,
                guard_config);
            nonfix_drift_guard_inspected_segments = guard_result.inspected_segments;
            nonfix_drift_guard_rejected_segments = guard_result.rejected_segments;
            nonfix_drift_guard_rejected_epochs = guard_result.rejected_epochs;
            solution.solutions = std::move(guard_result.solutions);
        }

        if (config.enable_spp_height_step_guard &&
            rtk_config.position_mode != libgnss::RTKProcessor::RTKConfig::PositionMode::STATIC &&
            !solution.isEmpty()) {
            libgnss::rtk_validation::SppHeightStepGuardConfig guard_config;
            guard_config.min_step_m = config.spp_height_step_guard_min_m;
            guard_config.max_rate_mps = config.spp_height_step_guard_max_rate_mps;
            auto guard_result = libgnss::rtk_validation::filterSppHeightSteps(
                solution.solutions,
                guard_config);
            spp_height_step_guard_rejected_epochs = guard_result.rejected_epochs;
            solution.solutions = std::move(guard_result.solutions);
        }

        if (config.enable_float_bridge_tail_guard &&
            rtk_config.position_mode != libgnss::RTKProcessor::RTKConfig::PositionMode::STATIC &&
            !solution.isEmpty()) {
            libgnss::rtk_validation::FloatBridgeTailGuardConfig guard_config;
            guard_config.max_anchor_gap_s = config.float_bridge_tail_guard_max_anchor_gap_s;
            guard_config.min_anchor_speed_mps =
                config.float_bridge_tail_guard_min_anchor_speed_mps;
            guard_config.max_anchor_speed_mps =
                config.float_bridge_tail_guard_max_anchor_speed_mps;
            guard_config.max_residual_m = config.float_bridge_tail_guard_max_residual_m;
            guard_config.min_segment_epochs =
                config.float_bridge_tail_guard_min_segment_epochs;
            auto guard_result = libgnss::rtk_validation::filterFloatBridgeTail(
                solution.solutions,
                guard_config);
            float_bridge_tail_guard_inspected_segments = guard_result.inspected_segments;
            float_bridge_tail_guard_rejected_segments = guard_result.rejected_segments;
            float_bridge_tail_guard_rejected_epochs = guard_result.rejected_epochs;
            solution.solutions = std::move(guard_result.solutions);
        }

        if (config.enable_fixed_bridge_burst_guard &&
            rtk_config.position_mode != libgnss::RTKProcessor::RTKConfig::PositionMode::STATIC &&
            !solution.isEmpty()) {
            libgnss::rtk_validation::FixedBridgeBurstGuardConfig guard_config;
            guard_config.max_anchor_gap_s = config.fixed_bridge_burst_guard_max_anchor_gap_s;
            guard_config.min_boundary_gap_s = config.fixed_bridge_burst_guard_min_boundary_gap_s;
            guard_config.max_residual_m = config.fixed_bridge_burst_guard_max_residual_m;
            guard_config.max_segment_epochs = config.fixed_bridge_burst_guard_max_segment_epochs;
            auto guard_result = libgnss::rtk_validation::filterFixedBridgeBursts(
                solution.solutions,
                guard_config);
            fixed_bridge_burst_guard_inspected_segments = guard_result.inspected_segments;
            fixed_bridge_burst_guard_rejected_segments = guard_result.rejected_segments;
            fixed_bridge_burst_guard_rejected_epochs = guard_result.rejected_epochs;
            solution.solutions = std::move(guard_result.solutions);
        }

        if (config.enable_kinematic_post_filter &&
            rtk_config.position_mode != libgnss::RTKProcessor::RTKConfig::PositionMode::STATIC &&
            !solution.isEmpty()) {
            // Single-epoch drop on height-step jumps. The previous "suppress
            // until N consecutive FIXED" cascade was removed because, with
            // PR #34's state-restore in validateFixedSolution and PR #35's
            // --max-pos-jump=5.0 default, wrong-FIX is already rejected at
            // the AR layer. A wrong-FIX occasionally still slipped past and
            // became `last_kept`, after which the cascade silently dropped
            // many subsequent valid epochs (truth validation: 92.1% -> 47.8%
            // coverage). Drop only the offending epoch and leave `last_kept`
            // pointing at the prior trusted anchor.
            libgnss::Solution filtered_solution;
            filtered_solution.solutions.reserve(solution.solutions.size());
            bool have_kept_fixed = false;
            const libgnss::PositionSolution* last_kept = nullptr;
            for (const auto& epoch_solution : solution.solutions) {
                if (!epoch_solution.isValid()) {
                    continue;
                }

                if (last_kept != nullptr && have_kept_fixed) {
                    double dt = epoch_solution.time - last_kept->time;
                    if (!std::isfinite(dt) || dt <= 0.0) {
                        dt = 1.0;
                    }
                    const double height_step = std::abs(
                        epoch_solution.position_geodetic.height -
                        last_kept->position_geodetic.height);
                    const double max_height_step =
                        std::max(kDefaultVerticalStepGuardMeters, 4.0 * dt);
                    if (height_step > max_height_step) {
                        // Drop this single epoch only; do not advance the
                        // anchor or trigger a multi-epoch suppression.
                        continue;
                    }
                }

                filtered_solution.addSolution(epoch_solution);
                last_kept = &filtered_solution.solutions.back();
                if (epoch_solution.isFixed()) {
                    have_kept_fixed = true;
                }
            }
            solution = std::move(filtered_solution);
        }

        if (solution.isEmpty()) {
            std::cerr << "Error: no valid solutions were produced" << std::endl;
            return 1;
        }
        if (!writeSolutions(config, solution)) {
            return 1;
        }

        Eigen::Vector3d mean_pos = Eigen::Vector3d::Zero();
        int mean_count = 0;
        for (const auto& epoch_solution : solution.solutions) {
            if (epoch_solution.isValid()) {
                mean_pos += epoch_solution.position_ecef;
                mean_count++;
            }
        }
        if (mean_count > 0) {
            mean_pos /= static_cast<double>(mean_count);
        }

        const auto stats = solution.calculateStatistics(mean_pos);
        std::cout << "\nSummary" << std::endl;
        std::cout << "  total solutions: " << solution.size() << std::endl;
        std::cout << "  valid solutions: " << stats.valid_solutions << std::endl;
        std::cout << "  fixed solutions: " << stats.fixed_solutions << std::endl;
        std::cout << "  fix rate: " << std::fixed << std::setprecision(2)
                  << stats.fix_rate * 100.0 << "%" << std::endl;
        if (rtk_config.position_mode == libgnss::RTKProcessor::RTKConfig::PositionMode::STATIC) {
            std::cout << "  RMS horizontal (self-consistency): "
                      << stats.rms_horizontal << " m" << std::endl;
            std::cout << "  RMS vertical (self-consistency): "
                      << stats.rms_vertical << " m" << std::endl;
        } else {
            std::cout << "  self-consistency metrics: omitted in dynamic mode; "
                      << "use reference-based comparison for accuracy." << std::endl;
        }
        std::cout << "  exact base epochs: " << exact_base_epochs << std::endl;
        std::cout << "  interpolated base epochs: " << interpolated_base_epochs << std::endl;
        std::cout << "  skipped rover epochs: " << skipped_rover_epochs << std::endl;
        if (rtk_config.position_mode != libgnss::RTKProcessor::RTKConfig::PositionMode::STATIC) {
            std::cout << "  non-FIX drift guard: "
                      << (config.enable_nonfix_drift_guard ? "enabled" : "disabled")
                      << " inspected_segments=" << nonfix_drift_guard_inspected_segments
                      << " rejected_segments=" << nonfix_drift_guard_rejected_segments
                      << " rejected_epochs=" << nonfix_drift_guard_rejected_epochs
                      << std::endl;
            std::cout << "  SPP height-step guard: "
                      << (config.enable_spp_height_step_guard ? "enabled" : "disabled")
                      << " rejected_epochs=" << spp_height_step_guard_rejected_epochs
                      << std::endl;
            std::cout << "  FLOAT bridge-tail guard: "
                      << (config.enable_float_bridge_tail_guard ? "enabled" : "disabled")
                      << " inspected_segments=" << float_bridge_tail_guard_inspected_segments
                      << " rejected_segments=" << float_bridge_tail_guard_rejected_segments
                      << " rejected_epochs=" << float_bridge_tail_guard_rejected_epochs
                      << std::endl;
            std::cout << "  fixed bridge-burst guard: "
                      << (config.enable_fixed_bridge_burst_guard ? "enabled" : "disabled")
                      << " inspected_segments=" << fixed_bridge_burst_guard_inspected_segments
                      << " rejected_segments=" << fixed_bridge_burst_guard_rejected_segments
                      << " rejected_epochs=" << fixed_bridge_burst_guard_rejected_epochs
                      << std::endl;
        }
        if (rtk_config.position_mode == libgnss::RTKProcessor::RTKConfig::PositionMode::STATIC &&
            rover_header.approximate_position.norm() > 0.0 && mean_count > 0) {
            std::cout << "  header vs mean diff: "
                      << (mean_pos - rover_header.approximate_position).norm() << " m" << std::endl;
        }
        std::cout << "  output written: " << config.output_pos_path << std::endl;
        if (config.write_kml) {
            std::cout << "  KML written: " << config.output_kml_path << std::endl;
        }

        return 0;
    } catch (const std::invalid_argument& e) {
        std::cerr << "Error (invalid_argument): " << e.what() << std::endl;
        return 1;
    } catch (const std::out_of_range& e) {
        std::cerr << "Error (out_of_range): " << e.what() << std::endl;
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
