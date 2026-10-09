#include <libgnss++/algorithms/rtk_presets.hpp>

namespace libgnss {

bool applyRtkPreset(RTKProcessor::RTKConfig& rtk_config, const std::string& name) {
    if (name.empty() || name == "none") {
        return true;
    }
    if (name == "survey") {
        rtk_config.ratio_threshold = 3.0;
        rtk_config.ambiguity_ratio_threshold = 3.0;
        rtk_config.enable_ar_filter = false;
        rtk_config.ar_filter_margin = 0.25;
        rtk_config.min_satellites_for_ar = 5;
        rtk_config.min_hold_count = 5;
        rtk_config.hold_ambiguity_ratio_threshold = 2.0;
        return true;
    }
    if (name == "low-cost" || name == "odaiba") {
        // odaiba = low-cost + arc-smoothed MW wide-lane AR.
        if (name == "odaiba") {
            rtk_config.enable_wide_lane_ar = true;
            rtk_config.wide_lane_acceptance_threshold = 0.12;
            rtk_config.wide_lane_min_arc_samples = 100;
        }
        rtk_config.ratio_threshold = 3.0;
        rtk_config.ambiguity_ratio_threshold = 3.0;
        rtk_config.enable_ar_filter = true;
        rtk_config.ar_filter_margin = 0.35;
        rtk_config.min_satellites_for_ar = 6;
        rtk_config.min_hold_count = 8;
        rtk_config.hold_ambiguity_ratio_threshold = 2.5;
        rtk_config.max_position_jump_rate_mps = 30.0;
        rtk_config.max_position_jump_min_m = 5.0;
        rtk_config.min_full_ratio_for_subset_ar = 1.5;
        rtk_config.max_float_prefit_residual_rms_m = 4.0;
        rtk_config.max_float_prefit_residual_max_m = 10.0;
        rtk_config.max_float_prefit_residual_reset_streak = 5;
        // gnss solve product default (SolveConfig::max_baseline_length_m).
        rtk_config.max_baseline_length = 20000.0;
        return true;
    }
    if (name == "moving-base") {
        rtk_config.ratio_threshold = 2.8;
        rtk_config.ambiguity_ratio_threshold = 2.8;
        rtk_config.enable_ar_filter = true;
        rtk_config.ar_filter_margin = 0.20;
        rtk_config.min_satellites_for_ar = 6;
        rtk_config.min_hold_count = 8;
        rtk_config.hold_ambiguity_ratio_threshold = 2.4;
        return true;
    }
    return false;
}

} // namespace libgnss
