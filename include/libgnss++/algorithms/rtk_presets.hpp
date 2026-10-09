#pragma once

/**
 * @file rtk_presets.hpp
 * @brief Library RTK tuning presets for RTKProcessor::RTKConfig.
 *
 * The numeric tables mirror the app helper `applyRtkConfigPreset()` in
 * apps/native/rtk_base_epoch_align.hpp (survey, low-cost, odaiba, moving-base);
 * the app header is left untouched and tests/test_rtk_presets.cpp asserts the
 * two stay equal field by field. In addition, low-cost and odaiba carry the
 * `gnss solve` product default max_baseline_length = 20000 m
 * (SolveConfig::max_baseline_length_m), which the app helper does not set.
 *
 * See docs/online_rtk_product_config_v1.md.
 */

#include <string>

#include <libgnss++/algorithms/rtk.hpp>

namespace libgnss {

/** Apply the named preset to `config`. "" and "none" are accepted and leave
 * the config untouched. An unknown name returns false and leaves the config
 * untouched. */
bool applyRtkPreset(RTKProcessor::RTKConfig& config, const std::string& name);

} // namespace libgnss
