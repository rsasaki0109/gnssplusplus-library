#include <gtest/gtest.h>

#include <libgnss++/algorithms/rtk_presets.hpp>

#include "../apps/native/rtk_base_epoch_align.hpp"

#include <string>
#include <vector>

using namespace libgnss;

namespace {
using Cfg = RTKProcessor::RTKConfig;

// Every field the app helper applyRtkConfigPreset() can touch, plus the one
// library-only field (max_baseline_length). A field is compared after both
// functions ran on default configs, so untouched fields compare equal too.
void expectAppFieldsEqual(const Cfg& a, const Cfg& b, const std::string& name) {
    SCOPED_TRACE(name);
    EXPECT_EQ(a.ratio_threshold, b.ratio_threshold);
    EXPECT_EQ(a.ambiguity_ratio_threshold, b.ambiguity_ratio_threshold);
    EXPECT_EQ(a.enable_ar_filter, b.enable_ar_filter);
    EXPECT_EQ(a.ar_filter_margin, b.ar_filter_margin);
    EXPECT_EQ(a.min_satellites_for_ar, b.min_satellites_for_ar);
    EXPECT_EQ(a.min_hold_count, b.min_hold_count);
    EXPECT_EQ(a.hold_ambiguity_ratio_threshold, b.hold_ambiguity_ratio_threshold);
    EXPECT_EQ(a.max_position_jump_rate_mps, b.max_position_jump_rate_mps);
    EXPECT_EQ(a.max_position_jump_min_m, b.max_position_jump_min_m);
    EXPECT_EQ(a.min_full_ratio_for_subset_ar, b.min_full_ratio_for_subset_ar);
    EXPECT_EQ(a.max_float_prefit_residual_rms_m, b.max_float_prefit_residual_rms_m);
    EXPECT_EQ(a.max_float_prefit_residual_max_m, b.max_float_prefit_residual_max_m);
    EXPECT_EQ(a.max_float_prefit_residual_reset_streak, b.max_float_prefit_residual_reset_streak);
    EXPECT_EQ(a.enable_wide_lane_ar, b.enable_wide_lane_ar);
    EXPECT_EQ(a.wide_lane_acceptance_threshold, b.wide_lane_acceptance_threshold);
    EXPECT_EQ(a.wide_lane_min_arc_samples, b.wide_lane_min_arc_samples);
}
}  // namespace

TEST(RtkPresetsTest, LibraryTablesEqualAppHelperFieldByField) {
    for (const std::string name : {"none", "", "survey", "low-cost", "odaiba", "moving-base"}) {
        Cfg app;
        Cfg lib;
        ASSERT_TRUE(libgnss_apps::applyRtkConfigPreset(name, app)) << name;
        ASSERT_TRUE(applyRtkPreset(lib, name)) << name;
        expectAppFieldsEqual(app, lib, name);
    }
}

TEST(RtkPresetsTest, ProductMaxBaselineLengthOnlyForLowCostFamily) {
    // gnss solve defaults max_baseline_length_m to 20000 (SolveConfig); the
    // app helper does not set it, the library presets do for low-cost/odaiba.
    const Cfg defaults;
    for (const std::string name : {"low-cost", "odaiba"}) {
        Cfg config;
        ASSERT_TRUE(applyRtkPreset(config, name));
        EXPECT_EQ(config.max_baseline_length, 20000.0) << name;
    }
    for (const std::string name : {"", "none", "survey", "moving-base"}) {
        Cfg config;
        ASSERT_TRUE(applyRtkPreset(config, name));
        EXPECT_EQ(config.max_baseline_length, defaults.max_baseline_length) << name;
    }
}

TEST(RtkPresetsTest, LowCostHardCodedTable) {
    Cfg config;
    ASSERT_TRUE(applyRtkPreset(config, "low-cost"));
    EXPECT_EQ(config.ratio_threshold, 3.0);
    EXPECT_EQ(config.ambiguity_ratio_threshold, 3.0);
    EXPECT_TRUE(config.enable_ar_filter);
    EXPECT_EQ(config.ar_filter_margin, 0.35);
    EXPECT_EQ(config.min_satellites_for_ar, 6);
    EXPECT_EQ(config.min_hold_count, 8);
    EXPECT_EQ(config.hold_ambiguity_ratio_threshold, 2.5);
    EXPECT_EQ(config.max_position_jump_rate_mps, 30.0);
    EXPECT_EQ(config.max_position_jump_min_m, 5.0);
    EXPECT_EQ(config.min_full_ratio_for_subset_ar, 1.5);
    EXPECT_EQ(config.max_float_prefit_residual_rms_m, 4.0);
    EXPECT_EQ(config.max_float_prefit_residual_max_m, 10.0);
    EXPECT_EQ(config.max_float_prefit_residual_reset_streak, 5);
    EXPECT_EQ(config.max_baseline_length, 20000.0);
}

TEST(RtkPresetsTest, NoPresetTouchesTheOptInHoldFixUnchangedSetOption) {
    // hold_fix_unchanged_set_only is an explicit opt-in (--hold-fix-unchanged-set):
    // neither the library table nor the app helper sets or clears it.
    for (const std::string name : {"none", "", "survey", "low-cost", "odaiba", "moving-base"}) {
        for (const bool initial : {false, true}) {
            Cfg lib;
            Cfg app;
            lib.hold_fix_unchanged_set_only = initial;
            app.hold_fix_unchanged_set_only = initial;
            ASSERT_TRUE(applyRtkPreset(lib, name)) << name;
            ASSERT_TRUE(libgnss_apps::applyRtkConfigPreset(name, app)) << name;
            EXPECT_EQ(lib.hold_fix_unchanged_set_only, initial) << name;
            EXPECT_EQ(app.hold_fix_unchanged_set_only, initial) << name;
        }
    }
}

TEST(RtkPresetsTest, NoPresetTouchesTheOptInHoldFixMinMatchedPairsOption) {
    // hold_fix_min_matched_pairs is an explicit opt-in (--hold-fix-min-matched-pairs):
    // every preset leaves the legacy floor of 4 in place, and neither the library
    // table nor the app helper sets or clears a raised value.
    EXPECT_EQ(Cfg().hold_fix_min_matched_pairs, 4);
    for (const std::string name : {"none", "", "survey", "low-cost", "odaiba", "moving-base"}) {
        for (const int initial : {4, 12}) {
            Cfg lib;
            Cfg app;
            lib.hold_fix_min_matched_pairs = initial;
            app.hold_fix_min_matched_pairs = initial;
            ASSERT_TRUE(applyRtkPreset(lib, name)) << name;
            ASSERT_TRUE(libgnss_apps::applyRtkConfigPreset(name, app)) << name;
            EXPECT_EQ(lib.hold_fix_min_matched_pairs, initial) << name;
            EXPECT_EQ(app.hold_fix_min_matched_pairs, initial) << name;
        }
    }
}

TEST(RtkPresetsTest, NoPresetTouchesTheOptInMinFixSatellitesOption) {
    // min_fix_satellites is an explicit opt-in (--min-fix-satellites): every preset
    // leaves it off (0), and neither the library table nor the app helper sets or
    // clears a raised value.
    EXPECT_EQ(Cfg().min_fix_satellites, 0);
    for (const std::string name : {"none", "", "survey", "low-cost", "odaiba", "moving-base"}) {
        for (const int initial : {0, 6}) {
            Cfg lib;
            Cfg app;
            lib.min_fix_satellites = initial;
            app.min_fix_satellites = initial;
            ASSERT_TRUE(applyRtkPreset(lib, name)) << name;
            ASSERT_TRUE(libgnss_apps::applyRtkConfigPreset(name, app)) << name;
            EXPECT_EQ(lib.min_fix_satellites, initial) << name;
            EXPECT_EQ(app.min_fix_satellites, initial) << name;
        }
    }
}

TEST(RtkPresetsTest, UnknownPresetReturnsFalseAndLeavesConfigUntouched) {
    Cfg config;
    config.ratio_threshold = 7.5;
    config.max_baseline_length = 123.0;
    const Cfg before = config;
    for (const std::string name : {"bogus", "Low-Cost", "low_cost", " low-cost"}) {
        EXPECT_FALSE(applyRtkPreset(config, name)) << name;
        expectAppFieldsEqual(before, config, name);
        EXPECT_EQ(config.max_baseline_length, before.max_baseline_length);
    }
}
