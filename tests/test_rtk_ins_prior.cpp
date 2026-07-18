// Phase 1 GNSS/IMU coupling unit tests (docs/design.md): RTKProcessor::
// setExternalPositionPrior() / clearExternalPositionPrior() /
// hasExternalPositionPrior() and RTKConfig::use_external_position_prior, plus
// M1's incremental setExternalPositionTimeUpdate() path.
//
// Deliberately PUBLIC-API-ONLY (no "#define private public" trick): that
// trick relies on the Itanium C++ ABI not encoding member access in mangled
// names, which does not hold for the MSVC ABI (see tests/test_rtk_legacy.cpp
// and tests/CMakeLists.txt's guard around it) -- and this build's clang
// targets the MSVC ABI on Windows. Exercising RTKProcessor purely through
// its public surface keeps this file portable across both ABIs.

#include <gtest/gtest.h>

#include <Eigen/Dense>
#include <libgnss++/algorithms/rtk.hpp>
#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/observation.hpp>
#include <libgnss++/io/rinex.hpp>

#include <cmath>
#include <filesystem>
#include <string>

using namespace libgnss;

namespace {

std::string sourcePath(const std::string& relative_path) {
    return std::string(GNSSPP_SOURCE_DIR) + "/" + relative_path;
}

bool sourcePathExists(const std::string& relative_path) {
    return std::filesystem::exists(sourcePath(relative_path));
}

class RTKInsPriorTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Prefer the small repo-local kinematic fixture if present; fall
        // back to the PPC-Dataset tokyo/run1 rover/base/nav triple (used by
        // this feature's own gate verification, docs/design.md) so this
        // test still runs in checkouts where the small fixture isn't
        // available (data/ is entirely .gitignore'd).
        std::string rover_path = "data/rover_kinematic.obs";
        std::string base_path = "data/base_kinematic.obs";
        std::string nav_path = "data/navigation_kinematic.nav";
        if (!sourcePathExists(rover_path) || !sourcePathExists(base_path) ||
            !sourcePathExists(nav_path)) {
            rover_path = "data/PPC-Dataset/tokyo/run1/rover.obs";
            base_path = "data/PPC-Dataset/tokyo/run1/base.obs";
            nav_path = "data/PPC-Dataset/tokyo/run1/base.nav";
        }
        if (!sourcePathExists(rover_path) || !sourcePathExists(base_path) ||
            !sourcePathExists(nav_path)) {
            GTEST_SKIP() << "no kinematic RTK test data available (neither data/*_kinematic.* "
                            "nor data/PPC-Dataset/tokyo/run1)";
        }
        ASSERT_TRUE(rover_reader_.open(sourcePath(rover_path)));
        ASSERT_TRUE(rover_reader_.readHeader(rover_header_));
        ASSERT_TRUE(base_reader_.open(sourcePath(base_path)));
        ASSERT_TRUE(base_reader_.readHeader(base_header_));
        ASSERT_TRUE(nav_reader_.open(sourcePath(nav_path)));
        ASSERT_TRUE(nav_reader_.readNavigationData(nav_data_));
        ASSERT_GT(rover_header_.approximate_position.norm(), 1e6);
        ASSERT_GT(base_header_.approximate_position.norm(), 1e6);

        // Need two consecutive aligned rover/base epochs: epoch 1 to
        // initialize the filter (legacy SPP-reset path, prior-agnostic),
        // epoch 2 to actually exercise resetPositionToSPP()'s prior-vs-
        // legacy branch.
        ASSERT_TRUE(readAlignedEpoch(rover_obs1_, base_obs1_));
        ASSERT_TRUE(readAlignedEpoch(rover_obs2_, base_obs2_));
        rover_obs1_.receiver_position = rover_header_.approximate_position;

        base_position_ = base_header_.approximate_position;

        config_.position_mode = RTKProcessor::RTKConfig::PositionMode::KINEMATIC;
        config_.ar_mode = RTKProcessor::RTKConfig::AmbiguityResolutionMode::CONTINUOUS;
        config_.min_satellites_for_ar = 5;
        config_.ratio_threshold = 3.0;
    }

    bool readAlignedEpoch(ObservationData& rover_obs, ObservationData& base_obs) {
        bool rover_ok = rover_reader_.readObservationEpoch(rover_obs);
        bool base_ok = base_reader_.readObservationEpoch(base_obs);
        while (rover_ok && base_ok) {
            const double time_diff = (rover_obs.time.week - base_obs.time.week) * 604800.0 +
                                     (rover_obs.time.tow - base_obs.time.tow);
            if (std::abs(time_diff) <= 0.5) {
                return true;
            }
            if (time_diff < 0.0) {
                rover_ok = rover_reader_.readObservationEpoch(rover_obs);
            } else {
                base_ok = base_reader_.readObservationEpoch(base_obs);
            }
        }
        return false;
    }

    RTKProcessor::RTKConfig config_;
    io::RINEXReader rover_reader_;
    io::RINEXReader base_reader_;
    io::RINEXReader nav_reader_;
    io::RINEXReader::RINEXHeader rover_header_;
    io::RINEXReader::RINEXHeader base_header_;
    NavigationData nav_data_;
    Vector3d base_position_;
    ObservationData rover_obs1_;
    ObservationData base_obs1_;
    ObservationData rover_obs2_;
    ObservationData base_obs2_;
};

// Knob off (default): a prior supplied via setExternalPositionPrior() must
// be silently consumed (see setExternalPositionPrior()'s doc comment) but
// have zero effect on the actual solution -- bit-identical to a processor
// that was never given one at all.
TEST_F(RTKInsPriorTest, KnobOffPriorHasNoEffectOnSolution) {
    ASSERT_FALSE(config_.use_external_position_prior);

    RTKProcessor rtk_no_prior;
    rtk_no_prior.setRTKConfig(config_);
    rtk_no_prior.setBasePosition(base_position_);

    RTKProcessor rtk_with_prior;
    rtk_with_prior.setRTKConfig(config_);
    rtk_with_prior.setBasePosition(base_position_);

    const auto sol1_a = rtk_no_prior.processRTKEpoch(rover_obs1_, base_obs1_, nav_data_);
    const auto sol1_b = rtk_with_prior.processRTKEpoch(rover_obs1_, base_obs1_, nav_data_);
    ASSERT_TRUE(sol1_a.isValid());
    ASSERT_TRUE(sol1_b.isValid());

    // Supply an obviously-wrong prior (large offset, tiny covariance) for
    // epoch 2 on rtk_with_prior only. With the knob off it must be ignored.
    const Vector3d bogus_prior = sol1_b.position_ecef + Vector3d(500.0, 500.0, 500.0);
    const Matrix3d tiny_cov = Matrix3d::Identity() * 1.0e-6;
    rtk_with_prior.setExternalPositionPrior(bogus_prior, tiny_cov);
    ASSERT_TRUE(rtk_with_prior.hasExternalPositionPrior());

    const auto sol2_a = rtk_no_prior.processRTKEpoch(rover_obs2_, base_obs2_, nav_data_);
    const auto sol2_b = rtk_with_prior.processRTKEpoch(rover_obs2_, base_obs2_, nav_data_);

    // The prior is always consumed (cleared), whether or not the knob let
    // it actually influence anything -- see resetPositionToSPP()'s doc
    // comment in rtk.cpp.
    EXPECT_FALSE(rtk_with_prior.hasExternalPositionPrior());

    ASSERT_TRUE(sol2_a.isValid());
    ASSERT_TRUE(sol2_b.isValid());
    // Same config, same inputs, no shared mutable state between the two
    // independently-constructed processors -> deterministically identical
    // output when the knob is off, regardless of the bogus prior.
    EXPECT_TRUE(sol2_a.position_ecef.isApprox(sol2_b.position_ecef, 1e-9));
    EXPECT_EQ(sol2_a.status, sol2_b.status);
}

// Knob on: a supplied prior must be consumed (cleared) by the very next
// processRTKEpoch() call.
TEST_F(RTKInsPriorTest, KnobOnConsumesSuppliedPrior) {
    config_.use_external_position_prior = true;

    RTKProcessor rtk;
    rtk.setRTKConfig(config_);
    rtk.setBasePosition(base_position_);

    const auto sol1 = rtk.processRTKEpoch(rover_obs1_, base_obs1_, nav_data_);
    ASSERT_TRUE(sol1.isValid());
    EXPECT_FALSE(rtk.hasExternalPositionPrior())
        << "no prior was ever supplied for epoch 1; must not report one pending";

    // A plausible (small) INS-predicted offset from the just-computed
    // solution, with a tight covariance -- the kind of prior an aligned,
    // healthy ESKF would hand RTKProcessor ahead of the next epoch.
    const Vector3d plausible_prior = sol1.position_ecef + Vector3d(0.3, -0.2, 0.1);
    const Matrix3d prior_cov = Matrix3d::Identity() * 1.0e-4;
    rtk.setExternalPositionPrior(plausible_prior, prior_cov);
    ASSERT_TRUE(rtk.hasExternalPositionPrior());

    const auto sol2 = rtk.processRTKEpoch(rover_obs2_, base_obs2_, nav_data_);

    EXPECT_FALSE(rtk.hasExternalPositionPrior()) << "prior must be consumed by the epoch it was supplied for";
    EXPECT_TRUE(sol2.isValid());
}

// clearExternalPositionPrior() must discard a pending prior without it ever
// reaching resetPositionToSPP() -- the caller-health-check escape hatch
// gnss_fuse.cpp relies on (only feed a prior when the ESKF is aligned).
TEST_F(RTKInsPriorTest, ClearExternalPositionPriorDiscardsPendingPrior) {
    config_.use_external_position_prior = true;

    RTKProcessor rtk;
    rtk.setRTKConfig(config_);
    rtk.setBasePosition(base_position_);

    const auto sol1 = rtk.processRTKEpoch(rover_obs1_, base_obs1_, nav_data_);
    ASSERT_TRUE(sol1.isValid());

    rtk.setExternalPositionPrior(sol1.position_ecef + Vector3d(100.0, 0.0, 0.0),
                                 Matrix3d::Identity() * 1.0e-6);
    ASSERT_TRUE(rtk.hasExternalPositionPrior());
    rtk.clearExternalPositionPrior();
    EXPECT_FALSE(rtk.hasExternalPositionPrior());

    // Epoch 2 must fall back to the legacy reseed exactly as if no prior
    // had ever been offered.
    const auto sol2 = rtk.processRTKEpoch(rover_obs2_, base_obs2_, nav_data_);
    EXPECT_TRUE(sol2.isValid());
}

// Numerical end-to-end check that the prior is actually *used* by the
// KINEMATIC time update (not merely accepted-and-discarded): a tight,
// deliberately offset prior measurably pulls the epoch's reported solution
// away from what the same epoch produces with no prior at all.
TEST_F(RTKInsPriorTest, KnobOnPriorMeasurablyShiftsSolution) {
    RTKProcessor::RTKConfig prior_config = config_;
    prior_config.use_external_position_prior = true;

    RTKProcessor rtk_baseline;
    rtk_baseline.setRTKConfig(config_);  // knob off
    rtk_baseline.setBasePosition(base_position_);

    RTKProcessor rtk_prior;
    rtk_prior.setRTKConfig(prior_config);  // knob on
    rtk_prior.setBasePosition(base_position_);

    const auto sol1_baseline = rtk_baseline.processRTKEpoch(rover_obs1_, base_obs1_, nav_data_);
    const auto sol1_prior = rtk_prior.processRTKEpoch(rover_obs1_, base_obs1_, nav_data_);
    ASSERT_TRUE(sol1_baseline.isValid());
    ASSERT_TRUE(sol1_prior.isValid());
    // No prior supplied yet for epoch 1 -> both processors take the
    // identical legacy path and must agree.
    ASSERT_TRUE(sol1_baseline.position_ecef.isApprox(sol1_prior.position_ecef, 1e-9));

    // A 2 m offset with a tight (1 cm sigma) covariance: tight enough that
    // resetPositionToSPP()'s seed dominates the position states going into
    // this epoch's DD measurement update, so a fraction of that 2 m offset
    // should survive into the reported solution.
    const Vector3d offset_prior = sol1_baseline.position_ecef + Vector3d(2.0, 0.0, 0.0);
    const Matrix3d tight_cov = Matrix3d::Identity() * 1.0e-4;
    rtk_prior.setExternalPositionPrior(offset_prior, tight_cov);

    const auto sol2_baseline = rtk_baseline.processRTKEpoch(rover_obs2_, base_obs2_, nav_data_);
    const auto sol2_prior = rtk_prior.processRTKEpoch(rover_obs2_, base_obs2_, nav_data_);

    EXPECT_FALSE(rtk_prior.hasExternalPositionPrior());
    ASSERT_TRUE(sol2_baseline.isValid());
    ASSERT_TRUE(sol2_prior.isValid());

    const double shift_m = (sol2_prior.position_ecef - sol2_baseline.position_ecef).norm();
    EXPECT_GT(shift_m, 0.3) << "a 2 m offset prior with a 1 cm-sigma covariance should measurably "
                               "pull the epoch's solution away from the no-prior baseline; "
                               "shift was only "
                            << shift_m << " m -- resetPositionToSPP() may not be consuming the prior";
}

TEST_F(RTKInsPriorTest, TimeUpdateKnobOffIsBitIdenticalAndConsumesUpdate) {
    ASSERT_FALSE(config_.use_external_position_time_update);

    RTKProcessor baseline;
    baseline.setRTKConfig(config_);
    baseline.setBasePosition(base_position_);
    RTKProcessor candidate;
    candidate.setRTKConfig(config_);
    candidate.setBasePosition(base_position_);

    const auto sol1_a = baseline.processRTKEpoch(rover_obs1_, base_obs1_, nav_data_);
    const auto sol1_b = candidate.processRTKEpoch(rover_obs1_, base_obs1_, nav_data_);
    ASSERT_TRUE(sol1_a.isValid());
    ASSERT_TRUE(sol1_b.isValid());

    candidate.setExternalPositionTimeUpdate(
        Vector3d(100.0, -200.0, 300.0), Matrix3d::Identity() * 10.0);
    ASSERT_TRUE(candidate.hasExternalPositionTimeUpdate());
    const auto sol2_a = baseline.processRTKEpoch(rover_obs2_, base_obs2_, nav_data_);
    const auto sol2_b = candidate.processRTKEpoch(rover_obs2_, base_obs2_, nav_data_);

    EXPECT_FALSE(candidate.hasExternalPositionTimeUpdate());
    ASSERT_TRUE(sol2_a.isValid());
    ASSERT_TRUE(sol2_b.isValid());
    EXPECT_TRUE(sol2_a.position_ecef.isApprox(sol2_b.position_ecef, 1e-9));
    EXPECT_EQ(sol2_a.status, sol2_b.status);
    const auto diagnostics = candidate.getInsTimeUpdateDiagnostics();
    EXPECT_EQ(diagnostics.applied_count, 0u);
    EXPECT_EQ(diagnostics.rejected_count, 0u);
    EXPECT_FALSE(diagnostics.applied_last_epoch);
}

TEST_F(RTKInsPriorTest, TimeUpdateKnobOnAppliesOnceAndExposesFloatPosterior) {
    config_.use_external_position_time_update = true;
    config_.ins_time_update_position_q_floor_m2 = 1e-4;
    RTKProcessor rtk;
    rtk.setRTKConfig(config_);
    rtk.setBasePosition(base_position_);

    const auto sol1 = rtk.processRTKEpoch(rover_obs1_, base_obs1_, nav_data_);
    ASSERT_TRUE(sol1.isValid());
    Vector3d float_position;
    Matrix3d float_covariance;
    ASSERT_TRUE(rtk.getFloatPosteriorPosition(float_position, float_covariance));
    EXPECT_TRUE(float_position.allFinite());
    EXPECT_TRUE(float_covariance.allFinite());

    rtk.setExternalPositionTimeUpdate(
        Vector3d(0.1, -0.05, 0.02), Matrix3d::Identity() * 1e-3);
    const auto sol2 = rtk.processRTKEpoch(rover_obs2_, base_obs2_, nav_data_);
    ASSERT_TRUE(sol2.isValid());
    EXPECT_FALSE(rtk.hasExternalPositionTimeUpdate());
    const auto diagnostics = rtk.getInsTimeUpdateDiagnostics();
    EXPECT_EQ(diagnostics.applied_count, 1u);
    EXPECT_EQ(diagnostics.rejected_count, 0u);
    EXPECT_TRUE(diagnostics.applied_last_epoch);
}

TEST_F(RTKInsPriorTest, InvalidTimeUpdateFallsBackAndIsCounted) {
    config_.use_external_position_time_update = true;
    RTKProcessor rtk;
    rtk.setRTKConfig(config_);
    rtk.setBasePosition(base_position_);
    ASSERT_TRUE(rtk.processRTKEpoch(rover_obs1_, base_obs1_, nav_data_).isValid());

    Matrix3d invalid_noise = Matrix3d::Zero();
    invalid_noise(0, 0) = -1.0;
    rtk.setExternalPositionTimeUpdate(Vector3d::Zero(), invalid_noise);
    const auto sol2 = rtk.processRTKEpoch(rover_obs2_, base_obs2_, nav_data_);

    EXPECT_TRUE(sol2.isValid());
    EXPECT_FALSE(rtk.hasExternalPositionTimeUpdate());
    const auto diagnostics = rtk.getInsTimeUpdateDiagnostics();
    EXPECT_EQ(diagnostics.applied_count, 0u);
    EXPECT_EQ(diagnostics.rejected_count, 1u);
    EXPECT_FALSE(diagnostics.applied_last_epoch);
}

TEST_F(RTKInsPriorTest, ClearTimeUpdateDiscardsPendingIncrement) {
    RTKProcessor rtk;
    rtk.setExternalPositionTimeUpdate(Vector3d::Ones(), Matrix3d::Identity());
    ASSERT_TRUE(rtk.hasExternalPositionTimeUpdate());
    rtk.clearExternalPositionTimeUpdate();
    EXPECT_FALSE(rtk.hasExternalPositionTimeUpdate());
}

}  // namespace
