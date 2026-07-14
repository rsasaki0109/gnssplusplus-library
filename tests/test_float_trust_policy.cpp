#include <gtest/gtest.h>

#include <libgnss++/algorithms/float_trust_policy.hpp>

#include <cmath>
#include <limits>

namespace libgnss {
namespace float_trust_policy {
namespace {

constexpr double kLegacyVar = 900.0;

// --------------------------------------------------------------------------
// hasTrustLapsed
// --------------------------------------------------------------------------

TEST(FloatTrustPolicyTest, LapsedWhenNeverTrusted) {
    EXPECT_TRUE(hasTrustLapsed(/*has_last_trusted=*/false, /*trust_refreshed_last_epoch=*/false));
    // has_last_trusted false should still report lapsed even if the second
    // flag is (nonsensically) true -- "never trusted" always wins.
    EXPECT_TRUE(hasTrustLapsed(false, true));
}

TEST(FloatTrustPolicyTest, LapsedWhenPreviousEpochDidNotRefresh) {
    EXPECT_TRUE(hasTrustLapsed(/*has_last_trusted=*/true, /*trust_refreshed_last_epoch=*/false));
}

TEST(FloatTrustPolicyTest, NotLapsedWhenPreviousEpochRefreshed) {
    EXPECT_FALSE(hasTrustLapsed(/*has_last_trusted=*/true, /*trust_refreshed_last_epoch=*/true));
}

// --------------------------------------------------------------------------
// growPositionVarianceCvPredict (CV_PREDICT)
// --------------------------------------------------------------------------

TEST(FloatTrustPolicyTest, CvPredictGrowsLinearlyWithDt) {
    // prev=10, qpos=5 m^2/s, dt=0.2s -> 10 + 5*0.2 = 11
    EXPECT_NEAR(growPositionVarianceCvPredict(10.0, 5.0, 0.2, kLegacyVar), 11.0, 1e-9);
}

TEST(FloatTrustPolicyTest, CvPredictZeroQposMeansNoGrowth) {
    EXPECT_NEAR(growPositionVarianceCvPredict(42.0, 0.0, 5.0, kLegacyVar), 42.0, 1e-9);
}

TEST(FloatTrustPolicyTest, CvPredictCapsAtLegacyValue) {
    // Large qpos/dt should saturate at the legacy cap, not exceed it.
    EXPECT_DOUBLE_EQ(growPositionVarianceCvPredict(100.0, 1000.0, 100.0, kLegacyVar), kLegacyVar);
}

TEST(FloatTrustPolicyTest, CvPredictConvergesToLegacyUnderLongDrought) {
    // Repeated application (simulating consecutive lapsed epochs) must
    // monotonically approach, and eventually sit at, the legacy cap --
    // i.e. bounded divergence from legacy by construction.
    double var = 1.0;
    for (int i = 0; i < 1000; ++i) {
        const double next = growPositionVarianceCvPredict(var, 50.0, 0.2, kLegacyVar);
        EXPECT_GE(next, var - 1e-9);  // monotonically non-decreasing
        EXPECT_LE(next, kLegacyVar + 1e-9);
        var = next;
    }
    EXPECT_NEAR(var, kLegacyVar, 1e-6);
}

TEST(FloatTrustPolicyTest, CvPredictClampsNegativeAndNonFiniteInputsDefensively) {
    // Negative previous variance treated as 0.
    EXPECT_NEAR(growPositionVarianceCvPredict(-5.0, 5.0, 1.0, kLegacyVar), 5.0, 1e-9);
    // Negative qpos treated as 0 (no growth).
    EXPECT_NEAR(growPositionVarianceCvPredict(20.0, -5.0, 1.0, kLegacyVar), 20.0, 1e-9);
    // Negative dt treated as 0.
    EXPECT_NEAR(growPositionVarianceCvPredict(20.0, 5.0, -1.0, kLegacyVar), 20.0, 1e-9);
    // Non-finite previous variance falls back to the legacy cap (safe default).
    const double nan_prev = std::numeric_limits<double>::quiet_NaN();
    EXPECT_DOUBLE_EQ(growPositionVarianceCvPredict(nan_prev, 5.0, 1.0, kLegacyVar), kLegacyVar);
}

// --------------------------------------------------------------------------
// scaledResetPositionVariance (SCALED_RESET)
// --------------------------------------------------------------------------

TEST(FloatTrustPolicyTest, ScaledResetIsBaseAtZeroTrustAge) {
    EXPECT_NEAR(scaledResetPositionVariance(25.0, 10.0, 0.0, kLegacyVar), 25.0, 1e-9);
}

TEST(FloatTrustPolicyTest, ScaledResetGrowsQuadraticallyWithTrustAge) {
    // var = min(900, 25 + 10 * 2^2) = min(900, 65) = 65
    EXPECT_NEAR(scaledResetPositionVariance(25.0, 10.0, 2.0, kLegacyVar), 65.0, 1e-9);
    // Doubling dt should more than double the growth term (quadratic, not linear).
    const double growth_at_2s = scaledResetPositionVariance(25.0, 10.0, 2.0, kLegacyVar) - 25.0;
    const double growth_at_4s = scaledResetPositionVariance(25.0, 10.0, 4.0, kLegacyVar) - 25.0;
    EXPECT_NEAR(growth_at_4s, growth_at_2s * 4.0, 1e-9);
}

TEST(FloatTrustPolicyTest, ScaledResetCapsAtLegacyValue) {
    EXPECT_DOUBLE_EQ(scaledResetPositionVariance(25.0, 10.0, 1000.0, kLegacyVar), kLegacyVar);
}

TEST(FloatTrustPolicyTest, ScaledResetClampsNegativeInputsDefensively) {
    EXPECT_NEAR(scaledResetPositionVariance(-5.0, 10.0, 1.0, kLegacyVar), 10.0, 1e-9);
    EXPECT_NEAR(scaledResetPositionVariance(25.0, -10.0, 1.0, kLegacyVar), 25.0, 1e-9);
    EXPECT_NEAR(scaledResetPositionVariance(25.0, 10.0, -1.0, kLegacyVar), 25.0, 1e-9);
}

// --------------------------------------------------------------------------
// estimateVelocityFromTrustedDeltas
// --------------------------------------------------------------------------

TEST(FloatTrustPolicyTest, VelocityFromTrustedDeltasBasicCase) {
    const Eigen::Vector3d newer(10.0, 0.0, 0.0);
    const Eigen::Vector3d older(0.0, 0.0, 0.0);
    const Eigen::Vector3d v = estimateVelocityFromTrustedDeltas(newer, older, 2.0, 10.0);
    EXPECT_NEAR(v.x(), 5.0, 1e-9);
    EXPECT_NEAR(v.y(), 0.0, 1e-9);
    EXPECT_NEAR(v.z(), 0.0, 1e-9);
}

TEST(FloatTrustPolicyTest, VelocityIsZeroForNonPositiveDt) {
    const Eigen::Vector3d newer(10.0, 0.0, 0.0), older(0.0, 0.0, 0.0);
    EXPECT_TRUE(estimateVelocityFromTrustedDeltas(newer, older, 0.0, 10.0).isZero());
    EXPECT_TRUE(estimateVelocityFromTrustedDeltas(newer, older, -1.0, 10.0).isZero());
}

TEST(FloatTrustPolicyTest, VelocityIsZeroWhenDeltaTooOld) {
    const Eigen::Vector3d newer(10.0, 0.0, 0.0), older(0.0, 0.0, 0.0);
    // dt=20s > max_dt_s=10s -> samples too far apart, don't trust the delta.
    EXPECT_TRUE(estimateVelocityFromTrustedDeltas(newer, older, 20.0, 10.0).isZero());
}

TEST(FloatTrustPolicyTest, VelocityIsZeroForNonFinitePositions) {
    const Eigen::Vector3d bad(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0);
    const Eigen::Vector3d ok(0.0, 0.0, 0.0);
    EXPECT_TRUE(estimateVelocityFromTrustedDeltas(bad, ok, 1.0, 10.0).isZero());
    EXPECT_TRUE(estimateVelocityFromTrustedDeltas(ok, bad, 1.0, 10.0).isZero());
}

// --------------------------------------------------------------------------
// predictPositionConstantVelocity
// --------------------------------------------------------------------------

TEST(FloatTrustPolicyTest, PredictAdvancesPositionByVelocityTimesDt) {
    const Eigen::Vector3d pos(100.0, 200.0, 300.0);
    const Eigen::Vector3d vel(1.0, -2.0, 0.5);
    const Eigen::Vector3d predicted = predictPositionConstantVelocity(pos, vel, 0.2);
    EXPECT_NEAR(predicted.x(), 100.2, 1e-9);
    EXPECT_NEAR(predicted.y(), 199.6, 1e-9);
    EXPECT_NEAR(predicted.z(), 300.1, 1e-9);
}

TEST(FloatTrustPolicyTest, PredictWithZeroVelocityHoldsPosition) {
    const Eigen::Vector3d pos(1.0, 2.0, 3.0);
    const Eigen::Vector3d predicted = predictPositionConstantVelocity(pos, Eigen::Vector3d::Zero(), 5.0);
    EXPECT_TRUE(predicted.isApprox(pos));
}

TEST(FloatTrustPolicyTest, PredictHoldsPositionOnNonFiniteVelocity) {
    const Eigen::Vector3d pos(1.0, 2.0, 3.0);
    const Eigen::Vector3d bad_vel(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0);
    const Eigen::Vector3d predicted = predictPositionConstantVelocity(pos, bad_vel, 1.0);
    EXPECT_TRUE(predicted.isApprox(pos));
}

// --------------------------------------------------------------------------
// lapseGateExceeded (WP10: LAPSE_GATED)
// --------------------------------------------------------------------------

TEST(FloatTrustPolicyTest, LapseGateBelowGateStaysLegacy) {
    // A 3s lapse against a 5s gate must NOT yet trigger the scaled law.
    EXPECT_FALSE(lapseGateExceeded(3.0, 5.0));
    EXPECT_FALSE(lapseGateExceeded(4.999, 5.0));
}

TEST(FloatTrustPolicyTest, LapseGateBoundaryIsInclusive) {
    // Exactly at the gate: "exceeds" is treated as >=, i.e. the scaled law
    // engages the instant the lapse reaches the configured duration.
    EXPECT_TRUE(lapseGateExceeded(5.0, 5.0));
}

TEST(FloatTrustPolicyTest, LapseGateAboveGateSwitchesToScaledLaw) {
    EXPECT_TRUE(lapseGateExceeded(5.001, 5.0));
    EXPECT_TRUE(lapseGateExceeded(92.0, 5.0));  // the canyon's ~92s drought
}

TEST(FloatTrustPolicyTest, LapseGateZeroMeansAlwaysUseScaledLaw) {
    // gate=0 is the degenerate "always scaled, never legacy, once lapsed"
    // case -- useful as a sanity endpoint of a gate sweep.
    EXPECT_TRUE(lapseGateExceeded(0.0, 0.0));
    EXPECT_TRUE(lapseGateExceeded(0.001, 0.0));
}

TEST(FloatTrustPolicyTest, LapseGateVeryLargeGateNeverTriggersOnRealisticLapses) {
    // This is the bit-identity check's mechanism: a gate value larger than
    // any lapse that actually occurs in a run means lapseGateExceeded()
    // never returns true, so resetPositionToSPP()'s caller never takes the
    // scaled-reset branch -- every epoch falls through to the unmodified
    // legacy fallback code path.
    EXPECT_FALSE(lapseGateExceeded(92.0, 1.0e6));
}

TEST(FloatTrustPolicyTest, LapseGateClampsNegativeAndNonFiniteInputsDefensively) {
    // Negative/non-finite lapse duration clamped to 0 (treated as "no
    // lapse yet"); combined with any non-negative gate, that means "not
    // exceeded" unless the gate is also clamped to <= 0.
    EXPECT_FALSE(lapseGateExceeded(-1.0, 5.0));
    EXPECT_FALSE(lapseGateExceeded(std::numeric_limits<double>::quiet_NaN(), 5.0));
    // Negative/non-finite gate clamped to 0 -> "trigger immediately" once
    // dt_since_trust is itself clamped to >= 0.
    EXPECT_TRUE(lapseGateExceeded(0.5, -1.0));
    EXPECT_TRUE(lapseGateExceeded(0.5, std::numeric_limits<double>::quiet_NaN()));
}

TEST(FloatTrustPolicyTest, LapseGateComposesWithScaledResetLawAboveGate) {
    // Once the gate is exceeded, the resulting variance must match
    // scaledResetPositionVariance()'s formula exactly (this is how
    // resetPositionToSPP() wires the two functions together).
    const double dt = 10.0;
    const double gate = 5.0;
    const double qpos = 0.1;
    ASSERT_TRUE(lapseGateExceeded(dt, gate));
    const double var = scaledResetPositionVariance(25.0, qpos, dt, kLegacyVar);
    // 25 + 0.1 * 10^2 = 35
    EXPECT_NEAR(var, 35.0, 1e-9);
}

}  // namespace
}  // namespace float_trust_policy
}  // namespace libgnss
