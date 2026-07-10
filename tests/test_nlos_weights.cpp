#include <gtest/gtest.h>

#include <libgnss++/algorithms/nlos_weights.hpp>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

namespace libgnss {
namespace nlos_weights {
namespace {

std::string writeTempCsv(const std::string& contents, const std::string& suffix) {
    const auto path = std::filesystem::temp_directory_path() /
        ("nlos_weights_test_" + suffix + ".csv");
    std::ofstream out(path);
    out << contents;
    out.close();
    return path.string();
}

// --------------------------------------------------------------------------
// loadNlosWeightsCsv
// --------------------------------------------------------------------------

TEST(NlosWeightsTest, LoadsNativeTowSatLosProbContract) {
    const std::string path = writeTempCsv(
        "tow,sat,los_prob\n"
        "100.000,G01,1.0\n"
        "100.000,G02,0.0\n"
        "100.500,G01,0.25\n",
        "native");

    const auto table = loadNlosWeightsCsv(path);
    ASSERT_FALSE(table.empty());
    EXPECT_DOUBLE_EQ(lookupLosProb(table, 100.0, "G01", 0.1), 1.0);
    EXPECT_DOUBLE_EQ(lookupLosProb(table, 100.0, "G02", 0.1), 0.0);
    EXPECT_DOUBLE_EQ(lookupLosProb(table, 100.5, "G01", 0.1), 0.25);

    std::filesystem::remove(path);
}

TEST(NlosWeightsTest, LoadsExtendedIsLosBooleanContract) {
    // Matches build_per_epoch_nlos_csv.py's actual header/row shape.
    const std::string path = writeTempCsv(
        "tow,epoch_idx,prn,is_los,system,svid,elevation_deg,receiver_source,receiver_time_delta_s\n"
        "187470.000,0,C11,1,C,11,50.777,reference,0.000\n"
        "187470.000,0,C21,0,C,21,15.475,reference,0.000\n",
        "extended");

    const auto table = loadNlosWeightsCsv(path);
    ASSERT_FALSE(table.empty());
    EXPECT_DOUBLE_EQ(lookupLosProb(table, 187470.0, "C11", 0.01), 1.0);
    EXPECT_DOUBLE_EQ(lookupLosProb(table, 187470.0, "C21", 0.01), 0.0);

    std::filesystem::remove(path);
}

TEST(NlosWeightsTest, MissingFileThrows) {
    EXPECT_THROW(loadNlosWeightsCsv("/no/such/path/nlos_weights.csv"), std::runtime_error);
}

TEST(NlosWeightsTest, MissingRequiredColumnsThrows) {
    const std::string path = writeTempCsv("a,b,c\n1,2,3\n", "bad_header");
    EXPECT_THROW(loadNlosWeightsCsv(path), std::runtime_error);
    std::filesystem::remove(path);
}

TEST(NlosWeightsTest, MalformedRowsAreSkippedNotFatal) {
    const std::string path = writeTempCsv(
        "tow,sat,los_prob\n"
        "not_a_number,G01,1.0\n"
        "100.0,G02,not_a_number\n"
        "100.0,G03,0.5\n",
        "malformed");
    const auto table = loadNlosWeightsCsv(path);
    EXPECT_DOUBLE_EQ(lookupLosProb(table, 100.0, "G03", 0.1), 0.5);
    // Rows that failed to parse leave that (tow, sat) absent -> default 1.0.
    EXPECT_DOUBLE_EQ(lookupLosProb(table, 100.0, "G02", 0.1), 1.0);
    std::filesystem::remove(path);
}

// --------------------------------------------------------------------------
// lookupLosProb
// --------------------------------------------------------------------------

TEST(NlosWeightsTest, EmptyTableAlwaysReturnsLos) {
    NlosWeightTable table;
    EXPECT_DOUBLE_EQ(lookupLosProb(table, 123.0, "G01", 0.1), 1.0);
}

TEST(NlosWeightsTest, UnknownSatelliteDefaultsToLos) {
    NlosWeightTable table;
    table.by_tow[100.0]["G01"] = 0.0;
    EXPECT_DOUBLE_EQ(lookupLosProb(table, 100.0, "G99", 0.1), 1.0);
}

TEST(NlosWeightsTest, ToleranceRejectsFarTow) {
    NlosWeightTable table;
    table.by_tow[100.0]["G01"] = 0.0;
    EXPECT_DOUBLE_EQ(lookupLosProb(table, 100.5, "G01", 0.1), 1.0);
    EXPECT_DOUBLE_EQ(lookupLosProb(table, 100.05, "G01", 0.1), 0.0);
}

TEST(NlosWeightsTest, PicksNearestTowAmongCandidates) {
    NlosWeightTable table;
    table.by_tow[100.0]["G01"] = 0.1;
    table.by_tow[100.2]["G01"] = 0.9;
    // 100.11 is closer to 100.2 than to 100.0.
    EXPECT_DOUBLE_EQ(lookupLosProb(table, 100.11, "G01", 0.5), 0.9);
    // 100.05 is closer to 100.0.
    EXPECT_DOUBLE_EQ(lookupLosProb(table, 100.05, "G01", 0.5), 0.1);
}

// --------------------------------------------------------------------------
// nlosVarianceInflationFactor
// --------------------------------------------------------------------------

TEST(NlosWeightsTest, OffModeIsAlwaysNoOp) {
    EXPECT_DOUBLE_EQ(
        nlosVarianceInflationFactor(0.0, NlosWeightMode::OFF, 0.05, 0.5, 3.0), 1.0);
    EXPECT_DOUBLE_EQ(
        nlosVarianceInflationFactor(1.0, NlosWeightMode::OFF, 0.05, 0.5, 3.0), 1.0);
    EXPECT_DOUBLE_EQ(
        nlosVarianceInflationFactor(std::nan(""), NlosWeightMode::OFF, 0.05, 0.5, 3.0), 1.0);
}

TEST(NlosWeightsTest, TwoTierInflatesOnlyBelowThreshold) {
    EXPECT_DOUBLE_EQ(
        nlosVarianceInflationFactor(1.0, NlosWeightMode::TWO_TIER, 0.05, 0.5, 3.0), 1.0);
    EXPECT_DOUBLE_EQ(
        nlosVarianceInflationFactor(0.5, NlosWeightMode::TWO_TIER, 0.05, 0.5, 3.0), 1.0);
    EXPECT_DOUBLE_EQ(
        nlosVarianceInflationFactor(0.0, NlosWeightMode::TWO_TIER, 0.05, 0.5, 3.0), 9.0);
    EXPECT_DOUBLE_EQ(
        nlosVarianceInflationFactor(0.49, NlosWeightMode::TWO_TIER, 0.05, 0.5, 5.0), 25.0);
}

TEST(NlosWeightsTest, TwoTierClampsSubUnitInflationToOne) {
    EXPECT_DOUBLE_EQ(
        nlosVarianceInflationFactor(0.0, NlosWeightMode::TWO_TIER, 0.05, 0.5, 0.5), 1.0);
}

TEST(NlosWeightsTest, ContinuousMatchesOneOverMaxLosProbFloor) {
    EXPECT_NEAR(
        nlosVarianceInflationFactor(1.0, NlosWeightMode::CONTINUOUS, 0.05, 0.5, 3.0), 1.0, 1e-12);
    EXPECT_NEAR(
        nlosVarianceInflationFactor(0.0, NlosWeightMode::CONTINUOUS, 0.05, 0.5, 3.0), 20.0, 1e-9);
    EXPECT_NEAR(
        nlosVarianceInflationFactor(0.5, NlosWeightMode::CONTINUOUS, 0.05, 0.5, 3.0), 2.0, 1e-9);
}

TEST(NlosWeightsTest, ContinuousFloorClampedToSaneRange) {
    // A non-finite / out-of-range floor should fall back to a safe default
    // rather than dividing by zero or a negative number.
    const double factor =
        nlosVarianceInflationFactor(0.0, NlosWeightMode::CONTINUOUS, std::nan(""), 0.5, 3.0);
    EXPECT_TRUE(std::isfinite(factor));
    EXPECT_GT(factor, 1.0);
}

TEST(NlosWeightsTest, ExcludeModeIsAlwaysNoOpForInflationFactor) {
    // EXCLUDE mode drops satellites entirely (nlosShouldExclude), it never
    // inflates sigma -- nlosVarianceInflationFactor should treat it exactly
    // like OFF for any surviving (non-excluded) satellite.
    EXPECT_DOUBLE_EQ(
        nlosVarianceInflationFactor(0.0, NlosWeightMode::EXCLUDE, 0.05, 0.5, 3.0), 1.0);
    EXPECT_DOUBLE_EQ(
        nlosVarianceInflationFactor(1.0, NlosWeightMode::EXCLUDE, 0.05, 0.5, 3.0), 1.0);
}

// --------------------------------------------------------------------------
// nlosShouldExclude (WP8)
// --------------------------------------------------------------------------

TEST(NlosWeightsTest, ExcludeModeDropsBelowThreshold) {
    EXPECT_TRUE(nlosShouldExclude(0.3, NlosWeightMode::EXCLUDE, 0.5));
    EXPECT_FALSE(nlosShouldExclude(0.5, NlosWeightMode::EXCLUDE, 0.5));  // >= threshold kept
    EXPECT_FALSE(nlosShouldExclude(0.7, NlosWeightMode::EXCLUDE, 0.5));
    EXPECT_TRUE(nlosShouldExclude(0.0, NlosWeightMode::EXCLUDE, 0.5));
}

TEST(NlosWeightsTest, NonExcludeModesNeverExclude) {
    EXPECT_FALSE(nlosShouldExclude(0.0, NlosWeightMode::OFF, 0.5));
    EXPECT_FALSE(nlosShouldExclude(0.0, NlosWeightMode::TWO_TIER, 0.5));
    EXPECT_FALSE(nlosShouldExclude(0.0, NlosWeightMode::CONTINUOUS, 0.5));
}

TEST(NlosWeightsTest, NonFiniteLosProbIsNeverExcluded) {
    EXPECT_FALSE(nlosShouldExclude(std::nan(""), NlosWeightMode::EXCLUDE, 0.5));
}

TEST(NlosWeightsTest, NonFiniteExcludeThresholdFallsBackToHalf) {
    EXPECT_TRUE(nlosShouldExclude(0.3, NlosWeightMode::EXCLUDE, std::nan("")));
    EXPECT_FALSE(nlosShouldExclude(0.7, NlosWeightMode::EXCLUDE, std::nan("")));
}

// --------------------------------------------------------------------------
// nlosExclusionGuardAllows (WP8 --nlos-min-sats safety guard)
// --------------------------------------------------------------------------

TEST(NlosWeightsTest, GuardAllowsExclusionWhenEnoughSatellitesSurvive) {
    // 10 total, 3 excluded -> 7 survive, floor is 5 -> allowed.
    EXPECT_TRUE(nlosExclusionGuardAllows(10, 3, 5));
}

TEST(NlosWeightsTest, GuardBlocksExclusionWhenTooFewWouldSurvive) {
    // 8 total, 4 excluded -> 4 survive, floor is 5 -> blocked (keep all).
    EXPECT_FALSE(nlosExclusionGuardAllows(8, 4, 5));
}

TEST(NlosWeightsTest, GuardAllowsExactlyAtTheFloor) {
    // 10 total, 5 excluded -> exactly 5 survive, floor is 5 -> allowed.
    EXPECT_TRUE(nlosExclusionGuardAllows(10, 5, 5));
}

TEST(NlosWeightsTest, GuardIsNoOpWhenNothingWouldBeExcluded) {
    EXPECT_FALSE(nlosExclusionGuardAllows(10, 0, 5));
}

TEST(NlosWeightsTest, GuardTreatsNegativeMinSatsAsZero) {
    // Even excluding everyone is allowed once min_sats itself is clamped to 0.
    EXPECT_TRUE(nlosExclusionGuardAllows(5, 5, -3));
}

// --------------------------------------------------------------------------
// nlosMinLosSatsGateAllows (WP10, WP8 recommendation 2 --nlos-min-los-sats)
// --------------------------------------------------------------------------

TEST(NlosWeightsTest, MinLosSatsGateDisabledAtZeroOrNegative) {
    EXPECT_TRUE(nlosMinLosSatsGateAllows(0, 0));
    EXPECT_TRUE(nlosMinLosSatsGateAllows(0, -1));
}

TEST(NlosWeightsTest, MinLosSatsGateBlocksBelowThreshold) {
    EXPECT_FALSE(nlosMinLosSatsGateAllows(3, 4));
}

TEST(NlosWeightsTest, MinLosSatsGateAllowsAtOrAboveThreshold) {
    EXPECT_TRUE(nlosMinLosSatsGateAllows(4, 4));
    EXPECT_TRUE(nlosMinLosSatsGateAllows(6, 4));
}

}  // namespace
}  // namespace nlos_weights
}  // namespace libgnss
