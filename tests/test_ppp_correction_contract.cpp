#include <gtest/gtest.h>

#include <libgnss++/algorithms/ppp_correction_contract.hpp>
#include <libgnss++/core/constants.hpp>

#include <set>
#include <string>

namespace contract = libgnss::algorithms::ppp_correction_contract;

TEST(PPPCorrectionContractTest, MadocalibSsrClockUsesStateVectorRelativity) {
    constexpr double polynomial_clock_s = 5.0e-4;
    constexpr double position_velocity_dot_m2_s = 1.25e11;
    constexpr double expected_relativity_s =
        -2.0 * position_velocity_dot_m2_s /
        (libgnss::constants::SPEED_OF_LIGHT *
         libgnss::constants::SPEED_OF_LIGHT);

    EXPECT_DOUBLE_EQ(
        contract::madocalibSsrBroadcastClock(
            polynomial_clock_s,
            position_velocity_dot_m2_s,
            libgnss::constants::SPEED_OF_LIGHT),
        polynomial_clock_s + expected_relativity_s);
}

TEST(PPPCorrectionContractTest,
     FirstMadocaN1FixRequiresOneConsecutiveConfirmation) {
    EXPECT_TRUE(contract::deferFirstMadocaN1Fix(true, 0, false));
    EXPECT_FALSE(contract::deferFirstMadocaN1Fix(false, 0, false));
    EXPECT_FALSE(contract::deferFirstMadocaN1Fix(true, 0, true));
    EXPECT_FALSE(contract::deferFirstMadocaN1Fix(true, 1, false));
}

TEST(PPPCorrectionContractTest, ApplicationStagesAreUniqueAndOrdered) {
    std::set<contract::Stage> stages;
    std::set<std::string> names;
    for (std::size_t index = 0; index < contract::kApplicationOrder.size(); ++index) {
        const auto stage = contract::kApplicationOrder[index];
        EXPECT_TRUE(stages.insert(stage).second);
        EXPECT_TRUE(names.insert(contract::name(stage)).second);
        EXPECT_EQ(contract::orderOf(stage), index);
    }

    EXPECT_LT(contract::orderOf(contract::Stage::SatelliteOrbitClock),
              contract::orderOf(contract::Stage::SsrMeasurementBias));
    EXPECT_LT(contract::orderOf(contract::Stage::SsrAtmosphere),
              contract::orderOf(contract::Stage::DcbFallback));
    EXPECT_LT(contract::orderOf(contract::Stage::IonexFallback),
              contract::orderOf(contract::Stage::PhaseWindup));
}

TEST(PPPCorrectionContractTest, MadocaBiasConventionAddsMeasurementBiases) {
    EXPECT_DOUBLE_EQ(contract::ssrMeasurementBiasSign(true), 1.0);
    EXPECT_DOUBLE_EQ(contract::ssrMeasurementBiasSign(false), -1.0);
}

TEST(PPPCorrectionContractTest, MadocaPreservesRawCodeIonosphereSeed) {
    EXPECT_FALSE(contract::rederiveIonosphereSeedAfterSsrBias(true));
    EXPECT_TRUE(contract::rederiveIonosphereSeedAfterSsrBias(false));
}

TEST(PPPCorrectionContractTest, MadocaExcludesWindupFromAmbiguitySeed) {
    EXPECT_TRUE(contract::excludePhaseWindupFromAmbiguitySeed(true));
    EXPECT_FALSE(contract::excludePhaseWindupFromAmbiguitySeed(false));
}

TEST(PPPCorrectionContractTest, MadocaUsesMadocalibSolidEarthTideModel) {
    EXPECT_FALSE(contract::useIersSolidEarthTide(true, true));
    EXPECT_TRUE(contract::useIersSolidEarthTide(false, true));
    EXPECT_FALSE(contract::useIersSolidEarthTide(false, false));
}

TEST(PPPCorrectionContractTest, AtmosphereSignsMatchCodeAndPhasePhysics) {
    EXPECT_DOUBLE_EQ(contract::measurementCorrectionSign(false, false), -1.0);
    EXPECT_DOUBLE_EQ(contract::measurementCorrectionSign(true, false), -1.0);
    EXPECT_DOUBLE_EQ(contract::measurementCorrectionSign(false, true), -1.0);
    EXPECT_DOUBLE_EQ(contract::measurementCorrectionSign(true, true), 1.0);
}
