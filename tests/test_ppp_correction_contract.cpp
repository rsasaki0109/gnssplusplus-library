#include <gtest/gtest.h>

#include <libgnss++/algorithms/ppp_correction_contract.hpp>

#include <set>
#include <string>

namespace contract = libgnss::algorithms::ppp_correction_contract;

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

TEST(PPPCorrectionContractTest, AtmosphereSignsMatchCodeAndPhasePhysics) {
    EXPECT_DOUBLE_EQ(contract::measurementCorrectionSign(false, false), -1.0);
    EXPECT_DOUBLE_EQ(contract::measurementCorrectionSign(true, false), -1.0);
    EXPECT_DOUBLE_EQ(contract::measurementCorrectionSign(false, true), -1.0);
    EXPECT_DOUBLE_EQ(contract::measurementCorrectionSign(true, true), 1.0);
}
