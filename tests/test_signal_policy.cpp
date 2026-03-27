#include <gtest/gtest.h>

#include <libgnss++/core/signal_policy.hpp>

using namespace libgnss;

TEST(SignalPolicyTest, MapsBeiDouObservationTypesConsistently) {
    EXPECT_EQ(signal_policy::signalForObservationType(GNSSSystem::BeiDou, "C2I", true),
              SignalType::BDS_B1I);
    EXPECT_EQ(signal_policy::signalForObservationType(GNSSSystem::BeiDou, "C1X", true),
              SignalType::BDS_B1C);
    EXPECT_EQ(signal_policy::signalForObservationType(GNSSSystem::BeiDou, "C7I", false),
              SignalType::BDS_B2I);
    EXPECT_EQ(signal_policy::signalForObservationType(GNSSSystem::BeiDou, "C6I", false),
              SignalType::BDS_B3I);
    EXPECT_EQ(signal_policy::signalForObservationType(GNSSSystem::BeiDou, "C5P", false),
              SignalType::BDS_B2A);
}

TEST(SignalPolicyTest, PrefersBeiDouB1IOverB1CForPrimarySelection) {
    EXPECT_LT(signal_policy::signalPriority(GNSSSystem::BeiDou, SignalType::BDS_B1I, true),
              signal_policy::signalPriority(GNSSSystem::BeiDou, SignalType::BDS_B1C, true));
    EXPECT_EQ(signal_policy::observationPriority(GNSSSystem::BeiDou, "C2I", true), 0);
    EXPECT_EQ(signal_policy::observationPriority(GNSSSystem::BeiDou, "C1X", true), 1);
}

TEST(SignalPolicyTest, PrefersBeiDouB2IThenB3IThenB2AForSecondarySelection) {
    EXPECT_LT(signal_policy::signalPriority(GNSSSystem::BeiDou, SignalType::BDS_B2I, false),
              signal_policy::signalPriority(GNSSSystem::BeiDou, SignalType::BDS_B3I, false));
    EXPECT_LT(signal_policy::signalPriority(GNSSSystem::BeiDou, SignalType::BDS_B3I, false),
              signal_policy::signalPriority(GNSSSystem::BeiDou, SignalType::BDS_B2A, false));
    EXPECT_EQ(signal_policy::observationPriority(GNSSSystem::BeiDou, "C7I", false), 0);
    EXPECT_EQ(signal_policy::observationPriority(GNSSSystem::BeiDou, "C6I", false), 1);
    EXPECT_EQ(signal_policy::observationPriority(GNSSSystem::BeiDou, "C5P", false), 2);
}

TEST(SignalPolicyTest, RejectsUnknownObservationBands) {
    EXPECT_EQ(signal_policy::observationPriority(GNSSSystem::GPS, "C9X", true), 100);
    EXPECT_EQ(signal_policy::observationPriority(GNSSSystem::BeiDou, "C9X", false), 100);
}

TEST(SignalPolicyTest, DetectsBeiDouGeoPrnRanges) {
    EXPECT_TRUE(signal_policy::isBeiDouGeoSatellite(SatelliteId(GNSSSystem::BeiDou, 1)));
    EXPECT_TRUE(signal_policy::isBeiDouGeoSatellite(SatelliteId(GNSSSystem::BeiDou, 5)));
    EXPECT_TRUE(signal_policy::isBeiDouGeoSatellite(SatelliteId(GNSSSystem::BeiDou, 59)));
    EXPECT_TRUE(signal_policy::isBeiDouGeoSatellite(SatelliteId(GNSSSystem::BeiDou, 63)));
    EXPECT_FALSE(signal_policy::isBeiDouGeoSatellite(SatelliteId(GNSSSystem::BeiDou, 6)));
    EXPECT_FALSE(signal_policy::isBeiDouGeoSatellite(SatelliteId(GNSSSystem::GPS, 1)));
}
