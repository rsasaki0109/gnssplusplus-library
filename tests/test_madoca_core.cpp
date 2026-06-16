#include <gtest/gtest.h>

#include <libgnss++/algorithms/madoca_core.hpp>
#include <libgnss++/algorithms/madoca_parity.hpp>

namespace {

namespace core = libgnss::algorithms::madoca_core;
namespace mp = libgnss::algorithms::madoca_parity;

TEST(MadocaCore, MapsCompactSsrSystems) {
    EXPECT_EQ(core::gnssSystemForCompactSsrId(0), libgnss::GNSSSystem::GPS);
    EXPECT_EQ(core::gnssSystemForCompactSsrId(1), libgnss::GNSSSystem::GLONASS);
    EXPECT_EQ(core::gnssSystemForCompactSsrId(2), libgnss::GNSSSystem::Galileo);
    EXPECT_EQ(core::gnssSystemForCompactSsrId(3), libgnss::GNSSSystem::BeiDou);
    EXPECT_EQ(core::gnssSystemForCompactSsrId(4), libgnss::GNSSSystem::QZSS);
    EXPECT_EQ(core::gnssSystemForCompactSsrId(7), libgnss::GNSSSystem::BeiDou);
    EXPECT_EQ(core::gnssSystemForCompactSsrId(15), libgnss::GNSSSystem::UNKNOWN);

    ASSERT_TRUE(core::compactSsrIdForGnssSystem(libgnss::GNSSSystem::BeiDou).has_value());
    EXPECT_EQ(*core::compactSsrIdForGnssSystem(libgnss::GNSSSystem::BeiDou), 3);
    EXPECT_EQ(*core::compactSsrIdForGnssSystem(libgnss::GNSSSystem::BeiDou, true), 7);
    EXPECT_FALSE(core::compactSsrIdForGnssSystem(libgnss::GNSSSystem::SBAS).has_value());

    EXPECT_EQ(core::compactSsrSystemName(4), "qzss");
    EXPECT_EQ(core::compactSsrSystemName(7), "beidou-3");
    EXPECT_EQ(core::compactSsrSystemName(99), "unknown");
}

TEST(MadocaCore, NamesSupportedSubtypes) {
    EXPECT_TRUE(core::compactSsrSubtype(1).has_value());
    EXPECT_TRUE(core::compactSsrSubtype(7).has_value());
    EXPECT_FALSE(core::compactSsrSubtype(6).has_value());

    EXPECT_EQ(core::compactSsrSubtypeName(1), "mask");
    EXPECT_EQ(core::compactSsrSubtypeName(2), "orbit");
    EXPECT_EQ(core::compactSsrSubtypeName(3), "clock");
    EXPECT_EQ(core::compactSsrSubtypeName(4), "code-bias");
    EXPECT_EQ(core::compactSsrSubtypeName(5), "phase-bias");
    EXPECT_EQ(core::compactSsrSubtypeName(7), "ura");
    EXPECT_EQ(core::compactSsrSubtypeName(12), "unknown");
}

TEST(MadocaCore, ProvidesSsrUpdateIntervals) {
    ASSERT_TRUE(core::ssrUpdateIntervalSeconds(0).has_value());
    EXPECT_DOUBLE_EQ(*core::ssrUpdateIntervalSeconds(0), 1.0);
    EXPECT_DOUBLE_EQ(*core::ssrUpdateIntervalSeconds(5), 30.0);
    EXPECT_DOUBLE_EQ(*core::ssrUpdateIntervalSeconds(15), 10800.0);
    EXPECT_FALSE(core::ssrUpdateIntervalSeconds(-1).has_value());
    EXPECT_FALSE(core::ssrUpdateIntervalSeconds(16).has_value());
}

TEST(MadocaCore, ExpandsSignalMasksInMadocalibOrder) {
    EXPECT_EQ(core::compactSsrSignalCodeAt(0, 0), mp::kCodeL1C);
    EXPECT_EQ(core::compactSsrSignalCodeAt(0, 13), mp::kCodeL5X);
    EXPECT_EQ(core::compactSsrSignalCodeAt(4, 13), mp::kCodeL1E);
    EXPECT_EQ(core::compactSsrSignalCodeAt(7, 6), mp::kCodeL7D);
    EXPECT_EQ(core::compactSsrSignalCodeAt(7, 2), mp::kCodeNone);
    EXPECT_EQ(core::compactSsrSignalCodeAt(99, 0), mp::kCodeNone);
    EXPECT_EQ(core::compactSsrSignalCodeAt(0, 16), mp::kCodeNone);

    const auto gps = core::compactSsrSignalCodes(0, 0xE001);
    ASSERT_EQ(gps.size(), 4U);
    EXPECT_EQ(gps[0], mp::kCodeL1C);
    EXPECT_EQ(gps[1], mp::kCodeL1P);
    EXPECT_EQ(gps[2], mp::kCodeL1W);
    EXPECT_EQ(gps[3], mp::kCodeNone);

    const auto qzss = core::compactSsrSignalCodes(4, 0x0044);
    ASSERT_EQ(qzss.size(), 2U);
    EXPECT_EQ(qzss[0], mp::kCodeL5X);
    EXPECT_EQ(qzss[1], mp::kCodeL1E);
}

TEST(MadocaCore, ConvertsSsrUraIndicatorToSigmaMeters) {
    EXPECT_DOUBLE_EQ(core::ssrUraIndicatorToSigmaMeters(0), 0.15);
    EXPECT_NEAR(core::ssrUraIndicatorToSigmaMeters(9), 0.00275, 1e-12);
    EXPECT_DOUBLE_EQ(core::ssrUraIndicatorToSigmaMeters(63), 5.4665);
    EXPECT_DOUBLE_EQ(core::ssrUraIndicatorToSigmaMeters(-1), 5.4665);
}

TEST(MadocaCore, SelectsBiasCodesThroughParityHelper) {
    EXPECT_EQ(core::selectBiasCode(mp::kSysGps, mp::kCodeL1C), mp::kCodeL1C);
    EXPECT_EQ(core::selectBiasCode(mp::kSysGps, mp::kCodeL1P), mp::kCodeL1W);
    EXPECT_EQ(core::selectBiasCode(mp::kSysQzs, mp::kCodeL1L), mp::kCodeL1X);
    EXPECT_EQ(core::selectBiasCode(mp::kSysCmp, mp::kCodeL6X), mp::kCodeL6I);
    EXPECT_EQ(core::selectBiasCode(mp::kSysSbs, mp::kCodeL1C), mp::kCodeNone);
}

}  // namespace
