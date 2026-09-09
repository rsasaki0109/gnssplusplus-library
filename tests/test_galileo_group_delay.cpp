#include <gtest/gtest.h>

#include <libgnss++/algorithms/galileo_group_delay.hpp>

#include <cmath>
#include <limits>

namespace {

libgnss::Observation galileoObservation(libgnss::SignalType signal) {
    libgnss::Observation observation;
    observation.satellite =
        libgnss::SatelliteId(libgnss::GNSSSystem::Galileo, 2);
    observation.signal = signal;
    return observation;
}

libgnss::Ephemeris ephemeris(int source_code) {
    libgnss::Ephemeris eph;
    eph.satellite =
        libgnss::SatelliteId(libgnss::GNSSSystem::Galileo, 2);
    eph.tgd = 1.0e-9;
    eph.tgd_secondary = 2.0e-9;
    eph.data_source_code = source_code;
    return eph;
}

}  // namespace

TEST(GalileoGroupDelayTest, FNavE1UsesPrimaryBgd) {
    const auto selected = libgnss::galileo_group_delay::select(
        galileoObservation(libgnss::SignalType::GAL_E1), ephemeris(1 << 8),
        true);
    EXPECT_TRUE(selected.valid);
    EXPECT_EQ(selected.reference,
              libgnss::galileo_group_delay::Reference::FNavE1E5a);
    EXPECT_FALSE(selected.usedSecondaryField());
    EXPECT_DOUBLE_EQ(selected.delay_seconds, 1.0e-9);
}

TEST(GalileoGroupDelayTest, INavE1UsesSecondaryBgd) {
    const auto selected = libgnss::galileo_group_delay::select(
        galileoObservation(libgnss::SignalType::GAL_E1), ephemeris(1 << 9),
        true);
    EXPECT_TRUE(selected.valid);
    EXPECT_EQ(selected.reference,
              libgnss::galileo_group_delay::Reference::INavE1E5b);
    EXPECT_TRUE(selected.usedSecondaryField());
    EXPECT_DOUBLE_EQ(selected.delay_seconds, 2.0e-9);
    EXPECT_DOUBLE_EQ(
        libgnss::galileo_group_delay::correctionMeters(
            galileoObservation(libgnss::SignalType::GAL_E1), ephemeris(1 << 9),
            true),
        2.0e-9 * libgnss::constants::SPEED_OF_LIGHT);
}

TEST(GalileoGroupDelayTest, MissingOrContradictorySourceRetainsLegacyField) {
    for (const int source_code : {0, (1 << 8) | (1 << 9)}) {
        const auto selected = libgnss::galileo_group_delay::select(
            galileoObservation(libgnss::SignalType::GAL_E1),
            ephemeris(source_code), true);
        EXPECT_TRUE(selected.valid);
        EXPECT_EQ(selected.reference,
                  libgnss::galileo_group_delay::Reference::Ambiguous);
        EXPECT_TRUE(selected.fellBackToPrimaryField());
        EXPECT_DOUBLE_EQ(selected.delay_seconds, 1.0e-9);
    }
}

TEST(GalileoGroupDelayTest, DisabledIsByteCompatibleAndOtherSignalUnchanged) {
    const auto e1 = galileoObservation(libgnss::SignalType::GAL_E1);
    const auto e5a = galileoObservation(libgnss::SignalType::GAL_E5A);
    const auto eph = ephemeris(1 << 9);
    EXPECT_DOUBLE_EQ(libgnss::galileo_group_delay::correctionMeters(
                         e1, eph, false),
                     eph.tgd * libgnss::constants::SPEED_OF_LIGHT);
    EXPECT_DOUBLE_EQ(libgnss::galileo_group_delay::correctionMeters(
                         e5a, eph, true),
                     eph.tgd * libgnss::constants::SPEED_OF_LIGHT);
}

TEST(GalileoGroupDelayTest, NonfiniteSelectedSecondaryFailsClosed) {
    auto eph = ephemeris(1 << 9);
    eph.tgd_secondary = std::numeric_limits<double>::quiet_NaN();
    const auto selected = libgnss::galileo_group_delay::select(
        galileoObservation(libgnss::SignalType::GAL_E1), eph, true);
    EXPECT_FALSE(selected.valid);
}
