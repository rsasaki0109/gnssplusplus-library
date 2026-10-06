#include <gtest/gtest.h>
#include <libgnss++/io/rtcm.hpp>
#include <limits>

using namespace libgnss;
TEST(RtcmTimeContext, HistoricalGpsWeekUsesReceptionEra) {
    io::RTCMProcessor encoder, decoder;
    Ephemeris eph;
    eph.satellite = SatelliteId(GNSSSystem::GPS, 3);
    eph.week = 900;
    eph.toe = eph.toc = GNSSTime(900, 172800.0);
    eph.sqrt_a = 5153.795;
    eph.valid = true;
    const auto message = encoder.encodeEphemeris(eph);
    ASSERT_TRUE(message.valid);
    decoder.setReferenceTime(GNSSTime(900, 172900.0));
    NavigationData nav;
    ASSERT_TRUE(decoder.decodeNavigationData(message, nav));
    ASSERT_EQ(nav.ephemeris_data.at(eph.satellite).size(), 1U);
    EXPECT_EQ(nav.ephemeris_data.at(eph.satellite).front().toe.week, 900);
    decoder.setReferenceTime(GNSSTime(1924, 172900.0));
    NavigationData later;
    ASSERT_TRUE(decoder.decodeNavigationData(message, later));
    EXPECT_EQ(later.ephemeris_data.at(eph.satellite).front().toe.week, 1924);
}
TEST(RtcmTimeContext, GlonassEphemerisDayUsesHistoricalReception) {
    io::RTCMProcessor encoder, decoder;
    Ephemeris eph;
    eph.satellite = SatelliteId(GNSSSystem::GLONASS, 7);
    eph.week = 2324;
    // UTC time-of-day on a 900 second broadcast boundary, plus leap seconds.
    eph.toe = eph.toc = GNSSTime(2324, 187218.0);
    eph.tof = GNSSTime(2324, 187248.0);
    eph.glonass_position = Vector3d(19123456.5, -12345678.0, 21765432.5);
    eph.glonass_frequency_channel = -4;
    eph.valid = true;
    const auto message = encoder.encodeEphemeris(eph);
    ASSERT_TRUE(message.valid);
    decoder.setReferenceTime(GNSSTime(2324, 187300.0));
    NavigationData nav;
    ASSERT_TRUE(decoder.decodeNavigationData(message, nav));
    const auto& actual = nav.ephemeris_data.at(eph.satellite).front();
    EXPECT_DOUBLE_EQ(actual.toe - eph.toe, 0.0);
    EXPECT_DOUBLE_EQ(actual.tof - eph.tof, 0.0);
}
TEST(RtcmTimeContext, GlonassMsmWeekUsesReceptionContext) {
    io::RTCMProcessor encoder, decoder;
    ObservationData obs(GNSSTime(2324, 187300.125));
    Observation row(SatelliteId(GNSSSystem::GLONASS, 7), SignalType::GLO_L1CA);
    encoder.setGlonassFrequencyChannel(row.satellite, 0);
    row.has_glonass_frequency_channel = true;
    row.glonass_frequency_channel = 0;
    row.pseudorange = 22000000.0;
    row.has_pseudorange = true;
    obs.addObservation(row);
    const auto message = encoder.encodeObservations(obs, io::RTCMMessageType::RTCM_1087);
    ASSERT_TRUE(message.valid);
    decoder.setReferenceTime(GNSSTime(2324, 187301.0));
    ObservationData actual;
    ASSERT_TRUE(decoder.decodeObservationData(message, actual));
    EXPECT_EQ(actual.time.week, 2324);
    EXPECT_NEAR(actual.time.tow, obs.time.tow, 1e-6);
}
TEST(RtcmTimeContext, InvalidReferenceTimeIsRejected) {
    io::RTCMProcessor decoder;
    EXPECT_THROW(decoder.setReferenceTime(GNSSTime(-1, 10.0)), std::invalid_argument);
    GNSSTime invalid(2324, 10.0);
    invalid.tow = 604800.0;
    EXPECT_THROW(decoder.setReferenceTime(invalid), std::invalid_argument);
    invalid.tow = std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(decoder.setReferenceTime(invalid), std::invalid_argument);
}
