// RTCMStreamDecoder GPST reference: week resolution of TOW-only observations, seeding, and
// dating of GLONASS epochs from the stream instead of the host clock.
#include <gtest/gtest.h>
#include <libgnss++/io/rtcm.hpp>
#include <libgnss++/io/rtcm_stream.hpp>

#include <cstdint>
#include <vector>

using namespace libgnss;

namespace {

uint32_t crc24q(const uint8_t* data, size_t length) {
    uint32_t crc = 0;
    for (size_t i = 0; i < length; ++i) {
        crc ^= static_cast<uint32_t>(data[i]) << 16;
        for (int bit = 0; bit < 8; ++bit) {
            crc <<= 1;
            if (crc & 0x1000000U) {
                crc ^= 0x1864CFBU;
            }
        }
    }
    return crc & 0x00FFFFFFU;
}

std::vector<uint8_t> frameOf(const io::RTCMMessage& message) {
    std::vector<uint8_t> frame;
    frame.push_back(0xD3);
    frame.push_back(static_cast<uint8_t>((message.data.size() >> 8) & 0x03U));
    frame.push_back(static_cast<uint8_t>(message.data.size() & 0xFFU));
    frame.insert(frame.end(), message.data.begin(), message.data.end());
    const uint32_t crc = crc24q(frame.data(), frame.size());
    frame.push_back(static_cast<uint8_t>((crc >> 16) & 0xFFU));
    frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFFU));
    frame.push_back(static_cast<uint8_t>(crc & 0xFFU));
    return frame;
}

Observation codeRow(GNSSSystem system, int prn, SignalType signal, double pseudorange) {
    Observation obs(SatelliteId(system, static_cast<uint8_t>(prn)), signal);
    obs.has_pseudorange = true;
    obs.pseudorange = pseudorange;
    obs.snr = 44.0;
    obs.signal_strength = 7;
    obs.valid = true;
    return obs;
}

io::RTCMMessage encodeObs(const GNSSTime& time, GNSSSystem system, int prn, SignalType signal,
                          io::RTCMMessageType type, int glonass_channel = 0) {
    io::RTCMProcessor encoder;
    ObservationData obs(time);
    Observation row = codeRow(system, prn, signal, 21456780.25);
    if (system == GNSSSystem::GLONASS) {
        row.has_glonass_frequency_channel = true;
        row.glonass_frequency_channel = glonass_channel;
    }
    obs.addObservation(row);
    return encoder.encodeObservations(obs, type);
}

ObservationData push(io::RTCMStreamDecoder& decoder, const io::RTCMMessage& message) {
    const auto frame = frameOf(message);
    std::vector<io::RTCMStreamDecoder::Event> events;
    EXPECT_TRUE(decoder.pushFrame(frame.data(), frame.size(), events));
    EXPECT_EQ(events.size(), 1U);
    EXPECT_TRUE(events.front().has_observation);
    return events.front().observation;
}

Ephemeris gpsEphemeris(int prn, const GNSSTime& toe) {
    Ephemeris eph;
    eph.satellite = SatelliteId(GNSSSystem::GPS, static_cast<uint8_t>(prn));
    eph.toe = eph.toc = toe;
    eph.toes = toe.tow;
    eph.week = static_cast<uint16_t>(toe.week);
    eph.sqrt_a = 5153.795;
    eph.valid = true;
    return eph;
}

}  // namespace

TEST(RtcmStreamReference, NoReferenceKeepsTowOnlyPartsAtWeekZeroAndWritesNoContext) {
    io::RTCMStreamDecoder decoder;
    const auto gps = push(decoder, encodeObs(GNSSTime(2305, 288899.125), GNSSSystem::GPS, 6,
                                             SignalType::GPS_L1CA, io::RTCMMessageType::RTCM_1077));
    EXPECT_EQ(gps.time.week, 0);
    EXPECT_NEAR(gps.time.tow, 288899.125, 1e-6);
    EXPECT_FALSE(decoder.referenceTime().has_value());
    // A GLONASS epoch is dated from the host clock without a reference; that must neither
    // seed the reference nor change how later TOW-only parts are treated.
    const auto glo = push(decoder, encodeObs(GNSSTime(2305, 288899.125), GNSSSystem::GLONASS, 7,
                                             SignalType::GLO_L1CA, io::RTCMMessageType::RTCM_1087, -4));
    EXPECT_GT(glo.time.week, 0);
    EXPECT_FALSE(decoder.referenceTime().has_value());
    const auto gal = push(decoder, encodeObs(GNSSTime(2305, 288899.125), GNSSSystem::Galileo, 11,
                                             SignalType::GAL_E1, io::RTCMMessageType::RTCM_1097));
    EXPECT_EQ(gal.time.week, 0);
}

TEST(RtcmStreamReference, ExplicitSeedResolvesWeekAndDatesGlonassFromTheStream) {
    io::RTCMStreamDecoder decoder;
    decoder.setReferenceTime(GNSSTime(2305, 288900.0));
    const GNSSTime t(2305, 288899.125);
    const auto gps = push(decoder, encodeObs(t, GNSSSystem::GPS, 6, SignalType::GPS_L1CA,
                                             io::RTCMMessageType::RTCM_1077));
    EXPECT_EQ(gps.time.week, 2305);
    EXPECT_NEAR(gps.time - t, 0.0, 1e-6);
    // GLONASS: same instant as the GPS part, week from the stream reference (the host clock is years later).
    const auto glo = push(decoder, encodeObs(t, GNSSSystem::GLONASS, 7, SignalType::GLO_L1CA,
                                             io::RTCMMessageType::RTCM_1087, -4));
    EXPECT_EQ(glo.time.week, 2305);
    EXPECT_NEAR(glo.time - t, 0.0, 1e-3);
    const auto gal = push(decoder, encodeObs(t, GNSSSystem::Galileo, 11, SignalType::GAL_E1,
                                             io::RTCMMessageType::RTCM_1097));
    EXPECT_NEAR(gal.time - t, 0.0, 1e-6);
    // The reference follows the observations.
    ASSERT_TRUE(decoder.referenceTime().has_value());
    EXPECT_NEAR(*decoder.referenceTime() - t, 0.0, 1e-3);
}

TEST(RtcmStreamReference, SeedsFromTheFirstGpsEphemerisAndNotFromGlonass1020) {
    io::RTCMProcessor encoder;
    io::RTCMStreamDecoder decoder;
    std::vector<io::RTCMStreamDecoder::Event> events;

    Ephemeris glo;
    glo.satellite = SatelliteId(GNSSSystem::GLONASS, 7);
    glo.week = 2324;
    glo.toe = glo.toc = GNSSTime(2324, 187218.0);
    glo.tof = GNSSTime(2324, 187248.0);
    glo.glonass_position = Vector3d(19123456.5, -12345678.0, 21765432.5);
    glo.glonass_frequency_channel = -4;
    glo.valid = true;
    const auto glo_frame = frameOf(encoder.encodeEphemeris(glo));
    ASSERT_TRUE(decoder.pushFrame(glo_frame.data(), glo_frame.size(), events));
    ASSERT_EQ(events.size(), 1U);
    EXPECT_TRUE(events.front().has_navigation);
    EXPECT_FALSE(decoder.referenceTime().has_value()) << "a host-clock-dated 1020 must not seed";

    const auto gps_frame = frameOf(encoder.encodeEphemeris(gpsEphemeris(3, GNSSTime(2324, 187200.0))));
    ASSERT_TRUE(decoder.pushFrame(gps_frame.data(), gps_frame.size(), events));
    ASSERT_TRUE(decoder.referenceTime().has_value());
    EXPECT_EQ(decoder.referenceTime()->week, 2324);
    EXPECT_NEAR(decoder.referenceTime()->tow, 187200.0, 1e-6);

    // A later ephemeris does not move the reference.
    const auto later = frameOf(encoder.encodeEphemeris(gpsEphemeris(4, GNSSTime(2324, 194400.0))));
    ASSERT_TRUE(decoder.pushFrame(later.data(), later.size(), events));
    EXPECT_NEAR(decoder.referenceTime()->tow, 187200.0, 1e-6);

    const auto gps = push(decoder, encodeObs(GNSSTime(2324, 187250.0), GNSSSystem::GPS, 6,
                                             SignalType::GPS_L1CA, io::RTCMMessageType::RTCM_1077));
    EXPECT_EQ(gps.time.week, 2324);
}

TEST(RtcmStreamReference, ExplicitSeedBeatsEphemerisAndClearResets) {
    io::RTCMProcessor encoder;
    io::RTCMStreamDecoder decoder;
    decoder.setReferenceTime(GNSSTime(2324, 187200.0));
    const auto frame = frameOf(encoder.encodeEphemeris(gpsEphemeris(3, GNSSTime(2324, 194400.0))));
    std::vector<io::RTCMStreamDecoder::Event> events;
    ASSERT_TRUE(decoder.pushFrame(frame.data(), frame.size(), events));
    EXPECT_NEAR(decoder.referenceTime()->tow, 187200.0, 1e-6);
    decoder.clear();
    EXPECT_FALSE(decoder.referenceTime().has_value());
    GNSSTime invalid(2324, 604800.0);
    EXPECT_THROW(decoder.setReferenceTime(invalid), std::invalid_argument);
}

TEST(RtcmStreamReference, NearestWeekAcrossRolloverBothDirections) {
    {
        io::RTCMStreamDecoder decoder;
        decoder.setReferenceTime(GNSSTime(2305, 604790.0));
        const auto next = push(decoder, encodeObs(GNSSTime(2306, 5.0), GNSSSystem::GPS, 6,
                                                  SignalType::GPS_L1CA, io::RTCMMessageType::RTCM_1077));
        EXPECT_EQ(next.time.week, 2306);
        EXPECT_NEAR(next.time.tow, 5.0, 1e-6);
        // The reference advanced with the stream, so the rollover persists.
        const auto later = push(decoder, encodeObs(GNSSTime(2306, 6.0), GNSSSystem::Galileo, 11,
                                                   SignalType::GAL_E1, io::RTCMMessageType::RTCM_1097));
        EXPECT_EQ(later.time.week, 2306);
    }
    {
        io::RTCMStreamDecoder decoder;
        decoder.setReferenceTime(GNSSTime(2306, 3.0));
        const auto before = push(decoder, encodeObs(GNSSTime(2305, 604795.0), GNSSSystem::GPS, 6,
                                                    SignalType::GPS_L1CA, io::RTCMMessageType::RTCM_1077));
        EXPECT_EQ(before.time.week, 2305);
        EXPECT_NEAR(before.time.tow, 604795.0, 1e-6);
    }
}

TEST(RtcmStreamReference, BeidouWeekOneWrapIsResolved) {
    // GPST 604803 - 14 s is BDT 604789 in week N; the decoder adds 14 s, wrapping to week 1 tow 3.
    io::RTCMStreamDecoder decoder;
    decoder.setReferenceTime(GNSSTime(2305, 604790.0));
    const auto bds = push(decoder, encodeObs(GNSSTime(2306, 3.0), GNSSSystem::BeiDou, 8,
                                             SignalType::BDS_B1I, io::RTCMMessageType::RTCM_1127));
    EXPECT_EQ(bds.time.week, 2306);
    EXPECT_NEAR(bds.time.tow, 3.0, 1e-6);
    // A GPS part of the same epoch resolves to the same instant.
    const auto gps = push(decoder, encodeObs(GNSSTime(2306, 3.0), GNSSSystem::GPS, 6,
                                             SignalType::GPS_L1CA, io::RTCMMessageType::RTCM_1077));
    EXPECT_NEAR(gps.time - bds.time, 0.0, 1e-6);
}

TEST(RtcmStreamReference, ReferenceFromNavigationIsTheLatestGpsToe) {
    NavigationData nav;
    EXPECT_FALSE(io::RTCMStreamDecoder::referenceFromNavigation(nav).has_value());
    nav.addEphemeris(gpsEphemeris(3, GNSSTime(2324, 187200.0)));
    nav.addEphemeris(gpsEphemeris(4, GNSSTime(2324, 201600.0)));
    nav.addEphemeris(gpsEphemeris(5, GNSSTime(2324, 194400.0)));
    Ephemeris gal = gpsEphemeris(11, GNSSTime(2400, 100.0));
    gal.satellite = SatelliteId(GNSSSystem::Galileo, 11);
    nav.addEphemeris(gal);
    const auto ref = io::RTCMStreamDecoder::referenceFromNavigation(nav);
    ASSERT_TRUE(ref.has_value());
    EXPECT_EQ(ref->week, 2324);
    EXPECT_NEAR(ref->tow, 201600.0, 1e-9);
}

namespace {

// A plausible Kepler record for `system` that fits every field of its RTCM ephemeris message.
Ephemeris keplerEphemeris(GNSSSystem system, int prn, const GNSSTime& toe, double sqrt_a) {
    Ephemeris eph = gpsEphemeris(prn, toe);
    eph.satellite = SatelliteId(system, static_cast<uint8_t>(prn));
    eph.sqrt_a = sqrt_a;
    eph.e = 0.002;
    eph.i0 = 0.96;
    eph.omega0 = 1.0;
    eph.omega = 0.5;
    eph.m0 = 0.25;
    eph.sv_accuracy = 2.4;
    eph.sv_health = 0.0;
    eph.valid = true;
    return eph;
}

}  // namespace

TEST(RtcmStreamReference, BeiDouQzssAndGalileoEphemeridesAreDecodedAndSeedTheReference) {
    io::RTCMProcessor encoder;

    {
        // BeiDou: toe/toc are GPST, week is the BDT week (GPS week 2324 = BDT week 968).
        io::RTCMStreamDecoder decoder;
        Ephemeris bds = keplerEphemeris(GNSSSystem::BeiDou, 21, GNSSTime(2324, 187214.0), 5282.635);
        bds.week = 968;
        bds.toes = 187200.0;
        const auto frame = frameOf(encoder.encodeEphemeris(bds));
        std::vector<io::RTCMStreamDecoder::Event> events;
        ASSERT_TRUE(decoder.pushFrame(frame.data(), frame.size(), events));
        ASSERT_EQ(events.size(), 1U);
        EXPECT_EQ(events.front().message_type, io::RTCMMessageType::RTCM_1042);
        EXPECT_TRUE(events.front().has_navigation);
        EXPECT_EQ(decoder.getNavigationData().getEphemeris(bds.satellite).size(), 1U);
        ASSERT_TRUE(decoder.referenceTime().has_value());
        EXPECT_EQ(decoder.referenceTime()->week, 2324);
        EXPECT_NEAR(decoder.referenceTime()->tow, 187214.0, 1e-6);
    }
    {
        io::RTCMStreamDecoder decoder;
        Ephemeris qzss = keplerEphemeris(GNSSSystem::QZSS, 2, GNSSTime(2324, 187200.0), 6493.5);
        const auto frame = frameOf(encoder.encodeEphemeris(qzss));
        std::vector<io::RTCMStreamDecoder::Event> events;
        ASSERT_TRUE(decoder.pushFrame(frame.data(), frame.size(), events));
        ASSERT_EQ(events.size(), 1U);
        EXPECT_EQ(events.front().message_type, io::RTCMMessageType::RTCM_1044);
        EXPECT_TRUE(events.front().has_navigation);
        EXPECT_EQ(decoder.getNavigationData().getEphemeris(qzss.satellite).size(), 1U);
        ASSERT_TRUE(decoder.referenceTime().has_value());
        EXPECT_EQ(decoder.referenceTime()->week, 2324);
    }
    {
        io::RTCMStreamDecoder decoder;
        Ephemeris gal = keplerEphemeris(GNSSSystem::Galileo, 11, GNSSTime(2324, 187200.0), 5440.6);
        gal.data_source_code = 513;
        gal.iode = 47;
        const auto frame = frameOf(encoder.encodeEphemeris(gal));
        std::vector<io::RTCMStreamDecoder::Event> events;
        ASSERT_TRUE(decoder.pushFrame(frame.data(), frame.size(), events));
        ASSERT_EQ(events.size(), 1U);
        EXPECT_EQ(events.front().message_type, io::RTCMMessageType::RTCM_1046);
        EXPECT_TRUE(events.front().has_navigation);
        ASSERT_TRUE(decoder.referenceTime().has_value());
        EXPECT_EQ(decoder.referenceTime()->week, 2324);
    }
}
