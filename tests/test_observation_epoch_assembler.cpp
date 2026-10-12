#include <gtest/gtest.h>
#include <libgnss++/io/observation_epoch_assembler.hpp>
#include <libgnss++/io/rtcm.hpp>
#include <libgnss++/io/rtcm_stream.hpp>

#include <cmath>
#include <cstdint>
#include <set>
#include <utility>
#include <vector>

using namespace libgnss;

namespace {

Observation makeRow(GNSSSystem system, int prn, SignalType signal, double pseudorange) {
    Observation obs(SatelliteId(system, static_cast<uint8_t>(prn)), signal);
    obs.pseudorange = pseudorange;
    obs.has_pseudorange = true;
    obs.valid = true;
    return obs;
}

ObservationData makePart(const GNSSTime& time, std::vector<Observation> rows) {
    ObservationData part(time);
    for (const auto& row : rows) {
        part.addObservation(row);
    }
    return part;
}

std::set<std::pair<SatelliteId, SignalType>> rowKeys(const ObservationData& epoch) {
    std::set<std::pair<SatelliteId, SignalType>> keys;
    for (const auto& obs : epoch.observations) {
        keys.emplace(obs.satellite, obs.signal);
    }
    return keys;
}

}  // namespace

TEST(ObservationEpochAssembler, SingleMessageWithClearedBitPassesThrough) {
    io::ObservationEpochAssembler assembler;
    const auto part = makePart(GNSSTime(2200, 100.0), {makeRow(GNSSSystem::GPS, 3, SignalType::GPS_L1CA, 2.1e7)});
    const auto out = assembler.push(part, false);
    ASSERT_EQ(out.size(), 1U);
    EXPECT_EQ(out[0].observations.size(), 1U);
    EXPECT_DOUBLE_EQ(out[0].time - part.time, 0.0);
    EXPECT_FALSE(assembler.hasPending());
    EXPECT_TRUE(assembler.flush().empty());
}

TEST(ObservationEpochAssembler, MergesSameTimeAndEmitsOnClearedBit) {
    io::ObservationEpochAssembler assembler;
    const GNSSTime t(2200, 100.0);
    EXPECT_TRUE(assembler.push(makePart(t, {makeRow(GNSSSystem::GPS, 3, SignalType::GPS_L1CA, 2.1e7)}), true).empty());
    EXPECT_TRUE(assembler.hasPending());
    EXPECT_TRUE(assembler.push(makePart(t, {makeRow(GNSSSystem::GLONASS, 5, SignalType::GLO_L1CA, 2.2e7)}), true).empty());
    const auto out = assembler.push(
        makePart(t + 0.0005, {makeRow(GNSSSystem::Galileo, 11, SignalType::GAL_E1, 2.3e7)}), false);
    ASSERT_EQ(out.size(), 1U);
    EXPECT_EQ(out[0].observations.size(), 3U);
    EXPECT_FALSE(assembler.hasPending());
    // The merged epoch keeps the time of its first part.
    EXPECT_DOUBLE_EQ(out[0].time - t, 0.0);
}

TEST(ObservationEpochAssembler, LostLastMessageEmitsPendingOnTimeChange) {
    io::ObservationEpochAssembler assembler;
    const GNSSTime t0(2200, 100.0);
    const GNSSTime t1(2200, 100.2);
    EXPECT_TRUE(assembler.push(makePart(t0, {makeRow(GNSSSystem::GPS, 3, SignalType::GPS_L1CA, 2.1e7)}), true).empty());
    // The last message of epoch t0 (bit 0) never arrives; the next epoch starts.
    auto out = assembler.push(makePart(t1, {makeRow(GNSSSystem::GPS, 3, SignalType::GPS_L1CA, 2.1e7)}), true);
    ASSERT_EQ(out.size(), 1U);
    EXPECT_DOUBLE_EQ(out[0].time - t0, 0.0);
    EXPECT_EQ(out[0].observations.size(), 1U);
    EXPECT_TRUE(assembler.hasPending());
    out = assembler.push(makePart(t1, {makeRow(GNSSSystem::BeiDou, 8, SignalType::BDS_B1I, 2.4e7)}), false);
    ASSERT_EQ(out.size(), 1U);
    EXPECT_DOUBLE_EQ(out[0].time - t1, 0.0);
    EXPECT_EQ(out[0].observations.size(), 2U);
}

TEST(ObservationEpochAssembler, TimeChangeWithClearedBitEmitsBothEpochs) {
    io::ObservationEpochAssembler assembler;
    const GNSSTime t0(2200, 100.0);
    const GNSSTime t1(2200, 101.0);
    EXPECT_TRUE(assembler.push(makePart(t0, {makeRow(GNSSSystem::GPS, 3, SignalType::GPS_L1CA, 2.1e7)}), true).empty());
    const auto out = assembler.push(makePart(t1, {makeRow(GNSSSystem::GPS, 4, SignalType::GPS_L1CA, 2.1e7)}), false);
    ASSERT_EQ(out.size(), 2U);
    EXPECT_DOUBLE_EQ(out[0].time - t0, 0.0);
    EXPECT_DOUBLE_EQ(out[1].time - t1, 0.0);
    EXPECT_FALSE(assembler.hasPending());
}

TEST(ObservationEpochAssembler, DuplicateSatelliteSignalKeepsFirst) {
    io::ObservationEpochAssembler assembler;
    const GNSSTime t(2200, 100.0);
    EXPECT_TRUE(assembler.push(makePart(t, {makeRow(GNSSSystem::GPS, 3, SignalType::GPS_L1CA, 1.0e7)}), true).empty());
    const auto out = assembler.push(
        makePart(t, {makeRow(GNSSSystem::GPS, 3, SignalType::GPS_L1CA, 2.0e7),
                     makeRow(GNSSSystem::GPS, 3, SignalType::GPS_L2C, 3.0e7)}),
        false);
    ASSERT_EQ(out.size(), 1U);
    ASSERT_EQ(out[0].observations.size(), 2U);
    EXPECT_EQ(rowKeys(out[0]).size(), 2U);
    for (const auto& obs : out[0].observations) {
        if (obs.signal == SignalType::GPS_L1CA) {
            EXPECT_DOUBLE_EQ(obs.pseudorange, 1.0e7);
        }
    }
}

TEST(ObservationEpochAssembler, MergeKeepsTrackingObservationsAndPositionOfLaterParts) {
    io::ObservationEpochAssembler assembler;
    const GNSSTime t(2200, 100.0);
    auto gps = makePart(t, {makeRow(GNSSSystem::GPS, 3, SignalType::GPS_L1CA, 2.1e7)});
    gps.addRinexTrackingObservation("1C", gps.observations[0]);
    auto gal = makePart(t, {makeRow(GNSSSystem::Galileo, 11, SignalType::GAL_E5A, 2.3e7)});
    gal.addRinexTrackingObservation("5Q", gal.observations[0]);
    gal.addRinexTrackingObservation("5I", gal.observations[0]);
    gal.receiver_position = Vector3d(1.0, 2.0, 3.0);
    EXPECT_TRUE(assembler.push(gps, true).empty());
    const auto out = assembler.push(gal, false);
    ASSERT_EQ(out.size(), 1U);
    EXPECT_EQ(out[0].observations.size(), 2U);
    EXPECT_EQ(out[0].rinex_tracking_observations.size(), 3U);
    EXPECT_NE(out[0].getRinexTrackingObservation(SatelliteId(GNSSSystem::Galileo, 11), "5I"), nullptr);
    EXPECT_NEAR(out[0].receiver_position.x(), 1.0, 0.0);
}

TEST(ObservationEpochAssembler, FlushEmitsPendingEpochOnce) {
    io::ObservationEpochAssembler assembler;
    const GNSSTime t(2200, 100.0);
    EXPECT_TRUE(assembler.flush().empty());
    EXPECT_TRUE(assembler.push(makePart(t, {makeRow(GNSSSystem::GPS, 3, SignalType::GPS_L1CA, 2.1e7)}), true).empty());
    EXPECT_TRUE(assembler.push(makePart(t, {makeRow(GNSSSystem::GLONASS, 5, SignalType::GLO_L1CA, 2.2e7)}), true).empty());
    const auto out = assembler.flush();
    ASSERT_EQ(out.size(), 1U);
    EXPECT_EQ(out[0].observations.size(), 2U);
    EXPECT_FALSE(assembler.hasPending());
    EXPECT_TRUE(assembler.flush().empty());
}

TEST(ObservationEpochAssembler, ClearDropsPendingEpoch) {
    io::ObservationEpochAssembler assembler;
    const GNSSTime t(2200, 100.0);
    EXPECT_TRUE(assembler.push(makePart(t, {makeRow(GNSSSystem::GPS, 3, SignalType::GPS_L1CA, 2.1e7)}), true).empty());
    assembler.clear();
    EXPECT_FALSE(assembler.hasPending());
    EXPECT_TRUE(assembler.flush().empty());
}

// ---------------------------------------------------------------------------
// Encode -> decode -> assemble round trip of a synthetic G+R+E+C epoch.
// ---------------------------------------------------------------------------

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

Observation makeCodeRow(GNSSSystem system, int prn, SignalType signal, double pseudorange, double snr) {
    Observation obs(SatelliteId(system, static_cast<uint8_t>(prn)), signal);
    obs.has_pseudorange = true;
    obs.pseudorange = pseudorange;
    obs.snr = snr;
    obs.signal_strength = static_cast<int>(std::lround(snr / 6.0));
    obs.valid = true;
    return obs;
}

struct SyntheticEpoch {
    ObservationData gps, glonass, galileo, beidou;
    std::vector<io::RTCMMessageType> types;
    std::vector<ObservationData*> parts;
};

SyntheticEpoch makeSyntheticEpoch(const GNSSTime& time, double range_offset_m) {
    SyntheticEpoch e;
    e.gps = ObservationData(time);
    e.glonass = ObservationData(time);
    e.galileo = ObservationData(time);
    e.beidou = ObservationData(time);
    e.gps.addObservation(makeCodeRow(GNSSSystem::GPS, 6, SignalType::GPS_L1CA, 21456789.12 + range_offset_m, 46.0));
    e.gps.addObservation(makeCodeRow(GNSSSystem::GPS, 15, SignalType::GPS_L1CA, 22456780.34 + range_offset_m, 41.0));
    Observation glo = makeCodeRow(GNSSSystem::GLONASS, 7, SignalType::GLO_L1CA, 21456780.25 + range_offset_m, 43.0);
    glo.has_glonass_frequency_channel = true;
    glo.glonass_frequency_channel = -4;
    e.glonass.addObservation(glo);
    e.galileo.addObservation(makeCodeRow(GNSSSystem::Galileo, 11, SignalType::GAL_E1, 24456789.25 + range_offset_m, 45.0));
    e.galileo.addObservation(makeCodeRow(GNSSSystem::Galileo, 19, SignalType::GAL_E1, 25456780.80 + range_offset_m, 40.0));
    e.beidou.addObservation(makeCodeRow(GNSSSystem::BeiDou, 8, SignalType::BDS_B1I, 21456780.50 + range_offset_m, 44.0));
    e.types = {io::RTCMMessageType::RTCM_1077, io::RTCMMessageType::RTCM_1087,
               io::RTCMMessageType::RTCM_1097, io::RTCMMessageType::RTCM_1127};
    e.parts = {&e.gps, &e.glonass, &e.galileo, &e.beidou};
    return e;
}

// Same week resolution as gnss_convert: GPS/Galileo/BeiDou MSM carry only a time of
// week (week 0, or 1 when the BDT shift wraps); GLONASS is resolved from the reference time.
void resolveWeek(ObservationData& obs, int week) {
    if (obs.time.week <= 1) {
        const double tow = std::fmod(obs.time.week * 604800.0 + obs.time.tow, 604800.0);
        obs.time = GNSSTime(week, tow);
    }
}

}  // namespace

TEST(ObservationEpochAssemblerRoundTrip, MultipleMessageBitIsCarriedByTheDecoder) {
    constexpr int kWeek = 2305;
    io::RTCMProcessor encoder;
    const SyntheticEpoch epoch = makeSyntheticEpoch(GNSSTime(kWeek, 288899.125), 0.0);

    std::vector<uint8_t> stream;
    for (size_t i = 0; i < epoch.types.size(); ++i) {
        const bool more = i + 1 < epoch.types.size();
        const auto msg = encoder.encodeObservations(*epoch.parts[i], epoch.types[i], more);
        ASSERT_TRUE(msg.valid);
        const auto frame = frameOf(msg);
        stream.insert(stream.end(), frame.begin(), frame.end());
    }

    // The bit is exposed per message by the stream decoder and by the processor accessor.
    io::RTCMStreamDecoder stream_decoder;
    std::vector<io::RTCMStreamDecoder::Event> events;
    ASSERT_TRUE(stream_decoder.pushFrame(stream.data(), stream.size(), events));
    ASSERT_EQ(events.size(), 4U);
    for (size_t i = 0; i < events.size(); ++i) {
        EXPECT_TRUE(events[i].has_observation);
        EXPECT_EQ(events[i].message_type, epoch.types[i]);
        EXPECT_EQ(events[i].observation_more_follows, i + 1 < events.size()) << "message " << i;
    }

    io::RTCMProcessor decoder;
    decoder.setReferenceTime(GNSSTime(kWeek, 288900.0));
    io::ObservationEpochAssembler assembler;
    std::vector<ObservationData> assembled;
    const auto messages = decoder.decode(stream.data(), stream.size());
    ASSERT_EQ(messages.size(), 4U);
    for (const auto& message : messages) {
        ObservationData part;
        ASSERT_TRUE(decoder.decodeObservationData(message, part));
        resolveWeek(part, kWeek);
        for (auto& done : assembler.push(part, decoder.lastObservationMoreFollows())) {
            assembled.push_back(std::move(done));
        }
    }
    for (auto& done : assembler.flush()) {
        assembled.push_back(std::move(done));
    }

    ASSERT_EQ(assembled.size(), 1U);
    EXPECT_EQ(assembled[0].observations.size(), 6U);
    std::set<GNSSSystem> systems;
    for (const auto& obs : assembled[0].observations) {
        systems.insert(obs.satellite.system);
    }
    EXPECT_EQ(systems, (std::set<GNSSSystem>{GNSSSystem::GPS, GNSSSystem::GLONASS,
                                              GNSSSystem::Galileo, GNSSSystem::BeiDou}));
    EXPECT_NEAR(assembled[0].time - GNSSTime(kWeek, 288899.125), 0.0, 1e-3);
}

TEST(ObservationEpochAssemblerRoundTrip, TwoEpochsAssembleToTwoAndLostLastMessageDoesNotMerge) {
    constexpr int kWeek = 2305;
    io::RTCMProcessor encoder;
    std::vector<uint8_t> stream;
    auto append = [&](const SyntheticEpoch& e, size_t skip_index) {
        for (size_t i = 0; i < e.types.size(); ++i) {
            if (i == skip_index) {
                continue;
            }
            const auto msg = encoder.encodeObservations(*e.parts[i], e.types[i], i + 1 < e.types.size());
            ASSERT_TRUE(msg.valid);
            const auto frame = frameOf(msg);
            stream.insert(stream.end(), frame.begin(), frame.end());
        }
    };
    // Epoch 1 loses its last (bit 0) message; epoch 2 is complete.
    append(makeSyntheticEpoch(GNSSTime(kWeek, 288899.0), 0.0), 3);
    append(makeSyntheticEpoch(GNSSTime(kWeek, 288899.2), 1.0), static_cast<size_t>(-1));

    io::RTCMProcessor decoder;
    decoder.setReferenceTime(GNSSTime(kWeek, 288900.0));
    io::ObservationEpochAssembler assembler;
    std::vector<ObservationData> assembled;
    for (const auto& message : decoder.decode(stream.data(), stream.size())) {
        ObservationData part;
        ASSERT_TRUE(decoder.decodeObservationData(message, part));
        resolveWeek(part, kWeek);
        for (auto& done : assembler.push(part, decoder.lastObservationMoreFollows())) {
            assembled.push_back(std::move(done));
        }
    }
    for (auto& done : assembler.flush()) {
        assembled.push_back(std::move(done));
    }
    ASSERT_EQ(assembled.size(), 2U);
    EXPECT_NEAR(assembled[0].time - GNSSTime(kWeek, 288899.0), 0.0, 1e-3);
    EXPECT_EQ(assembled[0].observations.size(), 5U);  // no BeiDou row
    EXPECT_NEAR(assembled[1].time - GNSSTime(kWeek, 288899.2), 0.0, 1e-3);
    EXPECT_EQ(assembled[1].observations.size(), 6U);
}

TEST(ObservationEpochAssemblerRoundTrip, DefaultEncodingKeepsMultipleMessageBitClear) {
    io::RTCMProcessor encoder;
    const SyntheticEpoch epoch = makeSyntheticEpoch(GNSSTime(2305, 288899.125), 0.0);
    for (size_t i = 0; i < epoch.types.size(); ++i) {
        const auto plain = encoder.encodeObservations(*epoch.parts[i], epoch.types[i]);
        const auto explicit_false = encoder.encodeObservations(*epoch.parts[i], epoch.types[i], false);
        const auto flagged = encoder.encodeObservations(*epoch.parts[i], epoch.types[i], true);
        ASSERT_TRUE(plain.valid);
        ASSERT_TRUE(flagged.valid);
        EXPECT_EQ(plain.data, explicit_false.data);
        EXPECT_NE(plain.data, flagged.data);
        ASSERT_EQ(plain.data.size(), flagged.data.size());
        // Only the multiple-message bit (bit 54 of the MSM header) differs.
        size_t differing_bits = 0;
        for (size_t b = 0; b < plain.data.size(); ++b) {
            differing_bits += static_cast<size_t>(__builtin_popcount(plain.data[b] ^ flagged.data[b]));
        }
        EXPECT_EQ(differing_bits, 1U);
        EXPECT_EQ(plain.data[6] ^ flagged.data[6], 0x02);  // bit 54 = byte 6, bit 6 (0x02)
    }
}

TEST(ObservationEpochAssemblerRoundTrip, Legacy1004SynchronousFlagIsExposed) {
    io::RTCMProcessor encoder, decoder;
    ObservationData obs(GNSSTime(2305, 288899.0));
    for (const auto signal : {SignalType::GPS_L1CA, SignalType::GPS_L2C}) {
        const double wavelength = signal == SignalType::GPS_L1CA ? 0.1902936727984 : 0.2442102134246;
        Observation row(SatelliteId(GNSSSystem::GPS, 6), signal);
        row.has_pseudorange = true;
        row.pseudorange = 21456789.12;
        row.has_carrier_phase = true;
        row.carrier_phase = 21456790.0 / wavelength;
        row.snr = 45.0;
        row.signal_strength = 8;
        row.valid = true;
        obs.addObservation(row);
    }
    for (const bool more : {false, true}) {
        const auto msg = encoder.encodeObservations(obs, io::RTCMMessageType::RTCM_1004, more);
        ASSERT_TRUE(msg.valid);
        ObservationData decoded;
        ASSERT_TRUE(decoder.decodeObservationData(msg, decoded));
        EXPECT_EQ(decoder.lastObservationMoreFollows(), more);
    }
}
