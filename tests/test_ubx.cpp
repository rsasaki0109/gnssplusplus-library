#include <gtest/gtest.h>
#include <libgnss++/io/ubx.hpp>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

using namespace libgnss;

namespace {

template <typename T>
void appendLittleEndian(std::vector<uint8_t>& buffer, T value) {
    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(&value);
    buffer.insert(buffer.end(), ptr, ptr + sizeof(T));
}

void setLittleEndian(std::vector<uint8_t>& buffer, size_t offset, uint32_t value) {
    buffer[offset + 0] = static_cast<uint8_t>(value & 0xFFU);
    buffer[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xFFU);
    buffer[offset + 2] = static_cast<uint8_t>((value >> 16) & 0xFFU);
    buffer[offset + 3] = static_cast<uint8_t>((value >> 24) & 0xFFU);
}

void setLittleEndianI32(std::vector<uint8_t>& buffer, size_t offset, int32_t value) {
    std::memcpy(buffer.data() + offset, &value, sizeof(value));
}

std::vector<uint8_t> buildUBXMessage(uint8_t message_class,
                                     uint8_t message_id,
                                     const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> message = {0xB5, 0x62, message_class, message_id};
    appendLittleEndian<uint16_t>(message, static_cast<uint16_t>(payload.size()));
    message.insert(message.end(), payload.begin(), payload.end());

    uint8_t ck_a = 0;
    uint8_t ck_b = 0;
    for (size_t i = 2; i < message.size(); ++i) {
        ck_a = static_cast<uint8_t>(ck_a + message[i]);
        ck_b = static_cast<uint8_t>(ck_b + ck_a);
    }
    message.push_back(ck_a);
    message.push_back(ck_b);
    return message;
}

std::vector<uint8_t> buildNavPvtMessage() {
    std::vector<uint8_t> payload(92, 0);
    setLittleEndian(payload, 0, 345600000U);
    payload[20] = 3;
    payload[21] = static_cast<uint8_t>(0x01U | 0x02U | (0x02U << 6));
    payload[23] = 18;
    setLittleEndianI32(payload, 24, 1391234567);
    setLittleEndianI32(payload, 28, 356543210);
    setLittleEndianI32(payload, 32, 12345);
    setLittleEndian(payload, 40, 1500U);
    setLittleEndian(payload, 44, 2300U);
    return buildUBXMessage(0x01, 0x07, payload);
}

struct RawxMeasurement {
    double pseudorange = 0.0;
    double carrier_phase = 0.0;
    float doppler = 0.0f;
    uint8_t gnss_id = 0;
    uint8_t sv_id = 0;
    uint8_t sig_id = 0;
    uint16_t locktime = 500;
    uint8_t cno = 45;
    uint8_t trk_stat = 0x03;
};

std::vector<uint8_t> buildRawxMessage(const std::vector<RawxMeasurement>& measurements,
                                      double tow = 345600.125,
                                      uint16_t week = 2200) {
    std::vector<uint8_t> payload;
    appendLittleEndian<double>(payload, tow);
    appendLittleEndian<uint16_t>(payload, week);
    payload.push_back(18);
    payload.push_back(static_cast<uint8_t>(measurements.size()));
    payload.push_back(0x01);
    payload.push_back(0x01);
    payload.push_back(0x00);
    payload.push_back(0x00);

    for (const auto& measurement : measurements) {
        appendLittleEndian<double>(payload, measurement.pseudorange);
        appendLittleEndian<double>(payload, measurement.carrier_phase);
        appendLittleEndian<float>(payload, measurement.doppler);
        payload.push_back(measurement.gnss_id);
        payload.push_back(measurement.sv_id);
        payload.push_back(measurement.sig_id);
        payload.push_back(0);
        appendLittleEndian<uint16_t>(payload, measurement.locktime);
        payload.push_back(measurement.cno);
        payload.push_back(0);
        payload.push_back(0);
        payload.push_back(0);
        payload.push_back(measurement.trk_stat);
        payload.push_back(0);
    }

    return buildUBXMessage(0x02, 0x15, payload);
}

std::vector<uint8_t> buildRawxMessage() {
    return buildRawxMessage({RawxMeasurement{
        20200000.25, 110000.5, -1234.5f, 0, 12, 0, 500, 45, 0x03
    }});
}

std::vector<uint8_t> buildMixedRawxMessage() {
    return buildRawxMessage({
        RawxMeasurement{20200000.25, 110000.5, -1234.5f, 0, 12, 0, 500, 45, 0x03},
        RawxMeasurement{21400000.75, 120000.25, -432.5f, 2, 5, 0, 480, 42, 0x03},
        RawxMeasurement{22300000.50, 130000.75, 125.0f, 6, 7, 2, 460, 41, 0x03},
        RawxMeasurement{23400000.00, 140000.125, -55.0f, 3, 19, 0, 440, 40, 0x03},
        RawxMeasurement{24500000.25, 150000.875, 8.0f, 5, 3, 0, 420, 39, 0x03},
    });
}

std::vector<uint8_t> buildSfrbxMessage(uint8_t gnss_id,
                                       uint8_t sv_id,
                                       uint8_t frequency_id,
                                       uint8_t channel,
                                       const std::vector<uint32_t>& words) {
    std::vector<uint8_t> payload = {
        0x02,  // version
        static_cast<uint8_t>(words.size()),
        channel,
        0x00,  // reserved
        gnss_id,
        sv_id,
        0x00,  // reserved
        frequency_id,
    };
    for (const uint32_t word : words) {
        appendLittleEndian<uint32_t>(payload, word);
    }
    return buildUBXMessage(0x02, 0x13, payload);
}

std::vector<uint8_t> buildGpsSfrbxMessage() {
    return buildSfrbxMessage(0x00, 0x0C, 0x00, 0x01,
                             {0x8B0000AAU, 0x00000500U, 0xCAFEBABEU});
}

std::vector<uint8_t> buildBeiDouGeoSfrbxMessage() {
    return buildSfrbxMessage(0x03, 0x03, 0x00, 0x01,
                             {0x00001000U, 0x00028000U, 0x00000000U});
}

}  // namespace

TEST(UBXDecoderTest, RejectsMessageWithInvalidChecksum) {
    io::UBXDecoder decoder;
    auto message = buildNavPvtMessage();
    message.back() ^= 0x55U;

    const auto decoded = decoder.decode(message.data(), message.size());

    EXPECT_TRUE(decoded.empty());
    EXPECT_EQ(decoder.getStats().checksum_errors, 1U);
}

TEST(UBXDecoderTest, DecodesNavPvtMessage) {
    io::UBXDecoder decoder;
    const auto message = buildNavPvtMessage();

    const auto decoded = decoder.decode(message.data(), message.size());
    ASSERT_EQ(decoded.size(), 1U);
    io::UBXNavPVT nav_pvt;
    ASSERT_TRUE(decoder.decodeNavPVT(decoded.front(), nav_pvt));

    EXPECT_EQ(nav_pvt.fix_type, 3);
    EXPECT_TRUE(nav_pvt.gnss_fix_ok);
    EXPECT_TRUE(nav_pvt.differential_solution);
    EXPECT_EQ(nav_pvt.carrier_solution, 2);
    EXPECT_EQ(nav_pvt.num_sv, 18);
    EXPECT_TRUE(nav_pvt.valid_position);
    EXPECT_NEAR(nav_pvt.position_geodetic.longitude * 180.0 / M_PI, 139.1234567, 1e-7);
    EXPECT_NEAR(nav_pvt.position_geodetic.latitude * 180.0 / M_PI, 35.6543210, 1e-7);
    EXPECT_NEAR(nav_pvt.position_geodetic.height, 12.345, 1e-3);
    EXPECT_NEAR(nav_pvt.horizontal_accuracy_m, 1.5, 1e-6);
    EXPECT_NEAR(nav_pvt.vertical_accuracy_m, 2.3, 1e-6);
}

TEST(UBXDecoderTest, DecodesRawxObservationEpoch) {
    io::UBXDecoder decoder;
    const auto rawx_message = buildRawxMessage();

    const auto decoded = decoder.decode(rawx_message.data(), rawx_message.size());
    ASSERT_EQ(decoded.size(), 1U);

    ObservationData obs_data;
    ASSERT_TRUE(decoder.decodeRawx(decoded.front(), obs_data));
    ASSERT_EQ(obs_data.observations.size(), 1U);

    const Observation& obs = obs_data.observations.front();
    EXPECT_EQ(obs_data.time.week, 2200);
    EXPECT_NEAR(obs_data.time.tow, 345600.125, 1e-9);
    EXPECT_EQ(obs.satellite.system, GNSSSystem::GPS);
    EXPECT_EQ(obs.satellite.prn, 12);
    EXPECT_EQ(obs.signal, SignalType::GPS_L1CA);
    EXPECT_TRUE(obs.has_pseudorange);
    EXPECT_TRUE(obs.has_carrier_phase);
    EXPECT_TRUE(obs.has_doppler);
    EXPECT_NEAR(obs.pseudorange, 20200000.25, 1e-6);
    EXPECT_NEAR(obs.carrier_phase, 110000.5, 1e-6);
    EXPECT_NEAR(obs.doppler, -1234.5, 1e-3);
    EXPECT_DOUBLE_EQ(obs.snr, 45.0);
}

TEST(UBXDecoderTest, AppliesLastNavPositionToRawxEpoch) {
    io::UBXDecoder decoder;
    const auto rawx_message = buildRawxMessage();
    const auto nav_pvt_message = buildNavPvtMessage();

    auto decoded_rawx = decoder.decode(rawx_message.data(), rawx_message.size());
    ASSERT_EQ(decoded_rawx.size(), 1U);
    ObservationData first_obs;
    ASSERT_TRUE(decoder.decodeRawx(decoded_rawx.front(), first_obs));

    auto decoded_nav = decoder.decode(nav_pvt_message.data(), nav_pvt_message.size());
    ASSERT_EQ(decoded_nav.size(), 1U);
    io::UBXNavPVT nav_pvt;
    ASSERT_TRUE(decoder.decodeNavPVT(decoded_nav.front(), nav_pvt));
    EXPECT_TRUE(nav_pvt.valid_time);
    EXPECT_EQ(nav_pvt.time.week, 2200);

    decoded_rawx = decoder.decode(rawx_message.data(), rawx_message.size());
    ASSERT_EQ(decoded_rawx.size(), 1U);
    ObservationData positioned_obs;
    ASSERT_TRUE(decoder.decodeRawx(decoded_rawx.front(), positioned_obs));
    EXPECT_GT(positioned_obs.receiver_position.norm(), 1000.0);
}

TEST(UBXDecoderTest, DecodesMixedGnssRawxObservationEpoch) {
    io::UBXDecoder decoder;
    const auto rawx_message = buildMixedRawxMessage();

    const auto decoded = decoder.decode(rawx_message.data(), rawx_message.size());
    ASSERT_EQ(decoded.size(), 1U);

    ObservationData obs_data;
    ASSERT_TRUE(decoder.decodeRawx(decoded.front(), obs_data));
    ASSERT_EQ(obs_data.observations.size(), 5U);

    EXPECT_TRUE(obs_data.hasObservation(SatelliteId(GNSSSystem::GPS, 12), SignalType::GPS_L1CA));
    EXPECT_TRUE(obs_data.hasObservation(SatelliteId(GNSSSystem::Galileo, 5), SignalType::GAL_E1));
    EXPECT_TRUE(obs_data.hasObservation(SatelliteId(GNSSSystem::GLONASS, 7), SignalType::GLO_L2CA));
    EXPECT_TRUE(obs_data.hasObservation(SatelliteId(GNSSSystem::BeiDou, 19), SignalType::BDS_B1I));
    EXPECT_TRUE(obs_data.hasObservation(SatelliteId(GNSSSystem::QZSS, 3), SignalType::QZS_L1CA));

    const Observation* galileo =
        obs_data.getObservation(SatelliteId(GNSSSystem::Galileo, 5), SignalType::GAL_E1);
    ASSERT_NE(galileo, nullptr);
    EXPECT_NEAR(galileo->pseudorange, 21400000.75, 1e-6);

    const Observation* glonass =
        obs_data.getObservation(SatelliteId(GNSSSystem::GLONASS, 7), SignalType::GLO_L2CA);
    ASSERT_NE(glonass, nullptr);
    EXPECT_NEAR(glonass->doppler, 125.0, 1e-3);
}

TEST(UBXDecoderTest, DecodesSfrbxMessage) {
    io::UBXDecoder decoder;
    const auto sfrbx_message = buildGpsSfrbxMessage();

    const auto decoded = decoder.decode(sfrbx_message.data(), sfrbx_message.size());
    ASSERT_EQ(decoded.size(), 1U);

    io::UBXSfrbx sfrbx;
    ASSERT_TRUE(decoder.decodeSfrbx(decoded.front(), sfrbx));
    EXPECT_EQ(sfrbx.system, GNSSSystem::GPS);
    EXPECT_EQ(sfrbx.sv_id, 12);
    EXPECT_EQ(sfrbx.channel, 1);
    EXPECT_EQ(sfrbx.frequency_id, 0);
    ASSERT_EQ(sfrbx.words.size(), 3U);
    EXPECT_EQ(sfrbx.words[0], 0x8B0000AAU);
    EXPECT_EQ(sfrbx.words[1], 0x00000500U);
    EXPECT_EQ(sfrbx.words[2], 0xCAFEBABEU);
}

TEST(UBXUtilsTest, MapsMessageNamesAndSignals) {
    EXPECT_EQ(io::ubx_utils::getMessageName(0x01, 0x07), "UBX-NAV-PVT");
    EXPECT_EQ(io::ubx_utils::getMessageName(0x02, 0x15), "UBX-RXM-RAWX");
    EXPECT_EQ(io::ubx_utils::getMessageName(0x02, 0x13), "UBX-RXM-SFRBX");

    SignalType signal_type = SignalType::GPS_L1CA;
    EXPECT_TRUE(io::ubx_utils::getSignalType(0, 6, signal_type));
    EXPECT_EQ(signal_type, SignalType::GPS_L5);
    EXPECT_EQ(io::ubx_utils::getSystemFromGnssId(2), GNSSSystem::Galileo);
}

TEST(UBXUtilsTest, DecodesSfrbxFrameInfoAcrossConstellations) {
    io::UBXDecoder decoder;
    io::UBXSfrbx gps_sfrbx;
    io::UBXSfrbx bds_geo_sfrbx;

    const auto gps_message = buildGpsSfrbxMessage();
    const auto gps_messages = decoder.decode(gps_message.data(), gps_message.size());
    ASSERT_EQ(gps_messages.size(), 1U);
    ASSERT_TRUE(decoder.decodeSfrbx(gps_messages.front(), gps_sfrbx));

    const auto bds_geo_message = buildBeiDouGeoSfrbxMessage();
    const auto bds_messages = decoder.decode(bds_geo_message.data(), bds_geo_message.size());
    ASSERT_EQ(bds_messages.size(), 1U);
    ASSERT_TRUE(decoder.decodeSfrbx(bds_messages.front(), bds_geo_sfrbx));

    io::UBXSfrbxFrameInfo frame_info;
    ASSERT_TRUE(io::ubx_utils::decodeSfrbxFrameInfo(gps_sfrbx, frame_info));
    EXPECT_TRUE(frame_info.valid);
    EXPECT_EQ(frame_info.kind, io::UBXSfrbxFrameInfo::Kind::GPS_LNAV);
    EXPECT_EQ(frame_info.frame_id, 5);
    EXPECT_FALSE(frame_info.has_page_id);
    EXPECT_STREQ(io::ubx_utils::getSfrbxFrameKindName(frame_info.kind), "GPS_LNAV");

    ASSERT_TRUE(io::ubx_utils::decodeSfrbxFrameInfo(bds_geo_sfrbx, frame_info));
    EXPECT_TRUE(frame_info.valid);
    EXPECT_EQ(frame_info.kind, io::UBXSfrbxFrameInfo::Kind::BDS_D2);
    EXPECT_EQ(frame_info.frame_id, 1);
    EXPECT_TRUE(frame_info.has_page_id);
    EXPECT_EQ(frame_info.page_id, 10);
    EXPECT_STREQ(io::ubx_utils::getSfrbxFrameKindName(frame_info.kind), "BDS_D2");
}

TEST(UBXStreamDecoderTest, StreamsChunkedNavPvtAndRawxMessages) {
    io::UBXStreamDecoder decoder;
    const auto nav = buildNavPvtMessage();
    const auto rawx = buildRawxMessage();

    std::vector<uint8_t> combined;
    combined.insert(combined.end(), nav.begin(), nav.end());
    combined.insert(combined.end(), rawx.begin(), rawx.end());

    std::vector<io::UBXStreamDecoder::Event> events;
    EXPECT_FALSE(decoder.pushBytes(combined.data(), 5, events));
    EXPECT_TRUE(events.empty());

    EXPECT_TRUE(decoder.pushBytes(combined.data() + 5, nav.size() - 5, events));
    ASSERT_EQ(events.size(), 1U);
    EXPECT_TRUE(events.front().has_message);
    EXPECT_TRUE(events.front().has_nav_pvt);
    EXPECT_FALSE(events.front().has_observation);
    EXPECT_EQ(events.front().message.message_class, 0x01);
    EXPECT_EQ(events.front().message.message_id, 0x07);
    EXPECT_EQ(events.front().nav_pvt.fix_type, 3);

    EXPECT_TRUE(decoder.pushBytes(rawx.data(), rawx.size(), events));
    ASSERT_EQ(events.size(), 1U);
    EXPECT_TRUE(events.front().has_message);
    EXPECT_FALSE(events.front().has_nav_pvt);
    EXPECT_TRUE(events.front().has_observation);
    EXPECT_EQ(events.front().message.message_class, 0x02);
    EXPECT_EQ(events.front().message.message_id, 0x15);
    ASSERT_EQ(events.front().observation.time.week, 2200);
    ASSERT_EQ(events.front().observation.observations.size(), 1U);
    EXPECT_GT(events.front().observation.receiver_position.norm(), 1000.0);
}

TEST(UBXStreamDecoderTest, StreamsMixedGnssRawxMessage) {
    io::UBXStreamDecoder decoder;
    const auto rawx = buildMixedRawxMessage();

    std::vector<io::UBXStreamDecoder::Event> events;
    EXPECT_TRUE(decoder.pushBytes(rawx.data(), rawx.size(), events));
    ASSERT_EQ(events.size(), 1U);
    EXPECT_TRUE(events.front().has_message);
    EXPECT_TRUE(events.front().has_observation);
    EXPECT_EQ(events.front().message.message_class, 0x02);
    EXPECT_EQ(events.front().message.message_id, 0x15);
    EXPECT_EQ(events.front().observation.getNumSatellites(), 5U);
    EXPECT_TRUE(events.front().observation.hasObservation(
        SatelliteId(GNSSSystem::BeiDou, 19), SignalType::BDS_B1I));
}

TEST(UBXStreamDecoderTest, StreamsSfrbxMessage) {
    io::UBXStreamDecoder decoder;
    const auto sfrbx = buildGpsSfrbxMessage();

    std::vector<io::UBXStreamDecoder::Event> events;
    EXPECT_TRUE(decoder.pushBytes(sfrbx.data(), sfrbx.size(), events));
    ASSERT_EQ(events.size(), 1U);
    EXPECT_TRUE(events.front().has_message);
    EXPECT_FALSE(events.front().has_nav_pvt);
    EXPECT_FALSE(events.front().has_observation);
    EXPECT_TRUE(events.front().has_sfrbx);
    EXPECT_EQ(events.front().message.message_class, 0x02);
    EXPECT_EQ(events.front().message.message_id, 0x13);
    EXPECT_EQ(events.front().sfrbx.system, GNSSSystem::GPS);
    EXPECT_EQ(events.front().sfrbx.sv_id, 12);
    ASSERT_EQ(events.front().sfrbx.words.size(), 3U);
    io::UBXSfrbxFrameInfo frame_info;
    ASSERT_TRUE(io::ubx_utils::decodeSfrbxFrameInfo(events.front().sfrbx, frame_info));
    EXPECT_EQ(frame_info.kind, io::UBXSfrbxFrameInfo::Kind::GPS_LNAV);
    EXPECT_EQ(frame_info.frame_id, 5);
}
