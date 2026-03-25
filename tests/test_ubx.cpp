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

std::vector<uint8_t> buildRawxMessage() {
    std::vector<uint8_t> payload;
    appendLittleEndian<double>(payload, 345600.125);
    appendLittleEndian<uint16_t>(payload, 2200);
    payload.push_back(18);
    payload.push_back(1);
    payload.push_back(0x01);
    payload.push_back(0x01);
    payload.push_back(0x00);
    payload.push_back(0x00);

    appendLittleEndian<double>(payload, 20200000.25);
    appendLittleEndian<double>(payload, 110000.5);
    appendLittleEndian<float>(payload, -1234.5f);
    payload.push_back(0);
    payload.push_back(12);
    payload.push_back(0);
    payload.push_back(0);
    appendLittleEndian<uint16_t>(payload, 500);
    payload.push_back(45);
    payload.push_back(0);
    payload.push_back(0);
    payload.push_back(0);
    payload.push_back(0x03);
    payload.push_back(0);

    return buildUBXMessage(0x02, 0x15, payload);
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

TEST(UBXUtilsTest, MapsMessageNamesAndSignals) {
    EXPECT_EQ(io::ubx_utils::getMessageName(0x01, 0x07), "UBX-NAV-PVT");
    EXPECT_EQ(io::ubx_utils::getMessageName(0x02, 0x15), "UBX-RXM-RAWX");

    SignalType signal_type = SignalType::GPS_L1CA;
    EXPECT_TRUE(io::ubx_utils::getSignalType(0, 6, signal_type));
    EXPECT_EQ(signal_type, SignalType::GPS_L5);
    EXPECT_EQ(io::ubx_utils::getSystemFromGnssId(2), GNSSSystem::Galileo);
}
