#include <gtest/gtest.h>
#include <libgnss++/core/signal_policy.hpp>
#include <libgnss++/io/rinex.hpp>
#include <libgnss++/io/ubx.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <limits>
#include <string>
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
    uint8_t freq_id = 0;
    uint8_t cp_stdev = 0;
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
        payload.push_back(measurement.freq_id);
        appendLittleEndian<uint16_t>(payload, measurement.locktime);
        payload.push_back(measurement.cno);
        payload.push_back(0);
        payload.push_back(measurement.cp_stdev);
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

// UBX-RXM-SFRBX payload as documented by u-blox (M8/F9/X20): gnssId, svId,
// sigId, freqId, numWords, chn, version, reserved, then numWords U4 words.
std::vector<uint8_t> buildSfrbxMessage(uint8_t gnss_id,
                                       uint8_t sv_id,
                                       uint8_t sig_id,
                                       uint8_t frequency_id,
                                       uint8_t channel,
                                       const std::vector<uint32_t>& words) {
    std::vector<uint8_t> payload = {
        gnss_id,
        sv_id,
        sig_id,
        frequency_id,
        static_cast<uint8_t>(words.size()),
        channel,
        0x02,  // version
        0x00,  // reserved
    };
    for (const uint32_t word : words) {
        appendLittleEndian<uint32_t>(payload, word);
    }
    return buildUBXMessage(0x02, 0x13, payload);
}

std::vector<uint8_t> buildGpsSfrbxMessage() {
    return buildSfrbxMessage(0x00, 0x0C, 0x00, 0x00, 0x01,
                             {0x8B0000AAU, 0x00000500U, 0xCAFEBABEU});
}

std::vector<uint8_t> buildBeiDouGeoSfrbxMessage() {
    return buildSfrbxMessage(0x03, 0x03, 0x01, 0x00, 0x01,
                             {0x00001000U, 0x00028000U, 0x00000000U});
}

std::vector<uint8_t> fromHex(const std::string& hex) {
    std::vector<uint8_t> bytes;
    for (size_t i = 0; i + 1 < hex.size(); i += 2) {
        bytes.push_back(static_cast<uint8_t>(std::stoul(hex.substr(i, 2), nullptr, 16)));
    }
    return bytes;
}

// Complete UBX-RXM-SFRBX frames recorded by a u-blox X20 (rtklibexplorer/
// GNSS_IMU drive_0708/gnss_1934.ubx, BSD-3-Clause, see THIRD_PARTY_NOTICES).
struct RealSfrbxFrame {
    const char* hex;
    GNSSSystem system;
    int sv_id;
    int sig_id;
    int channel;
    size_t words;
    bool legacy_navigation;
    io::UBXSfrbxFrameInfo::Kind kind;
    int frame_id;
};

const std::vector<RealSfrbxFrame>& realX20SfrbxFrames() {
    using Kind = io::UBXSfrbxFrameInfo::Kind;
    static const std::vector<RealSfrbxFrame> frames = {
        // GPS L1 C/A LNAV subframe 1.
        {"B56202133000000A00000A3C02001846C122D849CC131A006414409BA58E90EE2D28B2A6992E71C1D69A30F1CE90FDE93F00F49E68B05A35",
         GNSSSystem::GPS, 10, 0, 60, 10, true, Kind::GPS_LNAV, 1},
        // Galileo E1-B I/NAV word type 1.
        {"B562021328000204010008590200C8CF0501760ED2136F2F260000C004AA88D3D9B02A61F7434154AAAA00C04AFD7808",
         GNSSSystem::Galileo, 4, 1, 89, 8, true, Kind::GAL_INAV, 1},
        // BeiDou B3I D1 subframe 1.
        {"B56202133000031B04000A2C0200BE139038B510A0187E75F407265FBD31C0FF1E1529CF033E257F313F0A2000001C40E7328361B2028CEB",
         GNSSSystem::BeiDou, 27, 4, 44, 10, true, Kind::BDS_D1, 1},
        // GPS L5-I CNAV: same gnssId and word count as LNAV, not LNAV.
        {"B56202133000001206000A010200F2E4498B76ED72F9A144635501303A8082FE0FEEE07FFCCFFEFF030AF9FF0931E2070046A1B4C86801A4",
         GNSSSystem::GPS, 18, 6, 1, 10, false, Kind::UNKNOWN, 0},
        // Galileo E5a-I F/NAV: same word count as I/NAV, not I/NAV.
        {"B5620213280002130300081D0200FC5C3005C82A4180FD500C00A00319AD8AF381394AC6768C43AAAAAA00000C4054F6",
         GNSSSystem::Galileo, 19, 3, 29, 8, false, Kind::UNKNOWN, 0},
        // BeiDou B1C B-CNAV1 subframe 2.
        {"B56202132C0003310600094D0200BCCA1E09A563FE380067F1FF7859FA9ABC1E7802A02F0A000000007052AC00001728A9696602",
         GNSSSystem::BeiDou, 49, 6, 77, 9, false, Kind::UNKNOWN, 0},
    };
    return frames;
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

// trkStat bits: 0 prValid, 1 cpValid, 2 halfCyc valid, 3 halfCyc subtracted.
constexpr uint8_t kTrkPrCp = 0x03;
constexpr uint8_t kTrkPrCpHalfValid = 0x07;

bool decodeRawxEpoch(io::UBXDecoder& decoder,
                     const std::vector<RawxMeasurement>& measurements,
                     ObservationData& obs_data) {
    const auto bytes = buildRawxMessage(measurements);
    const auto decoded = decoder.decode(bytes.data(), bytes.size());
    return decoded.size() == 1U && decoder.decodeRawx(decoded.front(), obs_data);
}

RawxMeasurement gpsL1(uint16_t locktime, uint8_t trk_stat, uint8_t cp_stdev = 0) {
    RawxMeasurement m;
    m.pseudorange = 20200000.25;
    m.carrier_phase = 110000.5;
    m.doppler = -1234.5f;
    m.gnss_id = 0;
    m.sv_id = 12;
    m.sig_id = 0;
    m.locktime = locktime;
    m.trk_stat = trk_stat;
    m.cp_stdev = cp_stdev;
    return m;
}

TEST(UBXDecoderTest, RawxLockTimeDecreaseFlagsSlipOnLli) {
    io::UBXDecoder decoder;
    ObservationData epoch;
    ASSERT_TRUE(decodeRawxEpoch(decoder, {gpsL1(1000, kTrkPrCpHalfValid)}, epoch));
    EXPECT_EQ(epoch.observations.front().lli, 0);
    EXPECT_FALSE(epoch.observations.front().loss_of_lock);

    ASSERT_TRUE(decodeRawxEpoch(decoder, {gpsL1(2000, kTrkPrCpHalfValid)}, epoch));
    EXPECT_EQ(epoch.observations.front().lli, 0);

    // locktime fell from 2000 ms to 500 ms: lock was lost and re-acquired.
    ASSERT_TRUE(decodeRawxEpoch(decoder, {gpsL1(500, kTrkPrCpHalfValid)}, epoch));
    EXPECT_EQ(epoch.observations.front().lli & 0x01, 1);
    EXPECT_TRUE(epoch.observations.front().loss_of_lock);

    // Next epoch is continuous again: the slip is reported only once.
    ASSERT_TRUE(decodeRawxEpoch(decoder, {gpsL1(1500, kTrkPrCpHalfValid)}, epoch));
    EXPECT_EQ(epoch.observations.front().lli, 0);
}

TEST(UBXDecoderTest, RawxZeroLockTimeFlagsSlip) {
    io::UBXDecoder decoder;
    ObservationData epoch;
    ASSERT_TRUE(decodeRawxEpoch(decoder, {gpsL1(0, kTrkPrCpHalfValid)}, epoch));
    EXPECT_EQ(epoch.observations.front().lli & 0x01, 1);
    EXPECT_TRUE(epoch.observations.front().loss_of_lock);
}

TEST(UBXDecoderTest, RawxSlipCarriesForwardUntilValidPhase) {
    io::UBXDecoder decoder;
    ObservationData epoch;
    ASSERT_TRUE(decodeRawxEpoch(decoder, {gpsL1(5000, kTrkPrCpHalfValid)}, epoch));
    // Slip epoch with an invalid phase (cpValid clear): nothing to flag yet.
    ASSERT_TRUE(decodeRawxEpoch(decoder, {gpsL1(0, 0x05)}, epoch));
    EXPECT_FALSE(epoch.observations.front().has_carrier_phase);
    EXPECT_EQ(epoch.observations.front().lli, 0);
    // First valid phase afterwards carries the slip.
    ASSERT_TRUE(decodeRawxEpoch(decoder, {gpsL1(1000, kTrkPrCpHalfValid)}, epoch));
    EXPECT_TRUE(epoch.observations.front().has_carrier_phase);
    EXPECT_EQ(epoch.observations.front().lli & 0x01, 1);
}

TEST(UBXDecoderTest, RawxHalfSubtractedToggleFlagsSlip) {
    io::UBXDecoder decoder;
    ObservationData epoch;
    ASSERT_TRUE(decodeRawxEpoch(decoder, {gpsL1(1000, 0x0F)}, epoch));  // first sight, halfSub=1
    ASSERT_TRUE(decodeRawxEpoch(decoder, {gpsL1(2000, 0x0F)}, epoch));
    EXPECT_EQ(epoch.observations.front().lli, 0);
    ASSERT_TRUE(decodeRawxEpoch(decoder, {gpsL1(3000, kTrkPrCpHalfValid)}, epoch));  // halfSub 1 -> 0
    EXPECT_EQ(epoch.observations.front().lli & 0x01, 1);
}

TEST(UBXDecoderTest, RawxSlipStateIsPerSatelliteAndSignal) {
    io::UBXDecoder decoder;
    ObservationData epoch;
    RawxMeasurement l1 = gpsL1(5000, kTrkPrCpHalfValid);
    RawxMeasurement l2 = gpsL1(5000, kTrkPrCpHalfValid);
    l2.sig_id = 3;  // L2CL
    ASSERT_TRUE(decodeRawxEpoch(decoder, {l1, l2}, epoch));
    l1.locktime = 6000;
    l2.locktime = 100;  // only L2 loses lock
    ASSERT_TRUE(decodeRawxEpoch(decoder, {l1, l2}, epoch));
    ASSERT_EQ(epoch.observations.size(), 2U);
    EXPECT_EQ(epoch.observations[0].signal, SignalType::GPS_L1CA);
    EXPECT_EQ(epoch.observations[0].lli & 0x01, 0);
    EXPECT_EQ(epoch.observations[1].signal, SignalType::GPS_L2C);
    EXPECT_EQ(epoch.observations[1].lli & 0x01, 1);
}

TEST(UBXDecoderTest, RawxUnresolvedHalfCycleSetsLliBit1ButKeepsPhase) {
    io::UBXDecoder decoder;
    ObservationData epoch;
    // trkStat bit2 clear: halfCyc not valid.
    ASSERT_TRUE(decodeRawxEpoch(decoder, {gpsL1(1000, kTrkPrCp)}, epoch));
    const Observation& unresolved = epoch.observations.front();
    EXPECT_TRUE(unresolved.has_carrier_phase);
    EXPECT_EQ(unresolved.lli & 0x02, 0x02);
    EXPECT_EQ(unresolved.lli & 0x01, 0);

    ASSERT_TRUE(decodeRawxEpoch(decoder, {gpsL1(2000, kTrkPrCpHalfValid)}, epoch));
    EXPECT_TRUE(epoch.observations.front().has_carrier_phase);
    EXPECT_EQ(epoch.observations.front().lli, 0);
}

TEST(UBXDecoderTest, RawxHighCpStdevInvalidatesCarrierPhase) {
    io::UBXDecoder decoder;
    ObservationData epoch;
    // Gen8-style receiver (only sigId 0 seen): threshold is 5.
    ASSERT_TRUE(decodeRawxEpoch(decoder, {gpsL1(1000, kTrkPrCpHalfValid, 5)}, epoch));
    EXPECT_TRUE(epoch.observations.front().has_carrier_phase);
    ASSERT_TRUE(decodeRawxEpoch(decoder, {gpsL1(2000, kTrkPrCpHalfValid, 6)}, epoch));
    const Observation& obs = epoch.observations.front();
    EXPECT_FALSE(obs.has_carrier_phase);
    EXPECT_DOUBLE_EQ(obs.carrier_phase, 0.0);
    EXPECT_TRUE(obs.has_pseudorange);
    EXPECT_EQ(obs.lli, 0);
}

TEST(UBXDecoderTest, RawxGen9ReceiverUsesLooserCpStdevThreshold) {
    io::UBXDecoder decoder;
    ObservationData epoch;
    RawxMeasurement l2 = gpsL1(1000, kTrkPrCpHalfValid);
    l2.sig_id = 4;  // sigId > 1 marks a Gen9 receiver
    RawxMeasurement l1 = gpsL1(1000, kTrkPrCpHalfValid, 10);
    ASSERT_TRUE(decodeRawxEpoch(decoder, {l2, l1}, epoch));
    ASSERT_EQ(epoch.observations.size(), 2U);
    EXPECT_TRUE(epoch.observations[1].has_carrier_phase);
    l1.cp_stdev = 15;
    l1.locktime = 2000;
    l2.locktime = 2000;
    ASSERT_TRUE(decodeRawxEpoch(decoder, {l2, l1}, epoch));
    EXPECT_FALSE(epoch.observations[1].has_carrier_phase);
}

TEST(UBXDecoderTest, RawxInvalidHalfCycleMarkerPhaseIsRejected) {
    io::UBXDecoder decoder;
    ObservationData epoch;
    RawxMeasurement m = gpsL1(1000, kTrkPrCpHalfValid);
    m.carrier_phase = -0.5;
    ASSERT_TRUE(decodeRawxEpoch(decoder, {m}, epoch));
    EXPECT_FALSE(epoch.observations.front().has_carrier_phase);
    EXPECT_TRUE(epoch.observations.front().has_pseudorange);
}

TEST(UBXDecoderTest, RawxUnknownSigIdIsSkippedWithoutDuplicates) {
    io::UBXDecoder decoder;
    ObservationData epoch;
    RawxMeasurement l1 = gpsL1(1000, kTrkPrCpHalfValid);
    RawxMeasurement l1_bogus = l1;
    l1_bogus.sig_id = 2;  // not a GPS signal
    l1_bogus.pseudorange = 99999999.0;
    RawxMeasurement qzss_l1s = l1;
    qzss_l1s.gnss_id = 5;
    qzss_l1s.sv_id = 3;
    qzss_l1s.sig_id = 1;  // L1S, not representable
    RawxMeasurement e6 = l1;
    e6.gnss_id = 2;
    e6.sv_id = 5;
    e6.sig_id = 8;  // E6B, not representable
    RawxMeasurement b3i = l1;
    b3i.gnss_id = 3;
    b3i.sv_id = 19;
    b3i.sig_id = 4;  // B3I D1, not representable
    ASSERT_TRUE(decodeRawxEpoch(decoder, {l1, l1_bogus, qzss_l1s, e6, b3i}, epoch));
    ASSERT_EQ(epoch.observations.size(), 1U);
    EXPECT_EQ(epoch.observations.front().signal, SignalType::GPS_L1CA);
    EXPECT_NEAR(epoch.observations.front().pseudorange, 20200000.25, 1e-6);
}

TEST(UBXDecoderTest, RawxGlonassUnknownSlotSkippedAndFcnFilled) {
    io::UBXDecoder decoder;
    ObservationData epoch;
    RawxMeasurement unknown_slot = gpsL1(1000, kTrkPrCpHalfValid);
    unknown_slot.gnss_id = 6;
    unknown_slot.sv_id = 255;
    unknown_slot.freq_id = 7;
    RawxMeasurement glo_minus4 = unknown_slot;
    glo_minus4.sv_id = 7;
    glo_minus4.freq_id = 3;  // FCN -4
    RawxMeasurement glo_plus6 = unknown_slot;
    glo_plus6.sv_id = 9;
    glo_plus6.freq_id = 13;  // FCN +6
    RawxMeasurement glo_bad_freq = unknown_slot;
    glo_bad_freq.sv_id = 11;
    glo_bad_freq.freq_id = 200;
    ASSERT_TRUE(decodeRawxEpoch(decoder, {unknown_slot, glo_minus4, glo_plus6, glo_bad_freq}, epoch));
    ASSERT_EQ(epoch.observations.size(), 3U);
    const Observation* a = epoch.getObservation(SatelliteId(GNSSSystem::GLONASS, 7), SignalType::GLO_L1CA);
    const Observation* b = epoch.getObservation(SatelliteId(GNSSSystem::GLONASS, 9), SignalType::GLO_L1CA);
    const Observation* c = epoch.getObservation(SatelliteId(GNSSSystem::GLONASS, 11), SignalType::GLO_L1CA);
    ASSERT_NE(a, nullptr);
    ASSERT_NE(b, nullptr);
    ASSERT_NE(c, nullptr);
    EXPECT_TRUE(a->has_glonass_frequency_channel);
    EXPECT_EQ(a->glonass_frequency_channel, -4);
    EXPECT_TRUE(b->has_glonass_frequency_channel);
    EXPECT_EQ(b->glonass_frequency_channel, 6);
    EXPECT_FALSE(c->has_glonass_frequency_channel);
    EXPECT_EQ(epoch.getObservation(SatelliteId(GNSSSystem::GLONASS, 255), SignalType::GLO_L1CA), nullptr);
}

TEST(UBXDecoderTest, RawxBeiDouGeoPhaseGetsHalfCycleCorrection) {
    io::UBXDecoder decoder;
    ObservationData epoch;
    RawxMeasurement geo = gpsL1(1000, kTrkPrCpHalfValid);
    geo.gnss_id = 3;
    geo.sv_id = 3;  // C03 GEO
    geo.sig_id = 0;
    RawxMeasurement igso = geo;
    igso.sv_id = 19;
    RawxMeasurement geo_late = geo;
    geo_late.sv_id = 60;  // C60 GEO
    ASSERT_TRUE(decodeRawxEpoch(decoder, {geo, igso, geo_late}, epoch));
    ASSERT_EQ(epoch.observations.size(), 3U);
    EXPECT_NEAR(epoch.observations[0].carrier_phase, 110000.5 + 0.5, 1e-9);
    EXPECT_NEAR(epoch.observations[1].carrier_phase, 110000.5, 1e-9);
    EXPECT_NEAR(epoch.observations[2].carrier_phase, 110000.5 + 0.5, 1e-9);
}

TEST(UBXDecoderTest, ClearResetsRawxTrackingState) {
    io::UBXDecoder decoder;
    ObservationData epoch;
    ASSERT_TRUE(decodeRawxEpoch(decoder, {gpsL1(9000, kTrkPrCpHalfValid)}, epoch));
    decoder.clear();
    ASSERT_TRUE(decodeRawxEpoch(decoder, {gpsL1(100, kTrkPrCpHalfValid)}, epoch));
    EXPECT_EQ(epoch.observations.front().lli & 0x01, 0);
}

// ---------------------------------------------------------------------------
// RINEX tracking codes (u-blox interface description + RTKLIB demo5 ubx_sig())
// ---------------------------------------------------------------------------

struct ExpectedRawxSignal {
    uint8_t gnss_id;
    uint8_t sig_id;
    GNSSSystem system;
    SignalType signal;
    const char* code;
};

// Independent copy of the u-blox sigId -> RINEX 3.04 code table (demo5
// ublox.c ubx_sig()); the decoder table must match it exactly.
const ExpectedRawxSignal kExpectedRawxSignals[] = {
    {0, 0, GNSSSystem::GPS, SignalType::GPS_L1CA, "1C"},
    {0, 3, GNSSSystem::GPS, SignalType::GPS_L2C, "2L"},
    {0, 4, GNSSSystem::GPS, SignalType::GPS_L2C, "2S"},
    {0, 6, GNSSSystem::GPS, SignalType::GPS_L5, "5I"},
    {0, 7, GNSSSystem::GPS, SignalType::GPS_L5, "5Q"},
    {2, 0, GNSSSystem::Galileo, SignalType::GAL_E1, "1C"},
    {2, 1, GNSSSystem::Galileo, SignalType::GAL_E1, "1B"},
    {2, 3, GNSSSystem::Galileo, SignalType::GAL_E5A, "5I"},
    {2, 4, GNSSSystem::Galileo, SignalType::GAL_E5A, "5Q"},
    {2, 5, GNSSSystem::Galileo, SignalType::GAL_E5B, "7I"},
    {2, 6, GNSSSystem::Galileo, SignalType::GAL_E5B, "7Q"},
    {3, 0, GNSSSystem::BeiDou, SignalType::BDS_B1I, "2I"},
    {3, 1, GNSSSystem::BeiDou, SignalType::BDS_B1I, "2I"},
    {3, 2, GNSSSystem::BeiDou, SignalType::BDS_B2I, "7I"},
    {3, 3, GNSSSystem::BeiDou, SignalType::BDS_B2I, "7I"},
    {3, 5, GNSSSystem::BeiDou, SignalType::BDS_B1C, "1P"},
    {3, 6, GNSSSystem::BeiDou, SignalType::BDS_B1C, "1D"},
    {3, 7, GNSSSystem::BeiDou, SignalType::BDS_B2A, "5P"},
    {3, 8, GNSSSystem::BeiDou, SignalType::BDS_B2A, "5D"},
    {5, 0, GNSSSystem::QZSS, SignalType::QZS_L1CA, "1C"},
    {5, 4, GNSSSystem::QZSS, SignalType::QZS_L2C, "2S"},
    {5, 5, GNSSSystem::QZSS, SignalType::QZS_L2C, "2L"},
    {5, 8, GNSSSystem::QZSS, SignalType::QZS_L5, "5I"},
    {5, 9, GNSSSystem::QZSS, SignalType::QZS_L5, "5Q"},
    {6, 0, GNSSSystem::GLONASS, SignalType::GLO_L1CA, "1C"},
    {6, 2, GNSSSystem::GLONASS, SignalType::GLO_L2CA, "2C"},
    {7, 0, GNSSSystem::NavIC, SignalType::GPS_L5, "5A"},
};

RawxMeasurement rawxSignal(uint8_t gnss_id,
                           uint8_t sv_id,
                           uint8_t sig_id,
                           double pseudorange,
                           double carrier_phase = 123456.5,
                           uint8_t trk_stat = kTrkPrCpHalfValid) {
    RawxMeasurement m;
    m.pseudorange = pseudorange;
    m.carrier_phase = carrier_phase;
    m.doppler = -100.0f;
    m.gnss_id = gnss_id;
    m.sv_id = sv_id;
    m.sig_id = sig_id;
    m.locktime = 1000;
    m.trk_stat = trk_stat;
    return m;
}

TEST(UBXUtilsTest, TrackingCodeTableMatchesSignalTypeAndRinexPolicy) {
    for (const auto& expected : kExpectedRawxSignals) {
        SCOPED_TRACE(std::string("gnssId ") + std::to_string(expected.gnss_id) + " sigId " +
                     std::to_string(expected.sig_id));
        EXPECT_STREQ(io::ubx_utils::getRinexTrackingCode(expected.gnss_id, expected.sig_id),
                     expected.code);
        SignalType signal = SignalType::SIGNAL_TYPE_COUNT;
        ASSERT_TRUE(io::ubx_utils::getSignalType(expected.gnss_id, expected.sig_id, signal));
        EXPECT_EQ(signal, expected.signal);
        // The RINEX reader / RTCM tables must map the same code to the same
        // SignalType, so all three inputs agree for one physical signal.
        SignalType from_code = SignalType::SIGNAL_TYPE_COUNT;
        ASSERT_TRUE(signal_policy::trySignalForObservationType(
            expected.system, std::string("C") + expected.code, from_code));
        EXPECT_EQ(from_code, expected.signal);
    }
    // SBAS L1 C/A has a code but no SignalType; unknown pairs have neither.
    EXPECT_STREQ(io::ubx_utils::getRinexTrackingCode(1, 0), "1C");
    EXPECT_STREQ(io::ubx_utils::getRinexTrackingCode(1, 1), "");
    EXPECT_STREQ(io::ubx_utils::getRinexTrackingCode(0, 1), "");
    EXPECT_STREQ(io::ubx_utils::getRinexTrackingCode(0, 5), "");
    EXPECT_STREQ(io::ubx_utils::getRinexTrackingCode(2, 8), "");   // E6B
    EXPECT_STREQ(io::ubx_utils::getRinexTrackingCode(3, 4), "");   // B3I D1
    EXPECT_STREQ(io::ubx_utils::getRinexTrackingCode(5, 1), "");   // L1S
    EXPECT_STREQ(io::ubx_utils::getRinexTrackingCode(4, 0), "");   // IMES
    EXPECT_STREQ(io::ubx_utils::getRinexTrackingCode(9, 0), "");
}

TEST(UBXDecoderTest, RawxFillsRinexObservationTypesForEverySupportedSignal) {
    io::UBXDecoder decoder;
    ObservationData epoch;
    std::vector<RawxMeasurement> measurements;
    uint8_t sv = 1;
    for (const auto& expected : kExpectedRawxSignals) {
        // One satellite per signal so nothing collides.
        measurements.push_back(rawxSignal(expected.gnss_id, sv++, expected.sig_id,
                                          2.0e7 + sv));
    }
    ASSERT_TRUE(decodeRawxEpoch(decoder, measurements, epoch));
    ASSERT_EQ(epoch.observations.size(), measurements.size());
    ASSERT_EQ(epoch.rinex_tracking_observations.size(), measurements.size());

    sv = 1;
    for (const auto& expected : kExpectedRawxSignals) {
        SCOPED_TRACE(std::string("gnssId ") + std::to_string(expected.gnss_id) + " sigId " +
                     std::to_string(expected.sig_id));
        const SatelliteId sat(expected.system, sv++);
        const Observation* selected = epoch.getObservation(sat, expected.signal);
        ASSERT_NE(selected, nullptr);
        EXPECT_EQ(selected->pseudorange_observation_type, std::string("C") + expected.code);
        EXPECT_EQ(selected->carrier_phase_observation_type, std::string("L") + expected.code);
        EXPECT_EQ(selected->code, expected.sig_id);
        const Observation* tracked = epoch.getRinexTrackingObservation(sat, expected.code);
        ASSERT_NE(tracked, nullptr);
        EXPECT_EQ(tracked->signal, expected.signal);
        EXPECT_EQ(tracked->pseudorange_observation_type, selected->pseudorange_observation_type);
        EXPECT_DOUBLE_EQ(tracked->pseudorange, selected->pseudorange);
    }
}

TEST(UBXDecoderTest, RawxObservationTypesFollowWhichMeasurementsAreValid) {
    io::UBXDecoder decoder;
    ObservationData epoch;
    RawxMeasurement code_only = rawxSignal(0, 5, 3, 22000000.0, 0.0, 0x01);   // L2 CL, no phase
    RawxMeasurement doppler_only = rawxSignal(0, 6, 0, 0.0, 0.0, 0x00);       // L1, nothing valid
    ASSERT_TRUE(decodeRawxEpoch(decoder, {code_only, doppler_only}, epoch));
    const Observation* l2 = epoch.getObservation(SatelliteId(GNSSSystem::GPS, 5),
                                                 SignalType::GPS_L2C);
    ASSERT_NE(l2, nullptr);
    EXPECT_EQ(l2->pseudorange_observation_type, "C2L");
    EXPECT_TRUE(l2->carrier_phase_observation_type.empty());
    // Doppler-only rows carry no C/L provenance (as in the RTCM decoder) but
    // are still keyed by their tracking code.
    const Observation* l1 = epoch.getObservation(SatelliteId(GNSSSystem::GPS, 6),
                                                 SignalType::GPS_L1CA);
    ASSERT_NE(l1, nullptr);
    EXPECT_TRUE(l1->pseudorange_observation_type.empty());
    EXPECT_TRUE(l1->carrier_phase_observation_type.empty());
    EXPECT_NE(epoch.getRinexTrackingObservation(SatelliteId(GNSSSystem::GPS, 6), "1C"), nullptr);
}

TEST(UBXDecoderTest, RawxGpsL2ClAndCmKeepDistinctCodesAndClIsSelected) {
    // CM listed before CL, then the reverse: the selected observation (and so
    // the code every solver sees) must not depend on the RAWX block order.
    for (const bool cm_first : {true, false}) {
        SCOPED_TRACE(cm_first ? "CM first" : "CL first");
        io::UBXDecoder decoder;
        ObservationData epoch;
        const RawxMeasurement cl = rawxSignal(0, 9, 3, 22000100.0, 111111.5);
        const RawxMeasurement cm = rawxSignal(0, 9, 4, 22000200.0, 222222.5);
        const RawxMeasurement l1 = rawxSignal(0, 9, 0, 21000000.0, 333333.5);
        ASSERT_TRUE(decodeRawxEpoch(
            decoder, cm_first ? std::vector<RawxMeasurement>{l1, cm, cl}
                              : std::vector<RawxMeasurement>{l1, cl, cm},
            epoch));
        const SatelliteId sat(GNSSSystem::GPS, 9);

        // One observation per SignalType (no duplicate GPS_L2C rows).
        ASSERT_EQ(epoch.observations.size(), 2U);
        const Observation* l2 = epoch.getObservation(sat, SignalType::GPS_L2C);
        ASSERT_NE(l2, nullptr);
        EXPECT_EQ(l2->pseudorange_observation_type, "C2L");
        EXPECT_EQ(l2->carrier_phase_observation_type, "L2L");
        EXPECT_DOUBLE_EQ(l2->pseudorange, 22000100.0);

        // Both tracking channels stay available under their own codes.
        const Observation* tracked_l = epoch.getRinexTrackingObservation(sat, "2L");
        const Observation* tracked_s = epoch.getRinexTrackingObservation(sat, "2S");
        ASSERT_NE(tracked_l, nullptr);
        ASSERT_NE(tracked_s, nullptr);
        EXPECT_DOUBLE_EQ(tracked_l->pseudorange, 22000100.0);
        EXPECT_DOUBLE_EQ(tracked_s->pseudorange, 22000200.0);
        EXPECT_EQ(tracked_s->pseudorange_observation_type, "C2S");
        EXPECT_EQ(tracked_s->carrier_phase_observation_type, "L2S");
        EXPECT_EQ(epoch.rinex_tracking_observations.size(), 3U);
    }
}

TEST(UBXDecoderTest, RawxFallsBackToCmWhenClHasNoData) {
    io::UBXDecoder decoder;
    ObservationData epoch;
    RawxMeasurement cl = rawxSignal(0, 9, 3, 0.0, 0.0, 0x00);  // nothing valid
    cl.doppler = std::numeric_limits<float>::quiet_NaN();
    const RawxMeasurement cm = rawxSignal(0, 9, 4, 22000200.0, 222222.5);
    ASSERT_TRUE(decodeRawxEpoch(decoder, {cl, cm}, epoch));
    const Observation* l2 =
        epoch.getObservation(SatelliteId(GNSSSystem::GPS, 9), SignalType::GPS_L2C);
    ASSERT_NE(l2, nullptr);
    EXPECT_EQ(l2->pseudorange_observation_type, "C2S");
    EXPECT_EQ(l2->carrier_phase_observation_type, "L2S");
    EXPECT_EQ(epoch.observations.size(), 1U);
}

TEST(UBXDecoderTest, RawxDopplerOnlyClDoesNotDisplaceCmWithRangeAndPhase) {
    io::UBXDecoder decoder;
    ObservationData epoch;
    const RawxMeasurement cl_doppler_only = rawxSignal(0, 9, 3, 0.0, 0.0, 0x00);
    const RawxMeasurement cm = rawxSignal(0, 9, 4, 22000200.0, 222222.5);
    ASSERT_TRUE(decodeRawxEpoch(decoder, {cl_doppler_only, cm}, epoch));
    const SatelliteId sat(GNSSSystem::GPS, 9);
    const Observation* l2 = epoch.getObservation(sat, SignalType::GPS_L2C);
    ASSERT_NE(l2, nullptr);
    EXPECT_EQ(epoch.observations.size(), 1U);
    EXPECT_EQ(l2->pseudorange_observation_type, "C2S");
    EXPECT_TRUE(l2->has_pseudorange);
    // The Doppler-only channel is still available under its own code.
    const Observation* cl = epoch.getRinexTrackingObservation(sat, "2L");
    ASSERT_NE(cl, nullptr);
    EXPECT_FALSE(cl->has_pseudorange);
    EXPECT_TRUE(cl->has_doppler);
}

TEST(UBXDecoderTest, RawxDopplerOnlyChannelIsWrittenUnderItsTrackingCode) {
    io::UBXDecoder decoder;
    ObservationData epoch;
    const RawxMeasurement l1 = rawxSignal(0, 9, 0, 21000000.0, 333333.5);
    // The only L2 channel of the satellite is a Doppler-only CL.
    const RawxMeasurement cl_doppler_only = rawxSignal(0, 9, 3, 0.0, 0.0, 0x00);
    ASSERT_TRUE(decodeRawxEpoch(decoder, {l1, cl_doppler_only}, epoch));
    ASSERT_EQ(epoch.observations.size(), 2U);

    const auto path = std::filesystem::temp_directory_path() / "libgnss_ubx_codes_doppler_only.obs";
    std::filesystem::remove(path);
    io::RINEXWriter writer;
    ASSERT_TRUE(writer.createObservationFile(path.string(), io::RINEXReader::RINEXHeader{}));
    ASSERT_TRUE(writer.writeObservationEpoch(epoch));
    ASSERT_TRUE(writer.close());

    io::RINEXReader reader;
    ASSERT_TRUE(reader.open(path.string()));
    io::RINEXReader::RINEXHeader header;
    ASSERT_TRUE(reader.readHeader(header));
    reader.close();
    std::filesystem::remove(path);
    const auto& types = header.system_obs_types.at('G');
    EXPECT_NE(std::find(types.begin(), types.end(), "D2L"), types.end());
    for (const auto& type : types) {
        EXPECT_NE(type.substr(1), "2X") << "default-code column " << type;
    }
}

TEST(UBXDecoderTest, RawxQzssL2ClAndCmUseSwappedSigIds) {
    io::UBXDecoder decoder;
    ObservationData epoch;
    // QZSS numbers L2 CM = 4 and CL = 5 (the reverse of GPS).
    const RawxMeasurement cm = rawxSignal(5, 3, 4, 38000200.0);
    const RawxMeasurement cl = rawxSignal(5, 3, 5, 38000100.0);
    ASSERT_TRUE(decodeRawxEpoch(decoder, {cm, cl}, epoch));
    const SatelliteId sat(GNSSSystem::QZSS, 3);
    ASSERT_EQ(epoch.observations.size(), 1U);
    EXPECT_EQ(epoch.observations.front().pseudorange_observation_type, "C2L");
    EXPECT_DOUBLE_EQ(epoch.observations.front().pseudorange, 38000100.0);
    ASSERT_NE(epoch.getRinexTrackingObservation(sat, "2S"), nullptr);
    EXPECT_DOUBLE_EQ(epoch.getRinexTrackingObservation(sat, "2S")->pseudorange, 38000200.0);
}

TEST(UBXDecoderTest, RawxMultiBandSatellitesKeepOneObservationPerSignalType) {
    io::UBXDecoder decoder;
    ObservationData epoch;
    std::vector<RawxMeasurement> measurements = {
        // Galileo E1 C + B, E5a I + Q, E5b Q only
        rawxSignal(2, 11, 1, 24000001.0),  // E1 B
        rawxSignal(2, 11, 0, 24000000.0),  // E1 C
        rawxSignal(2, 11, 4, 24000011.0),  // E5aQ
        rawxSignal(2, 11, 3, 24000010.0),  // E5aI
        rawxSignal(2, 11, 6, 24000020.0),  // E5bQ
        // BeiDou B1I D1 + B2I D1
        rawxSignal(3, 22, 0, 25000000.0),
        rawxSignal(3, 22, 2, 25000010.0),
        // BeiDou B1C pilot + data, B2a pilot + data
        rawxSignal(3, 33, 5, 26000000.0),  // B1 Cp
        rawxSignal(3, 33, 6, 26000001.0),  // B1 Cd
        rawxSignal(3, 33, 7, 26000010.0),  // B2 ap
        rawxSignal(3, 33, 8, 26000011.0),  // B2 ad
        // GLONASS L1 + L2
        rawxSignal(6, 4, 0, 23000000.0),
        rawxSignal(6, 4, 2, 23000010.0),
    };
    measurements[11].freq_id = 9;
    measurements[12].freq_id = 9;
    ASSERT_TRUE(decodeRawxEpoch(decoder, measurements, epoch));

    const SatelliteId gal(GNSSSystem::Galileo, 11);
    const SatelliteId bds(GNSSSystem::BeiDou, 22);
    const SatelliteId bds3(GNSSSystem::BeiDou, 33);
    const SatelliteId glo(GNSSSystem::GLONASS, 4);

    // E1 C beats E1 B, E5a I beats E5a Q (signal_policy "CABXZ" / "XIQ").
    EXPECT_EQ(epoch.getObservation(gal, SignalType::GAL_E1)->pseudorange_observation_type, "C1C");
    EXPECT_EQ(epoch.getObservation(gal, SignalType::GAL_E5A)->pseudorange_observation_type, "C5I");
    EXPECT_EQ(epoch.getObservation(gal, SignalType::GAL_E5B)->pseudorange_observation_type, "C7Q");
    EXPECT_NE(epoch.getRinexTrackingObservation(gal, "1B"), nullptr);
    EXPECT_NE(epoch.getRinexTrackingObservation(gal, "5Q"), nullptr);

    EXPECT_EQ(epoch.getObservation(bds, SignalType::BDS_B1I)->pseudorange_observation_type, "C2I");
    EXPECT_EQ(epoch.getObservation(bds, SignalType::BDS_B2I)->pseudorange_observation_type, "C7I");
    EXPECT_EQ(epoch.getObservation(bds, SignalType::BDS_B2I)->carrier_phase_observation_type, "L7I");

    // BeiDou-3 data component ranks above pilot ("DPX...").
    EXPECT_EQ(epoch.getObservation(bds3, SignalType::BDS_B1C)->pseudorange_observation_type, "C1D");
    EXPECT_EQ(epoch.getObservation(bds3, SignalType::BDS_B2A)->pseudorange_observation_type, "C5D");
    EXPECT_NE(epoch.getRinexTrackingObservation(bds3, "1P"), nullptr);
    EXPECT_NE(epoch.getRinexTrackingObservation(bds3, "5P"), nullptr);

    EXPECT_EQ(epoch.getObservation(glo, SignalType::GLO_L1CA)->pseudorange_observation_type, "C1C");
    EXPECT_EQ(epoch.getObservation(glo, SignalType::GLO_L2CA)->pseudorange_observation_type, "C2C");

    // 13 RAWX blocks -> 9 distinct (satellite, SignalType) observations, but
    // every block stays in the tracking-code map.
    EXPECT_EQ(epoch.observations.size(), 9U);
    EXPECT_EQ(epoch.rinex_tracking_observations.size(), 13U);
}

TEST(UBXDecoderTest, RawxSbasKeepsL1CaCode) {
    io::UBXDecoder decoder;
    ObservationData epoch;
    ASSERT_TRUE(decodeRawxEpoch(decoder, {rawxSignal(1, 123, 0, 38000000.0)}, epoch));
    ASSERT_EQ(epoch.observations.size(), 1U);
    EXPECT_EQ(epoch.observations.front().satellite.system, GNSSSystem::SBAS);
    EXPECT_EQ(epoch.observations.front().pseudorange_observation_type, "C1C");
    EXPECT_EQ(epoch.observations.front().carrier_phase_observation_type, "L1C");
}

TEST(UBXDecoderTest, RawxToRinexWriterKeepsL2ClAndL2CmColumnsDistinct) {
    io::UBXDecoder decoder;
    ObservationData epoch;
    const RawxMeasurement l1 = rawxSignal(0, 9, 0, 21000000.0, 333333.5);
    const RawxMeasurement cl = rawxSignal(0, 9, 3, 22000100.0, 111111.5);
    const RawxMeasurement cm = rawxSignal(0, 9, 4, 22000200.0, 222222.5);
    const RawxMeasurement l5 = rawxSignal(0, 9, 6, 23000000.0, 444444.5);
    const RawxMeasurement e1 = rawxSignal(2, 11, 0, 24000000.0, 555555.5);
    const RawxMeasurement e5b = rawxSignal(2, 11, 5, 24000010.0, 666666.5);
    const RawxMeasurement b1 = rawxSignal(3, 22, 0, 25000000.0, 777777.5);
    const RawxMeasurement b2 = rawxSignal(3, 22, 2, 25000010.0, 888888.5);
    ASSERT_TRUE(decodeRawxEpoch(decoder, {l1, cm, cl, l5, e1, e5b, b1, b2}, epoch));

    const auto path = std::filesystem::temp_directory_path() / "libgnss_ubx_codes_roundtrip.obs";
    std::filesystem::remove(path);
    io::RINEXWriter writer;
    ASSERT_TRUE(writer.createObservationFile(path.string(), io::RINEXReader::RINEXHeader{}));
    ASSERT_TRUE(writer.writeObservationEpoch(epoch));
    ASSERT_TRUE(writer.close());

    io::RINEXReader reader;
    ASSERT_TRUE(reader.open(path.string()));
    io::RINEXReader::RINEXHeader header;
    ASSERT_TRUE(reader.readHeader(header));
    const auto& gps_types = header.system_obs_types.at('G');
    for (const char* type : {"C1C", "L1C", "C2L", "L2L", "D2L", "S2L", "C2S", "L2S", "D2S", "S2S",
                             "C5I", "L5I"}) {
        EXPECT_NE(std::find(gps_types.begin(), gps_types.end(), type), gps_types.end()) << type;
    }
    EXPECT_EQ(std::find(gps_types.begin(), gps_types.end(), "C2X"), gps_types.end());
    const auto& gal_types = header.system_obs_types.at('E');
    for (const char* type : {"C1C", "L1C", "C7I", "L7I"}) {
        EXPECT_NE(std::find(gal_types.begin(), gal_types.end(), type), gal_types.end()) << type;
    }
    const auto& bds_types = header.system_obs_types.at('C');
    for (const char* type : {"C2I", "L2I", "C7I", "L7I"}) {
        EXPECT_NE(std::find(bds_types.begin(), bds_types.end(), type), bds_types.end()) << type;
    }

    ObservationData read_back;
    ASSERT_TRUE(reader.readObservationEpoch(read_back));
    reader.close();
    std::filesystem::remove(path);

    const SatelliteId gps(GNSSSystem::GPS, 9);
    const Observation* read_l = read_back.getRinexTrackingObservation(gps, "2L");
    const Observation* read_s = read_back.getRinexTrackingObservation(gps, "2S");
    ASSERT_NE(read_l, nullptr);
    ASSERT_NE(read_s, nullptr);
    EXPECT_NEAR(read_l->pseudorange, 22000100.0, 1e-3);
    EXPECT_NEAR(read_l->carrier_phase, 111111.5, 1e-3);
    EXPECT_NEAR(read_s->pseudorange, 22000200.0, 1e-3);
    EXPECT_NEAR(read_s->carrier_phase, 222222.5, 1e-3);
    EXPECT_NEAR(read_l->doppler, -100.0, 1e-3);
    EXPECT_NEAR(read_s->doppler, -100.0, 1e-3);
    EXPECT_EQ(read_l->pseudorange_observation_type, "C2L");
    EXPECT_EQ(read_s->pseudorange_observation_type, "C2S");
    // The reader selects the same L2 observation the UBX decoder did.
    const Observation* read_l2 = read_back.getObservation(gps, SignalType::GPS_L2C);
    ASSERT_NE(read_l2, nullptr);
    EXPECT_EQ(read_l2->pseudorange_observation_type, "C2L");
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
    EXPECT_EQ(sfrbx.signal_id, 0);
    EXPECT_EQ(sfrbx.channel, 1);
    EXPECT_EQ(sfrbx.frequency_id, 0);
    EXPECT_EQ(sfrbx.version, 2);
    ASSERT_EQ(sfrbx.words.size(), 3U);
    EXPECT_EQ(sfrbx.words[0], 0x8B0000AAU);
    EXPECT_EQ(sfrbx.words[1], 0x00000500U);
    EXPECT_EQ(sfrbx.words[2], 0xCAFEBABEU);
}

TEST(UBXDecoderTest, DecodesRealX20SfrbxFrames) {
    io::UBXDecoder decoder;
    for (const auto& frame : realX20SfrbxFrames()) {
        const auto bytes = fromHex(frame.hex);
        const auto decoded = decoder.decode(bytes.data(), bytes.size());
        ASSERT_EQ(decoded.size(), 1U) << frame.hex;

        io::UBXSfrbx sfrbx;
        ASSERT_TRUE(decoder.decodeSfrbx(decoded.front(), sfrbx)) << frame.hex;
        EXPECT_EQ(sfrbx.system, frame.system);
        EXPECT_EQ(sfrbx.sv_id, frame.sv_id);
        EXPECT_EQ(sfrbx.signal_id, frame.sig_id);
        EXPECT_EQ(sfrbx.frequency_id, 0);
        EXPECT_EQ(sfrbx.channel, frame.channel);
        EXPECT_EQ(sfrbx.version, 2);
        EXPECT_EQ(sfrbx.words.size(), frame.words);
        EXPECT_EQ(io::ubx_utils::isSfrbxLegacyNavigation(sfrbx), frame.legacy_navigation)
            << frame.sv_id << " sig " << frame.sig_id;

        io::UBXSfrbxFrameInfo frame_info;
        EXPECT_EQ(io::ubx_utils::decodeSfrbxFrameInfo(sfrbx, frame_info),
                  frame.legacy_navigation);
        EXPECT_EQ(frame_info.kind, frame.kind);
        EXPECT_EQ(frame_info.frame_id, frame.frame_id);
    }
}

TEST(UBXDecoderTest, DecodesSfrbxGlonassSlotAndBeiDouMessageType) {
    io::UBXDecoder decoder;
    const auto decode = [&decoder](const std::vector<uint8_t>& message) {
        io::UBXSfrbx sfrbx;
        const auto decoded = decoder.decode(message.data(), message.size());
        EXPECT_EQ(decoded.size(), 1U);
        EXPECT_TRUE(!decoded.empty() && decoder.decodeSfrbx(decoded.front(), sfrbx));
        return sfrbx;
    };

    // GLONASS L2OF, slot 7, frequency channel +1 (freqId = FCN + 7).
    const auto glonass = decode(buildSfrbxMessage(0x06, 7, 2, 8, 3, {1U, 2U, 3U, 4U}));
    EXPECT_EQ(glonass.system, GNSSSystem::GLONASS);
    EXPECT_EQ(glonass.sv_id, 7);
    EXPECT_EQ(glonass.signal_id, 2);
    EXPECT_EQ(glonass.frequency_id, 8);
    EXPECT_EQ(glonass.channel, 3);
    EXPECT_TRUE(io::ubx_utils::isSfrbxLegacyNavigation(glonass));

    // BeiDou D1/D2 from sigId, with the GEO PRN rule for sigId 0.
    const std::vector<uint32_t> words(10, 0U);
    EXPECT_TRUE(io::ubx_utils::isSfrbxBeiDouD2(decode(buildSfrbxMessage(0x03, 3, 0, 0, 1, words))));
    EXPECT_TRUE(io::ubx_utils::isSfrbxBeiDouD2(decode(buildSfrbxMessage(0x03, 60, 0, 0, 1, words))));
    EXPECT_TRUE(io::ubx_utils::isSfrbxBeiDouD2(decode(buildSfrbxMessage(0x03, 60, 1, 0, 1, words))));
    EXPECT_TRUE(io::ubx_utils::isSfrbxBeiDouD2(decode(buildSfrbxMessage(0x03, 61, 10, 0, 1, words))));
    EXPECT_FALSE(io::ubx_utils::isSfrbxBeiDouD2(decode(buildSfrbxMessage(0x03, 27, 4, 0, 1, words))));
    EXPECT_FALSE(io::ubx_utils::isSfrbxBeiDouD2(decode(buildSfrbxMessage(0x03, 27, 0, 0, 1, words))));
    EXPECT_FALSE(io::ubx_utils::isSfrbxLegacyNavigation(
        decode(buildSfrbxMessage(0x03, 27, 8, 0, 1, words))));

    // numWords beyond the payload is rejected.
    auto truncated = buildSfrbxMessage(0x00, 12, 0, 0, 1, {1U, 2U});
    std::vector<uint8_t> payload(truncated.begin() + 6, truncated.end() - 2);
    payload[4] = 3;
    io::UBXMessage message;
    message.message_class = 0x02;
    message.message_id = 0x13;
    message.payload = payload;
    io::UBXSfrbx sfrbx;
    EXPECT_FALSE(decoder.decodeSfrbx(message, sfrbx));
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
