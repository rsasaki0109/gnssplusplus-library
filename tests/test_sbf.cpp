#include <gtest/gtest.h>
#include <cstring>
#include <vector>
#include "libgnss++/io/sbf.hpp"

using namespace libgnss;
using namespace libgnss::io;

// ============================================================================
// Helper: build a valid SBF block
// ============================================================================
static std::vector<uint8_t> makeSBFBlock(SBFBlockId id, uint8_t revision,
                                         uint32_t tow_ms, uint16_t wnc,
                                         const std::vector<uint8_t>& payload) {
    // Total length = 14 (header) + payload, padded to multiple of 4
    size_t raw_len = 14 + payload.size();
    size_t padded_len = (raw_len + 3) & ~3u;

    std::vector<uint8_t> block(padded_len, 0);

    // Sync
    block[0] = SBF_SYNC_1;
    block[1] = SBF_SYNC_2;

    // ID + revision
    uint16_t id_rev = (static_cast<uint16_t>(id) & 0x1FFF) |
                      ((static_cast<uint16_t>(revision) & 0x07) << 13);
    std::memcpy(block.data() + 4, &id_rev, 2);

    // Length
    uint16_t length = static_cast<uint16_t>(padded_len);
    std::memcpy(block.data() + 6, &length, 2);

    // TOW + WNC
    std::memcpy(block.data() + 8, &tow_ms, 4);
    std::memcpy(block.data() + 12, &wnc, 2);

    // Payload
    if (!payload.empty()) {
        std::memcpy(block.data() + 14, payload.data(), payload.size());
    }

    // Compute CRC over bytes 4..end, store at bytes 2-3
    uint16_t crc = SBFProcessor::computeCRC16(block.data() + 4, padded_len - 4);
    std::memcpy(block.data() + 2, &crc, 2);

    return block;
}

// ============================================================================
// CRC Tests
// ============================================================================

TEST(SBFCRC, ComputeAndVerify) {
    // Build a block and verify CRC roundtrips
    auto block = makeSBFBlock(SBFBlockId::DOP, 0, 100000, 2300, {0x01, 0x02});
    ASSERT_GE(block.size(), 16u);  // 14 header + 2 payload + 0 pad = 16

    // Verify by recomputing
    uint16_t stored_crc;
    std::memcpy(&stored_crc, block.data() + 2, 2);
    uint16_t computed = SBFProcessor::computeCRC16(block.data() + 4, block.size() - 4);
    EXPECT_EQ(stored_crc, computed);
}

TEST(SBFCRC, CorruptedBlock) {
    auto block = makeSBFBlock(SBFBlockId::DOP, 0, 100000, 2300, {});
    block[10] ^= 0xFF;  // corrupt a byte

    SBFProcessor proc;
    auto msgs = proc.decode(block.data(), block.size());
    EXPECT_EQ(msgs.size(), 0u);
    EXPECT_EQ(proc.getStats().crc_errors, 1u);
}

// ============================================================================
// Decode Tests
// ============================================================================

TEST(SBFDecode, EmptyPayload) {
    auto block = makeSBFBlock(SBFBlockId::EndOfPVT, 0, 200000, 2301, {});

    SBFProcessor proc;
    auto msgs = proc.decode(block.data(), block.size());
    ASSERT_EQ(msgs.size(), 1u);
    EXPECT_EQ(msgs[0].id, SBFBlockId::EndOfPVT);
    EXPECT_EQ(msgs[0].tow_ms, 200000u);
    EXPECT_EQ(msgs[0].wnc, 2301u);
    EXPECT_TRUE(msgs[0].valid);
}

TEST(SBFDecode, MultipleBlocks) {
    auto b1 = makeSBFBlock(SBFBlockId::DOP, 0, 100000, 2300, std::vector<uint8_t>(12, 0));
    auto b2 = makeSBFBlock(SBFBlockId::EndOfPVT, 0, 100100, 2300, {});

    std::vector<uint8_t> combined;
    combined.insert(combined.end(), b1.begin(), b1.end());
    combined.insert(combined.end(), b2.begin(), b2.end());

    SBFProcessor proc;
    auto msgs = proc.decode(combined.data(), combined.size());
    ASSERT_EQ(msgs.size(), 2u);
    EXPECT_EQ(msgs[0].id, SBFBlockId::DOP);
    EXPECT_EQ(msgs[1].id, SBFBlockId::EndOfPVT);
}

TEST(SBFDecode, PartialBlock) {
    auto block = makeSBFBlock(SBFBlockId::DOP, 0, 100000, 2300, std::vector<uint8_t>(12, 0));

    SBFProcessor proc;

    // Feed first half
    size_t half = block.size() / 2;
    auto msgs1 = proc.decode(block.data(), half);
    EXPECT_EQ(msgs1.size(), 0u);

    // Feed second half
    auto msgs2 = proc.decode(block.data() + half, block.size() - half);
    ASSERT_EQ(msgs2.size(), 1u);
    EXPECT_EQ(msgs2[0].id, SBFBlockId::DOP);
}

TEST(SBFDecode, GarbageBeforeSync) {
    auto block = makeSBFBlock(SBFBlockId::EndOfPVT, 0, 100000, 2300, {});
    std::vector<uint8_t> with_garbage = {0xFF, 0xAA, 0x00};
    with_garbage.insert(with_garbage.end(), block.begin(), block.end());

    SBFProcessor proc;
    auto msgs = proc.decode(with_garbage.data(), with_garbage.size());
    ASSERT_EQ(msgs.size(), 1u);
    EXPECT_GT(proc.getStats().sync_errors, 0u);
}

// ============================================================================
// PVTGeodetic Decode
// ============================================================================

TEST(SBFPvtGeodetic, DecodeAndConvert) {
    SBFPvtGeodetic pvt{};
    pvt.mode = 1;  // StandAlone
    pvt.error = 0;
    pvt.latitude = 35.6683 * M_PI / 180.0;
    pvt.longitude = 139.7917 * M_PI / 180.0;
    pvt.height = 50.0;
    pvt.undulation = 36.5f;
    pvt.vn = 1.0f;
    pvt.ve = -0.5f;
    pvt.vu = 0.1f;
    pvt.nr_sv = 15;
    pvt.h_accuracy = 250;   // 2.50 m
    pvt.v_accuracy = 400;   // 4.00 m
    pvt.rx_clk_bias = 0.001;  // 0.001 ms = 1 us

    std::vector<uint8_t> payload(sizeof(SBFPvtGeodetic));
    std::memcpy(payload.data(), &pvt, sizeof(pvt));

    auto block = makeSBFBlock(SBFBlockId::PVTGeodetic, 0, 123456000, 2300, payload);

    SBFProcessor proc;
    auto msgs = proc.decode(block.data(), block.size());
    ASSERT_EQ(msgs.size(), 1u);

    PositionSolution pos;
    ASSERT_TRUE(proc.decodeMessage(msgs[0], nullptr, nullptr, &pos));

    EXPECT_EQ(pos.status, SolutionStatus::SPP);
    EXPECT_EQ(pos.num_satellites, 15);
    EXPECT_NEAR(pos.position_geodetic.latitude, 35.6683 * M_PI / 180.0, 1e-8);
    EXPECT_NEAR(pos.position_geodetic.longitude, 139.7917 * M_PI / 180.0, 1e-8);
    EXPECT_NEAR(pos.position_geodetic.height, 50.0, 0.01);
    EXPECT_NEAR(pos.velocity_ned(0), 1.0, 0.001);
    EXPECT_NEAR(pos.velocity_ned(1), -0.5, 0.001);
    EXPECT_NEAR(pos.velocity_ned(2), -0.1, 0.001);  // vu→-vd
    EXPECT_NEAR(pos.time.tow, 123456.0, 0.01);
}

TEST(SBFPvtGeodetic, RTKFixed) {
    SBFPvtGeodetic pvt{};
    pvt.mode = 5;  // RTK Fixed
    pvt.error = 0;
    pvt.latitude = 35.0 * M_PI / 180.0;
    pvt.longitude = 139.0 * M_PI / 180.0;
    pvt.height = 10.0;
    pvt.nr_sv = 20;
    pvt.h_accuracy = 5;   // 0.05 m
    pvt.v_accuracy = 10;  // 0.10 m

    std::vector<uint8_t> payload(sizeof(SBFPvtGeodetic));
    std::memcpy(payload.data(), &pvt, sizeof(pvt));
    auto block = makeSBFBlock(SBFBlockId::PVTGeodetic, 0, 100000, 2300, payload);

    SBFProcessor proc;
    auto msgs = proc.decode(block.data(), block.size());
    PositionSolution pos;
    proc.decodeMessage(msgs[0], nullptr, nullptr, &pos);

    EXPECT_EQ(pos.status, SolutionStatus::FIXED);
}

TEST(SBFPvtGeodetic, DoNotUseValues) {
    SBFPvtGeodetic pvt{};
    pvt.mode = 0;
    pvt.latitude = -2.0e10;  // Do Not Use
    pvt.longitude = -2.0e10;

    std::vector<uint8_t> payload(sizeof(SBFPvtGeodetic));
    std::memcpy(payload.data(), &pvt, sizeof(pvt));
    auto block = makeSBFBlock(SBFBlockId::PVTGeodetic, 0, 100000, 2300, payload);

    SBFProcessor proc;
    auto msgs = proc.decode(block.data(), block.size());
    PositionSolution pos;
    proc.decodeMessage(msgs[0], nullptr, nullptr, &pos);

    EXPECT_EQ(pos.status, SolutionStatus::NONE);
}

// ============================================================================
// Signal Mapping
// ============================================================================

TEST(SBFMapping, SystemFromSignalType) {
    EXPECT_EQ(SBFProcessor::sbfSignalTypeToSystem(0x00), GNSSSystem::GPS);      // constellation=0
    EXPECT_EQ(SBFProcessor::sbfSignalTypeToSystem(0x20), GNSSSystem::GLONASS);  // constellation=1
    EXPECT_EQ(SBFProcessor::sbfSignalTypeToSystem(0x40), GNSSSystem::Galileo);  // constellation=2
    EXPECT_EQ(SBFProcessor::sbfSignalTypeToSystem(0x80), GNSSSystem::BeiDou);   // constellation=4
    EXPECT_EQ(SBFProcessor::sbfSignalTypeToSystem(0xA0), GNSSSystem::QZSS);     // constellation=5
}

TEST(SBFMapping, SignalFromSignalType) {
    EXPECT_EQ(SBFProcessor::sbfSignalTypeToSignal(0x00), SignalType::GPS_L1CA);
    EXPECT_EQ(SBFProcessor::sbfSignalTypeToSignal(0x04), SignalType::GPS_L5);
    EXPECT_EQ(SBFProcessor::sbfSignalTypeToSignal(0x20), SignalType::GLO_L1CA);
    EXPECT_EQ(SBFProcessor::sbfSignalTypeToSignal(0x40), SignalType::GAL_E1);
    EXPECT_EQ(SBFProcessor::sbfSignalTypeToSignal(0x41), SignalType::GAL_E5A);
    EXPECT_EQ(SBFProcessor::sbfSignalTypeToSignal(0x80), SignalType::BDS_B1I);
}

// ============================================================================
// Utilities
// ============================================================================

TEST(SBFUtils, BlockNames) {
    EXPECT_EQ(sbf_utils::getBlockIdName(SBFBlockId::PVTGeodetic), "PVTGeodetic");
    EXPECT_EQ(sbf_utils::getBlockIdName(SBFBlockId::MeasEpoch), "MeasEpoch");
}

TEST(SBFUtils, Classification) {
    EXPECT_TRUE(sbf_utils::isPositionBlock(SBFBlockId::PVTGeodetic));
    EXPECT_TRUE(sbf_utils::isPositionBlock(SBFBlockId::PVTCartesian));
    EXPECT_FALSE(sbf_utils::isPositionBlock(SBFBlockId::MeasEpoch));
    EXPECT_TRUE(sbf_utils::isMeasurementBlock(SBFBlockId::MeasEpoch));
    EXPECT_FALSE(sbf_utils::isMeasurementBlock(SBFBlockId::PVTGeodetic));
}

TEST(SBFStats, Tracking) {
    auto b1 = makeSBFBlock(SBFBlockId::PVTGeodetic, 0, 100000, 2300,
                           std::vector<uint8_t>(sizeof(SBFPvtGeodetic), 0));
    auto b2 = makeSBFBlock(SBFBlockId::DOP, 0, 100100, 2300,
                           std::vector<uint8_t>(sizeof(SBFDop), 0));

    std::vector<uint8_t> combined;
    combined.insert(combined.end(), b1.begin(), b1.end());
    combined.insert(combined.end(), b2.begin(), b2.end());

    SBFProcessor proc;
    proc.decode(combined.data(), combined.size());
    auto stats = proc.getStats();

    EXPECT_EQ(stats.valid_messages, 2u);
    EXPECT_EQ(stats.crc_errors, 0u);
    EXPECT_EQ(stats.message_counts[SBFBlockId::PVTGeodetic], 1u);
    EXPECT_EQ(stats.message_counts[SBFBlockId::DOP], 1u);
}

// ============================================================================
// Sentinel Values
// ============================================================================

TEST(SBFSentinel, DoNotUse) {
    EXPECT_TRUE(sbf_sentinel::isDoNotUse(-2.0e10));
    EXPECT_FALSE(sbf_sentinel::isDoNotUse(0.0));
    EXPECT_TRUE(sbf_sentinel::isDoNotUse(-2.0e10f));
    EXPECT_TRUE(sbf_sentinel::isDoNotUse(static_cast<uint16_t>(65535)));
    EXPECT_FALSE(sbf_sentinel::isDoNotUse(static_cast<uint16_t>(100)));
    EXPECT_TRUE(sbf_sentinel::isDoNotUse(static_cast<uint8_t>(255)));
}
