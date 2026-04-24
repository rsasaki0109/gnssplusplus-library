#include <gtest/gtest.h>

#include <libgnss++/io/qzss_l6.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

using namespace libgnss;

namespace {

constexpr int kHeaderBits = 49;
constexpr int kCssrType = 4073;
constexpr int kVendorClas = 5;
constexpr int kVendorMadoca = 2;
constexpr int kServiceClockEphemeris = 0;

void setUnsignedBits(std::vector<std::uint8_t>& data,
                     int bit_offset,
                     int bit_count,
                     std::uint64_t value) {
    for (int bit = 0; bit < bit_count; ++bit) {
        const int absolute_bit = bit_offset + bit;
        const int byte_index = absolute_bit / 8;
        const int bit_index = 7 - (absolute_bit % 8);
        const std::uint8_t mask = static_cast<std::uint8_t>(1U << bit_index);
        data[static_cast<std::size_t>(byte_index)] &= static_cast<std::uint8_t>(~mask);
        if (((value >> (bit_count - 1 - bit)) & 1U) == 1U) {
            data[static_cast<std::size_t>(byte_index)] |= mask;
        }
    }
}

void setUnsignedBits(std::array<std::uint8_t, qzss_l6::kFrameBytes>& data,
                     int bit_offset,
                     int bit_count,
                     std::uint64_t value) {
    for (int bit = 0; bit < bit_count; ++bit) {
        const int absolute_bit = bit_offset + bit;
        const int byte_index = absolute_bit / 8;
        const int bit_index = 7 - (absolute_bit % 8);
        const std::uint8_t mask = static_cast<std::uint8_t>(1U << bit_index);
        data[static_cast<std::size_t>(byte_index)] &= static_cast<std::uint8_t>(~mask);
        if (((value >> (bit_count - 1 - bit)) & 1U) == 1U) {
            data[static_cast<std::size_t>(byte_index)] |= mask;
        }
    }
}

std::uint64_t signedBits(int value, int bit_count) {
    if (value >= 0) {
        return static_cast<std::uint64_t>(value);
    }
    return (1ULL << bit_count) + static_cast<std::uint64_t>(value);
}

void appendUnsigned(std::vector<std::uint8_t>& bits,
                    int& bit_offset,
                    int bit_count,
                    std::uint64_t value) {
    setUnsignedBits(bits, bit_offset, bit_count, value);
    bit_offset += bit_count;
}

void appendSigned(std::vector<std::uint8_t>& bits,
                  int& bit_offset,
                  int bit_count,
                  int value) {
    appendUnsigned(bits, bit_offset, bit_count, signedBits(value, bit_count));
}

std::vector<std::uint8_t> makeSubframeBits(int frame_count) {
    return std::vector<std::uint8_t>(
        static_cast<std::size_t>((frame_count * qzss_l6::kDataPartBits + 7) / 8),
        0);
}

struct MaskSatelliteFixture {
    int system_id = 0;
    int prn_base = 1;
    int prn = 1;
};

std::vector<std::uint8_t> buildMaskMessage(
    int tow,
    int iod,
    const std::vector<MaskSatelliteFixture>& satellites) {
    auto bits = makeSubframeBits(qzss_l6::kSubframeFrames);
    int offset = 0;
    appendUnsigned(bits, offset, 12, kCssrType);
    appendUnsigned(bits, offset, 4, 1);
    appendUnsigned(bits, offset, 20, static_cast<std::uint64_t>(tow));
    appendUnsigned(bits, offset, 4, 0);
    appendUnsigned(bits, offset, 1, 0);
    appendUnsigned(bits, offset, 4, static_cast<std::uint64_t>(iod));
    appendUnsigned(bits, offset, 4, static_cast<std::uint64_t>(satellites.size()));

    for (const auto& satellite : satellites) {
        appendUnsigned(bits, offset, 4, static_cast<std::uint64_t>(satellite.system_id));
        const int mask_index = satellite.prn - satellite.prn_base;
        const std::uint64_t satellite_mask = 1ULL << (39 - mask_index);
        appendUnsigned(bits, offset, 40, satellite_mask);
        appendUnsigned(bits, offset, 16, 1U << 15);
        appendUnsigned(bits, offset, 1, 0);
    }
    return bits;
}

std::vector<std::uint8_t> buildMaskMessage(int tow, int iod) {
    return buildMaskMessage(tow, iod, {MaskSatelliteFixture{0, 1, 5}});
}

std::vector<std::uint8_t> buildClockMessage(int tow_offset, int iod, int clock_lsb) {
    auto bits = makeSubframeBits(3);
    int offset = 0;
    appendUnsigned(bits, offset, 12, kCssrType);
    appendUnsigned(bits, offset, 4, 3);
    appendUnsigned(bits, offset, 12, static_cast<std::uint64_t>(tow_offset));
    appendUnsigned(bits, offset, 4, 0);
    appendUnsigned(bits, offset, 1, 0);
    appendUnsigned(bits, offset, 4, static_cast<std::uint64_t>(iod));
    appendSigned(bits, offset, 15, clock_lsb);
    return bits;
}

std::vector<std::uint8_t> buildOrbitMessage(
    int tow_offset,
    int iod,
    int orbit_iode,
    int radial_lsb,
    int along_lsb,
    int cross_lsb) {
    auto bits = makeSubframeBits(qzss_l6::kSubframeFrames);
    int offset = 0;
    appendUnsigned(bits, offset, 12, kCssrType);
    appendUnsigned(bits, offset, 4, 2);
    appendUnsigned(bits, offset, 12, static_cast<std::uint64_t>(tow_offset));
    appendUnsigned(bits, offset, 4, 0);
    appendUnsigned(bits, offset, 1, 0);
    appendUnsigned(bits, offset, 4, static_cast<std::uint64_t>(iod));
    appendUnsigned(bits, offset, 8, static_cast<std::uint64_t>(orbit_iode));
    appendSigned(bits, offset, 15, radial_lsb);
    appendSigned(bits, offset, 13, along_lsb);
    appendSigned(bits, offset, 13, cross_lsb);
    return bits;
}

std::vector<std::uint8_t> buildCombinedOrbitClockMessage(
    int tow_offset,
    int iod,
    int orbit_iode,
    int radial_lsb,
    int along_lsb,
    int cross_lsb,
    int clock_lsb) {
    auto bits = makeSubframeBits(qzss_l6::kSubframeFrames);
    int offset = 0;
    appendUnsigned(bits, offset, 12, kCssrType);
    appendUnsigned(bits, offset, 4, 11);
    appendUnsigned(bits, offset, 12, static_cast<std::uint64_t>(tow_offset));
    appendUnsigned(bits, offset, 4, 0);
    appendUnsigned(bits, offset, 1, 0);
    appendUnsigned(bits, offset, 4, static_cast<std::uint64_t>(iod));
    appendUnsigned(bits, offset, 1, 1);  // orbit
    appendUnsigned(bits, offset, 1, 1);  // clock
    appendUnsigned(bits, offset, 1, 0);  // no network mask
    appendUnsigned(bits, offset, 8, static_cast<std::uint64_t>(orbit_iode));
    appendSigned(bits, offset, 15, radial_lsb);
    appendSigned(bits, offset, 13, along_lsb);
    appendSigned(bits, offset, 13, cross_lsb);
    appendSigned(bits, offset, 15, clock_lsb);
    return bits;
}

std::vector<std::uint8_t> buildUraMessage(int tow_offset, int iod, int ura_index) {
    auto bits = makeSubframeBits(3);
    int offset = 0;
    appendUnsigned(bits, offset, 12, kCssrType);
    appendUnsigned(bits, offset, 4, 7);
    appendUnsigned(bits, offset, 12, static_cast<std::uint64_t>(tow_offset));
    appendUnsigned(bits, offset, 4, 0);
    appendUnsigned(bits, offset, 1, 0);
    appendUnsigned(bits, offset, 4, static_cast<std::uint64_t>(iod));
    appendUnsigned(bits, offset, 6, static_cast<std::uint64_t>(ura_index));
    return bits;
}

std::array<std::uint8_t, qzss_l6::kFrameBytes> buildFrame(
    const std::vector<std::uint8_t>& subframe_bits,
    int frame_index,
    int prn,
    int vendor_id,
    int facility_id = 0,
    int service_id = kServiceClockEphemeris) {
    std::array<std::uint8_t, qzss_l6::kFrameBytes> frame{};
    setUnsignedBits(frame, 0, 32, qzss_l6::kL6Preamble);
    setUnsignedBits(frame, 32, 8, static_cast<std::uint64_t>(prn));
    const int message_type = (vendor_id << 5) |
                             ((facility_id & 0x3) << 3) |
                             ((service_id & 0x1) << 2) |
                             (frame_index == 0 ? 1 : 0);
    setUnsignedBits(frame, 40, 8, static_cast<std::uint64_t>(message_type));
    setUnsignedBits(frame, 48, 1, 0);

    for (int bit = 0; bit < qzss_l6::kDataPartBits; ++bit) {
        const int source_bit = frame_index * qzss_l6::kDataPartBits + bit;
        const int source_byte = source_bit / 8;
        const int source_bit_index = 7 - (source_bit % 8);
        std::uint64_t value = 0;
        if (source_byte < static_cast<int>(subframe_bits.size())) {
            value = (subframe_bits[static_cast<std::size_t>(source_byte)] >> source_bit_index) & 1U;
        }
        setUnsignedBits(frame, kHeaderBits + bit, 1, value);
    }
    return frame;
}

}  // namespace

TEST(QzssL6DecoderTest, ClasVendorStillAssemblesFiveFrameSubframe) {
    qzss_l6::L6Decoder decoder;
    const auto mask_message = buildMaskMessage(345600, 6);

    for (int frame_index = 0; frame_index < qzss_l6::kSubframeFrames; ++frame_index) {
        const auto frame = buildFrame(mask_message, frame_index, 199, kVendorClas);
        const auto epochs = decoder.feedFrame(frame.data(), 2400);
        EXPECT_TRUE(epochs.empty());
    }

    ASSERT_EQ(decoder.maskState().satellites.size(), 1U);
    EXPECT_EQ(decoder.maskState().satellites.front().system, GNSSSystem::GPS);
    EXPECT_EQ(decoder.maskState().satellites.front().prn, 5);
}

TEST(QzssL6DecoderTest, MadocaL6EVendorAssemblesFiveFrameMask) {
    qzss_l6::L6Decoder decoder;
    const auto mask_message = buildMaskMessage(345600, 7);

    for (int frame_index = 0; frame_index < qzss_l6::kSubframeFrames; ++frame_index) {
        const auto frame = buildFrame(mask_message, frame_index, 204, kVendorMadoca);
        const auto epochs = decoder.feedFrame(frame.data(), 2401);
        EXPECT_TRUE(epochs.empty());
    }

    ASSERT_EQ(decoder.maskState().satellites.size(), 1U);
    EXPECT_EQ(decoder.maskState().satellites.front().system, GNSSSystem::GPS);
    EXPECT_EQ(decoder.maskState().satellites.front().prn, 5);
}

TEST(QzssL6DecoderTest, MadocaL6EMapsQzssAndBd3SystemIdsLikeMadocalib) {
    qzss_l6::L6Decoder decoder;
    const auto mask_message = buildMaskMessage(
        345600,
        9,
        {MaskSatelliteFixture{4, 193, 193}, MaskSatelliteFixture{7, 19, 19}});

    for (int frame_index = 0; frame_index < qzss_l6::kSubframeFrames; ++frame_index) {
        const auto frame = buildFrame(mask_message, frame_index, 204, kVendorMadoca);
        const auto epochs = decoder.feedFrame(frame.data(), 2401);
        EXPECT_TRUE(epochs.empty());
    }

    ASSERT_EQ(decoder.maskState().satellites.size(), 2U);
    EXPECT_EQ(decoder.maskState().satellites[0].system, GNSSSystem::QZSS);
    EXPECT_EQ(decoder.maskState().satellites[0].prn, 193);
    EXPECT_EQ(decoder.maskState().satellites[1].system, GNSSSystem::BeiDou);
    EXPECT_EQ(decoder.maskState().satellites[1].prn, 19);
}

TEST(QzssL6DecoderTest, MadocaL6EClockSubtypeCompletesAfterThreeFrames) {
    qzss_l6::L6Decoder decoder;
    const auto mask_message = buildMaskMessage(345600, 8);
    for (int frame_index = 0; frame_index < qzss_l6::kSubframeFrames; ++frame_index) {
        const auto frame = buildFrame(mask_message, frame_index, 204, kVendorMadoca);
        decoder.feedFrame(frame.data(), 2402);
    }

    const auto clock_message = buildClockMessage(105, 8, 625);
    for (int frame_index = 0; frame_index < 2; ++frame_index) {
        const auto frame = buildFrame(clock_message, frame_index, 204, kVendorMadoca);
        EXPECT_TRUE(decoder.feedFrame(frame.data(), 2402).empty());
    }

    const auto final_frame = buildFrame(clock_message, 2, 204, kVendorMadoca);
    const auto epochs = decoder.feedFrame(final_frame.data(), 2402);
    ASSERT_EQ(epochs.size(), 1U);
    EXPECT_EQ(epochs.front().week, 2402);
    EXPECT_DOUBLE_EQ(epochs.front().tow, 345705.0);

    const SatelliteId sat(GNSSSystem::GPS, 5);
    ASSERT_EQ(epochs.front().clocks.count(sat), 1U);
    EXPECT_NEAR(epochs.front().clocks.at(sat).dclock_m, 1.0, 1e-12);
}

TEST(QzssL6DecoderTest, ClockSubtypeIgnoresMaskIodMismatch) {
    qzss_l6::L6Decoder decoder;
    const auto mask_message = buildMaskMessage(345600, 8);
    for (int frame_index = 0; frame_index < qzss_l6::kSubframeFrames; ++frame_index) {
        const auto frame = buildFrame(mask_message, frame_index, 204, kVendorMadoca);
        decoder.feedFrame(frame.data(), 2402);
    }

    const auto stale_clock = buildClockMessage(105, 7, 625);
    for (int frame_index = 0; frame_index < 3; ++frame_index) {
        const auto frame = buildFrame(stale_clock, frame_index, 204, kVendorMadoca);
        EXPECT_TRUE(decoder.feedFrame(frame.data(), 2402).empty());
    }

    const auto clock_message = buildClockMessage(135, 8, 125);
    for (int frame_index = 0; frame_index < 2; ++frame_index) {
        const auto frame = buildFrame(clock_message, frame_index, 204, kVendorMadoca);
        EXPECT_TRUE(decoder.feedFrame(frame.data(), 2402).empty());
    }
    const auto final_frame = buildFrame(clock_message, 2, 204, kVendorMadoca);
    const auto epochs = decoder.feedFrame(final_frame.data(), 2402);
    ASSERT_EQ(epochs.size(), 1U);
    const SatelliteId sat(GNSSSystem::GPS, 5);
    ASSERT_EQ(epochs.front().clocks.count(sat), 1U);
    EXPECT_NEAR(epochs.front().clocks.at(sat).dclock_m, 0.2, 1e-12);
}

TEST(QzssL6DecoderTest, OrbitSubtypeIgnoresMaskIodMismatchBeforeClockEmit) {
    qzss_l6::L6Decoder decoder;
    const auto mask_message = buildMaskMessage(345600, 8);
    for (int frame_index = 0; frame_index < qzss_l6::kSubframeFrames; ++frame_index) {
        const auto frame = buildFrame(mask_message, frame_index, 199, kVendorClas);
        decoder.feedFrame(frame.data(), 2402);
    }

    const auto stale_orbit = buildOrbitMessage(100, 7, 77, 100, -20, 30);
    for (int frame_index = 0; frame_index < qzss_l6::kSubframeFrames; ++frame_index) {
        const auto frame = buildFrame(stale_orbit, frame_index, 199, kVendorClas);
        EXPECT_TRUE(decoder.feedFrame(frame.data(), 2402).empty());
    }

    const auto clock_message = buildClockMessage(105, 8, 625);
    for (int frame_index = 0; frame_index < qzss_l6::kSubframeFrames - 1; ++frame_index) {
        const auto frame = buildFrame(clock_message, frame_index, 199, kVendorClas);
        EXPECT_TRUE(decoder.feedFrame(frame.data(), 2402).empty());
    }
    const auto final_frame = buildFrame(
        clock_message,
        qzss_l6::kSubframeFrames - 1,
        199,
        kVendorClas);
    const auto epochs = decoder.feedFrame(final_frame.data(), 2402);
    ASSERT_EQ(epochs.size(), 1U);
    EXPECT_FALSE(epochs.front().has_orbit);
    EXPECT_TRUE(epochs.front().orbits.empty());
}

TEST(QzssL6DecoderTest, CombinedOrbitClockSubtypeIgnoresMaskIodMismatch) {
    qzss_l6::L6Decoder decoder;
    const auto mask_message = buildMaskMessage(345600, 8);
    for (int frame_index = 0; frame_index < qzss_l6::kSubframeFrames; ++frame_index) {
        const auto frame = buildFrame(mask_message, frame_index, 199, kVendorClas);
        decoder.feedFrame(frame.data(), 2402);
    }

    const auto stale_combined = buildCombinedOrbitClockMessage(105, 7, 77, 100, -20, 30, 625);
    for (int frame_index = 0; frame_index < qzss_l6::kSubframeFrames; ++frame_index) {
        const auto frame = buildFrame(stale_combined, frame_index, 199, kVendorClas);
        EXPECT_TRUE(decoder.feedFrame(frame.data(), 2402).empty());
    }

    const auto combined = buildCombinedOrbitClockMessage(135, 8, 78, 125, -25, 35, 500);
    for (int frame_index = 0; frame_index < qzss_l6::kSubframeFrames - 1; ++frame_index) {
        const auto frame = buildFrame(combined, frame_index, 199, kVendorClas);
        EXPECT_TRUE(decoder.feedFrame(frame.data(), 2402).empty());
    }
    const auto final_frame = buildFrame(combined, qzss_l6::kSubframeFrames - 1, 199, kVendorClas);
    const auto epochs = decoder.feedFrame(final_frame.data(), 2402);
    ASSERT_EQ(epochs.size(), 1U);
    const SatelliteId sat(GNSSSystem::GPS, 5);
    ASSERT_EQ(epochs.front().orbits.count(sat), 1U);
    ASSERT_EQ(epochs.front().clocks.count(sat), 1U);
    EXPECT_EQ(epochs.front().orbits.at(sat).iode, 78);
    EXPECT_NEAR(epochs.front().clocks.at(sat).dclock_m, 0.8, 1e-12);
}


TEST(QzssL6DecoderTest, MadocaL6EUraSubtypeCarriesIntoClockEpoch) {
    qzss_l6::L6Decoder decoder;
    const auto mask_message = buildMaskMessage(345600, 11);
    for (int frame_index = 0; frame_index < qzss_l6::kSubframeFrames; ++frame_index) {
        const auto frame = buildFrame(mask_message, frame_index, 204, kVendorMadoca);
        decoder.feedFrame(frame.data(), 2402);
    }

    const auto ura_message = buildUraMessage(100, 11, 9);
    for (int frame_index = 0; frame_index < 3; ++frame_index) {
        const auto frame = buildFrame(ura_message, frame_index, 204, kVendorMadoca);
        EXPECT_TRUE(decoder.feedFrame(frame.data(), 2402).empty());
    }

    const auto clock_message = buildClockMessage(105, 11, 625);
    for (int frame_index = 0; frame_index < 2; ++frame_index) {
        const auto frame = buildFrame(clock_message, frame_index, 204, kVendorMadoca);
        EXPECT_TRUE(decoder.feedFrame(frame.data(), 2402).empty());
    }

    const auto final_frame = buildFrame(clock_message, 2, 204, kVendorMadoca);
    const auto epochs = decoder.feedFrame(final_frame.data(), 2402);
    ASSERT_EQ(epochs.size(), 1U);
    const SatelliteId sat(GNSSSystem::GPS, 5);
    ASSERT_TRUE(epochs.front().has_ura);
    ASSERT_EQ(epochs.front().ura_indices.count(sat), 1U);
    EXPECT_EQ(epochs.front().ura_indices.at(sat), 9);
}

TEST(QzssL6DecoderTest, MadocaL6EClockOffsetRollsPastMaskHourBoundary) {
    qzss_l6::L6Decoder decoder;
    const auto mask_message = buildMaskMessage(176370, 10);
    for (int frame_index = 0; frame_index < qzss_l6::kSubframeFrames; ++frame_index) {
        const auto frame = buildFrame(mask_message, frame_index, 204, kVendorMadoca);
        decoder.feedFrame(frame.data(), 2360);
    }

    const auto clock_message = buildClockMessage(0, 10, 125);
    for (int frame_index = 0; frame_index < 2; ++frame_index) {
        const auto frame = buildFrame(clock_message, frame_index, 204, kVendorMadoca);
        EXPECT_TRUE(decoder.feedFrame(frame.data(), 2360).empty());
    }

    const auto final_frame = buildFrame(clock_message, 2, 204, kVendorMadoca);
    const auto epochs = decoder.feedFrame(final_frame.data(), 2360);
    ASSERT_EQ(epochs.size(), 1U);
    EXPECT_EQ(epochs.front().week, 2360);
    EXPECT_DOUBLE_EQ(epochs.front().tow, 176400.0);
}
