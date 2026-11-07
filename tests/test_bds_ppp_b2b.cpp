// BeiDou PPP-B2b decoder coverage with synthetic messages built here from the
// BDS-SIS-ICD-PPP-B2b 1.0 field layouts (no recorded messages are embedded).

#include <gtest/gtest.h>
#include <libgnss++/io/bds_ppp_b2b.hpp>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace libgnss;
using namespace libgnss::io;

namespace {

// MSB-first bit writer for one 486-bit message; finish() adds the CRC-24Q.
class MessageWriter {
public:
    explicit MessageWriter(int type) { put(static_cast<uint64_t>(type), 6); }

    void put(uint64_t value, int bits) {
        for (int i = bits - 1; i >= 0; --i) {
            set(pos_++, (value >> i) & 1U);
        }
    }
    void putSigned(int64_t value, int bits) {
        put(static_cast<uint64_t>(value) & ((uint64_t{1} << bits) - 1U), bits);
    }
    // Message header: epoch time (BDT seconds of day), 4 reserved bits, IODSSR.
    void header(int tod, int iodssr) {
        put(static_cast<uint64_t>(tod), 17);
        put(0, 4);
        put(static_cast<uint64_t>(iodssr), 2);
    }
    BdsB2bMessage finish(const GNSSTime& time, int prn = 59) {
        uint32_t crc = 0;
        for (int i = 0; i < 462; ++i) {
            crc ^= static_cast<uint32_t>(get(i)) << 23;
            crc <<= 1;
            if (crc & 0x1000000U) {
                crc ^= 0x1864CFBU;
            }
        }
        pos_ = 462;
        put(crc & 0xFFFFFFU, 24);
        message_.time = time;
        message_.prn = prn;
        return message_;
    }

private:
    void set(int bit, bool value) {
        uint8_t& byte = message_.bits[static_cast<size_t>(bit / 8)];
        const uint8_t mask = static_cast<uint8_t>(0x80U >> (bit % 8));
        byte = value ? static_cast<uint8_t>(byte | mask) : static_cast<uint8_t>(byte & ~mask);
    }
    int get(int bit) const {
        return (message_.bits[static_cast<size_t>(bit / 8)] >> (7 - bit % 8)) & 1;
    }

    BdsB2bMessage message_;
    int pos_ = 0;
};

// 2025-02-15 17:00:00 GPST (GPS week 2353, Saturday); BDT is 14 s behind.
const GNSSTime kReception(2353, 579600.0);
constexpr int kTod = 61186;  // 16:59:46 BDT = 17:00:00 GPST

// Mask: BDS C19, C20, C21 and GPS G01, G05 (slots 19, 20, 21, 64, 68).
BdsB2bMessage maskMessage(int iodp, int iodssr, const GNSSTime& time = kReception) {
    MessageWriter w(1);
    w.header(kTod, iodssr);
    w.put(static_cast<uint64_t>(iodp), 4);
    for (int slot = 1; slot <= 174; ++slot) {
        w.put(slot == 19 || slot == 20 || slot == 21 || slot == 64 || slot == 68 ? 1 : 0, 1);
    }
    return w.finish(time);
}

struct OrbitField {
    int slot;
    int iodn;
    int iod_corr;
    int64_t radial;
    int64_t along;
    int64_t cross;
};

BdsB2bMessage orbitMessage(int iodssr, std::vector<OrbitField> entries, int tod = kTod) {
    MessageWriter w(2);
    w.header(tod, iodssr);
    entries.resize(6, OrbitField{0, 0, 0, 0, 0, 0});
    for (const auto& e : entries) {
        w.put(static_cast<uint64_t>(e.slot), 9);
        w.put(static_cast<uint64_t>(e.iodn), 10);
        w.put(static_cast<uint64_t>(e.iod_corr), 3);
        w.putSigned(e.radial, 15);
        w.putSigned(e.along, 13);
        w.putSigned(e.cross, 13);
        w.put(2, 3);  // URA class
        w.put(1, 3);  // URA value
    }
    return w.finish(kReception);
}

// Clock subtype 0: the first 23 masked satellites, (IOD Corr, C0 raw) each.
BdsB2bMessage clockMessage(int iodssr, int iodp, const std::vector<std::pair<int, int64_t>>& clocks,
                           int tod = kTod) {
    MessageWriter w(4);
    w.header(tod, iodssr);
    w.put(static_cast<uint64_t>(iodp), 4);
    w.put(0, 5);
    for (int k = 0; k < 23; ++k) {
        const auto entry = k < static_cast<int>(clocks.size()) ? clocks[static_cast<size_t>(k)]
                                                                : std::make_pair(0, int64_t{0});
        w.put(static_cast<uint64_t>(entry.first), 3);
        w.putSigned(entry.second, 15);
    }
    return w.finish(kReception);
}

std::vector<BdsB2bUpdate> usableUpdates(const BdsB2bDecoder& decoder) {
    std::vector<BdsB2bUpdate> usable;
    for (const auto& update : decoder.updates()) {
        if (update.usable) usable.push_back(update);
    }
    return usable;
}

const SatelliteId kC19(GNSSSystem::BeiDou, 19);
const SatelliteId kC20(GNSSSystem::BeiDou, 20);
const SatelliteId kC21(GNSSSystem::BeiDou, 21);
const SatelliteId kG01(GNSSSystem::GPS, 1);
const SatelliteId kG05(GNSSSystem::GPS, 5);

}  // namespace

TEST(BdsPppB2bTest, SlotsSignalsAndUra) {
    EXPECT_EQ(bdsB2bSlotSatellite(1), SatelliteId(GNSSSystem::BeiDou, 1));
    EXPECT_EQ(bdsB2bSlotSatellite(63), SatelliteId(GNSSSystem::BeiDou, 63));
    EXPECT_EQ(bdsB2bSlotSatellite(64), kG01);
    EXPECT_EQ(bdsB2bSlotSatellite(100), SatelliteId(GNSSSystem::GPS, 37));
    EXPECT_EQ(bdsB2bSlotSatellite(101), SatelliteId(GNSSSystem::Galileo, 1));
    EXPECT_EQ(bdsB2bSlotSatellite(138), SatelliteId(GNSSSystem::GLONASS, 1));
    EXPECT_EQ(bdsB2bSlotSatellite(0).system, GNSSSystem::UNKNOWN);
    EXPECT_EQ(bdsB2bSlotSatellite(175).system, GNSSSystem::UNKNOWN);

    EXPECT_STREQ(bdsB2bSignalCode(GNSSSystem::BeiDou, 0), "2I");
    EXPECT_STREQ(bdsB2bSignalCode(GNSSSystem::BeiDou, 2), "1P");
    EXPECT_STREQ(bdsB2bSignalCode(GNSSSystem::BeiDou, 12), "6I");
    EXPECT_STREQ(bdsB2bSignalCode(GNSSSystem::BeiDou, 3), "");
    EXPECT_STREQ(bdsB2bSignalCode(GNSSSystem::GPS, 0), "1C");
    EXPECT_STREQ(bdsB2bSignalCode(GNSSSystem::GPS, 8), "2X");

    EXPECT_FALSE(bdsB2bUraMeters(0, 0).has_value());
    EXPECT_DOUBLE_EQ(*bdsB2bUraMeters(7, 7), 5.4665);
    EXPECT_NEAR(*bdsB2bUraMeters(2, 1), (9.0 * 1.25 - 1.0) * 1e-3, 1e-15);
}

TEST(BdsPppB2bTest, CrcRejectsCorruptedMessages) {
    BdsB2bMessage message = maskMessage(3, 1);
    EXPECT_TRUE(checkBdsB2bCrc(message));
    EXPECT_EQ(bdsB2bMessageType(message), 1);
    message.bits[20] ^= 0x10;
    EXPECT_FALSE(checkBdsB2bCrc(message));

    BdsB2bDecoder decoder;
    EXPECT_EQ(decoder.addMessage(message), -1);
    EXPECT_EQ(decoder.stats().crc_failures, 1U);
    EXPECT_EQ(decoder.iodp(), -1);
}

TEST(BdsPppB2bTest, DecodesMaskOrbitClockAndPairsByIodCorr) {
    BdsB2bDecoder decoder;
    ASSERT_EQ(decoder.addMessage(maskMessage(3, 1)), 1);
    EXPECT_EQ(decoder.iodp(), 3);
    EXPECT_EQ(decoder.iodssr(), 1);
    ASSERT_EQ(decoder.messages().size(), 1U);
    const auto& mask = decoder.messages().back();
    EXPECT_EQ(mask.mask, (std::vector<SatelliteId>{kC19, kC20, kC21, kG01, kG05}));
    // 16:59:46 BDT on the reception day is 17:00:00 GPST.
    EXPECT_EQ(mask.reference_time, kReception);

    // Orbits: C19 (IODN 0x30c -> IODE 0x0c), G01, an unmasked C22 and an
    // invalid C20 (radial = -2^14).
    ASSERT_EQ(decoder.addMessage(orbitMessage(1, {{19, 0x30c, 2, 100, -50, 25},
                                                  {64, 77, 5, -1, 2, -3},
                                                  {22, 1, 1, 7, 7, 7},
                                                  {20, 12, 0, -16384, 0, 0}})),
              2);
    const auto& orbits = decoder.messages().back().orbits;
    ASSERT_EQ(orbits.size(), 3U);  // C22 is not in the mask
    EXPECT_EQ(orbits[0].satellite, kC19);
    EXPECT_EQ(orbits[0].iodn, 0x30c);
    EXPECT_TRUE(orbits[0].available);
    EXPECT_NEAR(orbits[0].radial_m, 100 * 0.0016, 1e-12);
    EXPECT_NEAR(orbits[0].along_m, -50 * 0.0064, 1e-12);
    EXPECT_NEAR(orbits[0].cross_m, 25 * 0.0064, 1e-12);
    EXPECT_EQ(orbits[0].ura_class, 2);
    EXPECT_EQ(orbits[0].ura_value, 1);
    EXPECT_EQ(orbits[1].satellite, kG01);
    EXPECT_NEAR(orbits[1].cross_m, -3 * 0.0064, 1e-12);
    EXPECT_EQ(orbits[2].satellite, kC20);
    EXPECT_FALSE(orbits[2].available);
    EXPECT_TRUE(decoder.updates().empty());  // no clock yet

    // Clocks in mask order: C19, C20, C21, G01, G05.
    ASSERT_EQ(decoder.addMessage(clockMessage(1, 3, {{2, -300}, {0, 10}, {1, 20}, {5, 400}, {0, 5}})), 4);
    const auto& clocks = decoder.messages().back().clocks;
    ASSERT_EQ(clocks.size(), 5U);  // only masked slots
    EXPECT_EQ(clocks[0].satellite, kC19);
    EXPECT_NEAR(clocks[0].c0_m, -300 * 0.0016, 1e-12);
    EXPECT_EQ(clocks[3].satellite, kG01);
    EXPECT_EQ(clocks[3].iod_corr, 5);

    // Updates for the satellites with both an orbit and a clock: C19 and G01
    // pair by IOD Corr; C20's orbit is invalid, so its update only marks that
    // it has no usable correction.
    ASSERT_EQ(decoder.updates().size(), 3U);
    EXPECT_EQ(decoder.updates()[1].satellite, kC20);
    EXPECT_FALSE(decoder.updates()[1].usable);
    ASSERT_EQ(usableUpdates(decoder).size(), 2U);
    const BdsB2bUpdate c19 = usableUpdates(decoder)[0];
    EXPECT_EQ(c19.satellite, kC19);
    EXPECT_EQ(c19.iodn, 0x30c);
    EXPECT_EQ(c19.iode, 0x0c);
    EXPECT_EQ(c19.iod_corr, 2);
    EXPECT_EQ(c19.received, kReception);  // the clock message that completed it
    EXPECT_NEAR(c19.radial_m, 0.16, 1e-12);
    EXPECT_NEAR(c19.c0_m, -0.48, 1e-12);
    ASSERT_TRUE(c19.ura_m.has_value());
    EXPECT_EQ(usableUpdates(decoder)[1].satellite, kG01);
    EXPECT_EQ(usableUpdates(decoder)[1].iode, 77);

    // The C19 clock moves to IOD Corr 3: the orbit (still IOD Corr 2) pairs
    // with the previous clock, then with the new one once its orbit follows.
    ASSERT_EQ(decoder.addMessage(clockMessage(1, 3, {{3, -310}, {0, 10}, {1, 20}, {5, 410}, {0, 5}})), 4);
    auto usable = usableUpdates(decoder);
    ASSERT_EQ(usable.size(), 4U);
    EXPECT_EQ(usable[2].satellite, kC19);
    EXPECT_EQ(usable[2].iod_corr, 2);
    EXPECT_NEAR(usable[2].c0_m, -0.48, 1e-12);
    EXPECT_NEAR(usable[3].c0_m, 410 * 0.0016, 1e-12);
    ASSERT_EQ(decoder.addMessage(orbitMessage(1, {{19, 0x30d, 3, 101, -50, 25}})), 2);
    usable = usableUpdates(decoder);
    ASSERT_EQ(usable.size(), 5U);
    EXPECT_EQ(usable[4].iod_corr, 3);
    EXPECT_EQ(usable[4].iode, 0x0d);
    EXPECT_NEAR(usable[4].c0_m, -310 * 0.0016, 1e-12);

    // An invalid clock (raw -2^14) leaves C19 without a correction until the
    // next valid one.
    ASSERT_EQ(decoder.addMessage(clockMessage(1, 3, {{3, -16384}, {0, 10}, {1, 20}, {5, 410}, {0, 5}})), 4);
    const size_t after_invalid = decoder.updates().size();
    bool c19_cut = false;
    for (size_t i = 5; i < after_invalid; ++i) {
        if (decoder.updates()[i].satellite == kC19) {
            c19_cut = !decoder.updates()[i].usable;
        }
    }
    EXPECT_TRUE(c19_cut);
}

TEST(BdsPppB2bTest, IodssrAndIodpGateTheCorrections) {
    BdsB2bDecoder decoder;
    // Corrections before any mask are not usable.
    EXPECT_EQ(decoder.addMessage(orbitMessage(1, {{19, 1, 0, 1, 1, 1}})), 2);
    EXPECT_EQ(decoder.stats().without_mask, 1U);

    ASSERT_EQ(decoder.addMessage(maskMessage(3, 1)), 1);
    // Another IODSSR: ignored.
    EXPECT_EQ(decoder.addMessage(orbitMessage(2, {{19, 1, 0, 1, 1, 1}})), 2);
    EXPECT_EQ(decoder.stats().iodssr_mismatches, 1U);
    // Another IODP in a clock message: ignored.
    EXPECT_EQ(decoder.addMessage(clockMessage(1, 4, {{0, 1}})), 4);
    EXPECT_EQ(decoder.stats().iodp_mismatches, 1U);
    EXPECT_EQ(decoder.messages().size(), 1U);

    ASSERT_EQ(decoder.addMessage(orbitMessage(1, {{19, 1, 0, 1, 1, 1}})), 2);
    ASSERT_EQ(decoder.addMessage(clockMessage(1, 3, {{0, 1}})), 4);
    EXPECT_EQ(usableUpdates(decoder).size(), 1U);

    // A new IODP resets every correction: a clock alone does not pair with
    // the orbit decoded under the previous mask.
    ASSERT_EQ(decoder.addMessage(maskMessage(4, 2)), 1);
    EXPECT_EQ(decoder.stats().mask_changes, 1U);
    ASSERT_EQ(decoder.addMessage(clockMessage(2, 4, {{0, 1}})), 4);
    EXPECT_EQ(usableUpdates(decoder).size(), 1U);
}

TEST(BdsPppB2bTest, ReferenceTimeFollowsTheBdtDayOfReception) {
    BdsB2bDecoder decoder;
    // Received 00:00:05 GPST Sunday (BDT 23:59:51 Saturday of the previous
    // week); epoch 23:59:50 BDT is one second earlier, in the previous week.
    const GNSSTime reception(2354, 5.0);
    MessageWriter w(1);
    w.header(86390, 1);
    w.put(3, 4);
    for (int slot = 1; slot <= 174; ++slot) w.put(slot == 19 ? 1 : 0, 1);
    ASSERT_EQ(decoder.addMessage(w.finish(reception)), 1);
    EXPECT_EQ(decoder.messages().back().reference_time, GNSSTime(2354, 4.0));

    // Epoch 00:00:10 BDT, received 23:59:55 BDT the day before: next day.
    BdsB2bDecoder late;
    MessageWriter v(1);
    v.header(10, 1);
    v.put(3, 4);
    for (int slot = 1; slot <= 174; ++slot) v.put(slot == 19 ? 1 : 0, 1);
    ASSERT_EQ(late.addMessage(v.finish(GNSSTime(2353, 86400.0 + 9.0))), 1);
    EXPECT_EQ(late.messages().back().reference_time, GNSSTime(2353, 86400.0 + 24.0));
}

TEST(BdsPppB2bTest, CodeBiasesAndOneSatelliteStream) {
    BdsB2bDecoder decoder;
    ASSERT_EQ(decoder.addMessage(maskMessage(3, 1)), 1);
    EXPECT_EQ(decoder.prn(), 59);
    // A message of another GEO is counted and ignored.
    EXPECT_EQ(decoder.addMessage(maskMessage(5, 0, kReception)), 1);  // same PRN 59
    BdsB2bMessage other = maskMessage(6, 2);
    other.prn = 60;
    EXPECT_EQ(decoder.addMessage(other), -1);
    EXPECT_EQ(decoder.stats().other_satellite_messages, 1U);
    EXPECT_EQ(decoder.iodp(), 5);

    MessageWriter w(3);
    w.header(kTod, 0);
    w.put(2, 5);                  // two satellites
    w.put(19, 9); w.put(2, 4);    // C19: two signals
    w.put(0, 4); w.putSigned(-12, 12);   // B1I
    w.put(12, 4); w.putSigned(30, 12);   // B3I
    w.put(64, 9); w.put(1, 4);    // G01: one signal, invalid value
    w.put(0, 4); w.putSigned(-2048, 12);
    ASSERT_EQ(decoder.addMessage(w.finish(kReception)), 3);
    const auto& biases = decoder.messages().back().code_biases;
    ASSERT_EQ(biases.size(), 2U);
    EXPECT_EQ(biases[0].satellite, kC19);
    ASSERT_EQ(biases[0].bias_m.size(), 2U);
    EXPECT_NEAR(biases[0].bias_m.at(0), -12 * 0.017, 1e-12);
    EXPECT_NEAR(biases[0].bias_m.at(12), 30 * 0.017, 1e-12);
    EXPECT_TRUE(biases[1].bias_m.empty());

    // The held update carries the satellite's code biases.
    ASSERT_EQ(decoder.addMessage(orbitMessage(0, {{19, 1, 0, 1, 1, 1}})), 2);
    ASSERT_EQ(decoder.addMessage(clockMessage(0, 5, {{0, 1}})), 4);
    ASSERT_EQ(usableUpdates(decoder).size(), 1U);
    EXPECT_EQ(usableUpdates(decoder)[0].code_bias_m.size(), 2U);
}

TEST(BdsPppB2bTest, ReadsCssrlibTextLogs) {
    const BdsB2bMessage mask = maskMessage(3, 1);
    char hex[2 * kB2bMessageBytes + 8] = {};
    for (size_t b = 0; b < mask.bits.size(); ++b) {
        std::snprintf(hex + 2 * b, 3, "%02x", mask.bits[b]);
    }
    const auto path = std::filesystem::temp_directory_path() / "libgnss_b2b_log.txt";
    {
        std::ofstream file(path);
        file << "# wn tow prn type len hex\n";
        file << "2353\t579600\t 59\t34\t 64\t" << hex << "000000\n";
        file << "2353\t579600\t 60\t34\t 64\t" << hex << "000000\n";
        file << "2353 579601 59 34 64 zz\n";
    }
    std::vector<BdsB2bMessage> messages;
    BdsB2bReadStats stats;
    ASSERT_TRUE(readBdsB2bMessages(path.string(), messages, 59, &stats));
    EXPECT_EQ(stats.records, 3U);
    EXPECT_EQ(stats.skipped, 1U);
    ASSERT_EQ(messages.size(), 1U);
    EXPECT_EQ(messages[0].prn, 59);
    EXPECT_EQ(messages[0].time, kReception);
    EXPECT_TRUE(checkBdsB2bCrc(messages[0]));
    EXPECT_EQ(messages[0].bits, mask.bits);

    messages.clear();
    ASSERT_TRUE(readBdsB2bMessages(path.string(), messages));
    EXPECT_EQ(messages.size(), 2U);
    EXPECT_FALSE(readBdsB2bMessages((path.string() + ".missing"), messages));
    std::filesystem::remove(path);
}
