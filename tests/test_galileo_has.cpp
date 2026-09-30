#include <gtest/gtest.h>

#include <libgnss++/io/galileo_has.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

using namespace libgnss;

namespace {

// Galileo HAS SIS ICD Issue 1.0, Annex C (Reed-Solomon decoding example):
// the 15 encoded pages of a k = 15 page message and the decoded message.
// clang-format off
const std::vector<std::pair<int, std::array<uint8_t, 53>>> kAnnexCPages = {
    {55, {132, 123, 199, 73, 235, 125, 113, 116, 36, 71, 136, 251, 69, 70, 145, 140, 0, 39, 42, 235, 193, 84, 146, 204, 110, 181, 90, 88, 128, 226, 97, 186, 227, 23, 26, 35, 221, 11, 229, 98, 252, 141, 111, 216, 142, 98, 41, 194, 158, 125, 140, 153, 223}},
    {56, {52, 154, 227, 99, 77, 33, 11, 173, 50, 147, 166, 127, 182, 33, 1, 233, 221, 84, 48, 123, 198, 121, 237, 105, 155, 213, 12, 174, 174, 197, 100, 133, 243, 248, 22, 84, 12, 174, 206, 164, 198, 22, 146, 238, 91, 24, 202, 171, 181, 189, 162, 121, 57}},
    {57, {85, 1, 29, 145, 14, 230, 225, 85, 194, 242, 140, 77, 215, 250, 214, 40, 200, 226, 106, 5, 171, 215, 135, 151, 77, 226, 225, 111, 142, 246, 176, 156, 0, 215, 18, 228, 41, 8, 34, 151, 24, 174, 236, 105, 28, 5, 39, 243, 194, 63, 128, 181, 19}},
    {58, {44, 163, 27, 35, 21, 83, 238, 106, 156, 122, 59, 255, 250, 132, 43, 45, 12, 243, 8, 9, 16, 185, 194, 2, 126, 136, 115, 220, 237, 47, 141, 167, 212, 35, 164, 47, 217, 206, 88, 195, 238, 68, 125, 44, 175, 49, 177, 138, 4, 213, 165, 186, 120}},
    {59, {55, 190, 96, 216, 35, 121, 141, 182, 26, 28, 152, 34, 238, 248, 75, 122, 213, 237, 99, 213, 34, 61, 152, 173, 145, 204, 133, 143, 64, 117, 119, 92, 224, 76, 187, 36, 160, 208, 177, 95, 127, 213, 58, 214, 134, 44, 121, 248, 82, 63, 169, 191, 75}},
    {174, {187, 28, 69, 29, 89, 4, 160, 228, 22, 185, 43, 88, 154, 12, 86, 206, 43, 199, 115, 152, 40, 239, 11, 192, 73, 228, 145, 24, 154, 41, 63, 49, 40, 36, 224, 176, 100, 94, 31, 100, 152, 109, 111, 135, 185, 118, 207, 58, 18, 247, 59, 144, 33}},
    {175, {117, 25, 72, 154, 251, 194, 111, 69, 202, 191, 253, 159, 120, 178, 246, 68, 171, 41, 251, 163, 124, 202, 254, 239, 152, 25, 2, 5, 204, 223, 192, 231, 250, 120, 193, 179, 234, 80, 108, 166, 166, 167, 210, 195, 99, 135, 159, 118, 132, 143, 164, 128, 36}},
    {176, {143, 12, 156, 52, 139, 203, 193, 61, 89, 3, 53, 84, 14, 168, 101, 194, 207, 61, 113, 59, 188, 39, 200, 99, 26, 41, 88, 222, 211, 134, 178, 117, 71, 15, 136, 150, 150, 65, 88, 124, 204, 128, 23, 28, 51, 166, 204, 221, 251, 63, 53, 44, 190}},
    {187, {203, 226, 36, 10, 145, 27, 54, 129, 243, 142, 43, 63, 242, 57, 243, 98, 229, 59, 74, 201, 41, 44, 96, 199, 124, 97, 197, 70, 118, 78, 134, 66, 106, 138, 68, 197, 64, 140, 187, 91, 201, 10, 138, 135, 16, 254, 109, 113, 144, 220, 128, 204, 93}},
    {188, {29, 55, 158, 167, 195, 223, 144, 158, 158, 116, 87, 219, 101, 36, 71, 28, 189, 52, 215, 17, 199, 92, 176, 139, 74, 132, 108, 3, 25, 126, 46, 191, 226, 239, 14, 161, 44, 70, 247, 253, 202, 246, 58, 36, 35, 29, 77, 144, 52, 14, 217, 139, 221}},
    {239, {122, 57, 40, 21, 48, 65, 99, 21, 77, 50, 204, 30, 233, 166, 117, 3, 48, 3, 115, 250, 224, 78, 143, 108, 245, 144, 255, 199, 147, 114, 161, 38, 145, 41, 107, 172, 132, 82, 95, 202, 166, 152, 75, 83, 88, 143, 25, 25, 186, 202, 151, 159, 222}},
    {240, {125, 19, 56, 207, 112, 92, 184, 147, 239, 181, 113, 209, 24, 245, 173, 57, 173, 51, 3, 160, 148, 255, 182, 92, 140, 168, 146, 194, 234, 61, 53, 190, 137, 15, 91, 228, 231, 9, 111, 222, 52, 62, 205, 189, 90, 185, 129, 222, 74, 19, 154, 94, 29}},
    {241, {161, 204, 117, 222, 253, 61, 201, 66, 207, 106, 21, 166, 117, 149, 224, 164, 249, 50, 45, 172, 71, 205, 29, 87, 112, 81, 177, 95, 215, 130, 214, 162, 83, 43, 182, 9, 188, 112, 183, 111, 5, 174, 231, 176, 103, 151, 117, 7, 232, 167, 19, 33, 234}},
    {252, {207, 147, 205, 21, 140, 244, 31, 178, 149, 173, 157, 33, 161, 85, 130, 130, 237, 116, 136, 51, 54, 137, 106, 123, 126, 234, 208, 57, 145, 34, 116, 229, 209, 226, 26, 86, 63, 239, 245, 210, 21, 211, 61, 189, 43, 85, 215, 103, 160, 170, 234, 163, 56}},
    {253, {215, 200, 167, 19, 210, 166, 18, 96, 224, 77, 5, 145, 106, 148, 222, 103, 157, 196, 233, 132, 109, 61, 229, 187, 163, 152, 17, 62, 27, 210, 42, 67, 181, 2, 23, 108, 68, 206, 189, 76, 58, 39, 164, 43, 254, 9, 87, 41, 18, 228, 135, 212, 165}},
};

const char* const kAnnexCMessageHex =
    "000cc00b20ffdfffff008100f7ffff7df55ffdfe0beee8a79a41241000a6000a01a0128040020020"
    "0113fbc041febbf00080080042ff6822fea21807c193f7598035fd7f6a2f00080080016ff90287e7"
    "967f702580587fee217a10c9dfcc0e7f651df577d981603ffe4147f903ff9df7805c15ff9fdcff80"
    "08004004000a002407ff9d7c07df7ffe2b5fdcee305519011fd7fd24479f00500e8e7edc31401c43"
    "fdb02304007fe5030ff1ac40020020000200100100077fec06e00141feb02afcb2c400200200043f"
    "f5f6c022097f7c0e3f4412ff4fe1ff8825fe8ffcff0048081fe3fda097f4c04bf3812fe5ff27f002"
    "5fc6ff5ff40480edfa601c08ffe8023fcc0f00b00b80a825fdf00fff704bf71ffffdc097fb400c00"
    "812fe781a7f8025fe602203204801001a01607ffd006404012fec00e000825fc7fe500c04bff4056"
    "05c08804004403012fe27feffbf0bb23dc94458ef0420afe1fa61544abda77c130444320a1104303"
    "d3f76f65fbbee7ccf5fe6bddf8bfcff479b7a5f1dc3bf3fce1243b44e90d1784ac350b2f29f2bd60"
    "7b1a1e7bb207519201003807069f8feb7cf00c0d42d85b061f33d2fa7fa00fc3506a02015c4b0940"
    "9bf07cbf950400641582a04fc8f40e88d2dd9f73efbdc40080400407c198588ad0e9f43d67aef900"
    "9c220420cdefbc9f90f920f0338660401a45a0b411a0841c8380c206c1882d0121243e87d02bf27d"
    "1fa2fc6184518a50dcb0008004002001000800400200100080040020010008004002001000800400"
    "20010008004002001000800400200100080040020010008004002001000800400200100080040020"
    "01000800400200100080040020010008004002001000800400200100080040020010008004002001"
    "00080040020010008004002001000800400200100080040020010008004002001000800400200100"
    "08004002001000800400200100080040020010008004002001000800400200100080040020010008"
    "00400200100080040020010008004002001000800400200100080040020010008004002001000800"
    "2aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
    ;
// clang-format on

// Annex C sample C/NAV page (PID 55) including the reserved field, CRC and tail.
const char* const kAnnexCCnavPageHex =
    "fffc17b8de11ef1d27adf5c5d0911e23ed151a4630009cabaf05524b31bad569"
    "62038986eb8c5c688f742f958bf235bf623988a70a79f632677d0c4690000000";

// Two MT1 messages recorded by a u-blox X20 in Boulder, CO on 2025-07-08
// (GPS week 2374), from the rtklibexplorer/GNSS_IMU drive_0708 data set:
// Copyright (c) 2025, rtklibexplorer, BSD 3-Clause License. A mask / orbit /
// code-bias message (flags 110010, MID 17) and the next clock full-set
// message (flags 001000, MID 19) of the same Mask ID / IOD Set ID.
const char* const kX20MaskOrbitMessageHex =
    "802c800520be1ef5fe008140fffffefffdb7fffffc13fe595b76024820014f1fd0036fc31b011813"
    "fea197f740c3fc95800010608c91fb5f7a02b5a012819fda11808fc87fec50011fb802d0dfe9fdbf"
    "8239fdc01afbb07fdd819fe56e407bfa3f507ff9f065fd14ffee02c01009ff67f97e185809df1205"
    "69a029fb70051402bfb5810c0002441c1f81ff8dfbe0548403c0cffef24006813fe0a77ff7efc018"
    "17fd77e97ed82ff69ff2ff405fffa069fd80bfed80740780bff4014805829f8b00f00105feec0080"
    "2a0aff3006000c17ffe80380982ff8f037ff701ff2a025fb4058013f5ffd41701c80400882bfc6ff"
    "200b05ff6c03fff60bffd40700e017fe70037fe02ff60019fee053f4a04a0240bfe57fbbfdc17fe2"
    "82481f82c00401402f05bed6039fcf40b83384beefcc38ffe3fedf401b8720b5d7f7fef809c28c40"
    "05915a24c2808403011c140290c610c2489b0f004c2101f1a055c2d8960ca29bbbf3ce3fde394f21"
    "e95beb6a78bec1cfc1d8780c60d03a055f01d179686517e2980b828061fd3eafb3f6bf68180560ae"
    "0afe6fa5f4dfe7e87aaf45fa7da778ef7ef81d0670d4157f57d9fb5f6400802ff9f07d8771eeff07"
    "d1f59ebbe48168510801741a85e0bc0f7ef7c5f8fe7bf1fcdf87f6c1e06c0e20c8218780f4083f1f"
    "cdf9fe8814048083fe3f3fd4faffe3ddf84f0ffb41a05c0bc063e1791f19e7d55555555555555555"
    "55555555555555555555";
const char* const kX20ClockMessageHex =
    "8132000550044892419c14a075819c31e0cd069057c1b80b5e5275b3de00d6f4400ebf1400e02f7e"
    "ec36c051faf018bfb3fccfc20077fca038ffdffebf1808a010fde81b604dfd17f53fa5fe50377ce9"
    "5555555555555555555555555555555555555555555555555555";

std::vector<uint8_t> hexToBytes(const std::string& hex) {
    std::vector<uint8_t> bytes;
    for (size_t i = 0; i + 1 < hex.size(); i += 2) {
        bytes.push_back(static_cast<uint8_t>(std::stoi(hex.substr(i, 2), nullptr, 16)));
    }
    return bytes;
}

io::GalileoCnavPage pageFromHex(const std::string& hex) {
    io::GalileoCnavPage page;
    const auto bytes = hexToBytes(hex);
    for (size_t i = 0; i < page.bits.size() && i < bytes.size(); ++i) {
        page.bits[i] = bytes[i];
    }
    return page;
}

class BitWriter {
public:
    void put(uint64_t value, int bits) {
        for (int i = bits - 1; i >= 0; --i) {
            const size_t byte = count_ / 8U;
            if (byte >= bytes_.size()) {
                bytes_.push_back(0);
            }
            if (((value >> i) & 1U) != 0) {
                bytes_[byte] |= static_cast<uint8_t>(0x80U >> (count_ % 8U));
            }
            ++count_;
        }
    }
    void putSigned(int64_t value, int bits) {
        put(static_cast<uint64_t>(value) & ((uint64_t{1} << bits) - 1U), bits);
    }
    size_t size() const { return count_; }
    std::vector<uint8_t> bytes(size_t pad_to_bytes = 0) const {
        auto out = bytes_;
        if (out.size() < pad_to_bytes) {
            out.resize(pad_to_bytes, 0);
        }
        return out;
    }

private:
    std::vector<uint8_t> bytes_;
    size_t count_ = 0;
};

// Build a C/NAV page (reserved | HAS header | encoded page | CRC | tail).
io::GalileoCnavPage buildCnavPage(int hass, int mid, int ms, int pid,
                                  const std::array<uint8_t, 53>& encoded,
                                  const GNSSTime& time) {
    BitWriter writer;
    writer.put(0x3FFF, 14);
    writer.put(static_cast<uint64_t>(hass), 2);
    writer.put(0, 2);
    writer.put(1, 2);
    writer.put(static_cast<uint64_t>(mid), 5);
    writer.put(static_cast<uint64_t>(ms - 1), 5);
    writer.put(static_cast<uint64_t>(pid), 8);
    for (const uint8_t octet : encoded) {
        writer.put(octet, 8);
    }
    // CRC-24Q over the 462 bits written so far.
    const auto data = writer.bytes(62);
    uint32_t crc = 0;
    for (size_t i = 0; i < 462; ++i) {
        const uint32_t bit = (data[i / 8] >> (7 - i % 8)) & 1U;
        const uint32_t top = ((crc >> 23) & 1U) ^ bit;
        crc = (crc << 1) & 0xFFFFFFU;
        if (top != 0) {
            crc ^= 0x864CFBU;
        }
    }
    writer.put(crc, 24);
    writer.put(0, 6);
    io::GalileoCnavPage page;
    const auto bytes = writer.bytes(62);
    for (size_t i = 0; i < page.bits.size(); ++i) {
        page.bits[i] = bytes[i];
    }
    page.time = time;
    page.prn = 1;
    return page;
}

// Minimal MT1 mask for one Galileo satellite (E05, signals E1-C and E5a-Q)
// and one GPS satellite (G07, L1 C/A and L2 P).
void writeTestMask(BitWriter& writer) {
    writer.put(2, 4);            // Nsys
    writer.put(2, 4);            // Galileo
    writer.put(uint64_t{1} << (39 - 4), 40);  // E05
    writer.put((1U << (15 - 1)) | (1U << (15 - 4)), 16);  // E1-C, E5a-Q
    writer.put(0, 1);            // CMAF
    writer.put(0, 3);            // NM = I/NAV
    writer.put(0, 4);            // GPS
    writer.put(uint64_t{1} << (39 - 6), 40);  // G07
    writer.put((1U << 15) | (1U << (15 - 9)), 16);  // L1 C/A, L2 P
    writer.put(0, 1);
    writer.put(0, 3);
    writer.put(0, 6);            // reserved
}

std::vector<uint8_t> buildMaskOrbitBiasMessage(int toh, int mask_id, int iod_set_id) {
    BitWriter writer;
    writer.put(static_cast<uint64_t>(toh), 12);
    writer.put(0b110010, 6);
    writer.put(0, 4);
    writer.put(static_cast<uint64_t>(mask_id), 5);
    writer.put(static_cast<uint64_t>(iod_set_id), 5);
    writeTestMask(writer);
    writer.put(10, 4);           // orbit VI = 300 s
    writer.put(77, 10);          // E05 IODnav
    writer.putSigned(40, 13);    // radial 0.1 m
    writer.putSigned(-25, 12);   // in-track -0.2 m
    writer.putSigned(5, 12);     // cross-track 0.04 m
    writer.put(33, 8);           // G07 IODE
    writer.putSigned(-8, 13);    // radial -0.02 m
    writer.putSigned(100, 12);   // in-track 0.8 m
    writer.putSigned(-2048, 12); // cross-track: data not available
    writer.put(10, 4);           // code bias VI
    writer.putSigned(50, 11);    // E05 E1-C 1.00 m
    writer.putSigned(-10, 11);   // E05 E5a-Q -0.20 m
    writer.putSigned(7, 11);     // G07 L1 C/A 0.14 m
    writer.putSigned(-1024, 11); // G07 L2 P: not available
    return writer.bytes();
}

std::vector<uint8_t> buildClockMessage(int toh, int mask_id, int iod_set_id,
                                       int galileo_dcc, int gps_dcc) {
    BitWriter writer;
    writer.put(static_cast<uint64_t>(toh), 12);
    writer.put(0b001000, 6);
    writer.put(0, 4);
    writer.put(static_cast<uint64_t>(mask_id), 5);
    writer.put(static_cast<uint64_t>(iod_set_id), 5);
    writer.put(1, 4);            // clock VI = 10 s
    writer.put(1, 2);            // Galileo DCM = 2
    writer.put(0, 2);            // GPS DCM = 1
    writer.putSigned(galileo_dcc, 13);
    writer.putSigned(gps_dcc, 13);
    return writer.bytes();
}

}  // namespace

TEST(GalileoHasTest, GeneratorMatrixMatchesIcdAnnexB) {
    // Rows 0..31 are the identity.
    for (int r = 0; r < 32; ++r) {
        for (int c = 0; c < 32; ++c) {
            EXPECT_EQ(io::hasRsGeneratorMatrixEntry(r, c), r == c ? 1 : 0);
        }
    }
    // FNV-1a over the 255 x 32 matrix of the ICD Annex B attachment.
    uint32_t hash = 0x811c9dc5U;
    for (int r = 0; r < 255; ++r) {
        for (int c = 0; c < 32; ++c) {
            hash ^= io::hasRsGeneratorMatrixEntry(r, c);
            hash *= 0x01000193U;
        }
    }
    EXPECT_EQ(hash, 0x6b323368U);
    // Annex C decoding matrix D: rows of PID 55 and PID 253, first 15 columns.
    const std::array<int, 15> pid55 = {31, 50, 155, 253, 213, 220, 84, 174, 239, 85, 87, 105, 214, 81, 160};
    const std::array<int, 15> pid253 = {84, 157, 205, 255, 217, 251, 101, 194, 230, 208, 26, 232, 23, 201, 46};
    for (int c = 0; c < 15; ++c) {
        EXPECT_EQ(io::hasRsGeneratorMatrixEntry(54, c), pid55[static_cast<size_t>(c)]);
        EXPECT_EQ(io::hasRsGeneratorMatrixEntry(252, c), pid253[static_cast<size_t>(c)]);
    }
}

TEST(GalileoHasTest, CnavPageCrcAndHeaderFollowAnnexC) {
    auto page = pageFromHex(kAnnexCCnavPageHex);
    EXPECT_TRUE(io::checkGalileoCnavCrc(page));
    const auto header = io::decodeHasPageHeader(page);
    EXPECT_FALSE(header.dummy);
    EXPECT_EQ(header.hass, 0);
    EXPECT_EQ(header.mt, 1);
    EXPECT_EQ(header.mid, 15);
    EXPECT_EQ(header.ms, 15);
    EXPECT_EQ(header.pid, 55);
    EXPECT_EQ(io::extractHasEncodedPage(page), kAnnexCPages.front().second);
    page.bits[20] ^= 0x10U;
    EXPECT_FALSE(io::checkGalileoCnavCrc(page));
}

TEST(GalileoHasTest, HpvrsErasureDecodingReproducesAnnexCMessage) {
    std::vector<int> pids;
    std::vector<std::array<uint8_t, 53>> pages;
    for (const auto& [pid, octets] : kAnnexCPages) {
        pids.push_back(pid);
        pages.push_back(octets);
    }
    std::vector<uint8_t> message;
    ASSERT_TRUE(io::decodeHasHpvrs(pids, pages, 15, message));
    EXPECT_EQ(message, hexToBytes(kAnnexCMessageHex));

    // Any other 15 pages recover the same message (erasure channel).
    for (const auto& [pid, octets] : kAnnexCPages) {
        EXPECT_EQ(io::encodeHasPage(message, 15, pid), octets);
    }
    std::vector<int> other_pids;
    std::vector<std::array<uint8_t, 53>> other_pages;
    // Pages k+1..32 are all-zero (never transmitted); take parity pages.
    for (int pid = 40; pid <= 255 && other_pids.size() < 15; pid += 13) {
        other_pids.push_back(pid);
        other_pages.push_back(io::encodeHasPage(message, 15, pid));
    }
    std::vector<uint8_t> recovered;
    ASSERT_TRUE(io::decodeHasHpvrs(other_pids, other_pages, 15, recovered));
    EXPECT_EQ(recovered, message);
    // Fewer than k pages cannot be decoded.
    other_pids.pop_back();
    other_pages.pop_back();
    EXPECT_FALSE(io::decodeHasHpvrs(other_pids, other_pages, 15, recovered));
}

TEST(GalileoHasTest, Mt1DecodesAnnexCMessageLikeCssrlib) {
    // Expected values: cssrlib cssr_has.decode_cssr() on the same message
    // (orbit components with the ICD sign, i.e. cssrlib's stored value negated).
    io::HasMt1Message mt1;
    ASSERT_EQ(io::decodeHasMt1(hexToBytes(kAnnexCMessageHex), {}, mt1), io::HasMt1Status::Ok);
    EXPECT_EQ(mt1.toh, 0);
    EXPECT_EQ(mt1.flags, 0b110011);
    EXPECT_EQ(mt1.mask_id, 0);
    EXPECT_EQ(mt1.iod_set_id, 11);
    ASSERT_EQ(mt1.mask.systems.size(), 2U);
    EXPECT_EQ(mt1.mask.systems[0].system, GNSSSystem::GPS);
    EXPECT_EQ(mt1.mask.systems[1].system, GNSSSystem::Galileo);
    EXPECT_EQ(mt1.mask.satelliteCount(), 53U);
    ASSERT_EQ(mt1.orbits.size(), 53U);
    const auto& g01 = mt1.orbits.front();
    EXPECT_EQ(g01.satellite, SatelliteId(GNSSSystem::GPS, 1));
    EXPECT_EQ(g01.iodref, 96);
    EXPECT_NEAR(g01.radial_m, 0.0500, 1e-9);
    EXPECT_NEAR(g01.in_track_m, 0.4160, 1e-9);
    EXPECT_NEAR(g01.cross_track_m, 0.2960, 1e-9);
    EXPECT_TRUE(g01.available);
    EXPECT_FALSE(mt1.orbits[1].available);  // G02: data not available
    const auto& e36 = mt1.orbits.back();
    EXPECT_EQ(e36.satellite, SatelliteId(GNSSSystem::Galileo, 36));
    EXPECT_EQ(e36.iodref, 18);
    EXPECT_NEAR(e36.radial_m, -0.1500, 1e-9);
    EXPECT_NEAR(e36.in_track_m, -0.0240, 1e-9);
    EXPECT_NEAR(e36.cross_track_m, -0.0720, 1e-9);
    // Cell mask: G02 carries only an L1 C/A bias.
    ASSERT_GE(mt1.code_biases.size(), 4U);
    EXPECT_EQ(mt1.code_biases[0].satellite, SatelliteId(GNSSSystem::GPS, 1));
    EXPECT_EQ(mt1.code_biases[0].signal_index, 0);
    EXPECT_NEAR(mt1.code_biases[0].value, 3.74, 1e-9);
    EXPECT_EQ(mt1.code_biases[1].signal_index, 7);  // L2 CL
    EXPECT_NEAR(mt1.code_biases[1].value, 5.72, 1e-9);
    EXPECT_EQ(mt1.code_biases[2].satellite, SatelliteId(GNSSSystem::GPS, 2));
    EXPECT_NEAR(mt1.code_biases[2].value, -4.38, 1e-9);
    EXPECT_EQ(mt1.code_biases[3].satellite, SatelliteId(GNSSSystem::GPS, 3));
    const auto& e36_c6 = mt1.code_biases.back();
    EXPECT_EQ(e36_c6.satellite, SatelliteId(GNSSSystem::Galileo, 36));
    EXPECT_EQ(e36_c6.signal_index, 13);  // E6-C
    EXPECT_NEAR(e36_c6.value, 2.20, 1e-9);
    EXPECT_EQ(mt1.phase_biases.size(), mt1.code_biases.size());
    // The rest of the 15-page message is the ICD "0101..." padding.
    const auto bytes = hexToBytes(kAnnexCMessageHex);
    for (size_t bit = mt1.bits_used; bit < bytes.size() * 8U; ++bit) {
        const int value = (bytes[bit / 8U] >> (7U - bit % 8U)) & 1;
        ASSERT_EQ(value, static_cast<int>((bit - mt1.bits_used) % 2U)) << "bit " << bit;
    }
}

TEST(GalileoHasTest, Mt1DecodesRecordedX20MessagesLikeCssrlib) {
    std::map<int, io::HasMask> masks;
    io::HasMt1Message orbit_message;
    ASSERT_EQ(io::decodeHasMt1(hexToBytes(kX20MaskOrbitMessageHex), masks, orbit_message),
              io::HasMt1Status::Ok);
    EXPECT_EQ(orbit_message.toh, 2050);
    EXPECT_EQ(orbit_message.flags, 0b110010);
    EXPECT_EQ(orbit_message.iod_set_id, 5);
    ASSERT_EQ(orbit_message.orbits.size(), 46U);
    EXPECT_EQ(orbit_message.orbits.front().satellite, SatelliteId(GNSSSystem::GPS, 1));
    EXPECT_EQ(orbit_message.orbits.front().iodref, 120);
    EXPECT_NEAR(orbit_message.orbits.front().radial_m, -0.1200, 1e-9);
    EXPECT_NEAR(orbit_message.orbits.front().in_track_m, 0.4320, 1e-9);
    EXPECT_NEAR(orbit_message.orbits.front().cross_track_m, -0.4880, 1e-9);
    const auto e19 = std::find_if(orbit_message.orbits.begin(), orbit_message.orbits.end(),
                                  [](const io::HasOrbitEntry& entry) {
                                      return entry.satellite == SatelliteId(GNSSSystem::Galileo, 19);
                                  });
    ASSERT_NE(e19, orbit_message.orbits.end());
    EXPECT_EQ(e19->iodref, 23);
    EXPECT_NEAR(e19->radial_m, 0.1425, 1e-9);
    EXPECT_NEAR(e19->in_track_m, 0.0640, 1e-9);
    EXPECT_NEAR(e19->cross_track_m, 0.1360, 1e-9);
    // GPS signals L1 C/A, L2 CL, L2 P; Galileo E1-C, E5a-Q, E5b-Q, E6-C.
    EXPECT_EQ(orbit_message.mask.systems[0].signals, (std::vector<int>{0, 7, 9}));
    EXPECT_EQ(orbit_message.mask.systems[1].signals, (std::vector<int>{1, 4, 7, 13}));
    EXPECT_NEAR(orbit_message.code_biases[0].value, 0.92, 1e-9);
    EXPECT_NEAR(orbit_message.code_biases[1].value, 2.06, 1e-9);
    EXPECT_NEAR(orbit_message.code_biases[2].value, 1.50, 1e-9);

    // The clock message carries no mask: it needs the cached one.
    io::HasMt1Message clock_message;
    EXPECT_EQ(io::decodeHasMt1(hexToBytes(kX20ClockMessageHex), masks, clock_message),
              io::HasMt1Status::MissingMask);
    masks[orbit_message.mask_id] = orbit_message.mask;
    ASSERT_EQ(io::decodeHasMt1(hexToBytes(kX20ClockMessageHex), masks, clock_message),
              io::HasMt1Status::Ok);
    EXPECT_EQ(clock_message.toh, 2067);
    EXPECT_EQ(clock_message.flags, 0b001000);
    ASSERT_EQ(clock_message.clocks.size(), 46U);
    const auto clockOf = [&](GNSSSystem system, int prn) {
        for (const auto& entry : clock_message.clocks) {
            if (entry.satellite == SatelliteId(system, static_cast<uint8_t>(prn))) {
                return entry.delta_clock_m;
            }
        }
        return std::nan("");
    };
    EXPECT_NEAR(clockOf(GNSSSystem::GPS, 1), 0.3425, 1e-9);
    EXPECT_NEAR(clockOf(GNSSSystem::GPS, 31), 1.0950, 1e-9);
    EXPECT_NEAR(clockOf(GNSSSystem::Galileo, 2), 0.2025, 1e-9);
    EXPECT_NEAR(clockOf(GNSSSystem::Galileo, 19), 0.3450, 1e-9);
}

TEST(GalileoHasTest, ValidityIntervalsAndMessageReferenceTime) {
    EXPECT_DOUBLE_EQ(io::hasValidityIntervalSeconds(0), 5.0);
    EXPECT_DOUBLE_EQ(io::hasValidityIntervalSeconds(4), 30.0);
    EXPECT_DOUBLE_EQ(io::hasValidityIntervalSeconds(14), 3600.0);
    EXPECT_LT(io::hasValidityIntervalSeconds(15), 0.0);

    const GNSSTime reception(2374, 5.0 * 3600.0 + 20.0);
    EXPECT_EQ(io::hasMessageReferenceTime(reception, 10), GNSSTime(2374, 5.0 * 3600.0 + 10.0));
    // TOH after the reception time within the hour refers to the previous hour.
    EXPECT_EQ(io::hasMessageReferenceTime(reception, 3595), GNSSTime(2374, 4.0 * 3600.0 + 3595.0));
    // Week rollover.
    const GNSSTime week_start(2375, 12.0);
    EXPECT_EQ(io::hasMessageReferenceTime(week_start, 3590), GNSSTime(2374, 604800.0 - 10.0));
}

TEST(GalileoHasTest, SignalMappingToRtcmSsrIds) {
    EXPECT_EQ(io::hasSignalToRtcmSsrSignalId(GNSSSystem::Galileo, 1), 2);    // E1-C
    EXPECT_EQ(io::hasSignalToRtcmSsrSignalId(GNSSSystem::Galileo, 4), 6);    // E5a-Q
    EXPECT_EQ(io::hasSignalToRtcmSsrSignalId(GNSSSystem::Galileo, 7), 9);    // E5b-Q
    EXPECT_EQ(io::hasSignalToRtcmSsrSignalId(GNSSSystem::Galileo, 13), 16);  // E6-C
    EXPECT_EQ(io::hasSignalToRtcmSsrSignalId(GNSSSystem::GPS, 0), 0);        // L1 C/A
    EXPECT_EQ(io::hasSignalToRtcmSsrSignalId(GNSSSystem::GPS, 7), 8);        // L2 CL
    EXPECT_EQ(io::hasSignalToRtcmSsrSignalId(GNSSSystem::GPS, 9), 10);       // L2 P
    EXPECT_EQ(io::hasSignalToRtcmSsrSignalId(GNSSSystem::GPS, 12), 15);      // L5 Q
    EXPECT_EQ(io::hasSignalToRtcmSsrSignalId(GNSSSystem::GPS, 1), 255);      // reserved
    EXPECT_STREQ(io::hasSignalName(GNSSSystem::Galileo, 13), "E6-C");
    EXPECT_NEAR(io::hasSignalWavelength(GNSSSystem::Galileo, 4), 0.2548, 1e-4);
}

TEST(GalileoHasTest, DecoderPairsClocksWithOrbitsAndHoldsUpdates) {
    io::GalileoHasDecoder decoder;
    const int hour = 7;
    const GNSSTime rx0(2380, hour * 3600.0 + 130.0);
    ASSERT_EQ(decoder.addMessage(buildMaskOrbitBiasMessage(120, 3, 9), rx0),
              io::HasMt1Status::Ok);
    EXPECT_TRUE(decoder.updates().empty());  // no clock yet
    const GNSSTime rx1(2380, hour * 3600.0 + 132.0);
    ASSERT_EQ(decoder.addMessage(buildClockMessage(125, 3, 9, 100, -40), rx1),
              io::HasMt1Status::Ok);
    // G07 has no usable orbit (cross-track not available), E05 is updated.
    ASSERT_EQ(decoder.updates().size(), 1U);
    const auto& update = decoder.updates().front();
    EXPECT_EQ(update.satellite, SatelliteId(GNSSSystem::Galileo, 5));
    EXPECT_EQ(update.iode, 77);
    EXPECT_EQ(update.time, GNSSTime(2380, hour * 3600.0 + 125.0));
    EXPECT_EQ(update.valid_until, GNSSTime(2380, hour * 3600.0 + 135.0));  // clock VI 10 s
    EXPECT_NEAR(update.radial_m, 0.10, 1e-9);
    EXPECT_NEAR(update.in_track_m, -0.20, 1e-9);
    EXPECT_NEAR(update.cross_track_m, 0.04, 1e-9);
    EXPECT_NEAR(update.clock_m, 100 * 0.0025 * 2.0, 1e-9);  // DCM 2
    ASSERT_EQ(update.code_bias_m.size(), 2U);
    EXPECT_NEAR(update.code_bias_m.at(1), 1.00, 1e-9);
    EXPECT_NEAR(update.code_bias_m.at(4), -0.20, 1e-9);

    // RTCM SSR storage: orbit negated, clock kept, biases keyed by RTCM IDs.
    const auto rtcm = io::hasUpdateToRtcmSsrCorrection(update);
    EXPECT_EQ(rtcm.iode, 77);
    EXPECT_NEAR(rtcm.orbit_delta_rac_m.x(), -0.10, 1e-9);
    EXPECT_NEAR(rtcm.orbit_delta_rac_m.y(), 0.20, 1e-9);
    EXPECT_NEAR(rtcm.orbit_delta_rac_m.z(), -0.04, 1e-9);
    EXPECT_NEAR(rtcm.clock_delta_poly.x(), 0.5, 1e-9);
    EXPECT_NEAR(rtcm.code_bias_m.at(2), 1.00, 1e-9);
    EXPECT_NEAR(rtcm.code_bias_m.at(6), -0.20, 1e-9);

    // "Do not use" clock value cuts the held update at its reference time.
    const GNSSTime rx2(2380, hour * 3600.0 + 134.0);
    ASSERT_EQ(decoder.addMessage(buildClockMessage(130, 3, 9, 4095, 0), rx2),
              io::HasMt1Status::Ok);
    ASSERT_EQ(decoder.updates().size(), 1U);
    EXPECT_EQ(decoder.updates().front().valid_until, GNSSTime(2380, hour * 3600.0 + 130.0));
}

TEST(GalileoHasTest, DecoderCollectsPagesAndFlushesOnDontUse) {
    io::GalileoHasDecoder decoder;
    const GNSSTime t0(2380, 7 * 3600.0 + 140.0);
    // Encode a 2-page message: mask / orbit / bias MT1 padded to 106 octets.
    auto message = buildMaskOrbitBiasMessage(120, 3, 9);
    message.resize(106, 0x55);
    const auto clock_message = [&]() {
        auto bytes = buildClockMessage(130, 3, 9, 20, 0);
        bytes.resize(53, 0x55);
        return bytes;
    }();
    // Pages 1 and 200 of message MID 4, then the single clock page (MID 5).
    // Page 7 (> MS, <= 32) is an all-zero page that is never transmitted.
    decoder.addPage(buildCnavPage(1, 4, 2, 7, io::encodeHasPage(message, 2, 7), t0));
    EXPECT_EQ(decoder.stats().unsupported_type_pages, 1U);
    decoder.addPage(buildCnavPage(1, 4, 2, 1, io::encodeHasPage(message, 2, 1), t0));
    decoder.addPage(buildCnavPage(1, 4, 2, 1, io::encodeHasPage(message, 2, 1), t0));  // duplicate
    EXPECT_EQ(decoder.stats().messages_decoded, 0U);
    decoder.addPage(buildCnavPage(1, 4, 2, 200, io::encodeHasPage(message, 2, 200), t0 + 1.0));
    EXPECT_EQ(decoder.stats().messages_decoded, 1U);
    decoder.addPage(buildCnavPage(1, 4, 2, 33, io::encodeHasPage(message, 2, 33), t0 + 2.0));
    EXPECT_EQ(decoder.stats().redundant_pages, 1U);
    decoder.addPage(buildCnavPage(1, 5, 1, 1, io::encodeHasPage(clock_message, 1, 1), t0 + 3.0));
    EXPECT_EQ(decoder.stats().messages_decoded, 2U);
    ASSERT_EQ(decoder.updates().size(), 1U);
    EXPECT_EQ(decoder.updates().front().valid_until, GNSSTime(2380, 7 * 3600.0 + 140.0));

    // Corrupted CRC pages are rejected; HASS = 3 flushes the held state.
    auto corrupted = buildCnavPage(1, 6, 1, 1, io::encodeHasPage(clock_message, 1, 1), t0 + 4.0);
    corrupted.bits[30] ^= 0x01U;
    decoder.addPage(corrupted);
    EXPECT_EQ(decoder.stats().crc_failures, 1U);
    decoder.addPage(buildCnavPage(3, 6, 1, 1, io::encodeHasPage(clock_message, 1, 1), t0 - 5.0));
    EXPECT_EQ(decoder.stats().flushes, 1U);
    EXPECT_EQ(decoder.updates().front().valid_until, GNSSTime(2380, 7 * 3600.0 + 135.0));
    // After the flush the clock-only message has no mask to refer to.
    decoder.addPage(buildCnavPage(1, 7, 1, 1, io::encodeHasPage(clock_message, 1, 1), t0 + 6.0));
    EXPECT_EQ(decoder.stats().mt1_missing_mask, 1U);
}
