#include <libgnss++/io/galileo_has.hpp>

#include <libgnss++/core/constants.hpp>
#include <libgnss++/io/ubx.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <fstream>
#include <iterator>
#include <set>
#include <sstream>

namespace libgnss {
namespace io {

namespace {

// ---------------------------------------------------------------------------
// Bit helpers (MSB first)
// ---------------------------------------------------------------------------

class BitReader {
public:
    BitReader(const uint8_t* data, size_t size_bytes) : data_(data), size_bits_(size_bytes * 8U) {}

    bool ok() const { return ok_; }
    size_t position() const { return pos_; }

    uint64_t u(int bits) {
        if (bits <= 0) {
            return 0;
        }
        if (pos_ + static_cast<size_t>(bits) > size_bits_) {
            ok_ = false;
            pos_ = size_bits_;
            return 0;
        }
        uint64_t value = 0;
        for (int i = 0; i < bits; ++i) {
            const size_t bit = pos_ + static_cast<size_t>(i);
            value = (value << 1) | ((data_[bit / 8U] >> (7U - bit % 8U)) & 0x01U);
        }
        pos_ += static_cast<size_t>(bits);
        return value;
    }

    int64_t s(int bits) {
        const uint64_t raw = u(bits);
        if (bits <= 0 || bits >= 64) {
            return static_cast<int64_t>(raw);
        }
        const uint64_t sign = uint64_t{1} << (bits - 1);
        return (raw & sign) != 0 ? static_cast<int64_t>(raw) - static_cast<int64_t>(sign << 1)
                                 : static_cast<int64_t>(raw);
    }

private:
    const uint8_t* data_;
    size_t size_bits_;
    size_t pos_ = 0;
    bool ok_ = true;
};

uint32_t readBits(const uint8_t* data, size_t bit_pos, int bits) {
    uint32_t value = 0;
    for (int i = 0; i < bits; ++i) {
        const size_t bit = bit_pos + static_cast<size_t>(i);
        value = (value << 1) | ((data[bit / 8U] >> (7U - bit % 8U)) & 0x01U);
    }
    return value;
}

// ---------------------------------------------------------------------------
// GF(256), primitive polynomial x^8 + x^4 + x^3 + x^2 + 1 (0x11D)
// ---------------------------------------------------------------------------

struct GaloisField {
    std::array<uint8_t, 512> exp{};
    std::array<int, 256> log{};

    GaloisField() {
        int x = 1;
        for (int i = 0; i < 255; ++i) {
            exp[static_cast<size_t>(i)] = static_cast<uint8_t>(x);
            log[static_cast<size_t>(x)] = i;
            x <<= 1;
            if ((x & 0x100) != 0) {
                x ^= 0x11D;
            }
        }
        for (int i = 255; i < 512; ++i) {
            exp[static_cast<size_t>(i)] = exp[static_cast<size_t>(i - 255)];
        }
        log[0] = -1;
    }

    uint8_t mul(uint8_t a, uint8_t b) const {
        if (a == 0 || b == 0) {
            return 0;
        }
        return exp[static_cast<size_t>(log[a] + log[b])];
    }

    uint8_t inv(uint8_t a) const { return exp[static_cast<size_t>(255 - log[a])]; }
};

const GaloisField& gf() {
    static const GaloisField field;
    return field;
}

// Systematic generator matrix G (255 x 32), ICD Eq. 11-15: rows 0..31 are the
// identity, row r >= 32 holds the coefficient of x^(254 - r) of
// x^(223 + j) mod g(x) for column 31 - j, with the narrow-sense generator
// polynomial g(x) = prod_{i=1}^{223} (x - alpha^i).
struct GeneratorMatrix {
    std::array<std::array<uint8_t, kHasRsInfoLength>, kHasRsCodeLength> g{};

    GeneratorMatrix() {
        const auto& field = gf();
        constexpr int kParity = kHasRsCodeLength - kHasRsInfoLength;  // 223
        std::vector<uint8_t> poly{1};
        for (int i = 1; i <= kParity; ++i) {
            const uint8_t root = field.exp[static_cast<size_t>(i)];
            std::vector<uint8_t> next(poly.size() + 1, 0);
            for (size_t k = 0; k < poly.size(); ++k) {
                next[k + 1] ^= poly[k];
                next[k] ^= field.mul(poly[k], root);
            }
            poly.swap(next);
        }
        std::vector<uint8_t> rem(poly.begin(), poly.begin() + kParity);  // x^223 mod g
        std::vector<std::vector<uint8_t>> remainders;
        remainders.reserve(kHasRsInfoLength);
        for (int j = 0; j < kHasRsInfoLength; ++j) {
            remainders.push_back(rem);
            const uint8_t top = rem[kParity - 1];
            std::vector<uint8_t> next(static_cast<size_t>(kParity), 0);
            for (int k = kParity - 1; k >= 1; --k) {
                next[static_cast<size_t>(k)] = rem[static_cast<size_t>(k - 1)];
            }
            if (top != 0) {
                for (int k = 0; k < kParity; ++k) {
                    next[static_cast<size_t>(k)] ^= field.mul(top, poly[static_cast<size_t>(k)]);
                }
            }
            rem.swap(next);
        }
        for (int c = 0; c < kHasRsInfoLength; ++c) {
            g[static_cast<size_t>(c)][static_cast<size_t>(c)] = 1;
        }
        for (int r = kHasRsInfoLength; r < kHasRsCodeLength; ++r) {
            for (int c = 0; c < kHasRsInfoLength; ++c) {
                const int j = kHasRsInfoLength - 1 - c;
                g[static_cast<size_t>(r)][static_cast<size_t>(c)] =
                    remainders[static_cast<size_t>(j)][static_cast<size_t>(254 - r)];
            }
        }
    }
};

const GeneratorMatrix& generatorMatrix() {
    static const GeneratorMatrix matrix;
    return matrix;
}

// ---------------------------------------------------------------------------
// CRC-24Q, bitwise (the C/NAV CRC covers 462 bits, not a byte multiple)
// ---------------------------------------------------------------------------

uint32_t crc24qBits(const uint8_t* data, size_t bit_count) {
    uint32_t crc = 0;
    for (size_t i = 0; i < bit_count; ++i) {
        const uint32_t bit = (data[i / 8U] >> (7U - i % 8U)) & 0x01U;
        const uint32_t top = ((crc >> 23U) & 0x01U) ^ bit;
        crc = (crc << 1U) & 0xFFFFFFU;
        if (top != 0) {
            crc ^= 0x864CFBU;
        }
    }
    return crc;
}

constexpr int kHasPageBitOffset = 14;      // after the reserved field
constexpr size_t kCnavCrcCoveredBits = 462;
constexpr double kTohFutureToleranceSeconds = 30.0;

double gnssSeconds(const GNSSTime& time) {
    return static_cast<double>(time.week) * constants::SECONDS_PER_WEEK + time.tow;
}

GNSSTime normalizeTime(int week, double tow) {
    while (tow < 0.0) {
        tow += constants::SECONDS_PER_WEEK;
        --week;
    }
    while (tow >= constants::SECONDS_PER_WEEK) {
        tow -= constants::SECONDS_PER_WEEK;
        ++week;
    }
    return GNSSTime(week, tow);
}

template <typename T>
T readLittleEndian(const uint8_t* data) {
    T value{};
    std::memcpy(&value, data, sizeof(T));
    return value;
}

void setPageFromWords(GalileoCnavPage& page, const uint32_t* words, size_t word_count) {
    page.bits.fill(0U);
    for (size_t w = 0; w < word_count; ++w) {
        for (int b = 0; b < 4; ++b) {
            const size_t index = w * 4U + static_cast<size_t>(b);
            if (index >= page.bits.size()) {
                return;
            }
            page.bits[index] = static_cast<uint8_t>((words[w] >> (24 - 8 * b)) & 0xFFU);
        }
    }
}

bool readFileBytes(const std::string& path, std::vector<uint8_t>& bytes) {
    std::ifstream input(path, std::ios::binary);
    if (!input) {
        return false;
    }
    bytes.assign(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
    return true;
}

bool readUbxPages(const std::string& path,
                  std::vector<GalileoCnavPage>& pages,
                  HasPageReadStats& stats,
                  std::string* error) {
    std::vector<uint8_t> bytes;
    if (!readFileBytes(path, bytes)) {
        if (error != nullptr) {
            *error = "cannot open " + path;
        }
        return false;
    }
    UBXDecoder decoder;
    const auto messages = decoder.decode(bytes.data(), bytes.size());
    bool have_time = false;
    GNSSTime last_time;
    for (const auto& message : messages) {
        ++stats.records;
        if (message.message_class != 0x02) {
            continue;
        }
        const auto& payload = message.payload;
        if (message.message_id == 0x15) {  // RXM-RAWX: rcvTow R8, week U2
            if (payload.size() >= 16) {
                const double tow = readLittleEndian<double>(payload.data());
                const uint16_t week = readLittleEndian<uint16_t>(payload.data() + 8);
                if (week > 0 && tow >= 0.0) {
                    last_time = normalizeTime(week, tow);
                    have_time = true;
                }
            }
            continue;
        }
        if (message.message_id != 0x13) {  // RXM-SFRBX
            continue;
        }
        // UBX-RXM-SFRBX: gnssId, svId, sigId, freqId, numWords, chn, version,
        // reserved, dwrd[numWords].
        if (payload.size() < 8) {
            ++stats.skipped;
            continue;
        }
        const uint8_t gnss_id = payload[0];
        const uint8_t sv_id = payload[1];
        const uint8_t sig_id = payload[2];
        const uint8_t num_words = payload[4];
        if (gnss_id != 2 || sig_id != 8) {
            continue;
        }
        if (num_words < 16 || payload.size() < 8U + 4U * num_words || !have_time) {
            ++stats.skipped;
            continue;
        }
        std::array<uint32_t, 16> words{};
        for (size_t w = 0; w < words.size(); ++w) {
            words[w] = readLittleEndian<uint32_t>(payload.data() + 8U + 4U * w);
        }
        GalileoCnavPage page;
        page.time = last_time;
        page.prn = sv_id;
        setPageFromWords(page, words.data(), words.size());
        pages.push_back(page);
        ++stats.pages;
    }
    return true;
}

uint16_t crc16Ccitt(const uint8_t* data, size_t length) {
    uint16_t crc = 0;
    for (size_t i = 0; i < length; ++i) {
        crc = static_cast<uint16_t>(crc ^ (static_cast<uint16_t>(data[i]) << 8));
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x8000U) != 0 ? static_cast<uint16_t>((crc << 1) ^ 0x1021U)
                                       : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

bool readSbfPages(const std::string& path,
                  std::vector<GalileoCnavPage>& pages,
                  HasPageReadStats& stats,
                  std::string* error) {
    std::vector<uint8_t> bytes;
    if (!readFileBytes(path, bytes)) {
        if (error != nullptr) {
            *error = "cannot open " + path;
        }
        return false;
    }
    constexpr uint16_t kGalRawCnav = 4024;
    size_t i = 0;
    while (i + 8 <= bytes.size()) {
        if (bytes[i] != 0x24 || bytes[i + 1] != 0x40) {
            ++i;
            continue;
        }
        const uint16_t crc = readLittleEndian<uint16_t>(bytes.data() + i + 2);
        const uint16_t id_rev = readLittleEndian<uint16_t>(bytes.data() + i + 4);
        const uint16_t length = readLittleEndian<uint16_t>(bytes.data() + i + 6);
        if (length < 8 || (length % 4) != 0 || i + length > bytes.size() ||
            crc16Ccitt(bytes.data() + i + 4, static_cast<size_t>(length) - 4U) != crc) {
            ++i;
            continue;
        }
        ++stats.records;
        const uint16_t block = static_cast<uint16_t>(id_rev & 0x1FFFU);
        if (block == kGalRawCnav) {
            // TOW u4 (ms), WNc u2, SVID u1, CRCPassed u1, ViterbiCnt u1,
            // Source u1, FreqNr u1, RxChannel u1, NAVBits u4[16].
            if (length < 20U + 64U) {
                ++stats.skipped;
            } else {
                const uint8_t* body = bytes.data() + i + 8;
                const uint32_t tow_ms = readLittleEndian<uint32_t>(body);
                const uint16_t wnc = readLittleEndian<uint16_t>(body + 4);
                const uint8_t svid = body[6];
                const uint8_t crc_passed = body[7];
                if (tow_ms == 0xFFFFFFFFU || wnc == 0xFFFFU || svid < 71 || svid > 106) {
                    ++stats.skipped;
                } else {
                    std::array<uint32_t, 16> words{};
                    for (size_t w = 0; w < words.size(); ++w) {
                        words[w] = readLittleEndian<uint32_t>(body + 12 + 4 * w);
                    }
                    GalileoCnavPage page;
                    page.time = normalizeTime(wnc, static_cast<double>(tow_ms) * 1e-3);
                    page.prn = svid - 70;
                    page.receiver_crc_failed = crc_passed == 0;
                    setPageFromWords(page, words.data(), words.size());
                    pages.push_back(page);
                    ++stats.pages;
                }
            }
        }
        i += length;
    }
    return true;
}

int hexValue(char c) {
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    return -1;
}

bool readCssrlibTextPages(const std::string& path,
                          std::vector<GalileoCnavPage>& pages,
                          HasPageReadStats& stats,
                          std::string* error) {
    std::ifstream input(path);
    if (!input) {
        if (error != nullptr) {
            *error = "cannot open " + path;
        }
        return false;
    }
    std::string line;
    while (std::getline(input, line)) {
        const auto first = line.find_first_not_of(" \t\r");
        if (first == std::string::npos || line[first] == '#') {
            continue;
        }
        ++stats.records;
        std::istringstream fields(line);
        int week = 0;
        double tow = 0.0;
        int prn = 0;
        int type = 0;
        int length = 0;
        std::string hex;
        if (!(fields >> week >> tow >> prn >> type >> length >> hex) ||
            hex.size() < static_cast<size_t>(kGalileoCnavPageBytes) * 2U) {
            ++stats.skipped;
            continue;
        }
        GalileoCnavPage page;
        page.time = normalizeTime(week, tow);
        page.prn = prn;
        bool valid = true;
        for (size_t b = 0; b < page.bits.size(); ++b) {
            const int hi = hexValue(hex[2 * b]);
            const int lo = hexValue(hex[2 * b + 1]);
            if (hi < 0 || lo < 0) {
                valid = false;
                break;
            }
            page.bits[b] = static_cast<uint8_t>((hi << 4) | lo);
        }
        if (!valid) {
            ++stats.skipped;
            continue;
        }
        pages.push_back(page);
        ++stats.pages;
    }
    return true;
}

int orbitIodBits(GNSSSystem system) { return system == GNSSSystem::Galileo ? 10 : 8; }

bool isMinimum(int64_t raw, int bits) { return raw == -(int64_t{1} << (bits - 1)); }

}  // namespace

// ---------------------------------------------------------------------------
// Public helpers
// ---------------------------------------------------------------------------

bool parseHasPageInputFormat(const std::string& text, HasPageInputFormat& format) {
    std::string lower;
    for (char c : text) {
        lower.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    }
    if (lower == "ubx") {
        format = HasPageInputFormat::Ubx;
        return true;
    }
    if (lower == "cssrlib" || lower == "txt" || lower == "text") {
        format = HasPageInputFormat::CssrlibText;
        return true;
    }
    if (lower == "sbf") {
        format = HasPageInputFormat::Sbf;
        return true;
    }
    return false;
}

const char* hasPageInputFormatName(HasPageInputFormat format) {
    switch (format) {
        case HasPageInputFormat::Ubx: return "ubx";
        case HasPageInputFormat::CssrlibText: return "cssrlib";
        case HasPageInputFormat::Sbf: return "sbf";
    }
    return "unknown";
}

bool readGalileoCnavPages(const std::string& path,
                          HasPageInputFormat format,
                          std::vector<GalileoCnavPage>& pages,
                          HasPageReadStats* stats,
                          std::string* error) {
    HasPageReadStats local;
    bool ok = false;
    switch (format) {
        case HasPageInputFormat::Ubx: ok = readUbxPages(path, pages, local, error); break;
        case HasPageInputFormat::CssrlibText: ok = readCssrlibTextPages(path, pages, local, error); break;
        case HasPageInputFormat::Sbf: ok = readSbfPages(path, pages, local, error); break;
    }
    if (stats != nullptr) {
        *stats = local;
    }
    return ok;
}

HasPageInputFormat guessHasPageInputFormat(const std::string& path) {
    std::string lower;
    for (char c : path) {
        lower.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    }
    const auto endsWith = [&lower](const std::string& suffix) {
        return lower.size() >= suffix.size() &&
               lower.compare(lower.size() - suffix.size(), suffix.size(), suffix) == 0;
    };
    if (endsWith(".ubx")) {
        return HasPageInputFormat::Ubx;
    }
    if (endsWith(".sbf") || (!lower.empty() && lower.back() == '_')) {  // Septentrio *.yy_
        return HasPageInputFormat::Sbf;
    }
    return HasPageInputFormat::CssrlibText;
}

bool decodeGalileoHasPages(const std::string& path,
                           HasPageInputFormat format,
                           GalileoHasDecoder& decoder,
                           HasPageReadStats* stats,
                           std::string* error) {
    std::vector<GalileoCnavPage> pages;
    if (!readGalileoCnavPages(path, format, pages, stats, error)) {
        return false;
    }
    for (const auto& page : pages) {
        decoder.addPage(page);
    }
    return true;
}

bool checkGalileoCnavCrc(const GalileoCnavPage& page) {
    const uint32_t expected = readBits(page.bits.data(), kCnavCrcCoveredBits, 24);
    return crc24qBits(page.bits.data(), kCnavCrcCoveredBits) == expected;
}

HasPageHeader decodeHasPageHeader(const GalileoCnavPage& page) {
    HasPageHeader header;
    const uint32_t raw = readBits(page.bits.data(), kHasPageBitOffset, 24);
    if (raw == kHasDummyPageHeader) {
        header.dummy = true;
        return header;
    }
    header.hass = static_cast<int>((raw >> 22U) & 0x03U);
    header.reserved = static_cast<int>((raw >> 20U) & 0x03U);
    header.mt = static_cast<int>((raw >> 18U) & 0x03U);
    header.mid = static_cast<int>((raw >> 13U) & 0x1FU);
    header.ms = static_cast<int>((raw >> 8U) & 0x1FU) + 1;
    header.pid = static_cast<int>(raw & 0xFFU);
    return header;
}

std::array<uint8_t, kHasEncodedPageBytes> extractHasEncodedPage(const GalileoCnavPage& page) {
    std::array<uint8_t, kHasEncodedPageBytes> encoded{};
    for (size_t k = 0; k < encoded.size(); ++k) {
        encoded[k] = static_cast<uint8_t>(
            readBits(page.bits.data(), kHasPageBitOffset + 24U + 8U * k, 8));
    }
    return encoded;
}

uint8_t hasRsGeneratorMatrixEntry(int row, int col) {
    if (row < 0 || row >= kHasRsCodeLength || col < 0 || col >= kHasRsInfoLength) {
        return 0;
    }
    return generatorMatrix().g[static_cast<size_t>(row)][static_cast<size_t>(col)];
}

bool decodeHasHpvrs(const std::vector<int>& pids,
                    const std::vector<std::array<uint8_t, kHasEncodedPageBytes>>& encoded_pages,
                    int ms,
                    std::vector<uint8_t>& message) {
    message.clear();
    if (ms < 1 || ms > kHasRsInfoLength || pids.size() != encoded_pages.size() ||
        pids.size() < static_cast<size_t>(ms)) {
        return false;
    }
    const auto& field = gf();
    const auto& g = generatorMatrix().g;
    const size_t k = static_cast<size_t>(ms);
    // Augmented [D | W] with D the k x k sub-matrix of G (rows PID - 1, first
    // k columns) and W the k x 53 received octets; Gauss-Jordan in GF(256).
    std::vector<std::vector<uint8_t>> a(k, std::vector<uint8_t>(k + kHasEncodedPageBytes, 0));
    for (size_t r = 0; r < k; ++r) {
        const int pid = pids[r];
        if (pid < 1 || pid > kHasRsCodeLength) {
            return false;
        }
        for (size_t c = 0; c < k; ++c) {
            a[r][c] = g[static_cast<size_t>(pid - 1)][c];
        }
        std::copy(encoded_pages[r].begin(), encoded_pages[r].end(), a[r].begin() + static_cast<std::ptrdiff_t>(k));
    }
    for (size_t c = 0; c < k; ++c) {
        size_t pivot = c;
        while (pivot < k && a[pivot][c] == 0) {
            ++pivot;
        }
        if (pivot == k) {
            return false;
        }
        std::swap(a[c], a[pivot]);
        const uint8_t inv = field.inv(a[c][c]);
        for (auto& value : a[c]) {
            value = field.mul(value, inv);
        }
        for (size_t r = 0; r < k; ++r) {
            if (r == c || a[r][c] == 0) {
                continue;
            }
            const uint8_t factor = a[r][c];
            for (size_t j = 0; j < a[r].size(); ++j) {
                a[r][j] ^= field.mul(factor, a[c][j]);
            }
        }
    }
    message.reserve(k * kHasEncodedPageBytes);
    for (size_t r = 0; r < k; ++r) {
        message.insert(message.end(), a[r].begin() + static_cast<std::ptrdiff_t>(k), a[r].end());
    }
    return true;
}

std::array<uint8_t, kHasEncodedPageBytes> encodeHasPage(const std::vector<uint8_t>& message,
                                                        int ms,
                                                        int pid) {
    std::array<uint8_t, kHasEncodedPageBytes> page{};
    if (ms < 1 || ms > kHasRsInfoLength || pid < 1 || pid > kHasRsCodeLength ||
        message.size() < static_cast<size_t>(ms) * kHasEncodedPageBytes) {
        return page;
    }
    const auto& field = gf();
    const auto& row = generatorMatrix().g[static_cast<size_t>(pid - 1)];
    for (size_t j = 0; j < page.size(); ++j) {
        uint8_t value = 0;
        for (int c = 0; c < ms; ++c) {
            value ^= field.mul(row[static_cast<size_t>(c)],
                               message[static_cast<size_t>(c) * kHasEncodedPageBytes + j]);
        }
        page[j] = value;
    }
    return page;
}

double hasValidityIntervalSeconds(int index) {
    static constexpr std::array<double, 15> kTable = {
        5.0, 10.0, 15.0, 20.0, 30.0, 60.0, 90.0, 120.0, 180.0, 240.0, 300.0, 600.0, 900.0, 1800.0,
        3600.0};
    if (index < 0 || index >= static_cast<int>(kTable.size())) {
        return -1.0;
    }
    return kTable[static_cast<size_t>(index)];
}

GNSSTime hasMessageReferenceTime(const GNSSTime& reception_time, int toh) {
    const double hour_start = std::floor(reception_time.tow / 3600.0) * 3600.0;
    GNSSTime reference = normalizeTime(reception_time.week, hour_start + static_cast<double>(toh));
    // ICD Eq. 29; receiver page time tags may lag the true reception by a
    // second or two, so a TOH slightly ahead of the tag stays in this hour.
    if (reference - reception_time > kTohFutureToleranceSeconds) {
        reference = reference - 3600.0;
    }
    return reference;
}

GNSSSystem hasGnssIdToSystem(int gnss_id) {
    switch (gnss_id) {
        case 0: return GNSSSystem::GPS;
        case 2: return GNSSSystem::Galileo;
        default: return GNSSSystem::UNKNOWN;
    }
}

const char* hasSignalName(GNSSSystem system, int signal_index) {
    if (system == GNSSSystem::Galileo) {
        static constexpr std::array<const char*, 15> kNames = {
            "E1-B", "E1-C", "E1-B+C", "E5a-I", "E5a-Q", "E5a-I+Q", "E5b-I", "E5b-Q",
            "E5b-I+Q", "E5-I", "E5-Q", "E5-I+Q", "E6-B", "E6-C", "E6-B+C"};
        if (signal_index >= 0 && signal_index < static_cast<int>(kNames.size())) {
            return kNames[static_cast<size_t>(signal_index)];
        }
    } else if (system == GNSSSystem::GPS) {
        switch (signal_index) {
            case 0: return "L1C/A";
            case 3: return "L1C(D)";
            case 4: return "L1C(P)";
            case 5: return "L1C(D+P)";
            case 6: return "L2CM";
            case 7: return "L2CL";
            case 8: return "L2CM+CL";
            case 9: return "L2P";
            case 11: return "L5I";
            case 12: return "L5Q";
            case 13: return "L5I+Q";
            default: break;
        }
    }
    return "";
}

uint8_t hasSignalToRtcmSsrSignalId(GNSSSystem system, int signal_index) {
    if (system == GNSSSystem::Galileo) {
        // RTCM 10403.3 Table 3.5-100: 1 E1B, 2 E1C, 3 E1B+C, 5 E5aI, 6 E5aQ,
        // 7 E5aI+Q, 8 E5bI, 9 E5bQ, 10 E5bI+Q, 11 E5I, 12 E5Q, 13 E5I+Q,
        // 15 E6B, 16 E6C, 17 E6B+C.
        static constexpr std::array<uint8_t, 15> kMap = {1, 2, 3, 5, 6, 7, 8, 9, 10,
                                                         11, 12, 13, 15, 16, 17};
        if (signal_index >= 0 && signal_index < static_cast<int>(kMap.size())) {
            return kMap[static_cast<size_t>(signal_index)];
        }
    } else if (system == GNSSSystem::GPS) {
        // RTCM 10403.3 Table 3.5-91: 0 L1C/A, 7 L2C(M), 8 L2C(L), 9 L2C(M+L),
        // 10 L2P, 14 L5I, 15 L5Q, 16 L5I+Q, 17 L1C(D), 18 L1C(P), 19 L1C(D+P).
        switch (signal_index) {
            case 0: return 0;
            case 3: return 17;
            case 4: return 18;
            case 5: return 19;
            case 6: return 7;
            case 7: return 8;
            case 8: return 9;
            case 9: return 10;
            case 11: return 14;
            case 12: return 15;
            case 13: return 16;
            default: break;
        }
    }
    return 255;
}

double hasSignalWavelength(GNSSSystem system, int signal_index) {
    constexpr double kE5AltBocFreq = 1191.795e6;
    if (system == GNSSSystem::Galileo) {
        if (signal_index >= 0 && signal_index <= 2) return constants::GAL_E1_WAVELENGTH;
        if (signal_index >= 3 && signal_index <= 5) return constants::GAL_E5A_WAVELENGTH;
        if (signal_index >= 6 && signal_index <= 8) return constants::GAL_E5B_WAVELENGTH;
        if (signal_index >= 9 && signal_index <= 11) return constants::SPEED_OF_LIGHT / kE5AltBocFreq;
        if (signal_index >= 12 && signal_index <= 14) return constants::GAL_E6_WAVELENGTH;
    } else if (system == GNSSSystem::GPS) {
        if (signal_index == 0 || (signal_index >= 3 && signal_index <= 5)) return constants::GPS_L1_WAVELENGTH;
        if (signal_index >= 6 && signal_index <= 9) return constants::GPS_L2_WAVELENGTH;
        if (signal_index >= 11 && signal_index <= 13) return constants::GPS_L5_WAVELENGTH;
    }
    return 0.0;
}

size_t HasMask::satelliteCount() const {
    size_t count = 0;
    for (const auto& system : systems) {
        count += system.prns.size();
    }
    return count;
}

const char* hasMt1StatusName(HasMt1Status status) {
    switch (status) {
        case HasMt1Status::Ok: return "ok";
        case HasMt1Status::MissingMask: return "missing_mask";
        case HasMt1Status::Truncated: return "truncated";
        case HasMt1Status::InvalidHeader: return "invalid_header";
        case HasMt1Status::InvalidMask: return "invalid_mask";
    }
    return "unknown";
}

HasMt1Status decodeHasMt1(const std::vector<uint8_t>& message,
                          const std::map<int, HasMask>& known_masks,
                          HasMt1Message& decoded) {
    decoded = HasMt1Message{};
    BitReader reader(message.data(), message.size());
    decoded.toh = static_cast<int>(reader.u(12));
    decoded.flags = static_cast<int>(reader.u(6));
    reader.u(4);
    decoded.mask_id = static_cast<int>(reader.u(5));
    decoded.iod_set_id = static_cast<int>(reader.u(5));
    if (!reader.ok()) {
        return HasMt1Status::Truncated;
    }
    if (decoded.toh >= 3600) {
        return HasMt1Status::InvalidHeader;
    }
    decoded.has_mask = (decoded.flags & 0x20) != 0;
    decoded.has_orbit = (decoded.flags & 0x10) != 0;
    decoded.has_clock_full = (decoded.flags & 0x08) != 0;
    decoded.has_clock_subset = (decoded.flags & 0x04) != 0;
    decoded.has_code_bias = (decoded.flags & 0x02) != 0;
    decoded.has_phase_bias = (decoded.flags & 0x01) != 0;

    // Mask block (ICD Table 15/16).
    if (decoded.has_mask) {
        HasMask mask;
        mask.mask_id = decoded.mask_id;
        const int nsys = static_cast<int>(reader.u(4));
        if (nsys == 0) {
            return reader.ok() ? HasMt1Status::InvalidMask : HasMt1Status::Truncated;
        }
        for (int n = 0; n < nsys; ++n) {
            HasMaskSystem system;
            system.gnss_id = static_cast<int>(reader.u(4));
            system.system = hasGnssIdToSystem(system.gnss_id);
            const uint64_t sat_mask = reader.u(40);
            const uint64_t sig_mask = reader.u(16);
            system.cell_mask_available = reader.u(1) != 0;
            for (int i = 0; i < 40; ++i) {
                if (((sat_mask >> (39 - i)) & 0x01U) != 0) {
                    system.prns.push_back(i + 1);
                }
            }
            for (int j = 0; j < 16; ++j) {
                if (((sig_mask >> (15 - j)) & 0x01U) != 0) {
                    system.signals.push_back(j);
                }
            }
            if (system.cell_mask_available) {
                system.cell_mask.assign(system.prns.size(),
                                        std::vector<bool>(system.signals.size(), false));
                for (auto& row : system.cell_mask) {
                    for (size_t j = 0; j < row.size(); ++j) {
                        row[j] = reader.u(1) != 0;
                    }
                }
            }
            system.nav_message = static_cast<int>(reader.u(3));
            if (system.system == GNSSSystem::UNKNOWN) {
                return reader.ok() ? HasMt1Status::InvalidMask : HasMt1Status::Truncated;
            }
            mask.systems.push_back(std::move(system));
        }
        reader.u(6);
        if (!reader.ok()) {
            return HasMt1Status::Truncated;
        }
        decoded.mask = std::move(mask);
    } else {
        const auto it = known_masks.find(decoded.mask_id);
        if (it == known_masks.end()) {
            return HasMt1Status::MissingMask;
        }
        decoded.mask = it->second;
    }
    const HasMask& mask = decoded.mask;

    // Orbit corrections block (ICD Table 22/24).
    if (decoded.has_orbit) {
        decoded.orbit_vi = static_cast<int>(reader.u(4));
        for (const auto& system : mask.systems) {
            const int iod_bits = orbitIodBits(system.system);
            for (const int prn : system.prns) {
                HasOrbitEntry entry;
                entry.satellite = SatelliteId(system.system, static_cast<uint8_t>(prn));
                entry.iodref = static_cast<int>(reader.u(iod_bits));
                const int64_t dr = reader.s(13);
                const int64_t dit = reader.s(12);
                const int64_t dct = reader.s(12);
                entry.radial_m = static_cast<double>(dr) * 0.0025;
                entry.in_track_m = static_cast<double>(dit) * 0.0080;
                entry.cross_track_m = static_cast<double>(dct) * 0.0080;
                entry.available = !isMinimum(dr, 13) && !isMinimum(dit, 12) && !isMinimum(dct, 12);
                decoded.orbits.push_back(entry);
            }
        }
        if (!reader.ok()) {
            return HasMt1Status::Truncated;
        }
    }

    const auto makeClock = [](const SatelliteId& satellite, int64_t raw, int multiplier, bool subset) {
        HasClockEntry entry;
        entry.satellite = satellite;
        entry.dcc_raw = static_cast<int>(raw);
        entry.multiplier = multiplier;
        entry.subset = subset;
        if (raw == -4096) {
            entry.status = HasClockStatus::NotAvailable;
        } else if (raw == 4095) {
            entry.status = HasClockStatus::DoNotUse;
        } else {
            entry.status = HasClockStatus::Ok;
            entry.delta_clock_m = static_cast<double>(raw) * 0.0025 * static_cast<double>(multiplier);
        }
        return entry;
    };

    // Clock full-set block (ICD Table 27-31).
    if (decoded.has_clock_full) {
        decoded.clock_full_vi = static_cast<int>(reader.u(4));
        std::vector<int> multipliers;
        for (size_t n = 0; n < mask.systems.size(); ++n) {
            multipliers.push_back(static_cast<int>(reader.u(2)) + 1);
        }
        for (size_t n = 0; n < mask.systems.size(); ++n) {
            const auto& system = mask.systems[n];
            for (const int prn : system.prns) {
                const int64_t raw = reader.s(13);
                decoded.clocks.push_back(makeClock(SatelliteId(system.system, static_cast<uint8_t>(prn)),
                                                   raw, multipliers[n], false));
            }
        }
        if (!reader.ok()) {
            return HasMt1Status::Truncated;
        }
    }

    // Clock subset block (ICD Table 32-34).
    if (decoded.has_clock_subset) {
        decoded.clock_subset_vi = static_cast<int>(reader.u(4));
        const int nsys_sub = static_cast<int>(reader.u(4));
        for (int n = 0; n < nsys_sub; ++n) {
            const int gnss_id = static_cast<int>(reader.u(4));
            const int multiplier = static_cast<int>(reader.u(2)) + 1;
            const auto system_it = std::find_if(mask.systems.begin(), mask.systems.end(),
                                                [gnss_id](const HasMaskSystem& system) {
                                                    return system.gnss_id == gnss_id;
                                                });
            if (system_it == mask.systems.end()) {
                return reader.ok() ? HasMt1Status::InvalidMask : HasMt1Status::Truncated;
            }
            std::vector<int> subset;
            for (const int prn : system_it->prns) {
                if (reader.u(1) != 0) {
                    subset.push_back(prn);
                }
            }
            for (const int prn : subset) {
                const int64_t raw = reader.s(13);
                decoded.clocks.push_back(makeClock(
                    SatelliteId(system_it->system, static_cast<uint8_t>(prn)), raw, multiplier, true));
            }
        }
        if (!reader.ok()) {
            return HasMt1Status::Truncated;
        }
    }

    const auto readBiases = [&](bool phase, std::vector<HasBiasEntry>& out) {
        for (const auto& system : mask.systems) {
            for (size_t s = 0; s < system.prns.size(); ++s) {
                for (size_t j = 0; j < system.signals.size(); ++j) {
                    if (system.cell_mask_available && !system.cell_mask[s][j]) {
                        continue;
                    }
                    HasBiasEntry entry;
                    entry.satellite = SatelliteId(system.system, static_cast<uint8_t>(system.prns[s]));
                    entry.signal_index = system.signals[j];
                    const int64_t raw = reader.s(11);
                    entry.available = !isMinimum(raw, 11);
                    entry.value = static_cast<double>(raw) * (phase ? 0.01 : 0.02);
                    if (phase) {
                        entry.discontinuity = static_cast<int>(reader.u(2));
                    }
                    out.push_back(entry);
                }
            }
        }
    };

    // Code bias block (ICD Table 35-37).
    if (decoded.has_code_bias) {
        decoded.code_bias_vi = static_cast<int>(reader.u(4));
        readBiases(false, decoded.code_biases);
        if (!reader.ok()) {
            return HasMt1Status::Truncated;
        }
    }
    // Phase bias block (ICD Table 38-40).
    if (decoded.has_phase_bias) {
        decoded.phase_bias_vi = static_cast<int>(reader.u(4));
        readBiases(true, decoded.phase_biases);
        if (!reader.ok()) {
            return HasMt1Status::Truncated;
        }
    }
    decoded.bits_used = reader.position();
    return HasMt1Status::Ok;
}

RTCMSSRCorrection hasUpdateToRtcmSsrCorrection(const HasSsrUpdate& update) {
    RTCMSSRCorrection correction;
    correction.satellite = update.satellite;
    correction.time = update.time;
    correction.issue_of_data = static_cast<uint8_t>(update.iod_set_id < 0 ? 0 : update.iod_set_id);
    correction.iode = update.iode;
    correction.has_orbit = true;
    correction.has_clock = true;
    // HAS adds the NTW correction to the broadcast position; RTCM SSR stores
    // the correction subtracted from it (same radial / along / cross axes).
    correction.orbit_delta_rac_m =
        Vector3d(-update.radial_m, -update.in_track_m, -update.cross_track_m);
    correction.clock_delta_poly = Vector3d(update.clock_m, 0.0, 0.0);
    for (const auto& [signal_index, bias_m] : update.code_bias_m) {
        const uint8_t wire_id = hasSignalToRtcmSsrSignalId(update.satellite.system, signal_index);
        if (wire_id == 255) {
            continue;
        }
        correction.code_bias_m[wire_id] = bias_m;
    }
    correction.has_code_bias = !correction.code_bias_m.empty();
    return correction;
}

// ---------------------------------------------------------------------------
// Stateful decoder
// ---------------------------------------------------------------------------

GalileoHasDecoder::GalileoHasDecoder(const Options& options) : options_(options) {}

void GalileoHasDecoder::addPage(const GalileoCnavPage& page) {
    ++stats_.pages;
    if (page.receiver_crc_failed || (options_.check_crc && !checkGalileoCnavCrc(page))) {
        ++stats_.crc_failures;
        return;
    }
    const HasPageHeader header = decodeHasPageHeader(page);
    if (header.dummy) {
        ++stats_.dummy_pages;
        return;
    }
    if (header.hass == 3) {
        ++stats_.dont_use_pages;
        flush(page.time);
        return;
    }
    if (header.hass == 2) {
        ++stats_.reserved_status_pages;
        return;
    }
    if (header.hass == 0) {
        if (!options_.accept_test_mode) {
            return;
        }
        ++stats_.test_mode_pages;
    }
    // PIDs MS+1..32 are all-zero encoded pages that are never transmitted
    // (ICD section 6.3); they carry no information for the decoder.
    if (header.mt != 1 || header.pid == 0 ||
        (header.pid > header.ms && header.pid <= kHasRsInfoLength)) {
        ++stats_.unsupported_type_pages;
        return;
    }

    auto& collection = collections_[static_cast<size_t>(header.mid)];
    const auto encoded = extractHasEncodedPage(page);
    const auto reset = [&]() {
        if (collection.active) {
            ++stats_.collection_resets;
        }
        collection = Collection{};
        collection.active = true;
        collection.ms = header.ms;
        collection.first_time = page.time;
        collection.last_time = page.time;
    };
    bool stale = !collection.active || collection.ms != header.ms;
    if (!stale) {
        // An incomplete collection expires relative to its first page; a
        // decoded one after a gap without redundant pages.
        const GNSSTime& anchor = collection.decoded ? collection.last_time : collection.first_time;
        stale = std::abs(page.time - anchor) > options_.collection_timeout_s;
    }
    if (stale) {
        reset();
    }
    if (collection.decoded) {
        if (encodeHasPage(collection.message, collection.ms, header.pid) == encoded) {
            ++stats_.redundant_pages;
            collection.last_time = page.time;
            return;
        }
        reset();  // the message ID was reused for a new message
    }
    const auto existing = collection.pages.find(header.pid);
    if (existing != collection.pages.end()) {
        if (existing->second == encoded) {
            return;
        }
        reset();
    }
    collection.pages[header.pid] = encoded;
    collection.last_time = page.time;
    if (collection.pages.size() < static_cast<size_t>(collection.ms)) {
        return;
    }

    std::vector<int> pids;
    std::vector<std::array<uint8_t, kHasEncodedPageBytes>> pages;
    for (const auto& [pid, bytes] : collection.pages) {
        pids.push_back(pid);
        pages.push_back(bytes);
        if (pids.size() == static_cast<size_t>(collection.ms)) {
            break;
        }
    }
    std::vector<uint8_t> message;
    if (!decodeHasHpvrs(pids, pages, collection.ms, message)) {
        ++stats_.rs_failures;
        reset();
        return;
    }
    collection.decoded = true;
    collection.message = message;
    ++stats_.messages_decoded;
    addMessage(message, page.time, header.hass, header.mid, header.ms);
}

HasMt1Status GalileoHasDecoder::addMessage(const std::vector<uint8_t>& message,
                                           const GNSSTime& reception_time,
                                           int hass,
                                           int mid,
                                           int ms) {
    HasDecodedMessage record;
    record.reception_time = reception_time;
    record.hass = hass;
    record.mid = mid;
    record.ms = ms;
    record.status = decodeHasMt1(message, masks_, record.mt1);
    if (record.status == HasMt1Status::MissingMask) {
        ++stats_.mt1_missing_mask;
    } else if (record.status != HasMt1Status::Ok) {
        ++stats_.mt1_errors;
    }
    if (record.status == HasMt1Status::Ok) {
        const HasMt1Message& mt1 = record.mt1;
        const GNSSTime reference = hasMessageReferenceTime(reception_time, mt1.toh);
        record.reference_time = reference;
        if (mt1.has_mask) {
            masks_[mt1.mask_id] = mt1.mask;
        }
        std::set<SatelliteId> touched;
        if (mt1.has_orbit) {
            OrbitSet set;
            set.time = reference;
            set.validity_s = hasValidityIntervalSeconds(mt1.orbit_vi);
            for (const auto& entry : mt1.orbits) {
                set.entries[entry.satellite] = entry;
                touched.insert(entry.satellite);
            }
            orbit_sets_[{mt1.mask_id, mt1.iod_set_id}] = std::move(set);
        }
        for (const auto& entry : mt1.clocks) {
            if (entry.status != HasClockStatus::Ok) {
                clocks_.erase(entry.satellite);
                cut(entry.satellite, reference);
                continue;
            }
            ClockState state;
            state.time = reference;
            state.validity_s =
                hasValidityIntervalSeconds(entry.subset ? mt1.clock_subset_vi : mt1.clock_full_vi);
            state.entry = entry;
            state.mask_id = mt1.mask_id;
            state.iod_set_id = mt1.iod_set_id;
            clocks_[entry.satellite] = state;
            touched.insert(entry.satellite);
        }
        const auto storeBiases = [&](const std::vector<HasBiasEntry>& entries, int vi,
                                     std::map<SatelliteId, BiasState>& target) {
            std::map<SatelliteId, BiasState> fresh;
            for (const auto& entry : entries) {
                auto& state = fresh[entry.satellite];
                state.time = reference;
                state.validity_s = hasValidityIntervalSeconds(vi);
                if (entry.available) {
                    state.entries[entry.signal_index] = entry;
                }
            }
            for (auto& [satellite, state] : fresh) {
                target[satellite] = std::move(state);
                touched.insert(satellite);
            }
        };
        if (mt1.has_code_bias) {
            storeBiases(mt1.code_biases, mt1.code_bias_vi, code_biases_);
        }
        if (mt1.has_phase_bias) {
            storeBiases(mt1.phase_biases, mt1.phase_bias_vi, phase_biases_);
        }
        for (const auto& satellite : touched) {
            tryEmit(satellite);
        }
    }
    const HasMt1Status status = record.status;
    if (options_.keep_messages) {
        messages_.push_back(std::move(record));
    }
    return status;
}

void GalileoHasDecoder::tryEmit(const SatelliteId& satellite) {
    const auto clock_it = clocks_.find(satellite);
    if (clock_it == clocks_.end()) {
        return;
    }
    const ClockState& clock = clock_it->second;
    const auto set_it = orbit_sets_.find({clock.mask_id, clock.iod_set_id});
    if (set_it == orbit_sets_.end()) {
        ++stats_.unpaired_clocks;
        return;
    }
    const OrbitSet& set = set_it->second;
    const auto orbit_it = set.entries.find(satellite);
    if (orbit_it == set.entries.end()) {
        ++stats_.unpaired_clocks;
        return;
    }
    const GNSSTime start = clock.time > set.time ? clock.time : set.time;
    const HasOrbitEntry& orbit = orbit_it->second;
    if (!orbit.available) {
        cut(satellite, start);
        return;
    }
    if (set.validity_s < 0.0 || clock.validity_s < 0.0) {
        return;
    }
    const GNSSTime orbit_end = set.time + set.validity_s;
    const GNSSTime clock_end = clock.time + clock.validity_s;
    const GNSSTime end = orbit_end < clock_end ? orbit_end : clock_end;
    if (!(start < end)) {
        return;
    }

    HasSsrUpdate update;
    update.satellite = satellite;
    update.time = start;
    update.valid_until = end;
    update.iode = orbit.iodref;
    update.mask_id = clock.mask_id;
    update.iod_set_id = clock.iod_set_id;
    update.orbit_time = set.time;
    update.clock_time = clock.time;
    update.radial_m = orbit.radial_m;
    update.in_track_m = orbit.in_track_m;
    update.cross_track_m = orbit.cross_track_m;
    update.clock_m = clock.entry.delta_clock_m;
    const auto biasUsable = [&](const BiasState& state) {
        return state.validity_s >= 0.0 && std::abs(start - state.time) <= state.validity_s;
    };
    const auto code_it = code_biases_.find(satellite);
    if (code_it != code_biases_.end() && biasUsable(code_it->second)) {
        for (const auto& [signal, entry] : code_it->second.entries) {
            update.code_bias_m[signal] = entry.value;
        }
    }
    const auto phase_it = phase_biases_.find(satellite);
    if (phase_it != phase_biases_.end() && biasUsable(phase_it->second)) {
        for (const auto& [signal, entry] : phase_it->second.entries) {
            update.phase_bias_cycles[signal] = entry.value;
            update.phase_discontinuity[signal] = entry.discontinuity;
        }
    }

    const auto last_it = last_update_index_.find(satellite);
    if (last_it != last_update_index_.end() && updates_[last_it->second].time == start) {
        updates_[last_it->second] = std::move(update);
        return;
    }
    last_update_index_[satellite] = updates_.size();
    updates_.push_back(std::move(update));
    ++stats_.updates;
}

void GalileoHasDecoder::cut(const SatelliteId& satellite, const GNSSTime& time) {
    const auto it = last_update_index_.find(satellite);
    if (it == last_update_index_.end()) {
        return;
    }
    auto& update = updates_[it->second];
    if (time < update.valid_until) {
        update.valid_until = time < update.time ? update.time : time;
    }
}

void GalileoHasDecoder::flush(const GNSSTime& time) {
    const bool had_state = !masks_.empty() || !orbit_sets_.empty() || !clocks_.empty();
    for (auto& collection : collections_) {
        collection = Collection{};
    }
    masks_.clear();
    orbit_sets_.clear();
    clocks_.clear();
    code_biases_.clear();
    phase_biases_.clear();
    for (const auto& [satellite, index] : last_update_index_) {
        (void)index;
        cut(satellite, time);
    }
    if (had_state) {
        ++stats_.flushes;
    }
}

}  // namespace io
}  // namespace libgnss
