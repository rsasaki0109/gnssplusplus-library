#include <libgnss++/io/bds_ppp_b2b.hpp>

#include <cmath>
#include <fstream>
#include <sstream>

namespace libgnss {
namespace io {

namespace {

constexpr double kSecondsPerDay = 86400.0;
constexpr double kBdtMinusGpst = -14.0;  // BDT = GPST - 14 s

constexpr int kOrbitRadialBits = 15;
constexpr int kOrbitAlongBits = 13;
constexpr int kOrbitCrossBits = 13;
constexpr double kOrbitRadialScale = 0.0016;
constexpr double kOrbitAlongScale = 0.0064;
constexpr double kOrbitCrossScale = 0.0064;
constexpr int kClockBits = 15;
constexpr double kClockScale = 0.0016;
constexpr int kCodeBiasBits = 12;
constexpr double kCodeBiasScale = 0.017;
constexpr int kOrbitsPerMessage = 6;
constexpr int kClocksPerMessage = 23;

uint64_t getBits(const std::array<uint8_t, kB2bMessageBytes>& bits, int pos, int len) {
    uint64_t value = 0;
    for (int i = 0; i < len; ++i) {
        const int bit = pos + i;
        value = (value << 1) | ((bits[static_cast<size_t>(bit / 8)] >> (7 - bit % 8)) & 1U);
    }
    return value;
}

int64_t getSignedBits(const std::array<uint8_t, kB2bMessageBytes>& bits, int pos, int len) {
    const uint64_t raw = getBits(bits, pos, len);
    const uint64_t sign = uint64_t{1} << (len - 1);
    return (raw & sign) ? static_cast<int64_t>(raw) - static_cast<int64_t>(uint64_t{1} << len)
                        : static_cast<int64_t>(raw);
}

// The most negative value marks an invalid correction (ICD); like cssrlib,
// the next one (-(2^(n-1) - 1)) is treated as invalid too.
bool invalidValue(int64_t raw, int bits) {
    const int64_t minimum = -(int64_t{1} << (bits - 1));
    return raw == minimum || raw == minimum + 1;
}

int hexValue(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

uint32_t crc24q(const std::array<uint8_t, kB2bMessageBytes>& bits, int nbits) {
    uint32_t crc = 0;
    for (int i = 0; i < nbits; ++i) {
        const uint32_t bit = static_cast<uint32_t>(getBits(bits, i, 1));
        crc ^= bit << 23;
        crc <<= 1;
        if (crc & 0x1000000U) {
            crc ^= 0x1864CFBU;
        }
    }
    return crc & 0xFFFFFFU;
}

// Reference time of a B2b epoch: seconds of the BDT day of reception, moved
// by a day when that puts it more than half a day from the reception time.
GNSSTime referenceTime(const GNSSTime& reception_gpst, int tod) {
    const GNSSTime reception_bdt = reception_gpst + kBdtMinusGpst;
    const double day_start = std::floor(reception_bdt.tow / kSecondsPerDay) * kSecondsPerDay;
    GNSSTime reference_bdt(reception_bdt.week, day_start + tod);
    const double offset = reference_bdt - reception_bdt;
    if (offset > 0.5 * kSecondsPerDay) {
        reference_bdt = reference_bdt - kSecondsPerDay;
    } else if (offset < -0.5 * kSecondsPerDay) {
        reference_bdt = reference_bdt + kSecondsPerDay;
    }
    return reference_bdt - kBdtMinusGpst;
}

}  // namespace

bool readBdsB2bMessages(const std::string& path,
                        std::vector<BdsB2bMessage>& messages,
                        int prn,
                        BdsB2bReadStats* stats,
                        std::string* error) {
    std::ifstream input(path);
    if (!input) {
        if (error != nullptr) {
            *error = "cannot open " + path;
        }
        return false;
    }
    BdsB2bReadStats local;
    std::string line;
    while (std::getline(input, line)) {
        const auto first = line.find_first_not_of(" \t\r");
        if (first == std::string::npos || line[first] == '#') {
            continue;
        }
        ++local.records;
        std::istringstream fields(line);
        int week = 0;
        double tow = 0.0;
        int message_prn = 0;
        int type = 0;
        int length = 0;
        std::string hex;
        if (!(fields >> week >> tow >> message_prn >> type >> length >> hex) ||
            hex.size() < static_cast<size_t>(kB2bMessageBytes) * 2U - 1U) {
            ++local.skipped;
            continue;
        }
        if (prn > 0 && message_prn != prn) {
            continue;
        }
        BdsB2bMessage message;
        message.time = GNSSTime(week, tow);
        message.prn = message_prn;
        bool valid = true;
        // 486 bits are 121.5 hex digits: the last byte may be one digit.
        for (size_t b = 0; b < message.bits.size() && valid; ++b) {
            const int hi = hexValue(hex[2 * b]);
            const int lo = 2 * b + 1 < hex.size() ? hexValue(hex[2 * b + 1]) : 0;
            valid = hi >= 0 && lo >= 0;
            message.bits[b] = static_cast<uint8_t>((hi << 4) | lo);
        }
        if (!valid) {
            ++local.skipped;
            continue;
        }
        messages.push_back(message);
        ++local.messages;
    }
    if (stats != nullptr) {
        *stats = local;
    }
    return true;
}

bool checkBdsB2bCrc(const BdsB2bMessage& message) {
    const int data_bits = kB2bMessageBits - 24;
    return crc24q(message.bits, data_bits) ==
           static_cast<uint32_t>(getBits(message.bits, data_bits, 24));
}

int bdsB2bMessageType(const BdsB2bMessage& message) {
    return static_cast<int>(getBits(message.bits, 0, 6));
}

SatelliteId bdsB2bSlotSatellite(int slot) {
    if (slot >= 1 && slot <= 63) {
        return SatelliteId(GNSSSystem::BeiDou, static_cast<uint8_t>(slot));
    }
    if (slot >= 64 && slot <= 100) {
        return SatelliteId(GNSSSystem::GPS, static_cast<uint8_t>(slot - 63));
    }
    if (slot >= 101 && slot <= 137) {
        return SatelliteId(GNSSSystem::Galileo, static_cast<uint8_t>(slot - 100));
    }
    if (slot >= 138 && slot <= 174) {
        return SatelliteId(GNSSSystem::GLONASS, static_cast<uint8_t>(slot - 137));
    }
    return SatelliteId(GNSSSystem::UNKNOWN, 0);
}

std::optional<double> bdsB2bUraMeters(int ura_class, int ura_value) {
    if (ura_class == 0 && ura_value == 0) {
        return std::nullopt;
    }
    if (ura_class == 7 && ura_value == 7) {
        return 5.4665;
    }
    return (std::pow(3.0, ura_class) * (1.0 + 0.25 * ura_value) - 1.0) * 1e-3;
}

const char* bdsB2bSignalCode(GNSSSystem system, int signal_index) {
    switch (system) {
        case GNSSSystem::BeiDou:
            switch (signal_index) {
                case 0: return "2I";   // B1I
                case 1: return "1D";   // B1C (data)
                case 2: return "1P";   // B1C (pilot)
                case 4: return "5D";   // B2a (data)
                case 5: return "5P";   // B2a (pilot)
                case 7: return "7D";   // B2b-I
                case 8: return "7P";   // B2b-Q
                case 12: return "6I";  // B3I
                default: return "";
            }
        case GNSSSystem::GPS:
            switch (signal_index) {
                case 0: return "1C";
                case 1: return "1P";
                case 4: return "1L";
                case 5: return "1X";
                case 7: return "2L";
                case 8: return "2X";
                case 11: return "5I";
                case 12: return "5Q";
                case 13: return "5X";
                default: return "";
            }
        case GNSSSystem::Galileo:
            switch (signal_index) {
                case 1: return "1B";
                case 2: return "1C";
                case 4: return "5Q";
                case 5: return "5I";
                case 7: return "7I";
                case 8: return "7Q";
                case 11: return "6C";
                default: return "";
            }
        case GNSSSystem::GLONASS:
            switch (signal_index) {
                case 0: return "1C";
                case 1: return "1P";
                case 2: return "2C";
                default: return "";
            }
        default:
            return "";
    }
}

void BdsB2bDecoder::reset() {
    mask_.clear();
    orbits_.clear();
    clocks_.clear();
    code_biases_.clear();
}

bool BdsB2bDecoder::masked(const SatelliteId& satellite) const {
    for (const auto& entry : mask_) {
        if (entry == satellite) {
            return true;
        }
    }
    return false;
}

void BdsB2bDecoder::emit(const SatelliteId& satellite,
                         const GNSSTime& time,
                         const GNSSTime& received) {
    const auto orbit = orbits_.find(satellite);
    const auto clock = clocks_.find(satellite);
    if (orbit == orbits_.end() || clock == clocks_.end()) {
        return;
    }
    const BdsB2bOrbit& o = orbit->second.orbit;
    const ClockState& c = clock->second;
    // Orbit and clock pair by IOD Corr; a clock that just moved to a new IOD
    // Corr still serves an orbit on the previous one.
    double c0_m = 0.0;
    GNSSTime clock_time;
    bool paired = false;
    if (c.iod_corr == o.iod_corr) {
        paired = c.available;
        c0_m = c.c0_m;
        clock_time = c.time;
    } else if (c.has_previous && c.previous_iod_corr == o.iod_corr) {
        paired = c.previous_available;
        c0_m = c.previous_c0_m;
        clock_time = c.previous_time;
    }
    BdsB2bUpdate update;
    update.satellite = satellite;
    update.time = time;
    update.received = received;
    if (!paired || !o.available) {
        update.usable = false;
        updates_.push_back(std::move(update));
        return;
    }
    update.orbit_time = orbit->second.time;
    update.clock_time = clock_time;
    update.iodssr = iodssr_;
    update.iodp = iodp_;
    update.iodn = o.iodn;
    update.iode = o.iodn & 0xff;
    update.iod_corr = o.iod_corr;
    update.radial_m = o.radial_m;
    update.along_m = o.along_m;
    update.cross_m = o.cross_m;
    update.c0_m = c0_m;
    update.ura_m = bdsB2bUraMeters(o.ura_class, o.ura_value);
    const auto bias = code_biases_.find(satellite);
    if (bias != code_biases_.end()) {
        update.code_bias_m = bias->second.bias_m;
    }
    updates_.push_back(std::move(update));
    ++stats_.updates;
}

int BdsB2bDecoder::addMessage(const BdsB2bMessage& message) {
    if (prn_ == 0) {
        prn_ = message.prn;
    }
    if (message.prn != prn_) {
        ++stats_.other_satellite_messages;
        return -1;
    }
    ++stats_.messages;
    if (options_.check_crc && !checkBdsB2bCrc(message)) {
        ++stats_.crc_failures;
        return -1;
    }
    const auto& bits = message.bits;
    const int type = bdsB2bMessageType(message);
    if (type == 63) {
        ++stats_.null_messages;
        return type;
    }
    if (type < 1 || type > 4) {
        ++stats_.unsupported_messages;
        return type;
    }

    BdsB2bDecodedMessage decoded;
    decoded.reception_time = message.time;
    decoded.prn = message.prn;
    decoded.type = type;
    int pos = 6;
    decoded.header.tod = static_cast<int>(getBits(bits, pos, 17));
    decoded.header.iodssr = static_cast<int>(getBits(bits, pos + 21, 2));  // 4 reserved bits
    pos += 23;
    decoded.reference_time = referenceTime(message.time, decoded.header.tod);
    const GNSSTime& time = decoded.reference_time;

    if (type == 1) {
        ++stats_.masks;
        decoded.iodp = static_cast<int>(getBits(bits, pos, 4));
        pos += 4;
        // BDS 63, GPS 37, Galileo 37, GLONASS 37 slots, in that order.
        const int slot_counts[4] = {63, 37, 37, 37};
        const int slot_offsets[4] = {0, 63, 100, 137};
        for (int system = 0; system < 4; ++system) {
            for (int k = 0; k < slot_counts[system]; ++k) {
                if (getBits(bits, pos + k, 1)) {
                    decoded.mask.push_back(bdsB2bSlotSatellite(slot_offsets[system] + k + 1));
                }
            }
            pos += slot_counts[system];
        }
        if (decoded.iodp != iodp_) {
            if (iodp_ >= 0) {
                ++stats_.mask_changes;
            }
            reset();
            iodp_ = decoded.iodp;
            mask_ = decoded.mask;
        }
        iodssr_ = decoded.header.iodssr;
    } else if (iodp_ < 0) {
        ++stats_.without_mask;
        return type;
    } else if (decoded.header.iodssr != iodssr_) {
        ++stats_.iodssr_mismatches;
        return type;
    } else if (type == 2) {
        ++stats_.orbit_messages;
        for (int k = 0; k < kOrbitsPerMessage; ++k) {
            BdsB2bOrbit orbit;
            orbit.slot = static_cast<int>(getBits(bits, pos, 9));
            orbit.iodn = static_cast<int>(getBits(bits, pos + 9, 10));
            orbit.iod_corr = static_cast<int>(getBits(bits, pos + 19, 3));
            pos += 22;
            const int64_t radial = getSignedBits(bits, pos, kOrbitRadialBits);
            pos += kOrbitRadialBits;
            const int64_t along = getSignedBits(bits, pos, kOrbitAlongBits);
            pos += kOrbitAlongBits;
            const int64_t cross = getSignedBits(bits, pos, kOrbitCrossBits);
            pos += kOrbitCrossBits;
            orbit.ura_class = static_cast<int>(getBits(bits, pos, 3));
            orbit.ura_value = static_cast<int>(getBits(bits, pos + 3, 3));
            pos += 6;
            orbit.satellite = bdsB2bSlotSatellite(orbit.slot);
            if (orbit.satellite.system == GNSSSystem::UNKNOWN || !masked(orbit.satellite)) {
                continue;
            }
            orbit.available = !invalidValue(radial, kOrbitRadialBits) &&
                              !invalidValue(along, kOrbitAlongBits) &&
                              !invalidValue(cross, kOrbitCrossBits);
            orbit.radial_m = static_cast<double>(radial) * kOrbitRadialScale;
            orbit.along_m = static_cast<double>(along) * kOrbitAlongScale;
            orbit.cross_m = static_cast<double>(cross) * kOrbitCrossScale;
            orbits_[orbit.satellite] = OrbitState{time, orbit};
            decoded.orbits.push_back(orbit);
            emit(orbit.satellite, time, message.time);
        }
    } else if (type == 3) {
        ++stats_.code_bias_messages;
        const int satellites = static_cast<int>(getBits(bits, pos, 5));
        pos += 5;
        for (int k = 0; k < satellites; ++k) {
            const int slot = static_cast<int>(getBits(bits, pos, 9));
            const int signals = static_cast<int>(getBits(bits, pos + 9, 4));
            pos += 13;
            BdsB2bCodeBias bias;
            bias.satellite = bdsB2bSlotSatellite(slot);
            for (int j = 0; j < signals; ++j) {
                const int signal = static_cast<int>(getBits(bits, pos, 4));
                const int64_t raw = getSignedBits(bits, pos + 4, kCodeBiasBits);
                pos += 4 + kCodeBiasBits;
                if (!invalidValue(raw, kCodeBiasBits)) {
                    bias.bias_m[signal] = static_cast<double>(raw) * kCodeBiasScale;
                }
            }
            if (bias.satellite.system == GNSSSystem::UNKNOWN || !masked(bias.satellite)) {
                continue;
            }
            code_biases_[bias.satellite] = BiasState{time, bias.bias_m};
            decoded.code_biases.push_back(std::move(bias));
        }
    } else {  // type 4
        decoded.iodp = static_cast<int>(getBits(bits, pos, 4));
        decoded.subtype = static_cast<int>(getBits(bits, pos + 4, 5));
        pos += 9;
        if (decoded.iodp != iodp_) {
            ++stats_.iodp_mismatches;
            return type;
        }
        ++stats_.clock_messages;
        for (int k = 0; k < kClocksPerMessage; ++k) {
            const int iod_corr = static_cast<int>(getBits(bits, pos, 3));
            const int64_t raw = getSignedBits(bits, pos + 3, kClockBits);
            pos += 3 + kClockBits;
            const size_t index = static_cast<size_t>(decoded.subtype) * kClocksPerMessage + k;
            if (index >= mask_.size()) {
                continue;
            }
            BdsB2bClock clock;
            clock.satellite = mask_[index];
            clock.iod_corr = iod_corr;
            clock.available = !invalidValue(raw, kClockBits);
            clock.c0_m = static_cast<double>(raw) * kClockScale;
            auto& state = clocks_[clock.satellite];
            if (state.iod_corr >= 0 && state.iod_corr != iod_corr) {
                state.has_previous = true;
                state.previous_iod_corr = state.iod_corr;
                state.previous_c0_m = state.c0_m;
                state.previous_available = state.available;
                state.previous_time = state.time;
            }
            state.time = time;
            state.iod_corr = iod_corr;
            state.c0_m = clock.c0_m;
            state.available = clock.available;
            decoded.clocks.push_back(clock);
            emit(clock.satellite, time, message.time);
        }
    }

    if (options_.keep_messages) {
        messages_.push_back(std::move(decoded));
    }
    return type;
}

}  // namespace io
}  // namespace libgnss
