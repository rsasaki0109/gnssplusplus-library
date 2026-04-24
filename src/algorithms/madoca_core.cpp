#include <libgnss++/algorithms/madoca_core.hpp>

#include <libgnss++/core/constants.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <utility>

namespace libgnss::algorithms::madoca_core {

namespace {

constexpr std::int64_t kGpsEpochUnixSeconds = 315964800;
constexpr int kL6DFrameBytes = qzss_l6::kFrameBytes;
constexpr int kL6DMessageBytes = 218;
constexpr int kL6DHeaderBits = 49;
constexpr int kL6DDataBits = qzss_l6::kDataPartBits;
constexpr int kL6DMaxFrames = 40;
constexpr int kL6DVendorMadoca = 2;
constexpr int kL6DServiceIonosphere = 1;
constexpr int kL6DMessageCoverage = 1;
constexpr int kL6DMessageCorrection = 2;
constexpr int kL6DMaxSystems = 5;
constexpr int kMionoSystemGps = 0;
constexpr int kMionoSystemGlo = 1;
constexpr int kMionoSystemGal = 2;
constexpr int kMionoSystemBds = 3;
constexpr int kMionoSystemQzs = 4;
constexpr int kMionoInvalid14Bit = -8192;
constexpr int kMionoInvalid12Bit = -2048;
constexpr int kMionoInvalid10Bit = -512;
constexpr int kMionoInvalid8Bit = -128;

using L6DFrame = std::array<std::uint8_t, kL6DFrameBytes>;
using MionoAreaKey = std::pair<int, int>;
using MionoAreaMap = std::map<MionoAreaKey, madoca_parity::MionoAreaFixture>;

struct L6DStreamState {
    madoca_parity::GTime gt = {};
    int mgfid = -1;
    int maxframe = 0;
    int frame = 0;
    int tow0 = -1;
    int ep0 = 0;
    int iod = 0;
    std::vector<std::uint8_t> message;
    int message_bits = 0;
};

std::string formatDouble(double value) {
    std::ostringstream stream;
    stream << std::setprecision(17) << value;
    return stream.str();
}

double l1FrequencyHz(const SatelliteId& satellite) {
    switch (satellite.system) {
        case GNSSSystem::GPS:
        case GNSSSystem::Galileo:
        case GNSSSystem::QZSS:
            return constants::GPS_L1_FREQ;
        case GNSSSystem::BeiDou:
            return satellite.prn >= 19 ? constants::BDS_B1C_FREQ : 0.0;
        default:
            return 0.0;
    }
}

SatelliteId normalizeMadocaSatelliteId(const SatelliteId& satellite) {
    if (satellite.system == GNSSSystem::QZSS && satellite.prn >= 193) {
        return SatelliteId(GNSSSystem::QZSS,
                           static_cast<std::uint8_t>(satellite.prn - 192));
    }
    return satellite;
}

std::uint64_t readUnsignedBits(const std::vector<std::uint8_t>& data,
                               int bit,
                               int bit_count) {
    std::uint64_t value = 0;
    for (int offset = 0; offset < bit_count; ++offset) {
        value <<= 1U;
        const int src_bit = bit + offset;
        if (src_bit < 0 || src_bit >= static_cast<int>(data.size() * 8U)) {
            continue;
        }
        const int byte_index = src_bit / 8;
        const int bit_index = 7 - (src_bit % 8);
        value |= static_cast<std::uint64_t>(
            (data[static_cast<std::size_t>(byte_index)] >> bit_index) & 1U);
    }
    return value;
}

std::int64_t readSignedBits(const std::vector<std::uint8_t>& data,
                            int bit,
                            int bit_count) {
    std::uint64_t value = readUnsignedBits(data, bit, bit_count);
    if (bit_count > 0 && bit_count < 64 && (value & (1ULL << (bit_count - 1)))) {
        value |= ~((1ULL << bit_count) - 1ULL);
    }
    return static_cast<std::int64_t>(value);
}

int readFrameBit(const L6DFrame& frame, int bit) {
    const int byte_index = bit / 8;
    if (byte_index < 0 || byte_index >= static_cast<int>(frame.size())) {
        return 0;
    }
    const int bit_index = 7 - (bit % 8);
    return (frame[static_cast<std::size_t>(byte_index)] >> bit_index) & 1U;
}

void setPackedBit(std::vector<std::uint8_t>& data, int bit, int value) {
    if (bit < 0) {
        return;
    }
    const int byte_index = bit / 8;
    if (byte_index >= static_cast<int>(data.size())) {
        data.resize(static_cast<std::size_t>(byte_index + 1), 0);
    }
    const int bit_index = 7 - (bit % 8);
    if (value != 0) {
        data[static_cast<std::size_t>(byte_index)] |=
            static_cast<std::uint8_t>(1U << bit_index);
    }
}

std::vector<std::uint8_t> extractL6DDataPart(const L6DFrame& frame) {
    std::vector<std::uint8_t> data((kL6DDataBits + 7) / 8, 0);
    for (int bit = 0; bit < kL6DDataBits; ++bit) {
        setPackedBit(data, bit, readFrameBit(frame, kL6DHeaderBits + bit));
    }
    return data;
}

void appendL6DDataPart(L6DStreamState& state,
                       const std::vector<std::uint8_t>& data_part) {
    const int start_bit = state.message_bits;
    state.message.resize(static_cast<std::size_t>((start_bit + kL6DDataBits + 7) / 8), 0);
    for (int bit = 0; bit < kL6DDataBits; ++bit) {
        const int byte_index = bit / 8;
        const int bit_index = 7 - (bit % 8);
        const int value = (data_part[static_cast<std::size_t>(byte_index)] >> bit_index) & 1U;
        setPackedBit(state.message, start_bit + bit, value);
    }
    state.message_bits += kL6DDataBits;
}

bool isSupportedL6DPrn(int prn) {
    return prn == 200 || prn == 201 || prn == 197;
}

madoca_parity::GTime madocaTimeFromGpsWeekTow(int gps_week, double tow) {
    while (tow < 0.0) {
        tow += constants::SECONDS_PER_WEEK;
        --gps_week;
    }
    while (tow >= constants::SECONDS_PER_WEEK) {
        tow -= constants::SECONDS_PER_WEEK;
        ++gps_week;
    }

    const double whole_tow = std::floor(tow);
    madoca_parity::GTime time;
    time.time = kGpsEpochUnixSeconds +
                static_cast<std::int64_t>(gps_week) *
                    static_cast<std::int64_t>(constants::SECONDS_PER_WEEK) +
                static_cast<std::int64_t>(whole_tow);
    time.sec = tow - whole_tow;
    return time;
}

bool sameMadocaTime(madoca_parity::GTime lhs, madoca_parity::GTime rhs) {
    return lhs.time == rhs.time && std::abs(lhs.sec - rhs.sec) <= 1e-9;
}

int mionoGnssIdToMadocalibSys(int gnss_id) {
    switch (gnss_id) {
        case kMionoSystemGps:
            return madoca_parity::kSysGps;
        case kMionoSystemGlo:
            return madoca_parity::kSysGlo;
        case kMionoSystemGal:
            return madoca_parity::kSysGal;
        case kMionoSystemBds:
            return madoca_parity::kSysCmp;
        case kMionoSystemQzs:
            return madoca_parity::kSysQzs;
        default:
            return madoca_parity::kSysNone;
    }
}

bool hasMionoSatCorrectionsAtTime(const madoca_parity::MionoCorrResult& result,
                                  madoca_parity::GTime time) {
    for (int index = 0; index < madoca_parity::kMadocalibMaxSat; ++index) {
        if (result.t0[index].time != 0 && sameMadocaTime(result.t0[index], time)) {
            return true;
        }
    }
    return false;
}

std::vector<madoca_parity::MionoAreaFixture> collectMionoAreas(
    const MionoAreaMap& areas) {
    std::vector<madoca_parity::MionoAreaFixture> fixtures;
    fixtures.reserve(areas.size());
    for (const auto& entry : areas) {
        fixtures.push_back(entry.second);
    }
    return fixtures;
}

bool hasInvalidMionoCoefficient(const int coef[6], int type) {
    if (coef[0] == kMionoInvalid14Bit) {
        return true;
    }
    if (type > 0 && (coef[1] == kMionoInvalid12Bit || coef[2] == kMionoInvalid12Bit)) {
        return true;
    }
    if (type > 1 && coef[3] == kMionoInvalid10Bit) {
        return true;
    }
    if (type > 2 && (coef[4] == kMionoInvalid8Bit || coef[5] == kMionoInvalid8Bit)) {
        return true;
    }
    return false;
}

class L6DDecoder {
public:
    L6DDecoder(int gps_week, const Vector3d& receiver_position_ecef)
        : gps_week_(gps_week) {
        rr_[0] = receiver_position_ecef.x();
        rr_[1] = receiver_position_ecef.y();
        rr_[2] = receiver_position_ecef.z();
    }

    std::vector<madoca_parity::MionoCorrResult> feedFrame(const L6DFrame& frame) {
        const std::uint32_t preamble =
            (static_cast<std::uint32_t>(frame[0]) << 24U) |
            (static_cast<std::uint32_t>(frame[1]) << 16U) |
            (static_cast<std::uint32_t>(frame[2]) << 8U) |
            static_cast<std::uint32_t>(frame[3]);
        if (preamble != qzss_l6::kL6Preamble) {
            return {};
        }

        const int prn = frame[4];
        if (!isSupportedL6DPrn(prn)) {
            return {};
        }

        const int type = frame[5];
        const int vendor_id = (type >> 5) & 0x7;
        const int mgfid = (type >> 3) & 0x3;
        const int correction_service_id = (type >> 2) & 0x1;
        const bool subframe_start = (type & 0x1) != 0;
        const bool alert = (frame[6] & 0x80U) != 0;

        if (alert || vendor_id != kL6DVendorMadoca ||
            correction_service_id != kL6DServiceIonosphere) {
            return {};
        }

        L6DStreamState& state = states_[prn];
        if (state.gt.time == 0) {
            state.gt = madocaTimeFromGpsWeekTow(gps_week_, 0.0);
        }

        if (subframe_start) {
            resetAssembly(state);
        }

        const std::vector<std::uint8_t> data_part = extractL6DDataPart(frame);
        if (state.maxframe <= 0) {
            const int message_number = static_cast<int>(readUnsignedBits(data_part, 0, 12));
            if (message_number != kL6DMessageCoverage) {
                return {};
            }
            const int correction_length_bits =
                static_cast<int>(readUnsignedBits(data_part, 54, 16));
            const int area_count = static_cast<int>(readUnsignedBits(data_part, 70, 5));
            state.maxframe = (75 + area_count * 45 + correction_length_bits) /
                                 kL6DDataBits +
                             1;
            if (state.maxframe < 1 || state.maxframe > kL6DMaxFrames) {
                resetAssembly(state);
                return {};
            }
            state.frame = 0;
            state.message.clear();
            state.message_bits = 0;
        }

        ++state.frame;
        appendL6DDataPart(state, data_part);

        if (state.frame != state.maxframe) {
            return {};
        }

        std::vector<madoca_parity::MionoCorrResult> results = decodeMessage(state, mgfid);
        resetAssembly(state);
        return results;
    }

private:
    static void resetAssembly(L6DStreamState& state) {
        state.maxframe = 0;
        state.frame = 0;
        state.message.clear();
        state.message_bits = 0;
    }

    int decodeCoverage(L6DStreamState& state, int bit, int* correction_end_bit) {
        const int epoch_tow = static_cast<int>(readUnsignedBits(state.message, bit, 20));
        bit += 20;
        bit += 4;
        bit += 1;
        const int iod = static_cast<int>(readUnsignedBits(state.message, bit, 4));
        bit += 4;
        const int region_id = static_cast<int>(readUnsignedBits(state.message, bit, 8));
        bit += 8;
        const int region_alert = static_cast<int>(readUnsignedBits(state.message, bit, 1));
        bit += 1;
        const int correction_length_bits =
            static_cast<int>(readUnsignedBits(state.message, bit, 16));
        bit += 16;
        const int area_count = static_cast<int>(readUnsignedBits(state.message, bit, 5));
        bit += 5;

        state.tow0 = (epoch_tow / 3600) * 3600;
        state.ep0 = epoch_tow % 3600;
        state.gt = madocaTimeFromGpsWeekTow(gps_week_, epoch_tow);
        state.iod = iod;

        for (auto area_it = areas_.begin(); area_it != areas_.end();) {
            if (area_it->first.first == region_id) {
                area_it = areas_.erase(area_it);
            } else {
                ++area_it;
            }
        }

        for (int area_index = 0; area_index < area_count; ++area_index) {
            const int area_number = static_cast<int>(readUnsignedBits(state.message, bit, 5));
            bit += 5;
            madoca_parity::MionoAreaFixture& area = areas_[{region_id, area_number}];
            area.region_id = region_id;
            area.area_number = area_number;
            area.rvalid = 1;
            area.ralert = region_alert;
            area.sid = static_cast<int>(readUnsignedBits(state.message, bit, 1));
            bit += 1;
            if (area.sid == 0) {
                const int ref_lat = static_cast<int>(readSignedBits(state.message, bit, 11));
                bit += 11;
                const int ref_lon = static_cast<int>(readUnsignedBits(state.message, bit, 12));
                bit += 12;
                const int span_lat = static_cast<int>(readUnsignedBits(state.message, bit, 8));
                bit += 8;
                const int span_lon = static_cast<int>(readUnsignedBits(state.message, bit, 8));
                bit += 8;
                area.ref[0] = ref_lat * 0.1;
                area.ref[1] = ref_lon * 0.1;
                area.span[0] = span_lat * 0.1;
                area.span[1] = span_lon * 0.1;
            } else {
                const int ref_lat = static_cast<int>(readSignedBits(state.message, bit, 15));
                bit += 15;
                const int ref_lon = static_cast<int>(readUnsignedBits(state.message, bit, 16));
                bit += 16;
                const int span = static_cast<int>(readUnsignedBits(state.message, bit, 8));
                bit += 8;
                area.ref[0] = ref_lat * 0.01;
                area.ref[1] = ref_lon * 0.01;
                area.span[0] = span * 10.0;
                area.span[1] = 0.0;
            }
        }

        if (correction_end_bit != nullptr) {
            *correction_end_bit = bit + correction_length_bits;
        }
        return bit;
    }

    int decodeCorrection(L6DStreamState& state,
                         int bit,
                         int frame_mgfid,
                         std::vector<madoca_parity::MionoCorrResult>& results) {
        int hourly_epoch = static_cast<int>(readUnsignedBits(state.message, bit, 12));
        bit += 12;
        bit += 4;
        bit += 1;
        const int iod = static_cast<int>(readUnsignedBits(state.message, bit, 4));
        bit += 4;
        const int region_id = static_cast<int>(readUnsignedBits(state.message, bit, 8));
        bit += 8;
        const int area_number = static_cast<int>(readUnsignedBits(state.message, bit, 5));
        bit += 5;
        const int correction_type = static_cast<int>(readUnsignedBits(state.message, bit, 2));
        bit += 2;

        bool apply = true;
        if (state.tow0 < 0) {
            apply = false;
        } else {
            if (state.ep0 > hourly_epoch) {
                hourly_epoch += 3600;
            }
            state.gt = madocaTimeFromGpsWeekTow(gps_week_, state.tow0 + hourly_epoch);
            if (state.iod != iod) {
                apply = false;
            }
        }
        if ((state.mgfid & 0x1) != (frame_mgfid & 0x1)) {
            apply = false;
        }

        const MionoAreaKey key{region_id, area_number};
        auto area_it = areas_.find(key);
        if (area_it == areas_.end()) {
            apply = false;
        }

        int satellites_per_system[kL6DMaxSystems] = {};
        for (int system_index = 0; system_index < kL6DMaxSystems; ++system_index) {
            satellites_per_system[system_index] =
                static_cast<int>(readUnsignedBits(state.message, bit, 5));
            bit += 5;
        }

        for (int system_index = 0; system_index < kL6DMaxSystems; ++system_index) {
            for (int sat_index = 0; sat_index < satellites_per_system[system_index]; ++sat_index) {
                const int satellite_id = static_cast<int>(readUnsignedBits(state.message, bit, 6));
                bit += 6;
                const int sqi = static_cast<int>(readUnsignedBits(state.message, bit, 6));
                bit += 6;
                int coef[6] = {};
                coef[0] = static_cast<int>(readSignedBits(state.message, bit, 14));
                bit += 14;
                if (correction_type > 0) {
                    coef[1] = static_cast<int>(readSignedBits(state.message, bit, 12));
                    bit += 12;
                    coef[2] = static_cast<int>(readSignedBits(state.message, bit, 12));
                    bit += 12;
                }
                if (correction_type > 1) {
                    coef[3] = static_cast<int>(readSignedBits(state.message, bit, 10));
                    bit += 10;
                }
                if (correction_type > 2) {
                    coef[4] = static_cast<int>(readSignedBits(state.message, bit, 8));
                    bit += 8;
                    coef[5] = static_cast<int>(readSignedBits(state.message, bit, 8));
                    bit += 8;
                }

                const int prn_offset = system_index == kMionoSystemQzs ? 192 : 0;
                const int sat_no = madoca_parity::satno(
                    mionoGnssIdToMadocalibSys(system_index), prn_offset + satellite_id);
                if (sat_no <= 0 || hasInvalidMionoCoefficient(coef, correction_type)) {
                    continue;
                }
                if (!apply || area_it == areas_.end()) {
                    continue;
                }

                madoca_parity::MionoAreaFixture& area = area_it->second;
                area.type = correction_type;
                madoca_parity::MionoSatCorrection& sat = area.sat[sat_no - 1];
                sat.t0 = state.gt;
                sat.sqi = sqi;
                sat.coef[0] = coef[0] * 0.05;
                sat.coef[1] = coef[1] * 0.02;
                sat.coef[2] = coef[2] * 0.02;
                sat.coef[3] = coef[3] * 0.02;
                sat.coef[4] = coef[4] * 0.005;
                sat.coef[5] = coef[5] * 0.005;
            }
        }

        if (apply && area_it != areas_.end()) {
            area_it->second.avalid = 1;
            appendCurrentCorrectionResult(state, results);
        }
        return bit;
    }

    std::vector<madoca_parity::MionoCorrResult> decodeMessage(L6DStreamState& state,
                                                              int frame_mgfid) {
        std::vector<madoca_parity::MionoCorrResult> results;
        int bit = 0;
        int correction_end_bit = state.message_bits;

        while (bit + 16 <= state.message_bits) {
            const int message_number = static_cast<int>(readUnsignedBits(state.message, bit, 12));
            bit += 12;
            bit += 4;

            if (message_number == kL6DMessageCoverage) {
                state.mgfid = frame_mgfid;
                bit = decodeCoverage(state, bit, &correction_end_bit);
            } else if (message_number == kL6DMessageCorrection) {
                bit = decodeCorrection(state, bit, frame_mgfid, results);
            } else {
                break;
            }

            if (bit >= correction_end_bit) {
                break;
            }
        }
        return results;
    }

    void appendCurrentCorrectionResult(
        const L6DStreamState& state,
        std::vector<madoca_parity::MionoCorrResult>& results) const {
        std::vector<madoca_parity::MionoAreaFixture> fixtures = collectMionoAreas(areas_);
        if (fixtures.empty()) {
            return;
        }
        madoca_parity::MionoCorrResult result;
        if (madoca_parity::miono_get_corr(state.gt,
                                          rr_,
                                          fixtures.data(),
                                          static_cast<int>(fixtures.size()),
                                          &result) != 1) {
            return;
        }
        if (!hasMionoSatCorrectionsAtTime(result, state.gt)) {
            return;
        }
        results.push_back(result);
    }

    int gps_week_ = 0;
    double rr_[3] = {};
    std::map<int, L6DStreamState> states_;
    MionoAreaMap areas_;
};

std::vector<madoca_parity::MionoCorrResult> decodeL6DStreamFile(
    L6DDecoder& decoder,
    const std::string& path) {
    std::vector<madoca_parity::MionoCorrResult> all_results;
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        return all_results;
    }

    file.seekg(0, std::ios::end);
    const std::streamoff file_size = file.tellg();
    file.seekg(0, std::ios::beg);

    const auto has_preamble_at = [&](std::streamoff offset) {
        if (offset < 0 || file_size < offset + 4) {
            return false;
        }
        const std::streampos saved_position = file.tellg();
        file.seekg(offset, std::ios::beg);
        std::array<std::uint8_t, 4> preamble{};
        file.read(reinterpret_cast<char*>(preamble.data()), preamble.size());
        const bool ok = file.gcount() == static_cast<std::streamsize>(preamble.size()) &&
                        preamble[0] == 0x1A && preamble[1] == 0xCF &&
                        preamble[2] == 0xFC && preamble[3] == 0x1D;
        file.clear();
        file.seekg(saved_position);
        return ok;
    };

    int record_bytes = kL6DFrameBytes;
    const bool can_read_250 = file_size > 0 && file_size % kL6DFrameBytes == 0;
    const bool can_read_218 = file_size > 0 && file_size % kL6DMessageBytes == 0;
    if (can_read_218 && has_preamble_at(kL6DMessageBytes) &&
        (!can_read_250 || !has_preamble_at(kL6DFrameBytes))) {
        record_bytes = kL6DMessageBytes;
    } else if (!can_read_250 && can_read_218) {
        record_bytes = kL6DMessageBytes;
    }

    while (file) {
        L6DFrame frame{};
        file.read(reinterpret_cast<char*>(frame.data()), record_bytes);
        if (file.gcount() != record_bytes) {
            break;
        }
        std::vector<madoca_parity::MionoCorrResult> results = decoder.feedFrame(frame);
        all_results.insert(all_results.end(), results.begin(), results.end());
    }
    return all_results;
}

}  // namespace

SatelliteId satelliteIdFromMadocalibSat(int sat) {
    int prn = 0;
    const int sys = madoca_parity::satsys(sat, &prn);
    if (prn <= 0) {
        return SatelliteId(GNSSSystem::UNKNOWN, 0);
    }

    switch (sys) {
        case madoca_parity::kSysGps:
            return SatelliteId(GNSSSystem::GPS, static_cast<std::uint8_t>(prn));
        case madoca_parity::kSysGlo:
            return SatelliteId(GNSSSystem::GLONASS, static_cast<std::uint8_t>(prn));
        case madoca_parity::kSysGal:
            return SatelliteId(GNSSSystem::Galileo, static_cast<std::uint8_t>(prn));
        case madoca_parity::kSysQzs:
            return normalizeMadocaSatelliteId(
                SatelliteId(GNSSSystem::QZSS, static_cast<std::uint8_t>(prn)));
        case madoca_parity::kSysCmp:
        case madoca_parity::kSysBd2:
            return SatelliteId(GNSSSystem::BeiDou, static_cast<std::uint8_t>(prn));
        case madoca_parity::kSysIrn:
            return SatelliteId(GNSSSystem::NavIC, static_cast<std::uint8_t>(prn));
        case madoca_parity::kSysSbs:
            return SatelliteId(GNSSSystem::SBAS, static_cast<std::uint8_t>(prn));
        default:
            return SatelliteId(GNSSSystem::UNKNOWN, 0);
    }
}

GNSSTime gnssTimeFromMadocalibTime(madoca_parity::GTime time) {
    const double gps_seconds = static_cast<double>(time.time - kGpsEpochUnixSeconds) + time.sec;
    int week = static_cast<int>(std::floor(gps_seconds / constants::SECONDS_PER_WEEK));
    double tow = gps_seconds - week * constants::SECONDS_PER_WEEK;
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

double l1StecDelayFactorMeters(const SatelliteId& satellite) {
    const double frequency_hz = l1FrequencyHz(satellite);
    if (frequency_hz <= 0.0) {
        return 0.0;
    }
    return 40.31E16 / (frequency_hz * frequency_hz);
}

bool mionoDelayMetersToStecTecu(const SatelliteId& satellite,
                                double delay_m,
                                double& stec_tecu) {
    const double factor = l1StecDelayFactorMeters(satellite);
    if (factor <= 0.0 || std::isfinite(delay_m) == false) {
        stec_tecu = 0.0;
        return false;
    }
    stec_tecu = delay_m / factor;
    return std::isfinite(stec_tecu);
}

std::vector<SSROrbitClockCorrection> materializeMionoCorrections(
    const madoca_parity::MionoCorrResult& result,
    int network_id) {
    const int atmos_network_id = network_id > 0 ? network_id : result.rid;
    std::vector<SSROrbitClockCorrection> corrections;
    corrections.reserve(madoca_parity::kMadocalibMaxSat);

    for (int sat = 1; sat <= madoca_parity::kMadocalibMaxSat; ++sat) {
        const int index = sat - 1;
        if (result.t0[index].time == 0) {
            continue;
        }
        const SatelliteId satellite = satelliteIdFromMadocalibSat(sat);
        if (satellite.system == GNSSSystem::UNKNOWN) {
            continue;
        }
        double stec_tecu = 0.0;
        if (mionoDelayMetersToStecTecu(satellite, result.delay[index], stec_tecu) == false) {
            continue;
        }

        SSROrbitClockCorrection correction;
        correction.satellite = satellite;
        correction.time = gnssTimeFromMadocalibTime(result.t0[index]);
        correction.atmos_valid = true;
        correction.atmos_network_id = atmos_network_id;
        correction.atmos_tokens["atmos_network_id"] = std::to_string(atmos_network_id);
        correction.atmos_tokens["atmos_stec_avail"] = "1";
        correction.atmos_tokens["madoca_region_id"] = std::to_string(result.rid);
        correction.atmos_tokens["madoca_area_number"] = std::to_string(result.area_number);
        const std::string suffix = ":" + satellite.toString();
        correction.atmos_tokens["atmos_stec_type" + suffix] = "0";
        correction.atmos_tokens["atmos_stec_source_subtype" + suffix] = "madoca_l6d";
        correction.atmos_tokens["atmos_stec_c00_tecu" + suffix] = formatDouble(stec_tecu);
        correction.atmos_tokens["atmos_stec_delay_l1_m" + suffix] =
            formatDouble(result.delay[index]);
        correction.atmos_tokens["atmos_stec_std_l1_m" + suffix] =
            formatDouble(result.std[index]);
        corrections.push_back(correction);
    }

    for (auto& correction : corrections) {
        correction.atmos_tokens["atmos_selected_satellites"] =
            std::to_string(corrections.size());
    }
    return corrections;
}

std::vector<SSROrbitClockCorrection> materializeL6DCorrections(
    const std::vector<madoca_parity::MionoCorrResult>& results,
    int network_id) {
    std::vector<SSROrbitClockCorrection> corrections;
    for (const auto& result : results) {
        std::vector<SSROrbitClockCorrection> rows =
            materializeMionoCorrections(result, network_id);
        corrections.insert(corrections.end(), rows.begin(), rows.end());
    }
    return corrections;
}

std::vector<SSROrbitClockCorrection> materializeL6ECorrections(
    const std::vector<qzss_l6::CssrEpoch>& epochs,
    int preferred_network_id) {
    SSRProducts products;
    qzss_l6::populateSSRProducts(epochs, products, preferred_network_id);

    std::vector<SSROrbitClockCorrection> corrections;
    for (const auto& entry : products.orbit_clock_corrections) {
        corrections.insert(corrections.end(), entry.second.begin(), entry.second.end());
    }
    return corrections;
}

std::vector<madoca_parity::MionoCorrResult> decodeL6DFile(
    const std::string& l6d_file,
    int gps_week,
    const Vector3d& receiver_position_ecef) {
    L6DDecoder decoder(gps_week, receiver_position_ecef);
    return decodeL6DStreamFile(decoder, l6d_file);
}

std::vector<madoca_parity::MionoCorrResult> decodeL6DFiles(
    const std::vector<std::string>& l6d_files,
    int gps_week,
    const Vector3d& receiver_position_ecef) {
    L6DDecoder decoder(gps_week, receiver_position_ecef);
    std::vector<madoca_parity::MionoCorrResult> results;
    for (const std::string& l6d_file : l6d_files) {
        std::vector<madoca_parity::MionoCorrResult> file_results =
            decodeL6DStreamFile(decoder, l6d_file);
        results.insert(results.end(), file_results.begin(), file_results.end());
    }
    return results;
}

std::vector<qzss_l6::CssrEpoch> decodeL6EFile(const std::string& l6e_file,
                                              int gps_week) {
    qzss_l6::L6Decoder decoder;
    return decoder.decodeFile(l6e_file, gps_week);
}

std::vector<qzss_l6::CssrEpoch> decodeL6EFiles(
    const std::vector<std::string>& l6e_files, int gps_week) {
    std::vector<qzss_l6::CssrEpoch> epochs;
    for (const std::string& l6e_file : l6e_files) {
        std::vector<qzss_l6::CssrEpoch> file_epochs = decodeL6EFile(l6e_file, gps_week);
        epochs.insert(epochs.end(), file_epochs.begin(), file_epochs.end());
    }
    return epochs;
}

ppp_shared::PPPConfig makePPPConfig(const NativeCoreConfig& core_config,
                                    const ppp_shared::PPPConfig& base_config) {
    ppp_shared::PPPConfig config = base_config;

    config.use_precise_orbits = false;
    config.use_precise_clocks = false;
    config.use_ssr_corrections = core_config.use_l6e_orbit_clock_bias;
    config.require_ssr_orbit_clock = core_config.use_l6e_orbit_clock_bias;
    config.use_rtklib_broadcast_selection = core_config.use_l6e_orbit_clock_bias;
    config.use_clas_osr_filter = false;
    config.use_carrier_phase_without_precise_products = true;
    config.enable_ambiguity_resolution = core_config.enable_ambiguity_resolution;
    config.estimate_troposphere = true;
    config.initial_troposphere_variance = 0.12 * 0.12;
    config.estimate_ionosphere = false;
    config.prefer_receiver_position_seed = true;
    // Native MADOCA kinematic PPP keeps the filtered position state while
    // allowing the receiver clock to be reseeded from SPP each epoch.
    config.reset_kinematic_position_to_spp_each_epoch = false;
    config.allowed_systems = core_config.allowed_systems;
    // MADOCALIB streams L6 corrections causally and never applies SSR whose
    // broadcast epoch is after the observation epoch.
    config.allow_future_ssr_corrections = false;
    config.clas_epoch_policy =
        ppp_shared::PPPConfig::ClasEpochPolicy::HYBRID_STANDARD_PPP_FALLBACK;
    config.clas_correction_application_policy =
        ppp_shared::PPPConfig::ClasCorrectionApplicationPolicy::ORBIT_CLOCK_BIAS;
    config.clas_atmos_stale_after_seconds =
        std::isfinite(core_config.correction_max_age_seconds) &&
                core_config.correction_max_age_seconds > 0.0
            ? core_config.correction_max_age_seconds
            : kDefaultCorrectionMaxAgeSeconds;

    if (core_config.use_l6d_ionosphere) {
        config.ionex_file_path.clear();
    }
    return config;
}

void NativeCorrectionStore::clear() {
    products_.clear();
    products_.setOrbitCorrectionsAreRac(true);
    summary_ = CorrectionSummary{};
    satellites_.clear();
    source_counts_.clear();
}

void NativeCorrectionStore::addCorrection(const SSROrbitClockCorrection& correction,
                                          CorrectionSource source) {
    if (summary_.total == 0 && products_.orbit_clock_corrections.empty()) {
        products_.setOrbitCorrectionsAreRac(true);
    }

    products_.addCorrection(correction);
    ++summary_.total;
    satellites_.insert(correction.satellite);
    summary_.satellites = satellites_.size();
    if (correction.orbit_valid) {
        ++summary_.orbit;
    }
    if (correction.clock_valid) {
        ++summary_.clock;
    }
    if (correction.code_bias_valid) {
        ++summary_.code_bias;
    }
    if (correction.phase_bias_valid) {
        ++summary_.phase_bias;
    }
    if (correction.atmos_valid) {
        ++summary_.atmosphere;
    }
    ++source_counts_[source];
}

void NativeCorrectionStore::addCorrections(
    const std::vector<SSROrbitClockCorrection>& corrections,
    CorrectionSource source) {
    for (const auto& correction : corrections) {
        addCorrection(correction, source);
    }
}

void NativeCorrectionStore::addMionoCorrections(const madoca_parity::MionoCorrResult& result,
                                                int network_id) {
    addCorrections(materializeMionoCorrections(result, network_id), CorrectionSource::L6D);
}

void NativeCorrectionStore::addL6DCorrections(
    const std::vector<madoca_parity::MionoCorrResult>& results,
    int network_id) {
    addCorrections(materializeL6DCorrections(results, network_id), CorrectionSource::L6D);
}

void NativeCorrectionStore::addL6ECorrections(const std::vector<qzss_l6::CssrEpoch>& epochs,
                                              int preferred_network_id) {
    addCorrections(materializeL6ECorrections(epochs, preferred_network_id), CorrectionSource::L6E);
}

bool NativeCorrectionStore::interpolateCorrection(
    const SatelliteId& sat,
    const GNSSTime& time,
    Vector3d& orbit_correction_ecef,
    double& clock_correction_m,
    double* ura_sigma_m,
    std::map<uint8_t, double>* code_bias_m,
    std::map<uint8_t, double>* phase_bias_m,
    std::map<std::string, std::string>* atmos_tokens,
    int preferred_network_id) const {
    if (empty()) {
        orbit_correction_ecef.setZero();
        clock_correction_m = 0.0;
        if (ura_sigma_m) {
            *ura_sigma_m = 0.0;
        }
        if (code_bias_m) {
            code_bias_m->clear();
        }
        if (phase_bias_m) {
            phase_bias_m->clear();
        }
        if (atmos_tokens) {
            atmos_tokens->clear();
        }
        return false;
    }

    return products_.interpolateCorrection(sat,
                                           time,
                                           orbit_correction_ecef,
                                           clock_correction_m,
                                           ura_sigma_m,
                                           code_bias_m,
                                           phase_bias_m,
                                           atmos_tokens,
                                           nullptr,
                                           nullptr,
                                           nullptr,
                                           preferred_network_id);
}

bool loadCorrections(PPPProcessor& processor, const NativeCorrectionStore& store) {
    return processor.loadSSRProducts(store.products());
}

bool loadL6DCorrections(PPPProcessor& processor,
                        const std::vector<madoca_parity::MionoCorrResult>& results,
                        int network_id) {
    NativeCorrectionStore store;
    store.addL6DCorrections(results, network_id);
    return loadCorrections(processor, store);
}

bool loadL6DFile(PPPProcessor& processor,
                 const std::string& l6d_file,
                 int gps_week,
                 const Vector3d& receiver_position_ecef,
                 int network_id) {
    const auto results = decodeL6DFile(l6d_file, gps_week, receiver_position_ecef);
    if (results.empty()) {
        return false;
    }
    return loadL6DCorrections(processor, results, network_id);
}

bool loadL6DFiles(PPPProcessor& processor,
                  const std::vector<std::string>& l6d_files,
                  int gps_week,
                  const Vector3d& receiver_position_ecef,
                  int network_id) {
    const auto results = decodeL6DFiles(l6d_files, gps_week, receiver_position_ecef);
    if (results.empty()) {
        return false;
    }
    return loadL6DCorrections(processor, results, network_id);
}

bool loadL6ECorrections(PPPProcessor& processor,
                        const std::vector<qzss_l6::CssrEpoch>& epochs,
                        int preferred_network_id) {
    NativeCorrectionStore store;
    store.addL6ECorrections(epochs, preferred_network_id);
    return loadCorrections(processor, store);
}

bool loadL6EFile(PPPProcessor& processor,
                 const std::string& l6e_file,
                 int gps_week,
                 int preferred_network_id) {
    const auto epochs = decodeL6EFile(l6e_file, gps_week);
    if (epochs.empty()) {
        return false;
    }
    return loadL6ECorrections(processor, epochs, preferred_network_id);
}

bool loadL6EFiles(PPPProcessor& processor,
                  const std::vector<std::string>& l6e_files,
                  int gps_week,
                  int preferred_network_id) {
    const auto epochs = decodeL6EFiles(l6e_files, gps_week);
    if (epochs.empty()) {
        return false;
    }
    return loadL6ECorrections(processor, epochs, preferred_network_id);
}

}  // namespace libgnss::algorithms::madoca_core
