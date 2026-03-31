#include "libgnss++/io/sbf.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace libgnss {
namespace io {

// ============================================================================
// CRC-16-CCITT Table (from septentrio sbf_blocks.hpp)
// ============================================================================

const std::array<uint16_t, 256> SBFProcessor::crc_table_ = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
    0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
    0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
    0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
    0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
    0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
    0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
    0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
    0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
    0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
    0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
    0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
    0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
    0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
    0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
    0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
    0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
    0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
    0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
    0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
    0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

uint16_t SBFProcessor::computeCRC16(const uint8_t* buf, size_t length) {
    uint16_t crc = 0;
    for (size_t i = 0; i < length; ++i) {
        crc = (crc << 8) ^ crc_table_[static_cast<uint8_t>((crc >> 8) ^ buf[i])];
    }
    return crc;
}

bool SBFProcessor::verifyCRC(const uint8_t* block, size_t length) {
    if (length < SBF_HEADER_LEN) return false;

    // CRC is stored at bytes 2-3 (LE). Computed over bytes 4..end.
    uint16_t stored_crc;
    std::memcpy(&stored_crc, block + 2, 2);

    uint16_t computed = computeCRC16(block + 4, length - 4);
    return stored_crc == computed;
}

// ============================================================================
// Stream Decoder
// ============================================================================

std::vector<SBFMessage> SBFProcessor::decode(const uint8_t* buffer, size_t size) {
    stats_.total_bytes += size;
    residual_.insert(residual_.end(), buffer, buffer + size);

    std::vector<SBFMessage> messages;
    size_t pos = 0;

    while (pos + SBF_HEADER_LEN <= residual_.size()) {
        // Scan for sync
        if (residual_[pos] != SBF_SYNC_1 || residual_[pos + 1] != SBF_SYNC_2) {
            ++pos;
            ++stats_.sync_errors;
            continue;
        }

        // Read length (bytes 6-7, LE)
        if (pos + 8 > residual_.size()) break;

        uint16_t block_len;
        std::memcpy(&block_len, residual_.data() + pos + 6, 2);

        // SBF block length must be >= 8 and multiple of 4
        if (block_len < SBF_HEADER_LEN || (block_len & 3) != 0) {
            ++pos;
            ++stats_.sync_errors;
            continue;
        }

        if (pos + block_len > residual_.size()) break;  // incomplete

        stats_.total_messages++;

        // Verify CRC
        if (!verifyCRC(residual_.data() + pos, block_len)) {
            stats_.crc_errors++;
            pos += 2;
            continue;
        }

        // Parse header fields
        uint16_t id_rev;
        uint32_t tow;
        uint16_t wnc;
        std::memcpy(&id_rev, residual_.data() + pos + 4, 2);
        std::memcpy(&tow, residual_.data() + pos + 8, 4);
        std::memcpy(&wnc, residual_.data() + pos + 12, 2);

        SBFMessage msg;
        msg.id = static_cast<SBFBlockId>(id_rev & 0x1FFF);  // bits 0-12 = block number
        msg.revision = (id_rev >> 13) & 0x07;               // bits 13-15 = revision
        msg.length = block_len;
        msg.tow_ms = tow;
        msg.wnc = wnc;
        msg.valid = true;

        // Payload starts after 14-byte header (sync+crc+id+length+tow+wnc)
        size_t payload_offset = 14;
        if (block_len > payload_offset) {
            msg.payload.assign(residual_.begin() + pos + payload_offset,
                               residual_.begin() + pos + block_len);
        }

        stats_.valid_messages++;
        stats_.message_counts[msg.id]++;

        messages.push_back(std::move(msg));
        pos += block_len;
    }

    if (pos > 0) {
        size_t remaining = residual_.size() - pos;
        if (remaining > 0) {
            std::memmove(residual_.data(), residual_.data() + pos, remaining);
        }
        residual_.resize(remaining);
    }

    if (residual_.size() > 65536) {
        residual_.clear();
    }

    return messages;
}

// ============================================================================
// Message Decoders
// ============================================================================

bool SBFProcessor::decodeMessage(const SBFMessage& message,
                                 ObservationData* /*obs_data*/,
                                 NavigationData* /*nav_data*/,
                                 PositionSolution* pos_data) {
    if (!message.valid) return false;

    switch (message.id) {
        case SBFBlockId::PVTGeodetic: {
            if (!pos_data) return true;
            SBFPvtGeodetic pvt;
            if (!decodePvtGeodetic(message, pvt)) return false;
            return pvtGeodeticToPosition(pvt, message.tow_ms, message.wnc, *pos_data);
        }
        case SBFBlockId::PVTCartesian: {
            if (!pos_data) return true;
            SBFPvtCartesian pvt;
            if (!decodePvtCartesian(message, pvt)) return false;
            // Convert ECEF to position solution
            pos_data->time = GNSSTime(message.wnc, message.tow_ms * 1e-3);
            pos_data->position_ecef = Vector3d(pvt.x, pvt.y, pvt.z);
            pos_data->num_satellites = sbf_sentinel::isDoNotUse(pvt.nr_sv) ? 0 : pvt.nr_sv;
            pos_data->status = (pvt.error == 0 && pvt.mode != 0) ?
                               SolutionStatus::SPP : SolutionStatus::NONE;
            return true;
        }
        default:
            return true;
    }
}

bool SBFProcessor::decodePvtGeodetic(const SBFMessage& msg, SBFPvtGeodetic& pvt) {
    if (msg.payload.size() < sizeof(SBFPvtGeodetic)) return false;
    std::memcpy(&pvt, msg.payload.data(), sizeof(SBFPvtGeodetic));
    return true;
}

bool SBFProcessor::decodePvtCartesian(const SBFMessage& msg, SBFPvtCartesian& pvt) {
    if (msg.payload.size() < sizeof(SBFPvtCartesian)) return false;
    std::memcpy(&pvt, msg.payload.data(), sizeof(SBFPvtCartesian));
    return true;
}

bool SBFProcessor::decodeDop(const SBFMessage& msg, SBFDop& dop) {
    if (msg.payload.size() < sizeof(SBFDop)) return false;
    std::memcpy(&dop, msg.payload.data(), sizeof(SBFDop));
    return true;
}

bool SBFProcessor::pvtGeodeticToPosition(const SBFPvtGeodetic& pvt,
                                          uint32_t tow_ms, uint16_t wnc,
                                          PositionSolution& pos) {
    pos.time = GNSSTime(wnc, tow_ms * 1e-3);

    // Check "Do Not Use" values
    if (sbf_sentinel::isDoNotUse(pvt.latitude) ||
        sbf_sentinel::isDoNotUse(pvt.longitude)) {
        pos.status = SolutionStatus::NONE;
        return true;
    }

    // Mode to SolutionStatus
    // mode: 0=none, 1=StandAlone, 2=DGPS, 3=SBAS, 4=RTK_FLOAT, 5=RTK_FIXED, 6=PPP
    switch (pvt.mode & 0x0F) {
        case 0:  pos.status = SolutionStatus::NONE; break;
        case 1:  pos.status = SolutionStatus::SPP; break;
        case 2:
        case 3:  pos.status = SolutionStatus::DGPS; break;
        case 4:  pos.status = SolutionStatus::FLOAT; break;
        case 5:  pos.status = SolutionStatus::FIXED; break;
        case 6:  pos.status = SolutionStatus::PPP_FLOAT; break;
        default: pos.status = SolutionStatus::SPP; break;
    }

    if (pvt.error != 0) {
        pos.status = SolutionStatus::NONE;
    }

    pos.position_geodetic = GeodeticCoord(pvt.latitude, pvt.longitude, pvt.height);

    // Velocity
    if (!sbf_sentinel::isDoNotUse(pvt.vn)) {
        pos.velocity_ned = Vector3d(pvt.vn, pvt.ve, -pvt.vu);  // vu is up, NED needs down
        pos.has_velocity = true;
    }

    // Accuracy
    if (!sbf_sentinel::isDoNotUse(pvt.h_accuracy)) {
        double h_acc = pvt.h_accuracy * 0.01;  // cm → m
        double v_acc = sbf_sentinel::isDoNotUse(pvt.v_accuracy) ?
                       h_acc * 2.0 : pvt.v_accuracy * 0.01;
        double h_sig = h_acc / 1.96;
        double v_sig = v_acc / 1.96;
        pos.position_covariance = Matrix3d::Zero();
        pos.position_covariance(0, 0) = h_sig * h_sig;
        pos.position_covariance(1, 1) = h_sig * h_sig;
        pos.position_covariance(2, 2) = v_sig * v_sig;
    }

    pos.num_satellites = sbf_sentinel::isDoNotUse(pvt.nr_sv) ? 0 : pvt.nr_sv;
    pos.receiver_clock_bias = sbf_sentinel::isDoNotUse(pvt.rx_clk_bias) ?
                              0.0 : pvt.rx_clk_bias * 1e-3;  // ms → s

    return true;
}

// ============================================================================
// System / Signal Mapping
// ============================================================================

GNSSSystem SBFProcessor::sbfSignalTypeToSystem(uint8_t sig_type) {
    // Septentrio signal type byte: bits 5-7 = constellation
    uint8_t constellation = (sig_type >> 5) & 0x07;
    switch (constellation) {
        case 0: return GNSSSystem::GPS;
        case 1: return GNSSSystem::GLONASS;
        case 2: return GNSSSystem::Galileo;
        case 3: return GNSSSystem::SBAS;
        case 4: return GNSSSystem::BeiDou;
        case 5: return GNSSSystem::QZSS;
        case 6: return GNSSSystem::NavIC;
        default: return GNSSSystem::GPS;
    }
}

SignalType SBFProcessor::sbfSignalTypeToSignal(uint8_t sig_type) {
    // bits 0-4 = signal number, bits 5-7 = constellation
    uint8_t constellation = (sig_type >> 5) & 0x07;
    uint8_t sig_num = sig_type & 0x1F;

    switch (constellation) {
        case 0:  // GPS
            switch (sig_num) {
                case 0:  return SignalType::GPS_L1CA;
                case 1:  return SignalType::GPS_L1P;
                case 2:  return SignalType::GPS_L2P;
                case 3:  return SignalType::GPS_L2C;
                case 4:  return SignalType::GPS_L5;
                default: return SignalType::GPS_L1CA;
            }
        case 1:  // GLONASS
            switch (sig_num) {
                case 0:  return SignalType::GLO_L1CA;
                case 1:  return SignalType::GLO_L1P;
                case 2:  return SignalType::GLO_L2CA;
                case 3:  return SignalType::GLO_L2P;
                default: return SignalType::GLO_L1CA;
            }
        case 2:  // Galileo
            switch (sig_num) {
                case 0:  return SignalType::GAL_E1;
                case 1:  return SignalType::GAL_E5A;
                case 2:  return SignalType::GAL_E5B;
                case 3:  return SignalType::GAL_E6;
                default: return SignalType::GAL_E1;
            }
        case 4:  // BeiDou
            switch (sig_num) {
                case 0:  return SignalType::BDS_B1I;
                case 1:  return SignalType::BDS_B2I;
                case 2:  return SignalType::BDS_B3I;
                case 3:  return SignalType::BDS_B1C;
                case 4:  return SignalType::BDS_B2A;
                default: return SignalType::BDS_B1I;
            }
        case 5:  // QZSS
            switch (sig_num) {
                case 0:  return SignalType::QZS_L1CA;
                case 1:  return SignalType::QZS_L2C;
                case 2:  return SignalType::QZS_L5;
                default: return SignalType::QZS_L1CA;
            }
        default:
            return SignalType::GPS_L1CA;
    }
}

// ============================================================================
// Utilities
// ============================================================================

namespace sbf_utils {

std::string getBlockIdName(SBFBlockId id) {
    switch (id) {
        case SBFBlockId::MeasEpoch:       return "MeasEpoch";
        case SBFBlockId::MeasExtra:       return "MeasExtra";
        case SBFBlockId::PVTCartesian:    return "PVTCartesian";
        case SBFBlockId::PVTGeodetic:     return "PVTGeodetic";
        case SBFBlockId::PosCovCartesian:  return "PosCovCartesian";
        case SBFBlockId::PosCovGeodetic:   return "PosCovGeodetic";
        case SBFBlockId::DOP:             return "DOP";
        case SBFBlockId::VelCovCartesian:  return "VelCovCartesian";
        case SBFBlockId::VelCovGeodetic:   return "VelCovGeodetic";
        case SBFBlockId::ChannelStatus:   return "ChannelStatus";
        case SBFBlockId::ReceiverTime:    return "ReceiverTime";
        case SBFBlockId::QualityInd:      return "QualityInd";
        case SBFBlockId::EndOfPVT:        return "EndOfPVT";
        default: return "UNKNOWN";
    }
}

bool isMeasurementBlock(SBFBlockId id) {
    return id == SBFBlockId::MeasEpoch || id == SBFBlockId::MeasExtra;
}

bool isPositionBlock(SBFBlockId id) {
    return id == SBFBlockId::PVTCartesian || id == SBFBlockId::PVTGeodetic;
}

}  // namespace sbf_utils

}  // namespace io
}  // namespace libgnss
