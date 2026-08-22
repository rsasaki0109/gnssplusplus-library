#include "../../include/libgnss++/io/rtcm.hpp"
#include "../../include/libgnss++/io/ntrip.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <string>

#ifndef _WIN32
#include <netdb.h>
#include <sys/socket.h>
#include <cerrno>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

#include "rtcm_internal.hpp"


namespace libgnss {
namespace io {
using namespace rtcm_internal;

bool RTCMProcessor::decodeEphemerisMessage(const RTCMMessage& message, NavigationData& nav_data) {
    int bit_pos = 0;
    const uint64_t message_number = readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 12);
    bit_pos += 12;

    if (message.type == RTCMMessageType::RTCM_1019) {
        if (message.data.size() * 8U < 488U || message_number != 1019U) {
            return false;
        }

        const uint8_t prn = static_cast<uint8_t>(
            readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 6));
        bit_pos += 6;
        if (prn == 0 || prn > 32) {
            return false;
        }

        Ephemeris eph;
        eph.satellite = SatelliteId(GNSSSystem::GPS, prn);
        eph.week = static_cast<uint16_t>(adjustGpsWeek(static_cast<uint16_t>(
            readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 10))));
        bit_pos += 10;
        eph.ura = static_cast<uint8_t>(
            readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 4));
        eph.sv_accuracy = uraMetersFromIndex(eph.ura);
        bit_pos += 4;
        bit_pos += 2;  // code on L2
        eph.idot = static_cast<double>(
            readSignedBits(message.data.data(), message.data.size(), bit_pos, 14)) *
                   kPow2Neg43 * kSemiCircleToRadians;
        eph.i_dot = eph.idot;
        bit_pos += 14;
        eph.iode = static_cast<uint16_t>(
            readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 8));
        bit_pos += 8;
        const double toc = static_cast<double>(
            readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 16)) * 16.0;
        bit_pos += 16;
        eph.af2 = static_cast<double>(
            readSignedBits(message.data.data(), message.data.size(), bit_pos, 8)) * kPow2Neg55;
        bit_pos += 8;
        eph.af1 = static_cast<double>(
            readSignedBits(message.data.data(), message.data.size(), bit_pos, 16)) * kPow2Neg43;
        bit_pos += 16;
        eph.af0 = static_cast<double>(
            readSignedBits(message.data.data(), message.data.size(), bit_pos, 22)) * kPow2Neg31;
        bit_pos += 22;
        eph.iodc = static_cast<uint16_t>(
            readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 10));
        bit_pos += 10;
        eph.crs = static_cast<double>(
            readSignedBits(message.data.data(), message.data.size(), bit_pos, 16)) * kPow2Neg5;
        bit_pos += 16;
        eph.delta_n = static_cast<double>(
            readSignedBits(message.data.data(), message.data.size(), bit_pos, 16)) *
                      kPow2Neg43 * kSemiCircleToRadians;
        bit_pos += 16;
        eph.m0 = static_cast<double>(
            readSignedBits(message.data.data(), message.data.size(), bit_pos, 32)) *
                 kPow2Neg31 * kSemiCircleToRadians;
        bit_pos += 32;
        eph.cuc = static_cast<double>(
            readSignedBits(message.data.data(), message.data.size(), bit_pos, 16)) * kPow2Neg29;
        bit_pos += 16;
        eph.e = static_cast<double>(
            readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 32)) * kPow2Neg33;
        bit_pos += 32;
        eph.cus = static_cast<double>(
            readSignedBits(message.data.data(), message.data.size(), bit_pos, 16)) * kPow2Neg29;
        bit_pos += 16;
        eph.sqrt_a = static_cast<double>(
            readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 32)) * kPow2Neg19;
        bit_pos += 32;
        eph.toes = static_cast<double>(
            readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 16)) * 16.0;
        bit_pos += 16;
        eph.cic = static_cast<double>(
            readSignedBits(message.data.data(), message.data.size(), bit_pos, 16)) * kPow2Neg29;
        bit_pos += 16;
        eph.omega0 = static_cast<double>(
            readSignedBits(message.data.data(), message.data.size(), bit_pos, 32)) *
                     kPow2Neg31 * kSemiCircleToRadians;
        bit_pos += 32;
        eph.cis = static_cast<double>(
            readSignedBits(message.data.data(), message.data.size(), bit_pos, 16)) * kPow2Neg29;
        bit_pos += 16;
        eph.i0 = static_cast<double>(
            readSignedBits(message.data.data(), message.data.size(), bit_pos, 32)) *
                 kPow2Neg31 * kSemiCircleToRadians;
        bit_pos += 32;
        eph.crc = static_cast<double>(
            readSignedBits(message.data.data(), message.data.size(), bit_pos, 16)) * kPow2Neg5;
        bit_pos += 16;
        eph.omega = static_cast<double>(
            readSignedBits(message.data.data(), message.data.size(), bit_pos, 32)) *
                    kPow2Neg31 * kSemiCircleToRadians;
        bit_pos += 32;
        eph.omega_dot = static_cast<double>(
            readSignedBits(message.data.data(), message.data.size(), bit_pos, 24)) *
                        kPow2Neg43 * kSemiCircleToRadians;
        bit_pos += 24;
        eph.tgd = static_cast<double>(
            readSignedBits(message.data.data(), message.data.size(), bit_pos, 8)) * kPow2Neg31;
        bit_pos += 8;
        eph.health = static_cast<uint8_t>(
            readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 6));
        eph.sv_health = static_cast<double>(eph.health);
        bit_pos += 6;
        bit_pos += 1;
        bit_pos += 1;

        eph.toe = GNSSTime(eph.week, eph.toes);
        eph.toc = GNSSTime(eph.week, toc);
        eph.valid = true;
        nav_data.addEphemeris(eph);
        return true;
    }

    if (message.type == RTCMMessageType::RTCM_1020) {
        if (message.data.size() * 8U < 360U || message_number != 1020U) {
            return false;
        }

        const uint8_t prn = static_cast<uint8_t>(
            readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 6));
        bit_pos += 6;
        if (prn == 0 || prn > 32) {
            return false;
        }

        Ephemeris eph;
        eph.satellite = SatelliteId(GNSSSystem::GLONASS, prn);
        eph.glonass_frequency_channel = static_cast<int>(
            readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 5)) - 7;
        bit_pos += 5;
        bit_pos += 4;
        const double tk_h = static_cast<double>(
            readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 5));
        bit_pos += 5;
        const double tk_m = static_cast<double>(
            readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 6));
        bit_pos += 6;
        const double tk_s = static_cast<double>(
            readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 1)) * 30.0;
        bit_pos += 1;
        const uint8_t bn = static_cast<uint8_t>(
            readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 1));
        bit_pos += 1;
        bit_pos += 1;
        const uint8_t tb = static_cast<uint8_t>(
            readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 7));
        bit_pos += 7;
        eph.glonass_velocity.x() = static_cast<double>(
            readSignMagnitudeBits(message.data.data(), message.data.size(), bit_pos, 24)) * kPow2Neg20 * 1e3;
        bit_pos += 24;
        eph.glonass_position.x() = static_cast<double>(
            readSignMagnitudeBits(message.data.data(), message.data.size(), bit_pos, 27)) * kPow2Neg11 * 1e3;
        bit_pos += 27;
        eph.glonass_acceleration.x() = static_cast<double>(
            readSignMagnitudeBits(message.data.data(), message.data.size(), bit_pos, 5)) * kPow2Neg30 * 1e3;
        bit_pos += 5;
        eph.glonass_velocity.y() = static_cast<double>(
            readSignMagnitudeBits(message.data.data(), message.data.size(), bit_pos, 24)) * kPow2Neg20 * 1e3;
        bit_pos += 24;
        eph.glonass_position.y() = static_cast<double>(
            readSignMagnitudeBits(message.data.data(), message.data.size(), bit_pos, 27)) * kPow2Neg11 * 1e3;
        bit_pos += 27;
        eph.glonass_acceleration.y() = static_cast<double>(
            readSignMagnitudeBits(message.data.data(), message.data.size(), bit_pos, 5)) * kPow2Neg30 * 1e3;
        bit_pos += 5;
        eph.glonass_velocity.z() = static_cast<double>(
            readSignMagnitudeBits(message.data.data(), message.data.size(), bit_pos, 24)) * kPow2Neg20 * 1e3;
        bit_pos += 24;
        eph.glonass_position.z() = static_cast<double>(
            readSignMagnitudeBits(message.data.data(), message.data.size(), bit_pos, 27)) * kPow2Neg11 * 1e3;
        bit_pos += 27;
        eph.glonass_acceleration.z() = static_cast<double>(
            readSignMagnitudeBits(message.data.data(), message.data.size(), bit_pos, 5)) * kPow2Neg30 * 1e3;
        bit_pos += 5;
        bit_pos += 1;
        eph.glonass_gamn = static_cast<double>(
            readSignMagnitudeBits(message.data.data(), message.data.size(), bit_pos, 11)) * kPow2Neg40;
        bit_pos += 11;
        bit_pos += 3;
        eph.glonass_taun = static_cast<double>(
            readSignMagnitudeBits(message.data.data(), message.data.size(), bit_pos, 22)) * kPow2Neg30;
        bit_pos += 22;
        bit_pos += 5;
        eph.glonass_age = static_cast<int>(
            readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 5));
        bit_pos += 5;
        bit_pos += 1;
        bit_pos += 4;
        bit_pos += 11;
        bit_pos += 2;
        bit_pos += 1;
        bit_pos += 11;
        bit_pos += 32;
        bit_pos += 5;
        bit_pos += 22;
        bit_pos += 1;
        bit_pos += 7;

        eph.health = bn;
        eph.sv_health = static_cast<double>(bn);
        eph.iode = tb & 0x7FU;
        const GNSSTime current_utc = gpstToUtcApprox(currentGpstApprox());
        const GNSSTime tof_utc = alignUtcTimeOfDay(tk_h * 3600.0 + tk_m * 60.0 + tk_s - 10800.0, current_utc);
        const GNSSTime toe_utc = alignUtcTimeOfDay(static_cast<double>(tb) * 900.0 - 10800.0, current_utc);
        eph.tof = utcToGpstApprox(tof_utc);
        eph.toe = utcToGpstApprox(toe_utc);
        eph.week = static_cast<uint16_t>(eph.toe.week);
        eph.valid = true;
        glonass_frequency_channels_[eph.satellite] = eph.glonass_frequency_channel;
        nav_data.addEphemeris(eph);
        return true;
    }

    return false;
}


bool RTCMProcessor::decodeSSRCorrections(const RTCMMessage& message,
                                         std::vector<RTCMSSRCorrection>& corrections) {
    corrections.clear();
    if (!message.valid || message.data.empty() || !isSupportedSsrMessageType(message.type)) {
        return false;
    }

    const uint64_t message_number =
        readUnsignedBits(message.data.data(), message.data.size(), 0, 12);
    if (message_number != static_cast<uint16_t>(message.type)) {
        return false;
    }

    const auto descriptor = ssrDescriptorForMessageType(message.type);
    if (descriptor.system == GNSSSystem::UNKNOWN) {
        return false;
    }

    SsrHeader header;
    const bool orbit_message =
        isSsrOrbitMessageType(message.type) || isSsrCombinedMessageType(message.type);
    const bool clock_message =
        isSsrClockMessageType(message.type) || isSsrCombinedMessageType(message.type);
    const bool code_bias_message = isSsrCodeBiasMessageType(message.type);
    const bool ura_message = isSsrUraMessageType(message.type);
    const bool high_rate_clock_message = isSsrHighRateClockMessageType(message.type);
    const bool header_ok = orbit_message
        ? decodeSsrOrbitHeader(message, descriptor, header)
        : decodeSsrClockHeader(message, descriptor, header);
    if (!header_ok) {
        return false;
    }

    int bit_pos = header.bit_pos;
    const size_t data_bits = message.data.size() * 8U;
    corrections.reserve(static_cast<size_t>(header.satellite_count));

    for (int index = 0; index < header.satellite_count; ++index) {
        RTCMSSRCorrection correction;
        correction.time = header.time;
        correction.update_interval_seconds = header.update_interval_seconds;
        correction.issue_of_data = static_cast<uint8_t>(header.issue_of_data);
        correction.provider_id = header.provider_id;
        correction.solution_id = header.solution_id;
        correction.reference_datum = header.refd != 0;

        if (orbit_message) {
            const int required_bits =
                descriptor.prn_bits + descriptor.iode_bits + descriptor.iodcrc_bits + 121;
            if (static_cast<size_t>(bit_pos + required_bits) > data_bits) {
                return false;
            }

            const uint64_t raw_prn = readUnsignedBits(
                message.data.data(), message.data.size(), bit_pos, descriptor.prn_bits);
            bit_pos += descriptor.prn_bits;
            const uint8_t prn = decodeSsrPrn(descriptor, raw_prn);
            if (prn == 0) {
                return false;
            }
            correction.satellite = SatelliteId(descriptor.system, prn);
            correction.iode = static_cast<int>(
                readUnsignedBits(message.data.data(), message.data.size(), bit_pos, descriptor.iode_bits));
            bit_pos += descriptor.iode_bits;
            if (descriptor.iodcrc_bits > 0) {
                correction.iodcrc = static_cast<int>(
                    readUnsignedBits(message.data.data(), message.data.size(), bit_pos, descriptor.iodcrc_bits));
                bit_pos += descriptor.iodcrc_bits;
            }
            correction.orbit_delta_rac_m.x() = static_cast<double>(
                readSignedBits(message.data.data(), message.data.size(), bit_pos, 22)) * 1e-4;
            bit_pos += 22;
            correction.orbit_delta_rac_m.y() = static_cast<double>(
                readSignedBits(message.data.data(), message.data.size(), bit_pos, 20)) * 4e-4;
            bit_pos += 20;
            correction.orbit_delta_rac_m.z() = static_cast<double>(
                readSignedBits(message.data.data(), message.data.size(), bit_pos, 20)) * 4e-4;
            bit_pos += 20;
            correction.orbit_rate_rac_mps.x() = static_cast<double>(
                readSignedBits(message.data.data(), message.data.size(), bit_pos, 21)) * 1e-6;
            bit_pos += 21;
            correction.orbit_rate_rac_mps.y() = static_cast<double>(
                readSignedBits(message.data.data(), message.data.size(), bit_pos, 19)) * 4e-6;
            bit_pos += 19;
            correction.orbit_rate_rac_mps.z() = static_cast<double>(
                readSignedBits(message.data.data(), message.data.size(), bit_pos, 19)) * 4e-6;
            bit_pos += 19;
            correction.has_orbit = true;
        }

        if (clock_message) {
            const int required_bits = orbit_message ? 70 : (descriptor.prn_bits + 70);
            if (static_cast<size_t>(bit_pos + required_bits) > data_bits) {
                return false;
            }

            if (!orbit_message) {
                const uint64_t raw_prn = readUnsignedBits(
                    message.data.data(), message.data.size(), bit_pos, descriptor.prn_bits);
                bit_pos += descriptor.prn_bits;
                const uint8_t prn = decodeSsrPrn(descriptor, raw_prn);
                if (prn == 0) {
                    return false;
                }
                correction.satellite = SatelliteId(descriptor.system, prn);
            }
            correction.clock_delta_poly.x() = static_cast<double>(
                readSignedBits(message.data.data(), message.data.size(), bit_pos, 22)) * 1e-4;
            bit_pos += 22;
            correction.clock_delta_poly.y() = static_cast<double>(
                readSignedBits(message.data.data(), message.data.size(), bit_pos, 21)) * 1e-6;
            bit_pos += 21;
            correction.clock_delta_poly.z() = static_cast<double>(
                readSignedBits(message.data.data(), message.data.size(), bit_pos, 27)) * 2e-8;
            bit_pos += 27;
            correction.has_clock = true;
        }

        if (code_bias_message) {
            const int required_bits = descriptor.prn_bits + 5;
            if (static_cast<size_t>(bit_pos + required_bits) > data_bits) {
                return false;
            }
            const uint64_t raw_prn = readUnsignedBits(
                message.data.data(), message.data.size(), bit_pos, descriptor.prn_bits);
            bit_pos += descriptor.prn_bits;
            const uint8_t prn = decodeSsrPrn(descriptor, raw_prn);
            if (prn == 0) {
                return false;
            }
            correction.satellite = SatelliteId(descriptor.system, prn);

            const uint8_t num_biases = static_cast<uint8_t>(
                readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 5));
            bit_pos += 5;
            for (uint8_t bias_index = 0; bias_index < num_biases; ++bias_index) {
                if (static_cast<size_t>(bit_pos + 19) > data_bits) {
                    return false;
                }
                const uint8_t signal_id = static_cast<uint8_t>(
                    readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 5));
                bit_pos += 5;
                const double bias_m = static_cast<double>(
                    readSignedBits(message.data.data(), message.data.size(), bit_pos, 14)) * 0.01;
                bit_pos += 14;
                correction.code_bias_m[signal_id] = bias_m;
            }
            correction.has_code_bias = !correction.code_bias_m.empty();
        }

        if (ura_message) {
            const int required_bits = descriptor.prn_bits + 6;
            if (static_cast<size_t>(bit_pos + required_bits) > data_bits) {
                return false;
            }
            const uint64_t raw_prn = readUnsignedBits(
                message.data.data(), message.data.size(), bit_pos, descriptor.prn_bits);
            bit_pos += descriptor.prn_bits;
            const uint8_t prn = decodeSsrPrn(descriptor, raw_prn);
            if (prn == 0) {
                return false;
            }
            correction.satellite = SatelliteId(descriptor.system, prn);
            correction.ura_index = static_cast<uint8_t>(
                readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 6));
            bit_pos += 6;
            correction.ura_sigma_m = uraMetersFromSsrIndex(correction.ura_index);
            correction.has_ura = true;
        }

        if (high_rate_clock_message) {
            const int required_bits = descriptor.prn_bits + 22;
            if (static_cast<size_t>(bit_pos + required_bits) > data_bits) {
                return false;
            }
            const uint64_t raw_prn = readUnsignedBits(
                message.data.data(), message.data.size(), bit_pos, descriptor.prn_bits);
            bit_pos += descriptor.prn_bits;
            const uint8_t prn = decodeSsrPrn(descriptor, raw_prn);
            if (prn == 0) {
                return false;
            }
            correction.satellite = SatelliteId(descriptor.system, prn);
            correction.high_rate_clock_m = static_cast<double>(
                readSignedBits(message.data.data(), message.data.size(), bit_pos, 22)) * 1e-4;
            bit_pos += 22;
            correction.has_high_rate_clock = true;
        }

        corrections.push_back(correction);
    }

    return !corrections.empty();
}


uint32_t RTCMProcessor::calculateCRC24(const uint8_t* data, size_t length) {
    uint32_t crc = 0;
    for (size_t i = 0; i < length; ++i) {
        const uint8_t table_index = static_cast<uint8_t>(((crc >> 16) ^ data[i]) & 0xFFU);
        crc = (crc << 8) ^ crc24q_table[table_index];
    }
    return crc & 0x00FFFFFF;
}


int64_t RTCMProcessor::getBits(const uint8_t* data, size_t data_size, int bit_pos, int num_bits) {
    return static_cast<int64_t>(readUnsignedBits(data, data_size, bit_pos, num_bits));
}

} // namespace io
} // namespace libgnss
