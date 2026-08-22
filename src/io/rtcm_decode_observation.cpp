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

bool RTCMProcessor::decodeObservationMessage(const RTCMMessage& message, ObservationData& obs_data) {
    if (isGlonassMsmMessageType(message.type)) {
        const bool msm5 = isMsm5MessageType(message.type);
        const bool msm6 = isMsm6MessageType(message.type);
        const bool msm7 = isMsm7MessageType(message.type);
        if (message.data.size() * 8U < 187U) {
            return false;
        }

        int bit_pos = 0;
        const uint64_t message_number = readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 12);
        bit_pos += 12;
        if (message_number != static_cast<uint16_t>(message.type)) {
            return false;
        }

        bit_pos += 12;  // reference station id
        const uint32_t epoch_field =
            static_cast<uint32_t>(readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 30));
        bit_pos += 30;
        bit_pos += 1;  // multiple message bit
        bit_pos += 3;  // issue of data station
        bit_pos += 7;  // session transmit time
        bit_pos += 2;  // clock steering
        bit_pos += 2;  // external clock
        bit_pos += 1;  // smoothing indicator
        bit_pos += 3;  // smoothing interval

        std::vector<uint8_t> sat_ids;
        std::vector<uint8_t> signal_ids;
        sat_ids.reserve(64);
        signal_ids.reserve(32);
        for (int sat = 1; sat <= 64; ++sat) {
            if (readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 1) != 0) {
                sat_ids.push_back(static_cast<uint8_t>(sat));
            }
            bit_pos += 1;
        }
        for (int signal_id = 1; signal_id <= 32; ++signal_id) {
            if (readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 1) != 0) {
                signal_ids.push_back(static_cast<uint8_t>(signal_id));
            }
            bit_pos += 1;
        }

        if (sat_ids.empty() || signal_ids.empty() || sat_ids.size() * signal_ids.size() > 64) {
            return false;
        }

        std::vector<bool> cell_mask(sat_ids.size() * signal_ids.size(), false);
        size_t ncell = 0;
        for (size_t i = 0; i < cell_mask.size(); ++i) {
            cell_mask[i] = readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 1) != 0;
            bit_pos += 1;
            if (cell_mask[i]) {
                ++ncell;
            }
        }

        const size_t required_bits =
            169U + cell_mask.size() + sat_ids.size() * ((msm5 || msm7) ? 36U : 18U) +
            ncell * (msm7 ? 80U : (msm5 ? 63U : (msm6 ? 65U : 48U)));
        if (message.data.size() * 8U < required_bits) {
            return false;
        }

        std::vector<double> rough_ranges(sat_ids.size(), 0.0);
        for (size_t sat_index = 0; sat_index < sat_ids.size(); ++sat_index) {
            const uint32_t int_ms = static_cast<uint32_t>(
                readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 8));
            bit_pos += 8;
            rough_ranges[sat_index] = int_ms == 255U ? 0.0 : static_cast<double>(int_ms) * kRTCMGpsPrUnitMeters;
        }
        std::vector<uint8_t> satellite_info(sat_ids.size(), 15U);
        if (msm5 || msm7) {
            for (size_t sat_index = 0; sat_index < sat_ids.size(); ++sat_index) {
                satellite_info[sat_index] = static_cast<uint8_t>(
                    readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 4));
                bit_pos += 4;
            }
        }
        for (size_t sat_index = 0; sat_index < sat_ids.size(); ++sat_index) {
            const uint32_t mod_ms = static_cast<uint32_t>(
                readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 10));
            bit_pos += 10;
            if (rough_ranges[sat_index] != 0.0) {
                rough_ranges[sat_index] += static_cast<double>(mod_ms) * kPow2Neg10 * kRTCMGpsPrUnitMeters;
            }
        }
        std::vector<double> rough_phase_range_rates(sat_ids.size(), 0.0);
        std::vector<bool> has_rough_phase_range_rate(sat_ids.size(), false);
        if (msm5 || msm7) {
            for (size_t sat_index = 0; sat_index < sat_ids.size(); ++sat_index) {
                const int32_t rough_rate = static_cast<int32_t>(
                    readSignedBits(message.data.data(), message.data.size(), bit_pos, 14));
                bit_pos += 14;
                if (rough_rate != -8192) {
                    rough_phase_range_rates[sat_index] = static_cast<double>(rough_rate);
                    has_rough_phase_range_rate[sat_index] = true;
                }
            }
        }

        struct DecodedCellData {
            int32_t fine_pr = -16384;
            int32_t fine_cp = -2097152;
            uint8_t lock = 0;
            bool half = false;
            uint8_t cnr = 0;
            int16_t fine_rate = -16384;
            int32_t fine_pr_ext = -524288;
            int32_t fine_cp_ext = -8388608;
            uint16_t lock_ext = 0;
            uint16_t cnr_ext = 0;
        };
        std::vector<DecodedCellData> cells(ncell);
        for (auto& cell : cells) {
            if (msm6 || msm7) {
                cell.fine_pr_ext = static_cast<int32_t>(
                    readSignedBits(message.data.data(), message.data.size(), bit_pos, 20));
                bit_pos += 20;
            } else {
                cell.fine_pr = static_cast<int32_t>(
                    readSignedBits(message.data.data(), message.data.size(), bit_pos, 15));
                bit_pos += 15;
            }
        }
        for (auto& cell : cells) {
            if (msm6 || msm7) {
                cell.fine_cp_ext = static_cast<int32_t>(
                    readSignedBits(message.data.data(), message.data.size(), bit_pos, 24));
                bit_pos += 24;
            } else {
                cell.fine_cp = static_cast<int32_t>(
                    readSignedBits(message.data.data(), message.data.size(), bit_pos, 22));
                bit_pos += 22;
            }
        }
        for (auto& cell : cells) {
            if (msm6 || msm7) {
                cell.lock_ext = static_cast<uint16_t>(
                    readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 10));
                bit_pos += 10;
                cell.lock = static_cast<uint8_t>(cell.lock_ext == 0 ? 0U : 15U);
            } else {
                cell.lock = static_cast<uint8_t>(
                    readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 4));
                bit_pos += 4;
            }
        }
        for (auto& cell : cells) {
            cell.half = readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 1) != 0;
            bit_pos += 1;
        }
        for (auto& cell : cells) {
            if (msm6 || msm7) {
                cell.cnr_ext = static_cast<uint16_t>(
                    readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 10));
                bit_pos += 10;
            } else {
                cell.cnr = static_cast<uint8_t>(
                    readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 6));
                bit_pos += 6;
            }
        }
        if (msm5 || msm7) {
            for (auto& cell : cells) {
                cell.fine_rate = static_cast<int16_t>(
                    readSignedBits(message.data.data(), message.data.size(), bit_pos, 15));
                bit_pos += 15;
            }
        }

        obs_data.clear();
        obs_data.time = glonassMsmEpochToGpst(epoch_field);
        if (has_reference_position_) {
            obs_data.receiver_position = reference_position_;
        }

        size_t cell_index = 0;
        for (size_t sat_index = 0; sat_index < sat_ids.size(); ++sat_index) {
            const double rough_range = rough_ranges[sat_index];
            const SatelliteId sat_id(GNSSSystem::GLONASS, sat_ids[sat_index]);
            int frequency_channel = 0;
            bool has_frequency_channel =
                (msm5 || msm7) &&
                decodeGlonassMsmExtendedInfo(satellite_info[sat_index], frequency_channel);
            if (!has_frequency_channel) {
                const auto fcn_it = glonass_frequency_channels_.find(sat_id);
                has_frequency_channel = fcn_it != glonass_frequency_channels_.end();
                if (has_frequency_channel) {
                    frequency_channel = fcn_it->second;
                }
            } else {
                glonass_frequency_channels_[sat_id] = frequency_channel;
            }

            if (rough_range == 0.0) {
                for (size_t sig_index = 0; sig_index < signal_ids.size(); ++sig_index) {
                    if (cell_mask[sat_index * signal_ids.size() + sig_index]) {
                        ++cell_index;
                    }
                }
                continue;
            }
            for (size_t sig_index = 0; sig_index < signal_ids.size(); ++sig_index) {
                if (!cell_mask[sat_index * signal_ids.size() + sig_index]) {
                    continue;
                }
                if (cell_index >= cells.size()) {
                    return false;
                }

                const SignalType signal = decodeGlonassMsmSignal(signal_ids[sig_index]);
                const double wavelength =
                    has_frequency_channel ? glonassSignalWavelength(signal, frequency_channel) : 0.0;
                const auto& cell = cells[cell_index++];
                if (signal == SignalType::SIGNAL_TYPE_COUNT) {
                    continue;
                }

                Observation obs(sat_id, signal);
                if (has_frequency_channel) {
                    obs.has_glonass_frequency_channel = true;
                    obs.glonass_frequency_channel = frequency_channel;
                }
                if (msm6 || msm7) {
                    if (cell.fine_pr_ext != -524288) {
                        obs.has_pseudorange = true;
                        obs.pseudorange =
                            rough_range + static_cast<double>(cell.fine_pr_ext) * kPow2Neg29 * kRTCMGpsPrUnitMeters;
                    }
                } else if (cell.fine_pr != -16384) {
                    obs.has_pseudorange = true;
                    obs.pseudorange =
                        rough_range + static_cast<double>(cell.fine_pr) * kPow2Neg24 * kRTCMGpsPrUnitMeters;
                }
                if (msm6 || msm7) {
                    if (cell.fine_cp_ext != -8388608 && wavelength > 0.0) {
                        obs.has_carrier_phase = true;
                        obs.carrier_phase =
                            (rough_range + static_cast<double>(cell.fine_cp_ext) * kPow2Neg31 * kRTCMGpsPrUnitMeters) /
                            wavelength;
                    }
                } else if (cell.fine_cp != -2097152 && wavelength > 0.0) {
                    obs.has_carrier_phase = true;
                    obs.carrier_phase =
                        (rough_range + static_cast<double>(cell.fine_cp) * kPow2Neg29 * kRTCMGpsPrUnitMeters) /
                        wavelength;
                }
                if ((msm5 || msm7) && cell.fine_rate != -16384 && wavelength > 0.0) {
                    const double rate =
                        (has_rough_phase_range_rate[sat_index] ? rough_phase_range_rates[sat_index] : 0.0) +
                        static_cast<double>(cell.fine_rate) * 1e-4;
                    obs.has_doppler = true;
                    obs.doppler = -rate / wavelength;
                }
                obs.loss_of_lock = cell.lock == 0;
                obs.lli = obs.loss_of_lock ? 1U : 0U;
                if (cell.half) {
                    obs.lli |= 0x02U;
                }
                obs.snr = (msm6 || msm7) ? static_cast<double>(cell.cnr_ext) * 0.0625
                                         : static_cast<double>(cell.cnr);
                obs.signal_strength = static_cast<int>(std::lround(obs.snr / 6.0));
                obs.valid = obs.has_pseudorange || obs.has_carrier_phase || obs.has_doppler;
                if (obs.valid) {
                    obs_data.addObservation(obs);
                }
            }
        }

        return !obs_data.isEmpty();
    }

    if (isFixedFrequencyMsmMessageType(message.type)) {
        const bool msm5 = isMsm5MessageType(message.type);
        const bool msm6 = isMsm6MessageType(message.type);
        const bool msm7 = isMsm7MessageType(message.type);
        const GNSSSystem system = fixedFrequencyMsmSystem(message.type);
        if (system == GNSSSystem::UNKNOWN) {
            return false;
        }

        if (message.data.size() * 8U < 187U) {
            return false;
        }

        int bit_pos = 0;
        const uint64_t message_number = readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 12);
        bit_pos += 12;
        if (message_number != static_cast<uint16_t>(message.type)) {
            return false;
        }

        bit_pos += 12;  // reference station id
        const uint32_t epoch_ms =
            static_cast<uint32_t>(readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 30));
        bit_pos += 30;
        bit_pos += 1;  // multiple message bit
        bit_pos += 3;  // issue of data station
        bit_pos += 7;  // session transmit time
        bit_pos += 2;  // clock steering
        bit_pos += 2;  // external clock
        bit_pos += 1;  // smoothing indicator
        bit_pos += 3;  // smoothing interval

        std::vector<uint8_t> sat_ids;
        std::vector<uint8_t> signal_ids;
        sat_ids.reserve(64);
        signal_ids.reserve(32);
        for (int sat = 1; sat <= 64; ++sat) {
            if (readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 1) != 0) {
                sat_ids.push_back(static_cast<uint8_t>(sat));
            }
            bit_pos += 1;
        }
        for (int signal_id = 1; signal_id <= 32; ++signal_id) {
            if (readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 1) != 0) {
                signal_ids.push_back(static_cast<uint8_t>(signal_id));
            }
            bit_pos += 1;
        }

        if (sat_ids.empty() || signal_ids.empty() || sat_ids.size() * signal_ids.size() > 64) {
            return false;
        }

        std::vector<bool> cell_mask(sat_ids.size() * signal_ids.size(), false);
        size_t ncell = 0;
        for (size_t i = 0; i < cell_mask.size(); ++i) {
            cell_mask[i] = readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 1) != 0;
            bit_pos += 1;
            if (cell_mask[i]) {
                ++ncell;
            }
        }

        const size_t required_bits =
            169U + cell_mask.size() + sat_ids.size() * ((msm5 || msm7) ? 36U : 18U) +
            ncell * (msm7 ? 80U : (msm5 ? 63U : (msm6 ? 65U : 48U)));
        if (message.data.size() * 8U < required_bits) {
            return false;
        }

        std::vector<double> rough_ranges(sat_ids.size(), 0.0);
        for (size_t sat_index = 0; sat_index < sat_ids.size(); ++sat_index) {
            const uint32_t int_ms = static_cast<uint32_t>(
                readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 8));
            bit_pos += 8;
            rough_ranges[sat_index] = int_ms == 255U ? 0.0 : static_cast<double>(int_ms) * kRTCMGpsPrUnitMeters;
        }
        if (msm5 || msm7) {
            for (size_t sat_index = 0; sat_index < sat_ids.size(); ++sat_index) {
                (void)readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 4);
                bit_pos += 4;
            }
        }
        for (size_t sat_index = 0; sat_index < sat_ids.size(); ++sat_index) {
            const uint32_t mod_ms = static_cast<uint32_t>(
                readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 10));
            bit_pos += 10;
            if (rough_ranges[sat_index] != 0.0) {
                rough_ranges[sat_index] += static_cast<double>(mod_ms) * kPow2Neg10 * kRTCMGpsPrUnitMeters;
            }
        }
        std::vector<double> rough_phase_range_rates(sat_ids.size(), 0.0);
        std::vector<bool> has_rough_phase_range_rate(sat_ids.size(), false);
        if (msm5 || msm7) {
            for (size_t sat_index = 0; sat_index < sat_ids.size(); ++sat_index) {
                const int32_t rough_rate = static_cast<int32_t>(
                    readSignedBits(message.data.data(), message.data.size(), bit_pos, 14));
                bit_pos += 14;
                if (rough_rate != -8192) {
                    rough_phase_range_rates[sat_index] = static_cast<double>(rough_rate);
                    has_rough_phase_range_rate[sat_index] = true;
                }
            }
        }

        struct DecodedCellData {
            int32_t fine_pr = -16384;
            int32_t fine_cp = -2097152;
            uint8_t lock = 0;
            bool half = false;
            uint8_t cnr = 0;
            int16_t fine_rate = -16384;
            int32_t fine_pr_ext = -524288;
            int32_t fine_cp_ext = -8388608;
            uint16_t lock_ext = 0;
            uint16_t cnr_ext = 0;
        };
        std::vector<DecodedCellData> cells(ncell);
        for (auto& cell : cells) {
            if (msm6 || msm7) {
                cell.fine_pr_ext = static_cast<int32_t>(
                    readSignedBits(message.data.data(), message.data.size(), bit_pos, 20));
                bit_pos += 20;
            } else {
                cell.fine_pr = static_cast<int32_t>(
                    readSignedBits(message.data.data(), message.data.size(), bit_pos, 15));
                bit_pos += 15;
            }
        }
        for (auto& cell : cells) {
            if (msm6 || msm7) {
                cell.fine_cp_ext = static_cast<int32_t>(
                    readSignedBits(message.data.data(), message.data.size(), bit_pos, 24));
                bit_pos += 24;
            } else {
                cell.fine_cp = static_cast<int32_t>(
                    readSignedBits(message.data.data(), message.data.size(), bit_pos, 22));
                bit_pos += 22;
            }
        }
        for (auto& cell : cells) {
            if (msm6 || msm7) {
                cell.lock_ext = static_cast<uint16_t>(
                    readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 10));
                bit_pos += 10;
                cell.lock = static_cast<uint8_t>(cell.lock_ext == 0 ? 0U : 15U);
            } else {
                cell.lock = static_cast<uint8_t>(
                    readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 4));
                bit_pos += 4;
            }
        }
        for (auto& cell : cells) {
            cell.half = readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 1) != 0;
            bit_pos += 1;
        }
        for (auto& cell : cells) {
            if (msm6 || msm7) {
                cell.cnr_ext = static_cast<uint16_t>(
                    readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 10));
                bit_pos += 10;
            } else {
                cell.cnr = static_cast<uint8_t>(
                    readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 6));
                bit_pos += 6;
            }
        }
        if (msm5 || msm7) {
            for (auto& cell : cells) {
                cell.fine_rate = static_cast<int16_t>(
                    readSignedBits(message.data.data(), message.data.size(), bit_pos, 15));
                bit_pos += 15;
            }
        }

        obs_data.clear();
        obs_data.time = fixedFrequencyMsmEpochToTime(message.type, epoch_ms);
        if (has_reference_position_) {
            obs_data.receiver_position = reference_position_;
        }

        size_t cell_index = 0;
        for (size_t sat_index = 0; sat_index < sat_ids.size(); ++sat_index) {
            const double rough_range = rough_ranges[sat_index];
            if (rough_range == 0.0) {
                for (size_t sig_index = 0; sig_index < signal_ids.size(); ++sig_index) {
                    if (cell_mask[sat_index * signal_ids.size() + sig_index]) {
                        ++cell_index;
                    }
                }
                continue;
            }
            for (size_t sig_index = 0; sig_index < signal_ids.size(); ++sig_index) {
                if (!cell_mask[sat_index * signal_ids.size() + sig_index]) {
                    continue;
                }
                if (cell_index >= cells.size()) {
                    return false;
                }

                const SignalType signal = decodeFixedFrequencyMsmSignal(system, signal_ids[sig_index]);
                const double wavelength = signalWavelength(signal);
                const auto& cell = cells[cell_index++];
                if (signal == SignalType::SIGNAL_TYPE_COUNT) {
                    continue;
                }

                Observation obs(SatelliteId(system, sat_ids[sat_index]), signal);
                if (msm6 || msm7) {
                    if (cell.fine_pr_ext != -524288) {
                        obs.has_pseudorange = true;
                        obs.pseudorange =
                            rough_range + static_cast<double>(cell.fine_pr_ext) * kPow2Neg29 * kRTCMGpsPrUnitMeters;
                    }
                } else if (cell.fine_pr != -16384) {
                    obs.has_pseudorange = true;
                    obs.pseudorange =
                        rough_range + static_cast<double>(cell.fine_pr) * kPow2Neg24 * kRTCMGpsPrUnitMeters;
                }
                if (msm6 || msm7) {
                    if (cell.fine_cp_ext != -8388608 && wavelength > 0.0) {
                        obs.has_carrier_phase = true;
                        obs.carrier_phase =
                            (rough_range + static_cast<double>(cell.fine_cp_ext) * kPow2Neg31 * kRTCMGpsPrUnitMeters) /
                            wavelength;
                    }
                } else if (cell.fine_cp != -2097152 && wavelength > 0.0) {
                    obs.has_carrier_phase = true;
                    obs.carrier_phase =
                        (rough_range + static_cast<double>(cell.fine_cp) * kPow2Neg29 * kRTCMGpsPrUnitMeters) /
                        wavelength;
                }
                if ((msm5 || msm7) && cell.fine_rate != -16384 && wavelength > 0.0) {
                    const double rate =
                        (has_rough_phase_range_rate[sat_index] ? rough_phase_range_rates[sat_index] : 0.0) +
                        static_cast<double>(cell.fine_rate) * 1e-4;
                    obs.has_doppler = true;
                    obs.doppler = -rate / wavelength;
                }
                obs.loss_of_lock = cell.lock == 0;
                obs.lli = obs.loss_of_lock ? 1U : 0U;
                if (cell.half) {
                    obs.lli |= 0x02U;
                }
                obs.snr = (msm6 || msm7) ? static_cast<double>(cell.cnr_ext) * 0.0625
                                         : static_cast<double>(cell.cnr);
                obs.signal_strength = static_cast<int>(std::lround(obs.snr / 6.0));
                obs.valid = obs.has_pseudorange || obs.has_carrier_phase || obs.has_doppler;
                if (obs.valid) {
                    obs_data.addObservation(obs);
                }
            }
        }

        return !obs_data.isEmpty();
    }

    const bool extended = message.type == RTCMMessageType::RTCM_1004;
    if (!extended && message.type != RTCMMessageType::RTCM_1003) {
        return false;
    }
    if (message.data.size() * 8U < 64U) {
        return false;
    }

    int bit_pos = 0;
    const uint64_t message_number = readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 12);
    bit_pos += 12;
    if (message_number != static_cast<uint16_t>(message.type)) {
        return false;
    }

    bit_pos += 12;  // reference station id
    const uint32_t epoch_ms =
        static_cast<uint32_t>(readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 30));
    bit_pos += 30;
    bit_pos += 1;  // synchronous GNSS flag
    const uint64_t num_sats = readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 5);
    bit_pos += 5;
    bit_pos += 1;  // smoothing indicator
    bit_pos += 3;  // smoothing interval

    const size_t bits_per_sat = extended ? 125U : 109U;
    const size_t required_bits = 64U + static_cast<size_t>(num_sats) * bits_per_sat;
    if (message.data.size() * 8U < required_bits) {
        return false;
    }

    obs_data.clear();
    obs_data.time = rtcm_utils::rtcmTimeToGPSTime(epoch_ms, 0);
    if (has_reference_position_) {
        obs_data.receiver_position = reference_position_;
    }

    for (uint64_t sat_index = 0; sat_index < num_sats; ++sat_index) {
        const uint8_t prn =
            static_cast<uint8_t>(readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 6));
        bit_pos += 6;
        const uint64_t l1_code = readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 1);
        bit_pos += 1;
        const uint64_t l1_pr_units = readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 24);
        bit_pos += 24;
        const int64_t l1_phase_delta_units = readSignedBits(message.data.data(), message.data.size(), bit_pos, 20);
        bit_pos += 20;
        const uint8_t l1_lock =
            static_cast<uint8_t>(readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 7));
        bit_pos += 7;
        const uint64_t ambiguity = readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 8);
        bit_pos += 8;
        uint8_t l1_cnr = 0;
        if (extended) {
            l1_cnr =
                static_cast<uint8_t>(readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 8));
            bit_pos += 8;
        }
        const uint64_t l2_code = readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 2);
        bit_pos += 2;
        const int64_t l2_pr_delta_units = readSignedBits(message.data.data(), message.data.size(), bit_pos, 14);
        bit_pos += 14;
        const int64_t l2_phase_delta_units = readSignedBits(message.data.data(), message.data.size(), bit_pos, 20);
        bit_pos += 20;
        const uint8_t l2_lock =
            static_cast<uint8_t>(readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 7));
        bit_pos += 7;
        uint8_t l2_cnr = 0;
        if (extended) {
            l2_cnr =
                static_cast<uint8_t>(readUnsignedBits(message.data.data(), message.data.size(), bit_pos, 8));
            bit_pos += 8;
        }

        if (prn == 0 || prn > 32) {
            continue;
        }

        const double p1 = static_cast<double>(ambiguity) * kRTCMGpsPrUnitMeters +
                          static_cast<double>(l1_pr_units) * kRTCMGpsPseudorangeResolution;
        const double p2 = p1 + static_cast<double>(l2_pr_delta_units) * kRTCMGpsPseudorangeResolution;
        const double l1_range = p1 + static_cast<double>(l1_phase_delta_units) * kRTCMGpsPhaseResolution;
        const double l2_range = p1 + static_cast<double>(l2_phase_delta_units) * kRTCMGpsPhaseResolution;

        Observation l1_obs(SatelliteId(GNSSSystem::GPS, prn), decodeGpsL1Signal(l1_code));
        l1_obs.has_pseudorange = true;
        l1_obs.has_carrier_phase = true;
        l1_obs.pseudorange = p1;
        l1_obs.carrier_phase = l1_range / constants::GPS_L1_WAVELENGTH;
        l1_obs.snr = static_cast<double>(l1_cnr) * kRTCMCn0Resolution;
        l1_obs.signal_strength = static_cast<int>(l1_cnr);
        l1_obs.loss_of_lock = l1_lock == 0;
        l1_obs.lli = l1_obs.loss_of_lock ? 1U : 0U;
        l1_obs.valid = true;
        obs_data.addObservation(l1_obs);

        Observation l2_obs(SatelliteId(GNSSSystem::GPS, prn), decodeGpsL2Signal(l2_code));
        l2_obs.has_pseudorange = true;
        l2_obs.has_carrier_phase = true;
        l2_obs.pseudorange = p2;
        l2_obs.carrier_phase = l2_range / constants::GPS_L2_WAVELENGTH;
        l2_obs.snr = static_cast<double>(l2_cnr) * kRTCMCn0Resolution;
        l2_obs.signal_strength = static_cast<int>(l2_cnr);
        l2_obs.loss_of_lock = l2_lock == 0;
        l2_obs.lli = l2_obs.loss_of_lock ? 1U : 0U;
        l2_obs.valid = true;
        obs_data.addObservation(l2_obs);
    }

    return !obs_data.isEmpty();
}

} // namespace io
} // namespace libgnss
