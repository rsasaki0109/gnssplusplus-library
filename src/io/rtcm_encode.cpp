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

RTCMMessage RTCMProcessor::encodeObservations(const ObservationData& obs_data,
                                             RTCMMessageType message_type) {
    if (isGlonassMsmMessageType(message_type)) {
        const bool msm5 = isMsm5MessageType(message_type);
        const bool msm6 = isMsm6MessageType(message_type);
        const bool msm7 = isMsm7MessageType(message_type);
        struct MsmSatelliteData {
            uint8_t prn = 0;
            int glonass_frequency_channel = 0;
            uint8_t extended_info = 15U;
            uint32_t rough_range_units = 0;
            double rough_range = 0.0;
            int16_t rough_phase_range_rate = 0;
        };
        struct MsmCellData {
            const Observation* obs = nullptr;
            uint8_t prn = 0;
            uint8_t signal_id = 0;
        };

        std::map<uint8_t, std::map<uint8_t, const Observation*>> glonass_cells;
        for (const auto& obs : obs_data.observations) {
            if (obs.satellite.system != GNSSSystem::GLONASS || obs.satellite.prn == 0 ||
                obs.satellite.prn > 64 || !isSupportedGlonassMsmSignal(obs.signal)) {
                continue;
            }
            if (!obs.has_pseudorange && !obs.has_carrier_phase) {
                continue;
            }
            if (obs.has_glonass_frequency_channel) {
                glonass_frequency_channels_[obs.satellite] = obs.glonass_frequency_channel;
            }

            const uint8_t signal_id = glonassMsmSignalId(obs.signal);
            if (signal_id == 0) {
                continue;
            }
            auto& slot = glonass_cells[obs.satellite.prn][signal_id];
            if (slot == nullptr ||
                (!slot->has_pseudorange && obs.has_pseudorange) ||
                (!slot->has_carrier_phase && obs.has_carrier_phase)) {
                slot = &obs;
            }
        }

        if (glonass_cells.empty()) {
            return RTCMMessage();
        }

        std::vector<uint8_t> sat_ids;
        std::vector<uint8_t> signal_ids;
        sat_ids.reserve(glonass_cells.size());
        std::map<uint8_t, MsmSatelliteData> sat_data_by_prn;
        std::map<uint8_t, bool> signal_present;

        for (const auto& [prn, cells] : glonass_cells) {
            double base_range = std::numeric_limits<double>::quiet_NaN();
            double base_rate = std::numeric_limits<double>::quiet_NaN();
            int frequency_channel = 0;
            bool has_frequency_channel = false;

            for (const auto& [signal_id, obs] : cells) {
                signal_present[signal_id] = true;
                if (obs->has_glonass_frequency_channel) {
                    frequency_channel = obs->glonass_frequency_channel;
                    has_frequency_channel = true;
                } else {
                    const SatelliteId sat_id(GNSSSystem::GLONASS, prn);
                    const auto cache_it = glonass_frequency_channels_.find(sat_id);
                    if (cache_it != glonass_frequency_channels_.end()) {
                        frequency_channel = cache_it->second;
                        has_frequency_channel = true;
                    }
                }
            }

            if (!has_frequency_channel) {
                continue;
            }

            for (const auto& [signal_id, obs] : cells) {
                if (obs->has_pseudorange && std::isfinite(obs->pseudorange)) {
                    base_range = obs->pseudorange;
                    break;
                }
                if (has_frequency_channel &&
                    obs->has_carrier_phase && std::isfinite(obs->carrier_phase)) {
                    const double wavelength = glonassSignalWavelength(obs->signal, frequency_channel);
                    if (wavelength > 0.0) {
                        base_range = obs->carrier_phase * wavelength;
                        break;
                    }
                }
            }
            if (!std::isfinite(base_range) || base_range <= 0.0) {
                continue;
            }
            if (has_frequency_channel) {
                for (const auto& [signal_id, obs] : cells) {
                    base_rate = glonassObservationRangeRateMetersPerSecond(*obs, frequency_channel);
                    if (std::isfinite(base_rate)) {
                        break;
                    }
                }
            }

            const int64_t rough_range_units = static_cast<int64_t>(
                std::llround(base_range / (kRTCMGpsPrUnitMeters * kPow2Neg10)));
            const uint32_t rough_range_int_ms = static_cast<uint32_t>(rough_range_units >> 10);
            if (rough_range_units <= 0 || rough_range_int_ms >= 255U) {
                continue;
            }

            MsmSatelliteData sat_data;
            sat_data.prn = prn;
            sat_data.glonass_frequency_channel = frequency_channel;
            sat_data.extended_info =
                has_frequency_channel ? glonassMsmExtendedInfo(frequency_channel) : 15U;
            sat_data.rough_range_units = static_cast<uint32_t>(rough_range_units);
            sat_data.rough_range =
                static_cast<double>(sat_data.rough_range_units) * kPow2Neg10 * kRTCMGpsPrUnitMeters;
            if (std::isfinite(base_rate)) {
                if (std::abs(base_rate) > 8191.0) {
                    sat_data.rough_phase_range_rate = -8192;
                } else {
                    sat_data.rough_phase_range_rate =
                        static_cast<int16_t>(std::llround(base_rate));
                }
            }
            sat_data_by_prn[prn] = sat_data;
            if (has_frequency_channel) {
                glonass_frequency_channels_[SatelliteId(GNSSSystem::GLONASS, prn)] = frequency_channel;
            }
            sat_ids.push_back(prn);
        }

        if (sat_ids.empty()) {
            return RTCMMessage();
        }

        signal_ids.reserve(signal_present.size());
        for (const auto& [signal_id, present] : signal_present) {
            if (present) {
                signal_ids.push_back(signal_id);
            }
        }

        if (signal_ids.empty() || sat_ids.size() > 64 || signal_ids.size() > 32 ||
            sat_ids.size() * signal_ids.size() > 64) {
            return RTCMMessage();
        }

        std::vector<MsmCellData> cells;
        std::vector<bool> cell_mask(sat_ids.size() * signal_ids.size(), false);
        cells.reserve(glonass_cells.size() * 2U);
        for (size_t sat_index = 0; sat_index < sat_ids.size(); ++sat_index) {
            const auto sat_it = glonass_cells.find(sat_ids[sat_index]);
            if (sat_it == glonass_cells.end()) {
                continue;
            }
            for (size_t sig_index = 0; sig_index < signal_ids.size(); ++sig_index) {
                const auto cell_it = sat_it->second.find(signal_ids[sig_index]);
                if (cell_it == sat_it->second.end()) {
                    continue;
                }
                cell_mask[sat_index * signal_ids.size() + sig_index] = true;
                cells.push_back({cell_it->second, sat_ids[sat_index], signal_ids[sig_index]});
            }
        }

        if (cells.empty()) {
            return RTCMMessage();
        }

        const size_t total_bits =
            169U + cell_mask.size() + sat_ids.size() * ((msm5 || msm7) ? 36U : 18U) +
            cells.size() * (msm7 ? 80U : (msm5 ? 63U : (msm6 ? 65U : 48U)));
        std::vector<uint8_t> payload((total_bits + 7U) / 8U, 0);

        int bit_pos = 0;
        writeUnsignedBits(payload, bit_pos, 12, static_cast<uint16_t>(message_type)); bit_pos += 12;
        writeUnsignedBits(payload, bit_pos, 12, 0); bit_pos += 12;
        writeUnsignedBits(payload, bit_pos, 30, gpstToGlonassMsmEpoch(obs_data.time)); bit_pos += 30;
        writeUnsignedBits(payload, bit_pos, 1, 0); bit_pos += 1;
        writeUnsignedBits(payload, bit_pos, 3, 0); bit_pos += 3;
        writeUnsignedBits(payload, bit_pos, 7, 0); bit_pos += 7;
        writeUnsignedBits(payload, bit_pos, 2, 0); bit_pos += 2;
        writeUnsignedBits(payload, bit_pos, 2, 0); bit_pos += 2;
        writeUnsignedBits(payload, bit_pos, 1, 0); bit_pos += 1;
        writeUnsignedBits(payload, bit_pos, 3, 0); bit_pos += 3;

        for (int prn = 1; prn <= 64; ++prn) {
            writeUnsignedBits(
                payload,
                bit_pos,
                1,
                std::binary_search(sat_ids.begin(), sat_ids.end(), static_cast<uint8_t>(prn)) ? 1U : 0U);
            bit_pos += 1;
        }
        for (int signal_id = 1; signal_id <= 32; ++signal_id) {
            writeUnsignedBits(
                payload,
                bit_pos,
                1,
                std::binary_search(signal_ids.begin(), signal_ids.end(), static_cast<uint8_t>(signal_id)) ? 1U : 0U);
            bit_pos += 1;
        }
        for (const bool present : cell_mask) {
            writeUnsignedBits(payload, bit_pos, 1, present ? 1U : 0U);
            bit_pos += 1;
        }

        for (const uint8_t prn : sat_ids) {
            const auto sat_it = sat_data_by_prn.find(prn);
            if (sat_it == sat_data_by_prn.end()) {
                return RTCMMessage();
            }
            writeUnsignedBits(payload, bit_pos, 8, sat_it->second.rough_range_units >> 10); bit_pos += 8;
        }
        if (msm5 || msm7) {
            for (const uint8_t prn : sat_ids) {
                const auto sat_it = sat_data_by_prn.find(prn);
                if (sat_it == sat_data_by_prn.end()) {
                    return RTCMMessage();
                }
                writeUnsignedBits(payload, bit_pos, 4, sat_it->second.extended_info); bit_pos += 4;
            }
        }
        for (const uint8_t prn : sat_ids) {
            const auto sat_it = sat_data_by_prn.find(prn);
            if (sat_it == sat_data_by_prn.end()) {
                return RTCMMessage();
            }
            writeUnsignedBits(payload, bit_pos, 10, sat_it->second.rough_range_units & 0x3FFU); bit_pos += 10;
        }
        if (msm5 || msm7) {
            for (const uint8_t prn : sat_ids) {
                const auto sat_it = sat_data_by_prn.find(prn);
                if (sat_it == sat_data_by_prn.end()) {
                    return RTCMMessage();
                }
                writeSignedBits(payload, bit_pos, 14, sat_it->second.rough_phase_range_rate);
                bit_pos += 14;
            }
        }

        struct EncodedMsmCellData {
            int32_t fine_pr = -16384;
            int32_t fine_cp = -2097152;
            uint8_t lock = 0;
            uint8_t half = 0;
            uint8_t cnr = 0;
            int16_t fine_rate = -16384;
            int32_t fine_pr_ext = -524288;
            int32_t fine_cp_ext = -8388608;
            uint16_t lock_ext = 0;
            uint16_t cnr_ext = 0;
        };
        std::vector<EncodedMsmCellData> encoded_cells;
        encoded_cells.reserve(cells.size());
        for (const auto& cell : cells) {
            const auto sat_it = sat_data_by_prn.find(cell.prn);
            if (sat_it == sat_data_by_prn.end()) {
                return RTCMMessage();
            }
            const double wavelength =
                glonassSignalWavelength(cell.obs->signal, sat_it->second.glonass_frequency_channel);
            EncodedMsmCellData encoded_cell;

            if (cell.obs->has_pseudorange && std::isfinite(cell.obs->pseudorange)) {
                const double value = cell.obs->pseudorange - sat_it->second.rough_range;
                if (msm6 || msm7) {
                    if (std::abs(value) <= 292.7) {
                        encoded_cell.fine_pr_ext = static_cast<int32_t>(
                            std::llround(value / (kRTCMGpsPrUnitMeters * kPow2Neg29)));
                    }
                } else if (std::abs(value) <= 292.7) {
                    encoded_cell.fine_pr = static_cast<int32_t>(
                        std::llround(value / (kRTCMGpsPrUnitMeters * kPow2Neg24)));
                }
            }
            if (cell.obs->has_carrier_phase && wavelength > 0.0 && std::isfinite(cell.obs->carrier_phase)) {
                const double value = cell.obs->carrier_phase * wavelength - sat_it->second.rough_range;
                if (msm6 || msm7) {
                    if (std::abs(value) <= 1171.0) {
                        encoded_cell.fine_cp_ext = static_cast<int32_t>(
                            std::llround(value / (kRTCMGpsPrUnitMeters * kPow2Neg31)));
                    }
                } else if (std::abs(value) <= 1171.0) {
                    encoded_cell.fine_cp = static_cast<int32_t>(
                        std::llround(value / (kRTCMGpsPrUnitMeters * kPow2Neg29)));
                }
            }
            encoded_cell.lock = static_cast<uint8_t>(msmLockIndicatorFromObservation(*cell.obs));
            encoded_cell.lock_ext = msmExtendedLockIndicatorFromObservation(*cell.obs);
            encoded_cell.half = (cell.obs->lli & 0x02U) != 0U ? 1U : 0U;
            encoded_cell.cnr = msmCn0UnitsFromObservation(*cell.obs);
            encoded_cell.cnr_ext = msmExtendedCn0UnitsFromObservation(*cell.obs);
            if ((msm5 || msm7) && wavelength > 0.0) {
                const double total_rate = glonassObservationRangeRateMetersPerSecond(
                    *cell.obs, sat_it->second.glonass_frequency_channel);
                if (std::isfinite(total_rate) && sat_it->second.rough_phase_range_rate != -8192) {
                    const double fine_rate =
                        total_rate - static_cast<double>(sat_it->second.rough_phase_range_rate);
                    if (std::abs(fine_rate) <= 1.6383) {
                        encoded_cell.fine_rate =
                            static_cast<int16_t>(std::llround(fine_rate / 1e-4));
                    }
                }
            }
            encoded_cells.push_back(encoded_cell);
        }

        for (const auto& encoded_cell : encoded_cells) {
            writeSignedBits(
                payload,
                bit_pos,
                (msm6 || msm7) ? 20 : 15,
                (msm6 || msm7) ? encoded_cell.fine_pr_ext : encoded_cell.fine_pr);
            bit_pos += (msm6 || msm7) ? 20 : 15;
        }
        for (const auto& encoded_cell : encoded_cells) {
            writeSignedBits(
                payload,
                bit_pos,
                (msm6 || msm7) ? 24 : 22,
                (msm6 || msm7) ? encoded_cell.fine_cp_ext : encoded_cell.fine_cp);
            bit_pos += (msm6 || msm7) ? 24 : 22;
        }
        for (const auto& encoded_cell : encoded_cells) {
            writeUnsignedBits(
                payload,
                bit_pos,
                (msm6 || msm7) ? 10 : 4,
                (msm6 || msm7) ? encoded_cell.lock_ext : encoded_cell.lock);
            bit_pos += (msm6 || msm7) ? 10 : 4;
        }
        for (const auto& encoded_cell : encoded_cells) {
            writeUnsignedBits(payload, bit_pos, 1, encoded_cell.half); bit_pos += 1;
        }
        for (const auto& encoded_cell : encoded_cells) {
            writeUnsignedBits(
                payload,
                bit_pos,
                (msm6 || msm7) ? 10 : 6,
                (msm6 || msm7) ? encoded_cell.cnr_ext : encoded_cell.cnr);
            bit_pos += (msm6 || msm7) ? 10 : 6;
        }
        if (msm5 || msm7) {
            for (const auto& encoded_cell : encoded_cells) {
                writeSignedBits(payload, bit_pos, 15, encoded_cell.fine_rate); bit_pos += 15;
            }
        }

        RTCMMessage message;
        message.type = message_type;
        message.length = static_cast<uint16_t>(payload.size());
        message.data = std::move(payload);
        message.valid = true;
        return message;
    }

    if (isFixedFrequencyMsmMessageType(message_type)) {
        const bool msm5 = isMsm5MessageType(message_type);
        const bool msm6 = isMsm6MessageType(message_type);
        const bool msm7 = isMsm7MessageType(message_type);
        const GNSSSystem system = fixedFrequencyMsmSystem(message_type);
        if (system == GNSSSystem::UNKNOWN) {
            return RTCMMessage();
        }

        struct MsmSatelliteData {
            uint8_t prn = 0;
            uint8_t extended_info = 0U;
            uint32_t rough_range_units = 0;
            double rough_range = 0.0;
            int16_t rough_phase_range_rate = 0;
        };
        struct MsmCellData {
            const Observation* obs = nullptr;
            uint8_t prn = 0;
            uint8_t signal_id = 0;
        };

        std::map<uint8_t, std::map<uint8_t, const Observation*>> system_cells;
        for (const auto& obs : obs_data.observations) {
            if (obs.satellite.system != system || obs.satellite.prn == 0 ||
                obs.satellite.prn > 64 || !isSupportedFixedFrequencyMsmSignal(system, obs.signal)) {
                continue;
            }
            if (!obs.has_pseudorange && !obs.has_carrier_phase) {
                continue;
            }

            const uint8_t signal_id = fixedFrequencyMsmSignalId(system, obs.signal);
            if (signal_id == 0) {
                continue;
            }
            auto& slot = system_cells[obs.satellite.prn][signal_id];
            if (slot == nullptr ||
                (!slot->has_pseudorange && obs.has_pseudorange) ||
                (!slot->has_carrier_phase && obs.has_carrier_phase)) {
                slot = &obs;
            }
        }

        if (system_cells.empty()) {
            return RTCMMessage();
        }

        std::vector<uint8_t> sat_ids;
        std::vector<uint8_t> signal_ids;
        sat_ids.reserve(system_cells.size());
        std::map<uint8_t, MsmSatelliteData> sat_data_by_prn;
        std::map<uint8_t, bool> signal_present;

        for (const auto& [prn, cells] : system_cells) {
            double base_range = std::numeric_limits<double>::quiet_NaN();
            double base_rate = std::numeric_limits<double>::quiet_NaN();
            for (const auto& [signal_id, obs] : cells) {
                signal_present[signal_id] = true;
                if (!std::isfinite(base_range)) {
                    base_range = observationRangeMeters(*obs);
                }
                if (!std::isfinite(base_rate)) {
                    base_rate = observationRangeRateMetersPerSecond(*obs);
                }
            }
            if (!std::isfinite(base_range) || base_range <= 0.0) {
                continue;
            }

            const int64_t rough_range_units = static_cast<int64_t>(
                std::llround(base_range / (kRTCMGpsPrUnitMeters * kPow2Neg10)));
            const uint32_t rough_range_int_ms = static_cast<uint32_t>(rough_range_units >> 10);
            if (rough_range_units <= 0 || rough_range_int_ms >= 255U) {
                continue;
            }

            MsmSatelliteData sat_data;
            sat_data.prn = prn;
            sat_data.rough_range_units = static_cast<uint32_t>(rough_range_units);
            sat_data.rough_range =
                static_cast<double>(sat_data.rough_range_units) * kPow2Neg10 * kRTCMGpsPrUnitMeters;
            if (std::isfinite(base_rate)) {
                if (std::abs(base_rate) > 8191.0) {
                    sat_data.rough_phase_range_rate = -8192;
                } else {
                    sat_data.rough_phase_range_rate =
                        static_cast<int16_t>(std::llround(base_rate));
                }
            }
            sat_data_by_prn[prn] = sat_data;
            sat_ids.push_back(prn);
        }

        if (sat_ids.empty()) {
            return RTCMMessage();
        }

        signal_ids.reserve(signal_present.size());
        for (const auto& [signal_id, present] : signal_present) {
            if (present) {
                signal_ids.push_back(signal_id);
            }
        }

        if (signal_ids.empty() || sat_ids.size() > 64 || signal_ids.size() > 32 ||
            sat_ids.size() * signal_ids.size() > 64) {
            return RTCMMessage();
        }

        std::vector<MsmCellData> cells;
        std::vector<bool> cell_mask(sat_ids.size() * signal_ids.size(), false);
        cells.reserve(system_cells.size() * 2U);
        for (size_t sat_index = 0; sat_index < sat_ids.size(); ++sat_index) {
            const auto sat_it = system_cells.find(sat_ids[sat_index]);
            if (sat_it == system_cells.end()) {
                continue;
            }
            for (size_t sig_index = 0; sig_index < signal_ids.size(); ++sig_index) {
                const auto cell_it = sat_it->second.find(signal_ids[sig_index]);
                if (cell_it == sat_it->second.end()) {
                    continue;
                }
                cell_mask[sat_index * signal_ids.size() + sig_index] = true;
                cells.push_back({cell_it->second, sat_ids[sat_index], signal_ids[sig_index]});
            }
        }

        if (cells.empty()) {
            return RTCMMessage();
        }

        const size_t total_bits =
            169U + cell_mask.size() + sat_ids.size() * ((msm5 || msm7) ? 36U : 18U) +
            cells.size() * (msm7 ? 80U : (msm5 ? 63U : (msm6 ? 65U : 48U)));
        std::vector<uint8_t> payload((total_bits + 7U) / 8U, 0);

        int bit_pos = 0;
        writeUnsignedBits(payload, bit_pos, 12, static_cast<uint16_t>(message_type)); bit_pos += 12;
        writeUnsignedBits(payload, bit_pos, 12, 0); bit_pos += 12;
        writeUnsignedBits(payload, bit_pos, 30, fixedFrequencyMsmEpochMs(message_type, obs_data.time)); bit_pos += 30;
        writeUnsignedBits(payload, bit_pos, 1, 0); bit_pos += 1;
        writeUnsignedBits(payload, bit_pos, 3, 0); bit_pos += 3;
        writeUnsignedBits(payload, bit_pos, 7, 0); bit_pos += 7;
        writeUnsignedBits(payload, bit_pos, 2, 0); bit_pos += 2;
        writeUnsignedBits(payload, bit_pos, 2, 0); bit_pos += 2;
        writeUnsignedBits(payload, bit_pos, 1, 0); bit_pos += 1;
        writeUnsignedBits(payload, bit_pos, 3, 0); bit_pos += 3;

        for (int prn = 1; prn <= 64; ++prn) {
            writeUnsignedBits(
                payload,
                bit_pos,
                1,
                std::binary_search(sat_ids.begin(), sat_ids.end(), static_cast<uint8_t>(prn)) ? 1U : 0U);
            bit_pos += 1;
        }
        for (int signal_id = 1; signal_id <= 32; ++signal_id) {
            writeUnsignedBits(
                payload,
                bit_pos,
                1,
                std::binary_search(signal_ids.begin(), signal_ids.end(), static_cast<uint8_t>(signal_id)) ? 1U : 0U);
            bit_pos += 1;
        }
        for (const bool present : cell_mask) {
            writeUnsignedBits(payload, bit_pos, 1, present ? 1U : 0U);
            bit_pos += 1;
        }

        for (const uint8_t prn : sat_ids) {
            const auto sat_it = sat_data_by_prn.find(prn);
            if (sat_it == sat_data_by_prn.end()) {
                return RTCMMessage();
            }
            writeUnsignedBits(payload, bit_pos, 8, sat_it->second.rough_range_units >> 10); bit_pos += 8;
        }
        if (msm5 || msm7) {
            for (const uint8_t prn : sat_ids) {
                const auto sat_it = sat_data_by_prn.find(prn);
                if (sat_it == sat_data_by_prn.end()) {
                    return RTCMMessage();
                }
                writeUnsignedBits(payload, bit_pos, 4, sat_it->second.extended_info); bit_pos += 4;
            }
        }
        for (const uint8_t prn : sat_ids) {
            const auto sat_it = sat_data_by_prn.find(prn);
            if (sat_it == sat_data_by_prn.end()) {
                return RTCMMessage();
            }
            writeUnsignedBits(payload, bit_pos, 10, sat_it->second.rough_range_units & 0x3FFU); bit_pos += 10;
        }
        if (msm5 || msm7) {
            for (const uint8_t prn : sat_ids) {
                const auto sat_it = sat_data_by_prn.find(prn);
                if (sat_it == sat_data_by_prn.end()) {
                    return RTCMMessage();
                }
                writeSignedBits(payload, bit_pos, 14, sat_it->second.rough_phase_range_rate);
                bit_pos += 14;
            }
        }

        struct EncodedMsmCellData {
            int32_t fine_pr = -16384;
            int32_t fine_cp = -2097152;
            uint8_t lock = 0;
            uint8_t half = 0;
            uint8_t cnr = 0;
            int16_t fine_rate = -16384;
            int32_t fine_pr_ext = -524288;
            int32_t fine_cp_ext = -8388608;
            uint16_t lock_ext = 0;
            uint16_t cnr_ext = 0;
        };
        std::vector<EncodedMsmCellData> encoded_cells;
        encoded_cells.reserve(cells.size());
        for (const auto& cell : cells) {
            const auto sat_it = sat_data_by_prn.find(cell.prn);
            if (sat_it == sat_data_by_prn.end()) {
                return RTCMMessage();
            }
            const double wavelength = signalWavelength(cell.obs->signal);
            EncodedMsmCellData encoded_cell;

            if (cell.obs->has_pseudorange && std::isfinite(cell.obs->pseudorange)) {
                const double value = cell.obs->pseudorange - sat_it->second.rough_range;
                if (msm6 || msm7) {
                    if (std::abs(value) <= 292.7) {
                        encoded_cell.fine_pr_ext = static_cast<int32_t>(
                            std::llround(value / (kRTCMGpsPrUnitMeters * kPow2Neg29)));
                    }
                } else if (std::abs(value) <= 292.7) {
                    encoded_cell.fine_pr = static_cast<int32_t>(
                        std::llround(value / (kRTCMGpsPrUnitMeters * kPow2Neg24)));
                }
            }
            if (cell.obs->has_carrier_phase && wavelength > 0.0 && std::isfinite(cell.obs->carrier_phase)) {
                const double value = cell.obs->carrier_phase * wavelength - sat_it->second.rough_range;
                if (msm6 || msm7) {
                    if (std::abs(value) <= 1171.0) {
                        encoded_cell.fine_cp_ext = static_cast<int32_t>(
                            std::llround(value / (kRTCMGpsPrUnitMeters * kPow2Neg31)));
                    }
                } else if (std::abs(value) <= 1171.0) {
                    encoded_cell.fine_cp = static_cast<int32_t>(
                        std::llround(value / (kRTCMGpsPrUnitMeters * kPow2Neg29)));
                }
            }
            encoded_cell.lock = static_cast<uint8_t>(msmLockIndicatorFromObservation(*cell.obs));
            encoded_cell.lock_ext = msmExtendedLockIndicatorFromObservation(*cell.obs);
            encoded_cell.half = (cell.obs->lli & 0x02U) != 0U ? 1U : 0U;
            encoded_cell.cnr = msmCn0UnitsFromObservation(*cell.obs);
            encoded_cell.cnr_ext = msmExtendedCn0UnitsFromObservation(*cell.obs);
            if ((msm5 || msm7) && wavelength > 0.0) {
                const double total_rate = observationRangeRateMetersPerSecond(*cell.obs);
                if (std::isfinite(total_rate) && sat_it->second.rough_phase_range_rate != -8192) {
                    const double fine_rate =
                        total_rate - static_cast<double>(sat_it->second.rough_phase_range_rate);
                    if (std::abs(fine_rate) <= 1.6383) {
                        encoded_cell.fine_rate =
                            static_cast<int16_t>(std::llround(fine_rate / 1e-4));
                    }
                }
            }
            encoded_cells.push_back(encoded_cell);
        }

        for (const auto& encoded_cell : encoded_cells) {
            writeSignedBits(
                payload,
                bit_pos,
                (msm6 || msm7) ? 20 : 15,
                (msm6 || msm7) ? encoded_cell.fine_pr_ext : encoded_cell.fine_pr);
            bit_pos += (msm6 || msm7) ? 20 : 15;
        }
        for (const auto& encoded_cell : encoded_cells) {
            writeSignedBits(
                payload,
                bit_pos,
                (msm6 || msm7) ? 24 : 22,
                (msm6 || msm7) ? encoded_cell.fine_cp_ext : encoded_cell.fine_cp);
            bit_pos += (msm6 || msm7) ? 24 : 22;
        }
        for (const auto& encoded_cell : encoded_cells) {
            writeUnsignedBits(
                payload,
                bit_pos,
                (msm6 || msm7) ? 10 : 4,
                (msm6 || msm7) ? encoded_cell.lock_ext : encoded_cell.lock);
            bit_pos += (msm6 || msm7) ? 10 : 4;
        }
        for (const auto& encoded_cell : encoded_cells) {
            writeUnsignedBits(payload, bit_pos, 1, encoded_cell.half); bit_pos += 1;
        }
        for (const auto& encoded_cell : encoded_cells) {
            writeUnsignedBits(
                payload,
                bit_pos,
                (msm6 || msm7) ? 10 : 6,
                (msm6 || msm7) ? encoded_cell.cnr_ext : encoded_cell.cnr);
            bit_pos += (msm6 || msm7) ? 10 : 6;
        }
        if (msm5 || msm7) {
            for (const auto& encoded_cell : encoded_cells) {
                writeSignedBits(payload, bit_pos, 15, encoded_cell.fine_rate); bit_pos += 15;
            }
        }

        RTCMMessage message;
        message.type = message_type;
        message.length = static_cast<uint16_t>(payload.size());
        message.data = std::move(payload);
        message.valid = true;
        return message;
    }

    const bool extended = message_type == RTCMMessageType::RTCM_1004;
    if (!extended && message_type != RTCMMessageType::RTCM_1003) {
        return RTCMMessage();
    }

    struct GPSPair {
        const Observation* l1 = nullptr;
        const Observation* l2 = nullptr;
    };

    std::map<uint8_t, GPSPair> gps_pairs;
    for (const auto& obs : obs_data.observations) {
        if (obs.satellite.system != GNSSSystem::GPS) {
            continue;
        }
        auto& pair = gps_pairs[obs.satellite.prn];
        if (isSupportedGpsL1Signal(obs.signal)) {
            if (pair.l1 == nullptr && obs.has_pseudorange && obs.has_carrier_phase) {
                pair.l1 = &obs;
            }
        } else if (isSupportedGpsL2Signal(obs.signal)) {
            if (pair.l2 == nullptr && obs.has_pseudorange && obs.has_carrier_phase) {
                pair.l2 = &obs;
            }
        }
    }

    struct EncodedSatellite {
        uint8_t prn = 0;
        uint8_t l1_code = 0;
        uint32_t l1_pr_units = 0;
        int32_t l1_phase_delta_units = 0;
        uint8_t l1_lock = 0;
        uint8_t l1_cnr = 0;
        uint8_t ambiguity = 0;
        uint8_t l2_code = 0;
        int32_t l2_pr_delta_units = 0;
        int32_t l2_phase_delta_units = 0;
        uint8_t l2_lock = 0;
        uint8_t l2_cnr = 0;
    };

    std::vector<EncodedSatellite> encoded_sats;
    encoded_sats.reserve(gps_pairs.size());
    for (const auto& [prn, pair] : gps_pairs) {
        if (pair.l1 == nullptr || pair.l2 == nullptr || prn == 0 || prn > 32) {
            continue;
        }

        const double p1 = pair.l1->pseudorange;
        const double p2 = pair.l2->pseudorange;
        const double l1_range = pair.l1->carrier_phase * constants::GPS_L1_WAVELENGTH;
        const double l2_range = pair.l2->carrier_phase * constants::GPS_L2_WAVELENGTH;
        if (!std::isfinite(p1) || !std::isfinite(p2) ||
            !std::isfinite(l1_range) || !std::isfinite(l2_range) ||
            p1 < 0.0 || p2 < 0.0) {
            continue;
        }

        const double ambiguity_d = std::floor(p1 / kRTCMGpsPrUnitMeters);
        const double l1_pr_mod = p1 - ambiguity_d * kRTCMGpsPrUnitMeters;
        const int64_t ambiguity = static_cast<int64_t>(ambiguity_d);
        const int64_t l1_pr_units = static_cast<int64_t>(std::llround(l1_pr_mod / kRTCMGpsPseudorangeResolution));
        const int64_t l1_phase_delta_units =
            static_cast<int64_t>(std::llround((l1_range - p1) / kRTCMGpsPhaseResolution));
        const int64_t l2_pr_delta_units =
            static_cast<int64_t>(std::llround((p2 - p1) / kRTCMGpsPseudorangeResolution));
        const int64_t l2_phase_delta_units =
            static_cast<int64_t>(std::llround((l2_range - p1) / kRTCMGpsPhaseResolution));

        if (ambiguity < 0 || ambiguity > 255 ||
            l1_pr_units < 0 || l1_pr_units > 0xFFFFFF ||
            l1_phase_delta_units < -(1 << 19) || l1_phase_delta_units > ((1 << 19) - 1) ||
            l2_pr_delta_units < -(1 << 13) || l2_pr_delta_units > ((1 << 13) - 1) ||
            l2_phase_delta_units < -(1 << 19) || l2_phase_delta_units > ((1 << 19) - 1)) {
            continue;
        }

        EncodedSatellite encoded;
        encoded.prn = prn;
        encoded.l1_code = pair.l1->signal == SignalType::GPS_L1CA ? 0U : 1U;
        encoded.l1_pr_units = static_cast<uint32_t>(l1_pr_units);
        encoded.l1_phase_delta_units = static_cast<int32_t>(l1_phase_delta_units);
        encoded.l1_lock = static_cast<uint8_t>(lockIndicatorFromObservation(*pair.l1));
        encoded.l1_cnr = static_cast<uint8_t>(cnrUnitsFromObservation(*pair.l1));
        encoded.ambiguity = static_cast<uint8_t>(ambiguity);
        encoded.l2_code = pair.l2->signal == SignalType::GPS_L2C ? 0U : 1U;
        encoded.l2_pr_delta_units = static_cast<int32_t>(l2_pr_delta_units);
        encoded.l2_phase_delta_units = static_cast<int32_t>(l2_phase_delta_units);
        encoded.l2_lock = static_cast<uint8_t>(lockIndicatorFromObservation(*pair.l2));
        encoded.l2_cnr = static_cast<uint8_t>(cnrUnitsFromObservation(*pair.l2));
        encoded_sats.push_back(encoded);
    }

    if (encoded_sats.empty()) {
        return RTCMMessage();
    }

    const int num_sats = static_cast<int>(encoded_sats.size());
    const int bits_per_sat = extended ? 125 : 109;
    const int total_bits = 64 + num_sats * bits_per_sat;
    std::vector<uint8_t> payload(static_cast<size_t>((total_bits + 7) / 8), 0);

    int bit_pos = 0;
    writeUnsignedBits(payload, bit_pos, 12, static_cast<uint16_t>(message_type)); bit_pos += 12;
    writeUnsignedBits(payload, bit_pos, 12, 0); bit_pos += 12;
    const uint32_t epoch_ms = rtcm_utils::gpsTimeToRTCMTime(obs_data.time);
    writeUnsignedBits(payload, bit_pos, 30, epoch_ms); bit_pos += 30;
    writeUnsignedBits(payload, bit_pos, 1, 0); bit_pos += 1;
    writeUnsignedBits(payload, bit_pos, 5, static_cast<uint64_t>(num_sats)); bit_pos += 5;
    writeUnsignedBits(payload, bit_pos, 1, 0); bit_pos += 1;
    writeUnsignedBits(payload, bit_pos, 3, 0); bit_pos += 3;

    for (const auto& sat : encoded_sats) {
        writeUnsignedBits(payload, bit_pos, 6, sat.prn); bit_pos += 6;
        writeUnsignedBits(payload, bit_pos, 1, sat.l1_code); bit_pos += 1;
        writeUnsignedBits(payload, bit_pos, 24, sat.l1_pr_units); bit_pos += 24;
        writeSignedBits(payload, bit_pos, 20, sat.l1_phase_delta_units); bit_pos += 20;
        writeUnsignedBits(payload, bit_pos, 7, sat.l1_lock); bit_pos += 7;
        writeUnsignedBits(payload, bit_pos, 8, sat.ambiguity); bit_pos += 8;
        if (extended) {
            writeUnsignedBits(payload, bit_pos, 8, sat.l1_cnr); bit_pos += 8;
        }
        writeUnsignedBits(payload, bit_pos, 2, sat.l2_code); bit_pos += 2;
        writeSignedBits(payload, bit_pos, 14, sat.l2_pr_delta_units); bit_pos += 14;
        writeSignedBits(payload, bit_pos, 20, sat.l2_phase_delta_units); bit_pos += 20;
        writeUnsignedBits(payload, bit_pos, 7, sat.l2_lock); bit_pos += 7;
        if (extended) {
            writeUnsignedBits(payload, bit_pos, 8, sat.l2_cnr); bit_pos += 8;
        }
    }

    RTCMMessage message;
    message.type = message_type;
    message.length = static_cast<uint16_t>(payload.size());
    message.data = std::move(payload);
    message.valid = true;
    return message;
}

RTCMMessage RTCMProcessor::encodeEphemeris(const Ephemeris& ephemeris) {
    if (!ephemeris.valid) {
        return RTCMMessage();
    }

    if (ephemeris.satellite.system == GNSSSystem::GPS &&
        ephemeris.satellite.prn >= 1 && ephemeris.satellite.prn <= 32 &&
        ephemeris.sqrt_a > 0.0) {
        const double toe_seconds = ephemeris.toes != 0.0 ? ephemeris.toes : ephemeris.toe.tow;
        const double toc_seconds = ephemeris.toc.tow;
        const int64_t idot = static_cast<int64_t>(
            std::llround(ephemeris.idot / (kPow2Neg43 * kSemiCircleToRadians)));
        const int64_t af2 = static_cast<int64_t>(std::llround(ephemeris.af2 / kPow2Neg55));
        const int64_t af1 = static_cast<int64_t>(std::llround(ephemeris.af1 / kPow2Neg43));
        const int64_t af0 = static_cast<int64_t>(std::llround(ephemeris.af0 / kPow2Neg31));
        const int64_t crs = static_cast<int64_t>(std::llround(ephemeris.crs / kPow2Neg5));
        const int64_t delta_n = static_cast<int64_t>(
            std::llround(ephemeris.delta_n / (kPow2Neg43 * kSemiCircleToRadians)));
        const int64_t m0 = static_cast<int64_t>(
            std::llround(ephemeris.m0 / (kPow2Neg31 * kSemiCircleToRadians)));
        const int64_t cuc = static_cast<int64_t>(std::llround(ephemeris.cuc / kPow2Neg29));
        const uint64_t eccentricity = static_cast<uint64_t>(std::llround(ephemeris.e / kPow2Neg33));
        const int64_t cus = static_cast<int64_t>(std::llround(ephemeris.cus / kPow2Neg29));
        const uint64_t sqrt_a = static_cast<uint64_t>(std::llround(ephemeris.sqrt_a / kPow2Neg19));
        const int64_t cic = static_cast<int64_t>(std::llround(ephemeris.cic / kPow2Neg29));
        const int64_t omega0 = static_cast<int64_t>(
            std::llround(ephemeris.omega0 / (kPow2Neg31 * kSemiCircleToRadians)));
        const int64_t cis = static_cast<int64_t>(std::llround(ephemeris.cis / kPow2Neg29));
        const int64_t i0 = static_cast<int64_t>(
            std::llround(ephemeris.i0 / (kPow2Neg31 * kSemiCircleToRadians)));
        const int64_t crc = static_cast<int64_t>(std::llround(ephemeris.crc / kPow2Neg5));
        const int64_t omega = static_cast<int64_t>(
            std::llround(ephemeris.omega / (kPow2Neg31 * kSemiCircleToRadians)));
        const int64_t omega_dot = static_cast<int64_t>(
            std::llround(ephemeris.omega_dot / (kPow2Neg43 * kSemiCircleToRadians)));
        const int64_t tgd = static_cast<int64_t>(std::llround(ephemeris.tgd / kPow2Neg31));
        const uint64_t toe = static_cast<uint64_t>(std::llround(toe_seconds / 16.0));
        const uint64_t toc = static_cast<uint64_t>(std::llround(toc_seconds / 16.0));

        if (idot < -(1 << 13) || idot > ((1 << 13) - 1) ||
            af2 < -(1 << 7) || af2 > ((1 << 7) - 1) ||
            af1 < -(1 << 15) || af1 > ((1 << 15) - 1) ||
            af0 < -(1 << 21) || af0 > ((1 << 21) - 1) ||
            crs < -(1 << 15) || crs > ((1 << 15) - 1) ||
            delta_n < -(1 << 15) || delta_n > ((1 << 15) - 1) ||
            m0 < -(1LL << 31) || m0 > ((1LL << 31) - 1) ||
            cuc < -(1 << 15) || cuc > ((1 << 15) - 1) ||
            eccentricity > 0xFFFFFFFFULL ||
            cus < -(1 << 15) || cus > ((1 << 15) - 1) ||
            sqrt_a > 0xFFFFFFFFULL ||
            toe > 0xFFFFULL ||
            cic < -(1 << 15) || cic > ((1 << 15) - 1) ||
            omega0 < -(1LL << 31) || omega0 > ((1LL << 31) - 1) ||
            cis < -(1 << 15) || cis > ((1 << 15) - 1) ||
            i0 < -(1LL << 31) || i0 > ((1LL << 31) - 1) ||
            crc < -(1 << 15) || crc > ((1 << 15) - 1) ||
            omega < -(1LL << 31) || omega > ((1LL << 31) - 1) ||
            omega_dot < -(1 << 23) || omega_dot > ((1 << 23) - 1) ||
            tgd < -(1 << 7) || tgd > ((1 << 7) - 1) ||
            toc > 0xFFFFULL) {
            return RTCMMessage();
        }

        std::vector<uint8_t> payload(61, 0);
        int bit_pos = 0;
        writeUnsignedBits(payload, bit_pos, 12, 1019); bit_pos += 12;
        writeUnsignedBits(payload, bit_pos, 6, ephemeris.satellite.prn); bit_pos += 6;
        writeUnsignedBits(payload, bit_pos, 10, ephemeris.week % 1024U); bit_pos += 10;
        const uint8_t ura_index =
            (std::isfinite(ephemeris.sv_accuracy) && ephemeris.sv_accuracy > 0.0)
                ? uraIndexFromMeters(ephemeris.sv_accuracy)
                : ephemeris.ura;
        writeUnsignedBits(payload, bit_pos, 4, ura_index); bit_pos += 4;
        writeUnsignedBits(payload, bit_pos, 2, 0); bit_pos += 2;
        writeSignedBits(payload, bit_pos, 14, idot); bit_pos += 14;
        writeUnsignedBits(payload, bit_pos, 8, ephemeris.iode & 0xFFU); bit_pos += 8;
        writeUnsignedBits(payload, bit_pos, 16, toc); bit_pos += 16;
        writeSignedBits(payload, bit_pos, 8, af2); bit_pos += 8;
        writeSignedBits(payload, bit_pos, 16, af1); bit_pos += 16;
        writeSignedBits(payload, bit_pos, 22, af0); bit_pos += 22;
        writeUnsignedBits(payload, bit_pos, 10, ephemeris.iodc & 0x03FFU); bit_pos += 10;
        writeSignedBits(payload, bit_pos, 16, crs); bit_pos += 16;
        writeSignedBits(payload, bit_pos, 16, delta_n); bit_pos += 16;
        writeSignedBits(payload, bit_pos, 32, m0); bit_pos += 32;
        writeSignedBits(payload, bit_pos, 16, cuc); bit_pos += 16;
        writeUnsignedBits(payload, bit_pos, 32, eccentricity); bit_pos += 32;
        writeSignedBits(payload, bit_pos, 16, cus); bit_pos += 16;
        writeUnsignedBits(payload, bit_pos, 32, sqrt_a); bit_pos += 32;
        writeUnsignedBits(payload, bit_pos, 16, toe); bit_pos += 16;
        writeSignedBits(payload, bit_pos, 16, cic); bit_pos += 16;
        writeSignedBits(payload, bit_pos, 32, omega0); bit_pos += 32;
        writeSignedBits(payload, bit_pos, 16, cis); bit_pos += 16;
        writeSignedBits(payload, bit_pos, 32, i0); bit_pos += 32;
        writeSignedBits(payload, bit_pos, 16, crc); bit_pos += 16;
        writeSignedBits(payload, bit_pos, 32, omega); bit_pos += 32;
        writeSignedBits(payload, bit_pos, 24, omega_dot); bit_pos += 24;
        writeSignedBits(payload, bit_pos, 8, tgd); bit_pos += 8;
        writeUnsignedBits(payload, bit_pos, 6, ephemeris.health & 0x3FU); bit_pos += 6;
        writeUnsignedBits(payload, bit_pos, 1, 0); bit_pos += 1;
        writeUnsignedBits(payload, bit_pos, 1, 0); bit_pos += 1;

        RTCMMessage message;
        message.type = RTCMMessageType::RTCM_1019;
        message.length = static_cast<uint16_t>(payload.size());
        message.data = std::move(payload);
        message.valid = true;
        return message;
    }

    if (ephemeris.satellite.system == GNSSSystem::GLONASS &&
        ephemeris.satellite.prn >= 1 && ephemeris.satellite.prn <= 32) {
        const GNSSTime toe_gpst = ephemeris.toe;
        const GNSSTime tof_gpst =
            (ephemeris.tof.week != 0 || std::abs(ephemeris.tof.tow) > 0.0) ? ephemeris.tof : ephemeris.toe;
        const GNSSTime toe_utc = gpstToUtcApprox(toe_gpst);
        const GNSSTime tof_utc = gpstToUtcApprox(tof_gpst);
        const double toe_local = secondsOfDay(toe_utc.tow + 10800.0);
        const double tof_local = secondsOfDay(tof_utc.tow + 10800.0);
        const int half_minutes = static_cast<int>(std::llround(tof_local / 30.0));
        const int tk_h = (half_minutes / 120) % 24;
        const int tk_m = (half_minutes % 120) / 2;
        const int tk_s = half_minutes % 2;
        const int tb = static_cast<int>(std::llround(toe_local / 900.0)) % 96;
        const int fcn = ephemeris.glonass_frequency_channel + 7;
        const int64_t pos_x = static_cast<int64_t>(std::llround(ephemeris.glonass_position.x() / (kPow2Neg11 * 1e3)));
        const int64_t pos_y = static_cast<int64_t>(std::llround(ephemeris.glonass_position.y() / (kPow2Neg11 * 1e3)));
        const int64_t pos_z = static_cast<int64_t>(std::llround(ephemeris.glonass_position.z() / (kPow2Neg11 * 1e3)));
        const int64_t vel_x = static_cast<int64_t>(std::llround(ephemeris.glonass_velocity.x() / (kPow2Neg20 * 1e3)));
        const int64_t vel_y = static_cast<int64_t>(std::llround(ephemeris.glonass_velocity.y() / (kPow2Neg20 * 1e3)));
        const int64_t vel_z = static_cast<int64_t>(std::llround(ephemeris.glonass_velocity.z() / (kPow2Neg20 * 1e3)));
        const int64_t acc_x = static_cast<int64_t>(std::llround(ephemeris.glonass_acceleration.x() / (kPow2Neg30 * 1e3)));
        const int64_t acc_y = static_cast<int64_t>(std::llround(ephemeris.glonass_acceleration.y() / (kPow2Neg30 * 1e3)));
        const int64_t acc_z = static_cast<int64_t>(std::llround(ephemeris.glonass_acceleration.z() / (kPow2Neg30 * 1e3)));
        const int64_t gamn = static_cast<int64_t>(std::llround(ephemeris.glonass_gamn / kPow2Neg40));
        const int64_t taun = static_cast<int64_t>(std::llround(ephemeris.glonass_taun / kPow2Neg30));

        if (fcn < 0 || fcn > 31 ||
            pos_x < -((1 << 26) - 1) || pos_x > ((1 << 26) - 1) ||
            pos_y < -((1 << 26) - 1) || pos_y > ((1 << 26) - 1) ||
            pos_z < -((1 << 26) - 1) || pos_z > ((1 << 26) - 1) ||
            vel_x < -((1 << 23) - 1) || vel_x > ((1 << 23) - 1) ||
            vel_y < -((1 << 23) - 1) || vel_y > ((1 << 23) - 1) ||
            vel_z < -((1 << 23) - 1) || vel_z > ((1 << 23) - 1) ||
            acc_x < -((1 << 4) - 1) || acc_x > ((1 << 4) - 1) ||
            acc_y < -((1 << 4) - 1) || acc_y > ((1 << 4) - 1) ||
            acc_z < -((1 << 4) - 1) || acc_z > ((1 << 4) - 1) ||
            gamn < -((1 << 10) - 1) || gamn > ((1 << 10) - 1) ||
            taun < -((1 << 21) - 1) || taun > ((1 << 21) - 1)) {
            return RTCMMessage();
        }

        std::vector<uint8_t> payload(45, 0);
        int bit_pos = 0;
        writeUnsignedBits(payload, bit_pos, 12, 1020); bit_pos += 12;
        writeUnsignedBits(payload, bit_pos, 6, ephemeris.satellite.prn); bit_pos += 6;
        writeUnsignedBits(payload, bit_pos, 5, static_cast<uint64_t>(fcn)); bit_pos += 5;
        writeUnsignedBits(payload, bit_pos, 4, 0); bit_pos += 4;
        writeUnsignedBits(payload, bit_pos, 5, static_cast<uint64_t>(tk_h)); bit_pos += 5;
        writeUnsignedBits(payload, bit_pos, 6, static_cast<uint64_t>(tk_m)); bit_pos += 6;
        writeUnsignedBits(payload, bit_pos, 1, static_cast<uint64_t>(tk_s)); bit_pos += 1;
        writeUnsignedBits(payload, bit_pos, 1, ephemeris.health != 0 ? 1U : 0U); bit_pos += 1;
        writeUnsignedBits(payload, bit_pos, 1, 0); bit_pos += 1;
        writeUnsignedBits(payload, bit_pos, 7, static_cast<uint64_t>(tb)); bit_pos += 7;
        writeSignMagnitudeBits(payload, bit_pos, 24, vel_x); bit_pos += 24;
        writeSignMagnitudeBits(payload, bit_pos, 27, pos_x); bit_pos += 27;
        writeSignMagnitudeBits(payload, bit_pos, 5, acc_x); bit_pos += 5;
        writeSignMagnitudeBits(payload, bit_pos, 24, vel_y); bit_pos += 24;
        writeSignMagnitudeBits(payload, bit_pos, 27, pos_y); bit_pos += 27;
        writeSignMagnitudeBits(payload, bit_pos, 5, acc_y); bit_pos += 5;
        writeSignMagnitudeBits(payload, bit_pos, 24, vel_z); bit_pos += 24;
        writeSignMagnitudeBits(payload, bit_pos, 27, pos_z); bit_pos += 27;
        writeSignMagnitudeBits(payload, bit_pos, 5, acc_z); bit_pos += 5;
        writeUnsignedBits(payload, bit_pos, 1, 0); bit_pos += 1;
        writeSignMagnitudeBits(payload, bit_pos, 11, gamn); bit_pos += 11;
        writeUnsignedBits(payload, bit_pos, 3, 0); bit_pos += 3;
        writeSignMagnitudeBits(payload, bit_pos, 22, taun); bit_pos += 22;
        writeUnsignedBits(payload, bit_pos, 5, 0); bit_pos += 5;
        writeUnsignedBits(payload, bit_pos, 5, ephemeris.glonass_age & 0x1FU); bit_pos += 5;
        writeUnsignedBits(payload, bit_pos, 1, 0); bit_pos += 1;
        writeUnsignedBits(payload, bit_pos, 4, 0); bit_pos += 4;
        writeUnsignedBits(payload, bit_pos, 11, 0); bit_pos += 11;
        writeUnsignedBits(payload, bit_pos, 2, 0); bit_pos += 2;
        writeUnsignedBits(payload, bit_pos, 1, 0); bit_pos += 1;
        writeUnsignedBits(payload, bit_pos, 11, 0); bit_pos += 11;
        writeUnsignedBits(payload, bit_pos, 32, 0); bit_pos += 32;
        writeUnsignedBits(payload, bit_pos, 5, 0); bit_pos += 5;
        writeUnsignedBits(payload, bit_pos, 22, 0); bit_pos += 22;
        writeUnsignedBits(payload, bit_pos, 1, 0); bit_pos += 1;
        writeUnsignedBits(payload, bit_pos, 7, 0); bit_pos += 7;

        RTCMMessage message;
        message.type = RTCMMessageType::RTCM_1020;
        message.length = static_cast<uint16_t>(payload.size());
        message.data = std::move(payload);
        message.valid = true;
        return message;
    }

    return RTCMMessage();
}

} // namespace io
} // namespace libgnss
