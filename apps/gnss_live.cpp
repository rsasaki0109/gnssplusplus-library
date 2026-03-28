#include <Eigen/Dense>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include <libgnss++/algorithms/rtk.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/io/rinex.hpp>
#include <libgnss++/io/rtcm.hpp>
#include <libgnss++/io/rtcm_stream.hpp>
#include <libgnss++/io/solution_writer.hpp>
#include <libgnss++/io/ubx.hpp>
#include <libgnss++/models/troposphere.hpp>

#ifndef _WIN32
#include <cerrno>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace {

constexpr double kExactTimeToleranceSeconds = 1e-3;
constexpr double kMaxBaseInterpolationGapSeconds = 2.0;
constexpr double kDefaultBaseHoldSeconds = 0.5;

enum class GlonassARChoice {
    OFF,
    ON,
    AUTOCAL
};

struct LiveConfig {
    std::string rover_rtcm_path;
    std::string rover_ubx_path;
    std::string base_rtcm_path;
    std::string nav_rinex_path;
    std::string output_pos_path = "output/live_solution.pos";
    libgnss::io::SolutionWriter::Format output_format = libgnss::io::SolutionWriter::Format::POS;
    int max_epochs = -1;
    size_t rover_message_limit = 0;
    size_t base_message_limit = 0;
    bool quiet = false;
    bool verbose = false;
    bool enable_base_interpolation = true;
    double base_hold_seconds = kDefaultBaseHoldSeconds;
    bool base_position_override = false;
    Eigen::Vector3d base_position_ecef = Eigen::Vector3d::Zero();
    double ratio_threshold = 3.0;
    int min_satellites_for_ar = 5;
    double elevation_mask_deg = 15.0;
    bool enable_glonass = true;
    bool enable_beidou = true;
    GlonassARChoice glonass_ar = GlonassARChoice::OFF;
    int rover_ubx_baud = 115200;
};

struct SourceState {
    libgnss::io::RTCMReader reader;
    libgnss::io::RTCMStreamDecoder decoder;
    size_t messages_read = 0;
    size_t message_limit = 0;
    bool eof = false;
};

enum class RoverInputKind {
    RTCM,
    UBX
};

struct RoverUbxSource {
    libgnss::io::UBXStreamDecoder decoder;
    std::ifstream file;
#ifndef _WIN32
    int serial_fd = -1;
#endif
    size_t bytes_read = 0;
    bool serial = false;
    bool eof = false;
};

struct DecoderSummaryStats {
    size_t total_messages = 0;
    size_t valid_messages = 0;
    size_t decoder_errors = 0;
};

double timeDiffSeconds(const libgnss::GNSSTime& a, const libgnss::GNSSTime& b) {
    return a - b;
}

struct ObservationKey {
    libgnss::SatelliteId satellite;
    libgnss::SignalType signal;

    bool operator<(const ObservationKey& other) const {
        if (satellite < other.satellite) return true;
        if (other.satellite < satellite) return false;
        return signal < other.signal;
    }
};

std::map<ObservationKey, const libgnss::Observation*> indexObservations(
    const libgnss::ObservationData& epoch) {
    std::map<ObservationKey, const libgnss::Observation*> indexed;
    for (const auto& obs : epoch.observations) {
        indexed[{obs.satellite, obs.signal}] = &obs;
    }
    return indexed;
}

size_t countCommonObservationKeys(const libgnss::ObservationData& lhs,
                                  const libgnss::ObservationData& rhs) {
    const auto lhs_obs = indexObservations(lhs);
    const auto rhs_obs = indexObservations(rhs);
    size_t common = 0;
    for (const auto& [key, obs] : lhs_obs) {
        (void)obs;
        if (rhs_obs.find(key) != rhs_obs.end()) {
            ++common;
        }
    }
    return common;
}

double signalFrequencyHz(const libgnss::SatelliteId& satellite,
                         libgnss::SignalType signal,
                         const libgnss::GNSSTime& time,
                         const libgnss::NavigationData& nav) {
    const libgnss::Ephemeris* eph = nav.getEphemeris(satellite, time);
    switch (signal) {
        case libgnss::SignalType::GPS_L1CA:
        case libgnss::SignalType::QZS_L1CA:
            return libgnss::constants::GPS_L1_FREQ;
        case libgnss::SignalType::GPS_L2C:
        case libgnss::SignalType::QZS_L2C:
            return libgnss::constants::GPS_L2_FREQ;
        case libgnss::SignalType::GPS_L5:
        case libgnss::SignalType::QZS_L5:
            return libgnss::constants::GPS_L5_FREQ;
        case libgnss::SignalType::GLO_L1CA:
        case libgnss::SignalType::GLO_L1P:
            if (eph && eph->satellite.system == libgnss::GNSSSystem::GLONASS) {
                return libgnss::constants::GLO_L1_BASE_FREQ +
                    eph->glonass_frequency_channel * libgnss::constants::GLO_L1_STEP_FREQ;
            }
            return libgnss::constants::GLO_L1_BASE_FREQ;
        case libgnss::SignalType::GLO_L2CA:
        case libgnss::SignalType::GLO_L2P:
            if (eph && eph->satellite.system == libgnss::GNSSSystem::GLONASS) {
                return libgnss::constants::GLO_L2_BASE_FREQ +
                    eph->glonass_frequency_channel * libgnss::constants::GLO_L2_STEP_FREQ;
            }
            return libgnss::constants::GLO_L2_BASE_FREQ;
        case libgnss::SignalType::GAL_E1:
            return libgnss::constants::GAL_E1_FREQ;
        case libgnss::SignalType::GAL_E5A:
            return libgnss::constants::GAL_E5A_FREQ;
        case libgnss::SignalType::GAL_E5B:
            return libgnss::constants::GAL_E5B_FREQ;
        case libgnss::SignalType::GAL_E6:
            return libgnss::constants::GAL_E6_FREQ;
        case libgnss::SignalType::BDS_B1I:
            return libgnss::constants::BDS_B1I_FREQ;
        case libgnss::SignalType::BDS_B2I:
            return libgnss::constants::BDS_B2I_FREQ;
        case libgnss::SignalType::BDS_B3I:
            return libgnss::constants::BDS_B3I_FREQ;
        case libgnss::SignalType::BDS_B1C:
            return libgnss::constants::BDS_B1C_FREQ;
        case libgnss::SignalType::BDS_B2A:
            return libgnss::constants::BDS_B2A_FREQ;
        default:
            return 0.0;
    }
}

double signalWavelength(const libgnss::SatelliteId& satellite,
                        libgnss::SignalType signal,
                        const libgnss::GNSSTime& time,
                        const libgnss::NavigationData& nav) {
    const double frequency = signalFrequencyHz(satellite, signal, time, nav);
    if (frequency <= 0.0) {
        return 0.0;
    }
    return libgnss::constants::SPEED_OF_LIGHT / frequency;
}

bool calculateModeledBaseRange(const libgnss::SatelliteId& satellite,
                               const libgnss::GNSSTime& time,
                               double approx_pseudorange,
                               const libgnss::Vector3d& base_position,
                               const libgnss::NavigationData& nav,
                               double& modeled_range) {
    libgnss::Vector3d sat_pos;
    libgnss::Vector3d sat_vel;
    double sat_clk = 0.0;
    double sat_clk_drift = 0.0;

    const double travel_time = approx_pseudorange > 1.0
        ? approx_pseudorange / libgnss::constants::SPEED_OF_LIGHT
        : 0.075;
    libgnss::GNSSTime tx_time = time - travel_time;
    if (!nav.calculateSatelliteState(satellite, tx_time, sat_pos, sat_vel, sat_clk, sat_clk_drift)) {
        return false;
    }

    tx_time = tx_time - sat_clk;
    if (!nav.calculateSatelliteState(satellite, tx_time, sat_pos, sat_vel, sat_clk, sat_clk_drift)) {
        return false;
    }

    const auto geom = nav.calculateGeometry(base_position, sat_pos);
    if (geom.elevation <= 0.05) {
        return false;
    }

    modeled_range = libgnss::geodist(sat_pos, base_position) +
        libgnss::models::tropDelaySaastamoinen(base_position, geom.elevation);
    return std::isfinite(modeled_range);
}

bool interpolateBaseEpoch(const libgnss::ObservationData& before,
                          const libgnss::ObservationData& after,
                          const libgnss::GNSSTime& target_time,
                          const libgnss::Vector3d& base_position,
                          const libgnss::NavigationData& nav,
                          libgnss::ObservationData& interpolated_epoch) {
    const double total_dt = timeDiffSeconds(after.time, before.time);
    if (!std::isfinite(total_dt) || total_dt <= 1e-6 || total_dt > kMaxBaseInterpolationGapSeconds) {
        return false;
    }

    const double alpha = timeDiffSeconds(target_time, before.time) / total_dt;
    if (!std::isfinite(alpha) || alpha < -1e-6 || alpha > 1.0 + 1e-6) {
        return false;
    }

    interpolated_epoch = libgnss::ObservationData(target_time);
    interpolated_epoch.receiver_position =
        before.receiver_position.norm() > 0.0 ? before.receiver_position : after.receiver_position;
    interpolated_epoch.receiver_clock_bias =
        (1.0 - alpha) * before.receiver_clock_bias + alpha * after.receiver_clock_bias;

    const auto before_obs = indexObservations(before);
    const auto after_obs = indexObservations(after);
    libgnss::ObservationData fallback_epoch(target_time);
    fallback_epoch.receiver_position = interpolated_epoch.receiver_position;
    fallback_epoch.receiver_clock_bias = interpolated_epoch.receiver_clock_bias;

    for (const auto& [key, obs_before_ptr] : before_obs) {
        const auto after_it = after_obs.find(key);
        if (after_it == after_obs.end()) continue;

        const auto& obs_before = *obs_before_ptr;
        const auto& obs_after = *after_it->second;
        const double wavelength = signalWavelength(key.satellite, key.signal, target_time, nav);
        if (wavelength <= 0.0 || !obs_before.has_pseudorange || !obs_after.has_pseudorange) {
            continue;
        }

        const double approx_target_code =
            obs_before.pseudorange + alpha * (obs_after.pseudorange - obs_before.pseudorange);

        double modeled_before = 0.0;
        double modeled_after = 0.0;
        double modeled_target = 0.0;
        if (!calculateModeledBaseRange(key.satellite, before.time, obs_before.pseudorange,
                                       base_position, nav, modeled_before) ||
            !calculateModeledBaseRange(key.satellite, after.time, obs_after.pseudorange,
                                       base_position, nav, modeled_after) ||
            !calculateModeledBaseRange(key.satellite, target_time, approx_target_code,
                                       base_position, nav, modeled_target)) {
            continue;
        }

        libgnss::Observation obs(key.satellite, key.signal);
        obs.valid = obs_before.valid && obs_after.valid;
        obs.lli = alpha < 0.5 ? obs_before.lli : obs_after.lli;
        obs.code = alpha < 0.5 ? obs_before.code : obs_after.code;
        obs.signal_strength = std::max(obs_before.signal_strength, obs_after.signal_strength);
        obs.snr = (1.0 - alpha) * obs_before.snr + alpha * obs_after.snr;
        obs.loss_of_lock = obs_before.loss_of_lock || obs_after.loss_of_lock;
        obs.has_glonass_frequency_channel =
            obs_before.has_glonass_frequency_channel || obs_after.has_glonass_frequency_channel;
        obs.glonass_frequency_channel =
            obs_before.has_glonass_frequency_channel ? obs_before.glonass_frequency_channel
                                                     : obs_after.glonass_frequency_channel;

        const double code_residual_before = obs_before.pseudorange - modeled_before;
        const double code_residual_after = obs_after.pseudorange - modeled_after;
        obs.pseudorange = modeled_target +
            code_residual_before + alpha * (code_residual_after - code_residual_before);
        obs.has_pseudorange = std::isfinite(obs.pseudorange);

        if (obs_before.has_carrier_phase && obs_after.has_carrier_phase &&
            (obs_before.lli & 0x01) == 0 && (obs_after.lli & 0x01) == 0 && !obs.loss_of_lock) {
            const double phase_residual_before =
                obs_before.carrier_phase * wavelength - modeled_before;
            const double phase_residual_after =
                obs_after.carrier_phase * wavelength - modeled_after;
            const double phase_residual_target =
                phase_residual_before + alpha * (phase_residual_after - phase_residual_before);
            obs.carrier_phase = (modeled_target + phase_residual_target) / wavelength;
            obs.has_carrier_phase = std::isfinite(obs.carrier_phase);
        }

        if (obs_before.has_doppler && obs_after.has_doppler) {
            obs.doppler = obs_before.doppler + alpha * (obs_after.doppler - obs_before.doppler);
            obs.has_doppler = true;
        }

        if (obs.has_pseudorange || obs.has_carrier_phase || obs.has_doppler) {
            interpolated_epoch.addObservation(obs);
        }

        libgnss::Observation fallback_obs(key.satellite, key.signal);
        fallback_obs.valid = obs_before.valid && obs_after.valid;
        fallback_obs.lli = alpha < 0.5 ? obs_before.lli : obs_after.lli;
        fallback_obs.code = alpha < 0.5 ? obs_before.code : obs_after.code;
        fallback_obs.signal_strength = std::max(obs_before.signal_strength, obs_after.signal_strength);
        fallback_obs.snr = (1.0 - alpha) * obs_before.snr + alpha * obs_after.snr;
        fallback_obs.loss_of_lock = obs_before.loss_of_lock || obs_after.loss_of_lock;
        fallback_obs.has_glonass_frequency_channel =
            obs_before.has_glonass_frequency_channel || obs_after.has_glonass_frequency_channel;
        fallback_obs.glonass_frequency_channel =
            obs_before.has_glonass_frequency_channel ? obs_before.glonass_frequency_channel
                                                     : obs_after.glonass_frequency_channel;

        if (obs_before.has_pseudorange && obs_after.has_pseudorange) {
            fallback_obs.pseudorange =
                obs_before.pseudorange + alpha * (obs_after.pseudorange - obs_before.pseudorange);
            fallback_obs.has_pseudorange = std::isfinite(fallback_obs.pseudorange);
        }
        if (obs_before.has_carrier_phase && obs_after.has_carrier_phase &&
            (obs_before.lli & 0x01) == 0 && (obs_after.lli & 0x01) == 0 && !fallback_obs.loss_of_lock) {
            fallback_obs.carrier_phase =
                obs_before.carrier_phase + alpha * (obs_after.carrier_phase - obs_before.carrier_phase);
            fallback_obs.has_carrier_phase = std::isfinite(fallback_obs.carrier_phase);
        }
        if (obs_before.has_doppler && obs_after.has_doppler) {
            fallback_obs.doppler = obs_before.doppler + alpha * (obs_after.doppler - obs_before.doppler);
            fallback_obs.has_doppler = true;
        }
        if (fallback_obs.has_pseudorange || fallback_obs.has_carrier_phase || fallback_obs.has_doppler) {
            fallback_epoch.addObservation(fallback_obs);
        }
    }

    if (interpolated_epoch.observations.empty()) {
        interpolated_epoch = std::move(fallback_epoch);
    }
    return !interpolated_epoch.observations.empty();
}

bool interpolateBaseEpochLinear(const libgnss::ObservationData& before,
                                const libgnss::ObservationData& after,
                                const libgnss::GNSSTime& target_time,
                                libgnss::ObservationData& interpolated_epoch) {
    const double total_dt = timeDiffSeconds(after.time, before.time);
    if (!std::isfinite(total_dt) || total_dt <= 1e-6 || total_dt > kMaxBaseInterpolationGapSeconds) {
        return false;
    }

    const double alpha = timeDiffSeconds(target_time, before.time) / total_dt;
    if (!std::isfinite(alpha) || alpha < -1e-6 || alpha > 1.0 + 1e-6) {
        return false;
    }

    interpolated_epoch = libgnss::ObservationData(target_time);
    interpolated_epoch.receiver_position =
        before.receiver_position.norm() > 0.0 ? before.receiver_position : after.receiver_position;
    interpolated_epoch.receiver_clock_bias =
        (1.0 - alpha) * before.receiver_clock_bias + alpha * after.receiver_clock_bias;

    const auto before_obs = indexObservations(before);
    const auto after_obs = indexObservations(after);
    for (const auto& [key, obs_before_ptr] : before_obs) {
        const auto after_it = after_obs.find(key);
        if (after_it == after_obs.end()) continue;

        const auto& obs_before = *obs_before_ptr;
        const auto& obs_after = *after_it->second;

        libgnss::Observation obs(key.satellite, key.signal);
        obs.valid = obs_before.valid && obs_after.valid;
        obs.lli = alpha < 0.5 ? obs_before.lli : obs_after.lli;
        obs.code = alpha < 0.5 ? obs_before.code : obs_after.code;
        obs.signal_strength = std::max(obs_before.signal_strength, obs_after.signal_strength);
        obs.snr = (1.0 - alpha) * obs_before.snr + alpha * obs_after.snr;
        obs.loss_of_lock = obs_before.loss_of_lock || obs_after.loss_of_lock;
        obs.has_glonass_frequency_channel =
            obs_before.has_glonass_frequency_channel || obs_after.has_glonass_frequency_channel;
        obs.glonass_frequency_channel =
            obs_before.has_glonass_frequency_channel ? obs_before.glonass_frequency_channel
                                                     : obs_after.glonass_frequency_channel;

        if (obs_before.has_pseudorange && obs_after.has_pseudorange) {
            obs.pseudorange = obs_before.pseudorange + alpha * (obs_after.pseudorange - obs_before.pseudorange);
            obs.has_pseudorange = std::isfinite(obs.pseudorange);
        }
        if (obs_before.has_carrier_phase && obs_after.has_carrier_phase &&
            (obs_before.lli & 0x01) == 0 && (obs_after.lli & 0x01) == 0 && !obs.loss_of_lock) {
            obs.carrier_phase =
                obs_before.carrier_phase + alpha * (obs_after.carrier_phase - obs_before.carrier_phase);
            obs.has_carrier_phase = std::isfinite(obs.carrier_phase);
        }
        if (obs_before.has_doppler && obs_after.has_doppler) {
            obs.doppler = obs_before.doppler + alpha * (obs_after.doppler - obs_before.doppler);
            obs.has_doppler = true;
        }
        if (obs.has_pseudorange || obs.has_carrier_phase || obs.has_doppler) {
            interpolated_epoch.addObservation(obs);
        }
    }
    return !interpolated_epoch.observations.empty();
}

bool holdBaseEpoch(const libgnss::ObservationData& source,
                   const libgnss::GNSSTime& target_time,
                   double max_hold_seconds,
                   libgnss::ObservationData& held_epoch) {
    const double dt = timeDiffSeconds(target_time, source.time);
    if (!std::isfinite(dt) || dt < -1e-6 || dt > max_hold_seconds) {
        return false;
    }
    held_epoch = source;
    held_epoch.time = target_time;
    return !held_epoch.observations.empty();
}

int inferObservationWeekContext(const libgnss::NavigationData& nav_data) {
    for (const auto& [satellite, ephemerides] : nav_data.ephemeris_data) {
        (void)satellite;
        for (const auto& eph : ephemerides) {
            if (eph.week > 0) {
                return eph.week;
            }
            if (eph.toe.week > 0) {
                return eph.toe.week;
            }
            if (eph.toc.week > 0) {
                return eph.toc.week;
            }
        }
    }
    return 0;
}

uint32_t crc24q(const uint8_t* data, size_t length) {
    static const uint32_t table[256] = {
        0x000000, 0x864CFB, 0x8AD50D, 0x0C99F6, 0x93E6E1, 0x15AA1A, 0x1933EC, 0x9F7F17,
        0xA18139, 0x27CDC2, 0x2B5434, 0xAD18CF, 0x3267D8, 0xB42B23, 0xB8B2D5, 0x3EFE2E,
        0xC54E89, 0x430272, 0x4F9B84, 0xC9D77F, 0x56A868, 0xD0E493, 0xDC7D65, 0x5A319E,
        0x64CFB0, 0xE2834B, 0xEE1ABD, 0x685646, 0xF72951, 0x7165AA, 0x7DFC5C, 0xFBB0A7,
        0x0CD1E9, 0x8A9D12, 0x8604E4, 0x00481F, 0x9F3708, 0x197BF3, 0x15E205, 0x93AEFE,
        0xAD50D0, 0x2B1C2B, 0x2785DD, 0xA1C926, 0x3EB631, 0xB8FACA, 0xB4633C, 0x322FC7,
        0xC99F60, 0x4FD39B, 0x434A6D, 0xC50696, 0x5A7981, 0xDC357A, 0xD0AC8C, 0x56E077,
        0x681E59, 0xEE52A2, 0xE2CB54, 0x6487AF, 0xFBF8B8, 0x7DB443, 0x712DB5, 0xF7614E,
        0x19A3D2, 0x9FEF29, 0x9376DF, 0x153A24, 0x8A4533, 0x0C09C8, 0x00903E, 0x86DCC5,
        0xB822EB, 0x3E6E10, 0x32F7E6, 0xB4BB1D, 0x2BC40A, 0xAD88F1, 0xA11107, 0x275DFC,
        0xDCED5B, 0x5AA1A0, 0x563856, 0xD074AD, 0x4F0BBA, 0xC94741, 0xC5DEB7, 0x43924C,
        0x7D6C62, 0xFB2099, 0xF7B96F, 0x71F594, 0xEE8A83, 0x68C678, 0x645F8E, 0xE21375,
        0x15723B, 0x933EC0, 0x9FA736, 0x19EBCD, 0x8694DA, 0x00D821, 0x0C41D7, 0x8A0D2C,
        0xB4F302, 0x32BFF9, 0x3E260F, 0xB86AF4, 0x2715E3, 0xA15918, 0xADC0EE, 0x2B8C15,
        0xD03CB2, 0x567049, 0x5AE9BF, 0xDCA544, 0x43DA53, 0xC596A8, 0xC90F5E, 0x4F43A5,
        0x71BD8B, 0xF7F170, 0xFB6886, 0x7D247D, 0xE25B6A, 0x641791, 0x688E67, 0xEEC29C,
        0x3347A4, 0xB50B5F, 0xB992A9, 0x3FDE52, 0xA0A145, 0x26EDBE, 0x2A7448, 0xAC38B3,
        0x92C69D, 0x148A66, 0x181390, 0x9E5F6B, 0x01207C, 0x876C87, 0x8BF571, 0x0DB98A,
        0xF6092D, 0x7045D6, 0x7CDC20, 0xFA90DB, 0x65EFCC, 0xE3A337, 0xEF3AC1, 0x69763A,
        0x578814, 0xD1C4EF, 0xDD5D19, 0x5B11E2, 0xC46EF5, 0x42220E, 0x4EBBF8, 0xC8F703,
        0x3F964D, 0xB9DAB6, 0xB54340, 0x330FBB, 0xAC70AC, 0x2A3C57, 0x26A5A1, 0xA0E95A,
        0x9E1774, 0x185B8F, 0x14C279, 0x928E82, 0x0DF195, 0x8BBD6E, 0x872498, 0x016863,
        0xFAD8C4, 0x7C943F, 0x700DC9, 0xF64132, 0x693E25, 0xEF72DE, 0xE3EB28, 0x65A7D3,
        0x5B59FD, 0xDD1506, 0xD18CF0, 0x57C00B, 0xC8BF1C, 0x4EF3E7, 0x426A11, 0xC426EA,
        0x2AE476, 0xACA88D, 0xA0317B, 0x267D80, 0xB90297, 0x3F4E6C, 0x33D79A, 0xB59B61,
        0x8B654F, 0x0D29B4, 0x01B042, 0x87FCB9, 0x1883AE, 0x9ECF55, 0x9256A3, 0x141A58,
        0xEFAAFF, 0x69E604, 0x657FF2, 0xE33309, 0x7C4C1E, 0xFA00E5, 0xF69913, 0x70D5E8,
        0x4E2BC6, 0xC8673D, 0xC4FECB, 0x42B230, 0xDDCD27, 0x5B81DC, 0x57182A, 0xD154D1,
        0x26359F, 0xA07964, 0xACE092, 0x2AAC69, 0xB5D37E, 0x339F85, 0x3F0673, 0xB94A88,
        0x87B4A6, 0x01F85D, 0x0D61AB, 0x8B2D50, 0x145247, 0x921EBC, 0x9E874A, 0x18CBB1,
        0xE37B16, 0x6537ED, 0x69AE1B, 0xEFE2E0, 0x709DF7, 0xF6D10C, 0xFA48FA, 0x7C0401,
        0x42FA2F, 0xC4B6D4, 0xC82F22, 0x4E63D9, 0xD11CCE, 0x575035, 0x5BC9C3, 0xDD8538
    };

    uint32_t crc = 0;
    for (size_t i = 0; i < length; ++i) {
        const uint8_t index = static_cast<uint8_t>(((crc >> 16) ^ data[i]) & 0xFFU);
        crc = (crc << 8) ^ table[index];
    }
    return crc & 0x00FFFFFFU;
}

std::vector<uint8_t> buildRtcmFrame(const libgnss::io::RTCMMessage& message) {
    std::vector<uint8_t> frame;
    frame.reserve(3 + message.data.size() + 3);
    frame.push_back(0xD3);
    frame.push_back(static_cast<uint8_t>((message.data.size() >> 8) & 0x03U));
    frame.push_back(static_cast<uint8_t>(message.data.size() & 0xFFU));
    frame.insert(frame.end(), message.data.begin(), message.data.end());
    const uint32_t crc = crc24q(frame.data(), frame.size());
    frame.push_back(static_cast<uint8_t>((crc >> 16) & 0xFFU));
    frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFFU));
    frame.push_back(static_cast<uint8_t>(crc & 0xFFU));
    return frame;
}

void mergeNavigationData(libgnss::NavigationData& dst, const libgnss::NavigationData& src) {
    for (const auto& [satellite, ephemerides] : src.ephemeris_data) {
        (void)satellite;
        for (const auto& eph : ephemerides) {
            dst.addEphemeris(eph);
        }
    }
    if (src.ionosphere_model.valid) {
        dst.ionosphere_model = src.ionosphere_model;
    }
}

std::string outputFormatString(libgnss::io::SolutionWriter::Format format) {
    switch (format) {
        case libgnss::io::SolutionWriter::Format::POS: return "pos";
        case libgnss::io::SolutionWriter::Format::LLH: return "llh";
        case libgnss::io::SolutionWriter::Format::XYZ: return "xyz";
    }
    return "pos";
}

void printUsage(const char* argv0) {
    std::cout
        << "Usage: " << argv0 << " (--rover-rtcm <path|ntrip://...|tcp://host:port> | --rover-ubx <file|serial://...|/dev/tty...>) --base-rtcm <path|ntrip://...|serial://...|tcp://host:port> [options]\n"
        << "Options:\n"
        << "  --rover-ubx <path>         Rover UBX stream from file or serial device\n"
        << "  --rover-ubx-baud <baud>    Rover serial baud rate for UBX device input (default: 115200)\n"
        << "  --nav-rinex <file>         Supplemental broadcast navigation RINEX\n"
        << "  --out <file>               Output solution file (default: output/live_solution.pos)\n"
        << "  --format <pos|llh|xyz>     Output format (default: pos)\n"
        << "  --max-epochs <n>           Stop after n solved epochs\n"
        << "  --rover-message-limit <n>  Stop rover ingest after n RTCM messages (0 = all)\n"
        << "  --base-message-limit <n>   Stop base ingest after n RTCM messages (0 = all)\n"
        << "  --no-base-interp           Require exact rover/base epoch alignment\n"
        << "  --base-hold-seconds <v>    Reuse the latest base epoch for up to v seconds (default: 0.5)\n"
        << "  --base-ecef <x> <y> <z>    Override base ECEF position\n"
        << "  --ratio <value>            Ambiguity ratio threshold (default: 3.0)\n"
        << "  --min-ar-sats <n>          Minimum satellites for AR (default: 5)\n"
        << "  --elevation-mask-deg <v>   Elevation mask in degrees (default: 15)\n"
        << "  --no-glonass               Disable GLONASS carrier processing\n"
        << "  --no-beidou                Disable BeiDou carrier processing\n"
        << "  --glonass-ar <off|on|autocal>\n"
        << "  --quiet                    Suppress per-epoch prints\n"
        << "  --verbose                  Print per-epoch solve details\n"
        << "  -h, --help                 Show this help\n";
}

[[noreturn]] void argumentError(const std::string& message, const char* argv0) {
    std::cerr << "Argument error: " << message << "\n\n";
    printUsage(argv0);
    std::exit(1);
}

GlonassARChoice parseGlonassARChoice(const std::string& value, const char* argv0) {
    if (value == "off") return GlonassARChoice::OFF;
    if (value == "on") return GlonassARChoice::ON;
    if (value == "autocal") return GlonassARChoice::AUTOCAL;
    argumentError("unsupported --glonass-ar value: " + value, argv0);
}

libgnss::io::SolutionWriter::Format parseOutputFormat(const std::string& value, const char* argv0) {
    if (value == "pos") return libgnss::io::SolutionWriter::Format::POS;
    if (value == "llh") return libgnss::io::SolutionWriter::Format::LLH;
    if (value == "xyz") return libgnss::io::SolutionWriter::Format::XYZ;
    argumentError("unsupported --format value: " + value, argv0);
}

LiveConfig parseArguments(int argc, char** argv) {
    LiveConfig config;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            std::exit(0);
        } else if (arg == "--rover-rtcm" && i + 1 < argc) {
            config.rover_rtcm_path = argv[++i];
        } else if (arg == "--rover-ubx" && i + 1 < argc) {
            config.rover_ubx_path = argv[++i];
        } else if (arg == "--rover-ubx-baud" && i + 1 < argc) {
            config.rover_ubx_baud = std::stoi(argv[++i]);
        } else if (arg == "--base-rtcm" && i + 1 < argc) {
            config.base_rtcm_path = argv[++i];
        } else if (arg == "--nav-rinex" && i + 1 < argc) {
            config.nav_rinex_path = argv[++i];
        } else if (arg == "--out" && i + 1 < argc) {
            config.output_pos_path = argv[++i];
        } else if (arg == "--format" && i + 1 < argc) {
            config.output_format = parseOutputFormat(argv[++i], argv[0]);
        } else if (arg == "--max-epochs" && i + 1 < argc) {
            config.max_epochs = std::stoi(argv[++i]);
        } else if (arg == "--rover-message-limit" && i + 1 < argc) {
            config.rover_message_limit = static_cast<size_t>(std::stoull(argv[++i]));
        } else if (arg == "--base-message-limit" && i + 1 < argc) {
            config.base_message_limit = static_cast<size_t>(std::stoull(argv[++i]));
        } else if (arg == "--no-base-interp") {
            config.enable_base_interpolation = false;
        } else if (arg == "--base-hold-seconds" && i + 1 < argc) {
            config.base_hold_seconds = std::stod(argv[++i]);
        } else if (arg == "--base-ecef" && i + 3 < argc) {
            config.base_position_ecef =
                Eigen::Vector3d(std::stod(argv[++i]), std::stod(argv[++i]), std::stod(argv[++i]));
            config.base_position_override = true;
        } else if (arg == "--ratio" && i + 1 < argc) {
            config.ratio_threshold = std::stod(argv[++i]);
        } else if (arg == "--min-ar-sats" && i + 1 < argc) {
            config.min_satellites_for_ar = std::stoi(argv[++i]);
        } else if (arg == "--elevation-mask-deg" && i + 1 < argc) {
            config.elevation_mask_deg = std::stod(argv[++i]);
        } else if (arg == "--no-glonass") {
            config.enable_glonass = false;
        } else if (arg == "--no-beidou") {
            config.enable_beidou = false;
        } else if (arg == "--glonass-ar" && i + 1 < argc) {
            config.glonass_ar = parseGlonassARChoice(argv[++i], argv[0]);
        } else if (arg == "--quiet") {
            config.quiet = true;
        } else if (arg == "--verbose") {
            config.verbose = true;
        } else {
            argumentError("unknown or incomplete argument: " + arg, argv[0]);
        }
    }

    const bool has_rover_rtcm = !config.rover_rtcm_path.empty();
    const bool has_rover_ubx = !config.rover_ubx_path.empty();
    if (has_rover_rtcm == has_rover_ubx) {
        argumentError("choose exactly one of --rover-rtcm or --rover-ubx", argv[0]);
    }
    if (config.base_rtcm_path.empty()) {
        argumentError("--base-rtcm is required", argv[0]);
    }
    if (config.max_epochs == 0) {
        argumentError("--max-epochs must be != 0", argv[0]);
    }
    if (config.min_satellites_for_ar < 4) {
        argumentError("--min-ar-sats must be >= 4", argv[0]);
    }
    if (config.base_hold_seconds < 0.0) {
        argumentError("--base-hold-seconds must be >= 0", argv[0]);
    }
    if (config.rover_ubx_baud <= 0) {
        argumentError("--rover-ubx-baud must be > 0", argv[0]);
    }
    return config;
}

bool loadNavigationRinex(const std::string& path, libgnss::NavigationData& nav_data) {
    if (path.empty()) {
        return true;
    }
    libgnss::io::RINEXReader reader;
    if (!reader.open(path)) {
        return false;
    }
    libgnss::NavigationData loaded;
    const bool ok = reader.readNavigationData(loaded);
    reader.close();
    if (!ok) {
        return false;
    }
    mergeNavigationData(nav_data, loaded);
    return true;
}

bool openSource(const std::string& path, size_t message_limit, SourceState& source) {
    source.messages_read = 0;
    source.message_limit = message_limit;
    source.eof = false;
    source.decoder.clear();
    return source.reader.open(path);
}

std::string resolveSerialPath(const std::string& path) {
    constexpr const char* kPrefix = "serial://";
    if (path.rfind(kPrefix, 0) == 0) {
        std::string value = path.substr(std::char_traits<char>::length(kPrefix));
        if (!value.empty() && value.front() != '/') {
            return value;
        }
        if (value.size() >= 2 && value[0] == '/' && value[1] != '/') {
            return value;
        }
        while (value.size() > 1 && value[0] == '/' && value[1] == '/') {
            value.erase(value.begin());
        }
        return value;
    }
    return path;
}

#ifndef _WIN32
speed_t baudRateConstant(int baud) {
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        default:
            throw std::invalid_argument("unsupported serial baud rate");
    }
}

void configureSerialPort(int fd, int baud) {
    termios tio{};
    if (tcgetattr(fd, &tio) != 0) {
        throw std::runtime_error("failed to read serial port attributes");
    }
    cfmakeraw(&tio);
    const speed_t speed = baudRateConstant(baud);
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cc[VMIN] = 1;
    tio.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        throw std::runtime_error("failed to configure serial port");
    }
}
#endif

bool openRoverUbxSource(const std::string& path, int baud, RoverUbxSource& source) {
    source.decoder.clear();
    source.bytes_read = 0;
    source.eof = false;
    source.serial = false;
    source.file = std::ifstream();
#ifndef _WIN32
    if (source.serial_fd >= 0) {
        ::close(source.serial_fd);
        source.serial_fd = -1;
    }
#endif

    const std::string resolved_path = resolveSerialPath(path);
    std::error_code ec;
    const auto status = std::filesystem::status(resolved_path, ec);
    const bool is_regular_file = !ec && std::filesystem::is_regular_file(status);
    if (is_regular_file) {
        source.file.open(resolved_path, std::ios::binary);
        return source.file.is_open();
    }

#ifndef _WIN32
    const int fd = ::open(resolved_path.c_str(), O_RDONLY | O_NOCTTY);
    if (fd < 0) {
        return false;
    }
    try {
        configureSerialPort(fd, baud);
    } catch (...) {
        ::close(fd);
        throw;
    }
    source.serial = true;
    source.serial_fd = fd;
    return true;
#else
    (void)baud;
    return false;
#endif
}

bool readNextObservation(SourceState& source,
                         libgnss::ObservationData& obs,
                         std::vector<libgnss::io::RTCMStreamDecoder::Event>& events) {
    events.clear();
    libgnss::io::RTCMMessage raw_message;
    while (source.message_limit == 0 || source.messages_read < source.message_limit) {
        if (!source.reader.readMessage(raw_message)) {
            source.eof = true;
            return false;
        }
        ++source.messages_read;
        const auto frame = buildRtcmFrame(raw_message);
        std::vector<libgnss::io::RTCMStreamDecoder::Event> frame_events;
        source.decoder.pushFrame(frame.data(), frame.size(), frame_events);
        events.insert(events.end(), frame_events.begin(), frame_events.end());
        for (const auto& event : frame_events) {
            if (event.has_observation) {
                obs = event.observation;
                return true;
            }
        }
    }
    source.eof = true;
    return false;
}

bool readNextUbxObservation(RoverUbxSource& source,
                            libgnss::ObservationData& obs,
                            std::vector<libgnss::io::UBXStreamDecoder::Event>& events) {
    events.clear();
    std::vector<uint8_t> chunk(4096);

    while (true) {
        size_t bytes_read = 0;
        if (source.serial) {
#ifndef _WIN32
            const ssize_t count = ::read(source.serial_fd, chunk.data(), chunk.size());
            if (count < 0) {
                if (errno == EINTR) {
                    continue;
                }
                source.eof = true;
                return false;
            }
            if (count == 0) {
                source.eof = true;
                return false;
            }
            bytes_read = static_cast<size_t>(count);
#else
            source.eof = true;
            return false;
#endif
        } else {
            source.file.read(reinterpret_cast<char*>(chunk.data()), static_cast<std::streamsize>(chunk.size()));
            bytes_read = static_cast<size_t>(source.file.gcount());
            if (bytes_read == 0) {
                source.eof = true;
                return false;
            }
        }

        source.bytes_read += bytes_read;
        std::vector<libgnss::io::UBXStreamDecoder::Event> chunk_events;
        source.decoder.pushBytes(chunk.data(), bytes_read, chunk_events);
        events.insert(events.end(), chunk_events.begin(), chunk_events.end());
        for (const auto& event : chunk_events) {
            if (event.has_observation) {
                obs = event.observation;
                return true;
            }
        }
    }
}

DecoderSummaryStats summarizeRtcmSource(const SourceState& source) {
    const auto stats = source.reader.getStats();
    DecoderSummaryStats summary;
    summary.total_messages = std::max(stats.total_messages, source.messages_read);
    summary.valid_messages = std::max(stats.valid_messages, source.messages_read);
    summary.decoder_errors = stats.crc_errors;
    return summary;
}

DecoderSummaryStats summarizeUbxSource(const RoverUbxSource& source) {
    const auto stats = source.decoder.getStats();
    DecoderSummaryStats summary;
    summary.total_messages = stats.total_messages;
    summary.valid_messages = stats.valid_messages;
    summary.decoder_errors = stats.checksum_errors;
    return summary;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const auto wall_time_begin = std::chrono::steady_clock::now();
        const LiveConfig config = parseArguments(argc, argv);
        const RoverInputKind rover_input_kind =
            config.rover_ubx_path.empty() ? RoverInputKind::RTCM : RoverInputKind::UBX;

        SourceState rover_rtcm_source;
        RoverUbxSource rover_ubx_source;
        SourceState base_source;
        if (rover_input_kind == RoverInputKind::RTCM) {
            if (!openSource(config.rover_rtcm_path, config.rover_message_limit, rover_rtcm_source)) {
                std::cerr << "Error: failed to open rover RTCM source: "
                          << config.rover_rtcm_path << "\n";
                return 1;
            }
        } else {
            if (!openRoverUbxSource(config.rover_ubx_path, config.rover_ubx_baud, rover_ubx_source)) {
                std::cerr << "Error: failed to open rover UBX source: "
                          << config.rover_ubx_path << "\n";
                return 1;
            }
        }
        if (!openSource(config.base_rtcm_path, config.base_message_limit, base_source)) {
            std::cerr << "Error: failed to open base RTCM source: "
                      << config.base_rtcm_path << "\n";
            return 1;
        }

        libgnss::NavigationData nav_data;
        if (!loadNavigationRinex(config.nav_rinex_path, nav_data)) {
            std::cerr << "Error: failed to read navigation RINEX: "
                      << config.nav_rinex_path << "\n";
            return 1;
        }

        libgnss::RTKProcessor rtk;
        libgnss::RTKProcessor::RTKConfig rtk_config;
        rtk_config.position_mode = libgnss::RTKProcessor::RTKConfig::PositionMode::KINEMATIC;
        rtk_config.ratio_threshold = config.ratio_threshold;
        rtk_config.ambiguity_ratio_threshold = config.ratio_threshold;
        rtk_config.min_satellites_for_ar = config.min_satellites_for_ar;
        rtk_config.elevation_mask = config.elevation_mask_deg * M_PI / 180.0;
        rtk_config.enable_glonass = config.enable_glonass;
        rtk_config.enable_beidou = config.enable_beidou;
        rtk_config.glonass_ar_mode =
            config.glonass_ar == GlonassARChoice::AUTOCAL
                ? libgnss::RTKProcessor::RTKConfig::GlonassARMode::AUTOCAL
                : (config.glonass_ar == GlonassARChoice::ON
                    ? libgnss::RTKProcessor::RTKConfig::GlonassARMode::ON
                    : libgnss::RTKProcessor::RTKConfig::GlonassARMode::OFF);
        rtk.setRTKConfig(rtk_config);

        Eigen::Vector3d base_position = config.base_position_override
            ? config.base_position_ecef
            : Eigen::Vector3d::Zero();
        bool have_base_position = config.base_position_override;
        if (have_base_position) {
            rtk.setBasePosition(base_position);
        }

        libgnss::io::SolutionWriter writer;
        if (!writer.open(config.output_pos_path, config.output_format)) {
            std::cerr << "Error: failed to open output file\n";
            return 1;
        }

        libgnss::ObservationData rover_obs;
        libgnss::ObservationData base_obs;
        std::vector<libgnss::io::RTCMStreamDecoder::Event> rover_events;
        std::vector<libgnss::io::RTCMStreamDecoder::Event> base_events;
        std::vector<libgnss::io::UBXStreamDecoder::Event> rover_ubx_events;

        bool rover_ok = rover_input_kind == RoverInputKind::RTCM
            ? readNextObservation(rover_rtcm_source, rover_obs, rover_events)
            : readNextUbxObservation(rover_ubx_source, rover_obs, rover_ubx_events);
        bool base_ok = readNextObservation(base_source, base_obs, base_events);
        const bool initial_rover_observation_ok = rover_ok;
        const bool initial_base_observation_ok = base_ok;

        size_t aligned_epochs = 0;
        size_t exact_base_epochs = 0;
        size_t interpolated_base_epochs = 0;
        size_t held_base_epochs = 0;
        size_t skipped_rover_epochs = 0;
        size_t written_solutions = 0;
        size_t fixed_solutions = 0;
        size_t nav_event_updates = 0;
        size_t station_position_updates = 0;
        Eigen::Vector3d rover_seed = Eigen::Vector3d::Zero();
        int solved_epochs = 0;
        libgnss::ObservationData previous_base_obs;
        bool has_previous_base = false;
        bool have_first_aligned_time = false;
        libgnss::GNSSTime first_aligned_time;
        libgnss::GNSSTime last_aligned_time;
        std::string termination_reason = "running";

        auto applyEvents = [&](const std::vector<libgnss::io::RTCMStreamDecoder::Event>& events) {
            for (const auto& event : events) {
                if (event.has_navigation) {
                    mergeNavigationData(nav_data, event.navigation);
                    ++nav_event_updates;
                }
                if (event.has_station_position && !config.base_position_override) {
                    base_position = event.station_position;
                    have_base_position = true;
                    rtk.setBasePosition(base_position);
                    ++station_position_updates;
                }
            }
        };
        auto normalizeObservationWeek = [&](libgnss::ObservationData& obs) {
            if (obs.time.week != 0) {
                return;
            }
            const int week = inferObservationWeekContext(nav_data);
            if (week > 0) {
                obs.time.week = week;
            }
        };

        if (rover_ok && rover_input_kind == RoverInputKind::RTCM) {
            applyEvents(rover_events);
        }
        if (base_ok) {
            applyEvents(base_events);
        }
        if (rover_ok) {
            normalizeObservationWeek(rover_obs);
        }
        if (base_ok) {
            normalizeObservationWeek(base_obs);
        }

        while (rover_ok && base_ok && (config.max_epochs < 0 || solved_epochs < config.max_epochs)) {
            if (rover_obs.receiver_position.norm() == 0.0) {
                rover_obs.receiver_position = rover_seed.norm() > 0.0
                    ? rover_seed
                    : (have_base_position ? base_position + Eigen::Vector3d(3000.0, 0.0, 0.0)
                                          : Eigen::Vector3d(3000.0, 0.0, 0.0));
            }

            libgnss::ObservationData lower_base_obs = previous_base_obs;
            bool have_lower_base = has_previous_base;

            while (base_ok &&
                   timeDiffSeconds(base_obs.time, rover_obs.time) < -kExactTimeToleranceSeconds) {
                lower_base_obs = base_obs;
                have_lower_base = true;
                previous_base_obs = base_obs;
                has_previous_base = true;
                base_ok = readNextObservation(base_source, base_obs, base_events);
                if (base_ok) {
                    applyEvents(base_events);
                    normalizeObservationWeek(base_obs);
                }
            }

            libgnss::ObservationData aligned_base_obs;
            const double exact_dt = base_ok ? std::abs(timeDiffSeconds(base_obs.time, rover_obs.time))
                                            : std::numeric_limits<double>::infinity();
            bool have_aligned_base = false;
            const double lower_dt = have_lower_base
                ? timeDiffSeconds(rover_obs.time, lower_base_obs.time)
                : -std::numeric_limits<double>::infinity();
            const double upper_dt = base_ok
                ? timeDiffSeconds(base_obs.time, rover_obs.time)
                : -std::numeric_limits<double>::infinity();
            bool interpolation_attempted = false;
            bool interpolation_ok = false;
            bool hold_attempted = false;
            bool hold_ok = false;

            if (base_ok && exact_dt <= kExactTimeToleranceSeconds) {
                aligned_base_obs = base_obs;
                have_aligned_base = true;
                ++exact_base_epochs;
            } else if (config.enable_base_interpolation && base_ok && have_lower_base &&
                       have_base_position &&
                       lower_dt >= -kExactTimeToleranceSeconds &&
                       upper_dt >= -kExactTimeToleranceSeconds) {
                interpolation_attempted = true;
                interpolation_ok = interpolateBaseEpoch(lower_base_obs, base_obs, rover_obs.time,
                                                        base_position, nav_data, aligned_base_obs);
                if (!interpolation_ok) {
                    interpolation_ok = interpolateBaseEpochLinear(lower_base_obs, base_obs, rover_obs.time,
                                                                  aligned_base_obs);
                }
                if (interpolation_ok) {
                    have_aligned_base = true;
                    ++interpolated_base_epochs;
                }
            } else if (have_lower_base && have_base_position && config.base_hold_seconds > 0.0) {
                hold_attempted = true;
                hold_ok = holdBaseEpoch(lower_base_obs, rover_obs.time, config.base_hold_seconds, aligned_base_obs);
                if (hold_ok) {
                    have_aligned_base = true;
                    ++held_base_epochs;
                }
            }

            if (!have_aligned_base) {
                if (config.verbose && !config.quiet) {
                    std::cout << "skip rover week=" << rover_obs.time.week
                              << " tow=" << std::fixed << std::setprecision(3) << rover_obs.time.tow
                              << " base_week=" << (base_ok ? base_obs.time.week : -1)
                              << " base_tow=" << (base_ok ? base_obs.time.tow : -1.0)
                              << " have_lower_base=" << have_lower_base
                              << " have_base_position=" << have_base_position
                              << " nav_sats=" << nav_data.ephemeris_data.size()
                              << " rover_sats=" << rover_obs.getNumSatellites()
                              << " base_sats=" << (base_ok ? base_obs.getNumSatellites() : 0)
                              << " lower_base_sats=" << (have_lower_base ? lower_base_obs.getNumSatellites() : 0)
                              << " common_obs=" << (have_lower_base && base_ok
                                  ? countCommonObservationKeys(lower_base_obs, base_obs)
                                  : 0)
                              << " lower_dt=" << lower_dt
                              << " upper_dt=" << upper_dt
                              << " interp_attempted=" << interpolation_attempted
                              << " interp_ok=" << interpolation_ok
                              << " hold_attempted=" << hold_attempted
                              << " hold_ok=" << hold_ok
                              << "\n";
                }
                ++skipped_rover_epochs;
                rover_ok = rover_input_kind == RoverInputKind::RTCM
                    ? readNextObservation(rover_rtcm_source, rover_obs, rover_events)
                    : readNextUbxObservation(rover_ubx_source, rover_obs, rover_ubx_events);
                if (rover_ok && rover_input_kind == RoverInputKind::RTCM) {
                    applyEvents(rover_events);
                    normalizeObservationWeek(rover_obs);
                } else if (rover_ok) {
                    normalizeObservationWeek(rover_obs);
                }
                continue;
            }

            ++aligned_epochs;
            if (!have_first_aligned_time) {
                first_aligned_time = rover_obs.time;
                have_first_aligned_time = true;
            }
            last_aligned_time = rover_obs.time;
            if (!have_base_position || nav_data.ephemeris_data.empty()) {
                rover_ok = rover_input_kind == RoverInputKind::RTCM
                    ? readNextObservation(rover_rtcm_source, rover_obs, rover_events)
                    : readNextUbxObservation(rover_ubx_source, rover_obs, rover_ubx_events);
                if (rover_ok && rover_input_kind == RoverInputKind::RTCM) {
                    applyEvents(rover_events);
                    normalizeObservationWeek(rover_obs);
                } else if (rover_ok) {
                    normalizeObservationWeek(rover_obs);
                }
                continue;
            }

            const auto solution = rtk.processRTKEpoch(rover_obs, aligned_base_obs, nav_data);
            if (solution.isValid()) {
                writer.writeEpoch(solution);
                ++written_solutions;
                ++solved_epochs;
                if (solution.isFixed()) {
                    ++fixed_solutions;
                }
                rover_seed = solution.position_ecef;
                if (config.verbose && !config.quiet) {
                    std::cout << "epoch " << aligned_epochs
                              << " tow=" << std::fixed << std::setprecision(3) << solution.time.tow
                              << " status=" << static_cast<int>(solution.status)
                              << " sats=" << solution.num_satellites
                              << " ratio=" << std::setprecision(2) << solution.ratio << "\n";
                }
                if (config.max_epochs >= 0 && solved_epochs >= config.max_epochs) {
                    termination_reason = "max_epochs_reached";
                    break;
                }
            }

            rover_ok = rover_input_kind == RoverInputKind::RTCM
                ? readNextObservation(rover_rtcm_source, rover_obs, rover_events)
                : readNextUbxObservation(rover_ubx_source, rover_obs, rover_ubx_events);
            if (rover_ok && rover_input_kind == RoverInputKind::RTCM) {
                applyEvents(rover_events);
                normalizeObservationWeek(rover_obs);
            } else if (rover_ok) {
                normalizeObservationWeek(rover_obs);
            }
        }

        if (termination_reason == "running") {
            if (!initial_rover_observation_ok) {
                termination_reason = "no_initial_rover_observation";
            } else if (!initial_base_observation_ok) {
                termination_reason = "no_initial_base_observation";
            } else if (written_solutions == 0 && aligned_epochs == 0) {
                termination_reason = "no_aligned_epochs";
            } else if (!rover_ok) {
                termination_reason = "rover_source_exhausted";
            } else if (!base_ok) {
                termination_reason = "base_source_exhausted";
            } else if (written_solutions == 0) {
                termination_reason = "no_valid_solutions";
            } else {
                termination_reason = "completed";
            }
        }

        writer.close();
        if (rover_input_kind == RoverInputKind::RTCM) {
            rover_rtcm_source.reader.close();
        } else {
#ifndef _WIN32
            if (rover_ubx_source.serial_fd >= 0) {
                ::close(rover_ubx_source.serial_fd);
                rover_ubx_source.serial_fd = -1;
            }
#endif
            if (rover_ubx_source.file.is_open()) {
                rover_ubx_source.file.close();
            }
        }
        base_source.reader.close();

        const DecoderSummaryStats rover_stats =
            rover_input_kind == RoverInputKind::RTCM
                ? summarizeRtcmSource(rover_rtcm_source)
                : summarizeUbxSource(rover_ubx_source);
        const DecoderSummaryStats base_stats = summarizeRtcmSource(base_source);
        const std::string rover_source_label =
            rover_input_kind == RoverInputKind::RTCM ? "rtcm" : "ubx";
        const auto wall_time_end = std::chrono::steady_clock::now();
        const double solver_wall_time_s =
            std::chrono::duration<double>(wall_time_end - wall_time_begin).count();
        const double solution_span_s =
            have_first_aligned_time
                ? std::max(0.0, timeDiffSeconds(last_aligned_time, first_aligned_time))
                : 0.0;
        const double realtime_factor =
            solver_wall_time_s > 0.0 ? solution_span_s / solver_wall_time_s : 0.0;
        const double effective_epoch_rate_hz =
            solver_wall_time_s > 0.0 ? static_cast<double>(aligned_epochs) / solver_wall_time_s : 0.0;

        std::cout << "summary: rover_source=" << rover_source_label
                  << " termination=" << termination_reason
                  << " rover_messages=" << rover_stats.valid_messages
                  << " rover_total_messages=" << rover_stats.total_messages
                  << " rover_valid_messages=" << rover_stats.valid_messages
                  << " rover_decoder_errors=" << rover_stats.decoder_errors
                  << " base_messages=" << base_stats.valid_messages
                  << " base_total_messages=" << base_stats.total_messages
                  << " base_valid_messages=" << base_stats.valid_messages
                  << " base_decoder_errors=" << base_stats.decoder_errors
                  << " aligned_epochs=" << aligned_epochs
                  << " exact_base_epochs=" << exact_base_epochs
                  << " interpolated_base_epochs=" << interpolated_base_epochs
                  << " held_base_epochs=" << held_base_epochs
                  << " skipped_rover_epochs=" << skipped_rover_epochs
                  << " written_solutions=" << written_solutions
                  << " fixed_solutions=" << fixed_solutions
                  << " nav_event_updates=" << nav_event_updates
                  << " station_position_updates=" << station_position_updates
                  << " solver_wall_time_s=" << std::fixed << std::setprecision(6) << solver_wall_time_s
                  << " solution_span_s=" << solution_span_s
                  << " realtime_factor=" << realtime_factor
                  << " effective_epoch_rate_hz=" << effective_epoch_rate_hz
                  << " out=" << config.output_pos_path
                  << " format=" << outputFormatString(config.output_format)
                  << "\n";
        return written_solutions > 0 ? 0 : 1;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
