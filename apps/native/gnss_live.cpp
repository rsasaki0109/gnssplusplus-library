#include <Eigen/Dense>

#include <chrono>
#include <deque>
#include <cmath>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <libgnss++/algorithms/rtk.hpp>
#include <libgnss++/algorithms/rtk_validation.hpp>
#include <libgnss++/algorithms/spp.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/io/observation_epoch_assembler.hpp>
#include <libgnss++/io/rinex.hpp>
#include <libgnss++/io/rtcm.hpp>
#include <libgnss++/io/rtcm_stream.hpp>
#include <libgnss++/io/solution_writer.hpp>
#include <libgnss++/io/ubx.hpp>
#include <libgnss++/io/ubx_nav_decode.hpp>
#include <libgnss++/models/troposphere.hpp>

#ifndef _WIN32
#include <cerrno>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

#include "navigation_merge.hpp"
#include "rtcm_frame_common.hpp"
#include "serial_port_common.hpp"

namespace {

using libgnss_apps::buildRtcmFrame;
using libgnss_apps::mergeNavigationData;
using libgnss_apps::mergeNavigationDataIfNew;
#ifndef _WIN32
using libgnss_apps::baudRateConstant;
#endif

constexpr double kExactTimeToleranceSeconds = 1e-3;
constexpr double kMaxBaseInterpolationGapSeconds = 2.0;
constexpr double kDefaultBaseHoldSeconds = 0.5;
// A few hundred epochs of satellite states; larger than any single epoch's working set.
constexpr size_t kSatelliteStateCacheLimit = 16384;

enum class GlonassARChoice {
    OFF,
    ON,
    AUTOCAL
};

enum class RTKTuningPreset {
    NONE,
    SURVEY,
    LOW_COST,
    MOVING_BASE,
    URBAN
};

enum class ModeChoice {
    KINEMATIC,
    MOVING_BASE
};

struct LiveConfig {
    std::string rover_rtcm_path;
    std::string rover_ubx_path;
    std::string base_rtcm_path;
    std::string nav_rinex_path;
    // Opt-in (--ubx-nav): decode broadcast ephemerides and the GPS Klobuchar set from the rover's
    // UBX-RXM-SFRBX and merge them into the navigation database.
    bool ubx_nav = false;
    std::string output_pos_path = "output/live_solution.pos";
    libgnss::io::SolutionWriter::Format output_format = libgnss::io::SolutionWriter::Format::POS;
    bool pos_uncertainty = false;
    bool pos_fix_diagnostics = false;
    int max_epochs = -1;
    size_t rover_message_limit = 0;
    size_t base_message_limit = 0;
    bool quiet = false;
    bool verbose = false;
    bool reconnect = false;
    int reconnect_delay_ms = 2000;
    int reconnect_attempts = 0;
    // Opt-in stall watchdog for tcp:// and serial/device sources (0 = off, blocking reads).
    int stall_timeout_ms = 0;
    // Opt-in (--base-outage-fallback spp): a rover epoch that would be skipped for lack of a
    // base is solved with SPP and written with status SPP; live sources get reader threads.
    bool base_outage_fallback = false;
    bool enable_base_interpolation = true;
    // Merge the per-constellation MSM messages of one epoch (RTCM multiple-message
    // bit) into a single epoch; --no-rtcm-epoch-assembly restores one epoch per message.
    bool assemble_rtcm_epochs = true;
    double base_hold_seconds = kDefaultBaseHoldSeconds;
    bool base_position_override = false;
    Eigen::Vector3d base_position_ecef = Eigen::Vector3d::Zero();
    double ratio_threshold = 3.0;
    bool enable_ar_filter = false;
    bool has_ar_filter_override = false;
    double ar_filter_margin = 0.25;
    int min_satellites_for_ar = 5;
    int min_hold_count = 5;
    double hold_ratio_threshold = 2.0;
    bool hold_fix_unchanged_set = false;
    int hold_fix_min_matched_pairs = 4;
    int min_fix_satellites = 0;
    double min_full_ratio_for_subset_ar = 0.0;
    double max_fixed_anchor_age_s = 0.0;
    double max_fixed_doppler_consensus_m = 0.0;
    double max_fixed_doppler_consensus_age_s = 0.0;
    double low_ratio_guard_threshold = 0.0;
    int low_ratio_min_fixed_ambiguities = 0;
    bool fix_history_motion_compensation = false;
    bool max_position_jump_motion_compensation = false;
    double max_position_jump_min_m = 0.0;
    double max_position_jump_rate_mps = 0.0;
    double max_float_prefit_residual_rms_m = 0.0;
    double max_float_prefit_residual_max_m = 0.0;
    int max_float_prefit_residual_reset_streak = 3;
    double elevation_mask_deg = 15.0;
    bool enable_glonass = true;
    bool enable_beidou = true;
    GlonassARChoice glonass_ar = GlonassARChoice::OFF;
    // Opt-in causal output guard (default off; see printUsage).
    bool kinematic_post_filter = false;
    ModeChoice mode = ModeChoice::KINEMATIC;
    int rover_ubx_baud = 115200;
    RTKTuningPreset preset = RTKTuningPreset::NONE;
    bool ratio_threshold_set = false;
    bool ar_filter_margin_set = false;
    bool min_satellites_for_ar_set = false;
    bool min_hold_count_set = false;
    bool hold_ratio_threshold_set = false;
    bool min_full_ratio_for_subset_ar_set = false;
    bool max_fixed_anchor_age_s_set = false;
    bool max_fixed_doppler_consensus_m_set = false;
    bool max_fixed_doppler_consensus_age_s_set = false;
    bool low_ratio_guard_threshold_set = false;
    bool low_ratio_min_fixed_ambiguities_set = false;
    bool max_position_jump_min_m_set = false;
    bool max_position_jump_rate_mps_set = false;
    bool max_float_prefit_residual_rms_m_set = false;
    bool max_float_prefit_residual_max_m_set = false;
    bool max_float_prefit_residual_reset_streak_set = false;
};

using SteadyClock = std::chrono::steady_clock;
using Deadline = std::optional<SteadyClock::time_point>;

enum class PopStatus {
    ITEM,
    TIMEOUT,
    CLOSED
};

// Bounded single-producer/single-consumer hand-off between a reader thread and the main loop.
// Only used by the opt-in --base-outage-fallback path; without the flag no thread is created.
// The reader thread does blocking I/O only: every decoder, assembler and solver stays on the
// main thread.
template <typename Item>
class BoundedPump {
public:
    explicit BoundedPump(size_t capacity) : capacity_(capacity) {}
    BoundedPump(const BoundedPump&) = delete;
    BoundedPump& operator=(const BoundedPump&) = delete;
    ~BoundedPump() { stop(); }

    void start(std::function<void()> body, std::function<void()> abort_blocking_read) {
        abort_ = std::move(abort_blocking_read);
        thread_ = std::thread(std::move(body));
    }

    // Producer: false when the pump is being stopped.
    bool push(Item&& item) {
        std::unique_lock<std::mutex> lock(mutex_);
        not_full_.wait(lock, [&] { return queue_.size() < capacity_ || stopping_.load(); });
        if (stopping_.load()) {
            return false;
        }
        queue_.push_back(std::move(item));
        not_empty_.notify_one();
        return true;
    }

    void closeProducer() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            closed_ = true;
        }
        not_empty_.notify_all();
    }

    bool stopRequested() const { return stopping_.load(); }

    // Consumer: queued items are always delivered first; TIMEOUT only when the queue is empty
    // at the (absolute) deadline; CLOSED when the producer ended and the queue is drained.
    PopStatus pop(Item& out, const Deadline& deadline) {
        std::unique_lock<std::mutex> lock(mutex_);
        const auto ready = [&] { return !queue_.empty() || closed_ || stopping_.load(); };
        if (deadline) {
            if (!not_empty_.wait_until(lock, *deadline, ready)) {
                return PopStatus::TIMEOUT;
            }
        } else {
            not_empty_.wait(lock, ready);
        }
        if (queue_.empty()) {
            return PopStatus::CLOSED;
        }
        out = std::move(queue_.front());
        queue_.pop_front();
        not_full_.notify_one();
        return PopStatus::ITEM;
    }

    // Idempotent: wakes the producer, aborts its blocking read and joins the thread.
    void stop() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_.store(true);
        }
        not_full_.notify_all();
        not_empty_.notify_all();
        if (abort_) {
            abort_();
        }
        if (thread_.joinable()) {
            thread_.join();
        }
    }

private:
    const size_t capacity_;
    std::mutex mutex_;
    std::condition_variable not_empty_;
    std::condition_variable not_full_;
    std::deque<Item> queue_;
    bool closed_ = false;
    std::atomic<bool> stopping_{false};
    std::function<void()> abort_;
    std::thread thread_;
};

constexpr size_t kPumpQueueCapacity = 4096;
constexpr int kPumpPollSliceMs = 50;

// One RTCMReader::readMessage() result; ok == false is a failed read (EOF, stall, reconnect failure).
struct RtcmItem {
    bool ok = false;
    bool stalled = false;
    libgnss::io::RTCMMessage message;
};

// One chunk of rover UBX bytes; eof marks the end of the device (stalled: because of the watchdog).
struct UbxChunk {
    std::vector<uint8_t> data;
    bool eof = false;
    bool stalled = false;
};

struct SourceState {
    libgnss::io::RTCMReader reader;
    libgnss::io::RTCMStreamDecoder decoder;
    size_t messages_read = 0;
    size_t message_limit = 0;
    bool eof = false;
    // RTCM epoch assembly (see LiveConfig::assemble_rtcm_epochs).
    bool assemble_epochs = true;
    bool input_ended = false;
    libgnss::io::ObservationEpochAssembler assembler;
    std::deque<libgnss::ObservationData> ready_epochs;
    // --reconnect: keep retrying an NTRIP/tcp:// source that drops or goes quiet instead of
    // treating the first failed read as the end of the stream (0 attempts = retry forever).
    bool reconnect = false;
    int reconnect_attempts = 0;
    // --base-outage-fallback on a live source: reader thread feeding readNextObservation().
    // Declared last so it is stopped (joined) before the reader it uses is destroyed.
    int pump_failed_reads = 0;
    bool pump_last_stalled = false;
    std::unique_ptr<BoundedPump<RtcmItem>> pump;
};

bool sourceLastReadStalled(const SourceState& source) {
    return source.pump ? source.pump_last_stalled : source.reader.lastReadStalled();
}

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
    // --stall-timeout-ms for a serial/device rover (file input is unaffected).
    int stall_timeout_ms = 0;
    int stalls = 0;
    bool last_read_stalled = false;
    bool serial = false;
    bool eof = false;
    // A 4096-byte read can complete several RXM-RAWX epochs (a RAWX-only stream, a high rate);
    // readNextUbxObservation() hands out one epoch per call, so the rest wait here, in order.
    std::deque<libgnss::ObservationData> pending_observations;
    std::unique_ptr<BoundedPump<UbxChunk>> pump;
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
        << "  --ubx-nav                  With --rover-ubx: decode the rover's UBX-RXM-SFRBX into broadcast ephemerides\n"
        << "                            (GPS/QZSS LNAV, Galileo E1-B I/NAV, BeiDou D1/D2, GLONASS) and the GPS Klobuchar set\n"
        << "                            and merge them like RTCM ephemerides: a record at once, taken back out if a later copy of the\n"
        << "                            same issue differs (ubx_nav_conflicts, ubx_nav_retracted; that satellite then needs two equal copies\n"
        << "                            before anything is merged again; ubx_nav_unconfirmed counts records never confirmed). A new issue\n"
        << "                            of a satellite that already holds a valid record of ours is held until a second copy confirms it\n"
        << "                            (ubx_nav_held_new_issue), and a confirmation removes the satellite's unconfirmed records of other\n"
        << "                            issues (counted in ubx_nav_retracted). Records whose health flags mark the satellite unusable are\n"
        << "                            skipped (ubx_nav_unhealthy_skipped); a --nav-rinex ionosphere model wins (default: off)\n"
        << "  --out <file>               Output solution file (default: output/live_solution.pos)\n"
        << "  --format <pos|llh|xyz>     Output format (default: pos)\n"
        << "  --pos-uncertainty          Append sdn sde sdu sdne sdeu sdun (m) and DiffAge(s) to .pos rows\n"
        << "  --pos-fix-diagnostics      Append FloatFixH(m) FloatFixU(m) to .pos rows: horizontal / vertical distance between the\n"
        << "                            epoch's float and FIXED positions (nan unless FIXED). Diagnostic only, not an integrity bound\n"
        << "  --max-epochs <n>           Stop after n solved epochs\n"
        << "  --rover-message-limit <n>  Stop rover ingest after n RTCM messages (0 = all)\n"
        << "  --base-message-limit <n>   Stop base ingest after n RTCM messages (0 = all)\n"
        << "  --reconnect                Reconnect NTRIP/tcp:// sources that drop or stall (default: end at the first failed read)\n"
        << "  --reconnect-delay-ms <ms>  Delay between reconnect attempts (default: 2000)\n"
        << "  --reconnect-attempts <n>   Give up after n consecutive failed reads/reconnects (0 = never, default: 0)\n"
        << "  --stall-timeout-ms <ms>    Treat a tcp:// or serial/device source that sends no byte for <ms> as stalled\n"
        << "                            (default: 0 = off, blocking reads). With --reconnect a stalled tcp:// source is\n"
        << "                            closed and reconnected (counts as a failed read); otherwise the source ends like EOF.\n"
        << "                            NTRIP keeps its own timeout; file inputs are unaffected.\n"
        << "  --base-outage-fallback spp Solve rover epochs that have no usable base with SPP and write them with status SPP\n"
        << "                            (never FLOAT/FIXED; RTK state untouched; default: off, such epochs are skipped).\n"
        << "                            On tcp/ntrip/serial sources each live source is read by its own thread, so output\n"
        << "                            continues while the base is silent; an epoch is baseless after max(1 s,\n"
        << "                            --base-hold-seconds + base interval) of wall time without a new base epoch.\n"
        << "                            Adds fallback_spp_epochs= to the summary.\n"
        << "  --no-rtcm-epoch-assembly   Treat every RTCM observation message as its own epoch (default: merge the\n"
        << "                            per-constellation MSM messages of one epoch using the multiple-message bit)\n"
        << "  --no-base-interp           Require exact rover/base epoch alignment\n"
        << "  --base-hold-seconds <v>    Reuse the latest base epoch for up to v seconds (default: 0.5)\n"
        << "  --base-ecef <x> <y> <z>    Override base ECEF position\n"
        << "  --mode <kinematic|moving-base>\n"
        << "                            RTK mode (default: kinematic)\n"
        << "  --ratio <value>            Ambiguity ratio threshold (default: 3.0)\n"
        << "  --preset <survey|low-cost|moving-base|urban>\n"
        << "                            Apply a named RTK tuning preset\n"
        << "  --arfilter                 Require extra ratio margin for subset AR fixes\n"
        << "  --no-arfilter              Disable subset AR filter margin even if a preset enables it\n"
        << "  --arfilter-margin <v>      Extra ratio margin for --arfilter (default: 0.25)\n"
        << "  --min-ar-sats <n>          Minimum satellites for AR (default: 5)\n"
        << "  --min-hold-count <n>       Consecutive fixes before hold ambiguity is allowed (default: 5)\n"
        << "  --hold-ratio-threshold <v> Ratio threshold used while hold ambiguity is active (default: 2.0)\n"
        << "  --hold-fix-unchanged-set   Reject the hold-fix fallback when a DD pair has no held integer (default: off)\n"
        << "  --hold-fix-min-matched-pairs <n> Min held DD pairs for the hold-fix fallback (default: 4)\n"
        << "  --min-fix-satellites <k>   Min non-reference satellites in an ambiguity-resolution FIX (default: 0 = off)\n"
        << "  --min-full-ratio-for-subset-ar <v> Min full-constellation LAMBDA ratio to accept a subset-AR fix (0=off)\n"
        << "  --max-fixed-anchor-age <s> Expire a FIX anchor after s seconds (0=never)\n"
        << "  --max-fixed-doppler-consensus <m> Reject a FIX more than m from the Doppler track (0=off)\n"
        << "  --max-fixed-doppler-consensus-age <s> Doppler track stops vetoing s seconds after its last FIX (0=never)\n"
        << "  --low-ratio-guard-threshold <v> Below this ratio a FIX needs the minimum fixed ambiguities (0=off)\n"
        << "  --low-ratio-min-fixed-ambiguities <n> Minimum fixed ambiguities for that guard (0=off)\n"
        << "  --fix-history-motion-compensation Allow for rover motion in the FIX history check\n"
        << "  --max-position-jump-motion-compensation Allow for rover motion in the max position jump check\n"
        << "  --max-pos-jump-min <m>     Floor of the motion-aware fixed-jump gate [m] (0=static gate)\n"
        << "  --max-pos-jump-rate <m/s>  Per-second growth of the fixed-jump gate; needed for a moving rover (0=off)\n"
        << "  --max-float-prefit-rms <m> Float prefit DD residual RMS that triggers divergence reset (0=off)\n"
        << "  --max-float-prefit-max <m> Float prefit DD residual max that triggers divergence reset (0=off)\n"
        << "  --max-float-prefit-reset-streak <n> Consecutive high-residual epochs before reset (default: 3)\n"
        << "  --elevation-mask-deg <v>   Elevation mask in degrees (default: 15)\n"
        << "  --no-glonass               Disable GLONASS carrier processing\n"
        << "  --no-beidou                Disable BeiDou carrier processing\n"
        << "  --glonass-ar <off|on|autocal>\n"
        << "  --kinematic-post-filter    Causal output guard (default: off): once a FIX has been written, do not\n"
        << "                            write an epoch whose height differs from the last written epoch by more\n"
        << "                            than max(1 m, 4 m/s * dt); a dropped epoch does not move the anchor.\n"
        << "                            Same rule as gnss_solve's kinematic post-filter (shared implementation)\n"
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

RTKTuningPreset parseRTKTuningPreset(const std::string& value, const char* argv0) {
    if (value == "survey") return RTKTuningPreset::SURVEY;
    if (value == "low-cost") return RTKTuningPreset::LOW_COST;
    if (value == "moving-base") return RTKTuningPreset::MOVING_BASE;
    if (value == "urban") return RTKTuningPreset::URBAN;
    argumentError("unsupported --preset value: " + value, argv0);
}

void applyRTKTuningPreset(LiveConfig& config) {
    switch (config.preset) {
        case RTKTuningPreset::NONE:
            return;
        case RTKTuningPreset::SURVEY:
            if (!config.ratio_threshold_set) config.ratio_threshold = 3.0;
            if (!config.has_ar_filter_override) config.enable_ar_filter = false;
            if (!config.ar_filter_margin_set) config.ar_filter_margin = 0.25;
            if (!config.min_satellites_for_ar_set) config.min_satellites_for_ar = 5;
            if (!config.min_hold_count_set) config.min_hold_count = 5;
            if (!config.hold_ratio_threshold_set) config.hold_ratio_threshold = 2.0;
            return;
        case RTKTuningPreset::LOW_COST:
            if (!config.ratio_threshold_set) config.ratio_threshold = 3.0;
            if (!config.has_ar_filter_override) config.enable_ar_filter = true;
            if (!config.ar_filter_margin_set) config.ar_filter_margin = 0.35;
            if (!config.min_satellites_for_ar_set) config.min_satellites_for_ar = 6;
            if (!config.min_hold_count_set) config.min_hold_count = 8;
            if (!config.hold_ratio_threshold_set) config.hold_ratio_threshold = 2.5;
            // Motion-aware kinematic gates (PR #177/#178): a moving low-cost rover
            // outruns the static 5 m jump gate, starving fixes; loose prefit-residual
            // reset recovers diverged floats. Mirrors gnss_solve LOW_COST defaults.
            if (!config.max_position_jump_rate_mps_set) config.max_position_jump_rate_mps = 30.0;
            if (!config.max_position_jump_min_m_set) config.max_position_jump_min_m = 5.0;
            if (!config.min_full_ratio_for_subset_ar_set) config.min_full_ratio_for_subset_ar = 1.5;
            if (!config.max_float_prefit_residual_rms_m_set) config.max_float_prefit_residual_rms_m = 4.0;
            if (!config.max_float_prefit_residual_max_m_set) config.max_float_prefit_residual_max_m = 10.0;
            if (!config.max_float_prefit_residual_reset_streak_set) config.max_float_prefit_residual_reset_streak = 5;
            return;
        case RTKTuningPreset::URBAN:
            // Same as gnss_solve --preset urban: the CDME fix-recovery recipe of
            // docs/rtk_doppler_consensus_age.md on top of the default profile.
            if (!config.max_fixed_anchor_age_s_set) config.max_fixed_anchor_age_s = 5.0;
            if (!config.max_fixed_doppler_consensus_m_set) config.max_fixed_doppler_consensus_m = 3.0;
            if (!config.max_fixed_doppler_consensus_age_s_set) config.max_fixed_doppler_consensus_age_s = 5.0;
            if (!config.low_ratio_guard_threshold_set) config.low_ratio_guard_threshold = 1e9;
            if (!config.low_ratio_min_fixed_ambiguities_set) config.low_ratio_min_fixed_ambiguities = 10;
            config.fix_history_motion_compensation = true;
            config.max_position_jump_motion_compensation = true;
            return;
        case RTKTuningPreset::MOVING_BASE:
            if (!config.ratio_threshold_set) config.ratio_threshold = 2.8;
            if (!config.has_ar_filter_override) config.enable_ar_filter = true;
            if (!config.ar_filter_margin_set) config.ar_filter_margin = 0.20;
            if (!config.min_satellites_for_ar_set) config.min_satellites_for_ar = 6;
            if (!config.min_hold_count_set) config.min_hold_count = 8;
            if (!config.hold_ratio_threshold_set) config.hold_ratio_threshold = 2.4;
            return;
    }
}

ModeChoice parseModeChoice(const std::string& value, const char* argv0) {
    if (value == "kinematic") return ModeChoice::KINEMATIC;
    if (value == "moving-base") return ModeChoice::MOVING_BASE;
    argumentError("unsupported --mode value: " + value, argv0);
}

const char* modeChoiceString(ModeChoice mode) {
    switch (mode) {
        case ModeChoice::KINEMATIC:
            return "kinematic";
        case ModeChoice::MOVING_BASE:
            return "moving-base";
    }
    return "kinematic";
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
        } else if (arg == "--ubx-nav") {
            config.ubx_nav = true;
        } else if (arg == "--out" && i + 1 < argc) {
            config.output_pos_path = argv[++i];
        } else if (arg == "--format" && i + 1 < argc) {
            config.output_format = parseOutputFormat(argv[++i], argv[0]);
        } else if (arg == "--pos-uncertainty") {
            config.pos_uncertainty = true;
        } else if (arg == "--pos-fix-diagnostics") {
            config.pos_fix_diagnostics = true;
        } else if (arg == "--max-epochs" && i + 1 < argc) {
            config.max_epochs = std::stoi(argv[++i]);
        } else if (arg == "--rover-message-limit" && i + 1 < argc) {
            config.rover_message_limit = static_cast<size_t>(std::stoull(argv[++i]));
        } else if (arg == "--base-message-limit" && i + 1 < argc) {
            config.base_message_limit = static_cast<size_t>(std::stoull(argv[++i]));
        } else if (arg == "--reconnect") {
            config.reconnect = true;
        } else if (arg == "--reconnect-delay-ms" && i + 1 < argc) {
            config.reconnect_delay_ms = std::stoi(argv[++i]);
        } else if (arg == "--reconnect-attempts" && i + 1 < argc) {
            config.reconnect_attempts = std::stoi(argv[++i]);
        } else if (arg == "--stall-timeout-ms" && i + 1 < argc) {
            config.stall_timeout_ms = std::stoi(argv[++i]);
        } else if (arg == "--base-outage-fallback" && i + 1 < argc) {
            const std::string fallback = argv[++i];
            if (fallback != "spp") {
                argumentError("--base-outage-fallback accepts only 'spp'", argv[0]);
            }
            config.base_outage_fallback = true;
        } else if (arg == "--no-rtcm-epoch-assembly") {
            config.assemble_rtcm_epochs = false;
        } else if (arg == "--no-base-interp") {
            config.enable_base_interpolation = false;
        } else if (arg == "--base-hold-seconds" && i + 1 < argc) {
            config.base_hold_seconds = std::stod(argv[++i]);
        } else if (arg == "--base-ecef" && i + 3 < argc) {
            // Do not increment i multiple times in constructor arguments: C++
            // does not guarantee their evaluation order, and MSVC builds read
            // the supplied XYZ reversed.
            const double base_x = std::stod(argv[++i]);
            const double base_y = std::stod(argv[++i]);
            const double base_z = std::stod(argv[++i]);
            config.base_position_ecef = Eigen::Vector3d(base_x, base_y, base_z);
            config.base_position_override = true;
        } else if (arg == "--mode" && i + 1 < argc) {
            config.mode = parseModeChoice(argv[++i], argv[0]);
        } else if (arg == "--ratio" && i + 1 < argc) {
            config.ratio_threshold = std::stod(argv[++i]);
            config.ratio_threshold_set = true;
        } else if (arg == "--preset" && i + 1 < argc) {
            config.preset = parseRTKTuningPreset(argv[++i], argv[0]);
        } else if (arg == "--arfilter") {
            config.enable_ar_filter = true;
            config.has_ar_filter_override = true;
        } else if (arg == "--no-arfilter") {
            config.enable_ar_filter = false;
            config.has_ar_filter_override = true;
        } else if (arg == "--arfilter-margin" && i + 1 < argc) {
            config.ar_filter_margin = std::stod(argv[++i]);
            config.ar_filter_margin_set = true;
        } else if (arg == "--min-ar-sats" && i + 1 < argc) {
            config.min_satellites_for_ar = std::stoi(argv[++i]);
            config.min_satellites_for_ar_set = true;
        } else if (arg == "--min-hold-count" && i + 1 < argc) {
            config.min_hold_count = std::stoi(argv[++i]);
            config.min_hold_count_set = true;
        } else if (arg == "--hold-ratio-threshold" && i + 1 < argc) {
            config.hold_ratio_threshold = std::stod(argv[++i]);
            config.hold_ratio_threshold_set = true;
        } else if (arg == "--hold-fix-unchanged-set") {
            config.hold_fix_unchanged_set = true;
        } else if (arg == "--hold-fix-min-matched-pairs" && i + 1 < argc) {
            config.hold_fix_min_matched_pairs = std::stoi(argv[++i]);
        } else if (arg == "--min-fix-satellites" && i + 1 < argc) {
            config.min_fix_satellites = std::stoi(argv[++i]);
        } else if (arg == "--min-full-ratio-for-subset-ar" && i + 1 < argc) {
            config.min_full_ratio_for_subset_ar = std::stod(argv[++i]);
            config.min_full_ratio_for_subset_ar_set = true;
        } else if (arg == "--max-fixed-anchor-age" && i + 1 < argc) {
            config.max_fixed_anchor_age_s = std::stod(argv[++i]);
            config.max_fixed_anchor_age_s_set = true;
        } else if (arg == "--max-fixed-doppler-consensus" && i + 1 < argc) {
            config.max_fixed_doppler_consensus_m = std::stod(argv[++i]);
            config.max_fixed_doppler_consensus_m_set = true;
        } else if (arg == "--max-fixed-doppler-consensus-age" && i + 1 < argc) {
            config.max_fixed_doppler_consensus_age_s = std::stod(argv[++i]);
            config.max_fixed_doppler_consensus_age_s_set = true;
        } else if (arg == "--low-ratio-guard-threshold" && i + 1 < argc) {
            config.low_ratio_guard_threshold = std::stod(argv[++i]);
            config.low_ratio_guard_threshold_set = true;
        } else if (arg == "--low-ratio-min-fixed-ambiguities" && i + 1 < argc) {
            config.low_ratio_min_fixed_ambiguities = std::stoi(argv[++i]);
            config.low_ratio_min_fixed_ambiguities_set = true;
        } else if (arg == "--fix-history-motion-compensation") {
            config.fix_history_motion_compensation = true;
        } else if (arg == "--max-position-jump-motion-compensation") {
            config.max_position_jump_motion_compensation = true;
        } else if (arg == "--max-pos-jump-min" && i + 1 < argc) {
            config.max_position_jump_min_m = std::stod(argv[++i]);
            config.max_position_jump_min_m_set = true;
        } else if (arg == "--max-pos-jump-rate" && i + 1 < argc) {
            config.max_position_jump_rate_mps = std::stod(argv[++i]);
            config.max_position_jump_rate_mps_set = true;
        } else if (arg == "--max-float-prefit-rms" && i + 1 < argc) {
            config.max_float_prefit_residual_rms_m = std::stod(argv[++i]);
            config.max_float_prefit_residual_rms_m_set = true;
        } else if (arg == "--max-float-prefit-max" && i + 1 < argc) {
            config.max_float_prefit_residual_max_m = std::stod(argv[++i]);
            config.max_float_prefit_residual_max_m_set = true;
        } else if (arg == "--max-float-prefit-reset-streak" && i + 1 < argc) {
            config.max_float_prefit_residual_reset_streak = std::stoi(argv[++i]);
            config.max_float_prefit_residual_reset_streak_set = true;
        } else if (arg == "--elevation-mask-deg" && i + 1 < argc) {
            config.elevation_mask_deg = std::stod(argv[++i]);
        } else if (arg == "--no-glonass") {
            config.enable_glonass = false;
        } else if (arg == "--no-beidou") {
            config.enable_beidou = false;
        } else if (arg == "--glonass-ar" && i + 1 < argc) {
            config.glonass_ar = parseGlonassARChoice(argv[++i], argv[0]);
        } else if (arg == "--kinematic-post-filter") {
            config.kinematic_post_filter = true;
        } else if (arg == "--quiet") {
            config.quiet = true;
        } else if (arg == "--verbose") {
            config.verbose = true;
        } else {
            argumentError("unknown or incomplete argument: " + arg, argv[0]);
        }
    }

    applyRTKTuningPreset(config);

    const bool has_rover_rtcm = !config.rover_rtcm_path.empty();
    const bool has_rover_ubx = !config.rover_ubx_path.empty();
    if (has_rover_rtcm == has_rover_ubx) {
        argumentError("choose exactly one of --rover-rtcm or --rover-ubx", argv[0]);
    }
    if (config.base_rtcm_path.empty()) {
        argumentError("--base-rtcm is required", argv[0]);
    }
    if (config.ubx_nav && !has_rover_ubx) {
        argumentError("--ubx-nav requires --rover-ubx", argv[0]);
    }
    if (config.max_epochs == 0) {
        argumentError("--max-epochs must be != 0", argv[0]);
    }
    if (config.min_satellites_for_ar < 4) {
        argumentError("--min-ar-sats must be >= 4", argv[0]);
    }
    if (config.ar_filter_margin < 0.0) {
        argumentError("--arfilter-margin must be >= 0", argv[0]);
    }
    if (config.min_hold_count < 0) {
        argumentError("--min-hold-count must be >= 0", argv[0]);
    }
    if (config.hold_ratio_threshold <= 0.0) {
        argumentError("--hold-ratio-threshold must be > 0", argv[0]);
    }
    if (config.hold_fix_min_matched_pairs < 4) {
        argumentError("--hold-fix-min-matched-pairs must be >= 4", argv[0]);
    }
    if (config.min_fix_satellites < 0) {
        argumentError("--min-fix-satellites must be >= 0", argv[0]);
    }
    if (config.max_position_jump_min_m < 0.0) {
        argumentError("--max-pos-jump-min must be >= 0", argv[0]);
    }
    if (config.max_position_jump_rate_mps < 0.0) {
        argumentError("--max-pos-jump-rate must be >= 0", argv[0]);
    }
    if (config.max_fixed_anchor_age_s < 0.0) {
        argumentError("--max-fixed-anchor-age must be >= 0", argv[0]);
    }
    if (config.max_fixed_doppler_consensus_m < 0.0) {
        argumentError("--max-fixed-doppler-consensus must be >= 0", argv[0]);
    }
    if (config.max_fixed_doppler_consensus_age_s < 0.0) {
        argumentError("--max-fixed-doppler-consensus-age must be >= 0", argv[0]);
    }
    if (config.low_ratio_guard_threshold < 0.0) {
        argumentError("--low-ratio-guard-threshold must be >= 0", argv[0]);
    }
    if (config.low_ratio_min_fixed_ambiguities < 0) {
        argumentError("--low-ratio-min-fixed-ambiguities must be >= 0", argv[0]);
    }
    if (config.max_float_prefit_residual_rms_m < 0.0) {
        argumentError("--max-float-prefit-rms must be >= 0", argv[0]);
    }
    if (config.max_float_prefit_residual_max_m < 0.0) {
        argumentError("--max-float-prefit-max must be >= 0", argv[0]);
    }
    if (config.max_float_prefit_residual_reset_streak < 1) {
        argumentError("--max-float-prefit-reset-streak must be >= 1", argv[0]);
    }
    if (config.base_hold_seconds < 0.0) {
        argumentError("--base-hold-seconds must be >= 0", argv[0]);
    }
    if (config.pos_uncertainty &&
        config.output_format != libgnss::io::SolutionWriter::Format::POS) {
        argumentError("--pos-uncertainty requires --format pos", argv[0]);
    }
    if (config.pos_fix_diagnostics &&
        config.output_format != libgnss::io::SolutionWriter::Format::POS) {
        argumentError("--pos-fix-diagnostics requires --format pos", argv[0]);
    }
    if (config.rover_ubx_baud <= 0) {
        argumentError("--rover-ubx-baud must be > 0", argv[0]);
    }
    if (config.reconnect_delay_ms < 0) {
        argumentError("--reconnect-delay-ms must be >= 0", argv[0]);
    }
    if (config.stall_timeout_ms < 0) {
        argumentError("--stall-timeout-ms must be >= 0", argv[0]);
    }
    if (config.reconnect_attempts < 0) {
        argumentError("--reconnect-attempts must be >= 0", argv[0]);
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

bool openSource(const std::string& path, size_t message_limit, SourceState& source,
                const LiveConfig& config) {
    source.messages_read = 0;
    source.message_limit = message_limit;
    source.eof = false;
    source.decoder.clear();
    source.assemble_epochs = config.assemble_rtcm_epochs;
    source.input_ended = false;
    source.assembler.clear();
    source.ready_epochs.clear();
    source.reconnect = false;
    source.reconnect_attempts = 0;
    if (!source.reader.open(path)) {
        return false;
    }
    if (config.reconnect && (source.reader.isNtripSource() || source.reader.isTcpStream())) {
        source.reader.setAutoReconnect(true, config.reconnect_delay_ms);
        source.reconnect = true;
        source.reconnect_attempts = config.reconnect_attempts;
    }
    if (config.stall_timeout_ms > 0) {
        source.reader.setStallTimeoutMs(config.stall_timeout_ms);
    }
    return true;
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
    source.stalls = 0;
    source.last_read_stalled = false;
    source.serial = false;
    source.pending_observations.clear();
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

// deadline/timed_out are only meaningful for a source with a reader thread (--base-outage-fallback):
// a queue that stays empty until the deadline returns false with *timed_out set, and the source
// is NOT marked ended. A source without a pump reads (and blocks) exactly as before.
bool readNextObservation(SourceState& source,
                         libgnss::ObservationData& obs,
                         std::vector<libgnss::io::RTCMStreamDecoder::Event>& events,
                         const Deadline& deadline = std::nullopt,
                         bool* timed_out = nullptr) {
    events.clear();
    auto popReady = [&]() {
        if (source.ready_epochs.empty()) {
            return false;
        }
        obs = std::move(source.ready_epochs.front());
        source.ready_epochs.pop_front();
        return true;
    };
    if (popReady()) {
        return true;
    }
    libgnss::io::RTCMMessage raw_message;
    int local_failed_reads = 0;
    int& failed_reads = source.pump ? source.pump_failed_reads : local_failed_reads;
    while (!source.input_ended &&
           (source.message_limit == 0 || source.messages_read < source.message_limit)) {
        bool fetched = false;
        if (source.pump) {
            RtcmItem item;
            const PopStatus status = source.pump->pop(item, deadline);
            if (status == PopStatus::TIMEOUT) {
                if (timed_out) {
                    *timed_out = true;
                }
                return false;
            }
            if (status == PopStatus::CLOSED) {
                break;
            }
            fetched = item.ok;
            source.pump_last_stalled = item.stalled;
            raw_message = std::move(item.message);
        } else {
            fetched = source.reader.readMessage(raw_message);
        }
        if (!fetched) {
            // With --reconnect an empty read (NTRIP timeout, dropped link being re-established)
            // is not the end of the stream; keep trying until the attempt budget runs out.
            if (source.reconnect &&
                (source.reconnect_attempts == 0 || ++failed_reads < source.reconnect_attempts)) {
                continue;
            }
            break;
        }
        failed_reads = 0;
        ++source.messages_read;
        const auto frame = buildRtcmFrame(raw_message);
        std::vector<libgnss::io::RTCMStreamDecoder::Event> frame_events;
        source.decoder.pushFrame(frame.data(), frame.size(), frame_events);
        events.insert(events.end(), frame_events.begin(), frame_events.end());
        for (const auto& event : frame_events) {
            if (!event.has_observation) {
                continue;
            }
            if (!source.assemble_epochs) {
                obs = event.observation;
                return true;
            }
            for (auto& epoch : source.assembler.push(event.observation, event.observation_more_follows)) {
                source.ready_epochs.push_back(std::move(epoch));
            }
        }
        if (popReady()) {
            return true;
        }
    }
    // End of stream: a pending (partially assembled) epoch must still be processed.
    source.input_ended = true;
    for (auto& epoch : source.assembler.flush()) {
        source.ready_epochs.push_back(std::move(epoch));
    }
    if (popReady()) {
        return true;
    }
    source.eof = true;
    return false;
}

bool readNextUbxObservation(RoverUbxSource& source,
                            libgnss::ObservationData& obs,
                            std::vector<libgnss::io::UBXStreamDecoder::Event>& events) {
    events.clear();
    source.last_read_stalled = false;
    // Epochs an earlier chunk completed after the one already returned come first, before any
    // further read (and so before eof or a stall is reported). Their chunk's events went out
    // with the first epoch, so `events` stays empty here.
    if (!source.pending_observations.empty()) {
        obs = std::move(source.pending_observations.front());
        source.pending_observations.pop_front();
        return true;
    }
    std::vector<uint8_t> chunk(4096);

    while (true) {
        size_t bytes_read = 0;
        if (source.pump) {
            UbxChunk item;
            if (source.pump->pop(item, std::nullopt) != PopStatus::ITEM || item.eof) {
                if (item.stalled) {
                    ++source.stalls;
                    source.last_read_stalled = true;
                }
                source.eof = true;
                return false;
            }
            chunk = std::move(item.data);
            bytes_read = chunk.size();
        } else if (source.serial) {
#ifndef _WIN32
            // Opt-in stall watchdog (same poll() wait as RTCMReader); a serial device cannot be
            // reconnected, so a stall ends the rover source like EOF.
            if (source.stall_timeout_ms > 0 &&
                libgnss::io::waitForReadable(source.serial_fd, source.stall_timeout_ms) == 0) {
                ++source.stalls;
                source.last_read_stalled = true;
                source.eof = true;
                return false;
            }
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
        bool have_observation = false;
        for (const auto& event : chunk_events) {
            if (!event.has_observation) {
                continue;
            }
            if (!have_observation) {
                obs = event.observation;
                have_observation = true;
            } else {
                source.pending_observations.push_back(event.observation);
            }
        }
        if (have_observation) {
            return true;
        }
    }
}

// Start the reader thread of a live RTCM source (--base-outage-fallback only).
void startRtcmPump(SourceState& source) {
    source.reader.setAbortable(true);
    source.pump = std::make_unique<BoundedPump<RtcmItem>>(kPumpQueueCapacity);
    auto* pump = source.pump.get();
    libgnss::io::RTCMReader* reader = &source.reader;
    const bool keep_reading_after_failure = source.reconnect;
    pump->start(
        [pump, reader, keep_reading_after_failure] {
            try {
                while (!pump->stopRequested()) {
                    RtcmItem item;
                    item.ok = reader->readMessage(item.message);
                    item.stalled = reader->lastReadStalled();
                    if (pump->stopRequested()) {
                        break;
                    }
                    const bool ok = item.ok;
                    if (!pump->push(std::move(item))) {
                        break;
                    }
                    if (!ok && !keep_reading_after_failure) {
                        break;
                    }
                }
            } catch (...) {
                RtcmItem failed;
                pump->push(std::move(failed));
            }
            pump->closeProducer();
        },
        [reader] { reader->requestAbort(); });
}

#ifndef _WIN32
// Start the reader thread of a serial rover UBX device (--base-outage-fallback only).
void startUbxPump(RoverUbxSource& source) {
    source.pump = std::make_unique<BoundedPump<UbxChunk>>(kPumpQueueCapacity);
    auto* pump = source.pump.get();
    const int fd = source.serial_fd;
    const int stall_timeout_ms = source.stall_timeout_ms;
    pump->start(
        [pump, fd, stall_timeout_ms] {
            std::vector<uint8_t> buffer(4096);
            auto last_data = SteadyClock::now();
            while (!pump->stopRequested()) {
                if (libgnss::io::waitForReadable(fd, kPumpPollSliceMs) == 0) {
                    if (stall_timeout_ms > 0 &&
                        SteadyClock::now() - last_data >= std::chrono::milliseconds(stall_timeout_ms)) {
                        UbxChunk stalled;
                        stalled.eof = true;
                        stalled.stalled = true;
                        pump->push(std::move(stalled));
                        break;
                    }
                    continue;
                }
                const ssize_t count = ::read(fd, buffer.data(), buffer.size());
                if (count < 0 && errno == EINTR) {
                    continue;
                }
                UbxChunk chunk;
                if (count <= 0) {
                    chunk.eof = true;
                    pump->push(std::move(chunk));
                    break;
                }
                chunk.data.assign(buffer.begin(), buffer.begin() + count);
                last_data = SteadyClock::now();
                if (!pump->push(std::move(chunk))) {
                    break;
                }
            }
            pump->closeProducer();
        },
        {});
}
#endif

// The SPP configuration the RTK processor uses for its own seed (rtk_internal::makeRTKSppConfig).
libgnss::SPPProcessor::SPPConfig makeFallbackSppConfig(const LiveConfig& config) {
    libgnss::SPPProcessor::SPPConfig spp_config;
    spp_config.use_multi_constellation = true;
    spp_config.enable_glonass = config.enable_glonass;
    spp_config.enable_beidou = config.enable_beidou;
    spp_config.max_position_sigma_m = 1000.0;
    return spp_config;
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
            if (!openSource(config.rover_rtcm_path, config.rover_message_limit, rover_rtcm_source, config)) {
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
            rover_ubx_source.stall_timeout_ms = config.stall_timeout_ms;
        }
        if (!openSource(config.base_rtcm_path, config.base_message_limit, base_source, config)) {
            std::cerr << "Error: failed to open base RTCM source: "
                      << config.base_rtcm_path << "\n";
            return 1;
        }

        libgnss::NavigationData nav_data;
        // A live run is open-ended: cap the per-(satellite, time) memo so epochs that never see a
        // new ephemeris (navigation RINEX, or a quiet caster) cannot grow it without bound.
        nav_data.setSatelliteStateCacheLimit(kSatelliteStateCacheLimit);
        if (!loadNavigationRinex(config.nav_rinex_path, nav_data)) {
            std::cerr << "Error: failed to read navigation RINEX: "
                      << config.nav_rinex_path << "\n";
            return 1;
        }

        // Seed the RTCM decoders' GPST reference from the navigation RINEX (latest GPS toe) so that
        // TOW-only parts get their week and GLONASS epochs are dated from the data, not the host clock.
        if (const auto seed = libgnss::io::RTCMStreamDecoder::referenceFromNavigation(nav_data)) {
            base_source.decoder.setReferenceTime(*seed);
            if (rover_input_kind == RoverInputKind::RTCM) {
                rover_rtcm_source.decoder.setReferenceTime(*seed);
            }
        }

        libgnss::RTKProcessor rtk;
        libgnss::RTKProcessor::RTKConfig rtk_config;
        rtk_config.position_mode =
            config.mode == ModeChoice::MOVING_BASE
                ? libgnss::RTKProcessor::RTKConfig::PositionMode::MOVING_BASE
                : libgnss::RTKProcessor::RTKConfig::PositionMode::KINEMATIC;
        rtk_config.ratio_threshold = config.ratio_threshold;
        rtk_config.ambiguity_ratio_threshold = config.ratio_threshold;
        rtk_config.hold_ambiguity_ratio_threshold = config.hold_ratio_threshold;
        rtk_config.enable_ar_filter = config.enable_ar_filter;
        rtk_config.ar_filter_margin = config.ar_filter_margin;
        rtk_config.min_satellites_for_ar = config.min_satellites_for_ar;
        rtk_config.min_hold_count = config.min_hold_count;
        rtk_config.hold_fix_unchanged_set_only = config.hold_fix_unchanged_set;
        rtk_config.hold_fix_min_matched_pairs = config.hold_fix_min_matched_pairs;
        rtk_config.min_fix_satellites = config.min_fix_satellites;
        rtk_config.min_full_ratio_for_subset_ar = config.min_full_ratio_for_subset_ar;
        rtk_config.max_fixed_anchor_age_s = config.max_fixed_anchor_age_s;
        rtk_config.max_fixed_doppler_consensus_m = config.max_fixed_doppler_consensus_m;
        rtk_config.max_fixed_doppler_consensus_age_s = config.max_fixed_doppler_consensus_age_s;
        rtk_config.low_ratio_guard_threshold = config.low_ratio_guard_threshold;
        rtk_config.low_ratio_min_fixed_ambiguities = config.low_ratio_min_fixed_ambiguities;
        rtk_config.fix_history_motion_compensation = config.fix_history_motion_compensation;
        rtk_config.max_position_jump_motion_compensation =
            config.max_position_jump_motion_compensation;
        rtk_config.max_position_jump_min_m = config.max_position_jump_min_m;
        rtk_config.max_position_jump_rate_mps = config.max_position_jump_rate_mps;
        rtk_config.max_float_prefit_residual_rms_m = config.max_float_prefit_residual_rms_m;
        rtk_config.max_float_prefit_residual_max_m = config.max_float_prefit_residual_max_m;
        rtk_config.max_float_prefit_residual_reset_streak =
            config.max_float_prefit_residual_reset_streak;
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
        if (!writer.open(config.output_pos_path, config.output_format, config.pos_uncertainty,
                         config.pos_fix_diagnostics)) {
            std::cerr << "Error: failed to open output file\n";
            return 1;
        }

        libgnss::ObservationData rover_obs;
        libgnss::ObservationData base_obs;
        std::vector<libgnss::io::RTCMStreamDecoder::Event> rover_events;
        std::vector<libgnss::io::RTCMStreamDecoder::Event> base_events;
        std::vector<libgnss::io::UBXStreamDecoder::Event> rover_ubx_events;

        // --ubx-nav: ephemerides and the Klobuchar set decoded from the rover's UBX-RXM-SFRBX. The
        // decoder's GPST reference is the newest RAWX epoch; events are applied in stream order.
        libgnss::io::UBXNavDecoder ubx_nav_decoder;
        libgnss::io::UBXNavAcceptConfirm ubx_nav_merge;
        std::vector<libgnss::Ephemeris> ubx_nav_new;
        std::vector<libgnss::io::UBXNavDecoded> ubx_nav_copies;
        auto applyUbxNavEvents = [&](const std::vector<libgnss::io::UBXStreamDecoder::Event>& events) {
            if (!config.ubx_nav) {
                return;
            }
            for (const auto& event : events) {
                if (event.has_observation) {
                    ubx_nav_decoder.setReferenceTime(event.observation.time);
                }
                if (event.has_sfrbx) {
                    ubx_nav_new.clear();
                    ubx_nav_copies.clear();
                    ubx_nav_decoder.pushSfrbx(event.sfrbx, ubx_nav_new, &ubx_nav_copies);
                    for (const auto& copy : ubx_nav_copies) {
                        // Accept, then confirm (D4): the first copy of a record is merged at once
                        // (after the health rule) unless the satellite already holds a valid record
                        // of ours, in which case a new issue waits for a second copy; a differing
                        // second copy takes a merged record back out and the satellite waits for two
                        // equal copies (confirm-twice) before anything is merged for it again; a
                        // confirmation removes the satellite's unconfirmed records of other issues.
                        ubx_nav_merge.push(copy, nav_data);
                    }
                }
            }
            if (!nav_data.ionosphere_model.valid && ubx_nav_decoder.hasKlobuchar()) {
                nav_data.ionosphere_model = ubx_nav_decoder.klobuchar();
            }
        };
        auto readRoverUbx = [&](libgnss::ObservationData& obs) {
            const bool ok = readNextUbxObservation(rover_ubx_source, obs, rover_ubx_events);
            applyUbxNavEvents(rover_ubx_events);
            return ok;
        };

        // When one RTCM decoder obtains a GPST reference (ephemerides) and the other has none (the rover
        // stream usually carries no ephemerides), the other is seeded with it.
        auto shareReferences = [&]() {
            if (rover_input_kind != RoverInputKind::RTCM) {
                return;
            }
            const auto rover_ref = rover_rtcm_source.decoder.referenceTime();
            const auto base_ref = base_source.decoder.referenceTime();
            if (base_ref && !rover_ref) {
                rover_rtcm_source.decoder.setReferenceTime(*base_ref);
            } else if (rover_ref && !base_ref) {
                base_source.decoder.setReferenceTime(*rover_ref);
            }
        };
        // --base-outage-fallback: each live (tcp/ntrip/serial) source gets a reader thread so that
        // the rover keeps being read while the base is silent. File sources and the default
        // (flag off) path read inline and create no thread.
        const bool fallback_enabled = config.base_outage_fallback;
        if (fallback_enabled) {
            if (base_source.reader.isLiveStream()) {
                startRtcmPump(base_source);
            }
            if (rover_input_kind == RoverInputKind::RTCM) {
                if (rover_rtcm_source.reader.isLiveStream()) {
                    startRtcmPump(rover_rtcm_source);
                }
            }
#ifndef _WIN32
            else if (rover_ubx_source.serial) {
                startUbxPump(rover_ubx_source);
            }
#endif
        }
        const bool base_threaded = base_source.pump != nullptr;
        libgnss::SPPProcessor fallback_spp(makeFallbackSppConfig(config));

        // --ubx-nav: a base stream without ephemerides (and no --nav-rinex) gives its decoder no GPST
        // reference, so its TOW-only epochs would be dated from the host clock and never align with the
        // rover. The rover's RAWX time is that reference: read the first rover epoch before the base.
        bool rover_read_first = false;
        bool rover_ok = false;
        if (config.ubx_nav && rover_input_kind == RoverInputKind::UBX &&
            !base_source.decoder.referenceTime()) {
            rover_ok = readRoverUbx(rover_obs);
            rover_read_first = true;
            if (rover_ok && rover_obs.time.week > 0) {
                base_source.decoder.setReferenceTime(rover_obs.time);
            }
        }

        // The base is read first so that its ephemeris burst can seed the rover decoder before the
        // rover's first epoch is decoded.
        bool base_ok = readNextObservation(base_source, base_obs, base_events);
        shareReferences();
        if (!rover_read_first) {
            rover_ok = rover_input_kind == RoverInputKind::RTCM
                ? readNextObservation(rover_rtcm_source, rover_obs, rover_events)
                : readRoverUbx(rover_obs);
        }
        const bool initial_rover_observation_ok = rover_ok;
        const bool initial_base_observation_ok = base_ok;

        size_t aligned_epochs = 0;
        size_t exact_base_epochs = 0;
        size_t interpolated_base_epochs = 0;
        size_t held_base_epochs = 0;
        size_t skipped_rover_epochs = 0;
        size_t fallback_spp_epochs = 0;
        size_t written_solutions = 0;
        size_t fixed_solutions = 0;
        size_t post_filter_dropped_epochs = 0;
        libgnss::rtk_validation::KinematicPostFilter kinematic_post_filter;
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
                    mergeNavigationDataIfNew(nav_data, event.navigation);
                    ++nav_event_updates;
                }
                if (event.has_station_position && !config.base_position_override) {
                    base_position = event.station_position;
                    have_base_position = true;
                    rtk.setBasePosition(base_position);
                    ++station_position_updates;
                }
            }
            shareReferences();
        };
        // A week-0 (TOW-only) epoch gets the week nearest to the newest decoder reference; the
        // oldest-ephemeris week is only the fallback when no decoder has a reference (UBX rover
        // without a base reference).
        auto normalizeObservationWeek = [&](libgnss::ObservationData& obs) {
            if (obs.time.week != 0) {
                return;
            }
            std::optional<libgnss::GNSSTime> newest = base_source.decoder.referenceTime();
            if (rover_input_kind == RoverInputKind::RTCM) {
                const auto rover_ref = rover_rtcm_source.decoder.referenceTime();
                if (rover_ref && (!newest || *rover_ref - *newest > 0.0)) {
                    newest = rover_ref;
                }
            }
            if (newest) {
                obs.time = libgnss::io::RTCMStreamDecoder::resolveTowOnlyTime(obs.time, *newest);
                return;
            }
            const int week = inferObservationWeekContext(nav_data);
            if (week > 0) {
                obs.time.week = week;
            }
        };

        // Base silence clock for --base-outage-fallback (threaded base only): wall time of the newest
        // base epoch taken from the source, and the base epoch spacing (data time). A rover epoch is
        // baseless once max(1 s, base hold + base interval) of wall time passed with no newer base epoch.
        bool base_waiting = false;  // the last base read timed out (source silent, not ended)
        auto last_base_obs_wall = SteadyClock::now();
        double base_interval_s = 1.0;
        bool base_interval_measured = false;
        bool have_prev_base_time = false;
        libgnss::GNSSTime prev_base_time;
        auto noteBaseEpoch = [&](const libgnss::ObservationData& epoch) {
            if (!base_threaded) {
                return;
            }
            last_base_obs_wall = SteadyClock::now();
            if (have_prev_base_time) {
                const double dt = timeDiffSeconds(epoch.time, prev_base_time);
                // The first spacing sets the interval; later ones count only while within 2x of it
                // (a gap after an outage must not stretch the timeout).
                if (dt > 1e-3 && dt <= 60.0 && (!base_interval_measured || dt <= 2.0 * base_interval_s)) {
                    base_interval_s = dt;
                    base_interval_measured = true;
                }
            }
            prev_base_time = epoch.time;
            have_prev_base_time = true;
        };
        auto baseDeadline = [&]() -> Deadline {
            if (!base_threaded) {
                return std::nullopt;
            }
            const double timeout_s = std::max(1.0, config.base_hold_seconds + base_interval_s);
            return last_base_obs_wall + std::chrono::duration_cast<SteadyClock::duration>(
                                            std::chrono::duration<double>(timeout_s));
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
            noteBaseEpoch(base_obs);
        }

        while (rover_ok && (base_ok || base_waiting) &&
               (config.max_epochs < 0 || solved_epochs < config.max_epochs)) {
            if (base_waiting) {
                // The previous base read timed out: look again (blocking only until the silence deadline).
                bool timed_out = false;
                base_ok = readNextObservation(base_source, base_obs, base_events, baseDeadline(), &timed_out);
                if (base_ok) {
                    applyEvents(base_events);
                    normalizeObservationWeek(base_obs);
                    noteBaseEpoch(base_obs);
                    base_waiting = false;
                } else if (!timed_out) {
                    base_waiting = false;  // the base source ended
                    break;
                }
            }
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
                bool base_timed_out = false;
                base_ok = readNextObservation(base_source, base_obs, base_events, baseDeadline(), &base_timed_out);
                if (base_ok) {
                    applyEvents(base_events);
                    normalizeObservationWeek(base_obs);
                    noteBaseEpoch(base_obs);
                } else if (base_timed_out) {
                    base_waiting = true;
                }
            }

            libgnss::ObservationData aligned_base_obs;
            const double exact_dt = base_ok ? std::abs(timeDiffSeconds(base_obs.time, rover_obs.time))
                                            : std::numeric_limits<double>::infinity();
            bool have_aligned_base = false;
            // Age of differential for --pos-uncertainty: rover time minus the
            // (older) base observation time actually used.
            double epoch_differential_age_s = std::numeric_limits<double>::quiet_NaN();
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
                epoch_differential_age_s = timeDiffSeconds(rover_obs.time, base_obs.time);
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
                    epoch_differential_age_s = lower_dt;
                    ++interpolated_base_epochs;
                }
            } else if (have_lower_base && have_base_position && config.base_hold_seconds > 0.0) {
                hold_attempted = true;
                hold_ok = holdBaseEpoch(lower_base_obs, rover_obs.time, config.base_hold_seconds, aligned_base_obs);
                if (hold_ok) {
                    have_aligned_base = true;
                    epoch_differential_age_s = lower_dt;
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
                if (fallback_enabled) {
                    // No usable base for this rover epoch: a single-point solution, never FLOAT/FIXED.
                    // The RTK processor, the output guards and the rover seed are not touched.
                    const auto spp_solution = fallback_spp.processEpoch(rover_obs, nav_data);
                    if (spp_solution.status == libgnss::SolutionStatus::SPP && spp_solution.isValid()) {
                        writer.writeEpoch(spp_solution, std::numeric_limits<double>::quiet_NaN());
                        ++written_solutions;
                        ++solved_epochs;
                        ++fallback_spp_epochs;
                        if (config.verbose && !config.quiet) {
                            std::cout << "fallback tow=" << std::fixed << std::setprecision(3)
                                      << spp_solution.time.tow << " status="
                                      << static_cast<int>(spp_solution.status)
                                      << " sats=" << spp_solution.num_satellites << "\n";
                        }
                        if (config.max_epochs >= 0 && solved_epochs >= config.max_epochs) {
                            termination_reason = "max_epochs_reached";
                            break;
                        }
                    }
                }
                rover_ok = rover_input_kind == RoverInputKind::RTCM
                    ? readNextObservation(rover_rtcm_source, rover_obs, rover_events)
                    : readRoverUbx(rover_obs);
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
                    : readRoverUbx(rover_obs);
                if (rover_ok && rover_input_kind == RoverInputKind::RTCM) {
                    applyEvents(rover_events);
                    normalizeObservationWeek(rover_obs);
                } else if (rover_ok) {
                    normalizeObservationWeek(rover_obs);
                }
                continue;
            }

            const auto solution = rtk.processRTKEpoch(rover_obs, aligned_base_obs, nav_data);
            bool write_epoch = solution.isValid();
            if (write_epoch && config.kinematic_post_filter) {
                if (kinematic_post_filter.exceedsHeightStep(solution)) {
                    write_epoch = false;
                    ++post_filter_dropped_epochs;
                } else {
                    kinematic_post_filter.accept(solution);
                }
            }
            if (write_epoch) {
                writer.writeEpoch(solution, epoch_differential_age_s);
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
                : readRoverUbx(rover_obs);
            if (rover_ok && rover_input_kind == RoverInputKind::RTCM) {
                applyEvents(rover_events);
                normalizeObservationWeek(rover_obs);
            } else if (rover_ok) {
                normalizeObservationWeek(rover_obs);
            }
        }

        // Stop and join the reader threads (EOF, stall, max-epochs or an ended source): a blocked
        // read is aborted by RTCMReader::requestAbort() / the pump's stop flag, never left detached.
        if (base_source.pump) {
            base_source.pump->stop();
        }
        if (rover_rtcm_source.pump) {
            rover_rtcm_source.pump->stop();
        }
        if (rover_ubx_source.pump) {
            rover_ubx_source.pump->stop();
        }

        const bool rover_stalled = !rover_ok && (rover_input_kind == RoverInputKind::RTCM
            ? sourceLastReadStalled(rover_rtcm_source)
            : rover_ubx_source.last_read_stalled);
        const bool base_stalled = !base_ok && !base_waiting && sourceLastReadStalled(base_source);
        if (termination_reason == "running") {
            if (rover_stalled) {
                termination_reason = "rover_source_stalled";
            } else if (base_stalled) {
                termination_reason = "base_source_stalled";
            } else if (!initial_rover_observation_ok) {
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

        // Snapshot decoder statistics before the readers are closed: RTCMReader::close()
        // resets its counters, which used to make every *_decoder_errors read 0.
        const DecoderSummaryStats rover_stats =
            rover_input_kind == RoverInputKind::RTCM
                ? summarizeRtcmSource(rover_rtcm_source)
                : summarizeUbxSource(rover_ubx_source);
        const DecoderSummaryStats base_stats = summarizeRtcmSource(base_source);
        const int rover_reconnects = rover_rtcm_source.reader.reconnectCount();
        const int base_reconnects = base_source.reader.reconnectCount();
        const int rover_stalls = rover_input_kind == RoverInputKind::RTCM
            ? rover_rtcm_source.reader.stallCount()
            : rover_ubx_source.stalls;
        const int base_stalls = base_source.reader.stallCount();

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
                  << " mode=" << modeChoiceString(config.mode)
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
                  << " format=" << outputFormatString(config.output_format);
        if (config.kinematic_post_filter) {
            std::cout << " post_filter_dropped_epochs=" << post_filter_dropped_epochs;
        }
        if (config.reconnect) {
            std::cout << " rover_reconnects=" << rover_reconnects
                      << " base_reconnects=" << base_reconnects;
        }
        if (config.stall_timeout_ms > 0) {
            std::cout << " rover_stalls=" << rover_stalls
                      << " base_stalls=" << base_stalls;
        }
        if (config.base_outage_fallback) {
            std::cout << " fallback_spp_epochs=" << fallback_spp_epochs;
        }
        if (config.ubx_nav) {
            std::cout << " ubx_nav_ephemerides=" << ubx_nav_merge.merged()
                      << " ubx_nav_klobuchar=" << (ubx_nav_decoder.hasKlobuchar() ? 1 : 0)
                      << " ubx_nav_unhealthy_skipped=" << ubx_nav_merge.unhealthySkipped()
                      << " ubx_nav_unconfirmed=" << ubx_nav_merge.unconfirmed()
                      << " ubx_nav_conflicts=" << ubx_nav_merge.conflicts()
                      << " ubx_nav_retracted=" << ubx_nav_merge.retracted()
                      << " ubx_nav_held_new_issue=" << ubx_nav_merge.heldNewIssue();
        }
        std::cout << "\n";
        return written_solutions > 0 ? 0 : 1;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
