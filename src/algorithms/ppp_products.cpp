#include "ppp_internal.hpp"

#include <libgnss++/algorithms/ppp_utils.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/signals.hpp>
#include <libgnss++/iers/earth_rotation.hpp>
#include <libgnss++/iers/sub_daily_eop.hpp>
#include <libgnss++/io/bds_ppp_b2b.hpp>
#include <libgnss++/io/galileo_has.hpp>
#include <libgnss++/io/madoca_l6.hpp>
#include <libgnss++/io/qzss_l6.hpp>
#include <libgnss++/io/rtcm.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace libgnss {

using namespace ppp_internal;

namespace {

GNSSTime normalizeGpsWeekTow(int week, double tow) {
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

GNSSTime alignSsrTimeToNavigationWeek(const NavigationData& nav,
                                      const SatelliteId& satellite,
                                      const GNSSTime& ssr_time) {
    const auto eph_it = nav.ephemeris_data.find(satellite);
    if (eph_it == nav.ephemeris_data.end() || eph_it->second.empty()) {
        return ssr_time;
    }
    const auto& ephemerides = eph_it->second;

    GNSSTime best_time = ssr_time;
    double best_age = std::numeric_limits<double>::infinity();
    for (const auto& eph : ephemerides) {
        int candidate_week = eph.toe.week != 0 ? eph.toe.week : static_cast<int>(eph.week);
        if (candidate_week == 0) {
            candidate_week = ssr_time.week;
        }
        GNSSTime candidate = normalizeGpsWeekTow(candidate_week, ssr_time.tow);
        const double dt = candidate - eph.toe;
        if (dt < -constants::SECONDS_PER_WEEK / 2.0) {
            candidate = normalizeGpsWeekTow(candidate.week + 1, candidate.tow);
        } else if (dt > constants::SECONDS_PER_WEEK / 2.0) {
            candidate = normalizeGpsWeekTow(candidate.week - 1, candidate.tow);
        }

        const double age = std::abs(candidate - eph.toe);
        if (age < best_age) {
            best_age = age;
            best_time = candidate;
        }
    }
    return best_time;
}

void mergeRtcmSsrCorrection(const io::RTCMSSRCorrection& input,
                            io::RTCMSSRCorrection& merged) {
    merged.satellite = input.satellite;
    merged.time = input.time;
    merged.update_interval_seconds = input.update_interval_seconds;
    merged.issue_of_data = input.issue_of_data;
    merged.provider_id = input.provider_id;
    merged.solution_id = input.solution_id;
    merged.reference_datum = input.reference_datum;
    if (input.iode >= 0) {
        merged.iode = input.iode;
    }
    if (input.iodcrc >= 0) {
        merged.iodcrc = input.iodcrc;
    }
    if (input.has_orbit) {
        merged.orbit_delta_rac_m = input.orbit_delta_rac_m;
        merged.orbit_rate_rac_mps = input.orbit_rate_rac_mps;
        merged.has_orbit = true;
    }
    if (input.has_clock) {
        merged.clock_delta_poly = input.clock_delta_poly;
        merged.has_clock = true;
    }
    if (input.has_code_bias) {
        for (const auto& [signal_id, bias_m] : input.code_bias_m) {
            merged.code_bias_m[signal_id] = bias_m;
        }
        merged.has_code_bias = !merged.code_bias_m.empty();
    }
    if (input.has_ura) {
        merged.ura_index = input.ura_index;
        merged.ura_sigma_m = input.ura_sigma_m;
        merged.has_ura = true;
    }
    if (input.has_high_rate_clock) {
        merged.high_rate_clock_m = input.high_rate_clock_m;
        merged.has_high_rate_clock = true;
    }
}

// Convert RTCM SSR code biases (wire signal IDs, RTCM sign: added to the
// pseudorange) to the PPP bias map (internal rtcmSsrSignalId keys, subtracted
// from the pseudorange on the non-MADOCA SSR path). When several wire IDs map
// to one libgnss++ signal, the preferred tracking mode wins.
std::map<uint8_t, double> rtcmCodeBiasesToInternal(
    GNSSSystem system,
    const std::map<uint8_t, double>& rtcm_code_bias_m) {
    std::map<uint8_t, std::pair<int, double>> ranked;
    for (const auto& [wire_id, bias_m] : rtcm_code_bias_m) {
        int rank = 0;
        const SignalType signal = signalTypeFromRtcmSsrSignalId(system, wire_id, &rank);
        if (signal == SignalType::SIGNAL_TYPE_COUNT) {
            continue;
        }
        const uint8_t key = rtcmSsrSignalId(system, signal);
        if (key == 0U) {
            continue;
        }
        const auto it = ranked.find(key);
        if (it == ranked.end() || rank < it->second.first) {
            ranked[key] = {rank, -bias_m};
        }
    }
    std::map<uint8_t, double> internal;
    for (const auto& [key, value] : ranked) {
        internal[key] = value.second;
    }
    return internal;
}

// An IODE-tagged orbit/clock update held until the next update of the same
// satellite (bounded by max_hold_seconds), with the code biases in effect.
struct HeldSsrUpdate {
    io::RTCMSSRCorrection correction;
    std::map<uint8_t, double> code_bias_m;
    double max_hold_seconds = PPPProcessor::kHasIddMaxHoldSeconds;
    // Seconds after correction.time from which a receiver has the update
    // (--realtime: its reception, or --ssr-latency); until then the previous
    // update stays in use. 0: from correction.time (post-processing).
    double availability_delay_s = 0.0;
};

// Orbit correction frame of the held updates: RTCM (along = velocity) or
// BDS PPP-B2b (radial = position).
enum class SsrRacFrame { Rtcm, BdsB2b };

// mark_hold_end: where a hold ends before the next update (or with none),
// add an invalid sample there, since SSRProducts otherwise keeps returning
// the last sample of the satellite.
size_t sampleHeldSsrUpdates(std::map<SatelliteId, std::vector<HeldSsrUpdate>>& updates,
                            const NavigationData& nav,
                            double sample_step_seconds,
                            SSRProducts& products,
                            SsrRacFrame frame = SsrRacFrame::Rtcm,
                            bool mark_hold_end = false) {
    // Sample on the step grid so PPP epochs hit an exact sample and never
    // interpolate across an IODE change.
    size_t sampled_corrections = 0;
    for (auto&[satellite, satellite_updates] : updates) {
        // In the order a receiver has them (the reference epoch, plus the
        // reception delay with --realtime): each update holds until the next
        // one is in hand. B2b reception delays vary (6-29 s on the Kamakura
        // hours), so an update can arrive after one with a later epoch.
        const auto available = [](const HeldSsrUpdate& update) {
            return update.correction.time + update.availability_delay_s;
        };
        std::stable_sort(satellite_updates.begin(), satellite_updates.end(),
                         [&](const HeldSsrUpdate& lhs, const HeldSsrUpdate& rhs) {
                             return available(lhs) < available(rhs);
                         });
        for (size_t index = 0; index < satellite_updates.size(); ++index) {
            const auto& update = satellite_updates[index];
            const io::RTCMSSRCorrection& correction = update.correction;
            double hold_seconds = update.max_hold_seconds;
            bool hold_ends_early = true;
            if (index + 1 < satellite_updates.size()) {
                // The next update takes over once a receiver has it.
                const double to_next = available(satellite_updates[index + 1]) - correction.time;
                hold_ends_early = to_next > hold_seconds + 1e-6;
                hold_seconds = std::min(hold_seconds, to_next);
            }
            if (mark_hold_end && hold_ends_early) {
                SSROrbitClockCorrection end;
                end.satellite = satellite;
                end.time = correction.time + hold_seconds;
                end.iode = -1;
                end.orbit_valid = false;
                end.clock_valid = false;
                end.hold_end = true;
                products.addCorrection(end);
            }
            if (hold_seconds <= 0.0) {
                continue;
            }
            const int sample_count = std::max(
                1, static_cast<int>(std::ceil(hold_seconds / sample_step_seconds - 1e-9)));
            // First sample on the grid at or after the update is available.
            const int first_sample = std::max(
                0, static_cast<int>(std::ceil(update.availability_delay_s / sample_step_seconds - 1e-9)));
            for (int sample_index = first_sample; sample_index < sample_count; ++sample_index) {
                const double dt = sample_step_seconds * static_cast<double>(sample_index);
                const GNSSTime sample_time = correction.time + dt;
                Vector3d position = Vector3d::Zero();
                Vector3d velocity = Vector3d::Zero();
                double clock_bias = 0.0;
                double clock_drift = 0.0;
                // The RAC frame only needs the orbit geometry; fall back to
                // the default ephemeris when the referenced IODE is not
                // loaded. PPP still rejects the satellite at that epoch because
                // it requires the IODE-matched broadcast record.
                if (!nav.calculateSatelliteState(satellite, sample_time, position, velocity,
                                                 clock_bias, clock_drift, correction.iode) &&
                    !nav.calculateSatelliteState(satellite, sample_time, position, velocity,
                                                 clock_bias, clock_drift)) {
                    continue;
                }

                SSROrbitClockCorrection sampled;
                sampled.satellite = satellite;
                sampled.time = sample_time;
                sampled.orbit_reference_time = correction.time;
                sampled.clock_reference_time = correction.time;
                sampled.iode = correction.iode;
                sampled.ssr_orbit_iod = correction.issue_of_data;
                sampled.ssr_clock_iod = correction.issue_of_data;
                const Vector3d rac =
                    correction.orbit_delta_rac_m + correction.orbit_rate_rac_mps * dt;
                sampled.orbit_correction_ecef =
                    frame == SsrRacFrame::BdsB2b
                        ? ppp_utils::b2bRacToEcef(position, velocity, rac)
                        : ssrRacToEcef(position, velocity, rac);
                sampled.clock_correction_m =
                    correction.clock_delta_poly.x() +
                    correction.clock_delta_poly.y() * dt +
                    correction.clock_delta_poly.z() * dt * dt;
                if (correction.has_high_rate_clock) {
                    sampled.clock_correction_m += correction.high_rate_clock_m;
                }
                if (correction.has_ura) {
                    sampled.ura_sigma_m = correction.ura_sigma_m;
                    sampled.ura_valid = true;
                }
                if (!update.code_bias_m.empty()) {
                    sampled.code_bias_m = update.code_bias_m;
                    sampled.code_bias_valid = true;
                }
                sampled.orbit_valid = true;
                sampled.clock_valid = true;
                products.addCorrection(sampled);
                ++sampled_corrections;
            }
        }
    }
    return sampled_corrections;
}

}  // namespace

bool PPPProcessor::loadPreciseProducts(const std::string& orbit_file, const std::string& clock_file) {
    precise_products_.clear();

    bool loaded_any = false;
    if (!orbit_file.empty()) {
        loaded_any = precise_products_.loadSP3File(orbit_file) || loaded_any;
    }
    if (!clock_file.empty()) {
        loaded_any = precise_products_.loadClockFile(clock_file) || loaded_any;
    }
    precise_products_loaded_ = loaded_any;
    return precise_products_loaded_;
}

bool PPPProcessor::loadSSRProducts(const std::string& ssr_file) {
    ssr_products_.clear();
    require_coherent_ssr_ = false;
    if (ssr_file.empty()) {
        ssr_products_loaded_ = false;
        return false;
    }
    // Auto-detect L6 binary vs CSV based on file content
    if (ssr_file.size() >= 4) {
        std::ifstream probe(ssr_file, std::ios::binary);
        uint32_t magic = 0;
        if (probe.read(reinterpret_cast<char*>(&magic), 4)) {
            // L6 preamble in big-endian: 0x1ACFFC1D
            const uint32_t preamble =
                ((magic & 0xFF) << 24) | (((magic >> 8) & 0xFF) << 16) |
                (((magic >> 16) & 0xFF) << 8) | ((magic >> 24) & 0xFF);
            if (preamble == 0x1ACFFC1D) {
                return loadL6Products(ssr_file);
            }
        }
    }
    ssr_products_loaded_ = ssr_products_.loadCSVFile(ssr_file);
    return ssr_products_loaded_;
}

bool PPPProcessor::loadL6Products(const std::string& l6_file) {
    require_coherent_ssr_ = false;
    // Raw L6 given as --ssr: decode Compact SSR natively. The policy-rich
    // Python expansion of `gnss clas-ppp` (apps/commands/positioning/
    // gnss_clas_ppp.py) passes its expanded CSV to --ssr instead.
    int gps_week = ppp_config_.l6_gps_week > 0 ? ppp_config_.l6_gps_week : last_obs_gps_week_;
    if (gps_week <= 0) gps_week = 2068;

    qzss_l6::L6Decoder decoder;
    auto epochs = decoder.decodeFile(l6_file, gps_week);
    if (epochs.empty()) {
        ssr_products_loaded_ = false;
        return false;
    }
    int best_network = 0;
    if (ppp_config_.approximate_position.norm() > 1e3) {
        double lat = 0, lon = 0, h = 0;
        ecef2geodetic(ppp_config_.approximate_position, lat, lon, h);
        lat *= 180.0 / M_PI;
        lon *= 180.0 / M_PI;
        if (lat > 34 && lat < 38 && lon > 138 && lon < 142)
            best_network = 7;
    }
    qzss_l6::populateSSRProducts(epochs, ssr_products_, best_network);
    ssr_products_loaded_ = true;
    return true;
}

bool PPPProcessor::loadMadocaL6Products(const std::vector<std::string>& l6_files) {
    ssr_products_.clear();
    require_coherent_ssr_ = false;
    if (l6_files.empty()) {
        ssr_products_loaded_ = false;
        return false;
    }
    const int gps_week = ppp_config_.l6_gps_week > 0 ? ppp_config_.l6_gps_week
                                                     : last_obs_gps_week_;
    // --realtime: the L6 log holds no reception time; --ssr-latency dates
    // each correction when a receiver has it.
    const double availability_delay_s =
        ppp_config_.causal ? std::max(0.0, ppp_config_.ssr_latency_s) : 0.0;
    const int added = io::decodeMadocaL6eFilesToProducts(
        l6_files, gps_week, ssr_products_, availability_delay_s);
    ssr_products_loaded_ = added > 0;
    require_coherent_ssr_ =
        ssr_products_loaded_ && !env_overrides_.madoca_allow_partial_ssr;
    return ssr_products_loaded_;
}

io::MadocaL6dLoadStatus PPPProcessor::loadMadocaL6dProducts(
    const std::vector<std::string>& l6d_files,
    const double reference_epoch[6],
    const double receiver_ecef[3]) {
    madoca_iono_products_.clear();
    if (l6d_files.empty()) {
        return io::MadocaL6dLoadStatus::kFailed;
    }
    for (const auto& file : l6d_files) {
        std::vector<io::MadocaIonoSnapshot> snapshots;
        if (!io::decodeMadocaL6dFileToSnapshots(
                file, reference_epoch, receiver_ecef, snapshots)) {
            madoca_iono_products_.clear();
            return io::MadocaL6dLoadStatus::kFailed;
        }
        madoca_iono_products_.addSnapshots(snapshots);
    }
    const auto status = io::classifyMadocaL6dLoad(true, madoca_iono_products_.size());
    if (status == io::MadocaL6dLoadStatus::kOutsideCoverage) {
        ppp_config_.apply_madoca_l6d_ionosphere = false;
    }
    return status;
}

bool PPPProcessor::loadIONEXProducts(const std::string& ionex_file) {
    ionex_products_.clear();
    if (ionex_file.empty()) {
        ionex_products_loaded_ = false;
        return false;
    }
    ionex_products_loaded_ = ionex_products_.loadIONEXFile(ionex_file);
    return ionex_products_loaded_;
}

bool PPPProcessor::loadDCBProducts(const std::string& dcb_file) {
    dcb_products_.clear();
    if (dcb_file.empty()) {
        dcb_products_loaded_ = false;
        return false;
    }
    dcb_products_loaded_ = dcb_products_.loadFile(dcb_file);
    return dcb_products_loaded_;
}

bool PPPProcessor::loadEopC04(const std::string& path) {
    eop_table_.reset();
    if (path.empty()) {
        return false;
    }
    try {
        // Auto-detect IERS C04 vs Bulletin A. Both formats are
        // accepted on the same `--eop-c04` CLI knob; the discriminator
        // is the `I`/`P` flag character at fixed columns in Bulletin
        // A (C04 has numeric data in those columns).
        eop_table_ = std::make_unique<libgnss::iers::EopTable>(
            libgnss::iers::EopTable::fromFile(path));
    } catch (const std::exception&) {
        eop_table_.reset();
        return false;
    }
    return true;
}

bool PPPProcessor::loadAtmosphericTidalLoading(const std::string& path) {
    atm_tidal_loading_coefficients_ =
        libgnss::iers::AtmosphericTidalLoadingCoefficients{};
    atm_tidal_loading_loaded_ = false;
    if (path.empty()) {
        return false;
    }
    std::ifstream in(path);
    if (!in) {
        return false;
    }
    bool have_s1 = false;
    bool have_s2 = false;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line.front() == '$' || line.front() == '#') {
            continue;
        }
        std::istringstream iss(line);
        std::string tag;
        iss >> tag;
        std::size_t idx = 0;
        if (tag == "S1") {
            idx = 0;
        } else if (tag == "S2") {
            idx = 1;
        } else {
            // Anything that is not a comment and not an S1/S2 tag is
            // assumed to be the station-name line; ignore.
            continue;
        }
        double r_amp = 0.0, w_amp = 0.0, s_amp = 0.0;
        double r_pha = 0.0, w_pha = 0.0, s_pha = 0.0;
        if (!(iss >> r_amp >> w_amp >> s_amp >> r_pha >> w_pha >> s_pha)) {
            return false;
        }
        atm_tidal_loading_coefficients_.radial_amplitudes_m[idx] = r_amp;
        atm_tidal_loading_coefficients_.west_amplitudes_m[idx]   = w_amp;
        atm_tidal_loading_coefficients_.south_amplitudes_m[idx]  = s_amp;
        atm_tidal_loading_coefficients_.radial_phases_deg[idx]   = r_pha;
        atm_tidal_loading_coefficients_.west_phases_deg[idx]     = w_pha;
        atm_tidal_loading_coefficients_.south_phases_deg[idx]    = s_pha;
        if (idx == 0) have_s1 = true; else have_s2 = true;
    }
    atm_tidal_loading_loaded_ = have_s1 && have_s2;
    return atm_tidal_loading_loaded_;
}

libgnss::iers::EarthOrientationParams
PPPProcessor::getEarthOrientationParams(const GNSSTime& time) const {
    if (!eop_table_) {
        return libgnss::iers::EarthOrientationParams{};
    }
    const double mjd_utc = libgnss::iers::gnssTimeToMjdUtc(time);
    auto eop = eop_table_->interpolateAt(mjd_utc);
    if (ppp_config_.use_iers_sub_daily_eop) {
        const auto delta = libgnss::iers::subDailyEopCorrection(
            mjd_utc, eop.ut1_minus_utc_seconds);
        eop.xp_arcsec               += delta.dxp_arcsec;
        eop.yp_arcsec               += delta.dyp_arcsec;
        eop.ut1_minus_utc_seconds   += delta.dut1_seconds;
    }
    return eop;
}

bool PPPProcessor::loadRTCMSSRProducts(const std::string& rtcm_file,
                                       const NavigationData& nav,
                                       double sample_step_seconds) {
    ssr_products_.clear();
    require_coherent_ssr_ = false;
    // Legacy biases stay keyed by their wire signal ID.
    ssr_code_bias_tracking_identity_ = false;
    if (rtcm_file.empty() || sample_step_seconds <= 0.0) {
        ssr_products_loaded_ = false;
        return false;
    }
    // The legacy conversion interpolates each grid sample between the
    // updates around it, so its samples already hold later data.
    if (ppp_config_.causal) {
        ssr_products_loaded_ = false;
        return false;
    }

    io::RTCMReader reader;
    if (!reader.open(rtcm_file)) {
        ssr_products_loaded_ = false;
        return false;
    }

    io::RTCMProcessor rtcm_processor;
    std::map<SatelliteId, io::RTCMSSRCorrection> pending_corrections;
    io::RTCMMessage message;
    size_t sampled_corrections = 0;

    while (reader.readMessage(message)) {
        std::vector<io::RTCMSSRCorrection> decoded_corrections;
        if (!rtcm_processor.decodeSSRCorrections(message, decoded_corrections)) {
            continue;
        }

        for (const auto& correction : decoded_corrections) {
            io::RTCMSSRCorrection aligned_correction = correction;
            aligned_correction.time =
                alignSsrTimeToNavigationWeek(nav, aligned_correction.satellite, aligned_correction.time);

            auto& merged = pending_corrections[aligned_correction.satellite];
            const bool same_group =
                merged.satellite == aligned_correction.satellite &&
                std::abs(merged.time - aligned_correction.time) < 1e-6 &&
                merged.issue_of_data == aligned_correction.issue_of_data &&
                merged.provider_id == aligned_correction.provider_id &&
                merged.solution_id == aligned_correction.solution_id;
            if (!same_group) {
                merged = io::RTCMSSRCorrection{};
            }
            mergeRtcmSsrCorrection(aligned_correction, merged);
            if (!merged.has_orbit || !merged.has_clock) {
                continue;
            }

            Vector3d base_position = Vector3d::Zero();
            Vector3d base_velocity = Vector3d::Zero();
            double base_clock_bias = 0.0;
            double base_clock_drift = 0.0;
            if (!nav.calculateSatelliteState(
                    merged.satellite,
                    merged.time,
                    base_position,
                    base_velocity,
                    base_clock_bias,
                    base_clock_drift)) {
                continue;
            }

            const double update_interval =
                merged.update_interval_seconds > 0.0 ?
                    merged.update_interval_seconds :
                    sample_step_seconds;
            const int sample_count = std::max(
                1,
                static_cast<int>(std::floor(update_interval / sample_step_seconds + 1e-9)));
            for (int sample_index = 0; sample_index < sample_count; ++sample_index) {
                const double dt = sample_step_seconds * static_cast<double>(sample_index);
                const GNSSTime sample_time = merged.time + dt;
                Vector3d position = base_position;
                Vector3d velocity = base_velocity;
                double clock_bias = 0.0;
                double clock_drift = 0.0;
                if (!nav.calculateSatelliteState(
                        merged.satellite,
                        sample_time,
                        position,
                        velocity,
                        clock_bias,
                        clock_drift)) {
                    continue;
                }

                SSROrbitClockCorrection sampled;
                sampled.satellite = merged.satellite;
                sampled.time = sample_time;
                sampled.orbit_correction_ecef = ssrRacToEcef(
                    position,
                    velocity,
                    merged.orbit_delta_rac_m + merged.orbit_rate_rac_mps * dt);
                sampled.clock_correction_m =
                    merged.clock_delta_poly.x() +
                    merged.clock_delta_poly.y() * dt +
                    merged.clock_delta_poly.z() * dt * dt;
                if (merged.has_high_rate_clock) {
                    sampled.clock_correction_m += merged.high_rate_clock_m;
                }
                if (merged.has_ura) {
                    sampled.ura_sigma_m = merged.ura_sigma_m;
                    sampled.ura_valid = true;
                }
                if (merged.has_code_bias) {
                    sampled.code_bias_m = merged.code_bias_m;
                    sampled.code_bias_valid = !sampled.code_bias_m.empty();
                }
                sampled.orbit_valid = true;
                sampled.clock_valid = true;
                ssr_products_.addCorrection(sampled);
                ++sampled_corrections;
            }
        }
    }

    ssr_products_loaded_ = sampled_corrections > 0U;
    return ssr_products_loaded_;
}

bool PPPProcessor::loadRTCMSSRProducts(const std::string& rtcm_file,
                                       const NavigationData& nav,
                                       double sample_step_seconds,
                                       RTCMSSRProfile profile) {
    if (profile == RTCMSSRProfile::Legacy) {
        return loadRTCMSSRProducts(rtcm_file, nav, sample_step_seconds);
    }

    ssr_products_.clear();
    require_coherent_ssr_ = false;
    ssr_products_loaded_ = false;
    ssr_held_samples_ = true;
    // A satellite the stream does not correct at an epoch (another
    // constellation, or a gap in its corrections) must not fall back to the
    // broadcast orbit and clock: on the OBE4 HAS IDD hour G26 has no
    // correction for 29 minutes and its broadcast clock is 0.3 m off the
    // corrected ones. As for HAS SIS and PPP-B2b, and as cssrlib does.
    require_ssr_orbit_correction_ = true;
    // The biases are keyed by signal (rtcmCodeBiasesToInternal): give a GPS
    // L2 P(Y) / Z-tracked code (C2W, C2P, ...) the L2 P bias and an L2C code
    // the L2C one, as the HAS SIS path does. HAS sends both L2 CL and L2 P;
    // C2W took the L2 CL one, 0.5-0.9 m off ionosphere-free on the OBE4 hour.
    ssr_code_bias_tracking_identity_ = true;
    ssr_first_available_ = GNSSTime();
    ssr_satellite_first_available_.clear();
    if (rtcm_file.empty() || sample_step_seconds <= 0.0) {
        return false;
    }
    io::RTCMReader reader;
    if (!reader.open(rtcm_file)) {
        return false;
    }

    // Pass 1: decode the stream into per-satellite orbit/clock updates, each
    // tagged with the code biases held at that point of the stream. HAS IDD
    // code-bias messages carry a fixed, stale epoch, so biases are held by
    // stream order rather than merged by epoch.
    io::RTCMProcessor rtcm_processor;
    std::map<SatelliteId, io::RTCMSSRCorrection> pending_corrections;
    std::map<SatelliteId, std::map<uint8_t, double>> held_code_bias;
    std::map<SatelliteId, std::vector<HeldSsrUpdate>> updates;
    io::RTCMMessage message;
    while (reader.readMessage(message)) {
        std::vector<io::RTCMSSRCorrection> decoded_corrections;
        if (!rtcm_processor.decodeSSRCorrections(message, decoded_corrections)) {
            continue;
        }
        for (const auto& correction : decoded_corrections) {
            if (correction.has_code_bias) {
                held_code_bias[correction.satellite] = rtcmCodeBiasesToInternal(
                    correction.satellite.system, correction.code_bias_m);
            }
            if (!correction.has_orbit && !correction.has_clock &&
                !correction.has_ura && !correction.has_high_rate_clock) {
                continue;
            }
            io::RTCMSSRCorrection aligned_correction = correction;
            aligned_correction.has_code_bias = false;
            aligned_correction.code_bias_m.clear();
            aligned_correction.time =
                alignSsrTimeToNavigationWeek(nav, aligned_correction.satellite, aligned_correction.time);

            auto& merged = pending_corrections[aligned_correction.satellite];
            const bool same_group =
                merged.satellite == aligned_correction.satellite &&
                std::abs(merged.time - aligned_correction.time) < 1e-6 &&
                merged.issue_of_data == aligned_correction.issue_of_data &&
                merged.provider_id == aligned_correction.provider_id &&
                merged.solution_id == aligned_correction.solution_id;
            if (!same_group) {
                merged = io::RTCMSSRCorrection{};
            }
            mergeRtcmSsrCorrection(aligned_correction, merged);
            if (!merged.has_orbit || !merged.has_clock) {
                continue;
            }
            auto& satellite_updates = updates[merged.satellite];
            const auto bias_it = held_code_bias.find(merged.satellite);
            HeldSsrUpdate update;
            update.correction = merged;
            if (ppp_config_.causal) {
                // --realtime: the stream does not record its reception.
                update.availability_delay_s = std::max(0.0, ppp_config_.ssr_latency_s);
            }
            if (bias_it != held_code_bias.end()) {
                update.code_bias_m = bias_it->second;
            }
            if (!satellite_updates.empty() &&
                std::abs(satellite_updates.back().correction.time - merged.time) < 1e-6) {
                satellite_updates.back() = std::move(update);
            } else {
                satellite_updates.push_back(std::move(update));
            }
        }
    }

    // Pass 2: hold each update until the next one for the satellite (bounded
    // by kHasIddMaxHoldSeconds), sampling on the step grid so PPP epochs hit
    // an exact sample and never interpolate across an IODE change.
    const size_t sampled_corrections = sampleHeldSsrUpdates(
        updates, nav, sample_step_seconds, ssr_products_, SsrRacFrame::Rtcm, true);
    ssr_products_loaded_ = sampled_corrections > 0U;
    if (ppp_config_.causal) {
        // A real-time receiver starts SSR PPP once it has a correction:
        // solving the first epochs on broadcast orbits and then switching
        // every satellite to its correction would jump the filter.
        // The same holds for a satellite whose first correction comes later
        // than the stream's: it waits for it.
        for (const auto& [satellite, satellite_updates] : updates) {
            for (const auto& update : satellite_updates) {
                const GNSSTime available =
                    update.correction.time + update.availability_delay_s;
                if (ssr_first_available_.week == 0 || available < ssr_first_available_) {
                    ssr_first_available_ = available;
                }
                const auto [it, inserted] =
                    ssr_satellite_first_available_.emplace(satellite, available);
                if (!inserted && available < it->second) {
                    it->second = available;
                }
            }
        }
    }
    return ssr_products_loaded_;
}

bool PPPProcessor::waitsForFirstSsrUpdate(const SatelliteId& satellite,
                                          const GNSSTime& time) const {
    const auto it = ssr_satellite_first_available_.find(satellite);
    return it != ssr_satellite_first_available_.end() && time < it->second;
}

bool PPPProcessor::loadGalileoHasSisProducts(const std::vector<io::HasSsrUpdate>& has_updates,
                                             const NavigationData& nav,
                                             double sample_step_seconds) {
    ssr_products_.clear();
    require_coherent_ssr_ = false;
    ssr_products_loaded_ = false;
    // HAS covers GPS and Galileo only: other constellations (and satellites
    // whose HAS validity expired) must not fall back to the broadcast orbit.
    require_ssr_orbit_correction_ = true;
    ssr_held_samples_ = true;
    // HAS carries both L2 CL and L2 P code biases; apply the one of the
    // tracked GPS L2 code.
    ssr_code_bias_tracking_identity_ = true;
    if (sample_step_seconds <= 0.0) {
        return false;
    }
    // HAS SIS updates already carry their IODref, the clock / orbit pairing
    // of their IOD Set ID and the end of their validity interval; convert them
    // to RTCM SSR storage (orbit negated, RTCM code-bias signal IDs) and hold
    // them exactly like the HAS IDD samples.
    std::map<SatelliteId, std::vector<HeldSsrUpdate>> updates;
    for (const auto& has_update : has_updates) {
        const double validity_s = has_update.valid_until - has_update.time;
        if (!(validity_s > 0.0)) {
            continue;
        }
        HeldSsrUpdate update;
        update.correction = io::hasUpdateToRtcmSsrCorrection(has_update);
        if (ppp_config_.causal && has_update.received.week > 0) {
            // --realtime: from the reception of its last page on.
            update.availability_delay_s = std::max(0.0, has_update.received - has_update.time);
        }
        update.code_bias_m = rtcmCodeBiasesToInternal(has_update.satellite.system,
                                                      update.correction.code_bias_m);
        update.correction.has_code_bias = false;
        update.correction.code_bias_m.clear();
        update.max_hold_seconds = validity_s;
        auto& satellite_updates = updates[has_update.satellite];
        if (!satellite_updates.empty() &&
            std::abs(satellite_updates.back().correction.time - update.correction.time) < 1e-6) {
            satellite_updates.back() = std::move(update);
        } else {
            satellite_updates.push_back(std::move(update));
        }
    }
    const size_t sampled_corrections = sampleHeldSsrUpdates(
        updates, nav, sample_step_seconds, ssr_products_, SsrRacFrame::Rtcm, true);
    ssr_products_loaded_ = sampled_corrections > 0U;
    return ssr_products_loaded_;
}

namespace {

// B2b code-bias signal index (ICD Table 6-6) -> SignalType and preference
// rank (lower kept when several tracking modes share a SignalType). Signals
// without an internal bias key (BDS B1C, B2a, B2b; GPS L1C) are left out.
std::pair<SignalType, int> b2bCodeBiasSignal(GNSSSystem system, int index) {
    if (system == GNSSSystem::BeiDou) {
        switch (index) {
            case 0: return {SignalType::BDS_B1I, 0};
            case 12: return {SignalType::BDS_B3I, 0};
            default: break;
        }
    } else if (system == GNSSSystem::GPS) {
        switch (index) {
            case 0: return {SignalType::GPS_L1CA, 0};
            case 1: return {SignalType::GPS_L1P, 0};
            case 7: return {SignalType::GPS_L2C, 0};   // L2C (L)
            case 8: return {SignalType::GPS_L2C, 1};   // L2C (M+L)
            case 12: return {SignalType::GPS_L5, 0};   // L5 Q
            case 13: return {SignalType::GPS_L5, 1};   // L5 I+Q
            case 11: return {SignalType::GPS_L5, 2};   // L5 I
            default: break;
        }
    }
    return {SignalType::SIGNAL_TYPE_COUNT, 0};
}

}  // namespace

bool PPPProcessor::loadBdsB2bProducts(const std::vector<io::BdsB2bUpdate>& b2b_updates,
                                      const NavigationData& nav,
                                      double sample_step_seconds) {
    ssr_products_.clear();
    require_coherent_ssr_ = false;
    ssr_products_loaded_ = false;
    // B2b corrects GPS and BDS-3 only; the others must not fall back to the
    // broadcast orbit.
    require_ssr_orbit_correction_ = true;
    ssr_code_bias_tracking_identity_ = false;
    bds_b2b_corrections_ = true;
    ssr_held_samples_ = true;
    if (sample_step_seconds <= 0.0) {
        return false;
    }
    std::map<SatelliteId, std::vector<HeldSsrUpdate>> updates;
    for (const auto& b2b : b2b_updates) {
        const GNSSSystem system = b2b.satellite.system;
        if (system != GNSSSystem::GPS && system != GNSSSystem::BeiDou) {
            continue;
        }
        // --realtime: an update (or its withdrawal) applies from the
        // reception of the message that completed it.
        const bool use_reception = ppp_config_.causal && b2b.received.week > 0;
        if (!b2b.usable) {
            // No valid orbit / clock pair from here on: stop holding the
            // previous one (cssrlib drops the satellite the same way).
            auto it = updates.find(b2b.satellite);
            if (it != updates.end() && !it->second.empty()) {
                HeldSsrUpdate& previous = it->second.back();
                const GNSSTime withdrawn = use_reception ? b2b.received : b2b.time;
                previous.max_hold_seconds = std::min(
                    previous.max_hold_seconds,
                    std::max(0.0, withdrawn - previous.correction.time));
            }
            continue;
        }
        HeldSsrUpdate update;
        io::RTCMSSRCorrection& correction = update.correction;
        correction.satellite = b2b.satellite;
        correction.time = b2b.time;
        correction.issue_of_data = static_cast<uint8_t>(b2b.iod_corr);
        correction.iode = b2b.iode;
        correction.has_orbit = true;
        correction.has_clock = true;
        // Both subtracted, as in RTCM SSR: RTCM adds the clock term, B2b
        // subtracts C0.
        correction.orbit_delta_rac_m = Vector3d(b2b.radial_m, b2b.along_m, b2b.cross_m);
        correction.clock_delta_poly = Vector3d(-b2b.c0_m, 0.0, 0.0);
        if (b2b.ura_m) {
            correction.has_ura = true;
            correction.ura_sigma_m = *b2b.ura_m;
        }
        // Internal bias keys hold the term PPP subtracts from the code, the
        // B2b sign.
        std::map<uint8_t, std::pair<int, double>> ranked;
        for (const auto& [index, bias_m] : b2b.code_bias_m) {
            const auto [signal, rank] = b2bCodeBiasSignal(system, index);
            if (signal == SignalType::SIGNAL_TYPE_COUNT) {
                continue;
            }
            const uint8_t key = rtcmSsrSignalId(system, signal);
            const auto it = ranked.find(key);
            if (key != 0U && (it == ranked.end() || rank < it->second.first)) {
                ranked[key] = {rank, bias_m};
            }
        }
        for (const auto& [key, value] : ranked) {
            update.code_bias_m[key] = value.second;
        }
        update.max_hold_seconds = kBdsB2bMaxHoldSeconds;
        if (use_reception) {
            update.availability_delay_s = std::max(0.0, b2b.received - b2b.time);
        }
        auto& satellite_updates = updates[b2b.satellite];
        if (!satellite_updates.empty() &&
            std::abs(satellite_updates.back().correction.time - correction.time) < 1e-6) {
            satellite_updates.back() = std::move(update);
        } else {
            satellite_updates.push_back(std::move(update));
        }
    }
    const size_t sampled_corrections = sampleHeldSsrUpdates(
        updates, nav, sample_step_seconds, ssr_products_, SsrRacFrame::BdsB2b, true);
    ssr_products_loaded_ = sampled_corrections > 0U;
    return ssr_products_loaded_;
}

bool PPPProcessor::interpolateLoadedSSRCorrection(const SatelliteId& sat,
                                                  const GNSSTime& time,
                                                  Vector3d& orbit_correction_ecef,
                                                  double& clock_correction_m,
                                                  double* ura_sigma_m,
                                                  std::map<uint8_t, double>* code_bias_m,
                                                  std::map<uint8_t, double>* phase_bias_m,
                                                  std::map<std::string, std::string>* atmos_tokens) const {
    if (!ssr_products_loaded_) {
        orbit_correction_ecef.setZero();
        clock_correction_m = 0.0;
        if (ura_sigma_m != nullptr) {
            *ura_sigma_m = 0.0;
        }
        if (code_bias_m != nullptr) {
            code_bias_m->clear();
        }
        if (phase_bias_m != nullptr) {
            phase_bias_m->clear();
        }
        if (atmos_tokens != nullptr) {
            atmos_tokens->clear();
        }
        return false;
    }
    return ssr_products_.interpolateCorrection(
        sat,
        time,
        orbit_correction_ecef,
        clock_correction_m,
        ura_sigma_m,
        code_bias_m,
        phase_bias_m,
        atmos_tokens);
}

}  // namespace libgnss
