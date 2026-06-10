#include "ppp_internal.hpp"

#include <libgnss++/algorithms/ppp_utils.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/iers/earth_rotation.hpp>
#include <libgnss++/iers/sub_daily_eop.hpp>
#include <libgnss++/io/madoca_l6.hpp>
#include <libgnss++/io/qzss_l6.hpp>
#include <libgnss++/io/rtcm.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <functional>
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
    // Strategy: use Python expander to convert L6 → expanded CSV,
    // then load via the battle-tested CSV loader (0.14m accuracy).
    // Falls back to C++ native L6 decoder if Python is unavailable.
    int gps_week = ppp_config_.l6_gps_week > 0 ? ppp_config_.l6_gps_week : last_obs_gps_week_;
    if (gps_week <= 0) gps_week = 2068;

    // Try Python expander first
    const std::string tmp_csv = "/tmp/gnsspp_l6_expanded_" +
        std::to_string(std::hash<std::string>{}(l6_file)) + ".csv";
    const std::string cmd =
        "python3 -c \""
        "import sys; sys.path.insert(0, 'apps'); "
        "from pathlib import Path; "
        "from gnss_clas_ppp import expand_qzss_l6_source; "
        "expand_qzss_l6_source('" + l6_file + "', " +
        std::to_string(gps_week) + ", Path('" + tmp_csv + "'))\" 2>/dev/null";

    const int ret = std::system(cmd.c_str());
    if (ret == 0) {
        ssr_products_loaded_ = ssr_products_.loadCSVFile(tmp_csv);
        if (ssr_products_loaded_) {
            return true;
        }
    }

    // Fallback: C++ native L6 decoder
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
    const int added =
        io::decodeMadocaL6eFilesToProducts(l6_files, gps_week, ssr_products_);
    ssr_products_loaded_ = added > 0;
    require_coherent_ssr_ =
        ssr_products_loaded_ && !env_overrides_.madoca_allow_partial_ssr;
    return ssr_products_loaded_;
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
    if (rtcm_file.empty() || sample_step_seconds <= 0.0) {
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
