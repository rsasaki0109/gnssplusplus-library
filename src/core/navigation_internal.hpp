#pragma once

// Shared file-local helpers for the navigation/products implementation
// TUs. Extracted from the former monolithic navigation.cpp anonymous
// namespace; every function is inline and internal to the
// navigation_internal namespace.

#include <libgnss++/core/navigation.hpp>
#include <libgnss++/algorithms/ppp_env_overrides.hpp>
#include <algorithm>
#include <cmath>
#include <cctype>
#include <ctime>
#include <iostream>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace libgnss {
namespace navigation_internal {


constexpr double kGpsWeekSeconds = 604800.0;
constexpr double kHalfGpsWeekSeconds = 302400.0;
constexpr double kEarthRotationRate = 7.2921151467e-5;
constexpr double kEarthMu = 3.986005e14;
constexpr double kGalileoEarthMu = 3.986004418e14;
constexpr double kGlonassEarthMu = 3.9860044e14;
constexpr double kGlonassJ2 = 1.0826257e-3;
constexpr double kGlonassEarthRadius = 6378136.0;
constexpr double kGlonassEarthRotationRate = 7.292115e-5;
constexpr double kGlonassIntegrationStep = 60.0;
constexpr double kBeiDouEarthRotationRate = 7.292115e-5;
constexpr double kBeiDouEarthMu = 3.986004418e14;
constexpr double kRelativityF = -4.442807633e-10;
constexpr double kWgs84A = 6378137.0;
constexpr double kWgs84E2 = 6.69437999014e-3;
constexpr double kBeiDouGeoSin5Deg = -0.0871557427476582;
constexpr double kBeiDouGeoCos5Deg = 0.9961946980917456;
constexpr double kPreciseInterpolationGapSeconds = 900.0;
inline // CLASLIB STECVALIDAGE (cssr.h:79) — compact regional STEC held across this window.
constexpr double kClasStecValidAgeSeconds = 3600.0;

inline bool isQzssDedicatedBiasBankEntry(const SSROrbitClockCorrection& entry, bool phase) {
    const bool valid =
        phase ? (entry.phase_bias_valid && !entry.phase_bias_m.empty())
              : (entry.code_bias_valid && !entry.code_bias_m.empty());
    if (!valid) {
        return false;
    }
    if (phase) {
        if (entry.bias_network_id <= 0) {
            return false;
        }
        for (const auto& [signal_id, bias_m] : entry.phase_bias_m) {
            if (std::abs(bias_m) > 1e-12) {
                return true;
            }
        }
        return false;
    }
    return entry.orbit_valid || entry.clock_valid;
}

inline const SSROrbitClockCorrection* findQzssHeldPhaseBiasForServiceNetwork(
    const std::vector<SSROrbitClockCorrection>& entries,
    const GNSSTime& time,
    int service_network_id,
    size_t scan_end_index) {
    if (service_network_id <= 0 || scan_end_index == 0) {
        return nullptr;
    }
    size_t group_end = scan_end_index;
    while (group_end > 0) {
        const size_t group_last = group_end - 1;
        const GNSSTime group_time = entries[group_last].time;
        if (time - group_time > kClasStecValidAgeSeconds + 1e-9) {
            break;
        }
        size_t group_begin = group_last;
        while (group_begin > 0 && entries[group_begin - 1].time == group_time) {
            --group_begin;
        }
        for (size_t index = group_begin; index < group_end; ++index) {
            const SSROrbitClockCorrection& entry = entries[index];
            if (!isQzssDedicatedBiasBankEntry(entry, true)) {
                continue;
            }
            if (entry.bias_network_id == service_network_id) {
                return &entry;
            }
        }
        group_end = group_begin;
    }
    return nullptr;
}

inline std::string trimCopy(const std::string& value) {
    const auto begin = value.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) {
        return "";
    }
    const auto end = value.find_last_not_of(" \t\r\n");
    return value.substr(begin, end - begin + 1);
}

inline GNSSSystem systemFromChar(char system_char) {
    switch (system_char) {
        case 'G': return GNSSSystem::GPS;
        case 'R': return GNSSSystem::GLONASS;
        case 'E': return GNSSSystem::Galileo;
        case 'C': return GNSSSystem::BeiDou;
        case 'J': return GNSSSystem::QZSS;
        case 'S': return GNSSSystem::SBAS;
        case 'I': return GNSSSystem::NavIC;
        default: return GNSSSystem::UNKNOWN;
    }
}

inline bool parseSatelliteToken(const std::string& token, SatelliteId& satellite) {
    if (token.size() < 2) {
        return false;
    }
    const GNSSSystem system = systemFromChar(token[0]);
    if (system == GNSSSystem::UNKNOWN) {
        return false;
    }
    try {
        satellite = SatelliteId(system, static_cast<uint8_t>(std::stoi(token.substr(1))));
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

inline bool parseCompactSsrSatelliteToken(const std::string& token, SatelliteId& satellite) {
    if (pppEnvOverrides().clas_qzss_s_prn_fix &&
        token.size() >= 2 &&
        token[0] == 'S') {
        try {
            const int prn = std::stoi(token.substr(1));
            if (prn >= 120 && prn <= 122) {
                satellite = SatelliteId(
                    GNSSSystem::QZSS,
                    static_cast<uint8_t>(1 + (prn - 120)));
                return true;
            }
        } catch (const std::exception&) {
        }
    }
    return parseSatelliteToken(token, satellite);
}

inline std::string remapCompactSsrAtmosKey(std::string key) {
    if (!pppEnvOverrides().clas_qzss_s_prn_fix) {
        return key;
    }
    const std::pair<const char*, const char*> remaps[] = {
        {":S120", ":J01"},
        {":S121", ":J02"},
        {":S122", ":J03"},
    };
    for (const auto& [from, to] : remaps) {
        size_t pos = 0;
        while ((pos = key.find(from, pos)) != std::string::npos) {
            key.replace(pos, std::string(from).size(), to);
            pos += std::string(to).size();
        }
    }
    return key;
}

inline int parsePositiveIntToken(const std::map<std::string, std::string>& tokens,
                          const std::string& key) {
    const auto it = tokens.find(key);
    if (it == tokens.end()) {
        return 0;
    }
    try {
        const int value = std::stoi(it->second);
        return value > 0 ? value : 0;
    } catch (const std::exception&) {
        return 0;
    }
}

inline bool sameCorrectionVariant(const SSROrbitClockCorrection& lhs,
                           const SSROrbitClockCorrection& rhs) {
    return lhs.atmos_network_id == rhs.atmos_network_id &&
           lhs.bias_network_id == rhs.bias_network_id;
}

bool clasQzssBaseClockHygieneActive(const SatelliteId& sat);
bool atmosEmbeddedClockRow(const SSROrbitClockCorrection& entry);
void promoteClaslibBaseClockFields(SSROrbitClockCorrection& correction);

inline void mergeCorrectionFields(SSROrbitClockCorrection& target,
                           const SSROrbitClockCorrection& correction) {
    if (correction.clock_valid && correction.clock_network_id == 0 &&
        !target.mrtklib_base_clock_valid) {
        target.mrtklib_base_clock_correction_m = correction.clock_correction_m;
        target.mrtklib_base_clock_reference_time =
            correction.clock_reference_time.week != 0
                ? correction.clock_reference_time
                : correction.time;
        target.mrtklib_base_clock_valid = true;
    }
    if (correction.orbit_valid) {
        target.orbit_correction_ecef = correction.orbit_correction_ecef;
        target.orbit_valid = true;
        target.orbit_reference_time = correction.orbit_reference_time.week != 0 ?
            correction.orbit_reference_time : correction.time;
        target.iode = correction.iode;
        target.ssr_orbit_iod = correction.ssr_orbit_iod;
    }
    if (correction.clock_valid) {
        if (correction.clock_network_id == 0) {
            target.clock_correction_m = correction.clock_correction_m;
            const bool skip_atmos_base =
                clasQzssBaseClockHygieneActive(target.satellite) &&
                atmosEmbeddedClockRow(correction);
            if (!skip_atmos_base) {
                target.base_clock_correction_m = correction.clock_correction_m;
                target.base_clock_valid = true;
                target.base_clock_reference_time = correction.clock_reference_time.week != 0 ?
                    correction.clock_reference_time : correction.time;
            }
        } else {
            target.clock_correction_m = correction.clock_correction_m;
            if (clasQzssBaseClockHygieneActive(target.satellite) &&
                !atmosEmbeddedClockRow(correction)) {
                target.base_clock_correction_m = correction.clock_correction_m;
                target.base_clock_valid = true;
                target.base_clock_reference_time = correction.clock_reference_time.week != 0 ?
                    correction.clock_reference_time : correction.time;
            }
        }
        target.clock_valid = true;
        target.clock_reference_time = correction.clock_reference_time.week != 0 ?
            correction.clock_reference_time : correction.time;
        target.ssr_clock_iod = correction.ssr_clock_iod;
    }
    if (correction.clock_withdrawn) {
        target.clock_correction_m = 0.0;
        target.clock_valid = false;
        target.base_clock_correction_m = 0.0;
        target.base_clock_valid = false;
        target.mrtklib_base_clock_correction_m = 0.0;
        target.mrtklib_base_clock_valid = false;
        target.clock_withdrawn = true;
        target.clock_reference_time = correction.time;
    }
    if (correction.ura_valid) {
        target.ura_sigma_m = correction.ura_sigma_m;
        target.ura_valid = true;
    }
    if (correction.code_bias_valid) {
        for (const auto& [signal_id, bias_m] : correction.code_bias_m) {
            target.code_bias_m[signal_id] = bias_m;
        }
        target.code_bias_valid = !target.code_bias_m.empty();
        target.code_bias_rtklib_m = correction.code_bias_rtklib_m;
    }
    if (correction.phase_bias_valid) {
        for (const auto& [signal_id, bias_m] : correction.phase_bias_m) {
            target.phase_bias_m[signal_id] = bias_m;
        }
        for (const auto& [signal_id, discnt] : correction.phase_bias_discnt) {
            target.phase_bias_discnt[signal_id] = discnt;
        }
        target.phase_bias_valid = !target.phase_bias_m.empty();
        target.phase_bias_rtklib_m = correction.phase_bias_rtklib_m;
    }
    if (correction.atmos_valid) {
        for (const auto& [token, value] : correction.atmos_tokens) {
            target.atmos_tokens[token] = value;
        }
        target.atmos_valid = !target.atmos_tokens.empty();
    }
}

inline GNSSTime orbitReferenceTime(const SSROrbitClockCorrection& correction) {
    return correction.orbit_reference_time.week != 0 ?
        correction.orbit_reference_time : correction.time;
}

inline GNSSTime clockReferenceTime(const SSROrbitClockCorrection& correction) {
    return correction.clock_reference_time.week != 0 ?
        correction.clock_reference_time : correction.time;
}

inline double sampleBaseClockM(const SSROrbitClockCorrection& sample, bool& valid) {
    if (sample.base_clock_valid) {
        valid = true;
        return sample.base_clock_correction_m;
    }
    valid = sample.clock_valid;
    return sample.clock_correction_m;
}

inline void assignBaseClockOutputs(const SSROrbitClockCorrection& sample,
                            double merged_clock_m,
                            double* base_clock_correction_m,
                            bool* base_clock_valid) {
    if (base_clock_correction_m == nullptr || base_clock_valid == nullptr) {
        return;
    }
    bool valid = false;
    const double base_m = sampleBaseClockM(sample, valid);
    if (valid) {
        *base_clock_correction_m = base_m;
        *base_clock_valid = true;
        return;
    }
    *base_clock_correction_m = merged_clock_m;
    *base_clock_valid = false;
}

inline void interpolateBaseClockOutputs(const SSROrbitClockCorrection* before,
                                 const SSROrbitClockCorrection* after,
                                 double alpha,
                                 double merged_clock_m,
                                 double* base_clock_correction_m,
                                 bool* base_clock_valid,
                                 SSRClockSelectionPolicy clock_selection_policy =
                                     SSRClockSelectionPolicy::MergedInterpolate) {
    if (base_clock_correction_m == nullptr || base_clock_valid == nullptr) {
        return;
    }
    if (clock_selection_policy == SSRClockSelectionPolicy::ClaslibBaseHold ||
        clock_selection_policy == SSRClockSelectionPolicy::MrtklibLiteralBaseHold) {
        const SSROrbitClockCorrection* sample = before != nullptr ? before : after;
        if (sample != nullptr) {
            assignBaseClockOutputs(*sample, merged_clock_m,
                                   base_clock_correction_m, base_clock_valid);
        } else {
            *base_clock_correction_m = merged_clock_m;
            *base_clock_valid = false;
        }
        return;
    }
    const bool before_base = before != nullptr && before->base_clock_valid;
    const bool after_base = after != nullptr && after->base_clock_valid;
    if (before_base && after_base) {
        *base_clock_correction_m =
            before->base_clock_correction_m +
            alpha * (after->base_clock_correction_m - before->base_clock_correction_m);
        *base_clock_valid = true;
        return;
    }
    if (before_base) {
        *base_clock_correction_m = before->base_clock_correction_m;
        *base_clock_valid = true;
        return;
    }
    if (after_base) {
        *base_clock_correction_m = after->base_clock_correction_m;
        *base_clock_valid = true;
        return;
    }
    *base_clock_correction_m = merged_clock_m;
    *base_clock_valid = false;
}

constexpr double kClaslibMaxOrbitAgeFromObsSeconds = 180.0;
constexpr double kClaslibMaxClockAgeFromObsSeconds = 30.0;
constexpr double kClaslibMaxClockAgeFromOrbitSeconds = 30.0;
// The compact bias epoch is the correction reference epoch, not its L6
// reception epoch. In the MRTKLIB stream the ST6 bank becomes causal at the
inline // half-cycle point (reference TOW + 15 s); earlier observations still use the
// preceding 30 s bank.
constexpr double kMrtklibBiasReceptionLagSeconds = 15.0;

inline bool clasQzssBaseClockHygieneActive(const SatelliteId& sat) {
    return pppEnvOverrides().clas_base_clock_parity &&
           pppEnvOverrides().clas_qzss_s_prn_fix &&
           sat.system == GNSSSystem::QZSS && sat.prn >= 1 && sat.prn <= 3;
}

inline bool atmosEmbeddedClockRow(const SSROrbitClockCorrection& entry) {
    // Network-bank rows (clock_network_id>0) may carry forward-filled atmos
    // tokens from CSV ingest; only reject atmos-primary embedded dclock rows.
    return entry.atmos_valid && !entry.atmos_tokens.empty() &&
           entry.clock_network_id == 0;
}

inline bool claslibBaseClockCandidate(const SatelliteId& sat,
                               const SSROrbitClockCorrection& entry) {
    if (clasQzssBaseClockHygieneActive(sat) && atmosEmbeddedClockRow(entry)) {
        return false;
    }
    if (entry.base_clock_valid) {
        return true;
    }
    if (clasQzssBaseClockHygieneActive(sat) && entry.clock_valid &&
        entry.clock_network_id > 0 && !atmosEmbeddedClockRow(entry)) {
        return true;
    }
    return false;
}

inline double claslibBaseClockCorrectionM(const SSROrbitClockCorrection& entry) {
    if (entry.clock_valid && entry.clock_network_id > 0) {
        return entry.clock_correction_m;
    }
    return entry.base_clock_correction_m;
}

inline void promoteClaslibBaseClockFields(SSROrbitClockCorrection& correction) {
    if (!correction.clock_valid) {
        return;
    }
    const bool hygiene = clasQzssBaseClockHygieneActive(correction.satellite);
    if (hygiene && atmosEmbeddedClockRow(correction)) {
        return;
    }
    if (correction.clock_network_id == 0) {
        correction.base_clock_correction_m = correction.clock_correction_m;
        correction.base_clock_valid = true;
        correction.base_clock_reference_time =
            correction.clock_reference_time.week != 0 ?
                correction.clock_reference_time : correction.time;
        return;
    }
    if (hygiene && correction.clock_network_id > 0 && !atmosEmbeddedClockRow(correction)) {
        correction.base_clock_correction_m = correction.clock_correction_m;
        correction.base_clock_valid = true;
        correction.base_clock_reference_time =
            correction.clock_reference_time.week != 0 ?
                correction.clock_reference_time : correction.time;
    }
}

struct ClaslibClockPick {
    bool valid = false;
    bool withdrawn = false;
    double clock_correction_m = 0.0;
    GNSSTime clock_time;
    GNSSTime clock_reference_time;
    int ssr_clock_iod = -1;
};

inline ClaslibClockPick pickClaslibBaseClock(
    const SatelliteId& satellite,
    const std::vector<SSROrbitClockCorrection>& entries,
    const GNSSTime& obs_time,
    SSRClockSelectionPolicy policy) {
    const SSROrbitClockCorrection* orbit_ref = nullptr;
    GNSSTime orbit_ref_time;
    for (const auto& entry : entries) {
        if (!entry.orbit_valid) {
            continue;
        }
        const GNSSTime orbit_time = orbitReferenceTime(entry);
        const double obs_age = obs_time - orbit_time;
        if (obs_age < 0.0 || obs_age > kClaslibMaxOrbitAgeFromObsSeconds) {
            continue;
        }
        if (orbit_ref == nullptr || orbit_time > orbit_ref_time) {
            orbit_ref = &entry;
            orbit_ref_time = orbit_time;
        }
    }
    if (orbit_ref == nullptr) {
        return {};
    }

    const bool galileo_clock_withdrawal =
        policy == SSRClockSelectionPolicy::MrtklibLiteralBaseHold &&
        satellite.system == GNSSSystem::Galileo;
    const SSROrbitClockCorrection* best = nullptr;
    auto clock_end = std::upper_bound(
        entries.begin(),
        entries.end(),
        obs_time,
        [](const GNSSTime& lhs, const SSROrbitClockCorrection& rhs) {
            return lhs < rhs.time;
        });
    while (clock_end != entries.begin()) {
        const SSROrbitClockCorrection& entry = *--clock_end;
        const double obs_age = obs_time - entry.time;
        if (obs_age > kClaslibMaxClockAgeFromObsSeconds) {
            break;
        }
        if (best != nullptr && entry.time < best->time) {
            break;
        }
        const bool literal =
            policy == SSRClockSelectionPolicy::MrtklibLiteralBaseHold;
        const bool candidate_valid = literal
            ? entry.mrtklib_base_clock_valid
            : claslibBaseClockCandidate(satellite, entry);
        if (!candidate_valid &&
            !(galileo_clock_withdrawal && entry.clock_withdrawn)) {
            continue;
        }

        const GNSSTime clock_time = entry.time;
        if (clock_time > obs_time) {
            continue;
        }
        if (obs_age < 0.0 || obs_age > kClaslibMaxClockAgeFromObsSeconds) {
            continue;
        }
        const double orbit_age = clock_time - orbit_ref_time;
        if (orbit_age < 0.0 || orbit_age >= kClaslibMaxClockAgeFromOrbitSeconds) {
            continue;
        }
        if (best == nullptr || clock_time > best->time ||
            (clock_time == best->time && galileo_clock_withdrawal &&
             entry.clock_withdrawn && !best->clock_withdrawn) ||
            (clock_time == best->time &&
             clasQzssBaseClockHygieneActive(satellite) &&
             entry.clock_network_id > 0 && best->clock_network_id == 0)) {
            best = &entry;
        }
    }
    if (best == nullptr) {
        return {};
    }

    if (galileo_clock_withdrawal && best->clock_withdrawn) {
        ClaslibClockPick pick;
        pick.withdrawn = true;
        pick.clock_time = best->time;
        pick.clock_reference_time = best->time;
        return pick;
    }

    ClaslibClockPick pick;
    pick.valid = true;
    const bool literal =
        policy == SSRClockSelectionPolicy::MrtklibLiteralBaseHold;
    pick.clock_correction_m = literal
        ? best->mrtklib_base_clock_correction_m
        : claslibBaseClockCorrectionM(*best);
    pick.clock_time = best->time;
    pick.clock_reference_time = literal
        ? best->mrtklib_base_clock_reference_time
        : clockReferenceTime(*best);
    pick.ssr_clock_iod = best->ssr_clock_iod;
    return pick;
}

inline bool applyClaslibClockSelection(
    SSRClockSelectionPolicy clock_selection_policy,
    const SatelliteId& satellite,
    const std::vector<SSROrbitClockCorrection>& entries,
    const GNSSTime& time,
    double& clock_correction_m,
    double* base_clock_correction_m,
    bool* base_clock_valid,
    GNSSTime* clock_reference_time,
    SSRCorrectionStatus* status,
    bool base_result) {
    if (clock_selection_policy != SSRClockSelectionPolicy::ClaslibBaseHold &&
        clock_selection_policy != SSRClockSelectionPolicy::MrtklibLiteralBaseHold) {
        return base_result;
    }

    const ClaslibClockPick pick =
        pickClaslibBaseClock(satellite, entries, time, clock_selection_policy);
    if (!pick.valid) {
        if (clasQzssBaseClockHygieneActive(satellite) && base_result &&
            (status == nullptr || status->orbit_valid)) {
            return true;
        }
        clock_correction_m = 0.0;
        if (base_clock_correction_m != nullptr && base_clock_valid != nullptr) {
            *base_clock_correction_m = 0.0;
            *base_clock_valid = false;
        }
        if (clock_reference_time != nullptr) {
            *clock_reference_time = GNSSTime();
        }
        if (status != nullptr) {
            status->clock_valid = false;
            status->clock_withdrawn = pick.withdrawn;
            if (pick.withdrawn) {
                status->clock_reference_time = pick.clock_reference_time;
            }
        }
        return false;
    }

    clock_correction_m = pick.clock_correction_m;
    if (base_clock_correction_m != nullptr && base_clock_valid != nullptr) {
        *base_clock_correction_m = pick.clock_correction_m;
        *base_clock_valid = true;
    }
    if (clock_reference_time != nullptr) {
        *clock_reference_time = pick.clock_time;
    }
    if (status != nullptr) {
        status->clock_valid = true;
        status->clock_reference_time = pick.clock_reference_time;
        status->ssr_clock_iod = pick.ssr_clock_iod;
    }
    const bool orbit_ok = status == nullptr || status->orbit_valid;
    return pick.valid && orbit_ok;
}

inline bool parseEpochFields(int year,
                      int month,
                      int day,
                      int hour,
                      int minute,
                      double second,
                      GNSSTime& time) {
    std::tm epoch_tm{};
    epoch_tm.tm_year = year - 1900;
    epoch_tm.tm_mon = month - 1;
    epoch_tm.tm_mday = day;
    epoch_tm.tm_hour = hour;
    epoch_tm.tm_min = minute;
    epoch_tm.tm_sec = static_cast<int>(std::floor(second));
#if defined(_WIN32)
    const time_t seconds_since_unix = _mkgmtime(&epoch_tm);
#else
    const time_t seconds_since_unix = timegm(&epoch_tm);
#endif
    if (seconds_since_unix == static_cast<time_t>(-1)) {
        return false;
    }
    const auto tp = std::chrono::system_clock::from_time_t(seconds_since_unix) +
        std::chrono::microseconds(
            static_cast<long long>(std::llround((second - std::floor(second)) * 1e6)));
    time = GNSSTime::fromSystemTime(tp);
    return true;
}

inline bool parseSp3EpochLine(const std::string& line, GNSSTime& time) {
    if (line.empty() || line[0] != '*') {
        return false;
    }
    std::istringstream stream(line.substr(1));
    int year = 0;
    int month = 0;
    int day = 0;
    int hour = 0;
    int minute = 0;
    double second = 0.0;
    if (!(stream >> year >> month >> day >> hour >> minute >> second)) {
        return false;
    }
    return parseEpochFields(year, month, day, hour, minute, second, time);
}

inline bool parseClockEpochTokens(std::istringstream& stream, GNSSTime& time) {
    int year = 0;
    int month = 0;
    int day = 0;
    int hour = 0;
    int minute = 0;
    double second = 0.0;
    if (!(stream >> year >> month >> day >> hour >> minute >> second)) {
        return false;
    }
    return parseEpochFields(year, month, day, hour, minute, second, time);
}

inline bool parseIonexEpochLine(const std::string& line, GNSSTime& time) {
    const std::string epoch_text = trimCopy(line.substr(0, std::min<size_t>(43U, line.size())));
    if (epoch_text.empty()) {
        return false;
    }
    std::istringstream stream(epoch_text);
    int year = 0;
    int month = 0;
    int day = 0;
    int hour = 0;
    int minute = 0;
    double second = 0.0;
    if (!(stream >> year >> month >> day >> hour >> minute >> second)) {
        return false;
    }
    return parseEpochFields(year, month, day, hour, minute, second, time);
}

inline bool parseFixedFieldDoubles(const std::string& line,
                            size_t end_column,
                            std::vector<double>& values) {
    values.clear();
    std::istringstream stream(line.substr(0, std::min(end_column, line.size())));
    double value = 0.0;
    while (stream >> value) {
        values.push_back(value);
    }
    return !values.empty();
}

inline double normalizeLongitudeToGrid(double lon_deg, double lon_start_deg, double lon_end_deg) {
    double normalized = lon_deg;
    const double min_lon = std::min(lon_start_deg, lon_end_deg);
    const double max_lon = std::max(lon_start_deg, lon_end_deg);
    while (normalized < min_lon - 1e-9) {
        normalized += 360.0;
    }
    while (normalized > max_lon + 1e-9) {
        normalized -= 360.0;
    }
    return normalized;
}

inline bool interpolateIonexRow(const IONEXLatitudeRow& row, double longitude_deg, double& tecu) {
    if (row.values_tecu.empty() || std::abs(row.longitude_step_deg) < 1e-12) {
        return false;
    }
    const double normalized_lon =
        normalizeLongitudeToGrid(longitude_deg, row.longitude_start_deg, row.longitude_end_deg);
    const double index = (normalized_lon - row.longitude_start_deg) / row.longitude_step_deg;
    if (index < -1e-9 || index > static_cast<double>(row.values_tecu.size() - 1U) + 1e-9) {
        return false;
    }

    const double clamped = std::max(0.0, std::min(index, static_cast<double>(row.values_tecu.size() - 1U)));
    const size_t left = static_cast<size_t>(std::floor(clamped));
    const size_t right = std::min(left + 1U, row.values_tecu.size() - 1U);
    const double alpha = clamped - static_cast<double>(left);
    const double left_value = row.values_tecu[left];
    const double right_value = row.values_tecu[right];
    if (!std::isfinite(left_value) || !std::isfinite(right_value)) {
        return false;
    }
    tecu = left_value + alpha * (right_value - left_value);
    return true;
}

inline bool interpolateIonexMap(const IONEXMap& map,
                         double latitude_deg,
                         double longitude_deg,
                         double& tecu) {
    if (map.rows.empty()) {
        return false;
    }

    const IONEXLatitudeRow* before = nullptr;
    const IONEXLatitudeRow* after = nullptr;
    for (const auto& row : map.rows) {
        if (row.latitude_deg <= latitude_deg) {
            if (before == nullptr || row.latitude_deg > before->latitude_deg) {
                before = &row;
            }
        }
        if (row.latitude_deg >= latitude_deg) {
            if (after == nullptr || row.latitude_deg < after->latitude_deg) {
                after = &row;
            }
        }
    }
    if (before == nullptr) {
        before = &map.rows.front();
    }
    if (after == nullptr) {
        after = &map.rows.back();
    }

    double before_tecu = 0.0;
    double after_tecu = 0.0;
    if (before == after) {
        return interpolateIonexRow(*before, longitude_deg, tecu);
    }
    if (!interpolateIonexRow(*before, longitude_deg, before_tecu) ||
        !interpolateIonexRow(*after, longitude_deg, after_tecu)) {
        return false;
    }

    const double lat_delta = after->latitude_deg - before->latitude_deg;
    if (std::abs(lat_delta) < 1e-12) {
        tecu = before_tecu;
        return true;
    }
    const double alpha = (latitude_deg - before->latitude_deg) / lat_delta;
    tecu = before_tecu + alpha * (after_tecu - before_tecu);
    return true;
}

template <typename EntryType>
inline const EntryType* findEntryAtOrBefore(const std::vector<EntryType>& entries, const GNSSTime& time) {
    auto upper = std::lower_bound(
        entries.begin(),
        entries.end(),
        time,
        [](const EntryType& lhs, const GNSSTime& rhs) {
            return lhs.time < rhs;
        });
    if (upper == entries.end()) {
        return entries.empty() ? nullptr : &entries.back();
    }
    if (upper->time == time) {
        return &(*upper);
    }
    if (upper == entries.begin()) {
        return nullptr;
    }
    return &(*(upper - 1));
}

template <typename EntryType>
inline const EntryType* findEntryAtOrAfter(const std::vector<EntryType>& entries, const GNSSTime& time) {
    auto lower = std::lower_bound(
        entries.begin(),
        entries.end(),
        time,
        [](const EntryType& lhs, const GNSSTime& rhs) {
            return lhs.time < rhs;
        });
    if (lower == entries.end()) {
        return nullptr;
    }
    return &(*lower);
}

inline double normalizeGpsTime(double dt) {
    while (dt > kHalfGpsWeekSeconds) {
        dt -= kGpsWeekSeconds;
    }
    while (dt < -kHalfGpsWeekSeconds) {
        dt += kGpsWeekSeconds;
    }
    return dt;
}

inline double solveKepler(double mean_anomaly, double eccentricity) {
    double E = mean_anomaly;
    for (int i = 0; i < 30; ++i) {  // RTKLIB: MAX_ITER_KEPLER=30
        double Ek = E;
        E -= (E - eccentricity * std::sin(E) - mean_anomaly) / (1.0 - eccentricity * std::cos(E));
        if (std::abs(E - Ek) < 1e-14) break;  // RTKLIB: RTOL_KEPLER=1e-14
    }
    return E;
}

inline bool isBeiDouGeoPrn(uint8_t prn) {
    return prn <= 5 || (prn >= 59 && prn <= 63);
}

inline bool computeBroadcastState(const Ephemeris& eph,
                           const GNSSTime& time,
                           Vector3d& pos,
                           double& clock_bias,
                           double& clock_drift,
                           bool use_mrtklib_galileo_mu) {
    if (!eph.valid || eph.sqrt_a <= 0.0) {
        return false;
    }

    const double a = eph.sqrt_a * eph.sqrt_a;
    const double tk = normalizeGpsTime(time - eph.toe);
    const double tc = normalizeGpsTime(time - eph.toc);
    const bool is_beidou = eph.satellite.system == GNSSSystem::BeiDou;
    const bool is_galileo =
        use_mrtklib_galileo_mu &&
        eph.satellite.system == GNSSSystem::Galileo;
    const double earth_mu = is_beidou ? kBeiDouEarthMu
                                      : (is_galileo ? kGalileoEarthMu : kEarthMu);
    const double earth_rotation = is_beidou ? kBeiDouEarthRotationRate : kEarthRotationRate;

    const double n0 = std::sqrt(earth_mu / (a * a * a));
    const double n = n0 + eph.delta_n;
    const double mean_anomaly = eph.m0 + n * tk;
    const double eccentric_anomaly = solveKepler(mean_anomaly, eph.e);

    const double sin_e = std::sin(eccentric_anomaly);
    const double cos_e = std::cos(eccentric_anomaly);
    const double sqrt_one_minus_e2 = std::sqrt(1.0 - eph.e * eph.e);
    const double true_anomaly = std::atan2(sqrt_one_minus_e2 * sin_e, cos_e - eph.e);
    const double phi = true_anomaly + eph.omega;

    const double two_phi = 2.0 * phi;
    const double du = eph.cus * std::sin(two_phi) + eph.cuc * std::cos(two_phi);
    const double dr = eph.crs * std::sin(two_phi) + eph.crc * std::cos(two_phi);
    const double di = eph.cis * std::sin(two_phi) + eph.cic * std::cos(two_phi);

    const double u = phi + du;
    const double r = a * (1.0 - eph.e * cos_e) + dr;
    const double inclination = eph.i0 + eph.idot * tk + di;
    const double toe_seconds = eph.toes != 0.0 ? eph.toes : eph.toe.tow;

    const double x_orb = r * std::cos(u);
    const double y_orb = r * std::sin(u);

    const double cos_i = std::cos(inclination);
    const double sin_i = std::sin(inclination);

    if (is_beidou && isBeiDouGeoPrn(eph.satellite.prn)) {
        const double omega_geo =
            eph.omega0 + eph.omega_dot * tk - earth_rotation * toe_seconds;
        const double cos_omega = std::cos(omega_geo);
        const double sin_omega = std::sin(omega_geo);
        const double xg = x_orb * cos_omega - y_orb * cos_i * sin_omega;
        const double yg = x_orb * sin_omega + y_orb * cos_i * cos_omega;
        const double zg = y_orb * sin_i;
        const double sin_rot = std::sin(earth_rotation * tk);
        const double cos_rot = std::cos(earth_rotation * tk);
        pos(0) = xg * cos_rot + yg * sin_rot * kBeiDouGeoCos5Deg +
                 zg * sin_rot * kBeiDouGeoSin5Deg;
        pos(1) = -xg * sin_rot + yg * cos_rot * kBeiDouGeoCos5Deg +
                 zg * cos_rot * kBeiDouGeoSin5Deg;
        pos(2) = -yg * kBeiDouGeoSin5Deg + zg * kBeiDouGeoCos5Deg;
    } else {
        const double omega = eph.omega0 + (eph.omega_dot - earth_rotation) * tk
            - earth_rotation * toe_seconds;
        const double cos_omega = std::cos(omega);
        const double sin_omega = std::sin(omega);
        pos(0) = x_orb * cos_omega - y_orb * cos_i * sin_omega;
        pos(1) = x_orb * sin_omega + y_orb * cos_i * cos_omega;
        pos(2) = y_orb * sin_i;
    }

    clock_bias = eph.af0 + eph.af1 * tc + eph.af2 * tc * tc;
    clock_bias += kRelativityF * eph.e * eph.sqrt_a * sin_e;
    // TGD NOT applied here — it's applied in SPP only (DD cancels TGD)
    // clock_bias -= eph.tgd;
    clock_drift = eph.af1 + 2.0 * eph.af2 * tc;
    return true;
}

inline void computeGlonassDynamics(const double* state, double* derivative, const Vector3d& acceleration) {
    const double r2 = state[0] * state[0] + state[1] * state[1] + state[2] * state[2];
    if (r2 <= 0.0) {
        for (int i = 0; i < 6; ++i) derivative[i] = 0.0;
        return;
    }

    const double r3 = r2 * std::sqrt(r2);
    const double omg2 = kGlonassEarthRotationRate * kGlonassEarthRotationRate;
    const double a = 1.5 * kGlonassJ2 * kGlonassEarthMu *
                     (kGlonassEarthRadius * kGlonassEarthRadius) / (r2 * r3);
    const double b = 5.0 * state[2] * state[2] / r2;
    const double c = -kGlonassEarthMu / r3 - a * (1.0 - b);

    derivative[0] = state[3];
    derivative[1] = state[4];
    derivative[2] = state[5];
    derivative[3] = (c + omg2) * state[0] + 2.0 * kGlonassEarthRotationRate * state[4] + acceleration(0);
    derivative[4] = (c + omg2) * state[1] - 2.0 * kGlonassEarthRotationRate * state[3] + acceleration(1);
    derivative[5] = (c - 2.0 * a) * state[2] + acceleration(2);
}

inline void propagateGlonassOrbit(double dt, double* state, const Vector3d& acceleration) {
    double k1[6], k2[6], k3[6], k4[6], work[6];

    computeGlonassDynamics(state, k1, acceleration);
    for (int i = 0; i < 6; ++i) work[i] = state[i] + k1[i] * dt / 2.0;
    computeGlonassDynamics(work, k2, acceleration);
    for (int i = 0; i < 6; ++i) work[i] = state[i] + k2[i] * dt / 2.0;
    computeGlonassDynamics(work, k3, acceleration);
    for (int i = 0; i < 6; ++i) work[i] = state[i] + k3[i] * dt;
    computeGlonassDynamics(work, k4, acceleration);

    for (int i = 0; i < 6; ++i) {
        state[i] += (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) * dt / 6.0;
    }
}

inline bool computeGlonassState(const Ephemeris& eph,
                         const GNSSTime& time,
                         Vector3d& pos,
                         Vector3d& vel,
                         double& clock_bias,
                         double& clock_drift) {
    if (!eph.valid) {
        return false;
    }

    double state[6] = {
        eph.glonass_position(0), eph.glonass_position(1), eph.glonass_position(2),
        eph.glonass_velocity(0), eph.glonass_velocity(1), eph.glonass_velocity(2),
    };

    double t = time - eph.toe;
    clock_bias = -eph.glonass_taun + eph.glonass_gamn * t;
    clock_drift = eph.glonass_gamn;

    for (double step = t < 0.0 ? -kGlonassIntegrationStep : kGlonassIntegrationStep;
         std::abs(t) > 1e-9;
         t -= step) {
        if (std::abs(t) < kGlonassIntegrationStep) {
            step = t;
        }
        propagateGlonassOrbit(step, state, eph.glonass_acceleration);
    }

    pos = Vector3d(state[0], state[1], state[2]);
    vel = Vector3d(state[3], state[4], state[5]);
    return true;
}

inline bool computeSbasState(const Ephemeris& eph,
                      const GNSSTime& time,
                      Vector3d& pos,
                      Vector3d& vel,
                      double& clock_bias,
                      double& clock_drift) {
    if (!eph.valid) {
        return false;
    }

    // SBAS GEO ephemerides are propagated as a polynomial in the ECEF frame
    // (RTKLIB seph2pos): position and clock are evaluated at the time offset
    // from the reference epoch, matching the broadcast element set.
    const double t = time - eph.toe;
    pos = eph.glonass_position +
          eph.glonass_velocity * t +
          eph.glonass_acceleration * (t * t / 2.0);
    vel = eph.glonass_velocity + eph.glonass_acceleration * t;
    clock_bias = eph.af0 + eph.af1 * t;
    clock_drift = eph.af1;
    return true;
}

inline void ecefToGeodetic(const Vector3d& ecef, double& lat, double& lon, double& h) {
    lon = std::atan2(ecef(1), ecef(0));
    const double p = std::sqrt(ecef(0) * ecef(0) + ecef(1) * ecef(1));
    lat = std::atan2(ecef(2), p * (1.0 - kWgs84E2));

    for (int i = 0; i < 8; ++i) {
        const double sin_lat = std::sin(lat);
        const double n = kWgs84A / std::sqrt(1.0 - kWgs84E2 * sin_lat * sin_lat);
        h = p / std::cos(lat) - n;
        lat = std::atan2(ecef(2), p * (1.0 - kWgs84E2 * n / (n + h)));
    }

    const double sin_lat = std::sin(lat);
    const double n = kWgs84A / std::sqrt(1.0 - kWgs84E2 * sin_lat * sin_lat);
    h = p / std::cos(lat) - n;
}

}  // namespace navigation_internal
}  // namespace libgnss
