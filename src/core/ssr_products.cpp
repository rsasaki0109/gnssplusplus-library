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

#include "navigation_internal.hpp"

namespace libgnss {

using namespace navigation_internal;

void SSRProducts::addCorrection(const SSROrbitClockCorrection& correction) {
    auto& entries = orbit_clock_corrections[correction.satellite];
    auto lower = std::lower_bound(
        entries.begin(),
        entries.end(),
        correction.time,
        [](const SSROrbitClockCorrection& lhs, const GNSSTime& rhs) {
            return lhs.time < rhs;
        });
    auto upper = std::upper_bound(
        lower,
        entries.end(),
        correction.time,
        [](const GNSSTime& lhs, const SSROrbitClockCorrection& rhs) {
            return lhs < rhs.time;
        });

    for (auto it = lower; it != upper; ++it) {
        if (!sameCorrectionVariant(*it, correction)) {
            continue;
        }
        mergeCorrectionFields(*it, correction);
        return;
    }

    SSROrbitClockCorrection to_insert = correction;
    if (to_insert.clock_valid && to_insert.clock_network_id == 0) {
        to_insert.mrtklib_base_clock_correction_m =
            to_insert.clock_correction_m;
        to_insert.mrtklib_base_clock_reference_time =
            to_insert.clock_reference_time.week != 0
                ? to_insert.clock_reference_time
                : to_insert.time;
        to_insert.mrtklib_base_clock_valid = true;
    }
    promoteClaslibBaseClockFields(to_insert);
    entries.insert(upper, to_insert);
}

void SSRProducts::addCorrections(const std::vector<SSROrbitClockCorrection>& corrections) {
    if (corrections.empty()) {
        return;
    }

    struct CorrectionKey {
        GNSSTime time;
        int atmos_network_id = 0;
        int bias_network_id = 0;

        bool operator<(const CorrectionKey& other) const {
            if (time < other.time) {
                return true;
            }
            if (other.time < time) {
                return false;
            }
            if (atmos_network_id != other.atmos_network_id) {
                return atmos_network_id < other.atmos_network_id;
            }
            return bias_network_id < other.bias_network_id;
        }
    };

    struct PendingSatelliteCorrections {
        std::vector<SSROrbitClockCorrection> entries;
        std::map<CorrectionKey, size_t> index_by_key;
    };

    std::map<SatelliteId, PendingSatelliteCorrections> pending;
    for (const auto& [satellite, entries] : orbit_clock_corrections) {
        PendingSatelliteCorrections& sat_pending = pending[satellite];
        sat_pending.entries = entries;
        for (size_t index = 0; index < sat_pending.entries.size(); ++index) {
            const SSROrbitClockCorrection& entry = sat_pending.entries[index];
            const CorrectionKey key{entry.time, entry.atmos_network_id, entry.bias_network_id};
            sat_pending.index_by_key.emplace(key, index);
        }
    }

    for (const SSROrbitClockCorrection& correction : corrections) {
        PendingSatelliteCorrections& sat_pending = pending[correction.satellite];
        const CorrectionKey key{
            correction.time,
            correction.atmos_network_id,
            correction.bias_network_id,
        };
        const auto index_it = sat_pending.index_by_key.find(key);
        if (index_it != sat_pending.index_by_key.end()) {
            mergeCorrectionFields(sat_pending.entries[index_it->second], correction);
            continue;
        }
        sat_pending.index_by_key.emplace(key, sat_pending.entries.size());
        sat_pending.entries.push_back(correction);
    }

    orbit_clock_corrections.clear();
    for (auto& [satellite, sat_pending] : pending) {
        std::stable_sort(
            sat_pending.entries.begin(),
            sat_pending.entries.end(),
            [](const SSROrbitClockCorrection& lhs,
               const SSROrbitClockCorrection& rhs) {
                return lhs.time < rhs.time;
            });
        orbit_clock_corrections.emplace(satellite, std::move(sat_pending.entries));
    }
}

bool SSRProducts::interpolateCorrection(const SatelliteId& sat,
                                        const GNSSTime& time,
                                        Vector3d& orbit_correction_ecef,
                                        double& clock_correction_m,
                                        double* ura_sigma_m,
                                        std::map<uint8_t, double>* code_bias_m,
                                        std::map<uint8_t, double>* phase_bias_m,
                                        std::map<std::string, std::string>* atmos_tokens,
                                        GNSSTime* atmos_reference_time,
                                        GNSSTime* phase_bias_reference_time,
                                        GNSSTime* clock_reference_time,
                                        int preferred_network_id,
                                        int* orbit_iode,
                                        std::map<uint8_t, int>* phase_bias_discnt,
                                        SSRCorrectionStatus* status,
                                        bool allow_future_samples,
                                        double* base_clock_correction_m,
                                        bool* base_clock_valid,
                                        SSRClockSelectionPolicy clock_selection_policy,
                                        std::map<uint8_t, double>* code_bias_rtklib_m,
                                        std::map<uint8_t, double>* phase_bias_rtklib_m) const {
    if (status != nullptr) {
        *status = SSRCorrectionStatus{};
    }
    const auto sat_it = orbit_clock_corrections.find(sat);
    if (sat_it == orbit_clock_corrections.end() || sat_it->second.empty()) {
        return false;
    }
    if (orbit_iode != nullptr) {
        *orbit_iode = -1;
    }
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
    if (code_bias_rtklib_m != nullptr) {
        code_bias_rtklib_m->clear();
    }
    if (phase_bias_rtklib_m != nullptr) {
        phase_bias_rtklib_m->clear();
    }
    if (phase_bias_discnt != nullptr) {
        phase_bias_discnt->clear();
    }
    if (atmos_tokens != nullptr) {
        atmos_tokens->clear();
    }
    if (atmos_reference_time != nullptr) {
        *atmos_reference_time = GNSSTime();
    }
    if (phase_bias_reference_time != nullptr) {
        *phase_bias_reference_time = GNSSTime();
    }
    if (clock_reference_time != nullptr) {
        *clock_reference_time = GNSSTime();
    }

    const auto& entries = sat_it->second;
    const bool galileo_clock_withdrawal_active =
        sat.system == GNSSSystem::Galileo &&
        (clock_selection_policy == SSRClockSelectionPolicy::MrtklibLiteralBaseHold ||
         pppEnvOverrides().clas_code_row_full_prc);
    if (galileo_clock_withdrawal_active) {
        const SSROrbitClockCorrection* newest_clock_event = nullptr;
        auto event_end = std::upper_bound(
            entries.begin(),
            entries.end(),
            time,
            [](const GNSSTime& lhs, const SSROrbitClockCorrection& rhs) {
                return lhs < rhs.time;
            });
        while (event_end != entries.begin()) {
            const SSROrbitClockCorrection& entry = *--event_end;
            if (time - entry.time > kClaslibMaxClockAgeFromObsSeconds) {
                break;
            }
            if (!entry.clock_withdrawn && !entry.mrtklib_base_clock_valid) {
                continue;
            }
            if (newest_clock_event == nullptr ||
                entry.time > newest_clock_event->time ||
                (entry.time == newest_clock_event->time &&
                 entry.clock_withdrawn && !newest_clock_event->clock_withdrawn)) {
                newest_clock_event = &entry;
            }
        }
        if (newest_clock_event != nullptr && newest_clock_event->clock_withdrawn) {
            if (base_clock_correction_m != nullptr) {
                *base_clock_correction_m = 0.0;
            }
            if (base_clock_valid != nullptr) {
                *base_clock_valid = false;
            }
            if (status != nullptr) {
                status->clock_withdrawn = true;
                status->clock_reference_time = newest_clock_event->time;
            }
            return false;
        }
    }
    auto lower = std::lower_bound(
        entries.begin(),
        entries.end(),
        time,
        [](const SSROrbitClockCorrection& lhs, const GNSSTime& rhs) {
            return lhs.time < rhs;
        });
    auto upper = std::upper_bound(
        lower,
        entries.end(),
        time,
        [](const GNSSTime& lhs, const SSROrbitClockCorrection& rhs) {
            return lhs < rhs.time;
        });

    const auto biasScore = [&](const SSROrbitClockCorrection& entry, bool phase) -> int {
        const bool valid =
            phase
                ? (entry.phase_bias_valid && !entry.phase_bias_m.empty())
                : (entry.code_bias_valid && !entry.code_bias_m.empty());
        if (!valid) {
            return -1;
        }
        const size_t signal_count = phase ? entry.phase_bias_m.size() : entry.code_bias_m.size();
        if (preferred_network_id > 0) {
            if (entry.bias_network_id == preferred_network_id) {
                return 300000 + static_cast<int>(signal_count);
            }
            if (entry.bias_network_id == 0) {
                return 200000 + static_cast<int>(signal_count);
            }
            return 100000 + static_cast<int>(signal_count);
        }
        if (entry.bias_network_id == 0) {
            return 200000 + static_cast<int>(signal_count);
        }
        return 100000 + static_cast<int>(signal_count);
    };

    const bool qzss_atmos_fix =
        pppEnvOverrides().clas_qzss_s_prn_fix && sat.system == GNSSSystem::QZSS;

    // Compact SSR orbit subtype rows form a constellation-wide bank.  For the
    // legacy S120 -> J01 identity, CLASLIB clears the SSR orbit when a newer
    // bank omits S120 instead of carrying the preceding value. J02/J03 also
    // have direct compact identities, so applying the S-PRN withdrawal rule to
    // them would conflate direct J02/J03 rows with legacy S121/S122 aliases.
    // Keep this behind the explicit QZSS parity gate so ordinary SSR/MADOCA
    // selection retains the historical per-satellite hold.
    const auto qzssOrbitWithdrawnByNewBank =
        [&](const SSROrbitClockCorrection* source) -> bool {
        if (!qzss_atmos_fix || sat.prn != 1 || source == nullptr ||
            !source->orbit_valid) {
            return false;
        }
        const GNSSTime source_time = orbitReferenceTime(*source);
        GNSSTime newest_bank_time;
        bool have_newest_bank = false;
        for (const auto& [candidate_sat, candidate_entries] : orbit_clock_corrections) {
            if (candidate_sat.system != GNSSSystem::QZSS) {
                continue;
            }
            auto candidate_end = std::upper_bound(
                candidate_entries.begin(),
                candidate_entries.end(),
                time,
                [](const GNSSTime& lhs, const SSROrbitClockCorrection& rhs) {
                    return lhs < rhs.time;
                });
            while (candidate_end != candidate_entries.begin()) {
                const SSROrbitClockCorrection& candidate = *--candidate_end;
                const GNSSTime bank_time = orbitReferenceTime(candidate);
                if (bank_time > time) {
                    continue;
                }
                if (have_newest_bank && candidate.time < newest_bank_time) {
                    break;
                }
                if (!candidate.orbit_valid) {
                    continue;
                }
                if (!have_newest_bank || bank_time > newest_bank_time) {
                    newest_bank_time = bank_time;
                    have_newest_bank = true;
                }
            }
        }
        return have_newest_bank && source_time < newest_bank_time;
    };

    const auto atmosScore = [&](const SSROrbitClockCorrection& entry) -> int {
        if (!entry.atmos_valid || entry.atmos_tokens.empty()) {
            return -1;
        }
        int score = static_cast<int>(entry.atmos_tokens.size());
        if (entry.atmos_network_id > 0) {
            score += 100000;
        }
        if (preferred_network_id > 0) {
            if (entry.atmos_network_id == preferred_network_id) {
                score += 200000;
            } else if (entry.atmos_network_id == 0) {
                score += 50000;
            }
            // QZSS regional atmos (e.g. network 19 on S120-S122 rows) is valid near
            // Japan grids. Network-1 polynomial coeffs for J01-J03 are only meaningful
            // near network-1 grids (Okinawa); applying them at Chiba pierce points
            // yields ~30 TECU STEC error.
            if (qzss_atmos_fix && preferred_network_id == 1 &&
                entry.atmos_network_id > 1) {
                score += 300000;
            }
        }
        return score;
    };

    const auto pickBestAtmosAmong = [&](size_t begin, size_t end)
        -> const SSROrbitClockCorrection* {
        bool have_regional_qzss_atmos = false;
        if (qzss_atmos_fix) {
            for (size_t index = begin; index < end; ++index) {
                if (entries[index].atmos_valid &&
                    entries[index].atmos_network_id > 1) {
                    have_regional_qzss_atmos = true;
                    break;
                }
            }
        }
        const SSROrbitClockCorrection* best = nullptr;
        int best_score = -1;
        for (size_t index = begin; index < end; ++index) {
            if (!entries[index].atmos_valid || entries[index].atmos_tokens.empty()) {
                continue;
            }
            if (have_regional_qzss_atmos &&
                entries[index].atmos_network_id <= 1) {
                continue;
            }
            const int score = atmosScore(entries[index]);
            if (score > best_score) {
                best_score = score;
                best = &entries[index];
            }
        }
        return best;
    };

    const auto findQzssHeldCompactAtmos = [&](size_t scan_end_index)
        -> const SSROrbitClockCorrection* {
        const SSROrbitClockCorrection* held_preferred = nullptr;
        const SSROrbitClockCorrection* held_net19 = nullptr;
        GNSSTime held_preferred_time;
        GNSSTime held_net19_time;
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
                if (!entry.atmos_valid || entry.atmos_tokens.empty()) {
                    continue;
                }
                if (preferred_network_id > 0 &&
                    entry.atmos_network_id == preferred_network_id &&
                    (held_preferred == nullptr || entry.time > held_preferred_time)) {
                    held_preferred = &entry;
                    held_preferred_time = entry.time;
                }
                if (entry.atmos_network_id == 19 &&
                    (held_net19 == nullptr || entry.time > held_net19_time)) {
                    held_net19 = &entry;
                    held_net19_time = entry.time;
                }
            }
            group_end = group_begin;
        }
        // Regional compact network 19 must win over epoch-preferred network 1 when both
        // are held within age (ST9-only epochs still carry net-1 decoy polynomials).
        if (held_net19 != nullptr) {
            return held_net19;
        }
        if (held_preferred != nullptr) {
            return held_preferred;
        }
        if (scan_end_index > 0) {
            const size_t group_last = scan_end_index - 1;
            const GNSSTime group_time = entries[group_last].time;
            if (time - group_time <= kClasStecValidAgeSeconds + 1e-9) {
                size_t group_begin = group_last;
                while (group_begin > 0 && entries[group_begin - 1].time == group_time) {
                    --group_begin;
                }
                return pickBestAtmosAmong(group_begin, scan_end_index);
            }
        }
        return nullptr;
    };

    const auto isQzssDedicatedBiasBank = [&](const SSROrbitClockCorrection& entry, bool phase) -> bool {
        return isQzssDedicatedBiasBankEntry(entry, phase);
    };

    const auto findQzssHeldDedicatedBias = [&](bool phase, size_t scan_end_index)
        -> const SSROrbitClockCorrection* {
        const SSROrbitClockCorrection* held = nullptr;
        GNSSTime held_time;
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
                if (!isQzssDedicatedBiasBank(entry, phase)) {
                    continue;
                }
                // CLASLIB exposes a newly decoded compact bias bank only
                // after its 15 s reception delay.  The QZSS-specific path
                // bypasses scanBackwardBias(), so enforce the same causal
                // boundary here instead of consuming the bank at its tagged
                // epoch.
                if (time - entry.time < kMrtklibBiasReceptionLagSeconds) {
                    continue;
                }
                if (held == nullptr || entry.time > held_time) {
                    held = &entry;
                    held_time = entry.time;
                }
            }
            group_end = group_begin;
        }
        return held;
    };

    const auto findQzssHeldPhaseBiasForNetwork = [&](int service_network_id, size_t scan_end_index)
        -> const SSROrbitClockCorrection* {
        return findQzssHeldPhaseBiasForServiceNetwork(
            entries, time, service_network_id, scan_end_index);
    };

    const auto pickAtmosForTime = [&](const GNSSTime& atmos_time)
        -> const SSROrbitClockCorrection* {
        const double max_age =
            qzss_atmos_fix ? kClasStecValidAgeSeconds : kPreciseInterpolationGapSeconds;
        if (time - atmos_time > max_age + 1e-9 || atmos_time - time > 1e-9) {
            return nullptr;
        }
        const auto group_lower = std::lower_bound(
            entries.begin(),
            entries.end(),
            atmos_time,
            [](const SSROrbitClockCorrection& lhs, const GNSSTime& rhs) {
                return lhs.time < rhs;
            });
        if (group_lower == entries.end() || group_lower->time != atmos_time) {
            return nullptr;
        }
        const auto group_upper = std::upper_bound(
            group_lower,
            entries.end(),
            atmos_time,
            [](const GNSSTime& lhs, const SSROrbitClockCorrection& rhs) {
                return lhs < rhs.time;
            });
        const size_t group_begin = static_cast<size_t>(group_lower - entries.begin());
        const size_t group_end = static_cast<size_t>(group_upper - entries.begin());
        return pickBestAtmosAmong(group_begin, group_end);
    };

    const auto pickQzssHeldAtmos = [&]() -> const SSROrbitClockCorrection* {
        size_t scan_end = entries.size();
        if (lower != entries.end()) {
            scan_end = static_cast<size_t>(lower - entries.begin()) + 1;
        }
        return findQzssHeldCompactAtmos(scan_end);
    };

    const auto scanBackwardBias = [&](bool phase) -> const SSROrbitClockCorrection* {
        const auto scan = [&](int required_network_id)
            -> const SSROrbitClockCorrection* {
            size_t group_end = static_cast<size_t>(lower - entries.begin());
            while (group_end > 0) {
                const size_t group_last = group_end - 1;
                const GNSSTime group_time = entries[group_last].time;
                if (std::abs(group_time - time) >
                    kPreciseInterpolationGapSeconds) {
                    break;
                }
                size_t group_begin = group_last;
                while (group_begin > 0 &&
                       entries[group_begin - 1].time == group_time) {
                    --group_begin;
                }
                if (clock_selection_policy ==
                        SSRClockSelectionPolicy::MrtklibLiteralBaseHold &&
                    time - group_time < kMrtklibBiasReceptionLagSeconds) {
                    group_end = group_begin;
                    continue;
                }

                const SSROrbitClockCorrection* best = nullptr;
                int best_score = -1;
                for (size_t index = group_begin; index < group_end; ++index) {
                    if (required_network_id >= 0 &&
                        entries[index].bias_network_id != required_network_id) {
                        continue;
                    }
                    const int score = biasScore(entries[index], phase);
                    if (score > best_score) {
                        best_score = score;
                        best = &entries[index];
                    }
                }
                if (best != nullptr) {
                    return best;
                }
                group_end = group_begin;
            }
            return nullptr;
        };
        if (clock_selection_policy ==
            SSRClockSelectionPolicy::MrtklibLiteralBaseHold) {
            if (preferred_network_id > 0) {
                if (const SSROrbitClockCorrection* selected =
                        scan(preferred_network_id)) {
                    return selected;
                }
            }
            if (const SSROrbitClockCorrection* common = scan(0)) {
                return common;
            }
        }
        return scan(-1);
    };

    if (lower != entries.end() && lower->time == time) {
        const size_t lower_index = static_cast<size_t>(lower - entries.begin());
        size_t exact_begin = lower_index;
        while (exact_begin > 0 && entries[exact_begin - 1].time == time) {
            --exact_begin;
        }
        size_t exact_end = lower_index;
        while (exact_end < entries.size() && entries[exact_end].time == time) {
            ++exact_end;
        }

        const auto findRecent = [&](const auto& picker, size_t scan_end_index)
            -> const SSROrbitClockCorrection* {
            size_t group_end = scan_end_index;
            while (group_end > 0) {
                const size_t group_last = group_end - 1;
                const GNSSTime group_time = entries[group_last].time;
                if (std::abs(group_time - time) > kPreciseInterpolationGapSeconds) {
                    break;
                }
                size_t group_begin = group_last;
                while (group_begin > 0 && entries[group_begin - 1].time == group_time) {
                    --group_begin;
                }
                if (const SSROrbitClockCorrection* picked = picker(group_begin, group_end)) {
                    return picked;
                }
                group_end = group_begin;
            }
            return nullptr;
        };

        const auto pickOrbit = [&](size_t begin, size_t end) -> const SSROrbitClockCorrection* {
            for (size_t index = begin; index < end; ++index) {
                if (entries[index].orbit_valid) {
                    return &entries[index];
                }
            }
            return nullptr;
        };
        const auto pickClock = [&](size_t begin, size_t end) -> const SSROrbitClockCorrection* {
            const SSROrbitClockCorrection* fallback = nullptr;
            for (size_t index = begin; index < end; ++index) {
                if (!entries[index].clock_valid) {
                    continue;
                }
                if (clasQzssBaseClockHygieneActive(sat) &&
                    entries[index].clock_network_id > 0 &&
                    !atmosEmbeddedClockRow(entries[index])) {
                    return &entries[index];
                }
                if (fallback == nullptr) {
                    fallback = &entries[index];
                }
            }
            return fallback;
        };
        const auto pickUra = [&](size_t begin, size_t end) -> const SSROrbitClockCorrection* {
            for (size_t index = begin; index < end; ++index) {
                if (entries[index].ura_valid) {
                    return &entries[index];
                }
            }
            return nullptr;
        };
        const auto pickAtmos = [&](size_t begin, size_t end) -> const SSROrbitClockCorrection* {
            if (qzss_atmos_fix) {
                return pickBestAtmosAmong(begin, end);
            }
            const SSROrbitClockCorrection* best = nullptr;
            int best_score = -1;
            for (size_t index = begin; index < end; ++index) {
                const int score = atmosScore(entries[index]);
                if (score > best_score) {
                    best_score = score;
                    best = &entries[index];
                }
            }
            return best;
        };

        const SSROrbitClockCorrection* atmos_source =
            qzss_atmos_fix ? findQzssHeldCompactAtmos(exact_end)
                           : findRecent(pickAtmos, exact_end);
        int selected_network_id = preferred_network_id;
        if (selected_network_id <= 0 && atmos_source != nullptr && atmos_source->atmos_network_id > 0) {
            selected_network_id = atmos_source->atmos_network_id;
        }
        const auto pickBias = [&](bool phase) {
            return [&, phase](size_t begin, size_t end) -> const SSROrbitClockCorrection* {
                const SSROrbitClockCorrection* best = nullptr;
                int best_score = -1;
                for (size_t index = begin; index < end; ++index) {
                    const SSROrbitClockCorrection& entry = entries[index];
                    const bool valid =
                        phase
                            ? (entry.phase_bias_valid && !entry.phase_bias_m.empty())
                            : (entry.code_bias_valid && !entry.code_bias_m.empty());
                    if (!valid) {
                        continue;
                    }
                    if (clock_selection_policy ==
                            SSRClockSelectionPolicy::MrtklibLiteralBaseHold &&
                        time - entry.time < kMrtklibBiasReceptionLagSeconds) {
                        continue;
                    }
                    const size_t signal_count =
                        phase ? entry.phase_bias_m.size() : entry.code_bias_m.size();
                    int score = static_cast<int>(signal_count);
                    if (selected_network_id > 0) {
                        if (entry.bias_network_id == selected_network_id) {
                            score += 300000;
                        } else if (entry.bias_network_id == 0) {
                            score += 200000;
                        } else {
                            score += 100000;
                        }
                    } else if (entry.bias_network_id == 0) {
                        score += 200000;
                    } else {
                        score += 100000;
                    }
                    if (score > best_score) {
                        best_score = score;
                        best = &entry;
                    }
                }
                return best;
            };
        };
        const auto pickNetworkBias = [&](bool phase, int network_id) {
            return [&, phase, network_id](size_t begin, size_t end)
                -> const SSROrbitClockCorrection* {
                const SSROrbitClockCorrection* best = nullptr;
                size_t best_signal_count = 0;
                for (size_t index = begin; index < end; ++index) {
                    const SSROrbitClockCorrection& entry = entries[index];
                    const bool valid = phase
                        ? (entry.phase_bias_valid &&
                           !entry.phase_bias_m.empty())
                        : (entry.code_bias_valid &&
                           !entry.code_bias_m.empty());
                    if (!valid || entry.bias_network_id != network_id) {
                        continue;
                    }
                    if (clock_selection_policy ==
                            SSRClockSelectionPolicy::MrtklibLiteralBaseHold &&
                        time - entry.time < kMrtklibBiasReceptionLagSeconds) {
                        continue;
                    }
                    const size_t signal_count = phase
                        ? entry.phase_bias_m.size()
                        : entry.code_bias_m.size();
                    if (best == nullptr || signal_count > best_signal_count) {
                        best = &entry;
                        best_signal_count = signal_count;
                    }
                }
                return best;
            };
        };
        const auto findRecentBias = [&](bool phase)
            -> const SSROrbitClockCorrection* {
            // A newer correction for another CLAS service area is not a
            // refresh of the receiver's selected network. Search the entire
            // hold window for that network first, then use the historical
            // generic fallback only when no matching bank exists.
            if (clock_selection_policy ==
                SSRClockSelectionPolicy::MrtklibLiteralBaseHold) {
                if (selected_network_id > 0) {
                    if (const SSROrbitClockCorrection* selected =
                            findRecent(
                                pickNetworkBias(phase, selected_network_id),
                                exact_end)) {
                        return selected;
                    }
                }
                if (const SSROrbitClockCorrection* common =
                        findRecent(pickNetworkBias(phase, 0), exact_end)) {
                    return common;
                }
            }
            return findRecent(pickBias(phase), exact_end);
        };

        const SSROrbitClockCorrection* orbit_source = findRecent(pickOrbit, exact_end);
        if (qzssOrbitWithdrawnByNewBank(orbit_source)) {
            orbit_source = nullptr;
            if (status != nullptr) {
                status->orbit_withdrawn = true;
            }
        }
        const SSROrbitClockCorrection* clock_source = findRecent(pickClock, exact_end);
        const SSROrbitClockCorrection* ura_source = findRecent(pickUra, exact_end);
        const SSROrbitClockCorrection* code_source = nullptr;
        if (code_bias_m != nullptr) {
            code_source = qzss_atmos_fix
                ? findQzssHeldDedicatedBias(false, exact_end)
                : findRecentBias(false);
        }
        const SSROrbitClockCorrection* phase_source = nullptr;
        if (phase_bias_m != nullptr && !qzss_atmos_fix) {
            phase_source = findRecentBias(true);
        }

        if (orbit_source != nullptr && orbit_source->orbit_valid) {
            orbit_correction_ecef = orbit_source->orbit_correction_ecef;
            if (orbit_iode != nullptr) {
                *orbit_iode = orbit_source->iode;
            }
            if (status != nullptr) {
                status->orbit_valid = true;
                status->orbit_reference_time = orbitReferenceTime(*orbit_source);
                status->orbit_iode = orbit_source->iode;
                status->ssr_orbit_iod = orbit_source->ssr_orbit_iod;
            }
        }
        if (clock_source != nullptr && clock_source->clock_valid) {
            clock_correction_m = clock_source->clock_correction_m;
            assignBaseClockOutputs(*clock_source, clock_correction_m,
                                   base_clock_correction_m, base_clock_valid);
            if (clock_reference_time != nullptr) {
                *clock_reference_time = clock_source->time;
            }
            if (status != nullptr) {
                status->clock_valid = true;
                status->clock_reference_time = clockReferenceTime(*clock_source);
                status->ssr_clock_iod = clock_source->ssr_clock_iod;
            }
        }
        if (ura_sigma_m != nullptr && ura_source != nullptr && ura_source->ura_valid) {
            *ura_sigma_m = ura_source->ura_sigma_m;
            if (status != nullptr) {
                status->ura_valid = true;
                status->ura_reference_time = ura_source->time;
            }
        }
        if (code_bias_m != nullptr && code_source != nullptr && code_source->code_bias_valid) {
            *code_bias_m = code_source->code_bias_m;
            if (code_bias_rtklib_m != nullptr) {
                *code_bias_rtklib_m = code_source->code_bias_rtklib_m;
            }
            if (status != nullptr) {
                status->code_bias_valid = true;
                status->code_bias_reference_time = code_source->time;
            }
        }
        if (phase_bias_m != nullptr && phase_source != nullptr && phase_source->phase_bias_valid) {
            *phase_bias_m = phase_source->phase_bias_m;
            if (phase_bias_rtklib_m != nullptr) {
                *phase_bias_rtklib_m = phase_source->phase_bias_rtklib_m;
            }
            if (phase_bias_discnt != nullptr) {
                *phase_bias_discnt = phase_source->phase_bias_discnt;
            }
            if (phase_bias_reference_time != nullptr) {
                *phase_bias_reference_time = phase_source->time;
            }
            if (status != nullptr) {
                status->phase_bias_valid = true;
                status->phase_bias_reference_time = phase_source->time;
            }
        }
        if (atmos_tokens != nullptr && atmos_source != nullptr && atmos_source->atmos_valid) {
            *atmos_tokens = atmos_source->atmos_tokens;
            if (atmos_reference_time != nullptr) {
                *atmos_reference_time = atmos_source->time;
            }
            if (status != nullptr) {
                status->atmos_valid = true;
                status->atmos_reference_time = atmos_source->time;
            }
        }
        return applyClaslibClockSelection(
            clock_selection_policy,
            sat,
            entries,
            time,
            clock_correction_m,
            base_clock_correction_m,
            base_clock_valid,
            clock_reference_time,
            status,
            orbit_source != nullptr || clock_source != nullptr || ura_source != nullptr ||
                code_source != nullptr || phase_source != nullptr || atmos_source != nullptr);
    }

    const SSROrbitClockCorrection* before = nullptr;
    const SSROrbitClockCorrection* after = nullptr;
    if (lower != entries.end()) {
        after = &(*lower);
        if (lower != entries.begin()) {
            before = &(*(lower - 1));
        }
    } else {
        before = &entries.back();
    }

    if (before == nullptr && after == nullptr) {
        return false;
    }

    if (!allow_future_samples && before != nullptr && after != nullptr && before != after) {
        after = nullptr;
    }

    if (before != nullptr && after != nullptr && before != after) {
        const double dt_total = after->time - before->time;
        const double dt_before = time - before->time;
        if (std::abs(dt_total) < 1e-9 || std::abs(dt_total) > kPreciseInterpolationGapSeconds) {
            return false;
        }
        const double alpha = dt_before / dt_total;
        if (before->orbit_valid && after->orbit_valid) {
            orbit_correction_ecef =
                before->orbit_correction_ecef +
                alpha * (after->orbit_correction_ecef - before->orbit_correction_ecef);
            if (orbit_iode != nullptr) {
                *orbit_iode = before->iode;
            }
            if (status != nullptr) {
                status->orbit_valid = true;
                status->orbit_reference_time = orbitReferenceTime(*before);
                status->orbit_iode = before->iode;
                status->ssr_orbit_iod = before->ssr_orbit_iod;
            }
        } else if (before->orbit_valid) {
            orbit_correction_ecef = before->orbit_correction_ecef;
            if (orbit_iode != nullptr) {
                *orbit_iode = before->iode;
            }
            if (status != nullptr) {
                status->orbit_valid = true;
                status->orbit_reference_time = orbitReferenceTime(*before);
                status->orbit_iode = before->iode;
                status->ssr_orbit_iod = before->ssr_orbit_iod;
            }
        } else if (after->orbit_valid) {
            orbit_correction_ecef = after->orbit_correction_ecef;
            if (orbit_iode != nullptr) {
                *orbit_iode = after->iode;
            }
            if (status != nullptr) {
                status->orbit_valid = true;
                status->orbit_reference_time = orbitReferenceTime(*after);
                status->orbit_iode = after->iode;
                status->ssr_orbit_iod = after->ssr_orbit_iod;
            }
        } else {
            orbit_correction_ecef.setZero();
        }
        const SSROrbitClockCorrection* held_orbit_source = nullptr;
        GNSSTime held_orbit_time;
        auto orbit_end = std::upper_bound(
            entries.begin(),
            entries.end(),
            time,
            [](const GNSSTime& lhs, const SSROrbitClockCorrection& rhs) {
                return lhs < rhs.time;
            });
        while (orbit_end != entries.begin()) {
            const SSROrbitClockCorrection& candidate = *--orbit_end;
            const GNSSTime candidate_time = orbitReferenceTime(candidate);
            if (candidate_time > time) {
                continue;
            }
            // Compact-SSR reference epochs are causal: an entry cannot refer
            // to an orbit state newer than its reception/tag epoch. Once the
            // remaining entry times are older than the best reference epoch,
            // no earlier candidate can replace it.
            if (held_orbit_source != nullptr && candidate.time < held_orbit_time) {
                break;
            }
            if (!candidate.orbit_valid) {
                continue;
            }
            if (held_orbit_source == nullptr || candidate_time > held_orbit_time) {
                held_orbit_source = &candidate;
                held_orbit_time = candidate_time;
            }
        }
        const bool qzss_orbit_withdrawn =
            qzssOrbitWithdrawnByNewBank(held_orbit_source);
        if (qzss_orbit_withdrawn) {
            orbit_correction_ecef.setZero();
            if (status != nullptr) {
                status->orbit_valid = false;
                status->orbit_withdrawn = true;
                status->orbit_reference_time = GNSSTime();
                status->orbit_iode = -1;
                status->ssr_orbit_iod = -1;
            }
        }

        if (before->clock_valid && after->clock_valid) {
            clock_correction_m =
                before->clock_correction_m +
                alpha * (after->clock_correction_m - before->clock_correction_m);
            if (status != nullptr) {
                status->clock_valid = true;
                status->clock_reference_time = clockReferenceTime(*before);
                status->ssr_clock_iod = before->ssr_clock_iod;
            }
        } else if (before->clock_valid) {
            clock_correction_m = before->clock_correction_m;
            if (status != nullptr) {
                status->clock_valid = true;
                status->clock_reference_time = clockReferenceTime(*before);
                status->ssr_clock_iod = before->ssr_clock_iod;
            }
        } else if (after->clock_valid) {
            clock_correction_m = after->clock_correction_m;
            if (status != nullptr) {
                status->clock_valid = true;
                status->clock_reference_time = clockReferenceTime(*after);
                status->ssr_clock_iod = after->ssr_clock_iod;
            }
        } else {
            clock_correction_m = 0.0;
        }
        if (before != nullptr || after != nullptr) {
            interpolateBaseClockOutputs(before, after, alpha, clock_correction_m,
                                        base_clock_correction_m, base_clock_valid,
                                        clock_selection_policy);
        }
        if (ura_sigma_m != nullptr) {
            if (before->ura_valid && after->ura_valid) {
                *ura_sigma_m =
                    before->ura_sigma_m + alpha * (after->ura_sigma_m - before->ura_sigma_m);
                if (status != nullptr) {
                    status->ura_valid = true;
                    status->ura_reference_time = before->time;
                }
            } else if (before->ura_valid) {
                *ura_sigma_m = before->ura_sigma_m;
                if (status != nullptr) {
                    status->ura_valid = true;
                    status->ura_reference_time = before->time;
                }
            } else if (after->ura_valid) {
                *ura_sigma_m = after->ura_sigma_m;
                if (status != nullptr) {
                    status->ura_valid = true;
                    status->ura_reference_time = after->time;
                }
            }
        }
        // Bias forward-fill: CLAS broadcasts bias every 30s but clock every 5s.
        // The interpolation neighbours (before/after) are adjacent SSR entries
        // which are typically clock-only rows with no bias.  Scan backwards
        // through older entries to find the most recent bias, matching the
        // CLASLIB behaviour of holding the last received bias indefinitely.
        if (code_bias_m != nullptr) {
            // Code biases are stepwise corrections.  Do not consume a future
            // bias row just because the next clock interpolation neighbour has
            // one; CLAS holds the last received code-bias bank until a new
            // bank is causally available.
            const SSROrbitClockCorrection* picked = nullptr;
            if (qzss_atmos_fix) {
                const size_t scan_end = static_cast<size_t>(lower - entries.begin());
                picked = findQzssHeldDedicatedBias(false, scan_end);
            } else if (const auto* scanned = scanBackwardBias(false)) {
                picked = scanned;
            } else if (before->code_bias_valid && !before->code_bias_m.empty()) {
                picked = before;
            }
            if (picked != nullptr) {
                *code_bias_m = picked->code_bias_m;
                if (code_bias_rtklib_m != nullptr) {
                    *code_bias_rtklib_m = picked->code_bias_rtklib_m;
                }
                if (status != nullptr) {
                    status->code_bias_valid = true;
                    status->code_bias_reference_time = picked->time;
                }
            }
        }
        if (phase_bias_m != nullptr) {
            const SSROrbitClockCorrection* picked = nullptr;
            if (!qzss_atmos_fix) {
                if (clock_selection_policy ==
                    SSRClockSelectionPolicy::MrtklibLiteralBaseHold) {
                    picked = scanBackwardBias(true);
                } else if (biasScore(*before, true) >= biasScore(*after, true) &&
                    before->phase_bias_valid && !before->phase_bias_m.empty()) {
                    picked = before;
                } else if (after->phase_bias_valid && !after->phase_bias_m.empty()) {
                    picked = after;
                } else if (const auto* scanned = scanBackwardBias(true)) {
                    picked = scanned;
                }
            }
            if (picked != nullptr) {
                *phase_bias_m = picked->phase_bias_m;
                if (phase_bias_rtklib_m != nullptr) {
                    *phase_bias_rtklib_m = picked->phase_bias_rtklib_m;
                }
                if (phase_bias_discnt != nullptr) {
                    *phase_bias_discnt = picked->phase_bias_discnt;
                }
                if (phase_bias_reference_time != nullptr) {
                    *phase_bias_reference_time = picked->time;
                }
                if (status != nullptr) {
                    status->phase_bias_valid = true;
                    status->phase_bias_reference_time = picked->time;
                }
            }
        }
        if (clock_reference_time != nullptr) {
            if (after->clock_valid) {
                *clock_reference_time = after->time;
            } else if (before->clock_valid) {
                *clock_reference_time = before->time;
            }
        }
        if (atmos_tokens != nullptr) {
            if (qzss_atmos_fix) {
                const SSROrbitClockCorrection* picked_atmos = pickQzssHeldAtmos();
                if (picked_atmos != nullptr) {
                    *atmos_tokens = picked_atmos->atmos_tokens;
                    if (atmos_reference_time != nullptr) {
                        *atmos_reference_time = picked_atmos->time;
                    }
                    if (status != nullptr) {
                        status->atmos_valid = true;
                        status->atmos_reference_time = picked_atmos->time;
                    }
                }
            } else if (atmosScore(*before) >= atmosScore(*after) &&
                       before->atmos_valid && !before->atmos_tokens.empty()) {
                *atmos_tokens = before->atmos_tokens;
                if (atmos_reference_time != nullptr) {
                    *atmos_reference_time = before->time;
                }
                if (status != nullptr) {
                    status->atmos_valid = true;
                    status->atmos_reference_time = before->time;
                }
            } else if (after->atmos_valid && !after->atmos_tokens.empty()) {
                *atmos_tokens = after->atmos_tokens;
                if (atmos_reference_time != nullptr) {
                    *atmos_reference_time = after->time;
                }
                if (status != nullptr) {
                    status->atmos_valid = true;
                    status->atmos_reference_time = after->time;
                }
            } else {
                // Neither interpolation neighbour has atmos — scan backwards.
                auto scan = lower;
                while (scan != entries.begin()) {
                    --scan;
                    if (scan->atmos_valid && !scan->atmos_tokens.empty() &&
                        std::abs(scan->time - time) <= kPreciseInterpolationGapSeconds) {
                        *atmos_tokens = scan->atmos_tokens;
                        if (atmos_reference_time != nullptr) {
                            *atmos_reference_time = scan->time;
                        }
                        if (status != nullptr) {
                            status->atmos_valid = true;
                            status->atmos_reference_time = scan->time;
                        }
                        break;
                    }
                }
            }
        }
        if (qzss_orbit_withdrawn) {
            return false;
        }
        return applyClaslibClockSelection(
            clock_selection_policy,
            sat,
            entries,
            time,
            clock_correction_m,
            base_clock_correction_m,
            base_clock_valid,
            clock_reference_time,
            status,
            before->orbit_valid || after->orbit_valid ||
                before->clock_valid || after->clock_valid ||
                before->ura_valid || after->ura_valid ||
                before->code_bias_valid || after->code_bias_valid ||
                before->phase_bias_valid || after->phase_bias_valid ||
                before->atmos_valid || after->atmos_valid);
    }

    const SSROrbitClockCorrection* sample = before != nullptr ? before : after;
    if (sample == nullptr) {
        return false;
    }
    if (std::abs(sample->time - time) > kPreciseInterpolationGapSeconds) {
        return false;
    }

    // If the matched sample lacks orbit or clock, scan backwards for a
    // recent entry that has them.  This handles atmos-only rows that were
    // inserted at intervening TOWs without orbit/clock data.
    const SSROrbitClockCorrection* orbit_source = sample;
    const SSROrbitClockCorrection* clock_source = sample;
    if (!sample->orbit_valid || !sample->clock_valid) {
        // Start after the complete sample-time group.  A compact epoch can
        // contain several service-network/bias variants; lower_bound() starts
        // at the first one and the old pre-decrement skipped every sibling at
        // that same TOW.  In literal causal mode this made an orbit row at the
        // current compact epoch look stale whenever entries.back() for the
        // group happened to be an atmos/bias-only variant.
        auto scan = std::upper_bound(
            entries.begin(), entries.end(), sample->time,
            [](const GNSSTime& lhs, const SSROrbitClockCorrection& rhs) {
                return lhs < rhs.time;
            });
        while (scan != entries.begin()) {
            --scan;
            if (std::abs(scan->time - time) > kPreciseInterpolationGapSeconds) break;
            if (!orbit_source->orbit_valid && scan->orbit_valid) orbit_source = &(*scan);
            if (!clock_source->clock_valid && scan->clock_valid) clock_source = &(*scan);
            if (orbit_source->orbit_valid && clock_source->clock_valid) break;
        }
    }
    const bool orbit_withdrawn = qzssOrbitWithdrawnByNewBank(orbit_source);
    orbit_correction_ecef =
        orbit_source->orbit_valid && !orbit_withdrawn
            ? orbit_source->orbit_correction_ecef
            : Vector3d::Zero();
    clock_correction_m = clock_source->clock_valid ? clock_source->clock_correction_m : 0.0;
    assignBaseClockOutputs(*clock_source, clock_correction_m,
                           base_clock_correction_m, base_clock_valid);
    if (orbit_source->orbit_valid && !orbit_withdrawn) {
        if (orbit_iode != nullptr) {
            *orbit_iode = orbit_source->iode;
        }
        if (status != nullptr) {
            status->orbit_valid = true;
            status->orbit_reference_time = orbitReferenceTime(*orbit_source);
            status->orbit_iode = orbit_source->iode;
            status->ssr_orbit_iod = orbit_source->ssr_orbit_iod;
        }
    }
    if (orbit_withdrawn && status != nullptr) {
        status->orbit_withdrawn = true;
    }
    if (clock_reference_time != nullptr && clock_source->clock_valid) {
        *clock_reference_time = clock_source->time;
    }
    if (clock_source->clock_valid && status != nullptr) {
        status->clock_valid = true;
        status->clock_reference_time = clockReferenceTime(*clock_source);
        status->ssr_clock_iod = clock_source->ssr_clock_iod;
    }
    if (ura_sigma_m != nullptr) {
        *ura_sigma_m = sample->ura_valid ? sample->ura_sigma_m : 0.0;
        if (sample->ura_valid && status != nullptr) {
            status->ura_valid = true;
            status->ura_reference_time = sample->time;
        }
    }
    if (code_bias_m != nullptr) {
        const SSROrbitClockCorrection* picked = nullptr;
        if (qzss_atmos_fix) {
            const size_t scan_end =
                lower != entries.end() ? static_cast<size_t>(lower - entries.begin()) + 1
                                       : entries.size();
            picked = findQzssHeldDedicatedBias(false, scan_end);
        } else if (clock_selection_policy == SSRClockSelectionPolicy::ClaslibBaseHold ||
                   clock_selection_policy == SSRClockSelectionPolicy::MrtklibLiteralBaseHold) {
            picked = scanBackwardBias(false);
        }
        if (picked == nullptr) {
            picked = scanBackwardBias(false);
        }
        if (picked == nullptr && sample->code_bias_valid && !sample->code_bias_m.empty()) {
            picked = sample;
        } else if (picked == nullptr && orbit_source != sample &&
                   orbit_source->code_bias_valid && !orbit_source->code_bias_m.empty()) {
            picked = orbit_source;
        }
        if (picked != nullptr) {
            *code_bias_m = picked->code_bias_m;
            if (code_bias_rtklib_m != nullptr) {
                *code_bias_rtklib_m = picked->code_bias_rtklib_m;
            }
            if (status != nullptr) {
                status->code_bias_valid = true;
                status->code_bias_reference_time = picked->time;
            }
        }
    }
    if (phase_bias_m != nullptr) {
        const SSROrbitClockCorrection* picked = nullptr;
        if (!qzss_atmos_fix) {
            if (clock_selection_policy == SSRClockSelectionPolicy::ClaslibBaseHold ||
                clock_selection_policy == SSRClockSelectionPolicy::MrtklibLiteralBaseHold) {
                picked = scanBackwardBias(true);
            }
            if (picked == nullptr && sample->phase_bias_valid && !sample->phase_bias_m.empty()) {
                picked = sample;
            } else if (picked == nullptr && orbit_source != sample &&
                       orbit_source->phase_bias_valid && !orbit_source->phase_bias_m.empty()) {
                picked = orbit_source;
            }
        }
        if (picked != nullptr) {
            *phase_bias_m = picked->phase_bias_m;
            if (phase_bias_rtklib_m != nullptr) {
                *phase_bias_rtklib_m = picked->phase_bias_rtklib_m;
            }
            if (phase_bias_discnt != nullptr) {
                *phase_bias_discnt = picked->phase_bias_discnt;
            }
            if (phase_bias_reference_time != nullptr) {
                *phase_bias_reference_time = picked->time;
            }
            if (status != nullptr) {
                status->phase_bias_valid = true;
                status->phase_bias_reference_time = picked->time;
            }
        }
    }
    if (atmos_tokens != nullptr) {
        if (qzss_atmos_fix) {
            const SSROrbitClockCorrection* picked_atmos = pickQzssHeldAtmos();
            if (picked_atmos != nullptr) {
                *atmos_tokens = picked_atmos->atmos_tokens;
                if (atmos_reference_time != nullptr) {
                    *atmos_reference_time = picked_atmos->time;
                }
                if (status != nullptr) {
                    status->atmos_valid = true;
                    status->atmos_reference_time = picked_atmos->time;
                }
            }
        } else if (sample->atmos_valid) {
            *atmos_tokens = sample->atmos_tokens;
            if (atmos_reference_time != nullptr) {
                *atmos_reference_time = sample->time;
            }
            if (status != nullptr) {
                status->atmos_valid = true;
                status->atmos_reference_time = sample->time;
            }
        } else {
            // The exact-match sample has no atmos — scan backwards for the
            // nearest entry that does.  CLAS atmosphere messages arrive at a
            // lower cadence than orbit/clock, so a recent neighbour is valid.
            auto it = std::lower_bound(
                entries.begin(), entries.end(), sample->time,
                [](const SSROrbitClockCorrection& lhs, const GNSSTime& rhs) {
                    return lhs.time < rhs;
                });
            while (it != entries.begin()) {
                --it;
                if (it->atmos_valid && !it->atmos_tokens.empty() &&
                    std::abs(it->time - time) <= kPreciseInterpolationGapSeconds) {
                    *atmos_tokens = it->atmos_tokens;
                    if (atmos_reference_time != nullptr) {
                        *atmos_reference_time = it->time;
                    }
                    if (status != nullptr) {
                        status->atmos_valid = true;
                        status->atmos_reference_time = it->time;
                    }
                    break;
                }
            }
        }
    }
    return applyClaslibClockSelection(
        clock_selection_policy,
        sat,
        entries,
        time,
        clock_correction_m,
        base_clock_correction_m,
        base_clock_valid,
        clock_reference_time,
        status,
        sample->orbit_valid || sample->clock_valid || sample->ura_valid ||
            sample->code_bias_valid || sample->phase_bias_valid || sample->atmos_valid);
}

bool SSRProducts::heldQzssPhaseBiasForServiceNetwork(
    const SatelliteId& sat,
    const GNSSTime& time,
    int service_network_id,
    std::map<uint8_t, double>* phase_bias_m,
    std::map<uint8_t, int>* phase_bias_discnt,
    GNSSTime* phase_bias_reference_time,
    bool include_equal_time_group,
    double max_hold_age_seconds) const {
    if ((!include_equal_time_group &&
         !pppEnvOverrides().clas_qzss_s_prn_fix) ||
        sat.system != GNSSSystem::QZSS ||
        service_network_id <= 0 || phase_bias_m == nullptr) {
        return false;
    }
    return heldClasPhaseBiasForServiceNetwork(
        sat, time, service_network_id, phase_bias_m, phase_bias_discnt,
        phase_bias_reference_time, include_equal_time_group,
        max_hold_age_seconds);
}

bool SSRProducts::heldClasPhaseBiasForServiceNetwork(
    const SatelliteId& sat,
    const GNSSTime& time,
    int service_network_id,
    std::map<uint8_t, double>* phase_bias_m,
    std::map<uint8_t, int>* phase_bias_discnt,
    GNSSTime* phase_bias_reference_time,
    bool apply_reception_lag,
    double max_hold_age_seconds) const {
    if (service_network_id <= 0 || phase_bias_m == nullptr) {
        return false;
    }

    const auto sat_it = orbit_clock_corrections.find(sat);
    if (sat_it == orbit_clock_corrections.end()) {
        return false;
    }
    const auto& entries = sat_it->second;
    const GNSSTime selection_time = apply_reception_lag
        ? time - kMrtklibBiasReceptionLagSeconds
        : time;
    auto lower = std::lower_bound(
        entries.begin(),
        entries.end(),
        selection_time,
        [](const SSROrbitClockCorrection& lhs, const GNSSTime& rhs) {
            return lhs.time < rhs;
        });
    size_t scan_end;
    if (apply_reception_lag) {
        const auto causal_end = std::upper_bound(
            entries.begin(), entries.end(), selection_time,
            [](const GNSSTime& lhs, const SSROrbitClockCorrection& rhs) {
                return lhs < rhs.time;
            });
        scan_end = static_cast<size_t>(causal_end - entries.begin());
    } else {
        scan_end = lower != entries.end()
            ? static_cast<size_t>(lower - entries.begin()) + 1
            : entries.size();
    }
    const SSROrbitClockCorrection* picked =
        findQzssHeldPhaseBiasForServiceNetwork(
            entries, selection_time, service_network_id, scan_end);
    if (picked == nullptr) {
        return false;
    }
    // A CLAS ST6 network bank is replaced every 30 seconds.  A satellite or
    // service network omitted from the new bank is an explicit withdrawal,
    // not permission to retain the previous bank indefinitely.  Keep the
    // historical unbounded behavior unless the literal caller requests a
    // bank-age gate.
    const double hold_age_seconds = selection_time - picked->time;
    if (max_hold_age_seconds >= 0.0 &&
        hold_age_seconds >= max_hold_age_seconds - 1e-9) {
        return false;
    }

    phase_bias_m->clear();
    *phase_bias_m = picked->phase_bias_m;
    if (phase_bias_discnt != nullptr) {
        *phase_bias_discnt = picked->phase_bias_discnt;
    }
    if (phase_bias_reference_time != nullptr) {
        *phase_bias_reference_time = picked->time;
    }
    return !phase_bias_m->empty();
}

bool SSRProducts::heldAtmosTokensForNetwork(int network_id,
                                            const GNSSTime& time,
                                            double max_age_seconds,
                                            std::map<std::string, std::string>& atmos_tokens,
                                            GNSSTime* atmos_reference_time) const {
    if (network_id <= 0) {
        return false;
    }

    const SSROrbitClockCorrection* best = nullptr;
    GNSSTime best_time;
    for (const auto& sat_entry : orbit_clock_corrections) {
        const auto first_future = std::upper_bound(
            sat_entry.second.begin(), sat_entry.second.end(), time,
            [](const GNSSTime& epoch,
               const SSROrbitClockCorrection& entry) {
                return epoch < entry.time;
            });
        for (auto entry_it = std::make_reverse_iterator(first_future);
             entry_it != sat_entry.second.rend(); ++entry_it) {
            const auto& entry = *entry_it;
            if (time - entry.time > max_age_seconds + 1e-9) break;
            if (!entry.atmos_valid || entry.atmos_tokens.empty()) {
                continue;
            }
            if (entry.atmos_network_id != network_id) {
                continue;
            }
            if (best == nullptr || entry.time > best_time) {
                best = &entry;
                best_time = entry.time;
            }
            break;
        }
    }
    if (best == nullptr) {
        return false;
    }
    atmos_tokens = best->atmos_tokens;
    if (atmos_reference_time != nullptr) {
        *atmos_reference_time = best->time;
    }
    return true;
}

bool SSRProducts::heldClasTropTokens(
    const GNSSTime& time,
    double max_age_seconds,
    int network_id,
    int minimum_grid_count,
    std::map<std::string, std::string>& atmos_tokens,
    GNSSTime* atmos_reference_time) const {
    const SSROrbitClockCorrection* best = nullptr;
    GNSSTime best_time;
    for (auto entry_it = clas_trop_bank_corrections.rbegin();
         entry_it != clas_trop_bank_corrections.rend(); ++entry_it) {
        const auto& entry = *entry_it;
        if (entry.time - time > 1e-9) continue;
        if (time - entry.time > max_age_seconds + 1e-9) break;
        const int entry_network = parsePositiveIntToken(
            entry.atmos_tokens, "atmos_trop_network_id");
        if (network_id > 0 && entry_network != network_id) continue;
        const auto residual_it =
            entry.atmos_tokens.find("atmos_trop_residuals_m");
        if (residual_it == entry.atmos_tokens.end()) continue;
        const int residual_count = residual_it->second.empty()
            ? 0
            : 1 + static_cast<int>(std::count(
                  residual_it->second.begin(), residual_it->second.end(), ';'));
        if (minimum_grid_count > 0 && residual_count < minimum_grid_count) continue;
        best = &entry;
        best_time = entry.time;
        break;
    }
    for (const auto& sat_entry : orbit_clock_corrections) {
        // Correction histories are appended in epoch order.  Only the newest
        // compatible trop row in each satellite history can win, so avoid a
        // full-history scan on every satellite at every observation epoch.
        const auto first_future = std::upper_bound(
            sat_entry.second.begin(), sat_entry.second.end(), time,
            [](const GNSSTime& epoch,
               const SSROrbitClockCorrection& entry) {
                return epoch < entry.time;
            });
        for (auto entry_it = std::make_reverse_iterator(first_future);
             entry_it != sat_entry.second.rend(); ++entry_it) {
            const auto& entry = *entry_it;
            if (time - entry.time > max_age_seconds + 1e-9) break;
            if (!entry.atmos_valid || entry.atmos_tokens.empty()) continue;
            // atmos_tokens is an ordered map. Jump to the prefix range instead
            // of walking every string token for every satellite and receiver
            // epoch; dense CLAS histories make that linear scan dominant.
            static const std::string trop_prefix = "atmos_trop_";
            const auto trop_it = entry.atmos_tokens.lower_bound(trop_prefix);
            const bool has_trop =
                trop_it != entry.atmos_tokens.end() &&
                trop_it->first.compare(0, trop_prefix.size(), trop_prefix) == 0;
            if (!has_trop) continue;
            if (network_id > 0) {
                auto network_it =
                    entry.atmos_tokens.find("atmos_trop_network_id");
                if (network_it == entry.atmos_tokens.end()) {
                    network_it = entry.atmos_tokens.find("atmos_network_id");
                }
                if (network_it == entry.atmos_tokens.end()) continue;
                try {
                    if (std::stoi(network_it->second) != network_id) continue;
                } catch (const std::exception&) {
                    continue;
                }
            }
            if (minimum_grid_count > 0) {
                const auto residual_it =
                    entry.atmos_tokens.find("atmos_trop_residuals_m");
                if (residual_it == entry.atmos_tokens.end()) continue;
                const int residual_count = residual_it->second.empty()
                    ? 0
                    : 1 + static_cast<int>(std::count(
                          residual_it->second.begin(),
                          residual_it->second.end(), ';'));
                if (residual_count < minimum_grid_count) continue;
            }
            if (best == nullptr || entry.time > best_time) {
                best = &entry;
                best_time = entry.time;
            }
            break;
        }
    }
    if (best == nullptr) return false;
    atmos_tokens.clear();
    const bool bank_only = best->atmos_tokens.find("atmos_trop_bank_only") !=
                           best->atmos_tokens.end();
    for (const auto& [key, value] : best->atmos_tokens) {
        if (!bank_only || key.rfind("atmos_trop_", 0) == 0 ||
            key == "atmos_network_id" || key == "atmos_grid_count") {
            atmos_tokens.emplace(key, value);
        }
    }
    // A compact subtype-12 row can carry the surrounding correction row's
    // network in atmos_network_id while the troposphere payload itself belongs
    // to atmos_trop_network_id.  This method has already selected the payload
    // by the latter, and the grid interpolator consumes the former.  Keep the
    // returned token set self-consistent so a network-7 bank is not evaluated
    // against (for example) network-11 grid geometry.
    const int trop_network_id = parsePositiveIntToken(
        best->atmos_tokens, "atmos_trop_network_id");
    if (trop_network_id > 0) {
        atmos_tokens["atmos_network_id"] = std::to_string(trop_network_id);
    }
    if (atmos_reference_time != nullptr) {
        *atmos_reference_time = best->time;
    }
    return true;
}

const std::map<std::string, std::string>* SSRProducts::heldClasAtmosBankTokens(
    const GNSSTime& time,
    double max_age_seconds,
    int network_id,
    GNSSTime* atmos_reference_time) const {
    for (auto entry_it = clas_trop_bank_corrections.rbegin();
         entry_it != clas_trop_bank_corrections.rend(); ++entry_it) {
        const auto& entry = *entry_it;
        if (entry.time - time > 1e-9) continue;
        if (time - entry.time > max_age_seconds + 1e-9) break;
        if (network_id > 0 && parsePositiveIntToken(
                entry.atmos_tokens, "atmos_trop_network_id") != network_id) {
            continue;
        }
        if (atmos_reference_time != nullptr) *atmos_reference_time = entry.time;
        return &entry.atmos_tokens;
    }
    return nullptr;
}

bool SSRProducts::loadCSVFile(const std::string& filename) {
    std::ifstream input(filename);
    if (!input.is_open()) {
        return false;
    }

    orbit_corrections_are_rac_ = true;
    bool loaded_any = false;
    std::string line;
    while (std::getline(input, line)) {
        const std::string trimmed = trimCopy(line);
        if (trimmed.empty() || trimmed[0] == '#') {
            continue;
        }

        std::vector<std::string> columns;
        std::istringstream csv_stream(trimmed);
        std::string token;
        while (std::getline(csv_stream, token, ',')) {
            columns.push_back(trimCopy(token));
        }
        if (columns.size() < 7U) {
            continue;
        }

        int week = 0;
        double tow = 0.0;
        double dx = 0.0;
        double dy = 0.0;
        double dz = 0.0;
        double dclock_m = 0.0;
        const std::string& sat_token = columns[2];
        try {
            week = std::stoi(columns[0]);
            tow = std::stod(columns[1]);
            dx = std::stod(columns[3]);
            dy = std::stod(columns[4]);
            dz = std::stod(columns[5]);
            dclock_m = std::stod(columns[6]);
        } catch (const std::exception&) {
            continue;
        }

        SatelliteId satellite;
        if (!parseCompactSsrSatelliteToken(sat_token, satellite)) {
            continue;
        }

        SSROrbitClockCorrection correction;
        correction.satellite = satellite;
        correction.time = GNSSTime(week, tow);
        correction.orbit_correction_ecef = Vector3d(dx, dy, dz);
        correction.clock_correction_m = dclock_m;
        // orbit is valid only if at least one component is non-zero
        correction.orbit_valid =
            std::isfinite(dx) && std::isfinite(dy) && std::isfinite(dz) &&
            (std::abs(dx) > 0.0 || std::abs(dy) > 0.0 || std::abs(dz) > 0.0);
        correction.clock_valid = std::isfinite(dclock_m) && std::abs(dclock_m) > 0.0;
        correction.clock_withdrawn = !std::isfinite(dclock_m);
        for (size_t index = 7; index < columns.size(); ++index) {
            const std::string& extra = columns[index];
            if (extra.empty()) {
                continue;
            }
            if (extra.rfind("ura_sigma_m=", 0) == 0) {
                try {
                    correction.ura_sigma_m = std::stod(extra.substr(12));
                    correction.ura_valid = std::isfinite(correction.ura_sigma_m);
                } catch (const std::exception&) {
                }
                continue;
            }
            if (extra.rfind("orbit_iode=", 0) == 0) {
                try {
                    correction.iode = std::stoi(extra.substr(11));
                } catch (const std::exception&) {
                }
                continue;
            }
            if (extra.rfind("cbias:", 0) == 0) {
                const auto equal_pos = extra.find('=');
                if (equal_pos == std::string::npos || equal_pos <= 6) {
                    continue;
                }
                try {
                    const int signal_id = std::stoi(extra.substr(6, equal_pos - 6));
                    const double bias_m = std::stod(extra.substr(equal_pos + 1));
                    if (signal_id >= 0 && signal_id <= 255 && std::isfinite(bias_m)) {
                        correction.code_bias_m[static_cast<uint8_t>(signal_id)] = bias_m;
                    }
                } catch (const std::exception&) {
                }
                continue;
            }
            if (extra.rfind("cbias_code:", 0) == 0) {
                const auto equal_pos = extra.find('=');
                if (equal_pos == std::string::npos || equal_pos <= 11) {
                    continue;
                }
                try {
                    const int code = std::stoi(extra.substr(11, equal_pos - 11));
                    const double bias_m = std::stod(extra.substr(equal_pos + 1));
                    if (code > 0 && code <= 255) {
                        correction.code_bias_rtklib_m[static_cast<uint8_t>(code)] = bias_m;
                    }
                } catch (const std::exception&) {
                }
                continue;
            }
            if (extra.rfind("pbias:", 0) == 0) {
                const auto equal_pos = extra.find('=');
                if (equal_pos == std::string::npos || equal_pos <= 6) {
                    continue;
                }
                try {
                    const int signal_id = std::stoi(extra.substr(6, equal_pos - 6));
                    const double bias_m = std::stod(extra.substr(equal_pos + 1));
                    if (signal_id >= 0 && signal_id <= 255 && std::isfinite(bias_m)) {
                        correction.phase_bias_m[static_cast<uint8_t>(signal_id)] = bias_m;
                    }
                } catch (const std::exception&) {
                }
                continue;
            }
            if (extra.rfind("pbias_code:", 0) == 0) {
                const auto equal_pos = extra.find('=');
                if (equal_pos == std::string::npos || equal_pos <= 11) {
                    continue;
                }
                try {
                    const int code = std::stoi(extra.substr(11, equal_pos - 11));
                    const double bias_m = std::stod(extra.substr(equal_pos + 1));
                    if (code > 0 && code <= 255) {
                        correction.phase_bias_rtklib_m[static_cast<uint8_t>(code)] = bias_m;
                    }
                } catch (const std::exception&) {
                }
                continue;
            }
            if (extra.rfind("bias_network_id=", 0) == 0) {
                try {
                    correction.bias_network_id = std::max(0, std::stoi(extra.substr(16)));
                } catch (const std::exception&) {
                }
                continue;
            }
            if (extra.rfind("clock_network_id=", 0) == 0) {
                try {
                    correction.clock_network_id = std::max(0, std::stoi(extra.substr(17)));
                } catch (const std::exception&) {
                }
                continue;
            }
            if (extra.rfind("atmos_", 0) == 0) {
                const auto equal_pos = extra.find('=');
                if (equal_pos == std::string::npos || equal_pos <= 6) {
                    continue;
                }
                const std::string key =
                    remapCompactSsrAtmosKey(extra.substr(0, equal_pos));
                const std::string value = extra.substr(equal_pos + 1);
                correction.atmos_tokens[key] = value;
                if (key == "atmos_network_id") {
                    try {
                        correction.atmos_network_id = std::max(0, std::stoi(value));
                    } catch (const std::exception&) {
                    }
                }
            }
        }
        if (correction.atmos_network_id <= 0 && !correction.atmos_tokens.empty()) {
            correction.atmos_network_id =
                parsePositiveIntToken(correction.atmos_tokens, "atmos_network_id");
        }
        correction.code_bias_valid =
            !correction.code_bias_m.empty() || !correction.code_bias_rtklib_m.empty();
        correction.phase_bias_valid =
            !correction.phase_bias_m.empty() || !correction.phase_bias_rtklib_m.empty();
        correction.atmos_valid = !correction.atmos_tokens.empty();
        const auto bank_only_it =
            correction.atmos_tokens.find("atmos_trop_bank_only");
        if (bank_only_it != correction.atmos_tokens.end() &&
            bank_only_it->second == "1") {
            clas_trop_bank_corrections.push_back(std::move(correction));
            loaded_any = true;
            continue;
        }
        // Keep atmos-only rows. QZSS CLAS broadcasts network-wide atmosphere
        // corrections that may arrive without non-zero orbit/clock deltas, and
        // the PPP loader needs those rows to preserve the atmosphere tokens.
        addCorrection(correction);
        loaded_any = true;
    }

    // Debug: count atmos entries after loading
    if (pppEnvOverrides().debug) {
        size_t atmos_entries = 0;
        size_t total_entries = 0;
        for (const auto& [sat, entries] : orbit_clock_corrections) {
            for (const auto& e : entries) {
                ++total_entries;
                if (e.atmos_valid) ++atmos_entries;
            }
        }
        std::cerr << "[SSR-LOAD] total_entries=" << total_entries
                  << " atmos_entries=" << atmos_entries
                  << " satellites=" << orbit_clock_corrections.size() << "\n";
    }

    // Forward-fill orbit corrections: CLAS broadcasts orbit every 30s but
    // clock every 5s.  Carry the last valid orbit forward to clock-only epochs
    // so all epochs use SSR-corrected satellite positions.
    for (auto& [sat, entries] : orbit_clock_corrections) {
        (void)sat;
        Vector3d last_orbit = Vector3d::Zero();
        bool has_last_orbit = false;
        double last_base_clock = 0.0;
        bool has_last_base_clock = false;
        GNSSTime last_base_clock_reference_time;
        for (auto& entry : entries) {
            if (entry.orbit_valid) {
                last_orbit = entry.orbit_correction_ecef;
                has_last_orbit = true;
            } else if (has_last_orbit && entry.clock_valid) {
                entry.orbit_correction_ecef = last_orbit;
                entry.orbit_valid = true;
            }
            if (entry.base_clock_valid) {
                last_base_clock = entry.base_clock_correction_m;
                has_last_base_clock = true;
                last_base_clock_reference_time = entry.base_clock_reference_time.week != 0 ?
                    entry.base_clock_reference_time : entry.time;
            } else if (has_last_base_clock && entry.clock_valid) {
                entry.base_clock_correction_m = last_base_clock;
                entry.base_clock_valid = true;
                entry.base_clock_reference_time = last_base_clock_reference_time;
            }
        }
    }

    // Forward-fill atmosphere tokens: CLAS broadcasts atmos every 30s.
    // Carry the last valid atmos forward so all epochs have STEC/trop data.
    for (auto& [sat, entries] : orbit_clock_corrections) {
        (void)sat;
        std::map<std::string, std::string> last_atmos;
        int last_atmos_network_id = 0;
        bool has_last_atmos = false;
        for (auto& entry : entries) {
            if (entry.atmos_valid && !entry.atmos_tokens.empty()) {
                last_atmos = entry.atmos_tokens;
                last_atmos_network_id = entry.atmos_network_id;
                has_last_atmos = true;
            } else if (has_last_atmos && !entry.atmos_valid && entry.clock_valid &&
                       last_atmos.find("atmos_lifecycle") == last_atmos.end()) {
                entry.atmos_tokens = last_atmos;
                entry.atmos_network_id = last_atmos_network_id;
                entry.atmos_valid = true;
            }
        }
    }

    return loaded_any;
}

bool SSRProducts::hasData(const SatelliteId& sat, const GNSSTime& time) const {
    Vector3d orbit_correction = Vector3d::Zero();
    double clock_correction_m = 0.0;
    return interpolateCorrection(sat, time, orbit_correction, clock_correction_m);
}

void SSRProducts::clear() {
    orbit_clock_corrections.clear();
    clas_trop_bank_corrections.clear();
    orbit_corrections_are_rac_ = false;
}
} // namespace libgnss
