#pragma once

// Small, dependency-free contracts used by the Phase91 source-clock C0/D
// initializer. The raw-drift validator is shared by the GTSAM backend and
// focused tests; the epoch-identity validator is also used at the native raw
// entry point before the GNSS-first result is handed to the main graph.

#include <libgnss++/core/types.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace libgnss::source_clock_c0d {

struct RawDriftDInitializationReport {
    bool valid = false;
    std::size_t epoch_count = 0;
    std::size_t finite_count = 0;
    std::size_t nonfinite_count = 0;
    double min_mps = std::numeric_limits<double>::quiet_NaN();
    double max_mps = std::numeric_limits<double>::quiet_NaN();
    std::string failure;
};

// Validate the exact raw Android receiver-clock-drift sequence used to seed
// the main graph's D_i state. No value is inferred, held, interpolated, or
// otherwise repaired. `out` is populated only after the complete sequence
// passes, so callers cannot accidentally consume a partial initializer.
inline bool validateAndCopyRawDriftD(
    const std::vector<double>& raw_drift_mps,
    std::vector<double>& out,
    RawDriftDInitializationReport& report) {
    report = RawDriftDInitializationReport{};
    report.epoch_count = raw_drift_mps.size();
    out.clear();
    for (const double value : raw_drift_mps) {
        if (!std::isfinite(value)) {
            ++report.nonfinite_count;
            continue;
        }
        ++report.finite_count;
        if (report.finite_count == 1U) {
            report.min_mps = value;
            report.max_mps = value;
        } else {
            report.min_mps = std::min(report.min_mps, value);
            report.max_mps = std::max(report.max_mps, value);
        }
    }
    if (report.epoch_count == 0U) {
        report.failure = "raw receiver clock drift has no retained epochs";
        return false;
    }
    if (report.nonfinite_count != 0U) {
        report.failure = "raw receiver clock drift contains non-finite epoch values";
        return false;
    }
    if (report.finite_count != report.epoch_count) {
        report.failure = "raw receiver clock drift coverage is incomplete";
        return false;
    }
    out = raw_drift_mps;
    report.valid = true;
    return true;
}

struct ExactEpochAlignmentReport {
    bool valid = false;
    std::size_t main_epoch_count = 0;
    std::size_t gnss_first_epoch_count = 0;
    std::size_t solution_epoch_count = 0;
    std::size_t raw_epoch_count = 0;
    std::size_t raw_utc_key_count = 0;
    std::size_t aligned_epoch_count = 0;
    std::size_t epoch_count_mismatch_count = 0;
    std::size_t nonfinite_time_count = 0;
    std::size_t gnss_first_time_mismatch_count = 0;
    std::size_t raw_time_mismatch_count = 0;
    std::size_t raw_utc_key_order_mismatch_count = 0;
    std::size_t duplicate_raw_utc_key_count = 0;
    std::size_t raw_utc_key_mismatch_count = 0;
    // Retained-key diagnostics.  The raw vectors may contain epochs removed
    // by the problem builder; these counters describe only failures of the
    // explicit retained-source mapping, never a relaxed coverage gate.
    std::size_t retained_source_index_mismatch_count = 0;
    std::size_t retained_source_order_mismatch_count = 0;
    std::size_t duplicate_retained_source_index_count = 0;
    std::size_t retained_raw_key_mismatch_count = 0;
    std::size_t raw_drift_count_mismatch_count = 0;
    std::size_t raw_drift_nonfinite_count = 0;
    std::string failure;
};

struct RetainedRawEpochKey {
    std::size_t raw_source_index = std::numeric_limits<std::size_t>::max();
    std::int64_t raw_utc_time_millis = -1;
    GNSSTime time;
};

inline bool strictEpochTimeEqual(const GNSSTime& lhs, const GNSSTime& rhs) {
    // Do not use GNSSTime::operator== here: its 1e-6-second tolerance is
    // deliberately broader than the Phase91 exact retained-epoch contract.
    return lhs.week == rhs.week && std::isfinite(lhs.tow) &&
           std::isfinite(rhs.tow) && lhs.tow == rhs.tow;
}

inline bool validEpochTime(const GNSSTime& time) {
    return std::isfinite(time.tow) && time.tow >= 0.0 &&
           time.tow < 604800.0;
}

// Phase114's direct main-seed adapter is deliberately a pure, source-keyed
// contract.  It is shared with focused synthetic tests so the native app does
// not need to duplicate the exact-key, unit, or official C7 initialization
// rules.  The adapter consumes only values that the caller derived in-memory
// from the current raw run; it has no file/coordinate lookup surface.
struct DirectWlsEphemeralC7DSeedReport {
    bool valid = false;
    std::size_t raw_epoch_count = 0;
    std::size_t retained_epoch_count = 0;
    std::size_t exact_key_count = 0;
    std::size_t finite_position_count = 0;
    std::size_t finite_clock_count = 0;
    std::size_t finite_drift_count = 0;
    std::size_t finite_velocity_count = 0;
    std::size_t nonfinite_position_count = 0;
    std::size_t nonfinite_clock_count = 0;
    std::size_t nonfinite_drift_count = 0;
    std::size_t nonfinite_velocity_count = 0;
    std::size_t raw_key_mismatch_count = 0;
    std::size_t raw_key_order_mismatch_count = 0;
    std::size_t raw_drift_mismatch_count = 0;
    bool full_raw_coverage = false;
    std::string failure;
};

// Official fgo_gnss.m initial clock topology is [clk, zeros(n,6)].  The
// scalar SPP receiver clock is already required to be in metres at the FGO
// problem boundary; no magnitude-based conversion is performed here.  C[0]
// is the shared base/GPS-L1 clock and C[1..6] are explicit zero gauges until
// the raw PseudorangeFactor rows observe their mapped components.
inline bool makeOfficialEpochClockBiasVector(
    double scalar_clock_bias_m,
    std::array<double, 7>& clock_components_m) {
    if (!std::isfinite(scalar_clock_bias_m)) return false;
    clock_components_m.fill(0.0);
    clock_components_m[0] = scalar_clock_bias_m;
    return true;
}

// Validate and materialize one direct-WLS -> main C7/D seed sequence.  Every
// retained epoch must cover the complete raw source table in source order in
// this candidate.  This intentionally rejects sparse/filtered route inputs;
// no nearest-time match, finite-difference velocity, edge hold, zero fill, or
// other repair is allowed at this boundary.  `retained_drift_mps` must equal
// the exact raw value addressed by each retained key; that equality is the
// guard against a second clock source entering D.
inline bool validateAndCopyDirectWlsEphemeralC7DSeed(
    const std::vector<RetainedRawEpochKey>& retained_keys,
    const std::vector<Vector3d>& retained_positions_ecef,
    const std::vector<double>& retained_clock_bias_m,
    const std::vector<double>& retained_drift_mps,
    const std::vector<Vector3d>& retained_velocity_ecef_mps,
    const std::vector<GNSSTime>& raw_times,
    const std::vector<std::int64_t>& raw_utc_keys,
    const std::vector<double>& raw_drift_mps,
    std::vector<std::array<double, 7>>& c_out,
    std::vector<double>& d_out,
    DirectWlsEphemeralC7DSeedReport& report) {
    report = DirectWlsEphemeralC7DSeedReport{};
    c_out.clear();
    d_out.clear();
    report.raw_epoch_count = raw_times.size();
    report.retained_epoch_count = retained_keys.size();

    const std::size_t retained = retained_keys.size();
    const bool retained_sizes_match =
        retained_positions_ecef.size() == retained &&
        retained_clock_bias_m.size() == retained &&
        retained_drift_mps.size() == retained &&
        retained_velocity_ecef_mps.size() == retained;
    const bool raw_sizes_match = raw_times.size() == raw_utc_keys.size() &&
                                 raw_times.size() == raw_drift_mps.size();
    if (!retained_sizes_match || !raw_sizes_match) {
        report.failure = "direct WLS seed vector sizes do not match source tables";
        return false;
    }
    report.full_raw_coverage = retained == raw_times.size();

    for (std::size_t i = 0; i < raw_utc_keys.size(); ++i) {
        if (raw_utc_keys[i] < 0) ++report.raw_key_mismatch_count;
        if (i > 0U && raw_utc_keys[i] <= raw_utc_keys[i - 1U]) {
            ++report.raw_key_order_mismatch_count;
        }
        if (!validEpochTime(raw_times[i]) || !std::isfinite(raw_drift_mps[i])) {
            ++report.raw_drift_mismatch_count;
        }
    }

    c_out.resize(retained);
    d_out.resize(retained);
    bool c_mapping_valid = true;
    for (std::size_t i = 0; i < retained; ++i) {
        const auto& key = retained_keys[i];
        const bool key_valid =
            key.raw_source_index < raw_times.size() &&
            key.raw_utc_time_millis >= 0 && validEpochTime(key.time) &&
            raw_utc_keys[key.raw_source_index] == key.raw_utc_time_millis &&
            strictEpochTimeEqual(key.time, raw_times[key.raw_source_index]);
        if (!key_valid) {
            ++report.raw_key_mismatch_count;
        } else {
            ++report.exact_key_count;
            if (i > 0U &&
                (key.raw_source_index <= retained_keys[i - 1U].raw_source_index ||
                 key.raw_utc_time_millis <=
                     retained_keys[i - 1U].raw_utc_time_millis)) {
                ++report.raw_key_order_mismatch_count;
            }
            // Full coverage makes the source order itself part of the key
            // contract.  This catches a retained permutation even when all
            // individual key lookups happen to be valid.
            if (report.full_raw_coverage && key.raw_source_index != i) {
                ++report.raw_key_order_mismatch_count;
            }
        }

        if (retained_positions_ecef[i].allFinite()) {
            ++report.finite_position_count;
        } else {
            ++report.nonfinite_position_count;
        }
        if (std::isfinite(retained_clock_bias_m[i])) {
            ++report.finite_clock_count;
        } else {
            ++report.nonfinite_clock_count;
        }
        if (std::isfinite(retained_drift_mps[i])) {
            ++report.finite_drift_count;
        } else {
            ++report.nonfinite_drift_count;
        }
        if (retained_velocity_ecef_mps[i].allFinite()) {
            ++report.finite_velocity_count;
        } else {
            ++report.nonfinite_velocity_count;
        }

        if (key_valid &&
            (!std::isfinite(raw_drift_mps[key.raw_source_index]) ||
             retained_drift_mps[i] != raw_drift_mps[key.raw_source_index])) {
            ++report.raw_drift_mismatch_count;
        }
        if (!makeOfficialEpochClockBiasVector(retained_clock_bias_m[i],
                                              c_out[i])) {
            c_mapping_valid = false;
        }
        d_out[i] = retained_drift_mps[i];
    }

    const bool finite_complete =
        report.finite_position_count == retained &&
        report.finite_clock_count == retained &&
        report.finite_drift_count == retained &&
        report.finite_velocity_count == retained && c_mapping_valid;
    const bool keys_complete = report.exact_key_count == retained &&
                               report.raw_key_mismatch_count == 0U &&
                               report.raw_key_order_mismatch_count == 0U;
    const bool raw_complete = report.full_raw_coverage &&
                              report.raw_drift_mismatch_count == 0U;
    if (retained == 0U) {
        report.failure = "direct WLS seed has no retained epochs";
    } else if (!report.full_raw_coverage) {
        report.failure = "direct WLS seed does not cover every raw epoch";
    } else if (!keys_complete) {
        report.failure = "direct WLS seed retained keys are not exact and ordered";
    } else if (!raw_complete) {
        report.failure = "direct WLS seed D is not the exact raw drift sequence";
    } else if (!finite_complete) {
        report.failure = "direct WLS seed contains a non-finite position, clock, D, or velocity";
    } else {
        report.valid = true;
        return true;
    }
    c_out.clear();
    d_out.clear();
    return false;
}

// Validate the explicit retained EpochSeed -> raw Android mapping.  Raw
// epochs filtered out before the retained problem are allowed to be
// unmatched, but every retained key must identify one raw row exactly once,
// in source order, with the same GPST week/TOW and integer UTC key.  If
// raw_drift_mps is supplied, it must cover the complete raw source vector and
// be finite at every retained source index; no value is inferred or repaired.
// The strict retained-key path requires the complete raw drift vector.  The
// legacy timestamp-only compatibility wrapper below explicitly opts out of
// that additional source-D coverage check.
inline bool validateRetainedRawDAlignment(
    const std::vector<RetainedRawEpochKey>& main_keys,
    const std::vector<RetainedRawEpochKey>& gnss_first_keys,
    const std::vector<GNSSTime>& solution_times,
    const std::vector<GNSSTime>& raw_times,
    const std::vector<std::int64_t>& raw_utc_keys,
    const std::vector<double>& raw_drift_mps,
    ExactEpochAlignmentReport& report,
    bool require_raw_drift_coverage = true) {
    report = ExactEpochAlignmentReport{};
    report.main_epoch_count = main_keys.size();
    report.gnss_first_epoch_count = gnss_first_keys.size();
    report.solution_epoch_count = solution_times.size();
    report.raw_epoch_count = raw_times.size();
    report.raw_utc_key_count = raw_utc_keys.size();

    const std::size_t expected = report.main_epoch_count;
    const bool counts_match = report.gnss_first_epoch_count == expected &&
                              report.solution_epoch_count == expected &&
                              report.raw_epoch_count == report.raw_utc_key_count;
    if (!counts_match) ++report.epoch_count_mismatch_count;
    if (require_raw_drift_coverage && raw_drift_mps.size() != raw_times.size()) {
        ++report.raw_drift_count_mismatch_count;
    } else if (require_raw_drift_coverage) {
        for (const double drift : raw_drift_mps) {
            if (!std::isfinite(drift)) ++report.raw_drift_nonfinite_count;
        }
    }

    const auto count_nonfinite = [&report](const std::vector<GNSSTime>& times) {
        for (const auto& time : times) {
            if (!validEpochTime(time)) ++report.nonfinite_time_count;
        }
    };
    for (const auto& key : main_keys) {
        if (!validEpochTime(key.time)) ++report.nonfinite_time_count;
    }
    for (const auto& key : gnss_first_keys) {
        if (!validEpochTime(key.time)) ++report.nonfinite_time_count;
    }
    count_nonfinite(solution_times);
    count_nonfinite(raw_times);

    const std::size_t comparable = std::min(
        {expected, gnss_first_keys.size(), solution_times.size()});

    // The source vectors themselves must be a valid ordered key table.  This
    // is independent of how many rows the retained problem consumed.
    for (std::size_t i = 0; i < raw_utc_keys.size(); ++i) {
        if (raw_utc_keys[i] < 0) ++report.raw_utc_key_mismatch_count;
        if (i > 0U && raw_utc_keys[i] <= raw_utc_keys[i - 1U]) {
            ++report.raw_utc_key_order_mismatch_count;
            if (raw_utc_keys[i] == raw_utc_keys[i - 1U]) {
                ++report.duplicate_raw_utc_key_count;
            }
        }
    }

    const auto keyEqual = [](const RetainedRawEpochKey& lhs,
                             const RetainedRawEpochKey& rhs) {
        return lhs.raw_source_index == rhs.raw_source_index &&
               lhs.raw_utc_time_millis == rhs.raw_utc_time_millis &&
               strictEpochTimeEqual(lhs.time, rhs.time);
    };
    const auto validateKey = [&](const RetainedRawEpochKey& key,
                                 const RetainedRawEpochKey* previous) {
        if (key.raw_source_index == std::numeric_limits<std::size_t>::max() ||
            key.raw_source_index >= raw_times.size()) {
            ++report.retained_source_index_mismatch_count;
            return;
        }
        if (key.raw_utc_time_millis < 0 ||
            raw_utc_keys.size() != raw_times.size() ||
            raw_utc_keys[key.raw_source_index] != key.raw_utc_time_millis ||
            !strictEpochTimeEqual(key.time, raw_times[key.raw_source_index])) {
            ++report.retained_raw_key_mismatch_count;
        }
        if (previous != nullptr) {
            if (key.raw_source_index <= previous->raw_source_index) {
                ++report.retained_source_order_mismatch_count;
                if (key.raw_source_index == previous->raw_source_index) {
                    ++report.duplicate_retained_source_index_count;
                }
            }
            if (key.raw_utc_time_millis <= previous->raw_utc_time_millis) {
                ++report.retained_raw_key_mismatch_count;
            }
        }
    };

    const RetainedRawEpochKey* previous = nullptr;
    for (const auto& key : main_keys) {
        validateKey(key, previous);
        previous = &key;
    }
    previous = nullptr;
    for (const auto& key : gnss_first_keys) {
        validateKey(key, previous);
        previous = &key;
    }

    for (std::size_t i = 0; i < comparable; ++i) {
        if (!keyEqual(main_keys[i], gnss_first_keys[i]) ||
            !strictEpochTimeEqual(main_keys[i].time, solution_times[i])) {
            ++report.gnss_first_time_mismatch_count;
        }
        if (main_keys[i].raw_source_index < raw_times.size() &&
            raw_utc_keys.size() == raw_times.size() &&
            strictEpochTimeEqual(main_keys[i].time,
                                 raw_times[main_keys[i].raw_source_index])) {
            // Explicit key validation above covers the source row.  Keep this
            // branch for the legacy report's raw-time mismatch counter.
        } else {
            ++report.raw_time_mismatch_count;
        }
    }

    report.aligned_epoch_count =
        (report.nonfinite_time_count == 0U &&
         report.gnss_first_time_mismatch_count == 0U &&
         report.raw_time_mismatch_count == 0U &&
         report.raw_utc_key_order_mismatch_count == 0U &&
         report.raw_utc_key_mismatch_count == 0U &&
         report.retained_source_index_mismatch_count == 0U &&
         report.retained_source_order_mismatch_count == 0U &&
         report.retained_raw_key_mismatch_count == 0U &&
         report.raw_drift_count_mismatch_count == 0U &&
         report.raw_drift_nonfinite_count == 0U)
            ? comparable
            : 0U;
    report.valid = counts_match && report.nonfinite_time_count == 0U &&
                   report.gnss_first_time_mismatch_count == 0U &&
                   report.raw_time_mismatch_count == 0U &&
                   report.raw_utc_key_order_mismatch_count == 0U &&
                   report.raw_utc_key_mismatch_count == 0U &&
                   report.retained_source_index_mismatch_count == 0U &&
                   report.retained_source_order_mismatch_count == 0U &&
                   report.retained_raw_key_mismatch_count == 0U &&
                   report.raw_drift_count_mismatch_count == 0U &&
                   report.raw_drift_nonfinite_count == 0U;
    if (!report.valid) {
        if (report.epoch_count_mismatch_count != 0U) {
            report.failure = "retained epoch counts are not identical";
        } else if (report.nonfinite_time_count != 0U) {
            report.failure = "retained epoch identity contains non-finite time";
        } else if (report.gnss_first_time_mismatch_count != 0U) {
            report.failure = "GNSS-first/main epoch identity mismatch";
        } else if (report.raw_time_mismatch_count != 0U) {
            report.failure = "raw/main epoch identity mismatch";
        } else if (report.retained_source_index_mismatch_count != 0U) {
            report.failure = "retained epoch has no exact raw source index";
        } else if (report.retained_source_order_mismatch_count != 0U) {
            report.failure = "retained raw source indices are reordered or duplicated";
        } else if (report.retained_raw_key_mismatch_count != 0U) {
            report.failure = "retained epoch raw UTC/GPST key mismatch";
        } else if (report.raw_drift_count_mismatch_count != 0U) {
            report.failure = "raw receiver clock drift coverage does not match source epochs";
        } else if (report.raw_drift_nonfinite_count != 0U) {
            report.failure = "raw receiver clock drift lacks finite retained coverage";
        } else {
            report.failure = "raw UTC epoch keys are not strictly ordered and unique";
        }
    }
    return report.valid;
}

// Compatibility wrapper for callers that only have timestamps.  It performs
// an exact (not nearest) lookup into the full raw source table and then uses
// the retained-key validator above.  New raw paths should pass immutable
// source indices directly so duplicate timestamps cannot be ambiguous.
inline bool validateExactEpochIdentity(
    const std::vector<GNSSTime>& main_times,
    const std::vector<GNSSTime>& gnss_first_times,
    const std::vector<GNSSTime>& solution_times,
    const std::vector<GNSSTime>& raw_times,
    const std::vector<std::int64_t>& raw_utc_keys,
    ExactEpochAlignmentReport& report) {
    const auto makeKeys = [&](const std::vector<GNSSTime>& times) {
        std::vector<RetainedRawEpochKey> keys;
        keys.reserve(times.size());
        for (const auto& time : times) {
            RetainedRawEpochKey key;
            key.time = time;
            std::size_t match = std::numeric_limits<std::size_t>::max();
            std::size_t matches = 0U;
            for (std::size_t i = 0; i < raw_times.size(); ++i) {
                if (strictEpochTimeEqual(time, raw_times[i])) {
                    match = i;
                    ++matches;
                }
            }
            if (matches == 1U && match < raw_utc_keys.size()) {
                key.raw_source_index = match;
                key.raw_utc_time_millis = raw_utc_keys[match];
            }
            keys.push_back(key);
        }
        return keys;
    };
    return validateRetainedRawDAlignment(
        makeKeys(main_times), makeKeys(gnss_first_times), solution_times,
        raw_times, raw_utc_keys, {}, report, false);
}

}  // namespace libgnss::source_clock_c0d
