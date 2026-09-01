#pragma once

/**
 * @file source_pseudorange_miss_mask.hpp
 * @brief Source-exact finite-base-correction filtering for native FGO.
 *
 * The official taroz graph only inserts an undifferenced pseudorange factor
 * when its interpolated base correction is finite.  This small helper keeps
 * that operation explicit and testable without performing any I/O or
 * changing TDCP, Doppler, IMU, or epoch state vectors.
 */

#include <libgnss++/algorithms/fgo.hpp>

#include <functional>
#include <string>
#include <vector>

namespace libgnss::source_pseudorange_miss_mask {

struct Report {
    bool callback_contract_valid = false;
    bool factor_count_consistent = false;
    std::size_t original_adopted_rows = 0U;
    std::size_t retained_finite_pc_rows = 0U;
    std::size_t matched_exact_stream_rows = 0U;
    std::size_t finite_correction_rows_among_matched = 0U;
    std::size_t dropped_missing_exact_stream_rows = 0U;
    std::size_t dropped_out_of_domain_rows = 0U;
    std::size_t dropped_nonfinite_correction_rows = 0U;
    double retained_finite_pc_fraction = 0.0;
    double retained_over_original_fraction = 0.0;
    double correction_abs_p50_m = 0.0;
    double correction_abs_p95_m = 0.0;
    double correction_abs_max_m = 0.0;
    std::string failure;
};

using HasStream = std::function<bool(const SatelliteId&, SignalType)>;
using CorrectionAt = std::function<bool(const GNSSTime&,
                                         const SatelliteId&,
                                         SignalType,
                                         double&)>;

/**
 * Apply the official finite-pc miss mask in-place.
 *
 * A factor is retained only if its exact stream exists, its epoch index and
 * timestamp are valid, the callback supplies an in-domain correction, and
 * subtraction remains finite.  The retained vector preserves factor order
 * and each factor's original epoch_index; callers therefore do not need to
 * renumber epochs or touch any non-pseudorange factor collection.
 *
 * The callbacks are supplied by the already-built base model.  This function
 * performs no file or navigation access, and an invalid callback contract
 * leaves the input vector unchanged and returns false.
 */
bool apply(std::vector<FGOProcessor::PseudorangeFactor>& factors,
           const std::vector<FGOProcessor::EpochSeed>& epochs,
           const HasStream& has_stream,
           const CorrectionAt& correction_at,
           Report& report);

}  // namespace libgnss::source_pseudorange_miss_mask
