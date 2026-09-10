// CLAS-PPP epoch processing method for PPPProcessor.
// Split from ppp.cpp for modularity.

#include <libgnss++/algorithms/ppp.hpp>
#include <libgnss++/algorithms/ppp_ar.hpp>
#include <libgnss++/algorithms/ppp_clas.hpp>
#include <libgnss++/algorithms/ppp_clas_dd.hpp>
#include <libgnss++/algorithms/ppp_env_overrides.hpp>
#include <libgnss++/algorithms/ppp_osr.hpp>
#include <libgnss++/algorithms/rtk_validation.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/signals.hpp>
#include <libgnss++/external/madocalib_oracle.hpp>
#include <libgnss++/iers/earth_rotation.hpp>
#include <libgnss++/iers/ephemeris.hpp>

#include <algorithm>
#include <array>
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>

extern "C" {
#include "sofa.h"
}


#include "ppp_clas_epoch_internal.hpp"

namespace libgnss {

using PPPConfig = ppp_shared::PPPConfig;
using PPPState = ppp_shared::PPPState;
using PPPAmbiguityInfo = ppp_shared::PPPAmbiguityInfo;
using namespace ppp_clas_epoch_internal;
using PPPConfig = ppp_shared::PPPConfig;
using PPPState = ppp_shared::PPPState;
using PPPAmbiguityInfo = ppp_shared::PPPAmbiguityInfo;

PositionSolution PPPProcessor::processEpochCLAS(const ObservationData& obs,
                                                 const NavigationData& nav) {
    PositionSolution solution;
    solution.time = obs.time;
    solution.status = SolutionStatus::NONE;
    last_clas_hybrid_fallback_used_ = false;
    last_clas_hybrid_fallback_reason_.clear();
    last_ar_ratio_ = 0.0;
    last_fixed_ambiguities_ = 0;
    last_clas_constrained_fixed_state_valid_ = false;
    if (ppp_config_.clas_mrtklib_float_parity &&
        ppp_config_.kinematic_mode &&
        !ppp_config_.low_dynamics_mode &&
        ppp_config_.use_clas_osr_filter &&
        ppp_config_.use_dynamics_model) {
        recordClasDispersionWarmupPhasePairs(
            obs, clas_dispersion_compensation_);
    }
    // CLAS per-frequency mode uses WL-NL AR: MW averaging resolves WL integers,
    // then NL integers are extracted from OSR-corrected dual-freq observations.
    if (ppp_config_.enable_ambiguity_resolution && !ppp_config_.use_ionosphere_free) {
        ppp_config_.ar_method = PPPConfig::ARMethod::DD_WLNL;
        // CLAS corrections stabilize MW rapidly; fewer averaging epochs needed.
        if (ppp_config_.wl_min_averaging_epochs > 5) {
            ppp_config_.wl_min_averaging_epochs = 5;
        }
    }

    struct ClasFallbackSnapshot {
        PPPState filter_state;
        bool filter_initialized = false;
        GNSSTime convergence_start_time;
        Vector3d static_anchor_position = Vector3d::Zero();
        bool has_static_anchor_position = false;
        std::map<SatelliteId, PPPAmbiguityInfo> ambiguity_states;
        std::map<SatelliteId, CLASDispersionCompensationInfo> dispersion_compensation;
        std::map<SatelliteId, CLASSisContinuityInfo> sis_continuity;
        std::map<SatelliteId, double> windup_cache;
        std::map<SatelliteId, CLASPhaseBiasRepairInfo> phase_bias_repair;
        bool has_last_processed_time = false;
        GNSSTime last_processed_time;
        int last_clas_atmos_network_id = -1;
        bool has_last_clas_atmos_network_id = false;
        PositionSolution last_valid_spp_seed;
        bool has_last_valid_spp_seed = false;
        Vector3d last_rejected_output_position = Vector3d::Zero();
        GNSSTime last_rejected_output_time;
        bool has_last_rejected_output = false;
    };
    const ClasFallbackSnapshot fallback_snapshot{
        filter_state_,
        filter_initialized_,
        convergence_start_time_,
        static_anchor_position_,
        has_static_anchor_position_,
        ambiguity_states_,
        clas_dispersion_compensation_,
        clas_sis_continuity_,
        windup_cache_,
        clas_phase_bias_repair_,
        has_last_processed_time_,
        last_processed_time_,
        last_clas_atmos_network_id_,
        has_last_clas_atmos_network_id_,
        clas_last_valid_spp_seed_,
        has_clas_last_valid_spp_seed_,
        clas_last_rejected_output_position_ecef_,
        clas_last_rejected_output_time_,
        has_clas_last_rejected_output_,
    };
    const auto restore_clas_snapshot = [&]() {
        filter_state_ = fallback_snapshot.filter_state;
        filter_initialized_ = fallback_snapshot.filter_initialized;
        convergence_start_time_ = fallback_snapshot.convergence_start_time;
        static_anchor_position_ = fallback_snapshot.static_anchor_position;
        has_static_anchor_position_ = fallback_snapshot.has_static_anchor_position;
        ambiguity_states_ = fallback_snapshot.ambiguity_states;
        clas_dispersion_compensation_ = fallback_snapshot.dispersion_compensation;
        clas_sis_continuity_ = fallback_snapshot.sis_continuity;
        windup_cache_ = fallback_snapshot.windup_cache;
        clas_phase_bias_repair_ = fallback_snapshot.phase_bias_repair;
        has_last_processed_time_ = fallback_snapshot.has_last_processed_time;
        last_processed_time_ = fallback_snapshot.last_processed_time;
        last_clas_atmos_network_id_ =
            fallback_snapshot.last_clas_atmos_network_id;
        has_last_clas_atmos_network_id_ =
            fallback_snapshot.has_last_clas_atmos_network_id;
        clas_last_valid_spp_seed_ = fallback_snapshot.last_valid_spp_seed;
        has_clas_last_valid_spp_seed_ =
            fallback_snapshot.has_last_valid_spp_seed;
        clas_last_rejected_output_position_ecef_ =
            fallback_snapshot.last_rejected_output_position;
        clas_last_rejected_output_time_ =
            fallback_snapshot.last_rejected_output_time;
        has_clas_last_rejected_output_ =
            fallback_snapshot.has_last_rejected_output;
    };
    const bool allow_hybrid_fallback =
        ppp_config_.clas_epoch_policy ==
        PPPConfig::ClasEpochPolicy::HYBRID_STANDARD_PPP_FALLBACK;
    const auto clear_clas_ar_continuation = [&]() {
        had_fixed_last_epoch_ = false;
        clas_ar_attempt_used_reduced_dd_floor_ = false;
        clas_reduced_dd_fix_last_epoch_ = false;
        clas_full_dd_fix_last_epoch_ = false;
        clas_reduced_dd_publication_streak_ = 0;
    };
    const auto fallback_to_standard = [&](const char* reason) {
        restore_clas_snapshot();
        // A standard-PPP publication cannot authorize a later reduced-row
        // CLAS continuation, even if the historical CLAS hold survived the
        // snapshot restore.
        clear_clas_ar_continuation();
        return processEpochStandard(obs, nav, reason);
    };

    const bool clas_mrtklib_parity =
        ppp_config_.clas_mrtklib_float_parity &&
        ppp_config_.kinematic_mode && !ppp_config_.low_dynamics_mode &&
        ppp_config_.use_clas_osr_filter && ppp_config_.use_dynamics_model;
    PositionSolution seed;
    // Set below (parity path only) once the redundancy/jump guards have run;
    // see the long comment at its assignment for what this gates.
    bool clas_seed_untrusted_this_epoch = false;
    bool clas_baseline_seed_maxdiff_this_epoch = false;
    bool clas_seed_failed_before_continuity_fallback = false;
    // Mirrors the block-scoped `clas_seed_chi_square_failed` local below (its
    // scope ends with the `if (clas_mrtklib_parity)` block); kept alive at
    // function scope for the hold-continuation carve-out computed just before
    // the AR call, well after that block has closed.
    bool clas_seed_chi_square_failed_this_epoch = false;
    bool clas_seed_ar_recovery_this_epoch = false;
    // CLAS-HOLDCONT-DBG diagnostic: mirrors the block-scoped
    // baseline_filter_spp_distance_m local (same reasoning as
    // clas_seed_chi_square_failed_this_epoch above).
    double clas_baseline_filter_spp_distance_m_this_epoch =
        std::numeric_limits<double>::quiet_NaN();
    PositionSolution clas_continuity_output;
    bool has_clas_continuity_output = false;
    bool clas_rejected_seed_output_prepared = false;
    bool has_clas_validation_rejected_candidate = false;
    bool clas_seed_validation_failed = false;
    if (clas_mrtklib_parity) {
        const auto original_spp_config = spp_processor_.getSPPConfig();
        auto clas_spp_config = original_spp_config;
        // MRTKLIB CLAS pntpos only admits constellations represented by the
        // current CLAS SSR mask.  The benchmark stream contains GPS, Galileo,
        // and QZSS; feeding unrelated broadcast-only BDS/GLO observations to
        // the reset seed changes the kinematic float trajectory.
        clas_spp_config.enable_beidou = false;
        clas_spp_config.enable_glonass = false;
        // pntpos() switches a multi-frequency configuration to code IFLC.
        // Its CLAS err[] model is approximately one metre plus elevation,
        // with IFLC's 3x sigma multiplier; native's SNR/atmosphere variance
        // and MAD clipping are not part of that solve.
        clas_spp_config.use_ionosphere_free_combination = true;
        clas_spp_config.mrtklib_iflc_code_bias = true;
        clas_spp_config.mrtklib_clas_snr_mask = true;
        // rtkpos() forces EPHOPT_BRDC for the PPP-RTK pntpos() seed even
        // though the subsequent CLAS filter uses SSR APC products.
        clas_spp_config.use_ssr_corrections = false;
        clas_spp_config.pseudorange_sigma = 1.0;
        clas_spp_config.use_variance_model = false;
        clas_spp_config.enable_outlier_detection = false;
        clas_spp_config.elevation_mask_override_deg = 15.0;
        // First solve the baseline exactly once. Applying native's
        // leave-one-out selection to every valid epoch changes otherwise-good
        // reset seeds and AR cadence. Retry with FDE below only when the seed
        // has already failed the parity validity/maxdiff tests.
        clas_spp_config.enable_raim_fde = false;
        spp_processor_.setSPPConfig(clas_spp_config);
        seed = spp_processor_.processEpoch(obs, nav);
        const bool clas_seed_redundancy_failed =
            seed.isValid() && clasSeedFailsRedundancyGate(seed);
        double baseline_filter_spp_distance_m =
            std::numeric_limits<double>::quiet_NaN();
        if (seed.isValid() && filter_initialized_ &&
            filter_state_.pos_index >= 0 &&
            filter_state_.pos_index + 2 < filter_state_.state.size()) {
            baseline_filter_spp_distance_m =
                (filter_state_.state.segment<3>(filter_state_.pos_index) -
                 seed.position_ecef).norm();
        }
        const bool clas_seed_needs_fde =
            ppp_shared::shouldRetryClasSeedWithFde(
                seed.isValid(), clas_seed_redundancy_failed,
                filter_initialized_, baseline_filter_spp_distance_m,
                kMrtklibMaxSppDivergenceM);
        clas_baseline_seed_maxdiff_this_epoch =
            seed.isValid() && filter_initialized_ &&
            std::isfinite(baseline_filter_spp_distance_m) &&
            baseline_filter_spp_distance_m > kMrtklibMaxSppDivergenceM;
        clas_baseline_filter_spp_distance_m_this_epoch =
            baseline_filter_spp_distance_m;
        // CLAS-HOLDCONT-DBG diagnostic: streak length is inclusive of this
        // epoch (updated here, immediately after the flag above is known).
        if (clas_baseline_seed_maxdiff_this_epoch) {
            ++clas_maxdiff_consecutive_streak_epochs_;
            if (has_clas_last_full_state_reset_time_ &&
                (obs.time - clas_last_full_state_reset_time_) >= 0.0 &&
                (obs.time - clas_last_full_state_reset_time_) <= 1.0) {
                if (!clas_post_reset_saw_maxdiff_ &&
                    env_overrides_.clas_post_reset_ratio_floor > 0.0) {
                    std::cerr << "[CLAS-POST-RESET-HAZARD] tow="
                              << obs.time.tow
                              << " reset_age="
                              << (obs.time - clas_last_full_state_reset_time_)
                              << " filter_spp_distance_m="
                              << clas_baseline_filter_spp_distance_m_this_epoch
                              << "\n";
                }
                clas_post_reset_saw_maxdiff_ = true;
            }
        } else {
            clas_maxdiff_consecutive_streak_epochs_ = 0;
        }
        if (clas_seed_needs_fde) {
            clas_spp_config.enable_raim_fde = true;
            spp_processor_.setSPPConfig(clas_spp_config);
            const PositionSolution fde_seed =
                spp_processor_.processEpoch(obs, nav);
            if (fde_seed.isValid()) {
                seed = fde_seed;
            }
        }
        // pntpos() failure includes valsol() rejecting an otherwise finite LS
        // solution (chi-square/redundancy), not just failure to form the LS.
        // Capture the final FDE result using that same definition; the two
        // failure classes have different sol.rr semantics below.
        const bool clas_seed_chi_square_failed =
            seed.isValid() && clasSeedFailsChiSquareGate(seed);
        clas_seed_chi_square_failed_this_epoch = clas_seed_chi_square_failed;
        const bool clas_seed_dof_failed =
            seed.isValid() && clasSeedLacksRedundancy(seed);
        const PositionSolution clas_validation_rejected_candidate = seed;
        has_clas_validation_rejected_candidate = seed.isValid();
        // MRTKLIB's estpos() writes the converged current-epoch sol.rr and
        // sol.time before valsol() checks chi-square.  If valsol() then
        // returns false, dynamics mode still enters ppp_rtk_pos() with that
        // current SPP position; it is not the previous epoch's stale sol.
        // The full six-run gate showed that admitting ordinary chi-square
        // failures to the filter helps Tokyo 2 but regresses Nagoya 2 all-
        // solution RMS and FIX rate. Keep both validation failures out of
        // ordinary filter admission. The counted maxdiffp path below still
        // sees the finite candidate, matching MRTKLIB's sol.rr lifecycle;
        // a bounded output-only path can also publish it without admission.
        const bool clas_masked_spp_admission_failed =
            !seed.isValid() || clas_seed_chi_square_failed ||
            clas_seed_dof_failed;
        spp_processor_.setSPPConfig(original_spp_config);
        if (clas_seed_chi_square_failed || clas_seed_dof_failed) {
            if (pppDebugEnabled()) {
                std::cerr << "[CLAS-SEED-CHI2] reject tow=" << obs.time.tow
                          << " dof=" << seed.spp_degrees_of_freedom
                          << " chi2=" << seed.spp_chi_square
                          << " ns=" << seed.num_satellites << "\n";
            }
            seed.status = SolutionStatus::NONE;
        }
        // Second, complementary guard: a sparse multi-GNSS SPP epoch can
        // still pass the chi-square/dof check above (enough redundancy for
        // a numerically fine fit) while one of its few satellites carries an
        // undetected bias (bad ephemeris/SSR lookup, multipath) that a
        // single-satellite RAIM/FDE exclusion would normally catch. Observed
        // on nagoya_run2 tow 557038.4: dof=1, chi2=2.27 (well under the
        // dof=1 table entry 10.8), yet the fix was ~4.2 km from the last
        // published position. That epoch is also mid-reset (filter_
        // initialized_ is false: this urban segment cycles through repeated
        // cold reinitialization, never holding a fix long enough to
        // accumulate 5 maxdiffp-counted epochs below), so comparing against
        // filter_state_ would miss it -- there is no filter_state_ yet.
        // last_published_solution_position_ecef_ is the right reference: it
        // is updated from any isValid() publication regardless of filter_
        // initialized_/resets, so it still holds the last trustworthy fix
        // across a cold reinit. Reject a seed that implies covering more
        // than kClasSeedImplausibleSpeedMps in dt: no ground vehicle does,
        // regardless of how well the seed fits its own (possibly biased)
        // measurements.
        if (seed.isValid() && has_last_published_solution_position_) {
            const double dt = has_last_processed_time_
                ? std::max(obs.time - last_processed_time_, 0.001)
                : 1.0;
            const double seed_vs_last_published_m =
                (seed.position_ecef - last_published_solution_position_ecef_)
                    .norm();
            const double implausible_jump_m = std::max(
                kClasSeedImplausibleJumpFloorM,
                kClasSeedImplausibleSpeedMps * dt);
            if (seed_vs_last_published_m > implausible_jump_m) {
                if (pppDebugEnabled()) {
                    std::cerr << "[CLAS-SEED-JUMP] reject tow=" << obs.time.tow
                              << " jump=" << seed_vs_last_published_m
                              << " limit=" << implausible_jump_m
                              << " dt=" << dt << "\n";
                }
                seed.status = SolutionStatus::NONE;
            }
        }
        if (seed.isValid()) {
            clas_last_valid_spp_seed_ = seed;
            has_clas_last_valid_spp_seed_ = true;
            clas_last_rejected_output_position_ecef_ = seed.position_ecef;
            clas_last_rejected_output_time_ = obs.time;
            has_clas_last_rejected_output_ = true;
        }
        // Captured before the output-only coast splice below: this is the
        // guards' true verdict on the raw SPP this epoch
        // (redundancy/chi-square gate, jump gate, or a plain
        // pre-existing SPP failure -- e.g. <4 usable satellites -- all
        // collapse to !seed.isValid() here). Debug evidence on
        // nagoya_run2 (tow 556698-556733, a 176-epoch/35 s continuous
        // chi-square-reject stretch) showed the coast/tt-freeze mechanism
        // above stops position blind-extrapolation but does nothing to
        // stop WLNL AR: ambiguity eligibility is governed purely by
        // lock_count (phase continuity), which is completely decoupled
        // from seed quality, so AR kept resolving and PUBLISHING a stable
        // wrong fix (~12.2 m error, all 176/176 epochs guard-rejected)
        // every single epoch of the coast. Suppress the AR *attempt*
        // itself on any epoch whose own seed is untrusted -- mirroring
        // MRTKLIB's pntpos-failure semantics one step further: a stale/
        // coasted state should not be allowed to originate a new
        // publishable fix, only to keep the float filter alive until a
        // trustworthy seed returns.
        clas_seed_ar_recovery_this_epoch =
            ppp_shared::updateClasSeedArQuarantine(
                clas_baseline_seed_maxdiff_this_epoch,
                kClasSeedArQuarantineEpochs,
                clas_seed_ar_quarantine_epochs_);
        clas_seed_failed_before_continuity_fallback = !seed.isValid();
        clas_seed_untrusted_this_epoch =
            clas_seed_failed_before_continuity_fallback ||
            clas_seed_chi_square_failed ||
            clas_baseline_seed_maxdiff_this_epoch;
        // Suppressing this epoch's AR *attempt* alone is not enough: lock
        // counts are untouched by that gate, so a short (2-3 epoch)
        // rejection window can still leave every ambiguity's lock_count
        // comfortably above the min_lock_count=1 eligibility floor
        // (ppp_ar.cpp:210) the instant a seed is accepted again -- no
        // cooldown. Observed on tokyo_run1 tow 188097.0-188097.4 (~8.45 m,
        // 3 epochs): the seed was accepted there (no CHI2/JUMP reject in
        // the debug log), but a rejection/coast stretch had ended only
        // ~2.2 s (11 epochs) earlier at tow 188092.8, and the filter fixed
        // a wrong integer almost immediately on reconvergence. Reset every
        // tracked ambiguity's lock_count to -minlock on a rejected epoch,
        // mirroring udbias_ppp()'s existing outage-reset semantics
        // (mrtk_ppp_rtk.c ~865-875; the identical -kMrtklibMinLock/
        // outage_count=0 pattern already used for the floatcnt and
        // outage_gap resets elsewhere in this function): this reuses the
        // existing lock_count>=min_lock_count gate to enforce a natural
        // kMrtklibMinLock+1-epoch (6 accepted-phase-epoch) cooldown after
        // the rejected seed becomes trustworthy again.
        // A raw maxdiff sample does not invalidate MRTKLIB's ambiguity locks:
        // AR runs before cntdiffp is updated, and state reset only happens
        // after poserrcnt consecutive excesses. Keep the ambiguity cooldown
        // for an actually failed SPP seed, but let a maxdiff-only recovery
        // epoch reuse the still-valid lock history.
        if (clas_seed_failed_before_continuity_fallback ||
            clas_seed_chi_square_failed) {
            constexpr int kMrtklibMinLock = 5;
            for (auto& [_, ambiguity] : ambiguity_states_) {
                ambiguity.lock_count = -kMrtklibMinLock;
                ambiguity.outage_count = 0;
            }
        }
        // Validation protects the filter from inconsistent or
        // underdetermined solves, but freezing its last good SPP as the
        // public SINGLE position can accumulate hundreds of metres as the
        // receiver keeps moving. Keep the filter rejection unchanged and
        // use the finite candidate only as output when it is continuous with
        // the preceding SPP/output candidate and remains within 150 m of the
        // last trustworthy publication. The first condition rejects abrupt
        // 4.2 km/17 km rank-collapse jumps; the second prevents a sequence of
        // individually small steps from drifting unboundedly (observed on
        // Nagoya 2).
        clas_seed_validation_failed =
            clas_seed_chi_square_failed || clas_seed_dof_failed;
        if (clas_seed_validation_failed &&
            has_clas_last_rejected_output_) {
            const double rejected_dt = std::max(
                obs.time - clas_last_rejected_output_time_, 0.0);
            const double rejected_jump =
                (clas_validation_rejected_candidate.position_ecef -
                 clas_last_rejected_output_position_ecef_).norm();
            const double trusted_distance = has_last_published_solution_position_
                ? (clas_validation_rejected_candidate.position_ecef -
                   last_published_solution_position_ecef_).norm()
                : std::numeric_limits<double>::infinity();
            if (ppp_shared::clasRejectedSeedOutputIsContinuous(
                    clas_validation_rejected_candidate.isValid(),
                    clas_seed_validation_failed, true, rejected_jump,
                    rejected_dt, kClasRejectedOutputJumpFloorM,
                    kClasRejectedOutputSpeedMps, trusted_distance,
                    kClasRejectedOutputTrustedLimitM)) {
                clas_continuity_output = clas_validation_rejected_candidate;
                clas_continuity_output.status = SolutionStatus::SPP;
                clas_continuity_output.velocity_ecef = Vector3d::Zero();
                has_clas_continuity_output = true;
                clas_rejected_seed_output_prepared = true;
                clas_last_rejected_output_position_ecef_ =
                    clas_validation_rejected_candidate.position_ecef;
                clas_last_rejected_output_time_ = obs.time;
            }
        }
        // Coverage fallback for a seed that has no usable current position.
        // mrtk_rtkpos.c:2417-2425: when pntpos() fails, MRTKLIB's caller
        // only skips the epoch outright in the static branch
        // (!rtk->opt.dynamics -> return 0); in dynamics mode (our path) it
        // falls through and keeps running ppp_rtk_pos(). A failure before
        // estpos() converges leaves the prior sol.rr/time in place, whereas
        // the chi-square failure handled above has already written the
        // current position/time. Native has no usable current seed in this
        // branch, so mirror the former case with a continuity row while
        // leaving the invalid seed and filter lifecycle unchanged.
        // A plain masked-admission failure commonly reports fewer than four
        // satellites in native's invalid result. MRTKLIB still coasts in that
        // exact case because an early pntpos() failure leaves the prior
        // sol.rr untouched. Keep the >=4 guard only for native dof/jump
        // rejections, where it prevents manufacturing an epoch without a
        // minimally viable current solve.
        const bool coast_from_last_spp =
            clas_masked_spp_admission_failed &&
            has_clas_last_valid_spp_seed_;
        const bool coast_from_last_published =
            !clas_masked_spp_admission_failed &&
            has_last_published_solution_position_ &&
            ppp_shared::shouldCoastClasSeed(
                false, filter_initialized_,
                static_cast<int>(seed.satellites_used.size()));
        if (!seed.isValid() && !has_clas_continuity_output &&
            (coast_from_last_spp || coast_from_last_published)) {
            if (pppDebugEnabled()) {
                std::cerr << "[CLAS-SEED-COAST] tow=" << obs.time.tow
                          << " coasting to last published position\n";
            }
            PositionSolution coast_seed = seed;
            if (coast_from_last_spp) {
                coast_seed = clas_last_valid_spp_seed_;
                coast_seed.time = obs.time;
            } else {
                coast_seed.position_ecef =
                    last_published_solution_position_ecef_;
                coast_seed.num_satellites =
                    static_cast<int>(coast_seed.satellites_used.size());
            }
            coast_seed.velocity_ecef = Vector3d::Zero();
            coast_seed.status = SolutionStatus::SPP;
            // Preserve MRTKLIB's stale-sol continuity row without feeding it
            // into native's filter. The original invalid seed must continue
            // through the unchanged clear-path lifecycle; only an otherwise
            // invalid return value is spliced below. Injecting the stale code
            // position into either cold initialization or a running filter
            // can originate delayed wrong fixes.
            clas_continuity_output = coast_seed;
            has_clas_continuity_output = coast_seed.isValid();
        }
    } else {
        seed = spp_processor_.processEpoch(obs, nav);
    }
    if (clas_mrtklib_parity && pppDebugEnabled() &&
        (std::abs(std::fmod(obs.time.tow, 3.0)) < 1e-6)) {
        std::cerr << "[CLAS-SPP-SEED] tow=" << obs.time.tow
                  << " rr=" << seed.position_ecef.transpose()
                  << " ns=" << seed.satellites_used.size()
                  << " sats=";
        for (const auto& sat : seed.satellites_used) {
            std::cerr << sat.toString() << ',';
        }
        std::cerr << " rejected=";
        for (const auto& sat : seed.spp_rejected_satellites) {
            std::cerr << sat.toString() << ',';
        }
        std::cerr << '\n';
    }
    clas_mrtklib_ar_rejected_ambiguities_.clear();
    // MRTKLIB mrtk_rtkpos.c ~2395-2416: an early pntpos() failure leaves
    // rtk->sol.time unchanged and therefore freezes tt. Although valsol()
    // chi-square failure retains the current MRTKLIB sol.time/position,
    // admitting that candidate into native's filter regressed Nagoya 2 in
    // the six-run gate. Keep native's prior freeze lifecycle for every seed
    // rejection; only publication uses the bounded current candidate above.
    // Capture "was the filter already running before this epoch" here,
    // before the floatcnt/measurement-update resets below can change
    // filter_initialized_, so the freeze below only ever applies to a
    // genuine mid-stream coast (never a cold/re-init epoch, which already
    // has its own seed-availability handling).
    const bool clas_seed_guard_rejected_mid_stream =
        clas_mrtklib_parity && filter_initialized_ &&
        clas_seed_failed_before_continuity_fallback;
    // Advances clas_last_accepted_seed_time_ alongside every
    // last_processed_time_ update below, except on an epoch the guards
    // above just rejected -- i.e. exactly the pntpos-success gating
    // described above. Called at each of the existing bookkeeping sites
    // regardless of which later stage (OSR availability, measurement
    // update) the epoch ultimately exits through, since MRTKLIB's tt
    // freeze is governed solely by pntpos, not by ppp_rtk_pos()'s own
    // later success/failure.
    const auto clas_update_seed_anchor = [&]() {
        if (clas_mrtklib_parity && !clas_seed_guard_rejected_mid_stream) {
            clas_last_accepted_seed_time_ = obs.time;
            has_clas_last_accepted_seed_time_ = true;
        }
    };
    // The canonical raw-L6 CLASLIB oracle enters ppp_rtk_pos() at the first
    // observation and advances its float/iono/ambiguity states immediately.
    // Its first five epochs are FLOAT and it then fixes; it does not skip a
    // 15-second filter warm-up. Keep only the stream-origin bookkeeping used
    // by diagnostics. Publication quality comes from the filter result.
    if (clas_mrtklib_parity) {
        if (!has_clas_mrtklib_stream_start_time_) {
            clas_mrtklib_stream_start_time_ = obs.time;
            has_clas_mrtklib_stream_start_time_ = true;
        }
    }
    // MRTKLIB v0.5.1 mrtk_ppp_rtk.c:1989-2002, clas.toml float_count=15.
    // On the kinematic path every state is zeroed before udstate_ppp(), which
    // then initializes position from the current SPP solution and recreates
    // ionosphere/ambiguity states. Persistent OSR continuity contexts are not
    // part of x and intentionally survive this reset.
    constexpr int kMrtklibFloatResetEpochs = 15;
    bool clas_mrtklib_floatcnt_reset_this_epoch = false;
    if (clas_mrtklib_parity &&
        clas_mrtklib_float_count_ >= kMrtklibFloatResetEpochs) {
        if (pppDebugEnabled()) {
            std::cerr << "[CLAS-FLOATCNT] reset after "
                      << clas_mrtklib_float_count_ << " FLOAT epochs tow="
                      << obs.time.tow
                      << " spp_rr=" << seed.position_ecef.transpose()
                      << "\n";
        }
        filter_state_ = PPPState{};
        filter_initialized_ = false;
        ambiguity_states_.clear();
        est_stec_outage_.clear();
        clas_dd_accumulator_ = {};
        ppp_ar::clearWlnlHoldState(clas_wlnl_hold_);
        last_clas_constrained_fixed_state_valid_ = false;
        last_ar_ratio_ = 0.0;
        last_fixed_ambiguities_ = 0;
        clas_mrtklib_float_count_ = 0;
        clas_mrtklib_floatcnt_reset_this_epoch = true;
        // Feeds GNSS_PPP_CLAS_POST_RESET_RATIO_FLOOR's settle-window gate;
        // see clas_last_full_state_reset_time_'s declaration in ppp.hpp.
        clas_last_full_state_reset_time_ = obs.time;
        has_clas_last_full_state_reset_time_ = true;
        clas_post_reset_saw_maxdiff_ = false;
        last_clas_post_reset_floor_failed_ = false;
        clas_post_reset_fix_quarantine_ = false;
    }
    // The parity path runs detectClasCycleSlips() below on OSR phase-bias-
    // corrected GF/MW. Running the generic detector first stores raw GF/MW
    // after an LLI reset, so the CLAS detector compares corrected against raw
    // combinations and falsely resets nearly every ambiguity every epoch.
    if (!clas_mrtklib_parity) {
        detectCycleSlips(obs, nav);
    }
    // MRTKLIB literal-port track: clas.toml [kalman_filter.initial_std]
    // bias = 100 cycles (mrtk_ppp_rtk.c:792). The scalar is passed through
    // the shared reset machinery here; the parity path converts freshly
    // reset entries to meter^2 once the per-frequency OSR wavelengths exist.
    // Dynamics-model kinematic CLAS
    // only; other paths keep the historical 3600 / 1e6 values.
    const double clas_ambiguity_initial_variance =
        clas_mrtklib_parity
            ? 1e4
            : (precise_products_loaded_ ? 1e6
                                        : ppp_config_.initial_ambiguity_variance);
    // Feed prepareEpochState()/predictFilterState() the tt-style anchor
    // described above instead of the plain last_processed_time_ bookkeeping
    // whenever this is the parity path: clas_last_accepted_seed_time_ only
    // moves forward on epochs the seed guard accepted (see the six update
    // sites below), so a guard-rejected mid-stream epoch here synthesizes a
    // near-zero dt (freezing position/velocity/clock/iono/ambiguity
    // propagation for exactly this epoch, mirroring tt==0), while the
    // return-to-accepted epoch after one or more rejections naturally sees
    // the full accumulated real gap in one predict step -- exactly as
    // MRTKLIB's own tt does. Non-parity paths (static CLAS anchors,
    // white-noise mode) are untouched: they keep using last_processed_time_
    // directly, unaffected by this guard.
    bool clas_prepare_has_last_processed_time = has_last_processed_time_;
    GNSSTime clas_prepare_last_processed_time = last_processed_time_;
    if (clas_mrtklib_parity) {
        if (clas_seed_guard_rejected_mid_stream) {
            clas_prepare_has_last_processed_time = true;
            clas_prepare_last_processed_time = obs.time - 0.001;
        } else {
            clas_prepare_has_last_processed_time = has_clas_last_accepted_seed_time_;
            clas_prepare_last_processed_time = clas_last_accepted_seed_time_;
        }
    }
    const auto epoch_preparation = ppp_clas::prepareEpochState(
        obs,
        seed,
        ssr_products_,
        filter_state_,
        filter_initialized_,
        convergence_start_time_,
        static_anchor_position_,
        has_static_anchor_position_,
        ppp_config_,
        modeledZenithTroposphereDelayMeters(seed.position_ecef, obs.time),
        clas_prepare_has_last_processed_time,
        clas_prepare_last_processed_time,
        ambiguity_states_,
        clas_dispersion_compensation_,
        clas_phase_bias_repair_,
        clas_ambiguity_initial_variance);
    if (!epoch_preparation.ready) {
        if (allow_hybrid_fallback) {
            return fallback_to_standard("prepare_epoch_state");
        }
        if (has_clas_continuity_output) {
            solution = clas_continuity_output;
            applyOptionalSolutionEpochMetadata(solution, obs.time, ppp_config_);
            // Output-only splice: preserve the exact pre-existing filter
            // lifecycle. Advancing time/counters here changes the dt seen by
            // the next genuinely accepted seed and suppresses later FIX
            // recovery, even though no filter epoch was processed now.
            clear_clas_ar_continuation();
            return solution;
        }
        clear_clas_ar_continuation();
        return solution;
    }

    auto epoch_context = prepareClasEpochContext(
        obs,
        nav,
        ssr_products_,
        filter_state_.state.segment(0, 3),
        filter_state_.state(filter_state_.clock_index),
        filter_state_.state(filter_state_.trop_index),
        ppp_config_,
        windup_cache_,
        clas_dispersion_compensation_,
        clas_sis_continuity_,
        clas_phase_bias_repair_);
    if (clas_mrtklib_parity) {
        int tide_network_id = -1;
        std::array<double, 4> tide_grid_weights{{0.0, 0.0, 0.0, 0.0}};
        bool have_tide_network = readClasAtmosNetworkId(
            epoch_context.epoch_atmos_tokens, tide_network_id);
        bool have_tide_weights = false;
        {
            // The lifecycle matrix can materialize per-satellite atmosphere
            // without retaining a single epoch-level network token. CLASLIB
            // still applies the receiver tide for the selected service area;
            // recover that typed selection from the accepted OSR rows.
            for (const auto& osr : epoch_context.osr_corrections) {
                if (osr.valid && osr.atmos_network_id > 0) {
                    if (!have_tide_network) {
                        tide_network_id = osr.atmos_network_id;
                        have_tide_network = true;
                    }
                    if (osr.atmos_network_id == tide_network_id) {
                        double sum = 0.0;
                        for (double weight : osr.atmos_interpolation_weights) {
                            sum += weight;
                        }
                        if (sum > 0.0) {
                            tide_grid_weights = osr.atmos_interpolation_weights;
                            have_tide_weights = true;
                            break;
                        }
                    }
                }
            }
        }
        if (have_tide_network && have_tide_weights) {
            const Vector3d tide = mrtklibTokyoClasTideDisplacement(
                epoch_context.receiver_position, obs.time, tide_network_id,
                tide_grid_weights);
            epoch_context.receiver_tide_displacement = tide;
            if (pppDebugEnabled() && tide.squaredNorm() > 0.0) {
                std::cerr << "[CLAS-TIDE] tow=" << obs.time.tow
                          << " net=" << tide_network_id
                          << " d=" << tide.transpose() << "\n";
            }
        }
    }
    materializeClasReceiverAntennaCorrections(epoch_context.osr_corrections);
    const auto& epoch_atmos = epoch_context.epoch_atmos_tokens;
    auto& osr_corrections = epoch_context.osr_corrections;

    // Same tt-anchor substitution as the prepareEpochState() call above,
    // applied here so detectClasCycleSlips()'s outage_gap (dt_seconds > 2s)
    // ambiguity-reset path sees the real accumulated gap on the
    // return-to-accepted epoch, exactly like the predict step does --
    // otherwise a multi-epoch coast would leave every ambiguity's lock
    // count untouched even though the filter froze underneath it.
    const double clas_dt_seconds =
        clas_prepare_has_last_processed_time
            ? std::max(obs.time - clas_prepare_last_processed_time, 0.001)
            : 1.0;
    if (ppp_config_.kinematic_mode && ppp_config_.enable_cycle_slip_detection) {
        const auto slip_stats = ppp_clas::detectClasCycleSlips(
            obs,
            osr_corrections,
            ppp_config_,
            clas_dt_seconds,
            filter_state_,
            ambiguity_states_,
            clas_dispersion_compensation_,
            clas_phase_bias_repair_,
            [&](const SatelliteId& satellite, SignalType signal) {
                resetAmbiguity(satellite, signal);
            },
            clas_ambiguity_initial_variance,
            pppDebugEnabled());
        if (slip_stats.total_resets > 0) {
            clas_dd_accumulator_ = {};
        }
        if (clas_mrtklib_parity) {
            // MRTKLIB runs udbias_ppp() before corrmeas().  Recreating a
            // zeroed ambiguity sets ssat.pbreset for that frequency, and
            // compensatedisp() then returns compL=0 for the complete L1/L2
            // pair on this epoch.  Native prepares the OSR corrections
            // before running its slip/outage detector, so discard the
            // already-computed compensation for every satellite whose bias
            // was reset here.  The persistent carrier datum is deliberately
            // retained: pbreset suppresses only the current epoch and normal
            // compensation resumes on the next one.
            for (auto& osr : osr_corrections) {
                if (slip_stats.reset_satellites.count(osr.satellite) != 0) {
                    for (int frequency = 0;
                         frequency < osr.num_frequencies &&
                         frequency < OSR_MAX_FREQ;
                         ++frequency) {
                        // CPC was aggregated while preparing the OSR, before
                        // the later udbias-equivalent reset became known.
                        osr.CPC[frequency] -=
                            osr.phase_compensation_m[frequency];
                    }
                    std::fill(std::begin(osr.phase_compensation_m),
                              std::end(osr.phase_compensation_m), 0.0);
                }
            }
        }
    }

    if (pppEnvOverrides().clas_stec_constraint &&
        ppp_config_.estimate_ionosphere &&
        ppp_config_.use_clas_osr_filter) {
        int network_id = -1;
        if (readClasAtmosNetworkId(epoch_atmos, network_id)) {
            if (has_last_clas_atmos_network_id_ &&
                network_id != last_clas_atmos_network_id_) {
                resetClasIonosphereStateValues(filter_state_);
            }
            last_clas_atmos_network_id_ = network_id;
            has_last_clas_atmos_network_id_ = true;
        }
    }

    if (osr_corrections.size() < 4) {
        if (allow_hybrid_fallback) {
            return fallback_to_standard("insufficient_osr");
        }
        // Dynamics mode: prepareEpochState already ran predictFilterState,
        // which propagated pos += vel*dt and inflated the covariance for
        // this interval, so the processed-time bookkeeping MUST advance
        // before this early return. Otherwise the next epoch's dt spans
        // this interval AGAIN and the same coast is re-applied every epoch:
        // across a low-satellite stretch dt grows unboundedly (observed
        // 2.2 s -> 15+ s at 5 Hz approaching the tokyo_run2 bridge outage)
        // and the repeated vel*dt over-propagation drags the float
        // kilometers away (the 4.5 km post-bridge blunder). White-noise
        // mode keeps historical behavior (its per-epoch SPP re-anchor of
        // position and clock bounds the damage of a stale dt).
        if (ppp_config_.use_dynamics_model) {
            has_last_processed_time_ = true;
            last_processed_time_ = obs.time;
            clas_update_seed_anchor();
        }
        solution = has_clas_continuity_output
            ? clas_continuity_output
            : seed;
        clear_clas_ar_continuation();
        return solution;
    }

    ppp_clas::ensureAmbiguityStates(
        filter_state_, osr_corrections, clas_ambiguity_initial_variance);
    if (clas_mrtklib_parity) {
        constexpr double kMrtklibBiasVarianceCycles2 = 100.0 * 100.0;
        for (const auto& osr : osr_corrections) {
            if (!osr.valid) continue;
            for (int frequency = 0; frequency < osr.num_frequencies && frequency < 2;
                 ++frequency) {
                const double wavelength = osr.wavelengths[frequency];
                if (!(wavelength > 0.0)) continue;
                const SatelliteId ambiguity_satellite(
                    osr.satellite.system,
                    static_cast<uint8_t>(std::min(
                        255, static_cast<int>(osr.satellite.prn) +
                                 (frequency == 0 ? 0 : 100))));
                const int ambiguity_index = ambiguityStateIndex(ambiguity_satellite);
                if (ambiguity_index < 0) continue;
                double& variance =
                    filter_state_.covariance(ambiguity_index, ambiguity_index);
                // A reset can precede predictFilterState(), which adds the
                // tiny bias random walk and makes the sentinel slightly
                // larger than exactly 10000.
                if (variance > 0.9 * kMrtklibBiasVarianceCycles2) {
                    variance = kMrtklibBiasVarianceCycles2 *
                               wavelength * wavelength;
                }
            }
        }
    }
    if (clas_mrtklib_parity &&
        (clas_mrtklib_floatcnt_reset_this_epoch ||
         epoch_preparation.initialized_this_epoch)) {
        // udbias_ppp() recreates every zeroed phase-bias state with
        // lock=-minlock. The accepted observation at the end of this epoch
        // increments it once, so the published lock is -4 for minlock=5.
        // This happens on every whole-filter initialization, not only at a
        // 30-second CSSR phase-bias boundary: udbias_ppp() keys the reset to
        // x[IB]==0 after udpos/udstate, independently of correction timing.
        constexpr int kMrtklibMinLock = 5;
        for (const auto& osr : osr_corrections) {
            if (!osr.valid) continue;
            ambiguity_states_[osr.satellite].lock_count = -kMrtklibMinLock;
            ambiguity_states_[osr.satellite].outage_count = 0;
            const SatelliteId l2_satellite(
                osr.satellite.system,
                static_cast<uint8_t>(std::min(
                    255, static_cast<int>(osr.satellite.prn) + 100)));
            ambiguity_states_[l2_satellite].lock_count = -kMrtklibMinLock;
            ambiguity_states_[l2_satellite].outage_count = 0;
        }
    }
    if (pppEnvOverrides().clas_nl_datum_reset &&
        ppp_config_.enable_ambiguity_resolution &&
        ppp_config_.ar_method == PPPConfig::ARMethod::DD_WLNL &&
        ppp_config_.use_clas_osr_filter &&
        !ppp_config_.use_ionosphere_free &&
        ppp_config_.estimate_ionosphere) {
        if (applyClasNlDatumReset(
                obs.time, osr_corrections, ambiguity_states_, pppDebugEnabled())) {
            clas_dd_accumulator_ = {};
        }
    }
    if (clas_mrtklib_parity &&
        (clas_mrtklib_floatcnt_reset_this_epoch ||
         epoch_preparation.initialized_this_epoch)) {
        // Fresh udbias_ppp() states must be seeded from raw L*lambda-P.
        // A continuity shift queued for the discarded filter would make a
        // zero state non-zero before seeding and falsely mark it initialized.
        for (auto& [_, repair] : clas_phase_bias_repair_) {
            repair.pending_state_shift_cycles = {0.0, 0.0, 0.0};
        }
    }
    ppp_clas::applyPendingPhaseBiasStateShifts(
        filter_state_, osr_corrections, clas_phase_bias_repair_, pppDebugEnabled());

    const auto epoch_update = ppp_clas::runEpochMeasurementUpdate(
        obs,
        epoch_context,
        filter_state_,
        ppp_config_,
        seed,
        ambiguity_states_,
        [&](const Vector3d& receiver_pos, double elevation, const GNSSTime& time) {
            return calculateMappingFunction(receiver_pos, elevation, time);
        },
        [&](const SatelliteId& satellite, SignalType signal) {
            resetAmbiguity(satellite, signal);
        },
        [&](const SatelliteId& satellite) {
            return ambiguityStateIndex(satellite);
        },
        pppDebugEnabled());
    if (!epoch_update.updated) {
        if (allow_hybrid_fallback) {
            return fallback_to_standard("measurement_update");
        }
        if (clas_mrtklib_parity &&
            (epoch_update.insufficient_valid_satellites ||
             epoch_preparation.initialized_this_epoch)) {
            // mrtk_ppp_rtk.c resets every kinematic state when stat remains
            // SOLQ_NONE after the ssat.vsat L1 count check. float_count is
            // neither incremented nor cleared on this SINGLE epoch.
            filter_state_ = PPPState{};
            filter_initialized_ = false;
            ambiguity_states_.clear();
            est_stec_outage_.clear();
            clas_dd_accumulator_ = {};
            ppp_ar::clearWlnlHoldState(clas_wlnl_hold_);
            last_clas_constrained_fixed_state_valid_ = false;
            solution = has_clas_continuity_output
                ? clas_continuity_output
                : seed;
            applyOptionalSolutionEpochMetadata(solution, obs.time, ppp_config_);
            has_last_processed_time_ = true;
            last_processed_time_ = obs.time;
            clas_update_seed_anchor();
            ++total_epochs_processed_;
            clear_clas_ar_continuation();
            return solution;
        }
        // MRTKLIB keeps publishing the predicted FLOAT state when all
        // post-fit DD reference trials are rejected; the epoch is not
        // reclassified as SINGLE merely because filter2_ supplied no new
        // posterior. Keeping the float status also advances float_count and
        // preserves the literal 15-epoch reset cadence.
        if (clas_mrtklib_parity && filter_initialized_) {
            solution = ppp_clas::finalizeEpochSolution(
                filter_state_, obs.time, false, 0.0, 0,
                static_cast<int>(epoch_context.osr_corrections.size()));
            ++clas_mrtklib_float_count_;
            clear_clas_ar_continuation();
            applyOptionalSolutionEpochMetadata(solution, obs.time, ppp_config_);
            has_last_processed_time_ = true;
            last_processed_time_ = obs.time;
            clas_update_seed_anchor();
            ++total_epochs_processed_;
            if (solution.isValid()) {
                last_published_solution_position_ecef_ = solution.position_ecef;
                has_last_published_solution_position_ = true;
            }
            return solution;
        }
        // Same dt-bookkeeping requirement as the insufficient_osr early
        // return above: the predict for this interval already happened.
        if (ppp_config_.use_dynamics_model) {
            has_last_processed_time_ = true;
            last_processed_time_ = obs.time;
            clas_update_seed_anchor();
        }
        solution = has_clas_continuity_output
            ? clas_continuity_output
            : seed;
        clear_clas_ar_continuation();
        return solution;
    }
    dumpClasFloatPosition(obs.time, filter_state_, epoch_update, osr_corrections.size());
    const auto& update_stats = epoch_update.update_stats;
    if (clas_mrtklib_parity) {
        clas_mrtklib_ar_rejected_ambiguities_ =
            update_stats.rejected_phase_ambiguities;
    }
    pre_anchor_covariance_ = update_stats.pre_anchor_covariance;

    if (ppp_config_.use_clas_dd_filter) {
        if (!clas_dd_filter_) {
            clas_dd_filter_ = std::make_unique<ppp_clas_dd::DdFilterScaffold>();
        }
        const PositionSolution native_float_solution = ppp_clas::finalizeEpochSolution(
            filter_state_,
            obs.time,
            false,
            0.0,
            0,
            static_cast<int>(osr_corrections.size()));
        solution = clas_dd_filter_->processFloatUpdate(
            obs,
            epoch_context,
            filter_state_,
            native_float_solution,
            ppp_config_,
            [&](const Vector3d& receiver_pos, double elevation, const GNSSTime& time) {
                return calculateMappingFunction(receiver_pos, elevation, time);
            });
        applyOptionalSolutionEpochMetadata(solution, obs.time, ppp_config_);
        has_last_processed_time_ = true;
        last_processed_time_ = obs.time;
        clas_update_seed_anchor();
        ++total_epochs_processed_;
        clear_clas_ar_continuation();
        return solution;
    }

    // Accumulate Melbourne-Wübbena for WL-NL AR in CLAS per-frequency mode.
    if (ppp_config_.enable_ambiguity_resolution &&
        ppp_config_.ar_method == PPPConfig::ARMethod::DD_WLNL) {
        for (const auto& osr : osr_corrections) {
            if (!osr.valid || osr.num_frequencies < 2) continue;
            const Observation* l1_raw = findOsrFrequencyObservation(obs, osr, 0);
            const Observation* l2_raw = findOsrFrequencyObservation(obs, osr, 1);
            if (!l1_raw || !l2_raw || !l1_raw->valid || !l2_raw->valid) continue;
            if (!l1_raw->has_carrier_phase || !l2_raw->has_carrier_phase) continue;
            if (!l1_raw->has_pseudorange || !l2_raw->has_pseudorange) continue;
            if (ppp_config_.kinematic_mode &&
                ppp_config_.use_clas_osr_filter &&
                ((l1_raw->snr > 0.0 &&
                  ppp_ar::clasKinematicSnrMasked(0, osr.elevation, l1_raw->snr)) ||
                 (l2_raw->snr > 0.0 &&
                  ppp_ar::clasKinematicSnrMasked(1, osr.elevation, l2_raw->snr)))) {
                continue;
            }
            const double f1 = osr.frequencies[0];
            const double f2 = osr.frequencies[1];
            if (f1 <= 0.0 || f2 <= 0.0 || std::abs(f1 - f2) < 1e6) continue;
            const double l1_m = l1_raw->carrier_phase * osr.wavelengths[0]
                              - osr.phase_bias_m[0];
            const double l2_m = l2_raw->carrier_phase * osr.wavelengths[1]
                              - osr.phase_bias_m[1];
            const double p1 = l1_raw->pseudorange - osr.code_bias_m[0];
            const double p2 = l2_raw->pseudorange - osr.code_bias_m[1];
            const double mw_m = (f1 * l1_m - f2 * l2_m) / (f1 - f2)
                              - (f1 * p1 + f2 * p2) / (f1 + f2);
            constexpr double lambda_wl_gps = constants::SPEED_OF_LIGHT / (1575.42e6 - 1227.60e6);
            const double mw_cycles = mw_m / lambda_wl_gps;
            auto& amb = ambiguity_states_[osr.satellite];
            const bool mw_slip = amb.needs_reinitialization;
            if (!mw_slip && amb.mw_count > 0) {
                amb.mw_sum_cycles += mw_cycles;
                amb.mw_count += 1;
                amb.mw_mean_cycles = amb.mw_sum_cycles / amb.mw_count;
            } else {
                amb.mw_sum_cycles = mw_cycles;
                amb.mw_count = 1;
                amb.mw_mean_cycles = mw_cycles;
                amb.wl_is_fixed = false;
            }
        }
    }

    const auto trop_mapping_for_validation =
        [&](const Vector3d& receiver_pos, double elevation, const GNSSTime& time) {
            return calculateMappingFunction(receiver_pos, elevation, time);
        };
    const auto ambiguity_index_for_validation = [&](const SatelliteId& satellite) {
        return ambiguityStateIndex(satellite);
    };

    const Vector3d clas_float_position_ecef =
        filter_state_.state.segment(filter_state_.pos_index, 3);
    const double clas_float_horizontal_sigma_m =
        clasKinematicHorizontalPositionSigmaM(filter_state_);
    const PPPState clas_float_filter_state = filter_state_;
    const auto clas_float_ambiguity_states = ambiguity_states_;

    // MRTKLIB mrtk_ppp_rtk.c:2296-2330 parity: validate the fixed solution with
    // the post-fix DD phase chi-square. Publish FIX only when chisq < thres_fix
    // (5.0) and hold (constrain the float filter toward the fixed DD
    // ambiguities, holdamb()) only when chisq < thres_hold (0.5). AR-failed
    // epochs publish FLOAT and reset the nfix counter; there is no hold-driven
    // FIX publication.
    //
    // Computed here (ahead of the AR call below) so the hold-continuation
    // carve-out that follows can reference it.
    const bool kinematic_clas_wlnl_hold_path =
        ppp_config_.kinematic_mode &&
        ppp_config_.use_clas_osr_filter &&
        ppp_config_.ar_method == PPPConfig::ARMethod::DD_WLNL;

    // Hold-continuation carve-out (parity + kinematic WLNL path only). A
    // maxdiff-only seed rejection -- the extrinsic masked-SPP cross-check
    // disagreed with the filter by more than kMrtklibMaxSppDivergenceM while
    // the seed itself was otherwise valid (no chi-square/dof failure, no
    // jump/coast) -- does not by itself mean the current WLNL fix is wrong:
    // MRTKLIB has no equivalent extrinsic seed check and holds through these
    // windows (see the diagnosis in the commit message). Native was
    // suppressing the AR *attempt* outright on every such epoch, silently
    // dropping an otherwise-good hold to FLOAT.
    //
    // Only continue an *already active* hold: wlnlHoldStillValid() requires
    // clas_wlnl_hold_.active from a prior epoch's low-chisq fix plus no slip
    // on any held satellite this epoch (ambiguity_states_ already reflects
    // this epoch's cycle-slip detection at this point in the function). AR
    // can therefore never originate a brand-new fix through this carve-out:
    // an epoch with no active hold coming in still gets full suppression,
    // exactly as before.
    //
    // Also require a proven track record: the hold must already have
    // survived >= kClasHoldContinuationMinTrackRecordFixes consecutive
    // post-fix chi-square validations *before* this epoch (see the measured
    // n2-vs-t2 evidence at the constant's definition above). hold_age is
    // captured here, ahead of this epoch's own AR/hold processing further
    // below, so it is the as-of-epoch-entry value -- consecutive_fix_count
    // is not touched between here and the read.
    const int clas_maxdiff_hold_cont_entry_divcnt =
        clas_kinematic_spp_divergence_count_;
    const bool clas_seed_maxdiff_only_this_epoch =
        clas_baseline_seed_maxdiff_this_epoch &&
        !clas_seed_failed_before_continuity_fallback &&
        !clas_seed_chi_square_failed_this_epoch;
    const bool clas_maxdiff_hold_cont_hold_valid =
        ppp_ar::wlnlHoldStillValid(clas_wlnl_hold_, ambiguity_states_);
    const int clas_maxdiff_hold_cont_hold_age_nfix =
        clas_wlnl_hold_.consecutive_fix_count;
    const bool clas_maxdiff_hold_continuation_this_epoch =
        !clasMaxdiffHoldContinuationDisabledByEnv() &&
        kinematic_clas_wlnl_hold_path &&
        clas_seed_maxdiff_only_this_epoch &&
        clas_maxdiff_hold_cont_hold_valid &&
        clas_maxdiff_hold_cont_hold_age_nfix >=
            clasHoldContinuationMinTrackRecordFixes();

    const auto ambiguity_resolution =
        ppp_clas::resolveAndValidateAmbiguities(
            filter_state_,
            ambiguity_states_,
            [&]() {
                // clas_seed_untrusted_this_epoch (parity path only; always
                // false otherwise) -- do not let AR originate a new
                // publishable fix from a coasted/stale state; see the long
                // comment at its assignment above. The maxdiff-only
                // hold-continuation carve-out is the sole exception: it lets
                // AR continue validating an already-active, still-valid WLNL
                // hold (see clas_maxdiff_hold_continuation_this_epoch above)
                // instead of blacking out the epoch.
                return ppp_config_.enable_ambiguity_resolution &&
                       (!clas_seed_untrusted_this_epoch ||
                        clas_maxdiff_hold_continuation_this_epoch) &&
                       resolveAmbiguities(obs, nav);
            },
            (ppp_config_.ar_method == PPPConfig::ARMethod::DD_WLNL)
                ? ppp_clas::ValidateFixedSolutionFunction{}
                : ppp_clas::ValidateFixedSolutionFunction{[&]() {
                      return ppp_clas::validateFixedSolution(
                          obs, osr_corrections, filter_state_, ppp_config_,
                          trop_mapping_for_validation,
                          ambiguity_index_for_validation, pppDebugEnabled());
                  }},
            pppDebugEnabled());

    bool clas_kinematic_chisq_rejected = false;
    if (kinematic_clas_wlnl_hold_path &&
        ppp_config_.enable_ambiguity_resolution) {
        if (!ppp_ar::wlnlHoldStillValid(clas_wlnl_hold_, ambiguity_states_)) {
            ppp_ar::clearWlnlHoldState(clas_wlnl_hold_);
        }

        if (ambiguity_resolution.accepted &&
            !ppp_shared::clasRecoveryFixIsSupported(
                clas_seed_ar_recovery_this_epoch &&
                    !clas_maxdiff_hold_continuation_this_epoch,
                last_fixed_ambiguities_,
                last_ar_ratio_, kClasKinematicMinFixRatio)) {
            // Native's minimum-row fixes immediately after a maxdiff event
            // are the remaining wrong-integer mode (Tokyo run2: 24 bad FIX,
            // all nb=6). MRTKLIB's desired recovery begins at nb=8 and the
            // native equivalent at nb=7 with ratio above the normal
            // kinematic publication floor, so keep AR running but reject
            // only the under-supported recovery candidate.
            //
            // clas_maxdiff_hold_continuation_this_epoch exempts this specific
            // fix from that elevated floor: it is not a fresh, evidence-thin
            // recovery attempt but an already-active hold (see its
            // definition above) simply continuing through a maxdiff-only
            // seed disagreement. Every other epoch inside the 30-epoch
            // quarantine window (a genuine cold recovery with no valid hold,
            // or one whose hold broke to a slip) still requires nb>=7 and
            // the elevated ratio floor, unchanged.
            clas_kinematic_chisq_rejected = true;
            if (pppDebugEnabled()) {
                std::cerr << "[CLAS-KIN-RECOVERY] reject nb="
                          << last_fixed_ambiguities_ << " ratio="
                          << last_ar_ratio_ << "\n";
            }
        } else if (ambiguity_resolution.accepted &&
            !last_clas_constrained_fixed_state_valid_) {
            // MRTKLIB publishes FIX only from the constrained xa solution
            // (sol.rr = xa). Without a validated state-DD LAMBDA fix the epoch
            // stays FLOAT instead of labelling the float state as fixed.
            clas_kinematic_chisq_rejected = true;
            if (pppDebugEnabled()) {
                std::cerr << "[CLAS-KIN-CHISQ] no constrained state, demote"
                          << " ratio=" << last_ar_ratio_ << "\n";
            }
        } else if (ambiguity_resolution.accepted) {
            const PPPState& fixed_state_for_validation =
                last_clas_constrained_fixed_state_;
            ppp_clas::FixValidationOptions fix_validation_options;
            fix_validation_options.outlier_sigma_gate =
                kMrtklibPhaseResidualSigmaGate;
            fix_validation_options.mrtklib_chisq_fallback = true;
            // MRTKLIB parity (dynamics path only): the post-fix residual
            // gate/chi-square normalize by the innovation covariance
            // H'*P*H + R formed from the FLOAT posterior covariance
            // (mrtk_ppp_rtk.c:2296-2313 -> filter2_ -> residual_test), not
            // by the measurement variance alone. Under the tight Stage-2
            // varerr an R-only basis rejects every centimeter-level DD
            // residual and drives fix% to zero.
            if (ppp_config_.clas_mrtklib_float_parity &&
                ppp_config_.use_dynamics_model) {
                fix_validation_options.innovation_covariance =
                    &clas_float_filter_state.covariance;
            }
            const auto fix_validation = ppp_clas::validateFixedSolution(
                obs,
                osr_corrections,
                fixed_state_for_validation,
                ppp_config_,
                trop_mapping_for_validation,
                ambiguity_index_for_validation,
                pppDebugEnabled(),
                fix_validation_options);
            const double phase_chisq = fix_validation.phase_chisq;
            if (pppDebugEnabled()) {
                std::cerr << "[CLAS-KIN-CHISQ] phase_chisq=" << phase_chisq
                          << " rows=" << fix_validation.phase_rows
                          << " outliers=" << fix_validation.phase_outlier_rows
                          << " ratio=" << last_ar_ratio_ << "\n";
            }
            if (!std::isfinite(phase_chisq) ||
                phase_chisq >= clasKinematicChiSquareGateM()) {
                clas_kinematic_chisq_rejected = true;
            } else if (clas_ar_attempt_used_reduced_dd_floor_) {
                // A reduced-row candidate is a publication-only bridge over
                // a bounded run of degraded geometry. The ordinary minamb=6
                // path would have remained FLOAT here, resetting rtk->nfix
                // and leaving both the prior hold constraints and float x/P
                // untouched.  Mirror that internal lifecycle even though the
                // validated constrained state is published as FIX.  In
                // particular, never replace a full-row hold with the thinner
                // reduced-row constraint set or feed it back through
                // holdamb().
                //
                // Direct state-DD resolution also marks its accepted
                // participants fixed for downstream/non-direct diagnostics.
                // Restore those metadata flags from the pre-AR snapshot too;
                // the constrained publication does not require them and the
                // minamb=6 baseline would not have changed them.
                ambiguity_states_ = clas_float_ambiguity_states;
                clas_wlnl_hold_.consecutive_fix_count = 0;
                if (pppDebugEnabled()) {
                    std::cerr << "[CLAS-WLNL-HOLD] publication-only reduced-dd"
                              << " rows=" << last_fixed_ambiguities_
                              << " chisq=" << phase_chisq << "\n";
                }
            } else if (phase_chisq < kMrtklibHoldChiSquareGate) {
                std::map<SatelliteId, double> clas_satellite_elevations_rad;
                for (const auto& osr : osr_corrections) {
                    if (osr.valid) {
                        clas_satellite_elevations_rad[osr.satellite] =
                            osr.elevation;
                    }
                }
                ++clas_wlnl_hold_.consecutive_fix_count;
                std::vector<ppp_ar::WlnlHoldConstraint> hold_constraints;
                if (!clas_wlnl_candidate_hold_constraints_.empty()) {
                    hold_constraints = clas_wlnl_candidate_hold_constraints_;
                } else {
                    ppp_ar::buildWlnlHoldConstraints(
                        last_clas_constrained_fixed_state_,
                        ambiguity_states_,
                        clas_satellite_elevations_rad,
                        hold_constraints,
                        ppp_config_.use_dynamics_model &&
                            !ppp_config_.low_dynamics_mode);
                }
                if (clas_wlnl_hold_.consecutive_fix_count >=
                        ppp_ar::kMrtklibMinFixCount &&
                    !hold_constraints.empty()) {
                    clas_wlnl_hold_.constraints = std::move(hold_constraints);
                    clas_wlnl_hold_.active = true;
                    const bool hold_applied = ppp_ar::applyWlnlHoldAmbiguity(
                        filter_state_, clas_wlnl_hold_.constraints);
                    if (pppDebugEnabled()) {
                        std::cerr << "[CLAS-WLNL-HOLD] applied=" << hold_applied
                                  << " constraints="
                                  << clas_wlnl_hold_.constraints.size()
                                  << " nfix="
                                  << clas_wlnl_hold_.consecutive_fix_count
                                  << " chisq=" << phase_chisq << "\n";
                    }
                } else if (pppDebugEnabled()) {
                    std::cerr << "[CLAS-WLNL-HOLD] skipped nfix="
                              << clas_wlnl_hold_.consecutive_fix_count
                              << " chisq=" << phase_chisq << "\n";
                }
            }
        } else {
            // MRTKLIB resets rtk->nfix on every non-FIX epoch
            // (mrtk_ppp_rtk.c:2371).
            clas_wlnl_hold_.consecutive_fix_count = 0;
        }
    }

    if (ambiguity_resolution.accepted &&
        !clas_ar_attempt_used_reduced_dd_floor_ &&
        last_clas_post_reset_floor_failed_) {
        const int max_nfix =
            env_overrides_.clas_post_reset_quarantine_max_nfix;
        const bool nfix_within_limit =
            max_nfix <= 0 ||
            last_fixed_ambiguities_ <= max_nfix;
        if (nfix_within_limit) {
            clas_post_reset_fix_quarantine_ = true;
        }
    }
    if (clas_ar_attempt_used_reduced_dd_floor_) {
        // This per-attempt diagnostic is part of the quarantine lifecycle.
        // The baseline minamb rejection cannot produce it, so do not carry a
        // speculative reduced-row result into a later early-return epoch.
        last_clas_post_reset_floor_failed_ = false;
    }
    const bool clas_post_reset_quarantined_fix_this_epoch =
        ambiguity_resolution.accepted &&
        !clas_kinematic_chisq_rejected &&
        clas_post_reset_fix_quarantine_;
    bool ambiguity_fixed_epoch =
        ambiguity_resolution.accepted &&
        !clas_kinematic_chisq_rejected &&
        !clas_post_reset_quarantined_fix_this_epoch;
    if (clas_post_reset_quarantined_fix_this_epoch &&
        env_overrides_.clas_post_reset_ratio_floor > 0.0) {
        std::cerr << "[CLAS-POST-RESET-QUARANTINE] tow=" << obs.time.tow
                  << " ratio=" << last_ar_ratio_
                  << " nfix=" << last_fixed_ambiguities_ << "\n";
    }
    if (clas_kinematic_chisq_rejected) {
        last_ar_ratio_ = 0.0;
        last_fixed_ambiguities_ = 0;
        ambiguity_states_ = clas_float_ambiguity_states;
        if (clas_ar_attempt_used_reduced_dd_floor_) {
            // The minamb=6 baseline would simply report a non-FIX epoch and
            // retain the historical hold constraints for slip bookkeeping.
            // A failed speculative publication must not tear that hold down.
            clas_wlnl_hold_.consecutive_fix_count = 0;
        } else {
            ppp_ar::clearWlnlHoldState(clas_wlnl_hold_);
        }
        if (pppDebugEnabled()) {
            std::cerr << "[CLAS-KIN-CHISQ] reject fix (chisq >= "
                      << clasKinematicChiSquareGateM() << ")\n";
        }
    }
    if (ambiguity_resolution.rejected_after_fix) {
        last_ar_ratio_ = 0.0;
        last_fixed_ambiguities_ = 0;
        if (ppp_config_.kinematic_mode && ppp_config_.use_clas_osr_filter) {
            ppp_ar::clearWlnlHoldState(clas_wlnl_hold_);
        }
    }

    // CLAS-HOLDCONT-DBG: one line per epoch where the extrinsic masked-SPP
    // seed tripped the maxdiff watchdog, for offline mining of the features
    // that separate a productive hold-continuation activation from the
    // nagoya_run2 destructive one (see the carve-out comment above). Not
    // gated on kinematic_clas_wlnl_hold_path so it also captures maxdiff
    // epochs on paths where the carve-out structurally cannot apply.
    if (pppDebugEnabled() && clas_baseline_seed_maxdiff_this_epoch) {
        std::cerr << "[CLAS-HOLDCONT-DBG] tow=" << obs.time.tow
                  << " overshoot_m="
                  << (clas_baseline_filter_spp_distance_m_this_epoch -
                      kMrtklibMaxSppDivergenceM)
                  << " divcnt_entry=" << clas_maxdiff_hold_cont_entry_divcnt
                  << " streak=" << clas_maxdiff_consecutive_streak_epochs_
                  << " hold_valid=" << clas_maxdiff_hold_cont_hold_valid
                  << " hold_age_nfix=" << clas_maxdiff_hold_cont_hold_age_nfix
                  << " min_track_record=" << clasHoldContinuationMinTrackRecordFixes()
                  << " carve_out_fired=" << clas_maxdiff_hold_continuation_this_epoch
                  << " ar_attempted=" << ambiguity_resolution.attempted
                  << " ar_accepted=" << ambiguity_resolution.accepted
                  << " ratio=" << last_ar_ratio_
                  << " nfix=" << last_fixed_ambiguities_
                  << "\n";
    }

    if (pppDebugEnabled()) {
        ppp_clas::logUpdateSummary(update_stats, osr_corrections.size());
    }

    bool wlnl_fixed_position_ok = false;
    Vector3d wlnl_fixed_position = Vector3d::Zero();
    if (ambiguity_fixed_epoch &&
        ppp_config_.ar_method == PPPConfig::ARMethod::DD_WLNL &&
        !kinematic_clas_wlnl_hold_path) {
        wlnl_fixed_position_ok = solveFixedPosition(obs, nav, wlnl_fixed_position);
        if (pppDebugEnabled() && wlnl_fixed_position_ok) {
            const double shift = (wlnl_fixed_position -
                filter_state_.state.segment(filter_state_.pos_index, 3)).norm();
            std::cerr << "[CLAS-WLNL-FIX] pos_shift=" << shift << "m\n";
        }
    }

    // MRTKLIB publishes the fixed solution from xa/Pa while the float filter
    // x/P is only nudged by holdamb() (mrtk_ppp_rtk.c:2355-2361).
    const bool use_constrained_fixed_state =
        ambiguity_fixed_epoch &&
        ppp_config_.ar_method == PPPConfig::ARMethod::DD_WLNL &&
        last_clas_constrained_fixed_state_valid_ &&
        (env_overrides_.clas_resamb ||
         (ppp_config_.kinematic_mode && ppp_config_.use_clas_osr_filter));
    const PPPState& solution_filter_state =
        use_constrained_fixed_state ? last_clas_constrained_fixed_state_ : filter_state_;

    solution = ppp_clas::finalizeEpochSolution(
        solution_filter_state,
        obs.time,
        ambiguity_fixed_epoch,
        last_ar_ratio_,
        last_fixed_ambiguities_,
        static_cast<int>(osr_corrections.size()));

    if (wlnl_fixed_position_ok &&
        !ppp_config_.kinematic_mode &&
        !env_overrides_.clas_fixed_state_output &&
        !use_constrained_fixed_state) {
        solution.position_ecef = wlnl_fixed_position;
    }

    // Multi-epoch SD AR: accumulate DD float ambiguities over epochs,
    // then fix with LAMBDA when variance is small enough.
    {
        const auto sd_ar_result = ppp_clas_sd::solveMultiEpochSdAr(
            clas_dd_accumulator_,
            obs,
            osr_corrections,
            solution.position_ecef,
            3.0,   // AR ratio threshold
            20,    // Min accumulation epochs before attempting LAMBDA
            pppDebugEnabled());
        const bool sd_ar_fixed =
            sd_ar_result.valid && sd_ar_result.ar_ratio >= 3.0;
        const bool apply_sd_mar_shift_gate =
            ppp_config_.kinematic_mode ||
            env_overrides_.clas_base_clock_parity;
        const bool sd_ar_position_ok =
            !apply_sd_mar_shift_gate ||
            sd_ar_result.position_shift_m <=
                kClasBaseClockParitySdMarMaxPositionShiftM;
        if (sd_ar_fixed && sd_ar_position_ok && !ppp_config_.kinematic_mode) {
            solution.position_ecef = sd_ar_result.position;
            solution.status = SolutionStatus::PPP_FIXED;
        } else if (sd_ar_fixed && pppDebugEnabled()) {
            std::cerr << "[CLAS-SD-MAR] reject"
                      << (ppp_config_.kinematic_mode ? " kinematic" : " parity")
                      << " pos_shift="
                      << sd_ar_result.position_shift_m
                      << " ratio=" << sd_ar_result.ar_ratio << "\n";
        }
    }

    // On the kinematic CLAS WLNL path the MRTKLIB-parity post-fix chi-square
    // gate (plus the maxdiffp guard below) already validates every fixed
    // publication, so the custom float-jump/continuity gates are skipped: the
    // float filter itself carries meter-level error, and a correct fix
    // legitimately jumps away from the previously published float position.
    bool clas_kinematic_fix_rejected = false;
    if (ppp_config_.kinematic_mode &&
        solution.status == SolutionStatus::PPP_FIXED &&
        !(kinematic_clas_wlnl_hold_path && ambiguity_fixed_epoch)) {
        const double dt_seconds =
            has_last_processed_time_ ? obs.time - last_processed_time_ : 0.2;
        const double max_fixed_float_jump_m = clasKinematicMaxFixedFloatJumpM(
            dt_seconds,
            clas_float_horizontal_sigma_m);
        const Vector3d post_ar_filter_position =
            solution_filter_state.state.segment(solution_filter_state.pos_index, 3);
        double fixed_float_jump_m =
            (solution.position_ecef - clas_float_position_ecef).norm();
        if (wlnl_fixed_position_ok) {
            fixed_float_jump_m = std::max(
                fixed_float_jump_m,
                (wlnl_fixed_position - post_ar_filter_position).norm());
        }
        const bool ratio_ok =
            solution.ratio <= 0.0 ||
            solution.ratio >= (kinematic_clas_wlnl_hold_path
                 ? ppp_config_.ar_ratio_threshold
                 : std::max(
                       kClasKinematicMinFixRatio,
                       ppp_config_.ar_ratio_threshold + 0.5));
        const bool wlnl_shift_ok =
            !wlnl_fixed_position_ok ||
            (wlnl_fixed_position - post_ar_filter_position).norm() <=
                kClasKinematicWlnlMaxPositionShiftM;
        double continuity_jump_m = 0.0;
        if (has_last_published_solution_position_) {
            continuity_jump_m = (solution.position_ecef -
                last_published_solution_position_ecef_).norm();
        }
        const bool continuity_ok =
            !has_last_published_solution_position_ ||
            continuity_jump_m <= max_fixed_float_jump_m;
        const bool float_jump_ok =
            std::isfinite(fixed_float_jump_m) &&
            fixed_float_jump_m <= max_fixed_float_jump_m;
        const bool jump_ok =
            wlnl_shift_ok &&
            continuity_ok &&
            float_jump_ok &&
            ratio_ok;
        if (jump_ok) {
            ++clas_kinematic_fix_candidate_streak_;
        } else {
            clas_kinematic_fix_candidate_streak_ = 0;
        }
        const bool minfix_ok =
            clas_kinematic_fix_candidate_streak_ >= kClasKinematicMinFixCount;
        if (!jump_ok || !minfix_ok) {
            clas_kinematic_fix_rejected = true;
            solution.position_ecef = clas_float_position_ecef;
            solution.status = SolutionStatus::PPP_FLOAT;
            solution.ratio = 0.0;
            solution.num_fixed_ambiguities = 0;
            filter_state_ = clas_float_filter_state;
            ambiguity_states_ = clas_float_ambiguity_states;
            ppp_ar::clearWlnlHoldState(clas_wlnl_hold_);
            if (!jump_ok) {
                clas_kinematic_fix_candidate_streak_ = 0;
            }
            if (pppDebugEnabled()) {
                std::cerr << "[CLAS-KIN-FIX] reject"
                          << (!wlnl_shift_ok ? " wlnl_shift" :
                              !continuity_ok ? " continuity" :
                              !float_jump_ok ? " float_jump" :
                              !ratio_ok ? " ratio" :
                              " minfix")
                          << " jump=" << fixed_float_jump_m
                          << " continuity=" << continuity_jump_m
                          << " limit=" << max_fixed_float_jump_m
                          << " ratio=" << solution.ratio
                          << " streak=" << clas_kinematic_fix_candidate_streak_
                          << "\n";
            }
        }
    } else if (ppp_config_.kinematic_mode) {
        clas_kinematic_fix_candidate_streak_ = 0;
    }

    // MRTKLIB maxdiffp guard (mrtk_ppp_rtk.c:2333-2352). MRTKLIB applies it
    // with dynamics on (benchmark clas.toml: dynamics=true + pseudorange_diff);
    // without it the dynamics filter can enter a rejection spiral and coast
    // kilometers away on the velocity states.
    const PositionSolution* clas_maxdiff_seed = nullptr;
    bool clas_maxdiff_uses_validation_rejected_candidate = false;
    if (seed.isValid()) {
        clas_maxdiff_seed = &seed;
    } else {
        double rejected_filter_spp_distance_m =
            std::numeric_limits<double>::quiet_NaN();
        if (has_clas_validation_rejected_candidate && filter_initialized_ &&
            filter_state_.pos_index >= 0 &&
            filter_state_.pos_index + 2 < filter_state_.state.size()) {
            rejected_filter_spp_distance_m =
                (filter_state_.state.segment<3>(filter_state_.pos_index) -
                 seed.position_ecef)
                    .norm();
        }
        if (ppp_shared::clasMaxdiffCanUseValidationRejectedCandidate(
                clas_mrtklib_parity, clas_seed_validation_failed,
                has_clas_validation_rejected_candidate,
                rejected_filter_spp_distance_m,
                kClasRejectedMaxdiffRecoveryM)) {
            // estpos() stores its finite current-epoch solution in sol.rr
            // before MRTKLIB's valsol() chi-square/redundancy rejection.
            // Dynamics still reaches the maxdiffp guard with that position,
            // so cntdiffp can recover a catastrophically stale FLOAT state.
            // The candidate remains excluded from ordinary filter updates and
            // cold starts.
            // Validation only clears seed.status; its finite estpos()-style
            // coordinates and covariance remain available here.
            clas_maxdiff_seed = &seed;
            clas_maxdiff_uses_validation_rejected_candidate = true;
        }
    }
    if (ppp_config_.kinematic_mode &&
        ppp_config_.use_clas_osr_filter &&
        clas_maxdiff_seed != nullptr) {
        const Vector3d float_position =
            filter_state_.state.segment(filter_state_.pos_index, 3);
        const double spp_divergence_m =
            (float_position - clas_maxdiff_seed->position_ecef).norm();
        if (spp_divergence_m > kMrtklibMaxSppDivergenceM) {
            ++clas_kinematic_spp_divergence_count_;
            if (clas_kinematic_spp_divergence_count_ >
                    kMrtklibMaxSppDivergenceEpochs) {
                if (pppDebugEnabled()) {
                    std::cerr << "[CLAS-KIN-MAXDIFFP] reset to SPP dist="
                              << spp_divergence_m
                              << " tow=" << obs.time.tow
                              << " seed="
                              << (clas_maxdiff_uses_validation_rejected_candidate
                                      ? "validation-rejected"
                                      : "accepted")
                              << "\n";
                    // Same tag family as CLAS-HOLDCONT-DBG above: the
                    // divcnt value that just crossed the reset threshold
                    // (pre-reset-to-zero) and the independent maxdiff streak
                    // counter at the moment the teardown fires.
                    std::cerr << "[CLAS-HOLDCONT-DBG-RESET] tow=" << obs.time.tow
                              << " dist=" << spp_divergence_m
                              << " divcnt=" << clas_kinematic_spp_divergence_count_
                              << " streak=" << clas_maxdiff_consecutive_streak_epochs_
                              << "\n";
                }
                // Feeds GNSS_PPP_CLAS_POST_RESET_RATIO_FLOOR's settle-window
                // gate; see clas_last_full_state_reset_time_'s declaration
                // in ppp.hpp. This teardown (maxdiff watchdog / the
                // wlnl-hold reset logged above as CLAS-HOLDCONT-DBG-RESET)
                // is the second of the two full-state reset events tracked.
                clas_last_full_state_reset_time_ = obs.time;
                has_clas_last_full_state_reset_time_ = true;
                clas_post_reset_saw_maxdiff_ = false;
                last_clas_post_reset_floor_failed_ = false;
                clas_post_reset_fix_quarantine_ = false;
                filter_state_.state.segment(filter_state_.pos_index, 3) =
                    clas_maxdiff_seed->position_ecef;
                for (int i = 0; i < 3; ++i) {
                    const int idx = filter_state_.pos_index + i;
                    filter_state_.covariance.row(idx).setZero();
                    filter_state_.covariance.col(idx).setZero();
                    const double spp_variance =
                        clas_maxdiff_seed->position_covariance(i, i) > 0.0
                            ? clas_maxdiff_seed->position_covariance(i, i)
                            : 100.0;
                    filter_state_.covariance(idx, idx) = spp_variance;
                }
                if (ppp_config_.use_dynamics_model &&
                    filter_state_.vel_index >= 0 &&
                    filter_state_.vel_index + 2 < filter_state_.covariance.rows()) {
                    filter_state_.state.segment(filter_state_.vel_index, 3).setZero();
                    for (int i = 0; i < 3; ++i) {
                        const int idx = filter_state_.vel_index + i;
                        filter_state_.covariance.row(idx).setZero();
                        filter_state_.covariance.col(idx).setZero();
                        filter_state_.covariance(idx, idx) =
                            clas_mrtklib_parity
                                ? 1.0
                                : ppp_config_.initial_velocity_variance;
                    }
                }
                if (clas_mrtklib_parity &&
                    filter_state_.accel_index >= 0 &&
                    filter_state_.accel_index + 2 <
                        filter_state_.covariance.rows()) {
                    filter_state_.state.segment(filter_state_.accel_index, 3)
                        .setConstant(1e-6);
                    for (int i = 0; i < 3; ++i) {
                        const int idx = filter_state_.accel_index + i;
                        filter_state_.covariance.row(idx).setZero();
                        filter_state_.covariance.col(idx).setZero();
                        filter_state_.covariance(idx, idx) = 1.0;
                    }
                }
                solution.position_ecef = clas_maxdiff_seed->position_ecef;
                solution.status = SolutionStatus::SPP;
                solution.ratio = 0.0;
                solution.num_fixed_ambiguities = 0;
                clas_kinematic_fix_rejected = false;
                clas_kinematic_fix_candidate_streak_ = 0;
                clas_kinematic_spp_divergence_count_ = 0;
                ppp_ar::clearWlnlHoldState(clas_wlnl_hold_);
                if (clas_mrtklib_parity && ppp_config_.use_dynamics_model) {
                    // Drop the stale ambiguity/ionosphere state attached to
                    // the pre-reset position. AR stays quarantined through
                    // the recovery window above while the filter rebuilds.
                    filter_state_ = PPPState{};
                    filter_initialized_ = false;
                    ambiguity_states_.clear();
                    est_stec_outage_.clear();
                    clas_dd_accumulator_ = {};
                    last_clas_constrained_fixed_state_valid_ = false;
                    last_ar_ratio_ = 0.0;
                    last_fixed_ambiguities_ = 0;
                }
            }
        } else {
            clas_kinematic_spp_divergence_count_ = 0;
        }
    }

    had_fixed_last_epoch_ =
        solution.status == SolutionStatus::PPP_FIXED && !clas_kinematic_fix_rejected;
    clas_reduced_dd_fix_last_epoch_ =
        had_fixed_last_epoch_ && clas_ar_attempt_used_reduced_dd_floor_;
    clas_full_dd_fix_last_epoch_ =
        had_fixed_last_epoch_ &&
        !clas_ar_attempt_used_reduced_dd_floor_ &&
        last_fixed_ambiguities_ >= 6;
    if (clas_reduced_dd_fix_last_epoch_) {
        ++clas_reduced_dd_publication_streak_;
    } else {
        clas_reduced_dd_publication_streak_ = 0;
    }

    if (clas_mrtklib_parity) {
        if (clas_reduced_dd_fix_last_epoch_) {
            // The reduced-row FIX is output-only.  Count it as the FLOAT that
            // the minamb=6 baseline would have produced so the 15-epoch
            // reinitialization cadence, and therefore all later filter state,
            // remains unchanged.
            ++clas_mrtklib_float_count_;
        } else if (solution.status == SolutionStatus::PPP_FIXED ||
            clas_post_reset_quarantined_fix_this_epoch) {
            // Publication-only quarantine must not count as an internal FLOAT
            // or trigger a lifecycle reset earlier than the accepted AR/hold
            // baseline would.
            clas_mrtklib_float_count_ = 0;
        } else if (solution.status == SolutionStatus::PPP_FLOAT) {
            ++clas_mrtklib_float_count_;
        }
    }

    has_last_processed_time_ = true;
    last_processed_time_ = obs.time;
    clas_update_seed_anchor();
    ++total_epochs_processed_;

    applyOptionalSolutionEpochMetadata(solution, obs.time, ppp_config_);
    const bool publishing_rejected_seed_output =
        clas_rejected_seed_output_prepared &&
        solution.status == SolutionStatus::SPP &&
        (solution.position_ecef - clas_continuity_output.position_ecef).norm() <
            1e-4;
    if (ppp_config_.kinematic_mode && solution.isValid() &&
        !publishing_rejected_seed_output) {
        last_published_solution_position_ecef_ = solution.position_ecef;
        has_last_published_solution_position_ = true;
    }
    return solution;
}

}  // namespace libgnss
