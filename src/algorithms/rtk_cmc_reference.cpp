#include <libgnss++/algorithms/rtk_cmc_reference.hpp>

#include <cmath>

namespace libgnss {
namespace rtk_cmc_reference {

CmcSuspectTracker::CmcSuspectTracker(double level_threshold_m, int warmup_epochs,
                                     double baseline_alpha)
    : level_threshold_m_(level_threshold_m),
      warmup_epochs_(warmup_epochs),
      baseline_alpha_(baseline_alpha) {}

bool CmcSuspectTracker::classify(const SatelliteId& sat, double cmc_m, bool arc_restarted) {
    auto& state = baseline_by_sat_[sat];
    if (arc_restarted) {
        state = BaselineState{};
    }
    if (!std::isfinite(cmc_m)) {
        return false;
    }
    if (!state.has_baseline) {
        state.baseline_m = cmc_m;
        state.warmup_count = 1;
        state.has_baseline = true;
        return false;
    }
    if (state.warmup_count < warmup_epochs_) {
        state.baseline_m = (state.baseline_m * state.warmup_count + cmc_m) / (state.warmup_count + 1);
        ++state.warmup_count;
        return false;
    }
    if (level_threshold_m_ <= 0.0) {
        return false;
    }
    if (std::abs(cmc_m - state.baseline_m) > level_threshold_m_) {
        // Suspect epoch: do not fold this sample into the baseline (mirrors
        // FGOProcessor's CMC level-exclusion handling), so a sustained
        // multipath episode cannot drag the baseline toward it.
        return true;
    }
    state.baseline_m = (1.0 - baseline_alpha_) * state.baseline_m + baseline_alpha_ * cmc_m;
    return false;
}

void CmcSuspectTracker::pruneMissing(const std::set<SatelliteId>& seen) {
    for (auto it = baseline_by_sat_.begin(); it != baseline_by_sat_.end();) {
        if (seen.count(it->first) == 0) {
            it = baseline_by_sat_.erase(it);
        } else {
            ++it;
        }
    }
}

bool ReferenceHysteresis::update(const std::vector<Candidate>& candidates,
                                 const SatelliteId& natural_ref,
                                 double natural_ref_elevation_rad, int switch_epochs,
                                 double return_min_elev_rad, SatelliteId& out_ref,
                                 bool& switched, double switch_away_max_elev_drop_rad,
                                 double switch_away_min_elev_rad) {
    switched = false;
    if (candidates.empty()) {
        return false;
    }

    auto findCandidate = [&](const SatelliteId& sat) -> const Candidate* {
        for (const auto& c : candidates) {
            if (c.satellite == sat) return &c;
        }
        return nullptr;
    };

    const bool had_ref_before = has_current_ref_;
    const SatelliteId previous_ref = current_ref_;
    const Candidate* current = has_current_ref_ ? findCandidate(current_ref_) : nullptr;

    if (!has_current_ref_ || current == nullptr) {
        // Bootstrap or hard reacquisition: no tracked reference yet, or the
        // tracked reference is no longer an eligible candidate this epoch
        // at all (dropped below the elevation mask / lost lock entirely).
        // CMC hysteresis governs voluntary switches away from a still-
        // visible-but-suspect reference only -- it must never block
        // reacquisition when the satellite is simply gone.
        current_ref_ = natural_ref;
        has_current_ref_ = true;
        suspect_streak_ = 0;
        has_return_candidate_ = false;
        return_candidate_clean_streak_ = 0;
        out_ref = current_ref_;
        // Only count as a "switch" if this replaces a previously tracked
        // reference with a different satellite; the very first acquisition
        // for a fresh instance is not a switch.
        switched = had_ref_before && !(previous_ref == current_ref_);
        return true;
    }

    // 1) Switch-away: only after `switch_epochs` consecutive suspect
    // epochs for the CURRENT reference.
    suspect_streak_ = current->suspect ? suspect_streak_ + 1 : 0;
    if (suspect_streak_ >= switch_epochs) {
        const Candidate* best_dual = nullptr;
        const Candidate* best_l1 = nullptr;
        for (const auto& c : candidates) {
            if (c.suspect) continue;
            if (best_l1 == nullptr || c.elevation_rad > best_l1->elevation_rad) {
                best_l1 = &c;
            }
            if (c.dual_frequency &&
                (best_dual == nullptr || c.elevation_rad > best_dual->elevation_rad)) {
                best_dual = &c;
            }
        }
        const Candidate* clean = best_dual != nullptr ? best_dual : best_l1;
        // Elevation-quality gate: a switch-away is only worth its cost (see
        // rtk_cmc_reference.hpp's update() doc comment) if the replacement
        // is both close in elevation to the suspect reference it displaces
        // and itself high enough to keep the DD group's atmospheric
        // mismatch small. current->elevation_rad is the suspect reference's
        // own elevation this epoch (found above, before this block).
        const bool passes_elev_gate =
            clean != nullptr &&
            clean->elevation_rad >= switch_away_min_elev_rad &&
            (current->elevation_rad - clean->elevation_rad) <= switch_away_max_elev_drop_rad;
        if (clean != nullptr && !(clean->satellite == current_ref_) && passes_elev_gate) {
            current_ref_ = clean->satellite;
            suspect_streak_ = 0;
            has_return_candidate_ = false;
            return_candidate_clean_streak_ = 0;
        }
        // else: every non-suspect candidate is already the current
        // reference, none exist (whole group suspect), or the best
        // candidate failed the elevation-quality gate -- keep current.
    }

    // 2) Switch-back: only once the natural (CMC-blind) pick has been
    // non-suspect for `switch_epochs` consecutive epochs AND clears the
    // elevation margin over wherever step 1 left the current reference.
    const Candidate* current_after = findCandidate(current_ref_);
    const double current_elev_rad = current_after != nullptr ? current_after->elevation_rad : -1.0;
    if (!(natural_ref == current_ref_) &&
        natural_ref_elevation_rad > current_elev_rad + return_min_elev_rad) {
        const Candidate* natural_candidate = findCandidate(natural_ref);
        const bool natural_suspect = natural_candidate == nullptr || natural_candidate->suspect;
        if (has_return_candidate_ && return_candidate_ == natural_ref) {
            return_candidate_clean_streak_ = natural_suspect ? 0 : return_candidate_clean_streak_ + 1;
        } else {
            return_candidate_ = natural_ref;
            has_return_candidate_ = true;
            return_candidate_clean_streak_ = natural_suspect ? 0 : 1;
        }
        if (return_candidate_clean_streak_ >= switch_epochs) {
            current_ref_ = natural_ref;
            suspect_streak_ = 0;
            has_return_candidate_ = false;
            return_candidate_clean_streak_ = 0;
        }
    } else {
        has_return_candidate_ = false;
        return_candidate_clean_streak_ = 0;
    }

    out_ref = current_ref_;
    switched = !(previous_ref == current_ref_);
    return true;
}

}  // namespace rtk_cmc_reference
}  // namespace libgnss
