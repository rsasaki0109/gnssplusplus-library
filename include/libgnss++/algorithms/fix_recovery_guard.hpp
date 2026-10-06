#pragma once

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <libgnss++/core/solution.hpp>

namespace libgnss {

/** Optional runtime residual containment. Neither reference positions nor
 * covariance confidence are used to decide whether to report FIX. Clean
 * recovery is evidence of ordinary residual behavior, not proof of accuracy. */
class FixRecoveryGuard {
public:
    enum class State { NORMAL, SUSPECT, QUARANTINE, RECOVERY };
    enum Reason : std::uint32_t {
        NONE = 0, HARD_RESIDUAL = 1, REPEATED_PREFIT = 2,
        WAITING_FOR_CLEAN_FIX = 4, INPUT_GAP = 8
    };
    struct Config {
        bool enabled = false;
        int suspect_epochs = 2;
        int clean_epochs = 5;
        double max_epoch_gap_s = 1.0;
        double hard_post_rms_m = 4.0;
        double hard_nis_per_observation = 50.0;
        double suspect_prefit_rms_m = 10.0;
        double suspect_outlier_fraction = 0.5;
        double suspect_max_ratio = 6.0;
        int clean_min_satellites = 8;
        double clean_min_ratio = 3.0;
        double clean_max_prefit_rms_m = 5.0;
        double clean_max_post_rms_m = 2.0;
        double clean_max_nis_per_observation = 10.0;
        double clean_max_outlier_fraction = 0.2;
    };
    struct Decision {
        State state = State::NORMAL;
        bool demote_fixed = false;
        bool request_primary_reset = false;
        bool recovered = false;
        int suspect_streak = 0;
        int clean_streak = 0;
        std::uint32_t reasons = NONE;
    };
    FixRecoveryGuard() = default;
    explicit FixRecoveryGuard(Config config) : config_(config) {
        if (config.suspect_epochs < 1 || config.clean_epochs < 1 ||
            !positive(config.max_epoch_gap_s) || !positive(config.hard_post_rms_m) ||
            !positive(config.hard_nis_per_observation) || !positive(config.suspect_prefit_rms_m) ||
            !fraction(config.suspect_outlier_fraction) || !positive(config.suspect_max_ratio) ||
            config.clean_min_satellites < 4 || !positive(config.clean_min_ratio) ||
            !positive(config.clean_max_prefit_rms_m) || !positive(config.clean_max_post_rms_m) ||
            !positive(config.clean_max_nis_per_observation) || !fraction(config.clean_max_outlier_fraction))
            throw std::invalid_argument("invalid FIX recovery guard configuration");
    }
    Decision update(const PositionSolution& solution, const GNSSTime& observation_time) {
        if (!config_.enabled) return Decision{};
        if (observation_time.week < 0 || !std::isfinite(observation_time.tow) ||
            observation_time.tow < 0 || observation_time.tow >= 604800 ||
            (have_time_ && observation_time <= last_time_))
            throw std::invalid_argument("FIX recovery requires increasing normalized observation times");
        Decision decision;
        if (have_time_ && observation_time - last_time_ > config_.max_epoch_gap_s) {
            suspect_streak_ = clean_streak_ = 0;
            if (state_ == State::RECOVERY) state_ = State::QUARANTINE;
            if (state_ == State::SUSPECT) state_ = State::NORMAL;
            decision.reasons |= INPUT_GAP;
        }
        last_time_ = observation_time;
        have_time_ = true;
        const bool fixed = solution.isValid() && solution.isFixed();
        const double prefit = solution.rtk_update_prefit_residual_rms_m;
        const double post = solution.rtk_update_post_suppression_residual_rms_m;
        const double nis = solution.rtk_update_normalized_innovation_squared_per_observation;
        const double suppression = solution.rtk_update_observations > 0 &&
            solution.rtk_update_suppressed_outliers >= 0 ?
            static_cast<double>(solution.rtk_update_suppressed_outliers) / solution.rtk_update_observations :
            std::numeric_limits<double>::quiet_NaN();
        const bool hard = fixed && ((std::isfinite(post) && post > config_.hard_post_rms_m) ||
                                   (std::isfinite(nis) && nis > config_.hard_nis_per_observation));
        const bool suspect = fixed && std::isfinite(prefit) && prefit > config_.suspect_prefit_rms_m &&
            std::isfinite(suppression) && suppression >= config_.suspect_outlier_fraction &&
            std::isfinite(solution.ratio) && solution.ratio < config_.suspect_max_ratio;
        const bool clean = fixed && solution.position_ecef.allFinite() &&
            solution.num_satellites >= config_.clean_min_satellites &&
            std::isfinite(solution.ratio) && solution.ratio >= config_.clean_min_ratio &&
            std::isfinite(prefit) && prefit >= 0.0 && prefit <= config_.clean_max_prefit_rms_m &&
            std::isfinite(post) && post >= 0.0 && post <= config_.clean_max_post_rms_m &&
            std::isfinite(nis) && nis >= 0.0 && nis <= config_.clean_max_nis_per_observation &&
            std::isfinite(suppression) && suppression <= config_.clean_max_outlier_fraction;
        if (hard || suspect) {
            clean_streak_ = 0;
            if (suspect_streak_ < config_.suspect_epochs) ++suspect_streak_;
            decision.reasons |= hard ? HARD_RESIDUAL : REPEATED_PREFIT;
            const bool already_quarantined = state_ == State::QUARANTINE || state_ == State::RECOVERY;
            if (hard || suspect_streak_ >= config_.suspect_epochs || already_quarantined) {
                state_ = State::QUARANTINE;
                decision.request_primary_reset = !already_quarantined;
            } else state_ = State::SUSPECT;
        } else if (state_ == State::QUARANTINE || state_ == State::RECOVERY) {
            suspect_streak_ = 0;
            clean_streak_ = clean ? clean_streak_ + 1 : 0;
            state_ = clean ? State::RECOVERY : State::QUARANTINE;
            if (clean_streak_ >= config_.clean_epochs) {
                state_ = State::NORMAL;
                clean_streak_ = 0;
                decision.recovered = true;
            } else decision.reasons |= WAITING_FOR_CLEAN_FIX;
        } else {
            state_ = State::NORMAL;
            suspect_streak_ = clean_streak_ = 0;
        }
        decision.state = state_;
        decision.demote_fixed = fixed && state_ != State::NORMAL;
        decision.suspect_streak = suspect_streak_;
        decision.clean_streak = clean_streak_;
        return decision;
    }
    void reset() { state_ = State::NORMAL; suspect_streak_ = clean_streak_ = 0; have_time_ = false; }
private:
    static bool positive(double x) { return std::isfinite(x) && x > 0.0; }
    static bool fraction(double x) { return std::isfinite(x) && x >= 0.0 && x <= 1.0; }
    Config config_;
    State state_ = State::NORMAL;
    int suspect_streak_ = 0, clean_streak_ = 0;
    bool have_time_ = false;
    GNSSTime last_time_;
};
} // namespace libgnss
