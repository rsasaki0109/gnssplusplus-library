#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <limits>

namespace libgnss::safe_fix {

enum class State {
    DISABLED = 0,
    IDLE = 1,
    ACQUIRING = 2,
    FIXED = 3,
    HELD = 4,
    REVOKED = 5,
};

struct Config {
    bool enabled = false;
    bool require_independent_failure_budget = false;
    int acquisition_streak_epochs = 12;
    double maximum_epoch_gap_s = 0.21;
    double maximum_acquisition_correction_jump_m = 0.03;
    int maximum_hold_epochs = 100;
    double maximum_hold_correction_jump_m = 0.06;
    double maximum_hold_nis_per_observation = 50.0;
    double maximum_hold_prefit_residual_rms_m = 50.0;
    int minimum_hold_pairs = 16;
    double minimum_absolute_ratio = 0.0;
    double maximum_independent_consensus_delta_m =
        std::numeric_limits<double>::infinity();
    bool allow_strong_instant_acquisition = false;
    bool allow_change_point_acquisition = false;
    double minimum_change_point_jump_m = 0.40;
    double maximum_change_point_stable_jump_m = 0.02;
    int change_point_streak_epochs = 3;
};

struct Candidate {
    double time_s = 0.0;
    bool acquisition_eligible = false;
    Eigen::Vector3d correction_m =
        Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    double nis_per_observation = std::numeric_limits<double>::quiet_NaN();
    double prefit_residual_rms_m =
        std::numeric_limits<double>::quiet_NaN();
    int pair_count = 0;
    double ambiguity_ratio = std::numeric_limits<double>::quiet_NaN();
    double independent_consensus_delta_m =
        std::numeric_limits<double>::quiet_NaN();
    bool strong_acquisition_eligible = false;
    bool change_point_acquisition_eligible = false;
    bool independent_failure_budget_passed = false;
};

struct Decision {
    State state = State::DISABLED;
    bool declared_fixed = false;
    bool candidate_accepted = false;
    bool held = false;
    bool revoked = false;
    bool strong_acquisition = false;
    bool change_point_acquisition = false;
    bool independent_failure_budget_passed = false;
    int acquisition_streak = 0;
    int hold_epochs = 0;
};

class StateMachine {
public:
    Decision update(const Config& config, const Candidate& candidate) {
        if (!config.enabled) {
            reset();
            return {};
        }
        const bool finite_time = std::isfinite(candidate.time_s);
        const bool finite_correction = candidate.correction_m.allFinite();
        const bool eligible =
            finite_time && finite_correction &&
            candidate.acquisition_eligible &&
            (!config.require_independent_failure_budget ||
             candidate.independent_failure_budget_passed) &&
            (config.minimum_absolute_ratio <= 0.0 ||
             (std::isfinite(candidate.ambiguity_ratio) &&
              candidate.ambiguity_ratio >=
                  config.minimum_absolute_ratio)) &&
            (!std::isfinite(
                 config.maximum_independent_consensus_delta_m) ||
             (std::isfinite(
                  candidate.independent_consensus_delta_m) &&
              candidate.independent_consensus_delta_m <=
                  config.maximum_independent_consensus_delta_m));
        const bool hold_quality =
            finite_time && finite_correction &&
            std::isfinite(candidate.nis_per_observation) &&
            candidate.nis_per_observation <=
                config.maximum_hold_nis_per_observation &&
            std::isfinite(candidate.prefit_residual_rms_m) &&
            candidate.prefit_residual_rms_m <=
                config.maximum_hold_prefit_residual_rms_m &&
            candidate.pair_count >= config.minimum_hold_pairs;
        const bool change_point_candidate_eligible =
            candidate.change_point_acquisition_eligible &&
            (!config.require_independent_failure_budget ||
             candidate.independent_failure_budget_passed);

        Decision decision;
        decision.independent_failure_budget_passed =
            candidate.independent_failure_budget_passed;
        const bool was_declared =
            state_ == State::FIXED || state_ == State::HELD;
        bool change_point_ready = false;
        if (!was_declared && config.allow_change_point_acquisition &&
            finite_time && finite_correction) {
            const bool contiguous =
                have_change_point_previous_ &&
                candidate.time_s > change_point_previous_time_s_ &&
                candidate.time_s - change_point_previous_time_s_ <=
                    config.maximum_epoch_gap_s;
            const double jump =
                have_change_point_previous_
                    ? (candidate.correction_m -
                       change_point_previous_correction_)
                          .norm()
                    : 0.0;
            if (have_change_point_previous_ && contiguous &&
                jump >= config.minimum_change_point_jump_m) {
                change_point_armed_ = true;
                change_point_streak_ =
                    change_point_candidate_eligible ? 1 : 0;
            } else if (
                change_point_candidate_eligible &&
                change_point_armed_ && contiguous &&
                jump <= config.maximum_change_point_stable_jump_m) {
                ++change_point_streak_;
            } else if (!change_point_candidate_eligible) {
                change_point_streak_ = 0;
            } else {
                // A time gap or an unstable correction breaks the evidence
                // chain.  Keep the detector armed, but require a new fully
                // contiguous stable streak before declaring FIX.
                change_point_streak_ = 0;
            }
            change_point_previous_time_s_ = candidate.time_s;
            change_point_previous_correction_ = candidate.correction_m;
            have_change_point_previous_ = true;
            change_point_ready =
                change_point_armed_ &&
                change_point_streak_ >=
                    std::max(1, config.change_point_streak_epochs);
        } else if (was_declared) {
            clearChangePoint();
        }
        if (was_declared) {
            const bool fixed_continuity =
                eligible &&
                candidate.time_s > last_declared_time_s_ &&
                candidate.time_s - last_declared_time_s_ <=
                    config.maximum_epoch_gap_s &&
                (candidate.correction_m - last_fixed_correction_).norm() <=
                    config.maximum_acquisition_correction_jump_m;
            if (fixed_continuity) {
                state_ = State::FIXED;
                hold_epochs_ = 0;
                last_fixed_correction_ = candidate.correction_m;
                last_declared_time_s_ = candidate.time_s;
                decision.declared_fixed = true;
                decision.candidate_accepted = true;
            } else if (
                hold_quality && config.maximum_hold_epochs > 0 &&
                hold_epochs_ < config.maximum_hold_epochs &&
                candidate.time_s > last_declared_time_s_ &&
                candidate.time_s - last_declared_time_s_ <=
                    config.maximum_epoch_gap_s &&
                (candidate.correction_m - last_fixed_correction_).norm() <=
                    config.maximum_hold_correction_jump_m) {
                state_ = State::HELD;
                ++hold_epochs_;
                last_fixed_correction_ = candidate.correction_m;
                last_declared_time_s_ = candidate.time_s;
                decision.declared_fixed = true;
                decision.held = true;
            } else {
                state_ = State::REVOKED;
                decision.revoked = true;
                clearAcquisition();
                hold_epochs_ = 0;
            }
        } else if (
            change_point_ready &&
            (!config.require_independent_failure_budget ||
             candidate.independent_failure_budget_passed)) {
            state_ = State::FIXED;
            acquisition_streak_ = change_point_streak_;
            last_fixed_correction_ = candidate.correction_m;
            last_declared_time_s_ = candidate.time_s;
            hold_epochs_ = 0;
            decision.declared_fixed = true;
            decision.candidate_accepted = true;
            decision.change_point_acquisition = true;
            clearChangePoint();
        } else if (eligible) {
            if (config.allow_strong_instant_acquisition &&
                candidate.strong_acquisition_eligible) {
                state_ = State::FIXED;
                acquisition_streak_ = 1;
                previous_candidate_time_s_ = candidate.time_s;
                previous_candidate_correction_ = candidate.correction_m;
                have_previous_candidate_ = true;
                last_fixed_correction_ = candidate.correction_m;
                last_declared_time_s_ = candidate.time_s;
                hold_epochs_ = 0;
                decision.declared_fixed = true;
                decision.candidate_accepted = true;
                decision.strong_acquisition = true;
                decision.state = state_;
                decision.acquisition_streak = acquisition_streak_;
                decision.hold_epochs = hold_epochs_;
                return decision;
            }
            const bool contiguous =
                have_previous_candidate_ &&
                candidate.time_s > previous_candidate_time_s_ &&
                candidate.time_s - previous_candidate_time_s_ <=
                    config.maximum_epoch_gap_s &&
                (candidate.correction_m - previous_candidate_correction_)
                        .norm() <=
                    config.maximum_acquisition_correction_jump_m;
            acquisition_streak_ = contiguous ? acquisition_streak_ + 1 : 1;
            previous_candidate_time_s_ = candidate.time_s;
            previous_candidate_correction_ = candidate.correction_m;
            have_previous_candidate_ = true;
            if (acquisition_streak_ >=
                std::max(1, config.acquisition_streak_epochs)) {
                state_ = State::FIXED;
                last_fixed_correction_ = candidate.correction_m;
                last_declared_time_s_ = candidate.time_s;
                hold_epochs_ = 0;
                decision.declared_fixed = true;
                decision.candidate_accepted = true;
            } else {
                state_ = State::ACQUIRING;
            }
        } else {
            state_ = State::IDLE;
            clearAcquisition();
        }
        decision.state = state_;
        decision.acquisition_streak = acquisition_streak_;
        decision.hold_epochs = hold_epochs_;
        return decision;
    }

    void reset() {
        state_ = State::DISABLED;
        clearAcquisition();
        hold_epochs_ = 0;
        last_fixed_correction_.setZero();
        last_declared_time_s_ = 0.0;
        clearChangePoint();
    }

private:
    void clearAcquisition() {
        acquisition_streak_ = 0;
        have_previous_candidate_ = false;
        previous_candidate_time_s_ = 0.0;
        previous_candidate_correction_.setZero();
    }

    void clearChangePoint() {
        change_point_armed_ = false;
        change_point_streak_ = 0;
        have_change_point_previous_ = false;
        change_point_previous_time_s_ = 0.0;
        change_point_previous_correction_.setZero();
    }

    State state_ = State::DISABLED;
    int acquisition_streak_ = 0;
    int hold_epochs_ = 0;
    bool have_previous_candidate_ = false;
    double previous_candidate_time_s_ = 0.0;
    Eigen::Vector3d previous_candidate_correction_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d last_fixed_correction_ = Eigen::Vector3d::Zero();
    double last_declared_time_s_ = 0.0;
    bool change_point_armed_ = false;
    int change_point_streak_ = 0;
    bool have_change_point_previous_ = false;
    double change_point_previous_time_s_ = 0.0;
    Eigen::Vector3d change_point_previous_correction_ =
        Eigen::Vector3d::Zero();
};

}  // namespace libgnss::safe_fix
