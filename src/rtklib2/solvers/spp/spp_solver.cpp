#include "rtklib2/solvers/spp/spp_solver.hpp"

#include <algorithm>
#include <utility>

namespace rtklib2::solvers::spp {

namespace {

[[nodiscard]] SolutionQuality map_quality(libgnss::SolutionStatus status) {
    switch (status) {
        case libgnss::SolutionStatus::SPP:
            return SolutionQuality::SPP;
        case libgnss::SolutionStatus::DGPS:
            return SolutionQuality::DGPS;
        case libgnss::SolutionStatus::FLOAT:
            return SolutionQuality::Float;
        case libgnss::SolutionStatus::FIXED:
            return SolutionQuality::Fixed;
        case libgnss::SolutionStatus::PPP_FLOAT:
            return SolutionQuality::PPPFloat;
        case libgnss::SolutionStatus::PPP_FIXED:
            return SolutionQuality::PPPFixed;
        case libgnss::SolutionStatus::NONE:
        default:
            return SolutionQuality::None;
    }
}

class LegacySppSolver final : public SppSolver {
public:
    LegacySppSolver() = default;
    ~LegacySppSolver() override = default;

    void configure(const SolverConfig& config) override {
        config_ = config;
        processor_.setSPPConfig(config_.spp_config);
        processor_.initialize(config_.processor_config);
        configured_ = true;
    }

    void reset() override {
        processor_.reset();
    }

    SolverResult solve_epoch(const libgnss::ObservationData& observation,
                             const SolverContext& context) override {
        ensure_configured();

        SolverResult result{};
        result.solution = processor_.processEpoch(observation, context.navigation);
        populate_metrics(result);
        populate_summary(result);
        return result;
    }

private:
    void ensure_configured() {
        if (!configured_) {
            configure(SolverConfig{});
        }
    }

    void populate_metrics(SolverResult& result) const {
        result.metrics.pdop = result.solution.pdop;
        result.metrics.hdop = result.solution.hdop;
        result.metrics.vdop = result.solution.vdop;
        result.metrics.gdop = result.solution.gdop;
        result.metrics.residual_rms = result.solution.residual_rms;
        result.metrics.iterations = result.solution.iterations;
    }

    void populate_summary(SolverResult& result) const {
        result.summary.quality = map_quality(result.solution.status);
        result.summary.receiver_clock_bias = result.solution.receiver_clock_bias;
        result.summary.receiver_clock_drift = result.solution.receiver_clock_drift;
        result.summary.satellites_used = static_cast<std::int32_t>(result.solution.num_satellites);
        result.summary.satellite_elevations = result.solution.satellite_elevations;
    }

    SolverConfig config_{};
    libgnss::SPPProcessor processor_{};
    bool configured_{false};
};

}  // namespace

SppSolverPtr make_legacy_spp_solver() {
    return std::make_shared<LegacySppSolver>();
}

}  // namespace rtklib2::solvers::spp
