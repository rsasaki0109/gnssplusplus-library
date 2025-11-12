#pragma once

#include <memory>

#include "libgnss++/algorithms/spp.hpp"
#include "rtklib2/solvers/spp/spp_types.hpp"

namespace rtklib2::solvers::spp {

struct SolverConfig {
    libgnss::ProcessorConfig processor_config{};
    libgnss::SPPProcessor::SPPConfig spp_config{};
};

struct SolverContext {
    libgnss::NavigationData navigation;
};

struct SolverResult {
    libgnss::PositionSolution solution;
    SolutionMetrics metrics;
    SolutionSummary summary;
};

class SppSolver {
public:
    virtual ~SppSolver() = default;

    virtual void configure(const SolverConfig& config) = 0;

    virtual void reset() = 0;

    virtual SolverResult solve_epoch(const libgnss::ObservationData& observation,
                                     const SolverContext& context) = 0;
};

using SppSolverPtr = std::shared_ptr<SppSolver>;

SppSolverPtr make_legacy_spp_solver();

}  // namespace rtklib2::solvers::spp
