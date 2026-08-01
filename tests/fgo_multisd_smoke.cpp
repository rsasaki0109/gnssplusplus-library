#include <libgnss++/algorithms/fgo.hpp>
#include <libgnss++/core/constants.hpp>

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

using namespace libgnss;

namespace {

double ddGeometry(const Vector3d& rover,
                  const Vector3d& base,
                  const Vector3d& satellite,
                  const Vector3d& reference) {
    return ((satellite - rover).norm() - (satellite - base).norm()) -
           ((reference - rover).norm() - (reference - base).norm());
}

int fail(const char* message) {
    std::cerr << "MultiSD smoke failure: " << message << '\n';
    return 1;
}

}  // namespace

int main() {
    const std::vector<Vector3d> satellites = {
        {15600000.0, 7540000.0, 20140000.0},
        {-18760000.0, 2750000.0, 18610000.0},
        {17610000.0, -14630000.0, 13480000.0},
        {19170000.0, 610000.0, -18390000.0},
        {-13480000.0, -15600000.0, 17760000.0},
        {21700000.0, 13000000.0, 9000000.0},
        {-21100000.0, 8100000.0, -13200000.0},
        {9200000.0, 22400000.0, 10600000.0},
        {-6400000.0, 17100000.0, 19300000.0},
        {24100000.0, -7400000.0, 6300000.0},
    };
    std::vector<Vector3d> rover_positions;
    rover_positions.reserve(80);
    const Vector3d first_rover(1113194.0, -4841695.0, 3985350.0);
    const Vector3d epoch_motion(2.5, 1.0, -0.6);
    for (int epoch = 0; epoch < 80; ++epoch) {
        rover_positions.push_back(first_rover + epoch * epoch_motion);
    }
    const Vector3d base_position =
        first_rover + Vector3d(-320.0, 180.0, 45.0);
    const double wavelength = constants::GPS_L1_WAVELENGTH;

    FGOProcessor::FGOProblem problem;
    for (std::size_t sat = 0; sat < satellites.size(); ++sat) {
        FGOProcessor::AmbiguityState ambiguity;
        ambiguity.satellite =
            SatelliteId(GNSSSystem::GPS,
                        static_cast<uint8_t>(sat + 1));
        ambiguity.signal = SignalType::GPS_L1CA;
        ambiguity.wavelength_m = wavelength;
        ambiguity.initial_ambiguity_m =
            wavelength * static_cast<double>(96 + sat);
        ambiguity.is_double_difference = false;
        problem.ambiguity_states.push_back(ambiguity);
    }

    for (std::size_t epoch = 0; epoch < rover_positions.size(); ++epoch) {
        FGOProcessor::EpochSeed seed;
        seed.time = GNSSTime(2300, 100000.0 + epoch);
        seed.position_ecef =
            rover_positions[epoch] + Vector3d(35.0, -18.0, 12.0);
        problem.epochs.push_back(seed);

        for (std::size_t sat = 1; sat < satellites.size(); ++sat) {
            const double geometry = ddGeometry(
                rover_positions[epoch], base_position, satellites[sat],
                satellites[0]);

            FGOProcessor::DoubleDifferencePseudorangeFactor code;
            code.epoch_index = epoch;
            code.satellite = SatelliteId(
                GNSSSystem::GPS, static_cast<uint8_t>(sat + 1));
            code.reference_satellite = SatelliteId(GNSSSystem::GPS, 1);
            code.rover_satellite_position_ecef = satellites[sat];
            code.rover_reference_position_ecef = satellites[0];
            code.base_satellite_position_ecef = satellites[sat];
            code.base_reference_position_ecef = satellites[0];
            code.base_position_ecef = base_position;
            code.observed_dd_pseudorange_m = geometry;
            code.sigma_m = 0.5;
            problem.double_difference_pseudorange_factors.push_back(code);

            FGOProcessor::DoubleDifferenceCarrierFactor carrier;
            carrier.epoch_index = epoch;
            carrier.ambiguity_index = sat;
            carrier.reference_ambiguity_index = 0;
            carrier.use_ambiguity_difference = true;
            carrier.satellite = code.satellite;
            carrier.reference_satellite = code.reference_satellite;
            carrier.rover_satellite_position_ecef = satellites[sat];
            carrier.rover_reference_position_ecef = satellites[0];
            carrier.base_satellite_position_ecef = satellites[sat];
            carrier.base_reference_position_ecef = satellites[0];
            carrier.base_position_ecef = base_position;
            carrier.observed_dd_carrier_m =
                geometry + wavelength * static_cast<double>(sat);
            carrier.sigma_m = 0.01;
            problem.double_difference_carrier_factors.push_back(carrier);
        }
    }

    FGOProcessor::FGOConfig config;
    config.max_iterations = 12;
    config.convergence_threshold_m = 1e-8;
    config.use_motion_factors = false;
    config.use_multisd_ambiguities = true;
    config.fix_ambiguities = true;
    config.use_lambda_ambiguity_fix = true;
    config.lambda_top_k_candidates = 4;
    config.lambda_ratio_threshold = 1.5;
    config.min_fixed_ambiguities = 4;
    config.ambiguity_prior_sigma_m = 1000.0;
    config.fixed_ambiguity_sigma_m = 1e-4;
    config.use_multisd_disjoint_validation = true;
    config.multisd_validation_holdout_satellites = 4;
    config.multisd_validation_max_fixed_float_separation_m = 0.0;

    const auto result = FGOProcessor(config).optimizeProblem(problem);
#ifdef GNSSPP_TEST_HAS_CUDA_FGO
    const char* cuda_mode = std::getenv("GNSSPP_FGO_CUDA_SOLVER");
    const bool forced_cuda = cuda_mode != nullptr &&
                             (std::string(cuda_mode) == "on" ||
                              std::string(cuda_mode) == "1");
    if (forced_cuda &&
        (!result.diagnostics.cuda_dense_solver_selected ||
         result.diagnostics.cuda_dense_solve_attempts == 0 ||
         result.diagnostics.cuda_dense_solve_successes !=
             result.diagnostics.cuda_dense_solve_attempts ||
         result.diagnostics.cuda_dense_solve_fallbacks != 0 ||
         !(result.diagnostics.cuda_dense_solve_time_ms > 0.0))) {
        return fail("forced CUDA solve was not exercised successfully");
    }
    if (!forced_cuda &&
        (result.diagnostics.cuda_dense_solver_selected ||
         result.diagnostics.cuda_dense_solve_attempts != 0)) {
        return fail("CUDA solve was unexpectedly selected");
    }
#endif
    if (!result.diagnostics.lambda_ambiguity_fix_solved ||
        !result.diagnostics.lambda_ambiguity_fix_used ||
        !result.diagnostics.fixed_solution) {
        return fail("BSD LAMBDA candidate was not accepted");
    }
    if (result.diagnostics.fixed_ambiguities != 5 ||
        result.diagnostics.lambda_top_k_generated != 4) {
        return fail("unexpected BSD/top-K candidate count");
    }
    if (!result.diagnostics.multisd_validation_evaluated ||
        !result.diagnostics.multisd_validation_pass ||
        result.diagnostics.multisd_validation_holdout_satellites != 4 ||
        result.diagnostics.multisd_validation_carrier_used < 4 ||
        result.diagnostics.multisd_validation_pseudorange_used < 4) {
        return fail("disjoint validation did not pass clean holdout data");
    }
    if (!std::isfinite(
            result.diagnostics
                .multisd_validation_fixed_float_separation_m) ||
        !(result.diagnostics.multisd_validation_fixed_float_separation_m >= 0.0)) {
        return fail("fixed/float separation diagnostic was not recorded");
    }
    if (!(result.diagnostics.lambda_bootstrapped_success_rate > 0.0) ||
        !std::isfinite(result.diagnostics.lambda_adop_cycles) ||
        !(result.diagnostics.lambda_adop_cycles > 0.0)) {
        return fail("invalid success-rate or ADOP diagnostics");
    }
    if (result.ambiguity_estimates.size() != satellites.size()) {
        return fail("missing SD ambiguity estimates");
    }

    auto constellation_config = config;
    constellation_config.use_constellation_ranked_partial_ar = true;
    const auto constellation_result =
        FGOProcessor(constellation_config).optimizeProblem(problem);
    if (!constellation_result.diagnostics.fixed_solution ||
        !constellation_result.diagnostics.multisd_validation_pass ||
        constellation_result.solution.solutions.empty() ||
        result.solution.solutions.empty() ||
        (constellation_result.solution.solutions.back().position_ecef -
         result.solution.solutions.back().position_ecef).norm() > 1e-9) {
        return fail("constellation PAR changed the clean all-GPS solution");
    }

    auto parallel_config = config;
    parallel_config.parallelize_lambda_hypotheses = true;
    const auto parallel_result =
        FGOProcessor(parallel_config).optimizeProblem(problem);
    if (parallel_result.diagnostics.fixed_solution !=
            result.diagnostics.fixed_solution ||
        parallel_result.diagnostics.multisd_validation_pass !=
            result.diagnostics.multisd_validation_pass ||
        parallel_result.diagnostics.multisd_validation_selected_rank !=
            result.diagnostics.multisd_validation_selected_rank ||
        parallel_result.multisd_validation_hypotheses.size() !=
            result.multisd_validation_hypotheses.size() ||
        parallel_result.solution.solutions.empty() ||
        result.solution.solutions.empty() ||
        (parallel_result.solution.solutions.back().position_ecef -
         result.solution.solutions.back().position_ecef).norm() > 1e-9) {
        return fail("parallel top-K result differs from sequential result");
    }
    for (const auto& estimate : result.ambiguity_estimates) {
        if (estimate.is_fixed) {
            return fail("gauge-dependent SD state was labelled fixed");
        }
    }
    for (std::size_t sat = 1; sat < satellites.size(); ++sat) {
        const double bsd =
            result.ambiguity_estimates[sat].ambiguity_cycles -
            result.ambiguity_estimates[0].ambiguity_cycles;
        if (std::abs(bsd - static_cast<double>(sat)) > 1e-4) {
            return fail("fixed BSD does not match integer truth");
        }
    }

    // PRN 2 and 3 are the deterministic holdouts (all elevations tie, so the
    // satellite-id tie-break decides). Corrupting only their latest carrier
    // evidence cannot change the float/LAMBDA candidate, but must veto FIX.
    auto corrupt_problem = problem;
    for (auto& carrier : corrupt_problem.double_difference_carrier_factors) {
        if (carrier.epoch_index + 1 == rover_positions.size() &&
            carrier.satellite == SatelliteId(GNSSSystem::GPS, 2)) {
            carrier.observed_dd_carrier_m += 0.35 * wavelength;
        }
        if (carrier.epoch_index + 1 == rover_positions.size() &&
            carrier.satellite == SatelliteId(GNSSSystem::GPS, 3)) {
            carrier.observed_dd_carrier_m -= 0.31 * wavelength;
        }
        if (carrier.epoch_index + 1 == rover_positions.size() &&
            carrier.satellite == SatelliteId(GNSSSystem::GPS, 4)) {
            carrier.observed_dd_carrier_m += 0.27 * wavelength;
        }
        if (carrier.epoch_index + 1 == rover_positions.size() &&
            carrier.satellite == SatelliteId(GNSSSystem::GPS, 5)) {
            carrier.observed_dd_carrier_m -= 0.23 * wavelength;
        }
    }
    const auto rejected = FGOProcessor(config).optimizeProblem(corrupt_problem);
    if (!rejected.diagnostics.multisd_validation_evaluated ||
        rejected.diagnostics.multisd_validation_pass ||
        rejected.diagnostics.fixed_solution ||
        rejected.diagnostics.lambda_ambiguity_fix_used) {
        return fail("independent carrier corruption was not rejected");
    }

    // A single latest-epoch carrier outlier is recoverable only when a causal
    // history supplies enough disjoint evidence. Persistent/multi-satellite
    // corruption above remains rejected.
    auto isolated_outlier_problem = problem;
    for (auto& carrier :
         isolated_outlier_problem.double_difference_carrier_factors) {
        if (carrier.epoch_index + 1 == rover_positions.size() &&
            carrier.satellite == SatelliteId(GNSSSystem::GPS, 2)) {
            carrier.observed_dd_carrier_m += 0.35 * wavelength;
        }
    }
    auto temporal_config = config;
    temporal_config.multisd_validation_history_epochs = 3;
    temporal_config.multisd_validation_min_carrier_fraction = 0.75;
    const auto temporal = FGOProcessor(temporal_config).optimizeProblem(
        isolated_outlier_problem);
    if (!temporal.diagnostics.multisd_validation_evaluated ||
        !temporal.diagnostics.multisd_validation_pass ||
        !temporal.diagnostics.fixed_solution ||
        temporal.diagnostics.multisd_validation_carrier_used < 12 ||
        temporal.diagnostics.multisd_validation_carrier_passed < 9) {
        return fail("causal carrier history did not tolerate one outlier");
    }

    auto insufficient_config = config;
    insufficient_config.multisd_validation_holdout_satellites = 20;
    const auto insufficient =
        FGOProcessor(insufficient_config).optimizeProblem(problem);
    if (insufficient.diagnostics.multisd_validation_evaluated ||
        insufficient.diagnostics.multisd_validation_pass ||
        insufficient.diagnostics.fixed_solution) {
        return fail("insufficient holdout evidence did not fail closed");
    }

    std::cout << "MultiSD smoke passed: BSD=5 topK=4 success_rate="
              << result.diagnostics.lambda_bootstrapped_success_rate
              << " ADOP=" << result.diagnostics.lambda_adop_cycles << '\n';
    return 0;
}
