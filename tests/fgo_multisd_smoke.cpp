#include <libgnss++/algorithms/fgo.hpp>
#include <libgnss++/core/constants.hpp>

#include <array>
#include <cmath>
#include <iostream>
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

    const auto result = FGOProcessor(config).optimizeProblem(problem);
    if (!result.diagnostics.lambda_ambiguity_fix_solved ||
        !result.diagnostics.lambda_ambiguity_fix_used ||
        !result.diagnostics.fixed_solution) {
        return fail("BSD LAMBDA candidate was not accepted");
    }
    if (result.diagnostics.fixed_ambiguities != 5 ||
        result.diagnostics.lambda_top_k_generated != 4) {
        return fail("unexpected BSD/top-K candidate count");
    }
    if (!(result.diagnostics.lambda_bootstrapped_success_rate > 0.0) ||
        !std::isfinite(result.diagnostics.lambda_adop_cycles) ||
        !(result.diagnostics.lambda_adop_cycles > 0.0)) {
        return fail("invalid success-rate or ADOP diagnostics");
    }
    if (result.ambiguity_estimates.size() != satellites.size()) {
        return fail("missing SD ambiguity estimates");
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

    std::cout << "MultiSD smoke passed: BSD=5 topK=4 success_rate="
              << result.diagnostics.lambda_bootstrapped_success_rate
              << " ADOP=" << result.diagnostics.lambda_adop_cycles << '\n';
    return 0;
}
