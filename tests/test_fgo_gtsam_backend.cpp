#ifdef GNSSPP_HAS_GTSAM

#include <gtest/gtest.h>
#include <libgnss++/algorithms/fgo.hpp>
#include <libgnss++/core/constants.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <vector>

// Parity harness for Phase 1 of the GTSAM backend (docs/gtsam_backend_design.md):
// builds one synthetic double-difference RTK FGOProblem with fully populated
// raw (undifferenced) observation debug fields -- required because the GTSAM
// DD factors are built from the 4 raw rover/base x ref/target observations,
// not from the precomputed observed_dd_*_m scalar alone -- and runs it
// through both FGOProcessor backends, asserting the float ECEF solutions and
// DD residual RMS agree to well within the sub-cm target.

using namespace libgnss;

namespace {

std::vector<Vector3d> gtsamParitySatelliteGeometry() {
    return {
        Vector3d(15600000.0, 7540000.0, 20140000.0),
        Vector3d(-18760000.0, 2750000.0, 18610000.0),
        Vector3d(17610000.0, -14630000.0, 13480000.0),
        Vector3d(19170000.0, 610000.0, -18390000.0),
        Vector3d(-13480000.0, -15600000.0, 17760000.0),
        Vector3d(21700000.0, 13000000.0, 9000000.0),
    };
}

// Builds a two-epoch, single-reference-satellite DD RTK problem where every
// raw ObservationModelDebug field is populated consistently with how
// FGOProcessor::buildDoubleDifferenceProblem fills them in production, so
// the GTSAM backend's 4-raw-observation reconstruction of the DD factors has
// something real to reconstruct from (see the observed-DD assertion in
// fgo_gtsam_backend.cpp).
FGOProcessor::FGOProblem makeGtsamParityDoubleDifferenceProblem() {
    FGOProcessor::FGOProblem problem;
    const auto satellites = gtsamParitySatelliteGeometry();
    const double wavelength = constants::GPS_L1_WAVELENGTH;

    const std::array<Vector3d, 2> true_positions = {
        Vector3d(1113194.0, -4841695.0, 3985350.0),
        Vector3d(1113196.8, -4841692.5, 3985351.1),
    };
    const std::array<double, 2> rover_clock_bias_m = {42.0, 43.7};
    const double base_clock_bias_m = 11.0;
    const Vector3d base_position = true_positions[0] + Vector3d(-320.0, 180.0, 45.0);

    // One DD ambiguity state per non-reference satellite (index sat - 1),
    // held constant across both epochs (no cycle slip in this synthetic set).
    for (std::size_t sat = 1; sat < satellites.size(); ++sat) {
        FGOProcessor::AmbiguityState ambiguity;
        ambiguity.satellite = SatelliteId(GNSSSystem::GPS, static_cast<uint8_t>(sat + 1));
        ambiguity.reference_satellite = SatelliteId(GNSSSystem::GPS, 1);
        ambiguity.signal = SignalType::GPS_L1CA;
        ambiguity.wavelength_m = wavelength;
        ambiguity.is_double_difference = true;
        const int true_dd_cycles = 100 + static_cast<int>(sat);
        // Seed off the true value so the float solve has real work to do.
        ambiguity.initial_ambiguity_m = (static_cast<double>(true_dd_cycles) - 0.6) * wavelength;
        problem.ambiguity_states.push_back(ambiguity);
    }

    for (std::size_t epoch = 0; epoch < true_positions.size(); ++epoch) {
        FGOProcessor::EpochSeed seed;
        seed.time = GNSSTime(2300, 100000.0 + static_cast<double>(epoch));
        seed.position_ecef = true_positions[epoch] + Vector3d(12.0, -7.0, 5.0);
        seed.receiver_clock_bias_m = rover_clock_bias_m[epoch];
        problem.epochs.push_back(seed);

        const Vector3d& rover_pos = true_positions[epoch];
        const double rover_ref_range = (satellites[0] - rover_pos).norm();
        const double base_ref_range = (satellites[0] - base_position).norm();

        FGOProcessor::ObservationModelDebug rover_ref_model;
        rover_ref_model.corrected_pseudorange_m = rover_ref_range + rover_clock_bias_m[epoch];
        rover_ref_model.corrected_carrier_m = rover_ref_range + rover_clock_bias_m[epoch];

        FGOProcessor::ObservationModelDebug base_ref_model;
        base_ref_model.corrected_pseudorange_m = base_ref_range + base_clock_bias_m;
        base_ref_model.corrected_carrier_m = base_ref_range + base_clock_bias_m;

        for (std::size_t sat = 1; sat < satellites.size(); ++sat) {
            const double rover_target_range = (satellites[sat] - rover_pos).norm();
            const double base_target_range = (satellites[sat] - base_position).norm();
            const int true_dd_cycles = 100 + static_cast<int>(sat);

            FGOProcessor::ObservationModelDebug rover_target_model;
            rover_target_model.corrected_pseudorange_m =
                rover_target_range + rover_clock_bias_m[epoch];
            rover_target_model.corrected_carrier_m =
                rover_target_range + rover_clock_bias_m[epoch] +
                static_cast<double>(true_dd_cycles) * wavelength;

            FGOProcessor::ObservationModelDebug base_target_model;
            base_target_model.corrected_pseudorange_m = base_target_range + base_clock_bias_m;
            base_target_model.corrected_carrier_m = base_target_range + base_clock_bias_m;

            FGOProcessor::DoubleDifferencePseudorangeFactor pr_factor;
            pr_factor.epoch_index = epoch;
            pr_factor.satellite = SatelliteId(GNSSSystem::GPS, static_cast<uint8_t>(sat + 1));
            pr_factor.reference_satellite = SatelliteId(GNSSSystem::GPS, 1);
            pr_factor.signal = SignalType::GPS_L1CA;
            pr_factor.rover_satellite_position_ecef = satellites[sat];
            pr_factor.rover_reference_position_ecef = satellites[0];
            pr_factor.base_satellite_position_ecef = satellites[sat];
            pr_factor.base_reference_position_ecef = satellites[0];
            pr_factor.base_position_ecef = base_position;
            pr_factor.rover_satellite_model = rover_target_model;
            pr_factor.rover_reference_model = rover_ref_model;
            pr_factor.base_satellite_model = base_target_model;
            pr_factor.base_reference_model = base_ref_model;
            pr_factor.observed_dd_pseudorange_m =
                (rover_target_model.corrected_pseudorange_m -
                 base_target_model.corrected_pseudorange_m) -
                (rover_ref_model.corrected_pseudorange_m -
                 base_ref_model.corrected_pseudorange_m);
            pr_factor.sigma_m = 0.5;
            pr_factor.elevation_rad = 0.7;
            problem.double_difference_pseudorange_factors.push_back(pr_factor);

            FGOProcessor::DoubleDifferenceCarrierFactor cp_factor;
            cp_factor.epoch_index = epoch;
            cp_factor.ambiguity_index = sat - 1;
            cp_factor.use_ambiguity_difference = false;
            cp_factor.satellite = pr_factor.satellite;
            cp_factor.reference_satellite = pr_factor.reference_satellite;
            cp_factor.signal = SignalType::GPS_L1CA;
            cp_factor.rover_satellite_position_ecef = satellites[sat];
            cp_factor.rover_reference_position_ecef = satellites[0];
            cp_factor.base_satellite_position_ecef = satellites[sat];
            cp_factor.base_reference_position_ecef = satellites[0];
            cp_factor.base_position_ecef = base_position;
            cp_factor.rover_satellite_model = rover_target_model;
            cp_factor.rover_reference_model = rover_ref_model;
            cp_factor.base_satellite_model = base_target_model;
            cp_factor.base_reference_model = base_ref_model;
            cp_factor.observed_dd_carrier_m =
                (rover_target_model.corrected_carrier_m -
                 base_target_model.corrected_carrier_m) -
                (rover_ref_model.corrected_carrier_m - base_ref_model.corrected_carrier_m);
            cp_factor.sigma_m = 0.01;
            cp_factor.elevation_rad = 0.7;
            problem.double_difference_carrier_factors.push_back(cp_factor);
        }
    }

    problem.diagnostics.input_epochs = true_positions.size();
    problem.diagnostics.seeded_epochs = true_positions.size();
    problem.diagnostics.double_difference_matched_base_epochs = true_positions.size();
    problem.diagnostics.double_difference_candidate_pairs =
        problem.double_difference_carrier_factors.size();
    return problem;
}

FGOProcessor::FGOConfig makeParityBaseConfig() {
    FGOProcessor::FGOConfig config;
    config.use_double_difference_factors = true;
    config.use_carrier_phase_factors = false;
    config.use_pseudorange_factors = false;  // no undifferenced factors in this problem
    config.use_tdcp_factors = false;
    config.use_single_difference_doppler_factors = false;
    config.use_single_difference_tdcp_factors = false;
    config.use_ambiguity_priors = true;
    config.ambiguity_prior_sigma_m = 50.0;
    config.fix_ambiguities = false;
    config.use_lambda_ambiguity_fix = false;
    config.max_iterations = 15;
    return config;
}

}  // namespace

TEST(FGOGtsamBackendTest, FloatSolutionMatchesEigenBackendOnDoubleDifferenceProblem) {
    const FGOProcessor::FGOProblem problem = makeGtsamParityDoubleDifferenceProblem();

    FGOProcessor::FGOConfig eigen_config = makeParityBaseConfig();
    eigen_config.backend = FGOBackend::Eigen;

    FGOProcessor::FGOConfig gtsam_config = makeParityBaseConfig();
    gtsam_config.backend = FGOBackend::GTSAM;

    FGOProcessor eigen_processor(eigen_config);
    FGOProcessor gtsam_processor(gtsam_config);

    const FGOProcessor::FGOResult eigen_result = eigen_processor.optimizeProblem(problem);
    const FGOProcessor::FGOResult gtsam_result = gtsam_processor.optimizeProblem(problem);

    ASSERT_EQ(eigen_result.solution.size(), problem.epochs.size());
    ASSERT_EQ(gtsam_result.solution.size(), problem.epochs.size());

    double max_delta_m = 0.0;
    double sum_sq_delta = 0.0;
    for (std::size_t i = 0; i < problem.epochs.size(); ++i) {
        const Vector3d eigen_pos = eigen_result.solution.solutions[i].position_ecef;
        const Vector3d gtsam_pos = gtsam_result.solution.solutions[i].position_ecef;
        const double delta = (eigen_pos - gtsam_pos).norm();
        max_delta_m = std::max(max_delta_m, delta);
        sum_sq_delta += delta * delta;
    }
    const double rms_delta_m =
        std::sqrt(sum_sq_delta / static_cast<double>(problem.epochs.size()));

    // Emit the parity numbers so the harness output carries the deliverable
    // metric (max/RMS per-epoch ECEF delta between the two float solutions).
    std::fprintf(stderr,
                 "[parity] Eigen-vs-GTSAM float ECEF delta: max=%.6g m rms=%.6g m "
                 "(epochs=%zu)\n",
                 max_delta_m, rms_delta_m, problem.epochs.size());

    EXPECT_LT(max_delta_m, 0.01) << "max ECEF delta between backends should be sub-cm (m)";
    EXPECT_LT(rms_delta_m, 0.01) << "RMS ECEF delta between backends should be sub-cm (m)";

    EXPECT_NEAR(eigen_result.diagnostics.double_difference_pseudorange_residual_rms_m,
                gtsam_result.diagnostics.double_difference_pseudorange_residual_rms_m, 1e-3)
        << "eigen=" << eigen_result.diagnostics.double_difference_pseudorange_residual_rms_m
        << " gtsam=" << gtsam_result.diagnostics.double_difference_pseudorange_residual_rms_m;
    EXPECT_NEAR(eigen_result.diagnostics.double_difference_carrier_residual_rms_m,
                gtsam_result.diagnostics.double_difference_carrier_residual_rms_m, 1e-3)
        << "eigen=" << eigen_result.diagnostics.double_difference_carrier_residual_rms_m
        << " gtsam=" << gtsam_result.diagnostics.double_difference_carrier_residual_rms_m;
}

TEST(FGOGtsamBackendTest, GtsamBackendRecoversFloatAmbiguitiesNearIntegers) {
    const FGOProcessor::FGOProblem problem = makeGtsamParityDoubleDifferenceProblem();
    FGOProcessor::FGOConfig config = makeParityBaseConfig();
    config.backend = FGOBackend::GTSAM;

    FGOProcessor processor(config);
    const FGOProcessor::FGOResult result = processor.optimizeProblem(problem);

    ASSERT_EQ(result.ambiguity_estimates.size(), problem.ambiguity_states.size());
    for (std::size_t i = 0; i < result.ambiguity_estimates.size(); ++i) {
        const double cycles = result.ambiguity_estimates[i].ambiguity_cycles;
        const double nearest_integer = std::round(cycles);
        EXPECT_LT(std::abs(cycles - nearest_integer), 0.05)
            << "ambiguity index " << i << " float cycles=" << cycles;
    }
}

#endif  // GNSSPP_HAS_GTSAM
