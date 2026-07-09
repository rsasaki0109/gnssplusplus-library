#ifdef GNSSPP_HAS_GTSAM

#include <gtest/gtest.h>
#include <libgnss++/algorithms/fgo.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/io/imu.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <set>
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

// ============================================================================
// CP-hold / sanity FSM (FGOConfig::use_cp_hold_recovery) -- fixed-lag path.
//
// Builds a stationary, IMU-coupled, multi-epoch DD RTK problem (6 satellites,
// 1 reference + 5 targets) and corrupts specific epochs' DD CARRIER
// observation for satellite index 1 (ambiguity_index 0) by a large integer-
// cycle offset -- a "wrong basin" carrier lock the front-end never flags as a
// new arc (no cycle-slip marker), which the FSM must catch from post-fit DD
// PSEUDORANGE residuals alone. Corrupting carrier (very tight sigma) rather
// than pseudorange drags the GRAPH pose away from the true/IMU-predicted
// (stationary) position without the degenerate "uniform PR bias looks like a
// position shift" ambiguity a pseudorange-only corruption would have.
// ============================================================================
namespace {

struct CpHoldTestOptions {
    std::size_t num_epochs = 25;
    double epoch_dt_s = 1.0;
    // "Wrong basin" corruption: in the listed epochs, EVERY DD carrier
    // observation (reference AND all targets) is generated as if the rover
    // were actually at (true_position + carrier_corrupt_offset_ecef) instead
    // of true_position -- a self-consistent alternate hypothesis, exactly
    // like a real wrong-integer AR fix the front-end never flagged as a new
    // arc. Carrier sigma (0.02 m) is far tighter than pseudorange (0.5 m),
    // so this drags the GRAPH POSE to the wrong position; pseudorange is
    // ALWAYS generated from the true (stationary) position, so post-fit DD
    // pseudorange residuals at the dragged pose light up on every satellite
    // uniformly -- the intended "wrong basin", not single-satellite
    // multipath (see pr_corrupt_epochs below for that scenario).
    std::set<std::size_t> carrier_corrupt_epochs;
    Vector3d carrier_corrupt_offset_ecef = Vector3d(20.0, 0.0, 0.0);
    // Pseudorange-only corruption (carriers stay clean/consistent with
    // true_position, so the tight carrier constraints keep the graph pose
    // pinned at truth and any injected PR bias shows up almost entirely as
    // PER-SATELLITE residual) -- used to engineer a genuine "one dominant
    // multipath satellite among a noisy-but-small baseline" scenario for the
    // multipath-skip test.
    std::set<std::size_t> pr_corrupt_epochs;
    double pr_baseline_bias_m = 0.0;       ///< applied to every target satellite
    double pr_dominant_extra_bias_m = 0.0;  ///< additional bias, satellite index 1 only
};

FGOProcessor::FGOProblem makeCpHoldFixedLagProblem(const CpHoldTestOptions& opt) {
    FGOProcessor::FGOProblem problem;
    const auto satellites = gtsamParitySatelliteGeometry();
    const double wavelength = constants::GPS_L1_WAVELENGTH;

    const Vector3d true_position(1113194.0, -4841695.0, 3985350.0);
    const Vector3d base_position = true_position + Vector3d(-320.0, 180.0, 45.0);
    const double base_clock_bias_m = 11.0;
    const double rover_clock_bias_m = 42.0;

    double lat = 0.0, lon = 0.0, h = 0.0;
    ecef2geodetic(true_position, lat, lon, h);
    problem.imu.valid = true;
    problem.imu.nav_origin_ecef = true_position;
    problem.imu.nav_origin_lat_rad = lat;
    problem.imu.nav_origin_lon_rad = lon;
    problem.imu.init_attitude_body_to_nav = Matrix3d::Identity();
    problem.imu.init_velocity_nav = Vector3d::Zero();
    problem.imu.init_accel_bias = Vector3d::Zero();
    problem.imu.init_gyro_bias = Vector3d::Zero();

    // Ambiguity per non-reference satellite, held constant for the whole run
    // (no front-end arc break -- the corruption below is a mid-arc anomaly
    // only the backend FSM can react to).
    for (std::size_t sat = 1; sat < satellites.size(); ++sat) {
        FGOProcessor::AmbiguityState ambiguity;
        ambiguity.satellite = SatelliteId(GNSSSystem::GPS, static_cast<uint8_t>(sat + 1));
        ambiguity.reference_satellite = SatelliteId(GNSSSystem::GPS, 1);
        ambiguity.signal = SignalType::GPS_L1CA;
        ambiguity.wavelength_m = wavelength;
        ambiguity.is_double_difference = true;
        const int true_dd_cycles = 100 + static_cast<int>(sat);
        ambiguity.initial_ambiguity_m = static_cast<double>(true_dd_cycles) * wavelength;
        problem.ambiguity_states.push_back(ambiguity);
    }

    const GNSSTime t0(2300, 100000.0);
    for (std::size_t epoch = 0; epoch < opt.num_epochs; ++epoch) {
        FGOProcessor::EpochSeed seed;
        seed.time = t0 + static_cast<double>(epoch) * opt.epoch_dt_s;
        seed.position_ecef = true_position;  // stationary rover
        seed.receiver_clock_bias_m = rover_clock_bias_m;
        problem.epochs.push_back(seed);

        const bool corrupt = opt.carrier_corrupt_epochs.count(epoch) > 0;
        const bool pr_corrupt = opt.pr_corrupt_epochs.count(epoch) > 0;
        // Carriers are generated from carrier_pos (the wrong-basin hypothesis
        // when corrupt); pseudorange is ALWAYS generated from true_position.
        const Vector3d carrier_pos =
            corrupt ? true_position + opt.carrier_corrupt_offset_ecef : true_position;

        const double rover_ref_range_pr = (satellites[0] - true_position).norm();
        const double rover_ref_range_cp = (satellites[0] - carrier_pos).norm();
        const double base_ref_range = (satellites[0] - base_position).norm();
        FGOProcessor::ObservationModelDebug rover_ref_model;
        rover_ref_model.corrected_pseudorange_m = rover_ref_range_pr + rover_clock_bias_m;
        rover_ref_model.corrected_carrier_m = rover_ref_range_cp + rover_clock_bias_m;
        FGOProcessor::ObservationModelDebug base_ref_model;
        base_ref_model.corrected_pseudorange_m = base_ref_range + base_clock_bias_m;
        base_ref_model.corrected_carrier_m = base_ref_range + base_clock_bias_m;

        for (std::size_t sat = 1; sat < satellites.size(); ++sat) {
            const double rover_target_range_pr = (satellites[sat] - true_position).norm();
            const double rover_target_range_cp = (satellites[sat] - carrier_pos).norm();
            const double base_target_range = (satellites[sat] - base_position).norm();
            const int true_dd_cycles = 100 + static_cast<int>(sat);

            FGOProcessor::ObservationModelDebug rover_target_model;
            rover_target_model.corrected_pseudorange_m = rover_target_range_pr + rover_clock_bias_m;
            rover_target_model.corrected_carrier_m =
                rover_target_range_cp + rover_clock_bias_m +
                static_cast<double>(true_dd_cycles) * wavelength;
            if (pr_corrupt) {
                rover_target_model.corrected_pseudorange_m += opt.pr_baseline_bias_m;
                if (sat == 1) {
                    rover_target_model.corrected_pseudorange_m += opt.pr_dominant_extra_bias_m;
                }
            }

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
            // The backend's post-fit DDPR residual / quality-gate code reads
            // this field DIRECTLY (unlike the factor construction itself,
            // which reconstructs from the 4 raw model fields) -- it must be
            // populated for the CP-hold FSM / quality-gate tests to see
            // meaningful residuals.
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
                (rover_target_model.corrected_carrier_m - base_target_model.corrected_carrier_m) -
                (rover_ref_model.corrected_carrier_m - base_ref_model.corrected_carrier_m);
            cp_factor.sigma_m = 0.02;
            cp_factor.elevation_rad = 0.7;
            problem.double_difference_carrier_factors.push_back(cp_factor);
        }
    }

    // Synthetic stationary IMU at 10 Hz: identity attitude, zero velocity ->
    // accel = (0, 0, +g) in body, zero gyro.
    const double g = problem.imu.noise.gravity_mps2;
    const double total_s = static_cast<double>(opt.num_epochs) * opt.epoch_dt_s + 1.0;
    for (double t = 0.0; t <= total_s; t += 0.1) {
        ImuSample s;
        s.time = t0 + t;
        s.accel_raw = Vector3d(0.0, 0.0, g);
        s.gyro_raw_radps = Vector3d::Zero();
        problem.imu.samples_body_flu.push_back(s);
    }

    problem.diagnostics.input_epochs = opt.num_epochs;
    problem.diagnostics.seeded_epochs = opt.num_epochs;
    return problem;
}

FGOProcessor::FGOConfig makeCpHoldBaseConfig() {
    FGOProcessor::FGOConfig config;
    config.backend = FGOBackend::GTSAM;
    config.use_pose3_state = true;
    config.use_imu = true;
    config.use_fixed_lag_smoother = true;
    config.fixed_lag_smoother_lag_s = 6.0;
    config.use_double_difference_factors = true;
    config.use_carrier_phase_factors = false;
    config.use_pseudorange_factors = false;
    config.use_tdcp_factors = false;
    config.use_single_difference_doppler_factors = false;
    config.use_single_difference_tdcp_factors = false;
    config.use_ambiguity_priors = true;
    config.ambiguity_prior_sigma_m = 50.0;
    // No LAMBDA fixing in these tests: keeps nb == 0 always, so the
    // catastrophic fast path's "no fixed solution this epoch" gate is
    // trivially satisfied and unrelated to fix-and-hold behaviour.
    config.use_lambda_ambiguity_fix = false;
    config.use_ambiguity_hold = false;
    return config;
}

}  // namespace

TEST(FGOCpHoldFsmTest, DefaultOffIsNoOp) {
    CpHoldTestOptions opt;
    opt.carrier_corrupt_epochs = {5, 6, 7, 8, 9, 10, 11, 12};
    const auto problem = makeCpHoldFixedLagProblem(opt);

    FGOProcessor::FGOConfig config = makeCpHoldBaseConfig();
    ASSERT_FALSE(config.use_cp_hold_recovery);
    ASSERT_FALSE(config.use_solve_exception_recovery);

    FGOProcessor processor(config);
    const auto result = processor.optimizeProblem(problem);

    EXPECT_EQ(result.diagnostics.cp_hold_triggers, 0u);
    EXPECT_EQ(result.diagnostics.cp_hold_epochs_held, 0u);
    EXPECT_EQ(result.diagnostics.sanity_mass_resets, 0u);
    EXPECT_EQ(result.diagnostics.sanity_fast_resets, 0u);
    EXPECT_EQ(result.diagnostics.sanity_pose_replacements, 0u);
    EXPECT_EQ(result.diagnostics.ambiguity_generation_bumps, 0u);
    EXPECT_EQ(result.diagnostics.solve_exception_recoveries, 0u);
    EXPECT_EQ(result.diagnostics.solve_exception_warm_resets, 0u);
    EXPECT_EQ(result.solution.solutions.size(), problem.epochs.size());
}

TEST(FGOCpHoldFsmTest, HoldEngagesOnPersistAndSuppressesCarrier) {
    CpHoldTestOptions opt;
    // A long corrupt stretch: bad epochs 5..14 (10 consecutive), well past
    // the persist threshold (3), so the reset should fire around epoch 7 and
    // (with the fast path disabled below) stay held for a while afterward.
    for (std::size_t e = 5; e <= 14; ++e) opt.carrier_corrupt_epochs.insert(e);
    const auto problem = makeCpHoldFixedLagProblem(opt);

    FGOProcessor::FGOConfig config = makeCpHoldBaseConfig();
    config.use_cp_hold_recovery = true;
    config.cp_hold_main_residual_threshold_m = 3.0;
    config.cp_hold_persist_epochs = 3;
    config.cp_hold_catastrophic_threshold_m = 1.0e6;  // never take the fast path here
    config.cp_hold_epochs = 5;
    config.cp_hold_release_threshold_m = 2.0;
    config.cp_hold_release_count = 3;
    config.cp_hold_max_gdop = 0.0;  // no GDOP gate in this synthetic geometry

    FGOProcessor processor(config);
    const auto result = processor.optimizeProblem(problem);

    EXPECT_GT(result.diagnostics.cp_hold_triggers, 0u);
    EXPECT_GT(result.diagnostics.sanity_mass_resets, 0u)
        << "10 consecutive bad epochs should cross the persist threshold and reset";
    EXPECT_EQ(result.diagnostics.sanity_fast_resets, 0u)
        << "catastrophic threshold is disabled; only the persist path may fire";
    EXPECT_GT(result.diagnostics.cp_hold_epochs_held, 0u);
    EXPECT_GT(result.diagnostics.ambiguity_generation_bumps, 0u)
        << "the reset and/or held epochs should bump the corrupted arc's generation";
    for (const auto& sol : result.solution.solutions) {
        EXPECT_TRUE(sol.position_ecef.allFinite());
    }
}

TEST(FGOCpHoldFsmTest, ReleaseHysteresisExtendsHoldPastNominalLength) {
    CpHoldTestOptions opt;
    opt.num_epochs = 30;
    for (std::size_t e = 5; e <= 8; ++e) opt.carrier_corrupt_epochs.insert(e);  // 4 bad epochs
    const auto problem = makeCpHoldFixedLagProblem(opt);

    FGOProcessor::FGOConfig config = makeCpHoldBaseConfig();
    config.use_cp_hold_recovery = true;
    config.cp_hold_main_residual_threshold_m = 3.0;
    config.cp_hold_persist_epochs = 3;
    config.cp_hold_catastrophic_threshold_m = 1.0e6;
    // Nominal hold is short (3 epochs), but the release count (8) is longer
    // than that -- once carrier is suppressed the (now PR-only, clean)
    // residual should read as "clean" every held epoch, so the ONLY way to
    // reach release_count consecutive clean epochs is for the hold to keep
    // extending itself past cp_hold_epochs. This isolates the hysteresis
    // extension logic from the exact corruption magnitude.
    config.cp_hold_epochs = 3;
    config.cp_hold_release_threshold_m = 2.0;
    config.cp_hold_release_count = 8;
    config.cp_hold_max_gdop = 0.0;

    FGOProcessor processor(config);
    const auto result = processor.optimizeProblem(problem);

    EXPECT_GT(result.diagnostics.sanity_mass_resets, 0u);
    EXPECT_GE(result.diagnostics.cp_hold_epochs_held, 8u)
        << "hold must be extended by the release hysteresis past the nominal "
           "cp_hold_epochs=3 to reach the release_count=8 clean streak";
}

TEST(FGOCpHoldFsmTest, CatastrophicEpochTakesFastPath) {
    CpHoldTestOptions opt;
    opt.carrier_corrupt_epochs = {10};  // single isolated catastrophic epoch
    opt.carrier_corrupt_offset_ecef = Vector3d(150.0, 0.0, 0.0);  // unambiguously catastrophic
    const auto problem = makeCpHoldFixedLagProblem(opt);

    FGOProcessor::FGOConfig config = makeCpHoldBaseConfig();
    config.use_cp_hold_recovery = true;
    config.cp_hold_main_residual_threshold_m = 3.0;
    config.cp_hold_persist_epochs = 1000;  // never reachable within this run
    config.cp_hold_catastrophic_threshold_m = 5.0;
    config.cp_hold_fast_worst_satellite_min_m = 1.0;
    config.cp_hold_max_gdop = 0.0;

    FGOProcessor processor(config);
    const auto result = processor.optimizeProblem(problem);

    EXPECT_GT(result.diagnostics.sanity_fast_resets, 0u);
    EXPECT_EQ(result.diagnostics.sanity_mass_resets, 0u)
        << "persist_epochs is unreachable in this run; only the fast path may fire";
}

TEST(FGOCpHoldFsmTest, PoseReplacementOnlyAffectsReportedSolution) {
    // Pose replacement needs the IMU-predicted pose to be CLEAN at the reset
    // epoch: it is dead-reckoned from the PREVIOUS epoch's solved state, so
    // an isolated single-epoch corruption (like the fast-path scenario)
    // keeps pose_seed near truth right up to the bad epoch, while the graph
    // pose for that one epoch gets dragged to the wrong basin -- exactly the
    // "replace the reported position with the clean IMU prediction" case. A
    // run of several CONSECUTIVE corrupt epochs (as in the persist-path
    // tests above) would already have dragged the previous epoch's solved
    // state into the wrong basin by the time persist fires, so the IMU
    // prediction would be wrong too and the gap/pred_res gates correctly
    // decline to replace -- that is exercised implicitly by
    // HoldEngagesOnPersistAndSuppressesCarrier's absence of replacements.
    CpHoldTestOptions opt;
    opt.carrier_corrupt_epochs = {10};
    opt.carrier_corrupt_offset_ecef = Vector3d(150.0, 0.0, 0.0);
    const auto problem = makeCpHoldFixedLagProblem(opt);

    FGOProcessor::FGOConfig config = makeCpHoldBaseConfig();
    config.use_cp_hold_recovery = true;
    config.cp_hold_main_residual_threshold_m = 3.0;
    config.cp_hold_persist_epochs = 1000;  // isolate the fast path
    config.cp_hold_catastrophic_threshold_m = 5.0;
    config.cp_hold_fast_worst_satellite_min_m = 1.0;
    config.cp_hold_max_gdop = 0.0;
    config.cp_hold_pose_replace_threshold_m = 5.0;

    FGOProcessor processor(config);
    const auto result = processor.optimizeProblem(problem);

    ASSERT_GT(result.diagnostics.sanity_fast_resets, 0u);
    EXPECT_GT(result.diagnostics.sanity_pose_replacements, 0u)
        << "the corrupted (wrong-basin) graph pose should be far enough from "
           "the clean, stationary IMU-predicted pose to trigger a swap of the "
           "REPORTED position on the reset epoch";
    // The reset epoch(s) must be reported FLOAT (never FIXED via a wrong hold).
    for (const auto& sol : result.solution.solutions) {
        if (sol.status == SolutionStatus::FIXED) {
            // With use_ambiguity_hold/use_lambda_ambiguity_fix both off in
            // this config, no epoch should ever be labelled FIXED.
            FAIL() << "no epoch should be FIXED with fixing disabled";
        }
    }
}

TEST(FGOCpHoldFsmTest, MultipathDominatedEpochIsSkipped) {
    // A SINGLE dominant bad satellite among >=6 tracked satellites should be
    // read as multipath, not a wrong basin -- skip the reset entirely
    // (reference _ddpr_multipath_dominated). Corrupt PSEUDORANGE (not
    // carrier) here: with every satellite's carrier left clean, the tight
    // (0.02 m) carrier constraints pin the graph pose at truth regardless of
    // the loose (0.5 m) pseudorange, so the injected PR bias shows up almost
    // entirely as PER-SATELLITE residual rather than a shared position
    // error -- a small common baseline on every target plus one dominant
    // outlier on satellite index 1 gives the classic "one bad satellite"
    // shape (reference: max/median ratio far above threshold with >= 6
    // tracked satellites, this problem's exact floor: 5 targets + 1
    // reference).
    CpHoldTestOptions opt;
    opt.num_epochs = 30;
    for (std::size_t e = 5; e <= 19; ++e) opt.pr_corrupt_epochs.insert(e);
    opt.pr_baseline_bias_m = 0.3;         // small noise-like baseline, all targets
    opt.pr_dominant_extra_bias_m = 10.0;  // dominant outlier, satellite index 1 only
    const auto problem = makeCpHoldFixedLagProblem(opt);

    FGOProcessor::FGOConfig config = makeCpHoldBaseConfig();
    config.use_cp_hold_recovery = true;
    // NOTE: keep this at a realistic level (reference default 3.0), not an
    // artificially tiny threshold: the fixed-lag window takes several
    // epochs to fully forget the corruption after it ends (marginals decay,
    // not an instant drop to 0), and an unrealistically small threshold
    // turns that ordinary decay tail into a spurious multi-epoch "bad but
    // no longer single-satellite-shaped" streak that isn't the scenario
    // under test.
    config.cp_hold_main_residual_threshold_m = 3.0;
    config.cp_hold_persist_epochs = 3;
    config.cp_hold_catastrophic_threshold_m = 1.0e6;
    config.cp_hold_multipath_median_ratio = 1.5;  // easily cleared by one bad sat among 6
    config.cp_hold_multipath_min_satellites = 6;
    config.cp_hold_max_gdop = 0.0;

    FGOProcessor processor(config);
    const auto result = processor.optimizeProblem(problem);

    EXPECT_GT(result.diagnostics.sanity_multipath_skips, 0u);
    EXPECT_EQ(result.diagnostics.sanity_mass_resets, 0u)
        << "the multipath skip must prevent the persist path from ever resetting";
}

// ============================================================================
// DDPR-LS anchor (FGOConfig::use_ddpr_anchor) -- the anchor stages of the
// reference's postfit.py/recovery.py/optimize-stage.py that the CP-hold FSM
// port above deliberately skipped. Reuses makeCpHoldFixedLagProblem/
// makeCpHoldBaseConfig from the CP-hold FSM tests above.
// ============================================================================

TEST(FGODdprAnchorTest, DefaultOffIsNoOpEvenWithFsmOn) {
    CpHoldTestOptions opt;
    for (std::size_t e = 5; e <= 14; ++e) opt.carrier_corrupt_epochs.insert(e);
    const auto problem = makeCpHoldFixedLagProblem(opt);

    FGOProcessor::FGOConfig config = makeCpHoldBaseConfig();
    config.use_cp_hold_recovery = true;
    config.cp_hold_persist_epochs = 3;
    config.cp_hold_catastrophic_threshold_m = 1.0e6;
    config.cp_hold_max_gdop = 0.0;
    config.use_solve_exception_recovery = true;
    ASSERT_FALSE(config.use_ddpr_anchor);

    FGOProcessor processor(config);
    const auto result = processor.optimizeProblem(problem);

    EXPECT_GT(result.diagnostics.sanity_mass_resets, 0u)
        << "sanity check: the FSM itself must still fire in this scenario";
    EXPECT_EQ(result.diagnostics.ddpr_anchor_solves, 0u);
    EXPECT_EQ(result.diagnostics.ddpr_anchor_successes, 0u);
    EXPECT_EQ(result.diagnostics.ddpr_anchor_gated_resets_skipped, 0u);
    EXPECT_EQ(result.diagnostics.ddpr_anchor_gated_resets_allowed, 0u);
    EXPECT_EQ(result.diagnostics.ddpr_anchored_warm_resets, 0u);
    EXPECT_EQ(result.diagnostics.ddpr_anchor_bootstrap_prior_epochs, 0u);
}

TEST(FGODdprAnchorTest, OffMatchesFsmOnlyBaselineBitIdentical) {
    // use_ddpr_anchor=false must be a true no-op: setting it (and every new
    // anchor knob) to deliberately weird non-default values must not change
    // a single bit of the FSM-only result, since every new code path is
    // gated behind config.use_ddpr_anchor.
    CpHoldTestOptions opt;
    for (std::size_t e = 5; e <= 14; ++e) opt.carrier_corrupt_epochs.insert(e);
    const auto problem = makeCpHoldFixedLagProblem(opt);

    FGOProcessor::FGOConfig base_config = makeCpHoldBaseConfig();
    base_config.use_cp_hold_recovery = true;
    base_config.cp_hold_persist_epochs = 3;
    base_config.cp_hold_catastrophic_threshold_m = 1.0e6;
    base_config.cp_hold_max_gdop = 0.0;
    base_config.use_solve_exception_recovery = true;

    FGOProcessor::FGOConfig anchor_off_config = base_config;
    anchor_off_config.use_ddpr_anchor = false;  // explicit, still default
    anchor_off_config.ddpr_anchor_max_residual_m = 999.0;
    anchor_off_config.ddpr_anchor_fde_threshold_m = 0.001;
    anchor_off_config.ddpr_anchor_min_factors = 1;
    anchor_off_config.ddpr_anchor_bootstrap_epochs = 1000;
    anchor_off_config.ddpr_anchor_bootstrap_sigma_m = 1e-6;
    anchor_off_config.cp_hold_bootstrap_after_mass_reset = false;

    FGOProcessor base_processor(base_config);
    const auto base_result = base_processor.optimizeProblem(problem);
    FGOProcessor anchor_off_processor(anchor_off_config);
    const auto anchor_off_result = anchor_off_processor.optimizeProblem(problem);

    ASSERT_EQ(base_result.solution.solutions.size(), anchor_off_result.solution.solutions.size());
    for (std::size_t i = 0; i < base_result.solution.solutions.size(); ++i) {
        const auto& a = base_result.solution.solutions[i];
        const auto& b = anchor_off_result.solution.solutions[i];
        EXPECT_EQ(a.status, b.status) << "epoch " << i;
        EXPECT_TRUE(a.position_ecef.isApprox(b.position_ecef, 0.0) || a.position_ecef == b.position_ecef)
            << "epoch " << i << " position diverged with use_ddpr_anchor=false";
    }
    EXPECT_EQ(base_result.diagnostics.sanity_mass_resets, anchor_off_result.diagnostics.sanity_mass_resets);
    EXPECT_EQ(base_result.diagnostics.cp_hold_triggers, anchor_off_result.diagnostics.cp_hold_triggers);
    EXPECT_EQ(base_result.diagnostics.ambiguity_generation_bumps,
              anchor_off_result.diagnostics.ambiguity_generation_bumps);
    EXPECT_EQ(anchor_off_result.diagnostics.ddpr_anchor_solves, 0u);
}

TEST(FGODdprAnchorTest, BootstrapRecoversPositionAndRejectsFdeOutlier) {
    // Epochs 5-7: carrier-corrupted -> persist-path mass reset fires (~epoch
    // 7) and (with cp_hold_bootstrap_after_mass_reset explicitly enabled
    // below -- shipped default is false, see fgo.hpp) arms the bootstrap
    // countdown. Epochs 10-12 (inside the bootstrap window, well
    // after carrier corruption has ended): a small common PR bias (0.3 m,
    // noise-like) on every target plus a dominant 25 m outlier on satellite
    // index 1 -- the mini DDPR-LS anchor's FDE (threshold 4.0 m default)
    // must drop that one satellite (5 targets -> 4 remain, exactly the
    // min-factors floor). We prove FDE worked by comparing this run against
    // an otherwise-identical CLEAN run (no PR outlier): if FDE is doing its
    // job, the outlier run keeps the same res-gated success count and its
    // bootstrap-anchored position stays near truth.
    auto run = [](bool inject_outlier) {
        CpHoldTestOptions opt;
        opt.num_epochs = 40;
        for (std::size_t e = 5; e <= 7; ++e) opt.carrier_corrupt_epochs.insert(e);
        opt.carrier_corrupt_offset_ecef = Vector3d(30.0, 0.0, 0.0);
        if (inject_outlier) {
            for (std::size_t e = 10; e <= 12; ++e) opt.pr_corrupt_epochs.insert(e);
            opt.pr_baseline_bias_m = 0.3;
            // Large enough that the 5-factor LS fit cannot absorb it below
            // the 4.0 m FDE threshold by shifting position (an ~8 m outlier
            // among only 5 DD factors CAN be mostly absorbed -- measured).
            opt.pr_dominant_extra_bias_m = 25.0;
        }
        const auto problem = makeCpHoldFixedLagProblem(opt);

        FGOProcessor::FGOConfig config = makeCpHoldBaseConfig();
        config.use_cp_hold_recovery = true;
        config.cp_hold_persist_epochs = 3;
        config.cp_hold_catastrophic_threshold_m = 1.0e6;
        config.cp_hold_max_gdop = 0.0;
        config.cp_hold_epochs = 3;
        config.use_ddpr_anchor = true;
        config.ddpr_anchor_bootstrap_epochs = 15;
        config.ddpr_anchor_bootstrap_sigma_m = 0.5;
        config.cp_hold_bootstrap_after_mass_reset = true;

        FGOProcessor processor(config);
        return processor.optimizeProblem(problem);
    };

    const auto clean_result = run(/*inject_outlier=*/false);
    const auto outlier_result = run(/*inject_outlier=*/true);

    ASSERT_GT(outlier_result.diagnostics.sanity_mass_resets, 0u);
    EXPECT_GT(outlier_result.diagnostics.ddpr_anchor_solves, 0u);
    EXPECT_GT(outlier_result.diagnostics.ddpr_anchor_bootstrap_prior_epochs, 0u)
        << "bootstrap must be armed after the mass reset and add anchor priors";

    // FDE proof: ddpr_anchor_successes counts RES-GATED (res_rms <=
    // ddpr_anchor_max_residual_m = 2.0 m) trusted solves. On the outlier
    // epochs the anchor sees a dominant 8 m residual on satellite 1; only
    // if FDE drops that factor can the post-FDE res_rms come back under
    // 2.0 m and the epoch still count as a success. So a working FDE makes
    // the outlier run's success count track the clean run's; a broken FDE
    // loses (at least) the 3 outlier epochs.
    EXPECT_GT(outlier_result.diagnostics.ddpr_anchor_successes, 0u);
    EXPECT_GE(outlier_result.diagnostics.ddpr_anchor_successes + 1,
              clean_result.diagnostics.ddpr_anchor_successes)
        << "FDE should keep the 3 outlier epochs trusted (res-gated) -- a broken FDE "
           "would lose them (clean=" << clean_result.diagnostics.ddpr_anchor_successes
        << " outlier=" << outlier_result.diagnostics.ddpr_anchor_successes << ")";
    EXPECT_GE(outlier_result.diagnostics.ddpr_anchor_bootstrap_prior_epochs,
              clean_result.diagnostics.ddpr_anchor_bootstrap_prior_epochs - 1);

    // Position recovery: with noise-free synthetic pseudorange the DDPR-LS
    // anchor is exact, so the bootstrap-anchored post-reset float must pull
    // back to truth within the bootstrap window (the reset happened ~epoch
    // 7; by epoch 10 the anchor priors dominate). The outlier run must land
    // in the same place -- FDE'd anchors are computed from the 4 clean
    // factors only. (The main graph's own PR factors for the corrupted
    // epochs are NOT FDE'd -- only the standalone anchor solve is -- so
    // allow a wider tolerance there.)
    const Vector3d true_position(1113194.0, -4841695.0, 3985350.0);
    ASSERT_EQ(clean_result.solution.solutions.size(), outlier_result.solution.solutions.size());
    for (std::size_t e = 10; e <= 14; ++e) {
        const double clean_err =
            (clean_result.solution.solutions[e].position_ecef - true_position).norm();
        EXPECT_LT(clean_err, 0.5)
            << "epoch " << e << ": clean bootstrap-anchored float should be pinned at truth "
               "(err=" << clean_err << " m)";
        const double outlier_err =
            (outlier_result.solution.solutions[e].position_ecef - true_position).norm();
        EXPECT_LT(outlier_err, 3.0)
            << "epoch " << e << ": FDE'd anchor should keep the outlier run near truth "
               "(err=" << outlier_err << " m)";
    }
}

TEST(FGODdprAnchorTest, GatingDiagnosticsAllowsWhenAnchorAgreesWithImu) {
    // A single ISOLATED corrupt epoch (persist_epochs=1 so it fires
    // immediately on that one bad epoch, mirroring the existing
    // PoseReplacementOnlyAffectsReportedSolution / CatastrophicEpochTakesFastPath
    // fast-path fixtures' rationale): the IMU-predicted pose_seed is dead-
    // reckoned from the PREVIOUS (clean) epoch, so it stays at truth, while
    // the anchor -- solved from this epoch's uncorrupted DD PSEUDORANGE
    // alone -- also converges to truth. Anchor-vs-IMU gap should be ~0, well
    // under ddpr_anchor_imu_max_gap_m (20 m default): the gate agrees.
    CpHoldTestOptions opt;
    opt.carrier_corrupt_epochs = {10};
    opt.carrier_corrupt_offset_ecef = Vector3d(25.0, 0.0, 0.0);
    const auto problem = makeCpHoldFixedLagProblem(opt);

    FGOProcessor::FGOConfig config = makeCpHoldBaseConfig();
    config.use_cp_hold_recovery = true;
    config.cp_hold_persist_epochs = 1;  // fire on the very first (isolated) bad epoch
    config.cp_hold_catastrophic_threshold_m = 1.0e6;  // disable the fast path
    config.cp_hold_max_gdop = 0.0;
    config.use_ddpr_anchor = true;
    config.cp_hold_bootstrap_after_mass_reset = false;  // isolate the gating diagnostics

    FGOProcessor processor(config);
    const auto result = processor.optimizeProblem(problem);

    ASSERT_GT(result.diagnostics.sanity_mass_resets, 0u);
    EXPECT_GT(result.diagnostics.ddpr_anchor_gated_resets_allowed, 0u);
    EXPECT_EQ(result.diagnostics.ddpr_anchor_gated_resets_skipped, 0u);
}

TEST(FGODdprAnchorTest, GatingDiagnosticsSkipsWhenAnchorDisagreesWithImu) {
    // The multi-epoch persist scenario (carrier-corrupted for several
    // CONSECUTIVE epochs before the reset fires): by the time persist
    // triggers, the IMU-predicted pose_seed has ITSELF been dragged into the
    // wrong basin by the accumulated corrupt carrier evidence over several
    // epochs, while the anchor (recomputed fresh from this epoch's clean DD
    // pseudorange) still lands near truth -- their gap comfortably exceeds
    // ddpr_anchor_imu_max_gap_m (20 m default): the gate would reject.
    CpHoldTestOptions opt;
    for (std::size_t e = 5; e <= 9; ++e) opt.carrier_corrupt_epochs.insert(e);
    opt.carrier_corrupt_offset_ecef = Vector3d(20.0, 0.0, 0.0);
    const auto problem = makeCpHoldFixedLagProblem(opt);

    FGOProcessor::FGOConfig config = makeCpHoldBaseConfig();
    config.use_cp_hold_recovery = true;
    config.cp_hold_persist_epochs = 3;
    config.cp_hold_catastrophic_threshold_m = 1.0e6;  // disable the catastrophic override
    config.cp_hold_max_gdop = 0.0;
    config.use_ddpr_anchor = true;
    config.cp_hold_bootstrap_after_mass_reset = false;  // isolate the gating diagnostics

    FGOProcessor processor(config);
    const auto result = processor.optimizeProblem(problem);

    ASSERT_GT(result.diagnostics.sanity_mass_resets, 0u);
    EXPECT_GT(result.diagnostics.ddpr_anchor_gated_resets_skipped, 0u);
}

// ============================================================================
// FDE: GICI-style Fault Detection and Exclusion (FGOConfig::use_fde) -- port
// of the reference's validation/postfit.py apply_fde. Uses two dedicated
// fixtures: a PR-only fixed-lag problem (no carrier/ambiguity at all, so an
// injected pseudorange outlier's effect on the FLOAT POSITION is not masked
// by a tight, clean carrier pin) for the pseudorange-side tests, and the
// existing makeCpHoldFixedLagProblem (which does carry carrier/ambiguities)
// for the carrier-side test.
// ============================================================================
namespace {

// Stationary, IMU-coupled, DD-PSEUDORANGE-ONLY fixed-lag problem (no carrier
// factors, no ambiguities): the float position is driven purely by DD PR
// residuals, so an injected outlier's effect on the reported position is
// directly attributable to FDE (not masked by a tight carrier constraint).
// Always clean; tests mutate specific (epoch, satellite) factors afterward.
FGOProcessor::FGOProblem makeFdeDdPrOnlyProblem(std::size_t num_epochs) {
    FGOProcessor::FGOProblem problem;
    const auto satellites = gtsamParitySatelliteGeometry();

    const Vector3d true_position(1113194.0, -4841695.0, 3985350.0);
    const Vector3d base_position = true_position + Vector3d(-320.0, 180.0, 45.0);
    const double base_clock_bias_m = 11.0;
    const double rover_clock_bias_m = 42.0;

    double lat = 0.0, lon = 0.0, h = 0.0;
    ecef2geodetic(true_position, lat, lon, h);
    problem.imu.valid = true;
    problem.imu.nav_origin_ecef = true_position;
    problem.imu.nav_origin_lat_rad = lat;
    problem.imu.nav_origin_lon_rad = lon;
    problem.imu.init_attitude_body_to_nav = Matrix3d::Identity();
    problem.imu.init_velocity_nav = Vector3d::Zero();
    problem.imu.init_accel_bias = Vector3d::Zero();
    problem.imu.init_gyro_bias = Vector3d::Zero();

    const GNSSTime t0(2300, 100000.0);
    for (std::size_t epoch = 0; epoch < num_epochs; ++epoch) {
        FGOProcessor::EpochSeed seed;
        seed.time = t0 + static_cast<double>(epoch);
        seed.position_ecef = true_position;  // stationary rover
        seed.receiver_clock_bias_m = rover_clock_bias_m;
        problem.epochs.push_back(seed);

        const double rover_ref_range = (satellites[0] - true_position).norm();
        const double base_ref_range = (satellites[0] - base_position).norm();
        FGOProcessor::ObservationModelDebug rover_ref_model;
        rover_ref_model.corrected_pseudorange_m = rover_ref_range + rover_clock_bias_m;
        FGOProcessor::ObservationModelDebug base_ref_model;
        base_ref_model.corrected_pseudorange_m = base_ref_range + base_clock_bias_m;

        for (std::size_t sat = 1; sat < satellites.size(); ++sat) {
            const double rover_target_range = (satellites[sat] - true_position).norm();
            const double base_target_range = (satellites[sat] - base_position).norm();

            FGOProcessor::ObservationModelDebug rover_target_model;
            rover_target_model.corrected_pseudorange_m = rover_target_range + rover_clock_bias_m;
            FGOProcessor::ObservationModelDebug base_target_model;
            base_target_model.corrected_pseudorange_m = base_target_range + base_clock_bias_m;

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
                (rover_ref_model.corrected_pseudorange_m - base_ref_model.corrected_pseudorange_m);
            pr_factor.sigma_m = 0.5;
            pr_factor.elevation_rad = 0.7;
            problem.double_difference_pseudorange_factors.push_back(pr_factor);
        }
    }

    const double g = problem.imu.noise.gravity_mps2;
    const double total_s = static_cast<double>(num_epochs) + 1.0;
    for (double t = 0.0; t <= total_s; t += 0.1) {
        ImuSample s;
        s.time = t0 + t;
        s.accel_raw = Vector3d(0.0, 0.0, g);
        s.gyro_raw_radps = Vector3d::Zero();
        problem.imu.samples_body_flu.push_back(s);
    }

    problem.diagnostics.input_epochs = num_epochs;
    problem.diagnostics.seeded_epochs = num_epochs;
    return problem;
}

FGOProcessor::FGOConfig makeFdeBaseConfig() {
    FGOProcessor::FGOConfig config;
    config.backend = FGOBackend::GTSAM;
    config.use_pose3_state = true;
    config.use_imu = true;
    config.use_fixed_lag_smoother = true;
    config.fixed_lag_smoother_lag_s = 6.0;
    config.use_double_difference_factors = true;
    config.use_carrier_phase_factors = false;
    config.use_pseudorange_factors = false;
    config.use_tdcp_factors = false;
    config.use_single_difference_doppler_factors = false;
    config.use_single_difference_tdcp_factors = false;
    config.use_lambda_ambiguity_fix = false;
    config.use_ambiguity_hold = false;
    // Isolate FDE's own effect from Huber down-weighting, which would
    // otherwise already suppress a large outlier's influence before FDE
    // ever gets a chance to remove it outright.
    config.use_robust_loss = false;
    return config;
}

// Injects a raw pseudorange bias into ONE (epoch, satellite)'s DD PR factor,
// mutating the same raw model field (rover_satellite_model.corrected_
// pseudorange_m) the GTSAM factor is actually built from AND observed_dd_
// pseudorange_m (which the post-fit diagnostics/quality-gate code reads
// directly), mirroring how makeCpHoldFixedLagProblem's own pr_corrupt
// mechanism keeps both in sync.
void injectPseudorangeOutlier(FGOProcessor::FGOProblem& problem, std::size_t epoch_index,
                              uint8_t satellite_prn, double bias_m) {
    for (auto& pr : problem.double_difference_pseudorange_factors) {
        if (pr.epoch_index != epoch_index || pr.satellite.prn != satellite_prn) continue;
        pr.rover_satellite_model.corrected_pseudorange_m += bias_m;
        pr.observed_dd_pseudorange_m += bias_m;
    }
}

}  // namespace

TEST(FGOFdeTest, DefaultOffIsNoOp) {
    auto problem = makeFdeDdPrOnlyProblem(20);
    injectPseudorangeOutlier(problem, /*epoch=*/10, /*prn=*/2, /*bias_m=*/20.0);

    FGOProcessor::FGOConfig config = makeFdeBaseConfig();
    ASSERT_FALSE(config.use_fde);

    FGOProcessor processor(config);
    const auto result = processor.optimizeProblem(problem);

    EXPECT_EQ(result.diagnostics.fde_pseudorange_rejections, 0u);
    EXPECT_EQ(result.diagnostics.fde_carrier_rejections, 0u);
    EXPECT_EQ(result.diagnostics.fde_safeguard_skips, 0u);
    EXPECT_EQ(result.diagnostics.fde_epochs, 0u);
    EXPECT_EQ(result.solution.solutions.size(), problem.epochs.size());
}

TEST(FGOFdeTest, PseudorangeOutlierIsRemovedAndFloatRecovers) {
    constexpr std::size_t kOutlierEpoch = 10;
    constexpr double kOutlierBiasM = 20.0;  // well above fde_pseudorange_threshold_m default (4.0)
    const Vector3d true_position(1113194.0, -4841695.0, 3985350.0);

    auto problem = makeFdeDdPrOnlyProblem(20);
    injectPseudorangeOutlier(problem, kOutlierEpoch, /*prn=*/2, kOutlierBiasM);

    FGOProcessor::FGOConfig off_config = makeFdeBaseConfig();
    ASSERT_FALSE(off_config.use_fde);
    FGOProcessor off_processor(off_config);
    const auto off_result = off_processor.optimizeProblem(problem);

    FGOProcessor::FGOConfig on_config = makeFdeBaseConfig();
    on_config.use_fde = true;
    FGOProcessor on_processor(on_config);
    const auto on_result = on_processor.optimizeProblem(problem);

    ASSERT_GT(on_result.diagnostics.fde_pseudorange_rejections, 0u);
    ASSERT_GT(on_result.diagnostics.fde_epochs, 0u);
    EXPECT_EQ(on_result.diagnostics.fde_carrier_rejections, 0u);
    EXPECT_EQ(on_result.diagnostics.fde_safeguard_skips, 0u);

    ASSERT_EQ(off_result.solution.solutions.size(), on_result.solution.solutions.size());
    const double off_err =
        (off_result.solution.solutions[kOutlierEpoch].position_ecef - true_position).norm();
    const double on_err =
        (on_result.solution.solutions[kOutlierEpoch].position_ecef - true_position).norm();
    EXPECT_GT(off_err, 1.0) << "without FDE the outlier should visibly bias the float (err="
                            << off_err << " m)";
    EXPECT_LT(on_err, 0.5) << "with FDE the outlier factor should be excluded and the float "
                              "recover near truth (err=" << on_err << " m)";
}

TEST(FGOFdeTest, CarrierOutlierReleasesHoldAndBumpsGeneration) {
    // Fix-and-hold needs several clean epochs to validate and pin an arc
    // before the outlier can test its release.
    CpHoldTestOptions opt;
    opt.num_epochs = 30;
    auto problem = makeCpHoldFixedLagProblem(opt);

    // Single-satellite, single-epoch carrier-only outlier (ambiguity_index 0,
    // i.e. satellite PRN 2): mutate the same raw model field the GTSAM
    // carrier factor is built from, well past where fix-and-hold should have
    // pinned this arc (noise-free synthetic geometry converges in a handful
    // of epochs).
    constexpr std::size_t kOutlierEpoch = 15;
    constexpr double kOutlierBiasM = 5.0;  // >> fde_carrier_threshold_m (0.5)
    for (auto& cp : problem.double_difference_carrier_factors) {
        if (cp.epoch_index != kOutlierEpoch || cp.ambiguity_index != 0) continue;
        cp.rover_satellite_model.corrected_carrier_m += kOutlierBiasM;
        cp.observed_dd_carrier_m += kOutlierBiasM;
    }

    FGOProcessor::FGOConfig config = makeCpHoldBaseConfig();
    config.use_lambda_ambiguity_fix = true;
    config.use_ambiguity_hold = true;
    config.lambda_ratio_threshold = 1.5;      // easily cleared by this noise-free synthetic data
    config.ambiguity_hold_ratio_threshold = 1.5;
    config.ambiguity_hold_min_fixed = 4;
    config.min_fixed_ambiguities = 5;  // all 5 ambiguities

    FGOProcessor::FGOConfig off_config = config;
    ASSERT_FALSE(off_config.use_fde);
    FGOProcessor off_processor(off_config);
    const auto off_result = off_processor.optimizeProblem(problem);

    FGOProcessor::FGOConfig on_config = config;
    on_config.use_fde = true;
    FGOProcessor on_processor(on_config);
    const auto on_result = on_processor.optimizeProblem(problem);

    // With no other reset mechanism enabled (cp-hold FSM / solve-exception
    // recovery both off), the generation must never bump without FDE.
    EXPECT_EQ(off_result.diagnostics.ambiguity_generation_bumps, 0u);

    ASSERT_GT(on_result.diagnostics.fde_carrier_rejections, 0u);
    EXPECT_EQ(on_result.diagnostics.fde_pseudorange_rejections, 0u);
    EXPECT_GT(on_result.diagnostics.ambiguity_generation_bumps, 0u)
        << "the rejected carrier factor's arc should get a fresh generation, "
           "i.e. be treated as a cycle slip";
    for (const auto& sol : on_result.solution.solutions) {
        EXPECT_TRUE(sol.position_ecef.allFinite());
    }
}

TEST(FGOFdeTest, SafeguardSkipsWhenMajorityOfEpochRejected) {
    // 5 DD PR factors this epoch (ref PRN1 + 5 targets PRN2..6); corrupt 4 of
    // the 5 (PRNs 2-5) by a huge bias so the reject fraction (4/5 = 0.8)
    // exceeds the default fde_max_rejected_fraction (0.5) safeguard, leaving
    // only PRN6 clean. The corruption is placed on the LAST epoch of a short
    // run: since the safeguard leaves epoch kBadEpoch's factors un-cleaned
    // (that is the point of this test), the resulting wrong position estimate
    // would otherwise cascade into later epochs via the fixed-lag window and
    // create genuine (if unwanted) residual spikes there too -- placing it
    // last means there is no "later epoch" for that cascade to reach.
    constexpr std::size_t kBadEpoch = 9;
    auto problem = makeFdeDdPrOnlyProblem(kBadEpoch + 1);
    for (uint8_t prn = 2; prn <= 5; ++prn) {
        injectPseudorangeOutlier(problem, kBadEpoch, prn, 50.0);
    }

    // use_cp_hold_recovery OFF: the safeguard has no hold to engage, it must
    // simply skip FDE for the epoch with no other side effect.
    {
        FGOProcessor::FGOConfig config = makeFdeBaseConfig();
        config.use_fde = true;
        ASSERT_FALSE(config.use_cp_hold_recovery);
        FGOProcessor processor(config);
        const auto result = processor.optimizeProblem(problem);

        EXPECT_GT(result.diagnostics.fde_safeguard_skips, 0u);
        EXPECT_EQ(result.diagnostics.fde_pseudorange_rejections, 0u)
            << "the safeguard must abandon FDE entirely for the epoch -- no partial removal";
        EXPECT_EQ(result.diagnostics.cp_hold_triggers, 0u)
            << "with the FSM off there is no hold to engage";
    }
    // use_cp_hold_recovery ON: the safeguard should engage CP-hold (reference
    // trigger_cp_hold(..., skip_if_active=True)).
    {
        FGOProcessor::FGOConfig config = makeFdeBaseConfig();
        config.use_fde = true;
        config.use_cp_hold_recovery = true;
        config.cp_hold_epochs = 5;
        FGOProcessor processor(config);
        const auto result = processor.optimizeProblem(problem);

        EXPECT_GT(result.diagnostics.fde_safeguard_skips, 0u);
        EXPECT_EQ(result.diagnostics.fde_pseudorange_rejections, 0u);
        EXPECT_GT(result.diagnostics.cp_hold_triggers, 0u)
            << "the safeguard should engage CP-hold when the FSM is enabled";
    }
}

TEST(FGOFdeTest, SanityTriggerSeesPreFdeResidualNotPostFde) {
    // A single injected outlier that FDE fully cleans up this SAME epoch
    // must still have been visible to the CP-hold/sanity FSM's residual
    // input (reference: stage.py's main_ddpr_residuals -- which feeds the
    // FSM trigger -- runs BEFORE apply_fde). If the FSM instead read the
    // POST-FDE (cleaned) residual, it would never trigger here.
    constexpr std::size_t kOutlierEpoch = 10;
    constexpr double kOutlierBiasM = 20.0;
    auto problem = makeFdeDdPrOnlyProblem(20);
    injectPseudorangeOutlier(problem, kOutlierEpoch, /*prn=*/2, kOutlierBiasM);

    FGOProcessor::FGOConfig config = makeFdeBaseConfig();
    config.use_fde = true;
    config.use_cp_hold_recovery = true;
    config.cp_hold_persist_epochs = 1;  // fire on the very first bad epoch
    config.cp_hold_catastrophic_threshold_m = 1.0e6;  // isolate the persist path
    config.cp_hold_main_residual_threshold_m = 3.0;
    config.cp_hold_max_gdop = 0.0;

    FGOProcessor processor(config);
    const auto result = processor.optimizeProblem(problem);

    ASSERT_GT(result.diagnostics.fde_pseudorange_rejections, 0u)
        << "sanity check: FDE must actually clean this epoch";
    EXPECT_GT(result.diagnostics.cp_hold_triggers, 0u)
        << "the sanity FSM must still react to the PRE-FDE (dirty) residual even though "
           "FDE cleaned the SAME epoch";
}

// ============================================================================
// Sat-badness EWMA down-weighting (FGOConfig::use_sat_badness_downweight) --
// port of the inuex35 reference's preprocess/sat_quality.py SatQualityState.
// Reuses makeCpHoldFixedLagProblem/makeCpHoldBaseConfig (defined above): its
// pr_corrupt_epochs/pr_dominant_extra_bias_m knobs inject a per-satellite DD
// PSEUDORANGE bias (satellite index 1, i.e. PRN 2) while leaving carrier
// clean, which is exactly the "chronically bad satellite" pattern the
// backend's post-fit per-sat DDPR residual (per_sat_res) is meant to detect
// -- without corrupting the carrier data that the CP-sigma-inflation test
// needs to stay trustworthy.
// ============================================================================
namespace {

// Removes every DD PR/CP factor for one (epoch, satellite PRN) pair,
// simulating that satellite being untracked/absent for that single epoch --
// used to exercise the reference's "sats not seen this epoch hard-reset
// their EWMA/streak to 0" semantics.
void dropSatelliteAtEpoch(FGOProcessor::FGOProblem& problem, std::size_t epoch_index,
                          uint8_t satellite_prn) {
    auto& prs = problem.double_difference_pseudorange_factors;
    prs.erase(std::remove_if(prs.begin(), prs.end(),
                             [&](const auto& f) {
                                 return f.epoch_index == epoch_index &&
                                        f.satellite.prn == satellite_prn;
                             }),
             prs.end());
    auto& cps = problem.double_difference_carrier_factors;
    cps.erase(std::remove_if(cps.begin(), cps.end(),
                             [&](const auto& f) {
                                 return f.epoch_index == epoch_index &&
                                        f.satellite.prn == satellite_prn;
                             }),
             cps.end());
}

}  // namespace

TEST(FGOSatBadnessTest, DefaultOffIsNoOp) {
    CpHoldTestOptions opt;
    opt.num_epochs = 25;
    opt.pr_corrupt_epochs = {10, 11, 12, 13, 14, 15, 16, 17, 18, 19};
    opt.pr_dominant_extra_bias_m = 3.0;
    auto problem = makeCpHoldFixedLagProblem(opt);

    FGOProcessor::FGOConfig config = makeCpHoldBaseConfig();
    ASSERT_FALSE(config.use_sat_badness_downweight);

    FGOProcessor processor(config);
    const auto result = processor.optimizeProblem(problem);

    EXPECT_EQ(result.diagnostics.sat_badness_downweighted_factors, 0u);
    EXPECT_EQ(result.diagnostics.sat_badness_max_score_seen, 0.0);
    EXPECT_EQ(result.solution.solutions.size(), problem.epochs.size());
    for (const auto& sol : result.solution.solutions) {
        EXPECT_TRUE(sol.position_ecef.allFinite());
    }
}

TEST(FGOSatBadnessTest, ChronicallyBadSatelliteScoresHigherThanCleanRun) {
    CpHoldTestOptions bad_opt;
    bad_opt.num_epochs = 25;
    bad_opt.pr_corrupt_epochs = {10, 11, 12, 13, 14, 15, 16, 17, 18, 19};
    bad_opt.pr_dominant_extra_bias_m = 3.0;
    auto bad_problem = makeCpHoldFixedLagProblem(bad_opt);

    CpHoldTestOptions clean_opt;
    clean_opt.num_epochs = 25;
    auto clean_problem = makeCpHoldFixedLagProblem(clean_opt);

    FGOProcessor::FGOConfig config = makeCpHoldBaseConfig();
    config.use_sat_badness_downweight = true;

    FGOProcessor bad_processor(config);
    const auto bad_result = bad_processor.optimizeProblem(bad_problem);
    FGOProcessor clean_processor(config);
    const auto clean_result = clean_processor.optimizeProblem(clean_problem);

    EXPECT_GT(bad_result.diagnostics.sat_badness_downweighted_factors, 0u)
        << "the chronically-bad satellite's DD pairs must get inflated";
    EXPECT_GT(bad_result.diagnostics.sat_badness_max_score_seen,
              clean_result.diagnostics.sat_badness_max_score_seen)
        << "a chronically bad satellite's score must clearly exceed a clean run's "
           "(bad=" << bad_result.diagnostics.sat_badness_max_score_seen
        << ", clean=" << clean_result.diagnostics.sat_badness_max_score_seen << ")";
    for (const auto& sol : bad_result.solution.solutions) {
        EXPECT_TRUE(sol.position_ecef.allFinite());
    }
}

TEST(FGOSatBadnessTest, SigmaInflationAffectsCarrierNotPseudorangeAtDefaults) {
    // Carrier stays clean/true in this fixture; only the DD pseudorange for
    // satellite PRN 2 carries a bias. Truth is recoverable primarily through
    // the (unbiased) tight carrier constraint, so this isolates each sigma
    // knob's effect on the float position at the corrupted epoch.
    constexpr std::size_t kCorruptEpoch = 19;
    const Vector3d true_position(1113194.0, -4841695.0, 3985350.0);

    CpHoldTestOptions opt;
    opt.num_epochs = 25;
    for (std::size_t e = 10; e <= kCorruptEpoch; ++e) opt.pr_corrupt_epochs.insert(e);
    opt.pr_dominant_extra_bias_m = 3.0;
    auto problem = makeCpHoldFixedLagProblem(opt);

    FGOProcessor::FGOConfig base_config = makeCpHoldBaseConfig();

    FGOProcessor::FGOConfig off_config = base_config;
    ASSERT_FALSE(off_config.use_sat_badness_downweight);
    FGOProcessor off_processor(off_config);
    const auto off_result = off_processor.optimizeProblem(problem);
    const double off_err =
        (off_result.solution.solutions[kCorruptEpoch].position_ecef - true_position).norm();

    // Defaults: carrier_sigma_scale=1.5, pseudorange_sigma_scale=0.0. Carrier
    // is already far tighter than pseudorange in this fixture (0.02 m vs
    // 0.5 m), so inflating ONLY its sigma by a realistic factor barely moves
    // the relative PR/CP weighting -- the float position should stay close
    // to the badness-off baseline.
    FGOProcessor::FGOConfig cp_config = base_config;
    cp_config.use_sat_badness_downweight = true;
    ASSERT_GT(cp_config.sat_badness_carrier_sigma_scale, 0.0);
    ASSERT_EQ(cp_config.sat_badness_pseudorange_sigma_scale, 0.0);
    FGOProcessor cp_processor(cp_config);
    const auto cp_result = cp_processor.optimizeProblem(problem);
    const double cp_err =
        (cp_result.solution.solutions[kCorruptEpoch].position_ecef - true_position).norm();
    EXPECT_NEAR(cp_err, off_err, std::max(0.05, 0.2 * off_err))
        << "at defaults (pr_scale=0) the float position should be little-changed by CP-only "
           "sigma inflation (off_err=" << off_err << " m, cp_err=" << cp_err << " m)";

    // Enabling the pseudorange scale on the SAME bad pair should visibly pull
    // the float back toward truth (down-weighting the biased PR observation
    // in favour of the clean carrier).
    FGOProcessor::FGOConfig pr_config = base_config;
    pr_config.use_sat_badness_downweight = true;
    pr_config.sat_badness_carrier_sigma_scale = 0.0;
    pr_config.sat_badness_pseudorange_sigma_scale = 3.0;
    FGOProcessor pr_processor(pr_config);
    const auto pr_result = pr_processor.optimizeProblem(problem);
    const double pr_err =
        (pr_result.solution.solutions[kCorruptEpoch].position_ecef - true_position).norm();
    EXPECT_GT(pr_result.diagnostics.sat_badness_downweighted_factors, 0u);
    EXPECT_LT(pr_err, off_err)
        << "down-weighting the biased pseudorange should reduce float error vs the "
           "badness-off baseline (off_err=" << off_err << " m, pr_err=" << pr_err << " m)";
}

TEST(FGOSatBadnessTest, NotSeenThisEpochHardResetsBeatsContinuousAccumulation) {
    // Same 10-epoch chronic corruption (epochs 10..19) on satellite PRN 2 in
    // both variants; the "dropout" variant additionally removes PRN 2's DD
    // factors ENTIRELY (as if untracked) for one epoch in the middle of that
    // run. The reference hard-resets (not decays) a not-seen satellite's
    // EWMA/streak to 0, so the dropout variant's cumulative score should
    // fall clearly short of the uninterrupted run's peak.
    CpHoldTestOptions opt;
    opt.num_epochs = 25;
    for (std::size_t e = 10; e <= 19; ++e) opt.pr_corrupt_epochs.insert(e);
    opt.pr_dominant_extra_bias_m = 3.0;

    auto continuous_problem = makeCpHoldFixedLagProblem(opt);
    auto dropout_problem = makeCpHoldFixedLagProblem(opt);
    dropSatelliteAtEpoch(dropout_problem, /*epoch_index=*/15, /*satellite_prn=*/2);

    FGOProcessor::FGOConfig config = makeCpHoldBaseConfig();
    config.use_sat_badness_downweight = true;

    FGOProcessor continuous_processor(config);
    const auto continuous_result = continuous_processor.optimizeProblem(continuous_problem);
    FGOProcessor dropout_processor(config);
    const auto dropout_result = dropout_processor.optimizeProblem(dropout_problem);

    EXPECT_GT(continuous_result.diagnostics.sat_badness_max_score_seen,
              dropout_result.diagnostics.sat_badness_max_score_seen)
        << "an uninterrupted 10-epoch bad streak must accumulate a higher score than the same "
           "streak with a one-epoch absence resetting EWMA/streak mid-way (continuous="
        << continuous_result.diagnostics.sat_badness_max_score_seen
        << ", dropout=" << dropout_result.diagnostics.sat_badness_max_score_seen << ")";
}

TEST(FGOSatBadnessTest, RecentPairAlphaGatesPairMemoryContribution) {
    // alpha_recent_pair defaults to 0.0 (reference profile: the pair term is
    // provably inert). Raising it on the SAME chronically-bad-pair fixture
    // must be able to increase the observed score/down-weighting -- i.e. the
    // term is actually wired, just gated off by default.
    CpHoldTestOptions opt;
    opt.num_epochs = 25;
    for (std::size_t e = 10; e <= 19; ++e) opt.pr_corrupt_epochs.insert(e);
    opt.pr_dominant_extra_bias_m = 3.0;
    auto problem = makeCpHoldFixedLagProblem(opt);

    FGOProcessor::FGOConfig off_config = makeCpHoldBaseConfig();
    off_config.use_sat_badness_downweight = true;
    ASSERT_EQ(off_config.sat_badness_alpha_recent_pair, 0.0);
    FGOProcessor off_processor(off_config);
    const auto off_result = off_processor.optimizeProblem(problem);

    FGOProcessor::FGOConfig pair_config = off_config;
    pair_config.sat_badness_alpha_recent_pair = 5.0;
    FGOProcessor pair_processor(pair_config);
    const auto pair_result = pair_processor.optimizeProblem(problem);

    EXPECT_GT(pair_result.diagnostics.sat_badness_max_score_seen,
              off_result.diagnostics.sat_badness_max_score_seen)
        << "raising alpha_recent_pair from its inert default must raise the observed score";
}

#endif  // GNSSPP_HAS_GTSAM
