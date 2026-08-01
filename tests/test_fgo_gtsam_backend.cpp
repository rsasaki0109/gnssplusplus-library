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
#include <limits>
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
    EXPECT_EQ(gtsam_result.diagnostics.lambda_stale_candidates_filtered, 0u);
    EXPECT_EQ(gtsam_result.diagnostics.lambda_joint_marginal_failures, 0u);
}

TEST(FGOGtsamBackendTest, ReportsBuiltTdcpThatIsNotInsertedByGtsamBackend) {
    FGOProcessor::FGOProblem problem = makeGtsamParityDoubleDifferenceProblem();
    FGOProcessor::TimeDifferencedCarrierFactor tdcp;
    tdcp.previous_epoch_index = 0;
    tdcp.current_epoch_index = 1;
    tdcp.satellite = SatelliteId(GNSSSystem::GPS, 1);
    tdcp.signal = SignalType::GPS_L1CA;
    tdcp.previous_satellite_position_ecef = gtsamParitySatelliteGeometry().front();
    tdcp.current_satellite_position_ecef = gtsamParitySatelliteGeometry().front();
    tdcp.delta_carrier_m = 0.0;
    tdcp.sigma_m = 0.03;
    problem.tdcp_factors.push_back(tdcp);

    FGOProcessor::FGOConfig config = makeParityBaseConfig();
    config.backend = FGOBackend::GTSAM;
    config.use_tdcp_factors = true;

    const FGOProcessor processor(config);
    const FGOProcessor::FGOResult result = processor.optimizeProblem(problem);

    EXPECT_EQ(result.diagnostics.tdcp_factors, 1u);
    EXPECT_EQ(result.diagnostics.tdcp_factors_inserted, 0u);
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
    // Satellite geometry override; empty = gtsamParitySatelliteGeometry().
    // The fixed-lag per-epoch LAMBDA has a hard floor of 6 candidate
    // ambiguities (min_candidates in fgo_gtsam_backend.cpp), which the
    // default 6-satellite geometry (5 ambiguities) never reaches -- tests
    // that need LAMBDA/fix-and-hold to actually run must pass a bigger set.
    std::vector<Vector3d> satellites;
};

FGOProcessor::FGOProblem makeCpHoldFixedLagProblem(const CpHoldTestOptions& opt) {
    FGOProcessor::FGOProblem problem;
    const auto satellites =
        opt.satellites.empty() ? gtsamParitySatelliteGeometry() : opt.satellites;
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

TEST(FGOAmbiguityCandidateTelemetryTest, ReportsDisabledCandidateFunnel) {
    CpHoldTestOptions opt;
    opt.num_epochs = 3;
    const auto problem = makeCpHoldFixedLagProblem(opt);
    FGOProcessor::FGOConfig config = makeCpHoldBaseConfig();

    const FGOProcessor processor(config);
    const FGOProcessor::FGOResult result = processor.optimizeProblem(problem);

    ASSERT_EQ(result.epoch_diagnostics.size(), problem.epochs.size());
    for (const auto& epoch : result.epoch_diagnostics) {
        EXPECT_EQ(epoch.carrier_factors_available, 5);
        EXPECT_EQ(epoch.carrier_factors_added, 5);
        EXPECT_EQ(epoch.ambiguity_candidates_after_hold, 5);
        EXPECT_EQ(epoch.ambiguity_candidates, 5);
        EXPECT_EQ(epoch.ambiguity_candidates_final, 0);
        EXPECT_EQ(epoch.ambiguity_candidates_excluded_build_time, 0);
        EXPECT_EQ(epoch.ambiguity_candidates_excluded_hold, 0);
        EXPECT_EQ(epoch.ambiguity_candidates_excluded_one_band, 0);
        EXPECT_EQ(epoch.ambiguity_candidates_excluded_constellation, 0);
        EXPECT_EQ(epoch.ambiguity_candidates_excluded_previous_residual, 0);
        EXPECT_EQ(epoch.ambiguity_candidates_excluded_fde, 0);
        EXPECT_EQ(epoch.ambiguity_candidates_excluded_stale, 0);
        ASSERT_EQ(epoch.ambiguity_candidate_trace.size(), 5u);
        for (const auto& candidate : epoch.ambiguity_candidate_trace) {
            EXPECT_EQ(
                candidate.disposition,
                FGOProcessor::AmbiguityCandidateDisposition::
                    AmbiguityResolutionDisabled);
        }
    }
}

TEST(FGOAmbiguityCandidateTelemetryTest,
     ReportsFinalCandidatesAtInsufficientCountDecision) {
    CpHoldTestOptions opt;
    opt.num_epochs = 3;
    const auto problem = makeCpHoldFixedLagProblem(opt);
    FGOProcessor::FGOConfig config = makeCpHoldBaseConfig();
    config.use_lambda_ambiguity_fix = true;

    const FGOProcessor processor(config);
    const FGOProcessor::FGOResult result = processor.optimizeProblem(problem);

    ASSERT_EQ(result.epoch_diagnostics.size(), problem.epochs.size());
    for (const auto& epoch : result.epoch_diagnostics) {
        // Five candidates reach the count gate, so this is insufficient
        // rather than an epoch with no candidates.
        EXPECT_EQ(
            epoch.ar_outcome,
            FGOProcessor::AmbiguityResolutionOutcome::InsufficientCandidates);
        EXPECT_EQ(epoch.ambiguity_candidates_after_hold, 5);
        EXPECT_EQ(epoch.ambiguity_candidates, 5);
        EXPECT_EQ(epoch.ambiguity_candidates_final, 5);
        ASSERT_EQ(epoch.ambiguity_candidate_trace.size(), 5u);
        for (const auto& candidate : epoch.ambiguity_candidate_trace) {
            EXPECT_EQ(
                candidate.disposition,
                FGOProcessor::AmbiguityCandidateDisposition::LambdaEligible);
        }
    }
}

TEST(FGOAmbiguityCandidateTelemetryTest, ReportsBuildTimeExcludedCarrierRows) {
    CpHoldTestOptions opt;
    opt.num_epochs = 3;
    auto problem = makeCpHoldFixedLagProblem(opt);
    auto excluded = problem.double_difference_carrier_factors.front();
    excluded.ambiguity_index = std::numeric_limits<std::size_t>::max();
    problem.excluded_double_difference_carrier_factors.push_back(excluded);

    FGOProcessor::FGOConfig config = makeCpHoldBaseConfig();
    const FGOProcessor processor(config);
    const FGOProcessor::FGOResult result = processor.optimizeProblem(problem);

    ASSERT_FALSE(result.epoch_diagnostics.empty());
    const auto& epoch = result.epoch_diagnostics.front();
    EXPECT_EQ(epoch.ambiguity_candidates_excluded_build_time, 1);
    ASSERT_EQ(epoch.ambiguity_candidate_trace.size(), 6u);
    EXPECT_EQ(
        epoch.ambiguity_candidate_trace.front().disposition,
        FGOProcessor::AmbiguityCandidateDisposition::BuildTimeExcluded);
}

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

TEST(FGOCpHoldFsmTest, LongWindowMarginalizationKeepsCurrentPoseKeysValid) {
    CpHoldTestOptions opt;
    opt.num_epochs = 120;
    auto problem = makeCpHoldFixedLagProblem(opt);
    // Let every ambiguity disappear for longer than the six-second lag and
    // then reappear. This exercises both pose-chain marginalization and the
    // ambiguity-key reinsertion path over many lag-window turnovers.
    problem.double_difference_carrier_factors.erase(
        std::remove_if(
            problem.double_difference_carrier_factors.begin(),
            problem.double_difference_carrier_factors.end(),
            [](const FGOProcessor::DoubleDifferenceCarrierFactor& factor) {
                return factor.epoch_index >= 40 && factor.epoch_index <= 55;
            }),
        problem.double_difference_carrier_factors.end());

    FGOProcessor::FGOConfig config = makeCpHoldBaseConfig();
    config.use_solve_exception_recovery = true;
    FGOProcessor processor(config);
    const auto result = processor.optimizeProblem(problem);

    ASSERT_EQ(result.solution.solutions.size(), problem.epochs.size());
    EXPECT_TRUE(std::none_of(
        result.solution.solutions.begin(), result.solution.solutions.end(),
        [](const PositionSolution& solution) {
            return solution.status == SolutionStatus::NONE;
        }));
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

// ----------------------------------------------------------------------------
// Leaky persist accumulator (FGOConfig::use_cp_hold_leaky_persist) -- C1.
// Scenario: an INTERMITTENTLY-bad stretch (corrupt/clean epochs alternating
// one-for-one) never reaches cp_hold_persist_epochs CONSECUTIVE bad epochs
// under the reference's hard reset, so the persist path never fires. With
// leaky decay smaller than the per-bad-epoch increment (+1), the counter
// survives the interleaved clean epochs and accumulates net credit toward
// the threshold.
// ----------------------------------------------------------------------------

// Intermittent-bad fixture: a UNIFORM (diffuse -- not single-satellite)
// pseudorange-only bias on alternating epochs. Carriers stay clean (tight
// 0.02 m sigma), so the graph pose never leaves truth and the injected bias
// shows up directly and ONLY on the flagged epoch's own post-fit DD PR
// residual -- no cross-epoch memory/decay tail to confound the persist
// counter's consecutive-vs-intermittent distinction (unlike a carrier/pose
// corruption, which the fixed-lag window forgets only gradually).
FGOProcessor::FGOProblem makeLeakyPersistIntermittentProblem() {
    CpHoldTestOptions opt;
    opt.num_epochs = 20;
    opt.pr_corrupt_epochs = {5, 7, 9};
    opt.pr_baseline_bias_m = 12.0;  // uniform on every target -> diffuse, > 3 m threshold
    return makeCpHoldFixedLagProblem(opt);
}

TEST(FGOCpHoldFsmTest, LeakyPersistDefaultOffIsNoOp) {
    const auto problem = makeLeakyPersistIntermittentProblem();

    FGOProcessor::FGOConfig config = makeCpHoldBaseConfig();
    config.use_cp_hold_recovery = true;
    config.cp_hold_main_residual_threshold_m = 3.0;
    config.cp_hold_persist_epochs = 3;
    config.cp_hold_catastrophic_threshold_m = 1.0e6;  // never take the fast path here
    config.cp_hold_multipath_median_ratio = 0.0;      // diffuse bias, no multipath skip
    config.cp_hold_max_gdop = 0.0;
    ASSERT_FALSE(config.use_cp_hold_leaky_persist);
    ASSERT_DOUBLE_EQ(config.cp_hold_persist_decay, 1.0);

    FGOProcessor processor(config);
    const auto result = processor.optimizeProblem(problem);

    EXPECT_EQ(result.diagnostics.sanity_mass_resets, 0u)
        << "fixture sanity: one-for-one alternating bad/clean epochs never "
           "reach 3 CONSECUTIVE bad epochs under the hard reset";
    for (const auto& sol : result.solution.solutions) {
        EXPECT_TRUE(sol.position_ecef.allFinite());
    }
}

TEST(FGOCpHoldFsmTest, LeakyPersistAccumulatesAcrossIntermittentBadEpochs) {
    const auto problem = makeLeakyPersistIntermittentProblem();

    FGOProcessor::FGOConfig base = makeCpHoldBaseConfig();
    base.use_cp_hold_recovery = true;
    base.cp_hold_main_residual_threshold_m = 3.0;
    base.cp_hold_persist_epochs = 3;
    base.cp_hold_catastrophic_threshold_m = 1.0e6;
    base.cp_hold_multipath_median_ratio = 0.0;
    base.cp_hold_max_gdop = 0.0;

    // OFF (hard reset): the alternating pattern never crosses the persist
    // threshold (see LeakyPersistDefaultOffIsNoOp above).
    FGOProcessor off_processor(base);
    const auto off_result = off_processor.optimizeProblem(problem);
    ASSERT_EQ(off_result.diagnostics.sanity_mass_resets, 0u);

    // ON: decay (0.5) smaller than the +1-per-bad-epoch increment lets bad
    // epochs interleaved with clean ones accumulate net credit past the
    // persist_epochs=3 threshold.
    FGOProcessor::FGOConfig on_config = base;
    on_config.use_cp_hold_leaky_persist = true;
    on_config.cp_hold_persist_decay = 0.5;
    FGOProcessor on_processor(on_config);
    const auto on_result = on_processor.optimizeProblem(problem);

    EXPECT_GT(on_result.diagnostics.sanity_mass_resets, 0u)
        << "leaky decay must let intermittent bad epochs accumulate past the "
           "persist threshold where the hard reset never does";
}

TEST(FGOCpHoldFsmTest, LeakyPersistDecayAtPersistThresholdMatchesHardReset) {
    // cp_hold_persist_decay >= cp_hold_persist_epochs drains the counter to 0
    // in a single clean epoch -- reproducing the hard reset exactly -- even
    // though the knob is nominally "on".
    const auto problem = makeLeakyPersistIntermittentProblem();

    FGOProcessor::FGOConfig config = makeCpHoldBaseConfig();
    config.use_cp_hold_recovery = true;
    config.cp_hold_main_residual_threshold_m = 3.0;
    config.cp_hold_persist_epochs = 3;
    config.cp_hold_catastrophic_threshold_m = 1.0e6;
    config.cp_hold_multipath_median_ratio = 0.0;
    config.cp_hold_max_gdop = 0.0;
    config.use_cp_hold_leaky_persist = true;
    config.cp_hold_persist_decay = 3.0;  // == cp_hold_persist_epochs

    FGOProcessor processor(config);
    const auto result = processor.optimizeProblem(problem);

    EXPECT_EQ(result.diagnostics.sanity_mass_resets, 0u)
        << "decay >= persist_epochs drains a single clean epoch fully, just "
           "like the hard reset -- alternation must still never accumulate";
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
    const auto diagnostic_it = std::find_if(
        outlier_result.epoch_diagnostics.begin(), outlier_result.epoch_diagnostics.end(),
        [](const auto& diagnostic) {
            return diagnostic.ddpr_anchor_bootstrap_prior_applied;
        });
    ASSERT_NE(diagnostic_it, outlier_result.epoch_diagnostics.end());
    EXPECT_TRUE(diagnostic_it->ddpr_anchor_evaluated);
    EXPECT_GE(diagnostic_it->ddpr_anchor_active_factors, 4);
    EXPECT_TRUE(std::isfinite(diagnostic_it->ddpr_anchor_residual_rms_m));
    EXPECT_GT(diagnostic_it->ddpr_anchor_position_ecef.norm(), 1e6);

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

    // Single-satellite persistent carrier-only step (ambiguity_index 0,
    // i.e. satellite PRN 2): mutate the same raw model field the GTSAM
    // carrier factor is built from, well past where fix-and-hold should have
    // pinned this arc (noise-free synthetic geometry converges in a handful
    // of epochs). FDE must reject the transition once and mint a fresh
    // ambiguity generation that can absorb the new constant bias; repeatedly
    // rejecting every later epoch would prove that the generation bump is
    // diagnostic-only and not used by ambSymbolId().
    constexpr std::size_t kOutlierEpoch = 15;
    constexpr double kOutlierBiasM = 5.0;  // >> fde_carrier_threshold_m (0.5)
    for (auto& cp : problem.double_difference_carrier_factors) {
        if (cp.epoch_index < kOutlierEpoch || cp.ambiguity_index != 0) continue;
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
    EXPECT_LT(on_result.diagnostics.fde_carrier_rejections, 5u)
        << "a persistent carrier step should be absorbed by one fresh ambiguity "
           "generation instead of being rejected throughout the remaining arc";
    EXPECT_EQ(on_result.diagnostics.fde_pseudorange_rejections, 0u);
    EXPECT_GT(on_result.diagnostics.ambiguity_generation_bumps, 0u)
        << "the rejected carrier factor's arc should get a fresh generation, "
           "i.e. be treated as a cycle slip";
    for (const auto& sol : on_result.solution.solutions) {
        EXPECT_TRUE(sol.position_ecef.allFinite());
    }
}

TEST(FGOAmbiguityReacquisitionTest, ContinuousUnfixResetStartsFreshArcsWithoutCpHold) {
    CpHoldTestOptions opt;
    opt.num_epochs = 14;
    opt.satellites = gtsamParitySatelliteGeometry();
    opt.satellites.emplace_back(-20000000.0, 10000000.0, 12000000.0);
    opt.satellites.emplace_back(9000000.0, 20000000.0, 15000000.0);
    const auto problem = makeCpHoldFixedLagProblem(opt);

    FGOProcessor::FGOConfig config = makeCpHoldBaseConfig();
    config.use_lambda_ambiguity_fix = true;
    config.lambda_ratio_threshold = std::numeric_limits<double>::max();
    config.use_continuous_unfix_ambiguity_reset = true;
    config.continuous_unfix_reset_epochs = 2;
    config.continuous_unfix_min_satellites = 6;
    config.continuous_unfix_max_gdop = 100.0;
    config.continuous_unfix_max_fde_reject_fraction = 1.0;

    auto baseline_config = config;
    baseline_config.use_continuous_unfix_ambiguity_reset = false;
    FGOProcessor baseline_processor(baseline_config);
    const auto baseline_result = baseline_processor.optimizeProblem(problem);

    FGOProcessor processor(config);
    const auto result = processor.optimizeProblem(problem);

    EXPECT_GT(result.diagnostics.ambiguity_continuous_unfix_resets, 0u);
    EXPECT_GT(result.diagnostics.ambiguity_generation_bumps_reset, 0u);
    EXPECT_EQ(result.diagnostics.ambiguity_generation_bumps,
              result.diagnostics.ambiguity_generation_bumps_reset);
    EXPECT_EQ(result.diagnostics.cp_hold_triggers, 0u)
        << "reacquisition must not engage the CP-hold recovery FSM";
    ASSERT_EQ(result.solution.solutions.size(), problem.epochs.size());
    ASSERT_EQ(result.solution.solutions.size(),
              baseline_result.solution.solutions.size());
    bool changed_after_reset = false;
    for (std::size_t i = 0; i < result.solution.solutions.size(); ++i) {
        if ((result.solution.solutions[i].position_ecef -
             baseline_result.solution.solutions[i].position_ecef).norm() >
            1e-9) {
            changed_after_reset = true;
        }
    }
    EXPECT_TRUE(changed_after_reset)
        << "generation bumps must mint fresh graph keys, not only counters";
    for (const auto& solution : result.solution.solutions) {
        EXPECT_TRUE(solution.position_ecef.allFinite());
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

// Injects a raw carrier bias into ONE (epoch, satellite)'s DD carrier factor,
// mutating the same raw model field the GTSAM factor is built from AND
// observed_dd_carrier_m -- the carrier analogue of injectPseudorangeOutlier
// above. Used (with config.use_fde on) to engineer genuine single-satellite
// carrier outliers for FDE to reject, which is what populates
// sb_fde_cp_reject_count (the CLAMPED variant's decayed cppr substitute).
void injectCarrierOutlier(FGOProcessor::FGOProblem& problem, std::size_t epoch_index,
                          uint8_t satellite_prn, double bias_m) {
    for (auto& cp : problem.double_difference_carrier_factors) {
        if (cp.epoch_index != epoch_index || cp.satellite.prn != satellite_prn) continue;
        cp.rover_satellite_model.corrected_carrier_m += bias_m;
        cp.observed_dd_carrier_m += bias_m;
    }
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
    // CLAMPED-variant deviation: this test's invariant is about the score's
    // ORDERING (chronic vs clean), not about the cap -- disable it so the
    // new default sat_badness_score_cap (3.0) can't flatten both sides to
    // the same capped value and make the comparison vacuous.
    config.sat_badness_score_cap = 0.0;

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
    // CLAMPED-variant deviation: disable the new default score cap so it
    // can't flatten both sides to the same capped value (see the analogous
    // note in ChronicallyBadSatelliteScoresHigherThanCleanRun above).
    config.sat_badness_score_cap = 0.0;

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
    // CLAMPED-variant deviation: disable the new default score cap so it
    // can't flatten both sides to the same capped value (see the analogous
    // note in ChronicallyBadSatelliteScoresHigherThanCleanRun above).
    off_config.sat_badness_score_cap = 0.0;
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

// ============================================================================
// CLAMPED variant (fgo.hpp: sat_badness_residual_clamp_m / sat_badness_
// score_cap / sat_badness_cppr_decay) -- deliberate DEVIATION from the
// reference bounding the faithful port's feedback loop. See fgo.hpp's
// comment on these three knobs for the mechanistic rationale (the reference
// port is harmless in the reference's own ~1-2 m FLOAT regime but explodes
// on this codebase's 100s-of-meters deep-urban FLOAT excursions).
// ============================================================================

TEST(FGOSatBadnessClampedTest, ResidualClampMakesHugeResidualBehaveLikeTheClampValue) {
    // Two configs sharing the SAME clamp (15 m, the shipped default) but with
    // wildly different raw biases (100 m vs 10000 m) on the same corrupted
    // satellite/epochs. Both raw residuals land far above the clamp, so
    // EVERY badness term that reads the per-satellite residual (obsq EWMA,
    // obsq bad-streak, the next-epoch res_s snapshot, recent_ref_bad's
    // upstream input) must see the SAME hard-clamped-to-15 value in both --
    // i.e. a 100 m residual "behaves as" a 15 m one, regardless of how much
    // further it overshoots. (Comparing against a genuine, unclamped 15 m
    // bias instead is NOT a valid equivalence: the graph's Huber-robustified
    // least-squares absorbs a small fraction of ANY bias into the state, and
    // that absorbed fraction is itself bias-size-dependent near the Huber
    // knee -- so a real 15 m case and a clamped-from-100 m case are not
    // expected to match bit-for-bit. Two biases that both clamp is the clean
    // invariant.) Score cap disabled in both so it can't hide a mismatch.
    CpHoldTestOptions huge_opt;
    huge_opt.num_epochs = 25;
    for (std::size_t e = 10; e <= 19; ++e) huge_opt.pr_corrupt_epochs.insert(e);
    huge_opt.pr_dominant_extra_bias_m = 100.0;
    auto huge_problem = makeCpHoldFixedLagProblem(huge_opt);

    CpHoldTestOptions astronomical_opt;
    astronomical_opt.num_epochs = 25;
    for (std::size_t e = 10; e <= 19; ++e) astronomical_opt.pr_corrupt_epochs.insert(e);
    astronomical_opt.pr_dominant_extra_bias_m = 10000.0;
    auto astronomical_problem = makeCpHoldFixedLagProblem(astronomical_opt);

    FGOProcessor::FGOConfig config = makeCpHoldBaseConfig();
    config.use_sat_badness_downweight = true;
    ASSERT_EQ(config.sat_badness_residual_clamp_m, 15.0)
        << "test assumes the shipped default clamp";
    config.sat_badness_score_cap = 0.0;  // isolate the clamp from the cap

    FGOProcessor huge_processor(config);
    const auto huge_result = huge_processor.optimizeProblem(huge_problem);
    FGOProcessor astronomical_processor(config);
    const auto astronomical_result = astronomical_processor.optimizeProblem(astronomical_problem);

    EXPECT_NEAR(huge_result.diagnostics.sat_badness_max_score_seen,
                astronomical_result.diagnostics.sat_badness_max_score_seen, 1e-6)
        << "once the raw residual clears the clamp, growing it further (100m -> 10000m) must not "
           "change the score at all -- both must behave as exactly the clamp value (100m="
        << huge_result.diagnostics.sat_badness_max_score_seen
        << ", 10000m=" << astronomical_result.diagnostics.sat_badness_max_score_seen << ")";
    EXPECT_GT(huge_result.diagnostics.sat_badness_max_score_seen, 0.0)
        << "sanity: the corrupted satellite must actually register a nonzero score";
}

TEST(FGOSatBadnessClampedTest, ScoreCapBoundsTheFinalScore) {
    // An extreme 100 m bias with the clamp at its default (15 m) still lets
    // several additive terms in satBadness() accumulate substantially; the
    // score cap must nonetheless bound the value actually consumed for sigma
    // inflation. Compare against the SAME fixture with the cap disabled to
    // prove the cap is doing real work here, not just trivially satisfied.
    CpHoldTestOptions opt;
    opt.num_epochs = 25;
    for (std::size_t e = 5; e <= 19; ++e) opt.pr_corrupt_epochs.insert(e);
    opt.pr_dominant_extra_bias_m = 100.0;
    auto problem = makeCpHoldFixedLagProblem(opt);

    FGOProcessor::FGOConfig capped_config = makeCpHoldBaseConfig();
    capped_config.use_sat_badness_downweight = true;
    ASSERT_EQ(capped_config.sat_badness_score_cap, 3.0) << "test assumes the shipped default cap";
    FGOProcessor capped_processor(capped_config);
    const auto capped_result = capped_processor.optimizeProblem(problem);

    FGOProcessor::FGOConfig uncapped_config = capped_config;
    uncapped_config.sat_badness_score_cap = 0.0;  // 0 = no cap = faithful
    FGOProcessor uncapped_processor(uncapped_config);
    const auto uncapped_result = uncapped_processor.optimizeProblem(problem);

    EXPECT_LE(capped_result.diagnostics.sat_badness_max_score_seen,
              capped_config.sat_badness_score_cap + 1e-9)
        << "the capped run's score must never exceed sat_badness_score_cap (observed="
        << capped_result.diagnostics.sat_badness_max_score_seen << ")";
    EXPECT_GT(uncapped_result.diagnostics.sat_badness_max_score_seen,
              capped_config.sat_badness_score_cap)
        << "sanity: on this fixture the uncapped score must actually exceed the cap, otherwise "
           "the cap assertion above is vacuous (uncapped="
        << uncapped_result.diagnostics.sat_badness_max_score_seen << ")";
}

TEST(FGOSatBadnessClampedTest, CpprDecayFadesAfterFdeRejectsStop) {
    // Two well-separated bursts of genuine single-satellite carrier outliers
    // (PRN 2), each big enough that FDE (config.use_fde) rejects that DD
    // carrier factor outright, feeding sb_fde_cp_reject_count. With
    // sat_badness_cppr_decay=1.0 (never decays, the old faithful-port
    // behaviour) the SECOND burst's score carries the FIRST burst's count
    // forward undiminished; with the default 0.8 decay, ~25 clean epochs in
    // between decay that carry-over most of the way to zero. Every other
    // badness knob is identical between the two runs (same fixture, same
    // everything else), so any score difference is attributable to this one
    // knob.
    constexpr uint8_t kPrn = 2;
    constexpr double kBiasM = 5.0;  // >> fde_carrier_threshold_m default (0.5 m)
    CpHoldTestOptions opt;
    opt.num_epochs = 40;
    auto problem = makeCpHoldFixedLagProblem(opt);
    for (std::size_t e = 5; e <= 7; ++e) injectCarrierOutlier(problem, e, kPrn, kBiasM);
    for (std::size_t e = 30; e <= 32; ++e) injectCarrierOutlier(problem, e, kPrn, kBiasM);

    FGOProcessor::FGOConfig base_config = makeCpHoldBaseConfig();
    base_config.use_fde = true;
    base_config.use_sat_badness_downweight = true;
    base_config.sat_badness_score_cap = 0.0;  // isolate the decay: don't let the cap flatten it

    FGOProcessor::FGOConfig never_decay_config = base_config;
    never_decay_config.sat_badness_cppr_decay = 1.0;  // faithful: never decays
    FGOProcessor never_decay_processor(never_decay_config);
    const auto never_decay_result = never_decay_processor.optimizeProblem(problem);

    FGOProcessor::FGOConfig decay_config = base_config;
    ASSERT_EQ(decay_config.sat_badness_cppr_decay, 0.8) << "test assumes the shipped default decay";
    FGOProcessor decay_processor(decay_config);
    const auto decay_result = decay_processor.optimizeProblem(problem);

    ASSERT_GT(never_decay_result.diagnostics.fde_carrier_rejections, 0u)
        << "sanity: the injected outliers must actually get FDE-rejected in both bursts";
    ASSERT_GT(decay_result.diagnostics.fde_carrier_rejections, 0u);

    EXPECT_GT(never_decay_result.diagnostics.sat_badness_max_score_seen,
              decay_result.diagnostics.sat_badness_max_score_seen)
        << "an ever-growing (never-decaying) reject counter must score higher at the second "
           "burst than one that decayed away in between (never_decay="
        << never_decay_result.diagnostics.sat_badness_max_score_seen
        << ", decay=0.8=" << decay_result.diagnostics.sat_badness_max_score_seen << ")";
}

TEST(FGOSatBadnessClampedTest, FaithfulEscapeHatchReproducesUnboundedScoring) {
    // clamp=0, cap=0, cppr_decay=1.0 together must disable all three CLAMPED-
    // variant bounds at once, reproducing the original faithful port's
    // unbounded scoring exactly. Demonstrated here by contrast: on an extreme
    // 100 m bias fixture, the escape-hatch config's score must blow well past
    // both the shipped clamp (15 m) and cap (3.0) -- if any bound were still
    // silently active, the score could not grow this large.
    CpHoldTestOptions opt;
    opt.num_epochs = 25;
    for (std::size_t e = 5; e <= 19; ++e) opt.pr_corrupt_epochs.insert(e);
    opt.pr_dominant_extra_bias_m = 100.0;
    auto problem = makeCpHoldFixedLagProblem(opt);

    FGOProcessor::FGOConfig default_config = makeCpHoldBaseConfig();
    default_config.use_sat_badness_downweight = true;
    ASSERT_EQ(default_config.sat_badness_residual_clamp_m, 15.0);
    ASSERT_EQ(default_config.sat_badness_score_cap, 3.0);
    ASSERT_EQ(default_config.sat_badness_cppr_decay, 0.8);
    FGOProcessor default_processor(default_config);
    const auto default_result = default_processor.optimizeProblem(problem);

    FGOProcessor::FGOConfig escape_config = default_config;
    escape_config.sat_badness_residual_clamp_m = 0.0;  // no clamp
    escape_config.sat_badness_score_cap = 0.0;         // no cap
    escape_config.sat_badness_cppr_decay = 1.0;        // never decays
    FGOProcessor escape_processor(escape_config);
    const auto escape_result = escape_processor.optimizeProblem(problem);

    EXPECT_LE(default_result.diagnostics.sat_badness_max_score_seen,
              default_config.sat_badness_score_cap + 1e-9)
        << "sanity: the default (bounded) run must respect its own cap";
    EXPECT_GT(escape_result.diagnostics.sat_badness_max_score_seen,
              default_config.sat_badness_score_cap * 2.0)
        << "the escape hatch must let the score run well past the shipped cap (escape="
        << escape_result.diagnostics.sat_badness_max_score_seen
        << ", default(bounded)=" << default_result.diagnostics.sat_badness_max_score_seen << ")";
}

// --- IMU preintegration-covariance value semantics + per-epoch inflation
// (port of the inuex35 reference's buildfactor/imu_preintegration.py's
// _apply_mres_integ_cov_override + config.py's imu_integ_cov /
// imu_integ_cov_max -- see FGOConfig::imu_integration_covariance's comment
// in fgo.hpp for the full mapping). ---

TEST(FGOImuIntegrationCovarianceTest, DefaultsMatchPortedReferenceMapping) {
    const FGOProcessor::FGOConfig config;
    EXPECT_FALSE(config.use_integer_constrained_reoptimization);
    EXPECT_DOUBLE_EQ(config.integer_constrained_prior_sigma_cycles, 1e-3);
    EXPECT_DOUBLE_EQ(config.integer_constrained_cost_abs_tolerance, 1e-6);
    EXPECT_EQ(config.integer_constrained_max_iterations, 1);
    EXPECT_TRUE(config.report_held_ambiguities_as_fixed);
    EXPECT_FALSE(config.use_continuous_unfix_ambiguity_reset);
    EXPECT_FALSE(
        config.continuous_unfix_require_ddpr_anchor_disagreement);
    EXPECT_DOUBLE_EQ(config.continuous_unfix_anchor_min_gap_m, 1.0);
    EXPECT_DOUBLE_EQ(config.fixed_postfit_normal_ratio_ceiling, 0.0);
    EXPECT_DOUBLE_EQ(config.surplus_validation_veto_ratio_ceiling, 0.0);
    EXPECT_DOUBLE_EQ(config.surplus_validation_veto_min_ddpr_rms_m, 0.0);
    // 1e-6 == sq(1e-3), the harness's hardcoded (pre-port) effective
    // covariance -- this default alone must not change any existing run.
    EXPECT_DOUBLE_EQ(config.imu_integration_covariance, 1e-6);
    EXPECT_FALSE(config.use_imu_integration_covariance_inflation);
    EXPECT_DOUBLE_EQ(config.imu_integration_covariance_max, 0.5);   // reference imu_integ_cov_max
    EXPECT_EQ(config.imu_integration_covariance_stale_epochs, 2);
}

namespace {
// Pure reimplementation of the exact formula documented on
// FGOConfig::use_imu_integration_covariance_inflation (and applied in
// fgo_gtsam_backend.cpp's optimizeProblemFixedLag immediately before each
// epoch's PIM is constructed), so this test can check hand-computed values
// without depending on gtsam internals or a full nonlinear solve.
double expectedIntegEff(double default_cov, double cap, int stale_epochs,
                        long long epoch, long long last_mres_epoch,
                        double last_mres, double dt) {
    const bool is_stale =
        stale_epochs > 0 && (epoch - last_mres_epoch) > stale_epochs;
    if (is_stale) return default_cov;
    double integ_eff = std::max(default_cov, (last_mres * last_mres) / std::max(dt, 1e-3));
    if (cap > 0.0) integ_eff = std::min(integ_eff, cap);
    return integ_eff;
}
}  // namespace

TEST(FGOImuIntegrationCovarianceTest, InflationFormulaMaxCapAndStalenessHandComputed) {
    const double default_cov = 1e-3;  // reference imu_integ_cov value
    const double cap = 0.5;           // reference imu_integ_cov_max default

    // (a) Clean residual (mres=0): floor wins regardless of dt.
    EXPECT_DOUBLE_EQ(expectedIntegEff(default_cov, cap, 2, 10, 9, 0.0, 1.0), default_cov);

    // (b) Small residual that doesn't clear the floor: mres=0.02 m, dt=1.0 s
    // -> mres^2/dt = 4e-4 < default_cov (1e-3), so the floor still wins.
    EXPECT_DOUBLE_EQ(expectedIntegEff(default_cov, cap, 2, 10, 9, 0.02, 1.0), default_cov);

    // (c) Moderate residual that clears the floor but stays under the cap:
    // mres=0.1 m, dt=1.0 s -> mres^2/dt = 0.01, between 1e-3 and 0.5.
    EXPECT_DOUBLE_EQ(expectedIntegEff(default_cov, cap, 2, 10, 9, 0.1, 1.0), 0.01);

    // (d) Large residual that would blow past the cap: mres=5.0 m, dt=1.0 s
    // -> mres^2/dt = 25.0, clamped down to imu_integ_cov_max (0.5).
    EXPECT_DOUBLE_EQ(expectedIntegEff(default_cov, cap, 2, 10, 9, 5.0, 1.0), cap);

    // (e) Same large residual but a shorter dt makes it even larger pre-cap
    // (mres^2/dt = 25.0/0.2 = 125.0) -- still clamped to the same cap.
    EXPECT_DOUBLE_EQ(expectedIntegEff(default_cov, cap, 2, 10, 9, 5.0, 0.2), cap);

    // (f) Staleness: the same large residual as (d), but recorded 5 epochs
    // ago with stale_epochs=2 (5 > 2) -- the signal is stale, so the
    // inflation is skipped entirely and the static floor is used, even
    // though mres^2/dt alone would blow past both the floor and the cap.
    EXPECT_DOUBLE_EQ(expectedIntegEff(default_cov, cap, 2, 10, 5, 5.0, 1.0), default_cov);

    // (g) Exactly at the staleness boundary (epoch - last_mres_epoch ==
    // stale_epochs) is NOT stale (reference: `> stale_max`, strict): the
    // inflation still applies.
    EXPECT_DOUBLE_EQ(expectedIntegEff(default_cov, cap, 2, 10, 8, 5.0, 1.0), cap);

    // (h) stale_epochs<=0 disables the staleness check entirely (reference:
    // `stale_max > 0 and ...`) -- inflation applies no matter how old the
    // residual is.
    EXPECT_DOUBLE_EQ(expectedIntegEff(default_cov, cap, 0, 10, -1000000, 0.1, 1.0), 0.01);

    // (i) cap<=0 disables the cap entirely (reference: `if cap > 0`).
    EXPECT_DOUBLE_EQ(expectedIntegEff(default_cov, 0.0, 2, 10, 9, 5.0, 1.0), 25.0);
}

TEST(FGOImuIntegrationCovarianceTest, InflationIsNoOpOnCleanResidualsRegardlessOfSwitch) {
    // With no injected corruption, the post-fit DDPR RMS stays effectively
    // zero every epoch, so the inflation formula's max(default_cov, mres^2/dt)
    // collapses to default_cov on every epoch -- turning the switch on must
    // not perturb a clean run at all (wiring sanity + "default-off unchanged"
    // companion: this shows the ON path degenerates to the OFF path when the
    // residual signal has nothing to say).
    CpHoldTestOptions opt;
    opt.num_epochs = 25;  // clean, stationary, no corruption
    const auto problem = makeCpHoldFixedLagProblem(opt);

    FGOProcessor::FGOConfig off_config = makeCpHoldBaseConfig();
    off_config.imu_integration_covariance = 1e-3;
    ASSERT_FALSE(off_config.use_imu_integration_covariance_inflation);
    FGOProcessor off_processor(off_config);
    const auto off_result = off_processor.optimizeProblem(problem);

    FGOProcessor::FGOConfig on_config = off_config;
    on_config.use_imu_integration_covariance_inflation = true;
    FGOProcessor on_processor(on_config);
    const auto on_result = on_processor.optimizeProblem(problem);

    ASSERT_EQ(off_result.solution.solutions.size(), on_result.solution.solutions.size());
    for (std::size_t i = 0; i < off_result.solution.solutions.size(); ++i) {
        const auto& a = off_result.solution.solutions[i];
        const auto& b = on_result.solution.solutions[i];
        EXPECT_TRUE(a.position_ecef.isApprox(b.position_ecef, 0.0) || a.position_ecef == b.position_ecef)
            << "epoch " << i << " position diverged with a clean (zero-residual) run";
    }
}

TEST(FGOImuIntegrationCovarianceTest, ValuePassthroughNotSquaredAffectsDragResistance) {
    // Reuses the CP-hold FSM's "wrong basin" fixture: a self-consistent
    // erroneous carrier-phase hypothesis drags the graph pose away from the
    // (stationary) true position during the corrupt window. A TIGHTER
    // integration covariance makes the IMU chain stiffer (more resistant to
    // being dragged by the erroneous carrier pull); a LOOSER one lets the
    // carrier win more easily. If the backend still squared this field
    // internally (the pre-port bug), configuring 1e-3 would silently become
    // a covariance of 1e-6 -- identical to the "tight" run below -- so this
    // comparison would collapse to no difference and the test would fail.
    CpHoldTestOptions opt;
    opt.carrier_corrupt_epochs = {5, 6, 7, 8, 9, 10, 11, 12};
    const auto problem = makeCpHoldFixedLagProblem(opt);
    const Vector3d true_position(1113194.0, -4841695.0, 3985350.0);
    constexpr std::size_t kProbeEpoch = 9;

    FGOProcessor::FGOConfig tight_config = makeCpHoldBaseConfig();
    tight_config.imu_integration_covariance = 1e-6;  // shipped default
    FGOProcessor tight_processor(tight_config);
    const auto tight_result = tight_processor.optimizeProblem(problem);
    const double tight_dev =
        (tight_result.solution.solutions[kProbeEpoch].position_ecef - true_position).norm();

    FGOProcessor::FGOConfig loose_config = makeCpHoldBaseConfig();
    loose_config.imu_integration_covariance = 1e-3;  // reference imu_integ_cov, applied directly
    FGOProcessor loose_processor(loose_config);
    const auto loose_result = loose_processor.optimizeProblem(problem);
    const double loose_dev =
        (loose_result.solution.solutions[kProbeEpoch].position_ecef - true_position).norm();

    EXPECT_GT(loose_dev, tight_dev)
        << "a 1000x-looser integration covariance (applied directly, not squared) must let the "
           "erroneous carrier pull drag the pose further from truth than the tight default "
           "(tight_dev=" << tight_dev << " m, loose_dev=" << loose_dev << " m)";
}

// ============================================================================
// Stale-pin invalidation (FGOConfig::use_stale_pin_invalidation) -- per-arc
// fix-and-hold pin release at the CP-hold FSM trigger. Reuses
// makeCpHoldFixedLagProblem/makeCpHoldBaseConfig.
//
// Scenario: LAMBDA + fix-and-hold pin all 5 arcs during the clean opening
// epochs, then a DOMINANT single-satellite PSEUDORANGE bias (carriers stay
// clean, so the tight carrier constraints keep the pose at truth and the
// bias shows up as that satellite's per-sat post-fit residual) pushes the
// epoch DDPR RMS over the trigger threshold. The FSM's escalation paths are
// all disabled (persist unreachable, catastrophic unreachable, cp_hold_epochs
// = 0 so no carrier suppression / held-epoch generation bumps), leaving
// stale-pin invalidation as the ONLY mechanism that may bump a generation --
// so ambiguity_generation_bumps == stale_pin_invalidations proves the
// release was per-arc, not a mass reset.
// ============================================================================
namespace {

// 8 satellites -> 7 ambiguities: clears the fixed-lag per-epoch LAMBDA's
// hard floor of 6 candidates (the default 6-satellite geometry never fixes).
std::vector<Vector3d> lambdaCapableSatelliteGeometry() {
    auto satellites = gtsamParitySatelliteGeometry();
    satellites.emplace_back(-20000000.0, 10000000.0, 12000000.0);
    satellites.emplace_back(9000000.0, 20000000.0, 15000000.0);
    return satellites;
}

FGOProcessor::FGOConfig makeStalePinBaseConfig() {
    FGOProcessor::FGOConfig config = makeCpHoldBaseConfig();
    // Fix-and-hold (pins must exist to be released).
    config.use_lambda_ambiguity_fix = true;
    config.use_ambiguity_hold = true;
    config.lambda_ratio_threshold = 1.5;  // easily cleared by noise-free synthetic data
    config.ambiguity_hold_ratio_threshold = 1.5;
    config.ambiguity_hold_min_fixed = 4;
    config.min_fixed_ambiguities = 5;
    // FSM on (the trigger is the mechanism's hook) but every escalation off.
    config.use_cp_hold_recovery = true;
    config.cp_hold_main_residual_threshold_m = 3.0;
    config.cp_hold_persist_epochs = 1000;             // persist path unreachable
    config.cp_hold_catastrophic_threshold_m = 1.0e6;  // fast path unreachable
    config.cp_hold_multipath_median_ratio = 0.0;      // no multipath skip (single-sat scenario)
    config.cp_hold_epochs = 0;                        // no carrier suppression on trigger
    config.cp_hold_max_gdop = 0.0;
    return config;
}

FGOProcessor::FGOProblem makeStalePinProblem() {
    CpHoldTestOptions opt;
    opt.satellites = lambdaCapableSatelliteGeometry();
    opt.num_epochs = 30;
    // Corruption starts well after the pins form (noise-free geometry fixes
    // within a handful of epochs): dominant PR bias on satellite index 1
    // (PRN2, ambiguity_index 0) only.
    for (std::size_t e = 15; e <= 24; ++e) opt.pr_corrupt_epochs.insert(e);
    opt.pr_baseline_bias_m = 0.0;
    opt.pr_dominant_extra_bias_m = 12.0;  // per-sat res ~12 m; epoch RMS ~12/sqrt(5) > 3
    return makeCpHoldFixedLagProblem(opt);
}

}  // namespace

TEST(FGOStalePinTest, DefaultOffIsNoOp) {
    const auto problem = makeStalePinProblem();

    FGOProcessor::FGOConfig config = makeStalePinBaseConfig();
    ASSERT_FALSE(config.use_stale_pin_invalidation);

    FGOProcessor processor(config);
    const auto result = processor.optimizeProblem(problem);

    EXPECT_EQ(result.diagnostics.stale_pin_invalidations, 0u);
    // With every other generation-bumping mechanism disabled in this config
    // (no FDE, no mass/fast reset, cp_hold_epochs=0 so no held-epoch bumps),
    // the generation overlay must stay untouched too.
    EXPECT_EQ(result.diagnostics.ambiguity_generation_bumps, 0u);
    EXPECT_GT(result.diagnostics.ambiguity_hold_arcs, 0u)
        << "sanity: pins must actually form for this fixture to test anything";
    EXPECT_EQ(result.solution.solutions.size(), problem.epochs.size());
}

TEST(FGOAmbiguityOutcomeTelemetryTest, HoldFallbackPreservesRatioRejection) {
    CpHoldTestOptions opt;
    opt.satellites = lambdaCapableSatelliteGeometry();
    opt.num_epochs = 12;
    const auto problem = makeCpHoldFixedLagProblem(opt);

    FGOProcessor::FGOConfig config = makeStalePinBaseConfig();
    config.lambda_ratio_threshold = 1.0e200;
    config.ambiguity_hold_ratio_threshold = 1.0e200;
    config.use_cp_hold_recovery = false;

    FGOProcessor processor(config);
    const auto result = processor.optimizeProblem(problem);

    ASSERT_EQ(result.epoch_diagnostics.size(), problem.epochs.size());
    const auto rejected = std::find_if(
        result.epoch_diagnostics.begin(), result.epoch_diagnostics.end(),
        [](const FGOProcessor::FGOEpochDiagnostics& diagnostics) {
            return diagnostics.ar_outcome ==
                   FGOProcessor::AmbiguityResolutionOutcome::RatioRejected;
        });
    ASSERT_NE(rejected, result.epoch_diagnostics.end())
        << "the held-ambiguity fallback must not overwrite a terminal ratio-test rejection";
    EXPECT_TRUE(rejected->lambda_candidate_available);
    EXPECT_GT(rejected->lambda_candidate_position_ecef.norm(), 1.0e6);
    EXPECT_GT(rejected->lambda_candidate_fixed_ambiguities, 0);
    EXPECT_TRUE(std::isfinite(rejected->lambda_candidate_ratio));
    EXPECT_GT(rejected->lambda_candidate_bsr, 0.0);
    EXPECT_LE(rejected->lambda_candidate_bsr, 1.0);
    EXPECT_LE(rejected->lambda_candidate_bsr_qscale2,
              rejected->lambda_candidate_bsr);
    EXPECT_LE(rejected->lambda_candidate_bsr_qscale4,
              rejected->lambda_candidate_bsr_qscale2);
    EXPECT_LE(rejected->lambda_candidate_bsr_qscale8,
              rejected->lambda_candidate_bsr_qscale4);
    EXPECT_LE(rejected->lambda_candidate_bsr_qscale16,
              rejected->lambda_candidate_bsr_qscale8);
    EXPECT_TRUE(rejected->lambda_candidate_ffrt_table_supported);
    EXPECT_EQ(rejected->lambda_candidate_ffrt_pass,
              rejected->lambda_candidate_ffrt_accepts_any &&
                  rejected->lambda_candidate_ratio >
                      rejected->lambda_candidate_ffrt_min_ratio);
    const auto consensus = std::find_if(
        result.epoch_diagnostics.begin(), result.epoch_diagnostics.end(),
        [](const FGOProcessor::FGOEpochDiagnostics& diagnostics) {
            return diagnostics.lambda_candidate_integer_consensus_streak >= 2;
        });
    ASSERT_NE(consensus, result.epoch_diagnostics.end());
    EXPECT_GE(consensus->lambda_candidate_integer_overlap, 4);
    EXPECT_EQ(consensus->lambda_candidate_integer_agreements,
              consensus->lambda_candidate_integer_overlap);
    EXPECT_DOUBLE_EQ(consensus->lambda_candidate_integer_agreement_fraction,
                     1.0);
}

TEST(FGOAmbiguityOutcomeTelemetryTest,
     RatioImpactMonitorIsDiagnosticOnlyAndRecordsBestExclusion) {
    CpHoldTestOptions opt;
    opt.satellites = lambdaCapableSatelliteGeometry();
    opt.num_epochs = 12;
    const auto problem = makeCpHoldFixedLagProblem(opt);

    FGOProcessor::FGOConfig config = makeStalePinBaseConfig();
    config.lambda_ratio_threshold = 1.0e200;
    config.ambiguity_hold_ratio_threshold = 1.0e200;
    config.use_cp_hold_recovery = false;
    config.use_fixed_lag_partial_lambda = true;

    FGOProcessor baseline_processor(config);
    const auto baseline = baseline_processor.optimizeProblem(problem);

    config.monitor_ratio_impact_partial_ar = true;

    FGOProcessor processor(config);
    const auto result = processor.optimizeProblem(problem);

    ASSERT_EQ(result.solution.solutions.size(), baseline.solution.solutions.size());
    for (std::size_t i = 0; i < result.solution.solutions.size(); ++i) {
        EXPECT_EQ(result.solution.solutions[i].status,
                  baseline.solution.solutions[i].status);
        EXPECT_TRUE(result.solution.solutions[i].position_ecef.isApprox(
            baseline.solution.solutions[i].position_ecef, 0.0));
    }

    const auto monitored = std::find_if(
        result.epoch_diagnostics.begin(), result.epoch_diagnostics.end(),
        [](const FGOProcessor::FGOEpochDiagnostics& diagnostics) {
            return diagnostics.ratio_impact_evaluated &&
                   diagnostics.ratio_impact_best_fixed_ambiguities > 0;
        });
    ASSERT_NE(monitored, result.epoch_diagnostics.end());
    EXPECT_EQ(monitored->ar_outcome,
              FGOProcessor::AmbiguityResolutionOutcome::RatioRejected);
    EXPECT_GT(monitored->ratio_impact_trials, 0);
    EXPECT_GT(monitored->ratio_impact_best_ratio, 0.0);
    EXPECT_GT(monitored->ratio_impact_best_position_ecef.norm(), 1.0e6);
    ASSERT_FALSE(monitored->ratio_impact_trial_trace.empty());
    const auto available_trial = std::find_if(
        monitored->ratio_impact_trial_trace.begin(),
        monitored->ratio_impact_trial_trace.end(),
        [](const FGOProcessor::RatioImpactTrialTrace& trial) {
            return trial.candidate_available;
        });
    ASSERT_NE(available_trial, monitored->ratio_impact_trial_trace.end());
    EXPECT_GT(available_trial->excluded_ambiguities, 0);
    EXPECT_GT(available_trial->fixed_ambiguities, 0);
    EXPECT_GT(available_trial->candidate_position_ecef.norm(), 1.0e6);
}

TEST(FGOAmbiguityOutcomeTelemetryTest, NoCarrierCandidatesRemainNoCandidates) {
    CpHoldTestOptions opt;
    opt.satellites = lambdaCapableSatelliteGeometry();
    opt.num_epochs = 4;
    auto problem = makeCpHoldFixedLagProblem(opt);
    problem.double_difference_carrier_factors.clear();

    FGOProcessor::FGOConfig config = makeStalePinBaseConfig();
    config.use_cp_hold_recovery = false;

    FGOProcessor processor(config);
    const auto result = processor.optimizeProblem(problem);

    ASSERT_EQ(result.epoch_diagnostics.size(), problem.epochs.size());
    for (const auto& diagnostics : result.epoch_diagnostics) {
        EXPECT_EQ(diagnostics.ar_outcome,
                  FGOProcessor::AmbiguityResolutionOutcome::NoCandidates);
    }
}

TEST(FGOAmbiguityOutcomeTelemetryTest, BelowFloorCandidatesAreInsufficient) {
    CpHoldTestOptions opt;
    opt.satellites = gtsamParitySatelliteGeometry();
    opt.num_epochs = 4;
    const auto problem = makeCpHoldFixedLagProblem(opt);

    FGOProcessor::FGOConfig config = makeStalePinBaseConfig();
    config.use_cp_hold_recovery = false;

    FGOProcessor processor(config);
    const auto result = processor.optimizeProblem(problem);

    ASSERT_EQ(result.epoch_diagnostics.size(), problem.epochs.size());
    for (const auto& diagnostics : result.epoch_diagnostics) {
        ASSERT_EQ(diagnostics.ambiguity_candidates, 5);
        EXPECT_EQ(diagnostics.ar_outcome,
                  FGOProcessor::AmbiguityResolutionOutcome::InsufficientCandidates);
    }
}

TEST(FGOSppSeedTelemetryTest, ReportsFreshnessAndBuilderPositionReadOnly) {
    auto problem = makeStalePinProblem();
    for (std::size_t i = 0; i < problem.epochs.size(); ++i) {
        problem.epochs[i].fresh_spp_solution = (i % 2u) == 0u;
    }
    FGOProcessor processor(makeStalePinBaseConfig());
    const auto result = processor.optimizeProblem(problem);

    ASSERT_EQ(result.epoch_diagnostics.size(), problem.epochs.size());
    for (std::size_t i = 0; i < problem.epochs.size(); ++i) {
        EXPECT_EQ(result.epoch_diagnostics[i].fresh_spp_solution,
                  problem.epochs[i].fresh_spp_solution);
        EXPECT_TRUE(result.epoch_diagnostics[i]
                        .spp_seed_position_ecef.isApprox(
                            problem.epochs[i].position_ecef, 1e-12));
    }
}

TEST(FGOStalePinTest, ReleasesOnlyOffendingPinAtTrigger) {
    const auto problem = makeStalePinProblem();

    FGOProcessor::FGOConfig config = makeStalePinBaseConfig();
    config.use_stale_pin_invalidation = true;
    config.stale_pin_per_sat_residual_m = 2.0;

    FGOProcessor processor(config);
    const auto result = processor.optimizeProblem(problem);

    EXPECT_GT(result.diagnostics.stale_pin_invalidations, 0u)
        << "the dominant-satellite pin must be released at a trigger epoch";
    // Per-arc, not mass: the only generation bumps allowed in this config
    // are the stale-pin releases themselves.
    EXPECT_EQ(result.diagnostics.ambiguity_generation_bumps,
              result.diagnostics.stale_pin_invalidations);
    EXPECT_EQ(result.diagnostics.sanity_mass_resets, 0u);
    EXPECT_EQ(result.diagnostics.sanity_fast_resets, 0u);
    // Only PRN2's arc ever exceeds the per-sat threshold, and it can be
    // re-pinned/re-released at most once per corrupt epoch (10 of them).
    EXPECT_LE(result.diagnostics.stale_pin_invalidations, 10u);
    for (const auto& sol : result.solution.solutions) {
        EXPECT_TRUE(sol.position_ecef.allFinite());
    }
}

TEST(FGOStalePinTest, ReleasesPinOnMultipathDominatedEpoch) {
    // The dominant-single-satellite scenario is exactly what the FSM's
    // multipath skip catches (one bad satellite is not a wrong basin, so no
    // MASS reset) -- but a per-arc release of that satellite's pin is the
    // right-sized response, so stale-pin invalidation must still fire there.
    const auto problem = makeStalePinProblem();

    FGOProcessor::FGOConfig config = makeStalePinBaseConfig();
    config.cp_hold_multipath_median_ratio = 1.5;  // multipath skip ACTIVE
    config.cp_hold_multipath_min_satellites = 6;
    config.use_stale_pin_invalidation = true;
    config.stale_pin_per_sat_residual_m = 2.0;

    FGOProcessor processor(config);
    const auto result = processor.optimizeProblem(problem);

    EXPECT_GT(result.diagnostics.sanity_multipath_skips, 0u)
        << "fixture sanity: the dominant-satellite epochs must be classified "
           "multipath-dominated";
    EXPECT_GT(result.diagnostics.stale_pin_invalidations, 0u)
        << "the per-arc release must fire on multipath-dominated trigger epochs";
    EXPECT_EQ(result.diagnostics.sanity_mass_resets, 0u);
    EXPECT_EQ(result.diagnostics.sanity_fast_resets, 0u);
}

TEST(FGOStalePinTest, MinHoldAgeGuardsFreshPins) {
    const auto problem = makeStalePinProblem();

    FGOProcessor::FGOConfig config = makeStalePinBaseConfig();
    config.use_stale_pin_invalidation = true;
    config.stale_pin_per_sat_residual_m = 2.0;
    config.stale_pin_min_hold_age_epochs = 1000;  // no pin can ever be old enough

    FGOProcessor processor(config);
    const auto result = processor.optimizeProblem(problem);

    EXPECT_EQ(result.diagnostics.stale_pin_invalidations, 0u)
        << "an unreachable min-hold-age must suppress every release";
    EXPECT_EQ(result.diagnostics.ambiguity_generation_bumps, 0u);
}

TEST(FGOStalePinTest, PerSatThresholdRespected) {
    const auto problem = makeStalePinProblem();

    FGOProcessor::FGOConfig config = makeStalePinBaseConfig();
    config.use_stale_pin_invalidation = true;
    config.stale_pin_per_sat_residual_m = 1.0e6;  // nothing ever exceeds this

    FGOProcessor processor(config);
    const auto result = processor.optimizeProblem(problem);

    EXPECT_EQ(result.diagnostics.stale_pin_invalidations, 0u);
    EXPECT_EQ(result.diagnostics.ambiguity_generation_bumps, 0u);
}

// ============================================================================
// Fix plausibility demotion (FGOConfig::use_fix_plausibility_demotion) --
// label-level IMU-gap check, independent of the CP-hold FSM. Reuses
// makeCpHoldFixedLagProblem.
//
// Scenario: LAMBDA + fix-and-hold pin all arcs during the clean opening
// epochs, then a single-epoch "wrong basin" carrier corruption (all
// satellites, +20 m self-consistent hypothesis). Because the ambiguities
// are PINNED at the clean integers, the corrupted (tight-sigma) carrier
// factors drag the graph pose toward the wrong position while the
// IMU-predicted pose (dead-reckoned from the clean previous epoch,
// stationary rover) stays at truth -- an epoch labelled FIXED (via held
// integers) whose fixed position is implausibly far from the IMU
// prediction, exactly the label M2 must demote. The FSM stays OFF to prove
// independence.
// ============================================================================
namespace {

FGOProcessor::FGOConfig makeFixDemoteBaseConfig() {
    FGOProcessor::FGOConfig config = makeCpHoldBaseConfig();
    config.use_lambda_ambiguity_fix = true;
    config.use_ambiguity_hold = true;
    config.use_epoch_lambda_fixed_output = true;  // fresh LAMBDA fixes also labelled FIXED
    config.lambda_ratio_threshold = 1.5;
    config.ambiguity_hold_ratio_threshold = 1.5;
    config.ambiguity_hold_min_fixed = 4;
    config.min_fixed_ambiguities = 5;
    // NO CP-hold FSM: M2 must work without it.
    config.use_cp_hold_recovery = false;
    return config;
}

}  // namespace

TEST(FGOFixDemoteTest, DefaultOffIsNoOp) {
    CpHoldTestOptions opt;
    opt.satellites = lambdaCapableSatelliteGeometry();
    opt.num_epochs = 25;
    opt.carrier_corrupt_epochs = {15};
    const auto problem = makeCpHoldFixedLagProblem(opt);

    FGOProcessor::FGOConfig config = makeFixDemoteBaseConfig();
    ASSERT_FALSE(config.use_fix_plausibility_demotion);

    FGOProcessor processor(config);
    const auto result = processor.optimizeProblem(problem);

    EXPECT_EQ(result.diagnostics.fix_plausibility_demotions, 0u);
    EXPECT_EQ(result.diagnostics.fix_plausibility_hold_skips, 0u);
    EXPECT_EQ(result.solution.solutions.size(), problem.epochs.size());
}

TEST(FGOFixDemoteTest, DemotesImplausibleFixedEpochToFloat) {
    const Vector3d true_position(1113194.0, -4841695.0, 3985350.0);
    constexpr std::size_t kBadEpoch = 15;
    constexpr std::size_t kCleanProbeEpoch = 10;

    CpHoldTestOptions opt;
    opt.satellites = lambdaCapableSatelliteGeometry();
    opt.num_epochs = 25;
    opt.carrier_corrupt_epochs = {kBadEpoch};
    opt.carrier_corrupt_offset_ecef = Vector3d(20.0, 0.0, 0.0);
    const auto problem = makeCpHoldFixedLagProblem(opt);

    // OFF: the corrupted epoch is labelled FIXED (held integers) with a large
    // position error -- the exact failure mode.
    FGOProcessor::FGOConfig off_config = makeFixDemoteBaseConfig();
    FGOProcessor off_processor(off_config);
    const auto off_result = off_processor.optimizeProblem(problem);
    ASSERT_EQ(off_result.solution.solutions.size(), problem.epochs.size());
    const auto& off_bad = off_result.solution.solutions[kBadEpoch];
    ASSERT_EQ(off_bad.status, SolutionStatus::FIXED)
        << "fixture sanity: the corrupt epoch must be (wrongly) labelled FIXED without M2";
    const double off_err = (off_bad.position_ecef - true_position).norm();
    ASSERT_GT(off_err, 5.0)
        << "fixture sanity: the wrong-basin drag must exceed the demotion distance";

    // ON: same epoch demoted to FLOAT; clean epochs keep their FIXED label.
    FGOProcessor::FGOConfig on_config = makeFixDemoteBaseConfig();
    on_config.use_fix_plausibility_demotion = true;
    on_config.fix_demote_distance_m = 5.0;
    FGOProcessor on_processor(on_config);
    const auto on_result = on_processor.optimizeProblem(problem);
    ASSERT_EQ(on_result.solution.solutions.size(), problem.epochs.size());

    EXPECT_GT(on_result.diagnostics.fix_plausibility_demotions, 0u);
    EXPECT_NE(on_result.solution.solutions[kBadEpoch].status, SolutionStatus::FIXED)
        << "the implausible FIXED label must be demoted";
    EXPECT_EQ(on_result.solution.solutions[kCleanProbeEpoch].status, SolutionStatus::FIXED)
        << "plausible fixes must keep their FIXED label";
}

TEST(FGOFixDemoteTest, AnchorGapDemotesWhenImuGapCheckIsBlind) {
    // Isolate the anchor-gap path: the IMU-gap threshold is set unreachably
    // high (simulating the "IMU prediction rides the wrong basin" blindness
    // measured on tokyo run2), so ONLY a trusted DDPR-LS anchor -- re-solved
    // from this epoch's clean DD pseudoranges at the true position -- can
    // notice that the (wrong-basin) FIXED position is implausible.
    constexpr std::size_t kBadEpoch = 15;

    CpHoldTestOptions opt;
    opt.satellites = lambdaCapableSatelliteGeometry();
    opt.num_epochs = 25;
    opt.carrier_corrupt_epochs = {kBadEpoch};
    opt.carrier_corrupt_offset_ecef = Vector3d(20.0, 0.0, 0.0);
    const auto problem = makeCpHoldFixedLagProblem(opt);

    FGOProcessor::FGOConfig config = makeFixDemoteBaseConfig();
    config.use_fix_plausibility_demotion = true;
    config.fix_demote_distance_m = 1.0e9;  // IMU-gap check blind
    config.use_ddpr_anchor = true;         // anchor plumbing available
    config.fix_demote_use_ddpr_anchor = true;
    config.fix_demote_anchor_distance_m = 3.0;

    FGOProcessor processor(config);
    const auto result = processor.optimizeProblem(problem);
    ASSERT_EQ(result.solution.solutions.size(), problem.epochs.size());

    EXPECT_GT(result.diagnostics.fix_plausibility_anchor_demotions, 0u)
        << "the trusted anchor at truth must veto the wrong-basin FIXED label";
    EXPECT_NE(result.solution.solutions[kBadEpoch].status, SolutionStatus::FIXED);
    EXPECT_EQ(result.solution.solutions[10].status, SolutionStatus::FIXED)
        << "clean epochs (anchor agrees with the fix) keep their FIXED label";
}

// ----------------------------------------------------------------------------
// Gross-offender gate on the anchor-gap variant (FGOConfig::
// fix_demote_anchor_gross) -- C2. Round-1/2 measured the ungated anchor-gap
// variant catastrophic on tokyo run1 (1662 false demotions): run1's failure
// mode is DIFFUSE multipath, which drags the (non-robust) anchor by the same
// few metres as a genuinely wrong-basin fix, with every tracked satellite's
// per-sat residual roughly the same order (no single dominant offender). The
// gate is meant to let a GROSS single-satellite-offender epoch (tokyo run3's
// actual band signature) through while suppressing a diffuse one (run1's).
// Both tests reuse AnchorGapDemotesWhenImuGapCheckIsBlind's wrong-basin
// carrier-corruption fixture (all satellites, a shared +20 m hypothesis --
// the diffuse shape) and turn on use_epoch_quality_gates with thresholds too
// loose to ever gate, purely to activate the shared per-sat residual pass
// (per_sat_res) the gate reads -- exactly as ExtremeResidualDemotesRegardless
// OfImuAgreement below does for the same reason.
// ----------------------------------------------------------------------------

TEST(FGOFixDemoteTest, AnchorGrossGateSuppressesDiffuseFalseVeto) {
    constexpr std::size_t kBadEpoch = 15;

    CpHoldTestOptions opt;
    opt.satellites = lambdaCapableSatelliteGeometry();
    opt.num_epochs = 25;
    opt.carrier_corrupt_epochs = {kBadEpoch};
    opt.carrier_corrupt_offset_ecef = Vector3d(20.0, 0.0, 0.0);  // diffuse: all satellites
    const auto problem = makeCpHoldFixedLagProblem(opt);

    FGOProcessor::FGOConfig config = makeFixDemoteBaseConfig();
    config.use_epoch_quality_gates = true;  // activate the shared per_sat_res pass
    config.gate_gdop_max = 1e9;
    config.gate_min_satellites = 0;
    config.gate_ddpr_res_max_m = 1e9;
    config.gate_per_sat_res_max_m = 1e9;
    config.use_fix_plausibility_demotion = true;
    config.fix_demote_distance_m = 1.0e9;  // IMU-gap check blind
    config.use_ddpr_anchor = true;
    config.fix_demote_use_ddpr_anchor = true;
    config.fix_demote_anchor_distance_m = 3.0;
    config.fix_demote_anchor_gross = true;
    // A shared +20 m position error shows up as roughly the SAME order of
    // residual on every tracked satellite (diffuse) -- nowhere near a
    // 50 m single-satellite abs floor, so the abs criterion alone (which the
    // gate requires ANDed with the ratio criterion) never clears regardless
    // of how the ratio happens to fall out on this geometry.
    config.fix_demote_anchor_gross_ratio = 10.0;
    config.fix_demote_anchor_gross_abs_m = 50.0;

    FGOProcessor processor(config);
    const auto result = processor.optimizeProblem(problem);
    ASSERT_EQ(result.solution.solutions.size(), problem.epochs.size());

    EXPECT_GT(result.diagnostics.fix_plausibility_anchor_gross_gated, 0u)
        << "the diffuse epoch must never clear the gross-offender signature";
    EXPECT_EQ(result.diagnostics.fix_plausibility_anchor_demotions, 0u)
        << "gated out: the anchor-gap check must never even be evaluated";
    EXPECT_EQ(result.solution.solutions[kBadEpoch].status, SolutionStatus::FIXED)
        << "without the (gated-out) anchor veto the wrong-basin label stands "
           "-- the false-positive this gate exists to suppress";
}

TEST(FGOFixDemoteTest, AnchorGrossGateLetsThroughSingleSatelliteOffender) {
    constexpr std::size_t kBadEpoch = 15;

    CpHoldTestOptions opt;
    opt.satellites = lambdaCapableSatelliteGeometry();
    opt.num_epochs = 25;
    // Same wrong-basin carrier corruption as the diffuse case (so the FIXED
    // label is equally implausible and equally worth vetoing), PLUS a huge
    // ADDITIONAL pseudorange-only bias on satellite index 1 at the same
    // epoch. PR sigma is loose (0.5 m) vs the carrier's tight 0.02 m, so this
    // barely moves the graph pose -- it only inflates that one satellite's
    // post-fit DD residual, giving the epoch a gross single-satellite
    // signature on top of the same underlying wrong-basin fix.
    opt.carrier_corrupt_epochs = {kBadEpoch};
    opt.carrier_corrupt_offset_ecef = Vector3d(20.0, 0.0, 0.0);
    opt.pr_corrupt_epochs = {kBadEpoch};
    opt.pr_baseline_bias_m = 0.0;
    opt.pr_dominant_extra_bias_m = 300.0;
    const auto problem = makeCpHoldFixedLagProblem(opt);

    FGOProcessor::FGOConfig config = makeFixDemoteBaseConfig();
    config.use_epoch_quality_gates = true;
    config.gate_gdop_max = 1e9;
    config.gate_min_satellites = 0;
    config.gate_ddpr_res_max_m = 1e9;
    config.gate_per_sat_res_max_m = 1e9;
    config.use_fix_plausibility_demotion = true;
    config.fix_demote_distance_m = 1.0e9;  // IMU-gap check blind
    config.use_ddpr_anchor = true;
    config.fix_demote_use_ddpr_anchor = true;
    config.fix_demote_anchor_distance_m = 3.0;
    config.fix_demote_anchor_gross = true;
    config.fix_demote_anchor_gross_ratio = 10.0;
    config.fix_demote_anchor_gross_abs_m = 50.0;

    FGOProcessor processor(config);
    const auto result = processor.optimizeProblem(problem);
    ASSERT_EQ(result.solution.solutions.size(), problem.epochs.size());

    EXPECT_GT(result.diagnostics.fix_plausibility_anchor_demotions, 0u)
        << "the gross single-satellite signature must open the gate and let "
           "the (robust-retried) anchor veto the wrong-basin FIXED label";
    EXPECT_NE(result.solution.solutions[kBadEpoch].status, SolutionStatus::FIXED);
    EXPECT_EQ(result.solution.solutions[10].status, SolutionStatus::FIXED)
        << "clean epochs keep their FIXED label";
}

TEST(FGOFixDemoteTest, AnchorGrossGateDefaultOffMatchesUngatedBaseline) {
    // fix_demote_anchor_gross must be a strict no-op when off: bit-identical
    // demotion outcome to the pre-existing (ungated) anchor-gap variant.
    constexpr std::size_t kBadEpoch = 15;

    CpHoldTestOptions opt;
    opt.satellites = lambdaCapableSatelliteGeometry();
    opt.num_epochs = 25;
    opt.carrier_corrupt_epochs = {kBadEpoch};
    opt.carrier_corrupt_offset_ecef = Vector3d(20.0, 0.0, 0.0);
    const auto problem = makeCpHoldFixedLagProblem(opt);

    FGOProcessor::FGOConfig config = makeFixDemoteBaseConfig();
    config.use_fix_plausibility_demotion = true;
    config.fix_demote_distance_m = 1.0e9;
    config.use_ddpr_anchor = true;
    config.fix_demote_use_ddpr_anchor = true;
    config.fix_demote_anchor_distance_m = 3.0;
    ASSERT_FALSE(config.fix_demote_anchor_gross);

    FGOProcessor processor(config);
    const auto result = processor.optimizeProblem(problem);

    EXPECT_EQ(result.diagnostics.fix_plausibility_anchor_gross_gated, 0u);
    EXPECT_GT(result.diagnostics.fix_plausibility_anchor_demotions, 0u)
        << "fixture sanity: matches AnchorGapDemotesWhenImuGapCheckIsBlind's "
           "ungated outcome";
    EXPECT_NE(result.solution.solutions[kBadEpoch].status, SolutionStatus::FIXED);
}

TEST(FGOFixDemoteTest, ExtremeResidualDemotesRegardlessOfImuAgreement) {
    // fix_demote_res_m path: pseudorange-corrupted epochs push the post-fit
    // DDPR RMS far above the threshold while the pose (pinned by clean
    // carriers) stays at truth -- so the IMU gap is ~0 and the IMU/anchor
    // checks would never demote. The extreme-residual check must strip the
    // FIXED label anyway.
    CpHoldTestOptions opt;
    opt.satellites = lambdaCapableSatelliteGeometry();
    opt.num_epochs = 25;
    for (std::size_t e = 15; e <= 18; ++e) opt.pr_corrupt_epochs.insert(e);
    opt.pr_baseline_bias_m = 0.0;
    opt.pr_dominant_extra_bias_m = 40.0;  // epoch RMS ~40/sqrt(7) >> 5
    const auto problem = makeCpHoldFixedLagProblem(opt);

    // The shared post-fit residual pass only runs when a consumer feature is
    // enabled; use the quality-gates flag with thresholds too loose to ever
    // gate, so ONLY the residual computation is activated.
    FGOProcessor::FGOConfig base = makeFixDemoteBaseConfig();
    base.use_epoch_quality_gates = true;
    base.gate_gdop_max = 1e9;
    base.gate_min_satellites = 0;
    base.gate_ddpr_res_max_m = 1e9;
    base.gate_per_sat_res_max_m = 1e9;

    FGOProcessor off_processor(base);
    const auto off_result = off_processor.optimizeProblem(problem);
    ASSERT_EQ(off_result.solution.solutions[16].status, SolutionStatus::FIXED)
        << "fixture sanity: the corrupt epoch stays FIXED without the residual demotion";

    FGOProcessor::FGOConfig on_config = base;
    on_config.use_fix_plausibility_demotion = true;
    on_config.fix_demote_distance_m = 1.0e9;  // IMU-gap check blind
    on_config.fix_demote_res_m = 5.0;
    FGOProcessor on_processor(on_config);
    const auto on_result = on_processor.optimizeProblem(problem);

    EXPECT_GT(on_result.diagnostics.fix_plausibility_demotions, 0u);
    EXPECT_NE(on_result.solution.solutions[16].status, SolutionStatus::FIXED);
    EXPECT_EQ(on_result.solution.solutions[10].status, SolutionStatus::FIXED)
        << "clean epochs keep their FIXED label";
}

TEST(FGOFixDemoteTest, FreshSppAndStrongModelReprieveResidualOnlyDemotion) {
    CpHoldTestOptions opt;
    opt.satellites = lambdaCapableSatelliteGeometry();
    opt.num_epochs = 25;
    for (std::size_t e = 15; e <= 18; ++e) opt.pr_corrupt_epochs.insert(e);
    opt.pr_baseline_bias_m = 0.0;
    opt.pr_dominant_extra_bias_m = 40.0;
    auto problem = makeCpHoldFixedLagProblem(opt);

    FGOProcessor::FGOConfig config = makeFixDemoteBaseConfig();
    config.use_epoch_quality_gates = true;
    config.gate_gdop_max = 1e9;
    config.gate_min_satellites = 0;
    config.gate_ddpr_res_max_m = 1e9;
    config.gate_per_sat_res_max_m = 1e9;
    config.use_fix_plausibility_demotion = true;
    config.fix_demote_distance_m = 1.0e9;
    config.fix_demote_res_m = 5.0;
    config.fix_demote_spp_model_reprieve = true;
    // The compact synthetic fixture has fewer ambiguities than the frozen
    // production floor. Keep every other gate realistic while lowering only
    // this fixture-size requirement.
    config.fix_demote_spp_model_min_fixed_ambiguities = 1;

    // A coasted/header fallback must fail closed even when its stored
    // position happens to agree with the fixed candidate.
    FGOProcessor stale_processor(config);
    const auto stale_result = stale_processor.optimizeProblem(problem);
    ASSERT_NE(stale_result.solution.solutions[16].status,
              SolutionStatus::FIXED);
    EXPECT_EQ(stale_result.diagnostics.fix_plausibility_spp_model_reprieves,
              0u);

    for (auto& epoch : problem.epochs) epoch.fresh_spp_solution = true;
    FGOProcessor fresh_processor(config);
    const auto fresh_result = fresh_processor.optimizeProblem(problem);

    EXPECT_GT(fresh_result.diagnostics.fix_plausibility_spp_model_reprieves,
              0u);
    EXPECT_EQ(fresh_result.solution.solutions[16].status,
              SolutionStatus::FIXED)
        << "a fresh SPP witness agreeing with a strong carrier candidate "
           "must reprieve an isolated absolute-DDPR demotion";
    EXPECT_EQ(fresh_result.diagnostics.fix_plausibility_demotions, 0u);

    // Agreement cannot override another simultaneous demotion reason.
    config.fix_demote_distance_m = 0.0;
    FGOProcessor simultaneous_processor(config);
    const auto simultaneous_result =
        simultaneous_processor.optimizeProblem(problem);
    EXPECT_NE(simultaneous_result.solution.solutions[16].status,
              SolutionStatus::FIXED);
    EXPECT_EQ(
        simultaneous_result.diagnostics.fix_plausibility_spp_model_reprieves,
        0u);
}

TEST(FGOFixDemoteTest, RelativeResidualDemotesExcursionButToleratesChronicNoise) {
    // fix_demote_res_rel semantics: (1) a residual EXCURSION over a quiet
    // ambient demotes; (2) the SAME absolute residual level, when it is the
    // run's chronic normal (median high), does not -- the protection that
    // lets one preset serve both a chronically-noisy run (tokyo run1) and a
    // quiet run with wrong-basin excursions (run2/run3).
    FGOProcessor::FGOConfig base = makeFixDemoteBaseConfig();
    base.use_epoch_quality_gates = true;  // activate the shared residual pass
    base.gate_gdop_max = 1e9;
    base.gate_min_satellites = 0;
    base.gate_ddpr_res_max_m = 1e9;
    base.gate_per_sat_res_max_m = 1e9;
    base.use_fix_plausibility_demotion = true;
    base.fix_demote_distance_m = 1.0e9;  // isolate the relative criterion
    base.fix_demote_res_rel = 4.0;

    // (1) Quiet ambient, then a corrupt stretch: rms jumps from ~0 to ~15
    // (> floor 3.0, > 4x median) -- must demote.
    {
        CpHoldTestOptions opt;
        opt.satellites = lambdaCapableSatelliteGeometry();
        opt.num_epochs = 40;
        for (std::size_t e = 30; e <= 33; ++e) opt.pr_corrupt_epochs.insert(e);
        opt.pr_dominant_extra_bias_m = 40.0;
        const auto problem = makeCpHoldFixedLagProblem(opt);
        FGOProcessor processor(base);
        const auto result = processor.optimizeProblem(problem);
        EXPECT_GT(result.diagnostics.fix_plausibility_demotions, 0u)
            << "an excursion far above the quiet ambient must demote";
        EXPECT_NE(result.solution.solutions[31].status, SolutionStatus::FIXED);
    }
    // (2) Chronic noise: after a clean opening (pins/fixes established), a
    // moderate bias on every target row persists for the rest of the run.
    // Epoch RMS sits at ~4.5 m -- above the 3.0 m absolute floor, so an
    // ABSOLUTE criterion at that level would demote every remaining epoch.
    // The relative test may demote around the step's ONSET (a genuine
    // excursion over the clean median) but must stop once the rolling
    // median has adapted: late epochs keep their FIXED label.
    {
        CpHoldTestOptions opt;
        opt.satellites = lambdaCapableSatelliteGeometry();
        opt.num_epochs = 80;
        for (std::size_t e = 10; e < opt.num_epochs; ++e) opt.pr_corrupt_epochs.insert(e);
        opt.pr_baseline_bias_m = 4.5;
        const auto problem = makeCpHoldFixedLagProblem(opt);
        FGOProcessor processor(base);
        const auto result = processor.optimizeProblem(problem);
        const auto& last = result.solution.solutions[opt.num_epochs - 1];
        EXPECT_EQ(last.status, SolutionStatus::FIXED)
            << "once the rolling median reflects the run's chronic normal, "
               "fixes at that level must keep their label";
        EXPECT_LT(result.diagnostics.fix_plausibility_demotions, 40u)
            << "only the onset may demote -- never the whole chronic stretch";
    }
}

TEST(FGOFixDemoteTest, PostHoldCooldownDemotesFixesRightAfterRelease) {
    // fix_demote_posthold_epochs path: engage a real CP-hold via the FSM
    // (persist path on a corrupt stretch), then verify that the fixes
    // validated within the cooldown window after the hold releases are
    // demoted, while later fixes (cooldown expired) keep their label.
    CpHoldTestOptions opt;
    opt.satellites = lambdaCapableSatelliteGeometry();
    opt.num_epochs = 40;
    for (std::size_t e = 10; e <= 12; ++e) opt.carrier_corrupt_epochs.insert(e);
    const auto problem = makeCpHoldFixedLagProblem(opt);

    FGOProcessor::FGOConfig base = makeFixDemoteBaseConfig();
    base.use_cp_hold_recovery = true;
    base.cp_hold_main_residual_threshold_m = 3.0;
    base.cp_hold_persist_epochs = 3;
    base.cp_hold_catastrophic_threshold_m = 1.0e6;
    base.cp_hold_multipath_median_ratio = 0.0;
    base.cp_hold_max_gdop = 0.0;
    base.cp_hold_epochs = 3;
    base.cp_hold_release_threshold_m = 2.0;
    base.cp_hold_release_count = 1;

    FGOProcessor off_processor(base);
    const auto off_result = off_processor.optimizeProblem(problem);
    ASSERT_GT(off_result.diagnostics.cp_hold_epochs_held, 0u)
        << "fixture sanity: a hold must actually engage";
    std::size_t off_fixed = 0;
    for (const auto& sol : off_result.solution.solutions) {
        if (sol.status == SolutionStatus::FIXED) ++off_fixed;
    }
    ASSERT_GT(off_fixed, 0u);

    FGOProcessor::FGOConfig on_config = base;
    on_config.use_fix_plausibility_demotion = true;
    on_config.fix_demote_distance_m = 1.0e9;  // isolate the cooldown criterion
    on_config.fix_demote_posthold_epochs = 5;
    FGOProcessor on_processor(on_config);
    const auto on_result = on_processor.optimizeProblem(problem);

    EXPECT_GT(on_result.diagnostics.fix_plausibility_demotions, 0u)
        << "fixes validated within 5 epochs of the released hold must be demoted";
    std::size_t on_fixed = 0;
    for (const auto& sol : on_result.solution.solutions) {
        if (sol.status == SolutionStatus::FIXED) ++on_fixed;
    }
    EXPECT_LT(on_fixed, off_fixed);
    EXPECT_EQ(on_result.solution.solutions[problem.epochs.size() - 1].status,
              SolutionStatus::FIXED)
        << "fixes far past the cooldown keep their FIXED label";
}

TEST(FGOFixDemoteTest, ExtremeThresholdNeverPinsAndDemotesEveryFix) {
    // Clean data + an (absurd) near-zero plausibility distance: EVERY fix is
    // "implausible", so no epoch may end up FIXED and -- via the hold-skip
    // half of the mechanism -- no arc may ever be pinned.
    CpHoldTestOptions opt;
    opt.satellites = lambdaCapableSatelliteGeometry();
    opt.num_epochs = 20;
    const auto problem = makeCpHoldFixedLagProblem(opt);

    // Sanity control: without M2 this fixture produces FIXED epochs and pins.
    FGOProcessor::FGOConfig off_config = makeFixDemoteBaseConfig();
    FGOProcessor off_processor(off_config);
    const auto off_result = off_processor.optimizeProblem(problem);
    std::size_t off_fixed = 0;
    for (const auto& sol : off_result.solution.solutions) {
        if (sol.status == SolutionStatus::FIXED) ++off_fixed;
    }
    ASSERT_GT(off_fixed, 0u);
    ASSERT_GT(off_result.diagnostics.ambiguity_hold_arcs, 0u);

    FGOProcessor::FGOConfig on_config = makeFixDemoteBaseConfig();
    on_config.use_fix_plausibility_demotion = true;
    on_config.fix_demote_distance_m = 1e-9;
    FGOProcessor on_processor(on_config);
    const auto on_result = on_processor.optimizeProblem(problem);

    EXPECT_GT(on_result.diagnostics.fix_plausibility_demotions, 0u);
    EXPECT_GT(on_result.diagnostics.fix_plausibility_hold_skips, 0u)
        << "the hold-skip half must fire: validated integers on an implausible "
           "epoch are never pinned";
    EXPECT_EQ(on_result.diagnostics.ambiguity_hold_arcs, 0u)
        << "no arc may ever be pinned when every epoch fails the plausibility check";
    for (const auto& sol : on_result.solution.solutions) {
        EXPECT_NE(sol.status, SolutionStatus::FIXED);
    }
}

// ============================================================================
// Surplus-satellite independent integrity validation
// (FGOConfig::use_surplus_satellite_validation). Reuses the stale-pin
// fixture (lambdaCapableSatelliteGeometry + fix-and-hold): it has no FDE/CMC
// exclusions, so every DD-carrier arc that reaches LAMBDA also gets fixed --
// there is never a satellite EXCLUDED from the fixed subset to serve as a
// surplus candidate. That makes it the right "is this genuinely a no-op
// when there's nothing to validate against" fixture: the feature must
// gracefully report insufficient-surplus (never rescue, never crash, never
// change the FIXED/held outcome) rather than assume exclusions exist.
// ============================================================================

TEST(FGOSurplusValidationTest, DefaultOffIsNoOp) {
    const auto problem = makeStalePinProblem();

    FGOProcessor::FGOConfig config = makeStalePinBaseConfig();
    ASSERT_FALSE(config.use_surplus_satellite_validation);

    FGOProcessor processor(config);
    const auto result = processor.optimizeProblem(problem);

    EXPECT_EQ(result.diagnostics.surplus_validation_attempts, 0u);
    EXPECT_EQ(result.diagnostics.surplus_validation_passes, 0u);
    EXPECT_EQ(result.diagnostics.surplus_validation_fails, 0u);
    EXPECT_EQ(result.diagnostics.surplus_validation_insufficient_surplus, 0u);
    EXPECT_EQ(result.diagnostics.surplus_validation_rescued_epochs, 0u);
    EXPECT_EQ(result.diagnostics.surplus_validation_separation_rejects, 0u);
    EXPECT_EQ(result.diagnostics.surplus_validation_quality_rejects, 0u);
    EXPECT_EQ(result.diagnostics.surplus_validation_vetoed_epochs, 0u);
    for (auto count : result.diagnostics.surplus_validation_fallback_level_histogram) {
        EXPECT_EQ(count, 0u);
    }
}

TEST(FGOSurplusValidationTest, EnabledWithoutExclusionsReportsInsufficientSurplusAndStaysInert) {
    const auto problem = makeStalePinProblem();

    FGOProcessor::FGOConfig off_config = makeStalePinBaseConfig();
    FGOProcessor off_processor(off_config);
    const auto off_result = off_processor.optimizeProblem(problem);
    ASSERT_GT(off_result.diagnostics.ambiguity_hold_arcs, 0u)
        << "sanity: this fixture must actually pin arcs for the comparison below "
           "to mean anything (mirrors FGOStalePinTest.DefaultOffIsNoOp's sanity check)";

    FGOProcessor::FGOConfig on_config = makeStalePinBaseConfig();
    on_config.use_surplus_satellite_validation = true;
    on_config.surplus_validation_min_surplus_satellites = 2;
    FGOProcessor on_processor(on_config);
    const auto on_result = on_processor.optimizeProblem(problem);

    // No FDE/CMC in this fixture -> every arc reaching LAMBDA also gets
    // fixed -> zero surplus candidates at every fallback level -> every
    // evaluation is "insufficient surplus", never a rendered pass/fail.
    EXPECT_EQ(on_result.diagnostics.surplus_validation_attempts, 0u);
    EXPECT_EQ(on_result.diagnostics.surplus_validation_passes, 0u);
    EXPECT_EQ(on_result.diagnostics.surplus_validation_fails, 0u);
    EXPECT_GT(on_result.diagnostics.surplus_validation_insufficient_surplus, 0u);
    EXPECT_EQ(on_result.diagnostics.surplus_validation_rescued_epochs, 0u);
    EXPECT_EQ(on_result.diagnostics.surplus_validation_separation_rejects, 0u);
    EXPECT_EQ(on_result.diagnostics.surplus_validation_quality_rejects, 0u);
    EXPECT_EQ(on_result.diagnostics.surplus_validation_vetoed_epochs, 0u);

    // With nothing to rescue or veto, the held-arc outcome must be
    // byte-identical to the feature-off run.
    EXPECT_EQ(on_result.diagnostics.ambiguity_hold_arcs, off_result.diagnostics.ambiguity_hold_arcs);
    EXPECT_EQ(on_result.diagnostics.ambiguity_hold_epochs, off_result.diagnostics.ambiguity_hold_epochs);
    for (const auto& sol : on_result.solution.solutions) {
        EXPECT_TRUE(sol.position_ecef.allFinite());
    }
}

#endif  // GNSSPP_HAS_GTSAM
