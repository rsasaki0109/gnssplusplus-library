#ifdef GNSSPP_HAS_GTSAM

#include <gtest/gtest.h>
#include <gtsam/navigation/ImuFactor.h>
#include <libgnss++/algorithms/native_imu_bias_density.hpp>
#include <libgnss++/algorithms/fgo.hpp>
#include <libgnss++/algorithms/doppler_contract.hpp>
#include <libgnss++/algorithms/native_raw_p_ecef_doppler_staging.hpp>
#include <libgnss++/algorithms/native_utc_fallback_imu_noise.hpp>
#include <libgnss++/algorithms/source_clock_c0d_initializer.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/io/imu.hpp>

#include "../src/algorithms/fgo_gtsam_internal.hpp"
#include "../src/algorithms/fgo_gtsam_covariance_internal.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <map>
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

FGOProcessor::FGOProblem makePhase164RawNoDopplerProblem(
    bool moving = true, std::size_t satellite_count = 6,
    bool mixed_system = false,
    Vector3d requested_velocity = Vector3d(2.0, -1.5, 0.75)) {
    FGOProcessor::FGOProblem problem;
    const auto satellites = gtsamParitySatelliteGeometry();
    const Vector3d receiver(1113194.0, -4841695.0, 3985350.0);
    const Vector3d velocity = requested_velocity;
    const GNSSTime t0(2300, 300000.0);
    for (std::size_t epoch = 0; epoch < 2; ++epoch) {
        Vector3d position = receiver;
        if (moving) position += static_cast<double>(epoch) * velocity;
        const double clock_bias_m = 5.0 + 0.2 * epoch;
        FGOProcessor::EpochSeed epoch_seed;
        epoch_seed.time = t0 + static_cast<double>(epoch);
        epoch_seed.position_ecef = position;
        epoch_seed.receiver_clock_bias_m = clock_bias_m;
        epoch_seed.receiver_clock_bias_is_meters = true;
        epoch_seed.receiver_clock_drift_mps = 0.2;
        epoch_seed.raw_source_index = epoch;
        epoch_seed.raw_utc_time_millis = 1630000000000LL +
                                         static_cast<std::int64_t>(epoch) * 1000LL;
        problem.epochs.push_back(epoch_seed);

        raw_p_seed::RawPNoDopplerSeed raw_seed;
        raw_seed.epoch_index = epoch;
        raw_seed.time = epoch_seed.time;
        raw_seed.raw_source_index = epoch_seed.raw_source_index;
        raw_seed.raw_utc_time_millis = epoch_seed.raw_utc_time_millis;
        raw_seed.status = raw_p_seed::SeedAdapterStatus::Accepted;
        raw_seed.position_ecef = position;
        raw_seed.velocity_ecef_mps = velocity;
        raw_seed.clock_bias_m = clock_bias_m;
        raw_seed.clock_rate_mps = epoch_seed.receiver_clock_drift_mps;
        raw_seed.reference_clock_group = GNSSSystem::GPS;
        raw_seed.clock_bias_components_m[0] = clock_bias_m;
        raw_seed.clock_bias_component_available[0] = true;
        raw_seed.has_position = true;
        raw_seed.has_velocity = true;
        raw_seed.has_clock = true;
        raw_seed.has_clock_rate = true;
        raw_seed.c7_clock_mapping_supported = true;
        problem.native_raw_p_no_doppler_seeds.push_back(raw_seed);

        const std::array<GNSSSystem, 12> mixed_systems = {
            GNSSSystem::GPS,     GNSSSystem::GPS,     GNSSSystem::GLONASS,
            GNSSSystem::GLONASS, GNSSSystem::Galileo, GNSSSystem::Galileo,
            GNSSSystem::BeiDou,  GNSSSystem::BeiDou,  GNSSSystem::GPS,
            GNSSSystem::GPS,     GNSSSystem::Galileo, GNSSSystem::BeiDou};
        const std::array<SignalType, 12> mixed_signals = {
            SignalType::GPS_L1CA, SignalType::GPS_L1CA,
            SignalType::GLO_L1CA, SignalType::GLO_L1CA,
            SignalType::GAL_E1,   SignalType::GAL_E1,
            SignalType::BDS_B1I,  SignalType::BDS_B1I,
            SignalType::GPS_L5,   SignalType::GPS_L5,
            SignalType::GAL_E5A,  SignalType::BDS_B1I};
        const std::size_t row_count = mixed_system
                                           ? std::min<std::size_t>(
                                                 satellite_count, 12U)
                                           : std::min(satellite_count,
                                                       satellites.size());
        for (std::size_t sat = 0; sat < row_count; ++sat) {
            FGOProcessor::PseudorangeFactor pseudorange;
            pseudorange.epoch_index = epoch;
            const GNSSSystem system = mixed_system ? mixed_systems[sat]
                                                   : GNSSSystem::GPS;
            pseudorange.satellite = SatelliteId(
                system, static_cast<uint8_t>(sat + 1));
            pseudorange.signal = mixed_system ? mixed_signals[sat]
                                              : SignalType::GPS_L1CA;
            pseudorange.clock_group = system;
            pseudorange.satellite_position_ecef = satellites[sat % satellites.size()];
            pseudorange.corrected_pseudorange_m =
                (pseudorange.satellite_position_ecef - position).norm() +
                clock_bias_m;
            pseudorange.sigma_m = 1.0;
            pseudorange.snr_dbhz = 40.0;
            problem.pseudorange_factors.push_back(pseudorange);
        }
    }
    return problem;
}

FGOProcessor::FGOConfig makePhase164RawNoDopplerConfig() {
    FGOProcessor::FGOConfig config;
    config.backend = FGOBackend::GTSAM;
    config.use_pose3_state = false;
    config.use_imu = false;
    config.use_velocity_states = true;
    config.use_motion_factors = true;
    config.use_position_motion_factors = false;
    config.use_velocity_motion_factors = true;
    config.use_clock_motion_factors = true;
    config.use_native_source_clock_c0d_factor = true;
    config.native_source_clock_c0d_phone = "pixel5";
    config.use_native_source_clock_c0d_meter_state_parity = true;
    config.use_native_source_clock_c0d_epoch_vector_parity = true;
    config.use_native_raw_p_no_doppler_graph = true;
    config.use_inter_system_biases = false;
    config.use_receiver_signal_bias_states = false;
    config.use_residual_ionosphere_states = false;
    config.use_tdcp_factors = false;
    config.use_carrier_phase_factors = false;
    config.use_double_difference_factors = false;
    config.use_robust_loss = false;
    config.position_prior_sigma_m = 0.0;
    config.clock_prior_sigma_m = 0.0;
    config.velocity_motion_sigma_m = 0.1;
    config.clock_motion_sigma_m = 0.1;
    config.max_iterations = 20;
    return config;
}

FGOProcessor::FGOConfig makePhase171EcefDopplerConfig() {
    auto config = makePhase164RawNoDopplerConfig();
    config.use_native_raw_p_no_doppler_graph = false;
    config.use_native_raw_p_ecef_doppler_gnss_first = true;
    config.use_undifferenced_doppler_factors = true;
    config.use_corrected_undifferenced_doppler_factors = true;
    config.use_upstream_observable_quality = true;
    config.upstream_snr_percentile = 85.0;
    config.upstream_min_snr_dbhz = 20.0;
    config.upstream_min_elevation_deg = 5.0;
    config.upstream_max_adjacent_gap_s = 1.5;
    return config;
}

void appendPhase171SyntheticEcefDopplerRows(
    FGOProcessor::FGOProblem& problem) {
    const auto satellites = gtsamParitySatelliteGeometry();
    const Vector3d satellite_velocity(1200.0, -900.0, 800.0);
    for (std::size_t epoch = 0; epoch < problem.epochs.size(); ++epoch) {
        const Vector3d receiver = problem.epochs[epoch].position_ecef;
        const Vector3d receiver_velocity =
            problem.native_raw_p_no_doppler_seeds[epoch].velocity_ecef_mps;
        const double receiver_clock_drift =
            problem.native_raw_p_no_doppler_seeds[epoch].clock_rate_mps;
        for (std::size_t sat = 0; sat < satellites.size(); ++sat) {
            Vector3d corrected_position;
            Vector3d corrected_velocity;
            ASSERT_TRUE(doppler_contract::earthRotationCorrectedSatelliteState(
                satellites[sat], satellite_velocity, receiver,
                corrected_position, corrected_velocity));
            Vector3d receiver_to_satellite;
            double satellite_range_rate = 0.0;
            ASSERT_TRUE(doppler_contract::knownSatelliteRangeRate(
                satellites[sat], satellite_velocity, receiver, true,
                receiver_to_satellite, satellite_range_rate));
            const double measured_range_rate =
                satellite_range_rate +
                doppler_contract::receiverPrediction(
                    receiver_to_satellite, receiver_velocity,
                    receiver_clock_drift);
            FGOProcessor::UndifferencedDopplerFactor factor;
            factor.epoch_index = epoch;
            factor.satellite = SatelliteId(
                GNSSSystem::GPS, static_cast<uint8_t>(sat + 1));
            factor.signal = SignalType::GPS_L1CA;
            factor.los = -receiver_to_satellite;
            factor.residual_mps = doppler_contract::receiverOnlyResidual(
                measured_range_rate, satellite_range_rate, 0.0);
            factor.sigma_mps = 1.0;
            factor.elevation_rad = 0.8;
            factor.satellite_position_ecef = corrected_position;
            factor.satellite_velocity_ecef = corrected_velocity;
            factor.source_satellite_position_ecef = satellites[sat];
            factor.source_satellite_velocity_ecef = satellite_velocity;
            factor.source_satellite_state_available = true;
            factor.wavelength_m = 0.1902936728;
            factor.measured_range_rate_mps = measured_range_rate;
            factor.satellite_range_rate_mps = satellite_range_rate;
            factor.satellite_clock_drift_mps = 0.0;
            factor.includes_receiver_clock_drift = true;
            factor.uses_rotated_satellite_state = true;
            problem.undifferenced_doppler_factors.push_back(factor);
        }
    }
}

FGOProcessor::FGOProblem makePhase93GnssFirstClockStateProblem() {
    FGOProcessor::FGOProblem problem;
    const auto satellites = gtsamParitySatelliteGeometry();
    const Vector3d receiver(1113194.0, -4841695.0, 3985350.0);
    const GNSSTime t0(2300, 100000.0);
    for (std::size_t epoch = 0; epoch < 2; ++epoch) {
        FGOProcessor::EpochSeed seed;
        seed.time = t0 + static_cast<double>(epoch);
        seed.position_ecef = receiver;
        seed.receiver_clock_bias_m = 0.0;
        seed.receiver_clock_bias_is_meters = true;
        seed.receiver_clock_drift_mps = 0.25 + 0.10 * epoch;
        seed.raw_source_index = epoch;
        seed.raw_utc_time_millis = 1610000000000LL +
                                   static_cast<std::int64_t>(epoch) * 1000LL;
        problem.epochs.push_back(seed);
        for (std::size_t sat = 0; sat < satellites.size(); ++sat) {
            FGOProcessor::PseudorangeFactor pseudorange;
            pseudorange.epoch_index = epoch;
            pseudorange.satellite = SatelliteId(
                GNSSSystem::GPS, static_cast<uint8_t>(sat + 1));
            pseudorange.signal = SignalType::GPS_L1CA;
            pseudorange.clock_group = GNSSSystem::GPS;
            pseudorange.satellite_position_ecef = satellites[sat];
            pseudorange.corrected_pseudorange_m =
                (satellites[sat] - receiver).norm();
            pseudorange.sigma_m = 1.0;
            pseudorange.snr_dbhz = 35.0;
            problem.pseudorange_factors.push_back(pseudorange);

            FGOProcessor::UndifferencedDopplerFactor doppler;
            doppler.epoch_index = epoch;
            doppler.satellite = pseudorange.satellite;
            doppler.signal = SignalType::GPS_L1CA;
            doppler.los = (satellites[sat] - receiver).normalized();
            doppler.residual_mps = seed.receiver_clock_drift_mps;
            doppler.sigma_mps = 1.0;
            doppler.elevation_rad = 0.7;
            doppler.includes_receiver_clock_drift = true;
            problem.undifferenced_doppler_factors.push_back(doppler);
        }
    }
    return problem;
}

class ConstantResidualFactor
    : public gtsam::NoiseModelFactorN<double> {
 public:
    using Base = gtsam::NoiseModelFactorN<double>;
    using Base::evaluateError;

    ConstantResidualFactor(gtsam::Key key,
                           const gtsam::SharedNoiseModel& model)
        : Base(model, key) {}

    gtsam::Vector evaluateError(
        const double&, gtsam::OptionalMatrixType H) const override {
        if (H) *H = gtsam::Matrix::Zero(1, 1);
        return (gtsam::Vector(1) << 100.0).finished();
    }
};

// The pinned LM optimizer catches this exception inside tryLambda when it is
// thrown by the linear solver.  This synthetic factor throws during the next
// linearization instead, so the unchanged integration-boundary catch can be
// exercised without a dataset or a second solver invocation.
class ThrowingIndeterminateFactor
    : public gtsam::NoiseModelFactorN<double> {
 public:
    using Base = gtsam::NoiseModelFactorN<double>;
    using Base::evaluateError;

    ThrowingIndeterminateFactor(gtsam::Key key,
                                const gtsam::SharedNoiseModel& model)
        : Base(model, key), key_(key) {}

    gtsam::Vector evaluateError(
        const double&, gtsam::OptionalMatrixType H) const override {
        ++calls_;
        if (calls_ >= 2) {
            throw gtsam::IndeterminantLinearSystemException(key_);
        }
        if (H) *H = gtsam::Matrix::Zero(1, 1);
        return (gtsam::Vector(1) << 1.0).finished();
    }

 private:
    gtsam::Key key_;
    mutable std::size_t calls_ = 0;
};

// Two nearly collinear rows provide a finite, deliberately rank-challenged
// synthetic system for the Phase99 solver selector.  The graph is full rank
// (the tiny coefficient separation is intentional), so QR can take a strict
// cost-decreasing candidate step without changing the production graph.
class NearRankCoupledFactor
    : public gtsam::NoiseModelFactorN<double, double> {
 public:
    using Base = gtsam::NoiseModelFactorN<double, double>;
    using Base::evaluateError;

    NearRankCoupledFactor(gtsam::Key x_key, gtsam::Key y_key,
                          double y_coefficient, double target,
                          const gtsam::SharedNoiseModel& model)
        : Base(model, x_key, y_key), y_coefficient_(y_coefficient),
          target_(target) {}

    gtsam::Vector evaluateError(
        const double& x, const double& y, gtsam::OptionalMatrixType Hx,
        gtsam::OptionalMatrixType Hy) const override {
        if (Hx) *Hx = (gtsam::Matrix(1, 1) << 1.0).finished();
        if (Hy) *Hy = (gtsam::Matrix(1, 1) << y_coefficient_).finished();
        return (gtsam::Vector(1) << x + y_coefficient_ * y - target_)
            .finished();
    }

 private:
    double y_coefficient_;
    double target_;
};

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
    EXPECT_FALSE(gtsam_result.diagnostics
                     .native_source_clock_c0d_phase99_main_multifrontal_qr_solver_requested);
    EXPECT_FALSE(gtsam_result.diagnostics
                     .native_source_clock_c0d_phase99_main_multifrontal_qr_solver_selected);
    EXPECT_EQ(gtsam_result.diagnostics.selected_linear_solver_type,
              "MULTIFRONTAL_CHOLESKY");
    EXPECT_EQ(gtsam_result.diagnostics.selected_solver_branch, "multifrontal");
    EXPECT_EQ(gtsam_result.diagnostics.selected_elimination_function,
              "EliminatePreferCholesky");
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

TEST(FgoDdprGncShadowTest, ReportsWeightsWithoutChangingEstimatorOutput) {
    CpHoldTestOptions opt;
    opt.num_epochs = 3;
    const auto problem = makeCpHoldFixedLagProblem(opt);

    FGOProcessor::FGOConfig baseline_config = makeCpHoldBaseConfig();
    const auto baseline = FGOProcessor(baseline_config).optimizeProblem(problem);

    auto shadow_config = baseline_config;
    shadow_config.monitor_ddpr_gnc = true;
    const auto shadow = FGOProcessor(shadow_config).optimizeProblem(problem);

    ASSERT_EQ(baseline.solution.solutions.size(), shadow.solution.solutions.size());
    ASSERT_EQ(shadow.epoch_diagnostics.size(), problem.epochs.size());
    for (std::size_t i = 0; i < shadow.solution.solutions.size(); ++i) {
        EXPECT_EQ(baseline.solution.solutions[i].status,
                  shadow.solution.solutions[i].status);
        EXPECT_DOUBLE_EQ(baseline.solution.solutions[i].ratio,
                         shadow.solution.solutions[i].ratio);
        EXPECT_TRUE(baseline.solution.solutions[i].position_ecef.isApprox(
            shadow.solution.solutions[i].position_ecef, 0.0));

        const auto& diagnostic = shadow.epoch_diagnostics[i];
        EXPECT_TRUE(diagnostic.ddpr_gnc_evaluated);
        EXPECT_EQ(diagnostic.ddpr_gnc_factor_count, 5);
        EXPECT_GT(diagnostic.ddpr_gnc_stages, 0);
        EXPECT_GT(diagnostic.ddpr_gnc_min_weight, 0.0);
        EXPECT_LE(diagnostic.ddpr_gnc_min_weight, 1.0);
        EXPECT_GT(diagnostic.ddpr_gnc_effective_factor_count, 0.0);
        EXPECT_LE(diagnostic.ddpr_gnc_effective_factor_count,
                  diagnostic.ddpr_gnc_factor_count);
        ASSERT_EQ(diagnostic.ddpr_gnc_factor_trace.size(), 5u);
        for (const auto& trace : diagnostic.ddpr_gnc_factor_trace) {
            EXPECT_GT(trace.sigma_m, 0.0);
            EXPECT_GE(trace.normalized_residual, 0.0);
            EXPECT_GT(trace.weight, 0.0);
            EXPECT_LE(trace.weight, 1.0);
        }
    }
    EXPECT_EQ(shadow.diagnostics.ddpr_gnc_evaluated_epochs,
              problem.epochs.size());
    EXPECT_EQ(shadow.diagnostics.ddpr_gnc_factors,
              problem.epochs.size() * 5);
}

TEST(FgoDdprGncCounterfactualTest,
     AlternatingSolveRejectsOutlierWithoutChangingEstimatorOutput) {
    CpHoldTestOptions opt;
    opt.num_epochs = 3;
    opt.pr_corrupt_epochs = {2};
    opt.pr_dominant_extra_bias_m = 30.0;
    const auto problem = makeCpHoldFixedLagProblem(opt);

    auto baseline_config = makeCpHoldBaseConfig();
    const auto baseline = FGOProcessor(baseline_config).optimizeProblem(problem);

    auto shadow_config = baseline_config;
    shadow_config.monitor_ddpr_gnc_counterfactual = true;
    const auto shadow = FGOProcessor(shadow_config).optimizeProblem(problem);

    ASSERT_EQ(baseline.solution.solutions.size(), shadow.solution.solutions.size());
    ASSERT_EQ(shadow.epoch_diagnostics.size(), problem.epochs.size());
    for (std::size_t i = 0; i < shadow.solution.solutions.size(); ++i) {
        EXPECT_EQ(baseline.solution.solutions[i].status,
                  shadow.solution.solutions[i].status);
        EXPECT_DOUBLE_EQ(baseline.solution.solutions[i].ratio,
                         shadow.solution.solutions[i].ratio);
        EXPECT_TRUE(baseline.solution.solutions[i].position_ecef.isApprox(
            shadow.solution.solutions[i].position_ecef, 0.0));
    }

    const auto& diagnostic = shadow.epoch_diagnostics.back();
    EXPECT_TRUE(diagnostic.ddpr_gnc_counterfactual_evaluated);
    EXPECT_TRUE(diagnostic.ddpr_gnc_counterfactual_succeeded);
    EXPECT_EQ(diagnostic.ddpr_gnc_counterfactual_factor_count, 15);
    EXPECT_EQ(diagnostic.ddpr_gnc_counterfactual_stages,
              shadow_config.ddpr_gnc_counterfactual_max_stages);
    EXPECT_TRUE(diagnostic.ddpr_gnc_counterfactual_position_ecef.allFinite());
    EXPECT_GT(diagnostic.ddpr_gnc_counterfactual_float_separation_m, 0.0);
    EXPECT_NEAR(
        diagnostic.ddpr_gnc_counterfactual_float_separation_m,
        (diagnostic.ddpr_gnc_counterfactual_position_ecef -
         shadow.solution.solutions.back().position_ecef).norm(),
        1e-9);
    EXPECT_TRUE(diagnostic.ddpr_gnc_counterfactual_lambda_evaluated);
    EXPECT_EQ(diagnostic.ddpr_gnc_counterfactual_lambda_ambiguities, 5);
    EXPECT_GT(diagnostic.ddpr_gnc_counterfactual_lambda_ratio, 0.0);
    EXPECT_FALSE(diagnostic.ddpr_gnc_counterfactual_lambda_ratio_pass);
    const Vector3d truth(1113194.0, -4841695.0, 3985350.0);
    EXPECT_LT((diagnostic.ddpr_gnc_counterfactual_position_ecef - truth).norm(),
              (baseline.solution.solutions.back().position_ecef - truth).norm());
    EXPECT_EQ(shadow.diagnostics.ddpr_gnc_counterfactual_attempts,
              problem.epochs.size());
    EXPECT_EQ(shadow.diagnostics.ddpr_gnc_counterfactual_successes,
              problem.epochs.size());
}

TEST(FGODisjointConstellationArShadowTest,
     ProducesTwoCandidatesWithoutChangingFixedLagSolution) {
    CpHoldTestOptions opt;
    opt.num_epochs = 3;
    opt.satellites = gtsamParitySatelliteGeometry();
    opt.satellites.push_back(Vector3d(-21500000.0, 9000000.0, 11000000.0));
    opt.satellites.push_back(Vector3d(9000000.0, 22000000.0, -12000000.0));
    opt.satellites.push_back(Vector3d(-8000000.0, -21000000.0, -14000000.0));
    auto problem = makeCpHoldFixedLagProblem(opt);

    for (std::size_t index = 4; index < problem.ambiguity_states.size(); ++index) {
        auto& ambiguity = problem.ambiguity_states[index];
        ambiguity.satellite = SatelliteId(
            GNSSSystem::Galileo, ambiguity.satellite.prn);
        ambiguity.reference_satellite = SatelliteId(GNSSSystem::Galileo, 1);
    }
    for (auto& factor : problem.double_difference_carrier_factors) {
        if (factor.ambiguity_index < 4) continue;
        factor.satellite = SatelliteId(GNSSSystem::Galileo, factor.satellite.prn);
        factor.reference_satellite = SatelliteId(GNSSSystem::Galileo, 1);
    }

    FGOProcessor::FGOConfig base_config = makeCpHoldBaseConfig();
    base_config.use_carrier_phase_factors = true;
    base_config.use_lambda_ambiguity_fix = true;
    base_config.min_fixed_ambiguities = 4;
    const auto baseline = FGOProcessor(base_config).optimizeProblem(problem);

    auto shadow_config = base_config;
    shadow_config.monitor_disjoint_constellation_ar = true;
    shadow_config.disjoint_constellation_ar_min_ambiguities = 4;
    const auto shadow = FGOProcessor(shadow_config).optimizeProblem(problem);

    ASSERT_EQ(baseline.solution.solutions.size(), shadow.solution.solutions.size());
    ASSERT_EQ(shadow.epoch_diagnostics.size(), problem.epochs.size());
    for (std::size_t epoch = 0; epoch < shadow.solution.solutions.size(); ++epoch) {
        EXPECT_EQ(baseline.solution.solutions[epoch].status,
                  shadow.solution.solutions[epoch].status);
        EXPECT_DOUBLE_EQ(baseline.solution.solutions[epoch].ratio,
                         shadow.solution.solutions[epoch].ratio);
        EXPECT_EQ(baseline.solution.solutions[epoch].num_fixed_ambiguities,
                  shadow.solution.solutions[epoch].num_fixed_ambiguities);
        EXPECT_TRUE(baseline.solution.solutions[epoch].position_ecef.isApprox(
            shadow.solution.solutions[epoch].position_ecef, 0.0));

        const auto& diagnostic =
            shadow.epoch_diagnostics[epoch].disjoint_constellation_ar_shadow;
        EXPECT_TRUE(diagnostic.evaluated);
        EXPECT_EQ(diagnostic.partition_a_ambiguities, 4);
        EXPECT_EQ(diagnostic.partition_b_ambiguities, 4);
        EXPECT_EQ(diagnostic.partition_a_system_mask &
                      diagnostic.partition_b_system_mask,
                  0u);
        EXPECT_TRUE(diagnostic.partition_a_candidate_available);
        EXPECT_TRUE(diagnostic.partition_b_candidate_available);
        EXPECT_TRUE(std::isfinite(diagnostic.partition_separation_m));
    }
}

TEST(FGOClockResilientTemporalCarrierShadowTest,
     ReportsResidualsWithoutChangingFixedLagSolution) {
    CpHoldTestOptions opt;
    opt.num_epochs = 3;
    auto problem = makeCpHoldFixedLagProblem(opt);

    using ObservationKey = std::pair<std::size_t, SatelliteId>;
    std::map<ObservationKey, FGOProcessor::CarrierPhaseFactor> observations;
    for (const auto& factor : problem.double_difference_carrier_factors) {
        const auto add_observation =
            [&](const SatelliteId satellite, const Vector3d& satellite_position,
                const FGOProcessor::ObservationModelDebug& model) {
                FGOProcessor::CarrierPhaseFactor observation;
                observation.epoch_index = factor.epoch_index;
                observation.satellite = satellite;
                observation.signal = factor.signal;
                observation.satellite_position_ecef = satellite_position;
                observation.corrected_carrier_m = model.corrected_carrier_m;
                observation.has_carrier_phase = true;
                observation.loss_of_lock = false;
                const Vector3d range_vector =
                    satellite_position -
                    problem.epochs[factor.epoch_index].position_ecef;
                observation.los = range_vector.normalized();
                observation.elevation_rad = factor.elevation_rad;
                observation.model_debug.geometric_range_m =
                    range_vector.norm();
                observations[{factor.epoch_index, satellite}] = observation;
            };
        add_observation(factor.satellite,
                        factor.rover_satellite_position_ecef,
                        factor.rover_satellite_model);
        add_observation(factor.reference_satellite,
                        factor.rover_reference_position_ecef,
                        factor.rover_reference_model);
    }
    for (const auto& [key, observation] : observations) {
        (void)key;
        problem.carrier_observations.push_back(observation);
    }

    FGOProcessor::FGOConfig off_config = makeCpHoldBaseConfig();
    FGOProcessor off_processor(off_config);
    const auto off_result = off_processor.optimizeProblem(problem);

    FGOProcessor::FGOConfig on_config = off_config;
    on_config.monitor_clock_resilient_temporal_carrier = true;
    FGOProcessor on_processor(on_config);
    const auto on_result = on_processor.optimizeProblem(problem);

    ASSERT_EQ(off_result.solution.solutions.size(),
              on_result.solution.solutions.size());
    ASSERT_EQ(on_result.epoch_diagnostics.size(), problem.epochs.size());
    int total_shadow_factors = 0;
    for (std::size_t i = 0; i < on_result.solution.solutions.size(); ++i) {
        EXPECT_TRUE(off_result.solution.solutions[i].position_ecef.isApprox(
            on_result.solution.solutions[i].position_ecef, 0.0));
        EXPECT_EQ(off_result.solution.solutions[i].status,
                  on_result.solution.solutions[i].status);
        total_shadow_factors +=
            on_result.epoch_diagnostics[i].clock_resilient_tdcp_factors;
        EXPECT_TRUE(std::isfinite(
            on_result.epoch_diagnostics[i].clock_resilient_tdcp_rms_m));
        EXPECT_TRUE(std::isfinite(
            on_result.epoch_diagnostics[i].clock_resilient_tdcp_max_abs_m));
    }
    EXPECT_GT(total_shadow_factors, 0);
    ASSERT_EQ(on_result.temporal_carrier_shadow_factors.size(),
              static_cast<std::size_t>(total_shadow_factors));
    std::vector<Vector3d> replay_positions;
    replay_positions.reserve(on_result.solution.solutions.size());
    for (const auto& solution : on_result.solution.solutions) {
        replay_positions.push_back(solution.position_ecef);
    }
    auto replay_epoch_diagnostics = on_result.epoch_diagnostics;
    for (auto& diagnostics : replay_epoch_diagnostics) {
        diagnostics.clock_resilient_tdcp_factors = 0;
        diagnostics.clock_resilient_tdcp_rms_m = 0.0;
        diagnostics.clock_resilient_tdcp_max_abs_m = 0.0;
        diagnostics.clock_resilient_tdcp_clean = 0;
        diagnostics.clock_resilient_tdcp_witnessed_outliers = 0;
        diagnostics.clock_resilient_tdcp_unexplained_outliers = 0;
    }
    const auto replay_shadow =
        FGOProcessor::buildClockResilientTemporalCarrierShadow(problem);
    const auto replay_geometry_free =
        FGOProcessor::analyzeGeometryFreeSlipShadow(problem);
    const auto replay_rows =
        FGOProcessor::classifyClockResilientTemporalCarrierShadow(
            problem, replay_shadow, replay_positions,
            replay_epoch_diagnostics, &replay_geometry_free);
    ASSERT_EQ(replay_rows.size(),
              on_result.temporal_carrier_shadow_factors.size());
    for (std::size_t i = 0; i < replay_rows.size(); ++i) {
        EXPECT_DOUBLE_EQ(replay_rows[i].residual_m,
                         on_result.temporal_carrier_shadow_factors[i].residual_m);
        EXPECT_DOUBLE_EQ(
            replay_rows[i].normalized_doppler_innovation,
            on_result.temporal_carrier_shadow_factors[i]
                .normalized_doppler_innovation);
        EXPECT_EQ(replay_rows[i].classification,
                  on_result.temporal_carrier_shadow_factors[i].classification);
    }
    int classified_total = 0;
    for (const auto& epoch : on_result.epoch_diagnostics) {
        classified_total += epoch.clock_resilient_tdcp_clean;
        classified_total += epoch.clock_resilient_tdcp_witnessed_outliers;
        classified_total += epoch.clock_resilient_tdcp_unexplained_outliers;
    }
    EXPECT_EQ(classified_total, total_shadow_factors);
    for (const auto& row : on_result.temporal_carrier_shadow_factors) {
        EXPECT_TRUE(std::isfinite(row.residual_m));
        EXPECT_TRUE(std::isfinite(row.normalized_residual));
        EXPECT_GE(row.factor.arc_length_epochs, 2);
        if (row.doppler_evaluated) {
            EXPECT_GT(row.doppler_innovation_sigma_m, 0.0);
            EXPECT_TRUE(std::isfinite(row.normalized_doppler_innovation));
        }
        if (row.doppler_calibration_evaluated) {
            EXPECT_GT(row.doppler_calibrated_scale_m, 0.0);
            EXPECT_TRUE(std::isfinite(row.doppler_bias_m));
            EXPECT_TRUE(std::isfinite(row.doppler_centered_innovation_m));
            EXPECT_TRUE(std::isfinite(row.doppler_calibrated_score));
        }
    }

    auto outlier_problem = problem;
    const auto shadow =
        FGOProcessor::buildClockResilientTemporalCarrierShadow(outlier_problem);
    ASSERT_FALSE(shadow.empty());
    const auto& injected = shadow.back();
    bool injected_outlier = false;
    for (auto& observation : outlier_problem.carrier_observations) {
        if (observation.epoch_index == injected.current_epoch_index &&
            observation.satellite == injected.satellite &&
            observation.signal == injected.signal) {
            observation.corrected_carrier_m += 1.0;
            injected_outlier = true;
            break;
        }
    }
    ASSERT_TRUE(injected_outlier);
    const auto outlier_result = on_processor.optimizeProblem(outlier_problem);
    EXPECT_TRUE(std::any_of(
        outlier_result.temporal_carrier_shadow_factors.begin(),
        outlier_result.temporal_carrier_shadow_factors.end(),
        [](const auto& row) {
            return row.classification == FGOProcessor::
                TemporalCarrierShadowClassification::UnexplainedOutlier;
        }));

    auto witnessed_problem = outlier_problem;
    for (const std::size_t epoch_index :
         {injected.previous_epoch_index, injected.current_epoch_index}) {
        FGOProcessor::SingleDifferenceDopplerFactor doppler;
        doppler.epoch_index = epoch_index;
        doppler.satellite = injected.satellite;
        doppler.reference_satellite = injected.reference_satellite;
        doppler.signal = injected.signal;
        doppler.residual_mps = 0.0;
        doppler.sigma_mps = 0.2;
        witnessed_problem.single_difference_doppler_factors.push_back(doppler);
    }
    const auto modeled_doppler_result =
        on_processor.optimizeProblem(witnessed_problem);
    EXPECT_TRUE(std::any_of(
        modeled_doppler_result.temporal_carrier_shadow_factors.begin(),
        modeled_doppler_result.temporal_carrier_shadow_factors.end(),
        [&](const auto& row) {
            return row.factor.current_epoch_index ==
                       injected.current_epoch_index &&
                row.factor.satellite == injected.satellite &&
                row.factor.reference_satellite == injected.reference_satellite &&
                row.factor.signal == injected.signal && row.doppler_evaluated &&
                row.doppler_outlier &&
                row.normalized_doppler_innovation > 5.0 &&
                !row.doppler_calibration_evaluated &&
                !row.doppler_calibrated_outlier &&
                row.classification == FGOProcessor::
                    TemporalCarrierShadowClassification::UnexplainedOutlier;
        }));
}

TEST(FGOPredictedDdprQualityShadowTest,
     ReportsCausalRowsWithoutChangingFixedLagSolution) {
    CpHoldTestOptions opt;
    opt.num_epochs = 3;
    const auto problem = makeCpHoldFixedLagProblem(opt);

    FGOProcessor::FGOConfig off_config = makeCpHoldBaseConfig();
    const auto off_result = FGOProcessor(off_config).optimizeProblem(problem);

    FGOProcessor::FGOConfig on_config = off_config;
    on_config.monitor_predicted_ddpr_quality = true;
    on_config.monitor_predicted_ddpr_bias_state = true;
    const auto on_result = FGOProcessor(on_config).optimizeProblem(problem);

    ASSERT_EQ(off_result.solution.solutions.size(),
              on_result.solution.solutions.size());
    for (std::size_t i = 0; i < off_result.solution.solutions.size(); ++i) {
        EXPECT_TRUE(off_result.solution.solutions[i].position_ecef.isApprox(
            on_result.solution.solutions[i].position_ecef, 0.0));
        EXPECT_EQ(off_result.solution.solutions[i].status,
                  on_result.solution.solutions[i].status);
        EXPECT_EQ(off_result.solution.solutions[i].ratio,
                  on_result.solution.solutions[i].ratio);
        EXPECT_EQ(off_result.solution.solutions[i].num_fixed_ambiguities,
                  on_result.solution.solutions[i].num_fixed_ambiguities);
    }
    ASSERT_FALSE(on_result.predicted_ddpr_quality_factors.empty());
    bool saw_imu_geometry = false;
    for (const auto& row : on_result.predicted_ddpr_quality_factors) {
        EXPECT_GE(row.pair_age_epochs, 2);
        EXPECT_GT(row.dt_s, 0.0);
        EXPECT_TRUE(std::isfinite(row.measured_ddpr_change_m));
        if (row.imu_geometry_evaluated) {
            saw_imu_geometry = true;
            EXPECT_GT(row.imu_innovation_sigma_m, 0.0);
            EXPECT_TRUE(std::isfinite(row.normalized_imu_innovation));
        }
    }
    EXPECT_TRUE(saw_imu_geometry);
    ASSERT_FALSE(on_result.predicted_ddpr_bias_state_factors.empty());
    EXPECT_EQ(on_result.predicted_ddpr_bias_state_factors.size(),
              on_result.predicted_ddpr_quality_factors.size());
    for (const auto& row :
         on_result.predicted_ddpr_bias_state_factors) {
        EXPECT_TRUE(row.update_applied);
        EXPECT_TRUE(std::isfinite(row.prior_bias_m));
        EXPECT_TRUE(std::isfinite(row.posterior_bias_m));
        EXPECT_GT(row.measurement_sigma_m, 0.0);
    }
}

TEST(FGOSatelliteQuarantineShadowTest,
     AttributesTwoStageOutlierWithoutChangingFixedLagSolution) {
    CpHoldTestOptions opt;
    opt.num_epochs = 3;
    auto problem = makeCpHoldFixedLagProblem(opt);

    for (auto& factor : problem.double_difference_pseudorange_factors) {
        factor.rover_satellite_model.has_doppler_residual = true;
        factor.rover_reference_model.has_doppler_residual = true;
        factor.base_satellite_model.has_doppler_residual = true;
        factor.base_reference_model.has_doppler_residual = true;
        factor.rover_satellite_model.doppler_residual_mps = 0.0;
        factor.rover_reference_model.doppler_residual_mps = 0.0;
        factor.base_satellite_model.doppler_residual_mps = 0.0;
        factor.base_reference_model.doppler_residual_mps = 0.0;
    }

    auto injected = std::find_if(
        problem.double_difference_pseudorange_factors.begin(),
        problem.double_difference_pseudorange_factors.end(),
        [](const auto& factor) {
            return factor.epoch_index == 2 && factor.satellite.prn == 2;
        });
    ASSERT_NE(injected, problem.double_difference_pseudorange_factors.end());
    injected->rover_satellite_model.corrected_pseudorange_m += 30.0;
    injected->observed_dd_pseudorange_m += 30.0;
    const SatelliteId target = injected->satellite;
    const SatelliteId reference = injected->reference_satellite;

    FGOProcessor::FGOConfig off_config = makeCpHoldBaseConfig();
    off_config.use_carrier_phase_factors = true;
    const auto off_result = FGOProcessor(off_config).optimizeProblem(problem);

    FGOProcessor::FGOConfig on_config = off_config;
    on_config.monitor_satellite_quarantine_witness = true;
    const auto on_result = FGOProcessor(on_config).optimizeProblem(problem);

    ASSERT_EQ(off_result.solution.solutions.size(),
              on_result.solution.solutions.size());
    for (std::size_t i = 0; i < off_result.solution.solutions.size(); ++i) {
        EXPECT_TRUE(off_result.solution.solutions[i].position_ecef.isApprox(
            on_result.solution.solutions[i].position_ecef, 0.0));
        EXPECT_EQ(off_result.solution.solutions[i].status,
                  on_result.solution.solutions[i].status);
        EXPECT_EQ(off_result.solution.solutions[i].ratio,
                  on_result.solution.solutions[i].ratio);
        EXPECT_EQ(off_result.solution.solutions[i].num_fixed_ambiguities,
                  on_result.solution.solutions[i].num_fixed_ambiguities);
    }
    EXPECT_EQ(off_result.diagnostics.ambiguity_generation_bumps,
              on_result.diagnostics.ambiguity_generation_bumps);

    const auto isCandidate = [&](const SatelliteId satellite) {
        return std::any_of(
            on_result.satellite_quarantine_witnesses.begin(),
            on_result.satellite_quarantine_witnesses.end(),
            [&](const auto& row) {
                return row.epoch_index == 2 && row.satellite == satellite &&
                    row.postfit_gross && row.doppler_evaluated &&
                    row.doppler_outlier && row.imu_evaluated &&
                    row.imu_outlier && row.temporal_support_pairs > 0 &&
                    row.quarantine_candidate;
            });
    };
    EXPECT_TRUE(isCandidate(target));
    EXPECT_TRUE(isCandidate(reference));
    EXPECT_GE(on_result.diagnostics.satellite_quarantine_candidates, 2u);
}

TEST(FGOSelectiveArcRestartTest,
     OffOnSolutionsAreBitIdenticalWhenNoQuarantineCandidateFires) {
    CpHoldTestOptions opt;
    opt.num_epochs = 3;
    const auto problem = makeCpHoldFixedLagProblem(opt);

    FGOProcessor::FGOConfig off_config = makeCpHoldBaseConfig();
    off_config.use_carrier_phase_factors = true;
    const auto off_result = FGOProcessor(off_config).optimizeProblem(problem);

    FGOProcessor::FGOConfig on_config = off_config;
    on_config.use_selective_arc_restart = true;
    const auto on_result = FGOProcessor(on_config).optimizeProblem(problem);

    ASSERT_EQ(off_result.solution.solutions.size(),
              on_result.solution.solutions.size());
    for (std::size_t i = 0; i < off_result.solution.solutions.size(); ++i) {
        EXPECT_TRUE(off_result.solution.solutions[i].position_ecef.isApprox(
            on_result.solution.solutions[i].position_ecef, 0.0));
        EXPECT_EQ(off_result.solution.solutions[i].status,
                  on_result.solution.solutions[i].status);
        EXPECT_EQ(off_result.solution.solutions[i].ratio,
                  on_result.solution.solutions[i].ratio);
        EXPECT_EQ(off_result.solution.solutions[i].num_fixed_ambiguities,
                  on_result.solution.solutions[i].num_fixed_ambiguities);
    }
    EXPECT_EQ(off_result.diagnostics.ambiguity_generation_bumps,
              on_result.diagnostics.ambiguity_generation_bumps);
    EXPECT_EQ(on_result.diagnostics.selective_arc_restart_applied_arcs, 0u);
    EXPECT_EQ(on_result.diagnostics.selective_arc_restart_detection_epochs, 0u);
}

TEST(FGOSelectiveArcRestartTest,
     SyntheticGrossJumpArmsImplicatedPairAndRestartsOnlyItsArc) {
    CpHoldTestOptions opt;
    opt.num_epochs = 4;
    auto problem = makeCpHoldFixedLagProblem(opt);

    for (auto& factor : problem.double_difference_pseudorange_factors) {
        factor.rover_satellite_model.has_doppler_residual = true;
        factor.rover_reference_model.has_doppler_residual = true;
        factor.base_satellite_model.has_doppler_residual = true;
        factor.base_reference_model.has_doppler_residual = true;
        factor.rover_satellite_model.doppler_residual_mps = 0.0;
        factor.rover_reference_model.doppler_residual_mps = 0.0;
        factor.base_satellite_model.doppler_residual_mps = 0.0;
        factor.base_reference_model.doppler_residual_mps = 0.0;
    }

    auto injected = std::find_if(
        problem.double_difference_pseudorange_factors.begin(),
        problem.double_difference_pseudorange_factors.end(),
        [](const auto& factor) {
            return factor.epoch_index == 2 && factor.satellite.prn == 2;
        });
    ASSERT_NE(injected, problem.double_difference_pseudorange_factors.end());
    injected->rover_satellite_model.corrected_pseudorange_m += 30.0;
    injected->observed_dd_pseudorange_m += 30.0;

    FGOProcessor::FGOConfig off_config = makeCpHoldBaseConfig();
    off_config.use_carrier_phase_factors = true;
    const auto off_result = FGOProcessor(off_config).optimizeProblem(problem);

    FGOProcessor::FGOConfig on_config = off_config;
    on_config.use_selective_arc_restart = true;
    on_config.selective_arc_restart_skip_fixed_ratio = 1000.0;
    on_config.selective_arc_restart_max_arcs_per_epoch = 16;
    const auto on_result = FGOProcessor(on_config).optimizeProblem(problem);

    // Detection fired at epoch 2 and the implicated pair's arc was restarted
    // at epoch 3.
    EXPECT_GE(on_result.diagnostics.selective_arc_restart_detection_epochs, 1u);
    EXPECT_GE(on_result.diagnostics.selective_arc_restart_applied_arcs, 1u);
    EXPECT_EQ(on_result.diagnostics.selective_arc_restart_skipped_fixed, 0u);
    EXPECT_GT(on_result.diagnostics.ambiguity_generation_bumps,
              off_result.diagnostics.ambiguity_generation_bumps);

    // Exactly the ambiguity index whose DD pair was injected was armed.
    const auto& epoch2 = on_result.epoch_diagnostics[2];
    EXPECT_TRUE(epoch2.selective_arc_restart_armed);
    EXPECT_GE(epoch2.selective_arc_restart_candidate_pairs, 1);
    ASSERT_GE(epoch2.selective_arc_restart_trace.size(), 2u);
    const std::size_t armed_index = epoch2.selective_arc_restart_trace[0].ambiguity_index;
    for (const auto& row : epoch2.selective_arc_restart_trace) {
        EXPECT_EQ(row.ambiguity_index, armed_index);
        EXPECT_TRUE(row.detected);
        EXPECT_GT(row.postfit_residual_m, 10.0);
        EXPECT_GT(row.normalized_doppler_innovation, 5.0);
        EXPECT_GT(row.normalized_imu_innovation, 5.0);
    }

    // The apply marked the detection trace rows applied at epoch 3; with the
    // cap raised above the armed-arc count, every armed arc was restarted.
    EXPECT_EQ(on_result.epoch_diagnostics[3].selective_arc_restart_applied_arcs,
              static_cast<int>(epoch2.selective_arc_restart_candidate_pairs));
    EXPECT_GT(on_result.epoch_diagnostics[3].selective_arc_restart_applied_arcs, 0);
    for (const auto& row : on_result.selective_arc_restart_trace) {
        if (row.detection_epoch == 2) {
            EXPECT_TRUE(row.applied);
        }
    }
}

TEST(FGOSelectiveArcRestartTest,
     MonitorOnlyReportsCandidatesWithoutBumpingAnyGeneration) {
    CpHoldTestOptions opt;
    opt.num_epochs = 4;
    auto problem = makeCpHoldFixedLagProblem(opt);

    for (auto& factor : problem.double_difference_pseudorange_factors) {
        factor.rover_satellite_model.has_doppler_residual = true;
        factor.rover_reference_model.has_doppler_residual = true;
        factor.base_satellite_model.has_doppler_residual = true;
        factor.base_reference_model.has_doppler_residual = true;
        factor.rover_satellite_model.doppler_residual_mps = 0.0;
        factor.rover_reference_model.doppler_residual_mps = 0.0;
        factor.base_satellite_model.doppler_residual_mps = 0.0;
        factor.base_reference_model.doppler_residual_mps = 0.0;
    }
    auto injected = std::find_if(
        problem.double_difference_pseudorange_factors.begin(),
        problem.double_difference_pseudorange_factors.end(),
        [](const auto& factor) {
            return factor.epoch_index == 2 && factor.satellite.prn == 2;
        });
    ASSERT_NE(injected, problem.double_difference_pseudorange_factors.end());
    injected->rover_satellite_model.corrected_pseudorange_m += 30.0;
    injected->observed_dd_pseudorange_m += 30.0;

    FGOProcessor::FGOConfig off_config = makeCpHoldBaseConfig();
    off_config.use_carrier_phase_factors = true;
    const auto off_result = FGOProcessor(off_config).optimizeProblem(problem);

    FGOProcessor::FGOConfig monitor_config = off_config;
    monitor_config.monitor_selective_arc_restart_candidates = true;
    const auto monitor_result =
        FGOProcessor(monitor_config).optimizeProblem(problem);

    EXPECT_GE(monitor_result.diagnostics.selective_arc_restart_detection_epochs, 1u);
    EXPECT_EQ(monitor_result.diagnostics.selective_arc_restart_applied_arcs, 0u);
    EXPECT_TRUE(monitor_result.epoch_diagnostics[2].selective_arc_restart_armed);
    EXPECT_TRUE(monitor_result.epoch_diagnostics[2]
                    .selective_arc_restart_monitor_only);
    EXPECT_EQ(monitor_result.diagnostics.ambiguity_generation_bumps,
              off_result.diagnostics.ambiguity_generation_bumps);
    for (std::size_t i = 0; i < off_result.solution.solutions.size(); ++i) {
        EXPECT_TRUE(off_result.solution.solutions[i].position_ecef.isApprox(
            monitor_result.solution.solutions[i].position_ecef, 0.0));
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

TEST(FGOAmbiguityOutcomeTelemetryTest,
     MultiEpochArMonitorIsDiagnosticOnlyAndBuildsPersistentSubset) {
    CpHoldTestOptions opt;
    opt.satellites = lambdaCapableSatelliteGeometry();
    opt.num_epochs = 12;
    auto problem = makeCpHoldFixedLagProblem(opt);
    // Move one complete satellite arc outside the searchable ambiguity set.
    // It remains observable only through the build-time-excluded carrier
    // collection, which exercises the held-out verdict without duplicating a
    // satellite in both the fixed and surplus pools.
    const std::size_t heldout_index = problem.ambiguity_states.size() - 1;
    for (auto it = problem.double_difference_carrier_factors.begin();
         it != problem.double_difference_carrier_factors.end();) {
        if (it->ambiguity_index != heldout_index) {
            ++it;
            continue;
        }
        auto excluded = *it;
        excluded.ambiguity_index = std::numeric_limits<std::size_t>::max();
        problem.excluded_double_difference_carrier_factors.push_back(excluded);
        it = problem.double_difference_carrier_factors.erase(it);
    }
    problem.ambiguity_states.pop_back();

    FGOProcessor::FGOConfig config = makeStalePinBaseConfig();
    config.lambda_ratio_threshold = 1.0e200;
    config.ambiguity_hold_ratio_threshold = 1.0e200;
    config.use_cp_hold_recovery = false;

    FGOProcessor baseline_processor(config);
    const auto baseline = baseline_processor.optimizeProblem(problem);

    config.monitor_multiepoch_ar = true;
    config.multiepoch_ar_min_consensus_epochs = 3;
    config.multiepoch_ar_min_ambiguities = 4;
    config.surplus_validation_min_surplus_satellites = 1;
    FGOProcessor shadow_processor(config);
    const auto shadow = shadow_processor.optimizeProblem(problem);

    ASSERT_EQ(shadow.solution.solutions.size(), baseline.solution.solutions.size());
    for (std::size_t i = 0; i < shadow.solution.solutions.size(); ++i) {
        EXPECT_EQ(shadow.solution.solutions[i].status,
                  baseline.solution.solutions[i].status);
        EXPECT_DOUBLE_EQ(shadow.solution.solutions[i].ratio,
                         baseline.solution.solutions[i].ratio);
        EXPECT_TRUE(shadow.solution.solutions[i].position_ecef.isApprox(
            baseline.solution.solutions[i].position_ecef, 0.0));
    }

    const auto monitored = std::find_if(
        shadow.epoch_diagnostics.begin(), shadow.epoch_diagnostics.end(),
        [](const FGOProcessor::FGOEpochDiagnostics& diagnostics) {
            return diagnostics.multiepoch_ar_shadow.evaluated &&
                   diagnostics.multiepoch_ar_shadow.candidate_available;
        });
    ASSERT_NE(monitored, shadow.epoch_diagnostics.end());
    const auto& multi = monitored->multiepoch_ar_shadow;
    EXPECT_GE(multi.persistent_ambiguities, 4);
    EXPECT_GE(multi.minimum_support_epochs, 3);
    EXPECT_GT(multi.ratio, 0.0);
    EXPECT_GT(multi.bootstrapped_success_rate, 0.0);
    EXPECT_LE(multi.bootstrapped_success_rate, 1.0);
    EXPECT_TRUE(multi.history_integers_agree);
    EXPECT_GT(multi.candidate_position_ecef.norm(), 1.0e6);
    EXPECT_TRUE(multi.surplus_validation_evaluated);
    EXPECT_TRUE(multi.surplus_validation_pass);
    EXPECT_GE(multi.surplus_validation_fallback_level, 0);
    EXPECT_LE(multi.surplus_validation_fallback_level, 5);
    EXPECT_GT(multi.surplus_validation_surplus_used, 0);
    EXPECT_TRUE(multi.graph_cost_evaluated);
    EXPECT_GT(multi.graph_cost_factor_count, 0);
    EXPECT_TRUE(std::isfinite(multi.graph_cost_before));
    EXPECT_TRUE(std::isfinite(multi.graph_cost_after));
}

TEST(FGOAmbiguityOutcomeTelemetryTest,
     ConditionalMultibandMonitorIsDiagnosticOnlyAndBuildsTwoStageCandidate) {
    CpHoldTestOptions opt;
    opt.satellites = lambdaCapableSatelliteGeometry();
    opt.num_epochs = 12;
    auto problem = makeCpHoldFixedLagProblem(opt);

    // Add a clean L2 ambiguity/factor for every existing L1 satellite.  The
    // two bands share geometry but carry distinct integer cycles, matching
    // the raw multi-frequency DD state representation used by production.
    const std::size_t primary_count = problem.ambiguity_states.size();
    for (std::size_t idx = 0; idx < primary_count; ++idx) {
        auto secondary = problem.ambiguity_states[idx];
        secondary.signal = SignalType::GPS_L2C;
        secondary.wavelength_m = constants::GPS_L2_WAVELENGTH;
        const int sat_offset = static_cast<int>(secondary.satellite.prn) - 1;
        secondary.initial_ambiguity_m =
            static_cast<double>(200 + sat_offset) * secondary.wavelength_m;
        problem.ambiguity_states.push_back(secondary);
    }
    const auto primary_factors = problem.double_difference_carrier_factors;
    for (const auto& primary : primary_factors) {
        auto secondary = primary;
        const int sat_offset = static_cast<int>(secondary.satellite.prn) - 1;
        const double primary_ambiguity_m =
            static_cast<double>(100 + sat_offset) * constants::GPS_L1_WAVELENGTH;
        const double secondary_ambiguity_m =
            static_cast<double>(200 + sat_offset) * constants::GPS_L2_WAVELENGTH;
        const double ambiguity_delta_m =
            secondary_ambiguity_m - primary_ambiguity_m;
        secondary.ambiguity_index += primary_count;
        secondary.signal = SignalType::GPS_L2C;
        secondary.rover_satellite_model.corrected_carrier_m +=
            ambiguity_delta_m;
        secondary.observed_dd_carrier_m += ambiguity_delta_m;
        problem.double_difference_carrier_factors.push_back(secondary);
    }

    FGOProcessor::FGOConfig config = makeStalePinBaseConfig();
    config.lambda_ratio_threshold = 1.0;
    config.ambiguity_hold_ratio_threshold = 1.0e200;
    config.use_cp_hold_recovery = false;
    config.use_multi_frequency_double_difference = true;

    FGOProcessor baseline_processor(config);
    const auto baseline = baseline_processor.optimizeProblem(problem);

    config.monitor_conditional_multiband_ar = true;
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
            return diagnostics.conditional_multiband_ar_shadow
                       .candidate_position_ecef.norm() > 1.0e6;
        });
    ASSERT_NE(monitored, result.epoch_diagnostics.end());
    const auto& conditional = monitored->conditional_multiband_ar_shadow;
    EXPECT_TRUE(conditional.evaluated);
    EXPECT_EQ(conditional.primary_ambiguities,
              static_cast<int>(primary_count));
    EXPECT_EQ(conditional.secondary_ambiguities,
              static_cast<int>(primary_count));
    EXPECT_TRUE(conditional.primary_ratio_passed);
    EXPECT_TRUE(conditional.secondary_ratio_passed);
    EXPECT_TRUE(conditional.candidate_available);
    EXPECT_GT(conditional.primary_bootstrapped_success_rate, 0.0);
    EXPECT_GT(conditional.secondary_bootstrapped_success_rate, 0.0);
    EXPECT_GT(conditional.candidate_position_ecef.norm(), 1.0e6);
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
    EXPECT_DOUBLE_EQ(config.fix_demote_spp_model_max_agreement_m, 8.0);
    ASSERT_FALSE(config.use_fix_plausibility_demotion);

    FGOProcessor processor(config);
    const auto result = processor.optimizeProblem(problem);

    EXPECT_EQ(result.diagnostics.fix_plausibility_demotions, 0u);
    EXPECT_EQ(result.diagnostics.fix_plausibility_hold_skips, 0u);
    EXPECT_EQ(result.solution.solutions.size(), problem.epochs.size());
}

TEST(FGOFixedLagCovarianceTest, RejectsMissingAndIndefiniteCovariance) {
    using namespace fgo_gtsam_internal;
    EXPECT_FALSE(checkedPositionCovariance(missingPositionCovariance()).allFinite());
    EXPECT_FALSE(checkedPositionCovariance(Eigen::Matrix3d::Zero()).allFinite());
    Eigen::Matrix3d indefinite = Eigen::Matrix3d::Identity();
    indefinite(0, 1) = indefinite(1, 0) = 2.0;
    EXPECT_FALSE(checkedPositionCovariance(indefinite).allFinite());
    Eigen::Matrix3d roundoff = Eigen::Matrix3d::Identity();
    roundoff(0, 0) = -1e-12;
    const auto repaired = checkedPositionCovariance(roundoff);
    ASSERT_TRUE(repaired.allFinite());
    EXPECT_GE(Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(repaired)
                  .eigenvalues().minCoeff(), 0.0);
}

TEST(FGOFixedLagCovarianceTest, PropagatesRotationAndLeverArmAgainstNumericalJacobian) {
    const gtsam::Pose3 pose(gtsam::Rot3::RzRyRx(0.2, -0.3, 0.4),
                            gtsam::Point3(10, 20, 30));
    const gtsam::Pose3 ecef_T_nav(gtsam::Rot3::RzRyRx(-0.4, 0.8, 0.1),
                                  gtsam::Point3(100, 200, 300));
    const gtsam::gnss::LeverArm arm(gtsam::Point3(0.3, -0.7, 1.2), ecef_T_nav);
    gtsam::Matrix66 pose_cov = gtsam::Matrix66::Identity();
    pose_cov.diagonal() << 0.02, 0.03, 0.04, 2.0, 3.0, 4.0;
    pose_cov(0, 3) = pose_cov(3, 0) = 0.05;
    gtsam::Matrix36 numerical;
    for (int axis = 0; axis < 6; ++axis) {
        gtsam::Vector6 step = gtsam::Vector6::Zero();
        step(axis) = 1e-5;
        numerical.col(axis) = (arm.antennaPosition(pose.retract(step)) -
                              arm.antennaPosition(pose.retract(-step))) / 2e-5;
    }
    const Eigen::Matrix3d expected = numerical * pose_cov * numerical.transpose();
    const auto actual = fgo_gtsam_internal::antennaPositionCovariance(arm, pose, pose_cov);
    EXPECT_LT((actual - expected).norm(), 1e-7);
    EXPECT_LT((actual - actual.transpose()).norm(), 1e-12);
}

TEST(FGOFixedLagCovarianceTest, MatchesWindowExitAndTailWithoutChangingPositions) {
    CpHoldTestOptions opt;
    opt.num_epochs = 9;
    const auto problem = makeCpHoldFixedLagProblem(opt);
    auto config = makeCpHoldBaseConfig();
    config.fixed_lag_smoother_lag_s = 2.0;
    ASSERT_FALSE(config.compute_fixed_lag_position_covariance);
    const auto baseline = FGOProcessor(config).optimizeProblem(problem);
    config.compute_fixed_lag_position_covariance = true;
    const auto result = FGOProcessor(config).optimizeProblem(problem);
    ASSERT_EQ(result.solution.solutions.size(), opt.num_epochs);
    for (std::size_t i = 0; i < opt.num_epochs; ++i) {
        const auto& s = result.solution.solutions[i];
        EXPECT_TRUE(s.position_ecef.isApprox(baseline.solution.solutions[i].position_ecef, 0.0));
        EXPECT_EQ(s.status, baseline.solution.solutions[i].status);
        EXPECT_DOUBLE_EQ(s.ratio, baseline.solution.solutions[i].ratio);
        ASSERT_TRUE(s.position_covariance.allFinite()) << i;
        EXPECT_GT(s.position_covariance.trace(), 0.0);
        EXPECT_GE(Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(s.position_covariance)
                      .eigenvalues().minCoeff(), -1e-12);
    }
    // Epoch 0's final window snapshot is at epoch 2, not when it was born.
    opt.num_epochs = 3;
    const auto prefix = FGOProcessor(config).optimizeProblem(makeCpHoldFixedLagProblem(opt));
    EXPECT_TRUE(result.solution.solutions[0].position_covariance.isApprox(
        prefix.solution.solutions[0].position_covariance, 1e-10));
    EXPECT_DOUBLE_EQ(result.epoch_diagnostics[0].solution_latency_s, 2.0);
    EXPECT_DOUBLE_EQ(result.epoch_diagnostics.back().solution_latency_s, 0.0);
    // A one-epoch problem uses the batch backend; two epochs is the earliest
    // fixed-lag prefix and must still differ from the final window snapshot.
    opt.num_epochs = 2;
    const auto early = FGOProcessor(config).optimizeProblem(makeCpHoldFixedLagProblem(opt));
    ASSERT_TRUE(early.solution.solutions[0].position_covariance.allFinite());
    EXPECT_GT((early.solution.solutions[0].position_covariance -
               result.solution.solutions[0].position_covariance).norm(), 1e-6);
}

TEST(FGOFixedLagCovarianceTest, FixedSnapshotIsCausalAndPositionPreserving) {
    CpHoldTestOptions opt;
    opt.satellites = lambdaCapableSatelliteGeometry();
    opt.num_epochs = 12;
    const auto problem = makeCpHoldFixedLagProblem(opt);
    auto config = makeFixDemoteBaseConfig();
    const auto baseline = FGOProcessor(config).optimizeProblem(problem);
    config.compute_fixed_lag_position_covariance = true;
    const auto result = FGOProcessor(config).optimizeProblem(problem);
    std::size_t fixed = 0;
    for (std::size_t i = 0; i < opt.num_epochs; ++i) {
        const auto& s = result.solution.solutions.at(i);
        const auto& b = baseline.solution.solutions.at(i);
        EXPECT_TRUE(s.position_ecef.isApprox(b.position_ecef, 0.0));
        EXPECT_EQ(s.status, b.status);
        EXPECT_DOUBLE_EQ(s.ratio, b.ratio);
        if (s.status != SolutionStatus::FIXED) continue;
        ++fixed;
        ASSERT_TRUE(s.position_covariance.allFinite()) << i;
        EXPECT_GT(s.position_covariance.trace(), 0.0);
        EXPECT_DOUBLE_EQ(result.epoch_diagnostics[i].solution_latency_s, 0.0);
    }
    EXPECT_GT(fixed, 0u);
}

TEST(FGOFixedLagCovarianceTest, ReportOnlyPredictionDoesNotBorrowGraphMarginal) {
    CpHoldTestOptions opt;
    opt.num_epochs = 11;  // Stop at the replaced epoch before any later smoothing.
    opt.carrier_corrupt_epochs = {10};
    opt.carrier_corrupt_offset_ecef = Vector3d(150.0, 0.0, 0.0);
    const auto problem = makeCpHoldFixedLagProblem(opt);
    auto config = makeCpHoldBaseConfig();
    config.use_cp_hold_recovery = true;
    config.cp_hold_main_residual_threshold_m = 3.0;
    config.cp_hold_persist_epochs = 1000;
    config.cp_hold_catastrophic_threshold_m = 5.0;
    config.cp_hold_fast_worst_satellite_min_m = 1.0;
    config.cp_hold_max_gdop = 0.0;
    config.cp_hold_pose_replace_threshold_m = 5.0;
    const auto baseline = FGOProcessor(config).optimizeProblem(problem);
    config.compute_fixed_lag_position_covariance = true;
    const auto result = FGOProcessor(config).optimizeProblem(problem);
    ASSERT_GT(result.diagnostics.sanity_pose_replacements, 0u);
    EXPECT_FALSE(result.solution.solutions.back().position_covariance.allFinite());
    EXPECT_TRUE(result.solution.solutions.back().position_ecef.isApprox(
        baseline.solution.solutions.back().position_ecef, 0.0));
    EXPECT_EQ(result.solution.solutions.back().status, SolutionStatus::FLOAT);
    EXPECT_DOUBLE_EQ(result.epoch_diagnostics.back().solution_latency_s, 0.0);
}

TEST(FGOFixedLagCovarianceTest, ReoptimizedIntegerPoseCarriesItsOwnMarginal) {
    using namespace fgo_gtsam_internal;
    const gtsam::Key pose_key = gtsam::Symbol('x', 0);
    const gtsam::Key ambiguity_key = gtsam::Symbol('a', 0);
    gtsam::NonlinearFactorGraph graph;
    const gtsam::Pose3 pose(gtsam::Rot3(), gtsam::Point3(1, 2, 3));
    graph.addPrior(pose_key, pose, gtsam::noiseModel::Isotropic::Sigma(6, 2.0));
    graph.addPrior(ambiguity_key, 1.0, gtsam::noiseModel::Isotropic::Sigma(1, 1.0));
    gtsam::Values values;
    values.insert(pose_key, pose);
    values.insert(ambiguity_key, 1.0);
    FGOProcessor::FGOConfig config;
    config.compute_fixed_lag_position_covariance = true;
    const auto outcome = evaluateIntegerConstrainedGraphCost(
        graph, values, {{ambiguity_key, 1.0}}, pose_key, config);
    ASSERT_TRUE(outcome.pass);
    ASSERT_TRUE(outcome.optimized_pose.has_value());
    ASSERT_TRUE(outcome.optimized_pose_covariance.has_value());
    EXPECT_TRUE(outcome.optimized_pose_covariance->isApprox(
        4.0 * gtsam::Matrix66::Identity(), 1e-10));
}

TEST(FGOExternalDopplerDrShadowTest, MonitorDoesNotChangeSolutionAuthority) {
    CpHoldTestOptions opt;
    opt.satellites = lambdaCapableSatelliteGeometry();
    opt.num_epochs = 12;
    auto problem = makeCpHoldFixedLagProblem(opt);

    const std::array<Vector3d, 4> line_of_sight = {
        Vector3d::UnitX(), Vector3d::UnitY(), Vector3d::UnitZ(),
        Vector3d(1.0, 1.0, 1.0).normalized(),
    };
    for (std::size_t epoch = 0; epoch < problem.epochs.size(); ++epoch) {
        for (std::size_t row = 0; row < line_of_sight.size(); ++row) {
            FGOProcessor::SingleDifferenceDopplerFactor factor;
            factor.epoch_index = epoch;
            factor.satellite = SatelliteId(
                GNSSSystem::GPS, static_cast<uint8_t>(row + 2));
            factor.reference_satellite = SatelliteId(GNSSSystem::GPS, 1);
            factor.signal = SignalType::GPS_L1CA;
            factor.los = line_of_sight[row];
            factor.residual_mps = 0.0;
            factor.sigma_mps = 0.1;
            factor.elevation_rad = 0.7;
            problem.single_difference_doppler_factors.push_back(factor);
        }
    }

    FGOProcessor::FGOConfig off_config = makeFixDemoteBaseConfig();
    const auto off_result = FGOProcessor(off_config).optimizeProblem(problem);

    FGOProcessor::FGOConfig shadow_config = off_config;
    shadow_config.monitor_external_doppler_dr = true;
    shadow_config.external_doppler_dr_reset_min_ratio = 1.5;
    const auto shadow_result =
        FGOProcessor(shadow_config).optimizeProblem(problem);

    ASSERT_EQ(off_result.solution.solutions.size(),
              shadow_result.solution.solutions.size());
    for (std::size_t i = 0; i < off_result.solution.solutions.size(); ++i) {
        EXPECT_TRUE(off_result.solution.solutions[i].position_ecef.isApprox(
            shadow_result.solution.solutions[i].position_ecef, 0.0));
        EXPECT_EQ(off_result.solution.solutions[i].status,
                  shadow_result.solution.solutions[i].status);
    }
    EXPECT_GT(shadow_result.diagnostics.external_doppler_dr_accepts +
                  shadow_result.diagnostics.external_doppler_dr_rejects,
              0u);
    EXPECT_TRUE(std::any_of(
        shadow_result.epoch_diagnostics.begin(),
        shadow_result.epoch_diagnostics.end(),
        [](const auto& epoch) { return epoch.external_dr_evaluated; }));
}

TEST(FGOCandidateIntegrityWitnessTest,
     MonitorDoesNotChangeSolutionAuthorityAndEmitsEpochVerdicts) {
    CpHoldTestOptions opt;
    opt.satellites = lambdaCapableSatelliteGeometry();
    opt.num_epochs = 12;
    auto problem = makeCpHoldFixedLagProblem(opt);

    const std::array<Vector3d, 4> line_of_sight = {
        Vector3d::UnitX(), Vector3d::UnitY(), Vector3d::UnitZ(),
        Vector3d(1.0, 1.0, 1.0).normalized(),
    };
    for (std::size_t epoch = 0; epoch < problem.epochs.size(); ++epoch) {
        for (std::size_t row = 0; row < line_of_sight.size(); ++row) {
            FGOProcessor::SingleDifferenceDopplerFactor factor;
            factor.epoch_index = epoch;
            factor.satellite = SatelliteId(
                GNSSSystem::GPS, static_cast<uint8_t>(row + 2));
            factor.reference_satellite = SatelliteId(GNSSSystem::GPS, 1);
            factor.signal = SignalType::GPS_L1CA;
            factor.los = line_of_sight[row];
            factor.residual_mps = 0.0;
            factor.sigma_mps = 0.1;
            factor.elevation_rad = 0.7;
            problem.single_difference_doppler_factors.push_back(factor);
        }
    }

    FGOProcessor::FGOConfig off_config = makeFixDemoteBaseConfig();
    const auto off_result = FGOProcessor(off_config).optimizeProblem(problem);

    FGOProcessor::FGOConfig shadow_config = off_config;
    shadow_config.monitor_candidate_integrity_witness = true;
    shadow_config.external_doppler_dr_reset_min_ratio = 1.5;
    const auto shadow_result =
        FGOProcessor(shadow_config).optimizeProblem(problem);

    ASSERT_EQ(off_result.solution.solutions.size(),
              shadow_result.solution.solutions.size());
    for (std::size_t i = 0; i < off_result.solution.solutions.size(); ++i) {
        EXPECT_TRUE(off_result.solution.solutions[i].position_ecef.isApprox(
            shadow_result.solution.solutions[i].position_ecef, 0.0));
        EXPECT_EQ(off_result.solution.solutions[i].status,
                  shadow_result.solution.solutions[i].status);
    }
    EXPECT_GT(shadow_result.diagnostics.candidate_integrity_witness_evaluated,
              0u);
    EXPECT_EQ(shadow_result.diagnostics.candidate_integrity_witness_evaluated,
              static_cast<std::size_t>(std::count_if(
                  shadow_result.epoch_diagnostics.begin(),
                  shadow_result.epoch_diagnostics.end(),
                  [](const auto& epoch) {
                      return epoch.candidate_integrity_witness_evaluated;
                  })));
    EXPECT_TRUE(std::any_of(
        shadow_result.epoch_diagnostics.begin(),
        shadow_result.epoch_diagnostics.end(),
        [](const auto& epoch) {
            return epoch.candidate_integrity_witness_evaluated &&
                   epoch.candidate_integrity_anchor_available &&
                   epoch.candidate_integrity_doppler_available;
        }));
}

TEST(FGOMotionConstraintShadowTest,
     DirectionalVibrationIsVisibleAndMonitorDoesNotChangeSolution) {
    CpHoldTestOptions opt;
    opt.num_epochs = 3;
    auto problem = makeCpHoldFixedLagProblem(opt);
    const double g = problem.imu.noise.gravity_mps2;
    for (std::size_t i = 0; i < problem.imu.samples_body_flu.size(); ++i) {
        problem.imu.samples_body_flu[i].accel_raw =
            i % 2 == 0 ? Vector3d(g, 0.0, 0.0) : Vector3d(0.0, 0.0, g);
        problem.imu.samples_body_flu[i].gyro_raw_radps =
            i % 2 == 0 ? Vector3d(0.1, 0.0, 0.0)
                       : Vector3d(0.0, 0.1, 0.0);
    }

    FGOProcessor::FGOConfig off_config = makeCpHoldBaseConfig();
    const auto off_result = FGOProcessor(off_config).optimizeProblem(problem);

    FGOProcessor::FGOConfig shadow_config = off_config;
    shadow_config.monitor_motion_constraints = true;
    const auto shadow_result =
        FGOProcessor(shadow_config).optimizeProblem(problem);

    ASSERT_EQ(off_result.solution.solutions.size(),
              shadow_result.solution.solutions.size());
    ASSERT_EQ(shadow_result.epoch_diagnostics.size(), problem.epochs.size());
    for (std::size_t i = 0; i < off_result.solution.solutions.size(); ++i) {
        EXPECT_TRUE(off_result.solution.solutions[i].position_ecef.isApprox(
            shadow_result.solution.solutions[i].position_ecef, 0.0));
        EXPECT_EQ(off_result.solution.solutions[i].status,
                  shadow_result.solution.solutions[i].status);
    }
    EXPECT_GT(shadow_result.epoch_diagnostics.front()
                  .motion_constraint_accel_std_mps2,
              5.0);
    EXPECT_GT(shadow_result.epoch_diagnostics.front()
                  .motion_constraint_gyro_std_radps,
              0.05);
    EXPECT_FALSE(shadow_result.epoch_diagnostics.front().zupt_candidate);
}

TEST(FGOMotionConstraintGateTest, AppliesStationaryZuptAndRejectsTurningNhc) {
    CpHoldTestOptions opt;
    opt.num_epochs = 3;
    const auto stationary_problem = makeCpHoldFixedLagProblem(opt);

    FGOProcessor::FGOConfig zupt_config = makeCpHoldBaseConfig();
    zupt_config.use_zupt = true;
    const auto zupt_result =
        FGOProcessor(zupt_config).optimizeProblem(stationary_problem);
    EXPECT_GT(zupt_result.diagnostics.zupt_epochs, 0u);
    EXPECT_TRUE(std::any_of(
        zupt_result.epoch_diagnostics.begin(),
        zupt_result.epoch_diagnostics.end(),
        [](const auto& epoch) { return epoch.zupt_applied; }));

    auto straight_problem = stationary_problem;
    straight_problem.imu.init_velocity_nav = Vector3d(3.0, 0.0, 0.0);
    FGOProcessor::FGOConfig nhc_config = makeCpHoldBaseConfig();
    nhc_config.use_nhc = true;
    const auto straight_result =
        FGOProcessor(nhc_config).optimizeProblem(straight_problem);
    EXPECT_GT(straight_result.diagnostics.nhc_epochs, 0u);

    auto turning_problem = straight_problem;
    for (auto& sample : turning_problem.imu.samples_body_flu) {
        sample.gyro_raw_radps.z() += 0.5;
    }
    const auto turning_result =
        FGOProcessor(nhc_config).optimizeProblem(turning_problem);
    EXPECT_EQ(turning_result.diagnostics.nhc_epochs, 0u);
    EXPECT_TRUE(std::none_of(
        turning_result.epoch_diagnostics.begin(),
        turning_result.epoch_diagnostics.end(),
        [](const auto& epoch) { return epoch.nhc_applied; }));
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

TEST(FGOFixDemoteTest, GeometryFreeLowRedundancyGrossSppGuardDemotesAndSkipsHold) {
    constexpr std::size_t kBadEpoch = 1;
    CpHoldTestOptions opt;
    opt.satellites = lambdaCapableSatelliteGeometry();
    opt.num_epochs = 25;
    for (std::size_t e = 0; e <= 3; ++e) {
        opt.pr_corrupt_epochs.insert(e);
    }
    opt.pr_baseline_bias_m = 0.0;
    opt.pr_dominant_extra_bias_m = 80.0;
    auto problem = makeCpHoldFixedLagProblem(opt);
    // Keep the synthetic LAMBDA candidate accepted but non-trivial: a
    // quarter-cycle offset on one arc avoids the noise-free fixture's
    // unrealistically enormous ratio while retaining a clear integer winner.
    for (auto& factor : problem.double_difference_carrier_factors) {
        if (factor.ambiguity_index == 0) {
            factor.observed_dd_carrier_m += 0.05;
            factor.rover_satellite_model.corrected_carrier_m += 0.05;
        }
    }
    for (std::size_t e = 0; e <= 3; ++e) {
        problem.epochs[e].fresh_spp_solution = true;
        problem.epochs[e].position_ecef += Vector3d(100.0, 0.0, 0.0);
    }

    FGOProcessor::FGOConfig off_config = makeFixDemoteBaseConfig();
    off_config.use_fixed_lag_partial_lambda = true;
    off_config.max_lambda_ambiguities = 6;
    off_config.use_epoch_quality_gates = true;
    off_config.gate_gdop_max = 1e9;
    off_config.gate_min_satellites = 0;
    off_config.gate_ddpr_res_max_m = 1e9;
    off_config.gate_per_sat_res_max_m = 1e9;
    off_config.use_fix_plausibility_demotion = true;
    off_config.fix_demote_distance_m = 1e9;
    off_config.fix_demote_res_m = 0.0;

    const auto off_result = FGOProcessor(off_config).optimizeProblem(problem);
    ASSERT_EQ(off_result.solution.solutions[kBadEpoch].status,
              SolutionStatus::FIXED)
        << "fixture sanity: without the GF guard the grossly SPP-disagreed "
           "low-redundancy candidate must remain FIXED";
    ASSERT_LE(off_result.epoch_diagnostics[kBadEpoch]
                  .lambda_candidate_fixed_ambiguities,
              6);
    ASSERT_LE(off_result.epoch_diagnostics[kBadEpoch].lambda_candidate_ratio,
              10.0);

    FGOProcessor::FGOConfig on_config = off_config;
    on_config.use_fix_plausibility_demotion = false;
    on_config.use_geometry_free_cycle_slip_reset = true;
    const auto on_result = FGOProcessor(on_config).optimizeProblem(problem);

    EXPECT_NE(on_result.solution.solutions[kBadEpoch].status,
              SolutionStatus::FIXED);
    EXPECT_GT(on_result.diagnostics.fix_plausibility_demotions, 0u);
    EXPECT_GT(on_result.diagnostics.geometry_free_fix_guard_demotions, 0u);
    EXPECT_GT(on_result.diagnostics.fix_plausibility_hold_skips, 0u);
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

TEST(FGOCpHoldFloatRecoveryTest,
     KeepsWeakCarrierFactorsButQuarantinesTheirAmbiguitiesDuringHold) {
    CpHoldTestOptions opt;
    opt.satellites = lambdaCapableSatelliteGeometry();
    opt.num_epochs = 30;
    for (std::size_t epoch = 10; epoch <= 12; ++epoch) {
        opt.carrier_corrupt_epochs.insert(epoch);
    }
    const auto problem = makeCpHoldFixedLagProblem(opt);

    FGOProcessor::FGOConfig legacy = makeFixDemoteBaseConfig();
    legacy.use_cp_hold_recovery = true;
    legacy.cp_hold_main_residual_threshold_m = 3.0;
    legacy.cp_hold_persist_epochs = 3;
    legacy.cp_hold_catastrophic_threshold_m = 1.0e6;
    legacy.cp_hold_multipath_median_ratio = 0.0;
    legacy.cp_hold_max_gdop = 0.0;
    legacy.cp_hold_epochs = 3;
    legacy.cp_hold_release_threshold_m = 2.0;
    legacy.cp_hold_release_count = 1;

    const auto legacy_result = FGOProcessor(legacy).optimizeProblem(problem);
    ASSERT_GT(legacy_result.diagnostics.cp_hold_epochs_held, 0u);
    EXPECT_TRUE(std::any_of(
        legacy_result.epoch_diagnostics.begin(),
        legacy_result.epoch_diagnostics.end(),
        [](const auto& epoch) {
            return epoch.carrier_hold_active &&
                   epoch.carrier_factors_suppressed_hold > 0 &&
                   epoch.carrier_factors_added == 0;
        }));

    FGOProcessor::FGOConfig recovery = legacy;
    recovery.use_cp_hold_float_recovery = true;
    const auto recovery_result =
        FGOProcessor(recovery).optimizeProblem(problem);

    EXPECT_TRUE(std::any_of(
        recovery_result.epoch_diagnostics.begin(),
        recovery_result.epoch_diagnostics.end(),
        [](const auto& epoch) {
            return epoch.carrier_hold_active &&
                   epoch.carrier_factors_added > 0 &&
                   epoch.carrier_factors_suppressed_hold == 0 &&
                   epoch.ambiguity_candidates_excluded_hold ==
                       epoch.carrier_factors_added &&
                   epoch.ambiguity_candidates_after_hold == 0 &&
                   epoch.lambda_attempts == 0;
        }));
    for (std::size_t i = 0;
         i < recovery_result.solution.solutions.size(); ++i) {
        if (recovery_result.epoch_diagnostics[i].carrier_hold_active) {
            EXPECT_NE(recovery_result.solution.solutions[i].status,
                      SolutionStatus::FIXED);
        }
    }
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

TEST(FGOGtsamUndifferencedDopplerFactorTest, MatchesNativeSignAndJacobian) {
    using libgnss::fgo_gtsam_internal::UndifferencedDopplerVelocityFactor;

    // The raw adapter stores los as the satellite-to-receiver vector.  This
    // is the same convention consumed by doppler_velocity_wls::predict:
    // los dot v + receiver_clock_drift.  Keep the values non-axis-aligned so
    // a sign or column-order mistake cannot pass an axis-only fixture.
    const gtsam::Vector3 los(0.6, -0.8, 0.0);
    const gtsam::Vector3 velocity(5.0, -2.0, 1.0);
    constexpr double clock_drift_mps = 1.5;
    const double measured_mps = los.dot(velocity) + clock_drift_mps;
    const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
    const UndifferencedDopplerVelocityFactor factor(
        gtsam::Symbol('v', 0), gtsam::Symbol('d', 0), los,
        measured_mps, noise);

    gtsam::Matrix H_velocity;
    gtsam::Matrix H_clock;
    const gtsam::Vector zero_error = factor.evaluateError(
        velocity, clock_drift_mps, &H_velocity, &H_clock);
    ASSERT_EQ(zero_error.size(), 1);
    EXPECT_NEAR(zero_error(0), 0.0, 1e-12);
    ASSERT_EQ(H_velocity.rows(), 1);
    ASSERT_EQ(H_velocity.cols(), 3);
    EXPECT_NEAR(H_velocity(0, 0), 0.6, 1e-12);
    EXPECT_NEAR(H_velocity(0, 1), -0.8, 1e-12);
    EXPECT_NEAR(H_velocity(0, 2), 0.0, 1e-12);
    ASSERT_EQ(H_clock.rows(), 1);
    ASSERT_EQ(H_clock.cols(), 1);
    EXPECT_NEAR(H_clock(0, 0), 1.0, 1e-12);

    // A positive velocity perturbation along the first component must produce
    // the positive native-contract residual, not its negation.
    const gtsam::Vector perturbed_error = factor.evaluateError(
        velocity + gtsam::Vector3(2.0, 0.0, 0.0), clock_drift_mps, nullptr, nullptr);
    ASSERT_EQ(perturbed_error.size(), 1);
    EXPECT_NEAR(perturbed_error(0), 1.2, 1e-12);
}

TEST(FGOGtsamSignalBiasFactorTest, UsesMeterBiasAndAnalyticJacobians) {
    using libgnss::fgo_gtsam_internal::PseudorangeFactorPlainSignalBias;
    using libgnss::fgo_gtsam_internal::PseudorangeFactorISBSignalBias;

    const gtsam::Point3 receiver(6'370'000.0, 1'000.0, 2'000.0);
    const gtsam::Point3 satellite = receiver + gtsam::Point3(20'000'000.0,
                                                               1'000'000.0,
                                                               -500'000.0);
    const double range = (satellite - receiver).norm();
    constexpr double clock_s = 2.0e-6;
    constexpr double isb_s = -1.0e-6;
    constexpr double signal_bias_m = -160.0;
    const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);

    const PseudorangeFactorPlainSignalBias plain(
        gtsam::Symbol('x', 0), gtsam::Symbol('c', 0), gtsam::Symbol('f', 1),
        range + gtsam::gnss::C_LIGHT * clock_s + signal_bias_m, satellite,
        noise);
    gtsam::Matrix H_position;
    gtsam::Matrix H_clock;
    gtsam::Matrix H_signal;
    const gtsam::Vector plain_error = plain.evaluateError(
        receiver, clock_s, signal_bias_m, &H_position, &H_clock, &H_signal);
    ASSERT_EQ(plain_error.size(), 1);
    EXPECT_NEAR(plain_error(0), 0.0, 1e-9);
    ASSERT_EQ(H_position.rows(), 1);
    ASSERT_EQ(H_position.cols(), 3);
    EXPECT_NEAR(H_position.norm(), 1.0, 1e-12);
    EXPECT_NEAR(H_clock(0, 0), gtsam::gnss::C_LIGHT, 1e-6);
    EXPECT_DOUBLE_EQ(H_signal(0, 0), 1.0);

    const PseudorangeFactorISBSignalBias with_isb(
        gtsam::Symbol('x', 0), gtsam::Symbol('c', 0), gtsam::Symbol('i', 1),
        gtsam::Symbol('f', 2),
        range + gtsam::gnss::C_LIGHT * (clock_s + isb_s) + signal_bias_m,
        satellite, noise);
    gtsam::Matrix H_isb;
    const gtsam::Vector isb_error = with_isb.evaluateError(
        receiver, clock_s, isb_s, signal_bias_m, nullptr, nullptr, &H_isb,
        nullptr);
    ASSERT_EQ(isb_error.size(), 1);
    EXPECT_NEAR(isb_error(0), 0.0, 1e-9);
    EXPECT_NEAR(H_isb(0, 0), gtsam::gnss::C_LIGHT, 1e-6);
}

TEST(FGOGtsamSignalBiasFactorTest, KeyOrdinalIsStableForSecondarySignalsOnly) {
    EXPECT_GT(signal_bias::ordinal(GNSSSystem::GPS, SignalType::GPS_L5), 0);
    EXPECT_GT(signal_bias::ordinal(GNSSSystem::Galileo, SignalType::GAL_E5A), 0);
    EXPECT_NE(signal_bias::ordinal(GNSSSystem::GPS, SignalType::GPS_L5),
              signal_bias::ordinal(GNSSSystem::Galileo, SignalType::GAL_E5A));
    EXPECT_EQ(signal_bias::ordinal(GNSSSystem::GPS, SignalType::GPS_L1CA), -1);
    EXPECT_EQ(signal_bias::ordinal(GNSSSystem::Galileo, SignalType::GAL_E1), -1);
}

TEST(FGOGtsamResidualIonosphereFactorTest, UsesCoefficientSignAndJacobian) {
    using libgnss::fgo_gtsam_internal::PseudorangeFactorPlainResidualIonosphere;
    const gtsam::Point3 receiver(6'370'000.0, 1'000.0, 2'000.0);
    const gtsam::Point3 satellite = receiver + gtsam::Point3(20'000'000.0,
                                                               1'000'000.0,
                                                               -500'000.0);
    const double range = (satellite - receiver).norm();
    constexpr double clock_s = 2.0e-6;
    constexpr double ionosphere_m = 4.5;
    constexpr double coefficient = 1.75;
    const double measurement = range + gtsam::gnss::C_LIGHT * clock_s +
                               coefficient * ionosphere_m;
    const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
    const PseudorangeFactorPlainResidualIonosphere factor(
        gtsam::Symbol('x', 0), gtsam::Symbol('c', 0), gtsam::Symbol('j', 0),
        measurement, satellite, coefficient, noise);
    gtsam::Matrix H_position;
    gtsam::Matrix H_clock;
    gtsam::Matrix H_ionosphere;
    const gtsam::Vector error = factor.evaluateError(
        receiver, clock_s, ionosphere_m, &H_position, &H_clock,
        &H_ionosphere);
    ASSERT_EQ(error.size(), 1);
    EXPECT_NEAR(error(0), 0.0, 1e-9);
    EXPECT_NEAR(H_position.norm(), 1.0, 1e-12);
    EXPECT_NEAR(H_clock(0, 0), gtsam::gnss::C_LIGHT, 1e-6);
    EXPECT_NEAR(H_ionosphere(0, 0), coefficient, 1e-12);
    const gtsam::Vector positive_state = factor.evaluateError(
        receiver, clock_s, ionosphere_m + 1.0, nullptr, nullptr, nullptr);
    EXPECT_NEAR(positive_state(0), coefficient, 1e-9);
}

TEST(FGOGtsamResidualIonosphereFactorTest, FullBiasAndIonosphereKeysAreIndependent) {
    using libgnss::fgo_gtsam_internal::
        PseudorangeFactorISBSignalBiasResidualIonosphere;
    const gtsam::Point3 receiver(6'370'000.0, 1'000.0, 2'000.0);
    const gtsam::Point3 satellite = receiver + gtsam::Point3(21'000'000.0,
                                                               -1'000'000.0,
                                                               400'000.0);
    const double range = (satellite - receiver).norm();
    constexpr double clock_s = 1.0e-6;
    constexpr double isb_s = -0.25e-6;
    constexpr double signal_bias_m = -160.0;
    constexpr double ionosphere_m = 3.0;
    constexpr double coefficient = 1.25;
    const double measurement =
        range + gtsam::gnss::C_LIGHT * (clock_s + isb_s) + signal_bias_m +
        coefficient * ionosphere_m;
    const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
    const PseudorangeFactorISBSignalBiasResidualIonosphere factor(
        gtsam::Symbol('x', 0), gtsam::Symbol('c', 0), gtsam::Symbol('i', 1),
        gtsam::Symbol('f', 2), gtsam::Symbol('j', 0), measurement, satellite,
        coefficient, noise);
    gtsam::Matrix H_isb;
    gtsam::Matrix H_signal;
    gtsam::Matrix H_ionosphere;
    const gtsam::Vector error = factor.evaluateError(
        receiver, clock_s, isb_s, signal_bias_m, ionosphere_m, nullptr,
        nullptr, &H_isb, &H_signal, &H_ionosphere);
    ASSERT_EQ(error.size(), 1);
    EXPECT_NEAR(error(0), 0.0, 1e-9);
    EXPECT_NEAR(H_isb(0, 0), gtsam::gnss::C_LIGHT, 1e-6);
    EXPECT_DOUBLE_EQ(H_signal(0, 0), 1.0);
    EXPECT_NEAR(H_ionosphere(0, 0), coefficient, 1e-12);
}

TEST(FGOGtsamSourceClockC0DFactorTest,
     MatchesSourceResidualJacobiansAndSecondsNoise) {
    using libgnss::fgo_gtsam_internal::SourceClockC0DFactor;
    constexpr double dt_s = 1.25;
    constexpr double clock_previous_s = 2.0e-6;
    constexpr double drift_previous_mps = 1.5;
    constexpr double drift_current_mps = -0.5;
    const double drift_term_s =
        (drift_previous_mps + drift_current_mps) * dt_s /
        (2.0 * gtsam::gnss::C_LIGHT);
    const double clock_current_s =
        clock_previous_s + drift_term_s + 3.0e-12;
    const auto noise = gtsam::noiseModel::Isotropic::Sigma(
        1, 0.1 / gtsam::gnss::C_LIGHT);
    const SourceClockC0DFactor factor(
        gtsam::Symbol('c', 0), gtsam::Symbol('c', 1),
        gtsam::Symbol('d', 0), gtsam::Symbol('d', 1), dt_s, noise);

    gtsam::Matrix H_clock_previous;
    gtsam::Matrix H_clock_current;
    gtsam::Matrix H_drift_previous;
    gtsam::Matrix H_drift_current;
    const gtsam::Vector error = factor.evaluateError(
        clock_previous_s, clock_current_s, drift_previous_mps,
        drift_current_mps, &H_clock_previous, &H_clock_current,
        &H_drift_previous, &H_drift_current);

    ASSERT_EQ(error.size(), 1);
    EXPECT_NEAR(error(0), 3.0e-12, 1.0e-18);
    EXPECT_DOUBLE_EQ(H_clock_previous(0, 0), -1.0);
    EXPECT_DOUBLE_EQ(H_clock_current(0, 0), 1.0);
    EXPECT_DOUBLE_EQ(H_drift_previous(0, 0),
                     -dt_s / (2.0 * gtsam::gnss::C_LIGHT));
    EXPECT_DOUBLE_EQ(H_drift_current(0, 0),
                     -dt_s / (2.0 * gtsam::gnss::C_LIGHT));
    EXPECT_NEAR(noise->sigmas()(0), 0.1 / gtsam::gnss::C_LIGHT, 1.0e-24);
}

TEST(FGOGtsamSourceClockC0DFactorTest,
     MeterStateUsesOfficialMetreEquationAndNoise) {
    using libgnss::fgo_gtsam_internal::SourceClockC0DFactor;
    constexpr double dt_s = 1.25;
    constexpr double clock_previous_m = 17.0;
    constexpr double drift_previous_mps = 1.5;
    constexpr double drift_current_mps = -0.5;
    constexpr double residual_m = 0.003;
    const double drift_term_m =
        (drift_previous_mps + drift_current_mps) * dt_s / 2.0;
    const double clock_current_m = clock_previous_m + drift_term_m + residual_m;
    const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 0.1);
    const SourceClockC0DFactor factor(
        gtsam::Symbol('c', 0), gtsam::Symbol('c', 1),
        gtsam::Symbol('d', 0), gtsam::Symbol('d', 1), dt_s, noise, true);

    gtsam::Matrix H_clock_previous;
    gtsam::Matrix H_clock_current;
    gtsam::Matrix H_drift_previous;
    gtsam::Matrix H_drift_current;
    const gtsam::Vector error = factor.evaluateError(
        clock_previous_m, clock_current_m, drift_previous_mps,
        drift_current_mps, &H_clock_previous, &H_clock_current,
        &H_drift_previous, &H_drift_current);
    ASSERT_EQ(error.size(), 1);
    EXPECT_NEAR(error(0), residual_m, 1.0e-12);
    EXPECT_DOUBLE_EQ(H_clock_previous(0, 0), -1.0);
    EXPECT_DOUBLE_EQ(H_clock_current(0, 0), 1.0);
    EXPECT_DOUBLE_EQ(H_drift_previous(0, 0), -dt_s / 2.0);
    EXPECT_DOUBLE_EQ(H_drift_current(0, 0), -dt_s / 2.0);
    EXPECT_DOUBLE_EQ(noise->sigmas()(0), 0.1);
}

TEST(FGOGtsamClockStateParityTest,
     PseudorangePlainAndIsbSwitchClockAndIsbUnitsTogether) {
    using libgnss::fgo_gtsam_internal::PseudorangeFactorPlain;
    using libgnss::fgo_gtsam_internal::PseudorangeFactorISB;
    const gtsam::Point3 receiver(6'370'000.0, 1'000.0, 2'000.0);
    const gtsam::Point3 satellite = receiver + gtsam::Point3(
        20'000'000.0, 1'000'000.0, -500'000.0);
    const double range = (satellite - receiver).norm();
    constexpr double clock_m = 12.5;
    constexpr double isb_m = -2.25;
    const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);

    const PseudorangeFactorPlain plain_meter(
        gtsam::Symbol('x', 0), gtsam::Symbol('c', 0),
        range + clock_m, satellite, noise, true);
    gtsam::Matrix H_clock;
    const gtsam::Vector plain_meter_error = plain_meter.evaluateError(
        receiver, clock_m, nullptr, &H_clock);
    ASSERT_EQ(plain_meter_error.size(), 1);
    EXPECT_NEAR(plain_meter_error(0), 0.0, 1.0e-9);
    EXPECT_DOUBLE_EQ(H_clock(0, 0), 1.0);

    const PseudorangeFactorISB isb_meter(
        gtsam::Symbol('x', 0), gtsam::Symbol('c', 0), gtsam::Symbol('i', 0),
        range + clock_m + isb_m, satellite, noise, true);
    gtsam::Matrix H_base;
    gtsam::Matrix H_isb;
    const gtsam::Vector isb_meter_error = isb_meter.evaluateError(
        receiver, clock_m, isb_m, nullptr, &H_base, &H_isb);
    ASSERT_EQ(isb_meter_error.size(), 1);
    EXPECT_NEAR(isb_meter_error(0), 0.0, 1.0e-9);
    EXPECT_DOUBLE_EQ(H_base(0, 0), 1.0);
    EXPECT_DOUBLE_EQ(H_isb(0, 0), 1.0);

    constexpr double clock_s = 12.5e-9;
    constexpr double isb_s = -2.25e-9;
    const PseudorangeFactorISB isb_seconds(
        gtsam::Symbol('x', 0), gtsam::Symbol('c', 0), gtsam::Symbol('i', 0),
        range + gtsam::gnss::C_LIGHT * (clock_s + isb_s), satellite, noise);
    const gtsam::Vector isb_seconds_error = isb_seconds.evaluateError(
        receiver, clock_s, isb_s, nullptr, &H_base, &H_isb);
    ASSERT_EQ(isb_seconds_error.size(), 1);
    EXPECT_NEAR(isb_seconds_error(0), 0.0, 1.0e-9);
    EXPECT_NEAR(H_base(0, 0), gtsam::gnss::C_LIGHT, 1.0e-6);
    EXPECT_NEAR(H_isb(0, 0), gtsam::gnss::C_LIGHT, 1.0e-6);
}

TEST(FGOGtsamClockStateParityTest, PublicClockOutputConvertsMeterStateOnce) {
    using libgnss::fgo_gtsam_internal::publicClockSeconds;
    constexpr double clock_state_m = 123.4;
    EXPECT_DOUBLE_EQ(publicClockSeconds(clock_state_m, true),
                     clock_state_m / gtsam::gnss::C_LIGHT);
    EXPECT_DOUBLE_EQ(publicClockSeconds(clock_state_m, false), clock_state_m);
}

TEST(FGOGtsamSourceClockC0DFactorTest,
     EdgeGateUsesExactPhoneAndClockJumpGapReasons) {
    using libgnss::fgo_gtsam_internal::NativeSourceClockC0DSkipReason;
    using libgnss::fgo_gtsam_internal::nativeSourceClockC0DEdgeDecision;

    EXPECT_TRUE(nativeSourceClockC0DEdgeDecision(1.0, false, "pixel5").eligible);
    EXPECT_TRUE(nativeSourceClockC0DEdgeDecision(1.0, false, "pixel50").eligible);
    EXPECT_EQ(nativeSourceClockC0DEdgeDecision(
                  0.0, false, "pixel5").reason,
              NativeSourceClockC0DSkipReason::InvalidDt);
    EXPECT_EQ(nativeSourceClockC0DEdgeDecision(
                  std::numeric_limits<double>::quiet_NaN(), false, "pixel5")
                  .reason,
              NativeSourceClockC0DSkipReason::InvalidDt);
    EXPECT_EQ(nativeSourceClockC0DEdgeDecision(
                  1.5, false, "pixel5").reason,
              NativeSourceClockC0DSkipReason::Gap);
    EXPECT_EQ(nativeSourceClockC0DEdgeDecision(
                  1.0, false, "sm-a205u").reason,
              NativeSourceClockC0DSkipReason::PhoneExcluded);
    EXPECT_EQ(nativeSourceClockC0DEdgeDecision(
                  1.0, true, "pixel5").reason,
              NativeSourceClockC0DSkipReason::ClockJump);
}

TEST(FGOGtsamSourceClockC0DFactorTest,
     CandidateConfigurationIsDefaultOffAndConflictsFailClosed) {
    using libgnss::fgo_gtsam_internal::
        nativeSourceClockC0DBackendConfigurationAllowed;
    FGOProcessor::FGOConfig config;
    EXPECT_FALSE(config.use_native_source_clock_c0d_factor);
    EXPECT_FALSE(config.use_native_source_clock_c0d_meter_state_parity);
    EXPECT_FALSE(config
                     .use_native_source_clock_c0d_phase99_main_multifrontal_qr_solver);
    EXPECT_TRUE(nativeSourceClockC0DBackendConfigurationAllowed(config));

    config.use_native_source_clock_c0d_factor = true;
    EXPECT_FALSE(nativeSourceClockC0DBackendConfigurationAllowed(config));
    config.use_pose3_state = true;
    config.use_imu = true;
    config.use_upstream_observable_quality = true;
    config.use_undifferenced_doppler_factors = true;
    EXPECT_FALSE(nativeSourceClockC0DBackendConfigurationAllowed(config));
    config.native_source_clock_c0d_phone = "pixel5";
    EXPECT_TRUE(nativeSourceClockC0DBackendConfigurationAllowed(config));
    config.use_native_pdc_state_bridge = true;
    EXPECT_FALSE(nativeSourceClockC0DBackendConfigurationAllowed(config));
    config.use_native_pdc_state_bridge = false;
    config.use_fixed_lag_smoother = true;
    EXPECT_FALSE(nativeSourceClockC0DBackendConfigurationAllowed(config));

    // Phase93 admits exactly the separate GNSS-first Point3/velocity graph;
    // the selector is still default-off and the legacy Pose3+IMU contract
    // above remains the only valid path when it is absent.
    FGOProcessor::FGOConfig phase93;
    phase93.use_native_source_clock_c0d_factor = true;
    phase93.use_native_source_clock_c0d_meter_state_parity = true;
    phase93.use_native_source_clock_c0d_gnss_first_meter_state_handoff = true;
    phase93.use_pose3_state = false;
    phase93.use_imu = false;
    phase93.use_velocity_states = true;
    phase93.use_upstream_observable_quality = true;
    phase93.use_undifferenced_doppler_factors = true;
    phase93.native_source_clock_c0d_phone = "pixel5";
    EXPECT_TRUE(nativeSourceClockC0DBackendConfigurationAllowed(phase93));
    phase93.use_native_source_clock_c0d_gnss_first_meter_state_handoff = false;
    EXPECT_FALSE(nativeSourceClockC0DBackendConfigurationAllowed(phase93));
}

TEST(FGOGtsamSourceClockC0DPhase91Test,
     RawDriftInitializerRejectsMissingAndNonfiniteCoverage) {
    using libgnss::source_clock_c0d::RawDriftDInitializationReport;
    using libgnss::source_clock_c0d::validateAndCopyRawDriftD;

    std::vector<double> initialized;
    RawDriftDInitializationReport report;
    EXPECT_FALSE(validateAndCopyRawDriftD({}, initialized, report));
    EXPECT_FALSE(report.valid);
    EXPECT_EQ(report.epoch_count, 0U);
    EXPECT_EQ(report.nonfinite_count, 0U);
    EXPECT_TRUE(initialized.empty());

    const std::vector<double> missing = {0.25,
                                         std::numeric_limits<double>::quiet_NaN(),
                                         0.35};
    EXPECT_FALSE(validateAndCopyRawDriftD(missing, initialized, report));
    EXPECT_FALSE(report.valid);
    EXPECT_EQ(report.epoch_count, 3U);
    EXPECT_EQ(report.finite_count, 2U);
    EXPECT_EQ(report.nonfinite_count, 1U);
    EXPECT_TRUE(initialized.empty());

    const std::vector<double> finite = {0.25, 0.30, 0.35};
    EXPECT_TRUE(validateAndCopyRawDriftD(finite, initialized, report));
    EXPECT_TRUE(report.valid);
    EXPECT_EQ(report.finite_count, finite.size());
    EXPECT_EQ(report.nonfinite_count, 0U);
    EXPECT_EQ(initialized, finite);
    EXPECT_DOUBLE_EQ(report.min_mps, 0.25);
    EXPECT_DOUBLE_EQ(report.max_mps, 0.35);
}

TEST(FGOGtsamSourceClockC0DPhase91Test,
     ExactEpochIdentityRejectsMisalignmentAndAcceptsRawCoverage) {
    using libgnss::source_clock_c0d::ExactEpochAlignmentReport;
    using libgnss::source_clock_c0d::validateExactEpochIdentity;
    const std::vector<libgnss::GNSSTime> expected = {
        {2300, 100000.0}, {2300, 100001.0}, {2300, 100002.0}};
    const std::vector<std::int64_t> utc = {1610000000000LL, 1610000001000LL,
                                           1610000002000LL};
    ExactEpochAlignmentReport report;
    EXPECT_TRUE(validateExactEpochIdentity(
        expected, expected, expected, expected, utc, report));
    EXPECT_TRUE(report.valid);
    EXPECT_EQ(report.aligned_epoch_count, expected.size());

    auto shifted = expected;
    shifted[1].tow += 1.0e-7;
    EXPECT_FALSE(validateExactEpochIdentity(
        expected, shifted, expected, expected, utc, report));
    EXPECT_EQ(report.gnss_first_time_mismatch_count, 1U);
    EXPECT_FALSE(report.failure.empty());

    auto duplicate_keys = utc;
    duplicate_keys[2] = duplicate_keys[1];
    EXPECT_FALSE(validateExactEpochIdentity(
        expected, expected, expected, expected, duplicate_keys, report));
    EXPECT_EQ(report.duplicate_raw_utc_key_count, 1U);
    EXPECT_EQ(report.raw_utc_key_order_mismatch_count, 1U);
}

TEST(FGOGtsamSourceClockC0DPhase92Test,
     RetainedRawAlignmentUsesExplicitSourceKeysAndAllowsFilteredRows) {
    using libgnss::source_clock_c0d::ExactEpochAlignmentReport;
    using libgnss::source_clock_c0d::RetainedRawEpochKey;
    using libgnss::source_clock_c0d::validateRetainedRawDAlignment;

    const std::vector<libgnss::GNSSTime> raw_times = {
        {2300, 100000.0}, {2300, 100001.0}, {2300, 100002.0},
        {2300, 100003.0}};
    const std::vector<std::int64_t> raw_utc = {
        1610000000000LL, 1610000001000LL, 1610000002000LL,
        1610000003000LL};
    const std::vector<double> raw_drift = {0.10, 0.20, 0.30, 0.40};
    const std::vector<RetainedRawEpochKey> retained = {
        {0U, raw_utc[0], raw_times[0]},
        {2U, raw_utc[2], raw_times[2]},
        {3U, raw_utc[3], raw_times[3]}};
    ExactEpochAlignmentReport report;
    EXPECT_TRUE(validateRetainedRawDAlignment(
        retained, retained,
        {raw_times[0], raw_times[2], raw_times[3]}, raw_times, raw_utc,
        raw_drift, report));
    EXPECT_TRUE(report.valid);
    EXPECT_EQ(report.aligned_epoch_count, 3U);
    EXPECT_EQ(report.epoch_count_mismatch_count, 0U);
    EXPECT_EQ(report.retained_source_index_mismatch_count, 0U);
    EXPECT_EQ(report.retained_source_order_mismatch_count, 0U);
    EXPECT_EQ(report.raw_drift_count_mismatch_count, 0U);

    auto reordered = retained;
    std::swap(reordered[0], reordered[1]);
    EXPECT_FALSE(validateRetainedRawDAlignment(
        reordered, reordered,
        {raw_times[2], raw_times[0], raw_times[3]}, raw_times, raw_utc,
        raw_drift, report));
    EXPECT_GT(report.retained_source_order_mismatch_count, 0U);

    auto duplicate = retained;
    duplicate[1].raw_source_index = duplicate[0].raw_source_index;
    duplicate[1].raw_utc_time_millis = duplicate[0].raw_utc_time_millis;
    duplicate[1].time = duplicate[0].time;
    EXPECT_FALSE(validateRetainedRawDAlignment(
        duplicate, duplicate,
        {raw_times[0], raw_times[0], raw_times[3]}, raw_times, raw_utc,
        raw_drift, report));
    EXPECT_GT(report.duplicate_retained_source_index_count, 0U);

    auto wrong_utc = retained;
    wrong_utc[1].raw_utc_time_millis += 1;
    EXPECT_FALSE(validateRetainedRawDAlignment(
        wrong_utc, wrong_utc,
        {raw_times[0], raw_times[2], raw_times[3]}, raw_times, raw_utc,
        raw_drift, report));
    EXPECT_GT(report.retained_raw_key_mismatch_count, 0U);

    auto wrong_week = retained;
    wrong_week[2].time.week += 1;
    EXPECT_FALSE(validateRetainedRawDAlignment(
        wrong_week, wrong_week,
        {raw_times[0], raw_times[2], wrong_week[2].time}, raw_times, raw_utc,
        raw_drift, report));
    EXPECT_GT(report.retained_raw_key_mismatch_count, 0U);

    EXPECT_FALSE(validateRetainedRawDAlignment(
        retained, retained,
        {raw_times[0], raw_times[2], raw_times[3]}, raw_times, raw_utc,
        {}, report));
    EXPECT_EQ(report.raw_drift_count_mismatch_count, 1U);

    auto nonfinite_drift = raw_drift;
    nonfinite_drift[2] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(validateRetainedRawDAlignment(
        retained, retained,
        {raw_times[0], raw_times[2], raw_times[3]}, raw_times, raw_utc,
        nonfinite_drift, report));
    EXPECT_EQ(report.raw_drift_nonfinite_count, 1U);
}

TEST(FGOGtsamSourceClockC0DFactorTest,
     BatchBackendReplacesLegacyClockRowsAndReportsEdgeAccounting) {
    FGOProcessor::FGOProblem problem;
    const auto satellites = gtsamParitySatelliteGeometry();
    const Vector3d receiver(1113194.0, -4841695.0, 3985350.0);
    double lat = 0.0;
    double lon = 0.0;
    double height = 0.0;
    ecef2geodetic(receiver, lat, lon, height);
    problem.imu.valid = true;
    problem.imu.nav_origin_ecef = receiver;
    problem.imu.nav_origin_lat_rad = lat;
    problem.imu.nav_origin_lon_rad = lon;
    problem.imu.init_attitude_body_to_nav = Matrix3d::Identity();

    const GNSSTime t0(2300, 100000.0);
    for (std::size_t epoch = 0; epoch < 2; ++epoch) {
        FGOProcessor::EpochSeed seed;
        seed.time = t0 + static_cast<double>(epoch);
        seed.position_ecef = receiver;
        seed.receiver_clock_drift_mps = 0.25 + 0.10 * static_cast<double>(epoch);
        problem.epochs.push_back(seed);
        for (std::size_t sat = 0; sat < satellites.size(); ++sat) {
            FGOProcessor::PseudorangeFactor factor;
            factor.epoch_index = epoch;
            factor.satellite = SatelliteId(
                GNSSSystem::GPS, static_cast<uint8_t>(sat + 1));
            factor.signal = SignalType::GPS_L1CA;
            factor.clock_group = GNSSSystem::GPS;
            factor.satellite_position_ecef = satellites[sat];
            factor.corrected_pseudorange_m =
                (satellites[sat] - receiver).norm();
            factor.sigma_m = 1.0;
            problem.pseudorange_factors.push_back(factor);
        }
    }

    FGOProcessor::FGOConfig config;
    config.backend = FGOBackend::GTSAM;
    config.use_pose3_state = true;
    config.use_imu = true;
    config.use_upstream_observable_quality = true;
    config.use_undifferenced_doppler_factors = true;
    config.use_native_source_clock_c0d_factor = true;
    config.native_source_clock_c0d_phone = "pixel5";
    config.use_native_source_clock_c0d_active_solve_diagnostic = true;
    config.use_native_source_clock_c0d_raw_drift_d_initializer = true;
    config.use_robust_loss = false;
    config.max_iterations = 1;

    const FGOProcessor processor(config);
    const FGOProcessor::FGOResult result = processor.optimizeProblem(problem);

    EXPECT_TRUE(result.diagnostics.native_source_clock_c0d_factor_enabled);
    EXPECT_TRUE(result.diagnostics
                    .native_source_clock_c0d_raw_drift_d_initializer_enabled);
    EXPECT_TRUE(result.diagnostics
                    .native_source_clock_c0d_raw_drift_d_initializer_attempted);
    EXPECT_TRUE(result.diagnostics
                    .native_source_clock_c0d_raw_drift_d_initializer_coverage_valid);
    EXPECT_EQ(result.diagnostics
                  .native_source_clock_c0d_raw_drift_d_initializer_epoch_count,
              2U);
    EXPECT_EQ(result.diagnostics
                  .native_source_clock_c0d_raw_drift_d_initializer_finite_count,
              2U);
    EXPECT_EQ(result.diagnostics
                  .native_source_clock_c0d_raw_drift_d_initializer_nonfinite_count,
              0U);
    EXPECT_DOUBLE_EQ(result.diagnostics
                         .native_source_clock_c0d_raw_drift_d_initializer_min_mps,
                     0.25);
    EXPECT_DOUBLE_EQ(result.diagnostics
                         .native_source_clock_c0d_raw_drift_d_initializer_max_mps,
                     0.35);
    EXPECT_EQ(result.diagnostics.native_source_clock_c0d_factor_count, 1U);
    EXPECT_EQ(result.diagnostics.native_source_clock_c0d_clock_jump_skips, 0U);
    EXPECT_EQ(result.diagnostics.native_source_clock_c0d_gap_skips, 0U);
    EXPECT_EQ(result.diagnostics.native_source_clock_c0d_invalid_dt_skips, 0U);
    EXPECT_EQ(result.diagnostics.native_source_clock_c0d_phone_exclusion_skips,
              0U);
    EXPECT_DOUBLE_EQ(result.diagnostics.native_source_clock_c0d_dt_min_s, 1.0);
    EXPECT_DOUBLE_EQ(result.diagnostics.native_source_clock_c0d_dt_max_s, 1.0);
    EXPECT_EQ(result.diagnostics.native_source_clock_c0d_legacy_between_factor_count,
              0U);
    EXPECT_TRUE(result.diagnostics
                    .native_source_clock_c0d_active_solve_diagnostic_enabled);
    EXPECT_TRUE(result.diagnostics.native_source_clock_c0d_active_solve_attempted);
    EXPECT_TRUE(result.diagnostics
                    .native_source_clock_c0d_active_solve_finite_costs);
    EXPECT_TRUE(result.diagnostics
                    .native_source_clock_c0d_termination_trace_complete);
    EXPECT_EQ(result.diagnostics
                  .native_source_clock_c0d_accepted_outer_iterations,
              static_cast<std::size_t>(result.diagnostics.iterations));
    EXPECT_GE(result.diagnostics
                  .native_source_clock_c0d_total_inner_lambda_attempts,
              result.diagnostics
                  .native_source_clock_c0d_accepted_outer_iterations);
    EXPECT_FALSE(result.diagnostics
                     .native_source_clock_c0d_termination_branch_reason.empty());
    EXPECT_GT(result.diagnostics
                  .native_source_clock_c0d_max_whitened_clock_column_norm,
              1.0e9);
    EXPECT_GT(result.diagnostics
                  .native_source_clock_c0d_max_whitened_drift_column_norm,
              1.0);
    EXPECT_GT(result.diagnostics.native_source_clock_c0d_conditioning_proxy,
              1.0e8);

    // Phase92 opt-in uses the same active graph and source row with metre
    // clock/ISB states.  The expected conditioning scale is now physical
    // (10 for the 0.1 m row, rather than 10*C for seconds), and the public
    // solution still exposes clock bias in seconds.
    config.use_native_source_clock_c0d_meter_state_parity = true;
    const FGOProcessor meter_processor(config);
    const FGOProcessor::FGOResult meter_result =
        meter_processor.optimizeProblem(problem);
    EXPECT_TRUE(meter_result.diagnostics
                    .native_source_clock_c0d_meter_state_parity_enabled);
    EXPECT_EQ(meter_result.diagnostics.native_source_clock_c0d_factor_count, 1U);
    EXPECT_DOUBLE_EQ(
        meter_result.diagnostics.native_source_clock_c0d_max_whitened_clock_column_norm,
        10.0);
    EXPECT_NEAR(
        meter_result.diagnostics.native_source_clock_c0d_max_whitened_drift_column_norm,
        5.0, 1.0e-12);
    EXPECT_TRUE(meter_result.diagnostics.native_source_clock_c0d_active_solve_attempted);
    EXPECT_TRUE(meter_result.diagnostics.native_source_clock_c0d_active_solve_finite_costs);
    EXPECT_GT(meter_result.diagnostics.native_source_clock_c0d_accepted_outer_iterations,
              0U);
    ASSERT_FALSE(meter_result.solution.solutions.empty());
    EXPECT_TRUE(std::isfinite(meter_result.solution.solutions.front().receiver_clock_bias));
    EXPECT_LT(std::abs(meter_result.solution.solutions.front().receiver_clock_bias),
              1.0e-3);

    // The backend-level guard is intentionally exercised after the accepted
    // solve: a missing/non-finite retained Android drift must stop before any
    // graph is constructed rather than silently reverting to D_i = 0.
    problem.epochs[1].receiver_clock_drift_mps =
        std::numeric_limits<double>::quiet_NaN();
    const FGOProcessor::FGOResult rejected = processor.optimizeProblem(problem);
    EXPECT_FALSE(rejected.diagnostics.native_source_clock_c0d_raw_drift_d_initializer_coverage_valid);
    EXPECT_EQ(rejected.diagnostics
                  .native_source_clock_c0d_raw_drift_d_initializer_epoch_count,
              2U);
    EXPECT_EQ(rejected.diagnostics
                  .native_source_clock_c0d_raw_drift_d_initializer_finite_count,
              1U);
    EXPECT_EQ(rejected.diagnostics
                  .native_source_clock_c0d_raw_drift_d_initializer_nonfinite_count,
              1U);
    EXPECT_TRUE(rejected.solution.isEmpty());
}

TEST(FGOGtsamPhase99SolverSelectorTest,
     OffKeepsHistoricalCholeskyAndOnChangesOnlySolverEnum) {
    using libgnss::fgo_gtsam_internal::selectPhase99MainSolver;

    gtsam::LevenbergMarquardtParams baseline;
    baseline.setMaxIterations(17);
    baseline.setAbsoluteErrorTol(2.0e-9);
    baseline.setRelativeErrorTol(3.0e-8);
    baseline.setDiagonalDamping(true);
    baseline.lambdaInitial = 2.5e-5;
    baseline.lambdaFactor = 7.0;
    baseline.minModelFidelity = 2.0e-3;

    auto off = baseline;
    selectPhase99MainSolver(off, false);
    EXPECT_EQ(off.linearSolverType,
              gtsam::NonlinearOptimizerParams::MULTIFRONTAL_CHOLESKY);
    EXPECT_EQ(off.getLinearSolverType(), "MULTIFRONTAL_CHOLESKY");
    EXPECT_EQ(off.getOrderingType(), baseline.getOrderingType());
    EXPECT_EQ(off.maxIterations, baseline.maxIterations);
    EXPECT_DOUBLE_EQ(off.absoluteErrorTol, baseline.absoluteErrorTol);
    EXPECT_DOUBLE_EQ(off.relativeErrorTol, baseline.relativeErrorTol);
    EXPECT_DOUBLE_EQ(off.lambdaInitial, baseline.lambdaInitial);
    EXPECT_DOUBLE_EQ(off.lambdaFactor, baseline.lambdaFactor);
    EXPECT_DOUBLE_EQ(off.lambdaLowerBound, baseline.lambdaLowerBound);
    EXPECT_DOUBLE_EQ(off.lambdaUpperBound, baseline.lambdaUpperBound);
    EXPECT_DOUBLE_EQ(off.minModelFidelity, baseline.minModelFidelity);
    EXPECT_EQ(off.useFixedLambdaFactor, baseline.useFixedLambdaFactor);
    EXPECT_EQ(off.getDiagonalDamping(), baseline.getDiagonalDamping());
    EXPECT_EQ(off.dampingParams.exactHessianDiagonal,
              baseline.dampingParams.exactHessianDiagonal);
    EXPECT_DOUBLE_EQ(off.dampingParams.minDiagonal,
                     baseline.dampingParams.minDiagonal);
    EXPECT_DOUBLE_EQ(off.dampingParams.maxDiagonal,
                     baseline.dampingParams.maxDiagonal);
    EXPECT_EQ(off.ordering.has_value(), baseline.ordering.has_value());

    auto on = baseline;
    selectPhase99MainSolver(on, true);
    EXPECT_EQ(on.linearSolverType,
              gtsam::NonlinearOptimizerParams::MULTIFRONTAL_QR);
    EXPECT_EQ(on.getLinearSolverType(), "MULTIFRONTAL_QR");
    EXPECT_EQ(on.getOrderingType(), baseline.getOrderingType());
    EXPECT_EQ(on.maxIterations, baseline.maxIterations);
    EXPECT_DOUBLE_EQ(on.absoluteErrorTol, baseline.absoluteErrorTol);
    EXPECT_DOUBLE_EQ(on.relativeErrorTol, baseline.relativeErrorTol);
    EXPECT_DOUBLE_EQ(on.lambdaInitial, baseline.lambdaInitial);
    EXPECT_DOUBLE_EQ(on.lambdaFactor, baseline.lambdaFactor);
    EXPECT_DOUBLE_EQ(on.lambdaLowerBound, baseline.lambdaLowerBound);
    EXPECT_DOUBLE_EQ(on.lambdaUpperBound, baseline.lambdaUpperBound);
    EXPECT_DOUBLE_EQ(on.minModelFidelity, baseline.minModelFidelity);
    EXPECT_EQ(on.useFixedLambdaFactor, baseline.useFixedLambdaFactor);
    EXPECT_EQ(on.getDiagonalDamping(), baseline.getDiagonalDamping());
    EXPECT_EQ(on.dampingParams.exactHessianDiagonal,
              baseline.dampingParams.exactHessianDiagonal);
    EXPECT_DOUBLE_EQ(on.dampingParams.minDiagonal,
                     baseline.dampingParams.minDiagonal);
    EXPECT_DOUBLE_EQ(on.dampingParams.maxDiagonal,
                     baseline.dampingParams.maxDiagonal);
    EXPECT_EQ(on.ordering.has_value(), baseline.ordering.has_value());
    EXPECT_EQ(libgnss::fgo_gtsam_internal::phase98SolverBranch(on),
              "multifrontal");
    EXPECT_EQ(libgnss::fgo_gtsam_internal::phase98EliminationFunction(on),
              "EliminateQR");
}

TEST(FGOGtsamPhase99SolverSelectorTest,
     RankChallengedGraphTakesStrictQrCandidateStep) {
    const gtsam::Key x_key = gtsam::Symbol('x', 0);
    const gtsam::Key y_key = gtsam::Symbol('y', 0);
    const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
    constexpr double epsilon = 1.0e-6;

    gtsam::NonlinearFactorGraph graph;
    graph.emplace_shared<NearRankCoupledFactor>(
        x_key, y_key, 1.0, 1.0, noise);
    graph.emplace_shared<NearRankCoupledFactor>(
        x_key, y_key, 1.0 + epsilon, 1.0 + epsilon, noise);

    gtsam::Values initial;
    initial.insert(x_key, 0.0);
    initial.insert(y_key, 0.0);
    const double initial_cost = graph.error(initial);

    gtsam::LevenbergMarquardtParams params;
    params.setMaxIterations(10);
    params.setAbsoluteErrorTol(0.0);
    params.setRelativeErrorTol(1.0e-12);
    libgnss::fgo_gtsam_internal::selectPhase99MainSolver(params, true);
    ASSERT_EQ(params.linearSolverType,
              gtsam::NonlinearOptimizerParams::MULTIFRONTAL_QR);

    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, params);
    const gtsam::Values optimized = optimizer.optimize();
    const double final_cost = graph.error(optimized);

    EXPECT_GT(optimizer.iterations(), 0u);
    EXPECT_TRUE(std::isfinite(final_cost));
    EXPECT_LT(final_cost, initial_cost);
    EXPECT_NEAR(optimized.at<double>(x_key) + optimized.at<double>(y_key),
                1.0, 1.0e-5);
}

TEST(FGOGtsamPhase99SolverSelectorTest,
     GnssFirstStagingRemainsCholeskyWhenSelectorIsCarried) {
    FGOProcessor::FGOProblem problem =
        makePhase93GnssFirstClockStateProblem();
    FGOProcessor::FGOConfig config;
    config.backend = FGOBackend::GTSAM;
    config.use_pose3_state = false;
    config.use_imu = false;
    config.use_velocity_states = true;
    config.use_upstream_observable_quality = true;
    config.use_undifferenced_doppler_factors = true;
    config.use_native_source_clock_c0d_factor = true;
    config.native_source_clock_c0d_phone = "pixel5";
    config.use_native_source_clock_c0d_meter_state_parity = true;
    config.use_native_source_clock_c0d_active_solve_diagnostic = true;
    config.use_native_source_clock_c0d_raw_drift_d_initializer = true;
    config.use_native_source_clock_c0d_gnss_first_meter_state_handoff = true;
    config.use_native_source_clock_c0d_phase99_main_multifrontal_qr_solver =
        true;
    config.use_robust_loss = false;
    config.max_iterations = 12;

    const FGOProcessor::FGOResult result =
        FGOProcessor(config).optimizeProblem(problem);
    EXPECT_TRUE(result.diagnostics
                    .native_source_clock_c0d_phase99_main_multifrontal_qr_solver_requested);
    EXPECT_FALSE(result.diagnostics
                     .native_source_clock_c0d_phase99_main_multifrontal_qr_solver_selected);
    EXPECT_EQ(result.diagnostics.selected_linear_solver_type,
              "MULTIFRONTAL_CHOLESKY");
    EXPECT_EQ(result.diagnostics.selected_solver_branch, "multifrontal");
    EXPECT_EQ(result.diagnostics.selected_elimination_function,
              "EliminatePreferCholesky");
    EXPECT_TRUE(result.diagnostics.converged);
    // This lane stages the raw-D initializer on the generic GNSS-first
    // velocity graph, not the Phase164 raw-P no-Doppler graph (which would
    // require the dedicated seed adapter and an empty generic Doppler
    // family). Its provenance must therefore stay false.
    EXPECT_FALSE(result.diagnostics.native_raw_p_no_doppler_graph_enabled);
}

FGOProcessor::FGOPhase143TerminationDiagnostics makePhase143TestTelemetry(
    const std::string& stage, std::size_t configured_max,
    std::size_t attempted, std::size_t accepted, std::size_t rejected,
    const std::string& branch) {
    FGOProcessor::FGOPhase143TerminationDiagnostics diagnostics;
    diagnostics.selector_enabled = true;
    diagnostics.stage = stage;
    diagnostics.configured_max_iterations = configured_max;
    diagnostics.effective_max_iterations = 1000;
    diagnostics.attempted = true;
    diagnostics.attempted_outer_iterations = attempted;
    diagnostics.accepted_outer_iterations = accepted;
    diagnostics.rejected_outer_iterations = rejected;
    diagnostics.total_inner_lambda_attempts = attempted;
    diagnostics.initial_cost = 10.0;
    diagnostics.final_cost = 5.0;
    diagnostics.costs_finite = true;
    diagnostics.strict_cost_decrease = true;
    diagnostics.termination_branch = branch;
    diagnostics.relative_error_tolerance = 1.0e-8;
    diagnostics.absolute_error_tolerance = 1.0e-10;
    diagnostics.error_tolerance = 0.0;
    diagnostics.initial_lambda = 1.0e-5;
    diagnostics.final_lambda = 1.0e-5;
    diagnostics.maximum_lambda = 1.0e-5;
    diagnostics.lambda_factor = 10.0;
    diagnostics.lambda_lower_bound = 0.0;
    diagnostics.lambda_upper_bound = 1.0e5;
    diagnostics.min_model_fidelity = 1.0e-3;
    diagnostics.diagonal_damping = false;
    diagnostics.use_fixed_lambda_factor = true;
    diagnostics.linear_solver = stage == "main" ? "MULTIFRONTAL_QR"
                                                  : "MULTIFRONTAL_CHOLESKY";
    diagnostics.elimination = stage == "main" ? "EliminateQR"
                                                : "EliminatePreferCholesky";
    diagnostics.ordering_type = "COLAMD";
    diagnostics.explicit_ordering_present = false;
    diagnostics.no_fallback = true;
    diagnostics.termination_trace_complete = true;
    return diagnostics;
}

TEST(FGOGtsamPhase143TerminationTest,
     SelectorChangesOnlyMainBudgetAndLeavesGtsamParametersUntouched) {
    using libgnss::fgo_gtsam_internal::phase143EffectiveMaxIterations;

    gtsam::LevenbergMarquardtParams baseline;
    baseline.setMaxIterations(12);
    baseline.setAbsoluteErrorTol(2.0e-9);
    baseline.setRelativeErrorTol(3.0e-8);
    baseline.setErrorTol(0.0);
    baseline.lambdaInitial = 2.5e-5;
    baseline.lambdaFactor = 7.0;
    baseline.lambdaLowerBound = 0.0;
    baseline.lambdaUpperBound = 8.0e4;
    baseline.minModelFidelity = 2.0e-3;
    baseline.setDiagonalDamping(false);
    baseline.useFixedLambdaFactor = true;

    const auto off = baseline;
    auto on = baseline;
    on.setMaxIterations(phase143EffectiveMaxIterations(
        static_cast<int>(baseline.getMaxIterations()), true, true));
    EXPECT_EQ(off.getMaxIterations(), 12U);
    EXPECT_EQ(on.getMaxIterations(), 1000U);
    EXPECT_DOUBLE_EQ(on.absoluteErrorTol, off.absoluteErrorTol);
    EXPECT_DOUBLE_EQ(on.relativeErrorTol, off.relativeErrorTol);
    EXPECT_DOUBLE_EQ(on.errorTol, off.errorTol);
    EXPECT_DOUBLE_EQ(on.lambdaInitial, off.lambdaInitial);
    EXPECT_DOUBLE_EQ(on.lambdaFactor, off.lambdaFactor);
    EXPECT_DOUBLE_EQ(on.lambdaLowerBound, off.lambdaLowerBound);
    EXPECT_DOUBLE_EQ(on.lambdaUpperBound, off.lambdaUpperBound);
    EXPECT_DOUBLE_EQ(on.minModelFidelity, off.minModelFidelity);
    EXPECT_EQ(on.getDiagonalDamping(), off.getDiagonalDamping());
    EXPECT_EQ(on.useFixedLambdaFactor, off.useFixedLambdaFactor);

    EXPECT_EQ(phase143EffectiveMaxIterations(12, false, true), 12);
    EXPECT_EQ(phase143EffectiveMaxIterations(12, true, true), 1000);
    // The selector is carried into GNSS-first only as an explicit no-op.
    EXPECT_EQ(phase143EffectiveMaxIterations(1000, true, false), 1000);
}

TEST(FGOGtsamPhase143TerminationTest,
     AcceptsEarlyConvergenceCapAndTerminalRejectedOuterAttempt) {
    using libgnss::fgo_gtsam_internal::validatePhase143TerminationDiagnostics;

    const auto early = makePhase143TestTelemetry(
        "main", 1000, 4, 4, 0, "outer_convergence_tolerance");
    EXPECT_TRUE(validatePhase143TerminationDiagnostics(early));

    const auto cap = makePhase143TestTelemetry(
        "main", 12, 1000, 1000, 0, "maximum_outer_iterations");
    EXPECT_TRUE(validatePhase143TerminationDiagnostics(cap));

    const auto rejected = makePhase143TestTelemetry(
        "main", 1000, 5, 4, 1, "small_cost_change");
    EXPECT_TRUE(validatePhase143TerminationDiagnostics(rejected));

    const auto gnss_first = makePhase143TestTelemetry(
        "gnss-first", 1000, 4, 4, 0, "outer_convergence_tolerance");
    EXPECT_TRUE(validatePhase143TerminationDiagnostics(gnss_first));
}

TEST(FGOGtsamPhase143TerminationTest,
     RejectsMissingConflictingOrNonfiniteAuthoritativeFields) {
    using libgnss::fgo_gtsam_internal::validatePhase143TerminationDiagnostics;
    auto diagnostics = makePhase143TestTelemetry(
        "main", 12, 1000, 1000, 0, "maximum_outer_iterations");
    EXPECT_TRUE(validatePhase143TerminationDiagnostics(diagnostics));

    diagnostics.termination_branch.clear();
    EXPECT_FALSE(validatePhase143TerminationDiagnostics(diagnostics));
    diagnostics.termination_branch = "maximum_outer_iterations";
    diagnostics.rejected_outer_iterations = 1;
    EXPECT_FALSE(validatePhase143TerminationDiagnostics(diagnostics));
    diagnostics.rejected_outer_iterations = 0;
    diagnostics.final_cost = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(validatePhase143TerminationDiagnostics(diagnostics));
    diagnostics.final_cost = 5.0;
    diagnostics.no_fallback = false;
    EXPECT_FALSE(validatePhase143TerminationDiagnostics(diagnostics));
    diagnostics.no_fallback = true;
    diagnostics.termination_trace_complete = false;
    EXPECT_FALSE(validatePhase143TerminationDiagnostics(diagnostics));
}

TEST(FGOGtsamPhase143TerminationTest,
     BackendPublishesAuthoritativeMainTerminationSidecar) {
    using libgnss::fgo_gtsam_internal::validatePhase143TerminationDiagnostics;
    FGOProcessor::FGOProblem problem =
        makePhase93GnssFirstClockStateProblem();
    const Vector3d receiver = problem.epochs.front().position_ecef;
    double lat = 0.0;
    double lon = 0.0;
    double height = 0.0;
    ecef2geodetic(receiver, lat, lon, height);
    problem.imu.valid = true;
    problem.imu.nav_origin_ecef = receiver;
    problem.imu.nav_origin_lat_rad = lat;
    problem.imu.nav_origin_lon_rad = lon;
    problem.imu.init_attitude_body_to_nav = Matrix3d::Identity();
    problem.native_source_clock_c0d_gnss_first_d_handoff_mps = {0.25, 0.35};
    FGOProcessor::EpochClockBiasComponentsM c0{};
    FGOProcessor::EpochClockBiasComponentsM c1{};
    c0[0] = 12.0;
    c0[2] = 2.0;
    c1[0] = 13.0;
    c1[2] = 2.0;
    problem.native_source_clock_c0d_gnss_first_c_handoff_m = {c0, c1};

    FGOProcessor::FGOConfig config;
    config.backend = FGOBackend::GTSAM;
    config.use_pose3_state = true;
    config.use_imu = true;
    config.use_upstream_observable_quality = true;
    config.use_undifferenced_doppler_factors = true;
    config.use_native_source_clock_c0d_factor = true;
    config.native_source_clock_c0d_phone = "pixel5";
    config.use_native_source_clock_c0d_meter_state_parity = true;
    config.use_native_source_clock_c0d_epoch_vector_parity = true;
    config.use_native_source_clock_c0d_gnss_first_meter_state_handoff = true;
    config.use_native_source_clock_c0d_active_solve_diagnostic = true;
    config.use_native_source_clock_c0d_phase99_main_multifrontal_qr_solver =
        true;
    config.use_native_phase143_official_main_lm_termination_budget = true;
    config.use_inter_system_biases = false;
    config.use_robust_loss = false;
    config.max_iterations = 12;

    const FGOProcessor::FGOResult result =
        FGOProcessor(config).optimizeProblem(problem);
    const auto& termination =
        result.diagnostics.native_phase143_termination;
    EXPECT_TRUE(result.diagnostics.converged);
    EXPECT_TRUE(termination.selector_enabled);
    EXPECT_EQ(termination.stage, "main");
    EXPECT_EQ(termination.configured_max_iterations, 12U);
    EXPECT_EQ(termination.effective_max_iterations, 1000U);
    EXPECT_TRUE(termination.attempted);
    EXPECT_GT(termination.accepted_outer_iterations, 0U);
    EXPECT_EQ(termination.rejected_outer_iterations,
              termination.attempted_outer_iterations -
                  termination.accepted_outer_iterations);
    EXPECT_TRUE(termination.costs_finite);
    EXPECT_TRUE(termination.strict_cost_decrease);
    EXPECT_TRUE(validatePhase143TerminationDiagnostics(termination));
    EXPECT_EQ(termination.linear_solver, "MULTIFRONTAL_QR");
    EXPECT_EQ(termination.elimination, "EliminateQR");
    EXPECT_EQ(termination.ordering_type, "COLAMD");
    EXPECT_TRUE(termination.no_fallback);
}

TEST(FGOGtsamPhase143TerminationTest,
     SummaryTraceIsAuthoritativeWithoutLegacyPhase96Diagnostics) {
    // The application enables the existing active-solve summary surface but
    // deliberately does not enable the older Phase96/97/98 diagnostic
    // selectors.  Exercise that exact boundary so Phase143 cannot silently
    // depend on TRY-LAMBDA parsing from another experiment.
    FGOProcessor::FGOProblem problem =
        makePhase93GnssFirstClockStateProblem();
    const Vector3d receiver = problem.epochs.front().position_ecef;
    double lat = 0.0;
    double lon = 0.0;
    double height = 0.0;
    ecef2geodetic(receiver, lat, lon, height);
    problem.imu.valid = true;
    problem.imu.nav_origin_ecef = receiver;
    problem.imu.nav_origin_lat_rad = lat;
    problem.imu.nav_origin_lon_rad = lon;
    problem.imu.init_attitude_body_to_nav = Matrix3d::Identity();
    problem.native_source_clock_c0d_gnss_first_d_handoff_mps = {0.25, 0.35};
    FGOProcessor::EpochClockBiasComponentsM c0{};
    FGOProcessor::EpochClockBiasComponentsM c1{};
    c0[0] = 12.0;
    c0[2] = 2.0;
    c1[0] = 13.0;
    c1[2] = 2.0;
    problem.native_source_clock_c0d_gnss_first_c_handoff_m = {c0, c1};

    FGOProcessor::FGOConfig config;
    config.backend = FGOBackend::GTSAM;
    config.use_pose3_state = true;
    config.use_imu = true;
    config.use_upstream_observable_quality = true;
    config.use_undifferenced_doppler_factors = true;
    config.use_native_source_clock_c0d_factor = true;
    config.native_source_clock_c0d_phone = "pixel5";
    config.use_native_source_clock_c0d_meter_state_parity = true;
    config.use_native_source_clock_c0d_epoch_vector_parity = true;
    config.use_native_source_clock_c0d_gnss_first_meter_state_handoff = true;
    config.use_native_source_clock_c0d_active_solve_diagnostic = true;
    config.use_native_source_clock_c0d_phase99_main_multifrontal_qr_solver =
        true;
    config.use_native_phase143_official_main_lm_termination_budget = true;
    config.use_inter_system_biases = false;
    config.use_robust_loss = false;
    config.max_iterations = 12;

    const FGOProcessor::FGOResult result =
        FGOProcessor(config).optimizeProblem(problem);
    const auto& termination = result.diagnostics.native_phase143_termination;
    EXPECT_TRUE(result.diagnostics.converged);
    EXPECT_TRUE(termination.configuration_valid);
    EXPECT_TRUE(termination.termination_trace_complete);
    EXPECT_GT(termination.total_inner_lambda_attempts, 0U);
    EXPECT_GT(termination.attempted_outer_iterations, 0U);
    EXPECT_EQ(termination.rejected_outer_iterations,
              termination.attempted_outer_iterations -
                  termination.accepted_outer_iterations);
    EXPECT_TRUE(
        fgo_gtsam_internal::validatePhase143TerminationDiagnostics(termination));
}

TEST(FGOGtsamPhase143TerminationTest,
     GnssFirstScopeKeepsItsExistingOfficialThousandIterationBudget) {
    FGOProcessor::FGOProblem problem =
        makePhase93GnssFirstClockStateProblem();
    FGOProcessor::FGOConfig config;
    config.backend = FGOBackend::GTSAM;
    config.use_pose3_state = false;
    config.use_imu = false;
    config.use_velocity_states = true;
    config.use_upstream_observable_quality = true;
    config.use_undifferenced_doppler_factors = true;
    config.use_native_source_clock_c0d_factor = true;
    config.native_source_clock_c0d_phone = "pixel5";
    config.use_native_source_clock_c0d_meter_state_parity = true;
    config.use_native_source_clock_c0d_raw_drift_d_initializer = true;
    config.use_native_source_clock_c0d_gnss_first_meter_state_handoff = true;
    config.use_native_source_clock_c0d_active_solve_diagnostic = true;
    config.use_native_phase143_official_main_lm_termination_budget = true;
    config.use_robust_loss = false;
    config.max_iterations = 1000;

    const FGOProcessor::FGOResult result =
        FGOProcessor(config).optimizeProblem(problem);
    const auto& termination = result.diagnostics.native_phase143_termination;
    EXPECT_TRUE(result.diagnostics.converged);
    EXPECT_TRUE(termination.selector_enabled);
    EXPECT_EQ(termination.stage, "gnss-first");
    EXPECT_EQ(termination.configured_max_iterations, 1000U);
    EXPECT_EQ(termination.effective_max_iterations, 1000U);
    EXPECT_TRUE(termination.configuration_valid);
    EXPECT_TRUE(termination.termination_trace_complete);
    EXPECT_GT(termination.accepted_outer_iterations, 0U);
    EXPECT_EQ(termination.linear_solver, "MULTIFRONTAL_CHOLESKY");
    EXPECT_EQ(termination.elimination, "EliminatePreferCholesky");
    EXPECT_TRUE(
        fgo_gtsam_internal::validatePhase143TerminationDiagnostics(termination));
}

TEST(FGOGtsamSourceClockC0DPhase93Test,
     GnssFirstPoint3VelocityUsesMeterC0DAndExportsOptimizedDrift) {
    FGOProcessor::FGOProblem problem =
        makePhase93GnssFirstClockStateProblem();
    FGOProcessor::FGOConfig config;
    config.backend = FGOBackend::GTSAM;
    config.use_pose3_state = false;
    config.use_imu = false;
    config.use_velocity_states = true;
    config.use_upstream_observable_quality = true;
    config.use_undifferenced_doppler_factors = true;
    config.use_native_source_clock_c0d_factor = true;
    config.native_source_clock_c0d_phone = "pixel5";
    config.use_native_source_clock_c0d_meter_state_parity = true;
    config.use_native_source_clock_c0d_active_solve_diagnostic = true;
    config.use_native_source_clock_c0d_raw_drift_d_initializer = true;
    config.use_native_source_clock_c0d_gnss_first_meter_state_handoff = true;
    config.use_robust_loss = false;
    config.max_iterations = 12;

    const FGOProcessor processor(config);
    const FGOProcessor::FGOResult result = processor.optimizeProblem(problem);

    EXPECT_TRUE(result.diagnostics.converged);
    EXPECT_TRUE(result.diagnostics.native_source_clock_c0d_factor_enabled);
    EXPECT_TRUE(result.diagnostics.native_source_clock_c0d_meter_state_parity_enabled);
    EXPECT_TRUE(result.diagnostics
                    .native_source_clock_c0d_raw_drift_d_initializer_coverage_valid);
    EXPECT_EQ(result.diagnostics.native_source_clock_c0d_factor_count, 1U);
    EXPECT_EQ(result.diagnostics.undifferenced_doppler_factors_inserted,
              problem.undifferenced_doppler_factors.size());
    EXPECT_GT(result.diagnostics.native_source_clock_c0d_accepted_outer_iterations,
              0U);
    EXPECT_TRUE(std::isfinite(result.diagnostics.initial_cost));
    EXPECT_TRUE(std::isfinite(result.diagnostics.final_cost));
    EXPECT_LT(result.diagnostics.final_cost, result.diagnostics.initial_cost);
    ASSERT_EQ(result.solution.solutions.size(), problem.epochs.size());
    for (const auto& solution : result.solution.solutions) {
        EXPECT_TRUE(solution.position_ecef.allFinite());
        EXPECT_GE(solution.position_ecef.norm(), 6.0e6);
        EXPECT_LE(solution.position_ecef.norm(), 7.0e6);
        EXPECT_TRUE(std::isfinite(solution.receiver_clock_bias));
    }
    ASSERT_EQ(result.epoch_velocities_ecef_mps.size(), problem.epochs.size());
    for (const auto& velocity : result.epoch_velocities_ecef_mps) {
        EXPECT_TRUE(velocity.allFinite());
    }
    ASSERT_EQ(result.epoch_clock_drift_mps.size(), problem.epochs.size());
    for (const double drift : result.epoch_clock_drift_mps) {
        EXPECT_TRUE(std::isfinite(drift));
    }
}

TEST(FGOGtsamSourceClockC0DPhase93Test,
     MainGraphRequiresOptimizedDVectorAndDoesNotUseRawFallback) {
    FGOProcessor::FGOProblem problem =
        makePhase93GnssFirstClockStateProblem();
    double lat = 0.0;
    double lon = 0.0;
    double height = 0.0;
    ecef2geodetic(problem.epochs.front().position_ecef, lat, lon, height);
    problem.imu.valid = true;
    problem.imu.nav_origin_ecef = problem.epochs.front().position_ecef;
    problem.imu.nav_origin_lat_rad = lat;
    problem.imu.nav_origin_lon_rad = lon;
    problem.imu.init_attitude_body_to_nav = Matrix3d::Identity();
    // Deliberately make the raw EpochSeed D unavailable.  The Phase93 main
    // graph must still be able to initialize from the explicit optimized-D
    // handoff vector, proving it does not silently fall back to raw/zero D.
    for (auto& epoch : problem.epochs) {
        epoch.receiver_clock_drift_mps =
            std::numeric_limits<double>::quiet_NaN();
    }
    problem.native_source_clock_c0d_gnss_first_d_handoff_mps = {0.40, 0.45};

    FGOProcessor::FGOConfig config;
    config.backend = FGOBackend::GTSAM;
    config.use_pose3_state = true;
    config.use_imu = true;
    config.use_upstream_observable_quality = true;
    config.use_undifferenced_doppler_factors = true;
    config.use_native_source_clock_c0d_factor = true;
    config.native_source_clock_c0d_phone = "pixel5";
    config.use_native_source_clock_c0d_meter_state_parity = true;
    config.use_native_source_clock_c0d_active_solve_diagnostic = true;
    config.use_native_source_clock_c0d_gnss_first_meter_state_handoff = true;
    config.use_robust_loss = false;
    config.max_iterations = 12;
    config.use_native_source_clock_c0d_phase99_main_multifrontal_qr_solver =
        false;
    const FGOProcessor::FGOResult legacy_main_result =
        FGOProcessor(config).optimizeProblem(problem);
    EXPECT_TRUE(legacy_main_result.diagnostics.converged);
    EXPECT_FALSE(legacy_main_result.diagnostics
                     .native_source_clock_c0d_phase99_main_multifrontal_qr_solver_requested);
    EXPECT_FALSE(legacy_main_result.diagnostics
                     .native_source_clock_c0d_phase99_main_multifrontal_qr_solver_selected);
    EXPECT_EQ(legacy_main_result.diagnostics.selected_linear_solver_type,
              "MULTIFRONTAL_CHOLESKY");
    EXPECT_EQ(legacy_main_result.diagnostics.selected_elimination_function,
              "EliminatePreferCholesky");
    config.use_native_source_clock_c0d_phase99_main_multifrontal_qr_solver =
        true;

    const FGOProcessor processor(config);
    const FGOProcessor::FGOResult result = processor.optimizeProblem(problem);

    EXPECT_TRUE(result.diagnostics.converged);
    EXPECT_TRUE(result.diagnostics
                    .native_source_clock_c0d_phase99_main_multifrontal_qr_solver_requested);
    EXPECT_TRUE(result.diagnostics
                    .native_source_clock_c0d_phase99_main_multifrontal_qr_solver_selected);
    EXPECT_EQ(result.diagnostics.selected_linear_solver_type,
              "MULTIFRONTAL_QR");
    EXPECT_EQ(result.diagnostics.selected_solver_branch, "multifrontal");
    EXPECT_EQ(result.diagnostics.selected_elimination_function,
              "EliminateQR");
    ASSERT_EQ(result.epoch_clock_drift_mps.size(), problem.epochs.size());
    for (const double drift : result.epoch_clock_drift_mps) {
        EXPECT_TRUE(std::isfinite(drift));
    }

    // Removing one optimized handoff entry must fail closed before graph
    // construction; raw EpochSeed D is NaN and cannot mask the defect.
    problem.native_source_clock_c0d_gnss_first_d_handoff_mps.pop_back();
    const FGOProcessor::FGOResult rejected = processor.optimizeProblem(problem);
    EXPECT_FALSE(rejected.diagnostics.converged);
    EXPECT_TRUE(rejected.solution.isEmpty());
    EXPECT_TRUE(rejected.epoch_clock_drift_mps.empty());
}

TEST(FGOGtsamActiveSolveDiagnosticTest,
     ReportsZeroProgressTerminationWithoutChangingOptimizerResult) {
    const gtsam::Key key = gtsam::Symbol('z', 0);
    gtsam::NonlinearFactorGraph graph;
    graph.emplace_shared<ConstantResidualFactor>(
        key, gtsam::noiseModel::Isotropic::Sigma(1, 1.0));
    gtsam::Values initial;
    initial.insert(key, 0.0);

    gtsam::LevenbergMarquardtParams params;
    params.setMaxIterations(12);
    params.setAbsoluteErrorTol(1.0e-10);
    params.setRelativeErrorTol(1.0e-8);
    libgnss::fgo_gtsam_internal::GtsamLmActiveSolveOptimizer optimizer(
        graph, initial, params, true);
    const gtsam::Values optimized = optimizer.optimize();
    const auto& telemetry = optimizer.telemetry();

    EXPECT_TRUE(telemetry.active_solve_attempted);
    EXPECT_TRUE(telemetry.finite_costs);
    EXPECT_TRUE(telemetry.termination_trace_complete);
    EXPECT_EQ(telemetry.accepted_outer_iterations, 0U);
    EXPECT_GE(telemetry.total_inner_lambda_attempts, 1U);
    EXPECT_EQ(telemetry.initial_cost, telemetry.final_cost);
    EXPECT_EQ(graph.error(initial), graph.error(optimized));
    EXPECT_FALSE(telemetry.termination_branch_reason.empty());
    EXPECT_EQ(telemetry.small_cost_change_stop_count +
                  telemetry.maximum_lambda_stop_count,
              1U);
}

TEST(FGOGtsamPhase96DiagnosticTest,
     CapturesRejectedTrialFromExistingOptimizerWithoutChangingState) {
    const gtsam::Key key = gtsam::Symbol('z', 0);
    gtsam::NonlinearFactorGraph graph;
    graph.emplace_shared<ConstantResidualFactor>(
        key, gtsam::noiseModel::Isotropic::Sigma(1, 1.0));
    gtsam::Values initial;
    initial.insert(key, 0.0);
    gtsam::LevenbergMarquardtParams params;
    params.setMaxIterations(12);
    params.setAbsoluteErrorTol(1.0e-10);
    params.setRelativeErrorTol(1.0e-8);

    fgo_gtsam_internal::GtsamLmActiveSolveOptimizer optimizer(
        graph, initial, params, false, true);
    const gtsam::Values optimized = optimizer.optimize();
    const auto& telemetry = optimizer.telemetry();
    ASSERT_FALSE(telemetry.phase96_lm_trials.empty());
    EXPECT_LE(telemetry.phase96_lm_trials.size(), 10U);
    EXPECT_EQ(telemetry.phase96_lm_trials.front().rejection_reason,
              "small_cost_change_stop");
    EXPECT_TRUE(telemetry.phase96_lm_trials.front().candidate_finite);
    EXPECT_EQ(graph.error(initial), graph.error(optimized));
}

TEST(FGOGtsamPhase96DiagnosticTest,
     ParsesAcceptedRejectedAndNonfinitePinnedLmTrialPaths) {
    // This is the exact line vocabulary emitted by the pinned
    // LevenbergMarquardtOptimizer at TRYLAMBDA verbosity.  The parser is
    // intentionally tested independently so the failure branches remain
    // covered without a raw dataset or a solution/accuracy lane.
    const std::string trace =
        "trying lambda = 1e-05\n"
        "linear delta norm = 2\n"
        "newlinearizedError = 4  linearizedCostChange = 6\n"
        "calculating error:\n"
        "old error (10) new (tentative) error (4)\n"
        "modelFidelity: 1\n"
        "trying lambda = 0.001\n"
        "linear delta norm = 1\n"
        "newlinearizedError = 5  linearizedCostChange = 5\n"
        "calculating error:\n"
        "old error (4) new (tentative) error (3.5)\n"
        "modelFidelity: 0.1\n"
        "increasing lambda\n"
        "trying lambda = 0.1\n"
        "linear delta norm = 1\n"
        "newlinearizedError = nan  linearizedCostChange = nan\n"
        "increasing lambda\n";
    const auto trials =
        fgo_gtsam_internal::GtsamLmActiveSolveOptimizer::parsePhase96TraceForTesting(
            trace);
    ASSERT_EQ(trials.size(), 3U);
    EXPECT_EQ(trials[0].rejection_reason, "accepted_outer_step");
    EXPECT_TRUE(trials[0].candidate_finite);
    EXPECT_DOUBLE_EQ(trials[0].predicted_reduction, 6.0);
    EXPECT_DOUBLE_EQ(trials[0].actual_reduction, 6.0);
    EXPECT_EQ(trials[1].rejection_reason, "model_fidelity_below_threshold");
    EXPECT_TRUE(trials[1].candidate_finite);
    EXPECT_EQ(trials[2].rejection_reason, "nonfinite_linearized_error");
    EXPECT_FALSE(trials[2].candidate_finite);
    EXPECT_TRUE(trials[2].linear_system_solved);
}

TEST(FGOGtsamPhase96DiagnosticTest,
     MainGraphSidecarReportsFamiliesNormsAndAtMostTenTrials) {
    EXPECT_FALSE(FGOProcessor::FGOConfig{}
                     .use_native_source_clock_c0d_phase96_main_diagnostics);
    FGOProcessor::FGOProblem problem = makePhase93GnssFirstClockStateProblem();
    double lat = 0.0;
    double lon = 0.0;
    double height = 0.0;
    ecef2geodetic(problem.epochs.front().position_ecef, lat, lon, height);
    problem.imu.valid = true;
    problem.imu.nav_origin_ecef = problem.epochs.front().position_ecef;
    problem.imu.nav_origin_lat_rad = lat;
    problem.imu.nav_origin_lon_rad = lon;
    problem.imu.init_attitude_body_to_nav = Matrix3d::Identity();
    problem.native_source_clock_c0d_gnss_first_d_handoff_mps = {0.25, 0.35};

    FGOProcessor::FGOConfig config;
    config.backend = FGOBackend::GTSAM;
    config.use_pose3_state = true;
    config.use_imu = true;
    config.use_upstream_observable_quality = true;
    config.use_undifferenced_doppler_factors = true;
    config.use_native_source_clock_c0d_factor = true;
    config.native_source_clock_c0d_phone = "pixel5";
    config.use_native_source_clock_c0d_meter_state_parity = true;
    config.use_native_source_clock_c0d_active_solve_diagnostic = true;
    config.use_native_source_clock_c0d_gnss_first_meter_state_handoff = true;
    config.use_native_source_clock_c0d_phase96_main_diagnostics = true;
    config.use_native_source_clock_c0d_phase97_singular_system_diagnostics = true;
    config.use_native_source_clock_c0d_phase98_solver_rank_diagnostic = true;
    config.use_robust_loss = false;
    config.max_iterations = 1;

    const FGOProcessor::FGOResult result =
        FGOProcessor(config).optimizeProblem(problem);
    const auto& diagnostics =
        result.diagnostics.native_source_clock_c0d_phase96_main;
    EXPECT_TRUE(diagnostics.enabled);
    EXPECT_TRUE(diagnostics.attempted);
    EXPECT_TRUE(diagnostics.graph_observed);
    EXPECT_TRUE(diagnostics.initial_linearization_observed);
    EXPECT_GT(diagnostics.graph_factor_count, 0U);
    EXPECT_GT(diagnostics.graph_value_count, 0U);
    EXPECT_TRUE(std::isfinite(diagnostics.graph_initial_cost));
    EXPECT_TRUE(std::isfinite(diagnostics.factor_family_cost_sum));
    ASSERT_FALSE(diagnostics.factor_families.empty());
    ASSERT_FALSE(diagnostics.variable_family_norms.empty());
    bool saw_clock_ccdd = false;
    bool saw_code = false;
    bool saw_doppler = false;
    bool saw_imu = false;
    bool saw_priors = false;
    bool saw_finite_norm = false;
    for (const auto& family : diagnostics.factor_families) {
        if (family.family == "clock_ccdd") {
            saw_clock_ccdd = family.factor_count == 1U &&
                             family.finite_factor_count == 1U;
        }
        saw_code = saw_code || family.family == "gnss_code";
        saw_doppler = saw_doppler || family.family == "gnss_doppler";
        saw_imu = saw_imu || family.family == "imu_preintegration_bias";
        saw_priors = saw_priors || family.family == "priors";
    }
    for (const auto& norm : diagnostics.variable_family_norms) {
        if (norm.finite_contribution_count > 0U &&
            std::isfinite(norm.gradient_l2_norm) &&
            std::isfinite(norm.normal_diagonal_l2_norm)) {
            saw_finite_norm = true;
        }
    }
    EXPECT_TRUE(saw_clock_ccdd);
    EXPECT_TRUE(saw_code);
    EXPECT_TRUE(saw_doppler);
    EXPECT_TRUE(saw_imu);
    EXPECT_TRUE(saw_priors);
    EXPECT_TRUE(saw_finite_norm);
    EXPECT_GT(diagnostics.lm_trials.size(), 0U);
    EXPECT_LE(diagnostics.lm_trials.size(), 10U);
    const auto& singular_diagnostics =
        result.diagnostics.native_source_clock_c0d_phase97_singular_system;
    EXPECT_TRUE(singular_diagnostics.enabled);
    EXPECT_TRUE(singular_diagnostics.attempted);
    EXPECT_TRUE(singular_diagnostics.graph_observed);
    EXPECT_TRUE(singular_diagnostics.initial_linearization_observed);
    EXPECT_TRUE(singular_diagnostics.diagnostic_complete);
    EXPECT_GT(singular_diagnostics.graph_value_dimension, 0U);
    EXPECT_GT(singular_diagnostics.connected_component_count, 0U);
    EXPECT_GT(singular_diagnostics.keys.size(), 0U);
    EXPECT_GT(singular_diagnostics.factors.size(), 0U);
    ASSERT_EQ(singular_diagnostics.rank_decompositions.size(), 1U);
    EXPECT_TRUE(singular_diagnostics.rank_decompositions.front().attempted);
    EXPECT_FALSE(singular_diagnostics.lm_trials.empty());
    for (const auto& trial : singular_diagnostics.lm_trials) {
        EXPECT_FALSE(trial.nearby_variable_available);
        EXPECT_FALSE(trial.nearby_variable_status.empty());
    }
    // The legacy aggregate diagnostic remains independently populated, while
    // the new sidecar is absent by default and therefore cannot alter it.
    FGOProcessor::FGOConfig legacy_config = config;
    legacy_config.use_native_source_clock_c0d_phase96_main_diagnostics = false;
    legacy_config.use_native_source_clock_c0d_phase97_singular_system_diagnostics =
        false;
    legacy_config.use_native_source_clock_c0d_phase98_solver_rank_diagnostic =
        false;
    const FGOProcessor::FGOResult legacy =
        FGOProcessor(legacy_config).optimizeProblem(problem);
    EXPECT_FALSE(legacy.diagnostics.native_source_clock_c0d_phase96_main.enabled);
    EXPECT_TRUE(legacy.diagnostics
                    .native_source_clock_c0d_active_solve_diagnostic_enabled);
    EXPECT_FALSE(legacy.diagnostics.native_source_clock_c0d_phase97_singular_system
                     .enabled);
    EXPECT_FALSE(legacy.diagnostics
                     .native_source_clock_c0d_phase98_solver_rank.enabled);
    const auto& solver_diagnostics =
        result.diagnostics.native_source_clock_c0d_phase98_solver_rank;
    EXPECT_TRUE(solver_diagnostics.enabled);
    EXPECT_TRUE(solver_diagnostics.attempted);
    EXPECT_EQ(solver_diagnostics.solver_type, "MULTIFRONTAL_CHOLESKY");
    EXPECT_EQ(solver_diagnostics.solver_branch, "multifrontal");
    EXPECT_EQ(solver_diagnostics.elimination_function,
              "EliminatePreferCholesky");
    EXPECT_EQ(solver_diagnostics.ordering_type, "COLAMD");
    EXPECT_FALSE(solver_diagnostics.explicit_ordering_present);
    EXPECT_GT(solver_diagnostics.ordering_size, 0U);
    EXPECT_FALSE(solver_diagnostics.ordering_digest.empty());
    EXPECT_FALSE(solver_diagnostics.diagonal_damping);
}

TEST(FGOGtsamPhase97SingularDiagnosticTest,
     CapturesUnanchoredComponentZeroColumnAndExactKeyReference) {
    const gtsam::Key unanchored_a = gtsam::Symbol('u', 4);
    const gtsam::Key unanchored_b = gtsam::Symbol('u', 5);
    const gtsam::Key zero_column = gtsam::Symbol('q', 9);
    gtsam::NonlinearFactorGraph graph;
    graph.emplace_shared<gtsam::BetweenFactor<double>>(
        unanchored_a, unanchored_b, 0.0,
        gtsam::noiseModel::Isotropic::Sigma(1, 1.0));
    graph.emplace_shared<ConstantResidualFactor>(
        zero_column, gtsam::noiseModel::Isotropic::Sigma(1, 1.0));
    gtsam::Values initial;
    initial.insert(unanchored_a, 0.0);
    initial.insert(unanchored_b, 1.0);
    initial.insert(zero_column, 0.0);

    FGOProcessor::FGOPhase97MainDiagnostics diagnostics;
    diagnostics.enabled = true;
    fgo_gtsam_internal::collectPhase97InitialGraphDiagnostics(
        graph, initial, diagnostics);

    EXPECT_TRUE(diagnostics.graph_observed);
    EXPECT_TRUE(diagnostics.initial_linearization_observed);
    EXPECT_TRUE(diagnostics.diagnostic_complete);
    EXPECT_EQ(diagnostics.graph_value_count, 3U);
    EXPECT_EQ(diagnostics.connected_component_count, 2U);
    EXPECT_EQ(diagnostics.isolated_value_key_count, 0U);
    ASSERT_EQ(diagnostics.keys.size(), 3U);
    ASSERT_EQ(diagnostics.factors.size(), 2U);
    ASSERT_EQ(diagnostics.rank_decompositions.size(), 1U);
    EXPECT_TRUE(diagnostics.rank_decompositions.front().rank_known);
    EXPECT_EQ(diagnostics.rank_decompositions.front().rank, 1U);
    EXPECT_EQ(diagnostics.rank_decompositions.front().nullity, 2U);

    bool saw_unanchored_pair = false;
    bool saw_zero_column = false;
    for (const auto& component : diagnostics.components) {
        if (!component.anchored && component.key_count == 2U) {
            saw_unanchored_pair = true;
        }
    }
    for (const auto& key : diagnostics.keys) {
        if (key.key.numeric_key == static_cast<std::uint64_t>(zero_column)) {
            saw_zero_column = key.exact_zero_column &&
                              key.exact_zero_normal_diagonal_count == 1U &&
                              key.value_dimension == 1U && key.value_present;
            EXPECT_EQ(key.key.symbol_character, 'q');
            EXPECT_EQ(key.key.symbol_index, 9U);
            EXPECT_EQ(key.value_type, "gtsam::GenericValue<double>");
        }
    }
    EXPECT_TRUE(saw_unanchored_pair);
    EXPECT_TRUE(saw_zero_column);

    bool attributed_zero_column = false;
    for (const auto& key : diagnostics.rank_decompositions.front()
                                .nullspace_attribution) {
        attributed_zero_column =
            attributed_zero_column || key.numeric_key == zero_column;
    }
    EXPECT_TRUE(attributed_zero_column);
}

TEST(FGOGtsamPhase97SingularDiagnosticTest,
     PreservesPinnedNearbyVariableWhenAnExceptionObjectIsAvailable) {
    const gtsam::Key key = gtsam::Symbol('n', 42);
    const gtsam::IndeterminantLinearSystemException exception(key);
    const auto reference =
        fgo_gtsam_internal::phase97NearbyVariableReference(exception);
    EXPECT_EQ(reference.numeric_key, static_cast<std::uint64_t>(key));
    EXPECT_EQ(reference.symbol_character, 'n');
    EXPECT_EQ(reference.symbol_index, 42U);
}

TEST(FGOGtsamPhase97SingularDiagnosticTest, IsDisabledInLegacyConfiguration) {
    const FGOProcessor::FGOConfig config;
    EXPECT_FALSE(
        config.use_native_source_clock_c0d_phase97_singular_system_diagnostics);
    EXPECT_FALSE(FGOProcessor::FGOResult{}
                     .diagnostics.native_source_clock_c0d_phase97_singular_system
                     .enabled);
}

TEST(FGOGtsamPhase98SolverDiagnosticTest,
     CapturesTypedIndeterminateExceptionMetadataAtBoundary) {
    const gtsam::Key key = gtsam::Symbol('n', 42);
    gtsam::NonlinearFactorGraph graph;
    graph.emplace_shared<ThrowingIndeterminateFactor>(
        key, gtsam::noiseModel::Isotropic::Sigma(1, 1.0));
    gtsam::Values initial;
    initial.insert(key, 0.0);
    gtsam::LevenbergMarquardtParams params;
    params.setMaxIterations(1);
    params.setDiagonalDamping(false);

    fgo_gtsam_internal::GtsamLmActiveSolveOptimizer optimizer(
        graph, initial, params, false, false, true);
    try {
        (void)optimizer.optimize();
        FAIL() << "the synthetic factor must throw an indeterminate system";
    } catch (const gtsam::IndeterminantLinearSystemException& exception) {
        const auto& telemetry = optimizer.telemetry();
        ASSERT_TRUE(telemetry.phase98.enabled);
        EXPECT_TRUE(telemetry.phase98.attempted);
        EXPECT_TRUE(telemetry.phase98.exception_captured);
        ASSERT_EQ(telemetry.phase98.indeterminate_exceptions.size(), 1U);
        const auto& record = telemetry.phase98.indeterminate_exceptions.front();
        EXPECT_EQ(record.stage, "lm_optimize");
        EXPECT_DOUBLE_EQ(record.lambda, params.lambdaInitial);
        EXPECT_EQ(record.solver_type, "MULTIFRONTAL_CHOLESKY");
        EXPECT_EQ(record.solver_branch, "multifrontal");
        EXPECT_EQ(record.elimination_function, "EliminatePreferCholesky");
        EXPECT_EQ(record.ordering_type, "COLAMD");
        EXPECT_FALSE(record.explicit_ordering_present);
        EXPECT_GT(record.ordering_size, 0U);
        EXPECT_FALSE(record.ordering_digest.empty());
        EXPECT_FALSE(record.diagonal_damping);
        ASSERT_TRUE(record.nearby_variable_available);
        EXPECT_EQ(record.nearby_variable.numeric_key,
                  static_cast<std::uint64_t>(key));
        EXPECT_EQ(record.nearby_variable.symbol_character, 'n');
        EXPECT_EQ(record.nearby_variable.symbol_index, 42U);
        EXPECT_EQ(record.nearby_variable_status, "captured_from_exception");
        EXPECT_EQ(record.exception_type,
                  "gtsam::IndeterminantLinearSystemException");
        EXPECT_STREQ(record.exception_message.c_str(), exception.what());
    }
}

TEST(FGOGtsamPhase98SolverDiagnosticTest, IsDisabledInLegacyConfiguration) {
    const FGOProcessor::FGOConfig config;
    EXPECT_FALSE(config.use_native_source_clock_c0d_phase98_solver_rank_diagnostic);
    EXPECT_FALSE(FGOProcessor::FGOResult{}
                     .diagnostics.native_source_clock_c0d_phase98_solver_rank
                     .enabled);
}

TEST(FGOGtsamPhase101SourceClockVectorTest,
     UsesOfficialSevenComponentSystemFrequencyMapping) {
    using fgo_gtsam_internal::sourceClockComponentFor;
    EXPECT_EQ(sourceClockComponentFor(GNSSSystem::GPS,
                                      SignalType::GPS_L1CA), 0);
    EXPECT_EQ(sourceClockComponentFor(GNSSSystem::GPS,
                                      SignalType::GPS_L1P), 0);
    EXPECT_EQ(sourceClockComponentFor(GNSSSystem::GLONASS,
                                      SignalType::GLO_L1CA), 1);
    EXPECT_EQ(sourceClockComponentFor(GNSSSystem::GLONASS,
                                      SignalType::GLO_L1P), 1);
    EXPECT_EQ(sourceClockComponentFor(GNSSSystem::Galileo,
                                      SignalType::GAL_E1), 2);
    EXPECT_EQ(sourceClockComponentFor(GNSSSystem::BeiDou,
                                      SignalType::BDS_B1I), 3);
    EXPECT_EQ(sourceClockComponentFor(GNSSSystem::BeiDou,
                                      SignalType::BDS_B1C), 3);
    EXPECT_EQ(sourceClockComponentFor(GNSSSystem::GPS,
                                      SignalType::GPS_L5), 4);
    EXPECT_EQ(sourceClockComponentFor(GNSSSystem::Galileo,
                                      SignalType::GAL_E5A), 5);
    EXPECT_EQ(sourceClockComponentFor(GNSSSystem::BeiDou,
                                      SignalType::BDS_B2A), 6);
    // sysfreq2sigtype.m returns its sentinel 7 for unsupported systems or
    // frequencies; the C++ candidate represents that as -1 and rejects it
    // before factor insertion.
    EXPECT_EQ(sourceClockComponentFor(GNSSSystem::QZSS,
                                      SignalType::QZS_L1CA), -1);
    EXPECT_EQ(sourceClockComponentFor(GNSSSystem::GPS,
                                      SignalType::GPS_L2C), -1);
}

TEST(FGOGtsamPhase101SourceClockVectorTest,
     PseudorangeAndClockFactorsShareTheFullVectorTopology) {
    const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
    const gtsam::Key x_key = gtsam::Symbol('x', 4);
    const gtsam::Key c_key = gtsam::Symbol('c', 4);
    const gtsam::Point3 receiver(0.0, 0.0, 0.0);
    const gtsam::Point3 satellite(10.0, 0.0, 0.0);
    fgo_gtsam_internal::PseudorangeFactorSourceClock factor(
        x_key, c_key, 13.0, satellite, 2, noise);
    gtsam::Vector clock = gtsam::Vector::Zero(7);
    clock(0) = 1.0;
    clock(2) = 2.0;
    gtsam::Matrix Hx;
    gtsam::Matrix Hc;
    const gtsam::Vector error = factor.evaluateError(
        receiver, clock, Hx, Hc);
    ASSERT_EQ(error.size(), 1);
    EXPECT_DOUBLE_EQ(error(0), 0.0);
    EXPECT_EQ(Hx.rows(), 1);
    EXPECT_EQ(Hx.cols(), 3);
    ASSERT_EQ(Hc.rows(), 1);
    ASSERT_EQ(Hc.cols(), 7);
    EXPECT_DOUBLE_EQ(Hc(0, 0), 1.0);
    EXPECT_DOUBLE_EQ(Hc(0, 2), 1.0);
    EXPECT_DOUBLE_EQ(Hc.row(0).sum(), 2.0);

    gtsam::Vector c1 = gtsam::Vector::Zero(7);
    gtsam::Vector c2 = gtsam::Vector::Zero(7);
    gtsam::Vector d1 = gtsam::Vector::Constant(1, 1.0);
    gtsam::Vector d2 = gtsam::Vector::Constant(1, 1.0);
    c2(0) = 2.0;
    fgo_gtsam_internal::SourceClockVectorC0DFactor ccdd(
        gtsam::Symbol('c', 0), gtsam::Symbol('c', 1),
        gtsam::Symbol('d', 0), gtsam::Symbol('d', 1), 1.0,
        gtsam::noiseModel::Isotropic::Sigma(7, 1.0));
    gtsam::Matrix Hc1;
    gtsam::Matrix Hc2;
    gtsam::Matrix Hd1;
    gtsam::Matrix Hd2;
    const gtsam::Vector ccdd_error = ccdd.evaluateError(
        c1, c2, d1, d2, Hc1, Hc2, Hd1, Hd2);
    ASSERT_EQ(ccdd_error.size(), 7);
    EXPECT_DOUBLE_EQ(ccdd_error(0), 1.0);
    EXPECT_EQ(Hc1.rows(), 7);
    EXPECT_EQ(Hc1.cols(), 7);
    EXPECT_EQ(Hc2.rows(), 7);
    EXPECT_EQ(Hc2.cols(), 7);
    EXPECT_EQ(Hd1.rows(), 7);
    EXPECT_EQ(Hd1.cols(), 1);
    EXPECT_DOUBLE_EQ(Hd1(0, 0), -0.5);
    EXPECT_DOUBLE_EQ(Hd1.bottomRows(6).norm(), 0.0);
}

TEST(FGOGtsamPhase101SourceClockVectorTest,
     MultiConstellationRowsRemainEpochLocalAndSingleGpsUsesOnlyGaugeComponent) {
    const std::array<std::tuple<GNSSSystem, SignalType, int>, 7> mappings = {{
        {GNSSSystem::GPS, SignalType::GPS_L1CA, 0},
        {GNSSSystem::GLONASS, SignalType::GLO_L1CA, 1},
        {GNSSSystem::Galileo, SignalType::GAL_E1, 2},
        {GNSSSystem::BeiDou, SignalType::BDS_B1I, 3},
        {GNSSSystem::GPS, SignalType::GPS_L5, 4},
        {GNSSSystem::Galileo, SignalType::GAL_E5A, 5},
        {GNSSSystem::BeiDou, SignalType::BDS_B2A, 6},
    }};
    for (const auto& [system, signal, component] : mappings) {
        const gtsam::Vector hc = fgo_gtsam_internal::sourceClockComponentJacobian(
            fgo_gtsam_internal::sourceClockComponentFor(system, signal));
        EXPECT_EQ(hc.size(), 7);
        EXPECT_DOUBLE_EQ(hc(0), 1.0);
        EXPECT_DOUBLE_EQ(hc(component), 1.0);
        EXPECT_EQ((hc.array() != 0.0).count(), component == 0 ? 1 : 2);
    }
    const gtsam::Vector gps_l1 =
        fgo_gtsam_internal::sourceClockComponentJacobian(0);
    EXPECT_EQ((gps_l1.array() != 0.0).count(), 1);
    EXPECT_EQ(fgo_gtsam_internal::sourceClockComponentFor(
                  GNSSSystem::QZSS, SignalType::QZS_L1CA), -1);
}

TEST(FGOGtsamPhase101SourceClockVectorTest,
     LegacyConfigurationDoesNotExposeEpochVectorOrGlobalStateMixing) {
    const FGOProcessor::FGOConfig config;
    EXPECT_FALSE(config.use_native_source_clock_c0d_epoch_vector_parity);
    const FGOProcessor::FGOResult result;
    EXPECT_TRUE(result.epoch_clock_bias_components_m.empty());
    EXPECT_FALSE(
        result.diagnostics.native_source_clock_c0d_epoch_vector_parity_enabled);
    EXPECT_EQ(result.diagnostics.native_source_clock_c0d_epoch_vector_dimension,
              0U);
}

TEST(FGOGtsamPhase101SourceClockVectorTest,
     SyntheticGnssFirstExportsFullFiniteCAndDWithoutGlobalIsb) {
    FGOProcessor::FGOProblem problem;
    const Vector3d receiver(1113194.0, -4841695.0, 3985350.0);
    const auto satellites = gtsamParitySatelliteGeometry();
    const std::array<std::pair<GNSSSystem, SignalType>, 7> signals = {{
        {GNSSSystem::GPS, SignalType::GPS_L1CA},
        {GNSSSystem::GLONASS, SignalType::GLO_L1CA},
        {GNSSSystem::Galileo, SignalType::GAL_E1},
        {GNSSSystem::BeiDou, SignalType::BDS_B1I},
        {GNSSSystem::GPS, SignalType::GPS_L5},
        {GNSSSystem::Galileo, SignalType::GAL_E5A},
        {GNSSSystem::BeiDou, SignalType::BDS_B2A},
    }};
    for (std::size_t epoch = 0; epoch < 2; ++epoch) {
        FGOProcessor::EpochSeed seed;
        seed.time = GNSSTime(2300, 200000.0 + static_cast<double>(epoch));
        seed.position_ecef = receiver;
        seed.receiver_clock_bias_m = 0.0;
        seed.receiver_clock_bias_is_meters = true;
        seed.receiver_clock_drift_mps = 0.2 + 0.1 * epoch;
        seed.raw_source_index = epoch;
        seed.raw_utc_time_millis = 1620000000000LL +
                                   static_cast<std::int64_t>(epoch) * 1000LL;
        problem.epochs.push_back(seed);
        for (std::size_t row = 0; row < signals.size(); ++row) {
            FGOProcessor::PseudorangeFactor pseudorange;
            pseudorange.epoch_index = epoch;
            pseudorange.satellite = SatelliteId(
                signals[row].first, static_cast<uint8_t>(row + 1));
            pseudorange.signal = signals[row].second;
            pseudorange.clock_group = signals[row].first;
            pseudorange.satellite_position_ecef = satellites[row % satellites.size()];
            pseudorange.corrected_pseudorange_m =
                (pseudorange.satellite_position_ecef - receiver).norm();
            pseudorange.sigma_m = 1.0;
            problem.pseudorange_factors.push_back(pseudorange);

            FGOProcessor::UndifferencedDopplerFactor doppler;
            doppler.epoch_index = epoch;
            doppler.satellite = pseudorange.satellite;
            doppler.signal = pseudorange.signal;
            doppler.los = (pseudorange.satellite_position_ecef - receiver).normalized();
            doppler.residual_mps = seed.receiver_clock_drift_mps;
            doppler.sigma_mps = 1.0;
            doppler.includes_receiver_clock_drift = true;
            problem.undifferenced_doppler_factors.push_back(doppler);
        }
    }

    FGOProcessor::FGOConfig config;
    config.backend = FGOBackend::GTSAM;
    config.use_pose3_state = false;
    config.use_imu = false;
    config.use_velocity_states = true;
    config.use_upstream_observable_quality = true;
    config.use_undifferenced_doppler_factors = true;
    config.use_motion_factors = true;
    config.use_clock_motion_factors = true;
    config.use_native_source_clock_c0d_factor = true;
    config.native_source_clock_c0d_phone = "pixel7";
    config.use_native_source_clock_c0d_meter_state_parity = true;
    config.use_native_source_clock_c0d_gnss_first_meter_state_handoff = true;
    config.use_native_source_clock_c0d_raw_drift_d_initializer = true;
    config.use_native_source_clock_c0d_epoch_vector_parity = true;
    config.use_inter_system_biases = false;
    config.use_tdcp_factors = false;
    config.use_carrier_phase_factors = false;
    config.use_double_difference_factors = false;
    config.position_prior_sigma_m = 1.0;
    config.max_iterations = 3;

    const FGOProcessor processor(config);
    const FGOProcessor::FGOResult result = processor.optimizeProblem(problem);
    ASSERT_EQ(result.epoch_clock_bias_components_m.size(), 2U);
    ASSERT_EQ(result.epoch_clock_drift_mps.size(), 2U);
    EXPECT_EQ(result.diagnostics.native_source_clock_c0d_epoch_vector_dimension,
              7U);
    EXPECT_EQ(result.diagnostics.native_source_clock_c0d_epoch_vector_state_count,
              2U);
    EXPECT_EQ(result.diagnostics.native_source_clock_c0d_global_isb_state_count,
              0U);
    for (const auto& clock : result.epoch_clock_bias_components_m) {
        for (const double value : clock) EXPECT_TRUE(std::isfinite(value));
    }
    for (const double drift : result.epoch_clock_drift_mps) {
        EXPECT_TRUE(std::isfinite(drift));
    }
}

TEST(FGOGtsamPhase101SourceClockVectorTest,
     MainHandoffRequiresFullExactCVectorCoverageAndUsesVectorDKeys) {
    FGOProcessor::FGOProblem problem =
        makePhase93GnssFirstClockStateProblem();
    double lat = 0.0;
    double lon = 0.0;
    double height = 0.0;
    ecef2geodetic(problem.epochs.front().position_ecef, lat, lon, height);
    problem.imu.valid = true;
    problem.imu.nav_origin_ecef = problem.epochs.front().position_ecef;
    problem.imu.nav_origin_lat_rad = lat;
    problem.imu.nav_origin_lon_rad = lon;
    problem.imu.init_attitude_body_to_nav = Matrix3d::Identity();
    for (auto& epoch : problem.epochs) {
        epoch.receiver_clock_drift_mps =
            std::numeric_limits<double>::quiet_NaN();
    }
    problem.native_source_clock_c0d_gnss_first_d_handoff_mps = {0.40, 0.45};
    FGOProcessor::EpochClockBiasComponentsM c0{};
    FGOProcessor::EpochClockBiasComponentsM c1{};
    c0[0] = 12.0;
    c0[2] = 2.0;
    c1[0] = 13.0;
    c1[2] = 2.0;
    problem.native_source_clock_c0d_gnss_first_c_handoff_m = {c0, c1};

    FGOProcessor::FGOConfig config;
    config.backend = FGOBackend::GTSAM;
    config.use_pose3_state = true;
    config.use_imu = true;
    config.use_upstream_observable_quality = true;
    config.use_undifferenced_doppler_factors = true;
    config.use_native_source_clock_c0d_factor = true;
    config.native_source_clock_c0d_phone = "pixel5";
    config.use_native_source_clock_c0d_meter_state_parity = true;
    config.use_native_source_clock_c0d_gnss_first_meter_state_handoff = true;
    config.use_native_source_clock_c0d_epoch_vector_parity = true;
    config.use_native_source_clock_c0d_phase99_main_multifrontal_qr_solver =
        true;
    config.use_inter_system_biases = false;
    config.use_robust_loss = false;
    config.use_tdcp_factors = false;
    config.max_iterations = 2;

    const FGOProcessor::FGOResult result =
        FGOProcessor(config).optimizeProblem(problem);
    EXPECT_TRUE(result.diagnostics.converged);
    EXPECT_TRUE(result.diagnostics
                    .native_source_clock_c0d_epoch_vector_parity_enabled);
    EXPECT_EQ(result.diagnostics.native_source_clock_c0d_global_isb_state_count,
              0U);
    ASSERT_EQ(result.epoch_clock_bias_components_m.size(), 2U);
    ASSERT_EQ(result.epoch_clock_drift_mps.size(), 2U);
    for (const auto& clock : result.epoch_clock_bias_components_m) {
        EXPECT_TRUE(std::all_of(clock.begin(), clock.end(),
                                [](double value) { return std::isfinite(value); }));
    }
    for (const double drift : result.epoch_clock_drift_mps) {
        EXPECT_TRUE(std::isfinite(drift));
    }

    // C handoff coverage is fail-closed, even when the raw EpochSeed D values
    // are unavailable; no scalar/global fallback may hide a missing vector.
    problem.native_source_clock_c0d_gnss_first_c_handoff_m.pop_back();
    const FGOProcessor::FGOResult rejected =
        FGOProcessor(config).optimizeProblem(problem);
    EXPECT_FALSE(rejected.diagnostics.converged);
    EXPECT_TRUE(rejected.solution.isEmpty());
    EXPECT_TRUE(rejected.epoch_clock_bias_components_m.empty());
}

TEST(FGOGtsamPhase135AffineFamilyTest,
     SelectorIsOffByDefaultAndGeodistUsesOneSagnacAndOfficialLos) {
    const FGOProcessor::FGOConfig config;
    EXPECT_FALSE(config.use_native_phase135_official_affine_measurement_family);
    EXPECT_FALSE(config.use_native_phase135_phase107_raw_base_recipe);

    const gtsam::Point3 satellite(20200000.0, 14000000.0, 21100000.0);
    const gtsam::Point3 receiver(1113194.0, -4841695.0, 3985350.0);
    fgo_gtsam_internal::Phase135SourceGeometry geometry;
    ASSERT_TRUE(fgo_gtsam_internal::phase135SourceGeodist(
        satellite, receiver, geometry));
    const gtsam::Point3 delta = satellite - receiver;
    const double geometric = delta.norm();
    const double expected_sagnac =
        libgnss::constants::OMEGA_E / libgnss::constants::SPEED_OF_LIGHT *
        (satellite.x() * receiver.y() - satellite.y() * receiver.x());
    EXPECT_NEAR(geometry.range_m, geometric + expected_sagnac, 1e-9);
    EXPECT_TRUE(geometry.los.isApprox(-delta / geometric, 1e-12));
    EXPECT_NEAR(geometry.los.norm(), 1.0, 1e-12);
    EXPECT_TRUE(geometry.los.allFinite());
    fgo_gtsam_internal::Phase135SourceGeometry invalid;
    EXPECT_FALSE(fgo_gtsam_internal::phase135SourceGeodist(
        satellite, satellite, invalid));
    EXPECT_FALSE(fgo_gtsam_internal::phase135SourceGeodist(
        gtsam::Point3(1.0, 1.0, 1.0), receiver, invalid));
}

TEST(FGOGtsamPhase138AffineTdcpTest,
     AnchorRangeConstantHandlesZeroMovingAndSignedGeometryDelta) {
    using libgnss::tdcp_contract::Phase138AffineTdcpMeasurement;
    Phase138AffineTdcpMeasurement adjusted;
    ASSERT_TRUE(
        libgnss::tdcp_contract::applyPhase138AffineTdcpAnchorRangeConstant(
            7.0, 100.0, 103.0, adjusted));
    EXPECT_NEAR(adjusted.range_delta_m, 3.0, 1e-12);
    EXPECT_NEAR(adjusted.tdcp_m, 4.0, 1e-12);

    ASSERT_TRUE(
        libgnss::tdcp_contract::applyPhase138AffineTdcpAnchorRangeConstant(
            7.0, 100.0, 100.0, adjusted));
    EXPECT_NEAR(adjusted.range_delta_m, 0.0, 1e-12);
    EXPECT_NEAR(adjusted.tdcp_m, 7.0, 1e-12);

    ASSERT_TRUE(
        libgnss::tdcp_contract::applyPhase138AffineTdcpAnchorRangeConstant(
            7.0, 100.0, 94.0, adjusted));
    EXPECT_NEAR(adjusted.range_delta_m, -6.0, 1e-12);
    EXPECT_NEAR(adjusted.tdcp_m, 13.0, 1e-12);

    EXPECT_FALSE(
        libgnss::tdcp_contract::applyPhase138AffineTdcpAnchorRangeConstant(
            std::numeric_limits<double>::quiet_NaN(), 100.0, 103.0,
            adjusted));
    EXPECT_FALSE(
        libgnss::tdcp_contract::applyPhase138AffineTdcpAnchorRangeConstant(
            7.0, 100.0, std::numeric_limits<double>::infinity(), adjusted));
}

TEST(FGOGtsamPhase138AffineTdcpTest,
     MovingSatelliteSingleSagnacRangeDeltaFeedsAffineMeasurement) {
    const gtsam::Point3 previous_satellite(20200000.0, 14000000.0,
                                           21100000.0);
    const gtsam::Point3 current_satellite(20200120.0, 13999940.0,
                                         21100025.0);
    const gtsam::Point3 previous_receiver(1113194.0, -4841695.0, 3985350.0);
    const gtsam::Point3 current_receiver(1113201.0, -4841698.0, 3985351.0);
    fgo_gtsam_internal::Phase135SourceGeometry previous_geometry;
    fgo_gtsam_internal::Phase135SourceGeometry current_geometry;
    ASSERT_TRUE(fgo_gtsam_internal::phase135SourceGeodist(
        previous_satellite, previous_receiver, previous_geometry));
    ASSERT_TRUE(fgo_gtsam_internal::phase135SourceGeodist(
        current_satellite, current_receiver, current_geometry));

    libgnss::tdcp_contract::Phase138AffineTdcpMeasurement adjusted;
    ASSERT_TRUE(
        libgnss::tdcp_contract::applyPhase138AffineTdcpAnchorRangeConstant(
            11.5, previous_geometry.range_m, current_geometry.range_m,
            adjusted));
    EXPECT_TRUE(std::isfinite(adjusted.range_delta_m));
    EXPECT_TRUE(std::isfinite(adjusted.tdcp_m));
    EXPECT_NEAR(adjusted.tdcp_m,
                11.5 - (current_geometry.range_m - previous_geometry.range_m),
                1e-12);
    EXPECT_TRUE(previous_geometry.los.allFinite());
    EXPECT_TRUE(current_geometry.los.allFinite());
    EXPECT_NEAR(previous_geometry.los.norm(), 1.0, 1e-12);
    EXPECT_NEAR(current_geometry.los.norm(), 1.0, 1e-12);
}

TEST(FGOGtsamPhase138AffineTdcpTest,
     InitialResidualMatchesLegacyAndJacobianIsUnchanged) {
    const gtsam::Point3 previous_satellite(20200000.0, 14000000.0,
                                           21100000.0);
    const gtsam::Point3 current_satellite(20200075.0, 14000035.0,
                                         21099980.0);
    const gtsam::Point3 previous_initial(1113194.0, -4841695.0, 3985350.0);
    const gtsam::Point3 current_initial(1113195.0, -4841693.0, 3985350.5);
    fgo_gtsam_internal::Phase135SourceGeometry previous_geometry;
    fgo_gtsam_internal::Phase135SourceGeometry current_geometry;
    ASSERT_TRUE(fgo_gtsam_internal::phase135SourceGeodist(
        previous_satellite, previous_initial, previous_geometry));
    ASSERT_TRUE(fgo_gtsam_internal::phase135SourceGeodist(
        current_satellite, current_initial, current_geometry));

    constexpr double kNativeTdcpM = 12.75;
    libgnss::tdcp_contract::Phase138AffineTdcpMeasurement adjusted;
    ASSERT_TRUE(
        libgnss::tdcp_contract::applyPhase138AffineTdcpAnchorRangeConstant(
            kNativeTdcpM, previous_geometry.range_m, current_geometry.range_m,
            adjusted));
    const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
    const gtsam::Vector previous_clock = gtsam::Vector::Zero(7);
    const gtsam::Vector current_clock = gtsam::Vector::Zero(7);
    fgo_gtsam_internal::Phase135TdcpAffinePointFactor affine_factor(
        gtsam::Symbol('X', 0), gtsam::Symbol('X', 1), gtsam::Symbol('C', 0),
        gtsam::Symbol('C', 1), previous_geometry.los, adjusted.tdcp_m,
        gtsam::Vector3(previous_initial), gtsam::Vector3(current_initial),
        noise);
    fgo_gtsam_internal::Phase135TdcpAffinePointFactor unadjusted_factor(
        gtsam::Symbol('X', 0), gtsam::Symbol('X', 1), gtsam::Symbol('C', 0),
        gtsam::Symbol('C', 1), previous_geometry.los, kNativeTdcpM,
        gtsam::Vector3(previous_initial), gtsam::Vector3(current_initial),
        noise);
    gtsam::Matrix affine_previous;
    gtsam::Matrix affine_current;
    gtsam::Matrix affine_previous_clock;
    gtsam::Matrix affine_current_clock;
    const gtsam::Vector affine_error = affine_factor.evaluateError(
        previous_initial, current_initial, previous_clock, current_clock,
        affine_previous, affine_current, affine_previous_clock,
        affine_current_clock);
    gtsam::Matrix legacy_previous;
    gtsam::Matrix legacy_current;
    gtsam::Matrix legacy_previous_clock;
    gtsam::Matrix legacy_current_clock;
    const gtsam::Vector unadjusted_error = unadjusted_factor.evaluateError(
        previous_initial, current_initial, previous_clock, current_clock,
        legacy_previous, legacy_current, legacy_previous_clock,
        legacy_current_clock);
    ASSERT_EQ(affine_error.size(), 1);
    ASSERT_EQ(unadjusted_error.size(), 1);
    EXPECT_NEAR(affine_error(0),
                current_geometry.range_m - previous_geometry.range_m -
                    kNativeTdcpM,
                1e-9);
    EXPECT_NEAR(unadjusted_error(0), -kNativeTdcpM, 1e-12);
    EXPECT_TRUE(affine_previous.isApprox(legacy_previous, 1e-12));
    EXPECT_TRUE(affine_current.isApprox(legacy_current, 1e-12));
    EXPECT_TRUE(affine_previous_clock.isApprox(legacy_previous_clock, 1e-12));
    EXPECT_TRUE(affine_current_clock.isApprox(legacy_current_clock, 1e-12));
}

TEST(FGOGtsamPhase267VelocityPriorTest, RequiresMainDopplerAndSupportedBackend) {
    FGOProcessor::FGOConfig config;
    EXPECT_FALSE(config.omit_native_first_imu_velocity_prior);
    config.omit_native_first_imu_velocity_prior = true;
    config.backend = FGOBackend::GTSAM;
    config.use_pose3_state = true;
    config.use_imu = true;
    config.use_native_phase171_raw_p_no_doppler_imu_main = true;
    FGOProcessor::FGOProblem problem;
    EXPECT_THROW(FGOProcessor(config).optimizeProblem(problem), std::invalid_argument);
    config.use_native_phase213_main_doppler = true;
    config.backend = FGOBackend::Eigen;
    EXPECT_THROW(FGOProcessor(config).optimizeProblem(problem), std::invalid_argument);
}

TEST(FGOGtsamPhase263BiasPriorTest, RejectsUnsupportedBackendAndState) {
    FGOProcessor::FGOConfig config;
    EXPECT_FALSE(config.omit_native_first_imu_bias_prior);
    config.omit_native_first_imu_bias_prior = true;
    FGOProcessor::FGOProblem problem;
    config.backend = FGOBackend::Eigen;
    EXPECT_THROW(FGOProcessor(config).optimizeProblem(problem), std::invalid_argument);
    config.backend = FGOBackend::GTSAM;
    config.use_imu = true;
    config.use_pose3_state = true;
    EXPECT_THROW(FGOProcessor(config).optimizeProblem(problem), std::invalid_argument);
    config.use_native_phase171_raw_p_no_doppler_imu_main = true;
    config.use_imu = false;
    EXPECT_THROW(FGOProcessor(config).optimizeProblem(problem), std::invalid_argument);
}

TEST(FGOGtsamPhase259EpochHeadingTest, RejectsMissingMisalignedAndInvalidRotations) {
    auto problem = makePhase164RawNoDopplerProblem(true, 6);
    FGOProcessor::FGOConfig config;
    EXPECT_FALSE(config.use_native_epoch_heading_attitude_seeds);
    config.use_native_epoch_heading_attitude_seeds = true;
    config.backend = FGOBackend::GTSAM;
    config.use_pose3_state = true;
    config.use_imu = true;
    config.use_native_phase171_raw_p_no_doppler_imu_main = true;
    EXPECT_THROW(FGOProcessor(config).optimizeProblem(problem), std::invalid_argument);
    for (const auto& epoch : problem.epochs) {
        problem.imu.epoch_heading_attitude_times.push_back(epoch.time);
        problem.imu.epoch_heading_attitudes_body_to_nav.push_back(Matrix3d::Identity());
    }
    auto malformed = problem;
    malformed.imu.epoch_heading_attitude_times[1] = problem.epochs[1].time + 0.001;
    EXPECT_THROW(FGOProcessor(config).optimizeProblem(malformed), std::invalid_argument);
    for (int kind = 0; kind < 3; ++kind) {
        malformed = problem;
        auto& rotation = malformed.imu.epoch_heading_attitudes_body_to_nav[1];
        rotation(0, 0) = kind == 0 ? -1.0 : kind == 1 ? 2.0 :
                            std::numeric_limits<double>::quiet_NaN();
        EXPECT_THROW(FGOProcessor(config).optimizeProblem(malformed), std::invalid_argument);
    }
    config.backend = FGOBackend::Eigen;
    EXPECT_THROW(FGOProcessor(config).optimizeProblem(problem), std::invalid_argument);
}

TEST(FGOGtsamPhase253TdcpOnlyAffineTest, RejectsEigenInsteadOfIgnoringSelector) {
    FGOProcessor::FGOConfig config;
    config.backend = FGOBackend::Eigen;
    config.use_native_tdcp_only_affine_geometry = true;
    FGOProcessor::FGOProblem problem;
    EXPECT_THROW(FGOProcessor(config).optimizeProblem(problem), std::invalid_argument);
}

TEST(FGOGtsamPhase274RelativeHeightGuardTest, RejectsUnsupportedAndMissingSeeds) {
    FGOProcessor::FGOConfig config;
    FGOProcessor::FGOProblem problem;
    config.use_native_relative_height_pairs = true;
    EXPECT_THROW(FGOProcessor(config).optimizeProblem(problem), std::invalid_argument);
    config.backend = FGOBackend::GTSAM;
    config.use_imu = true;
    config.use_pose3_state = true;
    config.use_upstream_stop_constraints = true;
    config.use_native_phase171_raw_p_no_doppler_imu_main = true;
    config.use_native_source_clock_c0d_gnss_first_meter_state_handoff = true;
    EXPECT_THROW(FGOProcessor(config).optimizeProblem(problem), std::invalid_argument);
    problem.epochs.resize(1);
    EXPECT_THROW(FGOProcessor(config).optimizeProblem(problem), std::invalid_argument);
    problem.imu.stop_velocity_seeds_nav.push_back(Vector3d::Zero());
    problem.imu.stop_velocity_seeds_nav[0].x() =
        std::numeric_limits<double>::quiet_NaN();
    EXPECT_THROW(FGOProcessor(config).optimizeProblem(problem), std::invalid_argument);
}

TEST(FGOGtsamPhase273RelativeHeightNoiseTest, ScalarMatchesInfiniteHorizontalSigmaHuber) {
    using namespace gtsam::noiseModel;
    const double inf = std::numeric_limits<double>::infinity();
    const auto vector_noise = Robust::Create(mEstimator::Huber::Create(0.5),
        Diagonal::Sigmas(gtsam::Vector3(inf, inf, 0.1)));
    const auto scalar_noise = Robust::Create(mEstimator::Huber::Create(0.5),
        Isotropic::Sigma(1, 0.1));
    // Exercise zero, quadratic, transition, and linear-loss branches,
    // both signs, with large finite horizontal residuals.
    for (double z : {-10.0, -0.051, -0.05, -0.02, 0.0, 0.02, 0.05, 0.051, 10.0}) {
        const gtsam::Vector r3 = gtsam::Vector3(12345.0, -9876.0, z);
        const gtsam::Vector r1 = gtsam::Vector::Constant(1, z);
        const double d3 = vector_noise->squaredMahalanobisDistance(r3);
        const double d1 = scalar_noise->squaredMahalanobisDistance(r1);
        EXPECT_DOUBLE_EQ(d3, d1);
        EXPECT_DOUBLE_EQ(vector_noise->loss(d3), scalar_noise->loss(d1));
        // Robust::weight takes a residual in the estimator's normalized
        // domain; raw metre vectors include unscaled horizontal components.
        EXPECT_DOUBLE_EQ(vector_noise->weight(vector_noise->unweightedWhiten(r3)),
                         scalar_noise->weight(scalar_noise->unweightedWhiten(r1)));
        const auto w3 = vector_noise->whiten(r3);
        const auto w1 = scalar_noise->whiten(r1);
        EXPECT_DOUBLE_EQ(w3(0), 0.0);
        EXPECT_DOUBLE_EQ(w3(1), 0.0);
        EXPECT_DOUBLE_EQ(w3(2), w1(0));
    }
}

TEST(FGOGtsamPhase272RelativeHeightTest, RotatedFrameLeverArmAndHorizontalNullspace) {
    const gtsam::Pose3 frame(gtsam::Rot3::RzRyRx(0.2,-0.4,0.7),
                            gtsam::Point3(6378137,100,200));
    const gtsam::Point3 lever(0.3,-0.2,0.5);
    const gtsam::gnss::LeverArm arm(lever,frame);
    const gtsam::Vector3 up = frame.rotation().rotate(gtsam::Vector3::UnitZ());
    const gtsam::Pose3 a(gtsam::Rot3::RzRyRx(-0.3,0.1,0.4), gtsam::Point3(3,4,5));
    const gtsam::Pose3 b(gtsam::Rot3::RzRyRx(0.5,-0.2,-0.6), gtsam::Point3(6,8,9));
    const auto noise = gtsam::noiseModel::Isotropic::Sigma(1,0.1);
    fgo_gtsam_internal::RelativeHeightPoseFactor factor(1,2,up,arm,noise);
    gtsam::Matrix h1,h2;
    const double residual = factor.evaluateError(a,b,h1,h2)(0);
    EXPECT_NEAR(residual, (b.transformFrom(lever)-a.transformFrom(lever)).z(), 1e-8);
    const gtsam::Pose3 shifted(b.rotation(),b.translation()+gtsam::Point3(100,-50,0));
    EXPECT_NEAR(factor.evaluateError(a,shifted)(0),residual,1e-8);
    constexpr double eps=1e-3;
    for(int j=0;j<6;++j) {
        auto d=gtsam::Vector::Zero(6).eval(); d(j)=eps;
        EXPECT_NEAR(h1(0,j),(factor.evaluateError(a.retract(d),b)(0)-
                    factor.evaluateError(a.retract(-d),b)(0))/(2*eps),2e-6);
        EXPECT_NEAR(h2(0,j),(factor.evaluateError(a,b.retract(d))(0)-
                    factor.evaluateError(a,b.retract(-d))(0))/(2*eps),2e-6);
    }
    EXPECT_THROW((fgo_gtsam_internal::RelativeHeightPoseFactor(
        1,2,gtsam::Vector3::Zero(),arm,noise)),std::invalid_argument);
    EXPECT_THROW((fgo_gtsam_internal::RelativeHeightPoseFactor(
        1,2,up,arm,gtsam::noiseModel::Isotropic::Sigma(3,1))),std::invalid_argument);
}

TEST(FGOGtsamPhase252AffinePoseTest, RotatedFrameAndLeverArmJacobiansMatchNumericalDerivative) {
    const gtsam::Pose3 frame(gtsam::Rot3::RzRyRx(0.2, -0.4, 0.7),
                            gtsam::Point3(6378137.0, 100.0, 200.0));
    const gtsam::Point3 lever(0.3, -0.2, 0.5);
    const gtsam::gnss::LeverArm arm(lever, frame);
    const gtsam::Pose3 x1(gtsam::Rot3::RzRyRx(-0.3, 0.1, 0.4), gtsam::Point3(3, 4, 5));
    const gtsam::Pose3 x2(gtsam::Rot3::RzRyRx(0.5, -0.2, -0.6), gtsam::Point3(6, 8, 9));
    const gtsam::Point3 p1 = frame.transformFrom(x1.transformFrom(lever));
    const gtsam::Point3 p2 = frame.transformFrom(x2.transformFrom(lever));
    const gtsam::Vector3 los = gtsam::Vector3(1, 2, -3).normalized();
    auto c1 = gtsam::Vector::Zero(7).eval();
    auto c2 = gtsam::Vector::Constant(7, 9.0).eval();
    c2(0) = 0.7;
    const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
    fgo_gtsam_internal::SourceAffineTdcpPoseFactor factor(
        1, 2, 3, 4, los, p1, p2, 0.2, arm, noise);
    gtsam::Matrix h1, hc1, h2, hc2;
    EXPECT_NEAR(factor.evaluateError(x1, c1, x2, c2, h1, hc1, h2, hc2)(0), 0.5, 1e-8);
    // Millimetre finite differences keep ECEF subtraction roundoff bounded.
    constexpr double eps = 1e-3;
    for (int j = 0; j < 6; ++j) {
        auto d = gtsam::Vector::Zero(6).eval(); d(j) = eps;
        const double n1 = (factor.evaluateError(x1.retract(d), c1, x2, c2)(0) -
                           factor.evaluateError(x1.retract(-d), c1, x2, c2)(0)) / (2 * eps);
        const double n2 = (factor.evaluateError(x1, c1, x2.retract(d), c2)(0) -
                           factor.evaluateError(x1, c1, x2.retract(-d), c2)(0)) / (2 * eps);
        EXPECT_NEAR(h1(0,j), n1, 2e-6);
        EXPECT_NEAR(h2(0,j), n2, 2e-6);
    }
    for (int j = 0; j < 7; ++j) {
        auto d = gtsam::Vector::Zero(7).eval(); d(j) = eps;
        EXPECT_NEAR(hc1(0,j), (factor.evaluateError(x1,c1+d,x2,c2)(0) -
                    factor.evaluateError(x1,c1-d,x2,c2)(0))/(2*eps), 1e-10);
        EXPECT_NEAR(hc2(0,j), (factor.evaluateError(x1,c1,x2,c2+d)(0) -
                    factor.evaluateError(x1,c1,x2,c2-d)(0))/(2*eps), 1e-10);
    }
    EXPECT_THROW((fgo_gtsam_internal::SourceAffineTdcpPoseFactor(
        1,2,3,4,gtsam::Vector3::Zero(),p1,p2,0.2,arm,noise)), std::invalid_argument);
    c1(0) = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(factor.evaluateError(x1,c1,x2,c2).allFinite());
}

TEST(FGOGtsamPhase256TdcpConventionTest, AtmosphereShiftIsIndependentOfGeometryAtFixedStates) {
    const gtsam::Point3 x1(6378137.0, 0.0, 0.0), x2(6378138.0, 0.0, 0.0);
    const gtsam::Point3 s1 = x1 + gtsam::Point3(20000000.0, 0.0, 0.0);
    const gtsam::Point3 s2 = x2 + gtsam::Point3(20000000.0, 2000.0, 0.0);
    const double r1 = (s1 - x1).norm(), r2 = (s2 - x2).norm();
    const gtsam::Vector3 los = (x1 - s1).normalized();
    auto c1 = gtsam::Vector::Zero(7).eval();
    auto c2 = gtsam::Vector::Zero(7).eval();
    c2(0) = 0.75;
    // Synthetic phase: range + receiver clock - satellite clock + T - I + N.
    // Constant ambiguity cancels. Delta(T-I) = -0.5 m remains in resL;
    // affine geometry cannot cancel that atmospheric measurement shift.
    const double raw1 = r1 - 2.0 + 3.0 - 1.0 + 100.0;
    const double raw2 = r2 + 0.75 - 2.5 + 3.25 - 1.75 + 100.0;
    const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
    std::array<double, 2> affine_moved{}, nonlinear_moved{};
    for (int source = 0; source < 2; ++source) {
        const double measurement = tdcp_contract::ordinaryTdcpCarrierMeters(
            raw2, 2.5, 1.75, 3.25, source) -
            tdcp_contract::ordinaryTdcpCarrierMeters(raw1, 2.0, 1.0, 3.0, source);
        fgo_gtsam_internal::Phase135TdcpAffinePointFactor affine(
            1, 2, 3, 4, los, measurement - (r2 - r1), x1, x2, noise);
        fgo_gtsam_internal::TimeDifferencedCarrierFactorSourceClockPoint nonlinear(
            1, 3, 2, 4, s1, s2, measurement, noise);
        EXPECT_NEAR(affine.evaluateError(x1, x2, c1, c2)(0), source ? 0.5 : 0.0, 1e-8);
        EXPECT_NEAR(nonlinear.evaluateError(x1, c1, x2, c2)(0), source ? 0.5 : 0.0, 1e-8);
        const gtsam::Point3 moved = x2 + gtsam::Point3(0.0, 100.0, 0.0);
        affine_moved[source] = affine.evaluateError(x1, moved, c1, c2)(0);
        nonlinear_moved[source] = nonlinear.evaluateError(x1, c1, moved, c2)(0);
    }
    EXPECT_NEAR(affine_moved[1] - affine_moved[0], 0.5, 1e-8);
    EXPECT_NEAR(nonlinear_moved[1] - nonlinear_moved[0], 0.5, 1e-8);
    EXPECT_NEAR(nonlinear_moved[1] - affine_moved[1],
                nonlinear_moved[0] - affine_moved[0], 1e-8);
    EXPECT_GT(std::abs(nonlinear_moved[0] - affine_moved[0]), 0.009);
}

TEST(FGOGtsamPhase251TdcpGeometryTest, MovingSatelliteSeparatesPreviousLosFromEndpointGeometry) {
    // Common Cartesian frame, without Sagnac/atmosphere, to isolate geometry.
    const gtsam::Point3 x1(6378137.0, 0.0, 0.0), x2(6378138.0, 0.0, 0.0);
    const gtsam::Point3 s1 = x1 + gtsam::Point3(20000000.0, 0.0, 0.0);
    const gtsam::Point3 s2 = x2 + gtsam::Point3(20000000.0, 2000.0, 0.0);
    const gtsam::Vector3 los1 = (x1 - s1).normalized();
    const gtsam::Vector3 los2 = (x2 - s2).normalized();
    const double dr = (s2 - x2).norm() - (s1 - x1).norm();
    const double measurement = dr + 0.75;
    auto c1 = gtsam::Vector::Zero(7).eval();
    auto c2 = gtsam::Vector::Zero(7).eval();
    c2(0) = 0.75;
    const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
    fgo_gtsam_internal::Phase135TdcpAffinePointFactor affine(
        1, 2, 3, 4, los1, measurement - dr, x1, x2, noise);
    fgo_gtsam_internal::TimeDifferencedCarrierFactorSourceClockPoint nonlinear(
        1, 3, 2, 4, s1, s2, measurement, noise);
    gtsam::Matrix a1, a2, ac1, ac2, n1, n2, nc1, nc2;
    EXPECT_NEAR(affine.evaluateError(x1, x2, c1, c2, a1, a2, ac1, ac2)(0), 0.0, 1e-12);
    EXPECT_NEAR(nonlinear.evaluateError(x1, c1, x2, c2, n1, nc1, n2, nc2)(0), 0.0, 1e-8);
    EXPECT_TRUE(a1.isApprox(n1, 1e-12));
    EXPECT_TRUE(a2.isApprox(los1.transpose(), 1e-12));
    EXPECT_TRUE(n2.isApprox(los2.transpose(), 1e-12));
    EXPECT_GT((a2 - n2).norm(), 0.00009);
    EXPECT_TRUE(ac1.isApprox(nc1, 1e-12));
    EXPECT_TRUE(ac2.isApprox(nc2, 1e-12));
    const gtsam::Point3 moved = x2 + gtsam::Point3(0.0, 100.0, 0.0);
    const double ea = affine.evaluateError(x1, moved, c1, c2)(0);
    const double en = nonlinear.evaluateError(x1, c1, moved, c2)(0);
    EXPECT_NEAR(ea, 0.0, 1e-12);
    EXPECT_NEAR(en, (s2 - moved).norm() - (s2 - x2).norm(), 1e-8);
    EXPECT_GT(std::abs(en - ea), 0.009);
}

TEST(FGOGtsamPhase138AffineTdcpTest,
     SelectorIsDefaultOffAndRequiresPhase135WithoutFallback) {
    FGOProcessor::FGOConfig config;
    EXPECT_FALSE(config.use_native_phase138_affine_tdcp_anchor_range_constant);
    config.backend = FGOBackend::GTSAM;
    config.use_native_phase138_affine_tdcp_anchor_range_constant = true;
    EXPECT_THROW(FGOProcessor(config).optimizeProblem(
                     FGOProcessor::FGOProblem{}),
                 std::invalid_argument);
}

TEST(FGOGtsamPhase135AffineFamilyTest,
     OfficialDopplerResidualIncludesReceiverVelocityAndExplicitSagnac) {
    const gtsam::Point3 satellite(20200000.0, 14000000.0, 21100000.0);
    const gtsam::Point3 receiver(1113194.0, -4841695.0, 3985350.0);
    const gtsam::Vector3 satellite_velocity(1200.0, -2200.0, 800.0);
    const gtsam::Vector3 receiver_velocity(12.0, -8.0, 3.0);
    const double satellite_clock_drift_mps = 0.42;
    const gtsam::Vector3 e =
        (satellite - receiver) / (satellite - receiver).norm();
    const double sagnac =
        libgnss::constants::OMEGA_E / libgnss::constants::SPEED_OF_LIGHT *
        (satellite_velocity.y() * receiver.x() +
         satellite.y() * receiver_velocity.x() -
         satellite_velocity.x() * receiver.y() -
         satellite.x() * receiver_velocity.y());
    const double expected_rate =
        (satellite_velocity - receiver_velocity).dot(e) + sagnac;
    const double expected_residual = 0.37;
    const double measured_rate =
        expected_rate - satellite_clock_drift_mps + expected_residual;
    double modeled_rate = 0.0;
    double residual = 0.0;
    ASSERT_TRUE(fgo_gtsam_internal::phase135OfficialDopplerResidual(
        satellite, satellite_velocity, receiver, receiver_velocity,
        measured_rate, satellite_clock_drift_mps, modeled_rate, residual));
    EXPECT_NEAR(modeled_rate, expected_rate, 1e-12);
    EXPECT_NEAR(residual, expected_residual, 1e-12);

    double unused_rate = 0.0;
    double unused_residual = 0.0;
    EXPECT_FALSE(fgo_gtsam_internal::phase135OfficialDopplerResidual(
        satellite, satellite_velocity, receiver, receiver_velocity,
        measured_rate, std::numeric_limits<double>::quiet_NaN(), unused_rate,
        unused_residual));
}

TEST(FGOGtsamPhase135AffineFamilyTest, PFactorMatchesOfficialResidualJacobianAndKeyOrder) {
    const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
    const gtsam::Vector3 los(0.6, -0.8, 0.0);
    const gtsam::Vector3 initial(10.0, 20.0, 30.0);
    fgo_gtsam_internal::Phase135PseudorangeAffinePointFactor factor(
        gtsam::Symbol('X', 4), gtsam::Symbol('C', 4), los, 10.0, 2,
        initial, noise);
    ASSERT_EQ(factor.keys().size(), 2U);
    EXPECT_EQ(factor.keys()[0], gtsam::Symbol('X', 4));
    EXPECT_EQ(factor.keys()[1], gtsam::Symbol('C', 4));

    gtsam::Matrix Hx;
    gtsam::Matrix Hc;
    const gtsam::Vector error = factor.evaluateError(
        gtsam::Point3(12.0, 19.0, 30.0),
        (gtsam::Vector(7) << 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0).finished(),
        Hx, Hc);
    ASSERT_EQ(error.size(), 1);
    EXPECT_NEAR(error(0), -5.0, 1e-12);
    ASSERT_EQ(Hx.rows(), 1);
    ASSERT_EQ(Hx.cols(), 3);
    EXPECT_TRUE(Hx.isApprox(los.transpose(), 1e-12));
    const gtsam::Vector expected_hc =
        (gtsam::Vector(7) << 1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0).finished();
    EXPECT_TRUE(Hc.row(0).transpose().isApprox(expected_hc, 1e-12));
}

TEST(FGOGtsamPhase135AffineFamilyTest, AnalyticJacobiansMatchCentralDifferences) {
    const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
    const double epsilon = 1e-7;

    const gtsam::Vector3 p_initial(10.0, 20.0, 30.0);
    const gtsam::Vector3 p_value(12.0, 19.0, 31.0);
    const gtsam::Vector c_value =
        (gtsam::Vector(7) << 0.2, -0.1, 3.0, 0.4, -0.3, 0.5, -0.6)
            .finished();
    fgo_gtsam_internal::Phase135PseudorangeAffinePointFactor p_factor(
        gtsam::Symbol('X', 7), gtsam::Symbol('C', 7),
        gtsam::Vector3(0.6, -0.8, 0.0), 10.0, 2, p_initial, noise);
    gtsam::Matrix Hx;
    gtsam::Matrix Hc;
    (void)p_factor.evaluateError(gtsam::Point3(p_value), c_value, Hx, Hc);
    const auto p_error = [&](const gtsam::Vector3& p,
                             const gtsam::Vector& c) {
        gtsam::Matrix unused_x;
        gtsam::Matrix unused_c;
        return p_factor.evaluateError(gtsam::Point3(p), c, unused_x, unused_c)(0);
    };
    for (int column = 0; column < 3; ++column) {
        gtsam::Vector3 plus = p_value;
        gtsam::Vector3 minus = p_value;
        plus(column) += epsilon;
        minus(column) -= epsilon;
        EXPECT_NEAR((p_error(plus, c_value) - p_error(minus, c_value)) /
                        (2.0 * epsilon),
                    Hx(0, column), 1e-8);
    }
    for (int column = 0; column < 7; ++column) {
        gtsam::Vector plus = c_value;
        gtsam::Vector minus = c_value;
        plus(column) += epsilon;
        minus(column) -= epsilon;
        EXPECT_NEAR((p_error(p_value, plus) - p_error(p_value, minus)) /
                        (2.0 * epsilon),
                    Hc(0, column), 1e-8);
    }

    const gtsam::Vector3 v_initial(2.0, -1.0, 0.5);
    const gtsam::Vector3 v_value(2.3, -0.7, 0.9);
    const gtsam::Vector d_value = (gtsam::Vector(1) << 0.25).finished();
    fgo_gtsam_internal::Phase135DopplerAffineFactor d_factor(
        gtsam::Symbol('V', 7), gtsam::Symbol('D', 7),
        gtsam::Vector3(-0.2, 0.3, 0.9327379053), 1.5, v_initial, noise);
    gtsam::Matrix Hv;
    gtsam::Matrix Hd;
    (void)d_factor.evaluateError(v_value, d_value, Hv, Hd);
    const auto d_error = [&](const gtsam::Vector3& v,
                             const gtsam::Vector& d) {
        gtsam::Matrix unused_v;
        gtsam::Matrix unused_d;
        return d_factor.evaluateError(v, d, unused_v, unused_d)(0);
    };
    for (int column = 0; column < 3; ++column) {
        gtsam::Vector3 plus = v_value;
        gtsam::Vector3 minus = v_value;
        plus(column) += epsilon;
        minus(column) -= epsilon;
        EXPECT_NEAR((d_error(plus, d_value) - d_error(minus, d_value)) /
                        (2.0 * epsilon),
                    Hv(0, column), 1e-8);
    }
    gtsam::Vector d_plus = d_value;
    gtsam::Vector d_minus = d_value;
    d_plus(0) += epsilon;
    d_minus(0) -= epsilon;
    EXPECT_NEAR((d_error(v_value, d_plus) - d_error(v_value, d_minus)) /
                    (2.0 * epsilon),
                Hd(0, 0), 1e-8);

    const gtsam::Vector3 x1_initial(0.0, 0.0, 0.0);
    const gtsam::Vector3 x2_initial(1.0, 0.0, 0.0);
    const gtsam::Vector3 x1_value(0.2, -0.1, 0.3);
    const gtsam::Vector3 x2_value(1.4, 0.4, -0.2);
    const gtsam::Vector c1_value =
        (gtsam::Vector(7) << 0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished();
    const gtsam::Vector c2_value =
        (gtsam::Vector(7) << 0.8, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished();
    fgo_gtsam_internal::Phase135TdcpAffinePointFactor tdcp_factor(
        gtsam::Symbol('X', 8), gtsam::Symbol('X', 9),
        gtsam::Symbol('C', 8), gtsam::Symbol('C', 9),
        gtsam::Vector3(0.7, -0.2, 0.6782329983), 1.4, x1_initial,
        x2_initial, noise);
    gtsam::Matrix Hx1;
    gtsam::Matrix Hx2;
    gtsam::Matrix Hc1;
    gtsam::Matrix Hc2;
    (void)tdcp_factor.evaluateError(x1_value, x2_value, c1_value, c2_value,
                                    Hx1, Hx2, Hc1, Hc2);
    const auto tdcp_error = [&](const gtsam::Vector3& x1,
                                const gtsam::Vector3& x2,
                                const gtsam::Vector& c1,
                                const gtsam::Vector& c2) {
        gtsam::Matrix unused_x1;
        gtsam::Matrix unused_x2;
        gtsam::Matrix unused_c1;
        gtsam::Matrix unused_c2;
        return tdcp_factor.evaluateError(x1, x2, c1, c2, unused_x1, unused_x2,
                                          unused_c1, unused_c2)(0);
    };
    for (int column = 0; column < 3; ++column) {
        gtsam::Vector3 plus = x1_value;
        gtsam::Vector3 minus = x1_value;
        plus(column) += epsilon;
        minus(column) -= epsilon;
        EXPECT_NEAR((tdcp_error(plus, x2_value, c1_value, c2_value) -
                     tdcp_error(minus, x2_value, c1_value, c2_value)) /
                        (2.0 * epsilon),
                    Hx1(0, column), 1e-8);
        plus = x2_value;
        minus = x2_value;
        plus(column) += epsilon;
        minus(column) -= epsilon;
        EXPECT_NEAR((tdcp_error(x1_value, plus, c1_value, c2_value) -
                     tdcp_error(x1_value, minus, c1_value, c2_value)) /
                        (2.0 * epsilon),
                    Hx2(0, column), 1e-8);
    }
    for (int column = 0; column < 7; ++column) {
        gtsam::Vector plus = c1_value;
        gtsam::Vector minus = c1_value;
        plus(column) += epsilon;
        minus(column) -= epsilon;
        EXPECT_NEAR((tdcp_error(x1_value, x2_value, plus, c2_value) -
                     tdcp_error(x1_value, x2_value, minus, c2_value)) /
                        (2.0 * epsilon),
                    Hc1(0, column), 1e-8);
        plus = c2_value;
        minus = c2_value;
        plus(column) += epsilon;
        minus(column) -= epsilon;
        EXPECT_NEAR((tdcp_error(x1_value, x2_value, c1_value, plus) -
                     tdcp_error(x1_value, x2_value, c1_value, minus)) /
                        (2.0 * epsilon),
                    Hc2(0, column), 1e-8);
    }
}

TEST(FGOGtsamPhase135AffineFamilyTest, DopplerFactorUsesVDUnitsAndKeyOrder) {
    const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
    const gtsam::Vector3 los(-0.2, 0.3, 0.9327379053);
    const gtsam::Vector3 initial(2.0, -1.0, 0.5);
    fgo_gtsam_internal::Phase135DopplerAffineFactor factor(
        gtsam::Symbol('V', 5), gtsam::Symbol('D', 5), los, 1.5, initial,
        noise);
    ASSERT_EQ(factor.keys().size(), 2U);
    EXPECT_EQ(factor.keys()[0], gtsam::Symbol('V', 5));
    EXPECT_EQ(factor.keys()[1], gtsam::Symbol('D', 5));
    gtsam::Matrix Hv;
    gtsam::Matrix Hd;
    const gtsam::Vector error = factor.evaluateError(
        initial + gtsam::Vector3(1.0, 0.0, 0.0),
        (gtsam::Vector(1) << 0.25).finished(), Hv, Hd);
    ASSERT_EQ(error.size(), 1);
    EXPECT_NEAR(error(0), -1.45, 1e-12);
    EXPECT_TRUE(Hv.row(0).isApprox(los.transpose(), 1e-12));
    EXPECT_NEAR(Hd(0, 0), 1.0, 1e-12);
}

TEST(FGOGtsamPhase135AffineFamilyTest, TdcpAndPoseBridgeMatchOfficialKeyOrder) {
    const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
    const gtsam::Vector3 los(1.0, 0.0, 0.0);
    const gtsam::Vector3 previous_initial(0.0, 0.0, 0.0);
    const gtsam::Vector3 current_initial(1.0, 0.0, 0.0);
    fgo_gtsam_internal::Phase135TdcpAffinePointFactor tdcp(
        gtsam::Symbol('X', 1), gtsam::Symbol('X', 2), gtsam::Symbol('C', 1),
        gtsam::Symbol('C', 2), los, 2.0, previous_initial, current_initial,
        noise);
    ASSERT_EQ(tdcp.keys().size(), 4U);
    EXPECT_EQ(tdcp.keys()[0], gtsam::Symbol('X', 1));
    EXPECT_EQ(tdcp.keys()[1], gtsam::Symbol('X', 2));
    EXPECT_EQ(tdcp.keys()[2], gtsam::Symbol('C', 1));
    EXPECT_EQ(tdcp.keys()[3], gtsam::Symbol('C', 2));
    gtsam::Matrix Hx1;
    gtsam::Matrix Hx2;
    gtsam::Matrix Hc1;
    gtsam::Matrix Hc2;
    const gtsam::Vector error = tdcp.evaluateError(
        gtsam::Point3(0.5, 0.0, 0.0), gtsam::Point3(3.5, 0.0, 0.0),
        gtsam::Vector::Zero(7),
        (gtsam::Vector(7) << 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished(),
        Hx1, Hx2, Hc1, Hc2);
    ASSERT_EQ(error.size(), 1);
    EXPECT_NEAR(error(0), 0.5, 1e-12);
    EXPECT_TRUE(Hx1.row(0).isApprox(-los.transpose(), 1e-12));
    EXPECT_TRUE(Hx2.row(0).isApprox(los.transpose(), 1e-12));
    EXPECT_NEAR(Hc1(0, 0), -1.0, 1e-12);
    EXPECT_NEAR(Hc2(0, 0), 1.0, 1e-12);

    fgo_gtsam_internal::Phase135Pose3Point3FactorPX bridge(
        gtsam::Symbol('x', 1), gtsam::Symbol('X', 1),
        gtsam::noiseModel::Constrained::All(3));
    ASSERT_EQ(bridge.keys().size(), 2U);
    EXPECT_EQ(bridge.keys()[0], gtsam::Symbol('x', 1));
    EXPECT_EQ(bridge.keys()[1], gtsam::Symbol('X', 1));
    gtsam::Matrix Hp;
    gtsam::Matrix Hx;
    const gtsam::Vector bridge_error = bridge.evaluateError(
        gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1.0, 2.0, 3.0)),
        (gtsam::Vector(3) << 0.5, 1.0, 1.5).finished(), Hp, Hx);
    ASSERT_EQ(bridge_error.size(), 3);
    EXPECT_TRUE(bridge_error.isApprox(
        (gtsam::Vector(3) << 0.5, 1.0, 1.5).finished(), 1e-12));
    EXPECT_TRUE(Hx.isApprox(-gtsam::I_3x3, 1e-12));
}

TEST(FGOGtsamPhase135AffineFamilyTest, SelectorRejectsEigenAndNeverFallsBack) {
    FGOProcessor::FGOConfig config;
    config.use_native_phase135_official_affine_measurement_family = true;
    config.backend = FGOBackend::Eigen;
    EXPECT_THROW(FGOProcessor(config).optimizeProblem(
                     FGOProcessor::FGOProblem{}),
                 std::invalid_argument);
}

TEST(FGOGtsamPhase135AffineFamilyTest,
     SelectorRejectsPartialCompositionBeforeGraphConstruction) {
    FGOProcessor::FGOConfig config;
    config.use_native_phase135_official_affine_measurement_family = true;
    config.backend = FGOBackend::GTSAM;
    // A Phase135 graph is a single opt-in composition.  In particular, the
    // source raw-base/provenance and fixed official TDCP-k selectors cannot
    // be silently omitted when the library is used without the native app.
    EXPECT_THROW(FGOProcessor(config).optimizeProblem(
                     FGOProcessor::FGOProblem{}),
                 std::invalid_argument);

    config.use_official_tdcp_huber_k = true;
    config.use_native_phase126_raw_base_source_complete = true;
    config.use_native_phase127_glonass_channel_provenance = true;
    config.use_native_phase128_glonass_provenance_parser_admission = true;
    config.use_native_phase129_glonass_local_miss_mask = true;
    config.use_native_phase131_canonical_correction_band_key = true;
    // C0/D graph composition is still incomplete, so this must remain
    // fail-closed rather than reaching a legacy or mixed factor path.
    EXPECT_THROW(FGOProcessor(config).optimizeProblem(
                     FGOProcessor::FGOProblem{}),
                 std::invalid_argument);
}

TEST(FGOGtsamPhase164RawNoDopplerGraphTest,
     SelectorOffAndMotionFactorUseDedicatedXXVVEquation) {
    const FGOProcessor::FGOConfig legacy;
    EXPECT_FALSE(legacy.use_native_raw_p_no_doppler_graph);

    const auto noise = gtsam::noiseModel::Isotropic::Sigma(3, 1.0);
    fgo_gtsam_internal::MotionFactorXXVV factor(
        gtsam::Symbol('x', 0), gtsam::Symbol('x', 1), gtsam::Symbol('v', 0),
        gtsam::Symbol('v', 1), 2.0, noise);
    gtsam::Matrix Hx0;
    gtsam::Matrix Hv0;
    gtsam::Matrix Hx1;
    gtsam::Matrix Hv1;
    const gtsam::Vector error = factor.evaluateError(
        gtsam::Point3(0.0, 0.0, 0.0), gtsam::Point3(2.0, 4.0, 6.0),
        gtsam::Vector3(1.0, 2.0, 3.0), gtsam::Vector3(1.0, 2.0, 3.0), Hx0,
        Hx1, Hv0, Hv1);
    EXPECT_TRUE(error.isApprox(gtsam::Vector3::Zero(), 1e-12));
    EXPECT_TRUE(Hx0.isApprox(-gtsam::I_3x3, 1e-12));
    EXPECT_TRUE(Hx1.isApprox(gtsam::I_3x3, 1e-12));
    EXPECT_TRUE(Hv0.isApprox(-gtsam::I_3x3, 1e-12));
    EXPECT_TRUE(Hv1.isApprox(-gtsam::I_3x3, 1e-12));
}

TEST(FGOGtsamPhase164RawNoDopplerObservabilityTest,
     XXVVAlternatingVelocityPerturbationLeavesResidualUnchangedForUnequalDt) {
    const auto noise = gtsam::noiseModel::Isotropic::Sigma(3, 1.0);
    const gtsam::Point3 previous_point(120.0, -40.0, 7.0);
    const gtsam::Vector3 previous_velocity(3.0, -2.0, 0.75);
    const gtsam::Vector3 current_velocity(-1.0, 4.0, 2.5);
    const gtsam::Vector3 alternating_perturbation(17.0, -9.0, 5.0);

    // The perturbation is deliberately large: this is an exact structural
    // null direction of the adjacent velocity sum, not a tolerance test.
    for (const double dt_s : {0.7, 1.3, 2.6}) {
        const gtsam::Point3 current_point =
            previous_point +
            0.5 * dt_s * (previous_velocity + current_velocity);
        fgo_gtsam_internal::MotionFactorXXVV factor(
            gtsam::Symbol('x', 0), gtsam::Symbol('x', 1),
            gtsam::Symbol('v', 0), gtsam::Symbol('v', 1), dt_s, noise);

        const gtsam::Vector baseline = factor.evaluateError(
            previous_point, current_point, previous_velocity, current_velocity,
            nullptr, nullptr, nullptr, nullptr);
        const gtsam::Vector perturbed = factor.evaluateError(
            previous_point, current_point,
            previous_velocity + alternating_perturbation,
            current_velocity - alternating_perturbation, nullptr, nullptr,
            nullptr, nullptr);
        ASSERT_EQ(baseline.size(), 3);
        ASSERT_EQ(perturbed.size(), 3);
        EXPECT_LE((baseline - perturbed).norm(), 1.0e-12);
        EXPECT_LE(baseline.norm(), 1.0e-12);
    }
}

TEST(FGOGtsamPhase164RawNoDopplerObservabilityTest,
     BroadVelocityPriorHasFiniteNonzeroCostWithoutClaimingFullGraphRank) {
    constexpr double sigma_mps = 1.0e6;
    const gtsam::Key velocity_key = gtsam::Symbol('v', 0);
    const gtsam::Vector3 candidate_velocity(17.0, -9.0, 5.0);
    gtsam::NonlinearFactorGraph graph;
    graph.emplace_shared<gtsam::PriorFactor<gtsam::Vector3>>(
        velocity_key, gtsam::Vector3::Zero(),
        gtsam::noiseModel::Isotropic::Sigma(3, sigma_mps));
    gtsam::Values values;
    values.insert(velocity_key, candidate_velocity);

    // This isolates the existing broad sigma=1e6 m/s prior.  It establishes
    // only that the prior contributes a finite, nonzero penalty; it does not
    // claim that the complete no-D graph has an observationally unique rank.
    const double cost = graph.error(values);
    const double expected =
        0.5 * candidate_velocity.squaredNorm() / (sigma_mps * sigma_mps);
    EXPECT_GT(cost, 0.0);
    EXPECT_TRUE(std::isfinite(cost));
    EXPECT_NEAR(cost, expected, expected * 1.0e-10);
}

TEST(FGOGtsamPhase164RawNoDopplerObservabilityTest,
     SourceDopplerEcefLosAndClockDesignHasFullRankAndDeficientControl) {
    using DopplerFactor =
        fgo_gtsam_internal::UndifferencedDopplerVelocityFactorSourceClock;
    const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
    const gtsam::Vector3 true_velocity(3.5, -2.0, 0.75);
    const double true_clock_drift = 1.25;

    // These are independent receiver-to-satellite ECEF directions.  Build a
    // complete corrected source measurement first (satellite range rate,
    // receiver-only contract, and RINEX Doppler conversion), then pass only
    // the adapter's receiver residual to the factor.  This catches a LOS
    // sign/unit mismatch rather than regenerating the factor equation in the
    // test itself.
    const std::array<gtsam::Vector3, 4> e_rx_to_sat = {
        gtsam::Vector3(1.0, 0.0, 0.0),
        gtsam::Vector3(0.0, 1.0, 0.0),
        gtsam::Vector3(0.0, 0.0, 1.0),
        gtsam::Vector3(1.0, 1.0, 1.0).normalized()};
    const Vector3d receiver_position_ecef(1113194.0, -4841695.0,
                                          3985350.0);
    const Vector3d receiver_velocity_ecef = true_velocity;
    constexpr double frequency_hz = 1.57542e9;
    constexpr double satellite_clock_drift_sps = 1.2e-9;
    Eigen::MatrixXd full_design(4, 4);
    gtsam::NonlinearFactorGraph full_rank_graph;
    const gtsam::Key velocity_key = gtsam::Symbol('q', 0);
    const gtsam::Key clock_key = gtsam::Symbol('r', 0);
    for (std::size_t row = 0; row < e_rx_to_sat.size(); ++row) {
        const Vector3d satellite_position =
            receiver_position_ecef + e_rx_to_sat[row] * 21.0e6;
        const Vector3d satellite_velocity(
            1200.0 + 70.0 * static_cast<double>(row),
            -700.0 + 55.0 * static_cast<double>(row),
            850.0 - 35.0 * static_cast<double>(row));
        Vector3d los_receiver_to_satellite;
        double satellite_range_rate_mps = 0.0;
        ASSERT_TRUE(doppler_contract::knownSatelliteRangeRate(
            satellite_position, satellite_velocity, receiver_position_ecef,
            true, los_receiver_to_satellite, satellite_range_rate_mps));
        const double receiver_only_mps = doppler_contract::receiverPrediction(
            los_receiver_to_satellite, receiver_velocity_ecef,
            true_clock_drift);
        const double measured_range_rate_mps =
            satellite_range_rate_mps -
            satellite_clock_drift_sps * constants::SPEED_OF_LIGHT +
            receiver_only_mps;
        const double doppler_hz =
            -measured_range_rate_mps * frequency_hz /
            constants::SPEED_OF_LIGHT;
        const double roundtrip_range_rate_mps =
            doppler_contract::rinexDopplerToRangeRate(doppler_hz,
                                                      frequency_hz);
        const double measured_mps = doppler_contract::receiverOnlyResidual(
            roundtrip_range_rate_mps, satellite_range_rate_mps,
            satellite_clock_drift_sps);
        ASSERT_TRUE(std::isfinite(measured_mps));
        EXPECT_NEAR(measured_mps, receiver_only_mps, 1.0e-10);
        const gtsam::Vector3 factor_los =
            -los_receiver_to_satellite;
        full_rank_graph.emplace_shared<DopplerFactor>(
            velocity_key, clock_key, factor_los, measured_mps, noise);
        DopplerFactor factor(velocity_key, clock_key, factor_los,
                             measured_mps, noise);
        gtsam::Matrix H_velocity;
        gtsam::Matrix H_clock;
        const gtsam::Vector error = factor.evaluateError(
            true_velocity,
            (gtsam::Vector(1) << true_clock_drift).finished(), H_velocity,
            H_clock);
        ASSERT_EQ(error.size(), 1);
        EXPECT_NEAR(error(0), 0.0, 1.0e-12);
        ASSERT_EQ(H_velocity.rows(), 1);
        ASSERT_EQ(H_velocity.cols(), 3);
        ASSERT_EQ(H_clock.rows(), 1);
        ASSERT_EQ(H_clock.cols(), 1);
        full_design.block<1, 3>(static_cast<Eigen::Index>(row), 0) =
            H_velocity;
        full_design(static_cast<Eigen::Index>(row), 3) = H_clock(0, 0);
    }
    EXPECT_EQ(full_design.colPivHouseholderQr().rank(), 4);

    // The full-rank source design should also recover both unknowns from a
    // deliberately large initial error, not merely report a matrix rank.
    // This is a local linear factor, so no extra prior or manufactured
    // observation is needed to resolve the four columns.
    gtsam::Values initial;
    const gtsam::Vector3 initial_velocity =
        true_velocity + gtsam::Vector3(80.0, -60.0, 45.0);
    initial.insert(velocity_key, initial_velocity);
    initial.insert(clock_key,
                   (gtsam::Vector(1) << (true_clock_drift - 25.0)).finished());
    gtsam::LevenbergMarquardtParams params;
    params.setMaxIterations(50);
    params.setAbsoluteErrorTol(1.0e-12);
    params.setRelativeErrorTol(1.0e-12);
    gtsam::LevenbergMarquardtOptimizer optimizer(full_rank_graph, initial,
                                                 params);
    const gtsam::Values optimized = optimizer.optimize();
    EXPECT_LT((optimized.at<gtsam::Vector3>(velocity_key) - true_velocity)
                  .norm(),
              1.0e-8);
    EXPECT_NEAR(optimized.at<gtsam::Vector>(clock_key)(0), true_clock_drift,
                1.0e-8);

    // A coplanar ECEF LOS set leaves the out-of-plane velocity component
    // unobserved.  Keep the same source sign and factor Jacobian path; only
    // the geometry is intentionally rank deficient (rank three, not a claim
    // that the missing component is mixed with the clock column).
    const std::array<gtsam::Vector3, 4> coplanar_e_rx_to_sat = {
        gtsam::Vector3(1.0, 0.0, 0.0),
        gtsam::Vector3(0.0, 1.0, 0.0),
        gtsam::Vector3(1.0, 1.0, 0.0).normalized(),
        gtsam::Vector3(1.0, -1.0, 0.0).normalized()};
    Eigen::MatrixXd deficient_design(4, 4);
    for (std::size_t row = 0; row < coplanar_e_rx_to_sat.size(); ++row) {
        const gtsam::Vector3 factor_los = -coplanar_e_rx_to_sat[row];
        DopplerFactor factor(gtsam::Symbol('v', 0), gtsam::Symbol('d', 0),
                             factor_los, 0.0, noise);
        gtsam::Matrix H_velocity;
        gtsam::Matrix H_clock;
        (void)factor.evaluateError(gtsam::Vector3::Zero(),
                                   gtsam::Vector::Zero(1), H_velocity,
                                   H_clock);
        deficient_design.block<1, 3>(static_cast<Eigen::Index>(row), 0) =
            H_velocity;
        deficient_design(static_cast<Eigen::Index>(row), 3) = H_clock(0, 0);
    }
    EXPECT_EQ(deficient_design.colPivHouseholderQr().rank(), 3);
}

TEST(FGOGtsamPhase164RawNoDopplerGraphTest,
     MovingGpsRawPGraphConvergesWithEmptyDopplerAndFiniteExports) {
    const FGOProcessor::FGOProblem problem =
        makePhase164RawNoDopplerProblem(true, 6);
    const FGOProcessor::FGOResult result =
        FGOProcessor(makePhase164RawNoDopplerConfig()).optimizeProblem(problem);

    EXPECT_TRUE(result.diagnostics.converged);
    EXPECT_EQ(result.diagnostics.undifferenced_doppler_factors_inserted, 0U);
    EXPECT_EQ(result.diagnostics.native_source_clock_c0d_factor_count, 1U);
    EXPECT_EQ(result.diagnostics.motion_factors, 1U);
    ASSERT_EQ(result.epoch_velocities_ecef_mps.size(), 2U);
    ASSERT_EQ(result.epoch_clock_bias_components_m.size(), 2U);
    ASSERT_EQ(result.epoch_clock_drift_mps.size(), 2U);
    ASSERT_EQ(result.solution.size(), 2U);
    for (std::size_t epoch = 0; epoch < 2; ++epoch) {
        EXPECT_LT((result.solution.solutions[epoch].position_ecef -
                   problem.epochs[epoch].position_ecef)
                      .norm(),
                  1.0e-4);
        EXPECT_LT((result.epoch_velocities_ecef_mps[epoch] -
                   problem.native_raw_p_no_doppler_seeds[epoch]
                       .velocity_ecef_mps)
                      .norm(),
                  1.0e-4);
    }
    for (const auto& velocity : result.epoch_velocities_ecef_mps) {
        EXPECT_TRUE(velocity.allFinite());
    }
    for (const auto& clock : result.epoch_clock_bias_components_m) {
        EXPECT_TRUE(std::all_of(clock.begin(), clock.end(),
                                [](double value) {
                                    return std::isfinite(value);
                                }));
    }
    for (const double drift : result.epoch_clock_drift_mps) {
        EXPECT_TRUE(std::isfinite(drift));
    }
}

TEST(FGOGtsamPhase171EcefDopplerGraphTest,
     UsesCorrectedEcefRowsAndKeepsVelocityFrameUnrotated) {
    auto problem = makePhase164RawNoDopplerProblem(true, 6);
    appendPhase171SyntheticEcefDopplerRows(problem);
    std::vector<Vector3d> expected_velocities;
    std::vector<double> expected_clock_rates;
    expected_velocities.reserve(problem.native_raw_p_no_doppler_seeds.size());
    expected_clock_rates.reserve(problem.native_raw_p_no_doppler_seeds.size());
    for (std::size_t epoch = 0;
         epoch < problem.native_raw_p_no_doppler_seeds.size(); ++epoch) {
        expected_velocities.push_back(
            problem.native_raw_p_no_doppler_seeds[epoch].velocity_ecef_mps);
        expected_clock_rates.push_back(
            problem.native_raw_p_no_doppler_seeds[epoch].clock_rate_mps);
        // Do not let the graph start at the D-consistent answer.  This is a
        // genuine same-graph recovery check, while the P trajectory and D
        // rows retain the known truth above.
        const double sign = epoch % 2U == 0U ? 1.0 : -1.0;
        problem.native_raw_p_no_doppler_seeds[epoch].velocity_ecef_mps +=
            sign * Vector3d(40.0, -20.0, 10.0);
        problem.native_raw_p_no_doppler_seeds[epoch].clock_rate_mps +=
            sign * 20.0;
    }
    const auto config = makePhase171EcefDopplerConfig();
    const auto result = FGOProcessor(config).optimizeProblem(problem);

    ASSERT_TRUE(result.diagnostics.converged);
    EXPECT_TRUE(
        result.diagnostics.native_raw_p_ecef_doppler_gnss_first_enabled);
    EXPECT_FALSE(result.diagnostics.native_raw_p_no_doppler_graph_enabled);
    EXPECT_EQ(result.diagnostics.undifferenced_doppler_factors_inserted,
              problem.undifferenced_doppler_factors.size());
    ASSERT_EQ(result.epoch_velocities_ecef_mps.size(), problem.epochs.size());
    for (std::size_t epoch = 0; epoch < result.epoch_velocities_ecef_mps.size();
         ++epoch) {
        const auto& velocity = result.epoch_velocities_ecef_mps[epoch];
        EXPECT_TRUE(velocity.allFinite());
        EXPECT_LT((velocity - expected_velocities[epoch]).norm(), 1.0e-4);
        EXPECT_NEAR(result.epoch_clock_drift_mps[epoch],
                    expected_clock_rates[epoch],
                    1.0e-4);
    }

    // The dedicated factor consumes factor LOS=-e and the ECEF velocity
    // directly.  This is a sign/frame check independent of the optimizer's
    // initial values; no ECEF->ENU rotation is involved in this stage.
    const auto& row = problem.undifferenced_doppler_factors.front();
    const gtsam::Key velocity_key = gtsam::Symbol('v', row.epoch_index);
    const gtsam::Key clock_key = gtsam::Symbol('d', row.epoch_index);
    fgo_gtsam_internal::UndifferencedDopplerVelocityFactorSourceClockEcef factor(
        velocity_key, clock_key, row.los, row.residual_mps,
        gtsam::noiseModel::Isotropic::Sigma(1, row.sigma_mps));
    gtsam::Matrix H_velocity;
    gtsam::Matrix H_clock;
    const gtsam::Vector error = factor.evaluateError(
        expected_velocities[row.epoch_index],
        (gtsam::Vector(1) << expected_clock_rates[row.epoch_index])
            .finished(),
        H_velocity, H_clock);
    ASSERT_EQ(error.size(), 1);
    EXPECT_NEAR(error(0), 0.0, 1e-10);
    EXPECT_NEAR(H_velocity(0, 0), row.los(0), 1e-15);
    EXPECT_NEAR(H_velocity(0, 1), row.los(1), 1e-15);
    EXPECT_NEAR(H_velocity(0, 2), row.los(2), 1e-15);
    EXPECT_DOUBLE_EQ(H_clock(0, 0), 1.0);
}

TEST(FGOGtsamPhase171EcefDopplerGraphTest,
     MissingOrInvalidCorrectedDopplerRowsFailClosed) {
    auto missing = makePhase164RawNoDopplerProblem(true, 6);
    EXPECT_THROW(
        FGOProcessor(makePhase171EcefDopplerConfig()).optimizeProblem(missing),
        std::invalid_argument);

    auto invalid = makePhase164RawNoDopplerProblem(true, 6);
    appendPhase171SyntheticEcefDopplerRows(invalid);
    invalid.undifferenced_doppler_factors.front().residual_mps =
        std::numeric_limits<double>::quiet_NaN();
    const auto invalid_result =
        FGOProcessor(makePhase171EcefDopplerConfig()).optimizeProblem(invalid);
    EXPECT_FALSE(invalid_result.diagnostics.converged);
    EXPECT_TRUE(invalid_result.solution.isEmpty());

    auto uncorrected = makePhase164RawNoDopplerProblem(true, 6);
    appendPhase171SyntheticEcefDopplerRows(uncorrected);
    uncorrected.undifferenced_doppler_factors.front()
        .uses_rotated_satellite_state = false;
    const auto uncorrected_result =
        FGOProcessor(makePhase171EcefDopplerConfig()).optimizeProblem(uncorrected);
    EXPECT_FALSE(uncorrected_result.diagnostics.converged);
    EXPECT_TRUE(uncorrected_result.solution.isEmpty());
}

TEST(FGOGtsamPhase171EcefDopplerGraphTest,
     SelectorOffRetainsEmptyDopplerPhase164Behavior) {
    const auto problem = makePhase164RawNoDopplerProblem(true, 6);
    const auto result =
        FGOProcessor(makePhase164RawNoDopplerConfig()).optimizeProblem(problem);
    ASSERT_TRUE(result.diagnostics.converged);
    EXPECT_FALSE(
        result.diagnostics.native_raw_p_ecef_doppler_gnss_first_enabled);
    EXPECT_EQ(result.diagnostics.undifferenced_doppler_factors_inserted, 0U);
}

TEST(FGOGtsamPhase171EcefDopplerGraphTest,
     SparsePDefaultRejectsDespiteValidSeedsAndDoppler) {
    auto problem = makePhase164RawNoDopplerProblem(true, 6);
    appendPhase171SyntheticEcefDopplerRows(problem);
    std::size_t retained = 0;
    auto& rows = problem.pseudorange_factors;
    rows.erase(std::remove_if(rows.begin(), rows.end(), [&](const auto& row) {
        return row.epoch_index == 1 && ++retained > 3;
    }), rows.end());
    ASSERT_EQ(std::count_if(rows.begin(), rows.end(), [](const auto& row) {
        return row.epoch_index == 1;
    }), 3);
    ASSERT_EQ(problem.native_raw_p_no_doppler_seeds.size(), problem.epochs.size());
    ASSERT_FALSE(problem.undifferenced_doppler_factors.empty());
    const auto result = FGOProcessor(makePhase171EcefDopplerConfig()).optimizeProblem(problem);
    EXPECT_FALSE(result.diagnostics.converged);
    EXPECT_TRUE(result.solution.isEmpty());
    EXPECT_EQ(result.diagnostics.iterations, 0);
    EXPECT_EQ(result.diagnostics.native_source_clock_c0d_factor_count, 0U);
}

TEST(FGOGtsamPhase171EcefDopplerGraphTest,
     SparsePOptInConnectedGraphAndDisconnectedClockControl) {
    auto problem = makePhase164RawNoDopplerProblem(true, 6);
    appendPhase171SyntheticEcefDopplerRows(problem);
    const Vector3d expected_sparse_position = problem.epochs[1].position_ecef;
    // Perturb the missing-P epoch's initial position, not observations.
    problem.native_raw_p_no_doppler_seeds[1].position_ecef += Vector3d(3,-2,1);
    problem.epochs[1].position_ecef += Vector3d(3,-2,1);
    auto& rows = problem.pseudorange_factors;
    rows.erase(std::remove_if(rows.begin(), rows.end(), [](const auto& row) {
        return row.epoch_index == 1;
    }), rows.end());
    auto config = makePhase171EcefDopplerConfig();
    config.allow_native_raw_p_sparse_epochs = true;
    const auto result = FGOProcessor(config).optimizeProblem(problem);
    ASSERT_TRUE(result.diagnostics.converged);
    ASSERT_EQ(result.solution.size(), problem.epochs.size());
    ASSERT_EQ(result.epoch_clock_bias_components_m.size(), problem.epochs.size());
    EXPECT_LT((result.solution.solutions[1].position_ecef - expected_sparse_position).norm(), 1e-3);
    for (std::size_t i=0;i<problem.epochs.size();++i) {
        EXPECT_TRUE(result.epoch_velocities_ecef_mps[i].allFinite());
        EXPECT_LT((result.epoch_velocities_ecef_mps[i] -
                   problem.native_raw_p_no_doppler_seeds[i].velocity_ecef_mps).norm(), 1e-3);
    }
    problem.clock_jumps.resize(problem.epochs.size(), false);
    problem.clock_jumps[1] = true;
    const auto rejected = FGOProcessor(config).optimizeProblem(problem);
    EXPECT_FALSE(rejected.diagnostics.converged);
    EXPECT_TRUE(rejected.solution.isEmpty());
}

TEST(FGOGtsamPhase171EcefDopplerGraphTest,
     SparsePRejectsAllSparseAndUnsupportedMode) {
    auto problem = makePhase164RawNoDopplerProblem(true, 6);
    appendPhase171SyntheticEcefDopplerRows(problem);
    std::map<std::size_t, std::size_t> counts;
    auto& rows = problem.pseudorange_factors;
    rows.erase(std::remove_if(rows.begin(), rows.end(), [&](const auto& row) {
        return ++counts[row.epoch_index] > 3;
    }), rows.end());
    auto config = makePhase171EcefDopplerConfig();
    config.allow_native_raw_p_sparse_epochs = true;
    const auto rejected = FGOProcessor(config).optimizeProblem(problem);
    EXPECT_FALSE(rejected.diagnostics.converged);
    EXPECT_TRUE(rejected.solution.isEmpty());
    auto unsupported = makePhase164RawNoDopplerConfig();
    unsupported.allow_native_raw_p_sparse_epochs = true;
    EXPECT_THROW(FGOProcessor(unsupported).optimizeProblem(
        makePhase164RawNoDopplerProblem(true,6)), std::invalid_argument);
    auto invalid_seed = makePhase164RawNoDopplerProblem(true,6);
    appendPhase171SyntheticEcefDopplerRows(invalid_seed);
    invalid_seed.native_raw_p_no_doppler_seeds[1].has_position = false;
    EXPECT_TRUE(FGOProcessor(config).optimizeProblem(invalid_seed).solution.isEmpty());
}

TEST(FGOGtsamPhase171EcefDopplerTransferTest,
     RemapsOnlyDWithExactRawIdentityAndRejectsInvalidTransfers) {
    auto original = makePhase164RawNoDopplerProblem(true, 6);
    appendPhase171SyntheticEcefDopplerRows(original);
    const std::size_t original_p_count = original.pseudorange_factors.size();
    const std::size_t original_tdcp_count = original.tdcp_factors.size();

    auto staged_epochs = original.epochs;
    std::swap(staged_epochs[0], staged_epochs[1]);
    std::vector<FGOProcessor::UndifferencedDopplerFactor> staged_doppler =
        original.undifferenced_doppler_factors;
    for (auto& factor : staged_doppler) {
        factor.epoch_index = factor.epoch_index == 0U ? 1U : 0U;
        if (factor.previous_epoch_index !=
            std::numeric_limits<std::size_t>::max()) {
            factor.previous_epoch_index =
                factor.previous_epoch_index == 0U ? 1U : 0U;
        }
    }
    std::vector<FGOProcessor::UndifferencedDopplerFactor> remapped;
    std::string failure;
    ASSERT_TRUE(raw_p_ecef_doppler::remapDopplerFactors(
        original.epochs, staged_epochs, staged_doppler, remapped, failure))
        << failure;
    ASSERT_EQ(remapped.size(), original.undifferenced_doppler_factors.size());
    for (std::size_t i = 0; i < remapped.size(); ++i) {
        EXPECT_EQ(remapped[i].epoch_index,
                  original.undifferenced_doppler_factors[i].epoch_index);
        EXPECT_EQ(remapped[i].satellite,
                  original.undifferenced_doppler_factors[i].satellite);
        EXPECT_EQ(remapped[i].signal,
                  original.undifferenced_doppler_factors[i].signal);
    }
    EXPECT_EQ(original.pseudorange_factors.size(), original_p_count);
    EXPECT_EQ(original.tdcp_factors.size(), original_tdcp_count);

    auto duplicate_epochs = staged_epochs;
    duplicate_epochs[1] = duplicate_epochs[0];
    EXPECT_FALSE(raw_p_ecef_doppler::remapDopplerFactors(
        original.epochs, duplicate_epochs, staged_doppler, remapped, failure));
    EXPECT_NE(failure.find("duplicate staged"), std::string::npos);

    auto near_time_epochs = original.epochs;
    near_time_epochs[0].time.tow += 0.5e-6;
    EXPECT_FALSE(raw_p_ecef_doppler::remapDopplerFactors(
        original.epochs, near_time_epochs, original.undifferenced_doppler_factors,
        remapped, failure));
    EXPECT_NE(failure.find("identity/time mismatch"), std::string::npos);

    auto missing_epoch = staged_epochs;
    missing_epoch[0].raw_source_index = 999999U;
    EXPECT_FALSE(raw_p_ecef_doppler::remapDopplerFactors(
        original.epochs, missing_epoch, staged_doppler, remapped, failure));
    EXPECT_NE(failure.find("identity/time mismatch"), std::string::npos);

    auto invalid_doppler = staged_doppler;
    invalid_doppler.front().residual_mps =
        std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(raw_p_ecef_doppler::remapDopplerFactors(
        original.epochs, staged_epochs, invalid_doppler, remapped, failure));
    EXPECT_NE(failure.find("invalid staged corrected ECEF Doppler"),
              std::string::npos);
}

TEST(FGOGtsamPhase165RawNoDopplerGraphTest,
     KeepsSupportedMixedSystemRowsAndRecoversKnownPosition) {
    const FGOProcessor::FGOProblem problem =
        makePhase164RawNoDopplerProblem(true, 12, true);
    const FGOProcessor::FGOResult result =
        FGOProcessor(makePhase164RawNoDopplerConfig()).optimizeProblem(problem);

    ASSERT_TRUE(result.diagnostics.converged);
    ASSERT_EQ(result.solution.size(), 2U);
    ASSERT_EQ(result.epoch_clock_bias_components_m.size(), 2U);
    for (std::size_t epoch = 0; epoch < 2; ++epoch) {
        EXPECT_LT((result.solution.solutions[epoch].position_ecef -
                   problem.epochs[epoch].position_ecef)
                      .norm(),
                  1.0e-4);
        EXPECT_TRUE(std::all_of(
            result.epoch_clock_bias_components_m[epoch].begin(),
            result.epoch_clock_bias_components_m[epoch].end(),
            [](double value) { return std::isfinite(value); }));
    }
    // C[0..5] appear in the retained synthetic rows; the absent C[6] slot is
    // weakly gauged by the dedicated graph rather than left unconstrained.
    EXPECT_EQ(result.diagnostics
                  .native_raw_p_no_doppler_unobserved_clock_gauge_components,
              1U);
}

TEST(FGOGtsamPhase165RawNoDopplerGraphTest,
     RejectsUnsupportedClockSlotAtGraphBoundary) {
    auto problem = makePhase164RawNoDopplerProblem(true, 6);
    problem.pseudorange_factors.front().signal = SignalType::GPS_L2C;
    const auto result =
        FGOProcessor(makePhase164RawNoDopplerConfig()).optimizeProblem(problem);
    EXPECT_FALSE(result.diagnostics.converged);
    EXPECT_TRUE(result.solution.isEmpty());
}

TEST(FGOGtsamPhase171NoDopplerImuMainTest,
     SameRunC7DHandOffAdmitsEmptyDopplerAndGaugesAbsentClockSlots) {
    // Exercise the actual same-run boundary: first solve the raw-P-only
    // Point3/velocity graph, then copy only its in-memory finite exports into
    // the Pose3+IMU problem.  No serialized seed or independently fabricated
    // C/D values are used by this synthetic handoff.
    const auto run = [](const bool phase184,
                        const bool use_stop_constraints = false,
                        const Vector3d stage_velocity =
                            Vector3d(2.0, -1.5, 0.75),
                        const bool moving_imu = false,
                        const bool malformed_imu = false,
                        // 0=normal, 1=seed vector unavailable, 2=seed
                        // values nonfinite, 3=exact threshold seed.
                        const int stop_seed_case = 0,
                        const bool stage_ecef_doppler = false,
                        const bool phase201_schedule = false,
                        const bool phase205_density = false,
                        const bool phase209_separate = false,
                        const bool phase213_main_doppler = false,
                        const int phase213_bad_rows = 0,
                        const bool phase217_motion = false,
                        const bool robust_main = false,
                        const bool affine_tdcp = false,
                        const bool epoch_heading = false,
                        const bool omit_bias_prior = false,
                        const bool omit_velocity_prior = false,
                        const bool synthetic_relative_height = false,
                        const std::string& phone = "pixel5") {
    SCOPED_TRACE("handoff phone=" + phone);
    auto stage_problem = makePhase164RawNoDopplerProblem(
        true, 6, false, stage_velocity);
    auto stage_config = stage_ecef_doppler
                            ? makePhase171EcefDopplerConfig()
                            : makePhase164RawNoDopplerConfig();
    stage_config.native_source_clock_c0d_phone = phone;
    if (stage_ecef_doppler) {
        appendPhase171SyntheticEcefDopplerRows(stage_problem);
    }
    stage_config.use_native_phase184_source_tdcp_huber_k = phase184;
    stage_config.use_native_tdcp_only_affine_geometry = affine_tdcp;
    if (affine_tdcp) {
        const auto& sat = stage_problem.pseudorange_factors.front();
        FGOProcessor::TimeDifferencedCarrierFactor row;
        row.previous_epoch_index = 0;
        row.current_epoch_index = 1;
        row.satellite = sat.satellite;
        row.signal = sat.signal;
        row.previous_satellite_position_ecef = sat.satellite_position_ecef;
        row.current_satellite_position_ecef = sat.satellite_position_ecef;
        row.delta_carrier_m =
            (sat.satellite_position_ecef - stage_problem.epochs[1].position_ecef).norm() -
            (sat.satellite_position_ecef - stage_problem.epochs[0].position_ecef).norm() +
            stage_problem.epochs[1].receiver_clock_bias_m -
            stage_problem.epochs[0].receiver_clock_bias_m;
        row.sigma_m = 1.0;
        row.dt_s = 1.0;
        stage_problem.tdcp_factors.push_back(row);
        stage_config.use_tdcp_factors = true;
        ASSERT_EQ(stage_problem.tdcp_factors.size(), 1U);
    }
    stage_config.native_phase184_tdcp_setting_type = phase184 ? "Street" : "";
    const auto stage_result =
        FGOProcessor(stage_config).optimizeProblem(stage_problem);
    ASSERT_TRUE(stage_result.diagnostics.converged);
    EXPECT_EQ(stage_result.diagnostics.tdcp_only_affine_factors_inserted,
              affine_tdcp ? stage_problem.tdcp_factors.size() : 0U);
    EXPECT_EQ(
        stage_result.diagnostics.native_phase184_source_tdcp_huber_k_enabled,
        phase184);
    EXPECT_EQ(
        stage_result.diagnostics.native_raw_p_ecef_doppler_gnss_first_enabled,
        stage_ecef_doppler);
    EXPECT_DOUBLE_EQ(stage_result.diagnostics.official_tdcp_huber_threshold_sigma,
                     phase184 ? 0.2 : 4.0);
    ASSERT_EQ(stage_result.solution.size(), stage_problem.epochs.size());
    ASSERT_EQ(stage_result.epoch_velocities_ecef_mps.size(),
              stage_problem.epochs.size());
    ASSERT_EQ(stage_result.epoch_clock_drift_mps.size(),
              stage_problem.epochs.size());
    ASSERT_EQ(stage_result.epoch_clock_bias_components_m.size(),
              stage_problem.epochs.size());
    for (std::size_t i = 0; i < stage_problem.epochs.size(); ++i) {
        ASSERT_TRUE(stage_result.solution.solutions[i].position_ecef.allFinite());
        ASSERT_TRUE(std::isfinite(
            stage_result.solution.solutions[i].receiver_clock_bias));
        ASSERT_TRUE(stage_result.epoch_velocities_ecef_mps[i].allFinite());
        ASSERT_TRUE(std::isfinite(stage_result.epoch_clock_drift_mps[i]));
        ASSERT_TRUE(std::all_of(
            stage_result.epoch_clock_bias_components_m[i].begin(),
            stage_result.epoch_clock_bias_components_m[i].end(),
            [](double value) { return std::isfinite(value); }));
    }

    auto problem = stage_problem;
    problem.undifferenced_doppler_factors.clear();
    problem.tdcp_factors.clear();
    if (phase213_main_doppler) {
        std::string failure;
        ASSERT_TRUE(raw_p_ecef_doppler::remapDopplerFactors(
            problem.epochs, stage_problem.epochs,
            stage_problem.undifferenced_doppler_factors,
            problem.native_phase213_main_doppler_rows, failure)) << failure;
        ASSERT_FALSE(problem.native_phase213_main_doppler_rows.empty());
        if (phase213_bad_rows == 1) problem.native_phase213_main_doppler_rows.clear();
        if (phase213_bad_rows == 2) problem.native_phase213_main_doppler_rows.front().sigma_mps = 0.0;
        if (phase213_bad_rows == 3) problem.native_phase213_main_doppler_rows.push_back(
            problem.native_phase213_main_doppler_rows.front());
    }
    for (std::size_t i = 0; i < problem.epochs.size(); ++i) {
        // PositionSolution's public clock is seconds; the source C0/D graph
        // consumes the epoch seed clock in metres, so this is the one
        // documented boundary conversion used by the application handoff.
        problem.epochs[i].position_ecef =
            stage_result.solution.solutions[i].position_ecef;
        problem.epochs[i].receiver_clock_bias_m =
            stage_result.solution.solutions[i].receiver_clock_bias *
            constants::SPEED_OF_LIGHT;
        problem.epochs[i].receiver_clock_bias_is_meters = true;
    }
    const Vector3d receiver = problem.epochs.front().position_ecef;
    double lat = 0.0;
    double lon = 0.0;
    double height = 0.0;
    ecef2geodetic(receiver, lat, lon, height);
    problem.imu.valid = true;
    problem.imu.nav_origin_ecef = receiver;
    problem.imu.nav_origin_lat_rad = lat;
    problem.imu.nav_origin_lon_rad = lon;
    problem.imu.init_attitude_body_to_nav = Matrix3d::Identity();
    // The stage velocity is ECEF.  Convert it once to the main IMU ENU seed,
    // matching buildImuInput; the optimized C7/D handoff remains untouched.
    problem.imu.init_velocity_nav = ecef2enu(
        stage_result.epoch_velocities_ecef_mps.front(), lat, lon);
    problem.imu.stop_velocity_seeds_nav.reserve(problem.epochs.size());
    for (const auto& velocity_ecef : stage_result.epoch_velocities_ecef_mps) {
        problem.imu.stop_velocity_seeds_nav.push_back(
            ecef2enu(velocity_ecef, lat, lon));
    }
    if (stop_seed_case == 1) {
        problem.imu.stop_velocity_seeds_nav.clear();
    } else if (stop_seed_case == 2) {
        for (auto& velocity : problem.imu.stop_velocity_seeds_nav) {
            velocity.x() = std::numeric_limits<double>::quiet_NaN();
        }
    } else if (stop_seed_case == 3) {
        for (auto& velocity : problem.imu.stop_velocity_seeds_nav) {
            velocity = Vector3d(0.5, 0.0, 0.0);
        }
    }
    const double g = problem.imu.noise.gravity_mps2;
    // Index-derived times keep the Phase201 endpoint/count assertions
    // independent of cumulative decimal-step drift.
    for (std::size_t sample_index = 0; sample_index <= 20; ++sample_index) {
        const double t = static_cast<double>(sample_index) * 0.1;
        ImuSample sample;
        sample.time = problem.epochs.front().time + t;
        sample.accel_raw = Vector3d(0.0, 0.0, g);
        sample.gyro_raw_radps = moving_imu
            ? Vector3d(0.1, 0.0, 0.0)
            : Vector3d::Zero();
        problem.imu.samples_body_flu.push_back(sample);
    }
    if (malformed_imu) {
        problem.imu.samples_body_flu[3].accel_raw.x() =
            std::numeric_limits<double>::quiet_NaN();
    }
    problem.native_source_clock_c0d_gnss_first_d_handoff_mps =
        stage_result.epoch_clock_drift_mps;
    problem.native_source_clock_c0d_gnss_first_c_handoff_m =
        stage_result.epoch_clock_bias_components_m;

    // Keep one valid TDCP row in the main synthetic family while the generic
    // Doppler family is deliberately empty for Phase171.
    const auto& sat = problem.pseudorange_factors.front();
    FGOProcessor::TimeDifferencedCarrierFactor tdcp;
    tdcp.previous_epoch_index = 0;
    tdcp.current_epoch_index = 1;
    tdcp.satellite = sat.satellite;
    tdcp.signal = sat.signal;
    tdcp.previous_satellite_position_ecef = sat.satellite_position_ecef;
    tdcp.current_satellite_position_ecef = sat.satellite_position_ecef;
    tdcp.delta_carrier_m =
        (sat.satellite_position_ecef - problem.epochs[1].position_ecef).norm() -
        (sat.satellite_position_ecef - problem.epochs[0].position_ecef).norm() +
        stage_result.epoch_clock_bias_components_m[1][0] -
        stage_result.epoch_clock_bias_components_m[0][0];
    tdcp.sigma_m = 1.0;
    tdcp.dt_s = problem.epochs[1].time - problem.epochs[0].time;
    problem.tdcp_factors.push_back(tdcp);

    FGOProcessor::FGOConfig config;
    config.backend = FGOBackend::GTSAM;
    config.use_pose3_state = true;
    config.use_imu = true;
    config.use_upstream_observable_quality = true;
    config.use_undifferenced_doppler_factors = false;
    config.use_native_source_clock_c0d_factor = true;
    config.native_source_clock_c0d_phone = "pixel5";
    config.use_native_source_clock_c0d_meter_state_parity = true;
    config.use_native_source_clock_c0d_epoch_vector_parity = true;
    config.use_native_source_clock_c0d_gnss_first_meter_state_handoff = true;
    config.use_native_source_clock_c0d_active_solve_diagnostic = true;
    config.use_native_source_clock_c0d_phase99_main_multifrontal_qr_solver =
        true;
    config.use_native_phase143_official_main_lm_termination_budget = true;
    config.use_native_phase171_raw_p_no_doppler_imu_main = true;
    config.use_native_phase201_source_inclusive_forward_imu_schedule =
        phase201_schedule;
    config.use_native_phase205_source_count_bias_density = phase205_density;
    config.use_native_phase209_source_separate_imu_factors = phase209_separate;
    config.native_source_clock_c0d_phone = phone;
    config.use_native_phase213_main_doppler = phase213_main_doppler && phase213_bad_rows != 4;
    config.use_native_phase217_main_pose3_motion = phase217_motion;
    if (phase217_motion) config.use_position_motion_factors = false;
    config.use_upstream_stop_constraints = use_stop_constraints;
    // Use the production contract in the full handoff as well; the short
    // synthetic stream still exercises the endpoint behavior without a
    // second detector configuration.
    config.upstream_stop_window_samples = 500;
    config.use_native_phase184_source_tdcp_huber_k = phase184;
    config.native_phase184_tdcp_setting_type = phase184 ? "Street" : "";
    config.use_inter_system_biases = false;
    config.use_receiver_signal_bias_states = false;
    config.use_residual_ionosphere_states = false;
    config.use_tdcp_factors = true;
    config.use_robust_loss = robust_main;
    config.use_native_tdcp_only_affine_geometry = affine_tdcp;
    config.use_native_epoch_heading_attitude_seeds = epoch_heading;
    config.omit_native_first_imu_bias_prior = omit_bias_prior;
    config.omit_native_first_imu_velocity_prior = omit_velocity_prior;
    config.use_native_relative_height_pairs = synthetic_relative_height;
    if (synthetic_relative_height) {
        // Explicit synthetic insertion control, NOT a same-run velocity
        // provenance test: ensure the tiny two-epoch fixture crosses the
        // source cumulative-speed threshold without moving its positions.
        for (auto& v : problem.imu.stop_velocity_seeds_nav)
            v = Vector3d(101.0, 0.0, 0.0);
    }
    if (omit_velocity_prior) {
        // Deliberately separate the prior target from the same-run velocity
        // state seed. Both reference and ablation receive this same problem.
        problem.imu.init_velocity_nav += Vector3d(0.4, -0.2, 0.1);
    }
    if (epoch_heading) {
        for (std::size_t i = 0; i < problem.epochs.size(); ++i) {
            problem.imu.epoch_heading_attitudes_body_to_nav.push_back(
                gtsam::Rot3::Rz(0.1 * static_cast<double>(i)).matrix());
            problem.imu.epoch_heading_attitude_times.push_back(problem.epochs[i].time);
        }
    }
    config.max_iterations = 12;

    if (phase217_motion && (phase201_schedule || phase205_density || phase209_separate)) {
        EXPECT_THROW(FGOProcessor(config).optimizeProblem(problem), std::invalid_argument);
        return;
    }
    if (phase213_main_doppler && (phase213_bad_rows != 0 || phase201_schedule ||
                                  phase205_density || phase209_separate)) {
        EXPECT_THROW(FGOProcessor(config).optimizeProblem(problem), std::invalid_argument);
        return;
    }
    if ((phase205_density && (phase201_schedule || malformed_imu)) ||
        (phase209_separate && (phase201_schedule || phase205_density || malformed_imu))) {
        EXPECT_THROW(FGOProcessor(config).optimizeProblem(problem), std::invalid_argument);
        return;
    }
    const auto result = FGOProcessor(config).optimizeProblem(problem);
    if (malformed_imu) {
        EXPECT_FALSE(result.diagnostics.converged);
        EXPECT_TRUE(result.solution.isEmpty());
        if (phase201_schedule) {
            EXPECT_FALSE(result.diagnostics
                             .native_phase201_source_inclusive_forward_imu_schedule_configuration_valid);
            EXPECT_EQ(result.diagnostics.native_phase201_invalid_sample_count, 1U);
        }
        return;
    }
    ASSERT_TRUE(result.diagnostics.converged);
    EXPECT_TRUE(result.diagnostics.native_phase171_no_doppler_imu_main_enabled);
    EXPECT_EQ(result.diagnostics
                  .native_phase201_source_inclusive_forward_imu_schedule_enabled,
              phase201_schedule);
    EXPECT_EQ(result.diagnostics
                  .native_phase201_source_inclusive_forward_imu_schedule_attempted,
              phase201_schedule);
    EXPECT_EQ(result.diagnostics.native_phase184_source_tdcp_huber_k_enabled,
              phase184);
    EXPECT_DOUBLE_EQ(result.diagnostics.official_tdcp_huber_threshold_sigma,
                     phase184 ? 0.2 : 4.0);
    EXPECT_EQ(result.diagnostics.undifferenced_doppler_factors_inserted, 0U);
    EXPECT_EQ(result.diagnostics.tdcp_factors_inserted, 1U);
    EXPECT_EQ(result.diagnostics.tdcp_only_affine_factors_inserted,
              affine_tdcp ? 1U : 0U);
    EXPECT_EQ(result.diagnostics.epoch_heading_attitude_seeds_inserted,
              epoch_heading ? problem.epochs.size() : 0U);
    if (epoch_heading) {
        auto second_problem = problem;
        second_problem.imu.refinement_velocity_seeds_nav = result.epoch_velocity_nav_mps;
        const auto second = FGOProcessor(config).optimizeProblem(second_problem);
        EXPECT_TRUE(second.diagnostics.converged);
        EXPECT_EQ(second.solution.solutions.size(), problem.epochs.size());
        second_problem.imu.refinement_velocity_seeds_nav.pop_back();
        EXPECT_THROW(FGOProcessor(config).optimizeProblem(second_problem), std::invalid_argument);
        second_problem.imu.refinement_velocity_seeds_nav = result.epoch_velocity_nav_mps;
        second_problem.imu.refinement_velocity_seeds_nav[0].x() =
            std::numeric_limits<double>::quiet_NaN();
        EXPECT_THROW(FGOProcessor(config).optimizeProblem(second_problem), std::invalid_argument);
        second_problem.imu.refinement_velocity_seeds_nav = result.epoch_velocity_nav_mps;
        second_problem.imu.epoch_heading_attitude_times[0] = problem.epochs[0].time + 0.001;
        EXPECT_THROW(FGOProcessor(config).optimizeProblem(second_problem), std::invalid_argument);
    }
    EXPECT_EQ(result.diagnostics.first_imu_bias_priors_inserted, omit_bias_prior ? 0U : 1U);
    EXPECT_EQ(result.diagnostics.first_imu_bias_priors_omitted, omit_bias_prior ? 1U : 0U);
    EXPECT_EQ(result.diagnostics.first_imu_velocity_priors_inserted, omit_velocity_prior ? 0U : 1U);
    EXPECT_EQ(result.diagnostics.first_imu_velocity_priors_omitted, omit_velocity_prior ? 1U : 0U);
    if (synthetic_relative_height) {
        EXPECT_GT(result.diagnostics.relative_height_pairs_selected, 0U);
        EXPECT_EQ(result.diagnostics.relative_height_factors_inserted,
                  result.diagnostics.relative_height_pairs_selected);
        auto reference_config = config;
        reference_config.use_native_relative_height_pairs = false;
        const auto reference = FGOProcessor(reference_config).optimizeProblem(problem);
        EXPECT_TRUE(reference.diagnostics.converged);
        EXPECT_EQ(reference.diagnostics.relative_height_factors_inserted, 0U);
    } else {
        EXPECT_EQ(result.diagnostics.relative_height_factors_inserted, 0U);
    }
    if (omit_velocity_prior) {
        auto reference_config = config;
        reference_config.omit_native_first_imu_velocity_prior = false;
        const auto reference = FGOProcessor(reference_config).optimizeProblem(problem);
        ASSERT_TRUE(reference.diagnostics.converged);
        EXPECT_EQ(result.diagnostics.graph_factors + 1U, reference.diagnostics.graph_factors);
        EXPECT_EQ(result.diagnostics.first_imu_bias_priors_inserted, 1U);
        EXPECT_EQ(result.diagnostics.imu_intervals, reference.diagnostics.imu_intervals);
        EXPECT_EQ(result.diagnostics.native_phase213_main_doppler_factors,
                  reference.diagnostics.native_phase213_main_doppler_factors);
    }
    if (omit_bias_prior) {
        auto reference_config = config;
        reference_config.omit_native_first_imu_bias_prior = false;
        const auto reference = FGOProcessor(reference_config).optimizeProblem(problem);
        ASSERT_TRUE(reference.diagnostics.converged);
        EXPECT_EQ(result.diagnostics.graph_factors + 1U, reference.diagnostics.graph_factors);
        EXPECT_EQ(result.diagnostics.imu_intervals, reference.diagnostics.imu_intervals);
        EXPECT_EQ(result.diagnostics.tdcp_factors_inserted, reference.diagnostics.tdcp_factors_inserted);
    }
    // A205U's source recipe deliberately omits inter-epoch clock factors.
    EXPECT_EQ(result.diagnostics.native_source_clock_c0d_factor_count,
              phone == "sm-a205u" ? 0U : 1U);
    EXPECT_EQ(result.diagnostics.imu_intervals, 1U);
    EXPECT_EQ(result.diagnostics.native_phase205_bias_density_enabled, phase205_density);
    EXPECT_EQ(result.diagnostics.native_phase217_main_motion_enabled, phase217_motion);
    EXPECT_EQ(result.diagnostics.native_phase217_main_motion_factors, phase217_motion ? 1U : 0U);
    EXPECT_EQ(result.diagnostics.native_phase217_main_motion_gap_skips, 0U);
    EXPECT_EQ(result.diagnostics.native_phase213_main_doppler_enabled, phase213_main_doppler);
    EXPECT_EQ(result.diagnostics.native_phase213_main_doppler_factors,
              phase213_main_doppler ? stage_problem.undifferenced_doppler_factors.size() : 0U);
    EXPECT_EQ(result.diagnostics.native_phase209_separate_imu_enabled, phase209_separate);
    EXPECT_EQ(result.diagnostics.native_phase209_motion_factors, phase209_separate ? 1U : 0U);
    EXPECT_EQ(result.diagnostics.native_phase209_bias_factors, phase209_separate ? 1U : 0U);
    EXPECT_EQ(result.diagnostics.native_phase209_inclusive_samples, phase209_separate ? 11U : 0U);
    if (phase205_density) {
        EXPECT_EQ(result.diagnostics.native_phase205_bias_density_intervals, 1U);
        EXPECT_EQ(result.diagnostics.native_phase205_bias_density_samples, 11U);
        EXPECT_NEAR(result.diagnostics.native_phase205_bias_density_scale_min, 11.0, 1e-9);
        EXPECT_NEAR(result.diagnostics.native_phase205_bias_density_scale_max, 11.0, 1e-9);
    }
    if (phase201_schedule) {
        EXPECT_TRUE(result.diagnostics
                        .native_phase201_source_inclusive_forward_imu_schedule_configuration_valid);
        EXPECT_EQ(result.diagnostics.native_phase201_intervals, 1U);
        EXPECT_EQ(result.diagnostics.native_phase201_intervals_with_samples, 1U);
        EXPECT_EQ(result.diagnostics.native_phase201_inserted_samples, 11U);
        EXPECT_NEAR(result.diagnostics.native_phase201_gnss_interval_duration_min_s,
                    1.0, 1e-9);
        EXPECT_NEAR(result.diagnostics.native_phase201_integrated_duration_min_s,
                    1.1, 1e-9);
        EXPECT_NEAR(result.diagnostics.native_phase201_duration_error_max_abs_s,
                    0.1, 1e-9);
    } else {
        EXPECT_EQ(result.diagnostics.native_phase201_intervals, 0U);
        EXPECT_EQ(result.diagnostics.native_phase201_inserted_samples, 0U);
    }
    EXPECT_EQ(result.diagnostics.native_phase171_unobserved_clock_gauge_components,
              6U);
    EXPECT_DOUBLE_EQ(
        result.diagnostics.native_phase171_unobserved_clock_gauge_sigma_m, 1.0e6);
    ASSERT_EQ(result.solution.size(), problem.epochs.size());
    for (const auto& solution : result.solution.solutions) {
        EXPECT_TRUE(solution.position_ecef.allFinite());
        EXPECT_TRUE(std::isfinite(solution.receiver_clock_bias));
    }
    ASSERT_EQ(result.epoch_velocity_nav_mps.size(), problem.epochs.size());
    for (const auto& velocity : result.epoch_velocity_nav_mps) {
        EXPECT_TRUE(velocity.allFinite());
    }
    ASSERT_EQ(result.epoch_clock_drift_mps.size(), problem.epochs.size());
    ASSERT_EQ(result.epoch_clock_bias_components_m.size(), problem.epochs.size());
    for (const double drift : result.epoch_clock_drift_mps) {
        EXPECT_TRUE(std::isfinite(drift));
    }
    for (const auto& clock : result.epoch_clock_bias_components_m) {
        EXPECT_TRUE(std::all_of(clock.begin(), clock.end(),
                                [](double value) {
                                    return std::isfinite(value);
                                }));
    }
    EXPECT_EQ(result.diagnostics.native_phase143_termination.stage, "main");
    EXPECT_EQ(result.diagnostics.native_phase143_termination.configured_max_iterations,
              12U);
    EXPECT_EQ(result.diagnostics.native_phase143_termination.effective_max_iterations,
              1000U);
    if (use_stop_constraints) {
        EXPECT_EQ(result.diagnostics.upstream_stop_epochs,
                  moving_imu ? 0U : 2U);
        const bool observed_slow_seed = std::all_of(
            stage_result.epoch_velocities_ecef_mps.begin(),
            stage_result.epoch_velocities_ecef_mps.end(),
            [](const Vector3d& velocity) {
                return velocity.allFinite() && velocity.norm() < 0.5;
            });
        const bool observed_fast_seed = std::any_of(
            stage_result.epoch_velocities_ecef_mps.begin(),
            stage_result.epoch_velocities_ecef_mps.end(),
            [](const Vector3d& velocity) {
                return velocity.allFinite() && velocity.norm() >= 0.5;
            });
        const bool observed_slow_gate =
            stop_seed_case == 3 ? false : observed_slow_seed;
        if (stage_velocity.norm() < 0.5) {
            EXPECT_TRUE(observed_slow_seed);
        } else {
            EXPECT_TRUE(observed_fast_seed);
        }
        EXPECT_EQ(result.diagnostics.upstream_stop_velocity_factors,
                  (!moving_imu && observed_slow_gate) ? 2U : 0U);
        EXPECT_EQ(result.diagnostics.upstream_stop_pose_factors,
                  (!moving_imu && observed_slow_gate) ? 1U : 0U);
        if (!moving_imu && observed_slow_gate && stop_seed_case == 0) {
            auto phase_config = config;
            phase_config.native_imu_stop_phase = NativeImuStopPhase::Initialization;
            const auto first = FGOProcessor(phase_config).optimizeProblem(problem);
            ASSERT_TRUE(first.diagnostics.converged);
            EXPECT_EQ(first.diagnostics.upstream_stop_velocity_factors, 0U);
            EXPECT_EQ(first.diagnostics.upstream_stop_velocity_phase_omissions, 2U);
            EXPECT_EQ(first.diagnostics.upstream_stop_pose_factors, 1U);
            EXPECT_EQ(first.diagnostics.upstream_stop_pose_gap_rejections, 0U);
            EXPECT_EQ(first.diagnostics.graph_factors + 2U, result.diagnostics.graph_factors);
            phase_config.native_imu_stop_phase = NativeImuStopPhase::Final;
            const auto final = FGOProcessor(phase_config).optimizeProblem(problem);
            ASSERT_TRUE(final.diagnostics.converged);
            EXPECT_EQ(final.diagnostics.upstream_stop_velocity_factors, 2U);
            EXPECT_EQ(final.diagnostics.upstream_stop_velocity_phase_omissions, 0U);
            EXPECT_EQ(final.diagnostics.upstream_stop_pose_factors, 1U);
            EXPECT_EQ(final.diagnostics.graph_factors, result.diagnostics.graph_factors);
            // Keep GPS/IMU epochs one second apart: the source pose gate
            // explicitly uses raw UTC, including the strict 1500 ms edge.
            for (const std::int64_t utc_gap_ms : {1499LL, 1500LL}) {
                auto boundary = problem;
                boundary.epochs[1].raw_utc_time_millis =
                    boundary.epochs[0].raw_utc_time_millis + utc_gap_ms;
                boundary.native_raw_p_no_doppler_seeds[1].raw_utc_time_millis =
                    boundary.epochs[1].raw_utc_time_millis;
                const auto checked = FGOProcessor(phase_config).optimizeProblem(boundary);
                ASSERT_TRUE(checked.diagnostics.converged);
                EXPECT_EQ(checked.diagnostics.upstream_stop_velocity_factors, 2U);
                EXPECT_EQ(checked.diagnostics.upstream_stop_pose_factors, utc_gap_ms < 1500 ? 1U : 0U);
                EXPECT_EQ(checked.diagnostics.upstream_stop_pose_gap_rejections, utc_gap_ms < 1500 ? 0U : 1U);
            }
            auto invalid = problem;
            invalid.imu.stop_velocity_seeds_nav.pop_back();
            EXPECT_THROW(FGOProcessor(phase_config).optimizeProblem(invalid), std::invalid_argument);
            invalid = problem;
            invalid.imu.stop_velocity_seeds_nav[0].x() = std::numeric_limits<double>::quiet_NaN();
            EXPECT_THROW(FGOProcessor(phase_config).optimizeProblem(invalid), std::invalid_argument);
            phase_config.use_upstream_stop_constraints = false;
            EXPECT_THROW(FGOProcessor(phase_config).optimizeProblem(problem), std::invalid_argument);
        }
        EXPECT_EQ(result.diagnostics.upstream_stop_velocity_key_missing_epochs,
                  0U);
        EXPECT_EQ(result.diagnostics.upstream_stop_graph_velocity_nonfinite_epochs,
                  0U);
        EXPECT_EQ(result.diagnostics.upstream_stop_speed_nonfinite_epochs, 0U);
        EXPECT_EQ(result.diagnostics.upstream_stop_speed_evaluated_epochs,
                  result.diagnostics.upstream_stop_epochs);
        EXPECT_EQ(
            result.diagnostics.upstream_stop_speed_gate_accepted_epochs +
                result.diagnostics.upstream_stop_speed_gate_rejected_epochs,
            result.diagnostics.upstream_stop_epochs);
        EXPECT_EQ(
            result.diagnostics.upstream_stop_epochs,
            result.diagnostics.upstream_stop_velocity_key_missing_epochs +
                result.diagnostics.upstream_stop_graph_velocity_nonfinite_epochs +
                result.diagnostics.upstream_stop_speed_nonfinite_epochs +
                result.diagnostics.upstream_stop_speed_gate_accepted_epochs +
                result.diagnostics.upstream_stop_speed_gate_rejected_epochs);
        EXPECT_EQ(result.diagnostics.upstream_stop_seed_unavailable_epochs,
                  stop_seed_case == 1
                      ? result.diagnostics.upstream_stop_epochs
                      : 0U);
        EXPECT_EQ(result.diagnostics.upstream_stop_seed_nonfinite_epochs,
                  stop_seed_case == 2
                      ? result.diagnostics.upstream_stop_epochs
                      : 0U);
        EXPECT_EQ(result.diagnostics.upstream_stop_graph_velocity_fallback_epochs,
                  (stop_seed_case == 1 || stop_seed_case == 2)
                      ? result.diagnostics.upstream_stop_epochs
                      : 0U);
        if (stop_seed_case == 3) {
            // The gate is explicitly >= threshold, including equality.
            EXPECT_EQ(result.diagnostics.upstream_stop_speed_gate_accepted_epochs,
                      0U);
            EXPECT_EQ(result.diagnostics.upstream_stop_speed_gate_rejected_epochs,
                      result.diagnostics.upstream_stop_epochs);
            EXPECT_DOUBLE_EQ(result.diagnostics.upstream_stop_speed_min_mps, 0.5);
            EXPECT_DOUBLE_EQ(result.diagnostics.upstream_stop_speed_max_mps, 0.5);
        }
    } else {
        EXPECT_EQ(result.diagnostics.upstream_stop_velocity_factors, 0U);
        EXPECT_EQ(result.diagnostics.upstream_stop_pose_factors, 0U);
        EXPECT_EQ(result.diagnostics.upstream_stop_velocity_key_missing_epochs,
                  0U);
        EXPECT_EQ(result.diagnostics.upstream_stop_seed_unavailable_epochs, 0U);
        EXPECT_EQ(result.diagnostics.upstream_stop_seed_nonfinite_epochs, 0U);
        EXPECT_EQ(result.diagnostics.upstream_stop_graph_velocity_fallback_epochs,
                  0U);
        EXPECT_EQ(result.diagnostics.upstream_stop_speed_evaluated_epochs, 0U);
    }
    if (robust_main && phase217_motion && !affine_tdcp && !epoch_heading &&
        !omit_bias_prior && !omit_velocity_prior && !synthetic_relative_height &&
        !use_stop_constraints && !phase184) {
        EXPECT_EQ(result.diagnostics.tdcp_frequency_residual_states, 0U);
        EXPECT_TRUE(result.tdcp_frequency_corrections.empty());
        auto paired_problem = problem;
        ASSERT_EQ(paired_problem.tdcp_factors.size(), 1U);
        auto second_band = paired_problem.tdcp_factors.front();
        ASSERT_EQ(second_band.signal, SignalType::GPS_L1CA);
        second_band.signal = SignalType::GPS_L5;
        paired_problem.tdcp_factors.push_back(second_band);
        // Inject a synthetic frequency-dependent discrepancy so the diagnostic
        // agreement check exercises nonzero corrections, not just zero-state parity.
        paired_problem.tdcp_factors[0].delta_carrier_m -= .03;
        paired_problem.tdcp_factors[1].delta_carrier_m -=
            std::pow(1575.42/1176.45,2)*.03;
        paired_problem.clock_jumps.assign(paired_problem.epochs.size(), false);
        auto paired_config = config;
        paired_config.use_native_tdcp_frequency_residual_states = true;
        paired_config.native_tdcp_frequency_residual_prior_sigma_m = .01;
        const auto paired = FGOProcessor(paired_config).optimizeProblem(paired_problem);
        ASSERT_TRUE(paired.diagnostics.converged);
        EXPECT_EQ(paired.diagnostics.tdcp_frequency_residual_states, 1U);
        EXPECT_EQ(paired.diagnostics.tdcp_frequency_residual_factors, 2U);
        EXPECT_EQ(paired.diagnostics.tdcp_frequency_residual_priors, 1U);
        EXPECT_EQ(paired.diagnostics.optimized_imu_bias_count, paired_problem.epochs.size());
        EXPECT_EQ(paired.diagnostics.nominal_p_information_epochs, paired_problem.epochs.size());
        EXPECT_LE(paired.diagnostics.nominal_p_information_rank_deficient_epochs,
                  paired_problem.epochs.size());
        EXPECT_GE(paired.diagnostics.nominal_p_information_min_eigenvalue_per_m2, 0.0);
        EXPECT_TRUE(std::isfinite(paired.diagnostics.optimized_accel_bias_max_norm_mps2));
        EXPECT_TRUE(std::isfinite(paired.diagnostics.optimized_gyro_bias_max_norm_radps));
        EXPECT_EQ(paired.diagnostics.tdcp_factors_inserted, 2U);
        EXPECT_TRUE(std::isfinite(paired.diagnostics.tdcp_residual_rms_m));
        ASSERT_EQ(paired.tdcp_frequency_corrections.size(), paired_problem.tdcp_factors.size());
        EXPECT_GT(std::abs(paired.tdcp_frequency_corrections[0].alpha_slant_change_m),1e-10)
            << "iterations=" << paired.diagnostics.iterations
            << " initial_cost=" << paired.diagnostics.initial_cost
            << " final_cost=" << paired.diagnostics.final_cost;
        double reconstructed_squared = 0.0;
        for (std::size_t index=0; index<paired_problem.tdcp_factors.size(); ++index) {
            const auto& f=paired_problem.tdcp_factors[index];
            const auto& correction=paired.tdcp_frequency_corrections[index];
            EXPECT_EQ(correction.previous_epoch_index,f.previous_epoch_index);
            EXPECT_EQ(correction.current_epoch_index,f.current_epoch_index);
            EXPECT_EQ(correction.satellite,f.satellite);
            EXPECT_EQ(correction.signal,f.signal);
            ASSERT_TRUE(std::isfinite(correction.alpha_slant_change_m));
            const auto& before=paired.solution.solutions[f.previous_epoch_index];
            const auto& after=paired.solution.solutions[f.current_epoch_index];
            const double r=(f.current_satellite_position_ecef-after.position_ecef).norm()+
                constants::SPEED_OF_LIGHT*after.receiver_clock_bias-
                (f.previous_satellite_position_ecef-before.position_ecef).norm()-
                constants::SPEED_OF_LIGHT*before.receiver_clock_bias-f.delta_carrier_m-
                correction.alpha_slant_change_m;
            reconstructed_squared+=r*r;
        }
        EXPECT_NEAR(std::sqrt(reconstructed_squared/paired_problem.tdcp_factors.size()),
                    paired.diagnostics.tdcp_residual_rms_m,1e-7);
        auto unsupported = paired_config;
        unsupported.use_fixed_lag_smoother = true;
        EXPECT_THROW(FGOProcessor(unsupported).optimizeProblem(paired_problem), std::invalid_argument);
        unsupported = paired_config;
        unsupported.use_imu = false;
        EXPECT_THROW(FGOProcessor(unsupported).optimizeProblem(paired_problem), std::invalid_argument);
        unsupported = paired_config;
        unsupported.use_source_tdcp_resl_observable = true;
        EXPECT_THROW(FGOProcessor(unsupported).optimizeProblem(paired_problem), std::invalid_argument);
        paired_config.native_tdcp_frequency_residual_prior_sigma_m = 0.;
        EXPECT_THROW(FGOProcessor(paired_config).optimizeProblem(paired_problem),
                     std::invalid_argument);
    }
    };
    run(false);
    run(true);
    // Corrected ECEF Doppler rows are a geometric/unit contract, independent
    // of phone metadata. Exercise actual stage-to-IMU solves for both clock
    // recipes while retaining all malformed-row rejection cases below.
    for (const std::string phone : {"mi8", "sm-a205u"}) {
        run(false, false, Vector3d(2.0, -1.5, 0.75), false, false, 0,
            true, false, false, false, true, 0, false, true,
            false, false, false, false, false, phone);
    }
    // Explicitly exercise the paired residual-state branch: Phase184 OFF,
    // ECEF staging, Phase213/217 ON, robust main, no other ablation.
    run(false, false, Vector3d(2.0, -1.5, 0.75), false, false, 0,
        true, false, false, false, true, 0, true, true);
    run(false, true, Vector3d(2.0, -1.5, 0.75), true, false, 0,
        true, false, false, false, true, 0, true, true,
        false, false, false, false, true);
    run(false, false, Vector3d(2.0, -1.5, 0.75), false, false, 0,
        true, false, false, false, true, 0, true, true, false, false, false, true);
    run(false, false, Vector3d(2.0, -1.5, 0.75), false, false, 0,
        true, false, false, false, true, 0, true, true, false, false, true);
    run(false, false, Vector3d(2.0, -1.5, 0.75), false, false, 0,
        true, false, false, false, true, 0, true, true, false, true);
    // Isolate the geometry selector with the production D/motion family,
    // using only the same-run native stage handoff in both graphs.
    run(false, false, Vector3d(2.0, -1.5, 0.75), false, false, 0,
        true, false, false, false, true, 0, true, true, true);
    // The same slow handoff with the selector off is the legacy/default
    // control: no upstream stop factors may appear.
    run(false, false, Vector3d(0.1, 0.0, 0.0));
    // Quiet IMU plus a valid slow same-run GNSS velocity seed inserts the
    // upstream stop constraints in the actual Pose3+IMU graph.
    run(false, true, Vector3d(0.1, 0.0, 0.0));
    // A valid fast handoff is outside the source velocity gate.
    run(false, true);
    // A moving gyro stream suppresses stop epochs even with a slow seed.
    run(false, true, Vector3d(0.1, 0.0, 0.0), true);
    // Malformed raw IMU input fails closed before graph admission.
    run(false, true, Vector3d(0.1, 0.0, 0.0), false, true);
    // A missing same-run seed vector is reported while the established graph
    // velocity fallback remains subject to the unchanged speed gate.
    run(false, true, Vector3d(0.1, 0.0, 0.0), false, false, 1);
    // Nonfinite same-run seeds are distinguished from a missing vector.
    run(false, true, Vector3d(0.1, 0.0, 0.0), false, false, 2);
    // Equality at the configured 0.5 m/s threshold is rejected (>=).
    run(false, true, Vector3d(0.1, 0.0, 0.0), false, false, 3);
    // The same main handoff also accepts the opt-in ECEF-D GNSS-first stage;
    // only the stage problem carries D rows, and the main graph remains empty-D.
    run(false, false, Vector3d(2.0, -1.5, 0.75), false, false, 0, true);
    // Phase201 exercises the actual staged-result -> Pose3/IMU handoff with
    // the source-inclusive schedule in the main graph.  The staging graph
    // remains on its legacy schedule and the main generic-D family is empty.
    run(false, false, Vector3d(2.0, -1.5, 0.75), false, false, 0, true, true);
    // The new branch fails closed on a malformed selected sample rather than
    // silently falling back to the legacy tail schedule.
    run(false, false, Vector3d(2.0, -1.5, 0.75), false, true, 0, true, true);
    run(false, false, Vector3d(2.0, -1.5, 0.75), false, false, 0, true, false, true);
    run(false, false, Vector3d(2.0, -1.5, 0.75), false, false, 0, true, true, true);
    run(false, false, Vector3d(2.0, -1.5, 0.75), false, true, 0, true, false, true);
    run(false, false, Vector3d(2.0, -1.5, 0.75), false, false, 0, true, false, false, true);
    run(false, false, Vector3d(2.0, -1.5, 0.75), false, false, 0, true, true, false, true);
    run(false, false, Vector3d(2.0, -1.5, 0.75), false, false, 0, true, false, true, true);
    run(false, false, Vector3d(2.0, -1.5, 0.75), false, true, 0, true, false, false, true);
    for (int bad_rows = 0; bad_rows <= 4; ++bad_rows) {
        run(false, false, Vector3d(2.0, -1.5, 0.75), false, false, 0,
            true, false, false, false, true, bad_rows);
    }
    run(false, false, Vector3d(2.0, -1.5, 0.75), false, false, 0,
        true, false, false, false, false, 0, true);
    run(false, false, Vector3d(2.0, -1.5, 0.75), false, false, 0,
        true, false, false, false, true, 0, true);
    // Source TDCP robustness in both stages with main Doppler and motion.
    run(true, false, Vector3d(2.0, -1.5, 0.75), false, false, 0,
        true, false, false, false, true, 0, true);
    // Also exercise the production robust-loss path, retaining the OFF control.
    run(true, false, Vector3d(2.0, -1.5, 0.75), false, false, 0,
        true, false, false, false, true, 0, true, true);
    run(false, false, Vector3d(2.0, -1.5, 0.75), false, false, 0,
        true, true, false, false, true);
    run(false, false, Vector3d(2.0, -1.5, 0.75), false, false, 0,
        true, false, true, false, true);
    run(false, false, Vector3d(2.0, -1.5, 0.75), false, false, 0,
        true, false, false, true, true);
}

TEST(FGOGtsamPhase171NoDopplerImuMainTest,
     RejectsMissingNonfiniteClockHandoffAndLeavesLegacySelectorOff) {
    const auto make_problem = [] {
        auto problem = makePhase93GnssFirstClockStateProblem();
        problem.undifferenced_doppler_factors.clear();
        const Vector3d receiver = problem.epochs.front().position_ecef;
        double lat = 0.0;
        double lon = 0.0;
        double height = 0.0;
        ecef2geodetic(receiver, lat, lon, height);
        problem.imu.valid = true;
        problem.imu.nav_origin_ecef = receiver;
        problem.imu.nav_origin_lat_rad = lat;
        problem.imu.nav_origin_lon_rad = lon;
        problem.imu.init_attitude_body_to_nav = Matrix3d::Identity();
        problem.native_source_clock_c0d_gnss_first_d_handoff_mps = {0.25, 0.35};
        FGOProcessor::EpochClockBiasComponentsM c0{};
        FGOProcessor::EpochClockBiasComponentsM c1{};
        c0[0] = 12.0;
        c1[0] = 13.0;
        problem.native_source_clock_c0d_gnss_first_c_handoff_m = {c0, c1};
        return problem;
    };
    const auto make_config = [] {
        FGOProcessor::FGOConfig config;
        config.backend = FGOBackend::GTSAM;
        config.use_pose3_state = true;
        config.use_imu = true;
        config.use_upstream_observable_quality = true;
        config.use_undifferenced_doppler_factors = false;
        config.use_native_source_clock_c0d_factor = true;
        config.native_source_clock_c0d_phone = "pixel5";
        config.use_native_source_clock_c0d_meter_state_parity = true;
        config.use_native_source_clock_c0d_epoch_vector_parity = true;
        config.use_native_source_clock_c0d_gnss_first_meter_state_handoff = true;
        config.use_native_source_clock_c0d_phase99_main_multifrontal_qr_solver =
            true;
        config.use_native_phase171_raw_p_no_doppler_imu_main = true;
        config.use_inter_system_biases = false;
        config.use_receiver_signal_bias_states = false;
        config.use_residual_ionosphere_states = false;
        config.use_tdcp_factors = false;
        config.max_iterations = 12;
        return config;
    };

    auto missing_d = make_problem();
    missing_d.native_source_clock_c0d_gnss_first_d_handoff_mps.pop_back();
    auto result = FGOProcessor(make_config()).optimizeProblem(missing_d);
    EXPECT_FALSE(result.diagnostics.converged);
    EXPECT_TRUE(result.solution.isEmpty());

    auto nonfinite_c = make_problem();
    nonfinite_c.native_source_clock_c0d_gnss_first_c_handoff_m[1][2] =
        std::numeric_limits<double>::quiet_NaN();
    result = FGOProcessor(make_config()).optimizeProblem(nonfinite_c);
    EXPECT_FALSE(result.diagnostics.converged);
    EXPECT_TRUE(result.solution.isEmpty());

    EXPECT_FALSE(FGOProcessor::FGOConfig{}
                     .use_native_phase171_raw_p_no_doppler_imu_main);
}

TEST(FGOGtsamPhase167RawNoDopplerTerminationTest,
     UsesDedicated1000BudgetAndDoesNotRelabelCapAsTolerance) {
    auto config = makePhase164RawNoDopplerConfig();
    config.max_iterations = 1000;
    config.use_native_phase167_raw_p_no_doppler_lm_termination_budget = true;
    const auto problem = makePhase164RawNoDopplerProblem(true, 6);
    const auto result = FGOProcessor(config).optimizeProblem(problem);
    const auto& termination = result.diagnostics.native_phase143_termination;

    EXPECT_TRUE(result.diagnostics.converged);
    EXPECT_TRUE(termination.selector_enabled);
    EXPECT_EQ(termination.stage, "gnss-first");
    EXPECT_EQ(termination.configured_max_iterations, 1000U);
    EXPECT_EQ(termination.effective_max_iterations, 1000U);
    EXPECT_TRUE(termination.configuration_valid);
    EXPECT_TRUE(termination.termination_trace_complete);
    EXPECT_TRUE(fgo_gtsam_internal::validatePhase143TerminationDiagnostics(
        termination));
    EXPECT_FALSE(termination.termination_branch.empty());
    EXPECT_EQ(termination.parsed_trial_count,
              termination.expected_trial_count);
    EXPECT_EQ(
        termination.expected_trial_count,
        termination.native_inner_iterations +
            (termination.termination_branch == "small_cost_change" ? 1U : 0U));
    if (termination.termination_branch == "maximum_outer_iterations") {
        EXPECT_EQ(termination.accepted_outer_iterations,
                  termination.effective_max_iterations);
    } else {
        EXPECT_NE(termination.termination_branch,
                  "maximum_outer_iterations");
    }
}

TEST(FGOGtsamPhase167RawNoDopplerTerminationTest,
     SummaryParserCountsNonfiniteNativeTrialRows) {
    using Optimizer = fgo_gtsam_internal::GtsamLmActiveSolveOptimizer;
    const std::string trace =
        "0 inf 0 1e-05 0 0.01\n"
        "1 nan 0 1e-04 1 0.01\n";
    // GTSAM emits `inf`/`nan` for a failed trial.  These rows remain counted
    // as diagnostic evidence, while the strict scalar contract still rejects
    // their nonfinite cost fields; they must never become a successful solve.
    EXPECT_EQ(Optimizer::parseAttemptsCountForTesting(trace), 2U);
    // A nonfinite nonlinear cost does not by itself mean the linear solve
    // threw. The fifth native column is the actual linear-solve status.
    EXPECT_EQ(Optimizer::parseLinearFailuresForTesting(trace), 1U);
    EXPECT_EQ(Optimizer::parseLinearFailuresForTesting(
        "iter cost cost_change lambda success iter_time\n"
        "0 100 10 1e-5 1 0.01\n"
        "1 inf 0 1e-6 0 0.01\n"
        "1 90 10 1e-5 1 0.01\n"), 1U);
}

TEST(FGOGtsamPhase167RawNoDopplerTerminationTest,
     TraceInstrumentationPreservesTheDedicatedGraphResult) {
    auto baseline_config = makePhase164RawNoDopplerConfig();
    baseline_config.max_iterations = 1000;
    const auto problem = makePhase164RawNoDopplerProblem(true, 6);
    const auto baseline =
        FGOProcessor(baseline_config).optimizeProblem(problem);

    auto traced_config = baseline_config;
    traced_config.use_native_phase167_raw_p_no_doppler_lm_termination_budget =
        true;
    const auto traced = FGOProcessor(traced_config).optimizeProblem(problem);

    EXPECT_EQ(traced.diagnostics.pseudorange_factors,
              baseline.diagnostics.pseudorange_factors);
    EXPECT_EQ(traced.diagnostics.tdcp_factors,
              baseline.diagnostics.tdcp_factors);
    EXPECT_EQ(traced.diagnostics.motion_factors,
              baseline.diagnostics.motion_factors);
    EXPECT_EQ(traced.diagnostics.graph_factors,
              baseline.diagnostics.graph_factors);
    EXPECT_EQ(traced.diagnostics.graph_values,
              baseline.diagnostics.graph_values);
    EXPECT_NEAR(traced.diagnostics.initial_cost,
                baseline.diagnostics.initial_cost, 1.0e-8);
    EXPECT_NEAR(traced.diagnostics.final_cost,
                baseline.diagnostics.final_cost, 1.0e-8);
    ASSERT_EQ(traced.solution.size(), baseline.solution.size());
    for (std::size_t i = 0; i < traced.solution.size(); ++i) {
        EXPECT_LT((traced.solution.solutions[i].position_ecef -
                   baseline.solution.solutions[i].position_ecef)
                      .norm(),
                  1.0e-8);
    }
    EXPECT_TRUE(traced.diagnostics.native_phase143_termination.selector_enabled);
}

TEST(FGOGtsamPhase164RawNoDopplerGraphTest,
     RejectsInsufficientRowsGapAndNonAcceptedRawSeed) {
    const auto config = makePhase164RawNoDopplerConfig();

    const auto insufficient = makePhase164RawNoDopplerProblem(true, 3);
    const auto insufficient_result =
        FGOProcessor(config).optimizeProblem(insufficient);
    EXPECT_FALSE(insufficient_result.diagnostics.converged);
    EXPECT_TRUE(insufficient_result.solution.isEmpty());

    auto gap = makePhase164RawNoDopplerProblem(true, 6);
    gap.epochs[1].time = gap.epochs[0].time + 2.0;
    const auto gap_result = FGOProcessor(config).optimizeProblem(gap);
    EXPECT_FALSE(gap_result.diagnostics.converged);
    EXPECT_TRUE(gap_result.solution.isEmpty());

    auto rejected_seed = makePhase164RawNoDopplerProblem(true, 6);
    rejected_seed.native_raw_p_no_doppler_seeds[1].status =
        raw_p_seed::SeedAdapterStatus::RawPResultRejected;
    const auto rejected_result =
        FGOProcessor(config).optimizeProblem(rejected_seed);
    EXPECT_FALSE(rejected_result.diagnostics.converged);
    EXPECT_TRUE(rejected_result.solution.isEmpty());
}

TEST(FGOPhase194UtcFallbackImuNoiseTest,
     SelectsSourceWhiteNoiseOnlyAfterActualFallbackAndSquaresComponents) {
    FGOProcessor::ImuNoiseParams noise;
    noise.gravity_mps2 = 9.80665;
    noise.accel_bias_rw_sigma = 0.00025;
    noise.gyro_bias_rw_sigma = 0.0000005;
    noise.integration_sigma = 0.05;

    const auto applied = native_utc_fallback_imu_noise::apply(noise, true, true);
    EXPECT_TRUE(applied.requested);
    EXPECT_TRUE(applied.fallback_applied);
    EXPECT_TRUE(applied.applied);
    EXPECT_DOUBLE_EQ(applied.measurement_sync_coefficient, 1.0);
    EXPECT_STREQ(applied.source,
                 "source-utc-wall-clock-fallback-coefficient-1.0");
    EXPECT_DOUBLE_EQ(noise.accel_noise_sigma, 0.05);
    EXPECT_DOUBLE_EQ(noise.gyro_noise_sigma, 0.001);
    EXPECT_DOUBLE_EQ(noise.accel_bias_rw_sigma, 0.00025);
    EXPECT_DOUBLE_EQ(noise.gyro_bias_rw_sigma, 0.0000005);
    EXPECT_DOUBLE_EQ(noise.integration_sigma, 0.05);

    // The native backend squares only these two continuous-time white-noise
    // densities for the corresponding CombinedImuFactor covariance blocks.
    // Bias random walk and integration covariance remain independently set.
    auto params = gtsam::PreintegrationCombinedParams::MakeSharedU(
        noise.gravity_mps2);
    params->setAccelerometerCovariance(
        gtsam::I_3x3 * (noise.accel_noise_sigma * noise.accel_noise_sigma));
    params->setGyroscopeCovariance(
        gtsam::I_3x3 * (noise.gyro_noise_sigma * noise.gyro_noise_sigma));
    params->setIntegrationCovariance(
        gtsam::I_3x3 * (noise.integration_sigma * noise.integration_sigma));
    params->setBiasAccCovariance(
        gtsam::I_3x3 * (noise.accel_bias_rw_sigma * noise.accel_bias_rw_sigma));
    params->setBiasOmegaCovariance(
        gtsam::I_3x3 * (noise.gyro_bias_rw_sigma * noise.gyro_bias_rw_sigma));
    EXPECT_DOUBLE_EQ(params->getAccelerometerCovariance()(0, 0), 0.05 * 0.05);
    EXPECT_DOUBLE_EQ(params->getGyroscopeCovariance()(0, 0), 0.001 * 0.001);
    EXPECT_DOUBLE_EQ(params->getIntegrationCovariance()(0, 0), 0.05 * 0.05);
    EXPECT_DOUBLE_EQ(params->getBiasAccCovariance()(0, 0), 0.00025 * 0.00025);
    EXPECT_DOUBLE_EQ(params->getBiasOmegaCovariance()(0, 0), 0.0000005 * 0.0000005);

    FGOProcessor::ImuNoiseParams selector_off;
    const auto off = native_utc_fallback_imu_noise::apply(selector_off, false, true);
    EXPECT_FALSE(off.applied);
    EXPECT_DOUBLE_EQ(selector_off.accel_noise_sigma, 0.025);
    EXPECT_DOUBLE_EQ(selector_off.gyro_noise_sigma, 0.0005);

    FGOProcessor::ImuNoiseParams fallback_not_applied;
    const auto not_applied =
        native_utc_fallback_imu_noise::apply(fallback_not_applied, true, false);
    EXPECT_TRUE(not_applied.requested);
    EXPECT_FALSE(not_applied.applied);
    EXPECT_STREQ(not_applied.source,
                 "native-pixel5-coefficient-0.5-fallback-not-applied");
    EXPECT_DOUBLE_EQ(fallback_not_applied.accel_noise_sigma, 0.025);
    EXPECT_DOUBLE_EQ(fallback_not_applied.gyro_noise_sigma, 0.0005);
}

// Phase204 is a source-parity audit only.  The cached MATLAB graph uses a
// five-way ImuFactor at B2 and a separate bias BetweenFactor, while the native
// graph uses the installed GTSAM six-way CombinedImuFactor at (B1, B2).  Keep
// these checks in the test TU so the production graph and its defaults remain
// untouched until a real-data gate authorizes an ablation.
namespace phase204_imu_bias_audit {

using Bias = gtsam::imuBias::ConstantBias;

constexpr double kAccelWhiteSigma = 0.10;
constexpr double kGyroWhiteSigma = 0.01;
constexpr double kIntegrationSigma = 0.02;
constexpr double kAccelBiasRwSigma = 0.002;
constexpr double kGyroBiasRwSigma = 0.0003;

std::shared_ptr<gtsam::PreintegrationParams> sourceParams() {
    auto params = gtsam::PreintegrationParams::MakeSharedU(9.80665);
    params->setAccelerometerCovariance(
        gtsam::I_3x3 * (kAccelWhiteSigma * kAccelWhiteSigma));
    params->setGyroscopeCovariance(
        gtsam::I_3x3 * (kGyroWhiteSigma * kGyroWhiteSigma));
    params->setIntegrationCovariance(
        gtsam::I_3x3 * (kIntegrationSigma * kIntegrationSigma));
    params->setOmegaCoriolis(gtsam::Vector3::Zero());
    return params;
}

std::shared_ptr<gtsam::PreintegrationCombinedParams> nativeParams() {
    auto params = gtsam::PreintegrationCombinedParams::MakeSharedU(9.80665);
    params->setAccelerometerCovariance(
        gtsam::I_3x3 * (kAccelWhiteSigma * kAccelWhiteSigma));
    params->setGyroscopeCovariance(
        gtsam::I_3x3 * (kGyroWhiteSigma * kGyroWhiteSigma));
    params->setIntegrationCovariance(
        gtsam::I_3x3 * (kIntegrationSigma * kIntegrationSigma));
    params->setBiasAccCovariance(
        gtsam::I_3x3 * (kAccelBiasRwSigma * kAccelBiasRwSigma));
    params->setBiasOmegaCovariance(
        gtsam::I_3x3 * (kGyroBiasRwSigma * kGyroBiasRwSigma));
    params->setOmegaCoriolis(gtsam::Vector3::Zero());
    return params;
}

}  // namespace phase204_imu_bias_audit

TEST(FGOGtsamPhase208SourceImuFactorTest,
     EndingBiasPredictionAndNumericalJacobianWithSeparateBiasEvolution) {
    using namespace phase204_imu_bias_audit;
    gtsam::PreintegratedImuMeasurements pim(sourceParams(), Bias());
    constexpr std::size_t sample_count = 20;
    for (std::size_t i = 0; i < sample_count; ++i) {
        const double t = static_cast<double>(i) * 0.05;
        pim.integrateMeasurement(gtsam::Vector3(0.2 + t, -0.1, 9.8),
                                 gtsam::Vector3(0.01, 0.02 + t * 0.01, -0.01),
                                 0.05);
    }
    const Bias bi(gtsam::Vector3(0.01, -0.02, 0.015),
                  gtsam::Vector3(0.003, -0.002, 0.001));
    const Bias bj(gtsam::Vector3(0.045, -0.005, 0.025),
                  gtsam::Vector3(0.009, -0.001, 0.004));
    const gtsam::NavState start(gtsam::Pose3::Identity(),
                                gtsam::Vector3(1.0, -0.2, 0.1));
    const auto end = pim.predict(start, bj);
    // Keys deliberately distinct: the motion factor must depend on Bj only.
    const gtsam::ImuFactor motion(400, 401, 402, 403, 405, pim);
    gtsam::Matrix h1, h2, h3, h4, hb;
    const auto residual = motion.evaluateError(
        start.pose(), start.velocity(), end.pose(), end.velocity(), bj,
        &h1, &h2, &h3, &h4, &hb);
    EXPECT_LT(residual.norm(), 1e-9);
    EXPECT_GT(motion.evaluateError(start.pose(), start.velocity(), end.pose(),
                                   end.velocity(), bi).norm(), 1e-4);
    gtsam::Matrix numerical(9, 6);
    constexpr double epsilon = 1e-6;
    for (int axis = 0; axis < 6; ++axis) {
        gtsam::Vector6 plus = bj.vector(), minus = bj.vector();
        plus(axis) += epsilon;
        minus(axis) -= epsilon;
        const Bias bp(plus.head<3>(), plus.tail<3>());
        const Bias bm(minus.head<3>(), minus.tail<3>());
        numerical.col(axis) = (motion.evaluateError(
            start.pose(), start.velocity(), end.pose(), end.velocity(), bp) -
            motion.evaluateError(start.pose(), start.velocity(), end.pose(),
                                 end.velocity(), bm)) / (2.0 * epsilon);
    }
    EXPECT_LT((hb - numerical).norm(), 1e-7);

    gtsam::Vector6 sigmas;
    sigmas << kAccelBiasRwSigma, kAccelBiasRwSigma, kAccelBiasRwSigma,
              kGyroBiasRwSigma, kGyroBiasRwSigma, kGyroBiasRwSigma;
    sigmas *= std::sqrt(static_cast<double>(sample_count));
    const auto noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
    const gtsam::BetweenFactor<Bias> evolution(404, 405, Bias(), noise);
    gtsam::Values values;
    values.insert(400, start.pose());
    values.insert(401, start.velocity());
    values.insert(402, end.pose());
    values.insert(403, end.velocity());
    values.insert(404, bi);
    values.insert(405, bj);
    gtsam::NonlinearFactorGraph graph;
    graph.add(motion);
    graph.add(evolution);
    const gtsam::Vector6 difference = bj.vector() - bi.vector();
    const double expected = 0.5 * difference.cwiseQuotient(sigmas).squaredNorm();
    EXPECT_NEAR(graph.error(values), expected, 1e-8);
    EXPECT_EQ(graph.size(), 2U);
    EXPECT_EQ(motion.keys().back(), 405U);
    EXPECT_EQ(evolution.keys().front(), 404U);
    EXPECT_EQ(evolution.keys().back(), 405U);
}

TEST(FGOGtsamPhase213MainDopplerFrameTest,
     CorrectedEcefRowPreservesLikelihoodAfterOneEnuRotation) {
    using libgnss::fgo_gtsam_internal::UndifferencedDopplerVelocityFactorSourceClock;
    using libgnss::fgo_gtsam_internal::UndifferencedDopplerVelocityFactorSourceClockEcef;
    const Vector3d los_ecef = Vector3d(0.3, -0.7, 0.6).normalized();
    const Vector3d velocity_ecef(2.0, -1.3, 0.8);
    const double latitude = 0.64, longitude = -2.13;
    const gtsam::Vector3 los_nav = ecef2enu(los_ecef, latitude, longitude);
    const gtsam::Vector3 velocity_nav = ecef2enu(velocity_ecef, latitude, longitude);
    const gtsam::Vector drift = gtsam::Vector::Constant(1, 0.37);
    // A controlled nonzero residual verifies sign, units and whitening.
    const double measured = los_ecef.dot(velocity_ecef) + drift(0) - 0.25;
    const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 0.5);
    const UndifferencedDopplerVelocityFactorSourceClockEcef stage(
        500, 501, los_ecef, measured, noise);
    const UndifferencedDopplerVelocityFactorSourceClock main(
        500, 501, los_nav, measured, noise);
    gtsam::Matrix hv, hd;
    const auto error = main.evaluateError(velocity_nav, drift, &hv, &hd);
    EXPECT_NEAR(error(0), 0.25, 1e-12);
    EXPECT_NEAR(stage.evaluateError(velocity_ecef, drift)(0), error(0), 1e-12);
    constexpr double epsilon = 1e-6;
    for (int axis = 0; axis < 3; ++axis) {
        gtsam::Vector3 plus = velocity_nav, minus = velocity_nav;
        plus(axis) += epsilon; minus(axis) -= epsilon;
        const double derivative = (main.evaluateError(plus, drift)(0) -
            main.evaluateError(minus, drift)(0)) / (2.0 * epsilon);
        EXPECT_NEAR(hv(0, axis), derivative, 1e-9);
    }
    EXPECT_DOUBLE_EQ(hd(0, 0), 1.0);
    gtsam::Values values;
    values.insert(500, velocity_nav); values.insert(501, drift);
    EXPECT_NEAR(main.error(values), 0.125, 1e-12);
    const UndifferencedDopplerVelocityFactorSourceClock unrotated(
        500, 501, los_ecef, measured, noise);
    EXPECT_GT(std::abs(unrotated.evaluateError(velocity_nav, drift)(0) - error(0)), 0.1);
    EXPECT_FALSE(main.evaluateError(velocity_nav, gtsam::Vector::Zero(2)).allFinite());
}

TEST(FGOGtsamPhase171RefinementBuilderTest, RejectsMissingVelocityAndSppReplacement) {
    std::vector<ObservationData> raw{ObservationData(GNSSTime(2300,100)),
                                     ObservationData(GNSSTime(2300,101))};
    std::vector<Vector3d> velocities(2,Vector3d::Zero());
    NavigationData nav;
    FGOProcessor::FGOConfig config;
    config.use_spp_seed=true;
    config.use_upstream_observable_quality=true;
    EXPECT_THROW(FGOProcessor(config).buildPseudorangeProblem(raw,nav,velocities),std::invalid_argument);
    config.use_spp_seed=false;
    config.use_native_phase171_raw_p_no_doppler_imu_main=true;
    config.use_native_source_clock_c0d_factor=true;
    config.use_corrected_undifferenced_doppler_factors=true;
    velocities.pop_back();
    EXPECT_THROW(FGOProcessor(config).buildPseudorangeProblem(raw,nav,velocities),std::invalid_argument);
    velocities.push_back(Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()));
    EXPECT_THROW(FGOProcessor(config).buildPseudorangeProblem(raw,nav,velocities),std::invalid_argument);
    velocities[1].setZero(); config.use_upstream_observable_quality=false;
    EXPECT_THROW(FGOProcessor(config).buildPseudorangeProblem(raw,nav,velocities),std::invalid_argument);
}

TEST(FGOGtsamPhase216Pose3MotionTest, MatchesPointResidualAndPoseTangentJacobians) {
    using namespace libgnss::fgo_gtsam_internal;
    const auto noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.05);
    const double dt = 0.8;
    const gtsam::Vector3 v1(2.0, -0.4, 0.1), v2(2.4, -0.2, 0.3);
    const gtsam::Point3 x1(1.0, 2.0, -0.5);
    const gtsam::Point3 x2 = x1 + 0.5 * dt * (v1 + v2);
    const gtsam::Pose3 p1(gtsam::Rot3::RzRyRx(0.3, -0.2, 0.5), x1);
    const gtsam::Pose3 p2(gtsam::Rot3::RzRyRx(-0.4, 0.1, -0.6), x2);
    MotionFactorPose3XXVV factor(600, 601, 602, 603, dt, noise);
    MotionFactorXXVV reference(600, 601, 602, 603, dt, noise);
    gtsam::Matrix h1, h2, h3, h4;
    const auto residual = factor.evaluateError(p1, p2, v1, v2, &h1, &h2, &h3, &h4);
    EXPECT_LT(residual.norm(), 1e-12);
    EXPECT_LT((residual - reference.evaluateError(x1, x2, v1, v2)).norm(), 1e-12);
    constexpr double eps = 1e-6;
    for (int k = 0; k < 6; ++k) {
        gtsam::Vector6 delta = gtsam::Vector6::Zero(); delta(k) = eps;
        const gtsam::Vector n1 = (factor.evaluateError(p1.retract(delta), p2, v1, v2) -
            factor.evaluateError(p1.retract(-delta), p2, v1, v2)) / (2 * eps);
        const gtsam::Vector n2 = (factor.evaluateError(p1, p2.retract(delta), v1, v2) -
            factor.evaluateError(p1, p2.retract(-delta), v1, v2)) / (2 * eps);
        EXPECT_LT((h1.col(k) - n1).norm(), 1e-8);
        EXPECT_LT((h2.col(k) - n2).norm(), 1e-8);
    }
    for (int k = 0; k < 3; ++k) {
        gtsam::Vector3 delta = gtsam::Vector3::Zero(); delta(k) = eps;
        EXPECT_LT((h3.col(k) - (factor.evaluateError(p1, p2, v1 + delta, v2) -
            factor.evaluateError(p1, p2, v1 - delta, v2)) / (2 * eps)).norm(), 1e-8);
        EXPECT_LT((h4.col(k) - (factor.evaluateError(p1, p2, v1, v2 + delta) -
            factor.evaluateError(p1, p2, v1, v2 - delta)) / (2 * eps)).norm(), 1e-8);
    }
    const gtsam::Vector3 inconsistent = v2 + gtsam::Vector3(1, 0, 0);
    EXPECT_NEAR(factor.evaluateError(p1, p2, v1, inconsistent)(0), -0.4, 1e-12);
    gtsam::Values values;
    values.insert(600, p1); values.insert(601, p2);
    values.insert(602, v1); values.insert(603, inconsistent);
    EXPECT_NEAR(factor.error(values), 32.0, 1e-9);
    EXPECT_THROW(MotionFactorPose3XXVV(600, 601, 602, 603, 0.0, noise), std::invalid_argument);
    EXPECT_THROW(MotionFactorPose3XXVV(600, 601, 602, 603,
        std::numeric_limits<double>::quiet_NaN(), noise), std::invalid_argument);
}

TEST(FGOGtsamPhase204ImuBiasAuditTest,
     SourceBetweenNoiseUsesSampleCountButCombinedRandomWalkUsesDuration) {
    using namespace phase204_imu_bias_audit;

    // The cached graph creates one bias BetweenFactor per GNSS interval with
    // sigma=sqrt(N)*sigma_between_b.  The installed GTSAM CombinedImuFactor
    // propagates its bias random walk as dt*biasCovariance per integration
    // sample.  Equal one-second intervals with 53 IMU samples make the unit
    // difference observable without depending on H data or a solver result.
    constexpr std::size_t kSamples = 53;
    constexpr double kIntervalSeconds = 1.0 / static_cast<double>(kSamples);

    auto params = nativeParams();
    params->setAccelerometerCovariance(gtsam::Matrix3::Zero());
    params->setGyroscopeCovariance(gtsam::Matrix3::Zero());
    params->setIntegrationCovariance(gtsam::Matrix3::Zero());
    gtsam::PreintegratedCombinedMeasurements native_pim(
        params, Bias());
    for (std::size_t i = 0; i < kSamples; ++i) {
        native_pim.integrateMeasurement(gtsam::Vector3::Zero(),
                                        gtsam::Vector3::Zero(),
                                        kIntervalSeconds);
    }

    gtsam::Vector6 source_sigmas;
    source_sigmas.head<3>().setConstant(
        std::sqrt(static_cast<double>(kSamples)) * kAccelBiasRwSigma);
    source_sigmas.tail<3>().setConstant(
        std::sqrt(static_cast<double>(kSamples)) * kGyroBiasRwSigma);
    const gtsam::BetweenFactor<Bias> source_between(
        100, 101, Bias(),
        gtsam::noiseModel::Diagonal::Sigmas(source_sigmas));

    const auto source_noise =
        std::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(
            source_between.noiseModel());
    ASSERT_NE(source_noise, nullptr);
    const gtsam::Matrix source_covariance = source_noise->covariance();
    const gtsam::Matrix native_covariance = native_pim.preintMeasCov();
    const double native_duration = native_pim.deltaTij();

    EXPECT_NEAR(native_duration, 1.0, 1.0e-12);
    EXPECT_NEAR(source_covariance(0, 0),
                static_cast<double>(kSamples) * kAccelBiasRwSigma *
                    kAccelBiasRwSigma,
                1.0e-14);
    EXPECT_NEAR(source_covariance(3, 3),
                static_cast<double>(kSamples) * kGyroBiasRwSigma *
                    kGyroBiasRwSigma,
                1.0e-18);
    EXPECT_NEAR(native_covariance(9, 9),
                native_duration * kAccelBiasRwSigma * kAccelBiasRwSigma,
                1.0e-14);
    EXPECT_NEAR(native_covariance(12, 12),
                native_duration * kGyroBiasRwSigma * kGyroBiasRwSigma,
                1.0e-18);
    EXPECT_NEAR(source_covariance(0, 0) / native_covariance(9, 9),
                static_cast<double>(kSamples), 1.0e-9);
    EXPECT_NEAR(source_covariance(3, 3) / native_covariance(12, 12),
                static_cast<double>(kSamples), 1.0e-9);
}

TEST(FGOGtsamPhase204ImuBiasAuditTest,
     FiveWayAndSixWayShareStaticBiasResidualsAndStateJacobians) {
    using namespace phase204_imu_bias_audit;

    auto source_pim_params = sourceParams();
    auto native_pim_params = nativeParams();
    gtsam::PreintegratedImuMeasurements source_pim(source_pim_params, Bias());
    gtsam::PreintegratedCombinedMeasurements native_pim(native_pim_params,
                                                         Bias());
    const std::array<double, 3> dts = {0.07, 0.11, 0.09};
    const std::array<gtsam::Vector3, 3> accelerations = {
        gtsam::Vector3(0.2, -0.1, 9.7),
        gtsam::Vector3(-0.3, 0.15, 9.8),
        gtsam::Vector3(0.1, 0.05, 9.75)};
    const std::array<gtsam::Vector3, 3> angular_rates = {
        gtsam::Vector3(0.03, -0.02, 0.01),
        gtsam::Vector3(-0.01, 0.025, -0.015),
        gtsam::Vector3(0.02, 0.01, 0.04)};
    for (std::size_t i = 0; i < dts.size(); ++i) {
        source_pim.integrateMeasurement(accelerations[i], angular_rates[i],
                                        dts[i]);
        native_pim.integrateMeasurement(accelerations[i], angular_rates[i],
                                        dts[i]);
    }

    const gtsam::Pose3 pose_i(
        gtsam::Rot3::Expmap(gtsam::Vector3(0.1, -0.04, 0.08)),
        gtsam::Point3(1.0, -2.0, 0.5));
    const gtsam::Pose3 pose_j(
        gtsam::Rot3::Expmap(gtsam::Vector3(0.14, -0.02, 0.12)),
        gtsam::Point3(1.3, -1.7, 0.7));
    const gtsam::Vector3 velocity_i(2.0, -0.5, 0.3);
    const gtsam::Vector3 velocity_j(2.1, -0.4, 0.25);
    const gtsam::Key pose_i_key = 200;
    const gtsam::Key velocity_i_key = 201;
    const gtsam::Key pose_j_key = 202;
    const gtsam::Key velocity_j_key = 203;
    const gtsam::Key bias_i_key = 204;
    const gtsam::Key bias_j_key = 205;

    // The source code explicitly passes B2 to ImuFactor.  The native factor
    // has both bias keys and its first nine residuals use Bi.
    const gtsam::ImuFactor source_factor(
        pose_i_key, velocity_i_key, pose_j_key, velocity_j_key, bias_j_key,
        source_pim);
    const gtsam::CombinedImuFactor native_factor(
        pose_i_key, velocity_i_key, pose_j_key, velocity_j_key, bias_i_key,
        bias_j_key, native_pim);
    ASSERT_EQ(source_factor.keys().size(), 5U);
    ASSERT_EQ(native_factor.keys().size(), 6U);
    EXPECT_EQ(source_factor.keys().at(4), bias_j_key);
    EXPECT_EQ(native_factor.keys().at(4), bias_i_key);
    EXPECT_EQ(native_factor.keys().at(5), bias_j_key);

    const auto compare_equal_bias = [&](const Bias& bias) {
        gtsam::Matrix source_h1, source_h2, source_h3, source_h4, source_h5;
        gtsam::Matrix native_h1, native_h2, native_h3, native_h4, native_h5,
            native_h6;
        const gtsam::Vector source_error = source_factor.evaluateError(
            pose_i, velocity_i, pose_j, velocity_j, bias, &source_h1,
            &source_h2, &source_h3, &source_h4, &source_h5);
        const gtsam::Vector native_error = native_factor.evaluateError(
            pose_i, velocity_i, pose_j, velocity_j, bias, bias, &native_h1,
            &native_h2, &native_h3, &native_h4, &native_h5, &native_h6);

        EXPECT_LT((source_error - native_error.head(9)).norm(), 1.0e-9);
        EXPECT_LT(native_error.tail(6).norm(), 1.0e-12);
        EXPECT_LT((source_h1 - native_h1.topRows(9)).norm(), 1.0e-8);
        EXPECT_LT((source_h2 - native_h2.topRows(9)).norm(), 1.0e-8);
        EXPECT_LT((source_h3 - native_h3.topRows(9)).norm(), 1.0e-8);
        EXPECT_LT((source_h4 - native_h4.topRows(9)).norm(), 1.0e-8);
        EXPECT_LT((source_h5 - native_h5.topRows(9)).norm(), 1.0e-8);
        EXPECT_LT(native_h6.topRows(9).norm(), 1.0e-12);
        // CombinedImuFactor calls Between(Bj, Bi), so its tail is Bi-Bj
        // (the reverse orientation of BetweenFactor(Bi, Bj)).
        EXPECT_LT((native_h5.bottomRows(6) - gtsam::I_6x6).norm(), 1.0e-12);
        EXPECT_LT((native_h6.bottomRows(6) + gtsam::I_6x6).norm(), 1.0e-12);
    };

    compare_equal_bias(Bias());
    compare_equal_bias(Bias(gtsam::Vector3(0.02, -0.01, 0.015),
                            gtsam::Vector3(0.003, -0.002, 0.001)));
}

TEST(FGOGtsamPhase204ImuBiasAuditTest,
     ChangingBiasShowsB2SourceCorrectionVersusNativeBiasEvolution) {
    using namespace phase204_imu_bias_audit;

    auto source_pim_params = sourceParams();
    auto native_pim_params = nativeParams();
    gtsam::PreintegratedImuMeasurements source_pim(source_pim_params, Bias());
    gtsam::PreintegratedCombinedMeasurements native_pim(native_pim_params,
                                                         Bias());
    source_pim.integrateMeasurement(gtsam::Vector3(0.4, -0.2, 9.7),
                                    gtsam::Vector3(0.04, -0.03, 0.02), 0.35);
    native_pim.integrateMeasurement(gtsam::Vector3(0.4, -0.2, 9.7),
                                    gtsam::Vector3(0.04, -0.03, 0.02), 0.35);

    const gtsam::Pose3 pose_i = gtsam::Pose3::Identity();
    const gtsam::Pose3 pose_j(gtsam::Rot3::Expmap(
                                  gtsam::Vector3(0.03, -0.02, 0.05)),
                              gtsam::Point3(0.8, -0.2, 0.1));
    const gtsam::Vector3 velocity_i(1.0, -0.4, 0.2);
    const gtsam::Vector3 velocity_j(1.2, -0.1, 0.3);
    const gtsam::Key pose_i_key = 300;
    const gtsam::Key velocity_i_key = 301;
    const gtsam::Key pose_j_key = 302;
    const gtsam::Key velocity_j_key = 303;
    const gtsam::Key bias_i_key = 304;
    const gtsam::Key bias_j_key = 305;
    const gtsam::ImuFactor source_factor(
        pose_i_key, velocity_i_key, pose_j_key, velocity_j_key, bias_j_key,
        source_pim);
    const gtsam::CombinedImuFactor native_factor(
        pose_i_key, velocity_i_key, pose_j_key, velocity_j_key, bias_i_key,
        bias_j_key, native_pim);
    const gtsam::BetweenFactor<Bias> source_between(
        bias_i_key, bias_j_key, Bias(),
        gtsam::noiseModel::Isotropic::Sigma(6, 1.0));

    const Bias bias_i(gtsam::Vector3(0.01, -0.02, 0.015),
                     gtsam::Vector3(0.003, -0.002, 0.001));
    const Bias bias_j(gtsam::Vector3(0.045, -0.005, 0.025),
                     gtsam::Vector3(0.009, -0.001, 0.004));
    const gtsam::Vector source_imu_error = source_factor.evaluateError(
        pose_i, velocity_i, pose_j, velocity_j, bias_j);
    const gtsam::Vector native_error = native_factor.evaluateError(
        pose_i, velocity_i, pose_j, velocity_j, bias_i, bias_j);
    const gtsam::Vector source_between_error =
        source_between.evaluateError(bias_i, bias_j);
    gtsam::Vector source_architecture_error(15);
    source_architecture_error << source_imu_error, source_between_error;

    // Both architectures encode the same bias-evolution residual, but only
    // the native CombinedImuFactor uses Bi in its first nine preintegration
    // residuals.  Passing Bj to the source ImuFactor is therefore observably
    // different when the bias changes over the interval.
    // GTSAM's CombinedImuFactor evaluates Between(Bj, Bi), while the cached
    // graph's separate BetweenFactor is keyed (Bi, Bj); the residuals have
    // equal magnitude but opposite orientation.
    EXPECT_LT((native_error.tail(6) + source_between_error).norm(), 1.0e-12);
    EXPECT_GT((native_error.head(9) - source_imu_error).norm(), 1.0e-8);
    EXPECT_GT((native_error - source_architecture_error).norm(), 1.0e-8);

    gtsam::Matrix native_h1, native_h2, native_h3, native_h4, native_h5,
        native_h6;
    (void)native_factor.evaluateError(
        pose_i, velocity_i, pose_j, velocity_j, bias_i, bias_j, &native_h1,
        &native_h2, &native_h3, &native_h4, &native_h5, &native_h6);
    EXPECT_LT((native_h5.bottomRows(6) - gtsam::I_6x6).norm(), 1.0e-12);
    EXPECT_LT((native_h6.bottomRows(6) + gtsam::I_6x6).norm(), 1.0e-12);
}

TEST(FGOGtsamPhase204ImuBiasAuditTest,
     CombinedCovarianceRetainsMotionBiasCorrelationAfterSignNormalization) {
    using namespace phase204_imu_bias_audit;
    gtsam::PreintegratedImuMeasurements source(sourceParams(), Bias());
    gtsam::PreintegratedCombinedMeasurements combined(nativeParams(), Bias());
    for (int i = 0; i < 20; ++i) {
        const gtsam::Vector3 acc(0.2, -0.1, 9.8);
        const gtsam::Vector3 gyro(0.01, 0.02, -0.01);
        source.integrateMeasurement(acc, gyro, 0.05);
        combined.integrateMeasurement(acc, gyro, 0.05);
    }
    const gtsam::Matrix covariance = combined.preintMeasCov();
    ASSERT_EQ(covariance.rows(), 15);
    EXPECT_GT(covariance.block(0, 9, 9, 6).norm(), 1e-12);
    EXPECT_GT((covariance.topLeftCorner(9, 9) - source.preintMeasCov()).norm(),
              1e-12);
    // Reversing the bias residual also reverses its motion cross covariance.
    // It does not make the joint distribution block diagonal.
    gtsam::Matrix sign = gtsam::Matrix::Identity(15, 15);
    sign.bottomRightCorner(6, 6) *= -1.0;
    const gtsam::Matrix normalized = sign * covariance * sign.transpose();
    EXPECT_LT((normalized.block(0, 9, 9, 6) +
               covariance.block(0, 9, 9, 6)).norm(), 1e-15);
    EXPECT_LT((normalized.bottomRightCorner(6, 6) -
               covariance.bottomRightCorner(6, 6)).norm(), 1e-15);
    EXPECT_GT(normalized.block(0, 9, 9, 6).norm(), 1e-12);
}

TEST(FGOGtsamPhase205ImuBiasDensityTest, MatchesCountMarginalAcrossRatesAndDurations) {
    using namespace phase204_imu_bias_audit;
    for (const std::size_t count : {10U, 53U, 100U}) {
        for (const double duration : {0.5, 1.0, 2.0}) {
            const double scale = libgnss::native_imu_bias_density::sourceCountScale(
                count, duration);
            const auto baseline = nativeParams();
            auto params = std::make_shared<gtsam::PreintegrationCombinedParams>(*baseline);
            params->setBiasAccCovariance(baseline->getBiasAccCovariance() * scale);
            params->setBiasOmegaCovariance(baseline->getBiasOmegaCovariance() * scale);
            gtsam::PreintegratedCombinedMeasurements pim(params, Bias());
            for (std::size_t i = 0; i < count; ++i) {
                pim.integrateMeasurement(gtsam::Vector3(0, 0, 9.80665),
                                         gtsam::Vector3::Zero(), duration / count);
            }
            EXPECT_NEAR(pim.preintMeasCov()(9, 9),
                        count * kAccelBiasRwSigma * kAccelBiasRwSigma, 1e-14);
            EXPECT_NEAR(pim.preintMeasCov()(12, 12),
                        count * kGyroBiasRwSigma * kGyroBiasRwSigma, 1e-16);
            EXPECT_DOUBLE_EQ(baseline->getBiasAccCovariance()(0, 0),
                             kAccelBiasRwSigma * kAccelBiasRwSigma);
        }
    }
}

TEST(FGOGtsamPhase205ImuBiasDensityTest, RejectsInvalidCountsAndDurations) {
    using libgnss::native_imu_bias_density::sourceCountScale;
    EXPECT_THROW(sourceCountScale(0, 1.0), std::invalid_argument);
    for (const double duration : {0.0, -1.0,
            std::numeric_limits<double>::infinity(),
            std::numeric_limits<double>::quiet_NaN(),
            std::numeric_limits<double>::denorm_min()}) {
        EXPECT_THROW(sourceCountScale(53, duration), std::invalid_argument);
    }
}

#endif  // GNSSPP_HAS_GTSAM
