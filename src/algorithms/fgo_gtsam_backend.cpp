// GTSAM backend for FGOProcessor::optimizeProblem (Phase 1, RTK/DD-focused).
//
// Consumes the exact same FGOProcessor::FGOProblem the native Eigen backend
// (fgo.cpp) consumes and produces an equivalent FGOProcessor::FGOResult.
// Everything in this file is compiled only when GNSSPP_HAS_GTSAM is defined
// (see CMakeLists.txt: only added to gnss_lib_noopt when find_package(GTSAM)
// succeeds), so no other translation unit needs to guard against a missing
// GTSAM install.
//
// --- Observation-convention note (see docs/gtsam_backend_design.md risk #2) ---
// gtsam::gnss::DoubleDifferenceData::observed() computes
//     (rovRef - baseRef) - (rovTarget - baseTarget)
// while libgnss's FGOProcessor::DoubleDifferencePseudorangeFactor stores
//     observed_dd_pseudorange_m = (satellite_* - base_satellite_*)
//                                - (reference_* - base_reference_*)
// i.e. (target - reference), the mirror image of GTSAM's (ref - target).
// Feeding libgnss's "satellite" (target) into GTSAM's "Ref" slot and
// libgnss's "reference_satellite" into GTSAM's "Target" slot makes the two
// conventions coincide exactly (verified by the runtime assertion below,
// and by the DD carrier equivalent). The same swap applies to the ambiguity
// keys of DoubleDifferenceCarrierPhaseFactor: GTSAM forms
// lam*(ambRef - ambTarget); libgnss tracks a single lumped DD ambiguity
// equal to lam*(N_target - N_reference). So ambRef <- the real ambiguity
// node (seeded with libgnss's lumped value, in cycles) and ambTarget <- a
// single shared dummy node pinned to 0 with a tight prior.

#include <libgnss++/algorithms/disjoint_constellation_partition.hpp>
#include "fgo_gtsam_internal.hpp"

namespace libgnss {

using namespace fgo_gtsam_internal;

FGOProcessor::FGOResult optimizeProblemWithGtsam(
    const FGOProcessor::FGOProblem& problem,
    const FGOProcessor::FGOConfig& config,
    FGOProcessor::FGOResult result) {
    // Milestone 2c: dispatch to the incremental fixed-lag smoother when
    // requested and the IMU-coupled Pose3 path is fully specified. Otherwise
    // fall through to the batch LM path below (Phase-1 / 2a / 2b), unchanged.
    if (config.use_fixed_lag_smoother && config.use_pose3_state && config.use_imu &&
        problem.imu.valid && problem.epochs.size() >= 2) {
        return optimizeProblemFixedLag(problem, config, std::move(result));
    }
    const auto start_time = std::chrono::high_resolution_clock::now();

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;

    // --- Milestone 2a/2b: Pose3 + lever-arm state (docs/gtsam_backend_design.md) ---
    // use_pose3 selects whether positionKey(epoch) holds a gtsam::Pose3 (body
    // pose, antenna offset by pose3_lever_arm_body_m) or the Phase-1
    // gtsam::Point3 (bare antenna position).
    //
    // use_imu (2b) additionally: interprets the Pose3 as body-in-nav (local
    // ENU) via a real ecef_T_nav, adds Vector3 velocity + imuBias states per
    // epoch, and links consecutive epochs with CombinedImuFactors. In 2a
    // (use_pose3 without IMU) the pose is expressed directly in ECEF (no
    // ecef_T_nav) and only the rotation gauge-pin constrains attitude.
    const std::size_t num_epochs = problem.epochs.size();
    const bool use_pose3 = config.use_pose3_state;
    const bool use_imu =
        use_pose3 && config.use_imu && problem.imu.valid && num_epochs >= 2;
    const Point3 lever_arm_body(config.pose3_lever_arm_body_m.x(),
                                config.pose3_lever_arm_body_m.y(),
                                config.pose3_lever_arm_body_m.z());

    // ecef_T_nav: identity/unused in 2a (pose in ECEF); the ENU-from-ECEF
    // transform at the nav origin in 2b (pose in nav). The DD '...FactorArm'
    // factors and the IMU factor share this same Pose3 through it.
    gtsam::gnss::LeverArm gnss_lever_arm(lever_arm_body);
    Pose3 ecef_T_nav;  // identity unless use_imu
    if (use_imu) {
        const Rot3 R_ecef_nav = ecefFromEnuRotation(problem.imu.nav_origin_lat_rad,
                                                    problem.imu.nav_origin_lon_rad);
        ecef_T_nav = Pose3(R_ecef_nav, Point3(problem.imu.nav_origin_ecef));
        gnss_lever_arm = gtsam::gnss::LeverArm(lever_arm_body, ecef_T_nav);
    }

    // Initial body->nav (ENU) attitude from Stage-1 alignment; used as the
    // dead-reckoning start point for the 2b pose attitude seeds below.
    const Rot3 init_attitude_nav(problem.imu.init_attitude_body_to_nav);

    // Antenna ECEF position for the optimized state at `epoch`, uniform across
    // all three representations (Point3, 2a Pose3-in-ECEF, 2b Pose3-in-nav):
    // gnss::LeverArm::antennaPosition applies ecef_T_nav internally when set.
    // Used everywhere downstream that needs "the position": LAMBDA covariance
    // propagation, residual RMS diagnostics, final per-epoch solution mapping.
    auto antennaPositionOf = [&](const gtsam::Values& values,
                                 std::size_t epoch) -> Point3 {
        if (use_pose3) {
            return gnss_lever_arm.antennaPosition(values.at<Pose3>(positionKey(epoch)));
        }
        return values.at<Point3>(positionKey(epoch));
    };

    // Pose seeds (Point3 and 2a Pose3-in-ECEF; the 2b IMU poses are seeded
    // below, after preintegration, so their attitudes can be dead-reckoned).
    for (std::size_t i = 0; i < num_epochs; ++i) {
        if (use_imu) {
            continue;
        }
        if (use_pose3) {
            // 2a: pose in ECEF, identity attitude (unobservable without IMU);
            // translation = antenna seed minus lever arm.
            initial.insert(positionKey(i),
                           Pose3(Rot3(), Point3(problem.epochs[i].position_ecef) - lever_arm_body));
        } else {
            initial.insert(positionKey(i), Point3(problem.epochs[i].position_ecef));
        }
    }

    if (use_imu) {
        const gtsam::imuBias::ConstantBias init_bias(problem.imu.init_accel_bias,
                                                     problem.imu.init_gyro_bias);

        // --- IMU preintegration params (ENU / Z-up; gravity along -Z) ---
        auto imu_params = gtsam::PreintegrationCombinedParams::MakeSharedU(
            problem.imu.noise.gravity_mps2);
        const auto sq = [](double s) { return s * s; };
        imu_params->setAccelerometerCovariance(gtsam::I_3x3 * sq(problem.imu.noise.accel_noise_sigma));
        imu_params->setGyroscopeCovariance(gtsam::I_3x3 * sq(problem.imu.noise.gyro_noise_sigma));
        imu_params->setIntegrationCovariance(gtsam::I_3x3 * sq(problem.imu.noise.integration_sigma));
        imu_params->setBiasAccCovariance(gtsam::I_3x3 * sq(problem.imu.noise.accel_bias_rw_sigma));
        imu_params->setBiasOmegaCovariance(gtsam::I_3x3 * sq(problem.imu.noise.gyro_bias_rw_sigma));

        // --- Preintegrate each [epoch i, epoch i+1) interval up front ---
        // We need the preintegrated rotations to dead-reckon attitude seeds
        // BEFORE inserting the pose values: seeding every pose at the same
        // initial attitude makes each IMU factor's rotation residual the full
        // accumulated turn between epochs, which gives a huge initial cost and
        // a divergent first LM step (observed: 1e19 cost, no convergence). By
        // forward-propagating attitude through deltaRij() the rotation residual
        // starts ~0 and the DD-accurate translation keeps position anchored, so
        // the graph is well conditioned from iteration 0.
        const auto& imu_samples = problem.imu.samples_body_flu;
        std::vector<gtsam::PreintegratedCombinedMeasurements> pims;
        pims.reserve(num_epochs > 0 ? num_epochs - 1 : 0);
        std::vector<bool> pim_valid(num_epochs > 0 ? num_epochs - 1 : 0, false);
        std::size_t sample_cursor = 0;
        std::size_t imu_intervals = 0;
        for (std::size_t i = 0; i + 1 < num_epochs; ++i) {
            const GNSSTime t0 = problem.epochs[i].time;
            const GNSSTime t1 = problem.epochs[i + 1].time;
            while (sample_cursor < imu_samples.size() && imu_samples[sample_cursor].time < t0) {
                ++sample_cursor;
            }
            gtsam::PreintegratedCombinedMeasurements pim(imu_params, init_bias);
            std::size_t j = sample_cursor;
            GNSSTime prev_time = t0;
            std::size_t integrated = 0;
            while (j < imu_samples.size() && imu_samples[j].time < t1) {
                const double dt = imu_samples[j].time - prev_time;
                if (dt > 1e-9) {
                    pim.integrateMeasurement(gtsam::Vector3(imu_samples[j].accel_raw),
                                             gtsam::Vector3(imu_samples[j].gyro_raw_radps), dt);
                    ++integrated;
                }
                prev_time = imu_samples[j].time;
                ++j;
            }
            const double dt_tail = t1 - prev_time;
            if (integrated > 0 && dt_tail > 1e-9 && j > 0) {
                pim.integrateMeasurement(gtsam::Vector3(imu_samples[j - 1].accel_raw),
                                         gtsam::Vector3(imu_samples[j - 1].gyro_raw_radps), dt_tail);
            }
            pims.push_back(pim);
            pim_valid[i] = integrated > 0;
            if (integrated > 0) ++imu_intervals;
        }

        // --- Dead-reckon per-epoch attitude seeds from the aligned initial
        // attitude (rotation residual ~0 at the seed). ---
        std::vector<Rot3> attitude_seed(num_epochs, init_attitude_nav);
        for (std::size_t i = 0; i + 1 < num_epochs; ++i) {
            attitude_seed[i + 1] =
                pim_valid[i] ? attitude_seed[i] * pims[i].deltaRij() : attitude_seed[i];
        }

        // GNSS antenna positions in nav, for translation + velocity seeds.
        std::vector<Point3> antenna_nav(num_epochs);
        for (std::size_t i = 0; i < num_epochs; ++i) {
            antenna_nav[i] = Point3(ecef2enu(
                problem.epochs[i].position_ecef - problem.imu.nav_origin_ecef,
                problem.imu.nav_origin_lat_rad, problem.imu.nav_origin_lon_rad));
        }

        // --- Insert pose / velocity / bias seeds ---
        for (std::size_t i = 0; i < num_epochs; ++i) {
            // body-in-nav translation: antenna at the DD position, offset back
            // through the (dead-reckoned) attitude's lever arm.
            const Point3 body_nav = antenna_nav[i] - attitude_seed[i] * lever_arm_body;
            initial.insert(positionKey(i), Pose3(attitude_seed[i], body_nav));

            Vector3d vel = problem.imu.init_velocity_nav;
            if (num_epochs >= 2) {
                const std::size_t a = (i + 1 < num_epochs) ? i : i - 1;
                const double dt = problem.epochs[a + 1].time - problem.epochs[a].time;
                if (dt > 1e-3) {
                    vel = (antenna_nav[a + 1] - antenna_nav[a]) / dt;
                }
            }
            initial.insert(velocityKey(i), gtsam::Vector3(vel));
            initial.insert(biasKey(i), init_bias);
        }

        // --- First-state priors (replace the 2a per-epoch rotation pin): they
        // anchor the IMU integration chain and resolve the gauge. Attitude is
        // now observable (gravity fixes roll/pitch; motion fixes yaw), so no
        // per-pose rotation pin is needed. Position stays fully DD-driven. ---
        gtsam::Vector6 pose_prior_sigmas;
        pose_prior_sigmas << problem.imu.init_attitude_sigma_roll_pitch_rad,
            problem.imu.init_attitude_sigma_roll_pitch_rad,
            problem.imu.init_attitude_sigma_yaw_rad,
            1e6, 1e6, 1e6;  // translation left free (DD constrains it)
        graph.addPrior(positionKey(0), initial.at<Pose3>(positionKey(0)),
                       gtsam::noiseModel::Diagonal::Sigmas(pose_prior_sigmas));
        graph.addPrior(velocityKey(0), gtsam::Vector3(problem.imu.init_velocity_nav),
                       gtsam::noiseModel::Isotropic::Sigma(3, problem.imu.init_velocity_sigma_mps));
        gtsam::Vector6 bias_prior_sigmas;
        bias_prior_sigmas << problem.imu.init_accel_bias_sigma, problem.imu.init_accel_bias_sigma,
            problem.imu.init_accel_bias_sigma, problem.imu.init_gyro_bias_sigma,
            problem.imu.init_gyro_bias_sigma, problem.imu.init_gyro_bias_sigma;
        graph.addPrior(biasKey(0), init_bias,
                       gtsam::noiseModel::Diagonal::Sigmas(bias_prior_sigmas));

        // --- Add the CombinedImuFactors (or a loose velocity/bias continuity
        // across an IMU dropout) ---
        for (std::size_t i = 0; i + 1 < num_epochs; ++i) {
            if (pim_valid[i]) {
                graph.emplace_shared<gtsam::CombinedImuFactor>(
                    positionKey(i), velocityKey(i), positionKey(i + 1), velocityKey(i + 1),
                    biasKey(i), biasKey(i + 1), pims[i]);
            } else {
                graph.emplace_shared<gtsam::BetweenFactor<gtsam::Vector3>>(
                    velocityKey(i), velocityKey(i + 1), gtsam::Vector3::Zero(),
                    gtsam::noiseModel::Isotropic::Sigma(3, 10.0));
                graph.emplace_shared<gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>>(
                    biasKey(i), biasKey(i + 1), gtsam::imuBias::ConstantBias(),
                    gtsam::noiseModel::Isotropic::Sigma(6, 1.0));
            }
        }
        result.diagnostics.imu_intervals = imu_intervals;
    } else if (use_pose3) {
        // 2a rotation-only gauge pin (no IMU): resolves the exact rotational
        // null space left by lever-arm-only position observations, without
        // perturbing the estimated antenna position beyond kRotationPinSigmaRad
        // * |lever_arm| (~0.06 mm at 1e-4 rad and the tokyo ~0.62 m lever arm).
        constexpr double kRotationPinSigmaRad = 1e-4;
        const auto rotation_noise = gtsam::noiseModel::Isotropic::Sigma(3, kRotationPinSigmaRad);
        for (std::size_t i = 0; i < num_epochs; ++i) {
            graph.emplace_shared<Pose3RotationPrior>(positionKey(i), Rot3(), rotation_noise);
        }
    }

    // Undifferenced factors need a per-epoch base receiver clock [s] plus, for
    // every non-GPS constellation, ONE global (time-constant) inter-system bias
    // node shared across all epochs -- matching the native backend's per-epoch
    // clock + per-constellation bias columns.
    const bool need_clock_states =
        !problem.pseudorange_factors.empty() || !problem.carrier_phase_factors.empty();
    std::set<gtsam::Key> inserted_clock_keys;
    std::set<int> inserted_isb_ordinals;
    auto ensureBaseClock = [&](std::size_t epoch) -> gtsam::Key {
        const gtsam::Key key = clockKey(epoch);
        if (inserted_clock_keys.insert(key).second) {
            initial.insert(key, epoch < num_epochs
                                    ? problem.epochs[epoch].receiver_clock_bias_m /
                                          constants::SPEED_OF_LIGHT
                                    : 0.0);
        }
        return key;
    };
    // Returns the ISB ordinal for a system: 0 means GPS/base (no ISB node),
    // >0 means a global ISB node was ensured for that constellation.
    auto ensureIsb = [&](GNSSSystem system) -> int {
        const int ordinal =
            clockGroupOrdinal(clockBiasGroup(system), config.use_inter_system_biases);
        if (ordinal != 0 && inserted_isb_ordinals.insert(ordinal).second) {
            initial.insert(isbKey(ordinal), 0.0);
        }
        return ordinal;
    };

    // Ambiguity nodes: one per AmbiguityState, in the units that state's
    // consuming factor expects (cycles for DD, meters for undifferenced).
    for (std::size_t i = 0; i < problem.ambiguity_states.size(); ++i) {
        const auto& ambiguity = problem.ambiguity_states[i];
        const double value =
            ambiguity.is_double_difference && ambiguity.wavelength_m > 0.0
                ? ambiguity.initial_ambiguity_m / ambiguity.wavelength_m
                : ambiguity.initial_ambiguity_m;
        initial.insert(ambiguityKey(i), value);
    }
    const bool need_dummy_ambiguity =
        std::any_of(problem.ambiguity_states.begin(), problem.ambiguity_states.end(),
                    [](const FGOProcessor::AmbiguityState& a) { return a.is_double_difference; });
    if (need_dummy_ambiguity) {
        initial.insert(dummyAmbiguityKey(), 0.0);
        // Pin the shared dummy ambiguity at 0 so the lumped libgnss DD ambiguity
        // lives entirely in ambRef (the estimate is then simply ambRef). The
        // dummy is shared across every DD carrier factor, which introduces a
        // common-mode null space (shift dummy + every ambRef by the same delta
        // leaves all carrier residuals unchanged); only this prior constrains
        // it. A near-zero sigma (e.g. 1e-6) pins it but makes the prior's
        // Hessian block ~1e12 against position blocks ~1e2, i.e. condition
        // number ~1e10 -> Cholesky returns a garbage step, every LM trial
        // diverges to inf error, and LM gives up at iteration 0. 1e-3 cycles
        // (~0.2 mm of range) is still "fixed" physically but keeps the system
        // well conditioned (~1e4).
        constexpr double kDummyAmbiguityPinSigmaCycles = 1e-3;
        graph.addPrior(dummyAmbiguityKey(), 0.0,
                       gtsam::noiseModel::Isotropic::Sigma(1, kDummyAmbiguityPinSigmaCycles));
    }

    // --- Undifferenced pseudorange / carrier-phase factors ---
    for (const auto& factor : problem.pseudorange_factors) {
        const gtsam::Key base_clock = ensureBaseClock(factor.epoch_index);
        const int ordinal = ensureIsb(factor.satellite.system);
        const auto noise = makeNoise(factor.sigma_m, config.use_robust_loss,
                                     config.pseudorange_huber_threshold_sigma);
        if (use_pose3) {
            if (ordinal == 0) {
                graph.emplace_shared<PseudorangeFactorPlainArm>(
                    positionKey(factor.epoch_index), base_clock,
                    factor.corrected_pseudorange_m, Point3(factor.satellite_position_ecef),
                    gnss_lever_arm, noise);
            } else {
                graph.emplace_shared<PseudorangeFactorISBArm>(
                    positionKey(factor.epoch_index), base_clock, isbKey(ordinal),
                    factor.corrected_pseudorange_m, Point3(factor.satellite_position_ecef),
                    gnss_lever_arm, noise);
            }
        } else if (ordinal == 0) {
            graph.emplace_shared<PseudorangeFactorPlain>(
                positionKey(factor.epoch_index), base_clock,
                factor.corrected_pseudorange_m, Point3(factor.satellite_position_ecef),
                noise);
        } else {
            graph.emplace_shared<PseudorangeFactorISB>(
                positionKey(factor.epoch_index), base_clock, isbKey(ordinal),
                factor.corrected_pseudorange_m, Point3(factor.satellite_position_ecef),
                noise);
        }
    }
    // Undifferenced carrier phase (rare here: use_carrier_phase_factors is off
    // in the DD RTK config) uses only the base clock; the ISB affects code and
    // phase identically and cancels in the DD path, so this is adequate for the
    // SPP-seed carrier path and can gain its own ISB term if PPP-RTK needs it.
    // Not yet ported to Pose3 (milestone 2a targets the pure-DD RTK path,
    // where use_carrier_phase_factors is off); skip with a warning rather
    // than silently mixing a Point3 factor into a Pose3 graph.
    for (const auto& factor : problem.carrier_phase_factors) {
        if (factor.ambiguity_index >= problem.ambiguity_states.size()) {
            continue;
        }
        if (use_pose3) {
            std::fprintf(stderr,
                         "[fgo_gtsam_backend] WARNING: undifferenced carrier_phase_factors are "
                         "not yet supported with use_pose3_state; skipping %zu factor(s).\n",
                         problem.carrier_phase_factors.size());
            break;
        }
        const gtsam::Key base_clock = ensureBaseClock(factor.epoch_index);
        const auto noise = makeNoise(factor.sigma_m, config.use_robust_loss,
                                     config.carrier_phase_huber_threshold_sigma);
        graph.emplace_shared<gtsam::CarrierPhaseFactor>(
            positionKey(factor.epoch_index), base_clock,
            ambiguityKey(factor.ambiguity_index), factor.corrected_carrier_m,
            Point3(factor.satellite_position_ecef), 0.0, noise);
    }

    // --- Ordinary (single-receiver) TDCP factors ---
    // Keep the exact v1 temporal carrier constraint in the IMU-coupled Pose3
    // graph.  This is deliberately separate from the base-dependent
    // single/double-difference paths: a no-base smartphone run has one
    // receiver clock per epoch and one undifferenced carrier delta per
    // satellite.  The factor uses the same sigma/Huber contract as the Eigen
    // backend; invalid epoch indices are skipped and counted as not inserted.
    std::size_t ordinary_tdcp_inserted = 0;
    if (use_pose3 && config.use_tdcp_factors) {
        for (const auto& factor : problem.tdcp_factors) {
            if (factor.previous_epoch_index >= num_epochs ||
                factor.current_epoch_index >= num_epochs ||
                factor.current_epoch_index <= factor.previous_epoch_index) {
                continue;
            }
            if (!std::isfinite(factor.delta_carrier_m) ||
                !factor.previous_satellite_position_ecef.allFinite() ||
                !factor.current_satellite_position_ecef.allFinite()) {
                continue;
            }
            const gtsam::Key previous_clock =
                ensureBaseClock(factor.previous_epoch_index);
            const gtsam::Key current_clock =
                ensureBaseClock(factor.current_epoch_index);
            const auto noise = makeNoise(
                factor.sigma_m, config.use_robust_loss,
                config.tdcp_huber_threshold_sigma);
            graph.emplace_shared<TimeDifferencedCarrierFactorArm>(
                positionKey(factor.previous_epoch_index), previous_clock,
                positionKey(factor.current_epoch_index), current_clock,
                Point3(factor.previous_satellite_position_ecef),
                Point3(factor.current_satellite_position_ecef),
                factor.delta_carrier_m, gnss_lever_arm, noise);
            ++ordinary_tdcp_inserted;
        }
    }
    result.diagnostics.tdcp_factors_inserted = ordinary_tdcp_inserted;

    // --- Double-difference pseudorange factors ---
    for (const auto& factor : problem.double_difference_pseudorange_factors) {
        const gtsam::gnss::DoubleDifferenceData dd{
            factor.rover_satellite_model.corrected_pseudorange_m,
            factor.base_satellite_model.corrected_pseudorange_m,
            factor.rover_reference_model.corrected_pseudorange_m,
            factor.base_reference_model.corrected_pseudorange_m,
            Point3(factor.rover_satellite_position_ecef),
            Point3(factor.rover_reference_position_ecef),
            Point3(factor.base_satellite_position_ecef),
            Point3(factor.base_reference_position_ecef),
            Point3(factor.base_position_ecef),
        };
        checkObservedDdMatches(dd.observed(), factor.observed_dd_pseudorange_m,
                              "DD pseudorange");

        const auto noise = makeNoise(factor.sigma_m, config.use_robust_loss,
                                     config.pseudorange_huber_threshold_sigma);
        if (use_imu) {
            // 2b: pose is body-in-nav (ENU) -> feed ecef_T_nav so the factor
            // reconstructs the antenna ECEF from the nav-frame pose.
            graph.emplace_shared<gtsam::DoubleDifferencePseudorangeFactorArm>(
                positionKey(factor.epoch_index),
                dd.rovRef, dd.baseRef, dd.rovTarget, dd.baseTarget,
                dd.satRefRov, dd.satTargetRov, dd.satRefBase, dd.satTargetBase,
                dd.basePos, lever_arm_body, ecef_T_nav, noise);
        } else if (use_pose3) {
            // 2a: pose expressed directly in ECEF (no ecef_T_nav).
            graph.emplace_shared<gtsam::DoubleDifferencePseudorangeFactorArm>(
                positionKey(factor.epoch_index),
                dd.rovRef, dd.baseRef, dd.rovTarget, dd.baseTarget,
                dd.satRefRov, dd.satTargetRov, dd.satRefBase, dd.satTargetBase,
                dd.basePos, lever_arm_body, noise);
        } else {
            graph.emplace_shared<gtsam::DoubleDifferencePseudorangeFactor>(
                positionKey(factor.epoch_index),
                dd.rovRef, dd.baseRef, dd.rovTarget, dd.baseTarget,
                dd.satRefRov, dd.satTargetRov, dd.satRefBase, dd.satTargetBase,
                dd.basePos, noise);
        }
    }

    // --- Double-difference carrier-phase factors ---
    for (const auto& factor : problem.double_difference_carrier_factors) {
        if (factor.ambiguity_index >= problem.ambiguity_states.size()) {
            continue;
        }
        const auto& ambiguity = problem.ambiguity_states[factor.ambiguity_index];
        const gtsam::gnss::DoubleDifferenceData dd{
            factor.rover_satellite_model.corrected_carrier_m,
            factor.base_satellite_model.corrected_carrier_m,
            factor.rover_reference_model.corrected_carrier_m,
            factor.base_reference_model.corrected_carrier_m,
            Point3(factor.rover_satellite_position_ecef),
            Point3(factor.rover_reference_position_ecef),
            Point3(factor.base_satellite_position_ecef),
            Point3(factor.base_reference_position_ecef),
            Point3(factor.base_position_ecef),
        };
        checkObservedDdMatches(dd.observed(), factor.observed_dd_carrier_m, "DD carrier");

        const auto noise = makeNoise(factor.sigma_m, config.use_robust_loss,
                                     config.carrier_phase_huber_threshold_sigma);
        if (use_imu) {
            graph.emplace_shared<gtsam::DoubleDifferenceCarrierPhaseFactorArm>(
                positionKey(factor.epoch_index),
                ambiguityKey(factor.ambiguity_index),  // ambRef <- real DD ambiguity node
                dummyAmbiguityKey(),                   // ambTarget <- pinned at 0
                dd.rovRef, dd.baseRef, dd.rovTarget, dd.baseTarget,
                dd.satRefRov, dd.satTargetRov, dd.satRefBase, dd.satTargetBase,
                dd.basePos, ambiguity.wavelength_m, lever_arm_body, ecef_T_nav, noise);
        } else if (use_pose3) {
            graph.emplace_shared<gtsam::DoubleDifferenceCarrierPhaseFactorArm>(
                positionKey(factor.epoch_index),
                ambiguityKey(factor.ambiguity_index),  // ambRef <- real DD ambiguity node
                dummyAmbiguityKey(),                   // ambTarget <- pinned at 0
                dd.rovRef, dd.baseRef, dd.rovTarget, dd.baseTarget,
                dd.satRefRov, dd.satTargetRov, dd.satRefBase, dd.satTargetBase,
                dd.basePos, ambiguity.wavelength_m, lever_arm_body, noise);
        } else {
            graph.emplace_shared<gtsam::DoubleDifferenceCarrierPhaseFactor>(
                positionKey(factor.epoch_index),
                ambiguityKey(factor.ambiguity_index),  // ambRef <- real DD ambiguity node
                dummyAmbiguityKey(),                   // ambTarget <- pinned at 0
                dd.rovRef, dd.baseRef, dd.rovTarget, dd.baseTarget,
                dd.satRefRov, dd.satTargetRov, dd.satRefBase, dd.satTargetBase,
                dd.basePos, ambiguity.wavelength_m, noise);
        }
    }

    // --- Ambiguity priors ---
    if (config.use_ambiguity_priors && config.ambiguity_prior_sigma_m > 0.0) {
        for (std::size_t i = 0; i < problem.ambiguity_states.size(); ++i) {
            const auto& ambiguity = problem.ambiguity_states[i];
            if (ambiguity.is_double_difference && ambiguity.wavelength_m > 0.0) {
                const double mean = ambiguity.initial_ambiguity_m / ambiguity.wavelength_m;
                const double sigma = config.ambiguity_prior_sigma_m / ambiguity.wavelength_m;
                graph.addPrior(ambiguityKey(i), mean, gtsam::noiseModel::Isotropic::Sigma(1, sigma));
            } else {
                graph.addPrior(ambiguityKey(i), ambiguity.initial_ambiguity_m,
                               gtsam::noiseModel::Isotropic::Sigma(1, config.ambiguity_prior_sigma_m));
            }
        }
    }

    // --- Ambiguity continuity (BetweenFactor<double>) ---
    for (const auto& factor : problem.ambiguity_between_factors) {
        if (factor.previous_ambiguity_index >= problem.ambiguity_states.size() ||
            factor.current_ambiguity_index >= problem.ambiguity_states.size()) {
            continue;
        }
        const auto& ambiguity = problem.ambiguity_states[factor.current_ambiguity_index];
        const double wavelength = ambiguity.wavelength_m;
        const double sigma_cycles = wavelength > 0.0 ? factor.sigma_m / wavelength : factor.sigma_m;
        graph.emplace_shared<gtsam::BetweenFactor<double>>(
            ambiguityKey(factor.previous_ambiguity_index),
            ambiguityKey(factor.current_ambiguity_index), 0.0,
            gtsam::noiseModel::Isotropic::Sigma(1, std::max(1e-9, sigma_cycles)));
    }

    // --- Position motion (random-walk) factors between consecutive epochs ---
    if (config.use_motion_factors && config.use_position_motion_factors &&
        config.motion_sigma_m > 0.0) {
        if (use_pose3) {
            // Pose3 analog of the Point3 zero-motion BetweenFactor below: same
            // loose translation sigma (motion_sigma_m, e.g. 100 m -- a
            // near-no-op regularizer at DD noise levels), and an even looser
            // rotation sigma so it never competes with the dedicated
            // Pose3RotationPrior gauge pin above.
            constexpr double kLooseRotationSigmaRad = 10.0;
            gtsam::Vector6 sigmas;
            sigmas << kLooseRotationSigmaRad, kLooseRotationSigmaRad, kLooseRotationSigmaRad,
                config.motion_sigma_m, config.motion_sigma_m, config.motion_sigma_m;
            const auto noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
            for (std::size_t i = 1; i < num_epochs; ++i) {
                graph.emplace_shared<gtsam::BetweenFactor<Pose3>>(
                    positionKey(i - 1), positionKey(i), Pose3(Rot3(), Point3(0.0, 0.0, 0.0)),
                    noise);
            }
        } else {
            const auto noise = gtsam::noiseModel::Isotropic::Sigma(3, config.motion_sigma_m);
            for (std::size_t i = 1; i < num_epochs; ++i) {
                graph.emplace_shared<gtsam::BetweenFactor<Point3>>(
                    positionKey(i - 1), positionKey(i), Point3(0.0, 0.0, 0.0), noise);
            }
        }
    }

    // The native v1 motion graph also contains an independent receiver-clock
    // random-walk row for each adjacent epoch.  Pose3's BetweenFactor above
    // only constrains pose translation/rotation, so retain the clock part
    // explicitly in seconds (the public configuration is in meters, matching
    // the native clock state and v1 profile).  A detected common GPS code jump
    // receives the same loose 1e6 m gate used by the Eigen backend.
    if (need_clock_states && config.use_motion_factors && config.use_clock_motion_factors &&
        config.clock_motion_sigma_m > 0.0 && num_epochs >= 2) {
        for (std::size_t i = 1; i < num_epochs; ++i) {
            const gtsam::Key previous_clock = ensureBaseClock(i - 1);
            const gtsam::Key current_clock = ensureBaseClock(i);
            const bool clock_jump =
                i < problem.clock_jumps.size() && problem.clock_jumps[i];
            const double sigma_m = clock_jump ? 1.0e6 : config.clock_motion_sigma_m;
            graph.emplace_shared<gtsam::BetweenFactor<double>>(
                previous_clock, current_clock, 0.0,
                gtsam::noiseModel::Isotropic::Sigma(
                    1, sigma_m / gtsam::gnss::C_LIGHT));
        }
    }

    result.diagnostics.motion_factors =
        (config.use_motion_factors && num_epochs >= 2)
            ? num_epochs - 1
            : 0;

    // --- Optional absolute priors (disabled by default: sigma <= 0) ---
    if (config.position_prior_sigma_m > 0.0) {
        if (use_pose3) {
            constexpr double kLooseRotationSigmaRad = 1e6;
            gtsam::Vector6 sigmas;
            sigmas << kLooseRotationSigmaRad, kLooseRotationSigmaRad, kLooseRotationSigmaRad,
                config.position_prior_sigma_m, config.position_prior_sigma_m,
                config.position_prior_sigma_m;
            const auto noise = gtsam::noiseModel::Diagonal::Sigmas(sigmas);
            for (std::size_t i = 0; i < num_epochs; ++i) {
                graph.addPrior(positionKey(i),
                               Pose3(Rot3(), Point3(problem.epochs[i].position_ecef) - lever_arm_body),
                               noise);
            }
        } else {
            const auto noise = gtsam::noiseModel::Isotropic::Sigma(3, config.position_prior_sigma_m);
            for (std::size_t i = 0; i < num_epochs; ++i) {
                graph.addPrior(positionKey(i), Point3(problem.epochs[i].position_ecef), noise);
            }
        }
    }
    if (need_clock_states && config.clock_prior_sigma_m > 0.0) {
        const auto noise = gtsam::noiseModel::Isotropic::Sigma(
            1, config.clock_prior_sigma_m / constants::SPEED_OF_LIGHT);
        for (const gtsam::Key key : inserted_clock_keys) {
            graph.addPrior(key, initial.at<double>(key), noise);
        }
    }

    result.diagnostics.graph_factors = graph.size();
    result.diagnostics.graph_values = initial.size();

    // --- Solve ---
    gtsam::LevenbergMarquardtParams params;
    if (std::getenv("GNSSPP_GTSAM_LM_VERBOSE") != nullptr) {
        params.setVerbosityLM("SUMMARY");
    }
    params.setMaxIterations(std::max(1, config.max_iterations));
    params.setAbsoluteErrorTol(config.absolute_cost_convergence_threshold > 0.0
                                   ? config.absolute_cost_convergence_threshold
                                   : 1e-10);
    params.setRelativeErrorTol(config.relative_cost_convergence_threshold > 0.0
                                   ? config.relative_cost_convergence_threshold
                                   : 1e-8);

    const double initial_cost = graph.error(initial);
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, params);
    gtsam::Values optimized;
    bool solved = true;
    try {
        optimized = optimizer.optimize();
    } catch (const std::exception& e) {
        std::fprintf(stderr, "[fgo_gtsam_backend] LM optimize() threw: %s\n", e.what());
        solved = false;
        optimized = initial;
    }
    const double final_cost = graph.error(optimized);

    result.diagnostics.iterations = static_cast<int>(optimizer.iterations());
    result.diagnostics.converged = solved;
    result.diagnostics.initial_cost = initial_cost;
    result.diagnostics.final_cost = final_cost;

    // --- Per-epoch / partial LAMBDA integer fixing ---
    // Mirrors the native backend's per-epoch LAMBDA path (fgo.cpp
    // process_epoch_lambda_work): group each epoch's DD ambiguities, take their
    // joint marginal covariance (here via gtsam::Marginals over the whole
    // graph), run the SAME libgnss::lambdaSearch with the same ratio test, and
    // -- when use_epoch_lambda_fixed_output is set -- snap that epoch's position
    // to the fixed solution via the position/ambiguity cross-covariance. GTSAM
    // ambiguity nodes are already in cycles, so no wavelength scaling is needed.
    std::map<std::size_t, int> fixed_cycles_by_index;      // amb index -> integer cycles
    std::map<std::size_t, double> fixed_residual_by_index;  // amb index -> float-int residual
    std::vector<bool> epoch_fixed(num_epochs, false);
    std::vector<int> epoch_fixed_count(num_epochs, 0);
    std::vector<double> epoch_ratio(num_epochs, 0.0);
    std::vector<Point3> epoch_output_position(num_epochs, Point3(0, 0, 0));
    for (std::size_t i = 0; i < num_epochs; ++i) {
        epoch_output_position[i] = antennaPositionOf(optimized, i);
    }
    std::size_t total_fixed_ambiguities = 0;
    std::size_t covariance_failures = 0;
    double best_ratio = 0.0;

    const bool do_fix =
        solved && config.use_lambda_ambiguity_fix &&
        !problem.double_difference_carrier_factors.empty();
    if (do_fix) {
        // DD ambiguity indices present at each epoch.
        std::map<std::size_t, std::set<std::size_t>> ambiguity_indices_by_epoch;
        for (const auto& factor : problem.double_difference_carrier_factors) {
            if (factor.epoch_index < num_epochs &&
                factor.ambiguity_index < problem.ambiguity_states.size()) {
                ambiguity_indices_by_epoch[factor.epoch_index].insert(factor.ambiguity_index);
            }
        }
        // Same minimum-candidate gate as the native per-epoch path.
        const int min_candidates =
            std::max(6, std::max(1, config.min_fixed_ambiguities + 1));

        gtsam::Marginals marginals;
        bool have_marginals = false;
        try {
            marginals = gtsam::Marginals(graph, optimized, gtsam::Marginals::CHOLESKY);
            have_marginals = true;
        } catch (const std::exception& e) {
            std::fprintf(stderr, "[fgo_gtsam_backend] Marginals failed, no fixing: %s\n",
                         e.what());
        }

        for (auto it = ambiguity_indices_by_epoch.begin();
             have_marginals && it != ambiguity_indices_by_epoch.end(); ++it) {
            const std::size_t epoch_index = it->first;
            if (static_cast<int>(it->second.size()) < min_candidates) {
                continue;
            }
            // Deterministic candidate order (satellite/reference), matching the
            // native path's sort.
            std::vector<std::size_t> candidates(it->second.begin(), it->second.end());
            // MF-AR step 1: one band per satellite for the integer ratio test.
            if (config.double_difference_lambda_one_band_per_satellite &&
                candidates.size() > 1) {
                candidates = selectOneBandPerSatellite(candidates, problem);
            }
            std::sort(candidates.begin(), candidates.end(),
                      [&](std::size_t a, std::size_t b) {
                          const auto& sa = problem.ambiguity_states[a];
                          const auto& sb = problem.ambiguity_states[b];
                          return std::tie(sa.satellite, sa.reference_satellite, sa.signal, a) <
                                 std::tie(sb.satellite, sb.reference_satellite, sb.signal, b);
                      });
            const int n = static_cast<int>(candidates.size());

            gtsam::KeyVector keys;
            keys.reserve(candidates.size() + 1);
            keys.push_back(positionKey(epoch_index));
            for (std::size_t idx : candidates) {
                keys.push_back(ambiguityKey(idx));
            }

            Eigen::VectorXd float_amb(n);
            Eigen::MatrixXd q_amb(n, n);
            Eigen::MatrixXd pos_amb(3, n);  // Cov(antenna_position, ambiguity_row)
            bool ok = true;
            try {
                const gtsam::JointMarginal joint = marginals.jointMarginalCovariance(keys);
                const gtsam::Key pos_key = positionKey(epoch_index);
                // Pose3 mode: the joint marginal gives Cov(pose_tangent, amb),
                // a 6x1 block; propagate it through the antenna Jacobian
                // (3x6, constant for this epoch/optimized-value) to get
                // Cov(antenna_position, amb), matching the Point3 case's
                // native 3x1 block below.
                gtsam::Matrix36 H_antenna_pose;
                if (use_pose3) {
                    gtsam::gnss::LeverArm::PoseFrame frame;
                    gnss_lever_arm.antennaPosition(optimized.at<Pose3>(pos_key), &frame);
                    for (int r3 = 0; r3 < 3; ++r3) {
                        gtsam::Matrix13 unit = gtsam::Matrix13::Zero();
                        unit(0, r3) = 1.0;
                        H_antenna_pose.row(r3) = gnss_lever_arm.antennaPoseJacobian(unit, frame);
                    }
                }
                for (int r = 0; r < n && ok; ++r) {
                    const gtsam::Key rk = ambiguityKey(candidates[r]);
                    float_amb(r) = optimized.at<double>(rk);
                    const gtsam::Matrix pr = joint(pos_key, rk);  // 6x1 (Pose3) or 3x1 (Point3)
                    const Eigen::Vector3d pr3 = use_pose3 ? Eigen::Vector3d(H_antenna_pose * pr)
                                                          : Eigen::Vector3d(pr.col(0));
                    pos_amb(0, r) = pr3(0);
                    pos_amb(1, r) = pr3(1);
                    pos_amb(2, r) = pr3(2);
                    for (int c = 0; c < n; ++c) {
                        q_amb(r, c) = joint(rk, ambiguityKey(candidates[c]))(0, 0);
                    }
                }
            } catch (const std::exception&) {
                ok = false;
            }
            if (!ok || !float_amb.allFinite() || !q_amb.allFinite()) {
                ++covariance_failures;
                continue;
            }

            q_amb = 0.5 * (q_amb + q_amb.transpose());
            for (int i = 0; i < n; ++i) {
                q_amb(i, i) += std::max(1e-12, std::abs(q_amb(i, i)) * 1e-9);
            }

            Eigen::VectorXd fixed_amb;
            double ratio = 0.0;
            ++result.diagnostics.lambda_ambiguity_attempts;
            result.diagnostics.lambda_ambiguity_candidates += static_cast<std::size_t>(n);
            const bool solved_lambda = lambdaSearch(float_amb, q_amb, fixed_amb, ratio);
            if (!solved_lambda) {
                continue;
            }
            result.diagnostics.lambda_ambiguity_fix_solved = true;
            best_ratio = std::max(best_ratio, ratio);
            epoch_ratio[epoch_index] = ratio;
            const bool fixed_epoch =
                std::isfinite(ratio) &&
                (config.lambda_ratio_threshold <= 0.0 || ratio > config.lambda_ratio_threshold) &&
                fixed_amb.size() == n;
            if (!fixed_epoch) {
                continue;
            }

            epoch_fixed[epoch_index] = true;
            epoch_fixed_count[epoch_index] = n;
            total_fixed_ambiguities += static_cast<std::size_t>(n);
            result.diagnostics.lambda_ambiguity_fix_used = true;
            result.diagnostics.lambda_ambiguity_used_candidates += static_cast<std::size_t>(n);
            for (int r = 0; r < n; ++r) {
                const int fixed_int = static_cast<int>(std::lround(fixed_amb(r)));
                fixed_cycles_by_index[candidates[r]] = fixed_int;
                fixed_residual_by_index[candidates[r]] = float_amb(r) - static_cast<double>(fixed_int);
            }

            // Position snap to the fixed ambiguities (only when requested):
            //   x_fixed = x_float - Cov(x,N) Cov(N,N)^-1 (N_float - N_fixed)
            if (config.use_epoch_lambda_fixed_output) {
                const Eigen::VectorXd delta = float_amb - fixed_amb;
                const Eigen::LDLT<Eigen::MatrixXd> ldlt(q_amb);
                if (ldlt.info() == Eigen::Success) {
                    const Eigen::VectorXd correction = ldlt.solve(delta);
                    if (correction.allFinite()) {
                        const Eigen::Vector3d pos_delta = pos_amb * correction;
                        if (pos_delta.allFinite()) {
                            epoch_output_position[epoch_index] =
                                antennaPositionOf(optimized, epoch_index) - Point3(pos_delta);
                        }
                    }
                }
            }
        }
    }

    const bool any_fixed = total_fixed_ambiguities > 0;
    result.diagnostics.lambda_ambiguity_ratio = best_ratio;
    result.diagnostics.fixed_solution = any_fixed;
    result.diagnostics.fixed_ambiguities = total_fixed_ambiguities;
    if (covariance_failures > 0) {
        std::fprintf(stderr,
                     "[fgo_gtsam_backend] %zu epoch(s) skipped: non-PSD / marginal failure\n",
                     covariance_failures);
    }

    // --- Map ambiguity estimates ---
    result.ambiguity_estimates.reserve(problem.ambiguity_states.size());
    for (std::size_t i = 0; i < problem.ambiguity_states.size(); ++i) {
        const auto& ambiguity = problem.ambiguity_states[i];
        FGOProcessor::AmbiguityEstimate estimate;
        estimate.satellite = ambiguity.satellite;
        estimate.signal = ambiguity.signal;
        estimate.segment_index = ambiguity.segment_index;
        estimate.wavelength_m = ambiguity.wavelength_m;
        const double value = optimized.at<double>(ambiguityKey(i));
        if (ambiguity.is_double_difference && ambiguity.wavelength_m > 0.0) {
            estimate.ambiguity_cycles = value;
            estimate.ambiguity_m = value * ambiguity.wavelength_m;
        } else {
            estimate.ambiguity_m = value;
            estimate.ambiguity_cycles =
                ambiguity.wavelength_m > 0.0 ? value / ambiguity.wavelength_m : 0.0;
        }
        const auto fixed_it = fixed_cycles_by_index.find(i);
        if (fixed_it != fixed_cycles_by_index.end()) {
            estimate.is_fixed = true;
            estimate.fixed_by_lambda = true;
            estimate.fixed_cycles = fixed_it->second;
            estimate.fixed_ambiguity_m = fixed_it->second * ambiguity.wavelength_m;
            estimate.fix_residual_cycles = fixed_residual_by_index.at(i);
        }
        result.ambiguity_estimates.push_back(estimate);
    }

    // --- Map per-epoch solutions ---
    const bool have_ambiguities = !problem.ambiguity_states.empty();
    for (std::size_t i = 0; i < num_epochs; ++i) {
        PositionSolution solution;
        solution.time = problem.epochs[i].time;
        if (epoch_fixed[i]) {
            solution.status = SolutionStatus::FIXED;
        } else if (have_ambiguities) {
            solution.status = SolutionStatus::FLOAT;
        } else {
            solution.status = SolutionStatus::SPP;
        }
        // Fixed position snap only takes effect when use_epoch_lambda_fixed_output
        // is set; otherwise epoch_output_position holds the float position.
        solution.position_ecef = epoch_output_position[i];
        if (need_clock_states && optimized.exists(clockKey(i))) {
            solution.receiver_clock_bias = optimized.at<double>(clockKey(i));
        }
        solution.num_frequencies = 1;
        solution.ratio = epoch_ratio[i];
        solution.num_fixed_ambiguities = epoch_fixed_count[i];
        solution.iterations = result.diagnostics.iterations;

        double lat = 0.0;
        double lon = 0.0;
        double height = 0.0;
        ecef2geodetic(solution.position_ecef, lat, lon, height);
        solution.position_geodetic = GeodeticCoord(lat, lon, height);

        result.solution.addSolution(solution);
    }

    // --- Milestone 2b: export estimated attitude (roll/pitch/heading, deg)
    // and ENU velocity so callers can confirm attitude is now observable. ---
    if (use_imu) {
        result.epoch_attitude_rpy_deg.resize(num_epochs);
        result.epoch_velocity_nav_mps.resize(num_epochs);
        constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
        for (std::size_t i = 0; i < num_epochs; ++i) {
            const Rot3 R_body_to_nav = optimized.at<Pose3>(positionKey(i)).rotation();
            const gtsam::Matrix3 R = R_body_to_nav.matrix();
            const Eigen::Vector3d fwd = R.col(0);   // body forward in ENU
            const Eigen::Vector3d left = R.col(1);  // body left in ENU
            // heading: clockwise from North (atan2(E, N)); pitch: nose-up from
            // the forward axis Up component; roll: from the left axis Up
            // component. Matches the reference.csv Roll/Pitch/Heading convention.
            double heading = std::atan2(fwd.x(), fwd.y()) * kRadToDeg;
            if (heading < 0.0) heading += 360.0;
            const double pitch =
                std::asin(std::max(-1.0, std::min(1.0, fwd.z()))) * kRadToDeg;
            const double roll = std::atan2(left.z(), R.col(2).z()) * kRadToDeg;
            result.epoch_attitude_rpy_deg[i] = Vector3d(roll, pitch, heading);
            result.epoch_velocity_nav_mps[i] =
                Vector3d(optimized.at<gtsam::Vector3>(velocityKey(i)));
        }
    }

    // --- Residual RMS diagnostics (recomputed from the optimized state so
    // they are directly comparable with the native backend's fields) ---
    auto accumulate_rms = [](double sum, std::size_t count) {
        return count > 0 ? std::sqrt(sum / static_cast<double>(count)) : 0.0;
    };

    {
        double sum = 0.0;
        std::size_t count = 0;
        for (const auto& factor : problem.double_difference_pseudorange_factors) {
            const Point3 position = antennaPositionOf(optimized, factor.epoch_index);
            const gtsam::gnss::DoubleDifferenceData dd{
                factor.rover_satellite_model.corrected_pseudorange_m,
                factor.base_satellite_model.corrected_pseudorange_m,
                factor.rover_reference_model.corrected_pseudorange_m,
                factor.base_reference_model.corrected_pseudorange_m,
                Point3(factor.rover_satellite_position_ecef),
                Point3(factor.rover_reference_position_ecef),
                Point3(factor.base_satellite_position_ecef),
                Point3(factor.base_reference_position_ecef),
                Point3(factor.base_position_ecef),
            };
            const double residual = dd.observed() - dd.model(position);
            sum += residual * residual;
            ++count;
        }
        result.diagnostics.double_difference_pseudorange_residual_rms_m =
            accumulate_rms(sum, count);
    }
    {
        double sum = 0.0;
        std::size_t count = 0;
        for (const auto& factor : problem.double_difference_carrier_factors) {
            if (factor.ambiguity_index >= problem.ambiguity_states.size()) {
                continue;
            }
            const auto& ambiguity = problem.ambiguity_states[factor.ambiguity_index];
            const Point3 position = antennaPositionOf(optimized, factor.epoch_index);
            const double amb_cycles = optimized.at<double>(ambiguityKey(factor.ambiguity_index));
            const gtsam::gnss::DoubleDifferenceData dd{
                factor.rover_satellite_model.corrected_carrier_m,
                factor.base_satellite_model.corrected_carrier_m,
                factor.rover_reference_model.corrected_carrier_m,
                factor.base_reference_model.corrected_carrier_m,
                Point3(factor.rover_satellite_position_ecef),
                Point3(factor.rover_reference_position_ecef),
                Point3(factor.base_satellite_position_ecef),
                Point3(factor.base_reference_position_ecef),
                Point3(factor.base_position_ecef),
            };
            const double residual =
                dd.observed() - (dd.model(position) + ambiguity.wavelength_m * amb_cycles);
            sum += residual * residual;
            ++count;
        }
        result.diagnostics.double_difference_carrier_residual_rms_m = accumulate_rms(sum, count);
    }
    {
        double sum = 0.0;
        std::size_t count = 0;
        for (const auto& factor : problem.pseudorange_factors) {
            const Point3 position = antennaPositionOf(optimized, factor.epoch_index);
            const int ordinal = clockGroupOrdinal(
                clockBiasGroup(factor.satellite.system), config.use_inter_system_biases);
            double clock_s = optimized.at<double>(clockKey(factor.epoch_index));
            if (ordinal != 0 && optimized.exists(isbKey(ordinal))) {
                clock_s += optimized.at<double>(isbKey(ordinal));
            }
            // Plain Euclidean range to match the native backend and the factor
            // model above (satellite positions are already earth-rotation
            // corrected; no additional Sagnac term).
            const double range =
                (Point3(factor.satellite_position_ecef) - position).norm();
            const double predicted = range + constants::SPEED_OF_LIGHT * clock_s;
            const double residual = factor.corrected_pseudorange_m - predicted;
            sum += residual * residual;
            ++count;
        }
        result.diagnostics.residual_rms_m = accumulate_rms(sum, count);
    }

    const auto end_time = std::chrono::high_resolution_clock::now();
    result.diagnostics.processing_time_ms =
        std::chrono::duration_cast<std::chrono::duration<double, std::milli>>(end_time -
                                                                               start_time)
            .count();
    result.diagnostics.total_processing_time_ms = result.diagnostics.processing_time_ms;

    return result;
}

}  // namespace libgnss
