#include <libgnss++/algorithms/spp_velocity.hpp>

#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/signals.hpp>

#include <cmath>

namespace libgnss {
namespace spp_velocity {

namespace {

// Elevation weighting mirrors spp.cpp's own pseudorange convention
// (var ~ 1/sin^2(el)): a satellite near the horizon gets a looser Doppler
// sigma than one overhead.
double elevationSinSquared(double elevation_rad) {
    double sin_el = std::sin(elevation_rad);
    if (sin_el < 0.1) {
        sin_el = 0.1;
    }
    return sin_el * sin_el;
}

}  // namespace

DopplerVelocityResult solveVelocity(const std::vector<DopplerObservation>& observations,
                                    const Eigen::Vector3d& receiver_position_ecef,
                                    double doppler_sigma_mps,
                                    int min_satellites) {
    DopplerVelocityResult result;

    if (min_satellites < 4) {
        min_satellites = 4;
    }
    if (!(doppler_sigma_mps > 0.0) || !std::isfinite(doppler_sigma_mps)) {
        doppler_sigma_mps = 0.5;
    }
    if (!receiver_position_ecef.allFinite()) {
        return result;
    }

    std::vector<const DopplerObservation*> usable;
    usable.reserve(observations.size());
    for (const auto& obs : observations) {
        if (!std::isfinite(obs.doppler_hz) || obs.doppler_hz == 0.0) {
            continue;
        }
        if (!(obs.frequency_hz > 0.0) || !std::isfinite(obs.frequency_hz)) {
            continue;
        }
        if (!obs.satellite_position_ecef.allFinite() || !obs.satellite_velocity_ecef.allFinite() ||
            !std::isfinite(obs.satellite_clock_drift)) {
            continue;
        }
        if ((obs.satellite_position_ecef - receiver_position_ecef).norm() < 1.0) {
            continue;
        }
        usable.push_back(&obs);
    }

    const int n = static_cast<int>(usable.size());
    if (n < min_satellites) {
        return result;
    }

    // Unknowns: [vx, vy, vz, receiver_clock_drift_mps].
    constexpr int kNumUnknowns = 4;
    Eigen::MatrixXd design_matrix = Eigen::MatrixXd::Zero(n, kNumUnknowns);
    Eigen::VectorXd residuals = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd weights = Eigen::VectorXd::Zero(n);

    for (int i = 0; i < n; ++i) {
        const auto& obs = *usable[static_cast<size_t>(i)];
        const Eigen::Vector3d los = (obs.satellite_position_ecef - receiver_position_ecef).normalized();

        design_matrix(i, 0) = -los.x();
        design_matrix(i, 1) = -los.y();
        design_matrix(i, 2) = -los.z();
        design_matrix(i, 3) = 1.0;

        // RINEX Doppler convention: positive Doppler = approaching satellite
        // (range decreasing), so measured range-rate = -doppler * wavelength.
        const double measured_range_rate_mps =
            -obs.doppler_hz * (constants::SPEED_OF_LIGHT / obs.frequency_hz);
        const double predicted_range_rate_known_terms_mps =
            obs.satellite_velocity_ecef.dot(los) -
            constants::SPEED_OF_LIGHT * obs.satellite_clock_drift;
        residuals(i) = measured_range_rate_mps - predicted_range_rate_known_terms_mps;

        const double sigma = doppler_sigma_mps / std::sqrt(elevationSinSquared(obs.elevation_rad));
        weights(i) = 1.0 / (sigma * sigma);
    }

    Eigen::MatrixXd weighted_design_matrix = Eigen::MatrixXd::Zero(n, kNumUnknowns);
    Eigen::VectorXd weighted_residuals = Eigen::VectorXd::Zero(n);
    for (int i = 0; i < n; ++i) {
        const double sqrt_weight = std::sqrt(weights(i));
        weighted_design_matrix.row(i) = design_matrix.row(i) * sqrt_weight;
        weighted_residuals(i) = residuals(i) * sqrt_weight;
    }

    Eigen::ColPivHouseholderQR<Eigen::MatrixXd> qr(weighted_design_matrix);
    if (qr.rank() < kNumUnknowns) {
        return result;
    }

    Eigen::VectorXd x = qr.solve(weighted_residuals);
    if (!x.allFinite()) {
        return result;
    }

    const Eigen::VectorXd post_fit_residuals = residuals - design_matrix * x;
    double sum_sq_weighted_residual = 0.0;
    double sum_sq_residual = 0.0;
    for (int i = 0; i < n; ++i) {
        sum_sq_weighted_residual += weights(i) * post_fit_residuals(i) * post_fit_residuals(i);
        sum_sq_residual += post_fit_residuals(i) * post_fit_residuals(i);
    }

    const int degrees_of_freedom = std::max(1, n - kNumUnknowns);
    // A-posteriori unit-weight variance scale, same idea as the a-priori/
    // a-posteriori DOP*sigma covariance convention used elsewhere in this
    // codebase's SPP covariance reporting: never let it shrink the reported
    // covariance below the a-priori weighting (floor at 1.0) so a lucky
    // 4-satellite solve is never reported as unrealistically confident.
    const double chi_square_per_dof = sum_sq_weighted_residual / degrees_of_freedom;
    const double sigma0_squared = std::max(1.0, chi_square_per_dof);

    // Reject grossly inconsistent fits outright (docs/design.md validation
    // on PPC tokyo/run1: an ungated, poor-geometry Doppler solve right after
    // an RTK reacquisition -- typically exactly 4-5 satellites, some at low
    // elevation post-outage -- fed the fusion filter's velocity update a
    // wrong-but-plausible-looking velocity, which the lever-arm/angular-rate
    // coupling term in fusion_measurement::buildGnssVelocityUpdate then
    // amplified into a full attitude flip that never recovered). A
    // chi-square-per-DOF this large (~5-sigma-equivalent) means the
    // observations are not mutually consistent with any single velocity/
    // clock-drift solution -- same reasoning as SPPProcessor's own
    // max_chi_square_per_dof gate (spp.hpp) -- so report not-ok rather than
    // a confident-looking but wrong result; callers must then leave
    // has_velocity false for this epoch, same as the <min_satellites case.
    constexpr double kMaxChiSquarePerDof = 25.0;
    if (!std::isfinite(chi_square_per_dof) || chi_square_per_dof > kMaxChiSquarePerDof) {
        return result;
    }

    Eigen::MatrixXd normal_matrix = weighted_design_matrix.transpose() * weighted_design_matrix;
    Eigen::MatrixXd covariance =
        normal_matrix.ldlt().solve(Eigen::MatrixXd::Identity(kNumUnknowns, kNumUnknowns));

    // Geometry-quality gate independent of the chi-square check above: with
    // exactly min_satellites (commonly 4) observations and 4 unknowns the
    // fit is exact (near-zero residual/chi-square *by construction*, not
    // because the geometry is good), so a near-degenerate satellite
    // configuration (e.g. all remaining satellites clustered in a narrow
    // patch of sky right after an urban-canyon RTK reacquisition) would
    // otherwise sail through the chi-square gate while still producing a
    // wildly uncertain (and, per the a-priori weighting alone, potentially
    // wrong) velocity. Checking the *unscaled* covariance diagonal here
    // catches that case: a well-conditioned 4-satellite fix has velocity
    // variance within a few (m/s)^2 of the a-priori Doppler sigma, while a
    // near-singular one blows up by orders of magnitude.
    constexpr double kMaxUnscaledVelocityVarianceM2S2 = 400.0;  // (20 m/s)^2
    if (!covariance.allFinite() ||
        covariance.diagonal().head<3>().maxCoeff() > kMaxUnscaledVelocityVarianceM2S2) {
        return result;
    }

    covariance *= sigma0_squared;

    result.ok = true;
    result.velocity_ecef = x.head<3>();
    result.velocity_covariance = covariance.block<3, 3>(0, 0);
    result.receiver_clock_drift = x(3) / constants::SPEED_OF_LIGHT;
    result.clock_drift_variance =
        covariance(3, 3) / (constants::SPEED_OF_LIGHT * constants::SPEED_OF_LIGHT);
    result.num_satellites_used = n;
    result.residual_rms_mps = std::sqrt(sum_sq_residual / n);
    return result;
}

DopplerVelocityResult solveVelocityFromObservations(const ObservationData& obs,
                                                    const NavigationData& nav,
                                                    const Eigen::Vector3d& receiver_position_ecef,
                                                    double doppler_sigma_mps,
                                                    int min_satellites) {
    DopplerVelocityResult empty_result;
    if (!receiver_position_ecef.allFinite()) {
        return empty_result;
    }

    std::vector<DopplerObservation> doppler_observations;
    doppler_observations.reserve(obs.observations.size());

    for (const auto& observation : obs.observations) {
        if (!observation.valid || !observation.has_doppler || observation.doppler == 0.0) {
            continue;
        }

        // Same two-step transmission-time / satellite-clock-bias refinement
        // SPPProcessor's own position solve uses (src/algorithms/spp.cpp).
        double travel_time_s = 0.075;  // ~1 Earth-radius-scale fallback if no pseudorange yet
        if (observation.has_pseudorange && observation.pseudorange > 0.0) {
            travel_time_s = observation.pseudorange / constants::SPEED_OF_LIGHT;
        }
        GNSSTime tx_time = obs.time - travel_time_s;

        Eigen::Vector3d sat_pos = Eigen::Vector3d::Zero();
        Eigen::Vector3d sat_vel = Eigen::Vector3d::Zero();
        double sat_clock_bias = 0.0;
        double sat_clock_drift = 0.0;
        if (!nav.calculateSatelliteState(observation.satellite, tx_time, sat_pos, sat_vel,
                                         sat_clock_bias, sat_clock_drift)) {
            continue;
        }
        tx_time = tx_time - sat_clock_bias;
        if (!nav.calculateSatelliteState(observation.satellite, tx_time, sat_pos, sat_vel,
                                         sat_clock_bias, sat_clock_drift)) {
            continue;
        }

        const double signal_travel_s =
            (sat_pos - receiver_position_ecef).norm() / constants::SPEED_OF_LIGHT;
        const double angle = constants::OMEGA_E * signal_travel_s;
        Eigen::Matrix3d earth_rotation;
        earth_rotation << std::cos(angle), std::sin(angle), 0.0,
                         -std::sin(angle), std::cos(angle), 0.0,
                          0.0, 0.0, 1.0;
        const Eigen::Vector3d corrected_sat_pos = earth_rotation * sat_pos;
        const Eigen::Vector3d corrected_sat_vel = earth_rotation * sat_vel;

        const Ephemeris* eph = nav.getEphemeris(observation.satellite, tx_time);
        const double frequency_hz = signalFrequencyHz(observation.signal, eph);
        if (!(frequency_hz > 0.0)) {
            continue;
        }

        const auto geometry = nav.calculateGeometry(receiver_position_ecef, corrected_sat_pos);

        DopplerObservation doppler_obs;
        doppler_obs.satellite_position_ecef = corrected_sat_pos;
        doppler_obs.satellite_velocity_ecef = corrected_sat_vel;
        doppler_obs.satellite_clock_drift = sat_clock_drift;
        doppler_obs.doppler_hz = observation.doppler;
        doppler_obs.frequency_hz = frequency_hz;
        doppler_obs.elevation_rad = geometry.elevation;
        doppler_observations.push_back(doppler_obs);
    }

    return solveVelocity(doppler_observations, receiver_position_ecef, doppler_sigma_mps,
                         min_satellites);
}

}  // namespace spp_velocity
}  // namespace libgnss
