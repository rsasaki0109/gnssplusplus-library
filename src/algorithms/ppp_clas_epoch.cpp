// CLAS-PPP epoch processing method for PPPProcessor.
// Split from ppp.cpp for modularity.

#include <libgnss++/algorithms/ppp.hpp>
#include <libgnss++/algorithms/ppp_ar.hpp>
#include <libgnss++/algorithms/ppp_clas.hpp>
#include <libgnss++/algorithms/kalman.hpp>
#include <libgnss++/algorithms/ppp_osr.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/core/signals.hpp>
#include <libgnss++/models/ionosphere.hpp>
#include <libgnss++/models/troposphere.hpp>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <set>
#include <vector>

namespace libgnss {

using PPPConfig = ppp_shared::PPPConfig;
using PPPState = ppp_shared::PPPState;
using PPPAmbiguityInfo = ppp_shared::PPPAmbiguityInfo;

namespace {

bool pppDebugEnabled() {
    return ppp_shared::pppDebugEnabled();
}

bool shouldDumpReceiverPositionTrace(const PPPConfig& config,
                                     const GNSSTime& time) {
    return pppDebugEnabled() &&
           config.wlnl_strict_claslib_parity &&
           time.tow >= 230420.0 - 1e-6 &&
           time.tow <= 230425.0 + 1e-6;
}

GNSSSystem claslibPntposClockMaskGroup(GNSSSystem system) {
    if (system == GNSSSystem::GLONASS) return GNSSSystem::GLONASS;
    if (system == GNSSSystem::Galileo) return GNSSSystem::Galileo;
    if (system == GNSSSystem::BeiDou) return GNSSSystem::BeiDou;
    return GNSSSystem::GPS;
}

int claslibPntposBiasColumn(GNSSSystem group) {
    switch (group) {
        case GNSSSystem::GLONASS: return 4;
        case GNSSSystem::Galileo: return 5;
        case GNSSSystem::BeiDou: return 6;
        default: return -1;
    }
}

double claslibBroadcastIonoDelay(const NavigationData& nav,
                                 const Vector3d& receiver_position,
                                 double azimuth,
                                 double elevation,
                                 const GNSSTime& time) {
    static constexpr double kDefaultAlpha[4] = {
        0.1118e-07, -0.7451e-08, -0.5961e-07, 0.1192e-06};
    static constexpr double kDefaultBeta[4] = {
        0.1167e+06, -0.2294e+06, -0.1311e+06, 0.1049e+07};
    double lat = 0.0;
    double lon = 0.0;
    double height = 0.0;
    ecef2geodetic(receiver_position, lat, lon, height);
    if (height < -1000.0 || elevation <= 0.0) {
        return 0.0;
    }
    double ion_norm = 0.0;
    for (int i = 0; i < 4; ++i) {
        ion_norm += nav.ionosphere_model.alpha[i] * nav.ionosphere_model.alpha[i];
        ion_norm += nav.ionosphere_model.beta[i] * nav.ionosphere_model.beta[i];
    }
    const double* alpha =
        (nav.ionosphere_model.valid && ion_norm > 0.0)
            ? nav.ionosphere_model.alpha
            : kDefaultAlpha;
    const double* beta =
        (nav.ionosphere_model.valid && ion_norm > 0.0)
            ? nav.ionosphere_model.beta
            : kDefaultBeta;

    const double psi = 0.0137 / (elevation / M_PI + 0.11) - 0.022;
    double phi = lat / M_PI + psi * std::cos(azimuth);
    phi = std::clamp(phi, -0.416, 0.416);
    const double lam = lon / M_PI +
                       psi * std::sin(azimuth) / std::cos(phi * M_PI);
    phi += 0.064 * std::cos((lam - 1.617) * M_PI);

    double local_time = 43200.0 * lam + time.tow;
    local_time -= std::floor(local_time / 86400.0) * 86400.0;
    const double slant =
        1.0 + 16.0 * std::pow(0.53 - elevation / M_PI, 3.0);
    double amp =
        alpha[0] + phi * (alpha[1] + phi * (alpha[2] + phi * alpha[3]));
    double per = beta[0] + phi * (beta[1] + phi * (beta[2] + phi * beta[3]));
    amp = std::max(0.0, amp);
    per = std::max(72000.0, per);
    const double x = 2.0 * M_PI * (local_time - 50400.0) / per;
    const double delay_s =
        slant * (std::abs(x) < 1.57
                     ? 5e-9 + amp * (1.0 + x * x * (-0.5 + x * x / 24.0))
                     : 5e-9);
    return constants::SPEED_OF_LIGHT * delay_s;
}

double claslibBroadcastVariance(const Ephemeris& eph) {
    static constexpr double kGpsUra[] = {
        2.4, 3.4, 4.85, 6.85, 9.65, 13.65, 24.0, 48.0,
        96.0, 192.0, 384.0, 768.0, 1536.0, 3072.0, 6144.0};
    if (eph.satellite.system == GNSSSystem::Galileo) {
        const int sva = static_cast<int>(std::lround(eph.sv_accuracy));
        double temp = 6144.0 * 100.0;
        if (sva >= 0 && sva <= 49) {
            temp = static_cast<double>(sva);
        } else if (sva >= 50 && sva <= 74) {
            temp = static_cast<double>((sva - 50) * 2 + 50);
        } else if (sva >= 75 && sva <= 99) {
            temp = static_cast<double>((sva - 75) * 4 + 100);
        } else if (sva >= 100 && sva <= 125) {
            temp = static_cast<double>((sva - 100) * 16 + 200);
        }
        const double sigma = temp / 100.0;
        return sigma * sigma;
    }
    int ura_index = 15;
    for (int i = 0; i < 15; ++i) {
        if (kGpsUra[i] >= eph.sv_accuracy) {
            ura_index = i;
            break;
        }
    }
    if (ura_index >= 15) {
        return 6144.0 * 6144.0;
    }
    const double sigma = kGpsUra[ura_index];
    return sigma * sigma;
}

double claslibSnrThreshold(double elevation, int freq_index) {
    static constexpr double kMask[3][9] = {
        {10.0, 10.0, 10.0, 10.0, 30.0, 30.0, 30.0, 30.0, 30.0},
        {10.0, 10.0, 10.0, 10.0, 30.0, 30.0, 30.0, 30.0, 30.0},
        {10.0, 10.0, 10.0, 10.0, 30.0, 30.0, 30.0, 30.0, 30.0},
    };
    if (freq_index < 0 || freq_index >= 3) {
        return 0.0;
    }
    double a = (elevation * 180.0 / M_PI + 5.0) / 10.0;
    int i = static_cast<int>(std::floor(a));
    a -= static_cast<double>(i);
    if (i < 1) {
        return kMask[freq_index][0];
    }
    if (i > 8) {
        return kMask[freq_index][8];
    }
    return (1.0 - a) * kMask[freq_index][i - 1] + a * kMask[freq_index][i];
}

bool claslibSnrMasked(const Observation& observation,
                      double elevation,
                      int freq_index,
                      int iter) {
    if (iter <= 0) {
        return false;
    }
    return observation.snr < claslibSnrThreshold(elevation, freq_index);
}

bool selectClaslibPntposSignals(GNSSSystem system,
                                SignalType& primary,
                                SignalType& secondary) {
    switch (system) {
        case GNSSSystem::GPS:
            primary = SignalType::GPS_L1CA;
            secondary = SignalType::GPS_L2C;
            return true;
        case GNSSSystem::QZSS:
            primary = SignalType::QZS_L1CA;
            secondary = SignalType::QZS_L2C;
            return true;
        case GNSSSystem::Galileo:
            primary = SignalType::GAL_E1;
            secondary = SignalType::GAL_E5A;
            return true;
        default:
            return false;
    }
}

bool calculateClaslibBroadcastState(const Ephemeris& eph,
                                    const GNSSTime& time,
                                    Vector3d& position,
                                    double& clock_s) {
    if (!eph.valid || eph.sqrt_a <= 0.0) {
        return false;
    }

    constexpr double kMuGps = 3.9860050e14;
    constexpr double kMuGal = 3.986004418e14;
    constexpr double kMuBds = 3.986004418e14;
    constexpr double kOmgeGps = 7.2921151467e-5;
    constexpr double kOmgeGal = 7.2921151467e-5;
    constexpr double kOmgeBds = 7.292115e-5;
    constexpr double kSinMinusFiveDeg = -0.0871557427476582;
    constexpr double kCosMinusFiveDeg = 0.9961946980917456;

    const double a = eph.sqrt_a * eph.sqrt_a;
    const double tk = time - eph.toe;
    double mu = kMuGps;
    double omge = kOmgeGps;
    if (eph.satellite.system == GNSSSystem::Galileo) {
        mu = kMuGal;
        omge = kOmgeGal;
    } else if (eph.satellite.system == GNSSSystem::BeiDou) {
        mu = kMuBds;
        omge = kOmgeBds;
    }

    const double mean_anomaly =
        eph.m0 + (std::sqrt(mu / (a * a * a)) + eph.delta_n) * tk;
    double eccentric_anomaly = mean_anomaly;
    for (int i = 0; i < 30; ++i) {
        const double previous = eccentric_anomaly;
        eccentric_anomaly -=
            (eccentric_anomaly - eph.e * std::sin(eccentric_anomaly) -
             mean_anomaly) /
            (1.0 - eph.e * std::cos(eccentric_anomaly));
        if (std::abs(eccentric_anomaly - previous) <= 1e-14) {
            break;
        }
    }

    const double sin_e = std::sin(eccentric_anomaly);
    const double cos_e = std::cos(eccentric_anomaly);
    double u = std::atan2(std::sqrt(1.0 - eph.e * eph.e) * sin_e,
                          cos_e - eph.e) +
               eph.omega;
    double r = a * (1.0 - eph.e * cos_e);
    double inclination = eph.i0 + eph.idot * tk;
    const double sin_2u = std::sin(2.0 * u);
    const double cos_2u = std::cos(2.0 * u);
    u += eph.cus * sin_2u + eph.cuc * cos_2u;
    r += eph.crs * sin_2u + eph.crc * cos_2u;
    inclination += eph.cis * sin_2u + eph.cic * cos_2u;

    const double x_orb = r * std::cos(u);
    const double y_orb = r * std::sin(u);
    const double cos_i = std::cos(inclination);
    const double toes = eph.toes != 0.0 ? eph.toes : eph.toe.tow;
    const double omega =
        eph.omega0 + (eph.omega_dot - omge) * tk - omge * toes;
    const double sin_omega = std::sin(omega);
    const double cos_omega = std::cos(omega);

    if (eph.satellite.system == GNSSSystem::BeiDou && eph.satellite.prn <= 5) {
        const double xg = x_orb * cos_omega - y_orb * cos_i * sin_omega;
        const double yg = x_orb * sin_omega + y_orb * cos_i * cos_omega;
        const double zg = y_orb * std::sin(inclination);
        const double sin_rot = std::sin(omge * tk);
        const double cos_rot = std::cos(omge * tk);
        position.x() = xg * cos_rot + yg * sin_rot * kCosMinusFiveDeg +
                       zg * sin_rot * kSinMinusFiveDeg;
        position.y() = -xg * sin_rot + yg * cos_rot * kCosMinusFiveDeg +
                       zg * cos_rot * kSinMinusFiveDeg;
        position.z() = -yg * kSinMinusFiveDeg + zg * kCosMinusFiveDeg;
    } else {
        position.x() = x_orb * cos_omega - y_orb * cos_i * sin_omega;
        position.y() = x_orb * sin_omega + y_orb * cos_i * cos_omega;
        position.z() = y_orb * std::sin(inclination);
    }

    const double tc = time - eph.toc;
    clock_s = eph.af0 + eph.af1 * tc + eph.af2 * tc * tc;
    clock_s -=
        2.0 * std::sqrt(mu * a) * eph.e * sin_e /
        (constants::SPEED_OF_LIGHT * constants::SPEED_OF_LIGHT);
    return true;
}

struct ClaslibPntposMeasurement {
    SatelliteId satellite;
    SignalType primary_signal = SignalType::SIGNAL_TYPE_COUNT;
    SignalType secondary_signal = SignalType::SIGNAL_TYPE_COUNT;
    double pseudorange_if = 0.0;
    double primary_pseudorange = 0.0;
    Vector3d sat_position = Vector3d::Zero();
    double sat_clock_s = 0.0;
    double sat_variance = 0.0;
    double elevation = 0.0;
    double azimuth = 0.0;
    double residual = 0.0;
};

bool calculateClaslibPntposSatelliteState(const NavigationData& nav,
                                          const GNSSTime& time,
                                          const SatelliteId& satellite,
                                          double primary_pseudorange,
                                          Vector3d& sat_position,
                                          double& sat_clock_s,
                                          double& sat_variance) {
    const Ephemeris* eph = nav.getEphemeris(satellite, time);
    if (eph == nullptr || !eph->valid || eph->health != 0) {
        return false;
    }
    GNSSTime tx_time = time - primary_pseudorange / constants::SPEED_OF_LIGHT;
    const double tk_clock = tx_time - eph->toc;
    const double preliminary_clock =
        eph->af0 + eph->af1 * tk_clock + eph->af2 * tk_clock * tk_clock;
    tx_time = tx_time - preliminary_clock;
    if (!calculateClaslibBroadcastState(*eph, tx_time, sat_position, sat_clock_s)) {
        return false;
    }
    sat_variance = claslibBroadcastVariance(*eph);
    return true;
}

std::vector<ClaslibPntposMeasurement> buildClaslibPntposMeasurements(
    const ObservationData& obs,
    const NavigationData& nav,
    const Vector3d& receiver_position,
    const std::set<SatelliteId>& excluded_satellites,
    int iter) {
    std::vector<ClaslibPntposMeasurement> measurements;
    constexpr double kElevationMask = 15.0 * M_PI / 180.0;

    for (const SatelliteId& satellite : obs.getSatellites()) {
        if (excluded_satellites.find(satellite) != excluded_satellites.end()) {
            continue;
        }
        SignalType primary_signal = SignalType::SIGNAL_TYPE_COUNT;
        SignalType secondary_signal = SignalType::SIGNAL_TYPE_COUNT;
        if (!selectClaslibPntposSignals(
                satellite.system, primary_signal, secondary_signal)) {
            continue;
        }
        const Observation* primary_obs =
            obs.getObservation(satellite, primary_signal);
        const Observation* secondary_obs =
            obs.getObservation(satellite, secondary_signal);
        if (primary_obs == nullptr || secondary_obs == nullptr) {
            continue;
        }
        if (!primary_obs->valid || !secondary_obs->valid ||
            !primary_obs->has_pseudorange || !secondary_obs->has_pseudorange ||
            primary_obs->pseudorange <= 0.0 ||
            secondary_obs->pseudorange <= 0.0) {
            continue;
        }

        const double f1 = signalFrequencyHz(primary_signal);
        const double f2 = signalFrequencyHz(secondary_signal);
        if (f1 <= 0.0 || f2 <= 0.0 || std::abs(f1 - f2) < 1.0) {
            continue;
        }
        const double gamma = (f1 * f1) / (f2 * f2);
        if (std::abs(gamma - 1.0) < 1e-12) {
            continue;
        }
        const double pseudorange_if =
            (gamma * primary_obs->pseudorange -
             secondary_obs->pseudorange) /
            (gamma - 1.0);

        Vector3d sat_position;
        double sat_clock_s = 0.0;
        double sat_variance = 0.0;
        if (!calculateClaslibPntposSatelliteState(
                nav,
                obs.time,
                satellite,
                primary_obs->pseudorange,
                sat_position,
                sat_clock_s,
                sat_variance)) {
            continue;
        }

        const auto geometry = nav.calculateGeometry(receiver_position, sat_position);
        if (geometry.elevation < kElevationMask) {
            continue;
        }
        if (claslibSnrMasked(*primary_obs, geometry.elevation, 0, iter) ||
            claslibSnrMasked(*secondary_obs, geometry.elevation, 1, iter)) {
            continue;
        }

        ClaslibPntposMeasurement measurement;
        measurement.satellite = satellite;
        measurement.primary_signal = primary_signal;
        measurement.secondary_signal = secondary_signal;
        measurement.primary_pseudorange = primary_obs->pseudorange;
        measurement.pseudorange_if = pseudorange_if;
        measurement.sat_position = sat_position;
        measurement.sat_clock_s = sat_clock_s;
        measurement.sat_variance = sat_variance;
        measurement.elevation = geometry.elevation;
        measurement.azimuth = geometry.azimuth;
        measurements.push_back(measurement);
    }
    return measurements;
}

bool solveClaslibPntposSeed(const ObservationData& obs,
                            const NavigationData& nav,
                            const Vector3d& initial_position,
                            PositionSolution& solution) {
    constexpr int kNx = 7;
    constexpr int kMaxIterations = 10;
    constexpr double kErr0 = 100.0;
    constexpr double kErrPhase = 0.010;
    constexpr double kErrPhaseEl = 0.005;
    constexpr double kCodeBiasVariance = 0.3 * 0.3;
    constexpr double kResidualRejectThreshold = 18.0;

    VectorXd x = VectorXd::Zero(kNx);
    x.segment(0, 3) = initial_position;

    std::vector<ClaslibPntposMeasurement> final_measurements;
    std::set<SatelliteId> excluded_satellites;
    std::map<SatelliteId, double> previous_residuals;
    int iterations = 0;

    for (int iter = 0; iter < kMaxIterations; ++iter) {
        if (iter > 0 && static_cast<int>(previous_residuals.size()) > 7) {
            double sum_sq = 0.0;
            for (const auto& [satellite, residual] : previous_residuals) {
                (void)satellite;
                sum_sq += residual * residual;
            }
            const double mean_sq =
                sum_sq / static_cast<double>(previous_residuals.size());
            if (mean_sq <= 20.0 * 20.0) {
                for (const auto& [satellite, residual] : previous_residuals) {
                    if (std::abs(residual) > kResidualRejectThreshold) {
                        excluded_satellites.insert(satellite);
                    }
                }
            }
        }
        auto measurements =
            buildClaslibPntposMeasurements(
                obs, nav, x.segment(0, 3), excluded_satellites, iter);
        if (measurements.empty()) {
            return false;
        }

        bool mask[4] = {false, false, false, false};
        for (const auto& measurement : measurements) {
            switch (claslibPntposClockMaskGroup(measurement.satellite.system)) {
                case GNSSSystem::GLONASS:
                    mask[1] = true;
                    break;
                case GNSSSystem::Galileo:
                    mask[2] = true;
                    break;
                case GNSSSystem::BeiDou:
                    mask[3] = true;
                    break;
                default:
                    mask[0] = true;
                    break;
            }
        }

        const int constraint_count =
            static_cast<int>(std::count(mask, mask + 4, false));
        const int nv = static_cast<int>(measurements.size()) + constraint_count;
        if (nv < kNx) {
            return false;
        }

        MatrixXd h = MatrixXd::Zero(nv, kNx);
        VectorXd v = VectorXd::Zero(nv);
        int row = 0;
        for (auto& measurement : measurements) {
            const Vector3d los =
                (measurement.sat_position - x.segment(0, 3)).normalized();
            const double range = geodist(measurement.sat_position, x.segment(0, 3));
            const double iono =
                iter == 0 ? claslibBroadcastIonoDelay(
                                nav,
                                x.segment(0, 3),
                                measurement.azimuth,
                                measurement.elevation,
                                obs.time)
                          : 0.0;
            const double trop =
                models::tropDelaySaastamoinen(x.segment(0, 3), measurement.elevation);
            double residual =
                measurement.pseudorange_if -
                (range + x(3) -
                 constants::SPEED_OF_LIGHT * measurement.sat_clock_s +
                 iono + trop);

            h(row, 0) = -los.x();
            h(row, 1) = -los.y();
            h(row, 2) = -los.z();
            h(row, 3) = 1.0;

            const int bias_col = claslibPntposBiasColumn(
                claslibPntposClockMaskGroup(measurement.satellite.system));
            if (bias_col >= 0) {
                residual -= x(bias_col);
                h(row, bias_col) = 1.0;
            }
            measurement.residual = residual;

            const double sin_el = std::max(std::sin(measurement.elevation), 0.05);
            const double varerr =
                kErr0 * kErr0 *
                (kErrPhase * kErrPhase + kErrPhaseEl * kErrPhaseEl / sin_el) *
                9.0;
            const double trop_variance =
                (0.3 / (sin_el + 0.1)) * (0.3 / (sin_el + 0.1));
            const double variance =
                std::max(1e-6,
                         varerr + measurement.sat_variance +
                             kCodeBiasVariance + trop_variance);
            const double sigma = std::sqrt(variance);
            v(row) = residual / sigma;
            h.row(row) /= sigma;
            ++row;
        }

        for (int i = 0; i < 4; ++i) {
            if (mask[i]) {
                continue;
            }
            h(row, i + 3) = 1.0 / 0.1;
            v(row) = 0.0;
            ++row;
        }

        const MatrixXd normal_matrix = h.transpose() * h;
        const VectorXd normal_rhs = h.transpose() * v;
        Eigen::FullPivLU<MatrixXd> lu(normal_matrix);
        if (!lu.isInvertible()) {
            return false;
        }
        const VectorXd dx = lu.solve(normal_rhs);
        if (!dx.allFinite()) {
            return false;
        }
        x += dx;
        iterations = iter + 1;
        previous_residuals.clear();
        for (const auto& measurement : measurements) {
            previous_residuals[measurement.satellite] = measurement.residual;
        }
        if (pppDebugEnabled() &&
            obs.time.tow >= 230420.0 - 1e-6 &&
            obs.time.tow <= 230420.0 + 1e-6) {
            std::cerr << std::setprecision(12)
                      << "[CLAS-SPP-ITER] source=lib"
                      << " tow=" << obs.time.tow
                      << " iter=" << iter
                      << " ns=" << measurements.size()
                      << " dx_norm=" << dx.norm()
                      << " residuals=";
            for (size_t i = 0; i < measurements.size(); ++i) {
                if (i > 0) {
                    std::cerr << ",";
                }
                std::cerr << measurements[i].satellite.toString()
                          << ":" << measurements[i].residual;
            }
            std::cerr << "\n";
        }
        final_measurements = std::move(measurements);
        if (dx.norm() < 1e-4) {
            break;
        }
    }

    final_measurements =
        buildClaslibPntposMeasurements(
            obs, nav, x.segment(0, 3), excluded_satellites, 1);
    if (final_measurements.size() < 4) {
        return false;
    }

    solution = PositionSolution{};
    solution.time = obs.time;
    solution.status = SolutionStatus::SPP;
    solution.position_ecef = x.segment(0, 3);
    double lat = 0.0;
    double lon = 0.0;
    double height = 0.0;
    ecef2geodetic(solution.position_ecef, lat, lon, height);
    solution.position_geodetic = GeodeticCoord(lat, lon, height);
    solution.receiver_clock_bias = x(3);
    solution.num_satellites = static_cast<int>(final_measurements.size());
    solution.num_frequencies = 2;
    solution.iterations = iterations;

    VectorXd residuals = VectorXd::Zero(static_cast<int>(final_measurements.size()));
    std::vector<Vector3d> sat_positions;
    sat_positions.reserve(final_measurements.size());
    for (int i = 0; i < static_cast<int>(final_measurements.size()); ++i) {
        auto& measurement = final_measurements[i];
        const double range = geodist(measurement.sat_position, solution.position_ecef);
        const double trop = models::tropDelaySaastamoinen(
            solution.position_ecef, measurement.elevation);
        double residual =
            measurement.pseudorange_if -
            (range + x(3) -
             constants::SPEED_OF_LIGHT * measurement.sat_clock_s + trop);
        const int bias_col = claslibPntposBiasColumn(
            claslibPntposClockMaskGroup(measurement.satellite.system));
        if (bias_col >= 0) {
            residual -= x(bias_col);
        }
        residuals(i) = residual;
        solution.satellites_used.push_back(measurement.satellite);
        solution.satellite_elevations.push_back(measurement.elevation);
        solution.satellite_residuals.push_back(residual);
        sat_positions.push_back(measurement.sat_position);
    }
    solution.residual_rms =
        std::sqrt(residuals.squaredNorm() /
                  static_cast<double>(std::max<int>(1, residuals.size())));
    if (sat_positions.size() >= 4) {
        MatrixXd dop_h = MatrixXd::Zero(static_cast<int>(sat_positions.size()), 4);
        for (int i = 0; i < static_cast<int>(sat_positions.size()); ++i) {
            const Vector3d los =
                (sat_positions[i] - solution.position_ecef).normalized();
            dop_h(i, 0) = -los.x();
            dop_h(i, 1) = -los.y();
            dop_h(i, 2) = -los.z();
            dop_h(i, 3) = 1.0;
        }
        const MatrixXd q = (dop_h.transpose() * dop_h).inverse();
        solution.gdop = std::sqrt(q.trace());
        solution.pdop = std::sqrt(q(0, 0) + q(1, 1) + q(2, 2));
        solution.hdop = std::sqrt(q(0, 0) + q(1, 1));
        solution.vdop = std::sqrt(q(2, 2));
    }
    if (pppDebugEnabled() &&
        obs.time.tow >= 230420.0 - 1e-6 &&
        obs.time.tow <= 230425.0 + 1e-6) {
        std::cerr << std::setprecision(15)
                  << "[CLAS-SPP-SEED] source=lib"
                  << " week=" << obs.time.week
                  << " tow=" << obs.time.tow
                  << " ns=" << solution.num_satellites
                  << " iter=" << solution.iterations
                  << " clk_m=" << solution.receiver_clock_bias
                  << " rms=" << solution.residual_rms
                  << " gal_bias_m=" << x(5)
                  << " sats=";
        for (size_t i = 0; i < solution.satellites_used.size(); ++i) {
            if (i > 0) {
                std::cerr << ",";
            }
            std::cerr << solution.satellites_used[i].toString()
                      << ":" << solution.satellite_residuals[i];
        }
        std::cerr << "\n";
    }
    return solution.isValid();
}

void dumpReceiverPositionTrace(const char* stage,
                               const GNSSTime& time,
                               const ObservationData& obs,
                               const PPPConfig& config,
                               const PositionSolution* seed_solution,
                               const PPPState* filter_state) {
    if (!shouldDumpReceiverPositionTrace(config, time)) {
        return;
    }

    const double nan = std::numeric_limits<double>::quiet_NaN();
    Vector3d seed_position(nan, nan, nan);
    double seed_clock_m = nan;
    if (seed_solution != nullptr && seed_solution->isValid()) {
        seed_position = seed_solution->position_ecef;
        seed_clock_m = seed_solution->receiver_clock_bias;
    }

    Vector3d filter_position(nan, nan, nan);
    double filter_clock_m = nan;
    double filter_trop_m = nan;
    if (filter_state != nullptr &&
        filter_state->state.size() >= filter_state->pos_index + 3) {
        filter_position =
            filter_state->state.segment(filter_state->pos_index, 3);
        if (filter_state->clock_index >= 0 &&
            filter_state->clock_index < filter_state->state.size()) {
            filter_clock_m = filter_state->state(filter_state->clock_index);
        }
        if (filter_state->trop_index >= 0 &&
            filter_state->trop_index < filter_state->state.size()) {
            filter_trop_m = filter_state->state(filter_state->trop_index);
        }
    }

    std::cerr << std::setprecision(15)
              << "[CLAS-RCV-POS] source=lib"
              << " week=" << time.week
              << " tow=" << time.tow
              << " stage=" << (stage != nullptr ? stage : "")
              << " spp_x=" << seed_position.x()
              << " spp_y=" << seed_position.y()
              << " spp_z=" << seed_position.z()
              << " spp_clk_m=" << seed_clock_m
              << " filter_x=" << filter_position.x()
              << " filter_y=" << filter_position.y()
              << " filter_z=" << filter_position.z()
              << " filter_clk_m=" << filter_clock_m
              << " filter_trop_m=" << filter_trop_m
              << " obs_x=" << obs.receiver_position.x()
              << " obs_y=" << obs.receiver_position.y()
              << " obs_z=" << obs.receiver_position.z()
              << " ref_x=" << config.approximate_position.x()
              << " ref_y=" << config.approximate_position.y()
              << " ref_z=" << config.approximate_position.z()
              << "\n";
}

}  // namespace

PositionSolution PPPProcessor::processEpochCLAS(const ObservationData& obs,
                                                 const NavigationData& nav) {
    PositionSolution solution;
    solution.time = obs.time;
    solution.status = SolutionStatus::NONE;
    last_clas_hybrid_fallback_used_ = false;
    last_clas_hybrid_fallback_reason_.clear();
    last_ar_ratio_ = 0.0;
    last_fixed_ambiguities_ = 0;
    last_clas_ar_phase_ambiguities_.clear();
    // CLAS path: apply solid earth tide as an observation-space correction
    // (projected onto each satellite's LOS) instead of modifying the receiver
    // position.  This avoids the position perturbation that degrades AR.
    // NOTE: solid earth tide as OSR was tested but did NOT improve accuracy.
    // The filter absorbs the constant tide into clock/position over short windows.
    // Both sign conventions were tested: neither beat the no-tide baseline.
    // Keeping the infrastructure for future use when float precision improves.
    // if (ppp_config_.apply_solid_earth_tides && !ppp_config_.apply_tide_as_osr) {
    //     ppp_config_.apply_tide_as_osr = true;
    //     ppp_config_.apply_solid_earth_tides = false;
    // }

    // CLAS per-frequency mode defaults to WL-NL AR, but do not override an
    // explicit alternative such as DD_PER_FREQ when the caller is running a
    // parity experiment against CLASLIB's ddmat/resamb_LAMBDA path.
    if (ppp_config_.enable_ambiguity_resolution &&
        !ppp_config_.use_ionosphere_free &&
        ppp_config_.ar_method == PPPConfig::ARMethod::DD_WLNL) {
        // CLASLIB attempts WL fixing without a hard multi-epoch warmup.
        // Let the DD-WL/NL ratio test decide from the first epoch onward.
        if (ppp_config_.wl_min_averaging_epochs > 1) {
            ppp_config_.wl_min_averaging_epochs = 1;
        }
    }

    struct ClasFallbackSnapshot {
        PPPState filter_state;
        bool filter_initialized = false;
        GNSSTime convergence_start_time;
        Vector3d static_anchor_position = Vector3d::Zero();
        bool has_static_anchor_position = false;
        std::map<SatelliteId, PPPAmbiguityInfo> ambiguity_states;
        std::map<SatelliteId, CLASDispersionCompensationInfo> dispersion_compensation;
        std::map<SatelliteId, CLASSisContinuityInfo> sis_continuity;
        std::map<SatelliteId, double> windup_cache;
        std::map<SatelliteId, CLASPhaseBiasRepairInfo> phase_bias_repair;
        bool has_last_processed_time = false;
        GNSSTime last_processed_time;
    };
    const ClasFallbackSnapshot fallback_snapshot{
        filter_state_,
        filter_initialized_,
        convergence_start_time_,
        static_anchor_position_,
        has_static_anchor_position_,
        ambiguity_states_,
        clas_dispersion_compensation_,
        clas_sis_continuity_,
        windup_cache_,
        clas_phase_bias_repair_,
        has_last_processed_time_,
        last_processed_time_,
    };
    const auto restore_clas_snapshot = [&]() {
        filter_state_ = fallback_snapshot.filter_state;
        filter_initialized_ = fallback_snapshot.filter_initialized;
        convergence_start_time_ = fallback_snapshot.convergence_start_time;
        static_anchor_position_ = fallback_snapshot.static_anchor_position;
        has_static_anchor_position_ = fallback_snapshot.has_static_anchor_position;
        ambiguity_states_ = fallback_snapshot.ambiguity_states;
        clas_dispersion_compensation_ = fallback_snapshot.dispersion_compensation;
        clas_sis_continuity_ = fallback_snapshot.sis_continuity;
        windup_cache_ = fallback_snapshot.windup_cache;
        clas_phase_bias_repair_ = fallback_snapshot.phase_bias_repair;
        has_last_processed_time_ = fallback_snapshot.has_last_processed_time;
        last_processed_time_ = fallback_snapshot.last_processed_time;
    };
    const bool allow_hybrid_fallback =
        ppp_config_.clas_epoch_policy ==
        PPPConfig::ClasEpochPolicy::HYBRID_STANDARD_PPP_FALLBACK;
    const auto fallback_to_standard = [&](const char* reason) {
        restore_clas_snapshot();
        return processEpochStandard(obs, nav, reason);
    };

    PositionSolution seed = spp_processor_.processEpoch(obs, nav);
    if (ppp_config_.wlnl_strict_claslib_parity && !filter_initialized_) {
        PositionSolution claslib_seed;
        const bool seed_ok = solveClaslibPntposSeed(
            obs, nav, Vector3d::Zero(), claslib_seed);
        if (seed_ok) {
            seed = claslib_seed;
        } else if (pppDebugEnabled()) {
            std::cerr << "[CLAS-RCV-POS] source=lib week=" << obs.time.week
                      << " tow=" << obs.time.tow
                      << " stage=claslib_spp_seed_failed\n";
        }
    }
    dumpReceiverPositionTrace(
        "spp_output", obs.time, obs, ppp_config_, &seed,
        filter_initialized_ ? &filter_state_ : nullptr);
    detectCycleSlips(obs);
    const double ambiguity_reset_variance =
        ppp_config_.initial_ambiguity_variance <
                ppp_config_.clas_ambiguity_reinit_threshold * 0.1
            ? ppp_config_.initial_ambiguity_variance
            : ppp_config_.clas_ambiguity_reinit_threshold * 0.1;
    const bool was_filter_initialized = filter_initialized_;
    const auto epoch_preparation = ppp_clas::prepareEpochState(
        obs,
        seed,
        ssr_products_,
        filter_state_,
        filter_initialized_,
        convergence_start_time_,
        static_anchor_position_,
        has_static_anchor_position_,
        ppp_config_,
        modeledZenithTroposphereDelayMeters(seed.position_ecef, obs.time),
        has_last_processed_time_,
        last_processed_time_,
        ambiguity_states_,
        clas_dispersion_compensation_,
        clas_phase_bias_repair_,
        ambiguity_reset_variance);
    if (!was_filter_initialized && filter_initialized_) {
        dumpReceiverPositionTrace(
            "filter_init", obs.time, obs, ppp_config_, &seed, &filter_state_);
    }
    if (!epoch_preparation.ready) {
        if (pppDebugEnabled()) {
            std::cerr << "[CLAS-EPOCH] tow=" << obs.time.tow
                      << " stage=prepare_epoch_state ready=0\n";
        }
        if (allow_hybrid_fallback) {
            return fallback_to_standard("prepare_epoch_state");
        }
        return solution;
    }
    dumpReceiverPositionTrace(
        "pre_update", obs.time, obs, ppp_config_, &seed, &filter_state_);

    const PPPConfig clas_epoch_config = effectiveClasAtmosConfig();

    if (pppDebugEnabled() &&
        ppp_config_.wlnl_strict_claslib_parity &&
        obs.time.tow >= 230420.0 - 1e-6 &&
        obs.time.tow <= 230425.0 + 1e-6) {
        std::cerr << std::setprecision(15)
                  << "[CLAS-RECV-PRE] tow=" << obs.time.tow
                  << " pos_x=" << filter_state_.state(0)
                  << " pos_y=" << filter_state_.state(1)
                  << " pos_z=" << filter_state_.state(2)
                  << " seed_pos_x=" << seed.position_ecef.x()
                  << " seed_pos_y=" << seed.position_ecef.y()
                  << " seed_pos_z=" << seed.position_ecef.z()
                  << " obs_pos_x=" << obs.receiver_position.x()
                  << " obs_pos_y=" << obs.receiver_position.y()
                  << " obs_pos_z=" << obs.receiver_position.z()
                  << " clk="
                  << (filter_state_.clock_index >= 0
                          ? filter_state_.state(filter_state_.clock_index)
                          : 0.0)
                  << " seed_clk=" << seed.receiver_clock_bias
                  << " trop="
                  << (filter_state_.trop_index >= 0
                          ? filter_state_.state(filter_state_.trop_index)
                          : 0.0)
                  << "\n";
    }

    const auto epoch_context = prepareClasEpochContext(
        obs,
        nav,
        ssr_products_,
        filter_state_.state.segment(0, 3),
        filter_state_.state(filter_state_.clock_index),
        filter_state_.state(filter_state_.trop_index),
        clas_epoch_config,
        windup_cache_,
        clas_dispersion_compensation_,
        clas_sis_continuity_,
        clas_phase_bias_repair_);
    const auto& epoch_atmos = epoch_context.epoch_atmos_tokens;
    const auto& osr_corrections = epoch_context.osr_corrections;

    if (osr_corrections.size() < 4) {
        if (pppDebugEnabled()) {
            std::cerr << "[CLAS-EPOCH] tow=" << obs.time.tow
                      << " stage=osr_count count=" << osr_corrections.size()
                      << " ready=0\n";
        }
        if (allow_hybrid_fallback) {
            return fallback_to_standard("insufficient_osr");
        }
        solution = seed;
        return solution;
    }

    ppp_clas::ensureAmbiguityStates(filter_state_, osr_corrections, ppp_config_);
    ppp_clas::applyPendingPhaseBiasStateShifts(
        filter_state_, osr_corrections, clas_phase_bias_repair_, pppDebugEnabled());

    const auto epoch_update = ppp_clas::runEpochMeasurementUpdate(
        obs,
        epoch_context,
        filter_state_,
        ppp_config_,
        seed,
        ambiguity_states_,
        [&](const Vector3d& receiver_pos, double elevation, const GNSSTime& time) {
            return calculateMappingFunction(receiver_pos, elevation, time);
        },
        [&](const SatelliteId& satellite, SignalType signal) {
            resetAmbiguity(satellite, signal);
        },
        [&](const SatelliteId& satellite) {
            return ambiguityStateIndex(satellite);
        },
        pppDebugEnabled());
    if (pppDebugEnabled()) {
        std::cerr << "[CLAS-EPOCH] tow=" << obs.time.tow
                  << " stage=measurement_update updated="
                  << (epoch_update.updated ? 1 : 0)
                  << "\n";
    }
    if (!epoch_update.updated) {
        if (allow_hybrid_fallback) {
            return fallback_to_standard("measurement_update");
        }
        solution = seed;
        return solution;
    }
    const auto& update_stats = epoch_update.update_stats;
    last_clas_ar_phase_ambiguities_ = epoch_update.ar_phase_ambiguities;
    pre_anchor_covariance_ = update_stats.pre_anchor_covariance;

    // Accumulate Melbourne-Wübbena for WL-NL AR in CLAS per-frequency mode.
    if (ppp_config_.enable_ambiguity_resolution &&
        ppp_config_.ar_method == PPPConfig::ARMethod::DD_WLNL) {
        for (const auto& osr : osr_corrections) {
            if (!osr.valid || osr.num_frequencies < 2) continue;
            const Observation* l1_raw = obs.getObservation(osr.satellite, osr.signals[0]);
            const Observation* l2_raw = obs.getObservation(osr.satellite, osr.signals[1]);
            if (!l1_raw || !l2_raw || !l1_raw->valid || !l2_raw->valid) continue;
            if (!l1_raw->has_carrier_phase || !l2_raw->has_carrier_phase) continue;
            if (!l1_raw->has_pseudorange || !l2_raw->has_pseudorange) continue;
            const double f1 = osr.frequencies[0];
            const double f2 = osr.frequencies[1];
            if (f1 <= 0.0 || f2 <= 0.0 || std::abs(f1 - f2) < 1e6) continue;
            const double l1_m = l1_raw->carrier_phase * osr.wavelengths[0]
                              - osr.phase_bias_m[0];
            const double l2_m = l2_raw->carrier_phase * osr.wavelengths[1]
                              - osr.phase_bias_m[1];
            const double p1 = l1_raw->pseudorange - osr.code_bias_m[0];
            const double p2 = l2_raw->pseudorange - osr.code_bias_m[1];
            const double mw_m = (f1 * l1_m - f2 * l2_m) / (f1 - f2)
                              - (f1 * p1 + f2 * p2) / (f1 + f2);
            const double lambda_wl = constants::SPEED_OF_LIGHT / std::abs(f1 - f2);
            if (lambda_wl <= 0.0 || !std::isfinite(lambda_wl)) {
                continue;
            }
            const double mw_cycles = mw_m / lambda_wl;
            auto& amb = ambiguity_states_[osr.satellite];
            const bool mw_slip = l1_raw->loss_of_lock || l2_raw->loss_of_lock;
            if (!mw_slip && amb.mw_count > 0) {
                amb.mw_sum_cycles += mw_cycles;
                amb.mw_count += 1;
                amb.mw_mean_cycles = amb.mw_sum_cycles / amb.mw_count;
            } else {
                amb.mw_sum_cycles = mw_cycles;
                amb.mw_count = 1;
                amb.mw_mean_cycles = mw_cycles;
                amb.wl_is_fixed = false;
            }
        }
    }

    const auto trop_mapping_for_validation =
        [&](const Vector3d& receiver_pos, double elevation, const GNSSTime& time) {
            return calculateMappingFunction(receiver_pos, elevation, time);
        };
    const auto ambiguity_index_for_validation = [&](const SatelliteId& satellite) {
        return ambiguityStateIndex(satellite);
    };

    const auto ambiguity_resolution =
        ppp_clas::resolveAndValidateAmbiguities(
            filter_state_,
            ambiguity_states_,
            [&]() {
                return ppp_config_.enable_ambiguity_resolution &&
                       resolveAmbiguities(obs, nav);
            },
            (ppp_config_.ar_method == PPPConfig::ARMethod::DD_WLNL)
                ? ppp_clas::ValidateFixedSolutionFunction{}
                : ppp_clas::ValidateFixedSolutionFunction{[&]() {
                      return ppp_clas::validateFixedSolution(
                          obs, osr_corrections, filter_state_, ppp_config_,
                          trop_mapping_for_validation,
                          ambiguity_index_for_validation, pppDebugEnabled());
                  }},
            ppp_config_.wlnl_strict_claslib_parity &&
                ppp_config_.ar_method == PPPConfig::ARMethod::DD_PER_FREQ,
            pppDebugEnabled());
    if (ambiguity_resolution.rejected_after_fix) {
        last_ar_ratio_ = 0.0;
        last_fixed_ambiguities_ = 0;
        has_clas_dd_hold_state_ = false;
    } else if (ambiguity_resolution.accepted &&
               has_clas_dd_hold_state_ &&
               ppp_config_.wlnl_strict_claslib_parity &&
               ppp_config_.ar_method == PPPConfig::ARMethod::DD_PER_FREQ) {
        filter_state_ = clas_dd_hold_state_;
        has_clas_dd_hold_state_ = false;
    }

    if (pppDebugEnabled()) {
        ppp_clas::logUpdateSummary(update_stats, osr_corrections.size(), filter_state_);
    }
    bool wlnl_fixed_position_ok = false;
    Vector3d wlnl_fixed_position = Vector3d::Zero();
    if (ppp_config_.ar_method == PPPConfig::ARMethod::DD_WLNL) {
        const PPPState& base_state = filter_state_;
        auto& beta_state = clas_iflc_beta_state_;
        constexpr double kBetaClockAnchorVariance = 1.0;
        auto global_clock_index_for_system = [&](GNSSSystem system) {
            if (system == GNSSSystem::Galileo && base_state.gal_clock_index >= 0) {
                return base_state.gal_clock_index;
            }
            if (system == GNSSSystem::GLONASS && base_state.glo_clock_index >= 0) {
                return base_state.glo_clock_index;
            }
            return base_state.clock_index;
        };
        auto ensure_beta_size = [&](int new_size) {
            if (beta_state.state.size() >= new_size) {
                return;
            }
            const int old_size = beta_state.state.size();
            VectorXd expanded_state = VectorXd::Zero(new_size);
            MatrixXd expanded_covariance = MatrixXd::Zero(new_size, new_size);
            if (old_size > 0) {
                expanded_state.head(old_size) = beta_state.state;
                expanded_covariance.topLeftCorner(old_size, old_size) =
                    beta_state.covariance;
            }
            beta_state.state = std::move(expanded_state);
            beta_state.covariance = std::move(expanded_covariance);
        };
        if (!beta_state.initialized) {
            ensure_beta_size(3);
            beta_state.state.head(3) =
                base_state.state.segment(base_state.pos_index, 3);
            beta_state.covariance.topLeftCorner(3, 3) =
                MatrixXd::Identity(3, 3) *
                std::max(ppp_config_.clas_initial_position_variance, 1.0);
            beta_state.initialized = true;
        }
        if (beta_state.trop_column < 0) {
            const int trop_column = beta_state.state.size();
            ensure_beta_size(trop_column + 1);
            beta_state.trop_column = trop_column;
            beta_state.state(trop_column) = base_state.state(base_state.trop_index);
            beta_state.covariance(trop_column, trop_column) =
                std::max(ppp_config_.clas_trop_initial_variance, 1.0);
        } else {
            beta_state.covariance(beta_state.trop_column, beta_state.trop_column) =
                std::max(
                    beta_state.covariance(beta_state.trop_column, beta_state.trop_column) +
                        std::max(ppp_config_.clas_trop_process_noise, 1e-8),
                    1e-8);
        }

        struct LocalIfRow {
            Eigen::RowVectorXd H;
            double residual = 0.0;
            double variance = 0.0;
            bool is_phase = false;
            SatelliteId satellite{};
        };
        struct IfObservationSeed {
            const OSRCorrection* osr = nullptr;
            int clock_column = -1;
            int ambiguity_column = -1;
            int l1_ambiguity_index = -1;
            int l2_ambiguity_index = -1;
            double alpha1 = 0.0;
            double alpha2 = 0.0;
        };

        std::map<SatelliteId, IfObservationSeed> if_seeds;
        std::vector<LocalIfRow> local_rows;
        local_rows.reserve(osr_corrections.size() * 2);

        for (const auto& osr : osr_corrections) {
            if (!osr.valid || osr.num_frequencies < 2) {
                continue;
            }
            const Observation* l1 = obs.getObservation(osr.satellite, osr.signals[0]);
            const Observation* l2 = obs.getObservation(osr.satellite, osr.signals[1]);
            if (l1 == nullptr || l2 == nullptr ||
                !l1->valid || !l2->valid ||
                !l1->has_carrier_phase || !l2->has_carrier_phase ||
                !std::isfinite(l1->carrier_phase) || !std::isfinite(l2->carrier_phase) ||
                osr.wavelengths[0] <= 0.0 || osr.wavelengths[1] <= 0.0) {
                continue;
            }
            const double f1 = osr.frequencies[0];
            const double f2 = osr.frequencies[1];
            const double denom = f1 * f1 - f2 * f2;
            if (f1 <= 0.0 || f2 <= 0.0 || std::abs(denom) < 1.0) {
                continue;
            }
            const double alpha1 = (f1 * f1) / denom;
            const double alpha2 = -(f2 * f2) / denom;

            const int global_clock_index =
                global_clock_index_for_system(osr.satellite.system);
            auto clock_it = beta_state.clock_columns.find(osr.satellite.system);
            if (clock_it == beta_state.clock_columns.end()) {
                const int clock_column = beta_state.state.size();
                ensure_beta_size(clock_column + 1);
                beta_state.clock_columns.emplace(osr.satellite.system, clock_column);
                beta_state.state(clock_column) = base_state.state(global_clock_index);
                beta_state.covariance(clock_column, clock_column) =
                    kBetaClockAnchorVariance;
                clock_it = beta_state.clock_columns.find(osr.satellite.system);
            } else {
                // The IFLC beta clock is an auxiliary copy of the main filter clock.
                // Re-anchor it every epoch so stale drift cannot corrupt the local
                // relinearization used by WL/NL fixed-state reconstruction.
                beta_state.state(clock_it->second) = base_state.state(global_clock_index);
                beta_state.covariance(clock_it->second, clock_it->second) =
                    kBetaClockAnchorVariance;
            }

            const SatelliteId l1_amb_sat(osr.satellite.system, osr.satellite.prn);
            const SatelliteId l2_amb_sat(
                osr.satellite.system,
                static_cast<uint8_t>(std::min(255, osr.satellite.prn + 100)));
            const auto l1_amb_it = base_state.ambiguity_indices.find(l1_amb_sat);
            const auto l2_amb_it = base_state.ambiguity_indices.find(l2_amb_sat);
            if (l1_amb_it == base_state.ambiguity_indices.end() ||
                l2_amb_it == base_state.ambiguity_indices.end()) {
                continue;
            }

            auto amb_it = beta_state.ambiguity_columns.find(osr.satellite);
            if (amb_it == beta_state.ambiguity_columns.end()) {
                const int ambiguity_column = beta_state.state.size();
                ensure_beta_size(ambiguity_column + 1);
                beta_state.ambiguity_columns.emplace(osr.satellite, ambiguity_column);
                beta_state.state(ambiguity_column) =
                    alpha1 * base_state.state(l1_amb_it->second) +
                    alpha2 * base_state.state(l2_amb_it->second);
                beta_state.covariance(ambiguity_column, ambiguity_column) =
                    std::max(ppp_config_.initial_ambiguity_variance, 1.0);
                amb_it = beta_state.ambiguity_columns.find(osr.satellite);
            } else {
                beta_state.covariance(amb_it->second, amb_it->second) =
                    std::max(beta_state.covariance(amb_it->second, amb_it->second) +
                                 ppp_config_.process_noise_ambiguity,
                             1e-6);
            }
            beta_state.state(amb_it->second) =
                alpha1 * base_state.state(l1_amb_it->second) +
                alpha2 * base_state.state(l2_amb_it->second);

            if_seeds.emplace(osr.satellite,
                             IfObservationSeed{&osr,
                                               clock_it->second,
                                               amb_it->second,
                                               l1_amb_it->second,
                                               l2_amb_it->second,
                                               alpha1,
                                               alpha2});
        }
        PPPState measurement_state = base_state;
        measurement_state.state.segment(measurement_state.pos_index, 3) =
            beta_state.state.head(3);
        if (beta_state.trop_column >= 0 &&
            measurement_state.trop_index >= 0 &&
            measurement_state.trop_index < measurement_state.state.size()) {
            measurement_state.state(measurement_state.trop_index) =
                beta_state.state(beta_state.trop_column);
        }
        for (const auto& [system, clock_column] : beta_state.clock_columns) {
            const int global_clock_index = global_clock_index_for_system(system);
            if (global_clock_index >= 0 &&
                global_clock_index < measurement_state.state.size()) {
                measurement_state.state(global_clock_index) =
                    beta_state.state(clock_column);
            }
        }
        const auto beta_measurements = ppp_clas::buildEpochMeasurements(
            obs,
            osr_corrections,
            measurement_state,
            ppp_config_,
            measurement_state.state.segment(measurement_state.pos_index, 3),
            measurement_state.state(measurement_state.clock_index),
            measurement_state.state(measurement_state.trop_index),
            epoch_atmos,
            trop_mapping_for_validation,
            {},
            false);
        struct PerFreqRows {
            const ppp_clas::MeasurementRow* code[2]{nullptr, nullptr};
            const ppp_clas::MeasurementRow* phase[2]{nullptr, nullptr};
        };
        std::map<SatelliteId, PerFreqRows> rows_by_satellite;
        for (const auto& row : beta_measurements.measurements) {
            if (row.freq_index < 0 || row.freq_index >= 2) {
                continue;
            }
            auto& slot = rows_by_satellite[row.satellite];
            if (row.is_phase) {
                slot.phase[row.freq_index] = &row;
            } else {
                slot.code[row.freq_index] = &row;
            }
        }

        for (const auto& [satellite, seed] : if_seeds) {
            const auto rows_it = rows_by_satellite.find(satellite);
            if (rows_it == rows_by_satellite.end()) {
                continue;
            }
            const auto& rows = rows_it->second;

            auto build_row = [&](bool phase) {
                LocalIfRow row;
                row.H = Eigen::RowVectorXd::Zero(beta_state.state.size());
                if (phase) {
                    row.H.segment(0, 3) =
                        seed.alpha1 * rows.phase[0]->H.segment(0, 3) +
                        seed.alpha2 * rows.phase[1]->H.segment(0, 3);
                    if (beta_state.trop_column >= 0 &&
                        measurement_state.trop_index >= 0 &&
                        measurement_state.trop_index < rows.phase[0]->H.size() &&
                        measurement_state.trop_index < rows.phase[1]->H.size()) {
                        row.H(beta_state.trop_column) =
                            seed.alpha1 * rows.phase[0]->H(measurement_state.trop_index) +
                            seed.alpha2 * rows.phase[1]->H(measurement_state.trop_index);
                    }
                } else {
                    row.H.segment(0, 3) =
                        seed.alpha1 * rows.code[0]->H.segment(0, 3) +
                        seed.alpha2 * rows.code[1]->H.segment(0, 3);
                    if (beta_state.trop_column >= 0 &&
                        measurement_state.trop_index >= 0 &&
                        measurement_state.trop_index < rows.code[0]->H.size() &&
                        measurement_state.trop_index < rows.code[1]->H.size()) {
                        row.H(beta_state.trop_column) =
                            seed.alpha1 * rows.code[0]->H(measurement_state.trop_index) +
                            seed.alpha2 * rows.code[1]->H(measurement_state.trop_index);
                    }
                }
                row.H(seed.clock_column) = 1.0;
                row.is_phase = phase;
                row.satellite = satellite;
                return row;
            };

            if (rows.phase[0] == nullptr || rows.phase[1] == nullptr) {
                continue;
            }
            const double base_if_ambiguity =
                seed.alpha1 * measurement_state.state(seed.l1_ambiguity_index) +
                seed.alpha2 * measurement_state.state(seed.l2_ambiguity_index);
            const double ambiguity_gap =
                base_if_ambiguity - beta_state.state(seed.ambiguity_column);
            auto phase_row = build_row(true);
            phase_row.H(seed.ambiguity_column) = 1.0;
            phase_row.residual =
                seed.alpha1 * rows.phase[0]->residual +
                seed.alpha2 * rows.phase[1]->residual +
                ambiguity_gap;
            phase_row.variance =
                std::max(
                    seed.alpha1 * seed.alpha1 * rows.phase[0]->variance +
                        seed.alpha2 * seed.alpha2 * rows.phase[1]->variance,
                    1e-8);
            if (pppDebugEnabled() &&
                phase_row.satellite.system == GNSSSystem::GPS &&
                (phase_row.satellite.prn == 25 || phase_row.satellite.prn == 26 ||
                 phase_row.satellite.prn == 29 || phase_row.satellite.prn == 31)) {
                std::cerr << "[CLAS-IF-PHASE] sat="
                          << phase_row.satellite.toString()
                          << " if_raw="
                          << (seed.alpha1 * rows.phase[0]->residual +
                              seed.alpha2 * rows.phase[1]->residual)
                          << " amb_gap=" << ambiguity_gap
                          << " beta_amb=" << beta_state.state(seed.ambiguity_column)
                          << " base_if_amb=" << base_if_ambiguity
                          << " if_residual=" << phase_row.residual
                          << "\n";
            }
            local_rows.push_back(std::move(phase_row));
        }

        for (const auto& [system, clock_column] : beta_state.clock_columns) {
            const int global_clock_index = global_clock_index_for_system(system);
            if (global_clock_index < 0 || global_clock_index >= base_state.state.size()) {
                continue;
            }
            LocalIfRow row;
            row.H = Eigen::RowVectorXd::Zero(beta_state.state.size());
            row.H(clock_column) = 1.0;
            row.residual = base_state.state(global_clock_index) - beta_state.state(clock_column);
            row.variance = kBetaClockAnchorVariance;
            row.is_phase = false;
            local_rows.push_back(std::move(row));
        }
        if (beta_state.trop_column >= 0 &&
            base_state.trop_index >= 0 &&
            base_state.trop_index < base_state.state.size()) {
            LocalIfRow row;
            row.H = Eigen::RowVectorXd::Zero(beta_state.state.size());
            row.H(beta_state.trop_column) = 1.0;
            row.residual =
                base_state.state(base_state.trop_index) -
                beta_state.state(beta_state.trop_column);
            row.variance = std::max(ppp_config_.clas_trop_prior_variance, 1e-8);
            row.is_phase = false;
            local_rows.push_back(std::move(row));
        }

        bool fixed_solved = false;
        double fixed_shift = 0.0;
        double fixed_phase_rms = 0.0;
        double fixed_constraint_mean_abs = 1e9;
        double fixed_constraint_max_abs = 1e9;
        int fixed_constraint_rows = 0;
        int minimum_constraint_rows = 3;
        VectorXd linearized_delta_debug;
        bool have_linearized_delta_debug = false;

        if (local_rows.size() >= 6 && beta_state.state.size() >= 4) {
            const VectorXd linearization_state = beta_state.state;
            const int nrows = static_cast<int>(local_rows.size());
            const int nstate = beta_state.state.size();
            MatrixXd H = MatrixXd::Zero(nrows, nstate);
            MatrixXd R = MatrixXd::Zero(nrows, nrows);
            VectorXd v = VectorXd::Zero(nrows);
            for (int i = 0; i < nrows; ++i) {
                H.row(i) = local_rows[static_cast<size_t>(i)].H;
                R(i, i) = local_rows[static_cast<size_t>(i)].variance;
                v(i) = local_rows[static_cast<size_t>(i)].residual;
            }
            const MatrixXd S = H * beta_state.covariance * H.transpose() + R;
            Eigen::LDLT<MatrixXd> s_ldlt(S);
            if (s_ldlt.info() == Eigen::Success) {
                const MatrixXd K =
                    beta_state.covariance * H.transpose() *
                    s_ldlt.solve(MatrixXd::Identity(nrows, nrows));
                beta_state.state += K * v;
                beta_state.covariance =
                    (MatrixXd::Identity(nstate, nstate) - K * H) * beta_state.covariance;
            }
            const Vector3d pre_constraint_position = beta_state.state.head(3);
            if (ambiguity_resolution.accepted) {
                std::vector<Eigen::RowVectorXd> constraint_H_rows;
                std::vector<double> constraint_residuals;
                std::vector<GNSSSystem> constraint_systems;
                for (const auto& dd_constraint : wlnl_dd_constraints_) {
                    const auto ref_it = if_seeds.find(dd_constraint.ref_satellite);
                    const auto sat_it = if_seeds.find(dd_constraint.sat_satellite);
                    if (ref_it == if_seeds.end() || sat_it == if_seeds.end()) {
                        continue;
                    }
                    const auto& sat_osr = *sat_it->second.osr;
                    const double f1 = sat_osr.frequencies[0];
                    const double f2 = sat_osr.frequencies[1];
                    const double denom = f1 * f1 - f2 * f2;
                    if (std::abs(denom) < 1.0) {
                        continue;
                    }
                    const double lambda1 = sat_osr.wavelengths[0];
                    const double lambda2 = sat_osr.wavelengths[1];
                    const double lambda_nl = constants::SPEED_OF_LIGHT / (f1 + f2);
                    const double lambda_wl =
                        constants::SPEED_OF_LIGHT / std::abs(f1 - f2);
                    const double beta = f1 * f2 / denom;
                    if (lambda1 <= 0.0 || lambda2 <= 0.0 ||
                        lambda_nl <= 0.0 || lambda_wl <= 0.0 ||
                        std::abs(beta * lambda_wl) < 1e-9 ||
                        std::abs(lambda2 * lambda2 - lambda1 * lambda1) < 1e-9) {
                        continue;
                    }
                    const double n_nl =
                        (dd_constraint.l1_dd_ambiguity_m +
                         dd_constraint.l2_dd_ambiguity_m) /
                        (2.0 * lambda_nl);
                    const double n_wl =
                        (dd_constraint.l1_dd_ambiguity_m -
                         dd_constraint.l2_dd_ambiguity_m) /
                        (2.0 * beta * lambda_wl);
                    const double n1 = 0.5 * (n_nl + n_wl);
                    const double n2 = 0.5 * (n_nl - n_wl);
                    const double c1 =
                        (lambda2 * lambda2) /
                        (lambda2 * lambda2 - lambda1 * lambda1);
                    const double c2 =
                        -(lambda1 * lambda1) /
                        (lambda2 * lambda2 - lambda1 * lambda1);
                    const double fixed_dd_if_m =
                        c1 * lambda1 * n1 + c2 * lambda2 * n2;
                    Eigen::RowVectorXd h =
                        Eigen::RowVectorXd::Zero(beta_state.state.size());
                    h(ref_it->second.ambiguity_column) = 1.0;
                    h(sat_it->second.ambiguity_column) = -1.0;
                    const double float_dd_if_m =
                        beta_state.state(ref_it->second.ambiguity_column) -
                        beta_state.state(sat_it->second.ambiguity_column);
                    const double dd_if_residual = fixed_dd_if_m - float_dd_if_m;
                    if (pppDebugEnabled() && constraint_H_rows.size() < 3) {
                        std::cerr << "[CLAS-WLNL-BETA] ref="
                                  << dd_constraint.ref_satellite.toString()
                                  << " sat=" << dd_constraint.sat_satellite.toString()
                                  << " fixed_if=" << fixed_dd_if_m
                                  << " float_if=" << float_dd_if_m
                                  << " seed_dd=0\n";
                    }
                    constraint_H_rows.push_back(std::move(h));
                    constraint_residuals.push_back(dd_if_residual);
                    constraint_systems.push_back(dd_constraint.sat_satellite.system);
                }

                if (constraint_residuals.size() >= 3) {
                    int gps_constraints = 0;
                    double max_gps_abs_residual = 0.0;
                    int non_gps_outlier_index = -1;
                    double non_gps_outlier_abs_residual = 0.0;
                    for (size_t i = 0; i < constraint_residuals.size(); ++i) {
                        const double abs_residual = std::abs(constraint_residuals[i]);
                        if (constraint_systems[i] == GNSSSystem::GPS) {
                            ++gps_constraints;
                            max_gps_abs_residual =
                                std::max(max_gps_abs_residual, abs_residual);
                        } else if (abs_residual > non_gps_outlier_abs_residual) {
                            non_gps_outlier_abs_residual = abs_residual;
                            non_gps_outlier_index = static_cast<int>(i);
                        }
                    }
                    if (gps_constraints >= 2 &&
                        non_gps_outlier_index >= 0 &&
                        non_gps_outlier_abs_residual > 0.5 &&
                        non_gps_outlier_abs_residual >
                            2.0 * std::max(max_gps_abs_residual, 1e-6)) {
                        constraint_H_rows.erase(
                            constraint_H_rows.begin() + non_gps_outlier_index);
                        constraint_residuals.erase(
                            constraint_residuals.begin() + non_gps_outlier_index);
                        constraint_systems.erase(
                            constraint_systems.begin() + non_gps_outlier_index);
                        minimum_constraint_rows = 2;
                        if (pppDebugEnabled()) {
                            std::cerr << "[CLAS-WLNL-FIX] drop non-gps outlier="
                                      << non_gps_outlier_abs_residual << "\n";
                        }
                    }
                }

                fixed_constraint_rows = static_cast<int>(constraint_H_rows.size());
                if (fixed_constraint_rows > 0) {
                    double sum_abs = 0.0;
                    double max_abs = 0.0;
                    for (const double residual : constraint_residuals) {
                        const double abs_residual = std::abs(residual);
                        sum_abs += abs_residual;
                        max_abs = std::max(max_abs, abs_residual);
                    }
                    fixed_constraint_mean_abs =
                        sum_abs / static_cast<double>(fixed_constraint_rows);
                    fixed_constraint_max_abs = max_abs;
                }
                if (fixed_constraint_rows >= minimum_constraint_rows) {
                    MatrixXd Hc = MatrixXd::Zero(fixed_constraint_rows, nstate);
                    MatrixXd Rc =
                        MatrixXd::Identity(fixed_constraint_rows, fixed_constraint_rows) *
                        1e-6;
                    VectorXd vc = VectorXd::Zero(fixed_constraint_rows);
                    for (int i = 0; i < fixed_constraint_rows; ++i) {
                        Hc.row(i) = constraint_H_rows[static_cast<size_t>(i)];
                        vc(i) = constraint_residuals[static_cast<size_t>(i)];
                    }
                    const MatrixXd Sc =
                        Hc * beta_state.covariance * Hc.transpose() + Rc;
                    Eigen::LDLT<MatrixXd> sc_ldlt(Sc);
                    if (sc_ldlt.info() == Eigen::Success) {
                        const MatrixXd Kc =
                            beta_state.covariance * Hc.transpose() *
                            sc_ldlt.solve(MatrixXd::Identity(
                                fixed_constraint_rows, fixed_constraint_rows));
                        beta_state.state += Kc * vc;
                        beta_state.covariance =
                            (MatrixXd::Identity(nstate, nstate) - Kc * Hc) *
                            beta_state.covariance;
                    }
                }
                fixed_shift =
                    (beta_state.state.head(3) - pre_constraint_position).norm();
                wlnl_fixed_position = beta_state.state.head(3);
            }

            double phase_sum_sq = 0.0;
            int phase_count = 0;
            const VectorXd linearized_delta = beta_state.state - linearization_state;
            for (const auto& row : local_rows) {
                if (!row.is_phase) {
                    continue;
                }
                const double postfit = row.residual - row.H.dot(linearized_delta);
                phase_sum_sq += postfit * postfit;
                ++phase_count;
            }
            if (phase_count > 0) {
                fixed_phase_rms = std::sqrt(phase_sum_sq / phase_count);
            }
            linearized_delta_debug = linearized_delta;
            have_linearized_delta_debug = true;

            fixed_solved =
                ambiguity_resolution.accepted &&
                std::isfinite(fixed_shift) &&
                std::isfinite(fixed_phase_rms) &&
                fixed_constraint_rows >= minimum_constraint_rows &&
                fixed_shift < 10.0 &&
                fixed_phase_rms < 0.40 &&
                fixed_constraint_mean_abs < 1.5 &&
                fixed_constraint_max_abs < 2.0;
        }
        if (pppDebugEnabled()) {
            std::cerr << "[CLAS-WLNL-FIX] rows=" << local_rows.size()
                      << " constraints=" << fixed_constraint_rows
                      << " solved=" << (fixed_solved ? 1 : 0)
                      << " pos_shift=" << fixed_shift
                      << " phase_rms=" << fixed_phase_rms
                      << " dd_mean=" << fixed_constraint_mean_abs
                      << " dd_max=" << fixed_constraint_max_abs
                      << "\n";
            if (ambiguity_resolution.accepted && have_linearized_delta_debug) {
                int logged_phase_rows = 0;
                for (const auto& row : local_rows) {
                    if (!row.is_phase) {
                        continue;
                    }
                    const double postfit =
                        row.residual - row.H.dot(linearized_delta_debug);
                    std::cerr << "[CLAS-WLNL-PHASE] sat="
                              << row.satellite.toString()
                              << " prefit=" << row.residual
                              << " postfit=" << postfit
                              << " sigma=" << std::sqrt(std::max(row.variance, 0.0))
                              << "\n";
                    if (++logged_phase_rows >= 4) {
                        break;
                    }
                }
            }
        }
        if (fixed_solved) {
            // Keep the CLASLIB-style fixed-state projection from resolveWlnlFix().
            // The local beta reprojection remains a diagnostic / fallback path only.
            wlnl_fixed_position_ok = true;
        }
    }

    const ppp_shared::PPPState* solution_state_ptr = &filter_state_;
    if (ambiguity_resolution.accepted &&
        ppp_config_.ar_method == PPPConfig::ARMethod::DD_WLNL &&
        has_wlnl_fixed_state_) {
        solution_state_ptr = &wlnl_fixed_state_;
    } else if (ambiguity_resolution.accepted &&
               ambiguity_resolution.has_fixed_filter_state) {
        solution_state_ptr = &ambiguity_resolution.fixed_filter_state;
    }
    const ppp_shared::PPPState& solution_state = *solution_state_ptr;
    solution = ppp_clas::finalizeEpochSolution(
        solution_state,
        ambiguity_resolution.accepted,
        last_ar_ratio_,
        last_fixed_ambiguities_,
        static_cast<int>(osr_corrections.size()));

    if (wlnl_fixed_position_ok && !has_wlnl_fixed_state_) {
        solution.position_ecef = wlnl_fixed_position;
    }

    // Multi-epoch SD AR: accumulate DD float ambiguities over epochs,
    // then fix with LAMBDA when variance is small enough.
    {
        const auto sd_ar_result = ppp_clas_sd::solveMultiEpochSdAr(
            clas_dd_accumulator_,
            obs,
            osr_corrections,
            solution.position_ecef,
            3.0,   // AR ratio threshold
            20,    // Min accumulation epochs before attempting LAMBDA
            pppDebugEnabled());
        if (sd_ar_result.valid && sd_ar_result.code_rms >= 3.0) {
            solution.position_ecef = sd_ar_result.position;
            solution.status = SolutionStatus::PPP_FIXED;
        }
    }

    solution.time = obs.time;
    solution.receiver_clock_bias =
        solution_state.state(solution_state.clock_index) / constants::SPEED_OF_LIGHT;
    double latitude = 0.0;
    double longitude = 0.0;
    double height = 0.0;
    ecef2geodetic(solution.position_ecef, latitude, longitude, height);
    solution.position_geodetic = GeodeticCoord(latitude, longitude, height);
    if (pppDebugEnabled()) {
        std::cerr << "[CLAS-STATE-DUMP] tow=" << obs.time.tow
                  << " pos_x=" << filter_state_.state(0)
                  << " pos_y=" << filter_state_.state(1)
                  << " pos_z=" << filter_state_.state(2)
                  << " trop="
                  << (filter_state_.trop_index >= 0
                          ? filter_state_.state(filter_state_.trop_index)
                          : 0.0)
                  << " clk="
                  << (filter_state_.clock_index >= 0
                          ? filter_state_.state(filter_state_.clock_index)
                          : 0.0)
                  << "\n";
        for (const auto& [sat, idx] : filter_state_.ionosphere_indices) {
            if (sat.system == GNSSSystem::GPS &&
                (sat.prn == 14 || sat.prn == 25 || sat.prn == 26 ||
                 sat.prn == 29 || sat.prn == 31 || sat.prn == 32)) {
                std::cerr << "[CLAS-STATE-IONO] tow=" << obs.time.tow
                          << " sat=" << sat.toString()
                          << " iono=" << filter_state_.state(idx)
                          << " var=" << filter_state_.covariance(idx, idx)
                          << "\n";
            }
        }
        for (const auto& [sat, idx] : filter_state_.ambiguity_indices) {
            bool log_sat = false;
            switch (sat.system) {
                case GNSSSystem::GPS:
                    log_sat = sat.prn == 14 || sat.prn == 25 || sat.prn == 26 ||
                              sat.prn == 29 || sat.prn == 31 || sat.prn == 32 ||
                              sat.prn == 114 ||
                              sat.prn == 125 || sat.prn == 126 || sat.prn == 129 ||
                              sat.prn == 131 || sat.prn == 132;
                    break;
                case GNSSSystem::Galileo:
                    log_sat = sat.prn == 7 || sat.prn == 27 || sat.prn == 30 ||
                              sat.prn == 107 || sat.prn == 127 || sat.prn == 130;
                    break;
                case GNSSSystem::QZSS:
                    log_sat = sat.prn == 1 || sat.prn == 2 || sat.prn == 3 ||
                              sat.prn == 101 || sat.prn == 102 || sat.prn == 103;
                    break;
                default:
                    break;
            }
            if (!log_sat) {
                continue;
            }
            const auto amb_it = ambiguity_states_.find(sat);
            const int lock_count =
                amb_it != ambiguity_states_.end() ? amb_it->second.lock_count : -1;
            const bool needs_reinit =
                amb_it != ambiguity_states_.end() ? amb_it->second.needs_reinitialization : false;
            std::cerr << "[CLAS-STATE-AMB] tow=" << obs.time.tow
                      << " sat=" << sat.toString()
                      << " x=" << filter_state_.state(idx)
                      << " var=" << filter_state_.covariance(idx, idx)
                      << " lock=" << lock_count
                      << " reinit=" << (needs_reinit ? 1 : 0)
                      << "\n";
        }
    }
    return solution;
}

}  // namespace libgnss
