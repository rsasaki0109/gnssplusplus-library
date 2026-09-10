#include <libgnss++/io/rinex.hpp>
#include <libgnss++/algorithms/base_pseudorange_compensation.hpp>
#include <libgnss++/algorithms/source_epoch_states.hpp>
#include <libgnss++/algorithms/phase126_raw_base_compound.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <Eigen/QR>
#include <algorithm>
#include <iostream>
#include <vector>
#include <optional>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/models/ionosphere.hpp>
#include <libgnss++/models/troposphere.hpp>

double l5IonosphereScale() {
    const double ratio = libgnss::constants::GPS_L1_FREQ / libgnss::constants::GPS_L5_FREQ;
    return ratio * ratio;
}

std::optional<Eigen::Vector4d> fit(const Eigen::MatrixXd& a, const Eigen::VectorXd& y) {
    if (a.cols() != 4 || a.rows() < 4 || y.size() != a.rows() ||
        !a.allFinite() || !y.allFinite()) return std::nullopt;
    const auto qr = a.colPivHouseholderQr();
    if (qr.rank() != 4) return std::nullopt;
    const Eigen::Vector4d x = qr.solve(y);
    if (!x.allFinite()) return std::nullopt;
    return x;
}

bool selfTest() {
    Eigen::MatrixXd a(6,4);
    a << -1,0,0,1, 1,0,0,1, 0,-1,0,1, 0,1,0,1, 0,0,-1,1, 0,0,1,1;
    const Eigen::Vector4d known(1,2,3,100);
    const auto recovered = fit(a, a*known);
    if (!recovered || (*recovered-known).norm() > 1e-10) return false;
    const Eigen::Vector4d frequency_bias(0.5,-0.25,0.75,4);
    const auto difference = fit(a, a*(known+frequency_bias)-a*known);
    if (!difference || (*difference-frequency_bias).norm() > 1e-10) return false;
    const double gamma = l5IonosphereScale();
    const Eigen::VectorXd y1 = a*(known+frequency_bias);
    const Eigen::VectorXd y5 = a*(known+gamma*frequency_bias);
    const auto neutral = fit(a,(gamma*y1-y5)/(gamma-1));
    const auto dispersive = fit(a,(y5-y1)/(gamma-1));
    if (!neutral || !dispersive || (*neutral-known).norm() > 1e-10 ||
        (*dispersive-frequency_bias).norm() > 1e-10) return false;
    if (fit(Eigen::MatrixXd::Ones(6,4), Eigen::VectorXd::Ones(6))) return false;
    if (fit(a.topRows(3), Eigen::VectorXd::Ones(3))) return false;
    auto bad = Eigen::VectorXd::Ones(6).eval();
    bad[0] = std::numeric_limits<double>::quiet_NaN();
    return !fit(a,bad);
}

double median(std::vector<double> v) {
    std::sort(v.begin(), v.end());
    const auto n = v.size();
    return n % 2 ? v[n/2] : (v[n/2-1] + v[n/2])/2;
}

// Apparent position/clock components of base residuals, not a surveyed
// coordinate solution. Diagnostic estimates never feed positioning.
int main(int argc, char** argv) {
    using namespace libgnss;
    if (argc == 2 && std::string(argv[1]) == "--self-test") {
        if (!selfTest()) return 7;
        std::cout << "synthetic clock/geometry/difference and invalid-input checks passed\n";
        return 0;
    }
    const bool raw_paired = argc == 4 && std::string(argv[3]) == "raw-paired-frequency";
    const bool paired = raw_paired || (argc == 4 && std::string(argv[3]) == "paired-frequency");
    if (argc != 3 && !paired) return 2;
    io::RINEXReader base, nr;
    base.setSourceHeaderTrackingFilter(true);
    base.setPreserveAdditionalFrequencyBands(true);
    io::RINEXReader::RINEXHeader h;
    NavigationData nav;
    ObservationSeries series;
    if (!base.open(argv[1]) || !base.readHeader(h) || h.version < 3 || h.version >= 4 ||
        !h.has_approximate_position || !h.has_antenna_delta || !h.antenna_delta.isZero() ||
        !nr.open(argv[2]) || !nr.readNavigationData(nav) || !base.readAllObservations(series)) return 3;
    base_pseudorange_compensation::Config c;
    c.source_complete = c.use_source_epoch_states = c.use_source_fgo_frequency_slots = true;
    c.use_dense_epoch_smoothing = true;
    c.base_position_ecef = h.approximate_position;
    c.approximate_position_present = c.station_reference_verified = true;
    c.antenna_reference_is_approx_position = true;
    c.expected_interval_s = 1; c.moving_mean_samples = 151;
    base_pseudorange_compensation::Model model;
    if (!raw_paired && !model.build(series, nav, c)) return 4;
    double lat, lon, height;
    ecef2geodetic(h.approximate_position,lat,lon,height);
    std::vector<double> modeled_l1_iono, modeled_trop;
    double maximum_neutral_iono_cancellation_error = 0.0;
    std::vector<double> displacement_norms, residual_rms, clock_abs;
    std::vector<double> l5_displacement, difference_displacement, difference_clock, difference_rms;
    std::vector<double> neutral_displacement, neutral_rms, dispersive_displacement;
    Vector3d first_half = Vector3d::Zero(), second_half = Vector3d::Zero();
    std::size_t first_count = 0, second_count = 0, rows_total = 0, rank_misses = 0;
    for (std::size_t ei = 0; ei < series.epochs.size(); ++ei) {
        const auto& epoch = series.epochs[ei];
        const auto states = source_transmission_clock::buildEpochStates(epoch.time, epoch.observations, nav);
        std::vector<Eigen::Vector4d> rows;
        std::vector<double> values;
        std::vector<double> values5;
        for (const auto& [sat, state] : states) {
            if (sat.system != GNSSSystem::GPS || state.ephemeris_health != 0) continue;
            const auto geom = phase126_raw_base::geodistWithSagnac(h.approximate_position, state.state.position_ecef);
            if (geom.elevation_rad < 15.0 * std::acos(-1.0)/180.0) continue;
            double correction;
            double correction5 = 0.0;
            if (raw_paired) {
                const Observation* l1 = nullptr;
                const Observation* l5 = nullptr;
                for (const auto& obs : epoch.observations) {
                    if (!(obs.satellite == sat) || !obs.valid || !obs.has_pseudorange ||
                        !std::isfinite(obs.pseudorange) || obs.pseudorange <= 0) continue;
                    if (obs.signal == SignalType::GPS_L1CA) l1 = &obs;
                    if (obs.signal == SignalType::GPS_L5) l5 = &obs;
                }
                if (!l1 || !l5) continue;
                const double iono = nav.ionosphere_model.valid ? models::ionoDelayKlobuchar(
                    lat,lon,geom.azimuth_rad,geom.elevation_rad,epoch.time.tow,
                    nav.ionosphere_model.alpha,nav.ionosphere_model.beta) : 0.0;
                const double trop = models::tropDelaySaastamoinen(h.approximate_position,geom.elevation_rad);
                const double clock = state.state.clock_seconds * constants::SPEED_OF_LIGHT;
                correction = phase126_raw_base::officialBaseCodeResidual(
                    l1->pseudorange,clock,geom.range_m,iono,trop);
                correction5 = phase126_raw_base::officialBaseCodeResidual(
                    l5->pseudorange,clock,geom.range_m,iono*l5IonosphereScale(),trop);
                const double gamma = l5IonosphereScale();
                const double with_iono = (gamma*correction-correction5)/(gamma-1);
                const double without_iono = (gamma*(correction+iono)-
                    (correction5+gamma*iono))/(gamma-1);
                maximum_neutral_iono_cancellation_error = std::max(
                    maximum_neutral_iono_cancellation_error, std::abs(with_iono-without_iono));
                modeled_l1_iono.push_back(iono); modeled_trop.push_back(trop);
            } else {
                if (!model.correctionAt(epoch.time, sat, SignalType::GPS_L1CA, correction)) continue;
                if (paired && !model.correctionAt(epoch.time, sat, SignalType::GPS_L5, correction5)) continue;
            }
            Eigen::Vector4d row;
            row.head<3>() = -geom.line_of_sight;
            row[3] = 1;
            rows.push_back(row); values.push_back(correction);
            values5.push_back(correction5);
        }
        if (rows.size() < 4) { ++rank_misses; continue; }
        Eigen::MatrixXd a(rows.size(),4);
        Eigen::VectorXd y(rows.size());
        for (std::size_t i=0; i<rows.size(); ++i) { a.row(i)=rows[i].transpose(); y[i]=values[i]; }
        const auto fitted = fit(a,y);
        if (!fitted) { ++rank_misses; continue; }
        const Eigen::Vector4d estimate = *fitted;
        if (paired) {
            Eigen::VectorXd y5(rows.size());
            for (std::size_t i=0; i<rows.size(); ++i) y5[i] = values5[i];
            const auto fitted5 = fit(a,y5);
            const auto delta = fit(a,y5-y);
            if (!fitted5 || !delta) return 5;
            l5_displacement.push_back(fitted5->head<3>().norm());
            difference_displacement.push_back(delta->head<3>().norm());
            difference_clock.push_back(std::abs((*delta)[3]));
            difference_rms.push_back((a*(*delta)-(y5-y)).norm()/std::sqrt(double(rows.size())));
            const double gamma = l5IonosphereScale();
            const Eigen::VectorXd neutral_y = (gamma*y-y5)/(gamma-1);
            const auto neutral = fit(a,neutral_y);
            if (!neutral) return 5;
            neutral_displacement.push_back(neutral->head<3>().norm());
            neutral_rms.push_back((a*(*neutral)-neutral_y).norm()/std::sqrt(double(rows.size())));
            dispersive_displacement.push_back(delta->head<3>().norm()/(gamma-1));
        }
        displacement_norms.push_back(estimate.head<3>().norm());
        residual_rms.push_back((a*estimate-y).norm()/std::sqrt(double(rows.size())));
        clock_abs.push_back(std::abs(estimate[3]));
        rows_total += rows.size();
        if (ei < series.epochs.size()/2) { first_half += estimate.head<3>(); ++first_count; }
        else { second_half += estimate.head<3>(); ++second_count; }
    }
    if (displacement_norms.empty() || !first_count || !second_count) return 6;
    std::cout.precision(17);
    if (raw_paired) {
        if (modeled_l1_iono.empty() || maximum_neutral_iono_cancellation_error > 1e-7) return 8;
        std::cout << "{\"raw_paired\":true,\"ionosphere_model_valid\":"
                  << (nav.ionosphere_model.valid ? "true" : "false")
                  << ",\"modeled_l1_iono_median_m\":" << median(modeled_l1_iono)
                  << ",\"modeled_trop_median_m\":" << median(modeled_trop)
                  << ",\"maximum_neutral_iono_cancellation_error_m\":"
                  << maximum_neutral_iono_cancellation_error << "}\n";
    }
    if (paired) {
        std::cout << "{\"paired_frequency\":true,\"l5_displacement_norm_median_m\":" << median(l5_displacement)
                  << ",\"l5_minus_l1_displacement_norm_median_m\":" << median(difference_displacement)
                  << ",\"l5_minus_l1_absolute_clock_median_m\":" << median(difference_clock)
                  << ",\"l5_minus_l1_post_fit_rms_median_m\":" << median(difference_rms)
                  << ",\"frequency_squared_ratio\":" << l5IonosphereScale()
                  << ",\"neutral_combination_displacement_norm_median_m\":" << median(neutral_displacement)
                  << ",\"neutral_combination_post_fit_rms_median_m\":" << median(neutral_rms)
                  << ",\"dispersive_l1_equivalent_displacement_norm_median_m\":" << median(dispersive_displacement)
                  << "}\n";
    }
    std::cout << "{\"base_epochs\":" << series.epochs.size()
              << ",\"fit_epochs\":" << displacement_norms.size()
              << ",\"rank_or_count_misses\":" << rank_misses
              << ",\"rows\":" << rows_total
              << ",\"apparent_displacement_norm_median_m\":" << median(displacement_norms)
              << ",\"post_fit_rms_median_m\":" << median(residual_rms)
              << ",\"absolute_clock_median_m\":" << median(clock_abs)
              << ",\"half_mean_displacement_difference_norm_m\":"
              << (first_half/double(first_count)-second_half/double(second_count)).norm() << "}\n";
}
