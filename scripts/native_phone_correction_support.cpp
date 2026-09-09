#include <libgnss++/io/rinex.hpp>
#include <libgnss++/algorithms/base_pseudorange_compensation.hpp>
#include <libgnss++/io/android_raw_gnss.hpp>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <cmath>
int main(int argc, char** argv) {
    const bool temporal=argc==5 && std::string(argv[4])=="dense-temporal";
    if (argc != 4 && !temporal) return 2;
    libgnss::io::RINEXReader base, navigation;
    base.setSourceHeaderTrackingFilter(true);
    base.setPreserveAdditionalFrequencyBands(true);
    if (!base.open(argv[1]) || !navigation.open(argv[2])) return 3;
    libgnss::io::RINEXReader::RINEXHeader header;
    if (!base.readHeader(header) || header.version < 3 || header.version >= 4 ||
        !header.has_approximate_position || !header.has_antenna_delta ||
        !header.antenna_delta.isZero()) return 4;
    libgnss::NavigationData nav;
    if (!navigation.readNavigationData(nav)) return 5;
    libgnss::ObservationSeries observations;
    if (!base.readAllObservations(observations)) return 6;
    libgnss::base_pseudorange_compensation::Config config;
    config.source_complete = config.use_source_epoch_states = true;
    config.use_source_fgo_frequency_slots = true;
    config.use_dense_epoch_smoothing = temporal;
    config.base_position_ecef = header.approximate_position;
    config.approximate_position_present = config.station_reference_verified = true;
    config.antenna_reference_is_approx_position = true;
    config.expected_interval_s = 1; config.moving_mean_samples = 151;
    libgnss::base_pseudorange_compensation::Model model;
    const bool ok = model.build(observations,nav,config);
    if (!ok) return 7;
    libgnss::io::AndroidRawGnssConfig phone_config;
    phone_config.verify_enriched_pseudorange = false;
    libgnss::io::AndroidRawGnssResult phone;
    std::string error;
    if (!libgnss::io::loadAndroidRawGnssCsv(argv[3], phone_config, phone, error)) return 8;
    std::size_t candidates=0, missing=0, unavailable=0, finite=0;
    std::size_t before_domain=0, after_domain=0, in_domain_failure=0;
    std::map<std::string, std::array<std::size_t,4>> breakdown;
    using Key=std::pair<libgnss::SatelliteId,libgnss::SignalType>;
    std::map<Key,double> previous;
    std::map<Key,std::vector<double>> gps_l1_levels;
    libgnss::GNSSTime previous_time;
    bool have_previous=false;
    std::vector<double> gps_l1_common_abs_rate;
    double maximum_individual_rate=0;
    for (const auto& epoch : phone.observations.epochs) {
        std::map<Key,double> current;
        std::vector<double> rates;
        for (const auto& row : epoch.observations) {
            if (!row.valid || !row.has_pseudorange) continue;
            ++candidates;
            auto& counts = breakdown[std::to_string(static_cast<int>(row.satellite.system)) +
                                    ":" + std::to_string(static_cast<int>(row.signal))];
            ++counts[0];
            if (!model.hasStream(row.satellite,row.signal)) { ++missing; ++counts[1]; continue; }
            double correction;
            if (!model.correctionAt(epoch.time,row.satellite,row.signal,correction) ||
                !std::isfinite(correction)) {
                ++unavailable; ++counts[2];
                libgnss::GNSSTime first_time, last_time;
                if (!model.streamTimeDomain(row.satellite,row.signal,first_time,last_time)) return 9;
                if (epoch.time < first_time) ++before_domain;
                else if (epoch.time > last_time) ++after_domain;
                else ++in_domain_failure;
                continue;
            }
            ++finite; ++counts[3];
            if (temporal && row.satellite.system==libgnss::GNSSSystem::GPS &&
                row.signal==libgnss::SignalType::GPS_L1CA) {
                const Key key{row.satellite,row.signal};
                gps_l1_levels[key].push_back(correction);
                if (!current.emplace(key,correction).second) return 10;
                const double dt=have_previous ? epoch.time-previous_time : 0;
                if (have_previous && dt>=0.5 && dt<=1.5 && previous.count(key)) {
                    const double rate=(correction-previous.at(key))/dt;
                    if (!std::isfinite(rate)) return 11;
                    rates.push_back(rate);
                    maximum_individual_rate=std::max(maximum_individual_rate,std::abs(rate));
                }
            }
        }
        if (rates.size()>=4) {
            std::sort(rates.begin(),rates.end());
            const auto n=rates.size();
            gps_l1_common_abs_rate.push_back(std::abs(n%2 ? rates[n/2] : (rates[n/2-1]+rates[n/2])/2));
        }
        previous.swap(current);previous_time=epoch.time;have_previous=true;
    }
    std::cout << "{\"phone_epochs\":" << phone.observations.epochs.size()
              << ",\"candidates\":" << candidates << ",\"missing_stream\":" << missing
              << ",\"unavailable\":" << unavailable << ",\"finite\":" << finite << "}\n";
    const auto& d = model.diagnostics();
    std::cout << "{\"before_domain\":" << before_domain
              << ",\"after_domain\":" << after_domain
              << ",\"in_domain_failure\":" << in_domain_failure << "}\n";
    std::cout << "{\"by_system_signal\":{";
    bool first = true;
    for (const auto& [key, counts] : breakdown) {
        if (!first) std::cout << ',';
        first = false;
        std::cout << std::quoted(key) << ":[" << counts[0] << ',' << counts[1]
                  << ',' << counts[2] << ',' << counts[3] << ']';
    }
    std::cout << "}}\n";
    std::cout << "{\"built\":" << (ok ? "true" : "false")
              << ",\"epochs\":" << observations.epochs.size()
              << ",\"states\":" << d.source_epoch_states_built
              << ",\"signal_rows\":" << d.source_complete_signal_rows
              << ",\"excluded_frequency_rows\":" << d.source_frequency_rows_excluded
              << ",\"streams\":" << d.matching_streams
              << ",\"failure\":" << std::quoted(d.failure) << "}\n";
    if (temporal) {
        if (gps_l1_common_abs_rate.empty()) return 12;
        std::sort(gps_l1_common_abs_rate.begin(),gps_l1_common_abs_rate.end());
        const auto percentile=[&](double q) {
            const double index=q*(gps_l1_common_abs_rate.size()-1);
            const auto lower=static_cast<std::size_t>(index);
            const auto upper=std::min(lower+1,gps_l1_common_abs_rate.size()-1);
            return gps_l1_common_abs_rate[lower]+(index-lower)*(gps_l1_common_abs_rate[upper]-gps_l1_common_abs_rate[lower]);
        };
        std::cout<<std::setprecision(17)<<"{\"gps_l1_common_rate_intervals\":"<<gps_l1_common_abs_rate.size()
                 <<",\"median_abs_common_rate_mps\":"<<percentile(0.5)
                 <<",\"p95_abs_common_rate_mps\":"<<percentile(0.95)
                 <<",\"max_abs_common_rate_mps\":"<<gps_l1_common_abs_rate.back()
                 <<",\"max_abs_individual_rate_mps\":"<<maximum_individual_rate<<"}\n";
        const auto median=[](std::vector<double> values) {
            std::sort(values.begin(),values.end());
            const auto n=values.size();
            return n%2 ? values[n/2] : (values[n/2-1]+values[n/2])/2;
        };
        std::vector<double> absolute_centers,within_stream_mads;
        std::size_t supported_rows=0;
        for (const auto& [key,values] : gps_l1_levels) {
            // Fixed support threshold; avoid treating brief satellite arcs
            // as evidence for a persistent correction component.
            if (values.size()<300) continue;
            const double center=median(values);
            std::vector<double> deviations;
            for (double value : values) deviations.push_back(std::abs(value-center));
            absolute_centers.push_back(std::abs(center));
            within_stream_mads.push_back(median(deviations));
            supported_rows+=values.size();
        }
        if (absolute_centers.empty()) return 13;
        std::cout<<"{\"gps_l1_supported_streams\":"<<absolute_centers.size()
                 <<",\"supported_rows\":"<<supported_rows
                 <<",\"median_absolute_stream_center_m\":"<<median(absolute_centers)
                 <<",\"median_within_stream_mad_m\":"<<median(within_stream_mads)<<"}\n";
    }
    return 0; // Reports model failure as diagnostic data, never a fallback.
}
