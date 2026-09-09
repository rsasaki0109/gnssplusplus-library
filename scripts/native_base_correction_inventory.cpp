#include <libgnss++/io/rinex.hpp>
#include <libgnss++/algorithms/base_pseudorange_compensation.hpp>
#include <iostream>
#include <iomanip>
int main(int argc, char** argv) {
    if (argc != 3 && !(argc == 4 && std::string(argv[3]) == "source-slots")) return 2;
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
    config.use_source_fgo_frequency_slots = argc == 4;
    config.base_position_ecef = header.approximate_position;
    config.approximate_position_present = config.station_reference_verified = true;
    config.antenna_reference_is_approx_position = true;
    config.expected_interval_s = 1; config.moving_mean_samples = 151;
    libgnss::base_pseudorange_compensation::Model model;
    const bool ok = model.build(observations,nav,config);
    const auto& d = model.diagnostics();
    std::cout << "{\"built\":" << (ok ? "true" : "false")
              << ",\"epochs\":" << observations.epochs.size()
              << ",\"states\":" << d.source_epoch_states_built
              << ",\"signal_rows\":" << d.source_complete_signal_rows
              << ",\"excluded_frequency_rows\":" << d.source_frequency_rows_excluded
              << ",\"streams\":" << d.matching_streams
              << ",\"failure\":" << std::quoted(d.failure) << "}\n";
    return 0; // Reports model failure as diagnostic data, never a fallback.
}
