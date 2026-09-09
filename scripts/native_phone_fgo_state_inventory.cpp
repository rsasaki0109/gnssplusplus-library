#include <libgnss++/algorithms/fgo.hpp>
#include <libgnss++/algorithms/base_pseudorange_compensation.hpp>
#include <libgnss++/algorithms/source_pseudorange_miss_mask.hpp>
#include <libgnss++/io/android_raw_gnss.hpp>
#include <libgnss++/io/rinex.hpp>
#include <iostream>

// Raw-input admission diagnostic, not an optimized trajectory or score.
int main(int argc, char** argv) {
    if (argc != 3 && argc != 4) return 2;
    libgnss::io::RINEXReader reader;
    libgnss::NavigationData navigation;
    if (!reader.open(argv[1]) || !reader.readNavigationData(navigation)) return 3;
    libgnss::io::AndroidRawGnssConfig raw_config;
    raw_config.verify_enriched_pseudorange = false;
    libgnss::io::AndroidRawGnssResult raw;
    std::string error;
    if (!libgnss::io::loadAndroidRawGnssCsv(argv[2], raw_config, raw, error)) return 4;
    libgnss::FGOProcessor::FGOConfig config;
    config.use_source_rover_epoch_states = true;
    config.use_multi_constellation = true;
    config.use_multi_frequency_double_difference = true;
    config.use_upstream_observable_quality = true;
    libgnss::base_pseudorange_compensation::Model base_model;
    if (argc == 4) {
        libgnss::io::RINEXReader base;
        base.setSourceHeaderTrackingFilter(true);
        base.setPreserveAdditionalFrequencyBands(true);
        libgnss::io::RINEXReader::RINEXHeader header;
        libgnss::ObservationSeries observations;
        if (!base.open(argv[3]) || !base.readHeader(header) ||
            header.version < 3 || header.version >= 4 ||
            !header.has_approximate_position || !header.has_antenna_delta ||
            !header.antenna_delta.isZero() || !base.readAllObservations(observations)) return 6;
        libgnss::base_pseudorange_compensation::Config bc;
        bc.source_complete = bc.use_source_epoch_states = true;
        bc.use_source_fgo_frequency_slots = true;
        bc.base_position_ecef = header.approximate_position;
        bc.approximate_position_present = bc.station_reference_verified = true;
        bc.antenna_reference_is_approx_position = true;
        bc.expected_interval_s = 1;
        bc.moving_mean_samples = 151;
        if (!base_model.build(observations, navigation, bc)) return 7;
        // Pair the base zero-group-delay convention with the rover and SPP.
        // Raw header reference is not proof of source station-table parity.
        config.use_native_phase126_raw_base_source_complete = true;
    }
    // SPP seeds are computed from raw observations in this same process.
    // Optional base corrections also stay in this process. No optimizer/IMU.
    try {
        auto problem = libgnss::FGOProcessor(config).buildPseudorangeProblem(
            raw.observations.epochs, navigation);
        if (argc == 4) {
            libgnss::source_pseudorange_miss_mask::Report report;
            if (!libgnss::source_pseudorange_miss_mask::apply(
                    problem.pseudorange_factors, problem.epochs,
                    [&](const auto& sat, auto signal) { return base_model.hasStream(sat, signal); },
                    [&](const auto& time, const auto& sat, auto signal, double& correction) {
                        return base_model.correctionAt(time, sat, signal, correction);
                    }, report)) return 8;
            std::cout << "{\"before_correction\":" << report.original_adopted_rows
                      << ",\"corrected\":" << report.corrected_rows
                      << ",\"missing_stream\":" << report.dropped_missing_exact_stream_rows
                      << ",\"unavailable\":" << report.dropped_out_of_domain_rows
                      << ",\"nonfinite\":" << report.dropped_nonfinite_correction_rows
                      << ",\"passes\":" << report.correction_application_passes
                      << ",\"base_states\":" << base_model.diagnostics().source_epoch_states_built
                      << "}\n";
        }
        const auto& d = problem.diagnostics;
        std::cout << "{\"input_epochs\":" << d.input_epochs
                  << ",\"states_built\":" << d.source_rover_epoch_states_built
                  << ",\"missing_nav_satellite_epochs\":"
                  << d.source_rover_missing_ephemeris_satellite_epochs
                  << ",\"skipped_epochs_without_seed\":" << d.skipped_epochs_without_seed
                  << ",\"pseudorange_factors\":" << problem.pseudorange_factors.size()
                  << "}\n";
    } catch (const std::exception&) {
        // No potentially data-bearing exception payload on diagnostic output.
        std::cerr << "Native FGO state admission failed\n";
        return 5;
    }
    return 0;
}
