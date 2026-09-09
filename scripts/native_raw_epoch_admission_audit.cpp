// Raw-only loader-stage reason counts. No coordinates or solver invocation.
#include <libgnss++/io/android_raw_gnss.hpp>
#include <libgnss++/algorithms/observable_upstream_preprocessing.hpp>
#include <iostream>
#include <map>
#include <string>

int main(int argc, char** argv) {
    if (argc != 4) return 2;
    const auto begin = std::stoull(argv[2]), end = std::stoull(argv[3]);
    libgnss::io::AndroidRawGnssConfig config;
    config.verify_enriched_pseudorange = false;
    config.device_model = "pixel5";
    libgnss::io::AndroidRawGnssResult raw;
    std::string error;
    if (!libgnss::io::loadAndroidRawGnssCsv(argv[1],config,raw,error)) {
        std::cerr << error << '\n'; return 1;
    }
    if (begin > end || end >= raw.epoch_utc_time_millis.size()) return 2;
    namespace up = libgnss::observable_upstream;
    std::vector<up::EpochMask> masks;
    std::size_t p_rejections=0, l_rejections=0;
    up::applyAdjacentMasks(raw.observations.epochs,"pixel5",masks,p_rejections,l_rejections);
    for (auto epoch=begin; epoch<=end; ++epoch) {
        std::map<std::string,std::size_t> reasons;
        std::size_t selected=0, count=0;
        for (const auto& row:raw.raw_row_diagnostics) {
            if (row.utc_time_millis != raw.epoch_utc_time_millis[epoch]) continue;
            ++count; selected += row.selected;
            ++reasons[row.loader_reason];
        }
        std::cout << "epoch=" << epoch << " raw=" << count << " selected=" << selected;
        std::size_t low_snr=0, masked=0, either=0;
        for (const auto& row:raw.observations.epochs[epoch].observations) {
            const bool low = !std::isfinite(row.snr) || row.snr < 20.0;
            const bool pd = masks[epoch].pseudorange.count({row.satellite,row.signal}) != 0;
            low_snr += low; masked += pd; either += low || pd;
        }
        std::cout << " snr_below_20=" << low_snr << " adjacent_pd_mask=" << masked
                  << " union_snr_pd=" << either;
        std::vector<double> pd_values;
        if (epoch > 0) {
            const auto& current = raw.observations.epochs[epoch];
            const auto& previous = raw.observations.epochs[epoch-1];
            const double dt = current.time - previous.time;
            for (const auto& row:current.observations) {
                const auto* old = previous.getObservation(row.satellite,row.signal);
                const double wavelength = libgnss::signalWavelengthMeters(row);
                if (!old || dt <= 0 || dt > 1.5 || !up::finitePositive(wavelength) ||
                    !old->has_pseudorange || !row.has_pseudorange ||
                    old->pseudorange <= 0 || row.pseudorange <= 0 ||
                    !old->has_doppler || !row.has_doppler) continue;
                const double pd = up::pseudorangeDopplerDifference(
                    old->pseudorange,row.pseudorange,old->doppler,row.doppler,wavelength,dt);
                if (std::isfinite(pd)) pd_values.push_back(pd);
            }
        }
        std::sort(pd_values.begin(),pd_values.end());
        std::cout << " incoming_pd_pairs=" << pd_values.size();
        if (!pd_values.empty()) std::cout << " pd_min_m=" << pd_values.front()
            << " pd_lower_median_m=" << pd_values[(pd_values.size()-1)/2]
            << " pd_max_m=" << pd_values.back();
        for (const auto& [reason,n]:reasons) std::cout << " reason[" << reason << "]=" << n;
        std::cout << '\n';
    }
}
