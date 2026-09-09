// Raw-only diagnostic: no positioning or truth input, no solver changes.
#include <libgnss++/io/imu.hpp>
#include <libgnss++/algorithms/upstream_stop_constraints.hpp>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>

int main(int argc, char** argv) {
    try {
        if (argc != 3 && !(argc == 4 && std::string(argv[3]) == "--stop-comparison"))
            throw std::invalid_argument("expected device_gnss.csv device_imu.csv [--stop-comparison]");
        libgnss::AndroidGnssUtcGpsMapping mapping;
        const auto clock = libgnss::loadAndroidGnssUtcGpsMapping(argv[1], mapping);
        if (!clock.ok) throw std::runtime_error(clock.error);
        libgnss::AndroidImuCsvConfig config;
        config.require_gnss_elapsed_anchor = true;
        config.allow_utc_wall_clock_fallback = true;
        config.apply_utc_wall_clock_fallback_offset = true;
        config.utc_wall_clock_fallback_offset_ms = -20;
        libgnss::ImuSeries series;
        const auto loaded = libgnss::loadAndroidImuCsv(argv[2], series, config, {}, &mapping);
        if (!loaded.ok) throw std::runtime_error(loaded.error);
        if (!loaded.utc_wall_clock_fallback_applied)
            throw std::runtime_error("audit requires the H UTC fallback branch");
        series.sortByTime();
        constexpr std::size_t count = 250;
        if (series.samples.size() < count) throw std::runtime_error("short IMU stream");
        Eigen::Vector3d mean = Eigen::Vector3d::Zero();
        double peak = 0.0, squared = 0.0;
        std::size_t above_source_stop_limit = 0;
        for (std::size_t i = 0; i < count; ++i) {
            const auto& g = series.samples[i].gyro_raw_radps;
            if (!g.allFinite()) throw std::runtime_error("nonfinite gyro");
            mean += g;
            peak = std::max(peak, g.norm());
            squared += g.squaredNorm();
            if (g.norm() >= 0.05) ++above_source_stop_limit;
        }
        mean /= static_cast<double>(count);
        if (argc == 4) {
            // Only the IMU mask is consumed; this sample-time query does not
            // stand in for a GNSS epoch alignment or a trajectory.
            const auto stops = libgnss::upstream_stop::detect(
                series.samples, {series.samples.front().time});
            if (!stops.ok) throw std::runtime_error(stops.error);
            std::vector<Eigen::Vector3d> block_means;
            Eigen::Vector3d block_sum = Eigen::Vector3d::Zero();
            std::size_t in_block = 0, initial_stops = 0;
            for (std::size_t i = 0; i < series.samples.size(); ++i) {
                if (i < count && stops.imu_stop[i]) ++initial_stops;
                const bool gap = i > 0 &&
                    libgnss::upstream_stop::detail::timeKey(series.samples[i].time) -
                    libgnss::upstream_stop::detail::timeKey(series.samples[i-1].time) > 0.1;
                if (!stops.imu_stop[i] || gap) { in_block = 0; block_sum.setZero(); }
                if (!stops.imu_stop[i]) continue;
                block_sum += series.samples[i].gyro_raw_radps;
                if (++in_block == count) {
                    block_means.push_back(block_sum / static_cast<double>(count));
                    in_block = 0; block_sum.setZero();
                }
            }
            if (block_means.size() < 2) throw std::runtime_error("insufficient stationary blocks");
            Eigen::Vector3d center = Eigen::Vector3d::Zero();
            for (const auto& b : block_means) center += b;
            center /= static_cast<double>(block_means.size());
            double scatter = 0.0, max_difference = 0.0;
            for (const auto& b : block_means) {
                scatter += (b-center).squaredNorm();
                max_difference = std::max(max_difference, (b-mean).norm());
            }
            std::cout << std::setprecision(17)
                      << "{\"stop_samples\":" << stops.stop_samples
                      << ",\"initial_stop_samples\":" << initial_stops
                      << ",\"stationary_blocks_250_samples\":" << block_means.size()
                      << ",\"stationary_mean_norm_radps\":" << center.norm()
                      << ",\"initial_to_stationary_mean_norm_radps\":" << (mean-center).norm()
                      << ",\"block_mean_scatter_rms_radps\":" << std::sqrt(scatter/block_means.size())
                      << ",\"max_block_to_initial_norm_radps\":" << max_difference << "}\n";
        }
        // Norms are invariant to the native fixed orthonormal mounting rotation.
        std::cout << std::setprecision(17)
                  << "{\"initial_samples\":" << count
                  << ",\"loaded_samples\":" << series.samples.size()
                  << ",\"gyro_mean_norm_radps\":" << mean.norm()
                  << ",\"gyro_rms_radps\":" << std::sqrt(squared / count)
                  << ",\"samples_at_or_above_source_stop_gyro_limit\":" << above_source_stop_limit
                  << ",\"gyro_peak_norm_radps\":" << peak << "}\n";
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
