#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <libgnss++/core/solution.hpp>
#include <libgnss++/fusion/fusion_processor.hpp>
#include <libgnss++/io/imu.hpp>

namespace {

struct AnchorEpoch {
    double tow = 0.0;
    bool anchor = false;
    Eigen::Vector3d ecef = Eigen::Vector3d::Zero();
};

std::vector<std::string> splitCsv(const std::string& line) {
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) fields.push_back(field);
    return fields;
}

int column(const std::vector<std::string>& header, const std::string& name) {
    for (std::size_t i = 0; i < header.size(); ++i) {
        if (header[i] == name) return static_cast<int>(i);
    }
    throw std::runtime_error("missing CSV column: " + name);
}

std::vector<AnchorEpoch> loadAnchors(const std::string& path) {
    std::ifstream input(path);
    if (!input) throw std::runtime_error("cannot open union CSV: " + path);
    std::string line;
    if (!std::getline(input, line)) throw std::runtime_error("empty union CSV");
    const auto header = splitCsv(line);
    const int tow_col = column(header, "tow");
    const int fix_col = column(header, "union_fix");
    const int x_col = column(header, "union_ecef_x");
    const int y_col = column(header, "union_ecef_y");
    const int z_col = column(header, "union_ecef_z");
    const int max_col = std::max({tow_col, fix_col, x_col, y_col, z_col});

    std::vector<AnchorEpoch> epochs;
    while (std::getline(input, line)) {
        const auto fields = splitCsv(line);
        if (static_cast<int>(fields.size()) <= max_col) continue;
        AnchorEpoch epoch;
        epoch.tow = std::stod(fields[tow_col]);
        epoch.anchor = fields[fix_col] == "1" && !fields[x_col].empty() &&
                       !fields[y_col].empty() && !fields[z_col].empty();
        if (epoch.anchor) {
            epoch.ecef = Eigen::Vector3d(std::stod(fields[x_col]),
                                         std::stod(fields[y_col]),
                                         std::stod(fields[z_col]));
            epoch.anchor = epoch.ecef.allFinite();
        }
        epochs.push_back(epoch);
    }
    return epochs;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 5) {
        std::cerr << "Usage: gnss_safe_bridge UNION.csv IMU.csv GPS_WEEK OUT.csv\n";
        return 2;
    }
    try {
        const auto started = std::chrono::steady_clock::now();
        const auto epochs = loadAnchors(argv[1]);
        libgnss::ImuSeries imu;
        const auto imu_result = libgnss::loadImuCsv(argv[2], imu);
        if (!imu_result.ok) throw std::runtime_error(imu_result.error);
        const int gps_week = std::stoi(argv[3]);

        libgnss::LooseCouplingProcessor::Config config;
        config.nhc_enable = true;
        config.max_position_update_nis_per_observation = 0.0;
        config.max_velocity_update_nis_per_observation = 0.0;
        libgnss::LooseCouplingProcessor processor(config);
        const libgnss::ImuAxisConvention axes;

        std::ofstream output(argv[4]);
        if (!output) throw std::runtime_error("cannot open output CSV");
        output << "tow,anchor,anchor_age_s,bridge_ecef_x,bridge_ecef_y,bridge_ecef_z,"
                  "initialized,heading_converged,speed_mps,position_sigma_max_m\n";
        output << std::setprecision(15);

        std::size_t imu_cursor = 0;
        bool have_anchor = false;
        double last_anchor_tow = -std::numeric_limits<double>::infinity();
        double previous_anchor_tow = -std::numeric_limits<double>::infinity();
        Eigen::Vector3d previous_anchor = Eigen::Vector3d::Zero();
        for (const auto& epoch : epochs) {
            while (imu_cursor < imu.samples.size() &&
                   imu.samples[imu_cursor].time.tow <= epoch.tow) {
                auto sample = imu.samples[imu_cursor++];
                sample.accel_raw = axes.apply(sample.accel_raw);
                sample.gyro_raw_radps = axes.apply(sample.gyro_raw_radps);
                processor.processImuSample(sample);
            }

            if (epoch.anchor) {
                libgnss::PositionSolution solution;
                solution.time = libgnss::GNSSTime(gps_week, epoch.tow);
                solution.status = libgnss::SolutionStatus::FIXED;
                solution.position_ecef = epoch.ecef;
                solution.position_covariance =
                    Eigen::Matrix3d::Identity() * 0.0025;
                solution.num_satellites = 4;
                const double velocity_dt = epoch.tow - previous_anchor_tow;
                if (velocity_dt > 0.0 && velocity_dt <= 1.0) {
                    solution.velocity_ecef =
                        (epoch.ecef - previous_anchor) / velocity_dt;
                    solution.velocity_covariance =
                        Eigen::Matrix3d::Identity() * 0.04;
                    solution.has_velocity = true;
                }
                processor.processGnssSolution(solution);
                previous_anchor_tow = epoch.tow;
                previous_anchor = epoch.ecef;
                last_anchor_tow = epoch.tow;
                have_anchor = true;
            }

            output << epoch.tow << ',' << (epoch.anchor ? 1 : 0) << ',';
            if (have_anchor) output << epoch.tow - last_anchor_tow;
            output << ',';
            if (processor.isOriginSet()) {
                const auto solution = processor.toPositionSolution();
                output << solution.position_ecef.x() << ','
                       << solution.position_ecef.y() << ','
                       << solution.position_ecef.z();
            } else {
                output << ",,";
            }
            output << ',' << (processor.isInitialized() ? 1 : 0)
                   << ',' << (processor.isHeadingConverged() ? 1 : 0) << ',';
            if (processor.isInitialized()) {
                const auto& state = processor.state();
                output << state.nominal.velocity_enu.norm() << ','
                       << std::sqrt(state.covariance.block<3, 3>(0, 0)
                                        .diagonal().maxCoeff());
            } else {
                output << ',';
            }
            output << '\n';
        }
        const double elapsed_ms =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - started).count();
        std::cerr << "epochs=" << epochs.size()
                  << " imu_samples=" << imu.samples.size()
                  << " elapsed_ms=" << elapsed_ms
                  << " ms_per_epoch=" << elapsed_ms / epochs.size() << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
}
