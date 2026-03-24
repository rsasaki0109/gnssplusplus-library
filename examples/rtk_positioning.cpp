#include <iostream>
#include <fstream>
#include <cstdlib>
#include <string>
#include <cmath>
#include <limits>
#include <map>
#include <libgnss++/gnss.hpp>
#include <libgnss++/algorithms/rtk.hpp>
#include <libgnss++/algorithms/spp.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/core/coordinates.hpp>
#include <libgnss++/io/rinex.hpp>
#include <libgnss++/models/troposphere.hpp>

namespace {

double timeDiffSeconds(const libgnss::GNSSTime& a, const libgnss::GNSSTime& b) {
    return a - b;
}

struct ObservationKey {
    libgnss::SatelliteId satellite;
    libgnss::SignalType signal;

    bool operator<(const ObservationKey& other) const {
        if (satellite < other.satellite) return true;
        if (other.satellite < satellite) return false;
        return signal < other.signal;
    }
};

std::map<ObservationKey, const libgnss::Observation*> indexObservations(
    const libgnss::ObservationData& epoch) {
    std::map<ObservationKey, const libgnss::Observation*> indexed;
    for (const auto& obs : epoch.observations) {
        indexed[{obs.satellite, obs.signal}] = &obs;
    }
    return indexed;
}

double signalWavelength(libgnss::SignalType signal) {
    switch (signal) {
        case libgnss::SignalType::GPS_L1CA:
            return libgnss::constants::GPS_L1_WAVELENGTH;
        case libgnss::SignalType::GPS_L2C:
            return libgnss::constants::GPS_L2_WAVELENGTH;
        default:
            return 0.0;
    }
}

bool calculateModeledBaseRange(const libgnss::SatelliteId& satellite,
                               const libgnss::GNSSTime& time,
                               double approx_pseudorange,
                               const libgnss::Vector3d& base_position,
                               const libgnss::NavigationData& nav,
                               double& modeled_range) {
    libgnss::Vector3d sat_pos;
    libgnss::Vector3d sat_vel;
    double sat_clk = 0.0;
    double sat_clk_drift = 0.0;

    double travel_time = approx_pseudorange > 1.0
        ? approx_pseudorange / libgnss::constants::SPEED_OF_LIGHT
        : 0.075;
    libgnss::GNSSTime tx_time = time - travel_time;
    if (!nav.calculateSatelliteState(satellite, tx_time, sat_pos, sat_vel, sat_clk, sat_clk_drift)) {
        return false;
    }

    tx_time = tx_time - sat_clk;
    if (!nav.calculateSatelliteState(satellite, tx_time, sat_pos, sat_vel, sat_clk, sat_clk_drift)) {
        return false;
    }

    auto geom = nav.calculateGeometry(base_position, sat_pos);
    if (geom.elevation <= 0.05) {
        return false;
    }

    modeled_range = libgnss::geodist(sat_pos, base_position) +
        libgnss::models::tropDelaySaastamoinen(base_position, geom.elevation);
    return std::isfinite(modeled_range);
}

bool interpolateBaseEpoch(const libgnss::ObservationData& before,
                          const libgnss::ObservationData& after,
                          const libgnss::GNSSTime& target_time,
                          const libgnss::Vector3d& base_position,
                          const libgnss::NavigationData& nav,
                          libgnss::ObservationData& interpolated_epoch) {
    constexpr double kMaxInterpolationGapSeconds = 2.0;
    const double total_dt = timeDiffSeconds(after.time, before.time);
    if (!std::isfinite(total_dt) || total_dt <= 1e-6 || total_dt > kMaxInterpolationGapSeconds) {
        return false;
    }

    const double alpha = timeDiffSeconds(target_time, before.time) / total_dt;
    if (!std::isfinite(alpha) || alpha < -1e-6 || alpha > 1.0 + 1e-6) {
        return false;
    }

    interpolated_epoch = libgnss::ObservationData(target_time);
    interpolated_epoch.receiver_position =
        before.receiver_position.norm() > 0.0 ? before.receiver_position : after.receiver_position;
    interpolated_epoch.receiver_clock_bias =
        (1.0 - alpha) * before.receiver_clock_bias + alpha * after.receiver_clock_bias;

    const auto before_obs = indexObservations(before);
    const auto after_obs = indexObservations(after);

    for (const auto& [key, obs_before_ptr] : before_obs) {
        auto after_it = after_obs.find(key);
        if (after_it == after_obs.end()) continue;

        const auto& obs_before = *obs_before_ptr;
        const auto& obs_after = *after_it->second;
        const double wavelength = signalWavelength(key.signal);
        if (wavelength <= 0.0 || !obs_before.has_pseudorange || !obs_after.has_pseudorange) {
            continue;
        }

        const double approx_target_code =
            obs_before.pseudorange + alpha * (obs_after.pseudorange - obs_before.pseudorange);

        double modeled_before = 0.0;
        double modeled_after = 0.0;
        double modeled_target = 0.0;
        if (!calculateModeledBaseRange(key.satellite, before.time, obs_before.pseudorange,
                                       base_position, nav, modeled_before) ||
            !calculateModeledBaseRange(key.satellite, after.time, obs_after.pseudorange,
                                       base_position, nav, modeled_after) ||
            !calculateModeledBaseRange(key.satellite, target_time, approx_target_code,
                                       base_position, nav, modeled_target)) {
            continue;
        }

        libgnss::Observation obs(key.satellite, key.signal);
        obs.valid = obs_before.valid && obs_after.valid;
        obs.lli = alpha < 0.5 ? obs_before.lli : obs_after.lli;
        obs.code = alpha < 0.5 ? obs_before.code : obs_after.code;
        obs.signal_strength = std::max(obs_before.signal_strength, obs_after.signal_strength);
        obs.snr = (1.0 - alpha) * obs_before.snr + alpha * obs_after.snr;
        obs.loss_of_lock = obs_before.loss_of_lock || obs_after.loss_of_lock;

        const double code_residual_before = obs_before.pseudorange - modeled_before;
        const double code_residual_after = obs_after.pseudorange - modeled_after;
        obs.pseudorange = modeled_target +
            code_residual_before + alpha * (code_residual_after - code_residual_before);
        obs.has_pseudorange = std::isfinite(obs.pseudorange);

        if (obs_before.has_carrier_phase && obs_after.has_carrier_phase &&
            (obs_before.lli & 0x01) == 0 && (obs_after.lli & 0x01) == 0 && !obs.loss_of_lock) {
            const double phase_residual_before = obs_before.carrier_phase * wavelength - modeled_before;
            const double phase_residual_after = obs_after.carrier_phase * wavelength - modeled_after;
            const double phase_residual_target =
                phase_residual_before + alpha * (phase_residual_after - phase_residual_before);
            obs.carrier_phase = (modeled_target + phase_residual_target) / wavelength;
            obs.has_carrier_phase = std::isfinite(obs.carrier_phase);
        }

        if (obs_before.has_doppler && obs_after.has_doppler) {
            obs.doppler = obs_before.doppler + alpha * (obs_after.doppler - obs_before.doppler);
            obs.has_doppler = true;
        }

        if (obs.has_pseudorange || obs.has_carrier_phase || obs.has_doppler) {
            interpolated_epoch.addObservation(obs);
        }
    }

    return !interpolated_epoch.observations.empty();
}

}  // namespace

int main(int argc, char* argv[]) {
    try {
        // Create RTK processor
        libgnss::RTKProcessor rtk_processor;
        libgnss::SPPProcessor spp_processor;
        
        // Configure RTK parameters
        libgnss::RTKProcessor::RTKConfig rtk_config;
        rtk_config.max_baseline_length = 20000.0;  // 20 km max baseline
        rtk_config.ar_mode = libgnss::RTKProcessor::RTKConfig::AmbiguityResolutionMode::CONTINUOUS;
        rtk_config.ratio_threshold = 3.0;
        rtk_config.ambiguity_ratio_threshold = 3.0;
        rtk_config.min_satellites_for_ar = 5;

        // Detect position mode from data directory argument or environment variable
        std::string data_arg = (argc > 1) ? argv[1] : "";
        const char* mode_env = std::getenv("RTK_MODE");
        std::string mode_str = mode_env ? mode_env : "";
        bool is_static = (data_arg.find("short_baseline") != std::string::npos ||
                         data_arg.find("static") != std::string::npos ||
                         mode_str == "static");
        bool is_short_baseline = (data_arg.find("short_baseline") != std::string::npos);
        if (is_static) {
            rtk_config.position_mode = libgnss::RTKProcessor::RTKConfig::PositionMode::STATIC;
            if (!is_short_baseline) {
                // Use ionosphere-free LC for long baselines (eliminates DD iono residuals)
                rtk_config.ionoopt = libgnss::RTKProcessor::RTKConfig::IonoOpt::IFLC;
                std::cout << "Position mode: STATIC (ionoopt=IFLC)" << std::endl;
            } else {
                std::cout << "Position mode: STATIC" << std::endl;
            }
        } else {
            rtk_config.position_mode = libgnss::RTKProcessor::RTKConfig::PositionMode::KINEMATIC;
            std::cout << "Position mode: KINEMATIC" << std::endl;
        }

        rtk_processor.setRTKConfig(rtk_config);

        // Set base station position (known coordinates)
        Eigen::Vector3d base_position(-3962108.7, 3381309.5, 3668678.8);  // Example ECEF coordinates
        rtk_processor.setBasePosition(base_position);
        
        // Determine data directory from command line or default
        std::string data_dir = "data";
        if (argc > 1) {
            data_dir = argv[1];
        }
        std::cout << "Data directory: " << data_dir << std::endl;

        // Read RINEX files
        libgnss::io::RINEXReader rover_reader, base_reader, nav_reader;

        std::cout << "Opening rover observation file..." << std::endl;
        if (!rover_reader.open(data_dir + "/rover.obs")) {
            std::cerr << "Error: Cannot open rover observation file" << std::endl;
            return 1;
        }
        libgnss::io::RINEXReader::RINEXHeader rover_header;
        rover_reader.readHeader(rover_header);
        std::cout << "Rover observation file opened successfully" << std::endl;

        std::cout << "Opening base observation file..." << std::endl;
        if (!base_reader.open(data_dir + "/base.obs")) {
            std::cerr << "Error: Cannot open base observation file" << std::endl;
            return 1;
        }
        libgnss::io::RINEXReader::RINEXHeader base_header;
        base_reader.readHeader(base_header);
        std::cout << "Base observation file opened successfully" << std::endl;

        // Update base position from header
        if (base_header.approximate_position.norm() > 0) {
            base_position = base_header.approximate_position;
            std::cout << "Using base position from header: " << base_position.transpose() << std::endl;
            rtk_processor.setBasePosition(base_position);
        }

        std::cout << "Opening navigation file..." << std::endl;
        if (!nav_reader.open(data_dir + "/navigation.nav")) {
            std::cerr << "Error: Cannot open navigation file" << std::endl;
            return 1;
        }
        std::cout << "Navigation file opened successfully" << std::endl;

        // Read navigation data
        std::cout << "Reading navigation data..." << std::endl;
        libgnss::NavigationData nav_data;
        nav_reader.readNavigationData(nav_data);
        std::cout << "Navigation data read successfully" << std::endl;
        std::cout << "Total ephemeris entries: " << nav_data.ephemeris_data.size() << std::endl;
        
        // Process epoch by epoch
        libgnss::Solution solution;
        libgnss::ObservationData rover_obs, base_obs;
        
        std::cout << "Processing RTK positioning..." << std::endl;

        int epoch_count = 0;
        int fixed_count = 0;
        int exact_base_epochs = 0;
        int interpolated_base_epochs = 0;
        int skipped_rover_epochs = 0;

        bool rover_ok = rover_reader.readObservationEpoch(rover_obs);
        bool base_ok = base_reader.readObservationEpoch(base_obs);
        libgnss::ObservationData previous_base_obs;
        bool has_previous_base = false;

        if (!rover_ok) {
            std::cout << "Failed to read first rover observation epoch" << std::endl;
        } else {
            std::cout << "First rover epoch: " << rover_obs.observations.size() << " observations" << std::endl;
        }

        if (!base_ok) {
            std::cout << "Failed to read first base observation epoch" << std::endl;
        } else {
            std::cout << "First base epoch: " << base_obs.observations.size() << " observations" << std::endl;
        }

        // Initialize rover position with approximate position from header or near base
        if (rover_header.approximate_position.norm() > 0) {
            rover_obs.receiver_position = rover_header.approximate_position;
        } else {
            // Use position near base station (offset by ~3km for typical baseline)
            rover_obs.receiver_position = base_position + Eigen::Vector3d(3000, 0, 0);
        }
        std::cout << "Initial rover position: " << rover_obs.receiver_position.transpose() << std::endl;

        while (rover_ok) {
            libgnss::ObservationData lower_base_obs = previous_base_obs;
            bool have_lower_base = has_previous_base;

            while (base_ok && timeDiffSeconds(base_obs.time, rover_obs.time) < -1e-6) {
                lower_base_obs = base_obs;
                have_lower_base = true;
                previous_base_obs = base_obs;
                has_previous_base = true;

                libgnss::ObservationData next_base_obs;
                base_ok = base_reader.readObservationEpoch(next_base_obs);
                if (base_ok) {
                    base_obs = std::move(next_base_obs);
                }
            }

            libgnss::ObservationData aligned_base_obs;
            const double exact_dt = base_ok ? std::abs(timeDiffSeconds(base_obs.time, rover_obs.time))
                                            : std::numeric_limits<double>::infinity();
            bool used_interpolated_base = false;

            bool have_aligned_base = false;
            if (base_ok && exact_dt <= 1e-6) {
                aligned_base_obs = base_obs;
                exact_base_epochs++;
                have_aligned_base = true;
            } else if (base_ok && have_lower_base &&
                       timeDiffSeconds(rover_obs.time, lower_base_obs.time) >= -1e-6 &&
                       timeDiffSeconds(base_obs.time, rover_obs.time) >= -1e-6 &&
                       interpolateBaseEpoch(lower_base_obs, base_obs, rover_obs.time,
                                            base_position, nav_data, aligned_base_obs)) {
                interpolated_base_epochs++;
                have_aligned_base = true;
                used_interpolated_base = true;
            } else {
                skipped_rover_epochs++;
            }

            if (!have_aligned_base) {
                Eigen::Vector3d saved_rover_pos = rover_obs.receiver_position;
                rover_ok = rover_reader.readObservationEpoch(rover_obs);
                if (rover_ok) rover_obs.receiver_position = saved_rover_pos;
                continue;
            }

            // Process RTK epoch
            auto pos_solution = rtk_processor.processRTKEpoch(rover_obs, aligned_base_obs, nav_data);
            if (used_interpolated_base && pos_solution.status == libgnss::SolutionStatus::FLOAT) {
                auto spp_solution = spp_processor.processEpoch(rover_obs, nav_data);
                if (spp_solution.isValid()) {
                    const double float_vs_spp = (pos_solution.position_ecef - spp_solution.position_ecef).norm();
                    if (!std::isfinite(float_vs_spp) || float_vs_spp > 30.0) {
                        spp_solution.status = libgnss::SolutionStatus::SPP;
                        pos_solution = spp_solution;
                    }
                }
            }

            static int debug_sol = 0;
            if (debug_sol < 3) {
                std::cout << "DEBUG: Solution status=" << static_cast<int>(pos_solution.status)
                          << ", num_sat=" << pos_solution.num_satellites
                          << ", isValid=" << pos_solution.isValid() << std::endl;
                debug_sol++;
            }

            if (pos_solution.isValid()) {
                solution.addSolution(pos_solution);

                if (pos_solution.isFixed()) {
                    fixed_count++;
                }

                epoch_count++;

                // Print position error for first few epochs and every 10th
                if (epoch_count <= 5 || epoch_count % 20 == 0) {
                    Eigen::Vector3d err = pos_solution.position_ecef - rover_header.approximate_position;
                    std::cout << "Epoch " << epoch_count
                              << " status=" << static_cast<int>(pos_solution.status)
                              << " err_xyz=(" << err(0) << "," << err(1) << "," << err(2) << ")"
                              << " norm=" << err.norm() << "m"
                              << std::endl;
                }

                // Print progress every 10 epochs
                if (epoch_count % 10 == 0) {
                    std::cout << "Processed " << epoch_count << " epochs, "
                              << fixed_count << " fixed solutions" << std::endl;
                }
            }

            // Save current receiver position before reading next epoch
            Eigen::Vector3d saved_rover_pos = rover_obs.receiver_position;

            // Read next rover epoch. Base epochs are advanced only when needed for interpolation.
            rover_ok = rover_reader.readObservationEpoch(rover_obs);

            // Restore and update receiver position for next epoch
            if (rover_ok) {
                if (pos_solution.isFixed() || pos_solution.status == libgnss::SolutionStatus::SPP) {
                    rover_obs.receiver_position = pos_solution.position_ecef;
                } else {
                    rover_obs.receiver_position = saved_rover_pos;
                }
            }
        }
        
        // Display results
        // Compute mean ECEF position for self-consistency RMS
        Eigen::Vector3d mean_pos = Eigen::Vector3d::Zero();
        int valid_count = 0;
        for (const auto& sol : solution.solutions) {
            if (sol.isValid()) {
                mean_pos += sol.position_ecef;
                valid_count++;
            }
        }
        if (valid_count > 0) mean_pos /= valid_count;

        std::cout << "  RINEX header position: " << rover_header.approximate_position.transpose() << std::endl;
        std::cout << "  Mean solution position: " << mean_pos.transpose() << std::endl;
        std::cout << "  Header vs mean diff: " << (mean_pos - rover_header.approximate_position).norm() << " m" << std::endl;

        // Use mean position as reference (self-consistency check)
        auto stats = solution.calculateStatistics(mean_pos);
        std::cout << "\nRTK Processing Results:" << std::endl;
        std::cout << "  Total epochs: " << stats.total_epochs << std::endl;
        std::cout << "  Valid solutions: " << stats.valid_solutions << std::endl;
        std::cout << "  Fixed solutions: " << stats.fixed_solutions << std::endl;
        std::cout << "  Fix rate: " << (stats.fix_rate * 100.0) << "%" << std::endl;
        std::cout << "  RMS horizontal: " << stats.rms_horizontal << " m" << std::endl;
        std::cout << "  RMS vertical: " << stats.rms_vertical << " m" << std::endl;
        std::cout << "  Base epochs used exactly: " << exact_base_epochs << std::endl;
        std::cout << "  Base epochs interpolated: " << interpolated_base_epochs << std::endl;
        std::cout << "  Rover epochs skipped: " << skipped_rover_epochs << std::endl;
        
        // Write results
        solution.writeToFile("output/rtk_solution.pos");
        solution.writeKML("output/rtk_solution.kml");
        
        std::cout << "Results written to output/ directory" << std::endl;
        
        return 0;
    }
    catch (const std::invalid_argument& e) {
        std::cerr << "Error (invalid_argument): " << e.what() << std::endl;
        std::cerr << "This likely indicates a parsing error in RINEX file format" << std::endl;
        return 1;
    }
    catch (const std::out_of_range& e) {
        std::cerr << "Error (out_of_range): " << e.what() << std::endl;
        std::cerr << "This likely indicates a string index or value out of range" << std::endl;
        return 1;
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
