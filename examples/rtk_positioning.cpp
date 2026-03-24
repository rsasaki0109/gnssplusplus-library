#include <iostream>
#include <fstream>
#include <cstdlib>
#include <string>
#include <libgnss++/gnss.hpp>
#include <libgnss++/algorithms/rtk.hpp>
#include <libgnss++/io/rinex.hpp>

int main(int argc, char* argv[]) {
    try {
        // Create RTK processor
        libgnss::RTKProcessor rtk_processor;
        
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

        bool rover_ok = rover_reader.readObservationEpoch(rover_obs);
        bool base_ok = base_reader.readObservationEpoch(base_obs);

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

        while (rover_ok && base_ok) {

            // Synchronize rover and base epoch times (tolerance-based)
            {
                double time_diff = (rover_obs.time.week - base_obs.time.week) * 604800.0
                                 + (rover_obs.time.tow - base_obs.time.tow);
                while (rover_ok && base_ok && std::abs(time_diff) > 0.5) {
                    if (time_diff < 0) {
                        std::cout << "Epoch sync: advancing rover (diff=" << time_diff << "s)" << std::endl;
                        Eigen::Vector3d saved_pos = rover_obs.receiver_position;
                        rover_ok = rover_reader.readObservationEpoch(rover_obs);
                        if (rover_ok) rover_obs.receiver_position = saved_pos;
                    } else {
                        std::cout << "Epoch sync: advancing base (diff=" << time_diff << "s)" << std::endl;
                        base_ok = base_reader.readObservationEpoch(base_obs);
                    }
                    time_diff = (rover_obs.time.week - base_obs.time.week) * 604800.0
                              + (rover_obs.time.tow - base_obs.time.tow);
                }
            }
            if (!rover_ok || !base_ok) break;

            // Process RTK epoch
            auto pos_solution = rtk_processor.processRTKEpoch(rover_obs, base_obs, nav_data);

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

            // Read next epochs
            rover_ok = rover_reader.readObservationEpoch(rover_obs);
            base_ok = base_reader.readObservationEpoch(base_obs);

            // Restore and update receiver position for next epoch
            if (rover_ok) {
                if (pos_solution.isValid()) {
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
