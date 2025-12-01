#include <iostream>
#include <libgnss++/gnss.hpp>
#include <libgnss++/algorithms/rtk.hpp>
#include <libgnss++/io/rinex.hpp>

int main() {
    try {
        // Create RTK processor
        libgnss::RTKProcessor rtk_processor;
        
        // Configure RTK parameters
        libgnss::RTKProcessor::RTKConfig rtk_config;
        rtk_config.max_baseline_length = 20000.0;  // 20 km max baseline
        rtk_config.ar_mode = libgnss::RTKProcessor::RTKConfig::AmbiguityResolutionMode::CONTINUOUS;
        rtk_config.ratio_threshold = 3.0;
        rtk_config.min_satellites_for_ar = 5;
        
        rtk_processor.setRTKConfig(rtk_config);
        
        // Set base station position (known coordinates)
        Eigen::Vector3d base_position(-3962108.7, 3381309.5, 3668678.8);  // Example ECEF coordinates
        rtk_processor.setBasePosition(base_position);
        
        // Read RINEX files
        libgnss::io::RINEXReader rover_reader, base_reader, nav_reader;

        std::cout << "Opening rover observation file..." << std::endl;
        if (!rover_reader.open("data/rover.obs")) {
            std::cerr << "Error: Cannot open rover observation file" << std::endl;
            return 1;
        }
        libgnss::io::RINEXReader::RINEXHeader rover_header;
        rover_reader.readHeader(rover_header);
        std::cout << "Rover observation file opened successfully" << std::endl;

        std::cout << "Opening base observation file..." << std::endl;
        if (!base_reader.open("data/base.obs")) {
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
        if (!nav_reader.open("data/navigation.nav")) {
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
        auto stats = solution.calculateStatistics();
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
