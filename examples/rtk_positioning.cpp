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
        
        if (!rover_reader.open("data/rover.obs")) {
            std::cerr << "Error: Cannot open rover observation file" << std::endl;
            return 1;
        }
        
        if (!base_reader.open("data/base.obs")) {
            std::cerr << "Error: Cannot open base observation file" << std::endl;
            return 1;
        }
        
        if (!nav_reader.open("data/navigation.nav")) {
            std::cerr << "Error: Cannot open navigation file" << std::endl;
            return 1;
        }
        
        // Read navigation data
        libgnss::NavigationData nav_data;
        nav_reader.readNavigationData(nav_data);
        
        // Process epoch by epoch
        libgnss::Solution solution;
        libgnss::ObservationData rover_obs, base_obs;
        
        std::cout << "Processing RTK positioning..." << std::endl;
        
        int epoch_count = 0;
        int fixed_count = 0;
        
        while (rover_reader.readObservationEpoch(rover_obs) && 
               base_reader.readObservationEpoch(base_obs)) {
            
            // Process RTK epoch
            auto pos_solution = rtk_processor.processRTKEpoch(rover_obs, base_obs, nav_data);
            
            if (pos_solution.isValid()) {
                solution.addSolution(pos_solution);
                
                if (pos_solution.isFixed()) {
                    fixed_count++;
                }
                
                // Print progress every 100 epochs
                if (++epoch_count % 100 == 0) {
                    std::cout << "Processed " << epoch_count << " epochs, "
                              << fixed_count << " fixed solutions" << std::endl;
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
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
