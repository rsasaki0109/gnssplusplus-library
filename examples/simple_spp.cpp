#include <iostream>
#include <libgnss++/gnss.hpp>

int main() {
    try {
        // Create GNSS processor
        libgnss::GNSSProcessor processor;
        
        // Set to SPP mode
        processor.setMode(libgnss::GNSSProcessor::Mode::SPP);
        
        // Process RINEX files
        std::cout << "Processing RINEX observation and navigation files..." << std::endl;
        
        auto solution = processor.processFile("data/observation.obs", "data/navigation.nav");
        
        // Check if processing was successful
        if (solution.isEmpty()) {
            std::cerr << "Error: No valid solutions found" << std::endl;
            return 1;
        }
        
        // Display statistics
        auto stats = solution.calculateStatistics();
        std::cout << "Processing Results:" << std::endl;
        std::cout << "  Total epochs: " << stats.total_epochs << std::endl;
        std::cout << "  Valid solutions: " << stats.valid_solutions << std::endl;
        std::cout << "  Solution rate: " << (stats.availability_rate * 100.0) << "%" << std::endl;
        std::cout << "  Average PDOP: " << stats.mean_pdop << std::endl;
        std::cout << "  RMS horizontal: " << stats.rms_horizontal << " m" << std::endl;
        std::cout << "  RMS vertical: " << stats.rms_vertical << " m" << std::endl;
        
        // Write results to file
        solution.writeToFile("output/spp_solution.pos");
        solution.writeKML("output/spp_solution.kml");
        
        std::cout << "Results written to output/ directory" << std::endl;
        
        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
