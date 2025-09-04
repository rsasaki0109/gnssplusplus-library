#include <iostream>
#include <string>
#include <vector>
#include <libgnss++/gnss.hpp>
#include <libgnss++/algorithms/spp.hpp>
#include <libgnss++/algorithms/rtk.hpp>
#include <libgnss++/algorithms/ppp.hpp>

void printUsage(const char* program_name) {
    std::cout << "Usage: " << program_name << " [options] obs_file nav_file\n"
              << "\nOptions:\n"
              << "  -m, --mode <mode>        Positioning mode (spp, rtk, ppp) [default: spp]\n"
              << "  -o, --output <file>      Output file [default: solution.pos]\n"
              << "  -b, --base <file>        Base station observation file (RTK mode)\n"
              << "  -p, --precise <dir>      Precise products directory (PPP mode)\n"
              << "  -c, --config <file>      Configuration file\n"
              << "  -e, --elevation <deg>    Elevation mask in degrees [default: 15]\n"
              << "  -s, --snr <db>           SNR mask in dB-Hz [default: 35]\n"
              << "  -k, --kml <file>         Output KML file for visualization\n"
              << "  -n, --nmea <file>        Output NMEA file\n"
              << "  -v, --verbose            Verbose output\n"
              << "  -h, --help               Show this help message\n"
              << "\nExamples:\n"
              << "  " << program_name << " obs.obs nav.nav\n"
              << "  " << program_name << " -m rtk -b base.obs rover.obs nav.nav\n"
              << "  " << program_name << " -m ppp -p precise/ obs.obs nav.nav\n";
}

struct CommandLineArgs {
    std::string mode = "spp";
    std::string obs_file;
    std::string nav_file;
    std::string base_file;
    std::string output_file = "solution.pos";
    std::string config_file;
    std::string precise_dir;
    std::string kml_file;
    std::string nmea_file;
    double elevation_mask = 15.0;
    double snr_mask = 35.0;
    bool verbose = false;
};

bool parseArguments(int argc, char* argv[], CommandLineArgs& args) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        
        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            return false;
        }
        else if (arg == "-m" || arg == "--mode") {
            if (++i >= argc) {
                std::cerr << "Error: Missing mode argument\n";
                return false;
            }
            args.mode = argv[i];
        }
        else if (arg == "-o" || arg == "--output") {
            if (++i >= argc) {
                std::cerr << "Error: Missing output file argument\n";
                return false;
            }
            args.output_file = argv[i];
        }
        else if (arg == "-b" || arg == "--base") {
            if (++i >= argc) {
                std::cerr << "Error: Missing base file argument\n";
                return false;
            }
            args.base_file = argv[i];
        }
        else if (arg == "-p" || arg == "--precise") {
            if (++i >= argc) {
                std::cerr << "Error: Missing precise directory argument\n";
                return false;
            }
            args.precise_dir = argv[i];
        }
        else if (arg == "-c" || arg == "--config") {
            if (++i >= argc) {
                std::cerr << "Error: Missing config file argument\n";
                return false;
            }
            args.config_file = argv[i];
        }
        else if (arg == "-e" || arg == "--elevation") {
            if (++i >= argc) {
                std::cerr << "Error: Missing elevation argument\n";
                return false;
            }
            args.elevation_mask = std::stod(argv[i]);
        }
        else if (arg == "-s" || arg == "--snr") {
            if (++i >= argc) {
                std::cerr << "Error: Missing SNR argument\n";
                return false;
            }
            args.snr_mask = std::stod(argv[i]);
        }
        else if (arg == "-k" || arg == "--kml") {
            if (++i >= argc) {
                std::cerr << "Error: Missing KML file argument\n";
                return false;
            }
            args.kml_file = argv[i];
        }
        else if (arg == "-n" || arg == "--nmea") {
            if (++i >= argc) {
                std::cerr << "Error: Missing NMEA file argument\n";
                return false;
            }
            args.nmea_file = argv[i];
        }
        else if (arg == "-v" || arg == "--verbose") {
            args.verbose = true;
        }
        else if (arg[0] != '-') {
            if (args.obs_file.empty()) {
                args.obs_file = arg;
            } else if (args.nav_file.empty()) {
                args.nav_file = arg;
            } else {
                std::cerr << "Error: Too many positional arguments\n";
                return false;
            }
        }
        else {
            std::cerr << "Error: Unknown option " << arg << "\n";
            return false;
        }
    }
    
    if (args.obs_file.empty() || args.nav_file.empty()) {
        std::cerr << "Error: Observation and navigation files are required\n";
        return false;
    }
    
    if (args.mode == "rtk" && args.base_file.empty()) {
        std::cerr << "Error: Base station file required for RTK mode\n";
        return false;
    }
    
    return true;
}

int main(int argc, char* argv[]) {
    CommandLineArgs args;
    
    if (!parseArguments(argc, argv, args)) {
        return 1;
    }
    
    try {
        libgnss::GNSSProcessor processor;
        
        // Set positioning mode
        if (args.mode == "spp") {
            processor.setMode(libgnss::GNSSProcessor::Mode::SPP);
        } else if (args.mode == "rtk") {
            processor.setMode(libgnss::GNSSProcessor::Mode::RTK);
        } else if (args.mode == "ppp") {
            processor.setMode(libgnss::GNSSProcessor::Mode::PPP);
        } else {
            std::cerr << "Error: Invalid mode '" << args.mode << "'\n";
            return 1;
        }
        
        // Load configuration if provided
        if (!args.config_file.empty()) {
            if (!processor.loadConfig(args.config_file)) {
                std::cerr << "Warning: Could not load config file " << args.config_file << "\n";
            }
        }
        
        if (args.verbose) {
            std::cout << "Processing with mode: " << args.mode << std::endl;
            std::cout << "Observation file: " << args.obs_file << std::endl;
            std::cout << "Navigation file: " << args.nav_file << std::endl;
            if (!args.base_file.empty()) {
                std::cout << "Base station file: " << args.base_file << std::endl;
            }
        }
        
        // Process files
        auto start_time = std::chrono::high_resolution_clock::now();
        
        libgnss::Solution solution;
        if (args.mode == "rtk" && !args.base_file.empty()) {
            // RTK processing with base station
            // This would require extending the processor interface
            std::cout << "RTK processing not yet implemented in CLI\n";
            return 1;
        } else {
            solution = processor.processFile(args.obs_file, args.nav_file);
        }
        
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        
        if (solution.isEmpty()) {
            std::cerr << "Error: No valid solutions found\n";
            return 1;
        }
        
        // Calculate statistics
        auto stats = solution.calculateStatistics();
        
        // Display results
        std::cout << "\nProcessing Results:\n";
        std::cout << "  Processing time: " << duration.count() << " ms\n";
        std::cout << "  Total epochs: " << stats.total_epochs << "\n";
        std::cout << "  Valid solutions: " << stats.valid_solutions << "\n";
        std::cout << "  Solution rate: " << (stats.availability_rate * 100.0) << "%\n";
        
        if (args.mode == "rtk") {
            std::cout << "  Fixed solutions: " << stats.fixed_solutions << "\n";
            std::cout << "  Fix rate: " << (stats.fix_rate * 100.0) << "%\n";
        }
        
        std::cout << "  Average PDOP: " << stats.mean_pdop << "\n";
        std::cout << "  RMS horizontal: " << stats.rms_horizontal << " m\n";
        std::cout << "  RMS vertical: " << stats.rms_vertical << " m\n";
        std::cout << "  Average satellites: " << stats.mean_satellites << "\n";
        
        // Write output files
        if (!solution.writeToFile(args.output_file)) {
            std::cerr << "Error: Could not write solution file\n";
            return 1;
        }
        std::cout << "  Solution written to: " << args.output_file << "\n";
        
        if (!args.kml_file.empty()) {
            if (solution.writeKML(args.kml_file)) {
                std::cout << "  KML written to: " << args.kml_file << "\n";
            } else {
                std::cerr << "Warning: Could not write KML file\n";
            }
        }
        
        if (!args.nmea_file.empty()) {
            if (solution.writeNMEA(args.nmea_file)) {
                std::cout << "  NMEA written to: " << args.nmea_file << "\n";
            } else {
                std::cerr << "Warning: Could not write NMEA file\n";
            }
        }
        
        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
