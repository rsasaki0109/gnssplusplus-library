#include <libgnss++/io/rinex.hpp>
#include <libgnss++/io/rtcm.hpp>
#include <libgnss++/io/ubx.hpp>

#include <fstream>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

namespace {

enum class InputFormat {
    RTCM,
    UBX
};

struct ConvertConfig {
    std::string input_path;
    std::string obs_out_path;
    std::string nav_out_path;
    InputFormat format = InputFormat::RTCM;
    size_t limit = 0;
    bool quiet = false;
};

void printUsage(const char* argv0) {
    std::cerr
        << "Usage: " << argv0 << " --input <file|ntrip://...> --format <rtcm|ubx> [options]\n"
        << "Options:\n"
        << "  --obs-out <file>          Export decoded observations to a simple RINEX file\n"
        << "  --nav-out <file>          Export decoded broadcast nav to a RINEX nav file\n"
        << "  --limit <count>           Stop after this many decoded messages (0 = all)\n"
        << "  --quiet                   Suppress per-message type lines\n"
        << "  --help                    Show this help text\n";
}

ConvertConfig parseArguments(int argc, char** argv) {
    ConvertConfig config;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--input" && i + 1 < argc) {
            config.input_path = argv[++i];
        } else if (arg == "--obs-out" && i + 1 < argc) {
            config.obs_out_path = argv[++i];
        } else if (arg == "--nav-out" && i + 1 < argc) {
            config.nav_out_path = argv[++i];
        } else if (arg == "--format" && i + 1 < argc) {
            const std::string value = argv[++i];
            if (value == "rtcm") {
                config.format = InputFormat::RTCM;
            } else if (value == "ubx") {
                config.format = InputFormat::UBX;
            } else {
                throw std::invalid_argument("unsupported --format value: " + value);
            }
        } else if (arg == "--limit" && i + 1 < argc) {
            config.limit = static_cast<size_t>(std::stoull(argv[++i]));
        } else if (arg == "--quiet") {
            config.quiet = true;
        } else if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            std::exit(0);
        } else {
            throw std::invalid_argument("unknown or incomplete argument: " + arg);
        }
    }

    if (config.input_path.empty()) {
        throw std::invalid_argument("--input is required");
    }
    if (config.obs_out_path.empty() && config.nav_out_path.empty()) {
        throw std::invalid_argument("provide at least one of --obs-out or --nav-out");
    }
    if (config.format == InputFormat::UBX && !config.nav_out_path.empty()) {
        throw std::invalid_argument("UBX navigation export is not supported yet; omit --nav-out");
    }
    return config;
}

libgnss::io::RINEXReader::RINEXHeader makeObservationHeader() {
    libgnss::io::RINEXReader::RINEXHeader header;
    header.version = 3.04;
    header.file_type = libgnss::io::RINEXReader::FileType::OBSERVATION;
    header.satellite_system = "M";
    header.program = "libgnss++";
    header.run_by = "gnss convert";
    header.observation_types = {"C1C", "L1C", "D1C", "S1C"};
    return header;
}

libgnss::io::RINEXReader::RINEXHeader makeNavigationHeader() {
    libgnss::io::RINEXReader::RINEXHeader header;
    header.version = 3.04;
    header.file_type = libgnss::io::RINEXReader::FileType::NAVIGATION;
    header.satellite_system = "M";
    header.program = "libgnss++";
    header.run_by = "gnss convert";
    return header;
}

int runRTCMConversion(const ConvertConfig& config) {
    libgnss::io::RTCMReader reader;
    if (!reader.open(config.input_path)) {
        std::cerr << "Error: failed to open RTCM source: " << config.input_path << "\n";
        return 1;
    }

    libgnss::io::RTCMProcessor processor;
    libgnss::io::RINEXWriter obs_writer;
    libgnss::io::RINEXWriter nav_writer;
    bool obs_writer_open = false;
    bool nav_writer_open = false;
    size_t processed_messages = 0;
    size_t exported_obs_epochs = 0;
    size_t exported_nav_messages = 0;
    size_t skipped_nav_messages = 0;

    libgnss::io::RTCMMessage message;
    while ((config.limit == 0 || processed_messages < config.limit) && reader.readMessage(message)) {
        ++processed_messages;
        if (!config.quiet) {
            std::cout << processed_messages << " "
                      << libgnss::io::rtcm_utils::getMessageTypeName(message.type)
                      << " (" << static_cast<uint16_t>(message.type) << ")\n";
        }

        if (!config.obs_out_path.empty() &&
            libgnss::io::rtcm_utils::isObservationMessage(message.type)) {
            libgnss::ObservationData obs_data;
            if (processor.decodeObservationData(message, obs_data)) {
                if (!obs_writer_open) {
                    if (!obs_writer.createObservationFile(config.obs_out_path, makeObservationHeader())) {
                        std::cerr << "Error: failed to create observation RINEX: "
                                  << config.obs_out_path << "\n";
                        return 1;
                    }
                    obs_writer_open = true;
                }
                if (!obs_writer.writeObservationEpoch(obs_data)) {
                    std::cerr << "Error: failed to write observation epoch to "
                              << config.obs_out_path << "\n";
                    return 1;
                }
                ++exported_obs_epochs;
            }
        }

        if (!config.nav_out_path.empty() &&
            libgnss::io::rtcm_utils::isEphemerisMessage(message.type)) {
            libgnss::NavigationData nav_data;
            if (processor.decodeNavigationData(message, nav_data)) {
                if (!nav_writer_open) {
                    if (!nav_writer.createNavigationFile(config.nav_out_path, makeNavigationHeader())) {
                        std::cerr << "Error: failed to create navigation RINEX: "
                                  << config.nav_out_path << "\n";
                        return 1;
                    }
                    nav_writer_open = true;
                }
                for (const auto& [satellite, ephemerides] : nav_data.ephemeris_data) {
                    (void)satellite;
                    for (const auto& eph : ephemerides) {
                        if (nav_writer.writeNavigationMessage(eph)) {
                            ++exported_nav_messages;
                        } else {
                            ++skipped_nav_messages;
                        }
                    }
                }
            }
        }
    }

    if (obs_writer_open) {
        obs_writer.close();
    }
    if (nav_writer_open) {
        nav_writer.close();
    }
    reader.close();

    std::cout << "summary: processed_messages=" << processed_messages
              << " exported_obs_epochs=" << exported_obs_epochs
              << " exported_nav_messages=" << exported_nav_messages
              << " skipped_nav_messages=" << skipped_nav_messages << "\n";
    return 0;
}

int runUBXConversion(const ConvertConfig& config) {
    std::ifstream input(config.input_path, std::ios::binary);
    if (!input.is_open()) {
        std::cerr << "Error: failed to open UBX file: " << config.input_path << "\n";
        return 1;
    }

    const std::vector<uint8_t> buffer((std::istreambuf_iterator<char>(input)),
                                      std::istreambuf_iterator<char>());
    input.close();

    libgnss::io::UBXDecoder decoder;
    const auto messages = decoder.decode(buffer.data(), buffer.size());
    libgnss::io::RINEXWriter obs_writer;
    bool obs_writer_open = false;
    size_t processed_messages = 0;
    size_t exported_obs_epochs = 0;

    for (const auto& message : messages) {
        if (config.limit != 0 && processed_messages >= config.limit) {
            break;
        }
        ++processed_messages;
        if (!config.quiet) {
            std::cout << processed_messages << " "
                      << libgnss::io::ubx_utils::getMessageName(
                             message.message_class, message.message_id)
                      << "\n";
        }

        if (config.obs_out_path.empty()) {
            continue;
        }

        libgnss::ObservationData obs_data;
        if (decoder.decodeRawx(message, obs_data)) {
            if (!obs_writer_open) {
                auto header = makeObservationHeader();
                if (decoder.hasLastNavPVT()) {
                    const auto nav = decoder.getLastNavPVT();
                    if (nav.valid_position) {
                        header.approximate_position = nav.position_ecef;
                    }
                }
                if (!obs_writer.createObservationFile(config.obs_out_path, header)) {
                    std::cerr << "Error: failed to create observation RINEX: "
                              << config.obs_out_path << "\n";
                    return 1;
                }
                obs_writer_open = true;
            }
            if (!obs_writer.writeObservationEpoch(obs_data)) {
                std::cerr << "Error: failed to write observation epoch to "
                          << config.obs_out_path << "\n";
                return 1;
            }
            ++exported_obs_epochs;
        }
    }

    if (obs_writer_open) {
        obs_writer.close();
    }

    const auto stats = decoder.getStats();
    std::cout << "summary: processed_messages=" << processed_messages
              << " valid_messages=" << stats.valid_messages
              << " checksum_errors=" << stats.checksum_errors
              << " exported_obs_epochs=" << exported_obs_epochs << "\n";
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const ConvertConfig config = parseArguments(argc, argv);
        if (config.format == InputFormat::RTCM) {
            return runRTCMConversion(config);
        }
        return runUBXConversion(config);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        printUsage(argv[0]);
        return 1;
    }
}
