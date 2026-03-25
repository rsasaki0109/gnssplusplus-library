#include <libgnss++/io/rinex.hpp>
#include <libgnss++/io/rtcm.hpp>
#include <libgnss++/io/ubx.hpp>

#include <array>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <system_error>
#include <string>
#include <vector>

#ifndef _WIN32
#include <cerrno>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

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

constexpr int kDefaultSerialBaud = 115200;

void printUsage(const char* argv0) {
    std::cerr
        << "Usage: " << argv0
        << " --input <file|ntrip://...|serial://...|/dev/tty...> --format <rtcm|ubx> [options]\n"
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

struct UbxInputSource {
    bool serial = false;
    bool eof = false;
    std::ifstream file;
#ifndef _WIN32
    int serial_fd = -1;
#endif
};

std::string resolveSerialPath(const std::string& source) {
    constexpr const char* kPrefix = "serial://";
    if (source.rfind(kPrefix, 0) != 0) {
        return source;
    }

    std::string path = source.substr(std::char_traits<char>::length(kPrefix));
    const size_t query_pos = path.find('?');
    if (query_pos != std::string::npos) {
        path.resize(query_pos);
    }
    if (!path.empty() && path.front() != '/') {
        return path;
    }
    while (path.size() > 1 && path[0] == '/' && path[1] == '/') {
        path.erase(path.begin());
    }
    return path;
}

int parseSerialBaud(const std::string& source) {
    constexpr const char* kQuery = "?baud=";
    const size_t query_pos = source.find(kQuery);
    if (query_pos == std::string::npos) {
        return kDefaultSerialBaud;
    }
    const std::string baud_text =
        source.substr(query_pos + std::char_traits<char>::length(kQuery));
    if (baud_text.empty()) {
        return kDefaultSerialBaud;
    }
    return std::stoi(baud_text);
}

#ifndef _WIN32
speed_t baudRateConstant(int baud) {
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        default:
            throw std::invalid_argument("unsupported serial baud rate");
    }
}

bool configureSerialPort(int fd, int baud) {
    termios tio{};
    if (tcgetattr(fd, &tio) != 0) {
        return false;
    }
    cfmakeraw(&tio);
    const speed_t speed = baudRateConstant(baud);
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cc[VMIN] = 1;
    tio.c_cc[VTIME] = 0;
    return tcsetattr(fd, TCSANOW, &tio) == 0;
}
#endif

void closeUbxInputSource(UbxInputSource& source) {
    if (source.file.is_open()) {
        source.file.close();
    }
#ifndef _WIN32
    if (source.serial_fd >= 0) {
        ::close(source.serial_fd);
        source.serial_fd = -1;
    }
#endif
    source.serial = false;
    source.eof = false;
}

bool openUbxInputSource(const std::string& input_path, UbxInputSource& source) {
    closeUbxInputSource(source);

    const std::string resolved_path = resolveSerialPath(input_path);
    std::error_code ec;
    const auto status = std::filesystem::status(resolved_path, ec);
    if (!ec && std::filesystem::is_regular_file(status)) {
        source.file.open(resolved_path, std::ios::binary);
        return source.file.is_open();
    }

#ifndef _WIN32
    const int baud = parseSerialBaud(input_path);
    const int fd = ::open(resolved_path.c_str(), O_RDONLY | O_NOCTTY);
    if (fd < 0) {
        return false;
    }
    if (!configureSerialPort(fd, baud)) {
        ::close(fd);
        return false;
    }
    source.serial = true;
    source.serial_fd = fd;
    return true;
#else
    (void)input_path;
    return false;
#endif
}

bool readNextUbxEvents(libgnss::io::UBXStreamDecoder& decoder,
                       UbxInputSource& source,
                       std::vector<libgnss::io::UBXStreamDecoder::Event>& events) {
    events.clear();
    std::array<uint8_t, 4096> chunk{};

    while (true) {
        size_t bytes_read = 0;
        if (source.serial) {
#ifndef _WIN32
            const ssize_t count = ::read(source.serial_fd, chunk.data(), chunk.size());
            if (count < 0) {
                if (errno == EINTR) {
                    continue;
                }
                source.eof = true;
                return false;
            }
            if (count == 0) {
                source.eof = true;
                return false;
            }
            bytes_read = static_cast<size_t>(count);
#else
            source.eof = true;
            return false;
#endif
        } else {
            source.file.read(reinterpret_cast<char*>(chunk.data()),
                             static_cast<std::streamsize>(chunk.size()));
            bytes_read = static_cast<size_t>(source.file.gcount());
            if (bytes_read == 0) {
                source.eof = true;
                return false;
            }
        }

        decoder.pushBytes(chunk.data(), bytes_read, events);
        if (!events.empty()) {
            return true;
        }
    }
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
    UbxInputSource input_source;
    if (!openUbxInputSource(config.input_path, input_source)) {
        std::cerr << "Error: failed to open UBX source: " << config.input_path << "\n";
        return 1;
    }
    libgnss::io::UBXStreamDecoder decoder;
    libgnss::io::RINEXWriter obs_writer;
    bool obs_writer_open = false;
    size_t processed_messages = 0;
    size_t exported_obs_epochs = 0;
    std::vector<libgnss::io::UBXStreamDecoder::Event> events;

    while ((config.limit == 0 || processed_messages < config.limit) &&
           readNextUbxEvents(decoder, input_source, events)) {
        for (const auto& event : events) {
            if (!event.has_message) {
                continue;
            }
            if (config.limit != 0 && processed_messages >= config.limit) {
                break;
            }
            ++processed_messages;
            if (!config.quiet) {
                std::cout << processed_messages << " "
                          << libgnss::io::ubx_utils::getMessageName(
                                 event.message.message_class, event.message.message_id)
                          << "\n";
            }

            if (config.obs_out_path.empty() || !event.has_observation) {
                continue;
            }

            if (!obs_writer_open) {
                auto header = makeObservationHeader();
                const auto& ubx_decoder = decoder.getDecoder();
                if (ubx_decoder.hasLastNavPVT()) {
                    const auto nav = ubx_decoder.getLastNavPVT();
                    if (nav.valid_position) {
                        header.approximate_position = nav.position_ecef;
                    }
                }
                if (!obs_writer.createObservationFile(config.obs_out_path, header)) {
                    std::cerr << "Error: failed to create observation RINEX: "
                              << config.obs_out_path << "\n";
                    closeUbxInputSource(input_source);
                    return 1;
                }
                obs_writer_open = true;
            }
            if (!obs_writer.writeObservationEpoch(event.observation)) {
                std::cerr << "Error: failed to write observation epoch to "
                          << config.obs_out_path << "\n";
                closeUbxInputSource(input_source);
                return 1;
            }
            ++exported_obs_epochs;
        }
    }

    if (obs_writer_open) {
        obs_writer.close();
    }
    closeUbxInputSource(input_source);

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
