#include <libgnss++/io/rinex.hpp>
#include <libgnss++/io/ubx.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
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

constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
constexpr int kDefaultSerialBaud = 115200;

void printUsage(const char* argv0) {
    std::cerr
        << "Usage: " << argv0 << " --input <file.ubx|serial://...|/dev/tty...> [options]\n"
        << "Options:\n"
        << "  --limit <count>           Stop after this many decoded UBX messages (0 = all)\n"
        << "  --decode-nav             Print decoded NAV-PVT summaries\n"
        << "  --decode-observations    Print decoded RXM-RAWX summaries\n"
        << "  --obs-rinex-out <file>   Export decoded RAWX epochs to a simple RINEX observation file\n"
        << "  --quiet                  Suppress per-message type lines\n"
        << "  --help                   Show this help text\n";
}

libgnss::io::RINEXReader::RINEXHeader makeObservationHeader(
    const libgnss::io::UBXDecoder& decoder) {
    libgnss::io::RINEXReader::RINEXHeader header;
    header.version = 3.04;
    header.file_type = libgnss::io::RINEXReader::FileType::OBSERVATION;
    header.satellite_system = "M";
    header.program = "libgnss++";
    header.run_by = "gnss ubx-info";
    header.observation_types = {"C1C", "L1C", "D1C", "S1C"};
    if (decoder.hasLastNavPVT()) {
        const auto nav = decoder.getLastNavPVT();
        if (nav.valid_position) {
            header.approximate_position = nav.position_ecef;
        }
    }
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

}  // namespace

int main(int argc, char** argv) {
    std::string input_path;
    std::string obs_rinex_out;
    size_t limit = 0;
    bool decode_nav = false;
    bool decode_observations = false;
    bool quiet = false;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--input" && i + 1 < argc) {
            input_path = argv[++i];
        } else if (arg == "--limit" && i + 1 < argc) {
            limit = static_cast<size_t>(std::stoull(argv[++i]));
        } else if (arg == "--decode-nav") {
            decode_nav = true;
        } else if (arg == "--decode-observations") {
            decode_observations = true;
        } else if (arg == "--obs-rinex-out" && i + 1 < argc) {
            obs_rinex_out = argv[++i];
        } else if (arg == "--quiet") {
            quiet = true;
        } else if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return 0;
        } else {
            std::cerr << "Error: unknown or incomplete argument: " << arg << "\n";
            printUsage(argv[0]);
            return 1;
        }
    }

    if (input_path.empty()) {
        std::cerr << "Error: --input is required\n";
        printUsage(argv[0]);
        return 1;
    }

    UbxInputSource input_source;
    if (!openUbxInputSource(input_path, input_source)) {
        std::cerr << "Error: failed to open UBX source: " << input_path << "\n";
        return 1;
    }

    libgnss::io::UBXStreamDecoder decoder;

    libgnss::io::RINEXWriter obs_writer;
    bool obs_writer_open = false;
    size_t processed_messages = 0;
    size_t nav_pvt_count = 0;
    size_t rawx_count = 0;
    size_t exported_obs_epochs = 0;
    std::vector<libgnss::io::UBXStreamDecoder::Event> events;

    while ((limit == 0 || processed_messages < limit) &&
           readNextUbxEvents(decoder, input_source, events)) {
        for (const auto& event : events) {
            if (!event.has_message) {
                continue;
            }
            if (limit != 0 && processed_messages >= limit) {
                break;
            }
            ++processed_messages;

            if (!quiet) {
                std::cout << std::setw(5) << processed_messages << " "
                          << libgnss::io::ubx_utils::getMessageName(
                                 event.message.message_class, event.message.message_id)
                          << " (class=0x" << std::hex << std::setw(2) << std::setfill('0')
                          << static_cast<int>(event.message.message_class)
                          << ", id=0x" << std::setw(2)
                          << static_cast<int>(event.message.message_id)
                          << std::dec << std::setfill(' ') << ")\n";
            }

            if ((decode_nav || !obs_rinex_out.empty()) && event.has_nav_pvt) {
                ++nav_pvt_count;
                if (decode_nav) {
                    std::cout << "  nav: fix_type=" << static_cast<int>(event.nav_pvt.fix_type)
                              << " carrier=" << static_cast<int>(event.nav_pvt.carrier_solution)
                              << " sats=" << static_cast<int>(event.nav_pvt.num_sv)
                              << " lat_deg=" << std::fixed << std::setprecision(7)
                              << event.nav_pvt.position_geodetic.latitude * kRadToDeg
                              << " lon_deg=" << event.nav_pvt.position_geodetic.longitude * kRadToDeg
                              << " height_m=" << std::setprecision(3)
                              << event.nav_pvt.position_geodetic.height << "\n";
                }
                continue;
            }

            if ((decode_observations || !obs_rinex_out.empty()) && event.has_observation) {
                ++rawx_count;
                if (decode_observations) {
                    std::cout << "  obs: week=" << event.observation.time.week
                              << " tow=" << std::fixed << std::setprecision(3)
                              << event.observation.time.tow
                              << " sats=" << event.observation.getNumSatellites()
                              << " obs=" << event.observation.observations.size() << "\n";
                }

                if (!obs_rinex_out.empty()) {
                    if (!obs_writer_open) {
                        if (!obs_writer.createObservationFile(
                                obs_rinex_out, makeObservationHeader(decoder.getDecoder()))) {
                            std::cerr << "Error: failed to create observation RINEX file: "
                                      << obs_rinex_out << "\n";
                            closeUbxInputSource(input_source);
                            return 1;
                        }
                        obs_writer_open = true;
                    }
                    if (!obs_writer.writeObservationEpoch(event.observation)) {
                        std::cerr << "Error: failed to write observation epoch to: "
                                  << obs_rinex_out << "\n";
                        closeUbxInputSource(input_source);
                        return 1;
                    }
                    ++exported_obs_epochs;
                }
            }
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
              << " nav_pvt=" << nav_pvt_count
              << " rawx=" << rawx_count
              << " exported_obs_epochs=" << exported_obs_epochs << "\n";
    return 0;
}
