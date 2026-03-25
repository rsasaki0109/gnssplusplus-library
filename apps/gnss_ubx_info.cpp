#include <libgnss++/io/rinex.hpp>
#include <libgnss++/io/ubx.hpp>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <string>
#include <vector>

namespace {

constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

void printUsage(const char* argv0) {
    std::cerr
        << "Usage: " << argv0 << " --input <file.ubx> [options]\n"
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

    std::ifstream input(input_path, std::ios::binary);
    if (!input.is_open()) {
        std::cerr << "Error: failed to open UBX file: " << input_path << "\n";
        return 1;
    }

    const std::vector<uint8_t> buffer((std::istreambuf_iterator<char>(input)),
                                      std::istreambuf_iterator<char>());
    input.close();

    libgnss::io::UBXDecoder decoder;
    const auto messages = decoder.decode(buffer.data(), buffer.size());
    const auto stats = decoder.getStats();

    libgnss::io::RINEXWriter obs_writer;
    bool obs_writer_open = false;
    size_t processed_messages = 0;
    size_t nav_pvt_count = 0;
    size_t rawx_count = 0;
    size_t exported_obs_epochs = 0;

    for (const auto& message : messages) {
        if (limit != 0 && processed_messages >= limit) {
            break;
        }
        ++processed_messages;

        if (!quiet) {
            std::cout << std::setw(5) << processed_messages << " "
                      << libgnss::io::ubx_utils::getMessageName(
                             message.message_class, message.message_id)
                      << " (class=0x" << std::hex << std::setw(2) << std::setfill('0')
                      << static_cast<int>(message.message_class)
                      << ", id=0x" << std::setw(2) << static_cast<int>(message.message_id)
                      << std::dec << std::setfill(' ') << ")\n";
        }

        libgnss::io::UBXNavPVT nav_pvt;
        if ((decode_nav || !obs_rinex_out.empty()) && decoder.decodeNavPVT(message, nav_pvt)) {
            ++nav_pvt_count;
            if (decode_nav) {
                std::cout << "  nav: fix_type=" << static_cast<int>(nav_pvt.fix_type)
                          << " carrier=" << static_cast<int>(nav_pvt.carrier_solution)
                          << " sats=" << static_cast<int>(nav_pvt.num_sv)
                          << " lat_deg=" << std::fixed << std::setprecision(7)
                          << nav_pvt.position_geodetic.latitude * kRadToDeg
                          << " lon_deg=" << nav_pvt.position_geodetic.longitude * kRadToDeg
                          << " height_m=" << std::setprecision(3)
                          << nav_pvt.position_geodetic.height << "\n";
            }
            continue;
        }

        libgnss::ObservationData obs_data;
        if ((decode_observations || !obs_rinex_out.empty()) && decoder.decodeRawx(message, obs_data)) {
            ++rawx_count;
            if (decode_observations) {
                std::cout << "  obs: week=" << obs_data.time.week
                          << " tow=" << std::fixed << std::setprecision(3) << obs_data.time.tow
                          << " sats=" << obs_data.getNumSatellites()
                          << " obs=" << obs_data.observations.size() << "\n";
            }

            if (!obs_rinex_out.empty()) {
                if (!obs_writer_open) {
                    if (!obs_writer.createObservationFile(obs_rinex_out, makeObservationHeader(decoder))) {
                        std::cerr << "Error: failed to create observation RINEX file: "
                                  << obs_rinex_out << "\n";
                        return 1;
                    }
                    obs_writer_open = true;
                }
                if (!obs_writer.writeObservationEpoch(obs_data)) {
                    std::cerr << "Error: failed to write observation epoch to: "
                              << obs_rinex_out << "\n";
                    return 1;
                }
                ++exported_obs_epochs;
            }
        }
    }

    if (obs_writer_open) {
        obs_writer.close();
    }

    std::cout << "summary: processed_messages=" << processed_messages
              << " valid_messages=" << stats.valid_messages
              << " checksum_errors=" << stats.checksum_errors
              << " nav_pvt=" << nav_pvt_count
              << " rawx=" << rawx_count
              << " exported_obs_epochs=" << exported_obs_epochs << "\n";
    return 0;
}
