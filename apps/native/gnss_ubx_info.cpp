#include <libgnss++/io/rinex.hpp>
#include <libgnss++/io/ubx.hpp>
#include <libgnss++/io/ubx_nav_decode.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <system_error>
#include <string>
#include <vector>

#ifndef _WIN32
#include <cerrno>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

#include "ubx_input_source.hpp"

namespace {

using libgnss_apps::UbxInputSource;
using libgnss_apps::closeUbxInputSource;
using libgnss_apps::openUbxInputSource;
using libgnss_apps::readNextUbxEvents;

constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

void printUsage(const char* argv0) {
    std::cerr
        << "Usage: " << argv0 << " --input <file.ubx|serial://...|/dev/tty...> [options]\n"
        << "Options:\n"
        << "  --limit <count>           Stop after this many decoded UBX messages (0 = all)\n"
        << "  --decode-nav             Print decoded NAV-PVT and RXM-SFRBX summaries and, before the summary\n"
        << "                           line, the number of broadcast ephemerides decoded from them per system\n"
        << "  --decode-observations    Print decoded RXM-RAWX summaries\n"
        << "  --obs-rinex-out <file>   Export decoded RAWX epochs to a RINEX 3.04 observation file\n"
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
    header.receiver_type = "u-blox";
    if (decoder.hasLastNavPVT()) {
        const auto nav = decoder.getLastNavPVT();
        if (nav.valid_position) {
            header.approximate_position = nav.position_ecef;
            header.has_approximate_position = true;
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
    size_t sfrbx_count = 0;
    size_t rawx_count = 0;
    size_t exported_obs_epochs = 0;
    libgnss::io::UBXNavDecoder nav_decoder;
    std::vector<libgnss::Ephemeris> decoded_ephemerides;
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

            if (decode_nav) {
                if (event.has_observation) {
                    nav_decoder.setReferenceTime(event.observation.time);
                }
                if (event.has_sfrbx) {
                    nav_decoder.pushSfrbx(event.sfrbx, decoded_ephemerides);
                }
            }

            if (event.has_nav_pvt) {
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

            if (event.has_sfrbx) {
                ++sfrbx_count;
                if (!decode_nav) {
                    continue;
                }
                libgnss::io::UBXSfrbxFrameInfo frame_info;
                const bool has_frame_info =
                    libgnss::io::ubx_utils::decodeSfrbxFrameInfo(event.sfrbx, frame_info);
                std::cout << "  subframe: system=";
                switch (event.sfrbx.system) {
                    case libgnss::GNSSSystem::GPS: std::cout << "GPS"; break;
                    case libgnss::GNSSSystem::GLONASS: std::cout << "GLONASS"; break;
                    case libgnss::GNSSSystem::Galileo: std::cout << "Galileo"; break;
                    case libgnss::GNSSSystem::BeiDou: std::cout << "BeiDou"; break;
                    case libgnss::GNSSSystem::QZSS: std::cout << "QZSS"; break;
                    case libgnss::GNSSSystem::NavIC: std::cout << "NavIC"; break;
                    case libgnss::GNSSSystem::SBAS: std::cout << "SBAS"; break;
                    default: std::cout << "UNKNOWN"; break;
                }
                std::cout << " sv=" << static_cast<int>(event.sfrbx.sv_id)
                          << " words=" << event.sfrbx.words.size()
                          << " kind="
                          << libgnss::io::ubx_utils::getSfrbxFrameKindName(frame_info.kind);
                if (has_frame_info) {
                    std::cout << " frame_id=" << frame_info.frame_id;
                    if (frame_info.has_page_id) {
                        std::cout << " page_id=" << frame_info.page_id;
                    }
                }
                std::cout
                          << " sig_id=" << static_cast<int>(event.sfrbx.signal_id)
                          << " freq_id=" << static_cast<int>(event.sfrbx.frequency_id)
                          << " channel=" << static_cast<int>(event.sfrbx.channel)
                          << "\n";
                continue;
            }

            if (event.has_observation) {
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

    bool obs_file_ok = true;
    if (obs_writer_open && !obs_writer.close()) {
        std::cerr << "Error: failed to write observation RINEX file: " << obs_rinex_out << "\n";
        obs_file_ok = false;
    }

    closeUbxInputSource(input_source);
    const auto stats = decoder.getStats();

    if (decode_nav) {
        std::map<libgnss::GNSSSystem, size_t> per_system;
        for (const auto& ephemeris : decoded_ephemerides) {
            ++per_system[ephemeris.satellite.system];
        }
        const auto count = [&per_system](libgnss::GNSSSystem system) {
            const auto it = per_system.find(system);
            return it == per_system.end() ? size_t{0} : it->second;
        };
        std::cout << "nav_decode: ephemerides=" << decoded_ephemerides.size()
                  << " gps=" << count(libgnss::GNSSSystem::GPS)
                  << " qzss=" << count(libgnss::GNSSSystem::QZSS)
                  << " galileo=" << count(libgnss::GNSSSystem::Galileo)
                  << " beidou=" << count(libgnss::GNSSSystem::BeiDou)
                  << " glonass=" << count(libgnss::GNSSSystem::GLONASS)
                  << " klobuchar=" << (nav_decoder.hasKlobuchar() ? 1 : 0) << "\n";
    }

    std::cout << "summary: processed_messages=" << processed_messages
              << " valid_messages=" << stats.valid_messages
              << " checksum_errors=" << stats.checksum_errors
              << " nav_pvt=" << nav_pvt_count
              << " sfrbx=" << sfrbx_count
              << " rawx=" << rawx_count
              << " exported_obs_epochs=" << exported_obs_epochs << "\n";
    return obs_file_ok ? 0 : 1;
}
