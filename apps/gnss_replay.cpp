#include <Eigen/Dense>

#include <cmath>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <string>
#include <vector>

#include <libgnss++/algorithms/rtk.hpp>
#include <libgnss++/core/constants.hpp>
#include <libgnss++/io/rinex.hpp>
#include <libgnss++/io/rtcm.hpp>
#include <libgnss++/io/solution_writer.hpp>
#include <libgnss++/io/ubx.hpp>

namespace {

constexpr double kExactTimeToleranceSeconds = 1e-3;

enum class ModeChoice {
    KINEMATIC,
    STATIC,
    MOVING_BASE
};

enum class GlonassARChoice {
    OFF,
    ON,
    AUTOCAL
};

struct ReplayConfig {
    std::string rover_rinex_path;
    std::string rover_ubx_path;
    std::string base_rinex_path;
    std::string base_rtcm_path;
    std::string nav_rinex_path;
    std::string output_pos_path = "output/replay_solution.pos";
    libgnss::io::SolutionWriter::Format output_format = libgnss::io::SolutionWriter::Format::POS;
    ModeChoice mode = ModeChoice::KINEMATIC;
    bool verbose = false;
    bool quiet = false;
    int max_epochs = -1;
    size_t rtcm_message_limit = 0;
    bool base_position_override = false;
    Eigen::Vector3d base_position_ecef = Eigen::Vector3d::Zero();
    double ratio_threshold = 3.0;
    bool enable_ar_filter = false;
    double ar_filter_margin = 0.25;
    int min_satellites_for_ar = 5;
    double elevation_mask_deg = 15.0;
    bool enable_glonass = true;
    bool enable_beidou = true;
    GlonassARChoice glonass_ar = GlonassARChoice::OFF;
};

double timeDiffSeconds(const libgnss::GNSSTime& a, const libgnss::GNSSTime& b) {
    return a - b;
}

uint32_t crc24q(const uint8_t* data, size_t length) {
    static const uint32_t table[256] = {
        0x000000, 0x864CFB, 0x8AD50D, 0x0C99F6, 0x93E6E1, 0x15AA1A, 0x1933EC, 0x9F7F17,
        0xA18139, 0x27CDC2, 0x2B5434, 0xAD18CF, 0x3267D8, 0xB42B23, 0xB8B2D5, 0x3EFE2E,
        0xC54E89, 0x430272, 0x4F9B84, 0xC9D77F, 0x56A868, 0xD0E493, 0xDC7D65, 0x5A319E,
        0x64CFB0, 0xE2834B, 0xEE1ABD, 0x685646, 0xF72951, 0x7165AA, 0x7DFC5C, 0xFBB0A7,
        0x0CD1E9, 0x8A9D12, 0x8604E4, 0x00481F, 0x9F3708, 0x197BF3, 0x15E205, 0x93AEFE,
        0xAD50D0, 0x2B1C2B, 0x2785DD, 0xA1C926, 0x3EB631, 0xB8FACA, 0xB4633C, 0x322FC7,
        0xC99F60, 0x4FD39B, 0x434A6D, 0xC50696, 0x5A7981, 0xDC357A, 0xD0AC8C, 0x56E077,
        0x681E59, 0xEE52A2, 0xE2CB54, 0x6487AF, 0xFBF8B8, 0x7DB443, 0x712DB5, 0xF7614E,
        0x19A3D2, 0x9FEF29, 0x9376DF, 0x153A24, 0x8A4533, 0x0C09C8, 0x00903E, 0x86DCC5,
        0xB822EB, 0x3E6E10, 0x32F7E6, 0xB4BB1D, 0x2BC40A, 0xAD88F1, 0xA11107, 0x275DFC,
        0xDCED5B, 0x5AA1A0, 0x563856, 0xD074AD, 0x4F0BBA, 0xC94741, 0xC5DEB7, 0x43924C,
        0x7D6C62, 0xFB2099, 0xF7B96F, 0x71F594, 0xEE8A83, 0x68C678, 0x645F8E, 0xE21375,
        0x15723B, 0x933EC0, 0x9FA736, 0x19EBCD, 0x8694DA, 0x00D821, 0x0C41D7, 0x8A0D2C,
        0xB4F302, 0x32BFF9, 0x3E260F, 0xB86AF4, 0x2715E3, 0xA15918, 0xADC0EE, 0x2B8C15,
        0xD03CB2, 0x567049, 0x5AE9BF, 0xDCA544, 0x43DA53, 0xC596A8, 0xC90F5E, 0x4F43A5,
        0x71BD8B, 0xF7F170, 0xFB6886, 0x7D247D, 0xE25B6A, 0x641791, 0x688E67, 0xEEC29C,
        0x3347A4, 0xB50B5F, 0xB992A9, 0x3FDE52, 0xA0A145, 0x26EDBE, 0x2A7448, 0xAC38B3,
        0x92C69D, 0x148A66, 0x181390, 0x9E5F6B, 0x01207C, 0x876C87, 0x8BF571, 0x0DB98A,
        0xF6092D, 0x7045D6, 0x7CDC20, 0xFA90DB, 0x65EFCC, 0xE3A337, 0xEF3AC1, 0x69763A,
        0x578814, 0xD1C4EF, 0xDD5D19, 0x5B11E2, 0xC46EF5, 0x42220E, 0x4EBBF8, 0xC8F703,
        0x3F964D, 0xB9DAB6, 0xB54340, 0x330FBB, 0xAC70AC, 0x2A3C57, 0x26A5A1, 0xA0E95A,
        0x9E1774, 0x185B8F, 0x14C279, 0x928E82, 0x0DF195, 0x8BBD6E, 0x872498, 0x016863,
        0xFAD8C4, 0x7C943F, 0x700DC9, 0xF64132, 0x693E25, 0xEF72DE, 0xE3EB28, 0x65A7D3,
        0x5B59FD, 0xDD1506, 0xD18CF0, 0x57C00B, 0xC8BF1C, 0x4EF3E7, 0x426A11, 0xC426EA,
        0x2AE476, 0xACA88D, 0xA0317B, 0x267D80, 0xB90297, 0x3F4E6C, 0x33D79A, 0xB59B61,
        0x8B654F, 0x0D29B4, 0x01B042, 0x87FCB9, 0x1883AE, 0x9ECF55, 0x9256A3, 0x141A58,
        0xEFAAFF, 0x69E604, 0x657FF2, 0xE33309, 0x7C4C1E, 0xFA00E5, 0xF69913, 0x70D5E8,
        0x4E2BC6, 0xC8673D, 0xC4FECB, 0x42B230, 0xDDCD27, 0x5B81DC, 0x57182A, 0xD154D1,
        0x26359F, 0xA07964, 0xACE092, 0x2AAC69, 0xB5D37E, 0x339F85, 0x3F0673, 0xB94A88,
        0x87B4A6, 0x01F85D, 0x0D61AB, 0x8B2D50, 0x145247, 0x921EBC, 0x9E874A, 0x18CBB1,
        0xE37B16, 0x6537ED, 0x69AE1B, 0xEFE2E0, 0x709DF7, 0xF6D10C, 0xFA48FA, 0x7C0401,
        0x42FA2F, 0xC4B6D4, 0xC82F22, 0x4E63D9, 0xD11CCE, 0x575035, 0x5BC9C3, 0xDD8538
    };

    uint32_t crc = 0;
    for (size_t i = 0; i < length; ++i) {
        const uint8_t index = static_cast<uint8_t>(((crc >> 16) ^ data[i]) & 0xFFU);
        crc = (crc << 8) ^ table[index];
    }
    return crc & 0x00FFFFFFU;
}

std::vector<uint8_t> buildRtcmFrame(const libgnss::io::RTCMMessage& message) {
    std::vector<uint8_t> frame;
    frame.reserve(3 + message.data.size() + 3);
    frame.push_back(0xD3);
    frame.push_back(static_cast<uint8_t>((message.data.size() >> 8) & 0x03U));
    frame.push_back(static_cast<uint8_t>(message.data.size() & 0xFFU));
    frame.insert(frame.end(), message.data.begin(), message.data.end());
    const uint32_t crc = crc24q(frame.data(), frame.size());
    frame.push_back(static_cast<uint8_t>((crc >> 16) & 0xFFU));
    frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFFU));
    frame.push_back(static_cast<uint8_t>(crc & 0xFFU));
    return frame;
}

void mergeNavigationData(libgnss::NavigationData& dst, const libgnss::NavigationData& src) {
    for (const auto& [satellite, ephemerides] : src.ephemeris_data) {
        (void)satellite;
        for (const auto& eph : ephemerides) {
            dst.addEphemeris(eph);
        }
    }
    if (src.ionosphere_model.valid) {
        dst.ionosphere_model = src.ionosphere_model;
    }
}

std::string outputFormatString(libgnss::io::SolutionWriter::Format format) {
    switch (format) {
        case libgnss::io::SolutionWriter::Format::POS:
            return "pos";
        case libgnss::io::SolutionWriter::Format::LLH:
            return "llh";
        case libgnss::io::SolutionWriter::Format::XYZ:
            return "xyz";
    }
    return "pos";
}

void printUsage(const char* argv0) {
    std::cout
        << "Usage: " << argv0 << " [options]\n"
        << "  Rover input (choose one)\n"
        << "    --rover-rinex <file>      Rover observation RINEX\n"
        << "    --rover-ubx <file>        Rover UBX file with NAV-PVT / RXM-RAWX\n"
        << "  Base input (choose one)\n"
        << "    --base-rinex <file>       Base observation RINEX\n"
        << "    --base-rtcm <path|url>    Base RTCM file or ntrip:// source\n"
        << "  Navigation\n"
        << "    --nav-rinex <file>        Broadcast navigation RINEX (optional if RTCM carries nav)\n"
        << "  Output\n"
        << "    --out <file>              Output solution file (default: output/replay_solution.pos)\n"
        << "    --format <pos|llh|xyz>    Output format (default: pos)\n"
        << "  Solver\n"
        << "    --mode <kinematic|static|moving-base> Replay mode (default: kinematic)\n"
        << "    --ratio <value>           Ambiguity ratio threshold (default: 3.0)\n"
        << "    --arfilter                Require extra ratio margin for subset AR fixes\n"
        << "    --arfilter-margin <v>     Extra ratio margin for --arfilter (default: 0.25)\n"
        << "    --min-ar-sats <n>         Minimum satellites for AR (default: 5)\n"
        << "    --elevation-mask-deg <v>  Elevation mask in degrees (default: 15)\n"
        << "    --no-glonass              Disable GLONASS carrier processing\n"
        << "    --no-beidou               Disable BeiDou carrier processing\n"
        << "    --glonass-ar <off|on|autocal>\n"
        << "  Replay\n"
        << "    --max-epochs <n>          Stop after n rover epochs\n"
        << "    --rtcm-message-limit <n>  Stop RTCM ingest after n messages (0 = all)\n"
        << "    --base-ecef <x> <y> <z>   Override base ECEF position\n"
        << "    --quiet                   Suppress per-epoch prints\n"
        << "    --verbose                 Print per-epoch solve details\n"
        << "    -h, --help                Show this help\n";
}

[[noreturn]] void argumentError(const std::string& message, const char* argv0) {
    std::cerr << "Argument error: " << message << "\n\n";
    printUsage(argv0);
    std::exit(1);
}

ModeChoice parseMode(const std::string& value, const char* argv0) {
    if (value == "kinematic") return ModeChoice::KINEMATIC;
    if (value == "static") return ModeChoice::STATIC;
    if (value == "moving-base") return ModeChoice::MOVING_BASE;
    argumentError("unsupported --mode value: " + value, argv0);
}

GlonassARChoice parseGlonassARChoice(const std::string& value, const char* argv0) {
    if (value == "off") return GlonassARChoice::OFF;
    if (value == "on") return GlonassARChoice::ON;
    if (value == "autocal") return GlonassARChoice::AUTOCAL;
    argumentError("unsupported --glonass-ar value: " + value, argv0);
}

libgnss::io::SolutionWriter::Format parseOutputFormat(const std::string& value, const char* argv0) {
    if (value == "pos") return libgnss::io::SolutionWriter::Format::POS;
    if (value == "llh") return libgnss::io::SolutionWriter::Format::LLH;
    if (value == "xyz") return libgnss::io::SolutionWriter::Format::XYZ;
    argumentError("unsupported --format value: " + value, argv0);
}

const char* modeChoiceString(ModeChoice mode) {
    switch (mode) {
        case ModeChoice::KINEMATIC:
            return "kinematic";
        case ModeChoice::STATIC:
            return "static";
        case ModeChoice::MOVING_BASE:
            return "moving-base";
    }
    return "kinematic";
}

ReplayConfig parseArguments(int argc, char** argv) {
    ReplayConfig config;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            std::exit(0);
        } else if (arg == "--rover-rinex" && i + 1 < argc) {
            config.rover_rinex_path = argv[++i];
        } else if (arg == "--rover-ubx" && i + 1 < argc) {
            config.rover_ubx_path = argv[++i];
        } else if (arg == "--base-rinex" && i + 1 < argc) {
            config.base_rinex_path = argv[++i];
        } else if (arg == "--base-rtcm" && i + 1 < argc) {
            config.base_rtcm_path = argv[++i];
        } else if (arg == "--nav-rinex" && i + 1 < argc) {
            config.nav_rinex_path = argv[++i];
        } else if (arg == "--out" && i + 1 < argc) {
            config.output_pos_path = argv[++i];
        } else if (arg == "--format" && i + 1 < argc) {
            config.output_format = parseOutputFormat(argv[++i], argv[0]);
        } else if (arg == "--mode" && i + 1 < argc) {
            config.mode = parseMode(argv[++i], argv[0]);
        } else if (arg == "--ratio" && i + 1 < argc) {
            config.ratio_threshold = std::stod(argv[++i]);
        } else if (arg == "--arfilter") {
            config.enable_ar_filter = true;
        } else if (arg == "--arfilter-margin" && i + 1 < argc) {
            config.ar_filter_margin = std::stod(argv[++i]);
        } else if (arg == "--min-ar-sats" && i + 1 < argc) {
            config.min_satellites_for_ar = std::stoi(argv[++i]);
        } else if (arg == "--elevation-mask-deg" && i + 1 < argc) {
            config.elevation_mask_deg = std::stod(argv[++i]);
        } else if (arg == "--no-glonass") {
            config.enable_glonass = false;
        } else if (arg == "--no-beidou") {
            config.enable_beidou = false;
        } else if (arg == "--glonass-ar" && i + 1 < argc) {
            config.glonass_ar = parseGlonassARChoice(argv[++i], argv[0]);
        } else if (arg == "--max-epochs" && i + 1 < argc) {
            config.max_epochs = std::stoi(argv[++i]);
        } else if (arg == "--rtcm-message-limit" && i + 1 < argc) {
            config.rtcm_message_limit = static_cast<size_t>(std::stoull(argv[++i]));
        } else if (arg == "--base-ecef" && i + 3 < argc) {
            config.base_position_ecef =
                Eigen::Vector3d(std::stod(argv[++i]), std::stod(argv[++i]), std::stod(argv[++i]));
            config.base_position_override = true;
        } else if (arg == "--quiet") {
            config.quiet = true;
        } else if (arg == "--verbose") {
            config.verbose = true;
        } else {
            argumentError("unknown or incomplete argument: " + arg, argv[0]);
        }
    }

    const bool has_rover_rinex = !config.rover_rinex_path.empty();
    const bool has_rover_ubx = !config.rover_ubx_path.empty();
    const bool has_base_rinex = !config.base_rinex_path.empty();
    const bool has_base_rtcm = !config.base_rtcm_path.empty();
    if (has_rover_rinex == has_rover_ubx) {
        argumentError("choose exactly one of --rover-rinex or --rover-ubx", argv[0]);
    }
    if (has_base_rinex == has_base_rtcm) {
        argumentError("choose exactly one of --base-rinex or --base-rtcm", argv[0]);
    }
    if (!has_base_rtcm && config.nav_rinex_path.empty()) {
        argumentError("--nav-rinex is required when base input is not RTCM", argv[0]);
    }
    if (config.max_epochs == 0) {
        argumentError("--max-epochs must be != 0", argv[0]);
    }
    if (config.min_satellites_for_ar < 4) {
        argumentError("--min-ar-sats must be >= 4", argv[0]);
    }
    if (config.ar_filter_margin < 0.0) {
        argumentError("--arfilter-margin must be >= 0", argv[0]);
    }
    if (!config.base_rtcm_path.empty() &&
        config.base_rtcm_path.rfind("ntrip://", 0) == 0 &&
        config.rtcm_message_limit == 0) {
        argumentError("use --rtcm-message-limit with NTRIP replay sources to bound ingest", argv[0]);
    }
    return config;
}

bool loadRoverRinex(const std::string& path,
                    int max_epochs,
                    std::vector<libgnss::ObservationData>& epochs,
                    libgnss::io::RINEXReader::RINEXHeader& header) {
    libgnss::io::RINEXReader reader;
    if (!reader.open(path) || !reader.readHeader(header)) {
        return false;
    }

    libgnss::ObservationData epoch;
    while ((max_epochs < 0 || static_cast<int>(epochs.size()) < max_epochs) &&
           reader.readObservationEpoch(epoch)) {
        if (header.approximate_position.norm() > 0.0) {
            epoch.receiver_position = header.approximate_position;
        }
        epochs.push_back(epoch);
    }
    reader.close();
    return !epochs.empty();
}

bool loadRoverUbx(const std::string& path,
                  int max_epochs,
                  std::vector<libgnss::ObservationData>& epochs) {
    std::ifstream input(path, std::ios::binary);
    if (!input.is_open()) {
        return false;
    }
    const std::vector<uint8_t> buffer((std::istreambuf_iterator<char>(input)),
                                      std::istreambuf_iterator<char>());
    input.close();

    libgnss::io::UBXDecoder decoder;
    const auto messages = decoder.decode(buffer.data(), buffer.size());
    for (const auto& message : messages) {
        if (max_epochs >= 0 && static_cast<int>(epochs.size()) >= max_epochs) {
            break;
        }
        libgnss::io::UBXNavPVT nav_pvt;
        (void)decoder.decodeNavPVT(message, nav_pvt);
        libgnss::ObservationData obs_data;
        if (decoder.decodeRawx(message, obs_data)) {
            epochs.push_back(obs_data);
        }
    }
    return !epochs.empty();
}

bool loadBaseRinex(const std::string& obs_path,
                   const std::string& nav_path,
                   std::vector<libgnss::ObservationData>& base_epochs,
                   libgnss::NavigationData& nav_data,
                   Eigen::Vector3d& base_position,
                   bool& have_base_position) {
    libgnss::io::RINEXReader base_reader;
    libgnss::io::RINEXReader nav_reader;
    libgnss::io::RINEXReader::RINEXHeader base_header;

    if (!base_reader.open(obs_path) || !base_reader.readHeader(base_header)) {
        return false;
    }
    if (!nav_reader.open(nav_path) || !nav_reader.readNavigationData(nav_data)) {
        return false;
    }

    libgnss::ObservationData epoch;
    while (base_reader.readObservationEpoch(epoch)) {
        base_epochs.push_back(epoch);
    }
    base_reader.close();
    nav_reader.close();

    if (base_header.approximate_position.norm() > 0.0) {
        base_position = base_header.approximate_position;
        have_base_position = true;
    }
    return !base_epochs.empty();
}

bool loadBaseRtcm(const std::string& source,
                  size_t message_limit,
                  std::vector<libgnss::ObservationData>& base_epochs,
                  libgnss::NavigationData& nav_data,
                  Eigen::Vector3d& base_position,
                  bool& have_base_position) {
    libgnss::io::RTCMReader reader;
    if (!reader.open(source)) {
        return false;
    }

    libgnss::io::RTCMProcessor processor;
    size_t message_count = 0;
    libgnss::io::RTCMMessage raw_message;
    while ((message_limit == 0 || message_count < message_limit) && reader.readMessage(raw_message)) {
        ++message_count;
        const auto frame = buildRtcmFrame(raw_message);
        const auto decoded_messages = processor.decode(frame.data(), frame.size());
        for (const auto& message : decoded_messages) {
            if (libgnss::io::rtcm_utils::isObservationMessage(message.type)) {
                libgnss::ObservationData obs_data;
                if (processor.decodeObservationData(message, obs_data)) {
                    base_epochs.push_back(obs_data);
                }
            } else if (libgnss::io::rtcm_utils::isEphemerisMessage(message.type)) {
                libgnss::NavigationData increment;
                if (processor.decodeNavigationData(message, increment)) {
                    mergeNavigationData(nav_data, increment);
                }
            }
        }
    }
    if (processor.hasReferencePosition()) {
        base_position = processor.getReferencePosition();
        have_base_position = true;
    }
    reader.close();
    return !base_epochs.empty();
}

bool loadNavigationRinex(const std::string& path, libgnss::NavigationData& nav_data) {
    libgnss::io::RINEXReader reader;
    if (!reader.open(path)) {
        return false;
    }
    libgnss::NavigationData loaded;
    if (!reader.readNavigationData(loaded)) {
        reader.close();
        return false;
    }
    reader.close();
    mergeNavigationData(nav_data, loaded);
    return true;
}

size_t runReplay(const ReplayConfig& config) {
    std::vector<libgnss::ObservationData> rover_epochs;
    std::vector<libgnss::ObservationData> base_epochs;
    libgnss::NavigationData nav_data;
    Eigen::Vector3d base_position = Eigen::Vector3d::Zero();
    bool have_base_position = false;
    libgnss::io::RINEXReader::RINEXHeader rover_header;

    if (!config.rover_rinex_path.empty()) {
        if (!loadRoverRinex(config.rover_rinex_path, config.max_epochs, rover_epochs, rover_header)) {
            throw std::runtime_error("failed to load rover RINEX observations");
        }
    } else if (!loadRoverUbx(config.rover_ubx_path, config.max_epochs, rover_epochs)) {
        throw std::runtime_error("failed to load rover UBX observations");
    }

    if (!config.base_rinex_path.empty()) {
        if (!loadBaseRinex(config.base_rinex_path,
                           config.nav_rinex_path,
                           base_epochs,
                           nav_data,
                           base_position,
                           have_base_position)) {
            throw std::runtime_error("failed to load base RINEX observations/navigation");
        }
    } else {
        if (!loadBaseRtcm(config.base_rtcm_path,
                          config.rtcm_message_limit,
                          base_epochs,
                          nav_data,
                          base_position,
                          have_base_position)) {
            throw std::runtime_error("failed to load base RTCM observations");
        }
        if (!config.nav_rinex_path.empty() && !loadNavigationRinex(config.nav_rinex_path, nav_data)) {
            throw std::runtime_error("failed to load supplemental navigation RINEX");
        }
    }

    if (nav_data.ephemeris_data.empty()) {
        throw std::runtime_error("no navigation data available");
    }

    if (config.base_position_override) {
        base_position = config.base_position_ecef;
        have_base_position = true;
    }
    if (!have_base_position) {
        throw std::runtime_error("base position unavailable; use --base-ecef or a source with base coordinates");
    }

    libgnss::RTKProcessor rtk;
    libgnss::RTKProcessor::RTKConfig rtk_config;
    rtk_config.position_mode =
        config.mode == ModeChoice::STATIC
            ? libgnss::RTKProcessor::RTKConfig::PositionMode::STATIC
            : (config.mode == ModeChoice::MOVING_BASE
                   ? libgnss::RTKProcessor::RTKConfig::PositionMode::MOVING_BASE
                   : libgnss::RTKProcessor::RTKConfig::PositionMode::KINEMATIC);
    rtk_config.ratio_threshold = config.ratio_threshold;
    rtk_config.ambiguity_ratio_threshold = config.ratio_threshold;
    rtk_config.enable_ar_filter = config.enable_ar_filter;
    rtk_config.ar_filter_margin = config.ar_filter_margin;
    rtk_config.min_satellites_for_ar = config.min_satellites_for_ar;
    rtk_config.elevation_mask = config.elevation_mask_deg * M_PI / 180.0;
    rtk_config.enable_glonass = config.enable_glonass;
    rtk_config.enable_beidou = config.enable_beidou;
    rtk_config.glonass_ar_mode =
        config.glonass_ar == GlonassARChoice::AUTOCAL
            ? libgnss::RTKProcessor::RTKConfig::GlonassARMode::AUTOCAL
            : (config.glonass_ar == GlonassARChoice::ON
                ? libgnss::RTKProcessor::RTKConfig::GlonassARMode::ON
                : libgnss::RTKProcessor::RTKConfig::GlonassARMode::OFF);
    rtk.setRTKConfig(rtk_config);
    rtk.setBasePosition(base_position);

    libgnss::io::SolutionWriter writer;
    if (!writer.open(config.output_pos_path, config.output_format)) {
        throw std::runtime_error("failed to open output file: " + config.output_pos_path);
    }

    size_t aligned_epochs = 0;
    size_t written_solutions = 0;
    size_t fixed_solutions = 0;
    size_t skipped_rover_epochs = 0;
    size_t base_index = 0;
    Eigen::Vector3d rover_seed = Eigen::Vector3d::Zero();
    if (!rover_epochs.empty() && rover_epochs.front().receiver_position.norm() > 0.0) {
        rover_seed = rover_epochs.front().receiver_position;
    } else if (rover_header.approximate_position.norm() > 0.0) {
        rover_seed = rover_header.approximate_position;
    } else {
        rover_seed = base_position + Eigen::Vector3d(3000.0, 0.0, 0.0);
    }

    for (size_t rover_index = 0; rover_index < rover_epochs.size(); ++rover_index) {
        auto rover_obs = rover_epochs[rover_index];
        if (rover_obs.receiver_position.norm() == 0.0) {
            rover_obs.receiver_position = rover_seed;
        } else {
            rover_seed = rover_obs.receiver_position;
        }

        while (base_index < base_epochs.size() &&
               timeDiffSeconds(base_epochs[base_index].time, rover_obs.time) < -kExactTimeToleranceSeconds) {
            ++base_index;
        }

        if (base_index >= base_epochs.size()) {
            break;
        }

        const double dt = std::abs(timeDiffSeconds(base_epochs[base_index].time, rover_obs.time));
        if (dt > kExactTimeToleranceSeconds) {
            ++skipped_rover_epochs;
            continue;
        }

        const auto solution = rtk.processRTKEpoch(rover_obs, base_epochs[base_index], nav_data);
        ++aligned_epochs;

        if (!solution.isValid()) {
            continue;
        }

        writer.writeEpoch(solution);
        ++written_solutions;
        if (solution.isFixed()) {
            ++fixed_solutions;
        }

        if (config.verbose && !config.quiet) {
            std::cout << "epoch " << aligned_epochs
                      << " tow=" << std::fixed << std::setprecision(3) << solution.time.tow
                      << " status=" << static_cast<int>(solution.status)
                      << " sats=" << solution.num_satellites
                      << " ratio=" << std::setprecision(2) << solution.ratio << "\n";
        }
    }

    writer.close();

    if (!config.quiet) {
        std::cout << "summary: rover_epochs=" << rover_epochs.size()
                  << " base_epochs=" << base_epochs.size()
                  << " mode=" << modeChoiceString(config.mode)
                  << " aligned_epochs=" << aligned_epochs
                  << " skipped_rover_epochs=" << skipped_rover_epochs
                  << " written_solutions=" << written_solutions
                  << " fixed_solutions=" << fixed_solutions
                  << " out=" << config.output_pos_path
                  << " format=" << outputFormatString(config.output_format)
                  << "\n";
    } else {
        std::cout << "summary: aligned_epochs=" << aligned_epochs
                  << " mode=" << modeChoiceString(config.mode)
                  << " written_solutions=" << written_solutions
                  << " fixed_solutions=" << fixed_solutions << "\n";
    }
    return written_solutions;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const ReplayConfig config = parseArguments(argc, argv);
        const size_t written_solutions = runReplay(config);
        return written_solutions == 0 ? 1 : 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
}
