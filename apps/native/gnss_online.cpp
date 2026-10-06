#include <libgnss++/fusion/online_pva_csv.hpp>
#include <libgnss++/io/rtcm.hpp>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>
#include <stdexcept>

using namespace libgnss;
namespace {
GNSSTime readTime(std::istream& in) {
    GNSSTime time;
    if (!(in >> time.week >> time.tow)) throw std::invalid_argument("expected GPST week tow");
    return time;
}
void requireEnd(std::istream& in) {
    std::string extra;
    if (in >> extra) throw std::invalid_argument("unexpected trailing field");
}
std::vector<std::uint8_t> hexBytes(const std::string& hex) {
    if (hex.size() < 12 || hex.size() > 2058 || hex.size() % 2)
        throw std::invalid_argument("expected one complete bounded RTCM3 frame");
    std::vector<std::uint8_t> bytes;
    for (std::size_t i = 0; i < hex.size(); i += 2) {
        auto nibble = [](char c) -> int {
            if (c >= '0' && c <= '9') return c - '0';
            if (c >= 'a' && c <= 'f') return c - 'a' + 10;
            if (c >= 'A' && c <= 'F') return c - 'A' + 10;
            throw std::invalid_argument("nonhexadecimal RTCM frame");
        };
        bytes.push_back(static_cast<std::uint8_t>(16 * nibble(hex[i]) + nibble(hex[i + 1])));
    }
    if (bytes[0] != 0xd3 || (bytes[1] & 0xfc) != 0 ||
        (static_cast<std::size_t>((bytes[1] & 3) * 256 + bytes[2]) + 6) != bytes.size())
        throw std::invalid_argument("RTCM3 framing/length mismatch");
    return bytes;
}
io::RTCMMessage decodeFrame(io::RTCMProcessor& decoder, const std::string& hex) {
    const auto bytes = hexBytes(hex);
    const auto messages = decoder.decode(bytes.data(), bytes.size());
    if (messages.size() != 1 || !messages.front().valid)
        throw std::invalid_argument("RTCM frame failed CRC or decoding");
    return messages.front();
}
ObservationData readEpoch(std::istream& in, io::RTCMProcessor& decoder, const GNSSTime& time) {
    ObservationData merged(time);
    merged.receiver_position = Vector3d::Zero();
    std::set<std::pair<SatelliteId, SignalType>> identities;
    std::string hex;
    int frames = 0;
    while (in >> hex) {
        if (++frames > 32) throw std::invalid_argument("too many frames in observation epoch");
        const auto message = decodeFrame(decoder, hex);
        ObservationData part;
        if (!decoder.decodeObservationData(message, part))
            throw std::invalid_argument("expected an RTCM observation message");
        // The envelope supplies the full GPS week. Decoder TOW is still checked
        // so an input adapter cannot quietly relabel a different/future epoch.
        if (std::abs(part.time.tow - time.tow) > 0.001)
            throw std::invalid_argument("RTCM observation TOW differs from epoch envelope");
        for (const auto& row : part.observations) {
            if (!identities.insert({row.satellite, row.signal}).second)
                throw std::invalid_argument("duplicate satellite/signal in epoch frames");
            merged.addObservation(row);
        }
    }
    if (!frames) throw std::invalid_argument("empty observation epoch");
    return merged;
}
void openNew(std::ofstream& out, const std::string& path) {
    if (path.empty()) return;
    if (std::filesystem::exists(path)) throw std::invalid_argument("output already exists: " + path);
    out.open(path);
    if (!out) throw std::runtime_error("cannot open output: " + path);
    Solution::writeHeader(out);
    out << "% causal_received_events=1 base_alignment=exact output_frame=antenna\n";
}
}

int main(int argc, char** argv) {
    std::size_t line_number = 0;
    try {
        OnlineRtkImuProcessor::Config config;
        std::string rtk_path, fused_path;
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            auto number = [&]() { if (++i >= argc) throw std::invalid_argument("missing option value");
                                 return std::stod(argv[i]); };
            if (arg == "--help") {
                std::cout << "gnss_online --base-ecef X Y Z [--lever-arm X Y Z] [--loose-only]\n"
                    "  [--rtk-out NEW.pos] [--fused-out NEW.pos] [--max-imu-gap SEC]\n"
                    "Reads one received event per stdin line, flushes one CSV output per ROVER.\n"
                    "NAV recv_week recv_tow RTCM3_hex\n"
                    "BASE|ROVER recv_week recv_tow epoch_week epoch_tow RTCM3_hex [RTCM3_hex ...]\n"
                    "IMU recv_week recv_tow sample_week sample_tow ax ay az gx gy gz\n"
                    "RESET recv_week recv_tow\n"
                    "IMU units: m/s^2 and rad/s, body Forward/Left/Up. All times: GPST.\n"
                    "An epoch line contains all its constellation/signal frames. No EOF batching.\n";
                return 0;
            } else if (arg == "--base-ecef") {
                for (int axis = 0; axis < 3; ++axis) config.base_position_ecef(axis) = number();
            } else if (arg == "--lever-arm") {
                for (int axis = 0; axis < 3; ++axis) config.fusion.lever_arm_body(axis) = number();
            } else if (arg == "--loose-only") config.tight_time_update = false;
            else if (arg == "--max-imu-gap") config.max_imu_gap_s = number();
            else if (arg == "--rtk-out" || arg == "--fused-out") {
                if (++i >= argc) throw std::invalid_argument("missing output path");
                (arg == "--rtk-out" ? rtk_path : fused_path) = argv[i];
            } else throw std::invalid_argument("unknown option: " + arg);
        }
        OnlineRtkImuProcessor processor(config);
        io::RTCMProcessor decoder;
        std::ofstream rtk_out, fused_out;
        openNew(rtk_out, rtk_path);
        openNew(fused_out, fused_path);
        std::cout << "% causal_received_events=1 base_alignment=exact output_frame=antenna online_csv_schema=2 attitude_frame=body_FLU_to_local_ENU\n"
            << kOnlinePvaCsvHeader << '\n' << std::flush;
        std::string line;
        while (std::getline(std::cin, line)) {
            ++line_number;
            if (line.size() > 70000) throw std::invalid_argument("event line capacity exceeded");
            std::istringstream in(line);
            std::string kind;
            if (!(in >> kind) || kind[0] == '#') continue;
            const auto received = readTime(in);
            decoder.setReferenceTime(received);
            if (kind == "IMU") {
                ImuSample sample;
                sample.time = readTime(in);
                for (int axis = 0; axis < 3; ++axis)
                    if (!(in >> sample.accel_raw(axis))) throw std::invalid_argument("missing acceleration");
                for (int axis = 0; axis < 3; ++axis)
                    if (!(in >> sample.gyro_raw_radps(axis))) throw std::invalid_argument("missing angular rate");
                requireEnd(in);
                processor.pushImu(sample, received);
            } else if (kind == "NAV") {
                std::string hex;
                if (!(in >> hex)) throw std::invalid_argument("missing NAV frame");
                requireEnd(in);
                NavigationData nav;
                if (!decoder.decodeNavigationData(decodeFrame(decoder, hex), nav))
                    throw std::invalid_argument("expected supported RTCM broadcast ephemeris");
                processor.pushNavigation(nav, received);
            } else if (kind == "BASE" || kind == "ROVER") {
                const auto time = readTime(in);
                const auto obs = readEpoch(in, decoder, time);
                if (kind == "BASE") processor.pushBase(obs, received);
                else {
                    const auto out = processor.processRover(obs, received);
                    writeOnlinePvaCsv(std::cout, time, out);
                    std::cout << std::flush;
                    if (rtk_out && out.rtk.isValid()) { Solution::appendSolutionLine(rtk_out, out.rtk); rtk_out.flush(); }
                    if (fused_out && out.fused.isValid()) { Solution::appendSolutionLine(fused_out, out.fused); fused_out.flush(); }
                }
            } else if (kind == "RESET") { requireEnd(in); processor.reset(received); decoder.clear(); }
            else throw std::invalid_argument("unknown event kind: " + kind);
        }
        if (!std::cin.eof()) throw std::runtime_error("stdin read failed");
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "gnss_online: line " << line_number << ": " << error.what() << '\n';
        return 2;
    }
}
