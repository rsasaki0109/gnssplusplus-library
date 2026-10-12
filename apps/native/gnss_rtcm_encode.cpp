// gnss_rtcm_encode: turn recorded RINEX observations (+ broadcast navigation)
// into an RTCM 3 byte stream plus a per-epoch index. It exists so that
// recorded public data (for example the PPC dataset) can be replayed over a
// socket by `gnss realtime-stress`; there is no receiver in the loop.
//
// The stream layout mirrors a reference-station feed: periodic 1005 station
// position, round-robin 1019/1020 broadcast ephemerides (plus 1042/1044/1045/1046
// with --eph-systems), and one observation message per constellation for every epoch. The index file has one line per
// epoch: "<tow> <week> <byte_offset> <byte_length> <obs_messages> <aux_messages>".
// The aux messages (station/ephemeris) are placed in front of the epoch they
// are attached to, so a consumer can pace the stream per epoch without
// parsing RTCM.

#include <libgnss++/core/navigation.hpp>
#include <libgnss++/core/observation.hpp>
#include <libgnss++/io/rinex.hpp>
#include <libgnss++/io/rtcm.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "rtcm_frame_common.hpp"

namespace {

using libgnss::GNSSSystem;
using libgnss::io::RTCMMessage;
using libgnss::io::RTCMMessageType;

enum class Format { MSM7, MSM5, MSM4, LEGACY };

struct Config {
    std::string obs_path;
    std::string nav_path;
    std::string out_path;
    std::string index_path;
    std::string summary_path;
    std::string rinex_out_path;
    std::string systems = "G";
    std::string eph_systems = "GR";    // constellations that get ephemeris messages
    Format format = Format::MSM7;
    double start_offset_s = 0.0;
    double duration_s = 0.0;   // 0 = until end of file
    int decimate = 1;
    double eph_interval_s = 30.0;      // 0 = no ephemerides
    double station_interval_s = 10.0;  // 0 = no station message
    bool station = false;
    bool have_station_ecef = false;
    double station_ecef[3] = {0.0, 0.0, 0.0};
    bool eph_burst = true;
};

void usage(const char* argv0) {
    std::cerr
        << "Usage: " << argv0 << " --obs <rinex> --out <file.rtcm> [options]\n"
        << "  --nav <rinex>            Broadcast navigation; enables ephemeris messages (see --eph-systems)\n"
        << "  --index <file>           Per-epoch index (tow week offset length obs_msgs aux_msgs)\n"
        << "  --summary-json <file>    Write a small JSON summary of what was encoded\n"
        << "  --rinex-out <file>       Also write the exact encoded epochs as RINEX (batch-parity input)\n"
        << "  --systems <G,R,E,J,C>    Constellations to encode (default: G)\n"
        << "  --format <msm7|msm5|msm4|legacy>  MSM level, or legacy 1004 (GPS L1/L2 only)\n"
        << "  --station                Emit 1005 using the RINEX header position (base role)\n"
        << "  --station-ecef x y z     Emit 1005 using this position\n"
        << "  --start-offset <s>       Skip the first s seconds of the file\n"
        << "  --duration <s>           Encode at most s seconds (0 = all)\n"
        << "  --decimate <n>           Keep every n-th epoch (default 1)\n"
        << "  --eph-systems <G,R,E,J,C>  Constellations that get ephemeris messages (default: G,R):\n"
        << "                           G 1019, R 1020, E 1045/1046, J 1044, C 1042. Only systems that\n"
        << "                           are also in --systems are broadcast.\n"
        << "  --eph-interval <s>       Ephemeris repeat interval (default 30, 0 = off)\n"
        << "  --station-interval <s>   Station repeat interval (default 10)\n"
        << "  --no-eph-burst           Do not emit all ephemerides before the first epoch\n";
}

void setUnsignedBits(std::vector<uint8_t>& data, int pos, int len, uint64_t value) {
    for (int i = 0; i < len; ++i) {
        const int bit_index = pos + len - 1 - i;
        const int byte_index = bit_index / 8;
        const int bit_in_byte = 7 - (bit_index % 8);
        const uint8_t mask = static_cast<uint8_t>(1U << bit_in_byte);
        if ((value >> i) & 0x01U) {
            data[byte_index] |= mask;
        } else {
            data[byte_index] &= static_cast<uint8_t>(~mask);
        }
    }
}

void setSignedBits(std::vector<uint8_t>& data, int pos, int len, int64_t value) {
    const uint64_t masked = static_cast<uint64_t>(value) & ((1ULL << len) - 1ULL);
    setUnsignedBits(data, pos, len, masked);
}

RTCMMessage buildRtcm1005(double x_m, double y_m, double z_m) {
    std::vector<uint8_t> payload(19, 0);
    int bit = 0;
    setUnsignedBits(payload, bit, 12, 1005); bit += 12;
    setUnsignedBits(payload, bit, 12, 42); bit += 12;
    setUnsignedBits(payload, bit, 6, 0); bit += 6;
    setUnsignedBits(payload, bit, 1, 1); bit += 1;
    setUnsignedBits(payload, bit, 1, 1); bit += 1;
    setUnsignedBits(payload, bit, 1, 1); bit += 1;
    setUnsignedBits(payload, bit, 1, 1); bit += 1;
    setSignedBits(payload, bit, 38, static_cast<int64_t>(std::llround(x_m * 10000.0))); bit += 38;
    setUnsignedBits(payload, bit, 1, 0); bit += 1;
    setUnsignedBits(payload, bit, 1, 0); bit += 1;
    setSignedBits(payload, bit, 38, static_cast<int64_t>(std::llround(y_m * 10000.0))); bit += 38;
    setUnsignedBits(payload, bit, 2, 0); bit += 2;
    setSignedBits(payload, bit, 38, static_cast<int64_t>(std::llround(z_m * 10000.0)));
    return RTCMMessage(RTCMMessageType::RTCM_1005, payload);
}

bool messageTypeFor(char system, Format format, RTCMMessageType& type, GNSSSystem& sys) {
    if (format == Format::LEGACY) {
        if (system != 'G') return false;
        type = RTCMMessageType::RTCM_1004;
        sys = GNSSSystem::GPS;
        return true;
    }
    const int level = format == Format::MSM7 ? 7 : (format == Format::MSM5 ? 5 : 4);
    int base = 0;
    switch (system) {
        case 'G': base = 1070; sys = GNSSSystem::GPS; break;
        case 'R': base = 1080; sys = GNSSSystem::GLONASS; break;
        case 'E': base = 1090; sys = GNSSSystem::Galileo; break;
        case 'J': base = 1110; sys = GNSSSystem::QZSS; break;
        case 'C': base = 1120; sys = GNSSSystem::BeiDou; break;
        default: return false;
    }
    type = static_cast<RTCMMessageType>(base + level);
    return true;
}

libgnss::ObservationData filterGpsLegacyPairs(const libgnss::ObservationData& input) {
    libgnss::ObservationData filtered(input.time);
    filtered.receiver_position = input.receiver_position;
    filtered.receiver_clock_bias = input.receiver_clock_bias;
    std::map<libgnss::SatelliteId, std::pair<const libgnss::Observation*, const libgnss::Observation*>> pairs;
    for (const auto& obs : input.observations) {
        if (obs.satellite.system != GNSSSystem::GPS) continue;
        auto& slot = pairs[obs.satellite];
        if (obs.signal == libgnss::SignalType::GPS_L1CA) slot.first = &obs;
        else if (obs.signal == libgnss::SignalType::GPS_L2C) slot.second = &obs;
    }
    for (const auto& [sat, pair] : pairs) {
        (void)sat;
        if (pair.first != nullptr && pair.second != nullptr) {
            filtered.addObservation(*pair.first);
            filtered.addObservation(*pair.second);
        }
    }
    return filtered;
}

}  // namespace

int main(int argc, char** argv) {
    Config cfg;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto need = [&](int n) { return i + n < argc; };
        if (arg == "-h" || arg == "--help") { usage(argv[0]); return 0; }
        else if (arg == "--obs" && need(1)) cfg.obs_path = argv[++i];
        else if (arg == "--nav" && need(1)) cfg.nav_path = argv[++i];
        else if (arg == "--out" && need(1)) cfg.out_path = argv[++i];
        else if (arg == "--index" && need(1)) cfg.index_path = argv[++i];
        else if (arg == "--summary-json" && need(1)) cfg.summary_path = argv[++i];
        else if (arg == "--rinex-out" && need(1)) cfg.rinex_out_path = argv[++i];
        else if (arg == "--systems" && need(1)) {
            cfg.systems.clear();
            for (char c : std::string(argv[++i])) {
                if (c != ',') cfg.systems.push_back(static_cast<char>(std::toupper(c)));
            }
        } else if (arg == "--format" && need(1)) {
            const std::string v = argv[++i];
            if (v == "msm7") cfg.format = Format::MSM7;
            else if (v == "msm5") cfg.format = Format::MSM5;
            else if (v == "msm4") cfg.format = Format::MSM4;
            else if (v == "legacy") cfg.format = Format::LEGACY;
            else { std::cerr << "unsupported --format " << v << "\n"; return 1; }
        } else if (arg == "--station") cfg.station = true;
        else if (arg == "--station-ecef" && need(3)) {
            const double x = std::stod(argv[++i]);
            const double y = std::stod(argv[++i]);
            const double z = std::stod(argv[++i]);
            cfg.station_ecef[0] = x; cfg.station_ecef[1] = y; cfg.station_ecef[2] = z;
            cfg.have_station_ecef = true;
            cfg.station = true;
        } else if (arg == "--start-offset" && need(1)) cfg.start_offset_s = std::stod(argv[++i]);
        else if (arg == "--duration" && need(1)) cfg.duration_s = std::stod(argv[++i]);
        else if (arg == "--decimate" && need(1)) cfg.decimate = std::max(1, std::stoi(argv[++i]));
        else if (arg == "--eph-systems" && need(1)) {
            cfg.eph_systems.clear();
            for (char c : std::string(argv[++i])) {
                if (c == ',') continue;
                c = static_cast<char>(std::toupper(c));
                if (std::string("GRECJ").find(c) == std::string::npos) {
                    std::cerr << "unsupported --eph-systems entry: " << c << " (use G,R,E,J,C)\n";
                    return 1;
                }
                if (cfg.eph_systems.find(c) == std::string::npos) cfg.eph_systems.push_back(c);
            }
            if (cfg.eph_systems.empty()) {
                std::cerr << "--eph-systems needs at least one of G,R,E,J,C\n";
                return 1;
            }
        }
        else if (arg == "--eph-interval" && need(1)) cfg.eph_interval_s = std::stod(argv[++i]);
        else if (arg == "--station-interval" && need(1)) cfg.station_interval_s = std::stod(argv[++i]);
        else if (arg == "--no-eph-burst") cfg.eph_burst = false;
        else { std::cerr << "unknown or incomplete argument: " << arg << "\n"; usage(argv[0]); return 1; }
    }
    if (cfg.obs_path.empty() || cfg.out_path.empty()) {
        usage(argv[0]);
        return 1;
    }

    struct SystemPlan { char letter; RTCMMessageType type; GNSSSystem system; };
    std::vector<SystemPlan> plans;
    for (char c : cfg.systems) {
        SystemPlan plan{c, RTCMMessageType::RTCM_UNKNOWN, GNSSSystem::UNKNOWN};
        if (!messageTypeFor(c, cfg.format, plan.type, plan.system)) {
            std::cerr << "unsupported system/format combination: " << c << "\n";
            return 1;
        }
        plans.push_back(plan);
    }

    libgnss::io::RINEXReader reader;
    libgnss::io::RINEXReader::RINEXHeader header;
    if (!reader.open(cfg.obs_path) || !reader.readHeader(header)) {
        std::cerr << "failed to open RINEX observations: " << cfg.obs_path << "\n";
        return 1;
    }

    libgnss::NavigationData nav;
    std::vector<libgnss::SatelliteId> eph_sats;
    if (!cfg.nav_path.empty()) {
        libgnss::io::RINEXReader nav_reader;
        if (!nav_reader.open(cfg.nav_path) || !nav_reader.readNavigationData(nav)) {
            std::cerr << "failed to read navigation RINEX: " << cfg.nav_path << "\n";
            return 1;
        }
        // Only broadcast ephemerides for constellations whose observations are in the stream: a
        // consumer infers the GPS week from the first ephemeris it sees, and decoded GLONASS
        // ephemerides carry a host-clock-derived week that is wrong for replayed old data.
        // --eph-systems further restricts which of those constellations get ephemeris messages.
        const auto systemLetter = [](GNSSSystem system) -> char {
            switch (system) {
                case GNSSSystem::GPS: return 'G';
                case GNSSSystem::GLONASS: return 'R';
                case GNSSSystem::Galileo: return 'E';
                case GNSSSystem::QZSS: return 'J';
                case GNSSSystem::BeiDou: return 'C';
                default: return '\0';
            }
        };
        for (const auto& [sat, list] : nav.ephemeris_data) {
            (void)list;
            const char letter = systemLetter(sat.system);
            if (letter != '\0' && cfg.systems.find(letter) != std::string::npos &&
                cfg.eph_systems.find(letter) != std::string::npos) {
                eph_sats.push_back(sat);
            }
        }
        std::sort(eph_sats.begin(), eph_sats.end());
    }

    double station[3] = {cfg.station_ecef[0], cfg.station_ecef[1], cfg.station_ecef[2]};
    if (cfg.station && !cfg.have_station_ecef) {
        if (!header.has_approximate_position) {
            std::cerr << "--station requested but RINEX header has no APPROX POSITION\n";
            return 1;
        }
        station[0] = header.approximate_position.x();
        station[1] = header.approximate_position.y();
        station[2] = header.approximate_position.z();
    }

    libgnss::io::RTCMProcessor encoder;
    std::ofstream out(cfg.out_path, std::ios::binary);
    if (!out) {
        std::cerr << "failed to open output " << cfg.out_path << "\n";
        return 1;
    }
    std::ofstream index;
    if (!cfg.index_path.empty()) {
        index.open(cfg.index_path);
        index << std::fixed;
    }

    libgnss::io::RINEXWriter rinex_writer;
    const bool write_rinex = !cfg.rinex_out_path.empty();
    if (write_rinex) {
        libgnss::io::RINEXReader::RINEXHeader out_header = header;
        out_header.program = "gnss_rtcm_encode";
        if (!rinex_writer.createObservationFile(cfg.rinex_out_path, out_header)) {
            std::cerr << "failed to create RINEX output " << cfg.rinex_out_path << "\n";
            return 1;
        }
    }

    uint64_t offset = 0;
    size_t epochs_written = 0;
    size_t obs_messages = 0;
    size_t aux_messages = 0;
    size_t ephemeris_messages = 0;
    size_t station_messages = 0;
    size_t epochs_read = 0;
    size_t empty_epochs = 0;
    bool have_first = false;
    libgnss::GNSSTime first_time;
    libgnss::GNSSTime last_time;
    double next_station_t = -1e18;
    double next_eph_t = -1e18;
    size_t eph_cursor = 0;
    double eph_budget = 0.0;
    std::map<libgnss::SatelliteId, std::pair<double, int>> last_sent;  // toe, iode

    auto frameOut = [&](const RTCMMessage& msg, std::vector<uint8_t>& sink) {
        const auto frame = libgnss_apps::buildRtcmFrame(msg);
        sink.insert(sink.end(), frame.begin(), frame.end());
    };

    auto emitEphemeris = [&](const libgnss::SatelliteId& sat, const libgnss::GNSSTime& t,
                             std::vector<uint8_t>& sink) -> bool {
        const libgnss::Ephemeris* eph = nav.getEphemeris(sat, t);
        if (eph == nullptr) return false;
        const RTCMMessage msg = encoder.encodeEphemeris(*eph);
        if (!msg.valid) return false;
        frameOut(msg, sink);
        ++ephemeris_messages;
        return true;
    };

    libgnss::ObservationData epoch;
    while (reader.readObservationEpoch(epoch)) {
        ++epochs_read;
        if (!have_first) {
            first_time = epoch.time;
            have_first = true;
        }
        const double rel = epoch.time - first_time;
        if (rel + 1e-6 < cfg.start_offset_s) continue;
        if (cfg.duration_s > 0.0 && rel - cfg.start_offset_s > cfg.duration_s + 1e-6) break;
        if (((epochs_read - 1) % static_cast<size_t>(cfg.decimate)) != 0) continue;

        std::vector<uint8_t> chunk;
        size_t chunk_aux = 0;
        size_t chunk_obs = 0;
        const double t = rel;

        if (cfg.station && cfg.station_interval_s > 0.0 && t >= next_station_t) {
            frameOut(buildRtcm1005(station[0], station[1], station[2]), chunk);
            ++chunk_aux;
            ++station_messages;
            next_station_t = t + cfg.station_interval_s;
        }
        if (!eph_sats.empty() && cfg.eph_interval_s > 0.0) {
            if (epochs_written == 0 && cfg.eph_burst) {
                for (const auto& sat : eph_sats) {
                    if (emitEphemeris(sat, epoch.time, chunk)) ++chunk_aux;
                }
                next_eph_t = t + cfg.eph_interval_s;
                eph_cursor = 0;
            } else {
                // Round-robin: spread one full cycle over eph_interval_s.
                const double dt = (epochs_written > 0) ? (epoch.time - last_time) : 1.0;
                eph_budget += static_cast<double>(eph_sats.size()) * std::max(dt, 1e-3) / cfg.eph_interval_s;
                const size_t per_epoch = static_cast<size_t>(eph_budget);
                eph_budget -= static_cast<double>(per_epoch);
                for (size_t k = 0; k < per_epoch; ++k) {
                    for (size_t tries = 0; tries < eph_sats.size(); ++tries) {
                        const auto& sat = eph_sats[eph_cursor % eph_sats.size()];
                        ++eph_cursor;
                        if (emitEphemeris(sat, epoch.time, chunk)) { ++chunk_aux; break; }
                    }
                }
            }
        }

        // One observation message per system. Per RTCM 10403.x the multiple-message bit
        // is 1 on every message of the epoch except the last one actually emitted, so
        // "last" is decided after the (possibly empty/invalid) messages are known.
        struct PendingObsMessage {
            const SystemPlan* plan;
            libgnss::ObservationData selected;
            RTCMMessage msg;
        };
        std::vector<PendingObsMessage> obs_msgs;
        for (const auto& plan : plans) {
            libgnss::ObservationData selected = epoch;
            if (cfg.format == Format::LEGACY) {
                selected = filterGpsLegacyPairs(epoch);
            } else {
                selected.observations.erase(
                    std::remove_if(selected.observations.begin(), selected.observations.end(),
                                   [&](const libgnss::Observation& o) { return o.satellite.system != plan.system; }),
                    selected.observations.end());
            }
            if (selected.observations.empty()) continue;
            RTCMMessage msg = encoder.encodeObservations(selected, plan.type);
            if (!msg.valid) continue;
            obs_msgs.push_back({&plan, std::move(selected), std::move(msg)});
        }
        for (size_t i = 0; i < obs_msgs.size(); ++i) {
            if (i + 1 < obs_msgs.size()) {
                RTCMMessage flagged = encoder.encodeObservations(
                    obs_msgs[i].selected, obs_msgs[i].plan->type, /*more_follows=*/true);
                if (flagged.valid) obs_msgs[i].msg = std::move(flagged);
            }
            frameOut(obs_msgs[i].msg, chunk);
            ++chunk_obs;
        }
        if (chunk_obs == 0) {
            ++empty_epochs;
            // keep the aux messages we promised, but do not index an epoch
            if (!chunk.empty()) {
                out.write(reinterpret_cast<const char*>(chunk.data()), static_cast<std::streamsize>(chunk.size()));
                offset += chunk.size();
                aux_messages += chunk_aux;
            }
            continue;
        }

        if (write_rinex) {
            libgnss::ObservationData sent = epoch;
            sent.observations.erase(
                std::remove_if(sent.observations.begin(), sent.observations.end(),
                               [&](const libgnss::Observation& o) {
                                   for (const auto& plan : plans) {
                                       if (o.satellite.system == plan.system) return false;
                                   }
                                   return true;
                               }),
                sent.observations.end());
            if (cfg.format == Format::LEGACY) sent = filterGpsLegacyPairs(sent);
            rinex_writer.writeObservationEpoch(sent);
        }
        if (index.is_open()) {
            index << std::setprecision(3) << epoch.time.tow << ' ' << epoch.time.week << ' ' << offset << ' '
                  << chunk.size() << ' ' << chunk_obs << ' ' << chunk_aux << '\n';
        }
        out.write(reinterpret_cast<const char*>(chunk.data()), static_cast<std::streamsize>(chunk.size()));
        offset += chunk.size();
        obs_messages += chunk_obs;
        aux_messages += chunk_aux;
        last_time = epoch.time;
        ++epochs_written;
    }
    out.close();
    if (write_rinex) rinex_writer.close();

    std::ostringstream summary;
    summary << std::fixed << std::setprecision(3)
            << "{\"obs\": \"" << cfg.obs_path << "\", \"out\": \"" << cfg.out_path << "\""
            << ", \"systems\": \"" << cfg.systems << "\""
            << ", \"epochs_read\": " << epochs_read
            << ", \"epochs_written\": " << epochs_written
            << ", \"empty_epochs\": " << empty_epochs
            << ", \"obs_messages\": " << obs_messages
            << ", \"aux_messages\": " << aux_messages
            << ", \"ephemeris_messages\": " << ephemeris_messages
            << ", \"station_messages\": " << station_messages
            << ", \"bytes\": " << offset
            << ", \"first_tow\": " << first_time.tow << ", \"first_week\": " << first_time.week
            << ", \"last_tow\": " << last_time.tow << ", \"last_week\": " << last_time.week
            << ", \"station_ecef\": [" << std::setprecision(4) << station[0] << ", " << station[1] << ", "
            << station[2] << "]}\n";
    std::cout << summary.str();
    if (!cfg.summary_path.empty()) {
        std::ofstream(cfg.summary_path) << summary.str();
    }
    return epochs_written > 0 ? 0 : 1;
}
