/**
 * @file ubx_reader.cpp
 * @brief CLI tool: read UBX binary stream from serial port, decode, and print.
 *
 * Usage: ubx_reader /dev/ttyUSB0 [baud_rate]
 */
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <iomanip>

#include "libgnss++/io/ubx.hpp"
#include "libgnss++/io/serial_port.hpp"

using namespace libgnss;
using namespace libgnss::io;

static volatile bool g_running = true;
static void signalHandler(int) { g_running = false; }

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <device> [baud_rate]\n";
        return 1;
    }

    SerialPort::Config cfg;
    cfg.device = argv[1];
    if (argc >= 3) cfg.baud_rate = static_cast<uint32_t>(std::atoi(argv[2]));

    SerialPort port;
    if (!port.open(cfg)) {
        std::cerr << "Failed to open " << cfg.device << " at " << cfg.baud_rate << " baud\n";
        return 1;
    }

    std::cout << "Listening on " << cfg.device << " @ " << cfg.baud_rate << " baud\n";
    std::signal(SIGINT, signalHandler);

    UBXStreamDecoder stream;
    std::vector<UBXStreamDecoder::Event> events;
    uint8_t buf[4096];

    while (g_running) {
        if (!port.waitForData(100)) continue;
        ssize_t n = port.read(buf, sizeof(buf));
        if (n <= 0) continue;

        events.clear();
        stream.pushBytes(buf, static_cast<size_t>(n), events);

        for (const auto& ev : events) {
            std::cout << "[" << ubx_utils::getMessageName(ev.message.message_class,
                                                          ev.message.message_id)
                      << "] len=" << ev.message.length;

            if (ev.has_nav_pvt) {
                double lat = ev.nav_pvt.position_geodetic.latitude * 180.0 / M_PI;
                double lon = ev.nav_pvt.position_geodetic.longitude * 180.0 / M_PI;
                std::cout << std::fixed << std::setprecision(7)
                          << " lat=" << lat << " lon=" << lon
                          << " h=" << std::setprecision(3) << ev.nav_pvt.position_geodetic.height
                          << " nSV=" << static_cast<int>(ev.nav_pvt.num_sv);
            }
            if (ev.has_observation) {
                std::cout << " week=" << ev.observation.time.week
                          << " tow=" << std::fixed << std::setprecision(3) << ev.observation.time.tow
                          << " nObs=" << ev.observation.observations.size();
            }
            std::cout << "\n";
        }
    }

    auto stats = stream.getStats();
    std::cout << "\n--- Statistics ---\n"
              << "Valid messages: " << stats.valid_messages << "\n"
              << "Checksum errors: " << stats.checksum_errors << "\n";
    return 0;
}
