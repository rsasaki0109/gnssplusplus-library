/**
 * @file gnss_data_generator.cpp
 * @brief Generate synthetic UBX or SBF binary data and write to a pty or file.
 *
 * Creates a pseudoterminal (pty) pair, prints the slave device path,
 * then continuously writes realistic GNSS messages at configurable rate.
 *
 * Usage:
 *   gnss_data_generator ubx [rate_hz]   # UBX NAV-PVT + RXM-RAWX at rate_hz
 *   gnss_data_generator sbf [rate_hz]   # SBF PVTGeodetic at rate_hz
 */
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <thread>
#include <vector>
#include <csignal>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <termios.h>

// For openpty
#if __has_include(<pty.h>)
#include <pty.h>
#elif __has_include(<util.h>)
#include <util.h>
#endif

#include "libgnss++/io/sbf.hpp"

using namespace libgnss::io;

// ============================================================================
// Standalone UBX frame builder (no dependency on libgnss++ UBX API)
// ============================================================================

namespace {

std::vector<uint8_t> buildUbxFrame(uint8_t cls, uint8_t id,
                                   const std::vector<uint8_t>& payload) {
    uint16_t len = static_cast<uint16_t>(payload.size());
    std::vector<uint8_t> pkt;
    pkt.reserve(6 + len + 2);
    pkt.push_back(0xB5);
    pkt.push_back(0x62);
    pkt.push_back(cls);
    pkt.push_back(id);
    pkt.push_back(static_cast<uint8_t>(len & 0xFF));
    pkt.push_back(static_cast<uint8_t>((len >> 8) & 0xFF));
    pkt.insert(pkt.end(), payload.begin(), payload.end());
    uint8_t ck_a = 0, ck_b = 0;
    for (size_t i = 2; i < pkt.size(); ++i) {
        ck_a += pkt[i];
        ck_b += ck_a;
    }
    pkt.push_back(ck_a);
    pkt.push_back(ck_b);
    return pkt;
}

template<typename T>
void appendLE(std::vector<uint8_t>& v, T val) {
    const uint8_t* p = reinterpret_cast<const uint8_t*>(&val);
    v.insert(v.end(), p, p + sizeof(T));
}

}  // namespace

static volatile bool g_running = true;
static void sigHandler(int) { g_running = false; }

// ============================================================================
// UBX frame builders
// ============================================================================

static std::vector<uint8_t> buildNavPvt(double lat_deg, double lon_deg,
                                        double height_m, int num_sv,
                                        uint32_t itow_ms, uint8_t fix_type) {
    // NAV-PVT payload: 92 bytes, manually constructed (no struct dependency)
    std::vector<uint8_t> payload(92, 0);
    // iTOW at offset 0
    std::memcpy(payload.data(), &itow_ms, 4);
    // fixType at offset 20
    payload[20] = fix_type;
    // flags at offset 21: gnssFixOK
    payload[21] = 0x01;
    // numSV at offset 23
    payload[23] = static_cast<uint8_t>(num_sv);
    // lon at offset 24 (1e-7 deg)
    int32_t lon_e7 = static_cast<int32_t>(lon_deg * 1e7);
    std::memcpy(payload.data() + 24, &lon_e7, 4);
    // lat at offset 28
    int32_t lat_e7 = static_cast<int32_t>(lat_deg * 1e7);
    std::memcpy(payload.data() + 28, &lat_e7, 4);
    // height at offset 32 (mm)
    int32_t h_mm = static_cast<int32_t>(height_m * 1e3);
    std::memcpy(payload.data() + 32, &h_mm, 4);
    // hAcc at offset 40 (mm)
    uint32_t h_acc = 2500;
    std::memcpy(payload.data() + 40, &h_acc, 4);
    // vAcc at offset 44
    uint32_t v_acc = 5000;
    std::memcpy(payload.data() + 44, &v_acc, 4);
    // pDOP at offset 76 (0.01 scale)
    uint16_t pdop = 120;
    std::memcpy(payload.data() + 76, &pdop, 2);

    return buildUbxFrame(0x01, 0x07, payload);
}

static std::vector<uint8_t> buildRxmRawx(uint16_t week, double tow,
                                          int num_meas) {
    // RXM-RAWX: 16-byte header + 32 bytes per measurement
    std::vector<uint8_t> payload(16 + num_meas * 32, 0);
    // rcvTow at offset 0 (double)
    std::memcpy(payload.data(), &tow, 8);
    // week at offset 8
    std::memcpy(payload.data() + 8, &week, 2);
    // numMeas at offset 11
    payload[11] = static_cast<uint8_t>(num_meas);
    // recStat at offset 12
    payload[12] = 0x01;

    for (int i = 0; i < num_meas; ++i) {
        size_t off = 16 + i * 32;
        double pr = 20000000.0 + i * 1000000.0;
        double cp = 105000000.0 + i * 5000000.0;
        float dop = static_cast<float>(1000.0 - i * 100.0);
        std::memcpy(payload.data() + off, &pr, 8);       // prMes
        std::memcpy(payload.data() + off + 8, &cp, 8);   // cpMes
        std::memcpy(payload.data() + off + 16, &dop, 4);  // doMes

        uint8_t gnss_id, sv_id, sig_id = 0;
        if (i < 8)       { gnss_id = 0; sv_id = static_cast<uint8_t>(i + 1); }
        else if (i < 12) { gnss_id = 2; sv_id = static_cast<uint8_t>(i - 7); }
        else if (i < 15) { gnss_id = 6; sv_id = static_cast<uint8_t>(i - 11); }
        else              { gnss_id = 5; sv_id = static_cast<uint8_t>(193 + i - 15); }

        payload[off + 20] = gnss_id;
        payload[off + 21] = sv_id;
        payload[off + 22] = sig_id;
        payload[off + 26] = static_cast<uint8_t>(35 + (i % 10));  // cno
        payload[off + 30] = 0x03;  // trkStat: prValid + cpValid
    }

    return buildUbxFrame(0x02, 0x15, payload);
}

// ============================================================================
// SBF frame builder
// ============================================================================

static std::vector<uint8_t> buildSbfPvtGeodetic(double lat_rad, double lon_rad,
                                                  double height_m, int num_sv,
                                                  uint32_t tow_ms, uint16_t wnc) {
    SBFPvtGeodetic pvt{};
    pvt.mode = 1;  // StandAlone
    pvt.error = 0;
    pvt.latitude = lat_rad;
    pvt.longitude = lon_rad;
    pvt.height = height_m;
    pvt.undulation = 36.5f;
    pvt.vn = 0.5f;
    pvt.ve = -0.3f;
    pvt.vu = 0.01f;
    pvt.nr_sv = static_cast<uint8_t>(num_sv);
    pvt.h_accuracy = 250;   // 2.5 m
    pvt.v_accuracy = 500;   // 5.0 m
    pvt.rx_clk_bias = 0.001;

    // Build SBF block
    std::vector<uint8_t> payload(sizeof(SBFPvtGeodetic));
    std::memcpy(payload.data(), &pvt, sizeof(pvt));

    size_t raw_len = 14 + payload.size();
    size_t padded_len = (raw_len + 3) & ~3u;
    std::vector<uint8_t> block(padded_len, 0);

    block[0] = SBF_SYNC_1;
    block[1] = SBF_SYNC_2;

    uint16_t id_rev = static_cast<uint16_t>(SBFBlockId::PVTGeodetic) & 0x1FFF;
    std::memcpy(block.data() + 4, &id_rev, 2);

    uint16_t length = static_cast<uint16_t>(padded_len);
    std::memcpy(block.data() + 6, &length, 2);
    std::memcpy(block.data() + 8, &tow_ms, 4);
    std::memcpy(block.data() + 12, &wnc, 2);

    if (!payload.empty()) {
        std::memcpy(block.data() + 14, payload.data(), payload.size());
    }

    uint16_t crc = SBFProcessor::computeCRC16(block.data() + 4, padded_len - 4);
    std::memcpy(block.data() + 2, &crc, 2);

    return block;
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <ubx|sbf> [rate_hz]\n";
        return 1;
    }

    std::string protocol = argv[1];
    double rate_hz = (argc >= 3) ? std::atof(argv[2]) : 1.0;
    bool is_ubx = (protocol == "ubx");

    // Create PTY pair
    int master_fd, slave_fd;
    char slave_name[256];
    if (openpty(&master_fd, &slave_fd, slave_name, nullptr, nullptr) != 0) {
        std::cerr << "Failed to create PTY pair\n";
        return 1;
    }

    // Configure master pty for raw binary I/O (no line discipline processing)
    {
        struct termios tty;
        tcgetattr(master_fd, &tty);
        cfmakeraw(&tty);
        tcsetattr(master_fd, TCSANOW, &tty);
    }

    // Write PTY path to a well-known file for scripts
    {
        std::ofstream pty_file("/tmp/gnss_data_generator.pty");
        pty_file << slave_name << std::endl;
    }

    std::cerr << "=== GNSS Data Generator ===" << std::endl;
    std::cerr << "Protocol: " << (is_ubx ? "UBX" : "SBF") << std::endl;
    std::cerr << "Rate: " << rate_hz << " Hz" << std::endl;
    std::cerr << "PTY device: " << slave_name << std::endl;
    std::cerr << std::endl;
    std::cerr << "Connect with:" << std::endl;
    if (is_ubx) {
        std::cerr << "  ./build/tools/ubx_reader " << slave_name << " 115200" << std::endl;
    }
    std::cerr << "  ros2 run gnss_raw_driver gnss_raw_driver_node --ros-args "
              << "-p device:=" << slave_name
              << " -p protocol:=" << protocol << std::endl;
    std::cerr << std::endl;
    std::cerr << "Press Ctrl+C to stop." << std::endl;
    std::cerr.flush();

    // Make slave readable by the reader process, then close our fd
    chmod(slave_name, 0666);
    ::close(slave_fd);
    slave_fd = -1;

    std::signal(SIGINT, sigHandler);
    std::signal(SIGTERM, sigHandler);

    // Simulate trajectory: walk in a circle around Odaiba, Tokyo
    double base_lat = 35.6283;
    double base_lon = 139.7745;
    double base_height = 5.0;
    int num_sv = 14;
    uint32_t tow_ms = 0;
    uint16_t week = 2356;
    int epoch = 0;

    auto interval = std::chrono::microseconds(static_cast<int64_t>(1e6 / rate_hz));

    while (g_running) {
        double t = epoch * (1.0 / rate_hz);
        double angle = t * 0.1;  // ~60s per revolution
        double lat = base_lat + 0.0001 * std::sin(angle);
        double lon = base_lon + 0.0001 * std::cos(angle);
        double h = base_height + 0.5 * std::sin(angle * 0.3);

        tow_ms = static_cast<uint32_t>(t * 1000.0);

        std::vector<uint8_t> data;

        if (is_ubx) {
            auto pvt = buildNavPvt(lat, lon, h, num_sv, tow_ms, 3);
            auto rawx = buildRxmRawx(week, t, std::min(num_sv, 16));
            data.insert(data.end(), pvt.begin(), pvt.end());
            data.insert(data.end(), rawx.begin(), rawx.end());
        } else {
            auto pvt = buildSbfPvtGeodetic(lat * M_PI / 180.0, lon * M_PI / 180.0,
                                            h, num_sv, tow_ms, week);
            data.insert(data.end(), pvt.begin(), pvt.end());
        }

        ssize_t written = ::write(master_fd, data.data(), data.size());
        if (written < 0) {
            // Reader not connected yet, ignore
        }

        if (epoch % static_cast<int>(rate_hz) == 0) {
            std::cerr << "\r[" << (is_ubx ? "UBX" : "SBF")
                      << "] t=" << std::fixed << std::setprecision(1) << t
                      << "s lat=" << std::setprecision(6) << lat
                      << " lon=" << lon
                      << " nSV=" << num_sv
                      << " bytes=" << data.size()
                      << std::flush;
        }

        ++epoch;
        std::this_thread::sleep_for(interval);
    }

    std::cerr << "\nStopped after " << epoch << " epochs.\n";
    ::close(master_fd);
    return 0;
}
