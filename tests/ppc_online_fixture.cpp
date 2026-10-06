// Offline staging and regression harness. File reads here are not part of the
// online processor. Reception is a declared simulation, not a PPC network log.
#include <libgnss++/fusion/online_rtk_imu.hpp>
#include <libgnss++/io/imu.hpp>
#include <libgnss++/io/rinex.hpp>
#include <libgnss++/io/rtcm.hpp>
#include <array>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

using namespace libgnss;
namespace fs = std::filesystem;
namespace {
void open(io::RINEXReader& reader, const fs::path& path, io::RINEXReader::RINEXHeader& header) {
    if (!reader.open(path.string()) || !reader.readHeader(header))
        throw std::runtime_error("cannot read RINEX header: " + path.string());
}
std::string stamp(const GNSSTime& time) {
    std::ostringstream out;
    out << std::setprecision(17) << time.week << ' ' << time.tow;
    return out.str();
}
std::string frame(const io::RTCMMessage& message) {
    if (!message.valid || message.data.empty() || message.data.size() > 1023)
        throw std::runtime_error("RTCM encoder could not stage frame");
    std::vector<std::uint8_t> bytes{0xd3, static_cast<std::uint8_t>(message.data.size() >> 8),
        static_cast<std::uint8_t>(message.data.size())};
    bytes.insert(bytes.end(), message.data.begin(), message.data.end());
    std::uint32_t crc = 0;
    for (auto byte : bytes) {
        crc ^= static_cast<std::uint32_t>(byte) << 16;
        for (int bit = 0; bit < 8; ++bit) {
            crc <<= 1;
            if (crc & 0x1000000U) crc ^= 0x1864cfbU;
        }
    }
    bytes.push_back(static_cast<std::uint8_t>(crc >> 16));
    bytes.push_back(static_cast<std::uint8_t>(crc >> 8));
    bytes.push_back(static_cast<std::uint8_t>(crc));
    std::ostringstream out;
    out << std::hex << std::setfill('0');
    for (auto byte : bytes) out << std::setw(2) << static_cast<int>(byte);
    return out.str();
}
void stageEpoch(std::ostream& out, io::RTCMProcessor& encoder, const char* kind,
                const ObservationData& obs, const GNSSTime& arrival) {
    // GPS-only transport fixture avoids pretending the existing encoder can
    // produce Galileo/BeiDou navigation. Typed API audit below uses all systems.
    ObservationData gps(obs.time);
    for (const auto& row : obs.observations)
        if (row.satellite.system == GNSSSystem::GPS) gps.addObservation(row);
    if (gps.observations.empty()) throw std::runtime_error("GPS transport epoch is empty");
    out << kind << ' ' << stamp(arrival) << ' ' << stamp(obs.time) << ' '
        << frame(encoder.encodeObservations(gps, io::RTCMMessageType::RTCM_1077)) << '\n';
}
void solution(std::ostream& out, const PositionSolution& value) {
    out << ',' << static_cast<int>(value.status) << ',' << value.time.week << ',' << value.time.tow;
    if (value.isValid()) out << ',' << value.position_ecef.x() << ',' << value.position_ecef.y()
                            << ',' << value.position_ecef.z();
    else out << ",nan,nan,nan";
}
void write(std::ostream& out, const ObservationData& obs, const OnlineRtkImuProcessor::Output& row) {
    out << std::setprecision(17) << obs.time.week << ',' << obs.time.tow << ','
        << row.received_at.week << ',' << row.received_at.tow << ',' << row.input_age_s << ','
        << row.exact_base_available << ',' << row.imu_consumed << ',' << row.reset_generation << ','
        << row.fusion_initialized << ',' << row.heading_converged << ',' << row.gnss_position_updated << ','
        << row.tight_time_update_supplied << ',' << row.fusion_age_s << ',' << row.processing_ms << ',' << row.reason;
    solution(out, row.rtk); solution(out, row.fused); out << '\n';
}
}
int main(int argc, char** argv) {
    try {
        if (argc != 4) throw std::invalid_argument("usage: gnss_online_ppc_fixture RAW_RUN NEW_OUTPUT_DIR MAX_EPOCHS");
        const fs::path data(argv[1]), output(argv[2]);
        const int limit = std::stoi(argv[3]);
        if (limit < 100 || fs::exists(output)) throw std::invalid_argument("need >=100 epochs and new output directory");
        fs::create_directories(output);
        io::RINEXReader rover_reader, base_reader, nav_reader;
        io::RINEXReader::RINEXHeader rover_header, base_header, nav_header;
        open(rover_reader, data / "rover.obs", rover_header);
        open(base_reader, data / "base.obs", base_header);
        open(nav_reader, data / "base.nav", nav_header);
        NavigationData archive;
        if (!nav_reader.readNavigationData(archive)) throw std::runtime_error("cannot read broadcast navigation");
        ImuSeries imu;
        const auto loaded = loadImuCsv((data / "imu.csv").string(), imu);
        if (!loaded.ok) throw std::runtime_error(loaded.error);
        imu.sortByTime();
        OnlineRtkImuProcessor::Config config;
        config.base_position_ecef = base_header.approximate_position;
        const bool nagoya = data.generic_string().find("nagoya") != std::string::npos;
        config.fusion.lever_arm_body = nagoya ? Vector3d(.593, -.670, -1.216) : Vector3d(.31, 0., .55);
        std::array<std::unique_ptr<OnlineRtkImuProcessor>, 5> processors;
        std::array<std::ofstream, 5> files;
        const std::array<std::string, 5> names{"normal", "missing_base", "late_base", "imu_gap", "delayed_rover"};
        for (std::size_t i = 0; i < processors.size(); ++i) {
            processors[i] = std::make_unique<OnlineRtkImuProcessor>(config);
            files[i].open(output / (names[i] + ".csv"));
            files[i] << "rover_week,rover_tow,received_week,received_tow,input_age_s,exact_base,imu_consumed,reset_generation,"
                "fusion_initialized,heading_converged,gnss_position_updated,tight_update,fusion_age_s,processing_ms,reason,"
                "rtk_status,rtk_week,rtk_tow,rtk_x_m,rtk_y_m,rtk_z_m,fused_status,fused_week,fused_tow,fused_x_m,fused_y_m,fused_z_m\n";
        }
        std::ofstream events(output / "events.txt"), metadata(output / "staging.json");
        io::RTCMProcessor encoder;
        ObservationData rover, base;
        bool have_base = base_reader.readObservationEpoch(base);
        std::size_t imu_cursor = 0;
        std::vector<Ephemeris> nav_records;
        for (const auto& entry : archive.ephemeris_data)
            for (const auto& eph : entry.second) if (eph.valid) nav_records.push_back(eph);
        std::vector<bool> sent(nav_records.size(), false);
        int count = 0;
        GNSSTime start;
        while (count < limit && rover_reader.readObservationEpoch(rover)) {
            if (count == 0) {
                start = rover.time;
                while (imu_cursor < imu.size() && imu.samples[imu_cursor].time < start - 2.5) ++imu_cursor;
                metadata << std::setprecision(17) << "{\"schema\":\"online_ppc_staging.v1\",\"base_ecef\":["
                    << config.base_position_ecef.x() << ',' << config.base_position_ecef.y() << ',' << config.base_position_ecef.z()
                    << "],\"lever_arm\":[" << config.fusion.lever_arm_body.x() << ',' << config.fusion.lever_arm_body.y()
                    << ',' << config.fusion.lever_arm_body.z() << "],\"prefix_epochs\":" << limit / 2
                    << ",\"late_base_reception_delay_s\":0.1,\"delayed_rover_reception_delay_s\":0.15,"
                    "\"reception_contract\":\"simulated arrival at each rover epoch; navigation admitted only after toc and tof; no observed PPC network receipts\","
                    "\"typed_api_systems\":\"all RINEX systems\",\"rtcm_transport_systems\":\"GPS only, MSM7/1019 encoder scope\","
                    "\"reference_used\":false}";
            }
            NavigationData received_nav;
            for (std::size_t i = 0; i < nav_records.size(); ++i) {
                const auto& eph = nav_records[i];
                if (!sent[i] && eph.toc <= rover.time && eph.tof <= rover.time) {
                    received_nav.addEphemeris(eph);
                    if (eph.satellite.system == GNSSSystem::GPS)
                        events << "NAV " << stamp(rover.time) << ' ' << frame(encoder.encodeEphemeris(eph)) << '\n';
                    sent[i] = true;
                }
            }
            for (auto& processor : processors) processor->pushNavigation(received_nav, rover.time);
            while (imu_cursor < imu.size() && imu.samples[imu_cursor].time <= rover.time) {
                const auto& sample = imu.samples[imu_cursor++];
                events << "IMU " << stamp(rover.time) << ' ' << stamp(sample.time) << std::setprecision(17);
                for (int axis = 0; axis < 3; ++axis) events << ' ' << sample.accel_raw(axis);
                for (int axis = 0; axis < 3; ++axis) events << ' ' << sample.gyro_raw_radps(axis);
                events << '\n';
                for (std::size_t i = 0; i < processors.size(); ++i)
                    if (!(i == 3 && count >= limit / 2 && count < limit / 2 + 20))
                        processors[i]->pushImu(sample, rover.time);
            }
            std::vector<ObservationData> late;
            while (have_base && base.time <= rover.time) {
                processors[0]->pushBase(base, rover.time);
                if (count < limit / 2) {
                    processors[1]->pushBase(base, rover.time);
                    processors[2]->pushBase(base, rover.time);
                } else late.push_back(base);
                processors[3]->pushBase(base, rover.time);
                processors[4]->pushBase(base, rover.time);
                stageEpoch(events, encoder, "BASE", base, rover.time);
                have_base = base_reader.readObservationEpoch(base);
            }
            stageEpoch(events, encoder, "ROVER", rover, rover.time);
            for (std::size_t i = 0; i < processors.size(); ++i)
                write(files[i], rover, processors[i]->processRover(rover,
                    i == 4 && count >= limit / 2 ? rover.time + .15 : rover.time));
            for (const auto& delayed : late) processors[2]->pushBase(delayed, rover.time + .1);
            ++count;
        }
        if (count != limit) throw std::runtime_error("too few rover epochs");
        std::cout << "staged " << count << " raw epochs in " << output.string() << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "gnss_online_ppc_fixture: " << error.what() << '\n';
        return 2;
    }
}
