// Offline file adapter; the processor sees only records admitted at each
// simulated reception event. No reference trajectory is opened here.
#include <libgnss++/fusion/online_pva_csv.hpp>
#include <libgnss++/io/imu.hpp>
#include <libgnss++/io/rinex.hpp>
#include <filesystem>
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdexcept>

using namespace libgnss;
namespace fs = std::filesystem;
namespace {
void open(io::RINEXReader& reader, const fs::path& path, io::RINEXReader::RINEXHeader& header) {
    if (!reader.open(path.string()) || !reader.readHeader(header))
        throw std::runtime_error("cannot read RINEX header: " + path.string());
}
}
int main(int argc, char** argv) {
    try {
        if (argc == 2 && std::string(argv[1]) == "--help") {
            std::cout << "gnss_pva_replay RAW_RUN NEW_OUTPUT_DIR MAX_EPOCHS [normal|gnss_outage|imu_gap|loose_only] [START_S DURATION_S]\n"
                "MAX_EPOCHS=0 means full input. Body FLU, local ENU, GPST. No reference input.\n";
            return 0;
        }
        if (argc < 4 || argc > 7 || argc == 6) throw std::invalid_argument("see --help for argument contract");
        const fs::path data(argv[1]), output(argv[2]);
        const int limit = std::stoi(argv[3]);
        const std::string scenario = argc >= 5 ? argv[4] : "normal";
        const double start_s = argc == 7 ? std::stod(argv[5]) : 60.0;
        const double duration_s = argc == 7 ? std::stod(argv[6]) : (scenario == "imu_gap" ? 4.0 : 10.0);
        if (limit < 0 || fs::exists(output) || !std::isfinite(start_s) || start_s < 0.0 ||
            !std::isfinite(duration_s) || duration_s <= 0.0 ||
            (scenario != "normal" && scenario != "gnss_outage" && scenario != "imu_gap" && scenario != "loose_only"))
            throw std::invalid_argument("invalid replay scenario or existing output directory");
        io::RINEXReader rover_reader, base_reader, nav_reader;
        io::RINEXReader::RINEXHeader rover_header, base_header, nav_header;
        open(rover_reader, data / "rover.obs", rover_header);
        open(base_reader, data / "base.obs", base_header);
        open(nav_reader, data / "base.nav", nav_header);
        NavigationData archive;
        if (!nav_reader.readNavigationData(archive)) throw std::runtime_error("cannot read navigation");
        ImuSeries imu;
        const auto loaded = loadImuCsv((data / "imu.csv").string(), imu);
        if (!loaded.ok) throw std::runtime_error(loaded.error);
        imu.sortByTime();
        OnlineRtkImuProcessor::Config config;
        config.base_position_ecef = base_header.approximate_position;
        const bool nagoya = data.generic_string().find("nagoya") != std::string::npos;
        config.fusion.lever_arm_body = nagoya ? Vector3d(.593, -.670, -1.216) : Vector3d(.31, 0., .55);
        config.tight_time_update = scenario != "loose_only";
        OnlineRtkImuProcessor processor(config);
        fs::create_directories(output);
        std::ofstream csv(output / "pva.csv"), meta(output / "replay.json");
        if (!csv || !meta) throw std::runtime_error("cannot create replay output");
        csv << kOnlinePvaCsvHeader << '\n';
        ObservationData rover, base;
        bool have_base = base_reader.readObservationEpoch(base);
        std::size_t imu_cursor = 0;
        std::vector<Ephemeris> records;
        for (const auto& entry : archive.ephemeris_data)
            for (const auto& eph : entry.second) if (eph.valid) records.push_back(eph);
        std::vector<bool> sent(records.size(), false);
        int count = 0;
        GNSSTime start;
        while ((limit == 0 || count < limit) && rover_reader.readObservationEpoch(rover)) {
            if (count == 0) {
                start = rover.time;
                while (imu_cursor < imu.size() && imu.samples[imu_cursor].time < start - 2.5) ++imu_cursor;
            }
            NavigationData received;
            for (std::size_t i = 0; i < records.size(); ++i) {
                if (!sent[i] && records[i].toc <= rover.time && records[i].tof <= rover.time) {
                    received.addEphemeris(records[i]); sent[i] = true;
                }
            }
            processor.pushNavigation(received, rover.time);
            while (imu_cursor < imu.size() && imu.samples[imu_cursor].time <= rover.time) {
                const auto& sample = imu.samples[imu_cursor++];
                const double age = sample.time - start;
                if (!(scenario == "imu_gap" && age >= start_s && age < start_s + duration_s))
                    processor.pushImu(sample, rover.time);
            }
            while (have_base && base.time <= rover.time) {
                processor.pushBase(base, rover.time);
                have_base = base_reader.readObservationEpoch(base);
            }
            const double age = rover.time - start;
            const bool outage = scenario == "gnss_outage" && age >= start_s && age < start_s + duration_s;
            writeOnlinePvaCsv(csv, rover.time, processor.processRover(
                outage ? ObservationData(rover.time) : rover, rover.time));
            ++count;
        }
        if (count == 0 || (limit > 0 && count != limit)) throw std::runtime_error("insufficient rover epochs");
        const auto diagnostics = processor.diagnostics();
        meta << std::setprecision(17) << "{\"schema\":\"libgnsspp.online_pva_replay.v1\",\"state\":\"passed\","
            "\"reference_used\":false,\"receipt_times\":\"simulated at rover epoch\","
            "\"navigation_policy\":\"toc and tof <= rover epoch\",\"scenario\":\"" << scenario
            << "\",\"scenario_start_s\":" << start_s << ",\"scenario_duration_s\":" << duration_s
            << ",\"epochs\":" << count << ",\"max_epochs\":" << limit << ",\"reset_count\":"
            << diagnostics.reset_generation << ",\"start_week\":" << start.week << ",\"start_tow\":" << start.tow
            << ",\"base_ecef\":[" << config.base_position_ecef.x() << ',' << config.base_position_ecef.y() << ','
            << config.base_position_ecef.z() << "],\"lever_arm_flu_m\":[" << config.fusion.lever_arm_body.x() << ','
            << config.fusion.lever_arm_body.y() << ',' << config.fusion.lever_arm_body.z() << "]}\n";
        std::cout << "replayed " << count << " epochs (" << scenario << ")\n";
        return 0;
    } catch (const std::exception& error) { std::cerr << "gnss_pva_replay: " << error.what() << '\n'; return 2; }
}
