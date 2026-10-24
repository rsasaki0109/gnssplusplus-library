// Export observed-satellite geometry using the library's actual broadcast and
// fisheye-projection implementations. This is visualization, not a solver input.
#include <libgnss++/core/navigation.hpp>
#include <libgnss++/io/rinex.hpp>
#include <libgnss++/visibility/proj_fisheye.hpp>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

int exportGeometry(int argc, char** argv) {
    using namespace libgnss;
    if (argc != 7) {
        std::cerr << "Usage: satellite_geometry nav rover reference.csv start_tow end_tow output.csv\n";
        return 2;
    }
    const double start = std::stod(argv[4]), end = std::stod(argv[5]);
    std::map<std::pair<int, long long>, Vector3d> reference;
    std::ifstream input(argv[3]);
    if (!input) throw std::runtime_error("Cannot open reference");
    std::string line;
    std::getline(input, line);
    while (std::getline(input, line)) {
        std::stringstream stream(line); std::vector<double> row;
        std::string cell;
        while (std::getline(stream, cell, ',')) row.push_back(std::stod(cell));
        if (row.size() < 8 || row[0] < start - .001 || row[0] > end + .001) continue;
        reference[{int(row[1]), std::llround(row[0] * 1000)}] = Vector3d(row[5], row[6], row[7]);
    }
    io::RINEXReader nav_reader;
    NavigationData nav;
    io::RINEXReader::RINEXHeader nav_header;
    if (!nav_reader.open(argv[1]) || !nav_reader.readHeader(nav_header) || !nav_reader.readNavigationData(nav))
        throw std::runtime_error("Cannot read broadcast navigation");
    io::RINEXReader obs_reader;
    io::RINEXReader::RINEXHeader obs_header;
    if (!obs_reader.open(argv[2]) || !obs_reader.readHeader(obs_header)) throw std::runtime_error("Cannot read rover observations");
    visibility::FisheyeCamera camera;
    camera.width = camera.height = 256;
    camera.xc = camera.yc = 128;
    camera.r_calib = {0, 112}; camera.el_calib = {90, 0}; camera.poly_degree = 1;
    std::ofstream output(argv[6]);
    if (!output) throw std::runtime_error("Cannot write output");
    output << "week,tow,satellite,receiver_x,receiver_y,receiver_z,satellite_x,satellite_y,satellite_z,az_deg,el_deg,fish_x,fish_y,health\n";
    output << std::setprecision(15);
    ObservationData obs;
    std::size_t count = 0, missing = 0;
    while (obs_reader.readObservationEpoch(obs)) {
        if (obs.time.tow > end + .001) break;
        auto ref = reference.find({obs.time.week, std::llround(obs.time.tow * 1000)});
        if (ref == reference.end()) continue;
        for (const auto& sat : obs.getSatellites()) {
            Vector3d position, velocity; double bias, drift;
            if (!nav.calculateSatelliteState(sat, obs.time, position, velocity, bias, drift)) { ++missing; continue; }
            const auto geometry = nav.calculateGeometry(ref->second, position);
            const double az = geometry.azimuth * 180 / M_PI, el = geometry.elevation * 180 / M_PI;
            if (el < 0) continue;
            const auto projection = visibility::projFisheye({az}, {el}, camera);
            if (projection.px.size() != 1) throw std::runtime_error("Fisheye projection failed");
            const auto* eph = nav.getEphemeris(sat, obs.time);
            output << obs.time.week << ',' << obs.time.tow << ',' << sat.toString();
            for (int i=0;i<3;++i) output << ',' << ref->second[i];
            for (int i=0;i<3;++i) output << ',' << position[i];
            output << ',' << az << ',' << el << ',' << projection.px[0] << ',' << projection.py[0] << ',' << (eph ? eph->health : -1) << '\n';
            ++count;
        }
    }
    if (!count) throw std::runtime_error("No satellite geometry exported");
    output.flush();
    if (!output) throw std::runtime_error("Failed writing satellite geometry");
    std::cout << "Exported " << count << " observed satellite/epoch records; " << missing << " without usable ephemeris\n";
    return 0;
}

int main(int argc, char** argv) {
    try {
        return exportGeometry(argc, argv);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
