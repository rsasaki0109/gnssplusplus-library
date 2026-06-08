/**
 * @file example_vehicle_animation.cpp
 * @brief Animation of a vehicle driving - C++ port of example_vehicle_animation.m
 *
 * Usage:
 *   ./example_vehicle_animation <reference.csv> <vehicle_model.dae>
 *
 * The CSV file should contain columns:
 *   col[2]=lat, col[3]=lon, col[4]=alt, col[10]=heading (degrees)
 *
 * Outputs: example_vehicle_animation.kml
 */

#include <libgnss++/kml/kml.hpp>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>

using namespace libgnss::kml;

struct TrajectoryPoint {
    double lat, lon, alt, heading;
};

std::vector<TrajectoryPoint> loadCsv(const std::string& filepath) {
    std::vector<TrajectoryPoint> points;
    std::ifstream ifs(filepath);
    if (!ifs.is_open()) {
        std::cerr << "Failed to open: " << filepath << std::endl;
        return points;
    }

    std::string line;
    while (std::getline(ifs, line)) {
        if (line.empty() || line[0] == '#') continue;

        std::istringstream ss(line);
        std::string token;
        std::vector<double> vals;

        while (std::getline(ss, token, ',')) {
            try {
                vals.push_back(std::stod(token));
            } catch (...) {
                vals.push_back(std::nan(""));
            }
        }

        if (vals.size() >= 11) {
            points.push_back({vals[2], vals[3], vals[4], vals[10]});
        }
    }
    return points;
}

int main(int argc, char* argv[]) {
    std::string csv_file = "data/vehicle_nagoya/reference.csv";
    std::string vehicle_model = "model/vehicle_gray.dae";

    if (argc >= 2) csv_file = argv[1];
    if (argc >= 3) vehicle_model = argv[2];

    // Load trajectory
    auto traj = loadCsv(csv_file);
    if (traj.empty()) {
        std::cerr << "No trajectory data loaded." << std::endl;
        return 1;
    }
    std::cout << "Loaded " << traj.size() << " trajectory points." << std::endl;

    size_t n = traj.size();
    double dt = 1.0;

    // Process headings: unwrap then wrapTo720
    std::vector<double> raw_heading(n);
    for (size_t i = 0; i < n; ++i) raw_heading[i] = traj[i].heading;
    auto headings = unwrap360(raw_heading);
    for (auto& h : headings) h = wrapTo720(h);

    // Build location/orientation arrays
    std::vector<Location> vloc(n);
    std::vector<Orientation> vori(n);
    for (size_t i = 0; i < n; ++i) {
        vloc[i] = {traj[i].lat, traj[i].lon, traj[i].alt};
        vori[i] = {headings[i], 0.0, 0.0};
    }

    // Settings
    double vehicle_scale = 1.0;
    double speedup = 5.0;
    LookAtParams cam(headings[0], 50.0, 90.0);

    // Line & point params
    double lw = 3.0;
    Color lcol(1, 0, 0);
    double lalpha = 0.8;
    double pscale = 0.5;
    Color pcol(1, 0, 0);
    double palpha = 0.8;

    // Generate KML elements
    auto kml_look0 = createLookAt(vloc[0], cam, AltitudeMode::Absolute);
    auto kml_model = createModel("Vehicle", vehicle_model, vloc[0], vori[0],
                                 vehicle_scale, AltitudeMode::Absolute);
    auto kml_line = createLine("Line", vloc, lw, lcol, lalpha);

    std::vector<std::string> pids(n);
    for (size_t i = 0; i < n; ++i) pids[i] = "Point";
    auto kml_point = createPoints(pids, vloc, pscale, {pcol}, palpha);

    // Generate tour
    std::string tour;
    tour += wrapFlyTo(kml_look0);

    for (size_t i = 1; i < n; ++i) {
        tour += animatedUpdateModel("Vehicle", dt / speedup, vloc[i],
                                    Orientation(headings[i], 0, 0));

        cam.heading = headings[i];
        tour += wrapFlyTo(createLookAt(vloc[i], cam, AltitudeMode::Absolute),
                          dt / speedup);

        if (i % 100 == 0) {
            std::cout << i << "/" << n << "..." << std::endl;
        }
    }
    tour = wrapTour(tour, "DriveTour");

    // Write KML
    std::string content = kml_look0 + kml_model + kml_line + kml_point + tour;
    writeKml("example_vehicle_animation.kml", content);

    std::cout << "Output: example_vehicle_animation.kml" << std::endl;
    return 0;
}
