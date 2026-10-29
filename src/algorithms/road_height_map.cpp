#include <libgnss++/algorithms/road_height_map.hpp>
#include <libgnss++/core/coordinates.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace libgnss {
namespace road_height {

namespace {

std::string toLower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

std::string trim(const std::string& s) {
    const auto begin = s.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos) return "";
    const auto end = s.find_last_not_of(" \t\r\n");
    return s.substr(begin, end - begin + 1);
}

std::vector<std::string> splitCsvLine(const std::string& line) {
    std::vector<std::string> out;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) out.push_back(trim(cell));
    return out;
}

long long cellKey(long long ix, long long iy) { return (ix << 32) ^ (iy & 0xffffffffLL); }

Eigen::Vector2d localEastNorth(const RoadHeightMap& map, const Eigen::Vector3d& ecef) {
    const Eigen::Vector3d enu = ecef2enu(ecef - map.origin_ecef, map.origin_lat_rad, map.origin_lon_rad);
    return enu.head<2>();
}

}  // namespace

RoadHeightMap buildRoadHeightMap(const std::vector<Eigen::Vector3d>& lat_lon_height,
                                 double cell_m) {
    RoadHeightMap map;
    if (lat_lon_height.empty()) return map;
    map.cell_m = cell_m > 0.0 ? cell_m : 2.0;
    map.origin_lat_rad = lat_lon_height.front().x() * M_PI / 180.0;
    map.origin_lon_rad = lat_lon_height.front().y() * M_PI / 180.0;
    map.origin_ecef = geodetic2ecef(map.origin_lat_rad, map.origin_lon_rad, 0.0);
    map.points.reserve(lat_lon_height.size());
    for (const auto& row : lat_lon_height) {
        const Eigen::Vector3d ecef =
            geodetic2ecef(row.x() * M_PI / 180.0, row.y() * M_PI / 180.0, row.z());
        const Eigen::Vector2d en = localEastNorth(map, ecef);
        const int index = static_cast<int>(map.points.size());
        map.points.emplace_back(en.x(), en.y(), row.z());
        map.grid[cellKey(static_cast<long long>(std::floor(en.x() / map.cell_m)),
                         static_cast<long long>(std::floor(en.y() / map.cell_m)))]
            .push_back(index);
    }
    return map;
}

RoadHeightMap loadRoadHeightCsv(const std::string& path, double cell_m) {
    std::ifstream input(path);
    if (!input.is_open()) throw std::runtime_error("failed to open road height CSV: " + path);
    std::string line;
    if (!std::getline(input, line)) throw std::runtime_error("road height CSV is empty: " + path);
    const auto header = splitCsvLine(line);
    int lat_col = -1, lon_col = -1, height_col = -1;
    for (size_t i = 0; i < header.size(); ++i) {
        const std::string name = toLower(header[i]);
        if (name == "lat_deg") lat_col = static_cast<int>(i);
        if (name == "lon_deg") lon_col = static_cast<int>(i);
        if (name == "height_m") height_col = static_cast<int>(i);
    }
    if (lat_col < 0 || lon_col < 0 || height_col < 0) {
        throw std::runtime_error(
            "road height CSV missing required columns (lat_deg, lon_deg, height_m): " + path);
    }
    const size_t needed = static_cast<size_t>(std::max({lat_col, lon_col, height_col})) + 1;
    std::vector<Eigen::Vector3d> rows;
    while (std::getline(input, line)) {
        if (line.empty()) continue;
        const auto cells = splitCsvLine(line);
        if (cells.size() < needed) continue;
        try {
            const Eigen::Vector3d row(std::stod(cells[lat_col]), std::stod(cells[lon_col]),
                                      std::stod(cells[height_col]));
            if (row.allFinite()) rows.push_back(row);
        } catch (const std::exception&) {
            continue;
        }
    }
    if (rows.empty()) throw std::runtime_error("road height CSV has no rows: " + path);
    return buildRoadHeightMap(rows, cell_m);
}

std::optional<double> lookupRoadHeight(const RoadHeightMap& map, const Eigen::Vector3d& ecef,
                                       double radius_m) {
    if (map.empty() || !ecef.allFinite() || !(radius_m > 0.0)) return std::nullopt;
    const Eigen::Vector2d en = localEastNorth(map, ecef);
    const long long ix = static_cast<long long>(std::floor(en.x() / map.cell_m));
    const long long iy = static_cast<long long>(std::floor(en.y() / map.cell_m));
    const double radius = std::min(radius_m, map.cell_m);
    double best = radius * radius;
    std::optional<double> height;
    for (long long dx = -1; dx <= 1; ++dx) {
        for (long long dy = -1; dy <= 1; ++dy) {
            const auto it = map.grid.find(cellKey(ix + dx, iy + dy));
            if (it == map.grid.end()) continue;
            for (int index : it->second) {
                const Eigen::Vector3d& p = map.points[static_cast<size_t>(index)];
                const double d2 = (p.head<2>() - en).squaredNorm();
                if (d2 <= best) {
                    best = d2;
                    height = p.z();
                }
            }
        }
    }
    return height;
}

std::optional<RoadSurface> lookupRoadSurface(const RoadHeightMap& map, const Eigen::Vector3d& ecef,
                                             double radius_m) {
    const auto height = lookupRoadHeight(map, ecef, radius_m);
    if (!height) return std::nullopt;
    RoadSurface surface;
    surface.height_m = *height;
    const Eigen::Vector2d en = localEastNorth(map, ecef);
    const long long ix = static_cast<long long>(std::floor(en.x() / map.cell_m));
    const long long iy = static_cast<long long>(std::floor(en.y() / map.cell_m));
    const double fit_radius = 2.0 * map.cell_m;
    Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
    Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
    int count = 0;
    for (long long dx = -2; dx <= 2; ++dx) {
        for (long long dy = -2; dy <= 2; ++dy) {
            const auto it = map.grid.find(cellKey(ix + dx, iy + dy));
            if (it == map.grid.end()) continue;
            for (int index : it->second) {
                const Eigen::Vector3d& p = map.points[static_cast<size_t>(index)];
                const Eigen::Vector2d d = p.head<2>() - en;
                if (d.squaredNorm() > fit_radius * fit_radius) continue;
                const Eigen::Vector3d row(1.0, d.x(), d.y());
                normal += row * row.transpose();
                rhs += row * p.z();
                ++count;
            }
        }
    }
    if (count >= 3) {
        Eigen::FullPivLU<Eigen::Matrix3d> lu(normal);
        if (lu.rank() == 3) surface.gradient = lu.solve(rhs).tail<2>();
    }
    return surface;
}

}  // namespace road_height
}  // namespace libgnss
