#pragma once

// Optional road-surface height prior for the RTK float filter
// (RTKConfig::road_height_sigma_m). The map is external, e.g. the road
// surfaces of PLATEAU LOD3 tran 3D Tiles sampled on a grid, and is loaded from
// a small CSV contract:
//
//   lat_deg,lon_deg,height_m
//   35.66755248,139.79109598,38.012
//
// ``height_m`` is the ellipsoidal height of the road surface. As with
// nlos_weights.hpp, no file I/O happens inside RTKProcessor: callers
// (apps/native/gnss_solve.cpp) load the map once with loadRoadHeightCsv() and
// hand a shared_ptr to RTKProcessor::setRoadHeightMap().

#include <Eigen/Dense>

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace libgnss {
namespace road_height {

/// Road-surface points in a local east/north frame, bucketed on a square grid.
struct RoadHeightMap {
    double origin_lat_rad = 0.0;
    double origin_lon_rad = 0.0;
    Eigen::Vector3d origin_ecef = Eigen::Vector3d::Zero();
    double cell_m = 2.0;
    /// (east m, north m, road ellipsoidal height m) per point.
    std::vector<Eigen::Vector3d> points;
    std::unordered_map<long long, std::vector<int>> grid;

    bool empty() const { return points.empty(); }
};

/// Builds the map from (lat_deg, lon_deg, height_m) rows. The local frame is
/// tangent at the first row.
RoadHeightMap buildRoadHeightMap(const std::vector<Eigen::Vector3d>& lat_lon_height,
                                 double cell_m = 2.0);

/// Parses a ``lat_deg,lon_deg,height_m`` CSV (header-driven, case-insensitive,
/// extra columns ignored). Throws std::runtime_error if the file cannot be
/// opened, a column is missing, or it has no rows.
RoadHeightMap loadRoadHeightCsv(const std::string& path, double cell_m = 2.0);

/// Road height of the map point nearest to ``ecef`` within ``radius_m``
/// horizontally (radius_m <= map.cell_m), or nullopt.
std::optional<double> lookupRoadHeight(const RoadHeightMap& map, const Eigen::Vector3d& ecef,
                                       double radius_m);

/// Road height as lookupRoadHeight() plus the local road gradient
/// (d height / d east, d height / d north) of a plane fitted to the map points
/// within two cells; the gradient is zero with fewer than three points.
struct RoadSurface {
    double height_m = 0.0;
    Eigen::Vector2d gradient = Eigen::Vector2d::Zero();
};
std::optional<RoadSurface> lookupRoadSurface(const RoadHeightMap& map, const Eigen::Vector3d& ecef,
                                             double radius_m);

}  // namespace road_height
}  // namespace libgnss
