#pragma once

#include <libgnss++/core/types.hpp>

#include <array>
#include <map>
#include <string>
#include <vector>

namespace libgnss {
namespace tidal {

struct EarthRotationParameters {
    struct Entry {
        double mjd = 0.0;
        double xp_rad = 0.0;
        double yp_rad = 0.0;
        double ut1_utc_s = 0.0;
        double lod_s_per_day = 0.0;
        double xpr_rad_per_day = 0.0;
        double ypr_rad_per_day = 0.0;
    };

    std::vector<Entry> entries;
};

struct EarthRotationValues {
    double xp_rad = 0.0;
    double yp_rad = 0.0;
    double ut1_utc_s = 0.0;
    double lod_s_per_day = 0.0;
    bool valid = false;
};

struct OceanLoadingCoefficients {
    std::array<double, 11> up_amplitudes_m{};
    std::array<double, 11> west_amplitudes_m{};
    std::array<double, 11> south_amplitudes_m{};
    std::array<double, 11> up_phases_deg{};
    std::array<double, 11> west_phases_deg{};
    std::array<double, 11> south_phases_deg{};
};

struct OceanLoadingGrid {
    // network id -> grid number -> BLQ coefficients
    std::map<int, std::map<int, OceanLoadingCoefficients>> networks;
};

struct ClasGridInterpolation {
    int network_id = 0;
    int grid_count = 0;
    std::array<int, 4> grid_numbers{};
    std::array<double, 4> weights{};
    bool use_model_interpolation = false;
    std::array<double, 16> model_gmat{};
    std::array<double, 4> model_emat{};
};

struct TideDisplacementComponents {
    Vector3d solid_ecef = Vector3d::Zero();
    Vector3d ocean_ecef = Vector3d::Zero();
    Vector3d pole_ecef = Vector3d::Zero();

    Vector3d total() const {
        return solid_ecef + ocean_ecef + pole_ecef;
    }
};

bool loadEarthRotationParameters(const std::string& filename,
                                 EarthRotationParameters& erp);

bool interpolateEarthRotationValues(const EarthRotationParameters& erp,
                                    const GNSSTime& time,
                                    EarthRotationValues& values);

bool loadOceanLoadingCoefficients(const std::string& filename,
                                  const std::string& station_name,
                                  OceanLoadingCoefficients& coefficients);

bool loadClasGridOceanLoading(const std::string& filename,
                              OceanLoadingGrid& grid);

const OceanLoadingCoefficients* findClasGridOceanLoading(
    const OceanLoadingGrid& grid,
    int network_id,
    int grid_number);

/// Julian date from GPS time.
double julianDateFromTime(const GNSSTime& time);

/// Greenwich Mean Sidereal Time (radians).
double greenwichMeanSiderealTime(const GNSSTime& time);

/// Rotate a vector from ECI to ECEF frame.
Vector3d rotateEciToEcef(const Vector3d& eci, double gmst_rad);

/// Approximate Sun position in ECEF (meters).
Vector3d approximateSunPositionEcef(const GNSSTime& time);

/// Approximate Moon position in ECEF (meters).
Vector3d approximateMoonPositionEcef(const GNSSTime& time);

/// Surface-up unit vector at a given ECEF position.
Vector3d surfaceUpVector(const Vector3d& receiver_position);

/// Degree-2 body tide displacement at the receiver due to one celestial body.
Vector3d bodyTideDisplacement(const Vector3d& receiver_position,
                              const Vector3d& body_position_ecef,
                              double body_gm_m3_s2);

/// Total solid Earth tide displacement at the receiver (meters, ECEF).
/// This is the sum of Sun and Moon degree-2 body tide contributions.
Vector3d calculateSolidEarthTides(const Vector3d& receiver_position,
                                  const GNSSTime& time);

Vector3d calculateSolidEarthTides(const Vector3d& receiver_position,
                                  const GNSSTime& time,
                                  const EarthRotationParameters* erp);

Vector3d calculateOceanLoading(const OceanLoadingCoefficients& coefficients,
                               const Vector3d& receiver_position,
                               const GNSSTime& time);

Vector3d calculateClasGridOceanLoading(const OceanLoadingGrid& grid,
                                       const ClasGridInterpolation& interpolation,
                                       const Vector3d& receiver_position,
                                       const GNSSTime& time,
                                       const EarthRotationValues& erp_values);

Vector3d calculatePoleTide(const Vector3d& receiver_position,
                           const GNSSTime& time,
                           const EarthRotationParameters* erp);

TideDisplacementComponents calculateTideDisplacement(
    const Vector3d& receiver_position,
    const GNSSTime& time,
    const EarthRotationParameters* erp,
    const OceanLoadingGrid* ocean_grid,
    const ClasGridInterpolation* ocean_interpolation,
    bool apply_solid,
    bool apply_ocean,
    bool apply_pole);

}  // namespace tidal
}  // namespace libgnss
