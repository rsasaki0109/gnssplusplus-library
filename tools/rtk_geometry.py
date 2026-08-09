"""Shared WGS84 coordinate helpers for the standalone RTK tools."""

import math


def geodetic_to_ecef(lat_deg: float, lon_deg: float, height_m: float) -> tuple[float, float, float]:
    """Convert WGS84 latitude/longitude/height to ECEF metres."""
    semi_major_axis = 6378137.0
    flattening = 1.0 / 298.257223563
    eccentricity_squared = 2.0 * flattening - flattening * flattening

    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    prime_vertical_radius = semi_major_axis / math.sqrt(
        1.0 - eccentricity_squared * sin_lat * sin_lat
    )

    return (
        (prime_vertical_radius + height_m) * cos_lat * math.cos(lon),
        (prime_vertical_radius + height_m) * cos_lat * math.sin(lon),
        (prime_vertical_radius * (1.0 - eccentricity_squared) + height_m) * sin_lat,
    )


def ecef_to_enu(
    dx: float,
    dy: float,
    dz: float,
    reference_lat_deg: float,
    reference_lon_deg: float,
) -> tuple[float, float, float]:
    """Convert an ECEF displacement to ENU at a geodetic reference point."""
    lat = math.radians(reference_lat_deg)
    lon = math.radians(reference_lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)

    return (
        -sin_lon * dx + cos_lon * dy,
        -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz,
        cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz,
    )
