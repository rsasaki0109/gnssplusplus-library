# Interfaces

## CLI

The primary interface is the `gnss` dispatcher plus native command binaries.

Examples:

- `gnss spp`
- `gnss solve`
- `gnss ppp`
- `gnss stream`
- `gnss convert`
- `gnss live`
- `gnss rcv`
- `gnss web`

## Python

Python bindings expose:

- RINEX header and epoch inspection
- `.pos` loading and solution statistics
- coordinate conversion helpers
- file-based `SPP`, `PPP`, and `RTK` solve helpers

## ROS2

The ROS2 playback node publishes `.pos` files as:

- `sensor_msgs/NavSatFix`
- `geometry_msgs/PoseStamped`
- `nav_msgs/Path`
- solution status and satellite-count telemetry

## Local web UI

`gnss web` is a local visualization layer for existing artifacts.

It shows:

- Odaiba metrics
- PPC summaries
- live sign-off summaries
- 2D trajectories
- `gnss rcv` receiver status
