# Interfaces

## CLI

The primary interface is the `gnss` dispatcher plus native command binaries.

Examples:

- `gnss spp`
- `gnss solve`
- `gnss ppp` (`--nav`, `--sp3`, `--clk`, `--ionex`, `--dcb`, `--ssr`, `--ssr-rtcm`)
- `gnss fetch-products`
- `gnss ionex-info`
- `gnss dcb-info`
- `gnss visibility`
- `gnss visibility-plot`
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
- visibility summaries and a polar visibility view
- `gnss rcv` receiver status
