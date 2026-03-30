# Interfaces

## CLI

The primary interface is the `gnss` dispatcher plus native command binaries.

Examples:

- `gnss spp`
- `gnss solve`
- `gnss ppp` (`--nav`, `--sp3`, `--clk`, `--ionex`, `--dcb`, `--ssr`, `--ssr-rtcm`)
- `gnss fetch-products`
- `gnss ppp-products-signoff`
- `gnss ionex-info`
- `gnss dcb-info`
- `gnss visibility`
- `gnss visibility-plot`
- `gnss moving-base-plot`
- `gnss moving-base-signoff`
- `gnss moving-base-prepare`
- `gnss scorpion-moving-base-signoff`
- `gnss stream`
- `gnss convert`
- `gnss live`
- `gnss rcv`
- `gnss web`

For longer workflows, `gnss web`, `gnss moving-base-signoff`,
`gnss scorpion-moving-base-signoff`, and `gnss ppp-products-signoff`
also accept `--config-toml <file>`. Example templates live in `configs/`.

## Docker

The repo also ships a multi-stage `Dockerfile`. The runtime image installs:

- the `gnss` dispatcher on `PATH`
- native CLI binaries under `/opt/libgnsspp/bin`
- the `libgnsspp` Python package via `PYTHONPATH`

The default entrypoint is `gnss`, so container invocations look like:

```bash
docker run --rm -it -v "$PWD:/workspace" libgnsspp:latest --help
docker run --rm -it -v "$PWD:/workspace" libgnsspp:latest ppp --help
docker run --rm -it -p 8085:8085 -v "$PWD:/workspace" libgnsspp:latest web --host 0.0.0.0 --port 8085 --root /workspace
docker compose up gnss-web
```

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

`gnss web` is a local visualization layer for existing artifacts, including live, moving-base, and PPP-product sign-off summaries, artifact bundles, moving-base history charts, and direct links to summary, plot, product, comparison CSV/PNG, match CSV, and provenance files.

It shows:

- Odaiba metrics
- PPC summaries
- live sign-off summaries
- moving-base plots
- 2D trajectories
- visibility summaries and a polar visibility view
- `gnss rcv` receiver status

## Moving-base

`gnss solve`, `gnss replay`, and `gnss live` accept `--mode moving-base`.

`gnss moving-base-prepare` extracts rover/base UBX streams plus a per-epoch reference CSV from a ROS2 moving-base bag or Zenodo zip. `gnss moving-base-signoff` is the validation entrypoint for replay/live runs against those references. `gnss scorpion-moving-base-signoff` wraps the public SCORPION bag flow by chaining prepare, BRDC nav fetch, and replay validation. Together they cover:

- ROS2 bag or zip ingestion with u-blox `NAV-PVT` / `RXM-RAWX` / `NAV-RELPOSNED`
- replay inputs via `--rover-ubx` and `--base-ubx`
- fix rate
- baseline error percentiles
- heading error percentiles
- realtime factor
- decoder error counts
