# libgnss++

Modern C++17 GNSS toolkit for non-GUI positioning.

Native `SPP`, `RTK`, `PPP`, `CLAS/MADOCA`, `RTCM`, `UBX`, and direct `QZSS L6`
handling without an external RTKLIB runtime.

![UrbanNav Odaiba social card](docs/driving_odaiba_social_card.png)

## What You Get

- Solvers: `gnss spp`, `gnss solve`, `gnss ppp`
- Inputs/tools: RINEX, RTCM, UBX, SBF, NMEA, BINEX, QZSS L6
- Products: `SP3`, `CLK`, `IONEX`, `DCB`
- Extras: benchmarks, web dashboard, Python bindings, Docker, ROS 2 playback

## Quick Start

Build:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

Run a solution:

```bash
python3 apps/gnss.py spp \
  --obs data/rover_static.obs \
  --nav data/navigation_static.nav \
  --out output/spp_solution.pos
```

RTK example:

```bash
python3 apps/gnss.py solve \
  --rover data/rover_kinematic.obs \
  --base data/base_kinematic.obs \
  --nav data/navigation_kinematic.nav \
  --mode kinematic \
  --out output/rtk_solution.pos
```

Run the web UI:

```bash
python3 apps/gnss.py web --port 8085
```

Then open `http://127.0.0.1:8085`.

List commands:

```bash
python3 apps/gnss.py --help
```

## Docker

```bash
docker build -t libgnsspp:latest .
docker run --rm -it -p 8085:8085 -v "$PWD:/workspace" \
  libgnsspp:latest web --host 0.0.0.0 --port 8085 --root /workspace
```

## Benchmarks

The repo includes PPC, UrbanNav, CLASLIB, and MADOCA/MADOCALIB sign-off tools.
The long tables and reproduction commands are kept in docs:

- [Benchmarks](docs/benchmarks.md)
- [Validation](docs/validation.md)
- [PPC reproduction commands](docs/ppc_reproduction.md)
- [SPP accuracy notes](docs/references/spp-accuracy-improvement.md)

## Docs

- <https://rsasaki0109.github.io/gnssplusplus-library/>
- [Documentation index](docs/index.md)
- [Quick start](docs/quickstart.md)
- [Interfaces](docs/interfaces.md)
- [Architecture](docs/architecture.md)
- [Reference analyses](docs/references/index.md)
- [Contributing](CONTRIBUTING.md)

## Install

```bash
cmake --install build --prefix /opt/libgnsspp
/opt/libgnsspp/bin/gnss --help
```

## Tests

```bash
ctest --test-dir build --output-on-failure
```

## License

MIT License. See [LICENSE](LICENSE).
