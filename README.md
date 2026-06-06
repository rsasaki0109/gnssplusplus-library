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

![Feature overview](docs/libgnsspp_feature_overview.png)

## Results

| Area | Public comparison | Current result |
|---|---|---|
| RTK | PPC Tokyo/Nagoya vs RTKLIB `demo5` | +17.0 pp positioning, +28.1 pp official score, -11.96 m P95 H delta |
| CLAS PPP | QZSS CLAS vs CLASLIB | 3.57 mm vs 7.29 mm RMS 3D |
| Urban RTK | UrbanNav Tokyo Odaiba vs RTKLIB `demo5` | More fixes, lower Hp95/Vp95; `--preset odaiba` closes Hmed |
| SPP | PPC SPP adaptive robust + policy gate | No P95 regression with <=1 pp positioning drop |

![PPC RTK coverage scorecard](docs/ppc_rtk_demo5_scorecard.png)

| CLASLIB 2D | libgnss++ 2D |
|---|---|
| ![CLASLIB 2D](docs/clas_claslib_2d.png) | ![libgnss++ CLAS 2D](docs/clas_native_2d.png) |

| RTKLIB Odaiba 2D | libgnss++ Odaiba 2D |
|---|---|
| ![RTKLIB Odaiba 2D](docs/driving_odaiba_comparison_rtklib_2d.png) | ![libgnss++ Odaiba 2D](docs/driving_odaiba_comparison_libgnss_2d.png) |

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
