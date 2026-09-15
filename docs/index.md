# libgnss++

Native C++20 GNSS stack for **SPP, RTK, PPP, CLAS/MADOCA, RTCM, UBX, and QZSS L6** —
no RTKLIB runtime dependency, no GUI required.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j
python3 apps/gnss.py doctor
```

That is it: build, verify your machine, then pick a path below.

## Pick your path

| I want to… | Start here | Time |
|---|---|---|
| Get a first solution with bundled data | [Quick Start](quickstart.md) | ~5 min |
| Prove it works with **zero installs** | [Self-contained offline demo](self_contained_demo.md) | ~1 min |
| Wire GNSS into a robot / ROS2 | [Robotics Quick Start](robotics_quickstart.md) | ~10 min |
| Move an existing `rnx2rtkp` job over | [RTKLIB migration](use_cases/rtklib_migration.md) | ~10 min |
| Deploy urban RTK with IMU continuity | [R1 urban RTK/IMU field checklist](use_cases/urban_rtk_imu_field_checklist.md) | ~15 min |
| Match my dataset / application | [Use cases](use_cases.md) | varies |

## Run it in one command (Docker)

No build, no dependencies — mount only the output directory:

```bash
docker pull ghcr.io/rsasaki0109/gnssplusplus-library:v0.2.0
mkdir -p output
docker run --rm -v "$PWD/output:/workspace/output" \
  ghcr.io/rsasaki0109/gnssplusplus-library:v0.2.0 \
  demo --output-dir /workspace/output/self-contained-demo
```

This is an offline plumbing check, not a field-accuracy claim.

## What it does

- **Positioning**: SPP, RTK (static/kinematic/moving-base), PPP, PPP-AR, CLAS/MADOCA.
- **Fusion**: GNSS + IMU FGO, NHC/ZUPT constraints, integrity gates.
- **I/O & interfaces**: RINEX 3/4, RTCM, UBX, NTRIP/serial streaming, ROS2, Python, browser UI.

| Understand the design | Validate results | Explore interfaces |
|---|---|---|
| [Architecture](architecture.md) | [Validation & sign-off](validation.md) | [Interfaces](interfaces.md) |
| [IMU fusion](imu_fusion.md) | [Benchmarks](benchmarks.md) | [Quick Start commands](quickstart.md) |
| [Tight coupling](tight_coupling.md) | [PPC reproduction](ppc_reproduction.md) | [Experiments](experiments.md) |

## Visuals

![Feature overview](libgnsspp_feature_overview.png)
![Architecture diagram](libgnsspp_architecture.png)

## Release & community

- [v0.2.0 release highlights](releases/v0.2.0.md)
- [Maintainer release runbook](release_runbook.md)
- [Community onboarding](community.md) · [Contributing](contributing.md)
- [Dataset gallery](dataset_gallery.md)
