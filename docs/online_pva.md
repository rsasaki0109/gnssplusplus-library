# Evaluate position, velocity and attitude without a receiver

The online API now exports a fresh, timestamped attitude quaternion, Euler
angles, the fixed local coordinate frame, first-heading and heading-health
flags, and estimated IMU biases. The CSV also contains antenna position and
velocity. Missing or stale outputs are NaN, never a plausible identity attitude.

## First run without a dataset

```sh
gnss pva-demo --output-dir new-pva-demo
```

Add `--plot` with matplotlib installed. This tracked, project-authored scoring
witness checks 12 synthetic snapshots, missing initialization and one
deliberate 180 degree heading error. It does not run a GNSS/IMU sensor solver
or demonstrate field accuracy. The last unhealthy heading must still appear
in the primary error report. The demo is included in installed binaries and
the Docker image.

## Replay an external PPC development run

Build `gnss_pva_replay`, or put the installed `bin` directory on PATH. Keep
the public PPC layout `<dataset>/tokyo/run1` (or `nagoya/run1`). Its rover.obs,
base.obs, base.nav, imu.csv and reference.csv are external inputs. The native
replay executable never opens truth; the Python scorer opens it after replay.

```sh
gnss pva-evaluate --run-dir /datasets/PPC-Dataset/tokyo/run1 \
  --output-dir new-tokyo1-evaluation --plot
```

For an out-of-tree build supply
`--replay-binary /build/apps/gnss_pva_replay` (Windows:
`/build/apps/Release/gnss_pva_replay.exe`). Use `--max-epochs 600` for a bounded
120-second development smoke; the default 0 reads the full input. The six
existing PPC runs are development/regression data, not independent holdouts.
PPC does not record network receipt times: replay admits IMU/base records at
each rover event and ephemerides only when both toc and tof have passed.
This preserves source-time causality but does not prove live network latency.

For fixed scenarios append `--scenario gnss_outage --start-s 60 --duration-s 10`
or `--scenario imu_gap --start-s 60 --duration-s 4`. GNSS outage withholds the
rover observations; IMU gap omits the sensor samples and exercises filter
reset/reinitialization. `--scenario loose_only` disables the tight time-update
feedback as an ablation. `--candidate vehicle_nhc_latched_v1` is a frozen,
opt-in vehicle experiment; it does not select production defaults.

The new output directory contains `score.json`, `errors.csv`, hash manifest,
replay log/metadata and raw `replay/pva.csv`; `--plot` adds `errors.png`.
For score-only use:

```sh
gnss pva-evaluate --estimate recorded-pva.csv --reference reference.csv \
  --output-dir new-score
```

## Interpret the report

Position and velocity are antenna-frame ECEF quantities. The scorer transports
errors into the reference ENU frame. Quaternion order is wxyz, body FLU to the
filter's fixed ENU; the recorded rotation transports it to each reference
location before comparison. Euler angles use FRD to NED, aerospace 3-2-1;
heading increases clockwise from north. GPST matches are exact after rounding
to microseconds. There is no interpolation, fitted time offset, mounting angle
or global heading adjustment. The identity IMU xyz-to-FLU convention and PPC
lever arms (Tokyo: 0.31,0,0.55 m; Nagoya: 0.593,-0.670,-1.216 m) are fixed.

Errors include per-axis and norm RMSE, P50, P95 and maximum; circular Euler
errors and full SO(3) rotation error; output fractions/rates; exact time-match
fractions; initial latch/reset recovery delays and processor latency. Primary
heading/rotation statistics require a first heading latch, including every
subsequent unhealthy estimate. Roll/pitch statistics include all fresh attitudes.
Healthy-only metrics are secondary. Missing statistics/recovery are JSON null,
including right-censored cases; they are not zeros.

Stop, low speed, turn and reverse labels come from truth only during offline
scoring. They may overlap. Rules and populations are recorded in the report.
Compare availability and metric counts as well as error magnitudes. Large
errors are retained; availability and heading health are not accuracy guarantees.
The existing `gnss fuse --attitude-csv` batch export uses offline preprocessing
and is a separate comparator, not proof of received-event causality.

## Docker and binary bundles

Build a local image from this branch before its publication:

```sh
docker build -t libgnsspp:pva-dev .
docker run --rm -v "$PWD/out:/out" libgnsspp:pva-dev \
  pva-demo --output-dir /out/pva-demo --plot
docker run --rm -v /datasets/PPC-Dataset:/datasets/PPC-Dataset:ro \
  -v "$PWD/out:/out" libgnsspp:pva-dev pva-evaluate \
  --run-dir /datasets/PPC-Dataset/tokyo/run1 --output-dir /out/tokyo1 --plot
```

No raw PPC log is bundled. The Dockerfile runs the synthetic PVA witness at
image build time; release smoke-install runs it from the installed DEB.
CPack produces TGZ/DEB on Linux and ZIP on Windows. Generate locally after a
full Release build using `cpack --config build/CPackConfig.cmake -C Release`.
Checksums and actual platform verification belong with each artifact; an
existing published image/tag does not contain these new changes yet.
The Linux DEB packager needs `file` and `dpkg-dev` for ELF dependency discovery;
the Docker builder includes these through its package dependencies.

For a Windows ZIP, extract it and add its `bin` to PATH. `gnss.cmd` uses Python
via the Python launcher or Python on PATH. PVA scoring/demo need Python 3.10+;
plots need matplotlib. The optional
Python binding requires the Python version used to build it (3.12 in the local
Windows build). Native executables require the x64 MSVC runtime; it is not
bundled. This is a compiler-free package, with those runtime prerequisites.
Linux TGZ bundles also need Python and the system C/C++ runtime; the DEB declares
its dependencies. SDK consumers additionally need Eigen development headers.

See the [development contract](online_pva_development_plan.md) and
[candidate contract](online_pva_candidate_v1.md) for the frozen acceptance rules.
The [development results](online_pva_development_results.md) include the
negative candidate decision, full-run errors and scenario limitations.
The [local delivery record](online_pva_delivery_v1.json) identifies verified
development ZIP, TGZ, DEB and Docker archives and their SHA256 checksums.
For the exported image, use `docker load -i libgnsspp-pva-1091b8c3-docker.tar`,
then run the examples above with image `libgnsspp:pva-1091b8c3`.
