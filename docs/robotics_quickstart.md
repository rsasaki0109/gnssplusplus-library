# Robotics Quick Start

This guide is for robotics developers who want to prove that libgnss++ can
ingest real GNSS data, publish ROS2-friendly outputs, and leave replayable
artifacts behind.

Use this path when your next consumer is Autoware, Nav2, a localization stack,
or a rosbag-based field workflow.

## 10-minute offline loop

Build the native tools:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
python3 apps/gnss.py doctor
```

Run a short RTK replay from the bundled PPC-Dataset checkout:

```bash
mkdir -p output/adoption

python3 apps/gnss.py solve \
  --rover data/PPC-Dataset/tokyo/run1/rover.obs \
  --base data/PPC-Dataset/tokyo/run1/base.obs \
  --nav data/PPC-Dataset/tokyo/run1/base.nav \
  --mode kinematic \
  --preset low-cost \
  --ratio 2.4 \
  --max-epochs 200 \
  --out output/adoption/tokyo_run1_rtk.pos \
  --kml output/adoption/tokyo_run1_rtk.kml
```

Open the local dashboard:

```bash
python3 apps/gnss.py web --port 8085 --root .
```

Then visit `http://127.0.0.1:8085`. The dashboard reads existing artifacts
from `output/`, so it is useful even when a run was produced by CI, Docker, or
a field laptop.

## Add a quality gate

For robotics work, do not stop at "a trajectory was produced". Run a sign-off
command that emits machine-readable metrics and fails on explicit thresholds:

```bash
python3 apps/gnss.py ppc-rtk-signoff \
  --dataset-root data/PPC-Dataset \
  --city tokyo \
  --run run1 \
  --preset low-cost \
  --ratio 2.4 \
  --max-epochs 200 \
  --out output/adoption/tokyo_run1_signoff.pos \
  --summary-json output/adoption/tokyo_run1_signoff.json \
  --require-positioning-rate-min 70 \
  --require-effective-epoch-rate-min 5
```

The resulting JSON is the artifact to archive in CI or attach to a regression
report. It records positioning rate, fix rate, timing, thresholds, and dataset
provenance.

## Realtime smoke test

Robotics users need to know whether a profile can keep up with recorded sensor
time. `gnss robotics-smoke` runs a short PPC RTK replay and gates the runtime
metrics that matter for deployment triage:

```bash
python3 apps/gnss.py robotics-smoke \
  --profile realtime \
  --max-epochs 200 \
  --realtime-factor-min 1.0 \
  --effective-epoch-rate-min 5.0
```

Profiles:

| Profile | Purpose | Default gate |
|---|---|---|
| `quick` | Fast wiring check for build/data/artifact flow | 50 epochs, no realtime gate |
| `realtime` | Robotics 5 Hz replay triage | 200 epochs, `realtime_factor >= 1.0`, `effective_epoch_rate_hz >= 5.0` |
| `full` | Dataset-level availability plus runtime gate | Full run, positioning rate plus realtime gates |

The summary includes:

| Metric | Meaning |
|---|---|
| `solver_wall_time_s` | Host wall-clock time spent solving the replay |
| `realtime_factor` | Dataset time span divided by solver wall time |
| `effective_epoch_rate_hz` | Solved epochs per wall-clock second |

For a 5 Hz GNSS stream, `realtime_factor >= 1.0` and
`effective_epoch_rate_hz >= 5.0` mean the offline solver is faster than the
recorded stream on that machine. Live deployment still needs serial/NTRIP I/O
margin, but this smoke catches obvious CPU regressions before field testing.

`positioning_rate_pct` is computed against the full reference trajectory, so it
is intentionally not gated by default for bounded `--max-epochs` smoke runs.
For a full replay, add `--positioning-rate-min <pct>` when you want availability
and runtime checked together.

Open the web UI after the smoke run:

```bash
python3 apps/gnss.py web --port 8085 --root .
```

The **Robotics realtime smoke** panel shows pass/fail state, realtime factor,
epoch rate, wall time, solution span, tuning knobs, threshold comparisons, and
direct links to the summary JSON, `.pos`, reference, rover/base/nav, and run
directory artifacts. That panel is the fastest way to debug whether a robotics
run is CPU-bound, data-limited, or failing a gate. Failed runs show `why:`
lines such as `realtime factor 0.98 < 1.00` or
`effective epoch rate 4.9 Hz < 5.0 Hz`.

## Docker path

The runtime image installs the `gnss` dispatcher on `PATH`. Mount the repo or a
dataset directory into `/workspace`:

```bash
docker build -t libgnsspp:latest .

docker run --rm -it -p 8085:8085 -v "$PWD:/workspace" \
  libgnsspp:latest web --host 0.0.0.0 --port 8085 --root /workspace
```

For batch runs:

```bash
docker run --rm -it -v "$PWD:/workspace" libgnsspp:latest solve \
  --rover /workspace/data/PPC-Dataset/tokyo/run1/rover.obs \
  --base /workspace/data/PPC-Dataset/tokyo/run1/base.obs \
  --nav /workspace/data/PPC-Dataset/tokyo/run1/base.nav \
  --mode kinematic \
  --preset low-cost \
  --ratio 2.4 \
  --max-epochs 200 \
  --out /workspace/output/adoption/tokyo_run1_rtk.pos
```

## ROS2 receiver loop

The ROS2 package is under `ros2/gnss_raw_driver`. It publishes:

| Topic | Type | Use |
|---|---|---|
| `/gnss/fix` | `sensor_msgs/NavSatFix` | Standard robot localization input |
| `/gnss/raw` | `gnss_raw_driver/GnssRawEpoch` | Parsed pseudorange/carrier/Doppler/SNR |
| `/gnss/raw_binary` | `std_msgs/UInt8MultiArray` | Lossless rosbag capture for replay |

Run the ROS2 field doctor before plugging the output into a robot:

```bash
python3 apps/gnss.py ros2-doctor --device /dev/ttyUSB0
```

It checks whether `ros2` and `colcon` are on `PATH`, whether the package has
been built and sourced, whether the serial device exists and is readable, and
prints copy-ready launch, rosbag record, and topic debug commands. Use
`--json` when you want the same checks in CI or a field-laptop diagnostic
report.

Build the ROS2 package:

```bash
cd ros2
colcon build --symlink-install --packages-select gnss_raw_driver
source install/setup.bash
```

Start a serial receiver. `protocol:=auto` detects UBX or SBF once enough bytes
arrive:

```bash
ros2 launch gnss_raw_driver gnss_raw_driver.launch.py \
  device:=/dev/ttyUSB0 \
  baud_rate:=115200 \
  protocol:=auto \
  frame_id:=gnss \
  publish_raw_binary:=true
```

Record the raw stream and the standard fix topic:

```bash
ros2 bag record /gnss/raw_binary /gnss/raw /gnss/fix
```

Inspect the bag before you leave the test site:

```bash
python3 apps/gnss.py ros2-bag-doctor \
  --bag <bag-directory> \
  --summary-json output/ros2_bag_doctor_summary.json
```

The doctor reads `metadata.yaml` plus sqlite storage directly, so it does not
need ROS2 Python imports. For sqlite bags it reports topic types, message
counts, duration, estimated rates, timestamp gaps, and whether
`/gnss/raw_binary` is available for lossless decoder replay. For MCAP bags, it
uses the optional Python `mcap` package when available to measure message-level
rates and gaps from the `.mcap` file. If that package is not installed, it falls
back to `metadata.yaml` and reports `partial-metadata`: topic presence, message
counts, and duration are checked, while message rates and timestamp gaps are
clearly marked as not measured. Missing `/gnss/raw_binary` means researchers can
still inspect `/gnss/fix`, but cannot replay improved decoders from the bag.

Create a field handoff report:

```bash
python3 apps/gnss.py field-report \
  --out output/field_report.md \
  --json-out output/field_report.json
```

The report aggregates setup readiness, ROS2 launch/record diagnostics, bag
doctor summaries, realtime smoke results, and next commands into one Markdown
file plus a machine-readable JSON artifact.
`gnss web` auto-discovers `output/field_report*.json`, links the Markdown/JSON
report, previews the Markdown, and shows the same next commands in the browser
for field debugging.

Decode a recorded raw-binary bag into ROS topics plus file artifacts:

```bash
ros2 bag play <bag-directory>

ros2 run gnss_raw_driver gnss_bag_processor_node --ros-args \
  -p protocol:=auto \
  -p output_pos:=output/adoption/bag_replay.pos \
  -p output_kml:=output/adoption/bag_replay.kml
```

## Integration checks

Use this checklist before wiring the output into a robot localization graph:

| Check | Why it matters |
|---|---|
| `frame_id` matches your localization frame contract | Prevents silent TF mistakes |
| `/gnss/fix.status.status` changes across no-fix, float, fixed states | Downstream filters can gate bad epochs |
| covariance is present on `/gnss/fix` | EKF/graph weighting is not arbitrary |
| `/gnss/raw_binary` is recorded | Decoder improvements can be replayed later |
| `gnss ros2-bag-doctor` summary is archived | Topic gaps and missing replay data are visible after the field test |
| a `*-signoff` JSON is archived | Performance regressions become comparable |

## Next steps

- Use [Dataset Gallery](dataset_gallery.md) to add a public or internal dataset.
- Use [Validation](validation.md) to choose the right sign-off command.
- Use [Interfaces](interfaces.md) when you need CLI, Python, ROS2, and web entrypoints in one workflow.
