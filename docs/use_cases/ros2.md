# ROS2 receiver and bag replay

Use this route when the input is a ROS2 receiver stream or an existing bag.
It deliberately starts with diagnostics, then checks whether the bag contains
the topics needed by the repository's existing replay node. The first output
is a JSON health report; a replay `.pos`/KML pair is the first position
artifact when a suitable bag is available.

Run dispatcher and `output/use_cases/...` commands from the repository root.
The ROS2 build changes the current shell's environment;
each new shell used for recording or replay must source the same
`ros2/install/setup.bash` file.

## 1. Check the receiver path

The doctor command is runnable before a ROS2 workspace is built. Replace the
device URI with the receiver in scope:

```bash
mkdir -p output/use_cases
python3 apps/gnss.py ros2-doctor \
  --device /dev/ttyUSB0 \
  --json > output/use_cases/ros2_doctor.json
```

The JSON has `root`, `ok`, `checks`, and `commands` keys. A zero exit and an
`ok` value of `true` mean the serial-device checks passed; retain the report
even when they do not. `ros2-doctor --help` lists `--strict` for a CI-style
failure on warnings. A missing device is a diagnostic result, not a reason to
invent a playback result.

If `ros2` or `colcon` is not installed, stop this route after saving the
doctor output and follow the [interfaces guide](../interfaces.md) for the
non-ROS2 CLI surfaces. Do not run the build, launch, or bag commands below
until the ROS2 dependencies are present.

## 2. Inspect an existing bag

For a bag directory, run the repository's bag doctor before playback:

```bash
python3 apps/gnss.py ros2-bag-doctor \
  --bag <bag-directory> \
  --summary-json output/use_cases/ros2_bag_doctor_summary.json
```

The report contains `status`, `replayable_raw_binary`, `topic_status`,
`topics`, `checks`, and `commands`. `status: ready` requires message-level
`/gnss/raw_binary` and `/gnss/fix` data; metadata-only bags are reported as
`partial-metadata`. If there is no bag, skip this step and use the live launch
path below. A skipped step is not evidence that replay is ready.

## 3. Build and use the existing driver/replay path

The commands below are the sequence documented by
`ros2/gnss_raw_driver/README.md`:

```bash
# Run this block from the repository root.
REPO_ROOT="$(pwd)"
cd ros2
colcon build --symlink-install --packages-select gnss_raw_driver
source install/setup.bash
ros2 launch gnss_raw_driver gnss_raw_driver.launch.py \
  device:=/dev/ttyUSB0 baud_rate:=115200 protocol:=auto frame_id:=gnss \
  publish_raw_binary:=true
```

The launch keeps this shell in `$REPO_ROOT/ros2` until you stop it with
Ctrl-C. After stopping it, return to the repository root before running any
dispatcher or output command:

```bash
cd "$REPO_ROOT"
```

In another shell, start from this repository, source the ROS2 workspace again,
and record a small bag while the launch is running:

```bash
cd "$(git rev-parse --show-toplevel)"
source ros2/install/setup.bash
ros2 bag record /gnss/raw_binary /gnss/raw /gnss/fix
```

Then inspect and replay that bag with the existing processor node:

```bash
cd "$(git rev-parse --show-toplevel)"
source ros2/install/setup.bash
python3 apps/gnss.py ros2-bag-doctor \
  --bag <bag-directory> \
  --summary-json output/use_cases/ros2_bag_doctor_summary.json
ros2 bag play <bag-directory>
ros2 run gnss_raw_driver gnss_bag_processor_node --ros-args \
  -p protocol:=auto \
  -p output_pos:=output/use_cases/ros2_bag_replay.pos \
  -p output_kml:=output/use_cases/ros2_bag_replay.kml
```

The driver dependencies are a ROS2 installation, `colcon`, the
`gnss_raw_driver` package, and a receiver/serial permission for live capture.
Replay additionally needs a bag containing the topics named above. The
processor's `protocol:=auto` may be changed only to a protocol supported by
the existing node; inspect the package README before doing so.

## First artifacts and exit criteria

The diagnostic exit is the JSON file and its `ok`/`status` fields. The replay
exit is a zero-returning processor with non-empty
`output/use_cases/ros2_bag_replay.pos` and, when requested,
`output/use_cases/ros2_bag_replay.kml`:

```bash
test -s output/use_cases/ros2_doctor.json
test -s output/use_cases/ros2_bag_doctor_summary.json
test -s output/use_cases/ros2_bag_replay.pos
test -s output/use_cases/ros2_bag_replay.kml
grep -q '^% LibGNSS++ Position Solution' \
  output/use_cases/ros2_bag_replay.pos
```

Use the [validation guide](../validation.md) to decide what to verify in the
solution, and the [benchmark notes](../benchmarks.md) for offline comparisons.
The [interfaces guide](../interfaces.md) describes the node and artifact
contracts. These commands check wiring and reproducibility; they do not
establish production real-time safety, scheduler margin, or localization
safety.

## Boundary and next step

This route covers the repository's raw-binary topics and bag processor. It is
not a claim that every ROS2 distribution, receiver protocol, QoS profile, TF
tree, or estimator integration is supported. A missing ROS2 installation or
bag has a clear skip path, not a passing replay result.

Next step: archive the doctor JSON, bag summary, exact launch/playback
arguments, and the validation evidence for the downstream node or estimator
before treating the artifact as a candidate input.
