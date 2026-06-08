# gnss_raw_driver

ROS2 driver package for GNSS receivers that speak UBX or SBF binary protocols.

It publishes:

| Topic | Type |
|---|---|
| `/gnss/fix` | `sensor_msgs/NavSatFix` |
| `/gnss/raw` | `gnss_raw_driver/GnssRawEpoch` |
| `/gnss/raw_binary` | `std_msgs/UInt8MultiArray` |

## Build

From the libgnss++ repository root:

```bash
python3 apps/gnss.py ros2-doctor --device /dev/ttyUSB0
```

The doctor checks ROS2/colcon availability, workspace source state, installed
driver binaries, serial permissions, and prints the matching launch, record,
and topic debug commands.

```bash
cd ros2
colcon build --symlink-install --packages-select gnss_raw_driver
source install/setup.bash
```

## Run a serial receiver

```bash
ros2 launch gnss_raw_driver gnss_raw_driver.launch.py \
  device:=/dev/ttyUSB0 \
  baud_rate:=115200 \
  protocol:=auto \
  frame_id:=gnss \
  publish_raw_binary:=true
```

Use `protocol:=ubx` or `protocol:=sbf` to skip auto-detection.

## Record replayable data

```bash
ros2 bag record /gnss/raw_binary /gnss/raw /gnss/fix
```

`/gnss/raw_binary` is the lossless stream. Keep it in field bags so decoder and
solver changes can be replayed without returning to the test site.

Inspect the recorded bag before archiving it:

```bash
python3 apps/gnss.py ros2-bag-doctor \
  --bag <bag-directory> \
  --summary-json output/ros2_bag_doctor_summary.json
```

The bag doctor checks topic presence, message counts, estimated rates,
timestamp gaps, and whether `/gnss/raw_binary` is present for replayable
decoder research. The summary JSON is displayed by `gnss web`.

Create a field handoff report:

```bash
python3 apps/gnss.py field-report --out output/field_report.md
```

This aggregates setup, ROS2, bag, and realtime-smoke diagnostics into one
Markdown file plus a JSON report for issue attachments and research handoff.

## Offline bag processor

Play a bag in one terminal:

```bash
ros2 bag play <bag-directory>
```

Decode in another terminal:

```bash
ros2 run gnss_raw_driver gnss_bag_processor_node --ros-args \
  -p protocol:=auto \
  -p output_pos:=output/ros2_bag_replay.pos \
  -p output_kml:=output/ros2_bag_replay.kml
```

See `docs/robotics_quickstart.md` for the full robotics adoption flow.
