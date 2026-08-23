# Reference trajectories for perception and map work

## Reproducible bundle and gate

After producing a `.pos`, package it once (this command does not rerun the
solver):

```bash
python3 apps/gnss.py trajectory-bundle \
  --pos output/use_cases/urban_fusion_full/r1_frozen_bundle/fused.pos \
  --reference-csv data/PPC-Dataset/tokyo/run1/reference.csv \
  --output-dir output/use_cases/trajectory/run1_visualization \
  --profile visualization --target-frame vehicle_base_link \
  --lever-arm-m=-0.31,0,0.55
python3 apps/gnss.py trajectory-bundle-validate \
  output/use_cases/trajectory/run1_visualization
```

The bundle contains raw/accepted POS, both KMLs, PNG, segment exceptions,
ROS2 metadata, summary, log, and a hash manifest. `slam_evaluation`,
`map_prototyping`, and `visualization` have separately versioned status,
distance-coverage, horizontal-P95, maximum-gap, and maximum-jump gates.
Validation checks every required artifact (not only the files listed by a
possibly incomplete manifest), input/output hashes, finite frame and lever-arm
metadata, schema, invocation, and the stored gate without positioning again.
It accepts only a `usable` bundle whose stored gate passed; a missing or
tampered artifact is therefore fail-closed. A missing independent reference
always emits `candidate_trajectory`, never `ground_truth`.

The POS rows remain GNSS antenna-phase-center coordinates. The declared
antenna-to-target lever arm is metadata for a consumer with synchronized
attitude; it is not silently rotated into the POS. A SLAM/camera/LiDAR
consumer must reject the bundle if it cannot perform that transform.

Use this route when the deliverable is a trajectory artifact that other teams
consume: camera/LiDAR SLAM evaluation, dataset annotation, or HD-map
prototyping. The output of one RTK run becomes several formats — `.pos`,
KML, plots, ROS2 topics, and Python-readable summaries — so downstream users
can pick their interface.

The quality bar is set by the consumer, not by this repository: an RTK
trajectory is a candidate reference, and its fitness depends on base
distance, sky view, and how much FIX coverage your consumer tolerates.

## 1. Solve once, with KML on

```bash
mkdir -p output/use_cases/ground_truth
python3 apps/gnss.py solve \
  --rover data/PPC-Dataset/tokyo/run1/rover.obs \
  --base data/PPC-Dataset/tokyo/run1/base.obs \
  --nav data/PPC-Dataset/tokyo/run1/base.nav \
  --mode kinematic \
  --preset survey \
  --out output/use_cases/ground_truth/traj.pos \
  --kml output/use_cases/ground_truth/traj.kml
```

`--preset survey` favors fix stability over latency; drop it when the
consumer needs realtime-like behavior and use `--preset low-cost` instead.

## 2. Inspect before publishing

```bash
python3 apps/gnss.py stats output/use_cases/ground_truth/traj.pos
python3 apps/gnss.py plot output/use_cases/ground_truth/traj.pos
python3 apps/gnss.py trackplot output/use_cases/ground_truth/traj.pos \
  output/use_cases/ground_truth/track.png
```

Decide the publishable subset from the `.pos` status column. For a
FIX-only reference track, re-export with `pos2kml --status fixed`:

```bash
python3 apps/gnss.py pos2kml \
  --status fixed \
  output/use_cases/ground_truth/traj.pos \
  output/use_cases/ground_truth/traj_fixed_only.kml
```

## 3. Publish as ROS2 topics

The solution node replays any `.pos` as `NavSatFix`, ECEF `PoseStamped`,
`Path`, status, and satellite-count topics:

```bash
source ros2/install/setup.bash
ros2 run gnss_raw_driver gnss_solution_node --ros-args \
  -p solution_file:=output/use_cases/ground_truth/traj.pos \
  -p frame_id:=earth \
  -p loop:=true
```

Bag this while replaying sensor data to get time-aligned GNSS reference
topics next to camera/LiDAR streams.

## 4. Hand the numbers to Python consumers

With the built bindings (`PYTHONPATH=build/python`), an evaluation harness
can load solutions without parsing text:

```python
from libgnsspp.artifacts import load_pos, pos_stats

records = load_pos("output/use_cases/ground_truth/traj.pos")
stats = pos_stats(records)
print(stats["total_epochs"], stats["status_counts"])
```

Archive `traj.pos`, both KML files, the stats output, and the exact command
line as provenance for the consuming team.

## First artifacts and exit criteria

```bash
test -s output/use_cases/ground_truth/traj.pos
test -s output/use_cases/ground_truth/traj.kml
grep -q '^% LibGNSS++ Position Solution' output/use_cases/ground_truth/traj.pos
```

State the hand-over numbers explicitly: epoch count, fix rate, and the
percentage of the route covered by the consumer's accepted statuses. When a
survey-grade reference exists (as in PPC-Dataset `reference.csv`), score the
candidate first via [the urban fusion route](urban_rtk_fgo.md) before calling
it ground truth.

## Boundary and next step

This route produces artifacts; it does not certify them. An RTK trajectory
without an independent reference is internal evidence only, and FIX-only
subsetting hides exactly the segments where positioning was weak. Antenna
phase-center and lever-arm conventions must match the consumer's frame
definitions.

Next step: agree with the consuming team which statuses and epochs are
acceptable, encode that as a threshold check over `pos_stats()`, and version
the artifacts together with the command that produced them.
