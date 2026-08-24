# UAV navigation and flight continuity (R6)

Status: development navigation and visualization pass the frozen profile;
mapping is No-Go. The untouched holdout run1 stopped fail-closed at the public
MCAP derivative's invalid embedded schema encoding, so the R6 release decision
is **No-Go for navigation and visualization promotion**. The failure is
preserved without reader changes, threshold changes, or a second holdout run.

## Application boundary

This route targets one MARS-LVIG flight container containing ZED-F9P raw GNSS,
Livox IMU, DJI attitude, independent DJI RTK position/yaw, and fixed CAD antenna
offsets. It produces an antenna-phase-centre standalone trajectory. It does not
claim body-origin accuracy, centimetre accuracy, a production flight controller,
or a mapping reference.

MARS-LVIG's [official dataset page](https://mars.hku.hk/dataset.html) does not
state a dataset licence. The article record is CC BY-NC-ND 4.0, which is not
evidence that downloaded flight containers may be redistributed. Keep source
data under ignored `data/`, fetch it for local evaluation, and do not ship it
with libgnss++ artifacts.

The frozen profile is
`configs/benchmarks/uav_r6_mars_lvig_development.json`. It records source sizes,
SHA-256 values, topic counts, frame/time contracts, IGS broadcast NAV products,
segmentation rules, and thresholds.

## One-command route

After acquiring the named flight and uncompressed IGS mixed broadcast NAV, run:

```bash
python3 apps/gnss.py uav-mars-workflow \
  --input data/mars_lvig/cache/HKisland_GNSS03.bag \
  --navigation data/mars_lvig/navigation/BRDC00IGS_R_20232970000_01D_MN.rnx \
  --profile configs/benchmarks/uav_r6_mars_lvig_development.json \
  --role development \
  --output-dir output/uav_r6/development/reproduction
```

Use `--max-epochs 600` for a bounded acquisition/adapter/native-SPP smoke. A
bounded run deliberately does not spend a sign-off population decision because
the required cruise, turn, climb/descent, hover, and landing populations may be
incomplete.

The workflow verifies the container and NAV hashes before reading them, then
runs the adapter, native `gnss spp`, frozen sign-off, KML, PNG, and hash
manifest stages. Important outputs are:

- `uav_observations.rnx`: mixed GPS/GLO/GAL/BDS RINEX 3.04 observations;
- `imu.csv`, `attitude.csv`, `rtk_truth.csv`, `rtk_yaw.csv`, `rtk_status.csv`;
- `libgnsspp_spp.pos` and `libgnsspp_spp_summary.json`;
- `uav_matches.csv` and `uav_signoff_summary.json`;
- `libgnsspp_spp.kml`, `libgnsspp_spp_trajectory.png`, and
  `workflow_manifest.json`.

The raw receiver PVT is used only for the solver's approximate initial ECEF
position. DJI RTK fields are adapter outputs and scoring truth; they never seed
or correct the native solution.

## Frames, time, and lever arm

The actual bag declares Livox IMU frame `livox_frame` and DJI attitude frame
`body_FLU`. DJI's ROS conversion defines the quaternion as body FLU to ground
ENU. Attitude alignment therefore compares RTK yaw with
`(-yaw_ccw(body_FLU→ENU)) mod 360` and fits no offset.

DJI's RTK `NavSatFix` leaves `frame_id` empty. The adapter accepts that only
because the frozen profile explicitly declares WGS84 latitude/longitude and
ellipsoidal height at the DJI RTK antenna phase centre. Any other undefined
frame fails.

CAD places the raw and truth antenna phase centres 0.615727 m apart in the
LiDAR frame. The LiDAR-to-DJI-body fixed rotation is not authoritatively
resolved, so the adapter does not rotate or apply that vector. Navigation
errors are antenna-to-antenna comparisons with the 0.615727 m reference-point
mismatch reported separately. Vehicle NHC is always disabled.

Sensor truth is UTC; raw observations preserve GPST week/TOW and use 18 leap
seconds for 2023 comparison. Truth interpolation is allowed only across
adjacent RTK samples no farther than 0.25 s apart.

## Frozen development evidence

HKisland_GNSS03 used the official ROS1 bag SHA-256
`8ce44d1d...c5dc317` and 2023 DOY 297 IGS NAV SHA-256
`d5135d67...7966288`. Native SPP produced 3,911/3,911 solutions.

| Population | Epochs | Horizontal p95 | Absolute vertical p95 |
|---|---:|---:|---:|
| Whole route | 3,907 | 6.253 m | 8.464 m |
| Cruise | 2,030 | 6.362 m | 8.941 m |
| Turn | 552 | 6.005 m | 8.954 m |
| Climb/descent | 666 | 4.656 m | 7.097 m |
| Hover | 1,146 | 5.235 m | 8.050 m |
| Landing tail | 300 | 4.680 m | 5.915 m |

Availability was 100%, truth coverage 99.898%, maximum solution gap 0.100 s,
maximum horizontal jump 2.135 m, attitude/RTK-yaw p95 6.322 degrees, and both
RTK fixed-status ratios 100%. All 28 frozen gates passed. The immutable summary
is `docs/use_cases/records/uav_r6_development.json`.

Development consumer decisions:

- navigation: Pass for the named standalone antenna trajectory and thresholds;
- visualization: Pass;
- mapping: No-Go because independent truth is four-DoF and the fixed
  LiDAR/body rotation is unresolved.

## Untouched holdout run1

Thresholds were committed at `0dfcc4e`; the derivative container, NAV, and
workflow were sealed at `5f14b31`, `5e17ba2`, and `604b3ce`. HKairport_GNSS03
run1 used the published MCAP SHA-256 `d85e39b9...7951de7`. Acquisition passed,
but the adapter stopped before deserializing messages because `rosbags` rejected
an embedded MCAP schema with an empty encoding (`KeyError: ''`). No position,
truth score, KML, or PNG was produced.

The failure is a container/provenance compatibility failure, not an accuracy
result. It makes navigation, mapping, and visualization promotion No-Go. The
immutable record is `docs/use_cases/records/uav_r6_holdout_run1.json`. Fixing
the reader and trying the same holdout would tune on an opened holdout; a future
R6 attempt needs a new versioned profile and a different sealed flight.

## Reject the workflow when

- source size/hash, NAV date/hash, or expected topic counts differ;
- a quota/login page is returned instead of the frozen ROS1/MCAP magic;
- time moves backwards, truth interpolation crosses a gap, or required motion
  populations are empty;
- IMU/attitude frames are empty or differ from the frozen contract;
- lever-arm values are missing/non-finite or vehicle NHC is enabled;
- attitude quaternions are non-unit, RTK fixed status is insufficient, or any
  frozen coverage/error/gap/jump/yaw/landing gate fails;
- a consumer asks for body-origin or mapping accuracy from this four-DoF truth.
