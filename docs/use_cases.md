# Use cases

Choose the row that matches the input already on your desk. Each route is
intentionally small: make one first artifact, inspect it, then decide whether
the longer validation path is worth the time.

| Who | Input | First output | Rough time | Guide |
|---|---|---|---:|---|
| Autonomous-driving or navigation developer bridging an urban RTK outage | Rover/base/navigation RINEX plus vehicle IMU CSV | Matched RTK and fused `.pos` streams plus KML | 5–15 min for a bounded replay | [Urban RTK + IMU/NHC/ZUPT](use_cases/urban_rtk_fgo.md) |
| Survey/developer evaluating a Japanese static station pair | IGS Tsukuba CRX/RINEX, broadcast nav, final SP3/CLK | Relative static and PPP `.pos`, KML, PNG, summaries, manifest | 1–15 min | [Japan static survey](use_cases/japan_static_survey.md) |
| Monitoring engineer qualifying a displacement-alert pipeline | Four frozen IGS Tsukuba days, SINEX truth, station logs | Daily/6-hour coordinates, covariance, steps, drift, witness score, decision JSON | 5–30 min | [Structural displacement monitoring](use_cases/structural_displacement_monitoring.md) |
| Perception or mapping developer consuming a reference trajectory | A solved `.pos` plus independent PPC Applanix reference | Versioned raw/accepted POS, KML, plot, ROS2 metadata, summary and hash manifest | 1–2 min after solving | [Reference trajectory bundle](use_cases/trajectory_ground_truth.md) |
| Fleet or guidance developer evaluating wide-area corrections | PPC RINEX, QZSS public L6, independent Applanix reference | CLAS solution, decode diagnostics, truth score and application decision | Full replay is compute-intensive | [CLAS/MADOCA sub-meter decision](use_cases/clas_submeter.md) |
| UAV operator evaluating standalone flight continuity | MARS-LVIG raw-GNSS flight container, matching IGS NAV, DJI RTK/attitude truth | POS, motion-population score, KML/PNG and separate navigation/mapping/visualization decisions | 1–3 min after the 10 GB source is local | [UAV navigation and flight continuity](use_cases/uav_navigation.md) |
| RTKLIB user moving an `rnx2rtkp` batch job | Rover/base/navigation RINEX and an RTKLIB-style configuration | One libgnss++ `.pos` file, with optional KML | 5–15 min after the native build | [RTKLIB migration](use_cases/rtklib_migration.md) |
| ROS2 or robotics developer checking a receiver bag | ROS2 package, a UBX/SBF receiver, or an existing bag | Doctor JSON plus a replay `.pos`/KML pair when a bag is available | 5–10 min | [ROS2](use_cases/ros2.md) |
| QZSS L6 / CLAS / MADOCA investigator | Raw L6 bytes, Compact SSR, or L6E/L6D files and matching RINEX | L6 frame/subframe CSV, correction CSV, or PPP summary JSON | 2–15 min | [QZSS L6 and CLAS/MADOCA](use_cases/qzss_l6.md) |

The routes share the repository's [interfaces](interfaces.md),
[validation](validation.md), and [benchmark](benchmarks.md) contracts. A first
artifact is a wiring check, not a claim about field accuracy or deployment
fitness.

For the frozen urban continuity candidate, use the [R1-09 field
checklist](use_cases/urban_rtk_imu_field_checklist.md) to create the one-command
bundle, inspect the KML/score artifacts, and classify a result as usable,
degraded, or unusable.

For the Tsukuba static-survey demonstration, use the [Japan static survey
guide](use_cases/japan_static_survey.md). Its independent truth and antenna
frame boundaries are stricter than a RINEX-header comparison.

## Next application releases

The [primary application roadmap](application_use_case_roadmap.md) puts
user-facing workflows ahead of general infrastructure. Smartphone raw GNSS
(R5) is complete and UAV navigation (R6) concluded with a documented No-Go;
R7 structural displacement monitoring is complete; the active release is
integrity/geofence decisions (R8), followed by
integrity/geofencing (R8) and GNSS timing (R9).
Station operations, RINEX 4, and MADOCA parity work enter this queue only when
an active application gate needs them.

R5 implementation begins at the [smartphone raw GNSS guide](use_cases/smartphone_raw_gnss.md).
R6 evidence and its immutable holdout failure are in the
[UAV navigation guide](use_cases/uav_navigation.md).
