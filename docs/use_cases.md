# Use cases

Choose the row that matches the input already on your desk. Each route is
intentionally small: make one first artifact, inspect it, then decide whether
the longer validation path is worth the time.

| Who | Input | First output | Rough time | Guide |
|---|---|---|---:|---|
| RTKLIB user moving an `rnx2rtkp` batch job | Rover/base/navigation RINEX and an RTKLIB-style configuration | One libgnss++ `.pos` file, with optional KML | 5–15 min after the native build | [RTKLIB migration](use_cases/rtklib_migration.md) |
| ROS2 or robotics developer checking a receiver bag | ROS2 package, a UBX/SBF receiver, or an existing bag | Doctor JSON plus a replay `.pos`/KML pair when a bag is available | 5–10 min | [ROS2](use_cases/ros2.md) |
| QZSS L6 / CLAS / MADOCA investigator | Raw L6 bytes, Compact SSR, or L6E/L6D files and matching RINEX | L6 frame/subframe CSV, correction CSV, or PPP summary JSON | 2–15 min | [QZSS L6 and CLAS/MADOCA](use_cases/qzss_l6.md) |

The routes share the repository's [interfaces](interfaces.md),
[validation](validation.md), and [benchmark](benchmarks.md) contracts. A first
artifact is a wiring check, not a claim about field accuracy or deployment
fitness.
