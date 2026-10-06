# Online position velocity and attitude evaluation

The deliverable is a hardware-free, reproducible public-log workflow for
position, velocity and attitude quality, including scenarios, improvement
decisions and runnable Docker/binary distribution artifacts.

The baseline is the received-event implementation merged in PR #559. Additive
attitude/velocity export must preserve its position, status, queue and reset
behavior. The existing batch attitude export is an offline comparator, with
its future-input preprocessing clearly distinguished from received-event
processing. Raw PPC inference never opens reference.csv.

Develop on Tokyo run1; the existing six PPC runs are development/regression
data. Closed application holdouts and paused smartphone work remain closed.
Record source, configuration, binary and input hashes and full raw argv. A
bounded smoke establishes wiring; full runs establish development metrics.

Quaternion is body FLU to the filter's fixed local ENU frame. Published RPY is
body FRD to world NED, aerospace 3-2-1, degrees; heading is clockwise from north.
GPST and the original IMU axis mapping/lever arms are fixed. Evaluation converts
the fixed local frame to each reference location explicitly. No fitted global
heading, mounting offset, time shift, retrospective initialization or smoothing
may improve the reported online result. Unavailable fields are NaN in CSV and
null in JSON; initial heading is not qualified until its latch is observed.

Report position/velocity norm and per-axis errors, circular roll/pitch/heading,
SO(3) rotation error, RMSE/P50/P95/maximum, availability, exact time matching,
uninitialized states, first alignment delay, resets/recovery delay and latency.
Use all available latched attitudes for primary errors, including unhealthy
heading; report healthy-only subsets separately. Reference labels are offline
only. Stops (<0.2 m/s), low speed (0.2–2 m/s), turns (>5 deg/s at >=1 m/s),
reverse (reference forward velocity <-0.5 m/s) and ordinary motion may overlap.
Scenario GNSS removal and IMU gaps use fixed elapsed-time windows; their
coverage and post-event recovery are retained, including right censoring.

After baseline diagnosis, freeze one causal candidate and its acceptance
contract before its comparison. Across six development runs require no more
than 1% regression in position, velocity and full-rotation RMSE/P95, no more
than 0.1 percentage point availability loss, no slower initial heading or
scenario recovery, and at least one targeted attitude metric improvement.
Runtime is reported with host contention. Failure keeps the original default
and records No-Go; threshold tuning after seeing comparison results is not
permitted in that experiment.

Ship a one-command scorer with JSON/CSV/plots, independent math/missing-data
tests, native frame/snapshot tests, real prefix parity and outage checks. Re-run
the applicable broad CLI/benchmark/binding/package/native checks. Docker and
binary bundles must expose the same command and a tracked offline smoke; record
platform/runtime requirements and actual distribution verification. Registry or
release publication is a separate action.
