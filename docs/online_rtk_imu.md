# Received-event RTK and IMU processing

`OnlineRtkImuProcessor` and `gnss_online` provide a separate received-event
route. The processor accepts navigation, base observations, body-FLU IMU
samples and rover observations one event at a time. It does not open a data
file, scan later observations, smooth outputs or interpolate a future base.
Existing batch commands retain their existing behavior.

## Build and input

```sh
cmake --build <build> --config Release --target gnss_online gnss_online_tests
<build>/apps/Release/gnss_online --help
```

On a single-configuration generator the executables have no `Release` directory.
Start the executable with a known reference antenna ECEF position in metres:

```sh
gnss_online --base-ecef X Y Z --lever-arm forward left up \
  --rtk-out new_rtk.pos --fused-out new_fused.pos
```

Supply these space-separated event envelopes on stdin. Every week/TOW pair
uses normalized GPST; the reception pair must be monotone across all events.

| Event | Remaining fields |
| --- | --- |
| `NAV` | reception week/TOW, one complete CRC-valid RTCM3 ephemeris frame in hexadecimal |
| `BASE` | reception week/TOW, observation week/TOW, all RTCM3 observation frames for that epoch |
| `ROVER` | reception week/TOW, observation week/TOW, all RTCM3 observation frames for that epoch |
| `IMU` | reception week/TOW, sample week/TOW, ax ay az gx gy gz |
| `RESET` | reception week/TOW |

Each event occupies one line. Acceleration uses m/s² and gyro uses rad/s,
both in Forward/Left/Up axes. The lever arm is IMU to antenna in the same
axes. The input adapter must assemble all observation frames for an epoch;
the command checks frame CRC, TOW and duplicate satellite/signal identities.
The full week comes from the envelope. Empty lines and `#` comments are allowed.
A malformed event stops with its line number and a nonzero exit code.

The command flushes one CSV record after every `ROVER` line, before reading
the next event. Optional POS sinks contain only valid solutions and must be
new files. CSV includes every rover event, including missing solutions.

## Time and availability contract

An observation/sample timestamp cannot exceed its reception timestamp. A
received IMU sample later than a delayed rover epoch remains queued. It is
not mechanized until a rover epoch reaches that sample time. Late or duplicate
IMU samples are dropped; late base epochs cannot revise emitted outputs.

RTK uses an already-received base epoch at the same observation timestamp.
There is no waiting or interpolation. If the exact base is absent, the RTK
stream emits an SPP fallback or a missing solution. With 1 Hz base and 5 Hz
rover input this means differential processing is available only at the
common timestamps. This availability cost must be measured explicitly.

GNSS feedback to the fusion state and tight RTK time updates require an IMU
sample at the rover epoch. An older prediction retains its actual timestamp;
the default maximum fusion age is 0.02 s. Propagation without an accepted
position correction reports `PROPAGATED`. Fusion requires fresh trailing
IMU alignment and a usable GNSS origin. Tight time updates additionally
require heading convergence and a valid FLOAT posterior anchor. `--loose-only`
disables that optional feedback path.

An IMU gap greater than 0.1 s, a stale last IMU sample, or a rover gap greater
than 2 s resets the RTK/fusion filters. Reinitialization uses newly received
trailing samples; it cannot reuse the old attitude alignment. Each reset
increments `reset_generation`. Explicit `RESET` clears pending observations
and IMU samples while retaining received broadcast navigation. It preserves
the monotone reception watermark.

Pending queues have explicit capacities (10,000 IMU samples and 16 base
epochs by default). Overflow rejects the event. Broadcast storage retains
at most eight records per satellite and satellite-state caches are scoped
to one processing epoch.

## Output and provenance

CSV records contain input age (reception minus rover timestamp), fusion age
(rover minus actual fused timestamp), processing wall time, exact-base
availability, IMU consumption, initialization/heading state, supplied tight
update, reset generation and fallback/reset reason. Antenna ECEF coordinates
are in metres. Reception timestamps describe adapter-provided availability;
they must come from actual receipt for a live causality claim. Assigning a
timestamp to a preloaded navigation file does not establish when its records
were broadcast or received.

This route is distinct from fixed-lag FGO and existing whole-file fusion.
Batch scores do not prove its accuracy or latency. Acceptance requires native
queue/reset tests, numerical prefix parity on usable raw PPC outputs,
late/missing-base and IMU-gap replays, and measured missing outputs and latency.
The processor and executable build, and all nine queue/reset tests pass.
Numerical real-data prefix parity and outage/latency checks remain pending.
