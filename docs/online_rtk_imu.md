# Received-event RTK and IMU processing

`OnlineRtkImuProcessor` and `gnss_online` provide a separate received-event
route. The processor accepts navigation, base observations, body-FLU IMU
samples and rover observations one event at a time. It does not open a data
file, scan later observations, smooth outputs or interpolate a future base.
Existing batch commands retain their existing behavior.

CSV schema v2 appends fresh attitude (wxyz, body FLU to fixed ENU), its source
time and fixed frame rotation, FRD/NED Roll/Pitch/Heading, estimated IMU biases
and antenna ECEF velocities. The first 27 columns are preserved. An available
attitude does not imply observed heading: `heading_aligned` is the first latch,
while `heading_converged` is recent innovation health. Uninitialized/stale
quaternions, frame rotations and angles are NaN. See the
[PVA workflow](online_pva.md) for public-log scoring and distribution demos.

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

The decoder receives the event's GPST reception time as the context for
truncated GPS week and GLONASS day fields. Historical replay does not use the
current PC date for those fields. The public `RTCMProcessor::setReferenceTime`
also exposes this context to adapters; callers that omit it retain the legacy
host-clock decoding behavior. `clear()` removes the explicit context.

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

An SPP fallback between exact base epochs leaves the RTK differential filter
unchanged and preserves the short IMU interval. The next exact base can then
receive the accumulated time update (for example, with 1 Hz base and 5 Hz
rover). An anchor interval exceeding 2 s discards the tight filter and requires
a fresh LC heading/bootstrap; failed exact-base anchors also discard it. The
public configuration exposes this bound as `max_tight_interval_s`.

This route is distinct from fixed-lag FGO and existing whole-file fusion.
Batch scores do not prove its accuracy or latency. Acceptance requires native
queue/reset tests, numerical prefix parity on usable raw PPC outputs,
late/missing-base and IMU-gap replays, and measured missing outputs and latency.
The processor and executable build; all nine queue/reset tests and four
historical RTCM context tests pass. The raw-data verification below passed
on Tokyo run1 with 600 epochs and an exactly matching 300-epoch prefix.

## Reproducible raw-data verification

Build the explicit staging harness and run the verifier against an existing
PPC development run. Keep the historical native test fixtures separate.
The verification targets require `BUILD_TESTING=ON` and Google Test; the
production `gnss_online` executable does not depend on Google Test.

```sh
cmake --build build --target gnss_online gnss_online_ppc_fixture
python scripts/analysis/verify_online_ppc.py \
  --run-dir /datasets/PPC-Dataset/tokyo/run1 \
  --fixture-exe build/tests/gnss_online_ppc_fixture \
  --online-exe build/apps/gnss_online --epochs 600 \
  --output-dir output/online-tokyo1
```

For Windows multi-config builds add `--config Release` and use
`build/tests/Release/*.exe` and `build/apps/Release/*.exe`. The output directory
must be new. The report records raw-input and executable hashes, the command,
every emitted row, processing time and availability counts.

The typed-API audit uses all constellations admitted by the RINEX reader. The
first half of five executions receives identical inputs; subsequent inputs
remove base epochs, deliver base epochs after their rover output, or omit IMU
samples for four seconds. A fifth execution delivers the rover 0.15 s late,
checks its input-age metadata and preserves its chronological numerical output.
Late base delivery is explicitly 0.1 s after its rover output. All
numerical/metadata prefix fields except wall
time must match exactly, with actual RTK/fused positions and supplied tight
time updates required. The IMU-gap execution must reinitialize fusion.

The stdin transport audit stages GPS MSM7/1019 frames because the existing
ephemeris encoder cannot emit Galileo/BeiDou navigation. This restriction is
on the transport fixture, not on the typed processor's constellation support.
It waits for each output with stdin still open before sending another rover
event, then repeats with a state reset in the suffix. The numerical prefix
must match and the suffix must actually change. This also tests CRC framing
and historical week interpretation through the production CLI.

PPC does not provide observed reception timestamps. The staging simulation
delivers samples/base records at each rover event and admits navigation only
after its clock epoch and transmission time. It reads files only to prepare
events; the processor sees received records alone. The results establish this
declared simulation's prefix causality, missing/late-data behavior and local
processing latency. They do not establish live network latency or full-run
accuracy against reference truth.

The 2026-10-06 Tokyo run1 verification emitted 600 valid typed RTK positions,
590 fresh fused positions and 103 tight time updates. Exact-base availability
was 120/600; removing or delivering the second half's base epochs late reduced
it to 60/600. The four-second IMU outage caused one filter reset and fresh
reinitialization, with 560 valid fused outputs. Normal processor wall-time P95
was 25.733 ms (maximum 51.5232 ms) on this concurrently busy Windows host.
The GPS-only stdin fixture emitted 108 valid RTK positions, 590 fused positions
(including explicit propagation), and 28 tight updates; its unavailable RTK
outputs remain in CSV. All 600 outputs arrived with stdin still open. Raw
inputs, binaries, CSVs and timing measurements are retained in
`output/online-tokyo1-final-20261006/report.json`.
The [verification record](ppc_online_verification.json) preserves the input and
binary hashes, scenario populations, prefix result and artifact manifest.
