# Real-time RTK with `gnss live`: a moving rover and an RTCM3 base

Use this page when you want a `.pos` stream from a **moving rover** corrected
by a **base station over RTCM3** while the data arrives, not from RINEX files
afterwards. `gnss live` (binary `gnss_live`) reads the rover as RTCM3 or u-blox
UBX and the base as RTCM3 over NTRIP, TCP or a serial device, solves RTK per
epoch and appends one row per solved epoch to a `.pos` file.

| You have | Route |
|---|---|
| Rover and base streams (NTRIP, `tcp://`, serial, or recorded RTCM3 files), a vehicle-style rover | this page |
| Rover and base RINEX files | [RTKLIB migration](rtklib_migration.md) (`gnss solve`) |
| Rover and base RINEX plus a vehicle IMU, urban outages | [Urban RTK and IMU continuity](urban_rtk_fgo.md) |

`gnss live` is a different algorithm from `gnss solve`: it streams epoch by
epoch and lacks `gnss solve`'s whole-solution post-filters. On the same
decoded epochs the two agree in status 98.5 % of the time with the bare
options and 76.4 % with the `moving` gate set
([Parity](../realtime_stress.md#parity)).

## First output

The repository tracks a 40 s recorded excerpt (GPS only, 5 Hz rover, 1 Hz
base, RTCM3 MSM7 + 1005 + 1019) in `tests/fixtures/realtime_stress`. It needs
no receiver, no network and no `--nav-rinex`, because the base stream carries
the 1019 ephemerides. Run it from the repository root:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target gnss_live --parallel 2
mkdir -p output/use_cases/live
```

```bash
./build/apps/gnss_live \
  --rover-rtcm tests/fixtures/realtime_stress/rover.rtcm \
  --base-rtcm tests/fixtures/realtime_stress/base.rtcm \
  --preset low-cost \
  --reconnect --stall-timeout-ms 5000 --base-outage-fallback spp \
  --pos-uncertainty --pos-fix-diagnostics \
  --out output/use_cases/live/first.pos --format pos \
  | tee output/use_cases/live/first.log
```

The `--out` directory must already exist; `gnss_live` does not create it. It
prints one `summary:` line (the fields are explained under
[Exit criteria](#exit-criteria)); on the fixture it reads:

```text
summary: rover_source=rtcm mode=kinematic termination=rover_source_exhausted rover_messages=201 rover_total_messages=201 rover_valid_messages=201 rover_decoder_errors=0 base_messages=97 base_total_messages=97 base_valid_messages=97 base_decoder_errors=0 aligned_epochs=201 exact_base_epochs=41 interpolated_base_epochs=160 held_base_epochs=0 skipped_rover_epochs=0 written_solutions=201 fixed_solutions=197 ... rover_reconnects=0 base_reconnects=0 rover_stalls=0 base_stalls=0 fallback_spp_epochs=0
```

(`...` stands for timing fields that change on every run.) The `.pos` file has
a `%` header and 201 rows, 4 FLOAT followed by 197 FIXED, no `SPP` rows. The
`Status` column is `1` for `SPP`, `3` for `FLOAT` and `4` for `FIXED`.

```bash
grep '^% GPS_Week' output/use_cases/live/first.pos | tr ' ' '\n' | tail -9
awk '!/^%/ {c[$9]++} END {for (s in c) print "status", s, c[s]}' output/use_cases/live/first.pos | sort
awk '!/^%/ && $9 == 4 {print $2, $(NF-1), $NF}' output/use_cases/live/first.pos | sort -k2 -gr | head -3
```

```text
sdn(m)
sde(m)
sdu(m)
sdne(m)
sdeu(m)
sdun(m)
DiffAge(s)
FloatFixH(m)
FloatFixU(m)
status 3 4
status 4 197
187470.800 0.18940 0.78508
187471.000 0.18179 0.80530
187471.200 0.17405 0.82775
```

How to read it:

- All 201 rover epochs got a row: `written_solutions` equals `rover_messages`,
  `skipped_rover_epochs=0` and `fallback_spp_epochs=0`. 41 epochs used an exact
  base epoch and 160 an interpolated one; that is what a 5 Hz rover against a 1
  Hz base looks like.
- The two trailing column groups come from `--pos-uncertainty`
  (`sdn sde sdu sdne sdeu sdun DiffAge`) and `--pos-fix-diagnostics`
  (`FloatFixH FloatFixU`); every other column is the plain `.pos` layout and is
  byte-identical without them.
- `FloatFixH` and `FloatFixU` are `nan` on the 4 FLOAT rows. The largest
  `FloatFixH` values on the fixture, 0.16 to 0.19 m, are the first 8 FIXED rows,
  where the integer solution moved the position away from a freshly started
  float filter; the other FIXED rows stay at or below 0.03 m (mostly
  millimetres). The `sort` above lists those rows first.
- Every row, FLOAT and FIXED, reports `sdn = sde = sdu = 0.10000`. That is the
  constant legacy sigma, not a measured uncertainty; see [Boundary](#boundary).

This run is a wiring check on 40 s of recorded data. It says nothing about your
receiver, antenna, network or sky.

## The command for a real rover

Pick the rover source, then add the base and the robustness options. The rover
as RTCM3 over TCP (for example a receiver or a relay on your network):

```bash
mkdir -p output/live
./build/apps/gnss_live \
  --rover-rtcm tcp://192.168.1.50:5018 \
  --base-rtcm ntrip://user:password@caster.example:2101/MOUNT \
  --nav-rinex data/brdc.nav \
  --preset low-cost \
  --reconnect --stall-timeout-ms 5000 --base-outage-fallback spp \
  --out output/live/run.pos --format pos \
  --pos-uncertainty --pos-fix-diagnostics
```

or a u-blox rover on a serial port, with the base over `tcp://` or a serial
radio link:

```bash
./build/apps/gnss_live \
  --rover-ubx /dev/ttyACM0 --rover-ubx-baud 115200 \
  --base-rtcm tcp://192.168.1.60:5019 \
  --nav-rinex data/brdc.nav --base-ecef "$BASE_X" "$BASE_Y" "$BASE_Z" \
  --preset low-cost \
  --reconnect --stall-timeout-ms 5000 --base-outage-fallback spp \
  --out output/live/run.pos --format pos --pos-fix-diagnostics
```

(`BASE_X/Y/Z` are your base antenna's surveyed ECEF coordinates in metres. Leave
`--base-ecef` out when the base's 1005 message already carries the surveyed mark.)

### Inputs

| Flag | What it does |
|---|---|
| `--rover-rtcm <path\|ntrip://\|tcp://>` | Rover as RTCM3 observations. |
| `--rover-ubx <file\|serial://\|/dev/tty...>` and `--rover-ubx-baud` | Rover as u-blox UBX instead (give one of the two rover flags). Baud default is 115200. |
| `--base-rtcm <path\|ntrip://\|tcp://\|serial://>` | Base RTCM3. NTRIP URLs take `ntrip://user:password@host[:port]/MOUNT`. Required. |
| `--nav-rinex <file>` | Supplemental broadcast navigation. It seeds the GPST week reference of both RTCM decoders (latest GPS toe) and supplies ephemerides. Multi-constellation needs an ephemeris for every system you want used, from this file or from 1019/1020/1042/1044/1045/1046 messages in the base stream; the multi-GNSS checks in [Real-time stress](../realtime_stress.md#multi-constellation-msm-epoch-assembly) all used `--nav-rinex`. It is optional on the fixture above because the base stream sends 1019. Without it and without any ephemeris in either stream, the head of the stream stays in week 0 ([Interfaces](../interfaces.md#multi-constellation-rtcm-epoch-assembly-no-rtcm-epoch-assembly)). |
| `--ubx-nav` | With `--rover-ubx`: decode the rover's `UBX-RXM-SFRBX` into broadcast ephemerides (GPS/QZSS L1 C/A, Galileo E1-B, BeiDou D1/D2, GLONASS) and the GPS Klobuchar set, and merge them like RTCM ephemerides. Default off. See [below](#a-u-blox-rover-without-a-navigation-file). |
| `--base-ecef <x> <y> <z>` | Overrides the base position from the stream. Use it when the position in the 1005 message is not your surveyed mark. |

### A u-blox rover without a navigation file

A u-blox receiver already logs the subframes it decodes, so with `--ubx-nav` a
rover needs neither `--nav-rinex` nor a base that sends ephemerides:

```bash
./build/apps/gnss_live \
  --rover-ubx /dev/ttyACM0 --rover-ubx-baud 115200 \
  --base-rtcm tcp://192.168.1.60:5019 --ubx-nav \
  --preset low-cost --out output/live/run.pos
```

The rover has to log `UBX-RXM-RAWX` (the observations) and `UBX-RXM-SFRBX` (the
subframes). The first RAWX epoch is also taken as the GPST reference of the base
decoder, so a base stream without ephemerides is dated correctly. Solutions start
once a satellite's ephemeris is complete: a median of 17 to 49 s per system after it
is first tracked, and minutes for a satellite the receiver does not decode
(measured on three logs of 2.67 to 4 h, [Interfaces](../interfaces.md#broadcast-ephemerides-from-a-u-blox-rover-ubx-nav-nav-rinex-out)).
A record is merged at once, from its first complete copy, when the satellite has no valid
record yet, so the start-up is not delayed by a repeat interval. If a later copy of the same issue
differs, the merged record is taken back out and that satellite is not used again until two equal
copies from different transmissions exist (accept, then confirm). A new issue of a satellite that
already holds a valid record of this kind (for example the next two-hourly GPS set) waits for a
second copy before it is merged, so a copy corrupted in its issue field (a wrong toe or GLONASS `tb`)
is not merged; and when any record is confirmed, the satellite's unconfirmed records of other
issues are taken back out, which removes a corrupted first copy merged at first acquisition. The cost
is that a corrupted record merged at first acquisition is selectable until it is retracted, and one
that is never followed by a confirmed issue of its satellite is never retracted.
Records whose health flags mark the satellite unusable (GPS/QZSS health 63,
Galileo E1-B health or data-validity flags, BeiDou health 1, GLONASS `Bn` or `ln`)
are skipped and counted; a receiver forwards such default sets, IGS BRDC files do
not carry them, and without the rule they were picked among the real records of the
same toe. The `summary:` line gains `ubx_nav_ephemerides=` (records added),
`ubx_nav_klobuchar=`, `ubx_nav_unhealthy_skipped=`, `ubx_nav_unconfirmed=` (records
merged and never confirmed by a second transmission, plus the candidates never confirmed:
copies still held or forgotten, and, for a satellite in fallback, replaced or pending ones),
`ubx_nav_conflicts=` (copies that differed from an
unconfirmed copy of the same issue), `ubx_nav_retracted=` (merged records taken back
out because of a conflict or by the clean-up on a confirmation) and
`ubx_nav_held_new_issue=` (copies of a new issue held until a second transmission because the
satellite already held a valid record). `gnss convert --format ubx --nav-rinex-out`
writes every record, unhealthy ones with their health word, as a RINEX navigation
file; it keeps confirm-twice (a record is written once a second transmission has
produced the same record), because a file has no start-up latency to protect.

Against `--nav-rinex` with the IGS BRDC files, on a log that was held out (3DOM
rover B, 4 h at 1 Hz, static, base about 3.6 km away, `--preset low-cost`, base
stream without ephemeris messages), `--ubx-nav` gave 14 379 against 14 400
solutions, the same status on 99.83 % of the epochs, 14 375 against 14 396 FIXED
epochs, a horizontal difference between the two FIXED positions of 0.0000 m at p50
and 0.0097 m at most, and no FIXED epoch more than 0.5 m from the median FIXED
position of the `--nav-rinex` run (0 against 0). The 21 missing solutions are the
first 21 s of the run, until enough ephemerides were complete. On that log and on
a 3 h Laura Bassi log, every healthy record that a second transmission confirmed
matched an IGS BRDC record of the same issue and toe. **`--ubx-nav` was run end to
end only on static rovers with a clear sky**: it is not validated on a moving rover
or in a canyon. That comparison started once, at the beginning of the log. Over
143 cold starts on three static logs, `--ubx-nav` without a navigation file gave
0.88 % of FIXED epochs more than 0.5 m off, against 0 with `--nav-rinex`, because
of the RTK hold-fix fallback (see [Boundary](#boundary)). Add `--nav-rinex` when you
have a navigation file, or `--hold-fix-unchanged-set` (16 against 2 241 such epochs on
the same starts). The measurements, their history and their limits are in
[Interfaces](../interfaces.md#broadcast-ephemerides-from-a-u-blox-rover-ubx-nav-nav-rinex-out).

Multi-constellation MSM needs no flag: the per-constellation messages of one
epoch are merged into one epoch using the RTCM multiple-message bit, with no
added latency. `--no-rtcm-epoch-assembly` restores one epoch per message and is
only an escape hatch ([Interfaces](../interfaces.md#multi-constellation-rtcm-epoch-assembly-no-rtcm-epoch-assembly)).
GPS-only streams are unchanged either way.

### Robustness

All three are off by default; without them a dropped link ends the run, a
silent peer blocks the read, and a base outage means no rows.

| Flag | What it does | Does not |
|---|---|---|
| `--reconnect` (`--reconnect-delay-ms`, default 2000; `--reconnect-attempts`, default 0 = never give up) | Reopens `tcp://` and NTRIP sources that drop or stall. Without it a lost link ended the run after 500 of 2086 solutions, exit 0, no error ([Fault matrix](../realtime_stress.md#fault-matrix-tcp-1x-420-s-window-first-fault-at-100-s)). With it the same fault resumed after 0.2 s with no loss of quality. | Reconnect serial or device sources; a stall ends those. |
| `--stall-timeout-ms 5000` | A `tcp://` or serial/device source that sends no byte for 5000 ms counts as stalled (reconnected with `--reconnect`, otherwise ends like EOF). Pick a value above the longest gap the peer legitimately leaves; a 1 Hz base needs well above 1000 ([Interfaces](../interfaces.md#stall-watchdog-for-gnss-live-stall-timeout-ms)). | Apply to NTRIP (it keeps its own 1 s `select()` wait and does not reconnect on a quiet caster) or to files. |
| `--base-outage-fallback spp` | A rover epoch with no usable base (after `max(1 s, base hold + base interval)` of wall time, 1.5 s by default for a 1 Hz base) is solved with the single-point solver and written with status `SPP`, never `FLOAT` or `FIXED`; the RTK state is untouched. Each live source gets its own reader thread, so the rover keeps flowing. | Give an RTK-quality position: SPP is code-only. It also does not help when the base never delivers its first epoch or its source has ended. |

Measured with a 30 s base outage (GPS only, PPC Tokyo run1, 5 Hz rover, 1 Hz
base): without the fallback 0 of 141 rover epochs got a row; with it 141 of 141
did, all `SPP`, with a 3D error against the reference of 8.5 m (p50) and 9.4 m
(p95), and the first `FLOAT` row after the outage was written at the same time as
without the fallback, 130.00 s ([Base-outage SPP fallback](../realtime_stress.md#base-outage-spp-fallback)).
The latency floor of the default path is one base interval: p50 0.41 s and max
0.82 s with a 1 Hz base, because a rover epoch waits for the next base epoch to
interpolate
([Limits](../realtime_stress.md#limits-found-and-not-fixed), item 2).

### Outputs

| Flag | What it does |
|---|---|
| `--out run.pos --format pos` | One row per solved epoch. `pos` is the default; `llh` and `xyz` exist but the two columns below need `pos`. |
| `--pos-uncertainty` | Appends `sdn sde sdu sdne sdeu sdun` (m) and `DiffAge(s)`. The sigma is a model output, not a certified bound ([Per-epoch uncertainty](../interfaces.md#per-epoch-uncertainty-in-pos-pos-uncertainty)). |
| `--pos-fix-diagnostics` | Appends `FloatFixH(m)` and `FloatFixU(m)`, the horizontal and vertical distance between the epoch's FLOAT position and its FIXED position; `nan` on every row that is not `FIXED`. A diagnostic, not an integrity bound ([Float-fix distance](../interfaces.md#float-fix-distance-in-pos-pos-fix-diagnostics)). |

### Profile: preset, motion gate, post-filter

`--preset low-cost` is the profile the wrong-FIX numbers below were measured
with. A moving rover also needs the motion-aware fixed-jump gate: the bare
static 5 m gate refuses fixes for a moving rover (first 300 s of the PPC Tokyo
run1 replay: 15 % fixed against 79 % with the gate;
[Method notes](../realtime_stress.md#method-notes)). The stress harness's
`moving` profile is `--max-pos-jump-min 5 --max-pos-jump-rate 30`; in this
build `--preset low-cost` already applies the same two values unless you set
them yourself, so you only add them to a **bare** command line. The gate has a
price: the `moving` set accepted more wrong fixes than the bare one in the
urban canyon (1055 wrong fixes of 3874 fixed epochs against none for the bare run in the
20x full-run replay, GPS only, where the bare run fixed 2.9 % of epochs against
49.5 %;
[Pacing runs](../realtime_stress.md#pacing-runs-1x-for-167-min-20x-for-the-full-run)).
Neither profile is a statement about RTK tuning in general.

`--kinematic-post-filter` is opt-in. Once a FIX has been written, it drops an
epoch whose height differs from the last written epoch by more than
`max(1 m, 4 m/s * dt)`, adds no delay, and adds `post_filter_dropped_epochs=` to
the summary. On the four UrbanNav Tokyo replays below it lowered Wrong/FIX only
slightly (6.6 to 5.0 %, 5.2 to 4.3 %, 1.1 to 0.4 %, 1.2 to 1.0 %) while
withholding 19 to 33 % of the solutions
([Wrong FIX on the real-time path](../realtime_stress.md#wrong-fix-on-the-real-time-path)).
It does not remove wrong fixes.

## Replay before you deploy

Before a field run, replay a recording through the same options with
`gnss realtime-stress`. It encodes a recorded RINEX run as RTCM3, serves it
paced over loopback TCP or a minimal local NTRIP caster, runs `gnss_live`
against those sockets, injects link faults and measures survival, latency,
recovery and accuracy against the run's `reference.csv`. It is a recorded-data
replay on loopback, not a field soak
([What this is and is not](../realtime_stress.md)).

```bash
cmake --build build --target gnss_live gnss_rtcm_encode gnss_solve

python3 apps/gnss.py realtime-stress prepare \
    --run-dir data/PPC-Dataset/tokyo/run1 --out-dir output/rt/prep \
    --duration 420 --systems G,R,E,J,C

python3 apps/gnss.py realtime-stress run \
    --prepared output/rt/prep --out-dir output/rt/outage \
    --speed 1 --transport ntrip --reconnect \
    --live-profile default --live-arg=--preset --live-arg=low-cost \
    --live-arg=--nav-rinex --live-arg=data/PPC-Dataset/tokyo/run1/base.nav \
    --live-arg=--stall-timeout-ms --live-arg=5000 \
    --live-arg=--base-outage-fallback --live-arg=spp \
    --fault base_outage@100+30
```

- `--live-profile default` is bare `gnss_live`; `--live-arg=...` adds your
  field options one by one, so the replay runs the options you plan to deploy.
  The harness default `moving` profile adds only the two jump-gate flags.
- `prepare` defaults to GPS only (`--systems G`); the results in
  [Real-time stress](../realtime_stress.md) are GPS-only except the
  [multi-constellation check](../realtime_stress.md#multi-constellation-msm-epoch-assembly)
  (G+R+E+J+C MSM7). Encode the constellations your field receiver sends.
  By default the encoded base stream carries only GPS and GLONASS ephemerides
  (1019/1020), so pass the run's navigation RINEX with `--nav-rinex` as above,
  or add `--eph-systems G,R,E,J,C` to `prepare` to put Galileo, QZSS and BeiDou
  ephemerides (1045/1046, 1044, 1042) in the base stream; without either,
  Galileo, QZSS and BeiDou satellites have no ephemeris and the replay solves
  with G+R only. The two are equivalent only when the navigation file has no
  `IONOSPHERIC CORR` header: `--nav-rinex` also supplies those ionosphere
  parameters (the PPC files have them, the UrbanNav BCEmerge file does not), a
  stream cannot, and the solutions then differ
  ([measured](../realtime_stress.md#ephemerides-in-the-base-stream-eph-systems)).
- Faults are `--fault kind@at+dur=value`, in stream seconds
  ([Fault syntax](../realtime_stress.md#fault-syntax)). Useful ones:
  `base_outage@100+30` (data dropped, link up), `base_stall@100+10`,
  `base_latency@100=2`, `corrupt_base@100=0.05` (5 % of frames get a flipped
  bit), `rover_disconnect@100+10`, `server_restart@100+10`.
- `base_blackhole@100` stops the server sending but keeps the connection open,
  which is a dead peer with no FIN. It is the fault `--stall-timeout-ms`
  exists for, so it needs wall time: use `--speed 1` and
  `--live-arg=--stall-timeout-ms --live-arg=5000 --reconnect
  --reconnect-delay-ms 2000`. It is open ended (`+dur` must be 0).
- Add `--batch-parity` to compare against `gnss_solve` on the same epochs, and
  `suite --preset faults|faults_ntrip` to run a named matrix; `report
  --suite-dir` summarises it.

The run prints one line (`survived=`, rows, latency, fix fraction, 3D p50) and
writes `result.json`, `epochs.csv`, the `.pos` and the `gnss_live` log. On the
tracked fixture the same command shape runs in seconds:

```bash
python3 apps/gnss.py realtime-stress run \
    --prepared tests/fixtures/realtime_stress --out-dir output/use_cases/live/rt_outage \
    --speed 20 --reconnect --live-profile default --live-arg=--preset --live-arg=low-cost \
    --live-arg=--base-outage-fallback --live-arg=spp --fault base_outage@10+5
```

```text
run: survived=True restarts=0 out=201 lat_p50/p95/max=0.03/0.20/0.29s fix=0.836 d3_p50=0.05m rss_peak=26.3MB
```

In that run `result.json` flags the fault as detected (`skipped_rover_epochs=29`)
and the `.pos` has 29 `SPP` rows. Latency and RSS are wall-clock figures from
one machine at 20x; treat them as a smoke check.

## Exit criteria

Call a configuration ready for a supervised field trial only when, on your own
replays and then on your own recorded drives:

1. **The process ends the way you expect.** Exit code 0 and
   `termination=` in the `summary:` line is the cause you intended
   (`rover_source_exhausted` at the end of a file; `rover_source_stalled` or
   `base_source_stalled` means the watchdog ended the run).
2. **The link counters match the link.** In the summary, `rover_decoder_errors`
   and `base_decoder_errors` are 0 on a clean link (CRC failures count there; 5 %
   corrupted frames cost about 6 % of the solutions in the replays);
   `rover_reconnects` / `base_reconnects` and `rover_stalls` / `base_stalls`
   appear when the flags are given and tell you how often the link dropped;
   `skipped_rover_epochs` counts every epoch that got no RTK base.
3. **Outages are visible, not silent.** `fallback_spp_epochs` equals the number
   of `Status = 1` rows, and none of those rows is treated as RTK downstream.
4. **Output matches input.** `written_solutions` against `rover_messages` (or
   rover epochs sent in a replay): GPS-only urban runs solve fewer epochs
   because fewer than 4 satellites are usable; multi-GNSS solved 0.99 to 1.00 of
   epochs on the six PPC replays
   ([Multi-constellation MSM](../realtime_stress.md#multi-constellation-msm-epoch-assembly)).
5. **The FIX share is plausible for the sky.** `fixed_solutions /
   written_solutions` from the summary, or the share of `Status = 4` rows. For
   calibration, multi-GNSS `--preset low-cost` fixed 37.8 to 78.0 % of epochs on
   the six PPC replays, in the same table. There is no pass threshold here.
6. **Wrong FIX, with an independent reference.** Count FIXED epochs whose 3D
   error against a reference trajectory exceeds 0.5 m, the definition used
   throughout [Real-time stress](../realtime_stress.md); `realtime-stress run`
   does this when the prepared run carries a `reference.csv`, and
   `gnss uncertainty-eval` scores a `.pos` against a PPC/UrbanNav
   `reference.csv`. Compare the Wrong/FIX share with the UrbanNav table under
   [Boundary](#boundary) before you trust a FIX flag in a similar environment.
7. **Large `FloatFixH` is inspected, not thresholded.** List the FIXED epochs
   with the largest values and look at what the sky and the track were doing
   there:

   ```bash
   awk '!/^%/ && $9 == 4 && $(NF-1) != "nan" {print $(NF-1), $NF, $2}' \
     output/live/run.pos | sort -gr | head -20
   ```

   `FloatFixH` ranked wrong FIX above correct FIX with AUROC 0.73 to 0.81 on
   four UrbanNav Tokyo runs and 0.77 to 0.95 on six PPC runs, and the
   distributions overlap (median 2 to 8 mm for a correct FIX, 1 to 59 cm for a
   wrong one). No threshold was tuned or is recommended
   ([Float-fix distance](../interfaces.md#float-fix-distance-in-pos-pos-fix-diagnostics)).
   A small value does not make a FIX correct.

   `gnss solve --pos-fix-diagnostics` and `gnss replay --pos-fix-diagnostics`
   write the same two columns, so you can compare the live run with a
   post-processed run on the logged data. There a FIXED row gets a value only
   if its written position is still the RTK epoch's own fixed position;
   otherwise it is `nan`. A FIXED row moved by post-processing is one example
   ([`gnss solve` rule](../interfaces.md#gnss-solve-only-for-the-rtk-epochs-own-fixed-position)).

## Boundary

**Do not trust a `gnss live` FIX in a dense urban canyon.** With
`--preset low-cost` and all constellations (G+R+E+J+C MSM7, `--nav-rinex`,
surveyed base, real host clock, offline replays of the 2018-12-19 UrbanNav Tokyo
runs, contract frozen before the run, measured on develop `52853076`, Odaiba
u-blox again after the Doppler slip fix of 2026-10-15), `gnss live` published
FIX epochs more than 0.5 m off at these rates
([source](../realtime_stress.md#wrong-fix-on-the-real-time-path)):

| UrbanNav Tokyo, `--preset low-cost` | FIX, GPS-only to multi | Wrong FIX, GPS-only to multi | Wrong/FIX, GPS-only to multi | FIXED 3D p95, multi |
|---|---|---|---|---|
| Shinjuku, Trimble rover | 3511 to 13636 | 1711 to 905 | 48.7 % to **6.6 %** | 0.78 m |
| Shinjuku, u-blox rover | 623 to 4988 | 202 to 258 | 32.4 % to **5.2 %** | 0.51 m |
| Odaiba, Trimble rover | 4419 to 7704 | 125 to 81 | 2.8 % to 1.1 % | 0.17 m |
| Odaiba, u-blox rover | 77 to 3456 | 4 to 43 | 5.2 % to 1.2 % | 0.17 m |

Multi-GNSS fixes 3.9 and 8.0 times as many epochs on the two Shinjuku runs, 1.7
times as many on Odaiba Trimble and 45 times as many on Odaiba u-blox, and
lowers Wrong/FIX on all four. The number of wrong fixes falls on the two
Trimble runs but rises on the two u-blox runs (202 to 258 and 4 to 43). Most
are decimetres to a few metres off rather than tens of metres, which still
matters for anything that trusts the FIX flag. On the PPC replays the
multi-GNSS low-cost Wrong/FIX was 1.2 to 3.7 % (measured before #615, not
re-measured). An earlier run of the same contract on
`b9a6e243` gave 12.1 %, 5.5 %, 1.0 % and 22.2 % in the table order; #615, the
deterministic same-toe ephemeris order, changed the multi-GNSS numbers
([details](../realtime_stress.md#wrong-fix-on-the-real-time-path)). Before the
Doppler slip fix of 2026-10-15, 23 % of the Odaiba u-blox fixes were wrong
([details](../realtime_stress.md#doppler-slip-test-and-time-shifted-base-epochs)).

What this page and its options do **not** do:

- A `FIXED` status is not an integrity guarantee. Nothing in the `.pos`, the
  summary or the options above is a protection level.
- **Cold start without a navigation file.** When satellites become usable one
  after another soon after the first fix, the RTK hold-fix fallback can keep a
  wrong FIX. This happens when ephemerides come from `--ubx-nav` or from the
  base stream and there is no `--nav-rinex`. Over 143 cold starts on three static
  open-sky logs, 0.88 % (`--ubx-nav`) and 0.52 % (ephemerides round-robin from the
  base) of FIXED epochs were more than 0.5 m off, some held for up to 30 min. With
  `--nav-rinex` it was 0. Such rows show a ratio of exactly the threshold (3.0
  with `--preset low-cost`). Pass `--nav-rinex` when you can. The opt-in
  `--hold-fix-unchanged-set` refuses that fallback when a satellite has joined,
  which brought those rates to 0.01 and 0.02 % and kept 99.7 to 99.9 % of the
  FIXED epochs there. It costs about 6 % of the FIXED epochs on the UrbanNav and
  PPC replays, so it is not the default. Without it, treat a FIXED row at exactly
  the threshold ratio as held, not validated
  ([details](../interfaces.md#rtk-hold-fix-fallback-known-weakness)). The other
  opt-in, `--hold-fix-min-matched-pairs 12`, was not measured on cold starts. On
  the UrbanNav and PPC replays it took multi-GNSS `gnss live` from 4.10 to 3.98 %
  of FIXED epochs more than 0.5 m off, keeping 98.0 % of the correct ones.
  Adding `--min-fix-satellites 4`, which refuses an ambiguity-resolution fix from
  fewer than 4 non-reference satellites, took it further to 1.42 %, keeping 96.1 %
  of the correct ones and at least 93.7 % in every run.
  This option was not measured on cold starts either
  ([details](../interfaces.md#rtk-fixes-from-few-satellites)).
- The reported sigma is not calibrated. In the fixture run every FLOAT and FIXED
  row says 0.10 m, the legacy constant (`0.01 m^2 * I`); on PPC FLOAT epochs
  that constant was far too small, with 1202 of 3834 epochs off by 10 m or more
  and a median reported sigma of 0.10 m there
  ([RTK FLOAT reported covariance](../rtk_float_covariance.md), measured on the
  `gnss_pva_replay` control, not on `gnss live`). `gnss uncertainty-eval` checks
  the columns against a reference; `K*sigma` is a diagnostic proxy there, not a
  protection level.
- `--pos-fix-diagnostics` and `--kinematic-post-filter` are diagnostics and a
  small guard, not a gate you can set and forget. The post-filter keeps most
  wrong FIX; `FloatFixH` overlaps between correct and wrong FIX and has no
  threshold.
- `--base-outage-fallback spp` keeps rows flowing during a base outage, in
  metres-level SPP, with no dead reckoning. It does not recover RTK sooner.
- A candidate that additionally used `FixRecoveryGuard` failed its frozen check
  and is not offered for `gnss live`; a proposed solve-on-arrival low-latency
  mode was No-Go and was not merged
  ([Limits](../realtime_stress.md#limits-found-and-not-fixed)).
- The robustness results are recorded-data replays on loopback, mostly GPS only,
  on a 4 vCPU VM. They are not a field soak, a real network, a real receiver or
  hours of real signal conditions. After a relaunch the first 30 s contained
  only wrong fixes under the `moving` gate set (32 epochs, median 3D error
  2.5 m).
- Multi-GNSS costs CPU: the offline realtime factor fell from about 220 to 335x
  to about 31 to 71x with four runs in parallel on 4 vCPU.

## Next step

Read the options in [Interfaces](../interfaces.md): the
[stall watchdog](../interfaces.md#stall-watchdog-for-gnss-live-stall-timeout-ms),
the [base-outage fallback](../interfaces.md#base-outage-spp-fallback-for-gnss-live-base-outage-fallback-spp),
[`--pos-uncertainty`](../interfaces.md#per-epoch-uncertainty-in-pos-pos-uncertainty)
and [`--pos-fix-diagnostics`](../interfaces.md#float-fix-distance-in-pos-pos-fix-diagnostics).
The full fault matrix, soak and known limits are in
[Real-time stress harness](../realtime_stress.md). If your drive is a dense city,
read [Urban canyon RTK options](urban_canyon_rtk_options.md) for the opt-in
robustness options (they are `gnss solve` options, not `gnss live` flags) and
[RTK wrong-FIX holdout](../rtk_wrong_fix_holdout.md) for how a candidate wrong-FIX
gate was frozen, checked and rejected. For an IMU to bridge outages, see
[Urban RTK and IMU continuity](urban_rtk_fgo.md). The sign-off process is in
[Validation](../validation.md) and the default-profile baselines are in
[Benchmarks](../benchmarks.md).
