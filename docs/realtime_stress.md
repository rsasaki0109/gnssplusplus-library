# Real-time stress harness (recorded-data replay)

`gnss realtime-stress` replays a **recorded** RINEX run as RTCM3 over local TCP or a minimal
local NTRIP caster, paced against the wall clock, runs `gnss live` against those sockets,
injects link faults and measures what `gnss live` does. There is no receiver in the loop.

!!! warning "What this is and is not"
    This is a **recorded-data replay on loopback**, not a field soak. It shows how `gnss live`
    behaves when a faithful byte stream is late, missing, corrupted, split or cut. It does not
    show behaviour on a real network (MTU splitting, bufferbloat, caster quirks), with a real
    receiver, or over days. The fault, soak and pacing results below are GPS-only;
    multi-constellation MSM was checked separately (see
    [Multi-constellation MSM](#multi-constellation-msm-epoch-assembly)). The "soak" below is a 60 minute wall-clock run at 60x that loops the 40 minute
    PPC run about 90 times (about 62 hours of stream time), not a 60 hour test.

## Commands

```bash
cmake --build build --target gnss_live gnss_rtcm_encode gnss_solve

# 1. Encode a recorded run: rover/base RINEX -> RTCM3 MSM7 + 1005 + 1019, plus an epoch index.
python3 apps/gnss.py realtime-stress prepare \
    --run-dir data/PPC-Dataset/tokyo/run1 --out-dir output/rt/prep --duration 420

# 2. One scenario: pacing 1x/5x/20x, TCP or NTRIP, any number of faults, parity vs gnss_solve.
python3 apps/gnss.py realtime-stress run --prepared output/rt/prep --out-dir output/rt/r1 \
    --speed 1 --transport ntrip --batch-parity \
    --fault base_outage@100+10 --fault corrupt_rover@200+0=0.05

# 3. A named matrix (smoke | faults | faults_ntrip), several scenarios in parallel.
python3 apps/gnss.py realtime-stress suite --prepared output/rt/prep --out-dir output/rt/faults \
    --speed 1 --preset faults --jobs 4
python3 apps/gnss.py realtime-stress report --suite-dir output/rt/faults

# 4. Soak: loop a window with time-shifted (orbit-consistent) data until a wall-clock cap
#    (prepare without --duration for the whole run).
python3 apps/gnss.py realtime-stress run --prepared output/rt/full --out-dir output/rt/soak \
    --speed 60 --loops 0 --max-wall 3660 --compact
```

`gnss rtcm-encode` is the underlying encoder (`--systems G,R,E,J,C`, `--format msm7|msm5|msm4|legacy`,
`--station`, `--rinex-out`, `--eph-systems`). `prepare --eph-systems G,R,E,J,C` puts Galileo, QZSS and BeiDou
ephemerides (1045/1046, 1044, 1042) in the base stream next to 1019/1020; it is passed to the base encode only
and the default (not passed, so the encoder's `G,R`) leaves the command line and the bytes unchanged. Only
systems that are also in `--systems` are broadcast. See [Ephemerides in the base
stream](#ephemerides-in-the-base-stream-eph-systems). `--loops` time-shifts GPS MSM and 1019 frames only, so a looped
soak of a stream with 1042/1044/1045/1046 keeps their original times. The ctest lane `python_realtime_stress_tests` replays a tracked 40 s
fixture (`tests/fixtures/realtime_stress`) at 20x against the built `gnss_live` in about 17 s
and checks survival plus detection of every fault kind.

### Fault syntax

`--fault kind@at+dur=value`, times in **stream** seconds from the start of the window (so a scenario has
the same shape at 1x and 20x):

| kind | effect |
|---|---|
| `base_outage@t+d` / `rover_outage@t+d` | link up, data dropped for `d` s |
| `base_stall@t+d` | data held for `d` s, then delivered in a burst |
| `base_latency@t=v` | constant extra delay `v` s on the base stream from `t` |
| `base_jitter@t=v` | random delay `U(0,v)` s per base epoch from `t`; epochs can be reordered |
| `corrupt_base@t=p`, `corrupt_rover@t=p` | flip one random bit in a fraction `p` of frames (CRC failures, bad lengths) |
| `rover_disconnect@t+d`, `base_disconnect@t+d` | server closes the connection and refuses reconnects for `d` s |
| `server_restart@t+d` | both servers go down for `d` s |
| `base_blackhole@t`, `rover_blackhole@t` | from `t` the server stops sending on the connection that exists at `t` but keeps it open (no FIN, no RST) until the client closes it or the run ends. A connection made after `t` is served normally from the live position in the schedule. Open ended: `dur` must be 0 (`base_blackhole@100+10` is rejected). Exercises `gnss live --stall-timeout-ms`. |

`--supervise` relaunches `gnss_live` when it exits early (what an init system would do);
`--reconnect` runs it with its own `--reconnect` (`--reconnect-delay-ms` sets its delay, default 200;
the attempt budget is sized by the harness). Any other `gnss_live` option goes through `--live-arg=...`, e.g.
`--live-arg=--stall-timeout-ms --live-arg=5000` for the stall watchdog.

Every fault report also carries `first_solution_after_s` (stream time between the fault start and the moment
`gnss_live` wrote the first row for data at or after it), `first_solution_data_s` (data time of that row minus the fault start)
and `solutions_from_30s_after` (rows with data time from `t+30` to the end of the stream). A blackhole report adds
`rover_stalls_reported` / `base_stalls_reported` (from the `summary:` line) and the server's `epochs_blackholed`.

## What is measured

| metric | definition |
|---|---|
| survival | one `gnss_live` process ran to the end of the stream and exited 0 (soak: still running when the feed was halted) |
| output vs input | solved epochs written / rover epochs sent |
| latency | solution row first seen in `--out` minus the time that rover epoch was scheduled on the socket (p50/p95/p99/max). The scheduled time is when the data was available to the client |
| recovery | stream seconds from the end of a fault to the first FLOAT / FIXED output, next to the same figure in the clean run |
| accuracy | ENU error against `reference.csv` for the 30 s before, during, 0-30 s and 30-90 s after each fault, next to the clean run in the same windows; wrong fixes are FIXED epochs with a 3D error above 0.5 m |
| detection | whether `gnss_live` itself reports the fault (decoder errors, held/skipped epochs, output gap, process exit, reconnect) |
| RSS / CPU | `/proc/<pid>` once per second, linear RSS slope after a warm-up, CPU seconds |
| parity | the same epochs from `gnss_solve` on the encoded RINEX (`vs_gnss_solve`) and from `gnss_live` over the same RTCM *files* with no sockets (`vs_gnss_live_files`) |

Output per scenario: `result.json`, `epochs.csv` (one row per solution with wall time, latency, ENU error),
`resources.csv`, `server_{rover,base}.csv` (what was sent and when, and which fault touched it), the
`.pos` and `gnss_live` log. A suite also writes `suite_summary.json`. Nothing needs matplotlib.

## Method notes

* Data: PPC Tokyo run1, rover 5 Hz and base 1 Hz, GPS only (L1/L2/L5) encoded as MSM7, 1005 every 10 s and 1019
  round-robin so each satellite repeats every 30 s. These runs predate multi-constellation epoch assembly, when
  `gnss_live` consumed one observation message per epoch, so only one constellation per stream was usable.
* Loops shift every timestamp by a multiple of 16 s and rotate the right ascension of each 1019 by
  `OMEGA_E * shift`, so the shifted broadcast orbit gives the same ECEF track: each loop is the same
  physics at a later time. The loop gap is 60 s of silence.
* Two `gnss_live` option sets are used. `moving` (the harness default) adds
  `--max-pos-jump-min 5 --max-pos-jump-rate 30`, the motion-aware fixed-jump gate a vehicle needs; `default` is
  bare `gnss_live`. The bare static 5 m gate refuses fixes for a moving rover (first 300 s: 15 % fixed against
  79 %), but `moving` accepts more wrong fixes in the urban canyon. The stress numbers are about the streaming
  behaviour; neither profile is a statement about RTK tuning.
* Hardware: 4 vCPU VM, Release build, Python 3.13 server threads, loopback only.

## Results

All numbers: PPC Tokyo run1 (2390 s, 5 Hz rover, 1 Hz base), GPS-only MSM7, loopback, 4 vCPU, Release build.
"Before" is `gnss_live` as of `origin/develop` (`ae177d37`); "after" includes the fixes
listed below. `moving` option set unless noted. Raw JSON/CSV are produced by the commands above.

### Pacing runs: 1x for 16.7 min, 20x for the full run

| run | wall | survives | solutions / rover epochs | fixed | accuracy vs reference | latency p50 / p95 / max | CPU, RSS |
|---|---|---|---|---|---|---|---|
| 1x, first 1000 s | 1000.8 s | yes, exit 0 | 3441 / 4996 | 70.4 % | 3D p50 6.5 cm, p95 22.8 m, 425 wrong fixes (> 0.5 m) | 0.41 / 0.80 / 0.83 s | 0.5 % of a core, RSS 27 MB, slope after warm-up 0.06 MB/h |
| 20x, full run | 120.3 s (19.87x) | yes, exit 0 | 7829 / 11919 | 49.5 % | 3D p50 3.29 m, p95 36.8 m, 1055 wrong fixes of 3874 | 0.025 / 0.044 / 0.066 s | 8.8 % of a core, RSS 27 MB |
| 20x, full run, bare `gnss_live` | 120.3 s | yes, exit 0 | 7611 / 11919 | 2.9 % | 3D p50 2.77 m, p95 36.7 m, 0 wrong fixes | 0.025 / 0.044 / 0.075 s | 8.4 % of a core, RSS 27 MB |

* Solutions are fewer than epochs because GPS alone often has fewer than 4 usable satellites in this urban run.
* The latency is the wait for the **next 1 Hz base epoch**, which `gnss live` needs for interpolation: at 1x it is
  uniformly 0 to 0.8 s (p50 0.41 s), at 20x it is the same 0.5 s of stream time shrunk to 25 ms. It is not
  processing time. `gnss live` over the files processes the whole run in 9.6 s (about 250x real time) on one core.

### Parity

| comparison | epochs | result |
|---|---|---|
| `gnss_live` over sockets (paced) vs `gnss_live` over the same RTCM files, 1x/1000 s, 20x/full (`moving`) and 20x/full (bare) | 3441 / 7829 / 7611 | **byte-identical** `.pos` rows in all three |
| vs batch `gnss_solve --mode kinematic` on the same encoded epochs, bare, full run | 6244 common (1367 only live, 93 only batch) | status agreement **98.5 %**; both fixed (223): position difference p50 1.2 cm, p95 4.0 cm, max 39 cm; all statuses p50 0.67 m, p95 14.2 m |
| same, `moving`, full run | 5813 common (2016 only live, 88 only batch) | status agreement **76.4 %**; both fixed (2755): p50 0.9 cm, p95 8.6 m (a wrong fix on one side), max 60 m |
| same, `moving`, first 1000 s | 2762 common (679 only live) | status agreement **91.3 %**; both fixed (2014): p50 0.8 cm, p95 9.5 m |

Streaming does not change the solution. The differences against `gnss_solve` come from its extra batch logic
(non-fix drift guard, height-step guard, FLOAT bridge-tail guard, SPP fallback and so on, printed in its summary),
which `gnss live` does not have, and they grow with the looser `moving` fixed-jump gate.

### Fault matrix, TCP, 1x, 420 s window, first fault at 100 s

"Before" and "after" are `gnss_live` before and after the fixes below; `*_reconnect` rows run
`gnss live --reconnect`. Times are stream seconds. "Recovery" is the time from the end of the fault to the first
FLOAT/FIXED output; "post-fault accuracy" is the 30 s after a windowed fault (everything after the start for
open-ended faults) with the clean run in brackets; "wrong" counts fixed epochs with a 3D error above 0.5 m.
Outage, stall, disconnect and restart last 10 s ("long": 30 s); latency 2 s; jitter up to 3 s; corruption flips one bit in 5 % of frames.

| scenario | before: survives | after: survives | out epochs (clean 2086) | latency p50/p95/max (s) | max gap (s) | recovery FLOAT/FIX (s) | post-fault accuracy | reported before/after |
|---|---|---|---|---|---|---|---|---|
| base_jitter | yes | yes | 1807 | 1.29/3.00/3.76 | 3.0 | 0.0/0.0 | 63% fix, 15 cm p50, 119 wrong (clean 75%, 5 cm, 117) | Y/Y |
| base_latency | yes | yes | 2086 | 2.21/2.80/2.84 | 0.2 | 0.0/0.0 | 75% fix, 5 cm p50, 117 wrong (clean 75%, 5 cm, 117) | Y/Y |
| base_outage | yes | yes | 2032 | 0.41/0.81/0.87 | 11.0 | 0.0/0.0 | 100% fix, 3 cm p50, 0 wrong (clean 100%, 3 cm, 0) | Y/Y |
| base_outage_long | yes | yes | 1932 | 0.41/0.81/0.84 | 31.0 | 0.0/0.0 | 100% fix, 3 cm p50, 5 wrong (clean 100%, 3 cm, 0) | Y/Y |
| base_stall | yes | yes | 2086 | 0.41/0.81/10.81 | 2.0 | 0.0/0.0 | 100% fix, 3 cm p50, 0 wrong (clean 100%, 3 cm, 0) | Y/Y |
| baseline | yes | yes | 2086 | 0.41/0.81/0.84 | - | - | - | -/- |
| combined | yes | yes | 1989 | 0.80/1.80/2.81 | 11.0 | 0.0/0.0 | 100% fix, 3 cm p50, 0 wrong (clean 100%, 3 cm, 0) | N/Y |
| corrupt_base | yes | yes | 1969 | 0.41/0.81/3.80 | 5.0 | 0.0/0.0 | 74% fix, 5 cm p50, 111 wrong (clean 75%, 5 cm, 117) | N/Y |
| corrupt_rover | yes | yes | 1965 | 0.41/0.81/0.83 | 1.0 | 0.0/0.0 | 74% fix, 5 cm p50, 90 wrong (clean 75%, 5 cm, 117) | N/Y |
| rover_disconnect | no | no | 500 | 0.41/0.80/0.81 | 0.0 | -/- | n/a (process ended) | Y/Y |
| rover_disconnect_reconnect | n/a | yes | 2035 | 0.41/0.80/0.84 | 10.4 | 0.2/0.2 | 100% fix, 3 cm p50, 0 wrong (clean 100%, 3 cm, 0) | -/Y |
| rover_disconnect_supervised | no (10 relaunches) | no (10 relaunches) | 1968 | 0.41/0.80/0.82 | 23.4 | 13.2/14.0 | 38% fix, 246 cm p50, 32 wrong (clean 100%, 3 cm, 0) | Y/Y |
| server_restart | no | no | 497 | 0.41/0.80/0.82 | 0.0 | -/- | n/a (process ended) | Y/Y |
| server_restart_reconnect | n/a | yes | 2025 | 0.41/0.80/0.81 | 12.4 | 1.4/1.4 | 100% fix, 3 cm p50, 0 wrong (clean 100%, 3 cm, 0) | -/Y |

What the table shows:

* **A base outage costs exactly the outage plus the interpolation look-ahead and nothing else.** With a 10 s
  outage 54 epochs are lost (the 50 inside it and the 4 before it that were waiting for the next base epoch),
  the filter returns FIXED at the first epoch after the base is back, and the next 30 s are as good as the clean
  run (100 % fixed, 3 cm). After a 30 s outage 5 of the first 150 epochs are wrong fixes (clean: 0).
* **Stalls and latency are absorbed by TCP.** A 10 s stall is delivered as a burst and every epoch is
  solved with the clean-run accuracy; the price is latency (max 10.8 s, i.e. the stall). A constant 2 s base
  delay shifts every latency by 1.8 s and changes nothing else.
* **Jitter that reorders epochs loses data:** base epochs that arrive after a later one are unusable, 279 of 2086
  solutions are lost, 63 % fixed afterwards instead of 75 %, 3D p50 15 cm instead of 5 cm.
* **Corrupt frames do not crash `gnss_live`;** 5 % bad frames cost about 6 % of the solutions. The loss is bigger than the
  corrupted fraction because a corrupted length field swallows the next frames before the parser re-synchronises.
  Before the fix the run reported `rover_decoder_errors=0` and `base_decoder_errors=0`; now 118 and 38.
* **Losing the link ends the run** unless `--reconnect` is used: 500 of 2086 solutions, exit 0, no error. With
  `--reconnect` it resumes after 0.2 s (rover only) or 1.4 s (both servers restarted) with no loss of quality.
  Relaunching from outside (`--supervise`, one attempt per second; 9 of the 10 launches could not connect while the
  server was down) recovers FLOAT 13 s and FIXED 14 s after the link returns, but the cold start is poor: in the first 30 s only 84 epochs are solved, 38 % of them are
  fixed and every one of those fixes is wrong (32 epochs, median 3D error 2.5 m; clean: 100 % fixed, 0 wrong), because the loose `moving` gate
  accepts early fixes.

### Fault matrix, NTRIP, 1x

The same faults served by the local caster (`ntrip://127.0.0.1:port/MOUNT`, `ICY 200 OK`).
Outputs of 2086 are the clean run.

| scenario | before fixes | after fixes | after fixes + `--reconnect` | recovery FLOAT/FIX with `--reconnect` (s) |
|---|---|---|---|---|
| baseline | runs to end, 2086 | runs to end, 2086 | runs to end, 2086 | - |
| base_outage | ends early, 497 | ends early, 497 | runs to end, 2032 | 0.0/0.0 |
| base_outage_long | ends early, 497 | ends early, 497 | runs to end, 1932 | 0.0/0.0 |
| base_stall | ends early, 497 | ends early, 497 | runs to end, 2086 | 0.0/0.0 |
| base_latency | ends early, 497 | ends early, 497 | runs to end, 2086 | 0.0/0.0 |
| base_jitter | ends early, 497 | ends early, 497 | runs to end, 1807 | 0.0/0.0 |
| corrupt_base | ends early, 522 | ends early, 522 | runs to end, 2037 | 0.0/0.0 |
| corrupt_rover | ends early, 600 | runs to end, 1985 | runs to end, 1985 | 0.0/0.0 |
| rover_disconnect | ends early, 500 | ends early, 500 | runs to end, 2035 | 0.2/0.2 |
| server_restart | ends early, 497 | ends early, 497 | runs to end, 2025 | 1.4/1.4 |
| combined | ends early, 497 | ends early, 497 | runs to end, 2018 | 0.0/0.0 |

* The NTRIP client reads with a **1 s select timeout and `gnss live` took one empty read for end of stream**, so
  any gap longer than a second (a base epoch lost to a CRC error, a 2 s latency step, any outage) ended the
  run, 497 outputs in, with exit 0. A base epoch is 1 s apart, so even one dropped base frame is enough.
* A **partial frame in one TCP read** was treated the same way (see the fixes), which is why 5 % corrupt rover
  frames stopped the NTRIP run at 600 outputs before and let it finish (1985) after.
* With `--reconnect` every NTRIP scenario above runs to the end.

### Soak: 60 min wall at 60x, the 2390 s run looped with a 60 s gap (about 62 h of stream)

Same stream, same `moving` option set, `--compact`, RSS and CPU sampled each second from `/proc`.

| build | RSS 10 / 30 / 60 min | RSS slope | CPU | achieved speed | end |
|---|---|---|---|---|---|
| `origin/develop` (before) | 69 / 145 / 209 MB | **+175 MB/h** | 43 % at 10 min, 98 % from 30 min (mean 86 %, 3226 CPU s) | 60x, then 51x at 30 min, 39x at 40 min, **30x at 60 min** | feed halted at 60 min with a backlog (latency up to 14 s); 70 loops, 830k of 1.05M epochs delivered; did not exit within the drain timeout, killed |
| + ephemeris de-duplication | 132 / 133 / 133 MB | +0.4 MB/h after warm-up | 28 to 31 % (1063 CPU s) | 60x all hour | exit 0, summary line; 90 loops (221 686 s of stream), 1 062 244 rover epochs, 676 472 solutions |
| + bounded satellite-state memo (final) | 27 / 28 / 29 MB | +2.6 MB/h | 25 to 27 % (946 CPU s) | 60x all hour | exit 0, summary line; 90 loops, 1 062 237 rover epochs, 676 466 solutions |

The 132 MB plateau of the middle row is the satellite-state memo, which only empties when a new ephemeris is
added (once per loop here); the last row caps it. The remaining 2.6 MB/h is the 90 loops' worth of genuinely new
ephemerides (a real stream adds one set about every two hours).

`gnss live` was ingesting an ephemeris rebroadcast every 30 s per satellite as the stream really carries them
(60x means one every half second of wall time). Before the fixes every repeat was appended, sorted and invalidated
the satellite-state cache: memory grew by 175 MB/h, and the per-epoch ephemeris scan grew with it until the
process was CPU-bound and fell from 60x to 30x. The loops are physically consistent (in a 3-loop test of a 300 s window every loop reproduced the same 1179 fixes),
so the growth is receiver-side bookkeeping and not the data.

### Other measurements

* `--no-base-interp --base-hold-seconds 1.0` (the low-latency-looking option) has the **same latency** (p50 0.41,
  p95 0.81, max 0.81 s) because the loop still reads ahead to the next base epoch before it chooses between exact,
  interpolated and held; it also loses RTK completely with a 1 Hz base (0 % fixed, 3D p50 8.8 m).
* Heavy corruption (30 % and 90 % of frames on both streams) neither crashes nor hangs `gnss_live`: it ends with
  `base_source_exhausted` (exit 0) after 39 solutions, or with exit 1 and no solution.
* A 5-minute window encoded as G+R+E+J+C MSM7 gives **1 solution** from 6004 rover messages (see Limits).

## Bugs found and fixed

Each fix is its own commit with a regression test that uses the tracked recorded fixture
(`tests/fixtures/realtime_stress`) and needs no hardware or external data. For every fix the default
`gnss live` output on the full PPC run is byte-identical before and after (`.pos` and summary line apart from
timings, `gnss_solve` output unchanged); the behaviour changes only in the fault case.

| commit | problem found by the harness | evidence |
|---|---|---|
| `fix(live): report RTCM decoder errors ...` | `rover_decoder_errors` / `base_decoder_errors` (`gnss live`) and `crc_errors` / `reconnect_count` (`gnss stream`) were always 0: both tools read the reader's counters after `close()` had reset them | TCP corrupt-base scenario: 0 reported before, 38 after (26 frames corrupted); the fault was undetectable from the summary |
| `fix(live): do not re-add rebroadcast RTCM ephemerides ...` | every rebroadcast 1019/1020 was appended to the navigation database, re-sorted, and cleared the satellite-state cache; memory and lookup cost grow without bound | soak: +175 MB/h and 43 % to 98 % CPU before; RSS flat afterwards (below) |
| `feat(live): opt-in --reconnect ...` | a dropped `tcp://` link, or one NTRIP read that returns nothing for 1 s, ended the run; NTRIP auto-reconnect existed but `gnss live` never enabled it and `tcp://` had none | rover disconnect: 500 of 2086 solutions before, 2035 after with `--reconnect`; restart of both servers 497 to 2025; every NTRIP outage scenario runs to the end |
| `fix(live): bound the satellite-state memo ...` | `NavigationData` memoises satellite states per (satellite, time) and only clears the memo when an ephemeris is added; with a navigation RINEX and no RTCM ephemerides it grows with every epoch | opt-in `setSatelliteStateCacheLimit` (default unbounded, so batch multi-pass solvers keep their cache), 16384 entries in `gnss live`; flat RSS in the final soak |
| `fix(ntrip): a partial RTCM frame in one read ...` | `NTRIPClient::readMessages` returned "no message" when a `recv()` ended inside a frame, and `gnss live` took that for the end of the stream; any TCP segmentation of a frame, or a corrupted length byte, stopped the run | recorded stream with every 10th epoch split over two segments: 10 solutions before, 201 of 201 after; NTRIP corrupt-rover 600 to 1985 outputs |

`--reconnect[-delay-ms N][-attempts N]` is opt-in on purpose: without it a finite NTRIP stream still ends
the run, and the summary line only gains `rover_reconnects=` / `base_reconnects=` when it is given.

## Wrong FIX on the real-time path

**Do not trust a `gnss live` FIX in a dense urban canyon.** Even with all
constellations, `gnss live --preset low-cost` published FIX epochs that were more
than 0.5 m off on two of the four UrbanNav Tokyo runs at rates above 5 %. These
are offline replays of the 2018-12-19 runs: G+R+E+J+C MSM7, `--nav-rinex`, the
surveyed base and the real host clock, measured against a contract frozen
before the run on 2026-10-12. The numbers below were measured on develop
`52853076`; the contract was first run on `b9a6e243`, and the multi-GNSS
numbers changed between the two (see the last bullet). The Odaiba u-blox row
was measured again on 2026-10-15, after the
[Doppler slip fix](#doppler-slip-test-and-time-shifted-base-epochs); the other
rows are unchanged by it.

| UrbanNav Tokyo, `--preset low-cost` | FIX, GPS-only → multi | wrong FIX, GPS-only → multi | Wrong/FIX, GPS-only → multi | FIXED 3D p95, multi |
|---|---|---|---|---|
| Shinjuku, Trimble rover | 3511 → 13636 | 1711 → 905 | 48.7 % → **6.6 %** | 0.78 m |
| Shinjuku, u-blox rover | 623 → 4988 | 202 → 258 | 32.4 % → **5.2 %** | 0.51 m |
| Odaiba, Trimble rover | 4419 → 7704 | 125 → 81 | 2.8 % → 1.1 % | 0.17 m |
| Odaiba, u-blox rover | 77 → 3456 | 4 → 43 | 5.2 % → 1.2 % | 0.17 m |

* Multi-GNSS fixes 3.9 and 8.0 times as many epochs on the two Shinjuku runs, 1.7 times as many on Odaiba Trimble
  and 45 times as many on Odaiba u-blox, and cuts Wrong/FIX on all four. The number of wrong fixes falls on
  Shinjuku Trimble (1711 → 905) and Odaiba Trimble (125 → 81) and rises on Shinjuku u-blox (202 → 258) and Odaiba
  u-blox (4 → 43). On Odaiba u-blox, GPS alone fixes 1.2 % of epochs and multi-GNSS 56 %.
* Most of these wrong fixes are decimetres to a few metres off, not tens of metres: see the FIXED 3D p95
  column. That still matters for anything that trusts the FIX flag.
* `--kinematic-post-filter` (opt-in) lowers multi-GNSS Wrong/FIX only slightly (6.6 → 5.0 %, 5.2 → 4.3 %,
  1.1 → 0.4 %, 1.2 → 1.0 %). It cuts the all-solution 3D p95 several-fold (Shinjuku Trimble 65 → 5 m) by
  withholding 19–33 % of solutions. It does not remove wrong fixes.
* On the PPC runs the multi-GNSS low-cost Wrong/FIX was 1.2–3.7 %
  (see [Multi-constellation MSM](#multi-constellation-msm-epoch-assembly)), measured before #615 and not
  re-measured since.
* `gnss live` lacks `gnss solve`'s whole-solution post-filters.
* The GPS-only numbers (48.7 % and 32.4 %) come from the GPS-only replay described below.
* Change since the first run of this contract (`b9a6e243`, which gave Wrong/FIX 12.1 %, 5.5 %, 1.0 % and
  22.2 % in the table order above, and 11.2 %, 4.8 %, 0.4 % and 20.5 % with `--kinematic-post-filter`): the
  encoded streams and all GPS-only arms are identical in the two runs, and the multi-GNSS low-cost arms
  differ. Rebuilding develop at `f32ea3b5` (before #615) reproduced all four earlier multi-GNSS low-cost
  results, and `9d58ef51` (#615, deterministic same-toe ephemeris order, Galileo I/NAV first) gave the new
  ones. The decision rule is unchanged: three of the four runs were still above 5 %, and two are after the Doppler
  slip fix.

A frozen-contract check (2026-10-11) of
`--preset low-cost --kinematic-post-filter` plus `FixRecoveryGuard` was
**No-Go**. The combination was chosen on PPC Tokyo run1 only and checked once on
nine other GPS-only replays: PPC Tokyo run2–3 and Nagoya run1–3, and UrbanNav
Shinjuku and Odaiba with the Trimble and u-blox rovers (surveyed base).

| Pooled over the 9 check runs | emitted | FIX | correct | wrong | Wrong/FIX |
|---|---|---|---|---|---|
| `--preset low-cost` | 59472 | 26836 | 23470 | 3366 | 12.5 % |
| + `--kinematic-post-filter` + FixRecoveryGuard | 47064 | 13578 | 12787 | 791 | 5.8 % |

| Gate | Result |
|---|---|
| Wrong FIX −50 % | pass (−76 %) |
| Pooled Wrong/FIX ≤ 10 % | pass |
| Correct FIX kept ≥ 60 % | **fail** (54.5 %) |
| No run with a higher Wrong/FIX | **fail**: Tokyo run2 7.0 → 14.4 %, Shinjuku Trimble 48.7 → 98.8 % |

Per-flag ablation on the same nine runs, run after the gates (not a gate):

| Arm | emitted | FIX | correct | wrong | Wrong/FIX |
|---|---|---|---|---|---|
| `--kinematic-post-filter` only | 47077 | 25654 | 23178 | 2476 | 9.7 % |
| FixRecoveryGuard only | 59467 | 14124 | 12909 | 1215 | 8.6 % |

- **FixRecoveryGuard.** It removes almost half of all FIX epochs. On Shinjuku
  Trimble it keeps the wrong ones (346 FIX, 343 wrong), so it is not offered for
  `gnss live`.
- **`--kinematic-post-filter`.** It is offered as an opt-in flag (see
  [Interfaces](interfaces.md)). It keeps 98.8 % of correct FIX, makes no run's
  Wrong/FIX higher, and removes 26 % of wrong FIX.
- **Status of these numbers.** They come from the ablation run after the No-Go
  decision on the same check runs, so they describe the flag rather than prove
  it. They are not evidence for a default. All of these GPS-only numbers were
  measured before the
  [Doppler slip fix](#doppler-slip-test-and-time-shifted-base-epochs), which
  changes the Odaiba u-blox run: GPS-only `--preset low-cost` now fixes 77 epochs
  there, 4 of them wrong, against none before.

## Doppler slip test and time-shifted base epochs

Fixed on 2026-10-15 in the RTK filter (`src/algorithms/rtk_bias.cpp`) and checked under a contract frozen before the
run. It changed `gnss live` and `gnss replay` on UrbanNav Odaiba with the u-blox rover. The numbers for that run in
this document, in [`gnss live`](use_cases/realtime_rtk_gnss_live.md) and in [Interfaces](interfaces.md) were measured
after the fix unless a table says otherwise.

* **Cause.** `gnss live` and `gnss replay` use a base epoch within 1 ms of the rover epoch as it is and interpolate
  the others to the rover time. The Odaiba u-blox rover tags 6069 of its 6206 epochs 1 ms after the second. With a
  5 Hz rover and a 1 Hz base the two kinds alternate, so the base epoch advances 0.199 s or 0.201 s while the rover
  advances 0.2 s. The Doppler slip test predicted the single-difference phase change with the rover's span for both
  receivers. At a Doppler of 1 kHz that is about one cycle off, against a threshold of 0.2 cycles. With
  `--preset low-cost --hold-fix-min-matched-pairs 12`, the test flagged false slips on about five satellites at a
  time at 93.5 % of the epochs at .0 s and .2 s, and an L1 ambiguity was reset on 41 % of all epochs.
* **Fix.** The test now predicts the change with each receiver's own span. When the two spans agree within 1 µs, or
  the base span is not known, it evaluates the old expression, so time-matched data gives byte-identical output.
* **Not affected:** `gnss solve`, which treats a base epoch as time-matched only within 1 µs and interpolates the
  others; base streams without Doppler (MSM4), where the test does not run; and rovers whose epochs fall on the base
  epochs.
* **Byte-identical `.pos` before and after:**
  * `gnss solve` on PPC Tokyo run1 and Nagoya run3, with and without `--preset low-cost`, and with
    `--preset low-cost` on the seven urban development runs of
    [Interfaces](interfaces.md#rtk-hold-fix-fallback-known-weakness) and the four UrbanNav Tokyo runs.
  * `gnss live` on the other five runs (PPC Tokyo run1 and Nagoya run3, UrbanNav Shinjuku Trimble and u-blox, Odaiba
    Trimble) with `--preset low-cost`, alone and with `--hold-fix-min-matched-pairs 12`, with that and
    `--min-fix-satellites 4`, and with `--hold-fix-unchanged-set`.
  * `gnss live` on the other three UrbanNav runs, GPS-only and multi-GNSS, without options, with `--preset low-cost`
    and with `--kinematic-post-filter` added.
  * `gnss replay` on three UrbanNav runs.
* **Odaiba u-blox, multi-GNSS `gnss live --preset low-cost`.** Each cell is FIXED more than 0.5 m off / FIXED.
  `gnss solve` on the RINEX of the same run is shown for comparison.

  | Options added | Before the fix | After the fix | `gnss solve` |
  |---|---|---|---|
  | none | 592 / 2588 (22.9 %) | 43 / 3456 (1.24 %) | 36 / 3693 (0.97 %) |
  | `--hold-fix-min-matched-pairs 12` | 495 / 2476 (20.0 %) | 43 / 3456 (1.24 %) | 33 / 3690 (0.89 %) |
  | `--hold-fix-min-matched-pairs 12 --min-fix-satellites 4` | 2 / 1201 (0.17 %) | 8 / 3253 (0.25 %) | 26 / 3606 (0.72 %) |

  * Correct FIXED epochs went from 1996, 1981 and 1199 to 3413, 3413 and 3245.
  * With `--hold-fix-min-matched-pairs 12`, the epochs with a Doppler slip went from 37.6 % to 0.29 % (`gnss solve`:
    0.7 %).
* **`gnss replay`** (rover RINEX, base RTCM, `--preset low-cost`) on the same run: 6 / 290 to 6 / 291.
* **Six-run totals** (PPC 2 + UrbanNav Tokyo 4, multi-GNSS, `--preset low-cost`): 2179 / 38903 (5.60 %) to
  1630 / 39771 (4.10 %); correct FIXED epochs 36724 to 38141.
* **Without the preset** (outside the contract):
  * Multi-GNSS went from 9 / 219 to 0 / 461.
  * With GPS only, one wrong fix lasted longer: 150 / 150 to 460 / 460, all about 1.1 m off with 4 to 6 satellites.
    The first 150, from the start of the run, are the same in both. Before the fix false slips ended that episode.
    After it the solution stayed about 1.1 m off for 310 more FIXED epochs within the next 73 s.
  * GPS alone with `--preset low-cost` fixed 77 epochs, 4 of them wrong (none before).

## Multi-constellation MSM (epoch assembly)

A receiver sends one MSM message per constellation with the same time tag. `gnss live` used to treat every
observation message as an epoch (limit 1 below), so only GPS-only streams worked. Two changes fix this, both on
by default:

* Messages are merged into one epoch until a message has the RTCM "multiple message" bit cleared, which adds no
  latency. A time change also closes the epoch, so a lost last message never merges two epochs. The escape
  hatch is `--no-rtcm-epoch-assembly` ([Interfaces](interfaces.md#multi-constellation-rtcm-epoch-assembly-no-rtcm-epoch-assembly)).
  `gnss rtcm-encode` now sets the bit on every observation message of an epoch except the last.
* The decoder dates epochs from a GPST reference taken from the stream instead of the host clock (limit 4
  below). Without that, a 2024 recording replayed on a 2026 host gave 1 solution, because GLONASS epochs took
  the host week.

Frozen-contract check (2026-10-12). It covered PPC Tokyo run2–3 and Nagoya run1–2, encoded G+R+E+J+C MSM7
(rover 5 Hz, base 1 Hz), with `--nav-rinex base.nav` and the real host clock. A first version without the
reference time was No-Go: the head epoch of every stream split in two, and the runs only worked with the
host clock set to the data date.

| gate | result |
|---|---|
| GPS-only path unchanged | 30/30 byte-identical comparisons against the previous develop (encoder, `gnss live` bare and low-cost, `gnss convert`, `gnss solve`) |
| assembled epochs | 8 streams: assembled = sent epochs, 0 duplicate times, 100 % equal to the per-message decoded union, 0 wrong week/TOW (one epoch per message before: 4x the epochs, ≤ 0.07 % equal) |
| `gnss live` vs `gnss solve` on the same decoded epochs (bare) | 100 % of batch epochs common; status agreement 99.32–99.96 %; both-FIXED horizontal difference p50 0.22–0.42 cm |
| paced over TCP (Tokyo run3, 300 s, 20x) | byte-identical to the file run, exit 0 |
| host-clock independence | byte-identical `.pos` with the real clock and with the clock at the data date: multi-GNSS + `--nav-rinex` (Tokyo run3), and G+R with 1019/1020 in the base stream and no navigation RINEX (Nagoya run2, 98.1 % of rover epochs solved) |

Multi-GNSS against GPS-only, `gnss live --preset low-cost` (all six PPC runs, report only, wrong FIX = FIXED with 3D
error > 0.5 m):

| run | solutions / rover epochs, GPS → multi | FIX % of epochs, GPS → multi | wrong FIX, GPS → multi | Wrong/FIX, GPS → multi |
|---|---|---|---|---|
| Tokyo run1 | 0.66 → 0.99 | 34.3 → 66.1 | 451 → 247 | 11.0 % → 3.1 % |
| Tokyo run2 | 0.74 → 1.00 | 44.6 → 78.0 | 201 → 141 | 4.9 % → 2.0 % |
| Tokyo run3 | 0.79 → 1.00 | 49.6 → 64.7 | 530 → 274 | 7.0 % → 2.8 % |
| Nagoya run1 | 0.79 → 0.99 | 35.9 → 69.1 | 122 → 137 | 4.5 % → 2.6 % |
| Nagoya run2 | 0.60 → 1.00 | 30.6 → 45.7 | 126 → 52 | 4.4 % → 1.2 % |
| Nagoya run3 | 0.56 → 1.00 | 17.6 → 37.8 | 152 → 72 | 16.6 % → 3.7 % |

* Wrong/FIX was lower with multi-GNSS in 11 of the 12 run/profile pairs, bare and low-cost, and equal in 1.
  Wrong fixes do not go away: there are still tens to hundreds per run with the low-cost preset, and on Nagoya run1
  the count rose (122 → 137) because many more epochs were fixed.
* Multi-GNSS costs CPU. The offline realtime factor fell from about 220–335x to about 31–71x, measured with four
  runs in parallel on a 4 vCPU VM. At 20x pacing the first 300 s of Tokyo run3 had a latency p50 of 0.35–0.84 s
  (0.05 s elsewhere). At 1x there is ample margin.
* Tokyo run1 was used while developing the change. Tokyo run2 and Nagoya run1 were check runs of the No-Go
  first version.

## Ephemerides in the base stream (`--eph-systems`)

Until now the encoded base stream carried 1019 and 1020 only, so a G+R+E+J+C replay without `--nav-rinex` solved with
G+R only (Tokyo run3, 120 s, 20x: FIX 0.519 against 0.952 with `--nav-rinex`). `gnss rtcm-encode --eph-systems
G,R,E,J,C` adds 1045/1046 (Galileo F/NAV, I/NAV), 1044 (QZSS) and 1042 (BeiDou), and the library now decodes
1042/1044 and routes 1042/1044/1045/1046 like 1019/1020 ([Interfaces](interfaces.md#rtcm-ephemeris-messages-gnss-rtcm-encode-eph-systems)).

Frozen-contract check (2026-10-12), against develop `9d58ef51` (which includes the deterministic same-toe ephemeris
order of #615): **Go**, all of G0 to G4 pass. It ran end to end on the four UrbanNav Tokyo runs (2018-12-19: Shinjuku
and Odaiba, Trimble and u-blox rovers; base `base_trimble.obs`, the DLR BCEmerge `base.nav`, Applanix `reference.csv`,
surveyed `--base-ecef -3961904.9530 3348993.7578 3698211.7553`), data that was not used while building the feature.
G+R+E+J+C MSM7, rover at its native rate (10 Hz Trimble, 5 Hz u-blox), base 1 Hz, `gnss live --preset low-cost`, offline on
the files. Arm S: base stream with `--eph-systems G,R,E,J,C`, no `--nav-rinex`. Arm N: the same streams with
`--nav-rinex base.nav`. Arm O: today's base stream (`G,R`), no `--nav-rinex`. The BCEmerge file has no
`IONOSPHERIC CORR` header lines, so neither S nor N applies a broadcast ionosphere model and the comparison isolates
the ephemeris path.

| gate | result |
|---|---|
| default output unchanged | 23 of 23 files byte-identical to develop `9d58ef51`: the regenerated tracked fixture (harness `prepare`, equal to the tracked bytes), `gnss rtcm-encode` of Tokyo run1 base with `--systems G` and `G,R,E,J,C` (bytes, index, summary), the G+R+E+J+C rover stream, `gnss live` on the fixture (bare, `--preset low-cost`) and on Tokyo run1 multi-GNSS with `--nav-rinex`, `gnss convert` of the fixture (the wall-clock `PGM / RUN BY / DATE` header line excluded) |
| encode, decode, compare (UrbanNav `base.nav`) | 5436 Galileo (4645), BeiDou (696) and QZSS (95) records: 100 % encoded, 100 % within 0.053 mm and 5e-16 s of the source record at nine times, toe-2 h to toe+2 h |
| RTKLIB 2.4.3 b34 parity, both directions | 100 % of the same records (our encoder into RTKLIB's decoder: 0.63 mm / 5e-15 s at worst; RTKLIB's encoder into ours: 0.053 mm / 5e-16 s); 693 of 696 BeiDou 1042 frames bit-identical to RTKLIB's |
| end to end, S against N | pass in all 4 runs: status agreement 99.4 to 100 % (criterion 97 %), S has 1.000 of N's solutions, both-FIXED horizontal difference p50 0 m (criterion 0.01 m) |
| safety, pooled | S Wrong/FIX 6.42 % (1867 of 29,059 FIXED), N 6.35 % (1836 of 28,916): +0.08 pp (limit +1.0 pp) |
| paced over TCP (Shinjuku Trimble, first 300 s, 20x), arm S | exit 0, 2987 of 2987 rows, `.pos` byte-identical to the offline run on the same files (latency p50 0.04 s, p95 0.05 s) |

Per run, `gnss live --preset low-cost` (report only for the O, N and S columns; wrong FIX = FIXED with 3D error above
0.5 m against `reference.csv`, nearest epoch within 0.25 s; `ns` is the mean `NumSat` column of the `.pos`):

| run | rover epochs | S/N solutions | S vs N status agreement | mean `ns` O, N, S | FIX % O, N, S | wrong FIX O, N, S | Wrong/FIX O, N, S |
|---|---|---|---|---|---|---|---|
| Shinjuku Trimble | 20,783 | 1.000 | 99.4 % | 5.7, 10.7, 10.7 | 15.7, 65.6, 66.2 | 1649, 905, 898 | 50.5, 6.6, 6.5 % |
| Shinjuku u-blox | 10,476 | 1.000 | 99.8 % | 5.4, 10.8, 10.8 | 5.9, 47.6, 47.6 | 202, 258, 264 | 32.4, 5.2, 5.3 % |
| Odaiba Trimble | 12,399 | 1.000 | 100.0 % | 5.4, 11.3, 11.3 | 35.6, 62.1, 62.1 | 125, 81, 81 | 2.8, 1.1, 1.1 % |
| Odaiba u-blox | 6206 | 1.000 | 99.6 % | 5.6, 11.8, 11.8 | 0.0, 41.7, 42.1 | 0, 592, 624 | 0.0, 22.9, 23.9 % |

* These per-run numbers and the gates above were measured before the
  [Doppler slip fix](#doppler-slip-test-and-time-shifted-base-epochs) of 2026-10-15, which changes the Odaiba u-blox
  run. They were not measured again.
* All five criteria pass in every run, including mean `ns` of S at least 5.1 above O: Galileo, QZSS and BeiDou
  satellites are used from the stream alone. S and N agree on the status of 99.4 to 100 % of the epochs. The
  both-FIXED horizontal difference is 0 m at p50 and p95 in every run; the largest single difference is 2.1 m (Odaiba
  u-blox).
* The Wrong/FIX levels are those of the urban canyon, not of the ephemeris source: S and N differ by at most 1.0 pp in a
  run (Odaiba u-blox 23.9 against 22.9 %), and pooled O has 23.8 % (1976 of 8308) on fewer FIXED epochs. Keep the
  caution in [Wrong FIX on the real-time path](#wrong-fix-on-the-real-time-path) for these receivers.
* Base stream size: O 6.7 to 7.2 kbit/s, S 7.9 to 8.3 kbit/s, 1.2 kbit/s more with the five systems.
* Realtime factor offline (three `gnss_live` processes in parallel on a 4 vCPU VM, so it is noisy): O 156 to 314x, N 33 to 96x,
  S 35 to 106x.

The first version of this check (v1, PPC Tokyo run2-3 and Nagoya run1-3, against develop `f32ea3b5`) was No-Go on one
criterion: status agreement of S against N was 77 to 91 % against 97 %. The cause was not the ephemerides. The PPC
navigation files carry an `IONOSPHERIC CORR` header that only the `--nav-rinex` arm reads (without a navigation RINEX
`gnss live` applies no Klobuchar model in its SPP seed, because `nav.ionosphere_model.valid` is false), and the Galileo
F/NAV against I/NAV choice at an equal toe depended on insertion order, which #615 fixed. A stream carries no ionosphere
parameters, so with a PPC-style navigation file `--nav-rinex` is not equivalent to ephemerides in the stream.

QZSS: the UrbanNav `base.nav` holds 95 real QZSS records (J01, J02, J03, J07) and the PPC navigation files hold
26 to 75 per file; they were used in arms S (as 1044) and N (from the file), and in the round trips and RTKLIB parity
above. No 1044 stream from a real receiver or caster was available, so no 1044 message has been checked as a
live broadcast; QZSS rests on the real records through the encode, decode and compare round trips and the bit-level
comparison with RTKLIB.

## Base-outage SPP fallback

`gnss live --base-outage-fallback spp` ([Interfaces](interfaces.md#base-outage-spp-fallback-for-gnss-live-base-outage-fallback-spp))
answers the remainder of limit 3 below: during a base outage the rover epochs that cannot get a base are solved with the
library's SPP solver and written with status `SPP`, never `FLOAT`/`FIXED`, and the RTK state is not touched.
Each live source gets a reader thread so the rover keeps flowing while the base is silent. Measured on the same setup as the stall
watchdog (PPC Tokyo run1, GPS only, 420 s window, 5 Hz rover / 1 Hz base, `moving` profile, TCP, first fault at 100 s, one run each, four
runs in parallel on a 4 vCPU VM; the gates were fixed before the first scored run):

| run | result |
|---|---|
| `base_outage@100+30`, no flag | 0 of 141 rover epochs in [101 s, 129 s] get a row; 154 rover epochs skipped without output |
| `base_outage@100+30`, `--base-outage-fallback spp` | 141 of 141 rows, all `SPP`; latency p50 0.00 s, p95 0.00 s, max 0.04 s; SPP 3D error vs the reference p50 8.5 m, p95 9.4 m, max 9.5 m (code-only, urban GPS-only, ~7 satellites) |
| recovery | first `FLOAT` row after the outage written at 130.00 s (no flag: 130.00 s); `FLOAT`+`FIXED` rows from 160 s to the end: 1100 against 1100 without the flag |
| `base_blackhole@100` + `--stall-timeout-ms 5000 --reconnect --reconnect-delay-ms 2000`, no fallback | 0 of 26 rover epochs in [101 s, 106 s] get a row |
| the same with the fallback | 26 of 26, all `SPP`; the run ends with exit 0; `FLOAT`+`FIXED` rows from 130 s: 1250, as in the clean run |
| clean 1x and 20x, with the flag | `.pos` byte-identical to the run without it, `fallback_spp_epochs=0` |

Details worth knowing: the flag changes nothing without an outage, and nothing at all when it is off (the `.pos` of the default
path is byte-identical to the previous release on the fixture, on Tokyo run1 and on a clean TCP run). Epochs within `--base-hold-seconds` of the
last base epoch are still RTK epochs with the held base; only the epochs after that become `SPP`, a silence timeout of
`max(1 s, hold + base interval)` (1.5 s here) after the newest base epoch, so the first rows of an outage are written about 1.3 s late
(those are the held `FLOAT` rows) and the `SPP` rows after that on arrival. The timeout is wall time on the main thread, so a base that is merely
slower than 1.5 s per epoch would be treated as in outage.

## Limits found and not fixed

Found by the harness, deliberately not changed here (each needs a design decision or is larger than a bug fix):

1. **`gnss live` handles one observation message per epoch, so real multi-constellation MSM does not work.** (Addressed: MSM epochs are now assembled using the multiple-message bit, see [Interfaces](interfaces.md#multi-constellation-rtcm-epoch-assembly-no-rtcm-epoch-assembly); the text below describes the behaviour before that change.)
   Each constellation arrives as its own MSM message with the same time tag, and `readNextObservation` treats every
   observation message as an epoch. A 5 min window encoded as G+R+E+J+C MSM7 produced 1 solution (6004 rover
   messages, 4800 skipped epochs, a bogus 7e7 s solution span). `gnss replay` has the same assumption. The harness
   therefore replays GPS only (`--systems G`). The fix is an epoch assembler that merges messages with the same time,
   using the MSM "multiple message" bit that the decoder currently discards (no extra latency) or the next time tag.
2. **The latency floor is one base interval.** A rover epoch is only solved after the next base epoch has been read
   (p50 0.41 s, max 0.82 s with a 1 Hz base), also with `--no-base-interp`, which only changes which base is used.
   A real low-latency mode would solve on arrival with the held or extrapolated base
   (see [online base extrapolation](online_rtk_base_extrapolation_v1.md)).

   A frozen-contract check of such a mode (2026-10-12) was **No-Go**, and the code was not merged. The candidate:
   * solved each rover epoch on arrival;
   * used the exact base if one was already read, otherwise the latest earlier base carried to the rover time by the
     causal geometry-corrected hold `rtk_base_alignment::holdBaseEpoch`, up to 2.0 s old.

   Check runs were PPC Tokyo run2–3 and Nagoya run1–3: G+R+E+J+C MSM7, `--preset low-cost`, `--nav-rinex`.

   | | default (interpolating) | solve on arrival |
   |---|---|---|
   | latency p50 / p95, 1x TCP, 300 s of Tokyo run2 | 0.416 / 0.816 s | **0.007 / 0.022 s** |
   | FIX, pooled | 28,576 | 28,131 (98.4 %) |
   | wrong FIX (> 0.5 m), pooled | 676 (2.37 %) | 695 (2.47 %) |
   | Wrong/FIX, Nagoya run3 | 72 / 1964 = 3.67 % | **153 / 2148 = 7.12 %** (gate: ≤ +2 pp) |
   | median horizontal error of correct FIX | 0.0280 m | 0.0294 m |

   * The latency goal was met, and the pooled numbers barely moved.
   * Per run, Wrong/FIX changed by −0.8 to +3.5 pp, and Nagoya run3 nearly doubled.
   * Tokyo run1 was used during development and was not scored. On it, Wrong/FIX went from 3.1 % to 10.8 %.
   * Holding a base forward over a sub-second gap is therefore not safe enough on this data to ship, even opt-in.
   * A follow-up, such as a shorter maximum age or never declaring FIXED on a held base, needs check runs that have not
     been seen. These PPC runs now have been.
3. **TCP reads have a watchdog only when asked for.** A silent but connected `tcp://` source blocks in `read()` with no
   timeout, so a dead peer that never sends a FIN blocks `gnss live`; the `base_blackhole` / `rover_blackhole` faults
   reproduce it. Opt-in fix: `gnss live --stall-timeout-ms <ms>` (with `--reconnect`, see
   [Interfaces](interfaces.md#stall-watchdog-for-gnss-live-stall-timeout-ms)); the default `0` keeps the blocking read. Still the case:
   the flag is off by default; it covers `tcp://` and serial/device reads but not NTRIP (which keeps its own `select()`
   timeout and does not reconnect on a quiet caster) or files; serial/device sources cannot reconnect, a stall ends them;
   the rover is not read while `gnss live` waits on the base (and vice versa), so during a base outage the rover socket just
   fills and recovery starts only after the stall timeout plus the reconnect delay; rover epochs that cannot get a base
   within 2 s are skipped without any output.
   Fixed, opt-in: `gnss live --base-outage-fallback spp` reads every live source in its own thread and writes an SPP row (status `SPP`,
   never `FLOAT`/`FIXED`, RTK state untouched) for each rover epoch that has no base after `max(1 s, base hold + base interval)` of
   wall time (see [Base-outage SPP fallback](#base-outage-spp-fallback)). Still the case: the flag is off by default; the fallback is
   single-point positioning (meters, no dead reckoning), a base that never delivers its first epoch or whose source has ended still
   stops the run, NTRIP reads keep their own bounded select() wait, and the rover is still waited for when it is the silent source.
4. **The GPS week.** Fixed for `gnss live`: `RTCMStreamDecoder` keeps one GPST reference (seeded from `--nav-rinex`,
   else from the first GPS/Galileo/BeiDou/QZSS ephemeris in either stream), resolves TOW-only observations to the nearest
   week, follows the stream across a week rollover, and dates GLONASS MSM epochs and 1020 ephemerides from it instead of the
   host clock, so replaying old recordings no longer depends on the host clock when a reference exists. Not fixed: a stream with
   neither `--nav-rinex` nor any GPS/Galileo/BeiDou/QZSS ephemeris before its first epochs has no reference (head epochs stay week 0,
   GLONASS is host-clock dated); a GLONASS 1020 that arrives before any reference is still dated from the host clock;
   and the 10-bit week of a 1019 is still expanded around the host clock until a reference exists. `gnss replay --base-rtcm` now uses
   the same decoder and reference (seeded from `--nav-rinex`, else from the first in-stream ephemeris); before that its raw
   `RTCMProcessor` left GPS/Galileo/QZSS/BeiDou parts in week 0 and dated GLONASS from the host clock, so a GPS-only base never
   aligned to a RINEX rover and a multi-GNSS one only matched on the GLONASS parts, and only when the host clock was near the data date.
5. **Cold start after a restart is optimistic.** With the `moving` option set the first 30 s after a relaunch
   contained only wrong fixes (32 epochs, median error 2.5 m), and the bare static gate refuses most fixes of a moving
   rover (2.9 % against 49.5 % fixed over the run, but 0 against 1055 wrong fixes). `gnss live` has no
   option set that is good at both.
6. **`gnss live` and `gnss solve` are different algorithms.** Streaming is exact (byte-identical to files), but
   `gnss_solve` adds batch guards, so the same epochs agree in status only 98.5 % (bare) or 76 % (`moving`).

## Limitations of this harness

* Loopback only: no real packet loss, reordering, MTU splitting, congestion or caster behaviour. The partial-frame
  bug was found through fault injection (corrupted length bytes) and confirmed with a hand-split stream, not seen
  on a network. The NTRIP caster is minimal (no authentication, no source table).
* One dataset run (PPC Tokyo run1) and a GPS-only subset of it; Nagoya and runs 2 and 3 are not run. The encoder
  quantises to MSM7 and writes only the satellites `gnss_live` can use.
* Faults are injected at the server; the receiver side of the link is real code, the server side is Python
  (timing jitter about 1 ms, GIL contention at hundreds of epochs per second).
* Latency uses the scheduled send time, so at speeds where `gnss_live` falls behind (the unfixed soak) it understates the lag;
  use the achieved stream speed there.
* The soak is accelerated (60x) and time-shifted; it exercises memory and CPU growth per ephemeris rebroadcast and
  per epoch, not hours of real signal conditions. Looped fix statistics are not accuracy figures.
* Wall-clock figures come from a shared 4 vCPU VM with other jobs running; compare runs by ratio, not by the last digit.
