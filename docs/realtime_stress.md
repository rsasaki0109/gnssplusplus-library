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
`--station`, `--rinex-out`). The ctest lane `python_realtime_stress_tests` replays a tracked 40 s
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

`--supervise` relaunches `gnss_live` when it exits early (what an init system would do);
`--reconnect` runs it with its own `--reconnect`.

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

**Do not trust a `gnss live` FIX in a dense urban canyon.** In GPS-only recorded
replays, `gnss live --preset low-cost` published FIX epochs of which 48.7 % were
more than 0.5 m off on UrbanNav Shinjuku (Trimble rover; 1710 of 3510) and
32.4 % on Shinjuku (u-blox), against 0.7–17.9 % on the PPC runs below. `gnss
live` lacks `gnss solve`'s whole-solution post-filters, and GPS alone gives weak
ambiguity geometry. Multi-constellation MSM now works in `gnss live` and lowers
Wrong/FIX on every PPC run (see [Multi-constellation MSM](#multi-constellation-msm-epoch-assembly));
the UrbanNav Shinjuku runs have not been replayed multi-constellation yet.

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
  it. They are not evidence for a default.

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
3. **No watchdog on TCP reads.** A silent but connected `tcp://` source blocks in `read()` with no timeout, so a dead peer
   that never sends a FIN would block it too (not tested here); during a base outage the rover is not read either. Rover epochs that
   cannot get a base within 2 s are skipped without any output, so there is no SPP or dead-reckoned fallback for the
   outage.
4. **The GPS week.** Fixed for `gnss live`: `RTCMStreamDecoder` keeps one GPST reference (seeded from `--nav-rinex`,
   else from the first GPS/Galileo/BeiDou/QZSS ephemeris in either stream), resolves TOW-only observations to the nearest
   week, follows the stream across a week rollover, and dates GLONASS MSM epochs and 1020 ephemerides from it instead of the
   host clock, so replaying old recordings no longer depends on the host clock when a reference exists. Not fixed: a stream with
   neither `--nav-rinex` nor any GPS/Galileo/BeiDou/QZSS ephemeris before its first epochs has no reference (head epochs stay week 0,
   GLONASS is host-clock dated); a GLONASS 1020 that arrives before any reference is still dated from the host clock;
   the 10-bit week of a 1019 is still expanded around the host clock until a reference exists; and `gnss replay`'s RTCM base path has no
   reference at all.
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
