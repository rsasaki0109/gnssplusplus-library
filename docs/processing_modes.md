# Real-time and post-processing modes

libgnss++ positions in three modes. They differ in the data a solution at
epoch *t* may use, not in the solver.

| Mode | May use | Tools | Use it for |
|---|---|---|---|
| Real-time | Data received up to *t* (with its latency) | `gnss live`, `gnss online` | Operation |
| Post-processing, real-time equivalent (`--realtime`) | Data tagged up to *t* | `gnss solve --realtime`, `gnss replay --realtime`, `gnss ppp --realtime`, `gnss spp --realtime` | Real-time performance from recorded data |
| Post-processing | Anything in the files, including data after *t* | `gnss solve`, `gnss ppp`, `gnss spp`, `gnss fgo`; `--combined` for `gnss solve` / `gnss ppp` | Best accuracy after the fact |

A `--realtime` run is meant to give what a real-time receiver would have
given on the same data, so results quoted for real-time use should come from
it. The default post-processing run is not: besides choosing between future
corrections, it may interpolate, smooth or check against later epochs.

In the library API the same mode is called *causal*: a solution uses no
data from after its epoch (`PPPConfig::causal`, `SPPConfig::causal`,
`SSRProducts::setCausal()`, `NavigationData::setCausal()`).

## What `--realtime` covers (2026-11-07)

`gnss ppp --realtime` and `gnss spp --realtime`:

- **SSR corrections.** Never taken from a sample tagged after the epoch,
  whatever the correction source (`--ssr`, `--ssr-rtcm`, `--has-pages`,
  `--b2b`, `--madoca-l6`, CLAS L6):
  - no interpolation towards the next update;
  - no correction for a satellite before its first update;
  - no later CLAS atmosphere rows.

  This applies to the corrections, the ambiguity-resolution helpers and the
  CLAS OSR path alike (`SSRProducts::setCausal`).
- **SSR reception.** An update applies from the time a receiver has it, not
  from its reference epoch; the previous one stays in use until then:
  - HAS SIS (`--has-pages`) and PPP-B2b (`--b2b`): from the reception of the
    page or message that completes it. On the Kamakura hours that is 1 to 4 s
    after the reference epoch for HAS (median 1 s), 6 to 29 s for B2b
    (median 7 s). Updates apply in the order they are received.
  - RTCM SSR (`--ssr-rtcm`) and MADOCA L6E / L6D (`--madoca-l6`,
    `--madoca-l6d`): the files hold no reception time. `--ssr-latency <s>`
    applies each update *s* seconds after its reference epoch (default 0).
    In an L6 log the corrections all sit at the same offset from the
    one-per-second L6 message that completes them, so the log does not give
    the latency either.
  - With RTCM SSR, PPP starts once the first update is in hand; the epochs
    before it are SPP. Starting on broadcast orbits and switching every
    satellite to its correction a few seconds later jumps the filter (HAS
    IDD with a 5 s latency: H 1.9 m at 2 min, converged after 20 min instead
    of 5.6). Satellites the stream never corrects stay on their broadcast
    orbit, as in post-processing.
- **Broadcast ephemerides.** Only records already broadcast at the epoch
  are candidates (`NavigationData::setCausal`). A record is available from
  its transmission time, which the RINEX reader now keeps for GPS, Galileo,
  QZSS and BeiDou records, as for GLONASS and the RTCM / UBX decoders.

  Merged BRDC files, and the RTCM 1019 / 1046 records of an SSR file, carry
  no transmission time. For those, a record is available from:
  - GPS: two hours before its toe;
  - QZSS: one hour before its toe;
  - Galileo and the other systems: its toe.

  These offsets are when each system starts broadcasting a record on the
  receiver NAV files of the Kamakura hours (Galileo actually starts some
  minutes after its toe).
- **Post-processed products are refused.** `--sp3`, `--clk` and `--ionex`
  hold final products, interpolated around the epoch.
  `--ssr-rtcm-profile legacy` interpolates its samples between updates while
  loading. `--madocalib-bridge` hands the run to MADOCALIB.
- **Summary.** The summary JSON records `"realtime": true` and
  `"ssr_latency_s"`.

Without `--realtime`, `gnss ppp` already takes no future SSR sample on the
held-update paths: RTCM SSR standard / `has-idd`, `--has-pages`, `--b2b`, and
coherent MADOCA. The exception is a satellite's first update, which also
serves the epochs before it. The `--ssr` CSV, raw CLAS L6 and legacy RTCM
paths interpolate between the samples around the epoch.

The default (post-processing) results do not change. Static runs with and
without `--realtime` (H at 30 / 60 min, then the time after which H stays
below 0.20 m):

| Run | Default | `--realtime` |
|---|---|---|
| HAS IDD, OBE4 2023-08-17 | 0.123 / 0.116 m, 5.8 min | 0.115 / 0.123 m, 5.6 min |
| JPL GDGPS RTCM SSR, Kamakura 2025-08-21 | 0.251 / 0.057 m, 42.1 min | 0.264 / 0.050 m, 42.0 min |
| JPL GDGPS RTCM SSR, Kamakura 2024-02-12 | 0.332 / 0.029 m, 38.6 min | 0.332 / 0.028 m, 38.2 min |
| HAS SIS, Kamakura 2025-02-15 | 0.027 / 0.118 m, 41.9 min | 0.075 / 0.071 m, 15.2 min |
| HAS SIS, Kamakura 2025-08-21 | 0.305 / 0.118 m, 43.9 min | 0.119 / 0.091 m, 34.0 min |
| PPP-B2b, Kamakura 2025-02-15 | 0.111 / 0.014 m, 15.8 min | 0.113 / 0.063 m, 16.2 min |
| MADOCA (L6E + L6D), MIZU sample, kinematic PPP-AR (120 epochs) | 91 FIX of 118 | 91 FIX of 118 |
| CLAS L6, CLASLIB sample (300 epochs) | H RMS 1.56 m, last 100 epochs | 3.05 m |

- **SSR alone (before the ephemeris rule).** HAS IDD and JPL were identical
  with `--realtime`: their streams hold corrections for every satellite before
  it is observed. So was the MADOCA sample.
- **Ephemeris rule: small changes.** The IODE-matched corrections already pin
  their broadcast record; what moves is the SPP seed and the satellites
  without a correction.
- **HAS SIS and B2b: changed.** These recordings start together with the
  observations, so with `--realtime` a satellite has no correction, and is not
  used, until its first update.
- **Reception time: small changes.** Using HAS SIS and B2b updates from
  their reception moved H at 30 / 60 min by at most 2 mm. The RTCM SSR rows
  above use no latency; see the table below.
- **CLAS: changed.**
  - CLAS loses its interpolation towards the next update and its later
    atmosphere rows.
  - The short CLAS sample is not a stable benchmark: H RMS over the last
    100 epochs against the published antenna coordinate is 1.36 m with the
    existing no-future switch `GNSS_PPP_CLAS_BASE_CLOCK_PARITY=1`, and
    0.72 m with `GNSS_PPP_CLAS_ATMOS_LIFECYCLE=1`.

MADOCA PPP-AR (MIZU sample with L6D, kinematic, 118 epochs) with
`--realtime --ssr-latency <s>`, against a 24 h static PPP of MIZU with the
IGS final orbits and clocks:

| Latency | FIX | H RMS | U RMS |
|---|---|---|---|
| 0 s | 91 | 0.099 m | 0.154 m |
| 5 s | 92 | 0.101 m | 0.161 m |
| 10 s | 90 | 0.115 m | 0.185 m |
| 20 s | 93 | 0.120 m | 0.186 m |

RTCM SSR with `--realtime --ssr-latency <s>` (same columns):

| Run | 0 s | 5 s | 10 s |
|---|---|---|---|
| HAS IDD, OBE4 2023-08-17 | 0.115 / 0.123 m, 5.6 min | 0.117 / 0.122 m, 5.9 min | 0.120 / 0.121 m, 7.5 min |
| JPL GDGPS, Kamakura 2025-08-21 | 0.264 / 0.050 m, 42.0 min | 0.279 / 0.051 m, 42.1 min | 0.269 / 0.051 m, 42.1 min |
| JPL GDGPS, Kamakura 2024-02-12 | 0.332 / 0.028 m, 38.2 min | 0.327 / 0.029 m, 38.6 min | 0.308 / 0.031 m, 38.1 min |

## RTK: `gnss solve --realtime`

- **No base interpolation.** By default `gnss solve` interpolates the base
  observations to the rover epoch between the base epochs around it.
  `--realtime` uses only an exact base epoch. With `--base-hold-seconds <s>` it
  also uses the latest past base epoch up to *s* seconds old, modeled to the
  rover time (`rtk_base_alignment::holdBaseEpoch()`, the hold of the online
  RTK / IMU processor's 2 s `rtk_base_extrapolation_v1` candidate).
- **No look-ahead guards.** The non-FIX drift guard and the FLOAT
  bridge-tail guard are off: both check an epoch against the next FIX.
  `--fixed-bridge-burst-guard`, `--safe-availability-fallback` and
  `--rover-seed-pos` (external per-epoch positions) are refused.
- **Ephemerides only once broadcast**, as for PPP.
- **`--integrity-shadow-csv` must carry its `solution_latency_s` and
  `causal_provenance_verified` columns.** Without them a smoother's output
  would be read as zero-latency and causal.
- **Summary.** The summary prints `realtime: yes` and the number of held base
  epochs.

On the `tests/fixtures/realtime_stress` drive (201 rover epochs at 5 Hz, 41
base epochs at 1 Hz), against its reference trajectory:

| Run | Epochs | FIX | H median / 95 % |
|---|---:|---:|---|
| `gnss solve` (default, base interpolation) | 201 | 197 | 0.043 / 0.108 m |
| `gnss solve --realtime` | 41 | 12 | 0.171 / 0.318 m |
| `gnss solve --realtime --base-hold-seconds 2` | 201 | 197 | 0.043 / 0.113 m |

## Replay: `gnss replay --realtime`

`gnss replay` reads its whole base stream before solving. With `--realtime`
it still only uses what had arrived by each epoch:

- **Ephemerides.** Only ephemerides already broadcast (`--nav-rinex`) or
  received are used. The RTCM decoder dates each stream ephemeris with the
  time of the latest observation epoch decoded before it, and a rebroadcast
  with that of its first reception. `gnss convert` dates RTCM ephemerides
  the same way. Ephemerides that arrive before the first observation epoch
  stay undated and follow the toe rule above.
- **Base position.** The station position is the first one received, not
  the last one of the stream, and no rover epoch before it arrived is
  solved.
- **Base alignment.** As before, only exactly aligned base epochs are used.

RINEX NAV files written from such streams carry the transmission time;
an unknown one is written as `0.9999E9` (it used to be 0, which a reader
takes as broadcast from the start of the week).

## Checking that no later data is used

`tests/cli_cases/runtime_receiver.py`
(`test_realtime_solve_and_replay_ignore_later_data_and_agree`) runs the
stress fixture:

- **Cut the data, compare the epochs before the cut.** The rover and base
  files are cut between two base epochs (`gnss solve`), or the base stream
  after 60 messages (`gnss replay --rtcm-message-limit`). Every epoch before
  the cut must be identical to the full run:
  - `gnss solve --realtime`: identical;
  - `gnss solve --realtime --base-hold-seconds 2`: identical;
  - `gnss replay --realtime`: identical;
  - default `gnss solve`: the last two epochs before the cut differ (they
    were interpolated towards the base epoch after it).
- **Replay against post-processing.** `gnss replay --realtime` (base RTCM
  stream) and `gnss solve --realtime` (the same data converted to RINEX)
  solve the same 41 epochs with the same FIX / FLOAT status, at most 0.8 mm
  apart.

## Not covered yet

These parts still see data after the epoch, also with `--realtime`; they are
the next steps of this work:

- **CLAS reception time.** CLAS corrections are still tagged with their
  reference epoch, not the time a receiver has them (about 15 s later), so a
  `--realtime` run uses each update slightly early.

A priori configuration is not treated as future data: the RINEX header
position, ANTEX, ocean and atmospheric loading and DCB files are used in
every mode, and so is `--eop-c04`, which only feeds the tide models.

## Forward + backward: `gnss solve --combined`

A post-processing RTK run that uses every later epoch: the filter runs
forward, then a second, fresh filter runs the same epochs backward, and each
epoch combines the two passes as RTKLIB's combined solution does:

- **Better status wins** (FIX > FLOAT > DGPS > SPP). A FIX of the backward
  pass fills, for example, the FLOAT epochs after an outage, before the
  forward filter has fixed again.
- **Same status: the covariance-weighted mean.** FLOAT epochs use the
  filter's own covariance.
- **Two FIXes more than `--combined-fix-tolerance` apart** (default 5 cm)
  fixed different ambiguities: their mean is kept as FLOAT.
- **A FIX the other pass has no FLOAT or FIX solution for** is reported as
  FLOAT. On the runs below such FIXes were wrong one time in five, against 1
  in 75 for a FIX the other pass solved FLOAT and 1 in 1500 for two agreeing
  FIXes.

The backward pass replays each epoch with the base epoch aligned to it on the
forward pass, takes each loss-of-lock flag from the epoch after it (a slip
lies between the two), and runs every in-loop guard. The guards after the
loop (non-FIX drift, FLOAT bridge tail, kinematic post-filter) run on the
combined solution. `--combined` cannot be used with `--realtime`,
`--fix-recovery`, `--realtime-fix-integrity` or `--pos-fix-diagnostics`;
per-epoch logs describe the forward pass. The backward pass reads the epochs
from the files again (the forward pass records where each one starts): about
10 % more memory and twice the run time of a forward run.

The six PPC Tokyo / Nagoya runs with the sign-off profile
(`configs/benchmarks/ppc_sigma_demote_nis2_ratio4.toml`), against their
reference trajectories:

| Run | FIX | Wrong FIX (> 0.5 m, of FIX) | PPC official score |
|---|---|---|---|
| Tokyo run1 | 59.6 → 65.5 % | 1.75 → 1.72 % | 71.8 → 78.2 % |
| Tokyo run2 | 62.3 → 73.1 % | 0.33 → 0.37 % | 83.7 → 87.3 % |
| Tokyo run3 | 66.3 → 70.9 % | 0.47 → 0.55 % | 77.4 → 81.0 % |
| Nagoya run1 | 64.5 → 73.0 % | 0.79 → 0.36 % | 51.5 → 59.8 % |
| Nagoya run2 | 33.3 → 46.6 % | 0.22 → 0.61 % | 32.1 → 37.3 % |
| Nagoya run3 | 30.0 → 36.8 % | 5.53 → 4.97 % | 43.0 → 45.8 % |
| All six | 51.0 → 61.1 % of the reference epochs | 0.96 → 0.96 % | 67.5 → 72.3 % (distance-weighted) |

More epochs are positioned (83-97 % → 93-99 %); the added ones are the hard
ones, so P95 H grows on four runs.

The default (forward) output does not change: the RTK filter now measures
the time since an earlier epoch in either direction, which changes nothing
when time only increases. Before that, the backward pass flagged a cycle slip
on every satellite at every epoch (the Doppler prediction used a forward time
step) and its jump gates, which grow with the elapsed time, rejected every
epoch once the vehicle had moved.

## Forward + backward PPP: `gnss ppp --combined`

The same combination for PPP: a copy of the loaded processor (products,
configuration) runs the epochs backward after the forward pass, and each
epoch combines the two passes as above. The PPP filter now predicts with the
signed time step (backward on that pass) and grows its process noise with
the elapsed time. The default (forward) output does not change: HAS IDD, HAS
SIS (both hours), B2b, JPL GDGPS (both hours), MADOCA and CLAS are
byte-identical.

The backward pass covers the forward filter's convergence: static runs,
then kinematic ones, H at 10 / 30 min and the time after which H stays below
0.20 m:

| Run | Static | Static `--combined` | Kinematic | Kinematic `--combined` |
|---|---|---|---|---|
| HAS SIS, Kamakura 2025-02-15 | 0.379 / 0.027 m, 41.9 min | 0.126 / 0.084 m, 0 min | 0.409 / 0.052 m | 0.066 / 0.043 m |
| HAS SIS, Kamakura 2025-08-21 | 0.515 / 0.305 m, 43.9 min | 0.097 / 0.092 m, 0 min | 0.315 / 0.177 m | 0.131 / 0.084 m |
| PPP-B2b, Kamakura 2025-02-15 | 0.466 / 0.111 m, 15.8 min | 0.022 / 0.007 m, 0 min | 0.493 / 0.108 m | 0.145 / 0.056 m |
| JPL GDGPS, Kamakura 2025-08-21 | 0.507 / 0.251 m, 42.1 min | 0.188 / 0.153 m, 0 min | 0.216 / 0.063 m | 0.082 / 0.028 m |
| JPL GDGPS, Kamakura 2024-02-12 | 0.274 / 0.332 m, 38.6 min | 0.172 / 0.112 m, 0 min | 0.198 / 0.215 m | 0.045 / 0.043 m |
| HAS IDD, OBE4 2023-08-17 | 0.063 / 0.123 m, 5.8 min | 0.122 / 0.250 m, 41.2 min | 0.127 / 0.205 m | 0.223 / 0.240 m |

- **OBE4 gets worse.** Its E1 / E5a drift (see [Galileo HAS
  support](galileo_has.md)) gives the two passes different height biases,
  and their mean is further from the reference in the middle of the hour.
- **MADOCA PPP-AR** (MIZU sample with L6D, kinematic, 118 epochs), against
  a 24 h static PPP of MIZU with the IGS final orbits and clocks:

  | | FIX | H RMS | U RMS |
  |---|---|---|---|
  | Forward | 91 | 0.099 m | 0.154 m |
  | `--combined` | 113 | 0.032 m | 0.031 m |

  No FIX is more than 0.1 m off horizontally. The two passes' PPP-AR fixes
  agree to a few cm horizontally but up to 10 cm in height, so
  `--combined-fix-tolerance` defaults to 0.10 m in `gnss ppp` (5 cm would
  have kept 21 correct FIX epochs FLOAT); pairs further apart had one fix
  0.2-0.4 m off.
- **Not with CLAS** (`--clas-osr`): its SIS continuity and atmosphere
  handling only run forward. Nor with `--realtime`, `--live-output` or
  `--madocalib-bridge`.
- The summary JSON records `"combined"`; the convergence and AR telemetry
  describe the forward pass.
