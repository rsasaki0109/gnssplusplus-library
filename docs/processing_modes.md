# Real-time and post-processing modes

libgnss++ positions in three modes. They differ in the data a solution at
epoch *t* may use, not in the solver.

| Mode | May use | Tools | Use it for |
|---|---|---|---|
| Real-time | Data received up to *t* (with its latency) | `gnss live`, `gnss online` | Operation |
| Post-processing, causal (`--causal`) | Data tagged up to *t* | `gnss ppp --causal`, `gnss spp --causal` | Real-time performance from recorded data |
| Post-processing | Anything in the files, including data after *t* | `gnss solve`, `gnss ppp`, `gnss spp`, `gnss fgo` | Best accuracy after the fact |

A causal run is meant to give what a real-time receiver would have given on
the same data, so results quoted for real-time use should come from it. The
default post-processing run is not causal: besides choosing between future
corrections, it may interpolate, smooth or check against later epochs.

## What `--causal` covers (2026-11-07)

`gnss ppp --causal` and `gnss spp --causal`:

- **SSR corrections.** Never taken from a sample tagged after the epoch,
  whatever the correction source (`--ssr`, `--ssr-rtcm`, `--has-pages`,
  `--b2b`, `--madoca-l6`, CLAS L6):
  - no interpolation towards the next update;
  - no correction for a satellite before its first update;
  - no later CLAS atmosphere rows.

  This applies to the corrections, the ambiguity-resolution helpers and the
  CLAS OSR path alike (`SSRProducts::setCausal`).
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
- **Summary.** The summary JSON records `"causal": true`.

Without `--causal`, `gnss ppp` already takes no future SSR sample on the
held-update paths: RTCM SSR standard / `has-idd`, `--has-pages`, `--b2b`, and
coherent MADOCA. The exception is a satellite's first update, which also
serves the epochs before it. The `--ssr` CSV, raw CLAS L6 and legacy RTCM
paths interpolate between the samples around the epoch.

The default (post-processing) results do not change. Static runs with and
without `--causal` (H at 30 / 60 min, then the time after which H stays
below 0.20 m):

| Run | Default | `--causal` |
|---|---|---|
| HAS IDD, OBE4 2023-08-17 | 0.123 / 0.116 m, 5.8 min | 0.115 / 0.123 m, 5.6 min |
| JPL GDGPS RTCM SSR, Kamakura 2025-08-21 | 0.251 / 0.057 m, 42.1 min | 0.264 / 0.050 m, 42.0 min |
| JPL GDGPS RTCM SSR, Kamakura 2024-02-12 | 0.332 / 0.029 m, 38.6 min | 0.332 / 0.028 m, 38.2 min |
| HAS SIS, Kamakura 2025-02-15 | 0.027 / 0.118 m, 41.9 min | 0.074 / 0.072 m, 15.2 min |
| HAS SIS, Kamakura 2025-08-21 | 0.305 / 0.118 m, 43.9 min | 0.121 / 0.091 m, 33.9 min |
| PPP-B2b, Kamakura 2025-02-15 | 0.111 / 0.014 m, 15.8 min | 0.111 / 0.065 m, 15.8 min |
| MADOCA (L6E + L6D), MIZU sample, kinematic PPP-AR (120 epochs) | 91 FIX of 118 | 91 FIX of 118 |
| CLAS L6, CLASLIB sample (300 epochs) | H RMS 1.56 m, last 100 epochs | 3.05 m |

- **SSR alone (before the ephemeris rule).** HAS IDD and JPL were identical
  in causal mode: their streams hold corrections for every satellite before
  it is observed. So was the MADOCA sample.
- **Ephemeris rule: small changes.** The IODE-matched corrections already pin
  their broadcast record; what moves is the SPP seed and the satellites
  without a correction.
- **HAS SIS and B2b: changed.** These recordings start together with the
  observations, so in causal mode a satellite has no correction, and is not
  used, until its first update.
- **CLAS: changed.**
  - CLAS loses its interpolation towards the next update and its later
    atmosphere rows.
  - The short CLAS sample is not a stable benchmark: H RMS over the last
    100 epochs against the published antenna coordinate is 1.36 m with the
    existing no-future switch `GNSS_PPP_CLAS_BASE_CLOCK_PARITY=1`, and
    0.72 m with `GNSS_PPP_CLAS_ATMOS_LIFECYCLE=1`.

## Not covered yet

These parts still see data after the epoch, also with `--causal`; they are
the next steps of this work:

- **SSR reception time.**
  - Corrections are tagged with their reference epoch, not the time a
    receiver has them (a few seconds later; CLAS about 15 s).
  - A causal run therefore uses each update slightly early.
- **RTK (`gnss solve`)** has no causal mode yet. It uses:
  - base interpolation towards the next base epoch;
  - the non-FIX drift and FLOAT bridge-tail guards, which look at the next
    FIX.
- **`gnss replay`** preloads its streams, so it is not a real-time reference
  either. Only `gnss online` processes received events in order.

A priori configuration is not treated as future data: the RINEX header
position, ANTEX, ocean and atmospheric loading and DCB files are used in
every mode, and so is `--eop-c04`, which only feeds the tide models.
