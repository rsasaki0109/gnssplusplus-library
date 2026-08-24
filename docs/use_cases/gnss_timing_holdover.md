# GNSS timing and holdover assessment

R9 evaluates receiver-clock estimates without pretending that a position FIX
or an offline clock column is an NTP/PTP service. It uses BRUX00BEL because the
IGS final CLK product contains an independent station-clock series and the
official IGS station metadata identifies an external cesium standard. The
released assessment is **No-Go**: the sealed day exceeded the frozen clock-step
gate, and neither day contains a physical PPS outage test.

## Commands

Run a bounded 120-epoch smoke entirely from a verified cache:

```bash
python3 apps/gnss.py timing-holdover-workflow \
  --phase development --mode smoke \
  --output-dir output/timing_r9/smoke \
  --cache-dir data/timing_r9/cache --offline
```

The full development replay is:

```bash
python3 apps/gnss.py timing-holdover-workflow \
  --phase development --mode full \
  --output-dir output/timing_r9/development-replay \
  --cache-dir data/timing_r9/cache
```

The 2024-01-02 holdout is `closed_no_go`. Do not change it back to `sealed` or
rerun it. Select a different hash-pinned day for a future release.

## Data and clock contract

The workflow fetches BKG IGS 30-second BRUX observations, broadcast navigation,
the IGS final CLK product, the official IGS Network station API snapshot, and
the GSI RNXCMP converter. Every compressed and materialized file has byte-count
and SHA-256 provenance. The 2024 RINEX headers agree across both days:

- monument `13101M010`;
- receiver `3057609`, SEPT POLARX5TR firmware 5.5.0;
- antenna `00464`, JAVRINGANT_DM SCIS.

The current pinned IGS API snapshot reports the same monument, receiver serial,
antenna serial/type, IGS20 membership, and an external cesium frequency
standard; its current firmware field is 5.7.0. Historical RINEX, not the later
API firmware, is authoritative for the two 2024 runs.

The normal `.pos` schema intentionally remains unchanged. `gnss spp` adds an
optional `--clock-csv` artifact containing SPP receiver clock bias in metres,
the explicit metres-to-seconds conversion, Doppler clock drift, status,
satellites, and PDOP. This is important because the internal SPP bias state is
range-equivalent metres even though other solver interfaces may describe clock
states differently.

IGS final CLK `AR BRUX` rows are an independent combined receiver-clock
reference aligned to the IGS time scale. They are available every five minutes:
288 points on the development day and 283 on the holdout. The workflow matches
by GPST epoch and reports absolute phase error and finite-difference relative
frequency error.

## Simulated holdover

At each eligible IGS clock epoch, the evaluator fits a line to the preceding
1,800 seconds of GNSS-connected SPP clock bias and extrapolates 300, 600, 900,
or 1,800 seconds. It then compares that prediction with the future IGS clock.
This emulates an open-loop model but does **not** disconnect the receiver or
measure its PPS output. It therefore cannot reveal real oscillator switching,
temperature, power, cable, timestamping, kernel, NIC, or network-servo effects.

The frozen development gates were 95% lock/reference coverage, lock phase p95
at most 39 ns, maximum five-minute lock step at most 12 ns, and holdover p95 at
most 40/40/40/42 ns for 5/10/15/30 minutes. Development passed with:

- 100% reference coverage and lock phase p95 25.71 ns;
- maximum lock step 7.41 ns;
- relative-frequency p95 `7.94e-12`;
- simulated holdover p95 26.05–27.78 ns.

The sealed day kept 100% coverage, phase p95 21.26 ns, and holdover p95
21.86–22.52 ns, but its maximum five-minute phase-error step was 14.71 ns.
That exceeds the unchanged 12 ns gate, so the assessment is degraded and the
release is No-Go. Evidence is preserved in the
[development record](records/timing_r9_development.json) and
[holdout run1 record](records/timing_r9_holdout_run1.json).

## Outputs and service boundary

The bundle contains `spp.pos`, `receiver_clock.csv`, SPP summary, workflow log
and manifest. The sign-off directory adds `clock_comparison.csv`,
`clock_scorecard.png`, summary, and manifest.

No NTP/PTP daemon or PPS service is exposed. Before one can be considered,
perform a real GNSS disconnect/reconnect test while comparing physical PPS to
an independent traceable counter, record oscillator/environment/power state,
measure system-clock and network timestamping paths, define UTC/leap-second
handling, characterize MTIE/TDEV/Allan deviation, freeze alert/holdover limits,
and validate a new untouched multi-day holdout.
