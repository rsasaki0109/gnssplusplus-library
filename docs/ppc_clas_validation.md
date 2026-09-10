# PPC CLAS validation

Benchmark source: [GNSS測位エコシステムの統合を目指して - モダンな次世代RTKLIB「MRTKLIB」の公開](https://zenn.dev/hatognss/articles/7a54dd82606faf)

## Current verdict

The full six-run PPC moving-data hard sign-off passes. Across 58,259 scored
epochs LibGNSS++ produced 14,635 FIX epochs (25.120582%) with 0.359365 m
aggregate FIX RMS2D, 0.347847 m FIX p68, and zero FIX epochs above 3 m. Every
run covers 100% of the reference interval and at least 99.95% of observation
epochs.

The accepted policy keeps the ordinary CLAS direct state-DD floor at six rows.
Only an uninterrupted valid ambiguity hold from a preceding full-row FIX may
publish a reduced four-row solution, and the bridge is capped at five
consecutive reduced-row publications. Reduced-row results are publication-only:
they do not replace or apply the held constraints to the float filter, advance
the internal FIX lifecycle, or seed quarantine. Any non-FIX publication or
early return clears bridge eligibility.

Against the fixed baseline this adds 176 FIX epochs and loses none. Each of the
six runs independently meets or exceeds the published MRTKLIB v0.4.2 FIX rate,
beats its FIX RMS2D, retains at least 99% time and epoch coverage, and has zero
FIX epochs above 3 m. Tokyo 2 now clears its narrow rate deficit at 21.859186%
versus 21.7%, with 0.329495 m FIX RMS2D versus 0.514 m. Nagoya 2's prior unsafe
tail is absent: its maximum FIX error is 0.767281 m.

p68 and TTFF remain soft diagnostics rather than hard gates. p68 passes on
Tokyo 1 and Nagoya 3 and misses on the other four runs; TTFF passes on five
runs and misses only on Nagoya 3 (27 s versus 9 s). Aggregate all-solution,
FLOAT, and SINGLE RMS2D are 36.522236 m, 16.884790 m, and 70.360920 m.

The promoted production defaults are a held minimum of four DD rows and a
maximum publication streak of five. The previous behavior remains available
with explicit environment settings:

```text
GNSS_PPP_CLAS_AR_HELD_MIN_DD_ROWS=6
GNSS_PPP_CLAS_AR_HELD_MAX_PUBLICATION_STREAK=1
```

The precision comparison is directly comparable: both implementations are
scored against the raw, already antenna-positioned PPC reference with no
lever-arm transform.

## Reference-point methodology

PPC's `reference.csv` is already an antenna-positioned trajectory, not an
IMU/vehicle-frame trajectory: projecting the no-lever-arm fixed-epoch error
vectors into the vehicle body frame (via the reference heading) on Nagoya
run3 gives a mean of (0.075, 0.088) m — essentially zero — rather than the
(0.593, -0.670) m horizontal offset that the city-specific vehicle→antenna
lever arm would produce if the reference were truly at the IMU. Earlier
tables nonetheless applied that lever-arm correction to the reference on top
of an already-antenna-positioned reference, a double correction that
inflated FIX RMS2D by roughly 0.89 m (Nagoya) and 0.31 m (Tokyo),
common-mode across epochs. A per-epoch error correlation of r=0.991 against
a real MRTKLIB v0.5.1 rerun on 206 co-fixed Nagoya run3 epochs confirms the
bias was reference-side, not solver-side. The published MRTKLIB v0.4.2
targets were computed without any lever-arm transform (raw reference), so
the raw-reference scoring used from this point forward is the
apples-to-apples basis for comparison. One side effect of the old bias was
flattering: in a historical baseline it shifted a 19-epoch Nagoya 2 burst
from an honest 3.200 m maximum to 2.486 m, creating a false "no FIX epoch
above 3 m" claim. The accepted candidate removes that burst (Nagoya 2 maximum
0.767281 m), but raw-antenna scoring remains essential for an honest gate.

## Non-FIX continuity fix

MRTKLIB's SPP least-squares path writes a finite current position before its
final chi-square validation. Native CLAS rejected that candidate and could
publish the last accepted SPP position for a long rejection stretch, allowing
the reported SINGLE error to grow as the vehicle moved. Directly admitting the
rejected candidate to the filter improved Tokyo 2 but materially regressed
Nagoya 2, so that approach was rejected.

The accepted implementation keeps the existing filter, `float_count`,
`cntdiffp`, maxdiff, and AR lifecycle unchanged. A separate output-only tracker
publishes a validation-rejected SPP candidate only when its per-step motion is
causal and it remains within 150 m of the last trustworthy publication. This
prevents both stale-position coast and a sequence of small but cumulatively
unbounded SPP jumps. The six-run audit found zero status differences and zero
non-SINGLE coordinate differences versus the prior baseline.

## Historical 480-second probe (not final sign-off)

The values below predate the city-specific vehicle-to-antenna lever-arm
correction.  They are retained only to document the earlier false-FIX
diagnosis and are not used by the full six-run acceptance result.  (The
lever-arm transform mentioned below was later found to double-correct an
already-antenna-positioned reference; see "Reference-point methodology"
above — the current six-run acceptance result scores against the raw PPC
reference, matching the MRTKLIB convention.)

- Dataset: PPC Tokyo run2, first 2400 receiver epochs (480 seconds at 5 Hz)
- Solver: kinematic CLAS OSR, dynamics model enabled, parity environment gates
- Historical reference: published `reference.csv` coordinates without a
  lever-arm transform
- Warm-up: discard the first 60 matched epochs
- FIX: LibGNSS++ status 6 (article NMEA quality 4 equivalent)
- TTFF: start of the first run of at least 30 consecutive FIX epochs
- `1sigma`: 68th percentile of FIX horizontal error, matching the article's
  `p68_2d_fix` definition

## Tokyo run2 result

| Metric | MRTKLIB v0.4.2 full run | LibGNSS++ 480 s probe | Assessment |
|---|---:|---:|---|
| Fix rate | 21.7% | 5.060% | Partial-window value; not directly comparable |
| FIX RMS2D | 0.514 m | 0.114 m | Better than target in this window |
| FIX 68th percentile | 0.120 m | 0.091 m | Better than target in this window |
| TTFF | 368 s | 301.2 s | 66.8 s earlier in this window |

The corrected probe contains 2332 scored epochs and 118 fixed epochs:

- First 120 seconds: 25 FIX, RMS2D 0.192 m, p68 0.194 m.
- About 230--260 seconds: zero FIX.  The former 34 approximately 10 m false fixes
  are now 32 FLOAT and 2 SINGLE solutions.
- 260--370 seconds: 32 FIX, RMS2D 0.094 m, p68 0.076 m.  The first 30-epoch
  continuous run starts at 301.2 seconds.
- 370--480 seconds: 61 FIX, RMS2D 0.074 m, p68 0.084 m.

The root cause was an incomplete MRTKLIB `maxdiffp` reset.  With dynamics
enabled, MRTKLIB clears the complete filter state after `SOLQ_NONE`; LibGNSS++
previously reset only position and velocity, leaving tightly constrained stale
ionosphere and ambiguity states attached to the new SPP position.  Clearing the
full filter and AR state removed all 34 false fixes while preserving all 25
early valid fixes bit-for-bit.  In the 1300-epoch A/B window, FIX RMS2D improved
from 7.640 m to 0.192 m.

## Artifacts

- Complete result table: [ppc_clas_full_table.md](ppc_clas_full_table.md)
- Machine-readable metrics: [ppc_clas_full_metrics.json](ppc_clas_full_metrics.json)
- Metric comparison: [ppc_clas_full_comparison.png](ppc_clas_full_comparison.png)
- Complete trajectories: [ppc_clas_full_trajectories.png](ppc_clas_full_trajectories.png)
- Horizontal-error histories: [ppc_clas_full_errors.png](ppc_clas_full_errors.png)

## Next development gates

1. Improve the four remaining p68 soft misses without reducing FIX rate.
2. Improve Nagoya 3 TTFF while preserving its narrow 0.318 m RMS2D margin.
3. Improve FLOAT recovery; the publication-only held-DD policy intentionally
   leaves the float-filter lifecycle unchanged.
