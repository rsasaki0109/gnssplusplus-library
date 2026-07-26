# PPC CLAS validation

Benchmark source: [GNSS測位エコシステムの統合を目指して - モダンな次世代RTKLIB「MRTKLIB」の公開](https://zenn.dev/hatognss/articles/7a54dd82606faf)

## Current verdict

The full six-run PPC moving-data validation is complete. After the
hold-continuation carve-out landed in #349 and the outage-counter parity fix
landed in #351, across 58,259 scored epochs LibGNSS++ produced 14,478 FIX
epochs (24.851%) with 0.377 m aggregate FIX RMS2D and 0.349 m FIX p68 (up
from the pre-#351 24.110% / 0.370 m aggregate, and the earlier 23.645% /
0.593 m aggregate before #349). The outage-counter fix clears a
per-satellite outage counter from that epoch's raw observation validity
(matching MRTKLIB's reset), instead of leaving it reachable only through a
narrower DD/measurement-row path; previously a satellite that briefly
dropped out of that path for one epoch could park above the outage
threshold indefinitely and keep re-triggering false "outage_sat" flags and
float-state wipes even while still being observed and corrected. The effect
is largest on Nagoya 3, where it was breaking healthy fix holds most often
(FIX rate nearly doubles, 4.865% to 8.776%), with smaller gains on the other
five runs. Every run covers 100% of the reference interval and at
least 99.95% of observation epochs.

Of the 14,478 FIX epochs, 19 (0.03%) exceed 3 m horizontal error. All 19 sit
in a single contiguous 4-second burst on Nagoya 2 (TOW 556406.4–556410.4,
errors 3.17–3.20 m, max 3.200 m), inside the known seed-geometry `maxdiffp`
reset zone. The identical 19 epochs, at matching TOWs and errors to 0.01 m,
are present in the pre-#349 baseline run (`clas_current_develop_20260723`)
and are unchanged by the #351 outage-counter fix, so this is a pre-existing
wrong-fix tail rather than a regression from either change. The other five
runs have zero FIX epochs above 3 m. See "Reference-point methodology" below
for why this tail was not visible in earlier tables.

The same gate now reports non-FIX quality explicitly. Aggregate all-solution
RMS2D is 36.523 m, FLOAT RMS2D is 16.843 m, and SINGLE RMS2D is 70.337 m.

Compared with the published MRTKLIB v0.4.2 results, native FIX rate is now
higher on five of the six runs — Tokyo 1, Tokyo 3, Nagoya 1, Nagoya 2, and
Nagoya 3, which the outage-counter fix pushes from 4.865% (below its 6.3%
target) to 8.776% (above it). Tokyo 2 remains the sole run below target,
and only narrowly: 21.507% native vs. 21.7% MRTKLIB, within 0.2 percentage
points. Native FIX RMS2D is now lower than MRTKLIB's on all six runs,
including Nagoya 3 (0.304 m vs. 0.318 m), which had been the closest margin.
The precision comparison is directly comparable, not merely contextual: both
sides now score against the same raw (already antenna-positioned) PPC
reference point, with no lever-arm transform applied on either side.

This is therefore a completed six-run safety and performance sign-off, not a
claim of per-metric equivalence on every run. The active remaining work is
same-reference MRTKLIB replay coverage for all six runs and improved FLOAT
trajectory recovery; this change intentionally targets stale SINGLE output
without perturbing the FLOAT filter.

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
flattering: shifting Nagoya 2 errors down by ~0.89 m happened to pull the
19-epoch wrong-fix burst described above under the 3 m line (old Nagoya 2
max was 2.486 m vs. the honest 3.200 m), so the previous "no FIX epoch above
3 m" claim was itself partly a scoring artifact rather than a true property
of the solution — another reason the raw-antenna basis is the honest one to
report.

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

1. Re-run MRTKLIB on all six PPC sequences and score both implementations
   against the same antenna-phase-center reference.
2. Improve Nagoya 3 FIX precision, and resolve the pre-existing 19-epoch
   Nagoya 2 wrong-fix burst (TOW 556406.4–556410.4), without introducing any
   new FIX epochs above 3 m relative to the audited baseline.
3. Improve FLOAT recovery, which is unchanged by the output-only SINGLE fix.
4. Promote the six-run generator and cached outputs into a repeatable release
   sign-off so README assets cannot drift behind solver behavior.
