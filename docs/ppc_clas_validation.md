# PPC CLAS validation

Benchmark source: [GNSS測位エコシステムの統合を目指して - モダンな次世代RTKLIB「MRTKLIB」の公開](https://zenn.dev/hatognss/articles/7a54dd82606faf)

## Current verdict

LibGNSS++ has not yet demonstrated full six-run equivalence with the MRTKLIB
v0.4.2 PPC CLAS benchmark.  The Tokyo run2 480-second quality gate now has no
false fixes: all 118 fixed epochs are within 0.305 m horizontally, aggregate FIX
RMS2D is 0.114 m, and the 30-epoch TTFF is 301.2 seconds.  The remaining work is
full-window, six-run validation and FLOAT trajectory recovery/performance.

## Article-compatible scoring

- Dataset: PPC Tokyo run2, first 2400 receiver epochs (480 seconds at 5 Hz)
- Solver: kinematic CLAS OSR, dynamics model enabled, parity environment gates
- Reference: published `reference.csv` coordinates, no lever-arm transform
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

- Corrected position output:
  `output/ppc_clas_tokyo_run2_probe2400_fixed/native.pos`
- Machine-readable metrics:
  `output/ppc_clas_tokyo_run2_probe2400_fixed/article_compatible_metrics.json`
- Pre-fix position output: `output/ppc_clas_tokyo_run2_probe2400/native.pos`
- Short probe metrics:
  `output/ppc_clas_tokyo_run2_probe600/article_compatible_metrics.json`

## Next development gates

1. Index SSR corrections by epoch instead of repeatedly searching the expanded
   history.  The 480-second probe took about 47 minutes even with a time-trimmed
   SSR CSV, which is too slow for a six-run regression suite.
2. Improve FLOAT recovery after `maxdiffp`; FIX integrity now passes, while the
   480-second all-solution RMS2D remains 11.630 m.
3. Run the complete Tokyo run2 window and require aggregate metrics at least as
   good as the article target without weakening its scoring definition.
4. Run all six PPC sequences and publish a single default-profile scorecard.
