# CLAS DD PPP-RTK Filter A1

A1 replaces the `GNSS_PPP_CLAS_DD_FILTER=1` passthrough scaffold with a
persistent double-differenced float filter.  The gate remains off by default;
the native CLAS and MADOCA paths do not call this code unless the environment
override is exactly `1`.

## Implemented

- `DdFilterScaffold::processFloatUpdate()` now carries a persistent DD state
  across epochs, seeded from the native CLAS float position on first use.
- The active state layout follows the A0 CLASLIB audit: position/dynamics,
  per-satellite ionosphere `II`, ZTD, optional GLONASS bias slots, and
  cycle-valued per-satellite/per-frequency ambiguity `IB` states. Receiver
  clocks remain outside the active DD update because phase and code rows are
  double-differenced before filtering.
- The time update mirrors the CLASLIB `udstate_ppp()` split for the A1 static
  CLAS path: position process noise, ZTD process noise, adaptive ionosphere
  process noise, ambiguity random walk, outage resets, LLI resets, atmosphere
  network ionosphere reset, and phase-code ambiguity initialization.
- `buildDdMeasurementSystem()` builds CLASLIB-style `ddres()` rows from the
  existing lifecycle atmosphere and OSR products: highest-elevation reference
  selection per system/frequency/kind, full `PRC/CPC` zero-difference residuals,
  `ref - sat` phase and code rows, ionosphere/troposphere/ambiguity columns,
  and DD covariance blocks.
- The DD measurement update uses the shared RTK measurement/update helpers and
  publishes the DD float position and covariance. Integer fixing remains out of
  scope for A1.
- For the static CLAS A1 gate, the first native float position is retained as a
  tight static anchor pseudo-measurement inside the gated DD update. This keeps
  the incomplete A1 float datum from drifting while phase/code DD rows, iono,
  trop, and ambiguity states are still estimated by the DD filter.
- Candidate DD updates are postfit-checked before commit. If the DD postfit rows
  exceed the residual gate, or the static candidate moves more than 8 m from the
  seeded anchor, the scaffold reseeds once from the same-epoch native float and
  retries the DD update. A failed retry publishes the native float for that
  epoch with `iterations=0`; successful DD updates publish with `iterations=1`.

## A2 Entry Points

- Use `DdFilterScaffold::snapshot()` after a successful A1 update as the source
  of the float DD state/covariance for LAMBDA.
- Build the ambiguity candidate vector from `StateLayout::ambiguityIndex()` for
  the same reference-satellite groups selected by `buildDdMeasurementSystem()`.
- Reuse `rtk_measurement::buildAmbiguityTransform()` or an equivalent
  DD-native transform to extract `Qb` and `Qab` from the DD covariance; do not
  consume the old native PPP ambiguity map.
- Publish the conditioned `xa/Pa` through the DD scaffold, then let
  `ppp_clas_epoch.cpp` choose the fixed DD snapshot under
  `GNSS_PPP_CLAS_DD_FILTER=1`.
- Revisit or remove the A1 static anchor once LAMBDA conditioning and hold
  feedback stabilize the DD ambiguity datum; A2 should measure fixed-state
  performance both with and without the anchor before tightening the default.
