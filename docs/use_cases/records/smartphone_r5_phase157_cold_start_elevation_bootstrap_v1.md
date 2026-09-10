# Smartphone R5 Phase157 cold-start/elevation diagnosis

Status: implemented as an opt-in preparatory-path change.  This record is
source- and sealed-metadata-only; Phase157 did not read or rerun a real raw
route, truth, MAT, Kaggle, or solver payload.

## Evidence and diagnosis

The sealed Phase156 result `8f37d22567d63c5f5cb8853e2b4799ccdf1f6d78`
reported, for its only evaluated H epoch, 33 source rows: 2 accepted, 20
`below-elevation-mask`, and 11 `unsupported-signal`.  That count establishes
the observed pre-solve filtering outcome, but does not by itself attribute all
20 elevation rejects to initialization.

The source ordering proves an initialization exposure:

* `apps/native/gnss_fgo_imu_no_base.cpp:9632-9637` assigns Android epochs the
  fixed ECEF point `(6378137, 0, 0)`.
* `src/algorithms/spp.cpp:575-579` copies that point only when the internal
  position norm is below 1000; its norm therefore prevents that assignment
  from being a native cold start.
* `src/algorithms/spp.cpp:744-746` calls `initializePosition` only when the
  internal position is still below 1000.
* `src/algorithms/spp.cpp:801-815` computes the configured elevation mask
  from the current position, and the measurement loop applies it before the
  least-squares result is available.
* `src/algorithms/spp.cpp:2151-2161` is the existing native cold-start
  initializer: it derives a rough ECEF point from same-epoch satellite states.

Thus the fixed Android point can drive receiver-dependent elevation filtering
before a solved route receiver position exists.  The 20-row Phase156 result is
consistent with that condition, while the sealed metadata does not support a
stronger claim.  The 11 signal rejects cannot be assigned to individual bands
from the sealed result: source policy in
`include/libgnss++/core/signal_policy.hpp:110-148` accepts the primary GPS L1,
Galileo E1, and other constellation primary signals by default, while
secondary-band signals are admitted only for the ionosphere-free path.

## Bounded implementation

`raw_p_seed::Config::bootstrap_position_before_elevation` defaults to `false`.
When explicitly enabled, the first epoch is copied through the existing
raw-P-only path (Doppler fields cleared), forced to a zero receiver seed, and
solved once by a private native SPP instance with only its elevation gate set
to -90 degrees.  The private result is accepted only when native SPP reports a
valid solution with at least four satellites, finite ECEF/clock values, and a
finite ECEF norm above 1e6.  That same-run position is then supplied to the
ordinary SPP pass, whose caller-configured elevation mask is unchanged.  No
coordinate is imported from disk and no mask is weakened for the ordinary
pass.  Bootstrap failure is reported as
`native-raw-p-bootstrap-rejected`.

The app exposes this only as
`--native-phase157-raw-p-bootstrap`, and argument parsing requires the
existing `--native-phase149-raw-p-seed-stage`; ordinary recipes are unchanged.

## Synthetic acceptance coverage

`RawPSeedTest.*` passes 19/19, including stationary/moving raw-P regressions,
Doppler clearing, rank and clock-group checks, an unknown-origin bootstrap
with the ordinary zero-degree mask, a rotated multi-clock geometry, default-off
behavior, and an insufficient-visible-row rejection proving that the private
no-mask pass does not bypass the configured ordinary mask.  The affected
`gnss_fgo_imu_no_base` executable also builds successfully.

The tests use physically generated synthetic pseudoranges and native synthetic
ephemerides only.  This change does not remove the H no-Doppler FGO guards,
does not claim route accuracy, and does not establish that a real route has
enough post-bootstrap geometry; that requires a separately authorized raw
diagnostic execution.
