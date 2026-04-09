# CLAS Debug Tag Playbook

This page is the operator/developer playbook for the current CLAS parity debug
surface.

The goal is simple:

1. pick the right stderr tag,
2. decide which boundary is broken,
3. avoid random solver tuning before the boundary is identified.

## Debugging order

Use the tags in this order:

1. transport / sampled-row sanity
2. OSR selection
3. CLAS measurement construction
4. ambiguity update
5. DD/WLNL comparison
6. fixed-solution validation

That order follows the actual data path.

## Tag map

| Tag | Boundary | Use it when you need to answer |
| --- | --- | --- |
| `[SSR-BIAS-SEL]` | network and bias-source selection | which network / bias bank was selected |
| `[OSR-SSR]` | OSR-facing correction composition | what orbit/clock/bias counts are reaching OSR |
| `[OSR]` | final per-satellite OSR dump | whether `trop/iono/cbias/pbias/clk` numerically match expectation |
| `[CLAS-PHASE-ROW]` | CLAS measurement model | what corrected carrier row is entering the filter |
| `[CLAS-IF-PHASE]` | IF residual reconstruction | whether observation-side IF and ambiguity-side IF agree |
| `[CLAS-AMB-SEED]` | ambiguity initialization | what datum ambiguity states start from |
| `[CLAS-AMB-OBS]` | ambiguity bookkeeping after update | how ambiguity state moves epoch to epoch |
| `[CLAS-K-AMB]` / `[CLAS-K-AMB-SUM]` | Kalman cross-coupling into ambiguity states | which rows are pushing ambiguity during the update |
| `[PPP-WLNL-COMP]` | WLNL observation/state comparison | whether DD gap comes from observation-side vs state-side NL mismatch |
| `[PPP-WLNL-FIXSTATE]` | fixed-state projection | whether fixed ambiguity projection is reaching state space |
| `[CLAS-WLNL-FIX]` | CLAS fixed candidate solve | whether post-fix solve is numerically sane |
| `[CLAS-PPP]` | epoch summary | whether code/phase RMS and position delta are healthy |

## Common questions and the right tag

### 1. Which atmosphere/bias network did this satellite use?

Start with:

- `[SSR-BIAS-SEL]`
- `[OSR]`

If those already disagree with expectation, do not inspect AR yet.

### 2. Does OSR itself match the expected satellite numbers?

Start with:

- `[OSR-SSR]`
- `[OSR]`

Use this before touching PPP internals.

### 3. Did ambiguity start from the wrong datum?

Start with:

- `[CLAS-AMB-SEED]`
- `[CLAS-PHASE-ROW]`

If seed-time residual is already nonzero, the measurement/seed formula is wrong.

### 4. Did ambiguity move because of phase rows or because of code/prior coupling?

Start with:

- `[CLAS-AMB-OBS]`
- `[CLAS-K-AMB]`
- `[CLAS-K-AMB-SUM]`

Interpretation:

- if `h_amb=0` rows still move ambiguity, the coupling is indirect through the
  Kalman solve
- if direct phase rows dominate, the phase model/datum is the first suspect

### 5. Is the DD gap observation-side or state-side?

Start with:

- `[PPP-WLNL-COMP]`
- `[CLAS-IF-PHASE]`

Interpretation:

- if `obs_total - state_total` has a large common offset on all satellites,
  the problem is datum definition
- if only one or two satellites diverge from that common offset, focus on
  satellite-specific ambiguity maintenance

### 6. Did AR succeed but fixed position still go bad?

Start with:

- `[PPP-WLNL-FIXSTATE]`
- `[CLAS-WLNL-FIX]`
- `[CLAS-PPP]`

That separates:

- AR success
- fixed-state projection
- final position generation

## Recommended triage patterns

### Pattern A: OSR mismatch

Read:

1. `[SSR-BIAS-SEL]`
2. `[OSR-SSR]`
3. `[OSR]`

Likely code areas:

- `ppp_osr.cpp`
- `ppp_atmosphere.cpp`
- `gnss_qzss_l6_info.py`
- `qzss_l6.cpp`

### Pattern B: ambiguity datum mismatch

Read:

1. `[CLAS-AMB-SEED]`
2. `[CLAS-AMB-OBS]`
3. `[PPP-WLNL-COMP]`

Likely code areas:

- `ppp_clas.cpp`
- `ppp_ar.cpp`
- `ppp_wlnl.cpp`

### Pattern C: fixed but inaccurate

Read:

1. `[PPP-WLNL-FIXSTATE]`
2. `[CLAS-WLNL-FIX]`
3. `[CLAS-PPP]`

Likely code areas:

- `ppp_ar.cpp`
- `ppp_clas_epoch.cpp`
- `ppp_clas.cpp`

## Current practical rule

Do not start with gate tuning.

The current CLAS parity work already showed multiple times that:

- gate tuning can hide the real boundary,
- fixed-rate parity can improve while accuracy stays wrong,
- the correct next step usually comes from one stderr tag family, not from a
  broad parameter sweep.

## Related pages

- [CLAS API & Flow](clas.md)
- [CLAS Parity Datasets & Artifacts](clas_parity_artifacts.md)
- [CLAS Parity Blockers](clas_parity_blockers.md)
