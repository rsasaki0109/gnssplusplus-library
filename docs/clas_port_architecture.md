# CLAS Port Architecture

Status: iter52 default-promotion note.

This note records the CLASLIB port structure after the iter52 promotion of the
native CLASNAT path to the default `--claslib-parity` behavior.  The intent is
to keep the production path, bridge/oracle path, and retained legacy strict path
explicit while cleanup continues.

## 1. Current Architecture

The current CLAS PPP-RTK stack has five user-visible or historically relevant
paths:

```text
gnss_ppp
  |
  +-- --claslib-bridge
  |     |
  |     +-- external CLASLIB postpos()
  |           Native libgnss++ state is bypassed.
  |           This path remains the oracle/reference integration.
  |
  +-- --claslib-parity
  |     |
  |     +-- native CLASNAT path by default
  |           |
  |           +-- ppp_clasnat wrapper
  |                 |
  |                 +-- ppp_claslib_full core
  |                       |
  |                       +-- ppp_clasnat_osr
  |                       +-- ppp_clasnat_zdres
  |                       +-- ppp_clasnat_trop
  |                       +-- clasnat_parity helpers
  |
  +-- --ported-clasnat
  |     |
  |     +-- explicit alias for the same native CLASNAT path
  |           |
  |           +-- ppp_claslib_full core
  |                 |
  |                 +-- ppp_clasnat_osr
  |                 +-- ppp_clasnat_zdres
  |                 +-- ppp_clasnat_trop
  |                 +-- clasnat_parity helpers
  |
  +-- --ported-full
  |     |
  |     +-- ppp_claslib_full core directly
  |           Legacy regression arm from the Phase 4 unification work.
  |
  +-- --claslib-parity --legacy-strict-parity
        |
        +-- older native CLAS OSR filter path
              Useful as history and for selected debug fixtures, but no
              longer the target architecture for CLASNAT parity.
```

The important practical difference after iter52 is that `--claslib-parity`
now enters the same native CLASNAT wrapper that was previously selected only
with `--ported-clasnat`.  `--ported-clasnat` remains valid but redundant for
explicit scripts.  `--no-ported-clasnat` and `--legacy-strict-parity` reserve
the older strict CLAS OSR path for regression/reference work.  `--ported-full`
still skips the wrapper and calls the same core through a different state member
as a deprecated compatibility alias.

Bridge-related code is separate:

```text
claslib_bridge     -> external postpos() runner
claslib_oracle     -> direct CLASLIB function oracle for unit parity tests
clasnat_parity     -> native implementations tested against claslib_oracle
```

`claslib_oracle` and `claslib_bridge` both depend on linked CLASLIB sources
when `CLASLIB_PARITY_LINK=ON`, but they serve different purposes.  The bridge
is an executable path for whole-run comparison.  The oracle is a narrow
function-level reference for native helper tests.

## 2. Native Port Completion Status

The native CLASNAT path has reached the current numeric target and is now the
default for `--claslib-parity`:

- 7 `ClasnatParity` helper tests pass against the oracle.
- 2000-epoch 2019 CLAS static run stays at the fixed-only 3D RMS target
  established in iter50, approximately 5.32 mm.
- The core filter owns CLASLIB-shaped state with 303 states:
  position, per-satellite ionosphere, and per-frequency ambiguities.
- State management includes:
  position seeding and prediction,
  ionosphere and ambiguity maps,
  per-satellite continuity bookkeeping,
  phase wind-up cache,
  CLAS dispersion compensation state,
  SIS continuity state,
  phase-bias repair state,
  last epoch time, AR ratio, and fixed ambiguity counters.
- Filter update is native:
  CLAS OSR preparation,
  CLASLIB-style zero-difference residual construction,
  state propagation,
  measurement update,
  residual diagnostics,
  covariance propagation,
  and solution packaging are all in libgnss++.
- AR hold/fixed output is native:
  the core constructs the per-frequency DD AR state, applies fixed ambiguity
  results back into the CLASNAT state, and emits fixed solutions through the
  same `PositionSolution` surface as the rest of PPP.
- Helper modules already separated from the core:
  `ppp_clasnat_osr` for CLAS OSR preparation,
  `ppp_clasnat_zdres` for CLASLIB-shaped residual construction,
  `ppp_clasnat_trop` for CLAS grid/troposphere logic,
  `clasnat_parity` for small native CLASLIB-equivalent functions,
  and `claslib_oracle` for direct oracle calls.

## 3. Remaining Historical Technical Debt

The remaining debt is mostly structural rather than numeric:

- The successful CLASNAT path is named through historical layers:
  `ppp_clasnat` wrapper -> `ppp_claslib_full` core.  The word `full` now
  describes the old Phase 4 experiment, not the actual architecture.
- `PPPProcessor` holds two native states for one implementation:
  one wrapper state for `--ported-clasnat`, and one core state for
  `--ported-full`.
- `--ported-full` exists as a regression arm even though the maintained native
  target is now `--claslib-parity` entering the CLASNAT core.
- Naming mixes CLASLIB-origin terms and CLASNAT terms:
  `ClaslibRtkState`, `ppp_claslib_full`, `fullConfig`, and debug labels sit
  next to `ppp_clasnat_osr`, `ppp_clasnat_trop`, and `clasnat_parity`.
- Some source boundaries still reflect the order of discovery:
  the native residual builder, atmospheric helpers, parity helpers, and core
  filter are clean enough to test, but their names do not yet communicate the
  intended ownership model.
- `claslib_oracle` is correctly isolated, but the older naming can make it
  look like production native code still depends on CLASLIB.  It should remain
  a test/oracle facility only.

## 4. Target Architecture

The target is a single native CLASNAT path:

```text
gnss_ppp --claslib-parity
  |
  +-- ppp_clasnat_core
        |
        +-- ppp_clasnat_osr
        |     CLAS SSR/OSR epoch context and correction preparation.
        |
        +-- ppp_clasnat_zdres
        |     CLASLIB-style zero-difference residual transcription.
        |
        +-- ppp_clasnat_trop
        |     CLAS grid and troposphere support.
        |
        +-- clasnat_parity
              Small native CLASLIB-equivalent math/helpers.

optional references:
  --ported-clasnat -> explicit native CLASNAT selection, redundant with --claslib-parity
  --legacy-strict-parity / --no-ported-clasnat -> older strict CLAS OSR path
  --claslib-bridge -> external CLASLIB whole-run oracle
  claslib_oracle   -> direct function oracle for tests
```

Rules for the target structure:

- There is one production native core: `ppp_clasnat_core`.
- `--claslib-parity` is the canonical user-facing switch for that core.
- `--ported-clasnat` remains accepted as an explicit, redundant native path
  selector for existing scripts.
- `--legacy-strict-parity` and `--no-ported-clasnat` retain the iter13-era
  strict path for archive/regression reference only.
- `--ported-full` remains only as a deprecated alias while old scripts are
  migrated.
- The core owns the CLASNAT filter state directly as `ClasnatRtkState`.
- The bridge is optional and does not participate in native state or runtime
  decisions unless explicitly selected.
- `clasnat_parity` stays small and pure: native helpers plus test fixtures.
- `claslib_oracle` stays external-reference only and should not become a
  production dependency.

## 5. iter52+ Roadmap

Recommended next phases:

1. Finish name cleanup inside the core.
   Rename remaining internal `full` debug labels and helper names where doing
   so does not disturb numeric behavior.  Keep this mechanical and covered by
   CLASNAT regression checks.

2. Collapse legacy flags further.
   Keep `PPPConfig::use_ported_full` as a compatibility bit for one or two
   iterations, then remove internal branches once scripts no longer depend on
   it.  The CLI can keep a deprecated alias longer if needed.  Keep
   `--legacy-strict-parity` while the older strict path is still valuable as a
   regression reference.

3. Expand `ClasnatParity` coverage.
   Low-risk additions are broadcast ephemeris helpers (`eph2pos`, `eph2clk`),
   `geodist` including Sagnac correction, `tropmodel`, and extra `ionmapf`
   fixtures.  Add one helper at a time and keep oracle-backed tolerances tight.

4. Document production vs oracle boundaries in CI.
   CI should continue to build the native path without CLASLIB as production
   code.  CLASLIB-linked parity/oracle jobs should remain explicit.

5. Revisit long CLAS PPP regression cost.
   The 3600-epoch expanded-CSV regression can exceed practical test time.
   Split it into a short deterministic CI regression and a longer benchmark,
   or optimize the CLAS SSR lookup path before making it a required CI gate.

6. Remove obsolete Phase 4 language from user-facing docs.
   Once the code no longer exposes the old wrapper/core split, update README
   and scripts so new users see `--claslib-parity` as the production native
   entrypoint, with bridge/oracle options documented separately.
