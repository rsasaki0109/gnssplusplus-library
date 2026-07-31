# CLAS DD PPP-RTK Filter A0 Audit

This note audits the CLASLIB DD PPP-RTK filter that Option A will port behind
`GNSS_PPP_CLAS_DD_FILTER`, and maps it to the native CLAS PPP path in
libgnss++. A0 only adds the gated scaffold; A1 is the DD measurement-update
slice.

## CLASLIB State Layout

CLASLIB's CLAS PPP-RTK path lives in `/tmp/claslib/src/ppprtk.c` and uses the
RTKLIB/CLAS state index macros from `/tmp/claslib/src/cssr2osr.h`.

The relevant macros are:

- `NP(opt) = dynamics ? 9 : 3`, position-only for static mode and
  position/velocity/acceleration for dynamics
  (`/tmp/claslib/src/cssr2osr.h:14`).
- `NF(opt) = opt->nf` (`/tmp/claslib/src/cssr2osr.h:15`).
- `IC(s,opt) = NP(opt) + s` and `IT(opt) = IC(0,opt) + NSYS`, inherited
  receiver-clock/trop macros from the PPP family
  (`/tmp/claslib/src/cssr2osr.h:16` and
  `/tmp/claslib/src/cssr2osr.h:17`).
- `NI(opt) = MAXSAT` only for `IONOOPT_EST` or `IONOOPT_EST_ADPT`;
  otherwise zero (`/tmp/claslib/src/cssr2osr.h:18`).
- `NT(opt) = 0/1/3` for no trop, ZTD, or ZTD+gradients
  (`/tmp/claslib/src/cssr2osr.h:19`).
- `NL(opt) = NFREQGLO` only for GLONASS autocal mode
  (`/tmp/claslib/src/cssr2osr.h:20`).
- `NR(opt) = NP + NI + NT + NL`, and `NX(opt) = NR + MAXSAT * NF`
  (`/tmp/claslib/src/cssr2osr.h:21-23`).
- `II(s,opt) = NP + s - 1` for per-satellite ionosphere, 1-based satno
  (`/tmp/claslib/src/cssr2osr.h:24`).
- `ITT(opt) = NP + NI` for the active troposphere block
  (`/tmp/claslib/src/cssr2osr.h:25`).
- `IB(s,f,opt) = NR + MAXSAT * f + s - 1` for per-satellite,
  per-frequency phase-bias/ambiguity states
  (`/tmp/claslib/src/cssr2osr.h:26`).

Audit finding: although `IC()`/`IT()` are defined in the shared header,
`ppprtk.c`'s active `NX()` layout does not include receiver-clock states because
the CLAS measurement update is double-differenced and receiver clocks cancel.
The active non-ambiguity block is `position/dynamics + II + ITT + optional NL`;
`ppprtk.c` traces `x(0)` through `NR(opt)` before the ambiguity block
(`/tmp/claslib/src/ppprtk.c:1558`), and `ppp_rtk_nx()` returns `NX(opt)`
(`/tmp/claslib/src/ppprtk.c:1360-1364`).

## CLASLIB State Time Update

`initx()` sets one state value and zeros all covariance cross terms for that
state (`/tmp/claslib/src/ppprtk.c:181-187`).

Position and dynamics:

- Fixed mode pins position to `opt.ru` with tiny variance
  (`/tmp/claslib/src/ppprtk.c:201-204`).
- First epoch initializes position from `rtk->sol.rr` with `VAR_POS`; dynamic
  mode also initializes velocity and acceleration
  (`/tmp/claslib/src/ppprtk.c:207-214`).
- Large mean position variance resets position/velocity/acceleration
  (`/tmp/claslib/src/ppprtk.c:217-228`).
- Dynamic mode propagates position/velocity/acceleration with an `F` matrix
  (`/tmp/claslib/src/ppprtk.c:230-295`).
- Dynamic process noise uses `prn[3]` and `prn[4]` for acceleration
  (`/tmp/claslib/src/ppprtk.c:297-304`).
- Static/non-dynamic position process noise uses `prn[5]` and `prn[6]`;
  if `prnadpt` is enabled, the accumulated adaptive `rtk->Q` block is used
  instead (`/tmp/claslib/src/ppprtk.c:305-318`).

Troposphere:

- `udtrop()` initializes `ITT()` to `INIT_ZWD` with `std[2]^2`; gradients, when
  enabled, initialize to `1E-6` with `VAR_GRA`
  (`/tmp/claslib/src/ppprtk.c:323-338`).
- Existing ZTD gets `prn[2]^2 * tt`; gradients get
  `(0.3 * prn[2])^2 * |tt|` (`/tmp/claslib/src/ppprtk.c:339-345`).
- When CLAS trop SSR is valid at first reset, `ppp_rtk_pos()` can zero the ZTD
  state variance/process noise (`/tmp/claslib/src/ppprtk.c:1594-1601`).

Ionosphere:

- `udion()` resets `II(s)` to zero if an observed satellite exceeds the outage
  counter (`/tmp/claslib/src/ppprtk.c:357-371`).
- New ionosphere states initialize to `1E-6` with `std[1]^2`
  (`/tmp/claslib/src/ppprtk.c:374-379`).
- Normal iono process noise is `prn[1]^2 * cos(el)^2`.
  `IONOOPT_EST_ADPT` keeps an adaptive per-state `rtk->Q`, clamped between
  `prn[1]^2` and `prn[7]^2` before adding `qi * tt` to covariance
  (`/tmp/claslib/src/ppprtk.c:381-401`).
- After the float update, `IONOOPT_EST_ADPT` updates each iono process-noise
  entry from the posterior covariance using `forgetion` and `afgainion`
  (`/tmp/claslib/src/ppprtk.c:1653-1659`).
- A CLAS grid/network change zeros every `II()` state
  (`/tmp/claslib/src/ppprtk.c:1506-1511`).

Ambiguities / phase bias:

- `udbias_ppp()` clears slip/reset flags, detects LLI and geometry-free slips,
  then increments outage counters (`/tmp/claslib/src/ppprtk.c:470-489`).
- Instantaneous AR or max-outage resets an `IB(s,f)` state to zero and resets
  lock count (`/tmp/claslib/src/ppprtk.c:491-509`).
- Each observed ambiguity gets `prn[0]^2 * tt` process noise, and slip resets
  the state to zero (`/tmp/claslib/src/ppprtk.c:511-520`).
- Initial ambiguity value is `(phase_m - code_m - common_bias) / wavelength`,
  so the `IB()` state is in cycles; `ddres()` later multiplies it by wavelength
  (`/tmp/claslib/src/ppprtk.c:524-548` and
  `/tmp/claslib/src/ppprtk.c:985-991`).

Top-level reset semantics:

- Time gaps beyond `maxobsloss` zero every state
  (`/tmp/claslib/src/ppprtk.c:1471-1475`).
- Periodic reset zeros every state, clears CSSR state, and returns a single
  solution (`/tmp/claslib/src/ppprtk.c:1477-1489`).
- `nav->filreset` zeros all non-position states in static mode, or all states
  in dynamics mode (`/tmp/claslib/src/ppprtk.c:1493-1503`).
- `udstate_ppp()` applies position, trop, ambiguity, then ionosphere time
  updates in that order (`/tmp/claslib/src/ppprtk.c:557-578`).

## CLASLIB DD Residual Model

`ddres()` is the core CLAS DD measurement builder
(`/tmp/claslib/src/ppprtk.c:794-1032`).

Reference-satellite selection:

- Rows are grouped by system group `m` and by `f` over phase and code
  (`/tmp/claslib/src/ppprtk.c:832-847`).
- For each group, the reference satellite is the valid satellite with highest
  elevation; stale SSR channels and QZSS mode rules are filtered before
  selection (`/tmp/claslib/src/ppprtk.c:847-880`).
- The selected reference is remembered in `refsat`/`refsat2`; reference changes
  are logged and can trigger phase-bias reinitialization in merged L6 channel
  mode (`/tmp/claslib/src/ppprtk.c:884-946`).

Row formation:

- Each row is a single/double difference of zero-difference residuals:
  reference minus target satellite (`/tmp/claslib/src/ppprtk.c:954-959`).
- Position partials are `-e_ref + e_sat`
  (`/tmp/claslib/src/ppprtk.c:961-963`).
- Ionosphere contributes opposite signs for phase/code and scales by
  `(lambda_f/lambda_L1)^2 * ionmapf`; `II(ref)` and `II(sat)` get opposite
  columns (`/tmp/claslib/src/ppprtk.c:965-975`).
- Troposphere subtracts mapped reference-target trop and writes derivatives
  into `ITT()+k` (`/tmp/claslib/src/ppprtk.c:977-984`).
- Phase rows subtract `lambda_ref * IB(ref,f) - lambda_sat * IB(sat,f)` and
  write the same signed wavelengths into `H`
  (`/tmp/claslib/src/ppprtk.c:985-991`).
- Code rows have no ambiguity column.
- Phase and code variances are elevation-weighted through `varerr()`; L2 phase
  gets an extra `(2.55/1.55)^2` factor (`/tmp/claslib/src/ppprtk.c:997-1008`).
- `ddcov()` combines per-satellite `Ri`/`Rj` terms into the DD covariance
  (`/tmp/claslib/src/ppprtk.c:1026-1027`).

`varerr()` itself switches between extended code/phase error models and the
normal RTKLIB model, adds iono/trop error terms when those states are not
estimated, and weights by `1/sin(el)^2`
(`/tmp/claslib/src/ppprtk.c:130-178`).

Filter and fixed-state publication:

- `ppp_rtk_pos()` builds zero-difference OSR residuals, calls `ddres()` for the
  DD design matrix, runs `filter2()`, re-evaluates postfit DD rows, and accepts
  the float update when chi-square passes (`/tmp/claslib/src/ppprtk.c:1575-1640`).
- On success it commits `xp/Pp` back to `rtk->x/P`
  (`/tmp/claslib/src/ppprtk.c:1670-1675`).
- `resamb_LAMBDA()` builds a single-to-DD ambiguity transform, extracts `Qb`
  and `Qab`, runs LAMBDA, conditions `xa/Pa`, and restores single-difference
  ambiguity states (`/tmp/claslib/src/ppprtk.c:1166-1265`).
- Fixed postfit residuals call `ddres()` again on `xa`; accepted fixed states
  become `SOLQ_FIX`, and hold feedback can apply ambiguity constraints
  (`/tmp/claslib/src/ppprtk.c:1702-1768`).

## Native CLAS Path

Native PPP state:

- `PPPState` stores position, velocity, receiver clocks, ZTD, optional
  ionosphere states, and dynamically appended ambiguity maps
  (`include/libgnss++/algorithms/ppp_shared.hpp:282-303`).
- Standard PPP initialization reserves receiver clock states first, then
  optional ionosphere states, then ambiguity states
  (`src/algorithms/ppp_state.cpp:300-389`).
- Standard prediction applies process noise to position/velocity, receiver
  clocks, ISBs, ZTD, ambiguities, and ionosphere maps
  (`src/algorithms/ppp_state.cpp:399-467`).

Native CLAS float filter:

- `processEpochCLAS()` drives CLAS epochs: SPP seed, cycle slips, CLAS
  `prepareEpochState()`, OSR context construction, ambiguity-state allocation,
  native measurement update, then optional WLNL AR
  (`src/algorithms/ppp_clas_epoch.cpp:177-486`).
- `prepareEpochState()` initializes and predicts the CLAS filter state, using
  residual ionosphere satellites collected from observations/SSR products
  (`src/algorithms/ppp_clas.cpp:459-517`).
- CLAS initialization uses `[pos, GPS clock, GLO clock, ZTD, iono...]`, not the
  CLASLIB fixed `II/IB` layout (`src/algorithms/ppp_clas.cpp:519-548`).
- Native CLAS prediction uses white receiver-clock variance, ZTD process noise,
  optional iono process noise, ambiguity process noise, and clock/position
  decoupling (`src/algorithms/ppp_clas.cpp:604-647`).
- OSR-corrected code/phase rows are built as undifferenced rows with receiver
  clock columns, ZTD columns, iono columns, and ambiguity columns in meters
  (`src/algorithms/ppp_clas.cpp:746-1005`).
- Native CLAS adds trop and STEC pseudo-observation constraints
  (`src/algorithms/ppp_clas.cpp:1008-1062`).
- Carrier phase rows are single-differenced by system/signal by default, but
  code rows stay undifferenced unless a parity gate requests code SD
  (`src/algorithms/ppp_clas.cpp:1064-1140`).
- The Kalman update is a normal linear update on native rows
  (`src/algorithms/ppp_clas.cpp:1145-1205`).

Current native AR:

- CLAS per-frequency mode forces `DD_WLNL` in `processEpochCLAS()`
  (`src/algorithms/ppp_clas_epoch.cpp:187-195`).
- `resolveAmbiguitiesWLNL()` fixes WL integers from MW averages, then calls
  `ppp_ar::resolveWlnlFix()` using native ambiguity states and OSR-derived NL
  information (`src/algorithms/ppp_wlnl.cpp:113-212`).
- `ppp_ar::tryDirectDdFix()` forms DD ambiguity pairs from the native ambiguity
  map, builds DD covariance from native covariance, runs LAMBDA, and conditions
  the native head state (`src/algorithms/ppp_ar.cpp:155-350`).
- The current fixed-position WLS is post-hoc: `solveFixedPosition()` builds
  fixed NL observations after the float filter state exists
  (`src/algorithms/ppp_wlnl.cpp:214-240`).

Architectural gap:

- Native CLAS is an undifferenced/partly single-differenced float PPP filter
  with receiver-clock states, meter-valued ambiguity states, and post-hoc
  DD-WLNL AR.
- CLASLIB `ppprtk.c` is a DD PPP-RTK filter: phase and code rows are DD before
  the Kalman update, receiver-clock states cancel, `II()` and cycle-valued
  `IB()` have fixed RTKLIB satno-indexed slots, and LAMBDA publishes a
  constrained `xa` from that same DD covariance.
- Therefore the A1 port should not only adjust AR. It must build the DD design
  matrix and covariance first, then make AR consume that DD-native ambiguity
  covariance.

## A1 Entry Points

1. `src/algorithms/ppp_clas_epoch.cpp`: the A0 hook returns immediately after
   the native float update when `ppp_config_.use_clas_dd_filter` is true. A1
   should replace this passthrough return with a DD prediction/update call fed
   by `epoch_context`, `osr_corrections`, and `epoch_atmos`.
2. `include/libgnss++/algorithms/ppp_clas_dd.hpp` and
   `src/algorithms/ppp_clas_dd.cpp`: extend `DdFilterScaffold` from a
   typed layout/snapshot into a persistent DD filter with CLASLIB-style
   `udpos/udtrop/udion/udbias` equivalents.
3. `src/algorithms/ppp_clas_dd.cpp`: add an A1 DD row builder matching
   `ddres()` reference selection, phase+code differencing, `varerr()` weights,
   and `II/ITT/IB` columns.
4. `src/algorithms/ppp_atmosphere.cpp` / `prepareClasEpochContext()` wiring:
   consume lifecycle atmosphere tokens already exposed as `epoch_atmos`; this is
   where A1 should source the CLAS STEC/trop inputs instead of adding another
   correction path.
5. Keep `src/algorithms/ppp_wlnl.cpp` and `src/algorithms/ppp_ar.cpp` unchanged
   until A2, when LAMBDA should be moved to the DD filter covariance rather than
   the old native ambiguity datum.
