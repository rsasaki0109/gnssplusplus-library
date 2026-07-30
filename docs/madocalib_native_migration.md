# MADOCALIB Native Migration Ledger

This ledger is the active execution plan for issue #148.  The older
`madoca_port_plan.md` remains the detailed implementation history; when the two
documents differ, this ledger describes the current migration state and exit
criteria.

## Target

Move the supported MADOCALIB behavior into native libgnss++ C++ while retaining
MADOCALIB only as an opt-in test oracle.  A normal configure, build, install,
and runtime must not require a MADOCALIB checkout.

This is behavioral migration, not source-tree vendoring.  Native code keeps the
libgnss++ state, product-container, CLI, and JSON sign-off architecture.  Small
literal helper ports are allowed only with deliberate attribution and license
review.

## Frozen Oracle Contract

| Item | Frozen value |
|---|---|
| Upstream | `QZSS-Strategy-Office/madocalib` |
| Commit | `0089f7dc97e8e2ba283a40be2edf4b73a140df6c` |
| Reported version | MADOCALIB 2.1 |
| Opt-in configure | `-DMADOCALIB_PARITY_LINK=ON -DMADOCALIB_ROOT_DIR=<checkout>` |
| Default configure | `MADOCALIB_PARITY_LINK=OFF` |
| Primary observation | `sample_data/data/rinex/MIZU00JPN_R_20250910000_01D_30S_MO.rnx` |
| Primary navigation | `sample_data/data/rinex/BRDM00DLR_S_20250910000_01D_MN.rnx` |
| L6E channels | `sample_data/data/l6_is-qzss-mdc-004/2025/091/2025091A.{204,206}.l6` |
| L6D channels | `sample_data/data/l6/2025/091/2025091A.{200,201}.l6` |
| Antenna model | `sample_data/data/igs20.atx` |
| Primary window | 2025-04-01 00:00:00--00:59:30, 30 s interval |
| Reference ECEF | `-3857167.6484 3108694.9138 4004041.6876` m |

The `madoca-parity` GitHub Actions job is authoritative for oracle-linked
tests.  It checks out the exact commit above, builds `run_tests`, runs the
helper/decoder parity suite, generates the MADOCALIB and native PPP-AR
trajectories, and uploads their summaries and epoch matches.

The public materialization self-diff runner uses the same repository and commit.
Changing either pin is a dedicated PR: record the upstream version, regenerate
all oracle artifacts, and explain every behavior change before updating this
table.

## License Boundary

The upstream `readme.txt` identifies MADOCALIB as BSD 2-Clause with additional
clauses and copyright notices for the Cabinet Office, Japan; Lighthouse
Technology & Consulting; JAXA; Toshiba Electronic Technologies; and T. Takasu.
Source and binary redistributions must retain or reproduce the upstream notice,
conditions, and disclaimer.  Companion binaries retain their original
licenses.  `MADOCALIB_GUI_LICENSE.txt` is a separate GUI license and the GUI is
outside this migration.

Therefore:

- `external/madocalib` is an external checkout, never a production source
  directory or installed runtime dependency;
- oracle wrappers remain guarded by `GNSSPP_HAS_MADOCALIB_ORACLE` or
  `GNSSPP_HAS_MADOCALIB_BRIDGE`;
- the default build must compile and test with both macros false;
- direct upstream linkage is limited to the opt-in parity build and its
  artifacts; and
- any literal native port must preserve the required attribution in the source
  and distribution documentation.

## Capability Ledger

Status meanings: **native** is implemented without MADOCALIB; **oracle-only**
exists only in the opt-in linked lane; **partial** has a native foundation but
does not yet match end-to-end behavior; **open** has no accepted native parity.

| MADOCALIB surface | Native implementation/evidence | Status | Exit evidence |
|---|---|---|---|
| System, signal, update-interval, URA, and bias-code helpers | `madoca_core`, `madoca_parity`, `MadocaParity` tests | native | Keep deterministic helper parity at exact integer or `1e-6` scaled tolerance |
| L6E byte sync and subframe assembly | `MadocaL6eDecoder` | native | Byte-stepped oracle test remains green |
| L6E mask/orbit/clock/code-bias/phase-bias/URA | `madoca_l6`, `SSRProducts` materialization | native | No missing keys; supported numeric fields match the pinned oracle |
| Multi-file L6E replay and correction ordering | `decodeMadocaL6eFilesToProductsReplay`, correction contract | native | Materialization key, time, IOD, identity, and value diffs are zero for the pinned fixture |
| L6D coverage/correction decode | `MadocaL6dDecoder` | native | Byte-stepped region/area parity remains green |
| L6D receiver-area selection and delay/std | `MadocaIonoStore` | native | Selected receiver correction values match the oracle |
| L6D causal product snapshots | `MadocaIonoSnapshot`, `MadocaIonoProducts` | native | File snapshot/application sequence parity remains green |
| L6D use inside PPP | `--madoca-l6d-shadow` performs measurement-neutral lookup only | partial | M3 must add opt-in STEC state/row application and prove row/value parity |
| PPP correction signs and ordering | Native correction contract plus materialization/residual diff tools | native | Default PPP parity matrix remains within the accepted MIZU/ALIC envelope |
| PPP commit-on-success and diagnostics | Native postfit validation/shadow telemetry | native | MIZU 1 h/6 h/24 h and ALIC regression gates remain green |
| GLONASS phase in MADOCA PPP | Native L1/L2 rows plus corrected postfit-shadow audit | native | Keep the pinned per-satellite demeaned RMS at millimetre scale and span below 4 cm |
| MADOCALIB `exec_ppp` | Native PPP runs end to end; remaining solution delta is tracked | partial | M4/M5 matrix meets the declared trajectory thresholds |
| MADOCALIB `exec_pppar` | Windows and authoritative Ubuntu Release both match the oracle's 108 Fix / 10 Float over all 118 epochs | native | Keep exact status agreement, no wrong/missed Fix, RMS at most 0.20 m, and maximum at most 1.0 m |
| MADOCALIB `exec_pppar-ion` | Bridge profile/input validation exists; native L6D state injection does not | open | M3 then M5 PPP-AR-ion sign-off |
| Triple/quad-frequency PPP-AR | Upstream 2.0 behavior has not been fully audited or signed off | open | Dedicated fixtures and ambiguity/state parity after dual-frequency M2 |
| Linked `postpos()` bridge | `madocalib_bridge` | oracle-only | Retain until M6; never select it in production runtime |

## Current Numerical Baselines

The frozen one-hour MIZU PPP-AR window has 118 matched output epochs.  On both
the Windows Release evidence lane and the authoritative Ubuntu Release lane,
the MADOCALIB `pppar` oracle and native path produce 108 Fix / 10 Float with
exact 118/118 status agreement and no native-only or oracle-only Fix.

On Windows, the native/oracle 3D delta RMS is 0.054269 m, the maximum is
0.419804 m, and the trailing-1800-second RMS is 0.035616 m.  On Ubuntu, the
3D delta RMS is 0.189146 m and the maximum is 0.451489 m.  These results meet
the M2 status and trajectory exit gates.  M3--M6 remain open, so this does not
complete the overall migration.

The float-PPP history and older MIZU/ALIC/full-day measurements remain in issue
#148 and `madoca_port_plan.md`; they are regression context, not proof that
PPP-AR or L6D integration is complete.

## Migration Milestones

Each milestone is a sequence of focused issue-#148 branches and PRs.  Default
behavior stays bit-identical until that milestone's default-flip gate is met.

### M0 -- Freeze the ledger

- Keep this capability table synchronized with code and CI.
- Freeze the oracle, license boundary, fixtures, baseline metrics, and artifact
  names.
- Define the M1--M6 acceptance gates before changing solver behavior.

Exit: docs build and link checks pass; issue #148 records the merged ledger.

### M1 -- Reach the native PPP-AR resolver safely

- Replace the misleading 3D `convergence_threshold_horizontal` measurement with
  an actual local-horizontal deviation and report horizontal and vertical
  telemetry separately.
- Add a typed kinematic convergence policy; do not weaken the static default.
- Sweep the kinematic window and threshold on the pinned 118-epoch case.
- Prove that native per-frequency AR reaches candidate formation/LAMBDA while
  preserving the current default output when the policy is disabled.

Exit: after warm-up, at least one epoch reaches the per-frequency ambiguity
candidate stage; telemetry distinguishes warm-up, horizontal, vertical, and
candidate-stage rejection; default PPP/CLAS/MADOCA tests do not regress.

#### M1 implementation evidence (2026-07-13)

The native convergence window now reports the legacy ECEF 3D deviation plus
local horizontal and vertical deviations.  `legacy-3d` remains the default, so
existing admission behavior is unchanged; `local-enu` is an explicit diagnostic
policy with separate horizontal and vertical thresholds.

On the pinned MIZU 120-input-epoch smoke (118 published solutions), the legacy
window reached maxima of 1.17008 m ECEF 3D, 0.660907 m horizontal, and 1.15825 m
vertical and did not admit AR.  The strict 0.10 m horizontal / 0.20 m vertical
policy also did not admit AR.  A bounded threshold sweep reached the native
per-frequency resolver at 0.75 m horizontal / 1.25 m vertical (1200 s) and at
1.50 m / 2.00 m (1080 s); these relaxed values are evidence probes, not new
defaults.

The 0.75 m / 1.25 m probe produced 78 resolver attempts.  All 78 stopped at
`wide_lane_only_insufficient_n1`; zero reached a complete N1 integer fix.  The
current solver nevertheless published those 78 epochs as `PPP_FIXED`.  M1
therefore proves resolver reachability and identifies the exact next boundary:
M2 must separate wide-lane-only updates from complete N1 Fix admission before
any native Fix-rate or trajectory-parity claim is accepted.

### M2 -- Port `exec_pppar` ambiguity behavior

- Compare ambiguity eligibility, frequency pairs, datum/reference selection,
  float ambiguity values/covariance, integer candidates, ratio, and admission
  epoch by epoch.
- Port the missing MADOCALIB behavior in small slices.
- Apply the accepted integer constraint to a compatible native fixed state and
  compare the published solution, not Fix status alone.

Exit on the pinned window: 118 epochs remain aligned; native produces no wrong
Fix against the oracle trajectory; Fix/Float status agreement is at least 95%;
native Fix count is at least the oracle's 108; matched fixed-epoch 3D delta RMS
is at most 0.20 m and maximum is at most 1.0 m.  If upstream numerical limits
make one threshold infeasible, change it only in a dedicated evidence PR.

#### M2a -- Wide-lane-only admission contract

The pinned MADOCALIB source returns `1` for a wide-lane-only result and `2` for
a complete narrow-lane fix (`src/ppp_ar.c`).  `src/ppp.c` uses the temporary
wide-lane-constrained state for that epoch's position, maps `SOLQ_FIX_WL` back
to `SOLQ_PPP`, and only holds the ambiguity state for `SOLQ_FIX`.  Native follows
the same contract: a wide-lane-only result publishes its temporary position as
`PPP_FLOAT`, reports zero fixed ambiguities and zero ratio, then restores the
pre-AR float filter state.

On the pinned 120-input-epoch MIZU Windows Release probe this changes 78
incorrectly labelled Fix rows to Float without changing any of the 118
published positions (before versus after 3D delta RMS and maximum are both
0 m).  The stage counts remain 78 wide-lane-only attempts and zero complete N1
fixes.  The Linux parity build currently publishes 84 Float rows with 44
wide-lane-only attempts, exposing a remaining platform/state-trajectory delta
for M2b.  A dedicated parity CI gate rejects any published Fix on every valid
row while no complete N1 fix exists, independently of that row-count delta.

#### M2b -- One covariance update per MADOCA epoch

The pinned MADOCALIB `pppos()` loop copies `rtk->x` and `rtk->P` into its
temporary state before every residual-screening pass and commits the first
accepted result.  It therefore applies an epoch's measurement covariance
update once.  Native previously fed the already-updated covariance into as
many as eight relinearization iterations, repeatedly consuming the same rows
and becoming overconfident before ambiguity resolution.

The opt-in `GNSS_PPP_PFDUMP` trace now reports the position covariance norm at
the pre-EWL, post-EWL, and post-WL boundaries with GPS week/TOW.  At 00:03:00
the native post-WL norm changes from 0.962 m to 2.281 m, versus 2.49 m in the
MADOCALIB trace.  At 00:03:30 it changes from 0.742 m to 1.812 m, versus
2.28 m.  Remaining float-state differences are tracked separately; this slice
only removes the proven repeated-update mismatch.

On the pinned 120-input-epoch MIZU probe, all 118 solution epochs remain
aligned.  Native produces 31 complete fixes, all inside MADOCALIB fixed epochs,
and no wrong fix.  The full-window native/oracle 3D delta RMS improves from
10.923 m to 4.391 m; the final 30-minute RMS is 1.046 m.  The M2 exit criteria
are not yet met.

#### M2c -- Measurement variance parity

The coherent MADOCA CLI already applies MADOCALIB's 300:1 code-to-phase error
ratio.  The per-frequency filter then multiplied the resulting code variance by
the legacy diagnostic default of 9.0, making code sigma exactly three times the
oracle value.  That scale now defaults to 1.0 and remains available only as an
explicit diagnostic override.

Two other variance terms were missing.  MADOCALIB adds `0.01^2 m^2` estimated-
troposphere model variance to every code and phase row and adds 0.1 times the
SSR URA variance.  Native now applies both terms on the coherent MADOCA path.
The L6E decoder already retained subtype-7 URA indices, but product conversion
dropped them; conversion now carries the DF389/MADOCALIB sigma into the SSR
correction.  A focused unit test pins the complete URA-index mapping.

At GPS week 2360, TOW 173010, native/oracle sigmas now agree at trace precision:
G04 code/phase are 2.2719/0.0143 m, G08 are 1.8963/0.0136 m, and G16 are
1.3174/0.0129 m.  On the pinned 120-input-epoch probe, all 118 solution epochs
remain aligned.  Native publishes 38 complete fixes, all within the oracle's
fixed epochs, with no wrong fix.  Full-window native/oracle 3D delta RMS improves
from 4.391 m to 2.194 m, maximum delta falls from 14.78 m to 5.79 m, horizontal
RMS is 1.396 m, Up RMS is 1.692 m, and final-30-minute RMS is 1.091 m.  M2 remains
open because status agreement and fixed-position thresholds are not yet met.

#### M2d -- First-epoch carrier-phase admission

MADOCALIB initializes each per-frequency ambiguity before `ppp_res()` and uses
the corresponding carrier-phase row in that same epoch.  Native previously
required one completed ambiguity-lifecycle update, so its first usable MADOCA
epoch contained 31 code rows and no phase rows.  Coherent MADOCA per-frequency
mode now uses a zero minimum lock count while the shared non-MADOCA default
remains one.

On the pinned MIZU probe, the first native equation grows to 72 code and 68
phase rows and its first-epoch native/oracle 3D delta falls from 1.320 m to
0.770 m.  Across all 118 aligned solution epochs, native produces 47 complete
fixes, all inside oracle Fix epochs, with no wrong Fix.  Full-window 3D delta
RMS is 2.358 m, maximum is 5.907 m, and final-30-minute RMS is 1.125 m.  The
full-window RMS is higher than M2c's 2.194 m, so this row-contract correction
does not close M2; remaining native-only rows and state priors must be aligned
before judging its combined trajectory effect.

#### M2e -- Strict SSR broadcast-ephemeris admission

MADOCALIB's `seleph()` requires the broadcast ephemeris referenced by the SSR
orbit IODE and rejects a satellite when that record is unavailable.  Native's
IODE-aware selector previously fell back to the nearest-age ephemeris, applying
the SSR delta to a different broadcast orbit and admitting GPS, GLONASS, and
BeiDou satellites before the oracle.  The selector now returns no state when
the requested record is missing.  GPS, Galileo, QZSS, and GLONASS compare IODE
directly; BeiDou follows MADOCALIB's Compact SSR rule,
`toes mod 2048 == (IODE * 8) mod 2048`.

On the pinned MIZU trace, native now reproduces the oracle's constellation
admission sequence exactly: G08 appears at 00:02:00, GLONASS at 00:02:30, C28
at 00:03:30, and the seven-satellite BeiDou set at 00:04:00.  GPS and BeiDou
row counts also match at every checked early epoch.  The remaining row-count
gap is isolated to six QZSS rows per epoch and four GLONASS rows after its
admission, which are frequency-selection differences for a later slice.

Across all 118 aligned solution epochs, native produces 65 complete fixes, all
inside oracle Fix epochs, with no wrong Fix.  The full-window native/oracle 3D
delta RMS is 0.317 m, maximum is 0.988 m, horizontal RMS is 0.213 m, Up RMS is
0.235 m, and final-30-minute RMS is 0.299 m.  M2 remains open: 43 oracle Fix
epochs are still native Float, status agreement remains below 95%, and the
fixed-epoch RMS exit threshold is not yet proven.

#### M2f -- QZSS three-frequency row parity

MADOCALIB orders QZSS per-frequency observations as L1, L5, then L2.  Native
already preserved all three RINEX bands and implemented additional-frequency
states, corrections, and measurement rows, but selected L2 as the secondary
frequency by default.  That made L2 ineligible for the additional slot and
dropped L5, leaving six missing rows across J02/J03/J04 at every epoch.

Coherent MADOCA now selects L5 as the secondary frequency and retains L2 as
the additional MADOCALIB ordinal by default.  `GNSS_PPP_MADOCA_QZSS_L5=0`
remains an opt-out.  A RINEX reader regression test pins simultaneous L1/L5/L2
retention rather than testing L5 replacement alone.

On the pinned early MIZU trace, QZSS grows from 12 to the oracle's 18 rows per
epoch with exact L1/L5/L2 code-and-phase identities.  Across all 118 aligned
solution epochs, native produces 61 complete fixes, all inside oracle Fix
epochs, with no wrong Fix.  Full-window native/oracle 3D delta RMS is 0.337 m,
maximum is 0.854 m, horizontal RMS is 0.232 m, Up RMS is 0.244 m, and the
final-30-minute RMS is 0.309 m.  The full RMS is 0.020 m above M2e while the
maximum improves by 0.134 m; this slice accepts the exact oracle row contract.
M2 remains open.

#### M2g -- GLONASS phase rows and STEC prior parity

MADOCALIB initializes each estimated STEC state with `VAR_IONO=SQR(60.0)` and
admits both GLONASS L1 and L2 carrier-phase rows in the float filter.  Native
previously used a 100 m² generic STEC prior and suppressed only the GLONASS L1
phase row; L2 remained active despite the code-only comment and opt-out.

Coherent MADOCA now uses the oracle's 3600 m² STEC prior and enables both
GLONASS phase rows by default.  `GNSS_PPP_INIT_IONO_VAR` remains an explicit
prior override, and `GNSS_PPP_MADOCA_GLONASS_PHASE=0` consistently suppresses
both GLONASS phase rows.

The earlier metre-scale GLONASS drift diagnosis was caused by postfit telemetry
using receiver geometry materialized before the accepted filter update.  With
the corrected shadow from M2g's prerequisite PR, the four pinned GLONASS
satellites show demeaned residual RMS of 2.4--5.2 mm, spans of 1.6--3.4 cm, and
near-zero residual/elevation correlation.  On the 120-epoch MIZU probe, all 118
solution epochs remain aligned, native remains at 90 Fix / 28 Float with no
wrong Fix, and full-window 3D delta RMS improves from 0.196 m to 0.193 m.
M2 remains open because the early-window status agreement gate is not met.

#### M2h -- GLONASS receiver code-IFB variance

MADOCALIB adds `VAR_GLO_IFB=SQR(0.6)` to every GLONASS pseudorange row
while leaving carrier-phase rows unchanged. Native previously used the common
code variance for GLONASS, underestimating its code sigma after the constellation
entered the filter.

Coherent MADOCA per-frequency mode now adds the same `0.36 m^2` code-only
variance. At GPS week 2360, TOW 173160, the native/oracle GLONASS code sigmas
agree at trace precision: R09 is 3.1256 m, R19 is 6.3378 m, and R21 is
2.5205 m. On the pinned 120-epoch MIZU probe, all 118 solution epochs remain
aligned, native remains at 90 Fix / 28 Float with no wrong Fix, and full-window
3D delta RMS is 0.193236 m with a 0.330669 m maximum. M2 remains open because
the early-window status agreement gate is not met.

#### M2i -- Multi-frequency geometry-free slip scope

MADOCALIB evaluates the primary carrier against every configured secondary
frequency and resets only the primary plus the frequency whose geometry-free
jump crosses `pos2-slipthres=0.15 m`.  Native previously checked only its first
secondary and then reset every ambiguity for the satellite.

The coherent MADOCA path now uses the oracle threshold without the generic
0.5 m floor, evaluates every retained frequency, and performs frequency-scoped
resets.  The pinned G28 event at 00:02:00 has a 0.144 m L1-L2 change and a
0.175 m L1-L5 change, so L1/L5 reset immediately while L2 remains continuous.
Across the 118 aligned output epochs, native improves from 91 Fix / 27 Float
to 95 Fix / 23 Float, with no native-only Fix.  Status agreement is 88.98%.
M2 remains open because 13 oracle Fix epochs are still native Float.

#### M2j -- Solid-earth-tide convention

MADOCALIB uses the legacy Love-number solid-earth-tide displacement for this
profile.  Native's default IERS Step-1+2 model created a real receiver-geometry
difference even though it was not the remaining AR admission cause.  Coherent
MADOCA now selects the legacy convention while other PPP profiles retain their
configured tide model.

On the pinned window, the default output is exactly equal to the explicit
legacy-tide A/B output for all 118 rows.  Fix/Float counts remain 95/23, while
the full native/oracle 3D RMS improves from 0.209425 m to 0.112100 m and the
trailing-1800-second RMS improves from 0.221474 m to 0.070070 m.  M2 remains
open because the status and native-Fix-count gates are not met.

#### M2k -- SSR clock relativity and first-N1 confirmation

MADOCALIB `satpos_ssr()` recomputes broadcast clock relativity as
`-2 * dot(position, velocity) / c^2`, using RTKLIB's 1 ms forward-difference
velocity.  Native previously retained the eccentric-anomaly form returned by
its broadcast ephemeris propagation.  The difference is satellite-specific at
centimetre scale once harmonic orbit terms are included and was sufficient to
shift the tightly weighted carrier residuals and PAR candidate ordering.
Coherent MADOCA now reproduces the state-vector form with the exact SSR IODE;
other PPP profiles keep their existing clock convention.

On Windows Release, the clock correction alone recovers every oracle Fix but
admits one near-threshold native Fix one epoch early.  To keep the established
no-wrong-Fix guard without tuning the ratio threshold, the first
coherent-MADOCA N1 success is provisional and must be followed by a consecutive
successful AR attempt.  The already-applied EWL/WL state is retained and
published as Float during that confirmation epoch.  After the first N1 commit,
normal single-attempt reacquisition applies.

On the Windows Release probe, all 118 solution epochs align: native and oracle
both produce 108 Fix / 10 Float, with zero native-only and zero oracle-only Fix.
The native/oracle 3D delta RMS is 0.040094 m, the maximum is 0.246063 m, and the
trailing-1800-second RMS is 0.032859 m.

The authoritative Ubuntu Release probe improves from its previous 76 Fix / 42
Float baseline to 77 Fix / 41 Float, still with zero native-only Fix.  It
misses 31 oracle Fix epochs, so M2 remains open.  At the first cross-platform
split (TOW 175260), Windows retains 16 WL pairs and reaches an N1 Fix, while
Ubuntu retains 15 after E31-E06 crosses the 0.20-cycle WL admission boundary.
The oracle starts with 18 N1 pairs, excludes only G31 and C09, then fixes 16 at
ratio 2.333 against its 1.20 dimension-adjusted threshold.  Ubuntu exhausts
ten PAR exclusions without a fix.  Covariance sigmas remain closely aligned;
the remaining cause is the accumulated float ambiguity mean and WL candidate
path, not the SSR clock formula.

#### M2l -- Galileo MW-supported WL admission

The first Ubuntu-only miss at TOW 175260 was caused by the weakly constrained
float ambiguity gauge, not a different observation or covariance admission
sigma.  Windows and Ubuntu seed E06 with identical corrected code, phase, and
STEC values, but nanometre-scale phase rounding at the first epoch grows along
the ambiguity/STEC gauge.  E31-E06 therefore reaches ordinary WL admission on
Windows and misses the 0.20-cycle fractional gate on Ubuntu.  Its independent
Melbourne-Wübbena average is identical on both platforms.

Coherent MADOCA now lets a mature Galileo MW average support WL admission after
60 samples when its fractional distance is below 0.20 cycles.  The filter-state
WL value still supplies the integer and its covariance still must pass the
existing 1.0-cycle standard-deviation gate.  BeiDou is deliberately excluded:
its MW and filter-state WL integers use different bias datums in this profile.
A high-agreement MADOCA ratio may also pass within one percent of its adjusted
threshold, containing numerical boundary movement to candidate sets whose two
LAMBDA solutions already agree above 90 percent.

Windows remains exactly 108 Fix / 10 Float.  Authoritative Ubuntu improves from
77 Fix / 41 Float to 83 Fix / 35 Float, recovers TOW 175260, and retains zero
native-only Fix.  Full-window 3D delta RMS is 0.172757 m and the maximum is
0.439642 m.  Twenty-five oracle-Fix/native-Float epochs remain, beginning at
TOW 175320, so M2 remains open.

#### M2m -- Frequency-scoped STEC discontinuity reset

At the first remaining miss, G31 suffered a frequency-specific carrier slip.
MADOCALIB reset that carrier ambiguity and then cleared the carrier-derived
ionosphere-delta history before the next STEC prediction.  Native reset the
ambiguity but retained the old delta baseline, so it counted the same carrier
discontinuity again in the ionosphere prediction and displaced the ambiguity
mean used by partial AR.

The native frequency-scoped reset now also clears the affected carrier's
ionosphere-delta history.  At TOW 175320, G31's pre-update native STEC then
agrees with the oracle within 0.039 m and its post-update STEC within 0.006 m.
Ubuntu remains at 83 Fix / 35 Float with no wrong Fix; full-window 3D delta RMS
is 0.172594 m and the maximum is 0.428885 m.  Windows remains exactly 108 Fix /
10 Float.  The state-transition defect is removed, but M2 remains open because
the cross-platform ambiguity-gauge sensitivity still requires an admission
rule that is deterministic for equivalent valid partial-AR subsets.

#### M2n -- One-removal N1 subset lookahead

MADOCALIB's partial AR greedily removes the lowest-elevation satellite on which
the first two LAMBDA candidates disagree.  After the STEC reset was aligned,
weak ambiguity/STEC gauge differences still changed that disagreement set and
therefore the greedy removal order across platforms.  At TOW 175320, the
ordinary 14-pair Ubuntu set rejects at ratio 1.3746; removing the implicated
G26 pair produces a 13-pair child at ratio 1.5179 against its unchanged 1.20
threshold and passes the existing candidate-agreement gate.

Before taking the greedy removal, coherent MADOCA per-frequency AR now examines
implicated one-satellite children in canonical order and accepts the first one
that already passes the same LAMBDA ratio, dimension, and candidate-agreement
contracts.  It adds no ambiguity candidate and relaxes no threshold.  When no
child passes, the previous MADOCALIB-compatible greedy path is unchanged.

Both Windows and authoritative Ubuntu Release now match the oracle's 108 Fix /
10 Float at all 118 epochs, with no wrong or missed Fix.  Windows 3D delta RMS
is 0.054269 m with a 0.419804 m maximum; Ubuntu RMS is 0.189146 m with a
0.451489 m maximum.  The M2 exit criteria are therefore satisfied.

### M3 -- Apply L6D ionosphere products

- Promote the proven snapshot lookup from shadow telemetry to an explicit
  opt-in PPP STEC measurement/state input.
- Match MADOCALIB area selection, age, uncertainty, signal coefficient, state
  initialization, reset, and process-noise behavior.
- Compare `pppar-ion` row keys and values before final positions.

Exit: supported snapshot/application keys match exactly; delay/std values match
within `1e-6`; the configured common residual row set has no unmatched rows;
MIZU `pppar-ion` produces no wrong Fix and improves or preserves M2 trajectory
metrics.  The feature remains opt-in until M5.

#### M3 prerequisite -- key snapshots by the completed L6D message

The file decoder previously keyed every snapshot with `decodeTime()`, whose
contract is the primary PRN-200 channel time used by the global staleness gate.
When a PRN-201 ionosphere message completed independently, its satellite
correction times advanced but the snapshot key remained at the PRN-200
reference time.  A downstream causal lookup could therefore select a snapshot
whose corrections appeared about 3,487 seconds in the future and reject every
one against MADOCALIB's 300-second age gate.

`messageTime()` now records the channel time of the most recently completed
message, while `decodeTime()` retains its primary-channel contract.  File
snapshots use `messageTime()`.  The real MIZU PRN-201 fixture verifies that no
satellite correction is newer than its snapshot and that completed corrections
share the snapshot time.  In a layered, not-yet-published M3 STEC application
experiment, this changes the first valid epochs from zero accepted rows to
9, 9, and 10 rows, each with a 24-second correction age.  This establishes the
causal snapshot fix as a prerequisite; row/value and solution parity remain M3
work.

### M4 -- Close remaining row and boundary differences

- Resolve QZSS atmosphere admission and every unexplained native-only or
  oracle-only row.
- Trace differences in order: materialization, satellite set, row set,
  residual components, then solution.
- Close the remaining 24-hour boundary residue without time-window-specific
  position guards.

Exit: all supported materialization and residual identities are accounted for;
configured common component deltas pass their explicit tolerances; MIZU 24-hour
maximum native/oracle 3D delta is at most 1.0 m with no new boundary spike.

### M5 -- Make parity a release gate

- Run MIZU and ALIC for 1 h and 6 h on every solver-change PR.
- Run their 24 h windows on the extended/optional lane.
- Cover PPP, PPP-AR, and PPP-AR-ion with solution, status, row, and runtime
  artifacts.
- Record exact commands and schema versions in the CI summary.

Exit: all six solver/dataset profiles have pinned baselines; required lanes fail
on missing artifacts, status regression, trajectory regression, or runtime over
2x their accepted native baseline; default builds still run without MADOCALIB.

### M6 -- Complete the migration

- Promote native behavior only after M2--M5 pass with an explicit opt-out for
  one release cycle.
- Remove any runtime-facing bridge selection from normal user workflows while
  retaining the opt-in oracle build and tests.
- Remove superseded preview knobs and guards after proving their replacements.
- Update user documentation and close issue #148 with the final matrix.

Exit: production code, installed targets, and default CI have no MADOCALIB
dependency; the linked checkout is used only by the labeled oracle lane; native
PPP, PPP-AR, and PPP-AR-ion satisfy the M5 matrix; no required migration item
remains open.

## PR and Evidence Rules

- Branch from current `develop` and use one behavior surface per PR, for example
  `feature/148-madoca-ar-horizontal-convergence`.
- Link issue #148 and identify the milestone in every PR.
- Keep diagnostics measurement-neutral unless the PR explicitly declares a
  solver change.
- Run default tests without MADOCALIB and oracle tests with the pinned checkout.
- Publish materialization, satellite-set, residual-row, residual-component,
  status, solution, and runtime artifacts in that order as applicable.
- Do not accept a higher Fix count without wrong-Fix and constrained-position
  evidence.
- Record rejected hypotheses in issue #148 and remove their runtime wiring.

## Immediate Slice

Begin M3 by landing the completed-message snapshot-time prerequisite, then
promote the existing measurement-neutral L6D shadow lookup to an explicit
opt-in STEC input.  Pin receiver-area selection, correction age, delay,
standard deviation, signal coefficient, and snapshot application keys; then
compare the common native/oracle `pppar-ion` residual rows before judging Fix
status or trajectories.  Preserve the established M2 guards and keep the
feature opt-in until the M5 matrix passes.
