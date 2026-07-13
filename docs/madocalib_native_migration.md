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
| GLONASS phase in MADOCA PPP | Intentionally excluded: no MADOCA phase-bias product and measured time-varying residual | native policy | Keep phase preview off unless new correction evidence changes the model |
| MADOCALIB `exec_ppp` | Native PPP runs end to end; remaining solution delta is tracked | partial | M4/M5 matrix meets the declared trajectory thresholds |
| MADOCALIB `exec_pppar` | Native per-frequency path is reachable, but the pinned baseline is 0 native Fix versus 108 oracle Fix in 118 matched epochs | open | M1/M2 must match candidate, admission, status, and constrained-state behavior |
| MADOCALIB `exec_pppar-ion` | Bridge profile/input validation exists; native L6D state injection does not | open | M3 then M5 PPP-AR-ion sign-off |
| Triple/quad-frequency PPP-AR | Upstream 2.0 behavior has not been fully audited or signed off | open | Dedicated fixtures and ambiguity/state parity after dual-frequency M2 |
| Linked `postpos()` bridge | `madocalib_bridge` | oracle-only | Retain until M6; never select it in production runtime |

## Current Numerical Baselines

The frozen one-hour MIZU PPP-AR CI window has 118 matched output epochs.  The
MADOCALIB `pppar` oracle produces 108 Fix and 10 Float; the native per-frequency
path produces 0 Fix and 118 Float.  The recorded native/oracle 3D delta RMS is
10.090 m with a 45.380 m maximum after selecting the correct uncombined/STEC
state model.

Convergence telemetry localizes the immediate blocker before LAMBDA: 118
evaluations comprise 19 insufficient-history epochs and 99 position-deviation
rejects.  The final 20-epoch window has a 1.17008 m maximum ECEF deviation
against the current 0.1 m threshold.  The configuration calls this a horizontal
threshold, but the implementation currently uses a full ECEF 3D norm.

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

M1 begins with a measurement correction, not a threshold increase:

1. compute ECEF deltas from the convergence-window mean;
2. rotate them to local ENU at the mean position;
3. track maximum horizontal norm and maximum absolute Up separately;
4. preserve the legacy 3D metric for compatibility telemetry;
5. add a typed kinematic convergence policy and focused unit tests; and
6. rerun the pinned 118-epoch native/oracle case before selecting thresholds.

This prevents the current mislabeled metric from turning a parameter sweep into
an accidental change of meaning.
