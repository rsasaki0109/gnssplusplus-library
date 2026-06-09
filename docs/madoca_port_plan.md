# MADOCA Port Plan

This plan scopes a MADOCA foundation pass.  The goal is to capture what should
be ported from MADOCALIB, what should be shared with the existing CLAS work,
and how to build oracle parity without dragging reference code into production.

## Reference Inputs

Local MADOCALIB copy inspected for iter1:

```text

```

Primary source files:

- `src/mdccssr.c`
- `src/mdciono.c`
- `src/ppp.c`
- `src/ppp_ar.c`
- `sample_data/`
- `readme.txt`

Existing libgnss++ reference:

- `docs/references/madocalib-gap.md`
- `docs/clas_port_architecture.md`
- `docs/clas_validated_datasets.md`
- `include/libgnss++/io/qzss_l6.hpp`
- `src/io/qzss_l6.cpp`
- `include/libgnss++/algorithms/ppp.hpp`
- `src/algorithms/ppp.cpp`
- `include/libgnss++/algorithms/ppp_ar.hpp`
- `src/algorithms/ppp_ar.cpp`

## MADOCALIB License Note

`readme.txt` states that MADOCALIB is distributed under a BSD 2-clause license
with additional exclusive clauses.  It also states that users may develop,
produce, or sell non-commercial or commercial products using MADOCALIB as long
as they comply with the license.  The copyright notices list the Cabinet
Office, Japan; Lighthouse Technology & Consulting; JAXA; Toshiba Electronic
Technologies; and T. Takasu.

The GUI license is separate in `MADOCALIB_GUI_LICENSE.txt`; the GUI should not
be part of the C++ port scope.

Porting rule for libgnss++:

- Use MADOCALIB as a reference and oracle.
- Prefer literal small-function ports only when the license header and
  attribution are deliberately handled.
- Keep any direct MADOCALIB-linked oracle code test-only and opt-in.
- Do not create a production dependency on MADOCALIB.

## CLASNAT Lessons To Carry Forward

The CLASNAT work converged because it used a tight oracle loop:

- Start with small deterministic helper parity, not whole-PPP parity.
- Keep native code and external oracle code separated.
- Target literal behavior first, cleanup second.
- Use tight numerical tolerances, typically `1e-6` for helper-level values.
- Add tests around one helper family at a time.
- Preserve production behavior while oracle-linked builds are optional.
- Use whole-run trajectory comparisons only after helper parity is stable.

For MADOCA this means:

- Add `madoca_parity` as a small, pure helper namespace first.
- Add `test_madoca_parity.cpp` as an empty GoogleTest landing point.
- Later add a MADOCALIB oracle layer that is linked only when explicitly
  configured.
- Gate initial helper ports against MADOCALIB outputs before wiring them into
  PPP.

## Current libgnss++ MADOCA/CLAS Baseline

Already present:

- A native PPP core with SSR product application.
- A PPP-AR module.
- A QZSS L6/CSSR decoder used for CLAS-style correction ingestion.
- Compact sampled correction transport for CLI workflows.
- `clas-ppp --profile madoca` CLI naming for profile-level runs.
- Extensive synthetic QZSS L6 tests in `tests/test_cli_tools.py`.
- Analysis/sign-off helpers for MADOCA solution diffs, satellite-set diffs,
  PPP residual row-set diffs, and SPP residual dump summaries.

Important caveat:

- The current `qzss_l6` decoder is CLAS-oriented.  It has useful building
  blocks such as the bit reader, L6 frame handling, mask/orbit/clock/bias
  concepts, and SSR product population, but it is not yet a MADOCALIB-equivalent
  MADOCA L6E/L6D port.

## MADOCALIB Key Functions

`mdccssr.c` handles QZSS L6E MADOCA-PPP Compact SSR:

- `init_mcssr(gt)` initializes the global MADOCA CSSR state.
- `input_qzssl6e(rtcm, data)` synchronizes a byte stream on the L6 preamble.
- `input_qzssl6ef(rtcm, fp)` reads L6E bytes from a file.
- `decode_qzss_l6emsg(rtcm)` decodes a complete L6E frame/subframe sequence.
- `decode_mcssr_mask()` decodes Compact SSR subtype 1 mask.
- `decode_mcssr_oc()` decodes subtype 2 orbit correction.
- `decode_mcssr_cc()` decodes subtype 3 clock correction.
- `decode_mcssr_cb()` decodes subtype 4 code bias.
- `decode_mcssr_pb()` decodes subtype 5 phase bias and discontinuity
  indicator.
- `decode_mcssr_ura()` decodes subtype 7 URA.
- `mcssr_sel_biascode(sys, code)` maps observation codes to MADOCA bias codes.

`mdciono.c` handles QZSS L6D wide-area ionospheric correction:

- `init_miono(gt)` initializes L6D ionosphere decoder state.
- `input_qzssl6d(mdcl6d, data)` synchronizes L6D byte input.
- `input_qzssl6df(mdcl6d, fp)` reads L6D bytes from a file.
- `decode_qzss_l6dmsg(mdcl6d)` decodes L6D frame/subframe content.
- `decode_miono_coverage()` decodes coverage/region/area geometry.
- `decode_miono_correction()` decodes STEC correction coefficients.
- `miono_get_corr(rr, nav)` selects an ionosphere area for receiver ECEF
  position and materializes per-satellite delay/std corrections.

`ppp.c` contains the RTKLIB-derived PPP processing flow:

- Measurement correction and combination: `corr_meas()`.
- Slip detection: `detslp_ll()`, `detslp_gf()`, `detslp_mw()`, `detslp_ssr()`.
- State updates: `udpos_ppp()`, `udclk_ppp()`, `udtrop_ppp()`,
  `udiono_ppp()`, `udifb_ppp()`, `udbias_ppp()`, `udstate_ppp()`.
- Atmospheric models: `trop_model_prec()`, `model_trop()`, `model_iono()`.
- Residual construction: `ppp_res()`.
- Main solver entry: `pppos()`.

`ppp_ar.c` contains PPP ambiguity resolution:

- Wavelength setup: `get_wavelength()`.
- Single-difference satellite pair generation: `gen_sat_sd()`.
- Extra-wide-lane and wide-lane searches: `search_amb_ewl()`,
  `search_amb_wl()`.
- LAMBDA search: `search_amb_lambda()`.
- Fixed-state update: `update_states()`.
- Partial ambiguity exclusion: `exc_sat_par()`.
- Main AR entry: `ppp_ar()`.

## MADOCA Message Scope

MADOCA L6E in `mdccssr.c` is the immediate correction stream target:

- L6 preamble: `0x1ACFFC1D`.
- L6 message byte length without Reed-Solomon code: 218 bytes.
- L6 data payload: 1695 bits.
- MADOCA Compact SSR RTCM message number: 4073.
- Supported subtypes in this decoder: 1, 2, 3, 4, 5, and 7.
- Defined systems include GPS, GLONASS, Galileo, BeiDou, QZSS, and BeiDou-3.
- Defined MADOCA L6E PRNs in the inspected code are 204, 205, 206, 207, 209,
  210, and 211.

MADOCA L6D in `mdciono.c` is the ionosphere stream target:

- Coverage message type: 1.
- Correction message type: 2.
- Defined systems include GPS, GLONASS, Galileo, BeiDou, and QZSS.
- Defined L6D PRNs in the inspected code are 200 and 201, with 197 retained
  for testing.

## Sample Data Inventory

The inspected sample data includes:

- Observation RINEX:
  `sample_data/data/rinex/MIZU00JPN_R_20250910000_01D_30S_MO.rnx`
- Observation RINEX:
  `sample_data/data/rinex/ALIC00AUS_R_20250910000_01D_30S_MO.rnx`
- Navigation RINEX:
  `sample_data/data/rinex/BRDM00DLR_S_20250910000_01D_MN.rnx`
- Antenna data:
  `sample_data/data/igs20.atx`
- L6 sample trees:
  `sample_data/data/l6_is-qzss-mdc-003/2025/091/`
- L6 sample trees:
  `sample_data/data/l6_is-qzss-mdc-004/2025/091/`
- Convenience L6 tree:
  `sample_data/data/l6/2025/091/`

Batch examples show the intended scenarios:

- `exec_ppp.bat`: MADOCA-PPP with L6E PRNs 204 and 206.
- `exec_pppar.bat`: MADOCA-PPP-AR with L6E PRNs 204 and 206.
- `exec_pppar_ion.bat`: PPP-AR plus L6D ionosphere with PRNs 200 and 201.
- `exec_cssr2ssr.bat`: `cssr2ssr` conversion from L6E to RTCM3/debug text.

These sample files should become the first whole-run oracle candidates after
helper parity exists.

## Commonality With CLAS

Likely shared pieces:

- QZSS L6 byte synchronization and preamble handling.
- Bit extraction utilities.
- CSSR mask/orbit/clock/code-bias/phase-bias concepts.
- SSR update interval table.
- Conversion from decoded correction epochs to `SSRProducts`.
- PPP application points for orbit, clock, code bias, phase bias, trop, and
  ionosphere corrections.
- CLI profile plumbing for CLAS/MADOCA family correction runs.

Likely MADOCA-specific pieces:

- L6E PRN set and message generation facility handling.
- MADOCA Compact SSR signal masks and bias-code selection.
- Subtype transmission pattern and frame assembly from `framelen[]`.
- L6D wide-area ionosphere region/area selection.
- STEC coefficient scaling and URA-to-standard-deviation conversion.
- Triple/quad-frequency PPP and PPP-AR behavior described in MADOCALIB 2.0.
- MADOCALIB-specific options such as alert ignoring and PRN selection.

## Proposed Port Structure

Initial foundation:

- `include/libgnss++/algorithms/madoca_parity.hpp`
- `tests/test_madoca_parity.cpp`

Future production modules, not for iter1:

- `include/libgnss++/io/madoca_l6.hpp`
- `src/io/madoca_l6.cpp`
- `include/libgnss++/algorithms/ppp_madoca.hpp`
- `src/algorithms/ppp_madoca.cpp`

Future oracle modules, test-only and opt-in:

- `tests/madocalib_oracle/`
- `include/libgnss++/external/madocalib_bridge.hpp`
- `src/algorithms/madocalib_bridge.cpp`
- `MADOCALIB_PARITY_LINK=ON` links the MADOCALIB C sources for direct helper
  oracle calls and the whole-run `postpos()` bridge.  The default build keeps
  this off.

## Iter4 MADOCALIB Bridge Baseline

The first whole-run oracle path is an opt-in static-link delegate:

- Configure with `-DMADOCALIB_PARITY_LINK=ON`.
- Run `gnss_ppp --madocalib-bridge` to delegate the whole run to MADOCALIB
  `postpos()`.
- Pass MADOCA L6E files with repeated `--madocalib-l6 <file>` arguments.
- Pass L6D ionosphere files with repeated `--madocalib-mdciono <file>`
  arguments when using the ionosphere scenario.
- The default build does not link MADOCALIB and rejects `--madocalib-bridge`
  before producing output.

The public sample-data smoke case uses the `exec_ppp.bat` MIZU scenario with
the two L6E channels from the `is-qzss-mdc-004` tree:

```sh
ROOT=${MADOCALIB_ROOT}

./build_iter4_madocalib/apps/gnss_ppp \
  --madocalib-bridge \
  --obs "$ROOT/sample_data/data/rinex/MIZU00JPN_R_20250910000_01D_30S_MO.rnx" \
  --nav "$ROOT/sample_data/data/rinex/BRDM00DLR_S_20250910000_01D_MN.rnx" \
  --madocalib-l6 "$ROOT/sample_data/data/l6_is-qzss-mdc-004/2025/091/2025091A.204.l6" \
  --madocalib-l6 "$ROOT/sample_data/data/l6_is-qzss-mdc-004/2025/091/2025091A.206.l6" \
  --antex "$ROOT/sample_data/data/igs20.atx" \
  --madocalib-start "2025/04/01 00:00:00" \
  --madocalib-end "2025/04/01 00:59:30" \
  --madocalib-ti 30 \
  --out /tmp/madocalib_bridge_sample_0000.pos \
  --summary-json /tmp/madocalib_bridge_sample_0000.json \
  --quiet
```

Observed iter4 result:

- Command exit status: `0`.
- Output: `/tmp/madocalib_bridge_sample_0000.pos`, MADOCALIB format.
- Header program line: `MADOCALIB ver.2.1`.
- Time span in solution rows: `2025/04/01 00:01:00.000` through
  `2025/04/01 00:59:30.000`.
- Solution rows: `118`.
- Quality counts: `Q=6` for all `118` rows.
- Health-check RMS against the observation RINEX approximate-position header
  `(-3857167.6484, 3108694.9138, 4004041.6876)`:
  all rows `4.502422 m`, last 100 rows `4.461333 m`, last 60 rows
  `4.453027 m`, last 30 rows `4.457820 m`.

The RMS values above are not an accuracy acceptance gate.  No precise truth
coordinate was found in the inspected public sample-data files, so the RINEX
header approximate position is used only as a run-health check.  For the native
MADOCA port, the reference baseline is the generated MADOCALIB bridge
trajectory itself, starting with this MIZU one-hour PPP window and later adding
the PPP-AR and L6D ionosphere sample windows.

## MADOCA Solution Diff Tool

Use `scripts/analysis/madoca_solution_diff.py` to compare a MADOCALIB bridge
`.pos` file against a native LibGNSS++ `.pos` file.  It accepts both the
MADOCALIB calendar-time rows and the LibGNSS++ GPS-week/TOW rows, matches common
epochs, reports ENU and 3D deltas, and can write JSON plus per-epoch CSV output.

```sh
python3 scripts/analysis/madoca_solution_diff.py \
  /tmp/madocalib_bridge_sample_0000.pos \
  /tmp/native_madoca_sample_0000.pos \
  --reference-ecef -3857167.6484 3108694.9138 4004041.6876 \
  --base-label bridge \
  --candidate-label native \
  --json-out /tmp/madoca_solution_diff.json \
  --match-csv /tmp/madoca_solution_matches.csv
```

Keep this as an analysis/sign-off helper.  It is not part of the runtime
solver path.

## Split Cleanup Status

The old `feat/rinex4-madoca` branch mixed unrelated RINEX, MADOCA, PPP, SPP,
and tooling work.  It should remain an archive branch, not a branch to merge
directly.  The active cleanup path is to use issue `#123` as the tracker and
move only focused, develop-based slices through review.

Status as of 2026-06-10:

- Draft PR `#49` was closed as superseded by the split tracker.  Its branch is
  retained as historical context because it still contains useful candidate
  code, but its direct tree diff includes branch drift and should not be used
  as a merge plan.
- PR `#124` landed the RINEX `TIME OF FIRST OBS` parser fix.  This removes a
  focused RINEX item from the old branch without importing the wider branch.
- PR `#125` landed `scripts/analysis/madoca_satellite_set_diff.py` plus tests.
- PR `#126` landed `scripts/analysis/ppp_residual_row_set_diff.py` plus tests.
- PR `#127` landed `scripts/analysis/spp_residual_dump_summary.py` plus tests.
- The remaining useful work should be treated as candidate material, restored
  file-by-file or reimplemented against current `develop`, then validated in a
  small PR.

When reading `origin/feat/rinex4-madoca`, use this interpretation:

- Pure file additions are the safest candidates for direct restoration.
- Edits to runtime solver files need a fresh local review against current
  `develop` before any code is copied.
- Deletions in a direct branch comparison are usually drift from the stale
  branch, not intentional removals.
- RINEX and MADOCA items must stay separated unless a specific test proves a
  shared interface needs to move in the same PR.
- Temporary split branches should be deleted after merge; long-lived archive
  branches should be kept until their candidate content has been exhausted.

## Analysis Helper Ladder

The landed analysis tools now form a staged parity ladder.  They are not solver
features; they exist so native work can be compared to MADOCALIB, CLAS, or
existing libgnss++ output before changing runtime behavior.

`madoca_solution_diff.py`:

- Compares full solution `.pos` trajectories.
- Accepts MADOCALIB calendar-time rows and native GPS-week/TOW rows.
- Matches common epochs and reports ENU/3D deltas.
- Belongs late in the flow, after helper, decoder, and residual-level parity
  are stable enough that trajectory deltas are meaningful.

`madoca_satellite_set_diff.py`:

- Compares satellite participation sets across MADOCALIB `$SAT` rows and native
  PPP correction logs.
- Identifies epochs where the two stacks are solving with different satellite
  membership before investigating residual magnitudes.
- Belongs before position-delta investigation because a trajectory difference is
  not actionable if the satellite set is already different.

`ppp_residual_row_set_diff.py`:

- Compares residual row keys without requiring bit-for-bit residual values.
- Helps separate row selection, rejection, and logging-shape differences from
  numerical model differences.
- Belongs before tighter PPP residual value parity because missing or extra rows
  make value comparisons noisy.

`spp_residual_dump_summary.py`:

- Summarizes final SPP residual dumps by satellite and signal.
- Gives a low-cost health check for row counts, RMS, mean, min/max, and largest
  residuals.
- Belongs in the seed-diagnostic path because SPP quality affects PPP initial
  conditions and early convergence.

The next helper in the same style should be `spp_iteration_dump_summary.py`.
It should summarize per-iteration SPP behavior rather than only the final dump,
so it can show whether a run is failing because of bad initial rows, unstable
iteration updates, or late residual selection.  Keep it as an analysis tool with
unit tests first; do not wire it into normal CLI output until the dump format is
stable.

## Issue And Branch Policy

Use issue `#123` for the remaining work that originated from PR `#49`.  The
issue should collect links to each focused PR, a short note about the slice, and
whether the slice was merged, closed, or deferred.

Branch rules:

- Branch from current `develop`.
- Name split branches by purpose, for example
  `split/spp-iteration-dump-summary` or `split/madoca-core-foundation`.
- Keep a branch to one behavioral surface: one analysis tool, one parser
  feature, one runtime knob family, or one solver behavior change.
- Commit only files needed for that slice.
- Do not stage unrelated local artifacts or generated images.
- Open draft PRs first, wait for CI, then mark ready only when the branch is
  intended to merge.
- After merge, prune the remote branch and confirm `develop` is synced.

PR body checklist:

- State the source of the slice: fresh implementation, restored candidate file,
  or manual rework of archive branch material.
- List the user-facing or maintainer-facing behavior.
- List exact tests run locally.
- Mention whether the slice changes runtime behavior or only adds analysis
  tooling.
- Link issue `#123` when the slice is part of the old MADOCA/RINEX branch
  cleanup.

## Next Slice Queue

Preferred next low-risk slice:

- Add `scripts/analysis/spp_iteration_dump_summary.py`.
- Add `tests/test_spp_iteration_dump_summary.py`.
- Register `python_spp_iteration_dump_summary_tests` in `tests/CMakeLists.txt`
  and label it as `core`.
- Keep the tool input format close to the existing SPP dump conventions.
- Output a stable text summary and optional JSON if the candidate implementation
  already has it.
- Validate with `python3 -m py_compile`, the direct Python unit test, CMake
  configure, focused CTest, and `git diff --check`.

Next medium-risk slice:

- Add a native MADOCA core foundation under `madoca_core` or a similarly narrow
  namespace.
- Keep it independent from PPP runtime wiring.
- Start with deterministic types and helpers: system identifiers, signal masks,
  subtype names, update intervals, URA scaling, and bias-code mapping.
- Add GoogleTest coverage for each helper group before exposing the module to
  decoder or solver code.
- Avoid linking MADOCALIB in the default build.

Next decoder slice:

- Add an L6E frame/subframe assembly test using tiny fixtures or captured bytes.
- Decode only metadata and subtype dispatch first.
- Do not map decoded rows to `SSRProducts` until subtype-level tests are
  stable.
- Keep L6D ionosphere parsing separate from L6E correction parsing.

Next PPP diagnostics slice:

- Add explicit residual/logging knobs only after the analysis tools define the
  expected row keys and summaries.
- Prefer opt-in diagnostics over changing default solver output.
- Add tests that prove the same run with diagnostics disabled preserves existing
  output.

Deferred large slices:

- Whole-run native MADOCA PPP parity.
- PPP-AR parity against MADOCALIB sample windows.
- L6D ionosphere integration into PPP.
- Triple/quad-frequency PPP-AR behavior.
- First-class production MADOCALIB bridge behavior beyond the opt-in oracle
  path.

## MADOCALIB License Notes

`madocalib/readme.txt` identifies MADOCALIB as a MADOCA-PPP reference/test
library derived from RTKLIB 2.4.3 b34, with PPP-AR and message-conversion
functions copyrighted by third parties.  It is distributed under BSD 2-Clause
terms with additional clauses.  Source redistributions must retain the upstream
copyright notice, license conditions, and disclaimer; binary redistributions
must reproduce those notices in documentation or other materials.  The upstream
readme also notes companion Windows executables/shared libraries with their
own original licenses, and the optional GUI has separate license information in
`MADOCALIB_GUI_LICENSE.txt`.

For CI parity use, checkout `QZSS-Strategy-Office/madocalib` into an
`external/madocalib` directory and point `MADOCALIB_ROOT_DIR` there.  The
opt-in oracle build links only the required source file(s), does not vendor
MADOCALIB into the default build, and keeps the upstream checkout/readme
available in the workflow workspace so the BSD notice and additional terms are
retained.

## Implementation Roadmap

Phase 0, documentation and branch cleanup:

- Keep the port plan current enough that a new PR can be scoped from it without
  rereading the stale mega-branch first.
- Move all remaining branch-derived work through issue `#123`.
- Land low-risk analysis tools before runtime code.
- Close or merge old PRs deliberately; do not leave overlapping open PRs that
  touch the same behavior.
- Keep archive branches available until their useful additions have been either
  ported, reworked, or explicitly rejected.

Phase 1, diagnostics and oracle surface:

- Finish SPP iteration and residual summaries so PPP seed differences can be
  explained before comparing whole-run PPP.
- Keep the MADOCA solution, satellite-set, PPP residual row-set, SPP residual
  dump, and SPP iteration tools independent and testable.
- Add fixtures that are small enough to review in PRs.
- Prefer deterministic parser/summary tests over tests that require large
  external sample files.
- Document any external sample command separately from the required CI tests.

Phase 2, helper parity:

- Port pure helpers with stable inputs first: bit extraction expectations,
  system ID mapping, signal mask to code mapping, update interval selection,
  URA scaling, and bias-code selection.
- Add MADOCALIB oracle calls or generated fixture rows only behind explicit
  test/oracle configuration.
- Use `1e-6` tolerance for scaled metric outputs unless the upstream integer
  scaling implies an exact comparison.
- Keep helper tests free of PPP side effects.
- Treat helper parity failures as blockers for decoder-to-PPP wiring.

Phase 3, L6E decode foundation:

- Implement MADOCA L6E stream/frame assembly.
- Decode subtype metadata and dispatch before decoding all payload fields.
- Decode subtypes 1, 2, 3, 4, 5, and 7 into a neutral correction epoch.
- Compare decoded rows against MADOCALIB `cssr2ssr` or direct oracle output.
- Only then map decoded correction epochs to `SSRProducts`.
- Keep unsupported or incomplete subtypes explicit; silent partial decode is not
  acceptable.

Phase 4, L6D ionosphere foundation:

- Implement coverage and correction message parsing separately from L6E.
- Implement area selection and STEC delay/std calculation.
- Add sample-driven tests for PRNs 200 and 201.
- Keep PRN 197 only where it is needed for compatibility or fixture coverage.
- Do not feed L6D products into PPP until decoder-level values match the oracle
  for selected receiver positions.

Phase 5, PPP application:

- Wire MADOCA correction products into the existing PPP pipeline only after the
  correction objects are independently testable.
- Keep CLAS and MADOCA profiles explicit.
- Add runtime knobs as narrow, documented options, not broad solver rewrites.
- Compare sample `exec_ppp` and `exec_pppar` windows against MADOCALIB output.
- Use satellite-set and residual row-set diffs before interpreting final
  position differences.

Phase 6, PPP-AR and multifrequency:

- Audit MADOCALIB 2.0 triple/quad-frequency PPP-AR behavior.
- Add fixture-level AR tests before enabling broad CLI behavior.
- Separate ambiguity search behavior from correction ingestion behavior in PRs.
- Do not expand default PPP-AR behavior until single-frequency and dual-frequency
  parity diagnostics are already understandable.

Phase 7, whole-run sign-off:

- Run the MIZU one-hour PPP bridge case as the first whole-run oracle.
- Add the PPP-AR and L6D ionosphere sample windows only after the PPP-only
  window is stable.
- Compare common epochs with `madoca_solution_diff.py`.
- Record satellite-set, residual row-set, and final trajectory summaries in the
  PR or follow-up issue.
- Treat trajectory RMS against approximate RINEX header coordinates only as a
  smoke signal; parity against MADOCALIB output is the real sign-off target.

## Acceptance Gates

General gates:

- Existing tests pass without MADOCALIB installed.
- New analysis scripts have direct unit tests and CTest registrations.
- Python tools pass `python3 -m py_compile`.
- C++ additions pass focused GoogleTest or CTest coverage.
- `git diff --check` is clean before publishing.
- CI must pass before a split PR is considered done.

Oracle and licensing gates:

- Oracle-linked tests are opt-in and clearly labeled.
- Default builds do not require a MADOCALIB checkout.
- Any direct MADOCALIB-linked path keeps upstream notices available in the
  workflow workspace.
- Production code does not vendor MADOCALIB sources.

Helper and decoder gates:

- Helper parity reaches `1e-6` for deterministic scaled values unless exact
  integer parity is available.
- L6E decoded orbit, clock, code-bias, phase-bias, mask, and URA rows match the
  chosen MADOCALIB oracle rows for sample frames.
- L6D decoded STEC delay/std rows match MADOCALIB for selected receiver
  positions.
- Unsupported subtypes and systems produce deterministic skip/error behavior.

Solver gates:

- Diagnostic-enabled and diagnostic-disabled runs preserve existing default
  output unless a PR explicitly changes the contract.
- Satellite-set differences are understood before residual value differences are
  judged.
- Residual row-set differences are understood before whole-solution differences
  are judged.
- Whole-run PPP and PPP-AR sample windows are compared only after helper and
  decoder parity are stable.

## Current Non-Goals

- Do not merge `feat/rinex4-madoca` directly.
- Do not combine RINEX parser work with MADOCA runtime solver work.
- Do not introduce a production dependency on MADOCALIB.
- Do not wire incomplete L6E/L6D decoders into default PPP behavior.
- Do not change CLAS behavior while adding MADOCA-only foundations unless a
  shared helper has explicit CLAS regression coverage.
- Do not claim whole-run parity from approximate-coordinate RMS alone.
- Do not make large solver rewrites in analysis-tool PRs.
