# MADOCA Port Plan

This plan scopes a MADOCA foundation pass.  The goal is to capture what should
be ported from MADOCALIB, what should be shared with the existing CLAS work,
and how to build oracle parity without dragging reference code into production.

## Reference Inputs

Local MADOCALIB copy inspected for iter1:

```text
${MADOCALIB_ROOT}
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

The RMS values against the observation RINEX approximate-position header are
not an accuracy acceptance gate.  They are a run-health check because the
header coordinate is offset from an external station-coordinate reference by
about `4.505125 m`.

A follow-up check against the Nevada Geodetic Laboratory IGS20 daily time
series found the row for `MIZU` on `25APR01` / 2025-04-01:

- Source: `https://geodesy.unr.edu/gps_timeseries/IGS20/txyz/MIZU.txyz2`.
- Reference ECEF: `(-3857171.27021569, 3108692.75896666, 4004040.09533769)`.
- Offset from the RINEX header approximate coordinate: `4.505125 m`.
- MADOCALIB bridge RMS against this external reference: all `0.383720 m`,
  last 30 rows `0.143513 m`.

For native MADOCA development, compare against both the external MIZU reference
above and the generated MADOCALIB bridge trajectory.  The L6D/PPP-AR
ionosphere sample now has a first bridge-vs-native baseline below; closing the
remaining trajectory gap is PPP/AR behavior work, not L6D byte parsing work.


## Native L6E/L6D Status

The native path now has a production MADOCA foundation in `madoca_core`:

- `--madoca-l6e <file>` decodes MADOCA L6E Compact SSR from PRNs such as
  204/206 and loads orbit, clock, code-bias, phase-bias, and atmosphere-capable
  SSR products into `PPPProcessor`.
- `--madoca-l6d <file>` decodes MADOCA L6D wide-area ionosphere coverage and
  correction messages from PRNs 200/201/197, selects the receiver area using
  the RINEX approximate receiver position, and materializes per-satellite STEC
  atmosphere rows.  It is currently a supplemental native option that requires
  `--madoca-l6e`.
- The CLI rejects mixing native MADOCA inputs with `--ssr`, `--ssr-rtcm`, or
  `--madocalib-bridge`; the MADOCALIB path continues to use
  `--madocalib-l6` and `--madocalib-mdciono`.
- Native summary JSON reports `madoca_native_l6d_loaded`, input files, decoded
  L6D result count, and combined native correction counts.
- `gnss_ppp --ppp-correction-log <log.csv>` writes per-epoch/per-satellite PPP
  correction diagnostics for SSR orbit/clock, code/phase bias, URA,
  troposphere, ionosphere, DCB, and IONEX application.  This is the first
  native-vs-MADOCALIB PPP gap-analysis hook after L6E/L6D decode parity.
- `scripts/analysis/ppp_correction_log_diff.py` summarizes one correction log
  or compares two logs on `(week,tow,sat)`, with optional JSON and diff CSV
  output for native L6E versus L6E+L6D isolation runs.
- The opt-in MADOCALIB oracle wrapper can decode L6E files through
  `input_qzssl6ef()` and compare clock-epoch orbit/clock, code/phase
  bias, and URA rows against the native decoder.  This caught and fixed MADOCA
  L6E-specific GNSS ID mapping (`4=QZSS`, `7=BD3`), BD3 PRN base,
  hour-boundary TOW rollover handling, and duplicate PRN/file row ordering in
  oracle comparisons.
- The same oracle wrapper can decode L6D files through `input_qzssl6df()` and
  compare final per-epoch MIONO results against the native decoder.  This
  caught and fixed stale satellite rows that native used to retain across
  coverage-message updates.

Short native smoke check using the public MIZU sample:

```sh
ROOT=${MADOCALIB_ROOT}/sample_data/data

./build/apps/gnss_ppp \
  --obs "$ROOT/rinex/MIZU00JPN_R_20250910000_01D_30S_MO.rnx" \
  --nav "$ROOT/rinex/BRDM00DLR_S_20250910000_01D_MN.rnx" \
  --antex "$ROOT/igs20.atx" \
  --madoca-l6e "$ROOT/l6_is-qzss-mdc-004/2025/091/2025091A.204.l6" \
  --madoca-l6e "$ROOT/l6_is-qzss-mdc-004/2025/091/2025091A.206.l6" \
  --madoca-l6d "$ROOT/l6_is-qzss-mdc-004/2025/091/2025091A.200.l6" \
  --madoca-l6d "$ROOT/l6_is-qzss-mdc-004/2025/091/2025091A.201.l6" \
  --max-epochs 20 \
  --out /tmp/gnssplusplus_native_l6e_l6d.pos \
  --summary-json /tmp/gnssplusplus_native_l6e_l6d_summary.json \
  --ppp-correction-log /tmp/gnssplusplus_native_l6e_l6d_corrections.csv \
  --quiet
```

Observed native smoke result:

- Command exit status: `0`.
- Valid solutions: `20`.
- Native L6D loaded: `true`.
- Native L6D decoded results: `840`.
- Combined native corrections: `89576` total, `106` satellites, `14441`
  atmosphere rows after clearing stale L6D area rows on each coverage update.
- Atmospheric ionosphere applications in the first 20 epochs: `246`.
- L6E-only versus L6E+L6D correction-log comparison over the same 20 epochs now
  has `1173` common rows, `0` base-only/candidate-only rows, and zero deltas for
  orbit `dx/dy/dz`, clock, URA, code bias, and phase bias.  The remaining
  expected deltas are L6D atmosphere tokens and STEC (`123` non-zero ionosphere
  rows, `1153` rows carrying atmosphere tokens).
- That diagnostic caught atmos-only L6D rows hiding nearby L6E orbit/clock/URA
  samples at the leading edge and at intervening epochs.  `SSRProducts`
  interpolation now searches component-wise nearby samples so supplemental L6D
  atmosphere does not perturb L6E orbit/clock/bias application.

First one-hour `exec_pppar_ion.bat`-equivalent comparison:

- MADOCALIB bridge command used `sample_pppar_iono.conf`,
  L6E PRNs 204/206, L6D PRNs 200/201 from `l6_is-qzss-mdc-004`, and the
  window `2025/04/01 00:00:00` through `00:59:30` at `30 s`.
- Native parity command uses the same RINEX/nav/ANTEX/L6 files with
  `--madoca-navsys 61 --kinematic --no-ionosphere-free --estimate-ionosphere`
  plus `--enable-ar --ar-ratio-threshold 1.5 --max-epochs 120`.  The
  `sample_pppar_iono.conf` reference uses `pos1-ionoopt=est-stec`, so the
  native comparison should not use the default ionosphere-free mode for this
  window.
- MADOCALIB bridge: exit status `0`, `118` solution rows from GPS TOW
  `172860` through `176370`, quality counts `Q=6` for `21` rows and `Q=1`
  for `97` rows.
- Native after keeping receiver-clock SPP reseeding enabled: exit status `0`,
  `120` valid PPP-float rows, `0` fixed rows, `3704` applied STEC constraints,
  `840` decoded L6D results, and `14441` atmosphere correction rows loaded.
  Kinematic position SPP reseeding remains disabled; testing it made the
  bridge delta much worse.
- `scripts/analysis/madoca_solution_diff.py` is the standard solution-level
  comparison hook for this window.  It now reports max-delta epochs and first
  3D threshold crossings in addition to JSON and per-epoch CSV output.  The
  current report command is:

  ```sh
  python3 scripts/analysis/madoca_solution_diff.py \
    /tmp/madocalib_bridge_pppar_ion_0000.pos \
    /tmp/gnssplusplus_native_clockreset_confirm_stec_ar.pos \
    --reference-ecef -3857171.27021569 3108692.75896666 4004040.09533769 \
    --base-label madocalib_bridge \
    --candidate-label native_l6e_l6d_eststec \
    --json-out /tmp/gnssplusplus_madoca_solution_diff.json \
    --match-csv /tmp/gnssplusplus_madoca_solution_diff.csv
  ```

- Against the external MIZU IGS20 ECEF reference, the standardized report gives
  bridge RMS `0.108 m` horizontal / `0.149 m` up / `0.184 m` 3D over all `118`
  bridge rows, and `0.020 m` horizontal / `0.024 m` 3D over the final 30-minute
  common window.
- Before the PPP BeiDou-GEO gate, native over the same final 30-minute common
  window was `25.596 m` horizontal / `28.970 m` 3D, with native-vs-bridge RMS
  `25.589 m` horizontal / `13.577 m` up / `28.968 m` 3D.  The all-window max
  delta was early at GPS TOW `172920` with `1587.604 m` 3D, while the final
  30-minute max delta was TOW `174600` with `47.901 m` 3D.
- Native PPP now has optional machine-readable filter diagnostics via
  `--ppp-filter-log` and row residual diagnostics via `--ppp-residual-log`.
  The pre-GEO-gate 120-epoch MIZU run produced `960` filter-iteration rows and
  `63489` residual rows; the solution diff was unchanged by logging
  (`25.589 m` horizontal / `13.577 m` up / `28.968 m` 3D final-window RMS).
- The pre-GEO-gate TOW `172920` all-window spike was an early filter-update
  problem: final iteration residuals were still `2482.387 m` code RMS and
  `145.156 m` phase RMS, with `38.895 m` position update, `27.107 m` clock
  update, and `611.293 m` max ionosphere-state update.  The largest code
  residual was BeiDou `C02` at `-14130.509 m` with an ionosphere state of
  `-19326.328 m`; the largest carrier residual was `G26` at `-199.775 m`.
- The pre-GEO-gate TOW `174600` final-window spike was different: the final
  iteration position update was only `0.124 m` and carrier residual RMS was
  `1.962 m`, but code RMS remained `1175.209 m`.  The dominant rows were
  BeiDou GEO satellites `C03` (`-7166.319 m`, ionosphere state
  `-369011.377 m`) and `C04` (`+2571.68 m`, ionosphere state `+236041.9 m`).
  The correction log showed `corr_iono_m=0`, `stec_tecu=0`, and
  `ionosphere_estimation_constraint=0` for these rows because native MIONO
  conversion intentionally skips unsupported BDS2 frequencies.  An initial
  dual-code ionosphere-state seed did not materially change this result, and a
  single shared BeiDou receiver-clock state made the final-window 3D RMS worse
  (`73.520 m`), motivating a generation-specific BDS2/BDS3 receiver-clock
  split seeded from SPP-like biases.
- PPP now mirrors the existing SPP/RTK policy by excluding BeiDou GEO PRNs
  `C01`-`C05`/`C59`-`C63` until the native BDS2 GEO and receiver-clock model is
  ready.  With that gate, the same 120-epoch MIZU `exec_pppar_ion` comparison
  improves to native-vs-bridge RMS `5.152 m` horizontal / `8.730 m` up /
  `10.137 m` 3D over all `118` matched epochs, and `6.133 m` horizontal /
  `4.977 m` up / `7.898 m` 3D in the final 30-minute common window.  The max
  all-window delta is now TOW `172860` at `46.420 m` 3D, and there are no
  `100 m` or `1000 m` threshold crossings.  At TOW `174600`, BDS code rows are
  non-GEO (`C06`, `C07`, `C09`, `C10`, `C16`, `C27`, `C29`, `C30`, `C36`,
  `C39`, `C40`, `C45`, `C47`) with meter-level residuals on the low-PRN BDS2
  rows; `C03`/`C04` no longer enter residual diagnostics.
- A follow-up BDS2/BDS3 receiver-clock split is now in place: SPP reports
  `receiver_clock_biases_m` by `ReceiverClockBiasGroup`, and PPP allocates,
  seeds, and reseeds separate BDS2/BDS3 clock states when those groups are
  visible.  On the same 120-epoch MIZU run, native-vs-bridge RMS moves from
  GEO-gate-only `5.152 m` horizontal / `8.730 m` up / `10.137 m` 3D to
  `5.494 m` horizontal / `8.429 m` up / `10.061 m` 3D over all `118` matched
  epochs, and from `6.133 m` horizontal / `4.977 m` up / `7.898 m` 3D to
  `6.552 m` horizontal / `4.178 m` up / `7.771 m` 3D in the final 30-minute
  common window.  The early all-window max changes from `46.420 m` to
  `53.116 m` at TOW `172860`, still with no `100 m` or `1000 m` threshold
  crossings.  This is a modest 3D/up RMS improvement and aligns the filter with
  generation-specific BDS clock modeling, but it is not the final BDS residual
  fix.
- Residual diagnostics after the BDS split showed native was still admitting an
  average of `11.05` valid satellites per epoch without SSR orbit/clock rows
  (`R10`, `R11`, `R20`, `E18`, `C06`, `C47`, `J02`, `J03`, `J07`, plus
  intermittent `G18`, `G21`, and `E12`).  These rows explained persistent tail
  code outliers such as `E12` near `-65 m` with `ssr_available=0` and
  `orbit_clock_applied=0`.  Native MADOCA now requires SSR orbit/clock for
  satellite admission so broadcast-only fallback rows remain in diagnostics but
  do not enter the PPP filter.  With that gate, valid native satellites drop to
  `26.225` mean per epoch (`24` min / `29` max), with zero valid satellites
  lacking SSR orbit/clock.  The same native-vs-bridge comparison improves to
  `2.761 m` horizontal / `5.072 m` up / `5.775 m` 3D over all `118` matched
  epochs, and `2.924 m` horizontal / `3.359 m` up / `4.454 m` 3D in the final
  30-minute common window.  The all-window max is now `9.582 m` at TOW
  `172860`, and there are no `10 m`, `100 m`, or `1000 m` threshold crossings.
- PPP ionosphere-state allocation now also covers satellites that first become
  valid after filter initialization.  New per-satellite ionosphere states are
  appended dynamically before SSR atmosphere application, STEC diagnostics are
  counted only when the state constraint is actually applied, and ambiguity
  process noise is limited to explicit ambiguity states so appended ionosphere
  states do not inherit ambiguity process noise.  A regression test adds a late
  synthetic satellite and verifies that its STEC constraint and non-zero
  ionosphere state are present in the PPP diagnostics.
- With dynamic ionosphere states, the same 120-epoch MIZU comparison is now
  `2.276 m` horizontal / `5.239 m` up / `5.712 m` 3D over all `118` matched
  epochs, and `1.938 m` horizontal / `3.828 m` up / `4.291 m` 3D in the final
  30-minute common window.  The max remains the early TOW `172860` epoch at
  `9.582 m` 3D, with no `10 m`, `100 m`, or `1000 m` threshold crossings.
  The late G02 code residual RMS drops from `11.213 m` to `5.383 m` because the
  ionosphere state is no longer stuck at `0 m`; the late R22 code residual RMS
  drops from `33.628 m` to `4.242 m`.  Remaining error is now mostly an early
  up-component bias rather than large late code outliers.
- Native MADOCA now initializes the estimated troposphere state with the
  RTKLIB/MADOCALIB PPP ZTD variance `VAR_ZTD=SQR(0.12)` (`0.0144 m^2`) instead
  of the generic native PPP `0.36 m^2`.  This removes the initial over-loose ZTD
  coupling observed in diagnostics and improves the same comparison to `2.322 m`
  horizontal / `4.126 m` up / `4.735 m` 3D over all `118` matched epochs, and
  `1.940 m` horizontal / `3.589 m` up / `4.080 m` 3D in the final 30-minute
  common window.  The max 3D delta drops from TOW `172860` at `9.582 m` to TOW
  `173370` at `6.327 m`, still with no `10 m`, `100 m`, or `1000 m` threshold
  crossings.
- Native PPP now uses the RTKLIB/MADOCALIB troposphere-state form for estimated
  ZTD: the modeled slant delay remains hydrostatic plus wet climatology, while
  the estimated ZTD state contributes only through the wet mapping as
  `model_slant + m_w * (ztd_state - model_zenith)`.  This closes a model gap but
  only slightly changes the MIZU comparison by itself (`2.322 m` horizontal /
  `4.124 m` up / `4.733 m` 3D).
- Native MADOCA L6D STEC std is now honored by PPP ionosphere-state constraints.
  Per-satellite `atmos_stec_std_l1_m:*` values are used as the constraint sigma,
  and constraints with broadcast sigma above the MADOCALIB-style `1.0 m` reject
  gate are skipped instead of falling back to direct observation correction in
  `estimate_ionosphere` mode.  Preferred-network atmosphere token filtering also
  preserves STEC std/delay/source-subtype metadata.  On the same 120-epoch MIZU
  `exec_pppar_ion` comparison this improves native-vs-bridge RMS to `2.553 m`
  horizontal / `1.349 m` up / `2.887 m` 3D over all `118` matched epochs, and
  `2.608 m` horizontal / `1.681 m` up / `3.102 m` 3D in the final 30-minute
  common window.  The max 3D delta is now `3.395 m` at TOW `172920`, with no
  `10 m`, `100 m`, or `1000 m` threshold crossings.
- Native PPP now also reads satellite PRN ANTEX PCO entries and applies the
  satellite-frame offset to the corrected satellite position before range
  modeling.  The satellite offset direction follows the MADOCALIB/RTKLIB
  convention for the ANTEX satellite frame, so the loaded north/east/up vector
  is subtracted from the satellite center-of-mass position.  On the same MIZU
  run this improves native-vs-bridge RMS to `2.440 m` horizontal / `1.227 m` up
  / `2.731 m` 3D over all `118` matched epochs, and `2.501 m` horizontal /
  `1.206 m` up / `2.776 m` 3D in the final 30-minute common window.  The max
  3D delta is now `3.023 m` at TOW `174420`, with no `10 m`, `100 m`, or
  `1000 m` threshold crossings.
- Native PPP now reads receiver ANTEX `NOAZI` PCV grids in addition to receiver
  PCO and applies the interpolated code/phase antenna correction using the
  active ionosphere-free or single-frequency signal coefficients.  This matches
  the MADOCALIB receiver `pos1-posopt2=on` path; satellite PCV is intentionally
  not enabled because MADOCALIB disables `satantpcv()` for MADOCA PPP and the
  sample profile has `pos1-posopt1=off`.  On the same 120-epoch MIZU run the
  receiver PCV update leaves the bridge comparison essentially unchanged:
  `2.439 m` horizontal / `1.229 m` up / `2.732 m` 3D over all `118` matched
  epochs, and `2.501 m` horizontal / `1.198 m` up / `2.773 m` 3D in the final
  30-minute common window.  The max 3D delta is `3.031 m` at TOW `174420`, with
  no `10 m`, `100 m`, or `1000 m` threshold crossings.  A bridge-only
  `pos1-posopt2=on` versus `off` comparison confirms receiver ANTEX is a small
  term here (`0.021 m` horizontal / `0.130 m` up / `0.132 m` 3D RMS), so it does
  not explain the remaining `~2.4 m` horizontal bias.
- Native standard PPP now applies RTKLIB/MADOCALIB-style phase windup to carrier
  phase measurements, matching `sample_pppar_iono.conf` `pos1-posopt3=on`.
  This closes another model gap, but the MIZU impact is small because the effect
  is mostly absorbed by carrier ambiguities: native-vs-bridge RMS is `2.439 m`
  horizontal / `1.229 m` up / `2.731 m` 3D over all `118` matched epochs, and
  `2.500 m` horizontal / `1.190 m` up / `2.769 m` 3D in the final 30-minute
  common window.  The max 3D delta remains at TOW `174420` (`3.037 m`).
- The WLNL OSR ambiguity preparation path now uses the same corrected phase
  combination convention as fixed-position WLS (`L - CPC` with the OSR phase
  corrections already contained in CPC).  Native MADOCA still keeps its default
  `DD_IFLC` AR method for this sample; forced WLNL was investigated as a gap
  explanation but not adopted as a profile default.
- Additional tempting parity changes were tested and not adopted.  Skipping
  observation epochs before the first decoded L6E sample worsened the same run
  to `2.848 m` horizontal / `4.673 m` up / `5.472 m` 3D with an `18.772 m` max
  at TOW `172860`.  Forcing native MADOCA `filter_iterations=1` to mirror
  `sample_pppar_iono.conf` `pos2-niter=1` also worsened the run to `2.673 m`
  horizontal / `7.142 m` up / `7.626 m` 3D, with final-window `7.119 m` 3D.
  The native profile therefore keeps its default PPP iteration count.
- Native `gnss_ppp` now exposes `--elevation-mask <deg>` so parity runs can
  reproduce MADOCALIB `pos1-elmask` settings.  Setting the MIZU run to the
  sample's `10 deg` mask increases the candidate satellite count and improves
  horizontal RMS to `2.134 m`, but worsens up RMS to `2.407 m` and 3D RMS to
  `3.217 m` (`2.026 m` horizontal / `3.177 m` up / `3.767 m` 3D in the final
  30-minute window).  Matching `stats-eratio1/2=300` was also tested and was
  worse in native (`6.126 m` 3D at the default `15 deg` mask, `4.774 m` 3D at
  `10 deg`).  These settings are useful diagnostics but are not adopted as the
  current best native MADOCA profile.
- MADOCALIB bridge `.stat` residual output was enabled with
  `out-outstat=residual` and compared against native correction/residual logs.
  On matched epochs the bridge uses more satellites on average (`28.35` bridge
  `vsat` satellites versus `26.23` native valid satellites), with average set
  differences of `5.65` bridge-only and `3.53` native-only satellites.  At TOW
  `174420`, bridge uses `31` satellites while native uses `28`; bridge-only
  satellites are `E04 E05 G02 J02 J03`, and native-only satellites are
  `E23 E24`.  At TOW `176370`, bridge uses `30` while native uses `25`;
  bridge-only satellites are `E05 E06 G09 G31 J02 J03`, and native-only is
  `E23`.  Most non-QZSS bridge-only rows are below the native default
  `15 deg` elevation mask, while `J02/J03` lack native SSR orbit/clock rows and
  are excluded by the current native MADOCA SSR-required admission policy.
- Native DD ambiguity resolution has enough candidates on the MIZU run, but the
  `DD_IFLC` LAMBDA validation ratio is consistently `1.0` against the
  configured `1.5` threshold, so the default native run remains PPP-float.
  Forcing `--ar-ratio-threshold 1.0` produces `116` fixed rows out of `120`,
  but does not improve parity (`2.430 m` horizontal / `1.262 m` up /
  `2.738 m` 3D RMS).  Combining the forced ratio with `--elevation-mask 10`
  also remains worse (`2.133 m` horizontal / `2.433 m` up / `3.236 m` 3D RMS).
  This keeps forced native AR and low-elevation AR out of the current profile.
- RINEX 3 observation selection now keeps code, carrier, Doppler, and SNR on the
  same selected tracking code per band using RTKLIB-style code priorities.  This
  fixes the previous GPS MIZU behavior that mixed `C1W` code with `L1L` carrier
  while looking up MADOCA SSR bias as `C1C`; the parser now selects `C1C/L1C`
  for GPS L1 and `C2W/L2W` for GPS L2 in this file.  A regression test covers
  the mixed `C1C/C1L/C1W/L1C/L1L` case.  On the one-hour MIZU parity run this
  improves final residual-code RMS slightly for GPS/GLO at sampled epochs, but
  the solution RMS moves to `2.513 m` horizontal / `1.448 m` up / `2.900 m` 3D
  (`2.579 m` horizontal / `1.733 m` up / `3.108 m` 3D in the final 30-minute
  window).  The parser fix is a correctness step; the worsened trajectory means
  the next parity target is signal-specific SSR bias/frequency handling rather
  than satellite count or AR threshold tuning.
- GPS RINEX `P/Y/W/M/N` tracking codes now map to the existing `GPS_L1P` /
  `GPS_L2P` signal types, so the native SSR signal ID for selected `C2W/L2W`
  observations is the P/W bias group instead of L2C.  `Observation` and
  `IonosphereFreeObs` now retain the selected RINEX code strings, DCB OSB lookup
  uses those strings when present, and `--ppp-correction-log` writes
  `primary_observation_code` / `secondary_observation_code` columns.  On the
  same one-hour MIZU parity run the GPS diagnostic rows are mostly
  `primary_signal=0`, `secondary_signal=2`, `C1C/C2W` (`1412` rows), proving
  that `C2W` is no longer collapsed to `GPS_L2C`.  The solution RMS remains
  unchanged at `2.513 m` horizontal / `1.448 m` up / `2.900 m` 3D (`2.579 m`
  horizontal / `1.733 m` up / `3.108 m` 3D in the final 30-minute window),
  because this `sample_pppar_iono.conf` parity profile runs
  `--no-ionosphere-free --estimate-ionosphere`; the applied code/phase bias,
  antenna, and phase-windup observation corrections are L1-primary in this
  mode, while L2 mainly feeds ionosphere seeding and AR diagnostics.
- MADOCA QZSS PRNs are now normalized when compact SSR rows are materialized
  into native PPP products.  The raw CSSR mask still exposes the spec PRNs
  (`193+`), but orbit/clock, bias, URA, L6D STEC tokens, and L6E atmosphere
  token labels are keyed as the observation IDs used by RINEX/navigation
  (`J01..J10`).  On the same MIZU parity run this makes `J02` and `J03` valid
  with SSR orbit/clock and STEC in all `120` native epochs.  The matched-epoch
  satellite-count gap closes from `28.35` bridge versus `26.23` native to
  `28.35` bridge versus `28.23` native, and bridge-only satellites are now
  explained by the native `15 deg` elevation mask (`431` rows) rather than
  missing QZSS SSR (`256` rows before this fix).  The solution moves to
  `2.285 m` horizontal / `2.297 m` up / `3.240 m` 3D RMS over all matched
  epochs, with the final 30-minute window improving to `2.183 m` horizontal /
  `1.634 m` up / `2.726 m` 3D.  The all-window up degradation means the next
  parity target is no longer satellite admission; it is the QZSS/L1-primary
  ionosphere and residual observation model now that `J02/J03` enter the native
  filter.
- Native MADOCA ionosphere estimation now mirrors MADOCALIB
  `const_iono_corr()` more closely: MIONO/STEC is no longer applied as an
  absolute pre-update on each ionosphere state.  Instead, PPP forms dedicated
  `iono_constraint` pseudo-observation rows each filter iteration and subtracts
  the per-system common bias `bsys = mean(corr.dly - x_iono)` for GPS, GLONASS,
  Galileo, and QZSS before constraining the state.  The filter log now reports
  `ionosphere_constraint_rows`, and the residual log labels these rows as
  `iono_constraint` so code residual RMS is not polluted by MIONO pseudo-rows.
  On the same MIZU run this improves the QZSS-normalized comparison to
  `2.402 m` horizontal / `1.186 m` up / `2.678 m` 3D RMS over all `118`
  matched epochs, and `2.245 m` horizontal / `0.784 m` up / `2.378 m` 3D in
  the final 30-minute window.  The max 3D delta is `3.344 m` at TOW `174420`.
  Native-vs-bridge final-iteration ionosphere-state differences drop from
  multi-meter common offsets to sub-meter GPS means for most satellites
  (`G04` `0.095 m`, `G16` `0.002 m`, `G27` `0.620 m`) while QZSS remains about
  `1.43 m` high on `J02` and `1.25 m` high on `J03`, making QZSS residual
  modeling the next narrow target.  Retesting the tempting skip-first-two-RINEX
  experiment after QZSS admission worsened the run to `2.652 m` horizontal /
  `2.301 m` up / `3.511 m` 3D, with a `12.535 m` first-epoch delta, so epoch
  skipping remains out of the profile.
- Native MADOCA L6E product materialization now stores ST4 code bias with the
  PPP application sign.  The decoded `CssrEpoch` values remain MADOCALIB-scale
  values, but `SSROrbitClockCorrection::code_bias_m` is negated because native
  PPP subtracts SSR code bias while MADOCALIB's `corr_meas()` adds the decoded
  MADOCA code bias to pseudorange.  This resolves the satellite-dependent GPS
  code residual offsets: matched final-iteration code residual RMS drops from
  `2.525 m` to `0.576 m` overall, and from `4.359 m` to `0.602 m` for GPS
  (`G26` `7.123 m` to `0.416 m`, `G16` `4.125 m` to `0.418 m`, `G02`
  `8.323 m` to `0.958 m`).  On the same MIZU run the bridge comparison improves
  to `0.668 m` horizontal / `0.865 m` up / `1.093 m` 3D RMS over all `118`
  matched epochs, and `0.467 m` horizontal / `1.009 m` up / `1.111 m` 3D in
  the final 30-minute window.  Before the phase-bias sign audit, the remaining
  gap was dominated by vertical behavior and QZSS residual details (`J02` phase
  residual RMS was still about `0.272 m` against the bridge), so the next
  parity target was QZSS phase / ionosphere residual modeling rather than GPS
  code bias sign.

- Native MADOCA L6E product materialization now stores ST5/ST6 phase bias with
  the PPP application sign as well.  MADOCALIB's `corr_meas()` adds decoded
  phase bias to carrier phase, while native PPP applies SSR phase bias by
  subtraction, so `SSROrbitClockCorrection::phase_bias_m` is negated without
  changing the decoded `CssrEpoch` rows or the MADOCALIB oracle bias fixtures.
  On the same MIZU run the bridge comparison improves again to `0.645 m`
  horizontal / `0.721 m` up / `0.967 m` 3D RMS over all `118` matched epochs,
  and `0.457 m` horizontal / `0.877 m` up / `0.989 m` 3D in the final
  30-minute window.  The max 3D delta remains the first matched epoch, TOW
  `172860`, at `2.473 m`; QZSS ionosphere/code coupling remains the next target
  because `J02` code residual RMS is still about `1.40 m` and its phase
  residual RMS is about `0.29 m` in native final-iteration diagnostics.

- A MADOCALIB-style position-covariance gate for `const_iono_corr()` was tested
  and rejected.  MADOCALIB skips STEC pseudo-observations when the previous
  position covariance is already tight; the naive native port used the current
  filter covariance and collapsed the MIZU run from `13,576` ionosphere
  constraint rows to only `14` total rows after the first epoch.  Bridge
  comparison worsened to `2.476 m` horizontal / `4.609 m` up / `5.232 m` 3D
  RMS over all matched epochs and `0.568 m` horizontal / `2.445 m` up /
  `2.510 m` 3D in the final 30-minute window, so this gate is not adopted.

- Native no-IFLC estimated-ionosphere PPP now adds secondary-frequency
  code-only measurement rows with the MADOCALIB ionosphere scale
  `(f1/f)^2` while keeping a single L1 STEC pseudo-observation per satellite.
  This is a contained parity step before adding true per-frequency carrier
  ambiguities: L1 retains the phase row and ambiguity state, L2/L5/E5/B2/etc.
  contribute code rows only, and `allow_ionosphere_constraint=false` prevents
  duplicate MIONO constraints.  The synthetic PPP regression verifies that L2
  code rows double the code measurement count, apply the expected L2/L1 STEC
  ratio, and leave the ionosphere constraint count unchanged.  On MIZU this
  improves the bridge comparison to `0.459 m` horizontal / `0.418 m` up /
  `0.621 m` 3D RMS over all `118` matched epochs, and to `0.122 m` horizontal
  / `0.189 m` up / `0.224 m` 3D in the final 30-minute window.  The max 3D
  delta remains the first matched epoch, TOW `172860`, at `2.489 m`; the final
  epoch QZSS residuals are now small and balanced (`J02` code `0.261 m` /
  `-0.119 m`, phase `0.023 m`; `J03` code `0.404 m` / `0.132 m`, phase
  `0.049 m`).  The next parity target is therefore the early-epoch transient
  and, if needed, proper per-frequency carrier ambiguities rather than more
  code-only QZSS coupling.

- QZSS receiver-clock splitting was also checked because MADOCALIB `ppp.c`
  maps `SYS_QZS` to its own receiver-clock state (`IC(5,opt)`) and the
  upstream change log notes "estimate receiver clock of QZS".  Enabling the
  native QZSS clock state without SPP QZSS seeding improved only the final
  30-minute window (`0.252 m` horizontal / `0.871 m` up / `0.906 m` 3D) but
  worsened the all-window comparison to `0.428 m` horizontal / `1.884 m` up /
  `1.932 m` 3D, with an early up-component spike at TOW `173370`.  Adding SPP
  QZSS clock seeding and MADOCALIB-like initial clock variance was worse
  (`0.435 m` horizontal / `2.026 m` up / `2.072 m` 3D all-window, `0.260 m` /
  `0.954 m` / `0.989 m` in the final 30 minutes).  The split absorbs much of
  the QZSS residual but destabilizes early vertical behavior, so it is not
  adopted in the current best native profile.  After the code-only
  multifrequency change above, revisit the QZSS clock split only if the
  remaining early-epoch transient points back to receiver-clock separation.

- PPP solution metadata now counts unique valid satellites instead of valid
  observation rows.  This matters after no-IFLC secondary code rows: the MIZU
  native trajectory is unchanged, but the candidate satellite count now reports
  `28.225` mean satellites instead of `56.45` measurement rows, and the first
  matched epoch reports `27` satellites instead of `54` rows.  The synthetic
  multifrequency PPP regression now verifies both the doubled code-row count and
  the unique solution satellite count.

- Native MADOCA now carries L6E orbit IODE through `CssrOrbitCorrection` and
  `SSROrbitClockCorrection`, and `NavigationData::getEphemerisByIode()` mirrors
  RTKLIB/MADOCALIB selection details, including BDS `toe % 2048 ==
  (iode * 8) % 2048`.  Strict PPP-side IODE enforcement was tested as a
  diagnostic but is gated off by default because it currently degrades the MIZU
  parity run: after the BDS TOE-modulo fix it reports `0.633 m` horizontal /
  `1.827 m` up / `1.933 m` 3D all-window RMS and `0.189 m` horizontal /
  `1.312 m` up / `1.326 m` 3D in the final 30-minute window.  Keeping IODE
  storage is still useful for traceability and future targeted admission work;
  the default run with IODE stored but not enforced remains unchanged at
  `0.459 m` horizontal / `0.418 m` up / `0.621 m` 3D all-window and
  `0.122 m` horizontal / `0.189 m` up / `0.224 m` 3D tail RMS.

  The native PPP correction log now exposes `ssr_orbit_iode`, `broadcast_iode`,
  and `orbit_clock_skip_reason`, and `gnss_ppp --enforce-ssr-orbit-iode` lets
  parity runs reproduce the strict-admission diagnostic without changing the
  default MADOCA profile.  The correction-log diff helper summarizes these skip
  reasons while remaining backward-compatible with older log CSV files, and now
  keys comparisons by week/TOW/satellite plus signal and observation-code fields
  so no-IFLC duplicate satellite rows can be compared directly.

  The strict MIZU diagnostic correction log has `3,689` orbit/clock skip rows:
  `2,191` `ssr_unavailable`, `1,258`
  `ssr_orbit_iode_no_matching_ephemeris`, and `240`
  `broadcast_ephemeris_unavailable`.  Comparing it against the default
  stored-not-enforced run over the same `12,096` correction-log rows shows
  `1,258` fewer orbit/clock applications and `890` fewer valid corrected rows;
  all `890` valid-row drops are caused by
  `ssr_orbit_iode_no_matching_ephemeris`.  The largest valid-row losses are
  Galileo-heavy (`E23`: `216`, `E09`: `104`, `E33`: `96`, and `E04`/`E06`/
  `E26`/`E31`: `54` each), with smaller BDS/GLONASS/GPS losses, and the first
  four epochs drop `40`, `40`, `38`, and `38` valid rows respectively.  This
  points the strict-admission degradation at early IODE/broadcast-ephemeris
  alignment, especially Galileo, rather than the unrelated full-run
  `ssr_unavailable` satellites or the `C56` broadcast-ephemeris gap.

  The L6 CSSR decoder now also gates ST2 orbit, ST3 clock, and ST11 combined
  orbit/clock updates on the subtype header IOD matching the active mask IOD,
  after still consuming the payload bits.  Synthetic L6 regressions cover stale
  clock, orbit-before-clock, and combined orbit/clock messages.  Re-running the
  same MIZU default and strict profiles after this guard leaves the solution RMS
  and correction-log numeric columns unchanged: default remains `0.459 m` /
  `0.418 m` / `0.621 m` all-window and `0.122 m` / `0.189 m` / `0.224 m` tail,
  while strict remains `0.633 m` / `1.827 m` / `1.933 m` all-window and
  `0.189 m` / `1.312 m` / `1.326 m` tail.  That confirms the strict loss is not
  stale CSSR-mask mixing; it is an admission-data issue where early SSR orbit
  IODEs, especially Galileo `31`-`33`, do not have matching broadcast ephemerides
  in the current single-day BRDM window, whose early Galileo IODnav records start
  at `34+` for the affected satellites.

  `gnss_ppp` now accepts repeated `--nav` arguments and merges broadcast
  ephemerides from all supplied RINEX navigation files; summary JSON keeps the
  legacy primary `nav` field and adds `nav_files` for the full list.  This was
  used as an admission diagnostic rather than a default profile change.  Fetching
  BKG `BRDC00IGS_R_20250900000_01D_MN.rnx` for 2025-03-31 and combining it with
  the sample 2025-04-01 DLR BRDM reduces strict MIZU
  `ssr_orbit_iode_no_matching_ephemeris` rows from `1,258` to `80`, raises
  orbit/clock applications from `8,407` to `9,585`, and raises valid correction
  rows from `5,884` to `6,694`.  The bridge comparison improves to `0.460 m` /
  `1.048 m` / `1.145 m` all-window RMS and `0.097 m` / `0.961 m` / `0.966 m`
  tail RMS.  The remaining `80` no-match rows are only Galileo `E04`/`E06` IOD
  `33`.

  BKG `BRDC00IGS_R_20250910000_01D_MN.rnx` for 2025-04-01 contains those
  E04/E06 IOD `33` overlap rows, but by itself lacks other useful DLR rows for
  this sample (`E24`, `E33`, and BDS rows), leaving `466` strict no-match rows
  and worsening the bridge comparison.  Combining all three files (2025-03-31
  BKG BRDC, sample 2025-04-01 DLR BRDM, and 2025-04-01 BKG BRDC) removes
  `ssr_orbit_iode_no_matching_ephemeris` entirely: strict orbit/clock
  applications reach the default `9,665` rows and valid correction rows reach
  `6,774`.  The strict trajectory improves to `0.429 m` horizontal / `0.890 m`
  up / `0.988 m` 3D all-window RMS and `0.084 m` / `0.890 m` / `0.893 m` tail
  RMS.  However, the same three-nav run without IODE enforcement reports
  `0.483 m` / `0.454 m` / `0.663 m` all-window and `0.219 m` / `0.235 m` /
  `0.321 m` tail RMS, while the best default single-day DLR BRDM run remains
  better at `0.459 m` / `0.418 m` / `0.621 m` all-window and `0.122 m` /
  `0.189 m` / `0.224 m` tail.  Therefore complete IODE admission is now proven
  to be data-achievable, but it is not yet the best native profile; the remaining
  strict gap is an IODE-matched broadcast-state/SSR application behavior issue,
  especially vertical, not a missing-ephemeris issue.

  A no-IFLC `--ppp-residual-log` rerun of the same three-nav default/strict
  profiles keeps the correction-log counts identical (`12,096` rows and `3,394`
  atmospheric ionosphere constraints in both runs) and reproduces the relative
  default-vs-strict separation (`1.004 m` 3D delta RMS, dominated by
  `0.978 m` up).  The correction-log SSR vectors themselves differ only at
  floating-point noise scale, while the residual log shows the effect entering
  through predicted measurement geometry after broadcast IODE selection.  At
  the max-separation epoch `TOW 173400`, strict switches `32` valid observation
  rows to the SSR-matched broadcast IODE; initial-iteration code residuals move
  by roughly `1.8-2.6 m` on the affected GPS/Galileo/QZSS rows such as `J02`
  (`189 -> 193`), `G26` (`25 -> 28`), `G08` (`14 -> 82`), `E04/E26/E31`
  (`34 -> 33`), and `E06` (`35 -> 33`).  BDS rows with unchanged IODE move
  too because the common receiver/clock/ionosphere state absorbs the changed
  geometry.  The next parity target is therefore to compare the native
  SSR-matched broadcast satellite state and MADOCALIB's broadcast-state
  selection/application directly, not to fetch more navigation data.

  That state comparison found and fixed a smaller Galileo broadcast-state
  mismatch: native `computeBroadcastState()` was using the GPS gravitational
  constant for Galileo, while MADOCALIB/RTKLIB `eph2pos()` uses
  `MU_GAL=3.986004418E14`.  After adding the Galileo constant, the direct
  three-nav state probe at `TOW 173400` matches MADOCALIB parity at position
  level for the previously affected `E04`/`E06`/`E23`/`E26`/`E31` IODs
  (`pos_delta_m=0.0`, with only finite-difference velocity noise).  The full
  MIZU no-IFLC rerun shows this was not the main strict vertical issue: default
  moves only `0.035 m` 3D RMS versus the previous three-nav default, strict
  moves `0.099 m` 3D RMS versus the previous strict run, and the new
  default-vs-strict split is still `1.080 m` 3D RMS (`1.061 m` up) overall and
  `0.845 m` 3D RMS (`0.815 m` up) in the final 30-minute window.  The new
  bridge comparisons are `0.480 m` horizontal / `0.463 m` up / `0.667 m` 3D
  for default and `0.435 m` / `0.975 m` / `1.068 m` for strict.  Therefore the
  remaining strict gap is downstream of basic broadcast state constants, likely
  in how IODE-matched broadcast geometry, SSR orbit/clock application, and the
  estimated ionosphere/clock states interact.

  A follow-up filter-state diagnostic now logs per-iteration ECEF position
  updates, post-update position state, and constellation clock states.  In the
  same three-nav/no-IFLC MIZU default-vs-strict rerun, the final-iteration
  position-state split exactly reproduces the solution split: `0.201 m`
  horizontal / `1.061 m` up / `1.080 m` 3D RMS, with the max at `TOW 173400`
  (`0.103 m` horizontal, `1.712 m` up, `1.715 m` 3D).  The per-iteration
  update split is tiny by then (`0.0012 m` 3D at `TOW 173400`; `0.0076 m` 3D
  RMS over final iterations), so the meter-level vertical difference is not
  created by one late linearized update.  It accumulates epoch-to-epoch from
  early IODE-selected geometry differences while the native MADOCA profile keeps
  `reset_kinematic_position_to_spp_each_epoch=false`: the pre-update Up split is
  zero at `TOW 172800`, `0.199 m` by `TOW 172950`, `0.853 m` by `TOW 173040`,
  and `1.709 m` by `TOW 173400`.  Clock states absorb a comparable common-mode
  shift (`~1.9-2.0 m` GPS/BDS at `TOW 173400`) and trop moves by `-0.429 m`,
  while ionosphere RMS changes only millimeters.

  The corrected-satellite-position/LOS probe narrows this further.  The
  correction log now includes the receiver ECEF used during correction
  application, final corrected satellite ECEF, satellite clock bias in meters,
  geometric range, and LOS unit vector.  Comparing the three-nav default and
  strict runs shows that the correction vectors are still unchanged at
  floating-point scale; the difference comes from broadcast state/clock selected
  by the SSR IODE.  The largest satellite-only predicted-measurement change is
  `J02` at `TOW 172920`: IODE `189 -> 193`, satellite ECEF split `1.405 m`, LOS
  projection `1.404 m`, satellite-clock split `0.230 m`, and net
  range-minus-clock change `1.174 m` at `87.9 deg` elevation.  Other persistent
  contributors are `G27` (`0.433 m` RMS satellite-only), `G26` (`0.387 m`),
  `E06` (`0.359 m`), and `E23` (`0.356 m`), with `J02` dominating at
  `0.750 m` RMS over its changed rows.  By `TOW 173400`, however, the direct
  satellite-only changes are smaller (`0.158 m` RMS over IODE-changed valid
  rows, max `0.313 m`), while the actual geometric-plus-satellite-clock split
  is `1.185 m` RMS and is dominated by the accumulated receiver-state component
  (`-1.12 m` mean).  The remaining parity target is therefore not the SSR
  correction vector but the broadcast-state/clock convention used for SSR IODE
  matching, especially high-elevation QZSS rows that seed vertical state early
  and are then carried forward by the no-SPP-reset kinematic MADOCA profile.

  A direct parity probe confirms that this is an ephemeris-selection issue, not
  a broadcast propagation formula issue.  For the same broadcast ephemeris,
  native `Ephemeris::calculateSatelliteState()` and the parity `eph2pos()`
  agree at floating-point scale for the representative J02/G26/G27/E06/E23
  rows.  The parity `satpos()` helper, however, calls broadcast selection with
  `iode=-1`, so it does not enforce the SSR orbit IODE.  At `TOW 172920` and
  `173400`, J02 selects IODE `189@172800` while native strict selects SSR IODE
  `193@176400`; at `TOW 173400`, G26 selects `25@172800` instead of strict
  `28@180000`, and G27 selects `84@172800` instead of strict `85@180000`.
  Galileo adds another RTKLIB/MADOCALIB convention: `satpos()` ignores toe
  values at or after `teph`, so E06 at `TOW 173400` selects `34@172800` rather
  than native default `35@173400` or strict `33@172200`, while E23 selects
  `31@171000`, matching the strict SSR IODE case.

  Native PPP now has this as an explicit selection mode:
  `NavigationData::getRtklibEphemeris()` mirrors the non-IODE `satpos()`
  selection rule, and native MADOCA enables it while keeping
  `--enforce-ssr-orbit-iode` as a separate strict diagnostic override.  On the
  same three-nav/no-IFLC MIZU default profile, the bridge comparison changes
  from `0.480 m` horizontal / `0.463 m` up / `0.667 m` 3D RMS to `0.492 m` /
  `0.442 m` / `0.661 m`; the final 30-minute window improves more clearly from
  `0.210 m` / `0.203 m` / `0.292 m` to `0.202 m` / `0.066 m` / `0.212 m`.
  Correction-log admission is unchanged (`6,774` valid rows and `9,665`
  orbit/clock applications), but `1,454` rows switch broadcast IODE, mostly
  Galileo (`E23`: `178`, `E09`/`E33`: `140`, and `E04`/`E05`/`E06`/`E26`/
  `E31`: `120` each).  The strict IODE profile remains effectively unchanged
  against the bridge (`0.435 m` / `0.975 m` / `1.068 m`, tail `0.072 m` /
  `1.012 m` / `1.015 m`), confirming that strict SSR IODE remains a diagnostic
  path rather than the current bridge-comparison default.

  A logging rerun of the RTKLIB-selection default profile produces the same
  position series, so the remaining all-window bridge delta is a filter
  transient rather than instrumentation.  The largest bridge split is still the
  first matched epoch (`TOW 172860`, `2.556 m` 3D): MADOCALIB reports status `6`
  with `10` satellites, while native reports status `5` with `27` satellites
  after two native warm-up epochs.  Native residuals are already settling by
  then (`1.352 m` final-iteration code RMS, `0.760 m` phase RMS, `0.272 m`
  position update), and the next few epochs decay quickly (`2.202 m`, `1.794 m`,
  `1.162 m`, `1.011 m` 3D against the bridge through `TOW 173040`).  This makes
  the remaining all-window error primarily an initialization/solution-convention
  mismatch at the bridge boundary, not a new SSR correction-selection gap.

- Additional early-transient experiments were tested and rejected.  Initializing
  carrier ambiguity states with the current estimated ionosphere phase term and
  delaying ambiguity-state creation until the phase lock gate is met is
  defensible in isolation, but it slightly worsened this MIZU parity run to
  `0.459 m` horizontal / `0.423 m` up / `0.624 m` 3D RMS over all matched
  epochs and `0.122 m` horizontal / `0.212 m` up / `0.244 m` 3D in the final
  30-minute window.  Cutting the native RINEX input to start at the bridge's
  first TOW `172860` remained worse after the RTKLIB-selection fix: the first
  epoch jumps to `10.082 m` 3D, the all-window comparison is `0.792 m`
  horizontal / `0.832 m` up / `1.148 m` 3D, and only the final 30-minute window
  improves to `0.113 m` / `0.075 m` / `0.136 m`.  The cut run also differs from
  the full-start native run by `7.720 m` 3D at `TOW 172860`, proving that the
  two pre-bridge native epochs are useful warm-up rather than a removable source
  of mismatch.  Raising native MADOCA filter iterations from `8` to `16` also
  worsened all-window 3D to `0.821 m` and final-window 3D to `0.259 m`.  Adding
  kinematic position process noise in this no-dynamics MADOCA profile was
  unstable: both `100.0` and `0.01` settings diverged by tens of kilometers or
  more.  Restricting the run to `--madoca-navsys 29` reduced the observation mix
  but degraded parity to `0.521 m` horizontal / `0.661 m` up / `0.842 m` 3D
  all-window and `0.186 m` horizontal / `0.734 m` up / `0.758 m` 3D in the
  final window.  The current best native profile therefore keeps the original
  epoch window, `filter_iterations=8`, zero kinematic position process noise,
  and `--madoca-navsys 61`.

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

## Roadmap

Phase 0, iter1 foundation:

- Add docs and compile-time skeleton files only.
- Do not change PPP behavior.
- Do not touch CLAS code.

Phase 1, helper parity:

- Port pure helpers with stable inputs first:
  bit extraction expectations, system ID mapping, signal mask to code mapping,
  update interval selection, and bias-code selection.
- Add MADOCALIB oracle calls or generated fixture rows.
- Use `1e-6` tolerance for scaled metric outputs.

Phase 2, L6E decode:

- Implement MADOCA L6E stream/frame assembly.  Done for the native foundation.
- Decode subtypes 1, 2, 3, 4, 5, and 7 into a neutral correction epoch.
  Done for the native L6E foundation.
- Compare decoded rows against MADOCALIB `cssr2ssr` or direct oracle output.
  Done for MIZU sample orbit/clock, L6E code/phase bias, and URA rows through
  the direct MADOCALIB oracle.
- Map decoded rows to `SSRProducts`.  Done for L6E orbit/clock/bias/URA rows.

Phase 3, L6D ionosphere:

- Implement coverage and correction message parsing.  Done for native L6D.
- Implement area selection and STEC delay/std calculation.  Done through the
  native `miono_get_corr` parity helper.
- Add sample-driven tests for PRNs 200/201.  Synthetic frame coverage exists,
  and the opt-in MADOCALIB oracle test now compares the final native L6D
  result for each MIZU sample epoch against MADOCALIB `input_qzssl6df()` plus
  `miono_get_corr()`.

Phase 4, PPP application:

- Wire MADOCA correction products into the existing PPP pipeline.  Done for
  native L6E and L6D supplemental ionosphere.
- Keep CLAS and MADOCA profiles explicit.
- Compare sample `exec_ppp`, `exec_pppar`, and `exec_pppar_ion` windows against
  MADOCALIB output.  Use `--ppp-correction-log` on native runs to separate
  correction ingestion/application gaps from filter, AR, and multifrequency
  modeling gaps.

Phase 5, PPP-AR and multifrequency:

- Audit MADOCALIB 2.0 triple/quad-frequency PPP-AR behavior.  Started from
  `ppp.c`/`ppp_ar.c`: MADOCALIB uses `NF(opt)=opt->nf` for estimated-ionosphere
  PPP, adds constellation receiver-clock states plus 3rd/4th-frequency receiver
  bias states (`I3`/`I4`) for code rows, and keeps per-frequency phase-bias
  states `IB(sat,f)`.  Its AR path requires estimated ionosphere, then fixes
  extra-wide-lane ambiguities (`F2-F3`, `F2-F4`) when `nf>=3`, wide-lane
  ambiguities (`F1-F2`) when `nf>=2`, and finally N1 narrow-lane ambiguities by
  LAMBDA/PAR.  WL/EWL pre-fixes use `0.20 cycle` fraction and `1.00 cycle`
  standard-deviation gates; N1 LAMBDA inflates the ratio threshold for small
  ambiguity counts and can relax or disable fixing based on first/second
  candidate match rate.  Fixed-ambiguity constraints are usually `0.01 m`, but
  are intentionally de-weighted for BDS, BDS B2a, GAL E6, and GPS/BDS EWL rows.
  Native currently has only the L1-primary plus secondary-code parity step and a
  narrower WLNL helper, so the next implementation unit is explicit
  per-frequency phase-bias state integration before enabling any broad CLI mode.
- Add fixture-level AR tests before enabling broad CLI behavior.  Started with
  MADOCALIB/CLAS ratio-threshold table coverage and a direct-DD fixture proving
  that `use_clas_osr_filter` raises the required ratio and rejects an otherwise
  fixable ambiguity set.  The MADOCALIB-style design-row helper now also fixes
  N1, WL (`F1-F2`), and EWL (`F2-F3`) coefficient signs plus `D' x` and
  `D' P D` evaluation on a small fixture.  `PPPState` now has an explicit
  `(satellite,frequency_index)->state_index` table and `ppp_ar` has a provider
  adapter so the MADOCA AR rows can target per-frequency ambiguity states
  without changing the default per-satellite PPP ambiguity path yet.  The
  standard PPP path now has a default-off
  `enable_per_frequency_phase_bias_states` guard that keeps existing L1-primary
  no-IFLC behavior unchanged, but can allocate secondary-frequency carrier
  phase rows and ambiguity states for estimated-ionosphere parity experiments.
  The `gnss_ppp` CLI exposes this as
  `--enable-per-frequency-phase-bias-states`/
  `--disable-per-frequency-phase-bias-states`, and reports the selected
  mode in stdout plus `per_frequency_phase_bias_states` summary JSON.
  The same CLI also exposes `--disable-ppp-outlier-detection`/
  `--enable-ppp-outlier-detection` so short parity runs can inspect all
  raw multifrequency rows before residual rejection.  For the MADOCA
  estimated-ionosphere parity runs, `gnss_ppp` now also exposes
  `--kinematic-preconvergence-phase-residual-floor <meters>`; the
  library default remains `200 m` to preserve existing generic kinematic
  PPP behavior while allowing scoped MADOCA admission sweeps.
- Smoke-ran the MADOCALIB sample MIZU 2025-04-01 00:00:00-00:09:30
  window with native `gnss_ppp --madoca-l6e` using PRNs 204/206,
  `--enable-per-frequency-phase-bias-states`, and
  `--disable-ppp-outlier-detection`.  The 20-epoch native run produced
  `20/20` valid float solutions, loaded `1430` L6E epochs and `75135`
  native corrections across `106` satellites, and wrote `2040` correction
  rows, `160` filter rows, and `15040` residual rows.  The new
  `scripts/analysis/ppp_filter_log_summary.py` report showed max row shape
  `code=50, phase=50, iono=0`, with dominant shapes
  `code=48, phase=48` (`104` iterations) and `code=50, phase=50` (`32`
  iterations).  The same helper can now compare two filter logs with
  `--compare-filter-log`, reporting common/missing rows, first changed
  epoch/iteration rows, and max per-column deltas.  The residual log now
  carries signal, observation-code,
  frequency-index, and ionosphere-coefficient metadata so repeated per-satellite
  rows can be identified.  Across all filter iterations the largest transient
  residuals are GLONASS `R21`/`R09`, especially secondary-frequency `C2C`
  rows, but the final-iteration subset is `1880` rows with max residuals
  `4.832 m` code and `3.397 m` phase.  Its top code rows are `G09 C2W f=1`,
  `G26 C2W f=1`, and `C07 C2I/C7I f=0`; its top phase rows are
  `G09 C1C/C2W f=0`, `C09 C2I/C7I f=0`, and `G04 C2W f=1`.  Joining residuals
  to `--ppp-correction-log` on epoch/satellite/signal/code matched all `15040`
  rows (`1880/1880` final-iteration rows), so these top residuals are not caused
  by missing correction diagnostics.  The final `G09 C2W f=1` code row carries
  SSR/orbit-clock valid flags, clock `-0.331 m`, code bias `3.340 m`, phase bias
  `-0.437 m`, and variance PR `0.931 m^2`.  A follow-up RTKLIB/native audit
  confirmed the intended no-IFLC bias semantics: raw secondary-frequency rows
  use their own SSR signal bias directly, while ionosphere-free rows still use
  the coefficient-combined bias.  The audit did find a GPS P/Y/W ionosphere
  mapping gap: `GPS_L1P`/`GPS_L2P` were missing from STEC-to-meters conversion,
  so selected `C2W/L2W` rows had zero STEC correction diagnostics/direct
  corrections when STEC was available even though their estimated-ionosphere
  design coefficient was present.  A short L6E+L6D rerun confirms `G09 C2W`
  now carries non-zero `iono_m` (`18.592 m` at TOW `172860` with
  `69.523 TECU`).  `PPPAtmosphereTest.MapsGpsPYSignalsToCarrierFrequencies`
  and `PPPTest.ProcessorAppliesRawSecondaryBiasesToEstimatedIonosphereRows`
  cover the corrected behavior.  The same window through the linked MADOCALIB bridge
  produced `18`
  solution epochs; native-vs-MADOCALIB matched `18` epochs with `3.816 m` 3D
  RMS delta and `4.557 m` max 3D delta, and wrote the matched CSV with
  `--match-csv /tmp/libgnss_madoca_cli/mizu_00_solution_matches.csv`.
- For the L6D estimated-ionosphere PPP-AR sample, matching MADOCALIB's
  `pos1-elmask=10` setting reduced the short-window native/bridge split from
  `2.933 m` to `1.028 m` 3D RMS over the `18` matched epochs.  The elmask-10
  comparison reports `0.460 m` horizontal RMS, `0.919 m` up RMS, `1.431 m` max
  3D delta at TOW `172860`, and no `>=10 m` events.  The generated solution
  diff is
  `/tmp/libgnss_madoca_cli/mizu_00_l6d_pppar_ion_el10_solution_diff.json`.
  `scripts/analysis/madoca_satellite_set_diff.py` now compares MADOCALIB
  `.stat` `$SAT` valid sets against native correction/residual logs and writes
  per-epoch CSV plus JSON reports; `tests/test_madoca_satellite_set_diff.py`
  covers epoch set differences, native correction summaries, phase-capable set
  comparisons, and final-iteration residual classification.  The native PPP
  correction log now carries `frequency_index`, `ionosphere_coefficient`, and
  `has_carrier_phase`, so this report can separate all valid native rows from
  rows that still have a carrier-phase measurement after correction application.
  The report also includes `phase_frequency_overlap`, which compares MADOCALIB
  `$SAT` valid frequency rows in common bridge/native epochs with native
  final-iteration phase identities, and `phase_candidate_summary`, which groups
  skipped phase candidates by satellite/frequency identity, ready state, skip
  reason, first/minimum candidate residual, first/minimum ready residual versus
  the active phase limit, and first/minimum accepted phase residuals.  It also
  reports `initial_phase_admission_summary`, which joins the first
  phase/candidate iteration to the correction log and summarizes accepted versus
  skipped rows by constellation, frequency index, SSR/orbit-clock availability,
  STEC, phase bias, and residual range.
  `scripts/analysis/ppp_residual_row_set_diff.py` compares two residual logs at
  `(epoch, iteration, row_type, satellite, signal, frequency_index)` granularity;
  with `--explain-phase-candidates`, it also explains base-only phase rows
  using candidate-side `phase_candidate` diagnostics and skip reasons.
  `tests/test_ppp_residual_row_set_diff.py` covers the helper and CLI report.
  `scripts/analysis/ppp_phase_contribution_diff.py` compares selected
  residual-log rows from two PPP residual logs over repeated `name:start:end`
  windows and an optional iteration filter.  The default `--row-type phase`
  compares accepted carrier-phase rows, while `--row-type code` reuses the same
  contribution audit for code rows.  It aggregates row counts, distinct epochs,
  signed ECEF position update sums, signed position-update norm, absolute 3D
  position contribution, receiver-clock/ionosphere/ambiguity update sums, and
  residual absolute statistics for totals and for exact
  `(sat, frequency_index, signal, code, ionosphere_coefficient)` identities.
  The JSON/text report is intended for deciding whether an admission change
  moves the position update directly or mostly perturbs latent ambiguity,
  receiver-clock, and STEC states through common rows.  When the residual log
  has the diagnostic columns, the same report also totals innovation variance,
  `S`/`S^-1` code/phase/ionosphere-constraint coupling, and absolute Kalman
  gain sums for position, receiver clock, ionosphere, and ambiguity states.
  `tests/test_ppp_phase_contribution_diff.py` covers the windowed helper plus
  phase, code-row, optional coupling/gain, and CLI/JSON paths.
  `--ppp-residual-log` now also carries phase-admission diagnostics:
  `phase_candidate`, `phase_accepted`, `phase_ready`, `phase_skip_reason`,
  `ambiguity_state_index`, `ambiguity_lock_count`, `required_lock_count`, and
  `phase_limit_m`.
  On the same elmask-10 L6D run, bridge-only satellite appearances drop to `0`,
  while native-only appearances remain `200` across the `18` matched epochs.
  A current-binary diagnostic rerun with the new correction-log columns shows
  the same `200` native-only appearances even when filtering native sets to
  `has_carrier_phase=1`, so the mismatch is not just secondary code-only rows.
  The persistent native-only set is led by Galileo `E04/E05/E06/E26/E31` (`18`
  epochs each), followed by BDS `C37` (`14`) and `C36` (`12`).  Native-only
  valid correction rows total `400` rows over `22` satellites: BDS/GLO extras
  have no STEC constraints, while Galileo `E04/E05/E26/E31` carry STEC and
  `E06` does not.  In the phase-column rerun, native-only final-iteration rows
  include code, phase, and ionosphere-constraint diagnostics; native-only phase
  RMS is `0.338 m` (BDS), `0.190 m` (Galileo), `0.517 m` (GPS), and `0.310 m`
  (GLONASS), with top phase maxima at `C16`, `E31`, and `C36`.  The phase-state
  toggle is now separated from satellite admission: enabling
  `--enable-per-frequency-phase-bias-states` on the current binary adds the
  secondary-frequency phase rows and worsens the matched solution split to
  `3.014 m` 3D RMS, while `--disable-per-frequency-phase-bias-states` improves
  the current binary to `1.378 m` 3D RMS but still does not reproduce the older
  `1.028 m` baseline.  Comparing the older elmask-10 residual log against the
  current no-per-frequency residual log shows the first phase-row set split at
  TOW `172830`, iteration `0`: older `30` rows, current `27`, `16` common,
  `14` older-only, and `11` current-only.  Across all phase iterations in the
  short window, the older/current no-per-frequency logs have `4949`/`4079` rows,
  `2758` common rows, `2191` older-only rows, and `1321` current-only rows.  The
  first split is a row-admission/lock-count difference: older-only rows are
  secondary-frequency phase rows for `G04/G08/G09/G16/G26/G27/C07/C09/C10/C16`
  plus `C36/C39/C40/J02`, while current-only rows are primary-frequency rows for
  `G28/G31/R01/R09/R19/R21/C28/C29/C37/J03/J04`.  A phase-admission diagnostic
  rerun of the current no-per-frequency setup keeps the accepted phase row set
  unchanged and adds `817` skipped `phase_candidate` rows: `545` rejected by the
  phase outlier gate and `272` skipped by lock count.  At the first split
  (`TOW 172830`, iteration `0`), the only skipped candidates are Galileo
  `E04/E05/E06/E26/E31` with `lock=0`, `required=1`; the current-only primary
  rows above are already `phase_ready=1` with `lock=2`, `required=1`, and pass
  the kinematic pre-convergence `200 m` phase gate.  Scoped reruns with
  per-frequency phase states and a lower pre-convergence floor show that
  the gate is material but not the whole row-admission story.  With the
  guarded ionosphere-aware seed, the short-window sweep is sharply centered on
  `18 m`: `15/16/17 m` give `1.921/1.880/1.865 m` matched 3D delta RMS,
  because they drop too many older/common final-iteration phase rows and add
  no candidate-only rows; `19/20 m` give `1.156/1.155 m` by admitting too many
  candidate-only rows; `18 m` remains the best short-window candidate at
  `0.900 m` matched 3D delta RMS (`0.565 m` horizontal, `0.701 m` up,
  `0.658 m` matched reference 3D RMS).  At final iteration, the corresponding
  old-vs-candidate phase row-set deltas are: `15 m` common/base-only/
  candidate-only `613/88/0`, `16 m` `633/68/0`, `17 m` `653/48/0`, `18 m`
  `652/49/35`, `19 m` `683/18/55`, and `20 m` `683/18/71`.  Against the older elmask-10
  phase log at iteration `0`, the `18 m` run has `42` older rows, `40`
  candidate rows, `36` common rows, `6` older-only rows, and `4`
  candidate-only rows; across all phase iterations it still
  differs by `349` older-only and `249` candidate-only rows.  The new
  phase-candidate explanation report accounts for all `349` older-only rows
  as candidate-side `outlier` skips, led by `E06`, `E31`, and `C28`; no
  older-only rows lack a candidate diagnostic.  The remaining candidate-only
  accepted phase rows are led by `J03` and `C30`.  A follow-up frequency-level
  bridge audit shows those additions are not the narrow rows to remove:
  MADOCALIB marks `J03` valid in `18` bridge epochs while the older native log
  has no final-iteration `J03` phase rows and the guarded `18 m` run has `19`
  native epochs; `C30` is bridge-valid in `12` epochs, while both native runs
  keep the primary `C2I/C6I` phase identity for `19` epochs and the guarded run
  raises the secondary `C6I` identity from `3` to `19` epochs.  The sharper
  mismatch is the opposite: bridge-valid BDS `C28/C29` remain under-admitted
  (`C28` bridge-valid `10` epochs, older native primary `13`, guarded primary
  `2`; `C29` bridge-valid `12`, older native primary `1`, guarded `0`), while
  Galileo `E31` remains over-admitted on the primary identity even though
  MADOCALIB has no valid `E31` bridge epochs.  Correction-log inspection shows
  this is not explained by a simple no-STEC rule: `C30` has no L6D STEC
  constraint, but `J03` does carry STEC.  A direct L6D raw-payload audit of the
  same `200/201` streams found no sys-3 BDS MIONO correction records at all,
  while Galileo/QZSS records are present (`E31`/`J03`).  The MADOCALIB `$ION`
  rows seen for `C28/C29/C30` are PPP estimated ionosphere states printed by
  `pppoutstat()`, not external L6D STEC rows.  A synthetic `C28` unit case now
  confirms that BDS3 MIONO materialization works when a decoded MIONO result
  contains a BDS3 satellite.  Therefore the BDS gap is not a BDS3 STEC
  frequency-conversion bug; the next check is why MADOCALIB phase admission can
  keep bridge-valid BDS rows without L6D STEC while this PPP gate rejects
  `C28/C29`.  A targeted phase-candidate audit now shows this is an admission
  gate issue rather than missing observations: correction logs keep carrier
  phase, SSR orbit/clock, and valid correction rows for `C28/C29`, but the
  guarded `18 m` phase gate skips the first ready `C28` split at about
  `20/28 m` and `C29` at about `25/38 m`, while `C30` enters at about
  `-3/-4 m`.  `C29` later falls to about `18.4 m`, just above the current
  floor.  The common-epoch `phase_frequency_overlap`/`phase_candidate_summary`
  report also shows this is a broader phase-row policy mismatch, not just a BDS
  issue.  In the early 18-epoch diagnostic, `J04` and `G31` are bridge-valid but
  have no native accepted phase rows (`G31` is just over the gate, and `J04`
  starts around `30/45 m`), while `E26/E31` enter native phase before they appear
  in MADOCALIB `$SAT`.  A full 120-epoch native rerun against the 118-row
  MADOCALIB bridge confirms that the early Galileo mismatch is mostly timing:
  `E26/E31` later become bridge-valid for `93` common epochs, but native keeps
  them for `118`.  The larger full-window phase gaps are GLONASS/QZSS/GPS
  admission differences (`R21`/`R09` bridge-valid with no native phase rows, and
  `J03`/`G31` delayed until about TOW `173790`).  The enhanced candidate summary
  now exposes the missed first-epoch primary rows directly: `R21`/`R09` have
  `17.888/15.850 m` primary residuals at TOW `172800` but `lock_count=0`, then
  first-ready residuals jump to `42.705/48.290 m`; `G31`/`J03` show the same
  pattern with `7.684/11.386 m` primary residuals at TOW `172800` and delayed
  acceptance around TOW `173790`.  The full-window matched 3D delta RMS is
  `0.604 m` overall and `0.344 m` over the tail 1800 s, so the next fix should
  target constellation-specific ambiguity/admission parity rather than a global
  floor increase.  MADOCALIB's configured `pos2-rejionno=50` is broader, but
  the native `20 m` sweep worsened matched 3D RMS.  Allowing the non-default
  `--phase-measurement-min-lock-count 0` experiment admits those first primary
  rows and improves the full-window matched 3D delta RMS to `0.479 m` and the
  tail 1800 s to `0.160 m`, but it also over-admits native phase rows
  (`6446 -> 7040`; `R21` `120` native epochs vs `115` bridge, `G31/J03` `120`
  vs `118`, and Galileo `E06/E26/E31` `119` vs `93`) while secondary GLONASS
  remains over the gate.  A `lock0` run with Galileo excluded
  (`--madoca-navsys 53`) also worsens the split (`0.717 m` full-window matched
  3D, `0.510 m` tail 1800 s), so the improvement is not recovered by
  non-Galileo lock bypass alone.  Therefore lock `0` is useful evidence but not
  the default parity fix.  A narrower non-default primary-only warm-start hook
  (`--enable-initial-phase-admission-warm-start`) was then tested for
  non-Galileo SSR/MADOCA primary rows with low lock-0 residuals.  It reduces
  native phase rows relative to lock `0` (`6819` instead of `7040`) and avoids
  the broad Galileo secondary warm-start, but it worsens the full-window matched
  3D delta RMS to `0.805 m` and the tail 1800 s to `0.524 m`.  That rejects a
  simple first-primary lock bypass as the parity fix.  A second non-default
  all-frequency warm-start hook
  (`--enable-all-frequency-initial-phase-admission-warm-start`) admits every
  low-residual initial frequency before the lock gate while leaving the normal
  lock gate in place afterward; it exactly reproduces the full lock `0` phase
  row set (`49447/49447` common phase rows, no row-set deltas) and the same
  solution split (`0.479 m` full-window matched 3D, `0.160 m` tail 1800 s).
  Therefore the useful lock `0` behavior is specifically the first-epoch
  all-frequency phase update, not ongoing lock-gate removal.  Because that row
  set still over-admits native phase against MADOCALIB, the next target is to
  constrain the initial all-frequency admission set with MADOCALIB-like
  satellite/frequency policy instead of weakening the global lock gate.  A
  direct first-bridge `$SAT` filter is too blunt: the all-frequency warm-start
  admits `36` rows at native TOW `172800` across `22` satellites
  (`C:16/G:13/J:3/R:4` rows), while the first MADOCALIB valid `$SAT` epoch is
  TOW `172860` with only `10` GPS/QZSS satellites.  Only `8` of the warm-start
  satellites and about `15/36` primary/secondary rows match that first bridge
  epoch under the native `i0/i1` to bridge `f1/f2` mapping, and there are no
  native accepted phase rows exactly at TOW `172860`.  The follow-up
  `initial_phase_admission_summary` check narrows this further: the default
  lock-`1` run has `54` first-epoch phase candidates, all skipped by
  `lock_count`, and all `54` already have SSR/orbit-clock corrections.  The
  all-frequency warm-start run accepts `36` of those rows and skips `18`; both
  groups still have complete SSR/orbit-clock availability, no non-zero phase
  bias, and the same atmosphere-token source.  The accepted set is exactly the
  subset whose baseline candidate residual is within the `18 m` gate
  (`1.111..17.909 m`), while the skipped subset starts at `18.696 m` and runs
  to `54.117 m`.  Product availability alone is therefore not the discriminator.
  Comparing the lock-`1` and all-frequency warm-start filter logs confirms the
  first-epoch phase update is not a harmless row-shape change: both logs have
  `960` common filter rows and no missing rows, but every common row changes.
  The first iteration moves from `64` to `100` rows (`phase_rows 0 -> 36`), with
  immediate deltas in GLONASS clock state (`-17.415 m`), max ionosphere update
  (`+10.197 m`), and position (`+2.236/-2.221/-0.688 m` in ECEF components).
  The largest column deltas over the run are still seeded by that split or its
  consequences: `phase_rows/rows +36`, `phase_residual_max_abs_m -23.111 m`,
  `phase_residual_rms_m -22.125 m`, and GLO clock state `-17.415 m`.  Comparing
  all-frequency warm-start against full lock `0` gives `960/960` unchanged
  filter rows, so the diagnostic hook truly reproduces lock `0`.  Constellation
  subset warm-start sweeps show no single system reproduces the full `36`-row
  benefit.  GPS-only admits `13` rows and worsens the solution split to
  `1.453 m` full-window / `0.827 m` tail 1800 s.  GLONASS-only admits `4` rows,
  nearly reproduces the first-iteration GLO clock-state delta (`-17.065 m` vs
  `-17.415 m` full warm-start), and gives `0.586 m` / `0.307 m`.  BDS-only
  admits `16` rows and gives `0.554 m` / `0.383 m`, with the expected BDS3 clock
  first-iteration delta (`-2.754 m`).  QZSS-only admits `3` rows and gives a
  poor full-window split (`1.118 m`) but the best tail (`0.111 m`).  Therefore
  GLO and BDS carry most of the full-window improvement, QZSS mainly affects
  late convergence, and GPS-only is not a parity direction.  The remaining
  target is the residual-gated first-epoch phase update and its
  signal/frequency timing, not a same-epoch bridge `$SAT` set.  A narrower
  selector sweep confirms the BDS effect is distributed rather than a single
  frequency or generation: BDS f0/f1 admit `9`/`7` rows and give `0.563 m` /
  `0.540 m` full-window matched 3D RMS (`0.370 m` / `0.368 m` tail), while
  BDS2/BDS3 admit `8`/`8` rows and give `0.544 m` / `0.554 m` full-window
  (`0.344 m` / `0.380 m` tail).  The BDS3 split carries the expected first
  BDS3 clock-state jump (`-2.737 m`), but BDS2 gives the best tail among the
  BDS-only splits.  GLONASS splits are more asymmetric in time: R09/R21 admit
  `2` rows and have poor full-window RMS (`0.875 m`) but strong tail
  (`0.137 m`), while R01/R19 also admit `2` rows and give `0.665 m` /
  `0.285 m`; both pairs create roughly half of the full GLONASS first-epoch
  clock-state jump (`-7.757 m` and `-8.410 m`).  Therefore the useful
  first-epoch update is a coupled GLO/BDS clock/iono conditioning effect, not
  a single satellite or frequency row to hard-code.  The PPP residual log now
  includes row-level receiver-clock, ionosphere, and ambiguity state indexes,
  design coefficients, and per-row Kalman update contributions.  A one-epoch
  lock-`1` versus all-frequency warm-start rerun shows the admitted `36` phase
  rows have near-zero post-initialization residuals (max about
  `7.5e-9 m`) and zero direct clock/iono update contribution.  The
  first-iteration clock-state shift instead comes from changed code-row update
  contributions after the zero-residual phase rows alter the innovation
  geometry: GLO clock contribution changes by `-18.033 m`, BDS2 by
  `-2.698 m`, BDS3 by `-3.115 m`, and GPS/QZSS by only `-0.458 m`; the
  corresponding filter deltas move GLO from `3307.887 m` to `3289.494 m` and
  raise max ionosphere update from `6.335 m` to `16.187 m`.  Therefore the
  parity target is the covariance/constraint effect of admitting zero-residual
  phase rows, not a raw phase-residual push.  The residual log now also records
  `innovation_variance_m2` plus receiver-clock, ionosphere, and ambiguity
  Kalman gains.  Repeating the same one-epoch comparison shows all `54` common
  code rows keep identical diagonal innovation variance (`warm/lock = 1.0`
  for min/mean/max), so the scalar row variance is not the lever.  The gain
  matrix does change: code-row receiver-clock absolute gain sums move from
  `3.80754` to `3.77114` for GLONASS, `3.91514` to `3.89166` for BDS2, and
  `4.74369` to `4.72263` for BDS3, with GPS/QZSS nearly unchanged
  (`3.93362` to `3.92952`).  The largest row-level receiver-clock gain changes
  are `R21 C1C` (`-0.00973`), `R09 C1C` (`-0.00777`), `R21 C2C` (`+0.00655`),
  and `R09 C2C` (`+0.00540`).  The admitted phase rows themselves have nonzero
  gains (max receiver-clock/ionosphere about `0.00145`, ambiguity about
  `0.99998`), but their residuals stay at floating-noise scale, so their direct
  contributions remain effectively zero.  The residual log now also records
  per-row off-diagonal absolute coupling sums for `S` and `S^-1`, split by code,
  phase, and ionosphere-constraint rows.  The coupling rerun confirms the
  mechanism directly: lock-`1` has zero code-to-phase coupling because no phase
  rows are admitted, while warm-start code rows sum to about `5.755e6 m^2` of
  code-to-phase `S` coupling and `0.07325 1/m^2` of code-to-phase `S^-1`
  coupling.  Common code-row `S^-1` diagonal sums also rise from `74.41944` to
  `74.47316` even though the diagonal innovation variance was unchanged.  The
  split is concentrated where the clock gain moved: GLONASS contributes
  `0.00824 1/m^2` of warm code-to-phase `S^-1` coupling with a receiver-clock
  gain-sum delta of `-0.00545`, BDS2 contributes `0.01884` with `-0.00295`, and
  BDS3 contributes `0.02403` with `-0.00227`.  The largest row deltas remain
  `R21/R09 C1C/C2C`, each with nonzero phase coupling in `S^-1`.  The next
  parity target is therefore the off-diagonal innovation-covariance / `S^-1`
  coupling introduced by the zero-residual phase rows, not the diagonal
  innovation variance.  A current-binary selector sweep shows that coupling
  strength alone is not enough for a safe warm-start policy.  The full
  all-frequency warm-start admits `36` first-iteration phase rows and gives
  `0.479 m` full-window / `0.160 m` tail matched 3D RMS against the bridge,
  compared with lock-`1` at `0.545 m` / `0.340 m`.  BDS-only admits `16` rows
  and does not improve the trajectory (`0.554 m` / `0.383 m`), while
  GLONASS-only admits `4` rows and worsens full-window RMS (`0.586 m`) despite
  improving tail (`0.307 m`).  The high-coupling GLO+BDS union admits `20`
  rows and reproduces the first-epoch GLO/BDS clock movement, but the trajectory
  is worse (`0.612 m` / `0.347 m`), and GLO+BDS frequency-0 only is similar
  (`0.622 m` / `0.338 m`).  Adding QZSS to GLO+BDS recovers the full warm-start
  tail (`0.163 m`) but creates a large early excursion (`1.202 m` full-window,
  `2.826 m` max), while GPS+GLO+BDS is worse still (`1.467 m` / `0.840 m`).
  Therefore a navsys/frequency hard selector is not the parity fix: the useful
  behavior is a coupled row-set balance across GPS, GLO, BDS, and QZSS, and the
  next selector has to reason about early trajectory stability, not only
  first-epoch `S^-1` coupling magnitude.  The residual log now also records
  position-state Kalman gains and per-row ECEF position update contributions.
  A first-epoch rerun confirms that accepted zero-residual phase rows still have
  negligible direct position contribution (max below `9e-13 m`); the position
  shift is again code-row gain redistribution.  Relative to lock-`1`, full
  warm-start changes summed code-row position contribution by
  `(+1.114, -0.884, -0.287) m` ECEF, while GLO+BDS gives
  `(+0.823, -0.793, -0.101) m`, GLO+QZS+BDS gives
  `(+0.713, -0.696, -0.089) m`, and GPS+GLO+BDS gives
  `(+1.224, -0.981, -0.298) m`.  The full warm-start split is dominated by
  GPS code rows `(+1.323, -1.371, -0.367) m` and an opposing QZSS code-row
  response `(-1.029, +1.279, +0.181) m`, plus smaller GLO and BDS terms.
  Therefore first-epoch position-delta magnitude alone is not a sufficient
  selector either; the stable case depends on cross-constellation code-row
  cancellation after phase rows perturb `S^-1`.  A row-level warm-start pair
  selector,
  `--initial-phase-admission-warm-start-sat-frequency-pairs <csv>`, now gates
  only the diagnostic first-epoch phase warm-start by exact `sat:frequency`
  identities while preserving the broader PPP satellite selection.  The CLI
  reports the selected pairs in stdout and summary JSON via
  `initial_phase_admission_warm_start_sat_frequency_pairs`.  As a first
  row-level check, the top `13` high-coupling warm-start pairs admitted exactly
  `13` first-epoch phase rows, but the 120-epoch bridge comparison worsened to
  `0.637 m` full-window matched 3D RMS and `0.324 m` tail 1800 s, with a
  `1.353 m` max 3D excursion.  This rejects a simple row-coupling rank
  selector as the parity policy, while keeping the pair selector useful for
  exact row-set experiments.
  A follow-up exact row-set sweep shows the warm-start balance is strongly
  non-linear.  Keeping only GPS+QZSS gives a slightly better full-window RMS
  (`0.467 m`) but loses the tail (`0.356 m`), while adding GLO, BDS, or BDS
  subsets to GPS+QZSS still leaves the tail around `0.371..0.417 m`; the full
  `36`-row set is the only tested all-additive set that recovers the `0.160 m`
  tail.  Leave-one-satellite tests identify rows that improve early RMS when
  removed but usually damage the tail: removing `G09`, `C27`, `J02`, or `G08`
  gives `0.420`, `0.444`, `0.445`, and `0.448 m` full-window 3D RMS, but tail
  RMS remains `0.337..0.352 m`.  `J03` is essential (`1.566 m` full-window /
  `0.846 m` tail when removed), and most single GLO/BDS removals collapse the
  tail to about `0.39..0.43 m`.  The best current Pareto row sets are full
  minus `G09+C27+G08`, which admits `31` rows and gives `0.331 m` full-window
  / `0.209 m` tail / `0.798 m` max 3D, and full minus `G09+C27+J02+G08`,
  which admits `29` rows and gives `0.493 m` full-window / `0.139 m` tail /
  `1.254 m` max 3D.  Refining the best full-window set found one strict
  improvement over the full warm-start: full minus
  `G09:0/G09:1/C27:0/C27:1/G08:0/G26:1/C28:0` admits `29` first-epoch
  phase rows and gives `0.393 m` full-window / `0.148 m` tail / `1.094 m`
  max 3D, improving the full warm-start's `0.479 m` / `0.160 m` / `1.245 m`
  on all three metrics.  Nearby variants show why this was missed by coarser
  selectors: adding `G26:1` removal to the `G09+C27+G08` cut recovers the
  tail (`0.144 m`) but leaves max at `1.163 m`, while adding `C28:0` removal
  lowers the full-window and max split enough to produce the balanced row set.
  A row/bridge audit confirms this is not a direct `$SAT` valid-frequency
  subset rule.  Under the native `i0/i1` to bridge `f1/f2` mapping, removed
  `G09:0/G09:1` and `G26:1` are bridge-valid in all `118/118` bridge epochs,
  `G08:0` is bridge-valid in `116`, `C27:0/C27:1` in `112`, and only `C28:0`
  is a short early bridge row (`10` epochs, TOW `173010..173280`).  The
  mechanism is instead the same first-epoch code-row redistribution seen in
  the all-frequency warm-start diagnostics: full `36` rows versus the `29`-row
  set changes all `960/960` common filter rows, with first-iteration rows
  `100 -> 93` and phase rows `36 -> 29`; BDS3 clock state moves by
  `+1.902 m`, BDS2 by `+0.842 m`, GLO by `+0.569 m`, receiver clock by
  `+0.838 m`, and max ionosphere state drops by `1.245 m`.  Relative to the
  `31`-row `G09+C27+G08` cut, the extra `G26:1/C28:0` removals give the tail
  recovery (`0.209 -> 0.148 m`) while giving back some of the early/full-window
  improvement (`0.331 -> 0.393 m`, max `0.798 -> 1.094 m`), so this remains a
  diagnostic Pareto point rather than a hard-coded parity policy.  A
  current-binary reproduction check gives the same `0.393 m` / `0.148 m` /
  `1.094 m` split and first-iteration `93` rows (`54` code / `29` phase),
  confirming that this `29`-row run uses the standard PPP path, not
  `--clas-osr`; `gnss_ppp` now prints and writes `use_clas_osr_filter`,
  `use_ionosphere_free`, and `estimate_ionosphere` so command reconstruction
  mistakes are visible in logs and summary JSON.
  Combining
  Galileo exclusion
  (`--madoca-navsys 53`) with the `18 m` floor reduces MADOCALIB/native
  native-only satellite appearances from `200` to `110`, but worsens the
  matched 3D delta RMS to `1.454 m`, so constellation exclusion is not the
  parity fix.  Two additional coarse navsys cuts confirm that the remaining
  `J03`/`C30` row differences are not simple remove-this-constellation fixes:
  excluding QZSS (`--madoca-navsys 45`) worsens the `18 m` guarded-seed run to
  `2.657 m` matched 3D delta RMS, and excluding BDS (`--madoca-navsys 29`)
  worsens it to `1.471 m`.  An ionosphere-aware phase-ambiguity seed was added
  behind
  `--enable-ionosphere-aware-phase-ambiguity-init` with summary key
  `ionosphere_aware_phase_ambiguity_init`.  A direct version of that seed
  made the first phase-admission gate too permissive: the `18 m` floor run
  grew from `4849` accepted phase rows to `7354` and worsened the matched
  3D delta RMS from `0.900 m` to `2.806 m`.  The implementation now keeps
  the ionosphere-aware ambiguity value for accepted rows but uses a legacy-seed
  admission offset until the first phase row is admitted, preserving the
  intended pre-convergence residual screen.  With that guarded seed, the
  `18 m` phase row set exactly matches the unseeded `18 m` run (`4849` common,
  no base-only or candidate-only rows), and the matched 3D delta RMS is
  effectively unchanged at `0.900 m`.  The accepted first-split residuals shrink
  (`C30` from `11.634/17.903 m` to `-3.171/-4.520 m`, `J03` from `17.893 m`
  to `6.508 m`).  Final-iteration code rows are identical between the older
  elmask-10 run and the guarded `18 m` run (`1224` common, no row-set deltas),
  so the remaining solution shift is phase-only.  The phase row swap changes
  up error versus MADOCALIB from `+1.003 m` to `-0.377 m` at TOW `172860`, then
  improves the tail by about `0.30 m` (`-1.352 m` to `-1.053 m` at TOW
  `173370`) while slightly increasing horizontal RMS.  Therefore the residual
  seed formula is no longer the narrow target; the remaining target is the
  phase-admission and phase-update policy that lets the bridge-aligned `J03/C30`
  rows move the state while making bridge-aligned `C28/C29` rows fail the
  `18 m` gate and still leaving native-only `E31` primary rows admitted.  The
  existing `PPPConfig::phase_measurement_min_lock_count` is now exposed as
  `--phase-measurement-min-lock-count` with summary key
  `phase_measurement_min_lock_count` for SSR/MADOCA phase-admission sweeps; the
  CLI accepts `0` for explicit experiments while preserving the default `1`.
  The non-default `--enable-initial-phase-admission-warm-start` hook is kept as
  a diagnostic switch for the rejected primary-only warm-start experiment, and
  `--enable-all-frequency-initial-phase-admission-warm-start` captures the
  lock-0-equivalent first-epoch all-frequency update; neither is the native
  default.  The additional diagnostic
  `--initial-phase-admission-warm-start-navsys <mask>` limits those warm-start
  rows to a RTKLIB navsys mask without changing the rest of satellite admission,
  so constellation sensitivity can be tested without the broader
  `--madoca-navsys` side effects.  Two narrower diagnostics,
  `--initial-phase-admission-warm-start-sats <csv>` and
  `--initial-phase-admission-warm-start-frequency-indexes <csv>`, further gate
  only the warm-start rows by satellite ID and frequency index.  A separate
  diagnostic `--phase-admission-exclude-sat-frequency-pairs <csv>` now excludes
  all carrier-phase rows by exact `sat:frequency` and reports the excluded set
  in stdout/summary JSON, with residual skip reason
  `excluded_sat_frequency_pair`.  On the same `29`-row run, pair-exclusion
  sweeps show no simple remove-row fix: excluding `C30:0` gives `0.365 m` /
  `0.161 m` / `0.990 m`, `C30:1` gives `0.384 m` / `0.150 m` / `1.063 m`,
  `E26:0` gives `0.365 m` / `0.154 m` / `1.018 m`, `E31:0` gives `0.408 m`
  / `0.146 m` / `1.131 m`, `C28:0/C29:0/C29:1` is nearly neutral
  (`0.394 m` / `0.148 m` / `1.096 m`), and excluding both `C30` frequencies
  improves full-window/max while losing tail (`0.357 m` / `0.167 m` /
  `0.943 m`).  A narrower time-gated diagnostic,
  `--phase-admission-exclude-sat-frequency-pairs-before <csv>`, now excludes
  exact carrier-phase rows only before a GPS week/TOW threshold and reports the
  thresholds in stdout/summary JSON, with residual skip reason
  `excluded_sat_frequency_pair_before_time`.  On the same no-ANTEX elmask-10
  `29`-row run, the control remains `0.393 m` full-window / `0.148 m` tail /
  `1.094 m` max 3D.  Gating `C30:0/C30:1` before their first bridge-valid
  epoch gives `0.359 m` / `0.177 m` / `0.942 m`, `C30:0` alone gives
  `0.365 m` / `0.165 m` / `0.990 m`, `E26:0` before its first bridge-valid
  epoch gives `0.364 m` / `0.151 m` / `1.018 m`, `E31:0` gives `0.407 m` /
  `0.143 m` / `1.131 m`, `E26:0/E31:0` together give `0.377 m` / `0.149 m` /
  `1.055 m`, and combining `C30:0/C30:1/E26:0/E31:0` gives `0.345 m` /
  `0.178 m` / `0.902 m`.  The pattern mirrors the permanent exclusion sweep:
  C30/E26 reduce full-window and max error, E31 helps the tail, but no tested
  time gate improves all three metrics.  The follow-up diagnostic
  `--reset-phase-ambiguity-on-before-exclusion` resets the affected ambiguity
  state while a before-threshold phase row is being skipped.  On the `C30:0`
  / `C30:1` before-`173040` run this admits `C30:0` immediately at the
  threshold instead of keeping both frequencies out until later iterations, but
  `C30:1` remains rejected and the bridge split is essentially unchanged:
  `0.360 m` full-window / `0.176 m` tail / `0.948 m` max 3D versus
  `0.359 m` / `0.177 m` / `0.942 m` without the reset.  A narrower
  `--phase-admission-residual-floor-sat-frequency-pairs <csv>` diagnostic then
  tested only `C30:1` with a `60 m` phase residual floor, leaving the global
  pre-convergence floor at `18 m`; that admits `C30:1` after the threshold but
  still gives `0.360 m` / `0.177 m` / `0.950 m`.  Raising the global floor to
  `60 m` also admits the row but damages the early window (`1.298 m`
  full-window / `4.780 m` max), so the next target is not a broad threshold
  relaxation.  Additional two-row time-gate sweeps also fail to produce a strict
  all-metric improvement: `C30:0+E26:0` gives `0.339 m` / `0.167 m` /
  `0.912 m`, `C30:0+E31:0` gives `0.378 m` / `0.160 m` / `1.026 m`,
  `C30:1+E26:0` gives `0.356 m` / `0.154 m` / `0.986 m`, and
  `C30:1+E31:0` gives `0.398 m` / `0.146 m` / `1.100 m`.  The last case is
  close on tail, but it remains slightly worse than the `29`-row control on
  full-window RMS and max 3D, and changing the E31 threshold from `173400`
  through `173610` gives identical metrics because E31 primary remains excluded
  or outlier until later.  Therefore the sticky re-entry is not explained by
  stale ambiguity state, a single-frequency floor, or a simple C30/E26/E31
  before-gate combination; the remaining problem is coupled phase
  admission/state update after the threshold.  A side check with receiver/satellite
  ANTEX enabled changes the initial SPP seed and worsens this control to
  `0.544 m` / `0.378 m` / `2.062 m`, so the row-admission sweep remains tied
  to the no-ANTEX comparison condition.  The new phase-contribution comparison
  gives a more specific reading of that result.  Comparing the `C30:0/C30:1`
  reset run against the targeted `C30:1:60` residual-floor run admits `91`
  extra early accepted phase rows, `256` extra tail rows, and `620` rows over
  the full log, almost all from `C30:1`; however the accepted phase-row signed
  position-update norm barely moves (`0.852069 -> 0.850107 m` early and
  `4.01756 -> 4.01839 m` overall).  The added `C30:1` rows instead move
  latent states: over the full log they shift receiver-clock, ionosphere, and
  ambiguity contribution sums by about `-36.2 m`, `+34.5 m`, and `+58.8 m` on
  the `C30:1` identity.  This explains why admitting `C30:1` with a local floor
  does not improve the solution metrics: the direct position contribution is
  small, while the state-conditioning change is large and not bridge-aligned.

  Direct control comparisons also separate the useful and harmful parts of the
  two-row before-gate sweeps.  `C30:0+E26:0` removes `342` accepted early phase
  rows and `448` full-log rows relative to the `29`-row control, dominated by
  fewer `C30:0`, `C30:1`, and `E26:0` rows.  It still improves full-window RMS
  and max 3D (`0.339 m` / `0.912 m`) while worsening the tail (`0.167 m`), so
  the metric gain is not a simple reduction in phase contribution magnitude:
  the accepted phase-row signed position norm actually increases from
  `3.95917 m` to `4.00514 m` over the full log.  `C30:1+E31:0` has the same
  total row loss but trades the Galileo response: full-log ionosphere
  contribution changes by `+15.31 m` versus `+8.19 m` in the `E26` gate, with
  `E31:0` rows dropping `829 -> 619` instead of `E26:0`.  That case recovers a
  better tail (`0.146 m`) but loses full-window and max stability
  (`0.398 m` / `1.100 m`).  A direct `C30:0+E26:0` versus `C30:0+E31:0`
  comparison keeps total phase-row counts identical and mostly swaps early
  `E31:0` rows out for `E26:0` rows (`134 -> 0` and `0 -> 134` in the early
  window).  Therefore the remaining lever is the coupled Galileo STEC/ambiguity
  conditioning after the threshold, not another one-row C30 residual exception.

  The code-row contribution audit confirms the same mechanism under the
  post-threshold windows.  For both `C30:0+E26:0` and `C30:0+E31:0`, the code
  row count is unchanged relative to the `29`-row control in every window
  (`9328` early, `30768` tail, `60576` full), but the common code-row update
  contributions move substantially.  In the `C30:0+E26:0` run the early code
  rows shift receiver-clock and ionosphere contribution sums by `+10.93 m` and
  `+702.10 m`, led by common `C30` code identities (`C30:f1` contributes about
  `-75.13 m` clock and `+357.68 m` ionosphere, while `C30:f0` contributes
  `-44.80 m` and `+218.12 m`).  Over the full log the same run shifts code-row
  clock/ionosphere sums by `+34.72 m` / `+871.26 m`, with `C30` and `E26`
  identities dominating.  In the `C30:0+E31:0` run the early code-row
  ionosphere movement is similarly large (`+685.11 m`) but the full-log signed
  code position norm moves the opposite way (`4.37052 -> 4.24422 m`, delta
  `-0.12631 m`) because `E31` replaces the `E26` response.  Directly comparing
  the `E26` and `E31` two-row gates keeps all code-row counts identical and
  shows the Galileo swap clearly: full-log code ionosphere contribution changes
  by `-44.02 m`, with `E26:f1/E26:f0` moving by `-109.31 m` / `-56.56 m` and
  `E31:f1/E31:f0` moving by `+89.03 m` / `+46.31 m`.  Therefore the current
  C30/E26/E31 behavior is a common code-row gain/state redistribution problem,
  not a missing code row or a direct phase-row position push; signed
  contribution norm alone is also not enough, because the `E26` gate improves
  full/max despite a larger full-log code position norm while the `E31` gate
  lowers that norm but worsens full/max.

  The same code-row audit now includes innovation coupling and gain sums, which
  points to a sharper split.  Relative to the control, both C30 two-row gates
  remove essentially the same large amount of code-to-phase covariance coupling:
  `C30:0+E26:0` changes full-log code-row `S` phase-coupling by about
  `-127091 m^2`, and `C30:0+E31:0` changes it by about `-127092 m^2`.  Their
  direct comparison is only `-0.790 m^2`, so this term is a C30-gate effect,
  not the E26/E31 trade.  The E26/E31 choice instead shows up in inverse
  innovation coupling and gains: direct `C30:0+E26:0` versus `C30:0+E31:0`
  changes full-log code-row `S^-1` phase-coupling by `-928.31 1/m^2`,
  code-coupling by `-4.32 1/m^2`, position-gain abs sum by `+0.32149`,
  clock-gain abs sum by `+0.19426`, and ionosphere-gain abs sum by `+1.2666`.
  That is consistent with the metric trade: C30 controls the large phase
  covariance structure, while the Galileo satellite choice rotates the inverse
  coupling/gain redistribution that decides early stability versus tail.  The
  next audit should localize the update-order or covariance inversion term that
  makes the common C30/Galileo code rows diverge after the bridge-valid
  threshold.

  Current stopping point: the remaining gap is no longer "which rows exist" but
  "why the same common code rows get different `S^-1`/gain structure after the
  threshold".  The immediate follow-up is to rerun the same windows with
  `--row-type code` and a single `--iteration` filter around the first
  bridge-valid epochs, then line those epochs up with the PPP filter log to see
  whether the divergence first appears in covariance inversion, gain assembly,
  or state-update order.  The latest comparison artifacts are the phase-row
  reports `/tmp/madoca_phase_contrib_c30e26_vs_control.json`,
  `/tmp/madoca_phase_contrib_c30e31_vs_control.json`, and
  `/tmp/madoca_phase_contrib_c30e31_vs_c30e26.json`, plus the code-row reports
  `/tmp/madoca_code_contrib_c30e26_vs_control.json`,
  `/tmp/madoca_code_contrib_c30e31_vs_control.json`, and
  `/tmp/madoca_code_contrib_c30e31_vs_c30e26.json`.  Raising the short-window
  `18 m` guarded-seed run to lock `2` is identical to
  the default because the first admitted `C30`/`J03` rows already have lock `2`;
  raising it to lock `3` removes all candidate-only final-iteration rows but
  drops too many older/common phase rows and worsens the matched 3D delta RMS
  to `1.043 m`.  Therefore the global default remains `200 m`, and the `18 m`
  value is kept as a MADOCA parity experiment knob until the remaining
  satellite/STEC/frequency admission and phase-update policy is reconciled.  The
  older elmask-10 trajectory remains the solution-parity baseline; the
  phase-column and
  no-per-frequency reruns are diagnostic artifacts until the current-binary
  row-admission drift is reconciled.  A MADOCALIB-style
  SSR observation-bias availability gate was added as a non-default `PPPConfig`
  diagnostic hook; enabling it for the same native MADOCA run did not explain
  the parity gap and worsened the matched 3D RMS to `2.483 m`, so it is not the
  native CLI default.  The next parity target is native phase-update behavior
  after bridge-aligned `J03`/`C30` admission plus satellite/STEC/frequency
  admission for `C28/C29` and `E31` in L6D estimated-ionosphere runs, rather
  than bias sign, P/Y/W ionosphere mapping, elevation-mask mismatch,
  constellation exclusion, phase seed formula, or a simple SSR-bias
  availability gate.

  Iteration-filtered code-row contribution diffs (`--row-type code
  --iteration 0`) plus re-extracted PPP filter logs now localize the first
  divergence point.  Control was regenerated with `--ppp-filter-log`
  (`/tmp/madoca_control_regen_filter.csv`), matching the original control `.pos`
  exactly (0 m delta over all `120` epochs).  At the first epoch `TOW 172800`
  iter `0` the admitted phase-row count differs by one (control `29`, candidate
  `28` for `C30:0+C30:1` before `173040`), which immediately changes the joint
  innovation covariance: `S` phase coupling delta is `-177,446 m^2`, `S^-1`
  phase coupling delta is `-0.00303 1/m^2`, and `S^-1` code coupling delta is
  `+0.0176 1/m^2`, while the common `54` code rows are identical in count and
  identity.  Clock and position Kalman gain sums already move at this first
  iter-0 step (`+0.00171` clock, `+0.000457` position), so the divergence
  originates in `S`/`S^-1` construction driven by the missing warm-start phase
  row, not in gain assembly or state-update order per se.  Warm-start is the
  only iter-0 phase admission: iter-0 phase rows are `29->28` at `TOW 172800`,
  `9->8` at `172830`, `3->2` at `172860`, then `0->0` from `172890` onward.
  After the first three epochs, iter-0 has no phase rows in either run, and the
  S phase coupling delta collapses to zero; the code-row update delta
  nevertheless keeps growing on `C30:f1` (iono `+14.6 m` at `172860`, `+24.6 m`
  at `172890`, `+36.5 m` at `172950`) because the in-memory `P` covariance and
  state going into iter 0 have already absorbed the first-epoch S/gain
  divergence and propagate it through iterations `1..7` of each subsequent
  epoch.  At the `173040` threshold epoch, iter-0 phase rows are `0->0` and
  iter-1 phase rows are `41->38`, so the phase row re-admission is not
  restoring row parity; `C30:0`/`C30:1` stay below the lock gate until later.
  Supporting artifacts: `/tmp/madoca_code_contrib_iter0_c30e26_vs_control_narrow.json`,
  `/tmp/madoca_code_contrib_iter0_c30e31_vs_control_narrow.json`,
  `/tmp/madoca_code_contrib_iter0_c30e26_trace.json`,
  `/tmp/madoca_code_iter0_epoch_by_epoch.json`.

  This reframes the remaining target: the bridge parity gap is not a
  covariance-inversion bug that triggers at the bridge-valid threshold, but the
  accumulated state+covariance effect of a single warm-start phase row whose
  inclusion policy differs between runs.  The next follow-ups should therefore
  test (i) making the control-side warm-start respect the same before-threshold
  exclusion list so first-epoch `S` is identical to the candidate, (ii) an
  ambiguity/covariance reset at the before-threshold boundary that re-seeds
  `C30:0/C30:1` from the post-threshold observation rather than the carried
  state, and (iii) confirming on `C30:0+E26:0` and `C30:0+E31:0` variants that
  the residual state divergence after equalizing iter-0 admission is bounded.

  Follow-up (i) was then run directly: a control variant with `C30:0` removed
  from the warm-start pair list (`ctrl_noC30ws`,
  `/tmp/madoca_ctrl_noC30ws.pos`) reproduces the candidate's first-epoch iter-0
  row shape (code `54`, phase `28`, iono `10`) and matches `c30_e26` iter-0
  exactly (`/tmp/madoca_ctrl_noC30ws_filter.csv`).  Against the MADOCALIB
  bridge that gives `0.121 m` horizontal / `0.356 m` up / `0.376 m` 3D RMS
  over the `118` matched epochs and `0.074 m` / `0.138 m` / `0.156 m` tail.
  Compared with the `29`-row control's `0.122` / `0.374` / `0.393` (tail
  `0.110` / `0.128` / `0.169`), removing `C30:0` from warm-start alone
  accounts for roughly `0.017 m` full-window 3D improvement, while
  `c30_e26`'s before-threshold gates for `C30:0+C30:1+E26:0` give `0.125` /
  `0.310` / `0.334` (tail `0.207` / `0.047` / `0.212`).  Therefore first-epoch
  admission equalization explains only about `30 %` of the `c30_e26`
  improvement; the remaining `~0.042 m` full-window gap comes from mid-epoch
  admission timing for rows that are not in any warm-start.  A direct
  `ctrl_noC30ws` vs `c30_e26` comparison is `0.037` horizontal / `0.082` up /
  `0.090` 3D matched delta RMS with `0.175 m` max at `TOW 173430`, confirming
  that two runs with identical first-epoch `S` still diverge through the rest
  of the pass.  The mechanism is the lock-count gate admitting `C30:1` /
  `E26:0` before the bridge-aligned threshold in `ctrl_noC30ws`, while
  `c30_e26` defers those admissions past `173040`/`173610`.  The remaining
  parity target is therefore the mid-epoch first-admission policy for
  non-warm-start satellites, not the initial warm-start row set alone.

  A direct cross-check against the MADOCALIB bridge `.stat` `$SAT` rows now
  identifies why those specific thresholds exist.  Bridge first-valid epochs
  (extracted from `/tmp/madocalib_bridge_pppar_ion_0000_outstat.pos.stat`):
  `C28` at TOW `173010`, `C29`/`C30` at `173040`, `E04`/`E06`/`E26`/`E31`
  at `173610`, `E05` at `173910`, `J03`/`J04` at `172860`, `R09`/`R21` at
  `172950`.  The Galileo threshold `173610` coincides exactly with the
  CSSR orbit-IODE transition from `32` to `34` for those satellites, while
  the DLR BRDM broadcast IODE is already `34` by `172830`.  Before the SSR
  side catches up to the broadcast side, MADOCALIB's `satpos()` finds no
  usable SSR IODE match and silently drops those rows, which is why bridge
  writes no `$SAT` lines for `E04/E06/E26/E31` before `173610` even though
  their elevation is `16..60 deg`.  This matches the C30 case: CSSR IODE
  becomes `158` at `172800`, but the DLR BRDM broadcast IODE stays at `1`
  until the new IODnav appears, so bridge waits until `173040` for the first
  C30 `$SAT` row.

  Native strict IODE then reproduces the timing closely.  Re-running the
  `29`-row warm-start baseline with `--enforce-ssr-orbit-iode` on the same
  single DLR BRDM (`/tmp/madoca_ctrl_strictiode.pos`) defers
  `E04/E06/E26/E31:0` to `173640` (1 epoch after bridge), `C28:0` to
  `173040` (1 late), `C29:0` to `173160` (4 late), and `C30:0` to `173040`
  (matches bridge exactly).  Trajectory comparison against the bridge is
  `0.292 m` horizontal / `0.929 m` up / `0.973 m` 3D all-window, however,
  versus `0.125 m` / `0.310 m` / `0.334 m` for the manual `c30_e26`
  before-exclusion list.  Strict IODE therefore matches bridge admission
  timing almost perfectly without any hand-picked per-satellite before-gate,
  but changes the broadcast-state geometry downstream, which dominates the
  solution difference (up `+0.619 m` RMS, first-matched epoch
  `TOW 172860` shifts by `1.414 m`).

  Supporting artifacts:
  `/tmp/madoca_ctrl_noC30ws_filter.csv` /
  `/tmp/madoca_ctrl_noC30ws_residuals.csv` for the warm-start-equalized
  variant; `/tmp/madoca_ctrl_strictiode*` for the strict-IODE variant;
  bridge stat `/tmp/madocalib_bridge_pppar_ion_0000_outstat.pos.stat`.
  First-admission timing table was computed directly from the native
  residual logs by selecting the earliest `phase_accepted=1` row per
  `(sat, frequency_index)` identity.

  This closes the question "why does native admit too eagerly?": the
  single-BRDM run gets broadcast IODE=34 at `172830`, native
  `getRtklibEphemeris()` accepts that geometry despite SSR IODE=32 and
  admits carrier phase for `E04/E06/E26/E31` at `172860`, and `E26:0` then
  produces a `3.9 m` first residual that passes the `18 m` pre-convergence
  floor.  Bridge rejects the same broadcast state because its SSR IODE
  doesn't match, and only admits at `173610`.  The next native target is
  therefore a scoped admission gate that reproduces bridge timing (strict
  SSR orbit IODE for admission) while keeping the existing
  RTKLIB-selection broadcast state for range modeling, so the downstream
  geometry doesn't change like `--enforce-ssr-orbit-iode` currently does.

  `PPPConfig::enforce_ssr_orbit_iode_admission_only` now implements that
  scoped variant, exposed as `--enforce-ssr-orbit-iode-admission-only` on
  `gnss_ppp`.  When enabled, native still rejects satellites whose SSR
  orbit IODE has no matching broadcast ephemeris (same admission gate as
  `--enforce-ssr-orbit-iode`), but does not swap `eph` or recompute the
  broadcast state, so `getRtklibEphemeris()` selection continues to drive
  the geometric range term.  On the same MIZU `29`-row warm-start baseline
  (no ANTEX, elmask 10) this gives `0.255 m` horizontal / `0.285 m` up /
  `0.382 m` 3D all-window RMS against the bridge and `0.108` / `0.147` /
  `0.147 m` tail, versus `0.122 / 0.374 / 0.393` / `0.110 / 0.128 / 0.169`
  for default (no IODE gate) and `0.292 / 0.929 / 0.973` / `0.209 / 0.646 /
  0.682` for full `--enforce-ssr-orbit-iode`.  The new gate therefore
  gives the best tail RMS of all tested profiles on this single-BRDM run
  and improves the full-window RMS vs default, without the vertical
  regression that the broadcast-state swap produces.  First-admission
  timings align with the bridge to within one epoch for every tested
  Galileo identity (`E04/E06/E26/E31:0` all `173640` vs bridge `173610`),
  and C30 first-admission matches bridge exactly at `173040`.  The
  remaining max 3D delta is `1.787 m` at `TOW 172890`, an early transient
  caused by dropping `~4` Galileo satellites that the default admits; this
  transient accounts for most of the full-window RMS above the `c30_e26`
  manual before-exclusion list (`0.334 m`), while the tail behavior is
  better than any previously tested native profile.

  Two follow-up experiments confirmed the admission-only gate scope.
  Combining it with the existing `c30_e26` before-exclusion list
  (`C30:0/C30:1:2360:173040` and `E26:0:2360:173610`) produces an
  identical trajectory (`0.382 m` 3D all-window, bit-for-bit match at
  `TOW 172890` max delta), because the IODE gate already rejects those
  rows before their manual thresholds: the before-exclusion list is
  therefore subsumed by admission-only strict IODE on this scenario.
  Disabling the warm-start instead makes things worse (`0.534 m` 3D
  all-window, `1.701 m` max at `TOW 172860`), confirming that the
  warm-start is still useful for the non-gated satellites even in this
  mode.  A regression test
  (`PPPTest.ProcessorAdmissionOnlySsrOrbitIodeRejectsMismatch`) covers
  the gate on a synthetic one-satellite IODE mismatch.

  Per-window breakdown explains the remaining full-window gap vs
  `c30_e26`.  Against the same bridge comparison, admission-only gives
  `0.382 m` 3D all-window, `1.046 m` in the first `9` epochs
  (`TOW <= 173100`), `0.334 m` in the mid window
  (`173100 < TOW <= 174600`), and `0.177 m` in the tail (`TOW > 174600`).
  `c30_e26` gives `0.334 / 0.462 / 0.436 / 0.173`.  So admission-only
  is better than `c30_e26` in the mid window (`0.334` vs `0.436`) and
  ties in the tail, but is worse in the early `9` epochs
  (`1.046` vs `0.462`).  The early regression comes from the IODE gate
  rejecting stale-CSSR-IODE BDS observations at
  `TOW 172800..172890`: BDS C30 CSSR IODE is `158` there and no BRDM
  ephemeris with that IODE is available, so the gate drops BDS
  C07/C09/C10/C16/C27/C28/C29/C30/C36/C37/C39/C40/C45 plus GPS G07/G08
  and a GLONASS subset, leaving `~10` sats (GPS+QZSS) at the first
  epochs.  `c30_e26` has no IODE gate, so it admits the stale BDS rows
  for those first epochs and loses the mid-window advantage because
  stale-IODE BDS then drifts in the filter.

  A `--start-tow 172860` experiment tested the hypothesis that the two
  pre-bridge epochs (`172800/172830`) seed a bad state.  Starting the
  native run from the bridge's first solution row gave a `6.23 m` spike
  at `TOW 172860` and worsened both full-window (`0.889 m`) and tail
  (`0.200 m`) RMS, refuting the hypothesis: the two pre-bridge epochs
  are useful warm-up rather than a removable source of mismatch, even
  with the admission-only gate reducing their sat count.  An extended
  manual before-exclusion list adding `E04/E06/E31` to `c30_e26` gave
  `0.345 m` full-window / `0.178 m` tail, slightly worse than the
  two-satellite `c30_e26` list and still worse than admission-only in
  the tail; list extension is therefore not a useful direction.

  Current status: admission-only strict IODE is the best
  automatic-parity profile for this MIZU scenario.  It gives the best
  tail RMS of all tested profiles (`0.147 m`), matches bridge first-
  admission timing within one epoch on every tested Galileo/BDS
  identity, removes the need for per-satellite before-exclusion
  lists, and improves full-window RMS over the default.  Closing the
  remaining early-window gap would require admitting stale-CSSR-IODE
  BDS observations for the first few epochs only (a hybrid gate that
  is off initially and switches to strict IODE after a warm-up), which
  is a separate design decision and not the immediate priority.

  A deeper ANTEX investigation then explained why earlier native runs
  had been done without `--antex`: enabling the same ANTEX file that
  bridge uses degraded native from `0.382 m` to `0.811 m` 3D delta.
  The root cause turned out to be a MADOCA-specific SSR convention.
  MADOCALIB's sample `pos1-sateph = brdc+ssrapc` uses `EPHOPT_SSRAPC`,
  which passes `opt=0` to `satpos_ssr()` so that `satantoff()` is NOT
  called: MADOCA SSR orbit corrections are already delivered at the
  satellite antenna phase center.  Native was re-applying the
  satellite PCO on top of the APC-referenced SSR orbit, which double-
  corrected the satellite position by the GPS/QZSS/Galileo Z-axis
  offset (`~2.25 m` body-frame), and after rotation projected into
  meter-scale range errors per satellite.  A secondary bug in the
  same path: native was applying the satellite PCO with a reversed
  sign and in per-signal (not iono-free-LC) mode, while MADOCALIB
  always uses the IFLC of frequency-0/1 PCOs (`preceph.c:617-621`).

  `PPPConfig::ssr_orbit_reference_is_apc` now suppresses satellite
  PCO application when SSR orbits are delivered at the phase center,
  and MADOCA native path (`--madoca-l6e` enabled) sets this flag to
  true.  The remaining satellite PCO path (for CoM-referenced SSR)
  now uses the MADOCALIB iono-free-LC combination with the RTKLIB-
  standard positive sign.  On the `29`-row MIZU warm-start run with
  `--antex`, the bridge comparison moves from `0.254/0.285/0.382 m`
  to `0.253/0.261/0.364 m` 3D delta (full-window) and from
  `0.108/0.147/0.182` to `0.105/0.116/0.156` in the tail 1800 s, and
  the absolute reference RMS drops from `0.428/0.188 m`
  (full/tail) to `0.395/0.161 m`.  Without `--antex` the numbers are
  unchanged, confirming the fix is surgical to the ANTEX path.  The
  bridge itself reaches `0.184/0.024 m` absolute, so the structural
  receiver/satellite PCO double-correction is closed; the remaining
  tail gap is likely AR (bridge fixes, native stays float) and
  state-seeding details that need their own separate audit.

  A detailed MADOCALIB AR pipeline study then mapped out the gap
  between native and `ppp_amb_ILS` for future implementation.  Key
  constants from `ppp_ar.c`:
  `MAX_FRAC_WL_FIX=0.20` cycle, `MAX_STD_WL_FIX=1.00` cycle,
  `CONST_AMB=0.01 m` for N1, `CONST_AMB_BDS=0.05 m`,
  `CONST_AMB_BDS_B2a=0.10 m`, `CONST_AMB_GAL_E6=0.10 m`,
  `CONST_AMB_EWL=0.10 m`, `MAX_STD_FIX=0.15 m`, ratio threshold
  inflation table `{5.0, 5.0, 3.0, 2.0, 1.5}` applied for
  `na - MIN_AMB_RES(4)` in `{0..4}`, `REL_MATCH_RATE=0.90`
  (relax ratio * 0.8 when first/second LAMBDA candidates share
  >=90% integers), `MIN_MATCH_RATE=0.10` (abort when <10% share).

  The MADOCALIB pipeline is structured as:
  1. `gen_sat_sd()` builds single-difference satellite pairs by
     picking a reference sat per constellation and making SD with
     every other valid sat in that constellation.
  2. `search_amb_ewl()` (nf>=3) computes SD extra-wide-lane
     ambiguity from `x[j1]/lam[f1] - x[j2]/lam[f2]` for both ref
     and pair sats, rounds, and gates on `|frac| <= 0.20 cycles`
     AND `sigma <= 1.00 cycles`.  Kept rows feed a design matrix D.
  3. `update_states(STEP_EWL)` injects the EWL fixes as Kalman
     pseudo-observations with `R=CONST_AMB_EWL^2=0.01 m^2` (or
     `CONST_AMB_GAL_E6^2` for Galileo E6).  This tightens the
     per-frequency ambiguity covariance before WL.
  4. `search_amb_wl()` does the same thing for WL (F1-F2), using
     the now-tightened per-frequency ambiguity states.
  5. `update_states(STEP_WL)` applies WL fixes as constraints with
     `R=CONST_AMB^2=0.0001 m^2`.
  6. N1 narrow-lane: gate on `sqrt(P(pos)) <= thresar[1]=1.5 m`
     first, then loop `armaxiter=10` times calling
     `gen_sd_matrix_n1` (build D and Q on L1 SD pairs),
     `search_amb_lambda` (LAMBDA with inflated threshold and
     match-rate relaxation), and `exc_sat_par` (partial AR;
     exclude worst candidate).  On ratio pass, update state with
     `STEP_NL` and gate on `MAX_STD_FIX=0.15 m` post-fix position
     std.  On fail, keep WL-fixed state but return wide-lane-only
     solution.

  The sample `pos2-arthres=1.5` configuration means that at
  `na>=9`, the effective LAMBDA threshold matches the native
  default of `1.5`, so the ratio gate itself is not the
  discriminator.  The real discriminator is the **state
  tightening** performed by `update_states` at STEP_EWL and
  STEP_WL: by the time N1 LAMBDA runs, the per-frequency
  ambiguity covariance has been constrained by the EWL/WL fixed
  integers with `R=0.01..0.10 m^2`, which reduces the N1 float
  variance to the point where LAMBDA can distinguish first and
  second candidates.

  Native currently applies WL fixes as `ambiguity.wl_is_fixed=true`
  flags with MW-smoothed values, but does NOT inject them as
  Kalman pseudo-observations against the filter state.  The N1
  LAMBDA therefore runs on an untightened state where variances
  remain at the standard PPP process-noise scale, giving the
  `0.1..0.4` cycle fractional residuals observed in the earlier
  audit.  Porting MADOCALIB's `update_states(STEP_EWL/STEP_WL)`
  is the single largest expected win for AR parity.

  Implementation sketch for a future iteration:
  - Add `DD_MADOCA_CASCADED` ARMethod option.
  - Port `applyWideLaneStateConstraint(pairs, integers, D, R)`
    that builds a design matrix mapping per-frequency ambiguity
    state indices to WL (1/lam_L1, -1/lam_L2 for ref; -1/lam_L1,
    +1/lam_L2 for pair) and injects Kalman pseudo-observations
    with `R=CONST_AMB^2=1e-4 m^2` (per-constellation de-weighting
    for BDS/B2a/GAL-E6).
  - Compute EWL first when nf>=3 (QZSS L5, Galileo E5a/E5b/E6, BDS
    B2a/B2I/B3I); this requires per-frequency ambiguity states
    for the 3rd/4th frequency, which native already supports via
    `enable_per_frequency_phase_bias_states`.
  - After EWL+WL state tightening, run L1 DD LAMBDA on the
    constrained state.
  - Implement match-rate-based ratio relaxation: extend
    `lambdaSearch()` to also return the second-best integer
    vector, compute per-row agreement fraction, then apply
    `thres *= 0.8` when >=90% match, `thres = 99.99` when <10%.
  - Implement PAR iteration: on ratio fail, identify the SD pair
    whose integer most disagrees with the second candidate, drop
    it, repeat up to `armaxiter=10`.
  - Expected outcome: native fixes `9..30` sats per epoch on MIZU
    like bridge does; tail absolute RMS drops from 0.161 m
    toward bridge's 0.024 m.  The code change is order of
    200-400 lines in new helpers plus wiring; the validation is
    fixture-level test coverage at each cascade step.

  An AR (ambiguity resolution) audit then examined the remaining
  tail absolute gap.  Bridge tail absolute RMS is `0.024 m` because
  MADOCALIB cascaded AR fixes `9..30` ambiguities per epoch from
  `TOW 172860` onward (`$SAT` `fix=1` in the bridge stat).  Native
  with `--ar-method dd-wlnl` produces DD NL pairs but the fractional
  parts run `0.1..0.4` cycles, LAMBDA ratios stay at `1.0..1.2`, and
  the default `1.5` ratio gate rejects all fixes.  Lowering the
  ratio gate accepts marginal fixes that degrade the trajectory:
  at `ratio=1.0` native produces `101` fixed solutions but the
  bridge comparison worsens from `0.364/0.156 m` full/tail to
  `0.371/0.205 m`.  The N1 float precision (~40% cycles) is the
  bottleneck: bridge achieves the required precision by cascaded
  EWL/WL/N1 with MADOCALIB-specific phase-bias gates and variance
  de-weighting, which the native WLNL path does not yet replicate.
  Closing this gap is an AR pipeline refactor, out of scope for
  this admission/PCO commit series.  `gnss_ppp` now exposes
  `--ar-method <dd-iflc|dd-wlnl|dd-per-freq>` so future AR work
  can be triaged alongside existing diagnostics.

  A hybrid gate was then implemented as
  `PPPConfig::ssr_orbit_iode_admission_gate_warmup_epochs` / CLI flag
  `--ssr-orbit-iode-admission-gate-warmup-epochs <N>`.  The flag
  deactivates the admission-only IODE gate for the first `N` processed
  epochs, then activates it from epoch `N+1` onward.  Sweeping
  `N=2/3/5/7/10/15` on the same MIZU run gives
  `0.510/0.518/0.500/0.481/0.442/0.410 m` full-window 3D RMS and
  `0.189/0.157/0.156/0.152/0.142/0.133 m` tail RMS.  The warmup
  direction therefore monotonically degrades full-window RMS and
  monotonically improves tail RMS vs pure admission-only (`N=0`):
  pure admission-only is the best full-window (`0.382 m`), while
  larger warmups progressively approach default (`N=999` gives
  `0.393/0.169`).  The mid-run activation cliff (where satellites that
  were being tracked get suddenly rejected) is the mechanism behind
  the full-window degradation, and the tail improvement is a secondary
  effect from the larger warmup keeping a fuller state going into the
  mid window.  No intermediate warmup dominates on both metrics.  The
  hybrid flag is therefore kept as a diagnostic knob rather than a
  default-mode change; regression coverage in
  `PPPTest.ProcessorAdmissionOnlySsrOrbitIodeWarmupEpochsAdmitsEarly`
  verifies that warmup admits an otherwise-gated row during the first
  epoch.

## Acceptance Gates

- Existing tests pass without MADOCALIB installed.
- Oracle-linked tests are opt-in and clearly labeled.
- Helper parity reaches `1e-6` for deterministic scaled values.
- L6E decoded orbit/clock rows, code/phase bias rows, and URA rows match
  MADOCALIB for the MIZU sample frames in the opt-in oracle build.
- L6D decoded STEC delay/std rows match MADOCALIB for selected receiver
  positions.
- Whole-run PPP and PPP-AR sample windows are compared only after helper and
  decoder parity are stable.

## Non-Goals For Iter1

These were the constraints for the initial skeleton iteration.  Later iterations
now include native MADOCA decode, optional MADOCALIB bridge parity, and scoped
PPP integration.

- No MADOCA decoder implementation.
- No MADOCALIB linking.
- No PPP behavior change.
- No CLAS source changes.
