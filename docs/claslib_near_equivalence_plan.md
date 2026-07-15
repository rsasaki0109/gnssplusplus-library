# CLASLIB near-equivalence plan

This plan defines “near equivalent” as a reproducible, layered parity result,
not as similarity to one previously recorded position RMS. The canonical
scorecard schema is `claslib_parity_scorecard.v1`; all claimed metrics must be
traceable to one `claslib_parity_manifest.v1` file.

## Canonical contract (P0)

The primary case is the public 2019-08-27 QZSS CLAS sample, evaluated for 3600
one-second epochs beginning at 16:00:00 GPST. A manifest pins:

- OBS, broadcast NAV, and L6 inputs by path and SHA-256;
- CLASLIB implementation version, commit/build identity, and solution output;
- LibGNSS++ commit and solution output;
- reference ECEF, epoch window, 300-second warmup, status mappings, and gates;
- the constellation-specific code/phase ZD component diffs produced for the
  same run. `layers.zd_diff_jsons` keeps GPS, QZSS, and Galileo row surfaces
  independent; each constellation must pass complete coverage and both RMS
  gates. The legacy single `zd_diff_json` form remains accepted.

Start from
`configs/benchmarks/claslib_parity.example.json`, replace every placeholder,
and run:

```bash
python3 scripts/analysis/claslib_parity_scorecard.py \
  /path/to/claslib_parity.json \
  --json-out output/claslib_parity_scorecard.json \
  --fail-on-gate
```

The scorecard records artifact hashes, the raw FIX/FLOAT status matrix, status
agreement after warmup, native-only FIX epochs, FIX-count difference, the full
trajectory delta, and post-warmup all/FIX/FLOAT solution deltas. The trajectory
RMS/P95 gates use every matched epoch, including startup. It imports the existing
`madoca_solution_diff.py` implementation for epoch matching and ENU math, so
there is one position-comparison implementation. CLASLIB FIX is read from NMEA
quality `4`; native FIX is read from `.pos` `FixedAmbiguities > 0`. NMEA UTC is
converted to GPST using the applicable leap-second offset before matching.

The canonical ZD evidence uses the runner's fixed diagnostic overrides plus
`GNSS_PPP_CLAS_QZSS_S_PRN_FIX=1`. It does not enable the base-clock, SIS
boundary, or trop-grid position-scorecard gates. The runner summary records all
inherited and effective `GNSS_PPP_CLAS_*`/`GNSS_PPP_QZSS_*` inputs and emits a
typed `clas-zd-parity` profile, so an artifact cannot silently inherit a
behavior-changing gate.

## Delivery sequence

### P1 — GPS ZD closure

Match the CLASLIB row surface for GPS, including exact observation identity,
SIS 30-second boundary behavior, network/orbit suppression, atmosphere age,
and every PRC/CPC term. Exit when row keys are complete and PRC/CPC RMS deltas
are each at most 10 mm.

P1 is complete on the canonical 3580-epoch window: code and phase each match
all 52,065 GPS row keys, PRC RMS is 2.414 mm, and CPC RMS is 5.058 mm. The
implementation closed the following units without feeding the output-only L5
slot into the estimator:

1. select the network-7 bank with its 15-second reception lag and strict
   30-second lifetime;
2. combine SIS continuity with the IODE-dependent `adjust_r_dts()` geometry
   change;
3. materialize CLASLIB's zero PRC5/CPC5 L5 diagnostic surface separately from
   the L1/L2 estimator frequency count;
4. remove native-only rows when a required code/phase bias cell is withdrawn;
5. rerun the canonical scorecard and core regression suite.

### P2 — QZSS ZD closure

Extend the same contract to J01–J03. Close STEC, code bias, phase bias,
troposphere, and signal selection independently before measuring their combined
PRC/CPC result. The P1 thresholds remain unchanged.

P2 is closed. QZSS now uses CLASLIB's internal S120–S122 identity, emits the
output-only L5 slot, delays compact code-bias banks by 15 seconds, expires
phase-bias withdrawals, reproduces the legacy receiver-ANTEX slot-2 alias to
the final C02 entry, and treats omission of legacy S120 from a newer orbit bank
as an explicit J01 orbit withdrawal. The typed withdrawal status prevents the
OSR layer from publishing broadcast-only J01 rows while leaving ordinary
orbit-unavailable QZSS rows and direct J02/J03 identities unchanged.

On the full 3580-epoch run, code and phase each have 29,925 candidate rows,
29,925 common rows, and zero unmatched rows. Code bias and phase bias are
exact; PRC RMS is 1.375 mm and CPC RMS is 1.427 mm. The 300-epoch probe also
closes at 2,190/2,190 common rows with zero unmatched rows.

### P3 — Galileo row-surface closure

Make Galileo frequency and observation identity explicit and reproduce the
CLASLIB inclusion/exclusion surface. Missing rows are failures rather than
implicitly ignored comparisons.

P3 is closed on the joint GPS/QZSS/Galileo 3580-epoch ZD profile. Galileo code
and phase each have 15,640 oracle rows, 15,640 native rows, and zero unmatched
rows. Using Galileo's gravitational constant only in the old/current IODE
continuity reconstruction removes the false 0.2--0.4 m boundary jumps. PRC RMS
is 2.300 mm and CPC RMS is 1.648 mm, both below the 10 mm ZD gate. The remaining
sparse 39.5 mm code and 25.7 mm phase maxima are retained as P4/P5 diagnostic
leads; they do not prevent the specified RMS and complete-surface P3 exit.

### P4 — DD measurement parity

Add a versioned DD dump and comparator keyed by epoch, rover/base satellite,
frequency, and measurement type. Require phase DD RMS at most 5 mm and code DD
RMS at most 20 mm, with a complete common row surface.

P4 infrastructure is active. Both CLASLIB and native emit
`clas_dd_measurement.v3` prefit rows keyed by epoch, system/frequency,
phase/code, reference satellite, and target satellite. The
`clas_dd_measurement_diff.v1` comparator rejects duplicate or unmatched keys and
applies the 5/20 mm RMS gates. The first five canonical epochs exposed and then
closed the Galileo E5a DD slot mismatch (`f=2` in CLASLIB versus native `f=1`):
the surface is now 160/160 with zero unmatched rows. The residual baseline is
still open at 0.624 m phase RMS and 1.316 m code RMS. The dump also preserves
state-unadjusted `raw_dd_m`; it is diagnostic only because carrier ambiguity
makes it unsuitable for the phase gate. Version 3 also records the receiver
linearization ECEF and DD position Jacobian so measurement-model parity can be
evaluated at a common position without hiding lifecycle divergence.

### P5 — Filter lifecycle parity

Compare prefit/postfit residuals, state creation/removal, covariance, rejection,
slip/reset, and reference-satellite transitions at the first divergent epoch.
After the 300-second warmup, mutually FLOAT position delta RMS must be at most
20 mm.

### P6 — AR and FIX publication parity

Compare ambiguity candidates, integer validation, ratio, accepted set, hold,
and publication state. Require at least 99% FIX/FLOAT agreement, zero
native-only false FIX, and a FIX-count difference no greater than 1%.

### P7 — multi-dataset safety and default-on decision

Repeat the canonical scorecard on multiple public CLAS days/sites and keep
MADOCA, non-CLAS, static, white-noise, and parity-gate-off safety outputs
bit-exact. Only then consider enabling the typed CLASLIB parity profile by
default; the decision and rollback trigger must be recorded separately.

P7 reached a decision on 2026-07-15. The canonical 3580-epoch scorecard passes,
and MADOCA/non-CLAS/white-noise containment checks are bit-exact. The independent
2018-11-25 TSKB sample does not pass: its final 400 seconds are mutually FIX but
remain 58.417 mm 3D RMS from CLASLIB, above the 20 mm gate. Therefore the full
typed CLASLIB parity profile remains opt-in for 0.1. The evidence, reconsideration
criteria, and mandatory rollback trigger are recorded in
`docs/claslib_parity_default_decision.md`.

## Definition of done

For each sign-off dataset:

- expected and matched solution epoch coverage are 100%;
- ZD row-key coverage is 100%, with PRC and CPC RMS deltas at most 10 mm;
- DD phase/code RMS deltas are at most 5/20 mm;
- post-warmup FLOAT 3D delta RMS is at most 20 mm;
- FIX/FLOAT agreement is at least 99%, native-only false FIX is zero, and FIX
  count differs by at most 1%;
- full trajectory 3D delta RMS is at most 20 mm and P95 is at most 30 mm;
- safety profiles remain bit-exact.

Until every layer is present in the scorecard, a small final position delta is
evidence of progress, not proof of near equivalence.
