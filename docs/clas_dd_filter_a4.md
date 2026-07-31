# CLAS DD PPP-RTK Filter A4 Retry

A4 retry starts from the failed A4 attempt 1 and keeps the default CLAS/MADOCA
paths outside the gated `GNSS_PPP_CLAS_DD_FILTER=1` scaffold.  The retry result
is a passing partial revert: keep the A4 diagnostics, revert the harmful GPS L2C
group split and CLASLIB `f2` remap, and do not auto-enable QZSS `S120..S122`
remapping under the DD filter until QZSS residual parity is fixed.

## CLASLIB Row Audit

The CLASLIB comparison was re-run from the patched disposable build at
`/tmp/s22_claslib_build`:

```sh
CLASLIB_S24_DD_DUMP=/tmp/a4_claslib_dd_rows_codes.csv \
  bash -lc 'cd /tmp/s22_claslib_build/util/rnx2rtkp && \
  ./rnx2rtkp -k /tmp/a4_static_linux.conf \
    -ts 2019/08/27 16:00:20 -te 2019/08/27 16:01:40 \
    -o /tmp/a4_claslib_short_codes.pos \
    ../../data/0627239Q.obs ../../data/sept_2019239.nav \
    ../../data/2019239Q.l6'
```

At GPST 2068/230425 CLASLIB admits:

| Block | Reference | Targets | Rows |
| --- | --- | --- | ---: |
| GPS L1 phase/code | G14 | G25 G26 G29 G31 G32 | 10 |
| GPS L2W phase/code | G14 | G25 G26 G29 G31 G32 | 10 |
| Galileo E1 phase/code | E27 | E07 | 2 |
| Galileo E5 phase/code | E27 | E07 | 2 |
| QZSS L1 phase/code | J02 | J01 J03 | 4 |
| QZSS L2 phase/code | J02 | J01 J03 | 4 |

Attempt 1's native sample admitted GPS as a separate `m5/f1` L2C group and
mapped Galileo E5 to `f2`.  The retry found that both are unsafe in the native
model:

- CLASLIB's GPS f1 row is L2W (`CODE_L2W`), while native RINEX selection keeps
  GPS `C2X/L2X` under `SignalType::GPS_L2C`.  Splitting that as `m5/f1` was not
  CLASLIB parity and dominated fixed postfit rejects.
- The CLASLIB `f2` Galileo remap accepted bad fixed states in the current native
  ambiguity layout.  Reverting Galileo E5 to the native secondary slot restores
  the A3-stable ambiguity conditioning.
- CLASLIB does admit QZSS J01-J03 rows, but the native QZSS residual model is
  not yet parity-safe.  Enabling the `S120..S122` remap under the DD filter
  regressed oracle error even when QZSS phase rows were suppressed.

The final A4 retry row sample at GPST 2068/230425 is:

```text
m0f0C=5;m0f0P=5;m0f1C=5;m0f1P=5;m2f0C=2;m2f0P=2;m2f1C=2;m2f1P=2
m0f0*=G14, m0f1*=G14, m2f0/f1*=E27
```

That is intentionally the passing partial-revert surface, not full CLASLIB row
parity.  The unresolved QZSS and Galileo admission deltas are listed below as A5
entry points.

## A4 Retry Changes

- Reverted attempt 1's GPS L2C `m5` grouping.  GPS rows stay in CLASLIB/native
  group `m0`; this removes the worst A4#1 postfit reject source.
- Reverted attempt 1's Galileo E5 `f2` slot back toward the A3-stable native
  secondary-frequency layout.  Kept the richer per-frequency subset fallback;
  after the row-surface revert it improves fixed count and oracle error.
- Stopped auto-enabling `GNSS_PPP_CLAS_QZSS_S_PRN_FIX` from
  `GNSS_PPP_CLAS_DD_FILTER=1`.  The explicit QZSS remap knob remains available,
  and the focused QZSS row-builder unit test remains green, but default A4 retry
  gate-on does not consume the harmful QZSS rows.
- Kept the A4 diagnostics: `GNSS_PPP_CLAS_DD_DIAG` records row/ref summaries and
  the worst fixed-postfit phase row group/frequency/pair.

No ratio gate, postfit gate, static anchor, seed coordinate, or pseudo-measurement
was added.

## Oracle Result

2019-08-27 CLAS, same-epoch
`/tmp/s31_ref/claslib_oracle_xyz_full.pos`, `--ar-ratio-threshold 2.0`,
3599 matched epochs.  Fixed status uses native `FixedAmbiguities` column 13 > 0.

| Path | Fixed epochs | Fixed 3D mean | Fixed Up mean | All-epoch 3D mean |
| --- | ---: | ---: | ---: | ---: |
| A3 | 768 | 1.246752 m | 0.378185 m | 1.050019 m |
| A4#1 | 736 | 2.001001 m | 1.357775 m | 2.025515 m |
| A4#2 retry | 1067 | 1.122978 m | 0.353088 m | 1.007274 m |

A4#2 beats A3 on fixed count, fixed-only 3D, fixed Up, and all-epoch 3D.  It is
still about 0.188 m above the native parity target of about 0.935 m fixed-only
3D.

## Reject Counts

Final gate-on diagnostics from `/tmp/a4b_check_clas_dd_diag.csv`:

| Gate/result | Epochs |
| --- | ---: |
| Fixed accepted | 1067 |
| DD float postfit fallback | 969 |
| Ratio reject | 932 |
| Insufficient ambiguities | 553 |
| Fixed postfit RMS reject | 70 |
| Fixed postfit max reject | 8 |

Postfit reject split by worst phase row:

| Reason | Group/frequency | Epochs |
| --- | --- | ---: |
| postfit_rms | m0 f1 GPS/native L2 | 69 |
| postfit_max | m0 f1 GPS/native L2 | 8 |
| postfit_rms | m0 f0 GPS L1 | 1 |

Row participation over 3599 epochs:

| Group/frequency | Rows total | Mean rows/epoch |
| --- | ---: | ---: |
| m0 f0 GPS L1 phase/code | 41250 | 11.462 |
| m0 f1 GPS/native L2 phase/code | 41250 | 11.462 |
| m2 f0 Galileo E1 phase/code | 15184 | 4.218 |
| m2 f1 Galileo E5 native slot phase/code | 15184 | 4.218 |

## Verification

- `cmake --build build-madoca-parity --parallel --target gnss_ppp run_tests`
  passed.
- CLAS default gate off:
  `/tmp/a0_ref/clas_default.pos` and `/tmp/a4b_check_clas.pos` matched by
  `cmp`, md5 `92479fffdeb0010cd940aa997a482358`.
- MADOCA 1h gate off:
  `/tmp/a0_ref/madoca_1h.pos` and `/tmp/a4b_check_madoca_1h.pos` matched by
  `cmp`, md5 `1c67d6415ce92d317a392111a2d78471`.
- `./build-madoca-parity/tests/run_tests`: 323 passed, 43 skipped.

## A5 Entry Points

- QZSS is the largest unresolved parity blocker.  The CLASLIB row dump admits
  J01-J03, but native `S120..S122` remap rows worsen oracle error, including in
  a QZSS-code-only trial.  Compare native QZSS PRC/CPC residuals against
  CLASLIB `ddres()` before re-enabling automatic remap.
- GPS L2W parity requires preserving RINEX GPS `C2W/L2W` identity and applying
  SSR bias id 9.  Native currently collapses GPS band 2 to `GPS_L2C`, so L2W
  cannot be modeled faithfully inside `ppp_clas_dd.cpp` alone.
- Galileo exact admission remains open.  A broad broadcast-clock guard removes
  the extra E30 target at 230425, but it misses the all-epoch A3 bar.  Add a
  targeted CLASLIB-valid-ZD admission source rather than a broad clock heuristic.
- After QZSS and L2W identity are fixed, revisit CLASLIB `f2` frequency-slot
  parity.  The subset fallback is useful on the corrected native row surface,
  but should be rechecked when true CLASLIB f2 rows are restored.
