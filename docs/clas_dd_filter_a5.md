# CLAS DD Filter A5 Parity Closure

This pass is a STOP report rather than a model change.  The highest-value
tractable lead was QZSS remap, but the CLASLIB comparison shows that admitting
`S120..S122` as `J01..J03` exposes an upstream OSR correction mismatch rather
than a DD row-construction bug.  No default-path or gate-on accuracy behavior was
changed.

Gate-on is therefore not ready for the A5 default flip under #161.

## Verification Baseline

Run: 2019-08-27 CLAS, `--ar-ratio-threshold 2.0`, compared with
`/tmp/s31_ref/claslib_oracle_xyz_full.pos`.  Solution X/Y/Z are `.pos` columns
3-5; fixed epochs use `FixedAmbiguities` column 13 > 0.

| Path | Fixed epochs | Fixed 3D mean | Fixed Up mean | All-epoch 3D mean |
| --- | ---: | ---: | ---: | ---: |
| A4#2 entry | 1067 | 1.122978 m | 0.353088 m | 1.007274 m |
| A5 safe tree | 1067 | 1.122978 m | 0.353088 m | 1.007274 m |
| Native fixed parity target | - | ~0.935 m | - | - |
| CLASLIB target | - | ~0.06 m | - | - |

Reject split from `/tmp/a5_check_clas_dd_diag.csv`:

| Reject reason | Split | Count |
| --- | --- | ---: |
| `fixed` | accepted | 1067 |
| `postfit_dd_residual` | no worst-row split recorded | 969 |
| `ratio` | ratio gate | 932 |
| `insufficient_ambiguities` | ambiguity count | 553 |
| `postfit_rms` | `m0 f1` GPS/native L2 | 69 |
| `postfit_max` | `m0 f1` GPS/native L2 | 8 |
| `postfit_rms` | `m0 f0` GPS L1 | 1 |

## Lead 1 - GPS L2W Identity

CLASLIB's GPS `f1` DD rows use RTKLIB code `20` (`L2W`/`C2W`).  At the common
230425 epoch, `/tmp/a4_claslib_dd_rows_codes.csv` has GPS `m=0,freq=1` rows with
`ref_code=sat_code=20`; examples:

| Row | CLASLIB ref > target | Type | Residual |
| --- | --- | --- | ---: |
| 5 | 14 > 25 | phase | -0.008342690570 m |
| 15 | 14 > 25 | code | 0.079478304651 m |
| 19 | 14 > 32 | code | 0.010024597729 m |

The native OSR/component dump at the same epoch labels the GPS L2 rows as
`signal=3`, the collapsed `SignalType::GPS_L2C` identity, even when the RINEX
observation type is C2W/L2W:

| Native sample | Frequency | Signal | Component |
| --- | ---: | ---: | ---: |
| G14 code | 1 | 3 | `PRC=0.552408556406` |
| G25 code | 1 | 3 | `PRC=10.719942900064` |
| G14 phase | 1 | 3 | `CPC=4.948411342899` |
| G25 phase | 1 | 3 | `CPC=-2.501550299340` |

This cannot be fixed faithfully inside `ppp_clas_dd.cpp`: the DD builder can only
request observations by `SignalType`, and the epoch store has already collapsed
GPS band 2 to `GPS_L2C`.  A safe fix needs a shared, default-bit-exact design that
preserves GPS C2W/L2W identity through RINEX observation selection, OSR bias and
antenna lookup, WL/NL, and DD row building.  A gate-only DD tweak would still use
the wrong raw observation key and would be a model alias, not parity.

## Lead 2 - QZSS PRC/CPC Residual Parity

With explicit `GNSS_PPP_CLAS_QZSS_S_PRN_FIX=1`, native row topology matches
CLASLIB's QZSS reference choice at 230425: J02 is the reference and J01/J03 are
targets on L1 and L2.  The residuals do not match.

CLASLIB DD rows from `/tmp/a4_claslib_dd_rows_codes.csv`:

| Row | CLASLIB ref > target | Type | Freq | Residual |
| --- | --- | --- | ---: | ---: |
| 24 | J02 > J01 | phase | 0 | -0.003311505300 m |
| 25 | J02 > J03 | phase | 0 | 0.004681515684 m |
| 26 | J02 > J01 | phase | 1 | -0.014786717596 m |
| 27 | J02 > J03 | phase | 1 | -0.007062330676 m |
| 28 | J02 > J01 | code | 0 | -0.489158532707 m |
| 29 | J02 > J03 | code | 0 | -0.153980115389 m |
| 30 | J02 > J01 | code | 1 | 0.058317956512 m |
| 31 | J02 > J03 | code | 1 | 0.313130616906 m |

Native DD rows from a temporary row probe with
`GNSS_PPP_CLAS_DD_FILTER=1 GNSS_PPP_CLAS_QZSS_S_PRN_FIX=1`:

| Native stage | Native ref > target | Type | Freq | Residual |
| --- | --- | --- | ---: | ---: |
| prefit | J02 > J01 | code | 0 | -7.644313218630 m |
| prefit | J02 > J03 | code | 0 | -6.766124047531 m |
| prefit | J02 > J01 | phase | 0 | 8.084038047135 m |
| prefit | J02 > J03 | phase | 0 | 7.503420391175 m |
| prefit | J02 > J01 | code | 1 | -13.557480467596 m |
| prefit | J02 > J03 | code | 1 | -10.581925403847 m |
| prefit | J02 > J01 | phase | 1 | 13.209885538428 m |
| prefit | J02 > J03 | phase | 1 | 11.803313726309 m |
| postfit | J02 > J01 | phase | 0 | 1.132620151406 m |
| postfit | J02 > J03 | phase | 0 | 1.159074998545 m |
| postfit | J02 > J01 | phase | 1 | 1.776547525946 m |
| postfit | J02 > J03 | phase | 1 | 1.445458916695 m |

The component dumps explain why automatic remap still worsens the oracle result.
CLASLIB has nonzero QZSS bias and antenna terms:

| CLASLIB sat | Freq | PRC | CPC | L1 iono | Phase bias | Code bias | Receiver ant |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| J01 | 0 | 3.789371397254 | 5.364220448267 | -1.299466527107 | -0.425999999046 | 0.000000000000 | -0.054425941737 |
| J01 | 1 | 5.470865926566 | 5.481619447307 | -1.299466527107 | -0.953000009060 | 2.339999914169 | -0.057853936632 |
| J02 | 0 | 5.254326729746 | 0.400651602189 | 3.358765603601 | 0.370999991894 | 0.000000000000 | -0.086889480078 |
| J02 | 1 | 7.862920437506 | -1.168542003417 | 3.358765603601 | 0.495000004768 | 0.920000016689 | -0.091491696104 |
| J03 | 0 | 6.959985942442 | 3.457236432554 | 3.372070618832 | 1.730000019073 | 0.000000000000 | -0.061036169481 |
| J03 | 1 | 9.416675272577 | -2.437588956221 | 3.372070618832 | -2.467999935150 | 0.759999990463 | -0.064249962117 |

Native QZSS components at the same epoch have zero QZSS code/phase bias and zero
receiver antenna terms, plus different STEC/PRC/CPC values:

| Native sat | Freq | Signal | PRC | CPC | L1 iono | Code bias | Phase bias | Receiver ant |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| J01 | 0 | 18 | -1.611013831296 | 11.198163480636 | -6.362446855882 | 0 | 0 | 0 |
| J01 | 1 | 19 | -5.727163477782 | 15.338193480503 | -6.362446855882 | 0 | 0 | 0 |
| J02 | 0 | 18 | 7.726374369924 | -2.388152604420 | 5.061142288767 | 0 | 0 | 0 |
| J02 | 1 | 19 | 11.000652256185 | -5.660232503110 | 5.061142288767 | 0 | 0 | 0 |
| J03 | 0 | 18 | 1.840911616063 | 6.668529936110 | -2.457599675458 | 0 | 0 | 0 |
| J03 | 1 | 19 | 0.250981159358 | 8.233645767403 | -2.457599675458 | 0 | 0 | 0 |

The input expansion also shows the issue before interpolation: near 230400 the
`S121`/`S122` rows carry atmosphere fields but no `cbias:`/`pbias:` tokens, while
the GPS/Galileo rows in the same network carry bias tokens.  The next fix belongs
in CLAS compact expansion or OSR correction materialization, not in DD row
selection.  Do not auto-enable `GNSS_PPP_CLAS_QZSS_S_PRN_FIX` until the QZSS
native PRC/CPC components match CLASLIB at the ZD level.

## Lead 3 - Galileo ZD Admission

A4#2 admits the native Galileo row subset visible at 230425:

```
m2f0C=2;m2f0P=2;m2f1C=2;m2f1P=2
m2f0C=E27;m2f0P=E27;m2f1C=E27;m2f1P=E27
```

CLASLIB still has a different Galileo frequency surface at the same epoch.  In
`/tmp/a4_claslib_dd_rows_codes.csv`, the Galileo-like group has `freq=0` and
`freq=2` rows:

| Row | Group | Ref sat | Target sat | Type | Freq | Code | Residual |
| --- | ---: | ---: | ---: | --- | ---: | ---: | ---: |
| 20 | 3 | 59 | 39 | phase | 0 | 12 | 0.006493092184 m |
| 21 | 3 | 59 | 39 | phase | 2 | 26 | 0.008112591655 m |
| 22 | 3 | 59 | 39 | code | 0 | 12 | 0.015709226291 m |
| 23 | 3 | 59 | 39 | code | 2 | 26 | 0.019745366191 m |

The broad broadcast-clock guard tried during A4 admitted the missing epoch-local
target but missed the all-epoch oracle bar.  The required A5 work is a targeted
CLASLIB-valid ZD admission source: preserve which Galileo ZD rows CLASLIB admits,
then map the native frequency slot to CLASLIB `f2` only when that ZD source says
the satellite/signal is valid.  A broad clock heuristic is not safe enough for the
default flip.

## Decision

No safe improvement was reachable without changing shared observation/correction
identity or enabling QZSS rows that are known to be wrong at the component level.
The safe A5 tree remains bit-exact gate-off and numerically identical to A4#2
gate-on.

Next phase should start outside the DD translation unit:

1. Add a default-bit-exact GPS L2W identity design that preserves C2W/L2W through
   RINEX selection, SSR bias id 9, antenna lookup, WL/NL, and DD.
2. Fix QZSS CLAS compact bias and STEC materialization until native ZD PRC/CPC
   matches CLASLIB before remap is considered automatic.
3. Build a CLASLIB-valid Galileo ZD admission source, then re-test the `f2`
   frequency-slot parity with the all-epoch oracle.

The first diagnostic artifact for that phase is
`scripts/analysis/clas_zd_component_diff.py`.  It compares native CLAS
zero-difference component CSV dumps, such as `GNSS_PPP_CLAS_CODE_DUMP`, against a
CLASLIB-side component dump and writes a `clas_zd_component_diff.v1` JSON report
plus optional component-delta CSV.  The tool intentionally changes no solver
model; it is the gate for classifying PRC, CPC, ionosphere, bias, antenna,
wind-up, and related component deltas before any GPS L2W, QZSS, or Galileo model
change is attempted.
