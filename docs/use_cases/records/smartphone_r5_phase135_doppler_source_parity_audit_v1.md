# Smartphone R5 Phase135 Doppler source-parity audit

Status: read-only source audit, 2026-09-04.  Decision: the Phase135 D
adapter needs a source-locked correction before any structural/raw contract
can be qualified.  No raw, truth, MAT, solver, accuracy, precomputed
coordinate, or Kaggle artifact was read or executed.

## Scope and conclusion

The audit compares the pinned official Gsat/Gobs and DopplerFactor source with
the Phase135 implementation at `48b13f098a75da87e52bcb15f66e3434c950baf7`.
The existing P and TDCP affine rows already have the intended fixed-initial
factor topology.  The D row is not yet source-equivalent in two coupled
places:

1. the Phase135 geometry helper exposes `+(satellite-receiver)/range`, while
   the official scripts pass `[-satr.ex,-satr.ey,-satr.ez]` to the affine
   factor; and
2. native D provenance uses a satellite-only/rotated-state rate, while the
   official Gsat rate includes receiver velocity and the explicit first-order
   Sagnac term.  The native Phase135 backend currently reuses the prepared
   `residual_mps` instead of reconstructing that official `resD` from the
   retained measured rate, satellite clock drift, and same-run initial
   receiver velocity.

These are equation/geometry mismatches, not evidence for sigma, Huber, LM,
filter, or route tuning.  The only authorized pre-raw correction is a
   Phase135-local adapter: preserve the existing graph and row admission,
   expose official `-e` as the factor LOS, and compute the official range-rate
   and `resD` from finite raw provenance.  Legacy/default and all non-Phase135
   paths remain byte/semantic unchanged; missing provenance must fail closed.

## Official source evidence

The source files were read from the reproducibility cache only.  Their SHA-256
identities are:

| file | SHA-256 | relevant lines |
|---|---|---:|
| `gsdc2023/fgo_gnss.m` | `5368e4056f70f448b728792c5e4c124b7f2afc1e00653492b807083bd3ccf0c3` | 64–69 |
| `gsdc2023/fgo_gnss_imu.m` | `c70090ccb8b27fc8ac7fd2929e2f995a14cfe7f089bb1fef067370e22051c3e3` | 76–83 |
| `MatRTKLIB/+gt/Gsat.m` | `a56c3236646606c1b7e771180e9bc18b08f809fe8a6d71b795366b96207871f6` | 299–306 |
| `MatRTKLIB/+gt/Gobs.m` | `be27fd1a075829a00f0aec5f32a0040a884077a8ac256a10a2bc26def255fc88` | 1154–1158 |
| `MatRTKLIB/src/mex/geodist.c` | `3929ec3d57575634a921a81957ce91f2dce9314457694e3bd7e00bd82ec765c4` | 73–91 |
| `gtsam_gnss/src/DopplerFactor_VD.h` | `de2c11d06ff860a95785c84620e5ed57dcfd5ca24995a0e642e9a5984fe093e1` | 45–54 |

The native files at audit time were:

| file | SHA-256 | relevant lines |
|---|---|---:|
| `src/algorithms/fgo_problems.cpp` | `6443b13d9df54e627bc73270b7d28df9c5d309043e2e194ba2b6b5c1b274bc25` | 798–855, 899–907 |
| `src/algorithms/fgo_gtsam_internal.hpp` | `50abd1893d174ba770a3a9b7772c9a3fe5e55f01aec2f5f6d49364ca4d5b583c` | 1369–1400, 1520–1556 |
| `src/algorithms/fgo_gtsam_backend.cpp` | `b63fcc64b94a1a246f0dcd175dca59fa076207859cef14ffd51c4b1d49c3f114` | 1366–1420 |
| `include/libgnss++/algorithms/doppler_contract.hpp` | `66d97c9ca2cc474247464d8af782cc3ca1837195967e601576ee25b6f8e1f56d` | 42–150 |

## Formula comparison

`geodist.c` computes `e=(rs-rr)/|rs-rr|` and adds

```
OMEGA_E / CLIGHT * (rs_x * rr_y - rs_y * rr_x)
```

to the geometric range.  Both official scripts then deliberately pass
`-e` (the receiver derivative) as their factor LOS at
`fgo_gnss.m:64-69` and `fgo_gnss_imu.m:76-83`.  The official
`DopplerFactor_VD` residual is:

```
los^T * (v - initial_v) + d - prr
```

with `[los^T, 1]` Jacobian (`DopplerFactor_VD.h:45-54`).

`Gsat.m:300-306` computes the known range rate as

```
(sat_v - receiver_v)^T e
 + OMEGA_E/CLIGHT *
   (sat_v_y*receiver_x + sat_y*receiver_v_x
    - sat_v_x*receiver_y - sat_x*receiver_v_y)
```

and `Gobs.m:1158` forms the metres/second residual:

```
resD = -D*lambda - (known_rate - satellite_clock_drift)
```

The existing Phase135 backend uses `factor.residual_mps` and the helper LOS.
Its preparation path at `fgo_problems.cpp:798-855` instead calls the native
rotated-state rate helper in corrected Android mode and then stores a
receiver-only residual.  This does not provide the official receiver-velocity
term and explicit Sagnac expression to the Phase135 factor.

## Term classification

| term | classification | evidence / action |
|---|---|---|
| Hz → wavelength → metres/second | equivalent | `Gobs.m:1158`; `doppler_contract.hpp:18–40`; retained unchanged |
| satellite clock drift unit/sign | equivalent | `Gobs.m:1158`; native seconds/second × `CLIGHT` at `fgo_problems.cpp:826–835`; retained unchanged |
| factor equation and `[V,D]` key order | exact after adapter | official `DopplerFactor_VD.h:45–54`; Phase135 factor has same coefficients/keys |
| LOS sign | **divergent before correction** | official scripts pass `-e`; current `phase135SourceGeodist` returned `+e`; correction changes only Phase135 helper |
| range Sagnac | equivalent for range | `geodist.c`; Phase135 uses one unrotated-state term; no second term permitted |
| rate Sagnac | **divergent before correction** | official `Gsat.m:304–306` has explicit receiver position/velocity term; current corrected native branch rotates satellite state |
| receiver velocity in known rate | **missing before correction** | official `Gsat.m:300–306`; current Phase135 backend reused prepared residual and did not reconstruct with initial receiver velocity |
| D residual units/sign | **divergent in prepared value before correction** | official `Gobs.m:1158`; corrected adapter must use measured rate − (official rate − clock drift) |
| atmosphere in D | exact in scope | official atmospheric correction is only P/L (`Gobs.m:1160–1165`); unchanged |
| D sigma/Huber/filter/admission | fixed, out of scope | no change authorized; factor counts must remain identical |
| satellite velocity interpolation | unresolved | native `navigation.cpp` central-difference provenance is not claimed identical to official source; finite/source coverage is a hard gate |

## Required correction and fail-closed boundary

Before any structural/raw run, Phase135 must add a source-locked helper that
accepts the unrotated transmit-time satellite position/velocity, receiver ECEF
position, same-run initial receiver ECEF velocity, measured range rate, and
satellite clock drift.  It must return the official range rate and `resD` only
when every input and output is finite and the geometric range is positive.
The Phase135 D insertion must use that result and the official `-e` LOS.  The
same helper is tested with a synthetic nonzero receiver velocity and explicit
Sagnac term.  The non-Phase135 prepared residual path is untouched.

The structural contract must additionally require finite measured-rate and
clock-drift provenance for every admitted D row; missing or malformed values
abort before graph construction.  No fallback, zero synthesis, extrapolation,
extra Sagnac, factor-count change, epoch-0 admission, or solver/config change
is allowed.

## Candidate and read accounting

Exactly one implementation correction is selected: **Phase135 official D
range-rate/residual + official factor LOS adapter**, scoped to the already
frozen affine family.  It is not a new tuning candidate and does not authorize
raw execution.  The subsequent structural freeze must pin the correction
commit and the original `bfafa340803f2dd1b5528c918b5be60f6e0e5c14` design
freeze separately.

| class | reads | execution |
|---|---:|---:|
| official/native source | source only | 0 solver/raw runs |
| phone GNSS/IMU/nav or raw base | 0 | 0 |
| truth/MAT/solution coordinate rows | 0 | 0 |
| precomputed/PDC/accuracy | 0 | 0 |
| Kaggle/token/network dataset access | 0 | 0 |
| rerun/fallback/sweep | 0 | 0 |

This audit records the correction boundary only.  It authorizes no raw input,
solver, truth, accuracy, solution publication, or Kaggle operation.
