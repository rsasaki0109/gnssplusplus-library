# Phase120 TDCP equation and normalization parity audit

Status: sealed read-only source audit.  This audit compares the pinned
official GSDC2023 MATLAB TDCP path with the native Phase112/118 TDCP builder
and factors.  It does not authorize implementation or execution.  No new
raw phone GNSS/IMU/navigation or base read, truth or coordinate read, MAT
read, solver invocation, accuracy calculation, precomputed-coordinate read,
PDC access, Kaggle access, sweep, or rerun was performed.

## Decision

The official and native paths agree on the target Pixel5 adjacent
same-satellite/same-signal pair and on the metre-valued receiver C0
difference, but they do not use the same carrier measurement normalization.
The official TDCP observable is formed from `resL`, which contains carrier
phase in metres and satellite-clock correction but no ionosphere or
troposphere term.  Native `corrected_carrier_m` adds satellite-clock,
`-troposphere`, and `+ionosphere` before the adjacent difference.  This is a
direct source-backed, one-dimension mismatch.

Exactly one default-off candidate is therefore frozen for a future turn:
make the ordinary native TDCP pair measurement use the source-parity
`carrier_phase*wavelength + satellite_clock_m` observable (atmosphere
excluded), while leaving the native factor geometry, keys, gates, sigma,
Huber, solver, and all other observation families unchanged.  This is a
measurement-preparation candidate, not a claim that all official/native
geometry differences are solved.  No raw run is authorized by this audit.

## Source and notation

The official source tree is the pinned `gsdc2023` tree at commit
`29923f9f370f09ebc00f96d8cca375007a18e7d5`.  Its MATLAB graph uses
`FTYPE=["L1","L5"]` and the same TDCP block in both `fgo_gnss.m` and
`fgo_gnss_imu.m`.  The official preprocessing stores accumulated delta range
as cycles, while the residual helper converts it back to metres.  Native
Android parsing similarly stores ADR-derived carrier phase in cycles and
forms metre-valued prepared observations before the FGO problem builder.

The official custom `TDCPFactor_XXCC` error is predicted-minus-measured:

```text
los' * ((x2-inix2) - (x1-inix1)) + (C2[0]-C1[0]) - tdcp
```

The native ordinary factor is also predicted-minus-measured:

```text
range2 + C2 - range1 - C1 - delta_carrier_m
```

The Eigen path uses the opposite residual sign when assembling a weighted
row, but its squared robust cost is unchanged.  These representations are
not byte-identical because the official factor is an initial-point LOS
linearization and the native factor evaluates endpoint ranges directly.

## Requested term-by-term classification

Status values are restricted to `exact`, `equivalent`, `divergent`, and
`unresolved`.  “Equivalent” means the physical contract and sign/unit
convention agree for the audited Pixel5 routes; it does not assert identical
floating-point instruction order.

| term | status | official source evidence | native source evidence and consequence |
| --- | --- | --- | --- |
| Carrier cycles to metres, wavelength and frequency selection | equivalent (supported target signals) | `functions/gnsslog2obs.m:152-164` sets `lambda=c/freq`; `:183-184` sets `L=ADR/lambda` and `D=-rate/lambda`; `:254-268` maps each constellation to its nearest supported frequency | `src/io/android_raw_gnss.cpp:635-642,907-975` uses `c/raw_frequency`, preserves the device ADR policy, and stores carrier cycles; `include/libgnss++/core/signals.hpp:110-139` supplies typed wavelengths and GLONASS channels. The target supported signals have the same metre conversion, although the general frequency-selection implementations are not byte-identical. |
| Epoch differencing order and sign | equivalent | `fgo_gnss.m:185-188` and `fgo_gnss_imu.m:307-310` use `resL(i+1)-resL(i)`; `TDCPFactor_XXCC.h:62-76` uses predicted-minus-`tdcp` | `src/algorithms/fgo_problems.cpp:1446-1452` uses current-minus-previous; `src/algorithms/fgo_gtsam_internal.hpp:3403-3425,3497-3500` uses predicted-minus-measured; `src/algorithms/fgo.cpp:971-999` mirrors the row residual sign. |
| Receiver clock drift/bias difference | equivalent for Pixel5 C0 | Official `fgo_gnss.m:166-176` and `fgo_gnss_imu.m:263-275` construct C/D keys; `TDCPFactor_XXCC.h:63-70` and `:73-76` select only C0. Official XXDD's `dt*(D1+D2)/2` is a different phone branch | Native `TimeDifferencedCarrierFactorSourceClockArm` uses `sourceClockComponentJacobian(0)` at `fgo_gtsam_internal.hpp:3481,3490-3500`; the target Phase101/118 source-clock path therefore binds C0 at each endpoint. Native D remains available to the graph but is not silently substituted into the Pixel5 XXCC TDCP equation. |
| Satellite-clock difference | equivalent in sign and metre placement | `+dts` is embedded by `Gobs.m:1151-1158` in `resL=L*lambda-(range-dts)`; official TDCP is the difference of that residual (`fgo_gnss.m:185-186`) | `src/algorithms/fgo_problems.cpp:485-502` obtains the transmit-time state and clock, `:542-573` converts clock bias to metres, and `:659-665` adds it to carrier before `:1446-1449` differences it. Native atmosphere terms are a separate mismatch below. |
| Satellite motion, LOS, and geometric range | divergent | Official `Gsat.m:259-272` obtains range/LOS at the receiver seed; `fgo_gnss.m:179-186` passes one LOS vector, and `TDCPFactor_XXCC.h:62-76` linearizes about `inix1/inix2` | Native stores both endpoint satellite positions (`fgo_problems.cpp:1495-1502`) and evaluates two endpoint plain ranges (`fgo_gtsam_internal.hpp:3397-3407`) with state-dependent LOS in the Eigen path (`fgo.cpp:949-965`). This is a genuine endpoint/linearization difference, not selected because changing it would also change the factor equation and geometry contract. |
| Sagnac / Earth rotation | divergent representation (first-order physical intent is equivalent) | Official `geodist.m:18-20` documents Sagnac-inclusive distance; its MEX wrapper calls RTKLIB `geodist` at `src/mex/geodist.c:73-80`. Official `Gsat.m:304-306` also includes the range-rate Sagnac term | Native `fgo_internal.hpp:278-289` rotates satellite coordinates using signal travel time, then `fgo_gtsam_internal.hpp:2331-2343` deliberately uses a plain norm to avoid applying GTSAM Sagnac twice. Native coordinate `geodist` is analytical Sagnac (`coordinates.hpp:121-138`), but the TDCP backend uses the pre-rotated/plain-norm representation. The source does not justify replacing one representation in this candidate. |
| Ionosphere/troposphere cancellation and handling | divergent; selected candidate | `Gobs.m:1151-1158` defines `resL` without atmosphere; `:1160-1165` defines atmospheric `resLc` separately. Both official TDCP blocks (`fgo_gnss.m:185-194`, `fgo_gnss_imu.m:307-316`) consume `resL`, never `resLc` | `fgo_problems.cpp:518-540` computes ionosphere/troposphere and `:659-665` sets `corrected_carrier=carrier*wavelength+satellite_clock-trop+iono`; `:1446-1452` differences that field. The future candidate removes only those atmospheric additions from the ordinary TDCP measurement field; P/D, standalone carrier, geometry, and all factors remain unchanged. |
| Time-interval scaling | equivalent for Pixel5 XXCC | Official computes `dtgps` at `fgo_gnss.m:163-164` / `fgo_gnss_imu.m:263-275`; XXCC has no D term. The special XXDD factor uses `dt*(D1+D2)/2` in `TDCPFactor_XXDD.h:64-74`, but Pixel5 takes XXCC in the official lists | Native stores `factor.dt_s` and uses it for temporal gap admission (`fgo_problems.cpp:1411-1425,1521-1525`); the ordinary C0 source-clock factor has no D scaling (`fgo_gtsam_internal.hpp:3473-3500`). |
| Residual and noise units | divergent overall; residual units agree | Official `gnsslog2obs.m:183`, `Gobs.m:1151-1158`, and `fgo_gnss.m:186-188` make TDCP metres and attach per-observation `obserr.L` | Native `delta_carrier_m` and factor residual are metres (`fgo_problems.cpp:1446-1452`; `fgo_gtsam_internal.hpp:3403-3407`), but Phase112/118 freezes fixed `tdcp_sigma_m=0.03 m` (`fgo_config.hpp:393-394`; app recipe `gnss_fgo_imu_no_base.cpp:7581-7590`). Official `obserrmodel.m:32-37` produces elevation/SNR-dependent L noise. The fixed-vs-dynamic noise difference is retained, not re-frozen here. |
| Whitening | equivalent at the contract level | Official creates diagonal sigma noise and robust noise (`fgo_gnss.m:85-86,187-188`; same pattern in `fgo_gnss_imu.m:98-99,309-310`) | Native GTSAM attaches `makeNoise(factor.sigma_m, ..., threshold)` (`fgo_gtsam_backend.cpp:1224-1226`); Eigen normalizes by `raw_residual/sigma` before robust scaling (`fgo.cpp:971-999`). The sigma value itself is the divergence recorded in the prior row. |
| Huber application point | equivalent structurally | Official wraps the scalar TDCP diagonal noise at `fgo_gnss.m:187-188` and `fgo_gnss_imu.m:309-310`; Type-to-k mapping is `parameters.m:119-125` | Native applies the ordinary TDCP threshold only during noise creation (`fgo_gtsam_backend.cpp:46-52,1224-1226`) and the equivalent normalized scalar path in `fgo.cpp:973-979`. Phase118's Highway `k=0.5` selector is kept; no new k or sweep is proposed. |
| Slip and outlier gating | divergent | Official upstream carrier mask uses SNR, slip, valid, and multipath (`exobs.m:53-67`), removes `abs(diff(L))>2e4` cycles (`:69-75`), and applies the Doppler-carrier `dDL` 1.5 m endpoint mask (`exobs_residuals.m:89-99`). The FGO block additionally checks finite `resL` and `~clkjump` (`fgo_gnss.m:185-186`) | Native parser records raw masks and loss-of-lock (`android_raw_gnss.cpp:900-906,965-981`); builder gates gap, clock discontinuity, loss-of-lock, finite values, and code-phase jump through `tdcp_contract.hpp:34-67`, with the frozen 10 m threshold (`fgo_config.hpp:839-840`). Native has no exact source-equivalent `dDL` 1.5 m predicate in this builder. Changing admission would alter factor counts, so it is not the selected candidate. |
| Same-satellite / same-signal pairing | exact for adjacent pairs | Official loops epoch `i`, satellite `j`, and each `f`, then uses the same `j,f` at `i+1` (`fgo_gnss.m:179-186`; same in IMU `:301-308`) | Native keys the previous lookup by `(SatelliteId, SignalType)` (`fgo_problems.cpp:1428-1444`) and only creates current-minus-previous pairs after an exact key hit. No duplicate or cross-signal pairing is introduced. |

## Candidate boundary and alternatives

Candidate ID: `phase120-official-tdcp-resl-atmosphere-cancellation-v1`.

The candidate is selected because the mismatch is explicitly stated by the
official source (`resL` versus `resLc`) and can be isolated to the ordinary
TDCP measurement preparation without changing the graph topology, pair
admission, factor equation, wavelength, C0/D state topology, fixed 0.03 m
sigma, Phase118 Type-dependent Huber k, raw-base correction, C7, QR, Pixel5
offset, IMU, filter, initialization, or LM settings. It is default-off and
must fail closed on nonfinite or missing source-derived values.

Two other source observations were considered but are not frozen:

1. The official initial-LOS TDCP linearization versus native nonlinear
   endpoint ranges is a larger geometry/equation change and is coupled to the
   Sagnac representation. It cannot be safely combined with the selected
   measurement-only candidate.
2. Official carrier slip/dDL admission and per-observation L noise differ
   from the already frozen native gate and Phase118 fixed sigma. Changing
   either would change factor population or the Phase117/118 recipe rather
   than isolate the equation-normalization mismatch.

No truth score, sealed accuracy value, parameter sweep, or runtime evidence is
used to choose this candidate. There is exactly one candidate in the
companion freeze record.

## Future implementation contract (not authorized here)

If separately authorized, the opt-in branch may derive an ordinary TDCP pair
measurement from the existing raw carrier phase, retained wavelength, and
satellite clock metre term only. It must preserve the existing exact
`(SatelliteId, SignalType)` adjacent lookup and every existing reject reason,
factor count, endpoint position, C0/C7/D key, fixed sigma, robust threshold,
base correction, QR branch, Pixel5 final offset, and output withholding. The
legacy/default path must be semantically unchanged. No source-derived
atmosphere term may be reintroduced through a fallback, and no solution or
truth value may be used by the selector.

## Evidence pins

| item | SHA-256 or commit | role |
| --- | --- | --- |
| Official source tree | `29923f9f370f09ebc00f96d8cca375007a18e7d5` | pinned `gsdc2023` source commit |
| Official `fgo_gnss.m` | `5368e4056f70f448b728792c5e4c124b7f2afc1e00653492b807083bd3ccf0c3` | GNSS TDCP graph |
| Official `fgo_gnss_imu.m` | `c70090ccb8b27fc8ac7fd2929e2f995a14cfe7f089bb1fef067370e22051c3e3` | GNSS/IMU TDCP graph |
| Official `gnsslog2obs.m` | `665ce43d1fc3c5a9c3b3a6fd76f81539126e3ab897fbba484ab0ccd18964acff` | cycles, wavelength, frequency |
| Official `exobs.m` | `e22db383f372f26597333d674a12ee1e2adfdc97b00828fbbc5470903947f456` | carrier masks and cycle gate |
| Official `exobs_residuals.m` | `50c954a825edbdeaf5c9884486aff56c058f51d47610a06722d4f42b33095324` | dDL gate and residual preprocessing |
| Official `Gobs.m` | `be27fd1a075829a00f0aec5f32a0040a884077a8ac256a10a2bc26def255fc88` | resL/resLc equations |
| Official `Gsat.m` | `a56c3236646606c1b7e771180e9bc18b08f809fe8a6d71b795366b96207871f6` | range, LOS, atmosphere, range rate |
| Official `geodist.m` | `4ddced92aea78defd7eb8b2040dd23b1b0575eb0c1ac719a128c8db62ab4a91f` | Sagnac contract |
| Official `geodist.c` | `3929ec3d57575634a921a81957ce91f2dce9314457694e3bd7e00bd82ec765c4` | RTKLIB MEX call |
| Official `TDCPFactor_XXCC.h` | `cc8f5acabd43db25b5b4aca48f6c8ae6a0e2a399820fb950de9004555c9cd4b2` | C0 TDCP equation |
| Official `TDCPFactor_XXDD.h` | `dc6b92e824f439df56286b2c2f81c0acb744f698c45648baaf5da67d6b239aed` | D branch comparison |
| Native `android_raw_gnss.cpp` | `a020ce900db6a9d3a994cc3adb3dcebbe84cbf67c1ff9793991b7a75039529c5` | raw carrier conversion/masks |
| Native `fgo_problems.cpp` | `327fdc792afbeb29f150bc8566acaef63156c519d2dcb0a40f626659658edddc` | preparation/pair builder |
| Native `fgo_gtsam_internal.hpp` | `7aba732064d2cd6b3858c60226b84aa904423becdf66cb386e628fae16fc5afd` | ordinary TDCP factor |
| Native `fgo_gtsam_backend.cpp` | `43b80e75638f1b2d70907f1a9b36f6ee486c070c765ba733d852a022f2a58d2b` | factor/noise insertion |
| Native `tdcp_contract.hpp` | `1d56692d93f216703e2283b206b7bd0bca9c6593d51524d843fc6e385d96cbcf` | pair gates |
| Native `fgo_config.hpp` | `38ae28a5bdb398b0a764f9f09f8456107ddc0168654b1d1791879377cca24748` | fixed sigma and gates |
| Native `signals.hpp` | `bb8cd20d4a22581ed2beb83f4e1b4ed6d397f28a2d5104d4736778c5b279ba56` | typed wavelength map |

## Read accounting

| activity | Phase120 count/status |
| --- | ---: |
| Official/native source text and git metadata | source-only, permitted |
| New raw phone GNSS/IMU/navigation payload reads | `0` |
| New raw-base bytes, headers, or hashes | `0` |
| Truth payload, coordinate rows, or sealed solution rows read | `0` |
| MAT reads or generated MAT | `0` |
| Precomputed phone coordinates/trajectories | `0` |
| Solver/native process invocations | `0` |
| Accuracy calculations or score reads | `0` |
| PDC, Kaggle, or token access | `0` |
| Sweep, rerun, fallback, or tuning | `0` |

This record is an audit only.  The companion freeze must retain the exact
audit hash, keep the candidate default-off, and keep raw execution, truth,
accuracy, solution publication, and Kaggle authority false.
