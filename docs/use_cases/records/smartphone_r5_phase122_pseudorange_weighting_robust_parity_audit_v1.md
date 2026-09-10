# Phase122 pseudorange weighting and robust-loss parity audit

Status: sealed read-only source audit. This record compares the pinned
GSDC2023 MATLAB pseudorange path with the native Phase112/118 champion
source-quality path. It does not authorize implementation, raw execution,
solver execution, truth evaluation, accuracy calculation, solution
publication, or Kaggle action. No raw phone GNSS/IMU/navigation or raw-base
payload, MAT file, precomputed coordinate, PDC artifact, truth row, or
solution row was read.

## Decision

NO-CANDIDATE is frozen for Phase122.

The requested geometry-free weighting/robust comparison closes without a
remaining safe correction: the Phase112/118 direct/no-PDC observable-quality
lane already implements the official pseudorange SNR sigma model and the
official pseudorange Huber Type mapping. For the two primary Highway routes,
the native values are p85 SNR weighting with signal factors and Huber k=0.2,
exactly the official Highway P mapping. Changing sigma or Huber k would be a
duplicate of an existing source-parity contract or would be parameter tuning,
not a source-backed correction.

There are genuine source differences, but none is independently safe under
the frozen Phase112/118 boundary:

* The official P factor is a fixed initial-LOS affine Vector-X/C factor,
  whereas the native Phase101/118 factor is a relinearized range factor over
  the native Point3/Pose3 state and epoch-local C7 vector. Replacing its
  geometry is the already rejected Phase121 class and changes the factor
  equation/topology.
* Official RTKLIB satellite-clock output explicitly excludes code-bias
  correction. Native pseudorange preparation subtracts broadcast TGD/BGD
  group delay. In the Phase107/118 raw-base path the same native model is
  used for rover and base, so the term is coupled to base correction and to
  GNSS-first initialization. Removing it only on rover, or changing the
  base model as well, is not an isolated weight/Huber change and violates
  the frozen base/equation boundary.
* Official residual masks and the native source-quality masks have the same
  main thresholds but different seed/clock-group construction and
  application order. Changing that population would change factor counts
  and admission, which is explicitly outside Phase122.

Consequently the companion freeze records candidate_count=0. No Phase122
implementation or runtime contract is created.

## Scope and pinned baseline

The comparison is source text plus sealed structural metadata only. The
baseline is the already sealed Phase112/118 recipe:

* exact raw-only GNSS-first and main source-meter C7/D/C0D handoff;
* native direct observable quality, with no PDC bridge;
* raw-base pseudorange correction, applied once in the already frozen path;
* ordinary P factors, existing factor equations and admission;
* Phase118 fixed ordinary-TDCP sigma and official Type Huber mapping;
* Phase99 MULTIFRONTAL_QR main, Pixel5 final output offset, IMU,
  initialization, filtering, and LM contracts unchanged.

The target route settings are the two primary routes:

| route | source Type | official P Huber k | Phase118 native P Huber k |
| --- | --- | ---: | ---: |
| 2021-03-16-18-59-us-ca-mtv-a/pixel5 | Highway | 0.2 | 0.2 |
| 2022-04-01-18-22-us-ca-lax-t/pixel5 | Highway | 0.2 | 0.2 |

The Type rows are pinned source-list metadata from the Phase80 freeze and
were not inferred from a score, route name, truth, or result coordinates.
Phase118's dynamic TDCP sigma selector and all other candidate selectors are
not part of this audit.

## Official pseudorange construction

The official raw Android conversion in
functions/gnsslog2obs.m:152-184 stores pseudorange in metres as

    P = (tow_rx - tow_tx) * c

and retains C/N0, state, multipath, and carrier-frequency metadata. The
official exobs.m:30-51 code mask rejects low SNR, invalid tracking status,
multipath, P below 1e7 m, and P above 4e7 m; GLONASS uses the corresponding
TOD status bit while other systems use TOW.

Gsat computes satellite range, LOS, elevation, and atmosphere. Gobs.m then
defines the ordinary pseudorange residual and its atmospheric form:

    resP  = P - (range - dts)
    resPc = P - (range - dts + ionosphere + troposphere)

The FGO scripts use resPc for the pseudorange block and subtract the smoothed
base correction returned by functions/correct_pseudorange.m. That function
matches rover/base satellite and signal streams, forms the base resPc, smooths
it, and interpolates it at rover times. The official P factor receives this
residual, not a separately corrected absolute P.

The official RTKLIB satpos.m source notes that its satellite clock does not
include code-bias correction (TGD/BGD). There is no explicit TGD/BGD term in
the Gobs resPc equation above. This is a material measurement-model
difference from the current native code, not a weighting candidate.

The official PseudorangeFactor_XC factor has error

    e_off = los' * (X - initial_X) + h' * C - resPc

with key order X,C, h(0)=1 followed by h(sysidx)=1, and a fixed LOS captured
at the initial receiver geometry. Its position Jacobian is the stored LOS
and its clock Jacobian is h. This geometry is not altered by Phase122.

## Native Phase112/118 pseudorange construction

The native Android parser applies the source-compatible low-SNR,
multipath, range, status, and carrier-state masks while converting raw
pseudorange to metres. In the direct Phase80/112/118 lane,
fgo_problems.cpp:430-638 first requires an eligible supported signal,
finite positive pseudorange, usable navigation/health, and the exact
upstream SNR/elevation contract. It computes transmit time from P, applies
the satellite-clock iteration and build-time Earth-rotation satellite
position, then forms

    corrected_P_native =
        P + satellite_clock_m - ionosphere_m - troposphere_m - group_delay_m

The ordinary native P factor in the Phase101/118 source-parity lane predicts
range plus the selected receiver-clock component from the epoch-local C7
vector against that corrected measurement. (The legacy scalar/ISB lane is not
the audited champion.) The GTSAM source-clock factor is:

    e_native = plain_range(receiver, satellite)
               + h_C7' * C - corrected_P_native

The Phase118 raw-base contract subtracts the existing in-domain base
correction from this measurement exactly once. The base builder uses the
same atmosphere and group-delay terms in its base residual
(base_pseudorange_compensation.cpp:235-252), so group delay is not a
free-standing rover-only knob in the frozen recipe.

The native backend attaches the existing scalar noise and robust wrapper at
fgo_gtsam_backend.cpp:957-984. The native source-clock mapping at
fgo_gtsam_internal.hpp:1304-1340 is the seven-component C7 mapping:
GPS L1=0, GLONASS L1=1, Galileo E1=2, BeiDou B1=3, GPS L5=4,
Galileo E5a=5, and BeiDou B2a=6. The same component is used by the
source-clock P factor and CCDD state topology.

The native ordinary P factor is a nonlinear plain Euclidean range factor.
For the Phase101/118 IMU graph it is the Pose3/lever-arm variant; the
position Jacobian is recomputed on each evaluation. This is a geometry and
equation difference, not a Phase122 weight change.

## Official weighting and robust contract

The official parameters.m:22-44 selects SNR models for P, D, and L:

    sn_base = 10^(-(SNR - p85_band) / 20)
    sigma_P = sn_base * signal_factor

For P the ratio is one. The source factor order is
L1/G1/E1/B1/L5/E5/B2a with values
0.8/1.5/0.8/0.8/0.5/0.5/0.5. The
obserrmodel.m implementation obtains the 85th percentile over the active
observation matrix for each frequency type. The official P factor wraps
this scalar diagonal sigma in the GTSAM robust noise model.

Official P Huber k is set by parameters.m:77-83:

| official Type | P Huber k |
| --- | ---: |
| Street | 0.1 |
| Mix | 0.1 |
| any other Type, including Highway | 0.2 |

The Type is a run-level setting, not an SNR, frequency, constellation, or
individual-factor selection.

The native observable_upstream_preprocessing.hpp:27-199 ports the same
source constants and formula:

    p85 = 85
    SNR denominator = 20 dB
    sigma_P = 10^(-(SNR-p85)/20) * signal_type_factor

Its supported target factors are GPS/Galileo/BeiDou L1=0.8, GLONASS L1=1.5,
and GPS/Galileo/BeiDou L5=0.5. BDS B1C shares the official B1 factor. An
unsupported signal factor is NaN and is rejected rather than silently
folded into C7.

The Phase80 direct-quality resolver in
apps/native/gnss_fgo_imu_no_base.cpp:369-411 and :7679-7712 sets
use_upstream_observable_quality, p85=85, minimum SNR=20 dB-Hz, minimum
elevation=5 degrees, and the route P robust threshold. Its exact target
mapping is Highway=0.2, matching the official table. The Phase112/118
recipe enables this direct lane and does not enable the older PDC quality
path.

Native GTSAM makeNoise creates an isotropic metre-valued sigma and, when
enabled, wraps Huber with the supplied threshold. Therefore the
Phase112/118 GTSAM P whitening and robust application point are the same
normalized-residual contract as the official scalar diagonal sigma plus
Huber wrapper. The Eigen implementation independently normalizes the metre
residual by sigma before its robust scale; this is a solver backend
representation, not a candidate for the QR recipe.

## Term-by-term classification

Status values are restricted to exact, equivalent, divergent, and
unresolved. Equivalent means the physical contract agrees for the pinned
target source lane; it does not assert identical floating-point instruction
order.

| term | status | official evidence | native evidence and consequence |
| --- | --- | --- | --- |
| Raw pseudorange representation and metre unit | equivalent | gnsslog2obs.m:152-184 forms P from receive/transmit time times c | android_raw_gnss.cpp:880-985 retains finite Android raw P in metres before FGO preparation; parser and source are not byte-identical, but the target unit is the same |
| Corrected P measurement formula | divergent | Gobs.m:1151-1158 uses P+dts-range-ion-trop, then correct_pseudorange.m subtracts base pc | fgo_problems.cpp:542-573 uses P+satellite_clock-ion-trop-group_delay; Phase107/118 subtracts the native base model later; native TGD/BGD term is not present explicitly in official resPc |
| Satellite clock sign and unit | equivalent | Gobs dts is a metre-valued satellite clock term subtracted inside range-dts | native converts satellite clock seconds to metres and adds it to measured P before range prediction; fgo_problems.cpp:542-573 |
| Ionosphere and troposphere signs | equivalent at the residual contract | official resPc has +ion/+trop inside the subtracted predicted term, hence -ion/-trop in measured corrected P | native corrected_P subtracts ionosphere and troposphere; the ordinary factor predicts range against that measurement; model implementations and geometry are not claimed instruction-identical |
| Broadcast TGD/BGD/group delay | divergent | satpos.m:20-28 explicitly says satellite clock excludes code bias; no TGD/BGD term appears in Gobs resPc | fgo_internal.hpp:243-268 and base_pseudorange_compensation.cpp:37-69 subtract broadcast group delay; same model is used by the frozen rover/base correction path |
| SNR percentile and exponent | equivalent for Phase112/118 direct quality | obserrmodel.m:11-23 uses p85 and denominator 20 with P ratio one | observable_upstream_preprocessing.hpp:89-125,142-159,186-199 implements MATLAB percentile and identical SNR scale; direct app fixes p85=85 |
| Signal/frequency/constellation sigma factor | equivalent for supported target signals | sysfreq2sigtype.m:6-17 and parameters.m:22-44 map L1/L5 factors | signalTypeFactor/bandForSignal in observable_upstream_preprocessing.hpp maps the target GPS/GLO/GAL/BDS bands and BDS B1C to the same factors; unsupported factor is fail-closed |
| Elevation-dependent sigma weighting | equivalent in the target recipe | parameters.m:22-24 selects P_model=sn, so P sigma is not multiplied by the elevation model | upstream quality takes the SNR sigma branch; pseudorange_elevation_sigma_power is only the legacy non-upstream branch, which is off in Phase112/118 |
| Elevation gate | equivalent threshold, geometry source divergent | parameters.m:62-66 selects 5 degrees when L5 is enabled; Gsat supplies elevation | direct app sets upstream_min_elevation=5; native computes it from its seed and rotated satellite, so threshold agrees but exact admission geometry does not |
| C/N0 gate | equivalent threshold, missing-value behavior unresolved | exobs.m:30-51 uses Pmask_sn=20 dB-Hz | native direct lane requires finite SNR >=20 and finite positive source sigma; native parser maps absent C/N0 to its sentinel, so absent/nonfinite semantics are not proven identical |
| Raw status, multipath, and P range gate | equivalent for supported Android fields | exobs.m:30-51 checks code-lock/TOD/TOW, multipath, and 1e7<P<4e7 | android_raw_gnss.cpp applies corresponding source masks while parsing; later native eligibility/health checks are additional gates |
| P-D pair and residual outlier masks | divergent | exobs_residuals.m:34-80 uses dDP thresholds L1=40/L5=20 and residual median by obs.sys with clk/isb | observable_upstream_preprocessing.hpp:300-390 ports dDP thresholds; fgo_problems.cpp:1019-1063 centers residuals from native SPP-seed residuals by clock group and band. The same thresholds do not prove identical retained rows |
| Common-satellite/signal admission | equivalent key identity, factor population not exact | official loops the same satellite and frequency matrix and requires finite residuals in fgo_gnss.m:123-151 | native uses exact (SatelliteId, SignalType) keys plus eligibility, health, secondary-code, upstream masks, and base-domain checks; changing these would change factor count |
| C7 constellation/component slot | exact for the supported source mapping | sysfreq2sigtype.m gives L1 0/1/2/3 and L5 4/5/6, with unsupported 7 | sourceClockComponentFor gives the same seven slots and explicitly rejects unsupported components; assignment for GPS L1 preserves the official h(0)=1 convention |
| P sigma unit and whitening | equivalent in the frozen GTSAM lane | official sigma is a scalar metre noise wrapped by robust diagonal noise at fgo_gnss.m:123-151 | native sigma_m is metre-valued, makeNoise applies the existing isotropic sigma and robust wrapper at fgo_gtsam_backend.cpp:957-984 |
| Robust loss kind and P Huber k | exact for the two target routes | parameters.m:77-83 selects Huber and Highway k=0.2 | direct route table selects Highway P k=0.2; native makeNoise uses GTSAM Huber with that threshold |
| Robust application point | equivalent | official wraps the scalar P noise after constructing obserr.P | native wraps factor sigma at backend insertion; normalized residual application is unchanged by Phase118 |
| Linearization geometry | divergent | PseudorangeFactor_XC.h:15-64 stores initial LOS and affine X/C residual | native plainRange and SourceClock/SourceClockArm factors recompute range/Jacobian over Point3/Pose3 state; changing this violates the frozen equation/graph boundary |

## Candidate assessment

Three source observations were considered; none is frozen:

1. Official SNR P sigma or P Huber mapping. Rejected as a duplicate:
   Phase80/112/118 direct quality already uses the official p85, factor table,
   and exact Highway/Street Type mapping. A new sigma or k would be tuning
   or a second selector, not a correction.
2. Remove TGD/BGD from native ordinary P to match official satpos/Gobs.
   Rejected as not independently isolatable under the Phase112/118 contract:
   the same native group-delay model is used in the raw-base correction, where
   rover/base terms cancel by construction, and it also participates in
   GNSS-first SPP initialization. A rover-only removal is asymmetric; a
   rover-plus-base removal changes the frozen base measurement contract.
3. Replace native nonlinear P range with the official fixed-LOS
   PseudorangeFactor_XC. Rejected as a geometry/equation and key/value
   topology change. It is the same class already closed by the Phase121
   geometry audit and cannot be made a weight-only candidate.

The residual-mask center/group difference is documented as a divergent
admission detail but is not promoted to a fourth candidate: changing it
would alter factor population, not isolate weighting or robust loss.

Therefore candidate_count=0, implementation_authorized=false, and no
Phase122 selector, source patch, runtime, or structural contract is
authorized. The Phase112/118 pseudorange P quality and robust contract
remains the baseline.

## Evidence pins

### Official source

| source | SHA-256 | relevant lines |
| --- | --- | --- |
| output/reproducibility-cache/gsdc2023/fgo_gnss.m | 5368e4056f70f448b728792c5e4c124b7f2afc1e00653492b807083bd3ccf0c3 | 50-70,123-151 |
| output/reproducibility-cache/gsdc2023/fgo_gnss_imu.m | c70090ccb8b27fc8ac7fd2929e2f995a14cfe7f089bb1fef067370e22051c3e3 | 63-106,202-221 |
| output/reproducibility-cache/gsdc2023/functions/gnsslog2obs.m | 665ce43d1fc3c5a9c3b3a6fd76f81539126e3ab897fbba484ab0ccd18964acff | 152-184 |
| output/reproducibility-cache/gsdc2023/functions/exobs.m | e22db383f372f26597333d674a12ee1e2adfdc97b00828fbbc5470903947f456 | 30-51 |
| output/reproducibility-cache/gsdc2023/functions/exobs_residuals.m | 50c954a825edbdeaf5c9884486aff56c058f51d47610a06722d4f42b33095324 | 1-80 |
| output/reproducibility-cache/gsdc2023/functions/obserrmodel.m | 43d0671a25c81fa6d7df0e1fbe81bf48a14eb3a35c8399600200b44961a7f9ef | 1-37 |
| output/reproducibility-cache/gsdc2023/functions/parameters.m | 518925e9c75c7a14fceb5cd99432fe883311e0d64c72b94396744ac765120f52 | 22-44,77-83 |
| output/reproducibility-cache/gsdc2023/functions/correct_pseudorange.m | b0536ccff478b0aff253448ffb7a203c715b8064dd8dc85898e38f1f05d0441e | 1-34 |
| output/reproducibility-cache/gsdc2023/functions/sysfreq2sigtype.m | 5ec1d299e04b45f604921cbae3b1fac8f78f3f76a0152496d86011a32f1d0079 | 1-17 |
| output/reproducibility-cache/MatRTKLIB/+gt/Gobs.m | be27fd1a075829a00f0aec5f32a0040a884077a8ac256a10a2bc26def255fc88 | 1151-1165 |
| output/reproducibility-cache/MatRTKLIB/+gt/Gsat.m | a56c3236646606c1b7e771180e9bc18b08f809fe8a6d71b795366b96207871f6 | 259-306 |
| output/reproducibility-cache/MatRTKLIB/+rtklib/satpos.m | 20b7fd78065a22d9d2e02b0f394aabef2437e991ed2c63b04c65194a96827f5e | 20-28 |
| output/reproducibility-cache/MatRTKLIB/src/mex/geodist.c | 3929ec3d57575634a921a81957ce91f2dce9314457694e3bd7e00bd82ec765c4 | 50-80 |
| output/reproducibility-cache/gtsam_gnss/src/PseudorangeFactor_XC.h | 7f467698f2239818724eba4485b830fbc543586bbee06a21b8b2c03ae5b56415 | 15-64 |

### Native source

| source | SHA-256 | relevant lines |
| --- | --- | --- |
| apps/native/gnss_fgo_imu_no_base.cpp | df77ab6556f90d6d9a8464d360e85d2a49a50d6fc04ef396833f354f821a1176 | 369-411,7679-7712 |
| src/io/android_raw_gnss.cpp | a020ce900db6a9d3a994cc3adb3dcebbe84cbf67c1ff9793991b7a75039529c5 | 880-985 |
| include/libgnss++/algorithms/observable_upstream_preprocessing.hpp | 4e20a30730aeca9c93d7108e4edeb7dbe18d05d2f5842d3a4326e29c61f4b60f | 27-199,300-390 |
| src/algorithms/fgo_problems.cpp | 5675e82fc595933da4e7c14ad468ae69437dc04bac79a7ef55f1edfa03e098da | 430-638,1019-1063 |
| src/algorithms/fgo_internal.hpp | ee3a2bed13971c9bce52d043a293990703b92a2bc9d916376a896eadbf385ca7 | 224-268 |
| src/algorithms/fgo_gtsam_internal.hpp | 7aba732064d2cd6b3858c60226b84aa904423becdf66cb386e628fae16fc5afd | 1304-1340,2330-2478 |
| src/algorithms/fgo_gtsam_backend.cpp | c0da12d7a1d5eeb9492b02ad80f0e9868cbda32e452bbdd9c3f90cc57b4db0fa | 957-984 |
| src/algorithms/fgo.cpp | 145ffa3958d3a867ba53d962f07d346f9efa36e72140ad03d7a404ffdd3b5023 | 705-746 |
| src/algorithms/base_pseudorange_compensation.cpp | f3c3c859793c4bed9c7ae93825d32565f20eab0badb09412a4fab2a5930c9817 | 235-252 |
| include/libgnss++/algorithms/fgo_config.hpp | e864b31ab04dc0f8f0f4764b4e89406282349012283944ed64e88c01ee8163e6 | 42-50,330-333,490-493 |
| include/libgnss++/algorithms/galileo_group_delay.hpp | 222066be7861254ae61b2b8201882279ba34a42dae044107bd1ee801bcbfd907 | 30-84 |

### Sealed baseline provenance

| record | commit | SHA-256 | permitted use |
| --- | --- | --- | --- |
| Phase80 direct P-quality freeze | 57cf7698c36256a535ac388cbc45dd93afecc9bb | 9ebdde21fe7fd955029243d3bdbba3a9f9d3832388d67480760394177c8f685e | source-quality contract and Type mapping metadata |
| Phase112 structural result | dfa9c3a186c9d8633dc572192b513b584f18800 | 087b1bf51b4e584b86a7b558560df4ceb78e19b01e572f2d60aeef4c3d759ba0 | recipe/progress metadata only |
| Phase118 source-parity freeze | 5fcc06ab64189dd5dfb8001664bdb8496f85224f | c6f4fda2abe170417610d4fd1a4ae8a1a618d07096ff0432669081ab06a1cf77 | fixed TDCP/Type recipe provenance only |
| Phase118 structural contract freeze | 25f3bada02c4073175054b5d668de02020761208 | 267a2f51dea06f7d29d5b27be1f7ffd84febac1c72a9fbe2676c5f5a2649f81b | raw recipe/invariant provenance only |

The official source tree is pinned to commit
29923f9f370f09ebc00f96d8cca375007a18e7d5.

## Read accounting and authorization boundary

| resource or action | Phase122 count/status |
| --- | ---: |
| official/native source text and git metadata | source-only, permitted |
| sealed Phase80/112/118 metadata | metadata-only, permitted |
| new raw phone GNSS/IMU/navigation payload reads | 0 |
| new raw-base bytes, header, or correction reads | 0 |
| truth payload or coordinate-row reads | 0 |
| solution-row reads or publication | 0 |
| MAT reads or generation | 0 |
| precomputed phone-coordinate reads | 0 |
| PDC reads | 0 |
| native solver invocations | 0 |
| accuracy/evaluator invocation or score read | 0 |
| Kaggle/token access | 0 |
| reruns, fallbacks, or repairs | 0 |
| parameter/weight/Huber sweeps | 0 |
| code implementation | 0 |

This record is an audit only. The companion freeze keeps every
implementation, raw, solver, truth, accuracy, publication, and Kaggle
authority false. A future candidate must first isolate one source-backed
dimension without changing the Phase112/118 graph, equation, admission,
base, C7/D/C0D, QR, IMU, offset, or LM contracts.
