# Phase511 — native raw-input group-delay scale

Added scripts/native_group_delay_audit.cpp. All three raw runs exited 0.
No truth, MAT, saved positioning input, receiver position, optimizer or IMU
used. Satellite states are calculated from broadcast navigation in memory only.
The Android loader has enriched-pseudorange verification disabled and pixel5
device selection. Outputs are aggregate counts/metres, not positions.

Important scope correction: an initial diagnostic used source epoch message
selection. Phase507 manifest/summary instead has native_rover_epoch_states=false;
FGOConfig defaults use_source_rover_epoch_states=false. The diagnostic was
corrected and all three routes rerun using the actual default path: raw code
transmission time, satellite state/clock, clock-corrected transmission time,
second satellite state, getEphemeris. Initial results are superseded. Although
default TGD ranges barely changed, Galileo alternate changed-row counts did.

The helper is the actual fgo_internal::groupDelayCorrectionMeters, not a copied
formula. This is raw valid/eligible/healthy admission, NOT final graph admission:
no SNR/elevation/residual/adjacent masks or failed-seed exclusions are applied.
The metres below are corrections, not positioning errors or score predictions.

| Route | Signal | Rows | Min m | Mean m | Max m | E1 alternate changed | Max alternate delta m |
|---|---|---:|---:|---:|---:|---:|---:|
| H | GPS L1CA | 25666 | -3.2108400641 | -0.704492200445 | 1.53561916109 | 0 | 0 |
| H | GPS L5 | 19962 | -2.51283135451 | -0.251257959303 | 1.53561916109 | 0 | 0 |
| H | GLO L1CA | 11908 | 0 | 0 | 0 | 0 | 0 |
| H | GAL E1 | 16089 | -1.04701306438 | 0.636303334011 | 1.88462351589 | 14773 | 0.209402612879 |
| H | GAL E5A | 15474 | -1.04701306438 | 0.628501043786 | 1.88462351589 | 0 | 0 |
| H | BDS B1I | 19623 | -5.5761397188 | 1.59618133431 | 4.9765548028 | 0 | 0 |
| U | GPS L1CA | 8899 | -4.60685748329 | -2.29038470878 | 1.11681393534 | 0 | 0 |
| U | GPS L5 | 3111 | -2.65243309643 | -0.111376253115 | 1.11681393534 | 0 | 0 |
| U | GLO L1CA | 5516 | 0 | 0 | 0 | 0 | 0 |
| U | GAL E1 | 9980 | -4.0484505156 | 0.130096793658 | 1.88462351589 | 8423 | 0.349004354794 |
| U | GAL E5A | 7885 | -4.0484505156 | 0.208269510596 | 1.88462351589 | 0 | 0 |
| LAX | GPS L1CA | 13483 | -4.74645922519 | -2.10511516426 | 1.53561916109 | 0 | 0 |
| LAX | GPS L5 | 4857 | -2.23362787068 | 0.251955707154 | 1.53561916109 | 0 | 0 |
| LAX | GLO L1CA | 8638 | 0 | 0 | 0 | 0 | 0 |
| LAX | GAL E1 | 12198 | -1.25641567726 | -0.0729080912347 | 2.30342874164 | 6358 | 0.139601741918 |
| LAX | GAL E5A | 11530 | -1.25641567726 | -0.0614526141456 | 2.30342874164 | 0 | 0 |

Epoch counts H/U/LAX: 3140/1102/1466. Missing-state and missing/unhealthy rows
were zero for all three. Enum labels checked against core/types.hpp.

## Reproduction

Build from repository root:

```sh
c++ -std=c++17 -O0 -Iinclude -I/usr/include/eigen3 scripts/native_group_delay_audit.cpp -Wl,--start-group build/libgnss_lib.a build/libgnss_lib_solvers.a -Wl,--end-group -L/home/sasaki/.local/lib -Wl,-rpath,/home/sasaki/.local/lib -lgtsam build/third_party/ginan-iers2010/libginan_iers2010.a build/third_party/sofa/libsofa.a -lboost_serialization -lboost_timer -lboost_chrono -lcholmod -lpthread -o /tmp/native_group_delay_audit_phase511
```

Invoke with brdc.nav then device_gnss.csv under
output/smartphone-r5/phase37-pixel5-repeatability-v1/routes/ROUTE/pixel5/inputs/.
Routes: H=2021-08-24-20-32-us-ca-mtv-h,
U=2023-03-08-21-34-us-ca-mtv-u, LAX=2022-04-01-18-22-us-ca-lax-t.
Runtime LD_LIBRARY_PATH prepends /home/sasaki/.local/lib and preserves old value.
Initial build exposed missing helper prerequisite includes; corrected locally.
A mistyped link path also failed before execution and was corrected.

SHA256 script: 06222fdb62f2318db4b9cc155e0e0d78c05d7e8ec30d42966f0c48a40ba1520f

SHA256 executable: ec1d16f64b5a351124f0cd7f44e5a06cfed33500106508a6f5ca54e84b572ea3

Input SHA256 (navigation, raw CSV):

- H: 147d948f0eba3bf09e295e7f67fbe8db60c25e236bc3c7d958dc933483f10909,
  46482b82db0992c1f063dbd9cf697268605234d3e38bcbd23525fd4b60bc17a7
- U: 8893ef62fecdd9986f6b2cf1b7b980defa6430da5801aafcab4ecf4c99a03b92,
  a0fc8e71bdfc03be61b99efcd7d41fbba8ffec126df78b55243f681fd211f204
- LAX: 443d3d5a73f4895b83e576e24a568f4658f869e79a480856de7f0717763dcbe6,
  50362c01bff3e0bb7088e54021164591cd750227ed97c2fd7d95d763a08798f1

Next investigate GPS L5 broadcast-clock/code-bias convention (current helper
uses identical primary TGD across GPS signals). These aggregate ranges alone
do not establish a bug or justify a TGD-off experiment. No candidate promotion
or accuracy claim; the overall goal remains unmet.
