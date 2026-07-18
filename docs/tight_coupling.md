# RTK-hosted GNSS/IMU tight coupling

## Objective

The tight-coupling path keeps `RTKProcessor` as the estimator that owns the
baseline, single-difference ambiguity, ionosphere, lock, hold, and ambiguity
resolution state. IMU propagation replaces the kinematic path's unconditional
per-epoch SPP position reseed; it does not create a second ambiguity filter.

Every new behavior is opt-in. With all tight-coupling options disabled, the
existing RTK solution stream must remain bit-identical.

## State ownership

| State or responsibility | Owner |
|---|---|
| Baseline position and covariance | `RTKProcessor` |
| SD ambiguities, ionosphere states, lock/hold lifecycle, AR | `RTKProcessor` |
| IMU samples and interval selection | `TightCouplingProcessor` |
| Attitude, velocity, accelerometer bias, gyroscope bias | `TightCouplingProcessor` |
| IMU interval transition and accumulated process noise | `ImuPreintegrator` |
| ZUPT, NHC, lever arm | `TightCouplingProcessor` using existing fusion helpers |

The first three RTK states remain position. Velocity states, when introduced,
are appended after the existing RTK state layout so ambiguity indices and the
AR machinery do not move.

## Epoch sequence

For GNSS epoch `k`:

1. Start from the previous accepted RTK FLOAT posterior, never from the loose
   ESKF's absolute position.
2. Integrate body-FLU IMU samples from epoch `k-1` to `k` in the local ENU
   frame. Gyroscope input is already radians per second.
3. Transform the predicted antenna displacement and covariance to ECEF.
4. Apply an RTK time update that preserves position-to-ambiguity covariance.
5. Run the existing DD measurement construction, Kalman update, validation,
   and ambiguity resolution.
6. Validate an integer candidate with the self-reference-free CP-vs-PR test
   before a fixed solution can be fed back.
7. Re-anchor the private INS navigation state from the accepted RTK posterior,
   applying the configured body-FLU IMU-to-antenna lever arm.

M1 initially implements steps 1-4 with a private external INS update. M2 adds
step 6 before M3 enables the complete closed loop.

## Coordinate and covariance contract

- IMU body frame: FLU (Forward, Left, Up).
- Mechanization frame: local ENU with gravity along negative Up.
- RTK state frame: ECEF baseline relative to the configured base station.
- Lever arm: IMU origin to GNSS antenna, expressed in body FLU.
- `ImuPreintegrator` returns the predicted ENU nominal state, the accumulated
  15-state transition matrix, and process-noise covariance for one interval.
- ENU/ECEF rotation is owned by `TightCouplingProcessor`; the preintegrator has
  no base-station or antenna knowledge.
- M1 maps the position block into RTK without zeroing position covariance rows
  or columns. A configurable diagonal floor handles the regularization that
  the legacy wide reset previously supplied.

## Invalid interval policy

An interval is invalid when it is uninitialized, contains non-finite IMU data,
has non-monotonic timestamps, or contains a sample gap above the configured
limit. Invalid intervals are never partially consumed by RTK. M1 falls back to
the unchanged legacy SPP/trusted-position reseed for that epoch and re-anchors
at the next accepted RTK posterior.

No stale time update may be reused on a later epoch.

## Delivery order and gates

1. **M0, preintegration:** no RTK behavior change; analytic mechanization,
   transition-composition, noise, reset, and invalid-input tests.
2. **M1, INS time update:** `--tc-ins-time-update`; fix rate within 0.5 pp of
   baseline, no p95 regression, and solver wall time within 5%.
3. **M2, wrong-fix containment:** CP-vs-PR innovation gate and DDPR-LS anchor,
   implemented before closed-loop feedback.
4. **M3, closed loop:** `TightCouplingProcessor` owns interval selection,
   re-anchoring, ZUPT/NHC, and fallback.
5. **M4, velocity states:** append three RTK velocity states and add an explicit
   force-active mask to the RTKLIB-compatible Kalman helper.
6. **M5, TDCP diagnostics:** diagnostics first; measurement updates require a
   separate opt-in gate after validation.

Each milestone requires unit tests, a deterministic short run, and same-binary
OFF/ON full runs on Tokyo run1, Tokyo run3, and Nagoya run1. Major milestones
also run all five PPC datasets. Negative results remain documented and disabled
by default.

## M1 evaluation

The first `1e-4 m^2` process-noise floor was overconfident and reduced Tokyo
run3 fix rate by 4.63 percentage points. One bounded retune selected `25 m^2`
after 2,000- and 6,000-epoch prefix comparisons against `900 m^2`. Full-run
same-binary results for the selected value were:

| Dataset | Mode | Fix % | PPC 3D 50 cm % | p95 horizontal m | p95 abs up m | Wall s |
|---|---:|---:|---:|---:|---:|---:|
| Tokyo run1 | OFF | 76.67 | 72.11 | 6.83 | 31.33 | 538.4 |
| Tokyo run1 | M1 | 74.33 | 69.38 | 7.24 | 29.86 | 725.5 |
| Tokyo run3 | OFF | 74.63 | 78.95 | 8.73 | 13.31 | 1450.9 |
| Tokyo run3 | M1 | 78.20 | 82.22 | 4.23 | 5.89 | 1389.8 |
| Nagoya run1 | OFF | 77.49 | 71.54 | 9.44 | 17.02 | 443.4 |
| Nagoya run1 | M1 | 77.66 | 65.64 | 8.54 | 15.35 | 587.7 |

M1 is therefore a documented mixed-negative result: Tokyo run3 improves
strongly, but Tokyo run1 and Nagoya run1 fail the quality and wall-time gates.
The feature remains default-off. The selected `25 m^2` floor is only the
default after a caller explicitly enables the M1 path.

## M2 evaluation

M2 validates a fixed integer candidate without using either candidate
position. Non-GLONASS pairs apply
`|DD_PR - (DD_CP - fixed_DD_ambiguity)|`; GLONASS FDMA is skipped because the
reference and target wavelengths differ. A first pass that rejected a whole
candidate for one pair above 10 m was too strict on Tokyo run1. The one allowed
retune kept the 10 m reference threshold and allowed one bad pair. Two
consecutive vetoes escalate to an independent DDPR-only LS anchor with
leave-one-out FDE. M2 exposes that anchor for M3 but does not inject it yet.

Same-binary full runs with the selected settings produced:

| Dataset | OFF/ON fixed epochs | OFF/ON 3D 50 cm % | Vetoed candidates | DDPR anchors |
|---|---:|---:|---:|---:|
| Tokyo run1 | 9082 / 9079 | 72.11 / 72.11 | 195 | 191 |
| Tokyo run3 | 11419 / 11419 | 78.95 / 78.95 | 10 | 9 |
| Nagoya run1 | 5848 / 5848 | 71.54 / 71.54 | 5 | 4 |

Official score and horizontal/vertical p95 were also identical within each
OFF/ON pair. The gate remains default-off; enabling it uses a 10 m threshold,
minimum four checked pairs, one allowed bad pair, and two-epoch escalation.
