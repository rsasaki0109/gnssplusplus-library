# Phase 41 Doppler measurement-contract audit

Phase 41 is a truth-free diagnosis of the Android Doppler measurement
contract.  It does not tune a threshold, reverse a sign, or change the
production estimator.  The immutable pre-read contract is the [freeze record](records/smartphone_r5_phase41_doppler_measurement_contract_audit_freeze_v1.json)
(SHA-256
`5622099a9d7d37ee1e4de43385369e70c342acb515f8f83cf301964fc3c64758`).  The
source-frequency amendment and its evaluator manifest v2 were committed and
pushed before the second raw read; the v1 freeze remains unchanged.  The
sealed machine-readable result is the [Phase 41 result](records/smartphone_r5_phase41_doppler_measurement_contract_audit_result_v2.json),
and the six-route execution accounting is in the [structural manifest](records/smartphone_r5_phase41_doppler_measurement_contract_audit_structural_manifest_v2.json).

## Contract under test

For each valid nonzero Android row at the same first solvable epoch, the
audit records the raw `PseudorangeRateMetersPerSecond` value in m/s, the
adapter Doppler in Hz, both source and FGO wavelengths, and the measured rate:

```text
D = -PseudorangeRateMetersPerSecond / wavelength_m
measured_range_rate_mps = -D * wavelength_m
```

It independently reconstructs the broadcast satellite position and velocity,
clock drift, Earth-rotation/Sagnac term, receiver-to-satellite LOS, known
satellite range rate, and receiver residual.  The same fields are compared
with the unchanged corrected-undifferenced FGO factor.  The public SPP
`solveVelocityFromObservations` is also called at the same raw seed and epoch;
its row selection is reported separately from FGO's factor rows.  Raw source
frequency is diagnostic provenance only: the estimators continue to consume
the existing `Observation::doppler` field, as pinned by the focused tests.

The command is raw-only and opens each route's `device_gnss.csv` and
broadcast `brdc.nav` once in one process:

```bash
build/apps/gnss_doppler_contract_audit \
  --android-gnss <device_gnss.csv> --nav <brdc.nav> \
  --rows-csv <audit_rows.csv> --summary-json <audit.json> \
  --dataset-id <route/phone>
```

No `device_imu.csv`, truth, validation, holdout, MATLAB, precomputed
coordinate, device-WLS coordinate, Kaggle, or token input was opened or
materialized.  The failed Phase 40 candidate remains historical evidence and
was not rewritten.

## Sealed result

The target is MTV-h Pixel 5.  Its first solvable epoch is raw epoch 33
(UTC `1629837193437`, GPS week `2172`, TOW `246811.4372994703`).  All 33 raw
adapter rows are emitted; the unchanged FGO factor list contains 4 rows, so
29 row-selection differences are explicitly recorded.  SPP has 33 candidate
rows but returns `ok=false`; the public API does not expose whether rank,
chi-square, covariance, or another post-fit gate caused that result.  FGO is
diagnostic-invalid with 4 rows and reports the historical first-state norms
(`8919.753747298055 m/s` velocity and `7314.385037238741 m/s` clock rate).

The identities pass at floating-point precision on the target:

* raw rate versus `-D * adapter wavelength`: `5.684341886080802e-14 m/s`;
* adapter measured rate internal identity: `1.1368683772161603e-13 m/s`;
* FGO measured rate versus `-D * FGO wavelength`: `5.684341886080802e-14 m/s`;
* FGO known satellite state, range-rate, clock drift, LOS, and both receiver
  residual reconstructions: zero reported difference.

The Android source carrier frequency and the channel-nominal FGO frequency
produce a finite GLONASS conversion difference of at most
`3.364207657341467e-05 m/s` on MTV-h.  It is recorded as provenance, not
diagnosed as a unit/sign/known-term bug: the raw/adapter identity and all FGO
internal identities pass.  Across the six-route structure the corresponding
maximum is `5.3629604963134625e-05 m/s`; it does not change that verdict.
Earth-rotation terms are reported per row (target maxima
`0.004895658897802507 m/s` Sagnac and
`0.004896833975408299 m/s` corrected-minus-unrotated satellite range rate)
and are included in the independent comparison.

The other five frozen routes have first-epoch raw/FGO row counts of 38/27,
39/29, 37/27, 36/26, and 32/22.  Their raw-to-adapter identities also pass;
the SPP and FGO status, selection differences, and output SHA-256 values are
in the structural manifest.  No route produced code-plus-numeric evidence of
a concrete unit, sign, or known-satellite-term defect.

## Decision and next measurement

Phase 41 is sealed **diagnosis No-Go**.  No correction flag was added, no
default path changed, and no physical or residual gate was relaxed.  The next
truth-free measurement is narrowly scoped: classify MTV-h's raw 33-to-FGO-4
attrition by signal, broadcast-navigation availability/health, masks, and
geometry while keeping the Phase 41 model unchanged.  It must not infer a
sign change or threshold adjustment from the implausible velocity alone.

The source/evaluator provenance, row tolerances, one-process/read counts, and
focused tests are frozen in the evaluator [manifest v2](records/smartphone_r5_phase41_doppler_measurement_contract_audit_evaluator_manifest_v2.json).
