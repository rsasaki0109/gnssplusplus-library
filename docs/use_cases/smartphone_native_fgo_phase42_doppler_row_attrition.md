# Phase 42 Doppler row attrition audit

Phase 42 traces truth-free Android Raw GNSS rows through the unchanged
Phase41 corrected-undifferenced FGO Doppler-factor path.  The [freeze
record](records/smartphone_r5_phase42_doppler_row_attrition_freeze_v1.json) is
immutable.  The evaluator source/binary and its raw-only read contract were
sealed in [manifest v2](records/smartphone_r5_phase42_doppler_row_attrition_audit_evaluator_manifest_v2.json)
before the six route rerun.  The complete machine-readable result, including
per-row CSV/summary hashes, is the [sealed result](records/smartphone_r5_phase42_doppler_row_attrition_result_v1.json).

## Measurement boundary

Each route opens exactly one `device_gnss.csv` and one broadcast `brdc.nav`.
The audit retains every parsed `MessageType=Raw` row, including rows rejected
by the raw loader, and records a stable raw row/epoch/satellite/signal key.
Every row receives one exclusive first rejection reason in the following
order: raw validity and Doppler mask, raw SNR/multipath mask, supported
constellation/signal/frequency, raw pseudorange, transmit time, navigation
state calls 1 and 2, ephemeris presence/health, finite geometry, SNR/elevation
masks, clock jump/dt, receiver-clock-drift inclusion, and final factor
presence.  A finite zero Doppler is present; the unchanged corrected-factor
gate requires only `dt_s > 0`.

No IMU, truth, validation, holdout, MAT, precomputed/device-WLS/result
coordinates, Kaggle, or token input is read.  UTC/GPS identity is checked for
every emitted row by recomputing UTC milliseconds from GPS week/TOW with the
frozen 18-second offset; all 412,179 route rows match within 1 ms.

## Six-route result

| route | first raw → selected → FGO | selected loss | first supported → elevation pass | anchor / seed | first elevation (min / median / max; below 0) |
|---|---:|---:|---:|---|---|
| MTV-h / Pixel 5 | 35 → 33 → 4 | 29 | 24 → 4 | none / sentinel fallback | −86.9903° / −40.1623° / 21.6719°; 20 |
| MTV-a / Pixel 5 | 39 → 38 → 27 | 11 | 27 → 27 | fresh SPP | 6.5949° / 32.3438° / 70.4833°; 0 |
| MTV-b / Pixel 4 | 40 → 39 → 29 | 10 | 29 → 29 | fresh SPP | 7.3927° / 32.4742° / 69.6935°; 0 |
| MTV-e / SM-G988B | 44 → 42 → 27 | 15 | 33 → 27 | fresh SPP | 5.5915° / 34.3098° / 73.8491°; 0 |
| LAX-t / Pixel 5 | 36 → 36 → 26 | 10 | 26 → 26 | fresh SPP | 2.1931° / 30.5065° / 67.2849°; 0 |
| MTV-u / Pixel 5 | 32 → 32 → 22 | 10 | 22 → 22 | fresh SPP | 4.0194° / 33.1400° / 81.0975°; 0 |

On MTV-h, the 29 selected-row losses are exactly 9 unsupported
constellation/signal/frequency rows plus 20 elevation-mask rows.  All 24
supported rows pass raw pseudorange, transmit time, both navigation state
calls, ephemeris presence/health, geometry, and SNR; the four surviving rows
also pass positive `dt`, include receiver clock drift, and appear in the FGO
factor list.  The four factor identities are G15/GPS_L1CA, G24/GPS_L1CA,
C19/BDS_B1I, and E05/GAL_E1.  No broadcast-navigation missing or unhealthy
condition is present.

The selected five non-target routes show the structural control: their fresh
raw/nav SPP seeds have Earth norms 6,370,252–6,371,503 m, geodetic positions
near their route locations, positive minimum elevation, and zero below-zero
rows.  MTV-h instead has quality-anchor `selected=false`, no fresh or held
SPP seed, and exactly the route-independent `(6378137,0,0)` fallback (Earth
norm 6,378,137 m, lat/lon/height 0).

## Decision

The evidence proves a generalizable `fallback-seed-elevation-mask-bug`:
when no fresh or last-valid SPP seed exists, the path applies a
location-dependent elevation mask to the sentinel fallback and rejects
otherwise supported/nav-healthy rows.  Phase 42 does not change production,
thresholds, signs, or the default behavior.  The proposed next experiment is
an explicit opt-in that bypasses only elevation gating when the seed is the
sentinel fallback with neither fresh nor held SPP; raw validity, nav/health,
SNR, finite physical geometry, clock/dt, and receiver-drift gates remain
required.

