# Phase131 canonical physical-band structural raw result

- Status: `no-go-phase131-canonical-correction-band-structural`
- Matrix: MTV-A then LAX-T, exactly one inventory pass and at most one native solver attempt per route.
- The canonical key is `(GNSSSystem, PRN, physical-frequency-family[, certified GLONASS FCN])`; literal tracking text is provenance only.
- Solution rows were not opened or interpreted; only opaque hashes and expected row counts were sealed.
- Truth, MAT, PDC, precomputed coordinates, accuracy, and Kaggle remain unauthorized.
- Both routes failed at the pre-solver exact canonical-support gate; each raw GNSS/IMU/nav/base payload was read once, no native solver was launched, and no retry/fallback was attempted.
- Detailed counters are sealed Phase130 comparison metadata (not a Phase131 rerun): MTV-A rover `12068/0/11832` input/certified/missing-exact-key and `0` retained factors; LAX-T `8873/0/8626` and `0`. Base certified/miss was `9642/7557` and `396/524`; canonical and endpoint/bracket support were `0` for both.

| Route | Inventory | Solver | Return | Rover cert/miss | Base cert/miss | Canonical keys | Main accepted | Main cost | GO | Failure |
|---|---|---|---:|---:|---:|---:|---:|---|---|---|
| `2021-03-16-18-59-us-ca-mtv-a/pixel5` | `False` | `False` | `None` | `0/0` | `0/0` | `0` | `0` | `None->None` | `False` | `pre-solver inventory failed closed` |
| `2022-04-01-18-22-us-ca-lax-t/pixel5` | `False` | `False` | `None` | `0/0` | `0/0` | `0` | `0` | `None->None` | `False` | `pre-solver inventory failed closed` |

Inventory failure is fail-closed and prevents a native launch for that route.
