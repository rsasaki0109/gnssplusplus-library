# Phase130 keyed shared-ledger structural raw result

- Status: `no-go-phase130-shared-ledger-key-local-support-structural`
- Matrix: MTV-A then LAX-T, exactly one authorized inventory and at most one native solver attempt per route.
- Phase130 selector is enforced at the runner/contract boundary and is not forwarded to the pinned Phase129 binary.
- Solution rows were not opened or interpreted; only opaque hashes and expected row counts were sealed.
- Truth, MAT, PDC, precomputed coordinates, accuracy, and Kaggle remain unauthorized.

| Route | Inventory | Solver | Return | Rover cert/miss | Base cert/miss | Retained exact keys | Main accepted | Main cost | GO | Failure |
|---|---|---|---:|---:|---:|---:|---:|---|---|---|
| `2021-03-16-18-59-us-ca-mtv-a/pixel5` | `False` | `False` | `None` | `0/12068` | `9642/7557` | `0` | `0` | `None->None` | `False` | `pre-solver inventory failed closed` |
| `2022-04-01-18-22-us-ca-lax-t/pixel5` | `False` | `False` | `None` | `0/8873` | `396/524` | `0` | `0` | `None->None` | `False` | `pre-solver inventory failed closed` |

Inventory failures are fail-closed and prevent a native launch for that route.
