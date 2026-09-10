# Phase134 native-summary bridge structural raw result

- Status: `go-phase134-native-summary-bridge-structural`
- Matrix: MTV-A then LAX-T, exactly one authorized inventory pass and at most one native invocation per route.
- Native selectors 126/127/128/129/131/118 are each forwarded once; Phase130 is absent from native argv.
- Native `native_summary.json` is immutable; `structural_summary.json` is the separate normalized view.
- Solution content is not interpreted; only opaque hash and expected row count are sealed.
- Truth, MAT, PDC, precomputed coordinates, accuracy, and Kaggle remain forbidden.

| Route | Inventory | Return | Bridge | Resolver/attempt | Canonical rows/rejects | Main iterations | Main cost | Native bytes/hash | GO | Failure |
|---|---|---:|---|---:|---:|---:|---|---|---|---|
| `2021-03-16-18-59-us-ca-mtv-a/pixel5` | `True` | `0` | `True` | `121673/121673` | `79652/42021` | `12` | `122809034.24737301->28266.35349384695` | `35252/f38550a3f19ab3c7f49fffdd5c555825e45e7762ffe8a91e287ddefb66c36f36` | `True` | `` |
| `2022-04-01-18-22-us-ca-lax-t/pixel5` | `True` | `0` | `True` | `6342/6342` | `4418/1924` | `12` | `143982373.9341658->17995.990758732205` | `34563/821de4f1e5dc9ba5368afdf02f8344aed0bf313e822134d2c2d2254a81cbe988` | `True` | `` |

Read accounting excludes all truth/accuracy/MAT/PDC/precomputed-coordinate/Kaggle access.
Structural failure is sealed fail-closed; no rerun, fallback, repair, or solution publication is permitted.
