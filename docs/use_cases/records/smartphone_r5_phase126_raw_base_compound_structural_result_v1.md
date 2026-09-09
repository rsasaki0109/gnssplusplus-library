# Phase126 raw-base source-complete structural result

- Status: `no-go-phase126-raw-base-compound-structural`
- Matrix: MTV-A then LAX-T, exactly one authorized attempt per route; no rerun/fallback.
- Truth, MAT, PDC, precomputed coordinates, accuracy, Kaggle, and solution-row interpretation remain forbidden.

| Route | Inventory | Solver | Return | A/B/C | GNSS-first | Main QR/coverage | Failure |
|---|---|---|---:|---|---|---|---|
| `2021-03-16-18-59-us-ca-mtv-a/pixel5` | `False` | `False` | `None` | `False` | `False/False` | `False/False` | `pre-solver inventory failed closed` |
| `2022-04-01-18-22-us-ca-lax-t/pixel5` | `False` | `False` | `None` | `False` | `False/False` | `False/False` | `pre-solver inventory failed closed` |

Inventory failure is fail-closed and prevents native launch for that route.  The solution path is opaque metadata only; its contents were never opened or published.

Both raw GNSS inventories had complete observed signal/frequency mappings and
finite rows.  The broadcast-navigation inventories were finite and in the
supported file domain.  Both base members matched their sealed byte count and
SHA-256, and both had finite Earth-valid `APPROX POSITION XYZ`, but each had
GLONASS observation rows with zero entries in `GLONASS SLOT / FRQ #`:
MTV-A had 17,199 GLONASS body rows and LAX-T had 920.  Because GLONASS is
therefore used without a source-complete channel/frequency mapping, both
routes failed the mandatory pre-solver inventory gate.  No native solver was
launched, so A/B/C, GNSS-first/main progress, cost, and output coverage are
intentionally `false`/unavailable rather than inferred.
