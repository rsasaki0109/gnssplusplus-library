# Phase118 official TDCP Huber-k structural result

- status: `go-phase118-official-tdcp-huber-k-structural`
- matrix: exactly MTV-A then LAX-T, one native invocation per route
- recipe: Phase112 raw/base + C7/D handoff + Phase99 MULTIFRONTAL_QR + raw-base correction + Pixel5 final offset + Phase118 official Highway k
- fixed TDCP sigma: `0.03 m`; Phase117 dynamic sigma: disabled
- truth/MAT/phone-coordinate/precomputed/PDC/Kaggle/accuracy lanes: not read
- solution: opaque hash/header/row seal only; coordinates omitted and unpublished

| Route | Return | Passed gates |
|---|---:|---:|
| `2021-03-16-18-59-us-ca-mtv-a/pixel5` | `0` | `11/11` |
| `2022-04-01-18-22-us-ca-lax-t/pixel5` | `0` | `11/11` |

A failed gate is sealed fail-closed.  No retry, fallback, truth score, accuracy evaluation, or solution release is permitted.
