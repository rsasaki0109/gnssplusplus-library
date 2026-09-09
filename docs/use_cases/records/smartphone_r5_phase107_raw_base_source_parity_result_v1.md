# Phase107 raw-base source-parity structural result

- Status: `go-phase107-raw-base-structural`
- Decision: Structural gates passed; solution and truth/accuracy lanes remain unauthorized.
- Matrix: exactly MTV-A then LAX-T, one native invocation each; no rerun, fallback, truth, accuracy, or solution publication.
- Base accounting: one wrapper SHA verification read plus one native RINEX process read per route; station coordinate source is the raw RINEX header APPROX POSITION XYZ.

## Route summary

| Route | Return | Base correction | GNSS-first | C7/D | Main QR/progress | Output | Failed gates |
|---|---:|---|---|---|---|---|---|
| `2021-03-16-18-59-us-ca-mtv-a/pixel5` | `0` | `enabled=True; built=True; applied=True; retained=43259; miss=14022; native_reads=1` | `4516236.0222890135 → 16097.714277915162 / accepted=218` | `C 2159/15113; D 2159/2159; exact=True` | `MULTIFRONTAL_QR / 12 / 130723525.58263575 → 29729.5954122218` | `2159→2159` | `[]` |
| `2022-04-01-18-22-us-ca-lax-t/pixel5` | `0` | `enabled=True; built=True; applied=True; retained=30664; miss=134; native_reads=1` | `6412859.293244721 → 12298.593335217001 / accepted=234` | `C 1466/10262; D 1466/1466; exact=True` | `MULTIFRONTAL_QR / 12 / 162184507.52202186 → 19851.9725625441` | `1466→1466` | `[]` |

The withheld CSV paths are metadata only; they were never opened, published, scored, or committed. Structural GO does not authorize truth/accuracy evaluation or submission.
