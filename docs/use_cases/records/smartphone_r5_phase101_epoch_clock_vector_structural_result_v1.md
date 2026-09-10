# Phase101 epoch-local C/ISB structural result

- Status: `go-phase101-epoch-clock-vector-structural`
- Decision: Structural gates passed; solution and accuracy lanes remain unauthorized.
- Matrix: exactly MTV-A then LAX-T, one native invocation each; no rerun, fallback, Cholesky control, truth, accuracy, or solution publication.
- Candidate: Phase101 epoch-local seven-vector C/ISB plus D handoff, Phase99 multifrontal QR main selector; legacy default remains off.

## Route summary

| Route | Return | GNSS-first | C7/D handoff | Main QR/progress | Output | Failed gates |
|---|---:|---|---|---|---|---|
| `2021-03-16-18-59-us-ca-mtv-a/pixel5` | `0` | `5618000.786274777 → 21298.112661390598 / accepted=157` | `C 2159/15113; D 2159/2159; exact=True` | `MULTIFRONTAL_QR / 12 / 76503985.97055085 → 34843.23540741533` | `2159→2159` | `[]` |
| `2022-04-01-18-22-us-ca-lax-t/pixel5` | `0` | `6419784.856579111 → 12818.289257688237 / accepted=270` | `C 1466/10262; D 1466/1466; exact=True` | `MULTIFRONTAL_QR / 12 / 164902215.22814512 → 20386.026617946976` | `1466→1466` | `[]` |

The withheld CSV paths are metadata only; they were never opened, published, scored, or committed. Structural GO, if any, does not authorize accuracy evaluation or submission.
