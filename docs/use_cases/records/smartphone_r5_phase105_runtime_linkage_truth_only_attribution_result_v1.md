# Phase104 stage-vs-main attribution result

- Status: `complete-truth-only-attribution`
- Native: fresh Phase101 raw-only pipeline; exactly one MTV-A and one LAX-T run.
- Truth: one isolated evaluator read per route; no native rerun, fallback, or publication.

## Aggregate

- Stage macro: `4.558407586787478` m
- Main macro: `2.225306568701026` m
- Stage minus main: `2.333101018086452` m

## Routes

| Route | Stage | Main | Stage−main | Attribution | Truth reads |
|---|---:|---:|---:|---|---:|
| `2021-03-16-18-59-us-ca-mtv-a/pixel5` | `3.334004080470148` | `1.139793072101309` | `2.194211008368839` | `stage-regression-present` | `1` |
| `2022-04-01-18-22-us-ca-lax-t/pixel5` | `5.782811093104809` | `3.310820065300743` | `2.4719910278040658` | `stage-regression-present` | `1` |

The stage ECEF sidecar and main displacement statistics are private diagnostic artifacts; no coordinate rows are included in this result.
