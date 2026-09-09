# Phase456 — matched P-D replay result

Phase455 completed once with exit 0 in 227.168803254026 seconds.
Manifest SHA256: `9e919689fe25d07571cecfd9c2ffab22afadb227f404f354bd852d283cb64516`.
`python3 scripts/verify_native_imu_bias_diagnostic.py 455` passed all pins,
convergence, baseline output identity and aggregate conservation checks.
Candidate SHA256 remains
`d2c619121951d1a322ab39e15fcf16dbf7a6d61d2c651d03a4f68ea357c61e7d`.
Verifier regression suite: 6 passed. No truth reads or accuracy evaluations.

The following aggregates match each pre-mask P factor to its actual input
epoch and exact satellite/signal in immediate adjacent input epochs.

| Epoch | Population | Direction | Missing | Positive | Negative | Max absolute P-D (m) |
|---|---|---|---:|---:|---:|---:|
| 851 | retained | incoming | 0 | 1 | 0 | 4.499 |
| 851 | retained | outgoing | 0 | 1 | 0 | 23.069 |
| 851 | rejected | incoming | 2 | 2 | 3 | 32.083 |
| 851 | rejected | outgoing | 2 | 3 | 2 | 16.238 |
| 852 | retained | incoming | 2 | 3 | 1 | 23.069 |
| 852 | retained | outgoing | 1 | 0 | 5 | 24.324 |
| 852 | rejected | incoming | 3 | 2 | 1 | 34.139 |
| 852 | rejected | outgoing | 1 | 3 | 2 | 20.212 |

Zero-valued pairs: none in these groups. Each direction accounts for all
members of its population, including missing pairs. Final P counts are still
1 and 6; rejected centered residual signs are still 7 negative at 851 and
5 negative / 1 positive at 852. Nominal P rank and optimized bias aggregates
are unchanged. The native process is terminal; session 71089 is closed.

## Decision

Identity-matched pairs do not show a uniform signed temporal jump across
the rejected population. This rules out using the previous unmatched raw
extremes as an explanation for every rejection. It does not establish healthy
code observations: P-D differences measure changes, not persistent offsets,
and missing pairs provide no temporal constraint. Nor does it establish a
seed error. No quality threshold change or re-admission is justified yet.

Further full baseline replays adding only similar signed counters have low
value. Next investigate the same-run raw-P initializer's sensitivity, without
truth: its app configuration admits SNR/elevation down to zero for seed
construction, and its SPP robust-weight option is default-off. A bounded
synthetic contaminated-code test should determine whether seed clock/position
errors can drive clean observations beyond the existing centered-residual
mask, and whether the existing robust initializer addresses that mechanism.
Do not interpret this as authorization to tune thresholds on H or to promote
an untested initializer. Phase286 already showed that simple re-masking did
not improve the frozen development metric.

The 0.782-class and leaderboard objectives remain unverified and unmet.
No submission, saved-position inference input or MAT data was used.
