# Phase477 — lambda-floor candidate numerical result

Phase476 native build/run completed, exit 0; session 9220 closed. Manifest
SHA256 e974ccaf59ef7dfff78f284a942ba53b4ca2f14b7116945080f76a277936c19d.
verify_native_lambda_floor_stages.py passed pins, both effective 1e-8 floors,
preserved stage solvers (GNSS-first Cholesky, main QR), convergence and
diagnostic checks. It reports output mismatch, not a baseline-equivalent pass.
Candidate SHA256 255318bdad6c396deb5a6a634a177022defefc1edd4655c039b4686891288df4.

| Quantity | Phase472 baseline | Phase476 floor ON |
|---|---:|---:|
| GNSS-first accepted iterations / trials | 84 / 164 | 89 / 89 |
| Main accepted iterations / trials | 95 / 186 | 106 / 106 |
| Failed linear trials, first / main | 80 / 91 | 0 / 0 |
| GNSS-first final cost | 178533.28598901647 | 178533.27309960494 |
| Main final cost | 185581.0974156734 | 185581.0593719458 |
| Single launcher elapsed seconds | 230.802035050001 | 135.60952731303405 |

Zero failures in both stages is supported by trials equal accepted iterations,
no failed-lambda logs, and the main failure counter zero. The numerical floor
removes repeated failed trials at 1e-9 but changes the accepted optimization
path. It is not pure behavior-preserving acceleration. Lower objective cost
does not establish better true position. Single-run timing is not a controlled
speed benchmark; background workloads were not held constant.

Default remains OFF; candidate unpromoted. No truth/accuracy evaluation was
performed. Next quantify output differences with evaluation-only aggregate
comparison (never feed either output to inference), verify unchanged graph/
measurement contracts, and define multi-route development criteria before
any scoring. Do not keep adjusting lambda to optimize the reused route.

Phase475 scope/verification mistakes are corrected in Phase476's separate
source pins and stage-aware verifier; failed historical records remain intact.
Original native raw-only accuracy/leaderboard goal remains active and unmet.
