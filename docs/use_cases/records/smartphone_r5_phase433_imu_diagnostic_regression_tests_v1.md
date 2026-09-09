# Phase433 — IMU diagnostic regression checks

Native app rebuild completed. Fresh `/tmp/gnss_phase429_backend_tests.o`
was linked against the rebuilt libraries as `/tmp/gnss_phase429_backend_tests`.
Filter `*Imu*:*BiasPrior*:*ImuBias*` passed 19/19 tests across nine suites.
This includes the paired main test with optimized-bias count/finite-norm
assertions, IMU covariance controls, source endpoint-bias Jacobian comparison,
count-density guards and unsupported prior-removal configurations.

Actual app correction/diagnostic-guard extraction test passed; the new
aggregate-verifier synthetic test also passed. No full CTest or PPC byte-parity
claim. New fields change FGOResult layout; old standalone objects are stale.

Phase431 H and Phase432 LAX-T baseline raw runs remain live in sessions 57436
and 58118 respectively; native PIDs 3607429 and 3607466 were verified running.
No retries, no additional truth reads or accuracy scores. Await completion
before invoking `scripts/verify_native_imu_bias_diagnostic.py 431/432`.
