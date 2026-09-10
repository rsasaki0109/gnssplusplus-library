# Phase572: do not identify measurement covariance from pooled post-fit correlation

While Phase571 H/A replays are live, added a separate standalone synthetic
test TU (no changes to their frozen source/header/test pins or binary):
`tests/test_tdcp_temporal_correlation_controls.cpp`.
Compiled `/tmp/phase572_temporal_controls`; three tests passed.

Exact enumeration of eight equally weighted independent +/-1 endpoint noise
triples gives differences b-a, c-b with Pearson correlation -1/2. No random
sampling or fitted coefficients. Adding independent +/-10 stream-level
offsets changes pooled correlation to 99/102 while retaining the same
endpoint noise. Fitting and removing the mean of each two-difference pair
instead produces residual correlation -1. These are algebraic controls,
not actual GNSS noise measurements or a validated smartphone factor model.

Thus Phase571's pooled post-fit statistic cannot directly set temporal
whitening/noise coefficients. An inferred change in it is not evidence of a
specific hardware noise process. Do not promote correlated TDCP from one
aggregate or replace scalar Huber factors by a radial loss silently.

CMake registration deliberately deferred until both frozen Phase571 runs
finish, because tests/CMakeLists.txt is part of their source pins. No full
CTest, accuracy read or score this step.

Last authoritative poll: H session 67031/PID3996373 running at 2m34s;
A session67335/PID3996681 running at 2m11s. No terminal result yet. Resume
these handles; do not launch duplicates or modify frozen files during runs.
