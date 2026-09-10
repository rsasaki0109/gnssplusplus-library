# Phase256 coupled TDCP convention audit

Source-code inspection only; no MATLAB execution or MAT payload access.

The cached MatRTKLIB `+gt/Gobs.m` line 1155 defines resL as carrier metres
minus (range minus satellite clock). Line 1164 separately defines resLc
with ionosphere and troposphere terms. The cached gsdc2023
`fgo_gnss_imu.m` lines 307–316 use the temporal difference of resL and
previous-epoch LOS, not resLc. Cached gtsam_gnss `TDCPFactor_XXCC.h`
computes LOS dot endpoint displacement corrections plus C2[0]-C1[0],
minus that temporal residual. The native affine factor has this algebraic
structure; full geometry/preprocessing parity is not established.

For synthetic phase L = range + C - satellite_clock + T - I + ambiguity,
constant ambiguity cancels across time. At exact anchor positions and
receiver clocks, the source resL factor retains residual -delta(T-I).
The atmosphere-corrected native observable removes that term if its
atmospheric model is exact. Changing endpoint geometry to previous-LOS
affine geometry does not itself remove the atmospheric residual.

New C++ test `FGOGtsamPhase256TdcpConventionTest` exercises all four
combinations of two observable conventions and two actual geometry factors
at anchor positions and at a displaced second endpoint. It uses nonzero
satellite motion, receiver-clock change, satellite-clock change, ambiguity,
and atmospheric change. Build session 93025 completed successfully. Ten
selected C++ tests passed across Phase256/253/252/251 and Phase171. The
filter also named Phase255 and FGOTdcpReslNormalization, but neither matched
any tests; no coverage is claimed for those patterns or full CTest.
The new four-combination test passed. This fixed-state identity does not imply additivity of
optimized trajectories under robust loss.

Therefore no physical cancellation argument currently justifies promoting
resL plus affine geometry or expecting it to reverse both isolated H
regressions. Keep the reference recipe. Before another accuracy experiment,
inspect remaining source/native preprocessing or
initialization differences rather than sweeping these two switches.
