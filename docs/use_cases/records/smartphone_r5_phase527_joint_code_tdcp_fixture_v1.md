# Phase527 — shared code/TDCP endpoint-state recovery

Added JointCodeTdcpIonosphere.SharedEndpointStatesRecoverSyntheticDelays to
the existing registered test_tdcp_residual_state_factor.cpp. Uses actual native
C7 point-code and point-TDCP factors wrapped by Phase525/526 factors. Two bands
share the same two vertical residual keys; unequal endpoint mapping factors
and frequency scaling are explicit. Four code plus two TDCP observations are
inserted once each, with no original/wrapped double counting.

Synthetic measurements include +a*I for code and +a_prev*I_prev-a_curr*I_curr
for TDCP. At true residual states (.4,.7 m), graph error is near zero. From zero
states native GTSAM LM recovers both within 1e-5 m. Geometry and C7 clocks are
tightly pinned solely to isolate the wrapper coupling and signs. There is no
ionosphere prior in this fixture; code anchors both states. This does NOT prove
observability with freely estimated position/clocks or realistic phone noise.

Fresh standalone build /tmp/phase527_joint_ionosphere exited 0, then runtime
with preserved LD_LIBRARY_PATH prefixed by /home/sasaki/.local/lib passed 7/7
tests (one new, six existing). Full CTest not run. No production graph or raw
run changed; no MAT, saved positioning or truth input.

Next implementation remains temporal state segmentation/anchor/random-walk
contract and raw TDCP endpoint coefficient provenance. Do not infer a physical
noise prior from this synthetic fixture or diagnostic alignment. Actual Pose3
joint coverage remains outstanding. Goal active; accuracy target unproven.
