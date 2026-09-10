# Phase510 — pinned reference clock path and differential boundary

Source inspection only; no MAT opened or executed, no saved positioning input,
no ground-truth access, no accuracy evaluation, and no solver change.

MatRTKLIB cache HEAD is 69bcbd3faebc39815adb03e4b329033a4a7deb4a.
Its gitlink src/RTKLIB pins MALIB 159e150d4a54e6b7b15d81128289b8559523ca81.
The submodule source is absent locally. Inspected the exact pinned upstream
[ephemeris.c](https://raw.githubusercontent.com/JAXA-SNU/MALIB/159e150d4a54e6b7b15d81128289b8559523ca81/src/ephemeris.c),
not the different external/madocalib checkout (0089f7dc97e8e2ba283a40be2edf4b73a140df6c).
Broadcast clock uses polynomial plus relativity, with no TGD/BGD application;
satposs forwards the satellite clock (broadcast fallback likewise has no TGD).
Cached src/mex/satposs.c forwards dts multiplied by CLIGHT, without code bias.
This closes the missing-source clock check from Phase509, but does not prove
which binary/version produced any historical competition result.

Local source boundaries:

- Gsat.m setObs subtracts optional receiver clock from observations for satellite
  timing, then calls the wrapper; it does not add group delay to the output clock.
- Gobs.m residuals computes corrected code residual using range, satellite clock,
  ionosphere and troposphere, without an explicit group-delay term.
- gsdc2023/functions/gnsslog2obs.m builds code from receive-minus-transmit time;
  its assignment does not use the inter-signal bias columns listed in the header.
- fgo_gnss_imu.m unconditionally calls correct_pseudorange for each present band.
  That helper forms base residuals with the same Gsat/Gobs path, smooths them,
  and interpolates them to rover times before subtraction.

Inference: a common satellite/code bias can cancel in rover-minus-base
residuals even though neither residual explicitly corrects it. Different signal
codes, timing, smoothing and receiver biases limit this cancellation. Therefore
the absence of an explicit term in this differential reference is NOT evidence
that disabling native no-base TGD is a physically correct fix. Do not promote
an all-signals TGD-off option as source parity.

Also inspected preprocessing.m: it reads WlsPosition* columns for initialization
and serializes MATLAB data. Those operations remain forbidden here; source
inspection is not authorization to port that data dependency.

Next action: quantify native selected-message group-delay terms by signal on
raw input, then audit signal-specific clock/code conventions. Only after that
freeze a single justified native correction experiment; avoid another H-only
threshold sweep. The .782/LB objective remains unmet.
