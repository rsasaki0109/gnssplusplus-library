# Phase428 — Android IMU reported bias fields

Read raw Phase37 device_imu.csv MessageType and BiasX/Y/Z only, for H and
LAX-T. Every accelerometer and gyro bias component is finite and exactly zero.
H counts: 243457 UncalAccel, 166502 UncalGyro. LAX-T counts: 90008 UncalAccel,
77758 UncalGyro. Each sensor/route has exactly one distinct bias vector,
zero median/max norm and zero maximum adjacent-row bias change.

Native `src/io/imu.cpp` parses these fields but passes uncalibrated measurement
vectors to the estimator without subtracting the reported bias. Source
`deviceimu2imu.m` stores xyz and bias separately; `imuprocessing.m` interpolates
xyz. Native GTSAM preintegration consumes the raw vectors and estimates its
own bias state. No evidence here of double-applying Android-reported bias.
Subtracting these all-zero fields cannot change either route's result.

Zero reported fields do NOT prove physical sensor bias is zero, correct bias
observability, or adequate random-walk/initialization settings. The prior
stationary-gyro experiment remains a separate negative result. No code or
noise changes, truth reads, inference runs, or accuracy evaluations in this
audit. Do not launch a bias-field-subtraction candidate for these routes.
