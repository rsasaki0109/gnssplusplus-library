# Phase337: synthetic atmosphere oracle comparison

`scripts/native_atmosphere_oracle_audit.cpp` compares the native Klobuchar
and Saastamoinen helpers against separately compiled MADOCA `rtkcmn.c`
(revision and source hash recorded in Phase336). It uses only a synthetic
grid, the documented default nonzero ionosphere coefficients and humidity
0.7. No raw dataset, truth, receiver solution or MAT input is read.

Grid: latitude {-60,0,37,60} degrees; longitude -122 degrees; height
{-50,0,100,1000} m; elevation {-1,0,1,3,5,15,45,90} degrees;
azimuth {0,90,180,270} degrees; GPS week 2172 and TOW
{0,246598,604799}. Split reporting at 5 degrees was fixed in the code before
running. Pass tolerance on the >=5-degree group is 1e-6 m for both models;
the lower group reports differences without a pass criterion.

Successful build and run: 768 samples in each group. For elevation >=5
degrees, maximum ionosphere difference was 1.2434497875801753e-14 m and
maximum troposphere difference 3.3750779948604759e-12 m. Below 5 degrees,
the respective maxima were 16.532142312531384 m and 139.44255721018294 m.

Source inspection explains boundary differences: native troposphere returns
zero at elevation <=0.05 radians and floors the zenith cosine at 0.067;
RTKLIB returns zero only at nonpositive elevation and does not use that
floor. Native Klobuchar lacks RTKLIB's nonpositive-elevation guard. This
does not show a discrepancy above 5 degrees for this coefficient set or
prove the differences occur in H observations. Missing/all-zero coefficient
fallback and heights outside this grid remain outside this numerical check.

The source-complete base model calls these native helpers before smoothing
and has no elevation rejection at that call site. Therefore the next useful
check is actual low-elevation support in the raw base series and whether its
151-sample smoothing window can affect retained rover corrections. Do not
claim causality from these synthetic maxima or change the global helpers
and thereby silently alter legacy solvers. Production recipe/score unchanged.

Build uses the same native dependency link as Phase336 and the existing
independent `rtkcmn.o`, with C++17/O1 and section garbage collection. Output
executable: `/dev/shm/gnss-test-build-recovery.BJvQt9/atmosphere_oracle_audit`.
No user data was removed; temporary compiled artifacts are volatile.
