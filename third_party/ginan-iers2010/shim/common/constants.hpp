// Minimal stand-in for ginan's `common/constants.hpp`, supplied so that the
// vendored iers2010 wrappers in this directory compile without pulling in
// ginan's full common/ tree.
//
// The vendored iers2010 sources only reference D2R (degrees-to-radians).
// Definitions here match ginan's values exactly so behavior is bit-identical.

#pragma once

#include <cmath>

constexpr double D2R = (M_PI / 180.0);
