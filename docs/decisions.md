# Design Decisions

Key architectural and implementation decisions in libgnss++.

## Explicit State Management

All solvers maintain explicit state structures (`RTKProcessor`, `PPPProcessor`) rather than
global runtime singletons. This makes testing deterministic and state transitions auditable.

## Modular RTK Pipeline

The RTK solver is decomposed into discrete stages: **selection** (reference satellite and DD
pair construction), **measurement** (residual and covariance assembly), **update** (Kalman
filter), and **validation** (fix validation with conservative hold policy).

## Centralized Signal Policy

`signal_policy.hpp` provides a single source of truth for band mapping and constellation
priority, preventing code duplication across solvers and I/O modules.

## Protocol-Agnostic Solver Layer

Solvers accept canonical `ObservationEpoch` and `NavigationProducts` structures, remaining
independent of the input format (RINEX, RTCM, UBX, QZSS L6).

## Sign-off Framework

Every validation gate produces a machine-readable JSON summary so CI can enforce regression
thresholds without human inspection.
