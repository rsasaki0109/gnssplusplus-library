# RTKLIB v2 Architecture Overview

_Last updated: 2025-11-12_

## 1. Guiding Principles

1. **Layered & Modular** – each concern (ingestion, estimation, services, telemetry) lives in a
dedicated module with explicit interfaces.
2. **Real-Time Readiness** – deterministic pipelines, bounded queues, and asynchronous IO are
first-class citizens.
3. **Extensible** – new constellations, correction sources, or algorithms can be introduced without
touching core contracts by leveraging plugin registries and dependency injection.
4. **Observable** – every module surfaces metrics, logs, and trace IDs to simplify monitoring and
root-cause analysis in production deployments.
5. **Polyglot Friendly** – C++20 powers the engine, while services and bindings expose the same
capabilities to Python, Rust, or web clients.

## 2. High-Level Component Map

```
┌──────────────────────────────────────────────────────────────┐
│                         Applications                          │
│ CLI ▪ gRPC Client ▪ Python SDK ▪ Web UI ▪ Embedded Adapters   │
└───────────────┬─────────────────────────┬────────────────────┘
                │                         │
┌───────────┐   │   ┌────────────────┐    │
│  Service  │◀──┼──▶│  Binding Layer │◀───┘
│   Layer   │   │   │ (pybind11 etc.)│
└─────┬─────┘   │   └──────┬─────────┘
      │         │          │
┌─────▼─────────▼─────────────────────────────────────────────┐
│                         Core Engine                         │
│ Observation Pipeline ▪ Solver Plugins ▪ State Estimation ▪ │
│ Correction Services ▪ Timing/Synchronisation ▪ Config & DI │
│ Telemetry Bus ▪ Plugin Manager ▪ Persistence Facades       │
└─────┬───────────────┬───────────────┬───────────────┬──────┘
      │               │               │               │
┌─────▼──────┐ ┌──────▼──────┐ ┌──────▼──────┐ ┌──────▼──────┐
│ Data Abstr │ │  Hardware   │ │ Persistence │ │ Deployment │
│ (Streams)  │ │ Abstraction │ │  Layer      │ │ Targets    │
└────────────┘ └─────────────┘ └────────────┘ └────────────┘
```

## 3. Module Breakdown

| Module | Namespace | Responsibilities |
| --- | --- | --- |
| **Common** | `rtklib2::common` | Value types, utilities, versioning, compile options, concept checks |
| **Core** | `rtklib2::core` | System lifecycle, dependency injection, configuration, resource registry |
| **Pipeline** | `rtklib2::pipeline` | Stage orchestration, back-pressure, scheduling of observation flows |
| **IO** | `rtklib2::io` | Receivers, file/stream adapters (RINEX, RTCM, UBX), buffering, decoding |
| **Solvers** | `rtklib2::solvers` (planned) | RTK, PPP, Factor Graph implementations via strategy interface |
| **Services** | `rtklib2::services` | gRPC/REST endpoints, session management, auth, rate limiting |
| **Telemetry** | `rtklib2::telemetry` | Logging, metrics, tracing, event bus integration |
| **Persistence** | `rtklib2::storage` (planned) | Ephemeris cache, solution archival, offline replay |
| **Bindings** | `rtklib2::bindings` (planned) | pybind11 layer, FFI wrappers, SDK packaging |

## 4. Data Flow Summary

1. **Ingestion** – `io::DataStream` implementations open observation/correction sources (serial,
TCP, NTRIP, files). Each source emits standardized frames into the Pipeline.
2. **Pipeline Stages** – Stages perform decoding, quality checks, transformation, and queue
normalised observation batches.
3. **Solver Invocation** – A `SolverStage` pulls batches, consults the `core::SystemContext` for
current configuration (antenna models, atmospheric parameters), runs the chosen solver, and
produces solutions.
4. **State Publication** – Solutions propagate to telemetry (for monitoring) and persistence (for
historical storage). Service layer exposes the latest state via gRPC/REST and pushes updates to
subscribers.

## 5. Threading & Concurrency Model

- **Stage Execution** – Pipelines default to cooperative execution on a thread pool (configurable).
Future work: deterministically scheduled pipelines for hard real-time environments.
- **Resource Ownership** – `SystemContext` stores components via `std::shared_ptr` keyed by
`std::type_index`, simplifying dependency injection.
- **Cancellation** – Planned support for cooperative cancellation tokens (C++23 when available),
allowing graceful shutdown of pipelines and services.

## 6. Configuration Strategy

| Concern | Mechanism |
| --- | --- |
| Static defaults | `config/defaults/*.yaml` (planned) loaded during `Library::initialize()` |
| Profiles | `Library::initialize(profile)` selects environment-specific overrides |
| Hot reload | Watchers (optional) publish updates to Pipeline/Solver components |
| Secrets | Delegated to deployment (Kubernetes secrets, environment variables) |

Configuration values are normalised into `core::SystemContext` and surfaced to consumers via
strongly typed structs (e.g., `solver::RtkConfig`, `io::RtcmSourceConfig`).

## 7. Telemetry & Observability

- **Logging** – `telemetry::Logger` facade supports sinks (console, spdlog, OTLP). Default: console.
- **Metrics** – Planned Prometheus exporter with `telemetry::MetricRegistry` for counters/gauges.
- **Tracing** – OpenTelemetry spans for pipeline stages and solver executions, correlated with
request IDs from service layer.

## 8. Roadmap Checkpoints

| Milestone | Description |
| --- | --- |
| M1 | Baseline pipeline with mock stages + API scaffolding |
| M2 | First solver plugin (SPP) integrated end-to-end |
| M3 | Real-time correction ingest (NTRIP client) |
| M4 | Python bindings + sample notebook |
| M5 | Production-ready observability stack + deployment artefacts |

## 9. Open Questions

1. **Solver Isolation** – Should solvers run in dedicated worker pools to prevent contention with
ingestion stages?
2. **Persistence Backend** – Evaluate SQLite vs. Arrow vs. custom binary logs for long-term storage.
3. **Hardware Abstraction** – Investigate ROS2 integration for robotic platforms.
4. **Determinism** – Determine requirements for fixed-latency pipelines vs. best-effort throughput.

## 10. Next Steps

- Define concrete interfaces for solver plugins (`include/rtklib2/solvers/` namespace).
- Prototype stage scheduling policies (round-robin, priority queue).
- Draft ADR for dependency injection approach and configuration format.
- Set up CI to enforce formatting, linting, and tests.
