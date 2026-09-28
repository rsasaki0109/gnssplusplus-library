#!/usr/bin/env python3
"""Reproduce README "Results And Validation Status" numbers from tracked lane manifests.

Each lane lives in ``configs/reproduce/<lane>.toml`` and declares the dataset
layout it needs, the exact argv of every step, external tool pins, and the
expected metrics (README values) with tolerances.  This command renders and
runs those steps, then compares produced metrics against the expectations.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import time
from typing import Any, Iterable, Mapping, Sequence

try:
    import tomllib
except ModuleNotFoundError:  # pragma: no cover - Python < 3.11
    import tomli as tomllib  # type: ignore[no-redef]

from support.gnss_runtime import application_root


ROOT_DIR = application_root(__file__)
MANIFEST_DIR = ROOT_DIR / "configs" / "reproduce"
EXE_SUFFIX = ".exe" if os.name == "nt" else ""
BUILD_CONFIGS = ("Release", "RelWithDebInfo", "Debug", "MinSizeRel")
DATASET_ENV = {
    "ppc": "GNSSPP_PPC_DATASET_ROOT",
    "urbannav": "GNSSPP_URBANNAV_ROOT",
    "gsdc": "GNSSPP_GSDC_ROOT",
    "gsdc_truth": "GNSSPP_GSDC_TRUTH_ROOT",
    "ppc_goal_inputs": "GNSSPP_PPC_GOAL_INPUTS",
}
# CLI option that sets each dataset root (default: --<name>-root).
DATASET_OPTION = {"ppc_goal_inputs": "--ppc-goal-inputs"}
# A dataset root that is set neither by option nor by environment variable
# falls back to another dataset's root (GSDC ground truth usually ships inside
# the GSDC tree itself).
DATASET_FALLBACK = {"gsdc_truth": "gsdc"}
LANE_STATUSES = ("ready", "planned")
PLACEHOLDER_RE = re.compile(r"\{([A-Za-z_][A-Za-z0-9_]*)(?::([^{}]+))?\}")
PATH_TOKEN_RE = re.compile(r"([^.\[\]]+)|\[([^\]]+)\]")


class ReproduceError(Exception):
    """User-facing error (bad manifest, missing input, unknown lane)."""


# --------------------------------------------------------------------------
# Manifest loading
# --------------------------------------------------------------------------


def _require(mapping: Mapping[str, Any], key: str, where: str, kind: type | tuple[type, ...]) -> Any:
    if key not in mapping:
        raise ReproduceError(f"{where}: missing required key `{key}`")
    value = mapping[key]
    if not isinstance(value, kind):
        raise ReproduceError(f"{where}.{key}: expected {getattr(kind, '__name__', kind)}")
    return value


def _validate_steps(steps: Any, where: str) -> list[dict[str, Any]]:
    if steps is None:
        return []
    if not isinstance(steps, list):
        raise ReproduceError(f"{where}: expected an array of tables")
    validated: list[dict[str, Any]] = []
    for index, step in enumerate(steps):
        label = f"{where}[{index}]"
        if not isinstance(step, dict):
            raise ReproduceError(f"{label}: expected a table")
        _require(step, "name", label, str)
        argv = _require(step, "argv", label, list)
        if not argv or not all(isinstance(item, str) for item in argv):
            raise ReproduceError(f"{label}.argv: expected a non-empty array of strings")
        env = step.get("env", {})
        if not isinstance(env, dict) or not all(isinstance(v, str) for v in env.values()):
            raise ReproduceError(f"{label}.env: expected a table of strings")
        foreach = step.get("foreach")
        if foreach is not None:
            if not isinstance(foreach, list) or not all(isinstance(row, dict) for row in foreach):
                raise ReproduceError(f"{label}.foreach: expected an array of inline tables")
        validated.append(step)
    return validated


METRIC_NUMERIC_KEYS = ("expected", "abs_tol", "rel_tol", "min", "max")


def _substitute_row(text: str, row: Mapping[str, Any]) -> str:
    """Replace only ``{key}`` placeholders that the foreach row defines."""

    def replace(match: re.Match[str]) -> str:
        key, argument = match.group(1), match.group(2)
        if argument is None and key in row:
            return str(row[key])
        return match.group(0)

    return PLACEHOLDER_RE.sub(replace, text)


def expand_metric_specs(metrics: Sequence[Mapping[str, Any]], where: str) -> list[dict[str, Any]]:
    """Flatten ``foreach`` metric tables; row numeric keys override the template."""
    flat: list[dict[str, Any]] = []
    for index, metric in enumerate(metrics):
        if not isinstance(metric, dict):
            raise ReproduceError(f"{where}[{index}]: expected a table")
        rows = metric.get("foreach")
        if rows is None:
            flat.append({key: value for key, value in metric.items()})
            continue
        if not isinstance(rows, list) or not all(isinstance(row, dict) for row in rows):
            raise ReproduceError(f"{where}[{index}].foreach: expected an array of inline tables")
        for row in rows:
            spec = {key: value for key, value in metric.items() if key != "foreach"}
            for key in ("name", "source", "path", "readme", "minus_source", "minus_path"):
                if isinstance(spec.get(key), str):
                    spec[key] = _substitute_row(spec[key], row)
            for key in METRIC_NUMERIC_KEYS:
                if key in row:
                    spec[key] = row[key]
            flat.append(spec)
    return flat


def _validate_metrics(metrics: Any, where: str) -> list[dict[str, Any]]:
    if metrics is None:
        return []
    if not isinstance(metrics, list):
        raise ReproduceError(f"{where}: expected an array of tables")
    metrics = expand_metric_specs(metrics, where)
    for index, metric in enumerate(metrics):
        label = f"{where}[{index}]"
        if not isinstance(metric, dict):
            raise ReproduceError(f"{label}: expected a table")
        _require(metric, "name", label, str)
        _require(metric, "source", label, str)
        _require(metric, "path", label, str)
        has_expected = "expected" in metric
        has_bound = "min" in metric or "max" in metric
        if not has_expected and not has_bound:
            raise ReproduceError(f"{label}: set `expected` (with a tolerance) or `min`/`max`")
        if has_expected and isinstance(metric["expected"], bool):
            pass  # boolean gate (e.g. a sign-off `hard_pass`): exact match
        elif has_expected:
            if not isinstance(metric["expected"], (int, float)):
                raise ReproduceError(f"{label}.expected: expected a number or boolean")
            if "abs_tol" not in metric and "rel_tol" not in metric:
                raise ReproduceError(f"{label}: `expected` needs `abs_tol` or `rel_tol`")
        if "gate" in metric and not isinstance(metric["gate"], bool):
            raise ReproduceError(f"{label}.gate: expected a boolean")
        for key in ("abs_tol", "rel_tol", "min", "max"):
            if key in metric and (
                not isinstance(metric[key], (int, float)) or isinstance(metric[key], bool)
            ):
                raise ReproduceError(f"{label}.{key}: expected a number")
    return metrics


def load_manifest(path: Path) -> dict[str, Any]:
    try:
        with path.open("rb") as handle:
            payload = tomllib.load(handle)
    except tomllib.TOMLDecodeError as exc:
        raise ReproduceError(f"{path}: invalid TOML: {exc}") from exc
    lane = _require(payload, "lane", str(path), dict)
    name = _require(lane, "name", f"{path}:lane", str)
    status = lane.get("status", "ready")
    if status not in LANE_STATUSES:
        raise ReproduceError(f"{path}:lane.status: expected one of {', '.join(LANE_STATUSES)}")
    _require(lane, "title", f"{path}:lane", str)
    datasets = payload.get("datasets", {})
    if not isinstance(datasets, dict):
        raise ReproduceError(f"{path}:datasets: expected a table")
    for key, spec in datasets.items():
        if key not in DATASET_ENV:
            raise ReproduceError(f"{path}:datasets.{key}: unknown dataset (known: {', '.join(DATASET_ENV)})")
        if not isinstance(spec, dict):
            raise ReproduceError(f"{path}:datasets.{key}: expected a table")
        required = spec.get("required", [])
        if not isinstance(required, list) or not all(isinstance(item, str) for item in required):
            raise ReproduceError(f"{path}:datasets.{key}.required: expected an array of strings")
    tools = payload.get("tools", {})
    if not isinstance(tools, dict):
        raise ReproduceError(f"{path}:tools: expected a table")
    payload["steps"] = _validate_steps(payload.get("steps"), f"{path}:steps")
    payload["docs_steps"] = _validate_steps(payload.get("docs_steps"), f"{path}:docs_steps")
    payload["metrics"] = _validate_metrics(payload.get("metrics"), f"{path}:metrics")
    if status == "ready" and not payload["steps"]:
        raise ReproduceError(f"{path}: ready lane `{name}` declares no steps")
    payload["_path"] = str(path)
    return payload


def discover_manifests(manifest_dir: Path = MANIFEST_DIR) -> dict[str, dict[str, Any]]:
    manifests: dict[str, dict[str, Any]] = {}
    for path in sorted(manifest_dir.glob("*.toml")):
        manifest = load_manifest(path)
        name = manifest["lane"]["name"]
        if name in manifests:
            raise ReproduceError(f"duplicate lane name `{name}` in {path}")
        manifests[name] = manifest
    return manifests


# --------------------------------------------------------------------------
# Command rendering
# --------------------------------------------------------------------------


def find_build_binary(name: str, build_dir: Path | None) -> str | None:
    filename = name + EXE_SUFFIX
    roots: list[Path] = []
    if build_dir is not None:
        roots.append(build_dir)
    roots.extend(sorted(path for path in ROOT_DIR.glob("build*") if path.is_dir()))
    for root in roots:
        candidates = [root / "apps" / filename, root / filename]
        for config in BUILD_CONFIGS:
            candidates.append(root / "apps" / config / filename)
            candidates.append(root / config / "apps" / filename)
        for candidate in candidates:
            if candidate.is_file():
                return str(candidate)
    return None


def _portable(value: Path | str) -> str:
    """Render a path with forward slashes, repo-relative when inside the repo.

    Steps run with ``cwd`` set to the repository root, so repo-relative paths
    keep rendered commands short and identical across checkouts.
    """
    path = Path(value)
    if path.is_absolute():
        try:
            path = path.resolve().relative_to(ROOT_DIR.resolve())
        except ValueError:
            pass
    return str(path).replace("\\", "/")


def repo_path(value: str | Path) -> Path:
    """Resolve a rendered (possibly repo-relative) path for in-process use."""
    path = Path(value)
    return path if path.is_absolute() else ROOT_DIR / path


def expand_foreach(step: Mapping[str, Any]) -> list[dict[str, str]]:
    rows = step.get("foreach")
    if not rows:
        return [{}]
    expanded: list[dict[str, str]] = []
    for row in rows:
        expanded.append({str(key): str(value) for key, value in row.items()})
    return expanded


def render_text(text: str, context: Mapping[str, str], *, build_dir: Path | None, strict: bool) -> str:
    def replace(match: re.Match[str]) -> str:
        key, argument = match.group(1), match.group(2)
        if key == "bin":
            if argument is None:
                raise ReproduceError("`{bin:NAME}` needs a binary name")
            found = find_build_binary(argument, build_dir)
            if found is None:
                if strict:
                    raise ReproduceError(
                        f"built binary `{argument}` not found; pass --build-dir or build the target"
                    )
                return _portable(Path(build_dir or ROOT_DIR / "build") / "apps" / (argument + EXE_SUFFIX))
            return _portable(found)
        if argument is not None:
            raise ReproduceError(f"unsupported placeholder `{match.group(0)}`")
        if key not in context:
            raise ReproduceError(f"unknown placeholder `{{{key}}}` in `{text}`")
        return context[key]

    return PLACEHOLDER_RE.sub(replace, text)


def render_argv(
    argv: Sequence[str],
    context: Mapping[str, str],
    *,
    build_dir: Path | None,
    strict: bool,
) -> list[str]:
    rendered: list[str] = []
    for item in argv:
        if item == "{gnss}":
            rendered.extend([context["python"], _portable(ROOT_DIR / "apps" / "gnss.py")])
        elif item == "{python}":
            rendered.append(context["python"])
        else:
            rendered.append(render_text(item, context, build_dir=build_dir, strict=strict))
    return rendered


def render_steps(
    steps: Iterable[Mapping[str, Any]],
    context: Mapping[str, str],
    *,
    build_dir: Path | None,
    strict: bool,
) -> list[dict[str, Any]]:
    rendered_steps: list[dict[str, Any]] = []
    for step in steps:
        for row in expand_foreach(step):
            local = {**context, **row}
            name = render_text(step["name"], local, build_dir=build_dir, strict=strict)
            rendered_steps.append(
                {
                    "name": name,
                    "argv": render_argv(step["argv"], local, build_dir=build_dir, strict=strict),
                    "env": {
                        key: render_text(value, local, build_dir=build_dir, strict=strict)
                        for key, value in step.get("env", {}).items()
                    },
                    "outputs": [
                        render_text(value, local, build_dir=build_dir, strict=strict)
                        for value in step.get("outputs", [])
                    ],
                }
            )
    return rendered_steps


def format_command(step: Mapping[str, Any]) -> str:
    argv = list(step["argv"])
    joined = subprocess.list2cmdline(argv) if os.name == "nt" else shlex.join(argv)
    env_prefix = " ".join(f"{key}={value}" for key, value in sorted(step["env"].items()))
    return f"{env_prefix} {joined}".strip()


# --------------------------------------------------------------------------
# Metric checks
# --------------------------------------------------------------------------


def lookup_path(payload: Any, path: str) -> Any:
    """Resolve ``a.b[0].c`` or ``runs[label=tokyo_run1].value`` in JSON data."""
    current = payload
    for match in PATH_TOKEN_RE.finditer(path):
        key, selector = match.group(1), match.group(2)
        if key is not None:
            if not isinstance(current, dict) or key not in current:
                raise KeyError(path)
            current = current[key]
            continue
        assert selector is not None
        if "=" in selector:
            field, wanted = selector.split("=", 1)
            if not isinstance(current, list):
                raise KeyError(path)
            hits = [row for row in current if isinstance(row, dict) and str(row.get(field)) == wanted]
            if len(hits) != 1:
                raise KeyError(path)
            current = hits[0]
        else:
            if not isinstance(current, list):
                raise KeyError(path)
            current = current[int(selector)]
    return current


def evaluate_metric(spec: Mapping[str, Any], observed: Any) -> dict[str, Any]:
    result: dict[str, Any] = {
        "name": spec["name"],
        "readme": spec.get("readme"),
        "observed": observed,
        "expected": spec.get("expected"),
        "abs_tol": spec.get("abs_tol"),
        "rel_tol": spec.get("rel_tol"),
        "min": spec.get("min"),
        "max": spec.get("max"),
        "passed": True,
        "reasons": [],
    }
    if isinstance(spec.get("expected"), bool):
        if observed is not spec["expected"]:
            result["passed"] = False
            result["reasons"].append(f"observed {observed!r} != expected {spec['expected']!r}")
        return result
    if isinstance(observed, bool) or not isinstance(observed, (int, float)) or not math.isfinite(observed):
        result["passed"] = False
        result["reasons"].append(f"observed value is not a finite number: {observed!r}")
        return result
    value = float(observed)
    if "expected" in spec:
        expected = float(spec["expected"])
        tolerance = max(
            float(spec.get("abs_tol", 0.0)),
            abs(expected) * float(spec.get("rel_tol", 0.0)),
        )
        result["delta"] = value - expected
        result["tolerance"] = tolerance
        if abs(value - expected) > tolerance + 1e-12:
            result["passed"] = False
            result["reasons"].append(
                f"|{value:.6g} - {expected:.6g}| = {abs(value - expected):.6g} > tol {tolerance:.6g}"
            )
    if "min" in spec and value < float(spec["min"]) - 1e-12:
        result["passed"] = False
        result["reasons"].append(f"{value:.6g} < min {float(spec['min']):.6g}")
    if "max" in spec and value > float(spec["max"]) + 1e-12:
        result["passed"] = False
        result["reasons"].append(f"{value:.6g} > max {float(spec['max']):.6g}")
    return result


def check_metrics(
    metrics: Sequence[Mapping[str, Any]],
    context: Mapping[str, str],
    *,
    loader=None,
) -> list[dict[str, Any]]:
    cache: dict[str, Any] = {}

    def default_loader(path: str) -> Any:
        if path not in cache:
            cache[path] = json.loads(repo_path(path).read_text(encoding="utf-8"))
        return cache[path]

    load = loader or default_loader
    results: list[dict[str, Any]] = []
    for spec in metrics:
        source = render_text(spec["source"], context, build_dir=None, strict=False)
        minus_path = spec.get("minus_path")
        minus_source = (
            render_text(spec.get("minus_source", spec["source"]), context, build_dir=None, strict=False)
            if minus_path
            else None
        )
        missing_path = spec["path"]
        missing_source = source
        try:
            observed = lookup_path(load(source), spec["path"])
            if minus_path:
                missing_path, missing_source = minus_path, minus_source
                other = lookup_path(load(minus_source), minus_path)
                # A relational claim ("A beats B") is gated on A - B.
                if all(
                    isinstance(v, (int, float)) and not isinstance(v, bool) for v in (observed, other)
                ):
                    observed = observed - other
                else:
                    observed = None
        except FileNotFoundError:
            result = evaluate_metric(spec, None)
            result["reasons"] = [f"metrics source missing: {missing_source}"]
        except (KeyError, IndexError, ValueError):
            result = evaluate_metric(spec, None)
            result["reasons"] = [f"path `{missing_path}` not found in {missing_source}"]
        else:
            result = evaluate_metric(spec, observed)
        result["source"] = source
        result["path"] = spec["path"] if not minus_path else f"{spec['path']} - {minus_path}"
        result["gate"] = bool(spec.get("gate", True))
        results.append(result)
    return results


def format_check_table(results: Sequence[Mapping[str, Any]]) -> str:
    lines = [
        "| metric | README | expected | observed | status |",
        "|---|---|---:|---:|---|",
    ]
    for result in results:
        expected_parts: list[str] = []
        if isinstance(result.get("expected"), bool):
            expected_parts.append(str(result["expected"]).lower())
        elif result.get("expected") is not None:
            tol = result.get("tolerance")
            tol_text = f" +/- {tol:.3g}" if isinstance(tol, (int, float)) else ""
            expected_parts.append(f"{result['expected']:g}{tol_text}")
        if result.get("min") is not None:
            expected_parts.append(f">= {result['min']:g}")
        if result.get("max") is not None:
            expected_parts.append(f"<= {result['max']:g}")
        observed = result.get("observed")
        if isinstance(observed, bool):
            observed_text = str(observed).lower()
        elif isinstance(observed, (int, float)):
            observed_text = f"{observed:.6g}"
        else:
            observed_text = "n/a"
        if result["passed"]:
            status = "PASS"
        elif result.get("gate", True):
            status = "FAIL: " + "; ".join(result["reasons"])
        else:
            status = "INFO (not gated): " + "; ".join(result["reasons"])
        lines.append(
            f"| {result['name']} | {result.get('readme') or ''} | {', '.join(expected_parts)} | "
            f"{observed_text} | {status} |"
        )
    return "\n".join(lines)


# --------------------------------------------------------------------------
# Context and execution
# --------------------------------------------------------------------------


def dataset_root(name: str, args: argparse.Namespace) -> Path | None:
    explicit = getattr(args, f"{name}_root", None)
    if explicit is not None:
        return Path(explicit)
    env_value = os.environ.get(DATASET_ENV[name])
    if env_value:
        return Path(env_value)
    if name in DATASET_FALLBACK:
        return dataset_root(DATASET_FALLBACK[name], args)
    if args.data_root is not None:
        return Path(args.data_root) / DEFAULT_DATA_SUBDIRS[name]
    default = ROOT_DIR / DEFAULT_DATA_SUBDIRS[name]
    return default


DEFAULT_DATA_SUBDIRS = {
    "ppc": Path("data") / "PPC-Dataset",
    "urbannav": Path("data") / "driving" / "Tokyo_Data",
    "gsdc": Path("data") / "gsdc2023" / "dataset_2023",
    "ppc_goal_inputs": Path("data") / "ppc_goal_inputs",
}


def build_context(manifest: Mapping[str, Any], args: argparse.Namespace) -> dict[str, str]:
    lane_name = manifest["lane"]["name"]
    work_dir = Path(args.work_dir) if args.work_dir else ROOT_DIR / "output" / "reproduce" / lane_name
    context = {
        "repo": ".",
        "python": _portable(sys.executable),
        "lane": lane_name,
        "work_dir": _portable(work_dir.resolve() if work_dir.is_absolute() else Path.cwd() / work_dir),
        "build_dir": _portable(Path(args.build_dir).resolve()) if args.build_dir else _portable(ROOT_DIR / "build"),
        "rtklib_bin": _portable(args.rtklib_bin) if args.rtklib_bin else os.environ.get("RTKLIB_RNX2RTKP", ""),
    }
    for name in DATASET_ENV:
        root = dataset_root(name, args)
        context[f"{name}_root"] = _portable(root) if root is not None else ""
    for key, value in manifest.get("vars", {}).items():
        context[str(key)] = render_text(str(value), context, build_dir=None, strict=False)
    return context


def dataset_option(name: str) -> str:
    return DATASET_OPTION.get(name, f"--{name.replace('_', '-')}-root")


def validate_inputs(manifest: Mapping[str, Any], context: Mapping[str, str], rendered: Sequence[Mapping[str, Any]]) -> list[str]:
    problems: list[str] = []
    for name, spec in manifest.get("datasets", {}).items():
        root = repo_path(context[f"{name}_root"])
        missing = [item for item in spec.get("required", []) if not (root / item).exists()]
        if missing:
            env_name = DATASET_ENV[name]
            problems.append(
                f"dataset `{name}` incomplete under {root} (set {dataset_option(name)} or {env_name}); missing: "
                + ", ".join(missing[:6])
                + (" ..." if len(missing) > 6 else "")
            )
    uses_rtklib = any("{rtklib_bin}" in item for step in manifest["steps"] for item in step["argv"])
    if uses_rtklib:
        rtklib = context.get("rtklib_bin", "")
        if not rtklib or not repo_path(rtklib).is_file():
            problems.append(
                "RTKLIB demo5 rnx2rtkp not found; pass --rtklib-bin or set RTKLIB_RNX2RTKP "
                "(see docs/reproduce.md for the pinned build)"
            )
    return problems


def run_steps(
    steps: Sequence[Mapping[str, Any]],
    *,
    build_dir: Path | None,
    log_dir: Path,
) -> list[dict[str, Any]]:
    timings: list[dict[str, Any]] = []
    log_dir.mkdir(parents=True, exist_ok=True)
    base_env = os.environ.copy()
    if build_dir is not None:
        base_env["GNSSPP_BUILD_DIR"] = str(build_dir)
    for index, step in enumerate(steps, start=1):
        print(f"[{index}/{len(steps)}] {step['name']}", flush=True)
        print(f"  $ {format_command(step)}", flush=True)
        env = {**base_env, **step["env"]}
        log_path = log_dir / f"{index:02d}_{re.sub(r'[^A-Za-z0-9_.-]+', '_', step['name'])}.log"
        started = time.monotonic()
        with log_path.open("w", encoding="utf-8", errors="replace") as log:
            log.write(f"$ {format_command(step)}\n")
            log.flush()
            process = subprocess.Popen(
                step["argv"],
                cwd=ROOT_DIR,
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                encoding="utf-8",
                errors="replace",
            )
            assert process.stdout is not None
            tail: list[str] = []
            for line in process.stdout:
                log.write(line)
                tail.append(line)
                if len(tail) > 40:
                    tail.pop(0)
            returncode = process.wait()
        elapsed = time.monotonic() - started
        timings.append({"name": step["name"], "elapsed_s": round(elapsed, 3), "returncode": returncode, "log": str(log_path)})
        print(f"  -> exit {returncode} in {elapsed:.1f} s (log: {log_path})", flush=True)
        if returncode != 0:
            sys.stdout.write("".join(tail))
            raise ReproduceError(f"step `{step['name']}` failed with exit {returncode}; see {log_path}")
    return timings


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------


def add_common_options(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--data-root", type=Path, default=None,
                        help="Parent directory holding PPC-Dataset/ and driving/Tokyo_Data/ (default: <repo>/data).")
    parser.add_argument("--ppc-root", type=Path, default=None,
                        help="PPC-Dataset root (default: $GNSSPP_PPC_DATASET_ROOT or <data-root>/PPC-Dataset).")
    parser.add_argument("--urbannav-root", type=Path, default=None,
                        help="UrbanNav Tokyo_Data root holding Odaiba/ (default: $GNSSPP_URBANNAV_ROOT).")
    parser.add_argument("--gsdc-root", type=Path, default=None,
                        help="GSDC 2023 dataset_2023 root holding train/<drive>/ (default: $GNSSPP_GSDC_ROOT "
                             "or <data-root>/gsdc2023/dataset_2023).")
    parser.add_argument("--gsdc-truth-root", type=Path, default=None,
                        help="Where ground_truth.csv lives if not inside --gsdc-root "
                             "(default: $GNSSPP_GSDC_TRUTH_ROOT, then the GSDC root).")
    parser.add_argument("--ppc-goal-inputs", dest="ppc_goal_inputs_root", type=Path, default=None,
                        help="Frozen PPC goal-matrix tier inputs for the ppc-goal lane "
                             "(default: $GNSSPP_PPC_GOAL_INPUTS or <data-root>/ppc_goal_inputs).")
    parser.add_argument("--build-dir", type=Path, default=os.environ.get("GNSSPP_BUILD_DIR"),
                        help="CMake build directory holding apps/gnss_* binaries (default: $GNSSPP_BUILD_DIR or <repo>/build*).")
    parser.add_argument("--rtklib-bin", type=Path, default=None,
                        help="RTKLIB demo5 rnx2rtkp binary (default: $RTKLIB_RNX2RTKP).")
    parser.add_argument("--work-dir", type=Path, default=None,
                        help="Output directory for this lane (default: output/reproduce/<lane>).")
    parser.add_argument("--check", action="store_true",
                        help="Compare produced metrics with the manifest expectations; exit 3 on drift.")
    parser.add_argument("--check-only", action="store_true",
                        help="Skip the steps and only check metrics already present in --work-dir "
                             "(with --update-docs, still re-render the lane's docs from those outputs).")
    parser.add_argument("--update-docs", action="store_true",
                        help="Also regenerate the tracked docs artifacts owned by this lane.")
    parser.add_argument("--dry-run", action="store_true",
                        help="Print the rendered commands without running them.")
    parser.add_argument("--manifest", type=Path, default=None,
                        help="Use this lane manifest instead of configs/reproduce/<lane>.toml.")


def parse_args(argv: Sequence[str], lanes: Sequence[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME", "gnss reproduce"),
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    subparsers = parser.add_subparsers(dest="lane", metavar="{list," + ",".join(lanes) + "}")
    list_parser = subparsers.add_parser("list", help="List lanes and README coverage.")
    list_parser.add_argument("--json", action="store_true", help="Emit JSON.")
    for lane in lanes:
        lane_parser = subparsers.add_parser(lane, help=f"Run the `{lane}` lane.")
        add_common_options(lane_parser)
    args = parser.parse_args(argv)
    if args.lane is None:
        parser.print_help()
        raise SystemExit(1)
    return args


def list_lanes(manifests: Mapping[str, Mapping[str, Any]], as_json: bool) -> int:
    rows = []
    for name, manifest in manifests.items():
        lane = manifest["lane"]
        rows.append(
            {
                "lane": name,
                "status": lane.get("status", "ready"),
                "readme_row": lane.get("readme_row", ""),
                "title": lane["title"],
                "runtime_estimate": lane.get("runtime_estimate", ""),
                "manifest": os.path.relpath(manifest["_path"], ROOT_DIR).replace("\\", "/"),
            }
        )
    if as_json:
        print(json.dumps({"lanes": rows}, indent=2))
        return 0
    print(f"{'lane':<18} {'status':<8} {'runtime':<12} README row")
    for row in rows:
        print(f"{row['lane']:<18} {row['status']:<8} {row['runtime_estimate']:<12} {row['readme_row']}")
    return 0


def run_lane(manifest: Mapping[str, Any], args: argparse.Namespace) -> int:
    lane = manifest["lane"]
    if lane.get("status", "ready") != "ready":
        print(f"Lane `{lane['name']}` is planned and has no runnable steps yet: {lane['title']}", file=sys.stderr)
        return 2
    build_dir = Path(args.build_dir).resolve() if args.build_dir else None
    context = build_context(manifest, args)
    strict = not args.dry_run
    # --check-only never runs the lane steps, so their binaries need not exist.
    steps = render_steps(manifest["steps"], context, build_dir=build_dir,
                         strict=strict and not args.check_only)
    docs_steps = (
        render_steps(manifest["docs_steps"], context, build_dir=build_dir, strict=strict)
        if args.update_docs
        else []
    )
    work_dir = repo_path(context["work_dir"])

    if args.dry_run:
        print(f"# lane: {lane['name']} - {lane['title']}")
        for step in [*steps, *docs_steps]:
            print(f"# {step['name']}")
            print(format_command(step))
        problems = validate_inputs(manifest, context, steps)
        for problem in problems:
            print(f"# warning: {problem}")
        return 0

    timings: list[dict[str, Any]] = []
    started = time.monotonic()
    if not args.check_only:
        problems = validate_inputs(manifest, context, steps)
        if problems:
            for problem in problems:
                print(f"Error: {problem}", file=sys.stderr)
            return 1
        work_dir.mkdir(parents=True, exist_ok=True)
        timings = run_steps([*steps, *docs_steps], build_dir=build_dir, log_dir=work_dir / "logs")
    elif docs_steps:
        # --check-only --update-docs: re-render tracked docs from existing outputs.
        timings = run_steps(docs_steps, build_dir=build_dir, log_dir=work_dir / "logs")
    elapsed = time.monotonic() - started

    results = check_metrics(manifest["metrics"], context)
    passed = all(result["passed"] for result in results if result.get("gate", True))
    work_dir.mkdir(parents=True, exist_ok=True)
    report = {
        "schema": "gnss_reproduce_result.v1",
        "lane": lane["name"],
        "title": lane["title"],
        "manifest": manifest["_path"],
        "work_dir": str(work_dir),
        "elapsed_s": round(elapsed, 3),
        "steps": timings,
        "metrics": results,
        "passed": passed,
    }
    result_path = work_dir / "reproduce_result.json"
    result_path.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print()
    print(format_check_table(results))
    print(f"\nwall time: {elapsed:.1f} s; result: {result_path}", flush=True)
    if args.check and not passed:
        print("Error: reproduced metrics drifted from the manifest expectations.", file=sys.stderr)
        return 3
    return 0


def main(argv: Sequence[str] | None = None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    try:
        manifests = discover_manifests()
        lanes = list(manifests)
        # --manifest lets a caller run an untracked lane file.
        if "--manifest" in argv:
            index = argv.index("--manifest")
            if index + 1 < len(argv):
                custom = load_manifest(Path(argv[index + 1]))
                manifests[custom["lane"]["name"]] = custom
                if custom["lane"]["name"] not in lanes:
                    lanes.append(custom["lane"]["name"])
        args = parse_args(argv, lanes)
        if args.lane == "list":
            return list_lanes(manifests, args.json)
        return run_lane(manifests[args.lane], args)
    except ReproduceError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
