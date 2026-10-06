#!/usr/bin/env python3
"""Build and regenerate a PPC native baseline from raw GNSS/IMU inputs."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import subprocess
import sys
import tarfile
import time
from typing import Any

from support.gnss_runtime import application_root, resolve_gnss_command

ROOT = application_root(__file__)
RUNS = tuple(f"{city}/run{number}" for city in ("tokyo", "nagoya") for number in (1, 2, 3))
COMMON = ["--preset", "low-cost", "--ratio", "2.4", "--max-subset-ar-drop-steps", "18",
          "--rtk-snr-weighting", "--no-arfilter"]
LEVER_ARMS = {"tokyo": "0.31,0,0.55", "nagoya": "0.593,-0.670,-1.216"}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def file_record(path: Path) -> dict[str, Any]:
    return {"path": str(path.resolve()), "bytes": path.stat().st_size, "sha256": sha256(path)}


def solution_data_sha256(path: Path) -> str:
    """Ignore headers and whitespace; preserve every emitted numeric field."""
    lines = [" ".join(line.split()) for line in path.read_text(encoding="utf-8").splitlines()
             if line.strip() and not line.lstrip().startswith(("%", "#"))]
    if not lines:
        raise ValueError(f"solution has no data rows: {path}")
    return hashlib.sha256("\n".join(lines).encode()).hexdigest()


def compare_reports(baseline: dict, candidate: dict) -> dict:
    if baseline.get("state") != "passed":
        raise ValueError("comparison baseline must be a successful replay")
    for key in ("evaluation", "max_epochs", "runs", "paths"):
        if baseline.get(key) != candidate.get(key):
            raise ValueError(f"comparison recipe differs: {key}")
    if baseline.get("fix_recovery", False) != candidate.get("fix_recovery", False):
        raise ValueError("comparison recipe differs: fix_recovery")
    if baseline.get("solver_environment", {}) != candidate.get("solver_environment", {}):
        raise ValueError("comparison solver environment differs")
    if baseline["source"]["contents_sha256"] != candidate["source"]["contents_sha256"]:
        raise ValueError("comparison source contents differ")
    for key in ("inputs", "binaries", "runtime_libraries"):
        if baseline[key] != candidate[key]:
            raise ValueError(f"comparison provenance differs: {key}")
    previous = {(r["run"], r["stream"]): r["solution_data_sha256"] for r in baseline["results"]}
    current = {(r["run"], r["stream"]): r["solution_data_sha256"] for r in candidate["results"]}
    if previous.keys() != current.keys():
        raise ValueError("comparison output population differs")
    differences = [f"{run}:{stream}" for run, stream in previous if previous[(run, stream)] != current[(run, stream)]]
    return {"passed": not differences, "compared_streams": len(current),
            "different_streams": differences, "scope": "all POS data fields; headers excluded"}


def git(root: Path, *arguments: str) -> bytes:
    return subprocess.check_output(["git", "-C", str(root), *arguments], stderr=subprocess.PIPE)


def solver_environment(root: Path, env: dict[str, str]) -> dict[str, str | None]:
    """Record native algorithm knobs, including unset defaults, without account credentials."""
    files = [root / "apps/native/gnss_solve.cpp", root / "apps/native/gnss_fuse.cpp"]
    for directory in ("src/algorithms", "src/fusion", "include/libgnss++/algorithms", "include/libgnss++/fusion"):
        files.extend(path for path in (root / directory).rglob("*") if path.suffix in (".cpp", ".hpp"))
    names = set()
    for path in files:
        if path.is_file():
            names.update(re.findall(r'"(GNSS_[A-Z0-9_]+)"', path.read_text(encoding="utf-8")))
    return {name: env.get(name) for name in sorted(names)}


def source_snapshot(root: Path) -> dict[str, Any]:
    names = sorted(set(git(root, "ls-files", "--cached", "--others", "--exclude-standard", "-z")
                       .decode("utf-8").split("\0")) - {""})
    files = {}
    for name in names:
        path = root / name
        if path.is_symlink():
            raise ValueError(f"source snapshot does not accept symlinks: {name}")
        files[name] = sha256(path) if path.is_file() else None
    digest = hashlib.sha256(json.dumps(files, sort_keys=True).encode()).hexdigest()
    return {"revision": git(root, "rev-parse", "HEAD").decode().strip(),
            "status": git(root, "status", "--porcelain=v1").decode(),
            "contents_sha256": digest, "files": files}


def archive_source(root: Path, snapshot: dict, out: Path) -> None:
    # Includes edited and untracked, non-ignored source; deleted files stay absent.
    # Extraction into an empty directory reconstructs the exact build input tree.
    with tarfile.open(out, "w:gz") as archive:
        for name, digest in snapshot["files"].items():
            if digest is not None:
                archive.add(root / name, arcname=name, recursive=False)


def read_cache(build_dir: Path, root: Path) -> dict[str, str]:
    cache_path = build_dir / "CMakeCache.txt"
    result = {}
    for line in cache_path.read_text(encoding="utf-8").splitlines():
        if line.startswith(("#", "//")) or "=" not in line or ":" not in line.split("=", 1)[0]:
            continue
        key, value = line.split("=", 1)
        result[key.split(":", 1)[0]] = value
    home = result.get("CMAKE_HOME_DIRECTORY")
    if home is None or Path(home).resolve() != root.resolve():
        raise ValueError("build directory must be configured from this source tree")
    return result


def find_binary(build_dir: Path, config: str, target: str) -> Path:
    name = target + (".exe" if os.name == "nt" else "")
    candidates = [build_dir / "apps" / config / name, build_dir / "apps" / name]
    for candidate in candidates:
        if candidate.is_file():
            return candidate.resolve()
    raise ValueError(f"missing built executable {target} under {build_dir}")


def prepare_output(out: Path) -> None:
    if out.exists() and (not out.is_dir() or any(out.iterdir())):
        raise ValueError("output directory must be new or empty; old solutions are never reused")
    out.mkdir(parents=True, exist_ok=True)


def check_output_location(out: Path, root: Path) -> None:
    try:
        relative = out.relative_to(root)
    except ValueError:
        return
    result = subprocess.run(["git", "-C", str(root), "check-ignore", "-q",
                             str(relative / "native-replay-artifact")], capture_output=True)
    if result.returncode != 0:
        raise ValueError("in-tree output must be Git-ignored (use output/); otherwise it contaminates the source snapshot")


def write_json(path: Path, value: Any) -> None:
    temp = path.with_suffix(path.suffix + ".tmp")
    temp.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
    temp.replace(path)


def run_step(argv: list[str], log: Path, env: dict[str, str], report: dict, manifest: Path) -> float:
    entry = {"argv": argv, "cwd": str(ROOT), "log": str(log), "state": "running"}
    report["steps"].append(entry)
    write_json(manifest, report)
    print(f"Running {log.stem}", flush=True)
    started = time.perf_counter()
    try:
        with log.open("w", encoding="utf-8") as handle:
            process = subprocess.run(argv, cwd=ROOT, env=env, stdout=handle, stderr=subprocess.STDOUT)
    except OSError as error:
        entry.update(state="failed", wall_s=time.perf_counter() - started, error=str(error))
        write_json(manifest, report)
        raise
    wall = time.perf_counter() - started
    entry.update(exit_code=process.returncode, wall_s=wall, state="passed" if process.returncode == 0 else "failed")
    write_json(manifest, report)
    if process.returncode != 0:
        raise ValueError(f"command failed ({process.returncode}); inspect {log}")
    return wall


def solver_commands(binaries: dict[str, Path], run_dir: Path, city: str, out: Path,
                    paths: list[str], max_epochs: int, fix_recovery: bool = False) -> list[tuple[str, list[str], list[tuple[str, Path]]]]:
    cap = ["--max-epochs", str(max_epochs)] if max_epochs > 0 else []
    commands = []
    if "rtk" in paths:
        pos = out / "rtk.pos"
        argv = [str(binaries["gnss_solve"]), "--rover", str(run_dir / "rover.obs"),
                "--base", str(run_dir / "base.obs"), "--nav", str(run_dir / "base.nav"),
                "--mode", "kinematic", "--no-kml", *COMMON, *cap, "--out", str(pos),
                "--debug-epoch-log", str(out / "rtk_debug.csv")]
        if fix_recovery:
            argv.extend(["--fix-recovery-log", str(out / "fix_recovery.csv")])
        commands.append(("rtk", argv, [("rtk", pos)]))
    if "fusion" in paths:
        fused, coupled = out / "fused.pos", out / "coupled_rtk.pos"
        argv = [str(binaries["gnss_fuse"]), "--data-dir", str(run_dir),
                "--lever-arm", LEVER_ARMS[city], *COMMON, "--navi776-tc", *cap,
                "--out", str(fused), "--rtk-pos-out", str(coupled)]
        commands.append(("fusion", argv, [("fused", fused), ("coupled_rtk", coupled)]))
    return commands


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"), description=__doc__)
    parser.add_argument("--dataset-root", required=True, type=Path)
    parser.add_argument("--build-dir", required=True, type=Path,
                        help="Existing CMake build configured from this checkout; targets are rebuilt.")
    parser.add_argument("--build-config", default="Release")
    parser.add_argument("--jobs", type=int, default=2)
    parser.add_argument("--runs", nargs="+", choices=RUNS, default=list(RUNS))
    parser.add_argument("--paths", nargs="+", choices=("rtk", "fusion"), default=["rtk", "fusion"])
    parser.add_argument("--fix-recovery", action="store_true",
                        help="Evaluate optional runtime FIX recovery; requires --paths rtk.")
    parser.add_argument("--max-epochs", type=int, default=-1,
                        help="-1: full run (default); a positive cap is a smoke, never full evidence.")
    parser.add_argument("--runtime-dir", type=Path, action="append", default=[],
                        help="Prepend a shared-library directory to PATH (Windows) or LD_LIBRARY_PATH.")
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--compare-to", type=Path,
                        help="Prior successful manifest from the identical source, inputs and binaries; fail on changed POS data.")
    args = parser.parse_args(argv)
    if args.max_epochs != -1 and args.max_epochs <= 0:
        parser.error("--max-epochs must be -1 or positive")
    if args.jobs < 1:
        parser.error("--jobs must be positive")
    if len(set(args.runs)) != len(args.runs) or len(set(args.paths)) != len(args.paths):
        parser.error("runs and paths must not contain duplicates")
    if args.fix_recovery and args.paths != ["rtk"]:
        parser.error("--fix-recovery requires --paths rtk; the fusion recipe is unchanged")
    return args


def replay(args: argparse.Namespace) -> dict:
    dataset, build, out = (path.expanduser().resolve() for path in
                           (args.dataset_root, args.build_dir, args.output_dir))
    cache = read_cache(build, ROOT)
    if not (ROOT / "apps" / "gnss.py").is_file():
        raise ValueError("native replay currently requires a source checkout")
    required = ["rover.obs", "base.obs", "base.nav", "reference.csv"]
    if "fusion" in args.paths:
        required.append("imu.csv")
    inputs = []
    for run in args.runs:
        for name in required:
            path = dataset / run / name
            if not path.is_file():
                raise ValueError(f"missing raw input: {path}")
            inputs.append(file_record(path))
    for directory in args.runtime_dir:
        if not directory.is_dir():
            raise ValueError(f"missing runtime library directory: {directory}")
    if args.compare_to is not None:
        baseline = json.loads(args.compare_to.read_text(encoding="utf-8"))
        if not isinstance(baseline, dict) or baseline.get("state") != "passed":
            raise ValueError("comparison baseline must be a successful replay")
    check_output_location(out, ROOT)
    prepare_output(out)
    manifest = out / "manifest.json"
    source = source_snapshot(ROOT)
    # Preserve the actual CMake settings, rather than only a hash of a cache
    # that might be overwritten by a later configure operation.
    (out / "CMakeCache.before.txt").write_bytes((build / "CMakeCache.txt").read_bytes())
    report = {"schema_version": 1, "state": "running",
              "scope": "existing PPC development data; fresh native baseline, not historical selected tiers",
              "evaluation": "full" if args.max_epochs == -1 else "smoke",
              "max_epochs": args.max_epochs, "runs": args.runs, "paths": args.paths,
              "fix_recovery": args.fix_recovery,
              "source": source, "inputs": inputs, "steps": [], "results": [],
              "platform": platform.platform(), "python": sys.version,
              "build": {"directory": str(build), "config": args.build_config,
                        "initial_cache": file_record(out / "CMakeCache.before.txt"),
                        "settings": cache},
              "runtime_libraries": [file_record(file) for directory in args.runtime_dir
                                    for file in sorted(directory.glob("*"))
                                    if file.is_file() and (file.suffix.lower() == ".dll" or ".so" in file.name or file.suffix == ".dylib")],
              "reference_used_in_inference": False,
              "causal_provenance_verified": False}
    write_json(manifest, report)
    env = os.environ.copy()
    library_key = "PATH" if os.name == "nt" else "LD_LIBRARY_PATH"
    if args.runtime_dir:
        env[library_key] = os.pathsep.join([*(str(d.resolve()) for d in args.runtime_dir), env.get(library_key, "")])
    # Solver paths and arguments never come from the legacy PPC environment overrides.
    env.pop("PPC_EXTRA_SOLVER_ARGS", None)
    env.pop("PPC_DEBUG_EPOCH_LOG", None)
    report["solver_environment"] = solver_environment(ROOT, env)
    write_json(manifest, report)
    try:
        archive_source(ROOT, source, out / "source.tar.gz")
        report["source_archive"] = file_record(out / "source.tar.gz")
        targets = ["gnss_solve"] if args.paths == ["rtk"] else (
            ["gnss_fuse"] if args.paths == ["fusion"] else ["gnss_solve", "gnss_fuse"])
        run_step(["cmake", "--build", str(build), "--config", args.build_config,
                  "--target", *targets, "--parallel", str(args.jobs)], out / "build.log", env, report, manifest)
        binaries = {target: find_binary(build, args.build_config, target) for target in targets}
        report["binaries"] = {target: file_record(path) for target, path in binaries.items()}
        (out / "CMakeCache.after.txt").write_bytes((build / "CMakeCache.txt").read_bytes())
        report["build"]["final_cache"] = file_record(out / "CMakeCache.after.txt")
        report["build"]["settings"] = read_cache(build, ROOT)
        if source_snapshot(ROOT)["contents_sha256"] != source["contents_sha256"]:
            raise ValueError("source contents changed during build")
        gnss = resolve_gnss_command(ROOT)
        for run in args.runs:
            city, number = run.split("/")
            run_out = out / city / number
            run_out.mkdir(parents=True)
            for label, argv, streams in solver_commands(binaries, dataset / run, city, run_out,
                                                       args.paths, args.max_epochs, args.fix_recovery):
                wall = run_step(argv, run_out / f"{label}.log", env, report, manifest)
                for stream, pos in streams:
                    if not pos.is_file() or pos.stat().st_size == 0:
                        raise ValueError(f"solver did not write a fresh solution: {pos}")
                    summary = run_out / f"{stream}_summary.json"
                    run_step([*gnss, "ppc-demo", "--dataset-root", str(dataset), "--city", city,
                              "--run", number, "--solver", "rtk", "--use-existing-solution",
                              "--max-epochs", str(args.max_epochs), "--out", str(pos),
                              "--summary-json", str(summary), "--solver-wall-time-s", str(wall),
                              "--require-matched-epochs-min", "1"],
                             run_out / f"{stream}_score.log", env, report, manifest)
                    metrics = json.loads(summary.read_text(encoding="utf-8"))
                    report["results"].append({"run": run, "stream": stream, "native_wall_s": wall,
                                              "solution": file_record(pos), "summary": file_record(summary),
                                              "solution_data_sha256": solution_data_sha256(pos),
                                              "metrics": metrics})
                    write_json(manifest, report)
        if source_snapshot(ROOT)["contents_sha256"] != source["contents_sha256"]:
            raise ValueError("source contents changed during replay")
        for record in inputs:
            if file_record(Path(record["path"])) != record:
                raise ValueError(f"input changed during replay: {record['path']}")
        for target, binary in binaries.items():
            if file_record(binary) != report["binaries"][target]:
                raise ValueError(f"executable changed during replay: {target}")
        for record in report["runtime_libraries"]:
            if file_record(Path(record["path"])) != record:
                raise ValueError(f"runtime library changed during replay: {record['path']}")
        if args.compare_to is not None:
            baseline = json.loads(args.compare_to.read_text(encoding="utf-8"))
            report["repeatability"] = compare_reports(baseline, report)
            if not report["repeatability"]["passed"]:
                raise ValueError("repeated native solutions differ; inspect repeatability in manifest")
        report["state"] = "passed"
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        report.update(state="failed", error=str(error))
        write_json(manifest, report)
        raise
    report["artifacts"] = [file_record(path) for path in sorted(out.rglob("*"))
                           if path.is_file() and path != manifest]
    write_json(manifest, report)
    return report


def main() -> int:
    try:
        report = replay(parse_args())
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        print(f"Native replay failed: {error}", file=sys.stderr)
        return 1
    print(f"Native replay passed ({report['evaluation']}): {len(report['results'])} scored streams")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
