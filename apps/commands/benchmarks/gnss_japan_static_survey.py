#!/usr/bin/env python3
"""Materialize and evaluate the frozen IGS Tsukuba static-survey example."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import gzip
import hashlib
import json
import os
from pathlib import Path
import platform
import shlex
import shutil
import subprocess
import sys
import tarfile
import tempfile
import time
from typing import Any
from urllib import request

from support.gnss_runtime import application_root
from support.gnss_sinex import read_station_estimate


ROOT_DIR = application_root(__file__)
DISPATCHER = ROOT_DIR / "apps" / "gnss.py"
SCHEMA_VERSION = "libgnsspp.japan_static_survey.v1"
OBSERVATION_EPOCH = datetime(2024, 1, 1, tzinfo=timezone.utc)
PROFILE = "igs-tsukuba-2024-001"
TRUTH_SOURCE = "IGS weekly combined SINEX GPS week 2295"
TRUTH_FRAME = "IGS20, weekly solution mean epoch 2024-01-03T12:00:00Z"
IGS_POLICY_URL = "https://igs.org/why-join-the-igs"


SOURCES: dict[str, dict[str, Any]] = {
    "rover_crx_gz": {
        "url": "https://igs.bkg.bund.de/root_ftp/IGS/obs/2024/001/TSK200JPN_R_20240010000_01D_30S_MO.crx.gz",
        "bytes": 2096006,
        "sha256": "734745d8f892e6a8823a66f4822e4cd6d7e87fc7211885a68c992e0233c55844",
    },
    "base_crx_gz": {
        "url": "https://igs.bkg.bund.de/root_ftp/IGS/obs/2024/001/TSKB00JPN_R_20240010000_01D_30S_MO.crx.gz",
        "bytes": 1458738,
        "sha256": "422ef15539e2566c80dace992e4b19b7bd3312cddbcc774632efc41115930319",
    },
    "nav_gz": {
        "url": "https://igs.bkg.bund.de/root_ftp/IGS/BRDC/2024/001/BRDC00IGS_R_20240010000_01D_MN.rnx.gz",
        "bytes": 1269498,
        "sha256": "17f1a43c5074df56ea261136214d5fda5d9befa0a5422c4768ad14651b252e49",
    },
    "sp3_gz": {
        "url": "https://igs.bkg.bund.de/root_ftp/IGS/products/2295/IGS0OPSFIN_20240010000_01D_15M_ORB.SP3.gz",
        "bytes": 102609,
        "sha256": "26f16fa5f97cce851948fa88711aed47f60ee2e4ddfe36f0e98b1076e79fbe38",
    },
    "clk_gz": {
        "url": "https://igs.bkg.bund.de/root_ftp/IGS/products/2295/IGS0OPSFIN_20240010000_01D_30S_CLK.CLK.gz",
        "bytes": 2523166,
        "sha256": "616966ced674614f2ca66889f39de4156a5d97f75b51da67670742a58b59b8ae",
    },
    "truth_snx_gz": {
        "url": "https://garner.ucsd.edu/pub/products/2295/IGS0OPSSNX_20233650000_07D_07D_SOL.SNX.gz",
        "bytes": 12720664,
        "sha256": "f0d95d00ef5b1f789a52154992a5935a140e04e9ad77ae8224df3fa01aa9850f",
    },
    "rover_log": {
        "url": "https://files.igs.org/pub/station/log/tsk200jpn_20220912.log",
        "bytes": 17189,
        "sha256": "5d86286bce6b43b9570c7fec860e7002782f74a1ceffbec2991737fec0b6ec59",
    },
    "base_log": {
        "url": "https://files.igs.org/pub/station/log/tskb00jpn_20221018.log",
        "bytes": 24508,
        "sha256": "0aefc2404998f6e283267dc56007eb440efa6f1e9d5eaf39c120e6014092781e",
    },
    "igs20_ssc": {
        "url": "https://files.igs.org/pub/station/coord/IGS20/IGS20.ssc",
        "bytes": 1190657,
        "sha256": "1c69db9bf9ce3b26c97dd9df1cbfa04618970a527521a4b6355c425b91dd18ee",
    },
    "antex": {
        "url": "https://files.igs.org/pub/station/general/igs20.atx",
        "bytes": 60295761,
        "sha256": "8715268e17e09e5447f4949d67cbd067e7f0f33d48dd698aafe14f5cffb26de2",
    },
    "rnxcmp_linux_x86_64": {
        "url": "https://terras.gsi.go.jp/ja/crx2rnx/RNXCMP_4.2.0_Linux_gcc_64bit.tar.gz",
        "bytes": 28460,
        "sha256": "bb23428e10500357159d579ac8e8a877be73083405f759a94f5062141aabe5a0",
    },
}

HOLDOUT_PROFILE = "igs-tsukuba-2024-002-holdout"
HOLDOUT_SOURCES: dict[str, dict[str, Any]] = {
    **SOURCES,
    "rover_crx_gz": {
        "url": "https://igs.bkg.bund.de/root_ftp/IGS/obs/2024/002/TSK200JPN_R_20240020000_01D_30S_MO.crx.gz",
        "bytes": 2074945,
        "sha256": "b979a620052d449e55b3be629c6630d6833467fff55b11eb736927747fe8ad95",
    },
    "base_crx_gz": {
        "url": "https://igs.bkg.bund.de/root_ftp/IGS/obs/2024/002/TSKB00JPN_R_20240020000_01D_30S_MO.crx.gz",
        "bytes": 1473402,
        "sha256": "7cec122902d136ecd3e9a0514d2538a2bb9dd0811e653c7915322fe831ec86b8",
    },
    "nav_gz": {
        "url": "https://igs.bkg.bund.de/root_ftp/IGS/BRDC/2024/002/BRDC00IGS_R_20240020000_01D_MN.rnx.gz",
        "bytes": 1275347,
        "sha256": "5b3fb3aac5e43fcdeedcb6f476ca78c87e120e5bba413a2d63c4c599dde64502",
    },
    "sp3_gz": {
        "url": "https://igs.bkg.bund.de/root_ftp/IGS/products/2295/IGS0OPSFIN_20240020000_01D_15M_ORB.SP3.gz",
        "bytes": 103060,
        "sha256": "bc23bb524595cbaecd295dd03816a9caf852cdaeb15a79edd6899beccaceb32a",
    },
    "clk_gz": {
        "url": "https://igs.bkg.bund.de/root_ftp/IGS/products/2295/IGS0OPSFIN_20240020000_01D_30S_CLK.CLK.gz",
        "bytes": 2547535,
        "sha256": "ce249669b845fc7bd3ee9efa78547b15f4535000d333df9066ca8be8b6a4bd03",
    },
}

R7_DAY3_SOURCES: dict[str, dict[str, Any]] = {
    **SOURCES,
    "rover_crx_gz": {
        "url": "https://igs.bkg.bund.de/root_ftp/IGS/obs/2024/003/TSK200JPN_R_20240030000_01D_30S_MO.crx.gz",
        "bytes": 2096488,
        "sha256": "486da032007e088929cf0e30dd8d7de2a062f6033cddfa36159389811570d2b3",
    },
    "base_crx_gz": {
        "url": "https://igs.bkg.bund.de/root_ftp/IGS/obs/2024/003/TSKB00JPN_R_20240030000_01D_30S_MO.crx.gz",
        "bytes": 1460592,
        "sha256": "bab29258d2510e33174958a27d0f63426cf5b1d325cee512adce48c59d475821",
    },
    "nav_gz": {
        "url": "https://igs.bkg.bund.de/root_ftp/IGS/BRDC/2024/003/BRDC00IGS_R_20240030000_01D_MN.rnx.gz",
        "bytes": 1273188,
        "sha256": "144539c8e12ff51724c06a6d8aeeb04e8683510bf9a19fe8d65ce46ddd009f1c",
    },
    "sp3_gz": {
        "url": "https://igs.bkg.bund.de/root_ftp/IGS/products/2295/IGS0OPSFIN_20240030000_01D_15M_ORB.SP3.gz",
        "bytes": 103051,
        "sha256": "42d988af68fcce1eda621d3f89022644120a74ebf13914a3f7f1ad7f0526773b",
    },
    "clk_gz": {
        "url": "https://igs.bkg.bund.de/root_ftp/IGS/products/2295/IGS0OPSFIN_20240030000_01D_30S_CLK.CLK.gz",
        "bytes": 2549496,
        "sha256": "5c8295a152dd0f196ccbc08f26f8c11e6c071a066b61a87f43e9b2d8ae80acb8",
    },
}

R7_DAY4_SOURCES: dict[str, dict[str, Any]] = {
    **SOURCES,
    "rover_crx_gz": {
        "url": "https://igs.bkg.bund.de/root_ftp/IGS/obs/2024/004/TSK200JPN_R_20240040000_01D_30S_MO.crx.gz",
        "bytes": 2107072,
        "sha256": "4132e6b647c07e3e8437333364d345e7c156fff70a456797323e659e9446e62b",
    },
    "base_crx_gz": {
        "url": "https://igs.bkg.bund.de/root_ftp/IGS/obs/2024/004/TSKB00JPN_R_20240040000_01D_30S_MO.crx.gz",
        "bytes": 1478816,
        "sha256": "d0b6e306c8a1815dede253ba43b0aa918842279287ddd2210abadfef443fb906",
    },
    "nav_gz": {
        "url": "https://igs.bkg.bund.de/root_ftp/IGS/BRDC/2024/004/BRDC00IGS_R_20240040000_01D_MN.rnx.gz",
        "bytes": 1272904,
        "sha256": "f03b2c2da281dce479f4f46d65a5b15c8f889512b55bdf680aa395bf57fd1570",
    },
    "sp3_gz": {
        "url": "https://igs.bkg.bund.de/root_ftp/IGS/products/2295/IGS0OPSFIN_20240040000_01D_15M_ORB.SP3.gz",
        "bytes": 102794,
        "sha256": "798447aec712e7173e536bee83f02b8643ca7bdf797d59143615623e663d6624",
    },
    "clk_gz": {
        "url": "https://igs.bkg.bund.de/root_ftp/IGS/products/2295/IGS0OPSFIN_20240040000_01D_30S_CLK.CLK.gz",
        "bytes": 2545853,
        "sha256": "74bb50b415dc8caac8947e9fa875873da331b4863b88c848064de3b976fe6eed",
    },
}

DATASETS: dict[str, dict[str, Any]] = {
    "development": {
        "profile": PROFILE, "role": "development", "day": "001",
        "epoch": datetime(2024, 1, 1, tzinfo=timezone.utc), "sources": SOURCES,
    },
    "holdout": {
        "profile": HOLDOUT_PROFILE, "role": "sealed_holdout", "day": "002",
        "epoch": datetime(2024, 1, 2, tzinfo=timezone.utc), "sources": HOLDOUT_SOURCES,
    },
    "r7-development-day1": {
        "profile": "igs-tsukuba-r7-2024-001", "role": "r7_development", "day": "001",
        "epoch": datetime(2024, 1, 1, tzinfo=timezone.utc), "sources": SOURCES,
    },
    "r7-development-day2": {
        "profile": "igs-tsukuba-r7-2024-002", "role": "r7_development", "day": "002",
        "epoch": datetime(2024, 1, 2, tzinfo=timezone.utc), "sources": HOLDOUT_SOURCES,
    },
    "r7-development-day3": {
        "profile": "igs-tsukuba-r7-2024-003", "role": "r7_development", "day": "003",
        "epoch": datetime(2024, 1, 3, tzinfo=timezone.utc), "sources": R7_DAY3_SOURCES,
    },
    "r7-holdout-day4": {
        "profile": "igs-tsukuba-r7-2024-004-holdout", "role": "r7_sealed_holdout", "day": "004",
        "epoch": datetime(2024, 1, 4, tzinfo=timezone.utc), "sources": R7_DAY4_SOURCES,
    },
}


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    raw = list(sys.argv[1:] if argv is None else argv)
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME", "gnss japan-static-survey"))
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--cache-dir", type=Path, default=Path.home() / ".cache/libgnsspp/japan-static-survey")
    parser.add_argument("--mode", choices=("smoke", "full"), default="smoke")
    parser.add_argument(
        "--dataset", choices=tuple(DATASETS), default="development",
        help="Frozen R2 day or one of the separately role-labelled R7 monitoring days.",
    )
    parser.add_argument("--offline", action="store_true", help="Use only hash-verified cache entries.")
    parser.add_argument("--force-fetch", action="store_true")
    parser.add_argument("--materialize-only", action="store_true")
    parser.add_argument("--timeout-seconds", type=float, default=120.0)
    args = parser.parse_args(raw)
    setattr(args, "_raw_argv", raw)
    return args


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def record(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {"path": str(path), "exists": False, "bytes": None, "sha256": None}
    return {"path": str(path), "exists": True, "bytes": path.stat().st_size, "sha256": sha256_file(path)}


def software_record() -> dict[str, Any]:
    revision = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=ROOT_DIR, text=True,
        capture_output=True, check=False,
    )
    binaries = {}
    for name in ("gnss_solve", "gnss_ppp"):
        path = ROOT_DIR / "build" / "apps" / name
        if path.is_file():
            binaries[name] = record(path)
    return {
        "git_revision": revision.stdout.strip() if revision.returncode == 0 else "unknown",
        "binaries": binaries,
    }


def atomic_write(path: Path, content: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=path.parent, delete=False) as handle:
        handle.write(content)
        temporary = Path(handle.name)
    os.replace(temporary, path)


def source_cache_path(cache_dir: Path, source: dict[str, Any]) -> Path:
    return cache_dir / "downloads" / Path(str(source["url"])).name


def verify_source(path: Path, source: dict[str, Any]) -> None:
    if not path.is_file():
        raise ValueError(f"missing cached source: {path}")
    if path.stat().st_size != int(source["bytes"]):
        raise ValueError(f"source byte-size mismatch: {path}")
    actual = sha256_file(path)
    if actual != source["sha256"]:
        raise ValueError(f"source SHA-256 mismatch: {path}: {actual}")


def fetch_source(
    cache_dir: Path,
    source: dict[str, Any],
    *,
    offline: bool,
    force: bool,
    timeout: float,
) -> Path:
    path = source_cache_path(cache_dir, source)
    if path.exists() and not force:
        verify_source(path, source)
        return path
    if offline:
        raise ValueError(f"offline cache miss: {path}")
    error: Exception | None = None
    for attempt in range(3):
        try:
            with request.urlopen(str(source["url"]), timeout=timeout) as response:
                content = response.read()
            atomic_write(path, content)
            verify_source(path, source)
            return path
        except Exception as exc:  # network errors differ by Python/platform
            error = exc
            if attempt < 2:
                time.sleep(attempt + 1)
    raise ValueError(f"failed to fetch {source['url']}: {error}")


def materialized_paths(cache_dir: Path, profile: str, day: str) -> dict[str, Path]:
    root = cache_dir / "materialized" / profile
    return {
        "rover_obs": root / f"TSK200JPN_R_2024{day}0000_01D_30S_MO.rnx",
        "base_obs": root / f"TSKB00JPN_R_2024{day}0000_01D_30S_MO.rnx",
        "nav": root / f"BRDC00IGS_R_2024{day}0000_01D_MN.rnx",
        "sp3": root / f"IGS0OPSFIN_2024{day}0000_01D_15M_ORB.SP3",
        "clk": root / f"IGS0OPSFIN_2024{day}0000_01D_30S_CLK.CLK",
        "truth_snx": root / "IGS0OPSSNX_20233650000_07D_07D_SOL.SNX",
        "antex": root / "igs20.atx",
        "igs20_ssc": root / "IGS20.ssc",
        "rover_log": root / "tsk200jpn_20220912.log",
        "base_log": root / "tskb00jpn_20221018.log",
        "crx2rnx": cache_dir / "tools/RNXCMP_4.2.0/CRX2RNX",
    }


def materialize_gzip(source: Path, destination: Path) -> None:
    content = gzip.decompress(source.read_bytes())
    if not content:
        raise ValueError(f"empty gzip payload: {source}")
    atomic_write(destination, content)


def resolve_crx2rnx(downloads: dict[str, Path], paths: dict[str, Path]) -> Path:
    system = shutil.which("CRX2RNX") or shutil.which("crx2rnx")
    if system:
        return Path(system)
    machine = platform.machine().lower()
    if platform.system() != "Linux" or machine not in {"x86_64", "amd64"}:
        raise ValueError("CRX2RNX is required on this platform; install GSI RNXCMP 4.2.0")
    archive = downloads["rnxcmp_linux_x86_64"]
    destination = paths["crx2rnx"]
    with tarfile.open(archive, "r:gz") as bundle:
        member = bundle.getmember("RNXCMP_4.2.0_Linux_gcc_64bit/CRX2RNX")
        extracted = bundle.extractfile(member)
        if extracted is None:
            raise ValueError("RNXCMP archive lacks CRX2RNX")
        atomic_write(destination, extracted.read())
    destination.chmod(0o755)
    return destination


def convert_crx(source_gz: Path, destination: Path, converter: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    crx_path = destination.with_suffix(".crx")
    materialize_gzip(source_gz, crx_path)
    generated = crx_path.with_suffix(".rnx")
    # RNXCMP prompts on an existing output file.  A non-interactive replay
    # must deterministically regenerate this derivative from the verified
    # compact source instead of waiting forever for terminal input.
    if generated.exists():
        generated.unlink()
    completed = subprocess.run([str(converter), str(crx_path)], capture_output=True, text=True, check=False)
    try:
        if completed.returncode != 0 or not generated.is_file():
            raise ValueError(f"CRX2RNX failed ({completed.returncode}): {(completed.stdout + completed.stderr).strip()}")
        if generated != destination:
            os.replace(generated, destination)
    finally:
        try:
            crx_path.unlink()
        except OSError:
            pass


def rinex_antenna_type(path: Path) -> str:
    with path.open(encoding="ascii", errors="strict") as handle:
        for line in handle:
            if "ANT # / TYPE" in line:
                value = line[20:40].strip()
                if value:
                    return value
            if "END OF HEADER" in line:
                break
    raise ValueError(f"RINEX antenna type is missing: {path}")


def acquire(args: argparse.Namespace) -> tuple[dict[str, Path], dict[str, Any]]:
    cache_dir = args.cache_dir.resolve()
    dataset = DATASETS[args.dataset]
    sources = dataset["sources"]
    profile = str(dataset["profile"])
    observation_epoch = dataset["epoch"]
    day = str(dataset["day"])
    downloads: dict[str, Path] = {}
    for name, source in sources.items():
        if name == "rnxcmp_linux_x86_64" and (shutil.which("CRX2RNX") or shutil.which("crx2rnx")):
            continue
        downloads[name] = fetch_source(
            cache_dir, source, offline=args.offline, force=args.force_fetch,
            timeout=args.timeout_seconds,
        )
    paths = materialized_paths(cache_dir, profile, day)
    converter = resolve_crx2rnx(downloads, paths)
    convert_crx(downloads["rover_crx_gz"], paths["rover_obs"], converter)
    convert_crx(downloads["base_crx_gz"], paths["base_obs"], converter)
    for source_name, destination_name in (
        ("nav_gz", "nav"), ("sp3_gz", "sp3"), ("clk_gz", "clk"),
        ("truth_snx_gz", "truth_snx"),
    ):
        materialize_gzip(downloads[source_name], paths[destination_name])
    for source_name, destination_name in (
        ("antex", "antex"), ("igs20_ssc", "igs20_ssc"),
        ("rover_log", "rover_log"), ("base_log", "base_log"),
    ):
        atomic_write(paths[destination_name], downloads[source_name].read_bytes())
    truth = {
        station: read_station_estimate(paths["truth_snx"], station, observation_epoch)
        for station in ("TSK2", "TSKB")
    }
    rover_antenna = rinex_antenna_type(paths["rover_obs"])
    base_antenna = rinex_antenna_type(paths["base_obs"])
    metadata = {
        "profile": profile,
        "dataset_role": dataset["role"],
        "observation_epoch_utc": observation_epoch.isoformat(),
        "sources": {name: {**source, "cache": record(downloads[name]) if name in downloads else None} for name, source in sources.items()},
        "materialized": {name: record(path) for name, path in paths.items()},
        "truth": {
            "source_role": "independent weekly SINEX estimate; never the RINEX header",
            "frame": TRUTH_FRAME,
            "height_reference": "ellipsoidal ECEF-derived height; no orthometric transformation",
            "point_reference": "station solution point (ARP/marker relationship is not transformed here)",
            **{
                station: {
                    "solution_id": estimate.solution_id,
                    "validity_start": estimate.validity_start.isoformat(),
                    "validity_end": estimate.validity_end.isoformat() if estimate.validity_end else None,
                    "mean_epoch": estimate.mean_epoch.isoformat(),
                    "estimate_epoch": estimate.estimate_epoch.isoformat(),
                    "ecef_m": list(estimate.ecef_m),
                    "sigma_m": list(estimate.sigma_m),
                }
                for station, estimate in truth.items()
            },
        },
        "antennas": {"TSK2": rover_antenna, "TSKB": base_antenna},
    }
    return paths, metadata


def command(*parts: str | Path) -> list[str]:
    return [sys.executable, str(DISPATCHER), *[str(part) for part in parts]]


def child_commands(paths: dict[str, Path], metadata: dict[str, Any], output_dir: Path, mode: str) -> dict[str, list[str]]:
    epochs = 20 if mode == "smoke" else -1
    truth = metadata["truth"]["TSK2"]["ecef_m"]
    base_truth = metadata["truth"]["TSKB"]["ecef_m"]
    truth_text = ",".join(f"{value:.12f}" for value in truth)
    base_truth_text = ",".join(f"{value:.12f}" for value in base_truth)
    relative_pos = output_dir / "relative_static.pos"
    ppp_pos = output_dir / "ppp_static.pos"
    # Keep the negative X coordinate attached to the option; argparse otherwise
    # interprets the comma-separated ECEF token as another option.
    common_truth = [f"--truth-ecef={truth_text}", "--truth-source", TRUTH_SOURCE, "--truth-frame", TRUTH_FRAME]
    relative_gates: list[str | int | float] = [
        "--require-fix-rate-min", 70 if mode == "smoke" else 90,
        "--require-mean-sats-min", 8,
    ]
    ppp_gates: list[str | int | float] = [
        "--require-ppp-solution-rate-min", 90,
        "--require-mean-sats-min", 6,
    ]
    if mode == "full":
        relative_gates.extend(["--require-horizontal-error-max", 0.20, "--require-vertical-error-max", 0.40])
        ppp_gates.extend(["--require-horizontal-error-max", 2.0, "--require-vertical-error-max", 4.0])
    return {
        "relative": command(
            "short-baseline-signoff", "--rover", paths["rover_obs"], "--base", paths["base_obs"],
            "--nav", paths["nav"], "--out", relative_pos, "--summary-json", output_dir / "relative_summary.json",
            "--max-epochs", epochs, f"--base-ecef={base_truth_text}", *common_truth,
            *relative_gates,
        ),
        "ppp": command(
            "ppp-static-signoff", "--obs", paths["rover_obs"], "--nav", paths["nav"],
            "--sp3", paths["sp3"], "--clk", paths["clk"], "--antex", paths["antex"],
            "--receiver-antenna-type", metadata["antennas"]["TSK2"],
            "--out", ppp_pos, "--summary-json", output_dir / "ppp_summary.json",
            "--max-epochs", epochs, *common_truth,
            *ppp_gates,
        ),
        "relative_kml": command("pos2kml", relative_pos, output_dir / "relative_static.kml", "--status", "all"),
        "relative_png": command("trackplot", relative_pos),
        "ppp_kml": command("pos2kml", ppp_pos, output_dir / "ppp_static.kml", "--status", "all"),
        "ppp_png": command("trackplot", ppp_pos),
    }


def run_step(name: str, argv: list[str], log_path: Path) -> dict[str, Any]:
    with log_path.open("a", encoding="utf-8") as log:
        log.write("\n$ " + shlex.join(argv) + "\n")
        log.flush()
        completed = subprocess.run(argv, cwd=ROOT_DIR, stdout=log, stderr=subprocess.STDOUT, check=False)
    return {"name": name, "argv": argv, "exit_status": completed.returncode, "status": "passed" if completed.returncode == 0 else "failed"}


def artifact_paths(output_dir: Path) -> dict[str, Path]:
    return {
        "relative_pos": output_dir / "relative_static.pos", "relative_kml": output_dir / "relative_static.kml",
        "relative_png": output_dir / "relative_static_trajectory.png", "relative_summary": output_dir / "relative_summary.json",
        "ppp_pos": output_dir / "ppp_static.pos", "ppp_kml": output_dir / "ppp_static.kml",
        "ppp_png": output_dir / "ppp_static_trajectory.png", "ppp_summary": output_dir / "ppp_summary.json",
        "log": output_dir / "bundle.log",
    }


def persist(path: Path, payload: dict[str, Any], output_dir: Path) -> None:
    payload["artifacts"] = {name: record(value) for name, value in artifact_paths(output_dir).items()}
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def run(args: argparse.Namespace) -> int:
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    manifest_path = output_dir / "manifest.json"
    log_path = output_dir / "bundle.log"
    log_path.write_text("libgnss++ japan-static-survey\n", encoding="utf-8")
    manifest: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION, "status": "running", "exit_status": None,
        "profile": DATASETS[args.dataset]["profile"],
        "dataset_role": DATASETS[args.dataset]["role"],
        "mode": args.mode,
        "run": {"argv": [str(DISPATCHER), "japan-static-survey", *getattr(args, "_raw_argv", [])], "cwd": str(Path.cwd().resolve()), **software_record()},
        "provenance": {
            "network": "International GNSS Service (IGS)", "mirrors": ["BKG", "SOPAC", "IGS Central Bureau"],
            "data_policy": "openly available without restriction; best-effort; acknowledge IGS",
            "policy_url": IGS_POLICY_URL,
        },
        "frames": {
            "coordinate": TRUTH_FRAME, "time": "observation UTC; solution epochs GPST",
            "height": "ellipsoidal only; no orthometric-height conversion",
            "position_target": "IGS station monument/marker; not an arbitrary ground control point",
            "antenna": "PPP applies igs20.atx PCO/PCV; relative RTK currently lacks ANTEX and is demonstration-only",
        },
        "steps": [],
    }
    persist(manifest_path, manifest, output_dir)
    try:
        paths, metadata = acquire(args)
        manifest["acquisition"] = metadata
    except Exception as exc:
        manifest["status"] = "failed"
        manifest["exit_status"] = 1
        manifest["failure_reason"] = f"acquisition:{exc}"
        persist(manifest_path, manifest, output_dir)
        print(f"Japan static survey acquisition failed: {exc}", file=sys.stderr)
        return 1
    if args.materialize_only:
        manifest["status"] = "materialized"
        manifest["exit_status"] = 0
        persist(manifest_path, manifest, output_dir)
        return 0
    commands = child_commands(paths, metadata, output_dir, args.mode)
    manifest["commands"] = commands
    for name in ("relative", "ppp"):
        step = run_step(name, commands[name], log_path)
        manifest["steps"].append(step)
        persist(manifest_path, manifest, output_dir)
        prefix = name
        if step["status"] == "passed":
            for suffix in ("kml", "png"):
                convert = run_step(f"{prefix}_{suffix}", commands[f"{prefix}_{suffix}"], log_path)
                manifest["steps"].append(convert)
                persist(manifest_path, manifest, output_dir)
    summaries: dict[str, Any] = {}
    for name in ("relative", "ppp"):
        path = output_dir / f"{name}_summary.json"
        if path.is_file():
            summaries[name] = json.loads(path.read_text(encoding="utf-8"))
    manifest["lanes"] = summaries
    manifest["inputs"] = manifest["acquisition"]["sources"]
    manifest["populations"] = {
        name: {
            "all_epochs": lane.get("epochs"),
            "accepted_epochs": lane.get("fixed_epochs", lane.get("ppp_float_epochs", 0) + lane.get("ppp_fixed_epochs", 0)),
            "solution_states": {
                "fixed": lane.get("fixed_epochs", lane.get("ppp_fixed_epochs", 0)),
                "float": lane.get("ppp_float_epochs"),
                "fallback": lane.get("fallback_epochs"),
            },
        }
        for name, lane in summaries.items()
    }
    manifest["metrics"] = {
        name: {
            "horizontal_error_m": lane.get("static_truth_metrics", {}).get("horizontal_error_m"),
            "vertical_error_m": lane.get("static_truth_metrics", {}).get("vertical_error_m"),
            "solution_rate_pct": lane.get("fix_rate_pct", lane.get("ppp_solution_rate_pct")),
            "convergence_time_s": lane.get("ppp_convergence_time_s"),
        }
        for name, lane in summaries.items()
    }
    all_steps_passed = len(manifest["steps"]) == 6 and all(step["status"] == "passed" for step in manifest["steps"])
    all_artifacts = all(path.is_file() and path.stat().st_size > 0 for path in artifact_paths(output_dir).values())
    manifest["gate"] = {
        "status": "passed" if all_steps_passed and all_artifacts else "failed",
        "all_steps_passed": all_steps_passed, "all_artifacts_nonempty": all_artifacts,
        "relative_application_decision": "demonstration_only_no_rtk_antex",
        "ppp_application_decision": "control_point_candidate_requires_local_survey_authority_review" if all_steps_passed else "unusable",
    }
    manifest["status"] = manifest["gate"]["status"]
    manifest["exit_status"] = 0 if manifest["status"] == "passed" else 1
    persist(manifest_path, manifest, output_dir)
    print(f"Japan static survey bundle: {manifest['status']}")
    print(f"Manifest: {manifest_path}")
    return int(manifest["exit_status"])


def main(argv: list[str] | None = None) -> int:
    return run(parse_args(argv))


if __name__ == "__main__":
    raise SystemExit(main())
