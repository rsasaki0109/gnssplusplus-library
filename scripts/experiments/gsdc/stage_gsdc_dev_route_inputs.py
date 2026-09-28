#!/usr/bin/env python3
"""Stage the GSDC 2023 Pixel5 dev-route inputs for the base-surveyed table.

The README "Smartphone GNSS/IMU" base-surveyed table (routes H, U, A, LAX-T;
record ``docs/use_cases/records/smartphone_base_surveyed_route_results_v1.md``)
was measured on raw inputs that the research harnesses (phase37 / phase25 /
phase63, removed in PR #510) extracted byte-for-byte from the taroz
``dataset_2023`` archive of the Google Smartphone Decimeter Challenge 2023:

* ``train/<drive>/pixel5/device_gnss.csv`` and ``device_imu.csv``
* ``train/<drive>/brdc.nav`` (route broadcast navigation)
* ``train/<drive>/<BASE>_rnx2.obs`` (CORS base RINEX 2 observations)
* ``train/<drive>/pixel5/ground_truth.csv`` (scoring only)

No file is rewritten.  This script copies (or hard-links) each member into
``<out>/<route>/`` under a fixed name and verifies it against the SHA-256 the
research records pinned, so a mismatching dataset copy fails before any
solver runs.  It also cross-checks the surveyed base ECEF coordinates the lane
passes to ``--native-base-position-ecef`` against ``base/base_position.csv``
when that file is present.

``--gsdc-root`` accepts the extracted ``dataset_2023`` directory (or its
parent) or the ``dataset_2023.zip`` archive itself.  Ground truth is looked up
in the GSDC root first and then under ``--truth-root``, which may use the
nested ``train/<drive>/pixel5/ground_truth.csv`` layout or flat
``<drive>__pixel5__ground_truth.csv`` files.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import os
from pathlib import Path
import shutil
import sys
from typing import Any, Iterable
import zipfile


PHONE = "pixel5"

# Surveyed base station coordinates (ECEF, m).  Source: the year-matched
# station table ``base/base_position.csv`` of the taroz gsdc2023 dataset
# (https://github.com/taroz/gsdc2023), the same values recorded in
# docs/use_cases/records/smartphone_base_surveyed_route_results_v1.md.
ROUTES: dict[str, dict[str, Any]] = {
    "H": {
        "drive": "2021-08-24-20-32-us-ca-mtv-h",
        "base": "P221",
        "base_year": 2021,
        "base_ecef": (-2698117.9416, -4301326.2649, 3847286.2750),
        "sha256": {
            "device_gnss.csv": "46482b82db0992c1f063dbd9cf697268605234d3e38bcbd23525fd4b60bc17a7",
            "device_imu.csv": "fa3f17d07570fdbb8030f307130f33ca3613e8125475c2a907c48f7db3455480",
            "brdc.nav": "147d948f0eba3bf09e295e7f67fbe8db60c25e236bc3c7d958dc933483f10909",
            "base.obs": "4d3e37cbe0347fa56216db54ede9e0f30731885f337f1653ab5a86afb2bb2150",
            "ground_truth.csv": "a55f452e611426693677fbeacde227fc80e5f03f6040d04aef9d6a2baf08d249",
        },
    },
    "U": {
        "drive": "2023-03-08-21-34-us-ca-mtv-u",
        "base": "P221",
        "base_year": 2023,
        "base_ecef": (-2698117.9861, -4301326.2071, 3847286.2977),
        "sha256": {
            "device_gnss.csv": "a0fc8e71bdfc03be61b99efcd7d41fbba8ffec126df78b55243f681fd211f204",
            "device_imu.csv": "c7d726e1cc0dacc7a569bd8be9bc2333765bbb3b3010203a4aea2e49778101ab",
            "brdc.nav": "8893ef62fecdd9986f6b2cf1b7b980defa6430da5801aafcab4ecf4c99a03b92",
            "base.obs": "aedb7a39e7b6612ea97964b363c74cd2c10318255b7f2287f4720d18e71803e6",
            "ground_truth.csv": "7f27caff1f87f4e43821b8efdbbbc87b75b95c0820291cc906a21ea5aee4f080",
        },
    },
    "A": {
        "drive": "2021-03-16-18-59-us-ca-mtv-a",
        "base": "SLAC",
        "base_year": 2021,
        "base_ecef": (-2703116.3177, -4291766.7551, 3854248.0736),
        "sha256": {
            "device_gnss.csv": "c7d50d5127d16586adc6c79d724758e298b385496da22c5e5dfd6ec522cbc863",
            "device_imu.csv": "afc540e7c4ce2ca66b442a1afbcd604e9f6b3d2cc4d3733183739901b5b97bd6",
            "brdc.nav": "6adfaf7fe4452a4faeb94a7b607c15e05f578c46a028a030b94aa6f79de194cd",
            "base.obs": "380b8ff9091344fb756697e27f0983d9a0ba2cf0c201b96849bfc1ecc1af0e52",
            "ground_truth.csv": "7c84ed6a80b1bbb08c0ffad57493513833b9d5474e22a43c5a44da82824ee22d",
        },
    },
    "LAX-T": {
        "drive": "2022-04-01-18-22-us-ca-lax-t",
        "base": "LBCH",
        "base_year": 2022,
        "base_ecef": (-2507799.2243, -4676369.3031, 3526891.0358),
        "sha256": {
            "device_gnss.csv": "50362c01bff3e0bb7088e54021164591cd750227ed97c2fd7d95d763a08798f1",
            "device_imu.csv": "2e39a3e9f294c64b8ecfd452d0960025d1013b97f2d7497e6e48a2a1997b38c5",
            "brdc.nav": "443d3d5a73f4895b83e576e24a568f4658f869e79a480856de7f0717763dcbe6",
            "base.obs": "d731e0e8a7ba4396d62340c85b6238e66c50c6a349621f9dcea0ea6885fd4cfe",
            "ground_truth.csv": "29e0861dd1ecb8865c10adab69396d98ed96618e8877b09d04aa8d671edf79e8",
        },
    },
}


class StageError(Exception):
    """Missing or mismatching input."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def member_names(route: dict[str, Any]) -> dict[str, str]:
    """Staged filename -> path relative to ``dataset_2023/``."""
    drive = route["drive"]
    return {
        "device_gnss.csv": f"train/{drive}/{PHONE}/device_gnss.csv",
        "device_imu.csv": f"train/{drive}/{PHONE}/device_imu.csv",
        "brdc.nav": f"train/{drive}/brdc.nav",
        "base.obs": f"train/{drive}/{route['base']}_rnx2.obs",
        "ground_truth.csv": f"train/{drive}/{PHONE}/ground_truth.csv",
    }


def truth_candidates(truth_root: Path, drive: str) -> list[Path]:
    return [
        truth_root / "train" / drive / PHONE / "ground_truth.csv",
        truth_root / "dataset_2023" / "train" / drive / PHONE / "ground_truth.csv",
        truth_root / drive / PHONE / "ground_truth.csv",
        truth_root / f"{drive}__{PHONE}__ground_truth.csv",
    ]


def copy_file(source: Path, destination: Path, link: bool) -> None:
    """Hard-link (same volume) or copy ``source`` to ``destination`` atomically."""
    tmp = destination.with_name(destination.name + ".part")
    if tmp.exists():
        tmp.unlink()
    linked = False
    if link:
        try:
            os.link(source, tmp)
            linked = True
        except OSError:
            linked = False
    if not linked:
        shutil.copyfile(source, tmp)
    os.replace(tmp, destination)


class Source:
    """Read dataset_2023 members from a directory tree or the zip archive."""

    def __init__(self, root: Path) -> None:
        self.zip: zipfile.ZipFile | None = None
        self.prefix = ""
        self.root: Path | None = None
        if root.is_file() and zipfile.is_zipfile(root):
            self.zip = zipfile.ZipFile(root)
            names = set(self.zip.namelist())
            self.prefix = "dataset_2023/" if any(n.startswith("dataset_2023/") for n in names) else ""
        elif (root / "train").is_dir():
            self.root = root
        elif (root / "dataset_2023" / "train").is_dir():
            self.root = root / "dataset_2023"
        else:
            raise StageError(
                f"{root}: expected the dataset_2023 directory (with train/), its parent, or dataset_2023.zip"
            )

    def describe(self) -> str:
        if self.zip is not None:
            return f"zip:{self.zip.filename}"
        return str(self.root)

    def locate(self, relative: str) -> Path | str | None:
        if self.zip is not None:
            name = self.prefix + relative
            try:
                self.zip.getinfo(name)
            except KeyError:
                return None
            return name
        assert self.root is not None
        path = self.root / relative
        return path if path.is_file() else None

    def materialize(self, located: Path | str, destination: Path, link: bool) -> None:
        if isinstance(located, Path):
            copy_file(located, destination, link)
            return
        assert self.zip is not None
        tmp = destination.with_name(destination.name + ".part")
        with self.zip.open(located) as src, tmp.open("wb") as dst:
            shutil.copyfileobj(src, dst, 1 << 20)
        os.replace(tmp, destination)

    def read_bytes(self, relative: str) -> bytes | None:
        located = self.locate(relative)
        if located is None:
            return None
        if isinstance(located, str):
            assert self.zip is not None
            return self.zip.read(located)
        return Path(located).read_bytes()


def check_base_positions(source: Source, routes: Iterable[str]) -> dict[str, Any]:
    """Cross-check the pinned base ECEF against base/base_position.csv if present."""
    payload = source.read_bytes("base/base_position.csv")
    if payload is None:
        return {"checked": False, "reason": "base/base_position.csv not found in the GSDC root"}
    table: dict[tuple[str, int], tuple[float, float, float]] = {}
    for row in csv.DictReader(io.StringIO(payload.decode("utf-8"))):
        table[(row["Base"].strip(), int(row["Year"]))] = (float(row["X"]), float(row["Y"]), float(row["Z"]))
    report: dict[str, Any] = {"checked": True, "routes": {}}
    for name in routes:
        route = ROUTES[name]
        key = (route["base"], route["base_year"])
        if key not in table:
            raise StageError(f"{name}: {key[0]} {key[1]} missing from base/base_position.csv")
        delta = max(abs(a - b) for a, b in zip(table[key], route["base_ecef"]))
        if delta > 1e-4:
            raise StageError(
                f"{name}: base_position.csv {key} = {table[key]} differs from the pinned {route['base_ecef']}"
            )
        report["routes"][name] = {"base": key[0], "year": key[1], "ecef": list(table[key])}
    return report


def stage_route(
    name: str,
    source: Source,
    truth_root: Path | None,
    out: Path,
    *,
    link: bool,
    verify: bool,
) -> dict[str, Any]:
    route = ROUTES[name]
    route_dir = out / name
    route_dir.mkdir(parents=True, exist_ok=True)
    files: dict[str, Any] = {}
    for staged, relative in member_names(route).items():
        expected = route["sha256"][staged]
        destination = route_dir / staged
        origin: str | None = None
        if destination.is_file() and sha256_file(destination) == expected:
            files[staged] = {"sha256": expected, "bytes": destination.stat().st_size, "source": "already staged"}
            continue
        located = source.locate(relative)
        if located is None and staged == "ground_truth.csv" and truth_root is not None:
            located = next((path for path in truth_candidates(truth_root, route["drive"]) if path.is_file()), None)
            if located is not None:
                copy_file(located, destination, link)
                origin = str(located)
        if origin is None:
            if located is None:
                hint = " (pass --truth-root)" if staged == "ground_truth.csv" else ""
                raise StageError(f"{name}: {relative} not found in {source.describe()}{hint}")
            source.materialize(located, destination, link)
            origin = f"{source.describe()}/{relative}" if isinstance(located, str) else str(located)
        actual = sha256_file(destination)
        if verify and actual != expected:
            destination.unlink()
            raise StageError(f"{name}: {staged} from {origin} has SHA-256 {actual}, expected {expected}")
        files[staged] = {"sha256": actual, "bytes": destination.stat().st_size, "source": origin}
    return {
        "route": name,
        "dataset_id": f"{route['drive']}/{PHONE}",
        "drive": route["drive"],
        "phone": PHONE,
        "base": route["base"],
        "base_year": route["base_year"],
        "base_ecef": list(route["base_ecef"]),
        "dir": str(route_dir),
        "files": files,
    }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--gsdc-root", type=Path, required=True,
                        help="dataset_2023 directory (holding train/), its parent, or dataset_2023.zip.")
    parser.add_argument("--truth-root", type=Path, default=None,
                        help="Fallback location of ground_truth.csv (nested train/<drive>/pixel5/ or flat "
                             "<drive>__pixel5__ground_truth.csv).")
    parser.add_argument("--out", type=Path, required=True, help="Output directory; one sub-directory per route.")
    parser.add_argument("--routes", nargs="+", default=list(ROUTES), choices=list(ROUTES))
    parser.add_argument("--copy", action="store_true", help="Always copy instead of trying a hard link first.")
    parser.add_argument("--no-verify", action="store_true",
                        help="Stage even if a SHA-256 differs from the pin (the manifest is still written).")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        source = Source(args.gsdc_root)
        truth_root = args.truth_root
        base_check = check_base_positions(source, args.routes)
        routes = [
            stage_route(name, source, truth_root, args.out, link=not args.copy, verify=not args.no_verify)
            for name in args.routes
        ]
    except StageError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    manifest = {
        "schema": "gsdc_dev_route_inputs.v1",
        "gsdc_root": source.describe(),
        "truth_root": str(truth_root) if truth_root else None,
        "base_position_check": base_check,
        "routes": routes,
    }
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "staging.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    for route in routes:
        print(f"{route['route']:<6} {route['dataset_id']}  base {route['base']} {route['base_year']}  -> {route['dir']}")
    print(f"wrote {args.out / 'staging.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
