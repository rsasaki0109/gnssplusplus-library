#!/usr/bin/env python3
"""Fetch and cache GNSS product files such as SP3/CLK/IONEX/DCB."""

from __future__ import annotations

import argparse
import datetime as dt
import gzip
import json
import os
from pathlib import Path
import sys
import tempfile
from urllib import parse, request
from urllib import error as urlerror


GPS_EPOCH = dt.date(1980, 1, 6)
DEFAULT_TIMEOUT_SECONDS = 30.0
PRESET_DEFINITIONS = {
    "igs-final": [
        (
            "sp3",
            "https://cddis.nasa.gov/archive/gnss/products/{gps_week}/"
            "COD0OPSFIN_{yyyy}{doy}0000_01D_05M_ORB.SP3.gz",
        ),
        (
            "clk",
            "https://cddis.nasa.gov/archive/gnss/products/{gps_week}/"
            "COD0OPSFIN_{yyyy}{doy}0000_01D_30S_CLK.CLK.gz",
        ),
    ],
    "ionex": [
        (
            "ionex",
            "https://cddis.nasa.gov/archive/gnss/products/ionex/{yyyy}/{doy}/"
            "COD0OPSFIN_{yyyy}{doy}0000_01D_01H_GIM.INX.gz",
        ),
    ],
    "dcb": [
        (
            "dcb",
            "https://cddis.nasa.gov/archive/gnss/products/bias/{yyyy}/"
            "CAS0MGXRAP_{yyyy}{doy}0000_01D_01D_DCB.BSX.gz",
        ),
    ],
    "brdc-nav": [
        (
            "nav",
            "https://igs.bkg.bund.de/root_ftp/IGS/BRDC/{yyyy}/{doy}/"
            "BRDC00IGS_R_{yyyy}{doy}0000_01D_MN.rnx.gz",
        ),
    ],
}


def default_cache_dir() -> Path:
    xdg_cache_home = os.environ.get("XDG_CACHE_HOME")
    if xdg_cache_home:
        return Path(xdg_cache_home) / "libgnsspp" / "products"
    if os.name == "nt":
        local_app_data = os.environ.get("LOCALAPPDATA")
        if local_app_data:
            return Path(local_app_data) / "libgnsspp" / "products"
    return Path.home() / ".cache" / "libgnsspp" / "products"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument(
        "--date",
        default=dt.date.today().isoformat(),
        help="UTC calendar date used to resolve product templates (YYYY-MM-DD). Default: today.",
    )
    parser.add_argument(
        "--preset",
        action="append",
        default=[],
        choices=sorted(PRESET_DEFINITIONS.keys()),
        help="Expand a built-in product preset. May be repeated.",
    )
    parser.add_argument(
        "--product",
        action="append",
        default=[],
        metavar="KIND=SOURCE",
        help=(
            "Product spec such as sp3=https://host/{yyyy}{doy}.sp3.gz or "
            "ionex=file:///data/{yy}{doy}.ionex.gz. May be repeated."
        ),
    )
    parser.add_argument(
        "--list-presets",
        action="store_true",
        help="Print available preset names and exit.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Resolve preset/template URLs and cache paths without copying or downloading files.",
    )
    parser.add_argument(
        "--cache-dir",
        type=Path,
        default=default_cache_dir(),
        help="Cache directory for downloaded or copied products.",
    )
    parser.add_argument(
        "--summary-json",
        type=Path,
        default=None,
        help="Optional JSON summary output path.",
    )
    parser.add_argument(
        "--timeout-seconds",
        type=float,
        default=DEFAULT_TIMEOUT_SECONDS,
        help=f"Network timeout in seconds. Default: {DEFAULT_TIMEOUT_SECONDS:g}.",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Re-fetch products even if the cache entry already exists.",
    )
    return parser.parse_args()


def parse_date(date_text: str) -> dt.date:
    try:
        return dt.date.fromisoformat(date_text)
    except ValueError as exc:
        raise SystemExit(f"Invalid --date `{date_text}`. Expected YYYY-MM-DD.") from exc


def gps_week_and_dow(date_value: dt.date) -> tuple[int, int]:
    delta_days = (date_value - GPS_EPOCH).days
    if delta_days < 0:
        raise SystemExit("Dates earlier than GPS epoch are not supported.")
    return delta_days // 7, delta_days % 7


def template_values(date_value: dt.date) -> dict[str, str]:
    gps_week, gps_dow = gps_week_and_dow(date_value)
    return {
        "yyyy": f"{date_value.year:04d}",
        "yy": f"{date_value.year % 100:02d}",
        "mm": f"{date_value.month:02d}",
        "dd": f"{date_value.day:02d}",
        "doy": f"{date_value.timetuple().tm_yday:03d}",
        "gps_week": str(gps_week),
        "gps_dow": str(gps_dow),
    }


def parse_product_specs(specs: list[str], values: dict[str, str]) -> list[tuple[str, str, str]]:
    parsed_specs: list[tuple[str, str, str]] = []
    if not specs:
        raise SystemExit("Specify at least one --product KIND=SOURCE template.")
    for spec in specs:
        if "=" not in spec:
            raise SystemExit(f"Invalid --product `{spec}`. Expected KIND=SOURCE.")
        kind, source_template = spec.split("=", 1)
        normalized_kind = kind.strip().lower()
        if not normalized_kind:
            raise SystemExit(f"Invalid --product `{spec}`. Empty product kind.")
        try:
            resolved_source = source_template.format(**values)
        except KeyError as exc:
            raise SystemExit(f"Unknown template key {exc!s} in --product `{spec}`.") from exc
        parsed_specs.append((normalized_kind, source_template, resolved_source))
    return parsed_specs


def expand_presets(presets: list[str]) -> list[str]:
    expanded: list[str] = []
    for preset in presets:
        for kind, source in PRESET_DEFINITIONS[preset]:
            expanded.append(f"{kind}={source}")
    return expanded


def strip_compression_suffix(name: str) -> str:
    return name[:-3] if name.endswith(".gz") else name


def source_basename(source: str, kind: str, values: dict[str, str]) -> str:
    if len(source) >= 2 and source[1] == ":" and source[0].isalpha():
        base_name = Path(source).name
        if base_name:
            return strip_compression_suffix(base_name)
    parsed = parse.urlparse(source)
    if parsed.scheme:
        base_name = Path(parse.unquote(parsed.path)).name
    else:
        base_name = Path(source).expanduser().name
    if base_name:
        return strip_compression_suffix(base_name)
    fallback = f"{kind}_{values['yyyy']}{values['doy']}"
    return fallback


def resolve_local_source(source: str) -> Path | None:
    if len(source) >= 2 and source[1] == ":" and source[0].isalpha():
        return Path(source).expanduser()
    parsed = parse.urlparse(source)
    if parsed.scheme == "file":
        return Path(parse.unquote(parsed.path)).expanduser()
    if parsed.scheme:
        return None
    return Path(source).expanduser()


def read_source_bytes(source: str, timeout_seconds: float) -> tuple[bytes, str]:
    local_path = resolve_local_source(source)
    if local_path is not None:
        if not local_path.exists():
            raise SystemExit(f"Local product source does not exist: {local_path}")
        return local_path.read_bytes(), "copied"
    try:
        with request.urlopen(source, timeout=timeout_seconds) as response:
            return response.read(), "downloaded"
    except urlerror.URLError as exc:
        raise SystemExit(f"Failed to fetch product source `{source}`: {exc}") from exc


def maybe_decompress(raw_bytes: bytes, source: str) -> tuple[bytes, bool]:
    if source.lower().endswith(".gz"):
        try:
            return gzip.decompress(raw_bytes), True
        except gzip.BadGzipFile as exc:
            prefix = raw_bytes[:80].decode("ascii", errors="replace")
            raise SystemExit(
                f"Expected gzip-compressed content from `{source}`, but decompression failed. "
                f"First bytes: {prefix!r}"
            ) from exc
    return raw_bytes, False


def write_atomic(path: Path, content: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(dir=path.parent, delete=False) as temp_file:
        temp_file.write(content)
        temp_name = temp_file.name
    os.replace(temp_name, path)


def fetch_one_product(
    kind: str,
    source_template: str,
    resolved_source: str,
    date_value: dt.date,
    values: dict[str, str],
    cache_dir: Path,
    timeout_seconds: float,
    force: bool,
) -> dict[str, object]:
    relative_dir = Path(kind) / values["yyyy"] / values["doy"]
    destination_name = source_basename(resolved_source, kind, values)
    destination = cache_dir / relative_dir / destination_name
    if destination.exists() and not force:
        return {
            "kind": kind,
            "source_template": source_template,
            "resolved_source": resolved_source,
            "path": str(destination),
            "status": "cached",
            "compressed_source": resolved_source.lower().endswith(".gz"),
            "date": date_value.isoformat(),
        }

    raw_bytes, transfer_status = read_source_bytes(resolved_source, timeout_seconds)
    file_bytes, was_compressed = maybe_decompress(raw_bytes, resolved_source)
    write_atomic(destination, file_bytes)
    return {
        "kind": kind,
        "source_template": source_template,
        "resolved_source": resolved_source,
        "path": str(destination),
        "status": transfer_status,
        "compressed_source": was_compressed,
        "bytes": len(file_bytes),
        "date": date_value.isoformat(),
    }


def dry_run_product(
    kind: str,
    source_template: str,
    resolved_source: str,
    date_value: dt.date,
    values: dict[str, str],
    cache_dir: Path,
) -> dict[str, object]:
    relative_dir = Path(kind) / values["yyyy"] / values["doy"]
    destination_name = source_basename(resolved_source, kind, values)
    destination = cache_dir / relative_dir / destination_name
    return {
        "kind": kind,
        "source_template": source_template,
        "resolved_source": resolved_source,
        "path": str(destination),
        "status": "dry-run",
        "compressed_source": resolved_source.lower().endswith(".gz"),
        "date": date_value.isoformat(),
    }


def build_summary(results: list[dict[str, object]], cache_dir: Path, date_value: dt.date) -> dict[str, object]:
    products = {str(result["kind"]): str(result["path"]) for result in results}
    status_counts: dict[str, int] = {}
    for result in results:
        status = str(result["status"])
        status_counts[status] = status_counts.get(status, 0) + 1
    return {
        "date": date_value.isoformat(),
        "cache_dir": str(cache_dir),
        "products": products,
        "results": results,
        "status_counts": status_counts,
    }


def main() -> int:
    args = parse_args()
    if args.list_presets:
        for preset_name, entries in PRESET_DEFINITIONS.items():
            sys.stdout.write(f"{preset_name}\n")
            for kind, source in entries:
                sys.stdout.write(f"  {kind}={source}\n")
        return 0
    date_value = parse_date(args.date)
    values = template_values(date_value)
    combined_specs = [*expand_presets(args.preset), *args.product]
    parsed_specs = parse_product_specs(combined_specs, values)
    cache_dir = args.cache_dir.expanduser().resolve()
    results = []
    for kind, source_template, resolved_source in parsed_specs:
        if args.dry_run:
            results.append(
                dry_run_product(
                    kind,
                    source_template,
                    resolved_source,
                    date_value,
                    values,
                    cache_dir,
                )
            )
        else:
            results.append(
                fetch_one_product(
                    kind,
                    source_template,
                    resolved_source,
                    date_value,
                    values,
                    cache_dir,
                    args.timeout_seconds,
                    args.force,
                )
            )
    summary = build_summary(results, cache_dir, date_value)
    summary["presets"] = args.preset
    summary["dry_run"] = bool(args.dry_run)
    summary_text = json.dumps(summary, indent=2, sort_keys=True) + "\n"
    if args.summary_json is not None:
        args.summary_json.parent.mkdir(parents=True, exist_ok=True)
        args.summary_json.write_text(summary_text, encoding="utf-8")
    sys.stdout.write(summary_text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
