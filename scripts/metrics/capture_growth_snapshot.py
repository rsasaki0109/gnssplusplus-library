#!/usr/bin/env python3
"""Capture a reproducible, authenticated GitHub growth snapshot.

The repository and traffic endpoints are deliberately queried through the
GitHub REST API instead of the ``gh`` CLI so that this tool has no shelling
out, no implicit credential lookup, and no token-bearing diagnostic output.
Traffic data requires push/admin visibility and is intended to remain in the
ignored ``output/growth/`` directory.
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import os
import re
import sys
import tempfile
import urllib.error
import urllib.parse
import urllib.request
from pathlib import Path
from typing import Any, Mapping, Sequence


API_ROOT = "https://api.github.com"
DEFAULT_REPO = "rsasaki0109/gnssplusplus-library"
SCHEMA_VERSION = "growth_snapshot.v1"
TARGET_STARS = 200
API_TIMEOUT_SECONDS = 20.0
API_HEADERS = {
    "Accept": "application/vnd.github+json",
    "User-Agent": "gnssplusplus-growth-snapshot/1.0",
    "X-GitHub-Api-Version": "2022-11-28",
}


class SnapshotError(RuntimeError):
    """A safe, user-facing snapshot failure."""


def validate_repo(repo: str) -> str:
    """Validate and return an ``owner/name`` repository identifier."""

    if not re.fullmatch(r"[A-Za-z0-9_.-]+/[A-Za-z0-9_.-]+", repo):
        raise SnapshotError(
            f"invalid repository {repo!r}; expected an owner/name identifier"
        )
    return repo


def _git_config_candidates(root: Path) -> list[Path]:
    git_path = root / ".git"
    candidates: list[Path] = []
    if git_path.is_dir():
        candidates.append(git_path / "config")
    elif git_path.is_file():
        try:
            gitdir_line = next(
                line for line in git_path.read_text(encoding="utf-8").splitlines()
                if line.startswith("gitdir: ")
            )
        except (StopIteration, OSError, UnicodeDecodeError):
            return candidates
        gitdir = Path(gitdir_line[len("gitdir: ") :])
        if not gitdir.is_absolute():
            gitdir = (root / gitdir).resolve()
        candidates.append(gitdir / "config")
        common_dir_file = gitdir / "commondir"
        try:
            common_dir = Path(common_dir_file.read_text(encoding="utf-8").strip())
            if not common_dir.is_absolute():
                common_dir = (gitdir / common_dir).resolve()
            candidates.append(common_dir / "config")
        except (OSError, UnicodeDecodeError):
            pass
    return candidates


def _repo_from_remote_url(url: str) -> str | None:
    value = url.strip()
    match = re.search(
        r"github\.com[:/]([^/\s]+)/([^/\s]+?)(?:\.git)?/?$", value
    )
    if not match:
        return None
    return f"{match.group(1)}/{match.group(2)}"


def infer_default_repo() -> str:
    """Infer this checkout's GitHub repository without invoking Git."""

    for env_name in ("GITHUB_REPOSITORY",):
        value = os.environ.get(env_name)
        if value:
            try:
                return validate_repo(value)
            except SnapshotError:
                pass

    root = Path(__file__).resolve().parents[2]
    for config_path in _git_config_candidates(root):
        try:
            lines = config_path.read_text(encoding="utf-8").splitlines()
        except (OSError, UnicodeDecodeError):
            continue
        in_origin = False
        for line in lines:
            stripped = line.strip()
            if stripped.startswith("["):
                in_origin = stripped.lower() == '[remote "origin"]'
                continue
            if in_origin and stripped.startswith("url = "):
                repo = _repo_from_remote_url(stripped[6:])
                if repo:
                    return validate_repo(repo)
    return DEFAULT_REPO


def resolve_token(environment: Mapping[str, str] | None = None) -> str:
    """Read only the explicitly supported GitHub token environment names."""

    env = os.environ if environment is None else environment
    for name in ("GH_TOKEN", "GITHUB_TOKEN"):
        value = env.get(name)
        if value:
            return value
    raise SnapshotError(
        "GitHub API authentication is required; set GH_TOKEN or GITHUB_TOKEN "
        "in the calling environment (the token is never printed or stored)."
    )


class GitHubClient:
    """Small JSON-only GitHub REST client with safe error messages."""

    def __init__(
        self,
        token: str,
        *,
        api_root: str = API_ROOT,
        timeout: float = API_TIMEOUT_SECONDS,
    ) -> None:
        if not token:
            raise SnapshotError("GitHub API token is empty")
        self._token = token
        self._api_root = api_root.rstrip("/")
        self._timeout = timeout

    def get(self, endpoint: str) -> Any:
        path = endpoint if endpoint.startswith("/") else f"/{endpoint}"
        url = f"{self._api_root}{path}"
        headers = dict(API_HEADERS)
        headers["Authorization"] = f"Bearer {self._token}"
        request = urllib.request.Request(url, headers=headers, method="GET")
        try:
            with urllib.request.urlopen(request, timeout=self._timeout) as response:
                raw = response.read()
        except urllib.error.HTTPError as exc:
            if exc.code == 401:
                raise SnapshotError(
                    "GitHub API returned 401: the supplied token is invalid or "
                    "expired; refresh GH_TOKEN/GITHUB_TOKEN"
                ) from exc
            if exc.code == 403:
                raise SnapshotError(
                    "GitHub API returned 403: traffic endpoints require push or "
                    "admin visibility, or the API rate limit was reached"
                ) from exc
            if exc.code == 404:
                raise SnapshotError(
                    "GitHub API returned 404: check the repository name and token "
                    "access (traffic may be unavailable without repository access)"
                ) from exc
            raise SnapshotError(f"GitHub API returned HTTP {exc.code}") from exc
        except urllib.error.URLError as exc:
            raise SnapshotError(f"GitHub API request failed: {exc.reason}") from exc
        try:
            return json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise SnapshotError("GitHub API returned invalid JSON") from exc


def _integer(value: Any) -> int | None:
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _normalize_daily_rows(
    payload: Mapping[str, Any], source_key: str = "views"
) -> list[dict[str, Any]]:
    rows = payload.get(source_key)
    if not isinstance(rows, list):
        return []
    normalized: list[dict[str, Any]] = []
    for row in rows:
        if not isinstance(row, Mapping):
            continue
        item: dict[str, Any] = {}
        for key in ("timestamp", "count", "uniques"):
            if key in row:
                item[key] = row[key]
        if item:
            normalized.append(item)
    return normalized


def _normalize_traffic(payload: Any, source_key: str = "views") -> dict[str, Any]:
    source = payload if isinstance(payload, Mapping) else {}
    return {
        "count": _integer(source.get("count")),
        "uniques": _integer(source.get("uniques")),
        "daily": _normalize_daily_rows(source, source_key),
    }


def _normalize_referrers(payload: Any) -> list[dict[str, Any]]:
    if not isinstance(payload, list):
        return []
    rows: list[dict[str, Any]] = []
    for row in payload:
        if not isinstance(row, Mapping):
            continue
        item: dict[str, Any] = {}
        for key in ("referrer", "count", "uniques"):
            if key in row:
                item[key] = row[key]
        if item:
            rows.append(item)
    return rows


def _normalize_paths(payload: Any) -> list[dict[str, Any]]:
    if not isinstance(payload, list):
        return []
    rows: list[dict[str, Any]] = []
    for row in payload:
        if not isinstance(row, Mapping):
            continue
        item: dict[str, Any] = {}
        for key in ("path", "title", "count", "uniques"):
            if key in row:
                item[key] = row[key]
        if item:
            rows.append(item)
    return rows


def _normalize_assets(payload: Any) -> tuple[list[dict[str, Any]], int]:
    if not isinstance(payload, list):
        return [], 0
    assets: list[dict[str, Any]] = []
    total = 0
    for asset in payload:
        if not isinstance(asset, Mapping):
            continue
        downloads = _integer(asset.get("download_count"))
        if downloads is None:
            downloads = 0
        total += downloads
        assets.append(
            {
                "name": asset.get("name"),
                "download_count": downloads,
                "size": _integer(asset.get("size")),
                "browser_download_url": asset.get("browser_download_url"),
            }
        )
    return assets, total


def _normalize_releases(payload: Any) -> list[dict[str, Any]]:
    if not isinstance(payload, list):
        raise SnapshotError("GitHub releases endpoint returned a non-list payload")
    releases: list[dict[str, Any]] = []
    for release in payload:
        if not isinstance(release, Mapping):
            continue
        assets, asset_downloads = _normalize_assets(release.get("assets"))
        releases.append(
            {
                "tag_name": release.get("tag_name"),
                "name": release.get("name"),
                "published_at": release.get("published_at"),
                "html_url": release.get("html_url"),
                "draft": bool(release.get("draft", False)),
                "prerelease": bool(release.get("prerelease", False)),
                "assets": assets,
                "asset_downloads": asset_downloads,
            }
        )
    return releases


def normalize_captured_at(value: str | None = None) -> str:
    if value is None:
        parsed = dt.datetime.now(dt.timezone.utc)
    else:
        text = value.strip()
        if text.endswith("Z"):
            text = text[:-1] + "+00:00"
        try:
            parsed = dt.datetime.fromisoformat(text)
        except ValueError as exc:
            raise SnapshotError(
                "--captured-at must be an ISO-8601 timestamp"
            ) from exc
        if parsed.tzinfo is None:
            parsed = parsed.replace(tzinfo=dt.timezone.utc)
    parsed = parsed.astimezone(dt.timezone.utc).replace(microsecond=0)
    return parsed.isoformat().replace("+00:00", "Z")


def capture_snapshot(
    repo: str,
    client: Any,
    *,
    captured_at: str | None = None,
) -> dict[str, Any]:
    """Collect all endpoint payloads through a client-like ``get`` object."""

    repo = validate_repo(repo)
    prefix = f"/repos/{repo}"
    metadata = client.get(prefix)
    if not isinstance(metadata, Mapping):
        raise SnapshotError("GitHub repository endpoint returned a non-object payload")

    views = client.get(f"{prefix}/traffic/views")
    clones = client.get(f"{prefix}/traffic/clones")
    referrers = client.get(f"{prefix}/traffic/popular/referrers")
    popular_paths = client.get(f"{prefix}/traffic/popular/paths")
    releases = client.get(f"{prefix}/releases?per_page=100")
    subscribers = _integer(metadata.get("subscribers_count"))
    watchers = subscribers if subscribers is not None else _integer(metadata.get("watchers_count"))

    return {
        "schema_version": SCHEMA_VERSION,
        "captured_at": normalize_captured_at(captured_at),
        "repo": repo,
        "public": {
            "stars": _integer(metadata.get("stargazers_count")),
            "forks": _integer(metadata.get("forks_count")),
            # GitHub's legacy watchers_count can mirror stargazers_count.
            # The public UI's Watchers value is subscribers_count.
            "watchers": watchers,
            "subscribers": subscribers if subscribers is not None else watchers,
            "html_url": metadata.get("html_url") or f"https://github.com/{repo}",
        },
        "traffic": {
            "window_days": 14,
            "views": _normalize_traffic(views),
            "clones": _normalize_traffic(clones, "clones"),
            "referrers": _normalize_referrers(referrers),
            "popular_paths": _normalize_paths(popular_paths),
        },
        "releases": _normalize_releases(releases),
    }


def _traffic_delta(
    current: Mapping[str, Any], baseline: Mapping[str, Any]
) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key in ("count", "uniques"):
        current_value = _integer(current.get(key))
        baseline_value = _integer(baseline.get(key))
        result[key] = {
            "current": current_value,
            "baseline": baseline_value,
            "delta": (
                current_value - baseline_value
                if current_value is not None and baseline_value is not None
                else None
            ),
        }
    return result


def _asset_download_total(snapshot: Mapping[str, Any]) -> int:
    total = 0
    releases = snapshot.get("releases", [])
    if not isinstance(releases, list):
        return total
    for release in releases:
        if isinstance(release, Mapping):
            total += _integer(release.get("asset_downloads")) or 0
    return total


def compare_snapshots(
    current: Mapping[str, Any], baseline: Mapping[str, Any], *, target_stars: int = TARGET_STARS
) -> dict[str, Any]:
    current_public = current.get("public", {})
    baseline_public = baseline.get("public", {})
    current_stars = _integer(current_public.get("stars")) if isinstance(current_public, Mapping) else None
    baseline_stars = _integer(baseline_public.get("stars")) if isinstance(baseline_public, Mapping) else None
    stars_delta = (
        current_stars - baseline_stars
        if current_stars is not None and baseline_stars is not None
        else None
    )
    gap = max(0, target_stars - current_stars) if current_stars is not None else None

    current_traffic = current.get("traffic", {})
    baseline_traffic = baseline.get("traffic", {})
    traffic_deltas: dict[str, Any] = {}
    for key in ("views", "clones"):
        current_section = current_traffic.get(key, {}) if isinstance(current_traffic, Mapping) else {}
        baseline_section = baseline_traffic.get(key, {}) if isinstance(baseline_traffic, Mapping) else {}
        traffic_deltas[key] = _traffic_delta(
            current_section if isinstance(current_section, Mapping) else {},
            baseline_section if isinstance(baseline_section, Mapping) else {},
        )

    current_assets = _asset_download_total(current)
    baseline_assets = _asset_download_total(baseline)
    return {
        "target_stars": target_stars,
        "baseline_captured_at": baseline.get("captured_at"),
        "current_captured_at": current.get("captured_at"),
        "stars_delta": stars_delta,
        "star_delta": stars_delta,
        "gap_to_200": gap,
        "star_gap_to_200": gap,
        "traffic": traffic_deltas,
        "release_assets": {
            "current_downloads": current_assets,
            "baseline_downloads": baseline_assets,
            "delta": current_assets - baseline_assets,
        },
        "approximate": True,
        "note": (
            "Views and clones are rolling 14-day GitHub traffic windows; "
            "deltas can overlap and are not unique users or conversions."
        ),
    }


def render_markdown(snapshot: Mapping[str, Any]) -> str:
    public = snapshot.get("public", {})
    traffic = snapshot.get("traffic", {})
    lines = [
        "# GitHub growth snapshot",
        "",
        f"- Repository: `{snapshot.get('repo', 'unknown')}`",
        f"- Captured at (UTC): `{snapshot.get('captured_at', 'unknown')}`",
        f"- Stars: `{public.get('stars')}` (target: `{TARGET_STARS}`)",
        f"- Forks: `{public.get('forks')}`",
        f"- Watchers: `{public.get('watchers')}`",
        "",
        "## Rolling traffic",
        "",
        "GitHub traffic endpoints cover a rolling 14-day window. Counts are not "
        "unique people, and this file is intended for the ignored growth output "
        "directory.",
        "",
        "| Metric | Count | Uniques |",
        "|---|---:|---:|",
    ]
    for label, key in (("Views", "views"), ("Clones", "clones")):
        section = traffic.get(key, {}) if isinstance(traffic, Mapping) else {}
        lines.append(f"| {label} | {section.get('count')} | {section.get('uniques')} |")

    lines.extend(["", "## Releases and assets", "", "| Release | Asset downloads |", "|---|---:|"])
    releases = snapshot.get("releases", [])
    if isinstance(releases, list) and releases:
        for release in releases:
            if isinstance(release, Mapping):
                lines.append(
                    f"| `{release.get('tag_name')}` | {release.get('asset_downloads', 0)} |"
                )
    else:
        lines.append("| _(none returned)_ | 0 |")

    comparison = snapshot.get("comparison")
    if isinstance(comparison, Mapping):
        lines.extend(
            [
                "",
                "## Comparison",
                "",
                f"- Stars delta: `{comparison.get('stars_delta')}`",
                f"- Gap to 200 stars: `{comparison.get('gap_to_200')}`",
                f"- Release asset download delta: `{comparison.get('release_assets', {}).get('delta')}`",
                f"- Approximate rolling-window comparison: `{comparison.get('approximate')}`",
                f"- Note: {comparison.get('note')}",
            ]
        )
    return "\n".join(lines) + "\n"


def _ensure_safe_output(path: Path) -> None:
    if path.exists() and path.is_symlink():
        raise SnapshotError(f"refusing symlink output path: {path}")
    if path.exists() and path.is_dir():
        raise SnapshotError(f"output path is a directory: {path}")


def validate_output_paths(
    output: Path, markdown: Path | None, compare_to: Path | None
) -> None:
    _ensure_safe_output(output)
    if markdown is not None:
        _ensure_safe_output(markdown)
    resolved_output = output.resolve()
    if markdown is not None and resolved_output == markdown.resolve():
        raise SnapshotError("--output and --markdown must be different files")
    if compare_to is not None:
        if resolved_output == compare_to.resolve():
            raise SnapshotError("--output must not overwrite --compare-to")
        if markdown is not None and markdown.resolve() == compare_to.resolve():
            raise SnapshotError("--markdown must not overwrite --compare-to")


def _atomic_write(path: Path, text: str) -> None:
    _ensure_safe_output(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path: Path | None = None
    try:
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
        )
        temporary_path = Path(temporary_name)
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            handle.write(text)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary_path, path)
        temporary_path = None
    finally:
        if temporary_path is not None:
            try:
                temporary_path.unlink()
            except FileNotFoundError:
                pass


def _load_json(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise SnapshotError(f"cannot read comparison snapshot: {path}") from exc
    if not isinstance(payload, dict):
        raise SnapshotError(f"comparison snapshot is not a JSON object: {path}")
    return payload


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Capture authenticated GitHub public and rolling-traffic metrics."
    )
    parser.add_argument(
        "--repo",
        default=None,
        help="GitHub owner/name (default: infer this checkout's origin)",
    )
    parser.add_argument("--output", required=True, help="JSON snapshot output path")
    parser.add_argument("--markdown", help="Optional Markdown summary output path")
    parser.add_argument("--compare-to", help="Prior JSON snapshot for a comparison")
    parser.add_argument(
        "--captured-at",
        help="Deterministic ISO-8601 capture timestamp; normalized to UTC",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    output_path = Path(args.output)
    markdown_path = Path(args.markdown) if args.markdown else None
    compare_path = Path(args.compare_to) if args.compare_to else None
    try:
        validate_output_paths(output_path, markdown_path, compare_path)
        repo = validate_repo(args.repo) if args.repo else infer_default_repo()
        token = resolve_token()
        client = GitHubClient(token)
        snapshot = capture_snapshot(repo, client, captured_at=args.captured_at)
        if compare_path is not None:
            snapshot["comparison"] = compare_snapshots(snapshot, _load_json(compare_path))
        _atomic_write(
            output_path,
            json.dumps(snapshot, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        )
        if markdown_path is not None:
            _atomic_write(markdown_path, render_markdown(snapshot))
        print(f"growth snapshot written: {output_path}")
        if markdown_path is not None:
            print(f"growth markdown written: {markdown_path}")
        return 0
    except SnapshotError as exc:
        print(f"growth snapshot error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
