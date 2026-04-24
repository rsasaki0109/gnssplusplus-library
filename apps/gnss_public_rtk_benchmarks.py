#!/usr/bin/env python3
"""Public moving-RTK benchmark coverage matrix."""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
import os
from typing import Iterable


@dataclass(frozen=True)
class PublicRTKBenchmark:
    profile_id: str
    name: str
    status: str
    role: str
    reference: str
    receiver_artifacts: str
    adapter: str
    caveat: str
    source: str


PROFILES: tuple[PublicRTKBenchmark, ...] = (
    PublicRTKBenchmark(
        profile_id="ppc-dataset",
        name="PPC-Dataset Tokyo/Nagoya",
        status="primary-public-rtk-signoff",
        role="survey-grade receiver observation sign-off",
        reference="reference.csv trajectory truth for Tokyo/Nagoya runs",
        receiver_artifacts="Septentrio mosaic-X5 rover RINEX plus Trimble Alloy/NetR9 base RINEX/nav",
        adapter="native ppc-demo and ppc-rtk-signoff layout with receiver hardware provenance",
        caveat="survey-grade observations and reference truth are bundled; proprietary receiver-engine solution is not treated as the benchmark target",
        source="https://github.com/taroz/PPC-Dataset",
    ),
    PublicRTKBenchmark(
        profile_id="urban-nav-tokyo",
        name="UrbanNav Tokyo Odaiba/Shinjuku",
        status="wired-path-overrides",
        role="tier-1 public smoke",
        reference="Applanix POS LV620 reference.csv",
        receiver_artifacts="u-blox F9P rover RINEX plus Trimble NetR9 rover/base RINEX",
        adapter="ppc-rtk-signoff path overrides with --commercial-rover",
        caveat="two Tokyo runs; Trimble observations are solved by libgnss++, not the Trimble RTK engine",
        source="https://github.com/IPNL-POLYU/UrbanNavDataset",
    ),
    PublicRTKBenchmark(
        profile_id="smartloc",
        name="smartLoc urban GNSS",
        status="receiver-fix-signoff",
        role="urban NLOS stress",
        reference="NovAtel SPAN differential RTK/IMU reference",
        receiver_artifacts="u-blox EVK-M8T mass-market raw/fix data plus NLOS labels",
        adapter="smartloc-adapter exports receiver/raw data; smartloc-signoff gates receiver-fix metrics",
        caveat="solver sign-off still needs compatible nav/base inputs beyond the public receiver-fix path",
        source="https://www.tu-chemnitz.de/projekt/smartLoc/gnss_dataset.html.en",
    ),
    PublicRTKBenchmark(
        profile_id="gsdc",
        name="Google Smartphone Decimeter Challenge",
        status="candidate",
        role="phone-grade stress",
        reference="precise ground truth for raw GNSS and IMU traces",
        receiver_artifacts="Android raw GNSS measurements and sensor logs",
        adapter="needs smartphone measurement converter; not a commercial RTK receiver path",
        caveat="useful stress data, but phone antenna/clock behavior is a different receiver class",
        source="https://www.ion.org/gnss/googlecompetition.cfm",
    ),
    PublicRTKBenchmark(
        profile_id="ford-hdr",
        name="Ford Highway Driving RTK",
        status="candidate",
        role="large-scale highway coverage",
        reference="INS coupled with survey-grade GNSS receivers",
        receiver_artifacts="production automotive GNSS over long highway drives",
        adapter="needs Ford log normalizer and highway-specific thresholds",
        caveat="excellent scale, but not an urban canyon RTK receiver comparison",
        source="https://arxiv.org/abs/2010.01774",
    ),
    PublicRTKBenchmark(
        profile_id="oxford-robotcar-rtk",
        name="Oxford RobotCar RTK ground truth",
        status="candidate",
        role="long-term localization coverage",
        reference="post-processed raw GPS/IMU/static-base centimeter ground truth",
        receiver_artifacts="RobotCar traversals with reference localization products",
        adapter="needs RobotCar reference mapper and observation availability check",
        caveat="strong localization benchmark, but indirect for commercial RTK receiver claims",
        source="https://arxiv.org/abs/2002.10152",
    ),
)


def select_profiles(status: Iterable[str] | None = None) -> list[PublicRTKBenchmark]:
    statuses = set(status or ())
    if not statuses:
        return list(PROFILES)
    return [profile for profile in PROFILES if profile.status in statuses]


def profiles_as_dicts(profiles: Iterable[PublicRTKBenchmark]) -> list[dict[str, str]]:
    return [asdict(profile) for profile in profiles]


def render_markdown(profiles: Iterable[PublicRTKBenchmark]) -> str:
    lines = [
        "| Profile | Status | Role | Reference | Receiver artifacts | Adapter | Caveat |",
        "|---|---|---|---|---|---|---|",
    ]
    for profile in profiles:
        lines.append(
            "| "
            + " | ".join(
                [
                    f"[{profile.name}]({profile.source})",
                    profile.status,
                    profile.role,
                    profile.reference,
                    profile.receiver_artifacts,
                    profile.adapter,
                    profile.caveat,
                ]
            )
            + " |"
        )
    return "\n".join(lines)


def render_table(profiles: Iterable[PublicRTKBenchmark]) -> str:
    rows = profiles_as_dicts(profiles)
    columns = ("profile_id", "status", "role", "adapter")
    widths = {
        column: max(len(column), *(len(row[column]) for row in rows)) if rows else len(column)
        for column in columns
    }
    lines = ["  ".join(column.ljust(widths[column]) for column in columns)]
    lines.append("  ".join("-" * widths[column] for column in columns))
    for row in rows:
        lines.append("  ".join(row[column].ljust(widths[column]) for column in columns))
    return "\n".join(lines)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("--format", choices=("table", "markdown", "json"), default="table")
    parser.add_argument(
        "--status",
        action="append",
        choices=(
            "primary-public-rtk-signoff",
            "wired",
            "wired-path-overrides",
            "receiver-csv-adapter",
            "rawx-rinex-adapter",
            "receiver-fix-signoff",
            "candidate",
        ),
        default=None,
        help="Filter by adapter status. Repeat to include multiple statuses.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    profiles = select_profiles(args.status)
    if args.format == "json":
        print(json.dumps({"public_rtk_benchmarks": profiles_as_dicts(profiles)}, indent=2))
    elif args.format == "markdown":
        print(render_markdown(profiles))
    else:
        print(render_table(profiles))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
