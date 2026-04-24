#!/usr/bin/env python3
"""Normalize smartLoc NAV-POSLLH.csv into libgnss++ comparison artifacts."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import csv
from dataclasses import dataclass
import io
import json
import os
from pathlib import Path
import re
from datetime import datetime, timedelta
from typing import Iterator
from urllib import parse, request, error as urlerror
import zipfile


NAV_POSLLH_NAME = "NAV-POSLLH.csv"
RXM_RAWX_NAME = "RXM-RAWX.csv"
GPS_EPOCH = datetime(1980, 1, 6)
RINEX_OBS_TYPES = ("C1C", "L1C", "D1C", "S1C")
DEFAULT_SMARTLOC_ZIP_URL = (
    "https://www.tu-chemnitz.de/projekt/smartLoc/gnss_dataset/"
    "berlin/scenario1/berlin1_potsdamer_platz.zip"
)


@dataclass(frozen=True)
class SmartLocEpoch:
    week: int
    tow: float
    gt_lat_deg: float
    gt_lon_deg: float
    gt_height_m: float
    gt_heading_rad: float | None
    gt_velocity_mps: float | None
    gt_yaw_rate_rad_s: float | None
    receiver_lat_deg: float
    receiver_lon_deg: float
    receiver_height_m: float
    receiver_h_acc_m: float | None
    receiver_v_acc_m: float | None


@dataclass(frozen=True)
class SmartLocRawObservation:
    week: int
    tow: float
    system: str
    sv_id: int
    frequency_id: int
    pseudorange_m: float
    carrier_phase_cycles: float
    doppler_hz: float
    cno_dbhz: float
    locktime_ms: float | None
    tracking_status: int | None
    nlos: int | None


def normalize_header(name: str) -> str:
    normalized = re.sub(r"[^a-z0-9]+", "_", name.strip().lower())
    while "__" in normalized:
        normalized = normalized.replace("__", "_")
    return normalized.strip("_")


def get_token(row: dict[str, str], field_map: dict[str, str], *aliases: str) -> str | None:
    for alias in aliases:
        original = field_map.get(alias)
        if original is None:
            continue
        value = row.get(original)
        if value is not None and value.strip():
            return value.strip()
    return None


def parse_optional_float(token: str | None) -> float | None:
    if token is None or token == "":
        return None
    return float(token)


def parse_optional_int(token: str | None) -> int | None:
    if token is None or token == "" or token == "#":
        return None
    return int(float(token))


def normalize_system(token: str) -> str:
    normalized = normalize_header(token)
    if normalized in {"gps", "g"}:
        return "G"
    if normalized in {"glonass", "glo", "r"}:
        return "R"
    if normalized in {"galileo", "gal", "e"}:
        return "E"
    if normalized in {"beidou", "bds", "compass", "c"}:
        return "C"
    if normalized in {"qzss", "qz", "j"}:
        return "J"
    if normalized in {"sbas", "s"}:
        return "S"
    if normalized in {"navic", "irnss", "i"}:
        return "I"
    raise SystemExit(f"Unsupported smartLoc GNSS identifier: {token}")


def system_prn(system: str, sv_id: int) -> int:
    if system == "S" and sv_id >= 100:
        return sv_id
    return sv_id


def resolve_smartloc_member(
    input_path: Path | None,
    direct_path: Path | None,
    member_name: str,
) -> tuple[Path, str | None]:
    if direct_path is not None:
        return direct_path, None
    if input_path is None:
        raise SystemExit(f"Provide either --input or --{member_name.lower().replace('.', '-')}")
    if input_path.is_dir():
        direct = input_path / member_name
        if direct.exists():
            return direct, None
        matches = sorted(input_path.rglob(member_name))
        if not matches:
            raise SystemExit(f"No {member_name} found under {input_path}")
        return matches[0], None
    if input_path.suffix.lower() == ".zip":
        with zipfile.ZipFile(input_path) as archive:
            matches = [name for name in archive.namelist() if name.endswith(member_name)]
        if not matches:
            raise SystemExit(f"No {member_name} found in {input_path}")
        return input_path, sorted(matches)[0]
    return input_path, None


def resolve_nav_posllh_path(input_path: Path | None, nav_posllh: Path | None) -> tuple[Path, str | None]:
    return resolve_smartloc_member(input_path, nav_posllh, NAV_POSLLH_NAME)


def resolve_rawx_path(input_path: Path | None, rawx: Path | None) -> tuple[Path, str | None]:
    return resolve_smartloc_member(input_path, rawx, RXM_RAWX_NAME)


def download_cache_filename(url: str) -> str:
    parsed = parse.urlparse(url)
    path_parts = [parse.unquote(part) for part in parsed.path.split("/") if part]
    for part in reversed(path_parts):
        if part.lower().endswith(".zip"):
            return Path(part).name
    if path_parts:
        candidate = Path(path_parts[-1]).name
        if candidate.lower().endswith(".zip"):
            return candidate
    return "smartloc.zip"


def ensure_local_input(
    *,
    input_path: Path | None,
    nav_posllh: Path | None,
    rawx: Path | None,
    need_nav: bool,
    need_raw: bool,
    input_url: str | None,
    download_cache_dir: Path,
    force_download: bool = False,
) -> tuple[Path | None, str | None]:
    if input_path is not None:
        if not input_path.exists():
            raise SystemExit(f"Missing smartLoc input: {input_path}")
        return input_path, None

    needs_nav_zip = need_nav and nav_posllh is None
    needs_raw_zip = need_raw and rawx is None
    if not needs_nav_zip and not needs_raw_zip:
        return None, None
    if input_url is None:
        raise SystemExit("Provide --input, direct smartLoc CSV paths, or --input-url")

    download_cache_dir.mkdir(parents=True, exist_ok=True)
    destination = download_cache_dir / download_cache_filename(input_url)
    if force_download or not destination.exists():
        try:
            with request.urlopen(input_url, timeout=60.0) as response, destination.open("wb") as handle:
                handle.write(response.read())
        except urlerror.URLError as exc:
            raise SystemExit(f"Failed to download smartLoc input `{input_url}`: {exc}") from exc
    return destination, input_url


@contextmanager
def open_nav_posllh_text(path: Path, archive_member: str | None) -> Iterator[io.TextIOBase]:
    if archive_member is None:
        with path.open(encoding="utf-8", newline="") as handle:
            yield handle
        return
    with zipfile.ZipFile(path) as archive:
        with archive.open(archive_member) as binary:
            with io.TextIOWrapper(binary, encoding="utf-8", newline="") as text:
                yield text


def open_smartloc_text(path: Path, archive_member: str | None) -> Iterator[io.TextIOBase]:
    return open_nav_posllh_text(path, archive_member)


def read_nav_posllh_epochs(
    path: Path,
    *,
    archive_member: str | None = None,
    max_rows: int = -1,
) -> list[SmartLocEpoch]:
    epochs: list[SmartLocEpoch] = []
    with open_nav_posllh_text(path, archive_member) as handle:
        reader = csv.DictReader(handle, delimiter=";")
        if not reader.fieldnames:
            return epochs
        field_map = {normalize_header(name): name for name in reader.fieldnames if name}

        for row in reader:
            if max_rows > 0 and len(epochs) >= max_rows:
                break
            week_token = get_token(row, field_map, "gpsweek_weeks", "gps_week")
            tow_token = get_token(
                row,
                field_map,
                "gpssecondsofweek_s",
                "gps_seconds_of_week_s",
                "gps_tow_s",
            )
            gt_lon_token = get_token(
                row,
                field_map,
                "longitude_gt_lon_deg",
                "gt_lon_deg",
                "longitude_gt_lon",
            )
            gt_lat_token = get_token(
                row,
                field_map,
                "latitude_gt_lat_deg",
                "gt_lat_deg",
                "latitude_gt_lat",
            )
            gt_height_token = get_token(
                row,
                field_map,
                "height_above_ellipsoid_gt_height_m",
                "gt_height_m",
                "height_gt_height_m",
            )
            receiver_lon_token = get_token(
                row,
                field_map,
                "longitude_lon_deg",
                "lon_deg",
                "longitude_lon",
            )
            receiver_lat_token = get_token(
                row,
                field_map,
                "latitude_lat_deg",
                "lat_deg",
                "latitude_lat",
            )
            receiver_height_token = get_token(
                row,
                field_map,
                "height_above_ellipsoid_height_m",
                "height_m",
                "height",
            )
            required = (
                week_token,
                tow_token,
                gt_lon_token,
                gt_lat_token,
                gt_height_token,
                receiver_lon_token,
                receiver_lat_token,
                receiver_height_token,
            )
            if any(token is None for token in required):
                raise SystemExit(f"{path} is missing required smartLoc NAV-POSLLH columns")

            epochs.append(
                SmartLocEpoch(
                    week=int(float(week_token or 0)),
                    tow=float(tow_token or 0.0),
                    gt_lat_deg=float(gt_lat_token or 0.0),
                    gt_lon_deg=float(gt_lon_token or 0.0),
                    gt_height_m=float(gt_height_token or 0.0),
                    gt_heading_rad=parse_optional_float(
                        get_token(
                            row,
                            field_map,
                            "heading_0_east_counterclockwise_gt_heading_rad",
                            "gt_heading_rad",
                        )
                    ),
                    gt_velocity_mps=parse_optional_float(
                        get_token(row, field_map, "velocity_gt_velocity_m_s", "gt_velocity_m_s")
                    ),
                    gt_yaw_rate_rad_s=parse_optional_float(
                        get_token(
                            row,
                            field_map,
                            "yaw_rate_gt_yaw_rate_rad_s",
                            "gt_yaw_rate_rad_s",
                        )
                    ),
                    receiver_lat_deg=float(receiver_lat_token or 0.0),
                    receiver_lon_deg=float(receiver_lon_token or 0.0),
                    receiver_height_m=float(receiver_height_token or 0.0),
                    receiver_h_acc_m=parse_optional_float(
                        get_token(
                            row,
                            field_map,
                            "horizontal_accuracy_estimate_hacc_m",
                            "hacc_m",
                            "h_acc_m",
                        )
                    ),
                    receiver_v_acc_m=parse_optional_float(
                        get_token(
                            row,
                            field_map,
                            "vertical_accuracy_estimate_vacc_m",
                            "vacc_m",
                            "v_acc_m",
                        )
                    ),
                )
            )
    epochs.sort(key=lambda epoch: (epoch.week, epoch.tow))
    return epochs


def read_rawx_observations(
    path: Path,
    *,
    archive_member: str | None = None,
    max_epochs: int = -1,
) -> list[SmartLocRawObservation]:
    observations: list[SmartLocRawObservation] = []
    seen_epochs: set[tuple[int, float]] = set()
    with open_smartloc_text(path, archive_member) as handle:
        reader = csv.DictReader(handle, delimiter=";")
        if not reader.fieldnames:
            return observations
        field_map = {normalize_header(name): name for name in reader.fieldnames if name}
        for row in reader:
            week_token = get_token(
                row,
                field_map,
                "gps_week_number_week_weeks",
                "week_weeks",
                "week",
                "gpsweek_weeks",
            )
            tow_token = get_token(
                row,
                field_map,
                "measurement_time_of_week_rcvtow_s",
                "rcvtow_s",
                "gpssecondsofweek_s",
                "gps_tow_s",
            )
            system_token = get_token(row, field_map, "gnss_identifier_gnssid", "gnssid", "system")
            sv_token = get_token(row, field_map, "satellite_identifier_svid", "svid", "sv_id")
            pr_token = get_token(row, field_map, "pseudorange_measurement_prmes_m", "prmes_m")
            cp_token = get_token(row, field_map, "carrier_phase_measurement_cpmes_cycles", "cpmes_cycles")
            doppler_token = get_token(row, field_map, "doppler_measurement_domes_hz", "domes_hz")
            cno_token = get_token(row, field_map, "carrier_to_noise_density_ratio_cno_dbhz", "cno_dbhz")
            required = (
                week_token,
                tow_token,
                system_token,
                sv_token,
                pr_token,
                cp_token,
                doppler_token,
                cno_token,
            )
            if any(token is None for token in required):
                raise SystemExit(f"{path} is missing required smartLoc RXM-RAWX columns")
            week = int(float(week_token or 0))
            tow = float(tow_token or 0.0)
            epoch_key = (week, round(tow, 9))
            if epoch_key not in seen_epochs:
                if max_epochs > 0 and len(seen_epochs) >= max_epochs:
                    break
                seen_epochs.add(epoch_key)
            observations.append(
                SmartLocRawObservation(
                    week=week,
                    tow=tow,
                    system=normalize_system(system_token or ""),
                    sv_id=int(float(sv_token or 0)),
                    frequency_id=int(
                        float(
                            get_token(
                                row,
                                field_map,
                                "frequency_slot_only_glonass_freqid",
                                "freqid",
                            )
                            or 0
                        )
                    ),
                    pseudorange_m=float(pr_token or 0.0),
                    carrier_phase_cycles=float(cp_token or 0.0),
                    doppler_hz=float(doppler_token or 0.0),
                    cno_dbhz=float(cno_token or 0.0),
                    locktime_ms=parse_optional_float(
                        get_token(
                            row,
                            field_map,
                            "carrier_phase_locktime_counter_locktime_ms",
                            "locktime_ms",
                        )
                    ),
                    tracking_status=parse_optional_int(
                        get_token(row, field_map, "tracking_status_trkstat", "trkstat")
                    ),
                    nlos=parse_optional_int(
                        get_token(
                            row,
                            field_map,
                            "nlos_0_no_1_yes_no_information",
                            "nlos",
                        )
                    ),
                )
            )
    observations.sort(key=lambda obs: (obs.week, obs.tow, obs.system, obs.sv_id))
    return observations


def grouped_raw_observations(
    observations: list[SmartLocRawObservation],
) -> list[tuple[tuple[int, float], list[SmartLocRawObservation]]]:
    grouped: dict[tuple[int, float], list[SmartLocRawObservation]] = {}
    for observation in observations:
        grouped.setdefault((observation.week, round(observation.tow, 9)), []).append(observation)
    return sorted(grouped.items(), key=lambda item: item[0])


def write_reference_csv(path: Path, epochs: list[SmartLocEpoch]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "gps_week",
                "gps_tow_s",
                "lat_deg",
                "lon_deg",
                "height_m",
                "heading_rad",
                "velocity_mps",
                "yaw_rate_rad_s",
            ]
        )
        for epoch in epochs:
            writer.writerow(
                [
                    epoch.week,
                    f"{epoch.tow:.9f}",
                    f"{epoch.gt_lat_deg:.12f}",
                    f"{epoch.gt_lon_deg:.12f}",
                    f"{epoch.gt_height_m:.6f}",
                    "" if epoch.gt_heading_rad is None else f"{epoch.gt_heading_rad:.12f}",
                    "" if epoch.gt_velocity_mps is None else f"{epoch.gt_velocity_mps:.6f}",
                    "" if epoch.gt_yaw_rate_rad_s is None else f"{epoch.gt_yaw_rate_rad_s:.12f}",
                ]
            )


def write_receiver_csv(path: Path, epochs: list[SmartLocEpoch], receiver_label: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "gps_week",
                "gps_tow_s",
                "lat_deg",
                "lon_deg",
                "height_m",
                "status",
                "satellites",
                "h_acc_m",
                "v_acc_m",
                "label",
            ]
        )
        for epoch in epochs:
            writer.writerow(
                [
                    epoch.week,
                    f"{epoch.tow:.9f}",
                    f"{epoch.receiver_lat_deg:.12f}",
                    f"{epoch.receiver_lon_deg:.12f}",
                    f"{epoch.receiver_height_m:.6f}",
                    1,
                    0,
                    "" if epoch.receiver_h_acc_m is None else f"{epoch.receiver_h_acc_m:.6f}",
                    "" if epoch.receiver_v_acc_m is None else f"{epoch.receiver_v_acc_m:.6f}",
                    receiver_label,
                ]
            )


def write_raw_observation_csv(path: Path, observations: list[SmartLocRawObservation]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "gps_week",
                "gps_tow_s",
                "system",
                "sv_id",
                "prn",
                "frequency_id",
                "pseudorange_m",
                "carrier_phase_cycles",
                "doppler_hz",
                "cno_dbhz",
                "locktime_ms",
                "tracking_status",
                "nlos",
            ]
        )
        for obs in observations:
            writer.writerow(
                [
                    obs.week,
                    f"{obs.tow:.9f}",
                    obs.system,
                    obs.sv_id,
                    system_prn(obs.system, obs.sv_id),
                    obs.frequency_id,
                    f"{obs.pseudorange_m:.6f}",
                    f"{obs.carrier_phase_cycles:.6f}",
                    f"{obs.doppler_hz:.6f}",
                    f"{obs.cno_dbhz:.3f}",
                    "" if obs.locktime_ms is None else f"{obs.locktime_ms:.3f}",
                    "" if obs.tracking_status is None else obs.tracking_status,
                    "" if obs.nlos is None else obs.nlos,
                ]
            )


def header_line(content: str, label: str) -> str:
    return f"{content[:60]:<60}{label}\n"


def gps_week_tow_to_calendar(week: int, tow: float) -> tuple[int, int, int, int, int, float]:
    stamp = GPS_EPOCH + timedelta(weeks=week, seconds=tow)
    second = stamp.second + stamp.microsecond / 1_000_000.0
    return stamp.year, stamp.month, stamp.day, stamp.hour, stamp.minute, second


def rinex_epoch_line(week: int, tow: float, satellite_count: int) -> str:
    year, month, day, hour, minute, second = gps_week_tow_to_calendar(week, tow)
    return f"> {year:04d} {month:02d} {day:02d} {hour:02d} {minute:02d} {second:011.7f}  0{satellite_count:3d}\n"


def rinex_observation_field(value: float) -> str:
    return f"{value:14.3f}  "


def write_raw_observation_rinex(path: Path, observations: list[SmartLocRawObservation]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    systems = sorted({obs.system for obs in observations})
    with path.open("w", encoding="utf-8", newline="") as handle:
        handle.write(header_line("     3.04           OBSERVATION DATA    M", "RINEX VERSION / TYPE"))
        handle.write(header_line("gnss smartloc-adap  libgnss++", "PGM / RUN BY / DATE"))
        handle.write(header_line("smartLoc RXM-RAWX normalized by libgnss++ adapter", "COMMENT"))
        for system in systems:
            content = f"{system}  {len(RINEX_OBS_TYPES):3d} " + " ".join(RINEX_OBS_TYPES)
            handle.write(header_line(content, "SYS / # / OBS TYPES"))
        handle.write(header_line("", "END OF HEADER"))
        for (_week, _tow), epoch_observations in grouped_raw_observations(observations):
            filtered = [obs for obs in epoch_observations if obs.system in systems]
            handle.write(rinex_epoch_line(filtered[0].week, filtered[0].tow, len(filtered)))
            for obs in filtered:
                sat_id = f"{obs.system}{system_prn(obs.system, obs.sv_id):02d}"
                handle.write(
                    sat_id
                    + rinex_observation_field(obs.pseudorange_m)
                    + rinex_observation_field(obs.carrier_phase_cycles)
                    + rinex_observation_field(obs.doppler_hz)
                    + rinex_observation_field(obs.cno_dbhz)
                    + "\n"
                )


def build_summary(
    *,
    source_path: Path,
    archive_member: str | None,
    reference_csv: Path,
    receiver_csv: Path,
    receiver_label: str,
    epochs: list[SmartLocEpoch],
) -> dict[str, object]:
    return {
        "dataset": "smartLoc",
        "source": "NAV-POSLLH.csv",
        "source_path": str(source_path),
        "archive_member": archive_member,
        "reference_csv": str(reference_csv),
        "receiver_csv": str(receiver_csv),
        "receiver_label": receiver_label,
        "epochs": len(epochs),
        "license": "CC BY-NC-SA 4.0",
        "adapter_status": "receiver_csv_adapter",
    }


def build_raw_summary(
    *,
    source_path: Path,
    archive_member: str | None,
    raw_csv: Path | None,
    obs_rinex: Path | None,
    observations: list[SmartLocRawObservation],
) -> dict[str, object]:
    epoch_count = len(grouped_raw_observations(observations))
    nlos_known = [obs.nlos for obs in observations if obs.nlos is not None]
    nlos_count = sum(1 for value in nlos_known if value == 1)
    return {
        "dataset": "smartLoc",
        "source": "RXM-RAWX.csv",
        "source_path": str(source_path),
        "archive_member": archive_member,
        "raw_csv": str(raw_csv) if raw_csv is not None else None,
        "obs_rinex": str(obs_rinex) if obs_rinex is not None else None,
        "raw_observations": len(observations),
        "raw_epochs": epoch_count,
        "systems": sorted({obs.system for obs in observations}),
        "nlos_labeled_observations": len(nlos_known),
        "nlos_observations": nlos_count,
        "adapter_status": "rawx_rinex_adapter",
    }


def convert_nav_posllh(
    *,
    source_path: Path,
    reference_csv: Path,
    receiver_csv: Path,
    receiver_label: str,
    archive_member: str | None = None,
    summary_json: Path | None = None,
    max_rows: int = -1,
) -> dict[str, object]:
    epochs = read_nav_posllh_epochs(source_path, archive_member=archive_member, max_rows=max_rows)
    if not epochs:
        raise SystemExit(f"No smartLoc epochs found in {source_path}")
    write_reference_csv(reference_csv, epochs)
    write_receiver_csv(receiver_csv, epochs, receiver_label)
    summary = build_summary(
        source_path=source_path,
        archive_member=archive_member,
        reference_csv=reference_csv,
        receiver_csv=receiver_csv,
        receiver_label=receiver_label,
        epochs=epochs,
    )
    if summary_json is not None:
        summary_json.parent.mkdir(parents=True, exist_ok=True)
        summary_json.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return summary


def convert_rawx(
    *,
    source_path: Path,
    archive_member: str | None = None,
    raw_csv: Path | None = None,
    obs_rinex: Path | None = None,
    summary_json: Path | None = None,
    max_epochs: int = -1,
) -> dict[str, object]:
    if raw_csv is None and obs_rinex is None:
        raise SystemExit("Provide --raw-csv and/or --obs-rinex for RXM-RAWX export")
    observations = read_rawx_observations(
        source_path,
        archive_member=archive_member,
        max_epochs=max_epochs,
    )
    if not observations:
        raise SystemExit(f"No smartLoc raw observations found in {source_path}")
    if raw_csv is not None:
        write_raw_observation_csv(raw_csv, observations)
    if obs_rinex is not None:
        write_raw_observation_rinex(obs_rinex, observations)
    summary = build_raw_summary(
        source_path=source_path,
        archive_member=archive_member,
        raw_csv=raw_csv,
        obs_rinex=obs_rinex,
        observations=observations,
    )
    if summary_json is not None:
        summary_json.parent.mkdir(parents=True, exist_ok=True)
        summary_json.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument(
        "--input",
        type=Path,
        default=None,
        help="smartLoc dataset zip or extracted directory containing NAV-POSLLH.csv.",
    )
    parser.add_argument(
        "--input-url",
        default=DEFAULT_SMARTLOC_ZIP_URL,
        help="Public smartLoc dataset zip URL used when --input/direct CSV paths are omitted.",
    )
    parser.add_argument(
        "--download-cache-dir",
        type=Path,
        default=Path(__file__).resolve().parent.parent / "output" / "downloads",
        help="Directory for downloaded smartLoc zips.",
    )
    parser.add_argument(
        "--force-download",
        action="store_true",
        help="Re-fetch --input-url even when the cached zip already exists.",
    )
    parser.add_argument(
        "--nav-posllh",
        type=Path,
        default=None,
        help="Direct path to NAV-POSLLH.csv. Overrides --input.",
    )
    parser.add_argument(
        "--rawx",
        type=Path,
        default=None,
        help="Direct path to RXM-RAWX.csv. Overrides --input for raw export.",
    )
    parser.add_argument("--reference-csv", type=Path, default=None)
    parser.add_argument("--receiver-csv", type=Path, default=None)
    parser.add_argument("--receiver-label", default="smartloc_ublox_evk_m8t")
    parser.add_argument("--raw-csv", type=Path, default=None)
    parser.add_argument("--obs-rinex", type=Path, default=None)
    parser.add_argument("--summary-json", type=Path, default=None)
    parser.add_argument("--max-rows", type=int, default=-1)
    parser.add_argument("--raw-max-epochs", type=int, default=-1)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    export_nav = args.reference_csv is not None or args.receiver_csv is not None
    export_raw = args.raw_csv is not None or args.obs_rinex is not None
    if export_nav and (args.reference_csv is None or args.receiver_csv is None):
        raise SystemExit("--reference-csv and --receiver-csv must be provided together")
    if not export_nav and not export_raw:
        raise SystemExit("Provide reference/receiver CSV outputs or raw RXM-RAWX outputs")

    resolved_input, resolved_input_url = ensure_local_input(
        input_path=args.input,
        nav_posllh=args.nav_posllh,
        rawx=args.rawx,
        need_nav=export_nav,
        need_raw=export_raw,
        input_url=args.input_url,
        download_cache_dir=args.download_cache_dir,
        force_download=args.force_download,
    )

    summaries: dict[str, object] = {}
    nav_summary: dict[str, object] | None = None
    raw_summary: dict[str, object] | None = None
    if export_nav:
        assert args.reference_csv is not None
        assert args.receiver_csv is not None
        source_path, archive_member = resolve_nav_posllh_path(resolved_input, args.nav_posllh)
        nav_summary = convert_nav_posllh(
            source_path=source_path,
            archive_member=archive_member,
            reference_csv=args.reference_csv,
            receiver_csv=args.receiver_csv,
            receiver_label=args.receiver_label,
            max_rows=args.max_rows,
        )
        if resolved_input_url is not None:
            nav_summary["input_url"] = resolved_input_url
            nav_summary["downloaded_input"] = str(resolved_input)
        summaries["nav_posllh"] = nav_summary
    if export_raw:
        source_path, archive_member = resolve_rawx_path(resolved_input, args.rawx)
        raw_summary = convert_rawx(
            source_path=source_path,
            archive_member=archive_member,
            raw_csv=args.raw_csv,
            obs_rinex=args.obs_rinex,
            max_epochs=args.raw_max_epochs,
        )
        if resolved_input_url is not None:
            raw_summary["input_url"] = resolved_input_url
            raw_summary["downloaded_input"] = str(resolved_input)
        summaries["rxm_rawx"] = raw_summary

    if args.summary_json is not None:
        if nav_summary is not None and raw_summary is None:
            payload = nav_summary
        elif raw_summary is not None and nav_summary is None:
            payload = raw_summary
        else:
            payload = {"dataset": "smartLoc", "adapters": summaries}
        args.summary_json.parent.mkdir(parents=True, exist_ok=True)
        args.summary_json.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    print("Finished smartLoc adapter export.")
    if nav_summary is not None:
        print(f"  epochs: {nav_summary['epochs']}")
        print(f"  reference_csv: {nav_summary['reference_csv']}")
        print(f"  receiver_csv: {nav_summary['receiver_csv']}")
    if raw_summary is not None:
        print(f"  raw_epochs: {raw_summary['raw_epochs']}")
        print(f"  raw_observations: {raw_summary['raw_observations']}")
        if raw_summary["raw_csv"] is not None:
            print(f"  raw_csv: {raw_summary['raw_csv']}")
        if raw_summary["obs_rinex"] is not None:
            print(f"  obs_rinex: {raw_summary['obs_rinex']}")
    if args.summary_json is not None:
        print(f"  summary_json: {args.summary_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
