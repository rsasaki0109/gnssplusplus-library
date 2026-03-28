#!/usr/bin/env python3
"""Local web UI for browsing libgnss++ benchmark artifacts and receiver status."""

from __future__ import annotations

import argparse
import json
import os
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import signal
import sys
import threading
from typing import Any
from urllib.parse import parse_qs, urlparse


ROOT_DIR = Path(__file__).resolve().parent.parent
SCRIPTS_DIR = ROOT_DIR / "scripts"

if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import generate_driving_comparison as driving_comparison  # noqa: E402


STATUS_COLORS = {
    "FIXED": "#2ecc71",
    "FLOAT": "#f39c12",
    "DGPS": "#3498db",
    "SPP": "#e74c3c",
}

MAX_RENDER_POINTS = 2000


def default_root_dir() -> Path:
    source_root = Path(__file__).resolve().parent.parent
    if (source_root / "output").exists():
        return source_root
    return Path.cwd()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME"),
        description="Serve a local web UI for benchmark artifacts, trajectories, and receiver status.",
    )
    parser.add_argument("--host", default="127.0.0.1", help="Bind host (default: 127.0.0.1).")
    parser.add_argument("--port", type=int, default=8085, help="Bind port (default: 8085, use 0 for auto).")
    parser.add_argument(
        "--port-file",
        type=Path,
        default=None,
        help="Optional file that receives the bound port after startup.",
    )
    parser.add_argument(
        "--root",
        type=Path,
        default=default_root_dir(),
        help="Artifact root directory (default: source tree root or current working directory).",
    )
    parser.add_argument(
        "--lib-pos",
        type=Path,
        default=None,
        help="libgnss++ .pos path (default: <root>/output/rtk_solution.pos).",
    )
    parser.add_argument(
        "--rtklib-pos",
        type=Path,
        default=None,
        help="RTKLIB .pos path (default: <root>/output/driving_rtklib_rtk.pos).",
    )
    parser.add_argument(
        "--odaiba-summary",
        type=Path,
        default=None,
        help="Odaiba summary JSON path (default: <root>/output/odaiba_summary.json).",
    )
    parser.add_argument(
        "--rcv-status",
        type=Path,
        default=None,
        help="Receiver status JSON path exported by gnss rcv.",
    )
    parser.add_argument(
        "--ppc-summary-glob",
        default="output/ppc_*_summary.json",
        help="Glob under --root for PPC summary JSON files.",
    )
    parser.add_argument(
        "--live-summary-glob",
        default="output/live*_summary.json",
        help="Glob under --root for gnss live-signoff summary JSON files.",
    )
    return parser.parse_args()


def resolve_path(explicit: Path | None, root_dir: Path, relative: str) -> Path:
    if explicit is not None:
        return explicit
    return root_dir / relative


def relative_display(path: Path, root_dir: Path) -> str:
    try:
        return str(path.resolve().relative_to(root_dir.resolve()))
    except ValueError:
        return str(path)


def load_json(path: Path) -> dict[str, Any] | None:
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None


def classify_realtime_status(realtime_factor: Any) -> str:
    if not isinstance(realtime_factor, (int, float)):
        return "n/a"
    if realtime_factor >= 1.0:
        return "realtime"
    if realtime_factor >= 0.5:
        return "near-realtime"
    return "offline"


def classify_accuracy_status(p95_h_m: Any) -> str:
    if not isinstance(p95_h_m, (int, float)):
        return "n/a"
    if p95_h_m <= 0.5:
        return "excellent"
    if p95_h_m <= 2.0:
        return "good"
    if p95_h_m <= 10.0:
        return "rough"
    return "poor"


def downsample_points(points: list[dict[str, Any]], limit: int = MAX_RENDER_POINTS) -> list[dict[str, Any]]:
    if len(points) <= limit:
        return points
    stride = max(1, len(points) // limit)
    sampled = points[::stride]
    if sampled[-1] != points[-1]:
        sampled.append(points[-1])
    return sampled


def build_solution_payload(name: str, path: Path, root_dir: Path, solver_label: str) -> dict[str, Any]:
    if not path.exists():
        return {
            "name": name,
            "available": False,
            "path": relative_display(path, root_dir),
            "error": "file not found",
        }

    if name == "libgnsspp":
        epochs = driving_comparison.read_libgnss_pos(path)
    elif name == "rtklib":
        epochs = driving_comparison.read_rtklib_pos(path)
    else:
        raise ValueError(f"unsupported solution name: {name}")

    if not epochs:
        return {
            "name": name,
            "available": False,
            "path": relative_display(path, root_dir),
            "error": "no epochs",
        }

    origin = driving_comparison.ReferenceEpoch(
        week=epochs[0].week,
        tow=epochs[0].tow,
        lat_deg=epochs[0].lat_deg,
        lon_deg=epochs[0].lon_deg,
        height_m=epochs[0].height_m,
        ecef=epochs[0].ecef,
    )
    trajectory = driving_comparison.trajectory_enu(epochs, origin)
    points: list[dict[str, Any]] = []
    status_counts: dict[str, int] = {}
    for epoch, enu in zip(epochs, trajectory):
        status_name, color = driving_comparison.status_style(solver_label, epoch.status)
        status_counts[status_name] = status_counts.get(status_name, 0) + 1
        points.append(
            {
                "tow": round(epoch.tow, 3),
                "east_m": round(float(enu[0]), 3),
                "north_m": round(float(enu[1]), 3),
                "up_m": round(float(enu[2]), 3),
                "status": status_name,
                "color": color,
                "satellites": epoch.num_satellites,
            }
        )

    return {
        "name": name,
        "available": True,
        "path": relative_display(path, root_dir),
        "epoch_count": len(epochs),
        "status_counts": status_counts,
        "points": downsample_points(points),
    }


def build_overview(args: argparse.Namespace) -> dict[str, Any]:
    root_dir = args.root.resolve()
    odaiba_summary_path = resolve_path(args.odaiba_summary, root_dir, "output/odaiba_summary.json")
    lib_pos_path = resolve_path(args.lib_pos, root_dir, "output/rtk_solution.pos")
    rtklib_pos_path = resolve_path(args.rtklib_pos, root_dir, "output/driving_rtklib_rtk.pos")
    rcv_status = load_json(args.rcv_status) if args.rcv_status is not None else None

    ppc_summaries: list[dict[str, Any]] = []
    for path in sorted(root_dir.glob(args.ppc_summary_glob)):
        payload = load_json(path)
        if payload is None:
            continue
        payload = dict(payload)
        payload["_path"] = relative_display(path, root_dir)
        payload["runtime_status"] = classify_realtime_status(payload.get("realtime_factor"))
        payload["quality_status"] = classify_accuracy_status(payload.get("p95_h_m"))
        ppc_summaries.append(payload)

    live_summaries: list[dict[str, Any]] = []
    for path in sorted(root_dir.glob(args.live_summary_glob)):
        payload = load_json(path)
        if payload is None:
            continue
        metrics = payload.get("metrics")
        if not isinstance(metrics, dict):
            continue
        live_summaries.append(
            {
                "_path": relative_display(path, root_dir),
                "execution_mode": payload.get("execution_mode"),
                "exit_code": payload.get("exit_code"),
                "termination": metrics.get("termination"),
                "aligned_epochs": metrics.get("aligned_epochs"),
                "written_solutions": metrics.get("written_solutions"),
                "fixed_solutions": metrics.get("fixed_solutions"),
                "realtime_factor": metrics.get("realtime_factor"),
                "effective_epoch_rate_hz": metrics.get("effective_epoch_rate_hz"),
                "rover_decoder_errors": metrics.get("rover_decoder_errors"),
                "base_decoder_errors": metrics.get("base_decoder_errors"),
                "runtime_status": classify_realtime_status(metrics.get("realtime_factor")),
            }
        )

    return {
        "title": "libgnss++ web",
        "root": str(root_dir),
        "artifacts": {
            "odaiba_summary": relative_display(odaiba_summary_path, root_dir),
            "lib_pos": relative_display(lib_pos_path, root_dir),
            "rtklib_pos": relative_display(rtklib_pos_path, root_dir),
            "rcv_status": relative_display(args.rcv_status, root_dir) if args.rcv_status else None,
        },
        "odaiba_summary": load_json(odaiba_summary_path),
        "ppc_summaries": ppc_summaries,
        "live_summaries": live_summaries,
        "receiver_status": rcv_status,
    }


def render_html() -> str:
    return """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>libgnss++ web</title>
  <style>
    :root {
      --bg: #f4f1e8;
      --paper: #fffdf8;
      --ink: #1f2937;
      --muted: #6b7280;
      --line: #d6d0c4;
      --accent: #0f766e;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: "IBM Plex Sans", "Segoe UI", sans-serif;
      color: var(--ink);
      background:
        radial-gradient(circle at top left, rgba(15,118,110,0.08), transparent 35%),
        linear-gradient(180deg, #f8f5ec 0%, var(--bg) 100%);
    }
    main { max-width: 1280px; margin: 0 auto; padding: 28px 20px 40px; }
    h1, h2, h3 { margin: 0; }
    p { margin: 0; color: var(--muted); }
    .hero { display: grid; gap: 10px; margin-bottom: 24px; }
    .hero h1 { font-size: 2rem; letter-spacing: -0.03em; }
    .chips { display: flex; flex-wrap: wrap; gap: 10px; }
    .chip {
      border: 1px solid var(--line);
      background: rgba(255,255,255,0.75);
      padding: 6px 10px;
      border-radius: 999px;
      font-size: 0.9rem;
      color: var(--muted);
    }
    .grid { display: grid; gap: 18px; }
    .grid.two { grid-template-columns: repeat(auto-fit, minmax(320px, 1fr)); }
    .card {
      background: rgba(255,255,255,0.85);
      border: 1px solid var(--line);
      border-radius: 20px;
      padding: 18px;
      backdrop-filter: blur(8px);
      box-shadow: 0 16px 30px rgba(31,41,55,0.06);
    }
    .metrics {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(140px, 1fr));
      gap: 12px;
      margin-top: 14px;
    }
    .metric {
      background: var(--paper);
      border: 1px solid var(--line);
      border-radius: 14px;
      padding: 12px;
      min-height: 78px;
    }
    .metric .k { font-size: 0.78rem; color: var(--muted); margin-bottom: 6px; }
    .metric .v { font-size: 1.05rem; font-weight: 700; }
    .plot-wrap { display: grid; gap: 12px; }
    .plot-card canvas {
      width: 100%;
      height: 360px;
      border-radius: 14px;
      background: #fff;
      border: 1px solid var(--line);
    }
    .legend { display: flex; flex-wrap: wrap; gap: 10px; margin-top: 10px; }
    .legend-item { display: inline-flex; align-items: center; gap: 8px; font-size: 0.85rem; color: var(--muted); }
    .swatch { width: 12px; height: 12px; border-radius: 999px; border: 1px solid rgba(0,0,0,0.1); }
    .badge {
      display: inline-flex;
      align-items: center;
      padding: 4px 8px;
      border-radius: 999px;
      font-size: 0.78rem;
      font-weight: 700;
      letter-spacing: 0.02em;
      border: 1px solid transparent;
      white-space: nowrap;
    }
    .badge.realtime, .badge.excellent {
      background: rgba(46, 204, 113, 0.14);
      color: #0f7a43;
      border-color: rgba(46, 204, 113, 0.28);
    }
    .badge.near-realtime, .badge.good {
      background: rgba(243, 156, 18, 0.16);
      color: #9a5f00;
      border-color: rgba(243, 156, 18, 0.26);
    }
    .badge.offline, .badge.rough, .badge.poor {
      background: rgba(231, 76, 60, 0.12);
      color: #b03a2e;
      border-color: rgba(231, 76, 60, 0.24);
    }
    .badge.na {
      background: rgba(107, 114, 128, 0.12);
      color: #4b5563;
      border-color: rgba(107, 114, 128, 0.18);
    }
    .status-pre {
      margin-top: 12px;
      padding: 12px;
      border-radius: 14px;
      background: #111827;
      color: #f3f4f6;
      overflow: auto;
      font-size: 0.85rem;
    }
    .section-title { margin-bottom: 12px; display: flex; justify-content: space-between; gap: 12px; align-items: baseline; }
    .tiny { font-size: 0.85rem; color: var(--muted); }
    table {
      width: 100%;
      border-collapse: collapse;
      margin-top: 10px;
      font-size: 0.92rem;
    }
    th, td {
      text-align: left;
      padding: 10px 8px;
      border-bottom: 1px solid var(--line);
    }
    th { color: var(--muted); font-weight: 600; }
  </style>
</head>
<body>
  <main>
    <section class="hero">
      <h1>libgnss++ local web UI</h1>
      <p>Benchmark snapshot, live sign-offs, 2D trajectories, PPC summaries, and receiver status from the existing non-GUI stack.</p>
      <div class="chips" id="artifact-chips"></div>
    </section>

    <section class="grid two">
      <div class="card">
        <div class="section-title">
          <h2>Odaiba snapshot</h2>
          <span class="tiny" id="odaiba-source"></span>
        </div>
        <div class="metrics" id="odaiba-metrics"></div>
      </div>
      <div class="card">
        <div class="section-title">
          <h2>Receiver status</h2>
          <span class="tiny">auto-refresh 2s</span>
        </div>
        <div class="metrics" id="receiver-metrics"></div>
        <pre class="status-pre" id="receiver-json">No receiver status configured.</pre>
      </div>
    </section>

    <section class="card" style="margin-top: 18px;">
      <div class="section-title">
        <h2>2D trajectories</h2>
        <span class="tiny">status-colored ENU points from .pos files</span>
      </div>
      <div class="grid two plot-wrap">
        <div class="plot-card">
          <h3>libgnss++</h3>
          <canvas id="lib-canvas" width="600" height="360"></canvas>
        </div>
        <div class="plot-card">
          <h3>RTKLIB</h3>
          <canvas id="rtk-canvas" width="600" height="360"></canvas>
        </div>
      </div>
      <div class="legend" id="status-legend"></div>
    </section>

    <section class="card" style="margin-top: 18px;">
      <div class="section-title">
        <h2>Live sign-offs</h2>
        <span class="tiny">auto-discovered from output/live*_summary.json</span>
      </div>
      <table id="live-table">
        <thead>
          <tr>
            <th>Path</th>
            <th>Termination</th>
            <th>Written</th>
            <th>Fixed</th>
            <th>Realtime</th>
            <th>Epoch rate</th>
            <th>Decoder errors</th>
          </tr>
        </thead>
        <tbody></tbody>
      </table>
    </section>

    <section class="card" style="margin-top: 18px;">
      <div class="section-title">
        <h2>PPC summaries</h2>
        <span class="tiny">auto-discovered from output/ppc_*_summary.json</span>
      </div>
      <table id="ppc-table">
        <thead>
          <tr>
            <th>Path</th>
            <th>Matched</th>
            <th>Fix rate</th>
            <th>Median H</th>
            <th>P95 H</th>
            <th>Quality</th>
            <th>Realtime</th>
            <th>Wall</th>
            <th>Rate</th>
          </tr>
        </thead>
        <tbody></tbody>
      </table>
    </section>
  </main>
  <script>
    const STATUS_COLORS = {"FIXED":"#2ecc71","FLOAT":"#f39c12","DGPS":"#3498db","SPP":"#e74c3c"};

    async function fetchJson(path) {
      const response = await fetch(path);
      if (!response.ok) {
        const payload = await response.json().catch(() => ({error: response.statusText}));
        throw new Error(payload.error || response.statusText);
      }
      return response.json();
    }

    function renderMetricGrid(target, entries) {
      target.innerHTML = "";
      for (const [key, value] of entries) {
        const el = document.createElement("div");
        el.className = "metric";
        el.innerHTML = `<div class="k">${key}</div><div class="v">${value}</div>`;
        target.appendChild(el);
      }
    }

    function drawTrajectory(canvas, payload) {
      const ctx = canvas.getContext("2d");
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.fillStyle = "#fff";
      ctx.fillRect(0, 0, canvas.width, canvas.height);

      if (!payload.available || !payload.points.length) {
        ctx.fillStyle = "#6b7280";
        ctx.font = "16px IBM Plex Sans, sans-serif";
        ctx.fillText(payload.error || "No data", 24, 40);
        return;
      }

      const pad = 28;
      const xs = payload.points.map((point) => point.east_m);
      const ys = payload.points.map((point) => point.north_m);
      const minX = Math.min(...xs);
      const maxX = Math.max(...xs);
      const minY = Math.min(...ys);
      const maxY = Math.max(...ys);
      const spanX = Math.max(maxX - minX, 1);
      const spanY = Math.max(maxY - minY, 1);
      const scale = Math.min((canvas.width - pad * 2) / spanX, (canvas.height - pad * 2) / spanY);

      const mapX = (value) => pad + (value - minX) * scale;
      const mapY = (value) => canvas.height - pad - (value - minY) * scale;

      ctx.strokeStyle = "#d1d5db";
      ctx.lineWidth = 1.5;
      ctx.beginPath();
      payload.points.forEach((point, index) => {
        const x = mapX(point.east_m);
        const y = mapY(point.north_m);
        if (index === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      });
      ctx.stroke();

      payload.points.forEach((point) => {
        ctx.fillStyle = point.color || STATUS_COLORS[point.status] || "#64748b";
        ctx.beginPath();
        ctx.arc(mapX(point.east_m), mapY(point.north_m), 2.4, 0, Math.PI * 2);
        ctx.fill();
      });

      ctx.fillStyle = "#6b7280";
      ctx.font = "12px IBM Plex Sans, sans-serif";
      ctx.fillText(`epochs: ${payload.epoch_count}`, 14, 18);
      ctx.fillText(payload.path, 14, canvas.height - 10);
    }

    function renderLegend() {
      const legend = document.getElementById("status-legend");
      legend.innerHTML = "";
      for (const [name, color] of Object.entries(STATUS_COLORS)) {
        const item = document.createElement("div");
        item.className = "legend-item";
        item.innerHTML = `<span class="swatch" style="background:${color}"></span>${name}`;
        legend.appendChild(item);
      }
    }

    function formatMaybeNumber(value, digits = 2, suffix = "") {
      if (value === null || value === undefined) return "n/a";
      if (typeof value === "number") return `${value.toFixed(digits)}${suffix}`;
      return String(value);
    }

    function renderBadge(status) {
      const normalized = (status || "n/a");
      const className = normalized === "n/a" ? "na" : normalized;
      return `<span class="badge ${className}">${normalized}</span>`;
    }

    async function refreshStatus() {
      const metrics = document.getElementById("receiver-metrics");
      const pre = document.getElementById("receiver-json");
      try {
        const payload = await fetchJson("/api/status");
        if (!payload.available) {
          renderMetricGrid(metrics, [["state", "n/a"], ["path", "not configured"]]);
          pre.textContent = payload.message || "No receiver status configured.";
          return;
        }
        renderMetricGrid(metrics, [
          ["state", payload.state || "unknown"],
          ["pid", payload.pid_running ? `running (${payload.pid || "n/a"})` : "stopped"],
          ["uptime", payload.uptime_seconds !== null ? `${payload.uptime_seconds.toFixed(1)} s` : "n/a"],
          ["restarts", payload.restart_count ?? 0],
        ]);
        pre.textContent = JSON.stringify(payload.raw, null, 2);
      } catch (error) {
        renderMetricGrid(metrics, [["state", "error"]]);
        pre.textContent = String(error);
      }
    }

    async function init() {
      renderLegend();
      const overview = await fetchJson("/api/overview");
      const chips = document.getElementById("artifact-chips");
      for (const [key, value] of Object.entries(overview.artifacts)) {
        if (!value) continue;
        const chip = document.createElement("div");
        chip.className = "chip";
        chip.textContent = `${key}: ${value}`;
        chips.appendChild(chip);
      }

      const odaiba = overview.odaiba_summary || {};
      document.getElementById("odaiba-source").textContent = overview.artifacts.odaiba_summary || "";
      renderMetricGrid(document.getElementById("odaiba-metrics"), [
        ["lib epochs", odaiba.all_epochs?.libgnsspp?.epochs ?? "n/a"],
        ["RTKLIB epochs", odaiba.all_epochs?.rtklib?.epochs ?? "n/a"],
        ["lib fix rate", formatMaybeNumber(odaiba.all_epochs?.libgnsspp?.fix_rate_pct, 2, "%")],
        ["RTKLIB fix rate", formatMaybeNumber(odaiba.all_epochs?.rtklib?.fix_rate_pct, 2, "%")],
        ["common median H", formatMaybeNumber(odaiba.common_epochs?.libgnsspp?.median_h_m, 3, " m")],
        ["common p95 H", formatMaybeNumber(odaiba.common_epochs?.libgnsspp?.p95_h_m, 2, " m")],
      ]);

      const liveBody = document.querySelector("#live-table tbody");
      liveBody.innerHTML = "";
      for (const row of overview.live_summaries || []) {
        const tr = document.createElement("tr");
        tr.innerHTML = `
          <td>${row._path || "n/a"}</td>
          <td>${row.termination || "n/a"}</td>
          <td>${row.written_solutions ?? "n/a"}</td>
          <td>${row.fixed_solutions ?? "n/a"}</td>
          <td>${renderBadge(row.runtime_status)} ${formatMaybeNumber(row.realtime_factor, 2, "x")}</td>
          <td>${formatMaybeNumber(row.effective_epoch_rate_hz, 2, " Hz")}</td>
          <td>${(row.rover_decoder_errors ?? "n/a")}/${(row.base_decoder_errors ?? "n/a")}</td>`;
        liveBody.appendChild(tr);
      }

      const ppcBody = document.querySelector("#ppc-table tbody");
      ppcBody.innerHTML = "";
      for (const row of overview.ppc_summaries || []) {
        const tr = document.createElement("tr");
        tr.innerHTML = `
          <td>${row._path || "n/a"}</td>
          <td>${row.matched_epochs ?? "n/a"}</td>
          <td>${formatMaybeNumber(row.fix_rate_pct, 2, "%")}</td>
          <td>${formatMaybeNumber(row.median_h_m, 3, " m")}</td>
          <td>${formatMaybeNumber(row.p95_h_m, 2, " m")}</td>
          <td>${renderBadge(row.quality_status)}</td>
          <td>${renderBadge(row.runtime_status)} ${formatMaybeNumber(row.realtime_factor, 2, "x")}</td>
          <td>${formatMaybeNumber(row.solver_wall_time_s, 2, " s")}</td>
          <td>${formatMaybeNumber(row.effective_epoch_rate_hz, 2, " Hz")}</td>`;
        ppcBody.appendChild(tr);
      }

      drawTrajectory(document.getElementById("lib-canvas"), await fetchJson("/api/solution?name=libgnsspp"));
      drawTrajectory(document.getElementById("rtk-canvas"), await fetchJson("/api/solution?name=rtklib"));
      await refreshStatus();
      setInterval(refreshStatus, 2000);
    }

    init().catch((error) => {
      document.body.innerHTML = `<main><pre class="status-pre">${String(error)}</pre></main>`;
    });
  </script>
</body>
</html>
"""


def make_handler(args: argparse.Namespace):
    root_dir = args.root.resolve()
    lib_pos_path = resolve_path(args.lib_pos, root_dir, "output/rtk_solution.pos")
    rtklib_pos_path = resolve_path(args.rtklib_pos, root_dir, "output/driving_rtklib_rtk.pos")

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt: str, *values: Any) -> None:
            return

        def _write(self, body: bytes, content_type: str, status: HTTPStatus = HTTPStatus.OK) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _write_json(self, payload: dict[str, Any], status: HTTPStatus = HTTPStatus.OK) -> None:
            self._write(json.dumps(payload, indent=2, sort_keys=True).encode("utf-8"),
                        "application/json; charset=utf-8", status)

        def do_GET(self) -> None:
            parsed = urlparse(self.path)
            query = parse_qs(parsed.query)

            try:
                if parsed.path == "/":
                    self._write(render_html().encode("utf-8"), "text/html; charset=utf-8")
                    return
                if parsed.path == "/api/health":
                    self._write_json({"ok": True})
                    return
                if parsed.path == "/api/overview":
                    self._write_json(build_overview(args))
                    return
                if parsed.path == "/api/status":
                    if args.rcv_status is None:
                        self._write_json({"available": False, "message": "No --rcv-status configured."})
                        return
                    raw = load_json(args.rcv_status)
                    if raw is None:
                        self._write_json(
                            {
                                "available": False,
                                "message": f"Status file unavailable: {args.rcv_status}",
                            }
                        )
                        return
                    self._write_json(
                        {
                            "available": True,
                            "state": raw.get("state"),
                            "pid": raw.get("pid"),
                            "pid_running": raw.get("pid_running"),
                            "uptime_seconds": raw.get("uptime_seconds"),
                            "restart_count": raw.get("restart_count"),
                            "raw": raw,
                        }
                    )
                    return
                if parsed.path == "/api/solution":
                    name = query.get("name", [""])[0]
                    if name == "libgnsspp":
                        self._write_json(build_solution_payload(name, lib_pos_path, root_dir, "libgnss++"))
                        return
                    if name == "rtklib":
                        self._write_json(build_solution_payload(name, rtklib_pos_path, root_dir, "RTKLIB"))
                        return
                    self._write_json({"error": "unknown solution name"}, HTTPStatus.BAD_REQUEST)
                    return

                self._write_json({"error": "not found"}, HTTPStatus.NOT_FOUND)
            except Exception as exc:  # pragma: no cover - tested through API failure shape
                self._write_json({"error": str(exc)}, HTTPStatus.INTERNAL_SERVER_ERROR)

    return Handler


def main() -> int:
    args = parse_args()
    server = ThreadingHTTPServer((args.host, args.port), make_handler(args))
    shutdown_started = False
    previous_handlers: dict[int, Any] = {}

    def request_shutdown(signum: int, _frame: Any) -> None:
        nonlocal shutdown_started
        if shutdown_started:
            return
        shutdown_started = True
        threading.Thread(target=server.shutdown, daemon=True).start()

    for sig in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[sig] = signal.getsignal(sig)
        signal.signal(sig, request_shutdown)

    bound_host, bound_port = server.server_address[:2]
    if args.port_file is not None:
        args.port_file.parent.mkdir(parents=True, exist_ok=True)
        args.port_file.write_text(f"{bound_port}\n", encoding="utf-8")
    print(f"Serving gnss web at http://{bound_host}:{bound_port}", flush=True)
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
