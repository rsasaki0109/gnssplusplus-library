#!/usr/bin/env python3
"""Cross-platform dispatcher for the libgnss++ command suite."""

from __future__ import annotations

import glob
import os
import sys


ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
APPS_DIR = os.path.dirname(os.path.abspath(__file__))
EXE_SUFFIX = ".exe" if os.name == "nt" else ""
BUILD_CONFIGS = ("Release", "RelWithDebInfo", "Debug", "MinSizeRel")

COMMANDS = {
    "spp": {
        "kind": "binary",
        "target": "gnss_spp",
        "summary": "Batch SPP post-processing from rover/nav RINEX files.",
    },
    "solve": {
        "kind": "binary",
        "target": "gnss_solve",
        "summary": "Batch RTK post-processing from rover/base/nav RINEX files.",
    },
    "ppp": {
        "kind": "binary",
        "target": "gnss_ppp",
        "summary": "Batch PPP post-processing from rover RINEX plus precise SP3/CLK products.",
    },
    "visibility": {
        "kind": "binary",
        "target": "gnss_visibility",
        "summary": "Analyze satellite visibility from rover/nav RINEX into azimuth/elevation/SNR CSV and summary JSON.",
    },
    "visibility-plot": {
        "kind": "python",
        "target": os.path.join(APPS_DIR, "gnss_visibility_plot.py"),
        "summary": "Render a visibility CSV into a polar/elevation PNG quick-look.",
    },
    "nav-products": {
        "kind": "binary",
        "target": "gnss_nav_products",
        "summary": "Generate simple SP3/CLK precise-product files from observed epochs plus broadcast nav.",
    },
    "fetch-products": {
        "kind": "python",
        "target": os.path.join(APPS_DIR, "gnss_fetch_products.py"),
        "summary": "Fetch and cache SP3/CLK/IONEX/DCB-style product files from local paths or URLs.",
    },
    "artifact-manifest": {
        "kind": "python",
        "target": os.path.join(APPS_DIR, "gnss_artifact_manifest.py"),
        "summary": "Scan generated summaries and build a web/CI-friendly artifact manifest JSON.",
    },
    "rinex-info": {
        "kind": "binary",
        "target": "gnss_rinex_info",
        "summary": "Inspect RINEX headers and optionally count epochs or ephemerides.",
    },
    "stream": {
        "kind": "binary",
        "target": "gnss_stream",
        "summary": "Read and relay RTCM from file, NTRIP, or serial sources with optional decode summaries and relay sinks.",
    },
    "ubx-info": {
        "kind": "binary",
        "target": "gnss_ubx_info",
        "summary": "Inspect UBX NAV/RAWX/SFRBX files or serial streams and optionally export RAWX epochs to RINEX.",
    },
    "ionex-info": {
        "kind": "python",
        "target": os.path.join(APPS_DIR, "gnss_ionex_info.py"),
        "summary": "Inspect IONEX headers, map counts, grid metadata, and auxiliary DCB blocks.",
    },
    "dcb-info": {
        "kind": "python",
        "target": os.path.join(APPS_DIR, "gnss_dcb_info.py"),
        "summary": "Inspect Bias-SINEX or IONEX auxiliary DCB products and summarize systems/bias types.",
    },
    "nmea-info": {
        "kind": "python",
        "target": os.path.join(APPS_DIR, "gnss_nmea_info.py"),
        "summary": "Inspect NMEA GGA/RMC logs or serial streams and print decoded position summaries.",
    },
    "novatel-info": {
        "kind": "python",
        "target": os.path.join(APPS_DIR, "gnss_novatel_info.py"),
        "summary": "Inspect NovAtel ASCII BESTPOS/BESTVEL logs or serial streams.",
    },
    "sbp-info": {
        "kind": "python",
        "target": os.path.join(APPS_DIR, "gnss_sbp_info.py"),
        "summary": "Inspect Swift Binary Protocol GPS_TIME/POS_LLH/VEL_NED logs or serial streams.",
    },
    "sbf-info": {
        "kind": "python",
        "target": os.path.join(APPS_DIR, "gnss_sbf_info.py"),
        "summary": "Inspect Septentrio SBF PVTGeodetic/LBandTrackerStatus/P2PPStatus logs or serial streams.",
    },
    "trimble-info": {
        "kind": "python",
        "target": os.path.join(APPS_DIR, "gnss_trimble_info.py"),
        "summary": "Inspect Trimble GSOF GENOUT Type 1/2/8 packets from file or serial input.",
    },
    "skytraq-info": {
        "kind": "python",
        "target": os.path.join(APPS_DIR, "gnss_skytraq_info.py"),
        "summary": "Inspect SkyTraq binary epoch/raw/rawx/ack logs or serial streams.",
    },
    "binex-info": {
        "kind": "python",
        "target": os.path.join(APPS_DIR, "gnss_binex_info.py"),
        "summary": "Inspect BINEX big-endian regular-CRC metadata/navigation/prototyping records from file or serial input.",
    },
    "qzss-l6-info": {
        "kind": "python",
        "target": os.path.join(APPS_DIR, "gnss_qzss_l6_info.py"),
        "summary": "Inspect direct QZSS L6 250-byte frames and optionally export subframes, subtype 10 service-info packets, Compact SSR message inventory, or sampled corrections from subtype 1/2/3/4/5/6/7/8/9/11/12.",
    },
    "convert": {
        "kind": "binary",
        "target": "gnss_convert",
        "summary": "Convert RTCM or UBX input into simple RINEX files and export UBX SFRBX frame metadata.",
    },
    "replay": {
        "kind": "binary",
        "target": "gnss_replay",
        "summary": "Replay rover/base observations from RINEX, UBX, or RTCM through the RTK solver.",
    },
    "live": {
        "kind": "binary",
        "target": "gnss_live",
        "summary": "Continuously solve RTCM or UBX rover input against RTCM base corrections into a live RTK solution file.",
    },
    "live-signoff": {
        "kind": "python",
        "target": os.path.join(APPS_DIR, "gnss_live_signoff.py"),
        "summary": "Run gnss live with realtime/error-handling thresholds and emit summary JSON.",
    },
    "moving-base-signoff": {
        "kind": "python",
        "target": os.path.join(APPS_DIR, "gnss_moving_base_signoff.py"),
        "summary": "Run a real moving-base replay/live dataset against reference baseline/heading CSV and emit summary JSON.",
    },
    "scorpion-moving-base-signoff": {
        "kind": "python",
        "target": os.path.join(APPS_DIR, "gnss_scorpion_moving_base_signoff.py"),
        "summary": "Prepare and validate the public SCORPION moving-base ROS2 bag through replay plus BRDC nav fetch.",
    },
    "moving-base-prepare": {
        "kind": "python",
        "target": os.path.join(APPS_DIR, "gnss_moving_base_prepare.py"),
        "summary": "Extract rover/base UBX files plus reference CSV from a ROS2 moving-base bag with u-blox topics.",
    },
    "moving-base-plot": {
        "kind": "python",
        "target": os.path.join(APPS_DIR, "gnss_moving_base_plot.py"),
        "summary": "Render moving-base sign-off solution/reference pairs into a baseline and heading PNG quick-look.",
    },
    "rcv": {
        "kind": "python",
        "target": os.path.join(APPS_DIR, "gnss_rcv.py"),
        "summary": "Run the live solver from an rtkrcv-style config file and emit status snapshots.",
    },
    "web": {
        "kind": "python",
        "target": os.path.join(APPS_DIR, "gnss_web.py"),
        "summary": "Serve a local web UI for benchmark snapshots, live sign-offs, 2D trajectories, and receiver status.",
    },
    "stats": {
        "kind": "python",
        "target": os.path.join(ROOT_DIR, "tools", "rtk_stats.py"),
        "summary": "Show text statistics for a .pos solution file.",
    },
    "compare": {
        "kind": "python",
        "target": os.path.join(ROOT_DIR, "tools", "compare_rtklib.py"),
        "summary": "Compare libgnss++ output against RTKLIB output.",
    },
    "plot": {
        "kind": "python",
        "target": os.path.join(ROOT_DIR, "tools", "plot_rtk.py"),
        "summary": "Generate ENU/time-series plots for one or two .pos files.",
    },
    "trackplot": {
        "kind": "python",
        "target": os.path.join(ROOT_DIR, "tools", "plot_trajectory.py"),
        "summary": "Generate a 2D trajectory comparison plot with status colors.",
    },
    "rtklib2pos": {
        "kind": "python",
        "target": os.path.join(ROOT_DIR, "scripts", "convert_rtklib_pos.py"),
        "summary": "Convert raw RTKLIB .pos output into libgnss++ .pos format.",
    },
    "pos2kml": {
        "kind": "python",
        "target": os.path.join(APPS_DIR, "gnss_pos2kml"),
        "summary": "Convert libgnss++ or RTKLIB .pos output into KML.",
    },
    "driving-compare": {
        "kind": "python",
        "target": os.path.join(ROOT_DIR, "scripts", "generate_driving_comparison.py"),
        "summary": "Generate the UrbanNav/Odaiba comparison figure from solution files.",
    },
    "scorecard": {
        "kind": "python",
        "target": os.path.join(ROOT_DIR, "scripts", "generate_odaiba_scorecard.py"),
        "summary": "Generate the Odaiba benchmark scorecard image.",
    },
    "social-card": {
        "kind": "python",
        "target": os.path.join(ROOT_DIR, "scripts", "generate_odaiba_social_card.py"),
        "summary": "Generate a Twitter-ready Odaiba social card image.",
    },
    "architecture-card": {
        "kind": "python",
        "target": os.path.join(ROOT_DIR, "scripts", "generate_architecture_diagram.py"),
        "summary": "Generate a docs-friendly architecture diagram image.",
    },
    "odaiba-benchmark": {
        "kind": "python",
        "target": os.path.join(APPS_DIR, "gnss_odaiba_benchmark.py"),
        "summary": "Run the full libgnss++ vs RTKLIB Odaiba benchmark pipeline.",
    },
    "odaiba-scan": {
        "kind": "python",
        "target": os.path.join(APPS_DIR, "gnss_odaiba_scan.py"),
        "summary": "Scan Odaiba in epoch windows and report reference-based metrics.",
    },
    "short-baseline-signoff": {
        "kind": "python",
        "target": os.path.join(APPS_DIR, "gnss_short_baseline_signoff.py"),
        "summary": "Run a mixed-GNSS short-baseline static sign-off and emit summary JSON.",
    },
    "rtk-kinematic-signoff": {
        "kind": "python",
        "target": os.path.join(APPS_DIR, "gnss_rtk_kinematic_signoff.py"),
        "summary": "Run the bundled mixed-GNSS RTK kinematic sign-off and emit summary JSON.",
    },
    "ppp-static-signoff": {
        "kind": "python",
        "target": os.path.join(APPS_DIR, "gnss_ppp_static_signoff.py"),
        "summary": "Run the bundled static PPP sign-off and emit summary JSON.",
    },
    "ppp-kinematic-signoff": {
        "kind": "python",
        "target": os.path.join(APPS_DIR, "gnss_ppp_kinematic_signoff.py"),
        "summary": "Run the bundled kinematic PPP sign-off against an RTK reference and emit summary JSON.",
    },
    "ppp-products-signoff": {
        "kind": "python",
        "target": os.path.join(APPS_DIR, "gnss_ppp_products_signoff.py"),
        "summary": "Run static, kinematic, or PPC PPP sign-off with fetched SP3/CLK/IONEX/DCB products and emit summary JSON.",
    },
    "ppc-demo": {
        "kind": "python",
        "target": os.path.join(APPS_DIR, "gnss_ppc_demo.py"),
        "summary": "Run an external PPC-Dataset Tokyo/Nagoya run through RTK or PPP and compare against reference.csv.",
    },
    "ppc-rtk-signoff": {
        "kind": "python",
        "target": os.path.join(APPS_DIR, "gnss_ppc_rtk_signoff.py"),
        "summary": "Run the PPC-Dataset RTK sign-off profile for Tokyo/Nagoya, with optional RTKLIB side-by-side gates.",
    },
    "clas-ppp": {
        "kind": "python",
        "target": os.path.join(APPS_DIR, "gnss_clas_ppp.py"),
        "summary": "Run PPP with a named CLAS/MADOCA correction profile over RTCM, compact sampled transport, or raw QZSS L6 subtype 1/2/3/4/5/6/7/8/9/11/12 input.",
    },
    "ros2-solution-node": {
        "kind": "binary",
        "target": "gnss_solution_node",
        "summary": "Publish a .pos solution file as ROS2 NavSatFix, PoseStamped, Path, status, and satellite-count topics.",
    },
}


def usage() -> str:
    lines = [
        "Usage: gnss <command> [args...]",
        "",
        "Commands:",
    ]
    for name in sorted(COMMANDS):
        lines.append(f"  {name:<10} {COMMANDS[name]['summary']}")
    lines.extend(
        [
            "",
            "Examples:",
            "  python3 apps/gnss.py solve --data-dir data/driving --out output/rtk_solution.pos",
            "  python3 apps/gnss.py spp --obs data/rover_static.obs --nav data/navigation_static.nav --out output/spp_solution.pos",
            "  python3 apps/gnss.py visibility --obs data/rover_static.obs --nav data/navigation_static.nav --csv output/visibility.csv --summary-json output/visibility.json --max-epochs 60",
            "  python3 apps/gnss.py visibility-plot output/visibility.csv output/visibility.png",
            "  python3 apps/gnss.py solve --rover data/rover_kinematic.obs --base data/base_kinematic.obs --nav data/navigation_kinematic.nav --iono est --max-epochs 5",
            "  python3 apps/gnss.py ppp --static --obs data/rover_static.obs --nav data/navigation_static.nav --sp3 precise.sp3 --clk precise.clk --antex igs20.atx --blq station.blq --out output/ppp_solution.pos",
            "  python3 apps/gnss.py ppp --kinematic --enable-ar --convergence-min-epochs 4 --ar-ratio-threshold 2.0 --obs rover.obs --sp3 precise.sp3 --clk precise.clk --out output/ppp_ar.pos",
            "  python3 apps/gnss.py nav-products --obs data/rover_static.obs --nav data/navigation_static.nav --sp3-out output/static_products.sp3 --clk-out output/static_products.clk --max-epochs 60",
            "  python3 apps/gnss.py fetch-products --date 2024-01-02 --product sp3=https://example.net/{yyyy}{doy}.sp3.gz --product clk=file:///data/{yyyy}{doy}.clk.gz --summary-json output/products.json",
            "  python3 apps/gnss.py artifact-manifest --root . --output output/artifact_manifest.json",
            "  python3 apps/gnss.py stream --input ntrip://user:pass@caster:2101/MOUNT --limit 10",
            "  python3 apps/gnss.py stream --input tcp://127.0.0.1:9000 --limit 10",
            "  python3 apps/gnss.py stream --input correction.rtcm3 --output serial:///dev/ttyUSB1?baud=115200 --limit 10",
            "  python3 apps/gnss.py stream --input correction.rtcm3 --output tcp://127.0.0.1:9000 --limit 10",
            "  python3 apps/gnss.py web --port 8085 --rcv-status output/receiver.status.json",
            "  python3 apps/gnss.py ubx-info --input logs/session.ubx --decode-observations",
            "  python3 apps/gnss.py ubx-info --input serial:///dev/ttyACM0?baud=115200 --limit 10",
            "  python3 apps/gnss.py ionex-info --input products/codg0020.24i --summary-json output/ionex.json",
            "  python3 apps/gnss.py dcb-info --input products/CAS0MGXRAP_20240020000_01D_01D_DCB.BSX --summary-json output/dcb.json",
            "  python3 apps/gnss.py nmea-info --input logs/session.nmea --decode-gga --decode-rmc",
            "  python3 apps/gnss.py nmea-info --input serial:///dev/ttyUSB0?baud=9600 --limit 10",
            "  python3 apps/gnss.py novatel-info --input logs/novatel.log --decode-bestpos --decode-bestvel",
            "  python3 apps/gnss.py novatel-info --input serial:///dev/ttyUSB0?baud=115200 --limit 10",
            "  python3 apps/gnss.py sbp-info --input logs/session.sbp --decode-time --decode-pos --decode-vel",
            "  python3 apps/gnss.py sbp-info --input serial:///dev/ttyUSB0?baud=115200 --limit 10",
            "  python3 apps/gnss.py sbf-info --input logs/session.sbf --decode-pvt --decode-lband --decode-p2pp",
            "  python3 apps/gnss.py sbf-info --input serial:///dev/ttyUSB0?baud=115200 --limit 10",
            "  python3 apps/gnss.py trimble-info --input logs/session.gsof --decode-time --decode-llh --decode-vel",
            "  python3 apps/gnss.py trimble-info --input serial:///dev/ttyUSB0?baud=115200 --limit 10",
            "  python3 apps/gnss.py skytraq-info --input logs/session.stq --decode-epoch --decode-raw --decode-ack",
            "  python3 apps/gnss.py skytraq-info --input serial:///dev/ttyUSB0?baud=115200 --limit 10",
            "  python3 apps/gnss.py binex-info --input logs/session.bnx --decode-metadata --decode-nav --decode-proto",
            "  python3 apps/gnss.py binex-info --input serial:///dev/ttyUSB0?baud=115200 --limit 10",
            "  python3 apps/gnss.py qzss-l6-info --input logs/qzss_l6.bin --show-preview --extract-data-parts output/qzss_l6_frames.csv --extract-subframes output/qzss_l6_subframes.csv --extract-compact-messages output/qzss_l6_messages.csv",
            "  python3 apps/gnss.py qzss-l6-info --input serial:///dev/ttyUSB0?baud=115200 --limit 10",
            "  python3 apps/gnss.py convert --format ubx --input logs/session.ubx --obs-out output/session.obs",
            "  python3 apps/gnss.py convert --format ubx --input logs/session.ubx --nav-out output/session.nav",
            "    # exports GPS/QZSS/Galileo/GLONASS/BeiDou broadcast nav from RXM-SFRBX when available",
            "  python3 apps/gnss.py convert --format ubx --input logs/session.ubx --sfrbx-out output/session_sfrbx.csv",
            "  python3 apps/gnss.py social-card --lib-pos output/rtk_solution.pos --rtklib-pos output/driving_rtklib_rtk.pos --reference-csv data/driving/Tokyo_Data/Odaiba/reference.csv --output docs/driving_odaiba_social_card.png",
            "  python3 apps/gnss.py architecture-card --output docs/libgnsspp_architecture.png",
            "  python3 apps/gnss.py convert --format rtcm --input tcp://127.0.0.1:9000 --obs-out output/correction.obs",
            "  python3 apps/gnss.py convert --format ubx --input serial:///dev/ttyACM0?baud=115200 --obs-out output/stream.obs --limit 10",
            "  python3 apps/gnss.py replay --rover-rinex data/rover_kinematic.obs --base-rinex data/base_kinematic.obs --nav-rinex data/navigation_kinematic.nav --out output/replay.pos",
            "  python3 apps/gnss.py live --rover-rtcm rover.rtcm3 --base-rtcm base.rtcm3 --nav-rinex nav.rnx",
            "  python3 apps/gnss.py live-signoff --rover-rtcm rover.rtcm3 --base-rtcm base.rtcm3 --out output/live.pos --summary-json output/live_summary.json --require-written-solutions-min 3 --require-realtime-factor-min 1.0",
            "  python3 apps/gnss.py rcv start --config configs/live.conf --status-out output/receiver_status.json",
            "  python3 apps/gnss.py rcv status --status-out output/receiver_status.json --wait-seconds 5",
            "  python3 apps/gnss.py rcv status --status-out output/receiver_status.json --tail-log-lines 20",
            "  python3 apps/gnss.py rcv reload --status-out output/receiver_status.json --wait-seconds 1",
            "  python3 apps/gnss.py stats output/rtk_solution.pos",
            "  python3 apps/gnss.py compare output/rtk_solution.pos output/driving_rtklib_rtk.pos",
            "  python3 apps/gnss.py odaiba-benchmark --rtklib-bin /path/to/rnx2rtkp",
            "  python3 apps/gnss.py odaiba-benchmark --rtklib-bin /path/to/rnx2rtkp --malib-bin /path/to/malib/rnx2rtkp",
            "  python3 apps/gnss.py odaiba-benchmark --rtklib-bin /path/to/rnx2rtkp --require-all-epochs-min 11000 --require-common-epoch-pairs-min 8000 --require-lib-all-p95-h-max 8.0 --require-lib-common-median-h-max 0.8 --require-lib-common-p95-h-max 6.5",
            "  python3 apps/gnss.py odaiba-scan --glonass-ar autocal --window-size 1000",
            "  python3 apps/gnss.py short-baseline-signoff --max-epochs 120 --require-fix-rate-min 95 --require-mean-error-max 0.15 --require-max-error-max 0.6 --require-mean-sats-min 14",
            "  python3 apps/gnss.py rtk-kinematic-signoff --max-epochs 120 --require-valid-epochs-min 120 --require-fix-rate-min 95 --require-mean-error-max 3.0 --require-max-error-max 3.0 --require-mean-sats-min 25",
            "  python3 apps/gnss.py ppp-static-signoff --max-epochs 120 --require-valid-epochs-min 120 --require-mean-error-max 1.5 --require-max-error-max 1.5 --require-mean-sats-min 6.0 --require-ppp-solution-rate-min 100",
            "  python3 apps/gnss.py ppp-static-signoff --enable-ar --generate-products --ar-ratio-threshold 1.5 --require-mean-error-max 5.0 --require-max-error-max 6.0 --require-ppp-fixed-epochs-min 1 --require-ppp-solution-rate-min 100",
            "  python3 apps/gnss.py ppp-kinematic-signoff --max-epochs 120 --require-common-epoch-pairs-min 120 --require-reference-fix-rate-min 95 --require-converged --require-convergence-time-max 300 --require-mean-error-max 7.0 --require-p95-error-max 7.0 --require-max-error-max 7.0 --require-mean-sats-min 18 --require-ppp-solution-rate-min 100",
            "  python3 apps/gnss.py ppc-demo --dataset-root /datasets/PPC-Dataset --city tokyo --run run1 --solver rtk --require-realtime-factor-min 1.0 --summary-json output/ppc_tokyo_run1_rtk_summary.json",
            "  python3 apps/gnss.py ppc-rtk-signoff --dataset-root /datasets/PPC-Dataset --city tokyo --rtklib-bin /path/to/rnx2rtkp",
            "  python3 apps/gnss.py clas-ppp --profile madoca --obs data/rover_static.obs --nav data/navigation_static.nav --ssr-rtcm ntrip://caster/MOUNT --out output/madoca_ppp.pos --summary-json output/madoca_ppp_summary.json",
            "  python3 apps/gnss.py clas-ppp --profile clas --obs data/rover_static.obs --nav data/navigation_static.nav --compact-ssr corrections.compact.csv --out output/clas_ppp.pos --summary-json output/clas_ppp_summary.json",
            "  python3 apps/gnss.py clas-ppp --profile clas --obs data/rover_static.obs --nav data/navigation_static.nav --qzss-l6 logs/qzss_l6.bin --qzss-gps-week 2200 --out output/clas_l6.pos --summary-json output/clas_l6_summary.json",
            "  python3 apps/gnss.py ros2-solution-node --ros-args -p solution_file:=output/rtk_solution.pos -p max_messages:=1",
            "",
            "On Windows, use: py apps\\gnss.py <command> ...",
        ]
    )
    return "\n".join(lines)


def find_binary(target_name: str) -> str | None:
    filename = target_name + EXE_SUFFIX
    candidates = [
        os.path.join(APPS_DIR, filename),
        os.path.join(ROOT_DIR, "build", "apps", filename),
    ]
    for config in BUILD_CONFIGS:
        candidates.append(os.path.join(ROOT_DIR, "build", "apps", config, filename))
        candidates.append(os.path.join(ROOT_DIR, "build", config, "apps", filename))
        candidates.append(os.path.join(ROOT_DIR, "build", config, filename))

    for candidate in candidates:
        if os.path.isfile(candidate):
            return candidate

    recursive_hits = sorted(
        path for path in glob.glob(os.path.join(ROOT_DIR, "build", "**", filename), recursive=True)
        if os.path.isfile(path)
    )
    return recursive_hits[0] if recursive_hits else None


def run_python(target: str, command_name: str, args: list[str]) -> int:
    env = os.environ.copy()
    env["GNSS_CLI_NAME"] = f"gnss {command_name}"
    try:
        os.execvpe(sys.executable, [sys.executable, target, *args], env)
    except OSError as exc:
        print(f"Error: failed to exec {target}: {exc}", file=sys.stderr)
        return 1


def run_binary(target_name: str, args: list[str]) -> int:
    binary = find_binary(target_name)
    if binary is None:
        print(
            f"Error: built binary not found for {target_name}. "
            f"Run `cmake --build build --target {target_name}` first.",
            file=sys.stderr,
        )
        return 1
    try:
        os.execv(binary, [binary, *args])
    except OSError as exc:
        print(f"Error: failed to exec {binary}: {exc}", file=sys.stderr)
        return 1


def main() -> int:
    if len(sys.argv) < 2 or sys.argv[1] in ("-h", "--help", "help"):
        print(usage())
        return 0 if len(sys.argv) >= 2 else 1

    command_name = sys.argv[1]
    command = COMMANDS.get(command_name)
    if command is None:
        print(f"Error: unknown command `{command_name}`.\n", file=sys.stderr)
        print(usage(), file=sys.stderr)
        return 1

    args = sys.argv[2:]
    if command["kind"] == "python":
        return run_python(command["target"], command_name, args)
    return run_binary(command["target"], args)


if __name__ == "__main__":
    raise SystemExit(main())
