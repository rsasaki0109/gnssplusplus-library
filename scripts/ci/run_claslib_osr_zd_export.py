#!/usr/bin/env python3
"""Generate a normalized CLASLIB OSR ZD dump for CLAS A4b CI diffing."""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime, timedelta
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
from typing import Mapping, Sequence


sys.path.insert(0, str(Path(__file__).resolve().parent))

from optional_diff_runner import artifact_record, truthy  # noqa: E402


SUMMARY_SCHEMA = "ci_claslib_osr_zd_export.v4"
CONTRACT = "claslib_osr_zd_export.v4"
DEFAULT_CLASLIB_REPO = "https://github.com/QZSS-Strategy-Office/claslib.git"
DEFAULT_CLASLIB_REF = "23cfd363a2db6d8d8144e292c82e9d97ca2d3015"
DEFAULT_GPS_WEEK = 2068
DEFAULT_MAX_EPOCHS = 300
DEFAULT_START_DATE = "2019/08/27"
DEFAULT_START_TIME = "16:00:00"
REQUIRED_DATA_FILES = (
    "0627239Q.obs",
    "sept_2019239.nav",
    "2019239Q.l6",
    "igs14_L5copy.atx",
    "clas_grid.def",
    "clas_grid.blq",
    "igu00p01.erp",
    "isb.tbl",
    "l2csft.tbl",
)


@dataclass(frozen=True)
class RunPaths:
    output_dir: Path
    work_dir: Path
    log_path: Path
    summary_json: Path
    checkout_dir: Path
    instrumented_source_dir: Path
    claslib_config: Path
    claslib_solution: Path
    claslib_osr: Path
    normalized_csv: Path
    normalized_summary_json: Path
    claslib_dd_dump: Path
    claslib_state_dump: Path


@dataclass(frozen=True)
class RunConfig:
    repo_root: Path
    output_dir: Path
    source_root: Path | None
    auto_fetch: bool
    fail_on_blocked: bool
    claslib_repo: str
    claslib_ref: str
    gps_week: int
    max_epochs: int
    gps_l2w_rows_min: int
    start_date: str
    start_time: str
    python_executable: str
    observation_file: str = "0627239Q.obs"
    navigation_file: str = "sept_2019239.nav"
    l6_file: str = "2019239Q.l6"


def repo_root_from_script() -> Path:
    return Path(__file__).resolve().parents[2]


def default_paths(output_dir: Path) -> RunPaths:
    work_dir = output_dir / "claslib_osr_zd_export"
    return RunPaths(
        output_dir=output_dir,
        work_dir=work_dir,
        log_path=output_dir / "ci_claslib_osr_zd_export" / "claslib_osr_zd_export.log",
        summary_json=output_dir / "ci_claslib_osr_zd_export_summary.json",
        checkout_dir=work_dir / "claslib",
        instrumented_source_dir=work_dir / "instrumented_claslib",
        claslib_config=work_dir / "static_linux.conf",
        claslib_solution=work_dir / "claslib.nmea",
        claslib_osr=work_dir / "claslib.nmea.osr",
        normalized_csv=work_dir / "claslib_osr.normalized.csv",
        normalized_summary_json=work_dir / "claslib_osr_summary.json",
        claslib_dd_dump=work_dir / "claslib_dd_measurement_dump.csv",
        claslib_state_dump=work_dir / "claslib_dd_filter_state_dump.csv",
    )


def resolve_optional_path(repo_root: Path, value: str) -> Path | None:
    text = value.strip()
    if not text:
        return None
    path = Path(text)
    return path if path.is_absolute() else repo_root / path


def env_or(environ: Mapping[str, str], name: str, default: str) -> str:
    value = environ.get(name, "").strip()
    return value if value else default


def parse_config(args: argparse.Namespace, environ: Mapping[str, str]) -> RunConfig:
    repo_root = args.repo_root.resolve()
    output_dir = (args.output_dir if args.output_dir is not None else repo_root / "output").resolve()
    max_epochs = int(env_or(environ, "GNSSPP_CLASLIB_OSR_MAX_EPOCHS", str(DEFAULT_MAX_EPOCHS)))
    return RunConfig(
        repo_root=repo_root,
        output_dir=output_dir,
        source_root=resolve_optional_path(repo_root, environ.get("GNSSPP_CLASLIB_OSR_SOURCE_ROOT", "")),
        auto_fetch=truthy(env_or(environ, "GNSSPP_CLASLIB_OSR_AUTO_FETCH", "1")),
        fail_on_blocked=truthy(env_or(environ, "GNSSPP_CLASLIB_OSR_FAIL_ON_BLOCKED", "1")),
        claslib_repo=env_or(environ, "GNSSPP_CLASLIB_OSR_REPO", DEFAULT_CLASLIB_REPO),
        claslib_ref=env_or(environ, "GNSSPP_CLASLIB_OSR_REF", DEFAULT_CLASLIB_REF),
        gps_week=int(env_or(environ, "GNSSPP_CLASLIB_OSR_GPS_WEEK", str(DEFAULT_GPS_WEEK))),
        max_epochs=max_epochs,
        gps_l2w_rows_min=int(
            env_or(environ, "GNSSPP_CLASLIB_OSR_GPS_L2W_ROWS_MIN", str(max_epochs))
        ),
        start_date=env_or(environ, "GNSSPP_CLASLIB_OSR_START_DATE", DEFAULT_START_DATE),
        start_time=env_or(environ, "GNSSPP_CLASLIB_OSR_START_TIME", DEFAULT_START_TIME),
        python_executable=sys.executable,
        observation_file=env_or(
            environ, "GNSSPP_CLASLIB_OSR_OBSERVATION_FILE", "0627239Q.obs"
        ),
        navigation_file=env_or(
            environ, "GNSSPP_CLASLIB_OSR_NAVIGATION_FILE", "sept_2019239.nav"
        ),
        l6_file=env_or(environ, "GNSSPP_CLASLIB_OSR_L6_FILE", "2019239Q.l6"),
    )


def data_root_failures(
    source_root: Path, required_inputs: Sequence[str] = ()
) -> list[str]:
    data_root = source_root / "data"
    required = dict.fromkeys((*REQUIRED_DATA_FILES, *required_inputs))
    return [name for name in required if not (data_root / name).is_file()]


def run_logged(
    command: Sequence[str],
    *,
    cwd: Path,
    log_path: Path,
    env: Mapping[str, str] | None = None,
) -> subprocess.CompletedProcess[str]:
    display = " ".join(command)
    with log_path.open("a", encoding="utf-8") as handle:
        handle.write(f"$ {display}\n")
    started = time.monotonic()
    completed = subprocess.run(
        list(command),
        cwd=cwd,
        env=dict(env) if env is not None else None,
        text=True,
        capture_output=True,
        check=False,
    )
    elapsed = time.monotonic() - started
    combined = completed.stdout
    if completed.stderr:
        if combined and not combined.endswith("\n"):
            combined += "\n"
        combined += completed.stderr
    with log_path.open("a", encoding="utf-8") as handle:
        handle.write(combined)
        if combined and not combined.endswith("\n"):
            handle.write("\n")
        handle.write(f"# exit={completed.returncode} elapsed_s={elapsed:.3f}\n\n")
    return completed


def checkout_claslib_source(config: RunConfig, paths: RunPaths) -> tuple[Path | None, str | None]:
    dataset_inputs = (
        config.observation_file,
        config.navigation_file,
        config.l6_file,
    )
    if config.source_root is not None:
        missing = data_root_failures(config.source_root, dataset_inputs)
        if not missing:
            return config.source_root, None
        return None, f"configured CLASLIB source root is missing data files: {', '.join(missing)}"

    if not config.auto_fetch:
        return None, "GNSSPP_CLASLIB_OSR_SOURCE_ROOT is unset and auto-fetch is disabled"

    if paths.checkout_dir.exists():
        shutil.rmtree(paths.checkout_dir)
    paths.checkout_dir.parent.mkdir(parents=True, exist_ok=True)
    commands = [
        ["git", "init", str(paths.checkout_dir)],
        ["git", "-C", str(paths.checkout_dir), "remote", "add", "origin", config.claslib_repo],
        ["git", "-C", str(paths.checkout_dir), "fetch", "--depth", "1", "origin", config.claslib_ref],
        ["git", "-C", str(paths.checkout_dir), "checkout", "--detach", "FETCH_HEAD"],
    ]
    for command in commands:
        completed = run_logged(command, cwd=config.repo_root, log_path=paths.log_path)
        if completed.returncode != 0:
            return None, f"failed to fetch CLASLIB source: {' '.join(command)}"

    missing = data_root_failures(paths.checkout_dir, dataset_inputs)
    if missing:
        return None, f"fetched CLASLIB source is missing data files: {', '.join(missing)}"
    return paths.checkout_dir, None


DD_DUMP_HELPER = r'''
/* GNSS++ canonical CLAS DD oracle instrumentation (clas_dd_measurement.v3). */
static gtime_t clas_dd_dump_time={0};

static int begin_clas_dd_dump(gtime_t time, int niter, int order)
{
    if (order!=FST||niter!=0) return 0;
    if (clas_dd_dump_time.time!=0&&fabs(timediff(time,clas_dd_dump_time))<1E-9) return 0;
    clas_dd_dump_time=time;
    return 1;
}

static void dump_clas_dd_row(gtime_t time, int system_group, int frequency,
                             int phase, int refsat, int target,
                             double raw_dd, double residual,
                             double refvar, double targetvar,
                             double refel, double targetel,
                             const double *x, const double *h)
{
    const char *path=getenv("CLASLIB_DD_ROW_DUMP");
    FILE *fp;
    int week;
    double tow;
    char refid[16],targetid[16];
    long size;
    if (!path||!*path) return;
    fp=fopen(path,"a+");
    if (!fp) return;
    fseek(fp,0,SEEK_END);
    size=ftell(fp);
    if (size==0) {
        fprintf(fp,"schema,week,tow,stage,system_group,frequency_index,"
                   "measurement_type,reference_satellite,target_satellite,raw_dd_m,residual_m,"
                   "reference_variance_m2,target_variance_m2,"
                   "reference_elevation_rad,target_elevation_rad,"
                   "linearization_x_m,linearization_y_m,linearization_z_m,"
                   "position_coeff_x,position_coeff_y,position_coeff_z\n");
    }
    tow=time2gpst(time,&week);
    satno2id(refsat,refid);
    satno2id(target,targetid);
    fprintf(fp,"clas_dd_measurement.v3,%d,%.9f,prefit,%d,%d,%s,%s,%s,"
               "%.17g,%.17g,%.17g,%.17g,%.17g,%.17g,"
               "%.17g,%.17g,%.17g,%.17g,%.17g,%.17g\n",
            week,tow,system_group,frequency,phase?"phase":"code",refid,targetid,
            raw_dd,residual,refvar,targetvar,refel,targetel,
            x[0],x[1],x[2],h[0],h[1],h[2]);
    fclose(fp);
}

'''


FILTER_STATE_DUMP_HELPER = r'''
/* GNSS++ canonical CLAS filter-state oracle instrumentation (clas_dd_filter_state.v1). */
static void dump_clas_filter_state(rtk_t *rtk, const obsd_t *obs, int n)
{
    const char *path=getenv("CLASLIB_FILTER_STATE_DUMP");
    FILE *fp;
    int i,j,week,seen[MAXSAT]={0};
    double tow,fixed;
    char id[16],key[16];
    long size;
    if (!path||!*path||n<=0) return;
    fp=fopen(path,"a+");
    if (!fp) return;
    fseek(fp,0,SEEK_END);
    size=ftell(fp);
    if (size==0) {
        fprintf(fp,"schema,week,tow,solution_state,state_kind,state_key,state_index,"
                   "float_value,fixed_value,variance,process_noise\n");
    }
    tow=time2gpst(obs[0].time,&week);
    for (i=0;i<3;i++) {
        sprintf(key,"%c",'x'+i);
        fixed=rtk->sol.stat==SOLQ_FIX?rtk->xa[i]:nan("");
        fprintf(fp,"clas_dd_filter_state.v1,%d,%.9f,%s,position,%s,%d,"
                   "%.17g,%.17g,%.17g,%.17g\n",
                week,tow,rtk->sol.stat==SOLQ_FIX?"fix":"float",key,i,
                rtk->x[i],fixed,rtk->P[i+i*rtk->nx],rtk->Q[i+i*rtk->nx]);
    }
    for (i=0;i<n;i++) {
        const int sat=obs[i].sat;
        if (sat<=0||sat>MAXSAT||seen[sat-1]) continue;
        seen[sat-1]=1;
        j=II(sat,&rtk->opt);
        if (j<0||j>=rtk->nx||rtk->x[j]==0.0) continue;
        satno2id(sat,id);
        fixed=rtk->sol.stat==SOLQ_FIX&&j<rtk->na?rtk->xa[j]:nan("");
        fprintf(fp,"clas_dd_filter_state.v1,%d,%.9f,%s,ionosphere,%s,%d,"
                   "%.17g,%.17g,%.17g,%.17g\n",
                week,tow,rtk->sol.stat==SOLQ_FIX?"fix":"float",id,j,
                rtk->x[j],fixed,rtk->P[j+j*rtk->nx],rtk->Q[j+j*rtk->nx]);
    }
    fclose(fp);
}

'''


def instrument_claslib_dd_dump(source_root: Path) -> None:
    source_path = source_root / "src" / "ppprtk.c"
    text = source_path.read_text(encoding="utf-8")
    if "GNSS++ canonical CLAS DD oracle instrumentation" in text:
        return
    helper_anchor = "/* single-differenced phase/code residuals  ----------------------------------"
    declaration_anchor = "    int refchgflg;\n"
    value_declaration_anchor = "    double pos[3],lami,lamj,*Ri,*Rj,*Hi=NULL;\n"
    trace_anchor = '    trace(3,"ddres   :niter=%2d nx=%d n=%d\\n",niter,rtk->nx,n);\n'
    row_anchor = "                /* set valid data flags */\n"
    raw_anchor = "                if (H) { /* partial derivatives by rover position */\n"
    if (helper_anchor not in text or declaration_anchor not in text or
            value_declaration_anchor not in text or raw_anchor not in text or
            trace_anchor not in text or row_anchor not in text):
        raise RuntimeError("CLASLIB ppprtk.c no longer matches the DD instrumentation contract")
    text = text.replace(helper_anchor, DD_DUMP_HELPER + helper_anchor, 1)
    text = text.replace(declaration_anchor, "    int refchgflg,dump_rows;\n", 1)
    text = text.replace(
        value_declaration_anchor,
        "    double pos[3],lami,lamj,*Ri,*Rj,*Hi=NULL,raw_dd;\n",
        1,
    )
    text = text.replace(
        trace_anchor,
        trace_anchor + "    dump_rows=begin_clas_dd_dump(obs[0].time,niter,order);\n",
        1,
    )
    text = text.replace(
        raw_anchor,
        "                raw_dd=v[nv];\n\n" + raw_anchor,
        1,
    )
    text = text.replace(
        row_anchor,
        "                if (dump_rows) dump_clas_dd_row(obs[0].time,m,f%nf,f<nf,sati,satj,\n"
        "                                              raw_dd,v[nv],Ri[nv],Rj[nv],\n"
        "                                              azel[1+i*2],azel[1+j*2],x,Hi);\n"
        + row_anchor,
        1,
    )
    source_path.write_text(text, encoding="utf-8")


def instrument_claslib_filter_state_dump(source_root: Path) -> None:
    source_path = source_root / "src" / "ppprtk.c"
    text = source_path.read_text(encoding="utf-8")
    if "GNSS++ canonical CLAS filter-state oracle instrumentation" in text:
        return
    helper_anchor = "/* single-differenced phase/code residuals  ----------------------------------"
    call_anchor = "    dops(n,azel,opt->elmin,rtk->sol.dop); /* {GDOP,PDOP,HDOP,VDOP} */\n"
    if helper_anchor not in text or call_anchor not in text:
        raise RuntimeError("CLASLIB ppprtk.c no longer matches the filter-state instrumentation contract")
    text = text.replace(helper_anchor, FILTER_STATE_DUMP_HELPER + helper_anchor, 1)
    text = text.replace(
        call_anchor,
        "    dump_clas_filter_state(rtk,obs,n);\n\n" + call_anchor,
        1,
    )
    source_path.write_text(text, encoding="utf-8")


def prepare_instrumented_source(source_root: Path, paths: RunPaths) -> Path:
    if source_root.resolve() == paths.checkout_dir.resolve():
        build_root = source_root
    else:
        if paths.instrumented_source_dir.exists():
            shutil.rmtree(paths.instrumented_source_dir)
        shutil.copytree(
            source_root,
            paths.instrumented_source_dir,
            ignore=shutil.ignore_patterns(".git", "*.o"),
        )
        build_root = paths.instrumented_source_dir
    instrument_claslib_dd_dump(build_root)
    instrument_claslib_filter_state_dump(build_root)
    return build_root


def write_linux_config(source_root: Path, paths: RunPaths) -> None:
    source_config = source_root / "util" / "rnx2rtkp" / "static.conf"
    text = source_config.read_text(encoding="utf-8")
    data_prefix = str(source_root / "data") + "/"
    text = text.replace("..\\..\\data\\", data_prefix)
    text = text.replace("../../data/", data_prefix)
    paths.claslib_config.write_text(text, encoding="utf-8")


def end_date_time(start_date: str, start_time: str, max_epochs: int) -> tuple[str, str]:
    start = datetime.strptime(f"{start_date} {start_time}", "%Y/%m/%d %H:%M:%S")
    end = start + timedelta(seconds=max(max_epochs - 1, 0))
    return end.strftime("%Y/%m/%d"), end.strftime("%H:%M:%S")


def end_time(start_time: str, max_epochs: int) -> str:
    # Compatibility helper for callers that only need a same/next-day clock.
    _, value = end_date_time("2000/01/01", start_time, max_epochs)
    return value


def build_claslib_command(source_root: Path) -> list[str]:
    return ["make", "-C", str(source_root / "util" / "rnx2rtkp")]


def build_rnx2rtkp_command(config: RunConfig, paths: RunPaths, source_root: Path) -> list[str]:
    data_root = source_root / "data"
    binary = source_root / "util" / "rnx2rtkp" / "rnx2rtkp"
    end_date, end_clock = end_date_time(
        config.start_date, config.start_time, config.max_epochs
    )
    return [
        str(binary),
        "-ti",
        "1",
        "-ts",
        config.start_date,
        config.start_time,
        "-te",
        end_date,
        end_clock,
        "-l6w",
        str(config.gps_week),
        "-x",
        "0",
        "-s",
        "-k",
        str(paths.claslib_config),
        "-o",
        str(paths.claslib_solution),
        str(data_root / config.observation_file),
        str(data_root / config.navigation_file),
        str(data_root / config.l6_file),
    ]


def build_normalize_command(config: RunConfig, paths: RunPaths, source_root: Path) -> list[str]:
    return [
        config.python_executable,
        str(config.repo_root / "scripts" / "analysis" / "claslib_zd_component_export.py"),
        str(paths.claslib_osr),
        "--output",
        str(paths.normalized_csv),
        "--stage-label",
        "post",
        "--gps-week",
        str(config.gps_week),
        "--clas-grid-def",
        str(source_root / "data" / "clas_grid.def"),
    ]


def build_summary_command(config: RunConfig, paths: RunPaths) -> list[str]:
    return [
        config.python_executable,
        str(config.repo_root / "scripts" / "analysis" / "clas_zd_component_summary.py"),
        str(paths.normalized_csv),
        "--json-out",
        str(paths.normalized_summary_json),
        "--fail-on-issue",
        "--require-system",
        "GPS",
        "--require-row-type",
        "code",
        "--require-rinex-code",
        "C2W",
        "--require-component",
        "prc_m",
        "--require-gps-l2w-rows-min",
        str(config.gps_l2w_rows_min),
    ]


def load_json(path: Path) -> dict[str, object]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"{path}: expected JSON object")
    return payload


def count_csv_data_rows(path: Path) -> int:
    if not path.is_file():
        return 0
    with path.open(encoding="utf-8") as handle:
        return max(sum(1 for _line in handle) - 1, 0)


def count_gps_l2w_grid_provenance_rows(path: Path) -> int:
    if not path.is_file():
        return 0
    import csv

    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        count = 0
        for row in reader:
            row_type = row.get("row_type", "").strip().lower()
            signal = row.get("signal", "") or row.get("pseudorange_rinex_code", "")
            if row_type != "code" or signal != "C2W":
                continue
            if row.get("atmos_network_id", "") and row.get("atmos_grid_no", ""):
                count += 1
        return count


def evaluate(
    config: RunConfig,
    normalized_summary: Mapping[str, object],
    normalized_rows: int,
    grid_provenance_rows: int,
) -> tuple[dict[str, object], dict[str, object], list[str]]:
    identity = normalized_summary.get("identity_provenance")
    if not isinstance(identity, dict):
        identity = {}
    gps_l2w_rows = int(identity.get("gps_l2w_rows", 0))
    metrics = {
        "normalized_rows": normalized_rows,
        "normalized_summary_schema": normalized_summary.get("schema"),
        "normalized_summary_status": normalized_summary.get("status"),
        "normalized_summary_rows": normalized_summary.get("rows"),
        "gps_l2w_rows": gps_l2w_rows,
        "gps_l2w_grid_provenance_rows": grid_provenance_rows,
    }
    thresholds = {
        "normalized_rows_min": 1,
        "gps_l2w_rows_min": config.gps_l2w_rows_min,
        "gps_l2w_grid_provenance_rows_min": config.gps_l2w_rows_min,
    }
    failures: list[str] = []
    if normalized_rows <= 0:
        failures.append("normalized CLASLIB OSR dump has no data rows")
    if metrics["normalized_summary_schema"] != "clas_zd_component_summary.v2":
        failures.append(
            f"normalized summary schema {metrics['normalized_summary_schema']} != clas_zd_component_summary.v2"
        )
    if metrics["normalized_summary_status"] != "passed":
        failures.append(f"normalized summary status {metrics['normalized_summary_status']} != passed")
    if metrics["normalized_summary_rows"] != normalized_rows:
        failures.append(
            f"normalized summary rows {metrics['normalized_summary_rows']} != normalized rows {normalized_rows}"
        )
    if gps_l2w_rows < config.gps_l2w_rows_min:
        failures.append(f"GPS L2W rows {gps_l2w_rows} < {config.gps_l2w_rows_min}")
    if grid_provenance_rows < config.gps_l2w_rows_min:
        failures.append(
            f"GPS L2W grid provenance rows {grid_provenance_rows} < {config.gps_l2w_rows_min}"
        )
    return metrics, thresholds, failures


def summarize_artifacts(paths: RunPaths) -> list[dict[str, object]]:
    return [
        artifact_record(paths.log_path, role="log", required=True),
        artifact_record(paths.claslib_config, role="claslib_linux_config", required=True),
        artifact_record(paths.claslib_solution, role="claslib_solution", required=True),
        artifact_record(paths.claslib_osr, role="claslib_osr", required=True),
        artifact_record(paths.normalized_csv, role="normalized_zd_component_dump", required=True),
        artifact_record(paths.normalized_summary_json, role="normalized_zd_component_summary", required=True),
        artifact_record(paths.claslib_dd_dump, role="claslib_dd_measurement_dump", required=True),
        artifact_record(paths.claslib_state_dump, role="claslib_dd_filter_state_dump", required=True),
    ]


def write_summary(paths: RunPaths, payload: dict[str, object]) -> None:
    paths.summary_json.parent.mkdir(parents=True, exist_ok=True)
    paths.summary_json.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def build_payload(
    *,
    config: RunConfig,
    paths: RunPaths,
    status: str,
    source_root: Path | None,
    metrics: dict[str, object] | None = None,
    thresholds: dict[str, object] | None = None,
    failures: list[str] | None = None,
    block_reason: str | None = None,
) -> dict[str, object]:
    next_actions: list[str] = []
    if status == "blocked_infrastructure":
        next_actions.append("Provide CLASLIB source with GNSSPP_CLASLIB_OSR_SOURCE_ROOT or allow auto-fetch.")
        next_actions.append("Treat blocked_infrastructure as missing CLASLIB OSR oracle evidence.")
        if config.fail_on_blocked:
            next_actions.append("Set GNSSPP_CLASLIB_OSR_FAIL_ON_BLOCKED=0 only for diagnostic-only local runs.")
    elif status == "failed":
        next_actions.append("Inspect the CLASLIB build/run log and normalized summary before changing CLAS model behavior.")
    return {
        "summary_schema": SUMMARY_SCHEMA,
        "contract": CONTRACT,
        "status": status,
        "block_reason": block_reason,
        "failures": failures or [],
        "next_actions": next_actions,
        "input_provenance": {
            "claslib_repo": config.claslib_repo,
            "claslib_ref": config.claslib_ref,
            "source_root": str(source_root) if source_root is not None else None,
            "required_data_files": list(dict.fromkeys((
                *REQUIRED_DATA_FILES,
                config.observation_file,
                config.navigation_file,
                config.l6_file,
            ))),
        },
        "configuration": {
            "gps_week": config.gps_week,
            "dataset_inputs": {
                "observation": config.observation_file,
                "navigation": config.navigation_file,
                "l6": config.l6_file,
            },
            "max_epochs": config.max_epochs,
            "start_date": config.start_date,
            "start_time": config.start_time,
            "end_date": end_date_time(
                config.start_date, config.start_time, config.max_epochs
            )[0],
            "end_time": end_date_time(
                config.start_date, config.start_time, config.max_epochs
            )[1],
            "fail_on_blocked": config.fail_on_blocked,
            "dd_measurement_dump": {
                "schema": "clas_dd_measurement.v3",
                "stage": "prefit",
                "path": str(paths.claslib_dd_dump),
                "source_policy": "generated-checkout-or-copied-user-source",
            },
            "filter_state_dump": {
                "schema": "clas_dd_filter_state.v1",
                "path": str(paths.claslib_state_dump),
                "source_policy": "generated-checkout-or-copied-user-source",
            },
            "zd_filter": {"stage": "post", "row_type": "code", "sat": "G14", "freq": 1, "rinex_code": "C2W"},
        },
        "metrics": metrics or {},
        "thresholds": thresholds or {},
        "artifacts": summarize_artifacts(paths),
    }


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo-root", type=Path, default=repo_root_from_script())
    parser.add_argument("--output-dir", type=Path, default=None)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None, environ: Mapping[str, str] | None = None) -> int:
    args = parse_args(argv)
    env = os.environ if environ is None else environ
    config = parse_config(args, env)
    paths = default_paths(config.output_dir)
    paths.work_dir.mkdir(parents=True, exist_ok=True)
    paths.log_path.parent.mkdir(parents=True, exist_ok=True)
    paths.log_path.write_text("", encoding="utf-8")

    source_root, block_reason = checkout_claslib_source(config, paths)
    if source_root is None:
        payload = build_payload(
            config=config,
            paths=paths,
            status="blocked_infrastructure",
            source_root=None,
            block_reason=block_reason,
        )
        write_summary(paths, payload)
        print(f"CLASLIB OSR ZD export blocked: {block_reason}")
        print(f"  summary: {paths.summary_json}")
        return 1 if config.fail_on_blocked else 0

    try:
        build_root = prepare_instrumented_source(source_root, paths)
    except (OSError, RuntimeError) as error:
        payload = build_payload(
            config=config,
            paths=paths,
            status="failed",
            source_root=source_root,
            failures=[f"CLASLIB DD instrumentation failed: {error}"],
        )
        write_summary(paths, payload)
        print(f"CLASLIB DD instrumentation failed: {error}")
        return 1

    write_linux_config(build_root, paths)

    build_result = run_logged(build_claslib_command(build_root), cwd=config.repo_root, log_path=paths.log_path)
    if build_result.returncode != 0:
        payload = build_payload(
            config=config,
            paths=paths,
            status="failed",
            source_root=source_root,
            failures=["CLASLIB rnx2rtkp build failed"],
        )
        write_summary(paths, payload)
        print(f"CLASLIB build failed; see {paths.log_path}")
        return 1

    paths.claslib_dd_dump.unlink(missing_ok=True)
    paths.claslib_state_dump.unlink(missing_ok=True)
    claslib_env = dict(env)
    claslib_env["CLASLIB_DD_ROW_DUMP"] = str(paths.claslib_dd_dump)
    claslib_env["CLASLIB_FILTER_STATE_DUMP"] = str(paths.claslib_state_dump)
    rnx_result = run_logged(
        build_rnx2rtkp_command(config, paths, build_root),
        cwd=config.repo_root,
        log_path=paths.log_path,
        env=claslib_env,
    )
    if rnx_result.returncode != 0 or not paths.claslib_osr.is_file():
        payload = build_payload(
            config=config,
            paths=paths,
            status="failed",
            source_root=source_root,
            failures=["CLASLIB rnx2rtkp OSR run failed or did not produce .osr"],
        )
        write_summary(paths, payload)
        print(f"CLASLIB OSR run failed; see {paths.log_path}")
        return 1

    normalize_result = run_logged(
        build_normalize_command(config, paths, build_root),
        cwd=config.repo_root,
        log_path=paths.log_path,
    )
    if normalize_result.returncode != 0:
        payload = build_payload(
            config=config,
            paths=paths,
            status="failed",
            source_root=source_root,
            failures=["CLASLIB OSR normalization failed"],
        )
        write_summary(paths, payload)
        print(f"CLASLIB OSR normalization failed; see {paths.log_path}")
        return 1

    summary_result = run_logged(build_summary_command(config, paths), cwd=config.repo_root, log_path=paths.log_path)
    if summary_result.returncode != 0:
        payload = build_payload(
            config=config,
            paths=paths,
            status="failed",
            source_root=source_root,
            failures=["CLASLIB OSR normalized summary failed"],
        )
        write_summary(paths, payload)
        print(f"CLASLIB OSR normalized summary failed; see {paths.log_path}")
        return 1

    normalized_summary = load_json(paths.normalized_summary_json)
    metrics, thresholds, failures = evaluate(
        config,
        normalized_summary,
        count_csv_data_rows(paths.normalized_csv),
        count_gps_l2w_grid_provenance_rows(paths.normalized_csv),
    )
    dd_measurement_rows = count_csv_data_rows(paths.claslib_dd_dump)
    metrics["claslib_dd_measurement_rows"] = dd_measurement_rows
    thresholds["claslib_dd_measurement_rows_min"] = 1
    if dd_measurement_rows < 1:
        failures.append("CLASLIB DD measurement dump has no data rows")
    filter_state_rows = count_csv_data_rows(paths.claslib_state_dump)
    metrics["claslib_filter_state_rows"] = filter_state_rows
    thresholds["claslib_filter_state_rows_min"] = 1
    if filter_state_rows < 1:
        failures.append("CLASLIB filter-state dump has no data rows")
    status = "failed" if failures else "passed"
    payload = build_payload(
        config=config,
        paths=paths,
        status=status,
        source_root=source_root,
        metrics=metrics,
        thresholds=thresholds,
        failures=failures,
    )
    write_summary(paths, payload)
    print("CLASLIB OSR ZD export:", status)
    print(f"  summary: {paths.summary_json}")
    print(f"  normalized dump: {paths.normalized_csv}")
    if failures:
        for failure in failures:
            print(f"  - {failure}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
