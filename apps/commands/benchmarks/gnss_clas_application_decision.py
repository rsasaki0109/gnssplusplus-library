#!/usr/bin/env python3
"""Run and package a truth-scored CLAS application decision."""
from __future__ import annotations
import argparse, csv, hashlib, json, math, os, shutil, subprocess, sys
from pathlib import Path
from typing import Any
from support.gnss_runtime import application_root

ROOT=application_root(__file__); SCHEMA="libgnsspp.clas_application_decision.v1"
REQUIRED_ARTIFACTS=(
 "clas_solution.pos", "clas_solution.kml", "clas_trajectory.png", "scorecard.md",
 "decode.log", "decoded_messages.csv", "bundle.log", "summary.json",
)
PINS={
 "2024205D.l6": {"bytes":900000,"sha256":"19f399df1cfdb7c1e277cf4b9e03596d145d66a4a90fd50c2030fe36dbf6b076"},
 "2024205E.l6": {"bytes":900000,"sha256":"96b434b7ce7d60c6a5768bb429b4857b595df3b04f4a064ac6457f92c1e0d50b"},
}

def rec(p:Path)->dict[str,Any]: return {"path":str(p),"bytes":p.stat().st_size,"sha256":hashlib.sha256(p.read_bytes()).hexdigest()}
def first_data(path:Path, delimiter=","):
 with path.open(errors="replace") as f:
  for line in f:
   if line.strip() and not line.startswith(("#","%")): return line.strip().split(delimiter) if delimiter != " " else line.split()
 return None
def valid_position_rows(path:Path):
 with path.open(errors="replace") as f:
  for line in f:
   if not line.strip() or line.startswith(("#","%")): continue
   parts=line.split()
   if len(parts)<9: continue
   try:
    week=int(float(parts[0])); tow=float(parts[1]); ecef=[float(parts[i]) for i in (2,3,4)]; status=int(float(parts[8]))
   except (TypeError,ValueError,IndexError): continue
   if math.isfinite(tow) and all(math.isfinite(value) for value in ecef):
    yield {"week":week,"tow":tow,"ecef":ecef,"status":status}
def unique_ssr_tows(path:Path)->set[float]:
 result=set()
 with path.open(errors="replace") as f:
  for line in f:
   if not line.strip() or line.startswith("#"): continue
   try:
    tow=float(line.split(",",2)[1])
    if math.isfinite(tow): result.add(tow)
   except (ValueError,IndexError): pass
 return result
def pos_tow_bounds(path:Path)->tuple[float,float]|None:
 stamps=[row["tow"] for row in valid_position_rows(path)]
 return (min(stamps),max(stamps)) if stamps else None
def pos_span_s(path:Path)->float:
 bounds=pos_tow_bounds(path)
 return bounds[1]-bounds[0] if bounds is not None and bounds[1]>bounds[0] else 0.0
def correction_coverage_s(tows:set[float], bounds:tuple[float,float]|None, window_s:float=30.0)->float:
 if bounds is None or bounds[1]<=bounds[0]: return 0.0
 start,end=bounds; intervals=[]
 for tow in sorted(tows):
  left=max(start,tow); right=min(end,tow+window_s)
  if right>left: intervals.append((left,right))
 total=0.0; current=None
 for left,right in intervals:
  if current is None: current=[left,right]
  elif left<=current[1]: current[1]=max(current[1],right)
  else: total+=current[1]-current[0]; current=[left,right]
 if current is not None: total+=current[1]-current[0]
 return total
def parse_args(argv=None):
 p=argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME","gnss clas-application-decision")); p.add_argument("--dataset-root",type=Path,default=ROOT/"data/PPC-Dataset"); p.add_argument("--output-dir",type=Path,required=True); p.add_argument("--l6-cache",type=Path,required=True); p.add_argument("--run",default="tokyo_run1",choices=("tokyo_run1",)); p.add_argument("--reuse",action="store_true"); p.add_argument("--inject-correction-loss",action="store_true"); raw=list(argv) if argv is not None else list(sys.argv[1:]); args=p.parse_args(raw); args._raw_argv=raw; return args
def manifest_argv(args)->list[str]:
 if args._raw_argv==list(sys.argv[1:]): return list(sys.argv)
 return [sys.executable,str(Path(__file__).resolve()),*args._raw_argv]
def artifact_record(path:Path)->dict[str,Any]:
 if not path.is_file(): return {"path":str(path),"bytes":None,"sha256":None,"state":"missing"}
 return rec(path)
def finite_or_none(value):
 try:
  number=float(value)
 except (TypeError,ValueError): return None
 return number if math.isfinite(number) else None
def git_revision()->str:
 result=subprocess.run(["git","rev-parse","HEAD"],cwd=ROOT,text=True,capture_output=True,check=False)
 return result.stdout.strip() if result.returncode==0 and isinstance(result.stdout,str) and result.stdout.strip() else "unknown"
def run(argv=None)->int:
 a=parse_args(argv); out=a.output_dir.resolve(); out.mkdir(parents=True,exist_ok=True); report=out/"scorecard.md"
 if not a.reuse:
  cmd=[sys.executable,str(ROOT/"scripts/experiments/ppc/generate_ppc_clas_scorecard.py"),"--dataset-root",str(a.dataset_root),"--work-dir",str(out),"--l6-cache",str(a.l6_cache),"--configs","parity","--runs",a.run,"--report",str(report)]
  result=subprocess.run(cmd,cwd=ROOT,check=False)
  if result.returncode: return result.returncode
 run_dir=out/a.run; pos=run_dir/"pos"/f"{a.run}_parity.pos"; score=run_dir/"pos"/f"{a.run}_parity_summary.json"; ssr=run_dir/"ssr"/f"{a.run}_expanded.csv"; l6=run_dir/"l6"/f"{a.run}.l6"
 failures=[]
 for p in (pos,score,ssr,l6):
  if not p.is_file(): failures.append(f"missing_CLAS_artifact:{p}")
 if failures:
  print("; ".join(failures)); return 1
 solution_out=out/"clas_solution.pos"; shutil.copyfile(pos,solution_out)
 diag=out/"decode.log"; messages=out/"decoded_messages.csv"; decode_rc=0
 with diag.open("w") as log:
  decode_result=subprocess.run([sys.executable,str(ROOT/"apps/gnss.py"),"qzss-l6-info","--input",str(l6),"--gps-week","2324","--extract-compact-messages",str(messages)],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,check=False)
  decode_rc=decode_result.returncode
 if decode_rc: failures.append("decode_failed")
 if not messages.is_file() or messages.stat().st_size==0: failures.append("decode_message_inventory_missing")
 bundle_log=out/"bundle.log"
 visual_commands=[
  [sys.executable,str(ROOT/"apps/gnss.py"),"pos2kml",str(solution_out),str(out/"clas_solution.kml"),"--status","all"],
  [sys.executable,str(ROOT/"apps/gnss.py"),"trackplot",str(solution_out)],
 ]
 with bundle_log.open("w") as log:
  for cmd in visual_commands:
   log.write("$ "+" ".join(cmd)+"\n"); log.flush()
   if subprocess.run(cmd,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,check=False).returncode: failures.append("visualization_failed")
 generated=solution_out.with_name(solution_out.stem+"_trajectory.png")
 if generated.is_file(): os.replace(generated,out/"clas_trajectory.png")
 else: failures.append("visualization_missing")
 try:
  metrics=json.loads(score.read_text()); status={str(k):int(v) for k,v in metrics.get("status_counts",{}).items()}
  total=int(metrics["matched_epochs"]); fixed=int(metrics["fixed_epochs"]); reference_epochs=int(metrics["reference_epochs"])
 except (OSError,TypeError,ValueError,KeyError,json.JSONDecodeError) as exc:
  metrics={}; status={}; total=fixed=reference_epochs=0; failures.append(f"score_summary_invalid:{exc}")
 all_tows=unique_ssr_tows(ssr)
 if a.inject_correction_loss: all_tows=set()
 bounds=pos_tow_bounds(pos); in_route_tows={tow for tow in all_tows if bounds is not None and bounds[0]<=tow<=bounds[1]}
 first_pos=next(valid_position_rows(pos),None); first_position=None
 if first_pos is not None:
  first_position={"week":first_pos["week"],"tow":first_pos["tow"]}
 else: failures.append("first_valid_position_missing")
 first_ssr=min(in_route_tows) if in_route_tows else None
 correction_covered_s=correction_coverage_s(in_route_tows,bounds)
 span=pos_span_s(pos); coverage=100.0*correction_covered_s/span if span>0.0 else 0.0
 coverage=min(100.0,max(0.0,coverage)); fallback=status.get("1",0)
 if not in_route_tows: failures.append("correction_unavailable")
 if fixed==0: failures.append("no_fixed_solution")
 fixed_rms=metrics.get("rms2d_fixed_m",float("nan")); whole_rms=metrics.get("rms2d_all_m",float("nan"))
 try: fixed_rms=float(fixed_rms); whole_rms=float(whole_rms)
 except (TypeError,ValueError): fixed_rms=whole_rms=float("nan")
 if not math.isfinite(fixed_rms) or fixed_rms>1.0: failures.append("fixed_rms2d_not_submeter")
 if not math.isfinite(whole_rms): failures.append("whole_route_rms2d_invalid")
 elif whole_rms>1.0: failures.append("whole_route_rms2d_not_submeter")
 fix_pct=finite_or_none(metrics.get("fix_pct"))
 if fix_pct is None or not 0.0<=fix_pct<=100.0: failures.append("fix_rate_invalid")
 elif fix_pct<50.0: failures.append("fixed_solution_availability_below_minimum")
 if any(value<0 for value in status.values()): failures.append("status_population_invalid")
 if reference_epochs<=0 or total<0 or total>reference_epochs: failures.append("solution_population_invalid")
 availability=100.0*total/reference_epochs if reference_epochs>0 else 0.0
 for name,pin in PINS.items():
  cached=a.l6_cache/name
  if not cached.is_file(): failures.append(f"pinned_source_missing:{name}")
  elif cached.stat().st_size!=pin["bytes"] or hashlib.sha256(cached.read_bytes()).hexdigest()!=pin["sha256"]: failures.append(f"pinned_source_mismatch:{name}")
 if coverage<50.0: failures.append("correction_availability_below_minimum")
 for name in REQUIRED_ARTIFACTS:
  if name=="summary.json": continue
  if not (out/name).is_file(): failures.append(f"required_artifact_missing:{name}")
 state="usable" if not failures else ("degraded" if in_route_tows else "unusable")
 summary={"schema_version":SCHEMA,"state":state,"variant":"CLAS_L6D_compact_SSR","truth_role":"PPC_Applanix_independent_reference",
  "populations":{"reference_matched_epochs":total,"fixed_epochs":fixed,"whole_route_epochs":total,"correction_epochs":len(in_route_tows),"correction_epochs_outside_solution_window":len(all_tows)-len(in_route_tows)},
  "metrics":{"fixed_rms2d_m":fixed_rms if math.isfinite(fixed_rms) else None,"whole_route_rms2d_m":whole_rms if math.isfinite(whole_rms) else None,"solution_availability_pct":availability,"fix_pct":fix_pct,"correction_availability_pct":coverage,"first_correction_tow":first_ssr,"first_valid_position":first_position,"fallback_epochs":fallback},
  "decode":{"diagnostic_log":"decode.log","message_inventory":"decoded_messages.csv","unsupported_message_types":"see decode.log; unsupported types are never counted as corrections"},
  "variants":{"CLAS":{"coverage":"Japan QZSS L6","receiver":"L6D/CLAS-capable","evidence":"truth_scored_here"},"MADOCA_L6E_L6D":{"coverage":"service dependent","receiver":"L6E/L6D-capable","evidence":"separate gnss clas-ppp --profile madoca route; not scored by this CLAS capture"},"RTCM_SSR":{"coverage":"caster/network dependent","receiver":"IP/RTCM SSR client","evidence":"separate --ssr-rtcm route; not scored by this L6 capture"}},
  "gate":{"status":"pass" if not failures else "fail","thresholds":{"fixed_rms2d_max_m":1.0,"whole_route_rms2d_max_m":1.0,"fix_rate_min_pct":50.0,"correction_availability_min_pct":50.0},"failure_reasons":sorted(set(failures))}}
 (out/"summary.json").write_text(json.dumps(summary,indent=2,sort_keys=True)+"\n")
 input_paths={"rover_obs":a.dataset_root/"tokyo/run1/rover.obs","navigation":a.dataset_root/"tokyo/run1/base.nav","reference":a.dataset_root/"tokyo/run1/reference.csv","l6":l6,"expanded_ssr":ssr}
 inputs={name:(rec(path) if path.is_file() else {"path":str(path),"bytes":None,"sha256":None,"state":"missing"}) for name,path in input_paths.items()}
 artifacts={name:artifact_record(out/name) for name in REQUIRED_ARTIFACTS}
 manifest={"schema_version":SCHEMA,"state":state,"inputs":inputs,"run":{"argv":manifest_argv(a),"software_revision":git_revision()},"frames":{"solution":"ECEF/WGS84 antenna position","reference":"PPC raw antenna-positioned Applanix","time":"GPST","time_system":"GPST","lever_arm":{"status":"not_applicable","antenna_to_target_m":[0.0,0.0,0.0],"application":"solution and independent reference are antenna phase-center ECEF; no target-frame transform"}},"truth_role":summary["truth_role"],"populations":summary["populations"],"metrics":summary["metrics"],"gate":summary["gate"],"artifacts":artifacts,"provenance":{"source":"QZSS public CLAS L6 archive","url_template":"https://sys.qzss.go.jp/archives/l6/{year}/{YYYYDOY}{slot}.l6","pinned_hourly_files":PINS,"correction_tow_semantics":"GPS time-of-week; coverage is union of 30-second correction intervals clipped to the solution POS time window"}}
 (out/"manifest.json").write_text(json.dumps(manifest,indent=2,sort_keys=True)+"\n")
 print(f"CLAS application decision: {state}; gate={summary['gate']['status']}"); return 0 if not failures else 1
if __name__=="__main__": raise SystemExit(run())
