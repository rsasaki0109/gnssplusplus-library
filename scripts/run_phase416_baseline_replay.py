"""Run the frozen Phase416 raw-only experiment once; never evaluate truth."""

import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase416_h_baseline_replay_manifest_v1.json"


def digest(path):
    with path.open("rb") as handle:
        return hashlib.file_digest(handle, "sha256").hexdigest()


def main():
    manifest_bytes = MANIFEST.read_bytes()
    manifest = json.loads(manifest_bytes)
    assert manifest["phase"] == 416
    pins = {**manifest["source_pins"], **manifest["test_pins"],
            **manifest["algorithm_source_pins"]}
    for name, expected in pins.items():
        if digest(ROOT / name) != expected:
            raise RuntimeError(f"static pin mismatch: {name}")
    for item in [manifest["binary"], *manifest["inputs"].values()]:
        path = ROOT / item["path"]
        if path.stat().st_size != item["bytes"] or digest(path) != item["sha256"]:
            raise RuntimeError(f"input/binary pin mismatch: {path.name}")
    argv = manifest["argv"]
    for flag, name in [("--android-gnss", "android_gnss"),
                       ("--android-imu", "android_imu"), ("--nav", "nav")]:
        assert argv[argv.index(flag) + 1] == manifest["inputs"][name]["path"]
    assert "--native-source-tdcp-meter-sigma" in argv
    assert "--native-phase184-source-tdcp-huber-k" not in argv
    assert "--native-phase217-main-pose3-motion" in argv
    assert "--native-phase213-main-doppler" in argv
    assert "--native-phase209-source-separate-imu-factors" not in argv
    assert "--native-phase205-source-count-bias-density" not in argv
    assert "--native-phase201-source-inclusive-forward-imu-schedule" not in argv
    assert "--native-main-p-cauchy" not in argv
    assert "--native-base-pseudorange-compensation" not in argv
    assert "--native-tdcp-no-code-jump-gate" not in argv
    assert "--native-sparse-p-staging" not in argv
    assert "--native-stationary-gyro-initializer" not in argv
    assert "--native-tdcp-frequency-residual-states" in argv
    launch = manifest["launcher_contract"]
    status_path = ROOT / launch["execution_metadata_path"]
    # Exclusive directory creation blocks accidental duplicate invocations,
    # even after a failed or interrupted run. Never repair/retry automatically.
    status_path.parent.mkdir(parents=True, exist_ok=False)
    for flag in ["--out", "--summary-json"]:
        path = ROOT / argv[argv.index(flag) + 1]
        assert path.parent == status_path.parent and not path.exists()
    env = os.environ.copy()
    env["LD_LIBRARY_PATH"] = "/home/sasaki/.local/lib:" + env.get("LD_LIBRARY_PATH", "")
    started = time.monotonic()
    record = {
        "manifest_sha256": hashlib.sha256(manifest_bytes).hexdigest(),
        "start_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "return_code": None, "native_invocations": 1,
        "argv_loaded_from_manifest": True,
        "stdout_path": launch["stdout_path"], "stderr_path": launch["stderr_path"],
    }
    with (ROOT / launch["stdout_path"]).open("xb") as stdout, \
         (ROOT / launch["stderr_path"]).open("xb") as stderr:
        process = subprocess.Popen(argv, cwd=ROOT, env=env, stdout=stdout, stderr=stderr)
        record["native_pid"] = process.pid
        with status_path.open("x") as handle:
            json.dump(record, handle, indent=2)
        print(json.dumps(record), flush=True)
        record["return_code"] = process.wait()
    record["end_utc"] = datetime.datetime.now(datetime.timezone.utc).isoformat()
    record["elapsed_seconds"] = time.monotonic() - started
    final_path = status_path.with_name("launcher_completed.json")
    with final_path.open("x") as handle:
        json.dump(record, handle, indent=2)
    print(json.dumps(record), flush=True)
    return record["return_code"]


if __name__ == "__main__":
    raise SystemExit(main())
