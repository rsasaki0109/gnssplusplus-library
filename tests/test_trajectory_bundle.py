#!/usr/bin/env python3
from __future__ import annotations
import importlib.util, json, sys, tempfile, unittest
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
for p in (ROOT/"apps/commands", ROOT/"apps/commands/benchmarks"):
    sys.path.insert(0,str(p))
def load(name, path):
    spec=importlib.util.spec_from_file_location(name,path); mod=importlib.util.module_from_spec(spec); spec.loader.exec_module(mod); return mod
bundle=load("trajectory_bundle",ROOT/"apps/commands/benchmarks/gnss_trajectory_bundle.py")
validate=load("trajectory_validate",ROOT/"apps/commands/benchmarks/gnss_trajectory_bundle_validate.py")

class TrajectoryBundleTests(unittest.TestCase):
    def make_inputs(self, root):
        pos=root/"input.pos"; ref=root/"reference.csv"
        pos.write_text("% LibGNSS++ Position Solution\n"+"".join(f"2200 {100+i:.1f} {6378137+i:.3f} 0 0 0 0 {i:.3f} 4 12 0.1\n" for i in range(4)))
        ref.write_text("tow,week,lat,lon,height,x,y,z\n"+"".join(f"{100+i},2200,0,0,{i},{6378137+i},0,0\n" for i in range(4)))
        return pos,ref
    def test_real_bundle_contract_loadable_and_tamper_fails(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td); pos,ref=self.make_inputs(root); out=root/"bundle"
            self.assertEqual(bundle.run(["--pos",str(pos),"--reference-csv",str(ref),"--output-dir",str(out),"--profile","slam_evaluation","--target-frame","base_link","--lever-arm-m=0,0,0"]),0)
            self.assertEqual(validate.run([str(out)]),0)
            sys.path.insert(0,str(ROOT/"build/python")); from libgnsspp.artifacts import load_pos
            self.assertEqual(len(load_pos(out/"accepted.pos")),4)
            manifest=json.loads((out/"manifest.json").read_text()); self.assertEqual(manifest["label"],"ground_truth")
            (out/"accepted.pos").write_text("tampered\n")
            self.assertEqual(validate.run([str(out)]),1)
    def test_no_truth_never_labels_ground_truth(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td); pos,_=self.make_inputs(root); out=root/"bundle"
            self.assertEqual(bundle.run(["--pos",str(pos),"--output-dir",str(out),"--profile","visualization","--target-frame","map","--lever-arm-m=0,0,0"]),1)
            self.assertEqual(json.loads((out/"manifest.json").read_text())["label"],"candidate_trajectory")

    def test_validator_requires_every_declared_bundle_artifact(self):
        with tempfile.TemporaryDirectory() as td:
            root=Path(td); pos,ref=self.make_inputs(root); out=root/"bundle"
            self.assertEqual(bundle.run(["--pos",str(pos),"--reference-csv",str(ref),"--output-dir",str(out),"--profile","slam_evaluation","--target-frame","base_link","--lever-arm-m=0,0,0"]),0)
            manifest=json.loads((out/"manifest.json").read_text())
            del manifest["artifacts"]["trajectory.png"]
            (out/"manifest.json").write_text(json.dumps(manifest))
            self.assertEqual(validate.run([str(out)]),1)

if __name__=="__main__": unittest.main()
