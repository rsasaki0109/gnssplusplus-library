#!/usr/bin/env python3
from __future__ import annotations
import importlib.util, json, sys, tempfile, unittest
from pathlib import Path
from unittest import mock
ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT/"apps/commands"))
spec=importlib.util.spec_from_file_location("clas_decision",ROOT/"apps/commands/benchmarks/gnss_clas_application_decision.py"); mod=importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
class ClasDecisionTests(unittest.TestCase):
 def fixture(self,root):
  dataset=root/"data"; run=root/"out"/"tokyo_run1"; (dataset/"tokyo/run1").mkdir(parents=True); (run/"pos").mkdir(parents=True); (run/"ssr").mkdir(); (run/"l6").mkdir()
  for n in ("rover.obs","base.nav","reference.csv"): (dataset/"tokyo/run1"/n).write_text("fixture\n")
  (run/"l6/tokyo_run1.l6").write_bytes(b"l6")
  # qzss-l6-info is mocked below; keep the decode inventory contract explicit.
  (root/"out/decoded_messages.csv").write_text("tow,message_type\n100,1\n")
  (run/"ssr/tokyo_run1_expanded.csv").write_text("G,100.0,x\nG,130.0,x\nG,160.0,x\nG,190.0,x\n")
  (run/"pos/tokyo_run1_parity.pos").write_text("% pos\n2200 100 1 2 3 0 0 0 6 10 0\n2200 190 1 2 3 0 0 0 6 10 0\n")
  (root/"out/clas_solution_trajectory.png").write_bytes(b"png")
  (root/"out/clas_solution.kml").write_text("kml")
  score={"matched_epochs":2,"reference_epochs":2,"fixed_epochs":2,"rms2d_fixed_m":0.4,"rms2d_all_m":0.8,"fix_pct":100,"status_counts":{"6":2}}
  (run/"pos/tokyo_run1_parity_summary.json").write_text(json.dumps(score)); (root/"out/scorecard.md").write_text("report")
  return dataset,root/"out",root/"cache"
 @mock.patch.object(mod,"PINS",{})
 @mock.patch.object(mod.subprocess,"run",return_value=mock.Mock(returncode=0))
 def test_pass_and_explicit_loss_failure(self,_):
  with tempfile.TemporaryDirectory() as td:
   dataset,out,cache=self.fixture(Path(td)); argv=["--dataset-root",str(dataset),"--output-dir",str(out),"--l6-cache",str(cache),"--reuse"]
   self.assertEqual(mod.run(argv),0); s=json.loads((out/"summary.json").read_text()); self.assertEqual(s["gate"]["status"],"pass"); self.assertEqual(s["state"],"usable")
   self.assertEqual(mod.run(argv+["--inject-correction-loss"]),1); s=json.loads((out/"summary.json").read_text()); self.assertEqual(s["state"],"unusable"); self.assertIn("correction_unavailable",s["gate"]["failure_reasons"])

 @mock.patch.object(mod,"PINS",{})
 @mock.patch.object(mod.subprocess,"run",return_value=mock.Mock(returncode=0))
 def test_corrections_outside_solution_window_fail_closed(self,_):
  with tempfile.TemporaryDirectory() as td:
   dataset,out,cache=self.fixture(Path(td)); ssr=out/"tokyo_run1/ssr/tokyo_run1_expanded.csv"; ssr.write_text("G,1.0,x\nG,2.0,x\n")
   argv=["--dataset-root",str(dataset),"--output-dir",str(out),"--l6-cache",str(cache),"--reuse"]
   self.assertEqual(mod.run(argv),1); s=json.loads((out/"summary.json").read_text())
   self.assertEqual(s["state"],"unusable"); self.assertIn("correction_unavailable",s["gate"]["failure_reasons"])

 @mock.patch.object(mod,"PINS",{})
 @mock.patch.object(mod.subprocess,"run",return_value=mock.Mock(returncode=0))
 def test_missing_decode_inventory_is_not_a_success(self,_):
  with tempfile.TemporaryDirectory() as td:
   dataset,out,cache=self.fixture(Path(td)); (out/"decoded_messages.csv").unlink()
   argv=["--dataset-root",str(dataset),"--output-dir",str(out),"--l6-cache",str(cache),"--reuse"]
   self.assertEqual(mod.run(argv),1); s=json.loads((out/"summary.json").read_text())
   self.assertIn("decode_message_inventory_missing",s["gate"]["failure_reasons"])
if __name__=="__main__": unittest.main()
