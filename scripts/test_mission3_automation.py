import json
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path
import unittest
from unittest.mock import patch
from mission3_automation import BatchManager,distances,validate_options,count_contacts


class AutomationTests(unittest.TestCase):
    def test_lowercase_evaluator_verdict_and_readonly_restart_restore(self):
        with tempfile.TemporaryDirectory() as folder,patch('mission3_automation.OUTPUT',Path(folder)):
            p=Path(folder)/'20261009-example';p.mkdir()
            saved={'status':'finished','batch_id':p.name,'cases':[{'contact_status':'pass'},
                {'contact_status':'fail'},{}],'conditional_contacts':0}
            source=json.dumps(saved);(p/'progress.json').write_text(source)
            self.assertEqual(count_contacts(saved['cases']),1)
            manager=BatchManager();self.assertEqual(manager.state['conditional_contacts'],1)
            self.assertEqual(manager.batch,p);self.assertEqual((p/'progress.json').read_text(),source)
    def test_fixed_cm_grid_and_invalid_values(self):
        self.assertEqual(distances(1.25,.5),[round(1.25-i*.05,2) for i in range(16)])
        for start,end in ((1.25,.51),(.5,1.25),(1.3,.5),(1.25,.45),(float('nan'),.5),(True,.5)):
            with self.assertRaises(ValueError):distances(start,end)

    def test_request_cannot_add_shell_path_or_unbounded_workers(self):
        for settings in ({'cmd':'echo anything'},{'workers':5},{'seed':False},{'repeats':0},{'render_video':'true'}):
            with self.assertRaises(ValueError):validate_options(settings)

    def test_plan_retains_all_distances_and_paired_seeds_without_launch(self):
        with tempfile.TemporaryDirectory() as folder,patch('mission3_automation.OUTPUT',Path(folder)):
            m=BatchManager();m.start({},execute=False)
            plan=json.loads((m.batch/'plan.json').read_text())
            self.assertEqual(len(plan['cases']),16);self.assertEqual(m.state['status'],'planned')
            self.assertEqual({c['seed'] for c in plan['cases']},{1001})
            self.assertEqual(m.active,{})
            cfg=json.loads((m.batch/'d125-r01/config.json').read_text())
            self.assertEqual(cfg['entry']['positions'][0]['xy_m'],[2.,1.25])
            cfg=json.loads((m.batch/'d050-r01/config.json').read_text())
            self.assertEqual(cfg['entry']['positions'][0]['xy_m'],[2.,2.])
            m.verify()
            (m.batch/'profile.json').write_text('{}')
            with self.assertRaises(RuntimeError):m.verify()

    def test_stop_terminates_owned_process_only(self):
        with tempfile.TemporaryDirectory() as folder:
            m=BatchManager();m.batch=Path(folder);(m.batch/'case').mkdir()
            other=subprocess.Popen([sys.executable,'-c','import time;time.sleep(30)'])
            try:
                worker=threading.Thread(target=m.command,args=({'case_id':'case'},
                    [sys.executable,'-c','import time;time.sleep(30)'],'flight.log',30))
                worker.start()
                for _ in range(100):
                    if m.active:break
                    time.sleep(.01)
                self.assertTrue(m.active);m.stop();worker.join(8)
                self.assertFalse(worker.is_alive());self.assertEqual(m.active,{})
                self.assertIsNone(other.poll())
            finally:other.terminate();other.wait()


if __name__=='__main__':unittest.main()
