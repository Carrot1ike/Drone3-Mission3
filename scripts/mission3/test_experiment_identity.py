"""Holdout config identity must vary with model settings, not RNG trial seed."""
from copy import deepcopy
import hashlib
import json
import unittest
from run_mission3_physical import frozen_experiment_settings
from .settings import ROOT,load

class ExperimentIdentity(unittest.TestCase):
    def test_configuration_lock_separates_trial_seed_from_model(self):
        cfg=load();physical=json.loads((ROOT/"config/mission3-inertial-texture-bridge-sensitivity.json").read_text())
        settings=frozen_experiment_settings(cfg,physical,True)
        self.assertNotIn("seed",settings);self.assertNotIn("entry_id",settings)
        original=json.dumps(settings,sort_keys=True)
        changed=deepcopy(physical);changed["sensor"]["texture_quality_sensitivity"]["virtual_fov_deg"]=43.
        self.assertNotEqual(original,json.dumps(frozen_experiment_settings(cfg,changed,True),sort_keys=True))
        self.assertNotEqual(original,json.dumps(frozen_experiment_settings(cfg,physical,False),sort_keys=True))

if __name__=="__main__":unittest.main()
