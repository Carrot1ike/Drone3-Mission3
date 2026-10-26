import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import setup_mission3 as setup
import mission3_automation as automation


class BootstrapTests(unittest.TestCase):
    def test_existing_shared_link_is_never_repointed(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)/'mission with spaces'; root.mkdir()
            first = Path(folder)/'shared first'; first.mkdir()
            second = Path(folder)/'shared second'; second.mkdir()
            with patch.object(setup, 'ROOT', root):
                setup.link_shared(first)
                setup.link_shared(first)  # 반복 설치도 같은 링크를 유지합니다.
                with self.assertRaises(ValueError): setup.link_shared(second)
            self.assertEqual((root/'src/crazyflie-ros2-sil').resolve(), first)
            self.assertEqual(list(first.iterdir()), [])

    def test_existing_foreign_environment_is_not_installed_into(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder); env = root/'.venv-mission3'; env.mkdir()
            untouched = env/'foreign-data'; untouched.write_text('preserve')
            with patch.object(setup, 'ROOT', root), patch.object(setup, 'run') as runner:
                with self.assertRaises(ValueError): setup.prepare_environment(Path('/python'))
                runner.assert_not_called()
            self.assertEqual(untouched.read_text(), 'preserve')

    def test_symlink_environment_cannot_modify_shared_environment(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)/'mission'; root.mkdir()
            shared = Path(folder)/'shared-env'; shared.mkdir()
            (shared/'.mission3-owned.json').write_text(json.dumps({'root':str(root)}))
            (root/'.venv-mission3').symlink_to(shared)
            with patch.object(setup, 'ROOT', root), patch.object(setup, 'run') as runner:
                with self.assertRaises(ValueError): setup.prepare_environment(Path('/python'))
                runner.assert_not_called()

    def test_moved_venv_is_rejected_before_pip(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder); env = root/'.venv-mission3'; env.mkdir()
            (env/'.mission3-owned.json').write_text(json.dumps({'root':'/old/location'}))
            with patch.object(setup, 'ROOT', root), patch.object(setup, 'run') as runner:
                with self.assertRaises(ValueError): setup.prepare_environment(Path('/python'))
                runner.assert_not_called()

    def test_missing_shared_installation_has_actionable_error(self):
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaisesRegex(ValueError, 'shared-root'):
                setup.validate_shared(Path(folder))

    def test_children_use_same_interpreter_and_selected_backend(self):
        self.assertEqual(str(automation.PYTHON), sys.executable)
        with tempfile.TemporaryDirectory() as folder, patch.dict(os.environ, {'MUJOCO_GL':'osmesa'}):
            manager = automation.BatchManager(); manager.batch = Path(folder)
            (manager.batch/'case').mkdir()
            result = manager.command({'case_id':'case'}, [str(automation.PYTHON), '-c',
                'import os,sys,json;print(json.dumps([sys.executable,os.environ["MUJOCO_GL"]]))'], 'probe.log', 10)
            self.assertEqual(result, 0)
            self.assertEqual(json.loads((manager.batch/'case/probe.log').read_text()), [sys.executable,'osmesa'])


if __name__ == '__main__': unittest.main()
