import importlib.util
import json
import subprocess
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('setup_project', ROOT / 'deployment/runtime/setup.py')
setup = importlib.util.module_from_spec(spec)
spec.loader.exec_module(setup)


class SetupTest(unittest.TestCase):
    def test_existing_configuration_is_not_overwritten(self):
        with tempfile.TemporaryDirectory() as temporary:
            p = Path(temporary) / 'local/locations.json'
            old = {'schema_version': 1, 'host_role': 'linux_local', 'custom': 'preserved'}
            self.assertEqual(setup.write_new_json(p, old), old)
            before = p.read_bytes()
            self.assertEqual(setup.write_new_json(p, {'different': True}), old)
            self.assertEqual(p.read_bytes(), before)
            self.assertNotIn(b'\r', before)
            self.assertFalse(before.startswith(b'\xef\xbb\xbf'))

    def test_windows_cannot_use_shared_or_relative_wsl_root(self):
        for p in ('/mnt/c/project', '/mnt/d/project', '../project', 'relative', '/home/user/../project', '/home/user\nproject'):
            with self.subTest(path=p), self.assertRaises(ValueError):
                setup.require_separate_wsl_root(p)
        self.assertEqual(setup.require_separate_wsl_root('/home/user/project with spaces'), '/home/user/project with spaces')

    def test_linux_host_role_cannot_be_overwritten(self):
        with tempfile.TemporaryDirectory() as temporary:
            r = Path(temporary)
            p = r / setup.LOCAL / 'locations.json'
            setup.write_new_json(p, {'host_role': 'windows_simulator'})
            before = p.read_bytes()
            with self.assertRaisesRegex(ValueError, 'host_role'):
                setup.linux_locations(r, '/sdk', '/opt/ros/jazzy', False)
            self.assertEqual(p.read_bytes(), before)

    def test_check_does_not_write_locations_or_install(self):
        with tempfile.TemporaryDirectory() as temporary:
            r = Path(temporary)
            ros = r / 'ros';ros.mkdir();(ros/'setup.bash').touch()
            sdk = r / 'sdk';sdk.mkdir();(sdk/'isaac-sim.sh').touch()
            args = SimpleNamespace(ros_prefix=str(ros), isaac_root=str(sdk), asset_root=None,
                                   ml_python=None, dry_run=False, check=True)
            with patch.dict(setup.os.environ, {}, clear=True), patch.object(setup.shutil, 'which', return_value='/bin/tool'), \
                 patch.object(setup, 'check_installation') as check, patch.object(setup, 'run') as run:
                self.assertEqual(setup.native_setup(args, r), 0)
                check.assert_called_once_with(r, None);run.assert_not_called()
            self.assertFalse((r/setup.LOCAL).exists())

    def test_provisioning_dry_run_has_no_subprocess_or_file_writes(self):
        with tempfile.TemporaryDirectory() as temporary:
            r=Path(temporary);ros=r/'ros';ros.mkdir();(ros/'setup.bash').touch()
            sdk=r/'sdk';sdk.mkdir();(sdk/'isaac-sim.sh').touch()
            args=SimpleNamespace(ros_prefix=str(ros),isaac_root=str(sdk),asset_root=None,
                                 ml_python=None,dry_run=True,check=False)
            with patch.dict(setup.os.environ, {}, clear=True), patch.object(setup.shutil,'which',return_value='/bin/tool'), \
                 patch.object(setup,'run') as run, patch('builtins.print'):
                self.assertEqual(setup.native_setup(args,r),0);run.assert_not_called()
            self.assertFalse((r/setup.LOCAL).exists())


    def test_existing_non_object_configuration_fails_clearly(self):
        with tempfile.TemporaryDirectory() as temporary:
            p=Path(temporary)/'locations.json';p.write_text('[]')
            with self.assertRaisesRegex(ValueError, 'expected a JSON object'):
                setup.read_existing_object(p)
            with self.assertRaisesRegex(ValueError, 'expected a JSON object'):
                setup.write_new_json(p, {})

    def test_mismatched_root_is_preserved(self):
        with tempfile.TemporaryDirectory() as temporary:
            r=Path(temporary)/'current';r.mkdir()
            p=r/setup.LOCAL/'locations.json'
            setup.write_new_json(p, {'host_role':'linux_local','project_root':str(r/'other')})
            before=p.read_bytes()
            with self.assertRaisesRegex(ValueError,'project_root'):
                setup.linux_locations(r,'/sdk','/opt/ros/jazzy',False)
            self.assertEqual(p.read_bytes(),before)

    def test_failed_windows_source_verification_does_not_save_launcher(self):
        with tempfile.TemporaryDirectory() as temporary:
            r=Path(temporary);(r/'.git').mkdir()
            sdk=r/'sdk';sdk.mkdir();(sdk/'isaac-sim.bat').touch()
            helper=r/'deployment/runtime/release_source_manifest.py';helper.parent.mkdir(parents=True)
            helper.write_text("def collect(root): return {'source_sha256':'expected'}\n")
            args=SimpleNamespace(isaac_root=str(sdk),wsl_distribution='Ubuntu-24.04',
                                 wsl_project_root='/home/user/new-project',dry_run=False,check=False,
                                 ml_python=None,asset_root=None,ros_prefix='/opt/ros/jazzy')
            start=SimpleNamespace(require_matching_wsl_source=lambda *a: (_ for _ in ()).throw(ValueError('source differs')))
            with patch.object(setup.shutil,'which',return_value='/tool'), \
                 patch.object(setup,'start_entry',return_value=start), \
                 patch.object(setup.subprocess,'run',return_value=SimpleNamespace(stdout='1',returncode=0)):
                with self.assertRaisesRegex(ValueError,'source differs'):
                    setup.windows_setup(args,r)
            self.assertFalse((r/setup.LOCAL/'launcher.json').exists())
            self.assertFalse((r/setup.LOCAL/'locations.json').exists())

    def test_old_python_has_no_setup_side_effects(self):
        with patch.object(setup.sys,'version_info',(3,10)), patch.object(setup,'windows_setup') as child:
            self.assertEqual(setup.main([]),2)
            child.assert_not_called()
        with patch.object(setup.sys,'version_info',(3,10)), patch.object(setup.subprocess,'run') as run:
            with self.assertRaisesRegex(ValueError,'Python 3.12'):
                setup.windows_setup(None,Path('does-not-exist'))
            run.assert_not_called()

    def test_windows_check_rejects_drift_before_assets_or_install(self):
        with tempfile.TemporaryDirectory() as temporary:
            r=Path(temporary);(r/'.git').mkdir()
            sdk=r/'sdk';sdk.mkdir();(sdk/'isaac-sim.bat').touch()
            args=SimpleNamespace(isaac_root=str(sdk),wsl_distribution='Ubuntu',
                                 wsl_project_root='/home/user/project',dry_run=False,check=True)
            from unittest.mock import Mock
            start=Mock();start.require_matching_wsl_source.side_effect=ValueError('source differs')
            with patch.object(setup.shutil,'which',return_value='/tool'), \
                 patch.object(setup,'start_entry',return_value=start), patch.object(setup,'run') as run:
                with self.assertRaisesRegex(ValueError,'source differs'):
                    setup.windows_setup(args,r)
                run.assert_not_called()
            self.assertFalse((r/setup.LOCAL).exists())

    def test_windows_stages_demo_on_host_and_passes_only_local_path_to_wsl(self):
        with tempfile.TemporaryDirectory() as temporary:
            r=Path(temporary);(r/'.git').mkdir()
            sdk=r/'sdk';sdk.mkdir();(sdk/'isaac-sim.bat').touch()
            demo=r/'model with spaces.zip';demo.write_bytes(b'fixture')
            args=SimpleNamespace(isaac_root=str(sdk),wsl_distribution='Ubuntu',
                wsl_project_root='/home/user/project',dry_run=False,check=False,
                demo_model_bundle=demo,asset_root=None,ml_python=None,ros_prefix='/opt/ros/jazzy')
            from unittest.mock import Mock
            start=Mock()
            def child(argv,**kwargs):
                return SimpleNamespace(stdout='/mnt/d/model with spaces.zip' if 'wslpath' in argv else '1',returncode=0)
            with patch.object(setup.shutil,'which',return_value='/tool'), \
                 patch.object(setup,'start_entry',return_value=start), \
                 patch.object(setup.subprocess,'run',side_effect=child) as wsl,patch.object(setup,'run') as run:
                self.assertEqual(setup.windows_setup(args,r),0)
            self.assertTrue(any('deployment/runtime/default_model.py' in str(c.args[0]) for c in run.call_args_list))
            calls=[c.args[0] for c in wsl.call_args_list]
            backend=next(c for c in calls if '--demo-model-bundle' in c)
            self.assertEqual(backend[backend.index('--demo-model-bundle')+1],'/mnt/d/model with spaces.zip')
            self.assertNotIn('GH_TOKEN',str(calls));self.assertNotIn('GITHUB_TOKEN',str(calls))

if __name__ == '__main__':
    unittest.main()
