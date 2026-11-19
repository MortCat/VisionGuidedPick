import json
import shutil
import tempfile
import unittest
from pathlib import Path
from release_source_manifest import collect


class SourceManifestTest(unittest.TestCase):
    def test_windows_launch_helpers_are_part_of_source_identity(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            helper = root / 'deployment/windows/start_isaac.ps1'
            helper.parent.mkdir(parents=True)
            helper.write_text('first')
            first = collect(root)
            self.assertIn('deployment/windows/start_isaac.ps1', first['files'])
            helper.write_text('changed')
            self.assertNotEqual(first['source_sha256'], collect(root)['source_sha256'])

    def test_relocation_change_detection_and_host_data_exclusion(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary) / 'one'
            code = root / 'ros2_ws/src/demo/main.cpp'
            code.parent.mkdir(parents=True)
            code.write_text('int main() {}\n', encoding='utf-8')
            local = root / 'deployment/runtime/local/locations.json'
            local.parent.mkdir(parents=True)
            local.write_text('{"private_path":"host-only"}')
            generated = root / 'config/factory_demo/arm_01.semantic.yaml'
            generated.parent.mkdir(parents=True)
            generated.write_text('generated: before')
            original = collect(root)
            generated.write_text('generated: after')
            self.assertEqual(original['source_sha256'], collect(root)['source_sha256'])
            moved = root.with_name('two')
            shutil.copytree(root, moved)
            self.assertEqual(original['source_sha256'], collect(moved)['source_sha256'])
            self.assertNotIn(str(root), str(original))
            self.assertNotIn(local.relative_to(root).as_posix(), original['files'])
            code.write_text('int main() {return 1;}\n', encoding='utf-8')
            self.assertNotEqual(original['source_sha256'], collect(root)['source_sha256'])


    def test_authored_scene_dependencies_are_hashed_and_required(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            package = root / 'deployment/factory_rabbit_20261005'
            asset = package / 'scene/clutter/bear.usdc'
            asset.parent.mkdir(parents=True)
            asset.write_bytes(b'first scene bytes')
            manifest = package / 'production_manifest.json'
            payload = {'usd_dependencies': ['scene/clutter/bear.usdc',
                       'assets/official/isaac_6_1/not-acquired.usd'],
                       'runtime_sidecars': []}
            manifest.write_text(json.dumps(payload))
            original = collect(root)
            self.assertIn(asset.relative_to(root).as_posix(), original['files'])
            asset.write_bytes(b'updated scene bytes')
            self.assertNotEqual(original['source_sha256'], collect(root)['source_sha256'])
            asset.unlink()
            with self.assertRaisesRegex(ValueError, 'Missing authored scene dependency'):
                collect(root)
            payload['usd_dependencies'] = ['../outside.usda']
            manifest.write_text(json.dumps(payload))
            with self.assertRaisesRegex(ValueError, 'Invalid scene dependency path'):
                collect(root)


if __name__ == '__main__':
    unittest.main()
