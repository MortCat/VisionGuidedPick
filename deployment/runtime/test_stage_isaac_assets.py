import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import stage_isaac_assets as stage


class AssetStagingTest(unittest.TestCase):
    def test_copy_existing_and_source_tamper(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);package=root/'package';source=root/'source'
            package.mkdir();asset=source/'Isaac/test.usd';asset.parent.mkdir(parents=True)
            asset.write_bytes(b'pinned asset')
            name='assets/official/isaac_6_1/Isaac/test.usd'
            (package/'production_manifest.json').write_text(json.dumps({'usd_dependencies':[name]}))
            checks=root/'checks.json'
            checks.write_text(json.dumps({'schema_version':1,'sha256':{name:hashlib.sha256(asset.read_bytes()).hexdigest()}}))
            with patch.object(stage,'PACKAGE',package),patch.object(stage,'CHECKSUMS',checks):
                self.assertEqual(stage.stage_assets(source,check=True),(1,0))
                self.assertFalse((package/name).exists())
                self.assertEqual(stage.stage_assets(source),(1,1))
                self.assertEqual(stage.stage_assets(source),(1,0))
                (package/name).write_bytes(b'corrupted existing copy')
                with self.assertRaisesRegex(ValueError,'Existing destination checksum mismatch'):
                    stage.stage_assets(source)
                (package/name).unlink();asset.write_bytes(b'different asset version')
                with self.assertRaisesRegex(ValueError,'Source checksum mismatch'):
                    stage.stage_assets(source)
                self.assertFalse((package/name).exists())

    def test_mdl_indirect_texture_must_be_pinned_and_present(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);package=root/'package';source=root/'source'
            package.mkdir();mdl=source/'Isaac/Materials/Metal.mdl';mdl.parent.mkdir(parents=True)
            mdl.write_text('uniform texture_2d map=texture_2d("./Metal/BaseColor.png");')
            texture=source/'Isaac/Materials/Metal/BaseColor.png';texture.parent.mkdir();texture.write_bytes(b'texture')
            m='assets/official/isaac_6_1/Isaac/Materials/Metal.mdl'
            t='assets/official/isaac_6_1/Isaac/Materials/Metal/BaseColor.png'
            manifest=package/'production_manifest.json';checks=root/'checks.json'
            manifest.write_text(json.dumps({'usd_dependencies':[m]}))
            hashes={m:hashlib.sha256(mdl.read_bytes()).hexdigest()}
            checks.write_text(json.dumps({'schema_version':1,'sha256':hashes}))
            with patch.object(stage,'PACKAGE',package),patch.object(stage,'CHECKSUMS',checks):
                with self.assertRaisesRegex(ValueError,'Untracked material dependency'):
                    stage.stage_assets(source,check=True)
                self.assertFalse((package/m).exists())
                manifest.write_text(json.dumps({'usd_dependencies':[m,t]}))
                hashes[t]=hashlib.sha256(texture.read_bytes()).hexdigest()
                checks.write_text(json.dumps({'schema_version':1,'sha256':hashes}))
                self.assertEqual(stage.stage_assets(source),(2,2))
                self.assertEqual(stage.verify_assets(),(2,0))
                (package/t).unlink()
                with self.assertRaisesRegex(ValueError,'Missing scene assets'):
                    stage.verify_assets()



class AssetDownloadTest(unittest.TestCase):
    def setUp(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        root = Path(folder.name)
        self.package = root / 'package'
        self.package.mkdir()
        self.name = 'assets/official/isaac_6_1/Isaac/test.usd'
        self.payload = b'pinned official asset'
        (self.package / 'production_manifest.json').write_text(json.dumps({'usd_dependencies': [self.name]}))
        checks = root / 'checks.json'
        checks.write_text(json.dumps({'schema_version': 1, 'sha256': {
            self.name: hashlib.sha256(self.payload).hexdigest()}}))
        for name, value in (('PACKAGE', self.package), ('CHECKSUMS', checks)):
            patched = patch.object(stage, name, value)
            patched.start()
            self.addCleanup(patched.stop)

    def test_download_verify_and_reuse_without_network(self):
        with self.assertRaisesRegex(ValueError, 'Missing scene assets'):
            stage.verify_assets()
        with patch.object(stage.urllib.request, 'urlopen', return_value=io.BytesIO(self.payload)) as request:
            self.assertEqual(stage.download_assets(), (1, 1))
            self.assertEqual(request.call_args.args[0], stage.OFFICIAL_ROOT + 'Isaac/test.usd')
        self.assertEqual(stage.verify_assets(), (1, 0))
        with patch.object(stage.urllib.request, 'urlopen') as request:
            self.assertEqual(stage.download_assets(), (1, 0))
            request.assert_not_called()
        (self.package / self.name).write_bytes(b'locally changed')
        with patch.object(stage.urllib.request, 'urlopen') as request:
            with self.assertRaisesRegex(ValueError, 'Existing destination checksum mismatch'):
                stage.download_assets()
            request.assert_not_called()
        self.assertEqual((self.package / self.name).read_bytes(), b'locally changed')

    def test_reviewed_legacy_copy_is_accepted_only_as_a_local_source(self):
        legacy = b'reviewed equivalent legacy USD serialization'
        checks = stage.CHECKSUMS
        record = json.loads(checks.read_text())
        record['legacy_sha256'] = {self.name: hashlib.sha256(legacy).hexdigest()}
        checks.write_text(json.dumps(record))
        with patch.object(stage.urllib.request, 'urlopen', return_value=io.BytesIO(legacy)):
            with self.assertRaisesRegex(ValueError, 'Official asset checksum mismatch'):
                stage.download_assets()
        source = self.package.parent / 'offline'
        path = source / 'Isaac/test.usd'
        path.parent.mkdir(parents=True)
        path.write_bytes(legacy)
        self.assertEqual(stage.stage_assets(source), (1, 1))
        self.assertEqual(stage.verify_assets(), (1, 0))
        with patch.object(stage.urllib.request, 'urlopen') as request:
            self.assertEqual(stage.download_assets(), (1, 0))
            request.assert_not_called()

    def test_failed_download_never_installs_partial_or_wrong_file(self):
        with patch.object(stage.urllib.request, 'urlopen', return_value=io.BytesIO(b'wrong version')):
            with self.assertRaisesRegex(ValueError, 'Official asset checksum mismatch'):
                stage.download_assets()
        destination = self.package / self.name
        self.assertFalse(destination.exists())
        self.assertEqual(list(destination.parent.iterdir()), [])
        class Interrupted(io.BytesIO):
            def read(self, *args):
                raise OSError('connection lost')
        with patch.object(stage.urllib.request, 'urlopen', return_value=Interrupted(self.payload)):
            with self.assertRaisesRegex(ValueError, 'Rerun to continue'):
                stage.download_assets()
        self.assertFalse(destination.exists())
        self.assertEqual(list(destination.parent.iterdir()), [])


if __name__=='__main__':
    unittest.main()
