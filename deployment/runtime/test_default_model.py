import hashlib
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch, Mock
import urllib.request
import zipfile

ROOT = Path(__file__).resolve().parents[2]
MODULE = importlib.util.spec_from_file_location('default_model_setup', ROOT / 'deployment/runtime/default_model.py')
demo = importlib.util.module_from_spec(MODULE)
MODULE.loader.exec_module(demo)


class DefaultModelTest(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.storage = self.root / 'storage'
        version, target = '1' * 32, '2' * 32
        weights = b'known-demo-weight-bytes'
        row = dict(model_version=version, target_id=target, state='published', bundle_format=1,
                   checkpoint_sha256=hashlib.sha256(weights).hexdigest(), class_names=['Sample'],
                   class_mapping={'0':'Sample'}, preprocessing={'architecture':'RFDETRSegSmall'},
                   evaluation={}, source={})
        row = demo.bundle_module(ROOT).seal(row)
        self.archive = self.root / 'demo.zip'
        with zipfile.ZipFile(self.archive, 'w') as zipped:
            zipped.writestr(f'models/{version}/checkpoint.pth', weights)
            zipped.writestr(f'models/{version}/manifest.json', json.dumps(row))
            zipped.writestr(f'targets/{target}/profile.json', '{}')
            zipped.writestr('MODEL_CARD.md', 'Test only')
            zipped.writestr('SHA256SUMS', '')
        self.spec = dict(schema_version=1, name='Sample', model_version=version, target_id=target,
                         checkpoint_sha256=row['checkpoint_sha256'], checkpoint_bytes=len(weights),
                         contract_sha256=row['contract_sha256'], archive_name='demo.zip',
                         archive_bytes=self.archive.stat().st_size, archive_sha256=demo.digest(self.archive),
                         asset_api_url='https://api.github.com/repos/example/test/releases/assets/1')

    def test_install_registers_self_contained_model_and_retains_target_draft(self):
        profile = self.storage / 'targets' / self.spec['target_id'] / 'profile.json'
        profile.parent.mkdir(parents=True)
        original = json.dumps(dict(target_id=self.spec['target_id'], name='User name',
                                   images=[{'image_id':'retained'}], latest_job_id='retained-job')).encode()
        profile.write_bytes(original)
        demo.install(ROOT, self.storage, self.spec, self.archive)
        demo.check_installed(ROOT, self.storage, self.spec)
        self.assertEqual(profile.read_bytes(), original)
        self.assertFalse((self.storage / 'jobs').exists())
        self.assertTrue((self.storage/'models'/self.spec['model_version']/'MODEL_CARD.md').is_file())

    def test_repeat_needs_no_archive_or_network_and_new_target_has_no_job(self):
        demo.install(ROOT, self.storage, self.spec, self.archive)
        self.archive.unlink()
        with patch.object(demo, 'acquire', side_effect=AssertionError('Unexpected download')):
            demo.install(ROOT, self.storage, self.spec)
        profile = json.loads((self.storage / 'targets' / self.spec['target_id'] / 'profile.json').read_text())
        self.assertEqual(profile['images'], [])
        self.assertIsNone(profile['latest_job_id'])

    def test_corrupt_existing_weights_are_preserved_and_refused(self):
        demo.install(ROOT, self.storage, self.spec, self.archive)
        weights = self.storage / 'models' / self.spec['model_version'] / 'checkpoint.pth'
        weights.write_bytes(b'tampered')
        with self.assertRaises(ValueError):
            demo.install(ROOT, self.storage, self.spec, self.archive)
        self.assertEqual(weights.read_bytes(), b'tampered')

    def test_wrong_archive_hash_writes_no_model(self):
        self.archive.write_bytes(b'bad')
        with self.assertRaisesRegex(ValueError, 'SHA-256'):
            demo.install(ROOT, self.storage, self.spec, self.archive)
        self.assertFalse((self.storage / 'models' / self.spec['model_version']).exists())

    def test_traversal_entry_is_rejected_even_with_matching_archive_hash(self):
        with zipfile.ZipFile(self.archive, 'a') as zipped:
            zipped.writestr('../outside', 'bad')
        self.spec['archive_sha256'] = demo.digest(self.archive)
        with self.assertRaisesRegex(ValueError, 'Unexpected paths'):
            demo.install(ROOT, self.storage, self.spec, self.archive)
        self.assertFalse((self.root / 'outside').exists())

    def test_descriptor_rejects_escaping_model_identity(self):
        config=self.root/'config/default_model.json';config.parent.mkdir()
        self.spec['model_version']='../escape';config.write_text(json.dumps(self.spec))
        with self.assertRaisesRegex(ValueError,'identity'):
            demo.descriptor(self.root)

    def test_offline_bundle_never_requests_credentials_or_network(self):
        with patch.object(demo, 'token', side_effect=AssertionError('Unexpected credentials')), \
             patch.object(demo.urllib.request, 'build_opener', side_effect=AssertionError('Unexpected network')):
            self.assertEqual(demo.acquire(self.root, self.spec, self.archive), self.archive.resolve())

    def test_verified_download_and_repeat_cache(self):
        opener = Mock();opener.open.return_value = io.BytesIO(self.archive.read_bytes())
        with patch.object(demo.urllib.request, 'build_opener', return_value=opener):
            path = demo.acquire(self.root, self.spec)
        self.assertEqual(demo.digest(path), self.spec['archive_sha256'])
        with patch.object(demo.urllib.request, 'build_opener', side_effect=AssertionError('Unexpected network')):
            self.assertEqual(demo.acquire(self.root, self.spec), path)

    def test_multiple_ranges_are_reassembled_with_exact_offsets(self):
        data=b'x'*(4*1024*1024)+b'end'
        self.spec['archive_bytes']=len(data)
        self.spec['archive_sha256']=hashlib.sha256(data).hexdigest()
        def response(request,**kwargs):
            value=request.get_header('Range');left,right=map(int,value.removeprefix('bytes=').split('-'))
            chunk=io.BytesIO(data[left:right+1]);chunk.status=206
            chunk.headers={'Content-Range':f'bytes {left}-{right}/{len(data)}'}
            return chunk
        opener=Mock();opener.open.side_effect=response
        with patch.object(demo.urllib.request,'build_opener',return_value=opener):
            result=demo.acquire(self.root,self.spec)
        self.assertEqual(result.read_bytes(),data)
        self.assertEqual(opener.open.call_count,2)

    def test_wrong_range_is_refused_without_cache(self):
        response=io.BytesIO(self.archive.read_bytes());response.status=206
        response.headers={'Content-Range':'bytes 1-2/3'}
        opener=Mock();opener.open.return_value=response
        with patch.object(demo.urllib.request,'build_opener',return_value=opener):
            with self.assertRaisesRegex(ValueError,'byte range'):
                demo.acquire(self.root,self.spec)
        self.assertFalse(demo.cache_path(self.root,self.spec).exists())

    def test_failed_download_leaves_no_partial_or_registered_model(self):
        data = bytearray(self.archive.read_bytes());data[-1] ^= 1
        opener = Mock();opener.open.return_value = io.BytesIO(bytes(data))
        with patch.object(demo.urllib.request, 'build_opener', return_value=opener):
            with self.assertRaisesRegex(ValueError, 'SHA-256'):
                demo.acquire(self.root, self.spec)
        self.assertFalse(demo.cache_path(self.root, self.spec).exists())
        self.assertEqual(list((self.root / 'artifacts/default-model-cache').glob('*.partial')), [])

    def test_credentials_are_not_forwarded_to_storage_redirect(self):
        handler = demo.SafeRedirect()
        request = urllib.request.Request('https://api.github.com/repos/example/test/releases/assets/1',
                                         headers={'Authorization':'Bearer private-test'})
        redirected = handler.redirect_request(request, None, 302, 'Found', {},
                                               'https://release-assets.githubusercontent.com/a')
        self.assertIsNone(redirected.get_header('Authorization'))
        with self.assertRaisesRegex(ValueError, 'Unexpected'):
            handler.redirect_request(request, None, 302, 'Found', {}, 'https://example.com/private')


if __name__ == '__main__':
    unittest.main()
