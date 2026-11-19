"""Contract tests with a real owned subprocess, without ROS/GPU or robot commands."""
from pathlib import Path
import hashlib
import importlib.util
import json
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace
import yaml
from fastapi.testclient import TestClient

PROJECT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('teach', PROJECT / 'tools/teach_target/service.py')
service = importlib.util.module_from_spec(spec)
spec.loader.exec_module(service)

WORKER = '''import json,sys,base64
for line in sys.stdin:
 c=json.loads(line)
 if c['command']=='load_model':
  m=c['model'];activation=c['activation_id']
  print(json.dumps(dict(event='loaded',activation_id=activation,model_version=m['model_version'],checkpoint_sha256=m['checkpoint_sha256'],inference_contract={'class_mapping':m['class_mapping']})),flush=True)
 elif c['command']=='start_run':
  print(json.dumps(dict(event='started',run_id=c['run_id'])),flush=True)
  print(json.dumps(dict(event='wrist_preview',run_id=c['run_id'],arm='arm_02',jpeg=base64.b64encode(b'wrist-preview').decode())),flush=True)
  for i in range(11):
   s=dict(run_id=c['run_id'],model_version=m['model_version'],frame_key='frame_'+str(i),clock_epoch='1' if i<10 else '2',objects=[],capture_stamp_ns=str(i),original_size={'width':1280,'height':720},preview_size={'width':640,'height':360})
   if i==9:s['objects']=[dict(display_confirmed=True,ros_track_id='9',display_id='9',label='Rabbit')]
   print(json.dumps(dict(event='frame',run_id=c['run_id'],snapshot=s,rgb=base64.b64encode(b'rgb'+str(i).encode()).decode(),mask=base64.b64encode(b'mask'+str(i).encode()).decode())),flush=True)
 elif c['command']=='stop_run':break
'''


def wait_for(fn, predicate):
    for _ in range(100):
        value = fn()
        if predicate(value): return value
        time.sleep(.03)
    raise AssertionError(value)


with tempfile.TemporaryDirectory() as directory:
    directory = Path(directory)
    store = directory / 'store'
    model = store / 'models/version1'
    model.mkdir(parents=True)
    checkpoint = model / 'checkpoint.pth'
    checkpoint.write_bytes(b'contract-test-checkpoint')
    legacy = dict(model_version='version1', target_id='target1', job_id='job1', state='published',
                  checkpoint_sha256=hashlib.sha256(checkpoint.read_bytes()).hexdigest(), class_names=['Rabbit'])
    (model / 'manifest.json').write_text(json.dumps(legacy))
    job = store / 'jobs/job1'
    (job / 'dataset/train').mkdir(parents=True)
    (job / 'dataset/train/_annotations.coco.json').write_text(json.dumps({'categories':[{'id':1,'name':'Rabbit'}]}))
    (job / 'result.json').write_text(json.dumps({'metrics':{'mean_union_iou':0}}))
    (job / 'resolved_config.yaml').write_text(yaml.safe_dump({'training': {'resolution': 432}}))
    bundle_spec = importlib.util.spec_from_file_location('bundle', PROJECT / 'tools/teach_target/model_bundle.py')
    model_bundle = importlib.util.module_from_spec(bundle_spec)
    bundle_spec.loader.exec_module(model_bundle)
    # Explicit migration is the only path from a legacy manifest; then remove the job
    # record to prove activation is independent of training data.
    manifest, migrated = model_bundle.migrate(store, 'version1')
    assert migrated and json.loads((model / 'manifest.legacy.json').read_text()) == legacy
    assert manifest['state'] == 'published' and manifest['preprocessing']['resolution'] == 432
    import shutil
    shutil.rmtree(store / 'jobs')
    legacy_model = store / 'models/legacy1'
    legacy_model.mkdir()
    (legacy_model / 'checkpoint.pth').write_bytes(b'contract-test-checkpoint')
    (legacy_model / 'manifest.json').write_text(json.dumps(dict(legacy, model_version='legacy1')))
    cfg = yaml.safe_load((PROJECT / 'config/teaching.yaml').read_text())
    cfg['server']['allowed_ips'] = ['testclient']
    config = directory / 'config.yaml'
    config.write_text(yaml.safe_dump(cfg))
    worker = directory / 'worker.py'
    worker.write_text(WORKER)
    def factory():
        return subprocess.Popen([sys.executable, '-u', str(worker)], stdin=subprocess.PIPE,
                                stdout=subprocess.PIPE, text=True, encoding='utf-8')
    app = service.create_app(config, store, observe_factory=factory)
    with TestClient(app) as client:
        def post(path, data, key): return client.post(path, json=data, headers={'Idempotency-Key':key})
        assert client.get('/api/observe/status').json()['active_model_version'] is None
        assert '/api/observe/runs' in client.get('/openapi.json').json()['paths']
        assert client.post('/api/observe/model-activations',content='{',headers={'Content-Type':'application/json','Idempotency-Key':'malformed'}).status_code==400
        assert client.post('/api/observe/model-activations',json={'model_version':'version1','threshold':0},headers={'Idempotency-Key':'extra-fields'}).status_code==422
        assert [row['model_version'] for row in client.get('/api/teach/models').json()] == ['legacy1', 'version1']
        refused = post('/api/observe/model-activations', {'model_version':'legacy1'}, 'legacy')
        assert refused.status_code == 409 and refused.json()['error']['code'] == 'MODEL_BUNDLE_LEGACY'
        assert json.loads((legacy_model / 'manifest.json').read_text())['model_version'] == 'legacy1'
        assert 'bundle_format' not in json.loads((legacy_model / 'manifest.json').read_text())
        assert post('/api/observe/runs', {'activation_id':'missing'}, 'missing').status_code == 409
        assert post('/api/observe/model-activations', {'model_version':'missing'}, 'missing-model').status_code == 404
        activation = post('/api/observe/model-activations', {'model_version':'version1'}, 'load')
        assert activation.status_code == 202, activation.text
        aid = activation.json()['activation_id']
        assert post('/api/observe/model-activations', {'model_version':'version1'}, 'load').json()['activation_id'] == aid
        ack = wait_for(lambda:client.get('/api/observe/model-activations/'+aid).json(),lambda x:x['state']!='loading')
        assert ack['inference_contract']['class_mapping']=={'0':'Rabbit'}
        assert ack['state']=='active' and ack['acknowledged_checkpoint_sha256']==manifest['checkpoint_sha256'], ack
        assert post('/api/observe/model-activations', {'model_version':'version1'}, 'second').status_code==409
        assert post('/api/observe/model-activations', {'model_version':'version2'}, 'load').status_code==409
        run = post('/api/observe/runs', {'activation_id':aid}, 'start').json()
        rid=run['run_id']
        assert post('/api/observe/runs', {'activation_id':aid}, 'start').json()['run_id']==rid
        snapshot=wait_for(lambda:client.get(f'/api/observe/runs/{rid}/snapshot').json(),lambda x:x.get('frame_key')=='frame_10')
        assert snapshot['objects']==[] and snapshot['clock_epoch']=='2'
        assert client.get(snapshot['rgb_url']).content==b'rgb10'
        assert client.get(snapshot['mask_url']).content==b'mask10'
        assert client.get('/api/observe/frames/frame_9/preview').status_code==410
        assert len(app.state.observe.frames)==1  # Clock reset invalidates earlier epoch.
        assert client.get('/api/observe/status').json()['motion_control'] is False
        path = f'/api/observe/runs/{rid}/objects'
        assert client.get(path).json() == {'run_id': rid, 'detected_count': 0,
                                          'outcome_counts': dict(placed=0, failed=0, unverified=0),
                                          'objects': []}
        observer = app.state.observe
        with observer.lock:
            def finish_pick(track, arm, cycle, outcome):
                observer._update_object_outcome(
                    arm, dict(track_id=track, phase=4, epoch='2', cycle_id=cycle), None)
                observer._update_object_outcome(
                    arm, dict(track_id='0', phase=1, epoch='2', cycle_id=cycle,
                              last_result=dict(track_id=track, epoch='2', cycle_id=cycle, outcome=outcome)), None)

            for i in range(105):
                observer._remember_objects(dict(capture_stamp_ns=str(i), objects=[
                    dict(display_confirmed=True, ros_track_id=str(2**60 + i),
                         display_id=str(i), label='Rabbit'),
                    dict(display_confirmed=False, ros_track_id='1', display_id='1', label='Candidate'),
                ]))
                if i in (0, 102, 103, 104):
                    finish_pick(str(2**60 + i), 'arm_01', str(i + 100), 1 if i == 104 else 5)
            track_id = str(2**60 + 103)
            task = dict(track_id=track_id, phase=4, epoch='2', cycle_id='7')
            observer._update_object_outcome('arm_02', task, {'possession': 'holding'})
            observer._update_object_outcome('arm_02', dict(
                task, phase=1, cycle_id='8', track_id='0',
                last_result=dict(epoch='2', cycle_id='7', track_id=track_id, outcome=1)),
                {'possession': 'empty'})
            assert observer.object_history[track_id]['state'] == 'unverified'
            observer._accept_bin_placement('arm_02', dict(
                epoch='2', cycle_id='7', track_id=track_id, state='placed',
                reason='Stable increase in bin load', delta_n=.54))
            observer.pick_status['arm_02'] = dict(available=True, task=dict(task, phase=0),
                                                  gripper={'possession': 'empty'})
            observer.pick_seen['arm_02'] = time.monotonic()
        with observer.lock:
            identity = dict(epoch='2', cycle_id='7', track_id=track_id)
            frame_id = time.monotonic_ns()
            grasp = dict(identity, points=[[.5, .5], [.4, .5], [.6, .5]])
            observer.wrist_previews['arm_02'][frame_id] = (b'paired-rgb', grasp)
            observer.gantry_surface_epoch = '2'
            observer.gantry_surfaces[track_id] = dict(track_id=track_id, scans=3, revision=17,
                                                       latest=[[1, 2, 3]], accumulated=[[1, 2, 3]])
            observer.pick_status['arm_02']['task'] = dict(identity, phase=4, planned_grasp_active=True)
        pick_path = '/api/auto-pick/status?arm=arm_02'
        pick = client.get(pick_path).json()
        assert pick['wrist_camera']['grasp'] == grasp
        assert pick['gantry_cloud'] is None
        cloud_path = pick_path + '&point_cloud=true'
        view = client.get(cloud_path).json()
        assert view['gantry_cloud']['scans'] == 3
        revision = view['gantry_cloud_revision']
        cached = client.get('/api/auto-pick/status', params={
            'arm': 'arm_02', 'point_cloud': 'true', 'cloud_revision': revision}).json()
        assert cached['gantry_cloud_revision'] == revision and cached['gantry_cloud'] is None
        object_cloud_path = f'{path}/{track_id}/point-cloud'
        object_cloud = client.get(object_cloud_path).json()
        assert object_cloud['gantry_cloud_revision'] == revision
        assert object_cloud['track_id'] == track_id
        assert object_cloud['gantry_cloud']['accumulated'] == [[1, 2, 3]]
        unchanged = client.get(object_cloud_path, params={'cloud_revision': revision}).json()
        assert unchanged['gantry_cloud_revision'] == revision and unchanged['gantry_cloud'] is None
        other = client.get(f'{path}/{2**60 + 104}/point-cloud').json()
        assert other['gantry_cloud_revision'] is None and other['gantry_cloud'] is None
        assert client.get(f'{path}/1/point-cloud').status_code == 404
        with observer.lock:
            observer.snapshot['clock_epoch'] = '3'
        expired_epoch = client.get(object_cloud_path).json()
        assert expired_epoch['gantry_cloud_revision'] is None and expired_epoch['gantry_cloud'] is None
        with observer.lock:
            observer.snapshot['clock_epoch'] = '2'
        wrist_path = '/api/auto-pick/wrist-preview?arm=arm_02'
        assert client.get(f'{wrist_path}&frame={frame_id}').content == b'paired-rgb'
        assert client.get(f'{wrist_path}&frame={frame_id - 1}').status_code == 410
        with observer.lock:
            observer.pick_status['arm_02']['task'].update(track_id='456')
        switched = client.get(pick_path + '&point_cloud=true').json()
        assert switched['gantry_cloud'] is None and switched['wrist_camera']['grasp'] is None
        with observer.lock:
            observer.pick_status['arm_02']['task'] = dict(identity, phase=0)
        idle = client.get(pick_path + '&point_cloud=true').json()
        assert idle['gantry_cloud'] is None and idle['wrist_camera']['grasp'] is None
        assert idle['outcome'] is None  # No diagnostic manifest fallback.
        with observer.lock:
            observer.pick_status['arm_02']['task']['last_result'] = dict(identity, outcome=1)
        assert client.get(pick_path).json()['outcome'] == 1
        with observer.lock:
            del observer.wrist_previews['arm_02'][frame_id]
        payload = client.get(path).json()
        history = payload['objects']
        assert payload['detected_count'] == 105
        assert payload['outcome_counts'] == dict(placed=1, failed=2, unverified=1)
        assert [int(row['display_id']) for row in history] == list(range(105, 5, -1))
        assert history[1]['track_id'] == track_id and history[1]['state'] == 'placed'
        assert history[1]['arm'] == 'arm_02'
        with observer.lock:
            # A repeated, evicted ID keeps its original sequence and adds no count.
            revisit = dict(display_confirmed=True, ros_track_id=str(2**60), label='Rabbit')
            observer._remember_objects(dict(capture_stamp_ns='106', objects=[revisit]))
            assert revisit['display_id'] == '1'
            assert client.app.state.observe.object_numbers[revisit['ros_track_id']] == 1
            # Late re-association merges two already shown fragments and closes the gap.
            alias = dict(display_confirmed=True, ros_track_id=track_id,
                         raw_ros_track_id=str(2**60+102), label='Rabbit')
            observer._remember_objects(dict(capture_stamp_ns='107', objects=[alias]))
            assert alias['display_id'] == '103'
            rename = dict(display_confirmed=True, ros_track_id=str(2**60+500),
                          raw_ros_track_id=str(2**60+104), label='Rabbit')
            observer._remember_objects(dict(capture_stamp_ns='108', objects=[rename]))
            assert rename['display_id'] == '104'
        payload = client.get(path).json()
        assert payload['detected_count'] == 104
        assert payload['outcome_counts'] == dict(placed=1, failed=1, unverified=1)
        assert [int(row['display_id']) for row in payload['objects']] == list(range(104, 5, -1))
        assert payload['objects'][1]['track_id'] == track_id and payload['objects'][1]['state'] == 'placed'
        with observer.lock:
            missed_id = str(2**60+501)
            observer._remember_objects(dict(capture_stamp_ns='109', objects=[
                dict(display_confirmed=True, ros_track_id=missed_id, label='Rabbit')]))
            crossing = dict(run_id=rid, epoch='2', track_id=missed_id)
            observer._record_missed_pick(dict(crossing, epoch='1'))
            observer._record_missed_pick(dict(crossing, run_id='old'))
            observer._record_missed_pick(dict(crossing, track_id=track_id))
            assert observer.object_history[missed_id]['state'] == 'tracked'
            observer._record_missed_pick(crossing)
            observer._record_missed_pick(crossing)
        result = client.get(path).json()
        assert result['outcome_counts'] == dict(placed=1, failed=2, unverified=1)
        assert result['objects'][0]['state'] == 'failed'
        assert result['objects'][0]['reason'] == 'Passed the last pick area without being picked'
        clear = post('/api/auto-pick/clear-bin?arm=arm_02', {'run_id':rid}, 'clear-not-ready')
        assert clear.status_code == 409 and clear.json()['error']['code'] == 'BIN_CLEAR_NOT_READY'
        assert post('/api/auto-pick/home?arm=invalid', {'run_id':rid}, 'bad-arm').status_code == 422
        receipt = post('/api/auto-pick/home?arm=arm_02', {'run_id':rid}, 'go-home')
        assert receipt.status_code == 202 and receipt.json()['command'] == 'pick_home'
        assert receipt.json()['accepted'] is None
        assert client.get('/api/auto-pick/status?arm=arm_02').json()['gripper']['possession'] == 'empty'
        preview = client.get('/api/auto-pick/wrist-preview?arm=arm_02')
        assert preview.status_code == 200 and preview.headers['content-type'] == 'image/jpeg'
        assert preview.content == b'wrist-preview'
        assert client.get('/api/auto-pick/status?arm=arm_02').json()['wrist_camera']['frame_id']
        assert client.get('/api/auto-pick/wrist-preview?arm=arm_01').status_code == 404
        assert client.get('/api/auto-pick/wrist-preview?arm=invalid').status_code == 422
        with observer.lock:
            observer.pick_commands.clear()
            observer.received = time.monotonic()
            observer.pick_seen['arm_02'] = time.monotonic()
        receipt = post('/api/auto-pick/continuous?arm=arm_02', {'run_id':rid}, 'continuous')
        assert receipt.status_code == 202 and receipt.json()['command'] == 'pick_continuous', receipt.text
        with observer.lock:
            observer.pick_commands.clear()
            observer.pick_status['arm_02']['task']['continuous_operation'] = True
        assert post('/api/auto-pick/start?arm=arm_02', {'run_id':rid}, 'busy').status_code == 409
        assert post(f'/api/observe/runs/{rid}/stop', {}, 'busy-stop').status_code == 409
        receipt = post('/api/auto-pick/pause?arm=arm_02', {'run_id':rid}, 'pause')
        assert receipt.status_code == 202 and receipt.json()['command'] == 'pick_pause', receipt.text
        with observer.lock:
            observer.pick_commands.clear()
            observer.pick_status['arm_02']['task']['continuous_operation'] = False
        assert post(f'/api/observe/runs/{rid}/stop',{},'stop').status_code in (200,202)
        status=wait_for(lambda:client.get('/api/observe/status').json(),lambda x:x['run']['state']=='stopped' and x['active_model_version'] is None)
        assert client.get(snapshot['rgb_url']).content==b'rgb10'  # Frozen, bounded last frame.
        assert post('/api/observe/runs',{'activation_id':aid},'old-ack').status_code==409
        checkpoint.write_bytes(b'corrupted')
        assert post('/api/observe/model-activations',{'model_version':'version1'},'bad-hash').status_code==503
        assert client.get('/api/observe/status').json()['active_model_version'] is None
        assert client.get('/observe').status_code==200
        assert client.get('/observe.js').status_code==200
        assert not client.cookies
    # Wrong worker ACK must never become Active.
    checkpoint.write_bytes(b'contract-test-checkpoint')
    worker.write_text(WORKER.replace("checkpoint_sha256=m['checkpoint_sha256']", "checkpoint_sha256='wrong'"))
    app=service.create_app(config,store,observe_factory=factory)
    with TestClient(app) as client:
        activation=client.post('/api/observe/model-activations',json={'model_version':'version1'},headers={'Idempotency-Key':'wrong-ack'}).json()
        ack=wait_for(lambda:client.get('/api/observe/model-activations/'+activation['activation_id']).json(),lambda x:x['state']=='failed')
        assert 'mismatch' in ack['message']
        assert client.get('/api/observe/status').json()['active_model_version'] is None
print('Observe API: checksum, exact ACK, idempotency, real subprocess stop, epoch/cache bounds, frozen frame, no login/motion: passed')


# A transient second mask may keep a gantry preview ID without earning a
# competing transit hint. Heartbeats must not count as new measurements.
source = PROJECT / 'ros2_ws/src/ptc_inference/ptc_inference/observe_stream.py'
spec = importlib.util.spec_from_file_location('observe_stream', source)
stream_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(stream_module)

wrist_target_near = stream_module.wrist_target_near
roi = {'bounds_min_m': [.45, -1.1, .75], 'bounds_max_m': [.775, -.8, 1.1]}
assert stream_module.measured_pick_area_exit((.6, -.82), (.6, -.78), roi)
assert not stream_module.measured_pick_area_exit((.6, -.9), (.6, -.8), roi)
assert not stream_module.measured_pick_area_exit((.6, -.78), (.6, -.82), roi)
assert not stream_module.measured_pick_area_exit((.6, -.82), (.8, -.78), roi)
hint = SimpleNamespace(key=SimpleNamespace(epoch=4),
    pose=SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=0)),
        pose=SimpleNamespace(position=SimpleNamespace(x=.6, y=-1.31))),
    velocity_mps=SimpleNamespace(x=0., y=.06))
assert not wrist_target_near(hint, 4, 11_000_000_000, 22_000_000_000, roi, 2.)
assert wrist_target_near(hint, 4, 12_000_000_000, 22_000_000_000, roi, 2.)
assert not wrist_target_near(hint, 5, 12_000_000_000, 22_000_000_000, roi, 2.)
assert not wrist_target_near(hint, 4, 12_000_000_000, 1_000_000_000, roi, 2.)
assert not wrist_target_near(hint, 4, 19_000_000_000, 22_000_000_000, roi, 2.)
def working(phase, route, gantry_epoch=7):
    return stream_module.wrist_inference_working(
        phase, 4, 12_000_000_000, gantry_epoch, route, 700_000_000, 22_000_000_000, roi, 2.)

assert working(1, (7, 11_800_000_000, [hint]))
assert not working(1, (7, 11_800_000_000, []))  # Another arm has the hint.
assert not working(1, (7, 11_000_000_000, [hint]))
assert not working(1, (7, 11_800_000_000, [hint]), gantry_epoch=8)
assert working(3, (None, 0, ())) and working(4, (None, 0, ()))
hint.pose.pose.position.x = .9
assert not wrist_target_near(hint, 4, 12_000_000_000, 22_000_000_000, roi, 2.)
print('Wrist inference: pre-entry lead, expired/old-epoch hints and out-of-region targets passed')

# Restarting with an occupied bin still uses the recorded empty force.
bin_load = stream_module.BinLoadVerifier()
for i in range(10):
    bin_load.sample(i / 10, i * 100_000_000, 9.81 + .55)
assert 55 < bin_load.details(1.0, 9.81)['contents_g'] < 57
assert bin_load.details(1.0, None)['contents_g'] is None
assert bin_load.details(2.0, 9.81)['contents_g'] is None
bin_load.release(1.0, ('1', '2', '3'))
for i in range(10, 27):
    bin_load.sample(i / 10, i * 100_000_000, 9.81 + .55 + .49)
bin_load.results(2.6)
assert 49 < bin_load.details(2.6, 9.81)['last_drop_g'] < 51

def check_bin_load_with_spike(after_load, sample_after=True):
    verifier = stream_module.BinLoadVerifier()
    for i in range(21):
        load = 27.784 if i < 16 else (27.17, 27.46, 27.52, 27.79, 27.25)[i - 16]
        verifier.sample(i / 10, i * 100_000_000, load)
    verifier.release(2.0, ('1', '2', '4'))
    assert verifier.results(2.0)[0]['state'] == 'placing'
    if sample_after:
        for i in range(21, 39):
            verifier.sample(i / 10, i * 100_000_000, after_load)
    return verifier

recovered = check_bin_load_with_spike(28.644)
assert recovered.results(3.8)[0]['state'] == 'placed'
# A following release cannot reuse the plateau from before the previous release.
recovered.release(3.81, ('1', '3', '5'))
assert recovered.results(3.81)[0]['state'] == 'placing'
for i in range(39, 52):
    recovered.sample(i / 10, i * 100_000_000, 28.644)
assert recovered.results(5.1) == []
assert recovered.results(7.9)[0]['state'] == 'unverified'
no_gain = check_bin_load_with_spike(27.784)
assert no_gain.results(3.8) == []
assert no_gain.results(6.1)[0]['state'] == 'unverified'
# An old plateau cannot stand in for a bin that has been disturbed for too long.
stale = stream_module.BinLoadVerifier()
for i in range(25):
    stale.sample(i / 10, i * 100_000_000, 10.0 if i < 10 else 10.0 + i % 3)
stale.release(2.4, ('1', '2', '6'))
assert stale.results(2.4)[0]['reason'] == 'No stable bin load before release'
# Overlapping releases must not both claim the same increase.
overlapping = check_bin_load_with_spike(28.644, sample_after=False)
overlapping.release(2.1, ('1', '3', '7'))
assert all(event['state'] == 'unverified' for event in overlapping.results(2.1))
print('Bin load: short baseline disturbance, no gain, stale baseline and overlap passed')

# An object that reaches the bin counts as placed even when it arrives before
# the gripper opens, or after the grip is lost and no release follows.
def carried_bin(key):
    verifier = stream_module.BinLoadVerifier()
    for i in range(10):
        verifier.sample(i / 10, i * 100_000_000, 20.0)
    verifier.carry(.9, key)
    return verifier

early = carried_bin(('1', '2', '8'))
for i in range(10, 25):
    early.sample(i / 10, i * 100_000_000, 20.54)   # fell in during bin approach
early.release(2.4, ('1', '2', '8'))
assert early.results(2.4)[0]['state'] == 'placing'
for i in range(25, 40):
    early.sample(i / 10, i * 100_000_000, 20.54)
placed_early = early.results(3.9)
assert placed_early[0]['state'] == 'placed' and abs(placed_early[0]['delta_n'] - .54) < 1e-9
lost = carried_bin(('1', '3', '9'))
for i in range(10, 25):
    lost.sample(i / 10, i * 100_000_000, 20.54)    # grip lost above the bin
lost.ended(2.4, ('1', '3', '9'))
assert lost.results(2.4) == []
for i in range(25, 40):
    lost.sample(i / 10, i * 100_000_000, 20.54)
assert lost.results(3.9)[0]['state'] == 'placed'
lost.ended(4.0, ('1', '3', '9'))
assert lost.results(4.0) == []
# A lost object that did not reach the bin keeps the cycle's own failure.
missed = carried_bin(('1', '4', '10'))
missed.ended(1.0, ('1', '4', '10'))
for i in range(10, 70):
    missed.sample(i / 10, i * 100_000_000, 20.0)
assert missed.results(3.0) == [] and missed.results(6.9) == [] and missed.pending is None
# A cycle that never carried its object is not checked against the bin.
idle = stream_module.BinLoadVerifier()
idle.ended(1.0, ('1', '5', '11'))
assert idle.pending is None and idle.results(1.0) == []
print('Bin load: early arrival and loss above the bin count as placed')

def release_trace(near_rim, peak, final):
    verifier = stream_module.BinLoadVerifier()
    events = []
    for i in range(10):
        verifier.sample(i / 10, i * 100_000_000, 20.)
    verifier.release(.9, ('1', '6', '11'), near_rim=near_rim)
    for i in range(10, 52):
        value = peak if 10 <= i < 17 else final
        verifier.sample(i / 10, i * 100_000_000, value)
        events.extend(verifier.results(i / 10))
    return verifier, events

full, overflow = release_trace(True, 20.54, 20.)
assert overflow[-1]['state'] == 'unverified' and overflow[-1]['overflow'] is True
_, bounce = release_trace(False, 20.54, 20.)
assert bounce[-1]['state'] == 'unverified' and not any(e.get('overflow') for e in bounce)
_, retained = release_trace(True, 20.54, 20.54)
assert retained[-1]['state'] == 'placed' and not any(e.get('overflow') for e in retained)
_, no_arrival = release_trace(True, 20., 20.)
assert not any(e.get('overflow') for e in no_arrival)
assert not full.empty_after(5.1, 9.81, 5_000_000_000)
for i in range(52, 62):
    full.sample(i / 10, i * 100_000_000, 9.81)
assert not full.empty_after(6.1, 9.81, 6_000_000_000)
assert full.empty_after(6.1, 9.81, 5_700_000_000)
assert not full.empty_after(7., 9.81, 5_700_000_000)
print('Bin load: near-rim rise/fall, non-overflow cases and post-clear tare passed')

# A feedback sample may reach the subscriber one simulation step before /clock.
feedback_issue = stream_module.feedback_timestamp_issue
assert feedback_issue(2_000_000_000, 2_033_333_334, 30_000_000, 33_333_333) is None
assert '66.7 ms ahead' in feedback_issue(2_000_000_000, 2_066_666_667, 30_000_000, 33_333_333)
assert '66.7 ms ahead' in feedback_issue(2_000_000_000, 2_066_666_667, 30_000_000, 70_000_000)
assert '1100.0 ms behind' in feedback_issue(2_000_000_000, 900_000_000, 30_000_000, 33_333_333)


tracking = yaml.safe_load((PROJECT / 'config/factory_demo/variants/camera_bracket_v2/arm_01.yaml').read_text())['tracking']
track = None
for i in range(10):
    track = stream_module.update_transit_track(track, dict(measurement_stamp_ns=str(i*100_000_000),
                                                          position_world_m=[.6, -2.2 + i*.006, .78]))
speed, residual, uncertainty = stream_module.measured_transit_speed(track, tracking)
assert abs(speed - .06) < 1e-9 and residual < 1e-9 and 0 < uncertainty < tracking['max_velocity_uncertainty_mps']
assert stream_module.measured_transit_speed(track.__class__(track.origin, track.deadline_ns, track.candidate,
                                                             track.confirmations, track.samples[:2]), tracking) is None

# The second arm extrapolates its hint for up to 50 s. A short gantry fit is
# held within the association distance over that horizon, and a first-arm wrist
# measurement of the same ID re-anchors position, stamp and belt speed.
downstream = stream_module.downstream_transit_hint
position, measured_ns, speed, hint_source = downstream([.6, -2.2, .78], 0, .063, None, 30_000_000_000, .075)
assert hint_source == 'gantry' and position == [.6, -2.2, .78] and measured_ns == 0
assert abs(speed - .0615) < 1e-12 and abs(speed - .06) * 50 <= .075 + 1e-12
position, measured_ns, speed, hint_source = downstream(
    [.6, -2.2, .78], 0, .063, (20_000_000_000, [.61, -.99, .8]), 30_000_000_000, .075)
assert hint_source == 'upstream_wrist' and position == [.61, -.99, .8] and measured_ns == 20_000_000_000
assert abs(speed - .0605) < 1e-12
# A wrist record outside the gate, older than the gantry record or newer than
# the published frame leaves the gantry hint in place.
for wrist in ((20_000_000_000, [.6, -.80, .8]), (0, [.6, -2.2, .8]), (31_000_000_000, [.6, -.34, .8])):
    assert downstream([.6, -2.2, .78], 0, .06, wrist, 30_000_000_000, .075)[1:] == (0, .06, 'gantry')
print('Downstream hint: bounded belt speed and first-arm wrist re-anchor passed')


# Use capture time rather than the number of inference frames.
window = 1_000_000_000
streak = None
for capture in range(0, window, 100_000_000):
    streak, confirmed = stream_module.confirm_pick(streak, capture, window)
    assert not confirmed
assert not stream_module.confirm_pick(streak, 900_000_000, window)[1]
assert stream_module.confirm_pick(streak, window, window)[1]
assert not stream_module.confirm_pick(streak, 1_200_000_000, window)[1]
assert not stream_module.confirm_pick(streak, 200_000_000, window)[1]
assert not stream_module.confirm_pick(None, window, window)[1]

# One disqualified or missing frame is bridged without resetting the start;
# two in a row, or a disqualified first frame, leave no streak.
confirm = stream_module.confirm_pick
streak, _ = confirm(None, 0, window)
for capture in (250_000_000, 500_000_000):
    streak, _ = confirm(streak, capture, window)
streak, confirmed = confirm(streak, 750_000_000, window, qualified=False)
assert streak == (0, 750_000_000, 1) and not confirmed
streak, confirmed = confirm(streak, window, window)
assert streak == (0, window, 0) and confirmed
missed, _ = confirm(streak, 1_250_000_000, window, qualified=False)
assert confirm(missed, 1_500_000_000, window, qualified=False) == (None, False)
assert confirm(None, 0, window, qualified=False) == (None, False)
assert confirm(streak, 1_300_000_000, window, qualified=False) == (None, False)
print('Gantry confirmation: single bridged frame passed')

# A valid frame anchors the 3D box. RGB alone can preserve an estimate
# briefly, only while its projection agrees with the actual silhouette.
import numpy as np

# A small region on another object may be far enough to distort both the
# silhouette bounds and the registered depth, even at high class confidence.
fragmented = np.zeros((160, 100), dtype=bool)
fragmented[105:135, 37:63] = True
fragmented[100:103, 43:47] = True  # Nearby part separated by a small occlusion.
fragmented[20:25, 47:53] = True    # Distant clutter.
instance = dict(mask=fragmented, bbox_xyxy=[37., 20., 63., 135.], confidence=.97)
cleaned = stream_module.remove_detached_mask_fragments(instance)
assert cleaned['bbox_xyxy'] == [37., 100., 63., 135.]
assert cleaned['mask'][105:135, 37:63].all() and cleaned['mask'][100:103, 43:47].all()
assert not cleaned['mask'][:100].any() and fragmented[20:25, 47:53].all()
assert cleaned['confidence'] == .97
assert stream_module.remove_detached_mask_fragments(cleaned) is cleaned
ambiguous = dict(instance, mask=fragmented.copy())
ambiguous['mask'][20:45, 40:60] = True
assert stream_module.remove_detached_mask_fragments(ambiguous) is ambiguous

mask = np.zeros((100, 100), dtype=bool)
mask[45:56, 45:56] = True
camera = np.eye(4), [100., 0., 50., 0., 100., 50., 0., 0., 1.]
bounds = np.array([[-.05, -.05, 1.], [.05, .05, 1.1]])
# Edge measurements can still reach tracking, but must not touch the fusion window.
fusion = stream_module.GantryCloudHistory(0.)
interior = np.zeros((100, 200), dtype=bool)
interior[5:95, 10:190] = True
assert not stream_module.mask_near_image_edge(interior)
for pixel in ((4, 100), (95, 100), (50, 9), (50, 190)):
    border = interior.copy()
    border[pixel] = True
    assert stream_module.mask_near_image_edge(border)
step = 50_000_000
for offset in (0, 1):
    stamp = window + offset * step
    fusion.begin_frame(1, stamp)
    fusion.add(1, 'rabbit', stamp, [[offset * .003, 0., 1.0]])
accepted = fusion.fused(1, 'rabbit', window + step, [.06, 0., 0.])
assert accepted is not None and accepted[2] == 2
assert np.allclose(accepted[1], [[.003, 0., 1.], [.003, 0., 1.]])
for offset in (2, 3):
    stamp = window + offset * step
    fusion.begin_frame(1, stamp)
    fusion.add(1, 'rabbit', stamp, [[5., 5., 5.]], near_edge=True)
    assert fusion.fused(1, 'rabbit', stamp, [1.2, 1.2, 0.]) is None
for offset in (4, 5):
    stamp = window + offset * step
    fusion.begin_frame(1, stamp)
    fusion.add(1, 'rabbit', stamp, [[offset * .003, 0., 1.]])
    recovered = fusion.fused(1, 'rabbit', stamp, [.06, 0., 0.])
    assert (recovered is not None) == (offset == 5)
assert np.allclose(recovered[1], [[.015, 0., 1.], [.015, 0., 1.]])
fusion.begin_frame(2, window + 6 * step)
assert not fusion.surfaces
print('Gantry cloud: four image edges, clean-only alignment and fusion restart passed')

edges, box = stream_module.display_box(bounds, [.06, 0., 0.], mask, camera, None, window)
assert len(edges) == 12 and box[0] == window

# Repeated full measurements can refine an already confirmed box without
# letting the first slightly narrower measurement define the entire track.
wider = bounds.copy()
wider[:, 0] *= 1.2
updated = box
for index in range(1, 7):
    capture = window + index * 50_000_000
    projected, updated = stream_module.display_box(wider, None, mask, camera, updated,
                                                   capture, keep_size=True)
    assert len(projected) == 12 and updated[0] == capture
assert len(updated[3]) == 5
assert np.allclose(np.median(updated[3], axis=0), wider[1] - wider[0])

# A truncated depth slice can occur while the RGB mask is still central.
partial = bounds.copy()
partial[1, 0] = .01
held, cached = stream_module.display_box(partial, [.06, 0., 0.], mask, camera, box,
                                        window + 100_000_000, keep_size=True)
assert len(held) == 12 and cached[0] == window
assert np.allclose(cached[3], box[3])
held, cached = stream_module.display_box(None, None, mask, camera, cached,
                                        window + 200_000_000, keep_size=True)
assert len(held) == 12 and cached[0] == window
assert stream_module.display_box(None, None, mask, camera, cached, window + 300_000_000,
                                 keep_size=True) == ([], None)

# The recorded failure mode: a visible stationary mask must not be
# followed by a stale projected box continuing along the belt.
_, fast_box = stream_module.display_box(bounds, [1.2, 0., 0.], mask, camera, None, window)
drifted, unchanged = stream_module.display_box(None, None, mask, camera, fast_box,
                                               window + 100_000_000, keep_size=True)
assert drifted == [] and unchanged[0] == window
assert stream_module.display_box(bounds * [1., 3., 1.], None, mask, camera, None, window) == ([], None)

# The same short hold works when only the in-frame part of the box is visible.
edge_mask = np.zeros_like(mask)
edge_mask[45:56, 5:16] = True
edge_bounds = bounds - [.40, 0., 0.]
_, edge_box = stream_module.display_box(edge_bounds, [-.30, 0., 0.], edge_mask, camera, None, window)
assert edge_box is not None
edge_mask[:] = False
edge_mask[45:56, 2:13] = True
held, cached = stream_module.display_box(None, None, edge_mask, camera, edge_box,
                                        window + 100_000_000, keep_size=True)
assert len(held) == 12 and cached[0] == window
assert stream_module.display_box(None, None, edge_mask, camera, cached, window + 300_000_000,
                                 keep_size=True) == ([], None)

reacquired, fresh = stream_module.display_box(bounds, None, mask, camera, cached,
                                               window + 500_000_000, keep_size=True)
assert len(reacquired) == 12 and fresh[0] == window + 500_000_000
axis = np.linspace(-.05, .05, 10)
cloud = [[x, y, 1 + .1*(x+y+.1)] for x in axis for y in axis] + [[9., 9., 9.]]
displayed = stream_module.display_bounds(cloud)
assert .05 <= displayed[1][0] < .06 and .05 <= displayed[1][1] < .06 and displayed[1][2] < 1.03
assert stream_module.display_bounds(cloud[:10]) is None
assert stream_module.display_bounds(cloud[:-1] + (np.asarray(cloud[:-1]) + [.20, 0, 0]).tolist()) is None
merged = mask.copy()
merged[45:56, 60:69] = True
assert stream_module.display_box(bounds, None, merged, camera, None, window) == ([], None)
assert stream_module.display_box(bounds, None, merged, camera, box, window + 50_000_000)[0] == []
undersized = bounds.copy()
undersized[:, :2] *= .3
assert stream_module.display_box(undersized, None, mask, camera, None, window) == ([], None)
assert stream_module.display_box(bounds * [2.5, .95, 1.], None, mask, camera, None, window) == ([], None)
print('Observe box: bounded depth age, mask alignment, edge clipping, and outlier suppression passed')


def transit_sample(stamp_ns, x, y):
    return {'measurement_stamp_ns': str(stamp_ns), 'position_world_m': (x, y, .78)}


update = stream_module.update_transit_track
first = update(None, transit_sample(1_000_000_000, .60, -2.2))
assert not first.confirmed and update(first, first.candidate) is first
second = update(first, transit_sample(1_080_000_000, .60, -2.1952))
assert not second.confirmed
assert update(second, transit_sample(1_160_000_000, .69, -2.1904)) is second
# One bridged gantry frame keeps counting; a longer gap restarts it.
assert update(second, transit_sample(1_400_000_000, .60, -2.176)).confirmed
assert not update(second, transit_sample(1_700_000_000, .60, -2.158)).confirmed
third = update(second, transit_sample(1_160_000_000, .60, -2.1904))
assert third.confirmed
assert update(third, transit_sample(1_900_000_000, .60, -2.146)).confirmed
print('Gantry transit: distinct, timely belt measurements required before handoff')


def route_sample(key, stamp_ns, x=.60):
    return dict(transit_sample(stamp_ns, x, -2.2 + .06 * (stamp_ns-1_000_000_000)/1e9),
                track_key=key, ros_track_id=key)


def confirmed_route():
    routes = {}
    aliases = {}
    for stamp_ns in (1_000_000_000, 1_080_000_000, 1_160_000_000):
        stream_module.update_transit_routes(routes, aliases, [route_sample('first', stamp_ns)], 3.5)
    return routes, aliases


routes, aliases = confirmed_route()
for stamp_ns in (1_500_000_000, 1_580_000_000, 1_660_000_000):
    stream_module.update_transit_routes(routes, aliases, [route_sample('reappeared', stamp_ns)], 3.5)
assert aliases == {'reappeared': 'first'} and set(routes) == {'first'}
assert routes['first'].candidate['ros_track_id'] == 'first'
assert routes['first'].candidate['measurement_stamp_ns'] == '1660000000'

routes, aliases = confirmed_route()
for stamp_ns in (1_500_000_000, 1_580_000_000, 1_660_000_000):
    stream_module.update_transit_routes(
        routes, aliases, [route_sample('second', stamp_ns), route_sample('third', stamp_ns, .62)], 3.5)
assert not aliases and set(routes) == {'first', 'second', 'third'}

routes, aliases = confirmed_route()
for stamp_ns in (1_500_000_000, 1_580_000_000, 1_660_000_000):
    stream_module.update_transit_routes(
        routes, aliases, [route_sample('first', stamp_ns), route_sample('second', stamp_ns, .62)], 3.5)
assert not aliases and set(routes) == {'first', 'second'}
print('Gantry transit: one-to-one continuation retains the released ROS identity')


# Actual gantry case: a changed visible surface moved the centre45mm while
# the same rabbit remained on a unique belt trajectory. Use the configured
# tracking association distance rather than an extra35mm transit restriction.
routes, aliases = confirmed_route()
for stamp_ns in (1_500_000_000, 1_580_000_000, 1_660_000_000):
    sample = route_sample('shifted-surface', stamp_ns, .615)
    sample['position_world_m'] = (sample['position_world_m'][0], sample['position_world_m'][1]-.043, .78)
    stream_module.update_transit_routes(routes, aliases, [sample], 3.5, .075)
assert aliases == {'shifted-surface': 'first'}
assert routes['first'].candidate['measurement_stamp_ns'] == '1660000000'

routes, aliases = confirmed_route()
for stamp_ns in (1_500_000_000, 1_580_000_000, 1_660_000_000):
    stream_module.update_transit_routes(routes, aliases, [route_sample('outside', stamp_ns, .69)], 3.5, .075)
assert not aliases
print('Gantry transit: configured association radius joins the shifted surface, retains source time and rejects distant tracks')
