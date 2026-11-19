"""Observe ROS topic resolution from the selected arm profiles; no ROS, GPU or motion."""
from copy import deepcopy
import importlib.util
from pathlib import Path
from types import SimpleNamespace

PROJECT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location(
    'observe_stream', PROJECT / 'ros2_ws/src/ptc_inference/ptc_inference/observe_stream.py')
stream = importlib.util.module_from_spec(spec)
spec.loader.exec_module(stream)

# Previously hard-coded names: default profiles must resolve to exactly these.
LEGACY = {
    'tracks': 'vision/tracks', 'frame': 'vision/frame', 'vision_status': 'vision/status',
    'inference_status': 'inference/status', 'detections': 'vision/semantic_detections',
    'task_status': 'task/status', 'task_command': 'task/command', 'gripper_status': 'gripper/status',
    'arm_status': 'arm/status', 'joint_states': 'joint_states', 'rgb': 'camera/color/image_raw',
    'bin_load': 'bin/load', 'bin_release': 'task/bin_release', 'bin_clear': 'bin/clear',
}
profiles = {arm: stream.load_station_profile(PROJECT, arm) for arm in ('arm_01', 'arm_02')}
for arm, profile in profiles.items():
    assert stream.selected_profile_path(PROJECT, arm).name == arm + '.yaml'
    topics = stream.station_topics(profile)
    assert topics['namespace'] == '/' + arm
    assert {key: value for key, value in topics.items() if key != 'namespace'} == {
        key: f'/{arm}/{name}' for key, name in LEGACY.items()}, topics

# Own publishers are relative and resolve to the names the arm launcher and vision expect.
assert stream.resolve_topic(stream.OBSERVE_NAMESPACE, stream.PICK_PREVIEW_TOPICS['arm_01']) == '/factory/observe/pick_preview_tracks'
assert stream.resolve_topic(stream.OBSERVE_NAMESPACE, stream.PICK_PREVIEW_TOPICS['arm_02']) == '/factory/observe/pick_preview_tracks_arm_02'
assert stream.resolve_topic(stream.OBSERVE_NAMESPACE, stream.GANTRY_CLOUD_TOPIC) == '/factory/observe/gantry_cloud'
assert all(not name.startswith('/') for name in (*stream.PICK_PREVIEW_TOPICS.values(), stream.GANTRY_CLOUD_TOPIC))

# A different station namespace and an endpoint remap flow through; suffix-only topics follow the namespace.
custom = deepcopy(profiles['arm_02'])
custom['runtime']['namespace'] = 'cell_b/right_arm'
custom['ros']['endpoints']['task_status'] = 'coordinator/status'
topics = stream.station_topics(custom)
assert topics['namespace'] == '/cell_b/right_arm'
assert topics['task_status'] == '/cell_b/right_arm/coordinator/status'
assert topics['bin_load'] == '/cell_b/right_arm/bin/load' and topics['bin_clear'] == '/cell_b/right_arm/bin/clear'

for broken, message in ((lambda p: p['ros']['endpoints'].pop('tracks'), 'tracks'),
                        (lambda p: p['ros']['endpoints'].__setitem__('frame', '/absolute/frame'), 'relative'),
                        (lambda p: p['runtime'].__setitem__('namespace', '/'), 'namespace')):
    candidate = deepcopy(profiles['arm_01'])
    broken(candidate)
    try:
        stream.station_topics(candidate)
    except ValueError as error:
        assert message in str(error), error
    else:
        raise AssertionError('accepted invalid profile')

# Readiness uses the publisher's resolved endpoint list and the station namespace.
vision = SimpleNamespace(node_namespace='/cell_b/right_arm', node_name='vision')
assert stream.vision_subscribed([vision], '/cell_b/right_arm')
assert not stream.vision_subscribed([vision], '/arm_02')
assert not stream.vision_subscribed([SimpleNamespace(node_namespace='/arm_02', node_name='task_coordinator')], '/arm_02')
assert not stream.vision_subscribed([], '/arm_01')
print('Observe topics: selected profiles, legacy defaults, custom namespace/endpoint, relative own outputs and readiness passed')
