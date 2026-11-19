"""Observe process ownership; no Torch, ROS or motion commands in the Web process."""
import base64
from collections import OrderedDict
import importlib.util
import json
from pathlib import Path
import subprocess
import threading
import time
import uuid

_spec = importlib.util.spec_from_file_location('teach_model_bundle', Path(__file__).with_name('model_bundle.py'))
model_bundle = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(model_bundle)


class ObserveFailure(RuntimeError):
    def __init__(self, status, code, message):
        super().__init__(message)
        self.status, self.code = status, code


def read_json(path):
    return json.loads(path.read_text(encoding='utf-8'))


def resource_path(root, kind, key):
    if not isinstance(key, str) or not key or len(key) > 128 or not all(c.isalnum() or c in '_-' for c in key):
        raise ObserveFailure(404, 'NOT_FOUND', 'Unknown resource')
    return root / kind / key


def list_published_models(root, target_id=None):
    return [row for p in sorted(root.glob('models/*/manifest.json'))
            if (row := read_json(p))['state'] == 'published'
            and (target_id is None or row['target_id'] == target_id)]


def resolve_published_model(root, version):
    directory = resource_path(root, 'models', version)
    try:
        row = read_json(directory / 'manifest.json')
    except FileNotFoundError as error:
        raise ObserveFailure(404, 'MODEL_NOT_FOUND', 'Model not found') from error
    if row['state'] != 'published':
        raise ObserveFailure(409, 'MODEL_NOT_PUBLISHED', 'Publish this model before using Observe')
    # Activation reads only the model directory; jobs and training images may be absent.
    try:
        checkpoint = model_bundle.verify(directory, row)
    except model_bundle.BundleError as error:
        raise ObserveFailure(error.status, error.code, str(error)) from error
    return dict(row, checkpoint_path=str(checkpoint))


class ObserveRuntime:
    history_limit = 100

    def __init__(self, project, root, config, python, worker_factory=None):
        self.project, self.root, self.config, self.python = project, root, config, python
        self.lock = threading.RLock()
        self.factory = worker_factory
        self.process = None
        self.activation = None
        self.activations = OrderedDict()
        self.runs = OrderedDict()
        self.run = None
        self.frames = OrderedDict()
        self.snapshot = None
        self.generation = 0
        self.received = None
        self.last_health = {}
        self.pick_status = {}
        self.bin_placements = {}
        self.wrist_previews = {}
        self.gantry_surfaces = OrderedDict()
        self.gantry_surface_epoch = None
        self.pick_seen = {}
        self.pick_commands = {}
        self.object_history = OrderedDict()
        self.object_numbers = {}
        self.object_results = {}

    def busy(self):
        with self.lock:
            return self.process is not None

    def request_activation(self, version):
        with self.lock:
            if self.process is not None:
                raise ObserveFailure(409, 'OBSERVE_BUSY', 'Stop Observe before selecting a model')
            # Resolving/hashing runs in FastAPI's thread pool, not its event loop.
            model = resolve_published_model(self.root, version)
            key = uuid.uuid4().hex
            row = dict(activation_id=key, model_version=version, state='loading',
                       acknowledged_checkpoint_sha256=None, message='Loading model', error=None)
            self.activation = row
            self.activations[key] = row
            while len(self.activations) > 32:
                self.activations.popitem(last=False)
            self.generation += 1
            generation = self.generation
            self.snapshot = None
            self.frames.clear()
            self.last_health = {}
            try:
                if self.factory:
                    self.process = self.factory()
                else:
                    log = self.project / 'logs/observe/worker.log'
                    log.parent.mkdir(parents=True, exist_ok=True)
                    with log.open('a') as stream:
                        self.process = subprocess.Popen(
                            [self.python, '-u', str(self.project / 'tools/teach_target/observe_entry.py')],
                            cwd=self.project, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                            stderr=stream, text=True, encoding='utf-8')
                process = self.process
                threading.Thread(target=self._read, args=(process, generation, model), daemon=True).start()
                self._send(dict(command='load_model', activation_id=key, model=model,
                                config=self.config, project=str(self.project)))
                threading.Thread(target=self._load_timeout, args=(generation, key), daemon=True).start()
            except (OSError, ValueError) as error:
                self._fail(str(error))
            return dict(row)

    def _send(self, value):
        self.process.stdin.write(json.dumps(value) + '\n')
        self.process.stdin.flush()

    def _load_timeout(self, generation, key):
        time.sleep(self.config['load_timeout_sec'])
        with self.lock:
            if generation == self.generation and self.activation['activation_id'] == key and self.activation['state'] == 'loading':
                self._fail('Model load timed out; inspect logs/observe/worker.log')
                self.process.terminate()

    def _fail(self, message):
        error = {'code': 'OBSERVE_WORKER_FAILED', 'message': message}
        for command in self.pick_commands.values():
            if command['pending']:
                command.update(pending=False, accepted=None, message='Observe disconnected; ROS command result unknown')
        if self.activation and self.activation['state'] == 'loading':
            self.activation.update(state='failed', message=message, error=error)
        if self.run and self.run['state'] != 'stopped':
            self.run.update(state='failed', message=message, error=error)

    def _read(self, process, generation, model):
        try:
            for line in process.stdout:
                event = json.loads(line)
                with self.lock:
                    if generation != self.generation:
                        continue
                    if event['event'] == 'loaded':
                        if (event.get('activation_id') != self.activation['activation_id']
                                or event.get('model_version') != model['model_version']
                                or event.get('checkpoint_sha256') != model['checkpoint_sha256']):
                            raise ValueError('Model acknowledgement identity mismatch')
                        self.activation.update(state='active', acknowledged_checkpoint_sha256=event['checkpoint_sha256'],
                                               message='Model loaded; quality requires live validation',
                                               inference_contract=event['inference_contract'])
                    elif event['event'] == 'error':
                        raise ValueError(event['message'])
                    elif event['event'] == 'started' and self.run and event['run_id'] == self.run['run_id']:
                        self.run.update(state='waiting_for_frame', message='Waiting for gantry camera')
                    elif event['event'] == 'wrist_preview':
                        if self.run and event['run_id'] == self.run['run_id']:
                            cache = self.wrist_previews.setdefault(event['arm'], OrderedDict())
                            cache[time.monotonic_ns()] = (
                                base64.b64decode(event['jpeg'], validate=True), event.get('grasp'))
                            while len(cache) > 4:
                                cache.popitem(last=False)
                    elif event['event'] == 'gantry_surface':
                        if self.run and event['run_id'] == self.run['run_id']:
                            if self.gantry_surface_epoch != event['epoch']:
                                self.gantry_surfaces.clear()
                                self.gantry_surface_epoch = event['epoch']
                            key = event['track_id']
                            self.gantry_surfaces[key] = {
                                field: event[field] for field in ('track_id', 'scans', 'latest', 'accumulated')}
                            self.gantry_surfaces[key].update(
                                revision=event.get('revision'), capture_stamp_ns=event.get('capture_stamp_ns'))
                            self.gantry_surfaces.move_to_end(key)
                            while len(self.gantry_surfaces) > self.history_limit:
                                self.gantry_surfaces.popitem(last=False)
                    elif event['event'] == 'pick_status':
                        arm = event.get('arm', 'arm_01')
                        self.pick_status[arm] = {key: event.get(key)
                                                 for key in ('available', 'task', 'gripper', 'arm_feedback', 'joint_feedback', 'bin_load')}
                        self.pick_seen[arm] = time.monotonic()
                        self._update_object_outcome(arm, event['task'], event.get('gripper'))
                    elif event['event'] == 'pick_area_exit':
                        self._record_missed_pick(event)
                    elif event['event'] == 'bin_placement':
                        self._accept_bin_placement(event['arm'], event['placement'])
                    elif event['event'] == 'pick_reply':
                        command = self.pick_commands.get(event.get('arm', 'arm_01'))
                        if command and event['request_id'] == command['request_id']:
                            command.update(pending=False, accepted=event['accepted'], message=event['message'])
                    elif event['event'] == 'health':
                        self.last_health.update(event['health'])
                    elif event['event'] == 'frame' and self.run and event['run_id'] == self.run['run_id'] and self.run['state'] not in ('stopping', 'stopped', 'failed'):
                        self._accept_frame(event)
            process.wait(timeout=5)
            with self.lock:
                if generation == self.generation:
                    if self.run and self.run['state'] == 'stopping' and process.returncode == 0:
                        self.run.update(state='stopped', message='Stopped · last frame frozen')
                    elif not self.run or self.run['state'] != 'stopped':
                        self._fail('Observe worker exited; inspect logs/observe/worker.log')
        except (OSError, ValueError, KeyError, subprocess.TimeoutExpired) as error:
            with self.lock:
                if generation == self.generation:
                    self._fail(str(error))
            if process.poll() is None:
                process.terminate()
                try: process.wait(timeout=5)
                except subprocess.TimeoutExpired: process.kill(); process.wait()
        finally:
            process.stdin.close()
            process.stdout.close()
            with self.lock:
                if generation == self.generation:
                    self.process = None

    def _accept_frame(self, event):
        snapshot = event['snapshot']
        if snapshot['model_version'] != self.activation['model_version']:
            raise ValueError('Frame model version mismatch')
        key = snapshot['frame_key']
        if self.snapshot and snapshot['clock_epoch'] != self.snapshot['clock_epoch']:
            self.frames.clear()
            self.object_history.clear()
            self.object_numbers.clear()
            self.object_results.clear()
        blobs = {layer: base64.b64decode(event[layer], validate=True) for layer in ('rgb', 'mask')}
        self.frames[key] = blobs
        while len(self.frames) > self.config['frame_cache_size']:
            self.frames.popitem(last=False)
        snapshot.update(rgb_url=f'/api/observe/frames/{key}/preview?layer=rgb',
                        mask_url=f'/api/observe/frames/{key}/preview?layer=mask')
        self.snapshot = snapshot
        self._remember_objects(snapshot)
        self.received = time.monotonic()
        self.last_health = event.get('health', {})
        self.run.update(state='observing', message=snapshot.get('message', ''), error=None)

    def start_run(self, activation_id):
        with self.lock:
            if self.run and self.run['state'] not in ('stopped', 'failed'):
                raise ObserveFailure(409, 'RUN_EXISTS', 'An Observe run already exists')
            if not self.process or self.process.poll() is not None or not self.activation or self.activation['activation_id'] != activation_id or self.activation['state'] != 'active':
                raise ObserveFailure(409, 'MODEL_NOT_ACTIVE', 'Wait for a live matching model acknowledgement')
            row = dict(run_id=uuid.uuid4().hex, state='starting', activation_id=activation_id,
                       selected_model_version=self.activation['model_version'], message='Starting gantry observation', error=None)
            self.run = row
            self.pick_status, self.pick_seen, self.pick_commands = {}, {}, {}
            self.bin_placements.clear()
            self.wrist_previews.clear()
            self.gantry_surfaces.clear()
            self.gantry_surface_epoch = None
            self.object_history.clear()
            self.object_numbers.clear()
            self.object_results.clear()
            self.runs[row['run_id']] = row
            while len(self.runs) > 32: self.runs.popitem(last=False)
            self.snapshot = None
            self.received = None
            self.frames.clear()
            try: self._send(dict(command='start_run', run_id=row['run_id'], activation_id=activation_id))
            except OSError as error: self._fail(str(error))
            return dict(row)

    def _remember_objects(self, snapshot):
        confirmed = [obj for obj in snapshot.get('objects', [])
                     if obj.get('display_confirmed') and obj.get('ros_track_id')]
        for obj in confirmed:
            track_id = obj['ros_track_id']
            raw_id = obj.get('raw_ros_track_id')
            if raw_id == track_id or raw_id not in self.object_numbers:
                continue
            former_number = self.object_numbers.pop(raw_id)
            canonical_number = self.object_numbers.get(track_id, former_number)
            self.object_numbers[track_id] = min(former_number, canonical_number)
            if former_number != canonical_number:
                # Two previously counted fragments now represent one object.
                removed = max(former_number, canonical_number)
                self.object_numbers = {key: number - (number > removed)
                                       for key, number in self.object_numbers.items()}
                for key, row in self.object_history.items():
                    if key != raw_id:
                        row['display_id'] = str(self.object_numbers[key])
            former_result = self.object_results.pop(raw_id, None)
            current_result = self.object_results.get(track_id)
            if former_result and (former_result == 'placed' or current_result is None):
                self.object_results[track_id] = former_result
            former = self.object_history.pop(raw_id, None)
            current = self.object_history.get(track_id)
            if former is not None and (
                    current is None or former['state'] == 'placed'
                    or (current['state'] == 'tracked' and former_result is not None and current_result is None)):
                self.object_history[track_id] = dict(former, track_id=track_id,
                                                     display_id=str(self.object_numbers[track_id]))
            elif current is not None and former_result == 'placed':
                current['state'] = 'placed'

        for obj in confirmed:
            track_id = obj['ros_track_id']
            number = self.object_numbers.setdefault(track_id, len(self.object_numbers) + 1)
            obj['display_id'] = str(number)
            row = self.object_history.get(track_id)
            if row is None:
                row = dict(state=self.object_results.get(track_id, 'tracked'),
                           arm=None, cycle_id=None, epoch=None,
                           first_seen_ns=snapshot['capture_stamp_ns'])
                self.object_history[track_id] = row
            row.update(track_id=track_id, display_id=str(number), label=obj['label'])
        cutoff = len(self.object_numbers) - self.history_limit
        if len(self.object_history) > self.history_limit or any(
                self.object_numbers[key] <= cutoff for key in self.object_history):
            self.object_history = OrderedDict((key, row) for key, row in self.object_history.items()
                                              if self.object_numbers[key] > cutoff)

    def history(self, run_id):
        with self.lock:
            if run_id not in self.runs:
                raise ObserveFailure(404, 'RUN_NOT_FOUND', 'Unknown Observe run')
            if not self.run or self.run['run_id'] != run_id:
                raise ObserveFailure(410, 'RUN_EXPIRED', 'Run object history is no longer cached')
            ordered = sorted(self.object_history.values(), key=lambda row: int(row['display_id']), reverse=True)
            counts = dict(placed=0, failed=0, unverified=0)
            for state in self.object_results.values():
                counts[state] += 1
            return dict(run_id=run_id, detected_count=len(self.object_numbers),
                        outcome_counts=counts, objects=[dict(row) for row in ordered])

    def object_point_cloud(self, run_id, track_id, cloud_revision=None):
        with self.lock:
            if run_id not in self.runs:
                raise ObserveFailure(404, 'RUN_NOT_FOUND', 'Unknown Observe run')
            if not self.run or self.run['run_id'] != run_id:
                raise ObserveFailure(410, 'RUN_EXPIRED', 'Run point clouds are no longer cached')
            if track_id not in self.object_history:
                raise ObserveFailure(404, 'OBJECT_NOT_FOUND', 'Target is outside the recent object history')
            revision, cloud = self._gantry_cloud_preview(track_id, cloud_revision)
            return dict(run_id=run_id, track_id=track_id, gantry_cloud_revision=revision, gantry_cloud=cloud)

    def _gantry_cloud_preview(self, track_id, cloud_revision):
        if not self.snapshot or self.snapshot['clock_epoch'] != self.gantry_surface_epoch:
            return None, None
        cloud = self.gantry_surfaces.get(track_id)
        if not cloud or cloud.get('revision') is None:
            return None, None
        revision = f"{self.run['run_id']}:{self.gantry_surface_epoch}:{track_id}:{cloud['revision']}"
        return revision, None if revision == cloud_revision else cloud

    @staticmethod
    def _cycle_outcome(task):
        # Live TaskStatus.last_result only; no diagnostic files. Outcome 1 is the
        # controller's release report, not bin evidence, so it maps to unverified.
        result = (task or {}).get('last_result') or {}
        if all(result.get(key) == task[key] for key in ('epoch', 'cycle_id', 'track_id')):
            return result.get('outcome')
        return None

    def _record_missed_pick(self, event):
        if (not self.run or self.run['state'] != 'observing'
                or event['run_id'] != self.run['run_id']
                or not self.snapshot or event['epoch'] != self.snapshot['clock_epoch']):
            return
        row = self.object_history.get(event['track_id'])
        if (row and row['state'] == 'tracked' and row['arm'] is None
                and row['track_id'] not in self.object_results):
            self._set_object_state(row, 'failed')
            row['reason'] = 'Passed the last pick area without being picked'

    def _set_object_state(self, row, state):
        track_id = row['track_id']
        row.pop('reason', None)
        if self.object_results.get(track_id) == 'placed':
            row['state'] = 'placed'
            return
        row['state'] = state
        # Keep one settled outcome per ID after the visible row expires.
        if state in ('placed', 'failed', 'unverified'):
            self.object_results[track_id] = state

    def _accept_bin_placement(self, arm, placement):
        self.bin_placements[arm] = placement
        row = self.object_history.get(placement['track_id'])
        if row and (row['arm'], row['epoch'], row['cycle_id']) == (
            arm, placement['epoch'], placement['cycle_id']
        ):
            self._set_object_state(row, placement['state'])

    def _update_object_outcome(self, arm, task, gripper):
        if not task:
            return
        result = task.get('last_result')
        if result:
            finished = self.object_history.get(result['track_id'])
            if finished and finished['state'] != 'placed' and (
                finished['arm'], finished['epoch'], finished['cycle_id']
            ) == (arm, result['epoch'], result['cycle_id']):
                placement = self.bin_placements.get(arm, {})
                same_release = all(placement.get(key) == result[key]
                                   for key in ('epoch', 'cycle_id', 'track_id'))
                state = placement['state'] if same_release else {
                    1: 'unverified', 3: 'failed', 4: 'canceled', 5: 'failed'
                }.get(result['outcome'])
                if state:
                    self._set_object_state(finished, state)
        row = self.object_history.get(task['track_id'])
        if row is None or row['state'] == 'placed':
            return
        placement = self.bin_placements.get(arm, {})
        if all(placement.get(key) == task[key] for key in ('epoch', 'cycle_id', 'track_id')):
            if (row['arm'], row['epoch'], row['cycle_id']) == (arm, task['epoch'], task['cycle_id']):
                self._set_object_state(row, placement['state'])
                return
            return
        phase = task['phase']
        same_cycle = (row['arm'], row['epoch'], row['cycle_id']) == (arm, task['epoch'], task['cycle_id'])
        if phase == 0:
            if not same_cycle or row['state'] in ('failed', 'canceled'):
                return
            outcome = self._cycle_outcome(task)
            placement = self.bin_placements.get(arm, {})
            same_release = all(placement.get(key) == task[key]
                               for key in ('epoch', 'cycle_id', 'track_id'))
            state = placement['state'] if same_release else {
                1: 'unverified', 3: 'failed', 4: 'canceled', 5: 'failed'
            }.get(outcome)
        elif phase == 9:
            state = 'failed' if same_cycle else None
        elif phase >= 3:
            state = 'holding' if (gripper or {}).get('possession') == 'holding' else (
                'placing' if phase == 6 else 'picking')
        else:
            state = None
        if state:
            row.update(arm=arm, epoch=task['epoch'], cycle_id=task['cycle_id'])
            self._set_object_state(row, state)

    def auto_pick(self, arm="arm_01", point_cloud=False, cloud_revision=None):
        with self.lock:
            active = self.process is not None and time.monotonic() - self.pick_seen.get(arm, 0.0) < 2
            status = self.pick_status.get(arm, {})
            task = status.get('task') if active and status.get('available') else None
            result = self._cycle_outcome(task) if task and task['phase'] == 0 else None
            command = self.pick_commands.get(arm)
            previews = self.wrist_previews.get(arm)
            wrist_camera = None
            if previews:
                frame_id = next(reversed(previews))
                _, grasp = previews[frame_id]
                age = (time.monotonic_ns() - frame_id) / 1e9
                live = age <= self.config['stale_after_sec']
                show_grasp = (live and age <= .5 and task and task.get('planned_grasp_active') and grasp
                              and all(grasp[k] == task[k] for k in ('epoch', 'cycle_id', 'track_id')))
                wrist_camera = dict(frame_id=str(frame_id), live=live, grasp=grasp if show_grasp else None)
            revision, cloud = None, None
            if (point_cloud and task and task['phase'] in (3, 4, 5, 6)
                    and self.run and self.run['state'] == 'observing'):
                revision, cloud = self._gantry_cloud_preview(task['track_id'], cloud_revision)
            return dict(available=task is not None, task=task, wrist_camera=wrist_camera,
                        gantry_cloud=cloud, gantry_cloud_revision=revision,
                        command=dict(command, pending=self.pick_command_pending(arm, task)) if command else None,
                        gripper=status.get('gripper') if active else None,
                        arm_feedback=status.get('arm_feedback') if active else None,
                        joint_feedback=status.get('joint_feedback') if active else None,
                        bin_load=status.get('bin_load') if active else None,
                        placement=self.bin_placements.get(arm) if active else None,
                        outcome=result)

    def wrist_preview(self, arm, frame=None):
        with self.lock:
            previews = self.wrist_previews.get(arm)
            if not previews:
                raise ObserveFailure(404, 'WRIST_PREVIEW_UNAVAILABLE', 'Waiting for wrist camera')
            frame_id = next(reversed(previews)) if frame is None else frame
            if frame_id not in previews:
                raise ObserveFailure(410, 'WRIST_PREVIEW_EXPIRED', 'Wrist frame expired')
            return previews[frame_id][0]

    def command_pick(self, run_id, command, arm='arm_01'):
        with self.lock:
            starting = command in ('pick_start', 'pick_continuous')
            states = ('observing',) if starting else ('observing', 'waiting_for_frame')
            if not self.run or self.run['run_id'] != run_id or self.run['state'] not in states:
                raise ObserveFailure(409, 'OBSERVE_NOT_RUNNING', 'Start Observe first')
            pick = self.auto_pick(arm)
            # Return Home is a reset: it is sent whatever the arm's reported
            # state or an earlier command's reply. The arm decides how to return.
            homing = command == 'pick_home'
            bin_load = pick['bin_load'] or {}
            if command == 'pick_clear_bin' and not bin_load.get('clear_available'):
                raise ObserveFailure(409, 'BIN_CLEAR_NOT_READY', 'Clear Bin requires a stopped, empty, idle arm')
            if bin_load.get('clearing'):
                raise ObserveFailure(409, 'BIN_CLEAR_PENDING', 'Wait for empty-bin confirmation')
            if starting and bin_load.get('overflow'):
                raise ObserveFailure(409, 'BIN_OVERFLOW', 'Clear the overflow before starting')
            if not homing and not pick['available']:
                raise ObserveFailure(409, 'PICK_UNAVAILABLE', f'Start the {arm} semantic runtime first')
            if not homing and self.pick_command_pending(arm, pick['task']):
                raise ObserveFailure(409, 'PICK_COMMAND_PENDING', 'Wait for the current ROS command response')
            if starting and pick['task'].get('home_state') in (1, 2, 3):
                raise ObserveFailure(409, 'PICK_HOME_PENDING', 'Wait for Return Home')
            if starting and (self.status()['source_state'] != 'live' or pick['task']['phase'] != 0
                             or pick['task'].get('continuous_operation')):
                raise ObserveFailure(409, 'PICK_NOT_READY', 'A live camera and an idle arm are required')
            request_id = uuid.uuid4().hex
            try:
                self._send(dict(command=command, run_id=run_id, request_id=request_id, arm=arm))
            except OSError as error:
                raise ObserveFailure(503, 'PICK_WORKER_UNAVAILABLE', str(error)) from error
            self.pick_commands[arm] = dict(request_id=request_id, command=command, pending=True,
                                     baseline_cycle_id=(pick['task'] or {}).get('cycle_id'),
                                     accepted=None, message='Waiting for ROS response')
            return dict(self.pick_commands[arm])

    def pick_command_pending(self, arm, task):
        command = self.pick_commands.get(arm)
        return bool(command and (command['pending'] or (
            command['command'] in ('pick_start', 'pick_continuous') and command['accepted'] is True and task
            and task['cycle_id'] == command['baseline_cycle_id']
        )))

    def stop_run(self, run_id):
        with self.lock:
            row = self.runs.get(run_id)
            if row is None:
                raise ObserveFailure(404, 'RUN_NOT_FOUND', 'Unknown Observe run')
            if row is not self.run or row['state'] == 'stopped': return 200, dict(row)
            if not self.process:
                row.update(state='stopped', message='Stopped · last frame frozen')
                return 200, dict(row)
            for arm in ('arm_01', 'arm_02'):
                pick = self.auto_pick(arm)
                if (self.pick_command_pending(arm, pick['task']) or
                        (pick['task'] and (pick['task']['phase'] != 0 or pick['task'].get('continuous_operation')
                                           or pick['task'].get('home_state') in (1, 2, 3)))):
                    raise ObserveFailure(409, 'PICK_BUSY', f'Pause or cancel Auto Pick and wait for {arm} to become idle')
            row.update(state='stopping', message='Waiting for Observe worker exit')
            self._send(dict(command='stop_run', run_id=run_id))
            process = self.process
            def timeout():
                try: process.wait(timeout=self.config['stop_timeout_sec'])
                except subprocess.TimeoutExpired:
                    with self.lock:
                        if self.process is process:
                            self._fail('Worker did not stop within the deadline; terminating owned worker')
                    process.terminate()
            threading.Thread(target=timeout, daemon=True).start()
            return 202, dict(row)

    def status(self):
        with self.lock:
            resident = bool(self.process and self.process.poll() is None and self.activation and self.activation['state'] == 'active')
            age = time.monotonic() - self.received if self.received else None
            source = 'waiting'
            if self.snapshot:
                source = 'frozen' if self.run and self.run['state'] in ('stopping','stopped','failed') else ('stale' if age is not None and age > self.config['stale_after_sec'] else 'live')
            return dict(run=dict(self.run) if self.run else None,
                        active_model_version=self.activation['model_version'] if resident else None,
                        activation_id=self.activation['activation_id'] if self.activation else None,
                        last_frame_key=self.snapshot['frame_key'] if self.snapshot else None,
                        display_age_sec=age, source_state=source,
                        health=dict(self.last_health), confidence_threshold=self.config['confidence_threshold'],
                        preview_hz=self.config.get('preview_hz',self.config['inference_hz']),
                        inference_hz_limit=self.config['inference_hz'],motion_control=False)

    def close(self):
        with self.lock:
            self.generation += 1
            process = self.process
            self.process = None
        if process and process.poll() is None:
            process.terminate()
            try: process.wait(timeout=5)
            except subprocess.TimeoutExpired: process.kill(); process.wait()
