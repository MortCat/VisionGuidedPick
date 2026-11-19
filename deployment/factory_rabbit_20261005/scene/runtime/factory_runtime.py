"""Lifecycle owner for the scene's automatic feeder; no robot commands."""
import asyncio
import pathlib
import sys

import carb
import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Sdf, Usd

_manager = None


def clear_spawned_objects(stage):
    """Clear only prims authored by the factory feeders in this session."""
    session = stage.GetSessionLayer()
    group = session.GetPrimAtPath('/World/FactoryDemo/Objects')
    if group is None:
        return 0
    prefixes = {'MorandiBlock': 'MorandiBlock_', 'Rabbit': 'RabbitFeed_', 'TeddyBear': 'BearFeed_'}
    owned = []
    for prim in group.nameChildren:
        kind = prim.attributes.get('object:type')
        prefix = prefixes.get(kind.default) if kind is not None else None
        if prefix and prim.name.startswith(prefix) and prim.name[len(prefix):].isdigit():
            owned.append(prim.path)
    if owned:
        edit = Sdf.BatchNamespaceEdit()
        for path in owned:
            edit.Add(path, Sdf.Path.emptyPath)
        if not session.Apply(edit):
            raise RuntimeError('Could not clear generated feed objects from the session')
    return len(owned)


class FactoryRuntime:
    def __init__(self, stage, runtime_dir):
        self.stage = stage
        self.stage_id = omni.usd.get_context().get_stage_id()
        self.directory = pathlib.Path(runtime_dir)
        self.namespace = {}
        self.alive = True
        self.status = 'initializing'
        self.error = None
        self.initializations = 0
        self.rgb_resources = {}
        self.clutter = None
        self.bin_load = None
        self.stopped_wrist_mounts = None
        settings = carb.settings.get_settings()
        self.previous_min_frame_rate = settings.get('/persistent/simulation/minFrameRate')
        self.minimum_frame_rate = stage.GetPrimAtPath('/World/FactoryDemo').GetAttribute('runtime:minimumFrameRate').Get()
        self.previous_target_framerate = omni.timeline.get_timeline_interface().get_target_framerate()
        if self.minimum_frame_rate:
            settings.set('/persistent/simulation/minFrameRate', int(self.minimum_frame_rate))
            omni.timeline.get_timeline_interface().set_target_framerate(stage.GetTimeCodesPerSecond())
        self.task = asyncio.ensure_future(self.initialize())
        self.stage_subscription = omni.usd.get_context().get_stage_event_stream().create_subscription_to_pop(
            self.on_stage_event, name='FactoryAutoplayLifecycle')

    def clear_bin(self, arm):
        """Remove this station's generated bin contents from the session layer."""
        from pxr import UsdGeom

        if not self.alive or self.status != 'ready':
            raise RuntimeError('Factory runtime is not ready')
        if arm not in ('arm_01', 'arm_02'):
            raise ValueError('Unknown station')
        bin_path = f'/World/FactoryDemo/Stations/{arm}/Bin'
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])

        def bounds(part):
            prim = self.stage.GetPrimAtPath(f'{bin_path}/{part}/Geometry')
            if not prim:
                raise RuntimeError(f'Missing bin geometry: {part}')
            box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            if box.IsEmpty():
                raise RuntimeError(f'No bin geometry bounds: {part}')
            return box.GetMin(), box.GetMax()

        floor = bounds('Floor')[1][2]
        _, x_low_wall = bounds('Wall_0')
        x_high_wall, _ = bounds('Wall_1')
        _, y_low_wall = bounds('Wall_2')
        y_high_wall, _ = bounds('Wall_3')
        rim = x_low_wall[2]
        low = (x_low_wall[0], y_low_wall[1], floor - .02)
        high = (x_high_wall[0], y_high_wall[1], rim + .25)
        session = self.stage.GetSessionLayer()
        feeders = [self.feeder]
        if self.clutter:
            feeders.append(self.clutter)
        owned = []
        for feeder in feeders:
            for path in tuple(feeder.active):
                if not session.GetPrimAtPath(path):
                    continue
                pose = self.feeder.pose(path)
                position = pose['position'] if pose else ()
                if len(position) == 3 and all(lo < value < hi for lo, value, hi in zip(low, position, high)):
                    owned.append((feeder, path))
        if owned:
            edit = Sdf.BatchNamespaceEdit()
            for _, path in owned:
                edit.Add(Sdf.Path(path), Sdf.Path.emptyPath)
            if not session.Apply(edit):
                raise RuntimeError('Could not remove the bin contents from the session')
            for feeder, path in owned:
                feeder.active.pop(path, None)
                if feeder is self.clutter:
                    feeder.footprints.pop(path, None)
        return len(owned)

    def on_stage_event(self, event):
        if event.type in (int(omni.usd.StageEventType.CLOSING), int(omni.usd.StageEventType.CLOSED)):
            self.shutdown()

    def publish_status(self):
        if not self.alive or not self.stage.GetPrimAtPath('/World/FactoryDemo'):
            return
        with Usd.EditContext(self.stage, self.stage.GetSessionLayer()):
            prim = self.stage.GetPrimAtPath('/World/FactoryDemo')
            prim.CreateAttribute('runtime:status', Sdf.ValueTypeNames.String).Set(self.status)
            prim.CreateAttribute('runtime:error', Sdf.ValueTypeNames.String).Set(self.error or '')

    async def initialize(self):
        try:
            # Defer USD edits until outside the Action Graph evaluation stack.
            await omni.kit.app.get_app().next_update_async()
            if not self.alive:
                return
            if not omni.kit.app.get_app().get_extension_manager().is_extension_enabled('isaacsim.ros2.bridge'):
                raise RuntimeError('ros_bridge_unavailable: restart Isaac using deployment/windows/start_isaac.ps1 or deployment/linux/start_isaac.sh; do not change RMW during playback')
            import omni.graph.core as og
            if og.get_node_type('isaacsim.ros2.bridge.ROS2PublishImage') is None:
                raise RuntimeError('ros_bridge_node_unavailable: ROS2PublishImage is not registered')
            factory = self.stage.GetPrimAtPath('/World/FactoryDemo')
            value = factory.GetAttribute('feeder:assetDirectory').Get()
            asset_dir = pathlib.Path(Sdf.ComputeAssetPathRelativeToLayer(
                self.stage.GetRootLayer(), value.path))
            assert (asset_dir / 'feed_config.json').is_file(), f'Missing feeder assets: {asset_dir}'
            removed = clear_spawned_objects(self.stage)
            if removed:
                carb.log_info(f'Cleared {removed} stale factory feed objects')
            reports = self.directory / 'reports'
            self.namespace = {'__name__': 'factory_autoplay_instance',
                              '_factory_graph_managed': True,
                              '_factory_asset_dir': str(asset_dir),
                              '_factory_monitor_dir': str(reports)}
            for name in ('feed_v3.py', 'monitor.py'):
                source = self.directory / name
                exec(compile(source.read_text(encoding='utf-8'), str(source), 'exec'), self.namespace)
            if factory.GetAttribute('clutter:enabled').Get():
                clutter_namespace = {}
                source = self.directory / 'clutter_runtime.py'
                exec(compile(source.read_text(encoding='utf-8'), str(source), 'exec'), clutter_namespace)
                asset = factory.GetAttribute('clutter:asset').Get()
                asset_path = Sdf.ComputeAssetPathRelativeToLayer(self.stage.GetRootLayer(), asset.path)
                self.clutter = clutter_namespace['ClutterFeeder'](self.stage, self.feeder, asset_path)
                self.feeder.block_spawn_clearance = self.clutter.clear_of_blocks
            source = self.directory / 'stopped_wrist_mounts.py'
            mounts_namespace = {'__name__': 'factory_stopped_wrist_mounts'}
            exec(compile(source.read_bytes(), str(source), 'exec'), mounts_namespace)
            self.stopped_wrist_mounts = mounts_namespace['StoppedWristMounts'](self.stage)
            source = self.directory / 'rgb_runtime.py'
            camera_namespace = {'__name__': 'factory_rgb_runtime'}
            exec(compile(source.read_text(encoding='utf-8'), str(source), 'exec'), camera_namespace)
            self.rgb_detach = camera_namespace['detach']
            await camera_namespace['attach'](self.stage, self.rgb_resources)
            if self.stage.GetPrimAtPath('/World/FactoryDemo/Stations/arm_01/BinLoadCell'):
                try:
                    source = self.directory / 'bin_load_runtime.py'
                    sensor_namespace = {'__name__': 'factory_bin_load_runtime'}
                    exec(compile(source.read_text(encoding='utf-8'), str(source), 'exec'), sensor_namespace)
                    self.bin_load = sensor_namespace['BinLoadPublisher'](self.stage_id, self.clear_bin)
                except Exception as exc:
                    carb.log_warn(f'Bin load unavailable: {exc}')
            hooks = sys.modules.get('factory_autoplay_hooks')
            target_feed = getattr(hooks, 'target_feed', None)
            if target_feed is not None:
                target_feed.feeder = self.feeder
                target_feed.previous.clear()
                target_feed.run_id += 1
            self.initializations += 1
            self.status = 'ready'
            self.publish_status()
            carb.log_info('Factory autoplay ready: press Play/resume for rabbit and configured clutter feeding.')
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            self.error = repr(exc)
            self.status = 'failed'
            self.publish_status()
            carb.log_error('Factory autoplay failed: ' + self.error)
            omni.timeline.get_timeline_interface().pause()

    @property
    def feeder(self):
        return self.namespace.get('rabbit_feeder')

    def shutdown(self):
        if not self.alive:
            return
        self.alive = False
        if not self.task.done():
            self.task.cancel()
        if self.stopped_wrist_mounts is not None:
            self.stopped_wrist_mounts.shutdown()
        if self.bin_load is not None:
            self.bin_load.shutdown()
        if self.clutter is not None:
            self.clutter.shutdown()
        for name in ('conveyor_monitor', 'rabbit_feeder'):
            instance = self.namespace.get(name)
            if instance is not None:
                instance.shutdown()
        clear_spawned_objects(self.stage)
        if self.rgb_resources:
            self.rgb_detach(self.rgb_resources)
        self.stage_subscription = None
        if self.minimum_frame_rate:
            settings = carb.settings.get_settings()
            if settings.get('/persistent/simulation/minFrameRate') == self.minimum_frame_rate:
                settings.set('/persistent/simulation/minFrameRate', self.previous_min_frame_rate)
            omni.timeline.get_timeline_interface().set_target_framerate(self.previous_target_framerate)
        self.status = 'closed'


def get_manager(stage, runtime_dir):
    global _manager
    stage_id = omni.usd.get_context().get_stage_id()
    if _manager is None or not _manager.alive or _manager.stage_id != stage_id:
        if _manager is not None:
            _manager.shutdown()
        _manager = FactoryRuntime(stage, runtime_dir)
    return _manager
