"""Feed standard rabbits with seeded pose and inlet placement.

Run in the existing Isaac context after opening the saved factory USD. Physics
poses remain authoritative; spawned units are authored in the session layer.
"""
import asyncio
import collections
import hashlib
import json
import math
import os
import pathlib
import random
import secrets
import sys
import time
import uuid
import carb

import omni.kit.app
import omni.physx
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom

stage = omni.usd.get_context().get_stage()
timeline = omni.timeline.get_timeline_interface()
graph_managed = bool(globals().get('_factory_graph_managed', False))
if not graph_managed:
    timeline.pause()
workspace = pathlib.Path(os.environ.get('WORKSPACE_DIR', str(pathlib.Path.home() / 'workspace')))
v3_dir = pathlib.Path(globals().get('_factory_asset_dir', workspace / 'GeneralForAgent/assets/factory_v3'))
sys.path.insert(0, str(v3_dir))
import procedural
# Refresh the trusted sibling source during live iteration. Kit can retain the
# previous module code even after a standard importlib.reload().
exec(compile((v3_dir / 'procedural.py').read_text(), str(v3_dir / 'procedural.py'), 'exec'), procedural.__dict__)
from procedural import sample_inlet
feed_config = json.loads((v3_dir / 'feed_config.json').read_text())
good_variant = feed_config['good_variant']
v3_physics_assets = {good_variant: str(v3_dir / feed_config['assets'][good_variant])}
variant_report = json.loads((v3_dir / 'variant_report.json').read_text())
factory = stage.GetPrimAtPath('/World/FactoryDemo')
alternate_bear = bool(factory.GetAttribute('feeder:alternateBear').Get())
if alternate_bear:
    bear = factory.GetAttribute('feeder:bearAsset').Get()
    v3_physics_assets['teddy_bear'] = Sdf.ComputeAssetPathRelativeToLayer(stage.GetRootLayer(), bear.path)
    variant_report['teddy_bear'] = {'quality': 'distractor', 'defect': 'none', 'scale': 1.0}
asset_hashes = {name: hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()
                for name, path in v3_physics_assets.items()}
asset_bounds = {}
for name, path in v3_physics_assets.items():
    asset = Usd.Stage.Open(path)
    bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default', 'render']).ComputeWorldBound(asset.GetDefaultPrim()).ComputeAlignedRange()
    asset_bounds[name] = [list(bounds.GetMin()), list(bounds.GetMax())]
if 'rabbit_feeder' in globals():
    rabbit_feeder.shutdown()
for layer in ([stage.GetSessionLayer()] if graph_managed else [stage.GetRootLayer(), stage.GetSessionLayer()]):
    with Usd.EditContext(stage, layer):
        for child in list(stage.GetPrimAtPath('/World/FactoryDemo/Objects').GetChildren()):
            if child.GetName().startswith(('RabbitFeed_', 'BearFeed_')):
                stage.RemovePrim(child.GetPath())
dynamic_layer = stage.GetSessionLayer()


class FactoryRabbitFeederV3:
    def __init__(self):
        self.stage, self.timeline = stage, timeline
        self.alternate_bear = alternate_bear
        self.interval = float(factory.GetAttribute('feeder:intervalSeconds').Get() or feed_config['interval_sim_seconds'])
        self.speed, self.maximum_active = .06, 64
        self.active = {}
        self.asset_bounds = asset_bounds
        self.block_spawn_clearance = None
        self.events = collections.deque(maxlen=500)
        self.samples = collections.deque(maxlen=500)
        self.sequence = self.removed = 0
        self.alive = self.enabled = True
        self.pending_reset = False
        self.new_run()
        self.next_time = self.last_time = self.begin_sim = self.begin_wall = None
        self.last_report = -1.
        self.last_error = None
        self.directory = v3_dir
        belt = stage.GetPrimAtPath('/World/FactoryDemo/Conveyor/Belt/Geometry')
        bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default', 'render']).ComputeWorldBound(belt).ComputeAlignedRange()
        self.belt_bounds = [list(bounds.GetMin()), list(bounds.GetMax())]
        self.position = Gf.Vec3d((bounds.GetMin()[0] + bounds.GetMax()[0]) / 2,
                                bounds.GetMin()[1] + .12, bounds.GetMax()[2] + .005)
        self.outlet_y = bounds.GetMax()[1] + .18
        self.l_inlet = (feed_config.get('inlet_layout') == 'L_negative_x'
                        and bool(stage.GetPrimAtPath('/World/FactoryDemo/Conveyor/LInlet')))
        if self.l_inlet:
            # Sampling uses a +Y local frame, rotated 90 degrees into world -X.
            # Its local X .55-.65 becomes world Y -3.65..-3.55, then maps to
            # world X .55-.65 after the quarter-turn into the robot work line.
            self.belt_bounds = [[.425, -3.05, .72], [.775, -1.3, .75]]
            self.position = Gf.Vec3d(2.93, -3.6, .755)
        self.subscription = timeline.get_timeline_event_stream().create_subscription_to_pop(
            self.on_event, name='FactoryV3Feeder')
        self.task = asyncio.create_task(self.loop())

    def on_event(self, event):
        if event.type == int(omni.timeline.TimelineEventType.STOP):
            self.pending_reset = True

    def new_run(self):
        self.run_id = uuid.uuid4().hex[:12]
        self.run_seed = secrets.randbits(31)
        self.rng = random.Random(self.run_seed)

    def reset(self):
        for layer in ([dynamic_layer] if graph_managed else [dynamic_layer, self.stage.GetRootLayer()]):
            with Usd.EditContext(self.stage, layer):
                for path in self.active:
                    self.stage.RemovePrim(path)
        self.active.clear(); self.events.clear()
        self.sequence = self.removed = 0
        self.new_run()
        self.next_time = self.last_time = self.begin_sim = self.begin_wall = None
        self.last_report = -1.; self.pending_reset = False

    def pose(self, path):
        state = omni.physx.get_physx_interface().get_rigidbody_transformation(path)
        if state and 'position' in state:
            return {'position': list(state['position']), 'rotation': list(state['rotation'])}

    def spawn(self, now):
        if len(self.active) >= self.maximum_active:
            return
        variant = 'teddy_bear' if self.alternate_bear and (self.sequence + 1) % 2 == 0 else feed_config['good_variant']
        spec = variant_report[variant]
        seed = self.rng.getrandbits(31)
        yaw = random.Random(seed).uniform(0., 360.)
        position, placement = sample_inlet(asset_bounds[variant], yaw, self.belt_bounds,
                                           feed_config['placement'], seed ^ 0x584C)
        if self.l_inlet:
            position = [-position[1], position[0] - 4.2, position[2]]
            yaw = (yaw + 90.) % 360.
            placement['sampling_frame'] = 'L_inlet_local_X_world_Y_local_Y_world_negative_X'
            placement['world_lateral_axis'] = 'Y'
        if self.block_spawn_clearance:
            lo, hi = asset_bounds[variant]
            radius = math.hypot(*(max(abs(lo[axis]), abs(hi[axis])) for axis in (0, 1)))
            if not self.block_spawn_clearance(position, radius, now):
                self.next_time = now + .20
                return
        self.sequence += 1
        prefix = 'BearFeed' if variant == 'teddy_bear' else 'RabbitFeed'
        path = f'/World/FactoryDemo/Objects/{prefix}_{self.sequence:06d}'
        with Usd.EditContext(self.stage, dynamic_layer):
            prim = UsdGeom.Xform.Define(self.stage, path).GetPrim()
            prim.GetReferences().AddReference(v3_physics_assets[variant])
            prim.SetInstanceable(False)
            transform = UsdGeom.Xformable(prim)
            transform.AddTranslateOp().Set(Gf.Vec3d(*position))
            transform.AddRotateZOp().Set(yaw)
            prim.CreateAttribute('object:type', Sdf.ValueTypeNames.String).Set('TeddyBear' if variant == 'teddy_bear' else 'Rabbit')
            prim.CreateAttribute('inspection:variant', Sdf.ValueTypeNames.String).Set(variant)
            prim.CreateAttribute('inspection:spawnSeed', Sdf.ValueTypeNames.Int64).Set(seed)
        metadata = {'variant': variant, 'quality': spec['quality'], 'defect': spec['defect'],
                    'size_scale': spec['scale'], 'spawn_sim': now, 'spawn_wall': time.monotonic(),
                    'initial_position': position, 'initial_yaw_deg': yaw, 'random_seed': seed,
                    'placement': placement, 'defect_parameters': None, 'run_id': self.run_id,
                    'source_mesh_sha256': None if variant == 'teddy_bear' else feed_config['source_mesh_sha256'],
                    'source_asset_sha256': asset_hashes[variant]}
        self.active[path] = metadata
        self.events.append({'path': path, 'sim_time': now, 'yaw_deg': yaw, **metadata})
        # Schedule from the actual release so a full belt cannot queue a burst.
        self.next_time = now + self.interval

    def report(self, now):
        data = {'enabled': self.enabled, 'playing': timeline.is_playing(), 'spawned': self.sequence,
                'active_count': len(self.active), 'removed_at_outlet': self.removed,
                'interval_sim_seconds': self.interval, 'belt_speed_m_s': self.speed,
                'policy': 'rabbit_bear_alternating' if self.alternate_bear else 'standard_rabbit_only',
                'good_variant': feed_config['good_variant'],
                'spawn_position_m': list(self.position), 'outlet_y_m': self.outlet_y,
                'objects': {p: {**m, 'pose': self.pose(p)} for p, m in self.active.items()},
                'spawn_events': list(self.events), 'error': self.last_error}
        data.update(run_id=self.run_id, run_seed=self.run_seed,
                    placement_policy=feed_config['placement'])
        return data

    async def loop(self):
        try:
            while self.alive:
                await omni.kit.app.get_app().next_update_async()
                if self.pending_reset:
                    self.reset()
                if not self.enabled or not timeline.is_playing():
                    continue
                now = timeline.get_current_time()
                if self.last_time is not None and now < self.last_time:
                    self.reset()
                if self.begin_sim is None:
                    self.begin_sim, self.begin_wall = now, time.monotonic()
                self.last_time = now
                for path in list(self.active):
                    pose = self.pose(path)
                    if pose and (pose['position'][1] > self.outlet_y or pose['position'][2] < .20):
                        with Usd.EditContext(self.stage, dynamic_layer):
                            self.stage.RemovePrim(path)
                        del self.active[path]; self.removed += 1
                if self.next_time is None or now + 1e-6 >= self.next_time:
                    self.spawn(now)
        except Exception as exc:
            self.last_error = repr(exc); self.enabled = False
            timeline.pause()
            raise

    def pause(self):
        timeline.pause(); return self.report(timeline.get_current_time())

    def start(self):
        self.enabled = True
        self.stage.SetEditTarget(dynamic_layer)
        timeline.play()

    def shutdown(self):
        self.alive = self.enabled = False
        self.subscription = None; self.task.cancel()


rabbit_feeder = FactoryRabbitFeederV3()
if 'conveyor_monitor' in globals():
    conveyor_monitor.feeder = rabbit_feeder
if 'target_feed' in globals():
    target_feed.feeder = rabbit_feeder
    target_feed.previous.clear()
    target_feed.run_id += 1
if 'runtime' in globals():
    runtime.run = False; runtime.feeder = None
factory = stage.GetPrimAtPath('/World/FactoryDemo')
if not graph_managed:
 with Usd.EditContext(stage, stage.GetRootLayer()):
    factory.CreateAttribute('feeder:configPath', Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(str(v3_dir / 'feed_config.json')))
    factory.CreateAttribute('feeder:startupScript', Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(str(v3_dir / 'feed_v3.py')))
    factory.CreateAttribute('feeder:policy', Sdf.ValueTypeNames.String).Set('rabbit_bear_alternating' if alternate_bear else 'standard_rabbit_only')
    factory.CreateAttribute('factory:revision', Sdf.ValueTypeNames.String).Set(
        'industrial_rabbit_v4_L_inlet' if rabbit_feeder.l_inlet else 'industrial_rabbit_v3_robot_work_band')
    stage.GetRootLayer().Save()
stage.SetEditTarget(dynamic_layer)
print(f'Factory feeder armed: interval={rabbit_feeder.interval}s, alternate_bear={alternate_bear}; no robot commands.')
