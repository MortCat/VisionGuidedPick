"""Independent, bounded Morandi cube feeder; never issues robot commands."""
import asyncio
import collections
import math
import random
import time

import omni.physx
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics


class ClutterFeeder:
    def __init__(self, stage, rabbit_feeder, asset_path):
        self.stage = stage
        self.rabbit_feeder = rabbit_feeder
        self.asset = str(asset_path)
        self.timeline = omni.timeline.get_timeline_interface()
        factory = stage.GetPrimAtPath('/World/FactoryDemo')
        self.interval = float(factory.GetAttribute('clutter:intervalSeconds').Get())
        self.size_range = list(factory.GetAttribute('clutter:sizeRangeMeters').Get())
        source = Usd.Stage.Open(self.asset)
        self.colors = [p.GetName() for p in source.GetPrimAtPath('/Blocks').GetChildren()]
        self.rng = random.Random()
        self.active = {}
        self.footprints = {}
        self.target_radii = {
            variant: math.hypot(*(max(abs(lo[axis]), abs(hi[axis])) for axis in (0, 1)))
            for variant, (lo, hi) in rabbit_feeder.asset_bounds.items()
        }
        self.events = collections.deque(maxlen=32)
        self.sequence = self.removed = 0
        self.previous_color = None
        self.next_time = None
        self.pending_reset = False
        self.alive = True
        self.last_cleanup = -1
        self.error = None
        self.subscription = self.timeline.get_timeline_event_stream().create_subscription_to_pop(self.on_event, name='MorandiClutterLifecycle')
        self.task = asyncio.ensure_future(self.loop())

    def on_event(self, event):
        if event.type == int(omni.timeline.TimelineEventType.STOP):
            self.pending_reset = True

    def remove(self, path):
        with Usd.EditContext(self.stage, self.stage.GetSessionLayer()):
            self.stage.RemovePrim(path)
        self.active.pop(path, None)
        self.footprints.pop(path, None)
        self.removed += 1

    def reset(self):
        for path in list(self.active):
            self.remove(path)
        self.next_time = None
        self.sequence = self.removed = 0
        self.events.clear()
        self.last_cleanup = -1
        self.pending_reset = False

    @staticmethod
    def separated(position, radius, obstacles):
        # Planar clearance measured between conservative footprints.
        gap = .03
        return all((position[0]-other[0])**2 + (position[1]-other[1])**2 >=
                   (radius + other_radius + gap)**2 for other, other_radius in obstacles)

    def position_at(self, path, now, born, initial):
        # A just-authored rigid body may not exist in the physics view yet.
        if now-born < .10:
            return initial
        if not self.stage.GetPrimAtPath(path):
            return None
        state = self.rabbit_feeder.pose(path)
        return state['position'] if state else None

    def clear_of_blocks(self, position, radius, now):
        for path, born in self.active.items():
            initial, block_radius = self.footprints[path]
            other = self.position_at(path, now, born, initial)
            if other is not None and not self.separated(position, radius, [(other, block_radius)]):
                return False
        return True

    def spawn(self, now):
        feeder = self.rabbit_feeder
        # Reserve the next inlet slot for a scheduled rabbit or bear.
        if feeder.enabled and len(feeder.active) < feeder.maximum_active and (
            feeder.next_time is None or feeder.next_time - now < .75
        ):
            return
        targets = []
        for path, metadata in feeder.active.items():
            position = self.position_at(path, now, metadata['spawn_sim'], metadata['initial_position'])
            if position is not None:
                targets.append((position, self.target_radii[metadata['variant']]))
        color = self.rng.choice([c for c in self.colors if c != self.previous_color])
        size = self.rng.uniform(*self.size_range)
        values = [self.rng.gauss(0, 1) for _ in range(4)]
        length = sum(v*v for v in values) ** 0.5
        values = [v / length for v in values]
        q = Gf.Quatd(values[0], Gf.Vec3d(*values[1:]))
        rotation = Gf.Matrix4d().SetRotate(Gf.Rotation(q))
        height = size * 0.5 * sum(abs(rotation[i][2]) for i in range(3))
        base = feeder.position
        block_radius = size * math.sqrt(3) / 2
        x_spread, y_spread = (.035, .105) if feeder.l_inlet else (.105, .035)
        belt_z = .75 if feeder.l_inlet else feeder.belt_bounds[1][2]
        for _ in range(6):
            position = Gf.Vec3d(base[0] + self.rng.uniform(-x_spread, x_spread),
                                base[1] + self.rng.uniform(-y_spread, y_spread), belt_z + height + .003)
            if self.separated(position, block_radius, targets):
                break
        else:
            return
        # Bound scene growth when objects jam or remain in bins.
        if len(self.active) >= 160:
            self.remove(next(iter(self.active)))
        self.sequence += 1
        path = f'/World/FactoryDemo/Objects/MorandiBlock_{self.sequence:06d}'
        with Usd.EditContext(self.stage, self.stage.GetSessionLayer()):
            prim = UsdGeom.Xform.Define(self.stage, path).GetPrim()
            prim.GetReferences().AddReference(self.asset, '/Blocks/' + color)
            xf = UsdGeom.Xformable(prim)
            xf.AddTranslateOp().Set(position)
            xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(q)
            xf.AddScaleOp().Set(Gf.Vec3f(size))
            UsdPhysics.RigidBodyAPI.Apply(prim)
            UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(max(.008, size**3 * 450))
            prim.CreateAttribute('object:type', Sdf.ValueTypeNames.String).Set('MorandiBlock')
            prim.CreateAttribute('object:palette', Sdf.ValueTypeNames.String).Set(color)
        self.active[path] = now
        self.footprints[path] = (list(position), block_radius)
        self.previous_color = color
        self.events.append({'sim_time': now, 'path': path, 'color': color, 'size_m': size, 'position': list(position)})

    def cleanup(self, now):
        for path, born in list(self.active.items()):
            if not self.stage.GetPrimAtPath(path):
                self.active.pop(path, None)
                self.footprints.pop(path, None)
                continue
            if now-born > 180:
                self.remove(path)
                continue
            state = omni.physx.get_physx_interface().get_rigidbody_transformation(path)
            if state and 'position' in state:
                x,y,z = state['position']
                if z < .25 or y > self.rabbit_feeder.outlet_y or abs(x) > 5 or abs(y) > 7:
                    self.remove(path)

    async def loop(self):
        while self.alive:
            try:
                if self.pending_reset:
                    self.reset()
                if self.timeline.is_playing():
                    now = self.timeline.get_current_time()
                    if self.next_time is None:
                        self.next_time = now
                    if now + 1e-6 >= self.next_time:
                        self.spawn(now)
                        self.next_time += self.interval
                    if now-self.last_cleanup >= 1:
                        self.cleanup(now)
                        self.last_cleanup = now
            except asyncio.CancelledError:
                raise
            except Exception as error:
                self.error = repr(error)
                # Keep the existing rabbit feeder and robot lifecycle intact.
            await asyncio.sleep(.02)

    def shutdown(self):
        self.alive = False
        self.task.cancel()
        self.subscription = None
