"""Live indication from physics ground truth; no ROS or transport actuation."""
import asyncio
import itertools
import pathlib
import time
from pxr import Gf, Sdf, Usd, UsdGeom
import omni.kit.app
import omni.timeline
import omni.usd

directory=pathlib.Path(_factory_monitor_dir)


class IndustrialConveyorMonitor:
    def __init__(self, feeder, directory):
        self.stage = omni.usd.get_context().get_stage()
        self.timeline = omni.timeline.get_timeline_interface()
        self.feeder, self.directory = feeder, directory
        self.base = '/World/FactoryDemo/Conveyor/IndustrialDetails'
        self.sensors = {p.GetName(): p for p in self.stage.GetPrimAtPath(self.base+'/Sensors').GetChildren()}
        self.states = {name: False for name in self.sensors}
        self.counts = {name: 0 for name in self.sensors}
        self.bounds = {}
        self.last_wall = 0.; self.last_report = 0.; self.last_sim = None
        self.distance = 0.; self.alive = True; self.lights = {}
        self.task = asyncio.create_task(self.loop())

    def lamp(self, name, active):
        if self.lights.get(name) == active:
            return
        self.lights[name] = active
        shader = self.stage.GetPrimAtPath(self.base+'/Looks/'+name+'/Shader')
        with Usd.EditContext(self.stage,self.stage.GetSessionLayer()):
            shader.GetAttribute('inputs:emissive_intensity').Set(200. if active else .0)

    def world_bounds(self, path, pose):
        if path not in self.bounds:
            prim = self.stage.GetPrimAtPath(path)
            bound = UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render']).ComputeUntransformedBound(prim).ComputeAlignedRange()
            self.bounds[path] = list(itertools.product(*zip(bound.GetMin(),bound.GetMax())))
        xyz, q = pose['position'], pose['rotation']
        transform = Gf.Matrix4d(1.).SetRotate(Gf.Quatd(q[3],Gf.Vec3d(*q[:3])))
        transform.SetTranslateOnly(Gf.Vec3d(*xyz))
        points = [transform.Transform(Gf.Vec3d(*p)) for p in self.bounds[path]]
        return ([min(p[i] for p in points) for i in range(3)],
                [max(p[i] for p in points) for i in range(3)])

    async def loop(self):
        while self.alive:
            await omni.kit.app.get_app().next_update_async()
            wall=time.monotonic()
            if wall-self.last_wall < .05:
                continue
            self.last_wall=wall;now=self.timeline.get_current_time();playing=self.timeline.is_playing()
            if self.last_sim is not None and now < self.last_sim-.001:
                self.counts=dict.fromkeys(self.sensors,0);self.states=dict.fromkeys(self.sensors,False)
                self.bounds.clear();self.distance=0.
            speed=float(self.stage.GetPrimAtPath('/World/FactoryDemo/Conveyor/Belt/ConveyorGraph').GetAttribute('graph:variable:Velocity').Get())
            if playing and self.last_sim is not None:
                self.distance+=max(0.,now-self.last_sim)*abs(speed)
            self.last_sim=now
            error=bool(self.feeder.last_error)
            self.lamp('GreenLens',playing and not error)
            self.lamp('AmberLens',not playing and not error)
            self.lamp('RedLens',error)
            # Removed rabbits no longer need cached geometry. Keep only
            # active paths, including objects that remain physically in a bin.
            for path in self.bounds.keys() - self.feeder.active.keys():
                del self.bounds[path]
            boxes=[]
            for path in list(self.feeder.active):
                pose=self.feeder.pose(path)
                if pose:boxes.append(self.world_bounds(path,pose))
            enabled=self.stage.GetPrimAtPath(self.base).GetAttribute('line:monitorEnabled').Get()
            for name,prim in self.sensors.items():
                y=prim.GetAttribute('line:beamY').Get();z=prim.GetAttribute('line:beamZ').Get()
                occupied=bool(enabled) and any(lo[0]<.788 and hi[0]>.412 and lo[1]-.001<=y<=hi[1]+.001 and lo[2]<=z<=hi[2] for lo,hi in boxes)
                if occupied and not self.states[name] and playing:self.counts[name]+=1
                if occupied != self.states[name] or prim.GetAttribute('line:count').Get()!=self.counts[name]:
                    with Usd.EditContext(self.stage,self.stage.GetSessionLayer()):
                        prim.GetAttribute('line:occupied').Set(occupied)
                        prim.GetAttribute('line:count').Set(self.counts[name])
                self.states[name]=occupied;self.lamp('Sensor_'+name,occupied)

    def shutdown(self):
        self.alive=False;self.task.cancel()


if 'conveyor_monitor' in globals():
    conveyor_monitor.shutdown()
conveyor_monitor=IndustrialConveyorMonitor(rabbit_feeder,directory)
print('Industrial conveyor indicators active; ground-truth AABB occupancy, no robot or belt commands')
