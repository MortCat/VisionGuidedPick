"""Align wrist camera mounts to the stopped hand poses in a session layer."""
import asyncio

import carb
import omni.kit.app
import omni.timeline
import omni.usd
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics


def joint_frame(position, rotation):
    matrix = Gf.Matrix4d(1).SetRotate(Gf.Quatd(rotation))
    return matrix.SetTranslateOnly(Gf.Vec3d(position))


class StoppedWristMounts:
    def __init__(self, stage):
        self.stage = stage
        self.timeline = omni.timeline.get_timeline_interface()
        self.layer = Sdf.Layer.CreateAnonymous('FactoryStoppedWristMounts.usda')
        self.task = None
        self.timeline_subscription = self.timeline.get_timeline_event_stream().create_subscription_to_pop(
            self.on_timeline, name='FactoryStoppedWristMounts')
        self.stage_subscription = omni.usd.get_context().get_stage_event_stream().create_subscription_to_pop(
            self.on_stage, name='FactoryStoppedWristMountsLifecycle')
        if self.timeline.is_stopped():
            self.schedule()

    def on_timeline(self, event):
        if event.type == int(omni.timeline.TimelineEventType.STOP):
            self.schedule()

    def on_stage(self, event):
        if event.type == int(omni.usd.StageEventType.CLOSING):
            self.shutdown()

    def schedule(self):
        if self.task is None or self.task.done():
            self.task = asyncio.ensure_future(self.align_after_stop())

    async def align_after_stop(self):
        app = omni.kit.app.get_app()
        await app.next_update_async()
        await app.next_update_async()
        if not self.timeline.is_stopped():
            return
        try:
            self.align()
        except Exception as exc:
            carb.log_warn(f'Unable to align stopped wrist cameras: {exc}')

    def align(self):
        cache = UsdGeom.XformCache()
        updates = []
        for arm in ('arm_01', 'arm_02'):
            path = f'/World/FactoryDemo/Robots/{arm}'
            joint = UsdPhysics.FixedJoint.Get(self.stage, path + '/WristD455MountJoint')
            if not joint:
                continue
            hand, camera = (self.stage.GetPrimAtPath(rel.GetTargets()[0])
                            for rel in (joint.GetBody0Rel(), joint.GetBody1Rel()))
            wrapper = camera.GetParent()
            wrapper_xform = UsdGeom.Xformable(wrapper)
            camera_xform = UsdGeom.Xformable(camera)
            wrapper_ops = wrapper_xform.GetOrderedXformOps()
            if (len(wrapper_ops) != 1 or wrapper_ops[0].GetOpName() != 'xformOp:transform'
                    or wrapper_xform.GetResetXformStack() or camera_xform.GetResetXformStack()):
                raise RuntimeError(f'Unexpected camera transform stack for {arm}')
            hand_anchor = joint_frame(joint.GetLocalPos0Attr().Get(), joint.GetLocalRot0Attr().Get())
            camera_anchor = joint_frame(joint.GetLocalPos1Attr().Get(), joint.GetLocalRot1Attr().Get())
            camera_world = camera_anchor.GetInverse() * hand_anchor * cache.GetLocalToWorldTransform(hand)
            wrapper_local = (camera_xform.GetLocalTransformation().GetInverse() * camera_world
                             * cache.GetLocalToWorldTransform(wrapper.GetParent()).GetInverse())
            if not Gf.IsClose(wrapper_ops[0].Get(), wrapper_local, 1e-7):
                updates.append((wrapper_ops[0], wrapper_local))
        if updates:
            session = self.stage.GetSessionLayer()
            if self.layer.identifier not in session.subLayerPaths:
                session.subLayerPaths.insert(0, self.layer.identifier)
            with Usd.EditContext(self.stage, self.layer):
                for op, matrix in updates:
                    op.Set(matrix)

    def shutdown(self):
        self.timeline_subscription = None
        self.stage_subscription = None
        if self.task is not None and not self.task.done():
            self.task.cancel()
        paths = self.stage.GetSessionLayer().subLayerPaths
        if self.layer.identifier in paths:
            paths.remove(self.layer.identifier)
