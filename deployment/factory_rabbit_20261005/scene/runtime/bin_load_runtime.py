"""Read two fixed support joints. The inputs are forces, with no object identity."""
from math import isfinite

import carb
import omni.kit.app
import omni.timeline
import omni.usd
from omni.physics import tensors
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import Trigger
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data


class BinLoadPublisher:
    PERIOD_NS = 100_000_000

    def __init__(self, stage_id, clear_bin):
        self.stage_id = stage_id
        self.clear_bin = clear_bin
        self.timeline = omni.timeline.get_timeline_interface()
        self.context = Context()
        self.node = None
        self.executor = None
        self.subscription = None
        self.stage_subscription = None
        self.simulation = None
        self.views = {}
        self.last_ns = None
        self.error = None
        try:
            rclpy.init(context=self.context)
            self.node = rclpy.create_node('factory_bin_load', context=self.context)
            self.executor = SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
            self.publishers = {
                arm: self.node.create_publisher(WrenchStamped, f'/{arm}/bin/load', qos_profile_sensor_data)
                for arm in ('arm_01', 'arm_02')
            }
            self.services = [
                self.node.create_service(Trigger, f'/{arm}/bin/clear',
                                         lambda req, reply, arm=arm: self.on_clear(arm, reply))
                for arm in self.publishers
            ]
            self.subscription = omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
                self.update, name='FactoryBinLoad')
            self.stage_subscription = omni.usd.get_context().get_stage_event_stream().create_subscription_to_pop(
                self.on_stage_event, name='FactoryBinLoadLifecycle')
        except Exception:
            self.shutdown()
            raise

    def on_stage_event(self, event):
        if event.type == int(omni.usd.StageEventType.CLOSING):
            self.shutdown()

    def reset_views(self):
        self.views.clear()
        self.simulation = None

    def on_clear(self, arm, reply):
        if not self.timeline.is_playing():
            reply.message = 'Play the simulation before clearing a bin'
            return reply
        try:
            count = self.clear_bin(arm)
            reply.success = True
            reply.message = f'Removed {count} generated object(s)'
        except Exception as exc:
            reply.message = str(exc)
        return reply

    def update(self, _event):
        self.executor.spin_once(timeout_sec=0)
        if self.timeline.is_stopped():
            self.last_ns = None
            self.reset_views()
            return
        if not self.timeline.is_playing():
            return
        stamp_ns = round(self.timeline.get_current_time() * 1_000_000_000)
        if stamp_ns < 300_000_000:
            return
        if self.last_ns is not None:
            if stamp_ns < self.last_ns:
                self.reset_views()
            elif stamp_ns - self.last_ns < self.PERIOD_NS:
                return
        self.last_ns = stamp_ns
        try:
            if self.simulation is None:
                self.simulation = tensors.create_simulation_view('numpy', stage_id=self.stage_id)
                for arm in self.publishers:
                    path = f'/World/FactoryDemo/Stations/{arm}/BinScaleBase'
                    view = self.simulation.create_articulation_view(path)
                    if view.count != 1 or view.max_links != 2:
                        raise RuntimeError(f'Expected two links for {path}')
                    self.views[arm] = view
            for arm, view in self.views.items():
                load = float(view.get_link_incoming_joint_force()[0, 1, 2])
                if isfinite(load):
                    msg = WrenchStamped()
                    msg.header.frame_id = f'{arm}/bin_load'
                    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
                    msg.wrench.force.z = load
                    self.publishers[arm].publish(msg)
            self.error = None
        except Exception as exc:
            self.reset_views()
            if self.error != str(exc):
                self.error = str(exc)
                carb.log_warn(f'Bin load unavailable: {exc}')

    def shutdown(self):
        self.stage_subscription = None
        self.subscription = None
        self.reset_views()
        if self.executor is not None:
            self.executor.shutdown()
            self.executor = None
        if self.node is not None:
            self.node.destroy_node()
            self.node = None
        if self.context.ok():
            self.context.shutdown()
