import copy
import json
import os
import subprocess
import threading
from collections import OrderedDict
from pathlib import Path

from ament_index_python.packages import get_package_prefix
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ptc_interfaces.msg import CapabilityStatus, Detection, DetectionArray, RgbdFrame

from .rf_detr_adapter import RfDetrAdapter


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class InferenceNode(Node):
    def __init__(self):
        super().__init__("inference")
        path = self.declare_parameter("settings_file", "config/settings.yaml").value
        executable = "settings_check.exe" if os.name == "nt" else "settings_check"
        checker = Path(get_package_prefix("ptc_app")) / "lib" / "ptc_app" / executable
        # One authoritative C++ loader. Do not reimplement YAML/revision validation in Python.
        exported = subprocess.run([str(checker), path, "--export-json"], check=True, capture_output=True, text=True)
        self.config = json.loads(exported.stdout)
        if self.config["schema_version"] != 2 or self.get_namespace() != "/" + self.config["namespace"]:
            raise ValueError("inference requires matching ready schema v2 namespace")
        if self.get_parameter("use_sim_time").value != self.config["use_sim_time"]:
            raise ValueError("use_sim_time differs from Settings")
        self._condition = threading.Condition()
        self._cache = OrderedDict()
        self._latest = None
        self._stopping = False
        self._ready = False
        self._error = "loading RF-DETR"
        self._epoch = 1
        self._sequence = 0
        self._last_clock = 0
        endpoints = self.config["endpoints"]
        self._publisher = self.create_publisher(DetectionArray, endpoints["detections"], 5)
        self._status = self.create_publisher(CapabilityStatus, endpoints["inference_status"], 5)
        self._subscription = self.create_subscription(RgbdFrame, endpoints["frame"], self._frame, qos_profile_sensor_data)
        self._timer = self.create_timer(0.05, self._tick, clock=Clock(clock_type=ClockType.SYSTEM_TIME))
        self._worker = threading.Thread(target=self._run_model, name="rf_detr_worker")
        self._worker.start()

    def _key(self, frame):
        return (frame.key.epoch, frame.frame_id, frame.calibration_id, frame.key.settings_revision)

    def _valid(self, frame):
        now = self.get_clock().now().nanoseconds
        age = (now - stamp_ns(frame.color.header.stamp)) * 1e-9
        return (
            frame.key.settings_revision == self.config["settings_revision"]
            and frame.key.epoch == self._epoch
            and frame.calibration_id == self.config["calibration_id"]
            and -self.config["sensor_future_tolerance_sec"] <= age < self.config["frame_max_age_sec"]
            and frame.depth_registered and frame.frame_id != 0
            and frame.color.width == frame.depth.width and frame.color.height == frame.depth.height
            and len(frame.color.data) + len(frame.depth.data) <= self.config["max_frame_megabytes"] * 1024 * 1024
        )

    def _frame(self, frame):
        with self._condition:
            if self._ready and self._valid(frame):
                self._latest = frame  # stream backpressure: replace unprocessed latest, never task evidence
                self._condition.notify()

    def _predict(self, frame, adapter):
        key = self._key(frame)
        with self._condition:
            cached = self._cache.get(key)
        if cached is not None:
            return copy.deepcopy(cached)
        if not self._valid(frame):
            raise ValueError("frame expired before model execution")
        output = DetectionArray()
        output.key = copy.deepcopy(frame.key)
        output.frame_id = frame.frame_id
        output.capture_stamp = frame.color.header.stamp
        output.model_version = self.config["model_version"]
        output.decision_source = "assumed_ng" if self.config["operation_mode"] == "assumed_ng_demo" else "model"
        for label, defect, confidence, bbox, mask in adapter.predict(frame):
            detection = Detection()
            detection.class_id, detection.is_defect = label, defect
            detection.confidence, detection.bbox_xyxy = confidence, bbox
            if mask:
                detection.mask.header = frame.color.header
                detection.mask.format = "png"
                detection.mask.data = mask
            output.detections.append(detection)
        output.complete = True
        with self._condition:
            self._cache[key] = copy.deepcopy(output)
            while len(self._cache) > self.config["max_cache_frames"]:
                self._cache.popitem(last=False)
        return output

    def _run_model(self):
        try:
            if self.config["operation_mode"] == "assumed_ng_demo":
                from .assumed_ng_adapter import AssumedNgAdapter
                adapter = AssumedNgAdapter(self.config)
            else:
                adapter = RfDetrAdapter(self.config)
        except Exception as error:
            with self._condition:
                self._error = str(error)
            return
        with self._condition:
            self._ready, self._error = True, ("Explicit assumed-NG demo; no defect model evaluated" if self.config["operation_mode"] == "assumed_ng_demo" else "RF-DETR checkpoint loaded and class mapping verified")
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._stopping or self._latest is not None)
                if self._stopping:
                    return
                frame, self._latest = self._latest, None
            try:
                self._publisher.publish(self._predict(frame, adapter))
            except Exception as error:
                self.get_logger().warning(str(error))

    def _tick(self):
        now = self.get_clock().now().nanoseconds
        with self._condition:
            if now < self._last_clock:
                self._epoch += 1
                self._cache.clear()
                self._latest = None
            self._last_clock = now
            ready, message = self._ready, self._error
        status = CapabilityStatus()
        self._sequence += 1
        status.key.epoch, status.key.sequence = self._epoch, self._sequence
        status.key.settings_revision = self.config["settings_revision"]
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = self.config["world_frame"]
        status.ready, status.message = ready, message[:512]
        status.binding_revision = self.config["binding_revision"]
        status.calibration_id, status.model_version = self.config["calibration_id"], self.config["model_version"]
        self._status.publish(status)

    def stop_runtime(self):
        with self._condition:
            self._stopping = True
            self._condition.notify_all()
        self._worker.join()
