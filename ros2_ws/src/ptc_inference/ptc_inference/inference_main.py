import rclpy
from rclpy.executors import MultiThreadedExecutor
from ptc_inference.inference_node import InferenceNode

def main():
    rclpy.init()
    node = InferenceNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.stop_runtime()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
