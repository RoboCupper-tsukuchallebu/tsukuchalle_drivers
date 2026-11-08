"""Counts /velodyne_points frames in its own process (= its own Fast DDS participant) and prints `COUNT <n>`.

Started by test_velodyne_pipeline.py: nav-stack, the pointcloud node and Foxglove are separate
processes in production, and each one has its own shared memory segment and its own reader.
Usage: count_subscriber.py <best-effort|reliable>
"""

import sys
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

QOS_BY_NAME = {
    "best-effort": qos_profile_sensor_data,
    "reliable": QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
}
IDLE_SECONDS = 4.0
DEADLINE_SECONDS = 60.0


def main(qos_name):
    rclpy.init()
    node = rclpy.create_node(f"count_subscriber_{qos_name.replace('-', '_')}")
    frame_count = 0
    last_frame_time = None

    def on_frame(_serialized):
        nonlocal frame_count, last_frame_time
        frame_count += 1
        last_frame_time = time.monotonic()

    node.create_subscription(
        PointCloud2, "/velodyne_points", on_frame, QOS_BY_NAME[qos_name], raw=True
    )
    deadline = time.monotonic() + DEADLINE_SECONDS
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if last_frame_time is not None and time.monotonic() - last_frame_time > IDLE_SECONDS:
            break
    print(f"COUNT {frame_count}", flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv[1])
