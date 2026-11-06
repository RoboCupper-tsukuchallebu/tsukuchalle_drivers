"""Feeds synthetic HDL-32E packets to velodyne_driver_node and velodyne_transform_node.

Run inside the tsukuchalle-velodyne image:
    python3 -m unittest discover -s /test -v

The two nodes are started with the same arguments as ros2/compose.yaml; only device_ip
differs (127.0.0.1, so the sender filter is exercised too).
"""

import itertools
import math
import socket
import struct
import subprocess
import tempfile
import time
import unittest
from contextlib import contextmanager

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from velodyne_msgs.msg import VelodyneScan

import hdl32e_packets

DRIVER_COMMAND = [
    "/opt/ros/jazzy/lib/velodyne_driver/velodyne_driver_node",
    "--ros-args",
    "-p", "model:=32E",
    "-p", "rpm:=600.0",
    "-p", "frame_id:=velodyne",
    "-p", "device_ip:=127.0.0.1",
]
TRANSFORM_COMMAND = [
    "/opt/ros/jazzy/lib/velodyne_pointcloud/velodyne_transform_node",
    "--ros-args",
    "-p", "model:=32E",
    "-p", "calibration:=/opt/ros/jazzy/share/velodyne_pointcloud/params/32db.yaml",
]
RELIABLE_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
SENSOR_ADDRESS = ("127.0.0.1", 2368)

REVOLUTION_COUNT = 20
ACCEPTED_FRAME_LOSS = 3
PACKETS_PER_BATCH = 18
BATCH_INTERVAL_SECONDS = 0.01
DEADLINE_SECONDS = 30.0
IDLE_SECONDS = 2.0

EXPECTED_WIDTH = hdl32e_packets.CHANNELS_PER_BLOCK
EXPECTED_HEIGHT = hdl32e_packets.FIRINGS_PER_REVOLUTION
POINT_STEP = 22
POINT_FORMAT = "<4fHf"  # x, y, z, intensity, ring, time
EXPECTED_FIELD_NAMES = ["x", "y", "z", "intensity", "ring", "time"]
DISTANCE_TOLERANCE_METERS = 0.05


def spin_until(node, condition, deadline_seconds):
    """Spin `node` until `condition()` holds; return whether it did before the deadline."""
    deadline = time.monotonic() + deadline_seconds
    while not condition():
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return False
        rclpy.spin_once(node, timeout_sec=min(remaining, 0.1))
    return True


class LaunchedProcesses:
    def __init__(self, commands):
        self.log_file = tempfile.TemporaryFile()
        self.processes = [
            subprocess.Popen(command, stdout=self.log_file, stderr=subprocess.STDOUT)
            for command in commands
        ]

    def all_alive(self):
        return all(process.poll() is None for process in self.processes)

    def log_text(self):
        self.log_file.seek(0)
        return self.log_file.read().decode(errors="replace")[-4000:]

    def stop(self):
        for process in self.processes:
            process.terminate()
        for process in self.processes:
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        self.log_file.close()


@contextmanager
def launched(commands):
    processes = LaunchedProcesses(commands)
    try:
        yield processes
    finally:
        processes.stop()


def decode_points(cloud):
    return list(struct.iter_unpack(POINT_FORMAT, bytes(cloud.data)))


class VelodynePipelineTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_synthetic_revolutions_become_organized_point_clouds(self):
        node = rclpy.create_node("velodyne_pipeline_test")
        self.addCleanup(node.destroy_node)
        serialized_clouds = []
        best_effort_clouds = []
        # The callbacks only store the serialized bytes (raw=True): converting a 1.5 MB message
        # to Python objects, let alone walking 69,504 points, takes longer than the 100 ms
        # between frames.
        subscription = node.create_subscription(
            PointCloud2, "/velodyne_points", serialized_clouds.append, RELIABLE_QOS, raw=True
        )
        # nav-stack subscribes with SensorDataQoS; this one checks that it is compatible with
        # the reliable publisher. It is not used to count frames: in a container, Fast DDS drops
        # 1.5 MB best-effort samples (20 to 80 % of them in a run on the build machine, while
        # the reliable subscription above got all 20).
        best_effort_subscription = node.create_subscription(
            PointCloud2,
            "/velodyne_points",
            best_effort_clouds.append,
            qos_profile_sensor_data,
            raw=True,
        )

        with launched([DRIVER_COMMAND, TRANSFORM_COMMAND]) as processes:
            ready = spin_until(
                node,
                lambda: processes.all_alive()
                and subscription.get_publisher_count() >= 1
                and best_effort_subscription.get_publisher_count() >= 1
                and node.count_subscribers("/velodyne_packets") >= 1,
                DEADLINE_SECONDS,
            )
            self.assertTrue(ready, f"nodes were not discovered in time:\n{processes.log_text()}")

            sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.addCleanup(sender.close)
            batches = itertools.batched(
                hdl32e_packets.build_revolutions(REVOLUTION_COUNT), PACKETS_PER_BATCH
            )
            next_send_time = time.monotonic()
            for batch in batches:
                for packet in batch:
                    sender.sendto(packet, SENSOR_ADDRESS)
                next_send_time += BATCH_INTERVAL_SECONDS
                spin_until(node, lambda: False, max(next_send_time - time.monotonic(), 0))

            last_count = -1
            last_change = time.monotonic()

            def finished():
                nonlocal last_count, last_change
                if len(serialized_clouds) != last_count:
                    last_count = len(serialized_clouds)
                    last_change = time.monotonic()
                return (
                    last_count >= REVOLUTION_COUNT
                    or time.monotonic() - last_change > IDLE_SECONDS
                )

            spin_until(node, finished, DEADLINE_SECONDS)
            driver_log = processes.log_text()

        received_clouds = [deserialize_message(raw, PointCloud2) for raw in serialized_clouds]
        minimum_frames = REVOLUTION_COUNT - ACCEPTED_FRAME_LOSS
        self.assertGreaterEqual(
            len(serialized_clouds), minimum_frames, f"too few frames:\n{driver_log}"
        )
        self.assertGreaterEqual(len(best_effort_clouds), 1, "best-effort subscription got nothing")
        for cloud in received_clouds:
            self.assert_cloud_layout(cloud)
        self.assert_cloud_points(received_clouds[0])

    def assert_cloud_layout(self, cloud):
        self.assertEqual(cloud.header.frame_id, "velodyne")
        self.assertEqual([field.name for field in cloud.fields], EXPECTED_FIELD_NAMES)
        self.assertEqual(cloud.point_step, POINT_STEP)
        self.assertEqual((cloud.width, cloud.height), (EXPECTED_WIDTH, EXPECTED_HEIGHT))
        self.assertEqual(len(cloud.data), POINT_STEP * EXPECTED_WIDTH * EXPECTED_HEIGHT)
        self.assertFalse(cloud.is_dense)

    def assert_cloud_points(self, cloud):
        points = decode_points(cloud)
        finite_points = [point for point in points if all(math.isfinite(v) for v in point[:4])]
        self.assertEqual(len(finite_points), EXPECTED_WIDTH * EXPECTED_HEIGHT)
        for x, y, z, _intensity, _ring, _time in finite_points:
            self.assertAlmostEqual(
                math.sqrt(x * x + y * y + z * z),
                hdl32e_packets.DEFAULT_DISTANCE_METERS,
                delta=DISTANCE_TOLERANCE_METERS,
            )
        # width runs along ring, so the column index of a point is its ring.
        for index, point in enumerate(points):
            self.assertEqual(point[4], index % EXPECTED_WIDTH)
        self.assertEqual({point[4] for point in points}, set(range(EXPECTED_WIDTH)))

    def test_driver_stays_alive_without_packets(self):
        with launched([DRIVER_COMMAND]) as processes:
            with self.assertRaises(subprocess.TimeoutExpired):
                processes.processes[0].wait(timeout=IDLE_SECONDS)
            self.assertTrue(processes.all_alive(), processes.log_text())


if __name__ == "__main__":
    unittest.main()
