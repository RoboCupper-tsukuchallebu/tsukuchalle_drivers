"""Feeds synthetic HDL-32E packets to velodyne_driver_node and velodyne_pointcloud_node.

Both nodes run in their own containers (tsukuchalle-velodyne image, started by CI with the
`command` of ros2/compose.yaml; only device_ip differs: 127.0.0.1, so the sender filter is exercised
too). This test runs in the tools image, which has rclpy and the velodyne_msgs Python types:
    docker run --rm --network host --ipc host -v "$PWD/ros2/velodyne:/test:ro" \
        tsukuchalle-ros-tools:ci python3 -m unittest discover -s /test -v
Use the same ROS_DOMAIN_ID (-e) for all three containers.
"""

import itertools
import math
import socket
import struct
import time
import unittest

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2
from velodyne_msgs.msg import VelodyneScan

import hdl32e_packets

RELIABLE_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
SENSOR_ADDRESS = ("127.0.0.1", 2368)

REVOLUTION_COUNT = 20
ACCEPTED_FRAME_LOSS = 3
PACKETS_PER_BATCH = 18
BATCH_INTERVAL_SECONDS = 0.01
DEADLINE_SECONDS = 30.0
IDLE_SECONDS = 5.0
DRIVER_NODE_NAME = "velodyne_driver_node"

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


def decode_points(cloud):
    return list(struct.iter_unpack(POINT_FORMAT, bytes(cloud.data)))


class VelodynePipelineTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def node_names(self, node):
        return [name for name, _namespace in node.get_node_names_and_namespaces()]

    # unittest runs tests in alphabetical order, so this one runs before any packet is sent.
    def test_a_driver_stays_alive_without_packets(self):
        node = rclpy.create_node("velodyne_idle_test")
        self.addCleanup(node.destroy_node)
        discovered = spin_until(
            node, lambda: DRIVER_NODE_NAME in self.node_names(node), DEADLINE_SECONDS
        )
        self.assertTrue(discovered, f"{DRIVER_NODE_NAME} was not discovered in time")

        # Nothing is sent: the driver only logs `poll() timeout` and keeps running. CI also
        # checks `docker inspect` of the driver container after the whole run.
        spin_until(node, lambda: False, IDLE_SECONDS)
        self.assertIn(DRIVER_NODE_NAME, self.node_names(node))

    def test_synthetic_revolutions_become_organized_point_clouds(self):
        node = rclpy.create_node("velodyne_pipeline_test")
        self.addCleanup(node.destroy_node)
        serialized_clouds = []
        # The callbacks only store the serialized bytes (raw=True): converting a 1.5 MB message
        # to Python objects, let alone walking 69,504 points, takes longer than the 100 ms
        # between frames.
        subscription = node.create_subscription(
            PointCloud2, "/velodyne_points", serialized_clouds.append, RELIABLE_QOS, raw=True
        )
        # nav-stack subscribes with SensorDataQoS, so a match of a best-effort subscription with
        # the reliable publisher is the compatibility check. It is destroyed before sending:
        # its frames are not counted, and it would compete with the reliable one for the
        # publisher's shared memory segment. Best-effort frames do get lost: with
        # ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST the rmw adds a default Fast DDS shared memory
        # transport whose 512 KiB segment overflows when one 1.5 MB sample (about 24 fragments
        # of 64 KB) is sent, and best-effort does not retransmit. Tracked in a separate issue.
        best_effort_subscription = node.create_subscription(
            PointCloud2, "/velodyne_points", lambda _: None, qos_profile_sensor_data, raw=True
        )

        # driver publishes /velodyne_packets, the converter subscribes to it and publishes /velodyne_points.
        ready = spin_until(
            node,
            lambda: node.count_publishers("/velodyne_packets") >= 1
            and node.count_subscribers("/velodyne_packets") >= 1
            and subscription.get_publisher_count() >= 1
            and best_effort_subscription.get_publisher_count() >= 1,
            DEADLINE_SECONDS,
        )
        self.assertTrue(ready, "driver and converter were not discovered in time")
        node.destroy_subscription(best_effort_subscription)

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

        # Stop at the full count or the deadline; frames can arrive late on a slow runner.
        spin_until(node, lambda: len(serialized_clouds) >= REVOLUTION_COUNT, DEADLINE_SECONDS)

        received_clouds = [deserialize_message(raw, PointCloud2) for raw in serialized_clouds]
        minimum_frames = REVOLUTION_COUNT - ACCEPTED_FRAME_LOSS
        self.assertGreaterEqual(len(serialized_clouds), minimum_frames, "too few frames")
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


if __name__ == "__main__":
    unittest.main()
