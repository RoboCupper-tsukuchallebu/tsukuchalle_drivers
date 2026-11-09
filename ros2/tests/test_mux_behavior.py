#!/usr/bin/env python3
"""Run inside one isolated image container, with the repository mounted at /test."""
import subprocess
import time
import unittest

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class MuxBehaviorTest(unittest.TestCase):
    def test_priority_and_resume(self):
        proc = subprocess.Popen(['/opt/ros/jazzy/lib/twist_mux/twist_mux', '--ros-args',
                                 '--params-file', '/test/ros2/config/twist_mux.yaml',
                                 '-r', 'cmd_vel_out:=/cmd_vel_mux'])
        rclpy.init()
        node = Node('mux_behavior_test')
        nav = node.create_publisher(Twist, '/cmd_vel', 10)
        teleop = node.create_publisher(Twist, '/cmd_vel_teleop', 10)
        received = []
        node.create_subscription(Twist, '/cmd_vel_mux', lambda m: received.append((time.monotonic(), m.linear.x)), 10)

        def publish_for(seconds, manual=None):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                msg = Twist()
                msg.linear.x = 0.2
                nav.publish(msg)
                if manual is not None:
                    msg = Twist()
                    msg.linear.x = manual
                    teleop.publish(msg)
                until = time.monotonic() + 0.05
                while time.monotonic() < until:
                    rclpy.spin_once(node, timeout_sec=0.005)

        try:
            publish_for(2.0)
            self.assertTrue(any(abs(v - 0.2) < 0.001 for _, v in received), 'nav output missing')
            for value in (0.7, 0.0):
                publish_for(0.8, value)
                received.clear()
                publish_for(0.6, value)
                self.assertTrue(received, 'teleop output missing')
                self.assertTrue(all(abs(v - value) < 0.001 for _, v in received), received)
            received.clear()
            start = time.monotonic()
            publish_for(0.35)
            self.assertFalse(any(abs(v - 0.2) < 0.001 for _, v in received), 'nav resumed before timeout')
            publish_for(0.8)
            resumes = [t for t, v in received if abs(v - 0.2) < 0.001]
            self.assertTrue(resumes, 'nav did not resume')
            self.assertGreater(resumes[0] - start, 0.35)
        finally:
            node.destroy_node()
            rclpy.shutdown()
            proc.terminate()
            proc.wait(timeout=5)


if __name__ == '__main__':
    unittest.main()
