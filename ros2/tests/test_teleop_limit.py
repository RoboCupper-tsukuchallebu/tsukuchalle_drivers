#!/usr/bin/env python3
"""Run inside one isolated image container, with the repository mounted at /test.

Starts the real limited_teleop_node with the repository's config file, publishes a fake /joy and watches
/cmd_vel_teleop. The limits and the initial speeds are read from the config file, never written here.
"""
import copy
import math
import subprocess
import tempfile
import time
import unittest
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import Parameter as ParameterMessage
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from sensor_msgs.msg import Joy

EXECUTABLE = '/ws/install/tsukuchalle_teleop/lib/tsukuchalle_teleop/limited_teleop_node'
CONFIG_PATH = Path('/test/ros2/config/teleop_twist_joy.yaml')
NODE_NAME = '/teleop_twist_joy_node'
OUTPUT_TOLERANCE = 0.001
JOY_PERIOD_SECONDS = 0.05
SETTLE_SECONDS = 0.4
STARTUP_TIMEOUT_SECONDS = 15.0
SERVICE_TIMEOUT_SECONDS = 5.0
EXIT_TIMEOUT_SECONDS = 15.0
UNUSED_SCALE_NAMES = [
    ('scale_linear', 'y'), ('scale_linear', 'z'), ('scale_angular', 'pitch'), ('scale_angular', 'roll'),
    ('scale_linear_turbo', 'y'), ('scale_linear_turbo', 'z'), ('scale_angular_turbo', 'pitch'),
    ('scale_angular_turbo', 'roll')]


def load_parameters():
    return yaml.safe_load(CONFIG_PATH.read_text())['teleop_twist_joy_node']['ros__parameters']


def write_config(parameters):
    handle = tempfile.NamedTemporaryFile('w', suffix='.yaml', delete=False)
    yaml.safe_dump({'teleop_twist_joy_node': {'ros__parameters': parameters}}, handle)
    handle.close()
    return handle.name


def run_to_exit(parameters):
    """Starts the node with these parameters and returns (exit code, log), or (None, log) if it kept running."""
    process = subprocess.Popen([EXECUTABLE, '--ros-args', '--params-file', write_config(parameters)],
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        output, _ = process.communicate(timeout=EXIT_TIMEOUT_SECONDS)
        return process.returncode, output
    except subprocess.TimeoutExpired:
        process.terminate()
        output, _ = process.communicate(timeout=EXIT_TIMEOUT_SECONDS)
        return None, output


def double_value(value):
    return ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))


class TeleopLimitTestCase(unittest.TestCase):
    def setUp(self):
        self.parameters = load_parameters()
        self.linear_limit = self.parameters['scale_linear_limit']['x']
        self.angular_limit = self.parameters['scale_angular_limit']['yaw']
        self.linear_initial = self.parameters['scale_linear']['x']
        self.angular_initial = self.parameters['scale_angular']['yaw']
        self.process = None
        rclpy.init()
        self.node = Node('teleop_limit_test')
        self.joy_publisher = self.node.create_publisher(Joy, '/joy', 10)
        self.received = []
        self.node.create_subscription(Twist, '/cmd_vel_teleop', self.received.append, 10)
        self.set_client = self.node.create_client(SetParameters, NODE_NAME + '/set_parameters')
        self.get_client = self.node.create_client(GetParameters, NODE_NAME + '/get_parameters')

    def tearDown(self):
        self.stop_process()
        self.node.destroy_node()
        rclpy.shutdown()

    def start_process(self, parameters=None):
        config_path = write_config(self.parameters if parameters is None else parameters)
        self.process = subprocess.Popen([EXECUTABLE, '--ros-args', '--params-file', config_path,
                                         '-r', 'cmd_vel:=/cmd_vel_teleop'])
        self.assertTrue(self.set_client.wait_for_service(timeout_sec=STARTUP_TIMEOUT_SECONDS), 'node did not start')
        self.assertTrue(self.get_client.wait_for_service(timeout_sec=STARTUP_TIMEOUT_SECONDS))

    def stop_process(self):
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            self.process.wait(timeout=EXIT_TIMEOUT_SECONDS)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.005)

    def drive(self, seconds, enable=True, forward=1.0, left=0.0):
        """Publishes /joy at 20 Hz and returns the Twist messages that arrived meanwhile."""
        self.received.clear()
        message = Joy(axes=[left, forward], buttons=[1 if enable else 0])
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.joy_publisher.publish(message)
            self.spin_for(JOY_PERIOD_SECONDS)
        self.spin_for(SETTLE_SECONDS)
        return list(self.received)

    def full_stick_outputs(self):
        linear = self.drive(1.0, forward=1.0)
        angular = self.drive(1.0, forward=0.0, left=1.0)
        self.assertTrue(linear and angular, 'no output')
        return linear[-1].linear.x, angular[-1].angular.z

    def set_parameter(self, name, value):
        """Returns (successful, reason). value is a ParameterValue, or a Python bool/int/float."""
        if not isinstance(value, ParameterValue):
            if isinstance(value, bool):
                value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
            elif isinstance(value, int):
                value = ParameterValue(type=ParameterType.PARAMETER_INTEGER, integer_value=value)
            else:
                value = double_value(value)
        request = SetParameters.Request(parameters=[ParameterMessage(name=name, value=value)])
        future = self.set_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=SERVICE_TIMEOUT_SECONDS)
        result = future.result().results[0]
        return result.successful, result.reason

    def get_double(self, name):
        future = self.get_client.call_async(GetParameters.Request(names=[name]))
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=SERVICE_TIMEOUT_SECONDS)
        return future.result().values[0].double_value

    def assert_outputs(self, linear_expected, angular_expected):
        linear, angular = self.full_stick_outputs()
        self.assertAlmostEqual(linear, linear_expected, delta=OUTPUT_TOLERANCE)
        self.assertAlmostEqual(angular, angular_expected, delta=OUTPUT_TOLERANCE)


class RunningNodeTest(TeleopLimitTestCase):
    def setUp(self):
        super().setUp()
        self.start_process()

    def test_t1_full_stick_gives_the_configured_scales(self):
        self.assert_outputs(self.linear_initial, self.angular_initial)

    def test_t2_release_sends_one_zero_and_idle_stick_sends_nothing(self):
        self.assertTrue(self.drive(0.5, enable=True))
        after_release = self.drive(1.0, enable=False, forward=1.0)
        self.assertEqual(len(after_release), 1)
        self.assertEqual((after_release[0].linear.x, after_release[0].angular.z), (0.0, 0.0))
        self.assertTrue(all(m.linear.x == 0.0 and m.angular.z == 0.0 for m in self.drive(1.0, enable=False)))

    def test_t3_half_and_exact_limit_are_accepted(self):
        for name, limit in (('scale_linear.x', self.linear_limit), ('scale_angular.yaw', self.angular_limit)):
            for value in (limit / 2, limit):
                with self.subTest(name=name, value=value):
                    self.assertTrue(self.set_parameter(name, value)[0])
        self.assert_outputs(self.linear_limit, self.angular_limit)
        self.set_parameter('scale_linear.x', self.linear_limit / 2)
        self.set_parameter('scale_angular.yaw', self.angular_limit / 2)
        self.assert_outputs(self.linear_limit / 2, self.angular_limit / 2)

    def test_t4_above_limit_is_refused_and_not_applied(self):
        for name, limit in (('scale_linear.x', self.linear_limit), ('scale_angular.yaw', self.angular_limit)):
            with self.subTest(name=name):
                successful, reason = self.set_parameter(name, limit + 0.1)
                self.assertFalse(successful)
                self.assertIn(name, reason)
        self.assert_outputs(self.linear_initial, self.angular_initial)

    def test_t5_zero_negative_nonfinite_and_integer_are_refused(self):
        bad_values = [0.0, -0.1, math.nan, math.inf, 1]
        for name in ('scale_linear.x', 'scale_angular.yaw'):
            for value in bad_values:
                with self.subTest(name=name, value=value):
                    self.assertFalse(self.set_parameter(name, value)[0])
        self.assert_outputs(self.linear_initial, self.angular_initial)

    def test_t6_enable_button_cannot_be_turned_off(self):
        self.assertFalse(self.set_parameter('require_enable_button', False)[0])
        self.assertTrue(all(m.linear.x == 0.0 for m in self.drive(1.0, enable=False)))

    def test_t7_other_parameters_are_refused(self):
        attempts = [('enable_turbo_button', 3), ('enable_button', 1), ('axis_linear.x', 2),
                    ('scale_linear_turbo.x', 1.0), ('scale_linear.y', 0.1)]
        for name, value in attempts:
            with self.subTest(name=name):
                self.assertFalse(self.set_parameter(name, value)[0])

    def test_t8_limits_are_readable_and_read_only(self):
        self.assertAlmostEqual(self.get_double('scale_linear_limit.x'), self.linear_limit)
        self.assertAlmostEqual(self.get_double('scale_angular_limit.yaw'), self.angular_limit)
        self.assertFalse(self.set_parameter('scale_linear_limit.x', self.linear_limit / 2)[0])
        self.assertFalse(self.set_parameter('scale_angular_limit.yaw', self.angular_limit / 2)[0])
        self.assertFalse(self.set_parameter('scale_linear_limit.x', self.linear_limit * 2)[0])

    def test_t9_restart_returns_to_the_config_value(self):
        self.assertTrue(self.set_parameter('scale_linear.x', self.linear_limit / 2)[0])
        self.stop_process()
        self.start_process()
        self.assert_outputs(self.linear_initial, self.angular_initial)


class StartupCheckTest(unittest.TestCase):
    def setUp(self):
        self.parameters = load_parameters()
        self.linear_limit = self.parameters['scale_linear_limit']['x']
        self.angular_limit = self.parameters['scale_angular_limit']['yaw']

    def assert_refuses(self, mutate, expected_in_log):
        parameters = copy.deepcopy(self.parameters)
        mutate(parameters)
        exit_code, log = run_to_exit(parameters)
        self.assertIsNotNone(exit_code, 'node kept running')
        self.assertNotEqual(exit_code, 0, log)
        self.assertIn(expected_in_log, log)

    def test_t10_1_limit_missing_nonfinite_or_not_positive(self):
        for key, name in (('scale_linear_limit', 'x'), ('scale_angular_limit', 'yaw')):
            expected = '%s.%s must be a finite double greater than 0' % (key, name)
            for label, value in (('missing', None), ('nan', math.nan), ('inf', math.inf), ('zero', 0.0),
                                 ('negative', -0.5)):
                with self.subTest(key=key, value=label):
                    def mutate(parameters, key=key, name=name, value=value):
                        if value is None:
                            del parameters[key][name]
                        else:
                            parameters[key][name] = value
                    self.assert_refuses(mutate, expected)

    def test_t10_2_initial_scale_outside_range(self):
        for key, name, limit in (('scale_linear', 'x', self.linear_limit),
                                 ('scale_angular', 'yaw', self.angular_limit)):
            for label, value in (('above', limit + 0.1), ('zero', 0.0), ('negative', -0.1)):
                with self.subTest(key=key, value=label):
                    def mutate(parameters, key=key, name=name, value=value):
                        parameters[key][name] = value
                    self.assert_refuses(mutate, '%s.%s must be in (0, ' % (key, name))

    def test_t10_3_enable_button_not_required(self):
        self.assert_refuses(lambda parameters: parameters.update(require_enable_button=False),
                            'require_enable_button must be true')

    def test_t10_4_turbo_button_assigned(self):
        for button in (0, 5):
            with self.subTest(button=button):
                self.assert_refuses(lambda parameters, button=button: parameters.update(enable_turbo_button=button),
                                    'enable_turbo_button must be negative')

    def test_t10_5_unused_component_scale_not_zero(self):
        for key, name in UNUSED_SCALE_NAMES:
            with self.subTest(parameter='%s.%s' % (key, name)):
                def mutate(parameters, key=key, name=name):
                    parameters.setdefault(key, {})[name] = 0.1
                self.assert_refuses(mutate, '%s.%s must be 0.0' % (key, name))

    def test_t10_unchanged_config_file_starts(self):
        exit_code, log = run_to_exit(self.parameters)
        self.assertIsNone(exit_code, log)


if __name__ == '__main__':
    unittest.main()
