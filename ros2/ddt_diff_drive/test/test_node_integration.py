"""Runs the real node against a pseudo-terminal that plays the firmware's role."""

import math
import os
import pty
import shutil
import struct
import tempfile
import threading
import time
import tty
import unittest
from unittest import mock

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool
from tf2_msgs.msg import TFMessage

from ddt_diff_drive.ddt_bridge import (
    STATE_ACTIVE,
    STATE_DISABLED,
    TYPE_DISABLE,
    TYPE_ENABLE,
    TYPE_FEEDBACK,
    TYPE_SET_VELOCITY,
    TYPE_STATUS,
    decode_frames,
    encode_frame,
)
from ddt_diff_drive import diff_drive_node
from ddt_diff_drive.diff_drive_node import JOY_STALE_S, STATE_WAIT_S, DiffDriveNode


STATE_ARMING = 1
STATE_STOPPING = 3
STATUS_PERIOD_S = 0.1
ARMING_DELAY_S = 0.15
FEEDBACK_PERIOD_S = 0.01  # Per motor, as the firmware reports.
STOPPING_DELAY_S = 0.2
JOY_PERIOD_S = 0.05  # joy_node's autorepeat_rate of 20 Hz.
START_BUTTON = 7
BACK_BUTTON = 6
DEADMAN_BUTTON = 0
STICK_Y_AXIS = 1
BUTTON_COUNT = 11
AXIS_COUNT = 8


class FakeFirmware:
    """Mimics the firmware's delayed transitions and its 10 Hz STATUS from its own thread.

    A thread is required because the node's enable service blocks until the state changes.
    The node opens a symlink, like /dev/serial/by-id: disconnect() removes it and closes the
    pty, reconnect() makes a new pty (a rebooted board, DISABLED) and points the symlink at it.
    """

    def __init__(self):
        self._link_directory = tempfile.mkdtemp()
        self.device_path = os.path.join(self._link_directory, "by-id-board")
        self._lock = threading.RLock()
        self.state = STATE_DISABLED
        self.stop_reason = 0
        self.velocities = []
        self.rejected_velocity_count = 0
        self.enable_count = 0
        self.disable_count = 0
        self.ignored_enable = False  # Simulates an ENABLE that never reaches ACTIVE.
        self.frames_to_drop = 0  # Simulates DISABLE frames that are lost.
        self.is_status_sent = True  # False simulates a board that opens but stays silent.
        self._buffer = bytearray()
        self._pending = None
        self._last_status_time = 0.0
        self._feedbacks = {}  # motor id -> (rpm, current_raw, fault, mode)
        self._last_feedback_times = {}
        self._open_pty()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _open_pty(self) -> None:
        self.controller_fd, self._device_fd = pty.openpty()
        tty.setraw(self.controller_fd)
        os.set_blocking(self.controller_fd, False)
        temporary_link = self.device_path + ".new"
        os.symlink(os.ttyname(self._device_fd), temporary_link)
        os.replace(temporary_link, self.device_path)

    def disconnect(self) -> None:
        with self._lock:
            if self.controller_fd is None:
                return
            os.unlink(self.device_path)
            self._feedbacks.clear()  # A board that is gone reports nothing; start_feedback again.
            os.close(self.controller_fd)
            os.close(self._device_fd)
            self.controller_fd = None
            self._pending = None

    def reconnect(self) -> None:
        with self._lock:
            self.state = STATE_DISABLED
            self.stop_reason = 0
            self._buffer.clear()
            self._open_pty()

    def _run(self) -> None:
        while not self._stop.wait(0.001):
            with self._lock:
                if self.controller_fd is not None:
                    self._run_once()

    def _run_once(self) -> None:
        self._service()
        now = time.monotonic()
        if self._pending and now >= self._pending[0]:
            self.state = self._pending[1]
            self._pending = None
        self._send_feedbacks(now)
        if self.is_status_sent and now - self._last_status_time >= STATUS_PERIOD_S:
            self._last_status_time = now
            os.write(self.controller_fd, encode_frame(TYPE_STATUS, bytes([self.state, self.stop_reason])))

    def start_feedback(self, motor_id: int, rpm: int, current_raw: int = 0, fault: int = 0,
                       mode: int = 0x02) -> None:
        with self._lock:
            self._feedbacks[motor_id] = (rpm, current_raw, fault, mode)

    def stop_feedback(self, motor_id: int) -> None:
        with self._lock:
            self._feedbacks.pop(motor_id, None)

    def _send_feedbacks(self, now: float) -> None:
        for motor_id, (rpm, current_raw, fault, mode) in list(self._feedbacks.items()):
            if now - self._last_feedback_times.get(motor_id, 0.0) < FEEDBACK_PERIOD_S:
                continue
            self._last_feedback_times[motor_id] = now
            payload = struct.pack("<BhhHBB", motor_id, rpm, current_raw, 0, fault, mode)
            try:
                os.write(self.controller_fd, encode_frame(TYPE_FEEDBACK, payload))
            except BlockingIOError:
                pass  # The node is not reading right now; a real board would drop it as well.

    def set_state(self, state: int, stop_reason: int = 0) -> None:
        self.state = state
        self.stop_reason = stop_reason
        os.write(self.controller_fd, encode_frame(TYPE_STATUS, bytes([state, stop_reason])))

    def _service(self) -> None:
        try:
            self._buffer += os.read(self.controller_fd, 4096)
        except BlockingIOError:
            return
        for message_type, payload in decode_frames(self._buffer):
            # Transitions follow firmware/src/bridge_controller.cpp: ENABLE is accepted only in
            # DISABLED; DISABLE in ARMING or ACTIVE goes through STOPPING to DISABLED.
            if message_type == TYPE_ENABLE:
                self.enable_count += 1
                self.stop_reason = 0
                if self.state == STATE_DISABLED and not self.ignored_enable:
                    self.state = STATE_ARMING
                    self._pending = (time.monotonic() + ARMING_DELAY_S, STATE_ACTIVE)
            elif message_type == TYPE_DISABLE:
                self.disable_count += 1
                if self.frames_to_drop > 0:
                    self.frames_to_drop -= 1
                elif self.state in (STATE_ARMING, STATE_ACTIVE):
                    self.state = STATE_STOPPING
                    self._pending = (time.monotonic() + STOPPING_DELAY_S, STATE_DISABLED)
            elif message_type == TYPE_SET_VELOCITY and self.state == STATE_ACTIVE:
                self.velocities.append(struct.unpack("<hh", payload))
            elif message_type == TYPE_SET_VELOCITY:
                self.rejected_velocity_count += 1

    def close(self) -> None:
        self._stop.set()
        self._thread.join()
        self.disconnect()
        shutil.rmtree(self._link_directory)


class NodeTestBase(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.try_shutdown()

    def setUp(self):
        self.firmware = FakeFirmware()
        self.node = DiffDriveNode([Parameter("port", value=self.firmware.device_path)])
        self.client_node = rclpy.create_node("test_client")
        self.publisher = self.client_node.create_publisher(Twist, "cmd_vel", 10)
        self.joy_publisher = self.client_node.create_publisher(Joy, "joy", 10)
        self.enable_client = self.client_node.create_client(SetBool, "diff_drive_node/enable")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.executor.add_node(self.client_node)
        self.addCleanup(self._tear_down)
        deadline = time.monotonic() + 5.0
        while (self.publisher.get_subscription_count() == 0
               or self.joy_publisher.get_subscription_count() == 0) and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.01)
        self.assertEqual(self.publisher.get_subscription_count(), 1)
        self.assertEqual(self.joy_publisher.get_subscription_count(), 1)

    def _tear_down(self):
        self.executor.shutdown()
        self.client_node.destroy_node()
        self.node.close()
        self.node.destroy_node()
        self.firmware.close()

    def spin_for(self, seconds: float, publish: Twist | None = None,
                 joy: Joy | None = None) -> None:
        deadline = time.monotonic() + seconds
        next_joy_time = 0.0
        while time.monotonic() < deadline:
            if publish is not None:
                self.publisher.publish(publish)
            if joy is not None and time.monotonic() >= next_joy_time:
                self.joy_publisher.publish(joy)
                next_joy_time = time.monotonic() + JOY_PERIOD_S
            self.executor.spin_once(timeout_sec=0.005)

    def press(self, seconds: float, buttons=(), axes=None, publish: Twist | None = None) -> None:
        self.spin_for(seconds, publish, make_joy(buttons, axes))

    def release_all(self, seconds: float = 0.1) -> None:
        self.press(seconds)

    def enable_by_pad(self) -> None:
        self.release_all()
        self.press(0.1, [START_BUTTON])
        self.release_all(0.4)
        self.assertEqual(self.firmware.state, STATE_ACTIVE)

    def call_enable(self, is_enabled: bool) -> SetBool.Response:
        self.assertTrue(self.enable_client.wait_for_service(timeout_sec=5.0))
        future = self.enable_client.call_async(SetBool.Request(data=is_enabled))
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.005)
        self.assertTrue(future.done())
        return future.result()


    def set_parameter(self, name: str, value) -> None:
        self.assertTrue(self.node.set_parameters([Parameter(name, value=value)])[0].successful)


class DiffDriveNodeTest(NodeTestBase):
    def test_stays_silent_until_enabled(self):
        self.spin_for(0.3, forward_twist())
        self.assertEqual(self.firmware.velocities, [])

    def test_ramps_with_right_sign_inverted_after_enable(self):
        self.spin_for(0.15)
        self.assertTrue(self.call_enable(True).success)
        self.spin_for(0.5, forward_twist())
        first, second = self.firmware.velocities[-1]
        self.assertGreater(first, 10)
        self.assertLess(first, 100)
        self.assertEqual(second, -first)

    def test_stale_command_ramps_back_to_zero(self):
        self.assertTrue(self.call_enable(True).success)
        self.spin_for(0.4, forward_twist())
        self.assertGreater(self.firmware.velocities[-1][0], 0)
        self.spin_for(1.6)
        self.assertEqual(self.firmware.velocities[-1], (0, 0))

    def test_firmware_stop_blocks_commands_and_restart_begins_at_zero(self):
        self.assertTrue(self.call_enable(True).success)
        self.spin_for(0.4, forward_twist())
        self.firmware.set_state(STATE_DISABLED, 2)
        self.spin_for(0.05, forward_twist())
        self.firmware.velocities.clear()
        self.spin_for(0.1, forward_twist())
        self.assertEqual(self.firmware.velocities, [])
        self.assertTrue(self.call_enable(True).success)
        self.spin_for(0.03, forward_twist())
        self.assertLessEqual(abs(self.firmware.velocities[0][0]), 4)

    def test_repeated_enable_keeps_speed(self):
        self.assertTrue(self.call_enable(True).success)
        self.spin_for(0.4, forward_twist())
        speed_before = self.firmware.velocities[-1][0]
        self.assertTrue(self.call_enable(True).success)
        self.spin_for(0.03, forward_twist())
        self.assertGreaterEqual(self.firmware.velocities[-1][0], speed_before)
        self.assertEqual(self.firmware.enable_count, 1)

    def test_command_received_while_disabled_does_not_start_motors(self):
        self.spin_for(0.15, forward_twist())
        self.assertTrue(self.call_enable(True).success)
        self.spin_for(0.2)
        self.assertTrue(self.firmware.velocities)
        self.assertEqual(self.firmware.velocities[-1], (0, 0))

    def test_first_step_after_enable_respects_acceleration_limit(self):
        self.spin_for(0.15)
        self.assertTrue(self.call_enable(True).success)
        self.spin_for(0.1, forward_twist())
        steps = [abs(after[0] - before[0]) for before, after
                 in zip(self.firmware.velocities, self.firmware.velocities[1:])]
        self.assertLessEqual(abs(self.firmware.velocities[0][0]), 4)
        self.assertLessEqual(max(steps), 5)

    def test_gentle_acceleration_still_stops_quickly_on_command_loss(self):
        self.assertTrue(self.call_enable(True).success)
        self.spin_for(0.4, forward_twist())
        self.assertTrue(self.node.set_parameters(
            [Parameter("max_acceleration_rpm_per_s", value=5.0)])[0].successful)
        self.spin_for(1.0)
        self.assertEqual(self.firmware.velocities[-1], (0, 0))

    def test_rejects_unbounded_command_timeout(self):
        for seconds in (float("inf"), float("nan"), 0.0, 5.1):
            with self.subTest(seconds):
                results = self.node.set_parameters(
                    [Parameter("command_timeout_s", value=seconds)])
                self.assertFalse(results[0].successful)

    def test_rejects_wheel_assignment_change_while_enabled(self):
        self.assertTrue(self.call_enable(True).success)
        results = self.node.set_parameters([Parameter("left_sign", value=-1)])
        self.assertFalse(results[0].successful)

    def test_runtime_limit_applies(self):
        results = self.node.set_parameters([Parameter("max_wheel_rpm", value=5.0)])
        self.assertTrue(results[0].successful)
        self.assertTrue(self.call_enable(True).success)
        self.spin_for(0.5, forward_twist())
        self.assertEqual(self.firmware.velocities[-1], (5, -5))

    def test_rejects_unsafe_limit(self):
        results = self.node.set_parameters([Parameter("max_wheel_rpm", value=500.0)])
        self.assertFalse(results[0].successful)

    def test_disable_service_reports_disabled(self):
        self.assertTrue(self.call_enable(True).success)
        self.assertTrue(self.call_enable(False).success)
        self.assertEqual(self.firmware.state, STATE_DISABLED)

    def test_idle_joy_does_not_enable(self):
        self.release_all(0.3)
        self.assertEqual(self.firmware.enable_count, 0)

    def test_start_button_enables_once_per_press(self):
        self.release_all()
        self.press(0.1, [START_BUTTON])
        self.press(0.4, [START_BUTTON])
        self.assertEqual(self.firmware.state, STATE_ACTIVE)
        self.assertEqual(self.firmware.enable_count, 1)

    def test_start_held_in_first_joy_message_does_not_enable(self):
        self.press(0.3, [START_BUTTON])
        self.assertEqual(self.firmware.enable_count, 0)
        self.release_all()
        self.press(0.4, [START_BUTTON])
        self.assertEqual(self.firmware.state, STATE_ACTIVE)

    def test_back_button_disables(self):
        self.enable_by_pad()
        self.press(0.1, [BACK_BUTTON])
        self.release_all(0.4)
        self.assertEqual(self.firmware.state, STATE_DISABLED)

    def test_back_sends_no_velocity_to_a_stopping_firmware(self):
        self.enable_by_pad()
        self.firmware.rejected_velocity_count = 0
        self.press(0.1, [BACK_BUTTON])
        self.release_all(0.4)
        self.assertEqual(self.firmware.state, STATE_DISABLED)
        # SET_VELOCITY outside ACTIVE makes the firmware answer "invalid state".
        self.assertEqual(self.firmware.rejected_velocity_count, 0)

    def test_deadman_held_blocks_enable(self):
        self.release_all()
        self.press(0.3, [DEADMAN_BUTTON, START_BUTTON])
        self.assertEqual(self.firmware.enable_count, 0)

    def test_stick_away_from_center_blocks_enable_until_a_new_press(self):
        self.release_all()
        self.press(0.3, [START_BUTTON], {STICK_Y_AXIS: 0.5})
        self.assertEqual(self.firmware.enable_count, 0)
        self.press(0.3, [START_BUTTON])  # Centered now, but Start is still the same press.
        self.assertEqual(self.firmware.enable_count, 0)
        self.release_all()
        self.press(0.4, [START_BUTTON])
        self.assertEqual(self.firmware.state, STATE_ACTIVE)

    def test_start_and_back_together_do_not_enable(self):
        self.release_all()
        self.press(0.3, [BACK_BUTTON, START_BUTTON])
        self.assertEqual(self.firmware.enable_count, 0)

    def test_pad_enable_can_be_turned_off_but_disable_stays(self):
        self.assertTrue(self.node.set_parameters(
            [Parameter("joy_enable_button", value=-1)])[0].successful)
        self.release_all()
        self.press(0.3, [START_BUTTON])
        self.assertEqual(self.firmware.enable_count, 0)
        self.assertTrue(self.call_enable(True).success)
        self.press(0.1, [BACK_BUTTON])
        self.release_all(0.4)
        self.assertEqual(self.firmware.state, STATE_DISABLED)

    def test_joy_with_too_few_buttons_is_ignored(self):
        self.release_all()
        self.spin_for(0.3, joy=Joy(buttons=[1, 1, 1]))
        self.assertEqual(self.firmware.enable_count, 0)

    def test_first_velocity_after_pad_enable_is_small(self):
        self.release_all()
        self.press(0.1, [START_BUTTON])
        self.press(0.5, publish=forward_twist())
        self.assertLessEqual(abs(self.firmware.velocities[0][0]), 4)

    def test_back_during_arming_wins(self):
        self.release_all()
        self.press(0.04, [START_BUTTON])
        self.press(0.1, [BACK_BUTTON])
        self.release_all(0.6)
        self.assertEqual(self.firmware.enable_count, 1)
        self.assertEqual(self.firmware.state, STATE_DISABLED)
        # A blocking enable would let the firmware reach ACTIVE before Back is handled.
        self.assertEqual(self.firmware.velocities, [])
        self.spin_for(2.0, forward_twist())
        self.assertEqual(self.firmware.velocities, [])

    def test_lost_disable_during_arming_is_resent(self):
        self.release_all()
        self.press(0.04, [START_BUTTON])
        self.firmware.frames_to_drop = 1
        self.press(0.1, [BACK_BUTTON])
        self.release_all(0.1)
        self.spin_for(2.0, forward_twist())
        self.assertEqual(self.firmware.state, STATE_DISABLED)
        self.assertGreater(self.firmware.disable_count, 1)
        self.assertTrue(all(velocity == (0, 0) for velocity in self.firmware.velocities))

    def test_commands_stay_dropped_while_disable_is_unconfirmed(self):
        self.enable_by_pad()
        self.firmware.frames_to_drop = 1000
        self.press(0.1, [BACK_BUTTON])
        self.firmware.velocities.clear()
        self.spin_for(2.0, forward_twist())
        self.assertEqual(self.firmware.state, STATE_ACTIVE)
        self.assertTrue(all(velocity == (0, 0) for velocity in self.firmware.velocities))

    def test_start_is_ignored_after_a_silence_on_joy(self):
        self.release_all()
        self.spin_for(JOY_STALE_S + 0.2)  # joy_node restarted; Start is held when it returns.
        self.press(0.3, [START_BUTTON])
        self.assertEqual(self.firmware.enable_count, 0)
        self.release_all()
        self.press(0.4, [START_BUTTON])
        self.assertEqual(self.firmware.state, STATE_ACTIVE)

    def test_back_still_works_on_the_first_message_after_a_silence(self):
        self.enable_by_pad()
        self.spin_for(JOY_STALE_S + 0.2)
        self.press(0.1, [BACK_BUTTON])
        self.release_all(0.4)
        self.assertEqual(self.firmware.state, STATE_DISABLED)

    def test_enable_that_never_reaches_active_can_be_retried(self):
        self.firmware.ignored_enable = True
        self.release_all()
        self.press(0.1, [START_BUTTON])
        self.release_all(STATE_WAIT_S + 0.5)
        self.firmware.ignored_enable = False
        self.press(0.1, [START_BUTTON])
        self.release_all(0.4)
        self.assertEqual(self.firmware.state, STATE_ACTIVE)
        self.assertEqual(self.firmware.enable_count, 2)

    def test_start_while_active_does_not_send_enable_again(self):
        self.enable_by_pad()
        self.press(0.1, [START_BUTTON])
        self.release_all()
        self.assertEqual(self.firmware.enable_count, 1)

    def test_rejects_unusable_joy_parameters(self):
        invalid_changes = (
            {"joy_disable_button": -1},
            {"joy_enable_button": DEADMAN_BUTTON},
            {"joy_enable_button": BACK_BUTTON},
            {"joy_disable_button": DEADMAN_BUTTON},
            {"joy_neutral_axes": [-1]},
            {"joy_enable_button": -2},
        )
        for change in invalid_changes:
            with self.subTest(change):
                results = self.node.set_parameters(
                    [Parameter(name, value=value) for name, value in change.items()])
                self.assertFalse(results[0].successful)
        self.assertTrue(self.node.set_parameters(
            [Parameter("joy_enable_button", value=-1)])[0].successful)

    # --- USB disconnect and automatic recovery ---

    def unplug_and_replug(self, outage_s: float = 0.3, buttons=(), axes=None) -> None:
        self.firmware.disconnect()
        self.press(outage_s, buttons, axes)
        self.firmware.reconnect()

    def spin_until_waiting_for_neutral(self, buttons=(), axes=None) -> None:
        # Reopening takes 0.5 to 1 s after the loss; a fixed sleep would race with it.
        deadline = time.monotonic() + 3.0
        while self.node._connection != "WAITING_NEUTRAL" and time.monotonic() < deadline:
            self.press(0.02, buttons, axes)
        self.assertEqual(self.node._connection, "WAITING_NEUTRAL")

    def assert_reenabled_count(self, expected: int) -> None:
        self.assertEqual(self.firmware.enable_count, expected)
        self.assertEqual(self.firmware.state, STATE_ACTIVE if expected > 1 else STATE_DISABLED)

    def test_disconnect_does_not_end_the_node(self):
        self.enable_by_pad()
        self.firmware.disconnect()
        self.spin_for(0.3, forward_twist())
        self.assertTrue(rclpy.ok())
        self.assertFalse(self.call_enable(True).success)

    def test_commands_and_joy_while_the_board_is_missing_do_not_crash(self):
        self.release_all()
        self.firmware.disconnect()
        self.press(1.2, [START_BUTTON], publish=forward_twist())
        self.release_all(0.3)
        self.press(0.1, [START_BUTTON], publish=forward_twist())
        self.assertTrue(rclpy.ok())
        self.assertEqual(self.firmware.enable_count, 0)

    def test_replug_reenables_when_the_pad_is_idle(self):
        self.enable_by_pad()
        self.firmware.velocities.clear()
        self.unplug_and_replug()
        self.press(2.0, publish=forward_twist())
        self.assertEqual(self.firmware.enable_count, 2)
        self.assertEqual(self.firmware.state, STATE_ACTIVE)
        self.assertLessEqual(abs(self.firmware.velocities[0][0]), 4)

    def test_reenable_waits_for_the_deadman_to_be_released(self):
        self.enable_by_pad()
        self.unplug_and_replug(buttons=[DEADMAN_BUTTON])
        self.press(1.0, [DEADMAN_BUTTON])
        self.assertEqual(self.firmware.enable_count, 1)
        self.release_all(1.0)
        self.assert_reenabled_count(2)

    def test_reenable_waits_for_the_stick_to_return_to_center(self):
        self.enable_by_pad()
        self.unplug_and_replug(axes={STICK_Y_AXIS: 0.5})
        self.press(1.0, axes={STICK_Y_AXIS: 0.5})
        self.assertEqual(self.firmware.enable_count, 1)
        self.release_all(1.0)
        self.assert_reenabled_count(2)

    def test_reenable_is_abandoned_after_the_wait(self):
        self.set_parameter("auto_reenable_wait_s", 1.0)
        self.enable_by_pad()
        self.unplug_and_replug(buttons=[DEADMAN_BUTTON])
        self.press(1.5, [DEADMAN_BUTTON])
        self.assert_reenabled_count(1)
        self.release_all(0.5)
        self.assert_reenabled_count(1)
        self.press(0.1, [START_BUTTON])
        self.release_all(0.4)
        self.assertEqual(self.firmware.state, STATE_ACTIVE)

    def test_no_joy_at_all_is_not_neutral(self):
        self.set_parameter("auto_reenable_wait_s", 1.0)
        self.assertTrue(self.call_enable(True).success)
        self.firmware.disconnect()
        self.spin_for(0.3)
        self.firmware.reconnect()
        self.spin_for(2.0)
        self.assert_reenabled_count(1)

    def test_without_pad_enable_the_reenable_needs_no_joy(self):
        self.set_parameter("joy_enable_button", -1)
        self.assertTrue(self.call_enable(True).success)
        self.firmware.disconnect()
        self.spin_for(0.3)
        self.firmware.reconnect()
        self.spin_for(2.0)
        self.assert_reenabled_count(2)

    def test_without_pad_enable_a_fresh_joy_still_has_to_be_neutral(self):
        self.set_parameter("joy_enable_button", -1)
        self.assertTrue(self.call_enable(True).success)
        self.unplug_and_replug(buttons=[DEADMAN_BUTTON])
        self.press(1.0, [DEADMAN_BUTTON])
        self.assertEqual(self.firmware.enable_count, 1)
        self.release_all(1.0)
        self.assert_reenabled_count(2)

    def test_service_enable_while_waiting_for_neutral_ends_the_automatic_path(self):
        self.enable_by_pad()
        self.unplug_and_replug(buttons=[DEADMAN_BUTTON])
        self.spin_until_waiting_for_neutral([DEADMAN_BUTTON])
        self.assertTrue(self.call_enable(True).success)
        self.assertEqual(self.firmware.state, STATE_ACTIVE)
        self.press(0.5, [DEADMAN_BUTTON])
        self.assert_reenabled_count(2)
        # A path still waiting for neutral would send ENABLE once the deadman is released.
        self.release_all(1.0)
        self.assert_reenabled_count(2)

    def test_start_while_waiting_for_neutral_ends_the_automatic_path(self):
        self.set_parameter("auto_reenable_wait_s", 1.0)
        self.enable_by_pad()
        self.unplug_and_replug(axes={STICK_Y_AXIS: 0.5})
        self.spin_until_waiting_for_neutral(axes={STICK_Y_AXIS: 0.5})
        self.press(0.1, axes={STICK_Y_AXIS: 0.5})
        self.press(0.4, [START_BUTTON])
        self.assertEqual(self.firmware.state, STATE_ACTIVE)
        self.release_all(1.5)
        self.assert_reenabled_count(2)

    def test_back_while_unplugged_cancels_the_reenable(self):
        self.enable_by_pad()
        self.firmware.disconnect()
        self.press(0.1, [BACK_BUTTON])
        self.release_all(0.2)
        self.firmware.reconnect()
        self.release_all(1.5)
        self.assert_reenabled_count(1)

    def test_disable_service_while_unplugged_succeeds_and_cancels_the_reenable(self):
        self.enable_by_pad()
        self.firmware.disconnect()
        self.spin_for(0.2)
        self.assertTrue(self.call_enable(False).success)
        self.firmware.reconnect()
        self.release_all(1.5)
        self.assert_reenabled_count(1)

    def test_enable_requests_while_unplugged_are_ignored_but_the_memory_stays(self):
        self.enable_by_pad()
        self.firmware.disconnect()
        self.press(0.1, [START_BUTTON])
        self.release_all(0.1)
        self.assertFalse(self.call_enable(True).success)
        self.firmware.reconnect()
        self.release_all(2.0)
        self.assert_reenabled_count(2)

    def test_enable_request_while_unplugged_does_not_enable_a_disabled_board(self):
        self.release_all()
        self.firmware.disconnect()
        self.press(0.1, [START_BUTTON])
        self.release_all(0.1)
        self.firmware.reconnect()
        self.release_all(1.5)
        self.assertEqual(self.firmware.enable_count, 0)
        self.assertEqual(self.firmware.state, STATE_DISABLED)

    def test_back_while_waiting_for_neutral_cancels_the_reenable(self):
        self.enable_by_pad()
        self.unplug_and_replug(buttons=[DEADMAN_BUTTON])
        self.spin_until_waiting_for_neutral([DEADMAN_BUTTON])
        self.press(0.2, [DEADMAN_BUTTON, BACK_BUTTON])
        self.press(0.3, [DEADMAN_BUTTON])
        self.release_all(1.0)
        self.assert_reenabled_count(1)

    def test_firmware_that_stopped_itself_before_the_disconnect_is_not_reenabled(self):
        self.firmware.ignored_enable = True
        self.release_all()
        self.press(0.1, [START_BUTTON])  # The node waits for ACTIVE for STATE_WAIT_S.
        self.firmware.set_state(STATE_DISABLED, 5)  # arming timeout
        self.spin_for(0.15)
        self.firmware.ignored_enable = False
        self.unplug_and_replug()
        self.release_all(2.0)
        self.assertEqual(self.firmware.enable_count, 1)
        self.assertEqual(self.firmware.state, STATE_DISABLED)

    def test_status_that_went_silent_before_the_disconnect_is_not_trusted(self):
        self.enable_by_pad()
        self.firmware.is_status_sent = False
        self.spin_for(0.6)
        self.firmware.is_status_sent = True
        self.unplug_and_replug()
        self.release_all(2.0)
        self.assert_reenabled_count(1)

    def test_service_enable_during_automatic_arming_starts_the_settle_window(self):
        self.set_parameter("joy_enable_button", -1)
        self.assertTrue(self.call_enable(True).success)
        self.unplug_and_replug()
        deadline = time.monotonic() + 3.0
        while self.node._connection != "ARMING" and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.002)
        self.assertEqual(self.node._connection, "ARMING")
        self.assertTrue(self.call_enable(True).success)
        self.assertGreater(self.node._ignore_commands_until, time.monotonic())
        self.assertEqual(self.node._connection, "CONNECTED")

    def test_replug_after_a_manual_disable_stays_disabled(self):
        self.enable_by_pad()
        self.press(0.1, [BACK_BUTTON])
        self.release_all(0.5)
        self.unplug_and_replug()
        self.release_all(1.5)
        self.assert_reenabled_count(1)

    def test_replug_while_disabled_stays_disabled(self):
        self.release_all()
        self.unplug_and_replug()
        self.release_all(1.5)
        self.assertEqual(self.firmware.enable_count, 0)
        self.assertEqual(self.firmware.state, STATE_DISABLED)

    def test_silent_board_is_closed_and_retried(self):
        self.enable_by_pad()
        self.firmware.is_status_sent = False
        with mock.patch.object(diff_drive_node, "LoggingBridge",
                               wraps=diff_drive_node.LoggingBridge) as open_bridge:
            self.unplug_and_replug()
            self.release_all(STATE_WAIT_S * 2 + 0.5)
            self.assertGreaterEqual(open_bridge.call_count, 2)
            self.assertEqual(self.firmware.enable_count, 1)
            self.firmware.is_status_sent = True
            self.release_all(1.5)
        self.assert_reenabled_count(2)

    def test_recovers_every_time_without_a_limit(self):
        self.enable_by_pad()
        for expected_enable_count in range(2, 7):
            with self.subTest(expected_enable_count):
                self.unplug_and_replug()
                self.release_all(1.5)
                self.assert_reenabled_count(expected_enable_count)

    def test_closing_while_unplugged_does_not_raise(self):
        self.enable_by_pad()
        self.firmware.disconnect()
        self.spin_for(0.3)
        self.node.stop_motors()
        self.node.close()

    def test_rejects_invalid_auto_reenable_wait(self):
        for seconds in (-1.0, float("nan"), float("inf")):
            with self.subTest(seconds):
                results = self.node.set_parameters(
                    [Parameter("auto_reenable_wait_s", value=seconds)])
                self.assertFalse(results[0].successful)


METERS_PER_REVOLUTION = 2.0 * math.pi * 0.0935
FORWARD_SPEED_M_S = METERS_PER_REVOLUTION  # 60 rpm
SPIN_RATE_RAD_S = METERS_PER_REVOLUTION * 2.0 / 0.26  # wheels at -60 / +60 rpm
FEEDBACK_STALE_S = 0.1
STEADY_AFTER_S = 0.2
MIN_MESSAGES_PER_SECOND = 40
TICK_S = 0.02


def stamp_seconds(message: Odometry) -> float:
    return message.header.stamp.sec + message.header.stamp.nanosec * 1e-9


class WheelOdometryTest(NodeTestBase):
    def setUp(self):
        super().setUp()
        self.odometry = []
        self.transforms = []
        self.client_node.create_subscription(Odometry, "/wheel/odom", self.odometry.append, 50)
        self.client_node.create_subscription(TFMessage, "/tf", self.transforms.append, 50)
        odom_publisher = self.node._odometry._odom_publisher
        deadline = time.monotonic() + 5.0
        while odom_publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.01)
        self.spin_for(0.2)

    def feed(self, first_rpm: int, second_rpm: int) -> None:
        self.firmware.start_feedback(1, first_rpm)
        self.firmware.start_feedback(2, second_rpm)

    def feed_forward(self) -> None:
        self.feed(60, -60)

    def feed_spin(self) -> None:
        # With the default signs both motors turn the same way for the wheels to oppose.
        self.feed(-60, -60)

    def steady_rates(self, field) -> float:
        """Slope of a pose field over the steady part of what was received so far."""
        start = stamp_seconds(self.odometry[0]) + STEADY_AFTER_S
        steady = [message for message in self.odometry if stamp_seconds(message) >= start]
        self.assertGreater(len(steady), 5)
        return ((field(steady[-1]) - field(steady[0]))
                / (stamp_seconds(steady[-1]) - stamp_seconds(steady[0])))

    def message_count_after(self, seconds: float) -> int:
        self.spin_for(seconds)
        return len(self.odometry)

    @staticmethod
    def x(message): return message.pose.pose.position.x

    @staticmethod
    def y(message): return message.pose.pose.position.y

    @staticmethod
    def yaw(message):
        orientation = message.pose.pose.orientation
        return 2.0 * math.atan2(orientation.z, orientation.w)

    def test_publishes_nothing_without_feedback(self):
        self.spin_for(0.3)
        self.assertEqual(self.odometry, [])
        self.assertEqual(self.transforms, [])

    def test_straight_line_matches_the_wheel_speed(self):
        self.feed_forward()
        self.spin_for(1.0)
        self.assertAlmostEqual(self.steady_rates(self.x), FORWARD_SPEED_M_S,
                               delta=FORWARD_SPEED_M_S * 0.05)
        last = self.odometry[-1]
        self.assertAlmostEqual(last.twist.twist.linear.x, FORWARD_SPEED_M_S, delta=0.02)
        self.assertLess(abs(self.y(last)), 0.01)
        self.assertLess(abs(self.yaw(last)), 0.02)
        self.assertEqual(last.child_frame_id, "base_footprint")
        self.assertEqual(last.header.frame_id, "odom")
        self.assertAlmostEqual(self.x(last), FORWARD_SPEED_M_S, delta=FORWARD_SPEED_M_S * 0.15)

    def test_rate_and_stamps_are_steady(self):
        self.feed_forward()
        self.spin_for(1.0)
        self.assertGreaterEqual(len(self.odometry), MIN_MESSAGES_PER_SECOND)
        stamps = [stamp_seconds(message) for message in self.odometry]
        self.assertEqual(stamps, sorted(set(stamps)))

    def test_spin_in_place(self):
        self.feed_spin()
        self.spin_for(0.5)
        self.assertAlmostEqual(self.steady_rates(self.yaw), SPIN_RATE_RAD_S,
                               delta=SPIN_RATE_RAD_S * 0.05)
        self.assertLess(abs(self.x(self.odometry[-1])), 0.01)
        self.assertLess(abs(self.y(self.odometry[-1])), 0.01)

    def test_flipped_signs_still_drive_forward(self):
        self.set_parameter("left_sign", -1)
        self.set_parameter("right_sign", 1)
        self.feed(-60, 60)
        self.spin_for(0.6)
        self.assertAlmostEqual(self.steady_rates(self.x), FORWARD_SPEED_M_S,
                               delta=FORWARD_SPEED_M_S * 0.05)

    def test_swapped_motor_ids_still_drive_forward(self):
        self.set_parameter("left_motor_id", 2)
        self.feed(-60, 60)  # Motor 2 is the left wheel (sign +1), motor 1 the right (sign -1).
        self.spin_for(0.6)
        self.assertAlmostEqual(self.steady_rates(self.x), FORWARD_SPEED_M_S,
                               delta=FORWARD_SPEED_M_S * 0.05)

    def test_integrates_while_disabled(self):
        self.assertEqual(self.firmware.state, STATE_DISABLED)
        self.feed_forward()
        self.spin_for(1.0)
        self.assertEqual(self.firmware.state, STATE_DISABLED)
        self.assertGreater(self.x(self.odometry[-1]), 0.3)

    def test_publishes_at_rate_while_standing_still(self):
        self.feed(0, 0)
        self.spin_for(1.0)
        self.assertGreaterEqual(len(self.odometry), MIN_MESSAGES_PER_SECOND)
        for message in self.odometry:
            self.assertEqual((self.x(message), self.y(message), self.yaw(message)), (0.0, 0.0, 0.0))
        stamps = [stamp_seconds(message) for message in self.odometry]
        self.assertEqual(stamps, sorted(set(stamps)))

    def test_silent_wheel_stops_the_extrapolation_after_a_stale_period(self):
        self.feed_forward()
        self.spin_for(0.6)
        x_at_stop = self.x(self.odometry[-1])
        self.firmware.stop_feedback(2)
        self.spin_for(0.3)
        count_when_quiet = len(self.odometry)
        self.assertEqual(self.message_count_after(0.2), count_when_quiet)
        extrapolated_m = self.x(self.odometry[-1]) - x_at_stop
        self.assertLessEqual(extrapolated_m,
                             (FEEDBACK_STALE_S + 2 * TICK_S) * FORWARD_SPEED_M_S)
        x_paused = self.x(self.odometry[-1])
        self.firmware.start_feedback(2, -60)
        self.spin_for(0.5)
        self.assertGreater(self.x(self.odometry[-1]), x_paused)
        self.assertGreaterEqual(min(self.x(message) for message in self.odometry[-5:]),
                                x_at_stop)

    def test_silent_wheel_while_spinning_does_not_keep_turning(self):
        self.feed_spin()
        self.spin_for(0.4)
        yaw_at_stop = self.yaw(self.odometry[-1])
        self.firmware.stop_feedback(1)
        self.spin_for(0.3)
        count_when_quiet = len(self.odometry)
        self.assertEqual(self.message_count_after(0.2), count_when_quiet)
        turned_rad = self.yaw(self.odometry[-1]) - yaw_at_stop
        self.assertLessEqual(turned_rad, (FEEDBACK_STALE_S + 2 * TICK_S) * SPIN_RATE_RAD_S)

    def test_stops_on_disconnect_and_continues_from_the_same_pose(self):
        self.feed_forward()
        self.spin_for(0.6)
        x_before = self.x(self.odometry[-1])
        self.firmware.disconnect()
        self.spin_for(0.4)
        count_when_quiet = len(self.odometry)
        self.assertEqual(self.message_count_after(0.2), count_when_quiet)
        self.firmware.reconnect()
        self.spin_for(1.5)
        self.assertEqual(len(self.odometry), count_when_quiet)
        self.feed_forward()
        self.spin_for(0.5)
        self.assertGreater(len(self.odometry), count_when_quiet)
        self.assertGreater(self.x(self.odometry[count_when_quiet]), x_before - 1e-9)
        self.assertGreater(self.x(self.odometry[-1]), x_before + 0.1)

    def test_pose_keeps_integrating_while_enable_blocks(self):
        self.feed_forward()
        self.spin_for(0.5)
        self.assertTrue(self.call_enable(True).success)
        self.spin_for(0.2)
        gaps = [(stamp_seconds(after) - stamp_seconds(before), before, after)
                for before, after in zip(self.odometry, self.odometry[1:])]
        gap_s, before, after = max(gaps, key=lambda gap: gap[0])
        self.assertGreater(gap_s, ARMING_DELAY_S * 0.8)  # It really blocked.
        self.assertAlmostEqual((self.x(after) - self.x(before)) / gap_s, FORWARD_SPEED_M_S,
                               delta=FORWARD_SPEED_M_S * 0.2)

    def test_tf_matches_odometry(self):
        self.feed_forward()
        self.spin_for(0.6)
        by_stamp = {(t.header.stamp.sec, t.header.stamp.nanosec): t
                    for message in self.transforms for t in message.transforms
                    if t.child_frame_id == "base_footprint"}
        self.assertGreater(len(by_stamp), 5)
        matched = 0
        for odometry in self.odometry:
            transform = by_stamp.get((odometry.header.stamp.sec, odometry.header.stamp.nanosec))
            if transform is None:
                continue
            matched += 1
            self.assertEqual(transform.header.frame_id, "odom")
            self.assertAlmostEqual(transform.transform.translation.x, self.x(odometry), delta=1e-6)
            self.assertAlmostEqual(transform.transform.translation.y, self.y(odometry), delta=1e-6)
            self.assertAlmostEqual(transform.transform.rotation.z,
                                   odometry.pose.pose.orientation.z, delta=1e-6)
            self.assertAlmostEqual(transform.transform.rotation.w,
                                   odometry.pose.pose.orientation.w, delta=1e-6)
        self.assertGreater(matched, 5)

    def test_publish_tf_false_keeps_odometry_but_drops_tf(self):
        self.set_parameter("publish_tf", False)
        self.feed_forward()
        self.spin_for(0.3)
        self.assertGreater(len(self.odometry), 5)
        self.assertEqual([t for message in self.transforms for t in message.transforms
                          if t.child_frame_id == "base_footprint"], [])
        self.set_parameter("publish_tf", True)
        self.spin_for(0.3)
        self.assertTrue([t for message in self.transforms for t in message.transforms
                         if t.child_frame_id == "base_footprint"])

    def test_rejects_non_bool_publish_tf(self):
        try:
            results = self.node.set_parameters([Parameter("publish_tf", value=1)])
        except rclpy.exceptions.ParameterException:
            return  # rclpy refuses the type itself before the node's callback.
        self.assertFalse(results[0].successful)

    def test_drive_ramps_while_feedback_flows(self):
        self.feed_forward()
        self.spin_for(0.15)
        self.assertTrue(self.call_enable(True).success)
        self.spin_for(0.5, forward_twist())
        first, second = self.firmware.velocities[-1]
        self.assertGreater(first, 10)
        self.assertEqual(second, -first)

    def test_failing_odometry_publisher_does_not_stop_the_drive(self):
        self.feed_forward()
        self.assertTrue(self.call_enable(True).success)
        with mock.patch.object(self.node._odometry._odom_publisher, "publish",
                               side_effect=RuntimeError("boom")):
            self.spin_for(0.5, forward_twist())
        first, second = self.firmware.velocities[-1]
        self.assertGreater(first, 10)
        self.assertEqual(second, -first)


def make_joy(buttons=(), axes=None) -> Joy:
    message = Joy()
    message.buttons = [1 if index in buttons else 0 for index in range(BUTTON_COUNT)]
    message.axes = [float((axes or {}).get(index, 0.0)) for index in range(AXIS_COUNT)]
    return message


def forward_twist() -> Twist:
    message = Twist()
    message.linear.x = 2.0
    return message


if __name__ == "__main__":
    unittest.main()
