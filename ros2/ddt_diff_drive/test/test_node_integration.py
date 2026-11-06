"""Runs the real node against a pseudo-terminal that plays the firmware's role."""

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
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool

from ddt_diff_drive.ddt_bridge import (
    STATE_ACTIVE,
    STATE_DISABLED,
    TYPE_DISABLE,
    TYPE_ENABLE,
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
        if self.is_status_sent and now - self._last_status_time >= STATUS_PERIOD_S:
            self._last_status_time = now
            os.write(self.controller_fd, encode_frame(TYPE_STATUS, bytes([self.state, self.stop_reason])))

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


class DiffDriveNodeTest(unittest.TestCase):
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

    def set_parameter(self, name: str, value) -> None:
        self.assertTrue(self.node.set_parameters([Parameter(name, value=value)])[0].successful)

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
