#!/usr/bin/env python3
"""PC client for the RP2350-CAN DDT motor bridge. Standard library only.

Examples:
    ddt_bridge.py /dev/cu.usbmodem1101 monitor
    ddt_bridge.py /dev/cu.usbmodem1101 run 20 -20 --seconds 3
"""

import argparse
import os
import select
import struct
import sys
import termios
import time
import tty
from dataclasses import dataclass

HEADER = b"\xaa\x55"
MAX_PAYLOAD_SIZE = 16
TYPE_ENABLE = 0x01
TYPE_DISABLE = 0x02
TYPE_SET_VELOCITY = 0x03
TYPE_FEEDBACK = 0x81
TYPE_STATUS = 0x82
TYPE_ERROR = 0x83
STATE_DISABLED = 0
STATE_ACTIVE = 2
STATE_NAMES = {0: "DISABLED", 1: "ARMING", 2: "ACTIVE", 3: "STOPPING"}
STOP_REASONS = {
    0: "none",
    1: "disable requested",
    2: "command timeout",
    3: "feedback lost",
    4: "motor fault",
    5: "arming timeout",
    6: "CAN transmit failed",
    7: "motor left velocity mode",
}
ERROR_NAMES = {
    1: "CRC mismatch",
    2: "invalid length",
    3: "velocity out of range",
    4: "unknown type",
    5: "invalid state",
    6: "CAN transmit timeout",
}
COMMAND_PERIOD_SECONDS = 0.02
ARMING_WAIT_SECONDS = 1.0
STOP_WAIT_SECONDS = 1.0
MAX_VELOCITY_RPM = 30  # Keep equal to firmware/src/bridge_controller.hpp


def crc16(data: bytes) -> int:
    """CRC16-CCITT-FALSE, matching firmware/src/pc_protocol.cpp."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def encode_frame(message_type: int, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD_SIZE:
        raise ValueError(f"payload too long: {len(payload)}")
    body = bytes([message_type, len(payload)]) + payload
    return HEADER + body + struct.pack("<H", crc16(body))


def decode_frames(buffer: bytearray) -> list[tuple[int, bytes]]:
    """Remove every complete valid frame from buffer; bad data is skipped."""
    frames = []
    while True:
        start = buffer.find(HEADER)
        if start < 0:
            # Keep a trailing 0xAA: it may be the first half of a split header.
            del buffer[: max(len(buffer) - 1, 0)]
            return frames
        del buffer[:start]
        if len(buffer) < 4:
            return frames
        payload_size = buffer[3]
        if payload_size > MAX_PAYLOAD_SIZE:
            del buffer[:2]
            continue
        frame_size = 6 + payload_size
        if len(buffer) < frame_size:
            return frames
        body = bytes(buffer[2 : 4 + payload_size])
        received = struct.unpack_from("<H", buffer, 4 + payload_size)[0]
        if received != crc16(body):
            del buffer[:2]
            continue
        frames.append((body[0], body[2:]))
        del buffer[:frame_size]


@dataclass(frozen=True)
class Feedback:
    motor_id: int
    velocity_rpm: int
    current_amps: float
    angle_degrees: float
    fault: int
    mode: int


def parse_feedback(payload: bytes) -> Feedback:
    motor_id, velocity, current, angle, fault, mode = struct.unpack("<BhhHBB", payload)
    return Feedback(motor_id, velocity, current * 33.0 / 32767, angle * 360.0 / 32767, fault, mode)


class Bridge:
    def __init__(self, port: str):
        self._file_descriptor = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        os.set_blocking(self._file_descriptor, True)  # Reads are gated by select().
        tty.setraw(self._file_descriptor)
        termios.tcflush(self._file_descriptor, termios.TCIOFLUSH)
        self._buffer = bytearray()
        self.state = None
        self.stop_reason = 0

    def close(self) -> None:
        os.close(self._file_descriptor)

    def send(self, message_type: int, payload: bytes = b"") -> None:
        frame = encode_frame(message_type, payload)
        while frame:
            frame = frame[os.write(self._file_descriptor, frame):]

    def set_velocity(self, first_rpm: int, second_rpm: int) -> None:
        self.send(TYPE_SET_VELOCITY, struct.pack("<hh", first_rpm, second_rpm))

    def poll(self, timeout_seconds: float) -> None:
        readable, _, _ = select.select([self._file_descriptor], [], [], timeout_seconds)
        if readable:
            self._buffer += os.read(self._file_descriptor, 4096)
        for message_type, payload in decode_frames(self._buffer):
            self._handle(message_type, payload)

    def wait_for_state(self, state: int, timeout_seconds: float) -> bool:
        deadline = time.monotonic() + timeout_seconds
        while time.monotonic() < deadline:
            self.poll(0.02)
            if self.state == state:
                return True
        return False

    def _handle(self, message_type: int, payload: bytes) -> None:
        if message_type == TYPE_FEEDBACK and len(payload) == 9:
            self.on_feedback(parse_feedback(payload))
        elif message_type == TYPE_STATUS and len(payload) == 2:
            self._on_status(payload[0], payload[1])
        elif message_type == TYPE_ERROR and len(payload) == 1:
            print(f"ERROR: {ERROR_NAMES.get(payload[0], payload[0])}", file=sys.stderr)

    def _on_status(self, state: int, stop_reason: int) -> None:
        if state != self.state or stop_reason != self.stop_reason:
            print(f"state={STATE_NAMES.get(state, state)} "
                  f"stop_reason={STOP_REASONS.get(stop_reason, stop_reason)}")
        self.state = state
        self.stop_reason = stop_reason

    def on_feedback(self, feedback: Feedback) -> None:
        pass


class PrintingBridge(Bridge):
    PRINT_PERIOD_SECONDS = 0.5

    def __init__(self, port: str):
        super().__init__(port)
        self._last_print = {}

    def on_feedback(self, feedback: Feedback) -> None:
        now = time.monotonic()
        if now - self._last_print.get(feedback.motor_id, 0.0) >= self.PRINT_PERIOD_SECONDS:
            self._last_print[feedback.motor_id] = now
            print(feedback)


def run_motors(bridge: Bridge, first_rpm: int, second_rpm: int, seconds: float) -> int:
    bridge.poll(0.2)
    bridge.send(TYPE_ENABLE)
    if not bridge.wait_for_state(STATE_ACTIVE, ARMING_WAIT_SECONDS):
        print("failed to reach ACTIVE", file=sys.stderr)
        bridge.send(TYPE_DISABLE)
        return 1
    try:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            bridge.set_velocity(first_rpm, second_rpm)
            bridge.poll(COMMAND_PERIOD_SECONDS)
            if bridge.state != STATE_ACTIVE:
                print("bridge left ACTIVE", file=sys.stderr)
                return 1
    finally:
        try:
            bridge.send(TYPE_DISABLE)
            bridge.wait_for_state(STATE_DISABLED, STOP_WAIT_SECONDS)
        except OSError as error:
            # The firmware's command timeout stops the motors on its own.
            print(f"could not confirm stop: {error}", file=sys.stderr)
    return 0


def monitor(bridge: Bridge) -> int:
    while True:
        bridge.poll(0.1)


def rpm_argument(text: str) -> int:
    rpm = int(text)
    if abs(rpm) > MAX_VELOCITY_RPM:
        raise argparse.ArgumentTypeError(f"must be within +-{MAX_VELOCITY_RPM} rpm")
    return rpm


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("port")
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("monitor", help="print state and feedback; never moves the motors")
    run_parser = commands.add_parser("run", help="spin both motors, then stop")
    run_parser.add_argument("first_rpm", type=rpm_argument)
    run_parser.add_argument("second_rpm", type=rpm_argument)
    run_parser.add_argument("--seconds", type=float, default=2.0)
    arguments = parser.parse_args()

    bridge = PrintingBridge(arguments.port)
    try:
        if arguments.command == "monitor":
            return monitor(bridge)
        return run_motors(bridge, arguments.first_rpm, arguments.second_rpm, arguments.seconds)
    except KeyboardInterrupt:
        return 0
    finally:
        bridge.close()


if __name__ == "__main__":
    sys.exit(main())
