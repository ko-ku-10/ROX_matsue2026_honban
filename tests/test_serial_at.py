from __future__ import annotations

import unittest

from rox_mecanum import (
    AT_NEUTRAL_VALUE,
    MecanumRobot,
    MotionCommand,
    build_enable_frame,
    build_velocity_frame,
    normalized_to_at_value,
)


class FakeTransport:
    def __init__(self) -> None:
        self.frames: list[bytes] = []

    def write(self, data: bytes) -> None:
        self.frames.append(data)


class SerialAtTests(unittest.TestCase):
    def test_at_velocity_frame_has_expected_neutral_value(self) -> None:
        frame = build_velocity_frame(0x0C, 0.0)
        self.assertEqual(len(frame), 17)
        self.assertEqual(frame[5], 0x0C)
        self.assertEqual(frame[12], 0x00)
        self.assertEqual(frame[13:15], AT_NEUTRAL_VALUE.to_bytes(2, "big"))
        self.assertEqual(normalized_to_at_value(2.0), 0xFFFF - 1)

    def test_robot_uses_motor_orientation_from_original_program(self) -> None:
        transport = FakeTransport()
        robot = MecanumRobot(transport)
        robot.drive(MotionCommand(forward=0.5))
        values = [int.from_bytes(frame[13:15], "big") for frame in transport.frames]
        self.assertEqual(values, [
            normalized_to_at_value(0.5),
            normalized_to_at_value(-0.5),
            normalized_to_at_value(0.5),
            normalized_to_at_value(-0.5),
        ])
        self.assertTrue(build_enable_frame(0x0C).startswith(b"AT"))
