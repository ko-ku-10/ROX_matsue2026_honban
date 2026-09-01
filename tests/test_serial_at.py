from __future__ import annotations

import unittest

from rox_mecanum import (
    AT_NEUTRAL_VALUE,
    MecanumRobot,
    MotionCommand,
    at_address_from_can_id,
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
    def test_converts_can_id_to_at_address(self) -> None:
        self.assertEqual(at_address_from_can_id(5), 0x2C)
        self.assertEqual(at_address_from_can_id(6), 0x34)

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

    def test_drive_status_shows_commands_not_measurements(self) -> None:
        transport = FakeTransport()
        robot = MecanumRobot(transport, acceleration_per_second=2.0)
        robot.drive(MotionCommand(forward=0.5))

        status = robot.drive_status()

        self.assertEqual(status["acceleration_limit_per_sec"], 2.0)
        self.assertEqual(set(status["wheel_speed_commands"]), {"FL", "FR", "RL", "RR"})

    def test_braking_can_be_faster_than_acceleration(self) -> None:
        transport = FakeTransport()
        robot = MecanumRobot(
            transport,
            acceleration_per_second=1.0,
            deceleration_per_second=10.0,
        )
        robot._last_wheel_speeds = {"FL": 0.5, "FR": 0.5, "RL": 0.5, "RR": 0.5}
        robot._last_drive_at = None

        stopped = robot._apply_acceleration_limit({"FL": 0.0, "FR": 0.0, "RL": 0.0, "RR": 0.0})

        # 最初の周期は0.02秒。減速10.0なら0.20だけ減速できる。
        self.assertAlmostEqual(stopped.front_left, 0.3)
