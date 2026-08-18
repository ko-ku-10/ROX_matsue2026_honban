from __future__ import annotations

import unittest

from rox_mecanum import EncoderServo, ServoConfig


class FakeMotor:
    def __init__(self) -> None:
        self.enabled = False
        self.commands: list[float] = []

    def enable(self) -> None:
        self.enabled = True

    def set_velocity(self, speed: float, *, force: bool = False) -> None:
        self.commands.append(speed)

    def stop(self) -> None:
        self.commands.append(0.0)


class EncoderServoTests(unittest.TestCase):
    def setUp(self) -> None:
        self.motor = FakeMotor()
        self.servo = EncoderServo(
            self.motor,
            ServoConfig(
                min_position_deg=-20,
                max_position_deg=90,
                max_command=0.3,
                position_kp=0.01,
                command_accel_per_sec=1000,
                tolerance_deg=1,
            ),
        )
        self.servo.enable()
        self.servo.set_home(100.0)

    def test_move_to_uses_encoder_error_for_velocity(self) -> None:
        self.assertEqual(self.servo.move_to(45), 45)
        state = self.servo.update(100.0, now=1.0)
        self.assertEqual(state.position_deg, 0.0)
        self.assertEqual(state.command, 0.3)
        self.assertFalse(state.at_target)

    def test_target_is_limited_and_stops_at_target(self) -> None:
        self.assertEqual(self.servo.move_to(999), 90)
        state = self.servo.update(190.0, now=1.0)
        self.assertTrue(state.at_target)
        self.assertEqual(state.command, 0.0)

    def test_soft_limit_is_reported_and_motor_returns_inward(self) -> None:
        self.servo.move_to(90)
        state = self.servo.update(195.0, now=1.0)
        self.assertTrue(state.limited)
        self.assertLess(state.command, 0.0)

    def test_direction_can_be_reversed(self) -> None:
        servo = EncoderServo(self.motor, ServoConfig(direction=-1, command_accel_per_sec=100))
        servo.set_home(10)
        servo.move_to(20)
        state = servo.update(0, now=1.0)
        self.assertEqual(state.position_deg, 10)
        self.assertGreater(state.command, 0)

    def test_pid_integral_term_accumulates_when_error_remains(self) -> None:
        servo = EncoderServo(
            self.motor,
            ServoConfig(
                max_command=0.8,
                position_kp=0.01,
                position_ki=0.02,
                position_kd=0.0,
                command_accel_per_sec=1000,
            ),
        )
        servo.set_home(0)
        servo.write(10)
        first = servo.loop(0, now=1.0)
        second = servo.loop(0, now=2.0)
        self.assertGreater(second.i_term, first.i_term)
        self.assertGreater(second.command, first.command)

    def test_arduino_style_aliases(self) -> None:
        self.servo.attach()
        self.assertEqual(self.servo.write(30), 30)
        self.assertEqual(self.servo.read(), 30)
        self.servo.loop(100.0, now=1.0)
        self.assertEqual(self.servo.read_position(), 0.0)
        self.servo.detach()
        self.assertEqual(self.motor.commands[-1], 0.0)
