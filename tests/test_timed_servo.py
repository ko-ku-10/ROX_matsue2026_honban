import unittest

from rox_mecanum import TimedServo, TimedServoConfig


class FakeMotor:
    def __init__(self):
        self.commands = []

    def enable(self):
        self.commands.append(("enable",))

    def set_velocity(self, speed, *, force=False):
        self.commands.append(("speed", speed, force))

    def stop(self):
        self.commands.append(("stop",))


class TimedServoTests(unittest.TestCase):
    def setUp(self):
        self.motor = FakeMotor()
        self.waits = []
        self.servo = TimedServo(
            self.motor,
            TimedServoConfig(0, 90, degrees_per_second=30, calibration_speed=0.2),
            sleep=self.waits.append,
        )

    def test_home_is_required(self):
        with self.assertRaises(RuntimeError):
            self.servo.write(30)

    def test_write_moves_for_calibrated_duration(self):
        self.servo.home(0)
        self.assertEqual(self.servo.write(90), 90)
        self.assertEqual(self.waits, [3.0])
        self.assertIn(("enable",), self.motor.commands)
        self.assertEqual(self.motor.commands[-1], ("stop",))

    def test_limits_target_and_scales_time_for_speed(self):
        self.servo.home(0)
        self.assertEqual(self.servo.write(200, speed=0.4), 90)
        self.assertEqual(self.waits, [1.5])

    def test_direction_reverses_motor_command(self):
        servo = TimedServo(
            self.motor,
            TimedServoConfig(0, 90, 30, 0.2, direction=-1),
            sleep=self.waits.append,
        )
        servo.home(0)
        servo.write(30)
        self.assertIn(("speed", -0.2, True), self.motor.commands)
