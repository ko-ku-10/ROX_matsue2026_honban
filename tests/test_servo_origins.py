import tempfile
import unittest
from pathlib import Path

from rox_mecanum.feedback_servo import EncoderPositionServo, PositionServoConfig
from servos import ServoMotors, _phase_delta_degrees


class FakeMotor:
    def enable(self):
        pass

    def set_velocity(self, _speed, *, force=False):
        pass

    def stop(self):
        pass


def make_servos() -> ServoMotors:
    config = PositionServoConfig(-360, 360, 100)
    return ServoMotors(
        EncoderPositionServo(FakeMotor(), config),
        EncoderPositionServo(FakeMotor(), config),
        reader=object(),
        transport=object(),
        owns_transport=False,
    )


class ServoOriginTests(unittest.TestCase):
    def test_phase_delta_ignores_the_zero_and_360_degree_boundary(self):
        # 359° → 1° は +2°の動きであり、-358°ではない。
        self.assertAlmostEqual(_phase_delta_degrees(-358.0 * 3.141592653589793 / 180.0), 2.0)

    def test_saved_mechpos_origins_are_restored(self):
        original = make_servos()
        original.catch.set_home_radians(1.25)
        original.lift.set_home_radians(-2.5)

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "servo_origins.json"
            original.save_origins(path)

            restored = make_servos()
            restored.load_origins(path)

        self.assertAlmostEqual(restored.catch.home_position_rad, 1.25)
        self.assertAlmostEqual(restored.lift.home_position_rad, -2.5)
