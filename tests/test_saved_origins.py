import json

from servos import ServoMotors


class FakeServo:
    def __init__(self):
        self.home_radians = None

    def set_home_radians(self, value):
        self.home_radians = value


def test_saved_origins_are_loaded_as_servo_home_positions(tmp_path):
    origin_file = tmp_path / "servo_origins.json"
    origin_file.write_text(
        json.dumps({"format": 1, "catch_zero_rad": 1.25, "lift_zero_rad": -0.5}),
        encoding="utf-8",
    )
    motors = object.__new__(ServoMotors)
    motors.catch = FakeServo()
    motors.lift = FakeServo()

    assert motors.load_origins(origin_file) is True
    assert motors.catch.home_radians == 1.25
    assert motors.lift.home_radians == -0.5


def test_missing_saved_origins_returns_false_without_moving_servos(tmp_path):
    motors = object.__new__(ServoMotors)
    motors.catch = FakeServo()
    motors.lift = FakeServo()

    assert motors.load_origins(tmp_path / "missing.json") is False
    assert motors.catch.home_radians is None
    assert motors.lift.home_radians is None
