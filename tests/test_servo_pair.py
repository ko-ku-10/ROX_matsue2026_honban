from types import SimpleNamespace

from rox_mecanum.servo import EncoderServo, ServoConfig
from rox_mecanum.servo_pair import ServoPair


class FakeMotor:
    def __init__(self):
        self.enabled = False
        self.commands = []

    def enable(self):
        self.enabled = True

    def set_velocity(self, speed, *, force=False):
        self.commands.append(speed)

    def stop(self):
        self.set_velocity(0.0, force=True)


class FakeBus:
    def __init__(self):
        self.messages = []
        self.closed = False

    def recv(self, timeout=None):
        return self.messages.pop(0) if self.messages else None

    def shutdown(self):
        self.closed = True


class Message:
    is_extended_id = True

    def __init__(self, motor_id, raw_position=32768):
        self.arbitration_id = (0x02 << 24) | (motor_id << 8)
        self.data = raw_position.to_bytes(2, "big") + bytes(6)


def test_pair_homes_each_axis_then_updates_only_its_axis():
    catch_motor = FakeMotor()
    lift_motor = FakeMotor()
    catch = EncoderServo(catch_motor, ServoConfig(min_position_deg=-90, max_position_deg=90))
    lift = EncoderServo(lift_motor, ServoConfig(min_position_deg=-90, max_position_deg=90))
    bus = FakeBus()
    pair = ServoPair(catch, lift, bus, catch_id=5, lift_id=6)

    pair.catch.attach()
    pair.lift.attach()
    pair.catch.write(30)  # 原点前の目標は原点登録時に安全に0へ戻る
    bus.messages = [Message(5), Message(6), Message(5, raw_position=40000)]

    assert pair.update() is None
    assert catch.is_homed
    assert not lift.is_homed
    assert pair.update() is None
    assert pair.is_ready

    pair.catch.write(30)
    state = pair.update()
    assert state is not None
    assert state.position_deg > 0
    assert catch_motor.commands[-1] < 0
