import unittest

from rox_mecanum.feedback_servo import ATEncoderReader, EncoderPositionServo, PositionServoConfig


class FakeTransport:
    def __init__(self, incoming=b""):
        self.incoming = incoming
        self.writes = []

    def write(self, data):
        self.writes.append(data)

    def read_available(self):
        value, self.incoming = self.incoming, b""
        return value


class FakeMotor:
    def __init__(self):
        self.speeds = []

    def enable(self):
        pass

    def set_velocity(self, speed, *, force=False):
        self.speeds.append((speed, force))

    def stop(self):
        self.speeds.append((0.0, True))


class FeedbackServoTests(unittest.TestCase):
    def test_reader_decodes_encoder_frame(self):
        # address 0x2C の応答CAN IDは 0x2F。count=0x1234 little-endian。
        packet = b"AT" + bytes((0x10, 0x00, 0x2F, 0x2C, 0x08, 0x34, 0x12, 0, 0, 0, 0, 0, 0)) + b"\r\n"
        transport = FakeTransport(packet)
        reader = ATEncoderReader(transport, {"catch": 0x2C})
        feedback = reader.poll(now=1.0)
        self.assertEqual(feedback[0].name, "catch")
        self.assertEqual(feedback[0].count, 0x1234)

    def test_position_servo_corrects_external_displacement(self):
        motor = FakeMotor()
        servo = EncoderPositionServo(motor, PositionServoConfig(-90, 90, 100, kp=0.02, max_speed=0.5))
        servo.set_home(1000)
        servo.write(10)
        servo.update(1000, 1.0)
        self.assertGreater(motor.speeds[-1][0], 0.0)
        # 外力で元へ戻された場合も、再び正方向へ補正する。
        servo.update(900, 1.1)
        self.assertGreater(motor.speeds[-1][0], 0.0)

    def test_hold_and_release_api(self):
        motor = FakeMotor()
        servo = EncoderPositionServo(motor, PositionServoConfig(-90, 90, 100, kp=0.02, ki=0.01))
        servo.set_home(1000)
        servo.write(10)
        servo.update(1000, 1.0)
        self.assertFalse(servo.is_at_target())
        servo.update(2000, 1.1)
        self.assertTrue(servo.is_at_target())
        servo.release()
        self.assertFalse(servo.status()["holding"])

    def test_deadband_stops_motor_near_target(self):
        motor = FakeMotor()
        servo = EncoderPositionServo(
            motor,
            PositionServoConfig(-90, 90, 100, kp=0.02, max_speed=0.5, tolerance_deg=2.0),
        )
        servo.set_home(1000)
        servo.write(10.0)
        # 現在9°は目標10°との差が1°なので、出力しない。
        command = servo.update(1900, 1.0)
        self.assertEqual(command, 0.0)
        self.assertEqual(motor.speeds[-1], (0.0, True))

    def test_pid_on_off_aliases(self):
        motor = FakeMotor()
        servo = EncoderPositionServo(motor, PositionServoConfig(-90, 90, 100))
        servo.set_home(100)
        servo.pid_off()
        self.assertFalse(servo.pid_enabled)
        servo.pid_on()
        self.assertTrue(servo.pid_enabled)

    def test_pid_can_be_changed_while_running(self):
        motor = FakeMotor()
        servo = EncoderPositionServo(motor, PositionServoConfig(-90, 90, 100, kp=0.02, max_speed=0.5))
        config = servo.set_pid(kp=0.003, ki=0.001, max_speed=0.05, tolerance_deg=2.0)
        self.assertEqual(config.kp, 0.003)
        self.assertEqual(config.ki, 0.001)
        self.assertEqual(config.max_speed, 0.05)
        self.assertEqual(config.tolerance_deg, 2.0)
