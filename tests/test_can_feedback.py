from __future__ import annotations

import unittest

from rox_mecanum import CanEncoderReceiver, decode_motor_feedback


class FakeMessage:
    def __init__(self, arbitration_id: int, data: bytes, extended: bool = True) -> None:
        self.arbitration_id = arbitration_id
        self.data = data
        self.is_extended_id = extended


class FakeBus:
    def __init__(self, messages: list[FakeMessage]) -> None:
        self.messages = messages
        self.closed = False

    def recv(self, timeout: float | None = None) -> FakeMessage | None:
        return self.messages.pop(0) if self.messages else None

    def shutdown(self) -> None:
        self.closed = True


def _type_2_id(motor_id: int, host_id: int = 0xFD) -> int:
    return (0x02 << 24) | (motor_id << 8) | host_id


class CanFeedbackTests(unittest.TestCase):
    def test_decodes_type_2_feedback(self) -> None:
        message = FakeMessage(
            _type_2_id(5),
            bytes.fromhex("80008000800000FA"),
        )
        feedback = decode_motor_feedback(message)
        assert feedback is not None
        self.assertEqual(feedback.motor_id, 5)
        self.assertEqual(feedback.host_id, 0xFD)
        self.assertAlmostEqual(feedback.position_rad, 0.0, places=3)
        self.assertAlmostEqual(feedback.velocity_rad_per_sec, 0.0, places=2)
        self.assertAlmostEqual(feedback.torque_nm, 0.0, places=3)
        self.assertEqual(feedback.temperature_c, 25.0)

    def test_ignores_non_feedback_frames(self) -> None:
        message = FakeMessage(_type_2_id(5), b"\x00" * 8, extended=False)
        self.assertIsNone(decode_motor_feedback(message))

    def test_receiver_filters_for_requested_motor(self) -> None:
        bus = FakeBus([
            FakeMessage(_type_2_id(12), b"\x00" * 8),
            FakeMessage(_type_2_id(5), b"\x00" * 8),
        ])
        receiver = CanEncoderReceiver(bus)
        feedback = receiver.read(5, timeout=0.1)
        assert feedback is not None
        self.assertEqual(feedback.motor_id, 5)
        receiver.close()
        self.assertTrue(bus.closed)
