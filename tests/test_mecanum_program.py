from __future__ import annotations

import unittest

import mecanum


class MecanumProgramTests(unittest.TestCase):
    def test_speed_percent_is_limited_to_safe_range(self) -> None:
        self.assertEqual(mecanum._speed_span(-1), 0)
        self.assertEqual(mecanum._speed_span(50), 16384)
        self.assertEqual(mecanum._speed_span(101), 32767)

    def test_control_hz_must_be_positive(self) -> None:
        self.assertEqual(mecanum._control_interval(20), 0.05)
        with self.assertRaises(ValueError):
            mecanum._control_interval(0)
