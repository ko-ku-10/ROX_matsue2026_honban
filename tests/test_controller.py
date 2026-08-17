from __future__ import annotations

import math
import unittest

from rox_mecanum import AnalogStick, Button, ControllerState


class ControllerTests(unittest.TestCase):
    def test_stick_angle_and_direction(self) -> None:
        self.assertIsNone(AnalogStick().angle_degrees)
        self.assertEqual(AnalogStick(1.0, 0.0).angle_degrees, 0.0)
        self.assertEqual(AnalogStick(0.0, 1.0).angle_degrees, 90.0)
        self.assertEqual(AnalogStick(-1.0, 0.0).direction, "left")
        self.assertEqual(AnalogStick(0.5, 0.5).direction, "up_right")

    def test_stick_magnitude_is_normalized(self) -> None:
        stick = AnalogStick(1.0, 1.0)
        self.assertEqual(stick.magnitude, 1.0)
        self.assertTrue(math.isclose(AnalogStick(0.5, 0.0).with_deadzone(0.1).x, 4 / 9))

    def test_controller_state_exposes_buttons_and_transitions(self) -> None:
        state = ControllerState(
            buttons={Button.CROSS: True, Button.OPTIONS: False},
            pressed=frozenset({Button.CROSS}),
            released=frozenset({Button.OPTIONS}),
            raw_axes=(0.1, -0.2),
            raw_buttons=(True, False),
        )
        self.assertTrue(state.button("cross"))
        self.assertTrue(state.was_pressed(Button.CROSS))
        self.assertTrue(state.was_released("options"))
        self.assertEqual(state.active_buttons, frozenset({Button.CROSS}))
        self.assertEqual(state.raw_buttons, (True, False))
