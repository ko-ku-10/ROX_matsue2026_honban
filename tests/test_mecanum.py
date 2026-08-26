from __future__ import annotations

import unittest

from rox_mecanum import (
    Button,
    ControllerState,
    DualSenseMotionMapping,
    MecanumMixer,
    MotionCommand,
    AnalogStick,
    backward,
    forward,
    strafe_left,
    turn_right,
)


class MecanumTests(unittest.TestCase):
    def test_shortcut_commands(self) -> None:
        self.assertEqual(forward(0.4), MotionCommand(forward=0.4))
        self.assertEqual(backward(0.4), MotionCommand(forward=-0.4))
        self.assertEqual(strafe_left(0.4), MotionCommand(strafe=-0.4))
        self.assertEqual(turn_right(0.4), MotionCommand(rotate=0.4))

    def test_forward_and_strafe_wheel_mixing(self) -> None:
        mixer = MecanumMixer(rotation_gain=0.22)
        self.assertEqual(mixer.wheel_speeds(forward=1.0).as_dict(), {"FL": 1.0, "FR": 1.0, "RL": 1.0, "RR": 1.0})
        self.assertEqual(mixer.wheel_speeds(strafe=1.0).as_dict(), {"FL": -1.0, "FR": 1.0, "RL": 1.0, "RR": -1.0})

    def test_mixer_normalizes_combined_motion(self) -> None:
        speeds = MecanumMixer(rotation_gain=1.0).wheel_speeds(forward=1.0, strafe=1.0, rotate=1.0)
        self.assertEqual(speeds.as_dict(), {"FL": -1 / 3, "FR": 1.0, "RL": 1 / 3, "RR": 1 / 3})

    def test_dualsense_mapping_does_not_require_trigger_for_rotation(self) -> None:
        state = ControllerState(
            left_stick=AnalogStick(0.4, 0.6),
            right_stick=AnalogStick(0.5, 0.0),
            buttons={Button.L2: False, Button.R2: False},
        )
        command = DualSenseMotionMapping(deadzone=0.0).command(state)
        self.assertEqual(command, MotionCommand(forward=0.6, strafe=0.4, rotate=0.5))
