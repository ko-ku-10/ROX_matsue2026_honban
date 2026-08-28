"""ROX 向け DualSense・メカナム制御ライブラリ。

入力の取得、移動計算、AT シリアル出力を別々にも一緒にも利用できる。
"""

from .controller import (
    DEFAULT_PYGAME_PROFILE,
    AnalogStick,
    Axis,
    Button,
    ControllerProfile,
    ControllerState,
    PygameDualSense,
    open_configured_dualsense,
)
from .mecanum import (
    DualSenseMotionMapping,
    MecanumMixer,
    MotionCommand,
    WheelSpeeds,
    backward,
    forward,
    stop,
    strafe_left,
    strafe_right,
    turn_left,
    turn_right,
)
from .serial_at import (
    AT_NEUTRAL_VALUE,
    DEFAULT_MOTOR_DIRECTIONS,
    DEFAULT_MOTOR_IDS,
    ATMotor,
    MecanumRobot,
    PySerialTransport,
    at_address_from_can_id,
    build_enable_frame,
    build_velocity_frame,
    normalized_to_at_value,
)
from .timed_servo import TimedServo, TimedServoConfig
from .feedback_servo import ATEncoderReader, EncoderFeedback, EncoderPositionServo, PositionServoConfig
from .autonomy import ControlMode, ModeController, TimedMotion, add_manual_command, face_target_command
from .vision import AprilTagDetector, OpenCVSingleCamera, OpenCVStereoCamera, RDKMIPICamera, RDKMIPIStereoCamera, TagObservation, TagStore, midpoint, open_camera, open_stereo_camera, robot_center_horizontal_error
from .runtime import RobotRuntime
from .maintenance_site import MaintenanceSite
from .game_status_site import GameStatusSite
from .vision_worker import VisionWorker
from .targeting import PanelTarget, choose_panel_target
from .ball_mechanism import BallMechanism

__all__ = [
    "AT_NEUTRAL_VALUE",
    "ATMotor",
    "at_address_from_can_id",
    "DEFAULT_MOTOR_DIRECTIONS",
    "DEFAULT_MOTOR_IDS",
    "DEFAULT_PYGAME_PROFILE",
    "AnalogStick",
    "Axis",
    "Button",
    "BallMechanism",
    "ControllerProfile",
    "ControllerState",
    "DualSenseMotionMapping",
    "MecanumMixer",
    "MecanumRobot",
    "MotionCommand",
    "PySerialTransport",
    "PygameDualSense",
    "open_configured_dualsense",
    "TimedServo",
    "TimedServoConfig",
    "ATEncoderReader",
    "EncoderFeedback",
    "EncoderPositionServo",
    "PositionServoConfig",
    "WheelSpeeds",
    "build_enable_frame",
    "build_velocity_frame",
    "backward",
    "forward",
    "normalized_to_at_value",
    "stop",
    "strafe_left",
    "strafe_right",
    "turn_left",
    "turn_right",
    "ControlMode",
    "ModeController",
    "TimedMotion",
    "add_manual_command",
    "face_target_command",
    "AprilTagDetector",
    "OpenCVStereoCamera",
    "RDKMIPIStereoCamera",
    "RDKMIPICamera",
    "OpenCVSingleCamera",
    "open_camera",
    "open_stereo_camera",
    "TagObservation",
    "TagStore",
    "midpoint",
    "robot_center_horizontal_error",
    "RobotRuntime",
    "MaintenanceSite",
    "GameStatusSite",
    "VisionWorker",
    "PanelTarget",
    "choose_panel_target",
]
