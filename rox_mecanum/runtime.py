"""GAME1/GAME2で共通の実機起動・停止処理。"""

from __future__ import annotations

from dataclasses import dataclass
import hensuu

from .ball_mechanism import set_transport_pose, transport_pose_ready
from .controller import Button, PygameDualSense, open_configured_dualsense
from .mecanum import DualSenseMotionMapping, MecanumMixer, MotionCommand
from .serial_at import AT_NEUTRAL_VALUE, MecanumRobot, PySerialTransport


def _speed_span(percent: float) -> int:
    return int(round(AT_NEUTRAL_VALUE * max(0.0, min(100.0, float(percent))) / 100.0))


@dataclass
class RobotRuntime:
    """ゲーム実行に必要な既存ハードウェアをまとめる。"""

    controller: PygameDualSense
    transport: PySerialTransport
    mecanum: MecanumRobot
    servos: object
    mapping: DualSenseMotionMapping

    @classmethod
    def open(cls) -> "RobotRuntime":
        """モーターを安全停止状態で有効化し、サーボPIDを起動する。"""
        from servos import open_servos

        controller = open_configured_dualsense()
        transport = PySerialTransport.open(hensuu.serial_port, hensuu.serial_baud, minimum_interval=0.0008)
        mecanum = MecanumRobot(
            transport,
            motor_ids={"FL": 0x0C, "FR": 0x14, "RL": 0x1C, "RR": 0x24},
            motor_directions={"FL": 1.0, "FR": -1.0, "RL": 1.0, "RR": -1.0},
            mixer=MecanumMixer(rotation_gain=0.22),
            speed_span=_speed_span(hensuu.mecanum_speed_percent),
            acceleration_per_second=hensuu.mecanum_acceleration_percent_per_sec / 100.0,
            deceleration_per_second=hensuu.mecanum_deceleration_percent_per_sec / 100.0,
        )
        servos = open_servos(transport=transport)
        try:
            mecanum.enable_all(retries=3, interval=0.05)
            servos.attach()
            # catch/liftをそれぞれ機械ストッパーまで自動で動かし、mechPosの変化が
            # 止まった位置を0度として登録する。保存済みの原点は使わない。
            servos.home_to_stop(
                "catch",
                speed_percent=hensuu.catch_homing_speed_percent,
                direction=hensuu.catch_homing_direction,
                stillness_deg=hensuu.catch_homing_stillness_deg,
                stillness_sec=hensuu.catch_homing_stillness_sec,
                timeout_sec=hensuu.catch_homing_timeout_sec,
            )
            servos.home_to_stop(
                "lift",
                speed_percent=hensuu.lift_homing_speed_percent,
                direction=hensuu.lift_homing_direction,
                stillness_deg=hensuu.lift_homing_stillness_deg,
                stillness_sec=hensuu.lift_homing_stillness_sec,
                timeout_sec=hensuu.lift_homing_timeout_sec,
            )
            # 原点登録した現在位置を目標にしてからPIDを開始する。
            servos.hold_all_current()
            servos.start_pid()
            return cls(
                controller=controller,
                transport=transport,
                mecanum=mecanum,
                servos=servos,
                mapping=DualSenseMotionMapping(
                    deadzone=hensuu.mecanum_deadzone,
                    response_exponent=hensuu.mecanum_response_exponent,
                    rotation_enable=Button.R2 if hensuu.mecanum_rotation_requires_r2 else None,
                    invert_forward=hensuu.mecanum_invert_forward_input,
                ),
            )
        except Exception:
            servos.close()
            transport.close()
            controller.close()
            raise

    def manual_command(self, state: object) -> MotionCommand:
        return self.mapping.command(state)

    def set_ball_transport_pose(self) -> None:
        """ボールを地面に付けて保持したまま移動する共通姿勢にする。"""
        set_transport_pose(self.servos)

    def ball_transport_pose_ready(self) -> bool:
        """地面保持姿勢へ両方の機構が到達した時だけTrue。"""
        return transport_pose_ready(self.servos)

    def emergency_stop(self) -> None:
        self.mecanum.stop()
        self.servos.release()

    def close(self) -> None:
        try:
            self.emergency_stop()
        finally:
            self.servos.close()
            self.transport.close()
            self.controller.close()
