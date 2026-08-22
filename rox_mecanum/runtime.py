"""GAME1/GAME2で共通の実機起動・停止処理。"""

from __future__ import annotations

from dataclasses import dataclass
from time import monotonic

import hensuu

from .controller import Button, PygameDualSense
from .mecanum import DualSenseMotionMapping, MecanumMixer, MotionCommand
from .serial_at import AT_NEUTRAL_VALUE, MecanumRobot, PySerialTransport
from .solenoid import RDKSolenoid


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
    solenoid: RDKSolenoid | None = None
    solenoid_until: float = 0.0

    @classmethod
    def open(cls, *, with_solenoid: bool) -> "RobotRuntime":
        """モーターを安全停止状態で有効化し、サーボPIDを起動する。"""
        from servos import open_servos

        controller = PygameDualSense.open()
        transport = PySerialTransport.open(hensuu.serial_port, hensuu.serial_baud, minimum_interval=0.0008)
        mecanum = MecanumRobot(
            transport,
            motor_ids={"FL": 0x0C, "FR": 0x14, "RL": 0x1C, "RR": 0x24},
            motor_directions={"FL": 1.0, "FR": -1.0, "RL": 1.0, "RR": -1.0},
            mixer=MecanumMixer(rotation_gain=0.22),
            speed_span=_speed_span(hensuu.mecanum_speed_percent),
        )
        servos = open_servos(transport=transport)
        solenoid = RDKSolenoid(hensuu.solenoid_pin) if with_solenoid else None
        try:
            mecanum.enable_all(retries=3, interval=0.05)
            servos.attach()
            print("catch/liftを機械的な0度へ合わせてから Enter を押してください")
            input()
            servos.home_from_feedback()
            servos.start_pid()
            return cls(
                controller=controller,
                transport=transport,
                mecanum=mecanum,
                servos=servos,
                solenoid=solenoid,
                mapping=DualSenseMotionMapping(
                    deadzone=0.08,
                    rotation_enable=Button.R2 if hensuu.mecanum_rotation_requires_r2 else None,
                ),
            )
        except Exception:
            if solenoid is not None:
                solenoid.close()
            servos.close()
            transport.close()
            controller.close()
            raise

    def manual_command(self, state: object) -> MotionCommand:
        return self.mapping.command(state)

    def set_ball_transport_pose(self) -> None:
        """ボールを地面に付けて保持したまま移動する共通姿勢にする。"""
        self.servos.catch.write(hensuu.catch_ball_hold_angle)
        self.servos.lift.write(hensuu.lift_ball_ground_angle)

    def ball_transport_pose_ready(self) -> bool:
        """地面保持姿勢へ両方の機構が到達した時だけTrue。"""
        return self.servos.catch.is_at_target() and self.servos.lift.is_at_target()

    def fire(self) -> None:
        if self.solenoid is None:
            raise RuntimeError("このゲームではソレノイドを使いません")
        self.solenoid.on()
        self.solenoid_until = monotonic() + float(hensuu.solenoid_time_sec)

    def update_outputs(self) -> None:
        if self.solenoid is not None and self.solenoid_until and monotonic() >= self.solenoid_until:
            self.solenoid.off()
            self.solenoid_until = 0.0

    def emergency_stop(self) -> None:
        self.mecanum.stop()
        self.servos.release()
        if self.solenoid is not None:
            self.solenoid.off()
        self.solenoid_until = 0.0

    def close(self) -> None:
        try:
            self.emergency_stop()
        finally:
            self.servos.close()
            if self.solenoid is not None:
                self.solenoid.close()
            self.transport.close()
            self.controller.close()
