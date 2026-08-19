"""2台のエンコーダー付きモーターを、ArduinoのServo風にまとめて扱う部品。"""

from __future__ import annotations

from time import monotonic
from types import ModuleType
from typing import Any

from .can_feedback import CanBus, decode_motor_feedback
from .can_motor import RobStrideCanMotor
from .servo import EncoderServo, ServoConfig, ServoState


class ServoPair:
    """catch と lift を、セットアップ済みの2軸サーボとして扱う。

    通常の使い方::

        robot = ServoPair.open_from_hensuu(hensuu)
        robot.begin()                 # 有効化・CAN受信・原点登録を完了するまで待つ
        robot.catch.write(30)         # CAN ID 5を30度へ
        robot.lift.write(50)          # CAN ID 6を50度へ
        while True:
            robot.update()            # PIDとCAN受信を進める

    ``begin()`` は、最初に受信した各エンコーダー角度を 0度として登録する。
    機構を必ず決めた物理原点に置いてから呼ぶこと。
    """

    def __init__(
        self,
        catch: EncoderServo,
        lift: EncoderServo,
        bus: CanBus,
        *,
        catch_id: int,
        lift_id: int,
    ) -> None:
        if catch_id == lift_id:
            raise ValueError("catch_id and lift_id must be different")
        self.catch = catch
        self.lift = lift
        self._bus = bus
        self._catch_id = int(catch_id)
        self._lift_id = int(lift_id)

    @classmethod
    def open_from_hensuu(cls, settings: ModuleType | Any) -> "ServoPair":
        """hensuu.pyの設定だけで、CANable直結のID 5/6 PIDサーボを作る。"""
        try:
            import can
        except ImportError as error:  # pragma: no cover - 実機依存
            raise RuntimeError("python-can が必要です: pip install 'rox-mecanum[can]'") from error

        try:
            bus = can.Bus(interface="socketcan", channel=settings.mechanism_can_channel)
        except Exception:
            raise

        host_id = getattr(settings, "mechanism_host_id", 0)
        catch_motor = RobStrideCanMotor(bus, settings.catch_motor_id, host_id=host_id)
        lift_motor = RobStrideCanMotor(bus, settings.lift_motor_id, host_id=host_id)
        return cls(
            EncoderServo(catch_motor, _servo_config(settings, "catch")),
            EncoderServo(lift_motor, _servo_config(settings, "lift")),
            bus,
            catch_id=settings.catch_motor_id,
            lift_id=settings.lift_motor_id,
        )

    @property
    def is_ready(self) -> bool:
        """2軸ともエンコーダーを受信して原点登録済みならTrue。"""
        return self.catch.is_homed and self.lift.is_homed

    def begin(self, home_timeout: float = 5.0) -> None:
        """2軸を有効化し、両方の最初のCAN値を原点として登録する。"""
        self.catch.attach()
        self.lift.attach()
        deadline = monotonic() + max(0.0, float(home_timeout))
        while not self.is_ready:
            remaining = deadline - monotonic()
            if remaining <= 0.0:
                self.stop()
                raise TimeoutError("catch/lift のCANエンコーダー値を受信できませんでした")
            self.update(timeout=min(0.05, remaining))

    def update(self, timeout: float = 0.0) -> ServoState | None:
        """CANを1件読み、該当する軸のPIDを1周期だけ更新する。"""
        message = self._bus.recv(max(0.0, float(timeout)))
        if message is None:
            return None
        feedback = decode_motor_feedback(message)
        if feedback is None:
            return None
        if feedback.motor_id == self._catch_id:
            return self._update_axis(self.catch, feedback.position_deg)
        if feedback.motor_id == self._lift_id:
            return self._update_axis(self.lift, feedback.position_deg)
        return None

    def stop(self) -> None:
        """catch と lift の両方へ停止指令を送る。"""
        self.catch.stop()
        self.lift.stop()

    def close(self) -> None:
        """停止・無効化してからCAN接続を閉じる。"""
        self.stop()
        if isinstance(self.catch.motor, RobStrideCanMotor):
            self.catch.motor.disable()
        if isinstance(self.lift.motor, RobStrideCanMotor):
            self.lift.motor.disable()
        self._bus.shutdown()

    @staticmethod
    def _update_axis(servo: EncoderServo, raw_encoder_deg: float) -> ServoState | None:
        if not servo.is_homed:
            servo.set_home(raw_encoder_deg)
            return None
        return servo.loop(raw_encoder_deg)


def _servo_config(settings: ModuleType | Any, name: str) -> ServoConfig:
    """hensuu.pyの ``catch_*`` / ``lift_*`` 設定をServoConfigへ変換する。"""
    get = lambda key: getattr(settings, f"{name}_{key}")
    return ServoConfig(
        min_position_deg=get("min_position_deg"),
        max_position_deg=get("max_position_deg"),
        max_command=get("max_command"),
        position_kp=get("pid_kp"),
        position_ki=get("pid_ki"),
        position_kd=get("pid_kd"),
        integral_limit=get("pid_integral_limit"),
        command_accel_per_sec=get("command_accel_per_sec"),
        tolerance_deg=get("tolerance_deg"),
        direction=get("encoder_direction"),
    )
