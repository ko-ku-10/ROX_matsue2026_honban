"""RobStride EDULITE05 の私有CANプロトコル送信部品。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


POSITION_MIN_RAD = -12.57
POSITION_MAX_RAD = 12.57
VELOCITY_MIN_RAD_PER_SEC = -50.0
VELOCITY_MAX_RAD_PER_SEC = 50.0
KP_MIN = 0.0
KP_MAX = 500.0
KD_MIN = 0.0
KD_MAX = 5.0


class CanSender(Protocol):
    def send(self, message: object) -> object: ...


@dataclass(frozen=True)
class CanCommand:
    """python-canへ渡す直前の、CANフレーム内容。"""

    arbitration_id: int
    data: bytes
    is_extended_id: bool = True


def build_private_arbitration_id(communication_type: int, motor_id: int, host_id: int = 0) -> int:
    """RobStride私有プロトコルの29ビット拡張CAN IDを作る。"""
    if not 0 <= int(communication_type) <= 0x1F:
        raise ValueError("communication_type must be 0..31")
    _byte(motor_id, "motor_id")
    _byte(host_id, "host_id")
    return (int(communication_type) << 24) | (int(host_id) << 8) | int(motor_id)


def build_enable_command(motor_id: int, host_id: int = 0) -> CanCommand:
    """通信タイプ3：モーターを有効化する。"""
    return _command(3, motor_id, host_id, bytes(8))


def build_disable_command(motor_id: int, host_id: int = 0) -> CanCommand:
    """通信タイプ4：モーターを停止・無効化する。"""
    return _command(4, motor_id, host_id, bytes(8))


def build_operation_control_command(
    motor_id: int,
    *,
    position_rad: float = 0.0,
    velocity_rad_per_sec: float = 0.0,
    kp: float = 0.0,
    kd: float = 0.0,
    host_id: int = 0,
) -> CanCommand:
    """通信タイプ1：運転制御フレームを作る（位置・速度・Kp・Kd）。"""
    data = b"".join(
        _float_to_uint16(value, minimum, maximum).to_bytes(2, "big")
        for value, minimum, maximum in (
            (position_rad, POSITION_MIN_RAD, POSITION_MAX_RAD),
            (velocity_rad_per_sec, VELOCITY_MIN_RAD_PER_SEC, VELOCITY_MAX_RAD_PER_SEC),
            (kp, KP_MIN, KP_MAX),
            (kd, KD_MIN, KD_MAX),
        )
    )
    return _command(1, motor_id, host_id, data)


def build_active_report_command(motor_id: int, enabled: bool, host_id: int = 0) -> CanCommand:
    """通信タイプ24：10ms周期のエンコーダー能動送信をオン／オフする。"""
    return _command(24, motor_id, host_id, bytes((1 if enabled else 0, 0, 0, 0, 0, 0, 0, 0)))


class RobStrideCanMotor:
    """CANableで直接速度を送れるEDULITE05用モーター出力。

    ``EncoderServo`` にそのまま渡せる。位置PIDは既存ライブラリ側で計算し、
    このクラスはタイプ1の速度フィールドへCAN送信する。
    """

    def __init__(self, bus: CanSender, motor_id: int, *, host_id: int = 0) -> None:
        self._bus = bus
        self.motor_id = _byte(motor_id, "motor_id")
        self.host_id = _byte(host_id, "host_id")
        self._last_speed: float | None = None

    def enable(self) -> None:
        self._send(build_enable_command(self.motor_id, self.host_id))

    def disable(self) -> None:
        self._send(build_disable_command(self.motor_id, self.host_id))
        self._last_speed = None

    def enable_active_reporting(self, enabled: bool = True) -> None:
        self._send(build_active_report_command(self.motor_id, enabled, self.host_id))

    def set_velocity(self, speed: float, *, force: bool = False) -> None:
        """-1.0〜+1.0を-50〜+50rad/sのCAN速度指令へ変換して送る。"""
        speed = max(-1.0, min(1.0, float(speed)))
        if not force and speed == self._last_speed:
            return
        self._send(build_operation_control_command(
            self.motor_id,
            velocity_rad_per_sec=speed * VELOCITY_MAX_RAD_PER_SEC,
            host_id=self.host_id,
        ))
        self._last_speed = speed

    def stop(self) -> None:
        """速度ゼロを送る。無効化はしないため、次の目標をそのまま受け付けられる。"""
        self.set_velocity(0.0, force=True)

    def _send(self, command: CanCommand) -> None:
        try:
            import can
        except ImportError as error:  # pragma: no cover - 実機依存
            raise RuntimeError("python-can が必要です: pip install 'rox-mecanum[can]'") from error
        self._bus.send(can.Message(
            arbitration_id=command.arbitration_id,
            data=command.data,
            is_extended_id=command.is_extended_id,
        ))


def _command(communication_type: int, motor_id: int, host_id: int, data: bytes) -> CanCommand:
    if len(data) != 8:
        raise ValueError("RobStride CAN command data must be 8 bytes")
    return CanCommand(build_private_arbitration_id(communication_type, motor_id, host_id), data)


def _float_to_uint16(value: float, minimum: float, maximum: float) -> int:
    clipped = max(minimum, min(maximum, float(value)))
    return round((clipped - minimum) * 65535.0 / (maximum - minimum))


def _byte(value: int, name: str) -> int:
    integer = int(value)
    if not 0 <= integer <= 0xFF:
        raise ValueError(f"{name} must be an unsigned byte")
    return integer
