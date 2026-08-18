"""SocketCANでRobStride系モーターのエンコーダーフィードバックを受信する部品。"""

from __future__ import annotations

from dataclasses import dataclass
from time import monotonic
from typing import Protocol


POSITION_MIN_RAD = -12.57
POSITION_MAX_RAD = 12.57
VELOCITY_MIN_RAD_PER_SEC = -50.0
VELOCITY_MAX_RAD_PER_SEC = 50.0
TORQUE_MIN_NM = -6.0
TORQUE_MAX_NM = 6.0


class CanBus(Protocol):
    """python-can互換の、1フレーム受信可能なバス。"""

    def recv(self, timeout: float | None = None) -> object | None: ...

    def shutdown(self) -> None: ...


@dataclass(frozen=True)
class MotorFeedback:
    """通信タイプ2のエンコーダー／状態フィードバック。"""

    motor_id: int
    host_id: int
    position_rad: float
    velocity_rad_per_sec: float
    torque_nm: float
    temperature_c: float

    @property
    def position_deg(self) -> float:
        return self.position_rad * 180.0 / 3.141592653589793

    @property
    def velocity_deg_per_sec(self) -> float:
        return self.velocity_rad_per_sec * 180.0 / 3.141592653589793


class CanEncoderReceiver:
    """MKS CANable等のSocketCANバスからエンコーダー値を受信する。

    モーターが返す通信タイプ2フレームを待ち、指定CAN IDの値だけ返す。
    受信した ``feedback.position_deg`` を ``EncoderServo.update()`` に渡せる。
    """

    def __init__(self, bus: CanBus) -> None:
        self._bus = bus

    @classmethod
    def open_socketcan(cls, channel: str = "can0") -> "CanEncoderReceiver":
        """Linux SocketCANインターフェースを開く。python-can が必要。"""
        try:
            import can
        except ImportError as error:  # pragma: no cover - 実機依存
            raise RuntimeError("python-can が必要です: pip install 'rox-mecanum[can]'") from error
        return cls(can.Bus(interface="socketcan", channel=channel))

    def read(self, motor_id: int, timeout: float = 0.02) -> MotorFeedback | None:
        """指定モーターの次のフィードバックを待つ。

        ``timeout`` 秒以内に来なければ ``None``。モーターは通常、制御指令への
        応答としてタイプ2フレームを返すため、速度・位置指令を定期送信している
        状態で呼ぶ。
        """
        target_id = _byte(motor_id, "motor_id")
        deadline = monotonic() + max(0.0, float(timeout))
        while True:
            remaining = max(0.0, deadline - monotonic())
            message = self._bus.recv(remaining)
            if message is None:
                return None
            feedback = decode_motor_feedback(message)
            if feedback is not None and feedback.motor_id == target_id:
                return feedback
            if monotonic() >= deadline:
                return None

    def close(self) -> None:
        self._bus.shutdown()


def decode_motor_feedback(message: object) -> MotorFeedback | None:
    """python-canのMessageからRobStride私有プロトコルのタイプ2を復号する。

    対象外フレームは ``None`` を返す。形式が壊れたタイプ2フレームは例外として
    扱い、誤ったエンコーダー値で機構を動かさないようにする。
    """
    if not bool(getattr(message, "is_extended_id", False)):
        return None
    arbitration_id = int(getattr(message, "arbitration_id"))
    communication_type = (arbitration_id >> 24) & 0x1F
    if communication_type != 0x02:
        return None

    data = bytes(getattr(message, "data"))
    if len(data) != 8:
        raise ValueError("RobStride type-2 feedback must contain exactly 8 data bytes")

    motor_id = (arbitration_id >> 8) & 0xFF
    host_id = arbitration_id & 0xFF
    return MotorFeedback(
        motor_id=motor_id,
        host_id=host_id,
        position_rad=_map_uint16(data[0:2], POSITION_MIN_RAD, POSITION_MAX_RAD),
        velocity_rad_per_sec=_map_uint16(data[2:4], VELOCITY_MIN_RAD_PER_SEC, VELOCITY_MAX_RAD_PER_SEC),
        torque_nm=_map_uint16(data[4:6], TORQUE_MIN_NM, TORQUE_MAX_NM),
        temperature_c=int.from_bytes(data[6:8], "big") / 10.0,
    )


def _map_uint16(raw: bytes, minimum: float, maximum: float) -> float:
    value = int.from_bytes(raw, "big")
    return minimum + value * (maximum - minimum) / 65535.0


def _byte(value: int, name: str) -> int:
    integer = int(value)
    if not 0 <= integer <= 0xFF:
        raise ValueError(f"{name} must be an unsigned byte")
    return integer
