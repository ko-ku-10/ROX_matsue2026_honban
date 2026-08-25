"""RobStride EDULITE05 用 AT シリアルフレームとメカナム実機アダプター。"""

from __future__ import annotations

from dataclasses import dataclass, field
from time import monotonic, sleep
from threading import Lock
from typing import Mapping, Protocol

from .mecanum import MecanumMixer, MotionCommand, WheelSpeeds


AT_NEUTRAL_VALUE = 0x7FFF
DEFAULT_MOTOR_IDS: Mapping[str, int] = {"FL": 0x0C, "FR": 0x14, "RL": 0x1C, "RR": 0x24}
DEFAULT_MOTOR_DIRECTIONS: Mapping[str, float] = {"FL": 1.0, "FR": -1.0, "RL": 1.0, "RR": -1.0}


def at_address_from_can_id(can_id: int) -> int:
    """CAN IDを、このATシリアル変換器用の宛先値へ変換する。

    例: CAN ID 5 は ``0x2C``、CAN ID 6 は ``0x34`` になる。
    """
    can_id = int(can_id)
    if not 0 <= can_id <= 31:
        raise ValueError("can_id は 0〜31 の範囲にしてください")
    return (can_id << 3) + 4


class ByteTransport(Protocol):
    """AT フレームを書き込める最小限のインターフェース。"""

    def write(self, data: bytes) -> None: ...


def build_enable_frame(motor_id: int) -> bytes:
    """指定モーターを有効化する17バイトの AT フレームを作る。"""
    return bytes((
        0x41, 0x54, 0x20, 0x07, 0xE8, _motor_id(motor_id),
        0x08, 0x00, 0xC4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0D, 0x0A,
    ))


def normalized_to_at_value(speed: float, speed_span: int = AT_NEUTRAL_VALUE) -> int:
    """-1.0〜+1.0 の速度を AT の16ビット速度値に変換する。"""
    span = max(0, min(AT_NEUTRAL_VALUE, int(speed_span)))
    speed = max(-1.0, min(1.0, float(speed)))
    return max(0, min(0xFFFF, AT_NEUTRAL_VALUE + int(round(speed * span))))


def build_velocity_frame(
    motor_id: int,
    speed: float,
    speed_span: int = AT_NEUTRAL_VALUE,
) -> bytes:
    """正規化速度を指定する17バイトの AT フレームを作る。"""
    value = normalized_to_at_value(speed, speed_span)
    direction = 0x00 if value == AT_NEUTRAL_VALUE else 0x01
    return bytes((
        0x41, 0x54, 0x90, 0x07, 0xE8, _motor_id(motor_id),
        0x08, 0x05, 0x70, 0x00, 0x00, 0x07, direction,
        (value >> 8) & 0xFF, value & 0xFF, 0x0D, 0x0A,
    ))


class PySerialTransport:
    """pyserial を使う実機向けトランスポート。

    pyserial は ``open`` 時のみ必要なので、計算・テスト用途では不要です。
    """

    def __init__(self, serial_port: object, minimum_interval: float = 0.0008):
        if minimum_interval < 0.0:
            raise ValueError("minimum_interval must be non-negative")
        self._serial_port = serial_port
        self._minimum_interval = minimum_interval
        self._last_write_at = 0.0
        self._write_lock = Lock()

    @classmethod
    def open(
        cls,
        port: str,
        baudrate: int = 921600,
        minimum_interval: float = 0.0008,
        timeout: float = 0.01,
    ) -> "PySerialTransport":
        """指定ポートを開く。``pip install 'rox-mecanum[serial]'`` が必要。"""
        try:
            import serial
        except ImportError as error:  # pragma: no cover - 実機依存
            raise RuntimeError("pyserial が必要です: pip install 'rox-mecanum[serial]'") from error
        return cls(serial.Serial(port=port, baudrate=baudrate, timeout=timeout), minimum_interval)

    def write(self, data: bytes) -> None:
        with self._write_lock:
            delay = self._minimum_interval - (monotonic() - self._last_write_at)
            if delay > 0.0:
                sleep(delay)
            self._serial_port.write(data)
            self._last_write_at = monotonic()

    def close(self) -> None:
        self._serial_port.close()

    def read_available(self) -> bytes:
        """到着済みの受信データをすべて読む。エンコーダーフィードバック用。"""
        waiting = int(getattr(self._serial_port, "in_waiting", 0))
        return bytes(self._serial_port.read(waiting)) if waiting else b""


@dataclass
class ATMotor:
    """1台の EDULITE05 を扱う軽量なモータークラス。"""

    transport: ByteTransport
    motor_id: int
    speed_span: int = AT_NEUTRAL_VALUE
    zero_hold_band: float = 0.06
    _last_speed: float | None = field(default=None, init=False, repr=False)

    def enable(self) -> None:
        self.transport.write(build_enable_frame(self.motor_id))

    def set_velocity(self, speed: float, *, force: bool = False) -> None:
        """速度を送信する。小さい速度は安全のため中立にする。"""
        speed = max(-1.0, min(1.0, float(speed)))
        if abs(speed) < self.zero_hold_band:
            speed = 0.0
        if not force and speed == self._last_speed:
            return
        self.transport.write(build_velocity_frame(self.motor_id, speed, self.speed_span))
        self._last_speed = speed

    def stop(self) -> None:
        self.set_velocity(0.0, force=True)


class MecanumRobot:
    """ミキサー計算結果を4台の EDULITE05 へ送信する実機アダプター。"""

    def __init__(
        self,
        transport: ByteTransport,
        *,
        motor_ids: Mapping[str, int] = DEFAULT_MOTOR_IDS,
        motor_directions: Mapping[str, float] = DEFAULT_MOTOR_DIRECTIONS,
        mixer: MecanumMixer | None = None,
        speed_span: int = AT_NEUTRAL_VALUE,
        acceleration_per_second: float | None = None,
    ) -> None:
        missing = {"FL", "FR", "RL", "RR"} - set(motor_ids)
        if missing:
            raise ValueError(f"motor_ids is missing: {sorted(missing)}")
        if acceleration_per_second is not None and acceleration_per_second <= 0.0:
            raise ValueError("acceleration_per_second は0より大きくしてください")
        self.mixer = mixer or MecanumMixer()
        self.motor_directions = dict(motor_directions)
        # Noneなら従来どおり即座に速度を変える。実機では加速制限を指定する。
        self.acceleration_per_second = acceleration_per_second
        self._last_wheel_speeds = {name: 0.0 for name in ("FL", "FR", "RL", "RR")}
        self._last_drive_at: float | None = None
        self.motors = {
            name: ATMotor(transport, motor_ids[name], speed_span=speed_span)
            for name in ("FL", "FR", "RL", "RR")
        }

    def enable_all(self, retries: int = 3, interval: float = 0.05) -> None:
        """全モーターを有効化し、停止状態にする。"""
        if retries < 1:
            raise ValueError("retries must be at least 1")
        for _ in range(retries):
            for motor in self.motors.values():
                motor.enable()
            if interval > 0.0:
                sleep(interval)
        self.stop()

    def drive(self, command: MotionCommand) -> WheelSpeeds:
        """移動指令を送り、取付方向補正後の4輪出力を返す。"""
        target = self.mixer.mix(command).with_motor_directions(self.motor_directions)
        target_values = target.as_dict()
        speeds = self._apply_acceleration_limit(target_values)
        for name, speed in speeds.as_dict().items():
            self.motors[name].set_velocity(speed)
        return speeds

    def stop(self) -> None:
        """全輪に停止指令を送る。"""
        for motor in self.motors.values():
            motor.stop()
        # OPTIONSなどで止めた後の次の発進も、必ず0からゆっくり始める。
        self._last_wheel_speeds = {name: 0.0 for name in ("FL", "FR", "RL", "RR")}
        self._last_drive_at = None

    def _apply_acceleration_limit(self, target: Mapping[str, float]) -> WheelSpeeds:
        """目標速度へ急に変えず、1輪ずつ少しずつ近づける。"""
        if self.acceleration_per_second is None:
            return WheelSpeeds(target["FL"], target["FR"], target["RL"], target["RR"])

        now = monotonic()
        # 最初の1回は制御周期50Hzを仮定する。起動から時間が経っていても急発進しない。
        dt = 0.02 if self._last_drive_at is None else min(0.10, now - self._last_drive_at)
        maximum_change = self.acceleration_per_second * max(0.0, dt)
        values: dict[str, float] = {}

        for name in ("FL", "FR", "RL", "RR"):
            previous = self._last_wheel_speeds[name]
            difference = target[name] - previous
            change = max(-maximum_change, min(maximum_change, difference))
            values[name] = previous + change

        self._last_wheel_speeds = values
        self._last_drive_at = now
        return WheelSpeeds(values["FL"], values["FR"], values["RL"], values["RR"])


def _motor_id(motor_id: int) -> int:
    if not 0 <= int(motor_id) <= 0xFF:
        raise ValueError("motor_id must be an unsigned byte")
    return int(motor_id)
