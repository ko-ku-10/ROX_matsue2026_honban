"""ATシリアル経由のエンコーダー読取りと、位置を保持するPIDサーボ。"""

from __future__ import annotations

from dataclasses import dataclass, replace
from time import monotonic
from typing import Mapping

from .serial_at import ATMotor, PySerialTransport

ENCODER_REGISTER = 0x7019
COUNTS_PER_REV = 65536


def build_encoder_read_command(motor_address: int) -> bytes:
    """レジスタ0x7019（16-bitエンコーダー）を読む17バイトATフレーム。"""
    return bytes((
        0x41, 0x54, 0x90, 0x07, 0xE8, int(motor_address), 0x08,
        ENCODER_REGISTER & 0xFF, ENCODER_REGISTER >> 8,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0D, 0x0A,
    ))


@dataclass(frozen=True)
class EncoderFeedback:
    name: str
    address: int
    count: int
    received_at: float


class ATEncoderReader:
    """AT応答を分解し、登録済みモーターのエンコーダー値を取得する。"""

    def __init__(self, transport: PySerialTransport, motor_addresses: Mapping[str, int]) -> None:
        self.transport = transport
        self.addresses = dict(motor_addresses)
        self._by_address = {address: name for name, address in self.addresses.items()}
        self._by_status_id = {address + 3: name for name, address in self.addresses.items()}
        self._buffer = bytearray()

    def request_all(self) -> None:
        for address in self.addresses.values():
            self.transport.write(build_encoder_read_command(address))

    def poll(self, now: float | None = None) -> list[EncoderFeedback]:
        self._buffer.extend(self.transport.read_available())
        timestamp = monotonic() if now is None else now
        values: list[EncoderFeedback] = []
        for _, can_id, address, data in _take_at_frames(self._buffer):
            name = self._by_status_id.get(can_id) or self._by_status_id.get(can_id & 0xFF) or self._by_address.get(address)
            if name is not None and len(data) >= 2:
                values.append(EncoderFeedback(name, self.addresses[name], int.from_bytes(data[:2], "little"), timestamp))
        return values


@dataclass(frozen=True)
class PositionServoConfig:
    min_angle: float
    max_angle: float
    counts_per_degree: float
    kp: float = 0.015
    kd: float = 0.0
    max_speed: float = 0.20
    tolerance_deg: float = 0.5
    direction: int = 1
    ki: float = 0.0
    integral_limit: float = 30.0

    def __post_init__(self) -> None:
        if self.max_angle <= self.min_angle or self.counts_per_degree <= 0.0:
            raise ValueError("角度範囲と counts_per_degree を確認してください")
        if self.kp < 0.0 or self.ki < 0.0 or self.kd < 0.0 or not 0.0 < self.max_speed <= 1.0:
            raise ValueError("PIDとmax_speedの値を確認してください")
        if self.tolerance_deg < 0.0 or self.integral_limit < 0.0:
            raise ValueError("tolerance_deg と integral_limit は0以上にしてください")
        if self.direction not in (-1, 1):
            raise ValueError("direction は 1 または -1 にしてください")


class EncoderPositionServo:
    """エンコーダー位置を読み、外力でずれたときも目標位置へ戻すサーボ。"""

    def __init__(self, motor: ATMotor, config: PositionServoConfig) -> None:
        self.motor = motor
        self.config = config
        self.target_angle = 0.0
        self.current_angle: float | None = None
        self._home_unwrapped: int | None = None
        self._last_raw: int | None = None
        self._unwrapped = 0
        self._last_error = 0.0
        self._last_update: float | None = None
        self.last_feedback_at: float | None = None
        self._integral = 0.0
        self._holding = True
        self.last_command = 0.0

    def enable(self, retries: int = 3) -> None:
        for _ in range(retries):
            self.motor.enable()

    def set_home(self, raw_count: int, angle: float = 0.0) -> None:
        self._last_raw = int(raw_count)
        self._unwrapped = int(raw_count)
        self._home_unwrapped = self._unwrapped
        self.current_angle = float(angle)
        self.target_angle = self._limit(angle)
        self._last_error = 0.0
        self._last_update = None
        self.last_feedback_at = None
        self._integral = 0.0
        self._holding = True

    def write(self, angle: float) -> float:
        """目標角度を指定し、以後その位置をPIDで保持する。Arduino Servoの ``write`` 相当。"""
        self.target_angle = self._limit(angle)
        self._holding = True
        return self.target_angle

    def read(self) -> float | None:
        """エンコーダーから得た現在角度を返す。"""
        return self.current_angle

    def hold(self) -> None:
        """現在の目標角度を連続PIDで保持する。"""
        self._holding = True

    def pid_on(self) -> None:
        """PID保持をオンにする。``hold()`` の分かりやすい別名。"""
        self.hold()

    def hold_current(self) -> float:
        """今いる角度を新しい目標にして保持する。"""
        if self.current_angle is None:
            raise RuntimeError("先にエンコーダー値を受信して set_home() を呼んでください")
        return self.write(self.current_angle)

    def set_pid(
        self,
        *,
        kp: float | None = None,
        ki: float | None = None,
        kd: float | None = None,
        max_speed: float | None = None,
        tolerance_deg: float | None = None,
    ) -> PositionServoConfig:
        """PID設定を実行中に変更する。指定しなかった値は維持する。"""
        changes = {
            key: value
            for key, value in {
                "kp": kp,
                "ki": ki,
                "kd": kd,
                "max_speed": max_speed,
                "tolerance_deg": tolerance_deg,
            }.items()
            if value is not None
        }
        self.config = replace(self.config, **changes)
        # 調整値の変更前にたまったI項やD項を持ち越さない。
        self._integral = 0.0
        self._last_error = 0.0
        self._last_update = None
        return self.config

    def release(self) -> None:
        """PID保持を解除し、モーターを停止する。"""
        self._holding = False
        self._integral = 0.0
        self.last_command = 0.0
        self.motor.stop()

    def pid_off(self) -> None:
        """PID保持をオフにして、モーターを自由にする。``release()`` の別名。"""
        self.release()

    @property
    def pid_enabled(self) -> bool:
        """PID保持がオンかどうか。"""
        return self._holding

    def is_at_target(self) -> bool:
        """現在角度が許容誤差内かを返す。"""
        return self.current_angle is not None and abs(self.target_angle - self.current_angle) <= self.config.tolerance_deg

    def update(self, raw_count: int, now: float) -> float | None:
        self._update_angle(raw_count)
        if self.current_angle is None:
            return None
        if not self._holding:
            # release() で停止フレームはすでに1回送っている。
            # 以後はエンコーダーだけ読み、不要な停止フレームを連続送信しない。
            return 0.0
        error = self.target_angle - self.current_angle
        # 目標の近くでは出力を止める。細かい測定値の揺れによる
        # 正逆転の繰り返し（ガタガタ）を防ぎ、範囲外にずれた時だけ再補正する。
        if abs(error) <= self.config.tolerance_deg:
            self._integral = 0.0
            self._last_error = 0.0
            self._last_update = now
            self.last_feedback_at = now
            self.last_command = 0.0
            self.motor.set_velocity(0.0, force=True)
            return 0.0
        dt = 0.0 if self._last_update is None else max(0.001, now - self._last_update)
        derivative = 0.0 if dt == 0.0 else (error - self._last_error) / dt
        if dt > 0.0:
            self._integral = max(
                -self.config.integral_limit,
                min(self.config.integral_limit, self._integral + error * dt),
            )
        speed = self.config.kp * error + self.config.ki * self._integral + self.config.kd * derivative
        speed = max(-self.config.max_speed, min(self.config.max_speed, speed))
        self.motor.set_velocity(speed * self.config.direction, force=True)
        self._last_error = error
        self._last_update = now
        self.last_feedback_at = now
        self.last_command = speed
        return speed

    def stop(self) -> None:
        self.release()

    def status(self) -> dict[str, float | bool | None]:
        return {
            "current_angle": self.current_angle,
            "target_angle": self.target_angle,
            "holding": self._holding,
            "at_target": self.is_at_target(),
            "command": self.last_command,
        }

    def _update_angle(self, raw_count: int) -> None:
        raw_count = int(raw_count)
        if self._last_raw is None:
            self._last_raw = raw_count
            self._unwrapped = raw_count
        else:
            delta = raw_count - self._last_raw
            if delta > COUNTS_PER_REV // 2:
                delta -= COUNTS_PER_REV
            elif delta < -COUNTS_PER_REV // 2:
                delta += COUNTS_PER_REV
            self._unwrapped += delta
            self._last_raw = raw_count
        if self._home_unwrapped is not None:
            self.current_angle = (self._unwrapped - self._home_unwrapped) / self.config.counts_per_degree

    def _limit(self, angle: float) -> float:
        return max(self.config.min_angle, min(self.config.max_angle, float(angle)))


def _take_at_frames(buffer: bytearray) -> list[tuple[int, int, int, bytes]]:
    frames = []
    while True:
        start = buffer.find(b"AT")
        if start < 0:
            buffer.clear()
            break
        if start:
            del buffer[:start]
        if len(buffer) < 9:
            break
        data_length = buffer[6]
        total = 7 + data_length + 2
        if len(buffer) < total:
            break
        if buffer[total - 2:total] != b"\r\n":
            del buffer[0]
            continue
        frames.append((buffer[2], (buffer[3] << 8) | buffer[4], buffer[5], bytes(buffer[7:7 + data_length])))
        del buffer[:total]
    return frames
