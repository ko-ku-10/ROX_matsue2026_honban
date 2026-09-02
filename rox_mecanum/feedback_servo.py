"""ATシリアル経由のエンコーダー読取りと、位置を保持するPIDサーボ。"""

from __future__ import annotations

from dataclasses import dataclass, replace
from math import atan2, cos, degrees, isfinite, pi, sin
from struct import unpack
from time import monotonic
from typing import Mapping

from .serial_at import ATMotor, PySerialTransport

ENCODER_REGISTER = 0x7019
COUNTS_PER_REV = 65536


def build_encoder_read_command(motor_address: int) -> bytes:
    """RobStride type17でmechPos(0x7019)を読む17バイトATフレーム。

    RobStride公式AT変換器は、拡張CAN IDを3bit左へずらし、下位3bitへ
    ``0b100`` を付けて送る。従来の ``90 07 E8`` はtype18（書込み）なので
    読取りには使わない。
    """
    motor_id = _can_id_from_at_address(motor_address)
    ext_can_id = (0x11 << 24) | (0xFD << 16) | motor_id
    at_extended_id = (ext_can_id << 3) | 0x04
    return bytes((
        0x41, 0x54,
        (at_extended_id >> 24) & 0xFF,
        (at_extended_id >> 16) & 0xFF,
        (at_extended_id >> 8) & 0xFF,
        at_extended_id & 0xFF,
        0x08,
        ENCODER_REGISTER & 0xFF, ENCODER_REGISTER >> 8,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0D, 0x0A,
    ))


@dataclass(frozen=True)
class EncoderFeedback:
    name: str
    address: int
    count: int | None
    received_at: float
    # RobStride正式形式のmechPos(0x7019): load側の多回転機械角[rad]。
    # AT変換器が旧ステータス形式しか返さない場合はNoneになり、countを使う。
    position_rad: float | None = None
    # 受信したAT応答フレーム全体。診断時に通信内容をそのまま表示できる。
    raw_at_frame: bytes = b""


class ATEncoderReader:
    """AT応答を分解し、登録済みモーターのエンコーダー値を取得する。"""

    def __init__(self, transport: PySerialTransport, motor_addresses: Mapping[str, int]) -> None:
        self.transport = transport
        self.addresses = dict(motor_addresses)
        self._by_address = {address: name for name, address in self.addresses.items()}
        self._by_status_id = {address + 3: name for name, address in self.addresses.items()}
        self._buffer = bytearray()
        self._cycle_addresses = tuple(self.addresses.values())
        self._next_address_index = 0

    def request_all(self) -> None:
        for address in self.addresses.values():
            self.transport.write(build_encoder_read_command(address))

    def request(self, name: str) -> None:
        """指定した1台だけにmechPosを要求する。初期化時の確実な読取り用。"""
        try:
            address = self.addresses[name]
        except KeyError as error:
            raise ValueError(f"未登録のモーター名です: {name}") from error
        self.transport.write(build_encoder_read_command(address))

    def request_next(self) -> None:
        """登録済みモーターへ1台ずつ順番にmechPosを要求する。

        USB-AT変換器は短時間に複数の要求を送ると片方の応答を落とすことがある。
        1回のPID周期では1台だけ送ることで、2台を交互に確実に読む。
        """
        if not self._cycle_addresses:
            return
        address = self._cycle_addresses[self._next_address_index]
        self._next_address_index = (self._next_address_index + 1) % len(self._cycle_addresses)
        self.transport.write(build_encoder_read_command(address))

    def discard_pending(self) -> None:
        """前のモーター操作に対する古いAT応答を捨てる。

        原点合わせではcatchを止めた直後にliftへ切り替える。停止応答が残った
        ままだと、次の機構の正式mechPos応答を待つ処理へ混ざるため、切替時だけ
        受信バッファを空にする。
        """
        self._buffer.clear()
        self.transport.read_available()

    def poll(self, now: float | None = None) -> list[EncoderFeedback]:
        self._buffer.extend(self.transport.read_available())
        timestamp = monotonic() if now is None else now
        values: list[EncoderFeedback] = []
        for at_identifier, data in _take_at_frames(self._buffer):
            # ATの拡張フレームは ``(29bit CAN ID << 3) | 0b100`` で格納される。
            ext_can_id = at_identifier >> 3
            mode = (ext_can_id >> 24) & 0x1F
            position_rad = _decode_mech_pos(data)

            if mode == 0x11 and position_rad is not None:
                # type17応答。実測したAT応答ではdata16の下位8bitがmotor CAN ID。
                data16 = (ext_can_id >> 8) & 0xFFFF
                candidate_ids = (data16 & 0xFF, data16 >> 8)
                name = next(
                    (self._by_address.get((motor_id << 3) + 4) for motor_id in candidate_ids
                     if self._by_address.get((motor_id << 3) + 4) is not None),
                    None,
                )
                count = None
            else:
                # 旧ATステータス形式も読み取り専用の表示用途として残す。
                can_id = (at_identifier >> 8) & 0xFFFF
                address = at_identifier & 0xFF
                name = self._by_status_id.get(can_id) or self._by_status_id.get(can_id & 0xFF) or self._by_address.get(address)
                count = int.from_bytes(data[:2], "little") if len(data) >= 2 else None

            if name is not None and len(data) >= 2:
                values.append(
                    EncoderFeedback(
                        name,
                        self.addresses[name],
                        count,
                        timestamp,
                        position_rad,
                        b"AT" + at_identifier.to_bytes(4, "big") + bytes((len(data),)) + data + b"\r\n",
                    )
                )
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
    # この時間を超えてmechPosが届かなければ、前回の速度指令を取り消す。
    feedback_timeout_sec: float = 0.25

    def __post_init__(self) -> None:
        if self.max_angle <= self.min_angle or self.counts_per_degree <= 0.0:
            raise ValueError("角度範囲と counts_per_degree を確認してください")
        if self.kp < 0.0 or self.ki < 0.0 or self.kd < 0.0 or not 0.0 < self.max_speed <= 1.0:
            raise ValueError("PIDとmax_speedの値を確認してください")
        if self.tolerance_deg < 0.0 or self.integral_limit < 0.0 or self.feedback_timeout_sec <= 0.0:
            raise ValueError("tolerance_deg、integral_limit、feedback_timeout_sec の値を確認してください")
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
        self._home_position_rad: float | None = None
        # mechPosは電源投入で回転回数だけ±2πずれることがある。
        # 1回転内の位相と、実行中の連続角度を分けて扱う。
        self._last_position_phase: float | None = None
        self._unwrapped_position_rad: float | None = None
        self._last_raw: int | None = None
        self._unwrapped = 0
        self._last_error = 0.0
        self._last_update: float | None = None
        self.last_feedback_at: float | None = None
        self._integral = 0.0
        # write()/hold_current()などの明示命令があるまで、絶対に出力しない。
        self._holding = False
        self.last_command = 0.0
        # エンコーダー応答断による停止フレームを、1回だけ送るための印。
        self._feedback_timed_out = False

    def enable(self, retries: int = 3) -> None:
        for _ in range(retries):
            self.motor.enable()

    def set_home(self, raw_count: int, angle: float = 0.0) -> None:
        self._last_raw = int(raw_count)
        self._unwrapped = int(raw_count)
        self._home_unwrapped = self._unwrapped
        self.current_angle = float(angle)
        self.target_angle = self._limit(angle)
        self._home_position_rad = None
        self._last_position_phase = None
        self._unwrapped_position_rad = None
        self._last_error = 0.0
        self._last_update = None
        self.last_feedback_at = None
        self._integral = 0.0
        self._holding = False
        self._feedback_timed_out = False
        self.motor.stop()

    def set_home_radians(self, position_rad: float, angle: float = 0.0) -> None:
        """RobStride正式のmechPos[rad]を基準に、現在位置を登録する。"""
        if not isfinite(position_rad):
            raise ValueError("position_rad は有限の値にしてください")
        # 電源を入れ直すと同じ物理位置が 0rad と 2πrad のように変わる。
        # 原点は1回転内の位相として保存し、±2πの差は無視する。
        phase = _normalize_phase_rad(position_rad)
        self._home_position_rad = phase
        self._last_position_phase = phase
        self._unwrapped_position_rad = phase
        self._home_unwrapped = None
        self._last_raw = None
        self.current_angle = float(angle)
        self.target_angle = self._limit(angle)
        self._last_error = 0.0
        self._last_update = None
        self.last_feedback_at = None
        self._integral = 0.0
        self._holding = False
        self._feedback_timed_out = False
        self.motor.stop()

    @property
    def home_position_rad(self) -> float | None:
        """登録済みの0度位置(mechPos[rad])。未登録ならNone。"""
        return self._home_position_rad

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

    def hold_here(self) -> float:
        """今いる実測位置を目標にしてPID保持を始める。``hold_current`` の別名。"""
        return self.hold_current()

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
        self._feedback_timed_out = False
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
        return self._update_control(now)

    def update_radians(self, position_rad: float, now: float) -> float | None:
        """RobStrideのmechPos[rad]でサーボ状態を更新する。"""
        if self._home_position_rad is None:
            return None
        if not isfinite(position_rad):
            return None
        phase = _normalize_phase_rad(position_rad)
        if self._last_position_phase is None or self._unwrapped_position_rad is None:
            # 起動直後は保存済み原点に最も近い1回転として解釈する。
            self._last_position_phase = self._home_position_rad
            self._unwrapped_position_rad = self._home_position_rad

        # 例: 6.2812rad → 0.0029rad のように境界をまたいでも、
        # 差分を-0.0049radとして扱う。実行中は複数回転も連続追跡できる。
        delta = _shortest_phase_delta_rad(phase - self._last_position_phase)
        self._unwrapped_position_rad += delta
        self._last_position_phase = phase
        self.current_angle = degrees(self._unwrapped_position_rad - self._home_position_rad)
        return self._update_control(now)

    def update_feedback(self, feedback: EncoderFeedback) -> float | None:
        """正式なmechPos応答でだけ更新する。未確認の旧AT生値では出力しない。"""
        if feedback.position_rad is not None and self._home_position_rad is not None:
            return self.update_radians(feedback.position_rad, feedback.received_at)
        return None

    def watchdog(self, now: float) -> bool:
        """角度応答が途切れた場合に停止する。

        PIDのオン状態と目標角度は残すため、通信が復帰すれば次の実測角度から
        自然に保持を再開する。角度を推定して補正することはしない。
        """
        if (
            self._holding
            and self.last_feedback_at is not None
            and now - self.last_feedback_at > self.config.feedback_timeout_sec
        ):
            # 応答断中に停止フレームを毎周期送ると、USB-CAN変換器を圧迫して
            # 角度応答の復帰まで妨げてしまう。最初の1回だけ安全停止する。
            if self._feedback_timed_out:
                return False
            self._integral = 0.0
            self._last_error = 0.0
            self._last_update = None
            self.last_command = 0.0
            self._feedback_timed_out = True
            self.motor.stop()
            return True
        return False

    def _update_control(self, now: float) -> float | None:
        if self.current_angle is None:
            return None
        self.last_feedback_at = now
        # 新しい正式mechPosを受信したので、応答断から復帰した。
        self._feedback_timed_out = False
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
            # ATMotorは前回と同じ速度を送らない。目標に着いた後も毎回
            # 停止フレームを送ると、mechPos要求・応答より送信が多くなり、
            # USB-AT変換器が応答を落とす原因になる。
            self.motor.set_velocity(0.0)
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
        # 同じ補正速度が続く時は、モーターが保持している速度指令を使う。
        # メカナムと同じく、値が変わった時だけ送信すればよい。
        self.motor.set_velocity(speed * self.config.direction)
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
            "feedback_age_sec": None if self.last_feedback_at is None else max(0.0, monotonic() - self.last_feedback_at),
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


def _decode_mech_pos(data: bytes) -> float | None:
    """RobStrideタイプ17応答からmechPos(0x7019)のfloat[rad]を読む。

    正式応答は data[0:2] がレジスタ番号、data[4:8] がlittle-endian float。
    AT変換器が別形式のステータスを返す場合はNoneとして、安全に従来形式へ
    フォールバックする。
    """
    if len(data) != 8 or int.from_bytes(data[:2], "little") != ENCODER_REGISTER:
        return None
    position_rad = unpack("<f", data[4:8])[0]
    return position_rad if isfinite(position_rad) else None


def _normalize_phase_rad(angle_rad: float) -> float:
    """任意のrad値を -π〜+π の1回転内位相へ正規化する。"""
    return atan2(sin(float(angle_rad)), cos(float(angle_rad)))


def _shortest_phase_delta_rad(delta_rad: float) -> float:
    """位相差を、最短方向の -π〜+π の差分へ正規化する。"""
    return _normalize_phase_rad(delta_rad)


def _can_id_from_at_address(address: int) -> int:
    """既存AT宛先値 ``(CAN ID << 3) + 4`` からCAN IDを戻す。"""
    address = int(address)
    if address < 4 or (address - 4) % 8:
        raise ValueError("AT宛先値が不正です")
    return (address - 4) // 8


def _take_at_frames(buffer: bytearray) -> list[tuple[int, bytes]]:
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
        # このAT変換器で扱うCANデータ長は0〜8だけ。壊れたヘッダーの大きな
        # 長さを信じて待ち続けると、後ろに届いた正式mechPosを永久に読めない。
        if data_length > 8:
            del buffer[0]
            continue
        total = 7 + data_length + 2
        if len(buffer) < total:
            break
        if buffer[total - 2:total] != b"\r\n":
            del buffer[0]
            continue
        at_identifier = int.from_bytes(buffer[2:6], "big")
        frames.append((at_identifier, bytes(buffer[7:7 + data_length])))
        del buffer[:total]
    return frames
