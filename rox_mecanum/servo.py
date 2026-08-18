"""エンコーダー付きモーターをサーボのように扱うための高水準API。

このモジュールは既存の ``ATMotor`` の速度出力を利用する。CANから受信した
エンコーダー角度を ``update()`` に渡すと、目標位置へ向かう安全な速度指令を
生成・送信する。CAN受信部は接続機器ごとに異なるため、ここからは分離している。
"""

from __future__ import annotations

from dataclasses import dataclass
from time import monotonic
from typing import Protocol


class VelocityOutput(Protocol):
    """正規化速度を送れるモーターの最小インターフェース。"""

    def enable(self) -> None: ...

    def set_velocity(self, speed: float, *, force: bool = False) -> None: ...

    def stop(self) -> None: ...


@dataclass(frozen=True)
class ServoConfig:
    """機構に合わせた安全範囲と操作感。

    角度はすべて度。``min_position_deg`` と ``max_position_deg`` は機構が物理的に
    動ける範囲より少し内側に設定する。初期値は必ず実機に合わせて変更すること。
    """

    min_position_deg: float = -90.0
    max_position_deg: float = 90.0
    max_command: float = 0.20
    position_kp: float = 0.015
    position_ki: float = 0.0
    position_kd: float = 0.0
    integral_limit: float = 100.0
    command_accel_per_sec: float = 1.0
    tolerance_deg: float = 1.0
    direction: int = 1

    def __post_init__(self) -> None:
        if self.min_position_deg >= self.max_position_deg:
            raise ValueError("min_position_deg must be smaller than max_position_deg")
        if not 0.0 < self.max_command <= 1.0:
            raise ValueError("max_command must be in the range (0.0, 1.0]")
        if self.position_kp < 0.0 or self.position_ki < 0.0 or self.position_kd < 0.0:
            raise ValueError("PID gains must be non-negative")
        if self.position_kp == 0.0 and self.position_ki == 0.0 and self.position_kd == 0.0:
            raise ValueError("at least one PID gain must be positive")
        if self.integral_limit < 0.0:
            raise ValueError("integral_limit must be non-negative")
        if self.command_accel_per_sec <= 0.0:
            raise ValueError("command_accel_per_sec must be positive")
        if self.tolerance_deg < 0.0:
            raise ValueError("tolerance_deg must be non-negative")
        if self.direction not in (-1, 1):
            raise ValueError("direction must be either -1 or 1")


@dataclass(frozen=True)
class ServoState:
    """最新の位置制御状態。"""

    position_deg: float
    target_deg: float
    error_deg: float
    command: float
    at_target: bool
    limited: bool
    p_term: float = 0.0
    i_term: float = 0.0
    d_term: float = 0.0


class EncoderServo:
    """エンコーダー値を使って位置を保つ、PID付きサーボ風の単体モーター制御。

    使用例::

        servo = EncoderServo(motor, ServoConfig(min_position_deg=0, max_position_deg=110))
        servo.enable()
        servo.set_home(raw_encoder_deg=42.3)
        servo.move_to(90)
        while True:
            state = servo.update(read_encoder_degrees())

    ``read_encoder_degrees()`` はCAN受信処理で得た、連続した実角度を返す関数を
    想定する。現在のATシリアル速度制御だけではこの受信値は得られない。
    """

    def __init__(self, motor: VelocityOutput, config: ServoConfig = ServoConfig()) -> None:
        self.motor = motor
        self.config = config
        self._zero_raw_deg: float | None = None
        self._position_deg: float | None = None
        self._target_deg = 0.0
        self._last_command = 0.0
        self._last_update_at: float | None = None
        self._integral = 0.0
        self._previous_error: float | None = None

    @property
    def position_deg(self) -> float | None:
        """原点基準の現在角度。まだエンコーダー未受信なら ``None``。"""
        return self._position_deg

    @property
    def target_deg(self) -> float:
        """現在の目標角度。常にソフトリミット内に収まる。"""
        return self._target_deg

    @property
    def is_homed(self) -> bool:
        return self._zero_raw_deg is not None

    def enable(self) -> None:
        """モーターを有効化する。原点合わせ前は停止状態のまま。"""
        self.motor.enable()
        self.motor.stop()

    def attach(self) -> None:
        """Arduino Servo風の別名。``enable()`` と同じ。"""
        self.enable()

    def detach(self) -> None:
        """Arduino Servo風の別名。出力を停止する（ドライバー無効化はしない）。"""
        self.stop()

    def set_home(self, raw_encoder_deg: float) -> None:
        """現在のエンコーダー値を機構座標の 0° として登録する。

        機構を物理原点やリミットスイッチ位置へ合わせてから一度だけ呼ぶ。
        """
        self._zero_raw_deg = float(raw_encoder_deg)
        self._position_deg = 0.0
        self._target_deg = 0.0
        self._reset_pid()
        self.motor.stop()

    def move_to(self, position_deg: float) -> float:
        """目標角度を設定する。範囲外は安全のためソフトリミットへ丸める。"""
        self._target_deg = self._clamp_position(position_deg)
        # 目標変更直後の積分残り・微分キックを防ぐ。
        self._integral = 0.0
        self._previous_error = None
        return self._target_deg

    def write(self, position_deg: float) -> float:
        """Arduino Servo風の別名。``write(角度)`` は ``move_to(角度)`` と同じ。"""
        return self.move_to(position_deg)

    def read(self) -> float:
        """Arduino Servo互換の読み取り。最後に指定した目標角度を返す。"""
        return self._target_deg

    def read_position(self) -> float | None:
        """エンコーダーから得た実際の現在角度を返す。"""
        return self._position_deg

    def move_relative(self, delta_deg: float) -> float:
        """現在位置を基準に相対移動する。エンコーダー受信後にのみ使える。"""
        if self._position_deg is None:
            raise RuntimeError("move_relative requires an encoder update or set_home first")
        return self.move_to(self._position_deg + float(delta_deg))

    def hold(self) -> float:
        """現在位置を新しい目標として保持する。"""
        if self._position_deg is None:
            raise RuntimeError("hold requires an encoder update or set_home first")
        return self.move_to(self._position_deg)

    def update(self, raw_encoder_deg: float, now: float | None = None) -> ServoState:
        """最新エンコーダー値で位置制御を1周期進める。

        20〜100 Hz程度で継続して呼ぶ。戻り値はログ表示や到達判定に使える。
        """
        if self._zero_raw_deg is None:
            raise RuntimeError("call set_home() before update()")

        timestamp = monotonic() if now is None else float(now)
        raw_offset = float(raw_encoder_deg) - self._zero_raw_deg
        position = self.config.direction * raw_offset
        self._position_deg = position
        error = self._target_deg - position
        at_target = abs(error) <= self.config.tolerance_deg
        desired_command, p_term, i_term, d_term, dt = self._pid_command(error, timestamp, at_target)

        # ソフトリミットを越えた場合は、範囲外へ進む向きだけを止める。
        limited = position <= self.config.min_position_deg or position >= self.config.max_position_deg
        blocked_by_limit = False
        if position <= self.config.min_position_deg and desired_command < 0.0:
            desired_command = 0.0
            blocked_by_limit = True
        elif position >= self.config.max_position_deg and desired_command > 0.0:
            desired_command = 0.0
            blocked_by_limit = True

        # 到達時・リミット到達時は惰性で進ませず、即座に停止する。
        if at_target or blocked_by_limit:
            command = 0.0
            self._reset_pid(timestamp)
        else:
            command = self._slew_command(desired_command, dt)
        self.motor.set_velocity(command)
        return ServoState(position, self._target_deg, error, command, at_target, limited, p_term, i_term, d_term)

    def loop(self, raw_encoder_deg: float, now: float | None = None) -> ServoState:
        """Arduinoの ``loop()`` 風の別名。``update()`` と同じ。"""
        return self.update(raw_encoder_deg, now)

    def stop(self) -> None:
        """出力を停止し、次回の制御開始も停止状態から行う。"""
        self._reset_pid()
        self.motor.stop()

    def _pid_command(self, error: float, timestamp: float, at_target: bool) -> tuple[float, float, float, float, float]:
        """位置誤差からPID出力を作り、積分飽和を防ぐ。"""
        if self._last_update_at is None:
            self._last_update_at = timestamp
        dt = max(0.001, timestamp - self._last_update_at)
        self._last_update_at = timestamp

        if at_target:
            self._integral = 0.0
            self._previous_error = None
            return 0.0, 0.0, 0.0, 0.0, dt

        p_term = self.config.position_kp * error
        derivative = 0.0 if self._previous_error is None else (error - self._previous_error) / dt
        d_term = self.config.position_kd * derivative
        candidate_integral = _clip(
            self._integral + error * dt,
            self.config.integral_limit,
        )
        candidate_i_term = self.config.position_ki * candidate_integral
        unclamped = p_term + candidate_i_term + d_term
        desired = _clip(unclamped, self.config.max_command)

        # 飽和方向への積分だけを捨て、積分ワインドアップを防ぐ。
        if desired == unclamped or (desired > 0.0 and error < 0.0) or (desired < 0.0 and error > 0.0):
            self._integral = candidate_integral
        i_term = self.config.position_ki * self._integral
        self._previous_error = error
        return _clip(p_term + i_term + d_term, self.config.max_command), p_term, i_term, d_term, dt

    def _slew_command(self, desired: float, dt: float) -> float:
        maximum_delta = self.config.command_accel_per_sec * dt
        command = max(self._last_command - maximum_delta, min(self._last_command + maximum_delta, desired))
        self._last_command = command
        return command

    def _reset_pid(self, timestamp: float | None = None) -> None:
        self._last_command = 0.0
        self._integral = 0.0
        self._previous_error = None
        self._last_update_at = timestamp

    def _clamp_position(self, position_deg: float) -> float:
        return max(self.config.min_position_deg, min(self.config.max_position_deg, float(position_deg)))


def _clip(value: float, maximum: float) -> float:
    return max(-maximum, min(maximum, float(value)))
