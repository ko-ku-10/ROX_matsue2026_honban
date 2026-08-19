"""エンコーダーなしのモーターを、時間から角度を推定して扱う疑似サーボ。"""

from __future__ import annotations

from dataclasses import dataclass
from time import sleep as default_sleep
from typing import Callable

from .serial_at import ATMotor


@dataclass(frozen=True)
class TimedServoConfig:
    """時間式サーボの設定。

    ``degrees_per_second`` は ``calibration_speed`` で実測した角速度。
    電源投入後は必ず ``home()`` を呼んで、現在角度を登録する。
    """

    min_angle: float
    max_angle: float
    degrees_per_second: float
    calibration_speed: float
    direction: int = 1

    def __post_init__(self) -> None:
        if self.max_angle <= self.min_angle:
            raise ValueError("max_angle は min_angle より大きくしてください")
        if self.degrees_per_second <= 0.0:
            raise ValueError("degrees_per_second は 0 より大きくしてください")
        if not 0.0 < abs(self.calibration_speed) <= 1.0:
            raise ValueError("calibration_speed は 0 より大きく1以下にしてください")
        if self.direction not in (-1, 1):
            raise ValueError("direction は 1 または -1 にしてください")


class TimedServo:
    """``home()`` と ``write(角度)`` で扱う、時間式のサーボ風API。"""

    def __init__(
        self,
        motor: ATMotor,
        config: TimedServoConfig,
        *,
        sleep: Callable[[float], None] = default_sleep,
    ) -> None:
        self.motor = motor
        self.config = config
        self._sleep = sleep
        self._angle: float | None = None
        self._attached = False

    def attach(self) -> None:
        """モーターを有効化する。"""
        self.motor.enable()
        self._attached = True

    def home(self, angle: float = 0.0) -> None:
        """現在の実機角度を登録する。モーターは動かさない。"""
        self._angle = self._limit(angle)
        self.motor.stop()

    def read(self) -> float | None:
        """推定中の現在角度を返す。原点未登録なら ``None``。"""
        return self._angle

    def write(self, angle: float, speed: float | None = None) -> float:
        """指定角度まで動かし、到達したと推定する角度を返す。

        ``speed`` は -1〜1。省略時は較正時と同じ速度を使用する。
        速度を変えた場合は、速度と角速度が比例すると仮定して時間を補正する。
        """
        if self._angle is None:
            raise RuntimeError("最初に実機を原点へ合わせて home() を呼んでください")
        if not self._attached:
            self.attach()

        target = self._limit(angle)
        delta = target - self._angle
        if delta == 0.0:
            return target

        requested_speed = self.config.calibration_speed if speed is None else float(speed)
        if not 0.0 < abs(requested_speed) <= 1.0:
            raise ValueError("speed は 0以外の -1〜1 にしてください")
        duration = abs(delta) / self.config.degrees_per_second
        duration *= abs(self.config.calibration_speed) / abs(requested_speed)
        direction = 1.0 if delta > 0.0 else -1.0

        try:
            self.motor.set_velocity(direction * self.config.direction * abs(requested_speed), force=True)
            self._sleep(duration)
            self._angle = target
            return target
        finally:
            self.motor.stop()

    def detach(self) -> None:
        """安全に停止する。AT方式には無効化フレームがないため停止のみ行う。"""
        self.motor.stop()
        self._attached = False

    def _limit(self, angle: float) -> float:
        return max(self.config.min_angle, min(self.config.max_angle, float(angle)))
