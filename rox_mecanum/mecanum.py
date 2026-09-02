"""メカナムホイールの移動コマンドと4輪速度への変換。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

from .controller import AnalogStick, Button, ControllerState


WHEEL_NAMES = ("FL", "FR", "RL", "RR")


@dataclass(frozen=True)
class MotionCommand:
    """ロボット座標系の正規化された移動指令。

    ``forward`` は前進、``strafe`` は右平行移動、``rotate`` は右旋回が正。
    いずれも -1.0〜+1.0 で表す。
    """

    forward: float = 0.0
    strafe: float = 0.0
    rotate: float = 0.0

    def __post_init__(self) -> None:
        object.__setattr__(self, "forward", _clip(self.forward))
        object.__setattr__(self, "strafe", _clip(self.strafe))
        object.__setattr__(self, "rotate", _clip(self.rotate))

    @classmethod
    def stop(cls) -> "MotionCommand":
        return cls()

    @classmethod
    def forward_motion(cls, speed: float = 1.0) -> "MotionCommand":
        return cls(forward=abs(speed))

    @classmethod
    def backward(cls, speed: float = 1.0) -> "MotionCommand":
        return cls(forward=-abs(speed))

    @classmethod
    def strafe_right(cls, speed: float = 1.0) -> "MotionCommand":
        return cls(strafe=abs(speed))

    @classmethod
    def strafe_left(cls, speed: float = 1.0) -> "MotionCommand":
        return cls(strafe=-abs(speed))

    @classmethod
    def turn_right(cls, speed: float = 1.0) -> "MotionCommand":
        return cls(rotate=abs(speed))

    @classmethod
    def turn_left(cls, speed: float = 1.0) -> "MotionCommand":
        return cls(rotate=-abs(speed))


@dataclass(frozen=True)
class WheelSpeeds:
    """論理上の各ホイールの正規化速度。各値は -1.0〜+1.0。"""

    front_left: float
    front_right: float
    rear_left: float
    rear_right: float

    def __post_init__(self) -> None:
        for name in ("front_left", "front_right", "rear_left", "rear_right"):
            object.__setattr__(self, name, _clip(getattr(self, name)))

    def as_dict(self) -> Mapping[str, float]:
        return {
            "FL": self.front_left,
            "FR": self.front_right,
            "RL": self.rear_left,
            "RR": self.rear_right,
        }

    def with_motor_directions(self, directions: Mapping[str, float]) -> "WheelSpeeds":
        """取付方向を補正したモーター出力を返す。

        例: 元プログラムと同じ構成では ``{"FR": -1, "RR": -1}`` を指定する。
        指定しないホイールは +1 で扱う。
        """
        values = self.as_dict()
        return WheelSpeeds(
            values["FL"] * directions.get("FL", 1.0),
            values["FR"] * directions.get("FR", 1.0),
            values["RL"] * directions.get("RL", 1.0),
            values["RR"] * directions.get("RR", 1.0),
        )


@dataclass(frozen=True)
class MecanumMixer:
    """移動指令をメカナム4輪の論理速度へ混合する。

    ``rotation_gain`` は ``L + W``。元のサンプルと同じ寸法なら 0.22。
    出力は必ず全輪共通の倍率で正規化され、移動の向きは保たれる。
    """

    rotation_gain: float = 0.22

    def __post_init__(self) -> None:
        if self.rotation_gain < 0.0:
            raise ValueError("rotation_gain must be non-negative")

    def mix(self, command: MotionCommand) -> WheelSpeeds:
        """前進・右ストレーフ・右旋回を4輪速度に変換する。"""
        forward, strafe, rotate = command.forward, command.strafe, command.rotate
        rotation = self.rotation_gain * rotate
        values = (
            forward - strafe - rotation,
            forward + strafe + rotation,
            forward + strafe - rotation,
            forward - strafe + rotation,
        )
        maximum = max(1.0, *(abs(value) for value in values))
        return WheelSpeeds(*(value / maximum for value in values))

    def wheel_speeds(self, forward: float = 0.0, strafe: float = 0.0, rotate: float = 0.0) -> WheelSpeeds:
        """数値を直接渡す短縮形。"""
        return self.mix(MotionCommand(forward, strafe, rotate))


@dataclass(frozen=True)
class DualSenseMotionMapping:
    """DualSense 状態から移動指令を作る規則。

    既定では左スティックが前後・平行移動、右スティック X が旋回。移動も旋回も
    常に有効で、L2/R2は他の用途へ自由に割り当てられる。
    """

    deadzone: float = 0.08
    translation_enable: Button | None = None
    rotation_enable: Button | None = None
    translation_gain: float = 1.0
    rotation_gain: float = 1.0
    response_exponent: float = 1.0
    # 横移動だけの速度倍率。前後・旋回の速度には影響しない。
    strafe_gain: float = 1.0
    # pygameのY軸は上方向が負になるため、実機に合わせて前後だけ反転できる。
    invert_forward: bool = False

    def __post_init__(self) -> None:
        if not 0.0 <= self.strafe_gain <= 1.0:
            raise ValueError("strafe_gain は 0.0〜1.0 にしてください")

    def command(self, state: ControllerState) -> MotionCommand:
        """コントローラーの最新状態を正規化移動指令へ変換する。"""
        left = state.left_stick.with_deadzone(self.deadzone)
        right = state.right_stick.with_deadzone(self.deadzone)
        if self.translation_enable is not None and not state.button(self.translation_enable):
            left = AnalogStick()
        if self.rotation_enable is not None and not state.button(self.rotation_enable):
            right = AnalogStick()
        forward = _shape(left.y, self.response_exponent, self.translation_gain)
        if self.invert_forward:
            forward = -forward
        strafe = _shape(left.x, self.response_exponent, self.translation_gain * self.strafe_gain)
        rotate = _shape(right.x, self.response_exponent, self.rotation_gain)
        return MotionCommand(
            forward=forward,
            strafe=strafe,
            rotate=rotate,
        )


def _clip(value: float) -> float:
    return max(-1.0, min(1.0, float(value)))


def stop() -> MotionCommand:
    """停止コマンドを作る。"""
    return MotionCommand.stop()


def forward(speed: float = 1.0) -> MotionCommand:
    """前進コマンドを作る。"""
    return MotionCommand.forward_motion(speed)


def backward(speed: float = 1.0) -> MotionCommand:
    """後退コマンドを作る。"""
    return MotionCommand.backward(speed)


def strafe_right(speed: float = 1.0) -> MotionCommand:
    """右へ平行移動するコマンドを作る。"""
    return MotionCommand.strafe_right(speed)


def strafe_left(speed: float = 1.0) -> MotionCommand:
    """左へ平行移動するコマンドを作る。"""
    return MotionCommand.strafe_left(speed)


def turn_right(speed: float = 1.0) -> MotionCommand:
    """右旋回コマンドを作る。"""
    return MotionCommand.turn_right(speed)


def turn_left(speed: float = 1.0) -> MotionCommand:
    """左旋回コマンドを作る。"""
    return MotionCommand.turn_left(speed)


def _shape(value: float, exponent: float, gain: float) -> float:
    if exponent < 1.0:
        raise ValueError("response_exponent must be at least 1.0")
    magnitude = abs(_clip(value)) ** exponent
    return _clip(magnitude * (1.0 if value >= 0.0 else -1.0) * gain)
