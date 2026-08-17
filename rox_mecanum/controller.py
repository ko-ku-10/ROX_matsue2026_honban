"""DualSense の入力を、機種依存の番号ではなく名前で扱うための部品。

pygame はこのモジュールを import しただけでは必要ありません。実際に
``PygameDualSense.open()`` を呼ぶときだけ必要です。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from math import atan2, degrees, hypot
from time import monotonic
from typing import Mapping


class Button(str, Enum):
    """DualSense の意味上の全ボタン名。"""

    CROSS = "cross"
    CIRCLE = "circle"
    SQUARE = "square"
    TRIANGLE = "triangle"
    L1 = "l1"
    R1 = "r1"
    L2 = "l2"
    R2 = "r2"
    CREATE = "create"
    OPTIONS = "options"
    L3 = "l3"
    R3 = "r3"
    PS = "ps"
    TOUCHPAD = "touchpad"
    MUTE = "mute"
    DPAD_UP = "dpad_up"
    DPAD_DOWN = "dpad_down"
    DPAD_LEFT = "dpad_left"
    DPAD_RIGHT = "dpad_right"


class Axis(str, Enum):
    """アナログ入力名。スティックの Y 正方向は常に「上」。"""

    LEFT_X = "left_x"
    LEFT_Y = "left_y"
    RIGHT_X = "right_x"
    RIGHT_Y = "right_y"
    L2 = "l2"
    R2 = "r2"


@dataclass(frozen=True)
class AnalogStick:
    """正規化済みスティック値。

    ``angle_degrees`` は右を 0°、上を 90°、左を ±180°、下を -90° とする。
    中立時は方向が定義できないため ``None``。
    """

    x: float = 0.0
    y: float = 0.0

    def __post_init__(self) -> None:
        object.__setattr__(self, "x", _clip(self.x))
        object.__setattr__(self, "y", _clip(self.y))

    @property
    def magnitude(self) -> float:
        """倒し量。対角方向を含めて 0.0〜1.0 に正規化する。"""
        return min(1.0, hypot(self.x, self.y))

    @property
    def angle_degrees(self) -> float | None:
        """右=0°、上=90°の角度。中立時は ``None``。"""
        if self.magnitude == 0.0:
            return None
        return degrees(atan2(self.y, self.x))

    @property
    def angle_radians(self) -> float | None:
        """``angle_degrees`` と同じ基準のラジアン角。"""
        if self.magnitude == 0.0:
            return None
        return atan2(self.y, self.x)

    @property
    def direction(self) -> str:
        """8方向の読みやすい方向名。中立時は ``neutral``。"""
        angle = self.angle_degrees
        if angle is None:
            return "neutral"
        names = ("right", "up_right", "up", "up_left", "left", "down_left", "down", "down_right")
        return names[int(((angle + 22.5) % 360) // 45)]

    def with_deadzone(self, deadzone: float = 0.08) -> "AnalogStick":
        """円形デッドゾーンを適用した新しい値を返す。"""
        if not 0.0 <= deadzone < 1.0:
            raise ValueError("deadzone must be in the range [0.0, 1.0)")
        length = self.magnitude
        if length <= deadzone:
            return AnalogStick()
        adjusted = (length - deadzone) / (1.0 - deadzone)
        scale = adjusted / length
        return AnalogStick(self.x * scale, self.y * scale)


@dataclass(frozen=True)
class ControllerState:
    """一時点の DualSense 入力。

    ``raw_axes`` と ``raw_buttons`` も保持するため、OS ごとの割当てが異なる場合
    でも、すべての物理入力を番号付きで確認できます。
    """

    left_stick: AnalogStick = field(default_factory=AnalogStick)
    right_stick: AnalogStick = field(default_factory=AnalogStick)
    l2: float = 0.0
    r2: float = 0.0
    buttons: Mapping[Button, bool] = field(default_factory=dict)
    pressed: frozenset[Button] = field(default_factory=frozenset)
    released: frozenset[Button] = field(default_factory=frozenset)
    raw_axes: tuple[float, ...] = ()
    raw_buttons: tuple[bool, ...] = ()
    timestamp: float = field(default_factory=monotonic)

    def __post_init__(self) -> None:
        object.__setattr__(self, "l2", _unit(self.l2))
        object.__setattr__(self, "r2", _unit(self.r2))

    def button(self, button: Button | str) -> bool:
        """指定ボタンを現在押しているか。文字列も受け付ける。"""
        return bool(self.buttons.get(Button(button), False))

    def was_pressed(self, button: Button | str) -> bool:
        """今回の読み取りで押下されたか。"""
        return Button(button) in self.pressed

    def was_released(self, button: Button | str) -> bool:
        """今回の読み取りで離されたか。"""
        return Button(button) in self.released

    @property
    def active_buttons(self) -> frozenset[Button]:
        """現在押されている意味上のボタン一覧。"""
        return frozenset(button for button, active in self.buttons.items() if active)

    @property
    def axes(self) -> Mapping[Axis, float]:
        """名前付きの全アナログ入力。Y 軸は上が正。"""
        return {
            Axis.LEFT_X: self.left_stick.x,
            Axis.LEFT_Y: self.left_stick.y,
            Axis.RIGHT_X: self.right_stick.x,
            Axis.RIGHT_Y: self.right_stick.y,
            Axis.L2: self.l2,
            Axis.R2: self.r2,
        }


@dataclass(frozen=True)
class ControllerProfile:
    """pygame の生番号を DualSense の意味上の入力名へ対応付ける設定。"""

    axes: Mapping[Axis, int]
    buttons: Mapping[Button, int]
    dpad_hat: int | None = 0
    trigger_axes_are_signed: bool = True


# 元の mecanum_rc.py で使われていた Ubuntu 向け軸番号を基準にした既定値。
# ボタン番号は接続方法で変わり得るため、raw_buttons で確認してプロファイルを調整できる。
DEFAULT_PYGAME_PROFILE = ControllerProfile(
    axes={
        Axis.LEFT_X: 0,
        Axis.LEFT_Y: 1,
        Axis.RIGHT_X: 2,
        Axis.RIGHT_Y: 3,
        Axis.L2: 5,
        Axis.R2: 4,
    },
    buttons={
        Button.CROSS: 0,
        Button.CIRCLE: 1,
        Button.SQUARE: 2,
        Button.TRIANGLE: 3,
        Button.L1: 4,
        Button.R1: 5,
        Button.L2: 6,
        Button.R2: 7,
        Button.CREATE: 8,
        Button.OPTIONS: 9,
        Button.L3: 10,
        Button.R3: 11,
        Button.PS: 12,
        Button.TOUCHPAD: 13,
        Button.MUTE: 14,
    },
)


class PygameDualSense:
    """pygame 経由で DualSense を読み取る入力アダプター。

    ``read()`` は常に全軸・全ボタンの生値も含む ``ControllerState`` を返す。
    ハードウェアを持たないテストでは ``ControllerState`` を直接作ればよい。
    """

    def __init__(self, joystick: object, profile: ControllerProfile = DEFAULT_PYGAME_PROFILE):
        self._joystick = joystick
        self.profile = profile
        self._previous_buttons: frozenset[Button] = frozenset()

    @classmethod
    def open(
        cls,
        index: int = 0,
        profile: ControllerProfile = DEFAULT_PYGAME_PROFILE,
    ) -> "PygameDualSense":
        """pygame の指定ジョイスティックを開く。pygame はここでだけ import する。"""
        try:
            import pygame
        except ImportError as error:  # pragma: no cover - 実機依存
            raise RuntimeError("pygame が必要です: pip install 'rox-mecanum[pygame]'") from error

        pygame.init()
        pygame.joystick.init()
        if not 0 <= index < pygame.joystick.get_count():
            raise RuntimeError(f"controller index {index} is unavailable")
        joystick = pygame.joystick.Joystick(index)
        joystick.init()
        return cls(joystick, profile)

    @property
    def name(self) -> str:
        return str(self._joystick.get_name())

    def read(self) -> ControllerState:
        """最新の入力を取得する。pygame のイベントキューもここで処理する。"""
        try:
            import pygame
        except ImportError as error:  # pragma: no cover - open 後に消えることはない
            raise RuntimeError("pygame が必要です") from error

        pygame.event.pump()
        raw_axes = tuple(float(self._joystick.get_axis(i)) for i in range(self._joystick.get_numaxes()))
        raw_buttons = tuple(bool(self._joystick.get_button(i)) for i in range(self._joystick.get_numbuttons()))
        buttons = {
            button: _at(raw_buttons, index, False)
            for button, index in self.profile.buttons.items()
        }
        buttons.update(self._read_dpad())
        active = frozenset(button for button, value in buttons.items() if value)
        state = ControllerState(
            left_stick=AnalogStick(
                _at(raw_axes, self.profile.axes[Axis.LEFT_X], 0.0),
                -_at(raw_axes, self.profile.axes[Axis.LEFT_Y], 0.0),
            ),
            right_stick=AnalogStick(
                _at(raw_axes, self.profile.axes[Axis.RIGHT_X], 0.0),
                -_at(raw_axes, self.profile.axes[Axis.RIGHT_Y], 0.0),
            ),
            l2=self._trigger_value(raw_axes, Axis.L2),
            r2=self._trigger_value(raw_axes, Axis.R2),
            buttons=buttons,
            pressed=active - self._previous_buttons,
            released=self._previous_buttons - active,
            raw_axes=raw_axes,
            raw_buttons=raw_buttons,
        )
        self._previous_buttons = active
        return state

    def close(self) -> None:
        """このジョイスティックを閉じる。pygame 全体は終了しない。"""
        self._joystick.quit()

    def _read_dpad(self) -> dict[Button, bool]:
        if self.profile.dpad_hat is None:
            return {}
        try:
            x, y = self._joystick.get_hat(self.profile.dpad_hat)
        except Exception:
            return {}
        return {
            Button.DPAD_UP: y > 0,
            Button.DPAD_DOWN: y < 0,
            Button.DPAD_LEFT: x < 0,
            Button.DPAD_RIGHT: x > 0,
        }

    def _trigger_value(self, raw_axes: tuple[float, ...], axis: Axis) -> float:
        value = _at(raw_axes, self.profile.axes[axis], 0.0)
        return _unit((value + 1.0) / 2.0) if self.profile.trigger_axes_are_signed else _unit(value)


def _at(values: tuple[float, ...] | tuple[bool, ...], index: int, default: float | bool) -> float | bool:
    return values[index] if 0 <= index < len(values) else default


def _clip(value: float) -> float:
    return max(-1.0, min(1.0, float(value)))


def _unit(value: float) -> float:
    return max(0.0, min(1.0, float(value)))
