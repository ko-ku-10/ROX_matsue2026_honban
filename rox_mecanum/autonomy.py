"""自動走行で共通に使う、小さく安全な部品。"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from time import monotonic
from typing import Protocol

from .controller import Button, ControllerState
from .mecanum import MotionCommand


class HorizontalTarget(Protocol):
    """画像中心からの左右ずれを持つ、Tagなどの検出対象。"""

    horizontal_error: float


class ControlMode(str, Enum):
    """走行の操作モード。"""

    MANUAL = "manual"
    AUTO = "auto"


@dataclass
class ModeController:
    """タッチパッドで完全手動と自動を切り替える。

    切替時は必ず自動コマンドを捨てる。以前の自動動作を勝手に再開しない。
    """

    mode: ControlMode = ControlMode.MANUAL
    changed_at: float = 0.0

    def update(self, state: ControllerState) -> bool:
        if not state.was_pressed(Button.TOUCHPAD):
            return False
        self.mode = ControlMode.AUTO if self.mode is ControlMode.MANUAL else ControlMode.MANUAL
        self.changed_at = monotonic()
        return True

    @property
    def auto_enabled(self) -> bool:
        return self.mode is ControlMode.AUTO


def add_manual_command(auto: MotionCommand, manual: MotionCommand, enabled: bool) -> MotionCommand:
    """自動速度と手動速度を合成する。

    完全手動モードでは ``auto`` を無視する。``MotionCommand`` が各軸を
    -1〜+1へ制限するため、合成後も危険な値にならない。
    """

    if not enabled:
        return manual
    return MotionCommand(
        forward=auto.forward + manual.forward,
        strafe=auto.strafe + manual.strafe,
        rotate=auto.rotate + manual.rotate,
    )


def face_target_command(
    target: HorizontalTarget,
    *,
    center_tolerance: float = 0.08,
    rotation_gain: float = 0.60,
    maximum_speed: float = 0.20,
) -> MotionCommand:
    """Tagが画面中央へ来るまで、その方向へだけ旋回する。

    画面端のTagは魚眼や斜め視点で距離が不正確になりやすいため、前進・横移動を
    始める前にこの指令でカメラ正面へ向ける。中央に入っていれば停止を返す。
    """
    if center_tolerance < 0.0 or rotation_gain <= 0.0 or maximum_speed <= 0.0:
        raise ValueError("中心許容値・旋回ゲイン・最大速度を確認してください")
    error = float(target.horizontal_error)
    if abs(error) <= center_tolerance:
        return MotionCommand.stop()
    speed = max(-maximum_speed, min(maximum_speed, error * rotation_gain))
    return MotionCommand(rotate=speed)


@dataclass
class TimedMotion:
    """短時間だけ一定速度を出す、設定値ベースの動作。"""

    command: MotionCommand
    duration_sec: float
    started_at: float | None = None

    def start(self, now: float | None = None) -> None:
        self.started_at = monotonic() if now is None else now

    def active_command(self, now: float | None = None) -> MotionCommand:
        if self.started_at is None:
            return MotionCommand.stop()
        current = monotonic() if now is None else now
        return self.command if current - self.started_at < self.duration_sec else MotionCommand.stop()

    def finished(self, now: float | None = None) -> bool:
        if self.started_at is None:
            return False
        current = monotonic() if now is None else now
        return current - self.started_at >= self.duration_sec
