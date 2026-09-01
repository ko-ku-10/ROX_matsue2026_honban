"""ロボット動作に連動する6個のLEDテープ演出。

今は未配線なので、LED_DATA_PIN は仮のGPIO番号で、実機送信は無効である。
各GAMEや機構の動きはこのファイルの関数だけを呼ぶため、配線が決まったら
``show()`` の中へWS2812用の送信処理を追加すれば、GAME側を直さず使える。
"""

from __future__ import annotations

from time import monotonic


# ==================================================
# ここを自由に書き換える。LEDテープは6個。
# ==================================================
LED_COUNT = 6
# DINへつなぐ予定の仮GPIO番号。配線が決まったらここだけ変更する。
LED_DATA_PIN = 22
# 未配線の間はFalseのまま。今は光らせず、ターミナルへ状態だけを表示する。
LED_OUTPUT_ENABLED = False

# 色は (赤, 緑, 青) の順。各値は 0〜255。
OFF = (0, 0, 0)
BLUE = (0, 30, 255)
GREEN = (0, 255, 40)
YELLOW = (255, 180, 0)
PURPLE = (180, 0, 255)
RED = (255, 0, 0)


# ==================================================
# ここに動作ごとの光り方を書く。
# 引数 ``seconds`` は、その動作が始まってからの秒数。
# 必ず6個ぶんの色を ``[色1, 色2, ...]`` で返す。
# ==================================================
def animation_standby(seconds: float) -> list[tuple[int, int, int]]:
    """待機: 青い光が左から右へ流れる。"""
    point = int(seconds * 4) % LED_COUNT
    return [BLUE if index == point else OFF for index in range(LED_COUNT)]


def animation_ground(seconds: float) -> list[tuple[int, int, int]]:
    """地面走行: 緑が左右へ往復する。"""
    order = [0, 1, 2, 3, 4, 5, 4, 3, 2, 1]
    point = order[int(seconds * 5) % len(order)]
    return [GREEN if index == point else OFF for index in range(LED_COUNT)]


def animation_lifting(seconds: float, completed_steps: int) -> list[tuple[int, int, int]]:
    """持上げ中: 完了した段は紫、残りは黄色が流れる。"""
    moving = int(seconds * 7) % LED_COUNT
    completed = max(0, min(LED_COUNT, completed_steps))
    return [
        PURPLE if index < completed else YELLOW if index == moving else OFF
        for index in range(LED_COUNT)
    ]


def animation_ready(seconds: float) -> list[tuple[int, int, int]]:
    """発射準備: 紫が点滅する。"""
    return [PURPLE if int(seconds * 3) % 2 == 0 else OFF] * LED_COUNT


def animation_firing(seconds: float) -> list[tuple[int, int, int]]:
    """発射: 赤が前から後ろへ流れる。"""
    point = int(seconds * 16) % LED_COUNT
    return [RED if index <= point else OFF for index in range(LED_COUNT)]


class RobotLights:
    """現在のLED色を管理する。未接続でもロボット動作を止めない。"""

    def __init__(self) -> None:
        self.pixels: list[tuple[int, int, int]] = [OFF] * LED_COUNT
        self.pattern_name = "消灯"
        self._animation = lambda _seconds: [OFF] * LED_COUNT
        self._started_at = monotonic()

    def _show(self, pixels: list[tuple[int, int, int]]) -> None:
        """LED6個の色を実機へ送る場所。未配線の間は何もしない。"""
        self.pixels = list(pixels[:LED_COUNT])
        while len(self.pixels) < LED_COUNT:
            self.pixels.append(OFF)

        if LED_OUTPUT_ENABLED:
            # WS2812への実送信は、DINの接続方法が決まってからここへ追加する。
            # 通常GPIOのON/OFFではWS2812の高速信号を正確に作れないため、
            # 未確認の配線へ適当な信号を送らない。
            pass

    def set_animation(self, name: str, animation) -> None:
        """動作に対応する光り方を切り替える。"""
        self.pattern_name = str(name)
        self._animation = animation
        self._started_at = monotonic()
        print(f"[LED] {self.pattern_name}")
        self.update()

    def update(self) -> None:
        """現在の光り方を1コマ進める。GAMEのループから繰り返し呼ばれる。"""
        pixels = self._animation(monotonic() - self._started_at)
        if list(pixels) != self.pixels:
            self._show(pixels)

    def off(self) -> None:
        self.set_animation("消灯", lambda _seconds: [OFF] * LED_COUNT)

    def standby(self) -> None:
        self.set_animation("待機", animation_standby)

    def ground(self) -> None:
        self.set_animation("地面走行", animation_ground)

    def lifting(self, step: int = 0) -> None:
        completed = int(step) + 1
        self.set_animation("持上げ中", lambda seconds: animation_lifting(seconds, completed))

    def ready_to_fire(self) -> None:
        self.set_animation("発射準備", animation_ready)

    def firing(self) -> None:
        self.set_animation("発射", animation_firing)


lights = RobotLights()
