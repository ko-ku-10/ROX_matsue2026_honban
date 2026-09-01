"""RDK X5のSPIで、6個のWS2812系LEDテープを動作に連動させる。"""

from __future__ import annotations

from time import monotonic


# ==================================================
# ここを自由に書き換える。LEDテープは6個。
# ==================================================
LED_COUNT = 6
# RDK X5のSPI1 MOSI（40pinの物理Pin 19）を使う。
# LEDテープの入力側DINへ、5Vレベルシフタを経由してつなぐ。
# SPIバス1の device 1 を使う: /dev/spidev1.1
# 実機で `ls -l /dev/spidev*` に存在することを確認済み。
LED_SPI_BUS = 1
LED_SPI_DEVICE = 1
LED_SPI_SPEED_HZ = 2_400_000
# 実機のSPI1.1へ実際にLEDデータを送信する。
LED_OUTPUT_ENABLED = True

# 色は (赤, 緑, 青) の順。各値は 0〜255。
OFF = (0, 0, 0)
BLUE = (0, 30, 255)
GREEN = (0, 255, 40)
YELLOW = (255, 180, 0)
PURPLE = (180, 0, 255)
RED = (255, 0, 0)


def fill(color: tuple[int, int, int]) -> list[tuple[int, int, int]]:
    """6個すべてを同じ色にする短縮記法。"""
    return [color] * LED_COUNT


def one(index: int, color: tuple[int, int, int]) -> list[tuple[int, int, int]]:
    """指定した1個だけを光らせる短縮記法。先頭は0、最後は5。"""
    return [color if pixel == index else OFF for pixel in range(LED_COUNT)]


def timeline(
    seconds: float,
    frames: list[tuple[float, list[tuple[int, int, int]]]],
    *,
    loop: bool = True,
) -> list[tuple[int, int, int]]:
    """時間指定の色リストから、現在表示する1コマを返す。

    ``frames`` の書き方は ``[(秒数, LED6個の色), ...]``。
    ``loop=True`` なら最後まで行った後に最初へ戻り、Falseなら最後の色で止まる。
    """
    valid = [(float(duration), list(colors)) for duration, colors in frames if float(duration) > 0.0]
    if not valid:
        return fill(OFF)
    total = sum(duration for duration, _colors in valid)
    moment = float(seconds) % total if loop else min(float(seconds), total - 0.000001)
    for duration, colors in valid:
        if moment < duration:
            return colors
        moment -= duration
    return valid[-1][1]


def encode_ws2812(pixels: list[tuple[int, int, int]]) -> bytes:
    """RGB色を、SPI 2.4MHzで送れるWS2812信号へ変換する。

    WS2812の1ビットをSPIの3ビットで表す。1=110、0=100。
    WS2812はGRB順なので、設定で書くRGB順とはここで入れ替える。
    """
    data = bytearray()
    for red, green, blue in pixels[:LED_COUNT]:
        for value in (green, red, blue):
            encoded = 0
            for shift in range(7, -1, -1):
                encoded = (encoded << 3) | (0b110 if (int(value) >> shift) & 1 else 0b100)
            data.extend(encoded.to_bytes(3, "big"))
    # 80us以上のLOWでWS2812へ表示更新を知らせる。
    data.extend(b"\x00" * 32)
    return bytes(data)


# ==================================================
# ここに動作ごとの光り方を書く。
# 引数 ``seconds`` は、その動作が始まってからの秒数。
# 必ず6個ぶんの色を ``[色1, 色2, ...]`` で返す。
# ==================================================
# このように時間と色を並べるだけで、自由な演出を書ける。
# 例: [(0.1, fill(RED)), (0.1, fill(OFF)), (0.5, fill(PURPLE))]
#       0.1秒赤 → 0.1秒消灯 → 0.5秒紫。

# 待機中の1周する青い光。最後まで行くと先頭から繰り返す。
STANDBY_FRAMES = [(0.12, one(index, BLUE)) for index in range(LED_COUNT)]

# 地面走行中の緑の往復。
GROUND_FRAMES = [(0.10, one(index, GREEN)) for index in (0, 1, 2, 3, 4, 5, 4, 3, 2, 1)]

# 発射準備中の紫点滅。最後まで行くと繰り返す。
READY_FRAMES = [(0.25, fill(PURPLE)), (0.25, fill(OFF))]

# 発射中の赤い流れ。loop=Falseなので、最後の消灯で止まる。
FIRING_FRAMES = [
    (0.06, one(0, RED)),
    (0.06, one(1, RED)),
    (0.06, one(2, RED)),
    (0.06, one(3, RED)),
    (0.06, one(4, RED)),
    (0.06, one(5, RED)),
    (0.10, fill(RED)),
    (0.10, fill(OFF)),
]


def animation_standby(seconds: float) -> list[tuple[int, int, int]]:
    """待機: 青い光が左から右へ流れる。"""
    return timeline(seconds, STANDBY_FRAMES)


def animation_ground(seconds: float) -> list[tuple[int, int, int]]:
    """地面走行: 緑が左右へ往復する。"""
    return timeline(seconds, GROUND_FRAMES)


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
    return timeline(seconds, READY_FRAMES)


def animation_firing(seconds: float) -> list[tuple[int, int, int]]:
    """発射: 赤が前から後ろへ流れる。"""
    return timeline(seconds, FIRING_FRAMES, loop=False)


class RobotLights:
    """現在のLED色を管理する。未接続でもロボット動作を止めない。"""

    def __init__(self) -> None:
        self.pixels: list[tuple[int, int, int]] = [OFF] * LED_COUNT
        self.pattern_name = "消灯"
        self._animation = lambda _seconds: [OFF] * LED_COUNT
        self._started_at = monotonic()
        self._spi = None
        self._last_output_error = ""

    def _show(self, pixels: list[tuple[int, int, int]]) -> None:
        """LED6個の色をSPI経由で実機へ送る。"""
        self.pixels = list(pixels[:LED_COUNT])
        while len(self.pixels) < LED_COUNT:
            self.pixels.append(OFF)

        if LED_OUTPUT_ENABLED:
            self._send_ws2812()

    def _send_ws2812(self) -> None:
        """RDK X5のSPI MOSIから、現在の6個の色を送る。"""
        try:
            if self._spi is None:
                import spidev

                self._spi = spidev.SpiDev()
                self._spi.open(LED_SPI_BUS, LED_SPI_DEVICE)
                self._spi.mode = 0
                self._spi.max_speed_hz = LED_SPI_SPEED_HZ
            self._spi.xfer2(list(encode_ws2812(self.pixels)))
            self._last_output_error = ""
        except Exception as error:  # pragma: no cover - RDK実機のSPI依存
            # LED不調だけでロボット本体が停止しないよう、LEDを無効化して続行する。
            message = str(error)
            if message != self._last_output_error:
                print(f"[LED] SPI送信失敗（LEDのみ無効）: {message}")
                self._last_output_error = message
            self.close()

    def close(self) -> None:
        """SPIを閉じる。次回の実行時には自動で開き直す。"""
        if self._spi is not None:
            try:
                self._spi.close()
            finally:
                self._spi = None

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
