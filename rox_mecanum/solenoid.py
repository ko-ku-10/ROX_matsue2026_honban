"""RDK X5のGPIOでソレノイド用出力を扱う。"""

from __future__ import annotations


class RDKSolenoid:
    """Hobot.GPIOを使う、active-highのソレノイド出力。

    ``pin`` はRaspberry Pi互換のBCM番号として扱う。既存の
    ``hensuu.solenoid_pin = 17`` はそのまま使える。
    """

    def __init__(self, pin: int) -> None:
        try:
            import Hobot.GPIO as GPIO
        except ImportError as error:  # pragma: no cover - RDK X5実機依存
            raise RuntimeError(
                "RDK X5用のHobot.GPIOが見つかりません。RDK X5上で実行してください"
            ) from error
        self._gpio = GPIO
        self.pin = int(pin)
        GPIO.setwarnings(False)
        # hensuu.pyの17は、従来どおりRaspberry Pi互換BCM番号17として使う。
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.OUT, initial=GPIO.LOW)

    def on(self) -> None:
        self._gpio.output(self.pin, self._gpio.HIGH)

    def off(self) -> None:
        self._gpio.output(self.pin, self._gpio.LOW)

    def close(self) -> None:
        self.off()
        self._gpio.cleanup(self.pin)
