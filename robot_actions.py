"""あなたがロボットの動きを自由に書くファイル。

GAME1〜3は、このファイルにある関数を呼ぶだけです。
CAN通信・PID・メカナムは書かなくてよいですが、lift/catch/GPIOの動きは
前の motiage.py と同じように自由に書けます。
"""

import time

try:  # PC上で構文確認する時はHobot.GPIOが無くてもよい。
    import Hobot.GPIO as GPIO
except ImportError:  # pragma: no cover - RDK X5実機依存
    GPIO = None


# ==================================================
# GPIO番号をここに直接書く。
# ==================================================
CYLINDER_EXTEND_PIN = 27
CYLINDER_RETRACT_PIN = 17
_configured_pins = set()


def setup_gpio():
    """GAME起動時に1回だけ呼ばれる。両方のGPIOをOFFで開始する。"""
    if GPIO is None:
        raise RuntimeError("Hobot.GPIOが見つかりません。RDK X5上で実行してください")
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    for pin in (CYLINDER_EXTEND_PIN, CYLINDER_RETRACT_PIN):
        if pin is not None:
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
            _configured_pins.add(pin)


def all_off():
    """OPTIONS・例外・終了時に必ず呼ばれる。2個ともOFFにする。"""
    if GPIO is None:
        return
    for pin in _configured_pins:
        GPIO.output(pin, GPIO.LOW)


def close_gpio():
    """GAME終了時のGPIO片付け。通常は自分で呼ばなくてよい。"""
    all_off()
    if GPIO is not None:
        for pin in _configured_pins.copy():
            GPIO.cleanup(pin)
            _configured_pins.remove(pin)


# ==================================================
# ここから下の関数の中身を、あなたが自由に書く。
# runtime.servos.lift.write(角度)
# runtime.servos.catch.write(角度)
# GPIO.output(CYLINDER_EXTEND_PIN, GPIO.HIGH)
# time.sleep(秒)
# などを自由に使える。
# ==================================================

def game1_start_pose(runtime):
    """GAME1: CREATEを押した時の開始姿勢。"""
    servos.lift.write(110)
    servos.catch.write(-30)


def game2_ground_pose(runtime):
    """GAME2: 地面にボールを付けて走る姿勢。"""
    servos.lift.write(110)
    servos.catch.write(-30)

def game2_lift_for_shot(runtime):
    """GAME2: 発射高さへliftを動かす。"""
    servos.catch.write(-45)
    time.sleep(0.5)
    servos.lift.write(20)
    time.sleep(1)
    servos.catch.write(-10)
    time.sleep(0.5)
    servos.lift.write(110)


def game2_fire(runtime):
    """GAME2: 発射動作。GPIOを直接使って自由に書く。"""
    GPIO.output(17, GPIO.LOW)
    GPIO.output(27, GPIO.HIGH)
    time.sleep(0.05)
    GPIO.output(27, GPIO.LOW)
    GPIO.output(17, GPIO.HIGH)
    time.sleep(0.05)
    GPIO.output(17, GPIO.LOW)
    GPIO.output(27, GPIO.LOW)


def game3_ground_pose(runtime):
    """GAME3: 地面走行姿勢。"""
    servos.lift.write(110)
    servos.catch.write(0)


def game3_grab(runtime):
    """GAME3: ○を押した時の掴む動作。"""
    servos.lift.write(110)
    servos.catch.write()

def game3_release(runtime):
    """GAME3: □を押した時の排出動作。"""
    raise NotImplementedError("robot_actions.py の game3_release() を書いてください")


def game3_motiage(runtime):
    """GAME3: △を押した時の持上げ・発射までの全動作。"""
    raise NotImplementedError("robot_actions.py の game3_motiage() を書いてください")


def game3_cylinder_extend(runtime):
    """GAME3: R1を押した時のシリンダーを伸ばす動作。"""
    raise NotImplementedError("robot_actions.py の game3_cylinder_extend() を書いてください")


def game3_cylinder_retract(runtime):
    """GAME3: L1を押した時のシリンダーを戻す動作。"""
    raise NotImplementedError("robot_actions.py の game3_cylinder_retract() を書いてください")
