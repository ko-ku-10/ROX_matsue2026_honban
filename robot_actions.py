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
lift_orosu = 106
lift_motiage = 30
catch_hozi = -38
catch_machi = -17
catch_tukamu = -45

# 指令した角度へ届かない時に、永遠に待ち続けないための安全時間。
# 動作完了の判定には使わず、エンコーダーの実測角度で判定する。
SERVO_MOVE_TIMEOUT_SEC = 8.0
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
    servos = runtime.servos
    servos.lift.write(lift_orosu)
    servos.catch.write(catch_hozi)


def game2_ground_pose(runtime):
    """GAME2: 地面にボールを付けて走る姿勢。"""
    servos = runtime.servos
    servos.lift.write(lift_orosu)
    servos.catch.write(catch_hozi)


def wait_until_reached(servo, name):
    """エンコーダー実測値が目標角度へ届くまで待つ。

    ``time.sleep(1)`` のような固定時間では完了扱いにしない。
    PIDスレッドが読むRobStrideのmechPosと目標角度との差が、設定した
    許容誤差以内になった時だけ次の行へ進む。
    """
    deadline = time.monotonic() + SERVO_MOVE_TIMEOUT_SEC

    while not servo.is_at_target():
        if time.monotonic() >= deadline:
            current = servo.read()
            raise TimeoutError(
                f"{name} が目標角度へ到達しません "
                f"(現在: {current}, 目標: {servo.target_angle})"
            )
        time.sleep(0.02)

    print(f"{name}: エンコーダーで到達を確認しました ({servo.read():.1f}度)")


def ball_lift_for_shot(runtime):
    """GAME2・GAME3共通: ボールを発射する高さへ動かす。"""
    servos = runtime.servos

    # 各行で目標を出し、エンコーダーが「届いた」と確認してから次へ進む。
    servos.lift.write(lift_orosu)
    wait_until_reached(servos.lift, "liftを下ろす")

    # すでに保持しているボールを、掴み直さずこの角度のまま運ぶ。
    servos.catch.write(catch_hozi)
    wait_until_reached(servos.catch, "catchで保持する")

    servos.lift.write(lift_motiage)
    wait_until_reached(servos.lift, "liftで発射台へ運ぶ")

    servos.catch.write(catch_machi)
    wait_until_reached(servos.catch, "catchで発射台へ載せる")

    servos.lift.write(lift_orosu)
    wait_until_reached(servos.lift, "liftを下ろす")


def ball_fire(runtime):
    """GAME2・GAME3共通: ボールを発射する。GPIOの順番はここだけで編集する。"""
    GPIO.output(CYLINDER_RETRACT_PIN, GPIO.LOW)
    GPIO.output(CYLINDER_EXTEND_PIN, GPIO.HIGH)
    time.sleep(0.05)
    GPIO.output(CYLINDER_EXTEND_PIN, GPIO.LOW)
    GPIO.output(CYLINDER_RETRACT_PIN, GPIO.HIGH)
    time.sleep(0.05)
    GPIO.output(CYLINDER_RETRACT_PIN, GPIO.LOW)
    GPIO.output(CYLINDER_EXTEND_PIN, GPIO.LOW)


def game3_ground_pose(runtime):
    """GAME3: 地面走行姿勢。"""
    servos = runtime.servos
    servos.lift.write(lift_orosu)
    servos.catch.write(catch_machi)


def game3_grab(runtime):
    """GAME3: ○を押した時の掴む動作。"""
    servos = runtime.servos
    servos.lift.write(lift_orosu)
    servos.catch.write(catch_hozi)

def game3_release(runtime):
    """GAME3: □を押した時の排出動作。"""
    servos = runtime.servos
    servos.lift.write(lift_orosu)
    servos.catch.write(catch_machi)

