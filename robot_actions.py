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
# 実機配線: GPIO17が発射（伸ばす側）、GPIO27が戻す側。
CYLINDER_EXTEND_PIN = 17
CYLINDER_RETRACT_PIN = 27
# 片側をOFFにしてから反対側をONにするまでの安全な待機時間。
CYLINDER_SWITCH_OFF_SEC = 0.02
lift_orosu = 106
lift_motiage = 20
catch_hozi = -40
catch_machi = -17
# 地面で保持する角度とは別に、持上げ中にボールを保持できる角度。
catch_motiage = -50

# 指令した角度を待つ最大時間。超えたら停止せず次の動作へ進む。
# 動作完了の判定には使わず、エンコーダーの実測角度で判定する。
SERVO_MOVE_TIMEOUT_SEC = 8.0

# 持上げ手順で「到達」とみなすエンコーダー実測誤差。
# PIDの通常保持精度は hensuu.py の値のまま変えない。
SERVO_MOVE_TOLERANCE_DEG = 3.0

# 持上げ中だけ使うliftの強さ。大きくするほど速く強く動く。
# 持上げ終了後は、hensuu.py の通常PID上限へ自動で戻る。
LIFT_MOVE_SPEED_PERCENT = 20.0
_configured_pins = set()


def setup_gpio():
    """GAME起動時に1回だけ呼ばれる。シリンダーを戻した状態で開始する。"""
    if GPIO is None:
        raise RuntimeError("Hobot.GPIOが見つかりません。RDK X5上で実行してください")
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    for pin in (CYLINDER_EXTEND_PIN, CYLINDER_RETRACT_PIN):
        if pin is not None:
            GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
            _configured_pins.add(pin)

    # VQZ315Kを2個使う構成: 起動時は「戻す側」だけをONにして待機する。
    # 2つ同時にONには絶対にしない。
    GPIO.output(CYLINDER_EXTEND_PIN, GPIO.LOW)
    time.sleep(CYLINDER_SWITCH_OFF_SEC)
    GPIO.output(CYLINDER_RETRACT_PIN, GPIO.HIGH)


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


def is_within_move_tolerance(servo):
    """現在の実測角度が、目標±3度に入っているかを返す。"""
    current = servo.read()
    return (
        current is not None
        and abs(servo.target_angle - current) <= SERVO_MOVE_TOLERANCE_DEG
    )


def wait_until_reached(servo, name):
    """エンコーダー実測値が目標±3度へ入るまで待つ。

    ``time.sleep(1)`` のような固定時間では完了扱いにしない。
    PIDスレッドが読むRobStrideのmechPosと目標角度との差が
    ``SERVO_MOVE_TOLERANCE_DEG`` 以内になった時だけ次の行へ進む。
    """
    deadline = time.monotonic() + SERVO_MOVE_TIMEOUT_SEC

    while True:
        current = servo.read()
        if is_within_move_tolerance(servo):
            error = servo.target_angle - current
            print(
                f"{name}: エンコーダーで到達を確認しました "
                f"({current:.1f}度、誤差 {error:+.1f}度)"
            )
            return

        if time.monotonic() >= deadline:
            print(
                f"{name}: 8秒以内に到達確認できませんでした。次の動作へ進みます "
                f"(現在: {current}, 目標: {servo.target_angle})"
            )
            return
        time.sleep(0.02)


def ball_lift_for_shot(runtime):
    """GAME2・GAME3共通: ボールを発射する高さへ動かす。"""
    servos = runtime.servos
    normal_lift_speed_percent = servos.lift.config.max_speed * 100.0

    # 持上げ中だけ強くする。通常のPID保持を強くし過ぎないため。
    servos.set_pid("lift", max_speed_percent=LIFT_MOVE_SPEED_PERCENT)

    try:
        # 各行で目標を出し、エンコーダーが「届いた」と確認してから次へ進む。
        servos.lift.write(lift_orosu)
        wait_until_reached(servos.lift, "liftを下ろす")

        # 持上げ用の角度にしてから、ボールを発射台へ運ぶ。
        servos.catch.write(catch_motiage)
        wait_until_reached(servos.catch, "catchを持上げ用の角度にする")

        servos.lift.write(lift_motiage)
        wait_until_reached(servos.lift, "liftで発射台へ運ぶ")

        servos.catch.write(catch_machi)
        wait_until_reached(servos.catch, "catchで発射台へ載せる")

        servos.lift.write(lift_orosu)
        wait_until_reached(servos.lift, "liftを下ろす")
    finally:
        # 次の待機中にガタガタしないよう、通常の保持上限へ戻す。
        servos.set_pid("lift", max_speed_percent=normal_lift_speed_percent)


def ball_fire(runtime):
    """GAME2・GAME3共通: 発射後、戻す側をONのままにして戻り位置を保持する。"""
    # 戻す側を先にOFFにし、両方OFFの時間を作ってから発射する。
    GPIO.output(CYLINDER_RETRACT_PIN, GPIO.LOW)
    time.sleep(CYLINDER_SWITCH_OFF_SEC)
    GPIO.output(CYLINDER_EXTEND_PIN, GPIO.HIGH)
    time.sleep(0.05)

    # 発射側をOFFにし、両方OFFの時間を作ってから戻す側へ切り替える。
    GPIO.output(CYLINDER_EXTEND_PIN, GPIO.LOW)
    time.sleep(CYLINDER_SWITCH_OFF_SEC)
    GPIO.output(CYLINDER_RETRACT_PIN, GPIO.HIGH)
    time.sleep(0.05)
    # 戻す側はOFFにしない。待機中もシリンダーを戻った位置に保つ。
    GPIO.output(CYLINDER_EXTEND_PIN, GPIO.LOW)


def game3_ground_pose(runtime):
    """GAME3: 地面走行姿勢。"""
    servos = runtime.servos
    servos.lift.write(lift_orosu)
    servos.catch.write(catch_hozi)


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

