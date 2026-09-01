"""あなたがロボットの動きを自由に書くファイル。

GAME1〜3は、このファイルにある関数を呼ぶだけです。
CAN通信・PID・メカナムは書かなくてよいですが、lift/catch/GPIOの動きは
前の motiage.py と同じように自由に書けます。
"""

import time
from dataclasses import dataclass

from robot_lights import lights

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
# liftは起動時に地面ドリブル位置のストッパーを0度として原点合わせする。
lift_orosu = 0
# 以前の「下ろす106度 → 持上げ20度」の差を、ストッパー原点へ換算した仮値。
# 実機で発射台に合う角度を確認してから自由に変える。
lift_motiage = -95
catch_hozi = 38
catch_machi = 0
# 地面で保持する角度とは別に、持上げ中にボールを保持できる角度。
catch_motiage = 47

# 指令した角度を待つ最大時間。超えたら停止せず次の動作へ進む。
# 動作完了の判定には使わず、エンコーダーの実測角度で判定する。
SERVO_MOVE_TIMEOUT_SEC = 1.0

# 持上げ手順で「到達」とみなすエンコーダー実測誤差。
# PIDの通常保持精度は hensuu.py の値のまま変えない。
SERVO_MOVE_TOLERANCE_DEG = 3.0

# 持上げ中だけ使うliftの強さ。強い衝撃や負荷でモーター保護が働かないよう、
# 初期値は控えめな60%。遅すぎなければこのまま使い、必要時だけ少しずつ上げる。
# 持上げ終了後は、hensuu.py の通常PID上限へ自動で戻る。
LIFT_MOVE_SPEED_PERCENT = 60
_configured_pins = set()


def update_lights():
    """LEDアニメーションを1コマ進める。GAMEの操作ループから呼ばれる。"""
    lights.update()


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
    GPIO.output(CYLINDER_RETRACT_PIN, GPIO.LOW)
    GPIO.output(CYLINDER_RETRACT_PIN, GPIO.LOW)
    lights.standby()


def all_off():
    """OPTIONS・例外・終了時に必ず呼ばれる。2個ともOFFにする。"""
    lights.off()
    if GPIO is None:
        return
    for pin in _configured_pins:
        GPIO.output(pin, GPIO.LOW)


def close_gpio():
    """GAME終了時のGPIO片付け。通常は自分で呼ばなくてよい。"""
    all_off()
    lights.close()
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
    lights.ground()


def game2_ground_pose(runtime):
    """GAME2: 地面にボールを付けて走る姿勢。"""
    servos = runtime.servos
    servos.lift.write(lift_orosu)
    servos.catch.write(catch_hozi)
    lights.ground()


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


@dataclass
class BallLiftAction:
    """GAME2・GAME3共通の持上げ動作。update()を繰り返すと少しずつ進む。"""

    runtime: object
    step: int = 0
    step_started: float = 0.0
    normal_lift_speed_percent: float = 0.0
    finished: bool = False

    def __post_init__(self):
        self.normal_lift_speed_percent = self.runtime.servos.lift.config.max_speed * 100.0
        self.runtime.servos.set_pid("lift", max_speed_percent=LIFT_MOVE_SPEED_PERCENT)
        self.step_started = time.monotonic()
        lights.lifting(self.step)
        self._send_step_target()

    def _steps(self):
        servos = self.runtime.servos
        return (
            (servos.lift, lift_orosu, "liftを下ろす"),
            (servos.catch, catch_motiage, "catchを持上げ用の角度にする"),
            (servos.lift, lift_motiage, "liftで発射台へ運ぶ"),
            (servos.catch, catch_machi, "catchで発射台へ載せる"),
            (servos.lift, lift_orosu, "liftを下ろす"),
        )

    def _send_step_target(self):
        servo, angle, name = self._steps()[self.step]
        servo.write(angle)
        print(f"持上げ {self.step + 1}/5: {name} ({angle}度)")

    def update(self):
        """1回だけ到達を確認する。完了した時だけTrueを返す。"""
        if self.finished:
            return True

        servo, _angle, name = self._steps()[self.step]
        if is_within_move_tolerance(servo):
            self.step += 1
            if self.step >= len(self._steps()):
                self.finish()
                lights.ready_to_fire()
                print("持上げ動作が完了しました")
                return True
            self.step_started = time.monotonic()
            lights.lifting(self.step)
            self._send_step_target()
        elif time.monotonic() - self.step_started >= SERVO_MOVE_TIMEOUT_SEC:
            print(f"{name}: 到達確認なし。次の動作へ進みます")
            self.step += 1
            if self.step >= len(self._steps()):
                self.finish()
                lights.ready_to_fire()
                return True
            self.step_started = time.monotonic()
            lights.lifting(self.step)
            self._send_step_target()
        return False

    def finish(self):
        """持上げ中だけ上げたliftの速度上限を通常値へ戻す。"""
        if not self.finished:
            self.runtime.servos.set_pid("lift", max_speed_percent=self.normal_lift_speed_percent)
            self.finished = True

    def cancel(self):
        """途中動作を終了する。機構を戻す角度の命令は呼び出し側が出す。"""
        self.finish()


def start_ball_lift_for_shot(runtime):
    """中断可能な持上げ動作を開始する。GAME2・GAME3から使う。"""
    return BallLiftAction(runtime)


def cancel_ball_lift_for_shot(action, runtime):
    """×で持上げを中断し、ボールを地面で保持する姿勢へ戻す。"""
    action.cancel()
    runtime.servos.lift.write(lift_orosu)
    runtime.servos.catch.write(catch_hozi)
    lights.ground()


def ball_lift_for_shot(runtime):
    """互換用の待機型持上げ。GAME本体ではstart_ball_lift_for_shot()を使う。"""
    action = start_ball_lift_for_shot(runtime)
    while not action.update():
        time.sleep(0.02)


def ball_fire(runtime):
    """GAME2・GAME3共通: 発射後、戻す側をONのままにして戻り位置を保持する。"""
    # 戻す側を先にOFFにし、両方OFFの時間を作ってから発射する。
    lights.firing()
    GPIO.output(CYLINDER_RETRACT_PIN, GPIO.LOW)
    time.sleep(CYLINDER_SWITCH_OFF_SEC)
    GPIO.output(CYLINDER_EXTEND_PIN, GPIO.HIGH)
    time.sleep(0.5)

    # 発射側をOFFにし、両方OFFの時間を作ってから戻す側へ切り替える。
    GPIO.output(CYLINDER_EXTEND_PIN, GPIO.LOW)
    time.sleep(CYLINDER_SWITCH_OFF_SEC)
    GPIO.output(CYLINDER_RETRACT_PIN, GPIO.HIGH)
    time.sleep(0.5)
    # 戻す側はOFFにしない。待機中もシリンダーを戻った位置に保つ。
    GPIO.output(CYLINDER_RETRACT_PIN, GPIO.LOW)
    GPIO.output(CYLINDER_EXTEND_PIN, GPIO.LOW)
    lights.ground()


def game3_ground_pose(runtime):
    """GAME3: 地面走行姿勢。"""
    servos = runtime.servos
    servos.lift.write(lift_orosu)
    servos.catch.write(catch_hozi)
    lights.ground()


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

