"""ROX2026 GAME3 / 操作練習。

実行: python3 game3.py

このファイルにGAME3の動きを直接書く。
ライブラリは通信・PID・非常停止だけを担当する。
"""

from __future__ import annotations

import time
from enum import Enum

from rox_mecanum import Button, MotionCommand, RobotRuntime


# ==================================================
# ここはGAME3の実際の動き。必要なら直接書き換える。
# ==================================================

# 地面にボールを付けて走る姿勢。
GROUND_CATCH_ANGLE = 0.0
GROUND_LIFT_ANGLE = 0.0

# ○で掴む姿勢、□で排出する姿勢。
GRAB_CATCH_ANGLE = 0.0
RELEASE_CATCH_ANGLE = 0.0

# △で実行する持上げの角度。
LIFT_FIRST_ANGLE = 110.0
CATCH_GRAB_ANGLE = -70.0
LIFT_AFTER_GRAB_ANGLE = 20.0
CATCH_RELEASE_ANGLE = 0.0
LIFT_FIRE_ANGLE = 110.0

# 最後にソレノイドをONにする時間[秒]。
SOLENOID_ON_TIME_SEC = 0.3

# 到達判定と安全停止の設定。
TARGET_ERROR_DEG = 3.0
SETTLE_TIME_SEC = 0.5
MOVE_TIMEOUT_SEC = 8.0
STICK_DEADZONE = 0.18


class Stage(str, Enum):
    WAIT = "手動走行・練習待ち"
    GROUND = "地面走行姿勢へ移動中"
    DRIVE = "ドリブル走行可能"
    GRAB = "catchを掴む角度へ移動中"
    RELEASE = "catchを排出角度へ移動中"
    LIFT_FIRST = "liftを110度へ移動中"
    CATCH_GRAB = "catchを-70度へ移動中"
    LIFT_AFTER_GRAB = "liftを20度へ移動中"
    CATCH_RELEASE = "catchを0度へ移動中"
    LIFT_FIRE = "liftを110度へ移動中（発射準備）"
    FIRED = "発射済み: ×で地面姿勢へ戻す"


def main() -> None:
    print("GAME3: CREATE=地面姿勢 / ○=掴む / □=排出 / △=持上げ+発射 / R1=ソレノイド / OPTIONS=停止")
    runtime = None

    try:
        runtime = RobotRuntime.open(with_solenoid=True)
        stage = Stage.WAIT
        move_started = time.monotonic()
        reached_at = None
        shown_stage = None

        while True:
            loop_started = time.monotonic()
            state = runtime.controller.read()

            # OPTIONSは最優先。全モーターとソレノイドを止めて終了する。
            if state.was_pressed(Button.OPTIONS):
                print("OPTIONS: 非常停止")
                runtime.emergency_stop()
                break

            # R1: ソレノイドだけを一回動かす。lift/catchは動かさない。
            if state.was_pressed(Button.R1):
                if runtime.solenoid is None:
                    raise RuntimeError("ソレノイドが開かれていません")
                runtime.solenoid.pulse(SOLENOID_ON_TIME_SEC)
                print(f"ソレノイド ON: {SOLENOID_ON_TIME_SEC}秒")

            # CREATE または ×: いつでも連続動作を中止し、地面走行姿勢へ戻す。
            if state.was_pressed(Button.CREATE) or state.was_pressed(Button.CROSS):
                runtime.servos.catch.write(GROUND_CATCH_ANGLE)
                runtime.servos.lift.write(GROUND_LIFT_ANGLE)
                stage = Stage.GROUND
                move_started = loop_started
                reached_at = None

            # ○: catchだけを掴む角度へ動かす。
            elif state.was_pressed(Button.CIRCLE):
                runtime.servos.catch.write(GRAB_CATCH_ANGLE)
                stage = Stage.GRAB
                move_started = loop_started

            # □: catchだけを排出角度へ動かす。
            elif state.was_pressed(Button.SQUARE):
                runtime.servos.catch.write(RELEASE_CATCH_ANGLE)
                stage = Stage.RELEASE
                move_started = loop_started

            # △: 前のmotiage.pyと同じ順番で、1つ目のlift移動を始める。
            elif state.was_pressed(Button.TRIANGLE):
                runtime.servos.lift.write(LIFT_FIRST_ANGLE)
                stage = Stage.LIFT_FIRST
                move_started = loop_started
                reached_at = None

            # 地面姿勢へ両方が到着した時だけ、スティック走行を許可する。
            if stage is Stage.GROUND:
                if runtime.servos.catch.is_at_target() and runtime.servos.lift.is_at_target():
                    stage = Stage.DRIVE
                elif loop_started - move_started > MOVE_TIMEOUT_SEC:
                    print("地面走行姿勢に到達しません。安全停止します")
                    runtime.emergency_stop()
                    break

            # ○ / □ はcatchが目標へ着いたら待機へ戻る。
            elif stage is Stage.GRAB or stage is Stage.RELEASE:
                if runtime.servos.catch.is_at_target():
                    stage = Stage.WAIT
                elif loop_started - move_started > MOVE_TIMEOUT_SEC:
                    print("catchが目標角度に到達しません。安全停止します")
                    runtime.emergency_stop()
                    break

            # 以下は△の持上げ動作。1つずつ到着確認してから次の行を書く。
            elif stage is Stage.LIFT_FIRST:
                angle = runtime.servos.lift.read()
                if angle is not None and abs(angle - LIFT_FIRST_ANGLE) <= TARGET_ERROR_DEG:
                    if reached_at is None:
                        reached_at = loop_started
                    elif loop_started - reached_at >= SETTLE_TIME_SEC:
                        runtime.servos.catch.write(CATCH_GRAB_ANGLE)
                        stage = Stage.CATCH_GRAB
                        move_started = loop_started
                        reached_at = None
                else:
                    reached_at = None

            elif stage is Stage.CATCH_GRAB:
                angle = runtime.servos.catch.read()
                if angle is not None and abs(angle - CATCH_GRAB_ANGLE) <= TARGET_ERROR_DEG:
                    if reached_at is None:
                        reached_at = loop_started
                    elif loop_started - reached_at >= SETTLE_TIME_SEC:
                        runtime.servos.lift.write(LIFT_AFTER_GRAB_ANGLE)
                        stage = Stage.LIFT_AFTER_GRAB
                        move_started = loop_started
                        reached_at = None
                else:
                    reached_at = None

            elif stage is Stage.LIFT_AFTER_GRAB:
                angle = runtime.servos.lift.read()
                if angle is not None and abs(angle - LIFT_AFTER_GRAB_ANGLE) <= TARGET_ERROR_DEG:
                    if reached_at is None:
                        reached_at = loop_started
                    elif loop_started - reached_at >= SETTLE_TIME_SEC:
                        runtime.servos.catch.write(CATCH_RELEASE_ANGLE)
                        stage = Stage.CATCH_RELEASE
                        move_started = loop_started
                        reached_at = None
                else:
                    reached_at = None

            elif stage is Stage.CATCH_RELEASE:
                angle = runtime.servos.catch.read()
                if angle is not None and abs(angle - CATCH_RELEASE_ANGLE) <= TARGET_ERROR_DEG:
                    if reached_at is None:
                        reached_at = loop_started
                    elif loop_started - reached_at >= SETTLE_TIME_SEC:
                        runtime.servos.lift.write(LIFT_FIRE_ANGLE)
                        stage = Stage.LIFT_FIRE
                        move_started = loop_started
                        reached_at = None
                else:
                    reached_at = None

            elif stage is Stage.LIFT_FIRE:
                angle = runtime.servos.lift.read()
                if angle is not None and abs(angle - LIFT_FIRE_ANGLE) <= TARGET_ERROR_DEG:
                    if reached_at is None:
                        reached_at = loop_started
                    elif loop_started - reached_at >= SETTLE_TIME_SEC:
                        if runtime.solenoid is None:
                            raise RuntimeError("ソレノイドが開かれていません")
                        runtime.solenoid.pulse(SOLENOID_ON_TIME_SEC)
                        print("持上げ完了: ソレノイド ON")
                        stage = Stage.FIRED
                else:
                    reached_at = None

            # 持上げ途中で1つの角度に着かなければ、安全停止する。
            if stage in {
                Stage.LIFT_FIRST,
                Stage.CATCH_GRAB,
                Stage.LIFT_AFTER_GRAB,
                Stage.CATCH_RELEASE,
                Stage.LIFT_FIRE,
            } and loop_started - move_started > MOVE_TIMEOUT_SEC:
                print("持上げが目標角度に到達しません。安全停止します")
                runtime.emergency_stop()
                break

            # 地面走行姿勢に着いたときだけ、手動でメカナムを動かせる。
            if stage is Stage.DRIVE:
                command = runtime.manual_command(state)
                if state.left_stick.magnitude < STICK_DEADZONE:
                    command = MotionCommand(rotate=command.rotate)
                if state.right_stick.magnitude < STICK_DEADZONE:
                    command = MotionCommand(forward=command.forward, strafe=command.strafe)
                if command == MotionCommand.stop():
                    runtime.mecanum.stop()
                else:
                    runtime.mecanum.drive(command)
            else:
                # 機構を動かす間・待機中・発射後は必ず車輪を止め続ける。
                runtime.mecanum.stop()

            runtime.update_outputs()

            if stage is not shown_stage:
                print(f"[{stage.value}]")
                shown_stage = stage

            time.sleep(max(0.0, 1.0 / 50.0 - (time.monotonic() - loop_started)))

    except KeyboardInterrupt:
        if runtime is not None:
            runtime.emergency_stop()
    finally:
        # エラー、Ctrl+C、OPTIONS、通常終了の全てで停止してから閉じる。
        if runtime is not None:
            runtime.close()


if __name__ == "__main__":
    main()
