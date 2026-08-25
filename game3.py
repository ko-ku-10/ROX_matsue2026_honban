"""ROX2026 GAME3 / 操作練習。

実行: python3 game3.py
動きの中身は robot_actions.py に自分で書く。
"""

from __future__ import annotations

import time
from enum import Enum

import robot_actions
from rox_mecanum import Button, MotionCommand, RobotRuntime


# スティックの微妙なずれを無視する範囲。勝手に走るなら少し上げる。
STICK_DEADZONE = 0.25

# CREATE / ○ / □で出したサーボ目標を待つ最大時間[秒]。
# 超えた場合も非常停止せず、次の段階へ進む。
MOVE_TIMEOUT_SEC = 8.0


class Stage(str, Enum):
    WAIT = "手動走行・練習待ち"
    GROUND = "地面走行姿勢へ移動中"
    DRIVE = "ドリブル走行可能"
    GRAB = "catchを掴む角度へ移動中"
    RELEASE = "catchを排出角度へ移動中"
    FIRED = "動作完了: ×で地面姿勢へ戻す"


def main() -> None:
    print("GAME3: CREATE=地面姿勢 / ○=掴む / □=排出 / △=持上げ / R1=発射 / OPTIONS=停止")
    runtime = None

    try:
        runtime = RobotRuntime.open()
        robot_actions.setup_gpio()

        stage = Stage.WAIT
        move_started = time.monotonic()
        shown_stage = None
        # 走行可能へ移った直後は、両スティックが一度ニュートラルになるまで走らない。
        drive_neutral_confirmed = False

        while True:
            loop_started = time.monotonic()
            state = runtime.controller.read()

            # OPTIONSは最優先。GPIOもモーターも停止して終了する。
            if state.was_pressed(Button.OPTIONS):
                print("OPTIONS: 非常停止")
                robot_actions.all_off()
                runtime.emergency_stop()
                break

            # R1: GAME2と共通の「発射して戻す」動作。
            if state.was_pressed(Button.R1):
                robot_actions.ball_fire(runtime)

            # CREATE または ×: 地面走行姿勢へ戻す。
            if state.was_pressed(Button.CREATE) or state.was_pressed(Button.CROSS):
                robot_actions.game3_ground_pose(runtime)
                stage = Stage.GROUND
                move_started = loop_started
                drive_neutral_confirmed = False

            # ○: 掴む動作。
            elif state.was_pressed(Button.CIRCLE):
                robot_actions.game3_grab(runtime)
                stage = Stage.GRAB
                move_started = loop_started
                drive_neutral_confirmed = False

            # □: 排出動作。
            elif state.was_pressed(Button.SQUARE):
                robot_actions.game3_release(runtime)
                stage = Stage.RELEASE
                move_started = loop_started
                drive_neutral_confirmed = False

            # △: GAME2と共通の持上げ動作。
            elif state.was_pressed(Button.TRIANGLE):
                robot_actions.ball_lift_for_shot(runtime)
                stage = Stage.FIRED
                drive_neutral_confirmed = False

            # 地面姿勢へ両方が到着した時だけ、スティック走行を許可する。
            if stage is Stage.GROUND:
                if (
                    robot_actions.is_within_move_tolerance(runtime.servos.catch)
                    and robot_actions.is_within_move_tolerance(runtime.servos.lift)
                ):
                    stage = Stage.DRIVE
                elif loop_started - move_started > MOVE_TIMEOUT_SEC:
                    print("地面走行姿勢の到達確認はできません。走行可能へ進みます")
                    stage = Stage.DRIVE

            # ○は掴む姿勢で待機する。□のmachi姿勢は到着後も走行できる。
            elif stage is Stage.GRAB or stage is Stage.RELEASE:
                if robot_actions.is_within_move_tolerance(runtime.servos.catch):
                    if stage is Stage.RELEASE:
                        stage = Stage.DRIVE
                    else:
                        stage = Stage.WAIT
                elif loop_started - move_started > MOVE_TIMEOUT_SEC:
                    print("catchの到達確認はできません。次の段階へ進みます")
                    if stage is Stage.RELEASE:
                        stage = Stage.DRIVE
                    else:
                        stage = Stage.WAIT

            # 地面走行姿勢に着いた時だけ、手動でメカナムを動かせる。
            if stage is Stage.DRIVE:
                if not drive_neutral_confirmed:
                    # スティックを離したことを確認するまでは、必ず停止する。
                    runtime.mecanum.stop()
                    if (
                        state.left_stick.magnitude < STICK_DEADZONE
                        and state.right_stick.magnitude < STICK_DEADZONE
                    ):
                        drive_neutral_confirmed = True
                        print("スティックのニュートラルを確認しました。走行できます")
                else:
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
                runtime.mecanum.stop()

            if stage is not shown_stage:
                print(f"[{stage.value}]")
                shown_stage = stage

            time.sleep(max(0.0, 1.0 / 50.0 - (time.monotonic() - loop_started)))

    except KeyboardInterrupt:
        if runtime is not None:
            robot_actions.all_off()
            runtime.emergency_stop()
    finally:
        # エラー、Ctrl+C、OPTIONS、通常終了の全てでGPIOをLOWにする。
        robot_actions.all_off()
        if runtime is not None:
            runtime.close()
        robot_actions.close_gpio()


if __name__ == "__main__":
    main()
