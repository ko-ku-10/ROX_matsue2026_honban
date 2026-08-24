"""ROX2026 GAME3兼操作練習プログラム。

実行: python3 game3.py

左スティック: メカナム移動 / R2 + 右スティック: 旋回
CREATE: ボールを地面に付けた走行姿勢
○: catchを掴む角度 / □: catchをRobot外へ出す角度
△: 掴む→持上げ→ソレノイド発射の連続動作 / ×: 地面走行姿勢へ戻す
R1: ソレノイド単体テスト
OPTIONS: 非常停止して終了
"""

from __future__ import annotations

import time
from enum import Enum

import game3_hensuu as cfg
from rox_mecanum import Button, MotionCommand, RobotRuntime
from rox_mecanum import BallMechanism


class Stage(str, Enum):
    IDLE = "手動走行・練習待ち"
    TRANSPORTING = "地面保持姿勢へ移動中"
    DRIBBLE = "ドリブル走行可能"
    GRABBING = "catchを掴む角度へ移動中"
    RELEASING = "catchを排出角度へ移動中"
    LIFT_FIRST = "liftを110度へ移動中"
    CATCH_GRAB = "catchを-70度へ移動中"
    LIFT_AFTER_GRAB = "liftを20度へ移動中"
    CATCH_RELEASE = "catchを0度へ移動中"
    LIFT_FIRE = "liftを110度へ移動中（発射準備）"
    FIRED = "発射済み: ×で地面姿勢へ戻す"
    FAULT = "安全停止"


def main() -> None:
    print("GAME3 / 操作練習: OPTIONS=非常停止")
    runtime = None
    try:
        runtime = RobotRuntime.open(with_solenoid=True)
        mechanism = BallMechanism(runtime.servos)
        stage = Stage.IDLE
        stage_started = time.monotonic()
        previous_stage = None

        def enter(next_stage: Stage) -> None:
            nonlocal stage, stage_started
            stage = next_stage
            stage_started = time.monotonic()

        while True:
            started = time.monotonic()
            state = runtime.controller.read()
            if state.was_pressed(Button.OPTIONS):
                print("OPTIONS: 非常停止")
                break

            # 操作練習用。機構の姿勢に関係なく、設定時間だけソレノイドをONにする。
            # 実戦用の発射操作は下の「READY_TO_FIRE + L2」のまま分離する。
            if state.was_pressed(Button.R1):
                runtime.fire()
                print("ソレノイド単体テスト: ON")

            # CREATE/×はいつでも連続動作を中止し、走行姿勢へ戻す。
            # catch/liftが両方到達するまで車輪は停止する。
            if state.was_pressed(Button.CREATE) or state.was_pressed(Button.CROSS):
                runtime.set_ball_transport_pose()
                stage = Stage.TRANSPORTING
                stage_started = started

            # catch角度の単体確認。機構が動く間は車輪を止める。
            elif state.was_pressed(Button.CIRCLE):
                mechanism.grab()
                stage = Stage.GRABBING
                stage_started = started
            elif state.was_pressed(Button.SQUARE):
                mechanism.release()
                stage = Stage.RELEASING
                stage_started = started

            # 発射姿勢はcatchを排出角度、liftを発射高さへ同時に動かす。
            elif state.was_pressed(Button.TRIANGLE):
                # 前のmotiage.pyで作った順番を、待ち時間ではなく実測角度で進める。
                runtime.servos.lift.write(cfg.sequence_lift_first_angle)
                enter(Stage.LIFT_FIRST)

            if stage is Stage.TRANSPORTING:
                if runtime.ball_transport_pose_ready():
                    stage = Stage.DRIBBLE
                elif started - stage_started > cfg.mechanism_target_timeout_sec:
                    stage = Stage.FAULT
                    print("地面保持姿勢に到達しません。角度・PID・CAN通信を確認してください")
            elif stage is Stage.GRABBING:
                if runtime.servos.catch.is_at_target():
                    stage = Stage.IDLE
            elif stage is Stage.RELEASING:
                if runtime.servos.catch.is_at_target():
                    stage = Stage.IDLE
            elif stage is Stage.LIFT_FIRST and runtime.servos.lift.is_at_target():
                runtime.servos.catch.write(cfg.sequence_catch_grab_angle)
                enter(Stage.CATCH_GRAB)
            elif stage is Stage.CATCH_GRAB and runtime.servos.catch.is_at_target():
                runtime.servos.lift.write(cfg.sequence_lift_after_grab_angle)
                enter(Stage.LIFT_AFTER_GRAB)
            elif stage is Stage.LIFT_AFTER_GRAB and runtime.servos.lift.is_at_target():
                runtime.servos.catch.write(cfg.sequence_catch_release_angle)
                enter(Stage.CATCH_RELEASE)
            elif stage is Stage.CATCH_RELEASE and runtime.servos.catch.is_at_target():
                runtime.servos.lift.write(cfg.lift_fire_angle)
                enter(Stage.LIFT_FIRE)
            elif stage is Stage.LIFT_FIRE and runtime.servos.lift.is_at_target():
                runtime.fire()
                print("発射: ソレノイド ON")
                stage = Stage.FIRED

            if stage in {
                Stage.LIFT_FIRST,
                Stage.CATCH_GRAB,
                Stage.LIFT_AFTER_GRAB,
                Stage.CATCH_RELEASE,
                Stage.LIFT_FIRE,
            } and started - stage_started > cfg.mechanism_target_timeout_sec:
                stage = Stage.FAULT
                print("連続動作が目標角度に到達しません。角度・PID・CAN通信を確認してください")

            # 暴走防止: 中立付近のスティックずれは無視し、停止フレームを連続送信する。
            if stage is Stage.DRIBBLE:
                command = runtime.manual_command(state)
                if state.left_stick.magnitude < cfg.manual_stick_deadzone:
                    command = MotionCommand(rotate=command.rotate)
                if state.right_stick.magnitude < cfg.manual_stick_deadzone:
                    command = MotionCommand(forward=command.forward, strafe=command.strafe)
                if command == MotionCommand.stop():
                    runtime.mecanum.stop()
                else:
                    runtime.mecanum.drive(command)
            else:
                runtime.mecanum.stop()
            runtime.update_outputs()

            if stage is not previous_stage:
                print(f"[{stage.value}]")
                previous_stage = stage
            time.sleep(max(0.0, 1.0 / 50.0 - (time.monotonic() - started)))
    except KeyboardInterrupt:
        pass
    finally:
        if runtime is not None:
            runtime.close()


if __name__ == "__main__":
    main()
