"""ROX2026 GAME3兼操作練習プログラム。

実行: python3 game3.py

左スティック: メカナム移動 / R2 + 右スティック: 旋回
CREATE: ボールを地面に付けた走行姿勢
○: catchを掴む角度 / □: catchをRobot外へ出す角度
△: 発射姿勢へ持上げ / L2: 発射 / ×: 地面走行姿勢へ戻す
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
    RAISING = "発射姿勢へ持上げ中"
    READY_TO_FIRE = "L2で発射可能"
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

            # 走行姿勢へ戻す。catch/liftが両方到達するまで車輪は停止する。
            if state.was_pressed(Button.CREATE) or (stage is Stage.FIRED and state.was_pressed(Button.CROSS)):
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
                if cfg.lift_fire_angle is None:
                    stage = Stage.FAULT
                    print("game3_hensuu.py の lift_fire_angle を実測値に設定してください")
                else:
                    mechanism.fire_pose(cfg.lift_fire_angle)
                    stage = Stage.RAISING
                    stage_started = started

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
            elif stage is Stage.RAISING:
                if mechanism.fire_ready():
                    stage = Stage.READY_TO_FIRE
                elif started - stage_started > cfg.mechanism_target_timeout_sec:
                    stage = Stage.FAULT
                    print("発射姿勢に到達しません。角度・PID・CAN通信を確認してください")
            elif stage is Stage.READY_TO_FIRE and state.was_pressed(Button.L2):
                runtime.fire()
                print("発射")
                stage = Stage.FIRED

            # ボールを持つ想定の時は、地面保持姿勢でのみ走行を許可する。
            if stage is Stage.DRIBBLE:
                command = runtime.manual_command(state)
            else:
                command = MotionCommand.stop()
            runtime.mecanum.drive(command)
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
