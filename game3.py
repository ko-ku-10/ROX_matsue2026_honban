"""ROX2026 GAME3兼操作練習プログラム。

実行: python3 game3.py

左スティック: メカナム移動 / R2 + 右スティック: 旋回
CREATE: ボールを地面に付けた走行姿勢
○: catchを掴む角度 / □: catchをRobot外へ出す角度
△: game3_hensuu.pyに書いた順で持上げ→ソレノイド発射 / ×: 地面走行姿勢へ戻す
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
    MOTIAGE = "持上げ動作中"
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
        settled_at = None
        previous_stage = None
        motiage_step_index = 0

        def enter(next_stage: Stage) -> None:
            nonlocal stage, stage_started, settled_at
            stage = next_stage
            stage_started = time.monotonic()
            settled_at = None

        def ready_after_settle(at_target: bool, now: float) -> bool:
            """目標角度に到達後、反動が収まるまで設定時間だけ待つ。"""
            nonlocal settled_at
            if not at_target:
                settled_at = None
                return False
            if settled_at is None:
                settled_at = now
                return False
            return now - settled_at >= cfg.mechanism_settle_sec

        def reached(servo: object, target_angle: float) -> bool:
            """GAME3用の実機到達判定。PIDの細かすぎる停止範囲には依存しない。"""
            current = servo.read()
            return current is not None and abs(current - target_angle) <= cfg.sequence_target_tolerance_deg

        def start_motiage_step() -> None:
            """game3_hensuu.pyの1行を実行する。安全な到達確認は下で行う。"""
            motor_name, angle = cfg.motiage_steps[motiage_step_index]
            if motor_name not in {"lift", "catch"}:
                raise ValueError("motiage_stepsのモーター名は 'lift' または 'catch' にしてください")
            getattr(runtime.servos, motor_name).write(float(angle))
            print(f"持上げ {motiage_step_index + 1}/{len(cfg.motiage_steps)}: {motor_name} -> {angle}度")

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
                # 編集する順番は game3_hensuu.py の motiage_steps だけ。
                if not cfg.motiage_steps:
                    raise ValueError("motiage_stepsが空です。最低1行は書いてください")
                motiage_step_index = 0
                start_motiage_step()
                enter(Stage.MOTIAGE)

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
            elif stage is Stage.MOTIAGE:
                motor_name, angle = cfg.motiage_steps[motiage_step_index]
                servo = getattr(runtime.servos, motor_name)
                if ready_after_settle(reached(servo, float(angle)), started):
                    motiage_step_index += 1
                    if motiage_step_index == len(cfg.motiage_steps):
                        runtime.fire()
                        print("持上げ完了: ソレノイド ON")
                        stage = Stage.FIRED
                    else:
                        start_motiage_step()
                        # 各行ごとに最大待機時間を数え直す。
                        enter(Stage.MOTIAGE)

            if stage in {
                Stage.MOTIAGE,
            } and started - stage_started > cfg.mechanism_target_timeout_sec:
                stage = Stage.FAULT
                motor_name, angle = cfg.motiage_steps[motiage_step_index]
                servo = getattr(runtime.servos, motor_name)
                print(
                    "連続動作が目標角度に到達しません。 "
                    f"{motor_name}={servo.read()}度 target={angle}度"
                )

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
