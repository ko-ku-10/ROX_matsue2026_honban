"""ROX2026 GAME3 / 操作練習。

実行: python3 game3.py
動きの中身は robot_actions.py に自分で書く。
"""

from __future__ import annotations

import time
from enum import Enum

import camera_hensuu
import hensuu
import robot_actions
from rox_mecanum import Button, GameStatusSite, MotionCommand, RobotRuntime, TagStore, VisionWorker, open_camera


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
    LIFTING = "発射台へ持上げ中（CREATEでリセット）"
    FIRED = "動作完了: ×で地面姿勢へ戻す"


def main() -> None:
    print("GAME3: CREATE=地面姿勢・持上げ中リセット / ○=掴む / □=排出 / △=持上げ / R1=発射 / OPTIONS=停止")
    runtime = None
    camera = None
    status_site = None
    vision_worker = None

    try:
        runtime = RobotRuntime.open()
        robot_actions.setup_gpio()
        status_site = GameStatusSite("GAME3", hensuu.dashboard_port, hensuu.dashboard_camera_hz)
        print(f"状態監視サイト: {status_site.url()}")
        tags = TagStore()
        camera_error = ""
        try:
            # GAME3はカメラ異常でも練習走行を止めない。サイトにエラーだけ表示する。
            camera = open_camera(
                backend=camera_hensuu.camera_backend,
                device=camera_hensuu.camera_device,
                pipe_id=camera_hensuu.mipi_pipe_id,
                host_index=camera_hensuu.mipi_host_index,
                fps=camera_hensuu.mipi_fps,
                width=camera_hensuu.mipi_width,
                height=camera_hensuu.mipi_height,
            )
        except Exception as error:
            camera = None
            camera_error = str(error)
        if camera is not None:
            vision_worker = VisionWorker(
                camera, None, tags, status_site,
                camera_hz=hensuu.dashboard_camera_hz, tag_hz=hensuu.dashboard_tag_hz,
            )
            vision_worker.start()

        stage = Stage.WAIT
        move_started = time.monotonic()
        lift_action = None
        shown_stage = None
        while True:
            loop_started = time.monotonic()
            state = runtime.controller.read()

            # GAME3はTagを使わない。映像表示は別スレッドで行う。
            if vision_worker is not None:
                vision_worker.set_paused(
                    state.left_stick.magnitude > 0.25 or state.right_stick.magnitude > 0.25
                )
                camera_error = vision_worker.error

            # OPTIONSは最優先。GPIOもモーターも停止して終了する。
            if state.was_pressed(Button.OPTIONS):
                print("OPTIONS: 非常停止")
                robot_actions.all_off()
                runtime.emergency_stop()
                break

            # R1: GAME2と共通の「発射して戻す」動作。
            if state.was_pressed(Button.R1):
                robot_actions.ball_fire(runtime)

            # CREATE: 地面走行姿勢へ戻す。持上げ中なら即時中断する。
            if state.was_pressed(Button.CREATE):
                if lift_action is not None:
                    robot_actions.cancel_ball_lift_for_shot(lift_action, runtime)
                    lift_action = None
                robot_actions.game3_ground_pose(runtime)
                stage = Stage.GROUND
                move_started = loop_started

            # ×は持上げが終わった後だけ、地面走行姿勢へ戻す。
            elif state.was_pressed(Button.CROSS) and lift_action is None:
                robot_actions.game3_ground_pose(runtime)
                stage = Stage.GROUND
                move_started = loop_started

            # ○: 掴む動作。
            elif state.was_pressed(Button.CIRCLE):
                robot_actions.game3_grab(runtime)
                stage = Stage.GRAB
                move_started = loop_started

            # □: 排出動作。
            elif state.was_pressed(Button.SQUARE):
                robot_actions.game3_release(runtime)
                stage = Stage.RELEASE
                move_started = loop_started

            # △: GAME2と共通の持上げ動作。
            elif state.was_pressed(Button.TRIANGLE):
                lift_action = robot_actions.start_ball_lift_for_shot(runtime)
                stage = Stage.LIFTING

            # 持上げ中も毎周期×を読める。完了時だけ発射可能へ進む。
            if lift_action is not None and lift_action.update():
                lift_action = None
                stage = Stage.FIRED

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

            # 姿勢の移動中・待機中でも、常にスティックで手動走行できる。
            command = runtime.manual_command(state)
            if state.left_stick.magnitude < STICK_DEADZONE:
                command = MotionCommand(rotate=command.rotate)
            if state.right_stick.magnitude < STICK_DEADZONE:
                command = MotionCommand(forward=command.forward, strafe=command.strafe)
            if command == MotionCommand.stop():
                runtime.mecanum.stop()
            else:
                runtime.mecanum.drive(command)

            status_site.update(
                runtime=runtime,
                state=state,
                stage=stage,
                mode=None,
                tags=tags,
                tag_max_age_sec=camera_hensuu.tag_max_age_sec,
                camera_lateral_offset_m=camera_hensuu.camera_lateral_offset_m,
                camera_focal_length_px=camera_hensuu.camera_focal_length_px,
                camera_error=camera_error,
            )

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
        if vision_worker is not None:
            vision_worker.stop()
        if status_site is not None:
            status_site.close()
        if camera is not None:
            camera.close()
        if runtime is not None:
            runtime.close()
        robot_actions.close_gpio()


if __name__ == "__main__":
    main()
