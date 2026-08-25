"""ROX2026 GAME2。

実行: python3 game2.py
このファイルにGAME2の動きと停止条件を直接書く。
"""

from __future__ import annotations

import time

import camera_hensuu
import robot_actions
from rox_mecanum import (
    AprilTagDetector,
    Button,
    ModeController,
    MotionCommand,
    RobotRuntime,
    TagStore,
    add_manual_command,
    choose_panel_target,
    midpoint,
    open_camera,
)


# ==================================================
# ここはGAME2の実際の動き。必要なら直接書き換える。
# ==================================================

# Tagを中心へ寄せるときの速さ・許容範囲。
AUTO_SPEED = 0.20
CENTER_GAIN = 0.45
CENTER_TOLERANCE = 0.08
DISTANCE_TOLERANCE_M = 0.08

LIFT_TIMEOUT_SEC = 8.0

# 発射後に補給場所へ戻る動き。
RETREAT_SPEED = 0.20
RETREAT_TIME_SEC = 0.0

# パネルのTag番号。中央段 → 上段 → 下段の順に狙う。
PANEL_ROWS = {
    "middle": (17, 18, 19),
    "top": (14, 15, 16),
    "bottom": (20, 21, 22),
}

# 各段を撃つとき、Tagまで残す距離[m]。
# 実射して当たりやすい距離をここへ入れる。
SHOT_DISTANCE_M = {
    "middle": 1.20,
    "top": 1.20,
    "bottom": 1.20,
}

def main() -> None:
    print("GAME2: タッチパッド=手動/自動, CREATE=照準, △=持上げ, L2=発射, ×=後退, OPTIONS=停止")
    runtime = None
    camera = None

    try:
        runtime = RobotRuntime.open()
        robot_actions.setup_gpio()
        camera = open_camera(
            backend=camera_hensuu.camera_backend,
            device=camera_hensuu.camera_device,
            pipe_id=camera_hensuu.mipi_pipe_id,
            host_index=camera_hensuu.mipi_host_index,
            fps=camera_hensuu.mipi_fps,
            width=camera_hensuu.mipi_width,
            height=camera_hensuu.mipi_height,
        )
        detector = AprilTagDetector(camera_hensuu.apriltag_size_m, camera_hensuu.camera_focal_length_px)
        tags = TagStore()
        mode = ModeController()

        stage = "補給待ち: CREATEで照準開始"
        target_ids = None
        target_row = None
        retreat_until = 0.0
        shown_stage = None

        while True:
            loop_started = time.monotonic()
            state = runtime.controller.read()

            # カメラが失敗した場合、今の自動動作は止める。
            try:
                tags.update(detector.detect(camera.read()))
                camera_error = ""
            except Exception as error:
                camera_error = str(error)

            # OPTIONSは最優先。すべて停止して終了する。
            if state.was_pressed(Button.OPTIONS):
                print("OPTIONS: 非常停止")
                robot_actions.all_off()
                runtime.emergency_stop()
                break

            # タッチパッドで完全手動 / 自動を切り替える。
            if mode.update(state):
                runtime.servos.hold_all_current()
                stage = "補給待ち: CREATEで照準開始"
                target_ids = None
                target_row = None
                retreat_until = 0.0
                print(f"モード: {'自動' if mode.auto_enabled else '完全手動'}")

            auto = MotionCommand.stop()

            if camera_error:
                stage = "自動停止: カメラエラー"

            if mode.auto_enabled and not camera_error:
                # CREATE: 見えているパネルから、中央→上→下の順に標的を決める。
                if stage == "補給待ち: CREATEで照準開始" and state.was_pressed(Button.CREATE):
                    choice = choose_panel_target(tags, PANEL_ROWS, camera_hensuu.tag_max_age_sec)
                    if choice is None:
                        stage = "自動停止: Tag14〜22が見えない"
                    elif choice.row not in SHOT_DISTANCE_M:
                        stage = "自動停止: 段の発射距離が未設定"
                    else:
                        target_ids = choice.tag_ids
                        target_row = choice.row
                        robot_actions.game2_ground_pose(runtime)
                        stage = "地面走行姿勢へ移動中"

                # ボールを地面に付ける姿勢に着くまで、自動走行しない。
                elif stage == "地面走行姿勢へ移動中":
                    if runtime.servos.catch.is_at_target() and runtime.servos.lift.is_at_target():
                        stage = "標的へ前進中"

                # Tagを中心・指定距離へ近づける。
                elif stage == "標的へ前進中":
                    first = tags.get(target_ids[0], camera_hensuu.tag_max_age_sec) if target_ids else None
                    last = tags.get(target_ids[-1], camera_hensuu.tag_max_age_sec) if target_ids else None
                    target = first if target_ids and len(target_ids) == 1 else midpoint(first, last) if first and last else None
                    distance = SHOT_DISTANCE_M.get(target_row)
                    if target is None or target.distance_m is None or distance is None:
                        stage = "自動停止: 標的Tagまたは距離が読めない"
                    else:
                        auto = MotionCommand(
                            forward=max(-AUTO_SPEED, min(AUTO_SPEED, (target.distance_m - distance) * CENTER_GAIN)),
                            strafe=target.horizontal_error * CENTER_GAIN,
                        )
                        if abs(target.distance_m - distance) <= DISTANCE_TOLERANCE_M:
                            stage = "横スライド照準中"

                # 前後移動を止め、横だけで中心に合わせる。
                elif stage == "横スライド照準中":
                    first = tags.get(target_ids[0], camera_hensuu.tag_max_age_sec) if target_ids else None
                    last = tags.get(target_ids[-1], camera_hensuu.tag_max_age_sec) if target_ids else None
                    target = first if target_ids and len(target_ids) == 1 else midpoint(first, last) if first and last else None
                    if target is None:
                        stage = "自動停止: 標的Tagが見えない"
                    else:
                        auto = MotionCommand(strafe=target.horizontal_error * CENTER_GAIN)
                        if abs(target.horizontal_error) <= CENTER_TOLERANCE:
                            stage = "照準完了: △で持上げ"

                # △: GAME3と共通の持上げ動作。
                elif stage == "照準完了: △で持上げ" and state.was_pressed(Button.TRIANGLE):
                    robot_actions.ball_lift_for_shot(runtime)
                    lift_started = loop_started
                    stage = "発射高さへ持上げ中"

                elif stage == "発射高さへ持上げ中":
                    if runtime.servos.lift.is_at_target():
                        stage = "発射準備完了: L2で発射"
                    elif loop_started - lift_started > LIFT_TIMEOUT_SEC:
                        print("liftが発射高さに到達しません。安全停止します")
                        runtime.emergency_stop()
                        break

                # L2: GAME3と共通の発射動作を実行する。
                elif stage == "発射準備完了: L2で発射" and state.was_pressed(Button.L2):
                    robot_actions.ball_fire(runtime)
                    stage = "発射済み: ×で後退"

                # ×: liftを下ろし、地面走行姿勢へ戻す。
                elif stage == "発射済み: ×で後退" and state.was_pressed(Button.CROSS):
                    robot_actions.game2_ground_pose(runtime)
                    stage = "liftを下げ中"

                elif stage == "liftを下げ中":
                    if runtime.servos.catch.is_at_target() and runtime.servos.lift.is_at_target():
                        if RETREAT_TIME_SEC <= 0.0:
                            stage = "自動停止: 後退時間が0秒"
                        else:
                            retreat_until = loop_started + RETREAT_TIME_SEC
                            stage = "補給地点へ後退中"

                elif stage == "補給地点へ後退中":
                    if loop_started >= retreat_until:
                        stage = "補給待ち: CREATEで照準開始"
                        target_ids = None
                        target_row = None
                    else:
                        auto = MotionCommand.backward(RETREAT_SPEED)

            # 自動速度へ手動スティックを足す。完全手動なら手動だけになる。
            command = add_manual_command(auto, runtime.manual_command(state), mode.auto_enabled)
            runtime.mecanum.drive(command)

            if stage != shown_stage:
                print(f"[{mode.mode.value}] {stage}  target={target_ids} row={target_row}")
                shown_stage = stage

            time.sleep(max(0.0, 1.0 / 50.0 - (time.monotonic() - loop_started)))

    except KeyboardInterrupt:
        if runtime is not None:
            robot_actions.all_off()
            runtime.emergency_stop()
    finally:
        robot_actions.all_off()
        if camera is not None:
            camera.close()
        if runtime is not None:
            runtime.close()
        robot_actions.close_gpio()


if __name__ == "__main__":
    main()
