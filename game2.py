"""ROX2026 GAME2。

実行: python3 game2.py
このファイルにGAME2の動きと停止条件を直接書く。
"""

from __future__ import annotations

import time

import camera_hensuu
import hensuu
import robot_actions
from rox_mecanum import (
    AprilTagDetector,
    Button,
    GameStatusSite,
    ModeController,
    MotionCommand,
    RobotRuntime,
    TagStore,
    VisionWorker,
    add_manual_command,
    choose_panel_target,
    face_target_command,
    midpoint,
    open_camera,
    robot_center_horizontal_error,
)


# ==================================================
# ここはGAME2の実際の動き。必要なら直接書き換える。
# ==================================================

# Tagを中心へ寄せるときの速さ・許容範囲。
AUTO_SPEED = 0.20
CENTER_GAIN = 0.45
CENTER_TOLERANCE = 0.08
DISTANCE_TOLERANCE_M = 0.08

# Tagの中央・角度合わせはGAME1と同じ共通設定を使う。
TAG_CENTER_TOLERANCE = camera_hensuu.tag_center_tolerance
TAG_CENTER_STABLE_SEC = camera_hensuu.tag_center_stable_sec
TAG_ROTATE_GAIN = camera_hensuu.tag_rotate_gain
TAG_ROTATE_MAX_SPEED = camera_hensuu.tag_rotate_max_speed

# ↑を押してからTag14〜22を見つけるまでの前進設定。
# 見つからないまま走り続けないよう、最大時間を必ず決めておく。
TAG_SEARCH_SPEED = 0.15
TAG_SEARCH_TIMEOUT_SEC = 5.0
# 複数Tagの片方が一瞬見えなくなった時、停止せずその場で再読取りを待つ時間。
TAG_REACQUIRE_TIMEOUT_SEC = 1.0

LIFT_TIMEOUT_SEC = 8.0

# パネルのTag番号。発射する段の優先順は上 → 中央 → 下。
PANEL_ROWS = {
    "middle": (17, 18, 19),
    "top": (14, 15, 16),
    "bottom": (20, 21, 22),
}
PANEL_PRIORITY = ("top", "middle", "bottom")

# 段ごとの照準距離[m]。前進を止めて横スライド照準へ切り替える距離。
# 中段・上段・下段で当たりやすい距離を、それぞれ実射して入力する。
AIM_DISTANCE_M = {
    "middle": 1.20,
    "top": 1.20,
    "bottom": 1.20,
}

def main() -> None:
    print("GAME2: タッチパッド=手動/自動")
    print("  手動: CREATE/×=地面姿勢, ○=掴む, □=排出, △=持上げ, R1=発射")
    print("  自動: ↑=Tag14〜22を探して照準開始, △=持上げ, L1=ソレノイド発射")
    runtime = None
    camera = None
    status_site = None
    vision_worker = None

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
        status_site = GameStatusSite("GAME2", hensuu.dashboard_port, hensuu.dashboard_camera_hz)
        vision_worker = VisionWorker(
            camera, detector, tags, status_site,
            camera_hz=hensuu.dashboard_camera_hz, tag_hz=hensuu.dashboard_tag_hz,
        )
        vision_worker.start()
        print(f"状態監視サイト: {status_site.url()}")

        stage = "補給後待ち: ↑で照準開始"
        target_ids = None
        target_row = None
        lift_action = None
        tag_search_started_at = None
        yaw_aligned_since = None
        target_missing_since = None
        shown_stage = None

        while True:
            loop_started = time.monotonic()
            state = runtime.controller.read()

            # カメラ・Tag処理は別スレッド。手動中は映像表示だけにする。
            vision_worker.set_paused(
                not mode.auto_enabled
                and (state.left_stick.magnitude > 0.25 or state.right_stick.magnitude > 0.25)
            )
            vision_worker.set_tag_detection_enabled(mode.auto_enabled)
            camera_error = vision_worker.error

            # OPTIONSは最優先。すべて停止して終了する。
            if state.was_pressed(Button.OPTIONS):
                print("OPTIONS: 非常停止")
                robot_actions.all_off()
                runtime.emergency_stop()
                break

            # タッチパッドで完全手動 / 自動を切り替える。
            if mode.update(state):
                if lift_action is not None:
                    robot_actions.cancel_ball_lift_for_shot(lift_action, runtime)
                    lift_action = None
                runtime.servos.hold_all_current()
                stage = "補給後待ち: ↑で照準開始"
                target_ids = None
                target_row = None
                tag_search_started_at = None
                yaw_aligned_since = None
                target_missing_since = None
                print(f"モード: {'自動' if mode.auto_enabled else '完全手動'}")

            # ボールは手動で装填済み。↑を押したら、機構の到達待ちをせず
            # 直ちにTag14〜22を探しながら前進する。
            # タッチパッドを先に押していなくても、↑は明示的な自動開始として扱う。
            if (
                lift_action is None
                and state.was_pressed(Button.DPAD_UP)
                and (
                    stage == "補給後待ち: ↑で照準開始"
                    or stage.startswith("自動停止:")
                )
            ):
                if mode.enable_auto():
                    print("↑: 自動モードへ切替えて照準を開始します")
                target_ids = None
                target_row = None
                yaw_aligned_since = None
                target_missing_since = None
                tag_search_started_at = time.monotonic()
                stage = "Tag14〜22を探索しながら前進中"

            auto = MotionCommand.stop()

            if camera_error:
                stage = "自動停止: カメラエラー"

            # 完全手動中はGAME3と同じ機構操作を使える。
            # カメラや自動照準の状態に関係なく、スティック走行もできる。
            if not mode.auto_enabled:
                if state.was_pressed(Button.R1):
                    robot_actions.ball_fire(runtime)

                if state.was_pressed(Button.CREATE):
                    if lift_action is not None:
                        robot_actions.cancel_ball_lift_for_shot(lift_action, runtime)
                        lift_action = None
                    robot_actions.game3_ground_pose(runtime)
                    stage = "完全手動: 地面走行姿勢へ"
                elif state.was_pressed(Button.CROSS) and lift_action is None:
                    robot_actions.game3_ground_pose(runtime)
                    stage = "完全手動: 地面走行姿勢へ"
                elif state.was_pressed(Button.CIRCLE):
                    robot_actions.game3_grab(runtime)
                    stage = "完全手動: 掴む姿勢"
                elif state.was_pressed(Button.SQUARE):
                    robot_actions.game3_release(runtime)
                    stage = "完全手動: 排出姿勢"
                elif state.was_pressed(Button.TRIANGLE):
                    lift_action = robot_actions.start_ball_lift_for_shot(runtime)
                    stage = "完全手動: 持上げ中"

                if lift_action is not None and lift_action.update():
                    lift_action = None
                    stage = "完全手動: 持上げ完了"

            if mode.auto_enabled and not camera_error:
                # Tagが見えるまで前進する。段は上→中央→下の順で選ぶ。
                if stage == "Tag14〜22を探索しながら前進中":
                    choice = choose_panel_target(
                        tags,
                        PANEL_ROWS,
                        camera_hensuu.tag_max_age_sec,
                        priority=PANEL_PRIORITY,
                    )
                    if choice is not None and choice.row in AIM_DISTANCE_M:
                        target_ids = choice.tag_ids
                        target_row = choice.row
                        yaw_aligned_since = None
                        target_missing_since = None
                        stage = f"{choice.row}段: 正面へ向き合わせ中"
                    elif tag_search_started_at is None or time.monotonic() - tag_search_started_at >= TAG_SEARCH_TIMEOUT_SEC:
                        stage = "自動停止: Tag14〜22を見つけられない"
                    else:
                        auto = MotionCommand(forward=TAG_SEARCH_SPEED)

                # Tagを画面中央へ寄せて、ロボットの向きを合わせる。
                # 単眼・魚眼補正なしのTag面yawは値が揺れやすいため、GAME2では
                # 「中央に安定して見えていること」を正面判定に使う。
                # 画面端のTagは探索に使えても、ここでは角度補正に使わない。
                elif target_row is not None and stage == f"{target_row}段: 正面へ向き合わせ中":
                    first = tags.get(target_ids[0], camera_hensuu.tag_max_age_sec) if target_ids else None
                    last = tags.get(target_ids[-1], camera_hensuu.tag_max_age_sec) if target_ids else None
                    target = first if target_ids and len(target_ids) == 1 else midpoint(first, last) if first and last else None
                    if target is None:
                        if target_missing_since is None:
                            target_missing_since = loop_started
                        vision_worker.request_tag_read()
                        if loop_started - target_missing_since >= TAG_REACQUIRE_TIMEOUT_SEC:
                            stage = "自動停止: 標的Tagを1秒間再取得できない"
                    else:
                        target_missing_since = None
                        horizontal_error = robot_center_horizontal_error(
                            target,
                            camera_lateral_offset_m=camera_hensuu.camera_lateral_offset_m,
                            focal_length_px=camera_hensuu.camera_focal_length_px,
                        )
                        if abs(horizontal_error) > TAG_CENTER_TOLERANCE:
                            yaw_aligned_since = None
                            auto = face_target_command(
                                target,
                                center_tolerance=TAG_CENTER_TOLERANCE,
                                rotation_gain=TAG_ROTATE_GAIN,
                                maximum_speed=TAG_ROTATE_MAX_SPEED,
                                horizontal_error=horizontal_error,
                            )
                        elif yaw_aligned_since is None:
                            yaw_aligned_since = loop_started
                        elif loop_started - yaw_aligned_since >= TAG_CENTER_STABLE_SEC:
                            stage = f"{target_row}段: 指定距離へ前進中"

                # 段別に設定した距離へ前後だけで合わせる。
                # 横ずれは次の段階でだけ補正するため、狙いを横に動かさない。
                elif target_row is not None and stage == f"{target_row}段: 指定距離へ前進中":
                    first = tags.get(target_ids[0], camera_hensuu.tag_max_age_sec) if target_ids else None
                    last = tags.get(target_ids[-1], camera_hensuu.tag_max_age_sec) if target_ids else None
                    target = first if target_ids and len(target_ids) == 1 else midpoint(first, last) if first and last else None
                    distance = AIM_DISTANCE_M.get(target_row)
                    if target is None:
                        if target_missing_since is None:
                            target_missing_since = loop_started
                        vision_worker.request_tag_read()
                        if loop_started - target_missing_since >= TAG_REACQUIRE_TIMEOUT_SEC:
                            stage = "自動停止: 標的Tagを1秒間再取得できない"
                    elif target.distance_m is None:
                        target_missing_since = None
                        stage = "自動停止: Tag距離が読めない。1m補正を確認"
                    elif distance is None:
                        target_missing_since = None
                        stage = "自動停止: この段の発射距離が未設定"
                    else:
                        target_missing_since = None
                        horizontal_error = robot_center_horizontal_error(
                            target,
                            camera_lateral_offset_m=camera_hensuu.camera_lateral_offset_m,
                            focal_length_px=camera_hensuu.camera_focal_length_px,
                        )
                        if (
                            abs(horizontal_error) > TAG_CENTER_TOLERANCE
                        ):
                            yaw_aligned_since = None
                            stage = f"{target_row}段: 正面へ向き合わせ中"
                        else:
                            auto = MotionCommand(
                                forward=max(-AUTO_SPEED, min(AUTO_SPEED, (target.distance_m - distance) * CENTER_GAIN)),
                            )
                            if abs(target.distance_m - distance) <= DISTANCE_TOLERANCE_M:
                                stage = "横スライド照準中"

                # 前後移動を止め、横だけで中心に合わせる。
                elif stage == "横スライド照準中":
                    first = tags.get(target_ids[0], camera_hensuu.tag_max_age_sec) if target_ids else None
                    last = tags.get(target_ids[-1], camera_hensuu.tag_max_age_sec) if target_ids else None
                    target = first if target_ids and len(target_ids) == 1 else midpoint(first, last) if first and last else None
                    if target is None:
                        if target_missing_since is None:
                            target_missing_since = loop_started
                        vision_worker.request_tag_read()
                        if loop_started - target_missing_since >= TAG_REACQUIRE_TIMEOUT_SEC:
                            stage = "自動停止: 標的Tagを1秒間再取得できない"
                    else:
                        target_missing_since = None
                        horizontal_error = robot_center_horizontal_error(
                            target,
                            camera_lateral_offset_m=camera_hensuu.camera_lateral_offset_m,
                            focal_length_px=camera_hensuu.camera_focal_length_px,
                        )
                        auto = MotionCommand(strafe=horizontal_error * CENTER_GAIN)
                        if abs(horizontal_error) <= CENTER_TOLERANCE:
                            stage = "照準完了: △で持上げ"

                # △: GAME3と共通の持上げ動作。
                elif stage == "照準完了: △で持上げ" and state.was_pressed(Button.TRIANGLE):
                    lift_action = robot_actions.start_ball_lift_for_shot(runtime)
                    stage = "発射高さへ持上げ中"

                elif stage == "発射高さへ持上げ中":
                    # 持上げ中だけCREATEは「スクリーンショット側のボタンでリセット」。
                    # 通常時のCREATEは、従来どおり照準開始に使う。
                    if state.was_pressed(Button.CREATE):
                        robot_actions.cancel_ball_lift_for_shot(lift_action, runtime)
                        lift_action = None
                        target_ids = None
                        target_row = None
                        tag_search_started_at = None
                        yaw_aligned_since = None
                        target_missing_since = None
                        stage = "リセット中: 地面ドリブル姿勢へ"
                    elif lift_action is not None and lift_action.update():
                        lift_action = None
                        stage = "発射準備完了: L1で発射"

                elif stage == "リセット中: 地面ドリブル姿勢へ":
                    if runtime.servos.catch.is_at_target() and runtime.servos.lift.is_at_target():
                        stage = "補給後待ち: ↑で照準開始"

                # L1: GAME3と共通の発射動作を実行する。
                elif stage == "発射準備完了: L1で発射" and state.was_pressed(Button.L1):
                    robot_actions.ball_fire(runtime)
                    stage = "発射完了: 手動で戻る"

            # 自動速度へ手動スティックを足す。完全手動なら手動だけになる。
            command = add_manual_command(auto, runtime.manual_command(state), mode.auto_enabled)
            runtime.mecanum.drive(command)
            status_site.update(
                runtime=runtime,
                state=state,
                stage=stage,
                mode=mode.mode,
                tags=tags,
                tag_max_age_sec=camera_hensuu.tag_max_age_sec,
                camera_lateral_offset_m=camera_hensuu.camera_lateral_offset_m,
                camera_focal_length_px=camera_hensuu.camera_focal_length_px,
                camera_error=camera_error,
            )

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
