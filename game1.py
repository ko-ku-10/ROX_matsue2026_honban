"""ROX2026 GAME1。

実行: python3 game1.py

手動走行を基本にして、ゲートを通る時だけTag 8を使う。
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
    face_target_command,
    open_camera,
    robot_center_horizontal_error,
)


# ==================================================
# Tag 8のゲートを通る時だけの設定。
# ==================================================

TAG_GATE = 8

# Tag 8を画面中央へ向ける時の設定。
# 画像中心からのずれ。小さいほど正面を厳密に合わせる。
TAG_CENTER_TOLERANCE = 0.03
# 一瞬だけ中心を横切った場合に前進しないよう、正面を保つ必要がある時間。
TAG_CENTER_STABLE_SEC = 0.30
TAG_ROTATE_GAIN = 0.60
TAG_ROTATE_MAX_SPEED = 0.20
# Tag面の向きも正面にする設定。カメラ未校正時は近似値になる。
TAG_YAW_TOLERANCE_DEG = 4.0
TAG_YAW_GAIN = 0.020
# 実機で角度合わせが逆に回る時だけ -1.0 に変更する。
TAG_YAW_DIRECTION = 1.0

# Tag8の正面へ近づく設定。距離はカメラが読み取った値[m]。
TAG8_TARGET_DISTANCE_M = 1.0
TAG8_DISTANCE_TOLERANCE_M = 0.08
TAG8_DISTANCE_GAIN = 0.45

# Tag8の正面1m地点から、ゲートを通る設定。
# 車輪距離を読んでいないため、2m通過は「速度 × 時間」で実現する。
# 実機で2m進む時間を測って TAG8_FINAL_FORWARD_SEC だけ書き換える。
TAG8_FINAL_FORWARD_DISTANCE_M = 2.0
# この機体では負の値が「ゲートへ進む」向き。
TAG8_FORWARD_SPEED = -0.20
TAG8_FINAL_FORWARD_SEC = 2.0
# ↑を押してからTag8を待つ最大時間。カメラ・検出スレッドの初回更新も待てるよう長めにする。
TAG8_SEARCH_TIMEOUT_SEC = 2.0


def main() -> None:
    print("GAME1: 手動走行 + Tag8ゲート通過支援")
    print("  タッチパッド: 手動 / 自動を切替")
    print("  手動モードの○: catchを掴む / □: catchを開く")
    print("  自動モードの↑: Tag8正面へ移動 → 距離1m → 2m通過")
    print("  OPTIONS: 非常停止")

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
        status_site = GameStatusSite("GAME1", hensuu.dashboard_port, hensuu.dashboard_camera_hz)
        vision_worker = VisionWorker(
            camera, detector, tags, status_site,
            camera_hz=hensuu.dashboard_camera_hz, tag_hz=hensuu.dashboard_tag_hz,
        )
        vision_worker.start()
        print(f"状態監視サイト: {status_site.url()}")

        stage = "手動走行"
        forward_until = 0.0
        tag_search_until = 0.0
        centered_since = None
        yaw_aligned_since = None
        shown_stage = None

        while True:
            loop_started = time.monotonic()
            state = runtime.controller.read()

            if state.was_pressed(Button.OPTIONS):
                print("OPTIONS: 非常停止")
                robot_actions.all_off()
                runtime.emergency_stop()
                break

            # タッチパッドを押したら、途中のゲート通過は即座に取り消す。
            if mode.update(state):
                forward_until = 0.0
                centered_since = None
                yaw_aligned_since = None
                stage = "自動待機: ↑でTag8ゲート通過" if mode.auto_enabled else "手動走行"
                print(f"モード: {'自動' if mode.auto_enabled else '完全手動'}")

            auto = MotionCommand.stop()

            # 手動走行中はカメラを一切読まない。重いAprilTag検出による
            # スティック操作のラグを防ぐ。↑を押した時と中心合わせ中だけ読む。
            start_gate = mode.auto_enabled and state.was_pressed(Button.DPAD_UP)
            needs_tag_frame = start_gate or stage in {
                "Tag8を検出中",
                "Tag8を画面中央へ合わせ中",
                "Tag8の角度を正面へ合わせ中",
                "Tag8正面の1.0m地点へ移動中",
            }
            # カメラ処理は別スレッド。ここでは有効化だけ行い、操縦入力は待たない。
            vision_worker.set_paused(
                state.left_stick.magnitude > 0.05 or state.right_stick.magnitude > 0.05
            )
            vision_worker.set_tag_detection_enabled(needs_tag_frame)
            if start_gate:
                vision_worker.request_tag_read()
            camera_error = vision_worker.error

            # GAME1の完全手動ではcatchだけを操作できる。
            # liftにはここから一切命令を出さない。
            if not mode.auto_enabled:
                if state.was_pressed(Button.CIRCLE):
                    runtime.servos.catch.write(robot_actions.catch_hozi)
                    stage = "手動: catchを掴む姿勢へ"
                elif state.was_pressed(Button.SQUARE):
                    runtime.servos.catch.write(robot_actions.catch_machi)
                    stage = "手動: catchを開く姿勢へ"

            # ↑は操縦者の「ゲートを通る」承認ボタン。
            if start_gate:
                centered_since = None
                yaw_aligned_since = None
                if camera_error:
                    stage = "自動停止: カメラエラー"
                else:
                    # 読取りは別スレッドなので、初回フレームを待つ時間を確保する。
                    tag_search_until = loop_started + TAG8_SEARCH_TIMEOUT_SEC
                    stage = "Tag8を検出中"

            if mode.auto_enabled:
                if stage == "Tag8を検出中":
                    tag8 = tags.get(TAG_GATE, camera_hensuu.tag_max_age_sec)
                    if camera_error:
                        stage = "自動停止: カメラエラー"
                    elif tag8 is not None:
                        stage = "Tag8を画面中央へ合わせ中"
                    elif loop_started >= tag_search_until:
                        stage = "自動停止: Tag8が見えない"

                elif stage == "Tag8を画面中央へ合わせ中":
                    tag8 = tags.get(TAG_GATE, camera_hensuu.tag_max_age_sec)
                    if camera_error:
                        stage = "自動停止: カメラエラー"
                    elif tag8 is None:
                        stage = "自動停止: Tag8を見失った"
                    else:
                        horizontal_error = robot_center_horizontal_error(
                            tag8,
                            camera_lateral_offset_m=camera_hensuu.camera_lateral_offset_m,
                            focal_length_px=camera_hensuu.camera_focal_length_px,
                        )
                        auto = face_target_command(
                            tag8,
                            center_tolerance=TAG_CENTER_TOLERANCE,
                            rotation_gain=TAG_ROTATE_GAIN,
                            maximum_speed=TAG_ROTATE_MAX_SPEED,
                            horizontal_error=horizontal_error,
                        )
                        if abs(horizontal_error) > TAG_CENTER_TOLERANCE:
                            centered_since = None
                        elif centered_since is None:
                            centered_since = loop_started
                        elif loop_started - centered_since >= TAG_CENTER_STABLE_SEC:
                            yaw_aligned_since = None
                            stage = "Tag8の角度を正面へ合わせ中"

                elif stage == "Tag8の角度を正面へ合わせ中":
                    tag8 = tags.get(TAG_GATE, camera_hensuu.tag_max_age_sec)
                    if camera_error:
                        stage = "自動停止: カメラエラー"
                    elif tag8 is None or tag8.yaw_degrees is None:
                        # 角度が不明なままでは「真正面」と判定しない。
                        stage = "自動停止: Tag8の角度を読めない"
                    else:
                        horizontal_error = robot_center_horizontal_error(
                            tag8,
                            camera_lateral_offset_m=camera_hensuu.camera_lateral_offset_m,
                            focal_length_px=camera_hensuu.camera_focal_length_px,
                        )
                        if abs(horizontal_error) > TAG_CENTER_TOLERANCE:
                            centered_since = None
                            yaw_aligned_since = None
                            stage = "Tag8を画面中央へ合わせ中"
                        elif abs(tag8.yaw_degrees) > TAG_YAW_TOLERANCE_DEG:
                            yaw_aligned_since = None
                            yaw_speed = max(
                                -TAG_ROTATE_MAX_SPEED,
                                min(TAG_ROTATE_MAX_SPEED, tag8.yaw_degrees * TAG_YAW_GAIN * TAG_YAW_DIRECTION),
                            )
                            auto = MotionCommand(rotate=yaw_speed)
                        elif yaw_aligned_since is None:
                            yaw_aligned_since = loop_started
                        elif loop_started - yaw_aligned_since >= TAG_CENTER_STABLE_SEC:
                            stage = "Tag8正面の1.0m地点へ移動中"

                elif stage == "Tag8正面の1.0m地点へ移動中":
                    tag8 = tags.get(TAG_GATE, camera_hensuu.tag_max_age_sec)
                    if camera_error:
                        stage = "自動停止: カメラエラー"
                    elif tag8 is None or tag8.distance_m is None or tag8.yaw_degrees is None:
                        stage = "自動停止: Tag8・距離・角度を見失った"
                    else:
                        horizontal_error = robot_center_horizontal_error(
                            tag8,
                            camera_lateral_offset_m=camera_hensuu.camera_lateral_offset_m,
                            focal_length_px=camera_hensuu.camera_focal_length_px,
                        )
                        turn = face_target_command(
                            tag8,
                            center_tolerance=TAG_CENTER_TOLERANCE,
                            rotation_gain=TAG_ROTATE_GAIN,
                            maximum_speed=TAG_ROTATE_MAX_SPEED,
                            horizontal_error=horizontal_error,
                        )
                        distance_error = tag8.distance_m - TAG8_TARGET_DISTANCE_M
                        if abs(horizontal_error) > TAG_CENTER_TOLERANCE or abs(tag8.yaw_degrees) > TAG_YAW_TOLERANCE_DEG:
                            # 正面からずれた時は、前後に動かず、最初の正面合わせからやり直す。
                            centered_since = None
                            yaw_aligned_since = None
                            auto = turn
                            stage = "Tag8を画面中央へ合わせ中"
                        elif abs(distance_error) <= TAG8_DISTANCE_TOLERANCE_M:
                            forward_until = loop_started + TAG8_FINAL_FORWARD_SEC
                            stage = (
                                f"Tag8正面1m到達: {TAG8_FINAL_FORWARD_DISTANCE_M:.1f}m通過中 "
                                f"(残り{TAG8_FINAL_FORWARD_SEC:.1f}秒)"
                            )
                        else:
                            # 遠ければTag8へ、近すぎればTag8から離れる。
                            # TAG8_FORWARD_SPEED の符号だけで実機の前後方向を合わせる。
                            maximum_speed = abs(TAG8_FORWARD_SPEED)
                            auto = MotionCommand(
                                forward=max(-maximum_speed, min(
                                    maximum_speed,
                                    distance_error * TAG8_DISTANCE_GAIN * (-1.0 if TAG8_FORWARD_SPEED < 0.0 else 1.0),
                                )),
                            )

                elif stage.startswith("Tag8正面1m到達"):
                    if loop_started >= forward_until:
                        stage = "Tag8ゲート通過完了: 手動で続行"
                    else:
                        auto = MotionCommand(forward=TAG8_FORWARD_SPEED)

            # 自動中もスティックで微調整できる。手動モードではスティックだけ使う。
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
                print(f"[{mode.mode.value}] {stage}")
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
