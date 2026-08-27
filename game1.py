"""ROX2026 GAME1。

実行: python3 game1.py

手動走行を基本にして、ゲートを通る時だけTag 8を使う。
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
    face_target_command,
    open_camera,
)


# ==================================================
# Tag 8のゲートを通る時だけの設定。
# ==================================================

TAG_GATE = 8

# Tag 8を画面中央へ向ける時の設定。
TAG_CENTER_TOLERANCE = 0.08
TAG_ROTATE_GAIN = 0.60
TAG_ROTATE_MAX_SPEED = 0.20

# Tag 8を中央へ合わせた後に、まっすぐ進む設定。
# 実機で測って、この2つだけを書き換える。
# この機体では負の値が「ゲートへ進む」向き。
TAG8_FORWARD_SPEED = -0.20
TAG8_FORWARD_SEC = 2.0


def main() -> None:
    print("GAME1: 手動走行 + Tag8ゲート通過支援")
    print("  タッチパッド: 手動 / 自動を切替")
    print("  手動モードの○: catchを掴む / □: catchを開く")
    print("  自動モードの↑: Tag8へ正面合わせ後、2秒間前進")
    print("  OPTIONS: 非常停止")

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

        stage = "手動走行"
        forward_until = 0.0
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
                stage = "自動待機: ↑でTag8ゲート通過" if mode.auto_enabled else "手動走行"
                print(f"モード: {'自動' if mode.auto_enabled else '完全手動'}")

            auto = MotionCommand.stop()

            # 手動走行中はカメラを一切読まない。重いAprilTag検出による
            # スティック操作のラグを防ぐ。↑を押した時と中心合わせ中だけ読む。
            start_gate = mode.auto_enabled and state.was_pressed(Button.DPAD_UP)
            needs_tag_frame = start_gate or stage == "Tag8を画面中央へ合わせ中"
            camera_error = ""
            if needs_tag_frame:
                try:
                    tags.update(detector.detect(camera.read()))
                except Exception as error:
                    camera_error = str(error)

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
                if camera_error:
                    stage = "自動停止: カメラエラー"
                elif tags.get(TAG_GATE, camera_hensuu.tag_max_age_sec) is None:
                    # Tagが見えない時に勝手に探して走らない。
                    stage = "自動停止: Tag8が見えない"
                else:
                    stage = "Tag8を画面中央へ合わせ中"

            if mode.auto_enabled:
                if stage == "Tag8を画面中央へ合わせ中":
                    tag8 = tags.get(TAG_GATE, camera_hensuu.tag_max_age_sec)
                    if camera_error:
                        stage = "自動停止: カメラエラー"
                    elif tag8 is None:
                        stage = "自動停止: Tag8を見失った"
                    else:
                        auto = face_target_command(
                            tag8,
                            center_tolerance=TAG_CENTER_TOLERANCE,
                            rotation_gain=TAG_ROTATE_GAIN,
                            maximum_speed=TAG_ROTATE_MAX_SPEED,
                        )
                        if auto == MotionCommand.stop():
                            forward_until = loop_started + TAG8_FORWARD_SEC
                            stage = f"Tag8へ前進中: 残り{TAG8_FORWARD_SEC:.1f}秒"

                elif stage.startswith("Tag8へ前進中"):
                    if loop_started >= forward_until:
                        stage = "Tag8ゲート通過完了: 手動で続行"
                    else:
                        auto = MotionCommand(forward=TAG8_FORWARD_SPEED)

            # 自動中もスティックで微調整できる。手動モードではスティックだけ使う。
            command = add_manual_command(auto, runtime.manual_command(state), mode.auto_enabled)
            runtime.mecanum.drive(command)

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
        if camera is not None:
            camera.close()
        if runtime is not None:
            runtime.close()
        robot_actions.close_gpio()


if __name__ == "__main__":
    main()
