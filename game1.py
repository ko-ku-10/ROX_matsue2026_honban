"""ROX2026 GAME1。

実行: python3 game1.py
このファイルにGAME1の動きと停止条件を直接書く。
GAME1中はCREATEで開始姿勢にした後、lift/catchを動かさない。
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
    midpoint,
    open_camera,
)


# ==================================================
# ここはGAME1の実際の動き。必要なら直接書き換える。
# ==================================================

# GAME1で使うTag番号。
TAG_START_PRIMARY = 1
TAG_START_FALLBACK = 9
TAG_GATE = 8
TAG_BOARD_LEFT = 12
TAG_BOARD_RIGHT = 13
TAG_RETURN_LEFT = 6
TAG_RETURN_RIGHT = 10
TAG_GOAL = 0

# Tagを中心・目標距離へ寄せる速さと許容範囲。
AUTO_SPEED = 0.20
CENTER_GAIN = 0.45
CENTER_TOLERANCE = 0.08
DISTANCE_TOLERANCE_M = 0.08
# 画面端のTagは距離を使わず、カメラ正面へ旋回してから判断する。
FACE_TAG_ROTATION_GAIN = 0.60
FACE_TAG_MAX_SPEED = 0.20

# Tagごとに止まりたい距離[m]。実測後に変更する。
TAG8_DISTANCE_M = 1
TAG12_13_DISTANCE_M = 2.30
TAG6_10_DISTANCE_M = 0.80
TAG0_DISTANCE_M = 0.80

# 時間で行う動き。探索時間は、実機で安全な上限へ調整する。
TURN_AROUND_SPEED = 5
TURN_AROUND_SEC = 0.0
TAG8_SEARCH_SPEED = 0.20
TAG8_SEARCH_SEC = 5
TUNNEL_SEARCH_SPEED = 0.20
TUNNEL_SEARCH_SEC = 10
BOARD_PUSH_SPEED = 0.15
RETURN_THROUGH_SPEED = 0.20
RETURN_THROUGH_SEC = 10


def main() -> None:
    print("GAME1: タッチパッド=手動/自動, CREATE=展開, △=Tag8へ, ○=トンネル通過, R1長押し=板を押す, ×=上がった確認, L2=帰還")
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

        stage = "CREATEで開始姿勢へ展開"
        deployed = False
        timed_command = MotionCommand.stop()
        timed_until = 0.0
        timed_next_stage = ""
        shown_stage = None

        while True:
            loop_started = time.monotonic()
            state = runtime.controller.read()

            # カメラ異常時は自動の速度を出さない。
            try:
                tags.update(detector.detect(camera.read()))
                camera_error = ""
            except Exception as error:
                camera_error = str(error)

            # OPTIONSは最優先。全モーターを止めて終了する。
            if state.was_pressed(Button.OPTIONS):
                print("OPTIONS: 非常停止")
                robot_actions.all_off()
                runtime.emergency_stop()
                break

            # タッチパッドで完全手動 / 自動を切り替える。
            if mode.update(state):
                runtime.servos.hold_all_current()
                timed_until = 0.0
                stage = "△でTag1/9からTag8へ移動" if deployed else "CREATEで開始姿勢へ展開"
                print(f"モード: {'自動' if mode.auto_enabled else '完全手動'}")

            auto = MotionCommand.stop()

            if camera_error:
                stage = "自動停止: カメラエラー"

            if mode.auto_enabled and not camera_error:
                # CREATE: スタート時のサイズ用の姿勢へ移動する。
                # ここからGAME1終了までlift/catchには命令を出さない。
                if stage == "CREATEで開始姿勢へ展開" and state.was_pressed(Button.CREATE):
                    robot_actions.game1_start_pose(runtime)
                    stage = "lift/catchを展開中"

                elif stage == "lift/catchを展開中":
                    if runtime.servos.catch.is_at_target() and runtime.servos.lift.is_at_target():
                        runtime.servos.hold_all_current()
                        deployed = True
                        stage = "△でTag1/9からTag8へ移動"

                # △: Tag1が見えれば1、見えなければTag9を基準に反対向きへ旋回する。
                elif stage == "△でTag1/9からTag8へ移動" and state.was_pressed(Button.TRIANGLE):
                    stage = "Tag1/9を探索中"

                elif stage == "Tag1/9を探索中":
                    reference = tags.get(TAG_START_PRIMARY, camera_hensuu.tag_max_age_sec)
                    if reference is None:
                        reference = tags.get(TAG_START_FALLBACK, camera_hensuu.tag_max_age_sec)
                    if reference is not None:
                        if TURN_AROUND_SEC <= 0.0:
                            stage = "自動停止: 旋回時間が0秒"
                        else:
                            timed_command = MotionCommand(rotate=TURN_AROUND_SPEED)
                            timed_until = loop_started + TURN_AROUND_SEC
                            timed_next_stage = "Tag8を探して前進中"
                            stage = "反対向きへ旋回中"
                    else:
                        stage = "自動停止: Tag1/9が見えない"

                # 旋回・通過・押し込み・帰還は、終了時刻まで同じ速度を出す。
                elif stage == "反対向きへ旋回中":
                    if loop_started >= timed_until:
                        stage = timed_next_stage
                    else:
                        auto = timed_command

                # Tag8が見えるまで前進する。端に見えたら次の段階で正面へ向ける。
                elif stage == "Tag8を探して前進中":
                    tag8 = tags.get(TAG_GATE, camera_hensuu.tag_max_age_sec)
                    if tag8 is not None:
                        timed_until = 0.0
                        stage = "Tag8へ中心合わせ中"
                    elif timed_until == 0.0:
                        if TAG8_SEARCH_SEC <= 0.0:
                            stage = "自動停止: Tag8探索時間が0秒"
                        else:
                            timed_command = MotionCommand(forward=TAG8_SEARCH_SPEED)
                            timed_until = loop_started + TAG8_SEARCH_SEC
                    elif loop_started >= timed_until:
                        stage = "自動停止: Tag8を見つけられない"
                    else:
                        auto = timed_command

                # Tag8を画面中心・指定距離へ合わせる。
                elif stage == "Tag8へ中心合わせ中":
                    tag8 = tags.get(TAG_GATE, camera_hensuu.tag_max_age_sec)
                    if tag8 is None or tag8.distance_m is None:
                        stage = "自動停止: Tag8または距離が読めない"
                    else:
                        auto = face_target_command(
                            tag8,
                            center_tolerance=CENTER_TOLERANCE,
                            rotation_gain=FACE_TAG_ROTATION_GAIN,
                            maximum_speed=FACE_TAG_MAX_SPEED,
                        )
                        if auto == MotionCommand.stop():
                            auto = MotionCommand(
                                forward=max(-AUTO_SPEED, min(AUTO_SPEED, (tag8.distance_m - TAG8_DISTANCE_M) * CENTER_GAIN)),
                            )
                        if abs(tag8.horizontal_error) <= CENTER_TOLERANCE and abs(tag8.distance_m - TAG8_DISTANCE_M) <= DISTANCE_TOLERANCE_M:
                            stage = "○でトンネル通過"

                # ○: トンネルを通過し、先のTag12/13が見えるまで前進する。
                elif stage == "○でトンネル通過" and state.was_pressed(Button.CIRCLE):
                    if TUNNEL_SEARCH_SEC <= 0.0:
                        stage = "自動停止: トンネル探索時間が0秒"
                    else:
                        timed_until = loop_started + TUNNEL_SEARCH_SEC
                        stage = "トンネル通過中: Tag12/13を探す"

                elif stage == "トンネル通過中: Tag12/13を探す":
                    tag12 = tags.get(TAG_BOARD_LEFT, camera_hensuu.tag_max_age_sec)
                    tag13 = tags.get(TAG_BOARD_RIGHT, camera_hensuu.tag_max_age_sec)
                    if tag12 is not None and tag13 is not None:
                        stage = "Tag12/13へ2.3m位置合わせ中"
                    elif loop_started >= timed_until:
                        stage = "自動停止: Tag12/13を見つけられない"
                    else:
                        auto = MotionCommand(forward=TUNNEL_SEARCH_SPEED)

                # Tag12・13を正面へ向け、2.3m地点まで進む。
                elif stage == "Tag12/13へ2.3m位置合わせ中":
                    tag12 = tags.get(TAG_BOARD_LEFT, camera_hensuu.tag_max_age_sec)
                    tag13 = tags.get(TAG_BOARD_RIGHT, camera_hensuu.tag_max_age_sec)
                    target = midpoint(tag12, tag13) if tag12 and tag13 else None
                    if target is None or target.distance_m is None:
                        stage = "自動停止: Tag12/13または距離が読めない"
                    else:
                        auto = face_target_command(
                            target,
                            center_tolerance=CENTER_TOLERANCE,
                            rotation_gain=FACE_TAG_ROTATION_GAIN,
                            maximum_speed=FACE_TAG_MAX_SPEED,
                        )
                        if auto == MotionCommand.stop():
                            auto = MotionCommand(
                                forward=max(-AUTO_SPEED, min(AUTO_SPEED, (target.distance_m - TAG12_13_DISTANCE_M) * CENTER_GAIN)),
                            )
                        if abs(target.horizontal_error) <= CENTER_TOLERANCE and abs(target.distance_m - TAG12_13_DISTANCE_M) <= DISTANCE_TOLERANCE_M:
                            stage = "R1を押している間だけ板を押す"

                # R1を押している間だけ板を押す。離せば自動速度はゼロになる。
                elif stage == "R1を押している間だけ板を押す":
                    if state.was_pressed(Button.CROSS):
                        stage = "L2でTag6/10から帰還"
                    elif state.button(Button.R1):
                        auto = MotionCommand(forward=BOARD_PUSH_SPEED)

                # L2: Tag6/10の中間へ合わせ、Tag0へ帰る。
                elif stage == "L2でTag6/10から帰還" and state.was_pressed(Button.L2):
                    stage = "Tag6/10の中央へ中心合わせ中"

                elif stage == "Tag6/10の中央へ中心合わせ中":
                    tag6 = tags.get(TAG_RETURN_LEFT, camera_hensuu.tag_max_age_sec)
                    tag10 = tags.get(TAG_RETURN_RIGHT, camera_hensuu.tag_max_age_sec)
                    target = midpoint(tag6, tag10) if tag6 and tag10 else None
                    if target is None or target.distance_m is None:
                        stage = "自動停止: Tag6/10または距離が読めない"
                    else:
                        auto = face_target_command(
                            target,
                            center_tolerance=CENTER_TOLERANCE,
                            rotation_gain=FACE_TAG_ROTATION_GAIN,
                            maximum_speed=FACE_TAG_MAX_SPEED,
                        )
                        if auto == MotionCommand.stop():
                            auto = MotionCommand(
                                forward=max(-AUTO_SPEED, min(AUTO_SPEED, (target.distance_m - TAG6_10_DISTANCE_M) * CENTER_GAIN)),
                            )
                        if abs(target.horizontal_error) <= CENTER_TOLERANCE and abs(target.distance_m - TAG6_10_DISTANCE_M) <= DISTANCE_TOLERANCE_M:
                            if RETURN_THROUGH_SEC <= 0.0:
                                stage = "自動停止: 帰還通過時間が0秒"
                            else:
                                timed_command = MotionCommand(forward=RETURN_THROUGH_SPEED)
                                timed_until = loop_started + RETURN_THROUGH_SEC
                                timed_next_stage = "Tag0へ中心合わせ中"
                                stage = "Tag6/10を通過中"

                elif stage == "Tag6/10を通過中":
                    if loop_started >= timed_until:
                        stage = timed_next_stage
                    else:
                        auto = timed_command

                elif stage == "Tag0へ中心合わせ中":
                    tag0 = tags.get(TAG_GOAL, camera_hensuu.tag_max_age_sec)
                    if tag0 is None or tag0.distance_m is None:
                        stage = "自動停止: Tag0または距離が読めない"
                    else:
                        auto = face_target_command(
                            tag0,
                            center_tolerance=CENTER_TOLERANCE,
                            rotation_gain=FACE_TAG_ROTATION_GAIN,
                            maximum_speed=FACE_TAG_MAX_SPEED,
                        )
                        if auto == MotionCommand.stop():
                            auto = MotionCommand(
                                forward=max(-AUTO_SPEED, min(AUTO_SPEED, (tag0.distance_m - TAG0_DISTANCE_M) * CENTER_GAIN)),
                            )
                        if abs(tag0.horizontal_error) <= CENTER_TOLERANCE and abs(tag0.distance_m - TAG0_DISTANCE_M) <= DISTANCE_TOLERANCE_M:
                            stage = "GAME1完了"

            # 自動速度へ手動スティックを足す。完全手動なら手動だけになる。
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
