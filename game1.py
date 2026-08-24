"""ROX2026 GAME1。

実行: python3 game1.py
このファイルにGAME1の動きと停止条件を直接書く。
GAME1中はCREATEで開始姿勢にした後、lift/catchを動かさない。
"""

from __future__ import annotations

import time

import camera_hensuu
from rox_mecanum import (
    AprilTagDetector,
    Button,
    ModeController,
    MotionCommand,
    RobotRuntime,
    TagStore,
    add_manual_command,
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

# CREATEで作る開始姿勢。以後GAME1中はlift/catchを動かさない。
START_CATCH_ANGLE = 0.0
START_LIFT_ANGLE = 0.0

# Tagを中心・目標距離へ寄せる速さと許容範囲。
AUTO_SPEED = 0.20
CENTER_GAIN = 0.45
CENTER_TOLERANCE = 0.08
DISTANCE_TOLERANCE_M = 0.08

# Tagごとに止まりたい距離[m]。実測後に変更する。
TAG8_DISTANCE_M = 0.80
TAG12_13_DISTANCE_M = 0.80
TAG6_10_DISTANCE_M = 0.80
TAG0_DISTANCE_M = 0.80

# 時間で行う動き。0秒のままでは安全のため動かない。
TURN_AROUND_SPEED = 0.20
TURN_AROUND_SEC = 0.0
SLIDE_SPEED = 0.20
SLIDE_SEC = 0.0
TUNNEL_SPEED = 0.20
TUNNEL_SEC = 0.0
BOARD_PUSH_SPEED = 0.15
BOARD_PUSH_SEC = 0.0
RETURN_THROUGH_SPEED = 0.20
RETURN_THROUGH_SEC = 0.0

# SIDE Aの横移動方向。SIDE Bでは自動で反対になる。
SIDE_A_SLIDE_SIGN = 1.0


def main() -> None:
    print("GAME1: タッチパッド=手動/自動, CREATE=展開, △=Tag8へ, ○=通過, □=板合わせ, R1=押す, ×=上がった確認, L2=帰還")
    runtime = None
    camera = None

    try:
        runtime = RobotRuntime.open(with_solenoid=False)
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
        side_a = True
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
                runtime.emergency_stop()
                break

            # タッチパッドで完全手動 / 自動を切り替える。
            if mode.update(state):
                runtime.servos.hold_all_current()
                timed_until = 0.0
                stage = "△でTag1/9からTag8へ移動" if deployed else "CREATEで開始姿勢へ展開"
                print(f"モード: {'自動' if mode.auto_enabled else '完全手動'}")

            # SIDE A/Bは開始前だけ選ぶ。横移動方向だけを反転する。
            if stage in {"CREATEで開始姿勢へ展開", "△でTag1/9からTag8へ移動"}:
                if state.was_pressed(Button.DPAD_LEFT):
                    side_a = True
                if state.was_pressed(Button.DPAD_RIGHT):
                    side_a = False

            auto = MotionCommand.stop()

            if camera_error:
                stage = "自動停止: カメラエラー"

            if mode.auto_enabled and not camera_error:
                # CREATE: スタート時のサイズ用の姿勢へ移動する。
                # ここからGAME1終了までlift/catchには命令を出さない。
                if stage == "CREATEで開始姿勢へ展開" and state.was_pressed(Button.CREATE):
                    runtime.servos.catch.write(START_CATCH_ANGLE)
                    runtime.servos.lift.write(START_LIFT_ANGLE)
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
                            timed_next_stage = "Tag8を探して横移動中"
                            stage = "反対向きへ旋回中"
                    else:
                        stage = "自動停止: Tag1/9が見えない"

                # 旋回・通過・押し込み・帰還は、終了時刻まで同じ速度を出す。
                elif stage == "反対向きへ旋回中":
                    if loop_started >= timed_until:
                        stage = timed_next_stage
                    else:
                        auto = timed_command

                # Tag8が見えるまで横へ移動。見つからないまま時間切れなら停止する。
                elif stage == "Tag8を探して横移動中":
                    tag8 = tags.get(TAG_GATE, camera_hensuu.tag_max_age_sec)
                    if tag8 is not None:
                        timed_until = 0.0
                        stage = "Tag8へ中心合わせ中"
                    elif timed_until == 0.0:
                        if SLIDE_SEC <= 0.0:
                            stage = "自動停止: 横移動時間が0秒"
                        else:
                            direction = SIDE_A_SLIDE_SIGN if side_a else -SIDE_A_SLIDE_SIGN
                            timed_command = MotionCommand(strafe=direction * SLIDE_SPEED)
                            timed_until = loop_started + SLIDE_SEC
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
                        auto = MotionCommand(
                            forward=max(-AUTO_SPEED, min(AUTO_SPEED, (tag8.distance_m - TAG8_DISTANCE_M) * CENTER_GAIN)),
                            strafe=tag8.horizontal_error * CENTER_GAIN,
                        )
                        if abs(tag8.horizontal_error) <= CENTER_TOLERANCE and abs(tag8.distance_m - TAG8_DISTANCE_M) <= DISTANCE_TOLERANCE_M:
                            stage = "○でトンネル通過"

                # ○: Tag8での中心合わせ完了後だけ通過する。
                elif stage == "○でトンネル通過" and state.was_pressed(Button.CIRCLE):
                    if TUNNEL_SEC <= 0.0:
                        stage = "自動停止: トンネル通過時間が0秒"
                    else:
                        timed_command = MotionCommand(forward=TUNNEL_SPEED)
                        timed_until = loop_started + TUNNEL_SEC
                        timed_next_stage = "□でTag12/13へ位置合わせ"
                        stage = "トンネル通過中"

                elif stage == "トンネル通過中":
                    if loop_started >= timed_until:
                        stage = timed_next_stage
                    else:
                        auto = timed_command

                # □: Tag12と13の中間へ中心・距離を合わせる。
                elif stage == "□でTag12/13へ位置合わせ" and state.was_pressed(Button.SQUARE):
                    stage = "Tag12/13へ中心合わせ中"

                elif stage == "Tag12/13へ中心合わせ中":
                    tag12 = tags.get(TAG_BOARD_LEFT, camera_hensuu.tag_max_age_sec)
                    tag13 = tags.get(TAG_BOARD_RIGHT, camera_hensuu.tag_max_age_sec)
                    target = midpoint(tag12, tag13) if tag12 and tag13 else None
                    if target is None or target.distance_m is None:
                        stage = "自動停止: Tag12/13または距離が読めない"
                    else:
                        auto = MotionCommand(
                            forward=max(-AUTO_SPEED, min(AUTO_SPEED, (target.distance_m - TAG12_13_DISTANCE_M) * CENTER_GAIN)),
                            strafe=target.horizontal_error * CENTER_GAIN,
                        )
                        if abs(target.horizontal_error) <= CENTER_TOLERANCE and abs(target.distance_m - TAG12_13_DISTANCE_M) <= DISTANCE_TOLERANCE_M:
                            stage = "R1で板を押す"

                # R1: 操縦者が手動修正後、低速で板を押す。
                elif stage == "R1で板を押す" and state.was_pressed(Button.R1):
                    if BOARD_PUSH_SEC <= 0.0:
                        stage = "自動停止: 押し込み時間が0秒"
                    else:
                        timed_command = MotionCommand(forward=BOARD_PUSH_SPEED)
                        timed_until = loop_started + BOARD_PUSH_SEC
                        timed_next_stage = "×で板へ上がったことを確認"
                        stage = "板を押し込み中"

                elif stage == "板を押し込み中":
                    if loop_started >= timed_until:
                        stage = timed_next_stage
                    else:
                        auto = timed_command

                # ×: 板へ上がれたことを操縦者が目視で確定する。
                elif stage == "×で板へ上がったことを確認" and state.was_pressed(Button.CROSS):
                    stage = "L2でTag6/10から帰還"

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
                        auto = MotionCommand(
                            forward=max(-AUTO_SPEED, min(AUTO_SPEED, (target.distance_m - TAG6_10_DISTANCE_M) * CENTER_GAIN)),
                            strafe=target.horizontal_error * CENTER_GAIN,
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
                        auto = MotionCommand(
                            forward=max(-AUTO_SPEED, min(AUTO_SPEED, (tag0.distance_m - TAG0_DISTANCE_M) * CENTER_GAIN)),
                            strafe=tag0.horizontal_error * CENTER_GAIN,
                        )
                        if abs(tag0.horizontal_error) <= CENTER_TOLERANCE and abs(tag0.distance_m - TAG0_DISTANCE_M) <= DISTANCE_TOLERANCE_M:
                            stage = "GAME1完了"

            # 自動速度へ手動スティックを足す。完全手動なら手動だけになる。
            command = add_manual_command(auto, runtime.manual_command(state), mode.auto_enabled)
            runtime.mecanum.drive(command)
            runtime.update_outputs()

            if stage != shown_stage:
                print(f"[{mode.mode.value}] {stage}  side={'A' if side_a else 'B'}")
                shown_stage = stage

            time.sleep(max(0.0, 1.0 / 50.0 - (time.monotonic() - loop_started)))

    except KeyboardInterrupt:
        if runtime is not None:
            runtime.emergency_stop()
    finally:
        if camera is not None:
            camera.close()
        if runtime is not None:
            runtime.close()


if __name__ == "__main__":
    main()
