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
    open_camera,
)


# ==================================================
# ここはGAME2の実際の動き。必要なら直接書き換える。
# ==================================================

# Tag位置（カメラ座標の右x・前z）を使う照準の速さ・許容範囲。
# まず画像中心へ旋回してから位置を使うため、魚眼の端の値で横移動しない。
AUTO_STRAFE_MAX_SPEED = 0.20
# P制御だけだと中心付近で1〜2%になり、実機では静止摩擦に負けて動かない。
# 動き出せる最低速度。横移動が強すぎる時は少し下げる。
AUTO_STRAFE_MIN_SPEED = 0.08
AUTO_FORWARD_MAX_SPEED = 0.20
CENTER_GAIN = 0.45
CENTER_TOLERANCE = 0.08
AUTO_LATERAL_TOLERANCE_M = 0.06
# zidou/mecanum.py と同じ、Tag位置 x からの旋回制御。
AUTO_POSITION_ROTATE_GAIN = 0.60
AUTO_POSITION_ROTATE_MAX_SPEED = 0.20
# 設定距離へ近づく時の許容誤差[m]。
DISTANCE_TOLERANCE_M = 0.08
# 自動で前後へ動く向き。前後が逆なら -1.0 に変える。
AUTO_FORWARD_DIRECTION = 1.0
# 前後移動中に距離誤差が改善しているか確認する間隔と最小改善量。
# 向き・距離計算が逆でも、走り続けないための安全停止である。
AUTO_FORWARD_PROGRESS_SEC = 0.70
AUTO_FORWARD_MIN_PROGRESS_M = 0.03

# Tagの中央・角度合わせはGAME1と同じ共通設定を使う。
TAG_CENTER_TOLERANCE = camera_hensuu.tag_center_tolerance
TAG_CENTER_STABLE_SEC = camera_hensuu.tag_center_stable_sec
TAG_ROTATE_MAX_SPEED = camera_hensuu.tag_rotate_max_speed
TAG_YAW_TOLERANCE_DEG = camera_hensuu.tag_yaw_tolerance_deg
TAG_YAW_GAIN = camera_hensuu.tag_yaw_gain
TAG_YAW_DIRECTION = camera_hensuu.tag_yaw_direction

# ↑を押してからTag14〜22を見つけるまでの待機時間。
TAG_SEARCH_TIMEOUT_SEC = 5.0
# 複数Tagの片方が一瞬見えなくなった時、停止せずその場で再読取りを待つ時間。
TAG_REACQUIRE_TIMEOUT_SEC = 1.0
# RDKのTag検出が一時的に遅い時でも、直前の実測を短時間だけ使う。
# 低速自動走行専用。これ以上古い値では動かない。
AUTO_TAG_MAX_AGE_SEC = 0.75

LIFT_TIMEOUT_SEC = 8.0

# パネルのTag番号。発射する段の優先順は上 → 中央 → 下。
PANEL_ROWS = {
    "middle": (17, 18, 19),
    "top": (14, 15, 16),
    "bottom": (20, 21, 22),
}
PANEL_PRIORITY = ("top", "middle", "bottom")

# 照準の実機テスト中は、狙うTagを1枚だけに固定する。
# 18番の確認が終わったら ``None`` に戻すと、上→中→下の自動選択へ戻る。
TEST_FIXED_TAG_ID = 18

# 段ごとの発射距離[m]。中段・上段・下段で当たりやすい距離を、
# それぞれ実射して入力する。距離はカメラレンズからTag面まで。
AIM_DISTANCE_M = {
    "middle": 1.20,
    "top": 1.20,
    "bottom": 1.20,
}

def main() -> None:
    print("GAME2: タッチパッド=手動/自動")
    print("  手動: CREATE/×=地面姿勢, ○=掴む, □=排出, △=持上げ, R1=発射")
    print("  自動: ↑=Tag18の位置(x/z)へ照準, △=持上げ, L1=ソレノイド発射")
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
        distance_error_at_check = None
        distance_checked_at = None
        last_position_report_at = 0.0
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
                distance_error_at_check = None
                distance_checked_at = None
                print(f"モード: {'自動' if mode.auto_enabled else '完全手動'}")

            # ボールは手動で装填済み。↑を押すとTag14〜22を探して、
            # 画面中心への旋回 → 垂直旋回 → 正面位置 → 設定距離の順に自動照準する。
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
                distance_error_at_check = None
                distance_checked_at = None
                tag_search_started_at = time.monotonic()
                stage = "Tag14〜22を探して照準開始待ち"

            auto = MotionCommand.stop()
            # サイトに出す、GAME2の判断根拠。ここは駆動には使わない表示専用の値。
            auto_debug: dict[str, object] = {
                "自動モード": mode.auto_enabled,
                "現在の段階": stage,
                "判断": "完全手動中" if not mode.auto_enabled else "開始ボタンを待機中",
                "目標Tag": "なし" if target_ids is None else ", ".join(str(item) for item in target_ids),
                "目標段": target_row or "なし",
            }

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
                # 照準テスト中はTag 18だけを使う。Noneなら上→中央→下で選ぶ。
                if stage == "Tag14〜22を探して照準開始待ち":
                    auto_debug["判断"] = "Tag 18を探索中"
                    if TEST_FIXED_TAG_ID is not None:
                        fixed_target = tags.get(TEST_FIXED_TAG_ID, AUTO_TAG_MAX_AGE_SEC)
                        if fixed_target is not None:
                            target_ids = (TEST_FIXED_TAG_ID,)
                            target_row = "middle"
                        else:
                            target_ids = None
                            target_row = None
                    else:
                        choice = choose_panel_target(
                            tags,
                            PANEL_ROWS,
                            AUTO_TAG_MAX_AGE_SEC,
                            priority=PANEL_PRIORITY,
                        )
                        target_ids = choice.tag_ids if choice is not None else None
                        target_row = choice.row if choice is not None else None

                    if target_ids is not None and target_row in AIM_DISTANCE_M:
                        target_missing_since = None
                        distance_error_at_check = None
                        distance_checked_at = None
                        print(
                            f"Tag {target_ids[0]}を選択: 位置x=0m・"
                            f"{AIM_DISTANCE_M[target_row]:.2f}mまで近づきます"
                        )
                        stage = f"{target_row}段: Tag位置(x/z)へ照準中"
                        auto_debug["判断"] = "Tagを発見。x/z位置照準を開始"
                    elif tag_search_started_at is None or time.monotonic() - tag_search_started_at >= TAG_SEARCH_TIMEOUT_SEC:
                        stage = "自動停止: Tag14〜22を見つけられない"
                        auto_debug["判断"] = "探索時間切れ。停止"

                # zidou/mecanum.py と同じ位置制御。
                # x はTagのカメラ座標の左右[m]、z は正面距離[m]。
                # xが0へ近づくよう旋回し、zが設定距離へ近づくよう前後へ動く。
                elif target_row is not None and stage == f"{target_row}段: Tag位置(x/z)へ照準中":
                    target = tags.get(target_ids[0], AUTO_TAG_MAX_AGE_SEC) if target_ids else None
                    if target is None:
                        auto_debug["判断"] = "Tag 18が未受信。再取得を待機"
                        if target_missing_since is None:
                            target_missing_since = loop_started
                        vision_worker.request_tag_read()
                        if loop_started - target_missing_since >= TAG_REACQUIRE_TIMEOUT_SEC:
                            stage = "自動停止: Tag 18を1秒間再取得できない"
                            auto_debug["判断"] = "Tag未受信が1秒継続。停止"
                    elif target.lateral_m is None or target.forward_m is None or target.forward_m <= 0.0:
                        target_missing_since = None
                        stage = "自動停止: Tag 18の位置(x/z)を読めない"
                        auto_debug["判断"] = "xまたはzが無効。停止"
                    else:
                        target_missing_since = None
                        position_x = target.lateral_m + camera_hensuu.camera_lateral_offset_m
                        distance_error = target.forward_m - AIM_DISTANCE_M[target_row]
                        auto_debug.update({
                            "Tag 18 x[m]": round(position_x, 3),
                            "Tag 18 z[m]": round(target.forward_m, 3),
                            "目標 x[m]": 0.0,
                            "目標 z[m]": round(AIM_DISTANCE_M[target_row], 3),
                            "x誤差[m]": round(position_x, 3),
                            "z誤差[m]": round(distance_error, 3),
                            "x到達": abs(position_x) <= AUTO_LATERAL_TOLERANCE_M,
                            "z到達": abs(distance_error) <= DISTANCE_TOLERANCE_M,
                        })
                        if loop_started - last_position_report_at >= 0.5:
                            print(
                                f"Tag {target.tag_id} 位置: x={position_x:+.3f}m "
                                f"z={target.forward_m:.3f}m "
                                f"目標z={AIM_DISTANCE_M[target_row]:.3f}m"
                            )
                            last_position_report_at = loop_started
                        if (
                            abs(position_x) <= AUTO_LATERAL_TOLERANCE_M
                            and abs(distance_error) <= DISTANCE_TOLERANCE_M
                        ):
                            stage = "照準完了: △で持上げ"
                            auto_debug["判断"] = "x/zとも許容範囲内。照準完了"
                        else:
                            # x/zのどちらも改善しない場合は、位置計算またはモーター向きが
                            # 合っていないため停止する。無限に走り続けることはない。
                            position_error_size = max(abs(position_x), abs(distance_error))
                            if distance_checked_at is None:
                                distance_checked_at = loop_started
                                distance_error_at_check = position_error_size
                            elif loop_started - distance_checked_at >= AUTO_FORWARD_PROGRESS_SEC:
                                progress = distance_error_at_check - position_error_size
                                if progress < AUTO_FORWARD_MIN_PROGRESS_M:
                                    stage = "自動停止: Tag 18との位置(x/z)が改善しない"
                                    auto_debug["判断"] = "0.7秒間、x/z誤差が改善しないため停止"
                                else:
                                    distance_checked_at = loop_started
                                    distance_error_at_check = position_error_size

                            # xは右が正。Tagが右なら右旋回する。
                            if not stage.startswith("自動停止:"):
                                rotate = max(
                                    -AUTO_POSITION_ROTATE_MAX_SPEED,
                                    min(
                                        AUTO_POSITION_ROTATE_MAX_SPEED,
                                        position_x * AUTO_POSITION_ROTATE_GAIN,
                                    ),
                                )
                                forward = max(
                                    -AUTO_FORWARD_MAX_SPEED,
                                    min(
                                        AUTO_FORWARD_MAX_SPEED,
                                        distance_error * CENTER_GAIN * AUTO_FORWARD_DIRECTION,
                                    ),
                                )
                                auto = MotionCommand(forward=forward, rotate=rotate)
                                auto_debug.update({
                                    "判断": "x誤差で旋回、z誤差で前後移動",
                                    "自動 前後": round(forward, 3),
                                    "自動 横": 0.0,
                                    "自動 旋回": round(rotate, 3),
                                })

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
            manual = runtime.manual_command(state)
            command = add_manual_command(auto, manual, mode.auto_enabled)
            auto_debug.update({
                "手動 前後": round(manual.forward, 3),
                "手動 横": round(manual.strafe, 3),
                "手動 旋回": round(manual.rotate, 3),
                "最終 前後": round(command.forward, 3),
                "最終 横": round(command.strafe, 3),
                "最終 旋回": round(command.rotate, 3),
                "現在の段階": stage,
            })
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
                auto_debug=auto_debug,
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
