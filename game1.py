"""ROX2026 GAME1専用プログラム。

実行: ``python3 game1.py``
設定値は game1_hensuu.py だけを変更する。
"""

from __future__ import annotations

import time
from enum import Enum

import camera_hensuu
import game1_hensuu as cfg
from rox_mecanum import (
    AprilTagDetector,
    Button,
    ControlMode,
    ModeController,
    MotionCommand,
    open_camera,
    RobotRuntime,
    TagObservation,
    TagStore,
    TimedMotion,
    add_manual_command,
    midpoint,
)


class Stage(str, Enum):
    WAIT_DEPLOY = "CREATEで開始姿勢へ展開"
    DEPLOYING = "lift/catchを展開中"
    WAIT_START = "△でTag1/9からTag8へ移動"
    FIND_REFERENCE = "Tag1/9を探索中"
    TURN = "反対向きへ旋回中"
    SLIDE = "Tag8を探して横移動中"
    ALIGN_TAG8 = "Tag8へ中心合わせ中"
    WAIT_PASS = "○でトンネル通過"
    PASSING = "トンネル通過中"
    WAIT_BOARD_ALIGN = "□でTag12・13への位置合わせを開始"
    ALIGN_BOARD = "Tag12/13へ中心合わせ中"
    WAIT_PUSH = "R1で板を押す"
    PUSHING = "板を押し込み中"
    WAIT_CONFIRM = "×で板へ上がったことを確認"
    WAIT_RETURN = "L2でTag6/10から帰還"
    ALIGN_RETURN = "Tag6/10の中央へ中心合わせ中"
    RETURN_THROUGH = "Tag6/10を通過中"
    ALIGN_TAG0 = "Tag0へ中心合わせ中"
    DONE = "GAME1完了"
    FAULT = "自動停止"


class Game1Auto:
    """GAME1固有の状態だけを管理する。ハードウェアは持たない。"""

    def __init__(self) -> None:
        self.stage = Stage.WAIT_DEPLOY
        self.side_a = True
        self.motion: TimedMotion | None = None
        self.error = ""
        self.deployed = False

    def reset_after_mode_change(self) -> None:
        """自動の途中再開を禁止する。展開済みなら最初の走行承認へ戻す。"""
        self.motion = None
        self.stage = Stage.WAIT_START if self.deployed else Stage.WAIT_DEPLOY

    def start_timed(self, command: MotionCommand, duration: float, next_stage: Stage) -> bool:
        if duration <= 0.0:
            self.stage = Stage.FAULT
            self.error = "時間式移動の秒数が0です。game1_hensuu.pyを設定してください"
            return False
        self.motion = TimedMotion(command, duration)
        self.motion.start()
        self._next_stage = next_stage
        return True

    def timed_command(self) -> MotionCommand:
        if self.motion is None:
            return MotionCommand.stop()
        command = self.motion.active_command()
        if self.motion.finished():
            self.motion = None
            self.stage = self._next_stage
            return MotionCommand.stop()
        return command

    def align(self, observation: TagObservation | None, target_distance: float) -> MotionCommand:
        """Tagを画面中心・設定距離へ合わせる。距離未校正なら前進しない。"""
        if observation is None:
            return MotionCommand.stop()
        strafe = observation.horizontal_error * cfg.center_gain
        forward = 0.0
        if observation.distance_m is not None:
            forward = max(-cfg.auto_speed, min(cfg.auto_speed, (observation.distance_m - target_distance) * cfg.center_gain))
        return MotionCommand(forward=forward, strafe=strafe)

    def aligned(self, observation: TagObservation | None, target_distance: float) -> bool:
        return bool(
            observation
            and observation.distance_m is not None
            and abs(observation.horizontal_error) <= cfg.center_tolerance
            and abs(observation.distance_m - target_distance) <= cfg.distance_tolerance_m
        )


def _read_tags(camera: object, detector: AprilTagDetector, store: TagStore) -> str | None:
    try:
        store.update(detector.detect(camera.read()))
        return None
    except Exception as error:  # 実機での抜け・再接続を安全側へ扱う。
        return str(error)


def main() -> None:
    print("GAME1: タッチパッド=手動/自動, OPTIONS=非常停止")
    runtime = None
    camera = None
    try:
        runtime = RobotRuntime.open(with_solenoid=False)
        camera = open_camera(
            backend=camera_hensuu.camera_backend, device=camera_hensuu.camera_device,
            pipe_id=camera_hensuu.mipi_pipe_id, host_index=camera_hensuu.mipi_host_index, fps=camera_hensuu.mipi_fps,
            width=camera_hensuu.mipi_width, height=camera_hensuu.mipi_height,
            fisheye_calibration_file=camera_hensuu.fisheye_calibration_file if camera_hensuu.fisheye_enabled else None,
            fisheye_balance=camera_hensuu.fisheye_balance,
        )
        detector = AprilTagDetector(camera_hensuu.apriltag_size_m, camera_hensuu.camera_focal_length_px)
        tags = TagStore()
        mode = ModeController()
        game = Game1Auto()
        previous_stage = None

        while True:
            started = time.monotonic()
            state = runtime.controller.read()
            camera_error = _read_tags(camera, detector, tags)
            if state.button(Button.OPTIONS):
                print("OPTIONS: 非常停止")
                break
            if mode.update(state):
                # 自動のサーボ目標も中断し、現在位置だけをPID保持する。
                runtime.servos.hold_all_current()
                game.reset_after_mode_change()
                print(f"モード: {'自動' if mode.auto_enabled else '完全手動'}")

            # SIDE選択は停止中だけ。A/Bで横移動を左右反転する。
            if game.stage in {Stage.WAIT_DEPLOY, Stage.WAIT_START}:
                if state.was_pressed(Button.DPAD_LEFT):
                    game.side_a = True
                if state.was_pressed(Button.DPAD_RIGHT):
                    game.side_a = False

            auto = MotionCommand.stop()
            if mode.auto_enabled:
                if game.stage is Stage.WAIT_DEPLOY and state.was_pressed(Button.CREATE):
                    runtime.servos.catch.write(cfg.game1_catch_start_angle)
                    runtime.servos.lift.write(cfg.game1_lift_start_angle)
                    game.stage = Stage.DEPLOYING
                elif game.stage is Stage.DEPLOYING:
                    if runtime.servos.catch.is_at_target() and runtime.servos.lift.is_at_target():
                        runtime.servos.hold_all_current()
                        game.deployed = True
                        game.stage = Stage.WAIT_START
                elif game.stage is Stage.WAIT_START and state.was_pressed(Button.TRIANGLE):
                    game.stage = Stage.FIND_REFERENCE
                elif game.stage is Stage.FIND_REFERENCE:
                    reference = tags.get(cfg.tag_start_primary, camera_hensuu.tag_max_age_sec) or tags.get(cfg.tag_start_fallback, camera_hensuu.tag_max_age_sec)
                    if reference is not None:
                        if game.start_timed(MotionCommand(rotate=cfg.turn_around_speed), cfg.turn_around_sec, Stage.SLIDE):
                            game.stage = Stage.TURN
                    else:
                        game.error = "Tag1/9が見えません。手動で見える位置へ移動してください"
                elif game.stage is Stage.TURN:
                    auto = game.timed_command()
                elif game.stage is Stage.SLIDE:
                    tag8 = tags.get(cfg.tag_gate, camera_hensuu.tag_max_age_sec)
                    if tag8 is not None:
                        game.motion = None
                        game.stage = Stage.ALIGN_TAG8
                    elif game.motion is None:
                        sign = cfg.side_a_slide_sign if game.side_a else -cfg.side_a_slide_sign
                        game.start_timed(MotionCommand(strafe=sign * cfg.slide_speed), cfg.slide_sec, Stage.FAULT)
                    else:
                        auto = game.timed_command()
                        if game.stage is Stage.FAULT:
                            game.error = "Tag8を見つけられません。距離またはカメラ向きを確認してください"
                elif game.stage is Stage.ALIGN_TAG8:
                    tag8 = tags.get(cfg.tag_gate, camera_hensuu.tag_max_age_sec)
                    auto = game.align(tag8, cfg.tag8_distance_m)
                    if game.aligned(tag8, cfg.tag8_distance_m):
                        game.stage = Stage.WAIT_PASS
                elif game.stage is Stage.WAIT_PASS and state.was_pressed(Button.CIRCLE):
                    if game.start_timed(MotionCommand(forward=cfg.tunnel_speed), cfg.tunnel_sec, Stage.WAIT_BOARD_ALIGN):
                        game.stage = Stage.PASSING
                elif game.stage is Stage.PASSING:
                    auto = game.timed_command()
                elif game.stage is Stage.WAIT_BOARD_ALIGN and state.was_pressed(Button.SQUARE):
                    game.stage = Stage.ALIGN_BOARD
                elif game.stage is Stage.ALIGN_BOARD:
                    first = tags.get(cfg.tag_board_left, camera_hensuu.tag_max_age_sec)
                    second = tags.get(cfg.tag_board_right, camera_hensuu.tag_max_age_sec)
                    target = midpoint(first, second) if first and second else None
                    auto = game.align(target, cfg.tag12_13_distance_m)
                    if game.aligned(target, cfg.tag12_13_distance_m):
                        game.stage = Stage.WAIT_PUSH
                elif game.stage is Stage.WAIT_PUSH and state.was_pressed(Button.R1):
                    if game.start_timed(MotionCommand(forward=cfg.board_push_speed), cfg.board_push_sec, Stage.WAIT_CONFIRM):
                        game.stage = Stage.PUSHING
                elif game.stage is Stage.PUSHING:
                    auto = game.timed_command()
                elif game.stage is Stage.WAIT_CONFIRM and state.was_pressed(Button.CROSS):
                    game.stage = Stage.WAIT_RETURN
                elif game.stage is Stage.WAIT_RETURN and state.was_pressed(Button.L2):
                    game.stage = Stage.ALIGN_RETURN
                elif game.stage is Stage.ALIGN_RETURN:
                    first = tags.get(cfg.tag_return_left, camera_hensuu.tag_max_age_sec)
                    second = tags.get(cfg.tag_return_right, camera_hensuu.tag_max_age_sec)
                    target = midpoint(first, second) if first and second else None
                    auto = game.align(target, cfg.tag6_10_distance_m)
                    if game.aligned(target, cfg.tag6_10_distance_m):
                        if game.start_timed(MotionCommand(forward=cfg.return_through_speed), cfg.return_through_sec, Stage.ALIGN_TAG0):
                            game.stage = Stage.RETURN_THROUGH
                elif game.stage is Stage.RETURN_THROUGH:
                    auto = game.timed_command()
                elif game.stage is Stage.ALIGN_TAG0:
                    tag0 = tags.get(cfg.tag_goal, camera_hensuu.tag_max_age_sec)
                    auto = game.align(tag0, cfg.tag0_distance_m)
                    if game.aligned(tag0, cfg.tag0_distance_m):
                        game.stage = Stage.DONE

            command = add_manual_command(auto, runtime.manual_command(state), mode.auto_enabled)
            runtime.mecanum.drive(command)
            runtime.update_outputs()
            if game.stage is not previous_stage:
                print(f"[{mode.mode.value}] {game.stage.value}  side={'A' if game.side_a else 'B'}")
                previous_stage = game.stage
            if camera_error:
                game.error = camera_error
            if game.error and game.stage is Stage.FAULT:
                print(f"停止理由: {game.error}")
            time.sleep(max(0.0, 1.0 / 50.0 - (time.monotonic() - started)))
    except KeyboardInterrupt:
        pass
    finally:
        if camera is not None:
            camera.close()
        if runtime is not None:
            runtime.close()


if __name__ == "__main__":
    main()
