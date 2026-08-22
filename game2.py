"""ROX2026 GAME2専用プログラム。実行: ``python3 game2.py``"""

from __future__ import annotations

import time
from enum import Enum

import camera_hensuu
import game2_hensuu as cfg
from rox_mecanum import (
    AprilTagDetector,
    Button,
    ModeController,
    MotionCommand,
    open_stereo_camera,
    RobotRuntime,
    TagObservation,
    TagStore,
    TimedMotion,
    add_manual_command,
    choose_panel_target,
    midpoint,
)


class Stage(str, Enum):
    WAIT_BALL = "補給待ち: CREATEで照準開始"
    APPROACH = "標的へ前進中"
    AIM_STRAFE = "横スライド照準中"
    AIM_READY = "照準完了: △で持上げ"
    LIFTING = "発射高さへ持上げ中"
    READY_TO_FIRE = "L2で発射"
    FIRED = "発射済み: ×で後退"
    LOWERING = "liftを下げ中"
    RETREAT = "補給地点へ後退中"
    FAULT = "自動停止"


class Game2Auto:
    def __init__(self) -> None:
        self.stage = Stage.WAIT_BALL
        self.target_ids: tuple[int, ...] | None = None
        self.motion: TimedMotion | None = None
        self.error = ""
        self.stage_started = time.monotonic()

    def enter(self, stage: Stage) -> None:
        self.stage = stage
        self.stage_started = time.monotonic()

    def reset_after_mode_change(self) -> None:
        self.motion = None
        self.target_ids = None
        self.enter(Stage.WAIT_BALL)

    def target(self, tags: TagStore) -> TagObservation | None:
        if not self.target_ids:
            return None
        observations = [tags.get(tag_id, camera_hensuu.tag_max_age_sec) for tag_id in self.target_ids]
        if any(item is None for item in observations):
            return None
        first = observations[0]
        return first if len(observations) == 1 else midpoint(first, observations[-1])

    def align_command(self, tag: TagObservation | None, *, forward_allowed: bool) -> MotionCommand:
        if tag is None:
            return MotionCommand.stop()
        strafe = tag.horizontal_error * cfg.center_gain
        forward = 0.0
        if forward_allowed:
            if tag.distance_m is None:
                return MotionCommand(strafe=strafe)
            forward = max(-cfg.auto_speed, min(cfg.auto_speed, (tag.distance_m - cfg.approach_distance_m) * cfg.center_gain))
        return MotionCommand(forward=forward, strafe=strafe)

    def distance_ready(self, tag: TagObservation | None) -> bool:
        return bool(tag and tag.distance_m is not None and abs(tag.distance_m - cfg.approach_distance_m) <= cfg.distance_tolerance_m)

    def center_ready(self, tag: TagObservation | None) -> bool:
        return bool(tag and abs(tag.horizontal_error) <= cfg.center_tolerance)


def _read_tags(camera: object, detector: AprilTagDetector, store: TagStore) -> str | None:
    try:
        left, _right = camera.read()
        store.update(detector.detect(left))
        return None
    except Exception as error:
        return str(error)


def main() -> None:
    print("GAME2: タッチパッド=手動/自動, CREATE=照準, △=持上げ, L2=発射, ×=後退")
    runtime = None
    camera = None
    try:
        runtime = RobotRuntime.open(with_solenoid=True)
        camera = open_stereo_camera(
            backend=camera_hensuu.camera_backend, left_device=camera_hensuu.left_camera_device,
            right_device=camera_hensuu.right_camera_device, left_index=camera_hensuu.left_mipi_camera_index,
            right_index=camera_hensuu.right_mipi_camera_index, fps=camera_hensuu.mipi_fps,
            width=camera_hensuu.mipi_width, height=camera_hensuu.mipi_height,
        )
        detector = AprilTagDetector(camera_hensuu.apriltag_size_m, camera_hensuu.camera_focal_length_px)
        tags = TagStore()
        mode = ModeController()
        game = Game2Auto()
        previous_stage = None

        while True:
            started = time.monotonic()
            state = runtime.controller.read()
            camera_error = _read_tags(camera, detector, tags)
            if state.button(Button.OPTIONS):
                print("OPTIONS: 非常停止")
                break
            if mode.update(state):
                # 持上げ途中などの自動サーボ目標は継続させず、今の位置を保持する。
                runtime.servos.hold_all_current()
                game.reset_after_mode_change()
                print(f"モード: {'自動' if mode.auto_enabled else '完全手動'}")

            auto = MotionCommand.stop()
            if mode.auto_enabled:
                if game.stage is Stage.WAIT_BALL and state.was_pressed(Button.CREATE):
                    choice = choose_panel_target(tags, cfg.panel_rows, camera_hensuu.tag_max_age_sec)
                    game.target_ids = choice.tag_ids if choice else None
                    if choice is None:
                        game.enter(Stage.FAULT)
                        game.error = "Tag14〜22が見えません。パネルへ向けてください"
                    else:
                        runtime.servos.lift.write(cfg.lift_ground_angle)
                        game.enter(Stage.APPROACH)
                elif game.stage is Stage.APPROACH:
                    tag = game.target(tags)
                    auto = game.align_command(tag, forward_allowed=True)
                    if game.distance_ready(tag):
                        game.enter(Stage.AIM_STRAFE)
                elif game.stage is Stage.AIM_STRAFE:
                    tag = game.target(tags)
                    auto = game.align_command(tag, forward_allowed=False)
                    if game.center_ready(tag):
                        game.enter(Stage.AIM_READY)
                elif game.stage is Stage.AIM_READY and state.was_pressed(Button.TRIANGLE):
                    runtime.servos.lift.write(cfg.lift_fire_angle)
                    game.enter(Stage.LIFTING)
                elif game.stage is Stage.LIFTING:
                    if runtime.servos.lift.is_at_target():
                        game.enter(Stage.READY_TO_FIRE)
                    elif started - game.stage_started > cfg.lift_target_timeout_sec:
                        game.enter(Stage.FAULT)
                        game.error = "liftが発射高さに到達しません"
                elif game.stage is Stage.READY_TO_FIRE and state.was_pressed(Button.L2):
                    runtime.fire()
                    game.enter(Stage.FIRED)
                elif game.stage is Stage.FIRED and state.was_pressed(Button.CROSS):
                    runtime.servos.lift.write(cfg.lift_ground_angle)
                    game.enter(Stage.LOWERING)
                elif game.stage is Stage.LOWERING:
                    if runtime.servos.lift.is_at_target():
                        if cfg.retreat_sec <= 0.0:
                            game.enter(Stage.FAULT)
                            game.error = "retreat_secが0です。game2_hensuu.pyを設定してください"
                        else:
                            game.motion = TimedMotion(MotionCommand.backward(cfg.retreat_speed), cfg.retreat_sec)
                            game.motion.start()
                            game.enter(Stage.RETREAT)
                elif game.stage is Stage.RETREAT and game.motion is not None:
                    auto = game.motion.active_command()
                    if game.motion.finished():
                        game.motion = None
                        game.target_ids = None
                        game.enter(Stage.WAIT_BALL)

            command = add_manual_command(auto, runtime.manual_command(state), mode.auto_enabled)
            runtime.mecanum.drive(command)
            runtime.update_outputs()
            if game.stage is not previous_stage:
                print(f"[{mode.mode.value}] {game.stage.value} target={game.target_ids}")
                previous_stage = game.stage
            if camera_error and game.stage is not Stage.FAULT:
                game.error = camera_error
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
