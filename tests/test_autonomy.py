from __future__ import annotations

from rox_mecanum import Button, ControlMode, ControllerState, ModeController, MotionCommand, TagObservation, TagStore, add_manual_command, face_target_command, midpoint, robot_center_horizontal_error


def test_touchpad_switches_mode_only_on_press() -> None:
    mode = ModeController()
    assert mode.mode is ControlMode.MANUAL
    assert mode.update(ControllerState(pressed=frozenset({Button.TOUCHPAD})))
    assert mode.mode is ControlMode.AUTO
    assert not mode.update(ControllerState())


def test_manual_mode_ignores_auto_but_auto_mode_adds_it() -> None:
    auto = MotionCommand(forward=0.4, strafe=0.2)
    manual = MotionCommand(forward=-0.1, rotate=0.3)
    assert add_manual_command(auto, manual, False) == manual
    combined = add_manual_command(auto, manual, True)
    assert round(combined.forward, 6) == 0.3
    assert combined.strafe == 0.2
    assert combined.rotate == 0.3


def test_tag_store_never_returns_old_observation() -> None:
    store = TagStore()
    tag = TagObservation(8, 500, 200, 1000, 1.0, timestamp=10.0)
    store.update([tag])
    assert store.get(8, max_age_sec=0.5, now=10.2) == tag
    assert store.get(8, max_age_sec=0.5, now=10.6) is None


def test_pair_midpoint_is_centered() -> None:
    first = TagObservation(12, 400, 200, 1000, 1.0, 1.0)
    second = TagObservation(13, 600, 200, 1000, 1.2, 1.0)
    target = midpoint(first, second)
    assert target.horizontal_error == 0.0
    assert target.distance_m == 1.1


def test_face_target_rotates_only_when_tag_is_off_center() -> None:
    centered = TagObservation(8, 500, 200, 1000, 1.0, 1.0)
    right_edge = TagObservation(8, 900, 200, 1000, 1.0, 1.0)
    assert face_target_command(centered) == MotionCommand.stop()
    command = face_target_command(right_edge, maximum_speed=0.2)
    assert command.forward == 0.0
    assert command.strafe == 0.0
    assert command.rotate > 0.0


def test_camera_lateral_offset_corrects_for_robot_center() -> None:
    # カメラが右へ20cm、Tagまで2m、焦点距離500pxの場合、ロボット正面のTagは
    # 画像中心より50px（正規化で-0.1）左に見える。補正後は0になる。
    target = TagObservation(8, 450, 200, 1000, 2.0, 1.0)
    assert robot_center_horizontal_error(target, camera_lateral_offset_m=0.2, focal_length_px=500.0) == 0.0
