"""ボール機構の共通姿勢。

GAME3と実験プログラムが同じ関数を使うため、角度を一度調整すれば両方へ反映される。
"""

from __future__ import annotations

import hensuu


def set_transport_pose(servos: object) -> None:
    """ボールを地面に付けたまま保持して走行する姿勢にする。"""
    servos.catch.write(hensuu.catch_ball_hold_angle)
    servos.lift.write(hensuu.lift_ball_ground_angle)


def transport_pose_ready(servos: object) -> bool:
    """地面保持姿勢にcatch/liftが両方到達した時だけTrue。"""
    return servos.catch.is_at_target() and servos.lift.is_at_target()


def set_grab_pose(servos: object) -> None:
    """catchだけをボールを掴む角度へ動かす。"""
    servos.catch.write(hensuu.catch_ball_grab_angle)


def set_release_pose(servos: object) -> None:
    """catchだけをボールをRobot外へ出す角度へ動かす。"""
    servos.catch.write(hensuu.catch_ball_release_angle)


def set_fire_pose(servos: object, lift_angle: float) -> None:
    """catchを排出角度、liftを指定した発射高さへ同時に動かす。"""
    set_release_pose(servos)
    servos.lift.write(float(lift_angle))


def fire_pose_ready(servos: object) -> bool:
    """発射姿勢にcatch/liftが両方到達した時だけTrue。"""
    return servos.catch.is_at_target() and servos.lift.is_at_target()
