from __future__ import annotations

from struct import pack, unpack

import robot_actions


def _mechpos(raw_position: int) -> float:
    return unpack("<f", pack("<I", raw_position))[0]


def test_catch_middle_position_is_mechpos_midpoint() -> None:
    expected = (
        _mechpos(robot_actions.CATCH_DRIBBLE_POSITION)
        + _mechpos(robot_actions.CATCH_OPEN_POSITION)
    ) / 2.0
    # float32へ戻す時の最小丸め誤差だけを許容する。
    assert abs(_mechpos(robot_actions.CATCH_MIDDLE_POSITION) - expected) < 1e-6
