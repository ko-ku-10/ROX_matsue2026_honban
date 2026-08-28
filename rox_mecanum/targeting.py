"""GAME2のパネル選択に使う、カメラ非依存の判定部品。"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

from .vision import TagStore


@dataclass(frozen=True)
class PanelTarget:
    """狙う行と、その行で狙いの基準にするTag。"""

    row: str
    tag_ids: tuple[int, ...]

    @property
    def label(self) -> str:
        ids = ", ".join(str(tag_id) for tag_id in self.tag_ids)
        return f"{self.row}段 / Tag {ids}"


def choose_panel_target(
    tags: TagStore,
    rows: Mapping[str, tuple[int, ...]],
    max_age_sec: float,
    priority: tuple[str, ...] = ("middle", "top", "bottom"),
) -> PanelTarget | None:
    """指定した優先順で、見えているパネルから発射候補を返す。

    同じ行で2枚以上見える時は、左右端2枚を返す。ゲーム側はその中間を
    照準位置として使えるため、2枚同時に倒す可能性を残せる。
    """

    for row in priority:
        if row not in rows:
            continue
        visible = [tag_id for tag_id in rows[row] if tags.get(tag_id, max_age_sec)]
        if visible:
            ids = (visible[0], visible[-1]) if len(visible) >= 2 else (visible[0],)
            return PanelTarget(row, ids)
    return None
