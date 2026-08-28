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

    同じ行で2枚見える時は、その2枚の間を返す。3枚見える時は、現在の
    画面中央に近い「隣り合う2枚」を返す。これにより中央の1枚を狙うのではなく、
    2枚の間を狙って同時に倒す可能性を残せる。
    """

    for row in priority:
        if row not in rows:
            continue
        visible = [tag_id for tag_id in rows[row] if tags.get(tag_id, max_age_sec)]
        if visible:
            if len(visible) == 1:
                ids = (visible[0],)
            elif len(visible) == 2:
                ids = (visible[0], visible[1])
            else:
                # (左・中央) と (中央・右) のうち、ロボット正面に近い方を使う。
                # 同じなら左・中央を選ぶため、選択は毎回一定になる。
                pairs = list(zip(visible, visible[1:]))

                def center_error(pair: tuple[int, int]) -> float:
                    first = tags.get(pair[0], max_age_sec)
                    second = tags.get(pair[1], max_age_sec)
                    if first is None or second is None:
                        return float("inf")
                    return abs((first.horizontal_error + second.horizontal_error) / 2.0)

                ids = min(pairs, key=center_error)
            return PanelTarget(row, ids)
    return None
