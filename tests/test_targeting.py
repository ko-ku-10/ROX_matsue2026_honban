from __future__ import annotations

from time import monotonic

from rox_mecanum import TagObservation, TagStore, choose_panel_target


ROWS = {"top": (14, 15, 16), "middle": (17, 18, 19), "bottom": (20, 21, 22)}


def _store(*ids: int) -> TagStore:
    store = TagStore()
    store.update(TagObservation(tag_id, 500, 200, 1000, 1.0, monotonic()) for tag_id in ids)
    return store


def test_middle_has_highest_priority() -> None:
    target = choose_panel_target(_store(14, 18, 22), ROWS, 1.0)
    assert target is not None
    assert target.row == "middle"
    assert target.tag_ids == (18,)


def test_two_panels_selects_between_them() -> None:
    target = choose_panel_target(_store(14, 16), ROWS, 1.0)
    assert target is not None
    assert target.tag_ids == (14, 16)
