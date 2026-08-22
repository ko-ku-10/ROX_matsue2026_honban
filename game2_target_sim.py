"""GAME2のパネル選択だけを試す、モーターを動かさないカメラ診断。

実行:
  python3 game2_target_sim.py

左右カメラが開けたこと、Tag 14〜22の検出、中央→上→下の優先順位を
ターミナルへ表示する。Ctrl+Cで終了する。
"""

from __future__ import annotations

import time

import camera_hensuu
import game2_hensuu as cfg
from rox_mecanum import AprilTagDetector, TagStore, choose_panel_target, open_stereo_camera


PANEL_IDS = tuple(range(14, 23))


def _ids_in_frame(detector: AprilTagDetector, image: object) -> tuple[int, ...]:
    return tuple(sorted(item.tag_id for item in detector.detect(image) if item.tag_id in PANEL_IDS))


def main() -> None:
    print("GAME2 パネル選択シミュレーション（モーターは動きません）")
    print("左右カメラを開いています。Ctrl+Cで終了します。")
    camera = None
    try:
        camera = open_stereo_camera(
            backend=camera_hensuu.camera_backend, left_device=camera_hensuu.left_camera_device,
            right_device=camera_hensuu.right_camera_device, left_index=camera_hensuu.left_mipi_camera_index,
            right_index=camera_hensuu.right_mipi_camera_index, fps=camera_hensuu.mipi_fps,
            width=camera_hensuu.mipi_width, height=camera_hensuu.mipi_height,
        )
        detector = AprilTagDetector(camera_hensuu.apriltag_size_m, camera_hensuu.camera_focal_length_px)
        tags = TagStore()
        previous: tuple[object, ...] | None = None
        last_report = 0.0
        print("左右カメラ接続: OK")

        while True:
            left, right = camera.read()
            # カメラごとの認識を表示する。判定用には両方の検出Tagを渡す。
            left_ids = _ids_in_frame(detector, left)
            right_ids = _ids_in_frame(detector, right)
            tags.update(detector.detect(left))
            tags.update(detector.detect(right))
            choice = choose_panel_target(tags, cfg.panel_rows, camera_hensuu.tag_max_age_sec)
            current = (left_ids, right_ids, choice.label if choice else "なし")
            now = time.monotonic()
            if current != previous or now - last_report >= 1.0:
                print(f"左カメラ: {left_ids or 'パネルTagなし'}")
                print(f"右カメラ: {right_ids or 'パネルTagなし'}")
                if choice is None:
                    print("狙い: なし（Tag 14〜22をカメラへ向けてください）")
                elif len(choice.tag_ids) == 1:
                    print(f"狙い: {choice.label}")
                else:
                    print(f"狙い: {choice.label} の中間（2枚同時狙い）")
                print()
                previous, last_report = current, now
            time.sleep(0.03)
    except KeyboardInterrupt:
        print("終了します")
    finally:
        if camera is not None:
            camera.close()


if __name__ == "__main__":
    main()
