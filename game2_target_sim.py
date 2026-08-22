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
from rox_mecanum import AprilTagDetector, RDKMIPICamera, TagStore, choose_panel_target


PANEL_IDS = tuple(range(14, 23))


def _ids_in_frame(detector: AprilTagDetector, image: object) -> tuple[int, ...]:
    return tuple(sorted(item.tag_id for item in detector.detect(image) if item.tag_id in PANEL_IDS))


def main() -> None:
    print("GAME2 パネル選択シミュレーション（モーターは動きません）")
    print("左右カメラを開いています。Ctrl+Cで終了します。")
    left_camera = None
    right_camera = None
    try:
        if camera_hensuu.camera_backend != "rdk_mipi":
            raise RuntimeError("この単体診断はRDK MIPIカメラ専用です")
        left_camera = RDKMIPICamera(
            camera_hensuu.left_mipi_pipe_id, camera_hensuu.left_mipi_host_index, camera_hensuu.mipi_fps,
            camera_hensuu.mipi_width, camera_hensuu.mipi_height,
        )
        try:
            right_camera = RDKMIPICamera(
                camera_hensuu.right_mipi_pipe_id, camera_hensuu.right_mipi_host_index, camera_hensuu.mipi_fps,
                camera_hensuu.mipi_width, camera_hensuu.mipi_height,
            )
            right_status = "OK"
        except RuntimeError as error:
            right_status = f"未接続 ({error})"
        detector = AprilTagDetector(camera_hensuu.apriltag_size_m, camera_hensuu.camera_focal_length_px)
        tags = TagStore()
        previous: tuple[object, ...] | None = None
        last_report = 0.0
        print("左カメラ接続: OK")
        print(f"右カメラ接続: {right_status}")

        while True:
            left = left_camera.read()
            right = right_camera.read() if right_camera is not None else None
            # カメラごとの認識を表示する。判定用には両方の検出Tagを渡す。
            left_ids = _ids_in_frame(detector, left)
            right_ids = _ids_in_frame(detector, right) if right is not None else ()
            tags.update(detector.detect(left))
            if right is not None:
                tags.update(detector.detect(right))
            choice = choose_panel_target(tags, cfg.panel_rows, camera_hensuu.tag_max_age_sec)
            current = (left_ids, right_ids, choice.label if choice else "なし")
            now = time.monotonic()
            if current != previous or now - last_report >= 1.0:
                print(f"左カメラ: {left_ids or 'パネルTagなし'}")
                print(f"右カメラ: {right_ids or ('未接続' if right is None else 'パネルTagなし')}")
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
        if right_camera is not None:
            right_camera.close()
        if left_camera is not None:
            left_camera.close()


if __name__ == "__main__":
    main()
