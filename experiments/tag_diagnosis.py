"""AprilTagだけを確認する診断。モーター・DualSense・GPIOは開かない。

実行: ``python3 -m experiments.tag_diagnosis``
終了: Ctrl+C
"""

from __future__ import annotations

import time

import camera_hensuu
from rox_mecanum import AprilTagDetector, open_camera


def main() -> None:
    print("AprilTag診断を開始します。モーターは一切動かしません。")
    camera = open_camera(
        backend=camera_hensuu.camera_backend,
        device=camera_hensuu.camera_device,
        pipe_id=camera_hensuu.mipi_pipe_id,
        host_index=camera_hensuu.mipi_host_index,
        fps=camera_hensuu.mipi_fps,
        width=camera_hensuu.mipi_width,
        height=camera_hensuu.mipi_height,
    )
    detector = AprilTagDetector(camera_hensuu.apriltag_size_m, camera_hensuu.camera_focal_length_px)
    last_ids: tuple[int, ...] | None = None
    last_notice = 0.0
    try:
        while True:
            image = camera.read()
            observations = detector.detect(image)
            ids = tuple(sorted(item.tag_id for item in observations))
            now = time.monotonic()
            if ids != last_ids:
                if ids:
                    print("検出ID:", ", ".join(str(tag_id) for tag_id in ids))
                    for item in observations:
                        print(
                            f"  Tag{item.tag_id}: x={item.horizontal_error:+.3f} "
                            f"distance={item.distance_m!s} yaw={item.yaw_degrees!s}"
                        )
                else:
                    print("検出ID: なし")
                last_ids = ids
            elif not ids and now - last_notice >= 2.0:
                print("検出ID: なし（Tagの全体・照明・印刷サイズを確認）")
                last_notice = now
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("診断を終了します")
    finally:
        camera.close()


if __name__ == "__main__":
    main()
