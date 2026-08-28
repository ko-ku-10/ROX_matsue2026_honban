"""単眼カメラ用のAprilTag距離校正。

実行:
  python3 -m experiments.calibrate_distance 1.00

180 mmの公式AprilTagをカメラ正面ちょうど1.00 mに置く。表示された
``camera_focal_length_px`` を camera_hensuu.py へ転記する。

画面中央・Tag正面の測定だけを使う。魚眼カメラの端や斜めのTagを混ぜると、
距離補正が狂うためである。
"""

from __future__ import annotations

import argparse
import statistics
import time

import camera_hensuu
from rox_mecanum import AprilTagDetector, open_camera


def main() -> None:
    parser = argparse.ArgumentParser(description="AprilTag実寸から単眼距離を校正する")
    parser.add_argument("distance_m", type=float, help="Tag正面までを測った距離[m]。例: 1.00")
    parser.add_argument("--tag-id", type=int, default=0, help="映すTag番号。既定: 0")
    parser.add_argument("--samples", type=int, default=30, help="採用する測定回数。既定: 30")
    args = parser.parse_args()
    if args.distance_m <= 0.0:
        raise ValueError("distance_m は0より大きくしてください")
    if args.samples < 5:
        raise ValueError("--samples は5以上にしてください")

    camera = open_camera(
        backend=camera_hensuu.camera_backend, device=camera_hensuu.camera_device,
        pipe_id=camera_hensuu.mipi_pipe_id, host_index=camera_hensuu.mipi_host_index,
        fps=camera_hensuu.mipi_fps, width=camera_hensuu.mipi_width, height=camera_hensuu.mipi_height,
    )
    # 現在の焦点距離を使うと、Tag面の角度も確認できる。
    # 校正の計算自体は observation.pixel_size だけを使う。
    detector = AprilTagDetector(camera_hensuu.apriltag_size_m, camera_hensuu.camera_focal_length_px)
    values: list[float] = []
    print(f"Tag {args.tag_id} をカメラ正面 {args.distance_m:.3f}m に固定してください。")
    print(
        f"画面中央±{camera_hensuu.tag_yaw_trust_center_error:.2f}、"
        f"Tag角度±{camera_hensuu.tag_yaw_tolerance_deg:.1f}°以内の"
        f"{args.samples}回だけを採用します。"
    )
    try:
        while len(values) < args.samples:
            image = camera.read()
            for observation in detector.detect(image):
                if observation.tag_id != args.tag_id or observation.pixel_size is None:
                    continue
                if abs(observation.horizontal_error) > camera_hensuu.tag_yaw_trust_center_error:
                    print("Tagが画面端です。カメラ正面へ動かしてください。")
                    break
                if observation.yaw_degrees is None or abs(observation.yaw_degrees) > camera_hensuu.tag_yaw_tolerance_deg:
                    print("Tagが斜めです。カメラと平行にしてください。")
                    break
                # focal = 画像上のTag辺長[pixel] × 実測距離[m] ÷ Tag実寸[m]
                values.append(observation.pixel_size * args.distance_m / camera_hensuu.apriltag_size_m)
                print(f"取得 {len(values)}/{args.samples}")
                break
            time.sleep(0.03)
        focal_length = statistics.median(values)
        spread = statistics.pstdev(values)
        print()
        print(f"採用値: camera_focal_length_px = {focal_length:.2f}")
        print(f"ばらつき: ±{spread:.2f}px")
        print("この値を camera_hensuu.py の camera_focal_length_px へ貼り付けてください。")
    finally:
        camera.close()


if __name__ == "__main__":
    main()
