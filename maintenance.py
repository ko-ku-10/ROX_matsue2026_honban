"""本番では使わない、カメラ・Tag・駆動の部分実験用プログラム。

カメラだけ確認: ``python3 maintenance.py --camera-only``
ロボットも接続して確認: ``python3 maintenance.py``
"""

from __future__ import annotations

import argparse
import socket
import time

import camera_hensuu
import game1_hensuu
import game2_hensuu
import hensuu
from rox_mecanum import (
    AprilTagDetector,
    Button,
    MaintenanceSite,
    MotionCommand,
    open_camera,
    RobotRuntime,
    TagStore,
)


TEST_SECONDS = 0.4


def _jpeg_with_tags(image: object, observations: list[object]) -> bytes:
    """映像へTag番号を重ねてJPEG化する。OpenCVはmaintenance時だけ使う。"""
    import cv2

    for item in observations:
        x, y = int(item.center_x), int(item.center_y)
        label = f"Tag {item.tag_id}"
        if item.distance_m is not None:
            label += f" {item.distance_m:.2f}m"
        cv2.circle(image, (x, y), 8, (0, 255, 0), 2)
        cv2.putText(image, label, (x + 10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
    ok, encoded = cv2.imencode(".jpg", image)
    return bytes(encoded) if ok else b""


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera-only", action="store_true", help="モーターを開かず、カメラだけ確認する")
    args = parser.parse_args()

    runtime = None if args.camera_only else RobotRuntime.open(with_solenoid=True)
    camera = None
    site = None
    test_command = MotionCommand.stop()
    test_until = 0.0

    def action(name: str) -> str:
        nonlocal test_command, test_until
        if runtime is None:
            return "--camera-onlyでは駆動テストできません"
        commands = {
            "forward": MotionCommand.forward_motion(0.15),
            "backward": MotionCommand.backward(0.15),
            "left": MotionCommand.strafe_left(0.15),
            "right": MotionCommand.strafe_right(0.15),
        }
        if name == "stop":
            test_command = MotionCommand.stop()
            test_until = 0.0
            runtime.mecanum.stop()
            return "停止しました"
        if name == "solenoid":
            runtime.fire()
            return "ソレノイドを短時間オンにしました"
        if name not in commands:
            return "未対応のテストです"
        test_command = commands[name]
        test_until = time.monotonic() + TEST_SECONDS
        return f"{name} を {TEST_SECONDS:.1f}秒テストします"

    try:
        camera = open_camera(
            backend=camera_hensuu.camera_backend, device=camera_hensuu.camera_device,
            pipe_id=camera_hensuu.mipi_pipe_id, host_index=camera_hensuu.mipi_host_index, fps=camera_hensuu.mipi_fps,
            width=camera_hensuu.mipi_width, height=camera_hensuu.mipi_height,
            fisheye_calibration_file=camera_hensuu.fisheye_calibration_file if camera_hensuu.fisheye_enabled else None,
            fisheye_balance=camera_hensuu.fisheye_balance,
        )
        detector = AprilTagDetector(camera_hensuu.apriltag_size_m, camera_hensuu.camera_focal_length_px)
        tags = TagStore()
        site = MaintenanceSite(hensuu.dashboard_port, action)
        host = socket.gethostbyname(socket.gethostname())
        print(f"メンテナンス画面: http://{host}:{hensuu.dashboard_port}")
        print("CREATEでブラウザ駆動テストを10秒有効化 / OPTIONSまたはCtrl+Cで停止")

        while True:
            started = time.monotonic()
            image = camera.read()
            observations = detector.detect(image)
            tags.update(observations)
            site.set_frame("left", _jpeg_with_tags(image, observations))

            state_info: dict[str, object] = {"camera_only": args.camera_only}
            if runtime is not None:
                state = runtime.controller.read()
                if state.button(Button.OPTIONS):
                    print("OPTIONS: 非常停止")
                    break
                if state.was_pressed(Button.CREATE):
                    site.arm(10.0)
                    print("ブラウザ駆動テストを10秒有効化しました")
                if started < test_until:
                    runtime.mecanum.drive(test_command)
                else:
                    runtime.mecanum.drive(runtime.manual_command(state))
                runtime.update_outputs()
                state_info.update(
                    active_buttons=[button.value for button in state.active_buttons],
                    catch=runtime.servos.catch.status(),
                    lift=runtime.servos.lift.status(),
                )

            monitored_tag_ids = (*game1_hensuu.game1_tag_ids, *game2_hensuu.game2_tag_ids)
            fresh = tags.fresh(monitored_tag_ids, camera_hensuu.tag_max_age_sec)
            state_info.update(
                message="カメラ・Tagを確認中",
                tags={tag_id: {"x_error": round(item.horizontal_error, 3), "distance_m": item.distance_m} for tag_id, item in fresh.items()},
                test_active=started < test_until,
            )
            site.set_status(**state_info)
            time.sleep(max(0.0, 1.0 / 30.0 - (time.monotonic() - started)))
    except KeyboardInterrupt:
        pass
    finally:
        if site is not None:
            site.close()
        if camera is not None:
            camera.close()
        if runtime is not None:
            runtime.close()


if __name__ == "__main__":
    main()
