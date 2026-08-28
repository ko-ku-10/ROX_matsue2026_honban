"""本番では使わない、カメラ・Tag・駆動の部分実験用プログラム。

カメラだけ確認: ``python3 -m experiments.maintenance --camera-only``
ロボットも接続して確認: ``python3 -m experiments.maintenance``
"""

from __future__ import annotations

import argparse
import socket
import time

import camera_hensuu
import game1
import game2
import hensuu
import robot_actions
from rox_mecanum import (
    AprilTagDetector,
    Button,
    MaintenanceSite,
    MotionCommand,
    open_camera,
    robot_center_horizontal_error,
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

    runtime = None if args.camera_only else RobotRuntime.open()
    if runtime is not None:
        robot_actions.setup_gpio()
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
            robot_actions.ball_fire(runtime)
            return "robot_actions.py の ball_fire() を実行しました"
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
        )
        detector = AprilTagDetector(camera_hensuu.apriltag_size_m, camera_hensuu.camera_focal_length_px)
        tags = TagStore()
        site = MaintenanceSite(hensuu.dashboard_port, action)
        host = socket.gethostbyname(socket.gethostname())
        print(f"状態監視サイト: http://{host}:{hensuu.dashboard_port}")
        print("同じWi-Fiのスマホ・PCからも、このURLのROBOT_IP部分で開けます")
        print("CREATEでブラウザ駆動テストを10秒有効化 / OPTIONSまたはCtrl+Cで停止")

        while True:
            started = time.monotonic()
            camera_error = None
            try:
                image = camera.read()
                observations = detector.detect(image)
                tags.update(observations)
                site.set_frame("left", _jpeg_with_tags(image, observations))
            except Exception as error:
                # 通信・モーターを止めず、サイト上でカメラ異常として確認できるようにする。
                camera_error = str(error)

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
                state_info.update(
                    controller={
                        "active_buttons": sorted(button.value for button in state.active_buttons),
                        "left_stick": {"x": round(state.left_stick.x, 3), "y": round(state.left_stick.y, 3), "magnitude": round(state.left_stick.magnitude, 3)},
                        "right_stick": {"x": round(state.right_stick.x, 3), "y": round(state.right_stick.y, 3), "magnitude": round(state.right_stick.magnitude, 3)},
                        "l2": round(state.l2, 3),
                        "r2": round(state.r2, 3),
                    },
                    servos={
                        "catch": runtime.servos.catch.status(),
                        "lift": runtime.servos.lift.status(),
                        "pid_error": runtime.servos.pid_error(),
                    },
                    mecanum=runtime.mecanum.drive_status(),
                )

            monitored_tag_ids = (
                game1.TAG_GATE,
                *(tag_id for row in game2.PANEL_ROWS.values() for tag_id in row),
            )
            fresh = tags.fresh(monitored_tag_ids, camera_hensuu.tag_max_age_sec)
            state_info.update(
                message="カメラ異常" if camera_error else "カメラ・Tagを確認中",
                camera={"connected": camera_error is None, "error": camera_error},
                tags={
                    tag_id: {
                        "image_x_error": round(item.horizontal_error, 3),
                        "robot_x_error": round(robot_center_horizontal_error(
                            item,
                            camera_lateral_offset_m=camera_hensuu.camera_lateral_offset_m,
                            focal_length_px=camera_hensuu.camera_focal_length_px,
                        ), 3),
                        "distance_m": None if item.distance_m is None else round(item.distance_m, 3),
                        "age_sec": round(max(0.0, started - item.timestamp), 3),
                    }
                    for tag_id, item in fresh.items()
                },
                test_active=started < test_until,
            )
            site.set_status(**state_info)
            time.sleep(max(0.0, 1.0 / 30.0 - (time.monotonic() - started)))
    except KeyboardInterrupt:
        pass
    finally:
        robot_actions.all_off()
        if site is not None:
            site.close()
        if camera is not None:
            camera.close()
        if runtime is not None:
            runtime.close()
        robot_actions.close_gpio()


if __name__ == "__main__":
    main()
