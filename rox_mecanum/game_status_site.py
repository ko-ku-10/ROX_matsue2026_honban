"""GAME実行中の状態を、ブラウザへ安全に表示する共通部品。"""

from __future__ import annotations

import socket
from time import monotonic
from typing import Iterable

from .maintenance_site import MaintenanceSite
from .vision import TagObservation, TagStore, robot_center_horizontal_error


class GameStatusSite:
    """GAME1〜3で共通の確認専用サイト。

    このクラスはモーター・GPIOへ命令を送らない。ゲーム本体が既に読み取った
    入力・サーボ・カメラの状態を、ブラウザ表示用にまとめるだけである。
    """

    def __init__(self, game_name: str, port: int, camera_hz: float = 3.0) -> None:
        if camera_hz <= 0.0:
            raise ValueError("camera_hz は0より大きくしてください")
        self.game_name = str(game_name)
        self.site = MaintenanceSite(port)
        self._camera_interval = 1.0 / float(camera_hz)
        self._next_camera_at = 0.0

    def url(self) -> str:
        """同じWi-Fiの端末で開くための目安URLを返す。"""
        host = socket.gethostbyname(socket.gethostname())
        return f"http://{host}:{self.site.server.server_port}"

    def camera_due(self, now: float | None = None) -> bool:
        """映像を更新する時刻か。映像更新でゲーム操作が重くならないよう間引く。"""
        current = monotonic() if now is None else float(now)
        if current < self._next_camera_at:
            return False
        self._next_camera_at = current + self._camera_interval
        return True

    def set_camera_frame(self, image: object, observations: Iterable[TagObservation]) -> None:
        """Tag番号を重ねたカメラ映像をサイトへ渡す。"""
        import cv2

        for item in observations:
            x, y = int(item.center_x), int(item.center_y)
            label = f"Tag {item.tag_id}"
            if item.distance_m is not None:
                label += f" {item.distance_m:.2f}m"
            cv2.circle(image, (x, y), 8, (0, 255, 0), 2)
            cv2.putText(image, label, (x + 10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        ok, encoded = cv2.imencode(".jpg", image)
        if ok:
            self.site.set_frame("left", bytes(encoded))

    def update(
        self,
        *,
        runtime: object,
        state: object,
        stage: object,
        mode: object | None,
        tags: TagStore | None,
        tag_max_age_sec: float,
        camera_lateral_offset_m: float,
        camera_focal_length_px: float,
        camera_error: str = "",
    ) -> None:
        """現在のゲーム状態をサイトへ反映する。"""
        now = monotonic()
        controller = {
            "active_buttons": sorted(button.value for button in state.active_buttons),
            "left_stick": {"x": round(state.left_stick.x, 3), "y": round(state.left_stick.y, 3), "magnitude": round(state.left_stick.magnitude, 3)},
            "right_stick": {"x": round(state.right_stick.x, 3), "y": round(state.right_stick.y, 3), "magnitude": round(state.right_stick.magnitude, 3)},
            "l2": round(state.l2, 3),
            "r2": round(state.r2, 3),
        }
        tag_values: dict[int, object] = {}
        if tags is not None:
            for tag_id, item in tags.snapshot().items():
                age = max(0.0, now - item.timestamp)
                tag_values[tag_id] = {
                    "fresh": age <= tag_max_age_sec,
                    "age_sec": round(age, 3),
                    "image_x_error": round(item.horizontal_error, 3),
                    "robot_x_error": round(robot_center_horizontal_error(
                        item,
                        camera_lateral_offset_m=camera_lateral_offset_m,
                        focal_length_px=camera_focal_length_px,
                    ), 3),
                    "distance_m": None if item.distance_m is None else round(item.distance_m, 3),
                    "yaw_degrees": None if item.yaw_degrees is None else round(item.yaw_degrees, 2),
                }
        self.site.set_status(
            game=self.game_name,
            stage=getattr(stage, "value", str(stage)),
            mode=None if mode is None else getattr(mode, "value", str(mode)),
            controller=controller,
            servos={
                "catch": runtime.servos.catch.status(),
                "lift": runtime.servos.lift.status(),
                "pid_error": runtime.servos.pid_error(),
            },
            mecanum=runtime.mecanum.drive_status(),
            camera={"connected": not bool(camera_error), "error": camera_error or None},
            tags=tag_values,
            message="カメラ異常" if camera_error else "状態を更新中",
        )

    def close(self) -> None:
        self.site.close()
