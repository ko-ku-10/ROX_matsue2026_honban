"""カメラ処理で操縦ループを止めないためのバックグラウンド処理。"""

from __future__ import annotations

import threading
from time import monotonic

from .vision import TagStore


class VisionWorker:
    """映像表示・AprilTag検出を別スレッドで行う。

    コントローラー読取りやCAN送信はこのクラスに入れない。カメラが一時的に
    遅くなっても、ゲーム本体の50Hz操作ループを止めないための部品である。
    """

    def __init__(
        self,
        camera: object,
        detector: object | None,
        tags: TagStore,
        status_site: object,
        *,
        camera_hz: float,
        tag_hz: float,
    ) -> None:
        self.camera = camera
        self.detector = detector
        self.tags = tags
        self.status_site = status_site
        self.camera_interval = 1.0 / max(0.1, float(camera_hz))
        self.tag_interval = 1.0 / max(0.1, float(tag_hz))
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._tag_enabled = False
        self._read_tag_now = False
        self._paused = False
        self._error = ""
        self._thread = threading.Thread(target=self._run, daemon=True, name="rox-vision")

    def start(self) -> None:
        self._thread.start()

    def set_tag_detection_enabled(self, enabled: bool) -> None:
        with self._lock:
            self._tag_enabled = bool(enabled)

    def request_tag_read(self) -> None:
        """次のカメラフレームでTag検出を1回行う。操作ループは待たない。"""
        with self._lock:
            self._read_tag_now = True

    def set_paused(self, paused: bool) -> None:
        """操縦中は映像処理を止め、操作応答を最優先にする。"""
        with self._lock:
            self._paused = bool(paused)

    @property
    def error(self) -> str:
        with self._lock:
            return self._error

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)

    def _run(self) -> None:
        next_camera_at = 0.0
        next_tag_at = 0.0
        while not self._stop.is_set():
            now = monotonic()
            with self._lock:
                tag_enabled = self._tag_enabled
                read_tag_now = self._read_tag_now
                self._read_tag_now = False
                paused = self._paused
            if paused:
                # カメラread・OpenCV・JPEG化を一切実行しない。
                self._stop.wait(0.02)
                continue
            camera_due = now >= next_camera_at
            tag_due = self.detector is not None and (read_tag_now or (tag_enabled and now >= next_tag_at))
            if not camera_due and not tag_due:
                self._stop.wait(0.005)
                continue
            try:
                image = self.camera.read()
                observations = self.detector.detect(image) if tag_due else []
                if tag_due:
                    self.tags.update(observations)
                    next_tag_at = now + self.tag_interval
                if camera_due:
                    self.status_site.set_camera_frame(image, observations)
                    next_camera_at = now + self.camera_interval
                with self._lock:
                    self._error = ""
            except Exception as error:  # pragma: no cover - 実機カメラ依存
                with self._lock:
                    self._error = str(error)
                self._stop.wait(0.05)
