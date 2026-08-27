"""AprilTagと本番用単眼カメラの共通部品。

OpenCVは実機でのみ必要。カメラが未接続のPCでも、このモジュールの
データ構造やゲーム状態機械は利用・テストできる。
"""

from __future__ import annotations

from dataclasses import dataclass
from time import monotonic
from typing import Iterable


@dataclass(frozen=True)
class TagObservation:
    """1枚のAprilTagの最新観測値。距離はカメラからのm単位。"""

    tag_id: int
    center_x: float
    center_y: float
    image_width: int
    distance_m: float | None
    timestamp: float
    pixel_size: float | None = None

    @property
    def horizontal_error(self) -> float:
        """画像中心からの左右ずれ。左=-1、右=+1。"""
        if self.image_width <= 0:
            return 0.0
        return (self.center_x - self.image_width / 2.0) / (self.image_width / 2.0)


def robot_center_horizontal_error(
    target: TagObservation,
    *,
    camera_lateral_offset_m: float = 0.0,
    focal_length_px: float = 0.0,
) -> float:
    """カメラの横取付けずれを補正した、ロボット中心基準の左右ずれを返す。

    ``camera_lateral_offset_m`` はロボット中心から見て、右が正・左が負。
    カメラが右へ付いている場合、ロボット正面のTagは画像では少し左に見える。
    その自然なずれを距離に応じて差し引く。距離または焦点距離が不明な時は、
    安全に従来どおり画像中心基準の値を返す。
    """
    error = target.horizontal_error
    if (
        target.distance_m is None
        or target.distance_m <= 0.0
        or focal_length_px <= 0.0
        or target.image_width <= 0
    ):
        return error
    half_width = target.image_width / 2.0
    camera_offset_error = (float(camera_lateral_offset_m) * float(focal_length_px)) / (target.distance_m * half_width)
    return error + camera_offset_error


class TagStore:
    """古い検出を現在位置として使わないTag保管庫。"""

    def __init__(self) -> None:
        self._latest: dict[int, TagObservation] = {}

    def update(self, observations: Iterable[TagObservation]) -> None:
        for observation in observations:
            self._latest[observation.tag_id] = observation

    def get(self, tag_id: int, max_age_sec: float = 0.35, now: float | None = None) -> TagObservation | None:
        observation = self._latest.get(int(tag_id))
        current = monotonic() if now is None else now
        if observation is None or current - observation.timestamp > max_age_sec:
            return None
        return observation

    def fresh(self, ids: Iterable[int], max_age_sec: float = 0.35) -> dict[int, TagObservation]:
        return {tag_id: item for tag_id in ids if (item := self.get(tag_id, max_age_sec)) is not None}

    def snapshot(self) -> dict[int, TagObservation]:
        return dict(self._latest)


class AprilTagDetector:
    """OpenCV ArUcoの tag16h5 検出器。

    ``detect`` はOpenCV画像を受け取る。利用前に ``rox-mecanum[vision]``
    をインストールする。
    """

    def __init__(self, tag_size_m: float = 0.180, focal_length_px: float = 0.0) -> None:
        self.tag_size_m = float(tag_size_m)
        self.focal_length_px = float(focal_length_px)
        try:
            import cv2
        except ImportError as error:  # pragma: no cover - 実機依存
            raise RuntimeError("OpenCVが必要です: pip install 'rox-mecanum[vision]'") from error
        if not hasattr(cv2, "aruco"):
            raise RuntimeError("opencv-contrib-python をインストールしてください")
        self._cv2 = cv2
        self._dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_16h5)
        self._parameters = cv2.aruco.DetectorParameters()
        self._detector = cv2.aruco.ArucoDetector(self._dictionary, self._parameters)

    def detect(self, image: object) -> list[TagObservation]:
        corners, ids, _ = self._detector.detectMarkers(image)
        if ids is None:
            return []
        height, width = image.shape[:2]
        timestamp = monotonic()
        result: list[TagObservation] = []
        for corner, raw_id in zip(corners, ids.flatten()):
            points = corner.reshape(4, 2)
            center_x = float(sum(point[0] for point in points) / 4.0)
            center_y = float(sum(point[1] for point in points) / 4.0)
            edge_lengths = []
            for index in range(4):
                a, b = points[index], points[(index + 1) % 4]
                edge_lengths.append(float(((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5))
            pixel_size = sum(edge_lengths) / len(edge_lengths)
            distance = None
            if self.focal_length_px > 0.0 and pixel_size > 0.0:
                distance = self.tag_size_m * self.focal_length_px / pixel_size
            result.append(TagObservation(int(raw_id), center_x, center_y, int(width), distance, timestamp, pixel_size))
        return result


class OpenCVStereoCamera:
    """V4L2として見える左右カメラを読む最小アダプター。

    RDKのMIPIカメラが ``/dev/video*`` として公開される設定で使う。別の
    RDK入力方式の場合も、``read`` と ``close`` が同じアダプターを追加すれば
    ゲーム側のコードは変えずに済む。
    """

    def __init__(self, left_device: int | str = 0, right_device: int | str = 1) -> None:
        try:
            import cv2
        except ImportError as error:  # pragma: no cover - 実機依存
            raise RuntimeError("OpenCVが必要です: pip install 'rox-mecanum[vision]'") from error
        self._cv2 = cv2
        self.left = cv2.VideoCapture(left_device)
        self.right = cv2.VideoCapture(right_device)
        if not self.left.isOpened() or not self.right.isOpened():
            self.close()
            raise RuntimeError("左右カメラを開けません。camera_hensuu.py の番号を確認してください")

    def read(self) -> tuple[object, object]:
        left_ok, left = self.left.read()
        right_ok, right = self.right.read()
        if not left_ok or not right_ok:
            raise RuntimeError("ステレオカメラの画像を取得できません")
        return left, right

    def close(self) -> None:
        for capture in (getattr(self, "left", None), getattr(self, "right", None)):
            if capture is not None:
                capture.release()


class OpenCVSingleCamera:
    """USB/V4L2カメラを1台だけ読むアダプター。"""

    def __init__(self, device: int | str = 0) -> None:
        try:
            import cv2
        except ImportError as error:  # pragma: no cover - 実機依存
            raise RuntimeError("OpenCVが必要です: pip install 'rox-mecanum[vision]'") from error
        self.camera = cv2.VideoCapture(device)
        if not self.camera.isOpened():
            self.close()
            raise RuntimeError("カメラを開けません。camera_hensuu.py を確認してください")

    def read(self) -> object:
        ok, image = self.camera.read()
        if not ok:
            raise RuntimeError("カメラ画像を取得できません")
        return image

    def close(self) -> None:
        camera = getattr(self, "camera", None)
        if camera is not None:
            camera.release()


class RDKMIPIStereoCamera:
    """RDK X5の ``hobot_vio.libsrcampy`` を使うMIPIステレオカメラ。

    RDK公式サンプルの ``libsrcampy.Camera().open_cam()`` と同じ方式。
    MIPIカメラは通常 ``/dev/video*`` を作らないため、このクラスを使う。
    """

    def __init__(
        self,
        left_pipe_id: int = 0,
        left_host_index: int = -1,
        right_pipe_id: int = 1,
        right_host_index: int = -1,
        fps: int = 30,
        width: int = 1920,
        height: int = 1080,
    ) -> None:
        try:
            import cv2
            import numpy as np
            from hobot_vio import libsrcampy
        except ImportError as error:  # pragma: no cover - RDK実機依存
            raise RuntimeError("RDK X5用のhobot_vioとOpenCVが必要です") from error
        self._cv2 = cv2
        self._np = np
        self.width = int(width)
        self.height = int(height)
        self.left = libsrcampy.Camera()
        self.right = libsrcampy.Camera()
        try:
            if self.left.open_cam(int(left_pipe_id), int(left_host_index), int(fps), self.width, self.height):
                raise RuntimeError(f"左MIPIカメラ(pipe={left_pipe_id}, host={left_host_index})を開けません")
            if self.right.open_cam(int(right_pipe_id), int(right_host_index), int(fps), self.width, self.height):
                raise RuntimeError(f"右MIPIカメラ(pipe={right_pipe_id}, host={right_host_index})を開けません")
        except Exception:
            self.close()
            raise

    def read(self) -> tuple[object, object]:
        return self._to_bgr(self.left.get_img(1), "左"), self._to_bgr(self.right.get_img(1), "右")

    def _to_bgr(self, raw: object, name: str) -> object:
        expected = self.width * self.height * 3 // 2
        data = self._np.frombuffer(raw, dtype=self._np.uint8)
        if data.size != expected:
            raise RuntimeError(f"{name}MIPIカメラ画像のサイズが不正です: {data.size} bytes (期待 {expected})")
        nv12 = data.reshape((self.height * 3 // 2, self.width))
        return self._cv2.cvtColor(nv12, self._cv2.COLOR_YUV2BGR_NV12)

    def close(self) -> None:
        for camera in (getattr(self, "left", None), getattr(self, "right", None)):
            if camera is not None:
                try:
                    camera.close_cam()
                except Exception:
                    pass


class RDKMIPICamera:
    """RDK MIPIカメラ1台だけを読むアダプター。診断用に使う。"""

    def __init__(self, pipe_id: int = 0, host_index: int = -1, fps: int = 30, width: int = 1920, height: int = 1080) -> None:
        try:
            import cv2
            import numpy as np
            from hobot_vio import libsrcampy
        except ImportError as error:  # pragma: no cover - RDK実機依存
            raise RuntimeError("RDK X5用のhobot_vioとOpenCVが必要です") from error
        self._cv2 = cv2
        self._np = np
        self.width = int(width)
        self.height = int(height)
        self.camera = libsrcampy.Camera()
        if self.camera.open_cam(int(pipe_id), int(host_index), int(fps), self.width, self.height):
            self.close()
            raise RuntimeError(f"MIPIカメラ(pipe={pipe_id}, host={host_index})を開けません")

    def read(self) -> object:
        raw = self.camera.get_img(1)
        expected = self.width * self.height * 3 // 2
        data = self._np.frombuffer(raw, dtype=self._np.uint8)
        if data.size != expected:
            raise RuntimeError(f"MIPIカメラ画像のサイズが不正です: {data.size} bytes (期待 {expected})")
        nv12 = data.reshape((self.height * 3 // 2, self.width))
        return self._cv2.cvtColor(nv12, self._cv2.COLOR_YUV2BGR_NV12)

    def close(self) -> None:
        camera = getattr(self, "camera", None)
        if camera is not None:
            try:
                camera.close_cam()
            except Exception:
                pass


def open_stereo_camera(
    *,
    backend: str,
    left_device: int | str = 0,
    right_device: int | str = 1,
    left_pipe_id: int = 0,
    left_host_index: int = -1,
    right_pipe_id: int = 1,
    right_host_index: int = -1,
    fps: int = 30,
    width: int = 1920,
    height: int = 1080,
) -> OpenCVStereoCamera | RDKMIPIStereoCamera:
    """設定値に従ってV4L2またはRDK MIPIのステレオカメラを開く。"""
    if backend == "rdk_mipi":
        return RDKMIPIStereoCamera(
            left_pipe_id, left_host_index, right_pipe_id, right_host_index, fps, width, height,
        )
    if backend == "v4l2":
        return OpenCVStereoCamera(left_device, right_device)
    raise ValueError("camera_backend は 'rdk_mipi' または 'v4l2' にしてください")


def open_camera(
    *,
    backend: str,
    device: int | str = 0,
    pipe_id: int = 0,
    host_index: int = -1,
    fps: int = 30,
    width: int = 1920,
    height: int = 1080,
) -> OpenCVSingleCamera | RDKMIPICamera:
    """設定値に従って、本番用の単眼カメラを開く。"""
    if backend == "rdk_mipi":
        camera: OpenCVSingleCamera | RDKMIPICamera = RDKMIPICamera(pipe_id, host_index, fps, width, height)
    elif backend == "v4l2":
        camera = OpenCVSingleCamera(device)
    else:
        raise ValueError("camera_backend は 'rdk_mipi' または 'v4l2' にしてください")
    return camera


def midpoint(first: TagObservation, second: TagObservation) -> TagObservation:
    """2枚のTagの中間を、中心合わせ用の仮想Tagとして返す。"""
    distance_values = [value for value in (first.distance_m, second.distance_m) if value is not None]
    distance = sum(distance_values) / len(distance_values) if distance_values else None
    return TagObservation(
        tag_id=-1,
        center_x=(first.center_x + second.center_x) / 2.0,
        center_y=(first.center_y + second.center_y) / 2.0,
        image_width=first.image_width,
        distance_m=distance,
        timestamp=min(first.timestamp, second.timestamp),
        pixel_size=(first.pixel_size + second.pixel_size) / 2.0 if first.pixel_size is not None and second.pixel_size is not None else None,
    )
