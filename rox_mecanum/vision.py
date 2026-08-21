"""AprilTagとステレオカメラの共通部品。

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

    @property
    def horizontal_error(self) -> float:
        """画像中心からの左右ずれ。左=-1、右=+1。"""
        if self.image_width <= 0:
            return 0.0
        return (self.center_x - self.image_width / 2.0) / (self.image_width / 2.0)


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
            result.append(TagObservation(int(raw_id), center_x, center_y, int(width), distance, timestamp))
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
    )
