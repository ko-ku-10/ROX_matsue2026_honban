"""AT変換器が返す16bit位置値と実機角度の対応を測る。速度指令は送らない。"""

from __future__ import annotations

import time

import hensuu
from rox_mecanum import ATEncoderReader, PySerialTransport, at_address_from_can_id


COUNTS_PER_TURN_HYPOTHESIS = 65536


class RawAngleTracker:
    def __init__(self) -> None:
        self.last_raw: int | None = None
        self.total_delta = 0

    def update(self, raw: int) -> tuple[int, float]:
        raw = int(raw)
        if self.last_raw is not None:
            delta = raw - self.last_raw
            if delta > COUNTS_PER_TURN_HYPOTHESIS // 2:
                delta -= COUNTS_PER_TURN_HYPOTHESIS
            elif delta < -COUNTS_PER_TURN_HYPOTHESIS // 2:
                delta += COUNTS_PER_TURN_HYPOTHESIS
            self.total_delta += delta
        self.last_raw = raw
        estimated_degrees = self.total_delta * 360.0 / COUNTS_PER_TURN_HYPOTHESIS
        return self.total_delta, estimated_degrees


transport = PySerialTransport.open(hensuu.serial_port, hensuu.serial_baud, minimum_interval=0.0008)
reader = ATEncoderReader(
    transport,
    {
        "catch": at_address_from_can_id(hensuu.catch_can_id),
        "lift": at_address_from_can_id(hensuu.lift_can_id),
    },
)
trackers = {"catch": RawAngleTracker(), "lift": RawAngleTracker()}

try:
    print("AT角度キャリブレーション。Ctrl+Cで終了。モーターは一切動かしません。")
    print("値を記録後、機構を正確に90°動かして、deltaと仮角度を確認してください。")
    print("仮角度は65536カウント=360°と仮定した表示で、PIDには使いません。")
    while True:
        reader.request_all()
        time.sleep(0.02)
        for feedback in reader.poll():
            if feedback.count is None:
                continue
            delta, tentative_deg = trackers[feedback.name].update(feedback.count)
            print(
                f"{feedback.name}: raw={feedback.count:5d} "
                f"delta={delta:+7d} 仮角度={tentative_deg:+8.2f}°"
            )
        time.sleep(0.08)
finally:
    transport.close()
