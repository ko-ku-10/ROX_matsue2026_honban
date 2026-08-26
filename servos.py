"""hensuu.py の設定だけでcatch（ID 5）とlift（ID 6）を扱う高水準サーボAPI。"""

from __future__ import annotations

import time
from dataclasses import dataclass
from threading import Event, Lock, Thread, current_thread

import hensuu
from rox_mecanum import (
    ATEncoderReader,
    EncoderFeedback,
    ATMotor,
    EncoderPositionServo,
    PositionServoConfig,
    PySerialTransport,
    at_address_from_can_id,
)


@dataclass
class ServoMotors:
    """catch/liftの2台をまとめたもの。設定はすべてhensuu.pyから読む。"""

    catch: EncoderPositionServo
    lift: EncoderPositionServo
    reader: ATEncoderReader
    transport: PySerialTransport
    owns_transport: bool = True

    def __post_init__(self) -> None:
        self._lock = Lock()
        self._stop_event = Event()
        self._thread: Thread | None = None

    def attach(self) -> None:
        """2台を停止状態で有効化する。前回の速度指令による急発進を防ぐ。"""
        for servo in (self.catch, self.lift):
            servo.motor.stop()  # 有効化前に古い速度指令を上書きする。
        for servo in (self.catch, self.lift):
            servo.enable(retries=3)
            servo.motor.stop()  # 有効化直後にも必ず停止を送る。

    def home(self, catch_count: int, lift_count: int) -> None:
        """指定した生エンコーダー値を両方の0°として登録する。"""
        self.catch.set_home(catch_count)
        self.lift.set_home(lift_count)

    def home_feedbacks(self, values: dict[str, EncoderFeedback]) -> None:
        """最新の角度応答を原点として登録する。登録後もPIDはオフのまま。"""
        for name in ("catch", "lift"):
            feedback = values[name]
            servo = self._servo(name)
            if feedback.position_rad is not None:
                servo.set_home_radians(feedback.position_rad)
            else:
                raise RuntimeError(
                    f"{name} から正式mechPos(0x7019 float)を受信できませんでした。"
                    "angle_monitor.pyでAT応答形式を確認してください"
                )

    def home_from_feedback(
        self,
        timeout_sec: float = 5.0,
        names: tuple[str, ...] = ("catch", "lift"),
    ) -> None:
        """指定した機構の現在位置を0°として登録する。"""
        if not names or any(name not in {"catch", "lift"} for name in names):
            raise ValueError("names は catch/lift を1台以上指定してください")
        deadline = time.monotonic() + timeout_sec
        values: dict[str, EncoderFeedback] = {}
        while time.monotonic() < deadline and len(values) < len(names):
            # AT変換器が応答を落とさないよう、初期化では1台ずつ50ms待つ。
            for name in names:
                if name in values:
                    continue
                self.reader.request(name)
                time.sleep(0.05)
                for feedback in self.reader.poll():
                    # attach()直後には有効化・停止に対する旧ステータス応答も来る。
                    # 原点には正式なmechPos(0x7019 float)だけを絶対に採用する。
                    if feedback.name in names and feedback.position_rad is not None:
                        values[feedback.name] = feedback
        if set(values) != set(names):
            requested = "/".join(names)
            raise TimeoutError(f"{requested}のエンコーダー応答を受信できませんでした")
        for name in names:
            feedback = values[name]
            self._servo(name).set_home_radians(feedback.position_rad)

    def home_to_stop(
        self,
        name: str,
        *,
        speed_percent: float,
        direction: int,
        stillness_deg: float,
        stillness_sec: float,
        timeout_sec: float,
    ) -> None:
        """指定した機構をストッパーまで動かし、停止位置を0°として登録する。

        ``direction=1`` は論理角度が増える向きへ動かす。ストッパーと逆へ
        動く実機だけ ``-1`` にする。リミットスイッチが無いため、mechPosの変化が
        ``stillness_sec`` の間 ``stillness_deg`` 以下になった時だけ成功にする。
        """
        servo = self._servo(name)
        if not 0.0 < speed_percent <= 100.0:
            raise ValueError("speed_percent は0より大きく100以下にしてください")
        if direction not in (-1, 1):
            raise ValueError("direction は 1 または -1 にしてください")
        if stillness_deg <= 0.0 or stillness_sec <= 0.0 or timeout_sec <= 0.0:
            raise ValueError("停止判定とタイムアウトは0より大きくしてください")

        speed = (speed_percent / 100.0) * direction * servo.config.direction
        deadline = time.monotonic() + timeout_sec
        quiet_since: float | None = None
        previous_position: float | None = None
        latest_position: float | None = None

        print(f"{name}原点合わせ: ストッパーへ {speed_percent:.1f}% で動かします")
        try:
            while time.monotonic() < deadline:
                # PIDを使わず、指定した1台だけを直接低速でストッパーへ動かす。
                servo.motor.set_velocity(speed, force=True)
                self.reader.request(name)
                time.sleep(0.04)

                now = time.monotonic()
                received_position = False
                for feedback in self.reader.poll(now):
                    if feedback.name == name and feedback.position_rad is not None:
                        latest_position = feedback.position_rad
                        received_position = True

                # 新しいmechPos応答が無い周期を「停止」と誤認しない。
                if not received_position or latest_position is None:
                    continue
                if previous_position is None:
                    previous_position = latest_position
                    continue

                moved_deg = abs((latest_position - previous_position) * 180.0 / 3.141592653589793)
                previous_position = latest_position
                if moved_deg <= stillness_deg:
                    quiet_since = now if quiet_since is None else quiet_since
                    if now - quiet_since >= stillness_sec:
                        servo.set_home_radians(latest_position)
                        print(f"{name}原点合わせ完了: ストッパー位置を0度に登録しました")
                        return
                else:
                    quiet_since = None
        finally:
            # 成功・失敗のどちらでも、ストッパーへ押し続けない。
            servo.motor.stop()

        raise TimeoutError(
            f"{name}原点合わせ失敗: ストッパーを検出できませんでした。"
            "方向・配線・速度を確認してください"
        )

    def update(self) -> None:
        """エンコーダーを要求・受信し、両方のPID保持を1回更新する。50Hzで呼ぶ。"""
        with self._lock:
            now = time.monotonic()
            # まず前周期に送った要求の応答を処理し、その後に次の1台を要求する。
            for feedback in self.reader.poll(now):
                if feedback.name == "catch":
                    self.catch.update_feedback(feedback)
                elif feedback.name == "lift":
                    self.lift.update_feedback(feedback)
            self.reader.request_next()
            # 応答が消えた時、古い速度を出し続けないための安全停止。
            self.catch.watchdog(now)
            self.lift.watchdog(now)

    def start_pid(self, hz: float | None = None) -> None:
        """PID更新をバックグラウンドで開始する。以後 ``update()`` は不要。"""
        if self._thread is not None and self._thread.is_alive():
            return
        frequency = float(hensuu.encoder_poll_hz if hz is None else hz)
        if frequency <= 0.0:
            raise ValueError("hz は0より大きくしてください")
        self._stop_event.clear()
        self._thread = Thread(target=self._pid_loop, args=(1.0 / frequency,), daemon=True, name="rox-servo-pid")
        self._thread.start()

    def stop_pid(self) -> None:
        """バックグラウンドPID更新だけを止める。各モーターの保持状態は変えない。"""
        self._stop_event.set()
        if self._thread is not None and self._thread is not current_thread():
            self._thread.join(timeout=1.0)
        self._thread = None

    def _pid_loop(self, interval: float) -> None:
        while not self._stop_event.is_set():
            started = time.monotonic()
            self.update()
            self._stop_event.wait(max(0.0, interval - (time.monotonic() - started)))

    def release(self) -> None:
        """2台ともPID保持を解除して停止する。"""
        self.catch.release()
        self.lift.release()

    def pid_on(self, name: str) -> None:
        """``'catch'`` または ``'lift'`` のPID保持をオンにする。"""
        self._servo(name).pid_on()

    def pid_off(self, name: str) -> None:
        """``'catch'`` または ``'lift'`` のPID保持をオフにする。"""
        self._servo(name).pid_off()

    def hold_current(self, name: str) -> float:
        """指定機構の今の実測位置を、そのまま保持目標にする。"""
        with self._lock:
            return self._servo(name).hold_current()

    def hold_all_current(self) -> None:
        """catch/liftの現在位置をそれぞれ保持する。起動時の固定に使う。"""
        with self._lock:
            self.catch.hold_current()
            self.lift.hold_current()

    def set_pid(
        self,
        name: str,
        *,
        kp: float | None = None,
        ki: float | None = None,
        kd: float | None = None,
        max_speed_percent: float | None = None,
        tolerance_deg: float | None = None,
    ) -> PositionServoConfig:
        """指定したサーボのPID値を実行中に変更する。速度は百分率で指定する。"""
        max_speed = None if max_speed_percent is None else float(max_speed_percent) / 100.0
        with self._lock:
            return self._servo(name).set_pid(
                kp=kp,
                ki=ki,
                kd=kd,
                max_speed=max_speed,
                tolerance_deg=tolerance_deg,
            )

    def _servo(self, name: str) -> EncoderPositionServo:
        if name == "catch":
            return self.catch
        if name == "lift":
            return self.lift
        raise ValueError("name は 'catch' または 'lift' にしてください")

    def close(self) -> None:
        self.stop_pid()
        self.release()
        if self.owns_transport:
            self.transport.close()


def _config(name: str) -> PositionServoConfig:
    """hensuu.pyの ``catch_*`` / ``lift_*`` 設定からPID設定を作る。"""
    return PositionServoConfig(
        min_angle=getattr(hensuu, f"{name}_min_angle"),
        max_angle=getattr(hensuu, f"{name}_max_angle"),
        counts_per_degree=getattr(hensuu, f"{name}_counts_per_degree"),
        kp=getattr(hensuu, f"{name}_pid_kp"),
        ki=getattr(hensuu, f"{name}_pid_ki"),
        kd=hensuu.servo_pid_kd,
        integral_limit=hensuu.servo_pid_integral_limit,
        max_speed=hensuu.servo_max_speed_percent / 100.0,
        tolerance_deg=hensuu.servo_tolerance_deg,
        feedback_timeout_sec=hensuu.servo_feedback_timeout_sec,
        direction=getattr(hensuu, f"{name}_direction"),
    )


def open_servos(transport: PySerialTransport | None = None, reader: ATEncoderReader | None = None) -> ServoMotors:
    """catch/liftを開く。

    通常は引数なしで使う。メカナムと同じ通信を共有する場合だけ、既存の
    ``transport`` と、その通信を読む ``reader`` を渡す。
    """
    owns_transport = transport is None
    transport = transport or PySerialTransport.open(hensuu.serial_port, hensuu.serial_baud, minimum_interval=0.0008)
    catch_address = at_address_from_can_id(hensuu.catch_can_id)
    lift_address = at_address_from_can_id(hensuu.lift_can_id)
    reader = reader or ATEncoderReader(transport, {"catch": catch_address, "lift": lift_address})
    return ServoMotors(
        # PIDは小さい補正速度も必要。通常の手動モーター用の6%停止帯を
        # ここで使うと、最大5%の安全なPID出力がすべて0になってしまう。
        catch=EncoderPositionServo(ATMotor(transport, catch_address, zero_hold_band=0.0), _config("catch")),
        lift=EncoderPositionServo(ATMotor(transport, lift_address, zero_hold_band=0.0), _config("lift")),
        reader=reader,
        transport=transport,
        owns_transport=owns_transport,
    )
