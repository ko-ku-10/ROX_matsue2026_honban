"""hensuu.py の設定だけでcatch（ID 5）とlift（ID 6）を扱う高水準サーボAPI。"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from math import atan2, cos, degrees, isfinite, sin
from pathlib import Path
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
        # PIDスレッド内で起きた最後の通信例外。画面表示や原因確認に使える。
        self._pid_error: str | None = None
        # PIDが最後に受信した正式mechPos応答。GAME画面の診断表示に使う。
        self._latest_mechpos_feedback: dict[str, EncoderFeedback] = {}

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
        values = self.read_mech_positions(timeout_sec=timeout_sec, names=names)
        self.home_feedbacks(values)

    def save_origins(self, file_path: str | Path) -> None:
        """現在登録済みのcatch/lift原点をJSONファイルへ保存する。

        原点はEDULITE 05の正式mechPos[rad]で保存する。まず ``home_to_stop()``
        などで物理0度を登録してから呼ぶ。
        """
        origins: dict[str, float] = {}
        for name in ("catch", "lift"):
            position = self._servo(name).home_position_rad
            if position is None or not isfinite(position):
                raise RuntimeError(
                    f"{name}の0度が未登録です。先にストッパー原点合わせをしてください"
                )
            origins[name] = float(position)

        path = Path(file_path)
        data = {
            "format": "edulite05-mechpos-origin-v1",
            "unit": "rad",
            "catch": origins["catch"],
            "lift": origins["lift"],
        }
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        temporary.replace(path)

    def load_origins(self, file_path: str | Path) -> None:
        """保存済みの0度を読み込む。モーターは動かさない。"""
        path = Path(file_path)
        if not path.is_file():
            raise FileNotFoundError(
                f"原点ファイルがありません: {path}。"
                "一度だけ python3 set_servo_origins.py を実行してください"
            )
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            if data.get("format") != "edulite05-mechpos-origin-v1":
                raise ValueError("原点ファイルの形式が違います")
            origins = {name: float(data[name]) for name in ("catch", "lift")}
        except (OSError, ValueError, TypeError, KeyError, json.JSONDecodeError) as error:
            raise RuntimeError(f"原点ファイルを読めません: {path}") from error

        for name, position in origins.items():
            if not isfinite(position):
                raise RuntimeError(f"原点ファイルの{name}が不正です")
            self._servo(name).set_home_radians(position)

    def refresh_positions_from_feedback(
        self,
        timeout_sec: float = 1.0,
        names: tuple[str, ...] = ("catch", "lift"),
    ) -> None:
        """保存済み原点を使って、現在角度を1回だけ更新する。モーターは動かない。"""
        values = self.read_mech_positions(timeout_sec=timeout_sec, names=names)
        now = time.monotonic()
        for name in names:
            self._servo(name).update_feedback(values[name])

    def read_mech_positions(
        self,
        timeout_sec: float = 5.0,
        names: tuple[str, ...] = ("catch", "lift"),
    ) -> dict[str, EncoderFeedback]:
        """正式なmechPosを読み取るだけで、原点・PID目標は変更しない。"""
        if not names or any(name not in {"catch", "lift"} for name in names):
            raise ValueError("names は catch/lift を1台以上指定してください")
        deadline = time.monotonic() + timeout_sec
        values: dict[str, EncoderFeedback] = {}
        while time.monotonic() < deadline and len(values) < len(names):
            # angle_monitor.py と同じ手順。USB-AT変換器へcatch/liftを交互に
            # 要求して15ms待ってから受信する。この実機で両方のmechPosが読める
            # ことを確認済みの通信間隔である。
            self.reader.request_next()
            time.sleep(0.015)
            for feedback in self.reader.poll():
                # 原点には正式なmechPos(0x7019 float)だけを絶対に採用する。
                if feedback.name in names and feedback.position_rad is not None:
                    values[feedback.name] = feedback
            time.sleep(0.015)
        if set(values) != set(names):
            requested = "/".join(names)
            raise TimeoutError(f"{requested}のエンコーダー応答を受信できませんでした")
        return values

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
        # 毎周期の速度フレームはmechPos要求より多くなり、USB-AT変換器が
        # 応答を落とすことがある。最初に動かし、以後は0.25秒ごとだけ
        # 再送する。モーターは最後の速度指令を保持する仕様を使う。
        next_speed_refresh = 0.0
        # 速度指令が届かなかった時に「最初から止まっていた=ストッパー」と
        # 誤認しないため、まず実測角度が動いたことを必ず確認する。
        # 動いた後だけ、stillness_sec間の停止をストッパーとして扱う。
        motion_confirmed = False
        motion_started_position: float | None = None
        window_started_at: float | None = None
        window_started_position: float | None = None
        latest_position: float | None = None
        feedback_samples = 0

        print(f"{name}原点合わせ: ストッパーへ {speed_percent:.1f}% で動かします")
        try:
            while time.monotonic() < deadline:
                now = time.monotonic()
                # PIDを使わず、指定した1台だけを直接低速でストッパーへ動かす。
                if now >= next_speed_refresh:
                    servo.motor.set_velocity(speed, force=True)
                    next_speed_refresh = now + 0.25
                self.reader.request(name)
                # 成功しているangle_monitor.pyと同じ15ms待機で受信する。
                time.sleep(0.015)

                now = time.monotonic()
                received_position = False
                for feedback in self.reader.poll(now):
                    if feedback.name == name and feedback.position_rad is not None:
                        latest_position = feedback.position_rad
                        received_position = True
                        feedback_samples += 1

                # 新しいmechPos応答が無い周期を「停止」と誤認しない。
                if received_position and latest_position is not None:
                    if motion_started_position is None:
                        motion_started_position = latest_position
                    elif not motion_confirmed:
                        # mechPosが0/2πをまたいでも、最短角度差で実際の動きを判定する。
                        moved_deg = abs(_phase_delta_degrees(latest_position - motion_started_position))
                        if moved_deg >= max(1.0, stillness_deg * 2.0):
                            motion_confirmed = True
                            window_started_at = now
                            window_started_position = latest_position
                            print(f"{name}原点合わせ: 角度変化を確認しました。ストッパーを待ちます")
                    elif window_started_at is None or window_started_position is None:
                        window_started_at = now
                        window_started_position = latest_position
                    elif now - window_started_at >= stillness_sec:
                        moved_deg = abs(_phase_delta_degrees(latest_position - window_started_position))
                        if moved_deg <= stillness_deg:
                            servo.set_home_radians(latest_position)
                            print(f"{name}原点合わせ完了: ストッパー位置を0度に登録しました")
                            return
                        # この0.1秒では十分に動いた。新しい区間として測り直す。
                        window_started_at = now
                        window_started_position = latest_position
                # 次のmechPos要求を詰め込みすぎない。
                time.sleep(0.015)
        finally:
            # 成功・失敗のどちらでも、ストッパーへ押し続けない。
            servo.motor.stop()

        if feedback_samples == 0:
            raise TimeoutError(
                f"{name}原点合わせ失敗: mechPos応答を1回も受信できませんでした。"
                "CAN通信・モーター電源を確認してください"
            )
        if not motion_confirmed:
            raise TimeoutError(
                f"{name}原点合わせ失敗: 速度指令後もmechPosが変化しませんでした。"
                "モーター出力・方向・機構の固着を確認してください"
            )
        raise TimeoutError(
            f"{name}原点合わせ失敗: {timeout_sec:.1f}秒間mechPosが動き続け、"
            "ストッパーを検出できませんでした。方向またはストッパー位置を確認してください"
        )

    def update(self) -> None:
        """エンコーダーを要求・受信し、両方のPID保持を1回更新する。50Hzで呼ぶ。"""
        with self._lock:
            def apply_feedbacks(now: float) -> None:
                """到着済みの正式mechPosを該当サーボへ渡す。"""
                for feedback in self.reader.poll(now):
                    if feedback.name == "catch":
                        if feedback.position_rad is not None:
                            self._latest_mechpos_feedback["catch"] = feedback
                        self.catch.update_feedback(feedback)
                    elif feedback.name == "lift":
                        if feedback.position_rad is not None:
                            self._latest_mechpos_feedback["lift"] = feedback
                        self.lift.update_feedback(feedback)

            now = time.monotonic()
            # 前周期までに到着した応答を先に処理する。
            apply_feedbacks(now)
            # angle_monitor.pyで実機確認済みの手順と同じく、1台へ要求してから
            # 少し待ち、その応答をこの周期内で読む。以前は要求直後に次周期へ
            # 進んでおり、USB-AT変換器が応答を落とすとcatchが安全停止していた。
            self.reader.request_next()
            time.sleep(hensuu.encoder_response_wait_sec)
            now = time.monotonic()
            apply_feedbacks(now)
            # 応答が消えた時、古い速度を出し続けないための安全停止。
            if self.catch.watchdog(now):
                print(
                    "catch PID安全停止: "
                    f"{self.catch.config.feedback_timeout_sec:.2f}秒間mechPos応答がありません"
                )
            if self.lift.watchdog(now):
                print(
                    "lift PID安全停止: "
                    f"{self.lift.config.feedback_timeout_sec:.2f}秒間mechPos応答がありません"
                )

    def raw_mechpos_report(self) -> str:
        """最新のmechPos応答をangle_monitor.py形式で文字列化する。

        読取り要求は追加せず、PIDスレッドが受信済みのデータだけを使う。
        「モーター位置」はfloat32の生4バイトをlittle-endian uint32として
        表した10進数であり、通信確認用の値である。
        """
        with self._lock:
            feedbacks = dict(self._latest_mechpos_feedback)

        lines = ["[mechPos 生データ]"]
        for name in ("catch", "lift"):
            feedback = feedbacks.get(name)
            frame = b"" if feedback is None else feedback.raw_at_frame
            if len(frame) < 15:
                lines.append(f"{name}: mechPos応答待ち")
                continue
            position_bytes = frame[11:15]
            position_raw = int.from_bytes(position_bytes, "little")
            lines.append(
                f"{name}: モーター位置（変換なし・10進数）= {position_raw} "
                f"/ mechPosバイト={' '.join(str(value) for value in position_bytes)}"
            )
        return "\n".join(lines)

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
        last_reported_error: str | None = None
        while not self._stop_event.is_set():
            started = time.monotonic()
            try:
                self.update()
                if self._pid_error is not None:
                    print("サーボPID通信が復帰しました")
                self._pid_error = None
                last_reported_error = None
            except Exception as error:
                # 以前は、ここで1回でもpyserialの例外が起きるとPIDスレッドが
                # 終了し、以後catch/liftが動かなくなっていた。出力は安全に止め、
                # 次周期以降に角度通信の復帰を試す。
                self._pid_error = f"{type(error).__name__}: {error}"
                if self._pid_error != last_reported_error:
                    print(f"サーボPID通信エラー: {self._pid_error}")
                    print("角度通信が復帰するまでcatch/liftの出力を停止します")
                    last_reported_error = self._pid_error
                for servo in (self.catch, self.lift):
                    servo.last_command = 0.0
                    try:
                        # release()にはせず、通信が戻れば同じ目標角度のPIDを再開する。
                        servo.motor.stop()
                    except Exception:
                        pass

            # 例外時も最低0.1秒だけ待つ。通信断中に送信を連打しない。
            elapsed = time.monotonic() - started
            wait = max(0.0, interval - elapsed)
            if self._pid_error is not None:
                wait = max(wait, 0.1)
            self._stop_event.wait(wait)

    def pid_error(self) -> str | None:
        """PID通信スレッドの直近エラー。正常動作中はNone。"""
        return self._pid_error

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
        # 機構の速度形式は、実機で調整済みの従来Type 18形式を維持する。
        # 足回りで動いたType 1形式を、未検証のliftへ流用しない。
        catch=EncoderPositionServo(
            ATMotor(
                transport,
                catch_address,
                zero_hold_band=0.0,
            ),
            _config("catch"),
        ),
        lift=EncoderPositionServo(
            ATMotor(
                transport,
                lift_address,
                zero_hold_band=0.0,
            ),
            _config("lift"),
        ),
        reader=reader,
        transport=transport,
        owns_transport=owns_transport,
    )


def _phase_delta_degrees(delta_rad: float) -> float:
    """2πの境界をまたいでも正しい、最短の角度差[deg]を返す。"""
    return degrees(atan2(sin(delta_rad), cos(delta_rad)))
