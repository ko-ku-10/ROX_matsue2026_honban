"""本番用統合プログラム: メカナム、catch/lift PIDサーボ、ソレノイド、状態Web表示。"""

from __future__ import annotations

import json
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import hensuu
from gpiozero import LED
from rox_mecanum import (
    ATEncoderReader,
    AT_NEUTRAL_VALUE,
    Button,
    DualSenseMotionMapping,
    MecanumMixer,
    MecanumRobot,
    PySerialTransport,
    PygameDualSense,
    at_address_from_can_id,
)
from servos import open_servos


def _speed_span(percent: float) -> int:
    return int(round(AT_NEUTRAL_VALUE * max(0.0, min(100.0, float(percent))) / 100.0))


class RobotStatus:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._value: dict[str, object] = {"running": True, "message": "起動中"}

    def set(self, **values: object) -> None:
        with self._lock:
            self._value.update(values)

    def snapshot(self) -> dict[str, object]:
        with self._lock:
            return dict(self._value)


def _start_dashboard(status: RobotStatus, port: int) -> ThreadingHTTPServer:
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self) -> None:  # noqa: N802
            if self.path == "/api/status":
                body = json.dumps(status.snapshot(), ensure_ascii=False).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            body = _DASHBOARD_HTML.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *_: object) -> None:
            pass

    server = ThreadingHTTPServer(("0.0.0.0", int(port)), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    return server


def _wait_for_home(reader: ATEncoderReader, addresses: dict[str, int]) -> dict[str, int]:
    deadline = time.monotonic() + 5.0
    values: dict[str, int] = {}
    while time.monotonic() < deadline and set(values) != set(addresses):
        reader.request_all()
        time.sleep(0.02)
        for feedback in reader.poll():
            values[feedback.name] = feedback.count
    missing = set(addresses) - set(values)
    if missing:
        raise TimeoutError(f"エンコーダー応答なし: {', '.join(sorted(missing))}")
    return values


def main() -> None:
    status = RobotStatus()
    server = _start_dashboard(status, hensuu.dashboard_port)
    host = socket.gethostbyname(socket.gethostname())
    print(f"状態表示: http://{host}:{hensuu.dashboard_port}")
    print("操作: 左スティック=移動 / R2+右スティック=旋回 / L2=ソレノイド")
    print("○=catch最大 / ×=catch原点 / △=lift最大 / □=lift原点 / OPTIONS=非常停止")

    controller = None
    transport = None
    solenoid = None
    mecanum = None
    servos = None
    try:
        controller = PygameDualSense.open()
        transport = PySerialTransport.open(hensuu.serial_port, hensuu.serial_baud, minimum_interval=0.0008)
        solenoid = LED(hensuu.solenoid_pin)
        mecanum = MecanumRobot(
            transport,
            motor_ids={"FL": 0x0C, "FR": 0x14, "RL": 0x1C, "RR": 0x24},
            motor_directions={"FL": 1.0, "FR": -1.0, "RL": 1.0, "RR": -1.0},
            mixer=MecanumMixer(rotation_gain=0.22),
            speed_span=_speed_span(hensuu.mecanum_speed_percent),
        )
        catch_address = at_address_from_can_id(hensuu.catch_can_id)
        lift_address = at_address_from_can_id(hensuu.lift_can_id)
        addresses = {"FL": 0x0C, "FR": 0x14, "RL": 0x1C, "RR": 0x24, "catch": catch_address, "lift": lift_address}
        reader = ATEncoderReader(transport, addresses)
        servos = open_servos(transport=transport, reader=reader)
        catch, lift = servos.catch, servos.lift
        mecanum.enable_all(retries=3, interval=0.05)
        servos.attach()

        input("catch/liftを機械的な0度へ合わせてから Enter: ")
        homes = _wait_for_home(reader, {"catch": catch_address, "lift": lift_address})
        servos.home(homes["catch"], homes["lift"])
        mapping = DualSenseMotionMapping(deadzone=0.08, rotation_enable=Button.R2 if hensuu.mecanum_rotation_requires_r2 else None)
        control_interval = 1.0 / 50.0
        next_query_at = 0.0
        next_drive_at = 0.0
        solenoid_until = 0.0
        latest_counts: dict[str, int] = homes
        status.set(message="運転中", dashboard_url=f"http://{host}:{hensuu.dashboard_port}")

        while True:
            started = time.monotonic()
            state = controller.read()
            if state.button(Button.OPTIONS):
                # 終了処理を待たず、この周期ですぐ全機構を安全側へ止める。
                status.set(message="非常停止")
                mecanum.stop()
                servos.release()
                solenoid.off()
                solenoid_until = 0.0
                print("OPTIONS: 非常停止")
                break
            # メカナムは実績のある20Hz、位置保持は下の50Hzで回す。
            if started >= next_drive_at:
                mecanum.drive(mapping.command(state))
                next_drive_at = started + 1.0 / 20.0

            if state.was_pressed(Button.CIRCLE):
                catch.write(hensuu.catch_max_angle)
            if state.was_pressed(Button.CROSS):
                catch.write(0.0)
            if state.was_pressed(Button.TRIANGLE):
                lift.write(hensuu.lift_max_angle)
            if state.was_pressed(Button.SQUARE):
                lift.write(0.0)
            if state.was_pressed(Button.L2):
                solenoid.on()
                solenoid_until = started + hensuu.solenoid_time_sec
            if solenoid_until and started >= solenoid_until:
                solenoid.off()
                solenoid_until = 0.0

            if started >= next_query_at:
                reader.request_all()
                next_query_at = started + 1.0 / hensuu.encoder_poll_hz
            for feedback in reader.poll(started):
                latest_counts[feedback.name] = feedback.count
                if feedback.name == "catch":
                    catch.update(feedback.count, started)
                elif feedback.name == "lift":
                    lift.update(feedback.count, started)

            # エンコーダー通信が途切れたら、最後の補正速度を残さず停止する。
            if catch.last_feedback_at is not None and started - catch.last_feedback_at > 0.20:
                catch.stop()
            if lift.last_feedback_at is not None and started - lift.last_feedback_at > 0.20:
                lift.stop()

            status.set(
                encoder_counts=latest_counts,
                catch=catch.status(),
                lift=lift.status(),
                solenoid=bool(solenoid_until),
                active_buttons=[button.value for button in state.active_buttons],
            )
            remaining = control_interval - (time.monotonic() - started)
            if remaining > 0.0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        status.set(message=f"エラー: {error}")
        raise
    finally:
        status.set(running=False, message="停止")
        if mecanum is not None:
            mecanum.stop()
        if servos is not None:
            servos.release()
        if solenoid is not None:
            solenoid.off()
        if transport is not None:
            transport.close()
        if controller is not None:
            controller.close()
        server.shutdown()


_DASHBOARD_HTML = """<!doctype html><meta charset=utf-8><title>ROX Robot</title>
<style>body{font-family:sans-serif;background:#10151f;color:#eaf0ff;margin:2rem}pre{background:#1b2535;padding:1rem;border-radius:8px;font-size:16px}</style>
<h1>ROX Robot 状態</h1><pre id=s>読み込み中...</pre><script>
async function refresh(){const r=await fetch('/api/status');document.querySelector('#s').textContent=JSON.stringify(await r.json(),null,2)}refresh();setInterval(refresh,200);
</script>"""


if __name__ == "__main__":
    main()
