"""本番用統合プログラム: メカナム、catch/lift PIDサーボ、ソレノイド、状態Web表示。"""

from __future__ import annotations

import json
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import hensuu
from rox_mecanum import (
    AT_NEUTRAL_VALUE,
    Button,
    DualSenseMotionMapping,
    MecanumMixer,
    MecanumRobot,
    PySerialTransport,
    PygameDualSense,
)
from rox_mecanum.solenoid import RDKSolenoid
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


def main() -> None:
    status = RobotStatus()
    server = _start_dashboard(status, hensuu.dashboard_port)
    host = socket.gethostbyname(socket.gethostname())
    print(f"状態表示: http://{host}:{hensuu.dashboard_port}")
    print("操作: 左スティック=移動 / R2+右スティック=旋回 / L2=ソレノイド")
    print("CREATE=lift上下切替 / △=持ち上げ動作 / ○=catch閉 / ×=catch開 / OPTIONS=非常停止")

    controller = None
    transport = None
    solenoid = None
    mecanum = None
    servos = None
    try:
        controller = PygameDualSense.open()
        transport = PySerialTransport.open(hensuu.serial_port, hensuu.serial_baud, minimum_interval=0.0008)
        solenoid = RDKSolenoid(hensuu.solenoid_pin)
        mecanum = MecanumRobot(
            transport,
            motor_ids={"FL": 0x0C, "FR": 0x14, "RL": 0x1C, "RR": 0x24},
            motor_directions={"FL": 1.0, "FR": -1.0, "RL": 1.0, "RR": -1.0},
            mixer=MecanumMixer(rotation_gain=0.22),
            speed_span=_speed_span(hensuu.mecanum_speed_percent),
        )
        # サーボは専用の角度読取りを使う。メカナムと通信ポートは共有する。
        servos = open_servos(transport=transport)
        catch, lift = servos.catch, servos.lift
        mecanum.enable_all(retries=3, interval=0.05)
        servos.attach()

        input("catch/liftを機械的な0度へ合わせてから Enter: ")
        servos.home_from_feedback()
        servos.start_pid()
        mapping = DualSenseMotionMapping(deadzone=0.08, rotation_enable=Button.R2 if hensuu.mecanum_rotation_requires_r2 else None)
        control_interval = 1.0 / 50.0
        next_drive_at = 0.0
        solenoid_until = 0.0
        lift_is_down = False
        action = "idle"
        action_started = 0.0
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
            if state.was_pressed(Button.CREATE) and action == "idle":
                # motiage.pyの「上下切替」。現在の安全範囲内で動かす。
                if lift_is_down:
                    lift.write(0.0)
                    lift_is_down = False
                    print("lift: 原点へ")
                else:
                    lift.write(hensuu.lift_min_angle)
                    lift_is_down = True
                    print("lift: 下へ")
            if state.was_pressed(Button.SQUARE) and action == "idle":
                lift.write(0.0)
                lift_is_down = False
            if state.was_pressed(Button.TRIANGLE) and action == "idle":
                # motiage.pyの「掴む」順番を、sleepで止めずに実行する。
                # 各段階は実測角度が目標に着くまで待つ。
                lift.write(hensuu.lift_min_angle)
                lift_is_down = True
                action = "lower"
                action_started = started
                print("持ち上げ動作: liftを下げます")
            if state.was_pressed(Button.L2):
                solenoid.on()
                solenoid_until = started + hensuu.solenoid_time_sec
            if solenoid_until and started >= solenoid_until:
                solenoid.off()
                solenoid_until = 0.0

            # 非ブロッキングの持ち上げシーケンス。OPTIONSは常に即座に受け付ける。
            if action == "lower" and lift.is_at_target():
                catch.write(hensuu.catch_max_angle)
                action = "close"
                action_started = started
                print("持ち上げ動作: catchを閉じます")
            elif action == "close" and catch.is_at_target():
                lift.write(0.0)
                lift_is_down = False
                action = "raise"
                action_started = started
                print("持ち上げ動作: liftを上げます")
            elif action == "raise" and lift.is_at_target():
                catch.write(0.0)
                action = "open"
                action_started = started
                print("持ち上げ動作: catchを開きます")
            elif action == "open" and catch.is_at_target():
                action = "idle"
                print("持ち上げ動作: 完了")
            elif action != "idle" and started - action_started > 15.0:
                # 物理的に動けない時に永遠に待たない。PIDも解除して安全停止する。
                print("持ち上げ動作: 15秒で時間切れ。サーボを停止します")
                servos.release()
                action = "idle"

            status.set(
                catch=catch.status(),
                lift=lift.status(),
                solenoid=bool(solenoid_until),
                mechanism_action=action,
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
            solenoid.close()
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
