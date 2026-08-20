"""catch/liftのPIDを実機上で調整するための専用プログラム。"""

from __future__ import annotations

import queue
import threading
import time

from rox_mecanum import Button, PygameDualSense
from servos import open_servos


def input_loop(commands: queue.Queue[str]) -> None:
    while True:
        try:
            commands.put(input("PID> ").strip())
        except EOFError:
            commands.put("quit")
            return


def show(servos) -> None:
    for name in ("catch", "lift"):
        servo = getattr(servos, name)
        config = servo.config
        print(
            f"{name}: P={config.kp:.5f} I={config.ki:.5f} D={config.kd:.5f} "
            f"max={config.max_speed * 100:.1f}% deadband={config.tolerance_deg:.1f}° "
            f"現在={servo.read()}° 目標={servo.target_angle:.2f}° PID={'ON' if servo.pid_enabled else 'OFF'}"
        )


def apply_command(servos, command: str) -> bool:
    """終了する場合だけTrueを返す。"""
    parts = command.lower().split()
    if not parts:
        return False
    if parts[0] in {"quit", "exit", "q"}:
        return True
    if parts[0] in {"show", "s"}:
        show(servos)
        return False
    if len(parts) == 2 and parts[0] in {"catch", "lift", "both"} and parts[1] in {"hold", "on", "off"}:
        names = ("catch", "lift") if parts[0] == "both" else (parts[0],)
        for name in names:
            servo = getattr(servos, name)
            if parts[1] == "off":
                servo.pid_off()
            else:
                servo.hold_current()
        show(servos)
        return False
    if len(parts) == 3 and parts[0] in {"catch", "lift", "both"}:
        try:
            value = float(parts[2])
        except ValueError:
            print("数値を入力してください")
            return False
        field = parts[1]
        settings = {
            "kp": {"kp": value},
            "ki": {"ki": value},
            "kd": {"kd": value},
            "max": {"max_speed_percent": value},
            "deadband": {"tolerance_deg": value},
        }
        if field not in settings:
            print("変更できる項目: kp / ki / kd / max / deadband")
            return False
        names = ("catch", "lift") if parts[0] == "both" else (parts[0],)
        try:
            for name in names:
                servos.set_pid(name, **settings[field])
        except ValueError as error:
            print(f"設定エラー: {error}")
            return False
        show(servos)
        return False
    print("例: show / catch kp 0.003 / both max 5 / lift deadband 3 / catch hold / lift off / quit")
    return False


servos = open_servos()
controller = PygameDualSense.open()
commands: queue.Queue[str] = queue.Queue()

try:
    servos.attach()
    servos.home_from_feedback()  # 現在位置を目標として、動かさず保持を開始する。
    servos.start_pid()
    threading.Thread(target=input_loop, args=(commands,), daemon=True).start()
    print("PID調整モード: OPTIONSで非常停止して終了")
    print("例: show / catch kp 0.003 / both max 5 / lift deadband 3")

    while True:
        if controller.read().button(Button.OPTIONS):
            print("OPTIONS: 非常停止")
            break
        try:
            command = commands.get_nowait()
        except queue.Empty:
            command = ""
        if command and apply_command(servos, command):
            break
        time.sleep(0.02)
finally:
    servos.close()
    controller.close()
