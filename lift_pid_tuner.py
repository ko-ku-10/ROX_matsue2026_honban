"""DualSenseだけでliftのPIDを調整するプログラム。"""

from __future__ import annotations

import time

from rox_mecanum import Button, PygameDualSense
from servos import open_servos


# 十字キー上下で選択する調整項目: 名前、変更幅、表示単位
ITEMS = (
    ("kp", 0.001, ""),
    ("ki", 0.0001, ""),
    ("kd", 0.0001, ""),
    ("max_speed_percent", 1.0, "%"),
    ("tolerance_deg", 0.5, "°"),
)


def print_status(servos, selected: int) -> None:
    config = servos.lift.config
    values = {
        "kp": config.kp,
        "ki": config.ki,
        "kd": config.kd,
        "max_speed_percent": config.max_speed * 100.0,
        "tolerance_deg": config.tolerance_deg,
    }
    name, _, unit = ITEMS[selected]
    print(
        f"選択={name} ({values[name]:.4f}{unit}) | "
        f"P={config.kp:.4f} I={config.ki:.4f} D={config.kd:.4f} "
        f"速度={config.max_speed * 100:.1f}% 停止範囲=±{config.tolerance_deg:.1f}° | "
        f"角度={servos.lift.read()} 目標={servos.lift.target_angle:.2f} "
        f"PID={'ON' if servos.lift.pid_enabled else 'OFF'}"
    )


servos = open_servos()
controller = PygameDualSense.open()
selected = 0
previous = {button: False for button in Button}

try:
    servos.attach()
    # 現在の物理位置をliftの目標位置にする。catchはPIDを解除する。
    servos.home_from_feedback()
    servos.catch.pid_off()
    servos.start_pid()

    print("lift PID調整モード")
    print("十字キー上下: 項目選択 / 十字キー左右: 値を増減")
    print("L1を押しながら十字キー左右: 10倍ずつ増減")
    print("○: 今の位置でlift保持 / ×: lift PID OFF / OPTIONS: 非常停止終了")
    print_status(servos, selected)

    while True:
        state = controller.read()

        if state.button(Button.OPTIONS):
            print("OPTIONS: 非常停止")
            servos.lift.pid_off()
            break

        pressed = {button: state.button(button) for button in Button}
        just_pressed = {button: pressed[button] and not previous[button] for button in Button}

        if just_pressed[Button.DPAD_UP]:
            selected = (selected - 1) % len(ITEMS)
            print_status(servos, selected)
        elif just_pressed[Button.DPAD_DOWN]:
            selected = (selected + 1) % len(ITEMS)
            print_status(servos, selected)
        elif just_pressed[Button.DPAD_LEFT] or just_pressed[Button.DPAD_RIGHT]:
            name, step, unit = ITEMS[selected]
            direction = 1.0 if just_pressed[Button.DPAD_RIGHT] else -1.0
            if pressed[Button.L1]:
                step *= 10.0
            config = servos.lift.config
            current = config.max_speed * 100.0 if name == "max_speed_percent" else getattr(config, name)
            value = max(0.0, current + direction * step)
            # max速度だけは0%にできないため、最低0.1%にする。
            if name == "max_speed_percent":
                value = max(0.1, min(100.0, value))
            servos.set_pid("lift", **{name: value})
            print_status(servos, selected)
        elif just_pressed[Button.CIRCLE]:
            servos.lift.hold_current()
            print("lift: 現在位置でPID保持")
            print_status(servos, selected)
        elif just_pressed[Button.CROSS]:
            servos.lift.pid_off()
            print("lift: PID OFF")
            print_status(servos, selected)

        previous = pressed
        time.sleep(0.02)
finally:
    servos.close()
    controller.close()
