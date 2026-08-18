"""DualSense でメカナム機体を操作する実行プログラム。

調整値はすべて hensuu.py にまとめている。

操作:
  左スティック 上下: 前進 / 後退
  左スティック 左右: 左右平行移動
  右スティック 左右: 旋回（既定では R2 を押している間だけ）
  OPTIONS: 安全停止して終了

L2 はこのプログラムでは一切使わない。他の操作用に自由に使える。
"""

from __future__ import annotations

import time

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


def _speed_span(percent: float) -> int:
    """hensuu.py の百分率をAT速度範囲へ変換する。"""
    limited_percent = max(0.0, min(100.0, float(percent)))
    return int(round(AT_NEUTRAL_VALUE * limited_percent / 100.0))


def _control_interval(hz: float) -> float:
    if hz <= 0.0:
        raise ValueError("hensuu.mecanum_control_hz は 0 より大きくしてください")
    return 1.0 / float(hz)


def main() -> None:
    """コントローラー入力を読み、メカナム4輪へ送信する。"""
    controller = None
    transport = None
    robot = None

    try:
        print("=" * 50)
        print("  メカナムホイール 4WD を起動します")
        print("=" * 50)
        print(f"  速度上限: {float(hensuu.mecanum_speed_percent):.0f}%")
        print("  左スティック: 前後・左右平行移動")
        if hensuu.mecanum_rotation_requires_r2:
            print("  R2 + 右スティック左右: 旋回")
        else:
            print("  右スティック左右: 旋回")
        print("  OPTIONS: 停止して終了")

        controller = PygameDualSense.open()
        transport = PySerialTransport.open(
            hensuu.mecanum_serial_port,
            baudrate=hensuu.mecanum_serial_baud,
            minimum_interval=hensuu.mecanum_serial_write_interval_sec,
        )
        mixer = MecanumMixer(
            rotation_gain=(
                float(hensuu.mecanum_wheel_base_half_l)
                + float(hensuu.mecanum_wheel_base_half_w)
            )
        )
        robot = MecanumRobot(
            transport,
            motor_ids=hensuu.mecanum_motor_ids,
            motor_directions=hensuu.mecanum_motor_directions,
            mixer=mixer,
            speed_span=_speed_span(hensuu.mecanum_speed_percent),
        )
        mapping = DualSenseMotionMapping(
            deadzone=float(hensuu.mecanum_deadzone),
            translation_enable=None,  # L2を移動条件にしない
            rotation_enable=Button.R2 if hensuu.mecanum_rotation_requires_r2 else None,
            translation_gain=float(hensuu.mecanum_translation_gain),
            rotation_gain=float(hensuu.mecanum_rotation_gain),
            response_exponent=float(hensuu.mecanum_response_exponent),
        )

        robot.enable_all(
            retries=int(hensuu.mecanum_enable_retries),
            interval=float(hensuu.mecanum_enable_interval_sec),
        )
        print("モーターを有効化しました。操作を開始できます。")

        interval = _control_interval(hensuu.mecanum_control_hz)
        startup_stop_until = time.monotonic() + max(0.0, float(hensuu.mecanum_startup_stop_sec))
        while True:
            loop_started = time.monotonic()
            state = controller.read()

            if state.button(Button.OPTIONS):
                print("OPTIONS が押されたため停止します。")
                break

            if time.monotonic() < startup_stop_until:
                robot.stop()
            else:
                robot.drive(mapping.command(state))

            remaining = interval - (time.monotonic() - loop_started)
            if remaining > 0.0:
                time.sleep(remaining)

    except KeyboardInterrupt:
        print("Ctrl+C を受け取りました。停止します。")
    finally:
        # どの終了経路でも停止指令を送ってから接続を閉じる。
        if robot is not None:
            try:
                robot.stop()
            except Exception as error:
                print(f"停止指令の送信に失敗しました: {error}")
        if transport is not None:
            transport.close()
        if controller is not None:
            controller.close()
        print("メカナムを停止しました。")


if __name__ == "__main__":
    main()
