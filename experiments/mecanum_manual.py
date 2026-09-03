"""DualSense でメカナム機体を操作する実行プログラム。

調整値はすべて hensuu.py にまとめている。

操作:
  左スティック 上下: 前進 / 後退
  左スティック 左右: 左右平行移動
  右スティック 左右: 旋回
  L1を押している間: 低速モード
  OPTIONS: 安全停止して終了
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
    open_configured_dualsense,
)


def _speed_span(percent: float) -> int:
    """hensuu.py の百分率をAT速度範囲へ変換する。"""
    limited_percent = max(0.0, min(100.0, float(percent)))
    return int(round(AT_NEUTRAL_VALUE * limited_percent / 100.0))


def _control_interval(hz: float) -> float:
    if hz <= 0.0:
        raise ValueError("制御周期は 0 より大きくしてください")
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
        print(f"  L1を押している間: 低速モード（通常の{float(hensuu.mecanum_slow_mode_percent):.0f}%）")
        print("  OPTIONS: 停止して終了")

        controller = open_configured_dualsense()
        transport = PySerialTransport.open(
            hensuu.serial_port,
            baudrate=hensuu.serial_baud,
            minimum_interval=0.0008,
        )
        mixer = MecanumMixer(rotation_gain=hensuu.mecanum_rotation_speed_percent / 100.0)
        robot = MecanumRobot(
            transport,
            motor_ids={"FL": 0x0C, "FR": 0x14, "RL": 0x1C, "RR": 0x24},
            motor_directions={"FL": 1.0, "FR": -1.0, "RL": 1.0, "RR": -1.0},
            mixer=mixer,
            speed_span=_speed_span(hensuu.mecanum_speed_percent),
            acceleration_per_second=hensuu.mecanum_acceleration_percent_per_sec / 100.0,
            deceleration_per_second=hensuu.mecanum_deceleration_percent_per_sec / 100.0,
            command_minimum_interval=hensuu.mecanum_command_minimum_interval_sec,
            command_force_delta=hensuu.mecanum_command_force_delta,
            command_value_hysteresis_counts=hensuu.mecanum_command_hysteresis_counts,
            command_reverse_guard_counts=hensuu.mecanum_command_reverse_guard_counts,
        )
        mapping = DualSenseMotionMapping(
            deadzone=hensuu.mecanum_deadzone,
            translation_enable=None,  # L1を移動条件にしない
            rotation_enable=Button.R2 if hensuu.mecanum_rotation_requires_r2 else None,
            translation_gain=1.0,
            rotation_gain=1.0,
            response_exponent=hensuu.mecanum_response_exponent,
            invert_forward=hensuu.mecanum_invert_forward_input,
            strafe_gain=hensuu.mecanum_strafe_speed_percent / 100.0,
            slow_mode_button=Button.L1,
            slow_mode_gain=hensuu.mecanum_slow_mode_percent / 100.0,
        )

        robot.enable_all(
            retries=3,
            interval=0.05,
        )
        print("モーターを有効化しました。操作を開始できます。")

        interval = _control_interval(20)
        startup_stop_until = time.monotonic() + 0.8
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
