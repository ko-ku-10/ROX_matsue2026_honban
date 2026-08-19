from rox_mecanum.can_motor import (
    build_active_report_command,
    build_disable_command,
    build_enable_command,
    build_get_device_id_command,
    build_operation_control_command,
)


def test_enable_and_disable_are_extended_private_frames():
    enable = build_enable_command(5, host_id=0xFF)
    disable = build_disable_command(5, host_id=0xFF)

    assert enable.arbitration_id == (3 << 24) | (0xFF << 8) | 5
    assert enable.data == bytes(8)
    assert disable.arbitration_id == (4 << 24) | (0xFF << 8) | 5


def test_operation_frame_encodes_velocity_in_second_uint16():
    command = build_operation_control_command(6, velocity_rad_per_sec=50.0)

    # タイプ1の中央16ビットはトルク。0Nmは0x7FFF。
    assert command.arbitration_id == (1 << 24) | (0x7FFF << 8) | 6
    assert command.data[2:4] == b"\xff\xff"
    assert len(command.data) == 8


def test_active_report_enable_frame():
    command = build_active_report_command(5, True, host_id=0xFF)

    assert command.arbitration_id == (24 << 24) | (0xFF << 8) | 5
    assert command.data[0] == 1


def test_get_device_id_is_non_actuating_type_zero_frame():
    command = build_get_device_id_command(5, host_id=0xFF)

    assert command.arbitration_id == (0xFF << 8) | 5
    assert command.data == bytes(8)
