from rox_mecanum.can_motor import (
    build_active_report_command,
    build_disable_command,
    build_enable_command,
    build_operation_control_command,
)


def test_enable_and_disable_are_extended_private_frames():
    enable = build_enable_command(5, host_id=0)
    disable = build_disable_command(5, host_id=0)

    assert enable.arbitration_id == (3 << 24) | 5
    assert enable.data == bytes(8)
    assert disable.arbitration_id == (4 << 24) | 5


def test_operation_frame_encodes_velocity_in_second_uint16():
    command = build_operation_control_command(6, velocity_rad_per_sec=50.0, host_id=0x12)

    assert command.arbitration_id == (1 << 24) | (0x12 << 8) | 6
    assert command.data[2:4] == b"\xff\xff"
    assert len(command.data) == 8


def test_active_report_enable_frame():
    command = build_active_report_command(5, True)

    assert command.arbitration_id == (24 << 24) | 5
    assert command.data[0] == 1
