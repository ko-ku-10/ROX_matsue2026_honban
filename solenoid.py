"""ソレノイドの動作を書くファイル。

GAME2、GAME3、experiments/solenoid_test.py は全てこの fire() を使う。
"""

# ===== ソレノイドの動きをここに書く =====
# ONにしている時間[秒]。短すぎる時は 0.4 などへ変える。
on_time_sec = 0.3


def fire(output: object) -> None:
    """ソレノイドを一回だけ発射する。"""
    output.pulse(on_time_sec)
