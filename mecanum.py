#!/usr/bin/env python3
"""メカナムだけをDualSenseで手動操縦する入口。

実行: ``python3 mecanum.py``

lift・catch・ソレノイド・カメラは起動しない。
実際の処理は ``experiments/mecanum_manual.py`` にまとめてあり、GAMEでも使う
メカナムライブラリと同じ送信方式・速度設定を使う。
"""

from experiments.mecanum_manual import main


if __name__ == "__main__":
    main()
