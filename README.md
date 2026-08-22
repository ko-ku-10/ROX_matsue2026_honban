# ROX2026 ロボットプログラム

このリポジトリは、ROX2026用のメカナムロボットをPythonで操作するためのコードです。ゲームごとの実行プログラムは分け、共通の難しい処理は `rox_mecanum` ライブラリへまとめています。

## 最初に使うプログラム

同じ `/dev/ttyUSB0` を使うため、次のプログラムは**同時に1つだけ**起動します。

| 目的 | 実行コマンド |
|---|---|
| GAME1 | `python3 game1.py` |
| GAME2 | `python3 game2.py` |
| GAME2のパネル選択だけ確認 | `python3 game2_target_sim.py` |
| カメラ・Tagだけ確認 | `python3 maintenance.py --camera-only` |
| 単眼カメラの距離校正 | `python3 calibrate_camera.py 1.00` |
| 魚眼カメラの歪み校正 | `python3 calibrate_fisheye.py` |
| カメラ・Tag・モーターを部分確認 | `python3 maintenance.py` |
| 従来の手動統合操作 | `python3 run_all.py` |

初回の必要パッケージです。

```bash
python3 -m pip install --user '.[hardware,vision]'
```

## 共通の操作

| 操作 | 機能 |
|---|---|
| タッチパッド押し込み | 完全手動モード / 自動モード切替 |
| 左スティック | 前後・左右への平行移動 |
| R2 + 右スティック左右 | 旋回 |
| OPTIONS | 全停止・終了 |

起動直後は完全手動モードです。自動モード中でもスティック操作を足せます。タッチパッドでモードを切り替えた時は自動動作を停止し、勝手に再開しません。

## GAME1の操作

| ボタン | 動作 |
|---|---|
| CREATE | lift/catchを開始姿勢へ展開して固定 |
| △ | Tag 1または9を基準に、Tag 8へ自動移動・中心合わせ |
| ○ | Tag 8中心合わせ後にトンネルを通過 |
| □ | Tag 12・13の中間へ自動停止 |
| R1 | 低速で板を押し込み |
| × | 板へ上がれたことを操縦者が確定 |
| L2 | Tag 6・10の間を通ってTag 0へ帰還 |

GAME1ではCREATEで展開した後、lift/catchを動かしません。`game1_hensuu.py` に距離・時間・開始角度を入力してから自動走行を使います。

## GAME2の操作

| ボタン | 動作 |
|---|---|
| CREATE | パネル読取、前進、横スライド照準を実行 |
| △ | 照準成功後にliftを発射高さへ上げる |
| L2 | 発射 |
| × | liftを下げ、後退して補給位置へ戻る |

パネルはTag 14〜22で判定します。中央段、上段、下段の順に選び、同じ段に2枚以上あれば中間を狙います。調整値は `game2_hensuu.py` にあります。

## カメラとAprilTag

カメラ設定は `camera_hensuu.py` にあります。

- Tag種類: `tag16h5`
- Tagサイズ: 180 mm (`apriltag_size_m = 0.180`)
- `camera_device`: USB/V4L2カメラを使う場合の番号
- `mipi_pipe_id` と `mipi_host_index`: RDK MIPIカメラ用。既定値のまま使う
- `fisheye_enabled`: 魚眼校正後だけ `True` にする
- `camera_focal_length_px`: 校正後の焦点距離。`0.0`の間は距離を使う自動移動を完了しません。

まず `maintenance.py --camera-only` を起動し、ブラウザで映像とTag番号を確認してください。画面にはカメラ映像、Tagの中心ずれ・距離、検出状態、通信エラーが表示されます。CREATEを物理的に押した後だけ、短時間のブラウザ駆動テストも使えます。

魚眼カメラでは、最初に競技用の180 mm AprilTag（Tag 0で可）を画面の中央・四隅・近距離・遠距離へ動かし、左右・上下にも傾けて歪みを校正します。チェスボードは不要です。

```bash
python3 calibrate_fisheye.py
```

完了後、`camera_hensuu.py` の `fisheye_enabled = True` に変更します。補正後の映像で、距離を使う自動移動の前に、公式AprilTag（180 mm）をカメラの正面に置き、メジャーでTag面までの距離を測って校正します。例えば1.00 mなら次を実行します。

```bash
python3 calibrate_camera.py 1.00
```

表示された焦点距離を `camera_hensuu.py` の `camera_focal_length_px` へ入力してください。Tagを斜めにせず、実機の取付高さ・解像度で行ってください。

## ライブラリを使うとき

```python
from rox_mecanum import Button, MotionCommand, TagStore

# メカナム移動は -1.0〜+1.0。
command = MotionCommand(forward=0.3, strafe=-0.2, rotate=0.0)

Button.CROSS       # ×
Button.CIRCLE      # ○
Button.SQUARE      # □
Button.TRIANGLE    # △
Button.CREATE      # CREATE
Button.OPTIONS     # OPTIONS
Button.TOUCHPAD    # タッチパッド押し込み
```

| クラス・関数 | 役割 |
|---|---|
| `PygameDualSense` | DualSense入力を読む |
| `MotionCommand` | 前後・横・旋回の移動指令 |
| `MecanumRobot.drive()` | 4輪モーターへ移動指令を送る |
| `RobotRuntime` | コントローラー・メカナム・サーボをまとめて開く |
| `TagStore` | 古いTag検出を無視して最新値だけ使う |
| `AprilTagDetector` | tag16h5を検出する |
| `ModeController` | 手動/自動モードの切替 |
| `add_manual_command()` | 自動速度とスティック速度を安全に合成する |
| `MaintenanceSite` | メンテナンス用ブラウザ画面 |

## RobStride角度サーボ

catchはCAN ID 5、liftはCAN ID 6です。実測角度 `mechPos (0x7019)` を使用し、速度積分で角度を推定しません。

```python
from servos import open_servos

servos = open_servos()
try:
    servos.attach()
    input("機械原点へ合わせてEnter: ")
    servos.home_from_feedback()
    servos.start_pid()
    servos.lift.write(45)
finally:
    servos.close()
```

PID、角度範囲、最高速度は `hensuu.py` で調整します。初回は必ず低速で、機構を安全な位置に置いて確認してください。

## 安全上の注意

- 自動の距離・時間設定が未調整なら、値を `0.0` のままにして動かさないでください。
- Tagやカメラを見失った時は、次段階へ進めず手動で位置を直します。
- ブラウザの駆動テストは周囲を確認してから、物理CREATEを押して短時間だけ有効化します。
- OPTIONSと実機の非常停止を常に使える状態にしてください。
