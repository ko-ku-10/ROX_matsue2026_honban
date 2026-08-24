# ROX2026 ロボットプログラム

本番は **GAME1** と **GAME2** だけを実行する。機構ごとの確認や距離測定は `experiments/` の実験プログラムで行う。

## フォルダ構成

| 場所 | 役割 |
|---|---|
| `game1.py` | GAME1本番用 |
| `game2.py` | GAME2本番用 |
| `game3.py` | GAME3・操作練習用 |
| `*_hensuu.py`, `hensuu.py` | 本番・実験で共通に使う調整値 |
| `experiments/` | 本番では実行しない単体実験・カメラ確認 |
| `rox_mecanum/` | 共通ライブラリ。通常は編集しない |
| `tests/` | PC上で行う自動テスト |

## 本番で実行するもの

```bash
# GAME1
python3 game1.py

# GAME2
python3 game2.py

# GAME3・メカナム/catch/lift/ソレノイドの操作練習
python3 game3.py
```

GAME1/GAME2は起動直後は完全手動モード。タッチパッドで自動モードへ切り替える。OPTIONSは非常停止・終了。GAME1〜3はEnter入力を待たず、catch/liftの起動時実測角度を自動で0度として登録する。起動前に機構を所定の開始姿勢へ置いておくこと。

GAME3は常に手動操作である。CREATEで地面保持姿勢にしてから走行する。スティック中立付近の誤入力は `game3_hensuu.py` の `manual_stick_deadzone` で無視し、停止中は毎周期に停止指令を送る。△で `game3_hensuu.py` の `motiage_steps` に上から書いた順で連続動作を開始する。各行はエンコーダーで到達確認後、`mechanism_settle_sec` 秒だけ反動が収まるのを待ってから次へ進む。最後の行が終わるとソレノイドを発射する。到達判定の範囲は `sequence_target_tolerance_deg` で調整する。R1は操作練習用のソレノイド単体テストで、機構姿勢に関係なく設定時間だけONにする。

同じコントローラーやモーター通信を使うプログラムは、同時に起動しないこと。

DualSenseは、起動時に `hensuu.py` の `dualsense_mac_address` へBluetooth接続を試し、OPTIONSまたは終了時に自動切断する。PSボタンでDualSenseの電源を入れてから起動する。これにより、プログラムを実行していない間は接続を維持せず、電池を節約できる。

## ボール機構ライブラリ

GAME3と機構実験は同じ `BallMechanism` を使う。実験で角度を調整すればGAME3にも同じ設定が反映される。

```python
from rox_mecanum import BallMechanism

mechanism = BallMechanism(servos)
mechanism.ground()          # 地面保持・走行姿勢
mechanism.grab()            # 掴む
mechanism.release()         # Robot外へ出す
mechanism.fire_pose(30.0)   # 発射姿勢へ
```

実験用ファイルは「この共通部が実機で動くか」を確認するための操作画面である。実験で確定した角度・速度は `hensuu.py` / `game3_hensuu.py` へ書く。GAME3にも反映したい動作そのものを変更する時は、`rox_mecanum/ball_mechanism.py` を変更する。

## 実験で実行するもの

必ずリポジトリのフォルダで、`python3 -m` を付けて実行する。

```bash
cd ~/Desktop/honban/ROX_matsue2026_honban

# メカナム手動走行だけ
python3 -m experiments.mecanum_manual

# GAME3と同じcatch / lift姿勢だけを確認
python3 -m experiments.mechanism_manual

# liftだけを低速で持上げ確認（△: 持上げ / ×: 戻す / OPTIONS: 停止）
python3 -m experiments.lift_test

# catch / lift の現在位置保持だけ
python3 -m experiments.servo_hold

# ソレノイドだけ
python3 -m experiments.solenoid_test

# カメラ映像とAprilTagだけ（モーターは動かない）
python3 -m experiments.maintenance --camera-only

# カメラ映像・Tag・短時間の駆動テスト
python3 -m experiments.maintenance

# GAME2でどのパネルを狙うかだけ確認（モーターは動かない）
python3 -m experiments.game2_target

# カメラからTagまでの距離を1.00mに置いて距離校正
python3 -m experiments.calibrate_distance 1.00
```

`experiments.maintenance --camera-only` と `experiments.mecanum_manual` は同時に使える。ブラウザで映像を見ながら手動走行を確認できる。

## 調整値を書く場所

実験で確定した値は、実験用のファイルではなく次の設定ファイルへ書く。本番も同じ値を使う。

| 変更したいもの | ファイル |
|---|---|
| メカナム、PID、catch/lift、ソレノイド | `hensuu.py` |
| カメラ、Tagサイズ、焦点距離 | `camera_hensuu.py` |
| GAME1のTag番号、時間、速度、距離 | `game1_hensuu.py` |
| GAME2のパネルTag、段ごとの発射距離、速度 | `game2_hensuu.py` |

ボールを扱う共通角度は `hensuu.py` にまとめる。移動中は
`catch_ball_hold_angle`（地面で保持）と `lift_ball_ground_angle`（地面高さ）を使う。
`catch_ball_grab_angle` は掴む時、`catch_ball_release_angle` はRobotの外へ出す時だけ使う。

GAME2の段ごとの発射距離はここで設定する。

```python
# game2_hensuu.py
shot_distance_m = {
    "top": 1.20,
    "middle": 1.20,
    "bottom": 1.20,
}
```

値は「カメラからパネルTagまで残す距離[m]」。実射して最も当たる値を、上・中央・下ごとに入れる。

## 共通のコントローラー操作

| 操作 | 機能 |
|---|---|
| 左スティック | 前後・左右の平行移動 |
| R2 + 右スティック左右 | 旋回 |
| タッチパッド押し込み | 完全手動 / 自動モード切替 |
| OPTIONS | 非常停止・終了 |

## GAME1の自動操作

| ボタン | 動作 |
|---|---|
| CREATE | lift/catchを開始姿勢へ展開して固定 |
| △ | Tag 1または9を基準にTag 8へ向かう |
| ○ | Tag 8で位置合わせ後、ゲートを通過 |
| □ | Tag 12・13への位置合わせを開始 |
| R1 | 板を低速で押し込む |
| × | 板へ上がれたことを操縦者が確定 |
| L2 | Tag 6・10を通ってTag 0へ帰還 |

## GAME2の自動操作

| ボタン | 動作 |
|---|---|
| CREATE | パネル読取、段の選択、接近、横スライド照準 |
| △ | liftを発射高さへ上げる |
| L2 | 発射 |
| × | liftを下げ、補給位置へ後退 |

中央段、上段、下段の順に狙う。同じ段に2枚以上あれば、その中間を狙う。

## 初回セットアップ

```bash
python3 -m pip install --user '.[hardware,vision]'
```

カメラ距離を使う自動接近の前に、180 mm AprilTagをカメラ正面1.00mへ置いて `experiments.calibrate_distance` を実行する。表示された値を `camera_hensuu.py` の `camera_focal_length_px` へ入力する。
