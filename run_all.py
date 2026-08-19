import subprocess
import time

scripts = [
    "mecanum.py"
    "motiage.py"
]

processes = []

print("--- すべてのプログラムを起動します ---")

# スクリプトを1つずつバックグラウンドプロセスとして起動
for script in scripts:
    # python3 script_name.py をバックグラウンドで非同期実行
    p = subprocess.Popen(["python3", script])
    processes.append(p)
    print(f"起動完了: {script} (PID: {p.pid})")

print("--- すべてのプログラムが実行中です ---")

try:
    # すべてのプログラムが終わるまで待機する場合（Ctrl+C で一括停止可能）
    for p in processes:
        p.wait()
except KeyboardInterrupt:
    print("\n中断信号（Ctrl+C）を検出しました。すべてのプロセスを停止します...")
    for p in processes:
        p.terminate()  # プロセスを強制終了
    print("すべてのプロセスを停止しました。")