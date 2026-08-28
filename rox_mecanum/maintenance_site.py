"""整備時だけ使う、カメラ映像と安全な部分テスト用Webサーバー。"""

from __future__ import annotations

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from time import monotonic
from typing import Callable


class MaintenanceSite:
    """状態JSON、左右MJPEG、物理アーム必須のテスト操作を提供する。"""

    def __init__(self, port: int, action: Callable[[str], str] | None = None) -> None:
        self._lock = threading.Lock()
        self._status: dict[str, object] = {
            "running": True,
            "message": "起動中",
            "armed": False,
            "controls_available": action is not None,
        }
        self._frames: dict[str, bytes] = {"left": b"", "right": b""}
        self._armed_until = 0.0
        self._action = action
        self.server = self._create_server(int(port))
        threading.Thread(target=self.server.serve_forever, daemon=True, name="rox-maintenance-web").start()

    def set_status(self, **values: object) -> None:
        with self._lock:
            self._status.update(values)
            self._status["armed"] = monotonic() < self._armed_until

    def arm(self, seconds: float = 10.0) -> None:
        with self._lock:
            self._armed_until = monotonic() + float(seconds)
            self._status["armed"] = True
            self._status["armed_seconds"] = float(seconds)

    def set_frame(self, side: str, jpeg: bytes) -> None:
        if side not in self._frames:
            raise ValueError("side は left または right")
        with self._lock:
            self._frames[side] = bytes(jpeg)

    def close(self) -> None:
        self.set_status(running=False, message="停止")
        self.server.shutdown()

    def _create_server(self, port: int) -> ThreadingHTTPServer:
        site = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self) -> None:  # noqa: N802
                if self.path == "/api/status":
                    return self._json(site._snapshot())
                if self.path in {"/stream/left.mjpg", "/stream/right.mjpg"}:
                    return self._stream("left" if "left" in self.path else "right")
                self._bytes(200, "text/html; charset=utf-8", _HTML.encode())

            def do_POST(self) -> None:  # noqa: N802
                if not self.path.startswith("/api/action/"):
                    return self._json({"ok": False, "message": "unknown endpoint"}, 404)
                name = self.path.rsplit("/", 1)[-1]
                if name == "stop":
                    message = site._action(name) if site._action else "停止処理は未接続"
                    return self._json({"ok": True, "message": message})
                if not site._is_armed():
                    return self._json({"ok": False, "message": "物理CREATEでテストを有効化してください"}, 403)
                message = site._action(name) if site._action else "テスト動作は未接続"
                return self._json({"ok": True, "message": message})

            def _stream(self, side: str) -> None:
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                try:
                    while True:
                        frame = site._frame(side)
                        if frame:
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
                            self.wfile.flush()
                        threading.Event().wait(0.10)
                except (BrokenPipeError, ConnectionResetError):
                    return

            def _json(self, value: object, status: int = 200) -> None:
                self._bytes(status, "application/json; charset=utf-8", json.dumps(value, ensure_ascii=False).encode())

            def _bytes(self, status: int, content_type: str, body: bytes) -> None:
                self.send_response(status)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, *_: object) -> None:
                pass

        return ThreadingHTTPServer(("0.0.0.0", port), Handler)

    def _snapshot(self) -> dict[str, object]:
        with self._lock:
            value = dict(self._status)
            value["armed"] = monotonic() < self._armed_until
            return value

    def _frame(self, side: str) -> bytes:
        with self._lock:
            return self._frames[side]

    def _is_armed(self) -> bool:
        with self._lock:
            return monotonic() < self._armed_until


_HTML = """<!doctype html><meta charset=utf-8><title>ROX 状態監視</title>
<style>body{font-family:sans-serif;background:#111827;color:#eef2ff;margin:20px;max-width:1100px}img{width:100%;max-width:960px;border:1px solid #64748b}button{padding:9px;margin:4px}pre{background:#1e293b;padding:12px;overflow:auto}small{color:#cbd5e1}</style>
<h1>ROX 状態監視</h1><p>カメラ映像、Tag、DualSense、catch/lift角度、4輪への速度指令を表示します。</p>
<p><small>4輪速度・加速値は送信指令です。実測の加速度や車輪角度は、この画面では表示しません。</small></p>
<img src='/stream/left.mjpg' alt='ロボット正面カメラ映像'>
<section id=controls><p>駆動テストはコントローラーのCREATEで10秒間だけ有効化されます。</p>
<p><button onclick=go('forward')>前進テスト</button><button onclick=go('backward')>後退テスト</button><button onclick=go('left')>左スライド</button><button onclick=go('right')>右スライド</button><button onclick=go('solenoid')>発射テスト</button><button onclick=go('stop')>停止</button></p></section>
<pre id=s>読み込み中...</pre><script>
async function refresh(){let r=await fetch('/api/status');let v=await r.json();s.textContent=JSON.stringify(v,null,2);controls.hidden=!v.controls_available}
async function go(n){let r=await fetch('/api/action/'+n,{method:'POST'});alert((await r.json()).message)}refresh();setInterval(refresh,200);
</script>"""
