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


_HTML = """<!doctype html>
<html lang=ja><meta charset=utf-8><meta name=viewport content="width=device-width,initial-scale=1">
<title>ROX | Robot Console</title>
<style>
*{box-sizing:border-box}body{margin:0;background:#07111f;color:#e7f0ff;font-family:system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif;letter-spacing:.02em}
body:before{content:"";position:fixed;inset:0;pointer-events:none;background:radial-gradient(circle at 15% 0%,#174d78 0,transparent 32%),radial-gradient(circle at 90% 10%,#25165c 0,transparent 26%);opacity:.75}
.shell{position:relative;max-width:1440px;margin:auto;padding:22px}.top{display:flex;align-items:center;justify-content:space-between;gap:16px;margin-bottom:18px}.brand{display:flex;align-items:center;gap:12px}.mark{width:42px;height:42px;border-radius:13px;display:grid;place-items:center;background:linear-gradient(135deg,#28d9ff,#7562ff);color:#07111f;font-weight:900;font-size:18px;box-shadow:0 0 30px #25c9ff66}.brand h1{margin:0;font-size:20px;letter-spacing:.08em}.brand small,.muted{color:#8ba4c7}.live{display:flex;align-items:center;gap:8px;padding:8px 12px;border:1px solid #255070;background:#091a2cbb;border-radius:999px;font-size:12px}.dot{width:9px;height:9px;border-radius:50%;background:#7d8ba0}.dot.ok{background:#35e899;box-shadow:0 0 13px #35e899}.dot.bad{background:#ff587a;box-shadow:0 0 13px #ff587a}.hero{display:flex;justify-content:space-between;gap:14px;align-items:end;padding:18px 20px;border:1px solid #254b70;border-radius:18px;background:linear-gradient(110deg,#0e2138e8,#0b172ae8);box-shadow:0 18px 45px #02071180;margin-bottom:16px}.game{font-weight:800;color:#58d8ff;font-size:12px;letter-spacing:.15em}.stage{font-size:20px;font-weight:700;margin-top:5px}.mode{border:1px solid #385f8b;border-radius:10px;padding:8px 12px;color:#c6e8ff;font-size:13px}.grid{display:grid;grid-template-columns:1.4fr .9fr;gap:16px}.card{background:#0b1a2de8;border:1px solid #1e405f;border-radius:18px;overflow:hidden;box-shadow:0 12px 30px #0004}.card h2{font-size:12px;letter-spacing:.12em;color:#79cfff;margin:0;padding:14px 16px;border-bottom:1px solid #1d3a55}.camera{display:block;width:100%;min-height:230px;object-fit:contain;background:#040a13}.camera-note{padding:9px 14px;color:#8ba4c7;font-size:12px}.side{display:grid;gap:16px}.section{padding:14px 16px}.servo-grid,.wheel-grid,.stick-grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:9px}.metric{background:#0a2438;border:1px solid #1c455e;border-radius:12px;padding:10px}.metric b{display:block;font-size:11px;color:#92b8d8;margin-bottom:5px}.metric strong{font-size:17px}.metric small{display:block;color:#84a3bd;margin-top:3px}.buttons{display:flex;gap:6px;flex-wrap:wrap}.button-name{padding:5px 8px;border-radius:7px;font-size:11px;border:1px solid #234461;transition:.12s}.button-name.off{background:#0a1b2c;color:#58758e}.button-name.on{background:#1675a8;color:#fff;border-color:#57d8ff;box-shadow:0 0 13px #26cfff99;font-weight:800}.empty{color:#7794ad;font-size:13px}.tags{display:grid;grid-template-columns:repeat(auto-fill,minmax(155px,1fr));gap:9px;padding:14px 16px}.tag{border:1px solid #254c66;background:#0a2235;border-radius:12px;padding:10px}.tag-head{display:flex;justify-content:space-between;color:#8ad9ff;font-weight:700}.fresh{color:#37e79b}.old{color:#ffbd61}.tag small{display:block;color:#9ab1c8;margin-top:5px}.controls{margin-top:16px;padding:14px 16px;border:1px solid #7c4e22;background:#2b1c0ce8;border-radius:18px}.controls h2{font-size:14px;margin:0 0 6px;color:#ffca77}.controls p{font-size:12px;color:#dcc49e;margin:0 0 10px}.actions{display:flex;flex-wrap:wrap;gap:8px}button{border:1px solid #3a6890;border-radius:10px;background:#113251;color:#e7f6ff;padding:9px 12px;font-weight:700;cursor:pointer}button:hover{background:#1b4d76}button.stop{border-color:#8d3851;background:#4a1828}.footer{font-size:11px;color:#6685a2;margin:16px 2px}details{margin-top:16px;border:1px solid #1d3d59;border-radius:12px;padding:10px 13px;color:#9bb7ce}summary{cursor:pointer}pre{white-space:pre-wrap;overflow:auto;color:#cde5fb;font-size:11px}
@media(max-width:850px){.shell{padding:12px}.grid{grid-template-columns:1fr}.hero{align-items:start;flex-direction:column}.top{align-items:flex-start}.stage{font-size:17px}}
</style>
<body><main class=shell>
<header class=top><div class=brand><div class=mark>R</div><div><h1>ROX ROBOT CONSOLE</h1><small>ライブ状態監視</small></div></div><div class=live><i id=dot class="dot"></i><span id=liveText>接続を確認中</span></div></header>
<section class=hero><div><div id=game class=game>ROX</div><div id=stage class=stage>起動中</div></div><div id=mode class=mode>モード: --</div></section>
<section class=grid><div class=card><h2>CAMERA / APRILTAG</h2><img class=camera src='/stream/left.mjpg' alt='ロボットカメラ映像'><div id=cameraNote class=camera-note>カメラを待っています…</div></div>
<div class=side><div class=card><h2>DUALSENSE INPUT <span class=muted>（光っているボタンが押下中）</span></h2><div class=section><div id=buttons class=buttons></div><div class=stick-grid style="margin-top:10px"><div class=metric><b>LEFT STICK</b><strong id=leftStick>--</strong><small id=leftMagnitude>magnitude --</small></div><div class=metric><b>RIGHT STICK</b><strong id=rightStick>--</strong><small id=rightMagnitude>magnitude --</small></div></div></div></div>
<div class=card><h2>MECHANISM / PID</h2><div class=section><div class=servo-grid><div id=catchServo class=metric>catch: --</div><div id=liftServo class=metric>lift: --</div></div><div id=pidError class=muted style="margin-top:9px;font-size:12px">PID: 確認中</div></div></div></div></section>
<section class=card style="margin-top:16px"><h2>MECANUM COMMAND</h2><div id=wheels class="wheel-grid section"></div><div class="camera-note">表示値は4輪へ送った速度指令です。実測の車輪速度・加速度ではありません。</div></section>
<section class=card style="margin-top:16px"><h2>TAG DETECTIONS</h2><div id=tags class=tags><span class=empty>Tagを待っています…</span></div></section>
<section id=controls class=controls hidden><h2>MAINTENANCE CONTROL</h2><p>物理コントローラーのCREATEで有効化した時だけ操作できます。</p><div class=actions><button onclick=go('forward')>前進テスト</button><button onclick=go('backward')>後退テスト</button><button onclick=go('left')>左スライド</button><button onclick=go('right')>右スライド</button><button onclick=go('solenoid')>発射テスト</button><button class=stop onclick=go('stop')>停止</button></div></section>
<details><summary>生データを表示</summary><pre id=raw>読み込み中...</pre></details><p class=footer>ROX2026 · 監視画面は確認用です。ゲーム実行中のブラウザからは駆動できません。</p>
</main><script>
const $=id=>document.getElementById(id);const n=(v,d=2)=>typeof v==='number'?v.toFixed(d):'--';
const allButtons=['cross','circle','square','triangle','l1','r1','l2','r2','create','options','l3','r3','ps','touchpad','mute','dpad_up','dpad_down','dpad_left','dpad_right'];
function putServo(id,name,v){let el=$(id);if(!v){el.textContent=name+': --';return}el.innerHTML='<b>'+name.toUpperCase()+'</b><strong>'+n(v.current_angle)+'°</strong><small>目標 '+n(v.target_angle)+'° / '+(v.holding?'PID HOLD':'PID OFF')+'</small>'}
function putStick(id,magId,v){$(id).textContent=v?'x '+n(v.x)+' / y '+n(v.y):'--';$(magId).textContent='magnitude '+(v?n(v.magnitude):'--')}
function render(v){let good=v.running!==false;$("dot").className='dot '+(good?'ok':'bad');$("liveText").textContent=good?'LIVE / 更新中':'STOPPED';$("game").textContent=v.game||'ROX MAINTENANCE';$("stage").textContent=v.stage||v.message||'状態を待っています';$("mode").textContent='モード: '+(v.mode||'--');let c=v.controller||{};let bs=c.active_buttons||[];$("buttons").innerHTML=allButtons.map(x=>'<span class="button-name '+(bs.includes(x)?'on':'off')+'">'+x+'</span>').join('');putStick('leftStick','leftMagnitude',c.left_stick);putStick('rightStick','rightMagnitude',c.right_stick);let sv=v.servos||{};putServo('catchServo','catch',sv.catch);putServo('liftServo','lift',sv.lift);$("pidError").textContent=sv.pid_error?'PID ERROR: '+sv.pid_error:'PID: 正常';let cm=v.camera||{};$("cameraNote").textContent=cm.connected===false?'CAMERA ERROR: '+(cm.error||'接続できません'):'カメラ接続中 / Tagの枠と番号を映像に重ねて表示';let wheels=(v.mecanum||{}).wheel_speed_commands||{};let a=(v.mecanum||{}).acceleration_limit_per_sec;$("wheels").innerHTML=['FL','FR','RL','RR'].map(x=>'<div class=metric><b>'+x+'</b><strong>'+n(wheels[x],3)+'</strong><small>加速制限 '+n(a,2)+'/s</small></div>').join('');let tags=v.tags||{};let entries=Object.entries(tags);$("tags").innerHTML=entries.length?entries.map(([id,t])=>'<div class=tag><div class=tag-head><span>TAG '+id+'</span><span class='+(t.fresh?'fresh':'old')+'>'+ (t.fresh?'LIVE':'OLD')+'</span></div><small>距離 '+n(t.distance_m,3)+' m</small><small>左右ずれ '+n(t.robot_x_error,3)+'</small><small>受信 '+n(t.age_sec,3)+' 秒前</small></div>').join(''):'<span class=empty>Tagを待っています…</span>';$("controls").hidden=!v.controls_available;$("raw").textContent=JSON.stringify(v,null,2)}
async function refresh(){try{let r=await fetch('/api/status',{cache:'no-store'});render(await r.json())}catch(e){$("dot").className='dot bad';$("liveText").textContent='サイトとの通信エラー'}}
async function go(nm){let r=await fetch('/api/action/'+nm,{method:'POST'});let v=await r.json();alert(v.message);refresh()}refresh();setInterval(refresh,250);
</script></body></html>"""
