#!/usr/bin/env python3
"""Live web viewer for om6dof_perception.

Subscribes to the perception debug image + status topics and serves them in
a browser: MJPEG stream, tracking status, and a box to retarget the VLM.
Camera-free (topics only), so it can run alongside perception_node.

  ros2 run om6dof_perception perception_view
  -> http://<robot-ip>:8083   (trusted LAN only, no auth)
"""
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

PORT = 8083

HTML = """<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>om6dof_perception</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
:root{color-scheme:dark}
body{margin:0;font:14px/1.5 system-ui,sans-serif;background:#14161c;color:#dfe3ea}
header{padding:10px 18px;background:#1b1e27;display:flex;gap:12px;align-items:center;flex-wrap:wrap}
h1{font-size:15px;margin:0}
main{display:flex;gap:16px;padding:16px;flex-wrap:wrap}
img{max-width:640px;width:100%;border-radius:8px;background:#000}
section{background:#1b1e27;border-radius:10px;padding:14px;min-width:300px;flex:1}
.badge{padding:2px 10px;border-radius:10px;font-size:12px;background:#333}
.ok{background:#1d4d2b}.warn{background:#5c4a17}
input{width:100%;box-sizing:border-box;padding:8px;border-radius:6px;border:1px solid #3a3f4d;background:#12141a;color:#dfe3ea;margin:6px 0}
button{padding:8px 16px;border:0;border-radius:6px;background:#2a5fd0;color:#fff;cursor:pointer}
pre{background:#12141a;border-radius:6px;padding:10px;font-size:12px;overflow-x:auto}
td{padding:2px 10px 2px 0}
</style></head><body>
<header><h1>om6dof_perception</h1>
 <span class="badge" id="btgt">target</span>
 <span class="badge" id="bee">ee</span>
 <span id="bdist" style="color:#ffd45e;font-weight:600"></span>
</header>
<main>
 <section style="flex:1.4"><img src="/stream" alt="menunggu frame dari perception_node..."></section>
 <section>
  <label>Ganti target (deskripsi bahasa Inggris, makin detail makin akurat)</label>
  <input id="tgt" placeholder="red cube on the table">
  <button onclick="retarget()">Set target</button>
  <div id="msg" style="color:#9aa3b2;font-size:13px;margin:4px 0"></div>
  <table>
   <tr><td>target</td><td id="ttgt">-</td></tr>
   <tr><td>gripper (EE)</td><td id="tee">-</td></tr>
   <tr><td>jarak EE→target</td><td id="tdist">-</td></tr>
  </table>
  <pre id="raw">menunggu status...</pre>
 </section>
</main>
<script>
async function retarget(){
 const v=document.getElementById('tgt').value.trim();
 if(!v)return;
 await fetch('/set_target',{method:'POST',body:JSON.stringify({target:v})});
 document.getElementById('msg').textContent='target dikirim: '+v;
}
const fmt=p=>p?`[${p.map(v=>v.toFixed(3)).join(', ')}] m`:'-';
setInterval(async()=>{
 try{
  const s=await (await fetch('/status.json')).json();
  if(!s.target)return;
  const bt=document.getElementById('btgt'),be=document.getElementById('bee');
  bt.textContent='target: '+s.target.state;
  bt.className='badge '+(s.target.state==='tracking'?'ok':'warn');
  be.textContent='ee: '+s.ee.state;
  be.className='badge '+(s.ee.state==='tracking'?'ok':'warn');
  document.getElementById('ttgt').textContent=fmt(s.target.point);
  document.getElementById('tee').textContent=fmt(s.ee.point);
  let d='-';
  if(s.target.point&&s.ee.point){
   const q=s.target.point.map((v,i)=>v-s.ee.point[i]);
   d=Math.hypot(...q).toFixed(3)+' m';
  }
  document.getElementById('tdist').textContent=d;
  document.getElementById('bdist').textContent=d==='-'?'':('EE\\u2192target '+d);
  document.getElementById('raw').textContent=JSON.stringify(s,null,1);
 }catch(e){}
},700);
</script></body></html>"""


class ViewNode(Node):
    def __init__(self):
        super().__init__("om6dof_perception_view")
        self.lock = threading.Lock()
        self.frame = None
        self.frame_seq = 0
        self.cond = threading.Condition(self.lock)
        self.status = "{}"
        self.create_subscription(
            CompressedImage, "/om6dof_perception/debug_image/compressed",
            self._on_img, 2)
        self.create_subscription(
            String, "/om6dof_perception/status", self._on_status, 10)
        self.target_pub = self.create_publisher(
            String, "/om6dof_perception/set_target", 1)

    def _on_img(self, msg):
        with self.cond:
            self.frame = bytes(msg.data)
            self.frame_seq += 1
            self.cond.notify_all()

    def _on_status(self, msg):
        with self.lock:
            self.status = msg.data

    def next_frame(self, last_seq, timeout=2.0):
        with self.cond:
            self.cond.wait_for(lambda: self.frame_seq != last_seq,
                               timeout=timeout)
            return self.frame, self.frame_seq


rclpy.init()
view = ViewNode()
threading.Thread(target=lambda: rclpy.spin(view), daemon=True).start()


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass

    def _body(self, code, body, ctype):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path == "/":
            self._body(200, HTML.encode(), "text/html; charset=utf-8")
        elif self.path.startswith("/status.json"):
            with view.lock:
                self._body(200, view.status.encode(), "application/json")
        elif self.path.startswith("/stream"):
            self.send_response(200)
            self.send_header(
                "Content-Type",
                "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            seq = -1
            try:
                while True:
                    frame, seq = view.next_frame(seq)
                    if frame is None:
                        time.sleep(0.2)
                        continue
                    self.wfile.write(b"--frame\r\n"
                                     b"Content-Type: image/jpeg\r\n"
                                     b"Content-Length: " +
                                     str(len(frame)).encode() + b"\r\n\r\n")
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self._body(404, b"{}", "application/json")

    def do_POST(self):
        if not self.path.startswith("/set_target"):
            return self._body(404, b"{}", "application/json")
        n = int(self.headers.get("Content-Length", 0))
        try:
            req = json.loads(self.rfile.read(n) or b"{}")
        except json.JSONDecodeError:
            req = {}
        target = str(req.get("target", "")).strip()
        if target:
            view.target_pub.publish(String(data=target))
        self._body(200, b"{}", "application/json")


def main(args=None):
    print(f"perception view on http://0.0.0.0:{PORT}")
    ThreadingHTTPServer(("0.0.0.0", PORT), Handler).serve_forever()


if __name__ == "__main__":
    main()
