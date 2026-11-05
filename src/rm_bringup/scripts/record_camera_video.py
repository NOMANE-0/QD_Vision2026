#!/usr/bin/env python3
"""相机实时预览及手动录制；仅导出原始分辨率、无叠加画面的无损视频。"""

import argparse
from datetime import datetime
from fractions import Fraction
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import queue
import signal
import subprocess
import tempfile
import threading
import time

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import av
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import yaml


PAGE = r"""<!doctype html><html lang="zh-CN"><meta charset="utf-8">
<title>相机预览与录制</title>
<style>body{background:#15191e;color:#eee;font:18px sans-serif;margin:24px}
button{padding:12px 24px;margin:8px 12px 12px 0;font-size:18px;cursor:pointer}
img{display:block;max-width:100%;max-height:72vh;background:#000}
#status{white-space:pre-wrap;line-height:1.6}</style>
<h2>相机实时画面</h2>
<p>预览时不保存。录制保留原始分辨率及画质，不叠加文字或检测框。</p>
<button id="start" onclick="control('start')">开始录制</button>
<button id="stop" onclick="control('stop')">停止并保存</button>
<div id="status">正在连接相机…</div><img id="preview" alt="等待相机画面">
<script>
async function control(action){
  try{const r=await fetch('/'+action,{method:'POST'});const s=await r.json();
      if(!r.ok)alert(s.error);}catch(e){alert('无法连接录制程序');}}
async function refresh(){
  try{const s=await(await fetch('/status',{cache:'no-store'})).json();
    const labels={preview:'预览中，尚未录制',recording:'录制中',saving:'正在保存',
                  saved:'已保存，可继续预览',error:'录制失败'};
    document.getElementById('status').textContent=(labels[s.phase]||s.phase)+
      ' | '+s.elapsed.toFixed(1)+' / '+s.limit+' 秒 | 已写入 '+s.frames+
      ' 帧 | 队列丢帧 '+s.dropped+'\n'+(s.file||'')+(s.error?'\n'+s.error:'')+
      (s.camera_alive?'':'\n未收到实时画面，请检查相机');
    document.getElementById('start').disabled=!s.camera_alive||
      s.phase==='recording'||s.phase==='saving';
    document.getElementById('stop').disabled=s.phase!=='recording';
  }catch(e){document.getElementById('status').textContent='连接已断开，请检查终端';}
  setTimeout(refresh,500);
}
const image=document.getElementById('preview');
function nextFrame(){image.src='/frame.jpg?t='+Date.now();}
image.onload=()=>setTimeout(nextFrame,100);
image.onerror=()=>setTimeout(nextFrame,500);
nextFrame();refresh();
</script></html>"""


def pixels_from_message(msg):
    """按 ROS 图像步长取彩色像素，保留原始分辨率与 RGB/BGR 顺序。"""
    if msg.encoding not in ("rgb8", "bgr8"):
        raise ValueError(f"不支持的图像编码：{msg.encoding}")
    pixels = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    return pixels[:, :msg.width * 3].reshape(msg.height, msg.width, 3)


class VideoRecorder(Node):
    def __init__(self, output_dir, duration):
        super().__init__("camera_video_preview")
        self.output_dir = output_dir
        self.duration = duration
        self.lock = threading.Lock()
        self.latest = None
        self.latest_wall = 0.0
        self.phase = "preview"
        self.frames = 0
        self.dropped = 0
        self.error = ""
        self.file = ""
        self.started = None
        self.elapsed = 0.0
        self.worker = None
        self.subscription = self.create_subscription(
            Image, "/image_raw", self.receive, qos_profile_sensor_data)

    def receive(self, msg):
        """更新预览帧，仅在用户开始录制后将图像交给编码线程。"""
        now = time.monotonic()
        with self.lock:
            self.latest, self.latest_wall = msg, now
            if self.phase != "recording":
                return
            if self.started is None:
                self.started = now
            self.elapsed = now - self.started
            if self.elapsed >= self.duration:
                self.phase = "saving"
                self.stop_event.set()
                return
            try:
                self.pending.put_nowait((msg, self.elapsed))
            except queue.Full:
                self.dropped += 1

    def start_recording(self):
        """由预览页面启动一段录制，已有录制未结束时拒绝重复启动。"""
        with self.lock:
            if self.phase in ("recording", "saving"):
                raise ValueError("当前视频尚未保存完成")
            if time.monotonic() - self.latest_wall > 2:
                raise ValueError("尚未收到实时相机画面")
            self.output_dir.mkdir(parents=True, exist_ok=True)
            name = "camera_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".mkv"
            self.file = str(self.output_dir / name)
            self.frames, self.dropped, self.error = 0, 0, ""
            self.started, self.elapsed = None, 0.0
            self.pending = queue.Queue(maxsize=64)
            self.stop_event = threading.Event()
            self.phase = "recording"
            self.worker = threading.Thread(target=self.encode_video)
            self.worker.start()
        print(f"开始录制：{self.file}，最多 {self.duration:g} 秒", flush=True)

    def stop_recording(self):
        """停止接收新帧，编码线程完成队列后封装视频。"""
        with self.lock:
            if self.phase == "recording":
                self.phase = "saving"
                self.stop_event.set()

    def encode_video(self):
        """无损编码 RGB 画面；预览缩放和 JPEG 压缩不会进入保存的视频。"""
        try:
            with av.open(self.file, "w") as container:
                stream = None
                previous_pts = -1
                while not (self.stop_event.is_set() and self.pending.empty()):
                    try:
                        msg, elapsed = self.pending.get(timeout=0.1)
                    except queue.Empty:
                        continue
                    if stream is None:
                        stream = container.add_stream("libx264rgb", rate=1000)
                        stream.width, stream.height = msg.width, msg.height
                        stream.pix_fmt = "rgb24"
                        stream.codec_context.time_base = Fraction(1, 1000)
                        stream.codec_context.thread_count = 4
                        stream.options = {"preset": "ultrafast", "crf": "0"}
                    if (msg.width, msg.height) != (stream.width, stream.height):
                        raise ValueError("录制期间相机分辨率发生变化")
                    frame = av.VideoFrame.from_ndarray(
                        pixels_from_message(msg), format=msg.encoding[:-1] + "24")
                    frame.pts = max(previous_pts + 1, round(elapsed * 1000))
                    frame.time_base = Fraction(1, 1000)
                    previous_pts = frame.pts
                    for packet in stream.encode(frame):
                        container.mux(packet)
                    with self.lock:
                        self.frames += 1
                if stream is not None:
                    for packet in stream.encode():
                        container.mux(packet)
            if not self.frames:
                Path(self.file).unlink(missing_ok=True)
                raise ValueError("本次没有录到图像，没有保留空文件")
            owner = Path(get_package_prefix("rm_bringup")).parents[1].stat()
            if os.getuid() == 0:
                os.chown(self.file, owner.st_uid, owner.st_gid)
            with self.lock:
                self.phase = "saved"
            print(f"已保存：{self.file}，{self.frames} 帧，队列丢帧 {self.dropped}", flush=True)
        except Exception as error:
            with self.lock:
                self.error, self.phase = str(error), "error"
            print(f"录制失败：{error}", flush=True)

    def status(self):
        """返回页面状态，同时处理自动结束和相机断流。"""
        with self.lock:
            now = time.monotonic()
            alive = now - self.latest_wall < 2
            if self.phase == "recording" and self.started is not None:
                self.elapsed = now - self.started
                if self.elapsed >= self.duration or not alive:
                    self.phase = "saving"
                    self.stop_event.set()
            return dict(phase=self.phase, elapsed=self.elapsed, limit=self.duration,
                        frames=self.frames, dropped=self.dropped, file=self.file,
                        error=self.error, camera_alive=alive)


def create_handler(recorder):
    """创建仅监听本机的预览和录制控制接口。"""
    class Handler(BaseHTTPRequestHandler):
        def reply(self, code, content_type, data):
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            route = self.path.split("?", 1)[0]
            if route == "/":
                self.reply(200, "text/html; charset=utf-8", PAGE.encode())
            elif route == "/status":
                self.reply(200, "application/json", json.dumps(recorder.status()).encode())
            elif route == "/frame.jpg":
                with recorder.lock:
                    msg = recorder.latest
                if msg is None:
                    self.reply(503, "text/plain", b"Waiting for camera")
                    return
                pixels = pixels_from_message(msg)
                if msg.encoding == "rgb8":
                    pixels = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
                if msg.width > 960:
                    pixels = cv2.resize(pixels, (960, round(msg.height * 960 / msg.width)))
                ok, jpeg = cv2.imencode(".jpg", pixels, [cv2.IMWRITE_JPEG_QUALITY, 80])
                self.reply(200 if ok else 500, "image/jpeg", jpeg.tobytes())
            else:
                self.reply(404, "text/plain", b"Not found")

        def do_POST(self):
            # 仅接受本机页面发起的控制操作，避免跨站表单误触发录制。
            origin = self.headers.get("Origin")
            if origin and origin != "http://" + self.headers.get("Host", ""):
                self.reply(403, "application/json", b'{"error":"Invalid origin"}')
                return
            try:
                if self.path == "/start":
                    recorder.start_recording()
                elif self.path == "/stop":
                    recorder.stop_recording()
                else:
                    self.reply(404, "application/json", b"{}")
                    return
                self.reply(200, "application/json", json.dumps(recorder.status()).encode())
            except ValueError as error:
                self.reply(409, "application/json", json.dumps({"error": str(error)}).encode())

        def log_message(self, *_args):
            pass

    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=120.0, help="每段最长录制秒数")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--output-dir", type=Path, default=Path("/ros_ws/record/camera_only"))
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error("录制时长必须大于 0")
    rclpy.init()
    recorder = VideoRecorder(args.output_dir.resolve(), args.duration)
    camera = None
    server = None
    with tempfile.TemporaryDirectory(prefix="qd_camera_preview_") as runtime:
        try:
            server = ThreadingHTTPServer(("127.0.0.1", args.port), create_handler(recorder))
            threading.Thread(target=server.serve_forever, daemon=True).start()
            # 先发现已有图像发布者，避免重复打开正在使用的相机。
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline:
                rclpy.spin_once(recorder, timeout_sec=0.1)
            if recorder.count_publishers("/image_raw") == 0:
                config_path = Path(get_package_share_directory("rm_bringup"))
                config_path /= "config/node_params/camera_driver_params.yaml"
                config = yaml.safe_load(config_path.read_text())
                params = Path(runtime) / "camera.yaml"
                params.write_text(yaml.safe_dump({"/**": {"ros__parameters": config}}))
                executable = Path(get_package_prefix("hik_camera"))
                executable /= "lib/hik_camera/hik_camera_node"
                env = os.environ.copy()
                libraries = "/opt/MVS/lib/64:/opt/MVS/lib/64/ThirdParty"
                env["LD_LIBRARY_PATH"] = libraries + ":" + env.get("LD_LIBRARY_PATH", "")
                camera = subprocess.Popen(
                    [str(executable), "--ros-args", "--params-file", str(params)],
                    cwd=runtime, env=env, start_new_session=True)
            print(f"预览地址：http://127.0.0.1:{args.port}（尚未开始录制）", flush=True)
            print("点击开始录制；点击停止并保存，或到两分钟自动停止。Ctrl+C 关闭程序。", flush=True)
            while rclpy.ok():
                rclpy.spin_once(recorder, timeout_sec=0.05)
                recorder.status()
                if camera is not None and camera.poll() is not None:
                    raise RuntimeError("相机进程退出，请检查连接或是否被其他程序占用")
        except KeyboardInterrupt:
            pass
        finally:
            if server is not None:
                server.shutdown()
                server.server_close()
            recorder.stop_recording()
            if recorder.worker is not None:
                recorder.worker.join()
            if camera is not None and camera.poll() is None:
                camera.send_signal(signal.SIGINT)
                try:
                    camera.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    camera.terminate()
                    try:
                        camera.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        camera.kill()
                        camera.wait()
            recorder.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    main()
