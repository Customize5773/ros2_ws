"""http_camera_bridge_logic — logika murni (tanpa rclpy) http_camera_bridge,
mengikuti pola qr_logic.py/gui_bridge_logic.py agar testable headless.

Berisi konversi sensor_msgs/Image -> ndarray BGR, penyimpanan frame JPEG
terbaru thread-safe, dan factory HTTP handler MJPEG. Lihat http_camera_bridge.py
(node) utk penjelasan kenapa jalur ini ada (bridge kamera Gazebo -> HTTP MJPEG
yang bisa dibaca cv2.VideoCapture GUI-ROV via source='rtsp').
"""

import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

_BOUNDARY = 'frame'


def to_bgr(height, width, encoding, data):
    """sensor_msgs/Image (height,width,encoding,data) -> ndarray BGR (utk
    cv2.imencode). Manual (bukan cv_bridge -- rusak akibat mismatch ABI
    numpy 1.x/2.x di environment ini)."""
    arr = np.frombuffer(data, dtype=np.uint8).reshape(height, width, -1)
    if encoding == 'rgb8':
        return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    if encoding == 'bgr8':
        return arr
    if encoding == 'mono8':
        return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
    raise ValueError(f'encoding tak didukung: {encoding}')


class FrameStore:
    """JPEG frame terbaru per kamera, thread-safe (ROS callback thread menulis,
    handler HTTP thread membaca)."""

    def __init__(self):
        self._lock = threading.Lock()
        self._jpeg = {}   # nama kamera -> bytes JPEG terbaru

    def update(self, name, jpeg_bytes):
        with self._lock:
            self._jpeg[name] = jpeg_bytes

    def get(self, name):
        with self._lock:
            return self._jpeg.get(name)


def make_handler(store: FrameStore, stream_hz: float, routes: dict):
    """`routes`: path HTTP (mis. '/cam_bottom') -> nama kamera di FrameStore."""

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass   # senyapkan access log bawaan -- bising, tak berguna di sini

        def do_GET(self):
            name = routes.get(self.path)
            if name is None:
                self.send_response(404)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header('Content-Type',
                              f'multipart/x-mixed-replace; boundary={_BOUNDARY}')
            self.end_headers()
            period = 1.0 / max(1.0, stream_hz)
            try:
                while True:
                    jpeg = store.get(name)
                    if jpeg is not None:
                        self.wfile.write(f'--{_BOUNDARY}\r\n'.encode())
                        self.wfile.write(b'Content-Type: image/jpeg\r\n')
                        self.wfile.write(f'Content-Length: {len(jpeg)}\r\n\r\n'.encode())
                        self.wfile.write(jpeg)
                        self.wfile.write(b'\r\n')
                    time.sleep(period)
            except (BrokenPipeError, ConnectionResetError):
                pass   # klien (cv2.VideoCapture) tutup koneksi -- normal saat stop()
    return Handler


def make_server(store: FrameStore, stream_hz: float, routes: dict, port: int):
    return ThreadingHTTPServer(('0.0.0.0', port), make_handler(store, stream_hz, routes))
