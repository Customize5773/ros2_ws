"""http_camera_bridge — jembatani topic kamera Gazebo -> stream MJPEG-over-HTTP
utk dikonsumsi `autonomy/vision/qr_detect.py` (GUI-ROV) via source='rtsp'
(cv2.VideoCapture menerima URL http:// MJPEG sama seperti rtsp://, backend
FFmpeg yang sama -- lihat docstring VisionPipeline: "'rtsp': stream ... via
RTSP/HTTP"). Dipilih drpd RTSP sungguhan krn tak butuh dependensi baru
(mediamtx/gst-rtsp-server tak tersedia di lingkungan ini) atau cv_bridge
(rusak akibat mismatch ABI numpy 1.x/2.x di environment ini) -- konversi
Image->numpy manual, lihat http_camera_bridge_logic.to_bgr().

Subscribe topic YANG SAMA dipakai qr_detector/hook_detector bawaan ros2_ws
(`/hydroships/camera_bottom/image_raw`, `camera_front/image_raw`) -- jadi
efek `camera_dropout_injector` (kalau dinyalakan) otomatis ikut, tanpa
wiring tambahan.

Pakai (lihat autonomy/GAZEBO_BACKEND.md utk contoh penuh):
    ros2 run hydroships_control http_camera_bridge
    # lalu di GUI-ROV:
    python3 fsm/mission5.py --vision rtsp \\
        --bottom-url http://127.0.0.1:8090/cam_bottom \\
        --wall-url   http://127.0.0.1:8090/cam_front

Parameter:
  bottom_topic (/hydroships/camera_bottom/image_raw)
  front_topic  (/hydroships/camera_front/image_raw)
  http_port    (8090)
  jpeg_quality (80)
  stream_hz    (10.0) : laju kirim frame per klien (independen dari laju topic)
"""

import threading

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from hydroships_control.http_camera_bridge_logic import FrameStore, make_server, to_bgr


class HttpCameraBridge(Node):
    def __init__(self):
        super().__init__('http_camera_bridge')
        p = self.declare_parameter
        p('bottom_topic', '/hydroships/camera_bottom/image_raw')
        p('front_topic', '/hydroships/camera_front/image_raw')
        p('http_port', 8090)
        p('jpeg_quality', 80)
        p('stream_hz', 10.0)
        g = lambda n: self.get_parameter(n).value

        self._quality = int(g('jpeg_quality'))
        self._store = FrameStore()

        self.create_subscription(Image, g('bottom_topic'),
                                  lambda m: self._on_image('cam_bottom', m), 10)
        self.create_subscription(Image, g('front_topic'),
                                  lambda m: self._on_image('cam_front', m), 10)

        routes = {'/cam_bottom': 'cam_bottom', '/cam_front': 'cam_front'}
        self._httpd = make_server(self._store, float(g('stream_hz')), routes,
                                   int(g('http_port')))
        self._http_thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._http_thread.start()
        self.get_logger().info(
            'http_camera_bridge siap -- http://0.0.0.0:%d/cam_bottom & /cam_front'
            % int(g('http_port')))

    def _on_image(self, name, msg: Image):
        try:
            bgr = to_bgr(msg.height, msg.width, msg.encoding, msg.data)
        except ValueError as e:
            self.get_logger().warn(f'{name}: {e}', throttle_duration_sec=5.0)
            return
        ok, buf = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, self._quality])
        if ok:
            self._store.update(name, buf.tobytes())

    def destroy_node(self):
        self._httpd.shutdown()
        super().destroy_node()


def main():
    rclpy.init()
    node = HttpCameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
