"""camera_dropout_injector — P2-B/R-8: suntik dropout frame ke image kamera (opsional).

Gazebo dibridge ke /hydroships/camera_{front,bottom}/image_raw_gt (lihat
bridge.yaml); node ini relay ke image_raw, drop frame acak kalau
camera_dropout:=true. Default false = passthrough identik. Frame yang di-drop
TIDAK diteruskan sama sekali (bukan republish frame lama) supaya freshness-check
yang sudah ada (qr_max_age, latch armed di gripper_logic) benar-benar teruji.

Kontrak topic:
    /hydroships/camera_{front,bottom}/image_raw_gt (sensor_msgs/Image) -> masuk
    /hydroships/camera_{front,bottom}/image_raw     (sensor_msgs/Image) -> keluar
"""

import random

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from hydroships_control.camera_dropout_logic import should_drop

CAMERAS = ('front', 'bottom')


class CameraDropoutInjector(Node):

    def __init__(self):
        super().__init__('camera_dropout_injector')

        def p(name, default):
            self.declare_parameter(name, default)

        def g(name):
            return self.get_parameter(name).value

        p('camera_dropout', False)   # aktifkan dropout (false = passthrough)
        p('drop_prob', 0.05)         # peluang drop per frame
        p('dropout_seed', 0)         # 0 = acak penuh; isi utk reproducible

        self.enabled = bool(g('camera_dropout'))
        self.drop_prob = float(g('drop_prob'))
        seed = int(g('dropout_seed'))
        self.rng = random.Random(seed) if seed else random.Random()

        self._pubs = {}
        for cam in CAMERAS:
            self._pubs[cam] = self.create_publisher(
                Image, '/hydroships/camera_%s/image_raw' % cam, 5)
            self.create_subscription(
                Image, '/hydroships/camera_%s/image_raw_gt' % cam,
                lambda msg, c=cam: self._on_image(msg, c), 5)

        self.get_logger().info(
            'camera_dropout_injector siap (dropout=%s drop_prob=%.3f)'
            % (self.enabled, self.drop_prob))

    def _on_image(self, msg, cam):
        if self.enabled and should_drop(self.rng, self.drop_prob):
            return
        self._pubs[cam].publish(msg)


def main():
    rclpy.init()
    node = CameraDropoutInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
