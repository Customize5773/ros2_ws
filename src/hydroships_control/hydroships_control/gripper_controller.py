"""gripper_controller — node manipulator ROV (rancang ulang M5, DetachableJoint).

Menerima perintah semantik open/close (kontrak GUI/autonomy dipertahankan) lalu:
  * menggerakkan DUA jari kosmetik opposing (Float64 -> JointPositionController gz;
    nilai identik ke kiri & kanan, mirroring dari tanda axis di URDF), dan
  * memicu **gz-sim DetachableJoint** attach/detach (std_msgs/Empty -> gz.msgs.Empty
    via bridge) untuk grasp fisik yang andal (bukan gesekan jari versi lama).

Attach hanya terjadi saat "close" DAN ROV berada di atas payload dalam jangkauan
aman (dinilai dari /hydroships/qr_offset). Logika keputusan ada di gripper_logic
(murni, teruji headless).

Kontrak topic (lihat docs/ARCHITECTURE.md — tak mengubah interface lama yg dipakai):
    /hydroships/gripper/command   (std_msgs/String  "open"|"close")   -> masuk
    /hydroships/qr_offset         (geometry_msgs/PointStamped)         -> masuk
    /hydroships/gripper_left/cmd  (std_msgs/Float64, rad)              -> keluar (bridge->gz)
    /hydroships/gripper_right/cmd (std_msgs/Float64, rad)              -> keluar (bridge->gz)
    /hydroships/gripper/attach    (std_msgs/Empty)                     -> keluar (bridge->gz)
    /hydroships/gripper/detach    (std_msgs/Empty)                     -> keluar (bridge->gz)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import String, Float64, Empty
from geometry_msgs.msg import PointStamped

from hydroships_control.gripper_logic import GripperLogic
# Geometri yang SAMA dipakai mission_fsm untuk membidik APPROACH_QR. Wajib satu
# sumber: kalau gerbang attach di sini dan kriteria centering FSM memakai acuan
# berbeda, keduanya bisa jadi mustahil dipenuhi bersamaan (itulah bug M5).
from hydroships_control.qr_logic import qr_ey_target


class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')
        p = self.declare_parameter
        p('max_offset', 0.30)       # |offset x/y| maks agar "di atas payload"
        p('min_size', 0.12)         # ukuran-tampak QR min (proxy dekat)
        p('offset_timeout', 1.5)    # umur maks qr_offset (s)
        # Kamera sumber qr_offset yang dipakai gerbang attach. HARUS sama dengan
        # filter mission_fsm (camera_bottom_link) — lihat _on_offset().
        p('offset_frame', 'camera_bottom_link')
        # Geometri untuk qr_ey_target — HARUS sama dengan default mission_fsm.
        p('cam_gripper_dx', 0.16)     # m, gripper_base di depan kamera bawah
        p('qr_floor_z', -0.894)       # m, ketinggian bidang QR di dunia
        p('cam_bottom_dz', 0.18)      # m, kamera bawah di bawah base_link
        p('cam_vfov_half_tan', 0.6293)
        p('ey_target_max', 0.8)
        p('jaw_open', 0.35)         # sudut jari terbuka (rad; <= upper limit URDF 0.5)
        p('jaw_close', 0.0)         # sudut jari menutup (rad)
        # Auto-detach startup kini DIPICU TOPIK /hydroships/payload/spawned (dari
        # payload_spawner) — detach terjadi SETELAH payload muncul di dunia, bukan
        # timer buta. startup_detach_fallback = jaring pengaman bila spawner tak ada
        # / topik hilang: detach paksa setelah delay ini (dibuat cukup PANJANG agar
        # tak mendahului spawn payload; sebelumnya 1.5s memicu detach sebelum payload
        # ada -> Fortress lalu auto-attach payload saat load -> payload nempel salah).
        p('startup_detach_fallback', 8.0)   # s (was startup_detach_delay=1.5)
        g = lambda n: self.get_parameter(n).value
        self.offset_frame = str(g('offset_frame'))
        self._ey_geom = (float(g('cam_gripper_dx')), float(g('qr_floor_z')),
                         float(g('cam_bottom_dz')), float(g('cam_vfov_half_tan')),
                         float(g('ey_target_max')))
        # Kedalaman terakhir; None = belum ada -> ey_target 0.0 (perilaku lama).
        self._depth = None

        self.logic = GripperLogic(
            max_offset=float(g('max_offset')), min_size=float(g('min_size')),
            offset_timeout=float(g('offset_timeout')),
            jaw_open=float(g('jaw_open')), jaw_close=float(g('jaw_close')))

        self.pub_jaw_left = self.create_publisher(Float64, '/hydroships/gripper_left/cmd', 10)
        self.pub_jaw_right = self.create_publisher(Float64, '/hydroships/gripper_right/cmd', 10)
        self.pub_attach = self.create_publisher(Empty, '/hydroships/gripper/attach', 10)
        self.pub_detach = self.create_publisher(Empty, '/hydroships/gripper/detach', 10)
        self.create_subscription(String, '/hydroships/gripper/command', self._on_cmd, 10)
        self.create_subscription(PointStamped, '/hydroships/qr_offset', self._on_offset, 10)
        self.create_subscription(Float64, '/hydroships/depth', self._on_depth, 10)

        # Terbitkan target jari berkala (2 Hz) agar tak hilang bila bridge/gz belum
        # siap saat publish awal — sama pola gripper lama.
        self._timer = self.create_timer(0.5, self._apply_jaw)

        # AUTO-DETACH STARTUP: gz-sim Fortress selalu attach DetachableJoint saat
        # model 'payload' LOAD (payload nge-lock ke ROV begitu spawn). Detach harus
        # terjadi SETELAH payload ada — dipicu topik /hydroships/payload/spawned yg
        # diterbitkan payload_spawner sesudah `ros_gz_sim create` sukses. QoS latched
        # (transient_local) agar sinyal tetap tertangkap walau terbit sebelum sub ini
        # terhubung. Timer fallback (delay panjang) hanya jaring pengaman bila spawner
        # tak jalan. Lihat gripper_logic.startup_detach & PROBLEM.md.
        self._did_startup_detach = False
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            Empty, '/hydroships/payload/spawned', self._on_payload_spawned, latched)
        self._startup_timer = self.create_timer(
            float(g('startup_detach_fallback')), self._startup_detach_fallback)
        self.get_logger().info('gripper_controller siap (DetachableJoint; detach saat payload spawn)')

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_offset(self, msg: PointStamped):
        # qr_detector menerbitkan KEDUA kamera ke topic yang sama, dibedakan
        # hanya lewat frame_id. Gerbang keamanan attach harus dinilai dari
        # kamera BAWAH: gripper_base sejajar pandangan kamera bawah (offset
        # tetap cam_gripper_dx=0.16 m, yang sudah dikoreksi mission_fsm), dan
        # mission_fsm juga memfilter ke frame yang sama (mission_fsm.py:466).
        # Tanpa filter ini, gerbang dinilai dari kamera mana pun yang kebetulan
        # terbit terakhir — biasanya kamera depan, yang melihat payload dengan
        # sudut lebar (terukur ex=0.90 ey=0.75 saat ROV justru terpusat rapi,
        # gripper_err=0.032 m) sehingga is_safe() selalu False dan attach tak
        # pernah terpicu. Lihat docs/STATUS.md M5.
        if msg.header.frame_id != self.offset_frame:
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # Bila stamp kosong (0), pakai jam node agar tetap dianggap segar.
        if stamp <= 0.0:
            stamp = self._now()
        ey_target = 0.0 if self._depth is None else qr_ey_target(
            self._depth, *self._ey_geom)
        self.logic.update_offset(msg.point.x, msg.point.y, msg.point.z, stamp,
                                 ey_target)

    def _on_depth(self, msg: Float64):
        self._depth = float(msg.data)

    def _apply_jaw(self):
        # Nilai sama untuk kedua jari: arah tutup dibedakan oleh tanda axis di URDF.
        m = Float64(); m.data = float(self.logic.jaw_target)
        self.pub_jaw_left.publish(m)
        self.pub_jaw_right.publish(m)

    def _on_payload_spawned(self, _msg: Empty):
        # Payload sudah muncul di dunia (dari payload_spawner) -> lepas attach bawaan
        # gz sekarang, dgn urutan benar (payload ada dulu, baru detach).
        self._do_startup_detach('payload spawn terdeteksi')

    def _startup_detach_fallback(self):
        # Jaring pengaman: bila topik spawned tak pernah tiba (spawner tak jalan),
        # tetap lepas attach bawaan agar payload tak nempel salah.
        self._startup_timer.cancel()
        self._do_startup_detach('fallback timer (topik spawned tak tiba)')

    def _do_startup_detach(self, trigger):
        # Idempoten: hanya sekali. Batalkan timer fallback bila masih aktif.
        if self._did_startup_detach:
            return
        self._did_startup_detach = True
        try:
            self._startup_timer.cancel()
        except Exception:
            pass
        action = self.logic.startup_detach()
        self._apply_jaw()
        self.pub_detach.publish(Empty())
        self.get_logger().info('gripper %s: %s [pemicu: %s]'
                               % (action['state'], action['reason'], trigger))

    def _on_cmd(self, msg: String):
        action = self.logic.on_command(msg.data, self._now())
        if action is None:
            self.get_logger().warn('perintah gripper tak dikenal: %r' % msg.data)
            return
        self._apply_jaw()
        if action['joint'] == 'attach':
            self.pub_attach.publish(Empty())
        elif action['joint'] == 'detach':
            self.pub_detach.publish(Empty())
        self.get_logger().info('gripper %s: %s' % (action['state'], action['reason']))


def main(args=None):
    rclpy.init(args=args)
    node = GripperController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
