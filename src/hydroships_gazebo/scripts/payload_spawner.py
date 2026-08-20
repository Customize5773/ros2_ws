#!/usr/bin/env python3
"""payload_spawner — spawn model payload QR random (A/B/C/D) di Gazebo Fortress.

Memilih huruf QR & posisi secara random (atau via parameter), lalu spawn model
payload via `ros2 run ros_gz_sim create` dan publikasi posisinya ke
/hydroships/payload_pose agar mission_fsm bisa navigasi APPROACH_QR dinamis.
"""

import os
import random
import subprocess
import tempfile
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Empty, String


# Template SDF payload — SAMA PERSIS dgn definisi 'payload' di
# worlds/kki_arena.sdf (mesh body, collision, quiet-zone, QR pbr). {letter}
# memilih qr_A/B/C/D.png. Non-static agar bisa diangkat DetachableJoint gripper.
PAYLOAD_SDF_TEMPLATE = '''<?xml version="1.0"?>
<sdf version="1.9">
  <model name="payload">
    <static>false</static>
    <pose>{x} {y} {z} 1.5708 0 0</pose>
    <link name="payload_link">
      <inertial>
        <mass>0.3</mass>
        <inertia>
          <ixx>2.51e-4</ixx><iyy>3.13e-4</iyy><izz>6.34e-5</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <!-- Kolisi plat (frame LINK; saat dibawa: link x=samping, y=tebal/vertikal,
           z=memanjang ke depan):
           - body_collision : badan plat di BELAKANG lubang, top di bawah lingkaran
             tip (z 0.0808) supaya tip tak mentok badan.
           - slot_*         : dua sayap di sisi lubang (z 0.0865..0.10) — lorong
             tip menembus plat di sini.
           - bar_collision  : ekstensi PEKAT di DEPAN lubang (z 0.11..0.17) yang
             bersandar di palang bawah hook saat plat turun (palang hook di
             x +-0.0125, jadi butuh material padat di sana — sayap di x +-0.035
             saja TIDAK menyentuh palang dan plat jatuh menembus hook).
           Tip (silinder tegak r=0.0125 di lubang) menembus lewat celah
           z 0.075..0.11 (bawah body & atas bar_collision kosong) dan lorong
           slot, lalu plat bersandar stabil di palang. -->
      <collision name="body_collision">
        <pose>0 0.004 0.0325 0 0 0</pose>
        <geometry><box><size>0.05 0.02 0.065</size></box></geometry>
      </collision>
      <collision name="slot_left_collision">
        <pose>-0.045 -0.003 0.09325 0 0 0</pose>
        <geometry><box><size>0.008 0.006 0.0135</size></box></geometry>
      </collision>
      <collision name="slot_right_collision">
        <pose>0.045 -0.003 0.09325 0 0 0</pose>
        <geometry><box><size>0.008 0.006 0.0135</size></box></geometry>
      </collision>
      <collision name="bar_collision">
        <pose>0 0.004 0.1275 0 0 0</pose>
        <geometry><box><size>0.05 0.02 0.025</size></box></geometry>
      </collision>
      <visual name="body_{vsuf}">
        <geometry>
          <mesh><uri>model://hydroships_gazebo/media/payload_body.obj</uri></mesh>
        </geometry>
        <material>
          <ambient>0.75 0.76 0.80 1</ambient>
          <diffuse>0.82 0.83 0.86 1</diffuse>
        </material>
      </visual>
      <visual name="qr_quiet_zone_{vsuf}">
        <pose>0 0.0006 0.04 0 0 0</pose>
        <geometry><plane><normal>0 1 0</normal><size>0.12 0.12</size></plane></geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <specular>0 0 0 1</specular>
          <emissive>1 1 1 1</emissive>
        </material>
      </visual>
      <visual name="qr_{vsuf}">
        <pose>0 0.0012 0.04 0 0 0</pose>
        <geometry><plane><normal>0 1 0</normal><size>0.06 0.06</size></plane></geometry>
        <material>
          <diffuse>1 1 1 1</diffuse>
          <specular>0 0 0 1</specular>
          <pbr>
            <metal>
              <albedo_map>model://hydroships_gazebo/media/qr_{letter}.png</albedo_map>
              <emissive_map>model://hydroships_gazebo/media/qr_{letter}.png</emissive_map>
              <metalness>0.0</metalness>
              <roughness>1.0</roughness>
            </metal>
          </pbr>
        </material>
      </visual>
      <velocity_decay>
        <linear>1.0</linear>
        <angular>1.0</angular>
      </velocity_decay>
    </link>
  </model>
</sdf>
'''


class PayloadSpawner(Node):
    def __init__(self):
        super().__init__('payload_spawner')
        p = self.declare_parameter
        p('qr_letter', '')          # '' = random A/B/C/D
        p('payload_x', 0.4)         # m
        p('payload_y', 0.04)        # m
        p('payload_z', -0.90)       # m (tepat di lantai kolam, top floor z=-0.90)
        p('spawn_delay', 4.0)       # s setelah node start (tunggu sim siap)
        p('randomize_pos', True)    # random posisi saat qr_letter kosong
        p('arena_x_min', 0.2)
        p('arena_x_max', 0.6)
        p('arena_y_min', -1.5)
        p('arena_y_max', 1.5)

        # QoS transient_local (latched): subscriber yg join belakangan (mis.
        # mission_fsm) tetap menerima pose terakhir walau spawn sudah lewat.
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_pose = self.create_publisher(PointStamped, '/hydroships/payload_pose', qos)
        # Sinyal "payload sudah muncul di dunia" -> gripper_controller lepas attach
        # bawaan gz SETELAH ini (urutan benar). Latched agar tak hilang bila terbit
        # sebelum subscriber terhubung.
        self.pub_spawned = self.create_publisher(Empty, '/hydroships/payload/spawned', qos)
        self._spawn_delay = float(self.get_parameter('spawn_delay').value)
        self._t0 = self._now()
        self._spawned_initial = False
        self._pose = None           # (x, y, z) hasil spawn utk republish periodik
        # Sufiks nama VISUAL per spawn. Nama model/link harus tetap 'payload' /
        # 'payload_link' (DetachableJoint), tapi gz-sim Fortress CRASH (segfault
        # di render/sensors scene — "Another item already exists with name:
        # payload::payload_link::qr_quiet_zone_geom") bila model kedua dgn nama
        # visual SAMA dibuat sebelum scene render menghapus visual model lama.
        # Sufiks unik per spawn menghindari konflik nama itu sepenuhnya.
        self._spawn_seq = 0
        # Multi-payload: mission_fsm meminta payload BARU utk wall berikutnya
        # (std_msgs/String = huruf QR, mis. "D") setelah 1 hook selesai.
        self.create_subscription(String, '/hydroships/payload/spawn_next',
                                 self._on_spawn_next, 10)
        self.create_timer(0.5, self._tick)
        self.get_logger().info('payload_spawner siap (spawn dalam %.1fs)' % self._spawn_delay)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_spawn_next(self, msg: String):
        """Permintaan payload baru dari mission_fsm setelah 1 wall selesai.
        Buang payload lama (masih ada di dunia, nama model harus tetap 'payload'
        utk DetachableJoint) lalu spawn yg baru dgn huruf QR dari FSM."""
        letter = (msg.data or '').strip().upper()
        if letter not in ('A', 'B', 'C', 'D'):
            letter = random.choice(['A', 'B', 'C', 'D'])
        self.get_logger().info('Spawn payload BARU utk siklus berikutnya (QR=%s)' % letter)
        self._remove_payload()
        # Kasih waktu render/sensors scene menghapus visual model lama sebelum
        # spawn model baru (lihat catatan _spawn_seq). Tanpa jeda ini, walau
        # nama visual sudah unik, ada jendela di mana dua payload ada serentak.
        time.sleep(1.0)
        self._spawn(letter)

    def _remove_payload(self):
        """Hapus model 'payload' lama dari dunia (kalau masih ada) via service
        /world/kki_arena/remove. Gagal/tak ada -> non-fatal."""
        try:
            subprocess.run(
                ['ign', 'service', '-s', '/world/kki_arena/remove',
                 '--reqtype', 'ignition.msgs.Entity',
                 '--reptype', 'ignition.msgs.Boolean',
                 '--timeout', '5000', '--req', 'name: "payload" type: MODEL'],
                capture_output=True, text=True, timeout=10.0)
            self.get_logger().info('Payload lama dihapus dari dunia')
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn('remove payload lama gagal (non-fatal): %s' % e)

    def _spawn(self, letter=None):
        g = lambda n: self.get_parameter(n).value

        if letter is None:
            letter = str(g('qr_letter')).strip().upper()
        random_letter = letter not in ('A', 'B', 'C', 'D')
        if random_letter:
            letter = random.choice(['A', 'B', 'C', 'D'])

        x = float(g('payload_x'))
        y = float(g('payload_y'))
        z = float(g('payload_z'))
        # Randomize posisi dalam bounds arena hanya bila huruf random (mode acak
        # penuh) DAN randomize_pos aktif. Bila user set qr_letter eksplisit,
        # hormati payload_x/y yg diberikan.
        if random_letter and bool(g('randomize_pos')):
            x = random.uniform(float(g('arena_x_min')), float(g('arena_x_max')))
            y = random.uniform(float(g('arena_y_min')), float(g('arena_y_max')))

        # Publikasi pose SEGERA (posisi target sudah diketahui) sebelum subprocess
        # create yg bisa lambat — FSM butuh pose utk navigasi, tak perlu tunggu model
        # benar-benar muncul. (Republish periodik + latched di _tick sbg jaring.)
        self._pose = (float(x), float(y), float(z))
        self._publish_pose()

        self._spawn_seq += 1
        sdf = PAYLOAD_SDF_TEMPLATE.format(x=x, y=y, z=z, letter=letter,
                                          vsuf=self._spawn_seq)
        tmp = None
        spawned_ok = False
        try:
            with tempfile.NamedTemporaryFile(mode='w', suffix='.sdf', delete=False) as f:
                f.write(sdf)
                tmp = f.name
            # Roll 1.5708 (=pose SDF) agar QR menghadap ATAS (dibaca kamera bawah).
            # -x/-y/-z CLI meng-override translasi pose; -R pastikan orientasi benar.
            cmd = [
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-file', tmp, '-name', 'payload',
                '-x', str(x), '-y', str(y), '-z', str(z), '-R', '1.5708',
            ]
            self.get_logger().info(
                'Spawn payload QR=%s pos=(%.2f, %.2f, %.2f)' % (letter, x, y, z))
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
            if result.returncode != 0:
                self.get_logger().warn('spawn gagal (FSM pakai default): %s'
                                       % result.stderr.strip())
            else:
                spawned_ok = True
                self.get_logger().info('Payload QR=%s spawned OK' % letter)
        except Exception as e:  # noqa: BLE001 — jangan matikan node bila spawn gagal
            self.get_logger().warn('spawn exception (FSM pakai default): %s' % e)
        finally:
            if tmp and os.path.exists(tmp):
                os.unlink(tmp)

        # Beritahu gripper_controller HANYA bila model benar-benar muncul, agar
        # detach terjadi setelah payload ada (bukan sebelum). Bila create gagal,
        # tak ada payload -> tak ada auto-attach bawaan -> tak perlu sinyal detach.
        if spawned_ok:
            self.pub_spawned.publish(Empty())
            self.get_logger().info('Sinyal /hydroships/payload/spawned diterbitkan')

        # Pastikan pose ter-publish (idempoten; sudah dipublish di awal _spawn).
        self._publish_pose()

    def _publish_pose(self):
        if self._pose is None:
            return
        ps = PointStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'world'
        ps.point.x, ps.point.y, ps.point.z = self._pose
        self.pub_pose.publish(ps)

    def _tick(self):
        if not self._spawned_initial:
            if (self._now() - self._t0) >= self._spawn_delay:
                self._spawned_initial = True
                self._spawn()
        else:
            # Republish periodik (di atas latching) sbg jaring pengaman late-join.
            self._publish_pose()


def main(args=None):
    rclpy.init(args=args)
    node = PayloadSpawner()
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
