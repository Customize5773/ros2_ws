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

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Empty


# Template SDF payload — SAMA PERSIS dgn definisi 'payload' di
# worlds/kki_arena.sdf (mesh body, collision, quiet-zone, QR pbr). {letter}
# memilih qr_A/B/C/D.png. {static}: spawn AWAL selalu 'true' — model bernama
# 'payload' otomatis di-attach gz-sim DetachableJoint (gripper_base<->payload)
# begitu entity muncul, TANPA syarat jarak. Bila dynamic & ROV jauh, koreksi
# constraint sesaat itu meledak -> ODE "aabbBound" assert & gz crash. Static
# kebal (badan static tak pernah digerakkan solver), jadi entity aman terbit
# di posisi arena manapun. Baru di-respawn dynamic ('static'='false') saat
# grasp SUNGGUHAN (ROV sudah dekat, lihat _make_dynamic/gripper_controller),
# sehingga saat itu auto-attach terjadi dgn offset kecil & aman.
PAYLOAD_SDF_TEMPLATE = '''<?xml version="1.0"?>
<sdf version="1.9">
  <model name="payload">
    <static>{static}</static>
    <pose>{x} {y} {z} 1.5708 0 0</pose>
    <link name="payload_link">
      <inertial>
        <mass>0.3</mass>
        <inertia>
          <ixx>3.0e-4</ixx><iyy>3.0e-4</iyy><izz>1.5e-4</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
       <collision name="body_collision">
         <geometry><box><size>0.05 0.020 0.10</size></box></geometry>
       </collision>
      <visual name="body">
        <geometry>
          <mesh><uri>model://hydroships_gazebo/media/payload_body.obj</uri></mesh>
        </geometry>
        <material>
          <ambient>0.75 0.76 0.80 1</ambient>
          <diffuse>0.82 0.83 0.86 1</diffuse>
        </material>
      </visual>
      <visual name="qr_quiet_zone">
        <pose>0 0.0006 0.04 0 0 0</pose>
        <geometry><plane><normal>0 1 0</normal><size>0.16 0.16</size></plane></geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <specular>0 0 0 1</specular>
          <emissive>1 1 1 1</emissive>
        </material>
      </visual>
      <visual name="qr">
        <pose>0 0.0012 0.04 0 0 0</pose>
        <geometry><plane><normal>0 1 0</normal><size>0.12 0.12</size></plane></geometry>
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
        p('payload_z', -0.894)      # m (di dasar kolam)
        p('spawn_delay', 4.0)       # s setelah node start (tunggu sim siap)
        p('randomize_pos', True)    # random posisi saat qr_letter kosong
        p('arena_x_min', 0.2)
        p('arena_x_max', 0.6)
        p('arena_y_min', -1.5)
        p('arena_y_max', 1.5)
        p('world_name', 'kki_arena')  # utk gz service remove saat respawn dynamic

        # QoS transient_local (latched): subscriber yg join belakangan (mis.
        # mission_fsm) tetap menerima pose terakhir walau spawn sudah lewat.
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_pose = self.create_publisher(PointStamped, '/hydroships/payload_pose', qos)
        # Sinyal "payload sudah muncul di dunia" -> gripper_controller lepas attach
        # bawaan gz SETELAH ini (urutan benar). Latched agar tak hilang bila terbit
        # sebelum subscriber terhubung.
        self.pub_spawned = self.create_publisher(Empty, '/hydroships/payload/spawned', qos)
        # Handshake grasp: gripper_controller minta payload jadi dynamic SESAAT
        # sebelum attach fisik (ROV sudah dekat/aman - lihat is_safe()); baru
        # setelah konfirmasi 'made_dynamic' gripper boleh publish attach_topic.
        self.pub_made_dynamic = self.create_publisher(
            Empty, '/hydroships/payload/made_dynamic', qos)
        self.create_subscription(
            Empty, '/hydroships/payload/request_dynamic', self._on_request_dynamic, 10)
        self._spawn_delay = float(self.get_parameter('spawn_delay').value)
        self._t0 = self._now()
        self._done = False
        self._is_dynamic = False
        self._letter = None
        self._pose = None           # (x, y, z) hasil spawn utk republish periodik
        self.create_timer(0.5, self._tick)
        self.get_logger().info('payload_spawner siap (spawn dalam %.1fs)' % self._spawn_delay)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _spawn(self):
        if self._done:
            return
        self._done = True
        g = lambda n: self.get_parameter(n).value

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
        self._letter = letter
        self._publish_pose()

        # Spawn AWAL selalu static=true — lihat catatan di PAYLOAD_SDF_TEMPLATE.
        spawned_ok = self._spawn_model(x, y, z, letter, static=True)
        self._is_dynamic = False

        # Beritahu gripper_controller HANYA bila model benar-benar muncul, agar
        # detach terjadi setelah payload ada (bukan sebelum). Bila create gagal,
        # tak ada payload -> tak ada auto-attach bawaan -> tak perlu sinyal detach.
        if spawned_ok:
            self.pub_spawned.publish(Empty())
            self.get_logger().info('Sinyal /hydroships/payload/spawned diterbitkan')

        # Pastikan pose ter-publish (idempoten; sudah dipublish di awal _spawn).
        self._publish_pose()

    def _spawn_model(self, x, y, z, letter, static):
        """Spawn (atau respawn) model 'payload' via ros_gz_sim create. Kembalikan
        True bila sukses. static=True/False -> field {static} template SDF."""
        sdf = PAYLOAD_SDF_TEMPLATE.format(
            x=x, y=y, z=z, letter=letter, static='true' if static else 'false')
        tmp = None
        ok = False
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
                'Spawn payload QR=%s pos=(%.2f, %.2f, %.2f) static=%s'
                % (letter, x, y, z, static))
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
            if result.returncode != 0:
                self.get_logger().warn('spawn gagal: %s' % result.stderr.strip())
            else:
                ok = True
                self.get_logger().info('Payload QR=%s spawned OK (static=%s)' % (letter, static))
        except Exception as e:  # noqa: BLE001 — jangan matikan node bila spawn gagal
            self.get_logger().warn('spawn exception: %s' % e)
        finally:
            if tmp and os.path.exists(tmp):
                os.unlink(tmp)
        return ok

    def _remove_payload(self):
        """Hapus entity 'payload' dari world via gz service (perlu utk toggle
        static->dynamic; gz-sim tak punya service ubah static entity langsung)."""
        world = str(self.get_parameter('world_name').value)
        cmd = [
            'gz', 'service', '-s', '/world/%s/remove' % world,
            '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '2000', '--req', 'name: "payload" type: MODEL',
        ]
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
            if result.returncode != 0:
                self.get_logger().warn('gz remove payload gagal: %s' % result.stderr.strip())
                return False
            return True
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn('gz remove payload exception: %s' % e)
            return False

    def _on_request_dynamic(self, _msg: Empty):
        """Dipicu gripper_controller SESAAT SEBELUM attach fisik (ROV sudah
        dekat & aman - is_safe()). Despawn payload static lalu respawn dynamic
        di pose yg sama; karena ROV dekat, auto-attach DetachableJoint yg
        terjadi begitu entity dynamic muncul aman (offset kecil, bukan lintas
        arena)."""
        if not self._done or self._pose is None:
            self.get_logger().warn('request_dynamic diabaikan: payload belum spawn')
            return
        if self._is_dynamic:
            # Sudah dynamic (mis. permintaan dobel) -> langsung konfirmasi.
            self.pub_made_dynamic.publish(Empty())
            return
        x, y, z = self._pose
        if not self._remove_payload():
            return
        if self._spawn_model(x, y, z, self._letter, static=False):
            self._is_dynamic = True
            self.pub_made_dynamic.publish(Empty())
            self.get_logger().info('Payload respawned dynamic utk grasp')
        else:
            self.get_logger().warn('respawn dynamic gagal - grasp dibatalkan')

    def _publish_pose(self):
        if self._pose is None:
            return
        ps = PointStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'world'
        ps.point.x, ps.point.y, ps.point.z = self._pose
        self.pub_pose.publish(ps)

    def _tick(self):
        if not self._done:
            if (self._now() - self._t0) >= self._spawn_delay:
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
