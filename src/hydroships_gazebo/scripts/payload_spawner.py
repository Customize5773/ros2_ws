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
from geometry_msgs.msg import PointStamped


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
          <ixx>3.0e-4</ixx><iyy>3.0e-4</iyy><izz>1.5e-4</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="body_collision">
        <geometry><box><size>0.05 0.006 0.10</size></box></geometry>
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

        self.pub_pose = self.create_publisher(PointStamped, '/hydroships/payload_pose', 10)
        self._spawn_delay = float(self.get_parameter('spawn_delay').value)
        self._t0 = self._now()
        self._done = False
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

        sdf = PAYLOAD_SDF_TEMPLATE.format(x=x, y=y, z=z, letter=letter)
        tmp = None
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
                self.get_logger().info('Payload QR=%s spawned OK' % letter)
        except Exception as e:  # noqa: BLE001 — jangan matikan node bila spawn gagal
            self.get_logger().warn('spawn exception (FSM pakai default): %s' % e)
        finally:
            if tmp and os.path.exists(tmp):
                os.unlink(tmp)

        # Publikasi posisi (walau spawn gagal, FSM tetap dapat target eksplisit).
        ps = PointStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = 'world'
        ps.point.x = float(x)
        ps.point.y = float(y)
        ps.point.z = float(z)
        self.pub_pose.publish(ps)

    def _tick(self):
        if not self._done and (self._now() - self._t0) >= self._spawn_delay:
            self._spawn()


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
