# Arsitektur Simulasi HYDROships (KKI 2026)

Stack: **ROS 2 Humble + Gazebo Fortress** (gz-sim 6) + `ros_gz` bridge.

## Diagram aliran (Milestone 1–2)

```
  MODE MANUAL (M1):
    teleop_keyboard ──► /hydroships/cmd_vel (Twist = wrench 6-DOF)

  MODE STABILIZED (M2):
    teleop_stabilized ─► /hydroships/manual/cmd      (Fx, Fy)
                      └► /hydroships/setpoint/depth   (Float64)
                      └► /hydroships/setpoint/heading (Float64)
                                   │
                              stabilizer  (PID depth-hold + heading-hold)
                                   │  Fz,Mz dari PID; Fx,Fy,Mx,My pass-through
                                   ▼
                        /hydroships/cmd_vel (Twist = wrench 6-DOF)
                                   │
                                   ▼
                       thruster_allocator  (f = pinv(TAM)·wrench)
                                   │
                    /hydroships/thruster_{1..6}/thrust (Float64, N)
                                   │
                             ros_gz_bridge
                                   │
                                   ▼
                 Gazebo Fortress ── plugin Thruster ×6
                        │  plugin Hydrodynamics (drag+added mass)
                        │  plugin Buoyancy (world)
                        ▼
                 /model/hydroships/odometry ─► bridge ─► /hydroships/odom
                                   (Odometry: umpan balik z & yaw ke stabilizer)
```

## Kontrak interface topic (untuk GUI / GCS tim)

GUI tim membaca/menulis topic ROS 2 standar berikut. Milestone 1 baru
menyediakan blok kendali & odometry; kamera/QR/kedalaman menyusul (M3).

| Topic | Tipe | Arah | Status |
|-------|------|------|--------|
| `/hydroships/cmd_vel` | `geometry_msgs/Twist` | teleop/stabilizer → allocator | ✅ M1 (wrench body: linear=gaya N, angular=torsi N·m) |
| `/hydroships/thruster_{1..6}/thrust` | `std_msgs/Float64` | allocator → sim | ✅ M1 |
| `/hydroships/odom` | `nav_msgs/Odometry` | sim → GUI/stabilizer | ✅ M1 |
| `/clock` | `rosgraph_msgs/Clock` | sim → semua | ✅ M1 |
| `/hydroships/manual/cmd` | `geometry_msgs/Twist` | GUI/pilot → stabilizer | ✅ M2 (horizontal: Fx, Fy) |
| `/hydroships/setpoint/depth` | `std_msgs/Float64` | GUI/pilot → stabilizer | ✅ M2 (target kedalaman, m) |
| `/hydroships/setpoint/heading` | `std_msgs/Float64` | GUI/pilot → stabilizer | ✅ M2 (target yaw, rad) |
| `/hydroships/depth` | `std_msgs/Float64` | sim → GUI | ⏳ M3 |
| `/hydroships/camera_front/image_raw` | `sensor_msgs/Image` | sim → GUI | ⏳ M3 |
| `/hydroships/camera_bottom/image_raw` | `sensor_msgs/Image` | sim → GUI | ⏳ M3 |
| `/hydroships/qr_result` | `std_msgs/String` | node QR → GUI | ⏳ M3 |

> Saat integrasi GUI (M7): jika GUI tim mengharapkan nama topic berbeda,
> cukup remap di launch (`--ros-args -r from:=to`) atau sesuaikan tabel ini —
> tipe pesannya sudah standar sehingga pemetaan mudah.

## Paket

| Paket | Isi |
|-------|-----|
| `hydroships_description` | URDF/xacro model + plugin gz (Thruster, Hydrodynamics, Odometry) |
| `hydroships_gazebo` | world kolam, `bridge.yaml`, `sim.launch.py` |
| `hydroships_control` | `thruster_allocator`, `teleop_keyboard` |
| `hydroships_bringup` | launch top-level `hydroships_sim.launch.py` |

## Keputusan desain

- **Fortress dipilih** karena plugin Buoyancy/Hydrodynamics/Thruster sudah bawaan
  → tidak perlu plugin fisika underwater custom.
- **Wrench di `/hydroships/cmd_vel`**: satu titik masuk untuk teleop maupun PID
  autonomy nanti, sehingga allocator tidak perlu tahu sumber perintah.
- **Watchdog di allocator**: bila perintah berhenti > 0,5 s, thruster dinolkan
  (aman bila teleop/GUI putus).
