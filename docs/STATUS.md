# STATUS — Progres Milestone HYDROships (KKI 2026)

Ringkasan **status terkini** tiap milestone. Riwayat kronologis lengkap (termasuk
keputusan yang sudah dibatalkan/diganti) ada di [CHANGELOG.md](CHANGELOG.md).

> ## ⚠️ Blocker utama: tidak ada environment ROS2/Gazebo di mesin dev
>
> Mesin pengembangan saat ini **tidak memiliki ROS2 + Gazebo (dan/atau GPU/EGL untuk
> render kamera headless)**. Akibatnya **semua klaim "runtime" belum bisa diverifikasi
> di sini** — kode & logika murni sudah lolos uji headless (`pytest`), tetapi bukti
> perilaku nyata di sim (render kamera, decode QR, grasp DetachableJoint, servo hook,
> GUI live) **menunggu satu kali run sim di mesin ber-GPU/Gazebo**. Item yang bergantung
> pada blocker ini ditandai **🧪 (kode ada, verifikasi runtime tertunda)** di bawah, dan
> tidak diulang baris-per-baris. Daftar uji berprioritas: lihat
> [VERIFICATION-CHECKLIST.md](VERIFICATION-CHECKLIST.md).

Legenda: ✅ jalan & terverifikasi di sim · 🧪 kode ada, verifikasi runtime tertunda
(lihat blocker) · ⏳ direncanakan/menyusul · OPEN gap desain/hardware (bukan sekadar env).

| Milestone | Status | Kondisi sekarang |
|-----------|--------|------------------|
| **M1** — Kendali dasar & thruster allocation | ✅ | Wrench `/hydroships/cmd_vel` → allocator (damped pinv) → 6 thruster; odom umpan balik. Dulu keliru ✅ (thrust tak nyambung) — sudah diperbaiki (topic namespace + graded buoyancy + frame thruster). |
| **M2** — Stabilizer PID (depth/heading hold) | ✅ | `stabilizer` PID depth & heading menulis wrench; `use_sim_time` diperbaiki. Test PID lolos. |
| **M3** — Sensor & persepsi | 🧪 | `depth_publisher`, `qr_detector` (+`qr_offset`), bridge `camera_info` **kode selesai**. **Keterbacaan QR kamera bawah TERBUKTI runtime (RESOLVED)** — root cause bug = FRAMING (kamera terlalu dekat → QR ter-crop), diperbaiki `scan_depth 0.62→0.46`; misi baca `A` otomatis & lanjut ke GRAB (lihat [CHANGELOG](CHANGELOG.md)). Aset `qr_B/C/D.png` kini ada. `camera_info` mengalir. Payload QR kini **di-spawn RANDOM** (A/B/C/D + posisi acak dalam bounds arena) oleh node `payload_spawner` tiap launch — huruf/posisi bisa dipaksa via launch arg `qr_letter`/`payload_x`/`payload_y`; FSM baca posisi via `/hydroships/payload_pose` (latched). **APPROACH_QR diperkuat** agar tak gagal baca QR: lampu `payload_fill` diperluas (range 3.0 m) menutupi seluruh area spawn, guard odom + timeout navigasi (`t_nav_qr`) + recovery, dan visual servo centering dari `/hydroships/qr_offset` (nudge target agar QR ke tengah frame; tanda sumbu **VERIFY** runtime). **VERIFY** tersisa: baca sisi B/C/D di sim & tuning `qr_offset` servo end-to-end. Kalibrasi ke kamera fisik = **OPEN** (gap hardware). |
| **M4** — Arena / world | ✅ | `worlds/kki_arena.sdf` dibangun (QR + hook). Payload **tidak lagi inline di world** — di-spawn runtime oleh `payload_spawner` (huruf QR random A/B/C/D, posisi via arg/random); `payload_fill` light tetap. Pemetaan label hook A–D & geometri hook Ø25 mm masih **VERIFY** (aproksimasi, perlu validasi arena nyata). |
| **M5** — Manipulator | 🧪 | Node `gripper_controller` + `gripper_logic` aktif; body gripper di URDF **dipasang di muka depan ROV** (joint `0.18 0 0`, body 0.10×0.10×0.06 m, massa 0.12 kg) menjorok ke depan (+X). Integrasi `_grip` ke `mission_fsm` **dipulihkan** (PR #14 merge menghapusnya, sekarang di-cherry-pick kembali dari `4fbc6f8`). Startup-detach kini **dipicu topik `/hydroships/payload/spawned`** (dari `payload_spawner`) — payload lepas SETELAH spawn (bukan timer 1.5 s yg mendahului payload → cegah auto-attach salah); attach hanya di GRAB saat `qr_offset` aman. **Grasp fisik & urutan spawn/attach belum diuji di sim/kolam.** |
| **M6** — Autonomy (FSM misi) | 🧪 | `mission_fsm` memiliki integrasi lengkap gripper (`_grip`) + visual servo PD `APPROACH_HOOK` (`hook_logic.hook_servo`) setelah dipulihkan dari regresi merge. `done_hooks` loop DIVE untuk <4 hook, heading `APPROACH_HOOK` di-hold ke wall. **NAV_WALL/HANG kini AMAN dinding**: target standoff `wall_face(2.5)-wall_standoff(0.45)=2.05 m`, SOFT-STOP dorong menjauh bila clearance < standoff, HANG hold di standoff (tanpa gerak nabrak lama), transisi butuh settle (dist<0.25 & |v|<0.10). QR fixes (`scan_depth=0.46`, `t_scan=60`, `start_wall`) tetap terjaga. Tuning gain/standoff & sim run penuh **tertunda verifikasi runtime**. |
| **M7** — Integrasi GUI tim | 🧪 | GUI-ROV bukan ROS2 (UDP-JSON/MAVLink) → adapter `gui_bridge` + `hook_detector` **dibuat**. **Belum diuji live end-to-end**; gain/tanda/port = estimasi. Lihat [GUI-INTEGRATION.md](GUI-INTEGRATION.md). |

## Manipulator (M5) — status final (satu-satunya versi aktif)

Untuk menghindari kebingungan dari riwayat lama (gripper 2-jari → dihapus → dirancang
ulang), berikut **satu-satunya desain yang aktif sekarang**:

- **Grasp sesungguhnya** = plugin gz-sim **`DetachableJoint`** yang menyambung kaku
  `gripper_base` (di muka depan ROV, +X) ↔ model `payload`. Attach/detach via topik
  `/hydroships/gripper/attach` & `/hydroships/gripper/detach` (`std_msgs/Empty`).
- **Jari `gripper_jaw`** = revolute 1-DOF **kosmetik** (indikator visual buka-tutup saja),
  dikendalikan lewat `/hydroships/gripper_jaw/cmd` (`Float64`).
- Node **`gripper_controller`** menerima `/hydroships/gripper/command` (`String` "open"/"close").
  Attach hanya dipicu saat "close" **dan** ROV berada di atas payload dalam jangkauan aman
  (dinilai dari `/hydroships/qr_offset`). Satu detach otomatis saat startup membatalkan
  attach bawaan Fortress.
- **Bukan** gripper 2-jari (versi lama, dibatalkan) dan **bukan** hook servo. `hook_detector`
  adalah subsistem terpisah untuk mendeteksi hook arena (state `APPROACH_HOOK`).

Detail keputusan & riwayat pembatalan: [CHANGELOG.md](CHANGELOG.md).
