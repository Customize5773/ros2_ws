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
| **M3** — Sensor & persepsi | 🧪 | `depth_publisher`, `qr_detector` (+`qr_offset`), bridge `camera_info` **kode selesai**. **Keterbacaan QR kamera bawah TERBUKTI runtime (RESOLVED)** — root cause bug = FRAMING (kamera terlalu dekat → QR ter-crop), diperbaiki `scan_depth 0.62→0.46`; misi baca `A` otomatis & lanjut ke GRAB (lihat [CHANGELOG](CHANGELOG.md)). Aset `qr_B/C/D.png` kini ada. `camera_info` mengalir. **VERIFY** tersisa: baca sisi B/C/D di sim & `qr_offset` untuk servo. Kalibrasi ke kamera fisik = **OPEN** (gap hardware). |
| **M4** — Arena / world | ✅ | `worlds/kki_arena.sdf` dibangun (payload + QR + hook). Pemetaan label hook A–D & geometri hook Ø25 mm masih **VERIFY** (aproksimasi, perlu validasi arena nyata). |
| **M5** — Manipulator | 🧪 | Rancang ulang **DetachableJoint** (grasp kaku ROV↔payload) + jari `gripper_jaw` 1-DOF **kosmetik**; node `gripper_controller` aktif. **Grasp fisik belum diuji di sim/kolam.** |
| **M6** — Autonomy (FSM misi) | 🧪 | `mission_fsm` jalan end-to-end setelah fix fisika; `APPROACH_HOOK` upgrade ke **servo PD holonomik**. Tuning gain/timeout & sim run penuh **tertunda verifikasi runtime**. |
| **M7** — Integrasi GUI tim | 🧪 | GUI-ROV bukan ROS2 (UDP-JSON/MAVLink) → adapter `gui_bridge` + `hook_detector` **dibuat**. **Belum diuji live end-to-end**; gain/tanda/port = estimasi. Lihat [GUI-INTEGRATION.md](GUI-INTEGRATION.md). |

## Manipulator (M5) — status final (satu-satunya versi aktif)

Untuk menghindari kebingungan dari riwayat lama (gripper 2-jari → dihapus → dirancang
ulang), berikut **satu-satunya desain yang aktif sekarang**:

- **Grasp sesungguhnya** = plugin gz-sim **`DetachableJoint`** yang menyambung kaku
  `gripper_base` (di perut ROV) ↔ model `payload`. Attach/detach via topik
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
