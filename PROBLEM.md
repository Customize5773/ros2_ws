# PROBLEM.md — Catatan Masalah & Verifikasi Tertunda (HYDROships ros2_ws)

Dokumen ini mengumpulkan masalah diketahui, keputusan sementara, dan hal yang **masih
perlu diverifikasi di kolam/hardware asli**. Dijadikan catatan akhir saat semua milestone
selesai. Format: `[status]` OPEN / VERIFY / RESOLVED.

> Konteks lintas-repo: dokumen arsitektur & roadmap referensi ada di repo lama
> **ROV-GAZEBO-SIMULATION** (GitHub), bukan di workspace ini.

---

## Arena (M4) — sudah dibangun
- `[VERIFY]` Pemetaan label hook **A/B/C/D → sisi kolam** masih sementara (A=−Y, B=+Y,
  C=+X, D=−X di `worlds/kki_arena.sdf`). Pengacakan posisi ("Posisi A,B,C,D diacak")
  belum diimplementasi — menyusul bersama logika tugas.
- `[VERIFY]` Geometri hook = aproksimasi silinder J (Ø25 mm), titik terendah z=−0.45.
  Belum divalidasi apakah cukup untuk uji sangkut ROV nyata.

## Sensor & Persepsi (M3) — sudah dibangun
- `[VERIFY]` Kamera di-render headless via `gz-sim-sensors-system` (ogre2). Jalan di mesin
  dev ini (~22 Hz). Di mesin/CI tanpa GPU/EGL, sensor kamera bisa gagal render — perlu cek.
- `[OPEN]` Kalibrasi/CameraInfo kamera sim belum dipetakan ke intrinsics nyata; `camera_info`
  di-publish gz tapi belum dijembatani ke ROS (baru image_raw). Tambah bila node PBVS butuh.
- `[VERIFY]` `depth_publisher` menurunkan kedalaman = max(0,−z) dari odom. Benar di sim
  (permukaan z=0). ROV near-neutral **sedikit positif** → mengapung ke permukaan saat
  thruster mati (depth→0). Ini sesuai desain M1, bukan bug.
- `[OPEN]` Node deteksi QR (`camera_bottom/image_raw` → `/hydroships/qr_result` A/B/C/D)
  belum dibuat. Butuh `cv_bridge` + OpenCV di ROS 2.

## Manipulator (M5) — sudah dibangun
- Gripper 2 jari (revolute sumbu z) di depan ROV, dikontrol gz JointPositionController.
  Perintah semantik `/hydroships/gripper/command` ("open"/"close") → node
  `gripper_controller` → setpoint 2 jari (bridge → gz). State jari di
  `/hydroships/joint_states`. **Terverifikasi**: open ≈ +0.50/−0.50, close ≈ −0.14/+0.15 rad.
- `[VERIFY]` Sudut `open_angle=0.5` / `close_angle=-0.15` (param node) & arah buka/tutup
  masih perlu dicek visual di GUI/kolam agar celah jepit pas ukuran payload.
- `[OPEN]` **Grasp fisik belum diuji**: jari punya collision tapi belum ada model payload
  (bagian M4). Menjepit andal kemungkinan butuh penyetelan friction atau plugin
  attach/detach (mis. gz-sim DetachableJoint) — belum diimplementasi.
- `[OPEN]` `/hydroships/joint_states` (dari gz) hanya berisi 2 joint gripper, bukan
  thruster. TF jari via robot_state_publisher belum tersambung (rsp mendengar
  `/joint_states`, sedangkan bridge ke `/hydroships/joint_states`). Remap bila butuh TF.
- `[note]` `ros2 topic pub --once` ke command bisa meleset karena race discovery;
  node menerbitkan ulang setpoint 2 Hz sehingga joint tetap menahan posisi. Konsumen
  nyata (GUI/autonomy) mengirim berulang, jadi aman.

## Autonomy (M6) — kode dibangun, INTEGRASI TERBLOKIR
Node `mission_fsm` (ROS 2) + launch `hydroships_bringup/launch/hydroships_mission.launch.py`
(sim + allocator + stabilizer + FSM). FSM mengendalikan lewat setpoint stabilizer
(`setpoint/depth`, `setpoint/heading`, `manual/cmd`) + `/hydroships/gripper/command`.

- `[OK]` Wiring benar & terverifikasi: node jalan (`mission_fsm`, `stabilizer`,
  `thruster_allocator`), FSM transisi `IDLE→DIVE`, publish `setpoint/depth=-0.7`,
  stabilizer keluar `cmd_vel.z=-61` (max menyelam), `thruster_4=-27 N`.
- `[OPEN] BLOKIR` **Depth-control DIVERGEN — ROV terbang ke atas keluar kolam.**
  Saat DIVE, `odom z` justru NAIK tak terbatas: terukur ~20 → 23 m dalam 8 s (naik
  ~konstan). `depth` tetap 0 → DIVE timeout → ABORT, seluruh misi gagal.
  Perintah menyelam (`cmd_vel.z<0`, thrust vertikal negatif) TIDAK menurunkan ROV —
  malah lari ke atas (umpan balik positif / kemungkinan tanda terbalik).
  **Tersangka (selesaikan belakangan):**
    1. Arah/alokasi thrust vertikal: perintah "turun" mungkin jadi gaya ke ATAS
       (cek axis 3 thruster vertikal di URDF vs TAM di `thruster_allocator.py`).
    2. Interpenetrasi saat spawn dgn arena/gripper/sensor → impuls awal (cek collision).
    3. Depth-hold M2 kemungkinan **belum pernah** diuji menyelam sungguhan (M3/M5 di sesi
       ini pakai `sim.launch.py` tanpa stabilizer → ROV cuma mengapung pasif).
    4. Setpoint −0.7 m dekat dasar −0.9 m; PID `out_limit` 60 N saturasi.
  **Debug nanti:** uji stabilizer terisolasi (`hydroships_stabilized.launch.py`, tanpa FSM),
  set `setpoint/depth`, amati apakah menyelam; jika lari ke atas juga → bug M2/arah thrust,
  bukan FSM. Cek tanda thruster vertikal & TAM; coba spawn lebih dalam & gain lebih lembut.
- `[OPEN]` `SCAN_QR` menunggu `/hydroships/qr_result` (node QR belum ada) → tanpa QR akan
  timeout; sementara uji dgn `ros2 topic pub /hydroships/qr_result` manual atau `start_state:=`.
- `[TODO]` `APPROACH_HOOK` masih *timed* (visual servo ArUco ROS 2 belum ada) — referensi
  port ada di `GUI-ROV/autonomy/`.

## Umum / lintas-milestone
- `[VERIFY]` Massa & koefisien hidrodinamika ROV masih **placeholder** near-neutral
  (dari `hydroships.urdf.xacro`), belum data ROV asli. Setel di M2+ dgn data nyata.
- `[OPEN]` Integrasi GUI tim (repo GUI-ROV) ↔ topik ROS 2 (M7) belum dijembatani.
