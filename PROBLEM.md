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

## Manipulator (M5) — SEDANG dikerjakan
- (diisi saat implementasi gripper)

## Umum / lintas-milestone
- `[VERIFY]` Massa & koefisien hidrodinamika ROV masih **placeholder** near-neutral
  (dari `hydroships.urdf.xacro`), belum data ROV asli. Setel di M2+ dgn data nyata.
- `[OPEN]` Integrasi GUI tim (repo GUI-ROV) ↔ topik ROS 2 (M7) belum dijembatani.
