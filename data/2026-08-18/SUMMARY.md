# Catatan Run Simulasi — 2026-08-18

Branch `hook-logic-v2` (perubahan belum di-commit). Mesin: tanpa GPU/EGL, semua run `headless:=true`, world `kki_arena.sdf`, ROS 2 Humble + Ignition Gazebo 6 (Fortress).

## 0. Environment note (PENTING)

Sim gagal ambil data di mesin ini karena **CloudflareWARP** memasang route multicast
(`239.0.0.0/9` dst.) ke VPN via tabel `65743` + `ip rule pref 32765` — seluruh
discovery ignition-transport (default `239.255.0.7`) tersedot ke VPN. Gejala:
`ign topic -l` kosong, topik ROS ada tapi tak ada data.

Fix (tanpa sudo) — set sebelum `ros2 launch` DAN semua client `ros2`/`ign`:

```bash
export IGN_IP=192.168.67.71          # paksa bind ZMQ ke WiFi, bukan WARP (172.16.0.2)
export IGN_DISCOVERY_MULTICAST_IP=224.0.0.239   # link-local, di luar jangkauan route WARP
```

Setelah fix: odom 29 Hz, imu 49 Hz, depth 29 Hz normal.

## Run 1 — Sim saja (spawn seed 1001)

- ROV spawn (1.216, -2.05, -0.5) yaw 1.75 rad. Payload QR spawn di (0.394, -1.131, -0.9).
- Depth: stabil **0.1104 m** sepanjang 1092 sampel (~37 s) — ROV mengapung dekat permukaan tanpa perintah thruster.
- `qr_result`: kosong (gagal decode — lihat §Temuan).

## Run 2 — Misi NAV_WALL → loop penuh (seed 1001, joy_trigger=false)

`start_state:=NAV_WALL start_wall:=B`. Trigger manual lewat
`/hydroships/mission/start_autonomous` saat WAIT_TRIGGER.

| Transisi | t (s rel. mulai) | Catatan |
|---|---|---|
| IDLE → NAV_WALL | 0.0 | wall order [B, C, A, D] |
| NAV_WALL → HANG (B) | 7.5 | standoff dist 0.20 m; target (0.00, 2.15) |
| HANG → SURFACE | 17.7 | lubang di atas tip hook B (dist 0.006, l_err 2.8 mm, yaw 0.3°); payload tergantung stabil depth 0.47 |
| SURFACE → WAIT_TRIGGER | 19.7 | menunggu trigger pilot |
| WAIT_TRIGGER → APPROACH_HOOK | +0.0 (setelah trigger) | "Mulai misi pelepasan payload AUTONOMOUS" |
| APPROACH_HOOK → AUTO_RELEASE | 9.9 | hook terpusat (ex 0.00, ey 0.54, size 0.51) |
| AUTO_RELEASE → DIVE | 18.3 | lepas & kembali ke awal loop |
| DIVE → APPROACH_QR → DESCEND → GRAB → NAV_WALL | ~10 | loop ke wall C |
| NAV_WALL → HANG (C) | 3.0 | lubang di atas tip hook C (dist 0.011, l_err 5.7 mm, yaw 1.6°) |
| HANG → SURFACE → WAIT_TRIGGER | 8.6 | payload tergantung di C (depth 0.45) |

Depth range: 0.096–0.760 m (avg 0.238). Odom: awal (0.00, 2.31) → akhir (2.09, -0.02).
Gripper: open (auto-detach startup) → close (GRAB) → close → close.

**Kesimpulan run 2: loop misi penuh dan hook-logic-v2 (APPROACH_HOOK → AUTO_RELEASE) bekerja end-to-end.**

## Run 3a — APPROACH_HOOK tanpa `start_wall` (seed 1002)

`start_state:=APPROACH_HOOK` TANPA `start_wall` → `IDLE → APPROACH_HOOK → ABORT` dalam 0.1 s.
Penyebab: `mission_fsm.py:1181` `if self.wall is None: ABORT` — `wall` hanya di-set dari
`start_wall` atau state sebelumnya. **Dokumentasi HOW-TO-RUN 3H menyebut state ini bisa di-start langsung; perlu `start_wall:=A/B/C/D`.**

## Run 3b — APPROACH_HOOK + start_wall:=B (tuning: hook_size_stop 0.40, hook_center_tol 0.10, hook_max_age 2.0, t_approach 30)

| Transisi | t (s) | Catatan |
|---|---|---|
| IDLE → APPROACH_HOOK | 0.0 | backoff 1.2 s pertama |
| APPROACH_HOOK → AUTO_RELEASE | 5.2 | visual servo berhasil: hook terpusat (ex 0.00, ey 0.00, size 0.70) |
| AUTO_RELEASE → ABORT | 44.0 | `AUTO_RELEASE timeout (turun, depth 0.194)` |

Fase turun AUTO_RELEASE tidak selesai: "lubang di atas hook (dist 0.012, l_err 5.6 mm)" tercapai
di t≈34 s tapi depth masih 0.194 m (target hook ~0.47 m) saat timeout → ABORT. Catatan: run ini
direct-start tanpa payload di gripper (tidak ada fase GRAB), jadi kontak/stall detector fase
turun kemungkinan tidak mendapat kondisi yang diharapkan.

Depth range run 3b: 0.050–0.829 m (avg 0.554).

## Temuan tambahan

1. **OpenCV tanpa QUIRC**: log qr_detector "Library QUIRC is not linked. No decoding is performed."
   → QR decode **selalu** gagal di mesin ini, bukan hanya karena render headless. `apt` opencv
   (ros-humble) dibangun tanpa QUIRC. Workaround: pip `opencv-python` ber-QUIRC, atau suntik
   `qr_result` manual untuk uji FSM.
2. **hook_detector bekerja headless**: "hook terdeteksi: center=(320,302) area=108312" — deteksi
   hook (warna/kontur) tidak bergantung QUIRC, jadi visual servo APPROACH_HOOK valid di headless.
3. `mission_fsm` TIDAK mempublish topik `/hydroships/fsm_state` (disebut di HOW-TO-RUN §9);
   transisi hanya di log `[FSM] X -> Y`. Cek status lewat log launch.

## File data (CSV `ros2 topic echo --csv`)

| File | Isi | Sampel |
|---|---|---|
| run1_sim_odom.csv | pose/twist ROV (89 kolom) | 1192 |
| run1_sim_depth.csv | depth (m) | 1092 |
| run2_mission_odom.csv | odom misi | 1825 |
| run2_mission_depth.csv | depth misi | 1826 |
| run2_mission_cmd_vel.csv | wrench perintah | 1242 |
| run2_mission_gripper.csv | perintah gripper (open/close) | 4 |
| run3a_abort_odom.csv | odom run abort (tanpa start_wall) | — |
| run3b_servo_odom.csv | odom APPROACH_HOOK run 3b | — |
| run3b_servo_depth.csv | depth run 3b | 6917 |
| run3b_servo_cmd_vel.csv | wrench perintah run 3b | — |
| run3b_servo_hook_offset.csv | offset hook dari hook_detector | — |

Log mentah launch: `/tmp/opencode/run{1,2,3}*.log` (tidak di-copy ke repo).
---

## Sesi verifikasi patch (2026-08-19, branch payload-logic-v2)

3 patch diuji (dua dipasang permanen, satu di-revert):

1. **PRESISI HANG fase 2** (mission_fsm.py `_st_hang`): gate sukses fase-turun
   sekarang menguji ulang `dist < hang_tol` DAN `l_err < hang_l_tol` (sebelumnya
   hanya depth+yaw — lubang bisa bergeser saat turun tapi tetap "lolos diam-diam").
   - Fase 1 terukur presisi: lubang di atas tip (dist 0.006-0.007, l_err 4.4-4.8 mm).
   - TAPI sim mengungkap masalah lama yang selama ini tertutup gate longgar: fase 2
     TIDAK bisa mempertahankan 8 mm lateral selama turun. Dua mode gagal berbeda:
     - fmax 0.6x: lubang meleset -> ROV turun bebas ke dasar (depth 0.827), timeout.
     - fmax 1.0x: surge kuat menyodok ROV ke dinding -> naik ke permukaan (0.113), timeout.
   - Status: gate presisi BENAR & dipertahankan; descent controller butuh tuning
     tersendiri (yaw-hold saat turun + arah koreksi kontak tip) — di luar 1 patch.
2. **QR false-positive gate** (qr_logic.py `_quiet_zone_ok` + payload quiet-zone
   plane 0.16 -> 0.20): corner QR palsu (siluet hook/tembok, "QR besar" di hook 2)
   kini ditolak tanpa quiet zone putih. Verifikasi sim: log qr_detector berubah dari
   "QR terdeteksi tapi decode kosong" -> "QR tak terdeteksi (pts=None)" utk objek
   non-payload. Unit test baru `test_quiet_zone_ok_*` lolos.
3. **Payload "di atas gripper"** — grab_depth 0.65 dicoba, REVERT ke 0.70: test
   `test_gripper_mencapai_payload_di_grab_depth` menangkap regresi (celah gripper->
   QR 0.090 m > 0.08 = weld lintas ruang). Root cause bukan grab_depth: collision
   body sudah sejajar jari; yang mengambang adalah MESH visual (`payload_body.obj`)
   yang origin-nya di atas titik weld. Perlu inspeksi mesh + verifikasi GUI.
