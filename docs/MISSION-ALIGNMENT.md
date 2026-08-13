# MISSION-ALIGNMENT — mission_fsm.py (sim, ROS2) ↔ mission5.py (hardware, GUI-ROV)

P2-C: basis pembandingan parameter untuk tuning di hardware nanti. Dua FSM ini
dikembangkan independen di dua repo (`hydroships_control/mission_fsm.py` — sim
ROS2/Gazebo, dan `GUI-ROV/autonomy/fsm/mission5.py` — kontrol hardware via UDP
ke `rov_link.py`/ArduSub) untuk misi kontes yang sama (KKI 2026). Dokumen ini
memetakan parameter yang **secara fisik sebanding**, dan secara eksplisit
memisahkan yang **berbeda secara arsitektur** (bukan cuma beda angka tuning —
memaksakan delta numerik di situ menyesatkan).

Sumber: `hydroships_control/mission_fsm.py` (param `p('nama', default)` di
`__init__`) vs `GUI-ROV/autonomy/fsm/mission5.py` (konstanta modul level atas).

## ⚠️ Perbedaan sistem unit — bukan parameter, tapi mendasari semua baris di bawah

| | mission_fsm.py (sim) | mission5.py (hardware) |
|---|---|---|
| Aktuasi | **Gaya (Newton)** lewat `thruster_allocator` (damped pseudo-inverse 6-DOF) | **Persen thruster mentah (0-100%)** dikirim langsung ke ArduSub via `rov_link.py`, tanpa alokasi 6-DOF |
| Servo visual | PID gain N/m, N/(m/s) (`approach_kp/kd`, `hook_kp_*`) | Gain piksel (IBVS, mis. `IBVS_KP_SWAY=45.0`) ATAU gain meter (PBVS, mis. `PBVS_KP_SWAY=140.0`) — dua mode berbeda, dipilih via kalibrasi kamera ada/tidak |

**Implikasi tuning**: gain N/m (`approach_kp=90.0`) TIDAK BISA dikonversi langsung ke gain persen (`IBVS_KP_SWAY=45.0`) tanpa model thruster fisik (N per % PWM) — align numerik dua kolom ini di baris manapun di bawah adalah **kesalahan kategori**, bukan sekadar "belum di-tune sama". Basis konversi yang valid hanya lewat karakterisasi thruster real (N vs % throttle), belum ada di kedua repo.

## Kedalaman & toleransi (unit sama: meter — bisa dibandingkan langsung)

| Parameter | mission_fsm.py | mission5.py | Delta | Action |
|---|---|---|---|---|
| Kedalaman dasar/grasp | `grab_depth=0.70` | `DEPTH_TARGET_BOTTOM=0.70` | 0 | ✅ match |
| Kedalaman scan QR | `scan_depth=0.30` | *(tidak ada — 1 fase, sama dgn 0.70)* | 0.40 m | **OPEN, bukan cuma angka**: mission_fsm 2-fase (scan dangkal→FOV lega, lalu `DESCEND` ke grab); mission5 1-fase (dive langsung ke 0.70 lalu scan di situ). Lihat `docs/STATUS.md` M3 soal kenapa 0.30 dipilih (QR 12cm ter-crop di 0.70/0.46). Sebelum align: putuskan apakah hardware perlu fase scan terpisah juga (kemungkinan besar YA, alasan optical framing yang sama berlaku fisik) — bukan tinggal ubah angka mission5.
| Toleransi kedalaman | `depth_tol=0.06` | `DEPTH_TOLERANCE=0.05` | 0.01 m | Dekat; boleh disamakan salah satu arah tanpa investigasi lebih.
| Ambang "di permukaan" | `depth_surface=0.08` | `DEPTH_TARGET_SURFACE=0.05` | 0.03 m | Dekat; sama seperti di atas.
| Kedalaman hook | `hook_depth=0.45` | `HOOK_DEPTH=0.45` | 0 | ✅ match

## Timeout per state (unit sama: detik)

| State (peran sebanding) | mission_fsm.py | mission5.py | Delta | Catatan |
|---|---|---|---|---|
| DIVE | `t_dive=20.0` | `TIMEOUT_DIVE=15.0` | 5.0 s | sim lebih longgar
| Scan QR | `t_scan=45.0` | `TIMEOUT_SCAN=20.0` | 25.0 s | sim JAUH lebih longgar — sim toleransi banyak retry decode candidate (`qr_logic.py`, 7 kandidat preprocessing/frame); kalau hardware pakai pipeline vision serupa, 20s mungkin terlalu ketat.
| GRAB | `t_grab=10.0` | `TIMEOUT_GRAB=10.0` | 0 | ✅ match
| NAV_WALL | `t_nav=30.0` | `TIMEOUT_NAV=30.0` | 0 | ✅ match
| HANG | `t_hang=20.0` | `TIMEOUT_HANG=15.0` | 5.0 s |
| SURFACE | `t_surface=20.0` | `TIMEOUT_SURFACE=15.0` | 5.0 s |
| APPROACH_HOOK (servo ke hook) | `t_approach=25.0` | `HOOK_ACQUIRE_T=8.0` (akuisisi) + `TIMEOUT_M5_DOCK=25.0` (docking closed-loop) | tak bisa 1 angka | **Bukan 1:1** — mission_fsm punya SATU state/timeout utk seluruh servo hook; mission5 memecahnya jadi akuisisi (8s) lalu docking (25s) terpisah. `t_approach=25.0` mission_fsm kebetulan sama dgn `TIMEOUT_M5_DOCK`, tapi itu cuma bagian kedua dari proses 2-fase mission5 — total anggaran mission5 utk peran yang sama ≈ 33s, bukan 25s.

## Geometri hook/wall (unit sama: meter, TAPI referensi beda)

| Parameter | mission_fsm.py | mission5.py | Catatan |
|---|---|---|---|
| Jarak approach ke hook | `hook_dist=0.30` | `HOOK_TARGET_DIST=0.30` (mode PBVS) | ✅ match numerik DI MODE PBVS. Mode IBVS mission5 pakai `HOOK_TARGET_AREA=3000` px² — unit beda total, tak sebanding.
| Diameter fisik hook | *(tak ada param eksplisit — dipakai implisit di world SDF Ø25mm, lihat `docs/STATUS.md` M4)* | `HOOK_PIPE_DIAM_M=0.025` | mission5 eksplisit di kode; mission_fsm cuma di world geometry. Pertimbangkan eksplisitkan juga di `mission_fsm.py`/`hook_logic.py` kalau estimasi jarak dari ukuran piksel mau dipakai (mission5 punya ini utk PBVS solvePnP-like, mission_fsm belum).
| Standoff jarak ke wall (NAV_WALL) | `wall_dist=2.15` | *(tak ada — mission5 navigasi arah via `WALL_HEADING` heading target, bukan jarak absolut standoff)* | Beda paradigma navigasi: mission_fsm target XY absolut (posisi wall diketahui dari geometri arena tetap); mission5 kemungkinan mengandalkan visual/timeout murni utk NAV_WALL krn tak ada localisasi XY absolut di hardware (tak ada ground truth odom!). **Ini justru pertanyaan inti P2 lainnya** — lihat P2-B (`docs/ARCHITECTURE.md`) soal odom noise; hardware nyata makin dekat ke pola mission5 (navigasi tanpa posisi absolut presisi) drpd mission_fsm sim (odom ground truth).
| Radius "tiba" (arrival tolerance) | `nav_tol=0.20` | *(tak ada radius eksplisit — mission5 pakai kombinasi timeout + servo distance, bukan radius+dwell)* | mission_fsm py: radius XY + `hold_settle_s=2.0` dwell. mission5: state-transition-on-timeout/servo-converged, tanpa dwell period eksplisit. Kalau mau align, mission5 perlu tambah radius+dwell check yg sama, bukan cuma ganti angka.

## Rekomendasi urutan align (untuk sesi tuning hardware)

1. **Kedalaman** (bagian pertama tabel) — unit & makna identik, paling aman di-align duluan. Delta kecil (`depth_tol`, `depth_surface`) bisa disamakan tanpa risiko.
2. **`scan_depth` dua-fase** — putuskan dulu apakah hardware butuh fase scan terpisah (mirip alasan framing QR di sim) SEBELUM menambah param baru ke `mission5.py`; kalau ya, ini perubahan STRUKTUR mission5 (state baru), bukan cuma konstanta.
3. **Timeout** — align langsung (unit sama, cuma beda toleransi kesabaran); mulai dari yang deltanya kecil (GRAB/NAV_WALL sudah match).
4. **Gain servo (N/m vs %/piksel/meter)** — JANGAN disamakan sampai ada karakterisasi thruster N-per-% (lihat catatan unit di atas). Prioritas rendah, butuh data hardware baru, bukan cuma baca dua file ini.
5. **Navigasi wall tanpa odom absolut (`wall_dist`, `nav_tol`)** — terkait langsung P2-B (odom noise injector) & rencana P2 lain (ground truth break) di `docs/ARCHITECTURE.md`; selesaikan setelah eksperimen odom-noise sim memberi gambaran seberapa robust `mission_fsm.py` tanpa localisasi presisi, baru putuskan pola navigasi mana yang mau diadopsi hardware.
