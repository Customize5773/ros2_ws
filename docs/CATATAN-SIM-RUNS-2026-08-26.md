# Catatan Sim Runs — 2026-08-26

Empat misi penuh headless berurutan (auto-trigger tiap `WAIT_TRIGGER`, window
maks 25 menit/run) setelah perbaikan HANG/NAV_WALL hari ini. Log mentah:
`/tmp/opencode/runs/R{1..4}.log` (mesin dev, bukan artefak repo). Runner:
`/tmp/opencode/run_one.sh`. Kode: `mission_fsm.py` dgn 4 fix 2026-08-26
(gate yaw NAV_WALL→HANG, retreat yaw-aktual, hold-at-standoff, seat via
stall + jalur longgar).

## Ringkasan

| Run | Urutan wall | Hasil akhir | Hook tergantung | Durasi misi* | Catatan |
|-----|-------------|-------------|-----------------|--------------|---------|
| R1 | A,C,D,B | **DONE — SKOR 100/100** | A(longgor), C, D, B | ~5m20s | bersih, tanpa anomali |
| R2 | B,A,C,D | **DONE — SKOR 100/100** | B(longgor), C, A, D | ~6m30s | bersih |
| R3 | C,A,B,D | **DONE — SKOR 100/100** | C(longgor), A, B×4, D | ~10m | loop QR salah baca 'B' 3 siklus (lihat Temuan 1) |
| R4 | D,B,C,A | **ABORT** (AUTO_RELEASE timeout posisi) | D(longgor), B, B(salah QR) | ~4m40s | physics explosion makan 40s anggaran (lihat Temuan 2) |

*dari `IDLE -> DIVE` sampai `SKOR:`/`-> ABORT`.

Agregat: **3/4 DONE dengan skor penuh 100/100; 18 kejadian payload
tergantung, 0 kegagalan HANG.** Sebelum fix hari ini, HANG bisa gagal di hook
pertama (retreat frame-salah) dan plate duduk geser 30-33mm langsung ABORT.

## Perilaku baru yang terkonfirmasi bekerja

- **Jalur longgar seat HANG** (`dist ≤ 50mm` + stall depth): terpakai 4×
  (R1-A 33mm, R2-B 33mm, R3-C 30mm, R4-D 32mm), selalu dgn log WARN "cek
  visual". Semua itu = ABORT pada kode lama.
- **Gate yaw NAV_WALL→HANG**: tidak ada lagi masuk HANG dgn yaw meleset >10°;
  cabang retreat `_st_hang` tidak pernah terpicu di 18 hang.

## Temuan baru (OPEN, di luar scope fix hari ini)

1. **QR/spawner salah huruf antar-siklus** (R3, R4): FSM minta spawner wall X
   ("Minta payload baru utk wall D/C"), tapi APPROACH_QR membaca huruf payload
   LAMA ('B') hingga 3 siklus berturut-turut → ROV menggantung ulang di hook
   yang sama. Dugaan: ROV membaca QR payload yg barunya dilepas/dekat wall
   sebelumnya saat DIVE, ATAU spawner men-spawn huruf lama. Perlu cek
   `payload_spawner.py` (respons `spawn_next`) & apakah payload lama masih
   ada di dunia saat scan. Impact: waktu bukan skor (hook duplikat tetap
   menambah done_hooks? TIDAK — done_hooks.add(w) idempotent; run tetap
   selesai tapi boros ±75s/siklus).
2. **Physics explosion** (R4 fatal; juga terlihat di run prasebar hari ini):
   pose ROV melompat mustahil-fisik (contoh R4: (-0.01,2.12,d0.11) →
   (1.36,1.90,d0.81) dalam <20s tanpa perintah; yaw +25° dalam 15ms) lalu
   kontrol pulih normal. Satu publisher odom terverifikasi saat kejadian →
   BUKAN dual-publisher (kandidat lama STATUS.md gugur untuk kasus ini).
   Dugaan baru: kontak solver meledak (weld DetachableJoint overlap / plat
   vs struktur). R4 ABORT karena budget T['release']=60s habis — tick
   terakhir malah sudah konvergen sempurna (dist 6mm, yaw 0.8°, depth 0.34).

## Tambahan run user (log terpisah, 2026-08-26 siang) + fix susulan

3. **AUTO_RELEASE fase-1 JAM tip-slot** (ABORT): ROV nyangkut dist 46mm
   (30mm terlalu maju + 30mm lateral) — tip masuk lubang tapi menekan
   dinding ujung slot; gaya koreksi dekat-target cuma ~6.6N (floor
   `min_fmax_frac=0.30`) kalah gesekan → merayap 0.5mm/s sampai timeout
   posisi. **FIX (kode, 2026-08-26)**: (a) floor gaya fase-1 HANG &
   AUTO_RELEASE 0.30→0.60; (b) gate fase-1 longgar dist≤50mm (dalam lebar
   slot ±45mm) — seating tetap diputuskan stall fase turun; (c) fase-2
   AUTO_RELEASE kini cermin HANG: stall depth memicu detach dgn penerimaan
   strict ≤25mm / LONGGOR ≤50mm. Regresi test: 138/138 hijau.
4. **`qr_detector` CRASH native** (`double free or corruption (out)`,
   exit -6) di jalur decode `adaptive_thresh_upscaled` — crash OpenCV,
   bukan exception Python; node mati sampai direstart. Belum difix (perlu
   reproduksi & bisect terpisah). Dampak: offset QR hilang → servo
   APPROACH_QR/APPROACH_HOOK buta setelah kejadian; AUTO_RELEASE tidak
   terpengaruh (murni odometri). Prioritas sedangkan #1/#2 menunggu data.

## Putaran verifikasi R5–R8 (setelah fix #3/#4, 2026-08-26)

| Run | Hook tergantung | Akhir | Temuan |
|-----|-----------------|-------|--------|
| R5 | B, C | ABORT — AR fase-1 macet dist 155mm | REVERSE serahkan posisi buruk (timeout step 22), plate tertanam melewati muka dinding wall C |
| R6 | A, B | ABORT — AR fase-1 beku dist 5.0m | REVERSE stranded di POJOK arena (stuck 74); jitter ±5mm me-reset detektor mandek absolut → margin diperbaiki jadi 1%-dr-jarak |
| R7 | A*, B*, C, D (**4/4**) | ABORT — AR hook-D fase-1 timeout | Boundary hover: konvergensi melintas batas 50mm bolak-balik, reset dwell berulang → dwell kini pakai grace 1s (`update_dwell`) |
| R8 | C | ABORT — SURFACE timeout (yaw_err 103.6°) | Setelah kontak HANG di wall C, heading tersisa ~104° & anggaran T['surface']=20s tak cukup |

\* LONGGOR (≤50mm).

**FIX tambahan hasil R5–R7** (semua tervalidasi unit test, total 139/139):
- Escape/backoff fase-1 AUTO_RELEASE: 15s tanpa progres (margin 1%-dr-jarak,
  nonaktif ≤50mm) → mundur 2s lalu coba lagi, bukan nebak struktur sampai
  timeout.
- Dwell fase-1 HANG & AUTO_RELEASE pakai grace 1s — tahan lintasan batas.
- Backoff terbukti tidak salah picu (0 okuransi palsu di R7/R8); pola gagal
  berpindah ke state DI HULUNYA (REVERSE/SURFACE) — artinya rantai
  NAV_WALL→HANG→AUTO_RELEASE sudah jauh lebih kuat dari pagi ini.

## Status OPEN yang tersisa (pre-existing, urutan dampak)

1. QR/spawner salah huruf antar-siklus (duplikasi hook, boros waktu).
2. Physics explosion / odom teleport + REVERSE stranded (R4/R5/R6/R8 —
   keluarga sama? perlu rosbag saat kejadian).
3. `qr_detector` double-free crash (butuh bisect OpenCV pipeline).
4. Budget align SURFACE (20s) vs laju rotasi terukur ~4°/s setelah kontak.


## Saran lanjut

- Temuan 1: audit `payload_spawner.py` spawn_next + despawn payload lama;
  pertimbangkan gate "abaikan qr_wall yg sama dgn done_hooks terakhir".
- Temuan 2: replikasi dgn rosbag (`ros2 bag record /hydroships/odom`) saat
  lompatan; cek log gz physics/contact; pertimbangkan naikkan T['release']
  atau reset timer saat deteksi teleport.
