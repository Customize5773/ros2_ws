# P0-2.3 SPEC — Positioning/centering accuracy at GRAB (KKI 2026)

Dokumen ini adalah **spec**, dengan satu pengecualian: §1 memuat hasil analisis yang memang
sudah dieksekusi, karena analisis itu **tidak membutuhkan run simulator baru** — ia hanya
mem-parsing data yang sudah ada dari battery P0-2.2b (`/tmp/p0-2-2b-battery/*.log`). Bagian
lainnya (§3 dst) adalah desain, belum dieksekusi. Tidak ada perubahan
`mission_fsm.py`/`qr_detector.py`/`qr_logic.py`/parameter dalam pembuatan dokumen ini.

Prasyarat: P0-2.2 **CLOSE-PARTIAL** (`docs/P0-2-2-VERDICT-OPTIONS.md` §4). Klaim yang sudah
ditutup: "QR integration/causal influence terbukti" (Gate 3, 5/6 run). Klaim yang **belum**:
"QR visual servo memberikan precision convergence yang repeatable" (Gate 4, 0/6 run masuk band
`qr_center_tol`). P0-2.3 dirancang khusus untuk klaim kedua ini, dengan scope yang dipertajam
oleh pengguna: **menguji apakah `APPROACH_QR` menghasilkan positioning/centering yang benar
pada titik `GRAB`, bukan sekadar apakah QR pernah memengaruhi command.**

---

## 1. Temuan pendahuluan (SUDAH dieksekusi — reuse data P0-2.2b, tanpa run baru)

`mission_fsm.py` sudah menghitung dan mencatat metrik ini sendiri: `_gripper_align_txt()`
(L488-503) menghitung `gripper_err` (jarak XY ground-truth dari GRIPPER — bukan base_link —
ke posisi payload asli) dan `base_err` (jarak base_link tanpa koreksi), dan angka ini **sudah
tercatat di log** setiap kali `GRAB` terpicu (L600 dan L614). Skrip baru
`tools/p0-experiments/extract_gripper_err.py` mem-parsing baris ini dari `$TAG.log` yang
sudah ada di `/tmp/p0-2-2b-battery/` — tidak menjalankan apa pun, tidak butuh data baru.

Hasil dari 6 log battery P0-2.2b:

| Run | Trigger | `gripper_err` (m) | `base_err` (m) |
|---|---|---|---|
| Q1 | `QR terpusat (jarak XY) -> GRAB` | 0.008 | 0.183 |
| Q2 | `QR terpusat (jarak XY) -> GRAB` | 0.029 | 0.201 |
| Q3 | `QR terpusat (jarak XY) -> GRAB` | 0.024 | 0.187 |
| Q4 | `Wall B dipilih (+15) [urutan ke-1]` (fallback) | 0.058 | 0.232 |
| Q5 | `QR terpusat (jarak XY) -> GRAB` | 0.030 | 0.210 |
| Q6 | `QR terpusat (visual servo) -> GRAB` | 0.050 | 0.193 |

`gripper_err`: min 0.008 m, max 0.058 m, mean 0.033 m, di keenam run — **kecil dan konsisten,
tidak peduli jalur exit-nya**.

## 2. Kenapa angka ini TIDAK menjawab pertanyaan yang tepat

`gripper_err` kecil ini **diharapkan secara struktural**, bukan bukti performa QR: exit
`dist < approach_tol` (§1.6 `docs/P0-2-AUDIT.md`) dihitung terhadap target yang SUDAH
dikoreksi `gripper_base_dx` (`mission_fsm.py:552-554`) — persis definisi `gripper_err`.
Artinya begitu exit terpicu lewat jalur jarak XY, `gripper_err` HAMPIR PASTI kecil **by
construction**, terlepas dari QR — bahkan run `GROUND_TRUTH_FALLBACK` murni (Q4, QR tidak
pernah terbaca) tetap menghasilkan `gripper_err=0.058m`, karena targetnya tetap
`payload_pose` ground-truth.

Ini krusial: **`payload_pose` adalah oracle simulasi**, dipublikasikan oleh `payload_spawner.py`
(§1.5 `docs/P0-2-AUDIT.md`) — di hardware asli, topic ini tidak ada. Jadi angka `gripper_err`
di atas mengukur "seberapa baik kontroler mencapai target yang SUDAH tahu jawabannya", bukan
"seberapa akurat sistem akan menempatkan gripper kalau HANYA punya sinyal QR seperti kondisi
nyata". Pertanyaan kedua inilah yang belum terjawab, dan itulah scope tajam P0-2.3.

## 3. Pertanyaan P0-2.3 yang dipertajam: QR sebagai sensor posisi independen

Alih-alih "apakah command mengikuti `qr_offset`" (sudah dijawab di P0-2.2 Gate 3), P0-2.3
menguji: **seberapa akurat estimasi posisi relatif yang BISA diturunkan murni dari sinyal QR
(`qr_ex, qr_ey, qr_size`), dibandingkan terhadap ground truth — sebagai simulasi kondisi
tanpa oracle `payload_pose`.**

Geometri kamera sudah cukup untuk estimasi jarak dari `qr_size` (proxy ukuran-tampak) via
proyeksi pinhole standar:

```text
distance_est = (fx_px * qr_side_m) / (qr_size * frame_width_px)
```

- `fx_px`, `frame_width_px`: dari `camera_info` — sudah ter-log identik di semua run battery
  (`fx=381.4, cx=320.0, frame=640x480`, lihat `*.log` "camera_info camera_bottom_link").
  **Asumsi**: nilai ini konstan sim-only, tidak per-run — cukup untuk analisis awal, tapi
  BUKAN kalibrasi hardware (`docs/P0-2-AUDIT.md` §1.2 sudah menandai `K` sim-only).
- `qr_side_m`: sisi fisik QR. Disebut "12cm" di komentar `mission_fsm.py:108-118`, **belum
  dikonfirmasi dari sumber otoritatif** (mis. world/payload model SDF) — perlu diverifikasi
  sebelum dipakai sebagai angka pasti, bukan diasumsikan benar begitu saja.

Offset lateral (`qr_ex`) bisa diubah jadi estimasi posisi 2D relatif dengan cara serupa
(memakai `distance_est` dan `cam_vfov_half_tan`/setara horizontal). Hasilnya: estimasi
`(dx_est, dy_est)` murni dari QR, dibandingkan terhadap `(dx_true, dy_true)` dari
`payload_pose - odom` (ground truth, sudah terekam) — **error metrik dalam meter**, bukan
sekadar koefisien korelasi seperti Gate 2 P0-2.2 (yang hanya mengonfirmasi arah hubungan, bukan
besarnya).

## 4. Data yang dibutuhkan

**Bisa dijawab dari CSV `recorder_qr.py` P0-2.2b yang SUDAH ADA** (`Q1.csv`...`Q6.csv`): kolom
`qr_ex, qr_ey, qr_size, x, y, payload_x, payload_y` semuanya sudah terekam sepanjang episode
`APPROACH_QR`. **Tidak perlu battery baru** untuk pass pertama analisis ini. Intrinsics kamera
(`fx, cx, frame_width`) diambil sebagai konstanta dari log (bukan per-baris CSV) — kalau
analisis awal menunjukkan itu tidak cukup presisi, instrumentasi tambahan (log `camera_info`
per baris) adalah perbaikan kecil observability-only untuk pass berikutnya, **belum
diperlukan sekarang**.

## 5. Gate P0-2.3 yang diusulkan

| Gate | Pertanyaan | Metode |
|---|---|---|
| P1 | Placement ground-truth di titik GRAB (deskriptif, BUKAN acceptance akhir) | Sudah ada, §1 — `gripper_err` dari log, mean 0.033m, tapi bias struktural dicatat di §2 |
| P2 | Akurasi estimasi posisi QR-murni vs ground truth | Reproyeksi pinhole `qr_size/qr_ex/qr_ey` → `(dx_est,dy_est)`, error metrik (m) terhadap `payload_pose-odom`, per baris `APPROACH_QR` dengan `qr_size` valid |
| P3 | Repeatability P2 lintas run/posisi | Statistik error P2 di 6 run existing (dan run baru kalau perlu, §6) |
| P4 (stretch, TBD) | Apakah presisi P2 cukup untuk gripper mekanis nyata? | **Butuh input domain** (lebar jepitan gripper) — belum ada di repo, jangan diasumsikan; kalau tidak tersedia, gate ini tetap terbuka tanpa angka acceptance |

## 6. Langkah berikutnya yang diusulkan (belum dieksekusi)

1. **Build `reduce_qr_precision.py`** (nama diusulkan) yang menghitung Gate P2/P3 dari CSV
   `Q1`-`Q6` yang sudah ada — analisis-saja, tanpa run simulator baru.
2. **Verifikasi `qr_side_m`** dari sumber otoritatif (world/payload SDF), bukan komentar kode,
   sebelum dipakai sebagai konstanta kalibrasi.
3. Hanya kalau data existing (6 run, jendela servoing 16-53 baris per run dari P0-2.2 Gate 3)
   ternyata tidak cukup untuk P2/P3 yang meyakinkan — baru desain battery baru/tertarget
   (P0-2.3b, terpisah, perlu persetujuan sendiri), bukan otomatis.

## 7. Non-goals eksplisit untuk pass ini

- Tidak ada run simulator baru dieksekusi untuk §3-§6 (§1 saja yang sudah jalan, dan itu
  murni re-parsing log lama).
- Tidak ada perubahan `mission_fsm.py`/`qr_detector.py`/`qr_logic.py`/parameter.
- Tidak ada klaim PASS/FAIL. Acceptance matrix `docs/P0-2-AUDIT.md` §2 tidak diubah.
- `qr_side_m=0.12` (dari komentar) TIDAK dipakai sebagai fakta terverifikasi sampai
  dikonfirmasi §6.2.

## 8. Status

```text
P0-2.2      CLOSE-PARTIAL   (QR influence VERIFIED, QR precision convergence OPEN)
P0-2.3      SPEC WRITTEN, §1 PRELIMINARY DATA REUSED — reduce_qr_precision.py NEXT
```
