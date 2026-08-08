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

## 7. §6.2 terverifikasi: `qr_side_m` dikonfirmasi dari SDF, bukan komentar

`src/hydroships_gazebo/scripts/payload_spawner.py`, `PAYLOAD_SDF_TEMPLATE`, visual `qr`
(L59-61): `<geometry><plane><size>0.12 0.12</size></plane></geometry>` — **0.12m × 0.12m**,
cocok dengan komentar "12cm" di `mission_fsm.py`, tapi sekarang berasal dari geometri SDF
yang benar-benar di-render Gazebo, bukan komentar tak terverifikasi. (Ada juga plane
`qr_quiet_zone` terpisah 0.16m×0.16m, L49-51, dan body payload berbeda lagi 0.05×0.006×0.10m,
L38 — bukan plane QR itu sendiri.) `kki_arena.sdf:181` mengonfirmasi `payload_spawner.py`
adalah satu-satunya sumber definisi model ini (tidak ada definisi statis lain di world).
Blocker §6.2 **selesai**.

## 8. Gate P2/P3 — hasil eksekusi (reuse data P0-2.2b, TANPA run baru)

Dieksekusi: `tools/p0-experiments/reduce_qr_precision.py` terhadap `Q1`-`Q6.csv` yang sudah
ada di `/tmp/p0-2-2b-battery`. **Sengaja dipisah total dari `gripper_err`** (§1) — tabel di
bawah TIDAK PERNAH digabung dengan tabel `gripper_err`, sesuai batasan pengguna.

| Run | n baris | QR-estimate `err_m` (mean) | median | min | max | `distance_est − h_cam` (mean) |
|---|---|---|---|---|---|---|
| Q1 | 21 | 0.093 | 0.085 | 0.057 | 0.149 | −0.652 |
| Q2 | 118 | 0.875 | 0.800 | 0.057 | 2.286 | −0.682 |
| Q3 | 54 | 0.420 | 0.332 | 0.052 | 1.114 | −0.761 |
| Q4 | 91 | 1.144 | 0.636 | 0.134 | 3.195 | −0.272 |
| Q5 | 126 | 1.036 | 0.337 | 0.045 | 3.530 | −0.812 |
| Q6 | 68 | 0.540 | 0.479 | 0.001 | 1.329 | −0.808 |

**Gate P3 (repeatability)**: mean `err_m` per run tersebar luas (0.093 – 1.144 m) —
**tidak repeatable/konsisten** lintas run.

### Temuan penting: bias sistematis di cross-check, bukan sekadar noise

`distance_est − h_cam` **negatif dan cukup konsisten besarnya** (−0.27 sampai −0.81 m) di
SEMUA 6 run — ini bukan noise acak, ini **bias sistematis**. `distance_est` (dari `qr_size`,
murni sinyal visual) secara konsisten lebih kecil dari `h_cam` (dari geometri
`depth_target` yang diketahui). Kemungkinan penyebab (belum diisolasi, dicatat sebagai
pertanyaan terbuka, BUKAN diperbaiki di pass ini):

1. **Asumsi pandangan frontal-tegak-lurus mungkin tidak berlaku** — formula pinhole yang
   dipakai mengasumsikan QR dilihat dari lurus di atas (kamera bawah tegak lurus ke bidang
   QR). Kalau ROV berada jauh dari QR secara lateral (bukan hanya vertikal), bounding-box QR
   di citra akan terdistorsi perspektif (foreshortening), membuat `qr_size` tidak lagi murni
   fungsi jarak vertikal saja.
2. **Deteksi corner-only yang noisy** — log `qr_detector` (§1.8 `docs/P0-2-AUDIT.md`)
   menunjukkan banyak kasus "QR terdeteksi (pts ada) tapi decode kosong" — bounding box dari
   4 titik sudut bisa saja terdeteksi tapi tidak akurat/reliable, menghasilkan `qr_size` yang
   menyimpang dari ukuran QR asli.
3. Kemungkinan lain (belum diperiksa): efek `UPSCALE` di `robust_decode()` (`qr_logic.py`
   pipeline preprocessing) yang mengubah skala citra sebelum deteksi corner, kalau tidak
   dikompensasi balik saat menghitung `bw/w, bh/h`.

**Implikasi**: estimasi posisi QR-murni dengan formula pinhole sederhana ini **belum
tervalidasi sebagai sensor posisi yang andal** — bias sistematis di atas berarti angka
`err_m` di tabel tidak boleh dibaca sebagai "akurasi QR sesungguhnya" tanpa pertama
menyelesaikan pertanyaan #1-#3 di atas. Ini sendiri adalah **evidence**, bukan kegagalan
eksperimen: ia menunjukkan reprojection sederhana tidak cukup, bukan bahwa QR "buruk" secara
umum (P0-2.2 Gate 3 sudah menunjukkan QR memang berkontribusi kausal ke command — pertanyaan
di sini murni soal kalibrasi metrik absolut, hal yang berbeda).

Data lengkap: `/tmp/p0-2-2b-battery/P0-2-3-precision-results.json`.

## 9. Non-goals eksplisit untuk pass ini

- Tidak ada run simulator baru dieksekusi untuk §8 — murni reuse `Q1`-`Q6.csv` dari P0-2.2b.
- Tidak ada perbaikan formula/kalibrasi terhadap bias sistematis §8 — dicatat sebagai
  pertanyaan terbuka, bukan ditambal di pass ini.
- Tidak ada perubahan `mission_fsm.py`/`qr_detector.py`/`qr_logic.py`/parameter.
- Tidak ada klaim PASS/FAIL. Acceptance matrix `docs/P0-2-AUDIT.md` §2 tidak diubah.
- Tabel `err_m` (§8) dan tabel `gripper_err` (§1) **tidak pernah digabung** jadi satu klaim.

## 10. Status

```text
P0-2.2      CLOSE-PARTIAL   (QR influence VERIFIED, QR precision convergence OPEN)
P0-2.3      qr_side_m CONFIRMED (§7) — Gate P2/P3 EXECUTED (§8, reuse data, no new run)
  Gate P2 (QR-alone estimate accuracy)   NOT VALIDATED — bias sistematis belum diisolasi
  Gate P3 (repeatability estimat QR)     NOT REPEATABLE — spread 0.093-1.144m lintas run
  Next: isolasi penyebab bias (§8 poin 1-3) sebelum data lebih lanjut dikumpulkan
```
