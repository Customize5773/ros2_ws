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

## 10. Status (superseded oleh §11 — lihat koreksi di bawah)

```text
P0-2.2      CLOSE-PARTIAL   (QR influence VERIFIED, QR precision convergence OPEN)
P0-2.3      qr_side_m CONFIRMED (§7) — Gate P2/P3 EXECUTED (§8, reuse data, no new run)
  Gate P2 (QR-alone estimate accuracy)   NOT VALIDATED — bias sistematis belum diisolasi
  Gate P3 (repeatability estimat QR)     NOT REPEATABLE — spread 0.093-1.144m lintas run
  Next: isolasi penyebab bias (§8 poin 1-3) sebelum data lebih lanjut dikumpulkan
```

**Koreksi penting (lihat §11): klaim "bias sistematis 0.65-0.81m" di §8 TIDAK BOLEH lagi
dibaca sebagai bukti masalah geometrik detector/reprojection** — investigasi lanjutan
menemukan sebagian besar angka itu berasal dari sign error di script reducer sendiri, bukan
sinyal QR. §11 punya angka yang sudah dikoreksi dan status yang menggantikan blok di atas.

---

## 11. Re-reduction dengan koreksi tanda depth (offline, TANPA run simulator baru)

Audit geometri read-only (dieksekusi setelah §8, terpisah dari dokumen ini pada saat itu)
menelusuri seluruh pipeline sinyal QR (`qr_detector.py` → `qr_logic.py` → `mission_fsm.py`)
dan menemukan: cross-check `distance_est` vs `h_cam` di `reduce_qr_precision.py`
memasukkan kolom CSV `sp_depth` langsung ke `h_cam_from_depth()`. Tapi `/hydroships/
setpoint/depth` diterbitkan **negatif** oleh `_set_depth()`
(`mission_fsm.py:319-320`: `m.data = -abs(d_pos)`), sedangkan `qr_ey_target()` — yang
`h_cam_from_depth()` meniru — selalu dipanggil di produksi dengan `depth_target` **positif**
(`mission_fsm.py:534,540`). Ini adalah **bug di script analisis P0-2.3 sendiri**, bukan di
`mission_fsm.py`/`qr_detector.py`/`qr_logic.py`.

**Perbaikan** (satu baris, `tools/p0-experiments/reduce_qr_precision.py`):
```python
h_cam = h_cam_from_depth(-sp_depth)   # sp_depth = -abs(depth_target); negasi untuk cocok
                                       # dengan konvensi produksi (mission_fsm.py:319-320,534,540)
```

**Re-reduction dijalankan** terhadap `Q1`-`Q6.csv` yang SAMA persis (tidak ada run simulator
baru, tidak ada perubahan data mentah):

| Run | `err_m` mean (Gate P2, TIDAK BERUBAH) | `distance_est − h_cam` SEBELUM | `distance_est − h_cam` SESUDAH koreksi |
|---|---|---|---|
| Q1 | 0.093 | −0.652 | **−0.052** |
| Q2 | 0.875 | −0.682 | **−0.082** |
| Q3 | 0.420 | −0.761 | **−0.161** |
| Q4 | 1.144 | −0.272 | **+0.328** |
| Q5 | 1.036 | −0.812 | **−0.212** |
| Q6 | 0.540 | −0.808 | **−0.208** |

Kolom `err_m` (Gate P2) **byte-identical** dengan §8 — dikonfirmasi tidak terpengaruh bug ini
sama sekali (formulanya memakai `payload_x/y, x, y, yaw`, tidak pernah memakai
`sp_depth`/`h_cam`). Hanya kolom cross-check yang berubah.

### Keputusan 3-arah (kriteria pengguna)

- **Bias hilang?** Tidak sepenuhnya — turun drastis dari ~0.65-0.81m ke kisaran
  −0.05 sampai −0.21m (5 run) dan +0.33m (Q4, berbalik tanda).
- **Bias tetap?** Tidak — magnitudonya turun 3-6× lipat, jelas bukan angka yang sama.
- ✅ **AMBIGU** — sebagian besar (dominan) dari bias asli terbukti berasal dari bug reducer
  (sign error), TAPI residual non-trivial (−0.21 sampai +0.33m, tidak konsisten tanda antar
  run, khususnya Q4 berbalik arah) masih tersisa dan belum dijelaskan.

**Kesimpulan**: klaim "bias sistematis detector/reprojection 0.65-0.81m" dari §8 **ditarik
kembali** — itu sebagian besar adalah artefak bug reducer, bukan evidence tentang kualitas
sinyal QR. Yang tersisa sekarang adalah residual lebih kecil (order 0.05-0.33m) yang statusnya
masih **UNKNOWN** — konsisten dengan audit geometri (kandidat #2 rotasi bounding-box, #3
deteksi corner-only/degenerate — keduanya belum terisolasi dari data yang ada).

### Eksperimen minimal berikutnya (didesain, BELUM dieksekusi)

Karena hasilnya ambigu (bukan "hilang total", bukan "tetap besar"), langkah berikutnya per
audit geometri §G adalah **menambah instrumentasi read-only yang sempit**, bukan run battery
baru:
1. Rekam titik sudut mentah QR (4 corner points) atau minimal keliling/diagonal quad —
   `qr_detector.py`/`qr_logic.py` sudah menghitungnya secara internal (`pts` di
   `robust_decode`), tapi tidak pernah dipublikasikan; instrumentasi observability-only bisa
   menambahkannya ke `/hydroships/qr_offset` atau topic baru, TANPA mengubah logika deteksi.
2. Rekam flag "apakah baris qr_offset ini berasal dari kandidat yang berhasil decode atau
   corner-only" per pesan — untuk memisahkan kontribusi kandidat #3 (deteksi degenerate) dari
   kandidat #2 (inflasi rotasi bounding-box).
3. Baru dengan data itu, hitung ulang `err_m`/cross-check dan lihat apakah residual
   −0.21..+0.33m menyempit atau tetap.
Ini **desain**, bukan tindakan — belum ada kode/simulator yang dijalankan untuk butir ini.

## 12. Status (superseded oleh §14 — lihat implementasi instrumentasi di bawah)

```text
P0-2.2      CLOSE-PARTIAL     (QR influence VERIFIED, QR precision convergence OPEN)
P0-2.3      qr_side_m CONFIRMED (§7)
  Gate P2 (QR-alone estimate accuracy)   err_m TIDAK BERUBAH, 0.093-1.144m per-run mean —
                                          spread masih besar, TIDAK REPEATABLE
  Cross-check distance_est vs h_cam      SIGN BUG DIPERBAIKI (§11) — residual turun ke
                                          -0.21..+0.33m (dari -0.65..-0.81m), status AMBIGU
  "Bias sistematis detector/reprojection 0.65-0.81m"   DITARIK — bukan lagi klaim valid
  Next                                    desain instrumentasi corner-point/decode-flag (§11),
                                           BELUM dieksekusi; TIDAK ADA battery baru,
                                           TIDAK ADA perubahan qr_detector.py/qr_logic.py
```

## 13. Desain instrumentasi corner-point/decode-flag — DISETUJUI, DIIMPLEMENTASI

Desain (§11 poin 1-2) direview dan disetujui pengguna. Diimplementasikan sebagai perluasan
**aditif murni**:

- **`qr_detector.py`**: publisher baru `/hydroships/qr_offset_debug`
  (`std_msgs/String`, `"decode_success,x1,y1,x2,y2,x3,y3,x4,y4"`), diterbitkan dari method
  baru `_publish_debug()` yang dipanggil tepat setelah `_publish_offset()` yang sudah ada
  (baris pemanggilan yang sama, `pts`/`data` yang sama — tidak ada perhitungan deteksi baru).
  `qr_logic.py` **tidak disentuh sama sekali** — `robust_decode`/`offset_from_points` identik.
- **`recorder_qr.py`**: subscription baru ke topic tsb, 9 kolom CSV baru
  (`qr_decode_success, qr_c1x..qr_c4y`), diisi dengan pola "nilai terakhir" yang sama seperti
  kolom `qr_*` lain — tidak mengubah kolom/format yang sudah ada.

## 14. Smoke test instrumentasi (SATU run, bukan battery)

Dieksekusi: `run_approach_qr_smoke.sh` dua kali (`I1`, `I2`) di `/tmp/p0-2-3-instr-smoke`,
`kki_arena`, 60s. `I1` gagal gerbang (`stabilizer`/`thruster_allocator`/`mission_fsm`
"missing" + `cmd_vel pub=0`) tapi log `I1` tidak menunjukkan error/traceback apa pun — misi
`I1` justru **selesai penuh** sampai `WAIT_TRIGGER` ~10 detik sebelum gerbang dicek (t+45s),
konsisten dengan flake discovery `ros2 node list` (bukan crash — thrust-publisher-count check,
yang query infrastruktur topic langsung bukan node-list, tetap PASS). `I2` (retry, lingkungan
dikonfirmasi bersih sebelum run — tidak ada proses gz/ROS tersisa) **PASS 7/7 bersih**, misi
sama-sama selesai sampai `WAIT_TRIGGER`, dipakai sebagai evidence verifikasi resmi.

| Kriteria (checklist desain §"Verification plan") | Hasil | Evidence (`I2`) |
|---|---|---|
| Raw corners benar-benar keluar | **CONFIRMED** | 29 baris `qr_decode_success=1` dengan 4 pasang koordinat piksel valid (mis. t=12.374s: `(306.68,117.71),(315.64,41.57),(391.96,49.02),(384.00,125.00)`); 876 baris `qr_decode_success=0` dengan corner tetap terisi (bukan NaN) — kedua kasus terekam |
| `decode_success` konsisten dengan hasil decode | **CONFIRMED** | Baris `decode_success=1` berkorelasi dengan `qr_result='C'` terisi & log "QR terbaca" (di-dedup jadi 1 baris log meski banyak baris CSV — sesuai `qr_detector.py:150`, `if data != self._last_data`); baris `decode_success=0` dengan corner berkorelasi dengan log "DECODE GAGAL: ... pts ada tapi decode kosong" (10 kejadian log dalam window observasi) |
| Timestamp valid | **CONFIRMED** | `_publish_debug()` dipanggil di tick `_on_image()` yang sama dengan `_publish_offset()`, `pts`/`data` yang sama — tidak ada mekanisme sinkronisasi terpisah yang bisa meleset |
| Tidak ada perubahan FSM/controller | **CONFIRMED** | `I2` gerbang 7/7 PASS, urutan transisi FSM sama seperti run-run sebelumnya (`IDLE→DIVE→APPROACH_QR→GRAB→NAV_WALL→HANG→SURFACE→WAIT_TRIGGER`), log `GRAB (gripper_err=0.031m base_err=0.154m...)` format identik dengan run-run P0-2.2b sebelumnya |
| Recorder/reducer tetap baca data lama & baru | **CONFIRMED** | `reduce_approach_qr.py` dan `reduce_qr_precision.py` dijalankan ulang terhadap `Q1.csv` (skema lama, 21 kolom, tanpa kolom baru) — jalan bersih tanpa error, karena keduanya tidak pernah merujuk kolom baru |

**Kesimpulan pass ini**: instrumentasi bekerja sesuai desain, murni aditif, tidak mengubah
perilaku deteksi/FSM/controller. Data corner-point + decode-flag sekarang tersedia untuk
menjawab pertanyaan residual `err_m`/cross-check §11 (rotasi bounding-box vs corner-only
detection) — **analisis itu sendiri BELUM dijalankan** (butuh keputusan terpisah: reuse data
smoke `I2` yang QR-nya cuma terlihat singkat, atau battery baru N-run seperti P0-2.2b untuk
cakupan yang sebanding). Tidak ada verdict P0-2.3 di pass ini.

## 15. Status (superseded oleh §17 — lihat coverage assessment di bawah)

```text
P0-2.2      CLOSE-PARTIAL     (QR influence VERIFIED, QR precision convergence OPEN)
P0-2.3      qr_side_m CONFIRMED (§7); sign bug reducer FIXED (§11, ambigu, residual
             -0.21..+0.33m)
  Instrumentation corner-point/decode-flag   IMPLEMENTED + SMOKE VERIFIED (§13-14)
    qr_detector.py     purely additive, 1 new publisher + 1 new method, qr_logic.py untouched
    recorder_qr.py     9 new CSV columns, old CSVs unaffected
  Next          analisis residual pakai data baru (belum dijalankan — battery-scope TBD)
  Verdict P0-2.3   BELUM DIBERIKAN
```

## 16. Coverage assessment — analisis data instrumentasi `I2` (read-only, tanpa battery/kode baru)

Dijalankan terhadap `I2.csv` (smoke run instrumentasi, §14) untuk menilai apakah data yang
ada cukup untuk menutup pertanyaan presisi P0-2.3, sebelum memutuskan battery baru.

**Statistik cakupan**: 930 baris CSV → 905 baris dengan corner valid (non-NaN) → setelah
dedup bacaan corner yang identik berturut-turut (detektor jalan ≤5Hz sementara recorder
sampling 10Hz, dan sebagian besar durasi run adalah pasca-`GRAB` dengan QR di luar frame),
hanya **28 observasi detektor yang benar-benar independen** sepanjang 60 detik. Dari 28 itu,
hanya **2** berstatus `decode_success=1`; **26** adalah corner-only (`decode_success=0`).

**Temuan baru — inflasi bounding-box terukur langsung, bukan lagi teoretis**: dihitung
`aabb_side / mean_edge_length` (rasio sisi AABB dari `qr_size` terhadap panjang sisi
rata-rata quad sesungguhnya) untuk ke-28 observasi:

| Metrik | Nilai |
|---|---|
| Mean inflation factor | **1.405×** |
| Median | 1.412× |
| Range | 1.098× – 1.834× |
| Implied distance underestimate (pada mean) | **~29%** |
| Sampel `decode_success=1` (n=2) | 1.110×, 1.409× |
| Sampel `decode_success=0` (n=26, mean) | 1.416× |

**Status hipotesis (audit geometri sebelumnya)**:
- Hipotesis #2 (inflasi AABB dari rotasi in-plane QR) — **CONFIRMED** dengan angka terukur
  langsung dari corner asli, bukan lagi dugaan teoretis. Faktor 1.4× rata-rata cukup untuk
  menjelaskan SEBAGIAN residual §11 (~0.09-0.15m dari pita -0.21..+0.33m pada `h_cam` di
  kisaran 0.3-0.5m) — tapi arah efek ini SELALU satu arah (distance underestimate), sehingga
  **tidak bisa** menjelaskan pembalikan tanda Q4 (+0.328m) dengan sendirinya.
- Hipotesis #3 (corner-only/degenerate detection noise) — **TETAP OPEN**. n=2 untuk
  `decode_success=1` terlalu kecil untuk membandingkan kualitas residual sukses-vs-corner-only
  dengan daya statistik apa pun.
- Anomali tanda-balik Q4 — **TETAP UNEXPLAINED**.
- `I2` adalah run terpisah dari `Q1`-`Q6` (kondisi spawn acak berbeda) — 28 observasinya
  membuktikan MEKANISME inflasi itu nyata di pipeline ini, tapi tidak terikat langsung ke
  kontribusi kuantitatifnya pada residual `Q1`-`Q6` yang spesifik.

**Kesimpulan coverage**: cukup untuk mengonfirmasi keberadaan & order-of-magnitude hipotesis
#2, **tidak cukup** untuk memisahkan kontribusi #2 vs #3 secara statistik, tidak cukup untuk
menutup precision question P0-2.3. Battery baru dengan instrumentasi aktif dan cakupan
sebanding `Q1`-`Q6` adalah langkah metodologis berikutnya yang paling kuat — **didesain di
sini sebagai kebutuhan, TIDAK dieksekusi dalam pass ini**.

**Eksplisit TIDAK dilakukan**: tidak ada perubahan `qr_detector.py`/`qr_logic.py` berdasarkan
temuan §16 ini. Instrumentasi membuktikan mekanismenya nyata; belum membuktikan mekanisme itu
menjelaskan residual `Q1`-`Q6` secara kuantitatif — mengubah detector sekarang akan menambal
sistem berdasarkan evidence yang masih parsial, melanggar prinsip yang dipegang sejak P0-1.

## 17. Status (superseded oleh §19 — lihat battery instrumentasi di bawah)

```text
P0-2.2                          CLOSE-PARTIAL  (QR influence VERIFIED, precision OPEN)
P0-2.3                          OPEN
  Instrumentation                 CLOSED   (§13-14, purely additive, smoke verified)
  AABB rotation hypothesis        CONFIRMED (§16) — mean inflation 1.405x, ~29% underestimate
  Corner-only/degenerate noise    OPEN     (§16) — n=2 decode_success terlalu kecil
  Q4 sign-flip anomaly            UNEXPLAINED
  Precision acceptance            OPEN
  qr_detector.py / qr_logic.py    TIDAK DIUBAH — evidence baru kuat tapi masih parsial
  Next                            battery baru + instrumentasi aktif, cakupan sebanding
                                   Q1-Q6 (untuk memisahkan AABB-rotation vs corner-only vs
                                   Q4 residual) — DIDESAIN SEBAGAI KEBUTUHAN, BELUM DIJALANKAN
```

**Koreksi kecil sebelum §18**: angka cakupan `I2` di §16 (28 observasi independen, 2
`decode_success`, inflasi mean 1.405×) dihitung dari SELURUH baris `I2.csv`, bukan dibatasi ke
`fsm_state=='APPROACH_QR'` seperti analisis `err_m` yang lain. Setelah `reduce_qr_precision.py`
diperluas (§18) dengan pembatasan yang konsisten (sama seperti `err_m`), angka `I2` yang benar
adalah **7 observasi independen, 0 `decode_success`, inflasi mean 1.435×** — kesimpulan yang
sama (cakupan smoke test jauh dari cukup), angka pastinya berbeda. Dicatat di sini demi
akurasi, bukan diam-diam diganti.

## 18. Battery P0-2.3 dengan instrumentasi aktif (protocol-comparable terhadap Q1-Q6)

Didesain dan direview sebelum eksekusi (lihat riwayat percakapan) — disetujui dengan opsi
**protocol-comparable**, bukan pose-matched: tidak ada RNG seed yang di-expose untuk memaksa
draw acak identik dengan `Q1`-`Q5`, jadi `R1`-`R5` adalah draw independen dari protokol yang
sama (`rov_random_spawn:=true`), dan `R6` identik secara protokol dengan `Q6` (pose
deterministik tetap). `tools/p0-experiments/run_approach_qr_battery.sh` diperluas dengan
`TAG_PREFIX` (default `Q`, dipakai `R` di sini) — perubahan kosmetik murni, tidak mengubah
protokol run apa pun. `reduce_qr_precision.py` diperluas dengan mode analisis corner (aktif
hanya kalau CSV punya kolom `qr_decode_success` — `Q1`-`Q6` lama tetap dilewati dengan aman,
diverifikasi ulang: hasil `Q1`/`Q4` byte-identical dengan sebelumnya).

**Kondisi spawn tiap run** (dicatat verbatim dari log, untuk komparabilitas):

| Run | ROV spawn (x,y,z,yaw) | Payload (QR huruf, x,y,z) | Gate |
|---|---|---|---|
| R1 | (-2.05, 1.619, -0.5, 0.217 rad) | C, (0.39, -0.25, -0.89) | PASS |
| R2 | (-2.05, -1.085, -0.5, -0.088 rad) | B, (0.30, 0.93, -0.89) | PASS |
| R3 | (-0.769, 2.05, -0.5, -1.857 rad) | C, (0.32, 1.15, -0.89) | PASS |
| R4 | (-0.617, -2.05, -0.5, 1.588 rad) | D, (0.40, 0.67, -0.89) | PASS |
| R5 | (-0.462, 2.05, -0.5, -1.477 rad) | C, (0.37, -0.88, -0.89) | PASS |
| R6 | (0.0, 0.0, -0.5, 0.0 rad) — deterministik | D, (0.30, 0.79, -0.89) | PASS |

6/6 gate PASS, 0 `INCONCLUSIVE`.

### Hasil per run

| Run | `err_m` mean | obs independen | `decode_success` | inflasi mean | residual RAW | residual KOREKSI-inflasi | sign-flip? |
|---|---|---|---|---|---|---|---|
| R1 | 1.228 | 7 | 0/7 | 1.504× | −0.222 | −0.107 | **Ya** |
| R2 | 1.082 | 8 | 0/8 | 1.401× | −0.226 | −0.149 | Tidak |
| R3 | 0.712 | 5 | 0/5 | 1.438× | −0.287 | −0.231 | Tidak |
| R4 | 1.063 | 9 | 3/9 | 1.245× | −0.140 | −0.100 | Tidak |
| R5 | 1.161 | 7 | 1/7 | 1.411× | −0.056 | +0.092 | **Ya** |
| R6 | 0.352 | 6 | 3/6 | 1.119× | −0.005 | +0.022 | **Ya** |

### Agregat (6 run)

- **Total observasi independen**: 42 (7-9 per run — jauh lebih baik dari `I2`'s 7, tapi masih
  tipis untuk klaim statistik kuat).
- **`decode_success` total**: 7/42 (17%); 3 dari 6 run (R4, R5, R6) punya observasi
  `decode_success=1` sama sekali, 3 run lain (R1-R3) nol.
- **Inflasi AABB**: mean-per-run berkisar 1.119×-1.504×, mean-lintas-run 1.353× — **konsisten
  dengan `I2` (1.435×)**, menguatkan Hipotesis #2 sebagai efek nyata dan cukup stabil.
- **Sign-flip (Q4-style)**: muncul di **3/6 run (R1, R5, R6)** — bukan anomali langka satu
  run. Q4 dari battery lama BUKAN outlier tunggal; pembalikan tanda residual adalah pola yang
  cukup sering muncul pada kondisi yang beragam.
- **Residual RAW vs KOREKSI-inflasi**: mean-lintas-run RAW = −0.156, KOREKSI = −0.079 (stdev
  0.100 → 0.107, relatif tak berubah tapi rata-rata magnitudo turun ~50%) — koreksi inflasi
  AABB **memangkas kira-kira separuh** residual mean, konsisten dengan Hipotesis #2 menjelaskan
  sebagian substansial tapi tidak semua.
- **Residual dipisah `decode_success`**: mean(residual | `decode_success=1`) = **+0.009**
  (hampir nol!) vs mean(residual | corner-only) = **−0.176** — perbedaan besar dan konsisten
  arah. **Ini evidence kuat untuk Hipotesis #3**: observasi corner-only/decode-gagal membawa
  bias jauh lebih besar daripada observasi yang benar-benar berhasil decode; untuk decode yang
  sukses, model pinhole (bahkan tanpa koreksi inflasi) sudah hampir tepat.

### Interpretasi (evidence, BUKAN verdict)

Ketiga pertanyaan yang tadinya OPEN sekarang punya jawaban parsial yang jauh lebih kuat:

1. **AABB rotation/inflation** — CONFIRMED sebelumnya (§16), sekarang **dikuantifikasi pada
   kondisi sebanding**: mean 1.353× lintas 6 run, cukup stabil (range per-run 1.12-1.50×).
2. **Corner-only/decode-failure noise** — sebelumnya OPEN, sekarang ada **evidence langsung**:
   residual corner-only (−0.176) jauh lebih besar dari residual decode-success (+0.009).
   Bukan sekadar dugaan lagi — meski n=7 untuk `decode_success` masih kecil untuk generalisasi
   penuh.
3. **Q4 sign-flip** — sebelumnya UNEXPLAINED/dianggap mungkin outlier, sekarang **muncul lagi
   di 3/6 run baru** — pola berulang, bukan kebetulan satu run. Penyebab PASTI-nya (kombinasi
   inflasi rendah + noise decode, atau sesuatu yang lain) masih belum diisolasi lebih jauh
   dalam pass ini.

**Tidak ada verdict P0-2.3 di sini.** Evidence sekarang jauh lebih kuat untuk keduanya
(inflasi AABB dan corner-noise), tapi `qr_detector.py`/`qr_logic.py` tetap TIDAK diubah —
sesuai instruksi eksplisit, mengubah detector sekarang masih akan mendahului evidence yang
walau kuat, coverage-nya (n=42 observasi, n=7 decode_success) belum sebesar yang idealnya
dipakai untuk keputusan desain permanen.

Data lengkap: `/tmp/p0-2-3-battery/*.csv`, `*.log`, `P0-2-3-precision-results.json`.

## 19. Status (superseded oleh §21 — lihat koreksi analisis korelasi di bawah)

```text
P0-2.2                          CLOSE-PARTIAL   (QR influence VERIFIED, precision OPEN)
P0-2.3                          OPEN
  Instrumentation                  CLOSED
  Battery instrumentasi aktif      CLOSED   (§18, 6/6 gate PASS, protocol-comparable Q1-Q6)
  AABB rotation hypothesis         CONFIRMED & QUANTIFIED  — mean 1.353x lintas 6 run
  Corner-only/degenerate noise     STRONG EVIDENCE  — residual corner-only -0.176 vs
                                    decode-success +0.009 (n=7 decode_success, masih tipis)
  Q4 sign-flip anomaly             REPRODUCED  — muncul di 3/6 run baru, bukan outlier tunggal,
                                    penyebab pasti masih belum diisolasi
  Precision acceptance             OPEN
  qr_detector.py / qr_logic.py     TIDAK DIUBAH
  P0-2.3 verdict                   BELUM DIBERIKAN — evidence jauh lebih kuat, coverage masih
                                    dianggap belum cukup untuk keputusan desain permanen
```

## 20. KOREKSI — korelasi `qr_size`/`1/qr_size` bersifat tautologis, BUKAN evidence

Analisis lanjutan (read-only, terhadap 42 observasi independen dari `R1`-`R6`, §18) mencoba
mengidentifikasi correlate sign-flip yang paling kuat. Ditemukan `r(dist_diff_raw,
1/qr_size)=+1.000` dan regresi berganda `R²=1.000` dengan koefisien `inflation`/`angle`/
`decode_success` ≈0. **Angka-angka ini SALAH DIBACA sebagai evidence dan harus ditarik.**

**Alasan**: `distance_est = K/qr_size` (K = `FX_PX × QR_SIDE_M / FRAME_W_PX`, konstanta), jadi
`dist_diff_raw = distance_est - h_cam = K/qr_size - h_cam` **secara aljabar** hampir linear
sempurna terhadap `1/qr_size` (`h_cam` nyaris konstan sepanjang satu episode `APPROACH_QR`,
karena `depth_target` jarang berubah). Korelasi `r=+1.000` hanya menemukan kembali definisi
rumusnya sendiri — bukan pola empiris apa pun. Regresi berganda `R²=1.000` dengan koefisien
prediktor lain ≈0 adalah tautologi yang sama menjenuhkan fit, **bukan bukti bahwa
`inflation`/`angle`/`decode_success` tidak berpengaruh**.

**ATURAN untuk pass berikutnya**: angka `r=+1.000`/`r=-0.883` (`qr_size`/`1/qr_size` vs
`dist_diff_raw`) dan `R²=1.000` dari regresi tersebut **TIDAK BOLEH** dikutip sebagai evidence
di P0-2.3 manapun setelah ini. Hipotesis "QR kecil menyebabkan noise/sign-flip" dari §16-§19
**ditarik** — crossover residual di sekitar nol bisa muncul murni dari struktur rumus
(`qr_size* ≈ K/h_cam`, titik di mana `distance_est` menyeberangi `h_cam`), bukan berarti
bukti kualitas deteksi memburuk pada QR kecil.

### Yang tetap valid (korelasi non-tautologis, dihitung ulang dari 41 observasi valid — 1
### outlier degenerat `R4` dikeluarkan, lihat §20.1)

| Correlate | r vs `dist_diff_raw` | Kekuatan sinyal |
|---|---|---|
| `decode_success` | **+0.442** | Satu-satunya correlate independen dengan sinyal cukup jelas — konsisten dengan pemisahan mean residual sebelumnya (+0.009m decode-success vs −0.176m corner-only, §18-19) |
| `inflation` | −0.269 | Lemah — belum cukup untuk klaim kausalitas |
| `angle` (rotasi) | −0.132 | Lemah — belum cukup untuk klaim kausalitas |

Stratifikasi (`1/qr_size` vs `dist_diff_raw` dalam strata inflasi rendah/tinggi dan sudut
rendah/tinggi) semuanya kembali menunjukkan `r≈+1.000` di setiap strata — ini murni
mengulang tautologi yang sama di setiap subset, **tidak informatif**, tidak dilaporkan
sebagai evidence robustness.

### §20.1 — Outlier `R4` (1 dari 42)

Terkonfirmasi satu kejadian degenerat: `R4 t=6.864`, satu titik sudut di `y≈-29968` (~30.000
piksel di luar frame 480px tinggi). **Penyebabnya masih belum diketahui.** Menghapusnya
menggeser mean `dist_diff_raw` run `R4` dari **−0.140 → −0.106 m** — pergeseran nyata tapi
kecil, tidak mengubah kesimpulan kualitatif run tersebut.

### §20.2 — R6 vs R1-R5 (deskriptif, BUKAN bukti mekanisme noise baru)

| | R6 (n=6) | R1-R5 (n=35, valid) |
|---|---|---|
| mean `dist_diff_raw` | −0.005 | −0.172 |
| mean `qr_size` | 0.208 | 0.470 |
| sign-flip rate | 5/6 (83%) | 4/35 (11%) |

Perbedaan run-level ini nyata secara deskriptif, tapi per §20 di atas, **bisa sepenuhnya
dijelaskan oleh ROV berada di sisi `qr_size` yang melewati titik crossover rumus untuk
sebagian besar run itu** — bukan otomatis bukti mekanisme noise yang berbeda di `R6`.

### Kesimpulan yang aman dari seluruh analisis korelasi (§16-§20)

- **`decode_success` tetap satu-satunya correlate yang punya dukungan independen dan
  konsisten** (muncul dua kali: pemisahan mean §18-19, dan korelasi §20) — data cukup untuk
  bilang observasi corner-only membawa bias residual lebih besar daripada observasi
  decode-success.
- **Belum cukup untuk menentukan PENYEBAB** kualitas corner-only yang lebih buruk itu —
  apakah rotation/inflasi AABB, noise deteksi corner, atau karakteristik lain — tidak
  terjawab oleh analisis ini.
- Hipotesis rotasi/inflasi sebagai penjelasan utama sign-flip: correlate lemah, tidak
  didukung kuat.
- Hipotesis "QR kecil menyebabkan sign-flip/noise": **ditarik**, kemungkinan besar artefak
  struktur rumus, bukan temuan tentang kualitas deteksi.
- **Tidak ada patch detector/controller yang disarankan dari hasil ini. P0-2.3 tetap OPEN.**

## 21. Status (menggantikan §19)

```text
P0-2.2                          CLOSE-PARTIAL   (QR influence VERIFIED, precision OPEN)
P0-2.3                          OPEN
  Instrumentation                  CLOSED
  Battery instrumentasi aktif      CLOSED   (§18, 6/6 gate PASS, protocol-comparable Q1-Q6)
  AABB rotation hypothesis         CONFIRMED (mean inflation 1.353x) tapi correlate lemah
                                    terhadap sign-flip (r=-0.269, §20) — bukan penjelasan utama
  qr_size/1/qr_size correlation    DITARIK — tautologis terhadap rumus distance_est=K/qr_size,
                                    r=+1.000/R²=1.000 TIDAK BOLEH dipakai sebagai evidence (§20)
  decode_success correlate         SATU-SATUNYA sinyal independen konsisten — r=+0.442,
                                    residual +0.009m (success) vs -0.176m (corner-only)
  Penyebab kualitas corner-only    OPEN — rotation/AABB vs corner noise vs lainnya belum
                                    terpisahkan
  R4 degenerate outlier            CONFIRMED, 1/42, penyebab UNKNOWN, efek pada mean R4 kecil
                                    (-0.140 -> -0.106 setelah dikeluarkan)
  R6 vs R1-R5                      Beda deskriptif nyata, TAPI bisa dijelaskan crossover rumus,
                                    bukan bukti mekanisme noise terpisah
  Precision acceptance             OPEN
  qr_detector.py / qr_logic.py     TIDAK DIUBAH
  P0-2.3 verdict                   BELUM DIBERIKAN
```
