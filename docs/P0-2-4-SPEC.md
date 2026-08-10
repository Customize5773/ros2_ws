# P0-2.4 SPEC — APPROACH_QR precision convergence retest (Gate 4) (KKI 2026)

Dokumen ini adalah **desain eksperimen, BUKAN eksekusi**. Tidak ada kode yang diubah, tidak ada
battery baru dijalankan, tidak ada perubahan pada `qr_detector.py`, `qr_logic.py`,
`mission_fsm.py`, controller, atau parameter apa pun dalam pembuatan dokumen ini. Tujuannya:
merancang retest untuk pertanyaan yang **masih FAIL/OPEN** setelah
[`docs/P0-2-3-ACCEPTANCE-REVIEW.md`](P0-2-3-ACCEPTANCE-REVIEW.md) — precision convergence
(Gate 4, diwarisi dari P0-2.2b) — memakai evidence root-cause P0-2.3 sebagai konteks analisis,
bukan sebagai pengganti retest.

## 0. Posisi dalam rantai P0-2

```text
P0-2.3 root-cause              CLOSE-PARTIAL  (docs/P0-2-3-ACCEPTANCE-REVIEW.md §4)
        ↓
P0-2.4 (dokumen ini)           mengukur ULANG convergence APPROACH_QR — BELUM dieksekusi
        ↓ metrik: qr_center_tol band + time-to-converge + residual trajectory +
          oscillation/divergence
        ↓
Keputusan perubahan controller/detector/model   DI LUAR SCOPE dokumen ini — menunggu hasil P0-2.4
```

P0-2.3 menjawab **kenapa** ada residual bias (AABB/inflasi DAN kualitas decode, keduanya
berkontribusi independen). P0-2.4 menjawab pertanyaan yang berbeda dan belum pernah diuji ulang
sejak P0-2.2b: **apakah `APPROACH_QR` benar-benar konvergen ke acceptance region**, seberapa
cepat, dan apakah stabil (bukan oscillatory/divergent). Evidence P0-2.3 dipakai di §7 sebagai
alat interpretasi kalau Gate 4 tetap FAIL — bukan dipakai untuk mengklaim convergence sudah
teratasi.

## 1. Pertanyaan yang diuji ulang (Gate 4, P0-2.2b)

Dari `docs/P0-2-AUDIT.md` §7: **"QR precision convergence OPEN — Gate 4: 0/6 run masuk band
`qr_center_tol`"**. Definisi band, dikutip persis dari
`src/hydroships_control/hydroships_control/mission_fsm.py`:

- **Band visual-servo** (L593-596): `centered = off_fresh AND |qr_ex| < qr_center_tol AND
  |qr_ey - ey_target| < qr_center_tol`, dengan `qr_center_tol = 0.12` (unit ternormalisasi
  offset piksel, param deklarasi disebut di `docs/P0-2-AUDIT.md` §1.5). `ey_target` dihitung
  `qr_ey_target()` (L540-542, definisi fungsi L55-78 per audit sebelumnya).
- **Fallback jarak XY** (L596, L603): `dist < approach_tol`, `approach_tol = 0.06` m, `dist`
  adalah nilai balik `self._goto_xy(tx, ty)` (L569) — jarak Euclidean `(x,y)` ROV ke target
  `(tx,ty)` di sumbu dunia.
- Salah satu kondisi terpenuhi → transisi ke `GRAB`. P0-2.4 mengukur **kapan** (jika pernah)
  salah satu kondisi ini terpenuhi selama window `APPROACH_QR`, bukan hanya apakah transisi
  akhirnya terjadi (transisi bisa terjadi lewat fallback jarak XY tanpa QR pernah benar-benar
  "centered" — risiko yang sudah dicatat `docs/P0-2-AUDIT.md` §3.2).

## 2. Instrumentasi — TIDAK ADA YANG BARU

Seluruh kolom yang dibutuhkan sudah direkam oleh `tools/p0-experiments/recorder_qr.py`
(10 Hz, header CSV):

```
t,fsm_state,sp_depth,depth,x,y,z,yaw,vx,vy,cmd_fx,cmd_fy,qr_result,qr_ex,qr_ey,qr_size,
qr_frame,qr_age,payload_x,payload_y,payload_z,qr_decode_success,qr_c1x,qr_c1y,...,qr_c4y
```

Tidak perlu recorder baru, tidak perlu topic baru. Kolom yang dipakai desain ini: `t`,
`fsm_state` (untuk mengisolasi window `APPROACH_QR`), `qr_ex`, `qr_ey`, `x`, `y`, `yaw`,
`cmd_fx`, `cmd_fy`, `qr_age`.

`tools/p0-experiments/run_approach_qr_battery.sh` (protokol 6 run: 5×
`rov_random_spawn:=true` + 1 deterministik) dan `run_approach_qr_smoke.sh` juga sudah ada dan
tidak perlu diubah — dipakai apa adanya kalau/ketika battery disetujui secara terpisah (lihat
§9, non-goals).

## 3. Metrik baru yang harus dihitung (belum ada di `reduce_approach_qr.py` saat ini)

Gate 4 yang ada sekarang di `reduce_approach_qr.py` hanya menghitung `trend(err_ex)`/
`trend(err_ey)` dan jumlah zero-crossing (`sign_changes`), plus flag boolean
`entered_qr_center_tol_band`. Tidak ada time-to-converge, tidak ada variance/overshoot. Desain
ini menambah tiga kelas metrik (implementasi kode-nya sendiri **di luar scope pass ini** — lihat
§9):

### 3.1 Time-to-converge

Timestamp pertama `t_conv` relatif terhadap entry `APPROACH_QR` di mana kondisi centered ATAU
`dist < approach_tol` (§1) bertahan selama **dwell minimum** — bukan satu sampel tunggal yang
kebetulan masuk band (mencegah flicker noise dihitung sebagai convergence). Dwell minimum
diusulkan: **≥3 tick berturut-turut pada 10 Hz (≥0.3 s)** — dipilih sebagai ambang minimal yang
tetap membedakan noise sesaat dari kondisi yang benar-benar bertahan, bukan hasil tuning
empiris. Kalau band tidak pernah dimasuki sebelum `t_scan` (45 s) atau sebelum transisi FSM
keluar `APPROACH_QR`, `t_conv = None` (run tidak konvergen).

### 3.2 Residual trajectory

Deret waktu `dist(t)`, `|qr_ex(t)|`, `|qr_ey(t) - ey_target(t)|` sepanjang window
`APPROACH_QR` (dari transisi masuk sampai keluar, via kolom `fsm_state`), dihitung ulang
`ey_target` persis seperti `reduce_approach_qr.py` sudah melakukan untuk Gate 2/3 (duplikasi
`qr_ey_target()`, `mission_fsm.py:55-78`). Dilaporkan sebagai tabel/plot per-run, bukan
ringkasan tunggal — dipakai untuk membedakan run yang mendekat monoton dari run yang berosilasi
atau divergen (§3.3).

### 3.3 Oscillation / divergence

- **Zero-crossing count** — sudah ada di Gate 4 existing, dipertahankan.
- **Variance/stdev `cmd_fx`, `cmd_fy`** (surge/sway, N) selama window `APPROACH_QR` — proxy
  osilasi kontrol. Belum ada di kode manapun.
- **Overshoot**: setelah `dist` pertama kali turun di bawah `approach_tol`, excursion maksimum
  `dist` sesudahnya (0 kalau tidak pernah turun di bawah tolerance sama sekali).
- **Kriteria divergen eksplisit** (didefinisikan di muka, §7): `dist` bertren naik untuk seluruh
  window `APPROACH_QR` (`trend(dist) > 0` dengan window ≥ setengah durasi state), ATAU
  `cmd_fx`/`cmd_fy` saturasi di `approach_fmax` (16 N, `mission_fsm.py` §1.4 audit) untuk
  ≥80% tick dalam window — indikasi controller terus mendorong maksimum tanpa progres.

## 4. Variabel per run (agregat, bukan per-observasi seperti P0-2.3)

Berbeda dari P0-2.3 (yang bekerja per-observasi corner independen), P0-2.4 bekerja per-run:
setiap run `APPROACH_QR` menghasilkan satu baris ringkasan: `t_conv` (atau `None`),
`entered_band` (bool), `dwell_ok` (bool), `zero_crossings_ex`, `zero_crossings_ey`,
`stdev_cmd_fx`, `stdev_cmd_fy`, `overshoot_dist`, `diverged` (bool, §3.3), `exit_path`
("visual servo" vs "jarak XY" vs "timeout/ABORT", dari log `_to()`), `qr_decode_rate` (fraksi
tick `qr_decode_success=1` dalam window — link ke evidence P0-2.3 untuk interpretasi §7).

## 5. Grup kontrol/perbandingan

- Tidak ada ablation — observasi pasif natural, sama seperti pola P0-2.2b/P0-2.3.
- Protokol run tetap: 5× `rov_random_spawn:=true` + 1× deterministik per batch, seperti
  `run_approach_qr_battery.sh` yang sudah ada.

## 6. Jumlah run / stopping rule (desain, BELUM dieksekusi)

P0-2.2b hanya punya N=6 run dan Gate 4 sudah 0/6 — sampel terlalu kecil untuk membedakan
"selalu gagal" dari "gagal karena variasi kondisi spawn/QR quality yang kebetulan buruk di 6
run itu". Diusulkan (sejalan dengan pola stopping-rule P0-2.3 §6):

- Jalankan dalam batch 6 run, protokol sama seperti sebelumnya.
- **Stopping rule**: berhenti begitu salah satu terpenuhi — (a) total run kumulatif ≥18 (3
  batch), ATAU (b) sudah ada ≥5 run yang `entered_band=True` dengan dwell terpenuhi (cukup untuk
  mulai menandai pola "kadang konvergen"), ATAU (c) 18 run habis dan masih 0 yang konvergen
  (cukup evidence untuk FAIL definitif, bukan under-sampling).
- Maksimum 3 batch (~18 run) — dipilih agar sebanding dengan skala battery P0-2.3 (12 run),
  sedikit lebih besar karena Gate 4 sudah pernah gagal total (0/6) dan butuh lebih banyak bukti
  sebelum menyimpulkan FAIL definitif vs INCONCLUSIVE karena under-sampling.

**Ini bukan otorisasi menjalankan battery** — angka di atas adalah desain yang menunggu
persetujuan terpisah, persis seperti §6 `P0-2-3-SEPARATION-SPEC.md` sebelum §13-nya dieksekusi.

## 7. Kriteria interpretasi (didefinisikan di muka)

- **Gate 4 PASS**: mayoritas run (>50%) mencapai `entered_band=True` dengan dwell terpenuhi
  sebelum `t_scan`, DAN tidak ada run yang `diverged=True`.
- **Gate 4 FAIL**: dua kondisi terpenuhi bersamaan — (a) minoritas run (≤50%, termasuk 0)
  mencapai band, DAN (b) jumlah run kumulatif sudah mencapai batas stopping rule §6 (bukan
  sekadar N kecil yang kebetulan gagal semua).
- **Gate 4 INCONCLUSIVE**: stopping rule §6 belum terpenuhi (data belum cukup), ATAU hasil
  campuran yang tidak stabil secara arah antar batch (mis. batch 1 mayoritas gagal, batch 2
  mayoritas konvergen, tanpa perbedaan kondisi yang jelas).
- **Interpretasi tambahan kalau FAIL** (memakai evidence P0-2.3 sebagai lensa, bukan
  menggantikan pengukuran baru): stratifikasi run yang gagal konvergen berdasarkan
  `qr_decode_rate` (§4) — kalau run berdecode-tinggi tetap gagal konvergen, itu mengarah ke
  masalah **controller/gain** (bukan sekadar noise deteksi, karena P0-2.3 sudah menunjukkan
  residual `decode_success=1` kecil, ±0.01-0.04 m, jauh di bawah gap yang perlu dijelaskan
  kegagalan Gate 4 total). Kalau run gagal konvergen justru terkonsentrasi pada
  `qr_decode_rate` rendah, itu konsisten dengan mekanisme corner-only/inflasi yang sudah
  dievidence P0-2.3 §14 sebagai kontributor dominan.

## 8. Kriteria inconclusive (didefinisikan di muka)

- Stopping rule §6 belum tercapai dalam batch yang sudah dijalankan.
- Kontaminasi gate (`gate_mission.sh` FAIL, contoh: proses sim crash/tidak start) pada ≥1 run —
  run itu dikeluarkan dari agregat, dilaporkan terpisah, tidak dihitung ke arah PASS/FAIL.
- Satu run mendominasi porsi tak proporsional dari run yang `entered_band=True` (pola outlier
  `R4`/`S4`/`T2` yang sudah muncul di battery P0-2.3 sebelumnya) — hasil dilaporkan dengan DAN
  tanpa run dominan itu, mengikuti aturan anti-dominasi yang sama seperti
  `P0-2-3-SEPARATION-SPEC.md` §8.
- Dua metrik utama (dwell-band entry vs `dist`-fallback entry) tidak sepakat arah pada mayoritas
  run — dilaporkan campuran, bukan dipaksa satu kesimpulan.

## 9. Non-goals eksplisit untuk dokumen ini

- **Tidak ada eksekusi battery apa pun** dalam pass ini — desain saja, sama seperti
  `P0-2-3-SEPARATION-SPEC.md` §1-§10 sebelum direview dan disetujui terpisah.
- **Tidak ada perubahan kode** — `reduce_approach_qr.py` belum diberi metrik §3 (time-to-
  converge, variance, overshoot); implementasi kodenya sendiri adalah pass terpisah yang
  menunggu review desain ini, bukan bagian dari dokumen ini.
- **Tidak ada perubahan `qr_detector.py`, `qr_logic.py`, `mission_fsm.py`, controller, atau
  parameter apa pun** (`approach_kp`, `approach_kd`, `qr_servo_gain`, `qr_center_tol`,
  `approach_tol`, dst) — keputusan tuning/perubahan itu eksplisit menunggu hasil pengukuran
  Gate 4 yang didesain di sini, bukan diasumsikan dari evidence root-cause P0-2.3 saja.
- **Tidak ada recorder/topic baru** — `recorder_qr.py` sudah cukup (§2).
- **Tidak mengubah verdict P0-2.3** (`docs/P0-2-3-ACCEPTANCE-REVIEW.md`,
  `docs/P0-2-3-SEPARATION-SPEC.md`) — dokumen ini hanya membaca evidence-nya sebagai konteks
  interpretasi (§7), tidak menulis ulang.

## 10. Status (lihat hasil lengkap di `docs/P0-2-4-RESULTS.md`)

```text
P0-2.3                              CLOSE-PARTIAL  (docs/P0-2-3-ACCEPTANCE-REVIEW.md)
P0-2.4                               CLOSED — Gate 4 verdict: FAIL
  Retest question                     Gate 4 (P0-2.2b) precision convergence — DIUJI ULANG
  Instrumentasi                       TIDAK BARU — recorder_qr.py dipakai apa adanya (§2)
  Metrik baru (time-to-converge,      DIIMPLEMENTASI di reduce_approach_qr.py (§3), lihat
    residual trajectory, oscillation) docs/P0-2-4-RESULTS.md §1
  Battery retest                      DIJALANKAN — 18 run (3 batch U/V/W), 17 valid,
                                       stopping rule §6 terpenuhi (entered=5>=5)
  Gate 4 verdict                      FAIL — 5/17 (29%) entered+held band, vs 0/6 P0-2.2b;
                                       konsisten arah, sampel lebih besar (docs/P0-2-4-RESULTS.md §5)
  Oscillation/divergence              TIDAK TERBUKTI — 0/17 diverged, overshoot=0 di semua run
  qr_detector.py / qr_logic.py /      TIDAK DIUBAH
    mission_fsm.py / controller
  Rekomendasi engineering fix         TIDAK ADA — di luar scope, keputusan terpisah menyusul
```
