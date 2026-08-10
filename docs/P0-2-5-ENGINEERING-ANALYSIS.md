# P0-2.5 ENGINEERING ANALYSIS & DESIGN — why Gate 4 is 5/17 (KKI 2026)

**STATUS: DIAGNOSIS + DESIGN ONLY. TIDAK ADA KODE YANG DIUBAH.** Dokumen ini berhenti di
langkah 2 dari protokol yang disepakati (Diagnosis & Trace → Design Fix → **STOP, tunggu
approval** → Implementation → Acceptance Battery → Baseline Comparison). Tidak ada perubahan
`qr_detector.py`, `qr_logic.py`, `mission_fsm.py`, controller, atau parameter di pass ini —
seluruh angka di bawah dikutip dari data battery `/tmp/p0-2-4-battery` (`docs/P0-2-4-RESULTS.md`)
dan evidence `docs/P0-2-3-SEPARATION-SPEC.md`, ditambah satu analisis read-only baru terhadap
CSV mentah yang sama (§0) — tidak ada battery baru dijalankan untuk dokumen ini.

**Baseline yang dibandingkan**: Gate 4 = **5/17 (29%)** run `entered_band_with_dwell`
(`docs/P0-2-4-RESULTS.md` §4-5), 0/17 diverged, overshoot=0 di semua run.

## 0. Analisis tambahan (read-only) yang mendasari dokumen ini

Untuk menjawab "apa yang terjadi pada 71% run yang gagal", CSV mentah 17 run valid
(`U2-U6,V1-V6,W1-W6`) dibaca ulang (bukan cuma ringkasan agregat) untuk menghitung per-run:
`min_dist_target` (jarak minimum ke target yang mungkin sudah digeser servo, dihitung persis
seperti `dist` di `mission_fsm.py:569`), `final_dist` (nilai di baris terakhir `APPROACH_QR`),
`off_fresh_frac` (fraksi tick offset QR masih segar, `qr_age < qr_max_age`), `servo_frac`
(fraksi tick visual servo aktif, `off_fresh AND dist_raw < 0.3`), dan `window_s` (durasi state).

Temuan kunci (tabel lengkap tersedia di riwayat sesi, dikutip di §1):

- **Jarak fisik nyaris selalu tercapai**: `min_dist_target ≤ 0.11 m` di **16/17 run** (hanya
  `U5`=0.0996 dan `W1`=0.1138 yang agak lebih jauh, tetap <0.12). Ini jauh lebih baik dari yang
  disiratkan verdict FAIL 5/17 — **ROV secara fisik memang sampai dekat target di hampir semua
  run**, bukan gagal mendekat sama sekali.
- **Tapi tidak bertahan**: `final_dist` (nilai saat state berakhir) sering **lebih besar**
  dari `min_dist_target` yang sempat dicapai (mis. `U2`: min=0.072 vs final=0.103; `U3`:
  min=0.066 vs final=0.125; `V2`: min=0.089 vs final=0.095; `V3`: min=0.067 vs final=0.100) —
  run-run ini **sempat masuk dekat tolerance lalu menjauh lagi sebelum exit**, bukan divergen
  liar (§4 `P0-2-4-RESULTS.md` sudah mengkonfirmasi 0 divergensi/overshoot besar), melainkan
  **berosilasi tepat di sekitar batas tolerance**.
- **`off_fresh_frac` berkorelasi kuat dengan convergence**: `r(off_fresh_frac,
  entered_band_with_dwell) = +0.653` (n=17); mean `off_fresh_frac` run yang konvergen = **0.880**
  vs run yang tidak konvergen = **0.521**. `r(servo_frac, entered_band) = +0.463` — arah sama,
  lebih lemah.
- **6/17 run (`U3,U5,W1,W4,W5,W6`) punya `qr_decode_rate=0.000` — QR TIDAK PERNAH ter-decode
  sama sekali** sepanjang episode `APPROACH_QR`, exit lewat `GROUND_TRUTH_FALLBACK`. Untuk
  run-run ini, precision convergence yang "berhasil" (`W1`, konvergen di t=0.00s) sepenuhnya
  kebetulan — ROV sudah dekat target dari homing PD ground-truth semata, **visual servo tidak
  pernah berperan**.

**Caveat metodologis**: rekonstruksi `dist`/`min_dist_target` di sini adalah duplikasi logika
`mission_fsm.py` di `reduce_approach_qr.py` (pendekatan `locked_yaw` di baris pertama
`APPROACH_QR`, offset servo dihitung ulang dari kolom CSV), bukan pembacaan langsung variabel
internal FSM — kemungkinan sedikit meleset dari nilai persis yang dipakai FSM pada tick transisi
yang sama (kolom `dist` sendiri tidak direkam `recorder_qr.py`). Ini tidak mengubah pola besar
(gap antara `min_dist_target` dan `final_dist` di banyak run konsisten dan besar, jauh dari
noise rekonstruksi 1-tick), tapi disebutkan eksplisit sebagai keterbatasan.

## A. Root Cause Trace

### A.1 — Dua mode kegagalan yang berbeda, bukan satu

Data §0 memisahkan 71% run gagal (12/17) menjadi **dua mekanisme berbeda**, bukan satu akar
masalah tunggal:

**Mode 1 — QR tidak pernah terdecode (6/17: `U3,U5,W1,W4,W5,W6`, tapi `W1` justru masuk hitungan
"entered_band" karena homing ground-truth kebetulan cukup dekat)**. `qr_decode_rate=0.000`
sepanjang episode. Untuk mode ini, precision convergence **bukan pertanyaan controller sama
sekali** — visual servo (`mission_fsm.py:559-567`) tidak pernah aktif karena syarat
`off_fresh` (`qr_age < qr_max_age=1.5s`) tidak pernah terpenuhi. FSM sepenuhnya bergantung pada
PD homing ke `payload_pose` ground-truth (radius `approach_tol=0.06m`, isotropik) — akurasi
"visual" dari run-run ini nol secara definisi. Ini konsisten dengan evidence P0-2.3
(`docs/P0-2-3-SEPARATION-SPEC.md` §14): total 17/58 observasi `decode_success=1` di battery
P0-2.3, mean rate ~29% per-observasi, tapi tersebar sangat tidak rata antar run/kondisi — di
P0-2.4 malah 6/17 *run* (bukan observasi) dapat nol decode sama sekali di seluruh episode.

**Mode 2 — QR terdecode sebagian, tapi target berosilasi di sekitar tolerance (11/17 sisanya)**.
`min_dist_target` mendekati/masuk tolerance di hampir semua run ini, tapi `final_dist` sering
menjauh lagi (§0). Trace matematis:

- Target visual-servo `(tx,ty)` dihitung **ULANG SETIAP TICK** langsung dari `qr_ex`, `qr_ey`
  mentah (`mission_fsm.py:559-567`, `body_dx = -(ey-ey_target)*k`, `body_dy = -ex*k`,
  `k=qr_servo_gain=0.15`) — **tidak ada filter/smoothing** (bukan EMA, bukan low-pass, bukan
  rejection outlier) sebelum offset dipakai menggeser target.
- `qr_ex`/`qr_ey` sendiri, dari evidence P0-2.3 (`docs/P0-2-3-SEPARATION-SPEC.md` §14), membawa
  bias residual signifikan pada observasi corner-only/inflasi-tinggi (−0.148 s.d. −0.188m
  setara jarak, `SEPARATION-SPEC.md:296-304`) dan gap tidak-nol bahkan pada observasi
  `decode_success=1` yang "baik" (+0.09 s.d. +0.14m gap vs corner-only). **Setiap kali offset
  QR yang noisy ini dipakai, target `(tx,ty)` bergeser** — ROV mengejar target yang bergerak,
  bukan titik tetap. Ini secara matematis konsisten dengan pola §0: `dist` turun mendekati
  tolerance saat kebetulan offset QR kecil/akurat, lalu naik lagi saat tick berikutnya membawa
  offset QR yang lebih bias — **osilasi berasal dari noise sinyal masukan visual servo, bukan
  dari gain controller PD terlalu agresif** (yang akan terlihat sebagai `diverged=True`/
  saturasi tinggi — TIDAK terjadi, 0/17, `docs/P0-2-4-RESULTS.md` §4).
- `_goto_xy()` (`mission_fsm.py:359-382`) adalah **PD murni, tanpa suku integral**. Taper gaya
  dekat target (`fm = fmax * max(min_fmax_frac=0.05, dist/slow_radius=1.0)`) mengecilkan
  otoritas kontrol jadi hanya **0.8 N** (`5% × approach_fmax=16N`) di dalam radius 1.0 m.
  Kombinasi "target bergerak akibat noise QR" + "otoritas kontrol mengecil justru saat paling
  dibutuhkan untuk presisi" adalah kandidat kuat kenapa `dist` tidak pernah *settle* stabil di
  dalam `approach_tol=0.06m`/`qr_center_tol=0.12` — bukan karena ROV tidak bisa sampai dekat
  (ia bisa, §0), tapi karena titik yang dikejar sendiri tidak diam.

### A.2 — Yang secara eksplisit BUKAN penyebab (dikonfirmasi, bukan diasumsikan)

- **Bukan instabilitas/divergensi controller**: `diverged=0/17`, `overshoot=0` di semua run yang
  sempat masuk tolerance (`docs/P0-2-4-RESULTS.md` §4). Menaikkan `approach_kd` (damping) atau
  menurunkan `approach_kp` — perbaikan tipikal untuk osilasi-akibat-gain-agresif — **tidak
  didukung evidence** sebagai langkah pertama, karena command tidak menunjukkan pola gain
  berlebihan (saturasi rendah, 0.0-0.244 fraksi tick).
- **Bukan soal `t_scan` terlalu pendek**: `window_s` (durasi `APPROACH_QR`) 3.3-7.6 s di semua
  17 run, jauh di bawah `t_scan=45s`. FSM keluar state jauh sebelum timeout — kegagalan
  konvergen bukan soal kehabisan waktu.
- **Bukan (semata) soal "ROV tidak bisa mendekat"**: 16/17 run mencapai `min_dist_target≤0.11m`
  (§0) — sangat dekat dengan `approach_tol=0.06m`. Masalahnya presisi/stabilitas *penutup*
  gap terakhir itu, bukan approach jarak jauh yang gagal.

## B. Candidate Changes (Measurable)

Setiap kandidat **terisolasi** (satu variabel), dengan hipotesis dan indikator sukses eksplisit,
dibandingkan terhadap baseline **5/17 (29%) entered_band_with_dwell, 0/17 diverged**. Semua
kandidat di bawah **BELUM diimplementasikan** — menunggu approval.

| # | Kandidat | Jenis | Hipotesis | Indikator sukses terukur |
|---|---|---|---|---|
| 1 | Perlebar gerbang servoing `dist_raw < 0.3` → `dist_raw < 0.6` (`mission_fsm.py:558`) | Parameter | Mode-2 run dapat lebih banyak tick untuk visual servo beroperasi sebelum FSM sempat exit | `servo_frac` rata-rata naik terukur; `entered_band_with_dwell` tidak *memburuk* (kontrol: tidak boleh menurun akibat servo aktif terlalu dini di jarak jauh yang belum akurat) |
| 2 | EMA/low-pass pada `qr_ex`,`qr_ey` sebelum dipakai hitung offset servo (mis. `α=0.3`, ditentukan lewat sweep terpisah bila kandidat ini lanjut) | Logika (filter, bukan gain) | Meredam noise per-tick dari corner detection (P0-2.3: bias hingga −0.19m pada observasi corner-only) sehingga target `(tx,ty)` tidak "bergerak" tiap tick | Variansi `tx,ty` per tick turun (metrik baru, dihitung dari log); gap `min_dist_target` vs `final_dist` (§0) menyempit; `entered_band_with_dwell` naik |
| 3 | Naikkan `min_fmax_frac` (mis. 0.05 → 0.15) supaya otoritas kontrol tidak mengecil terlalu jauh di dalam radius 1.0m | Parameter | Bias sisa/steady-state (dari noise QR atau gangguan fisik) bisa dikoreksi lebih cepat kalau force tidak diclamp sekecil 0.8N | `final_dist` median turun; **wajib dicek** `diverged`/`saturation_frac` tidak naik signifikan (kandidat ini satu-satunya yang punya risiko nyata memicu osilasi baru — exit criterion di §C) |
| 4 | Tambah syarat dwell N-tick pada kondisi exit `centered`/`dist<approach_tol` di FSM (`mission_fsm.py:596`), bukan cuma di reducer P0-2.4 | Logika (kondisi transisi) | Mencegah GRAB dipicu oleh satu tick noise yang kebetulan lolos tolerance (pola persis yang ditemukan §0: min tercapai tapi tidak bertahan) | Rate `GRAB` yang terjadi tanpa dwell (dibandingkan sebelum/sesudah) turun; **tapi ini kandidat paling invasif** (mengubah kriteria exit FSM, bukan cuma tuning) — diurutkan terakhir di roadmap |

**Tidak termasuk sebagai kandidat controller di sini**: perbaikan `qr_detector.py`/`qr_logic.py`
untuk menaikkan decode rate Mode-1 (6/17 run nol decode). Itu valid dan mungkin berdampak
besar, tapi di luar scope "controller" yang diminta task ini dan sudah diidentifikasi sebagai
pertanyaan terpisah oleh P0-2.3 (root-cause decode quality, `docs/P0-2-3-ACCEPTANCE-REVIEW.md`
§4) — dicatat di §C sebagai jalur eskalasi kalau kandidat 1-4 tidak cukup.

## C. Sequential Experiment Roadmap

Satu variabel per eksperimen, battery ukuran sebanding dengan P0-2.4 (protokol sama,
`run_approach_qr_battery.sh`, stopping rule `n≥18` ATAU `entered≥5` — pakai kembali tooling yang
sudah ada di `reduce_approach_qr.py`, tidak perlu tooling baru). Setiap eksperimen dibandingkan
terhadap **baseline P0-2.4 (5/17, 0 diverged)**, bukan terhadap eksperimen sebelumnya secara
berantai (supaya tiap kandidat tetap bisa diatribusi murni ke variabelnya sendiri).

1. **Eksperimen 1 — Kandidat #1 (lebar gerbang servoing)**. Paling murah/paling tidak invasif
   (satu angka parameter, tidak mengubah logika). Jalankan battery penuh, hitung
   `entered_band_with_dwell`, `servo_frac`, `diverged`.
   - **Exit criteria**: kalau `entered_band_with_dwell` tidak membaik melampaui noise sampel
     (mis. tetap ≤6/18) DAN `servo_frac` juga tidak naik berarti — kandidat ini tidak didukung,
     revert, lanjut ke Eksperimen 2. Kalau `servo_frac` naik tapi `entered_band` tidak — itu
     sendiri temuan (servoing lebih sering aktif tidak cukup, noise-nya yang jadi masalah) —
     memperkuat prioritas ke Eksperimen 2.

2. **Eksperimen 2 — Kandidat #2 (filter EMA pada qr_ex/qr_ey)**. Hanya dijalankan jika
   Eksperimen 1 inconclusive/gagal. Isolasi hipotesis noise/jitter (§A.1) secara langsung.
   - **Exit criteria**: kalau gap `min_dist_target` vs `final_dist` (§0) tidak menyempit
     dibanding baseline, DAN `entered_band_with_dwell` tidak membaik — hipotesis noise-jitter
     sebagai penyebab dominan **ditolak**, jangan lanjut menaikkan/menurunkan `α` secara
     coba-coba (itu sendiri jadi eksperimen baru yang perlu dijustifikasi terpisah) — lanjut ke
     Eksperimen 3.

3. **Eksperimen 3 — Kandidat #3 (`min_fmax_frac` naik)**. Hanya jika Eksperimen 1-2 belum cukup.
   Kandidat berisiko tertinggi dari keempatnya.
   - **Exit criteria (dua arah)**: (a) kalau `final_dist` median tidak turun — tolak, ini bukan
     masalah otoritas kontrol; (b) **lebih penting**, kalau `diverged` atau `saturation_frac`
     naik signifikan dibanding baseline (0/17, rata-rata 0.0-0.244) — **hentikan segera**,
     kandidat ini memperkenalkan risiko instabilitas yang sebelumnya tidak ada (§A.2), jangan
     dilanjutkan ke tuning lebih lanjut tanpa re-evaluasi desain.

4. **Eksperimen 4 — Kandidat #4 (dwell N-tick di FSM)**. Paling invasif (mengubah kondisi
   transisi state, bukan cuma parameter/filter) — dilakukan **terakhir**, dan hanya jika
   kandidat 1-3 menaikkan `min_dist_target` sustain tapi belum cukup untuk melewati threshold
   dwell yang sudah dipakai P0-2.4. Menambah dwell di FSM sendiri mengubah definisi "sukses
   GRAB", jadi harus dibarengi re-analisis Gate 5 (exit path) penuh, bukan cuma Gate 4.
   - **Exit criteria**: kalau menambah dwell membuat lebih banyak run timeout (`t_scan` habis
     tanpa pernah GRAB) dibanding baseline — trade-off ini harus dilaporkan eksplisit, bukan
     diam-diam diterima sebagai "lebih presisi jadi lebih baik".

**Exit criteria untuk keseluruhan roadmap**: kalau Eksperimen 1-3 semua gagal menaikkan
`entered_band_with_dwell` secara berarti di atas baseline 5/17 (mis. tidak ada satupun yang
tembus ~8/18), kesimpulan yang direkomendasikan **bukan** "coba kombinasi 1+2+3 sekaligus"
(itu melanggar prinsip satu-variabel), melainkan: eskalasi ke Mode-1 (§A.1) — perbaikan
`qr_detector.py`/`qr_logic.py` untuk decode rate, yang secara struktural berada di luar apa pun
yang bisa diperbaiki controller-side, karena 6/17 run baseline gagal total tanpa QR terdecode
sama sekali dan tidak ada kandidat 1-4 di atas yang menyentuh persoalan itu.

## Status

```text
P0-2.4                              CLOSED — Gate 4 FAIL (5/17)     (docs/P0-2-4-RESULTS.md)
P0-2.5                               DIAGNOSIS + DESIGN — MENUNGGU APPROVAL
  Root cause trace                    Dua mode: (1) 6/17 run nol-decode — di luar scope
                                       controller; (2) 11/17 run target-jitter dari qr_ex/ey
                                       mentah tanpa filter + PD tanpa integral term (§A)
  Divergensi/gain-agresif             DIKESAMPINGKAN sebagai penyebab (0/17 diverged, §A.2)
  Kandidat perbaikan                  4 kandidat terisolasi, BELUM diimplementasi (§B)
  Roadmap eksperimen                  4 langkah berurutan, satu variabel per langkah,
                                       exit criteria eksplisit per langkah (§C)
  qr_detector.py / qr_logic.py /      TIDAK DIUBAH
    mission_fsm.py / controller
  Implementasi                        BELUM DIMULAI — menunggu human approval sebelum
                                       Eksperimen 1 (langkah 4 protokol) dieksekusi
```
