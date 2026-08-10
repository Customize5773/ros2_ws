# P0-2.3 FINAL ACCEPTANCE REVIEW (KKI 2026)

Dokumen ini adalah **review acceptance, read-only**. Tidak ada battery baru dijalankan, tidak
ada kode/parameter yang diubah. Seluruh angka di bawah dikutip dari evidence yang sudah ada di
`docs/P0-2-3-SPEC.md` §16-21 dan `docs/P0-2-3-SEPARATION-SPEC.md` §14-15 — tidak ada
pengukuran baru dalam pass ini.

**Titik penting yang mendasari review ini**: hasil bahwa kedua hipotesis (AABB/model error dan
detection/decode quality) sama-sama didukung evidence **bukan otomatis berarti P0-2.3
diterima**. Dukungan terhadap kedua hipotesis menjelaskan *mekanisme* residual bias — itu tidak
sama dengan menunjukkan bahwa `APPROACH_QR` memenuhi tolerance/acceptance matrix P0-2
(`docs/P0-2-AUDIT.md` §1.5, §2). Kedua hal itu dicocokkan secara eksplisit di bawah.

## 1. Tolerance/matrix acuan (dari `docs/P0-2-AUDIT.md`)

| Parameter | Nilai | Sumber |
|---|---|---|
| `approach_tol` (radius konvergensi XY fallback) | **0.06 m** | §1.5, `mission_fsm.py:125/187` |
| `qr_center_tol` (band visual-servo, unit ternormalisasi) | **0.12** | §1.5, `mission_fsm.py:153` |
| `t_scan` (timeout abort) | 45.0 s | §1.5 |
| Gate 4 P0-2.2b (precision convergence) | **0/6 run** masuk band `qr_center_tol` | §7, `P0-2-AUDIT.md:335` |

Baris acceptance matrix (`docs/P0-2-AUDIT.md` §2) yang relevan dengan scope P0-2.3
("positioning/centering di titik `GRAB`", `P0-2-AUDIT.md:336-337`):

- "ROV konvergen ke acceptance region?" — **OPEN** (belum ditutup manapun sebelum review ini).
- "Error relatif berkurang?" — **OPEN**.
- "Tidak oscillatory/divergent?" — **OPEN**.

## 2. Kriteria per pertanyaan — PASS / FAIL / INCONCLUSIVE

### 2.1 "ROV konvergen ke acceptance region?" (`dist < approach_tol` atau band `qr_center_tol`)

**Verdict: FAIL** (belum berubah dari Gate 4).

- Evidence: P0-2.2b Gate 4 — 0/6 run masuk band `qr_center_tol` (`P0-2-AUDIT.md:335`).
- Battery separation P0-2.3 (`SEPARATION-SPEC.md` §13-14) **tidak mengukur ulang** convergence
  ke band ini — desainnya (§2, §7) secara eksplisit mengukur residual `dist_diff_raw` lintas
  strata `decode_success`/inflasi, bukan apakah observasi individual jatuh di dalam
  `qr_center_tol`. Tidak ada evidence baru yang menggantikan Gate 4.
- Namun ada evidence baru yang **relevan terhadap besar-kecilnya gap**: residual mean untuk
  `decode_success=1` (+0.012 raw / +0.041 terkoreksi-inflasi, bin `[1.00,1.20)`,
  `SEPARATION-SPEC.md:268-269`) berada **di dalam** `approach_tol=0.06m`. Residual mean untuk
  corner-only pada bin yang sama (−0.105 raw / −0.053 terkoreksi) juga masih di sekitar
  `approach_tol`, tapi pada bin inflasi lebih tinggi memburuk ke −0.148 s.d. −0.188
  (`SEPARATION-SPEC.md:296-300`) — **melebihi** `approach_tol` 2-3×.
- Kesimpulan: convergence tetap **FAIL** secara langsung (Gate 4 tidak dijalankan ulang), tapi
  evidence baru menunjukkan decode-success sudah mendekati tolerance sementara corner-only pada
  inflasi tinggi jauh di luar tolerance — ini **menjelaskan sebagian mengapa** Gate 4 gagal,
  bukan membalik hasilnya.

### 2.2 Penyebab residual bias (AABB/model error vs detection/decode quality)

**Verdict: PASS** — kedua hipotesis didukung evidence non-tautologis, memenuhi kriteria
interpretasi §7 dan lolos seluruh kriteria inconclusive §8 dari `SEPARATION-SPEC.md`:

- Hipotesis 1 (AABB/inflasi): dose-response monoton corner-only lintas 3 bin inflasi
  (−0.105 → −0.148 → −0.188, `SEPARATION-SPEC.md:296-304`) — murni di dalam grup corner-only,
  tidak bergantung pada `decode_success`.
- Hipotesis 2 (decode quality): gap `decode_success` bertahan (tidak menyusut ke nol) di **dua**
  bin inflasi independen (+0.117/+0.094 di bin 1, +0.101/+0.093 di bin 2,
  `SEPARATION-SPEC.md:264-292`), arah dan magnitudo konsisten.
- Cek §8 inconclusive: total `decode_success`=17 ≥15 ✓; kedua sel bin utama ≥3 observasi ✓;
  kedua metode (residual terstratifikasi vs proxy geometris) sepakat arah ✓; dominasi single-run
  diuji — bin 1 bertahan tanpa run dominan (`S4` dikeluarkan → gap membesar, bukan runtuh,
  `SEPARATION-SPEC.md:275-278`).
- **Caveat yang tetap berlaku dan menurunkan bobot, bukan membatalkan PASS**: seluruh 5
  observasi `decode_success=1` di bin `[1.20,1.50)` berasal dari satu run (`T2`, 100%,
  `SEPARATION-SPEC.md:288-292`) — bukti pendukung, bukan berdiri sendiri; frame-mixing kamera
  bottom/front tidak difilter (`SEPARATION-SPEC.md` §4); `h_cam` dari `sp_depth` bukan `depth`
  terukur; squareness/angle_dev confounded dengan perspektif off-nadir asli (§3).

### 2.3 `decode_success` sebagai correlate independen

**Verdict: PASS** — `r=+0.442` (`P0-2-3-SPEC.md:532`), satu-satunya correlate dengan sinyal
jelas, konsisten muncul dua kali (pemisahan mean §18-19 dan korelasi §20 di `P0-2-3-SPEC.md`).
Tidak dibangun dari korelasi tautologis (`r=+1.000` `qr_size`/`1/qr_size` sudah ditarik secara
eksplisit dan tidak dipakai di sini, `P0-2-3-SPEC.md` §20).

### 2.4 "Error relatif berkurang?" (tren dalam satu run)

**Verdict: INCONCLUSIVE** — tidak ada evidence yang mengukur tren `dist`/`qr_off` sepanjang satu
episode `APPROACH_QR`. Baik `P0-2-3-SPEC.md` §16-21 maupun `SEPARATION-SPEC.md` §14 hanya
melaporkan statistik cross-sectional (mean/median per grup/bin), bukan analisis time-series
per-run. Di luar scope kedua battery yang sudah dijalankan.

### 2.5 "Tidak oscillatory/divergent?"

**Verdict: INCONCLUSIVE** — tidak ada variansi `surge`/`sway` atau overshoot yang dilaporkan di
evidence P0-2.3 manapun. Tidak diukur oleh desain separation battery (§2-§4 `SEPARATION-SPEC.md`
mendefinisikan variabel geometris/residual saja, tidak ada variabel gaya/kontrol).

## 3. Rekonsiliasi: dukungan hipotesis vs acceptance

Ini bagian yang secara eksplisit **tidak** disamakan begitu saja:

- Bahwa kedua hipotesis (§2.2) didukung evidence menjawab pertanyaan **mekanisme** — kenapa ada
  residual bias, dan bahwa bias itu berasal dari dua sumber independen (inflasi geometris +
  kualitas decode), bukan salah satu semata.
- Itu **tidak** menjawab pertanyaan acceptance §2.1 (convergence ke `approach_tol`/
  `qr_center_tol`), yang tetap FAIL karena Gate 4 P0-2.2b tidak diukur ulang, dan gap residual
  pada inflasi tinggi (−0.148 s.d. −0.188 m) tetap melebihi `approach_tol=0.06m` 2-3×.
- Dengan kata lain: P0-2.3 sekarang **mengerti kenapa** precision convergence gagal (kombinasi
  inflasi AABB pada sudut pandang tertentu + noise decode pada corner-only), tapi **belum
  menunjukkan** bahwa masalah itu sudah teratasi atau berada dalam tolerance.

## 4. Disposisi keseluruhan: **CLOSE-PARTIAL**

Mengikuti pola keputusan P0-2.2 (`docs/P0-2-2-VERDICT-OPTIONS.md` §4, Opsi B) — pisahkan klaim
yang benar-benar didukung dari klaim yang belum, bukan satu verdict biner untuk seluruh P0-2.3.

**Klaim yang ditutup (CLOSED)**:
- Root-cause mekanisme residual bias `dist_diff_raw`: **AABB/model error dan detection/decode
  quality dua-duanya berkontribusi independen** (§2.2, §2.3) — evidence non-tautologis, lolos
  seluruh kriteria inconclusive pre-registered, direplikasi di dua bin inflasi independen dan
  bertahan terhadap uji dominasi single-run (bin 1).

**Klaim yang tetap OPEN, dibawa maju**:
- "QR visual servo memberikan precision convergence yang repeatable" (pertanyaan asli yang
  diwariskan dari Gate 4 P0-2.2b) — **FAIL/OPEN**, tidak ada battery convergence baru yang
  dijalankan sebagai bagian dari P0-2.3.
- "Error relatif berkurang sepanjang run?" dan "Tidak oscillatory/divergent?" — **INCONCLUSIVE**,
  di luar scope kedua battery yang sudah dijalankan; butuh instrumentasi/analisis time-series
  baru untuk dijawab, tidak diselesaikan oleh evidence yang ada.
- Keterbatasan yang belum diatasi dan relevan untuk battery masa depan mana pun: frame-mixing
  kamera, `h_cam` dari setpoint bukan depth terukur, confound squareness/angle dengan perspektif
  asli, dan ketergantungan bin `[1.20,1.50)` pada satu run (`T2`).

**Tidak direkomendasikan**: mengubah `qr_detector.py`/`qr_logic.py`/`mission_fsm.py`/parameter
apa pun berdasarkan review ini — akar masalah sudah dipahami secara mekanisme, tapi belum ada
evidence bahwa perbaikan spesifik apa pun akan membawa hasil ke dalam `approach_tol`/
`qr_center_tol`. Keputusan desain perbaikan (mis. koreksi inflasi di `qr_detector.py`, atau
menaikkan `qr_center_tol`) di luar scope review ini — butuh diskusi terpisah, bukan konsekuensi
otomatis dari review acceptance ini.

## 5. Status akhir

```text
P0-2.3                              CLOSE-PARTIAL (lihat docs/P0-2-3-ACCEPTANCE-REVIEW.md)
  Root-cause residual bias            CLOSED — AABB/inflasi DAN decode quality, kontribusi
                                       independen (§2.2-2.3 review ini)
  Precision convergence (Gate 4)      OPEN — FAIL, tidak diuji ulang, gap masih melebihi
                                       approach_tol pada inflasi tinggi
  Error-berkurang-dalam-run           INCONCLUSIVE — tidak diukur oleh battery manapun
  Oscillatory/divergent check         INCONCLUSIVE — tidak diukur oleh battery manapun
  qr_detector.py / qr_logic.py / mission_fsm.py / controller   TIDAK DIUBAH
  Rekomendasi kode/parameter          TIDAK ADA — di luar scope review acceptance ini
```
