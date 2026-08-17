# P0-2.3 SEPARATION EXPERIMENT SPEC — AABB/model error vs detection/decode quality (KKI 2026)

Dokumen ini adalah **desain eksperimen, BUKAN eksekusi**. Tidak ada kode yang diubah, tidak
ada simulator yang dijalankan, tidak ada battery baru dalam pembuatan dokumen ini. Tujuannya:
merancang eksperimen yang benar-benar bisa memisahkan dua hipotesis yang masih tercampur di
`docs/P0-2-3-SPEC.md` §16-§21.

**Prasyarat penting**: baca `docs/P0-2-3-SPEC.md` §20-§21 dulu. Korelasi `qr_size`/`1/qr_size`
terhadap `dist_diff_raw` (`r=+1.000`/`r=-0.883`, regresi `R²=1.000`) **DITARIK** — tautologis
terhadap rumus `distance_est=K/qr_size`. Angka-angka itu **TIDAK BOLEH** dipakai sebagai
evidence di desain ini atau pass manapun setelahnya.

## 1. Pertanyaan yang belum terjawab

`docs/P0-2-3-SPEC.md` §18-19 sudah menetapkan satu gap residual yang **valid, non-tautologis**
(perbandingan grup lewat flag independen, bukan self-korelasi):

- mean `dist_diff_raw` untuk `decode_success=1` ≈ **+0.009 m**
- mean `dist_diff_raw` untuk corner-only (`decode_success=0`) ≈ **−0.176 m**

Yang belum diketahui: **penyebabnya**. Dua hipotesis masih tercampur:

1. **AABB/model error**: observasi corner-only kebetulan punya rotasi/inflasi AABB rata-rata
   lebih buruk, dan seluruh gap `decode_success` bisa dijelaskan semata oleh itu.
2. **Detection/decode quality**: observasi corner-only membawa noise tambahan yang nyata
   (localisasi corner buruk, quad degenerate seperti outlier `R4` yang sudah dikonfirmasi)
   di luar apa pun yang sudah dijelaskan model rotasi/inflasi.

## 2. Metode utama — perbandingan `decode_success` distratifikasi inflasi

Inflasi (`aabb_side/mean_edge`, murni geometris) dan `decode_success` adalah dua pengukuran
**terpisah** dari data corner yang sama. Membandingkan residual antar grup `decode_success`
**di dalam** strata inflasi yang sepadan bukan tautologi seperti mengkorelasikan residual
terhadap `qr_size` (§20 `P0-2-3-SPEC.md`) — ini mengisolasi apakah gap `decode_success`
bertahan setelah model-error akibat rotasi ditahan kira-kira konstan.

- Stratifikasi ke bin inflasi **tetap** (bukan tertile per-run, supaya bisa dipool lintas
  banyak run): `[1.00-1.20)`, `[1.20-1.50)`, `[1.50+)`.
- Di dalam tiap bin, bandingkan mean/median `dist_diff_raw` (dan residual
  terkoreksi-inflasi) antara `decode_success=1` vs `decode_success=0`.

**⚠️ REVISI setelah review internal (§11) — bin coverage sudah dicek terhadap data `R1`-`R6`
yang ada, BUKAN asumsi**: seluruh 7 observasi `decode_success=1` di data yang ada jatuh di
bin `[1.00,1.20)` — nol di dua bin lain (tabel lengkap di §11). Desain 2×3 seperti tertulis
di atas **kemungkinan besar akan tetap degenerate** untuk bin 2 dan 3 berapa pun jumlah run
tambahan, karena ini mungkin bukan gap sampling tapi batasan struktural
`cv2.QRCodeDetector` (decode sukses mungkin memang butuh pandangan dekat-axis-aligned).
**Deliverable utama yang realistis**: perbandingan `decode_success` di dalam **bin
`[1.00,1.20)` saja** (sudah ada n=7 vs n=8 dari data existing — perbandingan yang bisa
dipakai). Bin `[1.20,1.50)` dan `[1.50+)` direpurpose jadi **dose-response inflasi
corner-only-saja** (§7) — bukan perbandingan `decode_success`, karena grup A kemungkinan
tidak akan pernah terisi di sana. Kalau battery masa depan TERNYATA menghasilkan
`decode_success=1` di luar bin 1, itu sendiri temuan penting (mematahkan bacaan
"decode butuh inflasi rendah") — perlu dicatat secara eksplisit, bukan diasumsikan tidak
akan terjadi.

## 3. Metode sekunder — proxy kualitas geometris independen dari `decode_success`

Dua sinyal kontinu, murni geometris, terhitung dari corner mentah terlepas dari sukses-tidaknya
decode — menguji apakah keduanya memprediksi residual lebih baik daripada flag biner
`decode_success` (yaitu, apakah `decode_success` cuma proxy kasar dari variabel kualitas
geometris yang mendasarinya, atau sesuatu yang berbeda):

- **Squareness**: `stdev(edge_lengths) / mean(edge_lengths)` — quad frontal sejati ≈0;
  quad degenerate/ter-localisasi buruk seharusnya punya variansi lebih tinggi.
- **Right-angle deviation**: rata-rata deviasi absolut 4 sudut interior quad dari 90°.

**⚠️ Caveat penting (dari review §11, Finding 3)**: kedua proxy ini **BUKAN murni sinyal
noise**. QR yang dilihat off-nadir (ROV tidak persis di atas payload, atau kamera
pitch/roll) akan terproyeksi sebagai trapesium non-square **meski deteksinya benar
sempurna** — itu perspektif nyata, bukan error. Dokumen ini belum bisa memisahkan
"non-square karena sudut pandang" dari "non-square karena localisasi corner buruk". Baca
squareness/angle-deviation dengan hati-hati sebagai proxy yang confounded dengan geometri
pandang, bukan indikator kualitas murni — ini keterbatasan terbuka, bukan sesuatu yang
diselesaikan desain ini.

## 4. Variabel yang diukur (per observasi independen)

`decode_success`, `inflation` (`aabb_side/mean_edge`), `squareness` (CV panjang sisi),
`angle_deviation_90` (kualitas sudut interior), `qr_size`/`qr_ex`/`qr_ey`, `distance_est`
mentah dan terkoreksi-inflasi, `dist_diff_raw`/`dist_diff_corrected` (vs `h_cam`), sudut
rotasi, tag run, kondisi spawn, timestamp relatif terhadap entry `APPROACH_QR`.

**Keterbatasan yang diwarisi (dari review §11, Finding 5 & 6), berlaku untuk seluruh
variabel di atas**:
- Data corner **tidak difilter per `frame_id`** (kamera bawah vs depan) seperti konsumsi
  `qr_offset` oleh `mission_fsm` sendiri (`mission_fsm.py:408-413`). Sebagian baris corner
  bisa saja berasal dari kamera depan, menambah noise tak terkait ke statistik geometri yang
  justru sedang dikarakterisasi desain ini.
- `h_cam` dihitung dari `sp_depth` (setpoint/perintah), bukan `depth` (kedalaman terukur) —
  lag/error tracking depth oleh controller akan tercampur ke residual seolah-olah itu error
  QR, padahal sumbernya di luar pipeline QR sama sekali.

## 5. Grup kontrol/perbandingan

- Grup A: `decode_success=1`. Grup B: `decode_success=0` (corner-only).
- Kedua grup dipecah lagi oleh 3 bin inflasi tetap di atas (desain 2×3).
- **Tanpa ablation** di mana pun — observasi pasif natural saja, sama seperti setiap pass
  P0-2 sebelumnya.

## 6. Jumlah run yang dibutuhkan (bagian yang butuh data baru — desain saja, BELUM dieksekusi)

Battery `R1`-`R6` yang ada sekarang: 42 observasi, cuma 7 `decode_success=1`, tersebar tidak
rata (0-3 per run). Itu underpowered untuk perbandingan terstratifikasi (idealnya ≥3-5
per sel, per §2's revisi bin-1-saja). Pada laju yang teramati (~1.2 observasi
`decode_success`/run), mencapai ~15-20 observasi `decode_success` yang layak pakai butuh
kira-kira **N=12-15 run** kalau laju itu stabil — tapi laju per-run yang teramati
(`0,0,0,3,1,3`) punya varians tinggi/menggerombol, bukan stabil. **REVISI (review §11,
Finding 2)**: pakai **stopping rule**, bukan jumlah run tetap — jalankan dalam batch 6 run,
berhenti begitu total observasi `decode_success` kumulatif ≥15, dibatasi maksimum ~20 run.
Protokol run tetap sama seperti sebelumnya (`rov_random_spawn:=true` untuk sebagian besar,
satu deterministik per batch) — draw independen lebih banyak, bukan protokol berbeda. Ini
**rekomendasi desain untuk battery masa depan**, secara eksplisit **belum dieksekusi dalam
pass ini**.

**Dicatat tapi ditandai sebagai pertanyaan desain terpisah/opsional, bukan diasumsikan**:
karena tiap run tanpa trigger cuma menghasilkan satu siklus `APPROACH_QR`/`GRAB` sebelum
parkir di `WAIT_TRIGGER`, memperpanjang *durasi* run saja tidak akan menambah observasi
`decode_success` — hanya lebih banyak *run* independen yang akan. Misi sebenarnya punya
trigger loop otonom yang sudah ada (`/hydroships/mission/start_autonomous`) yang secara
prinsip bisa membuat satu run mencakup beberapa siklus payload, tapi memakainya butuh tool
orkestrasi kecil baru (bukan perubahan `qr_detector.py`/`qr_logic.py`/FSM) — ditandai di sini
sebagai kemungkinan peningkatan efisiensi di masa depan, **bukan bagian desain ini**, supaya
tidak scope-creep dari yang diminta.

## 7. Kriteria interpretasi (didefinisikan di muka, supaya tidak rasionalisasi post-hoc)

**REVISI (review §11, Finding 1)** — dipecah sesuai realita coverage bin, bukan 2×3 seragam:

- **Perbandingan utama, bin `[1.00,1.20)` saja** (satu-satunya bin dengan kedua grup terisi):
  - **AABB/model error menjelaskannya**: gap `decode_success` di dalam bin ini kecil/tak
    signifikan relatif noise bin — sisa gap tak-terstratifikasi ternyata datang dari
    perbedaan sebaran inflasi antar grup, bukan dari `decode_success` itu sendiri.
  - **Detection/decode quality kontributor independen**: gap `decode_success` tetap ada dan
    sebanding magnitudonya dengan gap tak-terstratifikasi bahkan di dalam bin sesempit ini —
    berarti bukan semata soal inflasi.
- **Dose-response corner-only-saja, bin `[1.20,1.50)` dan `[1.50+)`** (hanya
  `decode_success=0` yang mengisi bin-bin ini pada data yang ada): apakah residual corner-only
  memburuk monoton seiring inflasi naik lintas ketiga bin — ini menguji hipotesis 1 secara
  independen dari `decode_success`, bukan menggantikan perbandingan `decode_success`.
- **Kalau battery masa depan ternyata menghasilkan `decode_success=1` di bin 2/3**: catat
  eksplisit sebagai temuan tersendiri (mematahkan bacaan "decode butuh inflasi rendah") dan,
  kalau n cukup (§8), jalankan juga perbandingan `decode_success` di bin itu.
- **Squareness/angle-deviation vs decode_success**: kalau proxy kualitas geometris kontinu
  berkorelasi dengan magnitudo residual lebih konsisten daripada flag biner `decode_success`,
  itu mengarah ke variabel kualitas-geometris yang mendasari sebagai penjelasan yang lebih
  fundamental (di mana `decode_success` cuma proxy kasarnya) — **dibaca dengan caveat
  perspektif-vs-noise di §3**, bukan sebagai sinyal noise murni.

## 8. Kriteria inconclusive (didefinisikan di muka)

- Bin inflasi mana pun dengan `<3` observasi `decode_success=1` ATAU `<3` observasi
  `decode_success=0`: perbandingan bin itu tidak diinterpretasikan, dilaporkan sebagai n
  tidak cukup.
- Total observasi `decode_success` `<15` lintas seluruh battery: seluruh perbandingan
  terstratifikasi dianggap inconclusive; rekomendasi mempertimbangkan ulang pendekatan
  pengumpulan data (bukan sekadar "lebih banyak dari yang sama") alih-alih memaksakan
  kesimpulan.
- Kalau dua metode (residual-terstratifikasi vs korelasi proxy-kualitas-geometris) tidak
  sepakat arah: dilaporkan sebagai inconclusive/campuran, tidak diselesaikan dengan memilih
  salah satu.
- Kalau satu run menyumbang porsi tidak proporsional ke salah satu grup (seperti `R4`/`R6`
  di data yang ada) — pelajaran dibawa dari kejadian outlier `R4` — hasil harus dilaporkan
  dengan DAN tanpa kontribusi run itu.
- **REVISI (review §11, Finding 4)**: aturan yang sama berlaku di level OBSERVASI, bukan
  cuma level run — kalau satu observasi individual jadi outlier ekstrem pada `squareness`
  atau `angle_deviation_90` (mis. quad degenerate serupa outlier `R4` yang sudah dikonfirmasi
  di `docs/P0-2-3-SPEC.md` §20.1), agregat kedua proxy itu juga harus dilaporkan dengan DAN
  tanpa observasi tersebut — jangan biarkan satu baris degenerate mendominasi mean/median
  seperti yang sempat terjadi pada `qr_size`/`qr_ey` sebelum dikoreksi.

## 9. Non-goals eksplisit untuk dokumen ini

- Tidak ada eksekusi di pass ini — desain saja.
- Tidak ada perubahan `qr_detector.py`, `qr_logic.py`, `mission_fsm.py`, atau
  controller/parameter apa pun.
- Tidak ada battery baru dijalankan.
- Tidak ada perubahan verdict P0-2.3 atau acceptance matrix `docs/P0-2-AUDIT.md` §2.
- Tidak boleh memakai kembali angka `r=+1.000`/`R²=1.000` yang sudah ditarik
  (`docs/P0-2-3-SPEC.md` §20) sebagai evidence di desain atau eksekusi manapun dari ini.

## 11. Review internal — hasil dan revisi (pass terpisah dari penulisan awal §1-10)

Dokumen ini direview secara internal, read-only, terhadap data `R1`-`R6` yang sudah ada
(bukan asumsi) sebelum diputuskan layak eksekusi atau tidak. Hasil: **BUKAN "siap eksekusi
tanpa revisi"** — satu kelemahan signifikan ditemukan dan dibuktikan dengan data, lima
lainnya kecil/menengah. Semua sudah dilipat ke §2-§9 di atas; ringkasannya:

| # | Temuan | Tingkat | Revisi |
|---|---|---|---|
| 1 | Ke-7 observasi `decode_success=1` yang ada SEMUA jatuh di bin inflasi `[1.00,1.20)`; nol di dua bin lain — kemungkinan struktural (batasan decoder), bukan cuma gap sampling | **Signifikan** | §2, §7: reframe jadi perbandingan bin-1-saja + dose-response corner-only untuk bin 2-3 |
| 2 | "N=12-15 run" dari laju rata-rata, padahal laju per-run sangat bervariasi (`0,0,0,3,1,3`) | Menengah | §6: stopping rule (≥15 `decode_success` kumulatif, batch 6, cap ~20 run) bukan N tetap |
| 3 | Squareness/angle-deviation bisa mencerminkan perspektif off-nadir asli, bukan cuma noise deteksi | Menengah | §3: caveat eksplisit ditambahkan |
| 4 | Aturan anti-dominasi cuma di level run (§8 lama), belum di level observasi individual | Kecil | §8: aturan with/without diperluas ke level observasi utk squareness/angle |
| 5 | Data corner tidak difilter `frame_id` (bottom vs front camera) | Kecil, diwarisi | §4: caveat ditambahkan |
| 6 | `h_cam` dari `sp_depth` (setpoint) bukan `depth` (terukur) | Kecil, diwarisi | §4: caveat ditambahkan |

Tabel bukti Finding 1 (41 observasi valid, degenerate `R4` dikeluarkan):

| Bin inflasi | n | `decode_success=1` | `decode_success=0` |
|---|---|---|---|
| `[1.00, 1.20)` | 15 | **7** | 8 |
| `[1.20, 1.50)` | 16 | **0** | 16 |
| `[1.50, +inf)` | 10 | **0** | 10 |

Tidak ada kode/detector/FSM/controller yang diubah untuk menghasilkan review ini — murni
query read-only terhadap CSV `R1`-`R6` yang sudah ada di `/tmp/p0-2-3-battery/`. Tidak ada
battery baru dijalankan. Tidak ada verdict P0-2.3 yang berubah.

## 12. Status (superseded oleh §14 — battery separation sudah dieksekusi)

```text
P0-2.3                              OPEN
  Separation experiment design        REVISED setelah review internal (§11) —
                                       bin-1-only reframing, stopping rule, 4 caveat baru
  Review verdict                      NOT READY AS-ORIGINALLY-WRITTEN — revisi sudah
                                       dilipat ke §2-§9, MENUNGGU review pengguna atas
                                       revisi sebelum dianggap READY FOR EXECUTION
  Battery N (stopping-rule, jika disetujui)   BELUM DIJALANKAN
  qr_detector.py / qr_logic.py / mission_fsm.py   TIDAK DIUBAH
  P0-2.3 verdict                      BELUM DIBERIKAN
```

## 13. Eksekusi battery separation (stopping-rule, disetujui pengguna)

Dieksekusi persis sesuai desain §2-§8: `tools/p0-experiments/run_approach_qr_battery.sh`
dijalankan per batch 6 run (`TAG_PREFIX=S` lalu `TAG_PREFIX=T`), protokol sama seperti
`Q1`-`Q6`/`R1`-`R6` (5 `rov_random_spawn:=true` + 1 deterministik per batch), ke
`/tmp/p0-2-3-separation-battery`. Stopping rule: berhenti begitu total observasi
`decode_success` kumulatif ≥15.

**Batch 1 (`S1`-`S6`)**: 6/6 gate PASS. Cumulative `decode_success` = **10** (< 15) → lanjut
batch 2.
**Batch 2 (`T1`-`T6`)**: 6/6 gate PASS. Cumulative `decode_success` = **17** (≥ 15) →
**stopping rule terpenuhi setelah 12 run, batch 3 tidak diperlukan.**

12/12 gate PASS, 0 `INCONCLUSIVE` karena gate gagal. Tidak ada perubahan
`qr_detector.py`/`qr_logic.py`/`mission_fsm.py`/parameter/controller selama eksekusi. Tidak
ada ablation.

`tools/p0-experiments/analyze_qr_separation.py` (baru, mengimplementasikan §2-§8 persis)
dijalankan terhadap ke-12 CSV.

## 14. Hasil analisis separation — EVIDENCE, BUKAN VERDICT

**Total 58 observasi independen** (naik dari 42 di `R1`-`R6`): **17 `decode_success=1`**,
**41 corner-only**.

### Bin coverage (dibandingkan dengan §11 — temuan baru penting)

| Bin inflasi | n | `decode_success=1` | `decode_success=0` |
|---|---|---|---|
| `[1.00,1.20)` | 17 | 12 | 5 |
| `[1.20,1.50)` | 29 | **5** | 24 |
| `[1.50,+inf)` | 12 | 0 | 12 |

Di `R1`-`R6`, SEMUA `decode_success=1` ada di bin 1 saja (§11). Di battery ini,
**`decode_success=1` muncul juga di bin `[1.20,1.50)` (n=5)** — persis skenario yang
diperingatkan §7 sebagai "kalau terjadi, itu temuan tersendiri (mematahkan bacaan 'decode
butuh inflasi rendah')". **Dicatat secara eksplisit**: bacaan struktural "decode hanya
sukses pada inflasi rendah" dari §11 **tidak sepenuhnya benar** — decode bisa sukses pada
inflasi sedang juga, hanya lebih jarang. Bin `[1.50,+inf)` tetap nol `decode_success=1`.

### PRIMER — perbandingan `decode_success` di bin `[1.00,1.20)`

| | `decode_success=1` | corner-only | Gap |
|---|---|---|---|
| Residual RAW | n=12, mean=+0.012 | n=5, mean=−0.105 | **+0.117** |
| Residual TERKOREKSI-inflasi | n=12, mean=+0.041 | n=5, mean=−0.053 | **+0.094** |

Gap bertahan setelah koreksi inflasi — **tidak menyusut ke nol**. Per kriteria interpretasi
§7: ini mengarah ke **"detection/decode quality adalah kontributor independen"**, bukan
semata dijelaskan oleh inflasi.

**Pengecekan dominasi single-run (§8 revisi)**: grup `decode_success=1` di bin ini didominasi
`S4` (7/12, 58%) — di atas ambang flag. Mean TANPA `S4`: **+0.036** (n=5) — gap terhadap
corner-only (−0.105) jadi **+0.141**, justru SEDIKIT LEBIH BESAR, bukan lebih kecil. Temuan
tidak runtuh saat `S4` dikeluarkan.

### TAMBAHAN — perbandingan `decode_success` di bin `[1.20,1.50)` (baru muncul, n cukup)

| | `decode_success=1` | corner-only | Gap |
|---|---|---|---|
| Residual RAW | n=5, mean=−0.047 | n=24, mean=−0.148 | **+0.101** |
| Residual TERKOREKSI-inflasi | n=5, mean=+0.039 | n=24, mean=−0.055 | **+0.093** |

Arah dan magnitudo gap **konsisten dengan bin `[1.00,1.20)`** (+0.101/+0.093 vs
+0.117/+0.094) — bukti independen kedua yang mengarah ke kesimpulan sama. **Tapi**: seluruh
5 observasi `decode_success=1` di bin ini berasal dari **satu run saja (`T2`, 5/5=100%)** —
tidak bisa dilaporkan "dengan vs tanpa" karena "tanpa" akan menyisakan n=0. Diberi bobot
lebih rendah daripada bin `[1.00,1.20)`, tapi arahnya tetap dicatat sebagai bukti pendukung,
bukan bukti berdiri sendiri.

### SEKUNDER — dose-response corner-only lintas bin (uji Hipotesis 1 secara independen)

| Bin | n | mean residual | median |
|---|---|---|---|
| `[1.00,1.20)` | 5 | −0.105 | −0.014 |
| `[1.20,1.50)` | 24 | −0.148 | −0.179 |
| `[1.50,+inf)` | 12 | −0.188 | −0.207 |

**Tren monoton memburuk** seiring inflasi naik, murni di dalam grup corner-only (tidak
melibatkan `decode_success` sama sekali) — evidence bahwa **inflasi/AABB memang berkontribusi
nyata**, terlepas dari isu kualitas deteksi.

### Squareness / angle_deviation_90

| | `decode_success=1` | corner-only |
|---|---|---|
| squareness (mean) | 0.0058 | 0.3965 |
| angle_dev (mean, °) | 0.520 | 30.563 |

Perbedaan sangat besar — tapi diingat caveat §3: sebagian bisa jadi perspektif nyata (QR
dilihat off-nadir), bukan cuma noise. Korelasi terhadap residual (`dist_diff_raw`, seluruh
data, bukan tautologis — bukan `qr_size`): `r(squareness)=−0.481`, `r(angle_dev)=−0.554`,
`r(decode_success)=+0.463` (n=56-58). **Ketiganya searah dan sebanding kekuatannya** — tidak
ada satu yang jelas mendominasi yang lain sebagai "penjelasan utama".

### Pengecekan outlier level-observasi (§8 revisi)

Observasi squareness paling ekstrem (`T6 t=5.376`, squareness=0.826, corner-only):
mean keseluruhan `dist_diff_raw` DENGAN observasi ini = −0.111, TANPA = −0.107 — **dampak
kecil**, tidak ada satu observasi yang mendominasi agregat kali ini (berbeda dari kejadian
`R4` di `R1`-`R6`).

### Cek kriteria inconclusive (§8)

- Total `decode_success`=17 ≥15 ✓ (bukan inconclusive dari kriteria ini).
- Sel bin `[1.00,1.20)` kedua grup ≥3 ✓. Sel bin `[1.20,1.50)` kedua grup ≥3 ✓ (tapi
  `decode_success=1` 100% dari satu run — dicatat sebagai keterbatasan, bukan pelanggaran
  ambang n).
- Dua metode (perbandingan residual-terstratifikasi vs korelasi proxy-geometris) **sepakat
  arah** — bukan inconclusive dari kriteria ini.
- Dominasi single-run: terdeteksi di kedua bin, ditangani sesuai §8 (bin 0: temuan bertahan
  tanpa run dominan; bin 1: tidak bisa diuji tanpa run dominan, diberi bobot lebih rendah).

### Ringkasan evidence (BUKAN kesimpulan/verdict)

- **Hipotesis 1 (AABB/model error)**: didukung oleh dose-response corner-only yang monoton
  memburuk lintas bin — inflasi memang berkontribusi nyata pada residual.
- **Hipotesis 2 (detection/decode quality)**: didukung oleh gap `decode_success` yang
  bertahan (tidak menyusut ke nol) di DUA bin inflasi independen, dengan arah dan magnitudo
  yang konsisten (+0.09 sampai +0.14 m), dan tidak runtuh saat dominasi single-run
  dikeluarkan (bin 0).
- **Kedua hipotesis sama-sama didukung evidence** — bukan salah satu vs yang lain. Data
  battery ini condong ke arah: AABB/inflasi DAN kualitas deteksi/decode **berkontribusi
  independen**, bukan salah satu menjelaskan seluruhnya.
- Keterbatasan yang masih berlaku: caveat frame-mixing dan `h_cam`-dari-setpoint (§4) belum
  diatasi; squareness/angle_dev masih confounded dengan perspektif nyata (§3); bin
  `[1.20,1.50)` untuk `decode_success` masih bergantung pada satu run.

**Tidak ada verdict P0-2.3 di sini.** Evidence ini disiapkan untuk direview sebelum keputusan
acceptance diambil — bukan keputusan itu sendiri.

Data lengkap: `/tmp/p0-2-3-separation-battery/*.csv`, `*.log`,
`P0-2-3-separation-results.json`.

## 15. Status (menggantikan §12; verdict akhir di §16)

```text
P0-2.3                              OPEN
  Separation battery                  CLOSED — 12/12 gate PASS, stopping rule terpenuhi
                                       (17 decode_success ≥15), 0 INCONCLUSIVE
  Hipotesis 1 (AABB/model error)      DIDUKUNG — dose-response corner-only monoton
  Hipotesis 2 (detection/decode)      DIDUKUNG — gap decode_success bertahan di 2 bin
                                       independen, tidak runtuh saat dominasi run dikeluarkan
  Kesimpulan sementara                KEDUANYA berkontribusi independen, bukan salah satu
  qr_detector.py / qr_logic.py / mission_fsm.py / controller   TIDAK DIUBAH
  P0-2.3 verdict                      BELUM DIBERIKAN — evidence siap direview
```

## 16. Final acceptance review — lihat `docs/P0-2-3-ACCEPTANCE-REVIEW.md`

Dukungan terhadap kedua hipotesis di §14-§15 di atas **bukan verdict acceptance** — itu
mekanisme, bukan konfirmasi bahwa `APPROACH_QR` memenuhi tolerance P0-2 (`approach_tol`,
`qr_center_tol`, `docs/P0-2-AUDIT.md` §1.5). Evidence di dokumen ini sudah dicocokkan terhadap
matrix acceptance P0-2 secara terpisah di `docs/P0-2-3-ACCEPTANCE-REVIEW.md`.

```text
P0-2.3 verdict (final)     CLOSE-PARTIAL — root-cause residual bias CLOSED (§2.2-2.3 di
                           ACCEPTANCE-REVIEW.md), precision convergence (Gate 4) tetap OPEN/FAIL
```
