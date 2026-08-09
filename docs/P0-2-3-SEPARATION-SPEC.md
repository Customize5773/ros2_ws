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

## 3. Metode sekunder — proxy kualitas geometris independen dari `decode_success`

Dua sinyal kontinu, murni geometris, terhitung dari corner mentah terlepas dari sukses-tidaknya
decode — menguji apakah keduanya memprediksi residual lebih baik daripada flag biner
`decode_success` (yaitu, apakah `decode_success` cuma proxy kasar dari variabel kualitas
geometris yang mendasarinya, atau sesuatu yang berbeda):

- **Squareness**: `stdev(edge_lengths) / mean(edge_lengths)` — quad frontal sejati ≈0;
  quad degenerate/ter-localisasi buruk seharusnya punya variansi lebih tinggi.
- **Right-angle deviation**: rata-rata deviasi absolut 4 sudut interior quad dari 90°.

## 4. Variabel yang diukur (per observasi independen)

`decode_success`, `inflation` (`aabb_side/mean_edge`), `squareness` (CV panjang sisi),
`angle_deviation_90` (kualitas sudut interior), `qr_size`/`qr_ex`/`qr_ey`, `distance_est`
mentah dan terkoreksi-inflasi, `dist_diff_raw`/`dist_diff_corrected` (vs `h_cam`), sudut
rotasi, tag run, kondisi spawn, timestamp relatif terhadap entry `APPROACH_QR`.

## 5. Grup kontrol/perbandingan

- Grup A: `decode_success=1`. Grup B: `decode_success=0` (corner-only).
- Kedua grup dipecah lagi oleh 3 bin inflasi tetap di atas (desain 2×3).
- **Tanpa ablation** di mana pun — observasi pasif natural saja, sama seperti setiap pass
  P0-2 sebelumnya.

## 6. Jumlah run yang dibutuhkan (bagian yang butuh data baru — desain saja, BELUM dieksekusi)

Battery `R1`-`R6` yang ada sekarang: 42 observasi, cuma 7 `decode_success=1`, tersebar tidak
rata (0-3 per run). Itu underpowered untuk perbandingan terstratifikasi 2×3 (idealnya ≥3-5
per sel). Pada laju yang teramati (~1.2 observasi `decode_success`/run), mencapai ~15-20
observasi `decode_success` yang layak pakai butuh kira-kira **N=12-15 run**, protokol sama
seperti sebelumnya (`rov_random_spawn:=true` untuk sebagian besar, satu deterministik) — draw
independen lebih banyak, bukan protokol berbeda. Ini **rekomendasi desain untuk battery masa
depan**, secara eksplisit **belum dieksekusi dalam pass ini**.

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

- **AABB/model error menjelaskannya**: di dalam tiap bin inflasi, gap `decode_success`
  menyusut mendekati level noise bin itu sendiri (grup konvergen) dibanding gap tak-
  terstratifikasi.
- **Detection/decode quality adalah kontributor independen**: gap `decode_success` bertahan
  pada magnitudo yang sebanding di dalam tiap bin inflasi.
- **Squareness/angle-deviation vs decode_success**: kalau proxy kualitas geometris kontinu
  berkorelasi dengan magnitudo residual lebih konsisten daripada flag biner `decode_success`,
  itu mengarah ke variabel kualitas-geometris yang mendasari sebagai penjelasan yang lebih
  fundamental (di mana `decode_success` cuma proxy kasarnya).

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

## 9. Non-goals eksplisit untuk dokumen ini

- Tidak ada eksekusi di pass ini — desain saja.
- Tidak ada perubahan `qr_detector.py`, `qr_logic.py`, `mission_fsm.py`, atau
  controller/parameter apa pun.
- Tidak ada battery baru dijalankan.
- Tidak ada perubahan verdict P0-2.3 atau acceptance matrix `docs/P0-2-AUDIT.md` §2.
- Tidak boleh memakai kembali angka `r=+1.000`/`R²=1.000` yang sudah ditarik
  (`docs/P0-2-3-SPEC.md` §20) sebagai evidence di desain atau eksekusi manapun dari ini.

## 10. Status

```text
P0-2.3                              OPEN
  Separation experiment design        DITULIS DI DOKUMEN INI — belum direview/disetujui
                                       untuk eksekusi
  Battery N=12-15 (jika disetujui)    BELUM DIJALANKAN
  qr_detector.py / qr_logic.py        TIDAK DIUBAH
  P0-2.3 verdict                      BELUM DIBERIKAN
```
