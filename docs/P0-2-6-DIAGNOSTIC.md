# P0-2.6 PERCEPTION PIPELINE AUDIT & DIAGNOSTIC ANALYSIS (KKI 2026)

**STATUS: DIAGNOSTIC ONLY. TIDAK ADA KODE YANG DIUBAH.** `qr_detector.py`, `qr_logic.py`, dan
`mission_fsm.py` dibaca, tidak disentuh. Dokumen ini tidak menyimpulkan bahwa `qr_detector.py`
harus ditulis ulang — tujuannya murni mengumpulkan dan mengkuantifikasi evidence, plus
memformulasikan kandidat perbaikan untuk direview terpisah sebelum implementasi apa pun.

## 0. Kenapa pass ini ada

P0-2.5 menguji empat kandidat controller-side terisolasi (gerbang servo lebih lebar, filter EMA,
lantai gaya lebih tinggi, dwell-gating) — tidak satu pun membalik Gate 4 dari FAIL
(`docs/P0-2-5-CANDIDATE1/2/3/4-RESULTS.md`). Exit condition roadmap-nya sendiri
(`docs/P0-2-5-ENGINEERING-ANALYSIS.md` §C) mengarahkan eskalasi ke pipeline persepsi: 6+/17 run
baseline punya `qr_decode_rate=0.000` sepanjang episode (Mode 1,
`docs/P0-2-5-ENGINEERING-ANALYSIS.md` §A.1) — kelas kegagalan yang tidak bisa disentuh
perubahan controller apa pun. P0-2.6 adalah diagnostic pass untuk eskalasi itu.

**Keterbatasan data**: CSV mentah battery P0-2.3 (`/tmp/p0-2-3-separation-battery`) sudah tidak
ada (dikonfirmasi lewat `find`, data `/tmp` memang ephemeral sesuai pola proyek ini) — re-audit
di §1 mengutip angka yang sudah dipublikasikan di `docs/P0-2-3-SPEC.md`/
`docs/P0-2-3-SEPARATION-SPEC.md`, bukan hitung ulang dari data mentah. CSV mentah P0-2.4
(`/tmp/p0-2-4-battery`, 18 file) masih ada — §3 memakai perhitungan segar dari 17 run valid.

## 1. Re-audit evidence log P0-2.3 (dari angka terpublikasi, data mentah tak tersedia)

| Battery | n observasi | `decode_success=1` | Corner-only (`decode_success=0`) | DSR |
|---|---|---|---|---|
| R1-R6 (`docs/P0-2-3-SPEC.md` §18) | 42 | 7 | 35 | 16.7% |
| Separation battery (`docs/P0-2-3-SEPARATION-SPEC.md` §14) | 58 | 17 | 41 | 29.3% |

Keduanya sampel **level-observasi** (satu baris per observasi-episode `APPROACH_QR`, bukan
setiap tick recorder) — unit berbeda dari CSV level-tick P0-2.4 di §3. Kedua DSR ini **tidak
dibandingkan langsung** dengan angka tick-level 10.7% di §3 karena alasan itu; dilaporkan apa
adanya, tidak direkonsiliasi jadi satu angka.

Drop-out (`corner-only`) sudah terdokumentasi sebagai kategori dominan di kedua battery P0-2.3
(83-88% dari observasi non-decode-success adalah corner-only, bukan non-deteksi total) —
konsisten dengan temuan segar §3 di bawah.

## 2. Trace coupling pipeline persepsi (`qr_detector.py` → `qr_logic.py` → `mission_fsm.py`)

Dibaca langsung (tidak ada kode diubah): `src/hydroships_control/hydroships_control/qr_logic.py`,
`qr_detector.py`.

**`robust_decode()` (`qr_logic.py:93-117`)**: mencoba 7 kandidat preprocessing berurutan tetap
(mentah, CLAHE, adaptive-threshold ×2, Otsu, upscaled-threshold ×2). `best_pts` dikunci dari
**kandidat PERTAMA yang menghasilkan corner points apa pun** (baris 112,
`if has_pts and best_pts is None`), terlepas dari apakah kandidat itu berhasil decode. Kalau
tidak ada kandidat yang decode, fungsi mengembalikan `('', best_pts)` — **titik sudut dari
kandidat paling awal/paling minim diproses, meski decode tidak pernah berhasil di kandidat itu
maupun kandidat manapun.** Tidak ada validasi `len(pts)==4`, tidak ada cek batas frame, tidak
ada cek bentuk/konveksitas.

**`qr_detector.py:_on_image()` (L113-152)**: memanggil `robust_decode()`, lalu **menerbitkan
`/hydroships/qr_offset` kapan pun `pts` tidak kosong (L136), terlepas dari sukses-tidaknya
decode** — ini disengaja sesuai komentar kode sendiri ("Offset diterbitkan begitu QR
TERDETEKSI, walau decode gagal, agar servo tetap bisa memusatkan"). `qr_result` (huruf wall,
dipakai men-gerbang `_wall_scored` di `mission_fsm.py`) diterbitkan **hanya saat decode sukses**
(L140, `if not data: ... return`). Jadi satu event deteksi menghasilkan dua output yang
di-gerbang terpisah: **pemilihan wall butuh decode; offset visual-servo cuma butuh corner
terdeteksi, tanpa validasi.**

**Jawaban langsung pertanyaan Task 2**: ya — pada frame corner-only, `qr_logic.py` meneruskan
`best_pts` apa adanya (berpotensi dari varian citra MENTAH, yang paling terpapar artefak
render sim) langsung ke `mission_fsm.py` lewat `/hydroships/qr_offset`, tanpa filter
plausibilitas di mana pun sepanjang rantai. Satu-satunya gerbang di `mission_fsm.py` adalah
freshness berbasis waktu (`qr_age < qr_max_age`), yang memvalidasi *kebaruan*, bukan
*plausibilitas geometris*. Ini mekanisme konkret yang sudah terikat ke baris kode persis untuk
bias residual corner-only yang sudah diukur P0-2.3 (mean −0.176m, sampai −0.19m pada observasi
degenerate individual seperti outlier `R4` yang sudah dikonfirmasi ~30.000px di luar frame) —
bukan temuan baru, tapi sekarang terhubung ke baris kode pasti, bukan cuma inferensi statistik.

## 3. Baseline observables, dihitung segar dari 17 run valid P0-2.4

Pass Python read-only atas `/tmp/p0-2-4-battery/*.csv` (masih ada), mereplikasi persis logika
`off_fresh`/`dist_raw` yang sudah dipakai sepanjang P0-2.4/P0-2.5 (`gripper_base_dx=0.18`,
`qr_max_age=1.5`, gerbang servo `dist_raw<0.3`). Total 1.829 tick `APPROACH_QR` lintas 17 run.

**Keseluruhan (semua tick):**

| Metrik | Nilai |
|---|---|
| Decode Success Rate (DSR) | **10.7%** (195/1829) |
| Corner-only rate (corner terlihat, decode gagal) | **83.7%** (1531/1829) |
| No-corner rate (dropout total — tanpa corner, tanpa decode) | **5.6%** (103/1829) |
| Transisi `off_fresh` → stale (umur melewati 1.5s) | 15 total lintas 17 run |

**Di dalam window aktivasi visual-servo (`dist_raw < 0.3`, 724 tick):**

| Metrik | Nilai |
|---|---|
| DSR | **26.0%** (188/724) — lebih baik dari keseluruhan, tapi tetap minoritas |
| Corner-only rate | **74.0%** (536/724) |
| No-corner rate | **0.0%** (0/724) — corner SELALU terdeteksi pada jarak dekat |

**Sebaran per-run**: DSR mean 11.7%, median 8.1%, rentang 0%-41.9% lintas 17 run (`U3, U5, W1,
W4, W5, W6` semuanya persis 0.0% sepanjang episode — run zero-decode Mode-1 yang ditandai
P0-2.5).

**Membaca semuanya bersama**: sistem persepsi nyaris tidak pernah gagal *melihat* QR pada jarak
dekat (0% dropout total di window servo) — bottleneck-nya nyaris seluruhnya di **decode**, bukan
**deteksi**. Ini mengubah "perbaiki `qr_detector.py`" dari target samar jadi spesifik: langkah
pencarian corner/quad sudah solid; langkah pembacaan simbol `detectAndDecode` yang gagal
~74-90% walau quad jelas ada.

## 4. Kandidat perbaikan persepsi terisolasi (BELUM disetujui, hanya diformulasikan)

Masing-masing terikat metrik spesifik dari §1-3, satu-variabel, mengikuti disiplin P0-2.5
(default-off, terisolasi, guardrail-checked). Tidak satu pun dari ini diimplementasikan atau
dijadwalkan — diserahkan ke manusia untuk urutan approve → implement → battery → analyze yang
sama seperti P0-2.5.

1. **Urutan ulang/perluas urutan preprocessing `_candidates()`** (`qr_logic.py:57-90`).
   Hipotesis: karena `best_pts` terkunci ke kandidat *pertama* dengan corner points (bukan yang
   akhirnya decode), dan DSR jauh lebih rendah dari rate deteksi-corner, sebagian kandidat
   belakangan (Otsu, upscaled) mungkin lebih sering berhasil decode daripada kandidat awal —
   bisa diuji dengan instrumentasi *kandidat mana* yang benar-benar menghasilkan decode sukses
   saat itu terjadi, sebelum mengusulkan pengurutan ulang. Indikator: distribusi "indeks kandidat
   pemenang" di antara 195 decode sukses pada data P0-2.4, atau battery baru yang
   diinstrumentasi.
2. **Gerbang plausibilitas corner sebelum menerbitkan `/hydroships/qr_offset` saat decode
   gagal** (`qr_detector.py:136`). Hipotesis: menolak `pts` yang gagal cek kewajaran murah
   (dalam batas frame, quad konveks, rasio panjang sisi) sebelum diterbitkan bisa mengurangi
   kontribusi bias residual dari observasi corner-only (mean P0-2.3 −0.176m, sebagian didorong
   kasus degenerate terkonfirmasi seperti `R4`). Indikator: distribusi residual `dist_diff_raw`
   untuk observasi corner-only, dengan vs tanpa filter plausibilitas yang disimulasikan offline
   pada data battery yang sama (bisa diuji TANPA menyentuh detector, memakai pendekatan analisis
   gaya-P0-2.3 pada data corner topic debug).
3. **Kenaikan `max_rate`** (saat ini 5.0 Hz, `qr_detector.py:53`) khusus selama window servo.
   Hipotesis: karena decode success di dalam window servo (26.0%) sudah lebih baik dari
   keseluruhan, dan `off_fresh`/`qr_max_age` men-gerbang staleness di 1.5s, rate deteksi lebih
   tinggi dekat target bisa menaikkan jumlah tick decode-sukses yang *segar* tersedia untuk
   controller tanpa mengubah logika decode itu sendiri. Indikator: fraksi `off_fresh` dan
   `servo_dsr` pada rate lebih tinggi vs baseline 5 Hz saat ini.

Tidak satu pun dari ketiganya punya hipotesis cukup kuat untuk langsung lompat ke implementasi —
sesuai execution rule task ini, bagian ini berhenti di formulasi.

## 5. Status

```text
P0-2.5                              CLOSED — 4 kandidat diuji, tidak ada yang membalik
                                     Gate 4 FAIL (docs/P0-2-AUDIT.md §7)
P0-2.6                               DIAGNOSIS — evidence dikumpulkan & dikuantifikasi,
                                     TIDAK ADA kesimpulan bahwa qr_detector.py harus
                                     ditulis ulang
  Re-audit P0-2.3                     Angka terpublikasi dikutip ulang (data mentah tak
                                     tersedia) — §1
  Trace pipeline persepsi              qr_logic.py/qr_detector.py dibaca, TIDAK DIUBAH —
                                     konfirmasi corner-only diteruskan tanpa validasi
                                     plausibilitas ke mission_fsm.py — §2
  Baseline observables                 DIHITUNG SEGAR dari 17 run P0-2.4 valid: DSR 10.7%
                                     keseluruhan / 26.0% window servo, corner-only 83.7%/
                                     74.0%, dropout total 5.6%/0.0% — §3
  Kandidat perbaikan persepsi          3 kandidat terisolasi DIFORMULASIKAN, BELUM
                                     disetujui/diimplementasi — §4
qr_detector.py / qr_logic.py /       TIDAK DIUBAH
  mission_fsm.py
Implementasi                         BELUM DIMULAI — menunggu review diagnostic ini
                                     sebelum kandidat manapun di §4 disetujui
```
