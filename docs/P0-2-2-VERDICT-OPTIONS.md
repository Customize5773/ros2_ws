# P0-2.2 VERDICT OPTIONS (KKI 2026)

Dokumen ini **tidak memilih opsi**. Ia menyusun opsi-opsi penutupan P0-2.2 berdasarkan
evidence yang sudah dihasilkan P0-2.2b (`docs/P0-2-2-SPEC.md` §7,
`P0-2-2b-results.json`), supaya keputusan ada di tangan yang berwenang mengambilnya —
konsisten dengan disiplin yang dipegang sepanjang P0-1/P0-2: audit dan eksperimen
menghasilkan evidence, bukan verdict.

## 1. Ringkasan evidence (rekap, bukan analisis baru)

Dari 6 run battery (`Q1`-`Q6`, 6/6 valid, 0 `INCONCLUSIVE`):

| Run | Exit path (Gate 5, dari log FSM) | Gate 3: koreksi-QR fit lebih baik? | Gate 4: masuk band `qr_center_tol`? |
|---|---|---|---|
| Q1 | `QR_SCORED_XY_TOL` | Ya | Tidak |
| Q2 | `QR_SCORED_XY_TOL` | Tidak | Tidak |
| Q3 | `QR_SCORED_XY_TOL` | Ya | Tidak |
| Q4 | `GROUND_TRUTH_FALLBACK` | Ya | Tidak |
| Q5 | `QR_SCORED_XY_TOL` | Ya | Tidak |
| Q6 | `QR_SCORED_VISUAL_SERVO` | Ya | Tidak |

Ringkas: **5/6 run QR ter-score** (wall letter terbaca sebelum exit); dari 5 itu, **hanya 1/6
(Q6) exit lewat visual centering asli** — 4/6 exit lewat toleransi jarak XY meski QR sudah
terbaca. **1/6 (Q4) murni ground-truth-fallback**, QR tidak pernah terbaca sama sekali.
**5/6 run** menunjukkan command aktual lebih cocok dengan model "dengan koreksi QR"
dibanding model "ground-truth murni" (Gate 3). **0/6 run** pernah masuk band
`qr_center_tol=0.12` (Gate 4) — tren error juga campur, tidak konsisten menurun lintas run.

Ini **evidence**, bukan kesimpulan PASS/FAIL — dan tetap begitu di seluruh dokumen ini.

## 2. Opsi-opsi

### Opsi A — CLOSE: "QR di dalam loop, presisi belum terbukti"

**Makna menutup P0-2.2 dengan opsi ini**: mengakui bahwa QR benar-benar berkontribusi secara
kausal ke command (Gate 3, 5/6 run) — pertanyaan P0-2.2 "apakah QR benar-benar menjadi input
kontrol" terjawab **ya** — tapi presisi visual-centering (`qr_center_tol`) tidak pernah
terdemonstrasi (Gate 4, 0/6). P0-2.3 kemudian didesain untuk secara eksplisit menguji akurasi
posisi/pose gripper SAAT `GRAB` terjadi (bukan sekadar "apakah GRAB terjadi") — terutama untuk
4/6 run yang exit lewat toleransi jarak XY, bukan visual centering.

**Mendukung**: Gate 3 cukup konsisten (5/6) untuk menjawab pertanyaan kausal utama P0-2.2.
Gate 5 juga konsisten dalam arti "QR ter-score" (5/6), meski jalur exit-nya bervariasi.

**Melawan**: Gate 4 di 0/6 berarti belum ada bukti presisi apa pun untuk didasarkan P0-2.3;
menutup sekarang berisiko P0-2.3 mendesain acceptance criteria tanpa baseline presisi yang
jelas.

**Langkah berikutnya jika dipilih**: desain P0-2.3 dengan metrik presisi eksplisit (jarak
gripper-ke-QR aktual saat `GRAB`, bukan hanya `dist<approach_tol`).

### Opsi B — CLOSE-PARTIAL: pisahkan pertanyaan acceptance

**Makna**: pertanyaan asli P0-2 sebenarnya membundel dua hal — (1) "apakah QR berkontribusi ke
pemilihan wall/keputusan" dan (2) "apakah QR memungkinkan approach presisi". Tutup hanya (1)
sebagai **terjawab (ya, 5/6)**; biarkan (2) tetap eksplisit terbuka, dibawa ke P0-2.3 sebagai
pertanyaan utamanya (bukan pertanyaan baru).

**Mendukung**: lebih jujur secara evidence — tidak menutup satu pertanyaan gabungan dengan
data yang cuma mendukung separuhnya.

**Melawan**: menambah birokrasi status (P0-2.2a/b sudah dua sub-tahap; ini menambah lagi
pemisahan konseptual) yang mungkin tidak perlu kalau Opsi A dianggap sudah cukup jelas.

**Langkah berikutnya jika dipilih**: update acceptance matrix `docs/P0-2-AUDIT.md` §2 baris
"QR terdeteksi secara konsisten?" dan "FSM keluar dengan benar?" saja yang di-resolve;
baris presisi/konvergensi tetap `OPEN` eksplisit menunggu P0-2.3.

### Opsi C — ITERATE: kumpulkan data lebih banyak sebelum memutuskan

**Makna**: N=6 dianggap belum cukup besar untuk percaya distribusi 4/6-1/6-1/6 itu stabil —
jalankan battery lebih besar atau lebih terarah (mis. skenario dengan payload di-spawn supaya
QR terlihat lebih lama, untuk menguji apakah band `qr_center_tol` pernah tercapai kalau
diberi waktu/kondisi lebih baik) sebelum menyimpulkan apa pun tentang P0-2.2.

**Mendukung**: 0/6 pada Gate 4 bisa jadi karena kondisi spawn battery ini kebetulan tidak
memberi cukup waktu di jendela servoing (n_servoing_rows per run cuma 16-53 dari ratusan baris
`APPROACH_QR`) — bukan berarti band itu TAK PERNAH bisa dicapai.

**Melawan**: biaya nyata (satu battery lagi ≈10-12 menit wall-clock + waktu analisis), dan
tidak ada jaminan N lebih besar mengubah kesimpulan — arsitektur controllernya (target selalu
ground-truth-anchored, servo cuma koreksi kecil bersyarat `dist_raw<0.3`) sama di semua run.

**Langkah berikutnya jika dipilih**: desain battery lanjutan (bisa pakai
`run_approach_qr_battery.sh` sebagai basis, dimodifikasi jumlah run/kondisi spawn) — pass
terpisah, bukan bagian dokumen ini.

### Opsi D — REDESIGN: pertanyakan apakah `qr_center_tol` bar yang tepat

**Makna**: dari `docs/P0-2-2-SPEC.md` §1, target `_goto_xy()` **selalu** ground-truth-anchored;
visual servo cuma koreksi aditif kecil, aktif hanya dalam radius 0.3 m. Mungkin
"presisi via `qr_center_tol`" bukan bar yang tepat untuk *arsitektur ini* — mungkin
acceptance P0-2.3 sebaiknya didefinisikan ulang di sekitar apa yang arsitektur ini memang
lakukan (konvergensi XY dengan QR sebagai gerbang pemilihan wall + koreksi kecil), bukan bar
presisi visual-servo yang sistem ini tidak pernah di-tuning untuk mencapainya secara ketat.

**PENTING — batas eksplisit opsi ini**: ini HANYA mengangkat pertanyaan desain untuk
didiskusikan. Opsi ini **tidak** mengusulkan perubahan arsitektur/kode/parameter di P0-2.2 atau
P0-2.3 sekarang — itu melanggar prinsip yang sudah ditegakkan sejak P0-1: jangan menambal
sistem demi mempermudah acceptance test. Kalau opsi ini dipilih, hasilnya adalah **diskusi**,
bukan tindakan otomatis.

**Mendukung**: `qr_center_tol=0.12` mungkin memang tidak realistis dicapai given `qr_servo_gain`
kecil (0.15) dan jendela servoing sempit (radius 0.3 m) — 0/6 di enam run yang beragam
kondisi spawn adalah sinyal yang cukup kuat untuk pertanyaan ini layak diajukan.

**Melawan**: mengganti target acceptance karena implementasi belum mencapainya berisiko
menjadi post-hoc rationalization — perlu kehati-hatian ekstra supaya tidak dianggap "menambal
test", bukan "menambal sistem" (secara teknis beda, tapi mudah disalahartikan).

**Langkah berikutnya jika dipilih**: diskusi terpisah (bukan tindakan kode) tentang definisi
acceptance P0-2.3, sebelum eksperimen apa pun dimulai.

## 3. Yang tidak berubah apa pun opsi yang dipilih

- Acceptance matrix `docs/P0-2-AUDIT.md` §2 tetap `OPEN` sampai opsi dipilih **dan**
  langkah berikutnya opsi itu benar-benar dieksekusi.
- Tidak ada perubahan `mission_fsm.py`/`qr_detector.py`/`qr_logic.py`/parameter apa pun akibat
  menulis dokumen ini.
- Dokumen evidence (`docs/P0-2-2-SPEC.md` §7) dan data mentah (`/tmp/p0-2-2b-battery/*`) tidak
  diubah oleh dokumen ini — dokumen ini murni lapisan keputusan di atasnya.

## 4. Keputusan

**Opsi B — CLOSE-PARTIAL dipilih.** Alasan yang diberikan (ringkas): Gate 3 (5/6 run, command
mengikuti `qr_offset`) dan Gate 5 (QR memang berkontribusi ke proses menuju `GRAB`) cukup
untuk menutup klaim "QR integration/causal influence terbukti" — tapi Gate 4 (0/6 masuk band
`qr_center_tol`) dan exit path yang tidak seragam (`QR_SCORED_XY_TOL` / `GROUND_TRUTH_FALLBACK`
/ `QR_SCORED_VISUAL_SERVO` ketiganya muncul) berarti klaim "QR visual servo memberikan
precision convergence yang repeatable" **belum** boleh ditutup. Opsi A ditolak karena berisiko
membuat status P0-2 terdengar seolah seluruh acceptance APPROACH_QR selesai. Opsi C (ITERATE)
ditunda — nilai lebih besar ada di menguji dulu apa yang benar-benar dibutuhkan P0-2.3
sebelum menambah data untuk gate yang mungkin bukan acceptance akhir yang tepat. Opsi D
(REDESIGN) tetap sebagai pertanyaan desain terbuka, bukan tindakan sekarang.

```text
P0-2.2
  Design                     CLOSED
  2.2a                       CLOSED
  2.2b                       CLOSED
  QR influence                VERIFIED
  QR precision convergence    OPEN
  Overall                     CLOSE-PARTIAL
```

Konsekuensi: tidak ada re-run P0-2.2. Langkah berikutnya adalah desain P0-2.3, dengan scope
diperjelas: menguji apakah `APPROACH_QR` menghasilkan positioning/centering yang benar pada
titik `GRAB`, bukan sekadar apakah QR pernah memengaruhi command. Lihat
[`docs/P0-2-3-SPEC.md`](P0-2-3-SPEC.md).
