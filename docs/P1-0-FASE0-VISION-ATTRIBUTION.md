# P1-0 Fase 0 — Pemisahan evidence: vision vs ground truth

**STATUS: PERUBAHAN TOOLING SAJA.** Tidak ada perubahan pada `mission_fsm.py`,
`qr_detector.py`, `qr_logic.py`, controller, parameter, world, atau URDF. Satu-satunya
berkas yang diubah: `tools/p0-experiments/reduce_approach_qr.py`.

Tanggal: 2026-08-12 · Baseline: `2fa7b76` · Konteks: [P1-0-ARCHITECTURE-AUDIT.md](P1-0-ARCHITECTURE-AUDIT.md)

## 1. Masalah

Verdict Gate 4 (`docs/P0-2-4-SPEC.md` S6-S7) dihitung dari `entered_band_with_dwell`,
yang di `reduce_approach_qr.py` dibangun sebagai:

```python
combined_entered.append(centered_i or dist_l[i] < params['approach_tol'])
```

`centered_i` adalah kriteria kamera. `dist_l[i] < approach_tol` adalah jarak ke
`/hydroships/payload_pose` — **koordinat spawn ground truth** dari `payload_spawner.py`.
Keduanya di-OR, lalu hasilnya dipakai sebagai satu angka.

Akibatnya angka itu **tidak bisa menjawab "apakah kamera berkontribusi?"**. Ia hanya
menjawab "apakah ROV sampai ke pita toleransi", dan homing PD terhadap koordinat sempurna
sudah cukup untuk itu sendirian. `docs/P0-2-5-ENGINEERING-ANALYSIS.md` §0 sudah menemukan
gejalanya (6/17 run dengan `qr_decode_rate=0.000` tetap lolos); Fase 0 menarik konsekuensi
pelaporannya.

## 2. Perubahan

Dwell test yang sama (`DWELL_TICKS=3`, tak diubah) kini dijalankan atas **empat deret
kriteria** alih-alih satu:

| Deret | Definisi | Arti |
|---|---|---|
| `combined` | `camera OR ground_truth` | **angka lama, tidak berubah** — statistik misi end-to-end |
| `camera` | offset dari kamera (`off_fresh` + dalam `qr_center_tol`) | kamera menghasilkan offset — termasuk *corner-only* saat decode gagal |
| `decoded` | `camera` **dan** tick itu `qr_decode_success==1` | kriteria paling ketat: QR benar-benar terbaca |
| `ground_truth` | `dist < approach_tol` ke `payload_pose` | odometri murni, tanpa kamera sama sekali |

Pemisahan `camera` vs `decoded` penting karena `qr_logic.robust_decode` mengembalikan
`best_pts` (titik sudut) **walaupun decode gagal**, dan `qr_detector` tetap menerbitkan
`qr_offset` dari titik itu. Centering bisa terjadi dari sudut yang tak pernah ter-decode —
tetap informasi kamera, tapi bukan bukti QR terbaca.

Ditambahkan juga atribusi per-run (`converged_via`): dwell mana yang tercapai lebih dulu.
**Seri dianggap milik `GROUND_TRUTH`,** karena homing ground truth berjalan tanpa syarat
dan karenanya adalah hipotesis nol yang harus dikalahkan kamera.

Perubahan bersifat **aditif**: seluruh key JSON dan baris output lama identik
bit-per-bit (diverifikasi, lihat §4).

## 3. Hasil pada data battery yang ada

Dihitung ulang dari CSV battery yang sudah ada — **tidak ada run baru yang dijalankan**.
Data P0-2.4 asli (`/tmp/p0-2-4-battery`) sudah terhapus, jadi angka 5/17 di
`docs/P0-2-4-RESULTS.md` tidak dapat dipecah ulang; tiga battery berikut bisa.

| Battery | n | combined | camera | **decoded** | ground truth saja | QR tak pernah decode |
|---|---|---|---|---|---|---|
| P0-2.6 | 5 | 2 (40%) | 0 | **0 (0%)** | 2 | 1/5 |
| P0-2.7 (failure-focused) | 17 | 8 (47%) | 5 | **3 (18%)** | 6 | 7/17 |
| P0-2.8 | 28 | 11 (39%) | 4 | **3 (11%)** | 11 | 12/28 |

Atribusi "siapa duluan sampai pita" pada P0-2.8 (n=28): `VISION`=4, `GROUND_TRUTH`=7,
`NONE`=17.

**Pembacaan:** angka Gate 4 yang selama ini dilaporkan (39–47%) melebih-lebihkan kinerja
persepsi dengan faktor ~3–4×. Angka jujur untuk persepsi adalah **11–18%**, dan pada
battery terbesar **12 dari 28 run tidak pernah men-decode QR sama sekali**.

Ini **tidak** membatalkan verdict FAIL Gate 4 — verdict itu tetap FAIL, dan kini dengan
alasan yang lebih tepat: bukan sekadar "controller kurang presisi", melainkan sebagian
besar run tidak punya input vision untuk dipresisikan.

## 4. Verifikasi

```bash
# self-check logika dwell + atribusi (tanpa data, tanpa ROS)
python3 tools/p0-experiments/reduce_approach_qr.py --selftest

# jalankan atas battery yang ada
TAGS=$(ls /tmp/p0-2-8-battery/*.csv | xargs -n1 basename | sed 's/.csv//' | tr '\n' ' ')
python3 tools/p0-experiments/reduce_approach_qr.py /tmp/p0-2-8-battery $TAGS

# bukti aditif: output versi lama vs baru, tak satu pun baris lama hilang/berubah
git show <baseline>:tools/p0-experiments/reduce_approach_qr.py > /tmp/orig.py
diff <(python3 /tmp/orig.py <dir> $TAGS) <(python3 tools/p0-experiments/reduce_approach_qr.py <dir> $TAGS) | grep '^<'
```

Diverifikasi 2026-08-12: `diff | grep '^<'` kosong; JSON pra-eksisting identik setelah
key `p1_0_*` dilepas; invarian `decoded ≤ camera ≤ combined` dan
`sum(converged_via) == n_reached` terpenuhi.

## 5. Batasan

- **Belum diverifikasi runtime** — reducer tidak dijalankan atas battery baru; semua angka
  di §3 adalah perhitungan ulang atas CSV lama.
- Angka 5/17 di `docs/P0-2-4-RESULTS.md` **tidak** dikoreksi di sini (datanya hilang).
  Yang berlaku: dokumen itu melaporkan populasi `combined`, bukan performa persepsi.
- Atribusi berbasis *urutan dwell*, bukan analisis kausal. Run `converged_via=VISION`
  berarti kriteria kamera lebih dulu terpenuhi — bukan bukti bahwa perintah controller
  benar-benar digerakkan oleh offset kamera. Untuk itu, Gate 3
  (`with_qr_fits_better`) yang sudah ada tetap alat yang tepat.
- Fase 0 **tidak** menghilangkan ketergantungan ground truth; ia hanya membuatnya terlihat
  di angka. Menghilangkannya adalah Fase 2.
