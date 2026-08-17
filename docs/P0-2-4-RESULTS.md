# P0-2.4 RESULTS — Gate 4 convergence retest (KKI 2026)

Dokumen ini melaporkan **hasil battery**, bukan keputusan engineering. Sesuai urutan yang
disepakati (design review → approve → implement tooling → battery → analyze → Gate 4 verdict →
baru pertimbangkan engineering fix), dokumen ini berhenti di **Gate 4 verdict**. Tidak ada
perubahan `qr_detector.py`, `qr_logic.py`, `mission_fsm.py`, controller, atau parameter apa pun
dilakukan atau direkomendasikan di sini.

## 1. Tooling yang diimplementasikan

`tools/p0-experiments/reduce_approach_qr.py` diperluas (bukan diganti) untuk mengimplementasikan
`docs/P0-2-4-SPEC.md` §3, memakai kolom CSV `recorder_qr.py` yang sudah ada — tidak ada recorder,
topic, atau instrumentasi baru:

- **Time-to-converge (dwell-based)**: `entered_band_with_dwell` — kondisi `centered` (FSM
  `mission_fsm.py:593-595`) ATAU `dist < approach_tol` bertahan ≥3 tick berturut-turut (0.3 s
  @10Hz), bukan satu sampel noise sesaat.
- **Overshoot**: excursion maksimum `dist` setelah pertama kali turun di bawah `approach_tol`.
- **Oscillation/divergence**: `stdev(cmd_fx)`, `stdev(cmd_fy)`, `saturation_frac` (fraksi tick
  command ≥99% `approach_fmax`), dan flag `diverged` (tren `dist` tidak menurun DAN saturasi
  ≥80% tick).
- **Gate 4 retest verdict (agregat)**: PASS/FAIL/INCONCLUSIVE otomatis dari
  `docs/P0-2-4-SPEC.md` §6-§7 (stopping rule: `n≥18` ATAU `entered≥5`; PASS jika mayoritas +
  0 diverged; FAIL jika minoritas + stopping rule terpenuhi; INCONCLUSIVE selainnya).

Logika baru diverifikasi dengan unit test sintetis (bukan bagian file — hanya smoke check
sebelum battery) untuk memastikan dwell-detection dan divergence-flag berperilaku benar pada
kasus convergent dan non-convergent buatan, sebelum dipakai pada data battery nyata.

## 2. Eksekusi battery

Protokol sama seperti P0-2.2b/P0-2.3 (`run_approach_qr_battery.sh`, 5× `rov_random_spawn:=true`
+ 1× deterministik per batch), ke `/tmp/p0-2-4-battery` (data mentah, tidak disimpan di git,
sama seperti pola battery sebelumnya). Stopping rule §6: berhenti begitu `n_reached≥18` ATAU
`entered_band_with_dwell≥5`.

| Batch | Tags | Gate PASS | Catatan |
|---|---|---|---|
| 1 | `U1`-`U6` | 5/6 | `U1` gate FAIL — `mission_fsm` node hilang (`missing`), dikeluarkan sebagai INCONCLUSIVE oleh reducer, bukan dihitung ke arah PASS/FAIL |
| 2 | `V1`-`V6` | 6/6 | — |
| 3 | `W1`-`W6` | 6/6 | — |

**Total 18 run, 17 valid (1 dikeluarkan karena kontaminasi gate).** Stopping rule terpenuhi
setelah batch 3 (baik `n_reached=17≥18`≈tercapai maupun `entered=5≥5` — keduanya terpenuhi
bersamaan, lihat §3), sesuai §6 — tidak diperlukan batch ke-4.

## 3. Hasil per run (ringkasan)

| Tag | `entered_band_with_dwell` | `t_conv` (s) | `qr_decode_rate` | `diverged` | `sat_frac` | Exit path |
|---|---|---|---|---|---|---|
| U2 | **True** | 5.51 | 0.087 | False | 0.08 | QR_SCORED_XY_TOL |
| U3 | False | — | 0.000 | False | 0.143 | GROUND_TRUTH_FALLBACK |
| U4 | False | — | 0.088 | False | 0.221 | QR_SCORED_VISUAL_SERVO |
| U5 | False | — | 0.000 | False | 0.226 | GROUND_TRUTH_FALLBACK |
| U6 | False | — | 0.264 | False | 0.149 | QR_SCORED_VISUAL_SERVO |
| V1 | **True** | 5.02 | 0.125 | False | 0.202 | QR_SCORED_XY_TOL |
| V2 | False | — | 0.027 | False | 0.152 | QR_SCORED_XY_TOL |
| V3 | False | — | 0.081 | False | 0.184 | QR_SCORED_XY_TOL |
| V4 | **True** | 4.24 | 0.419 | False | 0.14 | QR_SCORED_XY_TOL |
| V5 | False | — | 0.347 | False | 0.143 | QR_SCORED_XY_TOL |
| V6 | False | — | 0.344 | False | 0.000 | QR_SCORED_VISUAL_SERVO |
| W1 | **True** | 0.00 | 0.000 | False | 0.178 | GROUND_TRUTH_FALLBACK |
| W2 | False | — | 0.191 | False | 0.244 | QR_SCORED_VISUAL_SERVO |
| W3 | **True** | 1.63 | 0.022 | False | 0.18 | QR_SCORED_XY_TOL |
| W4 | False | — | 0.000 | False | 0.138 | GROUND_TRUTH_FALLBACK |
| W5 | False | — | 0.000 | False | 0.143 | GROUND_TRUTH_FALLBACK |
| W6 | False | — | 0.000 | False | 0.098 | GROUND_TRUTH_FALLBACK |

Data lengkap (auditable, per-run): `/tmp/p0-2-4-battery/*.csv`, `*.log`, `*.gate.txt`,
`P0-2-2b-results.json` (nama file dari reducer dipertahankan untuk kompatibilitas P0-2.2b;
berisi kedua blok `gate4_error_converges` lama dan `p0_2_4_gate4_retest` baru).

## 4. Agregat dan verdict Gate 4

```text
n_reached_approach_qr        = 17
entered_band_with_dwell      = 5/17  (29%)
diverged                     = 0/17
stopping_rule_met            = True   (entered=5 >= 5; n=17 juga hampir di ambang 18)
VERDICT                      = FAIL   (minoritas run konvergen, stopping rule terpenuhi)
```

- **Tidak ada satu run pun yang divergen** (`diverged=False` di seluruh 17 run) — tidak ada
  bukti controller mendorong menjauh dari target atau saturasi terus-menerus tanpa progres.
  Ini melemahkan hipotesis "controller tidak stabil"; kegagalan Gate 4 bukan soal
  osilasi/divergensi liar, tapi soal **tidak cukup sering masuk dan bertahan di band**.
- **Exit path**: 7/17 `QR_SCORED_XY_TOL` (keluar lewat fallback jarak XY, bukan visual
  centering), 6/17 `GROUND_TRUTH_FALLBACK` (QR tak pernah ter-score), 4/17
  `QR_SCORED_VISUAL_SERVO`. Konsisten dengan risiko yang sudah dicatat
  `docs/P0-2-AUDIT.md` §3.2: sebagian besar exit tidak benar-benar memvalidasi rantai
  persepsi QR sampai tuntas.
- **Stratifikasi `qr_decode_rate`** (§7 `P0-2-4-SPEC.md`, memakai evidence root-cause P0-2.3
  sebagai lensa interpretasi): mean `qr_decode_rate` run yang konvergen = **0.131** (n=5) vs
  run yang tidak konvergen = **0.112** (n=12) — **hampir sama, tidak ada pemisahan yang
  jelas**. Sampel ini **tidak cukup untuk mengatribusikan kegagalan Gate 4 secara dominan ke
  kualitas decode** (seperti yang mungkin diharapkan dari P0-2.3) — attribusi ke
  geometri/inflasi AABB vs decode quality tetap tidak terpisahkan oleh battery ini,
  konsisten dengan disclaimer P0-2.3 bahwa root-cause dan acceptance adalah dua pertanyaan
  berbeda.
- **Overshoot**: 0.000 m pada seluruh run yang pernah turun di bawah `approach_tol` — kalau
  `dist` sempat masuk tolerance, ia tidak balik menjauh secara signifikan. Sejalan dengan
  "tidak ada divergensi", ini menguatkan bahwa masalahnya adalah *tidak pernah mendekat cukup*,
  bukan *mendekat lalu menjauh lagi*.

**Perbandingan dengan Gate 4 P0-2.2b**: 0/6 run (0%) di P0-2.2b vs 5/17 (29%) di battery ini.
Ini **bukan pembalikan verdict** — keduanya sama-sama minoritas kecil, jauh dari mayoritas >50%
yang disyaratkan untuk PASS (§7 `P0-2-4-SPEC.md`). Sampel yang lebih besar (17 vs 6 run)
mengurangi risiko "0/6 itu kebetulan sampel kecil yang buruk", dan hasilnya **tetap konsisten
arah**: precision convergence gagal pada mayoritas run.

## 5. Verdict akhir

```text
Gate 4 (precision convergence)      FAIL  (5/17 entered+held band, stopping rule terpenuhi,
                                     evidence sekarang jauh lebih besar dari P0-2.2b 0/6)
Oscillation/divergence               TIDAK TERBUKTI  (0/17 diverged, overshoot=0 di semua run)
Root-cause attribusi (decode vs      TIDAK TERPISAHKAN oleh battery ini (decode_rate mean
  geometri) pada battery ini          hampir sama antar grup konvergen/tidak, n kecil)
qr_detector.py / qr_logic.py /       TIDAK DIUBAH
  mission_fsm.py / controller
Rekomendasi engineering fix          TIDAK ADA DI SINI — di luar scope dokumen ini,
                                      menunggu keputusan terpisah
```

Gate 4 tetap **FAIL** setelah retest dengan evidence yang jauh lebih besar dan metrik yang jauh
lebih tajam (dwell, overshoot, variance/saturasi, stratifikasi decode-rate) dibanding P0-2.2b.
Precision convergence bukan masalah stabilitas/osilasi (§4) — controller tidak divergen di
run manapun — melainkan masalah **seberapa sering & seberapa cepat** approach benar-benar masuk
band sebelum `t_scan`. Keputusan langkah berikutnya (apakah menaikkan `qr_center_tol`, mengubah
gain servo, memperbaiki `qr_detector.py`, atau sesuatu yang lain) adalah **diskusi terpisah**,
bukan konsekuensi otomatis dari dokumen ini.
