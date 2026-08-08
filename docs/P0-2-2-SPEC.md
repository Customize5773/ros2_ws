# P0-2.2 SPEC — QR-driven vs Ground-truth-driven APPROACH_QR (KKI 2026)

Dokumen ini adalah **spec, bukan hasil eksperimen**. Tidak ada kode yang ditulis, tidak ada
simulator yang dijalankan, tidak ada parameter yang diubah dalam pembuatan dokumen ini.
Tujuannya: mendesain eksperimen minimum yang bisa membedakan **QR-driven approach** dari
**ground-truth-driven approach** di `APPROACH_QR`, sebelum eksperimen itu benar-benar
dijalankan.

Prasyarat: [`docs/P0-2-AUDIT.md`](P0-2-AUDIT.md) (P0-2.0, CLOSED — audit statis) dan §6
dokumen yang sama (P0-2.1, CLOSED — instrumentasi + smoke test terverifikasi). Acceptance
matrix di `P0-2-AUDIT.md` §2 **tetap `OPEN`** setelah dokumen ini — spec tidak mengubah
verdict apa pun.

---

## 1. Mengapa menghitung transisi state saja tidak cukup

Audit P0-2.0 menemukan risiko kritis (§3.2 `P0-2-AUDIT.md`): `GRAB` bisa dicapai lewat
konvergensi XY murni terhadap `payload_pose` ground-truth tanpa QR pernah terbaca. Membaca
ulang `_st_approach_qr()` dengan fokus ini menunjukkan sesuatu yang lebih halus daripada
"dua jalur terpisah":

- Target XY `(tx, ty)` **selalu** dimulai dari `payload_pose` ground-truth
  (`mission_fsm.py:552`).
- Koreksi visual-servo dari `qr_offset` (`qr_servo_gain=0.15`, digerbang `off_fresh` dan
  `dist_raw<0.3`, L558-567) hanya **ditambahkan** ke atas target itu — bukan menggantikannya.
- Controller `_goto_xy()` (L359-382) tidak pernah tahu apakah target yang ia kejar murni
  ground-truth atau ground-truth+koreksi-visual — ia hanya mengejar `(tx, ty)`.

Jadi "Path A" (QR-driven) dan "Path B" (ground-truth-driven) di diagram bukan dua cabang kode
yang eksklusif — keduanya adalah **superposisi** pada controller yang sama. Satu-satunya
tempat di kode yang benar-benar membedakan "QR-driven" dari "ground-truth-only" adalah dua
kondisi exit ke `GRAB`:

- **L590-601**: exit karena `_wall_scored` (QR benar-benar ter-parse) **dan**
  (`centered` visual **atau** `dist < approach_tol`).
- **L603-615**: exit karena `dist < approach_tol` saja, `_wall_scored` masih `False` — QR
  tidak pernah berkontribusi pada keputusan exit ini sama sekali.

Karena itu, menghitung "`APPROACH_QR → GRAB` terjadi" tidak memberi tahu kita apa pun tentang
apakah visual servo benar-benar mengubah lintasan ROV, atau apakah convergence sepenuhnya
bisa dijelaskan oleh PD ground-truth-only. P0-2.2 harus mengukur **kontribusi kausal** koreksi
visual, bukan sekadar keberhasilan transisi.

## 2. Prinsip metodologis (dibawa dari P0-1 & diperkuat pengguna)

- **Tidak ada ablation.** Tidak boleh mematikan `/hydroships/qr_offset` sementara, memalsukan
  `payload_pose`, atau memodifikasi `mission_fsm.py`/`qr_detector.py` untuk "membuktikan"
  kausalitas. Itu mengubah sistem demi mempermudah test — pola yang eksplisit ditolak.
- Semua enam gate di bawah harus terjawab dari **observasi pasif**, memakai formula
  controller yang sudah publik (gain-gain di `docs/P0-2-AUDIT.md` §1.4) untuk membangun
  **prediksi counterfactual** — bukan mengganggu sistem nyata.
- **Jangan tambal arsitektur di P0-2.2.** Kalau analisis menunjukkan jalur ground-truth
  dominan, itu temuan yang dicatat, bukan bug yang langsung diperbaiki. Keputusan perbaikan
  arsitektur (jika perlu) menunggu setelah data cukup.
- Acceptance matrix `P0-2-AUDIT.md` §2 tidak disentuh oleh spec ini — spec hanya
  mempersiapkan cara mengisinya nanti.

## 3. Enam gate → kriteria terukur

| # | Gate (dari pengguna) | Kriteria terukur | Data dibutuhkan | Tersedia sekarang? |
|---|---|---|---|---|
| 1 | Apakah QR benar-benar menjadi input kontrol? | Konfirmasi code-trace (sudah selesai, `P0-2-AUDIT.md` §1.4: `qr_servo_gain` benar-benar menggeser target sebelum masuk `_goto_xy`) + cross-check korelasi di gate #3 | — | Ya (statis) |
| 2 | Apakah `qr_offset` berubah sesuai posisi relatif ROV? | Regresi `qr_ex`/`qr_ey` terhadap pose relatif ground-truth `(payload_pose − odom)` yang dirotasi oleh yaw; diharapkan hubungan monoton/tanda-konsisten (mendekati target → `|qr_ex|`,`|qr_ey|` mengecil) | `qr_offset` (ada), odom (ada), **`payload_pose` ground-truth (BELUM direkam)** | Sebagian — perlu 1 field recorder |
| 3 | Apakah command berubah mengikuti `qr_offset`? | Uji residual: hitung `cmd_fx/fy` prediksi dari `_goto_xy(payload_pose)` murni (reproduksi offline formula L359-382 dengan `approach_kp=90.0`, `approach_kd=140.0`, `approach_fmax=16.0`), bandingkan dengan `cmd_fx/fy` aktual yang terekam; residual (aktual − prediksi) harus berkorelasi dengan `qr_offset` selama jendela `servoing` (`dist_raw<0.3`) | `cmd` (ada), odom (ada), `qr_offset` (ada), **`payload_pose`** | Sebagian — gap sama |
| 4 | Apakah error QR konvergen? | Tren `|qr_ex|` dan `|qr_ey − ey_target|` sepanjang satu episode `APPROACH_QR`; diharapkan menurun net menuju `qr_center_tol=0.12`; ukur osilasi lewat jumlah pergantian tanda (sign-change count) pada `qr_ex`/`qr_ey` | `qr_offset` (ada) | Ya |
| 5 | Apakah convergence terjadi tanpa ground-truth shortcut? | Klasifikasi jalur exit per run dari **nilai `qr_result` terakhir sebelum transisi `APPROACH_QR→GRAB`**: huruf wall valid (A/B/C/D, sesuai `parse_wall`) tepat sebelum transisi ⇒ jalur QR-scored (L590-601); selain itu ⇒ kemungkinan jalur fallback (L603-615) yang memicu | `fsm_state` (ada), `qr_result` (ada — sudah terdemonstrasi informal di smoke run P0-2.1, `qr_result='D'` tepat di transisi) | Ya |
| 6 | Apakah hasil repeatable? | Terapkan klasifikasi + tren konvergensi di atas ke N run dengan initial condition bervariasi (pakai ulang argumen launch `rov_random_spawn`/`rov_x/y/z` seperti P0-1e); tabulasikan jalur-yang-dipakai, konvergensi Y/T, waktu-ke-konvergensi, metrik osilasi per run | Sama seperti di atas, N run | Ya (mekanis — tinggal jalankan) |

## 4. P0-2.2a — instrumentation extension (CLOSED)

`tools/p0-experiments/recorder_qr.py` (P0-2.1) tidak subscribe `/hydroships/payload_pose`.
Gate #2 dan #3 butuh ground-truth ini untuk menghitung prediksi counterfactual "PD murni
ground-truth". P0-2.2a menutup gap ini, **observability only**:

- Subscription baru di `recorder_qr.py`: `/hydroships/payload_pose` (`PointStamped`), QoS
  `QoSProfile(depth=1, durability=TRANSIENT_LOCAL)` — persis meniru cara `mission_fsm.py`
  sendiri subscribe topic ini (`mission_fsm.py:234-236`), karena `payload_spawner`
  menerbitkannya sekali dengan latch. Tidak ada publisher baru, tidak menyentuh
  `mission_fsm.py`/`qr_detector.py`/`qr_logic.py`.
- Kolom CSV baru: `payload_x, payload_y, payload_z` (append di akhir baris, skema lama tetap
  valid secara posisional untuk kolom 1-16).

**Smoke verification** (satu run, tag `A1`, `kki_arena`, 60 s, `P0_DATA_DIR=/tmp/p0-2-2a-smoke`,
via `run_approach_qr_smoke.sh` tanpa modifikasi):

| Kriteria (checklist pengguna) | Hasil | Evidence |
|---|---|---|
| Timestamp tersedia | **CONFIRMED** | Kolom `t` tetap terisi tiap baris seperti P0-2.1, 990 baris data |
| Pose payload valid | **CONFIRMED** | `payload_x=0.40814, payload_y=-1.47841, payload_z=-0.89400` — konstan (sesuai sifat latched, sekali terbit); `payload_z=-0.894` cocok persis dengan konstanta `qr_floor_z=-0.894` di `mission_fsm.py:166` (audit §1.3) — cross-check geometri sehat |
| Sinkron dengan odometry/QR data | **CONFIRMED** | Baris yang sama membawa `x,y` (odom), `qr_ex,qr_ey` (QR offset), dan `payload_x,payload_y` bersamaan pada `t` sim yang sama, mis. t=4.833s selama `APPROACH_QR` |
| CSV tetap dapat direduksi | **CONFIRMED** | 19 kolom konsisten di seluruh 990 baris, terparse bersih dengan `awk -F,`; skema kompatibel untuk `reduce_approach_qr.py` (§5) |
| Tidak ada perubahan behavior controller/FSM | **CONFIRMED** | Gate `gate_mission.sh` PASS penuh (7/7); urutan transisi FSM sama seperti P0-2.1 (`IDLE→DIVE→APPROACH_QR→...→WAIT_TRIGGER` dalam 60 s), tidak ada file kontrol yang diedit |

Catatan: 973/990 baris punya `payload_x` non-NaN — 17 baris pertama (sebelum `payload_pose`
diterima recorder, `t<3.5s`) tetap `nan`, sesuai ekspektasi (recorder start sebelum payload
ter-latch sepenuhnya tersalur — bukan bug, sama seperti `qr_ex/qr_ey` yang juga `nan` sebelum
deteksi pertama).

**P0-2.2a CLOSED.** Gap instrumentasi §4 (versi sebelumnya) tertutup; `recorder_qr.py` kini
punya semua sinyal yang didaftarkan sebagai "Data dibutuhkan" di tabel §3.

## 5. P0-2.2b — Rancangan run battery (dispesifikasi, belum dijalankan)

- **N = 4–6 run**, world `kki_arena`, mengikuti pola regresi P0-1e (`run_mission.sh` sebagai
  presedan): beberapa run `rov_random_spawn:=true` + minimal satu run deterministik
  (`rov_x/y/z` tetap), masing-masing ≥60 s atau sampai `ABORT`/`WAIT_TRIGGER`.
- **Gerbang kontaminasi**: jalankan `gate_mission.sh` tanpa modifikasi sebelum
  menginterpretasi run mana pun (sama seperti P0-1e/P0-2.1).
- **Script analisis offline** `reduce_approach_qr.py` (dinamai & dirancang, **belum
  ditulis**): input = CSV per run (dari `recorder_qr.py` + field `payload_pose` di §4); output
  = tabel enam-gate di §3 terisi per run + ringkasan repeatability agregat. Berperan sama
  seperti `reduce_mission.py` untuk P0-1e.

---

## 6. Dua prasyarat kecil sebelum eksekusi (ditemukan saat implementasi P0-2.2b)

Membaca ulang `_st_approach_qr()` (L505-618) dan `_goto_xy()` (L359-382) untuk implementasi
reducer menemukan dua hal yang mengubah cara reducer harus dibangun:

1. **Sistem sudah mencatat keputusan exit-path dalam teks log.** L597-600 mencatat
   `'QR terpusat (%s) -> GRAB (%s)'` (`visual servo` vs `jarak XY`) tepat di exit QR-scored,
   dan L612-614 mencatat `'Wall %s dipilih (+15) [urutan ke-%d] ...'` di exit fallback
   ground-truth (hanya tercapai kalau `_wall_scored` masih `False`). Kedua baris ini sudah
   masuk `$TAG.log` (stdout node di-redirect ke sana, sama seperti baris `[FSM] A -> B` yang
   dipakai P0-2.1). Ini sinyal Gate-5 yang jauh lebih langsung daripada menebak dari
   freshness `qr_result` (rencana awal di §3) — ini keputusan FSM sendiri, bukan inferensi.
2. **Damping `_goto_xy()` membaca `self.vx`/`self.vy` dari `twist.twist.linear` odom
   (L400-401), bukan dari finite-difference posisi.** `recorder_qr.py` (P0-2.1/P0-2.2a) belum
   merekam kecepatan twist sama sekali. Gate 3 butuh mereproduksi formula `_goto_xy()` persis
   secara offline — kalau reproduksi memakai sumber kecepatan berbeda dari yang benar-benar
   dipakai controller, selisih terhadap `cmd_fx/fy` aktual jadi artefak model, bukan bukti
   kontribusi QR.

Keduanya ditambal sebagai perluasan kecil, subscribe-only, sebelum battery dijalankan:
`recorder_qr.py` menambah kolom `vx, vy` (dari message `Odometry` yang sudah di-subscribe);
`run_approach_qr_smoke.sh` menambah `$TAG.gate.txt` (PASS/FAIL machine-readable) dan
`ros2 param dump /mission_fsm` ke `$TAG.params.yaml` (disiplin P0-1: "cek runtime, bukan
hanya source"). Tidak ada perubahan pada `mission_fsm.py`/`qr_detector.py`/`qr_logic.py`.

## 7. P0-2.2b — Hasil eksekusi battery (EVIDENCE, bukan verdict)

**PENTING: laporan di bawah bukan PASS/FAIL untuk `APPROACH_QR`. Transisi ke `GRAB` bukan
bukti keberhasilan QR — itu justru hipotesis yang sedang diuji.** Semua angka bersumber dari
`tools/p0-experiments/reduce_approach_qr.py`, dapat diaudit ulang dari CSV mentah.

Dieksekusi: `run_approach_qr_battery.sh`, 6 run (`Q1`-`Q5` `rov_random_spawn:=true`, `Q6`
deterministik `rov_x:=0 rov_y:=0 rov_z:=-0.5`), `kki_arena`, `P0_DATA_DIR=/tmp/p0-2-2b-battery`
(data mentah tidak masuk git, sama seperti pola P0-1/P0-2.1/P0-2.2a). **6/6 run `gate.txt=PASS`,
0 `INCONCLUSIVE`** — semua run lolos kualitas (recorder start sebelum `APPROACH_QR`,
`payload_pose` terekam, recorder selesai penuh).

### Gate 5 — jalur exit yang benar-benar terjadi (dari log FSM, bukan inferensi)

| Run | Jalur exit |
|---|---|
| Q1 | `QR_SCORED_XY_TOL` (QR terbaca, tapi trigger exit lewat jarak XY, bukan visual centering) |
| Q2 | `QR_SCORED_XY_TOL` |
| Q3 | `QR_SCORED_XY_TOL` |
| Q4 | `GROUND_TRUTH_FALLBACK` (QR tidak pernah ter-score, exit murni jarak XY ke ground-truth) |
| Q5 | `QR_SCORED_XY_TOL` |
| Q6 | `QR_SCORED_VISUAL_SERVO` (satu-satunya run yang exit lewat visual centering asli) |

Agregat: **5/6 run QR ter-score** (letter A-D berhasil dibaca sebelum exit), **1/6 run
ground-truth-fallback murni** (Q4 — QR tidak pernah terbaca dalam jendela `t_scan`). Dari 5
run yang QR-scored, **hanya 1/6 (Q6) yang benar-benar exit lewat visual servo/centering** —
4/6 QR-scored exit lewat kondisi jarak XY (`dist < approach_tol`), bukan `centered`. Artinya
QR *berkontribusi* (memilih wall) di 5/6 run, tapi presisi posisi akhir di sebagian besar run
tetap datang dari konvergensi XY ke `payload_pose`, bukan dari visual centering murni.

### Gate 4 — konvergensi error QR

**0/6 run** memasuki band `qr_center_tol=0.12` (bahkan run Q6 yang exit lewat visual servo
tidak pernah masuk band ini pada baris yang terekam — servo mungkin baru presisi tepat di
baris terakhir sebelum `GRAB`, di luar sampling 0.1s recorder, atau exit terjadi tepat saat
`centered` baru terpenuhi). Tren net-decrease `|qr_ex|`/`|qr_ey-ey_target|` **campur** —
sebagian run menurun (Q1, Q3-ex, Q4-ey), sebagian lain naik (Q2, Q5, Q6) — **tidak ada pola
konvergensi yang konsisten lintas run**.

### Gate 3 — apakah command mengikuti `qr_offset`

**5/6 run**: prediksi "dengan koreksi QR" (RMSE lebih rendah) cocok lebih baik dengan
`cmd_fx/fy` aktual dibanding prediksi "ground-truth murni" pada jendela servoing (`off_fresh`
& `dist_raw<0.3`) — bukti bahwa koreksi visual-servo memang mengubah command secara terukur,
bukan sekadar teori formula. Satu run (Q2) sebaliknya (`ground-truth murni` sedikit lebih
cocok, selisih RMSE kecil ~1%). Jumlah baris jendela-servoing bervariasi 16-53 dari ~100-950
baris per run — jendela servoing (`dist_raw<0.3`) relatif sempit dibanding durasi
`APPROACH_QR` penuh.

### Gate 2 — apakah `qr_offset` mengikuti pose relatif ground-truth

Korelasi `r` bervariasi luas dan **tidak konsisten arah/tandanya** lintas run (`r(qr_size,
dist)` dari −0.83 sampai +0.83; `r(qr_ex, lateral)` dari 0.07 sampai 0.87 magnitudo).
Kemungkinan penyebab: (a) `locked_yaw` didekati dari yaw saat entry `APPROACH_QR`, bukan nilai
`_locked_yaw` internal FSM yang sebenarnya (satu-satunya asumsi metodologis dalam reducer —
lihat catatan di output JSON per run); (b) korelasi dihitung atas SELURUH episode
`APPROACH_QR`, termasuk fase sebelum QR pernah terbaca (offset masih dari deteksi awal yang
noisy/corner-only tanpa decode berhasil).

### Gate 1

Dikonfirmasi statis (audit §1.4) — QR offset memang menggeser target sebelum masuk
`_goto_xy()`, dicross-check angka konkretnya di Gate 3.

### Gate 6 — repeatability

| Metrik | Hasil |
|---|---|
| Exit path | 4/6 `QR_SCORED_XY_TOL`, 1/6 `GROUND_TRUTH_FALLBACK`, 1/6 `QR_SCORED_VISUAL_SERVO` |
| Gate 3 "koreksi-QR lebih cocok" | 5/6 run |
| Gate 4 "masuk band `qr_center_tol`" | 0/6 run |

**Tidak repeatable dalam arti "exit path selalu sama"** — tiga kategori berbeda muncul dalam 6
run. **Konsisten dalam satu hal**: hampir semua run (5/6) menunjukkan command benar-benar
dipengaruhi koreksi visual (Gate 3), tapi presisi akhir (Gate 4, band `qr_center_tol`) tidak
pernah tercapai secara ketat — exit condition yang benar-benar dipakai hampir selalu yang
longgar (`approach_tol` 0.06 m jarak XY), bukan `qr_center_tol` visual.

Data mentah: `/tmp/p0-2-2b-battery/*.csv`, `*.log`, `*.params.yaml`, `*.gate.txt`, dan output
lengkap `P0-2-2b-results.json` (tidak masuk git — reproducible via `run_approach_qr_battery.sh`
+ `reduce_approach_qr.py`).

## 8. Status: evidence lengkap, keputusan penutupan P0-2.2 BELUM diambil

Keenam gate sekarang punya angka dari data (bukan `OPEN` konseptual lagi), tapi dokumen ini
**tidak** menyimpulkan APPROACH_QR PASS atau FAIL. Temuan paling menonjol untuk didiskusikan
sebelum menutup P0-2.2: exit path tidak seragam (campuran QR-scored-XY-tol, satu
ground-truth-fallback murni, satu visual-servo asli), dan tidak ada run yang mencapai presisi
visual-centering (`qr_center_tol`) sebelum exit — exit yang benar-benar dipakai sistem hampir
selalu jalur toleransi jarak XY yang lebih longgar. Ini temuan desain untuk dicatat (sesuai
prinsip P0-1: jangan menambal sistem demi mempermudah acceptance test), bukan bug yang harus
langsung diperbaiki di pass ini.

## 9. Non-goals eksplisit yang tetap dipertahankan sepanjang P0-2.2

- Tidak ada perubahan `mission_fsm.py`/`qr_detector.py`/`qr_logic.py`.
- Tidak ada perubahan gain/target/tolerance/timeout/parameter misi apa pun.
- Tidak ada ablation (tidak pernah mematikan `qr_offset`, tidak pernah memalsukan
  `payload_pose`).
- Acceptance matrix `docs/P0-2-AUDIT.md` §2 tidak diubah oleh dokumen ini — tetap seluruhnya
  `OPEN`. Keputusan menutup P0-2.2 (dan apakah/bagaimana mendesain acceptance P0-2.3) adalah
  keputusan terpisah setelah evidence di §7 direview, bukan otomatis dari selesainya battery.

## 10. Status P0-2 (update)

```text
P0-1        CLOSED / FROZEN
P0-2.0      CLOSED
P0-2.1      CLOSED
P0-2.2      CLOSE-PARTIAL         (keputusan pengguna — docs/P0-2-2-VERDICT-OPTIONS.md §4)
P0-2.2a     CLOSED               (payload_pose instrumentation + smoke verification — §4)
P0-2.2b     CLOSED               (6/6 run valid, 0 INCONCLUSIVE — §7)
  QR influence               VERIFIED  (Gate 3, 5/6 run)
  QR precision convergence   OPEN      (Gate 4, 0/6 run — dibawa ke P0-2.3)
P0-2.3      DESIGN                (docs/P0-2-3-SPEC.md — scope: positioning/centering di
                                    titik GRAB)
P0-3 GRAB   WAIT
NAV_WALL    WAIT
TAM         DEFERRED
```

**Tidak ada PASS/FAIL untuk `APPROACH_QR` secara keseluruhan.** Keputusan CLOSE-PARTIAL
menutup klaim "QR integration/causal influence terbukti" saja; klaim presisi/konvergensi
tetap terbuka. Acceptance matrix `docs/P0-2-AUDIT.md` §2 tetap seluruhnya `OPEN`.
