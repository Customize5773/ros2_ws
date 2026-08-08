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

## 6. Non-goals eksplisit untuk pass ini

- Tidak ada kode ditulis (`recorder_qr.py` tidak diedit, `reduce_approach_qr.py` tidak
  dibuat).
- Tidak ada `ros2 launch` / run simulator.
- Tidak ada perubahan gain/target/tolerance/timeout.
- Acceptance matrix `docs/P0-2-AUDIT.md` §2 tidak diubah — tetap seluruhnya `OPEN`.

## 7. Status P0-2 (update)

```text
P0-1        CLOSED / FROZEN
P0-2.0      CLOSED
P0-2.1      CLOSED
P0-2.2      DESIGN COMPLETE
P0-2.2a     CLOSED    (payload_pose instrumentation + smoke verification — §4)
P0-2.2b     NEXT      (N=4-6 execution battery + reducer — §5, belum dijalankan)
P0-2.3      WAIT
P0-3 GRAB   WAIT
NAV_WALL    WAIT
TAM         DEFERRED
```

**Tidak ada PASS/FAIL untuk `APPROACH_QR` secara keseluruhan sampai keenam gate di §3 benar-benar
dievaluasi dari data P0-2.2b.** Acceptance matrix `docs/P0-2-AUDIT.md` §2 tetap seluruhnya
`OPEN`.
