# P1 Fase 1 — Hasil: tutup blocker P0

Tanggal: 2026-08-12 · Data: `/tmp/p1-fase1` (run C1, E1, E7, E13) + battery lama
P0-2.6/2.7/2.8 · Roadmap: [P1-OWNER-DECISIONS-AND-ROADMAP.md](P1-OWNER-DECISIONS-AND-ROADMAP.md) §6

**Ringkasan satu kalimat:** dua blocker lama ditutup, tiga bug baru ditemukan dan
diperbaiki, dan perbaikan itu membuka **satu blocker yang jauh lebih fundamental** —
ROV tidak bisa bergerak begitu benar-benar mencengkeram payload.

---

## 1. P0-B (NAV_WALL) — DITUTUP untuk kasus tanpa beban, tanpa perubahan kode

`STATUS.md` (2026-08-06) mencatat NAV_WALL macet di `dist≈0.26` lalu ABORT.
Diukur ulang dari battery yang sudah ada (**tidak ada run baru**, 3.647 + 2.950 baris
NAV_WALL sudah terekam):

| Battery | run | konvergen | timeout |
|---|---|---|---|
| P0-2.7 + P0-2.8 + P0-2.6 | 47 | **47** | **0** |

`d_min` median **0.197 m** (`nav_tol=0.20`), durasi median **5.4 s** dari anggaran 30 s.

Akar penyebab lama hilang sejak commit **`f9a2d84` (2026-08-07)** yang mengubah
`wall_dist` **2.30 → 2.15**. Entri blocker ditulis 2026-08-06 — sehari sebelum
perbaikan itu — dan tak pernah ditutup.

> ⚠️ **Kualifikasi wajib (lihat §4):** 47/47 ini diukur pada run di mana attach
> **tidak pernah terpicu**, jadi ROV tak pernah membawa payload. NAV_WALL terbukti
> sehat **hanya untuk kasus tanpa beban**.

**Pelajaran metodologi:** instruksi "ukur dulu, jangan ubah gain" terbukti benar.
Menala `approach_kp/kd` akan menala sesuatu yang tidak rusak.

---

## 2. P0-A (GRAB tak pernah "close") — tiga bug bertumpuk

Audit memperkirakan ini "perbaikan 1 baris". **Estimasi itu salah** — satu baris itu
hanya membuka lapisan pertama.

| # | Bug | Bukti | Perbaikan |
|---|---|---|---|
| A-1 | `_st_grab` tak pernah publish `"close"`; `pub_grip` nol `.publish()` | grep, 1 hit | publish sekali per masuk state (`_to()` mereset `_hold_since`) |
| A-2 | `gripper_controller` tak memfilter `frame_id`, padahal `qr_detector` menerbitkan **dua kamera ke topik yang sama** → gerbang dinilai dari kamera DEPAN | run A1: `ex=0.90 ey=0.75` dari `camera_front_link` saat `gripper_err=0.032 m` | filter `offset_frame` (default `camera_bottom_link`), sama dengan `mission_fsm.py:466` |
| A-3 | **Akar sesungguhnya:** `is_safe()` menuntut `\|ey\| ≤ 0.30` (acuan **pusat kamera**), `mission_fsm` membidik `ey_target ≈ −0.52` (acuan **gripper**, 0.16 m di depan kamera). Mustahil dipenuhi bersamaan | run C1: **0/34 tick GRAB lolos**, satu-satunya suku yang gagal adalah `ey` | `qr_ey_target` dipindah ke `qr_logic` sebagai satu sumber; `is_safe()` menguji `\|ey − ey_target\|` |

Dikunci 3 test regresi di `test_gripper.py`. Total **79 test lolos**.

Hasil: `gripper closed: attach (payload dalam jangkauan)` — **attach kini benar-benar
terpicu**, terbukti runtime di E1/E7/E13.

---

## 3. Siklus 4-hook end-to-end

Run **C1** (gate PASS 7/7, jendela 420 s): **3 siklus penuh berurutan tanpa satu pun
ABORT** — `DIVE → APPROACH_QR → GRAB → NAV_WALL → HANG → SURFACE → WAIT_TRIGGER →
APPROACH_HOOK → AUTO_RELEASE` ×3, siklus ke-4 terpotong batas rekaman (bukan kegagalan).
C1 berjalan dengan kode **sebelum** perbaikan A-3, jadi attach belum terpicu.

Exit criteria Fase 1 (**4-hook × 3 seed**) **BELUM tercapai** — bukan karena regresi
tooling, melainkan karena blocker §4 yang baru terungkap.

---

## 4. ⛔ BLOCKER BARU M5-D — ROV terjangkar begitu benar-benar mencengkeram

Battery E1/E7/E13 (gate PASS 3/3, `spawn_seed` 1/7/13, jendela 620 s):

| Run | attach? | NAV_WALL |
|---|---|---|
| E1 | ✅ | ABORT, macet **0.55 m** |
| E7 | ✅ | ABORT, macet **0.53 m** |
| E13 siklus 1 | ❌ | **selesai penuh** → AUTO_RELEASE |
| E13 siklus 2 | ✅ | ABORT, macet **0.50 m** |

E13 adalah **kontrol dalam-run**: run sama, seed sama, siklus tanpa attach berhasil,
siklus dengan attach gagal.

**Mekanisme, terukur:**

| Kondisi | depth setpoint | depth aktual | simpangan | \|v\| rata-rata |
|---|---|---|---|---|
| tanpa attach (E13 s1) | −0.448 | 0.429 | **−0.019** | **0.533** |
| dengan attach (E1) | −0.450 | 0.253 | −0.197 | 0.092 |
| dengan attach (E7) | −0.450 | 0.260 | −0.190 | 0.117 |
| dengan attach (E13 s2) | −0.450 | 0.275 | −0.175 | 0.041 |

Jejak E1 selama macet: posisi konstan di (0.42, −1.80) selama 22+ s, `|v|≈0.005 m/s`,
sementara FSM memerintahkan surge **8 → 16 N**. Gerbang yaw **tidak** menyebabkannya —
`yaw_err` hanya 1.7–5.4° (batas 15°), jadi gerbang terbuka dan gaya memang dikirim.
Kendaraan terkunci secara mekanis.

**Akar penyebab:** `DetachableJoint` mengelas kedua link **pada pose saat itu**; ia
**tidak** menarik payload ke gripper. Jarak vertikal ROV↔payload saat GRAB, konsisten:

```
E1  0.602 m      E7  0.601 m      E13  0.605 m
```

Angka ~0.60 m itu bukan kebetulan: `scan_depth=0.30` sengaja dibuat tinggi supaya QR
12 cm muat di frame kamera bawah (lihat komentar param `scan_depth`). Jadi ROV mengelas
dirinya ke benda yang masih tergeletak di lantai 0.6 m di bawah, lalu terjangkar.

**Akar desain: FSM tidak punya fase TURUN-UNTUK-MENCENGKERAM.** Alurnya
`APPROACH_QR` (melayang 0.6 m di atas payload) → `GRAB` (attach seketika). Tidak ada
state yang pernah membawa ROV turun ke payload.

**Perilaku lama menyembunyikan ini sepenuhnya:** misi "selesai" justru KARENA grasp tak
pernah terjadi. Ini contoh persis dari prinsip "state transition ≠ observable success".

**Belum dikerjakan — butuh keputusan desain.** Opsi yang terlihat, dengan trade-off:

| Opsi | Isi | Trade-off |
|---|---|---|
| 1 | Tambah state `DESCEND` antara APPROACH_QR dan GRAB | Paling benar secara fisik & paling dekat ke hardware; QR keluar frame saat turun → butuh dead-reckoning singkat |
| 2 | Turunkan `scan_depth` mendekati payload sebelum publish "close" | Diff kecil; tapi `scan_depth=0.30` sudah hasil dua kali revisi demi keterbacaan QR (0.62→0.46→0.30) — menurunkannya mengulang bug framing lama |
| 3 | Buat attach memindahkan payload ke gripper | Paling mudah, tapi **SIMULATION-ONLY murni** dan makin menjauhkan sim dari hardware — bertentangan dengan tujuan repo |

Rekomendasi: **Opsi 1**. Opsi 3 akan membuat angka misi hijau lagi tanpa memperbaiki
apa pun yang bisa ditransfer ke ROV asli.

---

## 5. Status evidence

| Klaim | Status |
|---|---|
| NAV_WALL konvergen 47/47 **tanpa beban** | **VERIFIED** (data battery lama) |
| `wall_dist` 2.30→2.15 di `f9a2d84` menutup blocker lama | **VERIFIED** (git + data) |
| A-1/A-2/A-3 diperbaiki, attach terpicu | **VERIFIED** (runtime E1/E7/E13) |
| Attach → ROV terjangkar → NAV_WALL ABORT | **VERIFIED** (3/3 + kontrol dalam-run E13) |
| Jarak vertikal 0.60 m saat GRAB | **VERIFIED** (3/3 run) |
| Mekanisme = DetachableJoint mengelas lintas celah | **INFERRED** — konsisten dgn semua pengukuran, belum dibuktikan langsung (mis. lewat `/tf` atau pose payload runtime) |
| Asset QR adalah bidang VERTIKAL (`normal 0 1 0`) sedangkan `qr_ey_target` memodelkan QR mendatar | **INFERRED** — perlu verifikasi; ukuran-tampak terukur menyimpang dari prediksi (rasio 0.77× dan 2.17×) |
| Siklus 4-hook × 3 seed | **BELUM** — diblokir M5-D |

## 6. Catatan kontaminasi

Battery pertama dijalankan saat run C1 masih hidup → dua server Gazebo berebut topik.
Gate menangkapnya (`gz-servers=2`, `cmd_vel pub=2`) dan menandai run CONTAMINATED;
data itu **dibuang, tidak masuk dokumen ini**. `run_mission_cycle.sh` kini menolak start
bila masih ada Gazebo/node hidup (pre-run guard), sehingga penyebabnya tak bisa terulang.
