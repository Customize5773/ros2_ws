# P0-1 BASELINE — Investigasi & Koreksi DIVE (KKI 2026)

Dokumen ini **membekukan** hasil investigasi P0-1a → P0-1e: dari `DIVE timeout` yang
konsisten sampai DIVE lolos 4/4 run. Ini adalah *source of truth* untuk keadaan fisik &
integrasi ROV per **2026-08-08**, dan menjadi titik mulai P0-2 (APPROACH_QR).

Tag git: **`p0-1-baseline`**.
Commit perbaikan: `9219735` (collision gripper) · `8d6c49c` (`cob.x`) · `0941cd4` (world-name).

Label evidence dipakai konsisten:
`OBSERVED` terlihat langsung · `MEASURED` punya nilai numerik runtime ·
`INFERRED` kesimpulan dari evidence · `UNKNOWN` data tak ada · `BLOCKED` tak bisa diuji.

---

## ⚠️ Dua batasan yang TIDAK boleh dihilangkan dari ringkasan mana pun

1. **TAM belum terbukti benar.** Status **`DEFERRED`, bukan `VERIFIED`.** P0-1 hanya menunjukkan
   kopling Fz→My *bukan blocker pada titik operasi DIVE yang diuji*. Geometri thruster vertikal
   tetap menghasilkan momen pitch parasit; besarnya belum diuji di seluruh rentang thrust.
2. **"Tidak ada regresi integrasi" berlabel `INFERRED`, bukan proof formal.** Basisnya empat
   regression run (3 random spawn + 1 deterministik), bukan pembuktian menyeluruh.

Tambahan: **APPROACH_QR / GRAB / NAV_WALL BELUM dikarakterisasi.** Dalam run P0-1e, FSM memang
berlanjut sampai `WAIT_TRIGGER` tanpa ABORT dalam jendela 60 s — tetapi *"berjalan tanpa ABORT"
bukan acceptance evidence*. Status ketiganya: **menunggu P0-2/P0-3/P0-4**.

---

## 1. Ringkasan rantai investigasi

| Tahap | Pertanyaan | Hasil |
|---|---|---|
| **P0-1a** | Di mana performance DIVE hilang? | Controller `OK`, allocator-heave `OK` (fidelity 98%), pitch allocation `SUSPECT`. Dugaan awal "aktuator hanya menyalurkan ~10%" — kemudian **SUPERSEDED**. |
| **P0-1b** | Apakah thrust yang diperintah menghasilkan respons fisik? | Gaya **benar-benar sampai** ke wahana. Dugaan actuator-loss gugur. Data terkontaminasi trim & permukaan. |
| **P0-1c.1** | Mengapa kesetimbangan statis miring −24.5°? | **Akar ketemu:** `<collision>` pada 3 link gripper ikut dihitung sebagai volume apung. |
| **P0-1c.2/.3** | Koreksi model | Collision gripper dihapus; `cob.x` disejajarkan dgn CoG sistem. |
| **P0-1c.4** | Infrastruktur uji | `pool_empty` tak pernah spawn ROV — nama world diambil dari nama file, bukan isi SDF. |
| **P0-1d** | Setelah trim benar, apakah kopling Fz→My masih menghambat? | **PASS** — tidak menghambat pada titik operasi DIVE. |
| **P0-1e** | Apakah rantai tertutup mereproduksi otoritas itu? | **PASS 4/4** — DIVE lolos, tanpa timeout. |

## 2. Akar penyebab & koreksinya

Plugin `gz-sim-buoyancy-system` menghitung volume perpindahan air dari geometri `<collision>`.
`gripper_base` (box 0.10×0.10×0.06 @ x=+0.18 m) dan kedua `gripper_finger_*` punya collision,
sehingga ikut menghasilkan gaya apung yang **tidak pernah masuk neraca** di `rov_params.yaml` —
neraca itu hanya menghitung box `base_link`. **[MEASURED, dari SDF yang benar-benar di-spawn]**

| Besaran | Sebelum | Setelah koreksi | Target desain |
|---|---:|---:|---:|
| Volume terpindah | 0.009406 m³ | **0.008729 m³** | 0.008729 |
| Gaya apung | 92.27 N | **85.63 N** | 85.62 |
| **Net apung** | **+6.92 N** | **+0.28 N** | +0.28 |
| CoB | (13.6, 0, 18.6) mm | **(2.37, 0, 20) mm** | — |
| Δx (CoB−CoG) | +11.23 mm | **−0.00 mm** | 0 |
| Momen trim | 1.036 N·m bow-up | **0.0003 N·m** | 0 |
| Trim pasif prediksi | 31.5° | **0.0°** | 0° |

Rantai sebabnya: apung berlebih di haluan → CoB bergeser ke depan → momen bow-up 1.04 N·m melawan
momen pemulih maks 1.69 N·m → ROV trim ~31° → sumbu dorong "vertikal" ikut miring → perintah heave
terproyeksi jadi gerak horizontal. **[INFERRED, terkonfirmasi runtime]**

Verifikasi runtime bertahap (sikap kesetimbangan pasif, terendam, rig open-loop):

```
sebelum koreksi   prediksi 31.5° bow-up   → terukur −29.6° / −44.1°
setelah Perubahan 1  prediksi  6.9° bow-down → terukur  +4.44° / +3.58°
setelah Perubahan 2  prediksi  0.0°          → terukur  −0.02° / −0.01°
```

Laju naik bebas terukur **+0.0346 m/s** vs prediksi **+0.0347 m/s** untuk net +0.28 N — selisih
0.3%, memverifikasi besaran gaya, bukan hanya sikap. **[MEASURED]**

## 3. Regresi DIVE — sebelum vs sesudah

| | P0-1a (pra-koreksi) | P0-1e (baseline ini) |
|---|---|---|
| Hasil DIVE | **timeout 20 s → ABORT**, 3/3 run | **lolos**, 4/4 run |
| Waktu ke ambang 0.24 m | tak pernah tercapai | **1.65–1.76 s** (anggaran 20 s) |
| Kedalaman tercapai | mentok ~0.215 m | melewati ambang, lanjut ke setpoint −0.30 m |
| Pitch selama DIVE | divergen −13.6° → −33° | **maks 0.30°** |
| Roll selama DIVE | −7.8° | **maks 0.19°** |
| `cmd_vel.linear.z` | −7.6…−14.0 N, tumbuh terus | −5.25…+2.09 N, stabil, **0% saturasi** |
| Perintah thrust puncak | 16–19 N | **2.44 N** (batas +50/−40) |
| Fidelity allocator | 98% | **99.4%** |
| Nasib wahana | menabrak `hook_c`, x membeku 2.197 m | tanpa kontak |

**Tidak ada satu pun parameter kendali yang diubah sejak P0-1a** — tidak PID, tidak allocator
damping, tidak TAM, tidak timeout. Controller yang sama kini butuh ~6× lebih sedikit gaya untuk
menyelam ~11× lebih cepat. **[MEASURED]**

Karakterisasi open-loop pendukung (P0-1d, `pool_empty`, tanpa controller sama sekali): ambang
0.24 m tercapai dalam **0.55–0.57 s**, repeatable (E1 vs E3 berbeda 0.01 s), pitch maks 11.34°.
Split damped-pinv yang dipakai allocator memangkas momen parasit **42%** dan laju pitch **56×**
dibanding split gaya-sama. **[MEASURED]**

## 4. Yang TIDAK terbukti (jangan diisi dengan ekstrapolasi)

| Item | Status | Catatan |
|---|---|---|
| TAM / geometri thruster benar | **`DEFERRED`** | hanya terbukti bukan blocker DIVE |
| Kopling Fz→My pada −10 N / −14 N (B, B′) | **`INCONCLUSIVE`** | jendela bersih terlalu pendek, wahana mencapai lantai |
| Kontribusi individual T2 & T6 | **`INCONCLUSIVE`** | segmen dimulai dari kontak lantai |
| Skala thrust absolut (η) | **`UNKNOWN`** | butuh titik operasi level + terendam penuh |
| APPROACH_QR / GRAB / NAV_WALL | **`OPEN`** | belum dikarakterisasi; observasi 60 s bukan acceptance |
| Kalibrasi ke ROV fisik | **`OPEN`** | seluruh parameter fisik masih `[estimate]` |

## 5. Reproduksi

Pembagian tujuan world dipegang ketat: **`pool_empty` = fisik/open-loop, `kki_arena` =
integrasi/misi.** Jangan dicampur.

P0-1d — karakterisasi open-loop (tanpa stabilizer/allocator/FSM):

```bash
ros2 launch hydroships_gazebo sim.launch.py headless:=true world:=pool_empty.sdf \
    rov_random_spawn:=false rov_x:=0.0 rov_y:=0.0 rov_z:=-2.5
python3 tools/p0-experiments/driver.py out.csv B_pinv     # lalu reduce_openloop.py
```

P0-1e — regresi DIVE tertutup (stack penuh):

```bash
bash tools/p0-experiments/run_mission.sh R1                       # random spawn
bash tools/p0-experiments/run_mission.sh R4 rov_random_spawn:=false \
     rov_x:=0.0 rov_y:=0.0 rov_z:=-0.5                            # deterministik
python3 tools/p0-experiments/reduce_mission.py
```

Audit trim statis (tanpa sim): `python3 tools/p0-experiments/trim_audit.py`.

**Gate anti-kontaminasi wajib** sebelum tiap run — run yang gagal gate ditandai
`CONTAMINATED` dan **tidak diinterpretasi**, bukan ditafsirkan paksa:
tepat satu server Gazebo · `/hydroships/odom` terbit · komposisi node sesuai jenis rig
(open-loop: tanpa stabilizer/allocator/FSM; integrasi: ketiganya hadir).
Kontak lantai `kki_arena` di `z ≈ −0.809 m`, `pool_empty` di `z ≈ −4.829 m`; data setelah
kontak dibuang.

## 6. Pelajaran metodologi yang layak dipertahankan

- **`twist` odom pernah bertentangan dgn turunan posisi sampai faktor 13×.** Turunkan kecepatan
  dari **posisi**, pakai `twist` hanya sebagai pembanding silang.
- **Source ≠ runtime.** Nilai parameter, lumping link, dan geometri collision harus dicek dari
  SDF yang benar-benar di-spawn, bukan dari URDF/YAML saja.
- **Kegagalan sunyi mahal.** Spawn `pool_empty` gagal tanpa error selama berminggu-minggu; satu
  sesi karakterisasi hilang karenanya. Sejak `0941cd4`, kode-keluar `create` dilaporkan.
- **Kontaminasi kontak merusak interpretasi.** Diagnosis P0-1a sempat menyimpulkan defisit
  aktuator ~10% dari data yang ternyata terkontaminasi tabrakan dinding.
