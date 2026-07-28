# Konfigurasi & Alokasi 6 Thruster — HYDROships

Frame body mengikuti REP-103: **x maju (surge), y kiri (sway), z atas (heave)**.
Rotasi: roll (Mx, sekitar x), pitch (My, sekitar y), yaw (Mz, sekitar z).

Geometri ini adalah sumber kebenaran tunggal dan **harus identik** di tiga tempat
(konsistensi masih manual/duplikat — lihat catatan):

- `hydroships_description/urdf/hydroships.urdf.xacro` (model primitif box)
- `hydroships_description/urdf/rov_kki2026_new_design.urdf.xacro` (model mesh CAD)
- `hydroships_control/allocation.py` (konstanta `THRUSTERS`, modul murni yang dipakai
  node `thruster_allocator`)

## Konvensi posisi & sumber data

**Diperbarui 2026-07-28 — sumber data berganti dari CSV ke CAD.**

Posisi & sumbu kini **diukur langsung dari mesh CAD**
`DOKUMENTASI ROV/@ROV KKI 2026 NEW DESIGN.stl` memakai
`hydroships_description/scripts/measure_cad_frames.py`:

```bash
python3 src/hydroships_description/scripts/measure_cad_frames.py --stl "DOKUMENTASI ROV/@ROV KKI 2026 NEW DESIGN.stl" --verbose
```

Tiap unit dikenali lewat *connected-component analysis*: rumah thruster = bongkah
kompak 45–110 mm, arah dorong = normal cakram propeller/duct terdekat. Jumlahnya
cocok dengan `.f3z`: 4× T100 + 2× T200.

Origin STL bukan konvensi ROS — sumbu memanjang ROV ada di Y mesh dan haluan di
−Y mesh, jadi mesh diputar **+90° terhadap Z** lalu digeser sehingga `base_link`
berimpit dengan pusat massa:

    x_base = -y_mesh    y_base = x_mesh    z_base = z_mesh

**Penomoran** mengikuti Gambar 2.9/2.10 dokumen desain (penulis, 2026).

> ⚠️ **Koreksi peran (2026-07-28).** Mapping lama menukar peran pasangan haluan ↔
> pasangan tengah (dulu `#1,#2` dianggap vertikal dan `#3,#4` surge) serta menukar
> `#5` ↔ `#6`. CAD dan Gambar 2.9 keduanya menunjukkan sebaliknya: pasangan haluan
> digambar sebagai lingkaran (propeller dilihat searah sumbu → **vertikal**),
> pasangan tengah & unit tengah-bawah digambar profil samping → **horizontal**.
> Verifikasi geometris: dengan angka lama, `#1` dan `#3` berjarak 89 mm dan 108 mm
> dari permukaan mesh (menggantung di ruang kosong); setelah dikoreksi keenam titik
> berjarak 3,4–8,5 mm.

> ⚠️ **Bug historis (`14cf649`):** kolom CSV sempat disalin **mentah** `(X,Y,Z)→(x,y,z)`
> tanpa rotasi frame → posisi terputar 90°, `cond(TAM)≈1.2e4`. `docs/thruster_positions.csv`
> kini **hanya arsip**; jangan dipakai lagi sebagai sumber.

## Tabel thruster (sesuai `allocation.py`)

Urutan `thruster_1..6` = urutan `THRUSTERS` di `allocation.py`.

| # | Peran | Kelas | Posisi body (x, y, z) [m] | Arah dorong | Duct terukur | Putaran |
|---|-------|-------|---------------------------|-------------|--------------|---------|
| 1 | Surge (kanan) | T200 | (-0.0291, -0.1188, 0.0186) | (1, 0, 0) | duct 90,1 mm | CCW |
| 2 | Surge (kiri) | T200 | (-0.0287, 0.1274, 0.0190) | (1, 0, 0) | duct 90,1 mm | CW |
| 3 | Heave (haluan-kanan) | T100 | (0.1050, -0.1332, 0.0592) | (0, 0, 1) | prop 66,6 mm | CCW |
| 4 | Heave (haluan-kiri) | T100 | (0.1053, 0.1411, 0.0550) | (0, 0, 1) | prop 66,6 mm | CW |
| 5 | Heave (buritan) | T100 | (-0.1610, 0.0037, 0.0621) | (0, 0, 1) | prop 66,6 mm | CW |
| 6 | Sway (tengah-bawah) | T100 | (-0.0700, 0.0200, -0.0939) | (0, 1, 0) | prop 59,0 mm | CCW |

- **Horizontal:** `#1`, `#2` menghasilkan **surge** (dan yaw dari selisih kiri-kanan);
  `#6` menghasilkan **sway**.
- **Vertikal:** `#3`, `#4`, `#5` (dorong +z) menghasilkan **heave**, plus **roll/pitch**
  dari penempatan tak segaris.
- **Putaran** (Gambar 2.9): tiap pasangan berlawanan arah agar torsi reaksi saling
  meniadakan. Nilai ini dokumentatif (`SPIN` di `allocation.py`), tidak masuk TAM.
- **Diameter propeller** di plugin gz kini mengikuti kelas: T200 → 0,076 m,
  T100 → 0,048 m. Sebelumnya pembagian mengikuti nomor urut, bukan kelas.

### ⚠️ Tanda sumbu belum diverifikasi bench

Gambar 2.10 menggambar F1/F2 (surge) menunjuk ke **buritan** dan F6 (sway) ke **kiri**.
Arah F6 dipakai apa adanya (+y). Untuk surge dipakai **+x (maju)** mengikuti REP-103,
*bukan* arah panah gambar, karena panah untuk thruster vertikal (F3/F4/F5) jelas
artefak gambar — arah gaya thruster vertikal tidak bisa digambar di tampak atas —
sehingga gambar itu tidak konsisten dengan dirinya sendiri. **Uji bench:** beri
perintah positif ke `thruster_1`; kalau ROV terdorong mundur, balik tanda sumbu di
`allocation.py` **dan** kedua URDF.

### Dampak koreksi terhadap kendali

| | lama | baru |
|---|---|---|
| `cond(TAM)` | 19,7 | **9,98** |
| singular value terkecil | 0,088 | **0,174** |
| gaya thruster utk wrench satuan, DOF terlemah | 9,25 N | **4,06 N** |
| pitch (My) tercapai, damped pinv | 44 % | **82 %** |
| roll (Mx) | 75 % | 79 % |
| yaw (Mz) | 79 % | 75 % |

Geometri terukur ini **lebih mudah dikendalikan** daripada angka desain lama.

## Thrust Allocation Matrix (TAM)

Kolom ke-*i* dari TAM (6×N) adalah kontribusi thruster *i* ke wrench body:

```
kolom_i = [ axis_i ; pos_i × axis_i ]      (3 gaya + 3 torsi)
wrench  = TAM · f                          (f = vektor gaya thruster, N)
```

Alokasi memakai **pseudo-inverse teredam (damped least-squares / Tikhonov)**, bukan pinv
polos (`allocation.py: build_damped_pinv`):

```
f = pinv_damped(TAM) · wrench
pinv_damped = TAMᵀ (TAM·TAMᵀ + damping²·I)⁻¹        (param alloc_damping, default 0.1)
```

**Alasan redaman:** meski setelah frame-fix TAM sudah rank-6, bidang horizontal tetap
relatif lemah pada **yaw**. Dengan pinv polos, perintah pada arah lemah menuntut gaya
thruster raksasa (ribuan N) yang menjenuhkan batas lalu **merusak DOF lain** setelah di-clip.
Redaman membuat perintah tak-tercapai "menyerah anggun" (→ nol) sementara arah sehat
(heave/sway/surge) tetap terlayani. `alloc_damping → 0` mengembalikan pinv biasa. Node
`thruster_allocator` memberi peringatan bila `cond(TAM) > 100`.

Batas gaya per thruster: **-40 N … +50 N** (`MIN_THRUST`/`MAX_THRUST` di `allocation.py`,
konsisten dgn `max/min_thrust_cmd` plugin Thruster di URDF). Allocator juga punya
**watchdog**: bila perintah `/hydroships/cmd_vel` berhenti > 0,5 s, thruster dinolkan.

## Catatan penyetelan (menyusul)

- Konsistensi posisi/axis URDF ↔ `allocation.py` masih **manual (duplikat)**. Opsi lanjut:
  satu sumber-kebenaran parametrik atau test konsistensi otomatis (belum wajib).
- Nilai geometri di atas adalah desain simulasi; sesuaikan dengan rangka final bila berubah.
