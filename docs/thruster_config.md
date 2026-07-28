# Konfigurasi & Alokasi 6 Thruster — HYDROships

Frame body mengikuti REP-103: **x maju (surge), y kiri (sway), z atas (heave)**.
Rotasi: roll (Mx, sekitar x), pitch (My, sekitar y), yaw (Mz, sekitar z).

Geometri ini adalah sumber kebenaran tunggal dan **harus identik** di dua tempat
(konsistensi masih manual/duplikat — lihat catatan):

- `hydroships_description/urdf/hydroships.urdf.xacro` (posisi/sumbu joint thruster)
- `hydroships_control/allocation.py` (konstanta `THRUSTERS`, modul murni yang dipakai
  node `thruster_allocator`)

## Konvensi posisi & sumber data

Posisi diturunkan dari `docs/thruster_positions.csv` yang berkonvensi **berbeda** dari
frame body ROS:

- **CSV:** `X = lateral (kanan +)`, `Y = fore/aft (DEPAN negatif)`, `Z = atas`, satuan **mm**.
- **Konversi ke body ROS:** `x_body = -Y_csv`, `y_body = -X_csv`, `z_body = Z_csv` (mm→m).

> ⚠️ **Bug historis (sudah diperbaiki, `14cf649`):** kolom CSV sempat disalin **mentah**
> `(X,Y,Z)→(x,y,z)` tanpa rotasi frame → posisi terputar 90°, momen yaw T100-A/C saling
> meniadakan, `cond(TAM)≈1.2e4` (yaw near-singular). Setelah konversi di atas: `cond≈20`,
> yaw pulih. Lihat [CHANGELOG.md](CHANGELOG.md).

## Tabel thruster (sesuai `allocation.py`)

Urutan `thruster_1..6` = urutan `THRUSTERS` di `allocation.py`. Kolom "CSV" = nilai mentah
`(X,Y,Z)_mm` sebelum konversi.

| # | Label | Peran | Posisi body (x, y, z) [m] | Arah dorong (unit) | CSV (X,Y,Z) [mm] |
|---|-------|-------|---------------------------|--------------------|------------------|
| 1 | T200-E | Vertikal (kanan)     | (-0.0275, -0.1234, 0.0142) | (0, 0, 1) | (123.4, 27.5, 14.2) |
| 2 | T200-F | Vertikal (kiri)      | (-0.0290, 0.1228, 0.0148)  | (0, 0, 1) | (-122.8, 29.0, 14.8) |
| 3 | T100-C | Surge (depan-kanan)  | (0.1298, -0.1371, 0.0336)  | (1, 0, 0) | (137.1, -129.8, 33.6) |
| 4 | T100-A | Surge (depan-kiri)   | (0.1296, 0.1371, 0.0374)   | (1, 0, 0) | (-137.1, -129.6, 37.4) |
| 5 | T200-B | Sway (tengah-bawah)  | (-0.0455, -0.0003, -0.0994) | (0, 1, 0) | (0.3, 45.5, -99.4) |
| 6 | T100-D | Vertikal (belakang)  | (-0.1364, 0.0003, 0.0403)  | (0, 0, 1) | (-0.3, 136.4, 40.3) |

- **Horizontal:** `#3`, `#4` menghasilkan **surge** (dan yaw dari selisih kiri-kanan);
  `#5` menghasilkan **sway**.
- **Vertikal:** `#1`, `#2`, `#6` (dorong +z) menghasilkan **heave**, plus **roll/pitch**
  dari penempatan tak segaris.

> Catatan label: thruster `#5` dulu dilabeli **T100-B**, kini **T200-B** — tetap thruster
> sway horizontal. (`docs/thruster_positions.csv` masih menuliskan label T100-B pada baris
> tersebut; nilai posisinya tetap benar.) Peran/axis sudah dikonfirmasi via anotasi arah gaya.

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
