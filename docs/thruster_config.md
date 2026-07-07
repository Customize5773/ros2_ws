# Konfigurasi & Alokasi 6 Thruster — HYDROships

Frame body mengikuti REP-103: **x maju (surge), y kiri (sway), z atas (heave)**.
Rotasi: roll (Mx, sekitar x), pitch (My, sekitar y), yaw (Mz, sekitar z).

Geometri ini adalah sumber kebenaran tunggal dan **harus identik** di dua tempat:

- `hydroships_description/urdf/hydroships.urdf.xacro` (posisi/sumbu joint thruster)
- `hydroships_control/thruster_allocator.py` (konstanta `THRUSTERS`)

## Tabel thruster

| # | Peran | Posisi (x, y, z) [m] | Arah dorong (unit) | DOF utama |
|---|-------|----------------------|--------------------|-----------|
| 1 | Horizontal | (0.000, 0.160, 0.000)  | (-1.000, 0.000, 0.000) | surge/sway/yaw |
| 2 | Horizontal | (-0.1386, -0.080, 0.000) | (0.500, -0.866, 0.000) | surge/sway/yaw |
| 3 | Horizontal | (0.1386, -0.080, 0.000)  | (0.500, 0.866, 0.000) | surge/sway/yaw |
| 4 | Vertikal | (0.120, 0.000, 0.000)   | (0, 0, 1) | heave/pitch |
| 5 | Vertikal | (-0.100, 0.120, 0.000)  | (0, 0, 1) | heave/roll |
| 6 | Vertikal | (-0.100, -0.120, 0.000) | (0, 0, 1) | heave/roll |

**Horizontal (1–3):** susunan tangensial 120° pada radius rh = 0,16 m
(sudut posisi 90°, 210°, 330°; arah dorong = tangen lingkaran). Tiga vektor gaya
di bidang xy ini merentang penuh sehingga bisa menghasilkan gaya arah apa saja +
torsi yaw → mengontrol **surge, sway, yaw**.

**Vertikal (4–6):** dorong searah +z, ditempatkan tidak segaris (satu depan-tengah,
dua belakang kiri/kanan) → mengontrol **heave, roll, pitch**.

## Thrust Allocation Matrix (TAM)

Kolom ke-*i* dari TAM (6×6) adalah kontribusi thruster *i* ke wrench body:

```
kolom_i = [ axis_i ; pos_i × axis_i ]      (3 gaya + 3 torsi)
wrench  = TAM · f                          (f = vektor gaya 6 thruster, N)
```

Alokasi memakai **pseudo-inverse**: `f = pinv(TAM) · wrench`. Karena horizontal
mengurus (Fx, Fy, Mz) dan vertikal mengurus (Fz, Mx, My), TAM full-rank (rank 6)
dan alokasi terdefinisi baik.

Batas gaya per thruster: **-40 N … +50 N** (≈ kelas T100/T200), diterapkan di
`thruster_allocator.py` dan `max/min_thrust_cmd` plugin Thruster di URDF.

## Catatan penyetelan (Milestone berikutnya)

- Nilai posisi/arah di atas adalah desain awal simulasi; sesuaikan dengan
  geometri rangka final dari desain Fusion.
- M2 menambahkan PID depth-hold & heading-hold yang menulis wrench ke
  `/hydroships/cmd_vel` menggantikan teleop manual.
