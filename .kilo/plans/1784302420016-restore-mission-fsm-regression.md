# Plan: Mount Gripper Body on Front Face of ROV (+X forward)

## Konfirmasi Status Saat Ini

- **FSM integration sudah dipulihkan** di commit `6422d2e` ("fix(mission): restore gripper integration and hook visual servo PD").
  - `_grip()`, `hook_offset` subscription, `hook_logic`, `APPROACH_HOOK` visual servo PD, dan `done_hooks` tracking sudah kembali ke `mission_fsm.py`.
- **Body gripper sudah diperbesar** di URDF, tetapi masih dipasang di perut bawah ROV (`gripper_base_joint` di `0.08 0 -0.22`).
- **Tujuan:** Pindah mount gripper ke **muka depan ROV** (+X), menjorok ke depan, sesuai referensi desain `Gambar ROV/Right-View.jpeg`.

## File yang Harus Diperbaiki

### 1. `src/hydroships_description/urdf/hydroships.urdf.xacro` — Pindah mount ke depan

**Perubahan yang dibutuhkan:**
- **`gripper_base` link:** Ubah bentuk menjadi lebih cocok untuk front-mount (mis. `box size="0.10 0.10 0.06"` — 10 cm maju, 10 cm lebar, 6 cm tinggi).
- **Posisi `gripper_base_joint`:** Ubah `xyz` menjadi `0.18 0 0` (di muka depan ROV, sedikit ke depan dari `camera_front_link` di `body_x/2 ≈ 0.1725`).
- **Massa:** Tetap `grip_mass = 0.12` kg (sesuai volume baru).
- **Visual:** Material kuning kontras (`rgba="0.85 0.7 0.1 1.0"`) agar terlihat jelas di Gazebo.
- **`gripper_jaw` link:** Ubah menjadi menjorok ke depan (+X):
  - Joint origin relatif ke base: `0.05 0 0` (tepat di depan face base).
  - Box size: `0.12 0.016 0.04` (jari 12 cm ke depan, tipis, tinggi 4 cm).
  - Axis tetap `0 1 0` (revolute around Y → jaw membuka/menutup secara vertikal di bidang XZ).
- **Collision & inertia:** Sesuaikan dengan geometri baru.
- **Catatan:** Jangan ubah joint tipe (`fixed` untuk base, `revolute` untuk jaw) atau plugin DetachableJoint — mekanisme ROS/sim harus tetap sama.

### 2. `src/hydroships_control/hydroships_control/mission_fsm.py` — Konsistenkan mention gripper

- Pastikan semua elemen gripper + hook tetap ada (sudah terverifikasi di `6422d2e`):
  - `_grip()`, `_on_hook()`, `_hook_fresh()`, `pub_grip`, `hook_offset` subscription.
  - State handlers `_st_grab`, `_st_approach_hook`, `_st_auto_release` menggunakan gripper/hook logic.
  - `done_hooks` tracking.
- Docstring: konsistenkan aliran state (APPROACH_QR, bukan SCAN_QR di jalur default).

### 3. `docs/STATUS.md`
- Update M5: body gripper **dipasang di muka depan ROV** (joint `0.18 0 0`, body 0.10×0.10×0.06 m).

### 4. `docs/CHANGELOG.md`
- Tambah entri: pindah mount gripper dari perut bawah ke muka depan ROV.

### 5. `docs/HOW-TO-RUN.txt`
- Update langkah verifikasi visual: body gripper di muka depan, jari menjorok ke depan (+X).

## Strategi Perbaikan

1. Edit `hydroships.urdf.xacro` bagian gripper (lines 272–340).
2. Build: `colcon build`.
3. Test: `pytest src/hydroships_control/test/` (62 test).
4. Verifikasi syntax: `python -m py_compile` pada file Python gripper.
5. Update dokumen.

## Validasi

- `colcon build` sukses.
- 62 test pytest lolos.
- `python -m py_compile` tanpa error.
- Verifikasi visual di Gazebo: body gripper di muka depan, jari menjorok ke depan.
