# Plan: Build Body Gripper on ROV + Complete Gripper Mechanism Integration

## Konfirmasi Status Saat Ini

- **FSM integration sudah dipulihkan** di commit `6422d2e` ("fix(mission): restore gripper integration and hook visual servo PD").
  - `_grip()`, `hook_offset` subscription, `hook_logic`, `APPROACH_HOOK` visual servo PD, dan `done_hooks` tracking sudah kembali ke `mission_fsm.py`.
- **Sisa yang belum selesai:**
  - **Body gripper di URDF terlalu kecil** (box 0.05×0.05×0.03 m = 5 cm³) — hampir tidak terlihat di Gazebo.
  - Semua komponen gripper ada di kode, tetapi **belum diuji secara mekanis** di sim (attach/detach DetachableJoint, jaw visual, qr_offset gate).
  - Beberapa dokumen/comment masih menyebut desain gripper lama (2 jari) atau status yang sudah kadaluarsa.

## File yang Harus Diperbaiki

### 1. `src/hydroships_description/urdf/hydroships.urdf.xacro` — Bangun body gripper

**Tujuan:** Gripper harus terlihat jelas di Gazebo, memiliki massa yang masuk akal, dan tetap kompatibel dengan mekanisme DetachableJoint yang ada.

Perubahan yang dibutuhkan:
- **`gripper_base` link:** Perbesar geometri dari `box size="0.05 0.05 0.03"` menjadi bentuk yang lebih representatif (mis. `box size="0.06 0.06 0.08"` atau composite visual dengan cylinder/box) agar terlihat sebagai body gripper nyata.
- **Posisi/offset:** Sesuaikan `xyz` pada `gripper_base_joint` agar body gripper menjorok ke bawah dari perut ROV (mis. `0.06 0 -0.20` atau sesuai desain).
- **Massa:** Tingkatkan `grip_mass` dari `0.05` kg menjadi ~`0.10–0.15` kg (sesuai volume material baru).
- **Visual:** Tambah material yang kontras (mis. abu-abu `rgba="0.5 0.5 0.52 1.0"` atau kuning `rgba="0.85 0.7 0.1 1.0"`) agar mudah dilihat di sim.
- **Jari `gripper_jaw`:** Perbesar menjadi indikator visual yang jelas (mis. `box size="0.012 0.04 0.08"`) sehingga posisi buka/tutup terlihat.
- **Collision:** Sesuaikan box collision dengan visual agar tidak terlalu kecil untuk interaksi fisik.
- **Catatan:** Jangan ubah joint tipe (`fixed` untuk base, `revolute` untuk jaw) atau plugin DetachableJoint — mekanisme ROS/sim harus tetap sama.

### 2. `src/hydroships_control/hydroships_control/mission_fsm.py` — Selesaikan semua mention gripper

Perbaikan yang dibutuhkan:
- Import: pastikan `PointStamped` dari `geometry_msgs.msg` dan `from hydroships_control.hook_logic import HookServoGains, hook_servo` ada.
- Parameter ROS: pastikan `hook_max_age`, `hook_kp_surge`, `hook_kd_surge`, `hook_kp_sway`, `hook_kd_sway`, `hook_kp_depth`, `hook_fmax`, `hook_depth_range`, `hook_size_stop`, `hook_center_tol` ter-deklare.
- State variables: pastikan `self.hook_off`, `self.hook_time`, `self.hook_gains`, `self.done_hooks` ada.
- I/O: pastikan `self.pub_grip` (`/hydroships/gripper/command`) dan subscription `/hydroships/hook_offset` via `_on_hook` ada.
- Method: pastikan `_grip(close)`, `_on_hook(msg)`, `_hook_fresh()` ada.
- State handlers:
  - `_st_grab`: panggil `self._grip(True)` setiap tick
  - `_st_hang`: tambah `self.done_hooks.add(self.wall)` + logging done_hooks
  - `_st_surface`: tambah `self.done_hooks.add(self.wall)` + logging done_hooks sebelum cek >=4
  - `_st_approach_hook`: PD holonomik visual servo (`hook_servo`), fallback timed jika tidak ada deteksi
  - `_st_auto_release`: panggil `self._grip(True/False)`, tambah `done_hooks` tracking + loop kembali DIVE jika <4 hooks
- Docstring: konsistenkan aliran state (APPROACH_QR, bukan SCAN_QR di jalur default)

### 3. `src/hydroships_control/hydroships_control/gripper_controller.py` — Selesaikan mekanisme

Pastikan:
- Startup auto-detach tetap berjalan (`startup_detach_delay=1.5 s`) untuk mengatasi default-attach Fortress.
- Gate attach berdasarkan `/hydroships/qr_offset` (range aman, ukuran minimum, freshness).
- Publikasi `/hydroships/gripper/attach` & `/detach` hanya saat kondisi aman terpenuhi.
- Jaw publishing ke `/hydroships/gripper_jaw/cmd` dengan sudut yang jelas (open ≈ +0.8 rad, close ≈ 0 rad).

### 4. `docs/STATUS.md`
- Update M5: Grasp fisik **belum diuji** → setelah body gripper dibangun, tandai sebagai verifikasi runtime.
- Update M6: Integrasi lengkap gripper + hook servo sudah ada di kode.

### 5. `docs/CHANGELOG.md`
- Tambah entri untuk perubahan body gripper di URDF: dokumentasikan pergeseran dari box 5 cm³ ke body yang lebih besar dan terlihat.
- Dokumentasikan bahwa `6422d2e` sudah memulihkan integrasi FSM; sisa yang hilang hanyalah visibilitas body di Gazebo.

### 6. `docs/HOW-TO-RUN.txt`
- Tambah langkah verifikasi visual: setelah `colcon build` + launch, cek `gz sim` bahwa body gripper terlihat di perut ROV.
- Tambah perintah `ros2 topic echo /hydroships/gripper_jaw/cmd` untuk verifikasi jaw bergerak.

## Strategi Perbaikan

### Langkah 1: Bangun body gripper di URDF
- Edit `hydroships.urdf.xacro` bagian gripper (lines 272–450).
- Perbesar `gripper_base` dan `gripper_jaw`, sesuaikan offset dan massa.
- Build: `colcon build`.
- Verifikasi visual: launch sim dan pastikan body gripper terlihat di Gazebo.

### Langkah 2: Selesaikan integrasi mekanis
- Pastikan `mission_fsm.py` memiliki semua method dan state gripper yang konsisten.
- Pastikan `gripper_controller.py` gate attach/detach bekerja dengan `qr_offset`.
- Build + test: `colcon build && pytest src/hydroships_control/test/`.

### Langkah 3: Verifikasi mekanis di sim
- Jalankan `ros2 launch hydroships_bringup hydroships_mission.launch.py world:=kki_arena.sdf`.
- Verifikasi urutan:
  1. Startup: payload tidak menempel ROV (auto-detach).
  2. APPROACH_QR → GRAB: ROV approach payload, `gripper_controller` kirim attach, payload terangkat.
  3. NAV_WALL/HANG/SURFACE: payload ikut terbawa.
  4. APPROACH_HOOK: visual servo PD mengarah ke hook.
  5. AUTO_RELEASE: payload dilepas di hook, `done_hooks` bertambah, loop DIVE jika <4.

## Validasi

- `colcon build` harus sukses tanpa warning URDF/SDF.
- `pytest src/hydroships_control/test/` harus lolos (62 test existing).
- Verifikasi manual di Gazebo: body gripper terlihat, jaw bergerak saat command dikirim, payload ter-attach/detach dengan benar.
- `python -m py_compile` pada `mission_fsm.py` dan `gripper_controller.py` tanpa error.

## Prompt untuk Claude Code (eksekusi)

```
Selesaikan pembangunan body gripper pada ROV agar muncul dan bekerja secara mekanis.

Konteks:
- FSM integration gripper + hook servo sudah dipulihkan di commit `6422d2e`.
- Body gripper di URDF masih terlalu kecil (box 5×5×3 cm) sehingga hampir tidak terlihat di Gazebo.
- Semua file pendukung (gripper_controller, gripper_logic, hook_logic, hook_detector, payload di SDF) sudah ada.

Tugas:
1. Baca `src/hydroships_description/urdf/hydroships.urdf.xacro` (lines 272–450) dan perbarui body gripper:
   - Perbesar `gripper_base` dari `box size="0.05 0.05 0.03"` menjadi `box size="0.06 0.06 0.08"` (atau bentuk composite yang lebih terlihat).
   - Ubah posisi `gripper_base_joint` xyz menjadi `0.06 0 -0.20` agar body gripper menjorok lebih ke bawah.
   - Tingkatkan `grip_mass` dari `0.05` menjadi `0.12`.
   - Perbesar `gripper_jaw` visual menjadi `box size="0.014 0.04 0.08"`.
   - Sesuaikan collision box agar matching visual.
   - Pastikan `<preserveFixedJoint>true</preserveFixedJoint>` tetap ada.
   - Jangan ubah plugin DetachableJoint atau joint tipe.

2. Baca `src/hydroships_control/hydroships_control/mission_fsm.py` dan pastikan semua elemen gripper + hook ada:
   - `_grip()`, `_on_hook()`, `_hook_fresh()`, `pub_grip`, `hook_offset` subscription.
   - State handlers `_st_grab`, `_st_approach_hook`, `_st_auto_release` menggunakan gripper/hook logic.
   - `done_hooks` tracking.

3. Baca `src/hydroships_control/hydroships_control/gripper_controller.py` dan pastikan:
   - Startup auto-detach (`startup_detach_delay`) ada.
   - Gate attach menggunakan `gripper_logic.is_safe()` berdasarkan `/hydroships/qr_offset`.
   - Publikasi jaw angle yang jelas (open ≈ +0.8, close ≈ 0).

4. Build dan test:
   - Jalankan `colcon build` dari root workspace.
   - Jalankan `pytest src/hydroships_control/test/`.
   - Verifikasi syntax: `python -m py_compile src/hydroships_control/hydroships_control/mission_fsm.py` dan `python -m py_compile src/hydroships_control/hydroships_control/gripper_controller.py`.

5. Update dokumen:
   - `docs/STATUS.md`: update M5 agar mencerminkan body gripper yang sudah dibangun.
   - `docs/CHANGELOG.md`: tambah entri perubahan body gripper.
   - `docs/HOW-TO-RUN.txt`: tambah langkah verifikasi visual body gripper di Gazebo.

Setelah selesai, tulis ringkasan perubahan yang dilakukan dan file mana yang diubah.
```
