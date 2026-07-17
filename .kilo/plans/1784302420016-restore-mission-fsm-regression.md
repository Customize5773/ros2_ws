# Plan: Restore Gripper + Hook Visual Servo Integration in mission_fsm.py

## Konfirmasi Temuan (diverifikasi via git)

Regression terjadi di `mission_fsm.py` akibat merge PR #14 (`76dae05`):
- **`3f50a69`** (lockkers844-web, 17 Jul 21:55) menghapus 103 baris: `_grip()`, `hook_offset` subscription, integrasi `hook_logic`, dan `APPROACH_HOOK` visual servo PD. Dikembalikan ke navigasi buta koordinat hardcoded.
- **Merge `76dae05`** (17 Jul 22:24) mengambil sisi `3f50a69` untuk `mission_fsm.py`, cuma membawa fix QR (`scan_depth 0.46`, `t_scan 60`, `start_wall`) dari branch `rasya/dev2` (`4fbc6f8`).
- Branch `rasya/dev2` (`4fbc6f8`) **masih punya** integrasi lengkap gripper + hook servo, tapi tidak masuk ke main.

**Dampak:** Run 3C saat ini menghasilkan skor palsu — FSM sampai DONE dengan m5=40, tapi tidak ada grasp fisik (DetachableJoint tidak di-attach) dan APPROACH_HOOK buta tanpa visual servo.

## File yang Harus Diperbaiki

### 1. `src/hydroships_control/hydroships_control/mission_fsm.py`
**Sumber referensi:** `4fbc6f8` (branch `rasya/dev2`)

Perbaikan yang dibutuhkan:
- Import: tambah `PointStamped` dari `geometry_msgs.msg`, tambah `from hydroships_control.hook_logic import HookServoGains, hook_servo`
- Parameter ROS: tambah `hook_max_age`, `hook_kp_surge`, `hook_kd_surge`, `hook_kp_sway`, `hook_kd_sway`, `hook_kp_depth`, `hook_fmax`, `hook_depth_range`, `hook_size_stop`, `hook_center_tol`
- State variables: tambah `self.hook_off`, `self.hook_time`, `self.hook_gains`, `self.done_hooks`
- I/O: tambah `self.pub_grip` (`/hydroships/gripper/command`), tambah subscription `/hydroships/hook_offset`
- Method: tambah `_grip(close)`, `_on_hook(msg)`, `_hook_fresh()`
- State handlers:
  - `_st_grab`: panggil `self._grip(True)` setiap tick
  - `_st_hang`: tambah `self.done_hooks.add(self.wall)` + logging done_hooks
  - `_st_surface`: tambah `self.done_hooks.add(self.wall)` + logging done_hooks sebelum cek >=4
  - `_st_approach_hook`: ganti navigasi buta dengan PD holonomik visual servo (`hook_servo`), fallback timed jika tidak ada deteksi
  - `_st_auto_release`: panggil `self._grip(True/False)`, tambah `done_hooks` tracking + loop kembali DIVE jika <4 hooks
- Docstring: konsistenkan aliran state (APPROACH_QR, bukan SCAN_QR di jalur default)

### 2. `docs/STATUS.md`
- Pastikan M5 dan M6 menggambarkan fitur yang benar-benar ada di kode setelah perbaikan.

### 3. `docs/CHANGELOG.md`
- Tambah entri untuk commit `76dae05`: dokumentasikan bahwa merge membawa fix QR tapi kehilangan gripper + hook servo (regresi integrasi).
- Tambah entri untuk perbaikan ini.

## Strategi Perbaikan

**Opsi yang direkomendasikan: Ambil kode dari `4fbc6f8` dan gabungkan dengan fix QR di `main`.**

Langkah konkret:
1. `git show 4fbc6f8:src/hydroships_control/hydroships_control/mission_fsm.py` → diff dengan HEAD → terapkan perubahan yang hilang secara selektif (bukan seluruh file, agar fix QR tidak hilang).
2. Alternatif lebih aman: cherry-pick commit `4fbc6f8` ke branch baru, lalu merge ke main dengan conflict resolution yang sengaja memilih sisi `4fbc6f8` untuk bagian FSM.

## Validasi

- `colcon build` harus sukses
- `pytest src/hydroships_control/test/` harus lolos (62 test existing + pastikan tidak ada break)
- Verifikasi manual: `git diff HEAD -- src/hydroships_control/hydroships_control/mission_fsm.py` harus menunjukkan pulihnya `_grip`, `hook_offset`, `hook_servo`, dan `done_hooks` tracking.

## Prompt untuk Claude Code (eksekusi)

Salin dan jalankan prompt berikut di Claude Code:

```
Perbaiki regresi integrasi di `mission_fsm.py` akibat merge PR #14.

Konteks:
- Commit `3f50a69` menghapus 103 baris integrasi gripper + hook visual servo dari `mission_fsm.py`.
- Merge `76dae05` mengambil sisi `3f50a69` untuk FSM, cuma membawa fix QR.
- Branch `rasya/dev2` commit `4fbc6f8` masih punya versi lengkap dengan gripper + hook servo.

Tugas:
1. Baca `src/hydroships_control/hydroships_control/mission_fsm.py` di HEAD dan di `4fbc6f8`.
2. Terapkan perubahan yang hilang secara selektif agar:
   - `_grip(close)` method pulih (publish ke `/hydroships/gripper/command`)
   - Import `PointStamped` dan `hook_logic` pulih
   - Parameter hook_* (hook_max_age, hook_kp_*, hook_kd_*, hook_fmax, dll) pulih
   - State variables `hook_off`, `hook_time`, `hook_gains`, `done_hooks` pulih
   - Subscription `/hydroships/hook_offset` pulih
   - Method `_on_hook()` dan `_hook_fresh()` pulih
   - State `_st_grab` panggil `self._grip(True)` setiap tick
   - State `_st_hang` tambah `self.done_hooks.add(self.wall)` dan logging
   - State `_st_surface` tambah `self.done_hooks.add(self.wall)` dan logging sebelum cek >=4
   - State `_st_approach_hook` diubah menjadi PD holonomik visual servo pakai `hook_servo()` dengan fallback timed jika tidak ada deteksi
   - State `_st_auto_release` panggil `self._grip(True/False)` dan tambah `done_hooks` tracking + loop kembali DIVE jika <4 hooks
   - Docstring konsisten dengan jalur default (APPROACH_QR, bukan SCAN_QR)
3. JANGAN hapus fix QR yang sudah ada di main (`scan_depth 0.46`, `t_scan 60`, `start_wall`).
4. Build dan test: jalankan `colcon build` lalu `pytest src/hydroships_control/test/` dari root workspace. Pastikan semua test lolos.
5. Verifikasi: pastikan tidak ada syntax error dengan `python -m py_compile src/hydroships_control/hydroships_control/mission_fsm.py`.

Setelah selesai, tulis ringkasan perubahan yang dilakukan dan file mana yang diubah.
```
