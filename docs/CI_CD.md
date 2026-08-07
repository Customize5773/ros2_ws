# CI/CD

**Status saat ini: tidak ada CI/CD otomatis.** Tidak ada `.github/workflows/` atau
konfigurasi pipeline lain di repo ini. Semua build dan test dijalankan manual oleh
kontributor di mesin masing-masing. Dokumen ini menjelaskan (1) proses manual yang
ada sekarang, dan (2) rancangan pipeline CI yang disarankan untuk ditambahkan.

---

## 1. Proses build & test manual (kondisi saat ini)

### Build
```bash
cd ~/ros2_ws
colcon build
```
Wajib dijalankan ulang setiap kali file kode/URDF/world/config berubah — launch
file membaca dari `install/`, bukan `src/` (lihat `docs/HOW-TO-RUN.txt` §1).
`build/`, `install/`, `log/` sengaja di-gitignore (regenerasi lokal, tidak
di-commit).

### Test
```bash
cd ~/ros2_ws
colcon test --packages-select hydroships_control
colcon test-result --verbose
```
Framework: **pytest**, seluruhnya berupa unit test pure-logic (tanpa dependensi
Gazebo/ROS integration) di `src/hydroships_control/test/` — lihat daftar lengkap
di `docs/NODES_REFERENCE.md` §"Cakupan Test". Status terakhir tercatat: 76/76 lolos
(`docs/CHANGELOG.md`, entri 2026-08-06).

### Dependensi sistem (prasyarat sebelum build/test bisa jalan)
```bash
# Ubuntu 22.04 + ROS2 Humble harus sudah terpasang, lalu:
sudo apt install ros-humble-ros-gz-sim ros-humble-ros-gz-bridge \
    ros-humble-xacro ros-humble-robot-state-publisher \
    python3-numpy python3-opencv
# atau, bila python3-opencv gagal via apt:
pip install -r requirements.txt
```
Lihat `docs/HOW-TO-RUN.txt` §0 untuk detail lengkap & catatan GPU/headless.

---

## 2. Kenapa belum ada CI otomatis (analisis gap)

- Tidak ada trigger otomatis saat push/PR — regresi (seperti bug `mission_fsm`
  gripper/`NAV_WALL` di `docs/STATUS.md`) hanya ditemukan lewat sesi verifikasi
  manual, bukan tertangkap otomatis sebelum merge.
- Unit test yang ada **seluruhnya headless/pure-logic** — cocok sekali dijalankan
  di CI standar (tidak butuh GPU/Gazebo), tapi belum dimanfaatkan untuk itu.
- Test yang butuh Gazebo (integrasi node penuh, end-to-end mission) **tidak ada
  sama sekali** — baik manual maupun otomatis. Semua verifikasi Gazebo tercatat
  sebagai sesi manual di `docs/CHANGELOG.md`/`docs/VERIFICATION-CHECKLIST.md`.

---

## 3. Rancangan pipeline CI yang disarankan

### Tahap 1 — Lint + Unit Test (mudah, tidak butuh GPU, ROI tinggi)

Cocok dijalankan di GitHub Actions runner standar (`ubuntu-22.04`), tanpa GPU:

```yaml
# .github/workflows/ci.yml (BELUM ADA — usulan)
name: CI
on: [push, pull_request]
jobs:
  build-and-test:
    runs-on: ubuntu-22.04
    steps:
      - uses: actions/checkout@v4
      - name: Setup ROS2 Humble
        uses: ros-tooling/setup-ros@v0.7
        with:
          required-ros-distributions: humble
      - name: Install dependencies
        run: |
          sudo apt install -y ros-humble-ros-gz-sim ros-humble-ros-gz-bridge \
            ros-humble-xacro ros-humble-robot-state-publisher \
            python3-numpy python3-opencv
      - name: Build
        run: |
          source /opt/ros/humble/setup.bash
          colcon build
      - name: Test
        run: |
          source /opt/ros/humble/setup.bash
          source install/setup.bash
          colcon test --packages-select hydroships_control
          colcon test-result --verbose
```

Ini murni usulan struktur — belum diimplementasikan/divalidasi berjalan di GitHub
Actions sungguhan. Sebelum mengaktifkan, jalankan langkah yang sama secara manual
dulu di container `ubuntu-22.04` bersih untuk konfirmasi tidak ada dependensi
implisit dari mesin dev yang tidak tertangkap file `requirements.txt`/`package.xml`.

### Tahap 2 — Headless Gazebo smoke test (lebih sulit, opsional)

Karena `headless:=true` sudah didukung semua launch file (lihat
`docs/CONFIG_REFERENCE.md` §4), secara prinsip mungkin menjalankan smoke test
sim penuh di CI (mis. `hydroships_sim.launch.py headless:=true` lalu cek topic
`/hydroships/odom` mulai publish dalam N detik). **Belum dicoba** — risiko utama:
runner CI standar tidak selalu punya EGL/software rendering yang cukup untuk
`gz-sim-sensors-system` (kamera), dan `docs/TROUBLESHOOTING.md` §3 sudah
mendokumentasikan bahwa render headless punya masalah kualitas tersendiri (QR
decode gagal) bahkan di mesin dev — jadi smoke test sim penuh di CI kemungkinan
butuh scope terbatas (cek topic dasar seperti odom, bukan verifikasi QR/hook
detection) untuk tetap reliable.

### Tahap 3 — Artifact/coverage (nice-to-have)

- `colcon test-result --verbose` bisa diarahkan ke junit XML untuk laporan test
  yang lebih terstruktur di GitHub Actions summary.
- Coverage Python (`pytest-cov`) belum dipakai — semua modul logika murni
  (`pid.py`, `allocation.py`, dst., lihat `docs/NODES_REFERENCE.md`) adalah
  kandidat baik untuk coverage tinggi karena tidak butuh mock ROS/Gazebo berat.

---

## 4. Rekomendasi prioritas

1. **Tahap 1 (lint+unit test) adalah prioritas tertinggi** — biaya implementasi
   rendah (semua test sudah headless & lolos), manfaat langsung: cegah regresi
   pada modul logika inti (PID, alokasi thruster, QR/hook logic, gripper logic)
   sebelum merge, alih-alih hanya ketahuan saat sesi verifikasi manual berikutnya.
2. Tahap 2 (Gazebo smoke test) baru bernilai setelah bug blocking di
   `docs/STATUS.md` (`mission_fsm` gripper, `NAV_WALL`) selesai — smoke test yang
   dijalankan di atas kode yang sudah diketahui broken hanya menambah noise.
3. Tahap 3 (coverage) adalah polish, bukan blocking apa pun.

Setelah pipeline nyata dibuat, update dokumen ini dengan link ke file
`.github/workflows/*.yml` aktual dan status badge — bagian di atas adalah
rancangan usulan, bukan deskripsi sistem yang sudah berjalan.
