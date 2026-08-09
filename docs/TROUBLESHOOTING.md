# Troubleshooting

Debugging terstruktur untuk masalah kompleks. Bagian "cara cepat" (tersalin dari
`docs/HOW-TO-RUN.txt` §7, dipertahankan agar quick-reference lama tetap valid) ada
di §1; bagian debugging mendalam ada di §2 dan seterusnya.

---

## 1. Masalah umum & solusi cepat

| Gejala | Penyebab | Solusi |
|---|---|---|
| Model/perubahan tak muncul di Gazebo | Lupa `colcon build`, atau launch masih baca `install/` versi lama | `colcon build` lalu relaunch. Ingat: launch membaca URDF dari `install/`, **bukan** `src/`. |
| ROV berperilaku erratic / thruster "adu perintah" | Proses sim/node lama masih hidup dari run sebelumnya | `pkill -f 'gz sim'; pkill -f parameter_bridge; pkill -f robot_state_publisher; pkill -f mission_fsm; pkill -f stabilizer; pkill -f thruster_allocator` lalu run ulang bersih. |
| Kamera hitam / sim berat / crash render | Mesin tanpa GPU/EGL | `headless:=true`, atau jalankan di mesin ber-GPU. |
| QR tidak terdeteksi walau kamera & unit test normal | **Known issue** — render headless menghasilkan gambar berbeda dari fixture unit test | Lihat §3 di bawah untuk detail & workaround. |
| Model gagal spawn (service `create` belum siap) | Race condition — bridge/service belum siap saat spawn dipanggil | Naikkan `spawn_delay`, mis. `spawn_delay:=6.0`. |
| `ros2`/`gz` command "not found" | Terminal belum di-source | `source /opt/ros/humble/setup.bash && source install/setup.bash`. |

---

## 2. Metodologi debugging (untuk masalah yang tidak ada di tabel di atas)

1. **Isolasi layer.** Stack ini berlapis: Gazebo fisika → `ros_gz_bridge` → node
   Python. Sebelum menuduh logika Python salah, cek dulu apakah data mentah
   sudah benar di layer sebelumnya:
   ```
   ros2 topic echo /hydroships/odom         # cek fisika/pose dulu
   ros2 topic echo /hydroships/depth        # baru cek turunan (depth_publisher)
   ros2 topic hz /hydroships/camera_bottom/image_raw   # cek frame rate kamera hidup
   ```
2. **Isolasi node dengan `start_state`.** `mission_fsm` mendukung mulai dari state
   manapun (`start_state:=NAV_WALL`, dst — lihat `docs/HOW-TO-RUN.txt` §3H). Pakai
   ini untuk mempersempit masalah ke satu state tanpa menunggu seluruh urutan misi.
3. **Bandingkan dengan unit test.** Semua logika inti (`pid.py`, `allocation.py`,
   `qr_logic.py`, `hook_logic.py`, `gripper_logic.py`, `gui_bridge_logic.py`,
   `image_util.py`) punya test headless murni di `test/`. Jika perilaku node ROS2
   aneh tapi test terkait lolos, masalahnya kemungkinan di **integrasi** (topic
   mismatch, timing, param tidak ter-load) bukan di logika itu sendiri — lihat
   contoh kasus QR di §3.
4. **Cek param benar-benar ter-load.** `ros2 param get <node> <param_name>` untuk
   konfirmasi node memakai nilai yang kamu kira (bukan default lama karena launch
   tidak meneruskan argumen dengan benar).
5. **Baca `docs/STATUS.md` dan `docs/CHANGELOG.md` dulu** sebelum debug dari nol —
   bug yang sedang dialami kemungkinan besar sudah terdokumentasi sebagai known
   issue/regression dengan analisis root cause yang sudah ada.

---

## 3. Kasus dalam: QR tidak terdeteksi meski unit test lolos (KNOWN ISSUE)

**Gejala** (biasanya saat `headless:=true`), di log `qr_detector`:
```
[WARN] DECODE GAGAL: QR tak terdeteksi (pts=None)
[WARN] DECODE GAGAL: QR terdeteksi (pts ada) tapi decode kosong
```

**Kenapa ini membingungkan:** `/hydroships/camera_bottom/image_raw` dan
`camera_info` mengalir normal, DAN `test_qr_logic.py` 100% lolos. Tapi test
memakai **fixture gambar statis** (`qr_sim_bottom_A.png`), bukan hasil render
langsung dari sesi headless yang sedang berjalan — jadi "unit test lolos" hanya
membuktikan fungsi decode benar untuk *gambar itu*, bukan bahwa *gambar yang
dihasilkan render headless saat ini* cukup baik untuk didecode. Akar masalah ada
di sisi render/pencahayaan Gazebo headless, bukan di logika `qr_logic.py`.

**Workaround, urut dari paling disarankan:**
1. Jalankan **tanpa** headless (`headless:=false`) — render penuh biasanya
   menyelesaikan masalah langsung.
2. Turunkan `scan_depth` (lebih dekat ke payload) agar QR tampil lebih besar di
   frame — lihat `docs/TUNING_GUIDE.md` untuk pertimbangan trade-off FOV.
3. Untuk menguji FSM tanpa menunggu deteksi visual sama sekali, suntik manual:
   ```
   ros2 topic pub -1 /hydroships/qr_result std_msgs/msg/String "{data: 'A'}"
   ```

**Kapan harus dianggap selesai, bukan sekadar "di-workaround":** ini masih status
OPEN di `docs/VERIFICATION-CHECKLIST.md` (P1). Verifikasi run non-headless yang
konsisten lolos di beberapa percobaan berturut sebelum dianggap resolved.

---

## 4. Bug blocking aktif (per `docs/STATUS.md`, cek dulu apakah masih berlaku)

### 4a. Gripper tidak pernah benar-benar "close" saat state `GRAB`
`mission_fsm.py` mendeklarasikan publisher `pub_grip` ke
`/hydroships/gripper/command` tapi **tidak pernah memanggil `.publish()`** di
`_st_grab` (atau state manapun). Akibat: `gripper_controller` tidak pernah
menerima command "close", `DetachableJoint` tidak pernah attach, payload tidak
pernah benar-benar tergenggam meski FSM lanjut ke state berikutnya seolah berhasil.

**Cara verifikasi cepat:**
```
ros2 topic echo /hydroships/gripper/command   # harus muncul "close" saat state=GRAB
```
Bila topic ini diam total selama state `GRAB` berlangsung, bug ini masih ada.

**Arah perbaikan:** tambahkan `self.pub_grip.publish(String(data='close'))` (dan
`'open'` di tempat yang sesuai) pada transisi masuk/keluar state `GRAB` di
`mission_fsm.py`. Verifikasi lanjutan setelah fix: `gripper/attach` harus
ter-trigger dan payload harus bergerak mengikuti ROV di sim.

### 4b. `NAV_WALL` tidak konvergen ke `nav_tol`
Robot stall di jarak ≈0.26m dari target (parameter `nav_tol` default 0.20m — jadi
tidak pernah "cukup dekat") dan timeout ke `ABORT`. Kemungkinan penyebab (belum
dikonfirmasi definitif — investigasi lanjutan diperlukan):
- Interaksi `stabilizer` (heading-hold) dengan gaya `manual/cmd` dari
  `_st_nav_wall` menciptakan titik kesetimbangan bukan di jarak target sebenarnya.
- `nav_fmax=22.0` mungkin tidak cukup untuk mengatasi drag/damping pada jarak
  akhir approach (gaya makin kecil mendekati target, mendekati nol sebelum benar-benar
  sampai `nav_tol` karena proportional-only control tanpa integral).

**Debug disarankan:** jalankan `start_state:=NAV_WALL start_wall:=B` (lihat
`docs/HOW-TO-RUN.txt` §3C) dan amati `ros2 topic echo /hydroships/odom` vs jarak
target overlay manual — plot jarak-vs-waktu untuk melihat apakah benar-benar
plateau atau berosilasi pelan. Lihat juga `docs/TUNING_GUIDE.md` §2 untuk konteks
timeout, dan §1 untuk kemungkinan re-tuning `heading` PID yang berinteraksi.

---

## 5. Kapan minta bantuan / eskalasi

Jika sudah mengikuti §2 (isolasi layer, isolasi state, cek unit test, cek param)
dan masalah masih tidak jelas akar penyebabnya, dokumentasikan temuan di
`PROBLEM.md` (format `[OPEN]` mengikuti konvensi yang sudah ada) sebelum
menghabiskan waktu lebih lanjut sendirian — riwayat `docs/CHANGELOG.md`
menunjukkan pola: banyak bug di proyek ini butuh analisis lintas-layer (fisika sim
+ konvensi frame + logika kontrol sekaligus) yang lebih cepat diselesaikan dengan
diskusi tim daripada debugging solo berkepanjangan.
