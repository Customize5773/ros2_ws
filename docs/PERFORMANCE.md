# Performance — Metrik Harapan & Cara Mengukur

**Status: dokumen baseline.** Belum ada pengukuran sistematis tercatat di repo ini
sebelum dokumen ini ditulis — angka-angka di bawah adalah **target/harapan** yang
diturunkan dari batasan misi resmi (panduan lomba), parameter timeout `mission_fsm`
(`docs/CONFIG_REFERENCE.md` §5), dan batas fisik komponen (proposal tim), **bukan**
hasil profiling aktual. Setiap kali metrik diukur sungguhan, catat hasilnya di §5
supaya dokumen ini berevolusi jadi data historis, bukan cuma target.

---

## 1. Metrik waktu misi (mission timing)

Sumber otoritatif: panduan lomba §4.7.3, dan param timeout `mission_fsm`.

| Metrik | Target/batas | Sumber |
|---|---|---|
| Total durasi run kontes | ≤ 20 menit (5 prep + 10 run + 5 evac) | Panduan lomba §4.7.3 |
| `DIVE` (menyelam ke dasar) | < 20s (t_dive default) | `mission_fsm` param |
| `APPROACH_QR` (scan QR) | < 45s (t_scan default) | `mission_fsm` param |
| `GRAB` (ambil payload) | < 10s (t_grab default) | `mission_fsm` param |
| `NAV_WALL` (ke dinding target) | < 30s (t_nav default) | `mission_fsm` param — **saat ini timeout tercapai karena bug non-convergence, lihat `docs/TROUBLESHOOTING.md` §4b** |
| `HANG` (gantung payload) | < 20s (t_hang default) | `mission_fsm` param |
| `SURFACE` (naik + docking) | < 20s (t_surface default) | `mission_fsm` param |
| `APPROACH_HOOK` (servo ke hook) | < 25s (t_approach default) | `mission_fsm` param |
| `AUTO_RELEASE` (lepas payload otonom) | < 30s (t_release default) | `mission_fsm` param |
| **Total jalur penuh (DIVE→AUTO_RELEASE), best case** | ≈ 200s (~3.3 menit) jumlah semua `t_*` di atas | Dihitung, bukan diukur |

> Total ~3.3 menit adalah **jumlah timeout maksimum**, bukan waktu tempuh normal
> yang diharapkan — dalam praktik tiap state harus selesai jauh lebih cepat dari
> timeoutnya. Target realistis (belum diverifikasi): total run sukses < 8 menit,
> menyisakan margin dalam window 10 menit kontes untuk retry/recovery manual.

---

## 2. Metrik komputasi (CPU/latency)

**Belum diukur.** Yang perlu diukur dan dicatat di sini setelah profiling:

- CPU usage per node (`stabilizer` 20Hz loop, `qr_detector`/`hook_detector` OpenCV
  per-frame) — di mesin dev (laptop) DAN di Raspberry Pi 4B target (lihat
  `docs/HARDWARE.md` §3 poin 6 — ini validasi yang belum dilakukan sama sekali).
- Latency end-to-end: frame kamera masuk → `qr_result`/`hook_offset` keluar. Relevan
  karena visual servo (`docs/TUNING_GUIDE.md` §3) butuh loop rate cukup tinggi
  supaya PD gain yang ditala tidak berosilasi akibat delay deteksi.
- Frame rate aktual kamera vs `max_rate` param (`qr_detector`/`hook_detector`
  default 5.0 Hz) — cek `ros2 topic hz` di kedua topic offset untuk konfirmasi
  throttling bekerja sesuai desain, bukan dibatasi lebih rendah oleh bottleneck lain.
- Loop rate riil `stabilizer` (target 20Hz via `rate` param) — cek jitter, terutama
  di RPi yang mungkin tidak mempertahankan 20Hz stabil di bawah beban node lain
  berjalan bersamaan.

**Cara mengukur (disarankan):**
```
ros2 topic hz /hydroships/qr_offset
ros2 topic hz /hydroships/hook_offset
ros2 topic hz /hydroships/cmd_vel        # output stabilizer, cek dekat 20Hz
top -p $(pgrep -f qr_detector)           # CPU per-node
```

---

## 3. Metrik thrust/energi

**Belum diukur secara sistematis.** Yang relevan untuk dicatat:

- **Thrust utilization** — seberapa sering thruster individual menyentuh batas
  `MIN_THRUST`/`MAX_THRUST` (-40N/+50N, lihat `allocation.py`) selama satu misi
  penuh. Sering menyentuh batas = indikasi `alloc_damping` atau gain PID terlalu
  agresif untuk kapasitas thruster (lihat `docs/TUNING_GUIDE.md` §4).
- **Kondisi matriks alokasi (`cond(TAM)`)** — node `thruster_allocator` sudah
  warning otomatis bila `cond(TAM) > 100`; target desain adalah `cond≈20` (lihat
  `docs/thruster_config.md`). Catat nilai aktual setelah geometri thruster final
  (fisik, bukan sim) dipasang.
- **Daya listrik riil vs kapasitas PSU** — proposal tim: PSU 220V→24V 40A. Belum
  ada pengukuran arus tarik aktual saat semua 6 thruster + servo + Pixhawk + RPi
  aktif bersamaan (worst case: semua thruster mendekati batas maksimum simultan).
  Ini murni pengukuran hardware, dicatat di sini untuk kelengkapan lintas-dokumen
  dengan `docs/HARDWARE.md`.

---

## 4. Metrik akurasi/keandalan

| Metrik | Target | Cara ukur |
|---|---|---|
| Tingkat keberhasilan QR scan per attempt | Tinggi (idealnya 1 trial, skor tertinggi rubrik — lihat panduan lomba §4.7.4 item 2: nilai 15 jika 1 trial, turun ke 5 jika >2 trial) | Log jumlah percobaan `APPROACH_QR` sebelum berhasil, per sesi uji |
| Tingkat keberhasilan grasp payload per attempt | Sama pola skor rubrik (1 trial = nilai penuh) | Log jumlah percobaan `GRAB` sebelum `gripper/attach` berhasil — **saat ini tidak terukur karena bug 4a di TROUBLESHOOTING.md (gripper command tidak pernah terkirim)** |
| Akurasi `NAV_WALL` (jarak akhir vs `nav_tol`) | Konvergen dalam `nav_tol=0.20m` | Bandingkan jarak akhir aktual (dari `/hydroships/odom`) vs target — **saat ini gagal konvergen, lihat bug 4b** |
| Konsistensi hook detection (`hook_offset` valid rate) | Tinggi, minim gap > `hook_max_age` | `ros2 topic hz /hydroships/hook_offset` dibanding frame rate kamera — gap besar memicu fallback dead-reckoning lebih sering dari yang diinginkan |

---

## 5. Log pengukuran aktual

*(Kosong — isi baris baru setiap kali profiling nyata dilakukan, dengan tanggal,
device pengujian [laptop dev / RPi 4B / hardware final], dan link/referensi ke
log mentah bila ada.)*

| Tanggal | Device | Metrik | Hasil | Catatan |
|---|---|---|---|---|
| — | — | — | — | Belum ada data terekam |

---

## Referensi

- Rubrik skor resmi (mempengaruhi prioritas metrik mana yang paling penting
  dioptimalkan): panduan lomba §4.7.4.
- Parameter yang mempengaruhi metrik timing/thrust: `docs/CONFIG_REFERENCE.md`.
- Cara re-tuning bila metrik di atas tidak tercapai: `docs/TUNING_GUIDE.md`.
- Bug aktif yang membuat sebagian metrik §4 tidak bisa diukur saat ini:
  `docs/TROUBLESHOOTING.md` §4, `docs/STATUS.md`.
