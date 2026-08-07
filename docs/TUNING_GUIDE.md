# Tuning Guide — PID, FSM, Visual Servo

Panduan sistematis menyetel parameter kontrol. Untuk daftar lengkap nilai default
dan artinya satu-per-satu, lihat `docs/CONFIG_REFERENCE.md`. Dokumen ini fokus pada
**bagaimana** dan **kenapa**, bukan sekadar daftar.

Prasyarat: pahami loop kontrol di `docs/ARCHITECTURE.md` dan state machine di
`docs/NODES_REFERENCE.md` sebelum mengubah gain apa pun.

---

## 1. Tuning PID `stabilizer` (`config/gains.yaml`)

4 loop independen: `depth`, `heading`, `pitch`, `roll`. Semua memakai struktur
gain sama (`kp`, `ki`, `kd`, `integral_limit`, `out_limit`).

### Metode disarankan (Ziegler–Nichols manual, disederhanakan)

1. **Matikan** `ki` dan `kd` (set 0), sisakan `kp` saja. Aktifkan hanya loop yang
   diuji via `enable_*_hold` (matikan yang lain agar tidak saling ganggu).
2. Naikkan `kp` bertahap sampai sistem berosilasi stabil (bukan divergen) di sekitar
   setpoint. Catat `kp_osc` dan periode osilasi `T_osc`.
3. Set `kp ≈ 0.6 × kp_osc`.
4. Tambahkan `kd` untuk meredam overshoot — mulai dari `kd ≈ kp × T_osc / 8`, naikkan
   bila masih overshoot, turunkan bila responsnya lambat/teredam berlebihan.
5. Tambahkan `ki` kecil terakhir hanya jika ada steady-state error persisten
   (mis. depth tidak pernah persis mencapai target karena buoyancy_ff kurang tepat).
   Naikkan `ki` pelan — nilai terlalu besar menyebabkan overshoot osilasi lambat
   ("integral windup"). Selalu set `integral_limit` untuk membatasi dampak windup
   saat error besar (mis. tepat setelah perubahan setpoint besar).
6. `out_limit` adalah batas keras aktuator (N atau N·m) — set berdasarkan kapasitas
   fisik thruster setelah alokasi (lihat `docs/thruster_config.md`), bukan sekadar
   dinaikkan untuk "respons lebih cepat".

### Urutan tuning disarankan
`depth` dulu (paling kritikal untuk misi — semua state FSM bergantung depth-hold
stabil), lalu `heading` (kritikal untuk `NAV_WALL`/wall-facing), baru `pitch`/`roll`
(stabilitas pasif — biasanya cukup dengan gain kecil karena `cob.z` sudah memberi
momen pemulih pasif, lihat `rov_params.yaml`).

### Isyarat gain terlalu tinggi vs terlalu rendah

| Gejala | Kemungkinan penyebab | Perbaikan |
|---|---|---|
| Osilasi cepat di sekitar setpoint, tidak pernah diam | `kp` terlalu tinggi | Turunkan `kp`, atau naikkan `kd` |
| Respons sangat lambat mencapai setpoint | `kp` terlalu rendah | Naikkan `kp` bertahap |
| Overshoot besar lalu perlahan settle | `kd` kurang | Naikkan `kd` |
| Getaran/noise teramplifikasi (motor "menggigil") | `kd` terlalu tinggi (sensitif ke noise turunan) | Turunkan `kd`, atau tambah filter pada sinyal depth/heading sebelum masuk PID |
| Error kecil menetap tak pernah nol | `ki` kurang, atau `buoyancy_ff` tidak akurat | Naikkan `ki` sedikit; kalibrasi ulang `buoyancy_ff` |
| Output "meledak" setelah setpoint berubah besar lalu lambat pulih | Integral windup, `integral_limit` terlalu longgar | Turunkan `integral_limit` |

### Catatan khusus `buoyancy_ff`
`buoyancy_ff = -0.3` N adalah feed-forward statis — bukan dari sensor, murni
konstanta. Bila ROV tidak near-neutral-buoyancy sebenarnya (mis. setelah ganti
baterai/payload), depth PID akan bekerja lebih keras dari seharusnya untuk
menutup gap ini. Re-kalibrasi `buoyancy_ff` setiap kali ada perubahan hardware
(gripper, payload, baterai) — nilai yang benar: gaya PID depth di steady-state
(hover diam) mendekati nol.

### `use_sim_time`
**Wajib** diset `false` saat menjalankan di hardware fisik (`gains.yaml` atau
override `--params-file`) — nilai `true` (default) membuat `stabilizer` menunggu
`/clock` dari Gazebo yang tidak akan pernah ada di ROV fisik, menyebabkan node diam.

---

## 2. Tuning `mission_fsm` — Timeout per state

Setiap state punya timeout (`t_*`) untuk mencegah misi macet selamanya bila kondisi
gagal tercapai (lihat daftar lengkap `docs/CONFIG_REFERENCE.md` §5). Prinsip umum:

- **Terlalu ketat** (timeout kecil): FSM lompat ke `ABORT` sebelum ROV sempat
  menyelesaikan gerakan sah — terutama berisiko di `t_nav` (NAV_WALL, jarak tempuh
  variabel tergantung posisi spawn) dan `t_scan` (APPROACH_QR, tergantung seberapa
  cepat QR terdeteksi).
- **Terlalu longgar**: durasi misi kontes dibatasi **20 menit** (5 prep + 10 run +
  5 evac, lihat panduan lomba §4.7.3) — total semua `t_*` yang mungkin terpakai
  dalam satu jalur (DIVE→...→AUTO_RELEASE) **harus** muat di bawah 10 menit run,
  idealnya jauh di bawah untuk memberi margin recovery/retry.

Saat menaikkan satu timeout untuk debug (`t_nav` misalnya karena `NAV_WALL` gagal
konvergen — lihat bug aktif di `docs/STATUS.md`), jangan lupa turunkan lagi setelah
akar masalah gerak (bukan timeout) diperbaiki — timeout longgar menutupi gejala,
bukan menyembuhkan penyebab.

---

## 3. Tuning Visual Servo (QR approach & Hook approach)

Dua loop visual servo berbeda di `mission_fsm`, keduanya PD berbasis offset
piksel/area ternormalisasi:

### 3a. `APPROACH_QR` — servo ke QR code
Param: `approach_kp=90.0`, `approach_kd=140.0`, `approach_fmax=16.0`,
`approach_tol=0.06`, `qr_servo_gain=0.15`, `qr_servo_sign=1.0`, `qr_center_tol=0.12`.

- `qr_servo_sign` — **balik nilai ini (1.0 ↔ -1.0) jika ROV bergerak MENJAUH dari QR
  padahal seharusnya mendekat** (indikasi konvensi axis kamera terbalik/di-mirror).
  Selalu cek ini dulu sebelum mengutak-atik gain — arah salah tidak bisa diperbaiki
  dengan menambah/kurangi kp/kd.
- `approach_fmax` — batas gaya servo; terlalu tinggi bisa membuat ROV overshoot
  melewati posisi QR (terutama karena depth-hold dan yaw-hold `stabilizer` juga
  aktif bersamaan, menciptakan interaksi coupling).
- `qr_center_tol`/`approach_tol` — makin kecil = makin presisi tapi makin rawan
  "chattering" (tidak pernah dianggap cukup terpusat) bila ada noise deteksi.

### 3b. `APPROACH_HOOK` — servo ke hook dinding (dipakai `hook_logic.hook_servo`)
Param: `hook_kp_surge=40.0`, `hook_kd_surge=30.0`, `hook_kp_sway=45.0`,
`hook_kd_sway=30.0`, `hook_kp_depth=0.25`, `hook_size_stop=0.35`,
`hook_center_tol=0.15`, `hook_fmax=16.0`, `hook_depth_range=0.20`, `hook_max_age=1.0`.

- `hook_size_stop` — ambang **ukuran** (bukan posisi) hook di frame untuk berhenti
  mendekat: naikkan = berhenti lebih dekat ke hook, turunkan = berhenti lebih jauh.
  Ini pengganti pengukuran jarak riil (tidak ada sensor jarak) — kalibrasi ulang
  wajib bila ukuran fisik hook atau FOV kamera berubah (mis. pindah dari kamera sim
  ke kamera fisik — lihat `docs/HARDWARE.md`).
- `hook_center_tol` — turun = pemusatan (centering) lebih ketat sebelum dianggap
  "locked-on". Terlalu ketat + deteksi noise = ROV tidak pernah maju.
- `hook_max_age` — deteksi yang lebih tua dari nilai ini (detik) dianggap stale,
  FSM fallback ke target odometri murni (dead-reckoning). Turunkan jika frame rate
  deteksi tinggi & stabil (fallback lebih cepat aktif saat deteksi hilang); naikkan
  jika deteksi sering putus-putus sesaat (image loss sementara) agar tidak terlalu
  cepat fallback.
- **Diketahui berosilasi/partial di sim saat ini** (lihat `docs/STATUS.md`,
  `docs/VERIFICATION-CHECKLIST.md` P3) — timeout fallback tetap membuat misi lanjut
  meski servo tidak sempurna konvergen. Prioritaskan investigasi gain sebelum
  menambah toleransi (menambah toleransi menutupi osilasi, tidak menghilangkannya).

### Metode umum tuning visual servo
1. Uji dengan `start_state:=APPROACH_HOOK` (atau `APPROACH_QR`) langsung — lihat
   `docs/HOW-TO-RUN.txt` skenario 3H — supaya tidak perlu menunggu seluruh misi
   dari `DIVE`.
2. Amati `ros2 topic echo /hydroships/hook_offset` (atau `/hydroships/qr_offset`)
   bersamaan dengan `ros2 topic echo /hydroships/manual/cmd` untuk lihat korelasi
   offset→gaya secara langsung.
3. Ubah satu gain pada satu waktu; overshoot besar → turunkan `kp`; osilasi teredam
   lambat → naikkan `kd`; tidak responsif sama sekali → cek dulu apakah offset yang
   diterima memang berubah (masalah deteksi, bukan gain).

---

## 4. Tuning Thruster Allocation (`alloc_damping`)

Lihat `docs/thruster_config.md` untuk detail matematis penuh. Ringkasan tuning:

- `alloc_damping=0.1` (default) — Tikhonov damping pada pseudo-inverse. Menambah
  nilai ini → alokasi "menyerah" lebih cepat pada arah gerak yang lemah secara
  geometris (terutama yaw) alih-alih memaksakan gaya thruster ekstrem yang lantas
  di-clip dan merusak DOF lain.
- **Jangan set ke 0** kecuali kondisi TAM (`cond(TAM)`) sudah dikonfirmasi rendah
  (~20, lihat `thruster_config.md`) — node akan warning bila `cond(TAM) > 100`.
- Naikkan sedikit (mis. 0.15–0.2) jika mengamati thruster individual sering
  menyentuh batas `MIN_THRUST`/`MAX_THRUST` (-40/+50 N) saat gerakan gabungan
  (mis. surge+yaw bersamaan).

---

## 5. Urutan re-tuning setelah pindah ke hardware fisik

Karena semua gain di atas ditala di simulasi (dinamika Gazebo — massa/damping/added-mass
`estimate`, lihat `docs/CONFIG_REFERENCE.md` §1), **asumsikan semua gain PID dan
visual servo perlu ditala ulang dari nol** saat pindah ke ROV fisik. Urutan disarankan:
depth PID → heading PID → pitch/roll PID → alloc_damping (cek kondisi TAM real
setelah posisi thruster fisik final) → visual servo QR → visual servo hook (paling
terakhir karena paling sensitif ke kualitas kamera/pencahayaan asli). Lihat
`docs/HARDWARE.md` §3 untuk daftar lengkap driver yang harus ada dulu sebelum tuning
ini bisa dimulai.
