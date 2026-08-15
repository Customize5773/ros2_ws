# P2 — Investigasi GUI Bridge (Customize5773/GUI-ROV)

Investigasi jalur `GUI (UDP-JSON) -> gui_bridge -> /hydroships/cmd_vel -> thruster_allocator`.
Tools: [`tools/p2-gui-probe.py`](../tools/p2-gui-probe.py) (kirim perintah UDP manual),
[`tools/p2-gui-telem-profile.py`](../tools/p2-gui-telem-profile.py) (profil telemetri),
[`tools/p2-experiment.py`](../tools/p2-experiment.py) (runner gabungan: kirim perintah +
rekam odom/thruster dalam satu proses ROS, menghindari isu timing lintas-proses).

## 1. Root cause: thrust berhenti ~0.5s setelah perintah GUI dikirim

**Gejala awal**: perintah yaw "sustained" dari GUI terlihat seperti torsi
sesaat (spike) lalu ROV diam di satu heading tertentu, alih-alih terus berputar
selama tombol GUI ditekan.

**Reproduksi** (`p2-experiment.py --mode sustained --axis yaw --value 100 --duration 8`):
thruster (`t3`/`t4`, sepasang thruster yaw) jatuh ke **nol** pada t≈0.6s setelah
perintah dikirim, padahal `cmd_mz` pada topik `/hydroships/cmd_vel` tetap terbaca
12.0 (=yaw_gain 0.12 × 100) sepanjang durasi 8 detik. Yaw naik cepat lalu **flat**
di 122.97° — bukan berputar terus seperti diharapkan.

**Penyebab**: `thruster_allocator` punya watchdog 0.5s yang menolkan thrust bila
tidak ada `Twist` baru masuk ke `/hydroships/cmd_vel` (didesain untuk publisher
kontinu seperti `teleop_stabilized`, timer 0.1s). `gui_bridge._handle()` sebelumnya
hanya memanggil `pub_cmd.publish()` **sekali**, saat sebuah datagram UDP baru
diproses — bukan tiap tick. Karena GUI mengirim satu paket UDP per perubahan axis
(bukan stream kontinu), watchdog allocator menolkan thrust ~0.5s kemudian meski
"perintah logis" masih aktif secara GUI.

**Fix** ([`gui_bridge.py`](../src/hydroships_control/hydroships_control/gui_bridge.py),
commit `853f7ff`): pindahkan publish wrench dari `_handle()` ke timer `_poll_cmd`
yang sudah jalan 50Hz, republish `self.logic.wrench()` (state axis yang
di-hold, fail-safe ke nol saat disarm) tiap tick — bukan hanya saat datagram baru
tiba.

**Verifikasi pasca-fix**: eksperimen yaw sustained yang sama menunjukkan thrust
(`t3`/`t4` = ±34.55) bertahan penuh selama window 8s dan ROV berputar kontinu
(yaw wrap berulang -180°↔180°) alih-alih diam di satu heading; thrust hanya nol
lagi setelah perintah stop benar-benar dikirim di akhir window.

## 2. Profil telemetri (Task 2)

151 paket selama 15s → **10.07 Hz** efektif, interval median 100ms, stdev 2.66ms.
Fix steady-clock sebelumnya (`0d124f3`, decouple timer telemetri dari sim clock)
bekerja sesuai desain — headless/loaded sim tidak membuat link ke GUI tampak mati.

## 3. Kalibrasi gain — step response per axis (Task 3)

`p2-experiment.py --mode step --axis <axis> --value 50 --duration 5`, arena
`kki_arena.sdf`, tanpa stabilizer aktif (stack `hydroships_gui.launch.py` murni
wrench manual). Konfirmasi pertama: **wrench terukur di `/hydroships/cmd_vel`
cocok persis dengan `gain × value`** untuk keempat axis — gain di `gui_bridge.py`
tidak butuh kalibrasi ulang secara aljabar:

| Axis  | gain | cmd @ value=50 (terukur) | cmd (harapan) |
|-------|------|---------------------------|---------------|
| surge | 0.40 | 20.000 N                  | 20.000 N      |
| sway  | 0.40 | 20.000 N                  | 20.000 N      |
| heave | 0.30 | 15.000 N                  | 15.000 N      |
| yaw   | 0.12 | 6.000 N·m                 | 6.000 N·m     |

Karakteristik respons (bukan closed-loop — murni wrench terhadap drag hull):

- **surge/sway**: `vx`/`vy` naik ke puncak **1.79 m/s** / **1.27 m/s** dalam ~1-2s,
  lalu **jatuh mendadak ke ~0** di t≈2-3s berbarengan dengan posisi `x`/`y` yang
  berhenti berubah — pola konsisten dengan ROV **menabrak dinding arena**
  (`kki_arena.sdf` adalah kolam terbatas), bukan anomali kontrol. Perlu diulang
  dengan `spawn_seed` yang memberi ruang gerak lebih jauh dari dinding kalau mau
  ukur time-constant murni tanpa gangguan tabrakan.
- **heave**: respons **sangat lemah**, `vz` berosilasi di pita ±0.06 m/s tanpa
  tren naik jelas selama window 5s hold — berada dekat lantai noise
  `odom_injector` (vel_std=0.02 m/s, jadi ±0.06 ≈ 3σ). Pada 15N net force 50%
  command, percepatan heave nyaris tak terbedakan dari noise. Kandidat penyebab:
  trim buoyancy dominan di axis Z, atau `heave_gain=0.30` memang terlalu kecil
  relatif ke massa+drag vertikal ROV. **Belum disimpulkan** — perlu ulang dengan
  `value` lebih tinggi (mis. 100) dan noise dimatikan (`odom_injector` param) untuk
  memisahkan sinyal dari noise.
- **yaw**: respons bersih, tidak ada indikasi tabrakan (rotasi di tempat).

## Catatan proses (bukan temuan produk)

Selama menjalankan eksperimen di sesi ini sempat ada dua insiden operasional,
dicatat supaya tidak terulang:

1. `pkill -9 -f "hydroships_control|..."` yang dipakai untuk bersih-bersih stack
   sim sempat **menabrak proses sim milik eksperimen lain** (`run_r10_trajectory.sh`,
   sweep varian seed R-10) yang berjalan bersamaan di sesi/terminal lain pada
   `ROS_DOMAIN_ID` default yang sama — pkill berbasis pattern match nama proses,
   bukan PID spesifik. Eksperimen R-10 itu resilient (auto-restart per run) dan
   tampaknya menyelesaikan sendiri, tapi ke depan cleanup harus target PID/pgid
   spesifik dari stack yang benar-benar kita luncurkan, bukan pattern luas.
2. Karena dua stack sim jalan bersamaan di domain yang sama, satu run eksperimen
   sempat **hang tanpa batas** (silang data `/clock`/`/odom` antar dua Gazebo
   instance yang publish topik sama). Fix: jalankan stack investigasi P2 di
   `ROS_DOMAIN_ID` terpisah (dipakai `77` di sesi ini) supaya tidak pernah
   bertabrakan dengan eksperimen lain yang mungkin berjalan bersamaan.

## Status ringkas

- ✅ Task 1 — root cause + fix thrust drop-out GUI: **selesai, terverifikasi**.
- ✅ Task 2 — profil telemetri: **selesai**, steady-clock fix terkonfirmasi bekerja.
- 🧪 Task 3 — gain aljabar terverifikasi benar; karakterisasi respons fisik
  surge/sway terganggu tabrakan dinding, heave terganggu noise floor — **perlu
  run susulan** dengan parameter arena/noise disesuaikan sebelum dianggap
  lengkap.
- Belum dikerjakan: lint/typecheck penuh repo, entri CHANGELOG.
