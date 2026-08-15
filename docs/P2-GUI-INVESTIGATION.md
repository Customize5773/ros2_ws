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

## 4. M7 — roll/pitch spike re-test pasca-fix thrust drop-out

Klaim awal STATUS.md/CHANGELOG 2026-08-13 ("roll/pitch spike ±25–31° saat yaw
sustained") diuji ulang di atas kode **setelah** fix thrust drop-out (§1) dan
steady-clock (§2) sudah masuk — spike lama mungkin cuma gejala turunan dari
thrust yang jatuh-bangun tiap 0.5s (watchdog re-trigger), bukan defisiensi
stabilisasi murni.

**Setup**: `hydroships_gui.launch.py headless:=true spawn_seed:=5001`,
`ROS_DOMAIN_ID=77` (terpisah dari domain lain, ikuti catatan insiden §"Catatan
proses" di atas). `tools/p2-experiment.py` (`--mode sustained/pulsed`) sekaligus
merekam `/hydroships/odom` (roll/pitch/yaw, 100Hz) — script recorder terpisah
tidak dibuat karena `p2-experiment.py` sudah menggabungkan fungsi itu.

- **H1 test — sustained yaw 100% selama 10s**: peak roll **0.48°**, peak pitch
  **0.37°** (t≈3.5-4s, langsung setelah cmd_mz naik ke 12 N·m). Setelah cmd
  kembali 0 di t=13s, roll/pitch meluruh ke pita ±0.1-0.3° dalam beberapa detik.
  **Tidak mereproduksi spike ±25-31°** — dua orde besaran lebih kecil.
- **H2 test — pulsed yaw 100%, 0.5s on/off × 10 siklus**: peak roll **0.48°**,
  peak pitch **0.37°** — identik dengan kasus sustained, tidak ada eksitasi
  tambahan dari pola pulsa.
- **Telemetri selama eksperimen** (`p2-gui-telem-profile.py`, port 14551,
  15s window paralel dengan sustained-yaw run): **151 paket, 10.07 Hz efektif**,
  interval median 99.99ms, stdev 1.64ms, 15/16 window 1-detik tepat 10
  paket (window ke-16 terpotong akhir durasi) — **93.8% window memenuhi
  target**, jauh di atas kriteria akseptansi (≥9 Hz median, tak ada window
  <5 Hz). Konsisten dengan hasil §2, mengonfirmasi ulang steady-clock fix
  stabil di run terpisah.

**Kesimpulan**: spike ±25-31° yang tercatat 2026-08-13 **tidak reproduksi**
dengan probe sintetis terkontrol (baik sustained maupun pulsed) di atas kode
saat ini. Kandidat penjelasan (belum diverifikasi, urutan probabilitas):
1. Spike lama adalah efek turunan dari **thrust drop-out watchdog** (§1,
   sudah di-fix commit `853f7ff`) — GUI asli (bukan probe UDP) mengirim
   datagram per keypress, jadi thrust jatuh-bangun tiap 0.5s menghasilkan
   step transient berulang yang beda karakter dari wrench kontinu `p2-experiment.py`;
   H1/H2 di sini sama-sama sudah pasca-fix jadi tak bisa membedakan.
2. Spike direkam dari GUI dashboard asli (browser/keyboard nyata, latensi
   jaringan, kemungkinan multi-axis simultan) — bukan single-axis sintetis;
   kombinasi surge+yaw atau noise input manusia belum diuji di sini.
3. `spawn_seed`/kondisi arena berbeda saat observasi asli (mis. dekat
   dinding, collision-induced torque) — perlu dicek apakah observasi asli
   punya konteks posisi ROV yang dicatat.

**Belum ditutup sebagai non-issue** — perlu re-test dengan GUI dashboard
asli atau data mentah observasi 2026-08-13 (kalau ada log/video) untuk
konfirmasi apakah fix thrust drop-out sudah cukup, sebelum STATUS.md
menandai OPEN issue ini sebagai RESOLVED.

## Status ringkas

- ✅ Task 1 — root cause + fix thrust drop-out GUI: **selesai, terverifikasi**.
- ✅ Task 2 — profil telemetri: **selesai**, steady-clock fix terkonfirmasi bekerja
  (dikonfirmasi ulang di §4, 10.07 Hz, 93.8% window on-target).
- 🧪 Task 3 — gain aljabar terverifikasi benar; karakterisasi respons fisik
  surge/sway terganggu tabrakan dinding, heave terganggu noise floor — **perlu
  run susulan** dengan parameter arena/noise disesuaikan sebelum dianggap
  lengkap.
- 🧪 M7 — roll/pitch spike **tidak reproduksi** dengan probe sintetis
  (sustained & pulsed yaw, peak <0.5°) pasca-fix §1 — **kemungkinan besar
  sudah resolved sebagai efek samping fix thrust drop-out**, tapi belum
  dikonfirmasi definitif (lihat §4 kandidat penjelasan) karena observasi asli
  pakai GUI dashboard nyata, bukan probe sintetis.
- Belum dikerjakan: lint/typecheck penuh repo, entri CHANGELOG.
