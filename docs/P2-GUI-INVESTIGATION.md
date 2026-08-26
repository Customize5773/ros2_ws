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

## 5. Retest dgn dashboard GUI-ROV asli (2026-08-16, lanjutan) — spike, light, karakterisasi axis

Tindak lanjut §4 kandidat #2 (belum diuji: kombinasi axis / input manusia
nyata) dan item M7 yang belum pernah dites sama sekali (tombol light),
plus penyelesaian Task 3 (karakterisasi fisik surge/sway/heave).

**Setup**: `hydroships_gui.launch.py headless:=true rov_random_spawn:=false
rov_x:=0 rov_y:=0 odom_noise:=false spawn_seed:=5001`, `ROS_DOMAIN_ID=77`,
`server.js` GUI-ROV asli (`RPI_ADDR=127.0.0.1`) + dashboard browser asli
(bukan probe sintetis). Operator manusia: arm, tahan yaw penuh beberapa
detik, kombinasi surge+yaw & sway+yaw, toggle light on/off — sesi bebas
(bukan skrip terkontrol), direkam via `[TELEM]`/`[CMD]` log `server.js`.

### 5a. Spike roll/pitch — reproduksi parsial, jauh di bawah klaim asli

**Peak terukur**: roll **6.40°**, pitch **2.22°** (field `roll`/`pitch` di
`build_telemetry()` sudah dalam derajat, bukan radian — dikonfirmasi ulang
dari `gui_bridge_logic.py:136-137`). Ini **lebih tinggi** dari probe
sintetis single-axis di §4 (0.48°/0.37°) — kombinasi multi-axis + pulsa
manual manusia memang mengeksitasi lebih banyak daripada wrench kontinu
satu-sumbu — **tapi masih ~4-5× di bawah** klaim asli 2026-08-13 (±25-31°).
Peak roll terjadi tepat setelah rentetan pulsa `yaw=50/0` diikuti transisi
ke `sway=50` (`server.log` baris ~6043-6096) — konsisten dengan kandidat
(#2, kombinasi axis) sebagai kontributor, tapi tak cukup besar untuk
menjelaskan seluruh gap ke klaim asli.

**Kesimpulan**: spike ±25-31° 2026-08-13 **masih belum tereproduksi**
bahkan dengan dashboard asli + kombinasi axis manual. Kandidat penjelasan
tersisa (§4 #3, belum diuji): posisi/konteks arena spesifik saat observasi
asli (dekat dinding, collision-induced torque) — sesi retest ini pakai
`rov_x:=0 rov_y:=0` (jauh dari dinding) sehingga tidak menguji skenario
itu. **Status revisi**: turunkan dari "kemungkinan besar resolved" (§4) ke
**"gap besar tetap tak terjelaskan — retest dgn dashboard asli TIDAK
cukup untuk menutup sebagai non-issue"**, karena angka yang direproduksi
(6.4°) tetap jauh di bawah klaim (25-31°) walau memakai kondisi paling
mendekati observasi asli yang sejauh ini diuji.

### 5b. Tombol light — verifikasi round-trip, bukan bug

`[CMD] light = true -> 127.0.0.1:14550` dan `light = false` sukses
diterima `gui_bridge` (dikonfirmasi via `server.log`). Investigasi kode
(`gui_bridge_logic.py:101-103`, `gui_bridge.py`) mengonfirmasi: `light`
**disengaja** cuma disimpan sbg status flag & di-echo balik ke telemetry
(`build_telemetry` field `light`) — **tidak pernah memicu aksi ROS/topic
aktuator apa pun**, karena tak ada model lampu di sim/URDF saat ini.
**Bukan bug** — desain saat ini memang belum mengimplementasikan aktuator
lampu, cuma jalur status UI. Item ini bisa ditutup sbg "diverifikasi
sengaja non-aktuasi", bukan lagi "belum sempat dites".

### 5c. Karakterisasi fisik surge/sway/heave — Task 3 lanjutan

`p2-experiment.py --mode step --value 100` (naik dari 50), ROV
di-recenter (`rov_x:=0 rov_y:=0`) tiap sebelum axis surge/heave untuk
menghindari kontaminasi tabrakan dinding dari test sebelumnya.

- **surge** (cmd_fx=40N): **bersih, tanpa tabrakan** — peak `vx=1.083 m/s`
  di t≈1.2s pasca-cmd, plateau stabil sampai akhir window 5s, perpindahan
  cuma 0.92 m (aman dalam radius 2.55 m). Rise-time ~63% peak ≈ 0.49s.
- **sway** (cmd_fy=40N): **menabrak dinding lagi** — peak `vy=1.514 m/s`
  di t=2.03s pasca-cmd, lalu **jatuh mendadak ke 0.53→0.13 m/s** dalam
  0.1s berbarengan `y` berhenti di -2.38 m (radius arena -2.55 m minus
  radius hull). Rise-time ~63% ≈ 0.21s (lebih cepat dari surge — sway
  punya otoritas lebih tinggi ke gain yang sama, konsisten temuan awal
  §"Karakteristik respons"). **Konfirmasi ulang** mekanisme tabrakan yang
  sama seperti temuan awal — kali ini dgn `value` lebih tinggi (100 vs 50)
  sway mencapai kecepatan lebih tinggi shg menabrak LEBIH CEPAT (t≈2.03s
  vs t≈2-3s awal), makin menegaskan ini murni keterbatasan ukuran arena
  `kki_arena.sdf`, bukan anomali kontrol. Tidak diulang dgn window lebih
  pendek karena rise-time (0.21s) sudah cukup terekam sebelum tabrakan.
- **heave** (cmd_fz=30N, naik dari 15N): sinyal **lebih jelas terpisah
  dari noise floor** dibanding test awal — peak `vz≈0.15 m/s` (vs pita
  ±0.06 m/s di test 15N), rise berjenjang terlihat (bukan cuma osilasi
  tanpa tren). Masih **noisy/lemah** relatif terhadap surge/sway (dua
  orde besaran lebih rendah) — kandidat penyebab dari temuan awal (trim
  buoyancy dominan Z, atau `heave_gain=0.30` relatif kecil thd
  massa+drag vertikal) **belum terpisahkan**, tapi sekarang jelas bahwa
  respons heave BUKAN murni noise — ada tren rise yang konsisten dgn cmd
  aktif. `odom_noise:=false` dipakai (default launch arg, sudah nol di
  test awal juga — jitter yang teramati kemungkinan besar dari dinamika
  fisik/allocator, bukan noise sensor yang di-inject).

**Kesimpulan Task 3**: gain aljabar tetap terverifikasi benar (§3, tidak
berubah). Karakterisasi fisik surge sekarang **bersih & lengkap**. Sway
**tetap tak bisa diukur time-constant murni** dalam arena `kki_arena.sdf`
seukuran ini — root cause tetap ukuran arena, bukan kurang effort
pengukuran; kalau perlu angka time-constant murni sway, perlu arena test
terpisah yang lebih besar (di luar `kki_arena.sdf`) atau ukur dari
rise-time 0-63% yang sudah cukup terekam (0.21s) sebelum tabrakan. Heave
kini punya sinyal terukur di atas noise floor tapi respons fisiknya
sendiri (lemah, ~0.15 m/s @ 30N) belum dijelaskan — kandidat penyebab
tetap sama seperti temuan awal, prioritas rendah (bukan bug adapter GUI,
kemungkinan besar karakter fisik hull+buoyancy).

## 6. Retest ROV dekat dinding — kandidat §4 #3 (2026-08-19)

Tindak lanjut kandidat terakhir yang belum diuji dari §4/§5: **konteks posisi
arena** (dekat dinding, collision-induced torque) saat observasi asli
2026-08-13. Investigasi kode sebelum retest menemukan fakta relevan:
`rov_random_spawn` di `sim.launch.py`/`hydroships_gui.launch.py` **default
`true`** (spawn acak dekat salah satu dinding, "posisi kontes realistis") —
sedangkan §4 dan §5 keduanya eksplisit override ke `rov_random_spawn:=false
rov_x:=0 rov_y:=0` (tengah arena). Log 2026-08-13 (`CHANGELOG.md:637-641`)
tidak menyebut override posisi apa pun, jadi observasi asli plausibel memakai
default (dekat dinding) — kandidat ini punya dasar lebih kuat dari sekadar
tebakan.

**Setup**: `hydroships_gui.launch.py headless:=true`, `ROS_DOMAIN_ID=77`,
`tools/p2-experiment.py` (sama seperti H1/H2 di §4: `--axis yaw --value 100`,
mode `sustained --duration 10` / `pulsed --on 0.5 --off 0.5 --cycles 10`).
Catatan lingkungan: `python3` di PATH shell interaktif resolve ke venv proyek
tanpa `rclpy` terkompilasi (bukan masalah baru, sama seperti insiden opencv
2026-08-13 di STATUS.md) — dipakai `/usr/bin/python3` eksplisit, konsisten
dengan konvensi repo. `payload_spawner` mati di kedua run (env `rclpy`
serupa, proses tak terkait uji ini) — diabaikan, tak mempengaruhi
`gui_bridge`/`thruster_allocator`.

**Run 1 — near-wall via spawn default (`spawn_seed:=6001`)**: spawn
`(1.455, 1.75, -0.5)` — dekat wall B, clearance ~0.75-0.8 m dari muka dinding
(`rov_wall_margin=0.8`, dipilih 2026-08-13 utk cegah clipping saat spawn).
H1 sustained: peak roll **0.480°**, pitch **0.364°**. H2 pulsed: peak roll
**0.479°**, pitch **0.362°**. **Identik dengan baseline mid-arena §4**
(0.48°/0.37°) — clearance 0.75-0.8 m cukup jauh sehingga rotasi yaw di
tempat tidak pernah menyentuh dinding (setengah-diagonal hull ~0.24 m,
clearance dari pusat ~0.75+0.17≈0.92 m, jauh di atas itu). Proximity murni
tanpa kontak fisik **tidak** menambah spike — hasil ini belum benar-benar
menguji "collision-induced torque" yang disebut kandidat #3, cuma menguji
separuh "near" tanpa "collision".

**Run 2 — kontak sengaja dgn dinding (`rov_random_spawn:=false rov_x:=2.30
rov_y:=0.0`, haluan yaw=0 menghadap wall C @ x=2.5, clearance awal 0.0275 m
dari muka dinding, cukup dekat agar sapuan diagonal hull saat berputar pasti
menyentuh dinding)**:

- H2-style pulsed: peak roll **2.733°** (t≈29.3s), peak pitch **-2.708°**
  (t≈23.0s). Trace posisi menunjukkan mekanisme jelas: ROV **kontak &
  tergelincir di sepanjang dinding C** (x tetap 2.28-2.36 sementara yaw
  berputar liar -159°..160° dan y bergeser dari 0 ke -2.3, menuju sudut
  C/A) selama t≈21-29s — roll/pitch memuncak persis di jendela kontak ini.
  Setelah lepas dari dinding (t≥30s, x turun ke ~1.2), roll/pitch mereda ke
  pita ±0.03-0.3° (setara baseline bebas-dinding).
- H1-style sustained: peak roll **-1.343°**, peak pitch **-1.422°** — juga
  di atas baseline mid-arena tapi lebih rendah dari pulsed (konsisten
  dengan H2/pulsed selalu menghasilkan eksitasi kontak lebih besar dari
  sustained, pola yang sama seperti pulsa vs sustained di §4/§5).

**Kesimpulan**: kandidat #3 **terkonfirmasi sebagian** — kontak fisik dgn
dinding selama yaw (bukan sekadar "dekat" tanpa sentuh) memang menghasilkan
roll/pitch lebih tinggi dari baseline air-bebas (2.7° vs 0.48°, ~5-6×) lewat
mekanisme nyata & teramati langsung (hull menggesek/tergelincir sepanjang
dinding sambil berputar). **Tapi** angka ini (2.7°) masih **~9-11× di bawah**
klaim asli ±25-31°, bahkan lebih rendah dari kombinasi-axis dashboard asli
di tengah arena (§5a, 6.40°/2.22°). Jadi kontak dinding adalah **kontributor
nyata tapi bukan penjelasan tunggal** untuk gap besar ke klaim asli.

Dengan ini **ketiga kandidat di §4 sudah diuji habis**: (#1) thrust
drop-out watchdog — fixed, sudah tak jadi faktor sejak commit `853f7ff`;
(#2) kombinasi-axis input manusia — kontributor parsial (§5a, sampai 6.4°);
(#3) proximity/collision dinding — kontributor parsial (sesi ini, sampai
2.7° isolated, potensial lebih tinggi kalau digabung dgn #2 tapi belum
diuji kombinasinya). Tak satupun, sendiri-sendiri, mendekati ±25-31°. Klaim
asli kemungkinan besar adalah efek **gabungan** beberapa kandidat sekaligus
(kontak dinding + kombinasi-axis manusia + mungkin sisa artefak thrust
drop-out yang belum ter-fix saat observasi 2026-08-13) — atau anomali
satu-kali yang tak sepenuhnya reproducible secara sistematis dgn probe
terkontrol. **Tetap OPEN** — turunkan prioritas (tak ada bukti bug aktif di
kode saat ini), tapi jangan tandai RESOLVED di STATUS.md.

## 7. Kandidat #4 (2026-08-25, dari kode, belum diuji lapangan): clipping saturasi thruster membocorkan momen roll/pitch tak-terkomando

Re-audit `allocation.py`/`thruster_allocator.py` (bukan observasi lapangan)
mencari mekanisme kode yang belum diuji dari §4-6. `allocate()`
(`allocation.py:73-76`) meng-clip gaya per-thruster ke `[MIN_THRUST,
MAX_THRUST]` **setelah** pseudo-inverse teredam menghitung kombinasi gaya
yang seharusnya menghasilkan wrench komando persis (termasuk membatalkan
komponen roll/pitch yang tak diminta). Clipping terjadi PER-THRUSTER tanpa
redistribusi — begitu satu thruster jenuh, kombinasi gaya yang tersisa tak
lagi membatalkan roll/pitch seperti dirancang TAM, dan wrench yang benar-benar
terkirim ke sim (`TAM @ forces_clipped`) punya residual `mx`/`my` yang TAK
pernah dikomando.

**Verifikasi numerik offline** (`allocation.py` langsung, gain default
`surge_gain=0.40 sway_gain=0.40 heave_gain=0.30 yaw_gain=0.12`, damping=0.1):

| surge% | yaw% | thruster jenuh? | mx terkirim | my terkirim |
|---|---|---|---|---|
| 100 | 100 | ya (thruster_3 54.5N > 50N) | -0.007 | **0.568** |
| 80 | 60 | tidak | -0.005 | 0.589 |
| 50 | 50 | tidak | -0.003 | 0.359 |
| 100 | 30 | tidak | -0.005 | 0.769 |

(skrip: `pinv @ wrench` lalu `np.clip` lalu `tam @ forces_clipped`, dibanding
wrench komando `[fx,0,0,0,0,mz]`.)

Catatan: bahkan TANPA saturasi, `my` residual sudah tak nol (0.36-0.77 N·m) —
pseudo-inverse teredam (damping=0.1) sendiri sudah tak sempurna membatalkan
axis lain (trade-off yang disengaja, lihat komentar `build_damped_pinv`).
Saturasi (baris pertama) menambah my jadi lebih besar tapi BUKAN
menyebabkannya dari nol — jadi kandidat ini kemungkinan cuma memperbesar
kebocoran yang sudah ada di semua kombinasi axis, bukan mekanisme terpisah.
Tak ada validasi berapa N·m residual dibutuhkan utk spike 2-6° (butuh
massa/inertia & damping hidrodinamik ROV, tak tersedia dari kode saja).

**[UPDATE 2026-08-25] Kandidat #4 DIUJI RUNTIME & DIBANTAH.** `tools/p2-experiment.py`
diberi dukungan combo dua-axis (`--axis2`/`--value2`, sustained mode) supaya bisa
mengirim `surge=100 yaw=100` bersamaan lewat `gui_bridge` sungguhan (bukan hitung
offline lagi). Run: ROV mid-arena (`rov_random_spawn:=false`), `headless:=true`,
window aktif command 8s.

**Saturasi terkonfirmasi terjadi persis seperti prediksi analitik**: `t3=50.0000`
(pinned tepat di `MAX_THRUST`) sepanjang window command aktif — bukan spekulasi,
benar-benar saturasi di sim nyata. **Tapi respons roll/pitch yang dihasilkan tidak
signifikan**: `pitch` tetap di rentang -0.0115..-0.0068° sepanjang window aktif
(nyaris nol, jauh di bawah bahkan baseline air-bebas 0.4-0.5° dari §5a) — residual
`my` 0.36-0.77 N·m yang diprediksi offline **tidak menghasilkan spike pitch yang
terukur** begitu diserap dinamika ROV nyata (massa/inertia/damping hidrodinamik).
`roll` justru **meluruh** dari -0.71° (nilai SEBELUM command dikirim, sisa settling
dari spawn — bukan disebabkan command) turun ke -0.04° selama window aktif —
command tidak menambah roll, kalau ada efek malah meredam.

**Kandidat #4 DIBANTAH sbg penjelas spike**: mekanisme kode (clipping per-thruster
tanpa redistribusi) nyata terjadi, tapi besarnya efek pada attitude ROV terlalu
kecil untuk menjelaskan bahkan baseline 0.4-0.5°, apalagi klaim asli ±25-31°.
Konsisten dengan kandidat #2 (kombinasi-axis) — sama-sama kontributor real tapi
kecil, tak satupun (sendiri atau gabungan #1-#4) mendekati skala klaim asli.

Data: `/tmp/claude-*/scratchpad/p2-combo/combo_surge100_yaw100.csv`
(session-scoped, tak disertakan repo).

## Status ringkas

- ✅ Task 1 — root cause + fix thrust drop-out GUI: **selesai, terverifikasi**.
- ✅ Task 2 — profil telemetri: **selesai**, steady-clock fix terkonfirmasi bekerja
  (dikonfirmasi ulang di §4, 10.07 Hz, 93.8% window on-target).
- ✅ Task 3 — gain aljabar terverifikasi benar; **surge bersih & lengkap**
  (§5c); **sway tetap dibatasi ukuran arena** (root cause dikonfirmasi,
  bukan gap pengukuran — rise-time 0.21s cukup terekam); **heave sinyal
  kini terpisah dari noise** tapi respons fisik lemahnya masih belum
  dijelaskan (prioritas rendah, bukan isu adapter GUI).
- ✅ M7 light — **diverifikasi round-trip via dashboard asli** (§5b):
  command diterima `gui_bridge`, **disengaja non-aktuasi** (tak ada model
  lampu di sim) — bukan bug, bukan lagi item "belum dites".
- ⚠️ M7 roll/pitch spike — **keempat kandidat §4/§7 sudah diuji habis** (§6-7):
  kontak fisik dgn dinding selama yaw **terkonfirmasi sebagai kontributor
  nyata** (peak 2.73°/2.71°, ~5-6× di atas baseline air-bebas 0.48°/0.37°,
  mekanisme teramati langsung — hull tergelincir sepanjang dinding sambil
  berputar), tapi sendirian **masih ~9-11× di bawah** klaim asli ±25-31°;
  kombinasi-axis manusia (§5a) berkontribusi sampai 6.40°/2.22°; **saturasi
  allocator (§7) DIUJI RUNTIME 2026-08-25 (`surge=100 yaw=100` combo via
  `gui_bridge` sungguhan, thruster_3 terkonfirmasi saturasi persis 50N) dan
  DIBANTAH** — pitch yang dihasilkan cuma ~0.01°, roll malah meluruh
  (bukan naik) selama command aktif, jauh di bawah bahkan baseline. Tak
  satupun dari keempat kandidat, sendiri-sendiri, mendekati klaim asli —
  kemungkinan besar efek gabungan kontak-dinding + kombinasi-axis manusia
  (dua kandidat yg terbukti berkontribusi nyata), atau anomali satu-kali
  observasi asli 2026-08-13 yang tak sepenuhnya reproducible. **Tetap
  OPEN** (turunkan prioritas, tak ada bukti bug aktif di kode; ruang
  pencarian kandidat kode sudah habis — kandidat baru harus datang dari
  observasi lapangan tambahan, bukan re-audit kode), jangan tandai
  RESOLVED di STATUS.md.
- Belum dikerjakan: lint/typecheck penuh repo.
