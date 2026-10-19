# Uji 3C — misi autonomous FSM

## Tindak lanjut percepatan, 20 Oktober 2026

Perubahan kecil: OpenCV pada node QR memakai satu worker; perjalanan balik
menyimpan kemajuan target lookahead agar tick berikutnya tidak kembali
mengejar waypoint yang sudah dilewati. Tahap misi, timeout, dwell, dan gerbang
hook 2 mm tidak dipangkas. Dua tes regresi memeriksa kemajuan jalur balik dan
penyerahan ke waypoint presisi terakhir.

Benchmark lokal lima decode per gambar, median, pada frame QR D kecil dan
frame tanpa QR: 12 worker → 1 worker menurunkan waktu masing-masing
118 → 71 ms dan 426 → 261 ms. Ini waktu decoder, bukan durasi misi.

Uji penuh dengan perintah yang sama seperti run di bawah mencapai **DONE**,
urutan C → B → D → A, dengan 50 pesan QR yang mencakup keempat huruf.

| Pengukuran wall-clock | Run sebelumnya | Run setelah perubahan |
| --- | ---: | ---: |
| Total misi termasuk startup | 635,21 s | 660,52 s |
| Total REVERSE_RETURN | 99,76 s | 80,66 s |
| Total HANG + AUTO_RELEASE | 296,48 s | 299,69 s |
| Peringatan reverse skip stuck | 1 | 0 |

**Percepatan total belum terbukti:** perjalanan balik lebih singkat pada
run ini, tetapi total misi sekitar 4% lebih lama. Beban komputer dan urutan
wall tidak dikontrol sebagai benchmark berpasangan; jangan mengatribusikan
seluruh perbedaan waktu pada perubahan kode. Run terbaru mencatat 354,30 s
waktu simulasi; waktu simulasi baseline tidak direkam secara langsung.
Sasaran optimasi berikutnya adalah settling kontrol hook (sekitar lima menit
wall-clock), dengan tetap mempertahankan gerbang presisi dan pengamatan
payload setelah release.

Empat payload tetap duduk setelah release selama pengamatan 3,93–4,80 s:
error akhir pusat lubang C/B/D/A = 0,322/0,103/0,347/0,136 mm, semuanya pada
z = −0,4408 m. **186 tes lulus**, empat paket dibangun, tidak ada error runtime
atau traceback shutdown, dan tidak ada proses Gazebo tertinggal.

Bukti: `log/mission-3c-5x5/speed-final.log`, `speed-final.json`,
`speed-comparison.json`, `physical-speed-final.csv`, `tests-speed-final.log`,
dan `build-speed-final.log`. Run `speed` sebelumnya dihentikan setelah siklus
pertama untuk memasukkan koreksi jalur balik, bukan bukti misi penuh.
Batas simulasi/headless dan mekanisme attach pada laporan di bawah tetap berlaku.

## Baseline arena 5 × 5 m, 20 Oktober 2026

**Lulus satu misi penuh sampai DONE: C → B → A → D, dalam 635,21 detik
(sekitar 10 menit 35 detik).** Default launch kini `kki_arena.sdf`, dengan
lantai −0,90 m dan muka dinding ±2,50 m. Kedalaman grab, hook, dan bidang QR
disesuaikan dengan arena tersebut.

Perbaikan dibatasi pada penyebab kegagalan yang teramati:

- Koordinat spawn diteruskan eksplisit ke `ros_gz_sim/create`; sebelumnya
  posisi yang diumumkan berbeda dari posisi payload sebenarnya.
- Servo QR mengoreksi posisi ROV saat ini dan memakai kedalaman terukur.
  QR kecil yang gagal dibaca mendapat percobaan ZBar dengan pembesaran 3×.
- Pemilihan wall wajib memakai QR terdecode; fallback skor tanpa QR dan offset
  kamera sintetis dihapus. Gripper tidak mengakui attach yang ditolak/gagal.
- Gain posisi dan redaman hook diperbaiki, dengan gerbang alignment tetap
  **2 mm**. Retry AUTO_RELEASE mendapat ulang waktu tunggu yang semestinya.
- Gazebo dijalankan langsung agar shutdown tidak meninggalkan server lama.
  Penanganan error callback ROS setelah konteks shutdown diperbaiki; error
  runtime saat konteks aktif tetap diteruskan.

Perintah reproduksi yang dipakai:

```bash
bash tools/ros310.sh launch hydroships_bringup hydroships_mission.launch.py \
  headless:=true joy_trigger:=false spawn_seed:=1001 \
  qr_letter:=C payload_x:=0.4 payload_y:=0.04
```

Probe mengirim trigger operator saat WAIT_TRIGGER; tidak menyuntikkan QR.
Penggunaan biasa tetap membutuhkan trigger operator pada tahap itu.
Tercatat **59 pesan QR** yang mencakup A/B/C/D, **4 grab**, **4 release**,
10.834 pesan odometri, 5.510 frame bawah, dan 5.548 frame depan.
Tidak ada error sebelum DONE atau macetnya waktu simulasi pada run ini.

Pose payload Gazebo diamati setelah setiap pelepasan sampai pergantian siklus:

| Wall | Lama pengamatan setelah lepas | Error akhir pusat lubang terhadap hook |
| --- | ---: | ---: |
| C | 4,05 s | 0,290 mm |
| B | 4,07 s | 0,073 mm |
| A | 4,24 s | 0,073 mm |
| D | 4,38 s | 0,277 mm |

Keempat pengamatan berakhir pada tinggi pusat lubang z = −0,4408 m;
payload tetap duduk pada hook selama interval tersebut.

Verifikasi kode terakhir: **184 test lulus**, build **empat paket berhasil**,
dan `git diff --check` bersih. Run penuh sempat mengeluarkan traceback
`mission_fsm` saat SIGINT setelah DONE. Setelah perbaikan khusus shutdown,
uji 20 detik mengonfirmasi seluruh 13 proses keluar bersih tanpa traceback
atau proses Gazebo tertinggal. Misi penuh tidak diulang setelah perubahan
handler shutdown tersebut. Warning metadata setuptools `tests_require`
masih ada dan tidak menghentikan build.

Bukti lokal (diabaikan Git): `log/mission-3c-5x5/`, terutama `final.log`,
`final.json`, `summary.json`, `physical-final.csv`, `shutdown-check.log`,
`tests-final.log`, dan `build-final.log`.

Batas hasil: satu run dengan seed di atas, headless; belum pengujian banyak
seed, GUI, joystick fisik, atau ROV nyata. Navigasi presisi masih memakai pose
Gazebo dan mekanisme attach simulasi. Payload sebelumnya diganti pada siklus
berikutnya, sehingga ini empat siklus berurutan, bukan empat payload yang
dipertahankan bersamaan. Collision aperture 30 × 30 mm tetap merupakan
pendekatan persegi untuk lubang Ø30 mm; pengamatan duduk sekitar empat detik
per wall belum menguji ketahanan jangka panjang.

## Riwayat: 11 Oktober 2026 (sebelum perbaikan di atas)

Pengujian lokal ROS 2 Humble / Python 3.10 / Gazebo Fortress, kolam latihan
`pool_practice_arena.sdf`, headless, tanpa joystick fisik. Tidak mengendalikan ROV nyata.

## Perbaikan

- HANG sebelumnya memberi skor saat kedalaman berhenti berubah tanpa memeriksa
  jarak ke hook. Test membuktikan posisi meleset 100 mm tetap dianggap sukses.
  Kini keberhasilan memerlukan jarak <50 mm, sama dengan toleransi longgar
  AUTO_RELEASE yang sudah ada.
- Batas gaya penahan XY HANG turun dari 13,2 N pada fase pendekatan menjadi
  2,64 N pada fase turun. Run setelah koreksi gerbang berhenti jujur dengan
  `HANG timeout`, jarak 56 mm. Fase turun kini mempertahankan batas 13,2 N;
  gain PD dan toleransi keberhasilan tetap.
- ZBar dibatasi ke simbol QR. Run awal memunculkan assertion warning decoder
  DataBar yang tidak diperlukan untuk misi ini.
- Fixture FSM diperbarui terhadap geometri kolam aktif dan kecepatan terfilter.
  Test geometri sebelumnya membaca box jari dari link berikutnya setelah body
  gripper diganti mesh; kini memakai grasp envelope logis 60 mm yang masih
  digunakan controller, bukan menyamakan mesh visual dengan geometri grasp.
- `netifaces` ABI Python 3.10 dipasang di `.venv-ros310`; paket sistem terdeteksi
  pip tetapi tidak dapat diimpor interpreter ROS. `ros2 topic pub` berhasil
  menerbitkan pesan setelah perbaikan.
- Panduan memakai topik state `/hydroships/mission/state` dan menjelaskan batas
  reproduksibilitas `spawn_seed`.

## Verifikasi

- Awal: 149 test lulus, 6 gagal.
- Setelah perbaikan: **157 test lulus** (termasuk regresi QR dan gaya HANG).
- Build: empat paket selesai; warning metadata setuptools `tests_require`
  masih ada, tidak menghentikan build.
- Run awal mencapai WAIT_TRIGGER; trigger CLI gagal karena `netifaces`.
- Run dengan gerbang HANG diperbaiki, sebelum perbaikan gaya: ABORT di HANG
  setelah 90,77 detik; 1.594 odometri dan 835 frame per kamera tercatat.

Perintah run akhir:

```bash
bash tools/ros310.sh launch hydroships_bringup hydroships_mission.launch.py \
  headless:=true joy_trigger:=false spawn_seed:=1001 \
  qr_letter:=C payload_x:=0.4 payload_y:=0.04
```

Probe mengirim trigger operator ketika WAIT_TRIGGER. Tidak menyuntikkan QR.
FSM memiliki fallback ground-truth sendiri; keberhasilan state tanpa decode QR
bukan bukti keberhasilan visual autonomy. Urutan wall masih acak.

Bukti mentah lokal (diabaikan Git): `log/mission-3c-check/`, termasuk
`run2.log`, `run3.log`, `run4.log`, `result-run3.json`, `result.json`,
`tests-final.log`, dan `cli-check.log`.

## Hasil run akhir dan pekerjaan yang masih terbuka

**Belum lulus misi penuh sampai DONE.** Run akhir dibatasi 600 detik wall-clock:

- Urutan wall: A → C → D → B. A/C/D selesai sampai AUTO_RELEASE dan memulai
  payload berikutnya. B melewati HANG tetapi berhenti di SURFACE.
- Tercatat **9.936 pesan odometri**, **5.132 frame depan**, dan **5.132 frame
  bawah**. QR terdecode: **0**; ketiga siklus memakai fallback bawaan FSM.
- Tidak muncul kembali assertion DataBar. Keempat wall melewati HANG dengan
  gerbang jarak dan batas gaya yang diperbaiki. Ini satu run, bukan battery
  statistik atau bukti keberhasilan fisik menggantung payload.
- Log berhenti bertambah di SURFACE wall B. Pemeriksaan `/clock --once` selama
  8 detik tidak menerima pesan. Proses Gazebo masih hidup; penyebab berhentinya
  waktu simulasi belum teridentifikasi. Pengambilan stack dengan gdb ditolak
  mekanisme ptrace lingkungan; tidak mengubah pengaturan kernel.
- Probe menghentikan pengujian pada 600 detik. Node kontrol ROS berhenti bersih;
  Gazebo memerlukan eskalasi SIGINT → SIGTERM. Traceback KeyboardInterrupt pada
  run awal berasal dari penghentian group yang mengirim SIGINT ganda; run akhir
  mengirim SIGINT hanya ke launch dan membiarkan launch menghentikan anaknya.

Masih perlu diagnosis hang Gazebo, perbaikan deteksi QR pada render kamera,
dan run ulang sampai DONE sebelum menyebut 3C autonomous penuh berhasil.
Tidak dilakukan pengujian GUI visual, joystick fisik, atau perangkat ROV nyata.

## Tindak lanjut: payload gagal masuk gantungan

Log GUI `log/mission-3c-gui/restart.log` mengonfirmasi wall B ABORT di HANG:
transisi turun terjadi saat error maju 39,9 mm; lalu membesar sampai 58,4 mm.
Launch memakai `hang_approach_depth=0.14`, menimpa koreksi node untuk kolam
latihan (`0.105`). Jalur longgar setelah 15 detik membolehkan turun pada
error sampai 50 mm. Timer alignment juga tidak menyimpan waktu kondisi buruk,
sehingga dwell lama dapat meloloskan posisi yang sudah meleset.

Perbaikan lanjutan:

- Default launch depth disamakan dengan node: 0,105 m.
- HANG/AUTO_RELEASE memerlukan error maju dan radial <2 mm untuk
  turun, memberi skor, atau melepas. Jalur longgar 50 mm dihapus.
- Dwell diulang dari awal ketika alignment hilang.
- 164 test lulus, termasuk regresi error 40 mm setelah 15 detik dan hilangnya
  alignment di tengah dwell. Build empat paket berhasil.

Uji terisolasi wall B, spawn ROV seed 1001, payload B di (0,4; 0,04): pada
run pertama error maju saat mulai turun 4,2 mm dan saat keluar HANG 7,9 mm.
AUTO_RELEASE mengirim detach dan mencapai tahap berikutnya. Ini belum berarti
payload berhasil menggantung secara fisik: rekaman pose setelah lepas menunjukkan
payload bergeser/berputar. Probe tidak menyuntik QR; trigger operator otomatis.
Bukti: `log/hook-fix/run1.log`, `result-run1.json`, `tests-final.log`.

**Koreksi geometri sederhana (ukuran dikonfirmasi pengguna):** lubang visual
Ø30 mm dan pin Ø25 mm dipertahankan. Pusat target dan alat ukur disamakan dengan
mesh: (0, -0,003, 0,079) m lokal payload. Collision memakai lima box tersambung,
dengan aperture 30 × 30 mm. Ini pendekatan persegi untuk lubang bulat; kontak
pada sudut belum merepresentasikan lingkaran persis.

Feedback pose payload Gazebo mengoreksi offset pemasangan aktual; data lebih
tua dari 0,5 s kembali ke target nominal. Deadband dikurangi menjadi 0,5 mm
pada HANG/AUTO_RELEASE dan redaman disesuaikan agar gate 2 mm dapat dicapai.
Build empat paket berhasil. Suite 165 test lulus; setelah penambahan satu
regresi feedback, seluruh 22 test FSM lulus. Uji terbaru berhasil membaca QR B
namun tertahan di APPROACH_QR (pemusatan), sehingga dihentikan sebelum HANG.
Keberhasilan pemasangan fisik belum terverifikasi. Bukti: `log/hook-fix/run3.log`.
