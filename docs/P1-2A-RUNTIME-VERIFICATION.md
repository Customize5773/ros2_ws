# P1-2A: Verifikasi Runtime IMU — Gate Sebelum Implementasi Estimator

Dokumen ini adalah **gerbang verifikasi runtime (go/no-go)**, bukan implementasi. Tidak ada
`attitude_filter_logic.py` atau `attitude_estimator.py` yang dibuat di sini — itu eksplisit di
luar cakupan tugas ini. Tidak ada `stabilizer.py`, `mission_fsm.py`, `gui_bridge.py`,
`thruster_allocator.py`/`allocation.py`, atau `qr_logic.py` yang diubah untuk menghasilkan
dokumen ini. Melanjutkan `docs/P1-2A-ORIENTATION-ESTIMATION-DESIGN.md` (desain yang diverifikasi
di sini) dan `docs/P1-2-STATE-ESTIMATION-INTEGRATION-AUDIT.md` (audit pendahulu).

**Catatan penting soal metodologi:** desain §10.1/§10.2 meminta verifikasi via
`ros2 topic echo /hydroships/imu`. Untuk uji rotasi (Uji 1 gaya §8 desain), sim
(`sim.launch.py`) sendiri **tidak** menjalankan node kontrol apa pun — `/hydroships/cmd_vel`
tidak punya subscriber sampai `thruster_allocator` dijalankan. Untuk mengaktifkan rotasi tanpa
menyentuh `stabilizer.py`/FSM, node `thruster_allocator` (node yang sudah ada, tidak diubah)
dijalankan manual via `ros2 run hydroships_control thruster_allocator` berdampingan dengan sim
— ini bukan controller baru maupun perubahan kode, hanya menjalankan binary yang sudah ada,
persis jalur teleop manual (`cmd_vel` sebagai wrench 6-DOF langsung, sesuai `CLAUDE.md`).

---

## 1. Bukti runtime IMU

### 1.1 Sampel mentah saat diam (init, ~t=13.9s sim time)

```
header:
  stamp: {sec: 13, nanosec: 900000000}
  frame_id: hydroships/base_link/imu
orientation:
  x: 6.705554450571261e-05
  y: 2.584687134376016e-05
  z: -4.464721878894018e-05
  w: 0.9999999964210594
orientation_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
angular_velocity:
  x: 1.654065991789122e-05
  y: 0.00017110301037364804
  z: 1.6385352612178222e-07
angular_velocity_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
linear_acceleration:
  x: -0.0006052196326186167
  y: 0.0012747913216768958
  z: 9.619768482024007
linear_acceleration_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

**`orientation` — TERISI, mendekati identity, BUKAN nol/invalid.** Ini membalikkan asumsi
desain §2/§10.2 yang menduga `orientation` kosong/tidak diisi gz-sim-imu-system default.
Kenyataan runtime: gz-sim-imu-system default (tanpa `<noise>` eksplisit di xacor) **memang
mengisi `orientation`** dengan quaternion hasil integrasi internal plugin sendiri (bukan
`(0,0,0,0)` yang eksplisit invalid, bukan pula flag REP-145).

### 1.2 Beberapa sampel berurutan (steady-state, ~t=21.7-21.8s)

```
stamp 21.700 | orient (x=8.5997e-05, y=-2.7345e-05, z=-4.4412e-05, w=0.999999995) | accel z=9.810440
stamp 21.720 | orient (x=8.6004e-05, y=-2.7628e-05, z=-4.4412e-05, w=0.999999995) | accel z=9.832883
stamp 21.740 | orient (x=8.6010e-05, y=-2.7879e-05, z=-4.4412e-05, w=0.999999995) | accel z=9.832692
stamp 21.760 | orient (x=8.6016e-05, y=-2.8238e-05, z=-4.4412e-05, w=0.999999992) | accel z=9.820674
stamp 21.780 | orient (x=8.6022e-05, y=-2.8555e-05, z=-4.4412e-05, w=0.999999991) | accel z=9.809726
```

- **`header.stamp` maju monoton**, interval 20ms konsisten antar-sampel yang berurutan
  (`nanosec` 700000000 → 720000000 → 740000000 → ...) — sesuai `update_rate=50` Hz di xacro.
  Tidak beku, tidak mundur.
- **Ada jitter halus** antar-sampel (nilai `orientation`, `angular_velocity`, dan
  `linear_acceleration.z` bergeser kecil tiap sampel meski ROV diam) — ini **bukan** noise
  model eksplisit (tidak ada `<noise>` di sensor block, dikonfirmasi ulang di desain §2), tapi
  **variasi numerik riil dari integrasi fisika/plugin**, bukan nilai deterministik-identik
  yang persis sama tiap tick. `linear_acceleration.z` berosilasi ~9.61–9.83 m/s² di sekitar
  9.81 (gravitasi) — plausibel secara fisik untuk accelerometer sim tanpa noise eksplisit,
  bukan konstanta sempurna.

### 1.3 Rate publish aktual

`ros2 topic hz /hydroships/imu` (headless, ~10s window):

```
average rate: 31.455   (window 33)
average rate: 34.350   (window 72)
average rate: 35.308   (window 110)
average rate: 36.365   (window 150)
average rate: 36.088   (window 185)
min: 0.011s  max: 0.080s  std dev: ~0.008s
```

**Rate aktual ~31–36 Hz, BUKAN 50 Hz nominal** yang dikonfigurasi di xacro
(`update_rate=50`). Ini kemungkinan besar karena real-time factor sim headless < 1
(CPU-bound tanpa GPU di environment ini), bukan bug plugin — `header.stamp` (waktu sim)
tetap konsisten 20ms per-tick (§1.2), yang berubah adalah kecepatan wall-clock sim berjalan
relatif ke wall-clock nyata. **Implikasi desain:** estimator harus event-driven dari
`header.stamp`/dt aktual antar-pesan (persis §3.6 desain), bukan mengasumsikan rate tetap
50 Hz — bukti ini justru memperkuat keputusan desain yang sudah ada, bukan mengubahnya.

### 1.4 Uji rotasi — respons terhadap `angular_velocity.z`

Dengan `thruster_allocator` dijalankan dan `/hydroships/cmd_vel` diberi
`{angular: {z: 0.3}}` @ 10 Hz selama beberapa detik:

```
angular_velocity (IMU, saat rotasi mulai terbentuk):
  x: -0.0021270065670466035
  y: -0.001575747914759707
  z: 0.07605218164266028      <- naik dari ~1e-7 (diam) menuju target
```

`angular_velocity.z` **naik jelas dari ~0 (diam) ke ~0.076 rad/s** dan terus bertambah menuju
0.3 rad/s seiring waktu (konsisten dinamika ROV yang punya inersia, bukan instan) — **gyro
IMU merespons rotasi nyata**, bukan nilai statis.

`odom.twist.twist.angular.z` (ground-truth, log paralel) juga menunjukkan nilai nonzero yang
berubah selama window yang sama (`0.0350390228049946`, `-0.017747587571648182` pada sampel
berurutan — nilai berubah-ubah karena masih fase transien ramp-up rotasi, bukan konstan),
mengonfirmasi odom ground-truth memang berubah seperti diharapkan sebagai pembanding.
`odom.pose.pose.orientation` juga bukan konstan (`x=-0.00433...`, `y=-1.82363...` posisi
bergeser tiap sampel — ROV benar-benar bergerak).

**Catatan:** window uji ini pendek (beberapa detik, transien ramp-up thruster) sehingga
angka `angular_velocity.z` belum stabil di 0.3 rad/s persis — cukup untuk membuktikan respons
kualitatif (naik dari ~0 menuju nonzero mengikuti perintah), tidak dimaksudkan sebagai uji
kuantitatif presisi.

### 1.5 `/hydroships/odom` — pembanding (sekali, saat diam)

```
header: {stamp: {sec: 35, nanosec: 391000000}, frame_id: odom}
child_frame_id: base_link
pose.pose.position: {x: 2.0296812642060273, y: 0.4689320313937801, z: -0.11040441309295722}
pose.pose.orientation: {x: 3.716e-05, y: 9.396e-05, z: 0.9920020511583, w: -0.12622171084102335}
pose.covariance: semua 0.0
twist.twist.linear: {0,0,0}
twist.twist.angular: {0,0,0}
twist.covariance: semua 0.0
```

Bentuk pesan/rate/frame_id sesuai deskripsi desain §1 (ground-truth `nav_msgs/Odometry`,
`frame_id=odom`, `child_frame_id=base_link`). Catatan: `pose.pose.orientation` odom di sampel
awal ini sudah menunjukkan yaw signifikan (`z≈0.992`) — spawn orientation ROV di arena
`kki_arena.sdf` bukan identity, tidak relevan untuk verifikasi ini tapi dicatat sebagai
konteks baca sampel.

---

## 2. Consumer yang ada saat ini

`ros2 topic info /hydroships/imu -v`:

```
Type: sensor_msgs/msg/Imu
Publisher count: 1
  Node name: ros_gz_bridge
  Node namespace: /
Subscription count: 0
```

**Konfirmasi eksplisit: TIDAK ADA satu pun node yang subscribe `/hydroships/imu` hari ini.**
Hanya `ros_gz_bridge` sebagai publisher tunggal. Ini konsisten dengan temuan grep statis di
`docs/P1-2-STATE-ESTIMATION-INTEGRATION-AUDIT.md` §1/§6 (`create_subscription.*Imu`: nol
hasil) — dikonfirmasi ulang di runtime, bukan cuma grep source.

---

## 3. Determinasi viabilitas input estimator

- **Accelerometer untuk koreksi roll/pitch: VIABLE.** `linear_acceleration.z` saat diam
  terukur ~9.61–9.83 m/s² (§1.1/§1.2) — magnitudo mendekati g=9.81 m/s², masuk akal secara
  fisik, bukan nol/NaN/konstanta aneh. Cukup untuk menghitung `roll_accel`/`pitch_accel` via
  `atan2` sesuai §3.1 desain.
- **Gyroscope untuk integrasi: VIABLE.** `angular_velocity` diam ~1e-5–1e-7 rad/s (noise
  lantai kecil, bukan nol sempurna — bukti filter tidak bisa mengasumsikan gyro
  benar-benar bias-free), dan **terbukti responsif** terhadap rotasi nyata (§1.4, naik dari
  ~0 ke ~0.076 rad/s mengikuti perintah 0.3 rad/s). Cukup untuk dead-reckoning integrasi
  sesuai §3.2 desain.
- **`msg.orientation` dari `/hydroships/imu` sebagai sumber siap-pakai: TIDAK
  DIREKOMENDASIKAN, tapi dengan alasan berbeda dari dugaan desain awal.** Field ini
  **terisi** (bukan nol/invalid seperti dugaan §2/§10.2 desain), tapi:
  - `orientation_covariance` **semua nol** (bukan `[0]=-1` sesuai konvensi REP-145 untuk
    "tidak tersedia", dan bukan angka non-zero yang berarti "tersedia dengan confidence
    tertentu") — array all-zero secara literal berarti "unknown" per REP-145, jadi
    **tidak ada sinyal metadata yang membedakan apakah `orientation` ini bisa dipercaya**.
  - `orientation` yang terisi tampak berasal dari integrasi murni gyro plugin
    gz-sim-imu-system sendiri (mendekati identity saat diam, konsisten dengan integrasi
    tanpa koreksi eksternal) — **bukan hasil fusi accel+gyro**, jadi tidak lebih baik dari
    yang akan dihasilkan estimator baru sendiri, dan mewarisi masalah drift yang sama tanpa
    kontrol atas parameternya.
  - **Keputusan desain §2 (JANGAN pakai `msg.orientation` sebagai sumber kebenaran, hitung
    sendiri dari accel+gyro) tetap valid dan dikonfirmasi lebih kuat oleh temuan ini** — bukan
    karena field-nya kosong (ternyata terisi), tapi karena field itu tidak auditable/tidak
    dijamin fusi yang benar, dan covariance-nya tidak memberi sinyal apa pun untuk
    memvalidasinya.
- **Yaw: gyro-only tetap satu-satunya sumber yang viable.** Tidak ada topik
  magnetometer/compass apa pun di `ros2 topic list` (dikonfirmasi §1.3 daftar topik penuh:
  hanya `/hydroships/imu`, `/hydroships/odom`, tidak ada topik heading independen lain).
  `orientation.z`/`w` dari `/hydroships/imu` sendiri **tidak dipakai untuk yaw** dengan alasan
  yang sama seperti poin di atas (tidak auditable). Kesimpulan §3.3 desain (yaw drift tanpa
  batas, tidak terhindarkan tanpa magnetometer) **tidak berubah**.
- **Tidak ada data yang diamati yang bertentangan dengan asumsi kritikal desain lain**
  (rate event-driven per-pesan §3.6, kebutuhan dt-clamping §3.8, representasi internal
  quaternion §3.7) — semua tetap berlaku seperti dirancang.

---

## 4. Batas implementasi final (dikonfirmasi, tidak dibangun di sini)

Scope dua-file dari desain **dikonfirmasi tetap sebagai batas implementasi yang benar**,
dengan satu penyesuaian dari temuan §1.3 (rate):

- `attitude_filter_logic.py` — modul pure-logic, complementary filter accel(roll/pitch) +
  gyro-integration(roll/pitch/yaw), representasi internal quaternion, TIDAK membaca
  `msg.orientation` input sama sekali (§3 di atas). Tidak boleh mengasumsikan rate tetap
  50 Hz secara hardcode di parameter dt — harus terima dt aktual per-panggilan (dikonfirmasi
  perlu karena rate runtime terobservasi ~31–36 Hz headless, bukan 50 Hz nominal, §1.3).
- `attitude_estimator.py` — node ROS2 tipis, subscribe `/hydroships/imu`
  (`sensor_msgs/msg/Imu`, sudah ada, 0 subscriber lain — aman tidak mengganggu apa pun yang
  ada), publish `sensor_msgs/msg/Imu` baru pada topik terpisah (nama final TBD saat
  implementasi) dengan:
  - `orientation` = hasil fusi (bukan pass-through dari input, per §3 di atas).
  - `orientation_covariance` diisi non-zero oleh estimator sendiri saat sehat (bukan warisan
    all-zero dari IMU mentah — dikonfirmasi input mentah tidak memberi sinyal apa pun untuk
    diwariskan, §3).
  - `angular_velocity`/`angular_velocity_covariance`, `linear_acceleration`/
    `linear_acceleration_covariance` diteruskan dari IMU mentah (opsional low-pass ringan),
    field-field ini terbukti bermakna secara fisik di runtime (§1).
  - `header.stamp` = stamp IMU mentah yang dipakai untuk update terakhir; `header.frame_id`
    = `imu_link`/frame yang sama seperti input (terobservasi runtime: `hydroships/base_link/imu`,
    bukan `imu_link` polos seperti diduga desain §2 — implementasi harus membaca
    `msg.header.frame_id` dari pesan masuk, bukan hardcode string `"imu_link"`).
- **Tidak** mengubah `stabilizer.py`/`mission_fsm.py`/`gui_bridge.py`/`thruster_allocator.py`
  — dikonfirmasi ulang, tidak ada temuan runtime yang mengubah keputusan ini.

---

## 5. Determinasi perilaku kesehatan/kegagalan

Berdasarkan bukti runtime aktual (bukan hanya desain), dengan catatan eksplisit tentang batas
apa yang bisa/tidak bisa disimpulkan dari sim:

- **Startup/init:** dikonfirmasi `header.stamp` sim mulai dari kecil dan naik monoton sejak
  IMU pertama publish — estimator harus tetap menahan output valid selama fase inisialisasi
  N-sampel (§3.4 desain) meski di sim timestamp bersih; ini murni kontrak desain, tidak
  berubah oleh temuan runtime.
- **Input tidak valid/NaN/Inf:** tidak teramati sama sekali di sim (semua sampel yang diambil
  numerik valid) — desain tetap harus membuang sampel NaN/Inf (§3.8) sebagai pertahanan
  hardware, karena **sim yang bersih tidak membuktikan hardware akan sama bersih** (IMU fisik
  nyata punya kemungkinan glitch/dropout yang sim ini tidak mensimulasikan sama sekali, tidak
  ada `<noise>` ataupun bit-error model).
- **Timestamp mundur:** tidak teramati (§1.2, monoton konsisten 20ms) — tapi guard dt-negatif
  tetap wajib per desain §3.6/§3.8 karena ini properti sim ini secara spesifik (clock sim
  bersih), bukan jaminan umum; hardware real dengan clock terpisah (jika IMU tidak melalui
  jalur `use_sim_time` yang sama) bisa punya kasus ini.
- **dt berlebihan/gap:** rate aktual 31–36 Hz (bukan 50 Hz nominal, §1.3) berarti dt normal
  bervariasi ~28–32ms alih-alih 20ms tetap — **DT_MAX clamp harus dikalibrasi terhadap rate
  riil yang bisa bervariasi tergantung beban CPU sim**, bukan diasumsikan konstan 20ms±kecil.
  Rekomendasi: DT_MAX sebagai parameter node (declare_parameter), bukan konstanta hardcode
  sempit.
- **Sampel hilang/dropout:** `ros2 topic echo --once` awal sempat menunjukkan pesan
  "message was lost" 2x sebelum sampel pertama diterima (§1.1, artefak QoS BEST_EFFORT-ish
  saat subscriber baru join tengah stream) — ini bukti langsung bahwa **loss/gap pesan bisa
  terjadi bahkan di sim**, menguatkan kebutuhan watchdog staleness (§3.8 desain,
  ambang 0.5s mengikuti preseden `thruster_allocator`) sebagai kontrak wajib, bukan
  opsional.
- **Reset filter:** tidak diuji di sini (di luar scope gate ini, tidak ada mekanisme reset
  yang ada untuk diuji) — kontrak desain §3.5 tetap sebagai spesifikasi untuk diimplementasi.
- **Semantik covariance output:** dikonfirmasi §3 — TIDAK boleh mewarisi
  `orientation_covariance` all-zero dari input (yang terbukti tidak bermakna, §1.1/§3),
  estimator baru harus mengisi angka sendiri saat sehat dan `[0]=-1` saat tidak
  siap/stale (konvensi REP-145, sesuai kontrak §7 desain — tidak ada temuan runtime yang
  mengubah rekomendasi ini).
- **Indikasi kesehatan drift-yaw:** tidak ada mekanisme di sim ini untuk mengukur drift
  absolut tanpa referensi (konsisten §3.3/§8 desain) — gate ini tidak menjalankan uji drift
  jangka-panjang (di luar scope, itu bagian dari Uji verifikasi §6 di bawah untuk tugas
  implementasi berikutnya).

---

## 6. Rencana verifikasi (checklist konkret untuk tugas implementasi berikutnya)

### 6.1 Unit test pure-logic (`attitude_filter_logic.py`, tanpa ROS)

1. **Static orientation** — input accel `(0,0,9.81)`, gyro `(0,0,0)` konstan berulang → roll,
   pitch harus konvergen ke 0 (± toleransi kecil) dan tetap stabil (tidak drift) selama N
   sample lanjutan.
2. **Known angular-rate integration** — gyro `z=const rate`, accel tetap gravity-only
   (asumsi rotasi murni yaw, tidak mengubah proyeksi gravity ke roll/pitch) → yaw output
   harus match `rate * elapsed_time` dalam toleransi (dt aktual per-langkah, BUKAN dt tetap
   50 Hz — sesuai §1.3/§4).
3. **Roll/pitch convergence dari accel** — mulai filter dari state salah (mis. roll init 45°
   palsu), berikan accel gravity-only konsisten dan gyro nol → verifikasi roll konvergen
   menuju 0 dalam beberapa konstanta waktu filter (bukan instan, bukan tidak pernah
   konvergen).
4. **Yaw integration murni** — verifikasi yaw TIDAK dikoreksi oleh accel apa pun (accel hanya
   mempengaruhi roll/pitch) — beri accel non-gravity sembarang sambil gyro z nonzero, yaw
   harus tetap murni hasil integrasi gyro.
5. **dt clamp** — beri dt sangat besar (simulasi gap/drop) → verifikasi delta angle yang
   diintegrasikan tidak melonjak melebihi `DT_MAX * rate`, sesuai §3.6/§3.8 desain.
6. **Yaw wrap** — integrasi gyro melewati ±π berulang kali → verifikasi output yaw tetap
   dalam rentang `(-π, π]` memakai `wrap_to_pi` yang sudah ada di `hydroships_control/pid.py`
   (reuse, bukan implementasi baru, sesuai §4 desain).
7. **Invalid input** — sisipkan NaN/Inf pada salah satu field accel/gyro di tengah deret valid
   → verifikasi state filter TIDAK berubah dari sample itu (dipertahankan di nilai valid
   terakhir), dan sample berikutnya yang valid melanjutkan dari state sebelum-NaN, bukan dari
   state ter-corrupt.
8. **dt negatif/nol** (timestamp mundur) — verifikasi sample dibuang atau dt diklem ke
   `dt_min`, tidak menghasilkan integrasi terbalik/nan.

### 6.2 Smoke test node (`attitude_estimator.py`, dengan ROS, tanpa sim penuh — bisa pakai publisher IMU sintetis)

1. Publish beberapa pesan `sensor_msgs/Imu` sintetis ke `/hydroships/imu` → verifikasi node
   publish topik output baru dengan `orientation` **terisi** (bukan default `(0,0,0,0)`) dan
   valid (norm quaternion ≈ 1).
2. Verifikasi `angular_velocity`, `linear_acceleration` (dan covariance masing-masing) pada
   output **sama/mendekati** input (pass-through, dengan atau tanpa low-pass ringan opsional)
   — bukan nol, bukan diubah tanpa alasan.
3. Verifikasi `header.stamp` output = stamp pesan input terakhir yang dipakai (bukan
   wall-clock publish node).
4. Verifikasi selama fase inisialisasi (< N sampel pertama) node **tidak** publish output
   valid (atau publish dengan `orientation_covariance[0] = -1` eksplisit) sesuai kontrak §7
   desain.
5. Verifikasi node **tidak** subscribe `/hydroships/odom` sama sekali (cek statis via
   `ros2 topic info /hydroships/odom -v` setelah node jalan — subscription count node baru
   harus 0 di topik itu) — persyaratan struktural §8 desain untuk mencegah kegagalan (C).

### 6.3 Uji perilaku simulasi (dengan sim + thruster_allocator, seperti dijalankan di gate ini)

1. **Uji rotasi terskrip:** ulangi persis prosedur §1.4 gate ini (thruster_allocator +
   cmd_vel angular z konstan beberapa detik) TAPI dengan `attitude_estimator.py` juga
   berjalan paralel → log topik output estimator vs `/hydroships/odom` (ground-truth) secara
   bersamaan, verifikasi roll/pitch/yaw estimator **mengikuti tren** ground-truth (bukan
   RMS-error presisi ketat, karena §1.4 gate ini hanya window pendek/transien — implementer
   berikutnya sebaiknya pakai window lebih panjang & steady-state untuk metrik presisi sesuai
   §8 desain, target awal ±2° yang perlu dikalibrasi ulang).
2. **Uji independensi dari odom (definitif, deteksi fail-mode C):** selagi estimator jalan
   dan IMU terus mengalir dengan gerakan nyata, hentikan sementara aliran `/hydroships/odom`
   (mis. matikan bridge odom sesaat tanpa mengubah kode) → verifikasi output estimator TETAP
   berubah mengikuti rotasi (bukti tidak bergantung odom sama sekali), sesuai Uji 2 §8
   desain.
3. **Cek non-integrasi (fail-mode B):** setelah estimator ada, verifikasi `stabilizer.py`
   masih subscribe `/hydroships/odom` (bukan topik baru) — cukup code-review/grep statis,
   bukan uji runtime, sesuai Uji 3 §8 desain. Estimator harus EXIST tapi belum DIPAKAI
   controller mana pun setelah implementasi selesai (P1.2A tidak mengintegrasikan).

---

## 7. Verdict gate

**PASS — lanjut ke tugas implementasi.**

Alasan:
- Kedua field sensor yang dibutuhkan filter (accel untuk roll/pitch, gyro untuk integrasi)
  terbukti **valid, bermakna secara fisik, dan responsif terhadap gerakan nyata** di runtime
  (§1.1, §1.2, §1.4) — tidak ada blocker teknis untuk memulai implementasi complementary
  filter sesuai desain.
- `msg.orientation` yang ternyata terisi (bukan kosong seperti dugaan desain) **tidak
  mengubah keputusan desain untuk tidak memakainya sebagai sumber kebenaran** — justru
  memperkuat alasannya (§3) karena covariance-nya tidak memberi sinyal validitas apa pun.
- Tidak ada consumer lain di `/hydroships/imu` hari ini (§2) — aman menambah node baru tanpa
  risiko konflik/duplikasi resource.
- Satu penyesuaian desain minor teridentifikasi dan sudah dicatat di §4/§5 (rate aktual
  31–36 Hz bukan 50 Hz nominal di lingkungan headless ini — DT_MAX/asumsi dt harus adaptif,
  bukan hardcode) — ini **bukan blocker**, hanya parameter tuning yang harus dijadikan
  `declare_parameter` bukan konstanta.
- Tidak ditemukan diskrepansi yang membatalkan premis desain manapun di §2/§3 dokumen desain.

**Eksplisit: tidak ada integrasi consumer yang menjadi bagian tugas ini.** Node/modul
estimator baru **belum dibuat** di gate ini (sesuai batasan tugas) — tugas ini murni
mengumpulkan bukti runtime untuk memvalidasi asumsi desain sebelum implementasi ditulis.

**Konfirmasi tidak ada file terlarang yang diubah:** `stabilizer.py`, `mission_fsm.py`,
`gui_bridge.py`, `thruster_allocator.py`/`allocation.py`, dan `qr_logic.py` **tidak disentuh**
sama sekali oleh sesi kerja ini (perubahan `mission_fsm.py` yang terlihat di `git status`
pada repo ini berasal dari sesi/proses lain yang berjalan paralel di workspace yang sama,
bukan dari pekerjaan gate ini — dikonfirmasi dengan membaca diff-nya, isinya tidak berkaitan
dengan IMU/estimator sama sekali). Satu-satunya file yang dibuat oleh tugas ini adalah
dokumen `docs/P1-2A-RUNTIME-VERIFICATION.md` ini sendiri. Proses sim (`ign gazebo`,
`ros_gz_bridge`, `robot_state_publisher`, `thruster_allocator`) yang dijalankan untuk
verifikasi telah dihentikan (`kill`) di akhir sesi ini, ROV tidak dibiarkan berputar.
