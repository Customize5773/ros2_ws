# P1-2A: Desain Estimasi Orientasi (Attitude) — `ros2_ws`

Dokumen desain read-only. Tidak ada kode/config/launch yang diubah untuk menghasilkan
dokumen ini. Melanjutkan `docs/P1-2-STATE-ESTIMATION-INTEGRATION-AUDIT.md` (khususnya §11
item 1 dan Verdict akhir dokumen itu, yang menetapkan node fusi attitude minimal sebagai
tugas implementasi P1.2 paling penting). Keputusan arsitektur dari `docs/P1-1-ARCHITECTURE-DECISION.md`
(Keputusan A — ROS2-native control adalah otoritas; Gazebo ground-truth odom tidak boleh
tetap jadi sumber target-state implisit) dan status "P0 frozen" berlaku dan tidak
dipertanyakan ulang di sini.

Scope dokumen ini **sempit dan sengaja dibatasi**: hanya orientasi (roll/pitch/yaw +
angular velocity). Localization x/y absolut, kecepatan linear (vx/vy), dan
`/hydroships/payload_pose` sim-only **tidak** diselesaikan di sini — itu tetap open question
project-owner sesuai P1-2 §13.

---

## 1. Grafik dependensi saat ini

```
Gazebo physics engine (ground-truth pose model)
        │
        ▼
gz-sim-odometry-publisher-system (hydroships.urdf.xacro:403-410)
        │  gz topic: /model/hydroships/odometry
        ▼
ros_gz_bridge (bridge.yaml:18-22, GZ_TO_ROS)
        │  ros topic: /hydroships/odom (nav_msgs/Odometry)
        ▼
   ┌────┴──────────────┬──────────────────┐
   ▼                    ▼                  ▼
stabilizer.on_odom   mission_fsm._on_odom  gui_bridge._on_odom
(stabilizer.py:160-164)  (mission_fsm.py:450) (gui_bridge.py:93)
```

Bukti langsung, dibaca ulang dari source (bukan diambil mentah dari audit P1-2):

- `stabilizer.py:160-164` (`on_odom`):
  ```python
  def on_odom(self, msg: Odometry):
      self.cur_z = msg.pose.pose.position.z
      self.cur_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
      self.cur_roll, self.cur_pitch = roll_pitch_from_quaternion(
          msg.pose.pose.orientation)
  ```
  Ekstraksi yaw (`stabilizer.py:40-44`) dan roll/pitch (`stabilizer.py:47-56`) adalah
  implementasi quaternion→Euler manual lokal di file ini, bukan panggilan ke `tf_transformations`
  atau library standar mana pun — grep `tf2\|TransformBroadcaster` di seluruh `src/` nol
  hasil (dikonfirmasi ulang, konsisten P1-2 §2).
- Subscription: `stabilizer.py:139` — `create_subscription(Odometry, '/hydroships/odom', self.on_odom, 10)`.
  QoS: depth 10, default (tidak ada `QoSProfile` eksplisit, tidak ada durability/reliability
  khusus). Rate: mengikuti rate publisher, yaitu 30 Hz dari `odom_publish_frequency` plugin
  Gazebo (dikutip dari P1-2 §3, tidak diverifikasi ulang secara independen di dokumen ini
  karena berada di xacro yang sama sudah diaudit).
- `mission_fsm.py:450` — `self.yaw = yaw_from_quaternion(msg.pose.pose.orientation)` di dalam
  `_on_odom`, memakai fungsi `yaw_from_quaternion` yang didefinisikan lokal di
  `mission_fsm.py:45` (duplikat independen dari implementasi `stabilizer.py`, bukan impor
  bersama — tidak ada modul `attitude_math.py`/sejenis di `hydroships_control`).
  `mission_fsm.py` **tidak** memanggil `roll_pitch_from_quaternion` — hanya yaw yang dipakai
  di FSM (dikonfirmasi: grep `roll\|pitch` pada `mission_fsm.py` tidak menghasilkan pemakaian
  dari odom, hanya yaw).
- `gui_bridge.py:93` — `self._rpy = _yaw_rpy(msg.pose.pose.orientation)`, fungsi lokal ketiga
  yang independen (bukan impor dari `stabilizer.py`), khusus untuk telemetri GUI (kategori
  operator-display, bukan loop kendali).

**Kesimpulan §1:** Tiga implementasi quaternion→Euler independen, tiga node consumer,
semuanya membaca **field yang sama** — `msg.pose.pose.orientation` dari `/hydroships/odom`
— yaitu orientasi **ground-truth 100%** dari physics engine Gazebo, bukan pengukuran sensor
apa pun. Tidak ada satu baris kode pun di `src/` yang membaca `/hydroships/imu` (dikonfirmasi
ulang: grep `create_subscription.*Imu` dan `from sensor_msgs.msg import Imu` di seluruh
`src/*.py` — nol hasil). Angular velocity tidak dipakai sama sekali oleh siapa pun sebagai
sinyal langsung — `PID.update()` di `pid.py` menurunkan D-term secara numerik dari histori
error, bukan dari `msg.twist.twist.angular` (yang tersedia di `Odometry` tapi tidak pernah
diambil — grep `twist.angular` di `src/`: nol hasil, dikonfirmasi ulang).

`config/gains.yaml` (dicek untuk cakupan tugas ini) hanya berisi gain PID per axis
(`depth.kp/ki/kd`, `heading.kp/ki/kd`, `pitch.*`, `roll.*`, `buoyancy_ff`) — tidak ada
parameter yang menyebut sumber data (topic name/tipe pesan) untuk state; sumber
sepenuhnya hardcode di `stabilizer.py:139` (`/hydroships/odom`, `nav_msgs/Odometry`). Ini
relevan untuk §5: mengganti sumber state tidak menyentuh `gains.yaml`.

---

## 2. Input yang dibutuhkan estimator — semantik pesan IMU

Definisi sensor di sim (`hydroships.urdf.xacro:246-252`):

```xml
<gazebo reference="imu_link">
  <sensor name="imu" type="imu">
    <always_on>1</always_on>
    <update_rate>50</update_rate>
    <topic>hydroships/imu</topic>
  </sensor>
</gazebo>
```

Bridge (`bridge.yaml:26-30`):

```yaml
- ros_topic_name: "/hydroships/imu"
  gz_topic_name: "/hydroships/imu"
  ros_type_name: "sensor_msgs/msg/Imu"
  gz_type_name: "gz.msgs.IMU"
  direction: GZ_TO_ROS
```

Temuan penting, dikonfirmasi langsung dari blok sensor di atas:

- **Tidak ada elemen `<noise>`** untuk accelerometer maupun gyroscope, tidak ada bias, tidak
  ada `<orientation_reference_frame>`. Ini berarti IMU sim ini **default gz-sim-imu-system
  tanpa noise model kustom** — bukan "noise realistis yang di-tuning tim", melainkan
  konfigurasi paling minimal yang bisa ditulis (placeholder, sesuai komentar
  `<!-- Sensor Gazebo (Milestone 3) -->` dan `<!-- Butuh sistem gz-sim-imu-system ... -->`
  tepat di atasnya, `xacro:240-243`). **Tidak bisa dipastikan dari membaca file ini saja
  apakah gz-sim-imu-system menyuntik noise default sendiri di level plugin** (di luar apa
  yang ditulis di SDF) — ini masuk §10 sebagai unresolved assumption, bukan diklaim di sini.
- `update_rate=50` Hz — ini rate publish IMU di sim, **lebih cepat** dari rate odom
  ground-truth (30 Hz) dan lebih cepat dari loop `stabilizer` (`rate` param, default
  20 Hz — `stabilizer.py:64`). Estimator harus jalan di rate IMU-nya sendiri (event-driven per
  pesan masuk), bukan disamakan paksa ke 20/30 Hz.
- `sensor_msgs/msg/Imu` field yang relevan dan status kevalidannya di sumber ini:
  - `orientation` (geometry_msgs/Quaternion) + `orientation_covariance[9]` — **gz-sim IMU
    sensor pada umumnya TIDAK mengisi field ini secara bermakna** (perilaku standar dokumentasi
    upstream gz-sim: IMU sensor menghasilkan angular_velocity + linear_acceleration murni dari
    integrasi fisika, `orientation` sering dibiarkan identity/kosong kecuali plugin secara
    eksplisit dikonfigurasi menghitungnya). **Tidak dapat dipastikan dari membaca xacro/bridge
    saja** apakah field ini nol/identity/terisi — ini butuh diverifikasi runtime (`ros2 topic
    echo /hydroships/imu` saat sim jalan), yang **tidak dilakukan di sini** karena dokumen ini
    read-only/no-sim. **Asumsi desain aman: JANGAN memakai `msg.orientation` dari topik ini
    sebagai sumber kebenaran** — estimator harus menghitung orientasi sendiri dari
    `angular_velocity` + `linear_acceleration`, persis pola REP-145 untuk sensor yang tidak
    menyediakan orientasi absolut (indikasi lewat `orientation_covariance[0] == -1`, konvensi
    `sensor_msgs/Imu` resmi untuk "orientation tidak tersedia" — perlu dicek runtime apakah
    gz-sim benar-benar menyetel flag ini atau membiarkan array all-zero, dua hal yang beda
    makna per REP-145: all-zero = "unknown", -1 di elemen [0] = "tidak tersedia sama sekali").
  - `angular_velocity` (Vector3, rad/s) + `angular_velocity_covariance[9]` — **ini field yang
    secara fisik bermakna** dari IMU sim (gyro), diasumsikan valid untuk dipakai sebagai input
    gyro-integration filter.
  - `linear_acceleration` (Vector3, m/s²) + `linear_acceleration_covariance[9]` — **secara
    fisik bermakna** (accelerometer), termasuk komponen gravitasi saat diam (konvensi
    standar `sensor_msgs/Imu`: accel measured termasuk gravity reaction force, bukan
    accel bebas-gravitasi) — ini yang dipakai untuk menghitung roll/pitch dari vektor gravitasi
    di §3.
  - `header.frame_id` — akan terisi `imu_link` (nama link sensor di xacro,
    `xacro:210,214,246`) berdasarkan konvensi standar gz-sim-imu-system yang menyalin nama
    link sensor ke frame_id; **tidak diverifikasi runtime**, tapi konsisten dengan pola field
    lain dalam workspace ini.
  - `header.stamp` — bersumber dari clock simulasi (`use_sim_time` diasumsikan aktif di
    seluruh sim ini, konsisten dengan pola `/hydroships/odom` yang dicatat P1-2 §3); **tidak
    diverifikasi ulang di dokumen ini** apakah gz-sim IMU plugin mengisi stamp dengan benar,
    sama seperti catatan P1-2 yang menandai ini "di luar cakupan kode Python".
  - Covariance array (`*_covariance[9]`, 3×3 row-major): **status pengisian nyata tidak dapat
    dipastikan dari file sim** — perlu echo runtime. Desain estimator **tidak boleh berasumsi
    covariance sim ini bermakna secara kuantitatif** (mis. dipakai langsung sebagai bobot EKF)
    tanpa verifikasi; default aman: perlakukan sebagai "unknown", pakai gain filter
    tetap/tuned manual (lihat §3), bukan covariance-driven weighting.

**Kesimpulan §2:** IMU sim ini adalah *sumber gyro+accel yang secara fisik plausible*, tapi
*bukan* sumber orientasi siap-pakai — persis pola IMU murah pada umumnya (di sim maupun
hardware nyata sekalipun tanpa noise eksplisit ditambahkan). Estimator wajib menghitung
orientasi dari `angular_velocity`+`linear_acceleration`, tidak boleh membaca `msg.orientation`.

---

## 3. Pilihan algoritma estimator — desain filter komplementer

**Konteks IMU sim tanpa noise eksplisit (§2) mengubah trade-off desain**, dicatat eksplisit
di sini karena diminta: complementary filter biasanya dirancang untuk menyaring noise
accelerometer jangka-pendek dan drift gyro jangka-panjang. Jika IMU sim benar-benar
noiseless, hasil complementary filter di sim akan **konvergen ke ground-truth yang hampir
sempurna** (drift gyro murni numerik/integrasi dt, sangat kecil) — ini **bagus untuk
verifikasi struktur algoritma** (§8) tapi **berisiko menyembunyikan bug tuning gain** yang
baru muncul begitu IMU hardware asli (dengan noise/bias nyata) dipasang. Desain di bawah
tetap dibuat generik (asumsikan noise mungkin ada) supaya valid untuk kedua kasus, dan
§10 mencatat ini sebagai unresolved assumption eksplisit.

### 3.1 Kontribusi accelerometer (roll/pitch dari vektor gravitasi)

Saat linear acceleration eksternal ≈ 0 (quasi-static / dinamika rendah — asumsi valid untuk
ROV bawah air yang bergerak lambat dibanding pesawat/drone udara, tapi **tidak valid** saat
akselerasi tinggi mis. tabrakan/dorongan mendadak thruster penuh), vektor
`linear_acceleration` yang terukur ≈ reaksi gravitasi $(-g)$ diproyeksikan ke frame body.
Roll/pitch bisa dihitung langsung:

```
roll_accel  = atan2(ay, az)
pitch_accel = atan2(-ax, sqrt(ay^2 + az^2))
```

(konvensi sumbu body ROS REP-103: x-forward, y-left, z-up — perlu dicocokkan ke konvensi
frame `imu_link` di URDF, yang mewarisi orientasi `base_link` karena joint fixed tanpa rotasi,
`xacro:211-215` `<origin xyz="0 0 0" rpy="0 0 0"/>` — jadi `imu_link` sejajar `base_link`,
tidak butuh rotasi tambahan).

**Batasan eksplisit:** metode ini HANYA valid saat percepatan linear eksternal kecil relatif
terhadap g (9.81 m/s²). Saat ROV berakselerasi cepat (dorongan thruster penuh, benturan),
`roll_accel`/`pitch_accel` akan bias — ini alasan struktural kenapa accel-only tidak cukup,
harus digabung gyro.

**Tidak bisa memberi yaw** — accelerometer secara fisik tidak sensitif terhadap rotasi di
sekitar sumbu gravitasi (yaw), ini bukan keterbatasan implementasi, melainkan keterbatasan
fisika accelerometer itu sendiri.

### 3.2 Kontribusi gyro (dead-reckoning + koreksi frekuensi-tinggi)

```
angle_gyro(t) = angle(t-1) + angular_velocity * dt
```

Halus dan responsif jangka-pendek, tapi **drift tanpa batas** jika diintegrasikan sendirian
(bias gyro terintegrasi jadi error sudut yang tumbuh linear terhadap waktu) — untuk roll/pitch,
drift ini dikoreksi oleh accel (§3.1) via blending. **Untuk yaw, tidak ada koreksi absolut
yang tersedia** (lihat §3.3).

### 3.3 Yaw — TIDAK ada magnetometer, drift tidak terhindarkan

Dikonfirmasi eksplisit lewat grep menyeluruh: `grep -rn "mag\|compass\|heading.*sensor" src/`
tidak menghasilkan satu pun sensor magnetometer/compass di workspace ini, dan `HARDWARE.md`
tidak menyebut magnetometer di daftar komponen (Bab 2 proposal sesuai `HARDWARE.md` §1 tidak
dikutip mendetail tapi tabel §2 `HARDWARE.md` — daftar komponen fisik lengkap — juga tidak
memuat magnetometer). Kesimpulan: **yaw dari estimator ini akan selalu gyro-only, murni
dead-reckoning, drift bertambah linear terhadap waktu tanpa batas atas** — ini adalah
**batasan desain fisik yang harus dinyatakan terus terang, bukan bug yang harus "diselesaikan"
di ticket ini**. Implikasi langsung untuk `stabilizer.py` heading-hold dan `mission_fsm.py`
`WALL_HEADING_DEG` alignment: begitu estimator ini menggantikan odom ground-truth untuk yaw,
akurasi heading-hold jangka panjang (durasi misi berjam-jam idealnya, tapi kompetisi KKI
kemungkinan berdurasi menit) akan menurun seiring waktu berjalan sejak estimator diinisialisasi
— **berapa toleransi drift yang diterima adalah pertanyaan project-owner yang sudah dicatat
P1-2 §13 item 3, tidak dijawab di sini.**

### 3.4 Inisialisasi

- N sampel pertama (disarankan: rata-rata sampel selama ~0.5–1 detik pertama, disesuaikan
  dengan `update_rate=50` Hz IMU sim → ~25–50 sampel) dipakai untuk estimasi bias gyro awal
  (asumsi ROV diam/level saat start) dan seed roll/pitch awal dari accel rata-rata.
- Yaw awal **tidak bisa** diinisialisasi dari sensor apa pun (tidak ada referensi absolut) —
  default aman: yaw awal = 0 rad (didefinisikan sebagai origin arbitrer saat estimator start),
  didokumentasikan eksplisit sebagai konvensi, bukan "utara sebenarnya".
- Selama fase inisialisasi (belum cukup sampel), estimator TIDAK boleh publish output valid —
  ini masuk kontrak health/status di §7.

### 3.5 Reset semantics

- Reset manual (mis. dipicu service call/topic khusus, di luar scope implementasi P1.2A
  minimal — cukup dicatat sebagai kebutuhan desain) harus mengembalikan estimator ke fase
  inisialisasi §3.4, bukan sekadar menge-nol-kan output — bias gyro perlu diestimasi ulang.
- Tidak ada "reset otomatis" yang aman untuk yaw (tidak ada ground-truth untuk direset ke
  sana) — kecuali eksplisit dikonfigurasi menyamakan ke nilai tertentu oleh operator/FSM
  (di luar scope P1.2A).

### 3.6 Timestamp handling

Estimator **wajib** memakai `msg.header.stamp` dari `/hydroships/imu` untuk menghitung `dt`,
**bukan** wall clock node (`self.get_clock().now()` di ROS2 tunduk pada `use_sim_time`, tapi
dt yang dihitung dari selisih stamp pesan berturut-turut lebih akurat terhadap rate sensor
riil daripada dt loop timer terpisah — pola ini konsisten dengan cara `stabilizer.on_timer`
menghitung dt dari `self.get_clock().now()` untuk **loop timer**-nya sendiri, tapi untuk
**dt integrasi gyro**, stamp pesan IMU adalah sumber kebenaran yang benar karena update_rate
sensor (50 Hz) tidak dijamin sinkron dengan rate loop node manapun).

- Sample pertama: tidak ada dt sebelumnya — treat sebagai bagian fase inisialisasi (§3.4),
  jangan integrasikan gyro dari sample pertama.
- Gap besar (mis. topic drop lalu resume, dt >> 1/update_rate nominal): clamp dt ke batas atas
  (pola sama seperti `DT_MAX` di `GUI-ROV/attitude_filter.py:41` — ide yang reusable, lihat
  §4) untuk mencegah lonjakan integrasi gyro tunggal yang besar saat data resume setelah gap.

### 3.7 Frame & unit output

- Frame: body frame (`imu_link`, yang sejajar `base_link` — lihat §3.1), **bukan** world
  frame absolut untuk representasi internal quaternion, tapi output roll/pitch/yaw yang
  dipublish harus dalam konvensi yang sama dengan yang dikonsumsi `stabilizer.py` hari ini —
  yaitu **orientasi body relatif terhadap frame `odom`/world**, persis semantik
  `nav_msgs/Odometry.pose.pose.orientation` saat ini (quaternion dari body ke world/odom
  frame). Ini penting: filter komplementer secara alami menghasilkan estimasi *orientasi body
  relatif world* (bukan orientasi relatif-body-ke-body), jadi tidak ada transformasi frame
  tambahan yang dibutuhkan selain memastikan urutan rotasi (roll-pitch-yaw, konvensi mana)
  konsisten dengan `yaw_from_quaternion`/`roll_pitch_from_quaternion` yang sudah ada di
  `stabilizer.py:40-56` supaya semantik sudut yang dikonsumsi PID tidak berubah tanda/konvensi.
- Unit: rad (sudut), rad/s (rate) — konsisten dengan seluruh workspace ini (`stabilizer.py`
  memakai rad untuk target_heading dkk, dikonfirmasi `on_heading_sp` memakai `wrap_to_pi`
  langsung tanpa konversi derajat).
- Representasi internal: **quaternion**, bukan Euler langsung, untuk menghindari gimbal lock
  saat integrasi gyro pada sudut pitch mendekati ±90°. Konversi ke Euler (roll/pitch/yaw)
  hanya dilakukan di titik output, mengikuti pola yang sudah ada (`stabilizer.py` sendiri
  menyimpan `cur_roll/cur_pitch/cur_yaw` sebagai Euler *setelah* ekstraksi dari quaternion
  odom — pola serupa bisa dipertahankan di sisi consumer, estimator baru hanya perlu
  menjamin representasi *internalnya sendiri* quaternion supaya integrasi gyro tidak singular).
  ROV kompetisi KKI kemungkinan tidak beroperasi dekat pitch=±90° secara rutin, tapi memilih
  quaternion secara default menghindari kelas bug ini tanpa biaya tambahan berarti.

### 3.8 Data tidak valid/hilang

- IMU stale (tidak ada pesan baru dalam N × 1/update_rate, mis. > 0.5 detik mengikuti pola
  watchdog `thruster_allocator` 0.5 detik yang sudah ada di codebase ini sebagai preseden
  desain) → estimator berhenti update, publish flag tidak-valid (§7), TIDAK terus
  mengekstrapolasi gyro tanpa batas.
- NaN/Inf pada field apa pun dari pesan masuk → sample tersebut dibuang, tidak
  dipropagasikan ke state filter (state filter tetap di nilai valid terakhir).
- Loncatan dt negatif/nol (mis. stamp mundur, khas isu clock sim/replay) → dt diklem ke
  `dt_min` (pola sama `GUI-ROV/attitude_filter.py:40` `DT_MIN`) atau sample dibuang jika dt
  negatif.

### 3.9 Magnetometer

Tidak ada, tidak diasumsikan, tidak diusulkan untuk ditambahkan di ticket ini — dinyatakan
eksplisit sesuai batasan tugas. Jika project-owner nanti memutuskan magnetometer perlu
dibeli (§13 P1-2, atau keputusan baru di luar dokumen ini), itu perubahan desain terpisah
yang butuh dokumen sendiri, bukan asumsi diam-diam di sini.

---

## 4. Perbandingan dengan referensi implementasi — `GUI-ROV/attitude_filter.py`

Dibaca penuh (153 baris). Temuan struktural kunci: **ini BUKAN filter yang mengonsumsi raw
IMU sensor** — input `update()` (`attitude_filter.py:82-91`) adalah
`roll_deg, pitch_deg, yaw_deg, rollspeed_deg_s, pitchspeed_deg_s, yawspeed_deg_s, dt`, yang
menurut docstring modul (`attitude_filter.py:8-10`) berasal dari **pesan MAVLink `ATTITUDE`
milik ArduSub/Pixhawk** — yaitu *output EKF attitude milik firmware ArduSub sendiri* (sudah
terfusi di dalam Pixhawk, bukan raw accel/gyro). Ini artinya modul ini adalah **filter kedua
di atas estimator pertama (EKF ArduSub) yang sudah ada**, bukan filter fusi IMU dari nol.

| Aspek | Reusable? | Alasan |
|---|---|---|
| Struktur complementary filter (`angle = alpha*(angle_prev + rate*dt) + (1-alpha)*angle_raw`), `attitude_filter.py:119-124` | **Ya, konsep reusable** | Ini persis algoritma dasar yang dibutuhkan §3.2/3.1, hanya `angle_raw` di P1.2A harus berasal dari `roll_accel/pitch_accel` (§3.1), bukan `roll_deg` dari ATTITUDE Pixhawk |
| Yaw wrap-around handling (`_wrap_pi`, hitung `yaw_error` sebelum blending), `attitude_filter.py:126-132` | **Ya, langsung reusable sebagai pola** | Masalah wrap 359°→1° identik apa pun sumber datanya; `wrap_to_pi` yang setara **sudah ada** di `hydroships_control/pid.py` (dipakai `stabilizer.py` via `from hydroships_control.pid import PID, wrap_to_pi`) — implementasi baru harus reuse `wrap_to_pi` yang sudah ada di codebase ini, BUKAN port ulang `_wrap_pi` dari GUI-ROV (duplikasi tak perlu) |
| `DT_MIN`/`DT_MAX` clamping (`attitude_filter.py:38-41`) | **Ya, konsep reusable** | Preseden desain yang baik untuk §3.6/3.8, tidak spesifik MAVLink |
| Lapisan EMA tambahan di atas complementary filter (`attitude_filter.py:32-36, 134-146`) | **Opsional, evaluasi ulang** | Di GUI-ROV, EMA meredam noise sisa dari ATTITUDE MAVLink (yang sendiri sudah hasil EKF Pixhawk, jadi relatif halus) untuk kebutuhan **display telemetry**, bukan loop kendali cepat. Untuk P1.2A yang outputnya dikonsumsi `stabilizer.py` PID **loop kendali** (bukan cuma telemetry), EMA tambahan menambah lag fase yang bisa merugikan stabilitas PID — **rekomendasi: JANGAN port EMA layer secara default**, cukup complementary filter tunggal; EMA bisa ditambah belakangan HANYA jika noise IMU riil (hardware) terbukti butuh itu, bukan diasumsikan di muka |
| Nilai `COMPLEMENTARY_ALPHA=0.98` spesifik (`attitude_filter.py:30`) | **Tidak langsung reusable** | Di-tuning untuk rate update MAVLink `ATTITUDE` (khas 4–10 Hz on ArduSub default stream rate) dan karakteristik noise EKF Pixhawk-nya — IMU sim `ros2_ws` publish di 50 Hz (`xacro:249`), rate berbeda mengubah bobot efektif alpha per satuan waktu; nilai ini harus di-tuning ulang, bukan dicopy |
| Asumsi input "angle sudah terfusi" (ATTITUDE = output EKF Pixhawk) | **TIDAK PORTABLE — bertentangan langsung dengan Keputusan A** | Seluruh premis modul ini adalah "percayakan absolute-angle ke firmware eksternal (Pixhawk/ArduSub), filter ini cuma smoothing tambahan" — di bawah Keputusan A, Pixhawk BUKAN otoritas kendali dan (per §13 item 2 P1-2, masih pertanyaan terbuka) mungkin bahkan tidak dipakai untuk membaca IMU sama sekali. P1.2A harus memfusi **raw accel+gyro** (§3.1/3.2), bukan mem-filter attitude yang sudah difusi orang lain — ini beda kelas masalah, bukan variasi kecil |
| `rov_agent.py` sebagai bridge MAVLink pemanggil modul ini | **Tidak relevan/retired** | Sudah diputuskan retired di P1-1 §6, dikonfirmasi ulang P1-2 §9, tidak diulang di sini |

**Verdict §4/§9:** `attitude_filter.py` **bukan implementasi filter-fusi-IMU yang bisa
diadaptasi langsung** — ia adalah lapisan smoothing di atas estimator attitude yang SUDAH
ADA di firmware lain (ArduSub EKF). Yang reusable murni pada level **pola/ide algoritma**
(struktur complementary filter, wrap-around yaw, dt clamping) — bukan pada level "port
fungsi/kelas". `attitude_filter.py` sebaiknya dibaca sebagai *referensi arsitektur cara
menulis complementary filter yang bersih dan ter-unit-test* (pemisahan pure-logic dari
I/O — pola yang sama juga sudah dipraktikkan di `ros2_ws` sendiri via `qr_logic.py`/
`hook_logic.py`), bukan sebagai sumber kode untuk dipindah.

---

## 5. Antarmuka output — kontrak minimal untuk `stabilizer.py`

`stabilizer.py` hari ini membaca **`nav_msgs/Odometry` penuh** (`stabilizer.py:139`,
`create_subscription(Odometry, '/hydroships/odom', self.on_odom, 10)`) tapi di dalam
`on_odom` **hanya memakai** `pose.pose.position.z` dan `pose.pose.orientation` — TIDAK pernah
menyentuh `twist` (linear/angular), `pose.covariance`, `child_frame_id`, atau field lain dari
`Odometry`. Ini dikonfirmasi langsung dari `stabilizer.py:160-164` yang sudah dikutip §1.
`mission_fsm.py` yang memakai lebih banyak field (`x`, `y`, `vx`, `vy` — lihat P1-2 §7) berada
**di luar scope orientasi**, jadi tidak relevan untuk keputusan kontrak §5 ini (P1.2A tidak
menyentuh `mission_fsm.py`).

Opsi dievaluasi:

1. **Reuse `nav_msgs/Odometry`, isi hanya orientation+angular_velocity, kosongkan/nolkan
   position+linear velocity** — **DITOLAK.** Ini secara semantik menyesatkan: `Odometry` per
   definisi REP-105 membawa estimasi pose *dan* velocity relatif terhadap frame lokal;
   mempublish `position=(0,0,0)` palsu di topik bertipe `Odometry` berisiko dikonsumsi keliru
   oleh consumer masa depan (mis. jika `mission_fsm.py` suatu saat direfactor untuk membaca
   topik baru ini juga tanpa sadar field position-nya dummy). Juga tidak menyelesaikan
   masalah nyata — kalau tujuannya hanya orientasi, memaksakan bentuk `Odometry` menambah
   field yang harus dijaga-agar-tidak-disalahartikan tanpa manfaat.
2. **Publish `sensor_msgs/Imu` dengan `orientation` terisi (hasil fusi), `angular_velocity`
   diteruskan/difilter ringan, `linear_acceleration` diteruskan apa adanya** — **DIREKOMENDASIKAN.**
   Ini persis tujuan desain field `Imu.orientation` per definisi resmi `sensor_msgs/Imu`
   (satu-satunya pesan standar ROS yang secara eksplisit punya slot "orientasi hasil fusi
   sensor, bisa diisi estimator eksternal") — tidak ada field yang harus dikosongkan secara
   menyesatkan. Topik baru, mis. `/hydroships/imu/filtered` atau `/hydroships/attitude`
   (nama final di luar scope desain ini, hanya prinsip: **jangan menimpa `/hydroships/imu`
   mentah** supaya raw data sim tetap bisa diaudit terpisah dari hasil filter).
3. **Custom message** — **DITOLAK, tidak ada justifikasi.** `sensor_msgs/Imu` sudah mencakup
   seluruh field yang dibutuhkan (orientation quaternion + covariance, angular_velocity +
   covariance) tanpa kekurangan; menambah tipe pesan baru menambah dependency graph tanpa
   manfaat, bertentangan dengan prinsip topic-contract sederhana yang sudah dipraktikkan
   proyek ini (`ARCHITECTURE.md`, dikutip P1-2 §11 item 5).

**Rekomendasi konkret:** node baru (§11) subscribe `/hydroships/imu` (raw), publish
`sensor_msgs/Imu` baru pada topik terpisah dengan `orientation` terisi hasil complementary
filter (quaternion, dikonversi dari representasi internal §3.7), `orientation_covariance`
diisi estimasi kasar (bukan all-zero — REP-145: all-zero berarti "unknown", padahal
estimator ini justru TAHU rentang confidence-nya secara kasar, sebaiknya isi angka non-zero
walau approksimasi, supaya konsumen masa depan yang covariance-aware tidak salah baca
"unknown" jadi "sangat presisi"), `angular_velocity` diteruskan dari IMU mentah (opsional
low-pass ringan), `header.stamp` = stamp IMU mentah yang dipakai untuk update terakhir
(bukan stamp publish/wall-clock), `header.frame_id` = `imu_link` (konsisten sumber).
Perubahan `stabilizer.py` untuk memakai topik baru ini **secara eksplisit TIDAK termasuk**
dalam scope P1.2A (lihat §11) — didesain di sini supaya siap dipakai, tapi belum diwire.

---

## 6. Inisialisasi/reset — ringkasan

(Detail penuh di §3.4/§3.5, diringkas sesuai permintaan struktur dokumen.)

- Fase inisialisasi N-sampel-pertama (~25-50 sampel @ 50 Hz) mengasumsikan ROV level/diam
  saat estimator start → seed roll/pitch dari rata-rata accel, seed bias gyro dari rata-rata
  angular_velocity, seed yaw = 0 (arbitrer, tidak ada referensi absolut).
- Selama inisialisasi: tidak publish output valid (lihat health flag §7).
- Reset (jika diperlukan operator/manual): kembali ke fase inisialisasi penuh, bukan
  zero-output sesaat.
- Tidak ada mekanisme reset otomatis untuk yaw ke nilai "benar" — tidak ada sumber
  kebenaran yaw absolut di sistem ini sama sekali (§3.3).

---

## 7. Semantik kegagalan/kesehatan

Kontrak desain (tidak mengubah `stabilizer.py` sekarang, hanya spesifikasi):

| Kondisi | Perilaku estimator | Bagaimana downstream tahu |
|---|---|---|
| IMU belum pernah terima pesan (fase startup) | Tidak publish topik output sama sekali, atau publish dengan flag valid=false | Consumer harus treat "belum ada pesan di topik" = tidak valid — pola yang sudah familiar di codebase ini (`stabilizer.cur_z is None` dkk dicek sebelum dipakai, `stabilizer.py:229` `if self.enable_depth and self.cur_z is not None:`) |
| IMU dropout (tidak ada pesan baru > threshold, disarankan 0.5s mengikuti preseden watchdog `thruster_allocator`) | Berhenti update filter, set flag tidak-valid, TIDAK terus publish nilai lama seolah-olah masih fresh | Sama pola `_hook_fresh()` yang sudah ada di `mission_fsm.py` (dicatat P1-2 §6/§10 Finding #5 sebagai pola yang tim sudah tahu tapi belum diterapkan ke odom) — desain P1.2A mengadopsi pola yang sama, bukan pola baru |
| NaN/Inf pada field masuk | Sample dibuang, state filter dipertahankan di nilai valid terakhir, hitung sebagai "tidak ada update baru" untuk keperluan staleness | Sama seperti dropout di atas |
| Timestamp mundur/dt negatif | Sample dibuang atau dt diklem (§3.8) | Tidak mempengaruhi flag valid kecuali berturut-turut (baru jadi dropout jika tidak ada progres valid dalam window) |
| Recommended: publish flag kesehatan eksplisit | `Imu.orientation_covariance[0] = -1` (konvensi resmi REP-145 untuk "orientation tidak tersedia/tidak valid") saat estimator belum siap/stale, isi angka valid saat sehat | Ini memakai field yang SUDAH ADA di `sensor_msgs/Imu` (tidak perlu topik/field terpisah) — cara paling minimal dan standar untuk sinyal kesehatan tanpa menambah kontrak baru |

Catatan eksplisit: desain ini **tidak mengubah `stabilizer.py` untuk mengecek flag ini
sekarang** — itu pekerjaan integrasi terpisah (P1-2 §11 item 5, di luar scope P1.2A). Yang
didesain di sini hanya **kontrak** supaya integrasi nanti punya sinyal yang jelas untuk
dibaca.

---

## 8. Rencana verifikasi simulasi (desain, tidak dijalankan)

Tujuan: membedakan tiga kegagalan berbeda yang semuanya bisa "terlihat berhasil" secara
dangkal:

1. **(A) Estimator berfungsi** (filter benar-benar menghasilkan attitude yang masuk akal
   dari IMU).
2. **(B) Controller tetap diam-diam memakai ground-truth** meski estimator sudah ada (bug
   integrasi paling berbahaya: estimator jalan tapi tidak benar-benar dipakai).
3. **(C) Estimator sekadar echo ground-truth** (bug lebih halus: alih-alih memfusi IMU
   independen, implementasi salah/sengaja mengambil shortcut dari `/hydroships/odom`).

### Skenario uji

**Setup:** jalankan node estimator baru (§11) paralel dengan sim yang sudah berjalan (TIDAK
mengganti plugin odom Gazebo — `/hydroships/odom` tetap hidup sebagai referensi
pembanding, bukan dimatikan). Estimator subscribe HANYA `/hydroships/imu` (dan tidak
subscribe `/hydroships/odom` sama sekali — ini persyaratan implementasi yang harus dicek
lewat code review, bukan cuma dites: jika kode estimator secara fisik tidak mengimpor/
subscribe `Odometry`, kegagalan (C) terstruktur tidak mungkin terjadi by construction).

**Uji 1 — Rotasi terskrip, bandingkan lintasan (menguji A & C sekaligus):**
- Jalankan sim, kirim perintah rotasi terkontrol via `/hydroships/manual/cmd` atau
  `/hydroships/cmd_vel` (mis. rotasi yaw konstan 0.2 rad/s selama 10 detik, lalu roll/pitch
  disturbance kecil).
- Log paralel: `/hydroships/odom` (ground-truth, sebagai referensi HANYA untuk perbandingan
  post-hoc, bukan input estimator) vs topik output estimator (§5).
- **Metrik konvergensi:** setelah fase inisialisasi (~1 detik), selisih roll/pitch antara
  estimator dan ground-truth harus turun ke bawah ambang (disarankan ±2° untuk sim
  tanpa-noise berdasarkan §3, catat sebagai target awal yang perlu dikalibrasi ulang begitu
  noise model IMU riil diketahui — lihat §10) dan tetap di bawah ambang itu selama gerakan
  quasi-static (rotasi lambat/konstan, bukan akselerasi mendadak).
- **Pass/fail roll/pitch:** RMS error < 2° selama window steady-state (bukan saat transien
  awal fase inisialisasi).
- **Fail-mode (C) terdeteksi jika:** selisih ≈ 0 dengan presisi jauh lebih tinggi dari yang
  masuk akal untuk filter berbasis accel+gyro (mis. error < 0.01° konsisten sepanjang waktu,
  termasuk saat akselerasi tinggi di mana accel-based roll/pitch SEHARUSNYA menyimpang dari
  ground-truth per keterbatasan fisik §3.1) — itu indikasi kuat implementasi diam-diam
  membaca odom, bukan genuinely fusing IMU.

**Uji 2 — Injeksi bias/freeze ground-truth, verifikasi independensi (menguji C secara
definitif):**
- Selagi sim jalan dan estimator aktif, buat topik `/hydroships/odom` berhenti berubah
  (mis. pause sim fisika sesaat via layanan Gazebo, atau — kalau tidak memungkinkan tanpa
  ubah kode — matikan langganan bridge odom sesaat) SEMENTARA `/hydroships/imu` terus
  mengalir dengan gerakan nyata berlanjut (rotasi terus dipertahankan).
- **Kriteria pass:** topik output estimator HARUS terus berubah mengikuti rotasi nyata
  meski `/hydroships/odom` beku — ini bukti definitif estimator tidak bergantung odom sama
  sekali, karena secara harfiah odom sedang tidak menghasilkan info baru selama window ini.
- **Kriteria fail (C):** output estimator ikut beku/berhenti berubah bersamaan dengan
  freeze odom → estimator ternyata (langsung atau tidak langsung) bergantung pada odom.

**Uji 3 — Cek non-integrasi (menguji B, tidak butuh sim tambahan):**
- Ini murni code-review/static check, bukan uji runtime: konfirmasi `stabilizer.py` (dan
  `mission_fsm.py`) **masih** subscribe `/hydroships/odom` untuk orientasi setelah estimator
  ada (memang seharusnya demikian di P1.2A — lihat §11, stabilizer TIDAK diubah) — jadi (B)
  bukan "kegagalan" pada tahap P1.2A ini, melainkan **status yang harus didokumentasikan
  eksplisit dan diverifikasi TIDAK berubah diam-diam**: pastikan tidak ada PR/commit lain
  yang secara tidak sengaja mengubah `stabilizer.py` untuk baca topik baru tanpa keputusan
  eksplisit terpisah. Untuk P1.3+/integrasi nanti (di luar P1.2A), uji (B) yang sebenarnya
  adalah: setelah `stabilizer.py` diubah untuk baca topik estimator, ulangi Uji 1/2 dan
  verifikasi `cmd_vel` yang dihasilkan `stabilizer` berubah berdasarkan estimator, bukan odom
  (mis. matikan odom plugin sepenuhnya dan verifikasi depth/heading-hold TETAP berfungsi).

**Ekspektasi drift yaw (tanpa magnetometer, §3.3):** karena tidak ada uji ini yang bisa
memberi angka pasti tanpa menjalankan sim sungguhan dengan noise model IMU yang belum
diketahui (§10), rencana verifikasi HANYA bisa menetapkan **metodologi pengukuran drift
rate**, bukan angka target pasti: jalankan ROV level/diam selama durasi panjang (mis. 5-10
menit sim-time) dengan estimator aktif, ukur `|yaw_estimator - yaw_ground_truth|` di akhir
window, bagi dengan durasi menit → deg/menit. **Angka pass/fail konkret harus ditetapkan
setelah hasil uji pertama tersedia** (tidak bisa ditentukan a priori tanpa data — dicatat
sebagai unresolved di §10), tapi metodologi pengukurannya sudah lengkap di sini untuk
langsung dieksekusi implementer.

---

## 9. Penilaian akhir referensi GUI-ROV

(Perluasan §4 ke format verdict final, sesuai permintaan struktur.)

**Verdict:** `GUI-ROV/attitude_filter.py` adalah **referensi desain algoritma parsial**,
bukan kandidat port kode. Yang dibawa ke P1.2A: pola struktur complementary filter (state
update rule), pola wrap-around yaw, pola dt clamping, dan pola pemisahan pure-logic dari I/O
(`AttitudeFilter` class terisolasi dari `rov_agent.py`/pymavlink — pola arsitektur yang
SUDAH konsisten dengan konvensi `ros2_ws` sendiri, jadi tidak perlu "diajarkan", cukup
dikonfirmasi selaras). Yang TIDAK dibawa: asumsi input sudah berupa attitude ter-EKF dari
firmware eksternal (bertentangan dengan Keputusan A), nilai gain `alpha`/`EMA_ALPHA_*`
spesifik (perlu tuning ulang untuk rate 50 Hz IMU sim + karakteristik noise berbeda), dan
lapisan EMA tambahan (berisiko menambah lag ke loop kendali PID tanpa manfaat yang
terbukti perlu di sim tanpa-noise).

---

## 10. Asumsi belum terselesaikan

1. **Apakah gz-sim-imu-system menyuntik noise default** di luar apa yang ditulis eksplisit
   di blok `<sensor type="imu">` (`xacro:246-252`, yang tidak berisi elemen `<noise>` sama
   sekali) — tidak bisa dipastikan dari membaca file statis, butuh `ros2 topic echo
   /hydroships/imu` saat sim berjalan (di luar scope read-only dokumen ini).
2. **Apakah `Imu.orientation` dari `/hydroships/imu` benar-benar kosong/identity** atau
   terisi sesuatu oleh gz-sim-imu-system secara default — desain di §2/§3 mengasumsikan
   TIDAK dipakai (aman secara desain either way), tapi nilai pastinya butuh verifikasi
   runtime, bukan diklaim di sini.
3. **Konvensi tepat covariance array** yang dipublish gz-sim-imu-system (all-zero vs
   `orientation_covariance[0]=-1` vs nilai lain) — mempengaruhi apakah field kesehatan §7
   perlu diisi manual oleh estimator baru dari nol (kemungkinan besar ya, karena raw IMU sim
   kemungkinan tidak mengisi ini secara berguna) atau bisa diwarisi.
4. **Part number/spesifikasi IMU hardware fisik** — `HARDWARE.md` baris IMU (`HARDWARE.md`
   §2, baris "IMU (bagian dari Pixhawk)") hanya menyebut IMU sebagai "bagian dari Pixhawk",
   tidak ada part number spesifik (mis. MPU6000/ICM-20689, apa pun yang tertanam di board
   Pixhawk yang dipakai tim) atau datasheet noise/bias — tanpa ini, nilai gain filter final
   (`alpha` complementary, threshold staleness presisi) tidak bisa di-tuning untuk hardware
   asli, hanya untuk sim. Ini juga terkait §13 item 2 P1-2 yang masih pertanyaan
   project-owner (apakah IMU dibaca via Pixhawk pass-through atau standalone).
5. **Rate loop node baru** (berapa Hz node ROS2 estimator ini idealnya jalan) — didesain
   event-driven per pesan IMU masuk (§3.6, mengikuti update_rate 50 Hz IMU), tapi apakah ini
   realistis di Raspberry Pi 4B (disebut sebagai concern performa umum di `HARDWARE.md` §3
   item 6, belum divalidasi untuk node manapun) tidak bisa dipastikan tanpa uji hardware.
6. **Angka pass/fail drift yaw konkret** (deg/menit) untuk Uji verifikasi §8 — metodologi
   sudah lengkap, angka target belum bisa ditetapkan tanpa data pertama, dicatat eksplisit
   di §8.
7. **Nama topik output final** (§5 menyebut `/hydroships/imu/filtered` atau
   `/hydroships/attitude` sebagai contoh, bukan keputusan final) — nama pasti sebaiknya
   diputuskan saat implementasi, sejalan konvensi penamaan topic lain di
   `docs/ARCHITECTURE.md`/`docs/CONFIG_REFERENCE.md` yang tidak dibaca ulang detail di sini
   khusus untuk konvensi penamaan.

---

## 11. Ruang lingkup implementasi (untuk ticket P1.2A masa depan)

**AKAN disentuh:**
- Satu node ROS2 baru, mis. `attitude_estimator.py`, didaftarkan di
  `src/hydroships_control/setup.py` (mengikuti pola registrasi node lain, sesuai
  `CLAUDE.md`: "Node entry points are registered in `src/hydroships_control/setup.py`; add
  new nodes there").
- Satu modul pure-logic baru, mis. `attitude_filter_logic.py`, berisi kelas/fungsi
  complementary filter (§3) yang unit-testable tanpa ROS (mengikuti pola
  `qr_logic.py`/`hook_logic.py`/`pid.py` — pemisahan node ROS2 tipis dari logika murni,
  sesuai konvensi wajib proyek ini di `CLAUDE.md`).
- Reuse `wrap_to_pi` dari `hydroships_control/pid.py` yang sudah ada (§4) — bukan
  implementasi ulang.
- Subscribe `/hydroships/imu` (`sensor_msgs/msg/Imu`, sudah ada, tinggal konsumsi).
- Publish topik baru (nama final TBD, §10 item 7) bertipe `sensor_msgs/msg/Imu` dengan
  `orientation`, `orientation_covariance`, `angular_velocity` terisi sesuai §5.
- Unit test murni untuk `attitude_filter_logic.py` (deret IMU sintetis, termasuk kasus
  drop/NaN/dt-gap dari §3.8) — tidak butuh ROS/sim untuk dijalankan, konsisten pola test
  yang sudah ada (`pytest` per `CLAUDE.md` command reference).

**TIDAK akan disentuh (eksplisit):**
- `stabilizer.py` — tetap membaca `/hydroships/odom` seperti sekarang; mengganti sumber
  input stabilizer adalah pekerjaan integrasi terpisah (P1-2 §11 item 5), bukan bagian
  P1.2A.
- `mission_fsm.py` — tidak menyentuh sama sekali; FSM juga tidak butuh perubahan untuk
  ticket ini (localization x/y/vx/vy di luar scope P1.2A sama sekali, lihat §12).
- `gui_bridge.py`/`gui_bridge_logic.py` — tidak disentuh; per P1-2 §8, gui_bridge otomatis
  ikut benar begitu ada sumber baru yang dipakai node lain, tidak butuh perubahan langsung.
- `thruster_allocator.py`/`allocation.py` — tidak relevan dengan estimasi orientasi.
- `qr_logic.py`, `hook_logic.py`, apa pun terkait persepsi visual — tidak relevan.
- Semua yang berstatus P0 frozen.
- `GUI-ROV` repo — reference-only, tidak diubah (§4/§9).
- Konfigurasi `bridge.yaml`/xacro IMU sensor — topik `/hydroships/imu` sudah ada dan cukup
  untuk P1.2A; tidak perlu mengubah `update_rate` atau menambah `<noise>` di sim (kecuali
  nanti diputuskan perlu untuk kebutuhan uji tertentu, di luar scope minimal ini).
- `config/gains.yaml` — gain complementary filter yang baru (`alpha`, dsb.) sebaiknya jadi
  parameter node baru sendiri (declare_parameter, pola sama seperti `stabilizer.py`), bukan
  ditambahkan ke `gains.yaml` yang sudah didedikasikan untuk gain PID.

---

## Verdict persetujuan

**Approved with conditions.**

Alasan: desain filter komplementer roll/pitch dari accel+gyro (§3.1-3.2) solid dan bisa
diimplementasikan segera — semua input yang dibutuhkan sudah tersedia di sim
(`/hydroships/imu`, 50 Hz, tanpa dependency baru) tanpa perlu menunggu keputusan
project-owner apa pun. Namun ada dua kondisi yang harus dipenuhi sebelum implementasi
final di-merge (bukan blocker untuk MULAI mengerjakan):

1. **Verifikasi runtime item §10.1/§10.2** (isi aktual `Imu.orientation` dan noise model
   gz-sim-imu-system) — perlu dilakukan sekali di awal implementasi (`ros2 topic echo
   /hydroships/imu` saat sim jalan) untuk mengonfirmasi asumsi desain §2 sebelum kode
   filter ditulis final, supaya tidak salah asumsi tentang field mana yang benar-benar
   kosong.
2. **Yaw drift-only harus dikomunikasikan eksplisit ke project-owner sebagai batasan
   permanen** (§3.3) sebelum implementasi dianggap "selesai" untuk keperluan kompetisi —
   bukan sesuatu yang bisa diperbaiki nanti tanpa magnetometer/sensor tambahan.

Tugas implementasi terkecil yang bisa langsung dikerjakan: **node baru
`attitude_estimator.py` + modul pure-logic `attitude_filter_logic.py`, subscribe
`/hydroships/imu`, publish `sensor_msgs/msg/Imu` dengan `orientation` terisi hasil
complementary filter (accel roll/pitch + gyro integration, quaternion internal) pada topik
baru — TANPA mengubah `stabilizer.py`, `mission_fsm.py`, `gui_bridge.py`, atau
`thruster_allocator`/`allocation.py` sama sekali.**

**Yang tetap terblokir setelah estimator orientasi ini selesai** (tidak diselesaikan oleh
P1.2A, langsung mewarisi dari P1-2):
- **Localization x/y absolut** — masih nol solusi, masih butuh keputusan project-owner
  (P1-2 §13 item 1) apakah cukup visual-servo relatif atau butuh DVL/USBL baru.
- **Kecepatan linear vx/vy** — masih ground-truth sim, tidak ada sensor kandidat
  (P1-2 §5/§6).
- **`/hydroships/payload_pose` sim-only leak** — masih dependency struktural `mission_fsm`
  untuk mencapai `APPROACH_QR`/`GRAB`, tidak tersentuh sama sekali oleh estimator orientasi
  ini (P1-2 §7 item 1, §13 item 4).
- **Integrasi estimator ke `stabilizer.py`/`mission_fsm.py`** — estimator ini akan EXIST
  tapi TIDAK dipakai controller mana pun sampai ada pekerjaan integrasi terpisah
  (P1-2 §11 item 5) — ini penting ditekankan supaya tidak ada asumsi keliru bahwa
  menyelesaikan P1.2A berarti ROV "sudah pakai IMU" untuk kendali sungguhan.
