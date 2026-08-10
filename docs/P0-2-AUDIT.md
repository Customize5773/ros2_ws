# P0-2.0 AUDIT — APPROACH_QR (KKI 2026)

Dokumen ini adalah **audit statis, read-only**. Tidak ada kode yang diubah, tidak ada
simulator yang dijalankan, tidak ada parameter yang di-tuning dalam pembuatan dokumen ini.
Tujuannya satu: memetakan apa yang **sudah** diimplementasikan untuk state `APPROACH_QR`,
sebelum eksperimen apa pun dimulai.

Baseline P0-1 dibekukan pada tag **`p0-1-baseline`** (lihat
[`P0-1-BASELINE.md`](P0-1-BASELINE.md)), yang sudah menandai `APPROACH_QR / GRAB / NAV_WALL`
sebagai **`OPEN`** — bukan karena belum jalan, tapi karena "berjalan 60 s tanpa ABORT" pada
P0-1e bukan acceptance evidence. Dokumen ini adalah titik mulai P0-2.

## 0. Pertanyaan yang harus dijawab P0-2

> Apakah ROV dapat melakukan `APPROACH_QR` secara *repeatable*, mencapai pose/posisi relatif
> QR yang diperlukan, dalam batas waktu, dan tanpa divergensi?

Audit ini tidak menjawab pertanyaan itu — audit ini memetakan **apa yang ada** untuk
menentukan eksperimen minimum yang bisa membedakan *perception problem* vs
*frame/geometry problem* vs *controller/servo problem* vs *FSM/timing problem*.

---

## 1. Inventaris implementasi saat ini

Semua referensi baris di `src/hydroships_control/hydroships_control/mission_fsm.py` kecuali
disebutkan lain.

### 1.1 FSM — entry/exit

- Kelas `MissionFSM` (L90), state didefinisikan di `St(Enum)` (L84-87):
  `IDLE, DIVE, APPROACH_QR, GRAB, NAV_WALL, HANG, SURFACE, WAIT_TRIGGER, APPROACH_HOOK,
  AUTO_RELEASE, DONE, ABORT`.
- Dispatch: `_tick()` (L452-465), timer 10 Hz (`create_timer(0.1, self._tick)`, L295),
  memanggil `_st_<state>()` sesuai state aktif.
- **Entry ke `APPROACH_QR`**: dari `_st_dive()` (L468-486). Syarat (L480): depth tercapai
  (`depth_ok`) **dan** `self.payload_pose is not None` (pose ter-latch dari
  `/hydroships/payload_pose` harus sudah datang). `_to(St.APPROACH_QR)` (L484) mereset
  `_wall_scored` dan `_approach_recovered` (L313-317) — penting karena misi berulang per
  payload (`AUTO_RELEASE → DIVE → APPROACH_QR` lagi).
- **Exit — sukses (QR terbaca + centered)**: L590-601, jika `_wall_scored` **dan**
  (`centered` secara visual-servo **atau** `dist < approach_tol`) → `_to(St.GRAB)`.
- **Exit — sukses fallback (QR tak pernah terbaca, tapi XY payload tercapai)**: L603-615,
  jika `dist < approach_tol` dan QR belum ter-score → assign wall dari
  `_wall_sequence` (round-robin fallback) → `_to(St.GRAB)`.
- **Exit — timeout/abort**: L617-618, `if self._elapsed() > self.T['scan']: ... _to(St.ABORT)`.
  `T['scan']` = param `t_scan`, default **45.0 s** (L133).
- **Mid-state recovery (bukan abort)**: L522-531, jika QR tak terlihat lebih lama dari
  `t_nav_qr` (default **30.0 s**, L151), FSM menaikkan `depth_target` +0.10 m sekali
  (idempotent via `_approach_recovered`) untuk melebarkan FOV kamera bawah — secara eksplisit
  bukan abort (komentar L149-150, L522-524).
- Gerbang hulu: `_st_dive()` sendiri punya timeout `t_dive` default 20.0 s (L133) yang bisa
  ABORT sebelum pernah masuk `APPROACH_QR` (L485-486).

### 1.2 QR detection / visual input

- Node `qr_detector.py` (`QRDetector`, node name `qr_detector`, L47).
- **Bukan mock** — detektor OpenCV nyata, `cv2.QRCodeDetector()` (L73) + pipeline
  preprocessing custom di `qr_logic.py` (`robust_decode`, L93-117: grayscale+CLAHE →
  adaptive threshold → Otsu → varian upscale). Mendekode citra render Gazebo asli
  (`sensor_msgs/Image`), tanpa `cv_bridge` (`image_util.py`).
- Input: `/hydroships/camera_bottom/image_raw`, `/hydroships/camera_front/image_raw`
  (param `image_topics`, `qr_detector.py:50-52`) + `camera_info` masing-masing (L67-71).
  Intrinsics `K` disimpan tapi **sim-only, tidak dikalibrasi hardware**, dan tidak dipakai
  untuk estimasi jarak (komentar L26-29, L62-65).
- Output:
  - `/hydroships/qr_result` (`std_msgs/String`) — huruf wall A/B/C/D via `parse_wall()`
    (`qr_logic.py:41-46`), atau string mentah jika tak match (`qr_detector.py:146-149`).
  - `/hydroships/qr_offset` (`geometry_msgs/PointStamped`) — via `_publish_offset()`
    (L154-162) memakai `offset_from_points()` (`qr_logic.py:120-134`):
    `x=ex` (offset horizontal ternormalisasi [-1..1], + = QR di kanan center),
    `y=ey` (offset vertikal ternormalisasi, + = QR di bawah center),
    `z=size` (fraksi ukuran QR di frame, proxy jarak),
    `header.frame_id` = `camera_bottom_link` / `camera_front_link` (string tag sumber kamera).
  - **Tidak ada pose 3D metrik atau id/corner array** yang dipublish — hanya offset piksel
    ternormalisasi + size + string huruf wall.
- Rate limit: param `max_rate` default 5.0 Hz (L53, L117).

### 1.3 Frame transform

- **Tidak ada tf2/TransformListener/StaticTransform di mana pun dalam `src/`** (dicek via
  grep `tf2|TransformListener|lookup_transform|tf_broadcaster|StaticTransform` — nihil).
  `camera_bottom_link` di atas hanyalah string tag untuk multiplexing topic, **bukan** frame
  TF yang dilacak.
- Transformasi offset QR → koordinat dunia dilakukan **manual trigonometri** di
  `mission_fsm.py`:
  - `_st_approach_qr()` L544-567: offset `(ex, ey)` dari `/hydroships/qr_offset` dirotasi ke
    world pakai **yaw yang di-lock** sekali saat entry state (`self._locked_yaw`, L516-517),
    via rotasi manual (L565-567):
    `tx += body_dx*cos - body_dy*sin; ty += body_dx*sin + body_dy*cos`.
  - Koreksi offset gripper body-frame diterapkan lebih dulu (L552-554): target dikurangi
    `gripper_base_dx * cos/sin(locked_yaw)` karena gripper 0.18 m di depan `base_link`
    (sesuai `hydroships.urdf.xacro`, komentar L159-161).
  - Koreksi geometri kamera→gripper vertikal dihitung `qr_ey_target()` (L55-78), memakai
    konstanta geometri tetap: `cam_gripper_dx=0.16` (L164), `qr_floor_z=-0.894` (L166, cocok
    dengan `payload_spawner.py:88` default `payload_z`), `cam_bottom_dz=0.18` (L167),
    `cam_vfov_half_tan=0.6293` (L169, ≈tan(setengah hFOV 80° pada rasio 4:3)),
    `ey_target_max=0.8` (L171).
  - Odometri (`self.x`, `self.y`, `self.yaw`) langsung dari `/hydroships/odom`
    (`_on_odom()`, L396-401, via `yaw_from_quaternion()` L45-48) — diasumsikan frame
    `odom`/world dari plugin Gazebo, **tanpa verifikasi tree TF eksplisit di kode**.
- **NOT FOUND**: definisi tree TF, validasi rantai frame `robot_state_publisher`, atau lookup
  transform statis `camera_bottom_link → base_link`.

### 1.4 Approach controller

- `_goto_xy()` (L359-382) — PD holonomik (proporsional posisi + damping kecepatan), dipanggil
  dari `_st_approach_qr()` di L569 (`dist = self._goto_xy(tx, ty)`).
  - Dekomposisi body-frame (L375-380):
    `surge = approach_kp*bx - approach_kd*self.vx`,
    `sway  = approach_kp*by - approach_kd*self.vy`.
  - Diclamp `±fm` (L378), `fm = approach_fmax`, ditaper mendekati target (radius slow-down
    1.0 m, `min_fmax_frac=0.05`, L370-374).
  - Output dipublish via `_set_surge(fx, fy)` (L325-327) → `Twist` di `/hydroships/manual/cmd`.
- Gain/limit (param, deklarasi L122-125, cache L184-187):
  - `approach_kp = 90.0` (N/m)
  - `approach_kd = 140.0` (N/(m/s))
  - `approach_fmax = 16.0` (N)
  - `approach_tol = 0.06` (m) — radius konvergensi
- Lapisan visual-servo di atas PD posisi: L559-567 menggeser target XY `(tx, ty)` berdasarkan
  error offset piksel QR (`qr_off`), gain `qr_servo_gain = 0.15` (m per unit offset
  ternormalisasi, L154), sign flip `qr_servo_sign = 1.0` (L157, **dicatat sebagai bisa perlu
  dibalik jika polaritas feedback salah** — komentar L155-156). Servo hanya aktif jika
  `off_fresh` (umur pesan offset `< qr_max_age`=1.5s) dan `dist_raw < 0.3` m (gerbang
  `servoing`, L558).
- Catatan: `_goto_xy_yaw_first()` (L329-357) adalah controller PD non-holonomik terpisah,
  dipakai `_st_nav_wall`, **bukan** oleh `APPROACH_QR`.

### 1.5 Target pose & tolerance

- Target XY: `(self.payload_x, self.payload_y)` (L552), sumber dinamis dari
  `/hydroships/payload_pose` (`_on_payload_pose`, L434-440, QoS latched/TRANSIENT_LOCAL,
  L231-236), dengan fallback param statis `payload_x=0.4`, `payload_y=0.0` (L105-106) hanya
  dipakai sampai pesan latched datang.
- Target depth: `scan_depth = 0.30` m (L121; derivasi dikomentari panjang L107-120 — dipilih
  agar QR tetap penuh di frame sambil target `ey` tetap di dalam `ey_target_max`).
- Target centering visual: **bukan** `ey=0`, tapi `ey_target` dihitung `qr_ey_target()`
  (L55-78) sebagai fungsi dari depth target saat ini — mengakomodasi offset gripper 0.16 m di
  depan kamera bawah. Target `ex` = 0 (centering lateral).
- Threshold konvergensi:
  - Kondisi "centered" visual-servo (L593-595): `abs(qr_off.x) < qr_center_tol` **dan**
    `abs(qr_off.y - ey_target) < qr_center_tol`, `qr_center_tol = 0.12` (unit ternormalisasi,
    param L153).
  - Toleransi fallback XY: `dist < approach_tol` = 0.06 m (L125/187).
  - Salah satu kondisi terpenuhi → transisi ke `GRAB` (L596).
- Metrik alignment gripper (diagnostik saja, tidak menggerbang transisi):
  `_gripper_align_txt()` (L488-503) menghitung `gripper_err` = jarak posisi gripper aktual
  (`base_link + gripper_base_dx` dirotasi yaw) ke `(payload_x, payload_y)`, di-log saat
  transisi (L600) tapi tidak menggerbang transisi itu sendiri.

### 1.6 Timeout / abort

- Timeout abort utama: param `t_scan`, default **45.0 s** (L133), dicek L617
  (`if self._elapsed() > self.T['scan']: ... _to(St.ABORT)`). Saat timeout, FSM langsung ke
  `St.ABORT` (terminal — surge di-nol-kan, L459-461, tidak ada handler lanjut). **Tidak ada
  retry** setelah `ABORT` — terminal untuk seluruh run misi.
- Timer "recovery" sekunder (bukan abort): param `t_nav_qr`, default **30.0 s** (L151),
  dicek L525-531 — jika QR tak terbaca lebih lama dari ini, FSM menaikkan `depth_target`
  +0.10 m sekali, berharap menangkap QR sebelum deadline `t_scan` yang lebih keras. Ini
  mitigasi, bukan exit state.
- Timeout hulu (`DIVE`): `t_dive` default 20.0 s (L133) — jika `DIVE` tak mencapai
  `scan_depth` dalam jendela ini, abort sebelum pernah masuk `APPROACH_QR` (L485-486).
- Per `docs/PERFORMANCE.md:20`, target durasi "scan QR" yang diharapkan: `< 45s (t_scan
  default)`.

### 1.7 Observability

- Log transisi FSM: `_to()` (L307) — `'[FSM] %s -> %s'` via `get_logger().info` setiap
  transisi.
- Log debug `APPROACH_QR` (L570-579), throttle ~2 Hz
  (`int(self._elapsed()*2) % 20 == 0`): `dist`, `x`, `y`, `yaw`, `target=(tx,ty)`, offset QR
  `ex/ey`, `ey_target`, `h_cam` terhitung, flag `servo` (0/1), huruf wall `qr`. Tag log:
  `'APPROACH_QR dbg: ...'`.
- Topic yang tersedia untuk instrumentasi:
  - `/hydroships/qr_result` (String) — huruf wall / raw decode.
  - `/hydroships/qr_offset` (PointStamped) — ex/ey/size + frame_id.
  - `/hydroships/odom` (Odometry) — posisi/yaw/velocity.
  - `/hydroships/depth` (Float64) — depth terukur.
  - `/hydroships/setpoint/depth`, `/hydroships/setpoint/heading` (Float64) — setpoint FSM.
  - `/hydroships/manual/cmd` (Twist) — gaya horizontal (Fx/Fy) hasil `_set_surge`.
  - `/hydroships/payload_pose` (PointStamped, latched) — pose ground-truth dari
    `payload_spawner.py`.
  - `/hydroships/camera_bottom/image_raw`, `/hydroships/camera_bottom/camera_info` — citra
    mentah/intrinsics.
- Param runtime yang bisa diinspeksi/diubah: semua `approach_*`, `qr_*`, `t_scan`,
  `t_nav_qr`, `cam_*`, `ey_target_max` adalah param ROS2 (deklarasi L93-171), via
  `ros2 param get/set/dump mission_fsm`.
- **NOT FOUND**: topik diagnostik khusus "target pose error" atau "QR detection status" —
  harus diturunkan dari kombinasi `/hydroships/qr_offset` + `/hydroships/odom` + log debug
  (parsing teks), atau dari perluasan pola subscriber `tools/p0-experiments/recorder.py`
  (yang saat ini **tidak** subscribe ke `qr_result`/`qr_offset`).

### 1.8 Test/eval yang sudah ada

- Unit test logika murni (tanpa ROS/sim):
  - `src/hydroships_control/test/test_qr_ey_target.py` — menguji `qr_ey_target()`, mencakup
    pemilihan `scan_depth` (0.30 vs 0.46 lama), clamping, konvensi tanda, konstanta FOV.
    Docstring (L1-9) mendokumentasikan bahwa test ini mendasari keputusan `scan_depth=0.30`.
  - `src/hydroships_control/test/test_qr_logic.py` — menguji `parse_wall`, `robust_decode`,
    `offset_from_points`, `_to_gray`, `_candidates` di `qr_logic.py`, termasuk
    `test_robust_decodes_real_sim_frame` (L108) dan
    `test_robust_recovers_degraded_where_raw_fails` (L85) — memvalidasi logika detektor yang
    memberi makan `/hydroships/qr_result`/`qr_offset`, **tapi tidak menguji state FSM itu
    sendiri**.
- Script evidence P0-1 (`tools/p0-experiments/`) — **dikonfirmasi tidak mencakup
  `APPROACH_QR`**:
  - `driver.py` — rig thrust-schedule open-loop untuk P0-1d (world `pool_empty`, tanpa
    FSM/stabilizer/allocator berjalan) — tidak relevan untuk `APPROACH_QR` (tak ada FSM sama
    sekali dalam loop).
  - `recorder.py` (perekam closed-loop P0-1e) — hanya subscribe `/clock`,
    `/hydroships/odom`, `/hydroships/cmd_vel`, `/hydroships/setpoint/depth`,
    `/hydroships/depth`, `/hydroships/thruster_N/thrust` (L34-41) — **tidak** subscribe
    `/hydroships/qr_result` atau `/hydroships/qr_offset`, mengkonfirmasi rekaman ini dibangun
    untuk regresi DIVE, bukan instrumentasi `APPROACH_QR`.
  - `run_mission.sh`, `gate_mission.sh`, `reduce_mission.py`, `reduce_openloop.py`,
    `trim_audit.py` — tidak ada referensi spesifik state FSM `APPROACH_QR` (dicek via grep).
  - Sesuai `docs/P0-1-BASELINE.md` L24-26, run P0-1e memang membiarkan FSM lanjut lewat
    `APPROACH_QR`/`GRAB`/`NAV_WALL` hingga `WAIT_TRIGGER` dalam jendela 60 s tanpa ABORT —
    tetapi baseline itu sendiri secara eksplisit menyangkal ini sebagai acceptance evidence
    untuk ketiga state tersebut.
- `docs/CHANGELOG.md` (L150-343, 514) mendokumentasikan beberapa perbaikan/investigasi
  historis khusus di `APPROACH_QR` (reliabilitas baca QR, wiring offset gripper, penambahan
  `t_nav_qr`) — konteks berguna, bukan script test/eval otomatis.

---

## 2. Acceptance matrix (semua OPEN pada penulisan awal; dua baris diperbarui, lihat catatan di bawah tabel)

| Pertanyaan | Status | Apa yang perlu diukur | Sumber data yang tersedia |
|---|---|---|---|
| QR dapat dideteksi secara konsisten? | **OPEN** | Rate deteksi vs jarak/sudut/pencahayaan; berapa lama tanpa deteksi dalam run tipikal | `/hydroships/qr_result`, `/hydroships/qr_offset` (perlu direkam — `recorder.py` belum subscribe) |
| Transform QR → frame ROV benar? | **OPEN** | Konsistensi tanda (`qr_servo_sign`) dan skala (`qr_servo_gain`) antara offset piksel dan pergerakan aktual ROV; tidak ada tf2 untuk divalidasi silang | Perbandingan `/hydroships/qr_offset` vs delta `/hydroships/odom` per tick |
| Error relatif berkurang? | **OPEN** | Tren `dist`/`qr_off` sepanjang waktu dalam satu run | Log debug `APPROACH_QR dbg:` (L570-579) atau turunan dari topic di atas |
| ROV konvergen ke acceptance region? | **FAIL** (docs/P0-2-4-RESULTS.md §4-5) | Apakah `dist < approach_tol` atau kondisi `centered` tercapai sebelum `t_scan` | Diukur: 5/17 run (29%) entered+held band dengan dwell, stopping rule §6 P0-2-4-SPEC.md terpenuhi |
| Tidak oscillatory/divergent? | **PASS** (docs/P0-2-4-RESULTS.md §4) | Variansi/overshoot posisi & gaya `surge`/`sway` mendekati target | Diukur: 0/17 run diverged, overshoot=0 di seluruh run yang sempat masuk tolerance |
| Selesai sebelum timeout? | **OPEN** | Distribusi waktu-ke-GRAB vs `t_scan`=45s across run | Log transisi + timestamp |
| Repeatable pada beberapa initial condition? | **OPEN** | Multi-run dengan initial pose/spawn payload bervariasi | Perlu rig baru (belum ada — `recorder.py`/`driver.py` P0-1 tidak untuk ini) |
| FSM keluar `APPROACH_QR` dengan benar? | **OPEN** | Apakah exit selalu lewat jalur "QR-scored + centered" (bukan fallback XY-only, yang bisa lolos GRAB tanpa validasi persepsi) | Log `_wall_scored` state + jalur exit L590-601 vs L603-615 |

Baris ini seluruhnya **OPEN** pada penulisan awal dokumen (belum ada eksperimen yang
dijalankan). Dua baris ("ROV konvergen ke acceptance region?", "Tidak oscillatory/divergent?")
diperbarui kemudian dengan evidence dari `docs/P0-2-4-RESULTS.md` — baris lain masih OPEN.

---

## 3. Risiko yang teridentifikasi dari kode (belum diperbaiki, hanya dicatat)

1. **Polaritas visual-servo belum divalidasi.** `qr_servo_sign = 1.0` (L157) secara eksplisit
   dikomentari sebagai kandidat yang mungkin perlu dibalik jika feedback salah arah
   (L155-156) — ini belum pernah diuji secara closed-loop khusus untuk `APPROACH_QR`.
2. **Dua jalur sukses berbeda risiko.** Exit ke `GRAB` bisa terjadi tanpa QR pernah terbaca
   sama sekali (fallback L603-615, murni berbasis jarak XY ke `payload_pose` ground-truth).
   Ini berarti P0-1e yang "lolos ke WAIT_TRIGGER" bisa jadi tidak pernah benar-benar
   memvalidasi rantai persepsi QR — konsisten dengan alasan baseline P0-1 menahan klaim itu.
3. **Tidak ada TF tree.** Transform kamera→body sepenuhnya manual/hard-coded dengan konstanta
   geometri tetap (§1.3). Jika geometri fisik robot berubah (mis. re-kalibrasi gripper offset
   pasca P0-1c), nilai-nilai ini tidak otomatis mengikuti — harus disinkronkan manual dengan
   URDF.
4. **Intrinsics kamera tidak dipakai untuk estimasi jarak metrik** — `size` (proxy jarak) di
   `/hydroships/qr_offset.z` adalah fraksi area frame, bukan jarak terkalibrasi; akurasi
   depth-dari-visual belum diketahui.
5. **Tidak ada instrumentasi native untuk pose-error QR** — semua pengukuran di §2 harus
   diturunkan dari kombinasi topic + parsing log, belum ada topic diagnostik siap pakai.

---

## 4. Non-goals eksplisit untuk P0-2.0

- **Tidak ada perubahan controller** (`_goto_xy`, gain PD, visual-servo) dalam pass ini.
- **Tidak ada perubahan parameter** (`approach_*`, `qr_*`, `t_scan`, `t_nav_qr`, dst).
- **Tidak ada simulator yang dijalankan** — audit ini murni pembacaan kode statis + dokumen
  yang sudah ada.
- **Tidak ada klaim PASS/FAIL** untuk `APPROACH_QR` — matrix di §2 sengaja seluruhnya `OPEN`.

## 5. Langkah berikutnya yang diusulkan (P0-2.1, di luar scope dokumen ini)

Instrumentasi: perluas atau fork `tools/p0-experiments/recorder.py` agar juga subscribe
`/hydroships/qr_result` dan `/hydroships/qr_offset`, lalu hitung deret error pose turunan
(gabungan offset QR + odom) per tick — supaya baris-baris di §2 bisa mulai diisi dengan data
`MEASURED`, bukan `OPEN`. Ini **tidak** diimplementasikan dalam pass P0-2.0 ini.

---

## 6. P0-2.1 instrumentation smoke test

Dijalankan setelah §5 di atas. **Observability only** — tidak ada perubahan pada
`mission_fsm.py`, `qr_detector.py`, `qr_logic.py`, atau parameter apa pun. Dua berkas baru
ditambahkan (keduanya subscribe-only, tidak menerbitkan apa pun, `recorder.py` P0-1e tidak
disentuh):

- `tools/p0-experiments/recorder_qr.py` — fork `recorder.py`, menambah subscription
  `/hydroships/qr_result`, `/hydroships/qr_offset`, `/hydroships/manual/cmd`, dan `/rosout`
  (untuk parsing baris log `[FSM] A -> B` dari node `mission_fsm` — dipilih agar tidak perlu
  menambah publisher baru ke `mission_fsm.py` itu sendiri, yang notabene sedang diaudit).
- `tools/p0-experiments/run_approach_qr_smoke.sh` — fork `run_mission.sh`, memakai
  `recorder_qr.py` sebagai pengganti `recorder.py`, gate lewat `gate_mission.sh` (dipakai
  apa adanya, generik untuk stack penuh).

Satu run (`tag=S1`, `kki_arena`, 60 s sim, `P0_DATA_DIR=/tmp/p0-2-1-smoke`) dieksekusi pada
2026-08-08/09. Hasil gate: `gz-servers=1`, `odom-publishing`, `stabilizer`,
`thruster_allocator`, `mission_fsm`, `cmd_vel pub=1`, `thrust pub=1` — semua **PASS**, tidak
terkontaminasi. Recorder menulis 1010 baris (`S1.csv`, tidak disimpan di git — data mentah
run individual, sama seperti pola P0-1).

Verifikasi lima sinyal (tujuan langkah ini):

| Sinyal | Hasil | Evidence dari `S1.csv` |
|---|---|---|
| FSM state | **CONFIRMED** | Kolom `fsm_state` berubah sesuai urutan misi: `IDLE→DIVE→APPROACH_QR→GRAB→NAV_WALL→HANG→SURFACE→WAIT_TRIGGER` dalam satu run 60 s (107 baris `APPROACH_QR`, transisi ke `GRAB` terjadi ~t=11.1s). Parsing `/rosout` bekerja. |
| QR detection | **CONFIRMED** | 856/1010 baris punya `qr_result` non-kosong; huruf wall `D` terbaca berulang kali selama `APPROACH_QR` (mis. t=10.97–11.04s), cocok dengan log `qr_detector` di `S1.log` (beberapa `DECODE GAGAL` di awal sebelum sukses — konsisten dengan sifat "robust_decode" multi-tahap di §1.2). |
| QR offset/error | **CONFIRMED** | `qr_ex`/`qr_ey` terisi numerik (bukan NaN permanen) selama `APPROACH_QR`, mis. t=4.839s: `ex=0.17344, ey=0.03229, size=0.65417`; 107/107 baris `APPROACH_QR` punya `qr_ex` numerik. `qr_age` tetap kecil (<0.5s) — data segar, bukan stale. |
| Command/controller output | **CONFIRMED** | `cmd_fx`/`cmd_fy` non-zero selama `APPROACH_QR` (mis. t=4.952s: `fx=-16.0, fy=-16.0`, nilai clamp `approach_fmax`), turun ke 0 tepat saat transisi ke `GRAB` (`_set_surge` FSM state lain). |
| Odometry | **CONFIRMED** | `x,y,z,yaw` terisi tiap baris sejak `t0`, konsisten dengan pola P0-1e yang sudah tervalidasi. |

**Kesimpulan pass ini**: instrumentasi berfungsi — kelima sinyal yang dibutuhkan untuk
membedakan *perception* vs *frame/geometry* vs *controller/servo* vs *FSM/timing problem*
sudah bisa direkam bersamaan dan terkorelasi lewat sim-time yang sama. Ini **bukan** klaim
bahwa `APPROACH_QR` lolos acceptance — matrix di §2 di atas **tetap `OPEN`, tidak diubah**.
Satu run tunggal, tidak acak/berulang, tidak membuktikan repeatability atau convergence
formal; itu scope P0-2.2/P0-2.3, bukan langkah ini.

Catatan tambahan dari log run ini (observasi, bukan kesimpulan): run ini kebetulan mencapai
`GRAB` dalam ~10.7 s sejak entry `APPROACH_QR` dengan QR benar-benar terbaca (`D`) — berbeda
dari risiko §3.2 (jalur fallback XY-only tanpa QR) yang tetap belum diuji secara terpisah.
Satu run tidak cukup untuk menyimpulkan jalur mana yang dominan; itu pertanyaan untuk P0-2.2.

## 7. Status P0-2 saat ini

```text
P0-1        CLOSED / FROZEN        (tag p0-1-baseline)
P0-2.0      CLOSED                 (audit statis — dokumen ini, §1-5)
P0-2.1      CLOSED                 (instrumentasi — §6, observability terverifikasi)
P0-2.2      CLOSE-PARTIAL          (keputusan pengguna — lihat docs/P0-2-2-VERDICT-OPTIONS.md §4)
P0-2.2a     CLOSED                 (payload_pose instrumentation + smoke verification)
P0-2.2b     CLOSED                 (6/6 run valid, 0 INCONCLUSIVE — §7 spec)
  QR influence          VERIFIED   (Gate 3: 5/6 run, command benar-benar mengikuti qr_offset)
  QR precision convergence  OPEN   (Gate 4: 0/6 run masuk band qr_center_tol — dibawa ke P0-2.3)
P0-2.3      CLOSE-PARTIAL          (keputusan review acceptance — lihat
                                     docs/P0-2-3-ACCEPTANCE-REVIEW.md §4)
  Root-cause residual bias  CLOSED   (AABB/inflasi DAN decode quality, kontribusi independen —
                                     docs/P0-2-3-SEPARATION-SPEC.md §14)
  Precision convergence (Gate 4)   OPEN   (FAIL, tidak diuji ulang — 0/6 run P0-2.2b masuk band
                                     qr_center_tol, gap residual masih melebihi approach_tol
                                     pada inflasi tinggi)
P0-2.4      CLOSED — Gate 4 FAIL     (docs/P0-2-4-SPEC.md desain, docs/P0-2-4-RESULTS.md hasil:
                                     18 run/3 batch, 17 valid, 5/17 entered+held qr_center_tol
                                     band, stopping rule terpenuhi; 0/17 diverged; tidak ada
                                     rekomendasi engineering fix di dokumen manapun)
```

Detail P0-2.2: [`docs/P0-2-2-SPEC.md`](P0-2-2-SPEC.md) — spec eksperimen yang membedakan
QR-driven approach dari ground-truth-driven approach, memakai bukti kausal pasif (bukan
ablation). P0-2.2a (instrumentasi `payload_pose`+`vx/vy`) sudah tertutup dengan smoke
verification; run battery N=6 (P0-2.2b) sudah dieksekusi dan direduksi
(`reduce_approach_qr.py`) — hasil ringkas: 5/6 run QR ter-score tapi hanya 1/6 benar-benar
exit lewat visual centering (4/6 lewat toleransi jarak XY setelah QR ter-score, 1/6 murni
ground-truth-fallback tanpa QR pernah terbaca); 0/6 run masuk band `qr_center_tol`; 5/6 run
menunjukkan command memang bergerak mengikuti koreksi QR (Gate 3).

**Keputusan (pengguna, lihat `docs/P0-2-2-VERDICT-OPTIONS.md` §4): CLOSE-PARTIAL (Opsi B).**
Klaim yang ditutup: **"QR integration/causal influence terbukti"** (Gate 3, 5/6 run). Klaim
yang **tetap `OPEN`, dibawa ke P0-2.3**: "QR visual servo memberikan precision convergence
yang repeatable" (Gate 4, 0/6 run masuk band). Acceptance matrix §2 di atas **tidak diubah**
oleh keputusan ini — baris terkait presisi/konvergensi tetap `OPEN`, hanya baris "QR
benar-benar menjadi input kontrol?" yang kini punya evidence pendukung kuat. Tidak ada
re-run P0-2.2 yang direncanakan; langkah berikutnya adalah desain P0-2.3 dengan scope
lebih tajam — lihat `docs/P0-2-3-SPEC.md`.
