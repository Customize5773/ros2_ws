# PROBLEM.md — Catatan Masalah & Verifikasi Tertunda (HYDROships ros2_ws)

Dokumen ini mengumpulkan masalah diketahui, keputusan sementara, dan hal yang **masih
perlu diverifikasi di kolam/hardware asli**. Dijadikan catatan akhir saat semua milestone
selesai. Format: `[status]` OPEN / VERIFY / RESOLVED.

---

## Arena (M4) — sudah dibangun
- `[VERIFY]` Pemetaan label hook **A/B/C/D → sisi kolam** masih sementara (A=−Y, B=+Y,
  C=+X, D=−X di `worlds/kki_arena.sdf`). Pengacakan posisi ("Posisi A,B,C,D diacak")
  belum diimplementasi — menyusul bersama logika tugas.
- `[VERIFY]` Geometri hook = aproksimasi silinder J (Ø25 mm), titik terendah z=−0.45.
  Belum divalidasi apakah cukup untuk uji sangkut ROV nyata.

## Sensor & Persepsi (M3) — sudah dibangun
- `[VERIFY]` Kamera di-render headless via `gz-sim-sensors-system` (ogre2). Jalan di mesin
  dev ini (~22 Hz). Di mesin/CI tanpa GPU/EGL, sensor kamera bisa gagal render — perlu cek.
- `[VERIFY→terbagi]` **CameraInfo — dipisah jadi 2 sub-masalah** (status lama "belum
  dijembatani" sudah usang):
  - `[VERIFY]` **Jalur topic camera_info** SUDAH dijembatani di `bridge.yaml` (GZ_TO_ROS
    utk `/hydroships/camera_front/camera_info` & `/hydroships/camera_bottom/camera_info`,
    tipe `sensor_msgs/CameraInfo`). `qr_detector.py` kini juga SUBSCRIBE keduanya dan
    menyimpan matriks K per-kamera (log fx/fy/cx/cy saat pertama diterima). **Masih perlu
    verifikasi runtime**: `ros2 topic echo /hydroships/camera_front/camera_info --once`
    saat sim jalan → cek K/D/width/height terisi masuk akal (bukan nol) & nama topik gz
    aktual cocok (`gz topic -l`). Belum bisa diverifikasi di env ini (tak ada ROS2/Gazebo).
  - `[OPEN]` **Kalibrasi ke kamera fisik ROV asli** — intrinsics yang mengalir HANYA hasil
    kalkulasi Gazebo dari FOV/resolusi kamera SDF sim, **BUKAN** kalibrasi hardware ROV
    (belum ada datanya). K yang disimpan `qr_detector` **TIDAK** boleh dipakai untuk
    estimasi jarak riil sampai kalibrasi kamera fisik tersedia. Ini gap hardware, bukan
    bug kode — jangan "diselesaikan" dgn mengarang angka.
- `[VERIFY]` `depth_publisher` menurunkan kedalaman = max(0,−z) dari odom. Benar di sim
  (permukaan z=0). ROV near-neutral **sedikit positif** → mengapung ke permukaan saat
  thruster mati (depth→0). Ini sesuai desain M1, bukan bug.
- `[RESOLVED]` Node deteksi QR **sudah dibuat**: `qr_detector` (baca
  `camera_bottom/image_raw`, decode `cv2.QRCodeDetector`, publish `/hydroships/qr_result`
  A/B/C/D). Tanpa `cv_bridge` (decode Image manual via numpy). **Terverifikasi** dgn QR
  sintetik "A" → qr_result "A". Otomatis mengumpani FSM SCAN_QR.
- `[RESOLVED]` **Model payload dibangun** sesuai gambar KKI: braket-L (pelat 5×0.6×10 cm,
  lubang dia 3 cm 0.6 cm dari atas, kaki 3 cm) = mesh `media/payload_body.obj` (dibuat
  programatis, lubang bulat asli), + **QR 4×4 cm** di muka depan. Berdiri di dasar arena
  (pose `0.4 0 -0.9 ... yaw 90°`), QR menghadap −X. Model ada di scene, OBJ ter-load tanpa error.
- `[RESOLVED]` **Rendering tekstur QR** di kamera sensor Fortress headless: PBR `albedo_map`
  saja RENDER HITAM (butuh environment/IBL). **Fix:** pakai `<emissive_map>` (self-lit) →
  QR tampil & **terbaca** (`cv2` decode "A" dari kamera). Tekstur RGB (bukan grayscale) di
  plane (bukan box — box tak punya UV).
- `[RESOLVED]` **Perilaku APPROACH + depth/XY hold** (state `APPROACH_QR` di mission_fsm):
  ROV menyelam ke `scan_depth`, lalu **PD posisi** (kp/kd, redaman dari odom twist body-frame)
  mendorong ROV **DI ATAS** payload/QR datar & **menahannya**. Terverifikasi bersih: ROV
  menetap di **(0.41, 0.00, ~0.63)** tepat di target `payload_x/y` (0.4, 0) dan stabil.
  Kamera bawah digeser sedikit ke bawah badan (z=-0.18) agar tak terhalang mesh sendiri.
- `[VERIFY]` **QR belum terbaca meski posisi sudah tepat.** Di kedalaman scan, kamera bawah
  menampilkan lantai berfaset + bayangan; QR emissif 4 cm terlalu kecil pada render ini.
  **Fix diterapkan: perbesar QR khusus sim** 0.04→0.12 m (SIM_ONLY, di `kki_arena.sdf`;
  payload nyata tetap 4 cm). **Perlu verifikasi runtime** apakah kini `cv2` men-decode dari
  kamera sim. Bila belum: naikkan lagi ukuran / debug pencahayaan-scene. Sementara tetap bisa
  inject `/hydroships/qr_result` manual.
  - **UPDATE (percobaan fix, BELUM diverifikasi runtime sim — env dev ini tak punya
    ROS2/Gazebo, jadi hanya bukti OFFLINE):** menerapkan fix berjenjang dari yang paling
    ringan (urutan sesuai arahan), TANPA menambah dependency:
    1. **(opsi c) Preprocessing decode** — logika decode dipindah ke modul murni
       `qr_logic.robust_decode` (testable headless): coba decode via beberapa kandidat
       — mentah → grayscale+CLAHE → adaptive-threshold (±median-blur) → Otsu → upscale 2×.
       `qr_detector.py` kini memanggilnya. **Bukti offline:** `test/test_qr_logic.py`
       + harness adegan-terdegradasi (lantai berfaset + bayangan + kontras rendah + noise)
       → decode **mentah gagal** tapi **robust_decode memulihkan** frame ambang (mis. QR
       130 px kontras 0.5). Robust ≥ mentah (tak pernah regresi). CATATAN: adegan sintetik
       ekstrem (kontras 0.45, QR ≤110 px) masih gagal → preprocessing membantu frame
       marginal, BUKAN peluru perak; butuh uji render sim asli.
    2. **(opsi a) Quiet-zone** — plane putih self-lit 0.16 m di belakang QR (SIM_ONLY di
       `kki_arena.sdf`) memberi zona-tenang bersih agar finder-pattern QR tak terganggu
       lantai berfaset.
    3. **(opsi b) Lampu lokal** — `<light type="point" name="payload_fill">` (range 0.8 m)
       di atas payload menaikkan kontras QR tanpa mengubah exposure global scene.
    4. **(opsi d) Perbesar QR lagi**: TIDAK dilakukan (masih opsi terakhir; lihat "Opsi ditunda").
  - **Yang MASIH kurang untuk → [RESOLVED]:** log NYATA `/hydroships/qr_result` berisi
    A/B/C/D terbaca otomatis saat FSM di APPROACH_QR/SCAN_QR (tanpa inject manual), diambil
    dari run sim ber-GPU/EGL. Belum bisa dijalankan di environment ini.
  - **Instrumentasi diagnosis ditambahkan (untuk mempermudah run sim asli PERTAMA):**
    `qr_detector.py` kini (a) log "FRAME PERTAMA dari <topic>" sekali per kamera (bukti
    subscriber dapat data), (b) log "DECODE GAGAL" ber-throttle 5 s yang MEMBEDAKAN
    "QR tak terdeteksi (pts=None)" vs "QR terdeteksi tapi decode kosong" — dua kondisi
    ini butuh fix beda; dan (c) `_to_cv` kini menghormati `msg.step` (row stride) agar
    gambar tak ter-geser diam-diam bila publisher memberi padding baris. Log sukses
    "QR terbaca" tak diubah. **Run sim asli: TBD — run lokal Rasya** (hasil log akan
    dilaporkan balik untuk menentukan RESOLVED atau debug lanjutan).
- `[RESOLVED]` **`qr_detector` kini juga menerbitkan `/hydroships/qr_offset`**
  (geometry_msgs/PointStamped): offset piksel ternormalisasi + ukuran-tampak QR, sebagai
  sinyal **visual servo** (align presisi ROV ke payload). `camera_info` sudah dijembatani.
- `[note-uji]` Run sim yang tumpang-tindih meninggalkan proses `mission_fsm`/`parameter_bridge`
  lama yang **saling adu perintah** → ROV berperilaku erratic. Selalu pastikan proses lama
  mati (`ps | grep`) sebelum run baru. Bukan bug kode.

## Manipulator (M5) — DIHAPUS (akan dirancang ulang)
- `[REMOVED]` Seluruh subsistem gripper **dihapus** atas permintaan (rencana dibuat ulang).
  Yang dibuang: link `gripper_base` + 2 jari & plugin `JointPositionController`/
  `JointStatePublisher` di `hydroships.urdf.xacro`; node `gripper_controller`
  (+ entry-point `setup.py` & Node di `sim.launch.py`); topik `gripper_left/right/cmd`
  & `joint_states` di `bridge.yaml`; publisher `/hydroships/gripper/command`, method
  `_grip`, & semua panggilannya di `mission_fsm.py`.
- `[note]` State `GRAB`/`HANG`/`AUTO_RELEASE` di `mission_fsm` **dipertahankan sebagai
  kerangka gerakan** (depth/surge saja, tanpa jepit) agar misi tetap jalan; logika
  manipulasi menyusul saat rancangan gripper baru siap.
- `[TODO→DIKERJAKAN]` Rancang ulang manipulator — lihat subseksi
  **"Manipulator (M5) — RANCANG ULANG berbasis DetachableJoint"** di bawah.

## Manipulator (M5) — RANCANG ULANG berbasis DetachableJoint (kode ADA; grasp VERIFY)
Rancang ulang dari nol (bukan menghidupkan gripper 2-jari lama yg grasp fisiknya
tak pernah lolos uji). Pendekatan: **grasp = gz-sim DetachableJoint** (sambungan
kaku ROV↔payload), jari hanya kosmetik 1 DOF. Kontrak semantik lama dipertahankan
(`/hydroships/gripper/command` "open"/"close") demi kompatibilitas GUI/autonomy.

- `[RESOLVED]` **Model gripper 1-DOF dibuat ulang** di `hydroships.urdf.xacro`:
  link `gripper_base` (kaku di perut depan ROV, menghadap bawah) + `gripper_jaw`
  (revolute 1 DOF, kosmetik) + plugin `JointPositionController` (target sudut via
  `/hydroships/gripper_jaw/cmd`). Reach ke BAWAH karena payload di-approach dari
  atas (kamera bawah). URDF ter-proses bersih (xacro OK).
- `[RESOLVED]` **Plugin DetachableJoint** ditambahkan: `parent_link=gripper_base`,
  `child_model=payload`, `child_link=payload_link`, `attach_topic=/hydroships/gripper/attach`,
  `detach_topic=/hydroships/gripper/detach`. Payload di `worlds/kki_arena.sdf` diubah
  **non-static + diberi massa (0.3 kg) & collision box** agar bisa diangkat & diam di
  dasar (negatif apung). (CATATAN: tag `<suppress_initial_attach>` yang sempat dipakai
  di sini ternyata TIDAK valid di Fortress → dihapus; initial-attach ditangani lewat
  auto-detach startup node. Lihat item "Ketergantungan versi gz-sim Fortress" [RESOLVED].)
- `[RESOLVED]` **Node `gripper_controller` baru** (`hydroships_control`): terima
  open/close, gerakkan jari kosmetik, dan picu attach/detach. Attach HANYA saat
  "close" DAN ROV di atas payload dalam **jangkauan aman** (dinilai dari
  `/hydroships/qr_offset`: |offset x/y| kecil & ukuran-tampak QR cukup besar & segar)
  → cegah attach "dari jauh" yg menyeret payload menembus air. Detach saat "open".
- `[RESOLVED]` **Logika inti dipisah** ke `gripper_logic.py` (murni, tanpa rclpy) →
  **teruji headless**: `test/test_gripper.py` (13 test: attach dalam/luar jangkauan,
  sinyal basi, open/detach, sinonim, batas, dsb.) — **28/28 test paket lulus**
  (15 lama + 13 gripper). Bridge `bridge.yaml` menambah 3 topik gripper
  (jaw/cmd Float64↔Double, attach/detach Empty↔Empty). Node ditambahkan ke
  `sim.launch.py`; entry-point ditambah di `setup.py`.
- `[RESOLVED]` **Re-hook ke `mission_fsm.py`**: method `_grip(close)` dikembalikan;
  `St.GRAB` mengirim 'close' (attach), `St.AUTO_RELEASE` mengirim 'close' saat
  mendekati hook lalu 'open' (detach) saat melepas. Alur state IDLE→…→DONE TIDAK
  diubah (hanya isi perintah manipulasi ditambahkan).
- `[VERIFY]` **Grasp fisik BELUM diuji di sim/kolam.** Sama seperti gripper lama,
  belum ada bukti bahwa attach/detach benar-benar mengangkat & melepas payload di
  Gazebo. Perlu run sim: cek payload ter-attach saat GRAB, terbawa saat NAV_WALL,
  lepas saat AUTO_RELEASE.
- `[RESOLVED]` **Ketergantungan versi gz-sim Fortress (initial-attach).**
  **Akar masalah:** tag `<suppress_initial_attach>true</suppress_initial_attach>`
  yang dipakai di `hydroships.urdf.xacro` **TIDAK valid di gz-sim Fortress** — tak
  ada di dokumentasi plugin DetachableJoint versi 6/7/9; fitur serupa
  (`<initial_attach>`) baru diusulkan lewat **PR gz-sim #3268 (masih Open, menyasar
  gz-sim10)**, jauh di atas Fortress. Tag itu diabaikan Gazebo secara diam-diam →
  perilaku default DetachableJoint = **SELALU attached saat load**, payload nge-lock
  ke ROV sejak detik pertama sim, sebelum FSM masuk state GRAB.
  **Fix (opsi "spawn attached lalu detach"):** (1) hapus tag yang tak berfungsi dari
  URDF (+ komentar penjelas agar tak menyesatkan). (2) `gripper_controller` kini
  menerbitkan **satu pesan detach otomatis saat startup node** (timer satu-kali,
  delay `startup_detach_delay`=1.5 s agar model ROV & payload sudah ter-spawn),
  memaksa lepas kondisi attached bawaan sebelum menerima open/close apa pun. Logika
  murni `gripper_logic.startup_detach()` **teruji** (`test/test_gripper.py`:
  emit-detach, clear-prior-attach, re-attach-normal-setelahnya). Suite **31/31
  gripper lulus** (dari 28 → +3). Detach pada joint yg tak ada aman diabaikan gz,
  jadi fix idempoten. **Verifikasi runtime sim (payload benar-benar lepas saat
  spawn) masih menyusul** — env dev tanpa Gazebo, tapi mekanisme detach sudah
  terbukti dipakai di jalur GRAB/AUTO_RELEASE.
- `[OPEN]` **Tuning ambang jarak-aman & massa payload.** `max_offset=0.30`,
  `min_size=0.12`, massa payload 0.3 kg, gaya JointPositionController — semua
  ESTIMASI; setel setelah uji sim agar attach terpicu tepat & payload tak melayang.
- `[OPEN]` **Jari kosmetik tak menyatu dgn grasp fisik.** Sudut jari hanya visual;
  DetachableJoint yg memegang. Bila ingin jari benar-benar menjepit (dgn payload),
  perlu geometri jari + friction/contact — di luar lingkup rancang-ulang minimal ini.
## Manipulator (M5) — GRIPPER DIHILANGKAN dari model
- `[RESOLVED]` **Model gripper dihapus** dari ROV atas permintaan: link `gripper_base`
  + 2 jari + 2 `JointPositionController` + `JointStatePublisher` dilepas dari
  `hydroships.urdf.xacro`; node `gripper_controller` dilepas dari `sim.launch.py`;
  entri bridge `joint_states` & `gripper_left/right/cmd` dilepas dari `bridge.yaml`.
  Bila perlu manipulator lagi, kembalikan dari riwayat git. Catatan lama di bawah
  disimpan sebagai referensi bila akan dibangun ulang.

<details><summary>Catatan lama (gripper, sudah tidak aktif)</summary>

- Gripper 2 jari (revolute sumbu z) di depan ROV, dikontrol gz JointPositionController.
  Perintah semantik `/hydroships/gripper/command` ("open"/"close") → node
  `gripper_controller` → setpoint 2 jari (bridge → gz). State jari di
  `/hydroships/joint_states`. **Terverifikasi**: open ≈ +0.50/−0.50, close ≈ −0.14/+0.15 rad.
- `[VERIFY]` Sudut `open_angle=0.5` / `close_angle=-0.15` (param node) & arah buka/tutup
  masih perlu dicek visual di GUI/kolam agar celah jepit pas ukuran payload.
- `[OPEN]` **Grasp fisik belum diuji**: jari punya collision tapi belum ada model payload
  (bagian M4). Menjepit andal kemungkinan butuh penyetelan friction atau plugin
  attach/detach (mis. gz-sim DetachableJoint) — belum diimplementasi.
- `[OPEN]` `/hydroships/joint_states` (dari gz) hanya berisi 2 joint gripper, bukan
  thruster. TF jari via robot_state_publisher belum tersambung (rsp mendengar
  `/joint_states`, sedangkan bridge ke `/hydroships/joint_states`). Remap bila butuh TF.
- `[note]` `ros2 topic pub --once` ke command bisa meleset karena race discovery;
  node menerbitkan ulang setpoint 2 Hz sehingga joint tetap menahan posisi. Konsumen
  nyata (GUI/autonomy) mengirim berulang, jadi aman.

</details>

## FISIKA ROV — DUA BUG BESAR DITEMUKAN & DIPERBAIKI (RESOLVED)
Menjawab "kenapa ROV makin dibiarkan makin melayang, tidak menggenang di air":
1. `[RESOLVED]` **Buoyancy tanpa permukaan.** World pakai `<uniform_fluid_density>` yang
   memberi gaya apung **di MANA SAJA** (tak ada permukaan bebas). ROV sedikit-positif →
   net gaya ke atas selalu → melayang naik tanpa henti. **Fix:** ganti ke `<graded_buoyancy>`
   (air 1000 di bawah z=0, udara 1 di atas) di `worlds/kki_arena.sdf` & `pool_empty.sdf`.
   Hasil: ROV **menetap di permukaan** (odom z ≈ −0.14, stabil), tidak melayang lagi.
2. `[RESOLVED]` **Thrust tak pernah masuk.** Plugin Thruster `<namespace>hydroships</namespace>`
   MEN-prepend namespace ke `<topic>hydroships/…</topic>` → subscribe ke
   `/hydroships/hydroships/thruster_N/thrust`, sedangkan bridge publish ke
   `/hydroships/thruster_N/thrust` → **tak nyambung, gaya thruster nol**. (Gerakan naik
   sebelumnya murni buoyancy, bukan thrust.) **Fix:** ubah `<topic>` jadi `${name}/thrust`
   (tanpa prefix) di `hydroships.urdf.xacro`. Hasil: wrench −40 N → ROV **menyelam** dari
   −0.14 ke −0.75 m dan terkendali.

> Catatan: M1/M2 sebelumnya ditandai ✅ tapi ternyata **thrust tak pernah benar-benar
> menggerakkan ROV** (topik tak nyambung) — verifikasi lama kurang teliti. Kini teruji nyata.

## FISIKA ROV — TEMUAN KETIGA: frame posisi thruster salah → YAW mati (RESOLVED)
- `[RESOLVED]` **Bidang horizontal (surge/sway/yaw) badly-conditioned; YAW praktis mati.**
  Analisis numerik Thrust Allocation Matrix (dari `hydroships_control/allocation.py`):
  `cond(TAM) ≈ 12.377`, singular value terkecil ≈ `1e-4` (arah yaw). Dengan
  pseudo-inverse polos (invers eksak utk TAM 6×6 full-rank), perintah:
  - **yaw 1 N·m** menuntut T3/T4 = **±5000 N** (batas ±50 N),
  - **surge 1 N** menuntut T3/T4 = **±648 N**
  → semua ter-clip lalu **merusak DOF lain**. Heading-hold & NAV_WALL karena itu
  tak bisa bekerja andal (cocok dgn catatan "belum di-tune untuk gerak nyata").
- **Akar masalah (geometri) — POSISI, bukan axis:** `thruster_positions.csv`
  berkonvensi **X=lateral (kanan +), Y=fore/aft (DEPAN negatif), Z=atas** (lihat
  `thruster_topview.png`), sedangkan frame body ROS **x=maju, y=kiri, z=atas**.
  Posisi disalin **mentah** `(X,Y,Z)→(x,y,z)` tanpa rotasi frame → posisi terputar 90°.
  Akibatnya T100-A/C yang FISIKnya terpisah kiri-kanan (sumber yaw) di model jadi
  terpisah depan-belakang → **lengan momen yaw ≈ 0**. (Axis/peran sebenarnya **sudah
  benar**: A/C surge, B sway, D/E/F vertikal — dikonfirmasi pengguna via anotasi arah
  gaya. T100-B kini dilabeli **T200-B** tapi tetap thruster sway.)
- `[RESOLVED]` **Fix:** koreksi konversi frame posisi di `allocation.py` (THRUSTERS)
  & `hydroships.urdf.xacro`: `x_body=-Y_csv, y_body=-X_csv, z_body=Z_csv`. Hasil
  (terverifikasi numerik): **cond(TAM) 12.377 → 19,7**, rank 6/6, **yaw 5 N·m cukup
  ~18 N** (dulu 25.000 N), surge/sway/heave semua bersih. ROV kini terkendali penuh
  6-DOF; heading-hold & navigasi bekerja.
- `[RESOLVED-insurance]` **Allocator damped least-squares** (`build_damped_pinv`,
  param `alloc_damping=0.1`) tetap dipakai sebagai jaring pengaman + node log cond(TAM).
  Dgn TAM kini well-conditioned, redaman kecil ≈ pinv (tak merugikan). Bisa diturunkan
  ke 0 bila diinginkan pinv murni.
- `[note]` Konsistensi posisi URDF ↔ allocation.py masih manual (duplikat). Opsi lanjut:
  satu sumber-kebenaran parametrik / test konsistensi otomatis (belum wajib).

## Model Visual ROV — DIBUAT ULANG jadi PRIMITIF SEDERHANA (RESOLVED tahap-1)
- `[RESOLVED]` **Model visual dibuat ULANG dari primitif ringan** (ganti mesh STL 12 MB
  yang acak-acak & berat). Visual `base_link` kini = rangka kotak HITAM (bawah) + busa
  apung ORANYE (atas) + tabung elektronik abu-abu + dome kamera depan (penanda haluan +X),
  bergaya BlueROV Heavy sesuai foto `Gambar ROV/`. Tetap mengisi bbox desain
  0.345×0.345×0.286 m; collision box (fisika/buoyancy) TIDAK diubah. Tak ada lagi mesh
  berat → render kamera tak terbebani mesh 237k segitiga. Detail per-komponen (thruster
  ducted, gripper mesh, dsb.) menyusul sebagai penyempurnaan berikutnya.
- `[RESOLVED]` `meshes/rov.stl` & `model/rov.fbx` **sudah dihapus** dari repo (tak lagi
  dirujuk URDF). Bersama itu artefak build colcon (`build/`, `install/`, `log/`) & cache
  `__pycache__/*.pyc` juga dikeluarkan dari git + ditambah `.gitignore` (praktik standar
  ROS2). Sim perlu `colcon build` ulang di mesin lokal untuk regen `install/`.

### (Catatan lama — mesh FBX, sudah tidak dipakai)
- `[RESOLVED]` **Struktur mesh "acak-acak" diperbaiki.** Sebelumnya 279 sub-mesh FBX
  digabung pakai vertex LOKAL tanpa menerapkan transform node → semua bagian
  terkumpul salah posisi. **Fix:** load dgn assimp `aiProcess_PreTransformVertices`
  (bake transform hierarki ke world-space) sebelum merge.
- `[RESOLVED]` **Skala terpecahkan.** Setelah transform benar, bbox mesh =
  **350.7 × 344.5 × 286.0 mm** — persis ukuran kotak desain (0.345×0.345×0.286 m) →
  **FBX satuan MILIMETER → scale = 0.001**. Mesh di-recenter ke origin di file, jadi
  URDF origin xyz=0. (Fortress tak bisa FBX → dipakai STL hasil konversi.)
- `[RESOLVED]` `package://`→`model://` tak ketemu → tambah `IGN_GAZEBO_RESOURCE_PATH`
  di `sim.launch.py`. Mesh ter-load tanpa error (0 geometry-load-failures).
- `[MOOT]` **Poly-count berat.** (Historis — mesh STL 12 MB sudah dihapus & diganti
  primitif ringan; lihat "Model Visual ROV — DIBUAT ULANG". Tak lagi relevan kecuali
  mesh detail dihidupkan lagi.) Dulu: 279 komponen, fast-simplification mentok ~237 k
  segitiga → rate kamera ~22→10 Hz. Bila kelak pakai mesh: decimator quadric
  (open3d/pymeshlab) atau buang komponen kecil sebelum merge.
- `[VERIFY]` **Arah bow (haluan) belum dipastikan.** Footprint mesh ~persegi (X≈Y) jadi
  tak bisa ditebak dari bbox. Diatur via properti `bow_yaw` (rad) di `hydroships.urdf.xacro`
  (default 0 = bow menghadap +X). Cek di GUI & set 1.5708/−1.5708/3.14159 bila perlu.
- `[VERIFY]` STL monokrom (tanpa warna/material asli). Bila perlu warna, ekspor DAE
  ter-decimate (jaga ukuran) atau set material per-bagian di URDF.
- `[note]` Sumber `model/rov.fbx` (48 MB) dibiarkan di repo; yang dipakai sim = `meshes/rov.stl` (12 MB).

## Autonomy (M6) — kode dibangun, INTEGRASI JALAN (setelah fix fisika)
Node `mission_fsm` (ROS 2) + launch `hydroships_bringup/launch/hydroships_mission.launch.py`
(sim + allocator + stabilizer + FSM). FSM mengendalikan lewat setpoint stabilizer
(`setpoint/depth`, `setpoint/heading`, `manual/cmd`). (Perintah gripper dihapus — lihat M5.)

- `[RESOLVED]` Setelah dua fix fisika di atas, misi **berjalan**: FSM `IDLE→DIVE` →
  "Dasar tercapai (0.76 m)" → `DIVE→SCAN_QR` → (inject QR "A") "QR → wall A (+15)" →
  `SCAN_QR→GRAB`. Depth-hold & heading-hold bekerja.
- `[VERIFY]` Timeout tiap state, gaya (`surge_force` dll), & sudut belum di-tune untuk
  gerak nyata di arena; baru diuji transisi awal.
- `[VERIFY]` `SCAN_QR`/`APPROACH_QR` mengonsumsi `/hydroships/qr_result` — node
  `qr_detector` **sudah ada** (lihat M3, RESOLVED). Yang tersisa murni keterbacaan
  visual QR di render (lihat item "QR belum terbaca"); sampai itu beres, uji misi penuh
  dgn inject `ros2 topic pub /hydroships/qr_result` manual atau `start_state:=`.
- `[RESOLVED→VERIFY]` `APPROACH_HOOK` **tak lagi hanya timed**: node `hook_detector`
  (port dari `GUI-ROV/autonomy/vision/hook_detect.py`) menerbitkan `/hydroships/hook_offset`
  dan `mission_fsm` kini **servo** ke hook (fallback timed tetap ada). Belum diuji di
  render sim — lihat seksi "Integrasi GUI tim (M7)".
  - **UPDATE — servo di-upgrade jadi PD penuh (Tugas 3):** servo lama hanya proporsional
    pada HEADING (`hook_kp_yaw * ex`). Diganti **PD holonomik** di `hook_logic.hook_servo`
    (fungsi MURNI, testable): **sway** dari offset-x + **surge** dari ukuran-tampak +
    **koreksi setpoint kedalaman** dari offset-y, semua dgn **redaman kecepatan** body-frame
    (kp·err − kd·vel, pola `_goto_xy`). Heading di-hold menghadap wall (kamera depan tetap
    melihat hook). `_st_approach_hook` di-rewire; `T['approach']` tetap timeout aman (ABORT/
    lanjut bila tak konvergen); fallback timed dipertahankan bila deteksi hilang.
    **Bukti offline:** `test/test_hook_servo.py` (11 test: arah tanda sway/surge/depth,
    redaman kecepatan, clamp gaya, konvergensi loop, flag aligned/near). **Suite 58/58 lulus.**
  - **TETAP [VERIFY]** (bukan RESOLVED): belum ada run sim (env ini tak punya ROS2/Gazebo).
    Yang kurang: log NYATA servo konvergen ke hook di render sim + deteksi hook di kamera
    depan terbukti (ambang `min_area`/CLAHE default masih uji-meja). Gain PD (`hook_kp_*`,
    `hook_kd_*`) = estimasi, perlu tuning saat uji sim.

## Sistem Launch Simulasi Gazebo — DIPERBAIKI (RESOLVED)
- `[RESOLVED]` **`stabilizer` tak diberi `use_sim_time`.** Di
  `hydroships_stabilized.launch.py` node stabilizer jalan di wall-clock sementara
  sim & node lain di sim-time → PID d-term & laju setpoint salah timing. **Fix:**
  `parameters=[gains, {'use_sim_time': True}]`.
- `[RESOLVED]` **Race condition spawn ROV.** Node `create` dijalankan bersamaan
  dgn server gz; bila service `/world/<world>/create` belum  siap, model gagal
  di-spawn. **Fix:** bungkus spawn di `TimerAction` (delay default 3 s, arg
  `spawn_delay` bisa dinaikkan untuk mesin lambat) di `sim.launch.py`.
- `[note]` `install/` kini di-gitignore → jalankan `colcon build` lokal dulu
  sebelum `ros2 launch` agar share (launch/world/config) ter-regen.

## Opsi ditunda
- `[OPEN]` **Perbesar QR khusus sim.** Alternatif membuat scan andal tanpa approach presisi:
  pakai QR jauh lebih besar dari 4 cm (mis. 15–25 cm) HANYA untuk sim, agar mudah di-decode
  dari jarak. Trade-off: tak sesuai ukuran asli KKI (4 cm). Dipilih belakangan bila perilaku
  APPROACH + hold belum cukup.

## Umum / lintas-milestone
- `[VERIFY]` Massa & koefisien hidrodinamika ROV masih **placeholder** (near-neutral;
  massa/inertia dari geometri box, hidrodinamika acuan tipe BlueROV2), **belum data
  ROV asli**. STATUS BARU: parameter kini **ter-parameterisasi & siap diisi data nyata** —
  dipindah dari nilai hardcode di `hydroships.urdf.xacro` ke
  `hydroships_description/config/rov_params.yaml` (dibaca URDF via `xacro.load_yaml`).
  Yang ter-eksternal: `base_mass`, `thruster_mass`, `fluid_density`, `cog`/`cob`,
  tensor inertia 3×3, dan 18 koefisien hidrodinamika (added-mass + linear + quadratic).
  Alat bantu `hydroships_description/scripts/estimate_mass_inertia.py` menghitung
  estimasi massa & inertia dari dimensi box + massa jenis material (+ massa titik
  komponen via teorema sumbu sejajar) → tinggal isi input ukur nyata, re-run, tempel
  ke YAML, lalu ubah tag `[estimate]`→`[measured]`. URDF hasil parameterisasi
  **identik** dgn versi hardcode (inertia cocok ~1e-6, massa & buoyancy tak berubah);
  `test_allocation.py`/`test_pid.py` tetap 15/15 lulus. **Belum RESOLVED**: angka fisik
  asli HYDROships belum tersedia — masih estimasi sampai diukur.
## Integrasi GUI tim (M7) — ADAPTER DIBUAT (belum diverifikasi live)
Repo GUI **Customize5773/GUI-ROV** ternyata **tidak memakai ROS 2** — memakai
UDP-JSON + MAVLink (ArduSub). Analisis selisih lengkap & desain adapter di
`docs/GUI-INTEGRATION.md`. Karena transport beda total, remap topik tak cukup →
dibuat node adapter (bukan mengubah node inti).

- `[RESOLVED]` **Analisis selisih antarmuka** GUI-ROV vs kontrak ROS
  (`docs/ARCHITECTURE.md`) didokumentasikan (`docs/GUI-INTEGRATION.md`): transport,
  unit (persen vs N, deg vs rad), nama/arah, frame — lengkap dgn penanganan.
- `[RESOLVED]` **Node adapter `gui_bridge`** dibuat (`hydroships_control`):
  UDP JSON `{name,value}` GUI → `/hydroships/cmd_vel` (wrench) & `gripper/command`;
  `/hydroships/odom`+`/depth` → telemetri UDP JSON GUI. **Tak menyentuh** stabilizer/
  mission_fsm/thruster_allocator. Logika murni `gui_bridge_logic.py` **teruji headless**.
- `[RESOLVED]` **APPROACH_HOOK: visual servo** menggantikan behavior *timed*.
  `autonomy/vision/hook_detect.py` GUI-ROV **di-port** jadi node `hook_detector`
  (pola qr_detector) → `/hydroships/hook_offset`; `mission_fsm._st_approach_hook`
  kini servo dgn **fallback timed** aman. Normalisasi offset & PD servo murni di
  `hook_logic.py` teruji. **UPDATE (Tugas 3):** servo di-upgrade dari proporsional-heading
  jadi **PD holonomik penuh** (sway+surge+koreksi-depth, redaman kecepatan) —
  `hook_logic.hook_servo` + `test/test_hook_servo.py`. Lihat detail di seksi M6.
- `[VERIFY]` **Belum diuji end-to-end dgn GUI live** (joystick GUI → ROV sim gerak;
  telemetri muncul di dashboard). Belum diverifikasi di environment ini.
- `[VERIFY]` **Deteksi hook di render kamera sim belum diuji**; ambang default =
  uji-meja, perlu tuning (glare/kekeruhan/kontras).
- `[OPEN]` **Kalibrasi**: gain persen→N (`surge/sway/heave/yaw_gain`), offset heading
  kompas (0° vs +x REP-103), tanda sumbu, port UDP (default cmd 14550 / telem 14551),
  ambang servo hook (`hook_size_stop`, `hook_kp_yaw`). Semua estimasi.
- `[OPEN]` Servo hook masih IBVS (image-based: PD sway+surge+depth, tanpa kalibrasi);
  pose-based (solvePnP/PBVS) menyusul bila kalibrasi kamera FISIK hook tersedia
  (intrinsics sim sudah mengalir tapi bukan kalibrasi hardware — lihat Tugas 2 &
  `PoseServo` di `GUI-ROV/autonomy/control/visual_servo.py`).
