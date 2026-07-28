# Inventaris part — `@ROV KKI 2026 NEW DESIGN revisi FIXXXX GOOLLLL SIUUUU.f3z`

Sumber: `DOKUMENTASI ROV/@ROV KKI 2026 NEW DESIGN revisi FIXXXX GOOLLLL SIUUUU.f3z`
(165 MB, Fusion 360 archive; root design `16c35b6d-…f3d`, versi 95, 2026-07-14)

Dipakai untuk membangun [`rov_kki2026_new_design.urdf.xacro`](rov_kki2026_new_design.urdf.xacro).

## Apa yang bisa & tidak bisa dikonversi

| Data | Status | Alasan |
|---|---|---|
| Nama komponen & hierarki XREF | ✅ terbaca | `DesignDescription.json` |
| Nama occurrence internal (frame, bracket, dll) | ✅ terbaca | UTF-16 di `FusionDesignSegmentType1/BulkStream.dat` |
| Jumlah instance / mirror | ✅ terbaca | idem |
| **Mesh / geometri** | ❌ dari `.f3z` — ✅ dari STL | body = Autodesk ShapeManager B-rep (`BREP.*.smb`), format tertutup. Geometri sekarang datang dari `@ROV KKI 2026 NEW DESIGN.stl` |
| **Transform occurrence** | ❌ | Neutron binary stream, tidak terdokumentasi |
| **Massa & inersia** | ⚠️ estimasi | dihitung dari mesh dengan asumsi densitas seragam, bukan dari Fusion |

Format internal: `.f3z` = ZIP biasa, tapi tiap `.f3d` di dalamnya adalah ZIP
dengan compression method **93 (zstd)** — `unzip` dan `zipfile` Python 3.10
diam-diam gagal di entry ini. Dekompresi manual: baca raw bytes lalu pipe ke
`zstd -d`.

## Status geometri saat ini

Geometri sudah masuk lewat **`DOKUMENTASI ROV/@ROV KKI 2026 NEW DESIGN.stl`**
(75 MB, 1.515.494 face, unit mm). Mesh turunan di
`src/hydroships_description/meshes/kki2026/`:

| File | Face | Ukuran | Cara dibuat |
|---|---|---|---|
| `rov_kki2026_visual.stl` | 67.131 | 3,4 MB | buang fastener (<25 mm) lalu quadric decimation |
| `rov_kki2026_collision.stl` | 1.646 | 82 KB | convex hull |

Bounding box mesh: 346,9 × 507,9 × 295,9 mm.

**Origin STL bukan konvensi ROS.** Sumbu memanjang ROV ada di Y mesh dan
haluan di −Y mesh, jadi mesh diputar +90° terhadap Z lalu digeser supaya
`base_link` berimpit dengan pusat massa:

    x_base = -y_mesh    y_base = x_mesh    z_base = z_mesh
    origin  xyz="0.10542 0.00426 0.06756"  rpy="0 0 1.5708"

Bukti orientasi (semuanya terukur, bukan asumsi):

* tabung enclosure OD 130 mm × 210 mm bersumbu Y mesh → itu sumbu fore/aft;
* 3 thruster bersumbu Z mesh (heave), 2 bersumbu Y mesh (surge), 1 bersumbu
  X mesh (sway) — pola 3-2-1 khas ROV;
* struktur gripper/payload ada di ujung −Y mesh → itulah haluan.

Setelah transform, bbox di `base_link`: x −0,197…+0,311 · y −0,170…+0,177 ·
z −0,147…+0,149 m.

Model FBX lama (`Model lain/rov.urdf.xacro`) **tidak** memutar mesh, jadi
koordinatnya tidak sebanding dengan file ini.

**STL-nya satu mesh gabungan** (32.108 body terpisah, tanpa nama, tanpa
grouping), sehingga tidak bisa dipecah per komponen secara otomatis. Karena
itu di xacro seluruh visual/collision menempel di `base_link`, dan link
lain jadi frame referensi tanpa geometri sendiri. Jari gripper punya joint
revolute yang benar secara kinematik, tapi tidak akan terlihat bergerak
karena bentuknya ikut terpanggang di mesh `base_link`.

Decimation saja mentok di ~95k face karena mesh punya 32.108 body terpisah
yang masing-masing butuh minimal beberapa face. Karena itu 31.944 fastener
(baut/mur, diagonal <25 mm) dibuang lebih dulu — tersisa 164 body — baru
di-decimate. `PROBLEM.md` mencatat mesh 237k face dulu menjatuhkan rate
kamera 22→10 Hz, jadi jangan naikkan angka ini.

Collision memakai convex hull: murah untuk solver tapi **mengisi rongga
rangka terbuka**, dan volumenya menentukan gaya apung (lihat di bawah).

Catatan repo: file `.stl` sumber ada di `.gitignore`, tapi `meshes/kki2026/`
dan `urdf/` **tidak** — jadi 6 MB mesh turunan itu akan ikut ter-commit
kalau di-`git add`.

## Semua angka bisa dihitung ulang

Konstanta geometri di xacro tidak ditulis tangan. `scripts/measure_cad_frames.py`
menurunkannya dari STL — jalankan lagi kalau CAD direvisi:

```bash
python3 src/hydroships_description/scripts/measure_cad_frames.py --stl "DOKUMENTASI ROV/@ROV KKI 2026 NEW DESIGN.stl" --verbose
```

Tambahkan `--export-meshes --out-dir src/hydroships_description/meshes/kki2026`
untuk membuat ulang mesh visual & collision.

## Posisi thruster (terukur dari mesh)

Enam unit dikenali lewat connected-component analysis: rumah thruster =
bongkah kompak 45–110 mm, arah dorong = normal cakram propeller/duct terdekat.
Jumlahnya cocok dengan `.f3z`: 4× T100 + 2× T200.

| nama | peran | kelas | duct terukur | posisi (m) |
|---|---|---|---|---|
| `thruster_1` | heave depan-kanan | T100 | prop 66,6 mm | `0,1050 −0,1332 0,0592` |
| `thruster_2` | heave depan-kiri | T100 | prop 66,6 mm | `0,1053 0,1411 0,0550` |
| `thruster_3` | surge kanan | T200 | duct 90,1 mm | `−0,0291 −0,1188 0,0186` |
| `thruster_4` | surge kiri | T200 | duct 90,1 mm | `−0,0287 0,1274 0,0190` |
| `thruster_5` | sway bawah | T100 | prop 59,0 mm | `−0,0700 0,0200 −0,0939` |
| `thruster_6` | heave belakang | T100 | prop 66,6 mm | `−0,1610 0,0037 0,0621` |

Verifikasi: setelah transform, keenam titik berjarak 3,4–8,5 mm dari permukaan
mesh (sebelumnya, dengan koordinat desain lama, 8–108 mm).

**Sudah sinkron (2026-07-28).** Tabel `THRUSTERS` di
`hydroships_control/allocation.py`, `hydroships.urdf.xacro`, dan model ini
kini memakai angka yang sama, dijaga oleh
`hydroships_control/test/test_thruster_urdf_sync.py`.

Penomoran & peran mengikuti Gambar 2.9/2.10 dokumen desain (penulis, 2026),
yang **sepakat dengan CAD**: pasangan haluan digambar sebagai lingkaran
(propeller searah sumbu pandang → vertikal), pasangan tengah dan unit
tengah-bawah digambar profil samping → horizontal. Mapping lama menukar
peran kedua pasangan itu dan menukar `#5` ↔ `#6`.

Arah putar propeller dari Gambar 2.9 — 1 CCW, 2 CW, 3 CCW, 4 CW, 5 CW,
6 CCW — tersimpan sebagai `SPIN` di `allocation.py` (dokumentatif; TAM hanya
butuh gaya & lengan momen).

Dampak ke kendali: `cond(TAM)` **19,7 → 9,98**, singular value terkecil
**0,088 → 0,174**, pitch yang tercapai dengan damped pinv **44 % → 82 %**.
Geometri terukur ini justru lebih mudah dikendalikan.

Yang **belum** terverifikasi: tanda sumbu dorong. Gambar 2.10 menggambar
F1/F2 ke buritan dan F6 ke kiri, tapi panah untuk thruster vertikal
(F3/F4/F5) jelas artefak — arah gaya thruster vertikal tidak bisa digambar
di tampak atas — sehingga gambar itu tidak konsisten dengan dirinya sendiri.
Dipakai +x (maju) untuk surge mengikuti REP-103 dan +y untuk sway mengikuti
gambar. Uji bench: beri perintah positif ke `thruster_1`; kalau ROV mundur,
balik tanda di `allocation.py` dan kedua URDF.

## Apung — sudah disetel ke 6,64 kg

| besaran | nilai | apung setara |
|---|---|---|
| volume solid semua body | 0,004283 m³ | — |
| rongga tertutup tabung enclosure | 0,002361 m³ | — |
| **volume tergeser nyata** | **0,006644 m³** | **6,644 kg** |
| collision `hull` (convex hull) | 0,028047 m³ | 28,05 kg |
| collision `box` (bbox) | 0,052140 m³ | 52,14 kg |

Plugin Buoyancy gz menghitung gaya apung dari **volume collision**, jadi
ukuran collision menentukan fisika. Hull dan bbox 4–8× terlalu besar karena
rangka ROV ini **terbuka** — air mengalir menembusnya, yang tergeser hanya
bahan padat + rongga tabung.

Karena itu mode collision default `physical` menyusun dua primitif yang
volumenya dijumlah tepat = 0,006644 m³: silinder tabung enclosure (posisi &
ukuran terukur) + pelat tipis seluas footprint ROV. Tinggi pelat disetel agar
centroid volume jatuh di **+0,020 m di atas CoG** → CoB > CoG, stabil
terhadap roll/pitch.

`rov_params.yaml` kini memakai `vehicle_mass: 6.64` (massa **total**, bukan
massa base_link) dan `displaced_volume: 0.006644`. Tiap URDF menghitung
sendiri massa `base_link` = `vehicle_mass` − massa link tambahannya, sehingga
kedua model punya massa total identik. Hasil verifikasi keduanya: massa
6,6400 kg, apung 6,644 kg, net **−0,004 kg** (near-neutral, naik sangat
pelan saat thruster mati), CoB 0,018–0,020 m di atas CoG.

Nilai lama 33,6 kg bukan hasil timbang — itu pasangan dari apung **kotak
pejal** 0,345³ (34,05 kg). Tetap ganti `vehicle_mass` dan `displaced_volume`
dengan hasil **timbang** dan **uji celup** nyata.

Konsekuensi mode `physical`: pelatnya tipis, jadi saat menyentuh dasar kolam
badan ROV tampak tenggelam sebagian ke lantai — itu harga dari apung yang
benar. Pakai `collision_mode:=hull` kalau yang dibutuhkan kontak/visual
(tapi apungnya 4× salah).

## Kompatibilitas dengan hydroships.urdf.xacro

Nama link/joint/topic di xacro sudah disamakan dengan model produksi
`hydroships.urdf.xacro`, jadi bisa dipakai bergantian tanpa mengubah
`hydroships_control` / `hydroships_gazebo`. Ke-14 link produksi ada semua
(`base_link`, `thruster_1..6`, `imu_link`, `depth_link`,
`camera_front_link`, `camera_bottom_link`, `gripper_base`,
`gripper_finger_left/right`); model baru hanya **menambah** 6 frame:
`enclosure_tube`, `enclosure_flange`, `endcap_front`, `endcap_rear`,
`gripper_tcp_frame`, `payload_hook`.

Parameter fisik (massa, densitas fluida, koefisien hidrodinamika) dibaca
dari `config/rov_params.yaml` yang sama. Yang berbeda: visual/collision
memakai mesh CAD, inersia diturunkan dari mesh (bukan dari box), posisi
thruster terukur, dan diameter propeller kini mengikuti **kelas** hasil ukur
(T200 surge → 0,076 m; T100 heave/sway → 0,048 m). Di `hydroships.urdf.xacro`
pembagian 0,076/0,048 mengikuti nomor urut, bukan kelas, sehingga
`thruster_3` dan `thruster_4` dapat diameter berbeda padahal unitnya sama.

Lihat model di RViz tanpa Gazebo:

```bash
ros2 launch hydroships_description display_kki2026.launch.py
```

## Kalau butuh mesh terpisah per komponen (lewat Fusion 360)

1. Buka `.f3z` di Fusion 360 (File > Open > Upload).
2. Untuk tiap komponen di tabel bawah: klik kanan > **Save as Mesh**,
   format **STL (binary)**, unit **mm**, refinement Medium (High untuk
   visual, Low untuk collision).
3. Simpan ke `src/hydroships_description/meshes/kki2026/` dengan nama file
   persis seperti kolom *STL* di bawah — `mesh_pkg` di xacro sudah menunjuk
   ke situ.
4. Ambil massa & inersia asli lewat **Modify > Physical Material** (set bahan
   tiap body dulu) lalu **Inspect > Center of Mass** / properties, dan ganti
   nilai placeholder di xacro.
5. Ambil origin tiap komponen lewat **Inspect > Measure** relatif terhadap
   origin `base_link`, lalu isi `xyz`/`rpy` yang sekarang masih `0 0 0`.

## Komponen top-level (XREF dari root design)

Kolom *STL* = nama file yang disarankan **kalau** nanti mesh dipecah per
komponen. Saat ini belum dipakai xacro — semua geometri masih di mesh
gabungan `base_link`.

| ID | Nama di Fusion | STL yang diharapkan | Link di xacro |
|---|---|---|---|
| 65907 | @ROV KKI 2026 NEW DESIGN revisi *(root)* | `main_frame.stl` | `base_link` |
| 65945 | base enclosure mm | `base_enclosure_mm.stl` | `enclosure_tube` |
| 65910 | base enclosure mm polos | *(varian polos, opsional)* | — |
| 65909 | FLANGE ENCLOSURE BESAR | `flange_enclosure_besar.stl` | `enclosure_flange` |
| 65953 | @ROV KKI 2026_endcap depan | `endcap_depan.stl` | `endcap_front` |
| 65914 | @ROV KKI 2026_endcap belakang | `endcap_belakang.stl` | `endcap_rear` |
| 65916 | #front casing | `front_casing.stl` | *(menyatu di base_link)* |
| 65922 | @handle almini rov kki | `handle_almini.stl` | *(menyatu di base_link)* |
| 65927 | T100 THRUSTER ROVMAKER | — *(frame saja)* | `thruster_3/4/5/6` |
| 65923 | T200-ASM-CCW-THRUSTER-R1 | — *(frame saja)* | `thruster_1/2` |
| 65919 | @cover thrstr nyar | `cover_thruster_depan.stl` | *(menyatu di base_link)* |
| 65928 | @cover thrstr nyar T100 New | *(varian T100)* | — |
| 65929 | cover thruster belakang new | `cover_thruster_belakang.stl` | *(menyatu di base_link)* |
| 65924 | cover thruster belakang new T100 | *(varian T100)* | — |
| 65920 | klip cover thruster | *(fastener cover)* | — |
| 65946 | exploreHD WL 4.0 v1 | `explorehd_wl40.stl` | `camera_front_link` / `camera_bottom_link` |
| 65918 | exploreHD-mounting-bracket | `explorehd_mounting_bracket.stl` | *(menyatu di frame kamera)* |
| 65930 | depth sensor penetrator | `depth_sensor_penetrator.stl` | `depth_link` |
| 65931 | Gripper V4 KKI 2026 | `gripper_v4_base.stl` | `gripper_base` |
| 65947 | konsep gripper kki | *(konsep lama)* | — |
| 65908 | payload | `payload.stl` | `payload_hook` |

### Sub-assembly gripper

| ID | Nama | Induk |
|---|---|---|
| 65932 | Hi-Tech Servo 646WP | Gripper V4 → penggerak `gripper_left/right_joint` |
| 65933 | hs-422_horn_disc_sws(1) | Gripper V4 |
| 65934 | Full Assamble | Gripper V4 / last woi gripper |
| 65935 | MATE ROV 2026 | Gripper V4 |
| 65950 | Gripper V2 | konsep gripper kki |
| 65951 | Full Assamble | Gripper V2 |
| 65952 | #bracket c gripper | Gripper V2 |
| 65949 | Full Assamble | konsep gripper kki |
| 65939 | Gripper Nando_ | MATE ROV 2026 |
| 65940 | last woi gripper | MATE ROV 2026 |
| 65941/65936/65938 | bracket tabung atas 1 / 3 / 2 | MATE ROV 2026 |
| 65943 | bracket lampu rov 2 | MATE ROV 2026 |
| 65944 | belt atas | MATE ROV 2026 |

### Fastener standar (tidak dijadikan link)

`65911` BHCS M3×8 · `65913` M3×10 · `65925` M3×12 · `65915` M3×16 ·
`65912` M4×10 · `65921` M4×12 · `65954` M5×16 ·
`65926` hex thin nut M3 · `65937` hex thin nut ISO 4036 M3 ·
`65955` hex thin nut M3.5 · `65917` hex thin nut M4 ·
`65942` plain washer ISO 887 A3

## Occurrence internal root frame (dari design stream)

Nama-nama ini muncul sebagai body/komponen di dalam desain root, berguna
kalau ingin memecah `main_frame.stl` jadi beberapa link terpisah:

`main frame` · `side frame` (+ mirror) · `frame samping bawah` ·
`#corner frame v2/v5/v8` (+ mirror) · `strut bar back` · `siku L` (+ mirror) ·
`kupingan samping` · `kaki 1` · `kaki new` (+ mirror ×3) ·
`bracket l modular` · `bracket gripper` · `mounting thruster depan` (×5 varian) ·
`gabungan bending samping` · `bahan drawing sideframe` ·
`tray enclosure` · `electrical enclosure` · `dudukan kunci enclosure` ·
`@rumah ESC v1` · `BALLAST` · `handle pegangan`

Elektronik yang ikut ter-embed di CAD (tidak perlu jadi link URDF):
`Raspberry Pi 4 Model B`, `Pixhawk Pro FC`.
