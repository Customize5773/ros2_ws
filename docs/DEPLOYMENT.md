# Deployment Checklist — Hari-H Kontes

Checklist operasional untuk mencegah human error di lapangan (Waduk PDAM Kota
Bengkalis, misi kontes KKI 2026). Berbeda dari `docs/HOW-TO-RUN.txt` (panduan
teknis menjalankan sim di mesin dev) — dokumen ini adalah **urutan tindakan hari-H**
dengan checkbox, ditulis untuk dibawa/dicetak dan dicentang langsung di lapangan.

> **Catatan status:** dokumen ini ditulis dengan asumsi software sudah tertransfer
> ke jalur hardware fisik (lihat `docs/HARDWARE.md` untuk gap yang masih harus
> ditutup). Sebagian item di bawah (mis. driver sensor/kamera fisik) baru relevan
> **setelah** gap hardware itu selesai — sebelum itu, gunakan checklist ini untuk
> sesi uji coba kolam sendiri (bukan kontes) dengan menyesuaikan bagian yang masih
> simulasi.

Durasi misi kontes resmi: **maksimal 20 menit per run** (5 menit persiapan, 10
menit eksekusi misi, 5 menit evakuasi/meninggalkan lokasi) — lihat panduan lomba
§4.7.3. Checklist ini dirancang supaya fase 5-menit persiapan cukup.

---

## H-1 (sehari sebelum, di penginapan/base camp)

- [ ] `git pull` workspace terbaru, cek `docs/STATUS.md` untuk bug blocking yang
      belum diperbaiki (jangan berasumsi status sama dengan sesi latihan terakhir).
- [ ] `colcon build` bersih dari nol (`rm -rf build/ install/ log/` lalu build ulang)
      — pastikan tidak ada state build basi yang lolos ke hari-H.
- [ ] `colcon test --packages-select hydroships_control` — semua test harus lolos
      sebelum berangkat. Lihat `docs/CI_CD.md`.
- [ ] Charge penuh semua baterai (ROV + panel operator/laptop GCS + power bank cadangan).
- [ ] Cek fisik: enclosure kedap air (tanpa retak/celah), semua konektor Anderson/XT60
      terpasang rapat, tidak ada kabel terkelupas di tether.
- [ ] Bungkus tether dengan rapi (gulungan tidak kusut) — cegah waktu terbuang saat
      setup hari-H.
- [ ] Siapkan payload QR cadangan (fisik) dan pastikan huruf A/B/C/D sesuai spesifikasi
      panduan lomba (ukuran 5×3×10cm, lihat panduan §4.7.1 halaman 52).
- [ ] Backup config (`gains.yaml`, param `mission_fsm` hasil tuning terakhir) — simpan
      salinan di luar laptop utama (USB/cloud) untuk recovery cepat bila laptop utama
      bermasalah.

## H-0, saat tiba di venue (sebelum giliran/slot waktu tim)

- [ ] Uji kekedapan air ROV (waterproof test) di air dangkal/ember — **sebelum**
      dicelup ke waduk penuh. Lihat prosedur di Bab 2.6.4 proposal / flowchart
      pengujian.
- [ ] Nyalakan semua sistem, cek indikator daya (24V utama, step-down 12V, UBEC 5V)
      — semua harus menyala stabil tanpa drop tegangan mencurigakan.
- [ ] `ros2 topic list` di panel operator — pastikan semua topic inti muncul:
      `/hydroships/odom`, `/hydroships/depth`, `/hydroships/camera_front/image_raw`,
      `/hydroships/camera_bottom/image_raw`, `/hydroships/qr_result`,
      `/hydroships/hook_offset`.
- [ ] Cek GUI/GCS menerima telemetri (dua kamera live, altitude, trajectory map
      terisi) — lihat `docs/GUI-INTEGRATION.md` untuk kontrak data.
- [ ] Uji tombol **Emergency Stop** — wajib memastikan menghentikan seluruh thruster
      secara instan (persyaratan panduan lomba §4.7.2, item "Emergency Stop": WAJIB).
      **Jangan mulai misi tanpa mengonfirmasi E-stop berfungsi.**
- [ ] Kalibrasi/verifikasi compass/heading — pastikan `heading=0` menghadap arah yang
      dipahami operator (bukan arah acak sisa run sebelumnya).

## Fase Persiapan (5 menit, saat giliran run dimulai)

- [ ] Konfirmasi ROV pada posisi start yang ditentukan panitia (posisi hook A/B/C/D
      diacak panitia — lihat panduan lomba §4.7 gambar layout, posisi diacak tiap run).
- [ ] Jalankan launch file misi: `ros2 launch hydroships_bringup hydroships_mission.launch.py`
      (atau varian GUI: `hydroships_gui.launch.py` sesuai keputusan tim — lihat
      `docs/HOW-TO-RUN.txt` §3C/3D untuk argumen lengkap).
- [ ] Verifikasi state FSM awal = `IDLE` atau `DIVE` sesuai rencana (bukan sisa state
      dari run sebelumnya) — cek via `ros2 topic echo /hydroships/fsm_state` bila
      tersedia, atau log node `mission_fsm`.
- [ ] Konfirmasi kedua kamera menampilkan gambar jernih (bukan hitam/freeze) di GCS.
- [ ] Briefing singkat ke seluruh anggota tim: siapa pegang E-stop, siapa pegang
      panel kontrol, siapa mengawasi tether.

## Fase Eksekusi Misi (10 menit)

- [ ] Trigger mulai misi (`ros2 topic pub .../mission/start_autonomous` atau tombol
      GUI, sesuai desain final).
- [ ] Pantau progres state FSM di layar operator secara berkelanjutan — bila macet
      di satu state lebih lama dari timeout yang diharapkan (lihat
      `docs/CONFIG_REFERENCE.md` §5 untuk nilai `t_*`), siap intervensi manual.
- [ ] Bila autonomous macet/gagal, punya rencana fallback **remotely** yang sudah
      dilatih (nilai remotely lebih rendah dari autonomous tapi tetap dapat skor —
      lihat rubrik panduan lomba §4.7.4, item 5: 40% autonomous vs 10% remotely).
- [ ] Jaga tether tidak tersangkut struktur kolam/tim lain.
- [ ] Catat waktu tersisa secara verbal (mis. "5 menit lagi") ke seluruh tim.

## Fase Evakuasi (5 menit)

- [ ] Setelah misi selesai/waktu habis, angkat ROV dari air segera.
- [ ] Matikan daya utama sebelum melepas konektor (cegah short/spark).
- [ ] Bersihkan/keringkan ROV dan tether.
- [ ] Tinggalkan area kontes sesuai instruksi panitia agar tidak menghalangi tim
      berikutnya.

## Pasca-run

- [ ] Cek log/rekaman untuk debugging sebelum run berikutnya (bila ada run kedua/final).
- [ ] Catat state terakhir sebelum ABORT/DONE, dan hasil skor per-misi bila
      diumumkan panitia, untuk evaluasi cepat sebelum slot run berikutnya.
- [ ] Bila ada waktu antar-run, jalankan sub-checklist "H-0" ulang secara ringkas
      (terutama E-stop dan konektor) — jangan asumsikan kondisi tidak berubah.

---

## Prasyarat software sebelum checklist ini bisa dipakai penuh di hari-H

Checklist di atas mengasumsikan software sudah berjalan andal. Sebelum hari-H,
pastikan item berikut di `docs/STATUS.md` sudah **tidak** berstatus blocking:

- Bug `mission_fsm` tidak publish "close" ke `/hydroships/gripper/command` di state
  `GRAB` (grasp tidak pernah benar-benar terjadi) — lihat `docs/TROUBLESHOOTING.md`.
- `NAV_WALL` tidak konvergen ke `nav_tol`, stalls dan timeout ke `ABORT`.
- Driver hardware fisik (ESC, sensor kedalaman, kamera, servo gripper) — lihat
  `docs/HARDWARE.md` §3 untuk daftar lengkap yang harus selesai dulu.

Jangan berangkat ke kontes dengan bug blocking status di atas belum diverifikasi
selesai — checklist hari-H ini untuk mencegah *human error operasional*, bukan
pengganti verifikasi *software correctness* yang harus selesai sebelumnya.
