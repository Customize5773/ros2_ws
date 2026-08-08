# tools/p0-experiments — skrip investigasi P0-1

Skrip yang dipakai untuk menghasilkan evidence di
[docs/P0-1-BASELINE.md](../../docs/P0-1-BASELINE.md). Disimpan agar P0-1d dan P0-1e
dapat **direproduksi**, bukan hanya dipercaya dari laporan.

Bukan bagian dari paket ROS mana pun — dijalankan langsung dengan `python3`/`bash`,
tidak ikut `colcon build`.

## Isi

| Berkas | Guna |
|---|---|
| `trim_audit.py` | Audit statis CoG/CoB & trim dari model. **Tanpa simulator.** |
| `driver.py` | Rig open-loop (P0-1d): menerbitkan jadwal thrust ke `/hydroships/thruster_N/thrust`, mencatat pose+twist. |
| `reduce_openloop.py` | Reduksi data P0-1d; memotong tiap segmen pada kontak lantai pertama. |
| `recorder.py` | Perekam rantai kendali (P0-1e). **Subscribe-only** — tidak menerbitkan apa pun. |
| `run_mission.sh` | Satu run regresi DIVE tertutup: launch + rekam + gate + teardown. |
| `gate_mission.sh` | Gerbang anti-kontaminasi untuk rig integrasi. |
| `reduce_mission.py` | Reduksi data P0-1e; timing ambang DIVE, pitch/roll, fidelity allocator. |

## Cara pakai

Data ditulis ke direktori kerja saat ini, atau ke `$P0_DATA_DIR` bila diset.
Jalankan dari root repo setelah `colcon build`.

```bash
# audit statis, tidak butuh sim
python3 tools/p0-experiments/trim_audit.py

# P0-1d — karakterisasi open-loop (pool_empty, TANPA stabilizer/allocator/FSM)
ros2 launch hydroships_gazebo sim.launch.py headless:=true world:=pool_empty.sdf \
    rov_random_spawn:=false rov_x:=0.0 rov_y:=0.0 rov_z:=-2.5
python3 tools/p0-experiments/driver.py B_pinv.csv B_pinv
python3 tools/p0-experiments/reduce_openloop.py

# P0-1e — regresi DIVE tertutup (kki_arena, stack penuh)
export P0_DATA_DIR=/tmp/p0-1e && mkdir -p "$P0_DATA_DIR"
bash tools/p0-experiments/run_mission.sh R1
bash tools/p0-experiments/run_mission.sh R4 rov_random_spawn:=false \
     rov_x:=0.0 rov_y:=0.0 rov_z:=-0.5
cd "$P0_DATA_DIR" && python3 -            # reduce_mission.py membaca R1..R4 dari cwd
```

Jadwal thrust yang tersedia di `driver.py`: `A_trim`, `B_equal`, `B_pinv`, `C_t1/t2/t6`,
`C_all`, `E_dive_equal_14`, `E_dive_pinv_14`, `E_dive_pinv_7`.

## Aturan yang membuat angkanya bisa dipercaya

Diikuti ketat selama P0-1; melanggarnya menghasilkan kesimpulan yang salah (dan memang
sempat terjadi — lihat bagian "pelajaran metodologi" di dokumen baseline).

1. **Pisahkan tujuan world.** `pool_empty` untuk fisik/open-loop (lantai −5 m, tanpa
   rintangan), `kki_arena` untuk integrasi/misi. Jangan dicampur dalam satu eksperimen.
2. **Jalankan gerbang sebelum tiap run.** Tepat satu server Gazebo, `/hydroships/odom`
   terbit, dan komposisi node sesuai jenis rig. Run yang gagal gerbang ditandai
   `CONTAMINATED` dan **tidak diinterpretasi**.
3. **Buang data setelah kontak.** Kontak lantai: `z ≈ −0.809 m` (`kki_arena`),
   `z ≈ −4.829 m` (`pool_empty`). Tembus permukaan: `z + 0.111 ≥ 0`.
4. **Turunkan kecepatan dari posisi odom, bukan `twist`.** Keduanya pernah bertentangan
   sampai faktor 13×.
5. **Cek runtime, bukan hanya source.** Parameter aktif lewat `ros2 param dump`, geometri
   lewat SDF hasil spawn (`ign sdf -p`) — bukan dari URDF/YAML saja.
