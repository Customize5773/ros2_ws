# ARENA ROV KKI 2026
---

## Status Milestone

Ringkasan cepat progres (detail & blocker: [`docs/STATUS.md`](docs/STATUS.md)).
Legenda: ✅ terverifikasi sim · 🧪 kode ada, verifikasi runtime tertunda · OPEN gap desain/hardware.

| Milestone | Status | Ringkas |
|-----------|--------|---------|
| M1 — Kendali dasar & thruster allocation | ✅ | Wrench → allocator (damped pinv) → 6 thruster; odom umpan balik. |
| M2 — Stabilizer PID (depth/heading) | ✅ | PID depth & heading hold. |
| M3 — Sensor & persepsi | 🧪 | `depth_publisher`/`qr_detector`/`camera_info` selesai; keterbacaan QR runtime tertunda. |
| M4 — Arena / world | ✅ | `kki_arena.sdf` (payload+QR+hook); pemetaan hook A–D masih VERIFY. |
| M5 — Manipulator | 🧪 | DetachableJoint + jari kosmetik; grasp fisik belum diuji sim. |
| M6 — Autonomy (FSM misi) | 🧪 | `mission_fsm` jalan; `APPROACH_HOOK` servo PD; tuning/sim run tertunda. |
| M7 — Integrasi GUI tim | 🧪 | Adapter `gui_bridge` + `hook_detector`; belum diuji live end-to-end. |

> ⚠️ Mesin dev tidak punya ROS2/Gazebo → semua item 🧪 menunggu satu run sim ber-GPU.
> Checklist uji berprioritas: [`docs/VERIFICATION-CHECKLIST.md`](docs/VERIFICATION-CHECKLIST.md).

---

## Instalasi Dependensi

Beberapa node Python pada `hydroships_control` (mis. `qr_detector.py`, `allocation.py`, `stabilizer.py`) memerlukan `opencv` dan `numpy`. Install dependensi sistem dan Python berikut sebelum menjalankan simulasi:

```bash
sudo apt install python3-opencv python3-numpy
pip install -r requirements.txt
```
---

<img width="535" height="497" alt="Screenshot from 2026-07-04 18-39-12" src="https://github.com/user-attachments/assets/f9793a9f-5241-45bb-a489-960bd04cdac1" />

---

<img width="1068" height="992" alt="n4lqf0n4lqf0n4lq" src="https://github.com/user-attachments/assets/51577018-6999-48ee-bda7-46bd470d44bd" />


---

<img width="349" height="376" alt="Screenshot from 2026-07-04 18-39-55" src="https://github.com/user-attachments/assets/68b618bd-47b1-4267-be59-ffb37ba8fbcf" />

---

<img width="326" height="323" alt="Screenshot from 2026-07-04 18-40-40" src="https://github.com/user-attachments/assets/3bcfa80f-cb75-4f09-9ab0-03d4dd5f80ac" />

---

