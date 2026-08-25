"""gui_bridge_logic — inti terjemahan GUI-ROV <-> ROS 2 (murni, tanpa ROS/UDP).

Repo GUI tim (Customize5773/GUI-ROV) TIDAK berbicara ROS 2: ia memakai
UDP-JSON + MAVLink (ArduSub). Karena transport & tipe pesannya beda total dari
kontrak topik ROS hydroships (docs/ARCHITECTURE.md), tak ada topik yg bisa
di-`remap`; dibutuhkan node adapter (gui_bridge.py). Logika terjemahan murni
dipisah ke sini agar bisa diuji headless (pola allocation.py vs node).

Kontrak GUI-ROV (dari rov_agent.py / autonomy/rov_link.py):
  * Perintah  GUI->ROV : JSON {"name": <str>, "value": <val>} via UDP.
      name in {surge, sway, yaw, heave} -> rentang KAWAT -1000..1000, 0=diam
      (lihat server/server.js clampAxis() & autonomy/fsm/mission5.py
      CommandSender.send(), keduanya GUI-ROV, dikonfirmasi baca source
      langsung 2026-08-25 -- BUKAN persen -100..100 seperti dikira semula.
      on_command() membagi /10 di titik masuk supaya gain N-per-persen di
      bawah ini tetap valid tanpa diubah.)
      name == "arm"   -> bool
      name == "light" -> bool
      name == "stop"  -> failsafe (netral)
      name == "gripper" -> "open"/"close" (opsional; GUI pakai servo PWM)
  * Telemetri ROV->GUI : JSON {heading(deg), depth(m), roll, pitch, temp,
      voltage, armed, light, mode} via UDP (tanpa "ts" — lihat build_telemetry()).

Terjemahan ke ROS hydroships (tanpa mengubah node inti):
  * axis persen -> wrench body di /hydroships/cmd_vel (Twist: linear=gaya N,
    angular=torsi N·m) — titik masuk yg sama dgn teleop_keyboard -> allocator.
  * gripper open/close -> /hydroships/gripper/command (String).
  * telemetri: yaw(odom)->heading deg, /hydroships/depth->depth, orientasi->roll/pitch.

CATATAN: gain & konvensi tanda adalah ESTIMASI; belum diverifikasi dgn GUI live.
"""

import math
from collections import deque


class DelayLine:
    """Antrian FIFO tunda-waktu (R-8: simulasi latency tether). Murni -- `now`
    diberikan pemanggil (bukan time.time()) supaya testable headless. delay_s<=0
    -> item langsung 'ready' saat push, pass-through identik ke perilaku lama."""

    def __init__(self, delay_s=0.0):
        self.delay_s = float(delay_s)
        self._q = deque()

    def push(self, item, now):
        self._q.append((now + self.delay_s, item))

    def pop_ready(self, now):
        ready = []
        while self._q and self._q[0][0] <= now:
            ready.append(self._q.popleft()[1])
        return ready


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def parse_extra_dests(csv_str):
    """csv 'host:port,host2:port2' -> [(host, port_int), ...]. Entri kosong/
    spasi diabaikan. Sama pola dgn --telem-extra di autonomy/rov_link.py,
    dipakai gui_bridge utk fanout telemetri ke >1 tujuan (mis. dashboard GUI
    + mission5.py FSM bersamaan) tanpa rebut satu telem_port."""
    dests = []
    for item in (csv_str or '').split(','):
        item = item.strip()
        if item:
            host, port = item.rsplit(':', 1)
            dests.append((host, int(port)))
    return dests


def _num(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return 0.0


class GuiBridgeLogic:
    """Terjemahan stateless-ish GUI<->ROS. Simpan axis manual terakhir & status.

    Gain memetakan persen joystick (-100..100) ke gaya/torsi body (N, N·m).
    Dipilih agar 100% ~ mendekati batas thruster (≈50 N) tanpa langsung jenuh.
    """

    def __init__(self, surge_gain=0.40, sway_gain=0.40, heave_gain=0.30,
                 yaw_gain=0.12, mode='manual', hook_max_age=1.0):
        self.surge_gain = float(surge_gain)
        self.sway_gain = float(sway_gain)
        self.heave_gain = float(heave_gain)
        self.yaw_gain = float(yaw_gain)
        self.mode = mode  # vestigial: build_telemetry() kini menurunkan mode dari mission_state
        self.hook_max_age = float(hook_max_age)
        # axis manual terakhir (persen)
        self.axes = {'surge': 0.0, 'sway': 0.0, 'yaw': 0.0, 'heave': 0.0}
        self.armed = False
        self.light = False

    # ---- perintah GUI -> aksi ROS ----
    def on_command(self, name, value):
        """Proses satu pesan {name,value} GUI. Kembalikan dict aksi:
            {'wrench': (Fx,Fy,Fz,Mz)} bila axis/stop mengubah gerak,
            {'gripper': 'open'|'close'} bila perintah manipulator,
            {'arm': bool} / {'light': bool} untuk status,
            atau {} bila tak menghasilkan aksi ROS langsung."""
        n = (name or '').strip().lower()
        if n in self.axes:
            # value datang dalam skala kawat GUI-ROV (-1000..1000) -> /10 jadi
            # persen sebelum clamp, supaya gain N-per-persen di bawah tetap
            # berarti "100% ~ batas thruster", bukan disaturasi di 1/10 rentang.
            self.axes[n] = clamp(_num(value) / 10.0, -100.0, 100.0)
            return {'wrench': self.wrench()}
        if n == 'stop':
            self.axes = {k: 0.0 for k in self.axes}
            self.armed = False
            return {'wrench': (0.0, 0.0, 0.0, 0.0), 'arm': False}
        if n == 'arm':
            self.armed = bool(value)
            # saat disarm, netralkan gerak
            return {'arm': self.armed} if self.armed else {'arm': False,
                                                           'wrench': (0.0, 0.0, 0.0, 0.0)}
        if n == 'light':
            self.light = bool(value)
            return {'light': self.light}
        if n == 'gripper':
            g = str(value).strip().lower()
            if g in ('open', 'close'):
                return {'gripper': g}
            return {}
        if n == 'start_mission':
            return {'mission_start': True}
        if n == 'abort_mission':
            return {'mission_abort': True}
        return {}   # mode/pid/pool config dsb. tak dipetakan ke ROS di sini

    def wrench(self):
        """axis persen terakhir -> wrench body (Fx, Fy, Fz, Mz).
        Bila tidak armed, kembalikan nol (failsafe)."""
        if not self.armed:
            return (0.0, 0.0, 0.0, 0.0)
        a = self.axes
        return (
            self.surge_gain * a['surge'],
            self.sway_gain * a['sway'],
            self.heave_gain * a['heave'],
            self.yaw_gain * a['yaw'],
        )

    # ---- telemetri ROS -> GUI ----
    @staticmethod
    def yaw_to_heading_deg(yaw_rad):
        """yaw REP-103 (rad, CCW dari +x) -> heading GUI (derajat 0..360)."""
        return math.degrees(yaw_rad) % 360.0

    def build_telemetry(self, yaw_rad=None, depth_m=None, roll=None, pitch=None,
                        voltage=0.0, temp=0.0, mission_state=None, qr_result=None,
                        hook_offset=None, hook_age=None, gripper_status=None,
                        gripper_state=None):
        """Susun dict telemetri utk GUI (JSON). Nilai None -> 0/'' agar GUI aman."""
        mode = ('auto' if mission_state not in (None, 'IDLE', 'DONE', 'ABORT')
                 else 'manual')
        hex_, hey_, hsize_ = hook_offset if hook_offset is not None else (0.0, 0.0, 0.0)
        hook_fresh = (hook_offset is not None and hook_age is not None
                      and hook_age <= self.hook_max_age)
        return {
            'heading': self.yaw_to_heading_deg(yaw_rad) if yaw_rad is not None else 0.0,
            'depth': float(depth_m) if depth_m is not None else 0.0,
            'roll': math.degrees(roll) if roll is not None else 0.0,
            'pitch': math.degrees(pitch) if pitch is not None else 0.0,
            'temp': float(temp),
            'voltage': float(voltage),
            'armed': bool(self.armed),
            'light': bool(self.light),
            'mode': mode,
            'mission_state': mission_state if mission_state is not None else 'IDLE',
            'qr_result': qr_result if qr_result is not None else '',
            'hook_ex': float(hex_),
            'hook_ey': float(hey_),
            'hook_size': float(hsize_),
            'hook_fresh': bool(hook_fresh),
            'gripper_status': gripper_status if gripper_status is not None else '',
            'gripper_state': gripper_state if gripper_state is not None else '',
        }
