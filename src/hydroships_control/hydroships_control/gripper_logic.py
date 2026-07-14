"""gripper_logic — inti keputusan manipulator ROV (murni Python, tanpa ROS).

Dipisah dari node ``gripper_controller`` (yang butuh rclpy) agar logika
attach/detach & syarat jarak-aman bisa diuji headless (lihat test/test_gripper.py),
sama pola dgn ``allocation.py`` (murni) vs ``thruster_allocator.py`` (node).

DESAIN BARU (rancang ulang M5) — beda dari gripper lama:
  * Grasp fisik TIDAK mengandalkan gesekan 2 jari (versi lama tak pernah lolos
    uji grasp). Sebagai gantinya: **gz-sim DetachableJoint** membuat sambungan
    kaku ROV<->payload saat "close", dilepas saat "open"/AUTO_RELEASE.
  * "close" hanya benar-benar meng-attach bila ROV BERADA DI ATAS payload dalam
    jangkauan aman — diverifikasi dari sinyal ``/hydroships/qr_offset`` (offset
    piksel ternormalisasi + ukuran-tampak QR sebagai proxy jarak). Ini mencegah
    attach "dari jauh" yang menyeret payload menembus air (artefak sim).
  * Jari (1 DOF sederhana) hanya KOSMETIK/indikator visual buka-tutup; grasp
    sesungguhnya oleh DetachableJoint.

Kontrak semantik dipertahankan demi kompatibilitas GUI/autonomy:
    perintah "open"/"close" di /hydroships/gripper/command (String).
"""


# Sinonim perintah (kompat dgn GUI lama & FSM).
_OPEN_WORDS = frozenset(('open', 'buka', 'release', 'lepas', '0', 'false'))
_CLOSE_WORDS = frozenset(('close', 'tutup', 'grab', 'jepit', '1', 'true'))


class GripperLogic:
    """Mesin keputusan gripper. Semua waktu (``now``, ``stamp``) dalam detik.

    Parameter:
      max_offset    : |offset x/y| maksimum (ternormalisasi) agar dianggap
                      "ROV di atas payload" (0..1; 0 = tepat di tengah frame).
      min_size      : ukuran-tampak QR minimum (fraksi sisi frame) agar dianggap
                      cukup dekat untuk attach (besar = dekat).
      offset_timeout: umur maks sinyal qr_offset agar dianggap segar (s).
      jaw_open/close: target sudut jari (rad) saat terbuka/menutup (kosmetik).
    """

    def __init__(self, max_offset=0.30, min_size=0.12, offset_timeout=1.5,
                 jaw_open=0.6, jaw_close=0.0):
        self.max_offset = float(max_offset)
        self.min_size = float(min_size)
        self.offset_timeout = float(offset_timeout)
        self.jaw_open = float(jaw_open)
        self.jaw_close = float(jaw_close)
        # keadaan runtime
        self.attached = False
        self.jaw_target = self.jaw_open       # default: mulai terbuka
        self._offset = None                   # (x, y, z, stamp) atau None

    # ---- sinyal masuk ----
    def update_offset(self, x, y, z, stamp):
        """Simpan sinyal visual servo terbaru (dari /hydroships/qr_offset)."""
        self._offset = (float(x), float(y), float(z), float(stamp))

    def is_safe(self, now):
        """True bila payload ada di jangkauan aman untuk di-attach:
        offset kecil (ROV tepat di atas), ukuran-tampak cukup besar (dekat),
        dan sinyal masih segar."""
        if self._offset is None:
            return False
        x, y, z, stamp = self._offset
        if now - stamp > self.offset_timeout:
            return False
        return (abs(x) <= self.max_offset and abs(y) <= self.max_offset
                and z >= self.min_size)

    # ---- keputusan ----
    def on_command(self, cmd, now):
        """Proses perintah semantik. Kembalikan dict aksi tingkat-rendah:
            {'jaw': <sudut rad>, 'joint': 'attach'|'detach'|None,
             'state': 'open'|'closed', 'reason': <str>}
        atau None bila perintah tak dikenal. Meng-update keadaan internal."""
        c = (cmd or '').strip().lower()
        if c in _OPEN_WORDS:
            return self._do_open()
        if c in _CLOSE_WORDS:
            return self._do_close(now)
        return None

    def _do_open(self):
        self.jaw_target = self.jaw_open
        was_attached = self.attached
        self.attached = False
        return {
            'jaw': self.jaw_open,
            'joint': 'detach' if was_attached else None,
            'state': 'open',
            'reason': 'lepas payload' if was_attached else 'buka (tak ada attach)',
        }

    def _do_close(self, now):
        self.jaw_target = self.jaw_close
        if self.attached:
            return {'jaw': self.jaw_close, 'joint': None, 'state': 'closed',
                    'reason': 'sudah ter-attach'}
        if self.is_safe(now):
            self.attached = True
            return {'jaw': self.jaw_close, 'joint': 'attach', 'state': 'closed',
                    'reason': 'attach (payload dalam jangkauan)'}
        return {'jaw': self.jaw_close, 'joint': None, 'state': 'closed',
                'reason': 'tutup TAPI payload di luar jangkauan -> tak attach'}

    def force_detach(self):
        """Paksa lepas tanpa perintah (mis. saat shutdown/abort)."""
        was = self.attached
        self.attached = False
        self.jaw_target = self.jaw_open
        return was

    def startup_detach(self):
        """Aksi auto-detach saat node START.

        gz-sim Fortress SELALU meng-attach DetachableJoint saat load (payload
        langsung nge-lock ke ROV sejak sim jalan; tak bisa ditahan lewat SDF —
        lihat catatan di hydroships.urdf.xacro & PROBLEM.md). Node menerbitkan
        SATU pesan detach saat startup untuk memaksa lepas kondisi attached
        bawaan itu, sebelum menerima perintah open/close apa pun.

        Selalu mengembalikan aksi 'detach' (idempoten; aman walau gz kebetulan
        tak attach — detach pada joint yg tak ada diabaikan). Menyelaraskan
        keadaan internal ke 'open/lepas'."""
        self.attached = False
        self.jaw_target = self.jaw_open
        return {'jaw': self.jaw_open, 'joint': 'detach', 'state': 'open',
                'reason': 'auto-detach startup (lepas attach bawaan gz Fortress)'}
