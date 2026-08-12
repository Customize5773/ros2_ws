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
  * Dua jari opposing (masing-masing 1 DOF) hanya KOSMETIK/indikator visual
    buka-tutup dan digerakkan SEREMPAK — karena itu sudutnya cukup disimpan
    sebagai satu skalar ``jaw_target``; grasp sesungguhnya oleh DetachableJoint.

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
      max_alt_gap   : jarak vertikal maks (m) DASAR GRIPPER di atas lantai QR
                      agar boleh attach — gerbang fisik M5-D (docs/STATUS.md):
                      attach dari ketinggian jelajah membuat DetachableJoint
                      mengelas ROV ke payload yang masih di lantai. Disuplai
                      terpisah lewat ``update_altitude()`` (dari depth, yang
                      terbit terus-menerus), BUKAN lewat qr_offset — di
                      kedalaman grasp QR sudah tak terlihat.
      arm_timeout   : berapa lama (s) kondisi visual "payload di jangkauan"
                      tetap berlaku setelah terakhir terpenuhi. Selama DESCEND
                      kamera bawah turun sampai sejajar bidang QR, jadi QR
                      HILANG sebelum "close" dikirim; tanpa latch ini attach
                      tak akan pernah terpicu (kelas bug yang sama dgn 0/34
                      tick di run C1).
      jaw_open/close: target sudut kedua jari (rad) saat terbuka/menutup
                      (kosmetik). Harus berada dalam limit joint URDF
                      [-0.1, 0.5] — lihat hydroships.urdf.xacro.
    """

    def __init__(self, max_offset=0.30, min_size=0.12, offset_timeout=1.5,
                 max_alt_gap=0.08, arm_timeout=8.0, jaw_open=0.35, jaw_close=0.0):
        self.max_offset = float(max_offset)
        self.min_size = float(min_size)
        self.offset_timeout = float(offset_timeout)
        self.max_alt_gap = float(max_alt_gap)
        self.arm_timeout = float(arm_timeout)
        self.jaw_open = float(jaw_open)
        self.jaw_close = float(jaw_close)
        # keadaan runtime
        self.attached = False
        self.jaw_target = self.jaw_open       # default: mulai terbuka
        self._offset = None                   # (x, y, z, stamp, ey_target) atau None
        self._alt_gap = None                  # m dasar gripper di atas lantai QR
        self._armed_t = None                  # stamp terakhir kondisi visual aman

    # ---- sinyal masuk ----
    def update_offset(self, x, y, z, stamp, ey_target=0.0, alt_gap=None):
        """Simpan sinyal visual servo terbaru (dari /hydroships/qr_offset).

        ``ey_target`` = di mana QR SEHARUSNYA tampak di kamera bawah supaya
        GRIPPER-lah yang tepat di atas payload (lihat qr_logic.qr_ey_target).
        Default 0.0 = perilaku lama (berpatokan pusat kamera), dipertahankan
        agar pemanggil/test lama tak berubah artinya.

        ``alt_gap`` opsional = jalur lama yang menitipkan altitude lewat sinyal
        ini; jalur utama sekarang ``update_altitude()``.

        Efek samping penting: bila offset ini memenuhi syarat visual, gripper
        di-ARM pada ``stamp`` — hak attach itu bertahan ``arm_timeout`` detik
        supaya masih berlaku setelah QR hilang selama DESCEND.
        """
        self._offset = (float(x), float(y), float(z), float(stamp),
                        float(ey_target))
        if alt_gap is not None:
            self.update_altitude(alt_gap)
        if self._visual_ok():
            self._armed_t = float(stamp)

    def update_altitude(self, alt_gap):
        """Jarak vertikal DASAR GRIPPER di atas lantai QR (m), atau None bila
        belum diketahui (None = gerbang altitude nonaktif, kompat pemanggil lama)."""
        self._alt_gap = None if alt_gap is None else float(alt_gap)

    def _visual_ok(self):
        """Syarat visual murni (tanpa kesegaran & tanpa altitude)."""
        if self._offset is None:
            return False
        x, y, z, _stamp, ey_target = self._offset
        return (abs(x) <= self.max_offset
                and abs(y - ey_target) <= self.max_offset
                and z >= self.min_size)

    def is_safe(self, now):
        """True bila payload boleh di-attach. Dua gerbang, sengaja terpisah:

        1. VISUAL (arah/jangkauan): syarat offset terpenuhi baru-baru ini —
           entah dari sinyal yang masih segar (``offset_timeout``) atau dari
           latch ``arm_timeout``. Latch itu WAJIB: "close" dikirim di GRAB,
           setelah DESCEND menurunkan kamera bawah sampai sejajar bidang QR,
           jadi tak akan ada deteksi segar pada saat itu.
        2. FISIK (ketinggian): dasar gripper harus <= ``max_alt_gap`` di atas
           lantai QR. Ini yang menutup M5-D — attach dari ketinggian jelajah
           mengelas ROV ke payload yang masih tergeletak di lantai. Sinyalnya
           dari depth (terbit terus), jadi tetap hidup walau QR sudah hilang.

        Latch tanpa gerbang fisik akan mengizinkan attach dari mana saja; gerbang
        fisik tanpa latch tak akan pernah attach. Keduanya harus ada.

        PENTING — sumbu y diukur relatif ``ey_target``, bukan relatif pusat
        frame. Gripper berada 0.16 m DI DEPAN kamera bawah, jadi saat gripper
        tepat di atas payload, QR memang tampak jauh dari pusat (terukur
        ey~-0.52 pada scan_depth=0.30). Memakai |y|<=max_offset di sini berarti
        menuntut QR terpusat di KAMERA, yang bertentangan langsung dengan
        kriteria centering mission_fsm dan membuat is_safe() mustahil True
        selama APPROACH_QR berjalan benar (terukur 0/34 tick di run C1,
        2026-08-12). Lihat docs/STATUS.md M5.
        """
        if self._offset is None:
            return False
        if self._alt_gap is not None and self._alt_gap > self.max_alt_gap:
            return False
        fresh = (now - self._offset[3]) <= self.offset_timeout
        if fresh and self._visual_ok():
            return True
        return (self._armed_t is not None
                and (now - self._armed_t) <= self.arm_timeout)

    def explain(self, now):
        """Rincian TIAP sub-kondisi is_safe() pada `now` — instrumentasi
        diagnostik M5-D (DIAGNOSIS ONLY, tidak dipakai mengambil keputusan).

        Dipisah dari is_safe() supaya alur keputusan sama sekali tak berubah:
        pemanggil (gripper_controller._on_cmd) memanggil ini HANYA untuk
        mencetak log saat perintah "close" tiba. Kembalikan dict datar supaya
        gampang di-grep/di-parse dari log run."""
        if self._offset is None:
            return {'has_offset': False, 'result': False}
        x, y, z, stamp, ey_target = self._offset
        age = now - stamp
        d = {
            'has_offset': True,
            'x': x, 'y': y, 'z': z, 'ey_target': ey_target,
            'alt_gap': self._alt_gap,
            'age': age,
            'arm_age': None if self._armed_t is None else (now - self._armed_t),
            'max_offset': self.max_offset, 'min_size': self.min_size,
            'max_alt_gap': self.max_alt_gap,
            'offset_timeout': self.offset_timeout, 'arm_timeout': self.arm_timeout,
            # sub-kondisi, satu per baris keputusan di is_safe()
            'ok_alt': self._alt_gap is None or self._alt_gap <= self.max_alt_gap,
            'ok_fresh': age <= self.offset_timeout,
            'ok_x': abs(x) <= self.max_offset,
            'ok_y': abs(y - ey_target) <= self.max_offset,
            'ok_size': z >= self.min_size,
            'ok_armed': (self._armed_t is not None
                         and (now - self._armed_t) <= self.arm_timeout),
        }
        d['result'] = self.is_safe(now)
        return d

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
        # Hak attach hangus saat melepas: tanpa ini, payload BERIKUTNYA (siklus
        # AUTO_RELEASE -> DIVE -> APPROACH_QR) bisa ter-attach memakai arm sisa
        # dari payload sebelumnya — kelas bug yg sama dgn EMA basi di mission_fsm.
        self._armed_t = None
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
        self._armed_t = None
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
        self._armed_t = None
        return {'jaw': self.jaw_open, 'joint': 'detach', 'state': 'open',
                'reason': 'auto-detach startup (lepas attach bawaan gz Fortress)'}
