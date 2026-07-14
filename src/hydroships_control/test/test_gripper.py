"""Uji logika kontroler gripper (murni, tanpa ROS) — rancang ulang M5.

Menguji keputusan attach/detach DetachableJoint & syarat jarak-aman di
GripperLogic, sejalan gaya test_allocation.py (unit murni, headless)."""

from hydroships_control.gripper_logic import GripperLogic


def _fresh(logic, x=0.0, y=0.0, z=0.5, now=100.0):
    """Suplai offset segar (stamp = now) lalu kembalikan now."""
    logic.update_offset(x, y, z, now)
    return now


def test_start_open_not_attached():
    g = GripperLogic()
    assert g.attached is False
    assert g.jaw_target == g.jaw_open


def test_close_in_range_attaches():
    g = GripperLogic(max_offset=0.3, min_size=0.12)
    now = _fresh(g, x=0.05, y=-0.05, z=0.4)
    act = g.on_command('close', now)
    assert act['joint'] == 'attach'
    assert act['state'] == 'closed'
    assert g.attached is True
    assert g.jaw_target == g.jaw_close


def test_close_out_of_range_no_attach():
    g = GripperLogic(max_offset=0.3, min_size=0.12)
    # offset besar (belum di atas payload) -> tutup tapi TIDAK attach
    now = _fresh(g, x=0.8, y=0.0, z=0.4)
    act = g.on_command('close', now)
    assert act['joint'] is None
    assert g.attached is False
    assert g.jaw_target == g.jaw_close     # jari tetap menutup (kosmetik)


def test_close_too_far_small_size_no_attach():
    g = GripperLogic(max_offset=0.3, min_size=0.2)
    # terpusat tapi ukuran-tampak kecil (jauh) -> tak attach
    now = _fresh(g, x=0.0, y=0.0, z=0.05)
    act = g.on_command('close', now)
    assert act['joint'] is None
    assert g.attached is False


def test_stale_offset_not_safe():
    g = GripperLogic(offset_timeout=1.5)
    g.update_offset(0.0, 0.0, 0.5, stamp=100.0)
    # 2 s kemudian -> sinyal basi -> tak aman
    assert g.is_safe(now=102.0) is False
    act = g.on_command('close', now=102.0)
    assert act['joint'] is None


def test_no_offset_not_safe():
    g = GripperLogic()
    assert g.is_safe(now=10.0) is False
    act = g.on_command('close', now=10.0)
    assert act['joint'] is None
    assert g.attached is False


def test_open_after_attach_detaches():
    g = GripperLogic()
    now = _fresh(g, z=0.5)
    g.on_command('close', now)
    assert g.attached is True
    act = g.on_command('open', now)
    assert act['joint'] == 'detach'
    assert g.attached is False
    assert g.jaw_target == g.jaw_open


def test_open_without_attach_no_detach():
    g = GripperLogic()
    act = g.on_command('open', now=5.0)
    assert act['joint'] is None      # tak ada yg dilepas
    assert g.attached is False


def test_double_close_attaches_once():
    g = GripperLogic()
    now = _fresh(g, z=0.5)
    a1 = g.on_command('close', now)
    a2 = g.on_command('close', now)
    assert a1['joint'] == 'attach'
    assert a2['joint'] is None       # sudah ter-attach, tak attach lagi
    assert g.attached is True


def test_command_synonyms():
    g = GripperLogic()
    now = _fresh(g, z=0.5)
    assert g.on_command('CLOSE', now)['state'] == 'closed'
    g2 = GripperLogic(); _fresh(g2, z=0.5)
    assert g2.on_command('grab', now)['joint'] == 'attach'
    assert g2.on_command('lepas', now)['joint'] == 'detach'


def test_unknown_command_returns_none():
    g = GripperLogic()
    assert g.on_command('wiggle', now=1.0) is None
    assert g.on_command('', now=1.0) is None


def test_boundary_offset_inclusive():
    g = GripperLogic(max_offset=0.3, min_size=0.12)
    now = _fresh(g, x=0.3, y=0.3, z=0.12)   # tepat di batas -> masih aman
    assert g.is_safe(now) is True
    assert g.on_command('close', now)['joint'] == 'attach'


def test_force_detach():
    g = GripperLogic()
    now = _fresh(g, z=0.5)
    g.on_command('close', now)
    assert g.force_detach() is True         # ada yg dilepas
    assert g.attached is False
    assert g.force_detach() is False        # sudah lepas
