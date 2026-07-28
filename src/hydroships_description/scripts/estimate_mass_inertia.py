#!/usr/bin/env python3
"""estimate_mass_inertia — hitung ESTIMASI massa & inertia ROV dari geometri.

Alat bantu untuk mengisi ``config/rov_params.yaml`` saat dimensi / material
HYDROships final tersedia. Model dasar: **kotak pejal (solid box)** dengan
massa jenis material seragam, opsional ditambah **massa titik** komponen
(thruster, baterai, dsb.) via teorema sumbu sejajar (parallel axis theorem).

Konvensi frame = ROS REP-103 (x maju, y kiri, z atas). Inertia dihitung di
pusat massa gabungan (CoG), sesuai konvensi ``<inertial>`` URDF.

CATATAN PENTING
---------------
Skrip ini TIDAK mengarang data fisik ROV. Ia hanya menurunkan angka dari
INPUT yang Anda berikan (dimensi, massa/densitas, komponen). Selama input
masih placeholder (mis. dimensi bbox desain + tebakan densitas), keluarannya
tetap ESTIMASI — tandai demikian di rov_params.yaml. Isi input dengan hasil
pengukuran nyata untuk mendapat parameter final, lalu re-run & tempel.

Tanpa dependensi eksternal (pure Python) agar bisa dijalankan di mana saja.

Contoh
------
  # Kotak pejal dari total massa (near-neutral, seperti nilai model saat ini):
  ./estimate_mass_inertia.py --dims 0.345 0.345 0.286 --mass 33.6

  # Dari densitas material (massa = densitas * volume):
  ./estimate_mass_inertia.py --dims 0.345 0.345 0.286 --density 100.0

  # Tambah 6 thruster 0.05 kg (contoh 2 titik; ulang --point-mass sesuai butuh):
  ./estimate_mass_inertia.py --dims 0.345 0.345 0.286 --mass 33.6 \
      --point-mass 0.05 0.13 -0.137 0.034 --point-mass 0.05 -0.136 0.0 0.04

  # Cek regresi cepat (reproduksi angka rov_params.yaml model saat ini):
  ./estimate_mass_inertia.py --self-test
"""

import argparse
import sys


def box_inertia(mass, sx, sy, sz):
    """Tensor inertia kotak pejal (di pusatnya), massa seragam.

    Ixx = m/12 (sy^2 + sz^2), dst. Kembalikan (ixx, iyy, izz)."""
    c = mass / 12.0
    ixx = c * (sy * sy + sz * sz)
    iyy = c * (sx * sx + sz * sz)
    izz = c * (sx * sx + sy * sy)
    return ixx, iyy, izz


def combine(components):
    """Gabungkan daftar komponen -> (massa_total, cog, inertia_full_di_cog).

    components: list of dict {m, com:(x,y,z), I:(ixx,iyy,izz,ixy,ixz,iyz)}
    di mana I adalah inertia komponen DI PUSAT MASSANYA SENDIRI. Massa titik
    memakai I nol. Memakai teorema sumbu sejajar untuk memindahkan semua ke
    CoG gabungan. Kembalikan inertia penuh (ixx,iyy,izz,ixy,ixz,iyz)."""
    total_m = sum(c['m'] for c in components)
    if total_m <= 0.0:
        raise ValueError('massa total harus > 0')
    cx = sum(c['m'] * c['com'][0] for c in components) / total_m
    cy = sum(c['m'] * c['com'][1] for c in components) / total_m
    cz = sum(c['m'] * c['com'][2] for c in components) / total_m

    ixx = iyy = izz = ixy = ixz = iyz = 0.0
    for c in components:
        m = c['m']
        i = c.get('I', (0.0, 0.0, 0.0, 0.0, 0.0, 0.0))
        # offset komponen dari CoG gabungan
        dx = c['com'][0] - cx
        dy = c['com'][1] - cy
        dz = c['com'][2] - cz
        ixx += i[0] + m * (dy * dy + dz * dz)
        iyy += i[1] + m * (dx * dx + dz * dz)
        izz += i[2] + m * (dx * dx + dy * dy)
        # produk inersia (parallel axis): -m*dx*dy, dst.
        ixy += i[3] - m * dx * dy
        ixz += i[4] - m * dx * dz
        iyz += i[5] - m * dy * dz
    return total_m, (cx, cy, cz), (ixx, iyy, izz, ixy, ixz, iyz)


def build_components(dims, mass, density, point_masses):
    sx, sy, sz = dims
    volume = sx * sy * sz
    if mass is None:
        if density is None:
            raise ValueError('berikan --mass atau --density')
        mass = density * volume
    ixx, iyy, izz = box_inertia(mass, sx, sy, sz)
    comps = [{
        'm': mass,
        'com': (0.0, 0.0, 0.0),
        'I': (ixx, iyy, izz, 0.0, 0.0, 0.0),
    }]
    for pm in point_masses:
        m, x, y, z = pm
        comps.append({'m': m, 'com': (x, y, z), 'I': (0.0,) * 6})
    return comps, volume


def format_yaml(total_m, cog, inertia, volume, fluid_density):
    ixx, iyy, izz, ixy, ixz, iyz = inertia
    buoyant_mass = volume * fluid_density
    net = total_m - buoyant_mass
    sign = 'POSITIF (mengapung)' if net < 0 else ('NETRAL' if abs(net) < 1e-6 else 'NEGATIF (tenggelam)')
    lines = [
        '# --- Tempel blok berikut ke config/rov_params.yaml (bagian massa/inertia) ---',
        '# Dihasilkan estimate_mass_inertia.py — TANDAI [estimate] sampai input = data ukur nyata.',
        'base_mass: %.5f' % total_m,
        'cog: {x: %.5f, y: %.5f, z: %.5f}' % (cog[0], cog[1], cog[2]),
        'inertia:',
        '  ixx: %.5f' % ixx,
        '  iyy: %.5f' % iyy,
        '  izz: %.5f' % izz,
        '  ixy: %.5f' % ixy,
        '  ixz: %.5f' % ixz,
        '  iyz: %.5f' % iyz,
        '',
        '# Referensi apung: volume=%.6f m^3, massa air terpindah=%.4f kg (rho=%.0f)' % (
            volume, buoyant_mass, fluid_density),
        '# Net = massa - apung = %+.4f kg -> buoyancy %s' % (net, sign),
    ]
    return '\n'.join(lines)


def _parse_args(argv):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--dims', nargs=3, type=float, metavar=('X', 'Y', 'Z'),
                    default=[0.345, 0.345, 0.286],
                    help='dimensi bbox kotak pejal (m), default bbox desain HYDROships')
    g = ap.add_mutually_exclusive_group()
    g.add_argument('--mass', type=float, default=None, help='massa total kotak (kg)')
    g.add_argument('--density', type=float, default=None,
                   help='massa jenis material (kg/m^3); massa = densitas*volume')
    ap.add_argument('--point-mass', nargs=4, type=float, action='append',
                    metavar=('M', 'X', 'Y', 'Z'), default=[],
                    help='tambah massa titik komponen (kg, posisi m); boleh diulang')
    ap.add_argument('--fluid-density', type=float, default=1000.0,
                    help='massa jenis air untuk referensi apung (kg/m^3)')
    ap.add_argument('--self-test', action='store_true',
                    help='verifikasi reproduksi angka model saat ini lalu keluar')
    return ap.parse_args(argv)


def _self_test():
    # Model saat ini: kotak 0.345^3(z=0.286), massa 33.6 kg, CoG di origin.
    comps, vol = build_components([0.345, 0.345, 0.286], 33.6, None, [])
    m, cog, inertia = combine(comps)
    ixx, iyy, izz = inertia[0], inertia[1], inertia[2]
    assert abs(m - 33.6) < 1e-9, m
    assert all(abs(cog[k]) < 1e-12 for k in range(3)), cog
    # nilai acuan (m/12 * ...) — cocokkan dgn rov_params.yaml
    assert abs(ixx - 0.56230) < 1e-4, ixx
    assert abs(iyy - 0.56230) < 1e-4, iyy
    assert abs(izz - 0.66654) < 1e-4, izz
    # produk inersia harus nol (simetris)
    assert all(abs(inertia[k]) < 1e-12 for k in (3, 4, 5)), inertia
    print('self-test OK: ixx=%.5f iyy=%.5f izz=%.5f (cocok rov_params.yaml)'
          % (ixx, iyy, izz))


def main(argv=None):
    args = _parse_args(sys.argv[1:] if argv is None else argv)
    if args.self_test:
        _self_test()
        return 0
    comps, volume = build_components(args.dims, args.mass, args.density, args.point_mass)
    total_m, cog, inertia = combine(comps)
    print(format_yaml(total_m, cog, inertia, volume, args.fluid_density))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
