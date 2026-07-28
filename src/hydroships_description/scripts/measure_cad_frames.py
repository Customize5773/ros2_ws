#!/usr/bin/env python3
"""measure_cad_frames — ukur frame & properti fisik ROV dari mesh CAD.

Menurunkan angka yang dipakai ``urdf/rov_kki2026_new_design.urdf.xacro``
langsung dari ``DOKUMENTASI ROV/@ROV KKI 2026 NEW DESIGN.stl`` sehingga
konstanta di URDF tidak "jatuh dari langit" dan bisa dihitung ulang kalau
CAD-nya berubah.

Yang dihitung
-------------
* Transform mesh -> base_link (rotasi + translasi).
* Posisi & sumbu 6 thruster, dari analisis connected-component mesh.
* CoM dan tensor inersia per kilogram (asumsi densitas seragam).
* Volume solid, convex hull, dan estimasi volume tergeser (apung).
* Mesh turunan visual + collision (opsional, --export-meshes).

KENAPA MESH DIPUTAR
-------------------
Origin STL BUKAN konvensi ROS. Sumbu memanjang ROV ada di +Y/-Y mesh, dan
haluan (gripper/payload) ada di -Y. REP-103 menuntut x maju, jadi mesh
diputar +90 deg terhadap Z:  x_base = -y_mesh,  y_base = x_mesh,  z_base = z_mesh.
Origin lalu digeser ke pusat massa mesh supaya base_link = CoM.

Bukti orientasi (semua terukur, lihat --verbose):
  * tabung enclosure (OD 130 mm, panjang 210 mm) sumbunya di Y mesh -> fore/aft;
  * 3 thruster bersumbu Z mesh (heave), 2 bersumbu Y mesh (surge),
    1 bersumbu X mesh (sway) -> pola 3-2-1 khas ROV;
  * struktur gripper/payload ada di ujung -Y mesh -> haluan.

CATATAN: skrip ini TIDAK mengarang data. Semua keluaran diturunkan dari file
STL. Yang TIDAK bisa diturunkan darinya (massa nyata, densitas tiap komponen,
koefisien hidrodinamika, arah putar propeller) tetap harus diukur/diuji.

Butuh: trimesh, numpy. Untuk --export-meshes: fast_simplification.

Contoh
------
  ./measure_cad_frames.py --stl "DOKUMENTASI ROV/@ROV KKI 2026 NEW DESIGN.stl"
  ./measure_cad_frames.py --stl ... --export-meshes --out-dir ../meshes/kki2026
"""

import argparse
import sys

import numpy as np

# Ambang klasifikasi body (mm). Diturunkan dari inspeksi mesh: rumah thruster
# T100 ~73x75x68 mm, duct T200 ~90x17x90 mm, baut/mur < 25 mm diagonal.
MIN_BODY_DIAG = 25.0        # body lebih kecil dianggap fastener
THRUSTER_MIN_FACES = 5000   # rumah/duct thruster punya ribuan face


def load_mesh(path):
    import trimesh
    m = trimesh.load(path)
    if not hasattr(m, 'faces'):
        raise SystemExit('bukan mesh tunggal: %s' % path)
    m.merge_vertices()
    return m


def component_labels(mesh):
    import trimesh
    return trimesh.graph.connected_component_labels(
        mesh.face_adjacency, node_count=len(mesh.faces))


def bodies(mesh, labels, min_faces=100):
    """Ringkas tiap connected component -> dict bbox/center/extent."""
    out = []
    for i in range(labels.max() + 1):
        idx = np.where(labels == i)[0]
        if len(idx) < min_faces:
            continue
        v = mesh.vertices[np.unique(mesh.faces[idx])]
        lo, hi = v.min(0), v.max(0)
        out.append({'id': i, 'nf': len(idx), 'lo': lo, 'hi': hi,
                    'c': (lo + hi) / 2.0, 'e': hi - lo})
    return out


def _is_disc(b):
    """Cakram/duct: dua sisi lebar mirip, sisi ketiga jauh lebih tipis."""
    thin, mid, wide = np.sort(b['e'])
    return (50.0 < wide < 110.0 and mid > 45.0
            and thin < 0.45 * wide and abs(mid - wide) < 0.25 * wide)


def _is_housing(b):
    """Rumah thruster: bongkah kompak 45-110 mm di ketiga sumbu."""
    return b['nf'] >= THRUSTER_MIN_FACES and all(45.0 < x < 110.0 for x in b['e'])


def find_thrusters(bods):
    """Kenali 6 thruster dari bentuk body. Koordinat dikembalikan dlm MESH (mm).

    Dua tahap, karena satu unit thruster terpecah jadi beberapa body:
      1. Titik unit = body RUMAH (bongkah kompak). Body dgn jarak < 60 mm
         dianggap unit yang sama dan digabung.
      2. Sumbu dorong = sumbu TERTIPIS dari cakram/duct terdekat (< 90 mm)
         milik unit itu — normal cakram propeller = arah dorong.
    Pelat/tray besar tidak lolos karena dibatasi < 110 mm di tiap sumbu."""
    discs = [b for b in bods if _is_disc(b)]
    units = []
    for b in sorted((b for b in bods if _is_housing(b)), key=lambda x: -x['nf']):
        if any(np.linalg.norm(b['c'] - u['c']) < 60.0 for u in units):
            continue
        near = [d for d in discs if np.linalg.norm(d['c'] - b['c']) < 90.0]
        if not near:
            continue
        disc = min(near, key=lambda d: np.linalg.norm(d['c'] - b['c']))
        units.append({'c': b['c'], 'axis': int(np.argmin(disc['e'])),
                      'e': b['e'], 'duct': disc['e'], 'nf': b['nf']})
    return units


def mesh_to_base(p, com):
    """Mesh (mm) -> base_link (m). Rz(+90 deg) lalu geser CoM ke origin."""
    p = np.asarray(p, dtype=float)
    q = np.array([-p[1], p[0], p[2]])
    c = np.array([-com[1], com[0], com[2]])
    return (q - c) / 1000.0


def rotate_axis(axis_index):
    """Sumbu mesh (0=x,1=y,2=z) -> vektor sumbu di frame base."""
    return {0: (0.0, 1.0, 0.0),   # x mesh -> y base (sway)
            1: (1.0, 0.0, 0.0),   # y mesh -> x base (surge)
            2: (0.0, 0.0, 1.0)}[axis_index]   # z tetap (heave)


def inertia_per_kg(mesh):
    """Tensor inersia per kg di CoM, sudah dirotasi ke frame base (m^2)."""
    mesh.density = 1.0 / mesh.volume          # normalisasi -> massa 1 kg
    tensor = mesh.moment_inertia / 1e6        # mm^2 -> m^2
    rz = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    return rz @ tensor @ rz.T


def displaced_volume(mesh, tube_od=0.130, tube_len=0.210, wall=0.004):
    """Estimasi volume air tergeser (m^3).

    = volume solid semua body + rongga TERTUTUP tabung enclosure. Rongga itu
    tidak masuk hitungan volume solid karena tabung dimodelkan sbg cangkang,
    padahal di air ia menggeser volume luarnya."""
    solid = mesh.volume / 1e9
    r_in = tube_od / 2.0 - wall
    cavity = np.pi * r_in ** 2 * (tube_len - 2 * wall)
    return solid, cavity, solid + cavity


def export_meshes(mesh, labels, out_dir, visual_target=40000):
    """Tulis mesh visual (fastener dibuang + decimate) & collision (convex hull)."""
    import fast_simplification
    import trimesh
    keep = np.zeros(len(mesh.faces), dtype=bool)
    n_keep = 0
    for i in range(labels.max() + 1):
        idx = np.where(labels == i)[0]
        v = mesh.vertices[np.unique(mesh.faces[idx])]
        if np.linalg.norm(v.max(0) - v.min(0)) >= MIN_BODY_DIAG:
            keep[idx] = True
            n_keep += 1
    big = mesh.submesh([np.where(keep)[0]], append=True)
    ratio = max(1.0 - float(visual_target) / len(big.faces), 0.0)
    vt, ft = fast_simplification.simplify(
        big.vertices.astype('float32'), big.faces.astype('int32'), ratio)
    vis = trimesh.Trimesh(vertices=vt, faces=ft, process=False)
    vis.export('%s/rov_kki2026_visual.stl' % out_dir)
    mesh.convex_hull.export('%s/rov_kki2026_collision.stl' % out_dir)
    return n_keep, len(vis.faces), len(mesh.convex_hull.faces)


def main(argv=None):
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--stl', required=True, help='path STL gabungan ROV')
    ap.add_argument('--mass', type=float, default=33.6,
                    help='massa total (kg) untuk menskalakan inersia')
    ap.add_argument('--export-meshes', action='store_true')
    ap.add_argument('--out-dir', default='.', help='tujuan mesh turunan')
    ap.add_argument('--verbose', action='store_true',
                    help='tampilkan body besar (bukti orientasi)')
    args = ap.parse_args(sys.argv[1:] if argv is None else argv)

    mesh = load_mesh(args.stl)
    print('mesh: %d face, %d vertex, bbox %s mm'
          % (len(mesh.faces), len(mesh.vertices), np.round(mesh.extents, 1)))

    labels = component_labels(mesh)
    bods = bodies(mesh, labels)
    print('connected component: %d (>=100 face: %d)' % (labels.max() + 1, len(bods)))

    com = mesh.centroid
    offset = -np.array([-com[1], com[0], com[2]]) / 1000.0
    print('\n--- transform mesh -> base_link ---')
    print('  rpy  = 0 0 1.5708      (Rz +90 deg)')
    print('  xyz  = %.5f %.5f %.5f  (m, agar CoM jatuh di origin)'
          % (offset[0], offset[1], offset[2]))

    if args.verbose:
        print('\n--- body terbesar (mm, koordinat mesh) ---')
        for b in sorted(bods, key=lambda x: -x['nf'])[:12]:
            print('  nf=%7d c=[%7.1f %7.1f %7.1f] ext=[%6.1f %6.1f %6.1f]'
                  % (b['nf'], *b['c'], *b['e']))

    thr = find_thrusters(bods)
    role = {0: 'sway  (y)', 1: 'surge (x)', 2: 'heave (z)'}
    print('\n--- thruster terdeteksi: %d ---' % len(thr))
    for t in sorted(thr, key=lambda x: (x['axis'], -x['c'][0])):
        p = mesh_to_base(t['c'], com)
        ax = rotate_axis(t['axis'])
        print('  %-11s xyz="%.4f %.4f %.4f"  axis="%.0f %.0f %.0f"  duct=%s mm'
              % (role[t['axis']], p[0], p[1], p[2], *ax, np.round(t['duct'], 1)))
    if len(thr) != 6:
        print('  PERINGATAN: seharusnya 6 thruster; periksa ambang deteksi.')

    tensor = inertia_per_kg(mesh)
    print('\n--- inersia per kg di base_link (kg*m^2 / kg) ---')
    print('  ixx=%.8f iyy=%.8f izz=%.8f' % (tensor[0, 0], tensor[1, 1], tensor[2, 2]))
    print('  ixy=%.8f ixz=%.8f iyz=%.8f' % (tensor[0, 1], tensor[0, 2], tensor[1, 2]))
    print('  -> pada massa %.1f kg: ixx=%.6f iyy=%.6f izz=%.6f'
          % (args.mass, tensor[0, 0] * args.mass,
             tensor[1, 1] * args.mass, tensor[2, 2] * args.mass))

    solid, cavity, total = displaced_volume(mesh)
    print('\n--- apung ---')
    print('  volume solid mesh   = %.6f m^3' % solid)
    print('  rongga tabung       = %.6f m^3 (estimasi silinder OD130 x 210)' % cavity)
    print('  volume tergeser ~   = %.6f m^3 -> apung %.2f kg (air tawar)'
          % (total, total * 1000.0))
    print('  convex hull         = %.6f m^3 -> apung %.2f kg (TERLALU BESAR:'
          ' hull mengisi rongga rangka)'
          % (mesh.convex_hull.volume / 1e9, mesh.convex_hull.volume / 1e6))
    print('  massa utk NETRAL    = %.2f kg' % (total * 1000.0))
    if args.mass > total * 1000.0:
        print('  massa %.1f kg -> NEGATIF %.2f kg (tenggelam)'
              % (args.mass, args.mass - total * 1000.0))

    if args.export_meshes:
        nk, nv, nc = export_meshes(mesh, labels, args.out_dir)
        print('\n--- mesh turunan ---')
        print('  visual   : %d body dipertahankan, %d face' % (nk, nv))
        print('  collision: convex hull, %d face' % nc)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
