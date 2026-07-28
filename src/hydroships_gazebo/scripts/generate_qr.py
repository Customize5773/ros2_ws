#!/usr/bin/env python3
"""Generate tekstur QR payload HYDROships (A/B/C/D) untuk simulasi.

Menghasilkan media/qr_A.png .. qr_D.png berisi HURUF sisi tunggal "A".."D"
(qr_detector.parse_wall mengekstrak huruf A/B/C/D).

Isi QR sengaja HANYA huruf sisi (bukan "HYDROSHIP-M5-A") — string pendek =
QR versi rendah (21x21) = modul besar = jauh lebih tahan degradasi render
kamera sim (fix [RESOLVED] QR detection, lihat docs/CHANGELOG.md). Ini juga
menyamakan konvensi dgn qr_A.png yang sudah ter-commit & terbukti terbaca di
sim. parse_wall tetap menerima string panjang, jadi payload NYATA boleh pakai
label penuh tanpa mengubah kontrak.

Kenapa skrip (bukan aset commit langsung): bitmap QR = data biner, mudah usang
bila isi/format berubah. Regen deterministik dari sini agar reproducible.

Prasyarat:  pip install "qrcode[pil]"
Jalankan :  python3 src/hydroships_gazebo/scripts/generate_qr.py
            python3 .../generate_qr.py --size 768 --border 4 --out <dir>

Catatan render sim: material QR di world memakai <emissive_map> (self-lit) agar
terbaca di kamera sensor Fortress headless (albedo saja bisa render hitam tanpa
IBL). QR dibuat HITAM di atas PUTIH + quiet-zone (border) — jangan set border=0.
"""

import argparse
import os
import sys

# Isi QR = huruf sisi tunggal (QR versi rendah/modul besar -> tahan render sim).
CONTENT = {'A': 'A', 'B': 'B', 'C': 'C', 'D': 'D'}


def main():
    default_out = os.path.normpath(
        os.path.join(os.path.dirname(__file__), '..', 'media'))
    ap = argparse.ArgumentParser(description='Generate QR payload A/B/C/D.')
    ap.add_argument('--out', default=default_out, help='folder output (default: ../media)')
    ap.add_argument('--size', type=int, default=512, help='sisi PNG (px), default 512')
    ap.add_argument('--border', type=int, default=4, help='quiet-zone (modul), default 4')
    ap.add_argument('--letters', default='ABCD', help='huruf yg dibuat, mis. "A" atau "ABCD"')
    args = ap.parse_args()

    try:
        import qrcode
    except ImportError:
        sys.exit('ERROR: butuh pustaka qrcode. Pasang: pip install "qrcode[pil]"')

    os.makedirs(args.out, exist_ok=True)
    for letter in args.letters.upper():
        if letter not in CONTENT:
            print(f'lewati huruf tak dikenal: {letter}')
            continue
        qr = qrcode.QRCode(
            error_correction=qrcode.constants.ERROR_CORRECT_M,
            border=args.border,
        )
        qr.add_data(CONTENT[letter])
        qr.make(fit=True)
        img = qr.make_image(fill_color='black', back_color='white').convert('RGB')
        img = img.resize((args.size, args.size), resample=0)  # NEAREST -> modul tajam
        path = os.path.join(args.out, f'qr_{letter}.png')
        img.save(path)
        print(f'tulis {path}  ("{CONTENT[letter]}", {args.size}x{args.size})')


if __name__ == '__main__':
    main()
