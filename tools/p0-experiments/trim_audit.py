#!/usr/bin/env python3
"""P0-1c.1 static trim audit — mass/buoyancy inventory of the ROV model.

Answers, without running a simulator: where are the system CoG and CoB, how far
apart are they, and what passive trim angle does that imply?

    tan(theta_trim) = dx(CoB-CoG) / dz(CoB-CoG)

This is the script that located the P0-1 root cause: the gripper links carried
<collision> geometry, which the gz Buoyancy plugin counts as displaced volume.

Values are transcribed from src/hydroships_description/urdf/hydroships.urdf.xacro
and src/hydroships_description/config/rov_params.yaml. Keep them in sync if the
model changes — and re-run this before trusting any buoyancy_ff calibration.

usage: python3 tools/p0-experiments/trim_audit.py
"""
import numpy as np

G = 9.81
RHO = 1000.0

# (name, mass_kg, r_inertial in base_link frame [m])
MASSES = [
    ('base_link',             8.30, (0.0,     0.0,      0.0)),
    ('thruster_1 (T200-E v)', 0.05, (-0.0275, -0.1234,  0.0142)),
    ('thruster_2 (T200-F v)', 0.05, (-0.0290,  0.1228,  0.0148)),
    ('thruster_3 (T100-C s)', 0.05, (0.1298,  -0.1371,  0.0336)),
    ('thruster_4 (T100-A s)', 0.05, (0.1296,   0.1371,  0.0374)),
    ('thruster_5 (T200-B y)', 0.05, (-0.0455, -0.0003, -0.0994)),
    ('thruster_6 (T100-D v)', 0.05, (-0.1364,  0.0003,  0.0403)),
    ('gripper_base',          0.08, (0.18,     0.0,     0.0)),
    ('gripper_finger_left',   0.01, (0.26,    -0.025,   0.0)),
    ('gripper_finger_right',  0.01, (0.26,     0.025,   0.0)),
    # imu_link / depth_link / camera_*_link carry no <inertial> and no <collision>
]

# (name, box size [m], collision origin in base_link frame [m])
# Only links with <collision> displace water. Thrusters never had one; the
# gripper links had one until commit 9219735 and that was the P0-1 root cause.
COLLISIONS = [
    ('base_link (buoyancy_collision @ cob offset)',
     (0.219, 0.219, 0.182), (0.00237, 0.0, 0.02)),
]


def main():
    print('=' * 72)
    print('1. MASS INVENTORY (base_link frame, REP-103 x fwd / y left / z up)')
    print('=' * 72)
    print('%-26s %8s %10s %10s %10s' % ('component', 'mass kg', 'x', 'y', 'z'))
    m_tot, mom = 0.0, np.zeros(3)
    for n, m, r in MASSES:
        r = np.array(r)
        m_tot += m
        mom += m * r
        print('%-26s %8.3f %10.4f %10.4f %10.4f' % (n, m, *r))
    cog = mom / m_tot
    print('-' * 72)
    print('%-26s %8.3f' % ('TOTAL MASS', m_tot))
    print('%-26s %8s %10.5f %10.5f %10.5f' % ('SYSTEM CoG', '', *cog))
    print('%-26s %8.2f N' % ('WEIGHT', m_tot * G))
    print("note: rov_params.yaml `cog` is base_link's inertial origin only,")
    print('      NOT the system CoG computed above.')

    print()
    print('=' * 72)
    print('2. BUOYANCY INVENTORY (gz Buoyancy uses <collision> volume)')
    print('=' * 72)
    print('%-46s %10s %9s' % ('component', 'vol m^3', 'F_up N'))
    v_tot, bmom = 0.0, np.zeros(3)
    for n, s, r in COLLISIONS:
        v = s[0] * s[1] * s[2]
        r = np.array(r)
        v_tot += v
        bmom += v * r
        print('%-46s %10.6f %9.3f' % (n, v, RHO * v * G))
    cob = bmom / v_tot
    f_buoy = RHO * v_tot * G
    print('-' * 72)
    print('%-46s %10.6f %9.3f' % ('TOTAL', v_tot, f_buoy))
    print('SYSTEM CoB = (%.5f, %.5f, %.5f)' % (*cob,))
    print('net buoyancy = %+.2f N   (design target: +0.28 N, near-neutral)'
          % (f_buoy - m_tot * G))

    print()
    print('=' * 72)
    print('3. TRIM MOMENT, level attitude, fully submerged (about system CoG)')
    print('=' * 72)
    r_b = cob - cog
    M = np.cross(r_b, np.array([0.0, 0.0, f_buoy]))
    print('r_CoB - r_CoG    = (%+.5f, %+.5f, %+.5f) m' % (*r_b,))
    print('M_trim about CoG = (Mx %+.4f, My %+.4f, Mz %+.4f) N*m' % (*M,))
    print('righting arm dz  = %+.5f m  -> max righting %.4f N*m'
          % (r_b[2], f_buoy * r_b[2]))
    if abs(r_b[2]) > 1e-9:
        print('passive trim: tan(theta) = dx/dz = %+.4f -> theta = %+.1f deg'
              % (r_b[0] / r_b[2], np.degrees(np.arctan2(r_b[0], r_b[2]))))
    print()
    print('reference: before commit 9219735 the gripper collisions gave')
    print('  net buoyancy +6.92 N, CoB_x +13.6 mm, passive trim 31.5 deg bow-up')
    print('  (measured runtime: -29.6 / -44.1 deg). See docs/P0-1-BASELINE.md.')


if __name__ == '__main__':
    main()
