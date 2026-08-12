#!/usr/bin/env python3
"""P0-2.7 SPEC -- offline screening for failure-prone rov spawn seeds.

DESIGN ONLY, runs no simulator. Replicates the EXACT RNG call sequence of
_rov_spawn_pose() (src/hydroships_gazebo/launch/sim.launch.py:88-112) --
along = rng.uniform(-lim, lim); wall = rng.choice(('A','B','C','D'));
yaw = _WALL_INWARD_YAW[wall] + rng.uniform(-_YAW_JITTER, _YAW_JITTER) --
using the same random.Random(seed) construction as _spawn_rng()
(sim.launch.py:115-124), so a seed screened here reproduces bit-for-bit
when passed as spawn_seed:=<seed> rov_random_spawn:=true to sim.launch.py.

Selection rule (docs/P0-2-7-FAILURE-BATTERY-SPEC.md):
  |along| >= 0.85 * lim        -- spawn in the corner-adjacent 15% of the
                                   wall span (both walls meeting at that
                                   corner put the QR payload closer to a
                                   camera-frame edge).
  |yaw_offset| >= 0.85 * _YAW_JITTER  -- near-maximal in-plane rotation,
                                   targeting the AABB-inflation/rotation
                                   mechanism already confirmed (P0-2.3) to
                                   degrade corner-only quality.
Both thresholds and both constants are copied from sim.launch.py, not
invented here.

usage: select_failure_seeds.py [n_seeds_to_scan] [along_frac] [yaw_frac]
  (defaults: scan seeds 0..999, along_frac=0.85, yaw_frac=0.85)
Prints qualifying seeds with (wall, along, yaw_offset) for
run_qr_failure_battery.sh to consume. Does not run anything else.
"""
import random
import sys

# --- copied constants (sim.launch.py:85,101-102) -- keep in sync if those change ---
_YAW_JITTER = 0.35
ARENA_HALF = 2.55   # rov_arena_half default
WALL_MARGIN = 0.5    # rov_wall_margin default
LIM = ARENA_HALF - WALL_MARGIN  # 2.05


def spawn_pose_for_seed(seed):
    """Exact replica of _rov_spawn_pose()'s RNG call order for
    rov_random_spawn=true (sim.launch.py:96-112), returns (wall, along,
    yaw_offset) -- yaw_offset is the rng.uniform(-_YAW_JITTER,_YAW_JITTER)
    draw itself, not the absolute yaw (absolute yaw also depends on
    _WALL_INWARD_YAW[wall], irrelevant to this screening rule)."""
    rng = random.Random(seed)
    along = rng.uniform(-LIM, LIM)
    wall = rng.choice(('A', 'B', 'C', 'D'))
    yaw_offset = rng.uniform(-_YAW_JITTER, _YAW_JITTER)
    return wall, along, yaw_offset


def is_failure_prone(wall, along, yaw_offset, along_frac, yaw_frac):
    return abs(along) >= along_frac * LIM and abs(yaw_offset) >= yaw_frac * _YAW_JITTER


def main():
    n_scan = int(sys.argv[1]) if len(sys.argv) > 1 else 1000
    along_frac = float(sys.argv[2]) if len(sys.argv) > 2 else 0.85
    yaw_frac = float(sys.argv[3]) if len(sys.argv) > 3 else 0.85

    print('# P0-2.7 failure-prone seed screening (design only, no simulator run)')
    print('# lim=%.3f (arena_half=%.2f - wall_margin=%.2f), yaw_jitter=%.3f'
          % (LIM, ARENA_HALF, WALL_MARGIN, _YAW_JITTER))
    print('# rule: |along| >= %.2f*lim (%.3f)  AND  |yaw_offset| >= %.2f*jitter (%.3f)'
          % (along_frac, along_frac * LIM, yaw_frac, yaw_frac * _YAW_JITTER))
    print('# seed,wall,along,yaw_offset')

    qualifying = []
    for seed in range(n_scan):
        wall, along, yaw_offset = spawn_pose_for_seed(seed)
        if is_failure_prone(wall, along, yaw_offset, along_frac, yaw_frac):
            qualifying.append(seed)
            print('%d,%s,%.4f,%.4f' % (seed, wall, along, yaw_offset))

    print('# scanned %d seeds, %d qualify (%.1f%%)'
          % (n_scan, len(qualifying), 100.0 * len(qualifying) / n_scan))
    if qualifying:
        print('# seed pool for run_qr_failure_battery.sh: %s'
              % ' '.join(str(s) for s in qualifying))


if __name__ == '__main__':
    main()
