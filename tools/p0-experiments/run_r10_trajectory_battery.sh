#!/bin/bash
# R-10 trajectory battery: run seeds 3001-3006 × 2 tol values (0.06 old vs 0.02 new)
# plus the inconclusive 3003-old re-run.
#
# usage: P0_DATA_DIR=/somewhere bash run_r10_trajectory_battery.sh
set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SP="${P0_DATA_DIR:-$PWD}"
mkdir -p "$SP"

SEEDS=(3001 3002 3003 3004 3005 3006)
TOL_OLD=0.06
TOL_NEW=0.02

for seed in "${SEEDS[@]}"; do
    bash "$HERE/run_r10_trajectory.sh" "R10-before-${seed}" "$TOL_OLD" "$seed" 150
    bash "$HERE/run_r10_trajectory.sh" "R10-after-${seed}"  "$TOL_NEW" "$seed" 150
done

echo
echo "=== Summary: DESCEND window metrics ==="
python3 "$HERE/reduce_r10_trajectory.py" "$SP"
