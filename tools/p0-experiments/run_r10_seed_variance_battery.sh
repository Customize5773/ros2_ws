#!/bin/bash
# R-10 seed-variance battery: repeat ONE seed N times with identical params
# to quantify run-to-run alt_gap variance (CHANGELOG 2026-08-16 caveat —
# spawn_seed only seeds spawn pose, not physics/solver timing).
#
# usage: P0_DATA_DIR=/somewhere N=15 bash run_r10_seed_variance_battery.sh [seed] [tol] [duration]
set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SP="${P0_DATA_DIR:-$PWD}"
mkdir -p "$SP"

N="${N:-15}"
SEED="${1:-3001}"
TOL="${2:-0.02}"
DURATION="${3:-60}"

for i in $(seq 1 "$N"); do
    bash "$HERE/run_r10_trajectory.sh" "R10-var-${SEED}-run${i}" "$TOL" "$SEED" "$DURATION"
done

echo
echo "=== Summary: DESCEND window metrics (alt_gap distribution over $N repeats, seed=$SEED tol=$TOL) ==="
python3 "$HERE/reduce_r10_trajectory.py" "$SP"
