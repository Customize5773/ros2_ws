#!/bin/bash
# P0-2.2b run battery: N=6 APPROACH_QR runs (5 random-spawn + 1 deterministic)
# for reduce_approach_qr.py. Loops run_approach_qr_smoke.sh sequentially (one
# Gazebo server at a time, same constraint as P0-1e). No auto-retry on gate
# failure — a failed/contaminated run is left for the reducer to flag
# INCONCLUSIVE; re-running it is a manual decision after reviewing why.
#
# usage:  P0_DATA_DIR=/somewhere bash tools/p0-experiments/run_approach_qr_battery.sh
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SP="${P0_DATA_DIR:-$PWD}"
mkdir -p "$SP"

echo "=== P0-2.2b battery start: 6 runs into $SP ==="

bash "$HERE/run_approach_qr_smoke.sh" Q1 rov_random_spawn:=true
bash "$HERE/run_approach_qr_smoke.sh" Q2 rov_random_spawn:=true
bash "$HERE/run_approach_qr_smoke.sh" Q3 rov_random_spawn:=true
bash "$HERE/run_approach_qr_smoke.sh" Q4 rov_random_spawn:=true
bash "$HERE/run_approach_qr_smoke.sh" Q5 rov_random_spawn:=true
bash "$HERE/run_approach_qr_smoke.sh" Q6 rov_random_spawn:=false rov_x:=0.0 rov_y:=0.0 rov_z:=-0.5

echo "=== P0-2.2b battery done ==="
for t in Q1 Q2 Q3 Q4 Q5 Q6; do
    gate=$(cat "$SP/$t.gate.txt" 2>/dev/null || echo "MISSING")
    rows=$(wc -l < "$SP/$t.csv" 2>/dev/null || echo 0)
    echo "  $t: gate=$gate rows=$rows"
done
